#!/usr/bin/env python3
"""
RotorPy backend: flies a simulated quadrotor from PHLAI voice commands, in real time, and draws
it (plus a glowing pen trail) in a live matplotlib 3D window. Replaces Gazebo + ros_gz_bridge +
drone_adapter.py + gz_trail.py, and needs no ROS.

    voice app -> output/latest_command.json -> rotorpy_sim.py (RotorPy physics + SE3 controller)

How it flies: each command is the same timed, body-frame velocity schedule the Gazebo adapter
used (see command_schedule.py). Instead of sending raw velocities, the sim integrates them into
a moving position/yaw setpoint and lets RotorPy's geometric SE3 controller track it. So the
drone holds position when a command ends (no drift), and drawings close up accurately.

Command sources (pick one):
  default        watch output/latest_command.json (written by app.py on every utterance)
  --ros          subscribe to /phlai/voice_command (std_msgs/String JSON), like the old adapter
  --demo a,b,c   play named commands from commands.py back to back (no voice app needed)

Run it with the RotorPy venv:  .venv-rotorpy/bin/python3 rotorpy_sim.py
Headless check (no window, faster than real time, saves a picture):
    .venv-rotorpy/bin/python3 rotorpy_sim.py --headless --demo takeoff,spell_phlai --save run.png
"""
import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import command_schedule
import config

PHYSICS_DT = 0.01          # s; RotorPy integrates one step of this length per controller update
TRAIL_SPACING = 0.05       # m between trail dots (same as the Gazebo trail)
PATH_POINTS = 4000         # how much of the faint flight path to keep
LANDED_Z = 0.03            # below this (m) with a ground-level setpoint the motors idle


# ============================== the simulation ==============================

class RotorPySim:
    """Real-time RotorPy quadrotor driven by command_schedule sequences. Thread-safe snapshot()."""

    def __init__(self, vehicle="hummingbird", log=lambda text: print(text, flush=True)):
        from rotorpy.controllers.quadrotor_control import SE3Control
        from rotorpy.vehicles.multirotor import Multirotor
        if vehicle == "crazyflie":
            from rotorpy.vehicles.crazyflie_params import quad_params
        else:
            from rotorpy.vehicles.hummingbird_params import quad_params

        self.log = log
        self.params = quad_params
        self.vehicle = Multirotor(quad_params, initial_state=self._ground_state(quad_params),
                                  control_abstraction="cmd_motor_speeds",
                                  aero=True, enable_ground=True)
        self.controller = SE3Control(quad_params)
        self.state = self._ground_state(quad_params)

        self.t = 0.0                               # sim time, s
        self.ref_pos = np.zeros(3)                 # setpoint the controller tracks (world frame)
        self.ref_yaw = 0.0
        self.sequence, self.cmd_t0, self.cmd_name = [], 0.0, None
        self.pen = False

        self.commands = queue.Queue()
        self._lock = threading.Lock()
        self.path = []                             # recent positions (faint line)
        self.trail = []                            # pen dots; None = break between strokes
        self._last_dot = None

    @staticmethod
    def _ground_state(quad_params):
        return {"x": np.zeros(3), "v": np.zeros(3), "q": np.array([0.0, 0.0, 0.0, 1.0]),
                "w": np.zeros(3), "wind": np.zeros(3),
                "rotor_speeds": np.full(quad_params["num_rotors"], quad_params["rotor_speed_min"],
                                        dtype=float)}

    # ---------- commands ----------
    def submit(self, message):
        """Queue an accepted command message (dict with 'command' and 'params'). Any thread."""
        self.commands.put(message)

    def _start_command(self, message):
        params = message.get("params") or {}
        try:
            sequence = command_schedule.parse(params)
        except (ValueError, TypeError, AttributeError) as exc:
            self.log(f"Ignoring bad command {message.get('command')!r}: {exc}")
            return
        if params.get("clear_trail"):
            with self._lock:
                self.trail.clear()
            self._last_dot = None
        self.sequence, self.cmd_t0, self.cmd_name = sequence, self.t, message.get("command")
        total = command_schedule.total_duration(sequence)
        self.log(f"{self.cmd_name}: {len(sequence)} step(s), "
                 f"total {'until next command' if total is None else f'{total:.1f} s'}")

    def busy(self):
        """True while a timed command is still running (or one is waiting in the queue)."""
        total = command_schedule.total_duration(self.sequence)
        running = total is not None and self.t - self.cmd_t0 < total
        return running or not self.commands.empty()

    # ---------- physics ----------
    def step(self, dt=PHYSICS_DT):
        while True:
            try:
                self._start_command(self.commands.get_nowait())
            except queue.Empty:
                break

        cmd = command_schedule.step_at(self.sequence, self.t - self.cmd_t0)
        vx, vy, vz = cmd.linear
        c, s = np.cos(self.ref_yaw), np.sin(self.ref_yaw)
        v_world = np.array([c * vx - s * vy, s * vx + c * vy, vz])   # body -> world (yaw only)

        self.ref_pos = self.ref_pos + v_world * dt
        self.ref_yaw += cmd.angular_z * dt
        if self.ref_pos[2] <= 0.0:                 # never ask it to fly into the ground
            self.ref_pos[2] = 0.0
            v_world[2] = max(v_world[2], 0.0)

        landed = self.state["x"][2] < LANDED_Z and self.ref_pos[2] <= 0.0
        if landed:
            # Sitting on the ground with nothing asking it up: idle the motors and keep the
            # setpoint under the drone so it doesn't skid toward an old target on takeoff.
            self.ref_pos[:2] = self.state["x"][:2]
            control = {"cmd_motor_speeds": np.full(self.params["num_rotors"],
                                                   self.params["rotor_speed_min"], dtype=float)}
        else:
            flat = {"x": self.ref_pos.copy(), "x_dot": v_world, "x_ddot": np.zeros(3),
                    "x_dddot": np.zeros(3), "x_ddddot": np.zeros(3),
                    "yaw": self.ref_yaw, "yaw_dot": cmd.angular_z}
            control = self.controller.update(self.t, self.state, flat)

        self.state = self.vehicle.step(self.state, control, dt)
        self.t += dt
        self._record(cmd.pen)

    def _record(self, pen):
        pos = self.state["x"].copy()
        with self._lock:
            self.path.append(pos)
            if len(self.path) > PATH_POINTS:
                del self.path[: len(self.path) - PATH_POINTS]
            if pen and not self.pen:               # pen down: start a fresh stroke
                if self.trail and self.trail[-1] is not None:
                    self.trail.append(None)
                self._last_dot = None
            if pen and (self._last_dot is None or
                        np.linalg.norm(pos - self._last_dot) >= TRAIL_SPACING):
                self.trail.append(pos)
                self._last_dot = pos
            self.pen = pen

    def snapshot(self):
        with self._lock:
            return {"t": self.t, "x": self.state["x"].copy(), "q": self.state["q"].copy(),
                    "ref": self.ref_pos.copy(), "path": list(self.path),
                    "trail": list(self.trail), "command": self.cmd_name, "pen": self.pen}

    # ---------- loops ----------
    def run_realtime(self, stop):
        """Step in real time until `stop` is set. Falls behind gracefully if the PC is slow."""
        wall0, sim0, warned = time.monotonic(), self.t, False
        while not stop.is_set():
            self.step()
            ahead = (self.t - sim0) - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
            elif ahead < -0.5:
                if not warned:
                    self.log("Simulation can't keep up with real time; it will run slow-motion.")
                    warned = True
                wall0, sim0 = time.monotonic(), self.t


# ============================== command sources ==============================

def watch_file(sim, stop, path=config.LATEST_JSON, poll=0.05):
    """Feed every NEW accepted command written to latest_command.json (ignores the one already there)."""
    def stamp():
        try:
            return path.stat().st_mtime_ns
        except FileNotFoundError:
            return None

    last = stamp()
    sim.log(f"Watching {path} for voice commands")
    while not stop.wait(poll):
        now = stamp()
        if now is None or now == last:
            continue
        last = now
        try:
            message = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            sim.log(f"Could not read {path.name}: {exc}")
            continue
        if message.get("status") == "accepted":
            sim.submit(message)


def subscribe_ros(sim, stop, topic=config.ROS_COMMAND_TOPIC):
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("phlai_rotorpy_sim")

    def on_msg(msg):
        try:
            sim.submit(json.loads(msg.data))
        except ValueError as exc:
            sim.log(f"Ignoring bad ROS message: {exc}")

    node.create_subscription(String, topic, on_msg, 10)
    sim.log(f"Subscribed to ROS 2 topic {topic}")
    while not stop.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def demo_messages(names):
    from commands import COMMANDS
    by_name = {c.name: c for c in COMMANDS}
    unknown = [n for n in names if n not in by_name]
    if unknown:
        sys.exit(f"Unknown command(s) {unknown}. Known: {', '.join(sorted(by_name))}")
    return [{"status": "accepted", "command": n, "params": by_name[n].params} for n in names]


def play_demo(sim, stop, messages, settle=1.0):
    """Submit each message once the previous one has finished (plus `settle` s of hover)."""
    for message in messages:
        while sim.busy() and not stop.is_set():
            time.sleep(0.05)
        if stop.wait(settle):
            return
        sim.submit(message)


# ============================== viewer ==============================

def _box(ax, points, ref, margin=1.0, minimum=4.0):
    pts = np.vstack(points + [ref, np.zeros(3)])
    lo, hi = pts.min(axis=0) - margin, pts.max(axis=0) + margin
    lo[2] = 0.0
    span = max(minimum, float((hi - lo).max()))
    mid = (lo + hi) / 2
    ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
    ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
    ax.set_zlim(0.0, span)


def _trail_xyz(trail):
    if not trail:
        return [], [], []
    pts = np.array([p if p is not None else (np.nan,) * 3 for p in trail])
    return pts[:, 0], pts[:, 1], pts[:, 2]


class Viewer:
    def __init__(self, sim, arm=0.17):
        import matplotlib.pyplot as plt
        self.plt, self.sim, self.arm = plt, sim, arm
        self.fig = plt.figure(f"{config.APP_NAME} - RotorPy", figsize=(8, 7))
        self.ax = self.fig.add_subplot(projection="3d")
        ax = self.ax
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)")
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=15, azim=200)            # from behind the start heading, like the letters
        self.path_line, = ax.plot([], [], [], lw=0.8, color="0.6", alpha=0.6)
        self.trail_line, = ax.plot([], [], [], ls="none", marker="o", ms=4,
                                   color=(0.1, 0.8, 1.0), markeredgecolor=(0.0, 0.5, 0.8))
        self.arm_lines = [ax.plot([], [], [], lw=3, color="0.15")[0] for _ in range(2)]
        self.nose, = ax.plot([], [], [], lw=3, color="tab:red")
        self.ref_dot, = ax.plot([], [], [], ls="none", marker="+", ms=8, color="tab:orange")
        self.title = ax.set_title("")

    def draw(self, snap):
        x, R = snap["x"], Rotation.from_quat(snap["q"]).as_matrix()
        a = self.arm / np.sqrt(2)
        for line, (u, v) in zip(self.arm_lines, (((a, a), (-a, -a)), ((a, -a), (-a, a)))):
            p1, p2 = x + R @ (*u, 0), x + R @ (*v, 0)
            line.set_data_3d(*zip(p1, p2))
        self.nose.set_data_3d(*zip(x, x + R @ (self.arm * 1.4, 0, 0)))
        self.ref_dot.set_data_3d([snap["ref"][0]], [snap["ref"][1]], [snap["ref"][2]])
        if snap["path"]:
            self.path_line.set_data_3d(*np.array(snap["path"]).T)
        self.trail_line.set_data_3d(*_trail_xyz(snap["trail"]))
        _box(self.ax, snap["path"][-1:] + [p for p in snap["trail"] if p is not None], snap["ref"])
        pen = "  pen DOWN" if snap["pen"] else ""
        self.title.set_text(f"{snap['command'] or 'waiting for a command'}{pen}\n"
                            f"t={snap['t']:5.1f}s  pos=({x[0]:+.2f}, {x[1]:+.2f}, {x[2]:+.2f}) m")

    def run(self, stop, fps=20):
        from matplotlib.animation import FuncAnimation

        def update(_):
            self.draw(self.sim.snapshot())
            return []

        self._anim = FuncAnimation(self.fig, update, interval=1000 / fps, cache_frame_data=False)
        self.fig.canvas.mpl_connect("close_event", lambda _e: stop.set())
        self.plt.show()
        stop.set()


# ============================== main ==============================

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--ros", action="store_true", help="read commands from the ROS 2 topic")
    src.add_argument("--demo", metavar="CMD,CMD", help="play named commands from commands.py")
    ap.add_argument("--file", type=Path, default=config.LATEST_JSON,
                    help="command file to watch (default: %(default)s)")
    ap.add_argument("--vehicle", choices=("hummingbird", "crazyflie"), default="hummingbird")
    ap.add_argument("--headless", action="store_true",
                    help="no window, run faster than real time (needs --demo)")
    ap.add_argument("--save", type=Path, help="save a picture of the final state to this PNG")
    args = ap.parse_args(argv)
    if args.headless and not args.demo:
        ap.error("--headless needs --demo")

    sim = RotorPySim(vehicle=args.vehicle)
    stop = threading.Event()

    if args.headless:
        import matplotlib
        matplotlib.use("Agg")
        messages = demo_messages(args.demo.split(","))
        wall0 = time.monotonic()
        for message in messages:
            for _ in range(int(1.0 / PHYSICS_DT)):            # 1 s of hover between commands
                sim.step()
            sim.submit(message)
            sim.step()
            while sim.busy():
                sim.step()
        snap = sim.snapshot()
        x = snap["x"]
        sim.log(f"Done: {snap['t']:.1f} s simulated in {time.monotonic() - wall0:.1f} s wall; "
                f"final pos ({x[0]:+.2f}, {x[1]:+.2f}, {x[2]:+.2f}) m, "
                f"{sum(p is not None for p in snap['trail'])} trail dots")
        if args.save:
            viewer = Viewer(sim)
            viewer.draw(snap)
            viewer.fig.savefig(args.save, dpi=90)
            sim.log(f"Saved {args.save}")
        return

    if args.demo:
        source = threading.Thread(target=play_demo, args=(sim, stop, demo_messages(args.demo.split(","))))
    elif args.ros:
        source = threading.Thread(target=subscribe_ros, args=(sim, stop))
    else:
        source = threading.Thread(target=watch_file, args=(sim, stop, args.file))
    source.daemon = True
    source.start()
    threading.Thread(target=sim.run_realtime, args=(stop,), daemon=True).start()

    viewer = Viewer(sim)
    try:
        viewer.run(stop)
    except KeyboardInterrupt:
        stop.set()
    if args.save:
        viewer.draw(sim.snapshot())
        viewer.fig.savefig(args.save, dpi=90)


if __name__ == "__main__":
    main()
