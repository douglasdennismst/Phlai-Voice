#!/usr/bin/env python3
"""
RotorPy backend: flies a fleet of simulated quadrotors from PHLAI voice commands, in real time,
and draws them (each with its own coloured pen trail) in a live matplotlib 3D window. Replaces
Gazebo + ros_gz_bridge + drone_adapter.py + gz_trail.py, and needs no ROS.

    voice app -> output/latest_command.json -> rotorpy_sim.py (RotorPy physics + SE3 controller)

How it flies: each command is the same timed, body-frame velocity schedule the Gazebo adapter
used (see command_schedule.py). Instead of sending raw velocities, the sim integrates them into
a moving position/yaw setpoint and lets RotorPy's geometric SE3 controller track it. So a
drone holds position when a command ends (no drift), and drawings close up accurately.

Several drones: config.DRONE_COUNT drones start side by side, DRONE_SPACING apart (drone 1 on the
left as seen from behind). A command's "target" ("all" or a list like [1, 3], set by the matcher
from "drone one and three, ...") picks which drones fly it. Each drone is simulated on its own:
RotorPy has no drone-to-drone collisions or downwash, so they can pass through each other.

Command sources (pick one):
  default        watch output/latest_command.json (written by app.py on every utterance)
  --ros          subscribe to /phlai/voice_command (std_msgs/String JSON), like the old adapter
  --demo NAME    play a choreographed demo from demos.py, e.g. --demo showcase (no voice needed)
  --demo a,b,c   or your own list: takeoff,1:star,2:heart,wait,1+3:land  (see parse_demo)

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

PHYSICS_DT = 0.02          # s; one controller update + one RotorPy step (50 Hz)
INTEGRATOR = {"method": "RK23"}   # ~1.7x faster than RotorPy's default RK45 here, same flight to ~1 mm
TRAIL_SPACING = 0.05       # m between trail dots (same as the Gazebo trail)
PATH_POINTS = 4000         # how much of the faint flight path to keep, per drone
LANDED_Z = 0.03            # below this (m) with a ground-level setpoint the motors idle
COLORS = [(0.1, 0.8, 1.0), (1.0, 0.45, 0.1), (0.4, 0.9, 0.2), (0.85, 0.3, 0.9),
          (1.0, 0.85, 0.1), (1.0, 0.3, 0.45), (0.2, 0.5, 1.0), (0.6, 0.6, 0.6), (0.1, 0.7, 0.6)]


def _log(text):
    print(text, flush=True)


# ============================== one drone ==============================

class Drone:
    """One RotorPy quadrotor with its own controller, setpoint, current command and trail."""

    def __init__(self, number, start, quad_params):
        from rotorpy.controllers.quadrotor_control import SE3Control
        from rotorpy.vehicles.multirotor import Multirotor

        self.number = number                       # 1-based, as spoken
        self.start = np.array(start, dtype=float)
        self.color = COLORS[(number - 1) % len(COLORS)]
        self.params = quad_params
        self.state = self._ground_state(start)
        self.vehicle = Multirotor(quad_params, initial_state=self._ground_state(start),
                                  control_abstraction="cmd_motor_speeds",
                                  aero=True, enable_ground=True, integrator_kwargs=INTEGRATOR)
        self.controller = SE3Control(quad_params)

        self.ref_pos = np.array(start, dtype=float)    # setpoint the controller tracks (world)
        self.ref_yaw = 0.0
        self.sequence, self.cmd_t0, self.cmd_name = [], 0.0, None
        self.pen = False
        self.path = []                             # recent positions (faint line)
        self.trail = []                            # pen dots; None = break between strokes
        self._last_dot = None

    def _ground_state(self, start):
        p = self.params
        return {"x": np.array(start, dtype=float), "v": np.zeros(3),
                "q": np.array([0.0, 0.0, 0.0, 1.0]), "w": np.zeros(3), "wind": np.zeros(3),
                "rotor_speeds": np.full(p["num_rotors"], p["rotor_speed_min"], dtype=float)}

    def start_command(self, name, sequence, t, clear_trail):
        if clear_trail:
            self.trail.clear()
            self._last_dot = None
        self.sequence, self.cmd_t0, self.cmd_name = sequence, t, name

    def free_at(self):
        """Sim time when the current command ends (a hold command counts as ended at once)."""
        total = command_schedule.total_duration(self.sequence)
        return self.cmd_t0 + (total or 0.0)

    def busy(self, t):
        return t < self.free_at()

    def step(self, t, dt):
        cmd = command_schedule.step_at(self.sequence, t - self.cmd_t0)
        vx, vy, vz = cmd.linear
        c, s = np.cos(self.ref_yaw), np.sin(self.ref_yaw)
        v_world = np.array([c * vx - s * vy, s * vx + c * vy, vz])   # body -> world (yaw only)

        self.ref_pos = self.ref_pos + v_world * dt
        self.ref_yaw += cmd.angular_z * dt
        if self.ref_pos[2] <= 0.0:                 # never ask it to fly into the ground
            self.ref_pos[2] = 0.0
            v_world[2] = max(v_world[2], 0.0)

        if self.state["x"][2] < LANDED_Z and self.ref_pos[2] <= 0.0:
            # Sitting on the ground with nothing asking it up: idle the motors and keep the
            # setpoint under the drone so it doesn't skid toward an old target on takeoff.
            self.ref_pos[:2] = self.state["x"][:2]
            control = {"cmd_motor_speeds": np.full(self.params["num_rotors"],
                                                   self.params["rotor_speed_min"], dtype=float)}
        else:
            flat = {"x": self.ref_pos.copy(), "x_dot": v_world, "x_ddot": np.zeros(3),
                    "x_dddot": np.zeros(3), "x_ddddot": np.zeros(3),
                    "yaw": self.ref_yaw, "yaw_dot": cmd.angular_z}
            control = self.controller.update(t, self.state, flat)

        self.state = self.vehicle.step(self.state, control, dt)
        self._record(cmd.pen)

    def _record(self, pen):
        pos = self.state["x"].copy()
        self.path.append(pos)
        if len(self.path) > PATH_POINTS:
            del self.path[: len(self.path) - PATH_POINTS]
        if pen and not self.pen:                   # pen down: start a fresh stroke
            if self.trail and self.trail[-1] is not None:
                self.trail.append(None)
            self._last_dot = None
        if pen and (self._last_dot is None or
                    np.linalg.norm(pos - self._last_dot) >= TRAIL_SPACING):
            self.trail.append(pos)
            self._last_dot = pos
        self.pen = pen

    def snapshot(self):
        return {"number": self.number, "color": self.color, "x": self.state["x"].copy(),
                "q": self.state["q"].copy(), "ref": self.ref_pos.copy(), "path": list(self.path),
                "trail": list(self.trail), "command": self.cmd_name, "pen": self.pen}


# ============================== the fleet ==============================

class RotorPySim:
    """Real-time fleet of RotorPy drones driven by command messages. Thread-safe snapshot()."""

    def __init__(self, drones=None, spacing=None, vehicle="hummingbird", log=_log):
        if vehicle == "crazyflie":
            from rotorpy.vehicles.crazyflie_params import quad_params
        else:
            from rotorpy.vehicles.hummingbird_params import quad_params

        count = config.DRONE_COUNT if drones is None else drones
        spacing = config.DRONE_SPACING if spacing is None else spacing
        # Side by side along y, centred on 0. Drone 1 is on the left (+y) seen from behind.
        self.drones = [Drone(i + 1, ((count - 1) / 2 - i) * spacing * np.array([0.0, 1.0, 0.0]),
                             quad_params) for i in range(count)]
        self.log = log
        self.t = 0.0                               # sim time, s
        self.commands = queue.Queue()
        self.script = None                         # DemoScript, advanced inside step()
        self.caption = ""                          # shown above the 3D view (demo act names)
        self._lock = threading.Lock()

    # ---------- commands ----------
    def submit(self, message):
        """Queue an accepted command message (dict with 'command', 'params', 'target'). Any thread."""
        self.commands.put(message)

    def _targets(self, target):
        if target in (None, "all"):
            return list(self.drones)
        try:
            numbers = [int(n) for n in (target if isinstance(target, list) else [target])]
        except (TypeError, ValueError):
            self.log(f"Ignoring bad target {target!r}")
            return []
        missing = [n for n in numbers if not 1 <= n <= len(self.drones)]
        if missing:
            self.log(f"No drone {missing} (have 1-{len(self.drones)}); ignoring those")
        return [self.drones[n - 1] for n in numbers if 1 <= n <= len(self.drones)]

    def _start_command(self, message):
        params = message.get("params") or {}
        name = message.get("command")
        try:
            sequence = command_schedule.parse(params)
        except (ValueError, TypeError, AttributeError) as exc:
            self.log(f"Ignoring bad command {name!r}: {exc}")
            return
        drones = self._targets(message.get("target", "all"))
        if not drones:
            return
        with self._lock:
            for drone in drones:
                drone.start_command(name, sequence, self.t, bool(params.get("clear_trail")))
        total = command_schedule.total_duration(sequence)
        who = ("all drones" if len(drones) == len(self.drones) and len(self.drones) > 1
               else "drone " + ", ".join(str(d.number) for d in drones))
        self.log(f"{name} -> {who}: {len(sequence)} step(s), "
                 f"total {'until next command' if total is None else f'{total:.1f} s'}")

    def busy(self):
        """True while any drone is still running a timed command, one is queued, or a demo is playing."""
        return (any(d.busy(self.t) for d in self.drones) or not self.commands.empty()
                or (self.script is not None and not self.script.done))

    # ---------- physics ----------
    def step(self, dt=PHYSICS_DT):
        while True:
            try:
                self._start_command(self.commands.get_nowait())
            except queue.Empty:
                break
        if self.script is not None:
            self.script.update(self)
        with self._lock:
            for drone in self.drones:
                drone.step(self.t, dt)
            self.t += dt

    def snapshot(self):
        with self._lock:
            return {"t": self.t, "caption": self.caption,
                    "drones": [d.snapshot() for d in self.drones]}

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
                    self.log("Simulation can't keep up with real time; it will run slow-motion. "
                             "Try fewer drones (--drones).")
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


def parse_demo(spec):
    """A demo name from demos.py, or a comma list of items, -> list of script items.

    Items:  [drones:]command   run a command from commands.py ("takeoff", "2:star", "1+3:land");
                               it starts as soon as ITS drones are free, so different drones overlap
            wait               wait until every drone is free
            +N                 pause the script N seconds (drones already flying carry on)
            [drones:]home      fly back to the start spot at HOME_HEIGHT, facing forward again
                               (straightens drones out after spirals and orbits)
            # text             show text as the caption (act names)
    A demo in demos.py may also contain ready-made messages:
            {"target": [1], "command": "name", "params": {...}}
    """
    from commands import COMMANDS
    import demos
    items = demos.DEMOS.get(spec.strip())
    if items is None:
        items = [i.strip() for i in spec.split(",") if i.strip()]
    by_name = {c.name: c for c in COMMANDS}
    script = []
    for item in items:
        if isinstance(item, dict):
            script.append(("command", {"status": "accepted", "target": "all", **item}))
            continue
        if item.rpartition(":")[2] == "home":
            who = item.rpartition(":")[0]
            try:
                target = "all" if who in ("", "all") else [int(n) for n in who.split("+")]
            except ValueError:
                sys.exit(f"Bad drone list {who!r} in {item!r}")
            script.append(("home", target))
            continue
        if item == "wait":
            script.append(("wait",))
        elif item.startswith("#"):
            script.append(("caption", item[1:].strip()))
        elif item.startswith("+"):
            try:
                script.append(("pause", float(item[1:])))
            except ValueError:
                sys.exit(f"Bad pause {item!r}; use e.g. +2.5")
        else:
            who, _, name = item.rpartition(":")
            if name not in by_name:
                sys.exit(f"Unknown command or demo {name!r}. Demos: {', '.join(demos.DEMOS)}. "
                         f"Commands: {', '.join(sorted(by_name))}")
            try:
                target = "all" if who in ("", "all") else [int(n) for n in who.split("+")]
            except ValueError:
                sys.exit(f"Bad drone list {who!r} in {item!r}; use e.g. 2:forward or 1+3:land")
            script.append(("command", {"status": "accepted", "command": name, "target": target,
                                       "params": by_name[name].params}))
    return script


HOME_HEIGHT = 3.0          # m; same as after takeoff


def home_params(drone, speed=0.8, yaw_rate=0.8):
    """Steps that turn a drone back to facing +x, then fly it to its start spot at HOME_HEIGHT.
    Worked out from its setpoint, so it lands exactly on the spot it was asked to be at."""
    yaw = (drone.ref_yaw + np.pi) % (2 * np.pi) - np.pi       # -pi..pi, shortest way round
    steps = []
    if abs(yaw) > 1e-3:
        t = max(0.5, abs(yaw) / yaw_rate)
        steps.append({"angular_z": -yaw / t, "duration": t})
    delta = np.array([drone.start[0], drone.start[1], HOME_HEIGHT]) - drone.ref_pos
    dist = float(np.linalg.norm(delta))
    if dist > 1e-3:
        t = max(0.5, dist / speed)
        steps.append({"linear": list(delta / t), "duration": t})   # facing +x: body = world
    return {"steps": steps or [{"duration": 0.1}]}


class DemoScript:
    """Plays parsed demo items on sim time. RotorPySim.step() calls update() every step."""

    def __init__(self, items, settle=0.6):
        self.items, self.i, self.settle = items, 0, settle
        self.resume_at = 0.0                       # for +N pauses

    @property
    def done(self):
        return self.i >= len(self.items)

    def update(self, sim):
        while not self.done:
            item = self.items[self.i]
            kind = item[0]
            if sim.t < self.resume_at:
                return
            if kind == "pause":
                self.resume_at = sim.t + item[1]
            elif kind == "caption":
                sim.caption = item[1]
                sim.log(f"--- {item[1]} ---")
            elif kind == "wait":
                if any(sim.t < d.free_at() + self.settle for d in sim.drones):
                    return
            elif kind == "home":
                drones = sim._targets(item[1])
                if any(sim.t < d.free_at() + self.settle for d in drones):
                    return
                for d in drones:
                    sim._start_command({"command": "home", "target": [d.number],
                                        "params": home_params(d)})
            else:
                drones = sim._targets(item[1]["target"])
                if any(sim.t < d.free_at() + self.settle for d in drones):
                    return                         # its drones are still busy: try next step
                sim._start_command(item[1])
            self.i += 1


# ============================== viewer ==============================

def _box(ax, points, margin=1.0, minimum=4.0):
    pts = np.vstack(points + [np.zeros(3)])
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


class _DroneArtists:
    def __init__(self, ax, color, number):
        dark = tuple(c * 0.6 for c in color)
        self.path, = ax.plot([], [], [], lw=0.8, color=color, alpha=0.35)
        self.trail, = ax.plot([], [], [], ls="none", marker="o", ms=4, color=color,
                              markeredgecolor=dark)
        self.arms = [ax.plot([], [], [], lw=3, color="0.15")[0] for _ in range(2)]
        self.nose, = ax.plot([], [], [], lw=3, color=dark)
        self.ref, = ax.plot([], [], [], ls="none", marker="+", ms=8, color=dark)
        self.label = ax.text(0, 0, 0, str(number), color=dark, fontsize=11, fontweight="bold")


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
        self.artists = [_DroneArtists(ax, d.color, d.number) for d in sim.drones]
        self.title = ax.set_title("", fontsize=10)

    def draw(self, snap):
        a = self.arm / np.sqrt(2)
        points, lines = [], []
        for art, d in zip(self.artists, snap["drones"]):
            x, R = d["x"], Rotation.from_quat(d["q"]).as_matrix()
            for line, (u, v) in zip(art.arms, (((a, a), (-a, -a)), ((a, -a), (-a, a)))):
                line.set_data_3d(*zip(x + R @ (*u, 0), x + R @ (*v, 0)))
            art.nose.set_data_3d(*zip(x, x + R @ (self.arm * 1.4, 0, 0)))
            art.ref.set_data_3d([d["ref"][0]], [d["ref"][1]], [d["ref"][2]])
            art.label.set_position_3d(x + np.array([0.0, 0.0, -0.45]))   # below: drawings go up
            if d["path"]:
                art.path.set_data_3d(*np.array(d["path"]).T)
            art.trail.set_data_3d(*_trail_xyz(d["trail"]))
            points += [x, d["ref"]] + [p for p in d["trail"] if p is not None]
            pen = " (pen)" if d["pen"] else ""
            lines.append(f"{d['number']}: {d['command'] or '-'}{pen}")
        _box(self.ax, points)
        caption = f"{snap['caption']}\n" if snap.get("caption") else ""
        self.title.set_text(caption + "   ".join(lines) + f"\nt={snap['t']:5.1f}s")

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
    src.add_argument("--demo", metavar="CMD,CMD",
                     help="a demo from demos.py (e.g. showcase) or a list like takeoff,1:star,2:heart,wait,all:land")
    ap.add_argument("--file", type=Path, default=config.LATEST_JSON,
                    help="command file to watch (default: %(default)s)")
    ap.add_argument("--drones", type=int, default=config.DRONE_COUNT,
                    help="how many drones (default: config.DRONE_COUNT = %(default)s)")
    ap.add_argument("--spacing", type=float, default=config.DRONE_SPACING,
                    help="metres between start positions (default: %(default)s)")
    ap.add_argument("--vehicle", choices=("hummingbird", "crazyflie"), default="hummingbird")
    ap.add_argument("--headless", action="store_true",
                    help="no window, run faster than real time (needs --demo)")
    ap.add_argument("--save", type=Path, help="save a picture of the final state to this PNG")
    args = ap.parse_args(argv)
    if args.headless and not args.demo:
        ap.error("--headless needs --demo")
    if args.drones < 1:
        ap.error("--drones must be at least 1")
    if args.drones != config.DRONE_COUNT:
        _log(f"Note: the voice app accepts drones 1-{config.DRONE_COUNT} "
             f"(config.DRONE_COUNT); set it to {args.drones} there to match.")

    sim = RotorPySim(drones=args.drones, spacing=args.spacing, vehicle=args.vehicle)
    stop = threading.Event()

    if args.headless:
        import matplotlib
        matplotlib.use("Agg")
        sim.script = DemoScript(parse_demo(args.demo))
        wall0 = time.monotonic()
        for _ in range(int(1.0 / PHYSICS_DT)):                # 1 s on the ground first
            sim.step()
        while sim.busy():
            sim.step()
        snap = sim.snapshot()
        sim.log(f"Done: {snap['t']:.1f} s simulated in {time.monotonic() - wall0:.1f} s wall")
        for d in snap["drones"]:
            x = d["x"]
            sim.log(f"  drone {d['number']}: pos ({x[0]:+.2f}, {x[1]:+.2f}, {x[2]:+.2f}) m, "
                    f"{sum(p is not None for p in d['trail'])} trail dots")
        if args.save:
            viewer = Viewer(sim)
            viewer.draw(snap)
            viewer.fig.savefig(args.save, dpi=90)
            sim.log(f"Saved {args.save}")
        return

    source = None
    if args.demo:
        sim.script = DemoScript(parse_demo(args.demo))
        sim.t = -1.0                               # 1 s on the ground before the demo starts
    elif args.ros:
        source = threading.Thread(target=subscribe_ros, args=(sim, stop))
    else:
        source = threading.Thread(target=watch_file, args=(sim, stop, args.file))
    if source is not None:
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
