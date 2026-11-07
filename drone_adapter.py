#!/usr/bin/env python3
"""
Drone adapter: /phlai/voice_command (JSON String)  ->  geometry_msgs/Twist.

This is the ONLY drone-specific file. A command's params are either one move
{"linear": [x,y,z] m/s, "angular_z": rad/s, "duration": s} or a sequence {"steps": [move, ...]}
played in order on a fixed schedule (no drift). duration 0 = hold until the next command.
After a sequence ends the adapter publishes zeros (hover). Open-loop and timed on purpose.

Optional Gazebo trail (see gz_trail.py): steps with "pen": true leave glowing dots along the
drone's real path; "clear_trail": true in params wipes the old trail first.

Parameters (override with --ros-args -p name:=value):
  command_topic   default /phlai/voice_command
  cmd_vel_topic   default /X3/gazebo/command/twist   (Gazebo X3 quadcopter via ros_gz_bridge)
  rate_hz         default 20.0
  trail           default true    draw the Gazebo trail (needs python3-gz-transport13/-msgs10)
  trail_model     default x3      name of the model to follow
  trail_lifetime  default 120.0   seconds (sim time) before a dot disappears; 0 = never
  pen_delay       default 0.0     seconds to delay pen up/down (only if strokes look clipped or smeared)

For PX4/ArduPilot/etc., write a different adapter that consumes the same JSON.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class DroneAdapter(Node):
    def __init__(self):
        super().__init__("phlai_drone_adapter")
        self.declare_parameter("command_topic", "/phlai/voice_command")
        self.declare_parameter("cmd_vel_topic", "/X3/gazebo/command/twist")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("trail", True)
        self.declare_parameter("trail_model", "x3")
        self.declare_parameter("trail_lifetime", 120.0)
        self.declare_parameter("pen_delay", 0.0)
        command_topic = self.get_parameter("command_topic").value
        vel_topic = self.get_parameter("cmd_vel_topic").value
        rate = float(self.get_parameter("rate_hz").value)
        self.pen_delay = float(self.get_parameter("pen_delay").value)

        self.pub = self.create_publisher(Twist, vel_topic, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        self.create_timer(1.0 / rate, self._tick)

        self.sequence = []   # list of (end_time_or_None, Twist, pen)
        self.t0 = 0.0
        self.active = False  # stay silent until the first command arrives
        self.get_logger().info(f"{command_topic} -> {vel_topic}")

        self.trail = None
        self._pen_target = self._pen_applied = False
        self._pen_changed_at = 0.0
        if self.get_parameter("trail").value:
            self._start_trail()

    def _start_trail(self):
        try:
            from gz_trail import GzTrail
            self.trail = GzTrail(
                log=self.get_logger().info,
                model_name=self.get_parameter("trail_model").value,
                lifetime=float(self.get_parameter("trail_lifetime").value),
            )
            self.trail.start()
        except Exception as exc:
            self.trail = None
            self.get_logger().warn(
                f"Gazebo trail disabled ({type(exc).__name__}: {exc}). To enable it: "
                "sudo apt install python3-gz-transport13 python3-gz-msgs10, and run this node "
                "with the system Python (/usr/bin/python3), NOT the .venv (its pip protobuf "
                "breaks the Gazebo bindings).")

    @staticmethod
    def _make_step(step):
        x, y, z = (float(v) for v in step.get("linear", (0.0, 0.0, 0.0)))
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.linear.z = x, y, z
        twist.angular.z = float(step.get("angular_z", 0.0))
        return twist, float(step.get("duration", 0.0)), bool(step.get("pen", False))

    def _on_command(self, msg):
        try:
            data = json.loads(msg.data)
            params = data.get("params") or {}
            steps = params.get("steps")
            if steps is None:
                steps = [params]          # a single move
            sequence, t_end = [], 0.0
            for step in steps:
                twist, duration, pen = self._make_step(step)
                if duration <= 0:         # open-ended hold: last step
                    sequence.append((None, twist, pen))
                    break
                t_end += duration
                sequence.append((t_end, twist, pen))
        except (ValueError, TypeError, AttributeError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f"Ignoring bad command: {exc}")
            return

        if params.get("clear_trail") and self.trail is not None:
            self.trail.clear()
        self.sequence = sequence
        self.t0 = time.monotonic()
        self.active = True
        total = sequence[-1][0] if sequence and sequence[-1][0] is not None else "until next command"
        self.get_logger().info(f"{data.get('command')}: {len(sequence)} step(s), total {total}")

    def _update_pen(self, pen, now):
        """Apply pen up/down changes after pen_delay so they line up with the drone's real motion."""
        if self.trail is None:
            return
        if pen != self._pen_target:
            self._pen_target, self._pen_changed_at = pen, now
        if self._pen_applied != self._pen_target and now - self._pen_changed_at >= self.pen_delay:
            self._pen_applied = self._pen_target
            self.trail.set_pen(self._pen_applied)

    def _tick(self):
        if not self.active:
            return
        now = time.monotonic()
        elapsed = now - self.t0
        twist, pen = Twist(), False       # hover, pen up once the sequence is over
        for t_end, step_twist, step_pen in self.sequence:
            if t_end is None or elapsed < t_end:
                twist, pen = step_twist, step_pen
                break
        self._update_pen(pen, now)
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = DroneAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.trail is not None:
            node.trail.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
