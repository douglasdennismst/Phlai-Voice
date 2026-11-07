"""Where finished command messages go. Each sink has publish(message: dict) and close()."""
import json
import os

import config


class FileSink:
    """Always on: latest_command.json (atomic) + append-only command_history.jsonl."""

    def __init__(self):
        config.OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
        self.label = f"files: {config.OUTPUT_DIR}"

    def publish(self, message: dict):
        text = json.dumps(message, indent=2)
        tmp = config.LATEST_JSON.with_suffix(".json.tmp")
        tmp.write_text(text, encoding="utf-8")
        os.replace(tmp, config.LATEST_JSON)
        with config.HISTORY_JSONL.open("a", encoding="utf-8") as f:
            f.write(json.dumps(message, ensure_ascii=False) + "\n")

    def close(self):
        pass


class RosSink:
    """Publishes accepted commands as JSON on a std_msgs/String topic."""

    def __init__(self, topic):
        import rclpy
        from std_msgs.msg import String

        self._rclpy = rclpy
        self._String = String
        rclpy.init()
        self._node = rclpy.create_node(config.ROS_NODE_NAME)
        self._pub = self._node.create_publisher(String, topic, 10)
        self.label = f"ROS 2: publishing on {topic}"

    def publish(self, message: dict):
        if message.get("status") != "accepted":
            return
        msg = self._String()
        msg.data = json.dumps(message)
        self._pub.publish(msg)

    def close(self):
        try:
            self._node.destroy_node()
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass


def build_sinks(use_ros=True):
    sinks = [FileSink()]
    ros_status = "ROS 2: disabled"
    if use_ros and config.ROS_ENABLED:
        try:
            ros = RosSink(config.ROS_COMMAND_TOPIC)
            sinks.append(ros)
            ros_status = ros.label
        except Exception as exc:
            ros_status = f"ROS 2: unavailable ({type(exc).__name__}) - file output only"
    return sinks, ros_status
