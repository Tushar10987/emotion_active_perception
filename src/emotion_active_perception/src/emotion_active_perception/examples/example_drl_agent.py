from __future__ import annotations

import json
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ExampleDrlAgent(Node):
    def __init__(self) -> None:
        super().__init__("example_drl_agent")
        self.create_subscription(String, "/emotion_state", self.on_emotion, 10)

    def on_emotion(self, msg: String) -> None:
        # In a real integration, adjust the observation and policy conditioning
        self.get_logger().info(json.dumps({"event": "emotion_observed", "state": msg.data}))


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExampleDrlAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

