from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ..utils.json_logger import get_json_logger


class EmotionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("emotion_controller_node")
        self.logger = get_json_logger("emotion_controller")
        self.state = self.declare_parameter("initial_state", "neutral").get_parameter_value().string_value
        self.sub = self.create_subscription(String, "/emotion_state", self.on_state_in, 10)
        self.pub = self.create_publisher(String, "/emotion_state", 10)

    def on_state_in(self, msg: String) -> None:
        # For now pass-through with hysteresis placeholder
        new_state = msg.data
        if new_state != self.state:
            self.state = new_state
            self.pub.publish(String(data=self.state))
            self.logger.info(json.dumps({"event": "emotion_state_change", "state": self.state}))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = EmotionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

