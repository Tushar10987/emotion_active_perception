from __future__ import annotations

import json
import math
import random
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..utils.json_logger import get_json_logger


class UncertaintyNode(Node):
    def __init__(self) -> None:
        super().__init__("uncertainty_node")
        self.logger = get_json_logger("uncertainty")
        self.method = self.declare_parameter("method", "mc_dropout").get_parameter_value().string_value
        self.mc_samples = self.declare_parameter("mc_samples", 10).get_parameter_value().integer_value
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.pub_state = self.create_publisher(String, "/emotion_state", 10)

    def estimate_uncertainty(self) -> float:
        # Stub MC-dropout style uncertainty: higher variance => higher score
        samples = [random.random() for _ in range(max(1, int(self.mc_samples)))]
        mean = sum(samples) / len(samples)
        var = sum((s - mean) ** 2 for s in samples) / len(samples)
        return min(1.0, max(0.0, var * 3.0))

    def emotion_from_uncertainty(self, u: float) -> str:
        if u >= 0.5:
            return "cautious"
        if u <= 0.2:
            return "exploratory"
        return "neutral"

    def on_image(self, msg: Image) -> None:
        u = self.estimate_uncertainty()
        state = self.emotion_from_uncertainty(u)
        out = String()
        out.data = state
        self.pub_state.publish(out)
        self.logger.info(json.dumps({"event": "uncertainty", "u": u, "state": state}))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = UncertaintyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

