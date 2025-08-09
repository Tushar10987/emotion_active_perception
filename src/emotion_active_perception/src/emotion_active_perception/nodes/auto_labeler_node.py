from __future__ import annotations

import json
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..utils.json_logger import get_json_logger
from ..utils.onnx_utils import OnnxDetector


class AutoLabelerNode(Node):
    def __init__(self) -> None:
        super().__init__("auto_labeler_node")
        self.logger = get_json_logger("auto_labeler")
        model_path = self.declare_parameter("onnx_path", "").get_parameter_value().string_value
        self.detector = OnnxDetector(onnx_path=model_path)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.pub = self.create_publisher(String, "/ap/detections", 10)
        self.logger.info(json.dumps({"event": "started", "onnx_path": model_path}))

    def on_image(self, msg: Image) -> None:
        detections = self.detector.infer(image_bgr=None)
        serialized = json.dumps([det.__dict__ for det in detections])
        out = String()
        out.data = serialized
        self.pub.publish(out)
        self.logger.info(json.dumps({"event": "detections", "count": len(detections)}))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = AutoLabelerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

