from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..utils.json_logger import get_json_logger


@dataclass
class FrameMetadata:
    frame_id: str
    timestamp: float
    camera_pose: Dict[str, float]
    robot_action: Dict[str, Any]
    uncertainty: float
    info_gain_estimate: float
    pseudo_labels: List[Dict[str, Any]]
    human_labelled: bool


class DataManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("data_manager_node")
        self.logger = get_json_logger("data_manager")

        self.dataset_dir = Path(self.declare_parameter("dataset_dir", str(Path.home() / "ap_dataset")).get_parameter_value().string_value)
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        (self.dataset_dir / "images").mkdir(exist_ok=True)
        (self.dataset_dir / "meta").mkdir(exist_ok=True)

        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.on_image, 10)
        self.emotion_sub = self.create_subscription(String, "/emotion_state", self.on_emotion, 10)

        self.current_emotion = "neutral"
        self.frame_counter = 0
        self.logger.info(json.dumps({"event": "started", "dataset_dir": str(self.dataset_dir)}))

    def on_emotion(self, msg: String) -> None:
        self.current_emotion = msg.data

    def build_metadata(self) -> FrameMetadata:
        # In real integration, query TF for camera pose and planner for last action
        meta = FrameMetadata(
            frame_id=f"frame_{self.frame_counter:06d}",
            timestamp=self.get_clock().now().nanoseconds / 1e9,
            camera_pose={"x": 0.0, "y": 0.0, "z": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            robot_action={"type": "none", "value": 0.0, "emotion": self.current_emotion},
            uncertainty=0.0,
            info_gain_estimate=0.0,
            pseudo_labels=[],
            human_labelled=False,
        )
        return meta

    def on_image(self, msg: Image) -> None:
        self.frame_counter += 1
        meta = self.build_metadata()
        img_path = self.dataset_dir / "images" / f"{meta.frame_id}.png"
        meta_path = self.dataset_dir / "meta" / f"{meta.frame_id}.json"

        # Save placeholder image (no conversion without cv_bridge; keep minimal deps)
        # We record only metadata for the smoke path. Users can enable cv_bridge externally.
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(asdict(meta), f, indent=2)

        self.logger.info(json.dumps({"event": "saved_frame_metadata", "frame_id": meta.frame_id, "meta_path": str(meta_path)}))


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DataManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

