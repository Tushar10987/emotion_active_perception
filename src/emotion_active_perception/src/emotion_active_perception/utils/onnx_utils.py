from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Optional


@dataclass
class Detection:
    bbox: List[float]
    score: float
    cls: str


class OnnxDetector:
    """Stub ONNX runtime detector wrapper.

    Replace internals with onnxruntime.InferenceSession for a real model.
    """

    def __init__(self, onnx_path: Optional[str] = None, fp16: bool = True) -> None:
        self.onnx_path = onnx_path
        self.fp16 = fp16

    def infer(self, image_bgr: Any) -> List[Detection]:
        # Return a deterministic stub detection for pipeline wiring
        h, w = getattr(image_bgr, "shape", (480, 640))[:2]
        return [Detection(bbox=[w * 0.25, h * 0.25, w * 0.5, h * 0.5], score=0.42, cls="object")]

