from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List


@dataclass
class TrtDetection:
    bbox: List[float]
    score: float
    cls: str


class TensorRTWrapper:
    """Minimal stub for TensorRT engine execution.

    This is a no-op implementation to keep dependencies light. Swap with a real
    TensorRT runtime if available.
    """

    def __init__(self, engine_path: str | None = None) -> None:
        self.engine_path = engine_path

    def infer(self, image_bgr: Any) -> List[TrtDetection]:
        height, width = getattr(image_bgr, "shape", (480, 640))[:2]
        return [TrtDetection(bbox=[width * 0.3, height * 0.3, width * 0.4, height * 0.4], score=0.55, cls="object")]

