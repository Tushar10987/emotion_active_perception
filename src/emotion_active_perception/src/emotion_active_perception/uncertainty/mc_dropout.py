from __future__ import annotations

import random
from dataclasses import dataclass
from typing import List


@dataclass
class FrameUncertainty:
    scalar: float
    map_width: int
    map_height: int
    # A flattened grayscale map [0..255], optional lightweight representation
    map_gray_flat: List[int]


def mc_dropout_uncertainty(height: int = 120, width: int = 160, samples: int = 10) -> FrameUncertainty:
    values = [random.random() for _ in range(samples)]
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / len(values)
    scalar = min(1.0, max(0.0, var * 3.0))

    # Create a simple vignette-like map with noise to mimic spatial uncertainty
    grid = []
    for y in range(height):
        for x in range(width):
            nx = (x - width / 2) / (width / 2)
            ny = (y - height / 2) / (height / 2)
            base = (nx * nx + ny * ny) / 2.0
            noise = random.random() * 0.1
            val = min(1.0, max(0.0, scalar * (0.6 + noise) + base * 0.2))
            grid.append(int(val * 255))
    return FrameUncertainty(scalar=scalar, map_width=width, map_height=height, map_gray_flat=grid)

