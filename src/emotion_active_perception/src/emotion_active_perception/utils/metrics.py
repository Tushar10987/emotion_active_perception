from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple


@dataclass
class CalibrationMetrics:
    expected_calibration_error: float


def compute_ece(probabilities: Iterable[float], labels: Iterable[int], num_bins: int = 10) -> CalibrationMetrics:
    probs = list(probabilities)
    labs = list(labels)
    assert len(probs) == len(labs)
    if not probs:
        return CalibrationMetrics(expected_calibration_error=0.0)
    bins: List[Tuple[float, float]] = [(i / num_bins, (i + 1) / num_bins) for i in range(num_bins)]
    ece = 0.0
    n = len(probs)
    for low, high in bins:
        in_bin = [i for i, p in enumerate(probs) if low <= p < high or (high == 1.0 and p == 1.0)]
        if not in_bin:
            continue
        avg_conf = sum(probs[i] for i in in_bin) / len(in_bin)
        avg_acc = sum(1 if labs[i] == 1 else 0 for i in in_bin) / len(in_bin)
        ece += (len(in_bin) / n) * abs(avg_conf - avg_acc)
    return CalibrationMetrics(expected_calibration_error=ece)

