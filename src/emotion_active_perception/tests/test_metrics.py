from emotion_active_perception.utils.metrics import compute_ece


def test_compute_ece_empty():
    m = compute_ece([], [])
    assert m.expected_calibration_error == 0.0


def test_compute_ece_basic():
    m = compute_ece([0.9, 0.1], [1, 0], num_bins=2)
    assert 0.0 <= m.expected_calibration_error <= 1.0

