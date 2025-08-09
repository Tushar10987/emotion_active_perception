from pathlib import Path


def test_action_exists():
    action_path = Path(__file__).resolve().parents[1] / "src" / "emotion_active_perception" / "actions" / "MoveAndCapture.action"
    assert action_path.exists(), f"Missing action file at {action_path}"

