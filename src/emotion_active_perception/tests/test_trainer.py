from pathlib import Path
from emotion_active_perception.trainer.active_loop import run_active_cycle


def test_run_active_cycle(tmp_path: Path):
    dataset = tmp_path / "dataset"
    meta = dataset / "meta"
    meta.mkdir(parents=True)
    for i in range(3):
        (meta / f"frame_{i:06d}.json").write_text("{}")
    out = tmp_path / "out"
    res = run_active_cycle(dataset, out)
    assert Path(res["onnx"]).exists()
    assert Path(res["labels_manifest"]).exists()

