from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List


def select_frames_for_labelling(meta_dir: Path, k: int = 10) -> List[Path]:
    metas = sorted(meta_dir.glob("*.json"))
    # Placeholder: pick first k
    return metas[:k]


def export_onnx_stub(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(b"ONNX_STUB")


def run_active_cycle(dataset_dir: Path, out_dir: Path) -> Dict[str, str]:
    meta_dir = dataset_dir / "meta"
    selected = select_frames_for_labelling(meta_dir)
    labels_manifest = out_dir / "labels_selected.json"
    out_dir.mkdir(parents=True, exist_ok=True)
    labels_manifest.write_text(json.dumps([p.name for p in selected], indent=2))
    onnx_out = out_dir / "model.onnx"
    export_onnx_stub(onnx_out)
    return {"labels_manifest": str(labels_manifest), "onnx": str(onnx_out)}

