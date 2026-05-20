"""Tests for validation metrics and plots extraction."""

from __future__ import annotations

import json
from pathlib import Path

from model.joint_loop import validation


def test_read_task_metrics(tmp_path: Path) -> None:
    # 1. Non-existent file should return empty dict
    non_existent = tmp_path / "missing.json"
    assert validation._read_task_metrics(non_existent) == {}

    # 2. Correct JSON file should be parsed successfully
    valid_file = tmp_path / "valid.json"
    dummy_metrics = {"metrics/mAP50-95(B)": 0.45, "metrics/precision": 0.85}
    valid_file.write_text(json.dumps(dummy_metrics))
    assert validation._read_task_metrics(valid_file) == dummy_metrics

    # 3. Invalid JSON file should handle exception and return empty dict
    invalid_file = tmp_path / "invalid.json"
    invalid_file.write_text("{invalid")
    assert validation._read_task_metrics(invalid_file) == {}


def test_gather_validation_assets(tmp_path: Path) -> None:
    # 1. Empty/missing directory should return empty list
    missing_dir = tmp_path / "missing_dir"
    assert validation._gather_validation_assets(missing_dir) == []

    # 2. Existing directory with different asset types
    val_dir = tmp_path / "val_dir"
    val_dir.mkdir()

    # Create dummy assets
    pred0 = val_dir / "val_batch0_pred.jpg"
    pred1 = val_dir / "val_batch1_pred.jpg"
    conf = val_dir / "confusion_matrix.png"
    conf_norm = val_dir / "confusion_matrix_normalized.png"
    pr_curve = val_dir / "PR_curve.png"
    other_file = val_dir / "labels.txt"

    for file in [pred0, pred1, conf, conf_norm, pr_curve, other_file]:
        file.touch()

    assets = validation._gather_validation_assets(val_dir)
    # Check that only matching assets are returned and they are sorted by name
    expected = [conf, conf_norm, pr_curve, pred0, pred1]
    expected.sort(key=lambda p: p.name)
    assert assets == expected
