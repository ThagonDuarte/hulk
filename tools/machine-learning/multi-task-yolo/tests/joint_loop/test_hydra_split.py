"""Parity test: run_backbone+run_head composition matches forward()."""

from __future__ import annotations

from pathlib import Path

import pytest
import torch

from model.hydra import Hydra
from utils.export_hydra import set_export_mode
from utils.model_naming import TaskType

ASSETS_DIR = Path("assets")
BACKBONE_PATH = ASSETS_DIR / "yolo26m.pt"
POSE_HEAD_PATH = ASSETS_DIR / "yolo26m-pose.pt"
DETECT_HEAD_PATH = ASSETS_DIR / "yolo26m.pt"


@pytest.fixture(scope="module")
def hydra_two_head() -> Hydra:
    if not (BACKBONE_PATH.exists() and POSE_HEAD_PATH.exists()):
        pytest.skip("yolo26m / yolo26m-pose assets not present")
    return Hydra(
        backbone_path=str(BACKBONE_PATH),
        task_dict={
            TaskType.OBJECT: DETECT_HEAD_PATH,
            TaskType.POSE: POSE_HEAD_PATH,
        },
    )


def test_run_backbone_then_run_head_matches_forward(
    hydra_two_head: Hydra,
) -> None:
    # End2end heads (yolo26m, yolo26m-pose) emit `(tensor, dict)` in eval mode,
    # which `forward()` cannot flatten. Export mode forces them to emit a
    # single tensor, exercising the path the joint-training loop will use.
    set_export_mode(hydra_two_head)
    hydra_two_head.eval()
    x = torch.randn(1, 3, 640, 640)

    with torch.no_grad():
        baseline = hydra_two_head.forward(x)

        feat, y_backbone = hydra_two_head.run_backbone(x)
        for task, expected_keys in [
            (TaskType.OBJECT, TaskType.OBJECT.output_names()),
            (TaskType.POSE, TaskType.POSE.output_names()),
        ]:
            raw = hydra_two_head.run_head(str(task), feat, y_backbone)
            tensors = (raw,) if isinstance(raw, torch.Tensor) else tuple(raw)
            for key, tensor in zip(expected_keys, tensors, strict=True):
                assert torch.allclose(tensor, baseline[key], atol=1e-6), (
                    f"mismatch on {key} for task {task}"
                )
