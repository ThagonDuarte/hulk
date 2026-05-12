"""End-to-end smoke test for the joint training loop.

Skipped if the standard YOLO checkpoints / COCO YAMLs are not present on
disk. The test runs 2 epochs on whichever subset of tasks the user has
assets for and asserts:

* the loop completes without raising,
* per-task `last.pt` files are written,
* `joint_state.json` and `config.json` are written.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest
import torch

ASSETS_DIR = Path("assets")
COCO_YAML = ASSETS_DIR / "datasets" / "coco.yaml"
# dataset root declared inside coco.yaml; val split must be present for the
# dataloader to succeed without attempting a privileged download
_COCO_VAL_TXT = ASSETS_DIR / "datasets" / "coco" / "val2017.txt"

pytestmark = pytest.mark.skipif(
    not (
        ASSETS_DIR.exists()
        and (ASSETS_DIR / "yolo26m.pt").exists()
        and COCO_YAML.exists()
        and _COCO_VAL_TXT.exists()
    ),
    reason="yolo26m / coco.yaml / COCO image data not present",
)


def test_smoke_two_epochs(tmp_path: Path) -> None:
    from model.hydra import Hydra
    from model.joint_loop.criteria import JointLossHyp
    from model.joint_loop.dataloaders import (
        InterleavedTaskDataloader,
        build_task_dataloader,
    )
    from model.joint_loop.loop import JointTrainConfig, train_joint
    from utils.model_naming import HydraModelName, ModelName, TaskType

    hydra_model = HydraModelName(
        backbone=ModelName("yolo26m"),
        heads=[ModelName("yolo26m")],
        number_of_frozen_modules=11,
    )
    backbone_path = ASSETS_DIR / "yolo26m.pt"
    head_source_paths = {TaskType.OBJECT: backbone_path}
    task_dict = {TaskType.OBJECT: backbone_path}

    hydra = Hydra(
        backbone_path=str(backbone_path),
        task_dict=task_dict,
    )

    loader, _ = build_task_dataloader(
        TaskType.OBJECT,
        COCO_YAML,
        imgsz=320,
        batch=2,
        workers=0,
    )
    interleaved = InterleavedTaskDataloader({TaskType.OBJECT: loader})

    config = JointTrainConfig(
        epochs=2,
        patience=999,
        warmup_epochs=1,
        val_interval=1,
        log_interval=1,
        optimizer_name="MuSGD" if torch.cuda.is_available() else "AdamW",
        lr_backbone=0.0,
        lr_heads=0.001,
        lr_logvar=0.001,
        use_amp=False,
        use_ema=False,
        hyp=JointLossHyp(epochs=2),
        max_steps_per_epoch=5,
    )
    run_dir = tmp_path / "joint_train" / "smoke~test"
    runs_dir = tmp_path

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    os.environ["WANDB_MODE"] = "disabled"

    train_joint(
        hydra=hydra,
        hydra_model=hydra_model,
        interleaved=interleaved,
        datasets_per_task={TaskType.OBJECT: COCO_YAML},
        head_source_paths=head_source_paths,
        run_dir=run_dir,
        runs_dir=runs_dir,
        config=config,
        device=device,
        device_str="cpu" if device.type == "cpu" else "0",
        imgsz=320,
        batch=2,
        seed=0,
        wandb_run=None,
    )

    assert (run_dir / "config.json").exists()
    assert (run_dir / "joint_state.json").exists()
    assert (run_dir / "object" / "last.pt").exists()
