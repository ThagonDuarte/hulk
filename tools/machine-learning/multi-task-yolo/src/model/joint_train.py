"""Joint training CLI for multi-task Hydra YOLO models.

Distinct from `src/model/train.py` (single-task finetune + tuning). Loads a
single Hydra instance with all heads from one --hydra_model_name spec and
runs the custom joint training loop. Outputs land at:

    runs/joint_train/<hydra_model>~<wordlet>/<task>/{best,last}.pt
"""

from __future__ import annotations

import logging
import random
from pathlib import Path

import click
import numpy as np
import torch
import wandb
from ultralytics.utils.torch_utils import select_device
from wonderwords import RandomWord

from model.hydra import Hydra
from model.joint_loop.criteria import JointLossHyp
from model.joint_loop.dataloaders import (
    InterleavedTaskDataloader,
    build_task_dataloader,
)
from model.joint_loop.loop import JointTrainConfig, train_joint
from utils.model_naming import (
    HYDRA_MODEL_NAME_TYPE,
    HydraModelName,
    TaskType,
)
from validation.validator import DatasetNotFoundError

logger = logging.getLogger(__name__)


def _parse_kv_floats(values: tuple[str, ...]) -> dict[TaskType, float]:
    out: dict[TaskType, float] = {}
    for raw in values:
        if "=" not in raw:
            raise click.BadParameter(  # noqa: TRY003
                f"expected task=value, got {raw!r}",
            )
        k, v = raw.split("=", 1)
        try:
            task = TaskType(k.strip())
        except ValueError as exc:
            raise click.BadParameter(f"unknown task: {k!r}") from exc  # noqa: TRY003
        out[task] = float(v.strip())
    return out


def _resolve_dataset_path(
    task: TaskType,
    object_yaml: Path,
    pose_yaml: Path,
    seg_yaml: Path,
    assets_dir: Path,
) -> Path:
    match task:
        case TaskType.OBJECT:
            return assets_dir / "datasets" / object_yaml
        case TaskType.POSE:
            return assets_dir / "datasets" / pose_yaml
        case TaskType.SEGMENTATION:
            return assets_dir / "datasets" / seg_yaml
    raise DatasetNotFoundError(task)


def _seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)  # noqa: NPY002
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Joint multi-task training of a Hydra model.",
)
@click.option(
    "--hydra_model_name",
    required=True,
    type=HYDRA_MODEL_NAME_TYPE,
    help="Hydra model spec, e.g. yolo26m=f11+yolo26m-pose+yolo26m-seg.",
)
@click.option("--object_dataset_name", default="coco.yaml", type=Path)
@click.option("--pose_dataset_name", default="coco-pose.yaml", type=Path)
@click.option("--segmentation_dataset_name", default="coco.yaml", type=Path)
@click.option("--assets_dir", default=Path("assets"), type=Path)
@click.option("--runs_dir", default=Path("runs"), type=Path)
@click.option("--joint_train_dir", default=Path("joint_train"), type=Path)
@click.option("--device", default="-1", type=str)
@click.option("--workers", default=8, type=int)
@click.option("--seed", default=0, type=int)
@click.option("--epochs", default=100, type=int)
@click.option("--patience", default=30, type=int)
@click.option("--warmup_epochs", default=3, type=int)
@click.option("--val_interval", default=1, type=int)
@click.option("--batch", default=16, type=int)
@click.option("--imgsz", default=640, type=int)
@click.option("--lr_backbone", default=0.001, type=float)
@click.option("--lr_heads", default=0.01, type=float)
@click.option("--lr_logvar", default=0.001, type=float)
@click.option("--momentum", default=0.9, type=float)
@click.option("--weight_decay", default=1e-5, type=float)
@click.option("--max_grad_norm", default=10.0, type=float)
@click.option(
    "--optimizer",
    type=click.Choice(["MuSGD", "AdamW"], case_sensitive=False),
    default="MuSGD",
)
@click.option("--init_log_var", multiple=True, type=str)
@click.option("--task_weight", multiple=True, type=str)
@click.option("--amp/--no_amp", default=True)
@click.option("--ema/--no_ema", default=True)
@click.option("--clip_heads", is_flag=True, default=False)
@click.option("--resume", is_flag=True, default=False)
@click.option("--log_interval", default=50, type=int)
@click.option("--wandb_project", default="multi-task-yolo", type=str)
def main(
    *,
    hydra_model_name: HydraModelName,
    object_dataset_name: Path,
    pose_dataset_name: Path,
    segmentation_dataset_name: Path,
    assets_dir: Path,
    runs_dir: Path,
    joint_train_dir: Path,
    device: str,
    workers: int,
    seed: int,
    epochs: int,
    patience: int,
    warmup_epochs: int,
    val_interval: int,
    batch: int,
    imgsz: int,
    lr_backbone: float,
    lr_heads: float,
    lr_logvar: float,
    momentum: float,
    weight_decay: float,
    max_grad_norm: float,
    optimizer: str,
    init_log_var: tuple[str, ...],
    task_weight: tuple[str, ...],
    amp: bool,
    ema: bool,
    clip_heads: bool,
    resume: bool,
    log_interval: int,
    wandb_project: str,
) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    _seed_everything(seed)

    if resume:
        logger.warning(
            "--resume requested but mid-epoch resume is not supported in v1; "
            "the previous epoch is restarted from step 0",
        )

    wordlet = RandomWord().word(
        word_min_length=4, word_max_length=8, include_categories=["nouns"]
    )
    run_id = f"{hydra_model_name}~{wordlet}"
    run_dir = runs_dir / joint_train_dir / run_id

    head_source_paths: dict[TaskType, Path] = {}
    datasets_per_task: dict[TaskType, Path] = {}
    task_dict: dict[TaskType, Path] = {}
    for head in hydra_model_name.heads:
        task = head.task_type()
        head_path = assets_dir / (head.name + ".pt")
        head_source_paths[task] = head_path
        task_dict[task] = head_path
        datasets_per_task[task] = _resolve_dataset_path(
            task,
            object_dataset_name,
            pose_dataset_name,
            segmentation_dataset_name,
            assets_dir,
        )

    backbone_path = assets_dir / (hydra_model_name.backbone.name + ".pt")
    hydra = Hydra(backbone_path=str(backbone_path), task_dict=task_dict)
    selected_device = select_device(device)

    loaders = {}
    for task, dataset_yaml in datasets_per_task.items():
        loader, _dataset = build_task_dataloader(
            task,
            dataset_yaml,
            imgsz=imgsz,
            batch=batch,
            workers=workers,
        )
        loaders[task] = loader
    interleaved = InterleavedTaskDataloader(loaders)

    config = JointTrainConfig(
        epochs=epochs,
        patience=patience,
        warmup_epochs=warmup_epochs,
        val_interval=val_interval,
        log_interval=log_interval,
        optimizer_name=optimizer,
        lr_backbone=lr_backbone,
        lr_heads=lr_heads,
        lr_logvar=lr_logvar,
        momentum=momentum,
        weight_decay=weight_decay,
        max_grad_norm=max_grad_norm,
        use_amp=amp,
        use_ema=ema,
        clip_heads=clip_heads,
        init_log_var=_parse_kv_floats(init_log_var),
        task_weights=_parse_kv_floats(task_weight),
        hyp=JointLossHyp(epochs=epochs),
    )

    wandb_run = wandb.init(project=wandb_project, name=run_id)

    train_joint(
        hydra=hydra,
        hydra_model=hydra_model_name,
        interleaved=interleaved,
        datasets_per_task=datasets_per_task,
        head_source_paths=head_source_paths,
        run_dir=run_dir,
        runs_dir=runs_dir,
        config=config,
        device=selected_device,
        device_str=device,
        imgsz=imgsz,
        batch=batch,
        seed=seed,
        wandb_run=wandb_run,
    )


if __name__ == "__main__":
    main()
