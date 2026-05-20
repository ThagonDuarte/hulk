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
from ultralytics.utils.autodevice import GPUInfo
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


def option(*args: object, **kwargs: object) -> object:
    """click.option wrapper that sets show_default=True on every option."""
    kwargs.setdefault("show_default", True)
    return click.option(*args, **kwargs)


def _cuda_index_from_device_arg(raw: str) -> int:
    if "," in raw:
        msg = "joint_train supports a single CUDA device, not multi-GPU"
        raise click.ClickException(msg)
    if raw == "-1":
        selected_ids = GPUInfo().select_idle_gpu(
            count=1, min_memory_fraction=0.2
        )
        if not selected_ids:
            msg = "no idle CUDA device met the auto-selection criteria"
            raise click.ClickException(msg)
        return selected_ids[0]
    if raw in {"", "cuda"}:
        return 0
    if raw.isdigit():
        return int(raw)
    msg = f"invalid device {raw!r}; use cpu, mps, -1, or CUDA index"
    raise click.ClickException(msg)


def _select_training_device(device: str | torch.device) -> torch.device:
    """Select one device without CUDA_VISIBLE_DEVICES remapping."""
    if isinstance(device, torch.device):
        selected = device
    else:
        raw = str(device).lower().replace("cuda:", "").strip()
        if raw in {"cpu", "none"}:
            return torch.device("cpu")
        if raw in {"mps", "mps:0"}:
            return torch.device("mps")
        selected = torch.device(f"cuda:{_cuda_index_from_device_arg(raw)}")

    if selected.type == "cuda":
        idx = selected.index or 0
        if not torch.cuda.is_available() or idx >= torch.cuda.device_count():
            msg = f"CUDA device {idx} is not available"
            raise click.ClickException(msg)
        torch.cuda.set_device(idx)
        logger.info("Using CUDA:%d (%s)", idx, torch.cuda.get_device_name(idx))
    return selected


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Joint multi-task training of a Hydra model.",
)
@option(
    "--hydra_model_name",
    required=True,
    type=HYDRA_MODEL_NAME_TYPE,
    help="Hydra model spec, e.g. yolo26m=f11+yolo26m-pose+yolo26m-seg.",
)
@option("--object_dataset_name", default="coco.yaml", type=Path)
@option("--pose_dataset_name", default="coco-pose.yaml", type=Path)
@option("--segmentation_dataset_name", default="coco.yaml", type=Path)
@option("--assets_dir", default=Path("assets"), type=Path)
@option("--runs_dir", default=Path("runs"), type=Path)
@option("--joint_train_dir", default=Path("joint_train"), type=Path)
@option("--device", default="-1", type=str)
@option("--workers", default=8, type=int)
@option("--seed", default=42, type=int)
@option("--epochs", default=100, type=int)
@option("--patience", default=30, type=int)
@option("--warmup_epochs", default=10, type=int)
@option("--val_interval", default=1, type=int)
@option("--batch", default=16, type=int)
@option("--imgsz", default=640, type=int)
@option("--lr_backbone", default=0.001, type=float)
@option("--lr_heads", default=0.01, type=float)
@option("--lr_logvar", default=0.0001, type=float)
@option("--momentum", default=0.9, type=float)
@option("--weight_decay", default=1e-5, type=float)
@option("--max_grad_norm", default=10.0, type=float)
@option(
    "--optimizer",
    type=click.Choice(["MuSGD", "AdamW"], case_sensitive=False),
    default="MuSGD",
)
@option("--init_log_var", multiple=True, type=str)
@option("--task_weight", multiple=True, type=str)
@option("--amp/--no_amp", default=True)
@option("--ema/--no_ema", default=True)
@option("--clip_heads/--no_clip_heads", default=True)
@option("--resume", is_flag=True, default=False)
@option("--log_interval", default=50, type=int)
@option("--wandb_project", default="multi-task-yolo", type=str)
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

    selected_device = _select_training_device(device)
    backbone_path = assets_dir / (hydra_model_name.backbone.name + ".pt")
    hydra = Hydra(backbone_path=str(backbone_path), task_dict=task_dict)

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
