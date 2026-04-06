from __future__ import annotations

import logging
from pathlib import Path
from typing import cast

import click

from model.joint_config import (
    JointOptimizerConfig,
    JointSchedulerConfig,
    JointTaskConfig,
    JointTrainConfig,
    OptimizerName,
    TaskName,
)
from model.joint_trainer import JointHydraTrainer

DEVICE_EMPTY_ERROR = "must contain at least one device token"


def _normalize_device(device: str | None) -> str | None:
    if device is None:
        return None
    raw = device.strip()
    if not raw:
        return None
    if "," not in raw:
        return raw

    tokens = [token.strip() for token in raw.split(",") if token.strip()]
    if not tokens:
        raise click.BadParameter(DEVICE_EMPTY_ERROR)
    click.echo(
        f"Joint trainer currently runs single-device; using '{tokens[0]}'."
    )
    return tokens[0]


def _normalize_optimizer_name(raw_name: str) -> OptimizerName:
    normalized = raw_name.strip().lower()
    if normalized == "adamw":
        return "AdamW"
    return "SGD"


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Joint multi-task training for Hydra (detection + pose).",
)
@click.option(
    "--repo-root",
    type=click.Path(path_type=Path),
    default=Path.cwd().resolve(),
    show_default=True,
    help="Repository root used to derive default run paths.",
)
@click.option(
    "--project-train-dir",
    type=click.Path(path_type=Path),
    default=None,
    help="Train project directory. Defaults to <repo-root>/runs/train.",
)
@click.option(
    "--run-name",
    default="hydra-joint",
    show_default=True,
    help="Subfolder name for this joint training run.",
)
@click.option(
    "--foundation-path",
    type=click.Path(path_type=Path),
    default=Path("assets/yolo26m.pt"),
    show_default=True,
    help="Foundation checkpoint used for shared backbone.",
)
@click.option(
    "--detection-model",
    type=click.Path(path_type=Path),
    default=Path("assets/yolo26m.pt"),
    show_default=True,
    help="Detection checkpoint used to extract detection head.",
)
@click.option(
    "--pose-model",
    type=click.Path(path_type=Path),
    default=Path("assets/yolo26m-pose.pt"),
    show_default=True,
    help="Pose checkpoint used to extract pose head.",
)
@click.option(
    "--detection-data",
    type=click.Path(path_type=Path),
    default=Path("coco.yaml"),
    show_default=True,
    help="Detection dataset YAML path.",
)
@click.option(
    "--pose-data",
    type=click.Path(path_type=Path),
    default=Path("coco-pose.yaml"),
    show_default=True,
    help="Pose dataset YAML path.",
)
@click.option(
    "--epochs",
    type=int,
    default=70,
    show_default=True,
    help="Number of training epochs.",
)
@click.option(
    "--imgsz",
    type=int,
    default=640,
    show_default=True,
    help="Base image size for both tasks.",
)
@click.option(
    "--batch",
    type=int,
    default=16,
    show_default=True,
    help="Base batch size for both tasks.",
)
@click.option(
    "--workers",
    type=int,
    default=8,
    show_default=True,
    help="Base dataloader workers for both tasks.",
)
@click.option(
    "--detection-batch",
    type=int,
    default=None,
    help="Detection batch size override.",
)
@click.option(
    "--pose-batch",
    type=int,
    default=None,
    help="Pose batch size override.",
)
@click.option(
    "--detection-workers",
    type=int,
    default=None,
    help="Detection workers override.",
)
@click.option(
    "--pose-workers",
    type=int,
    default=None,
    help="Pose workers override.",
)
@click.option(
    "--detection-imgsz",
    type=int,
    default=None,
    help="Detection image size override.",
)
@click.option(
    "--pose-imgsz",
    type=int,
    default=None,
    help="Pose image size override.",
)
@click.option(
    "--fraction",
    type=float,
    default=1.0,
    show_default=True,
    help="Training data fraction for both datasets.",
)
@click.option(
    "--device",
    type=str,
    default=None,
    help="Single device, e.g. cpu, 0, cuda:0.",
)
@click.option(
    "--amp/--no-amp",
    default=True,
    show_default=True,
    help="Enable AMP mixed precision when CUDA is available.",
)
@click.option(
    "--use-ema/--no-use-ema",
    default=True,
    show_default=True,
    help="Enable EMA shadow model updates.",
)
@click.option(
    "--clip-grad-norm",
    type=float,
    default=10.0,
    show_default=True,
    help="Max gradient norm for clipping.",
)
@click.option(
    "--validate-every",
    type=int,
    default=1,
    show_default=True,
    help="Validate every N epochs.",
)
@click.option(
    "--save-period",
    type=int,
    default=-1,
    show_default=True,
    help="Save epoch checkpoint every N epochs (-1 disables).",
)
@click.option(
    "--log-every",
    type=int,
    default=25,
    show_default=True,
    help="Log training step metrics every N synchronized steps.",
)
@click.option(
    "--optimizer",
    type=click.Choice(["AdamW", "SGD"], case_sensitive=False),
    default="AdamW",
    show_default=True,
    help="Optimizer type for backbone and task heads.",
)
@click.option(
    "--backbone-lr",
    type=float,
    default=1e-4,
    show_default=True,
    help="Backbone learning rate.",
)
@click.option(
    "--head-lr",
    type=float,
    default=1e-4,
    show_default=True,
    help="Task head learning rate.",
)
@click.option(
    "--uncertainty-lr",
    type=float,
    default=1e-3,
    show_default=True,
    help="Uncertainty-weight learning rate.",
)
@click.option(
    "--weight-decay",
    type=float,
    default=5e-4,
    show_default=True,
    help="Weight decay for backbone and head optimizers.",
)
@click.option(
    "--momentum",
    type=float,
    default=0.9,
    show_default=True,
    help="Momentum used by SGD optimizer.",
)
@click.option(
    "--min-lr-ratio",
    type=float,
    default=0.01,
    show_default=True,
    help="Minimum LR ratio used for cosine schedulers.",
)
@click.option(
    "--seed",
    type=int,
    default=0,
    show_default=True,
    help="Random seed.",
)
@click.option(
    "--deterministic/--non-deterministic",
    default=True,
    show_default=True,
    help="Enable deterministic behavior where possible.",
)
@click.option(
    "--resume",
    type=click.Path(path_type=Path),
    default=None,
    help="Resume from joint training checkpoint (.pt).",
)
@click.option(
    "--dev-mode",
    is_flag=True,
    default=False,
    show_default=True,
    help="Use fast development settings.",
)
def main(
    *,
    repo_root: Path,
    project_train_dir: Path | None,
    run_name: str,
    foundation_path: Path,
    detection_model: Path,
    pose_model: Path,
    detection_data: Path,
    pose_data: Path,
    epochs: int,
    imgsz: int,
    batch: int,
    workers: int,
    detection_batch: int | None,
    pose_batch: int | None,
    detection_workers: int | None,
    pose_workers: int | None,
    detection_imgsz: int | None,
    pose_imgsz: int | None,
    fraction: float,
    device: str | None,
    amp: bool,
    use_ema: bool,
    clip_grad_norm: float,
    validate_every: int,
    save_period: int,
    log_every: int,
    optimizer: str,
    backbone_lr: float,
    head_lr: float,
    uncertainty_lr: float,
    weight_decay: float,
    momentum: float,
    min_lr_ratio: float,
    seed: int,
    deterministic: bool,
    resume: Path | None,
    dev_mode: bool,
) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(message)s",
    )

    repo_root = repo_root.resolve()
    project_dir = (
        project_train_dir.resolve()
        if project_train_dir
        else (repo_root / "runs" / "train")
    )

    normalized_device = _normalize_device(device)

    effective_epochs = epochs
    effective_fraction = fraction
    effective_run_name = run_name

    base_detection_batch = detection_batch or batch
    base_pose_batch = pose_batch or batch
    base_detection_workers = detection_workers or workers
    base_pose_workers = pose_workers or workers
    base_detection_imgsz = detection_imgsz or imgsz
    base_pose_imgsz = pose_imgsz or imgsz

    if dev_mode:
        effective_run_name = f"{run_name}-dev"
        effective_epochs = min(epochs, 5)
        effective_fraction = min(fraction, 0.1)
        base_detection_batch = min(base_detection_batch, 8)
        base_pose_batch = min(base_pose_batch, 8)
        base_detection_workers = min(base_detection_workers, 4)
        base_pose_workers = min(base_pose_workers, 4)
        base_detection_imgsz = min(base_detection_imgsz, 512)
        base_pose_imgsz = min(base_pose_imgsz, 512)

    tasks: dict[TaskName, JointTaskConfig] = {
        "detection": JointTaskConfig(
            head_name="detection",
            task="detect",
            model_path=detection_model.resolve(),
            data_yaml=detection_data.resolve(),
            batch=base_detection_batch,
            workers=base_detection_workers,
            imgsz=base_detection_imgsz,
        ),
        "pose": JointTaskConfig(
            head_name="pose",
            task="pose",
            model_path=pose_model.resolve(),
            data_yaml=pose_data.resolve(),
            batch=base_pose_batch,
            workers=base_pose_workers,
            imgsz=base_pose_imgsz,
        ),
    }

    config = JointTrainConfig(
        foundation_path=foundation_path.resolve(),
        tasks=tasks,
        project_dir=project_dir,
        run_name=effective_run_name,
        epochs=effective_epochs,
        fraction=effective_fraction,
        device=normalized_device,
        amp=amp,
        use_ema=use_ema,
        clip_grad_norm=clip_grad_norm,
        validate_every=max(validate_every, 1),
        save_period=save_period,
        log_every=max(log_every, 1),
        seed=seed,
        deterministic=deterministic,
        resume_checkpoint=resume.resolve() if resume else None,
        optimizer=JointOptimizerConfig(
            name=cast(OptimizerName, _normalize_optimizer_name(optimizer)),
            backbone_lr=backbone_lr,
            head_lr=head_lr,
            uncertainty_lr=uncertainty_lr,
            weight_decay=weight_decay,
            momentum=momentum,
        ),
        scheduler=JointSchedulerConfig(min_lr_ratio=min_lr_ratio),
    )

    trainer = JointHydraTrainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
