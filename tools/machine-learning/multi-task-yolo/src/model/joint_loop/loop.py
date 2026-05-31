"""Custom-loop joint trainer for multi-task Hydra models.

Owns synchronized gradient accumulation, AMP, gradient clipping, EMA update,
scheduler stepping, and the per-epoch validation/checkpoint flow.
"""

from __future__ import annotations

import logging
import random
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch.nn.utils import clip_grad_norm_
from ultralytics.utils import TQDM

import wandb
from model.hydra import Hydra
from model.joint_loop.checkpoints import (
    write_joint_state,
    write_per_task_checkpoint,
    write_run_config,
)
from model.joint_loop.criteria import (
    JointLossHyp,
    build_criterion,
    epoch_update,
)
from model.joint_loop.dataloaders import InterleavedTaskDataloader
from model.joint_loop.optim import (
    EMAHydra,
    JointOptimizers,
    build_joint_optimizers,
    build_schedulers,
    make_amp_scaler,
    parameters_in_optimizer,
)
from model.joint_loop.validation import run_validation
from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import HydraModelName, TaskType

logger = logging.getLogger(__name__)


@dataclass
class JointTrainConfig:
    epochs: int = 100
    patience: int = 30
    freeze_backbone_epochs: int = 10
    warmup_epochs_backbone: int = 15
    warmup_epochs_heads: int = 3
    val_interval: int = 1
    optimizer_name: str = "MuSGD"
    lr_backbone: float = 0.001
    lr_heads: float = 0.01
    lr_logvar: float = 0.0001
    momentum: float = 0.9
    weight_decay: float = 1e-5
    max_grad_norm: float = 10.0
    nominal_batch_size: int = 64
    use_amp: bool = True
    use_ema: bool = True
    clip_heads: bool = True
    init_log_var: dict[TaskType, float] = field(default_factory=dict)
    task_weights: dict[TaskType, float] = field(default_factory=dict)
    hyp: JointLossHyp = field(default_factory=JointLossHyp)
    max_steps_per_epoch: int | None = None  # cap steps; useful for smoke tests
    epoch_size_strategy: str | int = "max"


class _Patience:
    def __init__(self, patience: int) -> None:
        self._patience = patience
        self._counter = 0
        self.best = float("-inf")

    def update(self, score: float) -> bool:
        """Return True if `score` improves on the best so far."""
        if score > self.best:
            self.best = score
            self._counter = 0
            return True
        self._counter += 1
        return False

    def should_stop(self) -> bool:
        return self._counter >= self._patience


def _reset_trainable_flags(hydra: Hydra) -> None:
    """Undo checkpoint freeze flags while preserving YOLO's fixed DFL layer."""
    for p in hydra.parameters():
        p.requires_grad = True
    _freeze_dfl_parameters(hydra)


def _freeze_dfl_parameters(hydra: Hydra) -> None:
    """Keep DFL projection parameters frozen, matching Ultralytics Trainer."""
    for name, p in hydra.named_parameters():
        if ".dfl" in name:
            p.requires_grad = False


def train_joint(  # noqa: C901
    *,
    hydra: Hydra,
    hydra_model: HydraModelName,
    interleaved: InterleavedTaskDataloader,
    datasets_per_task: Mapping[TaskType, Path],
    head_source_paths: Mapping[TaskType, Path],
    run_dir: Path,
    config: JointTrainConfig,
    device: torch.device,
    device_str: str,
    imgsz: int,
    batch: int,
    seed: int,
    wandb_run: Any | None = None,
) -> None:
    """Run the joint training loop.

    `run_dir` is the per-run output directory (`runs/joint_train/<id>`).
    Per-task `.pt` files land in `<run_dir>/<task>/last.pt` and `best.pt`.
    """
    hydra.to(device)
    _reset_trainable_flags(hydra)
    tasks: list[TaskType] = sorted(
        (TaskType(k) for k in hydra.heads), key=lambda t: t.value
    )
    weighter = UncertaintyWeighter(tasks, init_log_var=config.init_log_var)
    weighter.to(device)

    accumulate = _accumulate_rounds(config, batch)
    weight_decay = config.weight_decay
    if config.nominal_batch_size > 0:
        weight_decay *= batch * accumulate / config.nominal_batch_size
    logger.info(
        "optimizer weight_decay=%g (base=%g, accumulate=%d)",
        weight_decay,
        config.weight_decay,
        accumulate,
    )
    optimizers = build_joint_optimizers(
        hydra,
        weighter,
        optimizer_name=config.optimizer_name,
        lr_backbone=config.lr_backbone,
        lr_heads=config.lr_heads,
        lr_logvar=config.lr_logvar,
        momentum=config.momentum,
        weight_decay=weight_decay,
    )
    schedulers = build_schedulers(
        optimizers,
        epochs=config.epochs,
        warmup_epochs_backbone=config.warmup_epochs_backbone,
        warmup_epochs_heads=config.warmup_epochs_heads,
    )
    criteria = {t: build_criterion(hydra, t, config.hyp) for t in tasks}

    ema = EMAHydra(hydra, weighter) if config.use_ema else None
    scaler = make_amp_scaler(
        enabled=config.use_amp and device.type == "cuda",
        device_type=device.type,
    )
    patience = _Patience(config.patience)

    write_run_config(
        run_dir / "config.json",
        {
            **{f"cfg.{k}": v for k, v in config.__dict__.items()},
            "hydra_model": str(hydra_model),
            "imgsz": imgsz,
            "batch": batch,
            "seed": seed,
            "device": device_str,
        },
    )

    global_step = 0
    for epoch in range(config.epochs):
        if epoch == 0:
            if config.freeze_backbone_epochs > 0:
                logger.info(
                    "Freezing backbone for the first %d epochs.",
                    config.freeze_backbone_epochs,
                )
                for p in hydra.shared_backbone.parameters():
                    p.requires_grad = False

        elif epoch == config.freeze_backbone_epochs:
            logger.info(
                "Unfreezing backbone at epoch %d. Joint learning begins.", epoch
            )
            for p in hydra.shared_backbone.parameters():
                p.requires_grad = True
            _freeze_dfl_parameters(hydra)

        interleaved.set_epoch(epoch)
        logger.info("epoch %d/%d", epoch + 1, config.epochs)
        global_step, train_metrics = _train_one_epoch(
            hydra=hydra,
            weighter=weighter,
            interleaved=interleaved,
            criteria=criteria,
            optimizers=optimizers,
            scaler=scaler,
            ema=ema,
            tasks=tasks,
            device=device,
            config=config,
            epoch=epoch,
            global_step=global_step,
            batch_size=batch,
        )
        _log_wandb_epoch(
            epoch=epoch,
            global_step=global_step,
            optimizers=optimizers,
            weighter=weighter,
            tasks=tasks,
            train_metrics=train_metrics,
            wandb_run=wandb_run,
        )

        epoch_update(criteria)
        for opt, sched in zip(optimizers.all(), schedulers, strict=True):
            backbone_frozen = epoch < config.freeze_backbone_epochs
            if opt is optimizers.backbone and backbone_frozen:
                continue
            sched.step()

        skip_val = (
            epoch % config.val_interval != 0 and epoch != config.epochs - 1
        )
        if skip_val:
            logger.info("epoch %d: validation skipped (val_interval)", epoch)
            continue

        validation_target = (
            ema if ema is not None else _ema_passthrough(hydra, weighter)
        )
        score, _per_task = _validate_epoch(
            epoch=epoch,
            global_step=global_step,
            validation_target=validation_target,
            hydra_model=hydra_model,
            datasets_per_task=datasets_per_task,
            head_source_paths=head_source_paths,
            run_dir=run_dir,
            imgsz=imgsz,
            batch=batch,
            device=device,
            config=config,
            wandb_run=wandb_run,
        )

        for task in tasks:
            write_per_task_checkpoint(
                ema=validation_target,
                hydra_model=HydraModelName(
                    backbone=hydra_model.backbone,
                    heads=[
                        h for h in hydra_model.heads if h.task_type() == task
                    ],
                    number_of_frozen_modules=hydra_model.number_of_frozen_modules,
                ),
                task=task,
                head_yolo_path=head_source_paths[task],
                output_path=run_dir / str(task) / "last.pt",
            )

        if patience.update(score):
            for task in tasks:
                src = run_dir / str(task) / "last.pt"
                dst = run_dir / str(task) / "best.pt"
                _atomic_copy(src, dst)

        write_joint_state(
            run_dir / "joint_state.json",
            epoch=epoch,
            best_score=patience.best,
            optimizers=optimizers,
            schedulers=schedulers,
            ema=validation_target,
            weighter=weighter,
            rng_state=_rng_state(),
        )

        if patience.should_stop():
            logger.info("early stopping triggered at epoch %d", epoch)
            break


def _step_all_optimizers(
    *,
    optimizers: JointOptimizers,
    scaler: torch.amp.GradScaler,
    hydra: Hydra,
    config: JointTrainConfig,
    backbone_grad_scale: float = 1.0,
) -> None:
    """Unscale, clip, and step all optimizers, then update the GradScaler.

    GradScaler.step() asserts that unscale_() recorded at least one gradient.
    Optimizers whose parameter groups all have grad=None are skipped because
    stepping them can still advance optimizer-specific state.
    """
    with_grads = {
        id(o): any(
            p.grad is not None for g in o.param_groups for p in g["params"]
        )
        for o in optimizers.all()
    }
    for o in optimizers.all():
        if with_grads[id(o)]:
            scaler.unscale_(o)
    if backbone_grad_scale != 1.0:
        _scale_gradients(
            hydra.shared_backbone.parameters(), scale=backbone_grad_scale
        )
    clip_grad_norm_(
        hydra.shared_backbone.parameters(), max_norm=config.max_grad_norm
    )
    if config.clip_heads:
        for o in optimizers.heads.values():
            clip_grad_norm_(
                list(parameters_in_optimizer(o)),
                max_norm=config.max_grad_norm,
            )
    for o in optimizers.all():
        if with_grads[id(o)]:
            scaler.step(o)
    scaler.update()


def _scale_gradients(
    parameters: Iterable[torch.Tensor],
    *,
    scale: float,
) -> None:
    for p in parameters:
        if p.grad is not None:
            p.grad.mul_(scale)


def _epoch_rounds(
    interleaved: InterleavedTaskDataloader,
    config: JointTrainConfig,
) -> int | None:
    """Total synchronized task rounds in one epoch.

    Capped by ``max_steps_per_epoch`` when set.
    """
    n = len(interleaved) if hasattr(interleaved, "__len__") else None
    if config.max_steps_per_epoch is not None:
        return (
            min(n, config.max_steps_per_epoch)
            if n is not None
            else config.max_steps_per_epoch
        )
    return n


def _accumulate_rounds(config: JointTrainConfig, batch_size: int) -> int:
    """Match Ultralytics' nominal-batch gradient accumulation heuristic."""
    if config.nominal_batch_size <= 0:
        return 1
    return max(round(config.nominal_batch_size / batch_size), 1)


def _cast_to_fp32(
    pred: torch.Tensor | list[Any] | tuple[Any, ...] | dict[str, Any],
) -> torch.Tensor | list[Any] | tuple[Any, ...] | dict[str, Any]:
    """Recursively cast tensors to fp32."""
    if isinstance(pred, torch.Tensor):
        return pred.float()
    if isinstance(pred, (list, tuple)):
        return type(pred)(_cast_to_fp32(p) for p in pred)
    if isinstance(pred, dict):
        return {k: _cast_to_fp32(v) for k, v in pred.items()}
    return pred


@dataclass
class _EpochLossAccumulator:
    counts: dict[TaskType, int]
    raw: dict[TaskType, float]
    per_image: dict[TaskType, float]
    per_branch: dict[TaskType, float]
    decomposed: dict[TaskType, dict[str, float]]
    decomposed_counts: dict[TaskType, dict[str, int]]

    @classmethod
    def for_tasks(cls, tasks: list[TaskType]) -> _EpochLossAccumulator:
        return cls(
            counts=dict.fromkeys(tasks, 0),
            raw=dict.fromkeys(tasks, 0.0),
            per_image=dict.fromkeys(tasks, 0.0),
            per_branch=dict.fromkeys(tasks, 0.0),
            decomposed={task: {} for task in tasks},
            decomposed_counts={task: {} for task in tasks},
        )

    def update(
        self,
        task: TaskType,
        *,
        raw: float,
        per_image: float,
        per_branch: float,
        decomposed: dict[str, float],
    ) -> None:
        self.counts[task] += 1
        self.raw[task] += raw
        self.per_image[task] += per_image
        self.per_branch[task] += per_branch
        for name, value in decomposed.items():
            self.decomposed[task][name] = (
                self.decomposed[task].get(name, 0.0) + value
            )
            self.decomposed_counts[task][name] = (
                self.decomposed_counts[task].get(name, 0) + 1
            )

    def as_metrics(self) -> dict[str, float]:
        metrics: dict[str, float] = {}
        for task, count in self.counts.items():
            if count == 0:
                continue
            metrics[f"loss/{task}"] = self.per_image[task] / count
            metrics[f"loss/{task}/per_image"] = self.per_image[task] / count
            metrics[f"loss/{task}/per_image_per_branch"] = (
                self.per_branch[task] / count
            )
            metrics[f"loss/{task}/raw_batch_scaled"] = self.raw[task] / count
            for name, total in self.decomposed[task].items():
                denom = self.decomposed_counts[task][name]
                metrics[f"loss/{task}/{name}"] = total / denom
        return metrics


def _train_one_epoch(
    *,
    hydra: Hydra,
    weighter: UncertaintyWeighter,
    interleaved: InterleavedTaskDataloader,
    criteria: dict[TaskType, Any],
    optimizers: JointOptimizers,
    scaler: torch.amp.GradScaler,
    ema: EMAHydra | None,
    tasks: list[TaskType],
    device: torch.device,
    config: JointTrainConfig,
    epoch: int,
    global_step: int,
    batch_size: int,
) -> tuple[int, dict[str, float]]:
    hydra.train()
    weighter.train()

    round_step = 0
    optimizer_steps = 0
    n_rounds = _epoch_rounds(interleaved, config)
    accumulate = _accumulate_rounds(config, batch_size)
    loss_accumulator = _EpochLossAccumulator.for_tasks(tasks)
    logger.info(
        "epoch %d: gradient accumulation=%d synchronized rounds",
        epoch,
        accumulate,
    )

    current_round_loss_per_image: dict[TaskType, float] = {}
    with TQDM(
        total=n_rounds, desc=f"  epoch {epoch + 1}", unit="round"
    ) as pbar:
        for task_step, (task, batch) in enumerate(interleaved):
            starts_round = task_step % len(tasks) == 0
            if starts_round:
                current_round_loss_per_image = {}
                if round_step % accumulate == 0:
                    for opt in optimizers.all():
                        opt.zero_grad(set_to_none=True)

            batch_on_device = _move_batch_to_device(batch, device)

            with torch.amp.autocast(
                device_type=device.type,
                dtype=torch.float16,
                enabled=config.use_amp and device.type == "cuda",
            ):
                feat, y_backbone = hydra.run_backbone(batch_on_device["img"])
                pred = hydra.run_head(str(task), feat, y_backbone)
            # Loss outside autocast (fp32): PoseLoss26 uses a normalizing-flow
            # (RLE) whose MultivariateNormal.log_prob produces NaN in fp16.
            loss_vector, _components = criteria[task](
                _cast_to_fp32(pred), batch_on_device
            )
            loss_total = loss_vector.sum()
            actual_batch_size = batch_on_device["img"].shape[0]
            loss_per_image = loss_total / actual_batch_size
            branch_count = 2 if hydra.head_end2end.get(str(task), False) else 1
            loss_per_branch = loss_per_image / branch_count
            weighted = weighter.weight_single(task, loss_total)

            scaler.scale(weighted).backward()
            current_round_loss_per_image[task] = loss_per_image.detach().item()

            loss_list = (
                _components.detach().cpu().tolist()
                if _components.ndim > 0
                else [_components.item()]
            )
            loss_names = _task_loss_names(task, len(loss_list))
            loss_accumulator.update(
                task,
                raw=loss_total.detach().item(),
                per_image=loss_per_image.detach().item(),
                per_branch=loss_per_branch.detach().item(),
                decomposed=dict(zip(loss_names, loss_list, strict=True)),
            )

            if (task_step + 1) % len(tasks) == 0:
                round_step += 1
                is_last_round = n_rounds is not None and round_step >= n_rounds
                reached_cap = (
                    config.max_steps_per_epoch is not None
                    and round_step >= config.max_steps_per_epoch
                )
                should_step = (
                    round_step % accumulate == 0
                    or is_last_round
                    or reached_cap
                )

                if should_step:
                    _step_all_optimizers(
                        optimizers=optimizers,
                        scaler=scaler,
                        hydra=hydra,
                        config=config,
                        backbone_grad_scale=1.0 / len(tasks),
                    )
                    optimizer_steps += 1
                    if ema is not None:
                        ema.update(hydra, weighter)

                mem = (
                    f"{torch.cuda.memory_reserved() / 1e9:.3g}G"
                    if device.type == "cuda"
                    else ""
                )
                pbar.set_postfix(
                    mem=mem,
                    **{
                        str(t): f"{current_round_loss_per_image[t]:.3f}"
                        for t in tasks
                        if t in current_round_loss_per_image
                    },
                )
                pbar.update(1)

                if reached_cap:
                    break

    train_metrics = loss_accumulator.as_metrics()
    train_metrics["train/rounds"] = float(round_step)
    train_metrics["train/optimizer_steps"] = float(optimizer_steps)
    if device.type == "cuda":
        train_metrics["train/mem_reserved_gb"] = (
            torch.cuda.memory_reserved() / 1e9
        )
    return global_step + optimizer_steps, train_metrics


def _task_loss_names(task: TaskType, length: int) -> list[str]:
    """Helper to return standard names for task loss components.

    Checks lengths match for standard YOLO loss patterns.
    """
    if task == TaskType.OBJECT and length == 3:
        return ["box_loss", "cls_loss", "dfl_loss"]
    if task == TaskType.SEGMENTATION and length == 5:
        return [
            "box_loss",
            "seg_loss",
            "cls_loss",
            "dfl_loss",
            "semseg_loss",
        ]
    if task == TaskType.POSE and length == 5:
        return ["box_loss", "kpt_loss", "kobj_loss", "cls_loss", "dfl_loss"]
    if task == TaskType.POSE and length == 6:
        return [
            "box_loss",
            "kpt_loss",
            "kobj_loss",
            "cls_loss",
            "dfl_loss",
            "rle_loss",
        ]
    return [f"loss_comp_{i}" for i in range(length)]


def _log_wandb_epoch(
    *,
    epoch: int,
    global_step: int,
    optimizers: JointOptimizers,
    weighter: UncertaintyWeighter,
    tasks: list[TaskType],
    train_metrics: dict[str, float],
    wandb_run: Any,
) -> None:
    """Emit one training metrics payload per epoch."""
    summary = {
        key: round(value, 4)
        for key, value in train_metrics.items()
        if key.startswith("loss/") and key.count("/") == 1
    }
    logger.info("epoch %d train metrics: %s", epoch, summary)

    if wandb_run is None:
        return

    lr_log = {
        "lr/backbone": optimizers.backbone.param_groups[0]["lr"],
        "lr/logvar": optimizers.log_var.param_groups[0]["lr"],
        **{
            f"lr/heads_{t}": opt.param_groups[0]["lr"]
            for t, opt in optimizers.heads.items()
        },
    }
    logvar_log = {
        f"logvar/{t}": weighter.log_var[weighter.tasks.index(t)].item()
        for t in tasks
    }
    wandb_run.log(
        {
            "epoch": epoch,
            "global_step": global_step,
            **lr_log,
            **train_metrics,
            **logvar_log,
        },
    )


def _move_batch_to_device(
    batch: dict[str, Any], device: torch.device
) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for k, v in batch.items():
        if isinstance(v, torch.Tensor):
            if k == "img":
                out[k] = v.to(device, non_blocking=True).float() / 255.0
            else:
                out[k] = v.to(device, non_blocking=True)
        else:
            out[k] = v
    return out


def _atomic_copy(src: Path, dst: Path) -> None:
    if not src.exists():
        logger.warning("source %s missing; skipping atomic copy", src)
        return
    tmp = dst.with_suffix(dst.suffix + ".tmp")
    tmp.write_bytes(src.read_bytes())
    tmp.replace(dst)


def _rng_state() -> dict[str, Any]:
    return {
        "python": random.getstate(),
        "numpy": np.random.get_state(),  # noqa: NPY002
        "torch": torch.get_rng_state(),
        "torch_cuda": (
            torch.cuda.get_rng_state_all() if torch.cuda.is_available() else []
        ),
    }


class _PassthroughEMA:
    """Stand-in EMA exposing the live model + weighter (no-op decay)."""

    def __init__(self, hydra: Hydra, weighter: UncertaintyWeighter) -> None:
        self.hydra = hydra
        self.weighter = weighter
        self.updates = 0

    def state_dict(self) -> dict[str, Any]:
        return {
            "updates": 0,
            "hydra": self.hydra.state_dict(),
            "weighter": self.weighter.state_dict(),
        }


def _ema_passthrough(hydra: Hydra, weighter: UncertaintyWeighter) -> Any:
    return _PassthroughEMA(hydra, weighter)


def _validate_epoch(
    *,
    epoch: int,
    global_step: int,
    validation_target: Any,
    hydra_model: HydraModelName,
    datasets_per_task: Mapping[TaskType, Path],
    head_source_paths: Mapping[TaskType, Path],
    run_dir: Path,
    imgsz: int,
    batch: int,
    device: torch.device | str,
    config: JointTrainConfig,
    wandb_run: Any,
) -> tuple[float, dict[TaskType, float]]:
    """Validate model checkpoints and upload metrics/plots to W&B."""
    score, per_task, all_metrics, task_visuals = run_validation(
        ema=validation_target,
        hydra_model=hydra_model,
        datasets_per_task=datasets_per_task,
        head_source_paths=head_source_paths,
        run_dir=run_dir,
        imgsz=imgsz,
        batch=batch,
        device=device,
        task_weights=config.task_weights,
    )
    logger.info("epoch %d: score=%.4f per_task=%s", epoch, score, per_task)
    if wandb_run is not None:
        val_metrics_log: dict[str, Any] = {
            "val/score": score,
            **{f"val/{t}": v for t, v in per_task.items()},
            "epoch": epoch,
            "global_step": global_step,
        }
        for task, metrics in all_metrics.items():
            for metric_name, val in metrics.items():
                val_metrics_log[f"val/{task}/{metric_name}"] = val

        for task, paths in task_visuals.items():
            for path in paths:
                asset_name = f"val/{task}/{path.stem}"
                try:
                    val_metrics_log[asset_name] = wandb.Image(
                        str(path), caption=path.name
                    )
                except Exception:
                    logger.exception(
                        "Failed to convert %s to wandb.Image", path
                    )

        wandb_run.log(val_metrics_log)
    return score, per_task
