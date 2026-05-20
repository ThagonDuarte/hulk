"""Custom-loop joint trainer for multi-task Hydra models.

Owns synchronized gradient accumulation, AMP, gradient clipping, EMA update,
scheduler stepping, and the per-epoch validation/checkpoint flow.
"""

from __future__ import annotations

import logging
import random
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch
import wandb
from torch.nn.utils import clip_grad_norm_
from ultralytics.utils import TQDM

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
    warmup_epochs: int = 10
    val_interval: int = 1
    log_interval: int = 50
    optimizer_name: str = "MuSGD"
    lr_backbone: float = 0.001
    lr_heads: float = 0.01
    lr_logvar: float = 0.001
    momentum: float = 0.9
    weight_decay: float = 1e-5
    max_grad_norm: float = 10.0
    use_amp: bool = True
    use_ema: bool = True
    clip_heads: bool = True
    init_log_var: dict[TaskType, float] = field(default_factory=dict)
    task_weights: dict[TaskType, float] = field(default_factory=dict)
    hyp: JointLossHyp = field(default_factory=JointLossHyp)
    max_steps_per_epoch: int | None = None  # cap steps; useful for smoke tests


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


def train_joint(
    *,
    hydra: Hydra,
    hydra_model: HydraModelName,
    interleaved: InterleavedTaskDataloader,
    datasets_per_task: Mapping[TaskType, Path],
    head_source_paths: Mapping[TaskType, Path],
    run_dir: Path,
    runs_dir: Path,
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
    # Ensure all parameters are trainable; YOLO checkpoints may load some
    # layers with requires_grad=False from single-task fine-tuning.
    for p in hydra.parameters():
        p.requires_grad_(requires_grad=True)
    tasks: list[TaskType] = sorted(
        (TaskType(k) for k in hydra.heads), key=lambda t: t.value
    )
    weighter = UncertaintyWeighter(tasks, init_log_var=config.init_log_var)
    weighter.to(device)

    optimizers = build_joint_optimizers(
        hydra,
        weighter,
        optimizer_name=config.optimizer_name,
        lr_backbone=config.lr_backbone,
        lr_heads=config.lr_heads,
        lr_logvar=config.lr_logvar,
        momentum=config.momentum,
        weight_decay=config.weight_decay,
    )
    schedulers = build_schedulers(
        optimizers, epochs=config.epochs, warmup_epochs=config.warmup_epochs
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
        interleaved.set_epoch(epoch)
        logger.info("epoch %d/%d", epoch + 1, config.epochs)
        global_step = _train_one_epoch(
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
            wandb_run=wandb_run,
            global_step=global_step,
        )

        epoch_update(criteria)
        for sched in schedulers:
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
        score, per_task = run_validation(
            ema=validation_target,
            hydra_model=hydra_model,
            datasets_per_task=datasets_per_task,
            head_source_paths=head_source_paths,
            runs_dir=runs_dir,
            imgsz=imgsz,
            batch=batch,
            device=device_str,
            task_weights=config.task_weights,
        )
        logger.info("epoch %d: score=%.4f per_task=%s", epoch, score, per_task)
        if wandb_run is not None:
            wandb.log(
                {
                    "val/score": score,
                    **{f"val/{t}": v for t, v in per_task.items()},
                    "epoch": epoch,
                },
                step=global_step,
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


def _epoch_rounds(
    interleaved: InterleavedTaskDataloader,
    config: JointTrainConfig,
) -> int | None:
    """Total full gradient-update rounds in one epoch.

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


def _cast_to_fp32(
    pred: torch.Tensor | list[Any] | tuple[Any, ...],
) -> torch.Tensor | list[Any] | tuple[Any, ...]:
    """Recursively cast tensors to fp32.

    PoseLoss26 uses a normalizing-flow (RLE) whose MultivariateNormal.log_prob
    is numerically unstable in fp16; running the loss at full precision avoids
    NaN regardless of AMP setting.
    """
    if isinstance(pred, torch.Tensor):
        return pred.float()
    if isinstance(pred, (list, tuple)):
        return type(pred)(_cast_to_fp32(p) for p in pred)
    return pred


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
    wandb_run: Any | None,
    global_step: int,
) -> int:
    hydra.train()
    weighter.train()

    step = 0
    n_rounds = _epoch_rounds(interleaved, config)
    with TQDM(total=n_rounds, desc=f"  epoch {epoch + 1}", unit="step") as pbar:
        for task, batch in interleaved:
            if step % len(tasks) == 0:
                for opt in optimizers.all():
                    opt.zero_grad(set_to_none=True)
                per_task_losses: dict[TaskType, torch.Tensor] = {}

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
            weighted = weighter.weight_single(task, loss_total) / len(tasks)

            scaler.scale(weighted).backward()
            per_task_losses[task] = loss_total.detach()

            step += 1
            if step % len(tasks) == 0:
                _step_all_optimizers(
                    optimizers=optimizers,
                    scaler=scaler,
                    hydra=hydra,
                    config=config,
                )
                if ema is not None:
                    ema.update(hydra, weighter)

                mem = (
                    f"{torch.cuda.memory_reserved() / 1e9:.3g}G"
                    if device.type == "cuda"
                    else ""
                )
                task_losses = {
                    str(t): f"{per_task_losses[t].item():.3f}" for t in tasks
                }
                pbar.set_postfix(mem=mem, **task_losses)
                pbar.update(1)

                _log_wandb_step(
                    step=step // len(tasks),
                    global_step=global_step + step // len(tasks),
                    epoch=epoch,
                    config=config,
                    optimizers=optimizers,
                    weighter=weighter,
                    tasks=tasks,
                    per_task_losses=per_task_losses,
                    wandb_run=wandb_run,
                )

                if (
                    config.max_steps_per_epoch is not None
                    and step // len(tasks) >= config.max_steps_per_epoch
                ):
                    break
    return global_step + step // len(tasks)


def _log_wandb_step(
    *,
    step: int,
    global_step: int,
    epoch: int,
    config: JointTrainConfig,
    optimizers: JointOptimizers,
    weighter: UncertaintyWeighter,
    tasks: list[TaskType],
    per_task_losses: dict[TaskType, torch.Tensor],
    wandb_run: Any,
) -> None:
    """Emit per-step metrics to W&B if interval matches and run is active."""
    if step % config.log_interval != 0 or not wandb_run:
        return
    lr_log = {
        "lr/backbone": optimizers.backbone.param_groups[0]["lr"],
        "lr/logvar": optimizers.log_var.param_groups[0]["lr"],
        **{
            f"lr/heads_{t}": opt.param_groups[0]["lr"]
            for t, opt in optimizers.heads.items()
        },
    }
    losses_log = {f"loss/{t}": v.item() for t, v in per_task_losses.items()}
    logvar_log = {
        f"logvar/{t}": weighter.log_var[weighter.tasks.index(t)].item()
        for t in tasks
    }
    wandb.log(
        {
            "epoch": epoch,
            "step": step,
            **lr_log,
            **losses_log,
            **logvar_log,
        },
        step=global_step,
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
