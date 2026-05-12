"""Loss criterion adapters and dispatcher for joint Hydra training.

Ultralytics' loss classes (`v8DetectionLoss`, `v8SegmentationLoss`,
`v8PoseLoss`, `PoseLoss26`, `E2ELoss`) read a small set of attributes off
the model they are constructed against - `model`, `stride`, `nc`, `args`,
`hyp`, `kpt_shape`, `device`, `end2end`. We don't have a single
``DetectionModel`` per task in joint training; we have one shared backbone
and N task heads. The adapter exposes exactly the attributes those losses
read, sourced from the appropriate slice of the Hydra instance.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any

import torch
from torch import nn
from ultralytics.utils.loss import (
    E2ELoss,
    PoseLoss26,
    v8DetectionLoss,
    v8PoseLoss,
    v8SegmentationLoss,
)

from model.hydra import Hydra
from utils.model_naming import TaskType


@dataclass
class JointLossHyp:
    """Loss-relevant hyperparameters consumed by Ultralytics losses."""

    box: float = 7.5
    cls: float = 0.5
    dfl: float = 1.5
    pose: float = 12.0
    kobj: float = 1.0
    rle: float = 1.0
    epochs: int = 100
    label_smoothing: float = 0.0
    fl_gamma: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)

    def as_namespace(self) -> SimpleNamespace:
        ns = SimpleNamespace(
            **{
                k: getattr(self, k)
                for k in (
                    "box",
                    "cls",
                    "dfl",
                    "pose",
                    "kobj",
                    "rle",
                    "epochs",
                    "label_smoothing",
                    "fl_gamma",
                )
            }
        )
        for k, v in self.extra.items():
            setattr(ns, k, v)
        return ns


class _HydraHeadAdapter(nn.Module):
    """Wraps a `Hydra` slice as the model surface Ultralytics losses expect."""

    def __init__(
        self,
        hydra: Hydra,
        task: TaskType,
        hyp: JointLossHyp,
    ) -> None:
        super().__init__()
        task_key = str(task)
        self._hydra = hydra
        self._task = task
        self.model = hydra.heads[task_key]
        self.stride = hydra.head_strides[task_key]
        names = hydra.head_class_names[task_key]
        self.nc = len(names) if hasattr(names, "__len__") else 80
        self.names = names
        self.kpt_shape = hydra.head_kpt_shapes.get(task_key)
        self.end2end = hydra.head_end2end[task_key]
        self.args = hyp.as_namespace()
        self.hyp = self.args

    @property
    def device(self) -> torch.device:
        return next(self._hydra.parameters()).device


# Inner loss class per task, used as the inner of E2ELoss when end2end=True.
# `PoseLoss26` is the YOLO26 RLE-capable pose loss and is only reachable
# through `E2ELoss` in stock Ultralytics.
_INNER_LOSS_E2E: dict[TaskType, type] = {
    TaskType.OBJECT: v8DetectionLoss,
    TaskType.POSE: PoseLoss26,
    TaskType.SEGMENTATION: v8SegmentationLoss,
}

# Non-end2end pose runs fall back to `v8PoseLoss` (no RLE).
_INNER_LOSS_NON_E2E: dict[TaskType, type] = {
    TaskType.OBJECT: v8DetectionLoss,
    TaskType.POSE: v8PoseLoss,
    TaskType.SEGMENTATION: v8SegmentationLoss,
}


def build_criterion(
    hydra: Hydra,
    task: TaskType,
    hyp: JointLossHyp,
) -> Callable[[Any, dict[str, Any]], tuple[torch.Tensor, torch.Tensor]]:
    """Construct the loss callable for a given task on a Hydra instance.

    Returns the loss object directly; Ultralytics' losses are callables that
    accept ``(preds, batch)`` and return ``(total_loss, loss_components)``.
    """
    adapter = _HydraHeadAdapter(hydra, task, hyp)
    if adapter.end2end:
        return E2ELoss(adapter, _INNER_LOSS_E2E[task])
    return _INNER_LOSS_NON_E2E[task](adapter)


def epoch_update(
    criteria: dict[TaskType, Callable[..., Any]],
) -> None:
    """Advance per-epoch state on criteria that support it (E2ELoss decay)."""
    for criterion in criteria.values():
        update = getattr(criterion, "update", None)
        if callable(update):
            update()
