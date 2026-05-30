"""Multi-task dataloader plumbing for joint training.

Two pieces:

* `build_task_dataloader(...)` - wraps `ultralytics.data.build` to produce a
  standard YOLO dataloader for a single task YAML. (Implemented in Task 5.)
* `InterleavedTaskDataloader` - round-robin iterator over per-task loaders.
  An "epoch" is `max(len(loader))` steps; shorter loaders silently restart.
"""

from __future__ import annotations

import logging
from collections.abc import Iterable, Iterator, Mapping, Sized
from pathlib import Path
from typing import Any, Protocol, cast

from ultralytics.cfg import get_cfg
from ultralytics.data.build import build_dataloader, build_yolo_dataset
from ultralytics.data.utils import check_det_dataset
from ultralytics.utils import DEFAULT_CFG_DICT, IterableSimpleNamespace

from utils.model_naming import TaskType

logger = logging.getLogger(__name__)


class _SizedIterable(Sized, Iterable[Any], Protocol):
    """Type protocol covering the `__iter__` + `__len__` surface we need."""


class InterleavedTaskDataloader:
    """Round-robin iterator over per-task dataloaders.

    Iteration yields `(task, batch)` pairs, with one batch per task per step.
    Tasks are visited in ``sorted(loaders.keys(), key=lambda t: t.value)``
    order each step (alphabetical-by-value, deterministic).

    `len(self)` is `max(len(loader) for loader in loaders.values())`, the
    number of steps per epoch. Shorter underlying loaders restart silently
    via cycle semantics; each restart is logged at INFO so it shows up in
    W&B run logs.
    """

    def __init__(
        self,
        loaders: Mapping[TaskType, _SizedIterable],
        epoch_size_strategy: str | int = "max",
    ) -> None:
        if not loaders:
            raise ValueError(  # noqa: TRY003
                "InterleavedTaskDataloader requires at least one loader"
            )
        self._loaders = dict(loaders)
        self._sorted_tasks: list[TaskType] = sorted(
            self._loaders.keys(), key=lambda t: t.value
        )
        if isinstance(epoch_size_strategy, int):
            if epoch_size_strategy <= 0:
                raise ValueError(  # noqa: TRY003
                    "epoch_size_strategy as an integer must be positive"
                )
            self._steps_per_epoch = epoch_size_strategy
        elif epoch_size_strategy == "max":
            self._steps_per_epoch = max(
                len(self._loaders[t]) for t in self._sorted_tasks
            )
        elif epoch_size_strategy == "min":
            self._steps_per_epoch = min(
                len(self._loaders[t]) for t in self._sorted_tasks
            )
        else:
            raise ValueError(  # noqa: TRY003
                f"unsupported epoch_size_strategy: {epoch_size_strategy!r}"
            )

    def __len__(self) -> int:
        return self._steps_per_epoch

    def __iter__(self) -> Iterator[tuple[TaskType, Any]]:
        cycles = {
            task: _CyclingIterator(task, self._loaders[task])
            for task in self._sorted_tasks
        }
        for _ in range(self._steps_per_epoch):
            for task in self._sorted_tasks:
                yield task, next(cycles[task])

    def set_epoch(self, epoch: int) -> None:
        for loader in self._loaders.values():
            set_epoch = getattr(loader, "set_epoch", None)
            if callable(set_epoch):
                set_epoch(epoch)


class _CyclingIterator:
    """Iterator that restarts the underlying iterable on exhaustion."""

    def __init__(self, task: TaskType, loader: _SizedIterable) -> None:
        self._task = task
        self._loader = loader
        self._iter: Iterator[Any] = iter(loader)
        self._restarts = 0

    def __iter__(self) -> _CyclingIterator:
        return self

    def __next__(self) -> Any:
        try:
            return next(self._iter)
        except StopIteration:
            self._restarts += 1
            logger.info(
                "InterleavedTaskDataloader: restarting %s (restart #%d)",
                self._task,
                self._restarts,
            )
            self._iter = iter(self._loader)
            return next(self._iter)


# ---------------------------------------------------------------------------
# Per-task dataloader factory
# ---------------------------------------------------------------------------


def build_task_dataloader(
    task: TaskType,
    dataset_yaml: Path,
    *,
    imgsz: int,
    batch: int,
    workers: int,
    stride: int = 32,
    mode: str = "train",
    rect: bool = False,
    overrides: dict[str, Any] | None = None,
) -> tuple[Any, Any]:
    """Build a YOLO dataloader for a single task YAML.

    Returns ``(loader, dataset)``. The caller can read `len(loader)` for the
    step count and `len(dataset)` for diagnostics. The loader is the standard
    Ultralytics ``InfiniteDataLoader`` so iteration/restart semantics match
    `BaseTrainer`.
    """
    base_overrides: dict[str, Any] = {
        **DEFAULT_CFG_DICT,
        "task": _ultralytics_task_name(task),
        "imgsz": imgsz,
        "batch": batch,
        "workers": workers,
        "mode": mode,
        "rect": rect,
        "data": str(dataset_yaml),
    }
    if overrides:
        base_overrides.update(overrides)
    args = cast(IterableSimpleNamespace, get_cfg(overrides=base_overrides))

    data = check_det_dataset(str(dataset_yaml))
    img_path = (
        data["train"] if mode == "train" else data.get("val", data["train"])
    )

    dataset = build_yolo_dataset(
        args, img_path, batch, data, mode=mode, rect=rect, stride=stride
    )
    loader = build_dataloader(
        dataset, batch, workers, shuffle=(mode == "train"), rank=-1
    )
    return loader, dataset


_TASK_TO_ULTRALYTICS_NAME: dict[TaskType, str] = {
    TaskType.OBJECT: "detect",
    TaskType.POSE: "pose",
    TaskType.SEGMENTATION: "segment",
}


def _ultralytics_task_name(task: TaskType) -> str:
    return _TASK_TO_ULTRALYTICS_NAME[task]
