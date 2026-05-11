"""Multi-task dataloader plumbing for joint training.

Two pieces:

* `build_task_dataloader(...)` - wraps `ultralytics.data.build` to produce a
  standard YOLO dataloader for a single task YAML. (Implemented in Task 5.)
* `InterleavedTaskDataloader` - round-robin iterator over per-task loaders.
  An "epoch" is `max(len(loader))` steps; shorter loaders silently restart.
"""

from __future__ import annotations

import logging
from collections.abc import Iterable, Iterator, Sized
from typing import Any, Protocol

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

    def __init__(self, loaders: dict[TaskType, _SizedIterable]) -> None:
        if not loaders:
            raise ValueError(  # noqa: TRY003
                "InterleavedTaskDataloader requires at least one loader"
            )
        self._loaders = loaders
        self._sorted_tasks: list[TaskType] = sorted(
            loaders.keys(), key=lambda t: t.value
        )
        self._steps_per_epoch = max(len(loaders[t]) for t in self._sorted_tasks)

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
