from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
from typing import Any, cast

from ultralytics.cfg import get_cfg
from ultralytics.data import build_dataloader, build_yolo_dataset
from ultralytics.data.build import InfiniteDataLoader
from ultralytics.data.utils import check_det_dataset
from ultralytics.utils import IterableSimpleNamespace

from model.joint_config import JointTaskConfig, TaskName


class EmptyTaskDataloadersError(ValueError):
    def __init__(self) -> None:
        super().__init__("At least one dataloader is required")


class InvalidTaskOrderError(ValueError):
    def __init__(self) -> None:
        super().__init__(
            "task_order must contain exactly all dataloader task names"
        )


@dataclass(frozen=True)
class TaskLoaderBundle:
    task_name: TaskName
    task_config: JointTaskConfig
    data: dict[str, Any]
    dataloader: InfiniteDataLoader


def build_task_train_loader(
    task_name: TaskName,
    task_config: JointTaskConfig,
    *,
    fraction: float,
    stride: int,
) -> TaskLoaderBundle:
    data = check_det_dataset(str(task_config.data_yaml), autodownload=False)
    cfg = cast(
        IterableSimpleNamespace,
        get_cfg(
            overrides={
                "task": task_config.task,
                "imgsz": task_config.imgsz,
                "workers": task_config.workers,
                "fraction": fraction,
                "batch": task_config.batch,
                "data": str(task_config.data_yaml),
                "rect": False,
            }
        ),
    )
    dataset = build_yolo_dataset(
        cfg=cfg,
        img_path=data["train"],
        batch=task_config.batch,
        data=data,
        mode="train",
        rect=False,
        stride=stride,
    )
    dataloader = build_dataloader(
        dataset=dataset,
        batch=task_config.batch,
        workers=task_config.workers,
        shuffle=True,
        rank=-1,
        drop_last=False,
    )
    return TaskLoaderBundle(
        task_name=task_name,
        task_config=task_config,
        data=data,
        dataloader=dataloader,
    )


class DynamicInterleavedLoader:
    def __init__(
        self,
        dataloaders: Mapping[TaskName, InfiniteDataLoader],
        *,
        task_order: Sequence[TaskName] | None = None,
    ) -> None:
        if not dataloaders:
            raise EmptyTaskDataloadersError
        self._dataloaders = dict(dataloaders)
        self._task_order = (
            list(task_order) if task_order is not None else list(dataloaders)
        )
        if set(self._task_order) != set(self._dataloaders):
            raise InvalidTaskOrderError

    def __len__(self) -> int:
        return max(len(loader) for loader in self._dataloaders.values())

    def iter_epoch(self) -> Iterator[dict[TaskName, dict[str, Any]]]:
        loader_iters: dict[TaskName, Iterator[dict[str, Any]]] = {
            task_name: iter(loader)
            for task_name, loader in self._dataloaders.items()
        }
        max_steps = len(self)
        for _ in range(max_steps):
            step_batches: dict[TaskName, dict[str, Any]] = {}
            for task_name in self._task_order:
                step_batches[task_name] = self._next_task_batch(
                    task_name,
                    loader_iters,
                )
            yield step_batches

    def _next_task_batch(
        self,
        task_name: TaskName,
        loader_iters: dict[TaskName, Iterator[dict[str, Any]]],
    ) -> dict[str, Any]:
        try:
            return next(loader_iters[task_name])
        except StopIteration:
            loader_iters[task_name] = iter(self._dataloaders[task_name])
            return next(loader_iters[task_name])
