"""Tests for the interleaved multi-task dataloader."""

from __future__ import annotations

from collections.abc import Iterator

import pytest

from model.joint_loop.dataloaders import InterleavedTaskDataloader
from utils.model_naming import TaskType


class _FakeLoader:
    """Minimal Sized-Iterable matching DataLoader's protocol surface."""

    def __init__(self, label: str, length: int) -> None:
        self.label = label
        self.length = length
        self.epoch_calls: list[int] = []

    def __iter__(self) -> Iterator[str]:
        for i in range(self.length):
            yield f"{self.label}{i}"

    def __len__(self) -> int:
        return self.length

    def set_epoch(self, epoch: int) -> None:
        self.epoch_calls.append(epoch)


def test_steps_per_epoch_is_max_loader_len() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 5),
    }
    iter_loader = InterleavedTaskDataloader(loaders)
    assert len(iter_loader) == 5


def test_epoch_rounds_matches_interleaved_steps_per_epoch() -> None:
    from model.joint_loop.loop import JointTrainConfig, _epoch_rounds

    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 5),
        TaskType.SEGMENTATION: _FakeLoader("s", 4),
    }
    iter_loader = InterleavedTaskDataloader(loaders)

    assert _epoch_rounds(iter_loader, JointTrainConfig()) == 5


def test_iteration_yields_one_batch_per_task_per_step() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 5),
    }
    iter_loader = InterleavedTaskDataloader(loaders)
    pairs = list(iter_loader)
    assert len(pairs) == 5 * 2

    # within each step, every task should have appeared exactly once
    for step in range(5):
        tasks_in_step = {pairs[step * 2 + j][0] for j in range(2)}
        assert tasks_in_step == {TaskType.OBJECT, TaskType.POSE}


def test_iteration_uses_sorted_task_order() -> None:
    loaders = {
        TaskType.POSE: _FakeLoader("p", 1),
        TaskType.OBJECT: _FakeLoader("o", 1),
    }
    iter_loader = InterleavedTaskDataloader(loaders)
    pairs = list(iter_loader)
    tasks_in_first_step = [task for task, _ in pairs]
    expected_sorted = sorted(
        [TaskType.POSE, TaskType.OBJECT], key=lambda t: t.value
    )
    assert tasks_in_first_step == expected_sorted


def test_shorter_loader_restarts_silently() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 2),
        TaskType.POSE: _FakeLoader("p", 5),
    }
    iter_loader = InterleavedTaskDataloader(loaders)
    pairs = list(iter_loader)
    object_batches = [b for task, b in pairs if task == TaskType.OBJECT]
    # OBJECT must yield 5 batches (== steps_per_epoch) cycling through "o0,o1"
    assert object_batches == ["o0", "o1", "o0", "o1", "o0"]


def test_set_epoch_propagates_to_all_loaders() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 1),
        TaskType.POSE: _FakeLoader("p", 1),
    }
    iter_loader = InterleavedTaskDataloader(loaders)
    iter_loader.set_epoch(7)
    assert loaders[TaskType.OBJECT].epoch_calls == [7]
    assert loaders[TaskType.POSE].epoch_calls == [7]


def test_set_epoch_tolerates_loaders_without_set_epoch() -> None:
    class _Plain:
        def __iter__(self) -> Iterator[str]:
            yield "x"

        def __len__(self) -> int:
            return 1

    loaders = {TaskType.OBJECT: _Plain()}
    iter_loader = InterleavedTaskDataloader(loaders)
    # Must not raise
    iter_loader.set_epoch(0)


def test_empty_loaders_raises() -> None:
    with pytest.raises(ValueError):
        InterleavedTaskDataloader({})


def test_epoch_size_strategy_max() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 7),
    }
    loader = InterleavedTaskDataloader(loaders, epoch_size_strategy="max")
    assert len(loader) == 7


def test_epoch_size_strategy_min() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 7),
    }
    loader = InterleavedTaskDataloader(loaders, epoch_size_strategy="min")
    assert len(loader) == 3


def test_epoch_size_strategy_integer() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
        TaskType.POSE: _FakeLoader("p", 7),
    }
    loader = InterleavedTaskDataloader(loaders, epoch_size_strategy=12)
    assert len(loader) == 12


def test_epoch_size_strategy_invalid() -> None:
    loaders = {
        TaskType.OBJECT: _FakeLoader("o", 3),
    }
    with pytest.raises(ValueError, match="unsupported epoch_size_strategy"):
        InterleavedTaskDataloader(loaders, epoch_size_strategy="invalid_strat")

    with pytest.raises(ValueError, match="must be positive"):
        InterleavedTaskDataloader(loaders, epoch_size_strategy=0)
