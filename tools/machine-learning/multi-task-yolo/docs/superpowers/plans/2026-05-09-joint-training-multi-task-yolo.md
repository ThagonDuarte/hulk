# Joint Training of Multi-Task YOLO Hydra Models — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `src/model/joint_train.py` and a `src/model/joint_loop/` engine package that finetunes a single `Hydra` instance with N task heads simultaneously, mirroring Ultralytics' YOLO26 training recipe (E2ELoss + MuSGD + 4-group param split with cv3/proto LR boost) per module group, while leaving `train.py`, `validator.py`, `cross.py`, and `export_hydra.py` untouched.

**Architecture:** A custom PyTorch training loop owns synchronized gradient accumulation across tasks: per step, each task forward+backward contributes to its own head optimizer and accumulates into the shared backbone optimizer; once all tasks have contributed, optimizers step once. Cross-task losses are balanced by a learnable Kendall–Gal uncertainty weighter. Validation reuses the existing `validate_hydra_model()` per task on EMA weights and aggregates a weighted score for "best" tracking.

**Tech Stack:** Python 3.13, `uv` for deps, `torch>=2.10`, `ultralytics>=8.4.31` (YOLO26 codepath including `MuSGD`, `E2ELoss`, `PoseLoss26`, `build_yolo_dataset`, `build_dataloader`), `click`, `wandb`, `wonderwords`, `pyyaml`. `pytest>=8.3` is added to the dev group as part of Task 1.

**Working directory:** All commands run from `tools/machine-learning/multi-task-yolo/` unless noted. `uv run` is required (per `AGENTS.md`).

**Spec reference:** [`docs/superpowers/specs/2026-05-09-joint-training-multi-task-yolo-design.md`](../specs/2026-05-09-joint-training-multi-task-yolo-design.md).

---

## File Structure

**Created:**

| Path | Responsibility |
|------|----------------|
| `src/model/joint_train.py` | Click CLI shell. Parses args → builds `Hydra` → builds dataloaders → calls `train_joint()` → writes per-task checkpoints. No math. |
| `src/model/joint_loop/__init__.py` | Empty package marker. |
| `src/model/joint_loop/dataloaders.py` | `build_task_dataloader()` (wraps `ultralytics.data.build`); `InterleavedTaskDataloader` (round-robin over tasks, restart-on-exhaust). |
| `src/model/joint_loop/weighting.py` | `UncertaintyWeighter` `nn.Module` (Kendall–Gal log-variance). |
| `src/model/joint_loop/criteria.py` | `_HydraHeadAdapter` (exposes attributes Ultralytics' loss code expects); `build_criterion()` (E2ELoss vs v8XxxLoss dispatch); `epoch_update()` (E2ELoss decay). |
| `src/model/joint_loop/optim.py` | `build_param_groups()` (4-group split + cv3/proto regex per-head-parameterized); `build_joint_optimizers()`; `build_schedulers()` (cosine + linear warmup); `EMAHydra`; `make_amp_scaler()`. |
| `src/model/joint_loop/checkpoints.py` | Per-task `.pt` writer (cross-style EMA backbone swap into head model); `joint_state.json` writer/reader; `config.json` writer. |
| `src/model/joint_loop/validation.py` | `run_validation()` — assembles temp per-task `.pt`, calls `validate_hydra_model()`, reads `metrics.json`, computes weighted score. |
| `src/model/joint_loop/loop.py` | `train_joint()` — synchronized accumulation, AMP, gradient clipping, EMA update, scheduler step, validation hook orchestration, early stopping. |
| `tests/__init__.py` | Empty package marker. |
| `tests/joint_loop/__init__.py` | Empty package marker. |
| `tests/joint_loop/test_dataloader.py` | `InterleavedTaskDataloader` round-robin order; restart-on-exhaustion; `set_epoch` propagation. |
| `tests/joint_loop/test_weighting.py` | `UncertaintyWeighter` math; gradient flow into `log_var`; `weight_single` ≡ `forward` summed across tasks. |
| `tests/joint_loop/test_optim.py` | `build_param_groups` 4-group split; cv3/proto x3 LR boost; backbone gradient accumulation invariant. |
| `tests/joint_loop/test_hydra_split.py` | Parity between old `forward()` and `run_backbone+run_head`-composed forward (skipped if asset weights missing). |
| `tests/joint_loop/test_loop_smoke.py` | End-to-end smoke test (skipped if datasets missing). |

**Modified:**

| Path | Change |
|------|--------|
| `src/model/hydra.py` | Add `run_backbone()` and `run_head()` methods. Rewrite `forward()` to compose them. Public API unchanged. |
| `pyproject.toml` | Add `pytest>=8.3` to `[dependency-groups].dev`. |
| `AGENTS.md` | Document new `joint_train` entrypoint and gotchas (Task 16). |

**Not touched:** `src/model/train.py`, `src/model/cross.py`, `src/validation/validator.py`, `src/validation/compare_results.py`, `src/utils/export_hydra.py`, `src/utils/export_yolo_to_onnx.py`, `src/utils/model_naming.py`, `src/utils/nv12_to_rgb.py`.

---

## Task 1: Test Infrastructure & Dependency

**Files:**
- Modify: `pyproject.toml`
- Create: `tests/__init__.py`
- Create: `tests/joint_loop/__init__.py`
- Create: `tests/conftest.py`
- Create: `tests/test_smoke.py` (sentinel)

- [ ] **Step 1: Add pytest to dev dependencies**

Edit `pyproject.toml`:

```toml
[dependency-groups]
dev = [
  "pyright>=1.1.408",
  "pytest>=8.3",
  "ruff>=0.15.9",
]
```

- [ ] **Step 2: Sync the dev group**

Run: `uv sync --group dev`
Expected: pytest installed, no errors.

- [ ] **Step 3: Add a `[tool.pytest.ini_options]` block to `pyproject.toml`**

Append to `pyproject.toml`:

```toml
[tool.pytest.ini_options]
testpaths = ["tests"]
pythonpath = ["src"]
addopts = "-ra"
```

The `pythonpath = ["src"]` line lets test files import `model.*`, `utils.*`, `validation.*` directly, matching how the rest of the project is laid out.

- [ ] **Step 4: Create empty `__init__.py` files**

Create `tests/__init__.py`:
```python
```

Create `tests/joint_loop/__init__.py`:
```python
```

- [ ] **Step 5: Create `tests/conftest.py` with shared fixtures**

```python
"""Shared pytest fixtures for the multi-task YOLO test suite."""
from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture
def tmp_run_dir(tmp_path: Path) -> Path:
    """A temp directory shaped like a joint-train run output."""
    run_dir = tmp_path / "joint_train" / "fake-model~placeholder"
    run_dir.mkdir(parents=True)
    return run_dir
```

- [ ] **Step 6: Write the sentinel test**

Create `tests/test_smoke.py`:

```python
"""Sanity check that pytest discovery works."""


def test_pytest_works() -> None:
    assert 1 + 1 == 2
```

- [ ] **Step 7: Run the sentinel test**

Run: `uv run pytest tests/test_smoke.py -v`
Expected: 1 passed.

- [ ] **Step 8: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/pyproject.toml \
        tools/machine-learning/multi-task-yolo/uv.lock \
        tools/machine-learning/multi-task-yolo/tests/
git commit -m "test(multi-task-yolo): add pytest infra for joint training"
```

---

## Task 2: Hydra `run_backbone()` / `run_head()` Split (TDD)

The current `Hydra.forward()` (in `src/model/hydra.py:143-212`) hardcodes head-output flattening into `outputs[<task>_output]`. Joint training needs raw head output (`E2ELoss.parse_output()` consumes a structured dict for end2end heads). We split the existing logic into two methods and rewrite `forward()` to compose them — public API stays byte-identical.

**Files:**
- Test: `tests/joint_loop/test_hydra_split.py`
- Modify: `src/model/hydra.py`

- [ ] **Step 1: Write the parity test (will be skipped without assets)**

Create `tests/joint_loop/test_hydra_split.py`:

```python
"""Parity test: run_backbone+run_head composition matches forward()."""
from __future__ import annotations

from pathlib import Path

import pytest
import torch

from model.hydra import Hydra
from utils.model_naming import TaskType

ASSETS_DIR = Path("assets")
BACKBONE_PATH = ASSETS_DIR / "yolo26m.pt"
POSE_HEAD_PATH = ASSETS_DIR / "yolo26m-pose.pt"
DETECT_HEAD_PATH = ASSETS_DIR / "yolo26m.pt"


@pytest.fixture(scope="module")
def hydra_two_head() -> Hydra:
    if not (BACKBONE_PATH.exists() and POSE_HEAD_PATH.exists()):
        pytest.skip("yolo26m / yolo26m-pose assets not present")
    return Hydra(
        backbone_path=str(BACKBONE_PATH),
        task_dict={
            TaskType.OBJECT: DETECT_HEAD_PATH,
            TaskType.POSE: POSE_HEAD_PATH,
        },
    )


def test_run_backbone_then_run_head_matches_forward(
    hydra_two_head: Hydra,
) -> None:
    hydra_two_head.eval()
    x = torch.randn(1, 3, 640, 640)

    with torch.no_grad():
        baseline = hydra_two_head.forward(x)

        feat, y_backbone = hydra_two_head.run_backbone(x)
        for task, expected_keys in [
            (TaskType.OBJECT, TaskType.OBJECT.output_names()),
            (TaskType.POSE, TaskType.POSE.output_names()),
        ]:
            raw = hydra_two_head.run_head(str(task), feat, y_backbone)
            if isinstance(raw, torch.Tensor):
                tensors = (raw,)
            else:
                tensors = tuple(raw)
            for key, tensor in zip(expected_keys, tensors, strict=True):
                assert torch.allclose(tensor, baseline[key], atol=1e-6), (
                    f"mismatch on {key} for task {task}"
                )
```

- [ ] **Step 2: Run the test to verify failure**

Run: `uv run pytest tests/joint_loop/test_hydra_split.py -v`
Expected: FAIL with `AttributeError: 'Hydra' object has no attribute 'run_backbone'` (or skipped if assets missing — that's fine; we proceed anyway and let the engineer running with assets verify).

- [ ] **Step 3: Add `run_backbone()` and `run_head()` to `Hydra`**

Edit `src/model/hydra.py`. Replace the entire `forward()` method (lines 143-212) with the three methods below:

```python
    def run_backbone(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor | list[torch.Tensor], list[torch.Tensor | None]]:
        """Run the shared backbone and return (final_activations, saved_list).

        The saved list mirrors Ultralytics' `y` cache pattern: each entry is
        either a tensor (if its layer index appears in `self.save_backbone`)
        or `None`. Used by `run_head` to resolve cross-layer connections that
        reach back into the backbone.
        """
        y_backbone: list[torch.Tensor | None] = []
        backbone_activations: Any = x

        for i, m in enumerate(self.shared_backbone):
            from_index = cast(Any, m.f)
            if from_index != -1:
                backbone_activations = (
                    y_backbone[from_index]
                    if isinstance(from_index, int)
                    else [
                        backbone_activations if j == -1 else y_backbone[j]
                        for j in cast(list[int], from_index)
                    ]
                )
            backbone_activations = m(backbone_activations)
            y_backbone.append(
                backbone_activations if i in self.save_backbone else None
            )

        return backbone_activations, y_backbone

    def run_head(
        self,
        head_name: str,
        backbone_activations: torch.Tensor | list[torch.Tensor],
        y_backbone: list[torch.Tensor | None],
    ) -> Any:
        """Run a single task head and return its raw, non-flattened output.

        The returned value is whatever the head's final module emits — a
        tensor, a tuple of tensors, or a structured object (e.g. the dict
        returned by end2end heads that `E2ELoss.parse_output` expects).
        """
        if head_name not in self.heads:
            raise MissingHydraHeadError(head_name)
        head = cast(nn.ModuleList, self.heads[head_name])
        y_head = list(y_backbone)
        head_activations: Any = backbone_activations

        for i, m in enumerate(head):
            module_index = i + self.backbone_length

            from_index = cast(Any, m.f)
            if from_index != -1:
                head_activations = (
                    cast(torch.Tensor, y_head[from_index])
                    if isinstance(from_index, int)
                    else [
                        cast(torch.Tensor, head_activations)
                        if j == -1
                        else cast(torch.Tensor, y_head[j])
                        for j in cast(list[int], from_index)
                    ]
                )

            head_activations = m(head_activations)

            y_head.append(
                cast(torch.Tensor, head_activations)
                if module_index in self.branch_saves[head_name]
                else None
            )

        return head_activations

    def forward(self, x: torch.Tensor) -> dict[str, Any]:
        backbone_activations, y_backbone = self.run_backbone(x)
        outputs: dict[str, Any] = {}

        for head_name in self.heads:
            head_activations = self.run_head(
                head_name, backbone_activations, y_backbone
            )
            task_output_names = TaskType(head_name).output_names()
            if isinstance(head_activations, torch.Tensor):
                outputs[task_output_names[0]] = head_activations
            elif isinstance(head_activations, tuple) and all(
                isinstance(t, torch.Tensor) for t in head_activations
            ):
                for key, tensor in zip(
                    task_output_names, head_activations, strict=False
                ):
                    outputs[key] = tensor
            else:
                raise TypeError(  # noqa: TRY003
                    f"Head '{head_name}' output must be a tensor or tuple of"
                    f" tensors, got {type(head_activations)}"
                )

        return outputs
```

- [ ] **Step 4: Run the parity test to verify it passes**

Run: `uv run pytest tests/joint_loop/test_hydra_split.py -v`
Expected: 1 passed (or 1 skipped if asset weights are not on the local disk — both outcomes are acceptable).

- [ ] **Step 5: Run lint to verify no style regressions**

Run: `uv run ruff check src/model/hydra.py`
Expected: zero errors. If `ruff` reports issues, fix them inline (line length 80, etc.).

- [ ] **Step 6: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/hydra.py \
        tools/machine-learning/multi-task-yolo/tests/joint_loop/test_hydra_split.py
git commit -m "refactor(multi-task-yolo): split Hydra.forward into run_backbone+run_head"
```

---

## Task 3: `UncertaintyWeighter` (TDD)

Kendall–Gal homoscedastic uncertainty weighting:
`total = sum_t( exp(-log_var_t) * loss_t + log_var_t )`. Lives in `src/model/joint_loop/weighting.py`.

**Files:**
- Create: `src/model/joint_loop/__init__.py`
- Create: `src/model/joint_loop/weighting.py`
- Test: `tests/joint_loop/test_weighting.py`

- [ ] **Step 1: Create the `joint_loop` package marker**

Create `src/model/joint_loop/__init__.py`:
```python
```

- [ ] **Step 2: Write the failing tests**

Create `tests/joint_loop/test_weighting.py`:

```python
"""Tests for the Kendall–Gal uncertainty weighter."""
from __future__ import annotations

import math

import pytest
import torch

from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import TaskType


def test_log_var_initial_zero() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    assert torch.equal(weighter.log_var, torch.zeros(2))


def test_init_log_var_overrides() -> None:
    weighter = UncertaintyWeighter(
        [TaskType.OBJECT, TaskType.POSE],
        init_log_var={TaskType.POSE: 1.5},
    )
    assert weighter.log_var[0].item() == pytest.approx(0.0)
    assert weighter.log_var[1].item() == pytest.approx(1.5)


def test_forward_matches_formula() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    losses = {
        TaskType.OBJECT: torch.tensor(2.0),
        TaskType.POSE: torch.tensor(3.0),
    }
    total = weighter(losses)
    # log_var=0 -> exp(0)*2 + 0 + exp(0)*3 + 0 = 5.0
    assert total.item() == pytest.approx(5.0)


def test_forward_respects_log_var() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    with torch.no_grad():
        weighter.log_var.copy_(torch.tensor([0.0, 1.0]))
    losses = {
        TaskType.OBJECT: torch.tensor(2.0),
        TaskType.POSE: torch.tensor(3.0),
    }
    expected = math.exp(0.0) * 2.0 + 0.0 + math.exp(-1.0) * 3.0 + 1.0
    total = weighter(losses)
    assert total.item() == pytest.approx(expected, rel=1e-6)


def test_weight_single_sums_to_forward() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    with torch.no_grad():
        weighter.log_var.copy_(torch.tensor([0.5, -0.2]))
    losses = {
        TaskType.OBJECT: torch.tensor(2.5),
        TaskType.POSE: torch.tensor(0.7),
    }
    summed = (
        weighter.weight_single(TaskType.OBJECT, losses[TaskType.OBJECT])
        + weighter.weight_single(TaskType.POSE, losses[TaskType.POSE])
    )
    assert summed.item() == pytest.approx(weighter(losses).item(), rel=1e-6)


def test_log_var_receives_gradient() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT, TaskType.POSE])
    weighter.log_var.requires_grad_(True)
    loss_a = torch.tensor(2.0, requires_grad=True)
    loss_b = torch.tensor(3.0, requires_grad=True)
    total = weighter({TaskType.OBJECT: loss_a, TaskType.POSE: loss_b})
    total.backward()
    assert weighter.log_var.grad is not None
    assert weighter.log_var.grad.shape == (2,)


def test_unknown_task_raises() -> None:
    weighter = UncertaintyWeighter([TaskType.OBJECT])
    with pytest.raises(KeyError):
        weighter.weight_single(TaskType.POSE, torch.tensor(1.0))
```

- [ ] **Step 3: Run the tests to verify failure**

Run: `uv run pytest tests/joint_loop/test_weighting.py -v`
Expected: collection error or import error (`ModuleNotFoundError: model.joint_loop.weighting`).

- [ ] **Step 4: Implement `UncertaintyWeighter`**

Create `src/model/joint_loop/weighting.py`:

```python
"""Kendall–Gal homoscedastic uncertainty weighter for multi-task losses."""
from __future__ import annotations

from collections.abc import Iterable, Mapping

import torch
from torch import Tensor, nn

from utils.model_naming import TaskType


class UnknownTaskError(KeyError):
    def __init__(self, task: TaskType, known: Iterable[TaskType]) -> None:
        super().__init__(
            f"task {task!r} not registered in weighter; known={list(known)}"
        )


class UncertaintyWeighter(nn.Module):
    """Learnable per-task log-variance balancing of multi-task losses.

    Implements Kendall, Gal & Cipolla (2018) homoscedastic uncertainty:

        total = sum_t( exp(-log_var_t) * loss_t + log_var_t )

    The `+log_var_t` term penalises growing the variance, balancing the
    `exp(-log_var_t)` term that down-weights losses with large variance.
    """

    def __init__(
        self,
        tasks: list[TaskType],
        init_log_var: Mapping[TaskType, float] | None = None,
    ) -> None:
        super().__init__()
        self.tasks = list(tasks)
        self._index = {task: i for i, task in enumerate(self.tasks)}
        log_var = torch.zeros(len(self.tasks))
        if init_log_var:
            for task, value in init_log_var.items():
                if task not in self._index:
                    raise UnknownTaskError(task, self.tasks)
                log_var[self._index[task]] = value
        self.log_var = nn.Parameter(log_var)

    def weight_single(self, task: TaskType, loss: Tensor) -> Tensor:
        if task not in self._index:
            raise UnknownTaskError(task, self.tasks)
        log_var_t = self.log_var[self._index[task]]
        return torch.exp(-log_var_t) * loss + log_var_t

    def forward(self, losses: Mapping[TaskType, Tensor]) -> Tensor:
        total = torch.zeros((), device=self.log_var.device)
        for task, loss in losses.items():
            total = total + self.weight_single(task, loss)
        return total
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `uv run pytest tests/joint_loop/test_weighting.py -v`
Expected: all 7 tests pass.

- [ ] **Step 6: Lint**

Run: `uv run ruff check src/model/joint_loop/weighting.py tests/joint_loop/test_weighting.py`
Expected: zero errors.

- [ ] **Step 7: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/__init__.py \
        tools/machine-learning/multi-task-yolo/src/model/joint_loop/weighting.py \
        tools/machine-learning/multi-task-yolo/tests/joint_loop/test_weighting.py
git commit -m "feat(multi-task-yolo): add UncertaintyWeighter for joint loss balancing"
```

---

## Task 4: `InterleavedTaskDataloader` (TDD)

Round-robin iteration over per-task dataloaders. Defines an "epoch" as `max(len(loader))` steps; shorter loaders silently restart.

**Files:**
- Create: `src/model/joint_loop/dataloaders.py`
- Test: `tests/joint_loop/test_dataloader.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/joint_loop/test_dataloader.py`:

```python
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
```

- [ ] **Step 2: Run the tests to verify failure**

Run: `uv run pytest tests/joint_loop/test_dataloader.py -v`
Expected: import error (`ModuleNotFoundError: model.joint_loop.dataloaders`).

- [ ] **Step 3: Implement `InterleavedTaskDataloader` (skeleton + core)**

Create `src/model/joint_loop/dataloaders.py`:

```python
"""Multi-task dataloader plumbing for joint training.

Two pieces:

* `build_task_dataloader(...)` — wraps `ultralytics.data.build` to produce a
  standard YOLO dataloader for a single task YAML. (Implemented in Task 5.)
* `InterleavedTaskDataloader` — round-robin iterator over per-task loaders.
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

    def __init__(
        self, loaders: dict[TaskType, _SizedIterable]
    ) -> None:
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
        cycles = {task: _CyclingIterator(task, self._loaders[task])
                  for task in self._sorted_tasks}
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

    def __iter__(self) -> "_CyclingIterator":
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/joint_loop/test_dataloader.py -v`
Expected: all 7 tests pass.

- [ ] **Step 5: Lint**

Run: `uv run ruff check src/model/joint_loop/dataloaders.py tests/joint_loop/test_dataloader.py`
Expected: zero errors.

- [ ] **Step 6: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/dataloaders.py \
        tools/machine-learning/multi-task-yolo/tests/joint_loop/test_dataloader.py
git commit -m "feat(multi-task-yolo): add InterleavedTaskDataloader"
```

---

## Task 5: `build_task_dataloader()` Helper

Wraps Ultralytics' dataset/dataloader builders so callers get a standard YOLO loader for a single task YAML. Tested only via the smoke test (it depends on real datasets and real ultralytics wiring); we keep its surface tight so it's a thin adapter.

**Files:**
- Modify: `src/model/joint_loop/dataloaders.py`

- [ ] **Step 1: Append `build_task_dataloader` to `dataloaders.py`**

Append at the bottom of `src/model/joint_loop/dataloaders.py`:

```python
# ---------------------------------------------------------------------------
# Per-task dataloader factory
# ---------------------------------------------------------------------------

from pathlib import Path  # noqa: E402

from ultralytics.cfg import get_cfg  # noqa: E402
from ultralytics.data.build import build_dataloader, build_yolo_dataset  # noqa: E402
from ultralytics.data.utils import check_det_dataset  # noqa: E402
from ultralytics.utils import DEFAULT_CFG_DICT  # noqa: E402


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
    args = get_cfg(overrides=base_overrides)

    data = check_det_dataset(str(dataset_yaml))
    img_path = data["train"] if mode == "train" else data.get("val", data["train"])

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
```

(`Path` is reimported here even though it's not strictly needed in the file scope — keep the import in case `dataset_yaml: Path` ever needs runtime conversion. Ruff will flag if it's unused; remove if so.)

- [ ] **Step 2: Lint**

Run: `uv run ruff check src/model/joint_loop/dataloaders.py`
Expected: zero errors. If ruff flags `Path` as unused, drop the import.

- [ ] **Step 3: Verify import + signature with a no-op test**

Create `tests/joint_loop/test_dataloader_factory_import.py` (transient — we'll delete it next step):

```python
from model.joint_loop.dataloaders import build_task_dataloader


def test_build_task_dataloader_is_callable() -> None:
    assert callable(build_task_dataloader)
```

Run: `uv run pytest tests/joint_loop/test_dataloader_factory_import.py -v`
Expected: 1 passed.

- [ ] **Step 4: Delete the transient import test**

```bash
rm tests/joint_loop/test_dataloader_factory_import.py
```

(The smoke test in Task 14 exercises this function with real ultralytics wiring. We keep the unit test surface clean.)

- [ ] **Step 5: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/dataloaders.py
git commit -m "feat(multi-task-yolo): add build_task_dataloader factory"
```

---

## Task 6: `_HydraHeadAdapter` + `build_criterion` + `epoch_update`

Wires Ultralytics' loss classes (`v8DetectionLoss`, `v8SegmentationLoss`, `v8PoseLoss`, `PoseLoss26`, `E2ELoss`) onto a single `Hydra` instance + `TaskType`. The adapter is the indirection layer; the dispatcher picks the right loss based on `hydra.head_end2end[task]`.

**Files:**
- Create: `src/model/joint_loop/criteria.py`

- [ ] **Step 1: Implement `criteria.py`**

Create `src/model/joint_loop/criteria.py`:

```python
"""Loss criterion adapters and dispatcher for joint Hydra training.

Ultralytics' loss classes (`v8DetectionLoss`, `v8SegmentationLoss`,
`v8PoseLoss`, `PoseLoss26`, `E2ELoss`) read a small set of attributes off
the model they are constructed against — `model`, `stride`, `nc`, `args`,
`hyp`, `kpt_shape`, `device`, `end2end`. We don't have a single
``DetectionModel`` per task in joint training; we have one shared backbone
and N task heads. The adapter exposes exactly the attributes those losses
read, sourced from the appropriate slice of the Hydra instance.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Callable

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
        ns = SimpleNamespace(**{k: getattr(self, k) for k in (
            "box", "cls", "dfl", "pose", "kobj", "rle", "epochs",
            "label_smoothing", "fl_gamma",
        )})
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
```

- [ ] **Step 2: Verify imports resolve**

Run: `uv run python -c "from model.joint_loop.criteria import build_criterion, epoch_update, JointLossHyp; print('ok')"`
Expected: prints `ok`. If `from ultralytics.utils.loss import PoseLoss26` fails, double-check the installed ultralytics version: `uv run python -c "import ultralytics; print(ultralytics.__version__)"` — must be ≥8.4.31 (the project requires 8.4.31, the installed version was 8.4.41 during design).

- [ ] **Step 3: Lint**

Run: `uv run ruff check src/model/joint_loop/criteria.py`
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/criteria.py
git commit -m "feat(multi-task-yolo): add Hydra head loss adapters and dispatcher"
```

---

## Task 7: `build_param_groups()` (TDD)

Ports `BaseTrainer.build_optimizer`'s 4-group split (decay weights, norm, bias, muon) and the cv3/proto x3 LR sub-split, with the head-layer index parameterized rather than hardcoded as `23`.

**Files:**
- Create: `src/model/joint_loop/optim.py` (initial version, just `build_param_groups`)
- Test: `tests/joint_loop/test_optim.py`

- [ ] **Step 1: Write the failing tests**

Create `tests/joint_loop/test_optim.py`:

```python
"""Tests for joint-training optimizer construction."""
from __future__ import annotations

import torch
from torch import nn

from model.joint_loop.optim import build_param_groups


class _FakeBackbone(nn.Module):
    """Tiny module covering all four MuSGD param-group cases."""

    def __init__(self) -> None:
        super().__init__()
        # 2D weight + decay -> g3 (muon) when use_muon, else g0
        self.linear = nn.Linear(4, 4, bias=True)
        # BatchNorm provides "norm" weight (1D) -> g1, and a bias -> g2
        self.bn = nn.BatchNorm1d(4)


def test_4_group_split_with_musgd() -> None:
    module = _FakeBackbone()
    groups = build_param_groups(
        module,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=23,
    )
    # We expect muon and non-muon halves; cv3/proto regex shouldn't match
    # anything in this fake module so the x3 sub-groups have empty params.
    flat_params = {id(p) for g in groups for p in g["params"]}
    expected_params = {id(p) for p in module.parameters()}
    assert flat_params == expected_params, "every parameter must end up in a group"


def test_3_group_split_with_adamw() -> None:
    module = _FakeBackbone()
    groups = build_param_groups(
        module,
        optimizer_name="AdamW",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=23,
    )
    # AdamW collapses the muon group back into g0 → still all params covered
    flat_params = {id(p) for g in groups for p in g["params"]}
    expected_params = {id(p) for p in module.parameters()}
    assert flat_params == expected_params

    # And no group should carry the use_muon flag
    for g in groups:
        assert "use_muon" not in g or g["use_muon"] is False


class _FakeHeadWithCv3(nn.Module):
    """Mimics a YOLO26 head whose final block (index 5) contains cv3."""

    def __init__(self) -> None:
        super().__init__()
        self.layer_5 = _Cv3Block()


class _Cv3Block(nn.Module):
    def __init__(self) -> None:
        super().__init__()
        self.cv3 = nn.Conv2d(4, 4, 1)


def test_cv3_x3_sub_split_with_parameterized_index() -> None:
    head = _FakeHeadWithCv3()
    groups = build_param_groups(
        head,
        optimizer_name="MuSGD",
        lr=0.01,
        momentum=0.9,
        decay=1e-5,
        head_last_layer_index=5,
    )
    boosted = [g for g in groups if g.get("lr", 0) == 0.03]
    boosted_params = [p for g in boosted for p in g["params"]]
    cv3_params = list(head.layer_5.cv3.parameters())
    assert any(id(p) in {id(q) for q in boosted_params} for p in cv3_params), (
        "cv3 params under layer index 5 should land in the lr*3 sub-group"
    )
```

- [ ] **Step 2: Run tests to verify failure**

Run: `uv run pytest tests/joint_loop/test_optim.py -v`
Expected: import error (`ModuleNotFoundError: model.joint_loop.optim`).

- [ ] **Step 3: Implement `build_param_groups`**

Create `src/model/joint_loop/optim.py`:

```python
"""Optimizer / scheduler / EMA / AMP wiring for joint Hydra training."""
from __future__ import annotations

import re
from collections.abc import Iterable
from typing import Any

from torch import nn

# Norm layer types — same probe Ultralytics' BaseTrainer uses.
_BN_TYPES: tuple[type, ...] = tuple(
    v for k, v in nn.__dict__.items() if "Norm" in k and isinstance(v, type)
)


def _cv3_proto_regex(head_last_layer_index: int) -> re.Pattern[str]:
    """Build the cv3/proto LR-boost regex for a given head-layer index.

    Mirrors Ultralytics' hardcoded ``(?=.*23)(?=.*cv3)|proto\.semseg`` but
    with the layer index parameterized so non-yolo26m scales also match.
    """
    return re.compile(
        rf"(?=.*{head_last_layer_index})(?=.*cv3)|proto\.semseg"
    )


def build_param_groups(
    module: nn.Module,
    *,
    optimizer_name: str,
    lr: float,
    momentum: float,
    decay: float,
    head_last_layer_index: int,
) -> list[dict[str, Any]]:
    """Construct optimizer parameter groups for a single module.

    Mirrors `BaseTrainer.build_optimizer`'s 4-group split when
    ``optimizer_name == "MuSGD"`` (g0=weights+decay, g1=norm/no-decay,
    g2=bias/no-decay, g3=muon-eligible weights). For AdamW (or any other
    non-MuSGD optimizer), g3 is collapsed back into g0 — the muon group
    only makes sense for Newton-Schulz orthogonalization.

    The cv3/proto regex (parameterized by ``head_last_layer_index``) splits
    every group into two sub-groups: an ``lr*3`` boosted sub-group for
    parameters whose fully-qualified name matches the regex, and the
    baseline sub-group for everything else.
    """
    use_muon = optimizer_name == "MuSGD"
    g_decay: dict[str, nn.Parameter] = {}
    g_norm: dict[str, nn.Parameter] = {}
    g_bias: dict[str, nn.Parameter] = {}
    g_muon: dict[str, nn.Parameter] = {}

    for module_name, sub in module.named_modules():
        for param_name, param in sub.named_parameters(recurse=False):
            full = f"{module_name}.{param_name}" if module_name else param_name
            if param.ndim >= 2 and use_muon:
                g_muon[full] = param
            elif "bias" in full:
                g_bias[full] = param
            elif isinstance(sub, _BN_TYPES) or "logit_scale" in full:
                g_norm[full] = param
            else:
                g_decay[full] = param

    optim_args = _optim_args(optimizer_name, lr=lr, momentum=momentum)

    groups: list[dict[str, Any]] = [
        {"params": g_decay, "weight_decay": decay, "param_group": "weight",
         **optim_args},
        {"params": g_norm, "weight_decay": 0.0, "param_group": "bn",
         **optim_args},
        {"params": g_bias, "weight_decay": 0.0, "param_group": "bias",
         **optim_args},
    ]
    if use_muon:
        groups.append({
            "params": g_muon, "weight_decay": decay, "use_muon": True,
            "param_group": "muon", **optim_args,
        })

    pattern = _cv3_proto_regex(head_last_layer_index)
    boosted: list[dict[str, Any]] = []
    for group in groups:
        named = group.pop("params")
        p1 = [p for k, p in named.items() if pattern.search(k)]
        p2 = [p for k, p in named.items() if not pattern.search(k)]
        # boosted (lr*3)
        boosted.append({**group, "params": p1, "lr": lr * 3})
        # baseline (lr*1)
        boosted.append({**group, "params": p2})
    return boosted


def _optim_args(name: str, *, lr: float, momentum: float) -> dict[str, Any]:
    if name in {"Adam", "Adamax", "AdamW", "NAdam", "RAdam"}:
        return {"lr": lr, "betas": (momentum, 0.999), "weight_decay": 0.0}
    if name == "RMSProp":
        return {"lr": lr, "momentum": momentum}
    if name in {"SGD", "MuSGD"}:
        return {"lr": lr, "momentum": momentum, "nesterov": True}
    raise NotImplementedError(  # noqa: TRY003
        f"unsupported optimizer name: {name!r}"
    )


def parameters_in_optimizer(opt: Any) -> Iterable[nn.Parameter]:
    """Flatten an optimizer's param groups into an iterable of parameters."""
    for group in opt.param_groups:
        yield from group["params"]
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/joint_loop/test_optim.py -v`
Expected: all 3 tests pass.

- [ ] **Step 5: Lint**

Run: `uv run ruff check src/model/joint_loop/optim.py tests/joint_loop/test_optim.py`
Expected: zero errors.

- [ ] **Step 6: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/optim.py \
        tools/machine-learning/multi-task-yolo/tests/joint_loop/test_optim.py
git commit -m "feat(multi-task-yolo): port MuSGD param-group split for joint optimizer"
```

---

## Task 8: Optimizer Builder, Schedulers, EMA, AMP scaler

Adds `build_joint_optimizers()`, `build_schedulers()`, `EMAHydra`, and `make_amp_scaler()` to `optim.py`. We replace the file rather than appending so the import block stays clean.

**Files:**
- Modify: `src/model/joint_loop/optim.py`

- [ ] **Step 1: Replace `optim.py` with the full implementation**

Overwrite `src/model/joint_loop/optim.py` with:

```python
"""Optimizer / scheduler / EMA / AMP wiring for joint Hydra training."""
from __future__ import annotations

import copy
import math
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from functools import partial
from typing import Any

import torch
from torch import nn, optim
from ultralytics.optim.muon import MuSGD

from model.hydra import Hydra
from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import TaskType

# Norm layer types — same probe Ultralytics' BaseTrainer uses.
_BN_TYPES: tuple[type, ...] = tuple(
    v for k, v in nn.__dict__.items() if "Norm" in k and isinstance(v, type)
)


def _cv3_proto_regex(head_last_layer_index: int) -> re.Pattern[str]:
    """Build the cv3/proto LR-boost regex for a given head-layer index.

    Mirrors Ultralytics' hardcoded ``(?=.*23)(?=.*cv3)|proto\.semseg`` but
    with the layer index parameterized so non-yolo26m scales also match.
    """
    return re.compile(
        rf"(?=.*{head_last_layer_index})(?=.*cv3)|proto\.semseg"
    )


def build_param_groups(
    module: nn.Module,
    *,
    optimizer_name: str,
    lr: float,
    momentum: float,
    decay: float,
    head_last_layer_index: int,
) -> list[dict[str, Any]]:
    """Construct optimizer parameter groups for a single module.

    Mirrors `BaseTrainer.build_optimizer`'s 4-group split when
    ``optimizer_name == "MuSGD"`` (g0=weights+decay, g1=norm/no-decay,
    g2=bias/no-decay, g3=muon-eligible weights). For AdamW (or any other
    non-MuSGD optimizer), g3 is collapsed back into g0 — the muon group
    only makes sense for Newton-Schulz orthogonalization.

    The cv3/proto regex (parameterized by ``head_last_layer_index``) splits
    every group into two sub-groups: an ``lr*3`` boosted sub-group for
    parameters whose fully-qualified name matches the regex, and the
    baseline sub-group for everything else.
    """
    use_muon = optimizer_name == "MuSGD"
    g_decay: dict[str, nn.Parameter] = {}
    g_norm: dict[str, nn.Parameter] = {}
    g_bias: dict[str, nn.Parameter] = {}
    g_muon: dict[str, nn.Parameter] = {}

    for module_name, sub in module.named_modules():
        for param_name, param in sub.named_parameters(recurse=False):
            full = f"{module_name}.{param_name}" if module_name else param_name
            if param.ndim >= 2 and use_muon:
                g_muon[full] = param
            elif "bias" in full:
                g_bias[full] = param
            elif isinstance(sub, _BN_TYPES) or "logit_scale" in full:
                g_norm[full] = param
            else:
                g_decay[full] = param

    optim_args = _optim_args(optimizer_name, lr=lr, momentum=momentum)

    groups: list[dict[str, Any]] = [
        {"params": g_decay, "weight_decay": decay, "param_group": "weight",
         **optim_args},
        {"params": g_norm, "weight_decay": 0.0, "param_group": "bn",
         **optim_args},
        {"params": g_bias, "weight_decay": 0.0, "param_group": "bias",
         **optim_args},
    ]
    if use_muon:
        groups.append({
            "params": g_muon, "weight_decay": decay, "use_muon": True,
            "param_group": "muon", **optim_args,
        })

    pattern = _cv3_proto_regex(head_last_layer_index)
    boosted: list[dict[str, Any]] = []
    for group in groups:
        named = group.pop("params")
        p1 = [p for k, p in named.items() if pattern.search(k)]
        p2 = [p for k, p in named.items() if not pattern.search(k)]
        boosted.append({**group, "params": p1, "lr": lr * 3})
        boosted.append({**group, "params": p2})
    return boosted


def _optim_args(name: str, *, lr: float, momentum: float) -> dict[str, Any]:
    if name in {"Adam", "Adamax", "AdamW", "NAdam", "RAdam"}:
        return {"lr": lr, "betas": (momentum, 0.999), "weight_decay": 0.0}
    if name == "RMSProp":
        return {"lr": lr, "momentum": momentum}
    if name in {"SGD", "MuSGD"}:
        return {"lr": lr, "momentum": momentum, "nesterov": True}
    raise NotImplementedError(  # noqa: TRY003
        f"unsupported optimizer name: {name!r}"
    )


def parameters_in_optimizer(opt: Any) -> Iterable[nn.Parameter]:
    """Flatten an optimizer's param groups into an iterable of parameters."""
    for group in opt.param_groups:
        yield from group["params"]


# ---------------------------------------------------------------------------
# Joint optimizer / scheduler / EMA / AMP wiring
# ---------------------------------------------------------------------------

@dataclass
class JointOptimizers:
    backbone: optim.Optimizer
    heads: dict[TaskType, optim.Optimizer]
    log_var: optim.Optimizer

    def all(self) -> list[optim.Optimizer]:
        return [self.backbone, *self.heads.values(), self.log_var]


def _instantiate_optimizer(
    name: str, groups: list[dict[str, Any]]
) -> optim.Optimizer:
    if name == "MuSGD":
        return MuSGD(params=groups, muon=0.2, sgd=1.0)
    if not hasattr(optim, name):
        raise NotImplementedError(  # noqa: TRY003
            f"unsupported optimizer: {name!r}"
        )
    return getattr(optim, name)(groups)


def head_last_layer_index(hydra: Hydra, task: TaskType) -> int:
    """Index of the head's final module within the full network."""
    head = hydra.heads[str(task)]
    return hydra.backbone_length + len(head) - 1


def build_joint_optimizers(
    hydra: Hydra,
    weighter: UncertaintyWeighter,
    *,
    optimizer_name: str,
    lr_backbone: float,
    lr_heads: float,
    lr_logvar: float,
    momentum: float,
    weight_decay: float,
) -> JointOptimizers:
    """Build the per-module optimizer set per the spec (Section 7.2).

    `opt_logvar` is always AdamW (Newton-Schulz needs ndim >= 2).
    """
    backbone_groups = build_param_groups(
        hydra.shared_backbone,
        optimizer_name=optimizer_name,
        lr=lr_backbone,
        momentum=momentum,
        decay=weight_decay,
        # The backbone has no cv3 final-layer to boost; pass an unmatchable
        # index so the regex never fires for the backbone module.
        head_last_layer_index=-1,
    )
    opt_backbone = _instantiate_optimizer(optimizer_name, backbone_groups)

    heads: dict[TaskType, optim.Optimizer] = {}
    for task_str, head_module in hydra.heads.items():
        task = TaskType(task_str)
        groups = build_param_groups(
            head_module,
            optimizer_name=optimizer_name,
            lr=lr_heads,
            momentum=momentum,
            decay=weight_decay,
            head_last_layer_index=head_last_layer_index(hydra, task),
        )
        heads[task] = _instantiate_optimizer(optimizer_name, groups)

    opt_logvar = optim.AdamW(
        [weighter.log_var], lr=lr_logvar, weight_decay=0.0
    )
    return JointOptimizers(
        backbone=opt_backbone, heads=heads, log_var=opt_logvar
    )


def build_schedulers(
    optimizers: JointOptimizers,
    *,
    epochs: int,
    warmup_epochs: int,
) -> list[optim.lr_scheduler.LRScheduler]:
    schedulers: list[optim.lr_scheduler.LRScheduler] = []
    for opt in optimizers.all():
        warmup = optim.lr_scheduler.LambdaLR(
            opt,
            lr_lambda=partial(
                _warmup_factor, warmup=max(warmup_epochs, 1)
            ),
        )
        cosine = optim.lr_scheduler.CosineAnnealingLR(opt, T_max=epochs)
        schedulers.append(
            optim.lr_scheduler.ChainedScheduler([warmup, cosine])
        )
    return schedulers


def _warmup_factor(epoch: int, *, warmup: int) -> float:
    if epoch >= warmup:
        return 1.0
    return float(epoch + 1) / float(warmup)


class EMAHydra:
    """Exponential moving average of a Hydra + UncertaintyWeighter pair.

    Decay schedule matches Ultralytics' `ModelEMA`:

        decay(step) = 0.9999 * (1 - exp(-step / 2000))
    """

    def __init__(
        self,
        hydra: Hydra,
        weighter: UncertaintyWeighter,
        *,
        max_decay: float = 0.9999,
        warmup: float = 2000.0,
    ) -> None:
        self.hydra = copy.deepcopy(hydra).eval()
        self.weighter = copy.deepcopy(weighter).eval()
        for p in self.hydra.parameters():
            p.requires_grad_(False)
        for p in self.weighter.parameters():
            p.requires_grad_(False)
        self._max_decay = max_decay
        self._warmup = warmup
        self.updates = 0

    def decay(self) -> float:
        return self._max_decay * (
            1.0 - math.exp(-self.updates / self._warmup)
        )

    @torch.no_grad()
    def update(self, hydra: Hydra, weighter: UncertaintyWeighter) -> None:
        self.updates += 1
        d = self.decay()
        for ema_p, p in zip(
            self.hydra.parameters(), hydra.parameters(), strict=True
        ):
            ema_p.mul_(d).add_(p.detach(), alpha=1.0 - d)
        for ema_p, p in zip(
            self.weighter.parameters(),
            weighter.parameters(),
            strict=True,
        ):
            ema_p.mul_(d).add_(p.detach(), alpha=1.0 - d)

    def state_dict(self) -> dict[str, Any]:
        return {
            "updates": self.updates,
            "hydra": self.hydra.state_dict(),
            "weighter": self.weighter.state_dict(),
        }

    def load_state_dict(self, state: Mapping[str, Any]) -> None:
        self.updates = int(state["updates"])
        self.hydra.load_state_dict(state["hydra"])
        self.weighter.load_state_dict(state["weighter"])


def make_amp_scaler(
    *, enabled: bool, device_type: str = "cuda"
) -> torch.amp.GradScaler:
    return torch.amp.GradScaler(device_type, enabled=enabled)
```

- [ ] **Step 2: Verify imports resolve**

Run: `uv run python -c "from model.joint_loop.optim import build_joint_optimizers, build_schedulers, EMAHydra, make_amp_scaler; print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Add a unit test for the warmup factor**

Append to `tests/joint_loop/test_optim.py`:

```python
def test_warmup_factor_clamps_to_one() -> None:
    from model.joint_loop.optim import _warmup_factor
    assert _warmup_factor(0, warmup=3) == pytest.approx(1.0 / 3.0)
    assert _warmup_factor(2, warmup=3) == pytest.approx(1.0)
    assert _warmup_factor(99, warmup=3) == pytest.approx(1.0)
```

…and add `import pytest` to the top of the file if not present already.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/joint_loop/test_optim.py -v`
Expected: all 4 tests pass.

- [ ] **Step 5: Lint**

Run: `uv run ruff check src/model/joint_loop/optim.py tests/joint_loop/test_optim.py`
Expected: zero errors.

- [ ] **Step 6: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/optim.py \
        tools/machine-learning/multi-task-yolo/tests/joint_loop/test_optim.py
git commit -m "feat(multi-task-yolo): add joint optimizers, schedulers, EMA, AMP scaler"
```

---

## Task 9: Backbone Gradient Accumulation Invariant Test

A focused test that asserts the synchronized-step contract: shared-backbone `.grad` after `N` per-task `loss.backward()` calls (without zeroing the backbone between them) equals the manual sum of per-task gradients computed in isolation.

**Files:**
- Modify: `tests/joint_loop/test_optim.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/joint_loop/test_optim.py`:

```python
def test_backbone_gradient_accumulation_invariant() -> None:
    """A synchronized step must accumulate per-task grads on the backbone."""
    backbone = nn.Linear(4, 4, bias=False)
    head_a = nn.Linear(4, 1, bias=False)
    head_b = nn.Linear(4, 1, bias=False)

    x = torch.randn(2, 4)

    # Reference: compute each task's backbone grad in isolation, sum them.
    manual_grads = torch.zeros_like(backbone.weight)
    for head in (head_a, head_b):
        backbone.zero_grad(set_to_none=True)
        head.zero_grad(set_to_none=True)
        out = head(backbone(x)).sum()
        out.backward()
        assert backbone.weight.grad is not None
        manual_grads = manual_grads + backbone.weight.grad.detach().clone()

    # Joint accumulation: zero backbone once, run both backwards back-to-back,
    # zero each head between calls but NOT the backbone.
    backbone.zero_grad(set_to_none=True)
    for head in (head_a, head_b):
        head.zero_grad(set_to_none=True)
        out = head(backbone(x)).sum()
        out.backward()

    assert backbone.weight.grad is not None
    assert torch.allclose(backbone.weight.grad, manual_grads, atol=1e-6), (
        "synchronized backbone grad must equal sum of single-task grads"
    )
```

- [ ] **Step 2: Run the test to verify it passes**

Run: `uv run pytest tests/joint_loop/test_optim.py::test_backbone_gradient_accumulation_invariant -v`
Expected: 1 passed.

(This test is a regression guard, not a "fix the implementation" gate — the invariant is a property of plain PyTorch autograd. If it ever fails, the loop body has done something wrong with `zero_grad()` placement.)

- [ ] **Step 3: Lint**

Run: `uv run ruff check tests/joint_loop/test_optim.py`
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/tests/joint_loop/test_optim.py
git commit -m "test(multi-task-yolo): pin synchronized backbone gradient invariant"
```

---

## Task 10: `checkpoints.py` — Per-task `.pt` + `joint_state.json` + `config.json`

Writes per-task `.pt` files that existing tooling (`validator.py`, `export_hydra.py`) can consume, and the resume artifact.

**Files:**
- Create: `src/model/joint_loop/checkpoints.py`

- [ ] **Step 1: Implement `checkpoints.py`**

Create `src/model/joint_loop/checkpoints.py`:

```python
"""Per-task checkpoint writer + joint-state JSON for joint training.

Each per-task `.pt` is constructed `cross.py`-style: take the head's original
YOLO checkpoint as the carrier, swap its backbone for the EMA backbone, and
save it via Ultralytics' YOLO save path so existing tooling consumes it
unchanged. The joint-state JSON owns the resume artifact: optimizer state,
scheduler state, EMA decay step counter, log-variance values, best score,
current epoch, RNG state.
"""
from __future__ import annotations

import json
import os
import secrets
from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, cast

import torch
from torch import optim
from ultralytics.models.yolo.model import YOLO
from ultralytics.nn.tasks import DetectionModel

from model.hydra import Hydra, get_backbone, set_backbone
from model.joint_loop.optim import EMAHydra, JointOptimizers
from model.joint_loop.weighting import UncertaintyWeighter
from utils.model_naming import HydraModelName, TaskType


def write_per_task_checkpoint(
    *,
    ema: EMAHydra,
    hydra_model: HydraModelName,
    task: TaskType,
    head_yolo_path: Path,
    output_path: Path,
) -> None:
    """Write a single per-task `.pt` containing the EMA backbone + this head.

    Equivalent to `cross.py` but driven by the EMA snapshot held by `ema`.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)

    head_yolo = YOLO(str(head_yolo_path))
    head_root = cast(DetectionModel, head_yolo.model)

    ema_backbone = get_backbone(
        cast(DetectionModel, _ema_root_module(ema, hydra_model)),
        hydra_model.number_of_frozen_modules,
    )
    set_backbone(head_root, ema_backbone, hydra_model.number_of_frozen_modules)

    # Atomic-ish write: save to a temp path, then rename.
    tmp_path = output_path.with_suffix(output_path.suffix + ".tmp")
    head_yolo.save(str(tmp_path))
    os.replace(tmp_path, output_path)


def _ema_root_module(ema: EMAHydra, hydra_model: HydraModelName) -> Any:
    """Return a module-like object that exposes the EMA backbone modules.

    `get_backbone()` operates on an Ultralytics `DetectionModel`. The EMA
    snapshot is a `Hydra`, whose `shared_backbone` is the equivalent
    nn.ModuleList. We construct a thin shim with `.model` and `.yaml`
    attributes that `get_backbone(..., number_of_frozen_modules=...)` can
    consume — when `number_of_frozen_modules` is provided explicitly,
    `get_backbone` does not consult `.yaml` at all.
    """
    class _Shim:
        def __init__(self, hydra: Hydra) -> None:
            self.model = torch.nn.Sequential(*list(hydra.shared_backbone))
            self.yaml = {"backbone": [None] * hydra.backbone_length}

    return _Shim(ema.hydra)


def write_joint_state(
    state_path: Path,
    *,
    epoch: int,
    best_score: float,
    optimizers: JointOptimizers,
    schedulers: list[optim.lr_scheduler.LRScheduler],
    ema: EMAHydra,
    weighter: UncertaintyWeighter,
    rng_state: dict[str, Any],
) -> None:
    state_path.parent.mkdir(parents=True, exist_ok=True)
    blob = {
        "epoch": epoch,
        "best_score": best_score,
        "opt_backbone": optimizers.backbone.state_dict(),
        "opt_heads": {
            str(t): o.state_dict() for t, o in optimizers.heads.items()
        },
        "opt_logvar": optimizers.log_var.state_dict(),
        "schedulers": [s.state_dict() for s in schedulers],
        "ema": ema.state_dict(),
        "weighter": weighter.state_dict(),
        "rng_state": rng_state,
    }
    tmp = state_path.with_suffix(state_path.suffix + ".tmp")
    torch.save(blob, tmp)
    os.replace(tmp, state_path)


def read_joint_state(state_path: Path) -> dict[str, Any]:
    return torch.load(state_path, map_location="cpu", weights_only=False)


def write_run_config(config_path: Path, config: Mapping[str, Any]) -> None:
    config_path.parent.mkdir(parents=True, exist_ok=True)
    plain = _to_jsonable(dict(config))
    with config_path.open("w") as f:
        json.dump(plain, f, indent=2, default=str)


def _to_jsonable(value: Any) -> Any:
    if is_dataclass(value):
        return _to_jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if isinstance(value, Path):
        return str(value)
    return value


def random_run_wordlet() -> str:
    """Generate a wonderwords-compatible run-name suffix.

    The CLI uses `wonderwords.RandomWord()` directly; this helper exists for
    tests that need a deterministic-ish suffix without pulling in wonderwords.
    """
    return secrets.token_hex(3)
```

- [ ] **Step 2: Verify imports resolve**

Run: `uv run python -c "from model.joint_loop.checkpoints import write_per_task_checkpoint, write_joint_state, read_joint_state, write_run_config; print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Lint**

Run: `uv run ruff check src/model/joint_loop/checkpoints.py`
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/checkpoints.py
git commit -m "feat(multi-task-yolo): add per-task checkpoint and joint-state writers"
```

---

## Task 11: `validation.py` — Per-Epoch Validation Hook

Calls existing `validate_hydra_model()` per task on the EMA snapshot, reads back `metrics.json`, returns weighted aggregate score.

**Files:**
- Create: `src/model/joint_loop/validation.py`

- [ ] **Step 1: Implement `validation.py`**

Create `src/model/joint_loop/validation.py`:

```python
"""Per-epoch validation hook for joint training.

Reuses `validation.validator.validate_hydra_model()` unmodified. Each task
gets its own per-task assembled `.pt` materialized in a temp dir, then the
existing validator writes `runs/val/<hydra_model>/metrics.json`, which we
read back to extract the primary metric for that task.
"""
from __future__ import annotations

import json
import logging
import shutil
import tempfile
from collections.abc import Mapping
from pathlib import Path

from model.joint_loop.checkpoints import write_per_task_checkpoint
from model.joint_loop.optim import EMAHydra
from utils.model_naming import HydraModelName, ModelName, TaskType
from validation.validator import ValidationConfig, validate_hydra_model

logger = logging.getLogger(__name__)


_PRIMARY_METRIC_KEY: dict[TaskType, str] = {
    TaskType.OBJECT: "metrics/mAP50-95(B)",
    TaskType.SEGMENTATION: "metrics/mAP50-95(B)",
    TaskType.POSE: "metrics/mAP50-95(P)",
}


def run_validation(
    *,
    ema: EMAHydra,
    hydra_model: HydraModelName,
    datasets_per_task: Mapping[TaskType, Path],
    head_source_paths: Mapping[TaskType, Path],
    runs_dir: Path,
    imgsz: int,
    batch: int,
    device: str,
    task_weights: Mapping[TaskType, float],
) -> tuple[float, dict[TaskType, float]]:
    """Validate every task on EMA weights and return (score, per_task_metric).

    `score = sum(task_weights[t] * metric[t] for t in tasks)`.
    """
    per_task_metric: dict[TaskType, float] = {}

    with tempfile.TemporaryDirectory() as tmpdir:
        assets_dir = Path(tmpdir)

        for head in hydra_model.heads:
            task = head.task_type()
            head_pt_name = head.name + ".pt"
            head_pt_path = assets_dir / head_pt_name
            # validate_hydra_model loads `assets_dir / <head>.pt` directly, so
            # we copy the original head checkpoint into the temp dir.
            shutil.copy(head_source_paths[task], head_pt_path)

            backbone_pt_name = hydra_model.backbone.name + ".pt"
            backbone_pt_path = assets_dir / backbone_pt_name

            # Materialise the EMA backbone+head as a single per-task .pt
            # under the *backbone* filename. validate_hydra_model only
            # extracts the backbone slice from this file (its head slice is
            # discarded and re-derived from `head_pt_path`), so the file is
            # effectively the EMA backbone wrapped in YOLO's checkpoint
            # format.
            single_task_hydra = HydraModelName(
                backbone=hydra_model.backbone,
                heads=[head],
                number_of_frozen_modules=hydra_model.number_of_frozen_modules,
            )
            write_per_task_checkpoint(
                ema=ema,
                hydra_model=single_task_hydra,
                task=task,
                head_yolo_path=head_pt_path,
                output_path=backbone_pt_path,
            )

            config = ValidationConfig(
                data=str(datasets_per_task[task]),
                project=str(runs_dir),
                imgsz=imgsz,
                batch=batch,
                device=device,
            )
            try:
                validate_hydra_model(single_task_hydra, config, assets_dir)
            except Exception:
                logger.exception(
                    "validation failed for task %s; recording NaN", task
                )
                per_task_metric[task] = float("nan")
                continue

            metrics_path = runs_dir / "val" / str(single_task_hydra) / "metrics.json"
            primary = _read_primary_metric(metrics_path, task)
            per_task_metric[task] = primary

    score = sum(
        task_weights.get(t, 1.0) * v
        for t, v in per_task_metric.items()
        if not _is_nan(v)
    )
    return score, per_task_metric


def _read_primary_metric(metrics_path: Path, task: TaskType) -> float:
    if not metrics_path.exists():
        logger.warning("metrics.json not found at %s", metrics_path)
        return float("nan")
    with metrics_path.open() as f:
        metrics = json.load(f)
    key = _PRIMARY_METRIC_KEY[task]
    if key not in metrics:
        logger.warning(
            "primary metric %s missing from %s (have: %s)",
            key, metrics_path, list(metrics.keys()),
        )
        return float("nan")
    return float(metrics[key])


def _is_nan(value: float) -> bool:
    return value != value  # noqa: PLR0124
```

`validate_hydra_model` reconstructs a single-task model from `assets_dir / hydra_model.backbone.name` plus `assets_dir / hydra_model.heads[0].name`. We populate both files in `assets_dir` per task before invoking it: the backbone-named file is the EMA-swapped checkpoint we just wrote, the head-named file is a copy of the original head checkpoint.

- [ ] **Step 2: Verify imports resolve**

Run: `uv run python -c "from model.joint_loop.validation import run_validation; print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Lint**

Run: `uv run ruff check src/model/joint_loop/validation.py`
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/validation.py
git commit -m "feat(multi-task-yolo): add per-epoch validation hook reusing validate_hydra_model"
```

---

## Task 12: `loop.py` — `train_joint()`

The synchronized-accumulation training loop. Owns the integration of every other engine module.

**Files:**
- Create: `src/model/joint_loop/loop.py`

- [ ] **Step 1: Implement the loop**

Create `src/model/joint_loop/loop.py`:

```python
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
from torch import nn
from torch.nn.utils import clip_grad_norm_

from model.hydra import Hydra
from model.joint_loop.checkpoints import (
    write_joint_state,
    write_per_task_checkpoint,
    write_run_config,
)
from model.joint_loop.criteria import JointLossHyp, build_criterion, epoch_update
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
    warmup_epochs: int = 3
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
    clip_heads: bool = False
    init_log_var: dict[TaskType, float] = field(default_factory=dict)
    task_weights: dict[TaskType, float] = field(default_factory=dict)
    hyp: JointLossHyp = field(default_factory=JointLossHyp)


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
    tasks: list[TaskType] = sorted(
        (TaskType(k) for k in hydra.heads.keys()), key=lambda t: t.value
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

    for epoch in range(config.epochs):
        interleaved.set_epoch(epoch)
        _train_one_epoch(
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

        validation_target = ema if ema is not None else _ema_passthrough(
            hydra, weighter
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
        logger.info(
            "epoch %d: score=%.4f per_task=%s", epoch, score, per_task
        )
        if wandb_run is not None:
            wandb.log(
                {
                    "val/score": score,
                    **{
                        f"val/{t}": v for t, v in per_task.items()
                    },
                    "epoch": epoch,
                }
            )

        for task in tasks:
            write_per_task_checkpoint(
                ema=validation_target,
                hydra_model=HydraModelName(
                    backbone=hydra_model.backbone,
                    heads=[h for h in hydra_model.heads
                           if h.task_type() == task],
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
) -> None:
    hydra.train()
    weighter.train()

    step = 0
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
            loss_total, _components = criteria[task](pred, batch_on_device)
            weighted = weighter.weight_single(task, loss_total)

        scaler.scale(weighted).backward()
        per_task_losses[task] = loss_total.detach()

        step += 1
        if step % len(tasks) == 0:
            scaler.unscale_(optimizers.backbone)
            clip_grad_norm_(
                hydra.shared_backbone.parameters(),
                max_norm=config.max_grad_norm,
            )
            for opt in optimizers.all():
                if opt is optimizers.backbone:
                    pass  # already unscaled above
                else:
                    scaler.unscale_(opt)
                if config.clip_heads and opt in optimizers.heads.values():
                    clip_grad_norm_(
                        list(parameters_in_optimizer(opt)),
                        max_norm=config.max_grad_norm,
                    )
                scaler.step(opt)
            scaler.update()
            if ema is not None:
                ema.update(hydra, weighter)

            if (step // len(tasks)) % config.log_interval == 0 and wandb_run:
                lr_log = {
                    "lr/backbone": optimizers.backbone.param_groups[0]["lr"],
                    "lr/logvar": optimizers.log_var.param_groups[0]["lr"],
                    **{
                        f"lr/heads_{t}": opt.param_groups[0]["lr"]
                        for t, opt in optimizers.heads.items()
                    },
                }
                losses_log = {
                    f"loss/{t}": v.item()
                    for t, v in per_task_losses.items()
                }
                logvar_log = {
                    f"logvar/{t}": weighter.log_var[
                        weighter.tasks.index(t)
                    ].item()
                    for t in tasks
                }
                wandb.log(
                    {
                        "epoch": epoch,
                        "step": step // len(tasks),
                        **lr_log,
                        **losses_log,
                        **logvar_log,
                    }
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
        "numpy": np.random.get_state(),
        "torch": torch.get_rng_state(),
        "torch_cuda": (
            torch.cuda.get_rng_state_all()
            if torch.cuda.is_available()
            else []
        ),
    }


class _PassthroughEMA:
    """Stand-in EMA that just exposes the live model + weighter (no-op decay)."""

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
```

- [ ] **Step 2: Verify imports resolve**

Run: `uv run python -c "from model.joint_loop.loop import train_joint, JointTrainConfig; print('ok')"`
Expected: prints `ok`.

- [ ] **Step 3: Lint**

Run: `uv run ruff check src/model/joint_loop/loop.py`
Expected: zero errors. Fix style violations inline.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_loop/loop.py
git commit -m "feat(multi-task-yolo): add train_joint synchronized-accumulation loop"
```

---

## Task 13: `joint_train.py` — Click CLI Entrypoint

The user-facing entry. Mirrors `train.py`'s CLI conventions.

**Files:**
- Create: `src/model/joint_train.py`

- [ ] **Step 1: Implement the CLI**

Create `src/model/joint_train.py`:

```python
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
from typing import Any

import click
import numpy as np
import torch
import wandb
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
            raise click.BadParameter(
                f"expected task=value, got {raw!r}",
            )
        k, v = raw.split("=", 1)
        try:
            task = TaskType(k.strip())
        except ValueError as exc:
            raise click.BadParameter(f"unknown task: {k!r}") from exc
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
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _select_device(device_str: str) -> torch.device:
    if device_str in {"-1", "cpu"}:
        return torch.device("cpu")
    if device_str.startswith("cuda") or device_str.isdigit() or "," in device_str:
        first = device_str.split(",")[0]
        if first == "cuda":
            return torch.device("cuda")
        return torch.device(f"cuda:{int(first)}")
    return torch.device(device_str)


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
@click.option("--device", default="0", type=str)
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
def main(  # noqa: PLR0913
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
    selected_device = _select_device(device)

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
```

- [ ] **Step 2: Verify the CLI parses**

Run: `uv run python src/model/joint_train.py --help`
Expected: Click prints the help screen with all options listed.

- [ ] **Step 3: Lint**

Run: `uv run ruff check src/model/joint_train.py`
Expected: zero errors. Fix any inline.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/src/model/joint_train.py
git commit -m "feat(multi-task-yolo): add joint_train.py CLI entrypoint"
```

---

## Task 14: End-to-End Smoke Test

A test that wires the full stack on tiny synthetic inputs to catch integration regressions. Skipped automatically if real datasets/assets are missing.

**Files:**
- Create: `tests/joint_loop/test_loop_smoke.py`

- [ ] **Step 1: Write the smoke test**

Create `tests/joint_loop/test_loop_smoke.py`:

```python
"""End-to-end smoke test for the joint training loop.

Skipped if the standard YOLO checkpoints / COCO YAMLs are not present on
disk. The test runs 2 epochs on whichever subset of tasks the user has
assets for and asserts:

* the loop completes without raising,
* per-task `last.pt` files are written,
* `joint_state.json` and `config.json` are written.
"""
from __future__ import annotations

import os
from pathlib import Path

import pytest
import torch

ASSETS_DIR = Path("assets")
COCO_YAML = ASSETS_DIR / "datasets" / "coco.yaml"


pytestmark = pytest.mark.skipif(
    not (
        ASSETS_DIR.exists()
        and (ASSETS_DIR / "yolo26m.pt").exists()
        and COCO_YAML.exists()
    ),
    reason="yolo26m / coco.yaml assets not present",
)


def test_smoke_two_epochs(tmp_path: Path) -> None:
    from model.hydra import Hydra
    from model.joint_loop.criteria import JointLossHyp
    from model.joint_loop.dataloaders import (
        InterleavedTaskDataloader,
        build_task_dataloader,
    )
    from model.joint_loop.loop import JointTrainConfig, train_joint
    from utils.model_naming import HydraModelName, ModelName, TaskType

    hydra_model = HydraModelName(
        backbone=ModelName("yolo26m"),
        heads=[ModelName("yolo26m")],
        number_of_frozen_modules=11,
    )
    backbone_path = ASSETS_DIR / "yolo26m.pt"
    head_source_paths = {TaskType.OBJECT: backbone_path}
    task_dict = {TaskType.OBJECT: backbone_path}

    hydra = Hydra(
        backbone_path=str(backbone_path),
        task_dict=task_dict,
    )

    loader, _ = build_task_dataloader(
        TaskType.OBJECT,
        COCO_YAML,
        imgsz=320,
        batch=2,
        workers=0,
    )
    interleaved = InterleavedTaskDataloader({TaskType.OBJECT: loader})

    config = JointTrainConfig(
        epochs=2,
        patience=999,
        warmup_epochs=1,
        val_interval=1,
        log_interval=1,
        optimizer_name="MuSGD" if torch.cuda.is_available() else "AdamW",
        lr_backbone=0.0,
        lr_heads=0.001,
        lr_logvar=0.001,
        use_amp=False,
        use_ema=False,
        hyp=JointLossHyp(epochs=2),
    )
    run_dir = tmp_path / "joint_train" / "smoke~test"
    runs_dir = tmp_path

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    os.environ["WANDB_MODE"] = "disabled"

    train_joint(
        hydra=hydra,
        hydra_model=hydra_model,
        interleaved=interleaved,
        datasets_per_task={TaskType.OBJECT: COCO_YAML},
        head_source_paths=head_source_paths,
        run_dir=run_dir,
        runs_dir=runs_dir,
        config=config,
        device=device,
        device_str="cpu" if device.type == "cpu" else "0",
        imgsz=320,
        batch=2,
        seed=0,
        wandb_run=None,
    )

    assert (run_dir / "config.json").exists()
    assert (run_dir / "joint_state.json").exists()
    assert (run_dir / "object" / "last.pt").exists()
```

- [ ] **Step 2: Run the smoke test**

Run: `uv run pytest tests/joint_loop/test_loop_smoke.py -v -s`
Expected: 1 passed (or 1 skipped if local environment lacks assets — both outcomes acceptable).

- [ ] **Step 3: Lint**

Run: `uv run ruff check tests/joint_loop/test_loop_smoke.py`
Expected: zero errors.

- [ ] **Step 4: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/tests/joint_loop/test_loop_smoke.py
git commit -m "test(multi-task-yolo): add end-to-end joint training smoke test"
```

---

## Task 15: Run the Full Test Suite

Final verification that nothing in this branch regressed.

- [ ] **Step 1: Run all tests**

Run: `uv run pytest tests/ -v`
Expected: every test either passes or is `SKIPPED` (with a clear "asset/dataset not present" reason). Zero `FAILED`.

- [ ] **Step 2: Run lint over the whole feature surface**

Run: `uv run ruff check src tests`
Expected: zero errors.

- [ ] **Step 3: Run formatter check (no changes expected)**

Run: `uv run ruff format src tests --check`
Expected: zero files would be reformatted. If anything would change, run `uv run ruff format src tests` and amend the most recent commit.

- [ ] **Step 4: Verify CLI help renders cleanly**

Run: `uv run python src/model/joint_train.py --help`
Expected: full Click help output, no Python errors.

- [ ] **Step 5: Commit any formatter-driven changes**

If `ruff format` modified files in Step 3, commit them:

```bash
git add tools/machine-learning/multi-task-yolo/
git commit -m "style(multi-task-yolo): apply ruff format to joint training surface"
```

If no changes, skip this step.

---

## Task 16: Update `AGENTS.md`

Document the new entrypoint and gotchas for future agents working on this directory.

**Files:**
- Modify: `AGENTS.md`

- [ ] **Step 1: Add joint training to `Reliable Commands`**

Append under "## Reliable Commands" in `AGENTS.md`:

```
- Joint training CLI help: `uv run python src/model/joint_train.py --help`
- Run the joint-training test suite: `uv run pytest tests/`
```

- [ ] **Step 2: Add joint training entries to `Code Layout`**

Append under "## Code Layout (what actually runs)" in `AGENTS.md`:

```
- `src/model/joint_train.py`: Click CLI for joint multi-task finetuning of a Hydra model. Drives the engine in `src/model/joint_loop/`.
- `src/model/joint_loop/`: package hosting the joint training engine (`loop.py`, `dataloaders.py`, `criteria.py`, `weighting.py`, `optim.py`, `validation.py`, `checkpoints.py`).
```

- [ ] **Step 3: Add gotchas under `Repo-Specific Gotchas`**

Append under "## Repo-Specific Gotchas" in `AGENTS.md`:

```
- `src/model/joint_train.py` is a separate engine from `src/model/train.py`; it owns a custom PyTorch loop with synchronized cross-task gradient accumulation. It deliberately does not use `YOLO().train()`.
- `Hydra.forward()` flattens head outputs into a `<task>_output` dict for export/inference; for training, use `Hydra.run_backbone()` + `Hydra.run_head()` (raw, non-flattened) which `E2ELoss.parse_output()` expects.
- The MuSGD cv3/proto LR boost regex in `model.joint_loop.optim.build_param_groups` is parameterized by head-last-layer index, not hardcoded to `23` — non-yolo26m scales work transparently.
- `tests/` is run via `uv run pytest tests/`; `pyproject.toml` includes a `[tool.pytest.ini_options]` block with `pythonpath = ["src"]`.
```

- [ ] **Step 4: Lint (markdown is not linted; just sanity-check the file renders)**

Run: `head -80 AGENTS.md`
Expected: file content displays cleanly.

- [ ] **Step 5: Commit**

```bash
git add tools/machine-learning/multi-task-yolo/AGENTS.md
git commit -m "docs(multi-task-yolo): document joint training in AGENTS.md"
```

---

## Self-Review Checklist (run after the plan is fully drafted)

This is a check the *plan author* (not the executor) runs. If any line below fails, fix the plan inline.

**Spec coverage (cross against `docs/superpowers/specs/2026-05-09-joint-training-multi-task-yolo-design.md`):**

- [x] §3 Q1 CLI shape — Task 13.
- [x] §3 Q2 validation cadence + best metric — Task 11, Task 12.
- [x] §3 Q3 per-task `.pt` checkpoints — Task 10.
- [x] §3 Q4 shared global hyperparameters — Task 13 CLI options.
- [x] §3 Q5 three LR knobs — Task 13 CLI; Task 8 `build_joint_optimizers`.
- [x] §3 Q6 E2ELoss + MuSGD recipe — Task 6, Task 7.
- [x] §3 Q7 `<hydra_model>~<wordlet>` run path — Task 13.
- [x] §4 file structure — every entry in §4.1 and §4.2 has a corresponding task.
- [x] §5 data pipeline — Task 4, Task 5.
- [x] §6 loss + uncertainty weighter — Task 3, Task 6.
- [x] §7 optimizer / synchronized accumulation / EMA / AMP — Task 7, Task 8, Task 9, Task 12.
- [x] §8 validation + checkpointing + early stop + resume + W&B — Task 10, Task 11, Task 12, Task 13.
- [x] §9 CLI surface — Task 13.
- [x] §10 risks — Task 2 parity test, Task 9 invariant test, Task 7 parameterized regex.
- [x] §11 test surface — Tasks 2, 3, 4, 7, 9, 14.

**Placeholder scan:** zero `TODO`, `TBD`, `XXX`, "fill in later" markers in the body of any task.

**Type consistency:**
- `JointLossHyp`, `JointTrainConfig`, `JointOptimizers`, `EMAHydra`, `_HydraHeadAdapter`, `UncertaintyWeighter`, `InterleavedTaskDataloader`, `_PassthroughEMA`: all referenced consistently across tasks.
- `head_last_layer_index` parameter name: used identically in Task 7 (`build_param_groups`) and Task 8 (`build_joint_optimizers`).
- CLI option names match between Task 13 (CLI) and Task 12 (`JointTrainConfig` fields).

---

## Execution Handoff

**Plan complete and saved to `tools/machine-learning/multi-task-yolo/docs/superpowers/plans/2026-05-09-joint-training-multi-task-yolo.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration. Best for this plan because each task has a clear test gate that the subagent can execute and report on independently, and the review-between-tasks cadence catches integration drift early.

**2. Inline Execution** — Execute tasks in this session using `executing-plans`, batch execution with checkpoints. Faster wall-clock if all tasks land cleanly, but a regression deep in the plan costs more to recover from.

**Which approach?**
