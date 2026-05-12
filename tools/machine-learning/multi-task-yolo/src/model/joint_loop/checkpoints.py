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
    task: TaskType,  # noqa: ARG001
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


def _ema_root_module(ema: EMAHydra, _hydra_model: HydraModelName) -> Any:
    """Return a module-like object that exposes the EMA backbone modules.

    `get_backbone()` operates on an Ultralytics `DetectionModel`. The EMA
    snapshot is a `Hydra`, whose `shared_backbone` is the equivalent
    nn.ModuleList. We construct a thin shim with `.model` and `.yaml`
    attributes that `get_backbone(..., number_of_frozen_modules=...)` can
    consume - when `number_of_frozen_modules` is provided explicitly,
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
    if isinstance(value, list | tuple):
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
