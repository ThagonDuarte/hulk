from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Literal

OptimizerName = Literal["AdamW", "SGD"]
TaskName = Literal["detection", "pose"]
UltralyticsTask = Literal["detect", "pose"]


@dataclass(frozen=True)
class JointTaskConfig:
    head_name: TaskName
    task: UltralyticsTask
    model_path: Path
    data_yaml: Path
    batch: int
    workers: int
    imgsz: int


@dataclass(frozen=True)
class JointOptimizerConfig:
    name: OptimizerName = "AdamW"
    backbone_lr: float = 1e-4
    head_lr: float = 1e-4
    uncertainty_lr: float = 1e-3
    weight_decay: float = 5e-4
    momentum: float = 0.9


@dataclass(frozen=True)
class JointSchedulerConfig:
    min_lr_ratio: float = 0.01


@dataclass(frozen=True)
class JointTrainConfig:
    foundation_path: Path
    tasks: dict[TaskName, JointTaskConfig]
    project_dir: Path
    run_name: str
    epochs: int
    fraction: float
    device: str | None
    amp: bool
    use_ema: bool
    clip_grad_norm: float
    validate_every: int
    save_period: int
    log_every: int
    seed: int
    deterministic: bool
    resume_checkpoint: Path | None
    optimizer: JointOptimizerConfig
    scheduler: JointSchedulerConfig

    @property
    def run_dir(self) -> Path:
        return self.project_dir / self.run_name

    def to_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["foundation_path"] = str(self.foundation_path)
        raw["project_dir"] = str(self.project_dir)
        raw["resume_checkpoint"] = (
            str(self.resume_checkpoint) if self.resume_checkpoint else None
        )
        raw["tasks"] = {
            key: {
                "head_name": value.head_name,
                "task": value.task,
                "model_path": str(value.model_path),
                "data_yaml": str(value.data_yaml),
                "batch": value.batch,
                "workers": value.workers,
                "imgsz": value.imgsz,
            }
            for key, value in self.tasks.items()
        }
        return raw
