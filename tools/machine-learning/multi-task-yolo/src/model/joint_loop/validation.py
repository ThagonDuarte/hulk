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
from typing import Any

from model.joint_loop.checkpoints import write_per_task_checkpoint
from model.joint_loop.optim import EMAHydra
from utils.model_naming import HydraModelName, TaskType
from validation.validator import ValidationConfig, validate_hydra_model

logger = logging.getLogger(__name__)

_PRIMARY_METRIC_KEY: dict[TaskType, str] = {
    TaskType.OBJECT: "metrics/mAP50-95(B)",
    TaskType.SEGMENTATION: "metrics/mAP50-95(M)",
    TaskType.POSE: "metrics/mAP50-95(P)",
}


def run_validation(
    *,
    ema: EMAHydra,
    hydra_model: HydraModelName,
    datasets_per_task: Mapping[TaskType, Path],
    head_source_paths: Mapping[TaskType, Path],
    run_dir: Path,
    imgsz: int,
    batch: int,
    device: Any,
    task_weights: Mapping[TaskType, float],
) -> tuple[
    float,
    dict[TaskType, float],
    dict[TaskType, dict[str, float]],
    dict[TaskType, list[Path]],
]:
    """Validate every task on EMA weights and return validation outputs.

    Validation artifacts (metrics, plots, curves) are written into
    ``run_dir / val / <task>``.  An **absolute** project path is passed
    to ultralytics so that its ``get_save_dir`` does not prepend
    ``RUNS_DIR / task /``.

    Returns:
        score: Combined weighted score.
        per_task_metric: Primary metric per task.
        all_metrics: All parsed metrics per task.
        task_visuals: Paths to validation images/curves per task.
    """
    per_task_metric: dict[TaskType, float] = {}
    all_metrics: dict[TaskType, dict[str, float]] = {}
    task_visuals: dict[TaskType, list[Path]] = {}

    # Absolute path prevents ultralytics from prepending RUNS_DIR/task.
    abs_run_dir = run_dir.resolve()

    with tempfile.TemporaryDirectory() as tmpdir:
        assets_dir = Path(tmpdir)

        for head in hydra_model.heads:
            task = head.task_type()
            head_pt_path = assets_dir / f"{head.name}.pt"
            backbone_pt_path = (
                assets_dir / f"{hydra_model.backbone.name}.pt"
            )

            single_task_hydra = HydraModelName(
                backbone=hydra_model.backbone,
                heads=[head],
                number_of_frozen_modules=(
                    hydra_model.number_of_frozen_modules
                ),
            )
            write_per_task_checkpoint(
                ema=ema,
                hydra_model=single_task_hydra,
                task=task,
                head_yolo_path=head_source_paths[task],
                output_path=head_pt_path,
            )
            if head_pt_path != backbone_pt_path:
                shutil.copy(head_pt_path, backbone_pt_path)

            # Ultralytics saves to project/name.  An absolute project
            # ensures get_save_dir uses it verbatim.
            val_name = f"val/{task}"
            config = ValidationConfig(
                data=str(datasets_per_task[task]),
                project=str(abs_run_dir),
                imgsz=imgsz,
                batch=batch,
                device=device,
            )
            try:
                validate_hydra_model(
                    single_task_hydra, config, assets_dir,
                    name_override=val_name,
                )
            except Exception:
                logger.exception(
                    "validation failed for task %s; recording NaN",
                    task,
                )
                per_task_metric[task] = float("nan")
                all_metrics[task] = {}
                task_visuals[task] = []
                continue

            # Both metrics.json and ultralytics plots land here.
            val_dir = abs_run_dir / val_name
            metrics_path = val_dir / "metrics.json"
            primary = _read_primary_metric(metrics_path, task)
            per_task_metric[task] = primary

            task_metrics = _read_task_metrics(metrics_path)
            if (
                not _is_nan(primary)
                and _PRIMARY_METRIC_KEY[task] not in task_metrics
            ):
                task_metrics[_PRIMARY_METRIC_KEY[task]] = primary
            all_metrics[task] = task_metrics

            task_visuals[task] = _gather_validation_assets(val_dir)

    score = sum(
        task_weights.get(t, 1.0) * v
        for t, v in per_task_metric.items()
        if not _is_nan(v)
    )
    return score, per_task_metric, all_metrics, task_visuals


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
            key,
            metrics_path,
            list(metrics.keys()),
        )
        return float("nan")
    return float(metrics[key])


def _read_task_metrics(metrics_path: Path) -> dict[str, float]:
    """Read metrics dictionary from the metrics JSON file."""
    if not metrics_path.exists():
        return {}
    try:
        with metrics_path.open() as f:
            return dict(json.load(f))
    except Exception:
        logger.exception("Failed to load metrics from %s", metrics_path)
        return {}


def _gather_validation_assets(val_dir: Path) -> list[Path]:
    """Scan validation directory for prediction plots and curves.

    Looks for predictions, confusion matrices, and curve plots.
    """
    visuals: list[Path] = []
    if val_dir.is_dir():
        for p in val_dir.glob("val_batch*_pred.jpg"):
            visuals.append(p)
        for p in val_dir.glob("confusion_matrix*.png"):
            visuals.append(p)
        for p in val_dir.glob("*curve.png"):
            visuals.append(p)
    visuals.sort(key=lambda p: p.name)
    return visuals


def _is_nan(value: float) -> bool:
    return value != value
