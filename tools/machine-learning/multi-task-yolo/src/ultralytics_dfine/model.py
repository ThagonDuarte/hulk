# ruff: noqa: TRY003

from pathlib import Path
from typing import Any, Literal

import torch

from ultralytics_dfine.config import (
    DFINERecipeConfig,
)
from ultralytics_dfine.data import load_dataset_yaml
from ultralytics_dfine.engine.exporter import DFINEExporter
from ultralytics_dfine.engine.predictor import DFINEPrediction, DFINEPredictor
from ultralytics_dfine.engine.trainer import (
    DFINETrainer,
    DFINETrainingConfig,
    seed_everything,
)
from ultralytics_dfine.engine.validator import DFINEValidator
from ultralytics_dfine.nn import DFINEDetectionModel


class DFINE:
    """Public train/val/predict/export API for the external D-FINE family."""

    def __init__(
        self,
        model: str | Path = "dfine-s",
        *,
        names: list[str] | None = None,
    ) -> None:
        self.source = model
        self.model: DFINEDetectionModel | None = None
        path = Path(model)
        if path.is_file():
            self.model = DFINEDetectionModel.from_checkpoint(path)
        elif str(model) != "dfine-s":
            raise ValueError("Only the dfine-s variant is currently supported")
        elif names is not None:
            self.model = DFINEDetectionModel.from_pretrained(names)

    def _require_model(self) -> DFINEDetectionModel:
        if self.model is None:
            raise RuntimeError(
                "D-FINE class names are unknown; provide names or call "
                "train(data=...)"
            )
        return self.model

    def load_official(
        self,
        _checkpoint: str | Path | None = None,
        *,
        weights: Literal["ema", "model"] = "ema",
        names: list[str] | None = None,
    ) -> "DFINE":
        if weights not in {"ema", "model"}:
            raise ValueError("weights must be 'ema' or 'model'")
        if _checkpoint is not None and Path(_checkpoint).is_file():
            raise ValueError(
                "Raw official .pth conversion requires an explicit class "
                "mapping; "
                "use the pinned converted dfine-small-coco checkpoint"
            )
        if names is None:
            raise ValueError(
                "Target class names are required for official weights"
            )
        self.model = DFINEDetectionModel.from_pretrained(names)
        return self

    def train(
        self,
        *,
        data: str | Path,
        output_dir: str | Path,
        epochs: int = 132,
        transition_epoch: int | None = None,
        batch: int = 32,
        device: str = "cuda",
        workers: int = 4,
        wandb_project: str = "multi-task-yolo-dfine",
        wandb_mode: Literal["online", "offline", "disabled"] = "online",
        name: str = "dfine-s",
        resume: str | Path | None = None,
        max_train_batches: int | None = None,
        max_val_batches: int | None = None,
        seed: int = 0,
        amp: bool = True,
    ) -> dict[str, object]:
        definition = load_dataset_yaml(data)
        seed_everything(seed)
        if self.model is None:
            self.model = DFINEDetectionModel.from_pretrained(definition.names)
        if self.model.names != definition.names:
            raise ValueError("Model and dataset class names differ")
        recipe_values = DFINERecipeConfig.for_epochs(
            epochs,
            transition_epoch=transition_epoch,
            total_batch_size=batch,
        )
        recipe = DFINERecipeConfig(**{**recipe_values.__dict__, "amp": amp})
        config = DFINETrainingConfig(
            data=Path(data),
            output_dir=Path(output_dir),
            recipe=recipe,
            device=device,
            workers=workers,
            seed=seed,
            run_name=name,
            wandb_project=wandb_project,
            wandb_mode=wandb_mode,
            resume=Path(resume) if resume is not None else None,
            max_train_batches=max_train_batches,
            max_val_batches=max_val_batches,
        )
        trainer = DFINETrainer(self.model, config)
        result = trainer.fit()
        self.model = trainer.ema.module
        return result

    def val(
        self,
        *,
        data: str | Path,
        split: str = "val",
        device: str = "cuda",
        batch: int = 16,
        workers: int = 4,
        render_dir: str | Path | None = None,
        max_batches: int | None = None,
    ) -> dict[str, float | list[float]]:
        validator = DFINEValidator(
            self._require_model(),
            data,
            device=device,
            batch_size=batch,
            workers=workers,
        )
        return validator.run(
            split=split,
            render_dir=render_dir,
            max_batches=max_batches,
        )

    def predict(
        self,
        source: str | Path,
        *,
        device: str = "cuda",
        confidence: float = 0.25,
        output_dir: str | Path | None = None,
        max_images: int | None = None,
    ) -> list[DFINEPrediction]:
        predictor = DFINEPredictor(
            self._require_model(),
            device=device,
            confidence=confidence,
        )
        return predictor.predict(
            source,
            output_dir=output_dir,
            max_images=max_images,
        )

    def export(
        self,
        path: str | Path,
        *,
        export_mode: Literal["raw", "end2end"] = "end2end",
        opset: int = 17,
        device: str = "cpu",
        verify: bool = True,
    ) -> Path:
        return DFINEExporter(self._require_model()).export_onnx(
            path,
            mode=export_mode,
            opset=opset,
            device=device,
            verify=verify,
        )

    def freeze_roles(self, roles: list[str]) -> None:
        self._require_model().freeze_roles(roles)

    def save(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        torch.save(self._require_model().checkpoint_payload(), destination)
        return destination

    @property
    def task_map(self) -> dict[str, dict[str, Any]]:
        return {
            "detect": {
                "model": DFINEDetectionModel,
                "trainer": DFINETrainer,
                "validator": DFINEValidator,
                "predictor": DFINEPredictor,
                "exporter": DFINEExporter,
            }
        }
