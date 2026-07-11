# ruff: noqa: TRY003

from pathlib import Path
from typing import Any

import torch
from torch import Tensor
from torch.utils.data import DataLoader
from torchmetrics.detection.mean_ap import MeanAveragePrecision
from tqdm import tqdm

from ultralytics_dfine.data import DatasetTarget, DFINEDataset
from ultralytics_dfine.engine.render import (
    normalized_targets_to_pixel_xyxy,
    render_detections,
)
from ultralytics_dfine.nn import DFINEDetectionModel


def validation_collate(
    items: list[tuple[Tensor, DatasetTarget]],
) -> tuple[Tensor, list[DatasetTarget]]:
    return torch.stack([item[0] for item in items]), [item[1] for item in items]


def _move_targets(
    targets: list[DatasetTarget],
    device: torch.device,
) -> list[DatasetTarget]:
    return [
        {
            "labels": target["labels"].to(device),
            "boxes": target["boxes"].to(device),
            "orig_size": target["orig_size"].to(device),
            "image_id": target["image_id"].to(device),
            "path": target["path"],
        }
        for target in targets
    ]


def _metric_value(value: object) -> float | list[float]:
    if isinstance(value, Tensor):
        if value.numel() == 1:
            return float(value.item())
        return [float(item) for item in value.cpu().tolist()]
    raise TypeError(f"Unsupported metric type: {type(value)}")


class DFINEValidator:
    def __init__(
        self,
        model: DFINEDetectionModel,
        data: str | Path,
        *,
        device: str | torch.device = "cuda",
        batch_size: int = 16,
        workers: int = 4,
        confidence: float = 0.001,
        render_confidence: float = 0.25,
    ) -> None:
        self.model = model
        self.data = Path(data)
        self.device = torch.device(device)
        self.batch_size = batch_size
        self.workers = workers
        self.confidence = confidence
        self.render_confidence = render_confidence

    @torch.no_grad()
    def run(
        self,
        *,
        split: str = "val",
        render_dir: str | Path | None = None,
        epoch: int = -1,
        wandb_run: Any | None = None,
        max_batches: int | None = None,
        render_count: int = 4,
    ) -> dict[str, float | list[float]]:
        if split not in {"val", "test"}:
            raise ValueError("D-FINE validation split must be val or test")
        dataset = DFINEDataset(self.data, split)  # type: ignore[arg-type]
        loader = DataLoader(
            dataset,
            batch_size=self.batch_size,
            shuffle=False,
            num_workers=self.workers,
            pin_memory=self.device.type == "cuda",
            collate_fn=validation_collate,
        )
        metric = MeanAveragePrecision(
            box_format="xyxy",
            iou_type="bbox",
            class_metrics=True,
            backend="faster_coco_eval",
            sync_on_compute=False,
        )
        metric.warn_on_many_detections = False
        self.model.to(self.device).eval()
        rendered: list[Path] = []
        progress = tqdm(loader, desc="Validating", leave=False)
        for batch_index, (images, targets) in enumerate(progress):
            if max_batches is not None and batch_index >= max_batches:
                break
            images = images.to(self.device, non_blocking=True)
            device_targets = _move_targets(targets, self.device)
            raw = self.model.forward_raw(images)
            normalized = self.model.postprocessor(raw)
            original_sizes = torch.stack(
                [target["orig_size"] for target in device_targets]
            )
            detections = self.model.postprocessor.to_pixel_xyxy(
                normalized,
                original_sizes,
            )
            predictions = []
            ground_truth = []
            for detection, target in zip(
                detections,
                device_targets,
                strict=True,
            ):
                keep = detection[:, 4] >= self.confidence
                predictions.append(
                    {
                        "boxes": detection[keep, :4].cpu(),
                        "scores": detection[keep, 4].cpu(),
                        "labels": detection[keep, 5].long().cpu(),
                    }
                )
                ground_truth.append(
                    {
                        "boxes": normalized_targets_to_pixel_xyxy(
                            target["boxes"],
                            target["orig_size"],
                        ).cpu(),
                        "labels": target["labels"].long().cpu(),
                    }
                )
            metric.update(predictions, ground_truth)

            if render_dir is not None and len(rendered) < render_count:
                for detection, target in zip(
                    detections,
                    device_targets,
                    strict=True,
                ):
                    if len(rendered) >= render_count:
                        break
                    source = Path(target["path"])
                    destination = (
                        Path(render_dir)
                        / f"epoch_{epoch:03d}"
                        / f"{source.stem}.jpg"
                    )
                    rendered.append(
                        render_detections(
                            source,
                            detection,
                            dataset.names,
                            destination,
                            confidence=self.render_confidence,
                            ground_truth=normalized_targets_to_pixel_xyxy(
                                target["boxes"],
                                target["orig_size"],
                            ),
                            ground_truth_labels=target["labels"],
                        )
                    )

        computed = metric.compute()
        metrics = {
            "metrics/mAP50-95(B)": _metric_value(computed["map"]),
            "metrics/mAP50(B)": _metric_value(computed["map_50"]),
            "metrics/mAP75(B)": _metric_value(computed["map_75"]),
            "metrics/mAR100(B)": _metric_value(computed["mar_100"]),
            "metrics/classes": _metric_value(computed["classes"]),
            "metrics/map_per_class": _metric_value(computed["map_per_class"]),
            "metrics/mar_100_per_class": _metric_value(
                computed["mar_100_per_class"]
            ),
        }
        if wandb_run is not None and rendered:
            import wandb

            wandb_run.log(
                {
                    "validation/rendered": [
                        wandb.Image(str(path), caption=path.stem)
                        for path in rendered
                    ],
                    "epoch": epoch,
                }
            )
        return metrics
