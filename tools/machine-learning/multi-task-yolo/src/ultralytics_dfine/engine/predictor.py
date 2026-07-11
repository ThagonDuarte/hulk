# ruff: noqa: TRY003

from dataclasses import dataclass
from pathlib import Path

import torch
import torchvision.transforms.v2 as transforms
from PIL import Image
from torch import Tensor

from ultralytics_dfine.engine.render import render_detections
from ultralytics_dfine.nn import DFINEDetectionModel


@dataclass(frozen=True)
class DFINEPrediction:
    source: Path
    detections: Tensor
    rendered_path: Path | None


def _source_images(source: str | Path) -> list[Path]:
    path = Path(source).expanduser()
    if path.is_file():
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(f"Prediction source does not exist: {path}")
    suffixes = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}
    return sorted(
        item
        for item in path.rglob("*")
        if item.is_file() and item.suffix.lower() in suffixes
    )


class DFINEPredictor:
    def __init__(
        self,
        model: DFINEDetectionModel,
        *,
        device: str | torch.device = "cuda",
        confidence: float = 0.25,
    ) -> None:
        self.model = model
        self.device = torch.device(device)
        self.confidence = confidence
        self._transform = transforms.Compose(
            [
                transforms.Resize(
                    (
                        model.architecture.image_size,
                        model.architecture.image_size,
                    ),
                    antialias=True,
                ),
                transforms.ToImage(),
                transforms.ToDtype(torch.float32, scale=True),
            ]
        )

    @torch.no_grad()
    def predict(
        self,
        source: str | Path,
        *,
        output_dir: str | Path | None = None,
        max_images: int | None = None,
    ) -> list[DFINEPrediction]:
        sources = _source_images(source)
        if max_images is not None:
            sources = sources[:max_images]
        self.model.to(self.device).eval()
        predictions = []
        for path in sources:
            with Image.open(path) as loaded_image:
                image = loaded_image.convert("RGB")
            width, height = image.size
            tensor = self._transform(image).as_subclass(Tensor)
            raw = self.model.forward_raw(tensor[None].to(self.device))
            normalized = self.model.postprocessor(raw)
            detections = self.model.postprocessor.to_pixel_xyxy(
                normalized,
                torch.tensor([[height, width]], device=self.device),
            )[0]
            detections = detections[detections[:, 4] >= self.confidence].cpu()
            rendered_path = None
            if output_dir is not None:
                rendered_path = render_detections(
                    path,
                    detections,
                    self.model.names,
                    Path(output_dir) / f"{path.stem}.jpg",
                    confidence=self.confidence,
                )
            predictions.append(
                DFINEPrediction(
                    source=path,
                    detections=detections,
                    rendered_path=rendered_path,
                )
            )
        return predictions
