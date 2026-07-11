# ruff: noqa: TRY003

from pathlib import Path

import torch
from PIL import Image, ImageDraw, ImageFont
from torch import Tensor

COLORS = (
    "#ff4d4d",
    "#33cc99",
    "#4d94ff",
    "#ffcc33",
    "#cc66ff",
    "#ff8533",
    "#66d9ff",
)


def render_detections(
    image_path: str | Path,
    detections: Tensor,
    names: list[str],
    output_path: str | Path,
    *,
    confidence: float = 0.25,
    ground_truth: Tensor | None = None,
    ground_truth_labels: Tensor | None = None,
    max_detections: int = 50,
) -> Path:
    source = Path(image_path)
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with Image.open(source) as loaded_image:
        image = loaded_image.convert("RGB")
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default(size=max(12, image.width // 80))

    if ground_truth is not None and ground_truth_labels is not None:
        for box, label in zip(
            ground_truth.detach().cpu(),
            ground_truth_labels.detach().cpu(),
            strict=True,
        ):
            class_index = int(label.item())
            color = COLORS[class_index % len(COLORS)]
            coordinates = tuple(float(value) for value in box.tolist())
            draw.rectangle(coordinates, outline=color, width=2)
            draw.text(
                (coordinates[0] + 2, coordinates[1] + 2),
                f"GT {names[class_index]}",
                fill=color,
                font=font,
                stroke_width=2,
                stroke_fill="black",
            )

    count = 0
    for detection in detections.detach().cpu():
        score = float(detection[4].item())
        if score < confidence:
            continue
        class_index = int(detection[5].item())
        if not 0 <= class_index < len(names):
            raise ValueError(f"Prediction class {class_index} is out of range")
        color = COLORS[class_index % len(COLORS)]
        coordinates = tuple(float(value) for value in detection[:4].tolist())
        draw.rectangle(coordinates, outline=color, width=4)
        draw.text(
            (coordinates[0] + 2, max(0, coordinates[1] - 18)),
            f"{names[class_index]} {score:.2f}",
            fill="white",
            font=font,
            stroke_width=2,
            stroke_fill="black",
        )
        count += 1
        if count >= max_detections:
            break

    image.save(destination, quality=92)
    return destination


def normalized_targets_to_pixel_xyxy(
    boxes: Tensor,
    original_size: Tensor,
) -> Tensor:
    center_x, center_y, width, height = boxes.unbind(-1)
    result = torch.stack(
        (
            center_x - width / 2,
            center_y - height / 2,
            center_x + width / 2,
            center_y + height / 2,
        ),
        dim=-1,
    )
    scale = original_size[[1, 0, 1, 0]].to(result)
    return result * scale
