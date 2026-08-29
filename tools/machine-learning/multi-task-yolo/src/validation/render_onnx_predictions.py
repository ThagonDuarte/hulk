from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from enum import Enum
from pathlib import Path

import click
import cv2
import numpy as np
import onnxruntime as ort

MODEL_INPUT_NAME = "raw_bytes_input"
OBJECT_OUTPUT_NAME = "object_output"
PERSON_POSE_OUTPUT_NAME = "person_pose_output"
ROBOT_POSE_OUTPUT_NAME = "robot_pose_output"
MODEL_OUTPUT_NAMES = (
    OBJECT_OUTPUT_NAME,
    PERSON_POSE_OUTPUT_NAME,
    ROBOT_POSE_OUTPUT_NAME,
)
IMAGE_SUFFIXES = {
    ".bmp",
    ".dng",
    ".jpeg",
    ".jpg",
    ".mpo",
    ".png",
    ".tif",
    ".tiff",
    ".webp",
}
CLASS_COLORS_BGR = (
    (56, 140, 255),
    (88, 214, 141),
    (255, 112, 67),
    (171, 71, 188),
    (38, 198, 218),
    (255, 202, 40),
    (239, 83, 80),
    (92, 107, 192),
    (102, 187, 106),
)
LABEL_FONT_SCALE = 0.32
LABEL_PAD_X = 4
LABEL_PAD_Y = 3
LABEL_BACKGROUND_ALPHA = 0.65
OBJECT_CLASS_NAMES = (
    "Ball",
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "Robot",
    "TSpot",
    "XSpot",
)
PERSON_POSE_CLASS_NAMES = ("Person pose",)
ROBOT_POSE_CLASS_NAMES = ("Robot pose",)
PERSON_POSE_COLOR_INDEX = len(OBJECT_CLASS_NAMES)
ROBOT_POSE_COLOR_INDEX = PERSON_POSE_COLOR_INDEX + 1
COCO_SKELETON = (
    (0, 1),
    (0, 2),
    (1, 3),
    (2, 4),
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (5, 7),
    (6, 8),
    (7, 9),
    (8, 10),
    (11, 13),
    (12, 14),
    (13, 15),
    (14, 16),
)
ROBOT_SKELETON = (
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (1, 5),
    (5, 6),
    (6, 7),
    (2, 8),
    (5, 11),
    (8, 11),
    (8, 9),
    (9, 10),
    (11, 12),
    (12, 13),
)


class ModelHead(Enum):
    OBJECT = "object"
    PERSON_POSE = "person_pose"
    ROBOT_POSE = "robot_pose"


@dataclass(frozen=True)
class OnnxRenderConfig:
    confidence_threshold: float
    nms_iou_threshold: float
    keypoint_confidence_threshold: float
    num_images: int
    draw_labels: bool
    intra_threads: int
    execution_provider: str
    filter_overlapping_person_poses: bool
    person_robot_overlap_iou: float


@dataclass(frozen=True)
class Prediction:
    cls: int
    conf: float
    box_xyxy: np.ndarray
    keypoints: np.ndarray | None = None


def sample_images(images: Sequence[Path], num_images: int) -> list[Path]:
    if num_images == -1 or num_images >= len(images):
        return list(images)
    if num_images <= 0:
        raise click.BadParameter(  # noqa: TRY003
            "--num-images must be -1 or a positive integer",
        )
    if num_images == 1:
        return [images[0]]

    step = (len(images) - 1) / (num_images - 1)
    indices = [round(index * step) for index in range(num_images)]
    return [images[index] for index in indices]


def load_source_images(
    source_image_dir: Path,
    *,
    num_images: int,
) -> list[Path]:
    images = sorted(
        path
        for path in source_image_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not images:
        raise click.ClickException(  # noqa: TRY003
            f"No images found under source directory: {source_image_dir}",
        )
    return sample_images(images, num_images)


def relative_source_image_path(image_path: Path, source_root: Path) -> Path:
    try:
        return image_path.resolve().relative_to(source_root.resolve())
    except ValueError:
        return Path(image_path.name)


def save_image(path: Path, image: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise OSError(path)


def prediction_area(prediction: Prediction) -> float:
    x1, y1, x2, y2 = prediction.box_xyxy
    return max(0.0, float(x2 - x1)) * max(0.0, float(y2 - y1))


def model_class_color(
    head: ModelHead,
    class_index: int = 0,
) -> tuple[int, int, int]:
    if head == ModelHead.OBJECT:
        color_index = class_index
    elif head == ModelHead.PERSON_POSE:
        color_index = PERSON_POSE_COLOR_INDEX
    else:
        color_index = ROBOT_POSE_COLOR_INDEX

    if not 0 <= color_index < len(CLASS_COLORS_BGR):
        raise ValueError(  # noqa: TRY003
            f"No unique color for {head.value} class {class_index}"
        )
    return CLASS_COLORS_BGR[color_index]


def model_class_colors() -> dict[str, tuple[int, int, int]]:
    colors = {
        name: model_class_color(ModelHead.OBJECT, class_index)
        for class_index, name in enumerate(OBJECT_CLASS_NAMES)
    }
    colors[PERSON_POSE_CLASS_NAMES[0]] = model_class_color(
        ModelHead.PERSON_POSE,
    )
    colors[ROBOT_POSE_CLASS_NAMES[0]] = model_class_color(
        ModelHead.ROBOT_POSE,
    )
    return colors


def bgr_to_nv12(image: np.ndarray) -> np.ndarray:
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(  # noqa: TRY003
            "Expected a BGR image with three channels",
        )

    height, width = image.shape[:2]
    if not height % 32 == width % 32 == 0:
        raise ValueError(  # noqa: TRY003
            f"Image dimensions must be multiples of 32, got {width}x{height}",
        )

    blue, green, red = cv2.split(image.astype(np.float32))
    luminance = 0.299 * red + 0.587 * green + 0.114 * blue
    chroma_u = -0.168736 * red - 0.331264 * green + 0.5 * blue + 128
    chroma_v = 0.5 * red - 0.418688 * green - 0.081312 * blue + 128
    chroma_u = chroma_u.reshape(height // 2, 2, width // 2, 2).mean(
        axis=(1, 3),
    )
    chroma_v = chroma_v.reshape(height // 2, 2, width // 2, 2).mean(
        axis=(1, 3),
    )
    chroma = np.stack((chroma_u, chroma_v), axis=-1)
    flat_nv12 = np.concatenate(
        (
            quantize_bytes(luminance).ravel(),
            quantize_bytes(chroma).ravel(),
        ),
    )
    return flat_nv12.reshape(height // 2, width // 2, 6)


def quantize_bytes(values: np.ndarray) -> np.ndarray:
    return np.clip(np.rint(values), 0, 255).astype(np.uint8)


def predictions_from_output(
    output: np.ndarray,
    *,
    confidence_threshold: float,
    keypoint_count: int = 0,
) -> list[Prediction]:
    if output.ndim != 3 or output.shape[0] != 1:
        raise ValueError(  # noqa: TRY003
            f"Expected a [1, N, C] output, got {output.shape}",
        )

    expected_values = 6 + keypoint_count * 3
    if output.shape[2] != expected_values:
        raise ValueError(  # noqa: TRY003
            f"Expected {expected_values} values per prediction, "
            f"got {output.shape[2]}",
        )

    predictions = []
    for row in output[0]:
        confidence = float(row[4])
        if not np.isfinite(confidence) or confidence < confidence_threshold:
            continue
        keypoints = (
            row[6:].reshape(keypoint_count, 3).copy()
            if keypoint_count
            else None
        )
        predictions.append(
            Prediction(
                cls=int(row[5]),
                conf=confidence,
                box_xyxy=row[:4].copy(),
                keypoints=keypoints,
            ),
        )
    return predictions


def bounding_box_iou(first: Prediction, second: Prediction) -> float:
    x1 = max(float(first.box_xyxy[0]), float(second.box_xyxy[0]))
    y1 = max(float(first.box_xyxy[1]), float(second.box_xyxy[1]))
    x2 = min(float(first.box_xyxy[2]), float(second.box_xyxy[2]))
    y2 = min(float(first.box_xyxy[3]), float(second.box_xyxy[3]))
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    union = prediction_area(first) + prediction_area(second) - intersection
    return intersection / union if union > 0 else 0.0


def non_maximum_suppression(
    predictions: list[Prediction],
    maximum_iou: float,
) -> list[Prediction]:
    pending = sorted(predictions, key=lambda prediction: prediction.conf)
    retained = []
    while pending:
        selected = pending.pop()
        pending = [
            candidate
            for candidate in pending
            if bounding_box_iou(selected, candidate) < maximum_iou
        ]
        retained.append(selected)
    return retained


def filter_person_poses_overlapping_robots(
    person_poses: list[Prediction],
    robot_poses: list[Prediction],
    maximum_iou: float,
) -> list[Prediction]:
    return [
        person_pose
        for person_pose in person_poses
        if not any(
            bounding_box_iou(person_pose, robot_pose) > maximum_iou
            for robot_pose in robot_poses
        )
    ]


def clipped_box(
    box_xyxy: np.ndarray,
    width: int,
    height: int,
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box_xyxy
    return (
        round(float(np.clip(x1, 0, width - 1))),
        round(float(np.clip(y1, 0, height - 1))),
        round(float(np.clip(x2, 0, width - 1))),
        round(float(np.clip(y2, 0, height - 1))),
    )


def draw_bounding_box(
    image: np.ndarray,
    box_xyxy: np.ndarray,
    color: tuple[int, int, int],
) -> None:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = clipped_box(box_xyxy, width, height)
    if x2 <= x1 or y2 <= y1:
        return
    cv2.rectangle(
        image,
        (x1, y1),
        (x2, y2),
        color,
        2,
        lineType=cv2.LINE_AA,
    )


def class_name(
    names: Mapping[int, str] | Sequence[str],
    class_index: int,
) -> str:
    if isinstance(names, Mapping):
        return str(names.get(class_index, class_index))
    if 0 <= class_index < len(names):
        return str(names[class_index])
    return str(class_index)


def draw_prediction_label(
    image: np.ndarray,
    prediction: Prediction,
    names: Mapping[int, str] | Sequence[str],
    color: tuple[int, int, int],
) -> None:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = clipped_box(prediction.box_xyxy, width, height)
    if x2 <= x1 or y2 <= y1:
        return

    text = f"{class_name(names, prediction.cls)} {prediction.conf:.2f}"
    text_size, baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        LABEL_FONT_SCALE,
        1,
    )
    text_width, text_height = text_size
    label_width = text_width + LABEL_PAD_X * 2
    label_height = text_height + baseline + LABEL_PAD_Y * 2
    label_x1 = max(0, min(x1, width - label_width))
    label_y1 = y1 - label_height
    if label_y1 < 0:
        label_y1 = min(y1, height - label_height)
    label_x2 = label_x1 + label_width
    label_y2 = label_y1 + label_height

    region = image[label_y1:label_y2, label_x1:label_x2]
    overlay = np.full_like(region, color)
    cv2.addWeighted(
        overlay,
        LABEL_BACKGROUND_ALPHA,
        region,
        1 - LABEL_BACKGROUND_ALPHA,
        0,
        dst=region,
    )
    cv2.putText(
        image,
        text,
        (label_x1 + LABEL_PAD_X, label_y1 + LABEL_PAD_Y + text_height),
        cv2.FONT_HERSHEY_SIMPLEX,
        LABEL_FONT_SCALE,
        (255, 255, 255),
        1,
        lineType=cv2.LINE_AA,
    )


def draw_pose_skeleton(
    image: np.ndarray,
    prediction: Prediction,
    color: tuple[int, int, int],
    *,
    skeleton: Sequence[tuple[int, int]],
    confidence_threshold: float,
) -> None:
    keypoints = prediction.keypoints
    if keypoints is None or len(keypoints) == 0:
        return

    visible = np.isfinite(keypoints).all(axis=1)
    visible &= keypoints[:, 2] > confidence_threshold
    for start, end in skeleton:
        if not visible[start] or not visible[end]:
            continue
        start_point = tuple(np.rint(keypoints[start, :2]).astype(int))
        end_point = tuple(np.rint(keypoints[end, :2]).astype(int))
        cv2.line(
            image,
            start_point,
            end_point,
            color,
            2,
            lineType=cv2.LINE_AA,
        )

    for index, keypoint in enumerate(keypoints):
        if not visible[index]:
            continue
        point = tuple(np.rint(keypoint[:2]).astype(int))
        cv2.circle(image, point, 3, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(image, point, 2, color, -1, cv2.LINE_AA)


def render_onnx_predictions(
    image: np.ndarray,
    *,
    objects: list[Prediction],
    person_poses: list[Prediction],
    robot_poses: list[Prediction],
    draw_labels: bool,
    keypoint_confidence_threshold: float,
) -> np.ndarray:
    rendered = image.copy()
    for prediction in sorted(objects, key=prediction_area, reverse=True):
        color = model_class_color(ModelHead.OBJECT, prediction.cls)
        draw_bounding_box(rendered, prediction.box_xyxy, color)
        if draw_labels:
            draw_prediction_label(
                rendered,
                prediction,
                OBJECT_CLASS_NAMES,
                color,
            )

    draw_pose_predictions(
        rendered,
        person_poses,
        head=ModelHead.PERSON_POSE,
        names=PERSON_POSE_CLASS_NAMES,
        skeleton=COCO_SKELETON,
        draw_labels=draw_labels,
        keypoint_confidence_threshold=keypoint_confidence_threshold,
    )
    draw_pose_predictions(
        rendered,
        robot_poses,
        head=ModelHead.ROBOT_POSE,
        names=ROBOT_POSE_CLASS_NAMES,
        skeleton=ROBOT_SKELETON,
        draw_labels=draw_labels,
        keypoint_confidence_threshold=keypoint_confidence_threshold,
    )
    return rendered


def draw_pose_predictions(
    image: np.ndarray,
    predictions: list[Prediction],
    *,
    head: ModelHead,
    names: tuple[str, ...],
    skeleton: tuple[tuple[int, int], ...],
    draw_labels: bool,
    keypoint_confidence_threshold: float,
) -> None:
    color = model_class_color(head)
    for prediction in sorted(predictions, key=prediction_area, reverse=True):
        draw_pose_skeleton(
            image,
            prediction,
            color,
            skeleton=skeleton,
            confidence_threshold=keypoint_confidence_threshold,
        )
        if draw_labels:
            draw_prediction_label(image, prediction, names, color)


def extract_predictions(
    outputs: dict[str, np.ndarray],
    config: OnnxRenderConfig,
) -> dict[ModelHead, list[Prediction]]:
    predictions = {
        ModelHead.OBJECT: predictions_from_output(
            outputs[OBJECT_OUTPUT_NAME],
            confidence_threshold=config.confidence_threshold,
        ),
        ModelHead.PERSON_POSE: predictions_from_output(
            outputs[PERSON_POSE_OUTPUT_NAME],
            confidence_threshold=config.confidence_threshold,
            keypoint_count=17,
        ),
        ModelHead.ROBOT_POSE: predictions_from_output(
            outputs[ROBOT_POSE_OUTPUT_NAME],
            confidence_threshold=config.confidence_threshold,
            keypoint_count=14,
        ),
    }
    retained = {
        head: non_maximum_suppression(
            head_predictions, config.nms_iou_threshold
        )
        for head, head_predictions in predictions.items()
    }
    if config.filter_overlapping_person_poses:
        retained[ModelHead.PERSON_POSE] = (
            filter_person_poses_overlapping_robots(
                retained[ModelHead.PERSON_POSE],
                retained[ModelHead.ROBOT_POSE],
                config.person_robot_overlap_iou,
            )
        )
    return retained


def choose_execution_provider(
    requested: str,
    available: Sequence[str] | None = None,
) -> str:
    available_providers = set(available or ort.get_available_providers())
    provider_names = {
        "tensorrt": "TensorrtExecutionProvider",
        "cuda": "CUDAExecutionProvider",
        "cpu": "CPUExecutionProvider",
    }
    if requested == "auto":
        for provider in (
            "TensorrtExecutionProvider",
            "CUDAExecutionProvider",
            "CPUExecutionProvider",
        ):
            if provider in available_providers:
                return provider
        raise click.ClickException("No ONNX Runtime provider is available")  # noqa: TRY003

    provider = provider_names[requested]
    if provider not in available_providers:
        available_text = ", ".join(sorted(available_providers))
        raise click.ClickException(  # noqa: TRY003
            f"Requested provider {provider} is unavailable. "
            f"Available providers: {available_text}",
        )
    return provider


def create_session(
    model_path: Path,
    intra_threads: int,
    execution_provider: str,
) -> ort.InferenceSession:
    if execution_provider != "CPUExecutionProvider" and hasattr(
        ort, "preload_dlls"
    ):
        ort.preload_dlls()
    options = ort.SessionOptions()
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    options.intra_op_num_threads = intra_threads
    providers = [execution_provider]
    if execution_provider != "CPUExecutionProvider":
        providers.append("CPUExecutionProvider")
    session = ort.InferenceSession(
        model_path,
        sess_options=options,
        providers=providers,
    )
    if execution_provider not in session.get_providers():
        active = ", ".join(session.get_providers())
        raise click.ClickException(  # noqa: TRY003
            f"Requested provider {execution_provider} did not activate. "
            f"Active providers: {active}",
        )
    return session


def validate_config(
    *,
    confidence_threshold: float,
    nms_iou_threshold: float,
    keypoint_confidence_threshold: float,
    person_robot_overlap_iou: float,
    num_images: int,
    intra_threads: int,
) -> None:
    for name, value in (
        ("--confidence-threshold", confidence_threshold),
        ("--nms-iou-threshold", nms_iou_threshold),
        ("--keypoint-confidence-threshold", keypoint_confidence_threshold),
        ("--person-robot-overlap-iou", person_robot_overlap_iou),
    ):
        if not 0 <= value <= 1:
            raise click.BadParameter(  # noqa: TRY003
                f"{name} must be between 0 and 1",
            )
    if num_images != -1 and num_images <= 0:
        raise click.BadParameter(  # noqa: TRY003
            "--num-images must be -1 or a positive integer",
        )
    if intra_threads <= 0:
        raise click.BadParameter(  # noqa: TRY003
            "--intra-threads must be positive",
        )


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Render all heads of an exported three-head Hydra ONNX model.",
)
@click.argument(
    "model_path",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.argument(
    "source_image_dir",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
)
@click.argument("output_dir", type=click.Path(path_type=Path))
@click.option(
    "--confidence-threshold",
    type=float,
    default=0.25,
    show_default=True,
)
@click.option(
    "--nms-iou-threshold",
    type=float,
    default=0.4,
    show_default=True,
)
@click.option(
    "--keypoint-confidence-threshold",
    type=float,
    default=0.25,
    show_default=True,
)
@click.option(
    "--num-images",
    type=int,
    default=-1,
    show_default=True,
    help="Uniformly sample this many images. -1 renders every image.",
)
@click.option(
    "--labels/--no-labels",
    "draw_labels",
    default=True,
    show_default=True,
    help="Draw compact class and confidence labels.",
)
@click.option(
    "--intra-threads",
    type=int,
    default=2,
    show_default=True,
    help="ONNX Runtime CPU inference threads and GPU fallback threads.",
)
@click.option(
    "--provider",
    type=click.Choice(("auto", "tensorrt", "cuda", "cpu")),
    default="auto",
    show_default=True,
    help="ONNX Runtime provider. Auto prefers TensorRT, then CUDA.",
)
@click.option(
    "--filter-overlapping-person-poses/--keep-overlapping-person-poses",
    default=True,
    show_default=True,
    help="Remove person poses that strongly overlap a robot pose.",
)
@click.option(
    "--person-robot-overlap-iou",
    type=float,
    default=0.8,
    show_default=True,
    help="IoU above which an overlapping person pose is removed.",
)
def main(
    model_path: Path,
    source_image_dir: Path,
    output_dir: Path,
    *,
    confidence_threshold: float,
    nms_iou_threshold: float,
    keypoint_confidence_threshold: float,
    num_images: int,
    draw_labels: bool,
    intra_threads: int,
    provider: str,
    filter_overlapping_person_poses: bool,
    person_robot_overlap_iou: float,
) -> None:
    validate_config(
        confidence_threshold=confidence_threshold,
        nms_iou_threshold=nms_iou_threshold,
        keypoint_confidence_threshold=keypoint_confidence_threshold,
        person_robot_overlap_iou=person_robot_overlap_iou,
        num_images=num_images,
        intra_threads=intra_threads,
    )
    execution_provider = choose_execution_provider(provider)
    config = OnnxRenderConfig(
        confidence_threshold=confidence_threshold,
        nms_iou_threshold=nms_iou_threshold,
        keypoint_confidence_threshold=keypoint_confidence_threshold,
        num_images=num_images,
        draw_labels=draw_labels,
        intra_threads=intra_threads,
        execution_provider=execution_provider,
        filter_overlapping_person_poses=filter_overlapping_person_poses,
        person_robot_overlap_iou=person_robot_overlap_iou,
    )
    image_paths = load_source_images(
        source_image_dir,
        num_images=num_images,
    )
    click.echo(f"Using {execution_provider}")
    session = create_session(model_path, intra_threads, execution_provider)
    available_outputs = {output.name for output in session.get_outputs()}
    missing_outputs = set(MODEL_OUTPUT_NAMES) - available_outputs
    if missing_outputs:
        missing = ", ".join(sorted(missing_outputs))
        raise click.ClickException(  # noqa: TRY003
            f"Model outputs are missing: {missing}",
        )

    totals = dict.fromkeys(ModelHead, 0)
    saved_images = []
    image_records = []
    for index, image_path in enumerate(image_paths, start=1):
        image = cv2.imread(str(image_path))
        if image is None:
            raise click.ClickException(  # noqa: TRY003
                f"Could not read image: {image_path}",
            )
        values = session.run(
            list(MODEL_OUTPUT_NAMES),
            {MODEL_INPUT_NAME: bgr_to_nv12(image)},
        )
        outputs = dict(zip(MODEL_OUTPUT_NAMES, values, strict=True))
        predictions = extract_predictions(outputs, config)
        for head, head_predictions in predictions.items():
            totals[head] += len(head_predictions)
        rendered = render_onnx_predictions(
            image,
            objects=predictions[ModelHead.OBJECT],
            person_poses=predictions[ModelHead.PERSON_POSE],
            robot_poses=predictions[ModelHead.ROBOT_POSE],
            draw_labels=draw_labels,
            keypoint_confidence_threshold=keypoint_confidence_threshold,
        )
        output_path = output_dir / relative_source_image_path(
            image_path,
            source_image_dir,
        )
        save_image(output_path, rendered)
        saved_images.append(str(output_path))
        image_records.append(
            {
                "source": str(image_path),
                "output": str(output_path),
                "prediction_count": {
                    head.value: len(predictions[head]) for head in ModelHead
                },
            }
        )
        if index == 1 or index % 10 == 0 or index == len(image_paths):
            click.echo(f"Rendered {index}/{len(image_paths)} images")

    colors = {
        name: {"b": color[0], "g": color[1], "r": color[2]}
        for name, color in model_class_colors().items()
    }
    manifest = {
        "model_path": str(model_path),
        "source_image_dir": str(source_image_dir),
        "output_dir": str(output_dir),
        "config": asdict(config),
        "image_count": len(image_paths),
        "prediction_count": {head.value: totals[head] for head in ModelHead},
        "class_colors_bgr": colors,
        "saved_images": saved_images,
        "images": image_records,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2),
        encoding="utf-8",
    )
    click.echo(f"Saved {len(image_paths)} rendered images to {output_dir}")


if __name__ == "__main__":
    main()
