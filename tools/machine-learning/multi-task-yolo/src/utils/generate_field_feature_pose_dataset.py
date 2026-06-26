import math
import shutil
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, NoReturn

import click
import yaml

IMAGE_SUFFIXES = frozenset(
    {
        ".bmp",
        ".jpeg",
        ".jpg",
        ".png",
        ".tif",
        ".tiff",
        ".webp",
    }
)

FIELD_FEATURE_CLASSES = {
    1: (0, "GoalPost"),
    2: (1, "LSpot"),
    3: (2, "PenaltySpot"),
    5: (3, "TSpot"),
    6: (4, "XSpot"),
}

DEFAULT_DATA_YAML = Path("assets/datasets/multi-task-objects.yaml")
DEFAULT_OUTPUT_NAME = "multi-task-field-features-pose"


@dataclass(frozen=True)
class PoseLabel:
    class_id: int
    x: float
    y: float
    width: float
    height: float
    point_x: float
    point_y: float

    def to_row(self) -> str:
        return (
            f"{self.class_id} "
            f"{self.x:.6f} {self.y:.6f} "
            f"{self.width:.6f} {self.height:.6f} "
            f"{self.point_x:.6f} {self.point_y:.6f}"
        )


@dataclass
class SplitStats:
    input_images: int = 0
    output_images: int = 0
    labeled_images: int = 0
    empty_labels: int = 0
    kept_instances: Counter[int] = field(default_factory=Counter)
    dropped_instances: Counter[int] = field(default_factory=Counter)


def _fail(message: str) -> NoReturn:
    raise click.ClickException(message)


def _is_image_file(path: Path) -> bool:
    return path.suffix.lower() in IMAGE_SUFFIXES


def _load_dataset_yaml(data_path: Path) -> dict[str, Any]:
    raw_data = yaml.safe_load(data_path.read_text())
    if raw_data is None:
        _fail(f"Dataset YAML is empty: {data_path}")
    if not isinstance(raw_data, dict):
        _fail(f"Dataset YAML must contain a mapping: {data_path}")
    return raw_data


def _resolve_dataset_root(data_path: Path, dataset: dict[str, Any]) -> Path:
    raw_root = dataset.get("path")
    if raw_root is None:
        return data_path.parent.resolve()
    if not isinstance(raw_root, str):
        _fail("Dataset YAML `path` must be a string when provided")

    root = Path(raw_root).expanduser()
    if root.is_absolute():
        return root.resolve()
    return (data_path.parent / root).resolve()


def _resolve_split_entries(
    dataset: dict[str, Any],
    split: str,
    dataset_root: Path,
) -> list[Path]:
    raw_split = dataset.get(split)
    if raw_split is None:
        _fail(f"Dataset YAML is missing required `{split}` key")

    if isinstance(raw_split, str):
        split_entries = [raw_split]
    elif isinstance(raw_split, list) and all(
        isinstance(entry, str) for entry in raw_split
    ):
        split_entries = raw_split
    else:
        _fail(f"Dataset YAML `{split}` must be a string or list of strings")

    return [
        entry_path.resolve()
        if (entry_path := Path(entry).expanduser()).is_absolute()
        else (dataset_root / entry_path).resolve()
        for entry in split_entries
    ]


def _resolve_list_image_path(
    raw_line: str,
    list_dir: Path,
    dataset_root: Path,
) -> Path:
    image_path = Path(raw_line).expanduser()
    if image_path.is_absolute():
        return image_path.resolve()

    list_relative_path = (list_dir / image_path).resolve()
    if list_relative_path.exists():
        return list_relative_path

    root_relative_path = (dataset_root / image_path).resolve()
    if root_relative_path.exists():
        return root_relative_path

    return list_relative_path


def _read_image_list(list_path: Path, dataset_root: Path) -> list[Path]:
    image_paths: list[Path] = []
    for line_number, raw_line in enumerate(
        list_path.read_text().splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        image_path = _resolve_list_image_path(
            line,
            list_path.parent,
            dataset_root,
        )
        if not image_path.exists():
            _fail(f"Missing image at {list_path}:{line_number}: {line}")
        if not _is_image_file(image_path):
            _fail(
                f"Unsupported image suffix at {list_path}:{line_number}: {line}"
            )
        image_paths.append(image_path.resolve())

    return image_paths


def _read_images_from_entry(entry_path: Path, dataset_root: Path) -> list[Path]:
    if entry_path.is_dir():
        return sorted(
            path.resolve()
            for path in entry_path.rglob("*")
            if path.is_file() and _is_image_file(path)
        )

    if not entry_path.exists():
        _fail(f"Missing dataset path: {entry_path}")

    if _is_image_file(entry_path):
        return [entry_path.resolve()]
    if entry_path.suffix.lower() == ".txt":
        return _read_image_list(entry_path, dataset_root)

    _fail(f"Unsupported dataset path type: {entry_path}")


def _load_split_images(
    dataset: dict[str, Any],
    split: str,
    dataset_root: Path,
) -> list[Path]:
    image_paths: list[Path] = []
    for entry_path in _resolve_split_entries(dataset, split, dataset_root):
        image_paths.extend(_read_images_from_entry(entry_path, dataset_root))

    return list(dict.fromkeys(image_paths))


def _label_path_for_image(image_path: Path) -> Path:
    parts = list(image_path.parts)
    for index in range(len(parts) - 1, -1, -1):
        if parts[index] == "images":
            parts[index] = "labels"
            return Path(*parts).with_suffix(".txt")

    return image_path.with_suffix(".txt")


def _parse_class_id(token: str, label_path: Path, line_number: int) -> int:
    try:
        raw_class_id = float(token)
    except ValueError:
        _fail(f"Invalid class id at {label_path}:{line_number}")

    if not raw_class_id.is_integer():
        _fail(f"Non-integer class id at {label_path}:{line_number}")
    class_id = int(raw_class_id)
    if class_id < 0:
        _fail(f"Negative class id at {label_path}:{line_number}")
    return class_id


def _parse_coordinate(
    token: str,
    label_path: Path,
    line_number: int,
) -> float:
    try:
        value = float(token)
    except ValueError:
        _fail(f"Invalid coordinate at {label_path}:{line_number}: {token}")

    if not math.isfinite(value):
        _fail(f"Non-finite coordinate at {label_path}:{line_number}: {token}")
    return value


def _clamp01(value: float) -> float:
    return min(max(value, 0.0), 1.0)


def _convert_label_row(
    old_class_id: int,
    coords: tuple[float, float, float, float],
) -> PoseLabel | None:
    class_mapping = FIELD_FEATURE_CLASSES.get(old_class_id)
    if class_mapping is None:
        return None

    new_class_id, _ = class_mapping
    x, y, width, height = coords
    point_x = x
    point_y = y + height / 2 if old_class_id == 1 else y

    return PoseLabel(
        class_id=new_class_id,
        x=x,
        y=y,
        width=width,
        height=height,
        point_x=_clamp01(point_x),
        point_y=_clamp01(point_y),
    )


def _read_pose_labels(label_path: Path, stats: SplitStats) -> list[PoseLabel]:
    if not label_path.exists():
        return []

    pose_labels: list[PoseLabel] = []
    for line_number, raw_line in enumerate(
        label_path.read_text().splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line:
            continue

        tokens = line.split()
        if len(tokens) != 5:
            _fail(
                f"Expected 5 detection columns at {label_path}:{line_number}, "
                f"got {len(tokens)}"
            )

        old_class_id = _parse_class_id(tokens[0], label_path, line_number)
        coords = tuple(
            _parse_coordinate(token, label_path, line_number)
            for token in tokens[1:]
        )
        pose_label = _convert_label_row(old_class_id, coords)
        if pose_label is None:
            stats.dropped_instances[old_class_id] += 1
            continue

        stats.kept_instances[pose_label.class_id] += 1
        pose_labels.append(pose_label)

    return pose_labels


def _path_relative_to_root(path: Path, dataset_root: Path) -> Path:
    resolved_path = path.resolve()
    resolved_root = dataset_root.resolve()
    try:
        return resolved_path.relative_to(resolved_root)
    except ValueError:
        return Path(*resolved_path.parts[1:])


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.resolve().relative_to(parent.resolve())
    except ValueError:
        return False
    return True


def _validate_output_root(
    output_root: Path,
    dataset_root: Path,
    source_entries: list[Path],
) -> None:
    if output_root.resolve() == dataset_root.resolve():
        _fail("Output root must not be the source dataset root")

    for source_entry in source_entries:
        if _is_relative_to(source_entry, output_root) or _is_relative_to(
            output_root,
            source_entry,
        ):
            _fail(
                "Output root must not overlap with source image entries: "
                f"{output_root} overlaps {source_entry}"
            )


def _clean_output_split(output_root: Path, split: str) -> None:
    for parent in ("images", "labels"):
        split_path = output_root / parent / split
        if split_path.exists():
            shutil.rmtree(split_path)


def _flatten_path(path: Path) -> str:
    return "__".join(path.parts)


def _output_paths_for_image(
    image_path: Path,
    dataset_root: Path,
    output_root: Path,
    split: str,
) -> tuple[Path, Path]:
    relative_source = _path_relative_to_root(image_path, dataset_root)
    output_name = f"{_flatten_path(relative_source.with_suffix(''))}"
    output_image = (
        output_root
        / "images"
        / split
        / f"{output_name}{image_path.suffix.lower()}"
    )
    output_label = output_root / "labels" / split / f"{output_name}.txt"
    return output_image, output_label


def _write_image_reference(
    source_path: Path,
    output_path: Path,
    image_mode: str,
    *,
    overwrite: bool,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() or output_path.is_symlink():
        if not overwrite:
            return
        if output_path.is_dir():
            _fail(f"Refusing to overwrite directory: {output_path}")
        output_path.unlink()

    match image_mode:
        case "symlink":
            output_path.symlink_to(source_path)
        case "hardlink":
            output_path.hardlink_to(source_path)
        case "copy":
            shutil.copy2(source_path, output_path)
        case _:
            _fail(f"Unsupported image mode: {image_mode}")


def _write_pose_label(label_path: Path, pose_labels: list[PoseLabel]) -> None:
    label_path.parent.mkdir(parents=True, exist_ok=True)
    contents = "\n".join(label.to_row() for label in pose_labels)
    label_path.write_text(f"{contents}\n" if contents else "")


def _process_split(
    dataset: dict[str, Any],
    dataset_root: Path,
    output_root: Path,
    split: str,
    image_mode: str,
    *,
    include_empty: bool,
    overwrite: bool,
    clean: bool,
) -> SplitStats:
    if clean:
        _clean_output_split(output_root, split)

    image_paths = _load_split_images(dataset, split, dataset_root)
    stats = SplitStats(input_images=len(image_paths))
    output_sources: dict[Path, Path] = {}

    for image_path in image_paths:
        pose_labels = _read_pose_labels(
            _label_path_for_image(image_path), stats
        )
        if not pose_labels and not include_empty:
            continue

        output_image, output_label = _output_paths_for_image(
            image_path,
            dataset_root,
            output_root,
            split,
        )
        existing_source = output_sources.get(output_image)
        if existing_source is not None and existing_source != image_path:
            _fail(
                f"Output path collision for {output_image}: "
                f"{existing_source} and {image_path}"
            )
        output_sources[output_image] = image_path

        _write_image_reference(
            image_path,
            output_image,
            image_mode,
            overwrite=overwrite,
        )
        _write_pose_label(output_label, pose_labels)

        stats.output_images += 1
        if pose_labels:
            stats.labeled_images += 1
        else:
            stats.empty_labels += 1

    return stats


def _class_name(class_id: int) -> str:
    for new_class_id, name in FIELD_FEATURE_CLASSES.values():
        if new_class_id == class_id:
            return name
    return f"class_{class_id}"


def _print_split_summary(split: str, stats: SplitStats) -> None:
    click.echo(f"{split} split:")
    click.echo(f"  input images: {stats.input_images}")
    click.echo(f"  output images: {stats.output_images}")
    click.echo(f"  labeled images: {stats.labeled_images}")
    click.echo(f"  empty labels: {stats.empty_labels}")
    click.echo("  kept instances:")
    for class_id in sorted(
        new_class_id for new_class_id, _ in FIELD_FEATURE_CLASSES.values()
    ):
        click.echo(
            f"    {class_id} {_class_name(class_id)}: "
            f"{stats.kept_instances[class_id]}"
        )
    if stats.dropped_instances:
        click.echo("  dropped source instances:")
        for old_class_id in sorted(stats.dropped_instances):
            click.echo(
                f"    source class {old_class_id}: "
                f"{stats.dropped_instances[old_class_id]}"
            )


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help=(
        "Generate a field-feature-only YOLO pose dataset from the "
        "multi-task object detection dataset."
    ),
)
@click.argument(
    "data-yaml",
    required=False,
    default=DEFAULT_DATA_YAML,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--output-root",
    type=click.Path(file_okay=False, path_type=Path),
    default=None,
    help=(
        "Output dataset root. Defaults to "
        f"<source dataset root>/{DEFAULT_OUTPUT_NAME}."
    ),
)
@click.option(
    "--image-mode",
    type=click.Choice(["symlink", "hardlink", "copy"]),
    default="symlink",
    show_default=True,
    help="How output images reference source images.",
)
@click.option(
    "--include-empty/--exclude-empty",
    default=True,
    show_default=True,
    help=(
        "Keep images with no field-feature labels as background samples. "
        "This intentionally writes empty label files."
    ),
)
@click.option(
    "--overwrite",
    is_flag=True,
    default=False,
    show_default=True,
    help="Replace existing output image links/files.",
)
@click.option(
    "--clean",
    is_flag=True,
    default=False,
    show_default=True,
    help="Remove generated images/<split> and labels/<split> before writing.",
)
def main(
    data_yaml: Path,
    output_root: Path | None,
    image_mode: str,
    *,
    include_empty: bool,
    overwrite: bool,
    clean: bool,
) -> None:
    dataset = _load_dataset_yaml(data_yaml)
    dataset_root = _resolve_dataset_root(data_yaml, dataset)
    if output_root is None:
        output_root = dataset_root / DEFAULT_OUTPUT_NAME
    else:
        output_root = output_root.expanduser().resolve()

    source_entries = [
        *_resolve_split_entries(dataset, "train", dataset_root),
        *_resolve_split_entries(dataset, "val", dataset_root),
    ]
    _validate_output_root(output_root, dataset_root, source_entries)

    click.echo(f"Source dataset YAML: {data_yaml}")
    click.echo(f"Source dataset root: {dataset_root}")
    click.echo(f"Output dataset root: {output_root}")
    click.echo()

    for split in ("train", "val"):
        stats = _process_split(
            dataset,
            dataset_root,
            output_root,
            split,
            image_mode,
            include_empty=include_empty,
            overwrite=overwrite,
            clean=clean,
        )
        _print_split_summary(split, stats)
        click.echo()

    click.echo("Generated field-feature pose dataset.")


if __name__ == "__main__":
    main()
