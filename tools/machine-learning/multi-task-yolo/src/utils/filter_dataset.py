import logging
import shutil
from dataclasses import dataclass
from pathlib import Path

import click
import yaml

logger = logging.getLogger(__name__)

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


class DatasetConfigError(Exception):
    def __init__(self, message: str) -> None:
        super().__init__(message)

    @classmethod
    def yaml_must_be_mapping(cls) -> "DatasetConfigError":
        return cls("Dataset YAML must contain a mapping")

    @classmethod
    def names_must_be_list_or_dict(cls) -> "DatasetConfigError":
        return cls("Dataset YAML must define 'names' as a list or dictionary")

    @classmethod
    def at_least_one_class_required(cls) -> "DatasetConfigError":
        return cls("At least one class id must be provided")

    @classmethod
    def class_ids_must_be_non_negative(
        cls,
        negative_ids: list[int],
    ) -> "DatasetConfigError":
        return cls(
            f"Class ids must be non-negative, got: {sorted(negative_ids)}"
        )

    @classmethod
    def class_ids_not_in_names(
        cls,
        missing_ids: list[int],
    ) -> "DatasetConfigError":
        return cls(
            f"Requested class ids do not exist in dataset names: {missing_ids}"
        )

    @classmethod
    def split_values_must_be_strings(cls) -> "DatasetConfigError":
        return cls("Split values must be strings or lists of strings")

    @classmethod
    def at_least_one_split_required(cls) -> "DatasetConfigError":
        return cls("Dataset YAML must define at least one split")


@dataclass(frozen=True)
class SplitInput:
    split_name: str
    image_dir: Path
    split_value: str


@dataclass
class FilterStats:
    scanned_images: int = 0
    kept_images: int = 0
    dropped_images: int = 0
    missing_labels: int = 0
    malformed_lines: int = 0
    total_kept_labels: int = 0


def _setup_logging(*, verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )


def _load_yaml(yaml_path: Path) -> dict:
    with open(yaml_path, encoding="utf-8") as file:
        content = yaml.safe_load(file)

    if not isinstance(content, dict):
        raise DatasetConfigError.yaml_must_be_mapping()

    return content


def _resolve_dataset_root(yaml_path: Path, config: dict) -> Path:
    raw_path = config.get("path")
    if raw_path is None:
        return yaml_path.parent.resolve()

    path = Path(raw_path)
    if path.is_absolute():
        return path

    return (yaml_path.parent / path).resolve()


def _extract_names(config: dict) -> dict[int, str]:
    names = config.get("names")
    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}

    if isinstance(names, dict):
        result: dict[int, str] = {}
        for key, value in names.items():
            try:
                class_id = int(key)
            except (TypeError, ValueError) as error:
                msg = f"Invalid class id key in 'names': {key!r}"
                raise DatasetConfigError(msg) from error
            result[class_id] = str(value)
        return result

    raise DatasetConfigError.names_must_be_list_or_dict()


def _validate_class_ids(
    selected_class_ids: tuple[int, ...],
    names: dict[int, str],
) -> set[int]:
    class_ids = set(selected_class_ids)
    if not class_ids:
        raise DatasetConfigError.at_least_one_class_required()

    negative_ids = [class_id for class_id in class_ids if class_id < 0]
    if negative_ids:
        raise DatasetConfigError.class_ids_must_be_non_negative(negative_ids)

    missing_ids = sorted(
        class_id for class_id in class_ids if class_id not in names
    )
    if missing_ids:
        raise DatasetConfigError.class_ids_not_in_names(missing_ids)

    return class_ids


def _split_items(value: object) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, list):
        if not all(isinstance(item, str) for item in value):
            raise DatasetConfigError.split_values_must_be_strings()
        return [str(item) for item in value]

    raise DatasetConfigError.split_values_must_be_strings()


def _resolve_split_inputs(dataset_root: Path, config: dict) -> list[SplitInput]:
    split_inputs: list[SplitInput] = []
    for split_name in ("train", "val", "test"):
        raw_value = config.get(split_name)
        if raw_value is None:
            continue

        for split_value in _split_items(raw_value):
            path = Path(split_value)
            image_dir = path if path.is_absolute() else dataset_root / path
            split_inputs.append(
                SplitInput(
                    split_name=split_name,
                    image_dir=image_dir.resolve(),
                    split_value=split_value,
                )
            )

    if not split_inputs:
        raise DatasetConfigError.at_least_one_split_required()

    return split_inputs


def _to_label_path(image_path: Path, image_dir: Path) -> Path:
    relative_path = image_path.relative_to(image_dir)
    parts = list(image_dir.parts)
    try:
        images_idx = parts.index("images")
        parts[images_idx] = "labels"
    except ValueError:
        parts[-1] = parts[-1].replace("images", "labels", 1)

    label_dir = Path(*parts)
    return (label_dir / relative_path).with_suffix(".txt")


def _read_filtered_label_lines(
    label_path: Path,
    allowed_class_ids: set[int],
) -> tuple[list[str], int]:
    kept_lines: list[str] = []
    malformed_lines = 0

    with open(label_path, encoding="utf-8") as file:
        for line_number, raw_line in enumerate(file, start=1):
            stripped = raw_line.strip()
            if not stripped:
                continue

            parts = stripped.split(maxsplit=1)
            try:
                class_id = int(parts[0])
            except ValueError:
                malformed_lines += 1
                logger.warning(
                    "Skipping malformed label line in %s:%d",
                    label_path,
                    line_number,
                )
                continue

            if class_id in allowed_class_ids:
                kept_lines.append(stripped)

    return kept_lines, malformed_lines


def _copy_filtered_sample(
    image_path: Path,
    source_image_dir: Path,
    source_label_path: Path,
    dest_image_dir: Path,
    dest_label_dir: Path,
    class_ids: set[int],
    *,
    dry_run: bool,
    stats: FilterStats,
) -> None:
    stats.scanned_images += 1

    if not source_label_path.exists():
        stats.dropped_images += 1
        stats.missing_labels += 1
        logger.debug("Skipping %s without label file", image_path)
        return

    kept_lines, malformed_lines = _read_filtered_label_lines(
        source_label_path,
        class_ids,
    )
    stats.malformed_lines += malformed_lines

    if not kept_lines:
        stats.dropped_images += 1
        return

    relative_path = image_path.relative_to(source_image_dir)
    destination_image = dest_image_dir / relative_path
    destination_label = (dest_label_dir / relative_path).with_suffix(".txt")

    if not dry_run:
        destination_image.parent.mkdir(parents=True, exist_ok=True)
        destination_label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(image_path, destination_image)
        with open(destination_label, "w", encoding="utf-8") as file:
            file.write("\n".join(kept_lines) + "\n")

    stats.kept_images += 1
    stats.total_kept_labels += len(kept_lines)


def _iter_images(image_dir: Path) -> list[Path]:
    images: list[Path] = []
    for path in image_dir.rglob("*"):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
            images.append(path)
    return sorted(images)


def _write_output_yaml(
    output_yaml_path: Path,
    output_root: Path,
    split_inputs: list[SplitInput],
    selected_names: dict[int, str],
    *,
    dry_run: bool,
) -> None:
    split_values: dict[str, list[str]] = {"train": [], "val": [], "test": []}
    for split in split_inputs:
        split_values[split.split_name].append(split.split_value)

    output_config: dict[str, object] = {
        "path": str(output_root.resolve()),
        "names": {
            class_id: selected_names[class_id] for class_id in selected_names
        },
        "nc": max(selected_names) + 1,
    }

    for split_name, values in split_values.items():
        if not values:
            continue
        output_config[split_name] = values[0] if len(values) == 1 else values

    if dry_run:
        logger.info("Dry-run: would write dataset YAML to %s", output_yaml_path)
        return

    output_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_yaml_path, "w", encoding="utf-8") as file:
        yaml.safe_dump(output_config, file, sort_keys=False)


def _build_destination_dirs(
    output_root: Path, split_value: str
) -> tuple[Path, Path]:
    split_path = Path(split_value)
    destination_image_dir = output_root / split_path
    labels_path_text = split_value.replace("images", "labels", 1)
    destination_label_dir = output_root / Path(labels_path_text)
    return destination_image_dir, destination_label_dir


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Extract a class-filtered YOLO dataset subset.",
)
@click.argument("dataset_yaml", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--class-id",
    "class_ids",
    type=int,
    multiple=True,
    required=True,
    help="Class ID to keep. Repeat to include multiple classes.",
)
@click.option(
    "--output-name",
    required=True,
    type=str,
    help="Name of the output dataset directory under assets/datasets/.",
)
@click.option(
    "--assets-dir",
    type=Path,
    default=Path("assets"),
    show_default=True,
    help="Assets directory that contains the datasets folder.",
)
@click.option(
    "--output-yaml-name",
    type=str,
    default="dataset.yaml",
    show_default=True,
    help="Filename for the generated dataset YAML.",
)
@click.option(
    "--overwrite",
    is_flag=True,
    default=False,
    show_default=True,
    help="Overwrite output directory if it already exists.",
)
@click.option(
    "--dry-run",
    is_flag=True,
    default=False,
    show_default=True,
    help="Preview actions without writing files.",
)
@click.option(
    "--verbose",
    is_flag=True,
    default=False,
    show_default=True,
    help="Enable debug logs.",
)
def main(
    dataset_yaml: Path,
    class_ids: tuple[int, ...],
    output_name: str,
    assets_dir: Path,
    output_yaml_name: str,
    *,
    overwrite: bool,
    dry_run: bool,
    verbose: bool,
) -> None:
    _setup_logging(verbose=verbose)

    source_config = _load_yaml(dataset_yaml)
    dataset_root = _resolve_dataset_root(dataset_yaml, source_config)
    names = _extract_names(source_config)
    selected_class_ids = _validate_class_ids(class_ids, names)
    split_inputs = _resolve_split_inputs(dataset_root, source_config)

    output_root = (assets_dir / "datasets" / output_name).resolve()
    output_yaml_path = output_root / output_yaml_name

    if output_root.exists() and not overwrite and not dry_run:
        msg = (
            f"Output directory already exists: {output_root}. "
            "Use --overwrite to replace it."
        )
        raise click.ClickException(msg)

    if output_root.exists() and overwrite and not dry_run:
        shutil.rmtree(output_root)

    stats = FilterStats()
    selected_names = {
        class_id: names[class_id] for class_id in sorted(selected_class_ids)
    }

    logger.info("Source dataset root: %s", dataset_root)
    logger.info("Output dataset root: %s", output_root)
    logger.info("Keeping class ids: %s", sorted(selected_class_ids))

    for split in split_inputs:
        if not split.image_dir.exists():
            logger.warning(
                "Skipping split %s because image directory does not exist: %s",
                split.split_name,
                split.image_dir,
            )
            continue

        destination_image_dir, destination_label_dir = _build_destination_dirs(
            output_root,
            split.split_value,
        )
        images = _iter_images(split.image_dir)
        logger.info(
            "Processing split %s (%d images)", split.split_name, len(images)
        )

        for image_path in images:
            label_path = _to_label_path(image_path, split.image_dir)
            _copy_filtered_sample(
                image_path,
                split.image_dir,
                label_path,
                destination_image_dir,
                destination_label_dir,
                selected_class_ids,
                dry_run=dry_run,
                stats=stats,
            )

    _write_output_yaml(
        output_yaml_path,
        output_root,
        split_inputs,
        selected_names,
        dry_run=dry_run,
    )

    logger.info("Finished dataset filtering")
    logger.info("Scanned images: %d", stats.scanned_images)
    logger.info("Kept images: %d", stats.kept_images)
    logger.info("Dropped images: %d", stats.dropped_images)
    logger.info("Missing labels: %d", stats.missing_labels)
    logger.info("Malformed label lines: %d", stats.malformed_lines)
    logger.info("Kept labels: %d", stats.total_kept_labels)


if __name__ == "__main__":
    main()
