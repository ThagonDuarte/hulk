import logging
from pathlib import Path

import click
import cv2
import numpy as np
import yaml
from ultralytics.data.utils import visualize_image_annotations

logger = logging.getLogger(__name__)


class DatasetConfigError(Exception):
    def __init__(self, message: str) -> None:
        super().__init__(message)

    @classmethod
    def yaml_must_be_mapping(cls) -> "DatasetConfigError":
        return cls("Dataset YAML must contain a mapping")

    @classmethod
    def invalid_split(cls, split: str) -> "DatasetConfigError":
        return cls(f"Split '{split}' not found in dataset YAML")


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


def _iter_images_for_split(
    dataset_root: Path,
    split_path: str,
    max_samples: int | None,
) -> list[tuple[Path, Path]]:
    """Iterate image/label pairs in split directory."""
    split_value = Path(split_path)
    image_dir = (
        split_value if split_value.is_absolute() else dataset_root / split_value
    )
    image_dir = image_dir.resolve()

    if not image_dir.exists():
        return []

    image_extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    pairs: list[tuple[Path, Path]] = []

    for image_path in sorted(image_dir.rglob("*")):
        if not image_path.is_file():
            continue
        if image_path.suffix.lower() not in image_extensions:
            continue

        # Resolve label directory by replacing "images" in path parts
        parts = list(image_dir.parts)
        try:
            images_idx = parts.index("images")
            parts[images_idx] = "labels"
        except ValueError:
            parts[-1] = parts[-1].replace("images", "labels", 1)

        label_dir = Path(*parts)
        label_path = (
            label_dir / image_path.relative_to(image_dir)
        ).with_suffix(".txt")

        pairs.append((image_path, label_path))

        if max_samples and len(pairs) >= max_samples:
            break

    return pairs


def _visualize_sample(
    image_path: Path,
    label_path: Path,
    class_names: dict[int, str],
) -> np.ndarray | None:
    """Load image and visualize annotations."""
    if not image_path.exists():
        logger.warning("Image not found: %s", image_path)
        return None

    if not label_path.exists():
        logger.debug("No label file for: %s", image_path)
        return cv2.imread(str(image_path))

    # Validate label file content
    try:
        with open(label_path, encoding="utf-8") as f:
            lines = f.readlines()

        if not lines:
            logger.debug("Label file is empty: %s", label_path)
            return cv2.imread(str(image_path))

        valid_lines = 0
        for line in lines:
            parts = line.strip().split()
            if parts and len(parts) >= 5:
                try:
                    int(parts[0])
                    [float(x) for x in parts[1:5]]
                    valid_lines += 1
                except ValueError:
                    logger.debug(
                        "Malformed annotation line in %s: %s", label_path, line
                    )

        if valid_lines == 0:
            logger.warning("No valid annotations in: %s", label_path)
            return cv2.imread(str(image_path))

        logger.debug(
            "Found %d valid annotations in %s", valid_lines, label_path
        )

    except (OSError, ValueError) as e:
        logger.warning("Error reading label file %s: %s", label_path, e)
        return cv2.imread(str(image_path))

    return visualize_image_annotations(
        str(image_path),
        str(label_path),
        class_names,
    )


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help="Visualize a YOLO dataset with image annotations.",
)
@click.argument("dataset_yaml", type=click.Path(exists=True, path_type=Path))
@click.option(
    "--split",
    type=str,
    default="val",
    show_default=True,
    help="Dataset split to visualize (train/val/test).",
)
@click.option(
    "--max-samples",
    type=int,
    default=10,
    show_default=True,
    help="Maximum number of images to visualize.",
)
@click.option(
    "--output-dir",
    type=Path,
    default=Path("runs/visualize"),
    show_default=True,
    help="Directory to save visualized images.",
)
@click.option(
    "--save",
    is_flag=True,
    default=False,
    show_default=True,
    help="Save visualized images to disk.",
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
    split: str,
    max_samples: int,
    output_dir: Path,
    *,
    save: bool,
    verbose: bool,
) -> None:
    _setup_logging(verbose=verbose)

    config = _load_yaml(dataset_yaml)
    dataset_root = Path(config.get("path", dataset_yaml.parent)).resolve()
    class_names = config.get("names", {})

    if isinstance(class_names, list):
        class_names = dict(enumerate(class_names))

    split_path = config.get(split)
    if split_path is None:
        raise DatasetConfigError.invalid_split(split)

    split_values = [split_path] if isinstance(split_path, str) else split_path

    total_visualized = 0

    for split_value in split_values:
        pairs = _iter_images_for_split(dataset_root, split_value, max_samples)
        logger.info("Found %d samples in split %s", len(pairs), split)

        for image_path, label_path in pairs:
            visualized = _visualize_sample(image_path, label_path, class_names)
            if visualized is None:
                continue

            total_visualized += 1

            if save:
                output_dir.mkdir(parents=True, exist_ok=True)
                output_path = output_dir / image_path.name.replace(
                    image_path.suffix, "_annotated.jpg"
                )
                cv2.imwrite(str(output_path), visualized)
                logger.debug("Saved: %s", output_path)

    if save:
        logger.info(
            "Saved %d visualized images to: %s", total_visualized, output_dir
        )
    else:
        logger.info(
            "Visualized %d samples (use --save to write files)",
            total_visualized,
        )


if __name__ == "__main__":
    main()
