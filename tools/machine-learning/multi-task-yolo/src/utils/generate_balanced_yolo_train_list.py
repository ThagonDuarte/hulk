import math
import random
from collections import Counter
from dataclasses import dataclass
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


@dataclass(frozen=True)
class ImageSample:
    image_path: Path
    class_ids: tuple[int, ...]


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


def _resolve_train_entry(raw_entry: str, dataset_root: Path) -> Path:
    entry = Path(raw_entry).expanduser()
    if entry.is_absolute():
        return entry.resolve()
    return (dataset_root / entry).resolve()


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


def _read_train_entry(entry_path: Path, dataset_root: Path) -> list[Path]:
    if entry_path.is_dir():
        return sorted(
            path.resolve()
            for path in entry_path.rglob("*")
            if path.is_file() and _is_image_file(path)
        )

    if not entry_path.exists():
        _fail(f"Missing train path: {entry_path}")

    if _is_image_file(entry_path):
        return [entry_path.resolve()]
    if entry_path.suffix.lower() == ".txt":
        return _read_image_list(entry_path, dataset_root)

    _fail(f"Unsupported train path type: {entry_path}")


def _load_train_images(
    dataset: dict[str, Any], dataset_root: Path
) -> list[Path]:
    raw_train = dataset.get("train")
    if raw_train is None:
        _fail("Dataset YAML is missing required `train` key")

    if isinstance(raw_train, str):
        train_entries = [raw_train]
    elif isinstance(raw_train, list) and all(
        isinstance(entry, str) for entry in raw_train
    ):
        train_entries = raw_train
    else:
        _fail("Dataset YAML `train` must be a string or list of strings")

    image_paths: list[Path] = []
    for raw_entry in train_entries:
        image_paths.extend(
            _read_train_entry(
                _resolve_train_entry(raw_entry, dataset_root),
                dataset_root,
            )
        )

    unique_image_paths = list(dict.fromkeys(image_paths))
    if not unique_image_paths:
        _fail("No training images found")
    return unique_image_paths


def _label_path_for_image(image_path: Path) -> Path:
    parts = list(image_path.parts)
    for index in range(len(parts) - 1, -1, -1):
        if parts[index] == "images":
            parts[index] = "labels"
            return Path(*parts).with_suffix(".txt")

    return image_path.with_suffix(".txt")


def _parse_label_file(label_path: Path) -> tuple[int, ...]:
    if not label_path.exists():
        return ()

    class_ids: list[int] = []
    for line_number, raw_line in enumerate(
        label_path.read_text().splitlines(),
        start=1,
    ):
        line = raw_line.strip()
        if not line:
            continue

        first_token = line.split(maxsplit=1)[0]
        try:
            class_id = int(first_token)
        except ValueError:
            _fail(f"Invalid class id at {label_path}:{line_number}")
        if class_id < 0:
            _fail(f"Negative class id at {label_path}:{line_number}")
        class_ids.append(class_id)

    return tuple(class_ids)


def _load_samples(image_paths: list[Path]) -> list[ImageSample]:
    return [
        ImageSample(
            image_path=image_path,
            class_ids=_parse_label_file(_label_path_for_image(image_path)),
        )
        for image_path in image_paths
    ]


def _parse_names(raw_names: Any) -> dict[int, str]:
    if isinstance(raw_names, list):
        return {index: str(name) for index, name in enumerate(raw_names)}
    if isinstance(raw_names, dict):
        names: dict[int, str] = {}
        for raw_class_id, name in raw_names.items():
            try:
                class_id = int(raw_class_id)
            except (TypeError, ValueError):
                _fail(f"Invalid class id in dataset names: {raw_class_id}")
            names[class_id] = str(name)
        return names
    if raw_names is None:
        return {}

    _fail("Dataset YAML `names` must be a list or mapping when provided")


def _class_ids_for_report(
    dataset: dict[str, Any],
    names: dict[int, str],
    image_counts: Counter[int],
    instance_counts: Counter[int],
) -> list[int]:
    class_ids = set(names) | set(image_counts) | set(instance_counts)
    raw_nc = dataset.get("nc")
    if isinstance(raw_nc, int):
        class_ids.update(range(raw_nc))
    return sorted(class_ids)


def _class_name(class_id: int, names: dict[int, str]) -> str:
    return names.get(class_id, f"class_{class_id}")


def _repeat_from_count(
    image_count: int,
    max_image_count: int,
    max_repeat: int,
) -> int:
    if image_count <= 0 or max_image_count <= 0:
        return 1
    raw_repeat = int(math.sqrt(max_image_count / image_count) + 0.5)
    return min(max(raw_repeat, 1), max_repeat)


def _sample_repeat(
    sample: ImageSample,
    image_counts: Counter[int],
    max_image_count: int,
    max_repeat: int,
    *,
    include_empty: bool,
) -> int:
    image_class_ids = set(sample.class_ids)
    if not image_class_ids:
        return 1 if include_empty else 0

    rarest_image_count = min(
        image_counts[class_id] for class_id in image_class_ids
    )
    return _repeat_from_count(rarest_image_count, max_image_count, max_repeat)


def _collect_class_counts(
    samples: list[ImageSample],
) -> tuple[Counter[int], Counter[int]]:
    image_counts: Counter[int] = Counter()
    instance_counts: Counter[int] = Counter()

    for sample in samples:
        instance_counts.update(sample.class_ids)
        image_counts.update(set(sample.class_ids))

    return image_counts, instance_counts


def _build_balanced_train_list(
    samples: list[ImageSample],
    image_counts: Counter[int],
    max_repeat: int,
    common_only_keep_ratio: float,
    seed: int,
    *,
    include_empty: bool,
) -> tuple[list[Path], Counter[int], int]:
    rng = random.Random(seed)  # noqa: S311 - deterministic sampling only.
    max_image_count = max(image_counts.values(), default=0)
    expanded_paths: list[Path] = []
    repeat_counts: Counter[int] = Counter()
    dropped_common_only = 0

    for sample in samples:
        repeat = _sample_repeat(
            sample,
            image_counts,
            max_image_count,
            max_repeat,
            include_empty=include_empty,
        )
        if (
            repeat == 1
            and sample.class_ids
            and rng.random() > common_only_keep_ratio
        ):
            dropped_common_only += 1
            repeat_counts[0] += 1
            continue

        repeat_counts[repeat] += 1
        expanded_paths.extend([sample.image_path] * repeat)

    if not expanded_paths:
        _fail("Balanced train list would be empty")

    rng.shuffle(expanded_paths)
    return expanded_paths, repeat_counts, dropped_common_only


def _path_relative_to_root(path: Path, dataset_root: Path) -> str:
    resolved_path = path.resolve()
    resolved_root = dataset_root.resolve()
    try:
        return resolved_path.relative_to(resolved_root).as_posix()
    except ValueError:
        return resolved_path.as_posix()


def _write_train_list(output_path: Path, image_paths: list[Path]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    contents = "\n".join(path.resolve().as_posix() for path in image_paths)
    output_path.write_text(f"{contents}\n")


def _write_balanced_dataset_yaml(
    dataset: dict[str, Any],
    dataset_root: Path,
    output_train_list: Path,
    output_data_yaml: Path,
) -> None:
    balanced_dataset = dict(dataset)
    balanced_dataset["path"] = dataset_root.resolve().as_posix()
    balanced_dataset["train"] = _path_relative_to_root(
        output_train_list,
        dataset_root,
    )

    output_data_yaml.parent.mkdir(parents=True, exist_ok=True)
    output_data_yaml.write_text(
        yaml.safe_dump(balanced_dataset, sort_keys=False)
    )


def _print_class_summary(
    dataset: dict[str, Any],
    names: dict[int, str],
    image_counts: Counter[int],
    instance_counts: Counter[int],
    max_repeat: int,
) -> None:
    class_ids = _class_ids_for_report(
        dataset,
        names,
        image_counts,
        instance_counts,
    )
    if not class_ids:
        click.echo("No labeled classes were found in the training set.")
        return

    max_image_count = max(image_counts.values(), default=0)
    click.echo("Class imbalance summary:")
    click.echo("id  name                          images  instances  repeat")
    for class_id in class_ids:
        image_count = image_counts[class_id]
        instance_count = instance_counts[class_id]
        repeat = "n/a"
        if image_count > 0:
            repeat = str(
                _repeat_from_count(
                    image_count,
                    max_image_count,
                    max_repeat,
                )
            )
        name = _class_name(class_id, names)[:28]
        click.echo(
            f"{class_id:>2}  {name:<28}  "
            f"{image_count:>6}  {instance_count:>9}  {repeat:>6}"
        )


@click.command(
    context_settings={"help_option_names": ["-h", "--help"]},
    help=(
        "Generate a repeat-factor balanced YOLO train list without copying "
        "or deleting image files."
    ),
)
@click.argument(
    "data-yaml",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--output-train-list",
    type=click.Path(dir_okay=False, path_type=Path),
    default=None,
    help=(
        "Path for the generated train list. Defaults to "
        "<data-yaml-stem>-balanced-train.txt next to the dataset YAML."
    ),
)
@click.option(
    "--output-data-yaml",
    type=click.Path(dir_okay=False, path_type=Path),
    default=None,
    help=(
        "Optional path for a copied dataset YAML that points train to the list."
    ),
)
@click.option(
    "--max-repeat",
    type=click.IntRange(min=1),
    default=4,
    show_default=True,
    help="Maximum number of times any single image can appear in the list.",
)
@click.option(
    "--common-only-keep-ratio",
    type=click.FloatRange(min=0.0, max=1.0),
    default=1.0,
    show_default=True,
    help=(
        "Fraction of repeat-1 labeled images to keep. Leave at 1.0 to avoid "
        "undersampling."
    ),
)
@click.option(
    "--seed",
    type=int,
    default=0,
    show_default=True,
    help="Random seed used for deterministic shuffling and optional dropping.",
)
@click.option(
    "--include-empty/--exclude-empty",
    default=True,
    show_default=True,
    help="Whether to keep training images with no label file or no objects.",
)
def main(
    data_yaml: Path,
    output_train_list: Path | None,
    output_data_yaml: Path | None,
    max_repeat: int,
    common_only_keep_ratio: float,
    seed: int,
    *,
    include_empty: bool,
) -> None:
    dataset = _load_dataset_yaml(data_yaml)
    dataset_root = _resolve_dataset_root(data_yaml, dataset)
    image_paths = _load_train_images(dataset, dataset_root)
    samples = _load_samples(image_paths)
    image_counts, instance_counts = _collect_class_counts(samples)
    names = _parse_names(dataset.get("names"))

    balanced_paths, repeat_counts, dropped_common_only = (
        _build_balanced_train_list(
            samples,
            image_counts,
            max_repeat,
            common_only_keep_ratio,
            seed,
            include_empty=include_empty,
        )
    )

    if output_train_list is None:
        output_train_list = data_yaml.with_name(
            f"{data_yaml.stem}-balanced-train.txt"
        )

    _write_train_list(output_train_list, balanced_paths)
    if output_data_yaml is not None:
        _write_balanced_dataset_yaml(
            dataset,
            dataset_root,
            output_train_list,
            output_data_yaml,
        )

    _print_class_summary(
        dataset,
        names,
        image_counts,
        instance_counts,
        max_repeat,
    )
    click.echo()
    click.echo(f"Input train images: {len(samples)}")
    click.echo(f"Balanced train rows: {len(balanced_paths)}")
    click.echo(f"Dropped repeat-1 labeled images: {dropped_common_only}")
    click.echo("Image repeat distribution:")
    for repeat in sorted(repeat_counts):
        click.echo(f"  repeat {repeat}: {repeat_counts[repeat]} images")
    click.echo(f"Wrote train list: {output_train_list}")
    if output_data_yaml is not None:
        click.echo(f"Wrote dataset YAML: {output_data_yaml}")


if __name__ == "__main__":
    main()
