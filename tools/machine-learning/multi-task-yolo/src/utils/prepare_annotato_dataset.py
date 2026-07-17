"""Prepare the fixed seven-chunk Annotato label-refresh experiment data."""

# ruff: noqa: C901, S603, TRY003

import json
import os
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterable
from dataclasses import asdict, dataclass
from pathlib import Path

import click
import yaml

from ultralytics_dfine.data.dataset import (
    IMAGE_SUFFIXES,
    _images_from_source,
    load_dataset_yaml,
)
from validation.multitask_data import sha256_file, write_json

CHUNKS = (
    "abaft-pith",
    "abashed-sound",
    "abounding-law",
    "absorbing-flow",
    "absorbing-whirlwind",
    "abusive-core",
    "accessible-lacquerware",
)
VALIDATION_CHUNK = "absorbing-whirlwind"
CLASS_NAMES = (
    "Ball",
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "Robot",
    "TSpot",
    "XSpot",
)
FIELD_CLASS_NAMES = (
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "TSpot",
    "XSpot",
)
FIELD_CLASSES = frozenset(FIELD_CLASS_NAMES)
EXPECTED_IMAGES = 700
EXPECTED_COMPLETE_IMAGES = 294
EXPECTED_OBJECT_ROWS = 6901
EXPECTED_FIELD_ROWS = 1709
EXPECTED_OBJECT_TRAIN = 600
EXPECTED_OBJECT_VAL = 100
EXPECTED_FIELD_TRAIN = 287
# The seven legacy validation images contain no field rows.  Keep them out of
# newly generated field validation views while retaining authoritative
# zero-field negatives in the already-used training view.
EXPECTED_FIELD_VAL = 0
DEFAULT_INPUT = Path("/home/alexschmander/datasets/hslvision_annotato")
DEFAULT_CONVERTER = Path(
    "/home/alexschmander/hulk-annotate/tools/annotato/scripts/"
    "convert_dataset_to_yolo.py"
)
DEFAULT_OBJECT_DATA = Path(
    "/home/alexschmander/datasets/multi-task-objects-hslvision-balanced.yaml"
)
DEFAULT_FIELD_DATA = Path(
    "/home/alexschmander/datasets/multi-task-field-features-pose.yaml"
)


@dataclass(frozen=True)
class AnnotatoSample:
    chunk: str
    split: str
    stem: str
    image_path: Path
    annotation_path: Path
    field_complete: bool
    field_instances: int
    image_sha256: str
    annotation_sha256: str


def _annotation_payload(
    path: Path,
) -> tuple[list[str], list[dict[str, object]]]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise TypeError(f"Annotato label must be an object: {path}")
    labeled_classes = value.get("labeled_classes")
    annotations = value.get("annotations")
    if not isinstance(labeled_classes, list) or not all(
        isinstance(name, str) for name in labeled_classes
    ):
        raise TypeError(f"Invalid labeled_classes: {path}")
    if not isinstance(annotations, list) or not all(
        isinstance(annotation, dict) for annotation in annotations
    ):
        raise TypeError(f"Invalid annotations: {path}")
    return labeled_classes, annotations


def _field_complete(annotations: list[dict[str, object]]) -> bool:
    """Return whether every authoritative field instance has a point."""
    return all(
        annotation.get("point") is not None
        for annotation in annotations
        if annotation.get("class") in FIELD_CLASSES
    )


def _field_instance_count(annotations: list[dict[str, object]]) -> int:
    """Count field instances that the converter can emit as pose rows."""
    return sum(
        annotation.get("class") in FIELD_CLASSES for annotation in annotations
    )


def _field_validation_eligible(
    annotations: list[dict[str, object]],
) -> bool:
    """Require complete points and at least one field row for validation."""
    return (
        _field_complete(annotations) and _field_instance_count(annotations) > 0
    )


def collect_samples(input_root: str | Path) -> list[AnnotatoSample]:
    """Validate and identify exactly the accepted seven chunks."""
    root = Path(input_root).resolve()
    samples = []
    seen_stems: set[str] = set()
    seen_hashes: set[str] = set()
    for chunk in CHUNKS:
        folder = root / chunk
        if not folder.is_dir():
            raise FileNotFoundError(f"Annotato chunk not found: {folder}")
        annotation_paths = sorted(folder.glob("*.json"))
        image_paths = sorted(
            path
            for path in folder.iterdir()
            if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
        )
        if len(annotation_paths) != 100 or len(image_paths) != 100:
            raise ValueError(
                f"Chunk '{chunk}' must contain 100 labels and 100 images"
            )
        images_by_stem = {path.stem: path for path in image_paths}
        if len(images_by_stem) != len(image_paths):
            raise ValueError(f"Chunk '{chunk}' has duplicate image stems")
        for annotation_path in annotation_paths:
            image_path = images_by_stem.get(annotation_path.stem)
            if image_path is None:
                raise ValueError(f"No image for annotation: {annotation_path}")
            if annotation_path.stem in seen_stems:
                raise ValueError(
                    f"Duplicate sample stem: {annotation_path.stem}"
                )
            seen_stems.add(annotation_path.stem)
            labeled_classes, annotations = _annotation_payload(annotation_path)
            if tuple(labeled_classes) != CLASS_NAMES:
                raise ValueError(
                    f"Label authority differs in {annotation_path}"
                )
            image_digest = sha256_file(image_path)
            if image_digest in seen_hashes:
                raise ValueError(f"Duplicate image content: {image_path}")
            seen_hashes.add(image_digest)
            samples.append(
                AnnotatoSample(
                    chunk=chunk,
                    split=("val" if chunk == VALIDATION_CHUNK else "train"),
                    stem=annotation_path.stem,
                    image_path=image_path.resolve(),
                    annotation_path=annotation_path.resolve(),
                    field_complete=_field_complete(annotations),
                    field_instances=_field_instance_count(annotations),
                    image_sha256=image_digest,
                    annotation_sha256=sha256_file(annotation_path),
                )
            )
    if len(samples) != EXPECTED_IMAGES:
        raise ValueError(f"Expected {EXPECTED_IMAGES} Annotato samples")
    complete_count = sum(sample.field_complete for sample in samples)
    if complete_count != EXPECTED_COMPLETE_IMAGES:
        raise ValueError(
            f"Expected {EXPECTED_COMPLETE_IMAGES} field-complete samples, "
            f"found {complete_count}"
        )
    return samples


def _run_converter(
    converter: Path,
    staged_input: Path,
    output: Path,
    task: str,
) -> None:
    # The converter is a PEP 723 script, but its only dependency (Click) is
    # already part of this project's locked environment. Running it with the
    # current interpreter keeps preparation fully offline and still executes
    # the authoritative external converter implementation.
    subprocess.run(
        [
            sys.executable,
            str(converter),
            str(staged_input),
            str(output),
            "--task",
            task,
            "--train-split",
            "1.0",
            "--seed",
            "20260716",
        ],
        check=True,
    )


def _write_yaml(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as file:
        yaml.safe_dump(value, file, sort_keys=False)


def _rows(path: Path) -> list[str]:
    if not path.is_file():
        raise FileNotFoundError(f"Converted label not found: {path}")
    return [
        line for line in path.read_text(encoding="utf-8").splitlines() if line
    ]


def _link_or_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def _materialize_views(
    samples: list[AnnotatoSample],
    mixed_output: Path,
    pose_output: Path,
    build_root: Path,
    final_root: Path,
) -> dict[str, int]:
    object_rows = 0
    field_rows = 0
    object_counts = {"train": 0, "val": 0}
    field_counts = {"train": 0, "val": 0}
    for split in ("train", "val"):
        (build_root / "field" / "images" / split).mkdir(
            parents=True,
            exist_ok=True,
        )
        (build_root / "field" / "labels" / split).mkdir(
            parents=True,
            exist_ok=True,
        )
    for sample in samples:
        source_image = (
            mixed_output / "images" / "train" / sample.image_path.name
        )
        mixed_label = mixed_output / "labels" / "train" / f"{sample.stem}.txt"
        object_image = (
            build_root
            / "object"
            / "images"
            / sample.split
            / sample.image_path.name
        )
        object_label = (
            build_root
            / "object"
            / "labels"
            / sample.split
            / f"{sample.stem}.txt"
        )
        _link_or_copy(source_image, object_image)
        converted_rows = _rows(mixed_label)
        box_rows = [" ".join(row.split()[:5]) for row in converted_rows]
        if any(len(row.split()) != 5 for row in box_rows):
            raise ValueError(f"Invalid derived box label: {mixed_label}")
        object_label.parent.mkdir(parents=True, exist_ok=True)
        object_label.write_text("\n".join(box_rows), encoding="utf-8")
        object_rows += len(box_rows)
        object_counts[sample.split] += 1

        if not sample.field_complete:
            continue
        pose_label = pose_output / "labels" / "train" / f"{sample.stem}.txt"
        pose_rows = _rows(pose_label)
        if bool(pose_rows) != (sample.field_instances > 0):
            raise ValueError(
                "Converted field-row count differs for "
                f"{sample.annotation_path}"
            )
        if sample.split == "val" and not pose_rows:
            continue
        field_image = (
            build_root
            / "field"
            / "images"
            / sample.split
            / sample.image_path.name
        )
        field_label = (
            build_root
            / "field"
            / "labels"
            / sample.split
            / f"{sample.stem}.txt"
        )
        _link_or_copy(object_image, field_image)
        field_lines = [" ".join(row.split()[:7]) for row in pose_rows]
        if any(len(row.split()) != 7 for row in field_lines):
            raise ValueError(f"Invalid derived field label: {pose_label}")
        field_label.parent.mkdir(parents=True, exist_ok=True)
        field_label.write_text("\n".join(field_lines), encoding="utf-8")
        field_rows += len(field_lines)
        field_counts[sample.split] += 1

    observed = {
        "images": len(samples),
        "field_complete_images": sum(
            sample.field_complete for sample in samples
        ),
        "object_rows": object_rows,
        "field_rows": field_rows,
        "object_train_images": object_counts["train"],
        "object_val_images": object_counts["val"],
        "field_train_images": field_counts["train"],
        "field_val_images": field_counts["val"],
    }
    expected = {
        "images": EXPECTED_IMAGES,
        "field_complete_images": EXPECTED_COMPLETE_IMAGES,
        "object_rows": EXPECTED_OBJECT_ROWS,
        "field_rows": EXPECTED_FIELD_ROWS,
        "object_train_images": EXPECTED_OBJECT_TRAIN,
        "object_val_images": EXPECTED_OBJECT_VAL,
        "field_train_images": EXPECTED_FIELD_TRAIN,
        "field_val_images": EXPECTED_FIELD_VAL,
    }
    if observed != expected:
        raise ValueError(f"Converted Annotato counts differ: {observed}")

    _write_yaml(
        build_root / "object" / "data.yaml",
        {
            "path": str((final_root / "object").resolve()),
            "train": "images/train",
            "val": "images/val",
            "names": list(CLASS_NAMES),
        },
    )
    _write_yaml(
        build_root / "field" / "data.yaml",
        {
            "path": str((final_root / "field").resolve()),
            "train": "images/train",
            "val": "images/val",
            "kpt_shape": [1, 2],
            "names": list(FIELD_CLASS_NAMES),
        },
    )
    return observed


def _all_images(sources: Iterable[Path]) -> list[Path]:
    return [
        Path(os.path.abspath(image))
        for source in sources
        for image in _images_from_source(source)
    ]


def _hash_paths(paths: Iterable[Path]) -> dict[Path, str]:
    return {path: sha256_file(path) for path in set(paths)}


def _write_manifest(path: Path, images: Iterable[Path]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(f"{Path(os.path.abspath(image))}\n" for image in images)
    )


def _replacement_object_dataset(
    samples: list[AnnotatoSample],
    build_root: Path,
    final_root: Path,
    base_data: Path,
    *,
    expected_replacements: int,
) -> dict[str, int]:
    definition = load_dataset_yaml(base_data)
    base_train = _all_images(definition.train)
    digests = _hash_paths(base_train)
    prepared = {
        sample.image_sha256: (
            final_root / "object" / "images" / "train" / sample.image_path.name
        )
        for sample in samples
        if sample.split == "train"
    }
    validation_hashes = {
        sample.image_sha256 for sample in samples if sample.split == "val"
    }
    if any(digests[path] in validation_hashes for path in base_train):
        raise ValueError("Annotato validation images occur in object training")
    replacements = sum(digests[path] in prepared for path in base_train)
    if replacements != expected_replacements:
        raise ValueError(
            f"Expected {expected_replacements} object replacements, "
            f"found {replacements}"
        )
    refreshed = [prepared.get(digests[path], path) for path in base_train]
    manifest = build_root / "object_refresh" / "train.txt"
    _write_manifest(manifest, refreshed)
    _write_yaml(
        build_root / "object_refresh" / "data.yaml",
        {
            "path": str((final_root / "object_refresh").resolve()),
            "train": str(
                (final_root / "object_refresh" / "train.txt").resolve()
            ),
            "val": [str(path.resolve()) for path in definition.val],
            "names": definition.names,
        },
    )
    return {
        "base_train_occurrences": len(base_train),
        "replaced_occurrences": replacements,
        "refreshed_train_occurrences": len(refreshed),
    }


def _field_mix_dataset(
    samples: list[AnnotatoSample],
    build_root: Path,
    final_root: Path,
    base_data: Path,
    *,
    repetitions: int,
    expected_base_train: int,
    expected_base_val: int,
) -> dict[str, int | float]:
    definition = load_dataset_yaml(base_data)
    base_train = _all_images(definition.train)
    base_val = _all_images(definition.val)
    if (
        len(base_train) != expected_base_train
        or len(base_val) != expected_base_val
    ):
        raise ValueError(
            "Base field dataset counts differ: "
            f"train={len(base_train)}, val={len(base_val)}"
        )
    digests = _hash_paths([*base_train, *base_val])
    selected = {
        sample.image_sha256: (
            final_root / "field" / "images" / "train" / sample.image_path.name
        )
        for sample in samples
        if sample.split == "train" and sample.field_complete
    }
    validation_hashes = {
        sample.image_sha256
        for sample in samples
        if sample.split == "val"
        and sample.field_complete
        and sample.field_instances > 0
    }
    if any(digests[path] in validation_hashes for path in base_train):
        raise ValueError("Annotato validation images occur in field training")
    retained = [path for path in base_train if digests[path] not in selected]
    prepared = [path for path in selected.values() for _ in range(repetitions)]
    mixed = [*retained, *prepared]
    folder = build_root / f"field_mix_{repetitions}x"
    _write_manifest(folder / "train.txt", mixed)
    _write_yaml(
        folder / "data.yaml",
        {
            "path": str((final_root / f"field_mix_{repetitions}x").resolve()),
            "train": str(
                (
                    final_root / f"field_mix_{repetitions}x" / "train.txt"
                ).resolve()
            ),
            "val": [str(path.resolve()) for path in definition.val],
            "kpt_shape": [1, 2],
            "names": definition.names,
        },
    )
    removed = len(base_train) - len(retained)
    return {
        "base_train_images": len(base_train),
        "base_val_images": len(base_val),
        "deduplicated_base_images": removed,
        "selected_images": len(selected),
        "repetitions": repetitions,
        "mixed_train_occurrences": len(mixed),
        "selected_exposure": len(prepared) / len(mixed),
    }


def prepare_dataset(
    input_root: Path,
    destination: Path,
    converter: Path,
    base_object_data: Path,
    base_field_data: Path,
    *,
    expected_object_replacements: int = 1016,
    expected_base_field_train: int = 14468,
    expected_base_field_val: int = 3632,
    converter_runner: Callable[[Path, Path, Path, str], None] = _run_converter,
) -> dict[str, object]:
    """Materialize views and training manifests after all invariants pass."""
    input_root = input_root.resolve()
    destination = destination.resolve()
    converter = converter.resolve()
    if destination.exists():
        raise FileExistsError(f"Destination already exists: {destination}")
    if not converter.is_file():
        raise FileNotFoundError(f"Converter not found: {converter}")
    samples = collect_samples(input_root)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{destination.name}-",
        dir=destination.parent,
    ) as temporary:
        temporary_root = Path(temporary)
        staged = temporary_root / "selected-chunks"
        staged.mkdir()
        for chunk in CHUNKS:
            (staged / chunk).symlink_to(
                input_root / chunk, target_is_directory=True
            )
        mixed = temporary_root / "converted-mixed"
        pose = temporary_root / "converted-pose"
        converter_runner(converter, staged, mixed, "mixed")
        converter_runner(converter, staged, pose, "pose")
        build_root = temporary_root / "payload"
        build_root.mkdir()
        conversion = _materialize_views(
            samples,
            mixed,
            pose,
            build_root,
            destination,
        )
        object_refresh = _replacement_object_dataset(
            samples,
            build_root,
            destination,
            base_object_data,
            expected_replacements=expected_object_replacements,
        )
        field_mixes = {
            f"{repetitions}x": _field_mix_dataset(
                samples,
                build_root,
                destination,
                base_field_data,
                repetitions=repetitions,
                expected_base_train=expected_base_field_train,
                expected_base_val=expected_base_field_val,
            )
            for repetitions in (6, 13)
        }
        report: dict[str, object] = {
            "version": 1,
            "chunks": list(CHUNKS),
            "validation_chunk": VALIDATION_CHUNK,
            "input_root": str(input_root),
            "converter": str(converter),
            "conversion": conversion,
            "object_refresh": object_refresh,
            "field_mixes": field_mixes,
            "compatibility": {
                "field_training_view": (
                    "unchanged: complete zero-field negatives remain eligible"
                ),
                "field_validation_policy": (
                    "field-complete and at least one converted field row"
                ),
                "legacy_field_validation": (
                    "the existing seven-image zero-row artifact is not "
                    "rewritten in place"
                ),
            },
            "samples": [
                {
                    **asdict(sample),
                    "image_path": str(sample.image_path),
                    "annotation_path": str(sample.annotation_path),
                }
                for sample in samples
            ],
        }
        write_json(build_root / "manifest.json", report)
        shutil.move(str(build_root), destination)
    return report


@click.command()
@click.argument("destination", type=click.Path(path_type=Path), required=True)
@click.option(
    "--input-root",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    default=DEFAULT_INPUT,
    show_default=True,
)
@click.option(
    "--converter",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_CONVERTER,
    show_default=True,
)
@click.option(
    "--base-object-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_OBJECT_DATA,
    show_default=True,
)
@click.option(
    "--base-field-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_FIELD_DATA,
    show_default=True,
)
def main(
    input_root: Path,
    destination: Path,
    converter: Path,
    base_object_data: Path,
    base_field_data: Path,
) -> None:
    """Prepare immutable Annotato refresh and field-mix datasets."""
    report = prepare_dataset(
        input_root,
        destination,
        converter,
        base_object_data,
        base_field_data,
    )
    click.echo(json.dumps(report["conversion"], indent=2))
    click.echo(f"Prepared dataset: {destination.resolve()}")


if __name__ == "__main__":
    main()
