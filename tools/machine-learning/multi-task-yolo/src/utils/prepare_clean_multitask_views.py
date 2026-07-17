"""Build exact-content-clean validation views without touching source data."""

# ruff: noqa: TRY003

import hashlib
import os
import shutil
import tempfile
from collections import Counter, defaultdict
from collections.abc import Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import click
import yaml

from ultralytics_dfine.data.dataset import (
    DatasetDefinition,
    _images_from_source,
    _label_path,
    load_dataset_yaml,
)
from ultralytics_dfine.data.dhrp import load_dhrp_records
from validation.multitask_data import sha256_file, write_json

DEFAULT_CANONICAL_OBJECT_DATA = Path(
    "/home/alexschmander/datasets/multi-task-objects-hslvision-balanced.yaml"
)
DEFAULT_REFRESH_OBJECT_DATA = Path(
    "runs/data/hslvision-annotato-selected-v1/object_refresh/data.yaml"
)
DEFAULT_PERSON_DATA = Path("/home/alexschmander/datasets/coco-pose.yaml")
DEFAULT_DHRP_ROOT = Path("/home/alexschmander/datasets/DHRP")
DEFAULT_FIELD_DATA = Path(
    "/home/alexschmander/datasets/multi-task-field-features-pose.yaml"
)
DEFAULT_DESTINATION = Path("runs/data/multitask-clean-v2")
MANIFEST_VERSION = 2


def _absolute_preserving_links(path: str | Path) -> Path:
    """Make a path absolute without resolving a dataset-view symlink."""
    return Path(os.path.abspath(Path(path).expanduser()))


def _dataset_images(
    definition: DatasetDefinition,
    split: str,
) -> list[Path]:
    sources = getattr(definition, split)
    return [
        _absolute_preserving_links(image)
        for source in sources
        for image in _images_from_source(source)
    ]


def _dhrp_split(
    root: Path,
    split: str,
) -> tuple[list[Path], list[Path]]:
    annotations = sorted(
        path.resolve() for path in (root / "annot").glob(f"{split}_set_*.json")
    )
    if not annotations:
        raise FileNotFoundError(f"DHRP has no {split} annotations")
    records = load_dhrp_records(root, annotations)
    return (
        [_absolute_preserving_links(record.image_path) for record in records],
        annotations,
    )


def _hash_one(path: Path) -> tuple[Path, str]:
    return path, sha256_file(path)


def _hash_paths(
    paths: Sequence[Path],
    *,
    workers: int,
) -> dict[Path, str]:
    unique = sorted(set(paths), key=str)
    with ThreadPoolExecutor(max_workers=workers) as executor:
        return dict(executor.map(_hash_one, unique))


def _content_counter(
    paths: Sequence[Path],
    digests: Mapping[Path, str],
) -> Counter[str]:
    return Counter(digests[path] for path in paths)


def _multiset_sha256(counter: Mapping[str, int]) -> str:
    digest = hashlib.sha256()
    for image_digest, count in sorted(counter.items()):
        digest.update(f"{image_digest} {count}\n".encode())
    return digest.hexdigest()


def _source_name(path: Path) -> str:
    encoded_source, separator, _ = path.name.partition("__images__")
    if separator:
        return encoded_source
    parts = path.parts
    if "datasets" in parts:
        index = parts.index("datasets")
        if index + 1 < len(parts):
            dataset = parts[index + 1]
            if dataset == "DHRP" and index + 3 < len(parts):
                return "/".join(parts[index + 1 : index + 4])
            return dataset
    return path.parent.name


def _set_summary(
    paths: Sequence[Path],
    digests: Mapping[Path, str],
) -> dict[str, object]:
    counter = _content_counter(paths, digests)
    return {
        "occurrences": len(paths),
        "unique_paths": len(set(paths)),
        "unique_hashes": len(counter),
        "duplicate_occurrences": len(paths) - len(counter),
        "content_multiset_sha256": _multiset_sha256(counter),
        "sources_by_occurrence": dict(
            sorted(Counter(_source_name(path) for path in paths).items())
        ),
    }


def _filter_and_deduplicate(
    paths: Sequence[Path],
    digests: Mapping[Path, str],
    excluded_hashes: set[str],
) -> tuple[list[Path], list[Path]]:
    clean = [path for path in paths if digests[path] not in excluded_hashes]
    seen: set[str] = set()
    unique = []
    for path in clean:
        image_digest = digests[path]
        if image_digest in seen:
            continue
        seen.add(image_digest)
        unique.append(path)
    return clean, unique


def _label_sha256(path: Path) -> str:
    label_path = _label_path(path)
    return sha256_file(label_path) if label_path.is_file() else "missing"


def _duplicate_groups(
    paths: Sequence[Path],
    digests: Mapping[Path, str],
) -> list[dict[str, object]]:
    grouped: dict[str, list[Path]] = defaultdict(list)
    for path in paths:
        grouped[digests[path]].append(path)
    result = []
    for image_digest, members in sorted(grouped.items()):
        if len(members) < 2:
            continue
        labels = [_label_sha256(path) for path in members]
        result.append(
            {
                "sha256": image_digest,
                "occurrences": len(members),
                "paths": [str(path) for path in members],
                "label_sha256": labels,
                "label_conflict": len(set(labels)) > 1,
                "selected_path": str(members[0]),
            }
        )
    return result


def _excluded_samples(
    validation_paths: Sequence[Path],
    training_sets: Mapping[str, Sequence[Path]],
    digests: Mapping[Path, str],
    excluded_hashes: set[str],
) -> list[dict[str, object]]:
    validation_by_hash: dict[str, list[Path]] = defaultdict(list)
    training_by_hash: dict[str, dict[str, list[Path]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for path in validation_paths:
        validation_by_hash[digests[path]].append(path)
    for task, paths in training_sets.items():
        for path in paths:
            training_by_hash[digests[path]][task].append(path)

    result = []
    for image_digest in sorted(excluded_hashes):
        validation_members = validation_by_hash[image_digest]
        memberships = {}
        for task, members in sorted(training_by_hash[image_digest].items()):
            unique_members = list(dict.fromkeys(members))
            memberships[task] = {
                "occurrences": len(members),
                "paths": [str(path) for path in unique_members],
                "sources": dict(
                    sorted(
                        Counter(
                            _source_name(path) for path in unique_members
                        ).items()
                    )
                ),
            }
        result.append(
            {
                "sha256": image_digest,
                "validation_occurrences": len(validation_members),
                "validation_paths": [str(path) for path in validation_members],
                "validation_sources": dict(
                    sorted(
                        Counter(
                            _source_name(path) for path in validation_members
                        ).items()
                    )
                ),
                "training_memberships": memberships,
            }
        )
    return result


def _overlap_summary(
    train_paths: Sequence[Path],
    validation_paths: Sequence[Path],
    digests: Mapping[Path, str],
) -> dict[str, int]:
    train = _content_counter(train_paths, digests)
    validation = _content_counter(validation_paths, digests)
    shared = set(train) & set(validation)
    return {
        "unique_hashes": len(shared),
        "train_occurrences": sum(train[value] for value in shared),
        "validation_occurrences": sum(validation[value] for value in shared),
    }


def _write_lines(path: Path, values: Sequence[Path]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "".join(f"{value}\n" for value in values),
        encoding="utf-8",
    )


def _source_value(sources: Sequence[Path]) -> str | list[str]:
    values = [str(_absolute_preserving_links(path)) for path in sources]
    return values[0] if len(values) == 1 else values


def _write_yaml(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(value, sort_keys=False),
        encoding="utf-8",
    )


def _artifact_report(build_root: Path) -> dict[str, dict[str, object]]:
    result = {}
    for path in sorted(build_root.iterdir(), key=lambda value: value.name):
        if not path.is_file() or path.name == "manifest.json":
            continue
        result[path.name] = {
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
            **(
                {"lines": len(path.read_text(encoding="utf-8").splitlines())}
                if path.suffix == ".txt"
                else {}
            ),
        }
    return result


def prepare_clean_views(
    destination: str | Path,
    canonical_object_data: str | Path,
    refresh_object_data: str | Path,
    person_data: str | Path,
    field_data: str | Path,
    dhrp_root: str | Path | None,
    *,
    workers: int = 32,
    robot_train_images: Sequence[Path] | None = None,
    robot_validation_images: Sequence[Path] | None = None,
) -> dict[str, object]:
    """Materialize clean wrappers and return their deterministic manifest."""
    if workers <= 0:
        raise ValueError("Hash workers must be positive")
    output = _absolute_preserving_links(destination)
    if output.exists():
        raise FileExistsError(
            f"Clean-view destination already exists: {output}"
        )

    canonical_path = _absolute_preserving_links(canonical_object_data)
    refresh_path = _absolute_preserving_links(refresh_object_data)
    person_path = _absolute_preserving_links(person_data)
    field_path = _absolute_preserving_links(field_data)
    canonical = load_dataset_yaml(canonical_path)
    refresh = load_dataset_yaml(refresh_path)
    person = load_dataset_yaml(person_path)
    field = load_dataset_yaml(field_path)
    if canonical.names != refresh.names:
        raise ValueError("Canonical and refreshed object classes differ")

    annotations: dict[str, list[Path]] = {"train": [], "eval": []}
    if robot_train_images is None or robot_validation_images is None:
        if dhrp_root is None:
            raise ValueError(
                "DHRP root is required without robot path overrides"
            )
        robot_root = _absolute_preserving_links(dhrp_root)
        loaded_train, annotations["train"] = _dhrp_split(
            robot_root,
            "train",
        )
        loaded_validation, annotations["eval"] = _dhrp_split(
            robot_root,
            "eval",
        )
        robot_train_images = loaded_train
        robot_validation_images = loaded_validation
    else:
        robot_root = (
            _absolute_preserving_links(dhrp_root)
            if dhrp_root is not None
            else None
        )

    sets: dict[str, list[Path]] = {
        "train/object_canonical": _dataset_images(canonical, "train"),
        "train/object_refresh": _dataset_images(refresh, "train"),
        "train/person": _dataset_images(person, "train"),
        "train/robot": [
            _absolute_preserving_links(path) for path in robot_train_images
        ],
        "train/field": _dataset_images(field, "train"),
        "val/object": _dataset_images(canonical, "val"),
        "val/person": _dataset_images(person, "val"),
        "val/robot": [
            _absolute_preserving_links(path) for path in robot_validation_images
        ],
        "val/field": _dataset_images(field, "val"),
    }
    all_paths = [path for paths in sets.values() for path in paths]
    digests = _hash_paths(all_paths, workers=workers)

    training_memberships = {
        key.removeprefix("train/"): value
        for key, value in sets.items()
        if key.startswith("train/")
    }
    task_training = {
        "object": sets["train/object_canonical"],
        "person": sets["train/person"],
        "robot": sets["train/robot"],
        "field": sets["train/field"],
    }
    task_validation = {
        "object": sets["val/object"],
        "person": sets["val/person"],
        "robot": sets["val/robot"],
        "field": sets["val/field"],
    }
    training_union = {
        digests[path]
        for key, paths in sets.items()
        if key.startswith("train/")
        for path in paths
    }
    object_hashes = {digests[path] for path in sets["val/object"]}
    field_hashes = {digests[path] for path in sets["val/field"]}
    object_excluded = object_hashes & training_union
    field_excluded = field_hashes & training_union
    object_clean, object_unique = _filter_and_deduplicate(
        sets["val/object"],
        digests,
        object_excluded,
    )
    field_clean, field_unique = _filter_and_deduplicate(
        sets["val/field"],
        digests,
        field_excluded,
    )
    if {digests[path] for path in object_clean} & training_union:
        raise RuntimeError("Object clean view still overlaps training")
    if {digests[path] for path in field_clean} & training_union:
        raise RuntimeError("Field clean view still overlaps training")

    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f".{output.name}-",
        dir=output.parent,
    ) as temporary:
        build_root = Path(temporary) / output.name
        build_root.mkdir()
        list_names = {
            "object-val-clean.txt": object_clean,
            "object-val-clean-unique.txt": object_unique,
            "field-val-clean.txt": field_clean,
            "field-val-clean-unique.txt": field_unique,
        }
        for name, values in list_names.items():
            _write_lines(build_root / name, values)

        canonical_yaml = {
            "path": str(canonical.root),
            "train": _source_value(canonical.train),
            "val": str(output / "object-val-clean.txt"),
            "names": canonical.names,
        }
        refresh_yaml = {
            "path": str(refresh.root),
            "train": _source_value(refresh.train),
            "val": str(output / "object-val-clean.txt"),
            "names": refresh.names,
        }
        field_yaml = {
            "path": str(field.root),
            "train": _source_value(field.train),
            "val": str(output / "field-val-clean.txt"),
            "kpt_shape": [1, 2],
            "names": field.names,
        }
        canonical_unique_yaml = {
            **canonical_yaml,
            "val": str(output / "object-val-clean-unique.txt"),
        }
        refresh_unique_yaml = {
            **refresh_yaml,
            "val": str(output / "object-val-clean-unique.txt"),
        }
        field_unique_yaml = {
            **field_yaml,
            "val": str(output / "field-val-clean-unique.txt"),
        }
        _write_yaml(build_root / "canonical-clean.yaml", canonical_yaml)
        _write_yaml(
            build_root / "canonical-clean-unique.yaml",
            canonical_unique_yaml,
        )
        _write_yaml(build_root / "refresh-clean.yaml", refresh_yaml)
        _write_yaml(
            build_root / "refresh-clean-unique.yaml",
            refresh_unique_yaml,
        )
        _write_yaml(build_root / "field-clean.yaml", field_yaml)
        _write_yaml(
            build_root / "field-clean-unique.yaml",
            field_unique_yaml,
        )

        canonical_counter = _content_counter(
            sets["train/object_canonical"],
            digests,
        )
        refresh_counter = _content_counter(
            sets["train/object_refresh"],
            digests,
        )
        pair_matrix = {
            f"train/{train_task}->val/{validation_task}": (
                _overlap_summary(train_paths, validation_paths, digests)
            )
            for train_task, train_paths in task_training.items()
            for validation_task, validation_paths in task_validation.items()
        }
        manifest: dict[str, object] = {
            "version": MANIFEST_VERSION,
            "algorithm": "sha256-exact-image-content",
            "selection": {
                "clean": "preserve loader order and multiplicity; remove "
                "every validation hash found in any task training view",
                "clean_unique": "from clean, retain the first loader-order "
                "path for each image hash",
                "paths": "absolute paths with symlinks intentionally not "
                "resolved so task-view labels remain addressable",
            },
            "inputs": {
                "canonical_object_data": {
                    "path": str(canonical_path),
                    "sha256": sha256_file(canonical_path),
                },
                "refresh_object_data": {
                    "path": str(refresh_path),
                    "sha256": sha256_file(refresh_path),
                },
                "person_data": {
                    "path": str(person_path),
                    "sha256": sha256_file(person_path),
                },
                "field_data": {
                    "path": str(field_path),
                    "sha256": sha256_file(field_path),
                },
                "dhrp_root": str(robot_root) if robot_root else None,
                "dhrp_annotations": {
                    split: [
                        {"path": str(path), "sha256": sha256_file(path)}
                        for path in paths
                    ]
                    for split, paths in annotations.items()
                },
            },
            "source_sets": {
                key: _set_summary(paths, digests) for key, paths in sets.items()
            },
            "object_refresh_image_invariance": {
                "same_occurrences": len(sets["train/object_canonical"])
                == len(sets["train/object_refresh"]),
                "same_content_multiset": canonical_counter == refresh_counter,
                "canonical_content_multiset_sha256": (
                    _multiset_sha256(canonical_counter)
                ),
                "refresh_content_multiset_sha256": (
                    _multiset_sha256(refresh_counter)
                ),
            },
            "training_union": {
                "unique_hashes": len(training_union),
                "memberships": sorted(training_memberships),
            },
            "train_to_validation": pair_matrix,
            "views": {
                "object": {
                    "original": _set_summary(
                        sets["val/object"],
                        digests,
                    ),
                    "clean": _set_summary(object_clean, digests),
                    "clean_unique": _set_summary(object_unique, digests),
                    "excluded_unique_hashes": len(object_excluded),
                    "excluded_occurrences": len(sets["val/object"])
                    - len(object_clean),
                    "excluded_sources": dict(
                        sorted(
                            Counter(
                                _source_name(path)
                                for path in sets["val/object"]
                                if digests[path] in object_excluded
                            ).items()
                        )
                    ),
                    "excluded_samples": _excluded_samples(
                        sets["val/object"],
                        training_memberships,
                        digests,
                        object_excluded,
                    ),
                    "clean_duplicate_groups": _duplicate_groups(
                        object_clean,
                        digests,
                    ),
                },
                "field": {
                    "original": _set_summary(
                        sets["val/field"],
                        digests,
                    ),
                    "clean": _set_summary(field_clean, digests),
                    "clean_unique": _set_summary(field_unique, digests),
                    "excluded_unique_hashes": len(field_excluded),
                    "excluded_occurrences": len(sets["val/field"])
                    - len(field_clean),
                    "excluded_sources": dict(
                        sorted(
                            Counter(
                                _source_name(path)
                                for path in sets["val/field"]
                                if digests[path] in field_excluded
                            ).items()
                        )
                    ),
                    "excluded_samples": _excluded_samples(
                        sets["val/field"],
                        training_memberships,
                        digests,
                        field_excluded,
                    ),
                    "clean_duplicate_groups": _duplicate_groups(
                        field_clean,
                        digests,
                    ),
                },
            },
            "compatibility": {
                "source_datasets_mutated": False,
                "selected_annotato_field_train": (
                    "unchanged; authoritative zero-field negatives remain "
                    "eligible for training"
                ),
                "selected_annotato_legacy_field_val": (
                    "the existing seven-image, zero-row view is not "
                    "rewritten; current generator policy excludes empty "
                    "field validation samples"
                ),
            },
        }
        manifest["artifacts"] = _artifact_report(build_root)
        write_json(build_root / "manifest.json", manifest)
        shutil.move(str(build_root), output)
    return manifest


@click.command()
@click.argument(
    "destination",
    type=click.Path(path_type=Path),
    default=DEFAULT_DESTINATION,
)
@click.option(
    "--canonical-object-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_CANONICAL_OBJECT_DATA,
    show_default=True,
)
@click.option(
    "--refresh-object-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_REFRESH_OBJECT_DATA,
    show_default=True,
)
@click.option(
    "--person-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_PERSON_DATA,
    show_default=True,
)
@click.option(
    "--dhrp-root",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    default=DEFAULT_DHRP_ROOT,
    show_default=True,
)
@click.option(
    "--field-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_FIELD_DATA,
    show_default=True,
)
@click.option("--workers", type=click.IntRange(min=1), default=32)
def main(
    destination: Path,
    canonical_object_data: Path,
    refresh_object_data: Path,
    person_data: Path,
    dhrp_root: Path,
    field_data: Path,
    workers: int,
) -> None:
    """Create immutable clean validation lists and wrapper YAML files."""
    manifest = prepare_clean_views(
        destination,
        canonical_object_data,
        refresh_object_data,
        person_data,
        field_data,
        dhrp_root,
        workers=workers,
    )
    views = manifest["views"]
    if not isinstance(views, dict):
        raise TypeError("Generated manifest has no views")
    for task in ("object", "field"):
        view = views.get(task)
        if not isinstance(view, dict):
            raise TypeError(f"Generated manifest has no {task} view")
        clean = view.get("clean")
        unique = view.get("clean_unique")
        if not isinstance(clean, dict) or not isinstance(unique, dict):
            raise TypeError(f"Generated {task} view has invalid counts")
        click.echo(
            f"{task}: clean={clean.get('occurrences')}, "
            f"unique={unique.get('occurrences')}, "
            f"excluded={view.get('excluded_occurrences')}"
        )
    click.echo(f"Prepared clean views: {Path(destination).absolute()}")


if __name__ == "__main__":
    main()
