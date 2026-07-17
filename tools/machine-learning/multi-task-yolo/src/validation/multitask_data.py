"""Deterministic dataset identities and stratified validation subsets."""

# ruff: noqa: C901, TRY003

import hashlib
import json
from collections import defaultdict
from collections.abc import Mapping, Sequence
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import cast

from torch.utils.data import Dataset, Subset

from ultralytics_dfine.data import (
    DFINEDataset,
    DHRPDataset,
    YOLOKeypointDataset,
)
from ultralytics_dfine.data.dataset import _label_path
from ultralytics_dfine.schemas import HeadId

FINGERPRINT_VERSION = 1
TARGET_FINGERPRINT_VERSION = 1
SUBSET_MANIFEST_VERSION = 1


def sha256_file(path: str | Path) -> str:
    """Return a streaming SHA-256 digest for one file."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as file:
        while block := file.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _base_sample(dataset: Dataset, index: int) -> tuple[Dataset, int]:
    while isinstance(dataset, Subset):
        index = int(dataset.indices[index])
        dataset = dataset.dataset
    return dataset, index


def _sample_path(dataset: Dataset, index: int) -> Path:
    dataset, index = _base_sample(dataset, index)
    if isinstance(dataset, (DFINEDataset, YOLOKeypointDataset)):
        return dataset.images[index].resolve()
    if isinstance(dataset, DHRPDataset):
        return dataset.records[index].image_path.resolve()
    raise TypeError(f"Unsupported validation dataset: {type(dataset).__name__}")


def _sample_labels(dataset: Dataset, index: int) -> tuple[int, ...]:
    dataset, index = _base_sample(dataset, index)
    if isinstance(dataset, DFINEDataset):
        rows, _ = dataset._parse_label_file(_label_path(dataset.images[index]))
        return tuple(sorted({class_id for class_id, _ in rows}))
    if isinstance(dataset, YOLOKeypointDataset):
        labels, _, _ = dataset._load_labels(dataset.images[index])
        return tuple(sorted({int(value) for value in labels.tolist()}))
    if isinstance(dataset, DHRPDataset):
        record = dataset.records[index]
        return () if record.box is None else (record.class_id,)
    raise TypeError(f"Unsupported validation dataset: {type(dataset).__name__}")


def _target_digest(dataset: Dataset, index: int) -> str:
    dataset, index = _base_sample(dataset, index)
    if isinstance(dataset, DFINEDataset):
        label_path = _label_path(dataset.images[index])
        return sha256_file(label_path) if label_path.is_file() else "missing"
    if isinstance(dataset, YOLOKeypointDataset):
        label_path = _label_path(dataset.images[index])
        label_digest = (
            sha256_file(label_path) if label_path.is_file() else "missing"
        )
        payload = {
            "label_sha256": label_digest,
            "robot_negative_reviewed": (
                dataset._robot_negative_reviewed[index]
            ),
            "robot_negative_verified": (
                dataset._robot_negative_verified[index]
            ),
            "robot_negative_eligible": (
                dataset._robot_negative_eligible[index]
            ),
            "robot_negative_excluded": (
                dataset._robot_negative_excluded[index]
            ),
        }
        return hashlib.sha256(
            json.dumps(payload, sort_keys=True).encode()
        ).hexdigest()
    if isinstance(dataset, DHRPDataset):
        record = dataset.records[index]
        value = asdict(record) if is_dataclass(record) else repr(record)
        payload = json.dumps(value, sort_keys=True, default=str).encode()
        return hashlib.sha256(payload).hexdigest()
    raise TypeError(f"Unsupported validation dataset: {type(dataset).__name__}")


def _dhrp_person_negative_metadata(dataset: Dataset) -> dict[str, object]:
    indices: Sequence[int] | None = None
    if isinstance(dataset, Subset):
        base = dataset
        resolved = []
        resolved_dataset: Dataset | None = None
        for index in range(len(base)):
            nested_dataset, nested_index = _base_sample(base, index)
            if not isinstance(nested_dataset, DHRPDataset):
                return {}
            resolved_dataset = nested_dataset
            resolved.append(nested_index)
        if resolved_dataset is None:
            return {}
        dataset = resolved_dataset
        indices = resolved
    if not isinstance(dataset, DHRPDataset):
        return {}
    records = (
        dataset.records
        if indices is None
        else tuple(dataset.records[index] for index in indices)
    )
    return {
        "person_negative_manifest_sha256": (
            dataset.person_negative_manifest_sha256
        ),
        "person_negative_verified_records": (
            sum(record.person_negative_verified for record in records)
        ),
        "person_negative_eligible_records": (
            sum(record.person_negative_eligible for record in records)
        ),
    }


def _keypoint_robot_negative_metadata(dataset: Dataset) -> dict[str, object]:
    indices: Sequence[int] | None = None
    if isinstance(dataset, Subset):
        base = dataset
        resolved = []
        resolved_dataset: Dataset | None = None
        for index in range(len(base)):
            nested_dataset, nested_index = _base_sample(base, index)
            if not isinstance(nested_dataset, YOLOKeypointDataset):
                return {}
            resolved_dataset = nested_dataset
            resolved.append(nested_index)
        if resolved_dataset is None:
            return {}
        dataset = resolved_dataset
        indices = resolved
    if not isinstance(dataset, YOLOKeypointDataset):
        return {}
    selected = range(len(dataset)) if indices is None else indices
    return {
        "robot_negative_manifest_sha256": (
            dataset.robot_negative_manifest_sha256
        ),
        "robot_negative_reviewed_records": sum(
            dataset._robot_negative_reviewed[index] for index in selected
        ),
        "robot_negative_verified_records": sum(
            dataset._robot_negative_verified[index] for index in selected
        ),
        "robot_negative_eligible_records": sum(
            dataset._robot_negative_eligible[index] for index in selected
        ),
        "robot_negative_excluded_records": sum(
            dataset._robot_negative_excluded[index] for index in selected
        ),
        "robot_negative_unreviewed_records": sum(
            not dataset._robot_negative_reviewed[index] for index in selected
        ),
    }


def _negative_metadata(dataset: Dataset) -> dict[str, object]:
    return {
        **_dhrp_person_negative_metadata(dataset),
        **_keypoint_robot_negative_metadata(dataset),
    }


def dataset_fingerprint(dataset: Dataset) -> dict[str, object]:
    """Deeply identify ordered image bytes and their effective targets."""
    digest = hashlib.sha256()
    digest.update(f"multitask-dataset-v{FINGERPRINT_VERSION}\0".encode())
    digest.update(type(dataset).__name__.encode())
    metadata = _negative_metadata(dataset)
    if metadata:
        digest.update(
            json.dumps(metadata, sort_keys=True, default=str).encode()
        )
    sample_paths = []
    for index in range(len(dataset)):
        path = _sample_path(dataset, index)
        image_digest = sha256_file(path)
        target_digest = _target_digest(dataset, index)
        digest.update(f"{index}\0{image_digest}\0{target_digest}\n".encode())
        sample_paths.append(str(path))
    return {
        "version": FINGERPRINT_VERSION,
        "algorithm": "sha256-image-and-effective-target",
        "digest": digest.hexdigest(),
        "samples": len(dataset),
        "first_path": sample_paths[0] if sample_paths else None,
        "last_path": sample_paths[-1] if sample_paths else None,
        **_negative_metadata(dataset),
    }


def dataset_target_fingerprint(dataset: Dataset) -> dict[str, object]:
    """Identify ordered sample paths and effective targets without image I/O."""
    digest = hashlib.sha256()
    digest.update(
        f"multitask-target-dataset-v{TARGET_FINGERPRINT_VERSION}\0".encode()
    )
    digest.update(type(dataset).__name__.encode())
    metadata = _negative_metadata(dataset)
    if metadata:
        digest.update(
            json.dumps(metadata, sort_keys=True, default=str).encode()
        )
    sample_paths = []
    for index in range(len(dataset)):
        path = _sample_path(dataset, index)
        target_digest = _target_digest(dataset, index)
        digest.update(f"{index}\0{path}\0{target_digest}\n".encode())
        sample_paths.append(str(path))
    return {
        "version": TARGET_FINGERPRINT_VERSION,
        "algorithm": "sha256-ordered-path-and-effective-target",
        "digest": digest.hexdigest(),
        "samples": len(dataset),
        "first_path": sample_paths[0] if sample_paths else None,
        "last_path": sample_paths[-1] if sample_paths else None,
        **_negative_metadata(dataset),
    }


def deterministic_stratified_indices(
    dataset: Dataset,
    count: int,
    *,
    seed: int,
    task: HeadId,
) -> list[int]:
    """Select a stable class-balanced subset, including negative images."""
    if count <= 0:
        raise ValueError("Subset size must be positive")
    if count >= len(dataset):
        return list(range(len(dataset)))

    buckets: dict[str, list[int]] = defaultdict(list)
    for index in range(len(dataset)):
        labels = _sample_labels(dataset, index)
        keys = [f"class:{value}" for value in labels] or ["negative"]
        for key in keys:
            buckets[key].append(index)

    def rank(index: int) -> str:
        path = _sample_path(dataset, index)
        value = f"{seed}:{task}:{path}".encode()
        return hashlib.sha256(value).hexdigest()

    ordered = {
        key: sorted(indices, key=rank)
        for key, indices in sorted(buckets.items())
    }
    offsets = dict.fromkeys(ordered, 0)
    selected: list[int] = []
    selected_set: set[int] = set()
    while len(selected) < count:
        progressed = False
        for key, indices in ordered.items():
            offset = offsets[key]
            while offset < len(indices) and indices[offset] in selected_set:
                offset += 1
            offsets[key] = offset
            if offset >= len(indices):
                continue
            index = indices[offset]
            offsets[key] += 1
            selected.append(index)
            selected_set.add(index)
            progressed = True
            if len(selected) == count:
                break
        if not progressed:
            break

    if len(selected) < count:
        remaining = sorted(
            (
                index
                for index in range(len(dataset))
                if index not in selected_set
            ),
            key=rank,
        )
        selected.extend(remaining[: count - len(selected)])
    return selected


def create_subset_manifest(
    datasets: Mapping[HeadId, Dataset],
    fingerprints: Mapping[HeadId, Mapping[str, object]],
    *,
    count_per_task: int,
    seed: int,
) -> dict[str, object]:
    """Create a portable manifest of deterministic per-task sample indices."""
    tasks: dict[str, object] = {}
    for task, dataset in sorted(
        datasets.items(),
        key=lambda item: str(item[0]),
    ):
        indices = deterministic_stratified_indices(
            dataset,
            count_per_task,
            seed=seed,
            task=task,
        )
        tasks[str(task)] = {
            "dataset_fingerprint": fingerprints[task]["digest"],
            "indices": indices,
            "paths": [str(_sample_path(dataset, index)) for index in indices],
        }
    return {
        "version": SUBSET_MANIFEST_VERSION,
        "strategy": "class-round-robin-sha256",
        "seed": seed,
        "count_per_task": count_per_task,
        "tasks": tasks,
    }


def subset_datasets_from_manifest(
    datasets: Mapping[HeadId, Dataset],
    fingerprints: Mapping[HeadId, Mapping[str, object]],
    manifest: Mapping[str, object],
) -> dict[HeadId, Dataset]:
    """Apply a manifest after verifying dataset identities and paths."""
    if manifest.get("version") != SUBSET_MANIFEST_VERSION:
        raise ValueError("Unsupported subset manifest version")
    raw_tasks = manifest.get("tasks")
    if not isinstance(raw_tasks, dict):
        raise TypeError("Subset manifest must contain a task mapping")

    result: dict[HeadId, Dataset] = {}
    for task, dataset in datasets.items():
        raw_entry = raw_tasks.get(str(task))
        if not isinstance(raw_entry, dict):
            raise TypeError(f"Subset manifest has no task '{task}'")
        if raw_entry.get("dataset_fingerprint") != fingerprints[task].get(
            "digest"
        ):
            raise ValueError(f"Subset dataset fingerprint differs for '{task}'")
        raw_indices = raw_entry.get("indices")
        raw_paths = raw_entry.get("paths")
        if not isinstance(raw_indices, list) or not all(
            isinstance(index, int) for index in raw_indices
        ):
            raise TypeError(f"Subset indices are invalid for '{task}'")
        if not isinstance(raw_paths, list) or not all(
            isinstance(path, str) for path in raw_paths
        ):
            raise TypeError(f"Subset paths are invalid for '{task}'")
        indices = cast("Sequence[int]", raw_indices)
        expected_paths = [
            str(_sample_path(dataset, index)) for index in indices
        ]
        if expected_paths != raw_paths:
            raise ValueError(f"Subset paths differ for '{task}'")
        result[task] = Subset(dataset, list(indices))
    return result


def write_json(path: str | Path, value: object) -> None:
    """Write deterministic, human-readable JSON."""
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(value, indent=2, sort_keys=True, default=str) + "\n",
        encoding="utf-8",
    )
