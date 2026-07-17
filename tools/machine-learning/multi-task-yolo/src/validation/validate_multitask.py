"""Standalone, reproducible validation for native multi-task D-FINE."""

# ruff: noqa: C901, S603, SIM115, TRY003

import gzip
import hashlib
import inspect
import json
import os
import subprocess
from collections.abc import Callable, Mapping
from functools import wraps
from pathlib import Path
from typing import Literal, cast

import click
import torch
import torch.distributed as dist
from torch.utils.data import DataLoader, Dataset, Subset

from ultralytics_dfine.data import (
    DFINEDataset,
    DHRPDataset,
    YOLOKeypointDataset,
    load_dataset_yaml,
)
from ultralytics_dfine.engine import multitask_collate
from ultralytics_dfine.engine.multitask_validator import MultiTaskValidator
from ultralytics_dfine.nn import DFINEMultiTaskModel
from ultralytics_dfine.schemas import (
    COCO_FLIP_IDX,
    FIELD_FEATURE_CLASSES,
    HeadId,
)
from utils.click_types import FiniteFloatRange
from validation.multitask_backends import (
    Nv12RoundtripModel,
    OnnxNv12Model,
    backend_metadata,
)
from validation.multitask_data import (
    create_subset_manifest,
    dataset_fingerprint,
    sha256_file,
    subset_datasets_from_manifest,
    write_json,
)

DETECTOR_CLASSES = (
    "Ball",
    "GoalPost",
    "LSpot",
    "PenaltySpot",
    "Robot",
    "TSpot",
    "XSpot",
    "Person",
)
DEFAULT_OBJECT_DATA = Path(
    "/home/alexschmander/datasets/multi-task-objects-hslvision-balanced.yaml"
)
DEFAULT_PERSON_DATA = Path("/home/alexschmander/datasets/coco-pose.yaml")
DEFAULT_DHRP_ROOT = Path("/home/alexschmander/datasets/DHRP")
DEFAULT_FIELD_DATA = Path(
    "/home/alexschmander/datasets/multi-task-field-features-pose.yaml"
)
type BackendName = Literal[
    "pytorch-rgb",
    "pytorch-nv12-roundtrip",
    "onnx-nv12",
]


class JsonlGzipSink:
    """Stream deterministic prediction records without retaining them in RAM."""

    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = gzip.open(path, "wt", encoding="utf-8")

    def __call__(self, record: dict[str, object]) -> None:
        self.file.write(
            json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
        )

    def close(self) -> None:
        self.file.close()


def _with_process_group_cleanup[**P, R](
    function: Callable[P, R],
) -> Callable[P, R]:
    """Destroy a process group even when validation exits exceptionally."""

    @wraps(function)
    def wrapped(*args: P.args, **kwargs: P.kwargs) -> R:
        try:
            return function(*args, **kwargs)
        finally:
            if dist.is_initialized():
                dist.destroy_process_group()

    return wrapped


def _dhrp_annotations(root: Path, split: str = "eval") -> list[Path]:
    files = sorted(
        path.resolve() for path in (root / "annot").glob(f"{split}_set_*.json")
    )
    if not files:
        raise FileNotFoundError(f"DHRP has no {split} annotations")
    return files


def _build_datasets(
    tasks: tuple[HeadId, ...],
    *,
    object_data: Path,
    person_data: Path,
    coco_robot_negative_manifest: Path | None,
    coco_robot_negative_role: str,
    dhrp_root: Path,
    dhrp_annotation_split: str,
    dhrp_person_negative_manifest: Path | None,
    dhrp_person_negative_role: str,
    field_data: Path,
    image_size: tuple[int, int],
) -> dict[HeadId, Dataset]:
    datasets: dict[HeadId, Dataset] = {}
    if HeadId.OBJECT in tasks:
        definition = load_dataset_yaml(object_data)
        if tuple(definition.names) != DETECTOR_CLASSES[: len(definition.names)]:
            raise ValueError("Object classes do not match the detector prefix")
        datasets[HeadId.OBJECT] = DFINEDataset(
            object_data,
            "val",
            image_size=image_size,
            num_detection_classes=len(DETECTOR_CLASSES),
            ignore_unlisted_classes=True,
        )
    if HeadId.PERSON_POSE in tasks:
        datasets[HeadId.PERSON_POSE] = YOLOKeypointDataset(
            person_data,
            "val",
            schema_id=HeadId.PERSON_POSE,
            keypoint_count=17,
            flip_idx=COCO_FLIP_IDX,
            global_class_ids=(7,),
            image_size=image_size,
            robot_negative_manifest=coco_robot_negative_manifest,
            robot_negative_roles=(coco_robot_negative_role,),
        )
    if HeadId.ROBOT_POSE in tasks:
        robot_dataset = DHRPDataset(
            dhrp_root,
            _dhrp_annotations(dhrp_root, dhrp_annotation_split),
            image_size=image_size,
            training=False,
            person_negative_manifest=dhrp_person_negative_manifest,
            person_negative_roles=(dhrp_person_negative_role,),
        )
        if dhrp_person_negative_manifest is None:
            datasets[HeadId.ROBOT_POSE] = robot_dataset
        else:
            eligible = [
                index
                for index, record in enumerate(robot_dataset.records)
                if record.person_negative_eligible
            ]
            if not eligible:
                raise ValueError(
                    "DHRP person-negative manifest selected no records"
                )
            datasets[HeadId.ROBOT_POSE] = Subset(robot_dataset, eligible)
    if HeadId.FIELD_FEATURES in tasks:
        definition = load_dataset_yaml(field_data)
        if sorted(definition.names) != sorted(FIELD_FEATURE_CLASSES):
            raise ValueError("Field classes do not match the field schema")
        global_ids = tuple(
            DETECTOR_CLASSES.index(name) for name in definition.names
        )
        point_ids = tuple(
            FIELD_FEATURE_CLASSES.index(name) for name in definition.names
        )
        datasets[HeadId.FIELD_FEATURES] = YOLOKeypointDataset(
            field_data,
            "val",
            schema_id=HeadId.FIELD_FEATURES,
            keypoint_count=1,
            keypoint_dimensions=2,
            flip_idx=(0,),
            global_class_ids=global_ids,
            image_size=image_size,
            horizontal_flip_probability=0.0,
            point_set=True,
            point_label_ids=point_ids,
        )
    return datasets


def _loader(dataset: Dataset, *, batch_size: int, workers: int) -> DataLoader:
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=workers,
        pin_memory=torch.cuda.is_available(),
        persistent_workers=workers > 0,
        prefetch_factor=2 if workers > 0 else None,
        collate_fn=multitask_collate,
    )


def _code_metadata() -> dict[str, object]:
    root = Path(__file__).resolve().parents[2]
    commands = {
        "revision": ["git", "rev-parse", "HEAD"],
        "diff": ["git", "diff", "--binary", "--", "."],
    }
    values: dict[str, object] = {}
    for key, command in commands.items():
        result = subprocess.run(
            command,
            cwd=root,
            check=False,
            capture_output=True,
            text=False,
        )
        if result.returncode != 0:
            values[key] = None
            continue
        output = result.stdout
        values[key] = (
            output.decode().strip()
            if key == "revision"
            else hashlib.sha256(output).hexdigest()
        )
    return values


def _load_backend(
    backend: BackendName,
    model_path: Path,
    manifest: Path | None,
    *,
    height: int,
    width: int,
    device: torch.device,
    onnx_provider: tuple[str, ...],
) -> torch.nn.Module:
    if backend == "onnx-nv12":
        if manifest is None:
            raise click.BadParameter(
                "--manifest is required for ONNX validation",
                param_hint="--manifest",
            )
        return OnnxNv12Model(
            model_path,
            manifest,
            height=height,
            width=width,
            providers=list(onnx_provider) or None,
        )
    model = DFINEMultiTaskModel.from_checkpoint(model_path).to(device)
    if backend == "pytorch-nv12-roundtrip":
        return Nv12RoundtripModel(model).to(device)
    return model


def _resolve_tasks(raw_tasks: tuple[str, ...]) -> tuple[HeadId, ...]:
    order = tuple(HeadId)
    if not raw_tasks or "all" in raw_tasks:
        return order
    requested = {HeadId(value) for value in raw_tasks}
    return tuple(task for task in order if task in requested)


def _distributed_context(device_name: str) -> tuple[int, int, torch.device]:
    rank = int(os.environ.get("RANK", "0"))
    world_size = int(os.environ.get("WORLD_SIZE", "1"))
    local_rank = int(os.environ.get("LOCAL_RANK", "0"))
    under_torchrun = "LOCAL_RANK" in os.environ
    is_cuda = device_name.startswith("cuda")
    if is_cuda and under_torchrun:
        device = torch.device("cuda", local_rank)
        torch.cuda.set_device(device)
    else:
        device = torch.device(device_name)
    if world_size > 1 and not dist.is_initialized():
        backend = "nccl" if is_cuda else "gloo"
        kwargs: dict[str, object] = {"backend": backend}
        try:
            supports_device_id = (
                "device_id"
                in inspect.signature(dist.init_process_group).parameters
            )
        except (TypeError, ValueError):
            supports_device_id = False
        if backend == "nccl" and supports_device_id:
            kwargs["device_id"] = device
        dist.init_process_group(**kwargs)
    return rank, world_size, device


def _merge_subset_manifests(
    manifests: list[Mapping[str, object]],
) -> dict[str, object]:
    """Merge disjoint rank-local task manifests with strict header checks."""
    if not manifests:
        raise ValueError("No subset manifests to merge")
    header_keys = ("version", "strategy", "seed", "count_per_task")
    header = {key: manifests[0].get(key) for key in header_keys}
    tasks: dict[str, object] = {}
    for manifest in manifests:
        if any(manifest.get(key) != value for key, value in header.items()):
            raise ValueError("Distributed subset manifest headers differ")
        raw_tasks = manifest.get("tasks")
        if not isinstance(raw_tasks, dict):
            raise TypeError("Subset manifest must contain a task mapping")
        overlap = tasks.keys() & raw_tasks.keys()
        if overlap:
            raise ValueError(
                "Duplicate distributed subset tasks: "
                + ", ".join(sorted(overlap))
            )
        tasks.update(raw_tasks)
    return {**header, "tasks": dict(sorted(tasks.items()))}


def _distributed_subset_manifest(
    local_manifest: Mapping[str, object],
    destination: Path,
    *,
    rank: int,
    world_size: int,
) -> dict[str, object]:
    """Gather all task subsets, write once on rank zero, and broadcast."""
    if world_size == 1:
        merged = dict(local_manifest)
        write_json(destination, merged)
        return merged
    gathered: list[Mapping[str, object] | None] = [None] * world_size
    dist.all_gather_object(gathered, dict(local_manifest))
    payload: list[dict[str, object] | None] = [None]
    if rank == 0:
        complete = [manifest for manifest in gathered if manifest is not None]
        if len(complete) != world_size:
            raise RuntimeError("A distributed subset manifest rank is missing")
        payload[0] = _merge_subset_manifests(complete)
        write_json(destination, payload[0])
    dist.broadcast_object_list(payload, src=0)
    merged = payload[0]
    if merged is None:
        raise RuntimeError("Rank zero did not broadcast a subset manifest")
    return merged


def _merge_shards(output_dir: Path, world_size: int) -> None:
    metrics: dict[str, object] = {}
    fingerprints: dict[str, object] = {}
    cross_pose_shards: list[Path] = []
    with gzip.open(
        output_dir / "predictions.jsonl.gz",
        "wt",
        encoding="utf-8",
    ) as destination:
        for rank in range(world_size):
            shard = output_dir / "shards" / f"rank-{rank:02d}"
            raw_metrics = json.loads((shard / "metrics.json").read_text())
            metrics.update(raw_metrics)
            raw_metadata = json.loads((shard / "metadata.json").read_text())
            fingerprints.update(raw_metadata["dataset_fingerprints"])
            predictions = shard / "predictions.jsonl.gz"
            if predictions.is_file():
                with gzip.open(predictions, "rt", encoding="utf-8") as source:
                    for line in source:
                        destination.write(line)
            cross_pose_predictions = shard / "cross_pose_predictions.jsonl.gz"
            if cross_pose_predictions.is_file():
                cross_pose_shards.append(cross_pose_predictions)
    if cross_pose_shards:
        with gzip.open(
            output_dir / "cross_pose_predictions.jsonl.gz",
            "wt",
            encoding="utf-8",
        ) as destination:
            for predictions in cross_pose_shards:
                with gzip.open(predictions, "rt", encoding="utf-8") as source:
                    for line in source:
                        destination.write(line)
    write_json(output_dir / "metrics.json", metrics)
    metadata = json.loads(
        (output_dir / "shards" / "rank-00" / "metadata.json").read_text()
    )
    metadata["dataset_fingerprints"] = fingerprints
    metadata["world_size"] = world_size
    write_json(output_dir / "metadata.json", metadata)
    config = json.loads(
        (output_dir / "shards" / "rank-00" / "config.json").read_text()
    )
    config["tasks"] = sorted(fingerprints)
    write_json(output_dir / "config.json", config)


@click.command()
@click.option(
    "--model",
    "model_path",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    required=True,
)
@click.option(
    "--backend",
    type=click.Choice(["pytorch-rgb", "pytorch-nv12-roundtrip", "onnx-nv12"]),
    default="pytorch-rgb",
    show_default=True,
)
@click.option(
    "--manifest",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--onnx-provider",
    multiple=True,
    help="Ordered ONNX Runtime providers; defaults to CPU.",
)
@click.option(
    "--task",
    "raw_tasks",
    multiple=True,
    type=click.Choice(["all", *(str(task) for task in HeadId)]),
    default=("all",),
)
@click.option(
    "--object-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_OBJECT_DATA,
    show_default=True,
)
@click.option(
    "--person-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_PERSON_DATA,
    show_default=True,
)
@click.option(
    "--coco-robot-negative-manifest",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--coco-robot-negative-role",
    type=click.Choice(
        ["train_negative", "primary_evaluation", "stress_evaluation"]
    ),
    default="primary_evaluation",
    show_default=True,
)
@click.option(
    "--dhrp-root",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    default=DEFAULT_DHRP_ROOT,
    show_default=True,
)
@click.option(
    "--dhrp-annotation-split",
    type=click.Choice(["train", "eval"]),
    default="eval",
    show_default=True,
)
@click.option(
    "--dhrp-person-negative-manifest",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--dhrp-person-negative-role",
    type=click.Choice(
        [
            "train_negative",
            "primary_evaluation",
            "stress_evaluation",
            "loss_holdout_validation",
        ]
    ),
    default="primary_evaluation",
    show_default=True,
)
@click.option(
    "--field-data",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
    default=DEFAULT_FIELD_DATA,
    show_default=True,
)
@click.option("--output-dir", type=click.Path(path_type=Path), required=True)
@click.option("--height", type=click.IntRange(min=32), default=448)
@click.option("--width", type=click.IntRange(min=32), default=544)
@click.option("--batch-size", type=click.IntRange(min=1), default=32)
@click.option("--workers", type=click.IntRange(min=0), default=4)
@click.option("--device", default="cuda", show_default=True)
@click.option("--confidence", type=FiniteFloatRange(0, 1), default=0.001)
@click.option(
    "--person-visibility-alpha",
    "person_visibility_alphas",
    type=FiniteFloatRange(min=0),
    multiple=True,
    default=(0.0,),
    show_default=True,
    help=(
        "Person evaluator score exponent; repeat to sweep values. The first "
        "value is emitted as person/map and in prediction records."
    ),
)
@click.option(
    "--robot-visibility-alpha",
    "robot_visibility_alphas",
    type=FiniteFloatRange(min=0),
    multiple=True,
    default=(0.0,),
    show_default=True,
    help=(
        "Robot evaluator score exponent; repeat to sweep values. The first "
        "value is emitted as robot/map and in prediction records."
    ),
)
@click.option(
    "--field-normalization",
    type=FiniteFloatRange(min=0, min_open=True),
    default=1.0,
)
@click.option("--max-batches", type=click.IntRange(min=1))
@click.option(
    "--subset-manifest",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--write-subset-manifest",
    type=click.Path(dir_okay=False, path_type=Path),
)
@click.option("--subset-size", type=click.IntRange(min=1), default=512)
@click.option("--subset-seed", type=int, default=20260716)
@click.option("--predictions/--no-predictions", default=True)
@click.option(
    "--cross-pose-negatives/--no-cross-pose-negatives",
    default=False,
    help=(
        "Evaluate each pose head on the opposite pose dataset as an empty-"
        "target false-detection set. Writes a separate paired-record sidecar."
    ),
)
@click.option(
    "--cross-pose-visibility-alpha",
    "cross_pose_visibility_alphas",
    type=FiniteFloatRange(min=0),
    multiple=True,
    default=(0.0, 1.0),
    show_default=True,
    help=(
        "Additional mean-visibility score exponent for cross-pose negatives; "
        "repeat to evaluate more values."
    ),
)
@_with_process_group_cleanup
def main(
    *,
    model_path: Path,
    backend: BackendName,
    manifest: Path | None,
    onnx_provider: tuple[str, ...],
    raw_tasks: tuple[str, ...],
    object_data: Path,
    person_data: Path,
    coco_robot_negative_manifest: Path | None,
    coco_robot_negative_role: str,
    dhrp_root: Path,
    dhrp_annotation_split: str,
    dhrp_person_negative_manifest: Path | None,
    dhrp_person_negative_role: str,
    field_data: Path,
    output_dir: Path,
    height: int,
    width: int,
    batch_size: int,
    workers: int,
    device: str,
    confidence: float,
    person_visibility_alphas: tuple[float, ...],
    robot_visibility_alphas: tuple[float, ...],
    field_normalization: float,
    max_batches: int | None,
    subset_manifest: Path | None,
    write_subset_manifest: Path | None,
    subset_size: int,
    subset_seed: int,
    predictions: bool,
    cross_pose_negatives: bool,
    cross_pose_visibility_alphas: tuple[float, ...],
) -> None:
    """Validate a checkpoint or packed-NV12 ONNX model on any task set."""
    if height % 2 or width % 2:
        raise click.BadParameter("Height and width must be even")
    if subset_manifest is not None and write_subset_manifest is not None:
        raise click.UsageError(
            "Use either --subset-manifest or --write-subset-manifest"
        )
    rank, world_size, torch_device = _distributed_context(device)
    tasks = _resolve_tasks(raw_tasks)
    assigned_tasks = tasks[rank::world_size]
    if not assigned_tasks:
        raise click.UsageError("More distributed ranks than requested tasks")
    shard_dir = (
        output_dir
        if world_size == 1
        else output_dir / "shards" / f"rank-{rank:02d}"
    )
    if shard_dir.exists() and any(shard_dir.iterdir()):
        raise click.FileError(
            str(shard_dir),
            hint="output directory is not empty",
        )
    shard_dir.mkdir(parents=True, exist_ok=True)

    datasets = _build_datasets(
        assigned_tasks,
        object_data=object_data,
        person_data=person_data,
        coco_robot_negative_manifest=coco_robot_negative_manifest,
        coco_robot_negative_role=coco_robot_negative_role,
        dhrp_root=dhrp_root,
        dhrp_annotation_split=dhrp_annotation_split,
        dhrp_person_negative_manifest=dhrp_person_negative_manifest,
        dhrp_person_negative_role=dhrp_person_negative_role,
        field_data=field_data,
        image_size=(height, width),
    )
    fingerprints = {
        task: dataset_fingerprint(dataset) for task, dataset in datasets.items()
    }
    if write_subset_manifest is not None:
        local_subset = create_subset_manifest(
            datasets,
            fingerprints,
            count_per_task=subset_size,
            seed=subset_seed,
        )
        subset = _distributed_subset_manifest(
            local_subset,
            write_subset_manifest,
            rank=rank,
            world_size=world_size,
        )
        datasets = subset_datasets_from_manifest(
            datasets,
            fingerprints,
            cast("Mapping[str, object]", subset),
        )
    elif subset_manifest is not None:
        value = json.loads(subset_manifest.read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise TypeError("Subset manifest must contain a JSON object")
        datasets = subset_datasets_from_manifest(
            datasets,
            fingerprints,
            value,
        )

    model = _load_backend(
        backend,
        model_path,
        manifest,
        height=height,
        width=width,
        device=torch_device,
        onnx_provider=onnx_provider,
    )
    validator_device = (
        torch.device("cpu") if backend == "onnx-nv12" else torch_device
    )
    validator = MultiTaskValidator(
        cast(DFINEMultiTaskModel, model),
        {
            task: _loader(dataset, batch_size=batch_size, workers=workers)
            for task, dataset in datasets.items()
        },
        device=validator_device,
        confidence=confidence,
        field_normalization=field_normalization,
        person_visibility_alphas=person_visibility_alphas,
        robot_visibility_alphas=robot_visibility_alphas,
        cross_pose_visibility_alphas=(
            cross_pose_visibility_alphas if cross_pose_negatives else None
        ),
    )
    sink = (
        JsonlGzipSink(shard_dir / "predictions.jsonl.gz")
        if predictions
        else None
    )
    cross_pose_sink = (
        JsonlGzipSink(shard_dir / "cross_pose_predictions.jsonl.gz")
        if predictions
        and cross_pose_negatives
        and any(
            task in {HeadId.PERSON_POSE, HeadId.ROBOT_POSE}
            for task in assigned_tasks
        )
        else None
    )
    try:
        metrics = validator.run(
            max_batches=max_batches,
            record_sink=sink,
            cross_pose_record_sink=cross_pose_sink,
        )
    finally:
        if sink is not None:
            sink.close()
        if cross_pose_sink is not None:
            cross_pose_sink.close()

    config = {
        "backend": backend,
        "model": str(model_path.resolve()),
        "manifest": str(manifest.resolve()) if manifest else None,
        "height": height,
        "width": width,
        "batch_size": batch_size,
        "workers": workers,
        "confidence": confidence,
        "person_visibility_alphas": list(person_visibility_alphas),
        "robot_visibility_alphas": list(robot_visibility_alphas),
        "field_normalization": field_normalization,
        "max_batches": max_batches,
        "tasks": [str(task) for task in assigned_tasks],
        "object_data": str(object_data.resolve()),
        "person_data": str(person_data.resolve()),
        "dhrp_root": str(dhrp_root.resolve()),
        "dhrp_annotation_split": dhrp_annotation_split,
        "dhrp_person_negative_manifest": (
            str(dhrp_person_negative_manifest.resolve())
            if dhrp_person_negative_manifest is not None
            else None
        ),
        "dhrp_person_negative_role": dhrp_person_negative_role,
        "coco_robot_negative_manifest": (
            str(coco_robot_negative_manifest.resolve())
            if coco_robot_negative_manifest is not None
            else None
        ),
        "coco_robot_negative_role": coco_robot_negative_role,
        "field_data": str(field_data.resolve()),
        "subset_manifest": (
            str((subset_manifest or write_subset_manifest).resolve())
            if subset_manifest is not None or write_subset_manifest is not None
            else None
        ),
    }
    if cross_pose_negatives:
        config.update(
            {
                "cross_pose_negatives": True,
                "cross_pose_visibility_alphas": list(
                    dict.fromkeys(cross_pose_visibility_alphas)
                ),
            }
        )
    metadata = {
        "architecture": "dfine-multitask",
        "checkpoint_sha256": sha256_file(model_path),
        "backend": dict(backend_metadata(model)),
        "dataset_fingerprints": {
            str(task): fingerprint for task, fingerprint in fingerprints.items()
        },
        "code": _code_metadata(),
        "rank": rank,
        "world_size": world_size,
    }
    write_json(shard_dir / "metrics.json", metrics)
    write_json(shard_dir / "config.json", config)
    write_json(shard_dir / "metadata.json", metadata)

    if world_size > 1:
        dist.barrier()
        if rank == 0:
            _merge_shards(output_dir, world_size)
        dist.barrier()
    if rank == 0:
        click.echo(f"Validation results: {output_dir.resolve()}")


if __name__ == "__main__":
    main()
