"""Train the native four-output multi-task D-FINE model."""

# ruff: noqa: C901, TRY003

import os
from dataclasses import replace
from pathlib import Path
from typing import Literal, cast

import click
import torch
from torch.utils.data import DataLoader, DistributedSampler

from ultralytics_dfine.data import (
    DFINEDataset,
    DHRPDataset,
    YOLOKeypointDataset,
    load_dataset_yaml,
)
from ultralytics_dfine.engine import (
    MultiTaskTrainer,
    multitask_collate,
    stage_training_config,
)
from ultralytics_dfine.nn import DFINEDetectionModel, DFINEMultiTaskModel
from ultralytics_dfine.schemas import (
    COCO_FLIP_IDX,
    FIELD_FEATURE_CLASSES,
    HeadId,
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
type DatasetSplit = Literal["train", "val", "test"]


def _load_model(source: str) -> DFINEMultiTaskModel:
    path = Path(source)
    if source == "dfine-s":
        detector = DFINEDetectionModel.from_pretrained(
            list(DETECTOR_CLASSES[:-1])
        )
        detector.append_detection_class(DETECTOR_CLASSES[-1])
        return DFINEMultiTaskModel(detector)
    if not path.is_file():
        raise FileNotFoundError(f"Model checkpoint does not exist: {path}")
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if not isinstance(checkpoint, dict):
        raise TypeError("Model checkpoint must contain a dictionary")
    if checkpoint.get("format_version") == 2:
        return DFINEMultiTaskModel.from_checkpoint(path)
    return DFINEMultiTaskModel.from_detection_checkpoint(path)


def _loader(
    dataset: torch.utils.data.Dataset,
    *,
    batch_size: int,
    workers: int,
    shuffle: bool,
) -> DataLoader:
    world_size = int(os.environ.get("WORLD_SIZE", "1"))
    sampler = (
        DistributedSampler(
            dataset,
            num_replicas=world_size,
            rank=int(os.environ.get("RANK", "0")),
            shuffle=shuffle,
        )
        if world_size > 1 and shuffle
        else None
    )
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle if sampler is None else False,
        sampler=sampler,
        num_workers=workers,
        pin_memory=torch.cuda.is_available(),
        persistent_workers=workers > 0,
        prefetch_factor=2 if workers > 0 else None,
        drop_last=shuffle,
        collate_fn=multitask_collate,
    )


def _dhrp_annotations(root: Path, split: str) -> list[Path]:
    files = sorted(
        path.resolve() for path in (root / "annot").glob(f"{split}_set_*.json")
    )
    if not files:
        raise FileNotFoundError(f"DHRP has no {split} annotations")
    return files


def _audit_dataset(
    dataset: torch.utils.data.Dataset,
) -> dict[str, int]:
    if isinstance(dataset, (DFINEDataset, YOLOKeypointDataset)):
        return dataset.audit_annotations()
    if isinstance(dataset, DHRPDataset):
        count = dataset.__len__()
        return {
            "images": count,
            "label_files": count,
            "missing_label_files": 0,
            "rows": count,
            "ignored_rows": 0,
        }
    raise TypeError(f"Unsupported dataset type: {type(dataset).__name__}")


@click.command()
@click.option("--model", "model_source", default="dfine-s", show_default=True)
@click.option(
    "--object-data",
    type=click.Path(exists=True, path_type=Path),
    required=True,
)
@click.option(
    "--person-data",
    type=click.Path(exists=True, path_type=Path),
    required=True,
)
@click.option(
    "--dhrp-root",
    type=click.Path(exists=True, path_type=Path),
    required=True,
)
@click.option("--field-data", type=click.Path(exists=True, path_type=Path))
@click.option("--output-dir", type=click.Path(path_type=Path), required=True)
@click.option("--resume", type=click.Path(exists=True, path_type=Path))
@click.option("--stage", type=click.IntRange(1, 3), required=True)
@click.option("--epochs", type=click.IntRange(min=1), required=True)
@click.option("--steps-per-epoch", type=click.IntRange(min=1))
@click.option("--max-validation-batches", type=click.IntRange(min=1))
@click.option("--batch-size", type=click.IntRange(min=1), default=4)
@click.option("--validation-batch-size", type=click.IntRange(min=1), default=32)
@click.option("--workers", type=click.IntRange(min=0), default=4)
@click.option("--image-size", type=click.IntRange(min=32), default=640)
@click.option("--learning-rate", type=float, default=1e-4)
@click.option("--head-learning-rate", type=float)
@click.option("--classifier-learning-rate", type=float)
@click.option("--proposal-learning-rate", type=float)
@click.option("--weight-decay", type=click.FloatRange(min=0), default=1e-4)
@click.option("--warmup-steps", type=click.IntRange(min=0), default=500)
@click.option("--clip-max-norm", type=click.FloatRange(min=0), default=0.1)
@click.option(
    "--ema-decay", type=click.FloatRange(min=0, max=1), default=0.9999
)
@click.option("--ema-warmups", type=click.IntRange(min=0), default=1000)
@click.option("--amp/--no-amp", default=False)
@click.option("--device", default="cuda")
@click.option("--object-weight", type=float, default=1.0)
@click.option("--person-weight", type=float, default=1.0)
@click.option("--robot-weight", type=float, default=1.0)
@click.option("--field-weight", type=float, default=1.0)
@click.option("--run-name", default="dfine-multitask", show_default=True)
@click.option(
    "--wandb-project",
    default="multi-task-yolo-dfine",
    show_default=True,
)
@click.option("--wandb-group")
@click.option(
    "--wandb-mode",
    type=click.Choice(["online", "offline", "disabled"]),
    default="online",
    show_default=True,
)
@click.option("--wandb-log-interval", type=click.IntRange(min=1), default=20)
@click.option(
    "--wandb-log-checkpoints/--no-wandb-log-checkpoints",
    default=True,
)
@click.option(
    "--render-object-confidence", type=click.FloatRange(0, 1), default=0.25
)
@click.option(
    "--render-person-confidence", type=click.FloatRange(0, 1), default=0.5
)
@click.option(
    "--render-robot-confidence", type=click.FloatRange(0, 1), default=0.5
)
@click.option(
    "--render-field-confidence", type=click.FloatRange(0, 1), default=0.35
)
@click.option(
    "--render-keypoint-confidence", type=click.FloatRange(0, 1), default=0.5
)
@click.option("--render-confidence", type=click.FloatRange(0, 1), hidden=True)
@click.option("--render-max-detections", type=click.IntRange(min=1), default=20)
def main(
    *,
    model_source: str,
    object_data: Path,
    person_data: Path,
    dhrp_root: Path,
    field_data: Path | None,
    output_dir: Path,
    resume: Path | None,
    stage: int,
    epochs: int,
    steps_per_epoch: int | None,
    max_validation_batches: int | None,
    batch_size: int,
    validation_batch_size: int,
    workers: int,
    image_size: int,
    learning_rate: float,
    head_learning_rate: float | None,
    classifier_learning_rate: float | None,
    proposal_learning_rate: float | None,
    weight_decay: float,
    warmup_steps: int,
    clip_max_norm: float,
    ema_decay: float,
    ema_warmups: int,
    amp: bool,
    device: str,
    object_weight: float,
    person_weight: float,
    robot_weight: float,
    field_weight: float,
    run_name: str,
    wandb_project: str,
    wandb_group: str | None,
    wandb_mode: str,
    wandb_log_interval: int,
    wandb_log_checkpoints: bool,
    render_object_confidence: float,
    render_person_confidence: float,
    render_robot_confidence: float,
    render_field_confidence: float,
    render_keypoint_confidence: float,
    render_confidence: float | None,
    render_max_detections: int,
) -> None:
    if render_confidence is not None:
        render_object_confidence = render_confidence
        render_person_confidence = render_confidence
        render_robot_confidence = render_confidence
        render_field_confidence = render_confidence
        render_keypoint_confidence = render_confidence
    train_datasets: dict[HeadId, torch.utils.data.Dataset] = {}
    validation_datasets: dict[HeadId, torch.utils.data.Dataset] = {}
    object_definition = load_dataset_yaml(object_data)
    expected_object_names = DETECTOR_CLASSES[: len(object_definition.names)]
    if tuple(object_definition.names) != expected_object_names:
        raise click.BadParameter(
            "Object dataset class IDs must match the detector prefix: "
            + ", ".join(expected_object_names),
            param_hint="--object-data",
        )
    if stage >= 2:
        train_datasets[HeadId.OBJECT] = DFINEDataset(
            object_data,
            "train",
            image_size=image_size,
            num_detection_classes=8,
            ignore_unlisted_classes=True,
        )
    validation_datasets[HeadId.OBJECT] = DFINEDataset(
        object_data,
        "val",
        image_size=image_size,
        num_detection_classes=8,
        ignore_unlisted_classes=True,
    )

    for split, datasets in (
        ("train", train_datasets),
        ("val", validation_datasets),
    ):
        datasets[HeadId.PERSON_POSE] = YOLOKeypointDataset(
            person_data,
            cast("DatasetSplit", split),
            schema_id=HeadId.PERSON_POSE,
            keypoint_count=17,
            flip_idx=COCO_FLIP_IDX,
            global_class_ids=(7,),
            image_size=image_size,
        )
    train_datasets[HeadId.ROBOT_POSE] = DHRPDataset(
        dhrp_root,
        _dhrp_annotations(dhrp_root, "train"),
        image_size=image_size,
        training=True,
    )
    validation_datasets[HeadId.ROBOT_POSE] = DHRPDataset(
        dhrp_root,
        _dhrp_annotations(dhrp_root, "eval"),
        image_size=image_size,
    )
    if field_data is not None:
        field_definition = load_dataset_yaml(field_data)
        if sorted(field_definition.names) != sorted(FIELD_FEATURE_CLASSES):
            raise click.BadParameter(
                "Field dataset classes must be exactly: "
                + ", ".join(FIELD_FEATURE_CLASSES),
                param_hint="--field-data",
            )
        field_global_class_ids = tuple(
            DETECTOR_CLASSES.index(name) for name in field_definition.names
        )
        field_point_label_ids = tuple(
            FIELD_FEATURE_CLASSES.index(name) for name in field_definition.names
        )
        for split, datasets in (
            ("train", train_datasets),
            ("val", validation_datasets),
        ):
            datasets[HeadId.FIELD_FEATURES] = YOLOKeypointDataset(
                field_data,
                cast("DatasetSplit", split),
                schema_id=HeadId.FIELD_FEATURES,
                keypoint_count=1,
                keypoint_dimensions=2,
                flip_idx=(0,),
                global_class_ids=field_global_class_ids,
                image_size=image_size,
                point_set=True,
                point_label_ids=field_point_label_ids,
            )
    else:
        click.echo("Field data omitted; field head remains inactive")

    dataset_audits: dict[str, dict[str, int]] = {}
    for split, datasets in (
        ("train", train_datasets),
        ("val", validation_datasets),
    ):
        for task, dataset in datasets.items():
            report = _audit_dataset(dataset)
            key = f"{split}/{task}"
            dataset_audits[key] = report
            click.echo(f"Dataset audit {key}: {report}")

    model = _load_model(model_source)
    train_loaders = {
        task: _loader(
            dataset,
            batch_size=batch_size,
            workers=workers,
            shuffle=True,
        )
        for task, dataset in train_datasets.items()
    }
    validation_loaders = {
        task: _loader(
            dataset,
            batch_size=validation_batch_size,
            workers=workers,
            shuffle=False,
        )
        for task, dataset in validation_datasets.items()
    }

    weights = {
        HeadId.OBJECT: object_weight,
        HeadId.PERSON_POSE: person_weight,
        HeadId.ROBOT_POSE: robot_weight,
        HeadId.FIELD_FEATURES: field_weight,
    }
    config = stage_training_config(
        stage,
        output_dir=output_dir,
        epochs=epochs,
        device=device,
        learning_rate=learning_rate,
        sampling_weights={task: weights[task] for task in train_loaders},
    )
    if steps_per_epoch is not None:
        config = replace(config, steps_per_epoch=steps_per_epoch)
    role_learning_rates = dict(config.role_learning_rates)
    for role, value in (
        ("heads", head_learning_rate),
        ("classifiers", classifier_learning_rate),
        ("proposal", proposal_learning_rate),
    ):
        if value is not None:
            role_learning_rates[role] = value
    config = replace(
        config,
        run_name=f"{run_name}-stage-{stage}",
        wandb_project=wandb_project,
        wandb_group=wandb_group or run_name,
        wandb_mode=cast(
            "Literal['online', 'offline', 'disabled']",
            wandb_mode,
        ),
        wandb_log_interval=wandb_log_interval,
        wandb_log_checkpoints=wandb_log_checkpoints,
        max_validation_batches=max_validation_batches,
        render_object_confidence=render_object_confidence,
        render_person_confidence=render_person_confidence,
        render_robot_confidence=render_robot_confidence,
        render_field_confidence=render_field_confidence,
        render_keypoint_confidence=render_keypoint_confidence,
        render_max_detections=render_max_detections,
        dataset_audits=dataset_audits,
        batch_size_per_rank=batch_size,
        validation_batch_size=validation_batch_size,
        role_learning_rates=role_learning_rates,
        weight_decay=weight_decay,
        warmup_steps=warmup_steps,
        clip_max_norm=clip_max_norm,
        ema_decay=ema_decay,
        ema_warmups=ema_warmups,
        amp=amp,
    )
    trainer = MultiTaskTrainer(
        model,
        train_loaders,
        validation_loaders,
        config,
    )
    if resume is not None:
        trainer.resume(resume)
    trainer.fit()


if __name__ == "__main__":
    main()
