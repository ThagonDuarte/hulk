"""Train the native four-output multi-task D-FINE model."""

# ruff: noqa: C901, TRY003

import hashlib
import os
import random
from dataclasses import asdict, replace
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
from utils.click_types import FiniteFloatRange
from validation.multitask_data import dataset_target_fingerprint

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
type ImageSize = int | tuple[int, int]


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
    if checkpoint.get("format_version") in {2, 3}:
        return DFINEMultiTaskModel.from_checkpoint(path)
    return DFINEMultiTaskModel.from_detection_checkpoint(path)


def _loader(
    dataset: torch.utils.data.Dataset,
    *,
    batch_size: int,
    workers: int,
    shuffle: bool,
    seed: int,
    deterministic: bool,
) -> DataLoader:
    world_size = int(os.environ.get("WORLD_SIZE", "1"))
    rank = int(os.environ.get("RANK", "0"))
    sampler = (
        DistributedSampler(
            dataset,
            num_replicas=world_size,
            rank=rank,
            shuffle=shuffle,
            seed=seed,
        )
        if world_size > 1 and shuffle
        else None
    )
    generator = torch.Generator().manual_seed(seed + rank)
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle if sampler is None else False,
        sampler=sampler,
        num_workers=workers,
        pin_memory=torch.cuda.is_available(),
        persistent_workers=workers > 0 and not deterministic,
        prefetch_factor=2 if workers > 0 else None,
        drop_last=shuffle,
        collate_fn=multitask_collate,
        generator=generator,
        worker_init_fn=_seed_worker,
    )


def _seed_worker(_worker_id: int) -> None:
    worker_seed = torch.initial_seed() % (2**32)
    random.seed(worker_seed)


def _resolve_image_size(
    *,
    square: int,
    height: int | None,
    width: int | None,
    option_prefix: str,
) -> ImageSize:
    if height is None and width is None:
        return square
    if height is None or width is None:
        raise click.BadParameter(
            f"--{option_prefix}-height and --{option_prefix}-width must be "
            "provided together"
        )
    return (height, width)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for block in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _source_sha256() -> str:
    digest = hashlib.sha256()
    source_root = Path(__file__).parents[1]
    for path in sorted(source_root.rglob("*.py")):
        digest.update(str(path.relative_to(source_root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _profile_training_tasks(profile: str) -> set[HeadId]:
    if profile == "stage1":
        return {
            HeadId.PERSON_POSE,
            HeadId.ROBOT_POSE,
            HeadId.FIELD_FEATURES,
        }
    if profile in {"stage2", "stage3"}:
        return set(HeadId)
    if profile == "field_head_only":
        return {HeadId.FIELD_FEATURES}
    raise ValueError(f"Unknown trainable profile: {profile}")


def _configured_model(
    model: DFINEMultiTaskModel,
    *,
    field_head_variant: str | None,
    field_refinement_dim: int | None,
    field_refinement_scale: float | None,
    field_classification_mode: str | None,
    field_class_weight: float | None,
    field_point_weight: float | None,
    field_focal_alpha: float | None,
    field_focal_gamma: float | None,
) -> DFINEMultiTaskModel:
    field_head_updates = {
        key: value
        for key, value in (
            ("variant", field_head_variant),
            ("refinement_dim", field_refinement_dim),
            ("refinement_scale", field_refinement_scale),
        )
        if value is not None
    }
    head_config = replace(
        model.head_config,
        field_features=replace(
            model.head_config.field_features,
            **field_head_updates,
        ),
    )
    field_loss_updates = {
        key: value
        for key, value in (
            ("classification_mode", field_classification_mode),
            ("class_weight", field_class_weight),
            ("point_weight", field_point_weight),
            ("focal_alpha", field_focal_alpha),
            ("focal_gamma", field_focal_gamma),
        )
        if value is not None
    }
    loss_config = replace(
        model.loss_config,
        field_features=replace(
            model.loss_config.field_features,
            **field_loss_updates,
        ),
    )
    if head_config == model.head_config:
        model.loss_config = loss_config
        return model
    source_state = model.state_dict()
    configured = DFINEMultiTaskModel(
        model.detector,
        head_config=head_config,
        loss_config=loss_config,
    )
    incompatible = configured.load_state_dict(source_state, strict=False)
    unsupported = [
        name
        for name in (*incompatible.missing_keys, *incompatible.unexpected_keys)
        if "spatial_refiner" not in name
    ]
    if unsupported:
        raise RuntimeError(
            "Head configuration produced incompatible weights: "
            + ", ".join(unsupported)
        )
    return configured


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
@click.option(
    "--resume-mode",
    type=click.Choice(["exact", "branch"]),
    default="exact",
    show_default=True,
)
@click.option("--stage", type=click.IntRange(1, 3), required=True)
@click.option(
    "--trainable-profile",
    type=click.Choice(
        [
            "stage1",
            "stage2",
            "stage3",
            "field_head_only",
        ]
    ),
)
@click.option("--epochs", type=click.IntRange(min=1), required=True)
@click.option("--steps-per-epoch", type=click.IntRange(min=1))
@click.option("--max-validation-batches", type=click.IntRange(min=1))
@click.option("--validation-interval", type=click.IntRange(min=1), default=1)
@click.option("--batch-size", type=click.IntRange(min=1), default=4)
@click.option("--validation-batch-size", type=click.IntRange(min=1), default=32)
@click.option("--workers", type=click.IntRange(min=0), default=4)
@click.option("--image-size", type=click.IntRange(min=32), default=640)
@click.option("--train-height", type=click.IntRange(min=32))
@click.option("--train-width", type=click.IntRange(min=32))
@click.option("--validation-height", type=click.IntRange(min=32))
@click.option("--validation-width", type=click.IntRange(min=32))
@click.option("--seed", type=int, default=0, show_default=True)
@click.option("--deterministic/--no-deterministic", default=True)
@click.option("--learning-rate", type=FiniteFloatRange(min=0), default=1e-4)
@click.option("--head-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--person-head-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--robot-head-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--field-head-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--classifier-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--proposal-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--decoder-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--encoder-last-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--backbone-last-learning-rate", type=FiniteFloatRange(min=0))
@click.option("--weight-decay", type=FiniteFloatRange(min=0), default=1e-4)
@click.option("--warmup-steps", type=click.IntRange(min=0), default=500)
@click.option(
    "--learning-rate-schedule",
    type=click.Choice(["constant"]),
    default="constant",
    show_default=True,
)
@click.option("--clip-max-norm", type=FiniteFloatRange(min=0), default=0.1)
@click.option(
    "--ema-decay", type=FiniteFloatRange(min=0, max=1), default=0.9999
)
@click.option("--ema-warmups", type=click.IntRange(min=0), default=1000)
@click.option("--amp/--no-amp", default=False)
@click.option(
    "--freeze-frozen-bn-stats/--update-frozen-bn-stats",
    default=False,
)
@click.option(
    "--freeze-all-bn-stats/--update-trainable-bn-stats",
    default=False,
    show_default=True,
)
@click.option("--sync-batch-norm/--no-sync-batch-norm", default=False)
@click.option(
    "--ddp-find-unused-parameters/--no-ddp-find-unused-parameters",
    default=True,
    show_default=True,
)
@click.option(
    "--strict-deterministic/--warn-nondeterministic",
    default=False,
    show_default=True,
)
@click.option(
    "--sdpa-backend",
    type=click.Choice(["auto", "math"]),
    default="auto",
    show_default=True,
)
@click.option(
    "--field-head-variant",
    type=click.Choice(["query_decoder", "spatial_refine"]),
)
@click.option("--field-refinement-dim", type=click.IntRange(min=1))
@click.option(
    "--field-refinement-scale",
    type=FiniteFloatRange(min=0, min_open=True),
)
@click.option(
    "--field-classification-mode",
    type=click.Choice(["binary"]),
)
@click.option("--field-class-weight", type=FiniteFloatRange(min=0))
@click.option("--field-point-weight", type=FiniteFloatRange(min=0))
@click.option(
    "--field-focal-alpha",
    type=FiniteFloatRange(min=0, max=1),
)
@click.option("--field-focal-gamma", type=FiniteFloatRange(min=0))
@click.option(
    "--field-augmentation-profile",
    type=click.Choice(["basic"]),
    default="basic",
    show_default=True,
)
@click.option("--device", default="cuda")
@click.option("--object-weight", type=FiniteFloatRange(min=0), default=1.0)
@click.option("--person-weight", type=FiniteFloatRange(min=0), default=1.0)
@click.option("--robot-weight", type=FiniteFloatRange(min=0), default=1.0)
@click.option("--field-weight", type=FiniteFloatRange(min=0), default=1.0)
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
    "--save-named-best-checkpoints/--no-save-named-best-checkpoints",
    default=True,
)
@click.option(
    "--render-object-confidence", type=FiniteFloatRange(0, 1), default=0.25
)
@click.option(
    "--render-person-confidence", type=FiniteFloatRange(0, 1), default=0.5
)
@click.option(
    "--render-robot-confidence", type=FiniteFloatRange(0, 1), default=0.5
)
@click.option(
    "--render-field-confidence", type=FiniteFloatRange(0, 1), default=0.35
)
@click.option(
    "--render-keypoint-confidence", type=FiniteFloatRange(0, 1), default=0.5
)
@click.option("--render-confidence", type=FiniteFloatRange(0, 1), hidden=True)
@click.option("--render-max-detections", type=click.IntRange(min=1), default=20)
@click.option("--allow-existing-output", is_flag=True)
def main(
    *,
    model_source: str,
    object_data: Path,
    person_data: Path,
    dhrp_root: Path,
    field_data: Path | None,
    output_dir: Path,
    resume: Path | None,
    resume_mode: str,
    stage: int,
    trainable_profile: str | None,
    epochs: int,
    steps_per_epoch: int | None,
    max_validation_batches: int | None,
    validation_interval: int,
    batch_size: int,
    validation_batch_size: int,
    workers: int,
    image_size: int,
    train_height: int | None,
    train_width: int | None,
    validation_height: int | None,
    validation_width: int | None,
    seed: int,
    deterministic: bool,
    learning_rate: float,
    head_learning_rate: float | None,
    person_head_learning_rate: float | None,
    robot_head_learning_rate: float | None,
    field_head_learning_rate: float | None,
    classifier_learning_rate: float | None,
    proposal_learning_rate: float | None,
    decoder_learning_rate: float | None,
    encoder_last_learning_rate: float | None,
    backbone_last_learning_rate: float | None,
    weight_decay: float,
    warmup_steps: int,
    learning_rate_schedule: str,
    clip_max_norm: float,
    ema_decay: float,
    ema_warmups: int,
    amp: bool,
    freeze_frozen_bn_stats: bool,
    freeze_all_bn_stats: bool,
    sync_batch_norm: bool,
    ddp_find_unused_parameters: bool,
    strict_deterministic: bool,
    sdpa_backend: str,
    field_head_variant: str | None,
    field_refinement_dim: int | None,
    field_refinement_scale: float | None,
    field_classification_mode: str | None,
    field_class_weight: float | None,
    field_point_weight: float | None,
    field_focal_alpha: float | None,
    field_focal_gamma: float | None,
    field_augmentation_profile: str,
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
    save_named_best_checkpoints: bool,
    render_object_confidence: float,
    render_person_confidence: float,
    render_robot_confidence: float,
    render_field_confidence: float,
    render_keypoint_confidence: float,
    render_confidence: float | None,
    render_max_detections: int,
    allow_existing_output: bool,
) -> None:
    profile = trainable_profile or f"stage{stage}"
    training_tasks = _profile_training_tasks(profile)
    if profile == "field_head_only" and field_data is None:
        raise click.BadParameter(
            f"Training profile '{profile}' requires field data",
            param_hint="--field-data",
        )
    if field_data is None:
        training_tasks.discard(HeadId.FIELD_FEATURES)
    train_image_size = _resolve_image_size(
        square=image_size,
        height=train_height,
        width=train_width,
        option_prefix="train",
    )
    validation_image_size = _resolve_image_size(
        square=image_size,
        height=validation_height,
        width=validation_width,
        option_prefix="validation",
    )
    random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
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
    if HeadId.OBJECT in training_tasks:
        train_datasets[HeadId.OBJECT] = DFINEDataset(
            object_data,
            "train",
            image_size=train_image_size,
            num_detection_classes=8,
            ignore_unlisted_classes=True,
        )
    validation_datasets[HeadId.OBJECT] = DFINEDataset(
        object_data,
        "val",
        image_size=validation_image_size,
        num_detection_classes=8,
        ignore_unlisted_classes=True,
    )

    validation_datasets[HeadId.PERSON_POSE] = YOLOKeypointDataset(
        person_data,
        "val",
        schema_id=HeadId.PERSON_POSE,
        keypoint_count=17,
        flip_idx=COCO_FLIP_IDX,
        global_class_ids=(7,),
        image_size=validation_image_size,
    )
    if HeadId.PERSON_POSE in training_tasks:
        train_datasets[HeadId.PERSON_POSE] = YOLOKeypointDataset(
            person_data,
            "train",
            schema_id=HeadId.PERSON_POSE,
            keypoint_count=17,
            flip_idx=COCO_FLIP_IDX,
            global_class_ids=(7,),
            image_size=train_image_size,
        )
    if HeadId.ROBOT_POSE in training_tasks:
        train_datasets[HeadId.ROBOT_POSE] = DHRPDataset(
            dhrp_root,
            _dhrp_annotations(dhrp_root, "train"),
            image_size=train_image_size,
            training=True,
        )
    validation_datasets[HeadId.ROBOT_POSE] = DHRPDataset(
        dhrp_root,
        _dhrp_annotations(dhrp_root, "eval"),
        image_size=validation_image_size,
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
        validation_datasets[HeadId.FIELD_FEATURES] = YOLOKeypointDataset(
            field_data,
            "val",
            schema_id=HeadId.FIELD_FEATURES,
            keypoint_count=1,
            keypoint_dimensions=2,
            flip_idx=(0,),
            global_class_ids=field_global_class_ids,
            image_size=validation_image_size,
            point_set=True,
            point_label_ids=field_point_label_ids,
        )
        if HeadId.FIELD_FEATURES in training_tasks:
            train_datasets[HeadId.FIELD_FEATURES] = YOLOKeypointDataset(
                field_data,
                "train",
                schema_id=HeadId.FIELD_FEATURES,
                keypoint_count=1,
                keypoint_dimensions=2,
                flip_idx=(0,),
                global_class_ids=field_global_class_ids,
                image_size=train_image_size,
                augmentation_profile=cast(
                    "Literal['basic']",
                    field_augmentation_profile,
                ),
                point_set=True,
                point_label_ids=field_point_label_ids,
            )
    else:
        click.echo("Field data omitted; field head remains inactive")

    dataset_audits: dict[str, dict[str, int]] = {}
    dataset_fingerprints: dict[str, dict[str, object]] = {}
    for split, datasets in (
        ("train", train_datasets),
        ("val", validation_datasets),
    ):
        for task, dataset in datasets.items():
            report = _audit_dataset(dataset)
            key = f"{split}/{task}"
            dataset_audits[key] = report
            click.echo(f"Dataset audit {key}: {report}")
            fingerprint = dataset_target_fingerprint(dataset)
            dataset_fingerprints[key] = fingerprint
            click.echo(f"Dataset fingerprint {key}: {fingerprint['digest']}")

    initialization_source = str(resume) if resume is not None else model_source
    model = _configured_model(
        _load_model(initialization_source),
        field_head_variant=field_head_variant,
        field_refinement_dim=field_refinement_dim,
        field_refinement_scale=field_refinement_scale,
        field_classification_mode=field_classification_mode,
        field_class_weight=field_class_weight,
        field_point_weight=field_point_weight,
        field_focal_alpha=field_focal_alpha,
        field_focal_gamma=field_focal_gamma,
    )
    train_loaders = {}
    for index, (task, dataset) in enumerate(train_datasets.items()):
        train_loaders[task] = _loader(
            dataset,
            batch_size=batch_size,
            workers=workers,
            shuffle=True,
            seed=seed + 10_000 * index,
            deterministic=deterministic,
        )
    validation_loaders = {}
    for index, (task, dataset) in enumerate(validation_datasets.items()):
        validation_loaders[task] = _loader(
            dataset,
            batch_size=validation_batch_size,
            workers=workers,
            shuffle=False,
            seed=seed + 100_000 + 10_000 * index,
            deterministic=deterministic,
        )

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
        trainable_profile=trainable_profile,
    )
    if steps_per_epoch is not None:
        config = replace(config, steps_per_epoch=steps_per_epoch)
    role_learning_rates = dict(config.role_learning_rates)
    for role, value in (
        ("heads", head_learning_rate),
        ("person_head", person_head_learning_rate),
        ("robot_head", robot_head_learning_rate),
        ("field_head", field_head_learning_rate),
        ("classifiers", classifier_learning_rate),
        ("proposal", proposal_learning_rate),
        ("decoder", decoder_learning_rate),
        ("encoder_last", encoder_last_learning_rate),
        ("backbone_last", backbone_last_learning_rate),
    ):
        if value is not None:
            role_learning_rates[role] = value
    config = replace(
        config,
        run_name=(
            f"{run_name}-stage-{stage}"
            if trainable_profile is None
            else f"{run_name}-{profile}"
        ),
        wandb_project=wandb_project,
        wandb_group=wandb_group or run_name,
        wandb_mode=cast(
            "Literal['online', 'offline', 'disabled']",
            wandb_mode,
        ),
        wandb_log_interval=wandb_log_interval,
        wandb_log_checkpoints=wandb_log_checkpoints,
        save_named_best_checkpoints=save_named_best_checkpoints,
        max_validation_batches=max_validation_batches,
        validation_interval=validation_interval,
        render_object_confidence=render_object_confidence,
        render_person_confidence=render_person_confidence,
        render_robot_confidence=render_robot_confidence,
        render_field_confidence=render_field_confidence,
        render_keypoint_confidence=render_keypoint_confidence,
        render_max_detections=render_max_detections,
        dataset_audits=dataset_audits,
        dataset_fingerprints=dataset_fingerprints,
        batch_size_per_rank=batch_size,
        validation_batch_size=validation_batch_size,
        role_learning_rates=role_learning_rates,
        weight_decay=weight_decay,
        warmup_steps=warmup_steps,
        learning_rate_schedule=cast(
            "Literal['constant']",
            learning_rate_schedule,
        ),
        clip_max_norm=clip_max_norm,
        ema_decay=ema_decay,
        ema_warmups=ema_warmups,
        amp=amp,
        freeze_frozen_bn_stats=freeze_frozen_bn_stats,
        freeze_all_bn_stats=freeze_all_bn_stats,
        sync_batch_norm=sync_batch_norm,
        deterministic=deterministic,
        strict_deterministic=strict_deterministic,
        sdpa_backend=cast("Literal['auto', 'math']", sdpa_backend),
        ddp_find_unused_parameters=ddp_find_unused_parameters,
        seed=seed,
        allow_existing_output=allow_existing_output,
        provenance={
            "source_code_sha256": _source_sha256(),
            "head_config": str(asdict(model.head_config)),
            "loss_config": str(asdict(model.loss_config)),
            "train_image_size": str(train_image_size),
            "validation_image_size": str(validation_image_size),
            "field_augmentation_profile": field_augmentation_profile,
            "ddp_find_unused_parameters": str(ddp_find_unused_parameters),
            "strict_deterministic": str(strict_deterministic),
            "sdpa_backend": sdpa_backend,
            "object_data_sha256": _file_sha256(object_data),
            "person_data_sha256": _file_sha256(person_data),
            **(
                {"field_data_sha256": _file_sha256(field_data)}
                if field_data is not None
                else {}
            ),
            **(
                {
                    "model_checkpoint_sha256": _file_sha256(
                        Path(initialization_source)
                    )
                }
                if Path(initialization_source).is_file()
                else {"model_source": initialization_source}
            ),
            **(
                {"resume_checkpoint_sha256": _file_sha256(resume)}
                if resume is not None
                else {}
            ),
        },
    )
    trainer = MultiTaskTrainer(
        model,
        train_loaders,
        validation_loaders,
        config,
    )
    if resume is not None:
        trainer.resume(
            resume,
            mode=cast("Literal['exact', 'branch']", resume_mode),
        )
    trainer.fit()


if __name__ == "__main__":
    main()
