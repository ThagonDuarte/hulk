from enum import Enum
from pathlib import Path, PurePath
from typing import Self

import click

YOLO26_SIZES = ("n", "s", "m", "l", "x")
FIELD_FEATURE_POSE_SUFFIX = "-pose-field-features"


def _yolo26_size_and_variant_suffix(stem: str) -> tuple[str, str] | None:
    for size in YOLO26_SIZES:
        prefix = f"yolo26{size}"
        if stem == prefix:
            return size, ""
        if stem.startswith(f"{prefix}-"):
            return size, stem.removeprefix(prefix)
    return None


def _yolo26_variant_suffix(stem: str) -> str | None:
    parsed = _yolo26_size_and_variant_suffix(stem)
    return None if parsed is None else parsed[1]


def yolo26_model_size(model_name: str | PurePath) -> str | None:
    parsed = _yolo26_size_and_variant_suffix(PurePath(model_name).stem)
    return None if parsed is None else parsed[0]


def is_field_feature_pose_model_name(model_name: str | PurePath) -> bool:
    variant_suffix = _yolo26_variant_suffix(PurePath(model_name).stem)
    return bool(
        variant_suffix is not None
        and variant_suffix.startswith(FIELD_FEATURE_POSE_SUFFIX)
    )


def field_feature_pose_pretrained_model_name(
    model_name: str | PurePath,
) -> str | None:
    size = yolo26_model_size(model_name)
    if size is None or not is_field_feature_pose_model_name(model_name):
        return None
    return f"yolo26{size}-pose"


def _field_feature_pose_size_agnostic_yaml(model_path: Path) -> Path | None:
    variant_suffix = _yolo26_variant_suffix(model_path.stem)
    if variant_suffix is None or not variant_suffix.startswith(
        FIELD_FEATURE_POSE_SUFFIX
    ):
        return None

    return model_path.with_suffix(".yaml").with_name(
        f"yolo26{variant_suffix}.yaml"
    )


def _model_path_candidates(model_path: Path) -> tuple[Path, ...]:
    if model_path.suffix:
        return (model_path,)
    return (
        model_path,
        model_path.with_suffix(".pt"),
        model_path.with_suffix(".yaml"),
    )


def resolve_model_path(model_name: str, assets_dir: Path) -> str | Path:
    model_path = Path(model_name)
    if model_path.is_absolute() or model_path.parent != Path("."):
        search_path = model_path
    else:
        search_path = assets_dir / model_path

    candidates = _model_path_candidates(search_path)

    for candidate in candidates:
        if candidate.exists():
            return candidate

    size_agnostic_yaml = _field_feature_pose_size_agnostic_yaml(search_path)
    if size_agnostic_yaml is not None and size_agnostic_yaml.exists():
        return search_path.with_suffix(".yaml")

    if model_path.suffix:
        return model_name
    return str(model_path.with_suffix(".pt"))


class ModelNameError(Exception):
    def __init__(self, name: str) -> None:
        self.name = name
        super().__init__(f"Unknown model name: {name}")


class TaskType(Enum):
    OBJECT = "object"
    POSE = "pose"
    SEGMENTATION = "segmentation"

    def __str__(self) -> str:
        return self.value

    def output_specs(self) -> list[tuple[str, dict[int, str]]]:
        base = (f"{self.value}_output", {0: "batch_size", 2: "num_predictions"})
        if self == TaskType.SEGMENTATION:
            return [base, (f"{self.value}_proto", {0: "batch_size"})]
        return [base]

    def output_names(self) -> list[str]:
        return [name for name, _ in self.output_specs()]


class ModelName:
    name: str

    def __init__(self, name: str | Self) -> None:
        if isinstance(name, ModelName):
            self.name = name.name
        else:
            self.name = name

    def __str__(self) -> str:
        return f"{self.name}"

    def task_type(self) -> TaskType:
        stem = PurePath(self.name).stem
        variant_suffix = _yolo26_variant_suffix(stem)
        if variant_suffix is None:
            raise ModelNameError(self.name)

        if variant_suffix.startswith("-pose"):
            return TaskType.POSE
        if variant_suffix.startswith("-seg"):
            return TaskType.SEGMENTATION
        return TaskType.OBJECT

    def is_finetuned_model(self) -> bool:
        return "~" in self.name

    def is_field_feature_pose_model(self) -> bool:
        return is_field_feature_pose_model_name(self.name)


class HydraModelName:
    backbone: ModelName
    heads: list[ModelName]
    number_of_frozen_modules: int

    def __init__(
        self,
        backbone: ModelName | str,
        heads: list[ModelName] | list[str],
        number_of_frozen_modules: int,
    ) -> None:
        self.backbone = ModelName(backbone)
        self.heads = [ModelName(head) for head in heads]
        self.number_of_frozen_modules = number_of_frozen_modules

    @classmethod
    def parse(cls, model_string: str) -> "HydraModelName":
        backbone_and_frozen_layers, *heads = model_string.split("+")
        heads = [ModelName(head) for head in heads]
        backbone, number_of_frozen_modules = backbone_and_frozen_layers.split(
            "="
        )

        return cls(backbone, heads, int(number_of_frozen_modules.strip("f")))

    def __str__(self) -> str:
        heads = "+".join(str(head) for head in self.heads)
        return f"{self.backbone}=f{self.number_of_frozen_modules}+{heads}"

    def integrated_model_name(self, model_name: ModelName) -> str | None:
        return (
            f"{self.backbone}=f{self.number_of_frozen_modules}+{model_name!s}"
        )


class HydraModelNameParam(click.ParamType):
    name = "hydra-model-name"

    def convert(
        self,
        value: str,
        param: click.Parameter | None,
        ctx: click.Context | None,
    ) -> HydraModelName:
        try:
            return HydraModelName.parse(value)
        except ValueError as exc:
            self.fail(str(exc), param, ctx)


HYDRA_MODEL_NAME_TYPE = HydraModelNameParam()
