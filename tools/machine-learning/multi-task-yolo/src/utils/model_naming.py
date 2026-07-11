# ruff: noqa: TRY003

import re
from enum import Enum
from typing import Self

import click


class ModelNameError(Exception):
    def __init__(self, name: str) -> None:
        self.name = name
        super().__init__(f"Unknown model name: {name}")


class IncompatibleModelFamilyError(ValueError):
    def __init__(self, backbone: "ModelName", head: "ModelName") -> None:
        super().__init__(
            f"Backbone '{backbone}' and head '{head}' belong to different "
            "model families"
        )


class UnsupportedHydraCompositionError(ValueError):
    pass


class ModelFamily(Enum):
    YOLO = "yolo"
    DFINE = "dfine"

    def __str__(self) -> str:
        return self.value


class TaskType(Enum):
    OBJECT = "object"
    POSE = "pose"
    SEGMENTATION = "segmentation"

    def __str__(self) -> str:
        return self.value

    def output_specs(self) -> list[tuple[str, dict[int, str]]]:
        base = (f"{self.value}_output", {0: "batch_size", 1: "num_predictions"})
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
        match self.name:
            case str if str.startswith("yolo26m-pose"):
                return TaskType.POSE
            case str if str.startswith("yolo26m-seg"):
                return TaskType.SEGMENTATION
            case str if str.startswith("yolo26m"):
                return TaskType.OBJECT
            case str if re.fullmatch(r"dfine-s(?:~[A-Za-z0-9_-]+)?", str):
                return TaskType.OBJECT
            case _:
                raise ModelNameError(self.name)

    def family(self) -> ModelFamily:
        if self.name.startswith("yolo26m"):
            return ModelFamily.YOLO
        if re.fullmatch(r"dfine-s(?:~[A-Za-z0-9_-]+)?", self.name):
            return ModelFamily.DFINE
        raise ModelNameError(self.name)

    def variant(self) -> str:
        if self.family() == ModelFamily.YOLO:
            return "yolo26m"
        return "dfine-s"

    def is_finetuned_model(self) -> bool:
        return "~" in self.name


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
        self._validate()

    def _validate(self) -> None:
        if not self.heads:
            raise UnsupportedHydraCompositionError(
                "A Hydra model must contain at least one head"
            )

        backbone_family = self.backbone.family()
        for head in self.heads:
            if head.family() != backbone_family:
                raise IncompatibleModelFamilyError(self.backbone, head)

        if backbone_family != ModelFamily.DFINE:
            return

        if self.number_of_frozen_modules != 1:
            raise UnsupportedHydraCompositionError(
                "D-FINE Hydra models currently support only f1, which shares "
                "the HGNetv2 backbone"
            )
        if len(self.heads) != 1 or self.heads[0].task_type() != TaskType.OBJECT:
            raise UnsupportedHydraCompositionError(
                "D-FINE currently supports exactly one D-FINE detection head"
            )
        if self.backbone.variant() != self.heads[0].variant():
            raise UnsupportedHydraCompositionError(
                "D-FINE backbone and head variants must match"
            )

    @classmethod
    def parse(cls, model_string: str) -> "HydraModelName":
        backbone_and_frozen_layers, *heads = model_string.split("+")
        if not heads:
            raise ValueError("Hydra model name must include at least one head")
        heads = [ModelName(head) for head in heads]
        parts = backbone_and_frozen_layers.split("=")
        if len(parts) != 2:
            raise ValueError(
                "Hydra model name must use BACKBONE=fN+HEAD syntax"
            )
        backbone, frozen = parts
        if not re.fullmatch(r"f\d+", frozen):
            raise ValueError("Hydra split must use fN syntax, for example f11")

        return cls(backbone, heads, int(frozen[1:]))

    def __str__(self) -> str:
        heads = "+".join(str(head) for head in self.heads)
        return f"{self.backbone}=f{self.number_of_frozen_modules}+{heads}"

    def integrated_model_name(self, model_name: ModelName) -> str:
        return (
            f"{self.backbone}=f{self.number_of_frozen_modules}+{model_name!s}"
        )

    def family(self) -> ModelFamily:
        return self.backbone.family()


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
