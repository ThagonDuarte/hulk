import logging
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, cast

import torch
import torch.nn as nn
from ultralytics.models.yolo.model import YOLO
from ultralytics.nn.tasks import DetectionModel

from ultralytics_dfine.nn import DFINEDetectionModel, DFINEMultiTaskModel
from utils.model_naming import ModelFamily, TaskType

logger = logging.getLogger(__name__)

ClassNames = Mapping[int, str] | Sequence[str] | None


def get_backbone_length(yaml_config: dict) -> int:
    """Returns the index of the last layer of the backbone."""
    return len(yaml_config.get("backbone", []))


def get_backbone(
    model: DetectionModel, number_of_frozen_modules: int | None = None
) -> nn.ModuleList:
    """Extracts the backbone as an nn.ModuleList dynamically."""
    if number_of_frozen_modules is not None:
        split_idx = number_of_frozen_modules
    else:
        split_idx = get_backbone_length(cast(dict[str, Any], model.yaml))
    return nn.ModuleList(list(model.model.children())[:split_idx])


def get_head(
    model: DetectionModel, number_of_frozen_modules: int | None = None
) -> nn.ModuleList:
    """Extracts the neck + head head dynamically."""
    if number_of_frozen_modules is not None:
        split_idx = number_of_frozen_modules
    else:
        split_idx = get_backbone_length(cast(dict[str, Any], model.yaml))
    return nn.ModuleList(list(model.model.children())[split_idx:])


def set_backbone(
    model: DetectionModel,
    backbone: nn.ModuleList,
    number_of_frozen_modules: int | None = None,
) -> None:
    """Replaces the backbone modules of a model."""
    if number_of_frozen_modules is not None:
        split_idx = number_of_frozen_modules
    else:
        split_idx = get_backbone_length(cast(dict[str, Any], model.yaml))
    head = list(model.model.children())

    nodes = list(backbone) + head[split_idx:]
    model.model = nn.Sequential(*nodes)


class MissingHydraHeadError(KeyError):
    def __init__(self, head_name: str) -> None:
        super().__init__(f"Hydra head '{head_name}' was not found")


class UnsupportedHydraHeadError(ValueError):
    def __init__(self, head_name: str) -> None:
        super().__init__(f"Hydra head '{head_name}' is not mapped to a task")


class InvalidHydraOutputError(TypeError):
    def __init__(self, head_name: str) -> None:
        super().__init__(
            f"Hydra output did not contain expected head '{head_name}'"
        )


class Hydra(nn.Module):
    def __init__(
        self,
        backbone_path: str,
        task_dict: dict[TaskType, Path],
        number_of_frozen_modules: int | None = None,
        family: ModelFamily = ModelFamily.YOLO,
    ) -> None:
        super().__init__()

        self.family = family
        self.deployment_output_names: list[str] = []
        if family == ModelFamily.DFINE:
            self._initialize_dfine(
                backbone_path,
                task_dict,
                number_of_frozen_modules,
            )
            return

        logger.info("Loading backbone from: %s", backbone_path)
        backbone_yolo = YOLO(backbone_path)
        backbone_root = cast(DetectionModel, backbone_yolo.model)

        self.backbone_length = (
            number_of_frozen_modules
            if number_of_frozen_modules is not None
            else get_backbone_length(cast(dict, backbone_root.yaml))
        )

        backbone_model_name = backbone_yolo.model_name or "unknown"
        self.backbone_name = Path(backbone_model_name).stem
        self.shared_backbone = get_backbone(
            backbone_root,
            number_of_frozen_modules,
        )
        self.save_backbone = cast(list[int], backbone_root.save)

        self.heads = nn.ModuleDict()
        self.branch_saves: dict[str, list[int]] = {}
        self.head_class_names: dict[str, Any] = {}
        self.head_model_names: dict[str, Any] = {}
        self.head_strides: dict[str, torch.Tensor] = {}
        self.head_end2end: dict[str, bool] = {}
        self.head_kpt_shapes: dict[str, tuple[int, int] | None] = {}

        for task_type, head_model_path in task_dict.items():
            task_type = str(task_type)
            logger.info(
                "Extracting %s head from: %s", task_type, head_model_path
            )
            task_yolo = YOLO(head_model_path)
            task_root = cast(DetectionModel, task_yolo.model)
            task_head = task_root.model[-1]

            self.heads[task_type] = get_head(
                task_root,
                number_of_frozen_modules,
            )
            self.branch_saves[task_type] = cast(list[int], task_root.save)
            self.head_class_names[task_type] = getattr(task_root, "names", {})
            task_model_name = task_yolo.model_name or "unknown"
            self.head_model_names[task_type] = Path(task_model_name).stem
            stride = getattr(task_head, "stride", torch.tensor([8, 16, 32]))
            self.head_strides[task_type] = torch.as_tensor(stride)
            self.head_end2end[task_type] = bool(
                getattr(
                    task_head,
                    "end2end",
                    getattr(task_root, "end2end", False),
                )
            )

            raw_kpt_shape = getattr(task_head, "kpt_shape", None)
            if (
                isinstance(raw_kpt_shape, (list, tuple))
                and len(raw_kpt_shape) >= 2
            ):
                self.head_kpt_shapes[task_type] = (
                    int(raw_kpt_shape[0]),
                    int(raw_kpt_shape[1]),
                )
            else:
                self.head_kpt_shapes[task_type] = None
        self.deployment_output_names = [
            name for task_type in task_dict for name in task_type.output_names()
        ]

    def _initialize_dfine(
        self,
        backbone_path: str,
        task_dict: dict[TaskType, Path],
        number_of_frozen_modules: int | None,
    ) -> None:
        if number_of_frozen_modules != 1:
            raise ValueError(  # noqa: TRY003
                "D-FINE Hydra models require the f1 split"
            )
        if set(task_dict) != {TaskType.OBJECT}:
            raise ValueError(  # noqa: TRY003
                "D-FINE Hydra supports one object-detection head"
            )
        head_path = task_dict[TaskType.OBJECT]
        checkpoint = torch.load(
            head_path,
            map_location="cpu",
            weights_only=True,
        )
        if (
            isinstance(checkpoint, dict)
            and checkpoint.get("format_version") == 2
            and checkpoint.get("architecture") == "dfine-multitask"
        ):
            model = DFINEMultiTaskModel.from_checkpoint(head_path)
            if backbone_path != "dfine-s":
                raise ValueError(  # noqa: TRY003
                    "Multi-task D-FINE checkpoints own their shared backbone"
                )
            self.dfine_multitask = model
            self.backbone_length = 1
            self.backbone_name = "dfine-s"
            self.head_class_names = {"object": model.detector.names}
            self.head_model_names = {"object": head_path.stem}
            self.head_strides = {"object": model.detector.stride}
            self.head_end2end = {"object": True}
            self.head_kpt_shapes = {
                "person_pose": (17, 3),
                "robot_pose": (14, 3),
            }
            self.deployment_output_names = [
                "object_output",
                "person_pose_output",
                "robot_pose_output",
                "field_feature_output",
            ]
            return
        detector = DFINEDetectionModel.from_checkpoint(head_path)
        source_path = Path(backbone_path)
        if source_path.is_file():
            backbone_model = DFINEDetectionModel.from_checkpoint(source_path)
            detector.replace_backbone(backbone_model.backbone)
        elif backbone_path == "dfine-s":
            # The only current D-FINE head is a complete trained model. Its
            # backbone is the shared Hydra backbone until multiple heads exist.
            pass
        else:
            raise ValueError(  # noqa: TRY003
                f"Unsupported D-FINE backbone: {backbone_path}"
            )
        self.dfine_detector = detector
        self.backbone_length = 1
        self.backbone_name = source_path.stem
        self.head_class_names = {str(TaskType.OBJECT): detector.names}
        self.head_model_names = {str(TaskType.OBJECT): head_path.stem}
        self.head_strides = {str(TaskType.OBJECT): detector.stride}
        self.head_end2end = {str(TaskType.OBJECT): True}
        self.head_kpt_shapes = {str(TaskType.OBJECT): None}
        self.deployment_output_names = ["object_output"]

    def forward(self, x: torch.Tensor) -> dict[str, Any]:  # noqa: C901
        if self.family == ModelFamily.DFINE:
            if hasattr(self, "dfine_multitask"):
                return self.dfine_multitask.forward_deploy(x)
            raw = self.dfine_detector.forward_raw(x)
            normalized = self.dfine_detector.postprocessor(raw)
            sizes = torch.tensor(
                [[x.shape[-2], x.shape[-1]]],
                device=x.device,
            ).expand(x.shape[0], -1)
            return {
                TaskType.OBJECT.output_names()[0]: (
                    self.dfine_detector.postprocessor.to_pixel_xyxy(
                        normalized,
                        sizes,
                    )
                )
            }

        y_backbone: list[torch.Tensor | None] = []
        backbone_activations: Any = x

        for i, m in enumerate(self.shared_backbone):
            from_index = cast(Any, m.f)
            if from_index != -1:
                backbone_activations = (
                    y_backbone[from_index]
                    if isinstance(from_index, int)
                    else [
                        backbone_activations if j == -1 else y_backbone[j]
                        for j in cast(list[int], from_index)
                    ]
                )
            backbone_activations = m(backbone_activations)
            y_backbone.append(
                backbone_activations if i in self.save_backbone else None
            )

        outputs: dict[str, Any] = {}

        for head_name, head_module in self.heads.items():
            head = cast(nn.ModuleList, head_module)
            y_head = list(y_backbone)
            head_activations: list[torch.Tensor] | torch.Tensor = (
                backbone_activations
            )

            for i, m in enumerate(head):
                module_index = i + self.backbone_length

                from_index = cast(Any, m.f)
                if from_index != -1:
                    head_activations = (
                        cast(torch.Tensor, y_head[from_index])
                        if isinstance(from_index, int)
                        else [
                            cast(torch.Tensor, head_activations)
                            if j == -1
                            else cast(torch.Tensor, y_head[j])
                            for j in cast(list[int], from_index)
                        ]
                    )

                head_activations = m(head_activations)

                y_head.append(
                    cast(torch.Tensor, head_activations)
                    if module_index in self.branch_saves[head_name]
                    else None
                )

            task_output_names = TaskType(head_name).output_names()
            if isinstance(head_activations, torch.Tensor):
                outputs[task_output_names[0]] = head_activations
            elif isinstance(head_activations, tuple) and all(
                isinstance(t, torch.Tensor) for t in head_activations
            ):
                for key, tensor in zip(
                    task_output_names, head_activations, strict=False
                ):
                    outputs[key] = tensor
            else:
                raise TypeError(  # noqa: TRY003
                    f"Head '{head_name}' output must be a tensor or tuple of"
                    f" tensors, got {type(head_activations)}"
                )

        return outputs
