import logging
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, cast

import torch
import torch.nn as nn
from ultralytics.models.yolo.model import YOLO
from ultralytics.nn.tasks import DetectionModel

from utils.model_naming import TaskType

logger = logging.getLogger(__name__)

ClassNames = Mapping[int, str] | Sequence[str] | None


def normalize_class_names(names: ClassNames) -> dict[int, str]:
    """Return class names as an index-keyed dictionary."""
    if names is None:
        return {}
    if isinstance(names, Mapping):
        return {int(cast(Any, k)): str(v) for k, v in names.items()}
    return {i: str(v) for i, v in enumerate(names)}


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


def _resize_detect_class_layers(
    head: nn.Module,
    old_nc: int,
    old_names: dict[int, str],
    new_names: dict[int, str],
) -> None:
    """Resize Detect/Segment/Pose class convs in-place."""
    resized = False
    stride = torch.as_tensor(getattr(head, "stride", []), dtype=torch.float32)
    for attr in ("cv3", "one2one_cv3"):
        module_list = getattr(head, attr, None)
        if isinstance(module_list, nn.ModuleList):
            _resize_class_module_list(
                module_list, old_names, new_names, stride=stride
            )
            resized = True

    # YOLO26 segmentation heads also produce semantic segmentation logits.
    proto = getattr(head, "proto", None)
    semseg = getattr(proto, "semseg", None)
    if isinstance(semseg, nn.Sequential) and isinstance(
        semseg[-1], nn.Conv2d
    ):
        semseg[-1] = _new_class_conv(
            semseg[-1], old_names, new_names, stride=None
        )

    if not resized:
        raise TypeError(  # noqa: TRY003
            f"Cannot resize class layers for head type {type(head)!r}"
        )

    new_nc = len(new_names)
    head_any = cast(Any, head)
    head_any.nc = new_nc
    if hasattr(head, "no"):
        reg_max = int(getattr(head, "reg_max", 1))
        head_any.no = new_nc + reg_max * 4
    logger.info(
        "Resized class predictors from nc=%d to nc=%d", old_nc, new_nc
    )


def _resize_class_module_list(
    module_list: nn.ModuleList,
    old_names: dict[int, str],
    new_names: dict[int, str],
    *,
    stride: torch.Tensor,
) -> None:
    for i, branch in enumerate(module_list):
        if not isinstance(branch, nn.Sequential):
            raise TypeError(  # noqa: TRY003
                f"Class branch {i} must be nn.Sequential, got {type(branch)!r}"
            )
        final = branch[-1]
        if not isinstance(final, nn.Conv2d):
            raise TypeError(  # noqa: TRY003
                f"Class branch {i} must end in Conv2d, got {type(final)!r}"
            )
        branch[-1] = _new_class_conv(
            final, old_names, new_names, stride=_stride_at(stride, i)
        )


def _new_class_conv(
    old: nn.Conv2d,
    old_names: dict[int, str],
    new_names: dict[int, str],
    *,
    stride: float | None,
) -> nn.Conv2d:
    new_nc = len(new_names)
    new = nn.Conv2d(
        old.in_channels,
        new_nc,
        cast(Any, old.kernel_size),
        cast(Any, old.stride),
        cast(Any, old.padding),
        cast(Any, old.dilation),
        old.groups,
        old.bias is not None,
        old.padding_mode,
        device=old.weight.device,
        dtype=old.weight.dtype,
    )
    old_by_name = {_class_name_key(v): k for k, v in old_names.items()}
    with torch.no_grad():
        if new.bias is not None and stride is not None:
            new.bias.fill_(_class_prior_bias(new_nc, stride))
        for new_idx in range(new_nc):
            new_name = new_names.get(new_idx, str(new_idx))
            old_idx = old_by_name.get(_class_name_key(new_name))
            if old_idx is None or old_idx >= old.out_channels:
                continue
            new.weight[new_idx].copy_(old.weight[old_idx])
            if new.bias is not None and old.bias is not None:
                new.bias[new_idx].copy_(old.bias[old_idx])
    return new


def _class_name_key(name: str) -> str:
    return name.strip().casefold()


def _class_prior_bias(nc: int, stride: float) -> float:
    return math.log(5 / nc / (640 / max(stride, 1e-9)) ** 2)


def _stride_at(stride: torch.Tensor, index: int) -> float | None:
    if stride.numel() <= index:
        return None
    return float(stride.flatten()[index].item())


class EmptyClassNamesError(ValueError):
    def __init__(self) -> None:
        super().__init__("class_names must contain at least one class")


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
        task_dict: Mapping[TaskType, Path],
        number_of_frozen_modules: int | None = None,
    ) -> None:
        super().__init__()

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
            backbone_root, number_of_frozen_modules
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
                task_root, number_of_frozen_modules
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

    def adapt_head_classes(
        self, task: TaskType, class_names: ClassNames
    ) -> None:
        """Resize a task head's class predictors to match a dataset.

        Ultralytics' trainer rebuilds a model with ``nc=data["nc"]`` before
        loading weights, which skips mismatched classifier tensors. The custom
        Hydra loop loads checkpoint modules directly, so we perform the small
        equivalent surgery here: replace only the final class-output convs and
        keep all shared/box/mask/keypoint weights.
        """
        task_key = str(task)
        names = normalize_class_names(class_names)
        if not names:
            raise EmptyClassNamesError
        if task_key not in self.heads:
            raise MissingHydraHeadError(task_key)

        head = cast(nn.ModuleList, self.heads[task_key])[-1]
        old_names = normalize_class_names(self.head_class_names.get(task_key))
        old_nc = int(getattr(head, "nc", len(old_names) or 0))
        new_nc = len(names)

        self.head_class_names[task_key] = names
        if old_nc == new_nc:
            head_any = cast(Any, head)
            head_any.nc = new_nc
            if hasattr(head, "no"):
                reg_max = int(getattr(head, "reg_max", 1))
                head_any.no = new_nc + reg_max * 4
            return

        logger.info(
            "Adapting %s head from nc=%d to dataset nc=%d",
            task,
            old_nc,
            new_nc,
        )
        _resize_detect_class_layers(head, old_nc, old_names, names)

    def run_backbone(
        self, x: torch.Tensor
    ) -> tuple[torch.Tensor | list[torch.Tensor], list[torch.Tensor | None]]:
        """Run the shared backbone and return (final_activations, saved_list).

        The saved list mirrors Ultralytics' `y` cache pattern: each entry is
        either a tensor (if its layer index appears in `self.save_backbone`)
        or `None`. Used by `run_head` to resolve cross-layer connections that
        reach back into the backbone.
        """
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

        return backbone_activations, y_backbone

    def run_head(
        self,
        head_name: str,
        backbone_activations: torch.Tensor | list[torch.Tensor],
        y_backbone: list[torch.Tensor | None],
    ) -> Any:
        """Run a single task head and return its raw, non-flattened output.

        The returned value is whatever the head's final module emits — a
        tensor, a tuple of tensors, or a structured object (e.g. the dict
        returned by end2end heads that `E2ELoss.parse_output` expects).
        """
        if head_name not in self.heads:
            raise MissingHydraHeadError(head_name)
        head = cast(nn.ModuleList, self.heads[head_name])
        y_head = list(y_backbone)
        head_activations: Any = backbone_activations

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

        return head_activations

    def forward(self, x: torch.Tensor) -> dict[str, Any]:
        backbone_activations, y_backbone = self.run_backbone(x)
        outputs: dict[str, Any] = {}

        for head_name in self.heads:
            head_activations = self.run_head(
                head_name, backbone_activations, y_backbone
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
