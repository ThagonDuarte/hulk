import torch
from torch import Tensor


def weighting_function(reg_max: int, up: Tensor, reg_scale: Tensor) -> Tensor:
    scale = reg_scale.reshape(-1)[0].abs()
    upper_bound = up[0].abs() * scale
    outer_bound = upper_bound * 2
    step = (upper_bound + 1) ** (2 / (reg_max - 2))
    left = [-(step**index) + 1 for index in range(reg_max // 2 - 1, 0, -1)]
    right = [step**index - 1 for index in range(1, reg_max // 2)]
    values = [-outer_bound, *left, torch.zeros_like(up[0]), *right, outer_bound]
    return torch.stack(values)


def translate_ground_truth(
    ground_truth: Tensor,
    reg_max: int,
    reg_scale: Tensor,
    up: Tensor,
) -> tuple[Tensor, Tensor, Tensor]:
    ground_truth = ground_truth.reshape(-1)
    function_values = weighting_function(reg_max, up, reg_scale)
    differences = function_values.unsqueeze(0) - ground_truth.unsqueeze(1)
    indices = (differences <= 0).sum(dim=1).sub(1).float()

    weight_right = torch.zeros_like(indices)
    weight_left = torch.zeros_like(indices)
    valid = (indices >= 0) & (indices < reg_max)
    valid_indices = indices[valid].long()
    left_diff = (ground_truth[valid] - function_values[valid_indices]).abs()
    right_diff = (
        function_values[valid_indices + 1] - ground_truth[valid]
    ).abs()
    weight_right[valid] = left_diff / (left_diff + right_diff)
    weight_left[valid] = 1.0 - weight_right[valid]

    below = indices < 0
    weight_left[below] = 1.0
    indices[below] = 0.0
    above = indices >= reg_max
    weight_right[above] = 1.0
    indices[above] = reg_max - 0.1
    return indices, weight_right, weight_left


def bbox_to_distance(
    points: Tensor,
    boxes_xyxy: Tensor,
    reg_max: int,
    reg_scale: Tensor,
    up: Tensor,
) -> tuple[Tensor, Tensor, Tensor]:
    scale = reg_scale.abs()
    horizontal = points[..., 2] / scale + 1e-16
    vertical = points[..., 3] / scale + 1e-16
    distances = torch.stack(
        (
            (points[:, 0] - boxes_xyxy[:, 0]) / horizontal - 0.5 * scale,
            (points[:, 1] - boxes_xyxy[:, 1]) / vertical - 0.5 * scale,
            (boxes_xyxy[:, 2] - points[:, 0]) / horizontal - 0.5 * scale,
            (boxes_xyxy[:, 3] - points[:, 1]) / vertical - 0.5 * scale,
        ),
        dim=-1,
    )
    targets, weight_right, weight_left = translate_ground_truth(
        distances,
        reg_max,
        reg_scale,
        up,
    )
    return (
        targets.clamp(min=0, max=reg_max - 0.1).reshape(-1).detach(),
        weight_right.detach(),
        weight_left.detach(),
    )
