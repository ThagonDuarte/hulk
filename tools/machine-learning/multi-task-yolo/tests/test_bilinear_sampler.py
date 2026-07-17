import io
import unittest
from unittest.mock import patch

import numpy as np
import onnx
import onnxruntime
import torch
import torch.nn as nn
import torch.nn.functional as functional

from ultralytics_dfine.config import FieldHeadConfig
from ultralytics_dfine.nn.multitask import (
    FieldFeatureHead,
    _bilinear_sample_border,
    _sample_encoder_feature,
)


class _SamplerModule(nn.Module):
    def forward(
        self, feature: torch.Tensor, grid: torch.Tensor
    ) -> torch.Tensor:
        return _sample_encoder_feature(feature, grid)


def _reference_sample(
    feature: torch.Tensor,
    grid: torch.Tensor,
) -> torch.Tensor:
    return functional.grid_sample(
        feature,
        grid,
        mode="bilinear",
        padding_mode="border",
        align_corners=False,
    )


class BilinearSamplerParityTests(unittest.TestCase):
    @staticmethod
    def _compare_forward_and_backward(
        feature: torch.Tensor,
        grid: torch.Tensor,
    ) -> None:
        reference_feature = feature.detach().clone().requires_grad_()
        manual_feature = feature.detach().clone().requires_grad_()
        reference_grid = grid.detach().clone().requires_grad_()
        manual_grid = grid.detach().clone().requires_grad_()
        reference = _reference_sample(reference_feature, reference_grid)
        manual = _bilinear_sample_border(manual_feature, manual_grid)
        torch.testing.assert_close(reference, manual, rtol=1e-12, atol=1e-12)

        generator = torch.Generator().manual_seed(191)
        output_gradient = torch.randn(
            reference.shape,
            dtype=reference.dtype,
            generator=generator,
        )
        reference_gradients = torch.autograd.grad(
            reference,
            (reference_feature, reference_grid),
            output_gradient,
        )
        manual_gradients = torch.autograd.grad(
            manual,
            (manual_feature, manual_grid),
            output_gradient,
        )
        for reference_gradient, manual_gradient in zip(
            reference_gradients,
            manual_gradients,
            strict=True,
        ):
            torch.testing.assert_close(
                reference_gradient,
                manual_gradient,
                rtol=1e-12,
                atol=1e-12,
            )

    def test_matches_at_interior_border_and_out_of_range_points(self) -> None:
        height = 3
        width = 5
        left = -1 + 1 / width
        right = 1 - 1 / width
        top = -1 + 1 / height
        bottom = 1 - 1 / height
        grid = torch.tensor(
            [
                [-3.0, -2.0],
                [-1.0, -1.0],
                [left, top],
                [left, 0.17],
                [right, -0.21],
                [right, bottom],
                [1.0, 1.0],
                [2.0, 4.0],
                [-0.41, 0.27],
                [0.11, -0.53],
            ],
            dtype=torch.float64,
        ).view(1, -1, 1, 2)
        grid = grid.expand(2, -1, -1, -1).clone()
        feature = torch.arange(
            2 * 3 * height * width,
            dtype=torch.float64,
        ).view(2, 3, height, width)

        self._compare_forward_and_backward(feature, grid)

    def test_normal_execution_keeps_native_grid_sample(self) -> None:
        feature = torch.randn(1, 2, 3, 5)
        grid = torch.rand(1, 4, 2, 2) * 2 - 1
        target = "ultralytics_dfine.nn.multitask.functional.grid_sample"

        with patch(target, wraps=functional.grid_sample) as native:
            actual = _sample_encoder_feature(feature, grid)

        native.assert_called_once_with(
            feature,
            grid,
            mode="bilinear",
            padding_mode="border",
            align_corners=False,
        )
        torch.testing.assert_close(actual, _reference_sample(feature, grid))

    def test_matches_random_rectangular_and_singleton_feature_maps(
        self,
    ) -> None:
        generator = torch.Generator().manual_seed(37)
        for height, width in ((3, 7), (6, 2), (1, 5), (4, 1)):
            with self.subTest(height=height, width=width):
                feature = torch.randn(
                    (2, 4, height, width),
                    dtype=torch.float64,
                    generator=generator,
                )
                grid = (
                    torch.randn(
                        (2, 5, 3, 2),
                        dtype=torch.float64,
                        generator=generator,
                    )
                    * 1.7
                )
                self._compare_forward_and_backward(feature, grid)

    def test_spatial_head_checkpoint_keys_are_unchanged_by_sampler(
        self,
    ) -> None:
        field_config = FieldHeadConfig(
            variant="spatial_refine",
            refinement_dim=8,
        )
        module = FieldFeatureHead(
            16,
            attention_heads=4,
            config=field_config,
        )
        clone = FieldFeatureHead(
            16,
            attention_heads=4,
            config=field_config,
        )
        state = module.state_dict()
        self.assertEqual(tuple(state), tuple(clone.state_dict()))
        self.assertFalse(
            any("sampler" in name for name, _ in module.named_modules())
        )
        self.assertTrue(state)
        clone.load_state_dict(state, strict=True)


class BilinearSamplerOnnxTests(unittest.TestCase):
    def test_dynamic_onnx_uses_basic_ops_without_grid_sample(self) -> None:
        module = _SamplerModule().eval()
        feature = torch.randn(1, 2, 3, 5)
        grid = torch.rand(1, 2, 4, 2) * 3 - 1.5
        destination = io.BytesIO()

        torch.onnx.export(
            module,
            (feature, grid),
            destination,
            input_names=["feature", "grid"],
            output_names=["sampled"],
            dynamic_axes={
                "feature": {0: "batch", 2: "height", 3: "width"},
                "grid": {0: "batch", 1: "output_height", 2: "output_width"},
                "sampled": {
                    0: "batch",
                    2: "output_height",
                    3: "output_width",
                },
            },
            opset_version=17,
            external_data=False,
            dynamo=False,
        )
        model_bytes = destination.getvalue()
        model = onnx.load_from_string(model_bytes)
        operators = {node.op_type for node in model.graph.node}

        self.assertNotIn("GridSample", operators)
        self.assertIn("Floor", operators)
        self.assertIn("Clip", operators)
        self.assertIn("GatherElements", operators)

        runtime_feature = torch.randn(2, 2, 4, 7)
        runtime_grid = torch.rand(2, 3, 2, 2) * 4 - 2
        expected = module(runtime_feature, runtime_grid).detach().numpy()
        session = onnxruntime.InferenceSession(
            model_bytes,
            providers=["CPUExecutionProvider"],
        )
        actual = session.run(
            None,
            {
                "feature": runtime_feature.numpy(),
                "grid": runtime_grid.numpy(),
            },
        )[0]
        np.testing.assert_allclose(actual, expected, rtol=1e-6, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
