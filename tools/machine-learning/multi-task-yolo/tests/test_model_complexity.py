import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import torch
import torch.nn as nn
from click.testing import CliRunner

from utils.model_complexity import (
    checkpoint_kind,
    main,
    profile_checkpoint,
)


class ModelComplexityTests(unittest.TestCase):
    def test_native_multitask_checkpoint_uses_rectangular_flops(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "best.pt"
            torch.save(
                {
                    "architecture": "dfine-multitask",
                    "format_version": 3,
                },
                checkpoint,
            )
            model = nn.Sequential(nn.Linear(2, 3))
            with (
                patch(
                    "utils.model_complexity."
                    "DFINEMultiTaskModel.from_checkpoint",
                    return_value=model,
                ) as native_loader,
                patch("utils.model_complexity.YOLO") as yolo_loader,
                patch(
                    "utils.model_complexity.get_flops",
                    return_value=12.5,
                ) as get_flops,
            ):
                result = profile_checkpoint(
                    checkpoint,
                    imgsz=(448, 544),
                    device="cpu",
                )

        native_loader.assert_called_once_with(checkpoint)
        yolo_loader.assert_not_called()
        get_flops.assert_called_once_with(model, imgsz=[448, 544])
        self.assertEqual(result.input_size, (448, 544))
        self.assertEqual(result.gflops, 12.5)
        self.assertEqual(result.gmacs, 6.25)

    def test_yolo_checkpoint_branch_is_selected_from_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            checkpoint = Path(directory) / "model.pt"
            torch.save({"architecture": "not-dfine"}, checkpoint)
            model = nn.Sequential(nn.Linear(2, 3))
            with (
                patch(
                    "utils.model_complexity.YOLO",
                    return_value=SimpleNamespace(model=model),
                ) as yolo_loader,
                patch(
                    "utils.model_complexity.get_flops",
                    return_value=2.0,
                ),
            ):
                result = profile_checkpoint(
                    checkpoint,
                    imgsz=640,
                    device="cpu",
                )
                self.assertEqual(checkpoint_kind(checkpoint), "yolo")

        yolo_loader.assert_called_once_with(checkpoint)
        self.assertEqual(result.input_size, 640)

    def test_cli_exposes_rectangular_overrides(self) -> None:
        result = CliRunner().invoke(main, ["--help"])

        self.assertEqual(result.exit_code, 0, result.output)
        self.assertIn("--height INTEGER", result.output)
        self.assertIn("--width INTEGER", result.output)


if __name__ == "__main__":
    unittest.main()
