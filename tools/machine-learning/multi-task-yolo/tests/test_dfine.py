import tempfile
import unittest
from pathlib import Path

import torch
import yaml
from PIL import Image

from ultralytics_dfine.config import DFINEArchitectureConfig, build_manifest
from ultralytics_dfine.data import DFINEDataset
from ultralytics_dfine.loss import DFINECriterion
from ultralytics_dfine.nn import DFINEPostProcessorAdapter
from utils.model_naming import (
    HydraModelName,
    IncompatibleModelFamilyError,
    ModelFamily,
    TaskType,
    UnsupportedHydraCompositionError,
)


class ModelNamingTests(unittest.TestCase):
    def test_dfine_hydra_name(self) -> None:
        name = HydraModelName.parse("dfine-s=f1+dfine-s")
        self.assertEqual(name.family(), ModelFamily.DFINE)
        self.assertEqual(name.heads[0].task_type(), TaskType.OBJECT)
        self.assertEqual(str(name), "dfine-s=f1+dfine-s")

    def test_cross_family_is_rejected(self) -> None:
        for name in ("dfine-s=f1+yolo26m", "yolo26m=f11+dfine-s"):
            with (
                self.subTest(name=name),
                self.assertRaises(IncompatibleModelFamilyError),
            ):
                HydraModelName.parse(name)

    def test_dfine_only_accepts_one_dfine_head(self) -> None:
        with self.assertRaises(UnsupportedHydraCompositionError):
            HydraModelName.parse("dfine-s=f1+dfine-s+dfine-s")

    def test_invalid_dfine_split_is_rejected(self) -> None:
        with self.assertRaises(UnsupportedHydraCompositionError):
            HydraModelName.parse("dfine-s=f11+dfine-s")


class PostProcessorTests(unittest.TestCase):
    def test_flattened_query_class_topk(self) -> None:
        processor = DFINEPostProcessorAdapter(num_classes=2, topk=2)
        outputs = {
            "pred_logits": torch.tensor([[[0.0, 3.0], [2.0, -1.0]]]),
            "pred_boxes": torch.tensor(
                [[[0.25, 0.25, 0.2, 0.2], [0.75, 0.75, 0.4, 0.4]]]
            ),
        }
        detections = processor(outputs)
        self.assertEqual(tuple(detections.shape), (1, 2, 6))
        self.assertEqual(int(detections[0, 0, 5]), 1)
        self.assertEqual(int(detections[0, 1, 5]), 0)
        torch.testing.assert_close(
            detections[0, 0, :4], outputs["pred_boxes"][0, 0]
        )
        torch.testing.assert_close(
            detections[0, 1, :4], outputs["pred_boxes"][0, 1]
        )


class DatasetTests(unittest.TestCase):
    def test_yolo_labels_become_normalized_cxcywh(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "images" / "val").mkdir(parents=True)
            (root / "labels" / "val").mkdir(parents=True)
            image_path = root / "images" / "val" / "image.jpg"
            Image.new("RGB", (80, 40), color="white").save(image_path)
            (root / "labels" / "val" / "image.txt").write_text(
                "0 0.5 0.5 0.25 0.5\n",
                encoding="utf-8",
            )
            data_path = root / "data.yaml"
            data_path.write_text(
                yaml.safe_dump(
                    {
                        "path": str(root),
                        "names": {0: "object"},
                        "train": "images/val",
                        "val": "images/val",
                    }
                ),
                encoding="utf-8",
            )
            dataset = DFINEDataset(data_path, "val")
            image, target = dataset[0]
            self.assertEqual(tuple(image.shape), (3, 640, 640))
            torch.testing.assert_close(
                target["boxes"],
                torch.tensor([[0.5, 0.5, 0.25, 0.5]]),
            )


class CriterionTests(unittest.TestCase):
    @staticmethod
    def _decoder_output(
        logits: torch.Tensor,
        boxes: torch.Tensor,
        corners: torch.Tensor,
        references: torch.Tensor,
        teacher_corners: torch.Tensor,
        teacher_logits: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        return {
            "pred_logits": logits,
            "pred_boxes": boxes,
            "pred_corners": corners,
            "ref_points": references,
            "teacher_corners": teacher_corners,
            "teacher_logits": teacher_logits,
        }

    def test_all_empty_batch_has_finite_loss_and_gradients(self) -> None:
        logits = torch.randn(1, 2, 2, requires_grad=True)
        boxes = torch.sigmoid(torch.randn(1, 2, 4, requires_grad=True))
        corners = torch.randn(1, 2, 132, requires_grad=True)
        references = torch.sigmoid(torch.randn(1, 2, 4))
        auxiliary_corners = torch.randn(1, 2, 132, requires_grad=True)
        auxiliary_logits = torch.randn(1, 2, 2, requires_grad=True)
        outputs: dict[str, object] = {
            "pred_logits": logits,
            "pred_boxes": boxes,
            "pred_corners": corners,
            "ref_points": references,
            "up": torch.tensor([0.5]),
            "reg_scale": torch.tensor([4.0]),
            "aux_outputs": [
                self._decoder_output(
                    auxiliary_logits,
                    boxes,
                    auxiliary_corners,
                    references,
                    corners,
                    logits,
                )
            ],
            "enc_aux_outputs": [{"pred_logits": logits, "pred_boxes": boxes}],
            "pre_outputs": {"pred_logits": logits, "pred_boxes": boxes},
            "enc_meta": {"class_agnostic": False},
        }
        targets = [
            {
                "labels": torch.empty(0, dtype=torch.long),
                "boxes": torch.empty(0, 4),
            }
        ]
        losses = DFINECriterion(num_classes=2)(outputs, targets)
        total = torch.stack(tuple(losses.values())).sum()
        self.assertTrue(torch.isfinite(total))
        total.backward()
        self.assertIsNotNone(logits.grad)


class ManifestTests(unittest.TestCase):
    def test_manifest_hash_is_stable(self) -> None:
        architecture = DFINEArchitectureConfig()
        first = build_manifest(architecture, ["ball", "robot"])
        second = build_manifest(architecture, ["ball", "robot"])
        self.assertEqual(first.config_hash, second.config_hash)
        self.assertEqual(first.postprocessing["type"], "topk_no_nms")


if __name__ == "__main__":
    unittest.main()
