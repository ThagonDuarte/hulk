import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import torch
import yaml
from PIL import Image

from ultralytics_dfine.config import (
    DFINEArchitectureConfig,
    build_manifest,
    build_multitask_manifest,
)
from ultralytics_dfine.data import DFINEDataset
from ultralytics_dfine.loss import DFINECriterion
from ultralytics_dfine.nn import (
    DFINEDetectionModel,
    DFINEPostProcessorAdapter,
    ONNXCompatibleDFINEIntegral,
)
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

    def test_dfine_multitask_hydra_name(self) -> None:
        name = HydraModelName.parse("dfine-s=f1+dfine-multitask~experiment")
        self.assertEqual(name.family(), ModelFamily.DFINE)
        self.assertEqual(name.heads[0].task_type(), TaskType.OBJECT)

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
        selected, query_indices = processor.select(outputs)
        self.assertEqual(tuple(detections.shape), (1, 2, 6))
        torch.testing.assert_close(selected, detections)
        self.assertEqual(query_indices.tolist(), [[0, 1]])
        self.assertEqual(int(detections[0, 0, 5]), 1)
        self.assertEqual(int(detections[0, 1, 5]), 0)
        torch.testing.assert_close(
            detections[0, 0, :4], outputs["pred_boxes"][0, 0]
        )
        torch.testing.assert_close(
            detections[0, 1, :4], outputs["pred_boxes"][0, 1]
        )


class ExportTests(unittest.TestCase):
    def test_onnx_integral_matches_vector_linear(self) -> None:
        max_num_bins = 32
        pred_corners = torch.randn(2, 5, 4 * (max_num_bins + 1))
        project = torch.linspace(0.1, 1.0, max_num_bins + 1)
        probabilities = torch.softmax(
            pred_corners.reshape(-1, max_num_bins + 1),
            dim=1,
        )
        expected = torch.nn.functional.linear(
            probabilities,
            project,
        ).reshape(2, 5, 4)

        actual = ONNXCompatibleDFINEIntegral(max_num_bins)(
            pred_corners,
            project,
        )

        torch.testing.assert_close(actual, expected)


class SharedFeatureTests(unittest.TestCase):
    def test_denoising_queries_are_removed_from_final_features(self) -> None:
        features = torch.arange(24, dtype=torch.float32).reshape(1, 6, 4)
        raw = SimpleNamespace(
            last_hidden_state=features,
            denoising_meta_values={"dn_num_split": [2, 4]},
        )

        selected = DFINEDetectionModel._final_query_features(raw)

        torch.testing.assert_close(selected, features[:, 2:])

    def test_encoder_features_require_a_tensor_sequence(self) -> None:
        expected = (torch.randn(1, 4, 8, 8), torch.randn(1, 4, 4, 4))
        raw = SimpleNamespace(encoder_last_hidden_state=list(expected))

        actual = DFINEDetectionModel._encoder_feature_maps(raw)

        self.assertEqual(actual, expected)


class ClassExpansionTests(unittest.TestCase):
    def test_classifier_expansion_preserves_rows(self) -> None:
        classifier = torch.nn.Linear(4, 2)
        original_weight = classifier.weight.detach().clone()
        original_bias = classifier.bias.detach().clone()

        expanded = DFINEDetectionModel._expanded_classifier(
            classifier,
            initial_bias=-10.0,
        )

        torch.testing.assert_close(expanded.weight[:2], original_weight)
        torch.testing.assert_close(expanded.bias[:2], original_bias)
        torch.testing.assert_close(expanded.weight[2], torch.zeros(4))
        self.assertEqual(expanded.bias[2].item(), -10.0)

    def test_denoising_expansion_moves_padding_row(self) -> None:
        embedding = torch.nn.Embedding(3, 4, padding_idx=2)
        with torch.no_grad():
            embedding.weight.copy_(torch.arange(12).reshape(3, 4))

        expanded = DFINEDetectionModel._expanded_denoising_embedding(embedding)

        self.assertEqual(expanded.padding_idx, 3)
        torch.testing.assert_close(expanded.weight[:2], embedding.weight[:2])
        torch.testing.assert_close(expanded.weight[2], torch.zeros(4))
        torch.testing.assert_close(expanded.weight[3], embedding.weight[2])


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

    def test_unlisted_model_class_can_be_explicitly_filtered(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "images" / "val").mkdir(parents=True)
            (root / "labels" / "val").mkdir(parents=True)
            Image.new("RGB", (80, 40), color="white").save(
                root / "images" / "val" / "image.jpg"
            )
            (root / "labels" / "val" / "image.txt").write_text(
                "0 0.5 0.5 0.25 0.5\n7 0.5 0.5 0.2 0.4\n",
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
            dataset = DFINEDataset(
                data_path,
                "val",
                num_detection_classes=8,
                ignore_unlisted_classes=True,
            )

            _, target = dataset[0]
            audit = dataset.audit_annotations()

        self.assertEqual(target["labels"].tolist(), [0])
        self.assertEqual(audit["ignored_rows"], 1)
        valid_classes = target.get("valid_detection_classes")
        self.assertIsInstance(valid_classes, torch.Tensor)
        assert isinstance(valid_classes, torch.Tensor)
        self.assertFalse(valid_classes[7])

    def test_class_outside_model_range_remains_an_error(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "images" / "val").mkdir(parents=True)
            (root / "labels" / "val").mkdir(parents=True)
            Image.new("RGB", (80, 40), color="white").save(
                root / "images" / "val" / "image.jpg"
            )
            (root / "labels" / "val" / "image.txt").write_text(
                "8 0.5 0.5 0.25 0.5\n",
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
            dataset = DFINEDataset(
                data_path,
                "val",
                num_detection_classes=8,
                ignore_unlisted_classes=True,
            )

            with self.assertRaisesRegex(ValueError, "class 8 is out of range"):
                dataset.audit_annotations()


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
        criterion = DFINECriterion(num_classes=2)
        result = criterion.forward_with_matches(outputs, targets)
        losses = result.losses
        self.assertEqual(len(result.final_matches), 1)
        total = torch.stack(tuple(losses.values())).sum()
        self.assertTrue(torch.isfinite(total))
        total.backward()
        self.assertIsNotNone(logits.grad)

    def test_invalid_detection_classes_have_zero_vfl_gradient(self) -> None:
        logits = torch.zeros(1, 2, 2, requires_grad=True)
        boxes = torch.rand(1, 2, 4)
        targets = [
            {
                "labels": torch.empty(0, dtype=torch.long),
                "boxes": torch.empty(0, 4),
                "valid_detection_classes": torch.tensor([False, True]),
            }
        ]
        empty_match = [
            (
                torch.empty(0, dtype=torch.long),
                torch.empty(0, dtype=torch.long),
            )
        ]

        loss = DFINECriterion(2)._loss_vfl(
            {"pred_logits": logits, "pred_boxes": boxes},
            targets,
            empty_match,
            1.0,
        )["loss_vfl"]
        loss.backward()

        self.assertIsNotNone(logits.grad)
        assert logits.grad is not None
        torch.testing.assert_close(logits.grad[..., 0], torch.zeros(1, 2))
        self.assertTrue((logits.grad[..., 1] > 0).all())


class ManifestTests(unittest.TestCase):
    def test_manifest_hash_is_stable(self) -> None:
        architecture = DFINEArchitectureConfig()
        first = build_manifest(architecture, ["ball", "robot"])
        second = build_manifest(architecture, ["ball", "robot"])
        self.assertEqual(first.config_hash, second.config_hash)
        self.assertEqual(first.postprocessing["type"], "topk_no_nms")

    def test_multitask_manifest_has_fixed_outputs(self) -> None:
        manifest = build_multitask_manifest(
            DFINEArchitectureConfig(),
            ["Ball", "Robot", "Person"],
        )
        self.assertEqual(manifest.schema_version, 2)
        self.assertEqual(
            set(manifest.outputs),
            {
                "object_output",
                "person_pose_output",
                "robot_pose_output",
                "field_feature_output",
            },
        )


if __name__ == "__main__":
    unittest.main()
