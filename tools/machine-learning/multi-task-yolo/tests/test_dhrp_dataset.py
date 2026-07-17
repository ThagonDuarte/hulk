import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import torch
from PIL import Image

from ultralytics_dfine.data.dhrp import (
    DHRP_JOINT_PARENTS,
    DHRP_KEYPOINT_NAMES,
    DHRP_NUM_KEYPOINTS,
    DHRPDataset,
    DHRPFormatError,
    correct_dhrp_image_path,
)
from validation.multitask_data import dataset_target_fingerprint


class DHRPDatasetTests(unittest.TestCase):
    @staticmethod
    def _write_fixture(
        root: Path,
        keypoints: list[list[float]],
    ) -> Path:
        image_path = root / "train" / "robot.png"
        image_path.parent.mkdir(parents=True)
        Image.new("RGB", (100, 200), color="white").save(image_path)
        annotation_path = root / "annot" / "train_set_fixture.json"
        annotation_path.parent.mkdir()
        annotation_path.write_text(
            json.dumps(
                {
                    "joint_names": list(DHRP_KEYPOINT_NAMES),
                    "joint_parents": list(DHRP_JOINT_PARENTS),
                    "num_joints": DHRP_NUM_KEYPOINTS,
                    "num_images": 1,
                    "annotations": [
                        {
                            "img": "train/robot.png",
                            "key": keypoints,
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )
        return annotation_path

    @staticmethod
    def _write_person_negative_manifest(
        root: Path,
        annotation_path: Path,
        *,
        role: str,
    ) -> Path:
        relative_path = "train/robot.png"
        image_path = root / relative_path
        image_sha256 = hashlib.sha256(image_path.read_bytes()).hexdigest()
        image_set = hashlib.sha256()
        image_set.update(b"dhrp-person-negative-images-v1\0")
        image_set.update(f"{relative_path}\0{image_sha256}\n".encode())
        manifest = root / "person-negative.json"
        manifest.write_text(
            json.dumps(
                {
                    "version": 1,
                    "policy": "verified-person-free-dhrp-v1",
                    "annotations": {
                        annotation_path.name: {
                            "annotation_sha256": hashlib.sha256(
                                annotation_path.read_bytes()
                            ).hexdigest(),
                            "image_set_sha256": image_set.hexdigest(),
                            "records": 1,
                            "role": role,
                        }
                    },
                }
            ),
            encoding="utf-8",
        )
        return manifest

    def test_exact_keypoint_schema_and_normalization(self) -> None:
        self.assertEqual(
            DHRP_KEYPOINT_NAMES,
            (
                "Nose",
                "Neck",
                "RShoulder",
                "RElbow",
                "RWrist",
                "LShoulder",
                "LElbow",
                "LWrist",
                "RHip",
                "RKnee",
                "RAnkle",
                "LHip",
                "LKnee",
                "LAnkle",
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[10.0, 20.0, 0.0] for _ in range(14)]
            annotation_path = self._write_fixture(root, points)
            _, target = DHRPDataset(root, annotation_path)[0]
        self.assertEqual(tuple(target["keypoints"].shape), (1, 14, 3))
        torch.testing.assert_close(
            target["keypoints"][0, 0],
            torch.tensor([0.1, 0.1, 0.0]),
        )
        self.assertFalse(target["person_negative_verified"])
        self.assertFalse(target["person_negative_eligible"])

    def test_exact_manifest_marks_only_selected_evaluation_role_eligible(
        self,
    ) -> None:
        points = [[10.0, 20.0, 1.0] for _ in range(14)]
        for role, eligible in (
            ("primary_evaluation", True),
            ("loss_holdout_validation", False),
        ):
            with (
                self.subTest(role=role),
                tempfile.TemporaryDirectory() as value,
            ):
                root = Path(value)
                annotation = self._write_fixture(root, points)
                manifest = self._write_person_negative_manifest(
                    root,
                    annotation,
                    role=role,
                )
                dataset = DHRPDataset(
                    root,
                    annotation,
                    person_negative_manifest=manifest,
                )
                _, target = dataset[0]

                self.assertTrue(target["person_negative_verified"])
                self.assertEqual(
                    target["person_negative_eligible"],
                    eligible,
                )
                self.assertEqual(dataset.person_negative_verified_records, 1)
                self.assertEqual(
                    dataset.person_negative_eligible_records,
                    int(eligible),
                )

    def test_manifest_rejects_changed_person_negative_image(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[10.0, 20.0, 1.0] for _ in range(14)]
            annotation = self._write_fixture(root, points)
            manifest = self._write_person_negative_manifest(
                root,
                annotation,
                role="primary_evaluation",
            )
            Image.new("RGB", (100, 200), color="black").save(
                root / "train" / "robot.png"
            )

            with self.assertRaisesRegex(
                DHRPFormatError,
                "image-set SHA-256 mismatch",
            ):
                DHRPDataset(
                    root,
                    annotation,
                    person_negative_manifest=manifest,
                )

    def test_target_fingerprint_binds_person_negative_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[10.0, 20.0, 1.0] for _ in range(14)]
            annotation = self._write_fixture(root, points)
            manifest = self._write_person_negative_manifest(
                root,
                annotation,
                role="primary_evaluation",
            )
            baseline = dataset_target_fingerprint(DHRPDataset(root, annotation))
            eligible = dataset_target_fingerprint(
                DHRPDataset(
                    root,
                    annotation,
                    person_negative_manifest=manifest,
                )
            )

        self.assertNotEqual(baseline["digest"], eligible["digest"])
        self.assertEqual(eligible["person_negative_verified_records"], 1)
        self.assertEqual(eligible["person_negative_eligible_records"], 1)
        self.assertIsNotNone(eligible["person_negative_manifest_sha256"])

    def test_positive_record_has_padded_normalized_box(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[0.0, 0.0, 0.0] for _ in range(14)]
            points[0] = [20.0, 40.0, 1.0]
            points[1] = [60.0, 120.0, 0.0]
            annotation_path = self._write_fixture(root, points)
            _, target = DHRPDataset(root, annotation_path)[0]
        self.assertEqual(target["labels"].tolist(), [4])
        torch.testing.assert_close(
            target["boxes"],
            torch.tensor([[0.4, 0.4, 0.52, 0.52]]),
        )
        self.assertEqual(target["keypoints"][0, 1, 2].item(), 0.0)

    def test_out_of_frame_points_do_not_define_box_or_visibility(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[0.0, 0.0, 0.0] for _ in range(14)]
            points[0] = [20.0, 40.0, 1.0]
            points[1] = [200.0, 300.0, 1.0]
            annotation_path = self._write_fixture(root, points)
            image, target = DHRPDataset(
                root,
                annotation_path,
                image_size=(448, 544),
            )[0]

        self.assertEqual(tuple(image.shape), (3, 448, 544))
        self.assertEqual(target["keypoints"][0, 1].tolist(), [0.0, 0.0, 0.0])
        torch.testing.assert_close(
            target["boxes"],
            torch.tensor([[0.2, 0.2, 0.01, 0.005]]),
        )

    def test_all_zero_background_has_empty_target(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = [[0.0, 0.0, 0.0] for _ in range(14)]
            annotation_path = self._write_fixture(root, points)
            _, target = DHRPDataset(root, annotation_path)[0]
        self.assertEqual(tuple(target["labels"].shape), (0,))
        self.assertEqual(tuple(target["boxes"].shape), (0, 4))
        self.assertEqual(tuple(target["keypoints"].shape), (0, 14, 3))

    def test_eve_annotation_correction(self) -> None:
        corrected = correct_dhrp_image_path(
            "annot/train_set_TargetHumanoidRobots_EVE.json",
            303,
            "train/TargetHumanoidRobots/EVE/eve_0.png",
        )
        self.assertEqual(
            corrected,
            "train/TargetHumanoidRobots/EVE/eve4_0.png",
        )
        unchanged = correct_dhrp_image_path(
            "annot/train_set_TargetHumanoidRobots_EVE.json",
            302,
            "train/TargetHumanoidRobots/EVE/eve_0.png",
        )
        self.assertEqual(
            unchanged,
            "train/TargetHumanoidRobots/EVE/eve_0.png",
        )


if __name__ == "__main__":
    unittest.main()
