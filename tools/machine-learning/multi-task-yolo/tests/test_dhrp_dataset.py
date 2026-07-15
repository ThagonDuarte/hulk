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
    correct_dhrp_image_path,
)


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
