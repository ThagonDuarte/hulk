import tempfile
import unittest
from pathlib import Path

import torch
import yaml
from PIL import Image

from ultralytics_dfine.data import YOLOKeypointDataset
from ultralytics_dfine.schemas import COCO_FLIP_IDX, HeadId


class YOLOKeypointDatasetTests(unittest.TestCase):
    @staticmethod
    def _fixture(
        root: Path,
        *,
        names: list[str],
        label: str,
    ) -> Path:
        (root / "images" / "train").mkdir(parents=True)
        (root / "labels" / "train").mkdir(parents=True)
        Image.new("RGB", (80, 40), color="white").save(
            root / "images" / "train" / "image.jpg"
        )
        (root / "labels" / "train" / "image.txt").write_text(
            label,
            encoding="utf-8",
        )
        data = root / "data.yaml"
        data.write_text(
            yaml.safe_dump(
                {
                    "path": str(root),
                    "names": names,
                    "train": "images/train",
                    "val": "images/train",
                }
            ),
            encoding="utf-8",
        )
        return data

    def test_person_rows_map_to_global_person_class(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            points = " ".join("0.25 0.5 2" for _ in range(17))
            data = self._fixture(
                root,
                names=["person"],
                label=f"0 0.5 0.5 0.4 0.8 {points}\n",
            )
            dataset = YOLOKeypointDataset(
                data,
                "train",
                schema_id=HeadId.PERSON_POSE,
                keypoint_count=17,
                flip_idx=COCO_FLIP_IDX,
                global_class_ids=(7,),
                horizontal_flip_probability=0,
            )

            image, target = dataset[0]

        self.assertEqual(image.shape, (3, 640, 640))
        self.assertEqual(target["labels"].tolist(), [7])
        keypoints = target.get("keypoints")
        valid_classes = target.get("valid_detection_classes")
        self.assertIsInstance(keypoints, torch.Tensor)
        self.assertIsInstance(valid_classes, torch.Tensor)
        assert isinstance(keypoints, torch.Tensor)
        assert isinstance(valid_classes, torch.Tensor)
        self.assertEqual(keypoints.shape, (1, 17, 3))
        self.assertEqual(valid_classes.sum().item(), 1)

    def test_field_rows_map_two_dimensional_points_to_schema(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            data = self._fixture(
                root,
                names=["GoalPost", "LSpot", "PenaltySpot", "TSpot", "XSpot"],
                label="2 0.5 0.5 0.2 0.2 0.6 0.7\n",
            )
            dataset = YOLOKeypointDataset(
                data,
                "train",
                schema_id=HeadId.FIELD_FEATURES,
                keypoint_count=1,
                keypoint_dimensions=2,
                flip_idx=(0,),
                global_class_ids=(1, 2, 3, 5, 6),
                horizontal_flip_probability=0,
                point_set=True,
                point_label_ids=(0, 1, 3, 2, 4),
            )

            _, target = dataset[0]

        self.assertEqual(target["labels"].tolist(), [3])
        point_labels = target.get("point_labels")
        points = target.get("points")
        self.assertIsInstance(point_labels, torch.Tensor)
        self.assertIsInstance(points, torch.Tensor)
        assert isinstance(point_labels, torch.Tensor)
        assert isinstance(points, torch.Tensor)
        self.assertEqual(point_labels.tolist(), [3])
        torch.testing.assert_close(
            points,
            torch.tensor([[0.6, 0.7]]),
        )


if __name__ == "__main__":
    unittest.main()
