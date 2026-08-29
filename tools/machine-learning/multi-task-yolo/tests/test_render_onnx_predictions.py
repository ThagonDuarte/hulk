from __future__ import annotations

from unittest import TestCase

import numpy as np

from validation import render_onnx_predictions

Prediction = render_onnx_predictions.Prediction


class RenderOnnxPredictionsTest(TestCase):
    def test_model_classes_have_unique_colors(self) -> None:
        colors = render_onnx_predictions.model_class_colors()

        self.assertEqual(len(colors), 9)
        self.assertEqual(len(set(colors.values())), len(colors))

    def test_bgr_to_nv12_uses_runtime_input_layout(self) -> None:
        image = np.zeros((64, 96, 3), dtype=np.uint8)

        nv12 = render_onnx_predictions.bgr_to_nv12(image)

        self.assertEqual(nv12.shape, (32, 48, 6))
        self.assertEqual(nv12.dtype, np.uint8)

    def test_auto_provider_prefers_gpu(self) -> None:
        provider = render_onnx_predictions.choose_execution_provider(
            "auto",
            available=("CPUExecutionProvider", "CUDAExecutionProvider"),
        )

        self.assertEqual(provider, "CUDAExecutionProvider")

    def test_label_bounds_avoid_an_occupied_label(self) -> None:
        first = render_onnx_predictions.label_bounds(
            box=(20, 0, 100, 80),
            label_size=(60, 16),
            image_size=(96, 120),
            occupied=(),
        )
        second = render_onnx_predictions.label_bounds(
            box=(24, 0, 104, 80),
            label_size=(60, 16),
            image_size=(96, 120),
            occupied=(first,),
        )

        self.assertFalse(
            render_onnx_predictions.rectangles_overlap(first, second)
        )

    def test_non_maximum_suppression_retains_best_overlap(self) -> None:
        predictions = [
            Prediction(
                cls=0,
                conf=0.9,
                box_xyxy=np.array([10, 10, 50, 50], dtype=np.float32),
            ),
            Prediction(
                cls=0,
                conf=0.7,
                box_xyxy=np.array([12, 12, 48, 48], dtype=np.float32),
            ),
        ]

        retained = render_onnx_predictions.non_maximum_suppression(
            predictions,
            maximum_iou=0.4,
        )

        self.assertEqual(len(retained), 1)
        self.assertEqual(retained[0].conf, 0.9)

    def test_person_pose_overlapping_robot_pose_is_filtered(self) -> None:
        person_pose = Prediction(
            cls=0,
            conf=0.9,
            box_xyxy=np.array([10, 10, 50, 50], dtype=np.float32),
        )
        robot_pose = Prediction(
            cls=0,
            conf=0.8,
            box_xyxy=np.array([11, 11, 49, 49], dtype=np.float32),
        )

        retained = (
            render_onnx_predictions.filter_person_poses_overlapping_robots(
                [person_pose],
                [robot_pose],
                maximum_iou=0.8,
            )
        )

        self.assertEqual(retained, [])

    def test_pose_rendering_does_not_draw_bounding_box(self) -> None:
        image = np.zeros((96, 96, 3), dtype=np.uint8)
        keypoints = np.zeros((17, 3), dtype=np.float32)
        prediction = Prediction(
            cls=0,
            conf=0.9,
            box_xyxy=np.array([10, 10, 80, 80], dtype=np.float32),
            keypoints=keypoints,
        )

        rendered = render_onnx_predictions.render_onnx_predictions(
            image,
            objects=[],
            person_poses=[prediction],
            robot_poses=[],
            draw_labels=False,
            keypoint_confidence_threshold=0.25,
        )

        np.testing.assert_array_equal(rendered, image)

    def test_no_labels_keeps_object_box_without_text(self) -> None:
        image = np.zeros((96, 120, 3), dtype=np.uint8)
        prediction = Prediction(
            cls=0,
            conf=0.9,
            box_xyxy=np.array([20, 20, 100, 80], dtype=np.float32),
        )

        without_labels = render_onnx_predictions.render_onnx_predictions(
            image,
            objects=[prediction],
            person_poses=[],
            robot_poses=[],
            draw_labels=False,
            keypoint_confidence_threshold=0.25,
        )
        with_labels = render_onnx_predictions.render_onnx_predictions(
            image,
            objects=[prediction],
            person_poses=[],
            robot_poses=[],
            draw_labels=True,
            keypoint_confidence_threshold=0.25,
        )

        self.assertGreater(int(without_labels.sum()), 0)
        self.assertGreater(int(with_labels.sum()), int(without_labels.sum()))
