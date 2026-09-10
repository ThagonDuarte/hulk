use eframe::egui::{Align2, Color32};
use types::{bounding_box::BoundingBox, pose_detection::Keypoint};

use super::super::image_overlay::{
    ConfidenceThresholdDefinition, ConfidenceThresholdKind, ConfidenceThresholds,
    ImageOverlayPainter,
};

pub(super) const POSE_CONFIDENCE_THRESHOLDS: [ConfidenceThresholdDefinition; 2] = [
    ConfidenceThresholdDefinition::new(
        ConfidenceThresholdKind::BoundingBox,
        "Bounding box confidence",
        "bounding_box_confidence_threshold",
    ),
    ConfidenceThresholdDefinition::new(
        ConfidenceThresholdKind::Keypoint,
        "Keypoint confidence",
        "keypoint_confidence_threshold",
    ),
];

pub(super) fn paint_pose(
    painter: &ImageOverlayPainter,
    bounding_box: BoundingBox,
    label: String,
    keypoints: &[Keypoint],
    skeleton: &[(usize, usize)],
    color: Color32,
    confidence_thresholds: &ConfidenceThresholds,
) {
    if bounding_box.confidence < confidence_thresholds.bounding_box {
        return;
    }
    for &(start, end) in skeleton {
        if keypoints[start].confidence < confidence_thresholds.keypoint
            || keypoints[end].confidence < confidence_thresholds.keypoint
        {
            continue;
        }
        painter.detection_line_segment(keypoints[start].point, keypoints[end].point, color);
    }
    for keypoint in keypoints {
        if keypoint.confidence < confidence_thresholds.keypoint {
            continue;
        }
        painter.circle_filled(keypoint.point, 1.0, color);
        painter.floating_text(
            keypoint.point,
            Align2::RIGHT_BOTTOM,
            format!("{:.2}", keypoint.confidence),
            Color32::WHITE,
        );
    }
    painter.detection_box(bounding_box, label, color);
}
