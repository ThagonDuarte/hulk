use eframe::egui::{Align2, Color32, Stroke};
use linear_algebra::point;
use types::{bounding_box::BoundingBox, pose_detection::Keypoint};

use super::super::image_overlay::ImageOverlayPainter;

pub(super) const KEYPOINT_CONFIDENCE_THRESHOLD: f32 = 0.8;

#[derive(Clone, Copy)]
pub(super) struct PoseStyle {
    pub(super) skeleton: Color32,
    pub(super) keypoint: Color32,
    pub(super) bounding_box: Color32,
}

pub(super) fn paint_pose<const NUMBER_OF_KEYPOINTS: usize>(
    painter: &ImageOverlayPainter,
    bounding_box: BoundingBox,
    label: String,
    keypoints: &[Keypoint; NUMBER_OF_KEYPOINTS],
    skeleton: &[(usize, usize)],
    style: PoseStyle,
) {
    for &(start, end) in skeleton {
        if keypoints[start].confidence < KEYPOINT_CONFIDENCE_THRESHOLD
            || keypoints[end].confidence < KEYPOINT_CONFIDENCE_THRESHOLD
        {
            continue;
        }

        painter.line_segment(
            keypoints[start].point,
            keypoints[end].point,
            Stroke::new(2.0, style.skeleton),
        );
    }

    for keypoint in keypoints {
        if keypoint.confidence < KEYPOINT_CONFIDENCE_THRESHOLD {
            continue;
        }

        painter.circle_filled(keypoint.point, 1.0, style.keypoint);
        painter.floating_text(
            keypoint.point,
            Align2::RIGHT_BOTTOM,
            format!("{:.2}", keypoint.confidence),
            Color32::WHITE,
        );
    }

    painter.rect_stroke(
        bounding_box.area.min,
        bounding_box.area.max,
        Stroke::new(2.0, style.bounding_box),
    );
    painter.floating_text(
        point![bounding_box.area.max.x(), bounding_box.area.min.y()],
        Align2::RIGHT_TOP,
        format!("{:.2}", bounding_box.confidence),
        Color32::WHITE,
    );
    painter.floating_text(
        point![bounding_box.area.min.x(), bounding_box.area.max.y()],
        Align2::LEFT_BOTTOM,
        label,
        Color32::WHITE,
    );
}
