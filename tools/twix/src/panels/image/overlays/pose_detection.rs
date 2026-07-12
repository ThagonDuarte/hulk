use color_eyre::Report;
use eframe::egui::{Align2, Color32, Stroke};
use linear_algebra::point;
use ros_z::time::Time;
use types::{
    object_detection::YOLOObjectLabel,
    pose_detection::{Keypoint, POSE_SKELETON_EDGES, Pose},
    time_wrapper::TimeWrapper,
};

use crate::repaint::ObservationContext;

use super::super::image_overlay::{ImageOverlay, ImageOverlayPainter, OverlayObservation};

const KEYPOINT_CONFIDENCE_THRESHOLD: f32 = 0.8;

pub(in crate::panels::image) struct PoseDetectionOverlay {
    poses: OverlayObservation<TimeWrapper<Vec<Pose<YOLOObjectLabel>>>>,
}

impl ImageOverlay for PoseDetectionOverlay {
    const NAME: &'static str = "Pose Detection";
    const STORAGE_KEY: &'static str = "pose_detection";

    fn new<C>(context: &C) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        Ok(Self {
            poses: OverlayObservation::new(context, "detected_poses")?,
        })
    }

    fn paint(&self, painter: &ImageOverlayPainter, image_time: Time) {
        let Some(poses) = self.poses.at_time(image_time) else {
            return;
        };
        paint_poses(painter, &poses.value.inner);
    }

    fn latest_time(&self) -> Option<Time> {
        self.poses.latest_time()
    }
}

fn paint_poses(painter: &ImageOverlayPainter, poses: &[Pose<YOLOObjectLabel>]) {
    for pose in poses {
        let keypoints: [Keypoint; 17] = pose.keypoints.into();

        for (idx1, idx2) in POSE_SKELETON_EDGES {
            if keypoints[idx1].confidence < KEYPOINT_CONFIDENCE_THRESHOLD
                || keypoints[idx2].confidence < KEYPOINT_CONFIDENCE_THRESHOLD
            {
                continue;
            }

            painter.line_segment(
                keypoints[idx1].point,
                keypoints[idx2].point,
                Stroke::new(2.0, Color32::LIGHT_BLUE.gamma_multiply(0.4)),
            );
        }

        for keypoint in keypoints {
            if keypoint.confidence < KEYPOINT_CONFIDENCE_THRESHOLD {
                continue;
            }

            painter.circle_filled(keypoint.point, 1.0, Color32::BLUE);
            painter.floating_text(
                keypoint.point,
                Align2::RIGHT_BOTTOM,
                format!("{:.2}", keypoint.confidence),
                Color32::WHITE,
            );
        }

        let bounding_box = pose.object.bounding_box;
        painter.rect_stroke(
            bounding_box.area.min,
            bounding_box.area.max,
            Stroke::new(2.0, Color32::DARK_BLUE.gamma_multiply(0.8)),
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
            format!("{:.2?}", pose.object.label),
            Color32::WHITE,
        );
    }
}
