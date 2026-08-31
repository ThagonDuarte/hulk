use color_eyre::Report;
use eframe::egui::Color32;
use ros_z::time::Time;
use types::{
    object_detection::YOLOObjectLabel,
    pose_detection::{Keypoint, Pose},
    time_wrapper::TimeWrapper,
};

use crate::repaint::ObservationContext;

use super::super::image_overlay::{ImageOverlay, ImageOverlayPainter, OverlayObservation};
use super::pose::{PoseStyle, paint_pose};

const POSE_SKELETON_KEYPOINT_LINE_MAPPING: [(usize, usize); 16] = [
    (0, 1),
    (0, 2),
    (1, 3),
    (2, 4),
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (5, 7),
    (6, 8),
    (7, 9),
    (8, 10),
    (11, 13),
    (12, 14),
    (13, 15),
    (14, 16),
];
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
        paint_pose(
            painter,
            pose.object.bounding_box,
            format!("{:.2?}", pose.object.label),
            &keypoints,
            &POSE_SKELETON_KEYPOINT_LINE_MAPPING,
            PoseStyle {
                skeleton: Color32::LIGHT_BLUE.gamma_multiply(0.4),
                keypoint: Color32::BLUE,
                bounding_box: Color32::DARK_BLUE.gamma_multiply(0.8),
            },
        );
    }
}
