use color_eyre::Report;
use ros_z::time::Time;
use types::{pose_detection::RobotPose, time_wrapper::TimeWrapper};

use super::super::image_overlay::{
    ConfidenceThresholdDefinition, ConfidenceThresholds, ImageOverlay, ImageOverlayPainter,
    OverlayObservation,
};
use super::{
    pose_detection::{POSE_CONFIDENCE_THRESHOLDS, paint_pose},
    prediction_colors,
};
use crate::repaint::ObservationContext;

pub(in crate::panels::image) struct RobotPoseDetectionOverlay {
    poses: OverlayObservation<TimeWrapper<Vec<RobotPose>>>,
}

impl ImageOverlay for RobotPoseDetectionOverlay {
    const NAME: &'static str = "Robot Pose Detection";
    const STORAGE_KEY: &'static str = "robot_pose_detection";
    const CONFIDENCE_THRESHOLDS: &'static [ConfidenceThresholdDefinition] =
        &POSE_CONFIDENCE_THRESHOLDS;

    fn new<C: ObservationContext>(context: &C) -> Result<Self, Report> {
        Ok(Self {
            poses: OverlayObservation::new(context, "detected_robot_poses")?,
        })
    }

    fn paint(
        &self,
        painter: &ImageOverlayPainter,
        image_time: Time,
        thresholds: &ConfidenceThresholds,
    ) {
        let Some(poses) = self.poses.at_time(image_time) else {
            return;
        };
        for pose in &poses.value.inner {
            paint_pose(
                painter,
                pose.object.bounding_box,
                pose.object.label.into(),
                &pose.keypoints,
                &[
                    (1, 0),
                    (1, 2),
                    (2, 3),
                    (3, 4),
                    (1, 5),
                    (5, 6),
                    (6, 7),
                    (1, 8),
                    (8, 9),
                    (9, 10),
                    (1, 11),
                    (11, 12),
                    (12, 13),
                ],
                prediction_colors::robocup_object(pose.object.label),
                thresholds,
            );
        }
    }

    fn latest_time(&self) -> Option<Time> {
        self.poses.latest_time()
    }
}
