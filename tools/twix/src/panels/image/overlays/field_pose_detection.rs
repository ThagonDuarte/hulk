use color_eyre::Report;
use ros_z::time::Time;
use types::{pose_detection::FieldPose, time_wrapper::TimeWrapper};

use super::super::image_overlay::{
    ConfidenceThresholdDefinition, ConfidenceThresholds, ImageOverlay, ImageOverlayPainter,
    OverlayObservation,
};
use super::{
    pose_detection::{POSE_CONFIDENCE_THRESHOLDS, paint_pose},
    prediction_colors,
};
use crate::repaint::ObservationContext;

pub(in crate::panels::image) struct FieldPoseDetectionOverlay {
    poses: OverlayObservation<TimeWrapper<Vec<FieldPose>>>,
}

impl ImageOverlay for FieldPoseDetectionOverlay {
    const NAME: &'static str = "Field Pose Detection";
    const STORAGE_KEY: &'static str = "field_pose_detection";
    const CONFIDENCE_THRESHOLDS: &'static [ConfidenceThresholdDefinition] =
        &POSE_CONFIDENCE_THRESHOLDS;

    fn new<C: ObservationContext>(context: &C) -> Result<Self, Report> {
        Ok(Self {
            poses: OverlayObservation::new(context, "detected_field_poses")?,
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
                &[],
                prediction_colors::robocup_object(pose.object.label),
                thresholds,
            );
        }
    }

    fn latest_time(&self) -> Option<Time> {
        self.poses.latest_time()
    }
}
