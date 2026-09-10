use color_eyre::Report;
use ros_z::time::Time;
use types::{
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
};

use crate::repaint::ObservationContext;

use super::super::image_overlay::{
    ConfidenceThresholdDefinition, ConfidenceThresholdKind, ConfidenceThresholds, ImageOverlay,
    ImageOverlayPainter, OverlayObservation,
};
use super::prediction_colors;

const BALL_CONFIDENCE_THRESHOLDS: [ConfidenceThresholdDefinition; 1] =
    [ConfidenceThresholdDefinition::new(
        ConfidenceThresholdKind::BoundingBox,
        "Confidence",
        "confidence_threshold",
    )];

pub(in crate::panels::image) struct BallDetectionOverlay {
    object_detections: OverlayObservation<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>,
}

impl ImageOverlay for BallDetectionOverlay {
    const NAME: &'static str = "Ball Detection";
    // Retain settings saved under the former Object Detection name.
    const STORAGE_KEY: &'static str = "object_detection";
    const CONFIDENCE_THRESHOLDS: &'static [ConfidenceThresholdDefinition] =
        &BALL_CONFIDENCE_THRESHOLDS;

    fn new<C>(context: &C) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        Ok(Self {
            object_detections: OverlayObservation::new(context, "detected_objects")?,
        })
    }

    fn paint(
        &self,
        painter: &ImageOverlayPainter,
        image_time: Time,
        confidence_thresholds: &ConfidenceThresholds,
    ) {
        let Some(object_detections) = self.object_detections.at_time(image_time) else {
            return;
        };
        paint_bounding_boxes(
            painter,
            &object_detections.value.inner,
            confidence_thresholds.bounding_box,
        );
    }

    fn latest_time(&self) -> Option<Time> {
        self.object_detections.latest_time()
    }
}

fn paint_bounding_boxes(
    painter: &ImageOverlayPainter,
    detections: &[Object<RobocupObjectLabel>],
    confidence_threshold: f32,
) {
    for detection in detections
        .iter()
        .filter(|detection| detection.label == RobocupObjectLabel::Ball)
    {
        let bounding_box = detection.bounding_box;
        if bounding_box.confidence < confidence_threshold {
            continue;
        }
        painter.detection_box(
            bounding_box,
            detection.label.into(),
            prediction_colors::robocup_object(detection.label),
        );
    }
}
