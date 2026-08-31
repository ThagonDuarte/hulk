use color_eyre::Report;
use eframe::egui::{Align2, Color32, Stroke};
use ros_z::time::Time;
use types::{
    object_detection::{Object, RobocupObjectLabel},
    time_wrapper::TimeWrapper,
};

use crate::repaint::ObservationContext;

use super::super::image_overlay::{
    ConfidenceThresholdDefinition, ImageOverlay, ImageOverlayPainter, OverlayObservation,
};

const OBJECT_CONFIDENCE_THRESHOLDS: [ConfidenceThresholdDefinition; 1] =
    [ConfidenceThresholdDefinition::new(
        "Confidence",
        "confidence_threshold",
    )];

pub(in crate::panels::image) struct ObjectDetectionOverlay {
    object_detections: OverlayObservation<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>,
}

impl ImageOverlay for ObjectDetectionOverlay {
    const NAME: &'static str = "Object Detection";
    const STORAGE_KEY: &'static str = "object_detection";
    const CONFIDENCE_THRESHOLDS: &'static [ConfidenceThresholdDefinition] =
        &OBJECT_CONFIDENCE_THRESHOLDS;

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
        confidence_thresholds: &[f32],
    ) {
        let Some(object_detections) = self.object_detections.at_time(image_time) else {
            return;
        };
        paint_bounding_boxes(
            painter,
            &object_detections.value.inner,
            confidence_thresholds[0],
            Color32::LIGHT_RED,
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
    line_color: Color32,
) {
    for detection in detections {
        let bounding_box = detection.bounding_box;
        if bounding_box.confidence < confidence_threshold {
            continue;
        }
        painter.rect_stroke(
            bounding_box.area.min,
            bounding_box.area.max,
            Stroke::new(1.0, line_color),
        );
        painter.floating_text(
            bounding_box.area.min,
            Align2::RIGHT_BOTTOM,
            format!("{:.2}", bounding_box.confidence),
            Color32::WHITE,
        );
        painter.floating_text(
            bounding_box.area.max,
            Align2::RIGHT_TOP,
            detection.label.into(),
            Color32::WHITE,
        );
    }
}
