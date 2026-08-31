use std::{sync::Arc, time::Duration};

use color_eyre::{Report, eyre::Context as _};
use coordinate_systems::Pixel;
use eframe::egui::{
    Align2, Color32, CornerRadius, DragValue, FontId, Painter, Pos2, Rect, Stroke, Ui, pos2, vec2,
};
use linear_algebra::{Point2, point};
use ros_z::{Message, time::Time};
use ros_z_debug::{RetentionPolicy, SampleRecord, TopicObservation};
use serde_json::{Value, json};
use types::time_wrapper::TimeWrapper;

use crate::repaint::{ObservationContext, ObservationRepaint, RepaintOnUpdates};

use super::detection_latency::DetectionLatencyDiagnostic;
use super::overlays::{
    BallDetectionOverlay, FieldBorderOverlay, HorizonOverlay, LineDetectionOverlay,
    ObjectDetectionOverlay, PoseDetectionOverlay, RobotPoseDetectionOverlay,
};

const OVERLAY_RETENTION_WINDOW: Duration = Duration::from_secs(2);
const DEFAULT_CONFIDENCE_THRESHOLD: f32 = 0.5;

pub(super) struct ImageOverlays {
    line_detection: OverlaySlot<LineDetectionOverlay>,
    ball_detection: OverlaySlot<BallDetectionOverlay>,
    horizon: OverlaySlot<HorizonOverlay>,
    field_border: OverlaySlot<FieldBorderOverlay>,
    object_detection: OverlaySlot<ObjectDetectionOverlay>,
    pose_detection: OverlaySlot<PoseDetectionOverlay>,
    robot_pose_detection: OverlaySlot<RobotPoseDetectionOverlay>,
    detection_latency: DetectionLatencyDiagnostic,
}

impl ImageOverlays {
    pub(super) fn new<C>(value: Option<&Value>, context: &C) -> Self
    where
        C: ObservationContext,
    {
        let object_detection = OverlaySlot::new(value, context);
        let pose_detection = OverlaySlot::new(value, context);
        let robot_pose_detection = OverlaySlot::new(value, context);
        let detection_latency = DetectionLatencyDiagnostic::new(
            object_detection.active || pose_detection.active || robot_pose_detection.active,
            context,
        );
        Self {
            line_detection: OverlaySlot::new(value, context),
            ball_detection: OverlaySlot::new(value, context),
            horizon: OverlaySlot::new(value, context),
            field_border: OverlaySlot::new(value, context),
            object_detection,
            pose_detection,
            robot_pose_detection,
            detection_latency,
        }
    }

    pub(super) fn ui<C>(&mut self, ui: &mut Ui, context: &C)
    where
        C: ObservationContext,
    {
        ui.menu_button("Overlays", |ui| {
            self.line_detection.checkbox(ui, context);
            self.ball_detection.checkbox(ui, context);
            self.horizon.checkbox(ui, context);
            self.field_border.checkbox(ui, context);
            self.object_detection.checkbox(ui, context);
            self.pose_detection.checkbox(ui, context);
            self.robot_pose_detection.checkbox(ui, context);
            self.detection_latency
                .set_active(self.has_active_detection_overlays(), context);
            self.detection_latency.show_error(ui);
        });
        self.detection_latency.refresh(ui.ctx());
    }

    pub(super) fn paint(&self, painter: &ImageOverlayPainter, image_time: Time) {
        self.line_detection.paint(painter, image_time);
        self.ball_detection.paint(painter, image_time);
        self.horizon.paint(painter, image_time);
        self.field_border.paint(painter, image_time);
        self.object_detection.paint(painter, image_time);
        self.pose_detection.paint(painter, image_time);
        self.robot_pose_detection.paint(painter, image_time);
        self.detection_latency.paint(painter);
    }

    pub(super) fn preferred_image_time(&self) -> Option<Time> {
        [
            self.object_detection.latest_time(),
            self.pose_detection.latest_time(),
            self.robot_pose_detection.latest_time(),
        ]
        .into_iter()
        .flatten()
        .min()
    }

    pub(super) fn save(&self) -> Value {
        json!({
            LineDetectionOverlay::STORAGE_KEY: self.line_detection.save(),
            BallDetectionOverlay::STORAGE_KEY: self.ball_detection.save(),
            HorizonOverlay::STORAGE_KEY: self.horizon.save(),
            FieldBorderOverlay::STORAGE_KEY: self.field_border.save(),
            ObjectDetectionOverlay::STORAGE_KEY: self.object_detection.save(),
            PoseDetectionOverlay::STORAGE_KEY: self.pose_detection.save(),
            RobotPoseDetectionOverlay::STORAGE_KEY: self.robot_pose_detection.save(),
        })
    }

    fn has_active_detection_overlays(&self) -> bool {
        self.object_detection.active
            || self.pose_detection.active
            || self.robot_pose_detection.active
    }
}

impl Default for ImageOverlays {
    fn default() -> Self {
        Self {
            line_detection: OverlaySlot::inactive(),
            ball_detection: OverlaySlot::inactive(),
            horizon: OverlaySlot::inactive(),
            field_border: OverlaySlot::inactive(),
            object_detection: OverlaySlot::inactive(),
            pose_detection: OverlaySlot::inactive(),
            robot_pose_detection: OverlaySlot::inactive(),
            detection_latency: DetectionLatencyDiagnostic::inactive(),
        }
    }
}

struct OverlaySlot<T> {
    active: bool,
    overlay: Option<T>,
    error: Option<String>,
    confidence_thresholds: Vec<f32>,
}

impl<T> OverlaySlot<T>
where
    T: ImageOverlay,
{
    fn new<C>(value: Option<&Value>, context: &C) -> Self
    where
        C: ObservationContext,
    {
        let mut slot = Self::inactive();
        let overlay_value = value.and_then(|value| value.get(T::STORAGE_KEY));
        slot.active = overlay_value
            .and_then(|value| value.get("active"))
            .and_then(Value::as_bool)
            .unwrap_or(false);
        for (threshold, definition) in slot
            .confidence_thresholds
            .iter_mut()
            .zip(T::CONFIDENCE_THRESHOLDS)
        {
            *threshold = overlay_value
                .and_then(|value| value.get(definition.storage_key))
                .and_then(Value::as_f64)
                .map(|value| value as f32)
                .unwrap_or(DEFAULT_CONFIDENCE_THRESHOLD)
                .clamp(0.0, 1.0);
        }
        if slot.active {
            slot.recreate(context);
        }
        slot
    }

    fn inactive() -> Self {
        Self {
            active: false,
            overlay: None,
            error: None,
            confidence_thresholds: vec![
                DEFAULT_CONFIDENCE_THRESHOLD;
                T::CONFIDENCE_THRESHOLDS.len()
            ],
        }
    }

    fn checkbox<C>(&mut self, ui: &mut Ui, context: &C)
    where
        C: ObservationContext,
    {
        let changed = ui.checkbox(&mut self.active, T::NAME).changed();
        if changed {
            if self.active {
                self.recreate(context);
            } else {
                self.overlay = None;
                self.error = None;
            }
        }
        if self.active && !T::CONFIDENCE_THRESHOLDS.is_empty() {
            ui.indent(T::STORAGE_KEY, |ui| {
                for (threshold, definition) in self
                    .confidence_thresholds
                    .iter_mut()
                    .zip(T::CONFIDENCE_THRESHOLDS)
                {
                    ui.horizontal(|ui| {
                        ui.label(definition.label);
                        ui.add(
                            DragValue::new(threshold)
                                .range(0.0..=1.0)
                                .speed(0.01)
                                .fixed_decimals(2),
                        );
                    });
                }
            });
        }
        if let Some(error) = &self.error {
            ui.colored_label(ui.visuals().error_fg_color, error);
        }
    }

    fn recreate<C>(&mut self, context: &C)
    where
        C: ObservationContext,
    {
        match T::new(context) {
            Ok(overlay) => {
                self.overlay = Some(overlay);
                self.error = None;
            }
            Err(error) => {
                self.overlay = None;
                self.error = Some(format!("{}: {error:#}", T::NAME));
            }
        }
    }

    fn paint(&self, painter: &ImageOverlayPainter, image_time: Time) {
        if let Some(overlay) = &self.overlay {
            overlay.paint(painter, image_time, &self.confidence_thresholds);
        }
    }

    fn latest_time(&self) -> Option<Time> {
        self.overlay.as_ref().and_then(ImageOverlay::latest_time)
    }

    fn save(&self) -> Value {
        let mut value = serde_json::Map::new();
        value.insert("active".to_string(), json!(self.active));
        for (threshold, definition) in self
            .confidence_thresholds
            .iter()
            .zip(T::CONFIDENCE_THRESHOLDS)
        {
            value.insert(definition.storage_key.to_string(), json!(threshold));
        }
        Value::Object(value)
    }
}

pub(super) struct ConfidenceThresholdDefinition {
    label: &'static str,
    storage_key: &'static str,
}

impl ConfidenceThresholdDefinition {
    pub(super) const fn new(label: &'static str, storage_key: &'static str) -> Self {
        Self { label, storage_key }
    }
}

pub(super) trait ImageOverlay: Sized {
    const NAME: &'static str;
    const STORAGE_KEY: &'static str;
    const CONFIDENCE_THRESHOLDS: &'static [ConfidenceThresholdDefinition] = &[];

    fn new<C>(context: &C) -> Result<Self, Report>
    where
        C: ObservationContext;

    fn paint(&self, painter: &ImageOverlayPainter, image_time: Time, confidence_thresholds: &[f32]);

    fn latest_time(&self) -> Option<Time> {
        None
    }
}

pub(super) struct OverlayObservation<T> {
    observation: TopicObservation<T>,
    _repaint: ObservationRepaint,
}

impl<T> OverlayObservation<T>
where
    T: Message + Send + Sync + 'static,
    T::Codec: Send + Sync,
{
    pub(super) fn new<C>(context: &C, topic: &str) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        let (observation, repaint) = create_typed_observation(context, topic)?;
        Ok(Self {
            observation,
            _repaint: repaint,
        })
    }

    pub(super) fn latest(&self) -> Option<Arc<SampleRecord<T>>> {
        self.observation.latest()
    }

    fn get_all(&self) -> Vec<Arc<SampleRecord<T>>> {
        self.observation.get_all()
    }
}

impl<T> OverlayObservation<TimeWrapper<T>>
where
    TimeWrapper<T>: Message + Send + Sync + 'static,
    <TimeWrapper<T> as Message>::Codec: Send + Sync,
{
    pub(super) fn latest_time(&self) -> Option<Time> {
        self.latest().map(|record| record.value.time)
    }

    pub(super) fn nearest_to_time(
        &self,
        time: Time,
        tolerance: Duration,
    ) -> Option<Arc<SampleRecord<TimeWrapper<T>>>> {
        let nearest = self
            .get_all()
            .into_iter()
            .min_by_key(|record| time_distance(record.value.time, time))?;
        (time_distance(nearest.value.time, time) <= tolerance).then_some(nearest)
    }

    pub(super) fn at_time(&self, time: Time) -> Option<Arc<SampleRecord<TimeWrapper<T>>>> {
        self.get_all()
            .into_iter()
            .rev()
            .find(|record| record.value.time == time)
    }
}

fn time_distance(first: Time, second: Time) -> Duration {
    first
        .duration_since(second)
        .max(second.duration_since(first))
}

fn create_typed_observation<T>(
    context: &impl ObservationContext,
    topic: &str,
) -> Result<(TopicObservation<T>, ObservationRepaint), Report>
where
    T: Message + Send + Sync + 'static,
    T::Codec: Send + Sync,
{
    let runtime_handle = context.backend().runtime_handle().clone();
    // ros_z_debug spawns observation tasks internally and needs a current runtime.
    let _runtime_context = runtime_handle.enter();
    let observation = context
        .backend()
        .observer()
        .observe_typed::<T>(topic)
        .wrap_err_with(|| format!("failed to create typed topic observation for {topic}"))?
        .retention(RetentionPolicy::time_window(OVERLAY_RETENTION_WINDOW)?)
        .spawn();
    let repaint = observation.repaint_on_updates(context);
    Ok((observation, repaint))
}

pub(super) struct ImageOverlayPainter {
    painter: Painter,
    rect: Rect,
    image_size: [usize; 2],
    scale: f32,
}

impl ImageOverlayPainter {
    pub(super) fn new(painter: Painter, rect: Rect, image_size: [usize; 2]) -> Self {
        let scale_x = rect.width() / image_size[0].max(1) as f32;
        let scale_y = rect.height() / image_size[1].max(1) as f32;
        Self {
            painter,
            rect,
            image_size,
            scale: scale_x.min(scale_y),
        }
    }

    pub(super) fn image_width(&self) -> f32 {
        self.image_size[0] as f32
    }

    fn position(&self, point: Point2<Pixel>) -> Pos2 {
        let scale_x = self.rect.width() / self.image_size[0].max(1) as f32;
        let scale_y = self.rect.height() / self.image_size[1].max(1) as f32;
        pos2(
            self.rect.left() + point.x() * scale_x,
            self.rect.top() + point.y() * scale_y,
        )
    }

    fn stroke(&self, stroke: Stroke) -> Stroke {
        Stroke {
            width: stroke.width * self.scale,
            ..stroke
        }
    }

    pub(super) fn line_segment(&self, start: Point2<Pixel>, end: Point2<Pixel>, stroke: Stroke) {
        self.painter.line_segment(
            [self.position(start), self.position(end)],
            self.stroke(stroke),
        );
    }

    pub(super) fn rect_stroke(&self, min: Point2<Pixel>, max: Point2<Pixel>, stroke: Stroke) {
        let top_right = point![max.x(), min.y()];
        let bottom_left = point![min.x(), max.y()];
        self.line_segment(min, top_right, stroke);
        self.line_segment(top_right, max, stroke);
        self.line_segment(max, bottom_left, stroke);
        self.line_segment(bottom_left, min, stroke);
    }

    pub(super) fn circle_filled(&self, center: Point2<Pixel>, radius: f32, fill_color: Color32) {
        self.painter
            .circle_filled(self.position(center), radius * self.scale, fill_color);
    }

    pub(super) fn circle_stroke(&self, center: Point2<Pixel>, radius: f32, stroke: Stroke) {
        self.painter.circle_stroke(
            self.position(center),
            radius * self.scale,
            self.stroke(stroke),
        );
    }

    pub(super) fn floating_text(
        &self,
        position: Point2<Pixel>,
        align: Align2,
        text: String,
        color: Color32,
    ) {
        self.painter.text(
            self.position(position),
            align,
            text,
            FontId::default(),
            color,
        );
    }

    pub(super) fn badge(&self, lines: &[ImageBadgeLine]) {
        const MARGIN: f32 = 8.0;
        const PADDING: f32 = 7.0;
        const COLUMN_GAP: f32 = 12.0;
        const LINE_GAP: f32 = 3.0;

        let font = FontId::monospace(12.0);
        let label_color = Color32::from_gray(220);
        let laid_out_lines = lines
            .iter()
            .map(|line| {
                let label =
                    self.painter
                        .layout_no_wrap(line.label.to_string(), font.clone(), label_color);
                let value =
                    self.painter
                        .layout_no_wrap(line.value.clone(), font.clone(), line.value_color);
                let height = label.size().y.max(value.size().y);
                (label, value, height)
            })
            .collect::<Vec<_>>();
        let label_width = laid_out_lines
            .iter()
            .map(|(label, _, _)| label.size().x)
            .fold(0.0, f32::max);
        let value_width = laid_out_lines
            .iter()
            .map(|(_, value, _)| value.size().x)
            .fold(0.0, f32::max);
        let content_height = laid_out_lines
            .iter()
            .map(|(_, _, height)| *height)
            .sum::<f32>()
            + LINE_GAP * lines.len().saturating_sub(1) as f32;
        let badge_size = vec2(
            PADDING * 2.0 + label_width + COLUMN_GAP + value_width,
            PADDING * 2.0 + content_height,
        );
        let badge_rect =
            Rect::from_min_size(self.rect.left_top() + vec2(MARGIN, MARGIN), badge_size);
        self.painter.rect_filled(
            badge_rect,
            CornerRadius::same(5),
            Color32::from_black_alpha(205),
        );

        let mut y = badge_rect.top() + PADDING;
        for (label, value, height) in laid_out_lines {
            let label_position = pos2(
                badge_rect.left() + PADDING,
                y + (height - label.size().y) / 2.0,
            );
            let value_position = pos2(
                badge_rect.left() + PADDING + label_width + COLUMN_GAP,
                y + (height - value.size().y) / 2.0,
            );
            self.painter.galley(label_position, label, label_color);
            self.painter.galley(value_position, value, Color32::WHITE);
            y += height + LINE_GAP;
        }
    }
}

pub(super) struct ImageBadgeLine {
    label: &'static str,
    value: String,
    value_color: Color32,
}

impl ImageBadgeLine {
    pub(super) fn new(label: &'static str, value: impl Into<String>, value_color: Color32) -> Self {
        Self {
            label,
            value: value.into(),
            value_color,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_model_output_overlays_activate_detection_latency() {
        let mut overlays = ImageOverlays::default();
        assert!(!overlays.has_active_detection_overlays());

        overlays.ball_detection.active = true;
        overlays.line_detection.active = true;
        assert!(!overlays.has_active_detection_overlays());

        overlays.object_detection.active = true;
        assert!(overlays.has_active_detection_overlays());
        overlays.object_detection.active = false;

        overlays.pose_detection.active = true;
        assert!(overlays.has_active_detection_overlays());
        overlays.pose_detection.active = false;

        overlays.robot_pose_detection.active = true;
        assert!(overlays.has_active_detection_overlays());
    }

    #[test]
    fn detection_overlays_have_five_default_confidence_thresholds() {
        let overlays = ImageOverlays::default();
        let thresholds = overlays
            .object_detection
            .confidence_thresholds
            .iter()
            .chain(&overlays.pose_detection.confidence_thresholds)
            .chain(&overlays.robot_pose_detection.confidence_thresholds)
            .copied()
            .collect::<Vec<_>>();

        assert_eq!(thresholds, vec![0.5; 5]);
    }
}
