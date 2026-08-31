use std::time::{Duration, Instant};

use color_eyre::Report;
use eframe::egui::{Color32, Context, Ui};
use ros_z::pubsub::PublicationId;

use crate::repaint::ObservationContext;

use super::image_overlay::{ImageBadgeLine, ImageOverlayPainter, OverlayObservation};

const FRAME_BUDGET: Duration = Duration::from_nanos(1_000_000_000 / 60);
const WARNING_THRESHOLD: Duration = Duration::from_millis(15);
const STALE_AFTER: Duration = Duration::from_millis(500);
const EMA_ALPHA: f64 = 0.1;

const HEALTHY_COLOR: Color32 = Color32::from_rgb(80, 220, 120);
const WARNING_COLOR: Color32 = Color32::from_rgb(255, 170, 40);
const OVER_BUDGET_COLOR: Color32 = Color32::from_rgb(255, 80, 80);
const STALE_COLOR: Color32 = Color32::from_gray(150);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LatencyStatus {
    Healthy,
    Warning,
    OverBudget,
}

impl LatencyStatus {
    fn for_duration(duration: Duration) -> Self {
        if duration < WARNING_THRESHOLD {
            Self::Healthy
        } else if duration <= FRAME_BUDGET {
            Self::Warning
        } else {
            Self::OverBudget
        }
    }

    fn color(self) -> Color32 {
        match self {
            Self::Healthy => HEALTHY_COLOR,
            Self::Warning => WARNING_COLOR,
            Self::OverBudget => OVER_BUDGET_COLOR,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LatencySnapshot {
    raw_inference: Duration,
    total_detection: Duration,
    received_at: Instant,
}

impl LatencySnapshot {
    fn new(
        raw_inference: Duration,
        post_processing: Duration,
        non_maximum_suppression: Duration,
        received_at: Instant,
    ) -> Self {
        Self {
            raw_inference,
            total_detection: raw_inference
                .saturating_add(post_processing)
                .saturating_add(non_maximum_suppression),
            received_at,
        }
    }

    fn is_stale(self, now: Instant) -> bool {
        now.saturating_duration_since(self.received_at) >= STALE_AFTER
    }

    fn update_ema(self, latest: Self) -> Self {
        Self {
            raw_inference: exponential_moving_average(self.raw_inference, latest.raw_inference),
            total_detection: exponential_moving_average(
                self.total_detection,
                latest.total_detection,
            ),
            received_at: latest.received_at,
        }
    }

    fn remaining_freshness(self, now: Instant) -> Option<Duration> {
        STALE_AFTER.checked_sub(now.saturating_duration_since(self.received_at))
    }
}

struct ActiveDiagnostic {
    inference: OverlayObservation<Duration>,
    post_processing: OverlayObservation<Duration>,
    non_maximum_suppression: OverlayObservation<Duration>,
    latest_nms_publication: Option<PublicationId>,
    snapshot: Option<LatencySnapshot>,
}

impl ActiveDiagnostic {
    fn new<C>(context: &C) -> Result<Self, Report>
    where
        C: ObservationContext,
    {
        Ok(Self {
            inference: OverlayObservation::new(context, "inference_duration")?,
            post_processing: OverlayObservation::new(context, "post_processing_duration")?,
            non_maximum_suppression: OverlayObservation::new(
                context,
                "non_maximum_suppression_duration",
            )?,
            latest_nms_publication: None,
            snapshot: None,
        })
    }

    fn refresh(&mut self, now: Instant) {
        let Some(non_maximum_suppression) = self.non_maximum_suppression.latest() else {
            return;
        };
        if self.latest_nms_publication == Some(non_maximum_suppression.publication_id) {
            return;
        }
        let (Some(inference), Some(post_processing)) =
            (self.inference.latest(), self.post_processing.latest())
        else {
            return;
        };

        let latest = LatencySnapshot::new(
            inference.value,
            post_processing.value,
            non_maximum_suppression.value,
            now,
        );
        self.snapshot = Some(match self.snapshot {
            Some(previous) => previous.update_ema(latest),
            None => latest,
        });
        self.latest_nms_publication = Some(non_maximum_suppression.publication_id);
    }
}

enum DiagnosticState {
    Inactive,
    Active(ActiveDiagnostic),
    Error(String),
}

pub(super) struct DetectionLatencyDiagnostic {
    state: DiagnosticState,
}

impl DetectionLatencyDiagnostic {
    pub(super) fn new<C>(active: bool, context: &C) -> Self
    where
        C: ObservationContext,
    {
        let mut diagnostic = Self::inactive();
        diagnostic.set_active(active, context);
        diagnostic
    }

    pub(super) fn inactive() -> Self {
        Self {
            state: DiagnosticState::Inactive,
        }
    }

    pub(super) fn set_active<C>(&mut self, active: bool, context: &C)
    where
        C: ObservationContext,
    {
        let was_active = !matches!(self.state, DiagnosticState::Inactive);
        if active == was_active {
            return;
        }

        self.state = if active {
            match ActiveDiagnostic::new(context) {
                Ok(diagnostic) => DiagnosticState::Active(diagnostic),
                Err(error) => DiagnosticState::Error(format!("Detection latency: {error:#}")),
            }
        } else {
            DiagnosticState::Inactive
        };
    }

    pub(super) fn refresh(&mut self, context: &Context) {
        let DiagnosticState::Active(diagnostic) = &mut self.state else {
            return;
        };

        let now = Instant::now();
        diagnostic.refresh(now);
        if let Some(remaining) = diagnostic
            .snapshot
            .and_then(|snapshot| snapshot.remaining_freshness(now))
        {
            context.request_repaint_after(remaining);
        }
    }

    pub(super) fn show_error(&self, ui: &mut Ui) {
        if let DiagnosticState::Error(error) = &self.state {
            ui.colored_label(ui.visuals().error_fg_color, error);
        }
    }

    pub(super) fn paint(&self, painter: &ImageOverlayPainter) {
        let DiagnosticState::Active(diagnostic) = &self.state else {
            return;
        };
        let Some(snapshot) = diagnostic.snapshot else {
            painter.badge(&[ImageBadgeLine::new(
                "Detection latency",
                "waiting",
                STALE_COLOR,
            )]);
            return;
        };

        let stale = snapshot.is_stale(Instant::now());
        let raw_color = latency_color(snapshot.raw_inference, stale);
        let total_color = latency_color(snapshot.total_detection, stale);
        let raw_label = if stale {
            "Raw inference latency (EMA, stale)"
        } else {
            "Raw inference latency (EMA)"
        };
        let total_label = if stale {
            "Total detection latency (EMA, stale)"
        } else {
            "Total detection latency (EMA)"
        };
        painter.badge(&[
            ImageBadgeLine::new(raw_label, format_latency(snapshot.raw_inference), raw_color),
            ImageBadgeLine::new(
                total_label,
                format_latency(snapshot.total_detection),
                total_color,
            ),
        ]);
    }
}

fn exponential_moving_average(previous: Duration, latest: Duration) -> Duration {
    Duration::from_secs_f64(
        previous.as_secs_f64() * (1.0 - EMA_ALPHA) + latest.as_secs_f64() * EMA_ALPHA,
    )
}

fn latency_color(duration: Duration, stale: bool) -> Color32 {
    if stale {
        STALE_COLOR
    } else {
        LatencyStatus::for_duration(duration).color()
    }
}

fn format_latency(duration: Duration) -> String {
    format!("{:.2} ms", duration.as_secs_f64() * 1_000.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latency_status_uses_ten_percent_headroom_and_sixty_hertz_budget() {
        assert_eq!(
            LatencyStatus::for_duration(WARNING_THRESHOLD - Duration::from_nanos(1)),
            LatencyStatus::Healthy
        );
        assert_eq!(
            LatencyStatus::for_duration(WARNING_THRESHOLD),
            LatencyStatus::Warning
        );
        assert_eq!(
            LatencyStatus::for_duration(FRAME_BUDGET),
            LatencyStatus::Warning
        );
        assert_eq!(
            LatencyStatus::for_duration(FRAME_BUDGET + Duration::from_nanos(1)),
            LatencyStatus::OverBudget
        );
    }

    #[test]
    fn snapshot_sums_the_complete_detection_latency() {
        let snapshot = LatencySnapshot::new(
            Duration::from_millis(12),
            Duration::from_millis(2),
            Duration::from_millis(1),
            Instant::now(),
        );

        assert_eq!(snapshot.raw_inference, Duration::from_millis(12));
        assert_eq!(snapshot.total_detection, Duration::from_millis(15));
    }

    #[test]
    fn snapshot_becomes_stale_after_half_a_second() {
        let received_at = Instant::now();
        let snapshot =
            LatencySnapshot::new(Duration::ZERO, Duration::ZERO, Duration::ZERO, received_at);

        assert!(!snapshot.is_stale(received_at + STALE_AFTER - Duration::from_nanos(1)));
        assert!(snapshot.is_stale(received_at + STALE_AFTER));
    }

    #[test]
    fn snapshot_uses_independent_exponential_moving_averages() {
        let received_at = Instant::now();
        let previous = LatencySnapshot::new(
            Duration::from_millis(10),
            Duration::from_millis(4),
            Duration::from_millis(1),
            received_at,
        );
        let latest = LatencySnapshot::new(
            Duration::from_millis(20),
            Duration::from_millis(8),
            Duration::from_millis(2),
            received_at + Duration::from_millis(10),
        );

        let smoothed = previous.update_ema(latest);

        assert_eq!(smoothed.raw_inference, Duration::from_millis(11));
        assert_eq!(smoothed.total_detection, Duration::from_micros(16_500));
        assert_eq!(smoothed.received_at, latest.received_at);
    }

    #[test]
    fn latency_is_formatted_in_milliseconds() {
        assert_eq!(format_latency(Duration::from_micros(14_725)), "14.72 ms");
    }
}
