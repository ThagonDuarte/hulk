use std::{collections::BTreeSet, ops::RangeInclusive};

use detection_replay::{BookmarkCollection, LoadedPredictionRun, Recording};
use eframe::egui::{
    self, Align2, Color32, FontId, Rect, Response, Sense, Stroke, StrokeKind, Ui, pos2, vec2,
};

const LABEL_WIDTH: f32 = 150.0;
const TICK_HEIGHT: f32 = 34.0;
const ROW_HEIGHT: f32 = 28.0;
const MIN_VIEW_NANOS: f64 = 1_000_000.0;

#[derive(Clone, Debug)]
pub struct TimelineState {
    viewport_start: f64,
    viewport_end: f64,
    nominal_frame_duration: f64,
    pub bookmarks: BookmarkCollection,
}

impl TimelineState {
    pub fn new(recording: &Recording, start_frame: usize, end_frame: usize) -> Self {
        let (viewport_start, viewport_end) = full_time_range(recording, start_frame, end_frame);
        Self {
            viewport_start,
            viewport_end,
            nominal_frame_duration: nominal_frame_duration(recording, start_frame, end_frame),
            bookmarks: BookmarkCollection::default(),
        }
    }

    pub fn reset(&mut self, recording: &Recording, start_frame: usize, end_frame: usize) {
        (self.viewport_start, self.viewport_end) =
            full_time_range(recording, start_frame, end_frame);
    }
}

pub fn desired_height(run_count: usize) -> f32 {
    TICK_HEIGHT + ROW_HEIGHT * (run_count + 1) as f32 + 12.0
}

pub struct TimelineResponse {
    pub response: Response,
    pub selected_frame: Option<usize>,
}

pub fn show(
    ui: &mut Ui,
    recording: &Recording,
    runs: &[LoadedPredictionRun],
    frame_range: RangeInclusive<usize>,
    selected_frame: usize,
    state: &mut TimelineState,
    hidden_runs: &BTreeSet<String>,
) -> TimelineResponse {
    let start_frame = *frame_range.start();
    let end_frame = *frame_range.end();
    let visible_run_count = runs
        .iter()
        .filter(|run| !hidden_runs.contains(&run.key))
        .count();
    let desired = vec2(
        ui.available_width().max(1.0),
        desired_height(visible_run_count).max(ui.available_height()),
    );
    let (rect, response) = ui.allocate_exact_size(desired, Sense::click_and_drag());
    let plot = Rect::from_min_max(
        pos2(rect.left() + LABEL_WIDTH, rect.top()),
        rect.right_bottom(),
    );
    let painter = ui.painter_at(rect);

    painter.rect_filled(rect, 3.0, ui.visuals().extreme_bg_color);
    painter.rect_filled(plot, 0.0, Color32::from_gray(22));

    let mut selected = None;
    if response.double_clicked() {
        state.reset(recording, start_frame, end_frame);
    } else if response.hovered() {
        let scroll = ui.input(|input| input.smooth_scroll_delta);
        if scroll.y.abs() > f32::EPSILON && !ui.input(|input| input.modifiers.shift) {
            let anchor = response
                .hover_pos()
                .map_or(0.5, |position| normalized_x(plot, position.x));
            zoom(state, 0.99_f64.powf(f64::from(scroll.y)), anchor);
            clamp_viewport(state, recording, start_frame, end_frame);
        } else {
            let pan_points = f64::from(
                scroll.x
                    + if ui.input(|input| input.modifiers.shift) {
                        scroll.y
                    } else {
                        0.0
                    },
            );
            if pan_points.abs() > f64::EPSILON {
                let nanos_per_point =
                    (state.viewport_end - state.viewport_start) / f64::from(plot.width().max(1.0));
                state.viewport_start -= pan_points * nanos_per_point;
                state.viewport_end -= pan_points * nanos_per_point;
                clamp_viewport(state, recording, start_frame, end_frame);
            }
        }
    }
    if (response.clicked() || response.dragged())
        && let Some(pointer) = response.interact_pointer_pos()
        && plot.contains(pointer)
    {
        selected = Some(frame_at_x(
            recording,
            start_frame,
            end_frame,
            x_to_time(plot, state, pointer.x),
        ));
    }

    paint_ticks(ui, plot, recording, start_frame, selected_frame, state);
    let column_ranges = visible_column_ranges(
        plot,
        recording,
        start_frame,
        end_frame,
        state,
        state.nominal_frame_duration,
    );
    paint_track(
        ui,
        plot,
        0,
        "Source images",
        &column_ranges,
        |_| true,
        Color32::from_rgb(70, 115, 155),
    );
    for (index, run) in runs
        .iter()
        .filter(|run| !hidden_runs.contains(&run.key))
        .enumerate()
    {
        paint_track(
            ui,
            plot,
            index + 1,
            &run.label,
            &column_ranges,
            |range| run.any_available(range),
            run_color(index),
        );
    }

    for (frame, bookmark) in state.bookmarks.0.range(start_frame..=end_frame) {
        let timestamp = recording.frames()[*frame].timestamp_nanos as f64;
        let x = time_to_x(plot, state, timestamp);
        if plot.left() <= x && x <= plot.right() {
            painter.line_segment(
                [pos2(x, plot.top()), pos2(x, plot.bottom())],
                Stroke::new(1.5_f32, Color32::RED),
            );
            painter.text(
                pos2(x + 3.0, plot.top() + 2.0),
                Align2::LEFT_TOP,
                &bookmark.name,
                FontId::proportional(11.0),
                Color32::LIGHT_RED,
            );
        }
    }

    let selected_time = recording.frames()[selected_frame].timestamp_nanos as f64;
    let selected_x = time_to_x(plot, state, selected_time);
    if selected_x < plot.left() || selected_x > plot.right() {
        let x = selected_x.clamp(plot.left(), plot.right());
        let points = if selected_x < plot.left() {
            [
                pos2(x, plot.top()),
                pos2(x + 9.0, plot.top() + 6.0),
                pos2(x, plot.top() + 12.0),
            ]
        } else {
            [
                pos2(x, plot.top()),
                pos2(x - 9.0, plot.top() + 6.0),
                pos2(x, plot.top() + 12.0),
            ]
        };
        painter.add(egui::Shape::convex_polygon(
            points.to_vec(),
            Color32::GREEN,
            Stroke::NONE,
        ));
    } else {
        painter.line_segment(
            [
                pos2(selected_x, plot.top()),
                pos2(selected_x, plot.bottom()),
            ],
            Stroke::new(2.0_f32, Color32::GREEN),
        );
    }
    painter.rect_stroke(
        plot,
        0.0,
        Stroke::new(1.0_f32, ui.visuals().widgets.noninteractive.bg_stroke.color),
        StrokeKind::Inside,
    );

    if response.hovered()
        && let Some(pointer) = response.hover_pos()
        && plot.contains(pointer)
    {
        let frame = frame_at_x(
            recording,
            start_frame,
            end_frame,
            x_to_time(plot, state, pointer.x),
        );
        response.clone().on_hover_ui_at_pointer(|ui| {
            ui.label(format!("frame {frame}"));
            ui.label(format!(
                "{:.3} s",
                relative_seconds(recording, start_frame, frame)
            ));
            for run in runs.iter().filter(|run| !hidden_runs.contains(&run.key)) {
                let available = if run.is_available(frame) {
                    "available"
                } else {
                    "missing"
                };
                ui.label(format!("{}: {available}", run.label));
            }
        });
    }

    TimelineResponse {
        response,
        selected_frame: selected,
    }
}

fn paint_track(
    ui: &Ui,
    plot: Rect,
    row: usize,
    label: &str,
    column_ranges: &[(f32, f32, Option<RangeInclusive<usize>>)],
    available: impl Fn(RangeInclusive<usize>) -> bool,
    color: Color32,
) {
    let top = plot.top() + TICK_HEIGHT + row as f32 * ROW_HEIGHT;
    let rect = Rect::from_min_max(
        pos2(plot.left(), top),
        pos2(plot.right(), top + ROW_HEIGHT - 3.0),
    );
    let label_rect = Rect::from_min_max(
        pos2(plot.left() - LABEL_WIDTH, top),
        pos2(plot.left(), rect.bottom()),
    );
    ui.painter().text(
        label_rect.left_center() + vec2(8.0, 0.0),
        Align2::LEFT_CENTER,
        label,
        FontId::proportional(12.0),
        ui.visuals().text_color(),
    );
    ui.painter().rect_filled(rect, 0.0, Color32::from_gray(30));

    let mut span = None;
    for (left, right, range) in column_ranges {
        if !range.clone().is_some_and(&available) {
            if let Some((left, right)) = span.take() {
                paint_span(ui, rect, left, right, color);
            }
            continue;
        }
        if right > left {
            match &mut span {
                Some((_, span_right)) if *left <= *span_right + 0.5 => {
                    *span_right = span_right.max(*right);
                }
                Some(_) => {
                    let (span_left, span_right) = span.replace((*left, *right)).unwrap();
                    paint_span(ui, rect, span_left, span_right, color);
                }
                None => span = Some((*left, *right)),
            }
        }
    }
    if let Some((left, right)) = span {
        paint_span(ui, rect, left, right, color);
    }
}

fn visible_column_ranges(
    plot: Rect,
    recording: &Recording,
    start_frame: usize,
    end_frame: usize,
    state: &TimelineState,
    frame_duration: f64,
) -> Vec<(f32, f32, Option<RangeInclusive<usize>>)> {
    let frames = recording.frames();
    let visible = &frames[start_frame..=end_frame];
    let columns = plot.width().ceil().max(1.0) as usize;
    (0..columns)
        .map(|column| {
            let left = plot.left() + column as f32;
            let right = (left + 1.0).min(plot.right());
            let column_start = x_to_time(plot, state, left);
            let column_end = x_to_time(plot, state, right);
            let search_start = column_start - frame_duration * 1.1;
            let mut first = visible
                .partition_point(|frame| (frame.timestamp_nanos as f64) < search_start)
                .saturating_add(start_frame);
            let last = visible
                .partition_point(|frame| frame.timestamp_nanos as f64 <= column_end)
                .saturating_sub(1)
                .saturating_add(start_frame)
                .min(end_frame);
            while first <= last && first <= end_frame {
                let frame_start = frames[first].timestamp_nanos as f64;
                let next_start = if first < end_frame {
                    frames[first + 1].timestamp_nanos as f64
                } else {
                    frame_start + frame_duration
                };
                let frame_end = next_start.min(frame_start + frame_duration * 1.1);
                if frame_end >= column_start {
                    break;
                }
                first += 1;
            }
            let range = (first <= last && first <= end_frame).then_some(first..=last);
            (left, right, range)
        })
        .collect()
}

fn paint_span(ui: &Ui, row: Rect, left: f32, right: f32, color: Color32) {
    ui.painter().rect_filled(
        Rect::from_min_max(
            pos2(left, row.top()),
            pos2(right.max(left + 1.0), row.bottom()),
        ),
        0.0,
        color,
    );
}

fn paint_ticks(
    ui: &Ui,
    plot: Rect,
    recording: &Recording,
    start_frame: usize,
    selected_frame: usize,
    state: &TimelineState,
) {
    let span_seconds = (state.viewport_end - state.viewport_start) / 1.0e9;
    let target_ticks = (plot.width() / 110.0).max(1.0) as usize;
    let spacing = nice_spacing(span_seconds / target_ticks as f64) * 1.0e9;
    let first = (state.viewport_start / spacing).floor() * spacing;
    let base = recording.frames()[start_frame].timestamp_nanos as f64;
    let mut tick = first;
    while tick <= state.viewport_end + spacing {
        let x = time_to_x(plot, state, tick);
        if plot.left() <= x && x <= plot.right() {
            ui.painter().line_segment(
                [pos2(x, plot.top() + 19.0), pos2(x, plot.bottom())],
                Stroke::new(1.0_f32, ui.visuals().weak_text_color()),
            );
            ui.painter().text(
                pos2(x, plot.top() + 2.0),
                Align2::CENTER_TOP,
                format_time((tick - base) / 1.0e9),
                FontId::proportional(11.0),
                ui.visuals().text_color(),
            );
        }
        tick += spacing;
    }
    let selected = relative_seconds(recording, start_frame, selected_frame);
    let x = time_to_x(
        plot,
        state,
        recording.frames()[selected_frame].timestamp_nanos as f64,
    );
    if plot.left() <= x && x <= plot.right() {
        ui.painter().text(
            pos2(x, plot.top() + 17.0),
            Align2::CENTER_TOP,
            format!("frame {selected_frame} · {}", format_time(selected)),
            FontId::proportional(11.0),
            Color32::GREEN,
        );
    }
}

fn full_time_range(recording: &Recording, start_frame: usize, end_frame: usize) -> (f64, f64) {
    let start = recording.frames()[start_frame].timestamp_nanos as f64;
    let end = if end_frame > start_frame {
        let last = recording.frames()[end_frame].timestamp_nanos as f64;
        last + (last - recording.frames()[end_frame - 1].timestamp_nanos as f64).max(1.0)
    } else {
        start + MIN_VIEW_NANOS
    };
    (start, end.max(start + MIN_VIEW_NANOS))
}

fn nominal_frame_duration(recording: &Recording, start_frame: usize, end_frame: usize) -> f64 {
    if start_frame == end_frame {
        return MIN_VIEW_NANOS;
    }
    let mut intervals = (start_frame..end_frame)
        .map(|frame| {
            (recording.frames()[frame + 1].timestamp_nanos
                - recording.frames()[frame].timestamp_nanos)
                .max(1) as f64
        })
        .collect::<Vec<_>>();
    let middle = intervals.len() / 2;
    intervals.select_nth_unstable_by(middle, f64::total_cmp);
    intervals[middle]
}

fn clamp_viewport(
    state: &mut TimelineState,
    recording: &Recording,
    start_frame: usize,
    end_frame: usize,
) {
    let (full_start, full_end) = full_time_range(recording, start_frame, end_frame);
    let full_width = full_end - full_start;
    let width = (state.viewport_end - state.viewport_start)
        .clamp(MIN_VIEW_NANOS.min(full_width), full_width);
    if state.viewport_start < full_start {
        state.viewport_start = full_start;
    }
    if state.viewport_start + width > full_end {
        state.viewport_start = full_end - width;
    }
    state.viewport_end = state.viewport_start + width;
}

fn zoom(state: &mut TimelineState, factor: f64, anchor: f32) {
    let width = state.viewport_end - state.viewport_start;
    let anchor_time = state.viewport_start + width * f64::from(anchor);
    let new_width = width * factor;
    state.viewport_start = anchor_time - new_width * f64::from(anchor);
    state.viewport_end = state.viewport_start + new_width;
}

fn frame_at_x(recording: &Recording, start: usize, end: usize, timestamp: f64) -> usize {
    let frames = &recording.frames()[start..=end];
    match frames.binary_search_by(|frame| (frame.timestamp_nanos as f64).total_cmp(&timestamp)) {
        Ok(index) => start + index,
        Err(0) => start,
        Err(index) if index == frames.len() => end,
        Err(index) => {
            let before = frames[index - 1].timestamp_nanos as f64;
            let after = frames[index].timestamp_nanos as f64;
            start
                + if timestamp - before <= after - timestamp {
                    index - 1
                } else {
                    index
                }
        }
    }
}

fn normalized_x(rect: Rect, x: f32) -> f32 {
    ((x - rect.left()) / rect.width().max(1.0)).clamp(0.0, 1.0)
}

fn x_to_time(rect: Rect, state: &TimelineState, x: f32) -> f64 {
    state.viewport_start
        + (state.viewport_end - state.viewport_start) * f64::from(normalized_x(rect, x))
}

fn time_to_x(rect: Rect, state: &TimelineState, time: f64) -> f32 {
    rect.left()
        + rect.width()
            * ((time - state.viewport_start) / (state.viewport_end - state.viewport_start)) as f32
}

fn relative_seconds(recording: &Recording, start: usize, frame: usize) -> f64 {
    (recording.frames()[frame].timestamp_nanos - recording.frames()[start].timestamp_nanos) as f64
        / 1.0e9
}

fn nice_spacing(raw: f64) -> f64 {
    if !raw.is_finite() || raw <= 0.0 {
        return 1.0;
    }
    let magnitude = 10.0_f64.powf(raw.log10().floor());
    let normalized = raw / magnitude;
    let step = if normalized <= 1.0 {
        1.0
    } else if normalized <= 2.0 {
        2.0
    } else if normalized <= 5.0 {
        5.0
    } else {
        10.0
    };
    step * magnitude
}

fn format_time(seconds: f64) -> String {
    if seconds.abs() >= 60.0 {
        format!(
            "{}:{:06.3}",
            (seconds / 60.0).floor() as i64,
            seconds.abs() % 60.0
        )
    } else {
        format!("{seconds:.3}s")
    }
}

fn run_color(index: usize) -> Color32 {
    const COLORS: [Color32; 8] = [
        Color32::from_rgb(76, 201, 240),
        Color32::from_rgb(255, 166, 43),
        Color32::from_rgb(255, 89, 123),
        Color32::from_rgb(93, 230, 129),
        Color32::from_rgb(179, 136, 255),
        Color32::from_rgb(255, 232, 92),
        Color32::from_rgb(114, 239, 221),
        Color32::from_rgb(240, 120, 210),
    ];
    COLORS[index % COLORS.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tick_spacing_uses_one_two_five_steps() {
        assert_eq!(nice_spacing(0.8), 1.0);
        assert_eq!(nice_spacing(1.2), 2.0);
        assert_eq!(nice_spacing(3.0), 5.0);
        assert_eq!(nice_spacing(8.0), 10.0);
    }

    #[test]
    fn bookmarks_wrap_within_range() {
        let mut bookmarks = BookmarkCollection::default();
        bookmarks.toggle(12);
        bookmarks.toggle(18);
        assert_eq!(bookmarks.next(12, 10, 20), Some(18));
        assert_eq!(bookmarks.next(18, 10, 20), Some(12));
        assert_eq!(bookmarks.previous(12, 10, 20), Some(18));
        assert_eq!(bookmarks.next(20, 10, 20), Some(12));
        assert_eq!(bookmarks.previous(20, 10, 20), Some(18));

        let empty = BookmarkCollection::default();
        assert_eq!(empty.next(10, 10, 10), None);
        assert_eq!(empty.previous(10, 10, 10), None);
    }

    #[test]
    fn zoom_keeps_anchor_time_stationary() {
        let mut state = TimelineState {
            viewport_start: 100.0,
            viewport_end: 200.0,
            nominal_frame_duration: 10.0,
            bookmarks: BookmarkCollection::default(),
        };
        zoom(&mut state, 0.5, 0.25);
        assert_eq!(state.viewport_start, 112.5);
        assert_eq!(state.viewport_end, 162.5);
    }
}
