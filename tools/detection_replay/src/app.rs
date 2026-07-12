use std::{
    collections::BTreeMap,
    sync::{Arc, mpsc},
    thread,
    time::{Duration, Instant},
};

use color_eyre::{Result, eyre::eyre};
use detection_replay::{LoadedPredictionRun, ModelRunState, Prediction, Recording};
use eframe::{
    App, Frame, NativeOptions, Renderer,
    egui::{
        self, Color32, ColorImage, FontId, Key, PointerButton, Pos2, Rect, RichText, Sense, Stroke,
        StrokeKind, TextureHandle, TextureOptions, Ui, Vec2, pos2, vec2,
    },
};
use types::object_detection::{Object, RobocupObjectLabel};

const MIN_ZOOM: f32 = 1.0;
const MAX_ZOOM: f32 = 20.0;
const PREFETCH_FRAMES: usize = 12;

pub fn run(recording: Recording, start_frame: usize, end_frame: usize) -> Result<()> {
    let runs = recording.load_runs()?;
    let recording = Arc::new(recording);
    let loader = FrameLoader::new(Arc::clone(&recording))?;
    eframe::run_native(
        "Detection Replay",
        NativeOptions {
            renderer: Renderer::Wgpu,
            ..Default::default()
        },
        Box::new(move |creation_context| {
            Ok(Box::new(ReplayApp::new(
                creation_context,
                recording,
                runs,
                loader,
                start_frame,
                end_frame,
            )))
        }),
    )
    .map_err(|error| eyre!("failed to run detection replay: {error}"))
}

struct ReplayApp {
    recording: Arc<Recording>,
    runs: Vec<LoadedPredictionRun>,
    visible_runs: Vec<bool>,
    available_counts: Vec<usize>,
    selected_frame: usize,
    displayed_frame: Option<usize>,
    texture: Option<TextureHandle>,
    decoded_frames: BTreeMap<usize, ColorImage>,
    loader: FrameLoader,
    load_error: Option<String>,
    is_playing: bool,
    loop_playback: bool,
    playback_speed: f32,
    playback_accumulator: f64,
    last_playback_update: Instant,
    confidence_filter: f32,
    camera_zoom: f32,
    camera_pan: Vec2,
    start_frame: usize,
    end_frame: usize,
}

impl ReplayApp {
    fn new(
        creation_context: &eframe::CreationContext<'_>,
        recording: Arc<Recording>,
        runs: Vec<LoadedPredictionRun>,
        mut loader: FrameLoader,
        start_frame: usize,
        end_frame: usize,
    ) -> Self {
        loader.request(start_frame, end_frame);
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());
        let visible_runs = vec![true; runs.len()];
        let available_counts = runs
            .iter()
            .map(|run| {
                run.predictions
                    .iter()
                    .filter(|value| value.is_some())
                    .count()
            })
            .collect();
        Self {
            recording,
            runs,
            visible_runs,
            available_counts,
            selected_frame: start_frame,
            displayed_frame: None,
            texture: None,
            decoded_frames: BTreeMap::new(),
            loader,
            load_error: None,
            is_playing: false,
            loop_playback: true,
            playback_speed: 1.0,
            playback_accumulator: 0.0,
            last_playback_update: Instant::now(),
            confidence_filter: 0.05,
            camera_zoom: 1.0,
            camera_pan: Vec2::ZERO,
            start_frame,
            end_frame,
        }
    }

    fn select_frame(&mut self, frame: usize) {
        self.selected_frame = frame.clamp(self.start_frame, self.end_frame);
        self.playback_accumulator = 0.0;
        self.last_playback_update = Instant::now();
        if !self.decoded_frames.contains_key(&self.selected_frame) {
            self.loader.request(self.selected_frame, self.end_frame);
        }
    }

    fn poll_loader(&mut self, context: &egui::Context) {
        while let Some(result) = self.loader.try_receive() {
            match result {
                Ok((frame_index, image)) => {
                    self.decoded_frames.insert(frame_index, image);
                }
                Err(error) => self.load_error = Some(error),
            }
        }
        self.display_selected_frame(context);
        let retain_from = self.selected_frame.saturating_sub(1);
        self.decoded_frames
            .retain(|frame, _| *frame >= retain_from && *frame <= self.end_frame);
        if self.displayed_frame != Some(self.selected_frame)
            && !self.decoded_frames.contains_key(&self.selected_frame)
        {
            self.loader.request(self.selected_frame, self.end_frame);
        }
    }

    fn display_selected_frame(&mut self, context: &egui::Context) {
        if self.displayed_frame == Some(self.selected_frame) {
            return;
        }
        let Some(image) = self.decoded_frames.remove(&self.selected_frame) else {
            return;
        };
        let size = image.size;
        if let Some(texture) = &mut self.texture
            && texture.size() == size
        {
            texture.set(image, TextureOptions::LINEAR);
        } else {
            self.texture =
                Some(context.load_texture("detection-replay-frame", image, TextureOptions::LINEAR));
        }
        self.displayed_frame = Some(self.selected_frame);
        self.load_error = None;
    }

    fn advance_playback(&mut self, context: &egui::Context) {
        if !self.is_playing || self.start_frame == self.end_frame {
            self.last_playback_update = Instant::now();
            return;
        }
        let now = Instant::now();
        self.playback_accumulator += now.duration_since(self.last_playback_update).as_secs_f64()
            * f64::from(self.playback_speed);
        self.last_playback_update = now;

        let mut next_frame = self.selected_frame;
        while next_frame < self.end_frame {
            let current = self.recording.frames()[next_frame].timestamp_nanos;
            let next = self.recording.frames()[next_frame + 1].timestamp_nanos;
            let frame_duration = ((next - current).max(1) as f64) / 1.0e9;
            if self.playback_accumulator < frame_duration {
                break;
            }
            self.playback_accumulator -= frame_duration;
            next_frame += 1;
        }
        if next_frame == self.end_frame {
            if self.loop_playback {
                next_frame = self.start_frame;
                self.playback_accumulator = 0.0;
            } else {
                self.is_playing = false;
            }
        }
        if next_frame != self.selected_frame {
            self.selected_frame = next_frame;
            if !self.decoded_frames.contains_key(&next_frame) {
                self.loader.request(next_frame, self.end_frame);
            }
        }
        context.request_repaint_after(Duration::from_millis(5));
    }

    fn handle_keys(&mut self, context: &egui::Context) {
        if context.wants_keyboard_input() {
            return;
        }
        let (toggle, previous, next) = context.input(|input| {
            (
                input.key_pressed(Key::Space),
                input.key_pressed(Key::ArrowLeft) || input.key_pressed(Key::Comma),
                input.key_pressed(Key::ArrowRight) || input.key_pressed(Key::Period),
            )
        });
        if toggle {
            self.is_playing = !self.is_playing;
            self.last_playback_update = Instant::now();
        }
        if previous {
            self.is_playing = false;
            self.select_frame(self.selected_frame.saturating_sub(1).max(self.start_frame));
        }
        if next {
            self.is_playing = false;
            self.select_frame(self.selected_frame.saturating_add(1));
        }
    }

    fn top_panel(&mut self, context: &egui::Context) {
        egui::TopBottomPanel::top("replay-controls").show(context, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Detection Replay");
                if ui
                    .button(if self.is_playing { "Pause" } else { "Play" })
                    .clicked()
                {
                    self.is_playing = !self.is_playing;
                    self.last_playback_update = Instant::now();
                }
                if ui.button("Previous").clicked() {
                    self.is_playing = false;
                    self.select_frame(self.selected_frame.saturating_sub(1).max(self.start_frame));
                }
                if ui.button("Next").clicked() {
                    self.is_playing = false;
                    self.select_frame(self.selected_frame.saturating_add(1));
                }
                ui.checkbox(&mut self.loop_playback, "Loop");
                egui::ComboBox::from_id_salt("playback-speed")
                    .selected_text(format!("{}x", self.playback_speed))
                    .show_ui(ui, |ui| {
                        for speed in [0.25, 0.5, 1.0, 2.0, 4.0, 8.0] {
                            ui.selectable_value(
                                &mut self.playback_speed,
                                speed,
                                format!("{speed}x"),
                            );
                        }
                    });
                ui.separator();
                ui.label(format!(
                    "frame {} ({}..={})",
                    self.selected_frame, self.start_frame, self.end_frame
                ));
                ui.label(format!("{:.3}s", self.relative_time(self.selected_frame)));
            });
            let mut frame = self.selected_frame;
            if ui
                .add(
                    egui::Slider::new(&mut frame, self.start_frame..=self.end_frame)
                        .show_value(false),
                )
                .changed()
            {
                self.is_playing = false;
                self.select_frame(frame);
            }
        });
    }

    fn side_panel(&mut self, context: &egui::Context) {
        egui::SidePanel::left("model-list")
            .resizable(true)
            .default_width(220.0)
            .show(context, |ui| {
                ui.heading("Models");
                ui.add(
                    egui::Slider::new(&mut self.confidence_filter, 0.0..=1.0)
                        .text("display confidence"),
                );
                ui.separator();
                for (index, run) in self.runs.iter().enumerate() {
                    ui.checkbox(&mut self.visible_runs[index], &run.label);
                    let available = self.available_counts[index];
                    let state = run
                        .manifest
                        .as_ref()
                        .map_or("recorded".to_string(), |manifest| match manifest.state {
                            ModelRunState::Running => "running".to_string(),
                            ModelRunState::Complete => "complete".to_string(),
                            ModelRunState::Incomplete => "incomplete".to_string(),
                            ModelRunState::Failed => "failed".to_string(),
                        });
                    ui.label(
                        RichText::new(format!("{state}, {available} frames"))
                            .small()
                            .color(Color32::GRAY),
                    );
                    ui.add_space(6.0);
                }
                if self.runs.is_empty() {
                    ui.label("No prediction caches found.");
                }
                ui.separator();
                ui.label("Space: play/pause");
                ui.label("Arrows: step frame");
                ui.label("Drag/scroll: pan/zoom");
                ui.label("Double-click: reset view");
            });
    }

    fn central_panel(&mut self, context: &egui::Context) {
        egui::CentralPanel::default().show(context, |ui| {
            if let Some(error) = &self.load_error {
                ui.colored_label(Color32::LIGHT_RED, error);
            }
            let Some(texture) = self.texture.clone() else {
                ui.centered_and_justified(|ui| ui.spinner());
                return;
            };
            let Some(frame_index) = self.displayed_frame else {
                return;
            };
            let visible = self
                .visible_runs
                .iter()
                .enumerate()
                .filter_map(|(index, visible)| visible.then_some(index))
                .collect::<Vec<_>>();
            if visible.is_empty() {
                ui.centered_and_justified(|ui| ui.label("Select at least one model."));
                return;
            }
            let columns = ((ui.available_width() / 430.0).floor() as usize).clamp(1, 4);
            egui::ScrollArea::vertical().show(ui, |ui| {
                for row in visible.chunks(columns) {
                    ui.columns(columns, |column_uis| {
                        for (column, run_index) in row.iter().copied().enumerate() {
                            let run = &self.runs[run_index];
                            let label = run.label.clone();
                            let prediction = run.predictions[frame_index].clone();
                            column_uis[column].group(|ui| {
                                ui.heading(label);
                                self.model_view(ui, &texture, frame_index, prediction.as_deref());
                            });
                        }
                    });
                    ui.add_space(8.0);
                }
            });
        });
    }

    fn model_view(
        &mut self,
        ui: &mut Ui,
        texture: &TextureHandle,
        frame_index: usize,
        prediction: Option<&Prediction>,
    ) {
        let frame = &self.recording.frames()[frame_index];
        let image_size = vec2(frame.width as f32, frame.height as f32);
        let viewport_width = ui.available_width().max(1.0);
        let viewport_height = (viewport_width * image_size.y / image_size.x).max(1.0);
        let (viewport, response) = ui.allocate_exact_size(
            vec2(viewport_width, viewport_height),
            Sense::click_and_drag(),
        );
        self.update_camera(ui, &response, viewport, image_size);
        let image_rect = camera_image_rect(viewport, image_size, self.camera_zoom, self.camera_pan);
        let painter = ui.painter_at(viewport);
        painter.image(
            texture.id(),
            image_rect,
            Rect::from_min_max(Pos2::ZERO, pos2(1.0, 1.0)),
            Color32::WHITE,
        );
        match prediction {
            Some(prediction) => {
                draw_objects(
                    ui,
                    viewport,
                    image_rect,
                    image_size,
                    &prediction.objects,
                    self.confidence_filter,
                );
                let inference = prediction
                    .inference_duration_nanos
                    .map(|value| format!("{:.1} ms", value as f64 / 1.0e6))
                    .unwrap_or_else(|| "recorded".to_string());
                ui.label(format!(
                    "{} detections, {inference}",
                    prediction
                        .objects
                        .iter()
                        .filter(|object| object.bounding_box.confidence >= self.confidence_filter)
                        .count()
                ));
            }
            None => {
                painter.rect_filled(viewport, 0.0, Color32::from_black_alpha(90));
                painter.text(
                    viewport.center(),
                    egui::Align2::CENTER_CENTER,
                    "prediction unavailable",
                    FontId::proportional(16.0),
                    Color32::LIGHT_GRAY,
                );
                ui.label("No cached result for this frame");
            }
        }
    }

    fn update_camera(
        &mut self,
        ui: &Ui,
        response: &egui::Response,
        viewport: Rect,
        image_size: Vec2,
    ) {
        if response.double_clicked_by(PointerButton::Primary)
            || response.double_clicked_by(PointerButton::Secondary)
        {
            self.camera_zoom = 1.0;
            self.camera_pan = Vec2::ZERO;
            return;
        }
        if response.dragged() {
            self.camera_pan += ui.input(|input| input.pointer.delta());
        }
        let Some(pointer) = response.hover_pos() else {
            return;
        };
        let scroll = ui.input(|input| input.smooth_scroll_delta.y);
        if scroll.abs() <= f32::EPSILON {
            return;
        }
        let old_zoom = self.camera_zoom;
        let new_zoom = (old_zoom * 1.01_f32.powf(scroll)).clamp(MIN_ZOOM, MAX_ZOOM);
        let fit = fitted_scale(viewport.size(), image_size);
        let old_rect = camera_image_rect(viewport, image_size, old_zoom, self.camera_pan);
        let image_pixel = (pointer - old_rect.min) / (fit * old_zoom).max(f32::EPSILON);
        let new_size = image_size * fit * new_zoom;
        let new_min = pointer - image_pixel * fit * new_zoom;
        self.camera_zoom = new_zoom;
        self.camera_pan = new_min + new_size * 0.5 - viewport.center();
    }

    fn relative_time(&self, frame: usize) -> f64 {
        let first = self.recording.frames()[self.start_frame].timestamp_nanos;
        (self.recording.frames()[frame].timestamp_nanos - first) as f64 / 1.0e9
    }
}

impl App for ReplayApp {
    fn update(&mut self, context: &egui::Context, _frame: &mut Frame) {
        self.poll_loader(context);
        self.handle_keys(context);
        self.advance_playback(context);
        self.display_selected_frame(context);
        self.top_panel(context);
        self.side_panel(context);
        self.central_panel(context);
    }
}

struct FrameLoader {
    requests: mpsc::Sender<FrameRequest>,
    results: mpsc::Receiver<FrameResult>,
    requested: Option<usize>,
    generation: u64,
}

#[derive(Clone, Copy)]
struct FrameRequest {
    frame: usize,
    end_frame: usize,
    generation: u64,
}

struct FrameResult {
    frame: usize,
    generation: u64,
    image: std::result::Result<ColorImage, String>,
}

impl FrameLoader {
    fn new(recording: Arc<Recording>) -> Result<Self> {
        let (request_sender, request_receiver) = mpsc::channel::<FrameRequest>();
        let (result_sender, result_receiver) =
            mpsc::sync_channel::<FrameResult>(PREFETCH_FRAMES * 2);
        thread::Builder::new()
            .name("detection-replay-frame-loader".to_string())
            .spawn(move || {
                let Ok(mut request) = request_receiver.recv() else {
                    return;
                };
                'requests: loop {
                    if let Some(latest) = request_receiver.try_iter().last() {
                        request = latest;
                    }
                    let prefetch_end = request
                        .frame
                        .saturating_add(PREFETCH_FRAMES - 1)
                        .min(request.end_frame);
                    for frame in request.frame..=prefetch_end {
                        if let Some(latest) = request_receiver.try_iter().last() {
                            request = latest;
                            continue 'requests;
                        }
                        let image = recording
                            .load_proxy_frame(frame)
                            .map(|image| {
                                ColorImage::from_rgb(
                                    [image.width() as usize, image.height() as usize],
                                    image.as_raw(),
                                )
                            })
                            .map_err(|error| format!("failed to load frame {frame}: {error:#}"));
                        if result_sender
                            .send(FrameResult {
                                frame,
                                generation: request.generation,
                                image,
                            })
                            .is_err()
                        {
                            return;
                        }
                    }
                    let Ok(next) = request_receiver.recv() else {
                        return;
                    };
                    request = next;
                }
            })
            .map_err(|error| eyre!("failed to spawn frame loader: {error}"))?;
        Ok(Self {
            requests: request_sender,
            results: result_receiver,
            requested: None,
            generation: 0,
        })
    }

    fn request(&mut self, frame: usize, end_frame: usize) {
        if self.requested == Some(frame) {
            return;
        }
        self.generation = self.generation.wrapping_add(1);
        if self
            .requests
            .send(FrameRequest {
                frame,
                end_frame,
                generation: self.generation,
            })
            .is_ok()
        {
            self.requested = Some(frame);
        }
    }

    fn try_receive(&mut self) -> Option<std::result::Result<(usize, ColorImage), String>> {
        let result = loop {
            let result = self.results.try_recv().ok()?;
            if result.generation == self.generation {
                break result;
            }
        };
        if self.requested == Some(result.frame) {
            self.requested = None;
        }
        Some(result.image.map(|image| (result.frame, image)))
    }
}

fn camera_image_rect(viewport: Rect, image: Vec2, zoom: f32, pan: Vec2) -> Rect {
    let size = image * fitted_scale(viewport.size(), image) * zoom;
    Rect::from_center_size(viewport.center() + pan, size)
}

fn fitted_scale(viewport: Vec2, image: Vec2) -> f32 {
    (viewport.x / image.x.max(1.0))
        .min(viewport.y / image.y.max(1.0))
        .max(0.01)
}

fn draw_objects(
    ui: &Ui,
    clip: Rect,
    image_rect: Rect,
    image_size: Vec2,
    objects: &[Object<RobocupObjectLabel>],
    confidence_filter: f32,
) {
    let scale = vec2(
        image_rect.width() / image_size.x.max(1.0),
        image_rect.height() / image_size.y.max(1.0),
    );
    let painter = ui.painter_at(clip);
    for object in objects
        .iter()
        .filter(|object| object.bounding_box.confidence >= confidence_filter)
    {
        let min = image_rect.min
            + vec2(
                object.bounding_box.area.min.x() * scale.x,
                object.bounding_box.area.min.y() * scale.y,
            );
        let max = image_rect.min
            + vec2(
                object.bounding_box.area.max.x() * scale.x,
                object.bounding_box.area.max.y() * scale.y,
            );
        let rect = Rect::from_min_max(min, max).intersect(clip);
        let color = label_color(object.label);
        painter.rect_stroke(
            rect,
            egui::CornerRadius::same(3),
            Stroke::new(2.0, color),
            StrokeKind::Outside,
        );
        let label: String = object.label.into();
        let text = format!("{label} {:.0}%", object.bounding_box.confidence * 100.0);
        let position = rect.min + vec2(4.0, 3.0);
        let galley = painter.layout_no_wrap(text, FontId::proportional(12.0), Color32::WHITE);
        painter.rect_filled(
            Rect::from_min_size(position - vec2(2.0, 1.0), galley.size() + vec2(4.0, 2.0)),
            2.0,
            color.gamma_multiply(0.8),
        );
        painter.galley(position, galley, Color32::WHITE);
    }
}

fn label_color(label: RobocupObjectLabel) -> Color32 {
    match label {
        RobocupObjectLabel::Ball => Color32::from_rgb(255, 166, 43),
        RobocupObjectLabel::GoalPost => Color32::from_rgb(255, 232, 92),
        RobocupObjectLabel::LSpot => Color32::from_rgb(76, 201, 240),
        RobocupObjectLabel::PenaltySpot => Color32::from_rgb(114, 239, 221),
        RobocupObjectLabel::Robot => Color32::from_rgb(255, 89, 123),
        RobocupObjectLabel::TSpot => Color32::from_rgb(179, 136, 255),
        RobocupObjectLabel::XSpot => Color32::from_rgb(93, 230, 129),
    }
}
