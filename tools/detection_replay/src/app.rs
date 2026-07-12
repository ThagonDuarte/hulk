use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{Arc, mpsc},
    thread,
    time::{Duration, Instant},
};

use color_eyre::{Result, eyre::eyre};
use detection_replay::{
    BookmarkCollection, LoadedPredictionRun, ModelRunState, Prediction, PredictionSource,
    Recording, RunUiMetadata,
};
use eframe::{
    App, Frame, NativeOptions, Renderer,
    egui::{
        self, Color32, ColorImage, FontId, Id, Key, Modifiers, PointerButton, Pos2, Rect, RichText,
        Sense, Stroke, StrokeKind, TextureHandle, TextureOptions, Ui, Vec2, WidgetText, pos2, vec2,
    },
};
use egui_dock::{DockArea, DockState, Node, Split, TabViewer};
use types::{
    object_detection::{Object, RobocupObjectLabel, YOLOObjectLabel},
    pose_detection::{Keypoint, Pose},
};

use crate::timeline::TimelineState;

const MIN_ZOOM: f32 = 1.0;
const MAX_ZOOM: f32 = 20.0;
const PREFETCH_FRAMES: usize = 12;
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

pub fn run(recording: Recording, start_frame: usize, end_frame: usize) -> Result<()> {
    let mut runs = recording.load_runs()?;
    let bookmarks_file_exists = recording.bookmarks_exist()?;
    let legacy_bookmark_storage_key = format!(
        "detection-replay-bookmarks-{}",
        recording.fingerprint().cache_key()?
    );
    let (run_metadata, metadata_error) = match recording.load_run_ui_metadata() {
        Ok(metadata) => (metadata, None),
        Err(error) => (
            BTreeMap::new(),
            Some(format!("failed to load run GUI metadata: {error:#}")),
        ),
    };
    let (bookmarks, bookmark_error) = match recording.load_bookmarks() {
        Ok(bookmarks) => (bookmarks, None),
        Err(error) => (
            BookmarkCollection::default(),
            Some(format!("failed to load bookmarks: {error:#}")),
        ),
    };
    for run in &mut runs {
        if let Some(label) = run_metadata
            .get(&run.key)
            .and_then(|metadata| metadata.renamed_label.as_ref())
        {
            run.label.clone_from(label);
        }
    }
    let recording = Arc::new(recording);
    let startup_errors = [metadata_error.clone(), bookmark_error]
        .into_iter()
        .flatten()
        .collect::<Vec<_>>();
    eframe::run_native(
        "Detection Replay",
        NativeOptions {
            renderer: Renderer::Wgpu,
            ..Default::default()
        },
        Box::new(move |creation_context| {
            let mut bookmarks = bookmarks;
            let mut startup_errors = startup_errors;
            let mut legacy_bookmarks_to_clear = None;
            if !bookmarks_file_exists
                && let Some(serialized) = creation_context
                    .storage
                    .and_then(|storage| storage.get_string(&legacy_bookmark_storage_key))
            {
                match serde_json::from_str::<BookmarkCollection>(&serialized) {
                    Ok(legacy_bookmarks) => {
                        if let Err(error) = recording.save_bookmarks(&legacy_bookmarks) {
                            startup_errors
                                .push(format!("failed to migrate legacy bookmarks: {error:#}"));
                        } else {
                            bookmarks = legacy_bookmarks;
                            legacy_bookmarks_to_clear = Some(legacy_bookmark_storage_key.clone());
                        }
                    }
                    Err(error) => {
                        startup_errors.push(format!("failed to decode legacy bookmarks: {error}"))
                    }
                }
            }
            let loader =
                FrameLoader::new(Arc::clone(&recording), creation_context.egui_ctx.clone())
                    .map_err(|error| -> Box<dyn std::error::Error + Send + Sync> {
                        error.into()
                    })?;
            Ok(Box::new(ReplayApp::new(
                creation_context,
                recording,
                runs,
                loader,
                start_frame,
                end_frame,
                run_metadata,
                bookmarks,
                metadata_error.is_some(),
                (!startup_errors.is_empty()).then(|| startup_errors.join("\n")),
                legacy_bookmarks_to_clear,
            )))
        }),
    )
    .map_err(|error| eyre!("failed to run detection replay: {error}"))
}

struct ReplayApp {
    recording: Arc<Recording>,
    runs: Vec<LoadedPredictionRun>,
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
    show_poses: bool,
    keypoint_confidence_filter: f32,
    camera_zoom: f32,
    camera_pan: Vec2,
    start_frame: usize,
    end_frame: usize,
    dock_state: DockState<PanelTab>,
    timeline: TimelineState,
    run_metadata: BTreeMap<String, RunUiMetadata>,
    metadata_needs_repair: bool,
    bookmarks_dirty: bool,
    legacy_bookmarks_to_clear: Option<String>,
    prediction_errors: BTreeMap<String, String>,
    rename_edits: BTreeMap<String, String>,
    management_error: Option<String>,
    pending_delete: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
enum PanelTab {
    Timeline,
    Models,
    Model(String),
}

enum RunAction {
    Open(String),
    Rename { key: String, label: String },
    SetHidden { key: String, hidden: bool },
    RequestDelete(String),
}

impl ReplayApp {
    #[allow(clippy::too_many_arguments)]
    fn new(
        creation_context: &eframe::CreationContext<'_>,
        recording: Arc<Recording>,
        runs: Vec<LoadedPredictionRun>,
        mut loader: FrameLoader,
        start_frame: usize,
        end_frame: usize,
        run_metadata: BTreeMap<String, RunUiMetadata>,
        bookmarks: BookmarkCollection,
        metadata_needs_repair: bool,
        startup_error: Option<String>,
        legacy_bookmarks_to_clear: Option<String>,
    ) -> Self {
        loader.request(start_frame, end_frame);
        creation_context.egui_ctx.set_visuals(egui::Visuals::dark());
        let available_counts: Vec<usize> = runs
            .iter()
            .map(|run| run.available_count(start_frame..=end_frame))
            .collect();
        let visible_run_count = runs
            .iter()
            .filter(|run| {
                !run_metadata
                    .get(&run.key)
                    .is_some_and(|metadata| metadata.hidden)
            })
            .count();
        let mut tabs = runs
            .iter()
            .filter(|run| {
                !run_metadata
                    .get(&run.key)
                    .is_some_and(|metadata| metadata.hidden)
            })
            .map(|run| PanelTab::Model(run.key.clone()))
            .collect::<Vec<_>>();
        tabs.push(PanelTab::Models);
        let mut dock_state = DockState::new(tabs);
        let available_height = creation_context
            .egui_ctx
            .input(|input| input.content_rect().height())
            .max(1.0);
        let timeline_height = crate::timeline::desired_height(visible_run_count) + 30.0;
        let central_fraction = (1.0 - timeline_height / available_height).clamp(0.2, 0.85);
        dock_state.split(
            (0.into(), 0.into()),
            Split::Below,
            central_fraction,
            Node::leaf(PanelTab::Timeline),
        );
        let mut timeline = TimelineState::new(&recording, start_frame, end_frame);
        timeline.bookmarks = bookmarks;
        let rename_edits = runs
            .iter()
            .map(|run| (run.key.clone(), run.label.clone()))
            .collect();
        Self {
            recording,
            runs,
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
            confidence_filter: 0.5,
            show_poses: false,
            keypoint_confidence_filter: 0.5,
            camera_zoom: 1.0,
            camera_pan: Vec2::ZERO,
            start_frame,
            end_frame,
            dock_state,
            timeline,
            run_metadata,
            metadata_needs_repair,
            bookmarks_dirty: false,
            legacy_bookmarks_to_clear,
            prediction_errors: BTreeMap::new(),
            rename_edits,
            management_error: startup_error,
            pending_delete: None,
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

        let mut accumulator = self.playback_accumulator;
        let (next_frame, is_playing) = consume_playback_time(
            self.selected_frame,
            self.start_frame,
            self.end_frame,
            &mut accumulator,
            self.loop_playback,
            |frame| self.playback_frame_duration(frame),
        );
        self.playback_accumulator = accumulator;
        self.is_playing = is_playing;
        if next_frame != self.selected_frame {
            self.selected_frame = next_frame;
            if !self.decoded_frames.contains_key(&next_frame) {
                self.loader.request(next_frame, self.end_frame);
            }
        }
        if self.is_playing {
            let repaint_after = ((self.playback_frame_duration(next_frame)
                - self.playback_accumulator)
                / f64::from(self.playback_speed))
            .max(0.001);
            context.request_repaint_after(Duration::from_secs_f64(repaint_after));
        }
    }

    fn handle_keys(&mut self, context: &egui::Context) {
        if context.wants_keyboard_input() {
            return;
        }
        let (toggle, previous, next, bookmark, previous_bookmark, next_bookmark) =
            context.input(|input| {
                (
                    input.key_pressed(Key::Space),
                    input.key_pressed(Key::ArrowLeft) || input.key_pressed(Key::Comma),
                    input.key_pressed(Key::ArrowRight) || input.key_pressed(Key::Period),
                    input.key_pressed(Key::B) && input.modifiers == Modifiers::NONE,
                    input.key_pressed(Key::PageUp),
                    input.key_pressed(Key::PageDown),
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
        if bookmark {
            self.timeline.bookmarks.toggle(self.selected_frame);
            self.persist_bookmarks();
        }
        if previous_bookmark
            && let Some(frame) = self.timeline.bookmarks.previous(
                self.selected_frame,
                self.start_frame,
                self.end_frame,
            )
        {
            self.is_playing = false;
            self.select_frame(frame);
        }
        if next_bookmark
            && let Some(frame) =
                self.timeline
                    .bookmarks
                    .next(self.selected_frame, self.start_frame, self.end_frame)
        {
            self.is_playing = false;
            self.select_frame(frame);
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
                ui.separator();
                ui.menu_button("View", |ui| {
                    if self.dock_state.find_tab(&PanelTab::Timeline).is_none()
                        && ui.button("Timeline").clicked()
                    {
                        self.dock_state.push_to_focused_leaf(PanelTab::Timeline);
                        ui.close();
                    }
                    if self.dock_state.find_tab(&PanelTab::Models).is_none()
                        && ui.button("Models").clicked()
                    {
                        self.dock_state.push_to_focused_leaf(PanelTab::Models);
                        ui.close();
                    }
                    for run in &self.runs {
                        let hidden = self
                            .run_metadata
                            .get(&run.key)
                            .is_some_and(|metadata| metadata.hidden);
                        let tab = PanelTab::Model(run.key.clone());
                        let open = self.dock_state.find_tab(&tab).is_some();
                        if !hidden && !open && ui.button(&run.label).clicked() {
                            self.dock_state.push_to_focused_leaf(tab);
                            ui.close();
                        }
                    }
                    if self.dock_state.find_tab(&PanelTab::Timeline).is_some()
                        && self.dock_state.find_tab(&PanelTab::Models).is_some()
                        && self.runs.iter().all(|run| {
                            self.run_metadata
                                .get(&run.key)
                                .is_some_and(|metadata| metadata.hidden)
                                || self
                                    .dock_state
                                    .find_tab(&PanelTab::Model(run.key.clone()))
                                    .is_some()
                        })
                    {
                        ui.label("All panels are open");
                    }
                });
                ui.menu_button("?", |ui| {
                    ui.label("Space: play/pause");
                    ui.label("Arrows or ,/.: step frame");
                    ui.label("B: toggle bookmark");
                    ui.label("Page Up/Down: previous/next bookmark");
                    ui.label("Timeline drag: scrub");
                    ui.label("Timeline wheel: zoom");
                    ui.label("Timeline Shift+wheel: pan");
                    ui.label("Image drag/scroll: pan/zoom");
                    ui.label("Double-click: reset view");
                });
            });
        });
    }

    fn models_panel(&mut self, ui: &mut Ui, open_models: &BTreeSet<String>) -> Vec<RunAction> {
        let mut actions = Vec::new();
        ui.heading("Models");
        ui.add(
            egui::Slider::new(&mut self.confidence_filter, 0.0..=1.0).text("display confidence"),
        );
        ui.checkbox(&mut self.show_poses, "Show poses");
        ui.add_enabled(
            self.show_poses,
            egui::Slider::new(&mut self.keypoint_confidence_filter, 0.0..=1.0)
                .text("keypoint confidence"),
        );
        if let Some(error) = &self.management_error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
        ui.separator();
        for (index, run) in self.runs.iter().enumerate() {
            let key = run.key.clone();
            let is_recorded = run.source == PredictionSource::RecordedBaseline;
            let mut hidden = self
                .run_metadata
                .get(&key)
                .is_some_and(|metadata| metadata.hidden);
            ui.horizontal(|ui| {
                if is_recorded {
                    ui.strong(&run.label);
                    ui.add_enabled(false, egui::Button::new("Rename"))
                        .on_disabled_hover_text("Recorded is derived from the MCAP");
                } else {
                    let edit = self
                        .rename_edits
                        .entry(key.clone())
                        .or_insert_with(|| run.label.clone());
                    let response = ui.text_edit_singleline(edit);
                    let submit = ui.small_button("Rename").clicked()
                        || (response.lost_focus()
                            && ui.input(|input| input.key_pressed(Key::Enter)));
                    if submit {
                        actions.push(RunAction::Rename {
                            key: key.clone(),
                            label: edit.clone(),
                        });
                    }
                }
                if ui.checkbox(&mut hidden, "Hidden").changed() {
                    actions.push(RunAction::SetHidden {
                        key: key.clone(),
                        hidden,
                    });
                }
                let open = open_models.contains(&key);
                if !hidden && !open && ui.small_button("Open viewport").clicked() {
                    actions.push(RunAction::Open(key.clone()));
                }
                if !is_recorded && ui.small_button("Delete").clicked() {
                    actions.push(RunAction::RequestDelete(key.clone()));
                } else if is_recorded {
                    ui.add_enabled(false, egui::Button::new("Delete"))
                        .on_disabled_hover_text("Recorded is derived from the MCAP");
                }
            });
            let available = self.available_counts[index];
            let state = run
                .manifest
                .as_ref()
                .map_or("recorded", |manifest| match manifest.state {
                    ModelRunState::Running => "running",
                    ModelRunState::Complete => "complete",
                    ModelRunState::Incomplete => "incomplete",
                    ModelRunState::Failed => "failed",
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
        actions
    }

    fn dock_area(&mut self, context: &egui::Context) {
        egui::CentralPanel::default().show(context, |ui| {
            let open_models = self
                .dock_state
                .iter_all_tabs()
                .filter_map(|(_, tab)| match tab {
                    PanelTab::Model(key) => Some(key.clone()),
                    _ => None,
                })
                .collect();
            let mut dock_state =
                std::mem::replace(&mut self.dock_state, DockState::new(Vec::new()));
            let mut viewer = DetectionTabViewer {
                app: self,
                selected_frame: None,
                run_actions: Vec::new(),
                open_models,
            };
            DockArea::new(&mut dock_state).show_inside(ui, &mut viewer);
            let selected_frame = viewer.selected_frame;
            for action in viewer.run_actions {
                self.handle_run_action(action, &mut dock_state);
            }
            self.dock_state = dock_state;
            if let Some(frame) = selected_frame {
                self.is_playing = false;
                self.select_frame(frame);
            }
        });
    }

    fn handle_run_action(&mut self, action: RunAction, dock_state: &mut DockState<PanelTab>) {
        self.management_error = None;
        match action {
            RunAction::Open(key) => {
                let hidden = self.is_hidden(&key);
                let tab = PanelTab::Model(key);
                if !hidden && dock_state.find_tab(&tab).is_none() {
                    dock_state.push_to_focused_leaf(tab);
                }
            }
            RunAction::Rename { key, label } => {
                let label = label.trim();
                if label.is_empty() {
                    self.management_error = Some("run name must not be empty".to_string());
                    return;
                }
                let Some(run) = self.runs.iter().find(|run| run.key == key) else {
                    return;
                };
                if run.source == PredictionSource::RecordedBaseline {
                    self.management_error =
                        Some("the Recorded baseline cannot be renamed".to_string());
                    return;
                }
                let metadata_result = if self.metadata_needs_repair {
                    let mut metadata = self.run_metadata.clone();
                    metadata.entry(key.clone()).or_default().renamed_label =
                        Some(label.to_string());
                    self.recording
                        .save_run_ui_metadata(&metadata)
                        .map(|()| metadata)
                } else {
                    self.recording.rename_run(&key, label)
                };
                self.run_metadata = match metadata_result {
                    Ok(metadata) => metadata,
                    Err(error) => {
                        self.management_error =
                            Some(format!("failed to persist run rename: {error:#}"));
                        return;
                    }
                };
                self.metadata_needs_repair = false;
                if let Some(run) = self.runs.iter_mut().find(|run| run.key == key) {
                    run.label = label.to_string();
                }
                self.rename_edits.insert(key, label.to_string());
            }
            RunAction::SetHidden { key, hidden } => {
                let metadata_result = if self.metadata_needs_repair {
                    let mut metadata = self.run_metadata.clone();
                    metadata.entry(key.clone()).or_default().hidden = hidden;
                    self.recording
                        .save_run_ui_metadata(&metadata)
                        .map(|()| metadata)
                } else {
                    self.recording.set_run_hidden(&key, hidden)
                };
                self.run_metadata = match metadata_result {
                    Ok(metadata) => metadata,
                    Err(error) => {
                        self.management_error =
                            Some(format!("failed to persist hidden state: {error:#}"));
                        return;
                    }
                };
                self.metadata_needs_repair = false;
                if hidden {
                    dock_state.retain_tabs(
                        |tab| !matches!(tab, PanelTab::Model(tab_key) if tab_key == &key),
                    );
                } else if dock_state.find_tab(&PanelTab::Model(key.clone())).is_none() {
                    dock_state.push_to_focused_leaf(PanelTab::Model(key));
                }
            }
            RunAction::RequestDelete(key) => self.pending_delete = Some(key),
        }
    }

    fn is_hidden(&self, key: &str) -> bool {
        self.run_metadata
            .get(key)
            .is_some_and(|metadata| metadata.hidden)
    }

    fn delete_run(&mut self, key: &str) {
        self.management_error = None;
        if let Err(error) = self.recording.delete_model_run(key) {
            self.management_error = Some(format!("failed to delete run: {error:#}"));
            return;
        }

        let metadata_result = if self.metadata_needs_repair {
            let mut metadata = self.run_metadata.clone();
            metadata.remove(key);
            self.recording
                .save_run_ui_metadata(&metadata)
                .map(|()| metadata)
        } else {
            self.recording.remove_run_ui_metadata(key)
        };
        match metadata_result {
            Ok(metadata) => {
                self.run_metadata = metadata;
                self.metadata_needs_repair = false;
            }
            Err(error) => {
                self.run_metadata.remove(key);
                self.management_error = Some(format!(
                    "run was deleted, but its GUI metadata could not be updated: {error:#}"
                ));
            }
        }
        self.dock_state
            .retain_tabs(|tab| !matches!(tab, PanelTab::Model(tab_key) if tab_key == key));
        if let Some(index) = self.runs.iter().position(|run| run.key == key) {
            self.runs.remove(index);
            self.available_counts.remove(index);
        }
        self.rename_edits.remove(key);
        self.pending_delete = None;
    }

    fn show_delete_confirmation(&mut self, context: &egui::Context) {
        let Some(key) = self.pending_delete.clone() else {
            return;
        };
        let Some(run) = self.runs.iter().find(|run| run.key == key) else {
            self.pending_delete = None;
            return;
        };
        let label = run.label.clone();
        let model_path = run
            .manifest
            .as_ref()
            .map(|manifest| manifest.canonical_model_path.display().to_string())
            .unwrap_or_default();
        egui::Window::new("Are you sure?")
            .id(Id::new("delete-run-confirmation"))
            .anchor(egui::Align2::CENTER_CENTER, Vec2::ZERO)
            .collapsible(false)
            .resizable(false)
            .show(context, |ui| {
                ui.label(format!("Delete cached predictions for `{label}`?"));
                ui.label(RichText::new(model_path).small().color(Color32::GRAY));
                ui.colored_label(
                    Color32::LIGHT_RED,
                    "This permanently removes the cached run and cannot be undone.",
                );
                ui.horizontal(|ui| {
                    if ui.button("Cancel").clicked() {
                        self.pending_delete = None;
                    }
                    if ui
                        .button(RichText::new("Delete run").color(Color32::LIGHT_RED))
                        .clicked()
                    {
                        self.delete_run(&key);
                    }
                });
            });
    }

    fn model_panel(&mut self, ui: &mut Ui, run_index: usize) {
        if let Some(error) = &self.load_error {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
        let run_key = self.runs[run_index].key.clone();
        let had_prediction_error = self.prediction_errors.contains_key(&run_key);
        if let Some(error) = self.prediction_errors.get(&run_key) {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
        let Some(texture) = self.texture.clone() else {
            ui.centered_and_justified(|ui| ui.spinner());
            return;
        };
        let Some(frame_index) = self.displayed_frame else {
            return;
        };
        let prediction = match self.runs[run_index].prediction(frame_index) {
            Ok(prediction) => prediction,
            Err(error) => {
                let error = format!("failed to load prediction for frame {frame_index}: {error:#}");
                if !had_prediction_error {
                    ui.colored_label(Color32::LIGHT_RED, &error);
                }
                self.prediction_errors.insert(run_key, error);
                None
            }
        };
        self.model_view(ui, &texture, frame_index, prediction.as_deref());
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
                if self.show_poses
                    && let Some(poses) = &prediction.poses
                {
                    draw_poses(
                        ui,
                        viewport,
                        image_rect,
                        image_size,
                        poses,
                        self.confidence_filter,
                        self.keypoint_confidence_filter,
                    );
                }
                let inference = prediction
                    .inference_duration_nanos
                    .map(|value| format!("{:.1} ms", value as f64 / 1.0e6))
                    .unwrap_or_else(|| "recorded".to_string());
                let object_count = prediction
                    .objects
                    .iter()
                    .filter(|object| object.bounding_box.confidence >= self.confidence_filter)
                    .count();
                let pose_status = prediction.poses.as_ref().map_or_else(
                    || "poses unavailable".to_string(),
                    |poses| {
                        let count = poses
                            .iter()
                            .filter(|pose| {
                                pose.object.bounding_box.confidence >= self.confidence_filter
                            })
                            .count();
                        format!("{count} poses")
                    },
                );
                ui.label(format!(
                    "{object_count} detections, {pose_status}, {inference}"
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

    fn playback_frame_duration(&self, frame: usize) -> f64 {
        let frames = self.recording.frames();
        let (current, next) = if frame < self.end_frame {
            (
                frames[frame].timestamp_nanos,
                frames[frame + 1].timestamp_nanos,
            )
        } else {
            (
                frames[frame - 1].timestamp_nanos,
                frames[frame].timestamp_nanos,
            )
        };
        ((next - current).max(1) as f64) / 1.0e9
    }

    fn persist_bookmarks(&mut self) {
        match self.recording.save_bookmarks(&self.timeline.bookmarks) {
            Ok(()) => self.bookmarks_dirty = false,
            Err(error) => {
                self.bookmarks_dirty = true;
                self.management_error = Some(format!("failed to persist bookmarks: {error:#}"));
            }
        }
    }
}

impl App for ReplayApp {
    fn update(&mut self, context: &egui::Context, _frame: &mut Frame) {
        self.poll_loader(context);
        self.handle_keys(context);
        self.advance_playback(context);
        self.display_selected_frame(context);
        self.top_panel(context);
        self.dock_area(context);
        self.show_delete_confirmation(context);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        if self.bookmarks_dirty {
            self.persist_bookmarks();
        }
        if let Some(key) = self.legacy_bookmarks_to_clear.take() {
            storage.set_string(&key, String::new());
        }
    }
}

struct DetectionTabViewer<'a> {
    app: &'a mut ReplayApp,
    selected_frame: Option<usize>,
    run_actions: Vec<RunAction>,
    open_models: BTreeSet<String>,
}

impl TabViewer for DetectionTabViewer<'_> {
    type Tab = PanelTab;

    fn title(&mut self, tab: &mut Self::Tab) -> WidgetText {
        match tab {
            PanelTab::Timeline => "Timeline".into(),
            PanelTab::Models => "Models".into(),
            PanelTab::Model(key) => self
                .app
                .runs
                .iter()
                .find(|run| &run.key == key)
                .map_or_else(|| "Missing run".into(), |run| run.label.clone().into()),
        }
    }

    fn id(&mut self, tab: &mut Self::Tab) -> Id {
        match tab {
            PanelTab::Timeline => Id::new("detection-replay-timeline"),
            PanelTab::Models => Id::new("detection-replay-models"),
            PanelTab::Model(key) => Id::new(("detection-replay-model", key)),
        }
    }

    fn ui(&mut self, ui: &mut Ui, tab: &mut Self::Tab) {
        match tab {
            PanelTab::Timeline => {
                let hidden_runs = self
                    .app
                    .run_metadata
                    .iter()
                    .filter_map(|(key, metadata)| metadata.hidden.then_some(key.clone()))
                    .collect::<BTreeSet<_>>();
                let response = crate::timeline::show(
                    ui,
                    &self.app.recording,
                    &self.app.runs,
                    self.app.start_frame..=self.app.end_frame,
                    self.app.selected_frame,
                    &mut self.app.timeline,
                    &hidden_runs,
                );
                if response.response.changed() || response.selected_frame.is_some() {
                    self.selected_frame = response.selected_frame;
                }
            }
            PanelTab::Models => self
                .run_actions
                .extend(self.app.models_panel(ui, &self.open_models)),
            PanelTab::Model(key) => {
                if let Some(index) = self.app.runs.iter().position(|run| &run.key == key) {
                    self.app.model_panel(ui, index);
                }
            }
        }
    }

    fn scroll_bars(&self, tab: &Self::Tab) -> [bool; 2] {
        match tab {
            PanelTab::Models => [true, true],
            PanelTab::Timeline => [false, true],
            PanelTab::Model(_) => [false, false],
        }
    }
}

struct FrameLoader {
    requests: mpsc::Sender<FrameRequest>,
    results: mpsc::Receiver<FrameResult>,
    requested: Option<usize>,
    failed: BTreeSet<usize>,
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
    fn new(recording: Arc<Recording>, context: egui::Context) -> Result<Self> {
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
                        context.request_repaint();
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
            failed: BTreeSet::new(),
            generation: 0,
        })
    }

    fn request(&mut self, frame: usize, end_frame: usize) {
        if self.requested == Some(frame) || self.failed.contains(&frame) {
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
        match &result.image {
            Ok(_) => {
                self.failed.remove(&result.frame);
            }
            Err(_) => {
                self.failed.insert(result.frame);
            }
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

fn consume_playback_time(
    mut frame: usize,
    start_frame: usize,
    end_frame: usize,
    accumulator: &mut f64,
    loop_playback: bool,
    frame_duration: impl Fn(usize) -> f64,
) -> (usize, bool) {
    loop {
        let duration = frame_duration(frame);
        if *accumulator < duration {
            return (frame, true);
        }
        *accumulator -= duration;
        if frame < end_frame {
            frame += 1;
        } else if loop_playback {
            frame = start_frame;
        } else {
            *accumulator = 0.0;
            return (frame, false);
        }
    }
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

fn draw_poses(
    ui: &Ui,
    clip: Rect,
    image_rect: Rect,
    image_size: Vec2,
    poses: &[Pose<YOLOObjectLabel>],
    pose_confidence_filter: f32,
    keypoint_confidence_filter: f32,
) {
    let scale = vec2(
        image_rect.width() / image_size.x.max(1.0),
        image_rect.height() / image_size.y.max(1.0),
    );
    let painter = ui.painter_at(clip);
    for pose in poses
        .iter()
        .filter(|pose| pose.object.bounding_box.confidence >= pose_confidence_filter)
    {
        let keypoints: [Keypoint; 17] = pose.keypoints.into();
        for (start, end) in POSE_SKELETON_KEYPOINT_LINE_MAPPING {
            if keypoints[start].confidence < keypoint_confidence_filter
                || keypoints[end].confidence < keypoint_confidence_filter
            {
                continue;
            }
            painter.line_segment(
                [
                    pose_point(image_rect, scale, keypoints[start]),
                    pose_point(image_rect, scale, keypoints[end]),
                ],
                Stroke::new(2.0, Color32::LIGHT_BLUE.gamma_multiply(0.8)),
            );
        }
        for keypoint in keypoints {
            if keypoint.confidence >= keypoint_confidence_filter {
                painter.circle_filled(
                    pose_point(image_rect, scale, keypoint),
                    3.0,
                    Color32::from_rgb(55, 145, 255),
                );
            }
        }

        let bounding_box = pose.object.bounding_box;
        let min = image_rect.min
            + vec2(
                bounding_box.area.min.x() * scale.x,
                bounding_box.area.min.y() * scale.y,
            );
        let max = image_rect.min
            + vec2(
                bounding_box.area.max.x() * scale.x,
                bounding_box.area.max.y() * scale.y,
            );
        painter.rect_stroke(
            Rect::from_min_max(min, max).intersect(clip),
            egui::CornerRadius::same(3),
            Stroke::new(2.0, Color32::DARK_BLUE),
            StrokeKind::Outside,
        );
    }
}

fn pose_point(image_rect: Rect, scale: Vec2, keypoint: Keypoint) -> Pos2 {
    image_rect.min + vec2(keypoint.point.x() * scale.x, keypoint.point.y() * scale.y)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn loop_playback_displays_end_before_wrapping() {
        let mut accumulator = 2.0;
        let (frame, playing) = consume_playback_time(0, 0, 2, &mut accumulator, true, |_| 1.0);
        assert_eq!(frame, 2);
        assert!(playing);
        assert_eq!(accumulator, 0.0);

        accumulator = 1.0;
        let (frame, playing) = consume_playback_time(2, 0, 2, &mut accumulator, true, |_| 1.0);
        assert_eq!(frame, 0);
        assert!(playing);
    }

    #[test]
    fn non_loop_playback_stops_after_end_duration() {
        let mut accumulator = 1.0;
        let (frame, playing) = consume_playback_time(2, 0, 2, &mut accumulator, false, |_| 1.0);
        assert_eq!(frame, 2);
        assert!(!playing);
        assert_eq!(accumulator, 0.0);
    }
}
