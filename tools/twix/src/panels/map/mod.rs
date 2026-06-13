use coordinate_systems::{Field, Ground};
use eframe::egui::{ComboBox, Ui, Widget};
use linear_algebra::{Isometry2, point, vector};
use serde::{Deserialize, Serialize};
use serde_json::{Map as JsonMap, Value, json};
use types::field_dimensions::FieldDimensions;

use crate::{
    panel::{Panel, PanelCreationContext},
    twix_painter::{Orientation, TwixPainter},
    value_buffer::BufferHandle,
    zoom_and_pan::ZoomAndPanTransform,
};

use self::layer::{EnabledLayer, Layer};

mod layer;
mod layers;

const SKIPPED_LAYER_KEYS: &[&str] = &["behavior_simulator", "pose_detection", "referee_position"];

#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
enum PlotType {
    Field,
    Ground,
}

trait GenericLayer {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    );
}

impl<T: Layer<Field>> GenericLayer for EnabledLayer<T, Field> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        _ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint_or_disable(painter, field_dimensions)
    }
}

impl<T: Layer<Ground>> GenericLayer for EnabledLayer<T, Ground> {
    fn generic_paint(
        &mut self,
        painter: &TwixPainter<Field>,
        ground_to_field: Isometry2<Ground, Field>,
        field_dimensions: &FieldDimensions,
    ) {
        self.paint_or_disable(
            &painter.transform_painter(ground_to_field.inverse()),
            field_dimensions,
        )
    }
}

pub struct MapPanel {
    current_plot_type: PlotType,
    skipped_layers: JsonMap<String, Value>,

    field_dimensions: BufferHandle<FieldDimensions>,
    ground_to_field: BufferHandle<Isometry2<Ground, Field>>,
    zoom_and_pan: ZoomAndPanTransform,

    field: EnabledLayer<layers::Field, Field>,
    image_segments: EnabledLayer<layers::ImageSegments, Ground>,
    lines: EnabledLayer<layers::Lines, Ground>,
    ball_search_heatmap: EnabledLayer<layers::BallSearchHeatmap, Field>,
    line_correspondences: EnabledLayer<layers::LineCorrespondences, Field>,
    path_obstacles: EnabledLayer<layers::PathObstacles, Ground>,
    obstacles: EnabledLayer<layers::Obstacles, Ground>,
    path: EnabledLayer<layers::Path, Ground>,
    robot_pose: EnabledLayer<layers::RobotPose, Ground>,
    ball_percept: EnabledLayer<layers::BallPercepts, Ground>,
    ball_position: EnabledLayer<layers::BallPosition, Field>,
    ball_filter: EnabledLayer<layers::BallFilter, Ground>,
    obstacle_filter: EnabledLayer<layers::ObstacleFilter, Ground>,
    localization: EnabledLayer<layers::Localization, Field>,
    voronoi_cells: EnabledLayer<layers::VoronoiCell, Field>,
}

impl<'a> Panel<'a> for MapPanel {
    const NAME: &'static str = "Map";

    fn new(context: PanelCreationContext) -> Self {
        let field = EnabledLayer::new(context.backend.clone(), context.value, true);
        let image_segments = EnabledLayer::new(context.backend.clone(), context.value, false);
        let line_correspondences = EnabledLayer::new(context.backend.clone(), context.value, false);
        let lines = EnabledLayer::new(context.backend.clone(), context.value, true);
        let ball_search_heatmap = EnabledLayer::new(context.backend.clone(), context.value, false);
        let path_obstacles = EnabledLayer::new(context.backend.clone(), context.value, false);
        let obstacles = EnabledLayer::new(context.backend.clone(), context.value, false);
        let path = EnabledLayer::new(context.backend.clone(), context.value, false);
        let robot_pose = EnabledLayer::new(context.backend.clone(), context.value, true);
        let ball_percept = EnabledLayer::new(context.backend.clone(), context.value, false);
        let ball_position = EnabledLayer::new(context.backend.clone(), context.value, true);
        let ball_filter = EnabledLayer::new(context.backend.clone(), context.value, false);
        let obstacle_filter = EnabledLayer::new(context.backend.clone(), context.value, false);
        let localization = EnabledLayer::new(context.backend.clone(), context.value, false);
        let voronoi_cells = EnabledLayer::new(context.backend.clone(), context.value, false);

        let field_dimensions = context
            .backend
            .subscribe_transient_local_value("field_dimensions");
        let ground_to_field = context.backend.subscribe_value("ground_to_field");

        let current_plot_type = context
            .value
            .and_then(|value| value.get("current_plot_type"))
            .and_then(|value| serde_json::from_value::<PlotType>(value.clone()).ok())
            .unwrap_or(PlotType::Ground);
        let zoom_and_pan = context
            .value
            .and_then(|value| value.get("zoom_and_pan"))
            .and_then(|value| serde_json::from_value::<ZoomAndPanTransform>(value.clone()).ok())
            .unwrap_or_default();
        let skipped_layers = preserved_skipped_layer_state(context.value);

        Self {
            current_plot_type,
            skipped_layers,
            field_dimensions,
            ground_to_field,
            zoom_and_pan,
            field,
            image_segments,
            line_correspondences,
            lines,
            ball_search_heatmap,
            path_obstacles,
            obstacles,
            path,
            robot_pose,
            ball_percept,
            ball_position,
            ball_filter,
            obstacle_filter,
            localization,
            voronoi_cells,
        }
    }

    fn save(&self) -> Value {
        let mut value = json!({
            "current_plot_type": self.current_plot_type,
            "zoom_and_pan": serde_json::to_value(&self.zoom_and_pan).expect("failed to serialize zoom_and_pan"),

            "field": self.field.save(),
            "image_segments": self.image_segments.save(),
            "line_correspondences": self.line_correspondences.save(),
            "lines": self.lines.save(),
            "ball_search_heatmap": self.ball_search_heatmap.save(),
            "path_obstacles": self.path_obstacles.save(),
            "obstacles": self.obstacles.save(),
            "path": self.path.save(),
            "robot_pose": self.robot_pose.save(),
            "ball_percept": self.ball_percept.save(),
            "ball_position": self.ball_position.save(),
            "ball_filter": self.ball_filter.save(),
            "obstacle_filter": self.obstacle_filter.save(),
            "localization": self.localization.save(),
            "voronoi_cells": self.voronoi_cells.save(),
        });

        let Value::Object(object) = &mut value else {
            return value;
        };
        object.extend(self.skipped_layers.clone());
        value
    }
}

fn preserved_skipped_layer_state(value: Option<&Value>) -> JsonMap<String, Value> {
    let Some(Value::Object(object)) = value else {
        return JsonMap::new();
    };

    SKIPPED_LAYER_KEYS
        .iter()
        .filter_map(|key| {
            object
                .get(*key)
                .map(|value| ((*key).to_owned(), value.clone()))
        })
        .collect()
}

impl Widget for &mut MapPanel {
    fn ui(self, ui: &mut Ui) -> eframe::egui::Response {
        ui.horizontal(|ui| {
            ui.menu_button("Overlays", |ui| {
                self.field.checkbox(ui);
                self.image_segments.checkbox(ui);
                self.line_correspondences.checkbox(ui);
                self.lines.checkbox(ui);
                self.ball_search_heatmap.checkbox(ui);
                self.path_obstacles.checkbox(ui);
                self.obstacles.checkbox(ui);
                self.path.checkbox(ui);
                self.robot_pose.checkbox(ui);
                self.ball_percept.checkbox(ui);
                self.ball_position.checkbox(ui);
                self.ball_filter.checkbox(ui);
                self.obstacle_filter.checkbox(ui);
                self.localization.checkbox(ui);
                self.voronoi_cells.checkbox(ui);
            });
            ComboBox::from_id_salt("plot_type_selector")
                .selected_text(format!("{:?}", self.current_plot_type))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.current_plot_type, PlotType::Ground, "Ground");
                    ui.selectable_value(&mut self.current_plot_type, PlotType::Field, "Field");
                });
        });

        let field_dimensions: FieldDimensions = match self.field_dimensions.get_last_value() {
            Ok(Some(value)) => value,
            Ok(None) => return ui.label("no response for field dimensions"),
            Err(error) => return ui.label(format!("{error:#}")),
        };

        let ground_to_field = self
            .ground_to_field
            .get_last_value()
            .ok()
            .flatten()
            .unwrap_or_default();
        let (response, mut painter) = match self.current_plot_type {
            PlotType::Field => {
                let width = field_dimensions.width;
                let length = field_dimensions.length;
                let border = field_dimensions.border_strip_width;

                TwixPainter::allocate(
                    ui,
                    vector![2.0 * border + length, 2.0 * border + width],
                    point![
                        border + field_dimensions.length / 2.0,
                        -border - field_dimensions.width / 2.0
                    ],
                    Orientation::RightHanded,
                )
            }
            PlotType::Ground => {
                let (response, painter) = TwixPainter::allocate(
                    ui,
                    vector![2.0, 2.0],
                    point![1.0, -1.0],
                    Orientation::RightHanded,
                );
                (response, painter.transform_painter(ground_to_field))
            }
        };
        self.zoom_and_pan.apply(ui, &mut painter, &response);

        // draw largest layers first so they don't obscure smaller ones
        self.field
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.image_segments
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        self.line_correspondences
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.lines
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        self.ball_search_heatmap
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.path_obstacles
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.obstacles
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.path
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.robot_pose
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_percept
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_position
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.ball_filter
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.obstacle_filter
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.localization
            .generic_paint(&painter, ground_to_field, &field_dimensions);
        self.voronoi_cells
            .generic_paint(&painter, ground_to_field, &field_dimensions);

        response
    }
}

#[cfg(test)]
mod tests {
    use serde_json::json;

    use super::*;

    #[test]
    fn save_preserves_skipped_overlay_state_from_saved_map() {
        let saved = json!({
            "field": { "enabled": false },
            "behavior_simulator": { "enabled": true },
            "pose_detection": { "accepted": true },
            "referee_position": { "enabled": true },
        });

        let preserved = preserved_skipped_layer_state(Some(&saved));

        assert_eq!(
            preserved.get("behavior_simulator"),
            saved.get("behavior_simulator")
        );
        assert_eq!(preserved.get("pose_detection"), saved.get("pose_detection"));
        assert_eq!(
            preserved.get("referee_position"),
            saved.get("referee_position")
        );
        assert!(!preserved.contains_key("field"));
    }
}
