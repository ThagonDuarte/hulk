use std::sync::Arc;

use color_eyre::Result;
use eframe::epaint::{Color32, Stroke};

use behavior_node::node::Blackboard;
use coordinate_systems::Ground;
use types::{field_dimensions::FieldDimensions, path_obstacles::PathObstacleShape};

use crate::{
    backend::TwixBackend, panels::map::layer::Layer, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct PathObstacles {
    blackboard: BufferHandle<Blackboard>,
}

impl Layer<Ground> for PathObstacles {
    const NAME: &'static str = "Path Obstacles";

    fn new(backend: Arc<TwixBackend>) -> Self {
        let blackboard = backend.subscribe_value("behavior/blackboard");
        Self { blackboard }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Ground>,
        _field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        if let Some(blackboard) = self.blackboard.get_last_value()? {
            let path_obstacle_stroke = Stroke {
                width: 0.025,
                color: Color32::RED,
            };
            for path_obstacle in blackboard.path_obstacles_output {
                match path_obstacle.shape {
                    PathObstacleShape::Circle(circle) => {
                        painter.circle_stroke(circle.center, circle.radius, path_obstacle_stroke)
                    }
                    PathObstacleShape::LineSegment(line_segment) => {
                        painter.line_segment(line_segment.0, line_segment.1, path_obstacle_stroke)
                    }
                }
            }
        }

        Ok(())
    }
}
