use std::{sync::Arc, time::Duration};

use color_eyre::Result;
use eframe::epaint::Color32;

use coordinate_systems::{Field, Ground};
use linear_algebra::Isometry2;
use types::field_dimensions::FieldDimensions;

use crate::{
    backend::TwixBackend, panels::map::layer::Layer, twix_painter::TwixPainter,
    value_buffer::BufferHandle,
};

pub struct BallPosition {
    ground_to_field: BufferHandle<Option<Isometry2<Ground, Field>>>,
    ball_position: BufferHandle<Option<types::ball_position::BallPosition<Ground>>>,
    team_ball: BufferHandle<types::ball_position::BallPosition<Field>>,
}

impl Layer<Field> for BallPosition {
    const NAME: &'static str = "Ball Position";

    fn new(backend: Arc<TwixBackend>) -> Self {
        let ground_to_field = backend.subscribe_buffered_value_with_queue_depth(
            "ground_to_field",
            Duration::from_secs(2),
            crate::backend::HIGH_RATE_SUBSCRIBER_QUEUE_DEPTH,
        );
        let ball_position = backend.subscribe_buffered_value_with_queue_depth(
            "ball_filter/ball_position",
            Duration::from_secs(2),
            crate::backend::HIGH_RATE_SUBSCRIBER_QUEUE_DEPTH,
        );
        let team_ball = backend.subscribe_buffered_value_with_queue_depth(
            "team_ball",
            Duration::ZERO,
            crate::backend::HIGH_RATE_SUBSCRIBER_QUEUE_DEPTH,
        );
        Self {
            ground_to_field,
            ball_position,
            team_ball,
        }
    }

    fn paint(
        &self,
        painter: &TwixPainter<Field>,
        field_dimensions: &FieldDimensions,
    ) -> Result<()> {
        let ground_to_fields = self.ground_to_field.get()?;
        let ball_positions = self.ball_position.get()?;

        for (ball, ground_to_field) in ball_positions.iter().zip(ground_to_fields.iter()) {
            let Some(ball) = ball.value else {
                continue;
            };
            let ground_to_field = ground_to_field.value.unwrap_or_default();
            painter.circle_filled(
                ground_to_field * ball.position,
                field_dimensions.ball_radius,
                Color32::from_white_alpha(10),
            );
        }

        if let Some(ball) = self.team_ball.get_last_value()? {
            painter.ball(ball.position, field_dimensions.ball_radius, Color32::RED);
        }

        if let Some(ball) = self.ball_position.get_last_value()?.flatten() {
            let ground_to_field = self
                .ground_to_field
                .get_last_value()?
                .flatten()
                .unwrap_or_default();
            painter.ball(
                ground_to_field * ball.position,
                field_dimensions.ball_radius,
                Color32::WHITE,
            );
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ground_to_field_uses_reference_optional_message_type() {
        fn assert_optional_ground_to_field(_: &BufferHandle<Option<Isometry2<Ground, Field>>>) {}

        fn assert_ball_position(layer: &BallPosition) {
            assert_optional_ground_to_field(&layer.ground_to_field);
        }

        let _ = assert_ball_position;
    }
}
