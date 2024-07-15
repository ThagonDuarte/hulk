use std::f32::consts::{FRAC_PI_2, FRAC_PI_4};

use coordinate_systems::{Field, Ground};
use framework::AdditionalOutput;
use geometry::look_at::LookAt;
use linear_algebra::{point, Orientation2, Point2, Pose2, Rotation2, Vector2};
use types::{
    ball,
    field_dimensions::FieldDimensions,
    filtered_game_state::FilteredGameState,
    motion_command::{MotionCommand, WalkSpeed},
    path_obstacles::PathObstacle,
    support_foot::Side,
    world_state::{BallState, WorldState},
};

use super::{head::LookAction, walk_to_pose::WalkAndStand};

#[allow(clippy::too_many_arguments)]
pub fn execute(
    world_state: &WorldState,
    field_dimensions: &FieldDimensions,
    field_side: Option<Side>,
    distance_to_ball: f32,
    maximum_x_in_ready_and_when_ball_is_not_free: f32,
    minimum_x: f32,
    walk_and_stand: &WalkAndStand,
    look_action: &LookAction,
    path_obstacles_output: &mut AdditionalOutput<Vec<PathObstacle>>,
    walk_speed: WalkSpeed,
) -> Option<MotionCommand> {
    let pose = support_pose(
        world_state,
        field_dimensions,
        field_side,
        distance_to_ball,
        maximum_x_in_ready_and_when_ball_is_not_free,
        minimum_x,
    )?;
    walk_and_stand.execute(
        pose,
        look_action.execute(),
        path_obstacles_output,
        walk_speed,
    )
}

fn support_pose(
    world_state: &WorldState,
    field_dimensions: &FieldDimensions,
    field_side: Option<Side>,
    distance_to_ball: f32,
    maximum_x_in_ready_and_when_ball_is_not_free: f32,
    minimum_x: f32,
) -> Option<Pose2<Ground>> {
    let ground_to_field = world_state.robot.ground_to_field?;
    let ball = world_state
        .rule_ball
        .or(world_state.ball)
        .unwrap_or_else(|| BallState::new_at_center(ground_to_field));
    let side = field_side.unwrap_or_else(|| ball.field_side.opposite());

    let offset_vector =
        calculate_position_offset_vector(ball.ball_in_field, side, distance_to_ball, minimum_x);
    let supporting_position = ball.ball_in_field + offset_vector;

    let filtered_game_state = world_state
        .filtered_game_controller_state
        .map(|filtered_game_controller_state| filtered_game_controller_state.game_state);
    let clamped_x = match filtered_game_state {
        Some(FilteredGameState::Ready { .. })
        | Some(FilteredGameState::Playing {
            ball_is_free: false,
            ..
        }) => supporting_position.x().clamp(
            minimum_x.min(maximum_x_in_ready_and_when_ball_is_not_free),
            minimum_x.max(maximum_x_in_ready_and_when_ball_is_not_free),
        ),
        _ => supporting_position
            .x()
            .clamp(minimum_x, field_dimensions.length / 2.0),
    };
    let clamped_y = supporting_position
        .y()
        .clamp(-field_dimensions.width / 2.0, field_dimensions.width / 2.0);
    let clamped_position = point![clamped_x, clamped_y];
    let support_pose = Pose2::new(
        clamped_position.coords(),
        clamped_position.look_at(&ball.ball_in_field).angle(),
    );
    Some(ground_to_field.inverse() * support_pose)
}

fn calculate_position_offset_vector(
    ball_in_field: Point2<Field>,
    side: Side,
    distance_to_ball: f32,
    minimum_x: f32,
) -> Vector2<Field> {
    // #[allow::clippy::]
    let offset_angle = if ball_in_field.x() < -minimum_x {
        FRAC_PI_2
    } else if (-minimum_x..0.0).contains(&ball_in_field.x()) {
        Orientation2::<Field>::new(FRAC_PI_4)
            .slerp(
                Orientation2::<Field>::new(FRAC_PI_2),
                ball_in_field.x() / (-minimum_x + f32::EPSILON),
            )
            .angle()
    } else {
        FRAC_PI_4
    };

    let rotation = Rotation2::new(match side {
        Side::Left => -offset_angle,
        Side::Right => offset_angle,
    });

    rotation * -(Vector2::<Field>::x_axis() * distance_to_ball)
}
