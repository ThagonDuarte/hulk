use std::{f32::consts::PI, time::Duration};

use approx::AbsDiffEq;
use booster::{JointsMotorState, MotorState};
use color_eyre::Result;
use coordinate_systems::{Ground, Robot};
use linear_algebra::{vector, IntoFramed, Vector2, Vector3};
use path_serde::{PathDeserialize, PathIntrospect, PathSerialize};
use serde::{Deserialize, Serialize};
use types::{
    joints::Joints,
    parameters::{MotorCommandParameters, RLWalkingParameters},
};

#[derive(
    Debug, Default, Clone, Serialize, Deserialize, PathSerialize, PathDeserialize, PathIntrospect,
)]
pub struct WalkingInferenceInputs {
    pub gravity: Vector3<Robot>,
    pub angular_velocity: Vector3<Robot>,
    pub linear_velocity_command: Vector2<Ground>,
    pub angular_velocity_command: f32,
    pub gait_progress: f32,
    pub gait_process: nalgebra::Vector2<f32>,
    pub joint_position_differences: Joints,
    pub joint_velocities: Joints,
    pub last_target_joint_positions: Joints,
}

impl WalkingInferenceInputs {
    #[allow(clippy::too_many_arguments)]
    pub fn try_new(
        last_cycle_duration: Duration,
        linear_velocity_command: Vector2<Ground>,
        angular_velocity_command: f32,
        roll_pitch_yaw: Vector3<Robot>,
        angular_velocity: Vector3<Robot>,
        current_serial_joints: Joints<MotorState>,
        last_linear_velocity_command: Vector2<Ground>,
        last_angular_velocity_command: f32,
        last_gait_progress: f32,
        last_target_joint_positions: Joints,
        walking_parameters: &RLWalkingParameters,
        motor_command_parameters: &MotorCommandParameters,
    ) -> Result<Self> {
        let policy_interval =
            last_cycle_duration.as_secs_f32() * walking_parameters.control.decimation;

        let linear_velocity_command_difference =
            linear_velocity_command - last_linear_velocity_command;
        let angular_velocity_command_difference =
            angular_velocity_command - last_angular_velocity_command;
        let (linear_velocity_command, angular_velocity_command) = (
            last_linear_velocity_command
                + vector![
                    linear_velocity_command_difference
                        .x()
                        .clamp(-policy_interval, policy_interval,),
                    linear_velocity_command_difference
                        .y()
                        .clamp(-policy_interval, policy_interval,)
                ],
            last_angular_velocity_command
                + angular_velocity_command_difference.clamp(-policy_interval, policy_interval),
        );

        let stabilizing_interval_progress = last_gait_progress
            + walking_parameters.gait_frequency * last_cycle_duration.as_secs_f32();

        let is_step_finished = (stabilizing_interval_progress
            * walking_parameters.stabilizing_interval_compression_factor
            * PI)
            .sin()
            .abs_diff_eq(
                &0.0,
                walking_parameters.stabilizing_interval_completion_threshold,
            )
            || (stabilizing_interval_progress
                * walking_parameters.stabilizing_interval_compression_factor
                * PI)
                .cos()
                .abs_diff_eq(
                    &1.0,
                    walking_parameters.stabilizing_interval_completion_threshold,
                );

        let (gait_frequency, last_gait_progress) = if linear_velocity_command.norm() < 1e-5
            && angular_velocity_command.abs() < 1e-5
            && is_step_finished
        {
            (0.0, 0.0)
        } else {
            (walking_parameters.gait_frequency, last_gait_progress)
        };
        let gait_progress = last_gait_progress + gait_frequency * last_cycle_duration.as_secs_f32();

        let gait_process =
            nalgebra::Rotation2::new(2.0 * PI * gait_progress) * nalgebra::Vector2::x();

        let current_joint_position = current_serial_joints.positions();
        let current_joint_velocities = current_serial_joints.velocities();

        let joint_position_differences =
            current_joint_position - motor_command_parameters.default_positions;

        let rotation = nalgebra::Rotation3::from_euler_angles(
            roll_pitch_yaw.x(),
            roll_pitch_yaw.y(),
            roll_pitch_yaw.z(),
        );
        let gravity = rotation
            .inverse()
            .transform_vector(&-nalgebra::Vector3::z_axis())
            .framed()
            * walking_parameters.normalization.linear_velocity;

        let linear_velocity_command =
            linear_velocity_command * walking_parameters.normalization.linear_velocity;
        let angular_velocity_command =
            angular_velocity_command * walking_parameters.normalization.angular_velocity;
        let normalized_joint_position_differences = joint_position_differences
            .into_iter()
            .map(|elem| elem * walking_parameters.normalization.joint_position)
            .collect();
        let normalized_joint_velocities = current_joint_velocities
            .into_iter()
            .map(|elem| elem * walking_parameters.normalization.joint_velocity)
            .collect();

        Ok(WalkingInferenceInputs {
            gravity,
            angular_velocity,
            linear_velocity_command,
            angular_velocity_command,
            gait_progress,
            gait_process,
            joint_position_differences: normalized_joint_position_differences,
            joint_velocities: normalized_joint_velocities,
            last_target_joint_positions,
        })
    }

    pub fn booster_gym_observation_vector(&self) -> Vec<f32> {
        [
            self.gravity.x(),
            self.gravity.y(),
            self.gravity.z(),
            self.angular_velocity.x(),
            self.angular_velocity.y(),
            self.angular_velocity.z(),
            self.linear_velocity_command.x(),
            self.linear_velocity_command.y(),
            self.angular_velocity_command,
            self.gait_process.x,
            self.gait_process.y,
        ]
        .into_iter()
        .chain(self.joint_position_differences.left_leg)
        .chain(self.joint_position_differences.right_leg)
        .chain(self.joint_velocities.left_leg)
        .chain(self.joint_velocities.right_leg)
        .chain(self.last_target_joint_positions.left_leg)
        .chain(self.last_target_joint_positions.right_leg)
        .collect::<Vec<f32>>()
    }

    pub fn booster_deploy_observation_vector(&self) -> Vec<f32> {
        [
            self.angular_velocity.x(),
            self.angular_velocity.y(),
            self.angular_velocity.z(),
            self.gravity.x(),
            self.gravity.y(),
            self.gravity.z(),
            self.linear_velocity_command.x(),
            self.linear_velocity_command.y(),
            self.angular_velocity_command,
        ]
        .into_iter()
        .chain(
            self.joint_position_differences
                .body()
                .to_booster_deploy_joint_array(),
        )
        .chain(self.joint_velocities.body().to_booster_deploy_joint_array())
        .chain(
            self.last_target_joint_positions
                .body()
                .to_booster_deploy_joint_array(),
        )
        .collect::<Vec<f32>>()
    }
}
