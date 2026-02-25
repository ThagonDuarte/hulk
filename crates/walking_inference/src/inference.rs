use std::path::Path;

use booster::{ImuState, MotorCommandParameters, MotorState};
use color_eyre::Result;
use coordinate_systems::Ground;
use framework::deserialize_not_implemented;
use kinematics::joints::{Joints, arm::ArmJoints, leg::LegJoints};
use linear_algebra::{Vector2, vector};
use ndarray::{Array1, Axis};
use ort::{
    execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider},
    inputs,
    session::{Session, builder::GraphOptimizationLevel},
    value::Tensor,
};
use serde::{Deserialize, Serialize};
use types::{cycle_time::CycleTime, parameters::RLWalkingParameters};

use crate::inputs::{WalkCommand, WalkingInferenceInputs};

#[derive(Deserialize, Serialize)]
pub struct WalkingInference {
    #[serde(skip, default = "deserialize_not_implemented")]
    session: Session,
    last_linear_velocity_command: Vector2<Ground>,
    last_angular_velocity_command: f32,
    last_gait_progress: f32,
    pub last_target_joint_positions: Joints,
}

impl WalkingInference {
    pub fn new(
        neural_network_folder: impl AsRef<Path>,
        prepare_motor_command_parameters: &MotorCommandParameters,
    ) -> Result<Self> {
        let neural_network_path = neural_network_folder
            .as_ref()
            .join("2026-02-25_11-03-50.onnx");

        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_execution_providers([
                TensorRTExecutionProvider::default().build(),
                CUDAExecutionProvider::default().build(),
            ])?
            .commit_from_file(neural_network_path)?;

        Ok(Self {
            session,
            last_linear_velocity_command: vector![0.0, 0.0],
            last_angular_velocity_command: 0.0,
            last_gait_progress: 0.0,
            last_target_joint_positions: prepare_motor_command_parameters.default_positions,
        })
    }

    fn calculate_inputs(
        &mut self,
        cycle_time: CycleTime,
        walk_command: &WalkCommand,
        imu_state: &ImuState,
        current_serial_joints: Joints<MotorState>,
        walking_parameters: &RLWalkingParameters,
        motor_command_parameters: &MotorCommandParameters,
    ) -> Result<WalkingInferenceInputs> {
        let walking_inference_inputs = WalkingInferenceInputs::try_new(
            cycle_time,
            walk_command,
            imu_state.roll_pitch_yaw,
            imu_state.angular_velocity,
            current_serial_joints,
            self.last_linear_velocity_command,
            self.last_angular_velocity_command,
            self.last_gait_progress,
            self.last_target_joint_positions,
            walking_parameters,
            motor_command_parameters,
        )?;

        self.last_linear_velocity_command = walking_inference_inputs.linear_velocity_command;
        self.last_angular_velocity_command = walking_inference_inputs.angular_velocity_command;
        self.last_gait_progress = walking_inference_inputs.gait_progress;

        Ok(walking_inference_inputs)
    }

    pub fn do_inference(
        &mut self,
        cycle_time: CycleTime,
        walk_command: &WalkCommand,
        imu_state: &ImuState,
        current_serial_joints: Joints<MotorState>,
        walking_parameters: &RLWalkingParameters,
        motor_command_parameters: &MotorCommandParameters,
    ) -> Result<(WalkingInferenceInputs, Joints)> {
        let Ok(walking_inference_inputs) = self.calculate_inputs(
            cycle_time,
            walk_command,
            imu_state,
            current_serial_joints,
            walking_parameters,
            motor_command_parameters,
        ) else {
            return Ok((
                WalkingInferenceInputs::default(),
                motor_command_parameters.default_positions,
            ));
        };

        let inputs: Array1<f32> = walking_inference_inputs.as_vec().into();

        assert!(inputs.len() == walking_parameters.number_of_observations);
        let inputs_tensor = Tensor::from_array(inputs.insert_axis(Axis(0)))?;

        let outputs = self.session.run(inputs![inputs_tensor])?;
        let predictions = outputs["actions"].try_extract_array::<f32>()?.squeeze();

        // predictions.clamp(
        //     -walking_parameters.normalization.clip_actions,
        //     walking_parameters.normalization.clip_actions,
        // );

        assert!(predictions.len() == walking_parameters.number_of_actions);

        // ALeft_Shoulder_Pitch,Left_Shoulder_Roll,Left_Elbow_Pitch,Left_Elbow_Yaw,
        // ARight_Shoulder_Pitch,Right_Shoulder_Roll,Right_Elbow_Pitch,Right_Elbow_Yaw,
        // Left_Hip_Pitch,Left_Hip_Roll,Left_Hip_Yaw,Left_Knee_Pitch,Left_Ankle_Pitch,Left_Ankle_Roll,
        // Right_Hip_Pitch,Right_Hip_Roll,Right_Hip_Yaw,Right_Knee_Pitch,Right_Ankle_Pitch,Right_Ankle_Roll
        self.last_target_joint_positions = Joints {
            left_arm: ArmJoints {
                shoulder_pitch: predictions[0],
                shoulder_roll: predictions[1],
                elbow: predictions[2],
                shoulder_yaw: predictions[3],
            },
            right_arm: ArmJoints {
                shoulder_pitch: predictions[4],
                shoulder_roll: predictions[5],
                elbow: predictions[6],
                shoulder_yaw: predictions[7],
            },
            left_leg: LegJoints {
                hip_pitch: predictions[8],
                hip_roll: predictions[9],
                hip_yaw: predictions[10],
                knee: predictions[11],
                ankle_up: predictions[12],
                ankle_down: predictions[13],
            },
            right_leg: LegJoints {
                hip_pitch: predictions[14],
                hip_roll: predictions[15],
                hip_yaw: predictions[16],
                knee: predictions[17],
                ankle_up: predictions[18],
                ankle_down: predictions[19],
            },
            ..Default::default()
        };

        Ok((walking_inference_inputs, self.last_target_joint_positions))
    }
}
