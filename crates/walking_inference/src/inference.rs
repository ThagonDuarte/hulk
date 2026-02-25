use std::{collections::VecDeque, path::Path, time::Duration};

use booster::{ImuState, MotorState};
use color_eyre::Result;
use coordinate_systems::Ground;
use framework::deserialize_not_implemented;
use linear_algebra::{vector, Vector2};
use ndarray::{Array1, Axis};
use ort::{
    execution_providers::{CUDAExecutionProvider, TensorRTExecutionProvider},
    inputs,
    session::{builder::GraphOptimizationLevel, Session},
    value::Tensor,
};
use serde::{Deserialize, Serialize};
use types::{
    joints::{body::BodyJoints, Joints},
    parameters::{MotorCommandParameters, RLWalkingParameters},
};

use crate::inputs::WalkingInferenceInputs;

#[derive(Deserialize, Serialize)]
pub struct WalkingInference {
    #[serde(skip, default = "deserialize_not_implemented")]
    session: Session,
    last_linear_velocity_command: Vector2<Ground>,
    last_angular_velocity_command: f32,
    last_gait_progress: f32,
    last_target_joint_positions: Joints,
    input_history: VecDeque<WalkingInferenceInputs>,
}

impl WalkingInference {
    pub fn new(
        neural_network_folder: impl AsRef<Path>,
        prepare_motor_command_parameters: &MotorCommandParameters,
    ) -> Result<Self> {
        let neural_network_path = neural_network_folder.as_ref().join("t1_walk.onnx");

        let session = Session::builder()?
            .with_optimization_level(GraphOptimizationLevel::Level3)?
            .with_execution_providers([
                TensorRTExecutionProvider::default().build(),
                CUDAExecutionProvider::default().build(),
            ])?
            .commit_from_file(neural_network_path)?;

        let mut input_history = VecDeque::with_capacity(10);
        for _ in 0..10 {
            input_history.push_front(Default::default());
        }

        Ok(Self {
            session,
            last_linear_velocity_command: vector![0.0, 0.0],
            last_angular_velocity_command: 0.0,
            last_gait_progress: 0.0,
            last_target_joint_positions: prepare_motor_command_parameters.default_positions,
            input_history,
        })
    }

    #[allow(clippy::too_many_arguments)]
    pub fn do_inference(
        &mut self,
        last_cycle_duration: Duration,
        linear_velocity_command: Vector2<Ground>,
        angular_velocity_command: f32,
        imu_state: &ImuState,
        current_serial_joints: Joints<MotorState>,
        walking_parameters: &RLWalkingParameters,
        motor_command_parameters: &MotorCommandParameters,
    ) -> Result<Joints> {
        let walking_inference_inputs = WalkingInferenceInputs::try_new(
            last_cycle_duration,
            linear_velocity_command,
            angular_velocity_command,
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

        self.input_history.push_front(walking_inference_inputs);
        self.input_history
            .truncate(walking_parameters.observation_history_length);

        let inputs: Array1<f32> = self
            .input_history
            .iter()
            .rev()
            .flat_map(|inputs| inputs.booster_deploy_observation_vector())
            .collect::<Vec<f32>>()
            .into();

        assert!(
            inputs.len()
                == walking_parameters.number_of_observations
                    * walking_parameters.observation_history_length
        );
        let inputs_tensor = Tensor::from_array(inputs.insert_axis(Axis(0)))?;

        let inference_input = inputs![inputs_tensor];

        let outputs = self.session.run(inference_input)?;
        let predictions = outputs["21"].try_extract_array::<f32>()?.squeeze();

        predictions.clamp(
            -walking_parameters.normalization.clip_actions,
            walking_parameters.normalization.clip_actions,
        );

        assert!(predictions.len() == walking_parameters.number_of_actions);

        self.last_target_joint_positions = Joints::from_head_and_body(
            Default::default(),
            BodyJoints::from_booster_deploy_joint_array(
                predictions.as_slice().unwrap().try_into()?,
            ),
        );

        Ok(self.last_target_joint_positions)
    }
}
