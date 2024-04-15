use std::time::{Duration, SystemTime};

use color_eyre::Result;
use context_attribute::context;
use framework::{MainOutput, PerceptionInput};
use serde::{Deserialize, Serialize};
use spl_network_messages::{Penalty, PlayerNumber};
use types::{
    cycle_time::CycleTime, game_controller_state::GameControllerState, messages::IncomingMessage,
    parameters::RefereePoseDetectionFilterParameters, players::Players, pose_types::PoseType,
};

#[derive(Deserialize, Serialize)]
pub struct RefereePoseDetectionFilter {
    pose_detection_times: Vec<Option<SystemTime>>,
}

#[context]
pub struct CreationContext {}

#[context]
pub struct CycleContext {
    game_controller_state: RequiredInput<Option<GameControllerState>, "game_controller_state?">,
    network_message: PerceptionInput<IncomingMessage, "SplNetwork", "message">,
    detected_referee_pose_type:
        PerceptionInput<PoseType, "DetectionTop", "detected_referee_pose_type">,
    cycle_time: Input<CycleTime, "cycle_time">,

    parameters: Parameter<RefereePoseDetectionFilterParameters, "referee_pose_detection_filter">,
    player_number: Parameter<PlayerNumber, "player_number">,
}

#[context]
#[derive(Default)]
pub struct MainOutputs {
    pub initial_to_ready_trigger: MainOutput<bool>,
}

impl RefereePoseDetectionFilter {
    pub fn new(_context: CreationContext) -> Result<Self> {
        Ok(Self {
            pose_detection_times: vec![None; 7],
        })
    }

    pub fn cycle(&mut self, context: CycleContext) -> Result<MainOutputs> {
        let cycle_start_time = context.cycle_time.start_time;

        let pose_detection_times = self.update(
            &context.detected_referee_pose_type,
            *context.player_number,
            &context.network_message,
        );

        let initial_to_ready_trigger = self.decide(
            pose_detection_times,
            cycle_start_time,
            context.parameters,
            context.game_controller_state.penalties,
            context
                .game_controller_state
                .hulks_team_is_home_after_coin_toss,
        );
        Ok(MainOutputs {
            initial_to_ready_trigger: initial_to_ready_trigger.into(),
        })
    }

    fn update(
        &mut self,
        detected_referee_pose_type: &PerceptionInput<Vec<&PoseType>>,
        player_number: PlayerNumber,
        spl_messages: &PerceptionInput<Vec<&IncomingMessage>>,
    ) -> Vec<Option<SystemTime>> {
        let persistent_messages: Vec<_> = spl_messages
            .persistent
            .iter()
            .flat_map(|(time, messages)| messages.iter().map(|message| (*time, message)))
            .filter_map(|(time, message)| match message {
                IncomingMessage::GameController(_) => None,
                IncomingMessage::Spl(message) => Some((time, message)),
            })
            .collect();

        for (time, message) in persistent_messages {
            if message.over_arms_pose_detected {
                self.pose_detection_times[message.player_number as usize] = Some(time);
            }
        }

        let persistent_own_detected_pose_time = detected_referee_pose_type
            .persistent
            .iter()
            .flat_map(|(time, pose_types)| pose_types.iter().map(|pose_type| (*time, pose_type)))
            .filter_map(|(time, pose_type)| match pose_type {
                PoseType::OverheadArms => Some(time),
                _ => None,
            })
            .last();

        if persistent_own_detected_pose_time.is_some() {
            self.pose_detection_times[player_number as usize] = persistent_own_detected_pose_time;
        }

        let mut temporary_pose_detection_times = self.pose_detection_times.clone();

        let temporary_messages: Vec<_> = spl_messages
            .temporary
            .iter()
            .flat_map(|(time, messages)| messages.iter().map(|message| (*time, message)))
            .filter_map(|(time, message)| match message {
                IncomingMessage::GameController(_) => None,
                IncomingMessage::Spl(message) => Some((time, message)),
            })
            .collect();

        for (time, message) in temporary_messages {
            if message.over_arms_pose_detected {
                temporary_pose_detection_times[message.player_number as usize] = Some(time);
            }
        }

        let temporary_own_detected_pose_time = detected_referee_pose_type
            .temporary
            .iter()
            .flat_map(|(time, pose_types)| pose_types.iter().map(|pose_type| (*time, pose_type)))
            .filter_map(|(time, pose_type)| match pose_type {
                PoseType::OverheadArms => Some(time),
                _ => None,
            })
            .last();

        if temporary_own_detected_pose_time.is_some() {
            temporary_pose_detection_times[player_number as usize] =
                temporary_own_detected_pose_time;
        }

        temporary_pose_detection_times
    }

    fn determine_minimum_overhead_arms_detections(
        &self,
        parameters: &RefereePoseDetectionFilterParameters,
        penalties: Players<Option<Penalty>>,
        hulks_team_is_home_after_coin_toss: bool,
    ) -> usize {
        let number_of_non_penalized_robots = penalties
            .iter()
            .filter(|(player_number, penalty)| {
                if hulks_team_is_home_after_coin_toss {
                    match (player_number, penalty) {
                        (PlayerNumber::Three, None) => true,
                        (PlayerNumber::Four, None) => true,
                        (PlayerNumber::Five, None) => true,
                        (PlayerNumber::Seven, None) => true,
                        (_, _) => false,
                    }
                } else {
                    match (player_number, penalty) {
                        (PlayerNumber::One, None) => true,
                        (PlayerNumber::Two, None) => true,
                        (PlayerNumber::Six, None) => true,
                        (_, _) => false,
                    }
                }
            })
            .count();
        f32::ceil(
            number_of_non_penalized_robots as f32
                * parameters.required_fraction_of_available_robots_for_decision,
        ) as usize
    }

    fn decide(
        &mut self,
        pose_detection_times: Vec<Option<SystemTime>>,
        cycle_start_time: SystemTime,
        parameters: &RefereePoseDetectionFilterParameters,
        penalties: Players<Option<Penalty>>,
        hulks_team_is_home_after_coin_toss: bool,
    ) -> bool {
        let minimum_over_head_arms_detections = self.determine_minimum_overhead_arms_detections(
            parameters,
            penalties,
            hulks_team_is_home_after_coin_toss,
        );

        let detected_over_head_arms_poses = pose_detection_times
            .iter()
            .filter(|detection_time| match detection_time {
                Some(detection_time) => Self::in_grace_period(
                    cycle_start_time,
                    *detection_time,
                    parameters.initial_message_grace_period,
                ),
                None => false,
            })
            .count();
        detected_over_head_arms_poses >= minimum_over_head_arms_detections
    }

    fn in_grace_period(
        cycle_start_time: SystemTime,
        earlier_time: SystemTime,
        grace_period: Duration,
    ) -> bool {
        cycle_start_time
            .duration_since(earlier_time)
            .expect("Time ran backwards")
            < grace_period
    }
}
