use std::{boxed::Box, future::Future, pin::Pin};
use std::{collections::HashSet, sync::Arc};

use color_eyre::Result;
use serde::{Deserialize, Serialize};

use hsl_network_messages::PlayerNumber;
use ros_z::{prelude::*, qos::QosDurability};
use types::{
    buttons::{ButtonPressType, Buttons},
    filtered_game_controller_state::FilteredGameControllerState,
    filtered_game_state::FilteredGameState,
    primary_state::{PrimaryState, SequencedPrimaryState},
    robot_mode::{RobotMode, SequencedRobotMode},
};

#[derive(Debug, Clone, Serialize, Deserialize, Message)]
#[serde(deny_unknown_fields)]
pub struct Parameters {
    pub injected_primary_state: Option<PrimaryState>,
    pub recorded_primary_states: HashSet<PrimaryState>,
}

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("primary_state_filter").build().await?;

    let parameters = node.bind_parameter_as::<Parameters>("primary_state_filter")?;
    let player_number_cache = node
        .create_cache::<PlayerNumber>("player_number", 1)?
        .with_qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let filtered_game_controller_state_sub = node
        .subscriber::<FilteredGameControllerState>("filtered_game_controller_state")?
        .build()
        .await?;
    let buttons_sub = node
        .subscriber::<Buttons<Option<ButtonPressType>>>("buttons")?
        .build()
        .await?;
    let robot_mode_sub = node
        .subscriber::<SequencedRobotMode>("robot_mode")?
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;
    let is_safe_pose_cache = node
        .create_cache::<bool>("is_safe_pose", 1)?
        .build()
        .await?;

    let primary_state_pub = node
        .publisher::<PrimaryState>("primary_state")?
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;
    let sequenced_primary_state_pub = node
        .publisher::<SequencedPrimaryState>("sequenced_primary_state")?
        .qos(QosProfile {
            durability: QosDurability::TransientLocal,
            ..Default::default()
        })
        .build()
        .await?;

    let mut primary_state_filter = PrimaryStateFilter::default();
    primary_state_pub
        .publish(&primary_state_filter.primary_state)
        .await?;
    sequenced_primary_state_pub
        .publish(&primary_state_filter.sequenced_primary_state())
        .await?;

    loop {
        let parameters_snapshot = parameters.snapshot();
        let parameters = parameters_snapshot.typed();
        tokio::select! {
            received_filtered_game_controller_state = filtered_game_controller_state_sub.recv() => {
                let Some(player_number) = player_number_cache.get_latest() else {continue};

                let filtered_game_controller_state = received_filtered_game_controller_state?;

                primary_state_filter.update_with_filtered_game_contoller_state(
                    &filtered_game_controller_state,
                    *player_number,
                );
            }
            received_buttons = buttons_sub.recv() => {
                let Some(is_safe_pose) = is_safe_pose_cache.get_latest() else {continue};

                let buttons = received_buttons?;

                primary_state_filter.update_with_buttons(&buttons, *is_safe_pose);

            }
            received_robot_mode = robot_mode_sub.recv() => {
                primary_state_filter.update_with_robot_mode(received_robot_mode?);
            }
        }

        if let Some(injected_primary_state) = parameters.injected_primary_state {
            primary_state_filter.update_with_injected_primary_state(injected_primary_state);
        }

        primary_state_pub
            .publish(&primary_state_filter.primary_state)
            .await?;
        sequenced_primary_state_pub
            .publish(&primary_state_filter.sequenced_primary_state())
            .await?;
    }
}

#[derive(Default)]
struct PrimaryStateFilter {
    pub primary_state: PrimaryState,
    pub robot_mode_sequence_number: u64,
}

impl PrimaryStateFilter {
    fn sequenced_primary_state(&self) -> SequencedPrimaryState {
        SequencedPrimaryState {
            primary_state: self.primary_state,
            robot_mode_sequence_number: self.robot_mode_sequence_number,
        }
    }

    fn update_with_robot_mode(&mut self, robot_mode: SequencedRobotMode) {
        self.robot_mode_sequence_number = robot_mode.sequence_number;
        self.primary_state = match robot_mode.mode {
            RobotMode::Damping => PrimaryState::Damping,
            RobotMode::Prepare => PrimaryState::Prepare,
            _ => self.primary_state,
        };
    }

    fn update_with_filtered_game_contoller_state(
        &mut self,
        filtered_game_controller_state: &FilteredGameControllerState,
        player_number: PlayerNumber,
    ) {
        let is_penalized = filtered_game_controller_state.penalties[player_number].is_some();
        let filtered_game_state = filtered_game_controller_state.game_state;

        self.primary_state = match (self.primary_state, filtered_game_state) {
            (PrimaryState::Damping, _) => PrimaryState::Damping,
            (PrimaryState::Initial, FilteredGameState::Ready) if !is_penalized => {
                PrimaryState::Ready
            }
            (PrimaryState::Ready, FilteredGameState::Set) if !is_penalized => PrimaryState::Set,
            (PrimaryState::Set, FilteredGameState::Playing { .. }) if !is_penalized => {
                PrimaryState::Playing
            }
            (PrimaryState::Playing, FilteredGameState::Ready) if !is_penalized => {
                PrimaryState::Ready
            }
            (state, FilteredGameState::Finished) if !matches!(state, PrimaryState::Damping) => {
                PrimaryState::Finished
            }
            (state, FilteredGameState::Stop) if !matches!(state, PrimaryState::Damping) => {
                PrimaryState::Stop
            }
            (state, _) if is_penalized && !matches!(state, PrimaryState::Damping) => {
                PrimaryState::Penalized
            }
            (PrimaryState::Stop, game_state) => {
                game_state_to_primary_state(game_state, is_penalized)
            }
            (PrimaryState::Penalized, game_state) if !is_penalized => {
                game_state_to_primary_state(game_state, false)
            }
            _ => self.primary_state,
        }
    }

    fn update_with_buttons(
        &mut self,
        buttons: &Buttons<Option<ButtonPressType>>,
        is_safe_pose: bool,
    ) {
        self.primary_state = match (self.primary_state, buttons) {
            (
                PrimaryState::Prepare,
                Buttons {
                    stand: Some(ButtonPressType::Long),
                    ..
                },
            ) if is_safe_pose => PrimaryState::Initial,
            (
                PrimaryState::Initial,
                Buttons {
                    walking: Some(ButtonPressType::Long),
                    ..
                },
            ) => PrimaryState::Playing,
            _ => self.primary_state,
        }
    }

    fn update_with_injected_primary_state(&mut self, injected_primary_state: PrimaryState) {
        self.primary_state = injected_primary_state
    }
}

fn game_state_to_primary_state(game_state: FilteredGameState, is_penalized: bool) -> PrimaryState {
    if is_penalized {
        if game_state == FilteredGameState::Finished {
            return PrimaryState::Finished;
        }
        PrimaryState::Penalized
    } else {
        match game_state {
            FilteredGameState::Initial => PrimaryState::Initial,
            FilteredGameState::Ready => PrimaryState::Ready,
            FilteredGameState::Set => PrimaryState::Set,
            FilteredGameState::Playing { .. } => PrimaryState::Playing,
            FilteredGameState::Finished => PrimaryState::Finished,
            FilteredGameState::Stop => PrimaryState::Stop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use types::robot_mode::{RobotMode, SequencedRobotMode};

    #[test]
    fn robot_mode_sets_safe_and_prepare_with_sequence_number() {
        let mut filter = PrimaryStateFilter::default();

        filter.update_with_robot_mode(SequencedRobotMode {
            mode: RobotMode::Prepare,
            sequence_number: 7,
        });

        assert_eq!(filter.primary_state, PrimaryState::Prepare);
        assert_eq!(filter.robot_mode_sequence_number, 7);

        filter.update_with_robot_mode(SequencedRobotMode {
            mode: RobotMode::Damping,
            sequence_number: 8,
        });

        assert_eq!(filter.primary_state, PrimaryState::Safe);
        assert_eq!(filter.robot_mode_sequence_number, 8);
    }

    #[test]
    fn buttons_only_handle_initial_and_playing_transitions() {
        let mut filter = PrimaryStateFilter::default();
        filter.update_with_robot_mode(SequencedRobotMode {
            mode: RobotMode::Prepare,
            sequence_number: 3,
        });

        filter.update_with_buttons(
            &Buttons {
                f1: Some(ButtonPressType::Short),
                stand: Some(ButtonPressType::Short),
                walking: None,
            },
            true,
        );

        assert_eq!(filter.primary_state, PrimaryState::Prepare);
        assert_eq!(filter.robot_mode_sequence_number, 3);

        filter.update_with_buttons(
            &Buttons {
                f1: None,
                stand: Some(ButtonPressType::Long),
                walking: None,
            },
            true,
        );

        assert_eq!(filter.primary_state, PrimaryState::Initial);
        assert_eq!(filter.robot_mode_sequence_number, 3);

        filter.update_with_buttons(
            &Buttons {
                f1: None,
                stand: None,
                walking: Some(ButtonPressType::Long),
            },
            true,
        );

        assert_eq!(filter.primary_state, PrimaryState::Playing);
        assert_eq!(filter.robot_mode_sequence_number, 3);
    }
}
