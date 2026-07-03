use std::sync::Arc;
use std::{boxed::Box, future::Future, pin::Pin};

use color_eyre::{Result, eyre::Context as _};

use booster::{ImuState, LowState, MotorState};
use kinematics::joints::Joints;
use log::error;
use ros_z::{prelude::*, time::Time};

pub fn run_boxed(ctx: Arc<Context>) -> Pin<Box<dyn Future<Output = Result<()>> + Send>> {
    Box::pin(run(ctx))
}

async fn run(ctx: Arc<Context>) -> Result<()> {
    let node = ctx.create_node("low_state_bridge").build().await?;

    let zenoh_session = ctx.session();

    let low_state_sub = zenoh_session
        .declare_subscriber("rt/low_state")
        .await
        .map_err(|error| color_eyre::eyre::eyre!("{error}"))?;

    let low_state_pub = node
        .publisher::<LowState>("inputs/low_state")
        .build()
        .await?;
    let imu_state_pub = node
        .publisher::<ImuState>("inputs/imu_state")
        .build()
        .await?;
    let serial_motor_states_pub = node
        .publisher::<Joints<MotorState>>("inputs/serial_motor_states")
        .build()
        .await?;
    let parallel_motor_states_pub = node
        .publisher::<Option<Joints<MotorState>>>("inputs/parallel_motor_states")
        .build()
        .await?;

    loop {
        let low_state_sample = low_state_sub
            .recv_async()
            .await
            .map_err(|error| color_eyre::eyre::eyre!("{error}"))?;

        let source_time = low_state_sample
            .timestamp()
            .map(|timestamp| Time::from_wallclock(timestamp.get_time().to_system_time()))
            .unwrap_or_else(|| {
                error!("No zenoh timestamp for low state. Falling back to node time");
                node.clock().now()
            });

        let low_state: LowState = cdr::deserialize(&low_state_sample.payload().to_bytes())
            .wrap_err("deserialization failed")?;

        let imu_state = low_state.imu_state;
        let serial_motor_states = low_state.serial_motor_states()?;
        let parallel_motor_states = low_state.parallel_motor_states().ok();

        low_state_pub
            .publish_with_source_time(&low_state, source_time)
            .await?;
        imu_state_pub
            .publish_with_source_time(&imu_state, source_time)
            .await?;
        serial_motor_states_pub
            .publish_with_source_time(&serial_motor_states, source_time)
            .await?;
        parallel_motor_states_pub
            .publish_with_source_time(&parallel_motor_states, source_time)
            .await?;
    }
}
