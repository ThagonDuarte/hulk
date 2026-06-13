use std::{future::Future, sync::Arc, time::Duration};

use color_eyre::eyre::eyre;
use color_eyre::{Result, eyre::Report};
use eframe::egui::Context as EguiContext;
use ros_z::{Message, node::Node, pubsub::PublicationId, qos::QosProfile, time::Time};
use ros_z_debug::{DebugEvent, ManagerOptions, RetentionPolicy, SampleRecord, SubscriptionManager};
use tokio::{runtime::Runtime, sync::watch, time};

use crate::{
    backend::latency::trace_forward_latency,
    value_buffer::{Buffer, BufferHandle, Datum},
};

const SUBSCRIBE_RETRY_DELAY: Duration = Duration::from_secs(1);

type TypedBuffer<T> = Buffer<T, Report>;

struct ActiveSubscription<T>
where
    T: Message + Clone,
{
    _manager: SubscriptionManager,
    handle: ros_z_debug::SubscriptionHandle<T>,
    retention: RetentionPolicy,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RebuildReason {
    Retarget,
    RetentionChanged,
    Retry,
}

pub fn subscribe_value<T>(
    runtime: &Runtime,
    node: Arc<Node>,
    target_namespace: watch::Receiver<String>,
    egui_context: EguiContext,
    selector: impl Into<String>,
    history: Duration,
    qos: Option<QosProfile>,
) -> BufferHandle<T>
where
    T: Message + Clone,
    T::Codec: Send + Sync,
{
    let (buffer, handle) = Buffer::new(history);
    runtime.spawn(run_typed_buffer(
        node,
        target_namespace,
        egui_context,
        selector.into(),
        qos,
        buffer,
    ));
    handle
}

async fn run_typed_buffer<T>(
    node: Arc<Node>,
    mut target_namespace: watch::Receiver<String>,
    egui_context: EguiContext,
    selector: String,
    qos: Option<QosProfile>,
    buffer: TypedBuffer<T>,
) where
    T: Message + Clone,
    T::Codec: Send + Sync,
{
    let mut clear_on_rebuild = true;

    loop {
        if buffer.is_closed() {
            break;
        }

        if clear_on_rebuild {
            buffer.replace(Vec::new());
            egui_context.request_repaint();
        }

        let namespace = target_namespace.borrow_and_update().clone();
        let retention = retention_policy(buffer.history().await);
        let subscription =
            subscribe_typed::<T>(node.clone(), namespace, selector.clone(), retention, qos);
        tokio::pin!(subscription);

        let active_subscription = tokio::select! {
            result = &mut subscription => result,
            changed = target_namespace.changed() => {
                if changed.is_err() {
                    break;
                }
                clear_on_rebuild = true;
                continue;
            }
            _ = buffer.closed() => break,
        };

        let rebuild_reason = match active_subscription {
            Ok(active_subscription) => {
                if buffer.clear_error() {
                    egui_context.request_repaint();
                }
                let Some(rebuild_reason) = forward_subscription(
                    active_subscription,
                    &mut target_namespace,
                    &buffer,
                    &egui_context,
                )
                .await
                else {
                    break;
                };
                rebuild_reason
            }
            Err(error) => {
                buffer.send_error(error);
                egui_context.request_repaint();
                let Some(rebuild_reason) =
                    wait_for_retry_or_retarget(&mut target_namespace, &buffer).await
                else {
                    break;
                };
                rebuild_reason
            }
        };

        clear_on_rebuild = should_clear_on_rebuild(rebuild_reason);
    }
}

async fn subscribe_typed<T>(
    node: Arc<Node>,
    target_namespace: String,
    selector: String,
    retention: RetentionPolicy,
    qos: Option<QosProfile>,
) -> Result<ActiveSubscription<T>>
where
    T: Message + Clone,
    T::Codec: Send + Sync,
{
    let manager = SubscriptionManager::new(
        node,
        ManagerOptions::with_target_namespace(target_namespace)?,
    );
    let mut builder = manager.subscribe_typed::<T>(selector).retention(retention);
    if let Some(qos) = qos {
        builder = builder.qos(qos);
    }
    let handle = builder.build().await?;

    Ok(ActiveSubscription {
        _manager: manager,
        handle,
        retention,
    })
}

async fn forward_subscription<T>(
    mut active_subscription: ActiveSubscription<T>,
    target_namespace: &mut watch::Receiver<String>,
    buffer: &TypedBuffer<T>,
    egui_context: &EguiContext,
) -> Option<RebuildReason>
where
    T: Message + Clone,
{
    let mut history_changes = buffer.subscribe_history();

    if let Some(rebuild_reason) = subscription_event_rebuild_reason(
        active_subscription.retention,
        drain_events(&active_subscription, buffer, egui_context),
        async { retention_policy(buffer.history().await) },
    )
    .await
    {
        return Some(rebuild_reason);
    }

    loop {
        tokio::select! {
            changed = active_subscription.handle.changed() => {
                if changed.is_err() {
                    return None;
                }
                if let Some(rebuild_reason) = subscription_event_rebuild_reason(
                    active_subscription.retention,
                    drain_events(
                        &active_subscription,
                        buffer,
                        egui_context,
                    ),
                    async { retention_policy(buffer.history().await) },
                ).await {
                    return Some(rebuild_reason);
                }
            }
            changed = history_changes.changed() => {
                if changed.is_err() {
                    return None;
                }
                if let Some(rebuild_reason) = subscription_event_rebuild_reason(
                    active_subscription.retention,
                    drain_events(
                        &active_subscription,
                        buffer,
                        egui_context,
                    ),
                    async { retention_policy(buffer.history().await) },
                ).await {
                    return Some(rebuild_reason);
                }
            }
            changed = target_namespace.changed() => return changed.ok().map(|()| RebuildReason::Retarget),
            _ = buffer.closed() => return None,
        }
    }
}

async fn drain_events<T>(
    active_subscription: &ActiveSubscription<T>,
    buffer: &TypedBuffer<T>,
    egui_context: &EguiContext,
) where
    T: Message + Clone,
{
    let events = active_subscription.handle.drain_events();
    if events.is_empty() {
        return;
    }

    let has_identity_events = events
        .iter()
        .any(|event| matches!(event, DebugEvent::ValueRetained { .. }));
    let mut requested_repaint = false;

    for event in events {
        match event {
            DebugEvent::ValueUpdated => {
                if !has_identity_events && let Some(record) = active_subscription.handle.latest() {
                    forward_record(record, buffer).await;
                    requested_repaint = true;
                }
            }
            DebugEvent::ValueRetained {
                source_time,
                publication_id,
            } => {
                if let Some(record) = retained_record(
                    &active_subscription.handle,
                    active_subscription.retention,
                    source_time,
                    publication_id,
                ) {
                    forward_record(record, buffer).await;
                    requested_repaint = true;
                }
            }
            DebugEvent::Diagnostic(message) => {
                buffer.send_error(eyre!(message));
                requested_repaint = true;
            }
            DebugEvent::StatusChanged => {}
            _ => {}
        }
    }

    if requested_repaint {
        egui_context.request_repaint();
    }
}

async fn subscription_event_rebuild_reason(
    active_retention: RetentionPolicy,
    drain_events: impl Future<Output = ()>,
    current_retention: impl Future<Output = RetentionPolicy>,
) -> Option<RebuildReason> {
    drain_events.await;
    (current_retention.await != active_retention).then_some(RebuildReason::RetentionChanged)
}

fn should_clear_on_rebuild(rebuild_reason: RebuildReason) -> bool {
    matches!(rebuild_reason, RebuildReason::Retarget)
}

async fn forward_record<T>(record: Arc<SampleRecord<T>>, buffer: &TypedBuffer<T>)
where
    T: Message + Clone,
{
    trace_forward_latency("typed", &record);
    buffer
        .push(Datum {
            timestamp: record.source_time.to_wallclock(),
            value: record.value.clone(),
        })
        .await;
}

fn retained_record<T>(
    handle: &ros_z_debug::SubscriptionHandle<T>,
    retention: RetentionPolicy,
    source_time: Time,
    publication_id: PublicationId,
) -> Option<Arc<SampleRecord<T>>>
where
    T: Message + Clone,
{
    match retention {
        RetentionPolicy::TimeWindow(_) => handle
            .window(source_time, source_time)
            .into_iter()
            .find(|record| record.publication_id == publication_id),
        RetentionPolicy::LatestOnly => handle.latest().filter(|record| {
            record.source_time == source_time && record.publication_id == publication_id
        }),
        _ => handle.latest().filter(|record| {
            record.source_time == source_time && record.publication_id == publication_id
        }),
    }
}

async fn wait_for_retry_or_retarget<T>(
    target_namespace: &mut watch::Receiver<String>,
    buffer: &TypedBuffer<T>,
) -> Option<RebuildReason> {
    let retry = time::sleep(SUBSCRIBE_RETRY_DELAY);
    tokio::pin!(retry);

    tokio::select! {
        _ = &mut retry => Some(RebuildReason::Retry),
        changed = target_namespace.changed() => changed.ok().map(|()| RebuildReason::Retarget),
        _ = buffer.closed() => None,
    }
}

fn retention_policy(history: Duration) -> RetentionPolicy {
    if history.is_zero() {
        return RetentionPolicy::LatestOnly;
    }

    match RetentionPolicy::time_window(history) {
        Ok(retention) => retention,
        Err(_) => RetentionPolicy::LatestOnly,
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use eframe::egui::Context as EguiContext;
    use ros_z::context::ContextBuilder;
    use tokio::{runtime::Builder, sync::watch};

    use super::*;

    #[test]
    fn typed_buffer_forwards_published_sample() {
        let runtime = Builder::new_multi_thread().enable_all().build().unwrap();
        runtime.block_on(async {
            let context = ContextBuilder::default()
                .disable_multicast_scouting()
                .with_json("connect/endpoints", serde_json::json!([]))
                .build()
                .await
                .unwrap();
            let publisher_node = context.create_node("twix_typed_pub").build().await.unwrap();
            let subscriber_node =
                Arc::new(context.create_node("twix_typed_sub").build().await.unwrap());
            let publisher = publisher_node
                .publisher::<String>("twix_debug_text")
                .unwrap()
                .build()
                .await
                .unwrap();
            let (_namespace_sender, namespace_receiver) = watch::channel("/".to_string());
            let buffer = subscribe_value::<String>(
                &runtime,
                subscriber_node,
                namespace_receiver,
                EguiContext::default(),
                "twix_debug_text",
                Duration::ZERO,
                None,
            );

            assert!(
                publisher
                    .wait_for_subscribers(1, Duration::from_secs(1))
                    .await
            );
            publisher.publish(&"hello".to_string()).await.unwrap();

            let deadline = tokio::time::Instant::now() + Duration::from_secs(1);
            loop {
                if buffer.get_last_value().unwrap() == Some("hello".to_string()) {
                    break;
                }
                assert!(
                    tokio::time::Instant::now() < deadline,
                    "timed out waiting for Twix typed buffer"
                );
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        });
    }

    #[test]
    fn rebuild_clears_only_after_retarget() {
        assert!(should_clear_on_rebuild(RebuildReason::Retarget));
        assert!(!should_clear_on_rebuild(RebuildReason::RetentionChanged));
        assert!(!should_clear_on_rebuild(RebuildReason::Retry));
    }
}
