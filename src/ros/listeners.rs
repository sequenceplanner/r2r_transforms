use crate::*;
use futures::{Stream, StreamExt};
use r2r::tf2_msgs::msg::TFMessage;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use tokio::time::Instant;

// updates the buffer with active frames from the tf topic
// TODO: if a stale active frame is on the tf for some reason, don't include it
// TODO: active frames should be merged with extra data from broadcaster.
pub async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    global_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = global_buffer.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        TransformStamped {
                            active: true,
                            time_stamp: Instant::now(), // TODO: use t.header.stamp,
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: ros_transform_to_isometry(t.transform.clone()),
                            json_metadata: "".to_string()
                        },
                    );
                });
                *global_buffer.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!("r2r_transforms", "Subscriber did not get the message?");
            }
        }
    }
}

pub async fn static_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    global_buffer: &Arc<Mutex<HashMap<String, TransformStamped>>>
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = global_buffer.lock().unwrap().clone();
                println!("{:?}", frames_local.keys());
                message.transforms.iter().for_each(|t| {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        TransformStamped {
                            active: false,
                            time_stamp: Instant::now(), // TODO: use t.header.stamp,
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: ros_transform_to_isometry(t.transform.clone()),
                            json_metadata: "".to_string()
                        },
                    );
                });
                *global_buffer.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!("r2r_transforms", "Subscriber did not get the message?");
            }
        }
    }
}