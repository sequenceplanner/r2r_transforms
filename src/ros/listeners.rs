// updates the buffer with active frames from the tf topic
// TODO: if a stale active frame is on the tf for some reason, don't include it
// TODO: active frames should be merged with extra data from broadcaster.
pub async fn active_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        FrameData {
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: t.transform.clone(),
                            extra_data: ExtraData {
                                active: Some(true),
                                time_stamp: Some(t.header.stamp.clone()),
                                ..Default::default()
                            },
                        },
                    );
                });
                *buffered_frames.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!(node_id, "Subscriber did not get the message?");
            }
        }
    }
}

// updates the buffer with static frames from the tf_static topic
pub async fn static_tf_listener_callback(
    mut subscriber: impl Stream<Item = TFMessage> + Unpin,
    buffered_frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        match subscriber.next().await {
            Some(message) => {
                let mut frames_local = buffered_frames.lock().unwrap().clone();
                message.transforms.iter().for_each(|t| {
                    // static frames are always true, so we don't need to check their timestamp
                    frames_local.insert(
                        t.child_frame_id.clone(),
                        FrameData {
                            parent_frame_id: t.header.frame_id.clone(),
                            child_frame_id: t.child_frame_id.clone(),
                            transform: t.transform.clone(),
                            extra_data: ExtraData {
                                active: Some(false),
                                time_stamp: Some(t.header.stamp.clone()),
                                ..Default::default()
                            },
                        },
                    );
                });
                *buffered_frames.lock().unwrap() = frames_local;
            }
            None => {
                r2r::log_error!(node_id, "Subscriber did not get the message?");
            }
        }
    }
}