// Broadcast static frames
pub async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, FrameData>>>,
    // node_id: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        transforms_local.iter().for_each(|(_, v)| match v.active {
            Some(false) => {
                updated_transforms.push(TransformStamped {
                    header: Header {
                        stamp: time_stamp.clone(),
                        frame_id: v.parent_frame_id.clone(),
                    },
                    child_frame_id: v.child_frame_id.clone(),
                    transform: v.transform.clone(),
                });
            }
            Some(true) | None => (),
        });

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    NODE_ID,
                    "Static broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}