use crate::TransformStamped;
use r2r::geometry_msgs::msg::{
    Quaternion, Transform, TransformStamped as TransformStampedMsg, Vector3,
};
use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
// use serde_json::Value;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

pub async fn static_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, TransformStamped>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];

        for (_, t) in &transforms_local {
            // if let Ok(metadata) = serde_json::from_str::<Value>(&t.json_metadata) {
            //     if let Some(active) = metadata["active"].as_bool() {
            //         if !active {
            //             if let Some(local) = metadata["local"].as_bool() {
            //                 if !local {
            //                     continue;
            //                 }
            //             }
                        updated_transforms.push(TransformStampedMsg {
                            header: Header {
                                stamp: time_stamp.clone(),
                                frame_id: t.parent_frame_id.clone(),
                            },
                            child_frame_id: t.child_frame_id.clone(),
                            transform: Transform {
                                translation: Vector3 {
                                    x: t.transform.translation.x,
                                    y: t.transform.translation.y,
                                    z: t.transform.translation.z,
                                },
                                rotation: Quaternion {
                                    x: t.transform.rotation.coords.x,
                                    y: t.transform.rotation.coords.y,
                                    z: t.transform.rotation.coords.z,
                                    w: t.transform.rotation.coords.w,
                                },
                            },
                        });
        //             } else {
        //                 continue;
        //             }
        //         }
        //     }
        }

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "r2r_transforms",
                    "Static broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}

pub async fn active_frame_broadcaster_callback(
    publisher: r2r::Publisher<TFMessage>,
    mut timer: r2r::Timer,
    frames: &Arc<Mutex<HashMap<String, TransformStamped>>>,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut clock = r2r::Clock::create(r2r::ClockType::RosTime).unwrap();
        let now = clock.get_now().unwrap();
        let time_stamp = r2r::Clock::to_builtin_time(&now);

        let transforms_local = frames.lock().unwrap().clone();
        let mut updated_transforms = vec![];



        for (_, t) in &transforms_local {
            // if let Ok(metadata) = serde_json::from_str::<Value>(&t.json_metadata) {
            //     // If the 'active' field is present and is false, skip this transform
            //     if let Some(false) = metadata["active"].as_bool() {
            //         continue;
            //     }
        
            //     // If the 'local' field is present and is false, skip this transform
            //     if let Some(false) = metadata["local"].as_bool() {
            //         continue;
            //     }
            // }
        
            updated_transforms.push(TransformStampedMsg {
                header: Header {
                    stamp: time_stamp.clone(),
                    frame_id: t.parent_frame_id.clone(),
                },
                child_frame_id: t.child_frame_id.clone(),
                transform: Transform {
                    translation: Vector3 {
                        x: t.transform.translation.x,
                        y: t.transform.translation.y,
                        z: t.transform.translation.z,
                    },
                    rotation: Quaternion {
                        x: t.transform.rotation.coords.x,
                        y: t.transform.rotation.coords.y,
                        z: t.transform.rotation.coords.z,
                        w: t.transform.rotation.coords.w,
                    },
                },
            });
        }




        // for (_, t) in &transforms_local {
        //     if let Ok(metadata) = serde_json::from_str::<Value>(&t.json_metadata) {
        //         if let Some(active) = metadata["active"].as_bool() {
        //             if active {
        //                 if let Some(local) = metadata["local"].as_bool() {
        //                     if !local {
        //                         continue;
        //                     }
        //                 }
        //             } else {
        //                 continue;
        //             }
        //         }
        //     }

        //     updated_transforms.push(TransformStampedMsg {
        //         header: Header {
        //             stamp: time_stamp.clone(),
        //             frame_id: t.parent_frame_id.clone(),
        //         },
        //         child_frame_id: t.child_frame_id.clone(),
        //         transform: Transform {
        //             translation: Vector3 {
        //                 x: t.transform.translation.x,
        //                 y: t.transform.translation.y,
        //                 z: t.transform.translation.z,
        //             },
        //             rotation: Quaternion {
        //                 x: t.transform.rotation.coords.x,
        //                 y: t.transform.rotation.coords.y,
        //                 z: t.transform.rotation.coords.z,
        //                 w: t.transform.rotation.coords.w,
        //             },
        //         },
        //     })
        // }

        let msg = TFMessage {
            transforms: updated_transforms,
        };

        match publisher.publish(&msg) {
            Ok(()) => (),
            Err(e) => {
                r2r::log_error!(
                    "r2r_transforms",
                    "Active broadcaster failed to send a message with: '{}'",
                    e
                );
            }
        };
        timer.tick().await?;
    }
}
