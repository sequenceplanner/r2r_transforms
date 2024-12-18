use nalgebra::Isometry3;
use crate::*;
use std::{
    collections::HashMap, sync::{Arc, Mutex}
};
use log;


// use r2r::std_msgs::msg::Header;
use r2r::tf2_msgs::msg::TFMessage;
// use r2r::Context;
use r2r::QosProfile;


/// Represents the type of update to perform on a transform.
#[derive(Clone, Debug)]
enum UpdateType {
    Add,
    Move,
    Remove,
    Rename,
    Reparent,
    Clone,
    DeleteAll,
}

/// Holds information about a transform update.
#[derive(Clone, Debug)]
struct UpdateContext {
    update_type: UpdateType,
    transform: TransformStamped,
}

/// A server that maintains a spatial tree buffer of transforms.
#[derive(Clone)]
pub struct RosSpaceTreeServer {
    pub name: String,
    // These are the transforms that we are in control of
    pub local_buffer: Arc<Mutex<HashMap<String, TransformStamped>>>,
    // These are all the transforms that exist when connected to the ROS world via /tf and /tf_static
    pub global_buffer: Arc<Mutex<HashMap<String, TransformStamped>>>,
    // We are only allowed to perform updates ont the local buffer
    pending_updates: Arc<Mutex<HashMap<String, UpdateContext>>>,
}

impl RosSpaceTreeServer {
    pub fn new(name: &str, node: &Arc<Mutex<r2r::Node>>) -> Self {

        let local_buffer = Arc::new(Mutex::new(HashMap::new()));
        let global_buffer = Arc::new(Mutex::new(HashMap::new()));

        let static_pub_timer = node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(STATIC_TF_BROADCAST_RATE))
            .expect("Failed to initialize static_pub_timer.");

        let static_frame_broadcaster = node
            .lock()
            .unwrap().create_publisher::<TFMessage>(
                "tf_static",
                QosProfile::transient_local(QosProfile::default()),
            ).expect("Failed to initialize static_frame_broadcaster.");

        let local_buffer_clone = local_buffer.clone();
        tokio::task::spawn(async move {
            match static_frame_broadcaster_callback(
                static_frame_broadcaster,
                static_pub_timer,
                &local_buffer_clone,
            )
            .await
            {
                Ok(()) => (),
                Err(e) => r2r::log_error!("r2r_transforms", "Static frame broadcaster failed with: '{}'.", e),
            };
        });

        let active_pub_timer = node
            .lock()
            .unwrap()
            .create_wall_timer(std::time::Duration::from_millis(ACTIVE_TF_BROADCAST_RATE))
            .expect("Failed to initialize active_pub_timer.");

        let active_frame_broadcaster = node
            .lock()
            .unwrap().create_publisher::<TFMessage>(
                "tf",
                QosProfile::transient_local(QosProfile::default()),
            ).expect("Failed to initialize active_frame_broadcaster.");

        let local_buffer_clone = local_buffer.clone();
        tokio::task::spawn(async move {
            match active_frame_broadcaster_callback(
                active_frame_broadcaster,
                active_pub_timer,
                &local_buffer_clone,
            )
            .await
            {
                Ok(()) => (),
                Err(e) => r2r::log_error!("r2r_transforms", "Active frame broadcaster failed with: '{}'.", e),
            };
        });

        let active_tf_listener =
            node.lock().unwrap().subscribe::<TFMessage>("tf", QosProfile::transient_local(QosProfile::default())).expect("Failed to initialize active_tf_listener.");
        let global_buffer_clone = global_buffer.clone();
        tokio::task::spawn(async move {
            match active_tf_listener_callback(
                active_tf_listener,
                &global_buffer_clone.clone(),
            )
            .await
            {
                Ok(()) => (),
                Err(e) => r2r::log_error!("r2r_transforms", "Active tf listener failed with: '{}'.", e),
            };
        });

        let static_tf_listener =
            node.lock().unwrap().subscribe::<TFMessage>("tf_static", QosProfile::best_effort(QosProfile::default())).expect("Failed to initialize static_tf_listener.");
        let global_buffer_clone = global_buffer.clone();
        tokio::task::spawn(async move {
            match static_tf_listener_callback(
                static_tf_listener,
                &global_buffer_clone.clone(),
            )
            .await
            {
                Ok(()) => (),
                Err(e) => r2r::log_error!("r2r_transforms", "Static tf listener failed with: '{}'.", e),
            };
        });

        let local_buffer_clone = local_buffer.clone();
        let global_buffer_clone = global_buffer.clone();
        Self {
            name: name.to_string(),
            local_buffer: local_buffer_clone,
            global_buffer: global_buffer_clone,
            pending_updates: Arc::new(Mutex::new(HashMap::new())),
        }
    }

    pub fn load_scenario(&self, scenario_path: &str, overlay: bool) {
        match list_frames_in_dir(scenario_path) {
            Ok(list) => {
                let frames = load_new_scenario(&list);
                if overlay {
                    frames.values().for_each(|frame| Self::insert_transform(&self, &frame.child_frame_id, frame.clone()));
                } else {
                    let buffer = self.local_buffer.lock().unwrap();
                    frames
                        .values()
                        .filter(|frame| buffer
                        .get(&frame.child_frame_id) == None)
                        .for_each(|frame| Self::insert_transform(&self, &frame.child_frame_id, frame.clone()));
                }
            },
            Err(_) => return
        }
    }

    pub fn insert_transform(&self, name: &str, transform: TransformStamped) {
        let mut pending_updates = self.pending_updates.lock().unwrap();

        // Add or update the pending update for the transform.
        let update_context =
            pending_updates
                .entry(name.to_string())
                .or_insert_with(|| UpdateContext {
                    update_type: UpdateType::Add,
                    transform: transform.clone(),
                });

        update_context.update_type = UpdateType::Add;
        update_context.transform = transform;

        log::info!("Pending update: Insert transform with name '{}'", name);
    }

    pub fn move_transform(&self, name: &str, pose: Isometry3<f64>) {
        let buffer = self.local_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !buffer.contains_key(name) {
            log::info!(
                "Can't move the frame '{}' to a new pose, buffer doesn't contain it.",
                name
            );
            return;
        }

        // Move the frame to a new pose
        let update_context =
            pending_updates
                .entry(name.to_string())
                .or_insert_with(|| UpdateContext {
                    update_type: UpdateType::Move,
                    transform: {
                        let mut tf = TransformStamped::default();
                        tf.transform = pose;
                        tf
                    },
                });

        update_context.update_type = UpdateType::Move;
        update_context.transform.transform = pose;

        log::info!("Pending update: Move transform with name '{}'", name);
    }

    pub fn remove_transform(&self, name: &str) {
        let buffer = self.local_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !buffer.contains_key(name) {
            log::info!(
                "Can't remove the frame '{}', buffer doesn't contain it.",
                name
            );
            return;
        }

        pending_updates.insert(
            name.to_string(),
            UpdateContext {
                update_type: UpdateType::Remove,
                transform: TransformStamped::default()
            },
        );

        log::info!("Pending update: Remove transform with name '{}'", name);

    }

    pub fn rename_transform(&self, name: &str, rename_to: &str) {
        let local_buffer = self.local_buffer.lock().unwrap();
        let global_buffer = self.global_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !local_buffer.contains_key(name) {
            log::info!(
                "Can't rename the frame '{}', buffer doesn't contain it.",
                name
            );
            return;
        }

        if global_buffer.contains_key(rename_to) {
            log::info!(
                "Can't rename the frame '{name}' to '{rename_to}', '{rename_to}' already exists.",
                
            );
            return;
        }

        pending_updates.insert(
            name.to_string(),
            UpdateContext {
                update_type: UpdateType::Rename,
                transform: {
                    let mut tf = TransformStamped::default();
                    tf.child_frame_id = rename_to.to_string();
                    tf
                },
            },
        );

        log::info!("Pending update: Rename transform with name '{name}' to '{rename_to}'.");

    }

    pub fn reparent_transform(&self, name: &str, reparent_to: &str) {
        let local_buffer = self.local_buffer.lock().unwrap();
        let global_buffer = self.global_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !local_buffer.contains_key(name) {
            log::info!(
                "Can't reparent the frame '{}', buffer doesn't contain it.",
                name
            );
            return;
        }

        if !global_buffer.contains_key(reparent_to) {
            log::info!(
                "Can't reparent the frame '{name}' to '{reparent_to}', reparent frame '{reparent_to}' doesn't exist.",
                
            );
            return;
        }

        pending_updates.insert(
            name.to_string(),
            UpdateContext {
                update_type: UpdateType::Reparent,
                transform: {
                    let mut tf = TransformStamped::default();
                    tf.parent_frame_id = reparent_to.to_string();
                    tf
                },
            },
        );

        log::info!("Pending update: Reparent transform with name '{name}' to '{reparent_to}'.");

    }

    pub fn clone_transform(&self, name: &str, clone_name: &str) {
        let global_buffer = self.global_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !global_buffer.contains_key(name){
            log::info!(
                "Can't clone the frame '{}', it doesn't exist.",
                name
            );
            return;
        }

        pending_updates.insert(
            name.to_string(),
            UpdateContext {
                update_type: UpdateType::Clone,
                transform: {
                    let mut tf = TransformStamped::default();
                    tf.child_frame_id = clone_name.to_string();
                    tf
                },
            },
        );

        log::info!("Pending update: Clone transform with name '{name}' to '{clone_name}'.");

    }

    pub fn delete_all_transforms(&self) {
        let mut pending_updates = self.pending_updates.lock().unwrap();
        pending_updates.clear();

        pending_updates.insert(
            "delete_all".to_string(),
            UpdateContext {
                update_type: UpdateType::DeleteAll,
                transform: TransformStamped::default(),
            },
        );

        log::info!("Pending update: Delete all (local) transforms.");
    }

    pub fn lookup_transform(&self, parent_frame_id: &str, child_frame_id: &str) -> Option<TransformStamped> {
        let buffer = self.global_buffer.lock().unwrap();
        match get_tree_root(&buffer) {
            Some(root) => {
                lookup_transform(parent_frame_id, child_frame_id, &root, &self.global_buffer)
            },
            None => {
                lookup_transform(parent_frame_id, child_frame_id, "world", &self.global_buffer)
            }
        }
        
    }

    pub fn lookup_with_root(&self, parent_frame_id: &str, child_frame_id: &str, root_frame_id: &str) -> Option<TransformStamped> {
        lookup_transform(parent_frame_id, child_frame_id, root_frame_id, &self.global_buffer)
    }

    pub fn get_all_transform_names(&self) -> Vec<String> {
        let buffer = self.global_buffer.lock().unwrap();
        buffer.keys().map(|k| k.to_owned()).collect::<Vec<String>>()
    }

    /// Applies pending updates to the transform buffer.
    pub fn apply_changes(&self) {
        let mut local_buffer = self.local_buffer.lock().unwrap().clone();
        let global_buffer = self.global_buffer.lock().unwrap().clone();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if pending_updates.is_empty() {
            log::info!("No changes to apply");
            return;
        }

        for (name, update_context) in pending_updates.iter() {
            match update_context.update_type {
                UpdateType::Add => {
                    if name != &update_context.transform.child_frame_id {
                        log::info!("Transform name '{name}' in buffer doesn't match the child_frame_id {}, they should be the same. Not added.", update_context.transform.child_frame_id);
                    } else if let Some(_) = global_buffer.get(name) {
                        log::info!("Transform '{}' already exists, not added.", name);
                    } else {
                        let transform = update_context.transform.clone();
                        if check_would_produce_cycle(&transform, &global_buffer) {
                            log::info!("Transform '{}' would produce cycle, not added.", name);
                        } else {
                            local_buffer.insert(name.to_string(), transform);
                            log::info!("Inserted transform '{name}'.");
                        }
                    }
                }
                UpdateType::Move => {
                    if let Some(transform) = local_buffer.get_mut(name) {
                        transform.transform = update_context.transform.transform.clone();
                        log::info!("Moved transform '{name}'.");
                    } else {
                        log::info!("Can't move transform '{}' because it doesn't exist.", name);
                    }
                }
                UpdateType::Remove => {
                    if let Some(_) = local_buffer.get(name) {
                        local_buffer.remove(name);
                        log::info!("Removed transform '{name}'.");
                    } else {
                        log::info!(
                            "Can't remove transform '{}' because it doesn't exist.",
                            name
                        );
                    }
                }
                UpdateType::Rename => {
                    if let Some(transform) = local_buffer.clone().get(name) {
                        let mut temp = transform.clone();
                        temp.child_frame_id = update_context.transform.child_frame_id.clone();
                        local_buffer.remove(name);
                        local_buffer.insert(
                            update_context.transform.child_frame_id.to_string(),
                            temp.clone(),
                        );
                        log::info!("Renamed transform '{name}' to '{}'.", update_context.transform.child_frame_id);
                    } else {
                        log::info!(
                            "Can't rename transform '{}' because it doesn't exist.",
                            name
                        );
                    }
                }
                UpdateType::Reparent => {
                    if let Some(transform) = local_buffer.get(name) {
                        let mut temp = transform.clone();
                        let old_parent = temp.parent_frame_id;
                        temp.parent_frame_id = update_context.transform.parent_frame_id.clone();
                        if check_would_produce_cycle(&temp, &global_buffer) {
                            log::info!("Transform '{}' would produce cycle if reparented, no action taken.", name);
                        } else {
                            temp.parent_frame_id = update_context.transform.parent_frame_id.clone();
                            local_buffer.insert(name.clone(), temp);
                            log::info!("Reparented transform '{name}' from '{}' to '{}'.", old_parent, update_context.transform.parent_frame_id);
                        }
                    } else {
                        log::info!(
                            "Can't reparent transform '{}' because it doesn't exist.",
                            name
                        );
                    }
                }
                UpdateType::Clone => {
                    if let Some(transform) = global_buffer.get(name) {
                        let mut new_transform = transform.clone();
                        new_transform.child_frame_id =
                            update_context.transform.child_frame_id.clone();
                        local_buffer.insert(
                            update_context.transform.child_frame_id.clone(),
                            new_transform.clone(),
                        );
                        log::info!("Cloned transform '{name}' as '{}'.", new_transform.child_frame_id);
                    } else {
                        log::info!("Can't clone transform '{}' because it doesn't exist.", name);
                    }
                }
                UpdateType::DeleteAll => {
                    local_buffer.clear();
                    log::info!("All transforms deleted from the (local) buffer.");
                }
            }
        }

        pending_updates.clear();
        *self.local_buffer.lock().unwrap() = local_buffer;
    }

    // This conditionally includes a method which implements r2r support
    // #[cfg(feature = "ros")]
    // pub async fn connect_to_ros(&self, node: &Arc<Mutex<r2r::Node>>) -> Result<(), Box<dyn std::error::Error>> {
    
    //     // let static_pub_timer =
    //     //     node.lock()
    //     // .unwrap().create_wall_timer(std::time::Duration::from_millis(100))?;
    //     // let static_frame_broadcaster = node
    //     // .lock()
    //     // .unwrap().create_publisher::<TFMessage>(
    //     //     "tf_static",
    //     //     QosProfile::transient_local(QosProfile::default()),
    //     // )?;
    //     // let broadcasted_frames_clone = self.buffer.clone();
    //     // tokio::task::spawn(async move {
    //     //     match static_frame_broadcaster_callback(
    //     //         static_frame_broadcaster,
    //     //         static_pub_timer,
    //     //         &broadcasted_frames_clone,
    //     //     )
    //     //     .await
    //     //     {
    //     //         Ok(()) => (),
    //     //         Err(e) => r2r::log_error!("r2r_transforms", "Static frame broadcaster failed with: '{}'.", e),
    //     //     };
    //     // });

    //     let active_pub_timer =
    //         node.lock()
    //     .unwrap().create_wall_timer(std::time::Duration::from_millis(100))?;
    //     let active_frame_broadcaster = node
    //     .lock()
    //     .unwrap().create_publisher::<TFMessage>(
    //         "tf",
    //         QosProfile::transient_local(QosProfile::default()),
    //     )?;
    //     let broadcasted_frames_clone = self.buffer.clone();
    //     tokio::task::spawn(async move {
    //         match active_frame_broadcaster_callback(
    //             active_frame_broadcaster,
    //             active_pub_timer,
    //             &broadcasted_frames_clone,
    //         )
    //         .await
    //         {
    //             Ok(()) => (),
    //             Err(e) => r2r::log_error!("r2r_transforms", "Active frame broadcaster failed with: '{}'.", e),
    //         };
    //     });

    //     Ok(())

    // }

    // pub async fn start_gui(&self) -> Result<(), Box<dyn std::error::Error>> {
    //     let frames = self.buffer.clone();
    //     run_tui(&frames);
    //     Ok(())
    // }

    // Perform the cyclic change here before adding all buffer U pending_updates
}
