use nalgebra::Isometry3;
use crate::*;
use std::{
    collections::HashMap, sync::{Arc, Mutex}
};
use log;

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
pub struct SpaceTreeServer {
    pub name: String,
    // These are the transforms that we are in control of
    pub local_buffer: Arc<Mutex<HashMap<String, TransformStamped>>>,
    // // These are all the transforms that exist when connected to the ROS world via /tf and /tf_static
    // pub global_buffer: Arc<Mutex<HashMap<String, TransformStamped>>>,
    // We are only allowed to perform updates ont the local buffer
    pending_updates: Arc<Mutex<HashMap<String, UpdateContext>>>,
}

impl SpaceTreeServer {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            local_buffer: Arc::new(Mutex::new(HashMap::new())),
            // global_buffer: Arc::new(Mutex::new(HashMap::new())),
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
                // Self::apply_changes(&self);
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
        // log::info!("Pending update: Insert transform with name '{}'", name);
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
        let buffer = self.local_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !buffer.contains_key(name) {
            log::info!(
                "Can't rename the frame '{}', buffer doesn't contain it.",
                name
            );
            return;
        }

        if buffer.contains_key(rename_to) {
            log::info!(
                "Can't rename the frame '{name}' to '{rename_to}', buffer already contains '{rename_to}'.",
                
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
        // let global_buffer = self.global_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !local_buffer.contains_key(name) {
            log::info!(
                "Can't reparent the frame '{}', buffer doesn't contain it.",
                name
            );
            return;
        }

        // if !global_buffer.contains_key(reparent_to) {
        //     log::info!(
        //         "Can't reparent the frame '{name}' to '{reparent_to}', reparent frame '{reparent_to}' doesn't exist.",
                
        //     );
        //     return;
        // }

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
        let local_buffer = self.local_buffer.lock().unwrap();
        let mut pending_updates = self.pending_updates.lock().unwrap();

        if !local_buffer.contains_key(name) {
            log::info!(
                "Can't clone the frame '{}', buffer doesn't contain it.",
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

        log::info!("Pending update: Delete all transforms.");
    }

    pub fn lookup_transform(&self, parent_frame_id: &str, child_frame_id: &str) -> Option<TransformStamped> {
        let buffer = self.local_buffer.lock().unwrap();
        match get_tree_root(&buffer) {
            Some(root) => {
                lookup_transform_with_root(parent_frame_id, child_frame_id, &root, &self.local_buffer)
            },
            None => {
                lookup_transform_with_root(parent_frame_id, child_frame_id, "world", &self.local_buffer)
            }
        }
        
    }

    pub fn lookup_with_root(&self, parent_frame_id: &str, child_frame_id: &str, root_frame_id: &str) -> Option<TransformStamped> {
        lookup_transform_with_root(parent_frame_id, child_frame_id, root_frame_id, &self.local_buffer)
    }

    pub fn get_local_transform_names(&self) -> Vec<String> {
        let buffer = self.local_buffer.lock().unwrap();
        buffer.keys().map(|k| k.to_owned()).collect::<Vec<String>>()
    }

    /// Applies pending updates to the transform buffer.
    /// TODO: Sort out the connection with ROS /tf
    pub fn apply_changes(&self) {
        let mut buffer = self.local_buffer.lock().unwrap().clone();
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
                    } else if let Some(_) = buffer.get(name) {
                        log::info!("Transform '{}' already exists, not added.", name);
                    } else {
                        let transform = update_context.transform.clone();
                        if check_would_produce_cycle(&transform, &buffer) {
                            log::info!("Transform '{}' would produce cycle, not added.", name);
                        } else {
                            buffer.insert(name.to_string(), transform);
                            log::info!("Inserted transform '{name}'.");
                        }
                    }
                }
                UpdateType::Move => {
                    if let Some(transform) = buffer.get_mut(name) {
                        transform.transform = update_context.transform.transform.clone();
                        log::info!("Moved transform '{name}'.");
                    } else {
                        log::info!("Can't move transform '{}' because it doesn't exist.", name);
                    }
                }
                UpdateType::Remove => {
                    if let Some(_) = buffer.get(name) {
                        buffer.remove(name);
                        log::info!("Removed transform '{name}'.");
                    } else {
                        log::info!(
                            "Can't remove transform '{}' because it doesn't exist.",
                            name
                        );
                    }
                }
                UpdateType::Rename => {
                    if let Some(transform) = buffer.clone().get(name) {
                        let mut temp = transform.clone();
                        temp.child_frame_id = update_context.transform.child_frame_id.clone();
                        buffer.remove(name);
                        buffer.insert(
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
                    if let Some(transform) = buffer.get(name) {
                        let mut temp = transform.clone();
                        let old_parent = temp.parent_frame_id;
                        temp.parent_frame_id = update_context.transform.parent_frame_id.clone();
                        if check_would_produce_cycle(&temp, &buffer) {
                            log::info!("Transform '{}' would produce cycle if reparented, no action taken.", name);
                        } else {
                            let new_transform = Self::lookup_transform(&self, &temp.parent_frame_id, &temp.child_frame_id);
                            temp.transform = new_transform.unwrap().transform;
                            buffer.insert(name.clone(), temp);
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
                    if let Some(transform) = buffer.get(name) {
                        let mut new_transform = transform.clone();
                        new_transform.child_frame_id =
                            update_context.transform.child_frame_id.clone();
                        buffer.insert(
                            update_context.transform.child_frame_id.clone(),
                            new_transform.clone(),
                        );
                        log::info!("Cloned transform '{name}' as '{}'.", new_transform.child_frame_id);
                    } else {
                        log::info!("Can't clone transform '{}' because it doesn't exist.", name);
                    }
                }
                UpdateType::DeleteAll => {
                    buffer.clear();
                    log::info!("All transforms deleted from the buffer.");
                }
            }
        }

        pending_updates.clear();
        *self.local_buffer.lock().unwrap() = buffer;
    }
}
