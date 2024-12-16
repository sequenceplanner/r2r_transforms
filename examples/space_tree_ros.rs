use std::sync::{Arc, Mutex};

use log::*;
use r2r::Context;
use r2r_transforms::*;
use tokio::time::{Duration, Instant};

pub static VISUALIZE_TREE_REFRESH_RATE: u64 = 100; // milliseconds

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    fn initialize_logging() {
        std::env::set_var("RUST_LOG", "warn");
        let _ = env_logger::builder().is_test(true).try_init();
    }

    initialize_logging();

    log::info!("Starting the r2r_transforms example...");

    let context = Context::create()?;
    let node = r2r::Node::create(context, "r2r_transforms", "")?;
    let arc_node = Arc::new(Mutex::new(node));

    let buffer = SpaceTreeServer::new("test");

    let buffer_clone = buffer.clone();
    tokio::task::spawn(async move {
        match space_tree_manipulation_example(&buffer_clone).await {
            Ok(()) => (),
            Err(e) => error!("Space tree manipulation task failed with: '{}'.", e),
        };
    });

    let buffer_clone = buffer.clone();
    let arc_node_clone = arc_node.clone();
    tokio::task::spawn(async move {
        match buffer_clone.connect_to_ros(&arc_node_clone).await {
            Ok(()) => (),
            Err(e) => error!("Connecting the buffer to ros failed with: '{}'.", e),
        };
    });

    // Keep the node alive
    let arc_node_clone: Arc<Mutex<r2r::Node>> = arc_node.clone();
    let handle = std::thread::spawn(move || loop {
        arc_node_clone
            .lock()
            .unwrap()
            .spin_once(std::time::Duration::from_millis(100));
    });

    r2r::log_info!("r2r_transforms", "Node started.");

    handle.join().unwrap();

    Ok(())
}

pub async fn space_tree_manipulation_example(
    buffer: &SpaceTreeServer,
) -> Result<(), Box<dyn std::error::Error>> {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");

    let path = format!("{}/examples/data", manifest_dir);

    let _ = visualize_tree_once(&buffer);

    // buffer.connect_to_ros(node);

    // loop {
    tokio::time::sleep(Duration::from_millis(500)).await;
    buffer.load_scenario(&path, false);
    buffer.apply_changes();

    println!("Open RViz for visualizing the ROS connection.");
    tokio::time::sleep(Duration::from_millis(5000)).await;

    let _ = visualize_tree_once(&buffer);

    tokio::time::sleep(Duration::from_millis(500)).await;
    let new_transform = TransformStamped {
        time_stamp: Instant::now(),
        parent_frame_id: "frame_5".to_string(),
        child_frame_id: "frame_6".to_string(),
        transform: json_transform_to_isometry(JsonTransform::default()),
        json_metadata: "".to_string(),
    };
    
    buffer.insert_transform("frame_6", new_transform.clone());
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.insert_transform("frame_7", new_transform);
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.move_transform("frame_6", json_transform_to_isometry(JsonTransform::default()));
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);

    buffer.remove_transform("frame_7");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.rename_transform("frame_6", "frame_8");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.reparent_transform("frame_6", "child_7");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.clone_transform("frame_6", "child_7");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.clone_transform("frame_8", "frame_9");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.reparent_transform("frame_8", "child_7");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.reparent_transform("frame_3", "frame_8");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.reparent_transform("frame_8", "frame_3");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.remove_transform("frame_9");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    buffer.remove_transform("frame_5");
    buffer.apply_changes();

    let _ = visualize_tree_once(&buffer);
    tokio::time::sleep(Duration::from_millis(500)).await;

    // buffer.delete_all_transforms();
    // buffer.apply_changes();

    // let _ = visualize_tree_once(&buffer);
    // tokio::time::sleep(Duration::from_millis(500)).await;

    Ok(())
}
