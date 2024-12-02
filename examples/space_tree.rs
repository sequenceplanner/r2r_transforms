use log::*;
// use utils::treeviz::vizualize_tree;
use r2r_transforms::utils::treeviz::vizualize_tree;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use r2r_transforms::*;
// use structopt::StructOpt;
use tokio::time::Duration;

pub static SPACE_TREE_BUFFER_MAINTAIN_RATE: u64 = 1; // milliseconds
pub static VISUALIZE_TREE_REFRESH_RATE: u64 = 100; // milliseconds

// fn handle_args() -> Args {
//     let args = ArgsCLI::from_args();
//     Args {
//         visualize: args.visualize,
//     }
// }

#[tokio::main]
async fn main() -> () {
    // let args = handle_args();

    fn initialize_logging() {
        std::env::set_var("RUST_LOG", "warn");
        let _ = env_logger::builder().is_test(true).try_init();
    }

    initialize_logging();

    log::info!("Starting the test_deserialize_transform_stamped test...");

    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR is not set");

    let path = format!("{}/examples/data", manifest_dir);
    let buffer = SpaceTreeServer::new("test");
    buffer.load_scenario(&path, false);
    buffer.apply_changes();

    // let buffer = Arc::new(Mutex::new(HashMap::<String, TransformStamped>::new()));
    // let buffer_clone = buffer.clone();
    // tokio::task::spawn(async move {
    //     match maintain_space_tree_buffer(&buffer_clone, SPACE_TREE_BUFFER_MAINTAIN_RATE).await {
    //         Ok(()) => (),
    //         Err(e) => error!("Space tree buffer maintainer failed with: '{}'.", e),
    //     };
    // });

    let buffer_clone = buffer.clone();
    // if args.visualize {
        tokio::task::spawn(async move {
            match vizualize_tree(buffer_clone, VISUALIZE_TREE_REFRESH_RATE).await {
                Ok(()) => (),
                Err(e) => error!("Space tree buffer maintainer failed with: '{}'.", e),
            };
        });
    // }

    let handle = std::thread::spawn(move || loop {
        std::thread::sleep(Duration::from_millis(1000));
    });

    handle.join().unwrap();
}