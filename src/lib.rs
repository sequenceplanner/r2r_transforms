pub static STATIC_TF_BROADCAST_RATE: u64 = 100;
pub static ACTIVE_TF_BROADCAST_RATE: u64 = 10;
pub static MAX_TRANSFORM_CHAIN: u64 = 1000;
pub static MAX_RECURSION_DEPTH: u64 = 1000;

pub mod core;
pub use core::structs::*;
pub use core::space_tree::*;

pub mod utils;
pub use utils::cycles::*;
pub use utils::lookup::*;
pub use utils::loading::*;
pub use utils::treeviz::*;

// This conditionally includes a module which implements r2r support.
// #[cfg(feature = "ros")]
pub mod ros;
// #[cfg(feature = "ros")]
pub use ros::broadcasters::*;
pub use ros::listeners::*;
pub use ros::space_tree_ros::*;