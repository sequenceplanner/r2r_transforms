
use nalgebra::{Isometry3, Quaternion, UnitQuaternion, Vector3};
use r2r::geometry_msgs::msg::Transform;
use serde::Deserialize;
use serde_json::Value;
// use serde::Deserialize;
// use structopt::StructOpt;
use tokio::time::Instant;


#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct JsonTranslation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct JsonRotation {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Debug, Deserialize, Clone, PartialEq)]
pub struct JsonTransform {
    pub translation: JsonTranslation,
    pub rotation: JsonRotation,
}

impl JsonTransform {
    pub fn default() -> JsonTransform {
        JsonTransform {
            translation: JsonTranslation {
                x: 0.0,
                y: 0.0,
                z: 0.0
            },
            rotation: JsonRotation {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
            }
        }
    }
}

pub fn json_transform_to_isometry(json: JsonTransform) -> Isometry3<f64> {
    let translation = Vector3::new(json.translation.x, json.translation.y, json.translation.z);
    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        json.rotation.w,
        json.rotation.x,
        json.rotation.y,
        json.rotation.z,
    ));

    Isometry3::from_parts(translation.into(), rotation)
}

pub fn ros_transform_to_isometry(t: Transform) -> Isometry3<f64> {
    let translation = Vector3::new(t.translation.x, t.translation.y, t.translation.z);
    let rotation = UnitQuaternion::from_quaternion(Quaternion::new(
        t.rotation.w,
        t.rotation.x,
        t.rotation.y,
        t.rotation.z,
    ));

    Isometry3::from_parts(translation.into(), rotation)
}

// Isometry3 should be similar to the Transform message in ROS
#[derive(Debug, Clone, PartialEq)]
pub struct TransformStamped {
    pub active: bool,
    // #[serde(deserialize_with = "deserialize_instant")]
    pub time_stamp: Instant,
    pub parent_frame_id: String,
    pub child_frame_id: String,
    // #[serde(deserialize_with = "deserialize_isometry_3_f64")]
    pub transform: Isometry3<f64>,
    // pub json_metadata: String,
    pub metadata: Value
}

impl TransformStamped {
    pub fn default() -> TransformStamped {
        TransformStamped {
            active: true,
            time_stamp: Instant::now(),
            parent_frame_id: "".to_string(),
            child_frame_id: "".to_string(),
            transform: Isometry3::default(),
            metadata: Value::default()
            // json_metadata: "".to_string()
        }
    }
}