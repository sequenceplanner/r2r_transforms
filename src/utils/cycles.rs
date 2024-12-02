use utils::lookup::get_frame_children;

use crate::*;
use std::collections::HashMap;

pub static MAX_TRANSFORM_CHAIN: u64 = 1000;

// Check for cycles in the tree segment starting from this frame
pub fn is_cyclic(frame: &str, buffer: &HashMap<String, TransformStamped>) -> bool {
    let mut stack = vec![frame.to_string()];
    let mut visited = vec![];

    loop {
        match stack.pop() {
            Some(current_frame) => {
                if visited.contains(&current_frame) && buffer.contains_key(&current_frame) {
                    break true;
                } else {
                    visited.push(current_frame.clone());

                    for child in get_frame_children(&current_frame, buffer) {
                        stack.push(child.1.child_frame_id);
                    }
                }
            }
            None => break false,
        }
    }
}

// Check for all cycles including all frames even if tree is segmented
pub fn is_cyclic_all(frames: &HashMap<String, TransformStamped>) -> bool {
    for (k, _) in frames {
        if is_cyclic(k, frames) {
            return true;
        } else {
            continue;
        }
    }
    false
}

// check if adding the frame to the tree would produce a cycle
pub fn check_would_produce_cycle(
    frame: &TransformStamped,
    buffer: &HashMap<String, TransformStamped>,
) -> bool {
    let mut buffer_local = buffer.clone();
    buffer_local.insert(frame.child_frame_id.clone(), frame.clone());
    is_cyclic_all(&buffer_local)
}

#[cfg(test)]
mod tests {

    use nalgebra::Isometry3;
    use utils::cycles::{check_would_produce_cycle, is_cyclic, is_cyclic_all};
    use std::collections::HashMap;
    use tokio::time::Instant;

    use crate::*;

    fn dummy_1_frame() -> TransformStamped {
        TransformStamped {
            time_stamp: Instant::now(),
            parent_frame_id: "world".to_string(),
            child_frame_id: "dummy_1".to_string(),
            transform: Isometry3::default(),
            json_metadata: String::default(),
        }
    }

    fn dummy_2_frame() -> TransformStamped {
        TransformStamped {
            time_stamp: Instant::now(),
            parent_frame_id: "dummy_1".to_string(),
            child_frame_id: "dummy_2".to_string(),
            transform: Isometry3::default(),
            json_metadata: String::default(),
        }
    }

    fn dummy_3_frame() -> TransformStamped {
        TransformStamped {
            time_stamp: Instant::now(),
            parent_frame_id: "dummy_2".to_string(),
            child_frame_id: "dummy_3".to_string(),
            transform: Isometry3::default(),
            json_metadata: String::default(),
        }
    }

    #[test]
    fn test_is_not_cyclic() {
        let mut buffer = HashMap::<String, TransformStamped>::new();
        buffer.insert("dummy_1".to_string(), dummy_1_frame());

        //          w
        //          |
        //          d1

        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, false);

        buffer.insert("dummy_2".to_string(), dummy_2_frame());

        //          w
        //          |
        //          d1
        //          |
        //          d2

        let res = is_cyclic("world", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_2", &buffer);
        assert_eq!(res, false);
    }

    #[test]
    fn test_is_cyclic() {
        let mut buffer = HashMap::<String, TransformStamped>::new();
        buffer.insert("dummy_1".to_string(), dummy_1_frame());
        buffer.insert("dummy_2".to_string(), dummy_2_frame());
        buffer.insert(
            "dummy_1".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_2".to_string(),
                child_frame_id: "dummy_1".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        //          w
        //          
        //          d1
        //          ||
        //          d2

        let res = is_cyclic("world", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, true);
        let res = is_cyclic("dummy_2", &buffer);
        assert_eq!(res, true);
    }

    #[test]
    fn test_is_cyclic_triangle() {
        let mut buffer = HashMap::<String, TransformStamped>::new();
        buffer.insert("dummy_1".to_string(), dummy_1_frame());
        buffer.insert("dummy_2".to_string(), dummy_2_frame());
        buffer.insert("dummy_3".to_string(), dummy_3_frame());

        //          w
        //          |
        //          d1
        //         /
        //       d2 -- d3

        let res = is_cyclic("world", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_2", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_3", &buffer);
        assert_eq!(res, false);

        buffer.insert(
            "dummy_1".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_3".to_string(),
                child_frame_id: "dummy_1".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        //          w
        //          
        //          d1
        //         /  \
        //       d2 -- d3

        let res = is_cyclic("world", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, true);
        let res = is_cyclic("dummy_2", &buffer);
        assert_eq!(res, true);
        let res = is_cyclic("dummy_3", &buffer);
        assert_eq!(res, true);
    }


    #[test]
    fn test_is_cyclic_all() {
        let mut buffer = HashMap::<String, TransformStamped>::new();
        buffer.insert("dummy_1".to_string(), dummy_1_frame());
        buffer.insert("dummy_2".to_string(), dummy_2_frame());
        buffer.insert("dummy_3".to_string(), dummy_3_frame());

        //          w
        //          |
        //          d1
        //         /
        //       d2 -- d3

        let res = is_cyclic_all(&buffer);
        assert_eq!(res, false);

        buffer.insert(
            "dummy_5".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_4".to_string(),
                child_frame_id: "dummy_5".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        buffer.insert(
            "dummy_6".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_5".to_string(),
                child_frame_id: "dummy_6".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        //          w           d4
        //          |           |
        //          d1          d5
        //         /            |
        //       d2 -- d3       d6

        let res = is_cyclic_all(&buffer);
        assert_eq!(res, false);

        buffer.insert(
            "dummy_4".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_6".to_string(),
                child_frame_id: "dummy_4".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        //          w           d4
        //          |          /  \
        //          d1       d5 -- d6
        //         /            
        //       d2 -- d3       

        let res = is_cyclic_all(&buffer);
        assert_eq!(res, true);

        buffer.insert(
            "dummy_4".to_string(),
            TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "world".to_string(),
                child_frame_id: "dummy_4".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            },
        );

        //          w --------- d4
        //          |          /  
        //          d1       d5 -- d6
        //         /            
        //       d2 -- d3   

        let res = is_cyclic_all(&buffer);
        assert_eq!(res, false);
    }

    #[test]
    fn test_would_produce_cycle() {
        let mut buffer = HashMap::<String, TransformStamped>::new();
        buffer.insert("dummy_1".to_string(), dummy_1_frame());
        buffer.insert("dummy_2".to_string(), dummy_2_frame());
        buffer.insert("dummy_3".to_string(), dummy_3_frame());

        //          w
        //          |
        //          d1
        //         /
        //       d2 -- d3

        let res = is_cyclic("world", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_1", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_2", &buffer);
        assert_eq!(res, false);
        let res = is_cyclic("dummy_3", &buffer);
        assert_eq!(res, false);

        assert_eq!(check_would_produce_cycle(
            &TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_4".to_string(),
                child_frame_id: "dummy_1".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            }, 
            &buffer), false
        );

        assert_eq!(check_would_produce_cycle(
            &TransformStamped {
                time_stamp: Instant::now(),
                parent_frame_id: "dummy_3".to_string(),
                child_frame_id: "dummy_1".to_string(),
                transform: Isometry3::default(),
                json_metadata: String::default(),
            }, 
            &buffer), true
        );
    }


}
