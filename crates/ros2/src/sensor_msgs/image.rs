/// This message contains an uncompressed image
/// (0, 0) is at top-left corner of image
use serde::{Deserialize, Serialize};

use crate::std_msgs::header::Header;
use image::RgbImage;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Image {
    /// Header timestamp should be acquisition time of image
    /// Header frame_id should be optical frame of camera
    /// If the frame_id here and the frame_id of the CameraInfo
    /// message associated with the image conflict
    /// the behavior is undefined
    pub header: Header,

    /// origin of frame should be optical center of cameara
    /// +x should point to the right in the image
    /// +y should point down in the image
    /// +z should point into to plane of the image
    ///
    /// image height, that is, number of rows
    pub height: u32,
    /// image width, that is, number of columns
    pub width: u32,

    /// The legal values for encoding are in file src/image_encodings.cpp
    /// If you want to standardize a new string format, join
    /// ros-users@lists.ros.org and send an email proposing a new encoding.
    /// Encoding of pixels -- channel meaning, ordering, size
    /// taken from the list of strings in include/sensor_msgs/image_encodings.hpp
    pub encoding: String,

    /// is this data bigendian?
    pub is_bigendian: u8,
    /// Full row length in bytes
    pub step: u32,
    /// actual matrix data, size is (step * rows)
    pub data: Vec<u8>,
}

impl From<Image> for RgbImage {
    fn from(ros2_image: Image) -> Self {
        let data = match ros2_image.encoding.as_str() {
            "rgb8" => ros2_image.data,
            "mono16" => ros2_image
                .data
                .chunks(2)
                .map(|distance_bytes| {
                    let value = u16::from_be_bytes(
                        distance_bytes
                            .try_into()
                            .expect("failed to construct distance"),
                    );
                    let intensity = ((value as f32 / 10000.0).clamp(0.0, 1.0) * 255.0) as u8;
                    [intensity, intensity, intensity]
                })
                .flatten()
                .collect(),
            _ => unimplemented!("image encoding not supported"),
        };

        RgbImage::from_vec(ros2_image.width, ros2_image.height, data)
            .expect("buffer is not big enough")
    }
}
