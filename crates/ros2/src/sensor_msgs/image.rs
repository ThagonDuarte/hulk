/// This message contains an uncompressed image
/// (0, 0) is at top-left corner of image
use serde::{Deserialize, Serialize};

use crate::std_msgs::header::Header;

#[derive(Debug, Serialize, Deserialize)]
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
    #[serde(
        serialize_with = "serde_image_encoding::serialize",
        deserialize_with = "serde_image_encoding::deserialize"
    )]
    pub encoding: ImageEncoding,

    /// is this data bigendian?
    pub is_bigendian: u8,
    /// Full row length in bytes
    pub step: u32,
    /// actual matrix data, size is (step * rows)
    pub data: Vec<u8>,
}

#[derive(Debug)]
pub enum ImageEncoding {
    // Standard encodings
    Rgb8,
    Rgba8,
    Rgb16,
    Rgba16,
    Bgr8,
    Bgra8,
    Bgr16,
    Bgra16,
    Mono8,
    Mono16,

    // OpenCV CvMat types
    CvMat8UC1,
    CvMat8UC2,
    CvMat8UC3,
    CvMat8UC4,
    CvMat8SC1,
    CvMat8SC2,
    CvMat8SC3,
    CvMat8SC4,
    CvMat16UC1,
    CvMat16UC2,
    CvMat16UC3,
    CvMat16UC4,
    CvMat16SC1,
    CvMat16SC2,
    CvMat16SC3,
    CvMat16SC4,
    CvMat32SC1,
    CvMat32SC2,
    CvMat32SC3,
    CvMat32SC4,
    CvMat32FC1,
    CvMat32FC2,
    CvMat32FC3,
    CvMat32FC4,
    CvMat64FC1,
    CvMat64FC2,
    CvMat64FC3,
    CvMat64FC4,

    // Bayer encodings
    BayerRggb8,
    BayerBggr8,
    BayerGbrg8,
    BayerGrbg8,
    BayerRggb16,
    BayerBggr16,
    BayerGbrg16,
    BayerGrbg16,

    // Misc encodings
    Yuv422,
}

mod serde_image_encoding {
    use serde::{de::Error, Deserialize, Deserializer, Serializer};

    use crate::sensor_msgs::image::ImageEncoding;

    pub fn deserialize<'de, D>(deserializer: D) -> Result<ImageEncoding, D::Error>
    where
        D: Deserializer<'de>,
    {
        let s = String::deserialize(deserializer)?;

        match s.as_str() {
            "rgb8" => Ok(ImageEncoding::Rgb8),
            "rgba8" => Ok(ImageEncoding::Rgba8),
            "rgb16" => Ok(ImageEncoding::Rgb16),
            "rgba16" => Ok(ImageEncoding::Rgba16),
            "bgr8" => Ok(ImageEncoding::Bgr8),
            "bgra8" => Ok(ImageEncoding::Bgra8),
            "bgr16" => Ok(ImageEncoding::Bgr16),
            "bgra16" => Ok(ImageEncoding::Bgra16),
            "mono8" => Ok(ImageEncoding::Mono8),
            "mono16" => Ok(ImageEncoding::Mono16),
            "8UC1" => Ok(ImageEncoding::CvMat8UC1),
            "8UC2" => Ok(ImageEncoding::CvMat8UC2),
            "8UC3" => Ok(ImageEncoding::CvMat8UC3),
            "8UC4" => Ok(ImageEncoding::CvMat8UC4),
            "8SC1" => Ok(ImageEncoding::CvMat8SC1),
            "8SC2" => Ok(ImageEncoding::CvMat8SC2),
            "8SC3" => Ok(ImageEncoding::CvMat8SC3),
            "8SC4" => Ok(ImageEncoding::CvMat8SC4),
            "16UC1" => Ok(ImageEncoding::CvMat16UC1),
            "16UC2" => Ok(ImageEncoding::CvMat16UC2),
            "16UC3" => Ok(ImageEncoding::CvMat16UC3),
            "16UC4" => Ok(ImageEncoding::CvMat16UC4),
            "16SC1" => Ok(ImageEncoding::CvMat16SC1),
            "16SC2" => Ok(ImageEncoding::CvMat16SC2),
            "16SC3" => Ok(ImageEncoding::CvMat16SC3),
            "16SC4" => Ok(ImageEncoding::CvMat16SC4),
            "32SC1" => Ok(ImageEncoding::CvMat32SC1),
            "32SC2" => Ok(ImageEncoding::CvMat32SC2),
            "32SC3" => Ok(ImageEncoding::CvMat32SC3),
            "32SC4" => Ok(ImageEncoding::CvMat32SC4),
            "32FC1" => Ok(ImageEncoding::CvMat32FC1),
            "32FC2" => Ok(ImageEncoding::CvMat32FC2),
            "32FC3" => Ok(ImageEncoding::CvMat32FC3),
            "32FC4" => Ok(ImageEncoding::CvMat32FC4),
            "64FC1" => Ok(ImageEncoding::CvMat64FC1),
            "64FC2" => Ok(ImageEncoding::CvMat64FC2),
            "64FC3" => Ok(ImageEncoding::CvMat64FC3),
            "64FC4" => Ok(ImageEncoding::CvMat64FC4),
            "bayer_rggb8" => Ok(ImageEncoding::BayerRggb8),
            "bayer_bggr8" => Ok(ImageEncoding::BayerBggr8),
            "bayer_gbrg8" => Ok(ImageEncoding::BayerGbrg8),
            "bayer_grbg8" => Ok(ImageEncoding::BayerGrbg8),
            "bayer_rggb16" => Ok(ImageEncoding::BayerRggb16),
            "bayer_bggr16" => Ok(ImageEncoding::BayerBggr16),
            "bayer_gbrg16" => Ok(ImageEncoding::BayerGbrg16),
            "bayer_grbg16" => Ok(ImageEncoding::BayerGrbg16),
            "yuv422" => Ok(ImageEncoding::Yuv422),
            _ => Err(D::Error::custom(format!("Unknown ImageEncoding: {}", s))),
        }
    }

    pub fn serialize<S>(image_encoding: &ImageEncoding, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_str(match image_encoding {
            ImageEncoding::Rgb8 => "rgb8",
            ImageEncoding::Bgr8 => "bgr8",
            ImageEncoding::Rgba8 => "rgba8",
            ImageEncoding::Rgb16 => "rgb16",
            ImageEncoding::Rgba16 => "rgba16",
            ImageEncoding::Bgra8 => "bgra8",
            ImageEncoding::Bgr16 => "bgr16",
            ImageEncoding::Bgra16 => "bgra16",
            ImageEncoding::Mono8 => "mono8",
            ImageEncoding::Mono16 => "mono16",
            ImageEncoding::CvMat8UC1 => "8UC1",
            ImageEncoding::CvMat8UC2 => "8UC2",
            ImageEncoding::CvMat8UC3 => "8UC3",
            ImageEncoding::CvMat8UC4 => "8UC4",
            ImageEncoding::CvMat8SC1 => "8SC1",
            ImageEncoding::CvMat8SC2 => "8SC2",
            ImageEncoding::CvMat8SC3 => "8SC3",
            ImageEncoding::CvMat8SC4 => "8SC4",
            ImageEncoding::CvMat16UC1 => "16UC1",
            ImageEncoding::CvMat16UC2 => "16UC2",
            ImageEncoding::CvMat16UC3 => "16UC3",
            ImageEncoding::CvMat16UC4 => "16UC4",
            ImageEncoding::CvMat16SC1 => "16SC1",
            ImageEncoding::CvMat16SC2 => "16SC2",
            ImageEncoding::CvMat16SC3 => "16SC3",
            ImageEncoding::CvMat16SC4 => "16SC4",
            ImageEncoding::CvMat32SC1 => "32SC1",
            ImageEncoding::CvMat32SC2 => "32SC2",
            ImageEncoding::CvMat32SC3 => "32SC3",
            ImageEncoding::CvMat32SC4 => "32SC4",
            ImageEncoding::CvMat32FC1 => "32FC1",
            ImageEncoding::CvMat32FC2 => "32FC2",
            ImageEncoding::CvMat32FC3 => "32FC3",
            ImageEncoding::CvMat32FC4 => "32FC4",
            ImageEncoding::CvMat64FC1 => "64FC1",
            ImageEncoding::CvMat64FC2 => "64FC2",
            ImageEncoding::CvMat64FC3 => "64FC3",
            ImageEncoding::CvMat64FC4 => "64FC4",
            ImageEncoding::BayerRggb8 => "bayer_rggb8",
            ImageEncoding::BayerBggr8 => "bayer_bggr8",
            ImageEncoding::BayerGbrg8 => "bayer_gbrg8",
            ImageEncoding::BayerGrbg8 => "bayer_grbg8",
            ImageEncoding::BayerRggb16 => "bayer_rggb16",
            ImageEncoding::BayerBggr16 => "bayer_bggr16",
            ImageEncoding::BayerGbrg16 => "bayer_gbrg16",
            ImageEncoding::BayerGrbg16 => "bayer_grbg16",
            ImageEncoding::Yuv422 => "yuv422",
        })
    }
}
