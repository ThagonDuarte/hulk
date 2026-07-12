use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap},
    fs::{self, File},
    io::{Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
    sync::Arc,
};

use color_eyre::{
    Result,
    eyre::{Context, ContextCompat, bail, eyre},
};
use image::{RgbImage, codecs::jpeg::JpegEncoder};
use mcap::{
    Channel, McapError, Schema, parse_record,
    records::{Record, op},
    sans_io::{LinearReadEvent, LinearReader, LinearReaderOptions},
};
use ros_z::{Message, SerdeCdrCodec, message::WireDecoder};
use ros2::sensor_msgs::image::Image;
use serde::{Deserialize, Serialize};
use tempfile::NamedTempFile;
use tokio::sync::mpsc;
use types::{
    object_detection::{Object, RobocupObjectLabel},
    stereo_image_pair::StereoImagePair,
    time_wrapper::TimeWrapper,
};

use crate::cache::{
    CACHE_VERSION, Prediction, RecordedBaseline, RecordingFingerprint, read_bincode,
    recorded_baseline_path, recording_cache_directory, save_recorded_baseline,
    write_bincode_atomic,
};

const RECORDING_INDEX_FILE: &str = "index.bin";
const PROXY_FILE: &str = "frames.jpg";
const JPEG_QUALITY: u8 = 85;
const LEFT_IMAGE_TOPIC: &str = "inputs/left_image";
const STEREO_IMAGE_TOPIC: &str = "inputs/stereo_image_pair";
const DETECTED_OBJECTS_TOPIC: &str = "detected_objects";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ImageTopic {
    LeftImage,
    StereoImagePair,
}

impl ImageTopic {
    fn name(self) -> &'static str {
        match self {
            Self::LeftImage => LEFT_IMAGE_TOPIC,
            Self::StereoImagePair => STEREO_IMAGE_TOPIC,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FrameIndexEntry {
    pub frame_index: usize,
    pub timestamp_nanos: i64,
    pub byte_offset: u64,
    pub byte_length: u64,
    pub width: u32,
    pub height: u32,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordingCacheIndex {
    pub cache_version: u32,
    pub fingerprint: RecordingFingerprint,
    pub image_topic: ImageTopic,
    pub frames: Vec<FrameIndexEntry>,
    pub tail_warning: Option<String>,
}

#[derive(Debug)]
pub struct OriginalFrame {
    pub frame_index: usize,
    pub timestamp_nanos: i64,
    pub image: Image,
}

pub struct Recording {
    cache_directory: PathBuf,
    recording_cache_directory: PathBuf,
    index: Arc<RecordingCacheIndex>,
}

impl Recording {
    pub fn open(
        recording_path: impl AsRef<Path>,
        cache_directory: impl AsRef<Path>,
    ) -> Result<Self> {
        let fingerprint = RecordingFingerprint::for_path(recording_path)?;
        let cache_directory = cache_directory.as_ref().to_path_buf();
        fs::create_dir_all(&cache_directory).wrap_err_with(|| {
            format!(
                "failed to create cache directory {}",
                cache_directory.display()
            )
        })?;
        let recording_cache_directory = recording_cache_directory(&cache_directory, &fingerprint)?;
        fs::create_dir_all(&recording_cache_directory).wrap_err_with(|| {
            format!(
                "failed to create recording cache {}",
                recording_cache_directory.display()
            )
        })?;

        let index_path = recording_cache_directory.join(RECORDING_INDEX_FILE);
        let proxy_path = recording_cache_directory.join(PROXY_FILE);
        let baseline_path = recorded_baseline_path(&cache_directory, &fingerprint)?;
        let index = if index_path.exists() && proxy_path.exists() && baseline_path.exists() {
            match read_bincode::<RecordingCacheIndex>(&index_path).and_then(|index| {
                validate_cached_index(&index, &fingerprint, &proxy_path)?;
                let baseline: RecordedBaseline = read_bincode(&baseline_path)?;
                validate_cached_baseline(&baseline, &fingerprint, &index.frames)?;
                Ok(index)
            }) {
                Ok(index) => index,
                Err(error) => {
                    tracing::warn!(
                        ?error,
                        "rebuilding invalid detection replay recording cache"
                    );
                    build_cache(&fingerprint, &cache_directory, &recording_cache_directory)?
                }
            }
        } else {
            build_cache(&fingerprint, &cache_directory, &recording_cache_directory)?
        };

        Ok(Self {
            cache_directory,
            recording_cache_directory,
            index: Arc::new(index),
        })
    }

    pub fn fingerprint(&self) -> &RecordingFingerprint {
        &self.index.fingerprint
    }

    pub fn index(&self) -> &RecordingCacheIndex {
        &self.index
    }

    pub fn frames(&self) -> &[FrameIndexEntry] {
        &self.index.frames
    }

    pub fn frame_count(&self) -> usize {
        self.index.frames.len()
    }

    pub fn cache_directory(&self) -> &Path {
        &self.cache_directory
    }

    pub fn load_proxy_frame(&self, frame_index: usize) -> Result<RgbImage> {
        let entry = self
            .index
            .frames
            .get(frame_index)
            .wrap_err_with(|| format!("proxy frame {frame_index} is out of range"))?;
        let length = usize::try_from(entry.byte_length)
            .wrap_err("proxy JPEG is too large for this platform")?;
        let mut bytes = vec![0; length];
        let proxy_path = self.recording_cache_directory.join(PROXY_FILE);
        let mut file = File::open(&proxy_path)
            .wrap_err_with(|| format!("failed to open {}", proxy_path.display()))?;
        file.seek(SeekFrom::Start(entry.byte_offset))
            .wrap_err_with(|| format!("failed to seek {}", proxy_path.display()))?;
        file.read_exact(&mut bytes)
            .wrap_err_with(|| format!("failed to read proxy frame {frame_index}"))?;
        let image = image::load_from_memory_with_format(&bytes, image::ImageFormat::Jpeg)
            .wrap_err_with(|| format!("failed to decode proxy frame {frame_index}"))?
            .to_rgb8();
        if image.width() != entry.width || image.height() != entry.height {
            bail!("proxy frame {frame_index} dimensions do not match its index");
        }
        Ok(image)
    }

    pub fn original_images(&self, skip: usize, take: Option<usize>) -> Result<OriginalImageStream> {
        if skip > self.frame_count() {
            bail!(
                "cannot skip {skip} frames in a {} frame recording",
                self.frame_count()
            );
        }
        let available = self.frame_count() - skip;
        let take = take.unwrap_or(available);
        if take > available {
            bail!("requested {take} frames after skip, but only {available} remain");
        }

        let path = self.index.fingerprint.canonical_path.clone();
        let image_topic = self.index.image_topic;
        let index = Arc::clone(&self.index);
        let (sender, receiver) = mpsc::channel(1);
        let task = tokio::task::spawn_blocking(move || {
            if let Err(error) =
                scan_original_images(&path, image_topic, &index.frames, skip, take, &sender)
            {
                let _ = sender.blocking_send(Err(error));
            }
        });
        Ok(OriginalImageStream {
            receiver,
            task,
            expected: take,
            received: 0,
            finished: false,
        })
    }

    pub fn load_runs(&self) -> Result<Vec<crate::cache::LoadedPredictionRun>> {
        crate::cache::load_all_runs(
            &self.cache_directory,
            self.fingerprint(),
            self.frame_count(),
        )
    }

    pub fn load_run_ui_metadata(&self) -> Result<BTreeMap<String, crate::cache::RunUiMetadata>> {
        crate::cache::load_run_ui_metadata(&self.cache_directory, self.fingerprint())
    }

    pub fn save_run_ui_metadata(
        &self,
        metadata: &BTreeMap<String, crate::cache::RunUiMetadata>,
    ) -> Result<()> {
        crate::cache::save_run_ui_metadata(&self.cache_directory, self.fingerprint(), metadata)
    }

    pub fn delete_model_run(&self, run_key: &str) -> Result<()> {
        crate::cache::delete_model_run(&self.cache_directory, self.fingerprint(), run_key)
    }

    pub fn rename_run(
        &self,
        run_key: &str,
        label: &str,
    ) -> Result<BTreeMap<String, crate::cache::RunUiMetadata>> {
        crate::cache::rename_run(&self.cache_directory, self.fingerprint(), run_key, label)
    }

    pub fn set_run_hidden(
        &self,
        run_key: &str,
        hidden: bool,
    ) -> Result<BTreeMap<String, crate::cache::RunUiMetadata>> {
        crate::cache::set_run_hidden(&self.cache_directory, self.fingerprint(), run_key, hidden)
    }

    pub fn remove_run_ui_metadata(
        &self,
        run_key: &str,
    ) -> Result<BTreeMap<String, crate::cache::RunUiMetadata>> {
        crate::cache::remove_run_ui_metadata(&self.cache_directory, self.fingerprint(), run_key)
    }
}

pub struct OriginalImageStream {
    receiver: mpsc::Receiver<Result<OriginalFrame>>,
    task: tokio::task::JoinHandle<()>,
    expected: usize,
    received: usize,
    finished: bool,
}

impl OriginalImageStream {
    pub async fn next(&mut self) -> Option<Result<OriginalFrame>> {
        if self.finished {
            return None;
        }
        match self.receiver.recv().await {
            Some(Ok(frame)) => {
                self.received += 1;
                Some(Ok(frame))
            }
            Some(Err(error)) => {
                self.finished = true;
                Some(Err(error))
            }
            None if self.received == self.expected => {
                self.finished = true;
                None
            }
            None => {
                self.finished = true;
                Some(Err(eyre!(
                    "original image scanner stopped after {} of {} frames",
                    self.received,
                    self.expected
                )))
            }
        }
    }
}

impl Drop for OriginalImageStream {
    fn drop(&mut self) {
        self.task.abort();
    }
}

fn build_cache(
    fingerprint: &RecordingFingerprint,
    cache_directory: &Path,
    recording_cache_directory: &Path,
) -> Result<RecordingCacheIndex> {
    let mut proxy = NamedTempFile::new_in(recording_cache_directory).wrap_err_with(|| {
        format!(
            "failed to create proxy temporary file in {}",
            recording_cache_directory.display()
        )
    })?;
    let mut frames = Vec::new();
    let mut active_image_topic = None;
    let mut recorded_objects = BTreeMap::new();
    let mut proxy_offset = 0_u64;
    let mut tail_warning = None;

    let scan_end = scan_messages(&fingerprint.canonical_path, |message| {
        let candidate_topic = match message.channel.topic.as_str() {
            LEFT_IMAGE_TOPIC => Some(ImageTopic::LeftImage),
            STEREO_IMAGE_TOPIC => Some(ImageTopic::StereoImagePair),
            _ => None,
        };
        if let Some(candidate_topic) = candidate_topic {
            let selected_topic = *active_image_topic.get_or_insert(candidate_topic);
            if candidate_topic != selected_topic {
                return Ok(ScanControl::Continue);
            }
            let image = decode_image_message(message, selected_topic)?;
            let timestamp_nanos = image_timestamp_nanos(&image)?;
            let rgb = RgbImage::try_from(image.clone())
                .wrap_err_with(|| format!("failed to convert frame {} to RGB", frames.len()))?;
            let mut jpeg = Vec::new();
            JpegEncoder::new_with_quality(&mut jpeg, JPEG_QUALITY)
                .encode_image(&rgb)
                .wrap_err_with(|| format!("failed to encode proxy frame {}", frames.len()))?;
            proxy
                .write_all(&jpeg)
                .wrap_err("failed to append proxy JPEG")?;
            let byte_length = u64::try_from(jpeg.len()).wrap_err("proxy JPEG length overflow")?;
            frames.push(FrameIndexEntry {
                frame_index: frames.len(),
                timestamp_nanos,
                byte_offset: proxy_offset,
                byte_length,
                width: image.width,
                height: image.height,
            });
            proxy_offset = proxy_offset
                .checked_add(byte_length)
                .wrap_err("concatenated proxy length overflow")?;
        } else if message.channel.topic == DETECTED_OBJECTS_TOPIC {
            validate_channel::<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>(message)?;
            let detected =
                SerdeCdrCodec::<TimeWrapper<Vec<Object<RobocupObjectLabel>>>>::deserialize(
                    message.data.as_ref(),
                )
                .wrap_err("failed to decode recorded detected_objects")?;
            recorded_objects.insert(detected.time.as_nanos(), detected.inner);
        }
        Ok(ScanControl::Continue)
    })?;
    if let ScanEnd::DamagedTail(error) = scan_end {
        if frames.is_empty() {
            return Err(error).wrap_err("MCAP was damaged before its first complete image");
        }
        let warning = format!(
            "ignored damaged MCAP tail after {} complete frames: {error}",
            frames.len()
        );
        tracing::warn!(frames = frames.len(), ?error, "ignored damaged MCAP tail");
        tail_warning = Some(warning);
    }

    let image_topic = active_image_topic.wrap_err("recording contains no supported image topic")?;
    if frames.is_empty() {
        bail!("recording contains no complete image frames");
    }
    proxy.flush().wrap_err("failed to flush proxy file")?;
    proxy
        .as_file_mut()
        .sync_all()
        .wrap_err("failed to sync proxy file")?;
    proxy
        .persist(recording_cache_directory.join(PROXY_FILE))
        .map_err(|error| error.error)
        .wrap_err("failed to atomically replace proxy file")?;

    let mut baseline_predictions = vec![None; frames.len()];
    for frame in &frames {
        if let Some(objects) = recorded_objects.remove(&frame.timestamp_nanos) {
            baseline_predictions[frame.frame_index] = Some(Prediction {
                frame_index: frame.frame_index,
                timestamp_nanos: frame.timestamp_nanos,
                objects,
                poses: None,
                inference_duration_nanos: None,
                postprocessing_duration_nanos: None,
                non_maximum_suppression_duration_nanos: None,
            });
        }
    }
    save_recorded_baseline(
        cache_directory,
        &RecordedBaseline {
            cache_version: CACHE_VERSION,
            recording_fingerprint: fingerprint.clone(),
            total_frame_count: frames.len(),
            predictions: baseline_predictions,
        },
    )?;

    let index = RecordingCacheIndex {
        cache_version: CACHE_VERSION,
        fingerprint: fingerprint.clone(),
        image_topic,
        frames,
        tail_warning,
    };
    write_bincode_atomic(
        &recording_cache_directory.join(RECORDING_INDEX_FILE),
        &index,
    )?;
    Ok(index)
}

fn scan_original_images(
    path: &Path,
    image_topic: ImageTopic,
    expected_frames: &[FrameIndexEntry],
    skip: usize,
    take: usize,
    sender: &mpsc::Sender<Result<OriginalFrame>>,
) -> Result<()> {
    if take == 0 {
        return Ok(());
    }
    let mut selected_frame = 0_usize;
    let mut emitted = 0_usize;

    scan_messages(path, |message| {
        if message.channel.topic != image_topic.name() {
            return Ok(ScanControl::Continue);
        }
        if sender.is_closed() {
            return Ok(ScanControl::Stop);
        }
        validate_image_channel(message, image_topic)?;
        let frame_index = selected_frame;
        selected_frame += 1;
        if frame_index < skip {
            return Ok(ScanControl::Continue);
        }
        if emitted == take {
            return Ok(ScanControl::Stop);
        }
        let image = decode_image_data(message, image_topic)?;
        let timestamp_nanos = image_timestamp_nanos(&image)?;
        let expected = expected_frames
            .get(frame_index)
            .wrap_err("original recording has more frames than its cache index")?;
        if timestamp_nanos != expected.timestamp_nanos
            || image.width != expected.width
            || image.height != expected.height
        {
            bail!("original frame {frame_index} does not match its cache index");
        }
        if sender
            .blocking_send(Ok(OriginalFrame {
                frame_index,
                timestamp_nanos,
                image,
            }))
            .is_err()
        {
            return Ok(ScanControl::Stop);
        }
        emitted += 1;
        if emitted == take {
            return Ok(ScanControl::Stop);
        }
        Ok(ScanControl::Continue)
    })?;
    if emitted != take {
        bail!("recording ended after {emitted} of {take} requested original frames");
    }
    Ok(())
}

#[derive(Clone, Copy)]
enum ScanControl {
    Continue,
    Stop,
}

enum ScanEnd {
    Complete,
    DamagedTail(McapError),
}

fn scan_messages(
    path: &Path,
    mut handle_message: impl FnMut(&mcap::Message<'_>) -> Result<ScanControl>,
) -> Result<ScanEnd> {
    let mut file = File::open(path)
        .wrap_err_with(|| format!("failed to open recording {}", path.display()))?;
    let file_length = file
        .metadata()
        .wrap_err_with(|| format!("failed to read metadata for {}", path.display()))?
        .len();
    let length_limit = usize::try_from(file_length).unwrap_or(usize::MAX);
    let options = LinearReaderOptions::default()
        .with_record_length_limit(length_limit)
        .with_validate_chunk_crcs(true);
    let mut reader = LinearReader::new_with_options(options);
    let mut schemas: HashMap<u16, Arc<Schema<'static>>> = HashMap::new();
    let mut channels: HashMap<u16, Arc<Channel<'static>>> = HashMap::new();

    while let Some(event) = reader.next_event() {
        match event {
            Ok(LinearReadEvent::ReadRequest(length)) => {
                let read = file
                    .read(reader.insert(length))
                    .wrap_err_with(|| format!("failed to read {}", path.display()))?;
                reader.notify_read(read);
            }
            Ok(LinearReadEvent::Record { data, opcode }) => match parse_record(opcode, data)? {
                Record::Schema { header, data } => {
                    let schema = Arc::new(Schema {
                        id: header.id,
                        name: header.name,
                        encoding: header.encoding,
                        data: Cow::Owned(data.into_owned()),
                    });
                    if let Some(existing) = schemas.insert(schema.id, Arc::clone(&schema))
                        && existing.as_ref() != schema.as_ref()
                    {
                        bail!(
                            "MCAP contains conflicting schema records for id {}",
                            schema.id
                        );
                    }
                }
                Record::Channel(channel) => {
                    let schema = if channel.schema_id == 0 {
                        None
                    } else {
                        Some(schemas.get(&channel.schema_id).cloned().wrap_err_with(|| {
                            format!(
                                "MCAP channel {} references unknown schema {}",
                                channel.topic, channel.schema_id
                            )
                        })?)
                    };
                    let channel = Arc::new(Channel {
                        id: channel.id,
                        topic: channel.topic,
                        schema,
                        message_encoding: channel.message_encoding,
                        metadata: channel.metadata,
                    });
                    if let Some(existing) = channels.insert(channel.id, Arc::clone(&channel))
                        && existing.as_ref() != channel.as_ref()
                    {
                        bail!(
                            "MCAP contains conflicting channel records for id {}",
                            channel.id
                        );
                    }
                }
                Record::Message { header, data } => {
                    let channel =
                        channels
                            .get(&header.channel_id)
                            .cloned()
                            .wrap_err_with(|| {
                                format!(
                                    "MCAP message {} references unknown channel {}",
                                    header.sequence, header.channel_id
                                )
                            })?;
                    let message = mcap::Message {
                        channel,
                        sequence: header.sequence,
                        log_time: header.log_time,
                        publish_time: header.publish_time,
                        data,
                    };
                    if matches!(handle_message(&message)?, ScanControl::Stop) {
                        return Ok(ScanEnd::Complete);
                    }
                }
                _ => {}
            },
            Err(error) => {
                let position = file
                    .stream_position()
                    .wrap_err_with(|| format!("failed to inspect {}", path.display()))?;
                let current_chunk_extends_to_eof =
                    current_chunk_record_extends_to_eof(&mut file, position, file_length)?;
                if is_recoverable_tail_error(
                    &error,
                    position,
                    file_length,
                    current_chunk_extends_to_eof,
                ) {
                    return Ok(ScanEnd::DamagedTail(error));
                }
                return Err(error).wrap_err_with(|| {
                    format!(
                        "failed while scanning MCAP messages at byte {position} of {file_length}"
                    )
                });
            }
        }
    }

    Ok(ScanEnd::Complete)
}

fn decode_image_message(message: &mcap::Message<'_>, topic: ImageTopic) -> Result<Image> {
    validate_image_channel(message, topic)?;
    decode_image_data(message, topic)
}

fn decode_image_data(message: &mcap::Message<'_>, topic: ImageTopic) -> Result<Image> {
    match topic {
        ImageTopic::LeftImage => SerdeCdrCodec::<Image>::deserialize(message.data.as_ref())
            .wrap_err("failed to decode inputs/left_image"),
        ImageTopic::StereoImagePair => {
            let pair = SerdeCdrCodec::<StereoImagePair>::deserialize(message.data.as_ref())
                .wrap_err("failed to decode inputs/stereo_image_pair")?;
            Ok(pair.left)
        }
    }
}

fn validate_image_channel(message: &mcap::Message<'_>, topic: ImageTopic) -> Result<()> {
    match topic {
        ImageTopic::LeftImage => validate_channel::<Image>(message),
        ImageTopic::StereoImagePair => validate_channel::<StereoImagePair>(message),
    }
}

fn validate_channel<T: Message>(message: &mcap::Message<'_>) -> Result<()> {
    if message.channel.message_encoding != "ros-z-cdr" {
        bail!(
            "topic {} uses unsupported message encoding {}",
            message.channel.topic,
            message.channel.message_encoding
        );
    }
    let expected_type = T::type_name();
    if let Some(recorded_type) = message.channel.metadata.get("ros_z.type_name")
        && recorded_type != &expected_type
    {
        bail!(
            "topic {} contains type {}, expected {}",
            message.channel.topic,
            recorded_type,
            expected_type
        );
    }
    if let Some(schema) = &message.channel.schema {
        if schema.encoding != "ros-z-schema-json" {
            bail!(
                "topic {} uses unsupported schema encoding {}",
                message.channel.topic,
                schema.encoding
            );
        }
        if schema.name != expected_type {
            bail!(
                "topic {} schema is {}, expected {}",
                message.channel.topic,
                schema.name,
                expected_type
            );
        }
    }
    Ok(())
}

fn image_timestamp_nanos(image: &Image) -> Result<i64> {
    if image.header.stamp.sec < 0 {
        bail!("negative image timestamps are not supported by ros-z Time");
    }
    if image.header.stamp.nanosec >= 1_000_000_000 {
        bail!("image timestamp has invalid nanosecond component");
    }
    i64::from(image.header.stamp.sec)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(i64::from(image.header.stamp.nanosec)))
        .wrap_err("image timestamp overflows i64 nanoseconds")
}

fn current_chunk_record_extends_to_eof(
    file: &mut File,
    read_position: u64,
    file_length: u64,
) -> Result<bool> {
    const SEARCH_LENGTH: u64 = 128;
    const COMPRESSION_LENGTH_OFFSET: usize = 9 + 8 + 8 + 8 + 4;
    const FIXED_CHUNK_HEADER_LENGTH: u64 = 9 + 8 + 8 + 8 + 4 + 4 + 8;

    let start = read_position.saturating_sub(SEARCH_LENGTH);
    let length = usize::try_from(read_position - start).wrap_err("tail search length overflow")?;
    let mut bytes = vec![0; length];
    file.seek(SeekFrom::Start(start))?;
    file.read_exact(&mut bytes)?;
    file.seek(SeekFrom::Start(read_position))?;

    for candidate in (0..bytes.len()).rev() {
        if bytes[candidate] != op::CHUNK
            || candidate + COMPRESSION_LENGTH_OFFSET + 4 > bytes.len()
            || candidate + 9 > bytes.len()
        {
            continue;
        }
        let record_length = u64::from_le_bytes(bytes[candidate + 1..candidate + 9].try_into()?);
        let compression_length = u32::from_le_bytes(
            bytes[candidate + COMPRESSION_LENGTH_OFFSET..candidate + COMPRESSION_LENGTH_OFFSET + 4]
                .try_into()?,
        ) as u64;
        let candidate_position = start + candidate as u64;
        let compressed_data_position = candidate_position
            .checked_add(FIXED_CHUNK_HEADER_LENGTH)
            .and_then(|position| position.checked_add(compression_length));
        if compressed_data_position != Some(read_position) {
            continue;
        }
        let record_end = candidate_position
            .checked_add(9)
            .and_then(|position| position.checked_add(record_length));
        return Ok(record_end.is_none_or(|end| end > file_length));
    }
    Ok(false)
}

fn is_recoverable_tail_error(
    error: &McapError,
    read_position: u64,
    file_length: u64,
    current_chunk_extends_to_eof: bool,
) -> bool {
    match error {
        McapError::UnexpectedEof | McapError::UnexpectedEoc | McapError::BadChunkLength { .. } => {
            read_position == file_length || current_chunk_extends_to_eof
        }
        McapError::RecordTooLarge { .. } | McapError::ChunkTooLarge(_) => {
            current_chunk_extends_to_eof
        }
        _ => false,
    }
}

fn validate_cached_index(
    index: &RecordingCacheIndex,
    fingerprint: &RecordingFingerprint,
    proxy_path: &Path,
) -> Result<()> {
    if index.cache_version != CACHE_VERSION || &index.fingerprint != fingerprint {
        bail!("recording cache fingerprint or version does not match");
    }
    if index.frames.is_empty() {
        bail!("recording cache contains no frames");
    }
    let mut expected_offset = 0_u64;
    for (frame_index, frame) in index.frames.iter().enumerate() {
        if frame.frame_index != frame_index
            || frame.byte_offset != expected_offset
            || frame.byte_length == 0
            || frame.width == 0
            || frame.height == 0
        {
            bail!("recording cache frame {frame_index} is invalid");
        }
        expected_offset = expected_offset
            .checked_add(frame.byte_length)
            .wrap_err("proxy offset overflow")?;
    }
    let proxy_length = proxy_path
        .metadata()
        .wrap_err_with(|| format!("failed to read metadata for {}", proxy_path.display()))?
        .len();
    if proxy_length != expected_offset {
        bail!("proxy length does not match recording cache index");
    }
    Ok(())
}

fn validate_cached_baseline(
    baseline: &RecordedBaseline,
    fingerprint: &RecordingFingerprint,
    frames: &[FrameIndexEntry],
) -> Result<()> {
    if baseline.cache_version != CACHE_VERSION
        || &baseline.recording_fingerprint != fingerprint
        || baseline.total_frame_count != frames.len()
        || baseline.predictions.len() != frames.len()
    {
        bail!("recorded baseline does not match its recording index");
    }
    for (frame_index, prediction) in baseline.predictions.iter().enumerate() {
        if prediction.as_ref().is_some_and(|prediction| {
            prediction.frame_index != frame_index
                || prediction.timestamp_nanos != frames[frame_index].timestamp_nanos
        }) {
            bail!("recorded baseline frame {frame_index} is invalid");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use mcap::{WriteOptions, records::MessageHeader};
    use ros_z::message::WireEncoder;

    use super::*;

    #[test]
    fn negative_timestamp_is_rejected() {
        let mut image = Image::default();
        image.header.stamp.sec = -2;
        image.header.stamp.nanosec = 300_000_000;
        assert!(image_timestamp_nanos(&image).is_err());
    }

    #[test]
    fn indexes_and_loads_a_proxy_frame() {
        let temporary = tempfile::tempdir().unwrap();
        let recording_path = temporary.path().join("recording.mcap");
        let cache_path = temporary.path().join("cache");
        let file = File::create(&recording_path).unwrap();
        let mut writer = WriteOptions::new().create(file).unwrap();
        let schema = serde_json::to_vec(&Image::schema()).unwrap();
        let schema_id = writer
            .add_schema(&Image::type_name(), "ros-z-schema-json", &schema)
            .unwrap();
        let channel_id = writer
            .add_channel(
                schema_id,
                LEFT_IMAGE_TOPIC,
                "ros-z-cdr",
                &BTreeMap::from([("ros_z.type_name".to_string(), Image::type_name())]),
            )
            .unwrap();
        let mut image = Image {
            width: 32,
            height: 32,
            encoding: "nv12".to_string(),
            step: 32,
            data: vec![128; 32 * 32 * 3 / 2].into(),
            ..Default::default()
        };
        image.header.stamp.sec = 12;
        image.header.stamp.nanosec = 345;
        let payload = SerdeCdrCodec::<Image>::serialize(&image).unwrap();
        writer
            .write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence: 0,
                    log_time: 12_000_000_345,
                    publish_time: 12_000_000_345,
                },
                &payload,
            )
            .unwrap();
        writer.finish().unwrap();
        drop(writer);

        let recording = Recording::open(&recording_path, &cache_path).unwrap();
        assert_eq!(recording.frame_count(), 1);
        assert_eq!(recording.frames()[0].timestamp_nanos, 12_000_000_345);
        let proxy = recording.load_proxy_frame(0).unwrap();
        assert_eq!(proxy.dimensions(), (32, 32));
    }

    #[test]
    fn unexpected_end_of_chunk_is_a_recoverable_tail() {
        assert!(is_recoverable_tail_error(
            &McapError::UnexpectedEoc,
            1024,
            1024,
            false,
        ));
        assert!(!is_recoverable_tail_error(
            &McapError::UnexpectedEoc,
            512,
            1024,
            false,
        ));
    }
}
