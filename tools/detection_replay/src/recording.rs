use std::{
    borrow::Cow,
    collections::{BTreeMap, HashMap},
    fs::{self, File, OpenOptions},
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
    sans_io::{
        IndexedReadEvent, IndexedReader, IndexedReaderOptions, LinearReadEvent, LinearReader,
        LinearReaderOptions, SummaryReadEvent, SummaryReader, indexed_reader::ReadOrder,
    },
};
use ros_z::{
    Message, SerdeCdrCodec,
    message::{WireDecoder, WireEncoder},
};
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
    recorded_baseline_path, recording_cache_directory, save_recorded_baseline, validate_baseline,
    write_bincode_atomic,
};

const RECORDING_INDEX_FILE: &str = "index.bin";
const PROXY_FILE: &str = "frames.jpg";
const ORIGINAL_FRAMES_FILE: &str = "original-frames.cdr";
const RECORDING_LOCK_FILE: &str = ".recording.lock";
const JPEG_QUALITY: u8 = 85;
const MAX_MCAP_RECORD_BYTES: usize = 512 * 1024 * 1024;
const IMAGE_CACHE_OVERHEAD_BYTES: usize = 1024 * 1024;
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
    pub source_sequence: u32,
    pub source_log_time: u64,
    pub source_publish_time: u64,
    pub source_data_hash: [u8; 32],
    pub source_byte_offset: Option<u64>,
    pub source_byte_length: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum OriginalImageSource {
    IndexedMcap,
    CachedCdr,
    LinearMcap,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordingCacheIndex {
    pub cache_version: u32,
    pub fingerprint: RecordingFingerprint,
    pub image_topic: ImageTopic,
    pub original_image_source: OriginalImageSource,
    pub frames: Vec<FrameIndexEntry>,
    pub tail_warning: Option<String>,
}

#[derive(Deserialize)]
struct LegacyFrameIndexEntry {
    frame_index: usize,
    timestamp_nanos: i64,
    byte_offset: u64,
    byte_length: u64,
    width: u32,
    height: u32,
}

#[derive(Deserialize)]
struct LegacyRecordingCacheIndex {
    cache_version: u32,
    fingerprint: RecordingFingerprint,
    image_topic: ImageTopic,
    frames: Vec<LegacyFrameIndexEntry>,
    tail_warning: Option<String>,
}

#[derive(Deserialize)]
struct LegacyRecordedBaseline {
    cache_version: u32,
    recording_fingerprint: RecordingFingerprint,
    total_frame_count: usize,
    predictions: Vec<Option<Prediction>>,
}

#[derive(Deserialize)]
struct InterimFrameIndexEntry {
    frame_index: usize,
    timestamp_nanos: i64,
    byte_offset: u64,
    byte_length: u64,
    width: u32,
    height: u32,
    source_sequence: u32,
    source_log_time: u64,
    source_publish_time: u64,
    source_byte_offset: Option<u64>,
    source_byte_length: Option<u64>,
}

#[derive(Deserialize)]
struct InterimRecordingCacheIndex {
    cache_version: u32,
    fingerprint: RecordingFingerprint,
    image_topic: ImageTopic,
    original_image_source: OriginalImageSource,
    frames: Vec<InterimFrameIndexEntry>,
    tail_warning: Option<String>,
}

#[derive(Debug)]
pub struct OriginalFrame {
    pub frame_index: usize,
    pub timestamp_nanos: i64,
    pub image: Image,
}

pub struct ProxyFrameReader {
    file: File,
    frames: Arc<RecordingCacheIndex>,
    bytes: Vec<u8>,
}

pub struct Recording {
    _cache_lock: File,
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
        let original_frames_path = recording_cache_directory.join(ORIGINAL_FRAMES_FILE);
        let baseline_path = recorded_baseline_path(&cache_directory, &fingerprint)?;
        let legacy_baseline_path = recording_cache_directory.join("recorded-baseline.bin");
        let cache_lock_path = recording_cache_directory.join(RECORDING_LOCK_FILE);
        let cache_lock = OpenOptions::new()
            .create(true)
            .truncate(false)
            .read(true)
            .write(true)
            .open(&cache_lock_path)
            .wrap_err_with(|| format!("failed to open {}", cache_lock_path.display()))?;
        let load_current = || -> Result<RecordingCacheIndex> {
            if !index_path.exists() || !proxy_path.exists() || !baseline_path.exists() {
                bail!("recording cache is incomplete");
            }
            let index: RecordingCacheIndex = read_bincode(&index_path)?;
            validate_cached_index(&index, &fingerprint, &proxy_path, &original_frames_path)?;
            let baseline: RecordedBaseline = read_bincode(&baseline_path)?;
            validate_cached_baseline(
                baseline_path
                    .parent()
                    .wrap_err("baseline path has no parent")?,
                &baseline,
                &fingerprint,
                &index.frames,
            )?;
            Ok(index)
        };
        cache_lock
            .lock_shared()
            .wrap_err("failed to lock recording cache for reading")?;
        let index = match load_current() {
            Ok(index) => index,
            Err(initial_error) => {
                cache_lock
                    .unlock()
                    .wrap_err("failed to release recording cache read lock")?;
                cache_lock
                    .lock()
                    .wrap_err("failed to lock recording cache for rebuilding")?;
                let index = match load_current().or_else(|current_error| {
                    let migration_error = format!(
                        "initial cache validation failed: {initial_error:#}; exclusive validation failed: {current_error:#}"
                    );
                    if !legacy_baseline_path.exists() {
                        bail!("{migration_error}");
                    }
                    migrate_legacy_recording_cache(
                        &fingerprint,
                        &cache_directory,
                        &recording_cache_directory,
                        &index_path,
                        &proxy_path,
                        &legacy_baseline_path,
                    )
                    .wrap_err(migration_error)
                }) {
                    Ok(index) => index,
                    Err(error) => {
                        tracing::warn!(
                            ?error,
                            "rebuilding invalid detection replay recording cache"
                        );
                        build_cache(&fingerprint, &cache_directory, &recording_cache_directory)?
                    }
                };
                cache_lock
                    .unlock()
                    .wrap_err("failed to release recording cache rebuild lock")?;
                cache_lock
                    .lock_shared()
                    .wrap_err("failed to lock rebuilt recording cache for reading")?;
                index
            }
        };

        Ok(Self {
            _cache_lock: cache_lock,
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
        self.proxy_frame_reader()?.load(frame_index)
    }

    pub fn proxy_frame_reader(&self) -> Result<ProxyFrameReader> {
        let proxy_path = self.recording_cache_directory.join(PROXY_FILE);
        Ok(ProxyFrameReader {
            file: File::open(&proxy_path)
                .wrap_err_with(|| format!("failed to open {}", proxy_path.display()))?,
            frames: Arc::clone(&self.index),
            bytes: Vec::new(),
        })
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
        let original_frames_path = self.recording_cache_directory.join(ORIGINAL_FRAMES_FILE);
        let image_topic = self.index.image_topic;
        let original_image_source = self.index.original_image_source;
        let index = Arc::clone(&self.index);
        let (sender, receiver) = mpsc::channel(1);
        let task = tokio::task::spawn_blocking(move || {
            let result = match original_image_source {
                OriginalImageSource::IndexedMcap => scan_indexed_original_images(
                    &path,
                    image_topic,
                    &index.frames,
                    skip,
                    take,
                    &sender,
                ),
                OriginalImageSource::CachedCdr => scan_cached_original_images(
                    &original_frames_path,
                    &index.frames,
                    skip,
                    take,
                    &sender,
                ),
                OriginalImageSource::LinearMcap => scan_linear_original_images(
                    &path,
                    image_topic,
                    &index.frames,
                    skip,
                    take,
                    &sender,
                ),
            };
            if let Err(error) = result {
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

    pub fn load_runs_with_errors(
        &self,
    ) -> Result<(Vec<crate::cache::LoadedPredictionRun>, Vec<String>)> {
        crate::cache::load_all_runs_with_errors(
            &self.cache_directory,
            self.fingerprint(),
            self.frame_count(),
        )
    }

    pub fn load_run_ui_metadata(&self) -> Result<BTreeMap<String, crate::cache::RunUiMetadata>> {
        crate::cache::load_run_ui_metadata(&self.cache_directory, self.fingerprint())
    }

    pub fn load_bookmarks(&self) -> Result<crate::cache::BookmarkCollection> {
        crate::cache::load_bookmarks(&self.cache_directory, self.fingerprint())
    }

    pub fn bookmarks_exist(&self) -> Result<bool> {
        crate::cache::bookmarks_exist(&self.cache_directory, self.fingerprint())
    }

    pub fn save_bookmarks(&self, bookmarks: &crate::cache::BookmarkCollection) -> Result<()> {
        crate::cache::save_bookmarks(&self.cache_directory, self.fingerprint(), bookmarks)
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

impl ProxyFrameReader {
    pub fn load(&mut self, frame_index: usize) -> Result<RgbImage> {
        let entry = self
            .frames
            .frames
            .get(frame_index)
            .wrap_err_with(|| format!("proxy frame {frame_index} is out of range"))?;
        let length = usize::try_from(entry.byte_length)
            .wrap_err("proxy JPEG is too large for this platform")?;
        self.bytes.resize(length, 0);
        self.file
            .seek(SeekFrom::Start(entry.byte_offset))
            .wrap_err("failed to seek proxy frame cache")?;
        self.file
            .read_exact(&mut self.bytes)
            .wrap_err_with(|| format!("failed to read proxy frame {frame_index}"))?;
        let image = image::load_from_memory_with_format(&self.bytes, image::ImageFormat::Jpeg)
            .wrap_err_with(|| format!("failed to decode proxy frame {frame_index}"))?
            .to_rgb8();
        if image.width() != entry.width || image.height() != entry.height {
            bail!("proxy frame {frame_index} dimensions do not match its index");
        }
        Ok(image)
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

fn migrate_legacy_recording_cache(
    fingerprint: &RecordingFingerprint,
    cache_directory: &Path,
    recording_cache_directory: &Path,
    index_path: &Path,
    proxy_path: &Path,
    baseline_path: &Path,
) -> Result<RecordingCacheIndex> {
    if let Ok(interim) = read_bincode::<InterimRecordingCacheIndex>(index_path)
        && interim.cache_version == CACHE_VERSION
        && &interim.fingerprint == fingerprint
    {
        let original_image_source = match interim.original_image_source {
            OriginalImageSource::IndexedMcap => OriginalImageSource::LinearMcap,
            source => source,
        };
        let index = RecordingCacheIndex {
            cache_version: CACHE_VERSION,
            fingerprint: interim.fingerprint,
            image_topic: interim.image_topic,
            original_image_source,
            frames: interim
                .frames
                .into_iter()
                .map(|frame| FrameIndexEntry {
                    frame_index: frame.frame_index,
                    timestamp_nanos: frame.timestamp_nanos,
                    byte_offset: frame.byte_offset,
                    byte_length: frame.byte_length,
                    width: frame.width,
                    height: frame.height,
                    source_sequence: frame.source_sequence,
                    source_log_time: frame.source_log_time,
                    source_publish_time: frame.source_publish_time,
                    source_data_hash: [0; 32],
                    source_byte_offset: frame.source_byte_offset,
                    source_byte_length: frame.source_byte_length,
                })
                .collect(),
            tail_warning: interim.tail_warning,
        };
        validate_cached_index(
            &index,
            fingerprint,
            proxy_path,
            &recording_cache_directory.join(ORIGINAL_FRAMES_FILE),
        )?;
        write_bincode_atomic(index_path, &index)?;
        return Ok(index);
    }
    let legacy: LegacyRecordingCacheIndex = read_bincode(index_path)?;
    if legacy.cache_version != 1 || &legacy.fingerprint != fingerprint || legacy.frames.is_empty() {
        bail!("legacy recording index does not match the recording");
    }
    let baseline: LegacyRecordedBaseline = read_bincode(baseline_path)?;
    if baseline.cache_version != 1
        || baseline.recording_fingerprint != legacy.fingerprint
        || baseline.total_frame_count != legacy.frames.len()
        || baseline.predictions.len() != legacy.frames.len()
    {
        bail!("legacy recorded baseline does not match its index");
    }
    let index = RecordingCacheIndex {
        cache_version: CACHE_VERSION,
        fingerprint: legacy.fingerprint,
        image_topic: legacy.image_topic,
        original_image_source: OriginalImageSource::LinearMcap,
        frames: legacy
            .frames
            .into_iter()
            .map(|frame| FrameIndexEntry {
                frame_index: frame.frame_index,
                timestamp_nanos: frame.timestamp_nanos,
                byte_offset: frame.byte_offset,
                byte_length: frame.byte_length,
                width: frame.width,
                height: frame.height,
                source_sequence: 0,
                source_log_time: 0,
                source_publish_time: 0,
                source_data_hash: [0; 32],
                source_byte_offset: None,
                source_byte_length: None,
            })
            .collect(),
        tail_warning: legacy.tail_warning,
    };
    validate_cached_index(
        &index,
        fingerprint,
        proxy_path,
        &recording_cache_directory.join(ORIGINAL_FRAMES_FILE),
    )?;
    save_recorded_baseline(cache_directory, fingerprint, &baseline.predictions)?;
    write_bincode_atomic(index_path, &index)?;
    tracing::info!("migrated detection replay recording cache without rescanning the MCAP");
    Ok(index)
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
    let has_chunk_index = load_summary(&fingerprint.canonical_path)?
        .is_some_and(|summary| !summary.chunk_indexes.is_empty());
    let mut original_frames = (!has_chunk_index)
        .then(|| NamedTempFile::new_in(recording_cache_directory))
        .transpose()
        .wrap_err("failed to create original-frame cache")?;
    let mut frames = Vec::new();
    let mut active_image_topic = None;
    let mut recorded_objects = BTreeMap::new();
    let mut proxy_offset = 0_u64;
    let mut source_offset = 0_u64;
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
            let source_bytes = original_frames
                .as_ref()
                .map(|_| SerdeCdrCodec::<Image>::serialize(&image))
                .transpose()
                .wrap_err_with(|| format!("failed to encode original frame {}", frames.len()))?;
            if let (Some(file), Some(bytes)) = (&mut original_frames, source_bytes.as_ref()) {
                file.write_all(bytes)
                    .wrap_err("failed to append original image data")?;
            }
            let source_length = source_bytes
                .as_ref()
                .map(|bytes| u64::try_from(bytes.len()).wrap_err("original frame length overflow"))
                .transpose()?;
            frames.push(FrameIndexEntry {
                frame_index: frames.len(),
                timestamp_nanos,
                byte_offset: proxy_offset,
                byte_length,
                width: image.width,
                height: image.height,
                source_sequence: message.sequence,
                source_log_time: message.log_time,
                source_publish_time: message.publish_time,
                source_data_hash: *blake3::hash(message.data.as_ref()).as_bytes(),
                source_byte_offset: source_length.map(|_| source_offset),
                source_byte_length: source_length,
            });
            proxy_offset = proxy_offset
                .checked_add(byte_length)
                .wrap_err("concatenated proxy length overflow")?;
            if let Some(source_length) = source_length {
                source_offset = source_offset
                    .checked_add(source_length)
                    .wrap_err("original frame cache length overflow")?;
            }
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
    let indexed_images_verified = if has_chunk_index
        && frames
            .windows(2)
            .all(|window| window[0].source_log_time <= window[1].source_log_time)
    {
        match verify_indexed_images(&fingerprint.canonical_path, image_topic, &frames) {
            Ok(verified) => verified,
            Err(error) => {
                tracing::warn!(?error, "MCAP index cannot safely replay source images");
                false
            }
        }
    } else {
        false
    };
    let original_image_source = if indexed_images_verified {
        for frame in &mut frames {
            frame.source_byte_offset = None;
            frame.source_byte_length = None;
        }
        OriginalImageSource::IndexedMcap
    } else {
        OriginalImageSource::CachedCdr
    };
    if original_image_source == OriginalImageSource::CachedCdr {
        if let Some(original_frames) = original_frames.take() {
            persist_original_frame_cache(original_frames, recording_cache_directory)?;
        } else {
            build_original_frame_cache(
                &fingerprint.canonical_path,
                image_topic,
                &mut frames,
                recording_cache_directory,
            )?;
        }
    } else {
        drop(original_frames);
        let stale = recording_cache_directory.join(ORIGINAL_FRAMES_FILE);
        if stale.exists() {
            fs::remove_file(&stale)
                .wrap_err_with(|| format!("failed to remove {}", stale.display()))?;
        }
    }

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
    save_recorded_baseline(cache_directory, fingerprint, &baseline_predictions)?;

    let index = RecordingCacheIndex {
        cache_version: CACHE_VERSION,
        fingerprint: fingerprint.clone(),
        image_topic,
        original_image_source,
        frames,
        tail_warning,
    };
    write_bincode_atomic(
        &recording_cache_directory.join(RECORDING_INDEX_FILE),
        &index,
    )?;
    Ok(index)
}

fn persist_original_frame_cache(
    mut original_frames: NamedTempFile,
    recording_cache_directory: &Path,
) -> Result<()> {
    original_frames
        .flush()
        .wrap_err("failed to flush original-frame cache")?;
    original_frames
        .as_file_mut()
        .sync_all()
        .wrap_err("failed to sync original-frame cache")?;
    original_frames
        .persist(recording_cache_directory.join(ORIGINAL_FRAMES_FILE))
        .map_err(|error| error.error)
        .wrap_err("failed to atomically replace original-frame cache")?;
    Ok(())
}

fn build_original_frame_cache(
    path: &Path,
    image_topic: ImageTopic,
    expected_frames: &mut [FrameIndexEntry],
    recording_cache_directory: &Path,
) -> Result<()> {
    let mut cache = NamedTempFile::new_in(recording_cache_directory)
        .wrap_err("failed to create original-frame cache")?;
    let mut frame_index = 0;
    let mut offset = 0_u64;
    scan_messages(path, |message| {
        if message.channel.topic != image_topic.name() {
            return Ok(ScanControl::Continue);
        }
        let expected = expected_frames
            .get_mut(frame_index)
            .wrap_err("recording contains more source images than its index")?;
        if message.sequence != expected.source_sequence
            || message.log_time != expected.source_log_time
            || message.publish_time != expected.source_publish_time
            || blake3::hash(message.data.as_ref()).as_bytes() != &expected.source_data_hash
        {
            bail!("source image {frame_index} changed while building its cache");
        }
        let image = decode_image_data(message, image_topic)?;
        let bytes = SerdeCdrCodec::<Image>::serialize(&image)
            .wrap_err_with(|| format!("failed to encode original frame {frame_index}"))?;
        cache
            .write_all(&bytes)
            .wrap_err("failed to append original image data")?;
        let length = u64::try_from(bytes.len()).wrap_err("original frame length overflow")?;
        expected.source_byte_offset = Some(offset);
        expected.source_byte_length = Some(length);
        offset = offset
            .checked_add(length)
            .wrap_err("original frame cache length overflow")?;
        frame_index += 1;
        Ok(if frame_index == expected_frames.len() {
            ScanControl::Stop
        } else {
            ScanControl::Continue
        })
    })?;
    if frame_index != expected_frames.len() {
        bail!("recording ended before the original-frame cache was complete");
    }
    persist_original_frame_cache(cache, recording_cache_directory)
}

fn scan_linear_original_images(
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
        let expected = expected_frames
            .get(frame_index)
            .wrap_err("original recording has more frames than its cache index")?;
        let image = decode_image_data(message, image_topic)?;
        if !send_original_frame(sender, frame_index, expected, image)? {
            return Ok(ScanControl::Stop);
        }
        emitted += 1;
        Ok(if emitted == take {
            ScanControl::Stop
        } else {
            ScanControl::Continue
        })
    })?;
    if emitted != take {
        bail!("recording ended after {emitted} of {take} requested original frames");
    }
    Ok(())
}

fn scan_indexed_original_images(
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
    let summary = load_summary(path)?.wrap_err("recording no longer has a usable MCAP summary")?;
    let first = expected_frames
        .get(skip)
        .wrap_err("requested original frame is outside the cache index")?;
    let options = IndexedReaderOptions::new()
        .with_order(ReadOrder::File)
        .include_topics([image_topic.name()])
        .log_time_on_or_after(first.source_log_time)
        .with_record_length_limit(MAX_MCAP_RECORD_BYTES);
    let mut reader = IndexedReader::new_with_options(&summary, options)
        .wrap_err("failed to initialize indexed MCAP reader")?;
    let mut file = File::open(path)
        .wrap_err_with(|| format!("failed to open recording {}", path.display()))?;
    let file_length = file
        .metadata()
        .wrap_err("failed to inspect indexed MCAP")?
        .len();
    let mut buffer = Vec::new();
    let mut emitted = 0_usize;
    while let Some(event) = reader.next_event() {
        if sender.is_closed() {
            return Ok(());
        }
        match event.wrap_err("failed to read indexed MCAP messages")? {
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                read_indexed_chunk(&mut file, file_length, offset, length, &mut buffer)?;
                reader
                    .insert_chunk_record_data(offset, &buffer)
                    .wrap_err("failed to decode indexed MCAP chunk")?;
            }
            IndexedReadEvent::Message { header, data } => {
                let frame_index = skip + emitted;
                let expected = expected_frames
                    .get(frame_index)
                    .wrap_err("indexed MCAP contains more frames than its cache index")?;
                if header.sequence != expected.source_sequence
                    || header.log_time != expected.source_log_time
                    || header.publish_time != expected.source_publish_time
                    || blake3::hash(data).as_bytes() != &expected.source_data_hash
                {
                    if emitted == 0 {
                        continue;
                    }
                    bail!("indexed original frame {frame_index} does not match its cache index");
                }
                let image = match image_topic {
                    ImageTopic::LeftImage => SerdeCdrCodec::<Image>::deserialize(data)
                        .wrap_err("failed to decode indexed original image")?,
                    ImageTopic::StereoImagePair => {
                        SerdeCdrCodec::<StereoImagePair>::deserialize(data)
                            .wrap_err("failed to decode indexed original stereo pair")?
                            .left
                    }
                };
                if !send_original_frame(sender, frame_index, expected, image)? {
                    return Ok(());
                }
                emitted += 1;
                if emitted == take {
                    return Ok(());
                }
            }
        }
    }
    if emitted != take {
        bail!("recording ended after {emitted} of {take} requested original frames");
    }
    Ok(())
}

fn verify_indexed_images(
    path: &Path,
    image_topic: ImageTopic,
    expected_frames: &[FrameIndexEntry],
) -> Result<bool> {
    let Some(summary) = load_summary(path)? else {
        return Ok(false);
    };
    let options = IndexedReaderOptions::new()
        .with_order(ReadOrder::File)
        .include_topics([image_topic.name()])
        .with_record_length_limit(MAX_MCAP_RECORD_BYTES);
    let mut reader = IndexedReader::new_with_options(&summary, options)
        .wrap_err("failed to initialize indexed MCAP verification")?;
    let mut file = File::open(path)
        .wrap_err_with(|| format!("failed to open recording {}", path.display()))?;
    let file_length = file
        .metadata()
        .wrap_err("failed to inspect indexed MCAP")?
        .len();
    let mut buffer = Vec::new();
    let mut frame_index = 0;
    while let Some(event) = reader.next_event() {
        match event.wrap_err("failed to verify indexed MCAP messages")? {
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                read_indexed_chunk(&mut file, file_length, offset, length, &mut buffer)?;
                reader
                    .insert_chunk_record_data(offset, &buffer)
                    .wrap_err("failed to decode indexed MCAP chunk")?;
            }
            IndexedReadEvent::Message { header, data } => {
                let Some(expected) = expected_frames.get(frame_index) else {
                    return Ok(false);
                };
                if header.sequence != expected.source_sequence
                    || header.log_time != expected.source_log_time
                    || header.publish_time != expected.source_publish_time
                    || blake3::hash(data).as_bytes() != &expected.source_data_hash
                {
                    return Ok(false);
                }
                frame_index += 1;
            }
        }
    }
    Ok(frame_index == expected_frames.len())
}

fn read_indexed_chunk(
    file: &mut File,
    file_length: u64,
    offset: u64,
    length: usize,
    buffer: &mut Vec<u8>,
) -> Result<()> {
    if length > MAX_MCAP_RECORD_BYTES
        || offset
            .checked_add(u64::try_from(length).wrap_err("MCAP chunk length overflow")?)
            .is_none_or(|end| end > file_length)
    {
        bail!("indexed MCAP chunk request is outside the recording");
    }
    buffer.clear();
    if buffer.capacity() < length {
        buffer
            .try_reserve_exact(length - buffer.capacity())
            .wrap_err("failed to allocate indexed MCAP chunk")?;
    }
    buffer.resize(length, 0);
    file.seek(SeekFrom::Start(offset))
        .wrap_err("failed to seek to indexed MCAP chunk")?;
    file.read_exact(buffer)
        .wrap_err("failed to read indexed MCAP chunk")
}

fn scan_cached_original_images(
    path: &Path,
    expected_frames: &[FrameIndexEntry],
    skip: usize,
    take: usize,
    sender: &mpsc::Sender<Result<OriginalFrame>>,
) -> Result<()> {
    let mut file = File::open(path)
        .wrap_err_with(|| format!("failed to open original-frame cache {}", path.display()))?;
    for (frame_index, expected) in expected_frames.iter().enumerate().skip(skip).take(take) {
        if sender.is_closed() {
            return Ok(());
        }
        let offset = expected
            .source_byte_offset
            .wrap_err("original-frame cache offset is missing")?;
        let length = usize::try_from(
            expected
                .source_byte_length
                .wrap_err("original-frame cache length is missing")?,
        )
        .wrap_err("original frame is too large for this platform")?;
        let pixel_count = usize::try_from(expected.width)
            .ok()
            .and_then(|width| {
                usize::try_from(expected.height)
                    .ok()
                    .and_then(|height| width.checked_mul(height))
            })
            .wrap_err("cached original frame dimensions overflow")?;
        let maximum_length = pixel_count
            .checked_mul(4)
            .and_then(|length| length.checked_add(IMAGE_CACHE_OVERHEAD_BYTES))
            .wrap_err("cached original frame size limit overflow")?;
        if length > maximum_length {
            bail!("cached original frame {frame_index} is implausibly large");
        }
        let mut bytes = Vec::new();
        bytes
            .try_reserve_exact(length)
            .wrap_err("failed to allocate cached original frame")?;
        bytes.resize(length, 0);
        file.seek(SeekFrom::Start(offset))
            .wrap_err("failed to seek original-frame cache")?;
        file.read_exact(&mut bytes)
            .wrap_err("failed to read original-frame cache")?;
        let image = SerdeCdrCodec::<Image>::deserialize(&bytes)
            .wrap_err("failed to decode cached original image")?;
        if !send_original_frame(sender, frame_index, expected, image)? {
            return Ok(());
        }
    }
    Ok(())
}

fn send_original_frame(
    sender: &mpsc::Sender<Result<OriginalFrame>>,
    frame_index: usize,
    expected: &FrameIndexEntry,
    image: Image,
) -> Result<bool> {
    let timestamp_nanos = image_timestamp_nanos(&image)?;
    if timestamp_nanos != expected.timestamp_nanos
        || image.width != expected.width
        || image.height != expected.height
    {
        bail!("original frame {frame_index} does not match its cache index");
    }
    Ok(sender
        .blocking_send(Ok(OriginalFrame {
            frame_index,
            timestamp_nanos,
            image,
        }))
        .is_ok())
}

fn load_summary(path: &Path) -> Result<Option<mcap::Summary>> {
    let mut file = File::open(path)
        .wrap_err_with(|| format!("failed to open recording {}", path.display()))?;
    let mut reader = SummaryReader::new();
    while let Some(event) = reader.next_event() {
        let event = match event {
            Ok(event) => event,
            Err(error @ McapError::Io(_)) => {
                return Err(error).wrap_err("failed to read MCAP summary");
            }
            Err(error) => {
                tracing::warn!(?error, "ignoring unusable MCAP summary");
                return Ok(None);
            }
        };
        match event {
            SummaryReadEvent::SeekRequest(position) => {
                let position = file
                    .seek(position)
                    .wrap_err("failed to seek MCAP summary")?;
                reader.notify_seeked(position);
            }
            SummaryReadEvent::ReadRequest(length) => {
                let read = file
                    .read(reader.insert(length))
                    .wrap_err("failed to read MCAP summary")?;
                reader.notify_read(read);
            }
        }
    }
    Ok(reader.finish())
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
    let options = LinearReaderOptions::default()
        .with_record_length_limit(MAX_MCAP_RECORD_BYTES)
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
            Ok(LinearReadEvent::Record { data, opcode }) => {
                let record = match parse_record(opcode, data) {
                    Ok(record) => record,
                    Err(error) => {
                        return classify_scan_error(path, &mut file, file_length, error);
                    }
                };
                match record {
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
                }
            }
            Err(error) => return classify_scan_error(path, &mut file, file_length, error),
        }
    }

    Ok(ScanEnd::Complete)
}

fn classify_scan_error(
    path: &Path,
    file: &mut File,
    file_length: u64,
    error: McapError,
) -> Result<ScanEnd> {
    let position = file
        .stream_position()
        .wrap_err_with(|| format!("failed to inspect {}", path.display()))?;
    let current_chunk_extends_to_eof =
        current_chunk_record_extends_to_eof(file, position, file_length)?;
    if is_recoverable_tail_error(&error, position, file_length, current_chunk_extends_to_eof) {
        return Ok(ScanEnd::DamagedTail(error));
    }
    Err(error).wrap_err_with(|| {
        format!("failed while scanning MCAP messages at byte {position} of {file_length}")
    })
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
        McapError::UnexpectedEof
        | McapError::UnexpectedEoc
        | McapError::BadChunkLength { .. }
        | McapError::BadSchemaLength { .. }
        | McapError::BadAttachmentLength { .. }
        | McapError::Parse(_) => read_position == file_length || current_chunk_extends_to_eof,
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
    original_frames_path: &Path,
) -> Result<()> {
    if index.cache_version != CACHE_VERSION || &index.fingerprint != fingerprint {
        bail!("recording cache fingerprint or version does not match");
    }
    if index.frames.is_empty() {
        bail!("recording cache contains no frames");
    }
    let mut expected_offset = 0_u64;
    let mut expected_source_offset = 0_u64;
    for (frame_index, frame) in index.frames.iter().enumerate() {
        if frame.frame_index != frame_index
            || frame.byte_offset != expected_offset
            || frame.byte_length == 0
            || frame.width == 0
            || frame.height == 0
        {
            bail!("recording cache frame {frame_index} is invalid");
        }
        match index.original_image_source {
            OriginalImageSource::IndexedMcap | OriginalImageSource::LinearMcap => {
                if frame.source_byte_offset.is_some() || frame.source_byte_length.is_some() {
                    bail!("indexed recording frame {frame_index} has cached source offsets");
                }
            }
            OriginalImageSource::CachedCdr => {
                let length = frame
                    .source_byte_length
                    .filter(|length| *length > 0)
                    .wrap_err("cached original frame has an invalid length")?;
                if frame.source_byte_offset != Some(expected_source_offset) {
                    bail!("cached original frame {frame_index} has an invalid offset");
                }
                expected_source_offset = expected_source_offset
                    .checked_add(length)
                    .wrap_err("original-frame cache offset overflow")?;
            }
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
    if index.original_image_source == OriginalImageSource::CachedCdr {
        let source_length = original_frames_path
            .metadata()
            .wrap_err_with(|| {
                format!(
                    "failed to read metadata for {}",
                    original_frames_path.display()
                )
            })?
            .len();
        if source_length != expected_source_offset {
            bail!("original-frame cache length does not match recording cache index");
        }
    }
    Ok(())
}

fn validate_cached_baseline(
    directory: &Path,
    baseline: &RecordedBaseline,
    fingerprint: &RecordingFingerprint,
    frames: &[FrameIndexEntry],
) -> Result<()> {
    validate_baseline(directory, baseline, fingerprint, frames.len())
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
        assert_eq!(
            recording.index().original_image_source,
            OriginalImageSource::IndexedMcap
        );
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .unwrap();
        runtime.block_on(async {
            let mut originals = recording.original_images(0, Some(1)).unwrap();
            let original = originals.next().await.unwrap().unwrap();
            assert_eq!(original.timestamp_nanos, 12_000_000_345);
            assert_eq!(original.image.data, image.data);
        });
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
