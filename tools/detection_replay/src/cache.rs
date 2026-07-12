use std::{
    collections::BTreeMap,
    fs::{self, File, OpenOptions},
    io::{BufWriter, Read, Write},
    ops::RangeInclusive,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
    time::UNIX_EPOCH,
};

use bincode::Options;
use color_eyre::{
    Result,
    eyre::{Context, ContextCompat, bail, eyre},
};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use tempfile::NamedTempFile;
use types::{
    object_detection::{Object, RobocupObjectLabel, YOLOObjectLabel},
    pose_detection::Pose,
};

pub const CACHE_VERSION: u32 = 2;
pub const PREDICTION_CHUNK_SIZE: usize = 128;
const RECORDING_FINGERPRINT_VERSION: u32 = 1;

const MAX_CACHE_FILE_BYTES: u64 = 256 * 1024 * 1024;
const MAX_PREDICTIONS_PER_FRAME: usize = 300;

const MODEL_RUNS_DIRECTORY: &str = "model-runs";
const MODEL_RUN_LOCKS_DIRECTORY: &str = ".locks";
const RECORDINGS_DIRECTORY: &str = "recordings";
const MANIFEST_FILE: &str = "manifest.bin";
const MIGRATED_MANIFEST_FILE: &str = "manifest-v2.bin";
const RECORDED_BASELINE_DIRECTORY: &str = "recorded-baseline";
const RECORDED_BASELINE_FILE: &str = "manifest.bin";
const RUN_METADATA_FILE: &str = "run-metadata.bin";
const RUN_METADATA_LOCK_FILE: &str = ".run-metadata.lock";
const BOOKMARKS_FILE: &str = "bookmarks.bin";

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RecordingFingerprint {
    pub canonical_path: PathBuf,
    pub size: u64,
    pub modified_unix_nanos: u128,
    pub format_version: u32,
}

impl RecordingFingerprint {
    pub fn for_path(path: impl AsRef<Path>) -> Result<Self> {
        let canonical_path = path
            .as_ref()
            .canonicalize()
            .wrap_err_with(|| format!("failed to canonicalize {}", path.as_ref().display()))?;
        let metadata = canonical_path.metadata().wrap_err_with(|| {
            format!("failed to read metadata for {}", canonical_path.display())
        })?;
        if !metadata.is_file() {
            bail!("recording is not a file: {}", canonical_path.display());
        }
        let modified = metadata
            .modified()
            .wrap_err_with(|| format!("failed to read mtime for {}", canonical_path.display()))?
            .duration_since(UNIX_EPOCH)
            .map_err(|error| eyre!("recording mtime predates Unix epoch: {error}"))?;

        Ok(Self {
            canonical_path,
            size: metadata.len(),
            modified_unix_nanos: modified.as_nanos(),
            // Keep recording cache location stable across prediction-cache schema changes.
            format_version: RECORDING_FINGERPRINT_VERSION,
        })
    }

    pub fn cache_key(&self) -> Result<String> {
        let bytes =
            bincode::serialize(self).wrap_err("failed to serialize recording fingerprint")?;
        Ok(blake3::hash(&bytes).to_hex().to_string())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DetectionThresholds {
    pub minimum_candidate_confidence: f32,
    pub maximum_intersection_over_union: f32,
}

impl Default for DetectionThresholds {
    fn default() -> Self {
        Self {
            minimum_candidate_confidence: 0.05,
            maximum_intersection_over_union: 0.4,
        }
    }
}

impl DetectionThresholds {
    pub fn validate(self) -> Result<Self> {
        for (name, value) in [
            (
                "minimum candidate confidence",
                self.minimum_candidate_confidence,
            ),
            (
                "maximum intersection over union",
                self.maximum_intersection_over_union,
            ),
        ] {
            if !value.is_finite() || !(0.0..=1.0).contains(&value) {
                bail!("{name} must be finite and between 0 and 1, got {value}");
            }
        }
        Ok(Self {
            minimum_candidate_confidence: normalize_zero(self.minimum_candidate_confidence),
            maximum_intersection_over_union: normalize_zero(self.maximum_intersection_over_union),
        })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Prediction {
    pub frame_index: usize,
    pub timestamp_nanos: i64,
    pub objects: Vec<Object<RobocupObjectLabel>>,
    pub poses: Option<Vec<Pose<YOLOObjectLabel>>>,
    pub inference_duration_nanos: Option<u64>,
    pub postprocessing_duration_nanos: Option<u64>,
    pub non_maximum_suppression_duration_nanos: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ModelRunState {
    Running,
    Complete,
    Incomplete,
    Failed,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ModelRunManifest {
    pub run_key: String,
    pub label: String,
    pub canonical_model_path: PathBuf,
    pub model_hash: [u8; 32],
    pub recording_fingerprint: RecordingFingerprint,
    pub thresholds: DetectionThresholds,
    pub total_frame_count: usize,
    pub frame_start: usize,
    pub frame_end: usize,
    pub completed_frame_count: usize,
    pub cache_version: u32,
    pub state: ModelRunState,
    pub error: Option<String>,
    pub provider_note: Option<String>,
}

impl ModelRunManifest {
    pub fn new(
        label: impl Into<String>,
        canonical_model_path: PathBuf,
        model_hash: [u8; 32],
        recording_fingerprint: RecordingFingerprint,
        thresholds: DetectionThresholds,
        total_frame_count: usize,
        frame_range: RangeInclusive<usize>,
    ) -> Result<Self> {
        let label = label.into();
        let thresholds = thresholds.validate()?;
        let frame_start = *frame_range.start();
        let frame_end = *frame_range.end();
        validate_frame_range(frame_start, frame_end, total_frame_count)?;
        let run_key = model_run_key(
            &canonical_model_path,
            &model_hash,
            &recording_fingerprint,
            thresholds,
            frame_start,
            frame_end,
            total_frame_count,
        )?;
        Ok(Self {
            run_key,
            label,
            canonical_model_path,
            model_hash,
            recording_fingerprint,
            thresholds,
            total_frame_count,
            frame_start,
            frame_end,
            completed_frame_count: 0,
            cache_version: CACHE_VERSION,
            state: ModelRunState::Running,
            error: None,
            provider_note: None,
        })
    }

    pub fn target_frame_count(&self) -> Result<usize> {
        validate_frame_range(self.frame_start, self.frame_end, self.total_frame_count)?;
        self.frame_end
            .checked_sub(self.frame_start)
            .and_then(|length| length.checked_add(1))
            .wrap_err("model run frame count overflow")
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PredictionSource {
    Model,
    RecordedBaseline,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PredictionAvailability {
    ranges: Vec<(usize, usize)>,
}

impl PredictionAvailability {
    pub fn from_sparse(predictions: &[Option<Prediction>]) -> Self {
        let mut ranges = Vec::new();
        let mut start = None;
        for (frame, prediction) in predictions.iter().enumerate() {
            match (start, prediction.is_some()) {
                (None, true) => start = Some(frame),
                (Some(range_start), false) => {
                    ranges.push((range_start, frame - 1));
                    start = None;
                }
                _ => {}
            }
        }
        if let Some(start) = start {
            ranges.push((start, predictions.len() - 1));
        }
        Self { ranges }
    }

    fn contiguous(start: usize, count: usize) -> Result<Self> {
        if count == 0 {
            return Ok(Self::default());
        }
        let end = start
            .checked_add(count - 1)
            .wrap_err("prediction availability range overflow")?;
        Ok(Self {
            ranges: vec![(start, end)],
        })
    }

    pub fn contains(&self, frame: usize) -> bool {
        let index = self.ranges.partition_point(|(_, end)| *end < frame);
        self.ranges
            .get(index)
            .is_some_and(|(start, end)| *start <= frame && frame <= *end)
    }

    pub fn any_in(&self, range: RangeInclusive<usize>) -> bool {
        let start = *range.start();
        let end = *range.end();
        let index = self
            .ranges
            .partition_point(|(_, range_end)| *range_end < start);
        self.ranges
            .get(index)
            .is_some_and(|(range_start, _)| *range_start <= end)
    }

    pub fn count_in(&self, range: RangeInclusive<usize>) -> usize {
        let start = *range.start();
        let end = *range.end();
        self.ranges
            .iter()
            .skip_while(|(_, range_end)| *range_end < start)
            .take_while(|(range_start, _)| *range_start <= end)
            .map(|(range_start, range_end)| range_end.min(&end) - range_start.max(&start) + 1)
            .sum()
    }
}

#[derive(Clone, Debug)]
pub struct LoadedPredictionRun {
    pub key: String,
    pub label: String,
    pub source: PredictionSource,
    pub manifest: Option<ModelRunManifest>,
    pub availability: PredictionAvailability,
    storage: PredictionStorage,
    cached_chunk: Arc<Mutex<Option<CachedPredictionChunk>>>,
    failed_chunks: Arc<Mutex<BTreeMap<usize, String>>>,
}

#[derive(Clone, Debug)]
enum PredictionStorage {
    RecordedBaseline {
        directory: PathBuf,
        total_frame_count: usize,
    },
    Model {
        directory: PathBuf,
        frame_start: usize,
    },
}

#[derive(Debug)]
struct CachedPredictionChunk {
    index: usize,
    start_frame: usize,
    predictions: Vec<Option<Arc<Prediction>>>,
}

impl LoadedPredictionRun {
    pub fn is_available(&self, frame: usize) -> bool {
        self.availability.contains(frame)
    }

    pub fn any_available(&self, range: RangeInclusive<usize>) -> bool {
        self.availability.any_in(range)
    }

    pub fn available_count(&self, range: RangeInclusive<usize>) -> usize {
        self.availability.count_in(range)
    }

    /// A present empty prediction means inference ran and found nothing. `None` means absent.
    pub fn prediction(&self, frame: usize) -> Result<Option<Arc<Prediction>>> {
        if !self.is_available(frame) {
            return Ok(None);
        }
        let (chunk_index, path) = match &self.storage {
            PredictionStorage::RecordedBaseline { directory, .. } => {
                let index = frame / PREDICTION_CHUNK_SIZE;
                (index, baseline_chunk_path(directory, index))
            }
            PredictionStorage::Model {
                directory,
                frame_start,
            } => {
                let relative = frame
                    .checked_sub(*frame_start)
                    .wrap_err("prediction precedes model run range")?;
                let index = relative / PREDICTION_CHUNK_SIZE;
                (index, chunk_path(directory, index))
            }
        };
        if let Some(error) = self
            .failed_chunks
            .lock()
            .map_err(|_| eyre!("failed-chunk cache for run {} was poisoned", self.key))?
            .get(&chunk_index)
            .cloned()
        {
            bail!("{error}");
        }
        let mut cached = self
            .cached_chunk
            .lock()
            .map_err(|_| eyre!("prediction chunk cache for run {} was poisoned", self.key))?;
        if cached
            .as_ref()
            .is_none_or(|chunk| chunk.index != chunk_index)
        {
            match load_prediction_chunk(&path, chunk_index, &self.storage, &self.availability) {
                Ok(chunk) => *cached = Some(chunk),
                Err(error) => {
                    let error = format!("{error:#}");
                    self.failed_chunks
                        .lock()
                        .map_err(|_| eyre!("failed-chunk cache for run {} was poisoned", self.key))?
                        .insert(chunk_index, error.clone());
                    bail!("{error}");
                }
            }
        }
        let chunk = cached
            .as_ref()
            .wrap_err("prediction chunk cache was unexpectedly empty")?;
        let offset = frame
            .checked_sub(chunk.start_frame)
            .wrap_err("prediction precedes cached chunk")?;
        chunk.predictions.get(offset).cloned().wrap_err_with(|| {
            format!(
                "frame {frame} is outside prediction chunk {}",
                path.display()
            )
        })
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct RunUiMetadata {
    pub renamed_label: Option<String>,
    pub hidden: bool,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Bookmark {
    pub name: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct BookmarkCollection(pub BTreeMap<usize, Bookmark>);

impl BookmarkCollection {
    pub fn toggle(&mut self, frame: usize) {
        if self.0.remove(&frame).is_none() {
            let next_number = self
                .0
                .values()
                .filter_map(|bookmark| bookmark.name.strip_prefix('#')?.parse::<usize>().ok())
                .max()
                .unwrap_or(0)
                + 1;
            self.0.insert(
                frame,
                Bookmark {
                    name: format!("#{next_number}"),
                },
            );
        }
    }

    pub fn next(&self, frame: usize, start: usize, end: usize) -> Option<usize> {
        (frame < end)
            .then(|| {
                self.0
                    .range(frame + 1..=end)
                    .next()
                    .map(|(frame, _)| *frame)
            })
            .flatten()
            .or_else(|| self.0.range(start..frame).next().map(|(frame, _)| *frame))
    }

    pub fn previous(&self, frame: usize, start: usize, end: usize) -> Option<usize> {
        self.0
            .range(start..frame)
            .next_back()
            .map(|(frame, _)| *frame)
            .or_else(|| {
                (frame < end)
                    .then(|| {
                        self.0
                            .range(frame + 1..=end)
                            .next_back()
                            .map(|(frame, _)| *frame)
                    })
                    .flatten()
            })
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct RunUiMetadataFile {
    cache_version: u32,
    recording_fingerprint: RecordingFingerprint,
    runs: BTreeMap<String, RunUiMetadata>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct BookmarkFile {
    cache_version: u32,
    recording_fingerprint: RecordingFingerprint,
    bookmarks: BookmarkCollection,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordedBaseline {
    pub cache_version: u32,
    pub recording_fingerprint: RecordingFingerprint,
    pub total_frame_count: usize,
    pub availability: PredictionAvailability,
}

#[derive(Debug, Serialize, Deserialize)]
struct RecordedBaselineChunk {
    start_frame: usize,
    predictions: Vec<Option<Prediction>>,
}

#[derive(Serialize)]
struct RecordedBaselineChunkRef<'a> {
    start_frame: usize,
    predictions: &'a [Option<Prediction>],
}

#[derive(Debug, Serialize, Deserialize)]
struct PredictionChunk {
    start_frame: usize,
    predictions: Vec<Prediction>,
}

#[derive(Debug, Serialize, Deserialize)]
struct LegacyModelRunManifest {
    run_key: String,
    label: String,
    canonical_model_path: PathBuf,
    model_hash: [u8; 32],
    recording_fingerprint: RecordingFingerprint,
    thresholds: DetectionThresholds,
    total_frame_count: usize,
    completed_frame_count: usize,
    cache_version: u32,
    state: ModelRunState,
    error: Option<String>,
    provider_note: Option<String>,
}

pub struct PredictionStore {
    _lock_file: File,
    run_directory: PathBuf,
    manifest: ModelRunManifest,
    completed_count: usize,
    pending: Vec<Prediction>,
}

impl PredictionStore {
    pub fn open(cache_directory: impl AsRef<Path>, proposed: ModelRunManifest) -> Result<Self> {
        let frame_count = proposed.target_frame_count()?;
        let root = cache_directory.as_ref().join(MODEL_RUNS_DIRECTORY);
        fs::create_dir_all(&root)
            .wrap_err_with(|| format!("failed to create {}", root.display()))?;
        let lock_file = lock_model_run(&root, &proposed.run_key, true)?;
        let run_directory = root.join(&proposed.run_key);
        fs::create_dir_all(&run_directory)
            .wrap_err_with(|| format!("failed to create {}", run_directory.display()))?;
        let manifest_path = run_directory.join(MANIFEST_FILE);

        let mut manifest = if manifest_path.exists() {
            let existing: ModelRunManifest = read_bincode(&manifest_path)?;
            validate_model_run_manifest(&existing)?;
            validate_same_run(&existing, &proposed)?;
            ModelRunManifest {
                label: proposed.label.clone(),
                ..existing
            }
        } else {
            write_bincode_atomic(&manifest_path, &proposed)?;
            proposed
        };

        let loaded = match load_contiguous_predictions(&run_directory, &manifest) {
            Ok(loaded) => loaded,
            Err(error) => {
                manifest.state = ModelRunState::Failed;
                manifest.error = Some(format!("invalid prediction cache: {error:#}"));
                if let Err(persist_error) = write_bincode_atomic(&manifest_path, &manifest) {
                    return Err(error.wrap_err(format!(
                        "failed to persist invalid-cache state: {persist_error:#}"
                    )));
                }
                return Err(error);
            }
        };
        if manifest.completed_frame_count != loaded.completed_count {
            manifest.completed_frame_count = loaded.completed_count;
            write_bincode_atomic(&manifest_path, &manifest)?;
        }
        if manifest.state == ModelRunState::Running
            || manifest.state == ModelRunState::Complete && loaded.completed_count != frame_count
        {
            manifest.state = ModelRunState::Incomplete;
            write_bincode_atomic(&manifest_path, &manifest)?;
        }

        Ok(Self {
            _lock_file: lock_file,
            run_directory,
            manifest,
            completed_count: loaded.completed_count,
            pending: loaded.pending,
        })
    }

    pub fn manifest(&self) -> &ModelRunManifest {
        &self.manifest
    }

    pub fn completed_count(&self) -> usize {
        self.completed_count
    }

    pub fn begin(&mut self, provider_note: impl Into<String>) -> Result<()> {
        self.manifest.state = ModelRunState::Running;
        self.manifest.error = None;
        self.manifest.provider_note = Some(provider_note.into());
        self.persist_manifest()
    }

    pub fn append(&mut self, prediction: Prediction) -> Result<()> {
        let expected_frame = self
            .manifest
            .frame_start
            .checked_add(self.completed_count)
            .wrap_err("expected prediction frame overflow")?;
        if prediction.frame_index != expected_frame {
            bail!(
                "prediction frame {} is not contiguous; expected {}",
                prediction.frame_index,
                expected_frame
            );
        }
        validate_prediction(&prediction)?;
        if self.completed_count >= self.manifest.target_frame_count()? {
            bail!("prediction exceeds the manifest frame count");
        }

        self.pending.push(prediction);
        self.completed_count += 1;
        if self.pending.len() == PREDICTION_CHUNK_SIZE {
            self.persist_pending()?;
            self.pending.clear();
        }
        Ok(())
    }

    pub fn finish(&mut self) -> Result<()> {
        self.persist_pending()?;
        self.manifest.state = if self.completed_count == self.manifest.target_frame_count()? {
            ModelRunState::Complete
        } else {
            ModelRunState::Incomplete
        };
        self.manifest.error = None;
        self.persist_manifest()
    }

    pub fn fail(&mut self, error: impl Into<String>) -> Result<()> {
        self.persist_pending()?;
        self.manifest.state = ModelRunState::Failed;
        self.manifest.error = Some(error.into());
        self.persist_manifest()
    }

    fn persist_manifest(&self) -> Result<()> {
        write_bincode_atomic(&self.run_directory.join(MANIFEST_FILE), &self.manifest)
    }

    fn persist_pending(&mut self) -> Result<()> {
        if self.manifest.completed_frame_count == self.completed_count {
            return Ok(());
        }
        let chunk_index = (self.completed_count - 1) / PREDICTION_CHUNK_SIZE;
        let relative_start = chunk_index
            .checked_mul(PREDICTION_CHUNK_SIZE)
            .wrap_err("prediction chunk start overflow")?;
        let chunk = PredictionChunkRef {
            start_frame: self
                .manifest
                .frame_start
                .checked_add(relative_start)
                .wrap_err("prediction chunk frame overflow")?,
            predictions: &self.pending,
        };
        write_bincode_atomic(&chunk_path(&self.run_directory, chunk_index), &chunk)?;
        self.manifest.completed_frame_count = self.completed_count;
        self.persist_manifest()
    }
}

struct LoadedContiguousPredictions {
    completed_count: usize,
    pending: Vec<Prediction>,
}

#[derive(Serialize)]
struct PredictionChunkRef<'a> {
    start_frame: usize,
    predictions: &'a [Prediction],
}

pub fn hash_model(path: impl AsRef<Path>) -> Result<(PathBuf, [u8; 32])> {
    let canonical = path
        .as_ref()
        .canonicalize()
        .wrap_err_with(|| format!("failed to canonicalize model {}", path.as_ref().display()))?;
    if !canonical.is_file() {
        bail!("model is not a file: {}", canonical.display());
    }
    let mut hasher = blake3::Hasher::new();
    hasher
        .update_mmap(&canonical)
        .wrap_err_with(|| format!("failed to hash model {}", canonical.display()))?;
    Ok((canonical, *hasher.finalize().as_bytes()))
}

pub fn save_recorded_baseline(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    predictions: &[Option<Prediction>],
) -> Result<()> {
    for (frame_index, prediction) in predictions.iter().enumerate() {
        if let Some(prediction) = prediction {
            if prediction.frame_index != frame_index {
                bail!("recorded baseline frame index does not match its sparse slot");
            }
            validate_prediction(prediction)?;
        }
    }
    let directory = recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?
        .join(RECORDED_BASELINE_DIRECTORY);
    if directory.exists() {
        fs::remove_dir_all(&directory)
            .wrap_err_with(|| format!("failed to replace {}", directory.display()))?;
    }
    fs::create_dir_all(&directory)
        .wrap_err_with(|| format!("failed to create {}", directory.display()))?;
    for (index, chunk) in predictions.chunks(PREDICTION_CHUNK_SIZE).enumerate() {
        write_bincode_atomic(
            &baseline_chunk_path(&directory, index),
            &RecordedBaselineChunkRef {
                start_frame: index * PREDICTION_CHUNK_SIZE,
                predictions: chunk,
            },
        )?;
    }
    write_bincode_atomic(
        &directory.join(RECORDED_BASELINE_FILE),
        &RecordedBaseline {
            cache_version: CACHE_VERSION,
            recording_fingerprint: recording_fingerprint.clone(),
            total_frame_count: predictions.len(),
            availability: PredictionAvailability::from_sparse(predictions),
        },
    )
}

pub fn load_all_runs(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<Vec<LoadedPredictionRun>> {
    let cache_directory = cache_directory.as_ref();
    migrate_legacy_model_runs(cache_directory, recording_fingerprint, total_frame_count)?;
    let mut runs = Vec::new();
    let baseline_directory = recording_cache_directory(cache_directory, recording_fingerprint)?
        .join(RECORDED_BASELINE_DIRECTORY);
    let baseline_path = baseline_directory.join(RECORDED_BASELINE_FILE);
    if baseline_path.exists() {
        let baseline: RecordedBaseline = read_bincode(&baseline_path)?;
        validate_baseline(
            &baseline_directory,
            &baseline,
            recording_fingerprint,
            total_frame_count,
        )?;
        runs.push(LoadedPredictionRun {
            key: "recorded".to_string(),
            label: "Recorded".to_string(),
            source: PredictionSource::RecordedBaseline,
            manifest: None,
            availability: baseline.availability,
            storage: PredictionStorage::RecordedBaseline {
                directory: baseline_directory,
                total_frame_count,
            },
            cached_chunk: Arc::new(Mutex::new(None)),
            failed_chunks: Arc::new(Mutex::new(BTreeMap::new())),
        });
    }

    let model_root = cache_directory.join(MODEL_RUNS_DIRECTORY);
    if !model_root.exists() {
        return Ok(runs);
    }
    let mut directories = fs::read_dir(&model_root)
        .wrap_err_with(|| format!("failed to read {}", model_root.display()))?
        .map(|entry| entry.map(|entry| entry.path()))
        .collect::<std::io::Result<Vec<_>>>()?;
    directories.sort();

    for directory in directories {
        if !directory.is_dir() {
            continue;
        }
        let name = directory
            .file_name()
            .and_then(|name| name.to_str())
            .unwrap_or_default();
        if name.starts_with(".deleting-") {
            if let Err(error) = fs::remove_dir_all(&directory) {
                tracing::warn!(
                    path = %directory.display(),
                    ?error,
                    "failed to clean stale deleted model run cache"
                );
            }
            continue;
        }
        if !directory.join(MANIFEST_FILE).exists() {
            continue;
        }
        match load_model_run(&directory, recording_fingerprint, total_frame_count) {
            Ok(Some(run)) => runs.push(run),
            Ok(None) => {}
            Err(error) => tracing::warn!(
                path = %directory.display(),
                ?error,
                "skipping invalid detection replay model cache"
            ),
        }
    }

    Ok(runs)
}

pub fn load_run_ui_metadata(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
) -> Result<BTreeMap<String, RunUiMetadata>> {
    let directory = recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?;
    read_run_ui_metadata(&directory.join(RUN_METADATA_FILE), recording_fingerprint)
}

pub fn save_run_ui_metadata(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    runs: &BTreeMap<String, RunUiMetadata>,
) -> Result<()> {
    let directory = recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?;
    fs::create_dir_all(&directory)
        .wrap_err_with(|| format!("failed to create {}", directory.display()))?;
    let _lock = lock_file(&directory.join(RUN_METADATA_LOCK_FILE), false)?;
    write_run_ui_metadata(&directory, recording_fingerprint, runs)
}

pub fn load_bookmarks(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
) -> Result<BookmarkCollection> {
    let path = recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?
        .join(BOOKMARKS_FILE);
    if !path.exists() {
        return Ok(BookmarkCollection::default());
    }
    let file: BookmarkFile = read_bincode(&path)?;
    if file.cache_version != CACHE_VERSION || &file.recording_fingerprint != recording_fingerprint {
        bail!("bookmarks do not match the recording cache");
    }
    Ok(file.bookmarks)
}

pub fn bookmarks_exist(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
) -> Result<bool> {
    Ok(
        recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?
            .join(BOOKMARKS_FILE)
            .exists(),
    )
}

pub fn save_bookmarks(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    bookmarks: &BookmarkCollection,
) -> Result<()> {
    let directory = recording_cache_directory(cache_directory.as_ref(), recording_fingerprint)?;
    fs::create_dir_all(&directory)
        .wrap_err_with(|| format!("failed to create {}", directory.display()))?;
    write_bincode_atomic(
        &directory.join(BOOKMARKS_FILE),
        &BookmarkFile {
            cache_version: CACHE_VERSION,
            recording_fingerprint: recording_fingerprint.clone(),
            bookmarks: bookmarks.clone(),
        },
    )
}

pub fn rename_run(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    run_key: &str,
    label: &str,
) -> Result<BTreeMap<String, RunUiMetadata>> {
    validate_model_run_key(run_key)?;
    let label = label.trim();
    if label.is_empty() {
        bail!("run name must not be empty");
    }
    update_run_ui_metadata(cache_directory.as_ref(), recording_fingerprint, |runs| {
        runs.entry(run_key.to_string()).or_default().renamed_label = Some(label.to_string())
    })
}

pub fn set_run_hidden(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    run_key: &str,
    hidden: bool,
) -> Result<BTreeMap<String, RunUiMetadata>> {
    validate_run_key(run_key)?;
    update_run_ui_metadata(cache_directory.as_ref(), recording_fingerprint, |runs| {
        runs.entry(run_key.to_string()).or_default().hidden = hidden
    })
}

pub fn remove_run_ui_metadata(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    run_key: &str,
) -> Result<BTreeMap<String, RunUiMetadata>> {
    validate_run_key(run_key)?;
    update_run_ui_metadata(cache_directory.as_ref(), recording_fingerprint, |runs| {
        runs.remove(run_key);
    })
}

pub fn delete_model_run(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    run_key: &str,
) -> Result<()> {
    validate_model_run_key(run_key)?;
    let root = cache_directory.as_ref().join(MODEL_RUNS_DIRECTORY);
    let _lock_file = lock_model_run(&root, run_key, true)?;
    let run_directory = root.join(run_key);
    let manifest: ModelRunManifest = read_bincode(&run_directory.join(MANIFEST_FILE))?;
    if manifest.run_key != run_key || &manifest.recording_fingerprint != recording_fingerprint {
        bail!("model run manifest does not match the requested recording and run key");
    }

    let mut suffix = 0_u32;
    let tombstone = loop {
        let candidate = root.join(format!(
            ".deleting-{run_key}-{}-{suffix}",
            std::process::id()
        ));
        if !candidate.exists() {
            break candidate;
        }
        suffix = suffix
            .checked_add(1)
            .wrap_err("deletion tombstone overflow")?;
    };
    fs::rename(&run_directory, &tombstone).wrap_err_with(|| {
        format!(
            "failed to detach model run cache {}",
            run_directory.display()
        )
    })?;
    if let Err(error) = fs::remove_dir_all(&tombstone) {
        tracing::warn!(
            path = %tombstone.display(),
            ?error,
            "model run was deleted but its detached cache could not be cleaned up"
        );
    }
    Ok(())
}

fn migrate_legacy_model_runs(
    cache_directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<()> {
    let root = cache_directory.join(MODEL_RUNS_DIRECTORY);
    let mut remapped_keys = BTreeMap::new();
    let mut directories = if root.exists() {
        fs::read_dir(&root)
            .wrap_err_with(|| format!("failed to read {}", root.display()))?
            .map(|entry| entry.map(|entry| entry.path()))
            .collect::<std::io::Result<Vec<_>>>()?
    } else {
        Vec::new()
    };
    directories.sort();
    for directory in &directories {
        if directory.join(MIGRATED_MANIFEST_FILE).exists() {
            match finish_staged_model_migration(&root, directory) {
                Ok(Some((old_key, new_key))) => {
                    remapped_keys.insert(old_key, new_key);
                }
                Ok(None) => {}
                Err(error) => tracing::warn!(
                    path = %directory.display(),
                    ?error,
                    "failed to finish staged model run migration"
                ),
            }
        }
    }
    directories = if root.exists() {
        fs::read_dir(&root)
            .wrap_err_with(|| format!("failed to read {}", root.display()))?
            .map(|entry| entry.map(|entry| entry.path()))
            .collect::<std::io::Result<Vec<_>>>()?
    } else {
        Vec::new()
    };
    directories.sort();
    for directory in directories {
        match migrate_legacy_model_run(&root, &directory, recording_fingerprint, total_frame_count)
        {
            Ok(Some((old_key, new_key))) => {
                remapped_keys.insert(old_key, new_key);
            }
            Ok(None) => {}
            Err(error) => tracing::warn!(
                path = %directory.display(),
                ?error,
                "skipping invalid legacy model run"
            ),
        }
    }
    if let Err(error) =
        migrate_legacy_run_ui_metadata(cache_directory, recording_fingerprint, &remapped_keys)
    {
        tracing::warn!(?error, "failed to migrate legacy run GUI metadata");
    }
    Ok(())
}

fn migrate_legacy_model_run(
    root: &Path,
    directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<Option<(String, String)>> {
    if !directory.is_dir() || !directory.join(MANIFEST_FILE).exists() {
        return Ok(None);
    }
    let directory_key = directory
        .file_name()
        .and_then(|name| name.to_str())
        .wrap_err("model run directory name is not valid UTF-8")?;
    if read_bincode::<ModelRunManifest>(&directory.join(MANIFEST_FILE)).is_ok() {
        return Ok(None);
    }
    let legacy: LegacyModelRunManifest = read_bincode(&directory.join(MANIFEST_FILE))?;
    if legacy.cache_version != 1
        || legacy.run_key != directory_key
        || &legacy.recording_fingerprint != recording_fingerprint
        || legacy.total_frame_count != total_frame_count
    {
        return Ok(None);
    }
    let _source_lock = lock_model_run(root, &legacy.run_key, true)?;
    let first_cached_frame = first_cached_frame(directory)?;
    let (frame_start, frame_end) =
        recover_legacy_frame_range(&legacy, first_cached_frame, total_frame_count)?;
    let mut migrated = ModelRunManifest::new(
        legacy.label,
        legacy.canonical_model_path,
        legacy.model_hash,
        legacy.recording_fingerprint,
        legacy.thresholds,
        legacy.total_frame_count,
        frame_start..=frame_end,
    )?;
    let loaded = load_contiguous_predictions(directory, &migrated)?;
    if legacy.completed_frame_count != loaded.completed_count {
        tracing::info!(
            manifest_count = legacy.completed_frame_count,
            durable_count = loaded.completed_count,
            "reconciled legacy model run with durable chunks"
        );
    }
    migrated.completed_frame_count = loaded.completed_count;
    migrated.state = match legacy.state {
        ModelRunState::Failed => ModelRunState::Failed,
        ModelRunState::Complete if loaded.completed_count == migrated.target_frame_count()? => {
            ModelRunState::Complete
        }
        _ => ModelRunState::Incomplete,
    };
    migrated.error = legacy.error;
    if migrated.state == ModelRunState::Failed && migrated.error.is_none() {
        migrated.error = Some("legacy model run failed without diagnostic details".to_string());
    }
    migrated.provider_note = legacy.provider_note;

    let target = root.join(&migrated.run_key);
    if target != directory && target.exists() {
        bail!("new model run cache {} already exists", target.display());
    }
    let _target_lock = lock_model_run(root, &migrated.run_key, true)?;
    write_bincode_atomic(&directory.join(MIGRATED_MANIFEST_FILE), &migrated)?;
    if target != directory {
        fs::rename(directory, &target).wrap_err_with(|| {
            format!(
                "failed to rename legacy model run {} to {}",
                directory.display(),
                target.display()
            )
        })?;
        sync_directory(root)?;
    }
    fs::rename(
        target.join(MIGRATED_MANIFEST_FILE),
        target.join(MANIFEST_FILE),
    )
    .wrap_err("failed to commit migrated model run manifest")?;
    sync_directory(&target)?;
    tracing::info!(
        run_key = migrated.run_key,
        "migrated legacy model run cache"
    );
    Ok(Some((legacy.run_key, migrated.run_key)))
}

fn finish_staged_model_migration(
    root: &Path,
    directory: &Path,
) -> Result<Option<(String, String)>> {
    let migrated: ModelRunManifest = read_bincode(&directory.join(MIGRATED_MANIFEST_FILE))?;
    let legacy: LegacyModelRunManifest = read_bincode(&directory.join(MANIFEST_FILE))?;
    let target = root.join(&migrated.run_key);
    let _source_lock = lock_model_run(root, &legacy.run_key, true)?;
    let _target_lock = lock_model_run(root, &migrated.run_key, true)?;
    if target != directory {
        if target.exists() {
            bail!(
                "staged migration target {} already exists",
                target.display()
            );
        }
        fs::rename(directory, &target).wrap_err("failed to resume model run migration")?;
        sync_directory(root)?;
    }
    fs::rename(
        target.join(MIGRATED_MANIFEST_FILE),
        target.join(MANIFEST_FILE),
    )
    .wrap_err("failed to commit staged model run migration")?;
    sync_directory(&target)?;
    Ok(Some((legacy.run_key, migrated.run_key)))
}

fn first_cached_frame(directory: &Path) -> Result<Option<usize>> {
    let path = chunk_path(directory, 0);
    if !path.exists() {
        return Ok(None);
    }
    let chunk: PredictionChunk = read_bincode(&path)?;
    let prediction = chunk
        .predictions
        .first()
        .wrap_err("legacy prediction chunk is empty")?;
    if prediction.frame_index != chunk.start_frame {
        bail!("legacy prediction chunk starts with an inconsistent frame");
    }
    Ok(Some(chunk.start_frame))
}

fn recover_legacy_frame_range(
    manifest: &LegacyModelRunManifest,
    first_cached_frame: Option<usize>,
    total_frame_count: usize,
) -> Result<(usize, usize)> {
    if manifest.run_key
        == legacy_model_run_key(
            &manifest.canonical_model_path,
            &manifest.model_hash,
            &manifest.recording_fingerprint,
            manifest.thresholds,
            None,
        )?
    {
        return Ok((0, total_frame_count - 1));
    }
    let start = first_cached_frame
        .wrap_err("cannot recover the selected frame range of an empty legacy model run")?;
    for end in start..total_frame_count {
        if manifest.run_key
            == legacy_model_run_key(
                &manifest.canonical_model_path,
                &manifest.model_hash,
                &manifest.recording_fingerprint,
                manifest.thresholds,
                Some((start, end)),
            )?
        {
            return Ok((start, end));
        }
    }
    bail!(
        "could not recover the selected frame range of legacy run {}",
        manifest.run_key
    )
}

fn legacy_model_run_key(
    canonical_model_path: &Path,
    model_hash: &[u8; 32],
    recording_fingerprint: &RecordingFingerprint,
    thresholds: DetectionThresholds,
    frame_range: Option<(usize, usize)>,
) -> Result<String> {
    let identity = match frame_range {
        Some((start, end)) => bincode::serialize(&(
            canonical_model_path,
            model_hash,
            recording_fingerprint,
            thresholds,
            1_u32,
            start,
            end,
        )),
        None => bincode::serialize(&(
            canonical_model_path,
            model_hash,
            recording_fingerprint,
            thresholds,
            1_u32,
        )),
    }
    .wrap_err("failed to serialize legacy model run identity")?;
    let digest = blake3::hash(&identity).to_hex().to_string();
    Ok(format!("model-{}", &digest[..16]))
}

fn migrate_legacy_run_ui_metadata(
    cache_directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    remapped_keys: &BTreeMap<String, String>,
) -> Result<()> {
    let directory = recording_cache_directory(cache_directory, recording_fingerprint)?;
    let path = directory.join(RUN_METADATA_FILE);
    if !path.exists() {
        return Ok(());
    }
    let _lock = lock_file(&directory.join(RUN_METADATA_LOCK_FILE), false)?;
    let mut metadata: RunUiMetadataFile = read_bincode(&path)?;
    if !matches!(metadata.cache_version, 1 | CACHE_VERSION)
        || &metadata.recording_fingerprint != recording_fingerprint
    {
        return Ok(());
    }
    if metadata.cache_version == CACHE_VERSION && remapped_keys.is_empty() {
        return Ok(());
    }
    metadata.cache_version = CACHE_VERSION;
    metadata.runs = metadata
        .runs
        .into_iter()
        .map(|(key, value)| (remapped_keys.get(&key).cloned().unwrap_or(key), value))
        .collect();
    write_bincode_atomic(&path, &metadata)
}

fn load_model_run(
    directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<Option<LoadedPredictionRun>> {
    let mut manifest: ModelRunManifest = read_bincode(&directory.join(MANIFEST_FILE))?;
    validate_model_run_manifest(&manifest)?;
    if directory.file_name().and_then(|name| name.to_str()) != Some(&manifest.run_key) {
        bail!("model run directory name does not match its manifest key");
    }
    if &manifest.recording_fingerprint != recording_fingerprint {
        return Ok(None);
    }
    if manifest.total_frame_count != total_frame_count {
        bail!(
            "model run {} has {} frames, expected {}",
            manifest.run_key,
            manifest.total_frame_count,
            total_frame_count
        );
    }
    if manifest.cache_version != CACHE_VERSION {
        bail!("model run cache version does not match");
    }
    let target_count = manifest.target_frame_count()?;
    let inspection = inspect_model_chunks(directory, &manifest);
    let availability = match inspection {
        Ok(()) => {
            let active = model_run_is_active(directory, &manifest.run_key)?;
            if manifest.state == ModelRunState::Running && !active
                || manifest.state == ModelRunState::Complete
                    && manifest.completed_frame_count != target_count
            {
                manifest.state = ModelRunState::Incomplete;
            }
            PredictionAvailability::contiguous(
                manifest.frame_start,
                manifest.completed_frame_count,
            )?
        }
        Err(error) => {
            manifest.state = ModelRunState::Failed;
            manifest.error = Some(format!("invalid prediction cache: {error:#}"));
            PredictionAvailability::default()
        }
    };
    let frame_start = manifest.frame_start;
    Ok(Some(LoadedPredictionRun {
        key: manifest.run_key.clone(),
        label: manifest.label.clone(),
        source: PredictionSource::Model,
        manifest: Some(manifest),
        availability,
        storage: PredictionStorage::Model {
            directory: directory.to_path_buf(),
            frame_start,
        },
        cached_chunk: Arc::new(Mutex::new(None)),
        failed_chunks: Arc::new(Mutex::new(BTreeMap::new())),
    }))
}

pub(crate) fn recording_cache_directory(
    cache_directory: &Path,
    fingerprint: &RecordingFingerprint,
) -> Result<PathBuf> {
    Ok(cache_directory
        .join(RECORDINGS_DIRECTORY)
        .join(fingerprint.cache_key()?))
}

pub(crate) fn recorded_baseline_path(
    cache_directory: &Path,
    fingerprint: &RecordingFingerprint,
) -> Result<PathBuf> {
    Ok(recording_cache_directory(cache_directory, fingerprint)?
        .join(RECORDED_BASELINE_DIRECTORY)
        .join(RECORDED_BASELINE_FILE))
}

pub(crate) fn read_bincode<T: DeserializeOwned>(path: &Path) -> Result<T> {
    let file = File::open(path).wrap_err_with(|| format!("failed to open {}", path.display()))?;
    let length = file
        .metadata()
        .wrap_err_with(|| format!("failed to inspect {}", path.display()))?
        .len();
    if length > MAX_CACHE_FILE_BYTES {
        bail!(
            "cache file {} is too large ({length} bytes, limit {MAX_CACHE_FILE_BYTES})",
            path.display()
        );
    }
    let mut bytes = Vec::with_capacity(usize::try_from(length).unwrap_or_default());
    file.take(MAX_CACHE_FILE_BYTES + 1)
        .read_to_end(&mut bytes)
        .wrap_err_with(|| format!("failed to read {}", path.display()))?;
    if bytes.len() as u64 > MAX_CACHE_FILE_BYTES {
        bail!("cache file {} grew beyond its size limit", path.display());
    }
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(MAX_CACHE_FILE_BYTES)
        .reject_trailing_bytes()
        .deserialize(&bytes)
        .wrap_err_with(|| format!("failed to decode {}", path.display()))
}

pub(crate) fn write_bincode_atomic<T: Serialize + ?Sized>(path: &Path, value: &T) -> Result<()> {
    let parent = path
        .parent()
        .wrap_err_with(|| format!("cache path has no parent: {}", path.display()))?;
    fs::create_dir_all(parent)
        .wrap_err_with(|| format!("failed to create {}", parent.display()))?;
    let mut temporary = NamedTempFile::new_in(parent)
        .wrap_err_with(|| format!("failed to create temporary file in {}", parent.display()))?;
    {
        let mut writer = BufWriter::new(temporary.as_file_mut());
        bincode::DefaultOptions::new()
            .with_fixint_encoding()
            .serialize_into(&mut writer, value)
            .wrap_err_with(|| format!("failed to encode {}", path.display()))?;
        writer
            .flush()
            .wrap_err_with(|| format!("failed to flush {}", path.display()))?;
    }
    temporary
        .as_file_mut()
        .sync_all()
        .wrap_err_with(|| format!("failed to sync {}", path.display()))?;
    temporary
        .persist(path)
        .map_err(|error| error.error)
        .wrap_err_with(|| format!("failed to atomically replace {}", path.display()))?;
    sync_directory(parent)?;
    Ok(())
}

fn sync_directory(path: &Path) -> Result<()> {
    File::open(path)
        .and_then(|directory| directory.sync_all())
        .wrap_err_with(|| format!("failed to sync cache directory {}", path.display()))
}

fn read_run_ui_metadata(
    path: &Path,
    recording_fingerprint: &RecordingFingerprint,
) -> Result<BTreeMap<String, RunUiMetadata>> {
    if !path.exists() {
        return Ok(BTreeMap::new());
    }
    let metadata: RunUiMetadataFile = read_bincode(path)?;
    if metadata.cache_version != CACHE_VERSION
        || &metadata.recording_fingerprint != recording_fingerprint
    {
        bail!("run UI metadata does not match the recording cache");
    }
    for key in metadata.runs.keys() {
        validate_run_key(key)?;
    }
    Ok(metadata.runs)
}

fn write_run_ui_metadata(
    directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    runs: &BTreeMap<String, RunUiMetadata>,
) -> Result<()> {
    for key in runs.keys() {
        validate_run_key(key)?;
    }
    write_bincode_atomic(
        &directory.join(RUN_METADATA_FILE),
        &RunUiMetadataFile {
            cache_version: CACHE_VERSION,
            recording_fingerprint: recording_fingerprint.clone(),
            runs: runs.clone(),
        },
    )
}

fn update_run_ui_metadata(
    cache_directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    update: impl FnOnce(&mut BTreeMap<String, RunUiMetadata>),
) -> Result<BTreeMap<String, RunUiMetadata>> {
    let directory = recording_cache_directory(cache_directory, recording_fingerprint)?;
    fs::create_dir_all(&directory)
        .wrap_err_with(|| format!("failed to create {}", directory.display()))?;
    let _lock = lock_file(&directory.join(RUN_METADATA_LOCK_FILE), false)?;
    let mut runs = read_run_ui_metadata(&directory.join(RUN_METADATA_FILE), recording_fingerprint)?;
    update(&mut runs);
    write_run_ui_metadata(&directory, recording_fingerprint, &runs)?;
    Ok(runs)
}

fn lock_file(path: &Path, nonblocking: bool) -> Result<File> {
    let file = OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(path)
        .wrap_err_with(|| format!("failed to open {}", path.display()))?;
    if nonblocking {
        file.try_lock()
            .wrap_err_with(|| format!("failed to lock {}", path.display()))?;
    } else {
        file.lock()
            .wrap_err_with(|| format!("failed to lock {}", path.display()))?;
    }
    Ok(file)
}

fn model_run_key(
    canonical_model_path: &Path,
    model_hash: &[u8; 32],
    recording_fingerprint: &RecordingFingerprint,
    thresholds: DetectionThresholds,
    frame_start: usize,
    frame_end: usize,
    total_frame_count: usize,
) -> Result<String> {
    validate_frame_range(frame_start, frame_end, total_frame_count)?;
    let identity = bincode::serialize(&(
        canonical_model_path,
        model_hash,
        recording_fingerprint,
        thresholds,
        CACHE_VERSION,
        frame_start,
        frame_end,
        total_frame_count,
    ))
    .wrap_err("failed to serialize model run identity")?;
    let digest = blake3::hash(&identity).to_hex().to_string();
    Ok(format!("model-{}", &digest[..16]))
}

fn validate_same_run(existing: &ModelRunManifest, proposed: &ModelRunManifest) -> Result<()> {
    if existing.run_key != proposed.run_key
        || existing.canonical_model_path != proposed.canonical_model_path
        || existing.model_hash != proposed.model_hash
        || existing.recording_fingerprint != proposed.recording_fingerprint
        || existing.thresholds != proposed.thresholds
        || existing.total_frame_count != proposed.total_frame_count
        || existing.frame_start != proposed.frame_start
        || existing.frame_end != proposed.frame_end
        || existing.cache_version != CACHE_VERSION
    {
        bail!("existing model run manifest does not match the requested run");
    }
    Ok(())
}

fn validate_model_run_manifest(manifest: &ModelRunManifest) -> Result<()> {
    let target_count = manifest.target_frame_count()?;
    if manifest.cache_version != CACHE_VERSION
        || manifest.completed_frame_count > target_count
        || manifest.thresholds.validate()? != manifest.thresholds
    {
        bail!("model run manifest contains invalid values");
    }
    match (manifest.state, manifest.error.as_deref()) {
        (ModelRunState::Failed, Some(error)) if !error.is_empty() => Ok(()),
        (ModelRunState::Failed, _) => bail!("failed model run manifest has no error"),
        (_, Some(_)) => bail!("non-failed model run manifest contains a stale error"),
        (_, None) => Ok(()),
    }
}

fn validate_frame_range(
    frame_start: usize,
    frame_end: usize,
    total_frame_count: usize,
) -> Result<()> {
    if total_frame_count == 0 {
        bail!("model run recording contains no frames");
    }
    if frame_start > frame_end || frame_end >= total_frame_count {
        bail!(
            "invalid model run frame range {frame_start}..={frame_end} for {total_frame_count} frames"
        );
    }
    Ok(())
}

fn validate_run_key(key: &str) -> Result<()> {
    if key == "recorded" {
        return Ok(());
    }
    validate_model_run_key(key)
}

fn validate_model_run_key(key: &str) -> Result<()> {
    let Some(hash) = key.strip_prefix("model-") else {
        bail!("invalid model run key `{key}`");
    };
    if hash.len() != 16 || !hash.bytes().all(|byte| byte.is_ascii_hexdigit()) {
        bail!("invalid model run key `{key}`");
    }
    Ok(())
}

fn lock_model_run(root: &Path, run_key: &str, nonblocking: bool) -> Result<File> {
    validate_model_run_key(run_key)?;
    let lock_directory = root.join(MODEL_RUN_LOCKS_DIRECTORY);
    fs::create_dir_all(&lock_directory)
        .wrap_err_with(|| format!("failed to create {}", lock_directory.display()))?;
    let path = lock_directory.join(format!("{run_key}.lock"));
    let file = OpenOptions::new()
        .create(true)
        .truncate(false)
        .read(true)
        .write(true)
        .open(&path)
        .wrap_err_with(|| format!("failed to open {}", path.display()))?;
    if nonblocking {
        file.try_lock()
            .wrap_err_with(|| format!("model run `{run_key}` is currently in use"))?;
    } else {
        file.lock()
            .wrap_err_with(|| format!("failed to lock model run `{run_key}`"))?;
    }
    Ok(file)
}

fn normalize_zero(value: f32) -> f32 {
    if value == 0.0 { 0.0 } else { value }
}

pub(crate) fn validate_baseline(
    directory: &Path,
    baseline: &RecordedBaseline,
    fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<()> {
    if baseline.cache_version != CACHE_VERSION
        || &baseline.recording_fingerprint != fingerprint
        || baseline.total_frame_count != total_frame_count
    {
        bail!("recorded baseline does not match the recording cache");
    }
    let mut previous_end = None;
    for (start, end) in &baseline.availability.ranges {
        if start > end
            || *end >= total_frame_count
            || previous_end.is_some_and(|previous| previous >= *start)
        {
            bail!("recorded baseline contains an invalid availability range");
        }
        previous_end = Some(*end);
    }
    let chunk_count = total_frame_count.div_ceil(PREDICTION_CHUNK_SIZE);
    for index in 0..chunk_count {
        let path = baseline_chunk_path(directory, index);
        let length = path
            .metadata()
            .wrap_err_with(|| format!("failed to inspect {}", path.display()))?
            .len();
        if length == 0 || length > MAX_CACHE_FILE_BYTES {
            bail!(
                "recorded baseline chunk {} has an invalid size",
                path.display()
            );
        }
    }
    Ok(())
}

fn inspect_model_chunks(directory: &Path, manifest: &ModelRunManifest) -> Result<()> {
    if manifest.completed_frame_count > manifest.target_frame_count()? {
        bail!("model run completed count exceeds its frame range");
    }
    let expected_chunks = manifest
        .completed_frame_count
        .div_ceil(PREDICTION_CHUNK_SIZE);
    let mut chunk_paths = BTreeMap::new();
    for entry in fs::read_dir(directory)
        .wrap_err_with(|| format!("failed to read {}", directory.display()))?
    {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        let Some(index) = name
            .strip_prefix("chunk-")
            .and_then(|name| name.strip_suffix(".bin"))
            .and_then(|index| index.parse::<usize>().ok())
        else {
            continue;
        };
        chunk_paths.insert(index, entry.path());
    }

    if chunk_paths.len() != expected_chunks {
        bail!(
            "prediction cache has {} chunks, expected {expected_chunks}",
            chunk_paths.len()
        );
    }
    for (position, (chunk_index, path)) in chunk_paths.into_iter().enumerate() {
        if chunk_index != position || !path.is_file() {
            bail!(
                "prediction chunks are not contiguous in {}",
                directory.display()
            );
        }
        let length = path
            .metadata()
            .wrap_err_with(|| format!("failed to inspect {}", path.display()))?
            .len();
        if length == 0 || length > MAX_CACHE_FILE_BYTES {
            bail!(
                "prediction chunk {} has an invalid file size",
                path.display()
            );
        }
    }
    Ok(())
}

fn load_contiguous_predictions(
    directory: &Path,
    manifest: &ModelRunManifest,
) -> Result<LoadedContiguousPredictions> {
    let mut chunk_paths = BTreeMap::new();
    for entry in fs::read_dir(directory)
        .wrap_err_with(|| format!("failed to read {}", directory.display()))?
    {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        let Some(index) = name
            .strip_prefix("chunk-")
            .and_then(|name| name.strip_suffix(".bin"))
            .and_then(|index| index.parse::<usize>().ok())
        else {
            continue;
        };
        chunk_paths.insert(index, entry.path());
    }

    let mut completed_count = 0_usize;
    let mut pending = Vec::new();
    let chunk_count = chunk_paths.len();
    for (position, (chunk_index, path)) in chunk_paths.into_iter().enumerate() {
        if chunk_index != position {
            bail!(
                "prediction chunks are not contiguous in {}",
                directory.display()
            );
        }
        let chunk: PredictionChunk = read_bincode(&path)?;
        let relative_start = chunk_index
            .checked_mul(PREDICTION_CHUNK_SIZE)
            .wrap_err("prediction chunk index overflow")?;
        let expected_start = manifest
            .frame_start
            .checked_add(relative_start)
            .wrap_err("prediction chunk start overflow")?;
        validate_model_chunk(&path, &chunk, expected_start, position + 1 == chunk_count)?;
        if position + 1 < chunk_count && chunk.predictions.len() != PREDICTION_CHUNK_SIZE {
            bail!(
                "non-final prediction chunk {} is incomplete",
                path.display()
            );
        }
        completed_count = completed_count
            .checked_add(chunk.predictions.len())
            .wrap_err("prediction count overflow")?;
        if completed_count > manifest.target_frame_count()? {
            bail!("prediction cache contains more frames than its manifest range");
        }
        if position + 1 == chunk_count && chunk.predictions.len() < PREDICTION_CHUNK_SIZE {
            pending = chunk.predictions;
        }
    }
    Ok(LoadedContiguousPredictions {
        completed_count,
        pending,
    })
}

fn validate_model_chunk(
    path: &Path,
    chunk: &PredictionChunk,
    expected_start: usize,
    is_final: bool,
) -> Result<()> {
    if chunk.start_frame != expected_start {
        bail!(
            "prediction chunk {} has an invalid start frame",
            path.display()
        );
    }
    if chunk.predictions.is_empty() || chunk.predictions.len() > PREDICTION_CHUNK_SIZE {
        bail!("prediction chunk {} has an invalid length", path.display());
    }
    if !is_final && chunk.predictions.len() != PREDICTION_CHUNK_SIZE {
        bail!(
            "non-final prediction chunk {} is incomplete",
            path.display()
        );
    }
    for (offset, prediction) in chunk.predictions.iter().enumerate() {
        let expected_frame = expected_start
            .checked_add(offset)
            .wrap_err("prediction frame index overflow")?;
        if prediction.frame_index != expected_frame {
            bail!("prediction chunk {} contains a frame gap", path.display());
        }
        validate_prediction(prediction)?;
    }
    Ok(())
}

fn validate_prediction(prediction: &Prediction) -> Result<()> {
    if prediction.objects.len() > MAX_PREDICTIONS_PER_FRAME
        || prediction
            .poses
            .as_ref()
            .is_some_and(|poses| poses.len() > MAX_PREDICTIONS_PER_FRAME)
    {
        bail!("prediction exceeds the model output capacity");
    }
    Ok(())
}

fn load_prediction_chunk(
    path: &Path,
    chunk_index: usize,
    storage: &PredictionStorage,
    availability: &PredictionAvailability,
) -> Result<CachedPredictionChunk> {
    match storage {
        PredictionStorage::RecordedBaseline {
            total_frame_count, ..
        } => {
            let chunk: RecordedBaselineChunk = read_bincode(path)?;
            let expected_start = chunk_index
                .checked_mul(PREDICTION_CHUNK_SIZE)
                .wrap_err("baseline chunk start overflow")?;
            let expected_length = (*total_frame_count - expected_start).min(PREDICTION_CHUNK_SIZE);
            if chunk.start_frame != expected_start || chunk.predictions.len() != expected_length {
                bail!("recorded baseline chunk {} is invalid", path.display());
            }
            for (offset, prediction) in chunk.predictions.iter().enumerate() {
                let expected_frame = expected_start
                    .checked_add(offset)
                    .wrap_err("baseline frame index overflow")?;
                if prediction.is_some() != availability.contains(expected_frame) {
                    bail!(
                        "recorded baseline chunk {} disagrees with its availability index",
                        path.display()
                    );
                }
                if let Some(prediction) = prediction {
                    if prediction.frame_index != expected_frame {
                        bail!(
                            "recorded baseline chunk {} has an invalid frame",
                            path.display()
                        );
                    }
                    validate_prediction(prediction)?;
                }
            }
            Ok(CachedPredictionChunk {
                index: chunk_index,
                start_frame: chunk.start_frame,
                predictions: chunk
                    .predictions
                    .into_iter()
                    .map(|prediction| prediction.map(Arc::new))
                    .collect(),
            })
        }
        PredictionStorage::Model { frame_start, .. } => {
            let chunk: PredictionChunk = read_bincode(path)?;
            let expected_start = frame_start
                .checked_add(
                    chunk_index
                        .checked_mul(PREDICTION_CHUNK_SIZE)
                        .wrap_err("prediction chunk start overflow")?,
                )
                .wrap_err("prediction chunk frame overflow")?;
            validate_model_chunk(path, &chunk, expected_start, true)?;
            Ok(CachedPredictionChunk {
                index: chunk_index,
                start_frame: chunk.start_frame,
                predictions: chunk
                    .predictions
                    .into_iter()
                    .map(|prediction| Some(Arc::new(prediction)))
                    .collect(),
            })
        }
    }
}

fn model_run_is_active(directory: &Path, run_key: &str) -> Result<bool> {
    let root = directory
        .parent()
        .wrap_err("model run directory has no parent")?;
    let path = root
        .join(MODEL_RUN_LOCKS_DIRECTORY)
        .join(format!("{run_key}.lock"));
    let file = match OpenOptions::new().read(true).write(true).open(&path) {
        Ok(file) => file,
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Ok(false),
        Err(error) => {
            return Err(error).wrap_err_with(|| format!("failed to open {}", path.display()));
        }
    };
    match file.try_lock() {
        Ok(()) => Ok(false),
        Err(std::fs::TryLockError::WouldBlock) => Ok(true),
        Err(std::fs::TryLockError::Error(error)) => {
            Err(error).wrap_err_with(|| format!("failed to inspect lock {}", path.display()))
        }
    }
}

fn chunk_path(directory: &Path, chunk_index: usize) -> PathBuf {
    directory.join(format!("chunk-{chunk_index:08}.bin"))
}

fn baseline_chunk_path(directory: &Path, chunk_index: usize) -> PathBuf {
    directory.join(format!("chunk-{chunk_index:08}.bin"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fingerprint() -> RecordingFingerprint {
        RecordingFingerprint {
            canonical_path: PathBuf::from("/tmp/fake-recording.mcap"),
            size: 123,
            modified_unix_nanos: 456,
            format_version: CACHE_VERSION,
        }
    }

    fn prediction(frame_index: usize) -> Prediction {
        Prediction {
            frame_index,
            timestamp_nanos: frame_index as i64 * 10,
            objects: Vec::new(),
            poses: Some(Vec::new()),
            inference_duration_nanos: Some(1),
            postprocessing_duration_nanos: Some(2),
            non_maximum_suppression_duration_nanos: Some(3),
        }
    }

    fn manifest(total: usize) -> ModelRunManifest {
        ModelRunManifest::new(
            "../unsafe label",
            PathBuf::from("/tmp/model.onnx"),
            [7; 32],
            fingerprint(),
            DetectionThresholds::default(),
            total,
            0..=total - 1,
        )
        .unwrap()
    }

    #[test]
    fn chunks_resume_and_distinguish_empty_from_absent() {
        let cache = tempfile::tempdir().unwrap();
        let mut store = PredictionStore::open(cache.path(), manifest(130)).unwrap();
        for frame_index in 0..129 {
            store.append(prediction(frame_index)).unwrap();
        }
        drop(store);

        let mut resumed = PredictionStore::open(cache.path(), manifest(130)).unwrap();
        assert_eq!(resumed.completed_count(), 128);
        resumed.append(prediction(128)).unwrap();
        resumed.finish().unwrap();
        drop(resumed);
        let resumed = PredictionStore::open(cache.path(), manifest(130)).unwrap();
        assert_eq!(resumed.completed_count(), 129);
        let run_directory = cache
            .path()
            .join(MODEL_RUNS_DIRECTORY)
            .join(&resumed.manifest().run_key);
        assert!(chunk_path(&run_directory, 0).exists());
        assert!(chunk_path(&run_directory, 1).exists());

        let loaded = load_all_runs(cache.path(), &fingerprint(), 130).unwrap();
        let model = loaded
            .iter()
            .find(|run| run.source == PredictionSource::Model)
            .unwrap();
        assert!(model.prediction(0).unwrap().unwrap().objects.is_empty());
        assert!(model.prediction(128).unwrap().is_some());
        assert!(model.prediction(129).unwrap().is_none());
    }

    #[test]
    fn labels_cannot_escape_the_cache_directory() {
        let key = manifest(1).run_key;
        assert!(!key.contains('/'));
        assert!(!key.contains(".."));
    }

    #[test]
    fn nonzero_frame_range_is_stored_in_global_slots() {
        let cache = tempfile::tempdir().unwrap();
        let proposed = ModelRunManifest::new(
            "range",
            PathBuf::from("/tmp/model.onnx"),
            [7; 32],
            fingerprint(),
            DetectionThresholds::default(),
            100,
            40..=42,
        )
        .unwrap();
        let mut store = PredictionStore::open(cache.path(), proposed).unwrap();
        for frame_index in 40..=42 {
            store.append(prediction(frame_index)).unwrap();
        }
        store.finish().unwrap();
        drop(store);

        let loaded = load_all_runs(cache.path(), &fingerprint(), 100).unwrap();
        let model = loaded
            .iter()
            .find(|run| run.source == PredictionSource::Model)
            .unwrap();
        assert!(model.prediction(39).unwrap().is_none());
        assert!(model.prediction(40).unwrap().is_some());
        assert!(model.prediction(42).unwrap().is_some());
        assert!(model.prediction(43).unwrap().is_none());
        assert_eq!(
            model.manifest.as_ref().unwrap().state,
            ModelRunState::Complete
        );
    }

    #[test]
    fn run_ui_metadata_round_trips() {
        let cache = tempfile::tempdir().unwrap();
        let metadata = BTreeMap::from([
            (
                "recorded".to_string(),
                RunUiMetadata {
                    renamed_label: None,
                    hidden: true,
                },
            ),
            (
                "model-0123456789abcdef".to_string(),
                RunUiMetadata {
                    renamed_label: Some("Renamed".to_string()),
                    hidden: false,
                },
            ),
        ]);
        save_run_ui_metadata(cache.path(), &fingerprint(), &metadata).unwrap();
        assert_eq!(
            load_run_ui_metadata(cache.path(), &fingerprint()).unwrap(),
            metadata
        );
    }

    #[test]
    fn deleting_model_run_validates_and_detaches_cache() {
        let cache = tempfile::tempdir().unwrap();
        let proposed = manifest(1);
        let key = proposed.run_key.clone();
        let run_directory = cache.path().join(MODEL_RUNS_DIRECTORY).join(&key);
        let store = PredictionStore::open(cache.path(), proposed).unwrap();
        drop(store);

        delete_model_run(cache.path(), &fingerprint(), &key).unwrap();
        assert!(!run_directory.exists());

        let active = PredictionStore::open(cache.path(), manifest(1)).unwrap();
        let error = delete_model_run(cache.path(), &fingerprint(), &key).unwrap_err();
        assert!(error.to_string().contains("currently in use"));
        drop(active);

        assert!(delete_model_run(cache.path(), &fingerprint(), "recorded").is_err());
        assert!(delete_model_run(cache.path(), &fingerprint(), "../escape").is_err());
    }

    #[test]
    fn manifest_persists_range_and_range_changes_identity() {
        let first = ModelRunManifest::new(
            "range",
            PathBuf::from("/tmp/model.onnx"),
            [7; 32],
            fingerprint(),
            DetectionThresholds::default(),
            100,
            10..=20,
        )
        .unwrap();
        let second = ModelRunManifest::new(
            "range",
            PathBuf::from("/tmp/model.onnx"),
            [7; 32],
            fingerprint(),
            DetectionThresholds::default(),
            100,
            11..=20,
        )
        .unwrap();
        assert_eq!((first.frame_start, first.frame_end), (10, 20));
        assert_eq!(first.target_frame_count().unwrap(), 11);
        assert_ne!(first.run_key, second.run_key);
    }

    #[test]
    fn corrupt_chunk_frame_is_reported_without_panicking() {
        let cache = tempfile::tempdir().unwrap();
        let proposed = manifest(1);
        let key = proposed.run_key.clone();
        let mut store = PredictionStore::open(cache.path(), proposed).unwrap();
        store.append(prediction(0)).unwrap();
        store.finish().unwrap();
        drop(store);

        let directory = cache.path().join(MODEL_RUNS_DIRECTORY).join(key);
        write_bincode_atomic(
            &chunk_path(&directory, 0),
            &PredictionChunk {
                start_frame: 0,
                predictions: vec![prediction(usize::MAX)],
            },
        )
        .unwrap();
        let runs = load_all_runs(cache.path(), &fingerprint(), 1).unwrap();
        let run = runs
            .iter()
            .find(|run| run.source == PredictionSource::Model)
            .unwrap();
        assert!(run.prediction(0).is_err());
        assert!(run.prediction(0).is_err());
    }

    #[test]
    fn bounded_decoder_rejects_trailing_bytes() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("value.bin");
        write_bincode_atomic(&path, &42_u32).unwrap();
        OpenOptions::new()
            .append(true)
            .open(&path)
            .unwrap()
            .write_all(&[0])
            .unwrap();
        assert!(read_bincode::<u32>(&path).is_err());
    }

    #[test]
    fn stale_running_manifest_loads_as_incomplete() {
        let cache = tempfile::tempdir().unwrap();
        let store = PredictionStore::open(cache.path(), manifest(1)).unwrap();
        drop(store);
        let runs = load_all_runs(cache.path(), &fingerprint(), 1).unwrap();
        let run = runs
            .iter()
            .find(|run| run.source == PredictionSource::Model)
            .unwrap();
        assert_eq!(
            run.manifest.as_ref().unwrap().state,
            ModelRunState::Incomplete
        );
    }

    #[test]
    fn bookmarks_round_trip() {
        let cache = tempfile::tempdir().unwrap();
        let bookmarks = BookmarkCollection(BTreeMap::from([(
            12,
            Bookmark {
                name: "#1".to_string(),
            },
        )]));
        save_bookmarks(cache.path(), &fingerprint(), &bookmarks).unwrap();
        assert_eq!(
            load_bookmarks(cache.path(), &fingerprint()).unwrap(),
            bookmarks
        );
    }

    #[test]
    fn legacy_model_run_and_gui_metadata_are_migrated() {
        let cache = tempfile::tempdir().unwrap();
        let fingerprint = fingerprint();
        let thresholds = DetectionThresholds::default();
        let old_key = legacy_model_run_key(
            Path::new("/tmp/model.onnx"),
            &[7; 32],
            &fingerprint,
            thresholds,
            None,
        )
        .unwrap();
        let old_directory = cache.path().join(MODEL_RUNS_DIRECTORY).join(&old_key);
        fs::create_dir_all(&old_directory).unwrap();
        write_bincode_atomic(
            &old_directory.join(MANIFEST_FILE),
            &LegacyModelRunManifest {
                run_key: old_key.clone(),
                label: "legacy".to_string(),
                canonical_model_path: PathBuf::from("/tmp/model.onnx"),
                model_hash: [7; 32],
                recording_fingerprint: fingerprint.clone(),
                thresholds,
                total_frame_count: 1,
                completed_frame_count: 1,
                cache_version: 1,
                state: ModelRunState::Complete,
                error: None,
                provider_note: Some("legacy provider".to_string()),
            },
        )
        .unwrap();
        write_bincode_atomic(
            &chunk_path(&old_directory, 0),
            &PredictionChunk {
                start_frame: 0,
                predictions: vec![prediction(0)],
            },
        )
        .unwrap();
        let metadata_directory = recording_cache_directory(cache.path(), &fingerprint).unwrap();
        fs::create_dir_all(&metadata_directory).unwrap();
        write_bincode_atomic(
            &metadata_directory.join(RUN_METADATA_FILE),
            &RunUiMetadataFile {
                cache_version: 1,
                recording_fingerprint: fingerprint.clone(),
                runs: BTreeMap::from([(
                    old_key.clone(),
                    RunUiMetadata {
                        renamed_label: Some("renamed".to_string()),
                        hidden: true,
                    },
                )]),
            },
        )
        .unwrap();

        let runs = load_all_runs(cache.path(), &fingerprint, 1).unwrap();
        let run = runs
            .iter()
            .find(|run| run.source == PredictionSource::Model)
            .unwrap();
        assert_ne!(run.key, old_key);
        assert!(run.prediction(0).unwrap().is_some());
        assert!(!old_directory.exists());
        let metadata = load_run_ui_metadata(cache.path(), &fingerprint).unwrap();
        assert_eq!(
            metadata.get(&run.key).unwrap().renamed_label.as_deref(),
            Some("renamed")
        );
        assert!(metadata.get(&run.key).unwrap().hidden);
    }

    #[test]
    fn invalid_legacy_run_does_not_hide_other_runs() {
        let cache = tempfile::tempdir().unwrap();
        let fingerprint = fingerprint();
        let thresholds = DetectionThresholds::default();
        let old_key = legacy_model_run_key(
            Path::new("/tmp/model.onnx"),
            &[7; 32],
            &fingerprint,
            thresholds,
            Some((5, 6)),
        )
        .unwrap();
        let old_directory = cache.path().join(MODEL_RUNS_DIRECTORY).join(&old_key);
        fs::create_dir_all(&old_directory).unwrap();
        write_bincode_atomic(
            &old_directory.join(MANIFEST_FILE),
            &LegacyModelRunManifest {
                run_key: old_key,
                label: "empty range".to_string(),
                canonical_model_path: PathBuf::from("/tmp/model.onnx"),
                model_hash: [7; 32],
                recording_fingerprint: fingerprint.clone(),
                thresholds,
                total_frame_count: 10,
                completed_frame_count: 0,
                cache_version: 1,
                state: ModelRunState::Incomplete,
                error: None,
                provider_note: None,
            },
        )
        .unwrap();

        assert!(load_all_runs(cache.path(), &fingerprint, 10).is_ok());
        assert!(old_directory.exists());
    }
}
