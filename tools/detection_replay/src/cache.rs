use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufReader, BufWriter, Write},
    path::{Path, PathBuf},
    sync::Arc,
    time::UNIX_EPOCH,
};

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

pub const CACHE_VERSION: u32 = 1;
pub const PREDICTION_CHUNK_SIZE: usize = 128;

const MODEL_RUNS_DIRECTORY: &str = "model-runs";
const RECORDINGS_DIRECTORY: &str = "recordings";
const MANIFEST_FILE: &str = "manifest.bin";
const RECORDED_BASELINE_FILE: &str = "recorded-baseline.bin";

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
            format_version: CACHE_VERSION,
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
    ) -> Result<Self> {
        let label = label.into();
        let thresholds = thresholds.validate()?;
        let run_key = model_run_key(
            &canonical_model_path,
            &model_hash,
            &recording_fingerprint,
            thresholds,
        )?;
        Ok(Self {
            run_key,
            label,
            canonical_model_path,
            model_hash,
            recording_fingerprint,
            thresholds,
            total_frame_count,
            completed_frame_count: 0,
            cache_version: CACHE_VERSION,
            state: ModelRunState::Running,
            error: None,
            provider_note: None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PredictionSource {
    Model,
    RecordedBaseline,
}

#[derive(Clone, Debug)]
pub struct LoadedPredictionRun {
    pub key: String,
    pub label: String,
    pub source: PredictionSource,
    pub manifest: Option<ModelRunManifest>,
    /// A present empty prediction means inference ran and found nothing. `None` means absent.
    pub predictions: Vec<Option<Arc<Prediction>>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RecordedBaseline {
    pub cache_version: u32,
    pub recording_fingerprint: RecordingFingerprint,
    pub total_frame_count: usize,
    pub predictions: Vec<Option<Prediction>>,
}

#[derive(Debug, Serialize, Deserialize)]
struct PredictionChunk {
    start_frame: usize,
    predictions: Vec<Prediction>,
}

pub struct PredictionStore {
    run_directory: PathBuf,
    manifest: ModelRunManifest,
    predictions: Vec<Prediction>,
}

impl PredictionStore {
    pub fn open(cache_directory: impl AsRef<Path>, proposed: ModelRunManifest) -> Result<Self> {
        let root = cache_directory.as_ref().join(MODEL_RUNS_DIRECTORY);
        fs::create_dir_all(&root)
            .wrap_err_with(|| format!("failed to create {}", root.display()))?;
        let run_directory = root.join(&proposed.run_key);
        fs::create_dir_all(&run_directory)
            .wrap_err_with(|| format!("failed to create {}", run_directory.display()))?;
        let manifest_path = run_directory.join(MANIFEST_FILE);

        let mut manifest = if manifest_path.exists() {
            let existing: ModelRunManifest = read_bincode(&manifest_path)?;
            validate_same_run(&existing, &proposed)?;
            ModelRunManifest {
                label: proposed.label.clone(),
                ..existing
            }
        } else {
            write_bincode_atomic(&manifest_path, &proposed)?;
            proposed
        };

        let predictions = load_contiguous_predictions(&run_directory, manifest.total_frame_count)?;
        if manifest.completed_frame_count != predictions.len() {
            manifest.completed_frame_count = predictions.len();
            write_bincode_atomic(&manifest_path, &manifest)?;
        }
        if manifest.state == ModelRunState::Complete
            && predictions.len() != manifest.total_frame_count
        {
            manifest.state = ModelRunState::Incomplete;
            write_bincode_atomic(&manifest_path, &manifest)?;
        }

        Ok(Self {
            run_directory,
            manifest,
            predictions,
        })
    }

    pub fn manifest(&self) -> &ModelRunManifest {
        &self.manifest
    }

    pub fn completed_count(&self) -> usize {
        self.predictions.len()
    }

    pub fn predictions(&self) -> &[Prediction] {
        &self.predictions
    }

    pub fn begin(&mut self, provider_note: impl Into<String>) -> Result<()> {
        self.manifest.state = ModelRunState::Running;
        self.manifest.error = None;
        self.manifest.provider_note = Some(provider_note.into());
        self.persist_manifest()
    }

    pub fn append(&mut self, prediction: Prediction) -> Result<()> {
        let expected_frame = self.predictions.len();
        if prediction.frame_index != expected_frame {
            bail!(
                "prediction frame {} is not contiguous; expected {}",
                prediction.frame_index,
                expected_frame
            );
        }
        if expected_frame >= self.manifest.total_frame_count {
            bail!("prediction exceeds the manifest frame count");
        }

        self.predictions.push(prediction);
        if self.predictions.len().is_multiple_of(PREDICTION_CHUNK_SIZE)
            && let Err(error) = self.persist_pending()
        {
            return Err(error);
        }
        Ok(())
    }

    pub fn finish(&mut self) -> Result<()> {
        self.persist_pending()?;
        self.manifest.state = if self.predictions.len() == self.manifest.total_frame_count {
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
        if self.manifest.completed_frame_count == self.predictions.len() {
            return Ok(());
        }
        let chunk_index = (self.predictions.len() - 1) / PREDICTION_CHUNK_SIZE;
        let chunk_start = chunk_index * PREDICTION_CHUNK_SIZE;
        let chunk = PredictionChunkRef {
            start_frame: chunk_start,
            predictions: &self.predictions[chunk_start..],
        };
        write_bincode_atomic(&chunk_path(&self.run_directory, chunk_index), &chunk)?;
        self.manifest.completed_frame_count = self.predictions.len();
        self.persist_manifest()
    }
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
    baseline: &RecordedBaseline,
) -> Result<()> {
    let directory =
        recording_cache_directory(cache_directory.as_ref(), &baseline.recording_fingerprint)?;
    fs::create_dir_all(&directory)
        .wrap_err_with(|| format!("failed to create {}", directory.display()))?;
    write_bincode_atomic(&directory.join(RECORDED_BASELINE_FILE), baseline)
}

pub fn load_all_runs(
    cache_directory: impl AsRef<Path>,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<Vec<LoadedPredictionRun>> {
    let cache_directory = cache_directory.as_ref();
    let mut runs = Vec::new();
    let baseline_path = recording_cache_directory(cache_directory, recording_fingerprint)?
        .join(RECORDED_BASELINE_FILE);
    if baseline_path.exists() {
        let baseline: RecordedBaseline = read_bincode(&baseline_path)?;
        validate_baseline(&baseline, recording_fingerprint, total_frame_count)?;
        runs.push(LoadedPredictionRun {
            key: "recorded".to_string(),
            label: "Recorded".to_string(),
            source: PredictionSource::RecordedBaseline,
            manifest: None,
            predictions: baseline
                .predictions
                .into_iter()
                .map(|prediction| prediction.map(Arc::new))
                .collect(),
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
        if !directory.is_dir() || !directory.join(MANIFEST_FILE).exists() {
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

fn load_model_run(
    directory: &Path,
    recording_fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<Option<LoadedPredictionRun>> {
    let mut manifest: ModelRunManifest = read_bincode(&directory.join(MANIFEST_FILE))?;
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
    let predictions = load_contiguous_predictions(directory, total_frame_count)?;
    manifest.completed_frame_count = predictions.len();
    if manifest.state == ModelRunState::Complete && predictions.len() != total_frame_count {
        manifest.state = ModelRunState::Incomplete;
    }
    let mut sparse = vec![None; total_frame_count];
    for prediction in predictions {
        let frame_index = prediction.frame_index;
        sparse[frame_index] = Some(Arc::new(prediction));
    }
    Ok(Some(LoadedPredictionRun {
        key: manifest.run_key.clone(),
        label: manifest.label.clone(),
        source: PredictionSource::Model,
        manifest: Some(manifest),
        predictions: sparse,
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
    Ok(recording_cache_directory(cache_directory, fingerprint)?.join(RECORDED_BASELINE_FILE))
}

pub(crate) fn read_bincode<T: DeserializeOwned>(path: &Path) -> Result<T> {
    let file = File::open(path).wrap_err_with(|| format!("failed to open {}", path.display()))?;
    bincode::deserialize_from(BufReader::new(file))
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
        bincode::serialize_into(&mut writer, value)
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
    Ok(())
}

fn model_run_key(
    canonical_model_path: &Path,
    model_hash: &[u8; 32],
    recording_fingerprint: &RecordingFingerprint,
    thresholds: DetectionThresholds,
) -> Result<String> {
    let identity = bincode::serialize(&(
        canonical_model_path,
        model_hash,
        recording_fingerprint,
        thresholds,
        CACHE_VERSION,
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
        || existing.cache_version != CACHE_VERSION
    {
        bail!("existing model run manifest does not match the requested run");
    }
    Ok(())
}

fn normalize_zero(value: f32) -> f32 {
    if value == 0.0 { 0.0 } else { value }
}

fn validate_baseline(
    baseline: &RecordedBaseline,
    fingerprint: &RecordingFingerprint,
    total_frame_count: usize,
) -> Result<()> {
    if baseline.cache_version != CACHE_VERSION
        || &baseline.recording_fingerprint != fingerprint
        || baseline.total_frame_count != total_frame_count
        || baseline.predictions.len() != total_frame_count
    {
        bail!("recorded baseline does not match the recording cache");
    }
    for (frame_index, prediction) in baseline.predictions.iter().enumerate() {
        if prediction
            .as_ref()
            .is_some_and(|prediction| prediction.frame_index != frame_index)
        {
            bail!("recorded baseline frame index is not contiguous with its sparse slot");
        }
    }
    Ok(())
}

fn load_contiguous_predictions(directory: &Path, total_frames: usize) -> Result<Vec<Prediction>> {
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

    let mut predictions = Vec::new();
    let chunk_count = chunk_paths.len();
    for (position, (chunk_index, path)) in chunk_paths.into_iter().enumerate() {
        if chunk_index != position {
            bail!(
                "prediction chunks are not contiguous in {}",
                directory.display()
            );
        }
        let chunk: PredictionChunk = read_bincode(&path)?;
        let expected_start = chunk_index * PREDICTION_CHUNK_SIZE;
        if chunk.start_frame != expected_start || chunk.start_frame != predictions.len() {
            bail!(
                "prediction chunk {} has an invalid start frame",
                path.display()
            );
        }
        if chunk.predictions.is_empty() || chunk.predictions.len() > PREDICTION_CHUNK_SIZE {
            bail!("prediction chunk {} has an invalid length", path.display());
        }
        if position + 1 < chunk_count && chunk.predictions.len() != PREDICTION_CHUNK_SIZE {
            bail!(
                "non-final prediction chunk {} is incomplete",
                path.display()
            );
        }
        for prediction in chunk.predictions {
            if prediction.frame_index != predictions.len() {
                bail!("prediction chunk {} contains a frame gap", path.display());
            }
            predictions.push(prediction);
        }
    }
    if predictions.len() > total_frames {
        bail!("prediction cache contains more frames than the recording");
    }
    Ok(predictions)
}

fn chunk_path(directory: &Path, chunk_index: usize) -> PathBuf {
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
        assert!(model.predictions[0].as_ref().unwrap().objects.is_empty());
        assert!(model.predictions[128].is_some());
        assert!(model.predictions[129].is_none());
    }

    #[test]
    fn labels_cannot_escape_the_cache_directory() {
        let key = manifest(1).run_key;
        assert!(!key.contains('/'));
        assert!(!key.contains(".."));
    }
}
