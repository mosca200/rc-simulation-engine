#![forbid(unsafe_code)]
//! M2.9G — deterministic promotion of completed M2.9E execution output into an
//! immutable XFOIL evidence bundle.

use std::{
    collections::HashSet,
    fs,
    path::{Path, PathBuf},
};

use model::{
    ConvergenceStatus, MetadataBuilder, XfoilCampaignCoverageRequest, XfoilEvidenceBridgeError,
    XfoilEvidenceCampaignBuilder, XfoilEvidenceCampaignError, XfoilEvidenceDatasetBuilder,
    XfoilPolarImportError, parse_xfoil_polar,
};
use serde::Serialize;
use thiserror::Error;

use crate::xfoil_runner_app::{
    CampaignExecutionStatus, ExecutionReport, ExecutionStatus, ValidationDataset,
    ValidationManifest, polar_filename,
};

const BUNDLE_MANIFEST: &str = "xfoil_evidence_bundle.json";
const POLAR_DATASETS: &str = "polar_datasets.json";
const POLAR_DIRECTORY: &str = "polars";
const EXECUTION_REPORT: &str = "xfoil_execution.json";
const VALIDATION_MANIFEST: &str = "xfoil_validation_manifest.json";
const GENERATED_BY: &str = "rcsim-app xfoil build-evidence-bundle";
const MANIFEST_SCHEMA_VERSION: u32 = 1;
const SUPPORTED_EXECUTION_SCHEMA_VERSION: u32 = 1;
const SUPPORTED_VALIDATION_SCHEMA_VERSION: u32 = 1;
const STAGING_DIRECTORY_PREFIX: &str = ".xfoil-bundle-staging-";

#[derive(Debug, Clone)]
pub struct XfoilEvidenceBundleOptions {
    execution_dir: PathBuf,
    output_dir: PathBuf,
}

impl XfoilEvidenceBundleOptions {
    pub fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut execution_dir = None;
        let mut output_dir = None;
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--execution-dir" => {
                    if execution_dir.is_some() {
                        return Err("--execution-dir may be supplied only once".to_owned());
                    }
                    execution_dir =
                        Some(PathBuf::from(args.next().ok_or_else(|| {
                            "missing value for --execution-dir".to_owned()
                        })?));
                }
                "--output-dir" => {
                    if output_dir.is_some() {
                        return Err("--output-dir may be supplied only once".to_owned());
                    }
                    output_dir =
                        Some(PathBuf::from(args.next().ok_or_else(|| {
                            "missing value for --output-dir".to_owned()
                        })?));
                }
                _ => {
                    return Err(format!(
                        "unknown XFOIL evidence-bundle argument: {argument}"
                    ));
                }
            }
        }
        Ok(Self {
            execution_dir: execution_dir
                .ok_or_else(|| "--execution-dir PATH is required".to_owned())?,
            output_dir: output_dir.ok_or_else(|| "--output-dir PATH is required".to_owned())?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XfoilEvidenceBundleStatus {
    Built,
    NotPromotable,
}

#[derive(Debug, Error)]
pub enum XfoilEvidenceBundleError {
    #[error("failed to read execution report {path:?}: {source}")]
    ExecutionReportRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to deserialize execution report {path:?}: {source}")]
    ExecutionReportDeserialize {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to read validation manifest {path:?}: {source}")]
    ValidationManifestRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to deserialize validation manifest {path:?}: {source}")]
    ValidationManifestDeserialize {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to read polar file {path:?}: {source}")]
    PolarRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to write evidence bundle artifact {path:?}: {source}")]
    WriteArtifact {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to serialize {artifact}: {source}")]
    SerializeArtifact {
        artifact: &'static str,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to remove leftover staging directory {path:?}: {source}")]
    CleanupStaging {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to remove leftover owned bundle artifacts under {path:?}: {source}")]
    CleanupOwned {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to build evidence campaign: {source}")]
    CampaignBuild {
        #[source]
        source: XfoilEvidenceCampaignError,
    },
    #[error("{0}")]
    NotPromotable(NotPromotable),
}

impl From<NotPromotable> for XfoilEvidenceBundleError {
    fn from(reason: NotPromotable) -> Self {
        XfoilEvidenceBundleError::NotPromotable(reason)
    }
}

#[derive(Debug, Error)]
pub(crate) enum NotPromotable {
    #[error(
        "unsupported execution report schema version {found}; supported version is {supported}"
    )]
    UnsupportedExecutionSchemaVersion { found: u32, supported: u32 },
    #[error(
        "unsupported validation manifest schema version {found}; supported version is {supported}"
    )]
    UnsupportedValidationSchemaVersion { found: u32, supported: u32 },
    #[error("execution report contains zero runs")]
    ZeroRuns,
    #[error("execution report status is {found:?}; expected Completed")]
    IncompleteExecutionStatus { found: CampaignExecutionStatus },
    #[error("execution report completed_run_count ({completed}) != run_count ({total})")]
    CompletedRunCountMismatch { completed: usize, total: usize },
    #[error(
        "validation manifest campaign_id {found:?} does not match execution campaign_id {expected:?}"
    )]
    CampaignIdMismatch { found: String, expected: String },
    #[error("validation manifest dataset count {found} != execution run count {expected}")]
    DatasetCountMismatch { found: usize, expected: usize },
    #[error("run {index} execution_status is {found:?}; expected completed_parseable")]
    NonCompletedRun {
        index: usize,
        found: ExecutionStatus,
    },
    #[error("run {index} dataset_id {found:?} does not match validation dataset {expected:?}")]
    DatasetIdMismatch {
        index: usize,
        found: String,
        expected: String,
    },
    #[error("run {index} reynolds {found} does not match validation dataset reynolds {expected}")]
    ReynoldsMismatch {
        index: usize,
        found: f64,
        expected: f64,
    },
    #[error("run {index} mach {found} does not match validation dataset mach {expected}")]
    MachMismatch {
        index: usize,
        found: f64,
        expected: f64,
    },
    #[error("duplicate dataset_id {dataset_id:?} across runs at index {index}")]
    DuplicateDatasetId { index: usize, dataset_id: String },
    #[error(
        "validation dataset {index} polar reference {polar_file:?} is not a relative polars/ path"
    )]
    UnsafePolarReference { index: usize, polar_file: String },
    #[error("polar file {path:?} could not be normalized inside execution directory")]
    PolarOutsideExecutionDir { path: PathBuf },
    #[error("polar file {path:?} for dataset index {index} does not exist")]
    MissingPolar { index: usize, path: PathBuf },
    #[error("non-finite metadata in execution/validation manifest for dataset index {index}")]
    NonFiniteMetadata { index: usize },
    #[error("polar for dataset index {index}, ID {dataset_id:?}, is malformed: {source}")]
    MalformedPolar {
        index: usize,
        dataset_id: String,
        #[source]
        source: XfoilPolarImportError,
    },
    #[error(
        "failed to build evidence bridge for dataset index {index}, ID {dataset_id:?}: {source}"
    )]
    EvidenceBridge {
        index: usize,
        dataset_id: String,
        #[source]
        source: XfoilEvidenceBridgeError,
    },
}

pub fn run_xfoil_evidence_bundle(
    options: XfoilEvidenceBundleOptions,
) -> Result<XfoilEvidenceBundleStatus, XfoilEvidenceBundleError> {
    let owned_artifacts = owned_bundle_artifacts(&options.output_dir);
    clean_owned_output(&options.output_dir, &owned_artifacts).map_err(|source| {
        XfoilEvidenceBundleError::CleanupOwned {
            path: options.output_dir.clone(),
            source,
        }
    })?;

    let staging_root = options
        .output_dir
        .join(format!("{STAGING_DIRECTORY_PREFIX}work"));

    let result = build_evidence_bundle(&options, &staging_root);

    match result {
        Ok(XfoilEvidenceBundleStatus::Built) => {
            promote_staging_to_output(&staging_root, &options.output_dir)?;
            Ok(XfoilEvidenceBundleStatus::Built)
        }
        Ok(XfoilEvidenceBundleStatus::NotPromotable) => {
            let _ = remove_dir_if_exists(&staging_root);
            Ok(XfoilEvidenceBundleStatus::NotPromotable)
        }
        Err(error) => {
            let _ = remove_dir_if_exists(&staging_root);
            clean_owned_output(&options.output_dir, &owned_artifacts).map_err(|source| {
                XfoilEvidenceBundleError::CleanupOwned {
                    path: options.output_dir.clone(),
                    source,
                }
            })?;
            Err(error)
        }
    }
}

fn build_evidence_bundle(
    options: &XfoilEvidenceBundleOptions,
    staging_root: &Path,
) -> Result<XfoilEvidenceBundleStatus, XfoilEvidenceBundleError> {
    fs::create_dir_all(staging_root).map_err(|source| {
        XfoilEvidenceBundleError::CleanupStaging {
            path: staging_root.to_path_buf(),
            source,
        }
    })?;
    fs::create_dir_all(staging_root.join(POLAR_DIRECTORY)).map_err(|source| {
        XfoilEvidenceBundleError::CleanupStaging {
            path: staging_root.join(POLAR_DIRECTORY),
            source,
        }
    })?;

    let result = build_evidence_bundle_inner(options, staging_root);
    match result {
        Ok(status) => Ok(status),
        Err(XfoilEvidenceBundleError::NotPromotable(reason)) => {
            eprintln!("{reason}");
            Ok(XfoilEvidenceBundleStatus::NotPromotable)
        }
        Err(other) => Err(other),
    }
}

fn build_evidence_bundle_inner(
    options: &XfoilEvidenceBundleOptions,
    staging_root: &Path,
) -> Result<XfoilEvidenceBundleStatus, XfoilEvidenceBundleError> {
    let execution_path = options.execution_dir.join(EXECUTION_REPORT);
    let validation_path = options.execution_dir.join(VALIDATION_MANIFEST);
    let polar_dir = options.execution_dir.join(POLAR_DIRECTORY);

    let execution = load_execution_report(&execution_path)?;
    let validation = load_validation_manifest(&validation_path)?;

    let evidence_inputs =
        validate_promotability(&execution, &validation, &options.execution_dir, &polar_dir)?;

    let mut polar_records = Vec::with_capacity(evidence_inputs.len());
    let mut datasets = Vec::with_capacity(evidence_inputs.len());
    for input in &evidence_inputs {
        let metadata = MetadataBuilder::new(input.run.reynolds, input.run.mach)
            .solver_name("XFOIL")
            .transition_assumptions(input.validation.transition_assumptions.clone())
            .ncrit(input.run.ncrit)
            .build()
            .map_err(|_source| NotPromotable::NonFiniteMetadata { index: input.index })?;

        let import = parse_xfoil_polar(&input.polar_text, metadata).map_err(|source| {
            NotPromotable::MalformedPolar {
                index: input.index,
                dataset_id: input.run.dataset_id.clone(),
                source,
            }
        })?;

        let dataset = XfoilEvidenceDatasetBuilder::new(
            import,
            input.run.dataset_id.clone(),
            input.validation.method_id.clone(),
            ConvergenceStatus::Unresolved,
            input.validation.source_ids.clone(),
        )
        .notes(input.validation.notes.clone())
        .build()
        .map_err(|source| NotPromotable::EvidenceBridge {
            index: input.index,
            dataset_id: input.run.dataset_id.clone(),
            source,
        })?;

        let polar_sha256 = sha256_hex(input.polar_text.as_bytes());
        polar_records.push(PolarRecord {
            index: input.index,
            dataset_id: input.run.dataset_id.clone(),
            reynolds: input.run.reynolds,
            mach: input.run.mach,
            convergence_status: "unresolved",
            polar_filename: polar_filename(input.index),
            polar_sha256,
            source_polar_path: input.source_polar_path.clone(),
        });
        datasets.push(dataset);
    }

    let campaign = XfoilEvidenceCampaignBuilder::new(datasets)
        .build()
        .map_err(|source| XfoilEvidenceBundleError::CampaignBuild { source })?;
    // Reuse the M2.9C request validator as a fail-closed sanity check on
    // numeric well-formedness. The canonical numeric fields are preserved
    // as-is in the manifest and never modified by this step.
    XfoilCampaignCoverageRequest::new(
        validation.coverage_request.required_reynolds_min,
        validation.coverage_request.required_reynolds_max,
        validation.coverage_request.required_alpha_min_rad,
        validation.coverage_request.required_alpha_max_rad,
        validation.coverage_request.require_converged,
    )
    .map_err(|source| XfoilEvidenceBundleError::CampaignBuild { source })?;

    let polar_datasets_json = campaign.to_polar_datasets_json_pretty();
    let polar_datasets_sha256 = sha256_hex(polar_datasets_json.as_bytes());

    for record in &polar_records {
        let source = &record.source_polar_path;
        let destination = staging_root
            .join(POLAR_DIRECTORY)
            .join(&record.polar_filename);
        let bytes = fs::read(source).map_err(|source_err| XfoilEvidenceBundleError::PolarRead {
            path: source.clone(),
            source: source_err,
        })?;
        fs::write(&destination, &bytes).map_err(|source_err| {
            XfoilEvidenceBundleError::WriteArtifact {
                path: destination.clone(),
                source: source_err,
            }
        })?;
    }

    let polar_datasets_staging = staging_root.join(POLAR_DATASETS);
    fs::write(&polar_datasets_staging, polar_datasets_json.as_bytes()).map_err(|source| {
        XfoilEvidenceBundleError::WriteArtifact {
            path: polar_datasets_staging.clone(),
            source,
        }
    })?;

    let bundle_manifest = BundleManifest {
        schema_version: MANIFEST_SCHEMA_VERSION,
        generated_by: GENERATED_BY,
        campaign_id: execution.campaign_id.clone(),
        dataset_count: polar_records.len(),
        coverage_request: BundleCoverageRequest::from(&validation.coverage_request),
        datasets: polar_records
            .iter()
            .map(BundleDatasetEntry::from_record)
            .collect(),
        polar_datasets_sha256: &polar_datasets_sha256,
    };
    let bundle_manifest_path = staging_root.join(BUNDLE_MANIFEST);
    let bundle_manifest_bytes = serde_json::to_vec_pretty(&bundle_manifest).map_err(|source| {
        XfoilEvidenceBundleError::SerializeArtifact {
            artifact: BUNDLE_MANIFEST,
            source,
        }
    })?;
    fs::write(&bundle_manifest_path, &bundle_manifest_bytes).map_err(|source| {
        XfoilEvidenceBundleError::WriteArtifact {
            path: bundle_manifest_path.clone(),
            source,
        }
    })?;

    Ok(XfoilEvidenceBundleStatus::Built)
}

fn load_execution_report(path: &Path) -> Result<ExecutionReport, XfoilEvidenceBundleError> {
    let text = fs::read_to_string(path).map_err(|source| {
        XfoilEvidenceBundleError::ExecutionReportRead {
            path: path.to_path_buf(),
            source,
        }
    })?;
    serde_json::from_str::<ExecutionReport>(&text).map_err(|source| {
        XfoilEvidenceBundleError::ExecutionReportDeserialize {
            path: path.to_path_buf(),
            source,
        }
    })
}

fn load_validation_manifest(path: &Path) -> Result<ValidationManifest, XfoilEvidenceBundleError> {
    let text = fs::read_to_string(path).map_err(|source| {
        XfoilEvidenceBundleError::ValidationManifestRead {
            path: path.to_path_buf(),
            source,
        }
    })?;
    serde_json::from_str::<ValidationManifest>(&text).map_err(|source| {
        XfoilEvidenceBundleError::ValidationManifestDeserialize {
            path: path.to_path_buf(),
            source,
        }
    })
}

#[derive(Debug)]
struct EvidenceInput {
    index: usize,
    run: RunMirror,
    validation: ValidationMirror,
    polar_text: String,
    source_polar_path: PathBuf,
}

#[derive(Debug, Clone)]
struct RunMirror {
    dataset_id: String,
    reynolds: f64,
    mach: f64,
    ncrit: f64,
}

#[derive(Debug, Clone)]
struct ValidationMirror {
    method_id: String,
    source_ids: Vec<String>,
    notes: String,
    transition_assumptions: String,
}

fn validate_promotability(
    execution: &ExecutionReport,
    validation: &ValidationManifest,
    execution_root: &Path,
    polar_root: &Path,
) -> Result<Vec<EvidenceInput>, NotPromotable> {
    if execution.schema_version != SUPPORTED_EXECUTION_SCHEMA_VERSION {
        return Err(NotPromotable::UnsupportedExecutionSchemaVersion {
            found: execution.schema_version,
            supported: SUPPORTED_EXECUTION_SCHEMA_VERSION,
        });
    }
    if validation.schema_version != SUPPORTED_VALIDATION_SCHEMA_VERSION {
        return Err(NotPromotable::UnsupportedValidationSchemaVersion {
            found: validation.schema_version,
            supported: SUPPORTED_VALIDATION_SCHEMA_VERSION,
        });
    }
    if execution.run_count == 0 {
        return Err(NotPromotable::ZeroRuns);
    }
    if execution.status != CampaignExecutionStatus::Completed {
        return Err(NotPromotable::IncompleteExecutionStatus {
            found: execution.status,
        });
    }
    if execution.completed_run_count != execution.run_count {
        return Err(NotPromotable::CompletedRunCountMismatch {
            completed: execution.completed_run_count,
            total: execution.run_count,
        });
    }
    if validation.campaign_id != execution.campaign_id {
        return Err(NotPromotable::CampaignIdMismatch {
            found: validation.campaign_id.clone(),
            expected: execution.campaign_id.clone(),
        });
    }
    if validation.datasets.len() != execution.run_count {
        return Err(NotPromotable::DatasetCountMismatch {
            found: validation.datasets.len(),
            expected: execution.run_count,
        });
    }

    let canonical_execution_root = execution_root
        .canonicalize()
        .unwrap_or_else(|_| execution_root.to_path_buf());
    let canonical_polar_root = polar_root
        .canonicalize()
        .unwrap_or_else(|_| polar_root.to_path_buf());

    let mut seen_ids: HashSet<String> = HashSet::new();
    let mut inputs = Vec::with_capacity(execution.run_count);
    for (index, run_report) in execution.runs.iter().enumerate() {
        if run_report.execution_status != ExecutionStatus::CompletedParseable {
            return Err(NotPromotable::NonCompletedRun {
                index,
                found: run_report.execution_status.clone(),
            });
        }
        if !run_report.reynolds.is_finite()
            || !run_report.mach.is_finite()
            || !run_report.ncrit.is_finite()
        {
            return Err(NotPromotable::NonFiniteMetadata { index });
        }
        if !seen_ids.insert(run_report.dataset_id.clone()) {
            return Err(NotPromotable::DuplicateDatasetId {
                index,
                dataset_id: run_report.dataset_id.clone(),
            });
        }

        let validation_dataset: &ValidationDataset =
            validation
                .datasets
                .get(index)
                .ok_or(NotPromotable::DatasetCountMismatch {
                    found: validation.datasets.len(),
                    expected: execution.run_count,
                })?;
        if validation_dataset.dataset_id != run_report.dataset_id {
            return Err(NotPromotable::DatasetIdMismatch {
                index,
                found: run_report.dataset_id.clone(),
                expected: validation_dataset.dataset_id.clone(),
            });
        }
        if validation_dataset.reynolds != run_report.reynolds {
            return Err(NotPromotable::ReynoldsMismatch {
                index,
                found: run_report.reynolds,
                expected: validation_dataset.reynolds,
            });
        }
        if validation_dataset.mach != run_report.mach {
            return Err(NotPromotable::MachMismatch {
                index,
                found: run_report.mach,
                expected: validation_dataset.mach,
            });
        }
        if validation_dataset.convergence_status != "unresolved" {
            return Err(NotPromotable::NonCompletedRun {
                index,
                found: ExecutionStatus::UnparseablePolarOutput,
            });
        }

        let canonical_name = polar_filename(index);
        let expected_relative = format!("{POLAR_DIRECTORY}/{canonical_name}");
        if validation_dataset.polar_file != expected_relative {
            return Err(NotPromotable::UnsafePolarReference {
                index,
                polar_file: validation_dataset.polar_file.clone(),
            });
        }

        let candidate_polar_path = canonical_polar_root.join(&canonical_name);
        if !candidate_polar_path.exists() {
            return Err(NotPromotable::MissingPolar {
                index,
                path: candidate_polar_path,
            });
        }
        let resolved_polar_path =
            candidate_polar_path
                .canonicalize()
                .map_err(|_| NotPromotable::MissingPolar {
                    index,
                    path: candidate_polar_path.clone(),
                })?;
        if !resolved_polar_path.starts_with(&canonical_execution_root) {
            return Err(NotPromotable::PolarOutsideExecutionDir {
                path: resolved_polar_path,
            });
        }

        let polar_text =
            fs::read_to_string(&resolved_polar_path).map_err(|_| NotPromotable::MissingPolar {
                index,
                path: resolved_polar_path.clone(),
            })?;

        inputs.push(EvidenceInput {
            index,
            run: RunMirror {
                dataset_id: run_report.dataset_id.clone(),
                reynolds: run_report.reynolds,
                mach: run_report.mach,
                ncrit: run_report.ncrit,
            },
            validation: ValidationMirror {
                method_id: validation_dataset.method_id.clone(),
                source_ids: validation_dataset.source_ids.clone(),
                notes: validation_dataset.notes.clone(),
                transition_assumptions: validation_dataset.transition_assumptions.clone(),
            },
            polar_text,
            source_polar_path: resolved_polar_path,
        });
    }
    Ok(inputs)
}

#[derive(Debug)]
struct PolarRecord {
    index: usize,
    dataset_id: String,
    reynolds: f64,
    mach: f64,
    convergence_status: &'static str,
    polar_filename: String,
    polar_sha256: String,
    source_polar_path: PathBuf,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct BundleManifest<'a> {
    schema_version: u32,
    generated_by: &'static str,
    campaign_id: String,
    dataset_count: usize,
    coverage_request: BundleCoverageRequest,
    datasets: Vec<BundleDatasetEntry<'a>>,
    polar_datasets_sha256: &'a str,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct BundleCoverageRequest {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

impl<'a> From<&'a crate::xfoil_runner_app::CoverageRequestSpec> for BundleCoverageRequest {
    fn from(source: &'a crate::xfoil_runner_app::CoverageRequestSpec) -> Self {
        Self {
            required_reynolds_min: source.required_reynolds_min,
            required_reynolds_max: source.required_reynolds_max,
            required_alpha_min_rad: source.required_alpha_min_rad,
            required_alpha_max_rad: source.required_alpha_max_rad,
            require_converged: source.require_converged,
        }
    }
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct BundleDatasetEntry<'a> {
    index: usize,
    dataset_id: &'a str,
    reynolds: f64,
    mach: f64,
    convergence_status: &'static str,
    polar_file: String,
    polar_sha256: &'a str,
    polar_dataset_index: usize,
}

impl<'a> BundleDatasetEntry<'a> {
    fn from_record(record: &'a PolarRecord) -> Self {
        Self {
            index: record.index,
            dataset_id: &record.dataset_id,
            reynolds: record.reynolds,
            mach: record.mach,
            convergence_status: record.convergence_status,
            polar_file: format!("{POLAR_DIRECTORY}/{}", record.polar_filename),
            polar_sha256: &record.polar_sha256,
            polar_dataset_index: record.index,
        }
    }
}

fn owned_bundle_artifacts(output_dir: &Path) -> Vec<PathBuf> {
    vec![
        output_dir.join(BUNDLE_MANIFEST),
        output_dir.join(POLAR_DATASETS),
        output_dir.join(POLAR_DIRECTORY),
    ]
}

fn clean_owned_output(output_dir: &Path, owned_artifacts: &[PathBuf]) -> std::io::Result<()> {
    for path in owned_artifacts {
        if path.is_dir() {
            fs::remove_dir_all(path)?;
        } else if path.exists() {
            fs::remove_file(path)?;
        }
    }
    let _ = output_dir;
    Ok(())
}

fn promote_staging_to_output(
    staging_root: &Path,
    output_dir: &Path,
) -> Result<(), XfoilEvidenceBundleError> {
    let staging_bundle = staging_root.join(BUNDLE_MANIFEST);
    let staging_polar_datasets = staging_root.join(POLAR_DATASETS);
    let staging_polars = staging_root.join(POLAR_DIRECTORY);

    let final_bundle = output_dir.join(BUNDLE_MANIFEST);
    let final_polar_datasets = output_dir.join(POLAR_DATASETS);
    let final_polars = output_dir.join(POLAR_DIRECTORY);

    for path in [&final_bundle, &final_polar_datasets, &final_polars] {
        if path.exists() {
            return Err(XfoilEvidenceBundleError::WriteArtifact {
                path: path.clone(),
                source: std::io::Error::new(
                    std::io::ErrorKind::AlreadyExists,
                    "owned bundle target already exists after cleanup",
                ),
            });
        }
    }

    fs::rename(&staging_bundle, &final_bundle).map_err(|source| {
        XfoilEvidenceBundleError::WriteArtifact {
            path: final_bundle.clone(),
            source,
        }
    })?;
    fs::rename(&staging_polar_datasets, &final_polar_datasets).map_err(|source| {
        XfoilEvidenceBundleError::WriteArtifact {
            path: final_polar_datasets.clone(),
            source,
        }
    })?;
    fs::rename(&staging_polars, &final_polars).map_err(|source| {
        XfoilEvidenceBundleError::WriteArtifact {
            path: final_polars.clone(),
            source,
        }
    })?;
    Ok(())
}

fn remove_dir_if_exists(path: &Path) -> std::io::Result<()> {
    if path.exists() {
        fs::remove_dir_all(path)?;
    }
    Ok(())
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}
