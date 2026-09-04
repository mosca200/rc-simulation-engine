//! Deterministic bridge from canonical XFOIL evidence JSON to runtime polar family.
//!
//! M2.9K deserializes the `polar_datasets[]` JSON array (the same format written
//! by M2.9B/M2.9C) and promotes it into an [`XfoilRuntimePolarFamily`] via the
//! existing M2.9H builder. No fitting, resampling, or coefficient modification is
//! performed. No filesystem access occurs.

use serde::Deserialize;

use crate::{
    ConvergenceStatus, MetadataBuilder, XfoilEvidenceCampaignBuilder, XfoilEvidenceDataset,
    XfoilEvidenceDatasetBuilder, XfoilPolarImport, XfoilPolarSample, XfoilRuntimePolarFamily,
    XfoilRuntimePolarFamilyError, build_xfoil_reynolds_polar_family,
};

/// Build a runtime polar family directly from canonical evidence JSON bytes.
///
/// Accepts the `polar_datasets[]` JSON array as produced by
/// `XfoilEvidenceCampaign::to_polar_datasets_json_value`. The conversion is
/// deterministic: repeated calls with the same input produce identical output.
///
/// # Errors
///
/// - Malformed JSON → [`XfoilEvidenceJsonError::MalformedJson`]
/// - Empty array → [`XfoilEvidenceJsonError::EmptyDatasetArray`]
/// - Non-converged dataset → [`XfoilEvidenceJsonError::DatasetNotConverged`]
/// - Inconsistent Mach → [`XfoilEvidenceJsonError::InconsistentMach`]
/// - Duplicate Reynolds → [`XfoilEvidenceJsonError::DuplicateReynolds`]
/// - Reynolds not increasing → [`XfoilEvidenceJsonError::ReynoldsNotIncreasing`]
pub fn build_xfoil_reynolds_polar_family_from_json(
    json_bytes: &[u8],
) -> Result<XfoilRuntimePolarFamily, XfoilEvidenceJsonError> {
    let canonical: Vec<CanonicalPolarDataset> =
        serde_json::from_slice(json_bytes).map_err(XfoilEvidenceJsonError::MalformedJson)?;

    pre_validate(&canonical)?;

    let datasets = construct_evidence_datasets(&canonical)?;

    let campaign = XfoilEvidenceCampaignBuilder::new(datasets)
        .build()
        .map_err(XfoilEvidenceJsonError::CampaignConstruction)?;

    build_xfoil_reynolds_polar_family(&campaign).map_err(XfoilEvidenceJsonError::from)
}

/// Build a runtime polar family from a canonical evidence JSON string.
///
/// Convenience wrapper around [`build_xfoil_reynolds_polar_family_from_json`].
pub fn build_xfoil_reynolds_polar_family_from_json_str(
    json_str: &str,
) -> Result<XfoilRuntimePolarFamily, XfoilEvidenceJsonError> {
    build_xfoil_reynolds_polar_family_from_json(json_str.as_bytes())
}

// ── Canonical JSON deserialization types ──────────────────────────────────────

#[derive(Deserialize)]
struct CanonicalPolarDataset {
    id: String,
    flow_conditions: CanonicalFlowConditions,
    transition: CanonicalTransition,
    method: CanonicalMethod,
    source_ids: Vec<String>,
    samples: Vec<CanonicalSample>,
}

#[derive(Deserialize)]
struct CanonicalFlowConditions {
    reynolds: f64,
    mach: f64,
}

#[derive(Deserialize)]
struct CanonicalTransition {
    assumptions: Option<String>,
    ncrit: Option<f64>,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
}

#[derive(Deserialize)]
struct CanonicalMethod {
    id: String,
    convergence_status: ConvergenceStatus,
    #[serde(default)]
    solver_or_tool: Option<String>,
    #[serde(default)]
    exact_version: Option<String>,
    #[serde(default)]
    command_or_config: Option<String>,
}

#[derive(Deserialize)]
struct CanonicalSample {
    alpha_rad: f64,
    cl: f64,
    cd: f64,
    cm: f64,
}

// ── Pre-validation ────────────────────────────────────────────────────────────

fn pre_validate(datasets: &[CanonicalPolarDataset]) -> Result<(), XfoilEvidenceJsonError> {
    if datasets.is_empty() {
        return Err(XfoilEvidenceJsonError::EmptyDatasetArray);
    }

    let common_mach = datasets[0].flow_conditions.mach;

    for (index, dataset) in datasets.iter().enumerate() {
        if dataset.convergence_status() != ConvergenceStatus::Converged {
            return Err(XfoilEvidenceJsonError::DatasetNotConverged {
                index,
                dataset_id: dataset.id.clone(),
                status: dataset.convergence_status(),
            });
        }

        let mach = dataset.flow_conditions.mach;
        if mach != common_mach {
            return Err(XfoilEvidenceJsonError::InconsistentMach {
                index,
                mach,
                expected_mach: common_mach,
            });
        }

        if index > 0 {
            let prev_re = datasets[index - 1].flow_conditions.reynolds;
            let re = dataset.flow_conditions.reynolds;
            if re == prev_re {
                return Err(XfoilEvidenceJsonError::DuplicateReynolds {
                    previous_index: index - 1,
                    index,
                    reynolds: re,
                });
            }
            if re < prev_re {
                return Err(XfoilEvidenceJsonError::ReynoldsNotIncreasing {
                    previous_index: index - 1,
                    index,
                    previous_reynolds: prev_re,
                    reynolds: re,
                });
            }
        }
    }

    Ok(())
}

impl CanonicalPolarDataset {
    fn convergence_status(&self) -> ConvergenceStatus {
        self.method.convergence_status
    }
}

// ── Construction of evidence datasets from canonical JSON ─────────────────────

fn construct_evidence_datasets(
    canonical: &[CanonicalPolarDataset],
) -> Result<Vec<XfoilEvidenceDataset>, XfoilEvidenceJsonError> {
    let mut datasets = Vec::with_capacity(canonical.len());

    for (index, ds) in canonical.iter().enumerate() {
        let samples: Vec<XfoilPolarSample> = ds
            .samples
            .iter()
            .map(|s| XfoilPolarSample::from_parts(s.alpha_rad, s.cl, s.cd, s.cm))
            .collect();

        let mut meta_builder =
            MetadataBuilder::new(ds.flow_conditions.reynolds, ds.flow_conditions.mach);

        if let Some(ref name) = ds.method.solver_or_tool {
            meta_builder = meta_builder.solver_name(name.as_str());
        }
        if let Some(ref version) = ds.method.exact_version {
            meta_builder = meta_builder.solver_version(version.as_str());
        }
        if let Some(ref config) = ds.method.command_or_config {
            meta_builder = meta_builder.command_or_config(config.as_str());
        }
        if let Some(ref assumptions) = ds.transition.assumptions {
            meta_builder = meta_builder.transition_assumptions(assumptions.as_str());
        }
        if let Some(ncrit) = ds.transition.ncrit {
            meta_builder = meta_builder.ncrit(ncrit);
        }
        if let Some(v) = ds.transition.forced_transition_upper_x_over_c {
            meta_builder = meta_builder.forced_transition_upper(v);
        }
        if let Some(v) = ds.transition.forced_transition_lower_x_over_c {
            meta_builder = meta_builder.forced_transition_lower(v);
        }

        let metadata = meta_builder
            .build()
            .map_err(|source| XfoilEvidenceJsonError::InvalidMetadata { index, source })?;

        let import = XfoilPolarImport::from_parts(metadata, samples);

        let dataset = XfoilEvidenceDatasetBuilder::new(
            import,
            &ds.id,
            &ds.method.id,
            ConvergenceStatus::Converged,
            ds.source_ids.clone(),
        )
        .build()
        .map_err(|source| XfoilEvidenceJsonError::from_evidence_bridge(index, source))?;

        datasets.push(dataset);
    }

    Ok(datasets)
}

// ── Error type ────────────────────────────────────────────────────────────────

/// Errors from the canonical-evidence-JSON to runtime polar family bridge.
#[derive(Debug, thiserror::Error)]
pub enum XfoilEvidenceJsonError {
    #[error("malformed canonical evidence JSON: {0}")]
    MalformedJson(serde_json::Error),

    #[error("canonical evidence dataset array is empty")]
    EmptyDatasetArray,

    #[error(
        "dataset at index {index} (id={dataset_id}) has convergence status {status:?}, expected Converged"
    )]
    DatasetNotConverged {
        index: usize,
        dataset_id: String,
        status: ConvergenceStatus,
    },

    #[error(
        "inconsistent Mach: dataset at index {index} has mach={mach} but expected {expected_mach}"
    )]
    InconsistentMach {
        index: usize,
        mach: f64,
        expected_mach: f64,
    },

    #[error("duplicate Reynolds node {reynolds} at indices {previous_index} and {index}")]
    DuplicateReynolds {
        previous_index: usize,
        index: usize,
        reynolds: f64,
    },

    #[error(
        "Reynolds nodes not increasing: index {previous_index} has {previous_reynolds}, index {index} has {reynolds}"
    )]
    ReynoldsNotIncreasing {
        previous_index: usize,
        index: usize,
        previous_reynolds: f64,
        reynolds: f64,
    },

    #[error("dataset at index {index}: invalid solver metadata: {source}")]
    InvalidMetadata {
        index: usize,
        source: crate::XfoilPolarImportError,
    },

    #[error("dataset at index {index}: {source}")]
    EvidenceBridge {
        index: usize,
        source: crate::XfoilEvidenceBridgeError,
    },

    #[error("campaign construction failed: {0}")]
    CampaignConstruction(crate::XfoilEvidenceCampaignError),

    #[error(
        "dataset at index {index} (id={dataset_id}) has convergence status {status:?}, expected Converged"
    )]
    RuntimeDatasetNotConverged {
        index: usize,
        dataset_id: String,
        status: ConvergenceStatus,
    },

    #[error(
        "inconsistent Mach: dataset at index {index} has mach={mach} but expected {expected_mach}"
    )]
    RuntimeInconsistentMach {
        index: usize,
        mach: f64,
        expected_mach: f64,
    },

    #[error("failed to construct PolarTable for dataset at index {index}: {source}")]
    PolarTableConstruction {
        index: usize,
        source: sim_core::PolarError,
    },

    #[error("failed to construct ReynoldsPolar for dataset at index {index}: {source}")]
    ReynoldsPolarConstruction {
        index: usize,
        source: sim_core::ReynoldsPolarFamilyError,
    },

    #[error("failed to construct ReynoldsPolarFamily: {0}")]
    ReynoldsPolarFamilyConstruction(#[from] sim_core::ReynoldsPolarFamilyError),
}

impl XfoilEvidenceJsonError {
    fn from_evidence_bridge(index: usize, source: crate::XfoilEvidenceBridgeError) -> Self {
        Self::EvidenceBridge { index, source }
    }
}

impl From<XfoilRuntimePolarFamilyError> for XfoilEvidenceJsonError {
    fn from(err: XfoilRuntimePolarFamilyError) -> Self {
        match err {
            XfoilRuntimePolarFamilyError::EmptyCampaign => Self::EmptyDatasetArray,
            XfoilRuntimePolarFamilyError::DatasetNotConverged {
                index,
                dataset_id,
                status,
            } => Self::RuntimeDatasetNotConverged {
                index,
                dataset_id,
                status,
            },
            XfoilRuntimePolarFamilyError::InconsistentMach {
                index,
                mach,
                expected_mach,
            } => Self::RuntimeInconsistentMach {
                index,
                mach,
                expected_mach,
            },
            XfoilRuntimePolarFamilyError::PolarTableConstruction { index, source } => {
                Self::PolarTableConstruction { index, source }
            }
            XfoilRuntimePolarFamilyError::ReynoldsPolarConstruction { index, source } => {
                Self::ReynoldsPolarConstruction { index, source }
            }
            XfoilRuntimePolarFamilyError::ReynoldsPolarFamilyConstruction(source) => {
                Self::ReynoldsPolarFamilyConstruction(source)
            }
        }
    }
}
