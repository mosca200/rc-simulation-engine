//! Bridge from validated XFOIL evidence campaigns to runtime Reynolds polar families.
//!
//! Converts [`XfoilEvidenceCampaign`] datasets into a [`ReynoldsPolarFamily`] with
//! deterministic, lossless sample mapping. No fitting, smoothing, resampling, or
//! alpha-grid normalization is performed.

use sim_core::{PolarError, PolarSample, PolarTable, ReynoldsPolar, ReynoldsPolarFamily};

use crate::{ConvergenceStatus, XfoilEvidenceCampaign, XfoilEvidenceDataset};

/// A runtime polar family derived from an XFOIL evidence campaign.
///
/// Contains the converted [`ReynoldsPolarFamily`] and the common Mach number
/// shared by all datasets in the source campaign.
#[derive(Debug, Clone)]
pub struct XfoilRuntimePolarFamily {
    family: ReynoldsPolarFamily,
    mach: f64,
}

impl XfoilRuntimePolarFamily {
    /// The converted Reynolds polar family.
    pub fn family(&self) -> &ReynoldsPolarFamily {
        &self.family
    }

    /// The common Mach number for all datasets in the source campaign.
    pub fn mach(&self) -> f64 {
        self.mach
    }
}

/// Errors that can occur when building a runtime polar family from XFOIL evidence.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum XfoilRuntimePolarFamilyError {
    #[error("campaign contains no datasets")]
    EmptyCampaign,

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

    #[error("failed to construct PolarTable for dataset at index {index}: {source}")]
    PolarTableConstruction { index: usize, source: PolarError },

    #[error("failed to construct ReynoldsPolar for dataset at index {index}: {source}")]
    ReynoldsPolarConstruction {
        index: usize,
        source: sim_core::ReynoldsPolarFamilyError,
    },

    #[error("failed to construct ReynoldsPolarFamily: {0}")]
    ReynoldsPolarFamilyConstruction(#[from] sim_core::ReynoldsPolarFamilyError),
}

/// Build a [`ReynoldsPolarFamily`] from a validated XFOIL evidence campaign.
///
/// The conversion is deterministic and lossless:
/// - Each converged dataset becomes one [`ReynoldsPolar`] node.
/// - Sample order, alpha grids, and coefficients are preserved exactly.
/// - All datasets must share the same Mach number (exact f64 equality).
/// - Only datasets with [`ConvergenceStatus::Converged`] are accepted.
pub fn build_xfoil_reynolds_polar_family(
    campaign: &XfoilEvidenceCampaign,
) -> Result<XfoilRuntimePolarFamily, XfoilRuntimePolarFamilyError> {
    let datasets = campaign.datasets();

    if datasets.is_empty() {
        return Err(XfoilRuntimePolarFamilyError::EmptyCampaign);
    }

    let common_mach = datasets[0].mach();

    let mut nodes = Vec::with_capacity(datasets.len());

    for (index, dataset) in datasets.iter().enumerate() {
        if dataset.convergence_status() != ConvergenceStatus::Converged {
            return Err(XfoilRuntimePolarFamilyError::DatasetNotConverged {
                index,
                dataset_id: dataset.dataset_id().to_owned(),
                status: dataset.convergence_status(),
            });
        }

        let dataset_mach = dataset.mach();
        if dataset_mach != common_mach {
            return Err(XfoilRuntimePolarFamilyError::InconsistentMach {
                index,
                mach: dataset_mach,
                expected_mach: common_mach,
            });
        }

        let samples = map_samples(dataset);
        let table = PolarTable::new(samples).map_err(|source| {
            XfoilRuntimePolarFamilyError::PolarTableConstruction { index, source }
        })?;

        let polar = ReynoldsPolar::new(dataset.reynolds(), table).map_err(|source| {
            XfoilRuntimePolarFamilyError::ReynoldsPolarConstruction { index, source }
        })?;

        nodes.push(polar);
    }

    let family = ReynoldsPolarFamily::new(nodes)?;

    Ok(XfoilRuntimePolarFamily {
        family,
        mach: common_mach,
    })
}

fn map_samples(dataset: &XfoilEvidenceDataset) -> Vec<PolarSample> {
    dataset
        .import()
        .samples()
        .iter()
        .map(|s| PolarSample {
            alpha_rad: s.alpha_rad(),
            cl: s.cl(),
            cd: s.cd(),
            cm: s.cm(),
        })
        .collect()
}
