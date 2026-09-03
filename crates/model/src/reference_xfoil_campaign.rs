//! Deterministic assembly and coverage auditing for XFOIL evidence campaigns.
//!
//! M2.9C combines ordered [`XfoilEvidenceDataset`] values from M2.9B and
//! audits an explicit Reynolds/alpha request. It remains evidence-only: it
//! does not create runtime polar types, interpolate coefficients, or alter
//! aerodynamic physics.

use std::collections::HashSet;

use serde_json::Value;
use thiserror::Error;

use crate::{ConvergenceStatus, XfoilEvidenceDataset};

/// An ordered, validated series of XFOIL evidence datasets.
#[derive(Debug, Clone)]
pub struct XfoilEvidenceCampaign {
    datasets: Vec<XfoilEvidenceDataset>,
}

impl XfoilEvidenceCampaign {
    /// Datasets in the exact caller-supplied order validated by the builder.
    pub fn datasets(&self) -> &[XfoilEvidenceDataset] {
        &self.datasets
    }

    /// Number of datasets in the campaign.
    pub fn dataset_count(&self) -> usize {
        self.datasets.len()
    }

    /// The first, and therefore minimum, Reynolds node.
    pub fn minimum_reynolds(&self) -> f64 {
        self.datasets[0].reynolds()
    }

    /// The last, and therefore maximum, Reynolds node.
    pub fn maximum_reynolds(&self) -> f64 {
        self.datasets[self.datasets.len() - 1].reynolds()
    }

    /// Audit this campaign against an explicit validated coverage request.
    pub fn audit_coverage(&self, request: &XfoilCampaignCoverageRequest) -> XfoilCampaignCoverage {
        let minimum_reynolds = self.minimum_reynolds();
        let maximum_reynolds = self.maximum_reynolds();
        let mut blockers = Vec::new();

        if minimum_reynolds > request.required_reynolds_min {
            blockers.push(
                XfoilCampaignCoverageBlocker::ReynoldsCoverageBelowRequired {
                    campaign_minimum_reynolds: minimum_reynolds,
                    required_minimum_reynolds: request.required_reynolds_min,
                },
            );
        }
        if maximum_reynolds < request.required_reynolds_max {
            blockers.push(
                XfoilCampaignCoverageBlocker::ReynoldsCoverageAboveRequired {
                    campaign_maximum_reynolds: maximum_reynolds,
                    required_maximum_reynolds: request.required_reynolds_max,
                },
            );
        }

        let mut dataset_coverage = Vec::with_capacity(self.datasets.len());
        for (index, dataset) in self.datasets.iter().enumerate() {
            let (alpha_min_rad, alpha_max_rad) = alpha_bounds(dataset);
            let covers_required_alpha_min = alpha_min_rad <= request.required_alpha_min_rad;
            let covers_required_alpha_max = alpha_max_rad >= request.required_alpha_max_rad;

            if request.require_converged
                && dataset.convergence_status() != ConvergenceStatus::Converged
            {
                blockers.push(XfoilCampaignCoverageBlocker::DatasetNotConverged {
                    index,
                    dataset_id: dataset.dataset_id().to_owned(),
                    status: dataset.convergence_status(),
                });
            }
            if !covers_required_alpha_min {
                blockers.push(XfoilCampaignCoverageBlocker::DatasetAlphaBelowRequired {
                    index,
                    dataset_id: dataset.dataset_id().to_owned(),
                    dataset_alpha_min_rad: alpha_min_rad,
                    required_alpha_min_rad: request.required_alpha_min_rad,
                });
            }
            if !covers_required_alpha_max {
                blockers.push(XfoilCampaignCoverageBlocker::DatasetAlphaAboveRequired {
                    index,
                    dataset_id: dataset.dataset_id().to_owned(),
                    dataset_alpha_max_rad: alpha_max_rad,
                    required_alpha_max_rad: request.required_alpha_max_rad,
                });
            }

            dataset_coverage.push(XfoilCampaignDatasetCoverage {
                index,
                dataset_id: dataset.dataset_id().to_owned(),
                method_id: dataset.method_id().to_owned(),
                reynolds: dataset.reynolds(),
                mach: dataset.mach(),
                convergence_status: dataset.convergence_status(),
                alpha_min_rad,
                alpha_max_rad,
                covers_required_alpha_min,
                covers_required_alpha_max,
            });
        }

        let status = if blockers.is_empty() {
            XfoilCampaignCoverageStatus::Qualified
        } else {
            XfoilCampaignCoverageStatus::NotQualified
        };

        XfoilCampaignCoverage {
            request: *request,
            campaign_minimum_reynolds: minimum_reynolds,
            campaign_maximum_reynolds: maximum_reynolds,
            datasets: dataset_coverage,
            blockers,
            status,
        }
    }

    /// Return the exact M2.9B JSON value for each dataset in campaign order.
    pub fn to_polar_datasets_json_value(&self) -> Value {
        Value::Array(
            self.datasets
                .iter()
                .map(XfoilEvidenceDataset::to_json_value)
                .collect(),
        )
    }

    /// Pretty-print the ordered `polar_datasets` JSON array deterministically.
    pub fn to_polar_datasets_json_pretty(&self) -> String {
        serde_json::to_string_pretty(&self.to_polar_datasets_json_value())
            .expect("campaign dataset array serializes")
    }
}

/// Builder for an ordered [`XfoilEvidenceCampaign`].
#[derive(Debug, Clone)]
pub struct XfoilEvidenceCampaignBuilder {
    datasets: Vec<XfoilEvidenceDataset>,
}

impl XfoilEvidenceCampaignBuilder {
    /// Create a builder. Dataset order is significant and is never sorted.
    pub fn new(datasets: Vec<XfoilEvidenceDataset>) -> Self {
        Self { datasets }
    }

    /// Validate campaign identity and strictly increasing Reynolds nodes.
    pub fn build(self) -> Result<XfoilEvidenceCampaign, XfoilEvidenceCampaignError> {
        if self.datasets.is_empty() {
            return Err(XfoilEvidenceCampaignError::EmptyCampaign);
        }

        let mut dataset_ids = HashSet::new();
        for (index, dataset) in self.datasets.iter().enumerate() {
            if !dataset_ids.insert(dataset.dataset_id()) {
                return Err(XfoilEvidenceCampaignError::DuplicateDatasetId {
                    index,
                    dataset_id: dataset.dataset_id().to_owned(),
                });
            }

            if index > 0 {
                let previous_reynolds = self.datasets[index - 1].reynolds();
                let reynolds = dataset.reynolds();
                if reynolds == previous_reynolds {
                    return Err(XfoilEvidenceCampaignError::DuplicateReynolds {
                        previous_index: index - 1,
                        index,
                        reynolds,
                    });
                }
                if reynolds < previous_reynolds {
                    return Err(XfoilEvidenceCampaignError::ReynoldsNotIncreasing {
                        previous_index: index - 1,
                        index,
                        previous_reynolds,
                        reynolds,
                    });
                }
            }
        }

        Ok(XfoilEvidenceCampaign {
            datasets: self.datasets,
        })
    }
}

/// Explicit aerodynamic envelope and convergence policy for a campaign audit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XfoilCampaignCoverageRequest {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

impl XfoilCampaignCoverageRequest {
    /// Validate and construct an explicit coverage request.
    pub fn new(
        required_reynolds_min: f64,
        required_reynolds_max: f64,
        required_alpha_min_rad: f64,
        required_alpha_max_rad: f64,
        require_converged: bool,
    ) -> Result<Self, XfoilEvidenceCampaignError> {
        if !required_reynolds_min.is_finite() {
            return Err(XfoilEvidenceCampaignError::RequiredReynoldsMinimumNotFinite);
        }
        if !required_reynolds_max.is_finite() {
            return Err(XfoilEvidenceCampaignError::RequiredReynoldsMaximumNotFinite);
        }
        if required_reynolds_min <= 0.0 {
            return Err(XfoilEvidenceCampaignError::RequiredReynoldsMinimumNotPositive);
        }
        if required_reynolds_max <= required_reynolds_min {
            return Err(XfoilEvidenceCampaignError::RequiredReynoldsBoundsNotIncreasing);
        }
        if !required_alpha_min_rad.is_finite() {
            return Err(XfoilEvidenceCampaignError::RequiredAlphaMinimumNotFinite);
        }
        if !required_alpha_max_rad.is_finite() {
            return Err(XfoilEvidenceCampaignError::RequiredAlphaMaximumNotFinite);
        }
        if required_alpha_max_rad <= required_alpha_min_rad {
            return Err(XfoilEvidenceCampaignError::RequiredAlphaBoundsNotIncreasing);
        }

        Ok(Self {
            required_reynolds_min,
            required_reynolds_max,
            required_alpha_min_rad,
            required_alpha_max_rad,
            require_converged,
        })
    }

    pub const fn required_reynolds_min(&self) -> f64 {
        self.required_reynolds_min
    }

    pub const fn required_reynolds_max(&self) -> f64 {
        self.required_reynolds_max
    }

    pub const fn required_alpha_min_rad(&self) -> f64 {
        self.required_alpha_min_rad
    }

    pub const fn required_alpha_max_rad(&self) -> f64 {
        self.required_alpha_max_rad
    }

    pub const fn require_converged(&self) -> bool {
        self.require_converged
    }
}

/// Qualification outcome for the complete explicit request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XfoilCampaignCoverageStatus {
    Qualified,
    NotQualified,
}

/// Ordered coverage facts for one campaign dataset.
#[derive(Debug, Clone, PartialEq)]
pub struct XfoilCampaignDatasetCoverage {
    index: usize,
    dataset_id: String,
    method_id: String,
    reynolds: f64,
    mach: f64,
    convergence_status: ConvergenceStatus,
    alpha_min_rad: f64,
    alpha_max_rad: f64,
    covers_required_alpha_min: bool,
    covers_required_alpha_max: bool,
}

impl XfoilCampaignDatasetCoverage {
    pub const fn index(&self) -> usize {
        self.index
    }

    pub fn dataset_id(&self) -> &str {
        &self.dataset_id
    }

    pub fn method_id(&self) -> &str {
        &self.method_id
    }

    pub const fn reynolds(&self) -> f64 {
        self.reynolds
    }

    pub const fn mach(&self) -> f64 {
        self.mach
    }

    pub const fn convergence_status(&self) -> ConvergenceStatus {
        self.convergence_status
    }

    pub const fn alpha_min_rad(&self) -> f64 {
        self.alpha_min_rad
    }

    pub const fn alpha_max_rad(&self) -> f64 {
        self.alpha_max_rad
    }

    pub const fn covers_required_alpha_min(&self) -> bool {
        self.covers_required_alpha_min
    }

    pub const fn covers_required_alpha_max(&self) -> bool {
        self.covers_required_alpha_max
    }
}

/// All deterministic facts and blockers produced by a campaign audit.
#[derive(Debug, Clone, PartialEq)]
pub struct XfoilCampaignCoverage {
    request: XfoilCampaignCoverageRequest,
    campaign_minimum_reynolds: f64,
    campaign_maximum_reynolds: f64,
    datasets: Vec<XfoilCampaignDatasetCoverage>,
    blockers: Vec<XfoilCampaignCoverageBlocker>,
    status: XfoilCampaignCoverageStatus,
}

impl XfoilCampaignCoverage {
    pub const fn request(&self) -> &XfoilCampaignCoverageRequest {
        &self.request
    }

    pub const fn campaign_minimum_reynolds(&self) -> f64 {
        self.campaign_minimum_reynolds
    }

    pub const fn campaign_maximum_reynolds(&self) -> f64 {
        self.campaign_maximum_reynolds
    }

    pub fn datasets(&self) -> &[XfoilCampaignDatasetCoverage] {
        &self.datasets
    }

    pub fn blockers(&self) -> &[XfoilCampaignCoverageBlocker] {
        &self.blockers
    }

    pub const fn status(&self) -> XfoilCampaignCoverageStatus {
        self.status
    }

    pub const fn is_qualified(&self) -> bool {
        matches!(self.status, XfoilCampaignCoverageStatus::Qualified)
    }
}

/// Typed reasons that an explicit campaign coverage request is not qualified.
#[derive(Debug, Clone, PartialEq)]
pub enum XfoilCampaignCoverageBlocker {
    ReynoldsCoverageBelowRequired {
        campaign_minimum_reynolds: f64,
        required_minimum_reynolds: f64,
    },
    ReynoldsCoverageAboveRequired {
        campaign_maximum_reynolds: f64,
        required_maximum_reynolds: f64,
    },
    DatasetNotConverged {
        index: usize,
        dataset_id: String,
        status: ConvergenceStatus,
    },
    DatasetAlphaBelowRequired {
        index: usize,
        dataset_id: String,
        dataset_alpha_min_rad: f64,
        required_alpha_min_rad: f64,
    },
    DatasetAlphaAboveRequired {
        index: usize,
        dataset_id: String,
        dataset_alpha_max_rad: f64,
        required_alpha_max_rad: f64,
    },
}

/// Fail-closed campaign construction and request-validation errors.
#[derive(Debug, Error, PartialEq)]
pub enum XfoilEvidenceCampaignError {
    #[error("XFOIL evidence campaign requires at least one dataset")]
    EmptyCampaign,

    #[error("duplicate campaign dataset ID {dataset_id:?} at index {index}")]
    DuplicateDatasetId { index: usize, dataset_id: String },

    #[error("duplicate campaign Reynolds node {reynolds} at indices {previous_index} and {index}")]
    DuplicateReynolds {
        previous_index: usize,
        index: usize,
        reynolds: f64,
    },

    #[error(
        "campaign Reynolds nodes are not increasing: index {previous_index} has {previous_reynolds}, index {index} has {reynolds}"
    )]
    ReynoldsNotIncreasing {
        previous_index: usize,
        index: usize,
        previous_reynolds: f64,
        reynolds: f64,
    },

    #[error("required minimum Reynolds number must be finite")]
    RequiredReynoldsMinimumNotFinite,

    #[error("required maximum Reynolds number must be finite")]
    RequiredReynoldsMaximumNotFinite,

    #[error("required minimum Reynolds number must be positive")]
    RequiredReynoldsMinimumNotPositive,

    #[error("required Reynolds bounds must be strictly increasing")]
    RequiredReynoldsBoundsNotIncreasing,

    #[error("required minimum alpha must be finite")]
    RequiredAlphaMinimumNotFinite,

    #[error("required maximum alpha must be finite")]
    RequiredAlphaMaximumNotFinite,

    #[error("required alpha bounds must be strictly increasing")]
    RequiredAlphaBoundsNotIncreasing,
}

fn alpha_bounds(dataset: &XfoilEvidenceDataset) -> (f64, f64) {
    let samples = dataset.import().samples();
    (
        samples[0].alpha_rad(),
        samples[samples.len() - 1].alpha_rad(),
    )
}
