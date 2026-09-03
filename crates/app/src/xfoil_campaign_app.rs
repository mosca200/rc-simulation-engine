#![forbid(unsafe_code)]

use std::{fs, path::PathBuf};

use model::{
    ConvergenceStatus, MetadataBuilder, XfoilCampaignCoverage, XfoilCampaignCoverageBlocker,
    XfoilCampaignCoverageRequest, XfoilCampaignCoverageStatus, XfoilEvidenceBridgeError,
    XfoilEvidenceCampaign, XfoilEvidenceCampaignBuilder, XfoilEvidenceCampaignError,
    XfoilEvidenceDatasetBuilder, XfoilPolarImportError, parse_xfoil_polar,
};
use serde::{Deserialize, Deserializer, Serialize, Serializer, de};
use thiserror::Error;

const MANIFEST_SCHEMA_VERSION: u32 = 1;
const REPORT_SCHEMA_VERSION: u32 = 1;
const GENERATED_BY: &str = "rcsim-app validate xfoil-campaign";
const JSON_REPORT: &str = "xfoil_campaign.json";
const MARKDOWN_REPORT: &str = "xfoil_campaign.md";
const POLAR_DATASETS_REPORT: &str = "polar_datasets.json";

#[derive(Debug, Clone)]
pub struct XfoilCampaignOptions {
    manifest_path: PathBuf,
    output_dir: PathBuf,
}

impl XfoilCampaignOptions {
    pub fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut manifest_path = None;
        let mut output_dir = None;
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--manifest" => {
                    if manifest_path.is_some() {
                        return Err("--manifest may be supplied only once".to_owned());
                    }
                    manifest_path = Some(PathBuf::from(
                        args.next()
                            .ok_or_else(|| "missing value for --manifest".to_owned())?,
                    ));
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
                _ => return Err(format!("unknown XFOIL campaign argument: {argument}")),
            }
        }
        Ok(Self {
            manifest_path: manifest_path.ok_or_else(|| "--manifest PATH is required".to_owned())?,
            output_dir: output_dir.ok_or_else(|| "--output-dir PATH is required".to_owned())?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XfoilCampaignRunStatus {
    Qualified,
    NotQualified,
}

#[derive(Debug, Error)]
pub enum XfoilCampaignAppError {
    #[error("failed to read XFOIL campaign manifest {path:?}: {source}")]
    ManifestRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to deserialize XFOIL campaign manifest {path:?}: {source}")]
    ManifestDeserialize {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error(
        "unsupported XFOIL campaign manifest schema version {found}; supported version is {supported}"
    )]
    UnsupportedManifestSchemaVersion { found: u32, supported: u32 },
    #[error(
        "failed to read polar for dataset index {index}, ID {dataset_id:?}, manifest path {polar_file:?}, resolved path {resolved_path:?}: {source}"
    )]
    PolarRead {
        index: usize,
        dataset_id: String,
        polar_file: String,
        resolved_path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error(
        "invalid solver metadata for dataset index {index}, ID {dataset_id:?}, polar path {polar_file:?}: {source}"
    )]
    Metadata {
        index: usize,
        dataset_id: String,
        polar_file: String,
        #[source]
        source: XfoilPolarImportError,
    },
    #[error(
        "failed to parse XFOIL polar for dataset index {index}, ID {dataset_id:?}, polar path {polar_file:?}: {source}"
    )]
    XfoilParse {
        index: usize,
        dataset_id: String,
        polar_file: String,
        #[source]
        source: XfoilPolarImportError,
    },
    #[error(
        "failed to build evidence for dataset index {index}, ID {dataset_id:?}, polar path {polar_file:?}: {source}"
    )]
    EvidenceBridge {
        index: usize,
        dataset_id: String,
        polar_file: String,
        #[source]
        source: XfoilEvidenceBridgeError,
    },
    #[error("failed to build ordered XFOIL evidence campaign: {source}")]
    CampaignBuild {
        #[source]
        source: XfoilEvidenceCampaignError,
    },
    #[error("invalid XFOIL campaign coverage request: {source}")]
    CoverageRequest {
        #[source]
        source: XfoilEvidenceCampaignError,
    },
    #[error("failed to create output directory {path:?}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to serialize report {report_name}: {source}")]
    SerializeReport {
        report_name: &'static str,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to write report {path:?}: {source}")]
    WriteReport {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    schema_version: u32,
    campaign_id: String,
    datasets: Vec<DatasetSpec>,
    coverage_request: CoverageRequestSpec,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetSpec {
    polar_file: String,
    dataset_id: String,
    method_id: String,
    convergence_status: ManifestConvergenceStatus,
    source_ids: Vec<String>,
    notes: Option<String>,
    reynolds: f64,
    mach: f64,
    solver_name: Option<String>,
    solver_version: Option<String>,
    command_or_config: Option<String>,
    transition_assumptions: Option<String>,
    ncrit: Option<f64>,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ManifestConvergenceStatus {
    Converged,
    Unresolved,
    Failed,
}

impl From<ManifestConvergenceStatus> for ConvergenceStatus {
    fn from(value: ManifestConvergenceStatus) -> Self {
        match value {
            ManifestConvergenceStatus::Converged => Self::Converged,
            ManifestConvergenceStatus::Unresolved => Self::Unresolved,
            ManifestConvergenceStatus::Failed => Self::Failed,
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoverageRequestSpec {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ReportSchemaVersion;

impl Serialize for ReportSchemaVersion {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serializer.serialize_u32(REPORT_SCHEMA_VERSION)
    }
}

impl<'de> Deserialize<'de> for ReportSchemaVersion {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let found = u32::deserialize(deserializer)?;
        if found == REPORT_SCHEMA_VERSION {
            Ok(Self)
        } else {
            Err(de::Error::custom(format!(
                "unsupported XFOIL campaign report schema version {found}; supported version is {REPORT_SCHEMA_VERSION}"
            )))
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignReport {
    schema_version: ReportSchemaVersion,
    generated_by: String,
    campaign_id: String,
    manifest: ManifestReport,
    summary: SummaryReport,
    coverage_request: CoverageRequestReport,
    campaign_reynolds_range: ReynoldsRangeReport,
    datasets: Vec<DatasetReport>,
    blockers: Vec<BlockerReport>,
    status: ReportStatus,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ManifestReport {
    schema_version: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SummaryReport {
    dataset_count: usize,
    converged_dataset_count: usize,
    unresolved_dataset_count: usize,
    failed_dataset_count: usize,
    blocker_count: usize,
    qualified: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoverageRequestReport {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReynoldsRangeReport {
    minimum_reynolds: f64,
    maximum_reynolds: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatasetReport {
    index: usize,
    dataset_id: String,
    method_id: String,
    polar_file: String,
    reynolds: f64,
    mach: f64,
    convergence_status: ConvergenceStatus,
    sample_count: usize,
    alpha_min_rad: f64,
    alpha_max_rad: f64,
    covers_required_alpha_min: bool,
    covers_required_alpha_max: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum BlockerReport {
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

impl From<&XfoilCampaignCoverageBlocker> for BlockerReport {
    fn from(value: &XfoilCampaignCoverageBlocker) -> Self {
        match value {
            XfoilCampaignCoverageBlocker::ReynoldsCoverageBelowRequired {
                campaign_minimum_reynolds,
                required_minimum_reynolds,
            } => Self::ReynoldsCoverageBelowRequired {
                campaign_minimum_reynolds: *campaign_minimum_reynolds,
                required_minimum_reynolds: *required_minimum_reynolds,
            },
            XfoilCampaignCoverageBlocker::ReynoldsCoverageAboveRequired {
                campaign_maximum_reynolds,
                required_maximum_reynolds,
            } => Self::ReynoldsCoverageAboveRequired {
                campaign_maximum_reynolds: *campaign_maximum_reynolds,
                required_maximum_reynolds: *required_maximum_reynolds,
            },
            XfoilCampaignCoverageBlocker::DatasetNotConverged {
                index,
                dataset_id,
                status,
            } => Self::DatasetNotConverged {
                index: *index,
                dataset_id: dataset_id.clone(),
                status: *status,
            },
            XfoilCampaignCoverageBlocker::DatasetAlphaBelowRequired {
                index,
                dataset_id,
                dataset_alpha_min_rad,
                required_alpha_min_rad,
            } => Self::DatasetAlphaBelowRequired {
                index: *index,
                dataset_id: dataset_id.clone(),
                dataset_alpha_min_rad: *dataset_alpha_min_rad,
                required_alpha_min_rad: *required_alpha_min_rad,
            },
            XfoilCampaignCoverageBlocker::DatasetAlphaAboveRequired {
                index,
                dataset_id,
                dataset_alpha_max_rad,
                required_alpha_max_rad,
            } => Self::DatasetAlphaAboveRequired {
                index: *index,
                dataset_id: dataset_id.clone(),
                dataset_alpha_max_rad: *dataset_alpha_max_rad,
                required_alpha_max_rad: *required_alpha_max_rad,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ReportStatus {
    Qualified,
    NotQualified,
}

pub fn run_xfoil_campaign_validation(
    options: XfoilCampaignOptions,
) -> Result<XfoilCampaignRunStatus, XfoilCampaignAppError> {
    let text = fs::read_to_string(&options.manifest_path).map_err(|source| {
        XfoilCampaignAppError::ManifestRead {
            path: options.manifest_path.clone(),
            source,
        }
    })?;
    let manifest: Manifest = serde_json::from_str(&text).map_err(|source| {
        XfoilCampaignAppError::ManifestDeserialize {
            path: options.manifest_path.clone(),
            source,
        }
    })?;
    if manifest.schema_version != MANIFEST_SCHEMA_VERSION {
        return Err(XfoilCampaignAppError::UnsupportedManifestSchemaVersion {
            found: manifest.schema_version,
            supported: MANIFEST_SCHEMA_VERSION,
        });
    }

    let manifest_directory = options
        .manifest_path
        .parent()
        .unwrap_or_else(|| std::path::Path::new(""));
    let mut datasets = Vec::with_capacity(manifest.datasets.len());
    for (index, spec) in manifest.datasets.iter().enumerate() {
        let supplied_path = PathBuf::from(&spec.polar_file);
        let resolved_path = if supplied_path.is_absolute() {
            supplied_path
        } else {
            manifest_directory.join(supplied_path)
        };
        let polar_text = fs::read_to_string(&resolved_path).map_err(|source| {
            XfoilCampaignAppError::PolarRead {
                index,
                dataset_id: spec.dataset_id.clone(),
                polar_file: spec.polar_file.clone(),
                resolved_path: resolved_path.clone(),
                source,
            }
        })?;

        let mut metadata = MetadataBuilder::new(spec.reynolds, spec.mach);
        if let Some(value) = &spec.solver_name {
            metadata = metadata.solver_name(value);
        }
        if let Some(value) = &spec.solver_version {
            metadata = metadata.solver_version(value);
        }
        if let Some(value) = &spec.command_or_config {
            metadata = metadata.command_or_config(value);
        }
        if let Some(value) = &spec.transition_assumptions {
            metadata = metadata.transition_assumptions(value);
        }
        if let Some(value) = spec.ncrit {
            metadata = metadata.ncrit(value);
        }
        if let Some(value) = spec.forced_transition_upper_x_over_c {
            metadata = metadata.forced_transition_upper(value);
        }
        if let Some(value) = spec.forced_transition_lower_x_over_c {
            metadata = metadata.forced_transition_lower(value);
        }
        let metadata = metadata
            .build()
            .map_err(|source| XfoilCampaignAppError::Metadata {
                index,
                dataset_id: spec.dataset_id.clone(),
                polar_file: spec.polar_file.clone(),
                source,
            })?;
        let import = parse_xfoil_polar(&polar_text, metadata).map_err(|source| {
            XfoilCampaignAppError::XfoilParse {
                index,
                dataset_id: spec.dataset_id.clone(),
                polar_file: spec.polar_file.clone(),
                source,
            }
        })?;
        let mut builder = XfoilEvidenceDatasetBuilder::new(
            import,
            &spec.dataset_id,
            &spec.method_id,
            spec.convergence_status.into(),
            spec.source_ids.clone(),
        );
        if let Some(notes) = &spec.notes {
            builder = builder.notes(notes);
        }
        datasets.push(
            builder
                .build()
                .map_err(|source| XfoilCampaignAppError::EvidenceBridge {
                    index,
                    dataset_id: spec.dataset_id.clone(),
                    polar_file: spec.polar_file.clone(),
                    source,
                })?,
        );
    }

    let campaign = XfoilEvidenceCampaignBuilder::new(datasets)
        .build()
        .map_err(|source| XfoilCampaignAppError::CampaignBuild { source })?;
    let request = XfoilCampaignCoverageRequest::new(
        manifest.coverage_request.required_reynolds_min,
        manifest.coverage_request.required_reynolds_max,
        manifest.coverage_request.required_alpha_min_rad,
        manifest.coverage_request.required_alpha_max_rad,
        manifest.coverage_request.require_converged,
    )
    .map_err(|source| XfoilCampaignAppError::CoverageRequest { source })?;
    let coverage = campaign.audit_coverage(&request);
    let report = build_report(&manifest, &campaign, &coverage);

    let json = serde_json::to_vec_pretty(&report).map_err(|source| {
        XfoilCampaignAppError::SerializeReport {
            report_name: JSON_REPORT,
            source,
        }
    })?;
    let markdown = render_markdown(&report).into_bytes();
    let polar_datasets = serde_json::to_vec_pretty(&campaign.to_polar_datasets_json_value())
        .map_err(|source| XfoilCampaignAppError::SerializeReport {
            report_name: POLAR_DATASETS_REPORT,
            source,
        })?;

    fs::create_dir_all(&options.output_dir).map_err(|source| {
        XfoilCampaignAppError::CreateOutputDirectory {
            path: options.output_dir.clone(),
            source,
        }
    })?;
    for (name, bytes) in [
        (JSON_REPORT, json.as_slice()),
        (MARKDOWN_REPORT, markdown.as_slice()),
        (POLAR_DATASETS_REPORT, polar_datasets.as_slice()),
    ] {
        let path = options.output_dir.join(name);
        fs::write(&path, bytes)
            .map_err(|source| XfoilCampaignAppError::WriteReport { path, source })?;
    }

    Ok(if coverage.is_qualified() {
        XfoilCampaignRunStatus::Qualified
    } else {
        XfoilCampaignRunStatus::NotQualified
    })
}

fn build_report(
    manifest: &Manifest,
    campaign: &XfoilEvidenceCampaign,
    coverage: &XfoilCampaignCoverage,
) -> CampaignReport {
    let mut converged = 0;
    let mut unresolved = 0;
    let mut failed = 0;
    let datasets = coverage
        .datasets()
        .iter()
        .zip(campaign.datasets())
        .zip(&manifest.datasets)
        .map(|((row, evidence), spec)| {
            match row.convergence_status() {
                ConvergenceStatus::Converged => converged += 1,
                ConvergenceStatus::Unresolved => unresolved += 1,
                ConvergenceStatus::Failed => failed += 1,
                ConvergenceStatus::NotApplicablePublished => {
                    unreachable!("M2.9B rejects published convergence")
                }
            }
            DatasetReport {
                index: row.index(),
                dataset_id: row.dataset_id().to_owned(),
                method_id: row.method_id().to_owned(),
                polar_file: spec.polar_file.clone(),
                reynolds: row.reynolds(),
                mach: row.mach(),
                convergence_status: row.convergence_status(),
                sample_count: evidence.sample_count(),
                alpha_min_rad: row.alpha_min_rad(),
                alpha_max_rad: row.alpha_max_rad(),
                covers_required_alpha_min: row.covers_required_alpha_min(),
                covers_required_alpha_max: row.covers_required_alpha_max(),
            }
        })
        .collect();
    CampaignReport {
        schema_version: ReportSchemaVersion,
        generated_by: GENERATED_BY.to_owned(),
        campaign_id: manifest.campaign_id.clone(),
        manifest: ManifestReport {
            schema_version: manifest.schema_version,
        },
        summary: SummaryReport {
            dataset_count: campaign.dataset_count(),
            converged_dataset_count: converged,
            unresolved_dataset_count: unresolved,
            failed_dataset_count: failed,
            blocker_count: coverage.blockers().len(),
            qualified: coverage.is_qualified(),
        },
        coverage_request: CoverageRequestReport {
            required_reynolds_min: coverage.request().required_reynolds_min(),
            required_reynolds_max: coverage.request().required_reynolds_max(),
            required_alpha_min_rad: coverage.request().required_alpha_min_rad(),
            required_alpha_max_rad: coverage.request().required_alpha_max_rad(),
            require_converged: coverage.request().require_converged(),
        },
        campaign_reynolds_range: ReynoldsRangeReport {
            minimum_reynolds: coverage.campaign_minimum_reynolds(),
            maximum_reynolds: coverage.campaign_maximum_reynolds(),
        },
        datasets,
        blockers: coverage.blockers().iter().map(Into::into).collect(),
        status: match coverage.status() {
            XfoilCampaignCoverageStatus::Qualified => ReportStatus::Qualified,
            XfoilCampaignCoverageStatus::NotQualified => ReportStatus::NotQualified,
        },
    }
}

fn render_markdown(report: &CampaignReport) -> String {
    let status = match report.status {
        ReportStatus::Qualified => "Qualified",
        ReportStatus::NotQualified => "Not Qualified",
    };
    let mut output = format!(
        "# XFOIL campaign coverage report\n\n\
         - Campaign ID: `{}`\n\
         - Status: **{status}**\n\
         - Requested Reynolds range: {} to {}\n\
         - Campaign Reynolds range: {} to {}\n\
         - Requested alpha range (rad): {} to {}\n\
         - Require converged datasets: {}\n\n\
         ## Summary\n\n\
         - Dataset count: {}\n\
         - Converged: {}\n\
         - Unresolved: {}\n\
         - Failed: {}\n\
         - Blockers: {}\n\n\
         ## Ordered datasets\n\n\
         | Index | Dataset | Re | Mach | Convergence | Samples | Alpha Min | Alpha Max | Lower Alpha Coverage | Upper Alpha Coverage |\n\
         | ---: | --- | ---: | ---: | --- | ---: | ---: | ---: | --- | --- |\n",
        markdown_text(&report.campaign_id),
        report.coverage_request.required_reynolds_min,
        report.coverage_request.required_reynolds_max,
        report.campaign_reynolds_range.minimum_reynolds,
        report.campaign_reynolds_range.maximum_reynolds,
        report.coverage_request.required_alpha_min_rad,
        report.coverage_request.required_alpha_max_rad,
        report.coverage_request.require_converged,
        report.summary.dataset_count,
        report.summary.converged_dataset_count,
        report.summary.unresolved_dataset_count,
        report.summary.failed_dataset_count,
        report.summary.blocker_count,
    );
    for row in &report.datasets {
        output.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            row.index,
            markdown_text(&row.dataset_id),
            row.reynolds,
            row.mach,
            convergence_label(row.convergence_status),
            row.sample_count,
            row.alpha_min_rad,
            row.alpha_max_rad,
            yes_no(row.covers_required_alpha_min),
            yes_no(row.covers_required_alpha_max),
        ));
    }
    output.push_str("\n## Ordered blockers\n\n");
    if report.blockers.is_empty() {
        output.push_str("None.\n");
    } else {
        for (index, blocker) in report.blockers.iter().enumerate() {
            output.push_str(&format!("{}. {}\n", index + 1, blocker_markdown(blocker)));
        }
    }
    output.push_str(
        "\n> **Scope disclaimer:** Coverage qualification only proves the requested evidence-domain rules implemented by M2.9C; it does not prove solver correctness, airfoil fidelity, aircraft fidelity, or runtime readiness.\n",
    );
    output
}

fn blocker_markdown(blocker: &BlockerReport) -> String {
    match blocker {
        BlockerReport::ReynoldsCoverageBelowRequired {
            campaign_minimum_reynolds,
            required_minimum_reynolds,
        } => format!(
            "`reynolds_coverage_below_required`: campaign minimum {campaign_minimum_reynolds}, required minimum {required_minimum_reynolds}"
        ),
        BlockerReport::ReynoldsCoverageAboveRequired {
            campaign_maximum_reynolds,
            required_maximum_reynolds,
        } => format!(
            "`reynolds_coverage_above_required`: campaign maximum {campaign_maximum_reynolds}, required maximum {required_maximum_reynolds}"
        ),
        BlockerReport::DatasetNotConverged {
            index,
            dataset_id,
            status,
        } => format!(
            "`dataset_not_converged`: dataset index {index}, ID `{}`, status `{}`",
            markdown_text(dataset_id),
            convergence_label(*status)
        ),
        BlockerReport::DatasetAlphaBelowRequired {
            index,
            dataset_id,
            dataset_alpha_min_rad,
            required_alpha_min_rad,
        } => format!(
            "`dataset_alpha_below_required`: dataset index {index}, ID `{}`, minimum alpha {dataset_alpha_min_rad} rad, required minimum {required_alpha_min_rad} rad",
            markdown_text(dataset_id)
        ),
        BlockerReport::DatasetAlphaAboveRequired {
            index,
            dataset_id,
            dataset_alpha_max_rad,
            required_alpha_max_rad,
        } => format!(
            "`dataset_alpha_above_required`: dataset index {index}, ID `{}`, maximum alpha {dataset_alpha_max_rad} rad, required maximum {required_alpha_max_rad} rad",
            markdown_text(dataset_id)
        ),
    }
}

fn markdown_text(value: &str) -> String {
    value.replace('|', "\\|").replace(['\r', '\n'], " ")
}

const fn convergence_label(status: ConvergenceStatus) -> &'static str {
    match status {
        ConvergenceStatus::Converged => "converged",
        ConvergenceStatus::Unresolved => "unresolved",
        ConvergenceStatus::Failed => "failed",
        ConvergenceStatus::NotApplicablePublished => "not_applicable_published",
    }
}

const fn yes_no(value: bool) -> &'static str {
    if value { "yes" } else { "no" }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_report() -> CampaignReport {
        CampaignReport {
            schema_version: ReportSchemaVersion,
            generated_by: GENERATED_BY.to_owned(),
            campaign_id: "synthetic-campaign".to_owned(),
            manifest: ManifestReport { schema_version: 1 },
            summary: SummaryReport {
                dataset_count: 0,
                converged_dataset_count: 0,
                unresolved_dataset_count: 0,
                failed_dataset_count: 0,
                blocker_count: 0,
                qualified: true,
            },
            coverage_request: CoverageRequestReport {
                required_reynolds_min: 1.0,
                required_reynolds_max: 2.0,
                required_alpha_min_rad: -0.1,
                required_alpha_max_rad: 0.1,
                require_converged: true,
            },
            campaign_reynolds_range: ReynoldsRangeReport {
                minimum_reynolds: 1.0,
                maximum_reynolds: 2.0,
            },
            datasets: Vec::new(),
            blockers: Vec::new(),
            status: ReportStatus::Qualified,
        }
    }

    #[test]
    fn report_round_trips() {
        let report = sample_report();
        let decoded = serde_json::from_slice(&serde_json::to_vec(&report).unwrap()).unwrap();
        assert_eq!(report, decoded);
    }

    #[test]
    fn report_rejects_unsupported_versions_and_unknown_fields() {
        let mut value = serde_json::to_value(sample_report()).unwrap();
        for version in [0, 2, 999] {
            value["schema_version"] = serde_json::json!(version);
            assert!(serde_json::from_value::<CampaignReport>(value.clone()).is_err());
        }
        value["schema_version"] = serde_json::json!(1);
        value["future_field"] = serde_json::json!(true);
        assert!(serde_json::from_value::<CampaignReport>(value).is_err());
    }
}
