//! Strict loading and deterministic evaluation of off-runtime electric-propulsion evidence.
//!
//! M2.4A deliberately cannot construct runtime propulsion configuration. Manufacturer
//! recommendations, historical configurations, and physically identified installations remain
//! distinct evidence claims, and [`PropulsionEvidenceEvaluation::runtime_ready`] is always false.

use std::{
    collections::{HashMap, HashSet},
    fs,
    path::{Component, Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{REFERENCE_PROPULSION_EVIDENCE_SCHEMA_V0, SurveyClassification};

const ARTIFACT_KIND: &str = "propulsion_evidence_not_runtime_configuration";
const DIMENSION_TOLERANCE_M: f64 = 1.0e-12;

#[derive(Debug, Error)]
pub enum ReferencePropulsionEvidenceError {
    #[error("failed to read propulsion evidence {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("propulsion evidence JSON has invalid structure: {source}")]
    InvalidStructure {
        #[source]
        source: serde_json::Error,
    },
    #[error("unsupported propulsion evidence schema {found:?}")]
    UnsupportedSchema { found: String },
    #[error("invalid propulsion evidence artifact kind {found:?}")]
    InvalidArtifactKind { found: String },
    #[error("invalid stable {kind} ID {value:?}; expected nonempty [a-z0-9_-]+")]
    InvalidStableId { kind: &'static str, value: String },
    #[error("duplicate stable {kind} ID {value:?}")]
    DuplicateStableId { kind: &'static str, value: String },
    #[error("{field} references unknown {kind} {reference_id:?}")]
    UnresolvedReference {
        field: String,
        kind: &'static str,
        reference_id: String,
    },
    #[error("{field} contains duplicate reference {reference_id:?}")]
    DuplicateReference { field: String, reference_id: String },
    #[error("invalid propulsion evidence {field}: {reason}")]
    InvalidEvidence { field: String, reason: &'static str },
    #[error("incompatible propulsion configuration identity in claim {claim_id:?}: {reason}")]
    IncompatibleConfigurationIdentity {
        claim_id: String,
        reason: &'static str,
    },
    #[error("malformed APC performance data at line {line}: {reason}")]
    MalformedApcData { line: usize, reason: &'static str },
    #[error("linked APC data {dataset_id:?} does not match declared {field}")]
    LinkedDatasetMismatch {
        dataset_id: String,
        field: &'static str,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropulsionConfigurationEvidenceClass {
    ManufacturerRecommendation,
    HistoricallyFlightTestedConfiguration,
    SpecificInstalledConfiguration,
    MeasuredConfiguration,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct ConfigurationClaimSummary {
    id: String,
    evidence_class: PropulsionConfigurationEvidenceClass,
    physical_airframe_id: Option<String>,
    operational_configuration_id: Option<String>,
    propulsion_configuration_id: Option<String>,
}

impl ConfigurationClaimSummary {
    pub fn id(&self) -> &str {
        &self.id
    }
    pub const fn evidence_class(&self) -> PropulsionConfigurationEvidenceClass {
        self.evidence_class
    }
    pub fn physical_airframe_id(&self) -> Option<&str> {
        self.physical_airframe_id.as_deref()
    }
    pub fn operational_configuration_id(&self) -> Option<&str> {
        self.operational_configuration_id.as_deref()
    }
    pub fn propulsion_configuration_id(&self) -> Option<&str> {
        self.propulsion_configuration_id.as_deref()
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct PropulsionEvidenceEvaluation {
    configuration_claims: Vec<ConfigurationClaimSummary>,
    blockers: Vec<String>,
    motor_evidence_ready: bool,
    esc_evidence_ready: bool,
    battery_evidence_ready: bool,
    propeller_evidence_ready: bool,
    configuration_identified: bool,
    propulsion_evidence_ready: bool,
    runtime_ready: bool,
}

impl PropulsionEvidenceEvaluation {
    pub fn configuration_claims(&self) -> &[ConfigurationClaimSummary] {
        &self.configuration_claims
    }
    pub fn blockers(&self) -> &[String] {
        &self.blockers
    }
    pub const fn motor_evidence_ready(&self) -> bool {
        self.motor_evidence_ready
    }
    pub const fn esc_evidence_ready(&self) -> bool {
        self.esc_evidence_ready
    }
    pub const fn battery_evidence_ready(&self) -> bool {
        self.battery_evidence_ready
    }
    pub const fn propeller_evidence_ready(&self) -> bool {
        self.propeller_evidence_ready
    }
    pub const fn configuration_identified(&self) -> bool {
        self.configuration_identified
    }
    pub const fn propulsion_evidence_ready(&self) -> bool {
        self.propulsion_evidence_ready
    }
    /// Evidence is never promoted into runtime configuration in M2.4A.
    pub const fn runtime_ready(&self) -> bool {
        self.runtime_ready
    }
}

#[derive(Debug, Clone)]
pub struct PropulsionEvidence {
    file: PropulsionEvidenceFile,
    apc_datasets: HashMap<String, ApcPerformanceData>,
    evaluation: PropulsionEvidenceEvaluation,
}

impl PropulsionEvidence {
    pub fn campaign_id(&self) -> &str {
        &self.file.campaign.id
    }
    pub const fn classification(&self) -> SurveyClassification {
        self.file.campaign.classification
    }
    pub const fn evaluation(&self) -> &PropulsionEvidenceEvaluation {
        &self.evaluation
    }
    pub fn apc_dataset(&self, id: &str) -> Option<&ApcPerformanceData> {
        self.apc_datasets.get(id)
    }
}

pub struct PropulsionEvidenceLoader;

impl PropulsionEvidenceLoader {
    pub fn from_json_str(
        json: &str,
    ) -> Result<PropulsionEvidence, ReferencePropulsionEvidenceError> {
        let file: PropulsionEvidenceFile = serde_json::from_str(json)
            .map_err(|source| ReferencePropulsionEvidenceError::InvalidStructure { source })?;
        validate_file(&file)?;
        let apc_datasets = HashMap::new();
        let evaluation = evaluate(&file, &apc_datasets);
        Ok(PropulsionEvidence {
            file,
            apc_datasets,
            evaluation,
        })
    }
}

pub fn load_reference_propulsion_evidence(
    path: impl AsRef<Path>,
) -> Result<PropulsionEvidence, ReferencePropulsionEvidenceError> {
    let path = path.as_ref();
    let json = fs::read_to_string(path).map_err(|source| ReferencePropulsionEvidenceError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mut evidence = PropulsionEvidenceLoader::from_json_str(&json)?;
    let base = path.parent().unwrap_or_else(|| Path::new("."));
    for dataset in &evidence.file.propeller_datasets {
        let source_path = base.join(&dataset.raw_source_path);
        let bytes =
            fs::read(&source_path).map_err(|source| ReferencePropulsionEvidenceError::Io {
                path: source_path.clone(),
                source,
            })?;
        let calculated_sha256 = sha256_hex(&bytes);
        if !calculated_sha256.eq_ignore_ascii_case(&dataset.sha256) {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "sha256",
            });
        }
        if bytes.len() != dataset.byte_count {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "byte_count",
            });
        }
        let raw = std::str::from_utf8(&bytes).map_err(|_| {
            ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "utf8_encoding",
            }
        })?;
        let line_count = raw.lines().count();
        if line_count != dataset.line_count {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "line_count",
            });
        }
        let parsed = ApcPerformanceDataLoader::parse_str(raw)?;
        if parsed.source_version() != dataset.source_version {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "source_version",
            });
        }
        if parsed.simulation_date() != dataset.simulation_date {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "simulation_date",
            });
        }
        if parsed.blocks().len() != dataset.rpm_block_count {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "rpm_block_count",
            });
        }
        if parsed.row_count() != dataset.row_count {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "row_count",
            });
        }
        if parsed.coefficient_row_count() != dataset.coefficient_row_count {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "coefficient_row_count",
            });
        }
        evidence.apc_datasets.insert(dataset.id.clone(), parsed);
    }
    evidence.evaluation = evaluate(&evidence.file, &evidence.apc_datasets);
    Ok(evidence)
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ApcPerformanceRow {
    speed_mph: f64,
    advance_ratio_j: f64,
    efficiency: Option<f64>,
    ct: Option<f64>,
    cp: Option<f64>,
}

impl ApcPerformanceRow {
    pub const fn speed_mph(&self) -> f64 {
        self.speed_mph
    }
    pub const fn advance_ratio_j(&self) -> f64 {
        self.advance_ratio_j
    }
    pub const fn efficiency(&self) -> Option<f64> {
        self.efficiency
    }
    pub const fn ct(&self) -> Option<f64> {
        self.ct
    }
    pub const fn cp(&self) -> Option<f64> {
        self.cp
    }
    /// Torque coefficient deterministically derived from APC's published power coefficient.
    pub fn cq_derived(&self) -> Option<f64> {
        self.cp.map(|cp| cp / (2.0 * std::f64::consts::PI))
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ApcRpmBlock {
    rpm: f64,
    rows: Vec<ApcPerformanceRow>,
}

impl ApcRpmBlock {
    pub const fn rpm(&self) -> f64 {
        self.rpm
    }
    pub fn rows(&self) -> &[ApcPerformanceRow] {
        &self.rows
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ApcPerformanceData {
    propeller_designation: String,
    source_version: String,
    simulation_date: String,
    blocks: Vec<ApcRpmBlock>,
}

impl ApcPerformanceData {
    pub fn propeller_designation(&self) -> &str {
        &self.propeller_designation
    }
    pub fn source_version(&self) -> &str {
        &self.source_version
    }
    pub fn simulation_date(&self) -> &str {
        &self.simulation_date
    }
    pub fn blocks(&self) -> &[ApcRpmBlock] {
        &self.blocks
    }
    pub fn row_count(&self) -> usize {
        self.blocks.iter().map(|block| block.rows.len()).sum()
    }
    pub fn coefficient_row_count(&self) -> usize {
        self.blocks
            .iter()
            .flat_map(|block| &block.rows)
            .filter(|row| row.cp.is_some())
            .count()
    }
}

pub struct ApcPerformanceDataLoader;

impl ApcPerformanceDataLoader {
    pub fn parse_str(raw: &str) -> Result<ApcPerformanceData, ReferencePropulsionEvidenceError> {
        parse_apc_performance_data(raw)
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PropulsionEvidenceFile {
    schema: String,
    artifact_kind: String,
    campaign: CampaignFile,
    provenance_sources: Vec<SourceFile>,
    photographs: Vec<PhotographFile>,
    configuration_claims: Vec<ConfigurationClaimFile>,
    motors: Vec<MotorEvidenceFile>,
    escs: Vec<EscEvidenceFile>,
    batteries: Vec<BatteryEvidenceFile>,
    propellers: Vec<PropellerEvidenceFile>,
    spinners: Vec<SpinnerEvidenceFile>,
    propeller_datasets: Vec<PropellerDatasetFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignFile {
    id: String,
    classification: SurveyClassification,
    manufacturer: String,
    family: String,
    variant: String,
    physical_airframe_id: Option<String>,
    operational_configuration_id: Option<String>,
    propulsion_configuration_id: Option<String>,
    measurement_date: Option<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SourceKind {
    ManufacturerDocumentation,
    ManufacturerData,
    PhysicalMeasurement,
    Photograph,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourceFile {
    id: String,
    kind: SourceKind,
    title: String,
    publisher: Option<String>,
    url: Option<String>,
    retrieval_date: Option<String>,
    sha256: Option<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PhotographFile {
    id: String,
    path: String,
    captured_on: Option<String>,
    description: String,
    source_ids: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ConfigurationClaimFile {
    id: String,
    evidence_class: PropulsionConfigurationEvidenceClass,
    physical_airframe_id: Option<String>,
    operational_configuration_id: Option<String>,
    propulsion_configuration_id: Option<String>,
    measurement_date: Option<String>,
    motor_id: Option<String>,
    esc_id: Option<String>,
    battery_id: Option<String>,
    propeller_id: Option<String>,
    spinner_id: Option<String>,
    recommendation: Option<RecommendationEnvelopeFile>,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RecommendationEnvelopeFile {
    electric_power_w: Option<[f64; 2]>,
    motor_kv_rpm_per_v: Option<[f64; 2]>,
    esc_current_a: Option<[f64; 2]>,
    battery_cell_count: Option<[u32; 2]>,
    battery_capacity_ah: Option<[f64; 2]>,
    battery_chemistry: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ComponentEvidenceClass {
    ManufacturerData,
    MeasuredData,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MotorEvidenceFile {
    id: String,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<String>,
    model: Option<String>,
    kv_rpm_per_v: Option<f64>,
    winding_resistance_ohm: Option<f64>,
    no_load_current_a: Option<f64>,
    mass_kg: Option<f64>,
    diameter_m: Option<f64>,
    length_m: Option<f64>,
    shaft_diameter_m: Option<f64>,
    maximum_current_a: Option<f64>,
    maximum_current_duration_s: Option<f64>,
    maximum_power_w: Option<f64>,
    efficient_current_range_a: Option<[f64; 2]>,
    efficiency: Option<f64>,
    applicable_configuration_claim_ids: Vec<String>,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct EscEvidenceFile {
    id: String,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<String>,
    model: Option<String>,
    current_rating_a: Option<f64>,
    minimum_cell_count: Option<u32>,
    maximum_cell_count: Option<u32>,
    resistance_ohm: Option<f64>,
    efficiency: Option<f64>,
    switching_frequency_hz: Option<f64>,
    control_protocol: Option<String>,
    applicable_configuration_claim_ids: Vec<String>,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct BatteryEvidenceFile {
    id: String,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<String>,
    model: Option<String>,
    chemistry: Option<String>,
    cell_count: Option<u32>,
    capacity_ah: Option<f64>,
    nominal_voltage_v: Option<f64>,
    mass_kg: Option<f64>,
    internal_resistance_ohm: Option<f64>,
    voltage_load_points: Vec<BatteryLoadPointFile>,
    applicable_configuration_claim_ids: Vec<String>,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct BatteryLoadPointFile {
    state_of_charge: f64,
    load_current_a: f64,
    voltage_v: f64,
    temperature_c: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PropellerEvidenceFile {
    id: String,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<String>,
    model: Option<String>,
    diameter_m: Option<f64>,
    pitch_m: Option<f64>,
    dataset_ids: Vec<String>,
    applicable_configuration_claim_ids: Vec<String>,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SpinnerEvidenceFile {
    id: String,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<String>,
    model: Option<String>,
    diameter_m: Option<f64>,
    mass_kg: Option<f64>,
    applicable_configuration_claim_ids: Vec<String>,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PropellerDatasetFile {
    id: String,
    propeller_id: String,
    evidence_class: ComponentEvidenceClass,
    source_id: String,
    raw_source_path: String,
    sha256: String,
    byte_count: usize,
    line_count: usize,
    source_format: String,
    source_version: String,
    simulation_date: String,
    diameter_m: f64,
    pitch_m: f64,
    rpm_block_count: usize,
    row_count: usize,
    coefficient_row_count: usize,
    parser_interpretation: ParserInterpretationFile,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ParserInterpretationFile {
    rpm: String,
    advance_ratio_j: String,
    ct: String,
    cp: String,
    efficiency: String,
    cq: String,
}

fn parse_apc_performance_data(
    raw: &str,
) -> Result<ApcPerformanceData, ReferencePropulsionEvidenceError> {
    let mut designation = None;
    let mut version = None;
    let mut simulation_date = None;
    let mut definitions = [false; 3];
    let mut blocks = Vec::new();
    let mut current: Option<ApcRpmBlock> = None;

    for (index, line) in raw.lines().enumerate() {
        let line_number = index + 1;
        let trimmed = line.trim();
        if designation.is_none() && !trimmed.is_empty() {
            designation = trimmed.split_whitespace().next().map(str::to_owned);
        }
        if version.is_none() && trimmed.starts_with('v') {
            version = Some(trimmed.to_owned());
        }
        if let Some(date) = trimmed.strip_prefix("Simulation Date:") {
            simulation_date = Some(date.trim().to_owned());
        }
        definitions[0] |= trimmed.starts_with("J=V/nD");
        definitions[1] |= trimmed.starts_with("Ct=T/");
        definitions[2] |= trimmed.starts_with("Cp=P/");

        if let Some(value) = trimmed.strip_prefix("PROP RPM =") {
            if let Some(block) = current.take() {
                finish_apc_block(block, &mut blocks, line_number)?;
            }
            let rpm = value.trim().parse::<f64>().map_err(|_| {
                ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "RPM marker is not numeric",
                }
            })?;
            if !rpm.is_finite() || rpm <= 0.0 {
                return Err(ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "RPM must be finite and positive",
                });
            }
            current = Some(ApcRpmBlock {
                rpm,
                rows: Vec::new(),
            });
            continue;
        }

        let starts_numeric = trimmed
            .as_bytes()
            .first()
            .is_some_and(|byte| byte.is_ascii_digit() || matches!(byte, b'-' | b'+' | b'.'));
        if starts_numeric {
            let Some(block) = current.as_mut() else {
                continue;
            };
            let values = trimmed
                .split_whitespace()
                .map(|token| token.parse::<f64>())
                .collect::<Result<Vec<_>, _>>()
                .map_err(|_| ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "numeric row contains a non-numeric field",
                })?;
            if !matches!(values.len(), 2 | 15) {
                return Err(ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "numeric row must contain V/J only or all 15 published APC columns",
                });
            }
            if values.iter().any(|value| !value.is_finite()) {
                return Err(ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "numeric row must contain only finite values",
                });
            }
            if values[0] < 0.0 || values[1] < 0.0 || (values.len() == 15 && values[4] <= 0.0) {
                return Err(ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "speed and J must be non-negative and Cp must be positive",
                });
            }
            if block
                .rows
                .last()
                .is_some_and(|previous| values[1] <= previous.advance_ratio_j)
            {
                return Err(ReferencePropulsionEvidenceError::MalformedApcData {
                    line: line_number,
                    reason: "advance ratio J must be strictly increasing within an RPM block",
                });
            }
            block.rows.push(ApcPerformanceRow {
                speed_mph: values[0],
                advance_ratio_j: values[1],
                efficiency: values.get(2).copied(),
                ct: values.get(3).copied(),
                cp: values.get(4).copied(),
            });
        }
    }
    if let Some(block) = current {
        finish_apc_block(block, &mut blocks, raw.lines().count())?;
    }
    if definitions.iter().any(|found| !found) {
        return Err(ReferencePropulsionEvidenceError::MalformedApcData {
            line: 1,
            reason: "APC J, Ct, and Cp definitions must be present",
        });
    }
    if blocks.is_empty() {
        return Err(ReferencePropulsionEvidenceError::MalformedApcData {
            line: 1,
            reason: "no RPM blocks found",
        });
    }
    Ok(ApcPerformanceData {
        propeller_designation: designation.ok_or(
            ReferencePropulsionEvidenceError::MalformedApcData {
                line: 1,
                reason: "missing propeller designation",
            },
        )?,
        source_version: version.ok_or(ReferencePropulsionEvidenceError::MalformedApcData {
            line: 1,
            reason: "missing APC source version",
        })?,
        simulation_date: simulation_date.ok_or(
            ReferencePropulsionEvidenceError::MalformedApcData {
                line: 1,
                reason: "missing simulation date",
            },
        )?,
        blocks,
    })
}

fn finish_apc_block(
    block: ApcRpmBlock,
    blocks: &mut Vec<ApcRpmBlock>,
    line: usize,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if block.rows.is_empty() {
        return Err(ReferencePropulsionEvidenceError::MalformedApcData {
            line,
            reason: "RPM block has no numeric rows",
        });
    }
    if blocks
        .last()
        .is_some_and(|previous| block.rpm <= previous.rpm)
    {
        return Err(ReferencePropulsionEvidenceError::MalformedApcData {
            line,
            reason: "RPM blocks must be strictly increasing",
        });
    }
    blocks.push(block);
    Ok(())
}

fn validate_file(file: &PropulsionEvidenceFile) -> Result<(), ReferencePropulsionEvidenceError> {
    if file.schema != REFERENCE_PROPULSION_EVIDENCE_SCHEMA_V0 {
        return Err(ReferencePropulsionEvidenceError::UnsupportedSchema {
            found: file.schema.clone(),
        });
    }
    if file.artifact_kind != ARTIFACT_KIND {
        return Err(ReferencePropulsionEvidenceError::InvalidArtifactKind {
            found: file.artifact_kind.clone(),
        });
    }
    validate_campaign(&file.campaign)?;
    let source_ids = validate_sources(&file.provenance_sources)?;
    let photo_ids = validate_photographs(&file.photographs, &source_ids)?;
    let claim_ids = validate_claim_ids(&file.configuration_claims)?;
    let motor_ids = component_ids("motor", file.motors.iter().map(|item| &item.id))?;
    let esc_ids = component_ids("ESC", file.escs.iter().map(|item| &item.id))?;
    let battery_ids = component_ids("battery", file.batteries.iter().map(|item| &item.id))?;
    let propeller_ids = component_ids("propeller", file.propellers.iter().map(|item| &item.id))?;
    let spinner_ids = component_ids("spinner", file.spinners.iter().map(|item| &item.id))?;
    validate_claims(
        file,
        &source_ids,
        &photo_ids,
        &motor_ids,
        &esc_ids,
        &battery_ids,
        &propeller_ids,
        &spinner_ids,
    )?;
    validate_motors(
        &file.motors,
        &claim_ids,
        &source_ids,
        &file.configuration_claims,
    )?;
    validate_escs(
        &file.escs,
        &claim_ids,
        &source_ids,
        &file.configuration_claims,
    )?;
    validate_batteries(
        &file.batteries,
        &claim_ids,
        &source_ids,
        &file.configuration_claims,
    )?;
    validate_spinners(
        &file.spinners,
        &claim_ids,
        &source_ids,
        &file.configuration_claims,
    )?;
    validate_propellers(
        &file.propellers,
        &claim_ids,
        &source_ids,
        &file.configuration_claims,
    )?;
    validate_datasets(file, &source_ids, &propeller_ids)
}

fn validate_campaign(campaign: &CampaignFile) -> Result<(), ReferencePropulsionEvidenceError> {
    validate_stable_id("campaign", &campaign.id)?;
    validate_required_text("campaign.manufacturer", &campaign.manufacturer)?;
    validate_required_text("campaign.family", &campaign.family)?;
    validate_required_text("campaign.variant", &campaign.variant)?;
    validate_optional_stable_id(
        "physical airframe",
        campaign.physical_airframe_id.as_deref(),
    )?;
    validate_optional_stable_id(
        "operational configuration",
        campaign.operational_configuration_id.as_deref(),
    )?;
    validate_optional_stable_id(
        "propulsion configuration",
        campaign.propulsion_configuration_id.as_deref(),
    )?;
    validate_optional_date(
        "campaign.measurement_date",
        campaign.measurement_date.as_deref(),
    )?;
    validate_optional_text("campaign.notes", campaign.notes.as_deref())
}

fn validate_sources(
    sources: &[SourceFile],
) -> Result<HashSet<String>, ReferencePropulsionEvidenceError> {
    let mut ids = HashSet::new();
    for source in sources {
        validate_stable_id("provenance source", &source.id)?;
        insert_unique(&mut ids, "provenance source", &source.id)?;
        validate_required_text("provenance_sources.title", &source.title)?;
        validate_optional_text("provenance_sources.publisher", source.publisher.as_deref())?;
        validate_optional_text("provenance_sources.url", source.url.as_deref())?;
        validate_optional_date(
            "provenance_sources.retrieval_date",
            source.retrieval_date.as_deref(),
        )?;
        validate_sha256(source.sha256.as_deref(), "provenance_sources.sha256")?;
        validate_optional_text("provenance_sources.notes", source.notes.as_deref())?;
        let _ = source.kind;
    }
    Ok(ids)
}

fn validate_photographs(
    photographs: &[PhotographFile],
    source_ids: &HashSet<String>,
) -> Result<HashSet<String>, ReferencePropulsionEvidenceError> {
    let mut ids = HashSet::new();
    for photo in photographs {
        validate_stable_id("photograph", &photo.id)?;
        insert_unique(&mut ids, "photograph", &photo.id)?;
        validate_required_text("photographs.path", &photo.path)?;
        validate_required_text("photographs.description", &photo.description)?;
        validate_optional_date("photographs.captured_on", photo.captured_on.as_deref())?;
        validate_refs(
            "photograph.source_ids",
            &photo.source_ids,
            source_ids,
            "source",
        )?;
    }
    Ok(ids)
}

fn validate_claim_ids(
    claims: &[ConfigurationClaimFile],
) -> Result<HashSet<String>, ReferencePropulsionEvidenceError> {
    component_ids("configuration claim", claims.iter().map(|claim| &claim.id))
}

#[allow(clippy::too_many_arguments)]
fn validate_claims(
    file: &PropulsionEvidenceFile,
    source_ids: &HashSet<String>,
    photo_ids: &HashSet<String>,
    motor_ids: &HashSet<String>,
    esc_ids: &HashSet<String>,
    battery_ids: &HashSet<String>,
    propeller_ids: &HashSet<String>,
    spinner_ids: &HashSet<String>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    for claim in &file.configuration_claims {
        validate_optional_stable_id("physical airframe", claim.physical_airframe_id.as_deref())?;
        validate_optional_stable_id(
            "operational configuration",
            claim.operational_configuration_id.as_deref(),
        )?;
        validate_optional_stable_id(
            "propulsion configuration",
            claim.propulsion_configuration_id.as_deref(),
        )?;
        validate_optional_date(
            "configuration_claim.measurement_date",
            claim.measurement_date.as_deref(),
        )?;
        validate_refs(
            "configuration_claim.source_ids",
            &claim.source_ids,
            source_ids,
            "source",
        )?;
        validate_refs(
            "configuration_claim.photograph_ids",
            &claim.photograph_ids,
            photo_ids,
            "photograph",
        )?;
        validate_optional_ref(
            "configuration_claim.motor_id",
            claim.motor_id.as_deref(),
            motor_ids,
            "motor",
        )?;
        validate_optional_ref(
            "configuration_claim.esc_id",
            claim.esc_id.as_deref(),
            esc_ids,
            "ESC",
        )?;
        validate_optional_ref(
            "configuration_claim.battery_id",
            claim.battery_id.as_deref(),
            battery_ids,
            "battery",
        )?;
        validate_optional_ref(
            "configuration_claim.propeller_id",
            claim.propeller_id.as_deref(),
            propeller_ids,
            "propeller",
        )?;
        validate_optional_ref(
            "configuration_claim.spinner_id",
            claim.spinner_id.as_deref(),
            spinner_ids,
            "spinner",
        )?;
        validate_optional_text("configuration_claim.notes", claim.notes.as_deref())?;

        match claim.evidence_class {
            PropulsionConfigurationEvidenceClass::ManufacturerRecommendation => {
                if claim.recommendation.is_none()
                    || claim.physical_airframe_id.is_some()
                    || claim.operational_configuration_id.is_some()
                    || claim.propulsion_configuration_id.is_some()
                    || claim.motor_id.is_some()
                    || claim.esc_id.is_some()
                    || claim.battery_id.is_some()
                    || claim.propeller_id.is_some()
                    || claim.spinner_id.is_some()
                {
                    return Err(
                        ReferencePropulsionEvidenceError::IncompatibleConfigurationIdentity {
                            claim_id: claim.id.clone(),
                            reason: "recommendation must be a type-level envelope, not a physical-airframe or installed-component claim",
                        },
                    );
                }
                validate_recommendation(claim.recommendation.as_ref().expect("checked"))?;
            }
            PropulsionConfigurationEvidenceClass::HistoricallyFlightTestedConfiguration => {
                if claim.recommendation.is_some() {
                    return Err(
                        ReferencePropulsionEvidenceError::IncompatibleConfigurationIdentity {
                            claim_id: claim.id.clone(),
                            reason: "historical configuration cannot contain a recommendation envelope",
                        },
                    );
                }
            }
            PropulsionConfigurationEvidenceClass::SpecificInstalledConfiguration
            | PropulsionConfigurationEvidenceClass::MeasuredConfiguration => {
                let complete = claim.physical_airframe_id.is_some()
                    && claim.physical_airframe_id == file.campaign.physical_airframe_id
                    && claim.operational_configuration_id.is_some()
                    && claim.operational_configuration_id
                        == file.campaign.operational_configuration_id
                    && claim.propulsion_configuration_id.is_some()
                    && claim.propulsion_configuration_id
                        == file.campaign.propulsion_configuration_id
                    && claim.motor_id.is_some()
                    && claim.esc_id.is_some()
                    && claim.battery_id.is_some()
                    && claim.propeller_id.is_some();
                if !complete {
                    return Err(
                        ReferencePropulsionEvidenceError::IncompatibleConfigurationIdentity {
                            claim_id: claim.id.clone(),
                            reason: "installed/measured claim must match the fully identified campaign and components",
                        },
                    );
                }
                if claim.recommendation.is_some() {
                    return Err(
                        ReferencePropulsionEvidenceError::IncompatibleConfigurationIdentity {
                            claim_id: claim.id.clone(),
                            reason: "installed/measured claim cannot contain a recommendation envelope",
                        },
                    );
                }
            }
        }
    }
    Ok(())
}

fn validate_recommendation(
    recommendation: &RecommendationEnvelopeFile,
) -> Result<(), ReferencePropulsionEvidenceError> {
    validate_positive_range(
        "recommendation.electric_power_w",
        recommendation.electric_power_w,
    )?;
    validate_positive_range(
        "recommendation.motor_kv_rpm_per_v",
        recommendation.motor_kv_rpm_per_v,
    )?;
    validate_positive_range("recommendation.esc_current_a", recommendation.esc_current_a)?;
    validate_positive_range(
        "recommendation.battery_capacity_ah",
        recommendation.battery_capacity_ah,
    )?;
    if let Some([minimum, maximum]) = recommendation.battery_cell_count
        && (minimum == 0 || minimum > maximum)
    {
        return invalid(
            "recommendation.battery_cell_count",
            "cell range must be positive and ordered",
        );
    }
    validate_optional_text(
        "recommendation.battery_chemistry",
        recommendation.battery_chemistry.as_deref(),
    )
}

fn validate_motors(
    motors: &[MotorEvidenceFile],
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
    claims: &[ConfigurationClaimFile],
) -> Result<(), ReferencePropulsionEvidenceError> {
    for motor in motors {
        validate_component_common(
            "motor",
            &motor.id,
            motor.evidence_class,
            motor.manufacturer.as_deref(),
            motor.model.as_deref(),
            &motor.applicable_configuration_claim_ids,
            &motor.source_ids,
            motor.notes.as_deref(),
            claim_ids,
            source_ids,
        )?;
        validate_applicability(
            &motor.id,
            "motor",
            &motor.applicable_configuration_claim_ids,
            claims,
            |claim| claim.motor_id.as_deref(),
        )?;
        for (field, value) in [
            ("motor.kv_rpm_per_v", motor.kv_rpm_per_v),
            ("motor.winding_resistance_ohm", motor.winding_resistance_ohm),
            ("motor.no_load_current_a", motor.no_load_current_a),
            ("motor.mass_kg", motor.mass_kg),
            ("motor.diameter_m", motor.diameter_m),
            ("motor.length_m", motor.length_m),
            ("motor.shaft_diameter_m", motor.shaft_diameter_m),
            ("motor.maximum_current_a", motor.maximum_current_a),
            (
                "motor.maximum_current_duration_s",
                motor.maximum_current_duration_s,
            ),
            ("motor.maximum_power_w", motor.maximum_power_w),
        ] {
            validate_optional_positive(field, value)?;
        }
        validate_positive_range(
            "motor.efficient_current_range_a",
            motor.efficient_current_range_a,
        )?;
        validate_optional_efficiency("motor.efficiency", motor.efficiency)?;
    }
    Ok(())
}

fn validate_escs(
    escs: &[EscEvidenceFile],
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
    claims: &[ConfigurationClaimFile],
) -> Result<(), ReferencePropulsionEvidenceError> {
    for esc in escs {
        validate_component_common(
            "ESC",
            &esc.id,
            esc.evidence_class,
            esc.manufacturer.as_deref(),
            esc.model.as_deref(),
            &esc.applicable_configuration_claim_ids,
            &esc.source_ids,
            esc.notes.as_deref(),
            claim_ids,
            source_ids,
        )?;
        validate_applicability(
            &esc.id,
            "ESC",
            &esc.applicable_configuration_claim_ids,
            claims,
            |claim| claim.esc_id.as_deref(),
        )?;
        validate_optional_positive("esc.current_rating_a", esc.current_rating_a)?;
        validate_optional_positive("esc.resistance_ohm", esc.resistance_ohm)?;
        validate_optional_positive("esc.switching_frequency_hz", esc.switching_frequency_hz)?;
        validate_optional_efficiency("esc.efficiency", esc.efficiency)?;
        if let Some(minimum) = esc.minimum_cell_count
            && minimum == 0
        {
            return invalid("esc.minimum_cell_count", "cell count must be positive");
        }
        if let Some(maximum) = esc.maximum_cell_count
            && (maximum == 0
                || esc
                    .minimum_cell_count
                    .is_some_and(|minimum| minimum > maximum))
        {
            return invalid(
                "esc.maximum_cell_count",
                "cell count must be positive and ordered",
            );
        }
        validate_optional_text("esc.control_protocol", esc.control_protocol.as_deref())?;
    }
    Ok(())
}

fn validate_batteries(
    batteries: &[BatteryEvidenceFile],
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
    claims: &[ConfigurationClaimFile],
) -> Result<(), ReferencePropulsionEvidenceError> {
    for battery in batteries {
        validate_component_common(
            "battery",
            &battery.id,
            battery.evidence_class,
            battery.manufacturer.as_deref(),
            battery.model.as_deref(),
            &battery.applicable_configuration_claim_ids,
            &battery.source_ids,
            battery.notes.as_deref(),
            claim_ids,
            source_ids,
        )?;
        validate_applicability(
            &battery.id,
            "battery",
            &battery.applicable_configuration_claim_ids,
            claims,
            |claim| claim.battery_id.as_deref(),
        )?;
        validate_optional_text("battery.chemistry", battery.chemistry.as_deref())?;
        if battery.cell_count.is_some_and(|count| count == 0) {
            return invalid("battery.cell_count", "cell count must be positive");
        }
        for (field, value) in [
            ("battery.capacity_ah", battery.capacity_ah),
            ("battery.nominal_voltage_v", battery.nominal_voltage_v),
            ("battery.mass_kg", battery.mass_kg),
            (
                "battery.internal_resistance_ohm",
                battery.internal_resistance_ohm,
            ),
        ] {
            validate_optional_positive(field, value)?;
        }
        let mut seen = HashSet::new();
        for point in &battery.voltage_load_points {
            if !point.state_of_charge.is_finite()
                || !(0.0..=1.0).contains(&point.state_of_charge)
                || !point.load_current_a.is_finite()
                || point.load_current_a < 0.0
                || !point.voltage_v.is_finite()
                || point.voltage_v <= 0.0
                || !point.temperature_c.is_finite()
            {
                return invalid(
                    "battery.voltage_load_points",
                    "invalid SOC/load/voltage/temperature point",
                );
            }
            let key = (
                canonical_bits(point.state_of_charge),
                canonical_bits(point.load_current_a),
                canonical_bits(point.temperature_c),
            );
            if !seen.insert(key) {
                return invalid(
                    "battery.voltage_load_points",
                    "duplicate SOC/load/temperature point",
                );
            }
        }
    }
    Ok(())
}

fn validate_propellers(
    propellers: &[PropellerEvidenceFile],
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
    claims: &[ConfigurationClaimFile],
) -> Result<(), ReferencePropulsionEvidenceError> {
    for propeller in propellers {
        validate_component_common(
            "propeller",
            &propeller.id,
            propeller.evidence_class,
            propeller.manufacturer.as_deref(),
            propeller.model.as_deref(),
            &propeller.applicable_configuration_claim_ids,
            &propeller.source_ids,
            propeller.notes.as_deref(),
            claim_ids,
            source_ids,
        )?;
        validate_applicability(
            &propeller.id,
            "propeller",
            &propeller.applicable_configuration_claim_ids,
            claims,
            |claim| claim.propeller_id.as_deref(),
        )?;
        validate_optional_positive("propeller.diameter_m", propeller.diameter_m)?;
        validate_optional_positive("propeller.pitch_m", propeller.pitch_m)?;
        let mut seen = HashSet::new();
        for id in &propeller.dataset_ids {
            validate_stable_id("propeller dataset", id)?;
            if !seen.insert(id) {
                return Err(ReferencePropulsionEvidenceError::DuplicateReference {
                    field: format!("propeller.{}.dataset_ids", propeller.id),
                    reference_id: id.clone(),
                });
            }
        }
    }
    Ok(())
}

fn validate_spinners(
    spinners: &[SpinnerEvidenceFile],
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
    claims: &[ConfigurationClaimFile],
) -> Result<(), ReferencePropulsionEvidenceError> {
    for spinner in spinners {
        validate_component_common(
            "spinner",
            &spinner.id,
            spinner.evidence_class,
            spinner.manufacturer.as_deref(),
            spinner.model.as_deref(),
            &spinner.applicable_configuration_claim_ids,
            &spinner.source_ids,
            spinner.notes.as_deref(),
            claim_ids,
            source_ids,
        )?;
        validate_applicability(
            &spinner.id,
            "spinner",
            &spinner.applicable_configuration_claim_ids,
            claims,
            |claim| claim.spinner_id.as_deref(),
        )?;
        validate_optional_positive("spinner.diameter_m", spinner.diameter_m)?;
        validate_optional_positive("spinner.mass_kg", spinner.mass_kg)?;
    }
    Ok(())
}

fn validate_datasets(
    file: &PropulsionEvidenceFile,
    source_ids: &HashSet<String>,
    propeller_ids: &HashSet<String>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    let dataset_ids = component_ids(
        "propeller dataset",
        file.propeller_datasets.iter().map(|item| &item.id),
    )?;
    for dataset in &file.propeller_datasets {
        validate_optional_ref(
            "propeller_dataset.propeller_id",
            Some(&dataset.propeller_id),
            propeller_ids,
            "propeller",
        )?;
        validate_optional_ref(
            "propeller_dataset.source_id",
            Some(&dataset.source_id),
            source_ids,
            "source",
        )?;
        if dataset.evidence_class != ComponentEvidenceClass::ManufacturerData {
            return invalid(
                "propeller_dataset.evidence_class",
                "APC raw data must be manufacturer data",
            );
        }
        validate_relative_path(&dataset.raw_source_path)?;
        validate_sha256(Some(&dataset.sha256), "propeller_dataset.sha256")?;
        let source = file
            .provenance_sources
            .iter()
            .find(|source| source.id == dataset.source_id)
            .expect("validated source reference");
        let Some(source_sha256) = source.sha256.as_deref() else {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "source_sha256",
            });
        };
        if !source_sha256.eq_ignore_ascii_case(&dataset.sha256) {
            return Err(ReferencePropulsionEvidenceError::LinkedDatasetMismatch {
                dataset_id: dataset.id.clone(),
                field: "source_sha256",
            });
        }
        if dataset.byte_count == 0
            || dataset.line_count == 0
            || dataset.rpm_block_count == 0
            || dataset.row_count == 0
            || dataset.coefficient_row_count == 0
            || dataset.coefficient_row_count > dataset.row_count
        {
            return invalid(
                "propeller_dataset.counts",
                "linked source counts must be positive",
            );
        }
        validate_required_text("propeller_dataset.source_format", &dataset.source_format)?;
        validate_required_text("propeller_dataset.source_version", &dataset.source_version)?;
        validate_required_text(
            "propeller_dataset.simulation_date",
            &dataset.simulation_date,
        )?;
        validate_optional_positive("propeller_dataset.diameter_m", Some(dataset.diameter_m))?;
        validate_optional_positive("propeller_dataset.pitch_m", Some(dataset.pitch_m))?;
        validate_optional_text("propeller_dataset.notes", dataset.notes.as_deref())?;
        let interpretation = &dataset.parser_interpretation;
        for (field, value) in [
            ("parser_interpretation.rpm", &interpretation.rpm),
            (
                "parser_interpretation.advance_ratio_j",
                &interpretation.advance_ratio_j,
            ),
            ("parser_interpretation.ct", &interpretation.ct),
            ("parser_interpretation.cp", &interpretation.cp),
            (
                "parser_interpretation.efficiency",
                &interpretation.efficiency,
            ),
            ("parser_interpretation.cq", &interpretation.cq),
        ] {
            validate_required_text(field, value)?;
        }
        let propeller = file
            .propellers
            .iter()
            .find(|item| item.id == dataset.propeller_id)
            .expect("validated propeller reference");
        if propeller
            .diameter_m
            .is_none_or(|value| (value - dataset.diameter_m).abs() > DIMENSION_TOLERANCE_M)
            || propeller
                .pitch_m
                .is_none_or(|value| (value - dataset.pitch_m).abs() > DIMENSION_TOLERANCE_M)
        {
            return invalid(
                "propeller_dataset.dimensions",
                "dataset dimensions must match its propeller evidence",
            );
        }
    }
    for propeller in &file.propellers {
        for dataset_id in &propeller.dataset_ids {
            if !dataset_ids.contains(dataset_id) {
                return Err(ReferencePropulsionEvidenceError::UnresolvedReference {
                    field: format!("propeller.{}.dataset_ids", propeller.id),
                    kind: "propeller dataset",
                    reference_id: dataset_id.clone(),
                });
            }
            let dataset = file
                .propeller_datasets
                .iter()
                .find(|item| item.id == *dataset_id)
                .expect("validated dataset ID");
            if dataset.propeller_id != propeller.id {
                return invalid(
                    "propeller.dataset_ids",
                    "dataset applicability names a different propeller",
                );
            }
        }
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn validate_component_common(
    kind: &'static str,
    id: &str,
    evidence_class: ComponentEvidenceClass,
    manufacturer: Option<&str>,
    model: Option<&str>,
    applicable_claim_ids: &[String],
    source_refs: &[String],
    notes: Option<&str>,
    claim_ids: &HashSet<String>,
    source_ids: &HashSet<String>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    validate_stable_id(kind, id)?;
    validate_optional_text("component.manufacturer", manufacturer)?;
    validate_optional_text("component.model", model)?;
    validate_refs(
        "component.applicable_configuration_claim_ids",
        applicable_claim_ids,
        claim_ids,
        "configuration claim",
    )?;
    validate_refs("component.source_ids", source_refs, source_ids, "source")?;
    if source_refs.is_empty() {
        return invalid(
            "component.source_ids",
            "component evidence requires provenance",
        );
    }
    validate_optional_text("component.notes", notes)?;
    let _ = evidence_class;
    Ok(())
}

fn validate_applicability<'a>(
    component_id: &str,
    kind: &'static str,
    applicable_claim_ids: &[String],
    claims: &'a [ConfigurationClaimFile],
    claim_component: impl Fn(&'a ConfigurationClaimFile) -> Option<&'a str>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    for claim_id in applicable_claim_ids {
        let claim = claims
            .iter()
            .find(|claim| claim.id == *claim_id)
            .expect("validated claim reference");
        if claim_component(claim) != Some(component_id) {
            return Err(ReferencePropulsionEvidenceError::InvalidEvidence {
                field: format!("{kind}.{component_id}.applicable_configuration_claim_ids"),
                reason: "claim does not identify this component",
            });
        }
    }
    Ok(())
}

fn evaluate(
    file: &PropulsionEvidenceFile,
    apc_datasets: &HashMap<String, ApcPerformanceData>,
) -> PropulsionEvidenceEvaluation {
    let configuration_claims = file
        .configuration_claims
        .iter()
        .map(|claim| ConfigurationClaimSummary {
            id: claim.id.clone(),
            evidence_class: claim.evidence_class,
            physical_airframe_id: claim.physical_airframe_id.clone(),
            operational_configuration_id: claim.operational_configuration_id.clone(),
            propulsion_configuration_id: claim.propulsion_configuration_id.clone(),
        })
        .collect();
    let installed = file.configuration_claims.iter().find(|claim| {
        matches!(
            claim.evidence_class,
            PropulsionConfigurationEvidenceClass::SpecificInstalledConfiguration
                | PropulsionConfigurationEvidenceClass::MeasuredConfiguration
        )
    });
    let configuration_identified = installed.is_some();
    let motor_evidence_ready = installed
        .and_then(|claim| claim.motor_id.as_deref())
        .and_then(|id| file.motors.iter().find(|motor| motor.id == id))
        .is_some_and(|motor| {
            motor.kv_rpm_per_v.is_some()
                && motor.winding_resistance_ohm.is_some()
                && motor.no_load_current_a.is_some()
        });
    let esc_evidence_ready = installed
        .and_then(|claim| claim.esc_id.as_deref())
        .and_then(|id| file.escs.iter().find(|esc| esc.id == id))
        .is_some_and(|esc| {
            esc.current_rating_a.is_some()
                && (esc.resistance_ohm.is_some() || esc.efficiency.is_some())
        });
    let battery_evidence_ready = installed
        .and_then(|claim| claim.battery_id.as_deref())
        .and_then(|id| file.batteries.iter().find(|battery| battery.id == id))
        .is_some_and(|battery| {
            battery.cell_count.is_some()
                && battery.capacity_ah.is_some()
                && battery.internal_resistance_ohm.is_some()
                && !battery.voltage_load_points.is_empty()
        });
    let propeller_evidence_ready = installed
        .and_then(|claim| claim.propeller_id.as_deref())
        .and_then(|id| file.propellers.iter().find(|propeller| propeller.id == id))
        .is_some_and(|propeller| {
            propeller.diameter_m.is_some()
                && propeller.pitch_m.is_some()
                && propeller
                    .dataset_ids
                    .iter()
                    .any(|id| apc_datasets.contains_key(id))
        });
    let propulsion_evidence_ready = configuration_identified
        && motor_evidence_ready
        && esc_evidence_ready
        && battery_evidence_ready
        && propeller_evidence_ready;
    let mut blockers = Vec::new();
    if !configuration_identified {
        blockers.push(
            "specific installed or measured propulsion configuration is not identified".to_owned(),
        );
    }
    if !motor_evidence_ready {
        blockers.push(
            "installed motor Kv, winding resistance, and no-load current are incomplete".to_owned(),
        );
    }
    if !esc_evidence_ready {
        blockers.push("installed ESC loss evidence is incomplete".to_owned());
    }
    if !battery_evidence_ready {
        blockers.push(
            "installed battery resistance and voltage-under-load evidence are incomplete"
                .to_owned(),
        );
    }
    if !propeller_evidence_ready {
        blockers.push(
            "installed propeller identity and parsed coefficient evidence are incomplete"
                .to_owned(),
        );
    }
    blockers.push("M2.4A is evidence-only and cannot authorize runtime configuration".to_owned());
    PropulsionEvidenceEvaluation {
        configuration_claims,
        blockers,
        motor_evidence_ready,
        esc_evidence_ready,
        battery_evidence_ready,
        propeller_evidence_ready,
        configuration_identified,
        propulsion_evidence_ready,
        runtime_ready: false,
    }
}

fn component_ids<'a>(
    kind: &'static str,
    ids: impl Iterator<Item = &'a String>,
) -> Result<HashSet<String>, ReferencePropulsionEvidenceError> {
    let mut found = HashSet::new();
    for id in ids {
        validate_stable_id(kind, id)?;
        insert_unique(&mut found, kind, id)?;
    }
    Ok(found)
}

fn insert_unique(
    ids: &mut HashSet<String>,
    kind: &'static str,
    id: &str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if !ids.insert(id.to_owned()) {
        return Err(ReferencePropulsionEvidenceError::DuplicateStableId {
            kind,
            value: id.to_owned(),
        });
    }
    Ok(())
}

fn validate_refs(
    field: &str,
    refs: &[String],
    known: &HashSet<String>,
    kind: &'static str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    let mut seen = HashSet::new();
    for reference in refs {
        if !seen.insert(reference) {
            return Err(ReferencePropulsionEvidenceError::DuplicateReference {
                field: field.to_owned(),
                reference_id: reference.clone(),
            });
        }
        validate_optional_ref(field, Some(reference), known, kind)?;
    }
    Ok(())
}

fn validate_optional_ref(
    field: &str,
    reference: Option<&str>,
    known: &HashSet<String>,
    kind: &'static str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if let Some(reference) = reference
        && !known.contains(reference)
    {
        return Err(ReferencePropulsionEvidenceError::UnresolvedReference {
            field: field.to_owned(),
            kind,
            reference_id: reference.to_owned(),
        });
    }
    Ok(())
}

fn validate_stable_id(
    kind: &'static str,
    value: &str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if value.is_empty()
        || !value.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'_' | b'-')
        })
    {
        return Err(ReferencePropulsionEvidenceError::InvalidStableId {
            kind,
            value: value.to_owned(),
        });
    }
    Ok(())
}

fn validate_optional_stable_id(
    kind: &'static str,
    value: Option<&str>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if let Some(value) = value {
        validate_stable_id(kind, value)?;
    }
    Ok(())
}

fn validate_required_text(
    field: &str,
    value: &str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if value.trim().is_empty() {
        return invalid(field, "text must not be blank");
    }
    Ok(())
}

fn validate_optional_text(
    field: &str,
    value: Option<&str>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if value.is_some_and(|value| value.trim().is_empty()) {
        return invalid(field, "present text must not be blank");
    }
    Ok(())
}

fn validate_optional_positive(
    field: &str,
    value: Option<f64>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if value.is_some_and(|value| !value.is_finite() || value <= 0.0) {
        return invalid(
            field,
            "present value must be finite and positive; use null for unknown",
        );
    }
    Ok(())
}

fn validate_optional_efficiency(
    field: &str,
    value: Option<f64>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if value.is_some_and(|value| !value.is_finite() || value <= 0.0 || value > 1.0) {
        return invalid(field, "efficiency must be finite in (0, 1]");
    }
    Ok(())
}

fn validate_positive_range(
    field: &str,
    range: Option<[f64; 2]>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if let Some([minimum, maximum]) = range
        && (!minimum.is_finite() || !maximum.is_finite() || minimum <= 0.0 || minimum > maximum)
    {
        return invalid(field, "range must be finite, positive, and ordered");
    }
    Ok(())
}

fn validate_optional_date(
    field: &str,
    date: Option<&str>,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if date.is_some_and(|date| !is_iso_date(date)) {
        return invalid(field, "expected YYYY-MM-DD");
    }
    Ok(())
}

fn validate_sha256(
    hash: Option<&str>,
    field: &str,
) -> Result<(), ReferencePropulsionEvidenceError> {
    if hash
        .is_some_and(|hash| hash.len() != 64 || !hash.bytes().all(|byte| byte.is_ascii_hexdigit()))
    {
        return invalid(field, "SHA-256 must contain exactly 64 hexadecimal digits");
    }
    Ok(())
}

fn validate_relative_path(path: &str) -> Result<(), ReferencePropulsionEvidenceError> {
    let path = Path::new(path);
    if path.as_os_str().is_empty()
        || path.is_absolute()
        || path.components().any(|component| {
            matches!(
                component,
                Component::ParentDir | Component::RootDir | Component::Prefix(_)
            )
        })
    {
        return invalid(
            "propeller_dataset.raw_source_path",
            "path must be a safe relative path",
        );
    }
    Ok(())
}

fn invalid<T>(field: &str, reason: &'static str) -> Result<T, ReferencePropulsionEvidenceError> {
    Err(ReferencePropulsionEvidenceError::InvalidEvidence {
        field: field.to_owned(),
        reason,
    })
}

fn canonical_bits(value: f64) -> u64 {
    if value == 0.0 { 0 } else { value.to_bits() }
}

fn sha256_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let digest = Sha256::digest(bytes);
    let mut encoded = String::with_capacity(digest.len() * 2);
    for byte in digest {
        encoded.push(char::from(HEX[usize::from(byte >> 4)]));
        encoded.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    encoded
}

fn is_iso_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    bytes.len() == 10
        && bytes[4] == b'-'
        && bytes[7] == b'-'
        && bytes
            .iter()
            .enumerate()
            .all(|(index, byte)| matches!(index, 4 | 7) || byte.is_ascii_digit())
}
