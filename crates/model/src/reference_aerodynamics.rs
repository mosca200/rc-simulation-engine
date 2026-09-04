//! Strict loading and evidence-level evaluation of off-runtime aerodynamic data.
//!
//! M2.3A does not construct [`sim_core::PolarTable`] values and is never consulted by the
//! simulation stepping path. It preserves traceable airfoil coordinates and polar evidence for a
//! later, separately reviewed runtime integration slice.

use std::{
    collections::HashSet,
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{REFERENCE_AERODYNAMIC_EVIDENCE_SCHEMA_V0, SurveyClassification};

const ARTIFACT_KIND: &str = "aerodynamic_evidence_not_runtime_configuration";
const COORDINATE_TOLERANCE: f64 = 1.0e-12;

#[derive(Debug, Error)]
pub enum ReferenceAerodynamicEvidenceError {
    #[error("failed to read reference aerodynamic evidence {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("reference aerodynamic evidence JSON has invalid structure: {source}")]
    InvalidStructure {
        #[source]
        source: serde_json::Error,
    },
    #[error("unsupported reference aerodynamic evidence schema {found:?}")]
    UnsupportedSchema { found: String },
    #[error("invalid reference aerodynamic evidence artifact kind {found:?}")]
    InvalidArtifactKind { found: String },
    #[error("invalid stable {kind} ID {value:?}; expected nonempty [a-z0-9_-]+")]
    InvalidStableId { kind: &'static str, value: String },
    #[error("duplicate stable {kind} ID {value:?}")]
    DuplicateStableId { kind: &'static str, value: String },
    #[error("{field} references unknown provenance source {source_id:?}")]
    UnresolvedSourceReference { field: String, source_id: String },
    #[error("{field} contains duplicate provenance reference {source_id:?}")]
    DuplicateSourceReference { field: String, source_id: String },
    #[error("invalid aerodynamic metadata {field}: {reason}")]
    InvalidMetadata {
        field: &'static str,
        reason: &'static str,
    },
    #[error("invalid airfoil coordinates at point {index}: {reason}")]
    InvalidCoordinate { index: usize, reason: &'static str },
    #[error("invalid polar dataset {dataset_id:?}: {reason}")]
    InvalidPolarDataset {
        dataset_id: String,
        reason: &'static str,
    },
    #[error(
        "duplicate Reynolds/Mach/method point in datasets {first_dataset_id:?} and {duplicate_dataset_id:?}"
    )]
    DuplicateFlowCondition {
        first_dataset_id: String,
        duplicate_dataset_id: String,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AerodynamicEvidenceClass {
    Published,
    GeneratedSolver,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ConvergenceStatus {
    NotApplicablePublished,
    Converged,
    Unresolved,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct CoveragePoint {
    reynolds: f64,
    mach: f64,
}

impl CoveragePoint {
    pub const fn reynolds(&self) -> f64 {
        self.reynolds
    }

    pub const fn mach(&self) -> f64 {
        self.mach
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AerodynamicDatasetSummary {
    id: String,
    evidence_class: AerodynamicEvidenceClass,
    reynolds: f64,
    mach: f64,
    method_id: String,
    convergence_status: ConvergenceStatus,
    evidence_ready: bool,
}

impl AerodynamicDatasetSummary {
    pub fn id(&self) -> &str {
        &self.id
    }

    pub const fn evidence_class(&self) -> AerodynamicEvidenceClass {
        self.evidence_class
    }

    pub const fn reynolds(&self) -> f64 {
        self.reynolds
    }

    pub const fn mach(&self) -> f64 {
        self.mach
    }

    pub fn method_id(&self) -> &str {
        &self.method_id
    }

    pub const fn convergence_status(&self) -> ConvergenceStatus {
        self.convergence_status
    }

    pub const fn evidence_ready(&self) -> bool {
        self.evidence_ready
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct AerodynamicEvidenceEvaluation {
    datasets: Vec<AerodynamicDatasetSummary>,
    coverage_holes: Vec<CoveragePoint>,
    blockers: Vec<String>,
    airfoil_identity_ready: bool,
    coordinates_ready: bool,
    polar_evidence_ready: bool,
    coverage_ready: bool,
    aerodynamic_evidence_ready: bool,
    runtime_ready: bool,
}

impl AerodynamicEvidenceEvaluation {
    pub fn datasets(&self) -> &[AerodynamicDatasetSummary] {
        &self.datasets
    }

    pub fn coverage_holes(&self) -> &[CoveragePoint] {
        &self.coverage_holes
    }

    pub fn blockers(&self) -> &[String] {
        &self.blockers
    }

    pub const fn airfoil_identity_ready(&self) -> bool {
        self.airfoil_identity_ready
    }

    pub const fn coordinates_ready(&self) -> bool {
        self.coordinates_ready
    }

    pub const fn polar_evidence_ready(&self) -> bool {
        self.polar_evidence_ready
    }

    pub const fn coverage_ready(&self) -> bool {
        self.coverage_ready
    }

    pub const fn aerodynamic_evidence_ready(&self) -> bool {
        self.aerodynamic_evidence_ready
    }

    /// M2.3A cannot authorize runtime aerodynamic data.
    pub const fn runtime_ready(&self) -> bool {
        self.runtime_ready
    }
}

#[derive(Debug, Clone)]
pub struct AerodynamicEvidence {
    file: AerodynamicEvidenceFile,
    evaluation: AerodynamicEvidenceEvaluation,
}

impl AerodynamicEvidence {
    pub fn campaign_id(&self) -> &str {
        &self.file.campaign.id
    }

    pub const fn classification(&self) -> SurveyClassification {
        self.file.campaign.classification
    }

    pub fn airfoil_name(&self) -> &str {
        &self.file.airfoil_identity.name
    }

    pub const fn evaluation(&self) -> &AerodynamicEvidenceEvaluation {
        &self.evaluation
    }
}

pub struct AerodynamicEvidenceLoader;

impl AerodynamicEvidenceLoader {
    pub fn from_json_str(
        json: &str,
    ) -> Result<AerodynamicEvidence, ReferenceAerodynamicEvidenceError> {
        let file: AerodynamicEvidenceFile = serde_json::from_str(json)
            .map_err(|source| ReferenceAerodynamicEvidenceError::InvalidStructure { source })?;
        validate_file(&file)?;
        let evaluation = evaluate(&file);
        Ok(AerodynamicEvidence { file, evaluation })
    }
}

pub fn load_reference_aerodynamic_evidence(
    path: impl AsRef<Path>,
) -> Result<AerodynamicEvidence, ReferenceAerodynamicEvidenceError> {
    let path = path.as_ref();
    let json =
        fs::read_to_string(path).map_err(|source| ReferenceAerodynamicEvidenceError::Io {
            path: path.to_path_buf(),
            source,
        })?;
    AerodynamicEvidenceLoader::from_json_str(&json)
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct AerodynamicEvidenceFile {
    schema: String,
    artifact_kind: String,
    campaign: CampaignFile,
    airfoil_identity: AirfoilIdentityFile,
    coordinates: Option<CoordinatesFile>,
    provenance_sources: Vec<SourceFile>,
    operating_envelope: Option<OperatingEnvelopeFile>,
    polar_datasets: Vec<PolarDatasetFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignFile {
    id: String,
    classification: SurveyClassification,
    manufacturer: String,
    family: String,
    variant: String,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct AirfoilIdentityFile {
    name: String,
    source_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SourceKind {
    ManufacturerDocumentation,
    AirfoilDatabase,
    PublishedResearch,
    SolverTool,
    NumericalAnalysis,
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

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CoordinateFormat {
    Selig,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CoordinateNormalization {
    UnitChordSourceAsPublished,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CoordinateOrdering {
    UpperTrailingEdgeToLeadingEdgeToLowerTrailingEdge,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum LeadingEdgeRepresentation {
    SinglePoint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum TrailingEdgeRepresentation {
    Open,
    Closed,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoordinatesFile {
    source_id: String,
    coordinate_format: CoordinateFormat,
    normalization: CoordinateNormalization,
    ordering: CoordinateOrdering,
    leading_edge_representation: LeadingEdgeRepresentation,
    trailing_edge_representation: TrailingEdgeRepresentation,
    transformation_provenance: String,
    points_x_over_c_y_over_c: Vec<[f64; 2]>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct OperatingEnvelopeFile {
    rationale: String,
    source_ids: Vec<String>,
    required_points: Vec<FlowPointFile>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct FlowPointFile {
    reynolds: f64,
    mach: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolarDatasetFile {
    id: String,
    evidence_class: AerodynamicEvidenceClass,
    flow_conditions: FlowConditionsFile,
    transition: TransitionFile,
    method: MethodFile,
    source_ids: Vec<String>,
    samples: Vec<PolarSampleFile>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct FlowConditionsFile {
    reynolds: f64,
    mach: f64,
    density_kg_m3: Option<f64>,
    dynamic_viscosity_pa_s: Option<f64>,
    kinematic_viscosity_m2_s: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TransitionFile {
    assumptions: Option<String>,
    ncrit: Option<f64>,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MethodFile {
    id: String,
    solver_or_tool: Option<String>,
    exact_version: Option<String>,
    command_or_config: Option<String>,
    convergence_status: ConvergenceStatus,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct PolarSampleFile {
    alpha_rad: f64,
    cl: f64,
    cd: f64,
    cm: f64,
}

fn validate_file(file: &AerodynamicEvidenceFile) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if file.schema != REFERENCE_AERODYNAMIC_EVIDENCE_SCHEMA_V0 {
        return Err(ReferenceAerodynamicEvidenceError::UnsupportedSchema {
            found: file.schema.clone(),
        });
    }
    if file.artifact_kind != ARTIFACT_KIND {
        return Err(ReferenceAerodynamicEvidenceError::InvalidArtifactKind {
            found: file.artifact_kind.clone(),
        });
    }
    validate_campaign(&file.campaign)?;
    validate_required_text("airfoil_identity.name", &file.airfoil_identity.name)?;
    validate_optional_text(
        "airfoil_identity.notes",
        file.airfoil_identity.notes.as_deref(),
    )?;
    let sources = validate_sources(&file.provenance_sources)?;
    validate_source_refs(
        "airfoil_identity",
        &file.airfoil_identity.source_ids,
        &sources,
    )?;
    if let Some(coordinates) = file.coordinates.as_ref() {
        validate_coordinates(coordinates, &sources)?;
    }
    if let Some(envelope) = file.operating_envelope.as_ref() {
        validate_operating_envelope(envelope, &sources)?;
    }
    validate_datasets(&file.polar_datasets, &sources)
}

fn validate_campaign(campaign: &CampaignFile) -> Result<(), ReferenceAerodynamicEvidenceError> {
    validate_stable_id("campaign", &campaign.id)?;
    validate_required_text("campaign.manufacturer", &campaign.manufacturer)?;
    validate_required_text("campaign.family", &campaign.family)?;
    validate_required_text("campaign.variant", &campaign.variant)?;
    validate_optional_text("campaign.notes", campaign.notes.as_deref())
}

fn validate_sources(
    sources: &[SourceFile],
) -> Result<HashSet<String>, ReferenceAerodynamicEvidenceError> {
    let mut ids = HashSet::new();
    for source in sources {
        validate_stable_id("provenance source", &source.id)?;
        if !ids.insert(source.id.clone()) {
            return Err(ReferenceAerodynamicEvidenceError::DuplicateStableId {
                kind: "provenance source",
                value: source.id.clone(),
            });
        }
        validate_required_text("provenance_sources.title", &source.title)?;
        validate_optional_text("provenance_sources.publisher", source.publisher.as_deref())?;
        validate_optional_text("provenance_sources.url", source.url.as_deref())?;
        validate_optional_text("provenance_sources.notes", source.notes.as_deref())?;
        if source
            .retrieval_date
            .as_deref()
            .is_some_and(|date| !is_iso_date(date))
        {
            return Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
                field: "provenance_sources.retrieval_date",
                reason: "expected a real YYYY-MM-DD calendar date",
            });
        }
        validate_sha256(source.sha256.as_deref())?;
    }
    Ok(ids)
}

fn validate_coordinates(
    coordinates: &CoordinatesFile,
    source_ids: &HashSet<String>,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    validate_stable_id("coordinate source", &coordinates.source_id)?;
    validate_source_refs(
        "coordinates",
        std::slice::from_ref(&coordinates.source_id),
        source_ids,
    )?;
    validate_required_text(
        "coordinates.transformation_provenance",
        &coordinates.transformation_provenance,
    )?;
    validate_optional_text("coordinates.notes", coordinates.notes.as_deref())?;
    let points = &coordinates.points_x_over_c_y_over_c;
    if points.len() < 5 {
        return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
            index: points.len(),
            reason: "airfoil requires at least five points",
        });
    }
    let mut seen = HashSet::new();
    for (index, &[x, y]) in points.iter().enumerate() {
        if !x.is_finite() || !y.is_finite() {
            return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
                index,
                reason: "coordinate must be finite",
            });
        }
        if !(-COORDINATE_TOLERANCE..=1.0 + COORDINATE_TOLERANCE).contains(&x)
            || !(-0.5..=0.5).contains(&y)
        {
            return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
                index,
                reason: "coordinate lies outside normalized chord bounds",
            });
        }
        if !seen.insert((x.to_bits(), y.to_bits())) {
            return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
                index,
                reason: "duplicate coordinate point",
            });
        }
    }
    let leading_edge_index = points
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a[0].total_cmp(&b[0]))
        .map(|(index, _)| index)
        .expect("coordinate list is nonempty");
    if leading_edge_index == 0 || leading_edge_index + 1 == points.len() {
        return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
            index: leading_edge_index,
            reason: "leading edge must separate upper and lower surfaces",
        });
    }
    if points[leading_edge_index][0].abs() > COORDINATE_TOLERANCE
        || (points[0][0] - 1.0).abs() > COORDINATE_TOLERANCE
        || (points[points.len() - 1][0] - 1.0).abs() > COORDINATE_TOLERANCE
    {
        return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
            index: leading_edge_index,
            reason: "unit-chord leading/trailing edge representation is incomplete",
        });
    }
    for (offset, pair) in points[..=leading_edge_index].windows(2).enumerate() {
        if pair[1][0] >= pair[0][0] {
            return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
                index: offset + 1,
                reason: "upper-surface x must strictly decrease toward the leading edge",
            });
        }
    }
    for (offset, pair) in points[leading_edge_index..].windows(2).enumerate() {
        if pair[1][0] <= pair[0][0] {
            return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
                index: leading_edge_index + offset + 1,
                reason: "lower-surface x must strictly increase toward the trailing edge",
            });
        }
    }
    let trailing_edge_gap = (points[0][1] - points[points.len() - 1][1]).abs();
    let representation_matches = match coordinates.trailing_edge_representation {
        TrailingEdgeRepresentation::Open => trailing_edge_gap > COORDINATE_TOLERANCE,
        TrailingEdgeRepresentation::Closed => trailing_edge_gap <= COORDINATE_TOLERANCE,
    };
    if !representation_matches {
        return Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate {
            index: points.len() - 1,
            reason: "declared trailing-edge representation does not match coordinates",
        });
    }
    let _ = (
        coordinates.coordinate_format,
        coordinates.normalization,
        coordinates.ordering,
        coordinates.leading_edge_representation,
    );
    Ok(())
}

fn validate_operating_envelope(
    envelope: &OperatingEnvelopeFile,
    sources: &HashSet<String>,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    validate_required_text("operating_envelope.rationale", &envelope.rationale)?;
    validate_source_refs("operating_envelope", &envelope.source_ids, sources)?;
    if envelope.required_points.is_empty() {
        return Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
            field: "operating_envelope.required_points",
            reason: "present envelope requires at least one explicit point",
        });
    }
    let mut seen = HashSet::new();
    for point in &envelope.required_points {
        validate_flow_point("operating envelope", point.reynolds, point.mach)?;
        if !seen.insert((canonical_bits(point.reynolds), canonical_bits(point.mach))) {
            return Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
                field: "operating_envelope.required_points",
                reason: "duplicate Reynolds/Mach requirement",
            });
        }
    }
    Ok(())
}

fn validate_datasets(
    datasets: &[PolarDatasetFile],
    sources: &HashSet<String>,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    let mut ids = HashSet::new();
    let mut conditions: Vec<(&PolarDatasetFile, (u64, u64))> = Vec::new();
    for dataset in datasets {
        validate_stable_id("polar dataset", &dataset.id)?;
        if !ids.insert(dataset.id.clone()) {
            return Err(ReferenceAerodynamicEvidenceError::DuplicateStableId {
                kind: "polar dataset",
                value: dataset.id.clone(),
            });
        }
        validate_stable_id("polar method", &dataset.method.id)?;
        validate_optional_text("polar_dataset.notes", dataset.notes.as_deref())?;
        validate_source_refs(
            &format!("polar_dataset.{}", dataset.id),
            &dataset.source_ids,
            sources,
        )?;
        validate_flow_conditions(dataset)?;
        validate_transition(dataset)?;
        validate_method(dataset)?;
        validate_samples(dataset)?;
        let condition = (
            canonical_bits(dataset.flow_conditions.reynolds),
            canonical_bits(dataset.flow_conditions.mach),
        );
        if let Some((first, _)) = conditions.iter().find(|(first, existing)| {
            *existing == condition && first.method.id == dataset.method.id
        }) {
            return Err(ReferenceAerodynamicEvidenceError::DuplicateFlowCondition {
                first_dataset_id: first.id.clone(),
                duplicate_dataset_id: dataset.id.clone(),
            });
        }
        conditions.push((dataset, condition));
    }
    Ok(())
}

fn validate_flow_conditions(
    dataset: &PolarDatasetFile,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    let flow = &dataset.flow_conditions;
    validate_flow_point(&dataset.id, flow.reynolds, flow.mach)?;
    for value in [
        flow.density_kg_m3,
        flow.dynamic_viscosity_pa_s,
        flow.kinematic_viscosity_m2_s,
    ]
    .into_iter()
    .flatten()
    {
        if !value.is_finite() || value <= 0.0 {
            return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
                dataset_id: dataset.id.clone(),
                reason: "present density or viscosity must be finite and positive",
            });
        }
    }
    Ok(())
}

fn validate_flow_point(
    field: &str,
    reynolds: f64,
    mach: f64,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if !reynolds.is_finite() || reynolds <= 0.0 {
        return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
            dataset_id: field.to_owned(),
            reason: "Reynolds number must be finite and positive",
        });
    }
    if !mach.is_finite() || mach < 0.0 {
        return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
            dataset_id: field.to_owned(),
            reason: "Mach number must be finite and nonnegative",
        });
    }
    Ok(())
}

fn validate_transition(
    dataset: &PolarDatasetFile,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    let transition = &dataset.transition;
    validate_optional_text(
        "polar_dataset.transition.assumptions",
        transition.assumptions.as_deref(),
    )?;
    if transition
        .ncrit
        .is_some_and(|value| !value.is_finite() || value <= 0.0)
    {
        return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
            dataset_id: dataset.id.clone(),
            reason: "present Ncrit must be finite and positive",
        });
    }
    for value in [
        transition.forced_transition_upper_x_over_c,
        transition.forced_transition_lower_x_over_c,
    ]
    .into_iter()
    .flatten()
    {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
                dataset_id: dataset.id.clone(),
                reason: "forced-transition station must be finite within [0, 1] chord",
            });
        }
    }
    Ok(())
}

fn validate_method(dataset: &PolarDatasetFile) -> Result<(), ReferenceAerodynamicEvidenceError> {
    for (field, value) in [
        (
            "polar_dataset.method.solver_or_tool",
            dataset.method.solver_or_tool.as_deref(),
        ),
        (
            "polar_dataset.method.exact_version",
            dataset.method.exact_version.as_deref(),
        ),
        (
            "polar_dataset.method.command_or_config",
            dataset.method.command_or_config.as_deref(),
        ),
    ] {
        validate_optional_text(field, value)?;
    }
    let valid_class_status = match dataset.evidence_class {
        AerodynamicEvidenceClass::Published => {
            dataset.method.convergence_status == ConvergenceStatus::NotApplicablePublished
        }
        AerodynamicEvidenceClass::GeneratedSolver => {
            dataset.method.convergence_status != ConvergenceStatus::NotApplicablePublished
        }
    };
    if !valid_class_status {
        return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
            dataset_id: dataset.id.clone(),
            reason: "evidence class and convergence status are inconsistent",
        });
    }
    Ok(())
}

fn validate_samples(dataset: &PolarDatasetFile) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if dataset.samples.len() < 2 {
        return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
            dataset_id: dataset.id.clone(),
            reason: "polar requires at least two samples",
        });
    }
    for (index, sample) in dataset.samples.iter().enumerate() {
        if ![sample.alpha_rad, sample.cl, sample.cd, sample.cm]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
                dataset_id: dataset.id.clone(),
                reason: "polar sample contains a non-finite value",
            });
        }
        if sample.cd < 0.0 {
            return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
                dataset_id: dataset.id.clone(),
                reason: "polar drag coefficient must be nonnegative",
            });
        }
        if index > 0 && sample.alpha_rad <= dataset.samples[index - 1].alpha_rad {
            return Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset {
                dataset_id: dataset.id.clone(),
                reason: "polar alpha must be strictly increasing",
            });
        }
    }
    Ok(())
}

fn evaluate(file: &AerodynamicEvidenceFile) -> AerodynamicEvidenceEvaluation {
    let mut blockers = Vec::new();
    let airfoil_identity_ready = !file.airfoil_identity.source_ids.is_empty();
    if !airfoil_identity_ready {
        blockers.push("airfoil_identity_evidence".to_owned());
    }
    let coordinates_ready = file.coordinates.as_ref().is_some_and(|coordinates| {
        let source = file
            .provenance_sources
            .iter()
            .find(|source| source.id == coordinates.source_id);
        source.is_some_and(|source| {
            source.kind == SourceKind::AirfoilDatabase
                && source.publisher.is_some()
                && source.url.is_some()
                && source.retrieval_date.is_some()
                && source.sha256.is_some()
        })
    });
    if !coordinates_ready {
        blockers.push("airfoil_coordinates_with_traceable_source".to_owned());
    }

    let mut datasets: Vec<_> = file
        .polar_datasets
        .iter()
        .map(|dataset| {
            let evidence_ready = dataset_ready(dataset);
            if dataset.source_ids.is_empty() {
                blockers.push(format!("polar_dataset_provenance:{}", dataset.id));
            }
            if dataset.evidence_class == AerodynamicEvidenceClass::GeneratedSolver {
                if dataset.method.convergence_status != ConvergenceStatus::Converged {
                    blockers.push(format!("generated_dataset_convergence:{}", dataset.id));
                }
                if !generated_metadata_complete(dataset) {
                    blockers.push(format!("generated_dataset_solver_metadata:{}", dataset.id));
                }
                if dataset.transition.assumptions.is_none() {
                    blockers.push(format!(
                        "generated_dataset_transition_assumptions:{}",
                        dataset.id
                    ));
                }
            }
            AerodynamicDatasetSummary {
                id: dataset.id.clone(),
                evidence_class: dataset.evidence_class,
                reynolds: dataset.flow_conditions.reynolds,
                mach: canonical_zero(dataset.flow_conditions.mach),
                method_id: dataset.method.id.clone(),
                convergence_status: dataset.method.convergence_status,
                evidence_ready,
            }
        })
        .collect();
    datasets.sort_by(|a, b| {
        a.reynolds
            .total_cmp(&b.reynolds)
            .then_with(|| a.mach.total_cmp(&b.mach))
            .then_with(|| a.method_id.cmp(&b.method_id))
            .then_with(|| a.id.cmp(&b.id))
    });
    let polar_evidence_ready =
        !datasets.is_empty() && datasets.iter().all(|item| item.evidence_ready);
    if datasets.is_empty() {
        blockers.push("polar_dataset_nonempty".to_owned());
    }

    let mut coverage_holes = Vec::new();
    let coverage_ready = match file.operating_envelope.as_ref() {
        None => {
            blockers.push("operating_envelope".to_owned());
            false
        }
        Some(envelope) => {
            if envelope.source_ids.is_empty() {
                blockers.push("operating_envelope_provenance".to_owned());
            }
            for point in &envelope.required_points {
                let covered = datasets.iter().any(|dataset| {
                    dataset.evidence_ready
                        && canonical_bits(dataset.reynolds) == canonical_bits(point.reynolds)
                        && canonical_bits(dataset.mach) == canonical_bits(point.mach)
                });
                if !covered {
                    coverage_holes.push(CoveragePoint {
                        reynolds: point.reynolds,
                        mach: canonical_zero(point.mach),
                    });
                    blockers.push(format!(
                        "coverage_point:re_{:016x}:mach_{:016x}",
                        canonical_bits(point.reynolds),
                        canonical_bits(point.mach)
                    ));
                }
            }
            !envelope.source_ids.is_empty() && coverage_holes.is_empty()
        }
    };
    coverage_holes.sort_by(|a, b| {
        a.reynolds
            .total_cmp(&b.reynolds)
            .then_with(|| a.mach.total_cmp(&b.mach))
    });
    blockers.sort();
    blockers.dedup();
    let aerodynamic_evidence_ready =
        airfoil_identity_ready && coordinates_ready && polar_evidence_ready && coverage_ready;
    AerodynamicEvidenceEvaluation {
        datasets,
        coverage_holes,
        blockers,
        airfoil_identity_ready,
        coordinates_ready,
        polar_evidence_ready,
        coverage_ready,
        aerodynamic_evidence_ready,
        runtime_ready: false,
    }
}

fn dataset_ready(dataset: &PolarDatasetFile) -> bool {
    if dataset.source_ids.is_empty() {
        return false;
    }
    match dataset.evidence_class {
        AerodynamicEvidenceClass::Published => true,
        AerodynamicEvidenceClass::GeneratedSolver => {
            dataset.method.convergence_status == ConvergenceStatus::Converged
                && generated_metadata_complete(dataset)
                && dataset.transition.assumptions.is_some()
        }
    }
}

fn generated_metadata_complete(dataset: &PolarDatasetFile) -> bool {
    dataset.method.solver_or_tool.is_some()
        && dataset.method.exact_version.is_some()
        && dataset.method.command_or_config.is_some()
}

fn validate_source_refs(
    field: &str,
    refs: &[String],
    known: &HashSet<String>,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    let mut seen = HashSet::new();
    for source_id in refs {
        validate_stable_id("provenance source reference", source_id)?;
        if !known.contains(source_id) {
            return Err(
                ReferenceAerodynamicEvidenceError::UnresolvedSourceReference {
                    field: field.to_owned(),
                    source_id: source_id.clone(),
                },
            );
        }
        if !seen.insert(source_id) {
            return Err(
                ReferenceAerodynamicEvidenceError::DuplicateSourceReference {
                    field: field.to_owned(),
                    source_id: source_id.clone(),
                },
            );
        }
    }
    Ok(())
}

pub(crate) fn validate_stable_id(
    kind: &'static str,
    value: &str,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if !value.is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"_-".contains(&byte))
    {
        Ok(())
    } else {
        Err(ReferenceAerodynamicEvidenceError::InvalidStableId {
            kind,
            value: value.to_owned(),
        })
    }
}

fn validate_required_text(
    field: &'static str,
    value: &str,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if value.trim().is_empty() {
        Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
            field,
            reason: "text must not be empty or whitespace",
        })
    } else {
        Ok(())
    }
}

fn validate_optional_text(
    field: &'static str,
    value: Option<&str>,
) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if value.is_some_and(|value| value.trim().is_empty()) {
        Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
            field,
            reason: "present text must not be empty or whitespace",
        })
    } else {
        Ok(())
    }
}

fn validate_sha256(value: Option<&str>) -> Result<(), ReferenceAerodynamicEvidenceError> {
    if value.is_some_and(|value| {
        value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit())
    }) {
        Err(ReferenceAerodynamicEvidenceError::InvalidMetadata {
            field: "provenance_sources.sha256",
            reason: "SHA-256 must contain exactly 64 hexadecimal characters",
        })
    } else {
        Ok(())
    }
}

fn is_iso_date(value: &str) -> bool {
    let bytes = value.as_bytes();
    if bytes.len() != 10
        || bytes[4] != b'-'
        || bytes[7] != b'-'
        || !bytes
            .iter()
            .enumerate()
            .all(|(index, byte)| index == 4 || index == 7 || byte.is_ascii_digit())
    {
        return false;
    }
    let Some((year, month, day)) = value[0..4]
        .parse::<u16>()
        .ok()
        .zip(value[5..7].parse::<u8>().ok())
        .zip(value[8..10].parse::<u8>().ok())
        .map(|((year, month), day)| (year, month, day))
    else {
        return false;
    };
    let leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return false,
    };
    year != 0 && day != 0 && day <= days
}

fn canonical_zero(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

fn canonical_bits(value: f64) -> u64 {
    canonical_zero(value).to_bits()
}
