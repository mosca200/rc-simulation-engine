//! Strict loading and deterministic evaluation of off-runtime mass-properties evidence.
//!
//! This module cannot create or mutate an [`crate::AircraftModel`]. Its outputs are documentary
//! candidates and readiness diagnostics, never runtime authority.

use std::{
    collections::HashSet,
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{REFERENCE_MASS_PROPERTIES_SCHEMA_V0, SurveyClassification};

const ARTIFACT_KIND: &str = "mass_properties_evidence_not_runtime_configuration";
const SYMMETRY_RELATIVE_TOLERANCE: f64 = 1.0e-12;

#[derive(Debug, Error)]
pub enum ReferenceMassPropertiesError {
    #[error("failed to read reference mass-properties campaign {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("reference mass-properties JSON has invalid structure: {source}")]
    InvalidStructure {
        #[source]
        source: serde_json::Error,
    },
    #[error("unsupported reference mass-properties schema {found:?}")]
    UnsupportedSchema { found: String },
    #[error("invalid reference mass-properties artifact kind {found:?}")]
    InvalidArtifactKind { found: String },
    #[error("invalid stable {kind} ID {value:?}; expected nonempty [a-z0-9_-]+")]
    InvalidStableId { kind: &'static str, value: String },
    #[error("duplicate stable {kind} ID {value:?}")]
    DuplicateStableId { kind: &'static str, value: String },
    #[error("{field} references unknown provenance source {source_id:?}")]
    UnresolvedSourceReference { field: String, source_id: String },
    #[error("{field} references unknown photograph {photograph_id:?}")]
    UnresolvedPhotographReference {
        field: String,
        photograph_id: String,
    },
    #[error("{field} contains duplicate evidence reference {reference_id:?}")]
    DuplicateEvidenceReference { field: String, reference_id: String },
    #[error("invalid mass-properties measurement {field}: {reason}")]
    InvalidMeasurement { field: String, reason: &'static str },
    #[error("invalid mass-properties metadata {field}: {reason}")]
    InvalidMetadata {
        field: &'static str,
        reason: &'static str,
    },
    #[error(
        "installed component {component_id:?} belongs to configuration {component_configuration_id:?}, not campaign configuration {campaign_configuration_id:?}"
    )]
    ComponentConfigurationMismatch {
        component_id: String,
        component_configuration_id: String,
        campaign_configuration_id: String,
    },
    #[error("inertia tensor {field} is not symmetric")]
    NonSymmetricInertia { field: String },
    #[error("inertia tensor {field} is not positive definite")]
    NonPositiveDefiniteInertia { field: String },
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct MassMeasurementSummary {
    mean: f64,
    minimum: f64,
    maximum: f64,
    range: f64,
    effective_uncertainty: f64,
}

impl MassMeasurementSummary {
    pub const fn mean(&self) -> f64 {
        self.mean
    }
    pub const fn minimum(&self) -> f64 {
        self.minimum
    }
    pub const fn maximum(&self) -> f64 {
        self.maximum
    }
    pub const fn range(&self) -> f64 {
        self.range
    }
    pub const fn effective_uncertainty(&self) -> f64 {
        self.effective_uncertainty
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ScalarEstimate {
    value: f64,
    uncertainty: f64,
}

impl ScalarEstimate {
    pub const fn value(&self) -> f64 {
        self.value
    }
    pub const fn uncertainty(&self) -> f64 {
        self.uncertainty
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct VectorEstimate {
    value: [f64; 3],
    uncertainty: [f64; 3],
}

impl VectorEstimate {
    pub const fn value(&self) -> &[f64; 3] {
        &self.value
    }
    pub const fn uncertainty(&self) -> &[f64; 3] {
        &self.uncertainty
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct InertiaEstimate {
    matrix_frd_kg_m2: [[f64; 3]; 3],
    uncertainty_kg_m2: [[f64; 3]; 3],
}

impl InertiaEstimate {
    pub const fn matrix_frd_kg_m2(&self) -> &[[f64; 3]; 3] {
        &self.matrix_frd_kg_m2
    }
    pub const fn uncertainty_kg_m2(&self) -> &[[f64; 3]; 3] {
        &self.uncertainty_kg_m2
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum PublishedWeightRangeStatus {
    WithinPublishedRange,
    OutsidePublishedRange,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MassPropertiesEvaluation {
    direct_mass: Option<MassMeasurementSummary>,
    component_build_up_mass: Option<ScalarEstimate>,
    direct_cg_frd_m: Option<VectorEstimate>,
    component_build_up_cg_frd_m: Option<VectorEstimate>,
    direct_inertia_frd_kg_m2: Option<InertiaEstimate>,
    component_build_up_inertia_frd_kg_m2: Option<InertiaEstimate>,
    direct_mass_published_range: PublishedWeightRangeStatus,
    component_mass_published_range: PublishedWeightRangeStatus,
    missing_requirements: Vec<String>,
    configuration_identified: bool,
    mass_ready: bool,
    cg_ready: bool,
    inertia_ready: bool,
    mass_properties_ready: bool,
    runtime_ready: bool,
}

impl MassPropertiesEvaluation {
    pub const fn direct_mass(&self) -> Option<&MassMeasurementSummary> {
        self.direct_mass.as_ref()
    }
    pub const fn component_build_up_mass(&self) -> Option<ScalarEstimate> {
        self.component_build_up_mass
    }
    pub const fn direct_cg_frd_m(&self) -> Option<VectorEstimate> {
        self.direct_cg_frd_m
    }
    pub const fn component_build_up_cg_frd_m(&self) -> Option<VectorEstimate> {
        self.component_build_up_cg_frd_m
    }
    pub const fn direct_inertia_frd_kg_m2(&self) -> Option<&InertiaEstimate> {
        self.direct_inertia_frd_kg_m2.as_ref()
    }
    pub const fn component_build_up_inertia_frd_kg_m2(&self) -> Option<&InertiaEstimate> {
        self.component_build_up_inertia_frd_kg_m2.as_ref()
    }
    pub const fn direct_mass_published_range(&self) -> PublishedWeightRangeStatus {
        self.direct_mass_published_range
    }
    pub const fn component_mass_published_range(&self) -> PublishedWeightRangeStatus {
        self.component_mass_published_range
    }
    pub fn missing_requirements(&self) -> &[String] {
        &self.missing_requirements
    }
    pub const fn configuration_identified(&self) -> bool {
        self.configuration_identified
    }
    pub const fn mass_ready(&self) -> bool {
        self.mass_ready
    }
    pub const fn cg_ready(&self) -> bool {
        self.cg_ready
    }
    pub const fn inertia_ready(&self) -> bool {
        self.inertia_ready
    }
    pub const fn mass_properties_ready(&self) -> bool {
        self.mass_properties_ready
    }
    /// M2.2D never promotes evidence into runtime configuration.
    pub const fn runtime_ready(&self) -> bool {
        self.runtime_ready
    }
}

#[derive(Debug, Clone)]
pub struct MassPropertiesCampaign {
    file: MassPropertiesFile,
    evaluation: MassPropertiesEvaluation,
}

impl MassPropertiesCampaign {
    pub const fn classification(&self) -> SurveyClassification {
        self.file.campaign.classification
    }
    pub fn campaign_id(&self) -> &str {
        &self.file.campaign.id
    }
    pub fn operational_configuration_id(&self) -> Option<&str> {
        self.file.campaign.operational_configuration.id.as_deref()
    }
    pub const fn evaluation(&self) -> &MassPropertiesEvaluation {
        &self.evaluation
    }
}

pub struct MassPropertiesLoader;

impl MassPropertiesLoader {
    pub fn from_json_str(
        json: &str,
    ) -> Result<MassPropertiesCampaign, ReferenceMassPropertiesError> {
        let file: MassPropertiesFile = serde_json::from_str(json)
            .map_err(|source| ReferenceMassPropertiesError::InvalidStructure { source })?;
        validate_file(&file)?;
        let evaluation = evaluate(&file)?;
        Ok(MassPropertiesCampaign { file, evaluation })
    }
}

pub fn load_reference_mass_properties(
    path: impl AsRef<Path>,
) -> Result<MassPropertiesCampaign, ReferenceMassPropertiesError> {
    let path = path.as_ref();
    let json = fs::read_to_string(path).map_err(|source| ReferenceMassPropertiesError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    MassPropertiesLoader::from_json_str(&json)
}

pub fn x_aft_to_frd_x(x_aft_m: f64) -> Result<f64, ReferenceMassPropertiesError> {
    if !x_aft_m.is_finite() {
        return Err(ReferenceMassPropertiesError::InvalidMeasurement {
            field: "x_aft_m".to_owned(),
            reason: "coordinate must be finite",
        });
    }
    Ok(-x_aft_m)
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MassPropertiesFile {
    schema: String,
    artifact_kind: String,
    campaign: CampaignFile,
    coordinate_frame: CoordinateFrameFile,
    provenance_sources: Vec<SourceFile>,
    photographs: Vec<PhotographFile>,
    acceptance_criteria: AcceptanceCriteriaFile,
    published_weight_range_comparison: PublishedWeightRangeFile,
    raw_observations: RawObservationsFile,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignFile {
    id: String,
    classification: SurveyClassification,
    identity: IdentityFile,
    measurement_date: Option<String>,
    linked_geometry_campaign_id: String,
    operational_configuration: OperationalConfigurationFile,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct IdentityFile {
    manufacturer: String,
    family: String,
    variant: String,
    airframe_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct OperationalConfigurationFile {
    id: Option<String>,
    battery_configuration_id: Option<String>,
    propulsion_configuration_description: Option<String>,
    landing_gear_configuration: Option<String>,
    installed_equipment_notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoordinateFrameFile {
    axes_parallel_to_frd: bool,
    origin_definition: String,
    positive_x_direction: AxisDirectionX,
    positive_y_direction: AxisDirectionY,
    positive_z_direction: AxisDirectionZ,
    wing_root_le_center_plane_datum_established: bool,
    lateral_datum_established: bool,
    lateral_datum_definition: String,
    vertical_datum_established: bool,
    vertical_datum_definition: String,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum AxisDirectionX {
    Forward,
}
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum AxisDirectionY {
    Right,
}
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum AxisDirectionZ {
    Down,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SourceKind {
    MeasurementSession,
    InstrumentCalibration,
    ManufacturerDocumentation,
    DerivedReference,
    CadMassModel,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SourceFile {
    id: String,
    kind: SourceKind,
    title: String,
    url: Option<String>,
    sha256: Option<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PhotographFile {
    id: String,
    path: Option<String>,
    url: Option<String>,
    sha256: Option<String>,
    description: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct AcceptanceCriteriaFile {
    maximum_direct_vs_build_up_mass_difference_kg: Option<f64>,
    maximum_direct_vs_build_up_cg_distance_m: Option<f64>,
    maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PublishedWeightRangeFile {
    minimum_kg: f64,
    maximum_kg: f64,
    source_ids: Vec<String>,
    authority: PublishedRangeAuthority,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum PublishedRangeAuthority {
    ComparisonOnlyNeverOperationalMass,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawObservationsFile {
    direct_total_mass_kg: Option<MeasurementSeriesFile>,
    direct_cg_position_frd_m: VectorObservationFile,
    direct_inertia_about_operational_cg_frd_kg_m2: Option<TensorObservationFile>,
    component_inventory_complete: bool,
    components: Vec<ComponentFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct VectorObservationFile {
    x: Option<MeasurementSeriesFile>,
    y: Option<MeasurementSeriesFile>,
    z: Option<MeasurementSeriesFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MeasurementSeriesFile {
    readings: [f64; 3],
    instrument_resolution: f64,
    stated_uncertainty: f64,
    datum_or_method_definition: String,
    notes: Option<String>,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum InertiaMethodClass {
    PhysicalPendulum,
    BifilarSuspension,
    TrifilarSuspension,
    EvidencedCadMassModel,
    OtherDocumentedMethod,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TensorObservationFile {
    method_class: InertiaMethodClass,
    method_definition: String,
    matrix_entries: TensorEntriesFile,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct TensorEntriesFile {
    ixx: MeasurementSeriesFile,
    ixy: MeasurementSeriesFile,
    ixz: MeasurementSeriesFile,
    iyx: MeasurementSeriesFile,
    iyy: MeasurementSeriesFile,
    iyz: MeasurementSeriesFile,
    izx: MeasurementSeriesFile,
    izy: MeasurementSeriesFile,
    izz: MeasurementSeriesFile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ComponentStatus {
    Installed,
    ReferenceOnly,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ComponentFile {
    id: String,
    category: String,
    description: String,
    status: ComponentStatus,
    configuration_id: Option<String>,
    mass_kg: Option<MeasurementSeriesFile>,
    cg_position_frd_m: VectorObservationFile,
    intrinsic_inertia_about_component_cg_frd_kg_m2: Option<TensorObservationFile>,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Clone, Copy)]
enum MeasurementDomain {
    Positive,
    Signed,
}

fn validate_file(file: &MassPropertiesFile) -> Result<(), ReferenceMassPropertiesError> {
    if file.schema != REFERENCE_MASS_PROPERTIES_SCHEMA_V0 {
        return Err(ReferenceMassPropertiesError::UnsupportedSchema {
            found: file.schema.clone(),
        });
    }
    if file.artifact_kind != ARTIFACT_KIND {
        return Err(ReferenceMassPropertiesError::InvalidArtifactKind {
            found: file.artifact_kind.clone(),
        });
    }
    validate_campaign(&file.campaign)?;
    validate_frame_text(&file.coordinate_frame)?;
    let source_ids = validate_sources(&file.provenance_sources)?;
    let photograph_ids = validate_photographs(&file.photographs)?;
    validate_refs(
        "coordinate_frame",
        &file.coordinate_frame.source_ids,
        &file.coordinate_frame.photograph_ids,
        &source_ids,
        &photograph_ids,
    )?;
    validate_acceptance_criteria(&file.acceptance_criteria)?;
    validate_published_range(&file.published_weight_range_comparison, &source_ids)?;
    validate_observations(
        &file.raw_observations,
        file.campaign.operational_configuration.id.as_deref(),
        &source_ids,
        &photograph_ids,
    )
}

fn validate_campaign(campaign: &CampaignFile) -> Result<(), ReferenceMassPropertiesError> {
    validate_stable_id("campaign", &campaign.id)?;
    validate_required_text(
        "campaign.identity.manufacturer",
        &campaign.identity.manufacturer,
    )?;
    validate_required_text("campaign.identity.family", &campaign.identity.family)?;
    validate_required_text("campaign.identity.variant", &campaign.identity.variant)?;
    if let Some(id) = campaign.identity.airframe_id.as_deref() {
        validate_stable_id("airframe", id)?;
    }
    if !campaign.linked_geometry_campaign_id.is_empty() {
        validate_stable_id(
            "linked geometry campaign",
            &campaign.linked_geometry_campaign_id,
        )?;
    }
    if let Some(date) = campaign.measurement_date.as_deref()
        && !is_iso_date(date)
    {
        return Err(ReferenceMassPropertiesError::InvalidMetadata {
            field: "campaign.measurement_date",
            reason: "expected a real YYYY-MM-DD calendar date",
        });
    }
    let config = &campaign.operational_configuration;
    if let Some(id) = config.id.as_deref() {
        validate_stable_id("operational configuration", id)?;
    }
    if let Some(id) = config.battery_configuration_id.as_deref() {
        validate_stable_id("battery configuration", id)?;
    }
    validate_optional_text(
        "campaign.operational_configuration.propulsion_configuration_description",
        config.propulsion_configuration_description.as_deref(),
    )?;
    validate_optional_text(
        "campaign.operational_configuration.landing_gear_configuration",
        config.landing_gear_configuration.as_deref(),
    )?;
    validate_optional_text(
        "campaign.operational_configuration.installed_equipment_notes",
        config.installed_equipment_notes.as_deref(),
    )?;
    validate_optional_text("campaign.notes", campaign.notes.as_deref())
}

fn validate_frame_text(frame: &CoordinateFrameFile) -> Result<(), ReferenceMassPropertiesError> {
    validate_required_text(
        "coordinate_frame.origin_definition",
        &frame.origin_definition,
    )?;
    validate_required_text(
        "coordinate_frame.lateral_datum_definition",
        &frame.lateral_datum_definition,
    )?;
    validate_required_text(
        "coordinate_frame.vertical_datum_definition",
        &frame.vertical_datum_definition,
    )?;
    let _ = (
        frame.positive_x_direction,
        frame.positive_y_direction,
        frame.positive_z_direction,
    );
    Ok(())
}

fn validate_sources(
    sources: &[SourceFile],
) -> Result<HashSet<String>, ReferenceMassPropertiesError> {
    let mut ids = HashSet::new();
    for source in sources {
        validate_stable_id("provenance source", &source.id)?;
        if !ids.insert(source.id.clone()) {
            return Err(ReferenceMassPropertiesError::DuplicateStableId {
                kind: "provenance source",
                value: source.id.clone(),
            });
        }
        validate_required_text("provenance_sources.title", &source.title)?;
        validate_optional_text("provenance_sources.url", source.url.as_deref())?;
        validate_optional_text("provenance_sources.notes", source.notes.as_deref())?;
        validate_sha256("provenance_sources.sha256", source.sha256.as_deref())?;
        let _ = source.kind;
    }
    Ok(ids)
}

fn validate_photographs(
    photographs: &[PhotographFile],
) -> Result<HashSet<String>, ReferenceMassPropertiesError> {
    let mut ids = HashSet::new();
    for photograph in photographs {
        validate_stable_id("photograph", &photograph.id)?;
        if !ids.insert(photograph.id.clone()) {
            return Err(ReferenceMassPropertiesError::DuplicateStableId {
                kind: "photograph",
                value: photograph.id.clone(),
            });
        }
        if photograph
            .path
            .as_deref()
            .is_none_or(|value| value.trim().is_empty())
            && photograph
                .url
                .as_deref()
                .is_none_or(|value| value.trim().is_empty())
        {
            return Err(ReferenceMassPropertiesError::InvalidMetadata {
                field: "photographs",
                reason: "each photograph needs a nonempty path or URL",
            });
        }
        validate_required_text("photographs.description", &photograph.description)?;
        validate_sha256("photographs.sha256", photograph.sha256.as_deref())?;
    }
    Ok(ids)
}

fn validate_acceptance_criteria(
    criteria: &AcceptanceCriteriaFile,
) -> Result<(), ReferenceMassPropertiesError> {
    for (field, value) in [
        (
            "acceptance_criteria.maximum_direct_vs_build_up_mass_difference_kg",
            criteria.maximum_direct_vs_build_up_mass_difference_kg,
        ),
        (
            "acceptance_criteria.maximum_direct_vs_build_up_cg_distance_m",
            criteria.maximum_direct_vs_build_up_cg_distance_m,
        ),
        (
            "acceptance_criteria.maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2",
            criteria.maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2,
        ),
    ] {
        if value.is_some_and(|value| !value.is_finite() || value < 0.0) {
            return Err(ReferenceMassPropertiesError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "criterion must be finite and nonnegative",
            });
        }
    }
    Ok(())
}

fn validate_published_range(
    range: &PublishedWeightRangeFile,
    source_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    if !range.minimum_kg.is_finite()
        || !range.maximum_kg.is_finite()
        || range.minimum_kg <= 0.0
        || range.maximum_kg <= range.minimum_kg
    {
        return Err(ReferenceMassPropertiesError::InvalidMeasurement {
            field: "published_weight_range_comparison".to_owned(),
            reason: "published comparison range must be finite, positive, and ordered",
        });
    }
    validate_source_refs(
        "published_weight_range_comparison",
        &range.source_ids,
        source_ids,
    )?;
    let _ = range.authority;
    Ok(())
}

fn validate_observations(
    observations: &RawObservationsFile,
    campaign_configuration_id: Option<&str>,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    validate_optional_series(
        "raw_observations.direct_total_mass_kg",
        observations.direct_total_mass_kg.as_ref(),
        MeasurementDomain::Positive,
        source_ids,
        photograph_ids,
    )?;
    validate_vector(
        "raw_observations.direct_cg_position_frd_m",
        &observations.direct_cg_position_frd_m,
        source_ids,
        photograph_ids,
    )?;
    if let Some(tensor) = observations
        .direct_inertia_about_operational_cg_frd_kg_m2
        .as_ref()
    {
        validate_tensor(
            "raw_observations.direct_inertia_about_operational_cg_frd_kg_m2",
            tensor,
            true,
            source_ids,
            photograph_ids,
        )?;
    }
    let mut component_ids = HashSet::new();
    for component in &observations.components {
        validate_stable_id("component", &component.id)?;
        if !component_ids.insert(component.id.clone()) {
            return Err(ReferenceMassPropertiesError::DuplicateStableId {
                kind: "component",
                value: component.id.clone(),
            });
        }
        validate_required_text("component.category", &component.category)?;
        validate_required_text("component.description", &component.description)?;
        validate_optional_text("component.notes", component.notes.as_deref())?;
        if let Some(id) = component.configuration_id.as_deref() {
            validate_stable_id("component configuration", id)?;
        }
        if component.status == ComponentStatus::Installed {
            let component_configuration_id = component.configuration_id.as_deref().unwrap_or("");
            let campaign_configuration_id = campaign_configuration_id.unwrap_or("");
            if component_configuration_id != campaign_configuration_id
                || component_configuration_id.is_empty()
            {
                return Err(
                    ReferenceMassPropertiesError::ComponentConfigurationMismatch {
                        component_id: component.id.clone(),
                        component_configuration_id: component_configuration_id.to_owned(),
                        campaign_configuration_id: campaign_configuration_id.to_owned(),
                    },
                );
            }
        }
        validate_refs(
            &format!("component.{}", component.id),
            &component.source_ids,
            &component.photograph_ids,
            source_ids,
            photograph_ids,
        )?;
        validate_optional_series(
            &format!("component.{}.mass_kg", component.id),
            component.mass_kg.as_ref(),
            MeasurementDomain::Positive,
            source_ids,
            photograph_ids,
        )?;
        validate_vector(
            &format!("component.{}.cg_position_frd_m", component.id),
            &component.cg_position_frd_m,
            source_ids,
            photograph_ids,
        )?;
        if let Some(tensor) = component
            .intrinsic_inertia_about_component_cg_frd_kg_m2
            .as_ref()
        {
            validate_tensor(
                &format!(
                    "component.{}.intrinsic_inertia_about_component_cg_frd_kg_m2",
                    component.id
                ),
                tensor,
                false,
                source_ids,
                photograph_ids,
            )?;
        }
    }
    Ok(())
}

fn validate_vector(
    field: &str,
    vector: &VectorObservationFile,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    for (axis, series) in [
        ("x", vector.x.as_ref()),
        ("y", vector.y.as_ref()),
        ("z", vector.z.as_ref()),
    ] {
        validate_optional_series(
            &format!("{field}.{axis}"),
            series,
            MeasurementDomain::Signed,
            source_ids,
            photograph_ids,
        )?;
    }
    Ok(())
}

fn validate_tensor(
    field: &str,
    tensor: &TensorObservationFile,
    require_positive_definite: bool,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    validate_required_text("inertia.method_definition", &tensor.method_definition)?;
    validate_optional_text("inertia.notes", tensor.notes.as_deref())?;
    validate_refs(
        field,
        &tensor.source_ids,
        &tensor.photograph_ids,
        source_ids,
        photograph_ids,
    )?;
    let entries = &tensor.matrix_entries;
    for (name, series, domain) in [
        ("ixx", &entries.ixx, MeasurementDomain::Positive),
        ("ixy", &entries.ixy, MeasurementDomain::Signed),
        ("ixz", &entries.ixz, MeasurementDomain::Signed),
        ("iyx", &entries.iyx, MeasurementDomain::Signed),
        ("iyy", &entries.iyy, MeasurementDomain::Positive),
        ("iyz", &entries.iyz, MeasurementDomain::Signed),
        ("izx", &entries.izx, MeasurementDomain::Signed),
        ("izy", &entries.izy, MeasurementDomain::Signed),
        ("izz", &entries.izz, MeasurementDomain::Positive),
    ] {
        validate_series(
            &format!("{field}.matrix_entries.{name}"),
            series,
            domain,
            source_ids,
            photograph_ids,
        )?;
    }
    let raw_matrix = raw_tensor_matrix(tensor);
    if !is_symmetric(&raw_matrix) {
        return Err(ReferenceMassPropertiesError::NonSymmetricInertia {
            field: field.to_owned(),
        });
    }
    if require_positive_definite && !is_positive_definite(&raw_matrix) {
        return Err(ReferenceMassPropertiesError::NonPositiveDefiniteInertia {
            field: field.to_owned(),
        });
    }
    let _ = tensor.method_class;
    Ok(())
}

fn validate_optional_series(
    field: &str,
    series: Option<&MeasurementSeriesFile>,
    domain: MeasurementDomain,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    if let Some(series) = series {
        validate_series(field, series, domain, source_ids, photograph_ids)?;
    }
    Ok(())
}

fn validate_series(
    field: &str,
    series: &MeasurementSeriesFile,
    domain: MeasurementDomain,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    if !series.instrument_resolution.is_finite() || series.instrument_resolution <= 0.0 {
        return Err(ReferenceMassPropertiesError::InvalidMeasurement {
            field: field.to_owned(),
            reason: "instrument resolution must be finite and positive",
        });
    }
    if !series.stated_uncertainty.is_finite() || series.stated_uncertainty < 0.0 {
        return Err(ReferenceMassPropertiesError::InvalidMeasurement {
            field: field.to_owned(),
            reason: "stated uncertainty must be finite and nonnegative",
        });
    }
    for value in series.readings {
        let valid = match domain {
            MeasurementDomain::Positive => value.is_finite() && value > 0.0,
            MeasurementDomain::Signed => value.is_finite(),
        };
        if !valid {
            return Err(ReferenceMassPropertiesError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "reading is non-finite or outside its physical domain",
            });
        }
    }
    validate_required_text(
        "measurement.datum_or_method_definition",
        &series.datum_or_method_definition,
    )?;
    validate_optional_text("measurement.notes", series.notes.as_deref())?;
    validate_refs(
        field,
        &series.source_ids,
        &series.photograph_ids,
        source_ids,
        photograph_ids,
    )
}

fn validate_refs(
    field: &str,
    source_refs: &[String],
    photograph_refs: &[String],
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    validate_reference_list(field, source_refs, source_ids, true)?;
    validate_reference_list(field, photograph_refs, photograph_ids, false)
}

fn validate_source_refs(
    field: &str,
    refs: &[String],
    source_ids: &HashSet<String>,
) -> Result<(), ReferenceMassPropertiesError> {
    validate_reference_list(field, refs, source_ids, true)
}

fn validate_reference_list(
    field: &str,
    refs: &[String],
    known: &HashSet<String>,
    sources: bool,
) -> Result<(), ReferenceMassPropertiesError> {
    let mut seen = HashSet::new();
    for reference in refs {
        if !is_valid_stable_id(reference) {
            return Err(ReferenceMassPropertiesError::InvalidStableId {
                kind: "evidence reference",
                value: reference.clone(),
            });
        }
        if !known.contains(reference) {
            return Err(if sources {
                ReferenceMassPropertiesError::UnresolvedSourceReference {
                    field: field.to_owned(),
                    source_id: reference.clone(),
                }
            } else {
                ReferenceMassPropertiesError::UnresolvedPhotographReference {
                    field: field.to_owned(),
                    photograph_id: reference.clone(),
                }
            });
        }
        if !seen.insert(reference) {
            return Err(ReferenceMassPropertiesError::DuplicateEvidenceReference {
                field: field.to_owned(),
                reference_id: reference.clone(),
            });
        }
    }
    Ok(())
}

fn validate_stable_id(kind: &'static str, value: &str) -> Result<(), ReferenceMassPropertiesError> {
    if is_valid_stable_id(value) {
        Ok(())
    } else {
        Err(ReferenceMassPropertiesError::InvalidStableId {
            kind,
            value: value.to_owned(),
        })
    }
}

fn is_valid_stable_id(value: &str) -> bool {
    !value.is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"_-".contains(&byte))
}

fn validate_required_text(
    field: &'static str,
    value: &str,
) -> Result<(), ReferenceMassPropertiesError> {
    if value.trim().is_empty() {
        Err(ReferenceMassPropertiesError::InvalidMetadata {
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
) -> Result<(), ReferenceMassPropertiesError> {
    if value.is_some_and(|value| value.trim().is_empty()) {
        Err(ReferenceMassPropertiesError::InvalidMetadata {
            field,
            reason: "present text must not be empty or whitespace",
        })
    } else {
        Ok(())
    }
}

fn validate_sha256(
    field: &'static str,
    value: Option<&str>,
) -> Result<(), ReferenceMassPropertiesError> {
    if value.is_some_and(|value| {
        value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit())
    }) {
        Err(ReferenceMassPropertiesError::InvalidMetadata {
            field,
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
    let year = value[0..4].parse::<u16>().ok();
    let month = value[5..7].parse::<u8>().ok();
    let day = value[8..10].parse::<u8>().ok();
    let Some((year, month, day)) = year.zip(month).zip(day).map(|((y, m), d)| (y, m, d)) else {
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

fn summarize(series: &MeasurementSeriesFile) -> MassMeasurementSummary {
    let [a, b, c] = series.readings;
    let minimum = a.min(b).min(c);
    let maximum = a.max(b).max(c);
    let range = maximum - minimum;
    MassMeasurementSummary {
        mean: (a + b + c) / 3.0,
        minimum,
        maximum,
        range,
        effective_uncertainty: series
            .stated_uncertainty
            .max(series.instrument_resolution / 2.0)
            .max(range / 2.0),
    }
}

fn estimate(series: &MeasurementSeriesFile) -> ScalarEstimate {
    let summary = summarize(series);
    ScalarEstimate {
        value: summary.mean,
        uncertainty: summary.effective_uncertainty,
    }
}

fn summarize_vector(vector: &VectorObservationFile) -> Option<VectorEstimate> {
    let values = [
        estimate(vector.x.as_ref()?),
        estimate(vector.y.as_ref()?),
        estimate(vector.z.as_ref()?),
    ];
    Some(VectorEstimate {
        value: [values[0].value, values[1].value, values[2].value],
        uncertainty: [
            values[0].uncertainty,
            values[1].uncertainty,
            values[2].uncertainty,
        ],
    })
}

fn summarize_tensor(tensor: &TensorObservationFile) -> InertiaEstimate {
    let entries = &tensor.matrix_entries;
    let values = [
        [
            estimate(&entries.ixx),
            estimate(&entries.ixy),
            estimate(&entries.ixz),
        ],
        [
            estimate(&entries.iyx),
            estimate(&entries.iyy),
            estimate(&entries.iyz),
        ],
        [
            estimate(&entries.izx),
            estimate(&entries.izy),
            estimate(&entries.izz),
        ],
    ];
    let mut matrix = [[0.0; 3]; 3];
    let mut uncertainty = [[0.0; 3]; 3];
    for row in 0..3 {
        for column in 0..3 {
            matrix[row][column] = values[row][column].value;
            uncertainty[row][column] = values[row][column].uncertainty;
        }
    }
    for (a, b) in [(0, 1), (0, 2), (1, 2)] {
        let average = (matrix[a][b] + matrix[b][a]) / 2.0;
        let pair_uncertainty = uncertainty[a][b]
            .max(uncertainty[b][a])
            .max((matrix[a][b] - matrix[b][a]).abs() / 2.0);
        matrix[a][b] = average;
        matrix[b][a] = average;
        uncertainty[a][b] = pair_uncertainty;
        uncertainty[b][a] = pair_uncertainty;
    }
    InertiaEstimate {
        matrix_frd_kg_m2: matrix,
        uncertainty_kg_m2: uncertainty,
    }
}

fn raw_tensor_matrix(tensor: &TensorObservationFile) -> [[f64; 3]; 3] {
    let entries = &tensor.matrix_entries;
    [
        [
            estimate(&entries.ixx).value,
            estimate(&entries.ixy).value,
            estimate(&entries.ixz).value,
        ],
        [
            estimate(&entries.iyx).value,
            estimate(&entries.iyy).value,
            estimate(&entries.iyz).value,
        ],
        [
            estimate(&entries.izx).value,
            estimate(&entries.izy).value,
            estimate(&entries.izz).value,
        ],
    ]
}

fn tensor_has_evidence(tensor: &TensorObservationFile) -> bool {
    !tensor.source_ids.is_empty()
        && !tensor.photograph_ids.is_empty()
        && tensor_entries(tensor).into_iter().all(series_has_evidence)
}

fn tensor_entries(tensor: &TensorObservationFile) -> [&MeasurementSeriesFile; 9] {
    let entries = &tensor.matrix_entries;
    [
        &entries.ixx,
        &entries.ixy,
        &entries.ixz,
        &entries.iyx,
        &entries.iyy,
        &entries.iyz,
        &entries.izx,
        &entries.izy,
        &entries.izz,
    ]
}

fn series_has_evidence(series: &MeasurementSeriesFile) -> bool {
    !series.source_ids.is_empty() && !series.photograph_ids.is_empty()
}

fn vector_has_evidence(vector: &VectorObservationFile) -> bool {
    [vector.x.as_ref(), vector.y.as_ref(), vector.z.as_ref()]
        .into_iter()
        .all(|series| series.is_some_and(series_has_evidence))
}

#[derive(Debug, Clone)]
struct ResolvedComponent {
    id: String,
    mass: Option<ScalarEstimate>,
    position: Option<VectorEstimate>,
    intrinsic_inertia: Option<InertiaEstimate>,
}

fn resolve_installed_components(file: &MassPropertiesFile) -> Vec<ResolvedComponent> {
    file.raw_observations
        .components
        .iter()
        .filter(|component| component.status == ComponentStatus::Installed)
        .map(|component| {
            let top_evidence =
                !component.source_ids.is_empty() && !component.photograph_ids.is_empty();
            ResolvedComponent {
                id: component.id.clone(),
                mass: component
                    .mass_kg
                    .as_ref()
                    .filter(|series| top_evidence && series_has_evidence(series))
                    .map(estimate),
                position: top_evidence
                    .then(|| summarize_vector(&component.cg_position_frd_m))
                    .flatten()
                    .filter(|_| vector_has_evidence(&component.cg_position_frd_m)),
                intrinsic_inertia: component
                    .intrinsic_inertia_about_component_cg_frd_kg_m2
                    .as_ref()
                    .filter(|tensor| top_evidence && tensor_has_evidence(tensor))
                    .map(summarize_tensor),
            }
        })
        .collect()
}

fn derive_mass(
    components: &[ResolvedComponent],
    inventory_complete: bool,
) -> Option<ScalarEstimate> {
    if !inventory_complete
        || components.is_empty()
        || components.iter().any(|item| item.mass.is_none())
    {
        return None;
    }
    let mut value = 0.0;
    let mut uncertainty_squared = 0.0;
    for component in components {
        let mass = component.mass?;
        value += mass.value;
        uncertainty_squared += mass.uncertainty.powi(2);
    }
    Some(ScalarEstimate {
        value,
        uncertainty: uncertainty_squared.sqrt(),
    })
}

fn derive_cg(components: &[ResolvedComponent], inventory_complete: bool) -> Option<VectorEstimate> {
    if !inventory_complete
        || components.is_empty()
        || components
            .iter()
            .any(|item| item.mass.is_none() || item.position.is_none())
    {
        return None;
    }
    let mass = derive_mass(components, true)?;
    let value = compute_cg(components)?;
    let mut uncertainty_squared = [0.0; 3];
    for component in components {
        let component_mass = component.mass?;
        let position = component.position?;
        for axis in 0..3 {
            let mass_contribution =
                (position.value[axis] - value[axis]) / mass.value * component_mass.uncertainty;
            let position_contribution =
                component_mass.value / mass.value * position.uncertainty[axis];
            uncertainty_squared[axis] += mass_contribution.powi(2) + position_contribution.powi(2);
        }
    }
    Some(VectorEstimate {
        value,
        uncertainty: uncertainty_squared.map(f64::sqrt),
    })
}

fn compute_cg(components: &[ResolvedComponent]) -> Option<[f64; 3]> {
    let mut total_mass = 0.0;
    let mut moment = [0.0; 3];
    for component in components {
        let mass = component.mass?.value;
        let position = component.position?.value;
        total_mass += mass;
        for axis in 0..3 {
            moment[axis] += mass * position[axis];
        }
    }
    (total_mass > 0.0).then(|| moment.map(|value| value / total_mass))
}

fn derive_inertia(
    components: &[ResolvedComponent],
    inventory_complete: bool,
) -> Result<Option<InertiaEstimate>, ReferenceMassPropertiesError> {
    if !inventory_complete
        || components.is_empty()
        || components.iter().any(|item| {
            item.mass.is_none() || item.position.is_none() || item.intrinsic_inertia.is_none()
        })
    {
        return Ok(None);
    }
    let nominal = compute_inertia_matrix(components).expect("complete components derive inertia");
    if !is_positive_definite(&nominal) {
        return Err(ReferenceMassPropertiesError::NonPositiveDefiniteInertia {
            field: "component_build_up_inertia_frd_kg_m2".to_owned(),
        });
    }
    let mut squared = [[0.0; 3]; 3];
    for index in 0..components.len() {
        let component = &components[index];
        let mass = component.mass.expect("complete mass");
        accumulate_perturbation(
            components,
            &nominal,
            &mut squared,
            index,
            Perturbation::Mass(mass.uncertainty),
        );
        let position = component.position.expect("complete position");
        for axis in 0..3 {
            accumulate_perturbation(
                components,
                &nominal,
                &mut squared,
                index,
                Perturbation::Position(axis, position.uncertainty[axis]),
            );
        }
        let intrinsic = component
            .intrinsic_inertia
            .expect("complete intrinsic inertia");
        for (row, column) in [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)] {
            accumulate_perturbation(
                components,
                &nominal,
                &mut squared,
                index,
                Perturbation::Intrinsic(row, column, intrinsic.uncertainty_kg_m2[row][column]),
            );
        }
    }
    Ok(Some(InertiaEstimate {
        matrix_frd_kg_m2: nominal,
        uncertainty_kg_m2: squared.map(|row| row.map(f64::sqrt)),
    }))
}

#[derive(Clone, Copy)]
enum Perturbation {
    Mass(f64),
    Position(usize, f64),
    Intrinsic(usize, usize, f64),
}

fn accumulate_perturbation(
    components: &[ResolvedComponent],
    nominal: &[[f64; 3]; 3],
    squared: &mut [[f64; 3]; 3],
    index: usize,
    perturbation: Perturbation,
) {
    let uncertainty = match perturbation {
        Perturbation::Mass(value)
        | Perturbation::Position(_, value)
        | Perturbation::Intrinsic(_, _, value) => value,
    };
    if uncertainty == 0.0 {
        return;
    }
    let mut deviations = [[0.0_f64; 3]; 3];
    for sign in [-1.0, 1.0] {
        let mut changed = components.to_vec();
        match perturbation {
            Perturbation::Mass(value) => {
                let mass = changed[index].mass.as_mut().expect("complete mass");
                mass.value += sign * value;
                if mass.value <= 0.0 {
                    continue;
                }
            }
            Perturbation::Position(axis, value) => {
                changed[index]
                    .position
                    .as_mut()
                    .expect("complete position")
                    .value[axis] += sign * value;
            }
            Perturbation::Intrinsic(row, column, value) => {
                let matrix = &mut changed[index]
                    .intrinsic_inertia
                    .as_mut()
                    .expect("complete intrinsic")
                    .matrix_frd_kg_m2;
                matrix[row][column] += sign * value;
                matrix[column][row] = matrix[row][column];
            }
        }
        if let Some(candidate) = compute_inertia_matrix(&changed) {
            for row in 0..3 {
                for column in 0..3 {
                    deviations[row][column] = deviations[row][column]
                        .max((candidate[row][column] - nominal[row][column]).abs());
                }
            }
        }
    }
    for row in 0..3 {
        for column in 0..3 {
            squared[row][column] += deviations[row][column].powi(2);
        }
    }
}

fn compute_inertia_matrix(components: &[ResolvedComponent]) -> Option<[[f64; 3]; 3]> {
    let cg = compute_cg(components)?;
    let mut result = [[0.0; 3]; 3];
    for component in components {
        let mass = component.mass?.value;
        let position = component.position?.value;
        let intrinsic = component.intrinsic_inertia?.matrix_frd_kg_m2;
        let r = [
            position[0] - cg[0],
            position[1] - cg[1],
            position[2] - cg[2],
        ];
        let radius_squared = dot(r, r);
        for row in 0..3 {
            for column in 0..3 {
                let identity = if row == column { 1.0 } else { 0.0 };
                result[row][column] += intrinsic[row][column]
                    + mass * (radius_squared * identity - r[row] * r[column]);
            }
        }
    }
    Some(result)
}

fn evaluate(
    file: &MassPropertiesFile,
) -> Result<MassPropertiesEvaluation, ReferenceMassPropertiesError> {
    let observations = &file.raw_observations;
    let direct_mass = observations
        .direct_total_mass_kg
        .as_ref()
        .and_then(|series| series_has_evidence(series).then(|| summarize(series)));
    let direct_cg = vector_has_evidence(&observations.direct_cg_position_frd_m)
        .then(|| summarize_vector(&observations.direct_cg_position_frd_m))
        .flatten();
    let direct_inertia = observations
        .direct_inertia_about_operational_cg_frd_kg_m2
        .as_ref()
        .filter(|tensor| tensor_has_evidence(tensor))
        .map(summarize_tensor);

    let components = resolve_installed_components(file);
    let component_mass = derive_mass(&components, observations.component_inventory_complete);
    let component_cg = derive_cg(&components, observations.component_inventory_complete);
    let component_inertia = derive_inertia(&components, observations.component_inventory_complete)?;

    let mut missing = Vec::new();
    let configuration_identified = configuration_readiness(file, &mut missing);

    let mass_consistent = scalar_consistency(
        direct_mass.as_ref().map(|summary| ScalarEstimate {
            value: summary.mean,
            uncertainty: summary.effective_uncertainty,
        }),
        component_mass,
        file.acceptance_criteria
            .maximum_direct_vs_build_up_mass_difference_kg,
        "maximum_direct_vs_build_up_mass_difference_kg",
        "direct_vs_build_up_mass_consistency",
        &mut missing,
    );
    let mass_ready = (direct_mass.is_some() || component_mass.is_some()) && mass_consistent;
    if direct_mass.is_none() && component_mass.is_none() {
        component_path_blockers(file, &components, ComponentRequirement::Mass, &mut missing);
        missing.push("direct_total_mass_or_complete_component_mass_inventory".to_owned());
    }

    let cg_consistent = vector_consistency(
        direct_cg,
        component_cg,
        file.acceptance_criteria
            .maximum_direct_vs_build_up_cg_distance_m,
        "maximum_direct_vs_build_up_cg_distance_m",
        "direct_vs_build_up_cg_consistency",
        &mut missing,
    );
    let cg_ready = (direct_cg.is_some() || component_cg.is_some()) && cg_consistent;
    if direct_cg.is_none() && component_cg.is_none() {
        component_path_blockers(file, &components, ComponentRequirement::Cg, &mut missing);
        missing.push("direct_3d_cg_or_complete_component_mass_position_inventory".to_owned());
    }

    let inertia_consistent = tensor_consistency(
        direct_inertia,
        component_inertia,
        file.acceptance_criteria
            .maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2,
        &mut missing,
    );
    let inertia_ready =
        (direct_inertia.is_some() || component_inertia.is_some()) && inertia_consistent;
    if direct_inertia.is_none() && component_inertia.is_none() {
        component_path_blockers(
            file,
            &components,
            ComponentRequirement::Inertia,
            &mut missing,
        );
        missing.push(
            "direct_full_inertia_or_complete_component_intrinsic_inertia_inventory".to_owned(),
        );
    }
    deduplicate(&mut missing);
    let mass_properties_ready = configuration_identified && mass_ready && cg_ready && inertia_ready;
    let range = &file.published_weight_range_comparison;

    Ok(MassPropertiesEvaluation {
        direct_mass,
        component_build_up_mass: component_mass,
        direct_cg_frd_m: direct_cg,
        component_build_up_cg_frd_m: component_cg,
        direct_inertia_frd_kg_m2: direct_inertia,
        component_build_up_inertia_frd_kg_m2: component_inertia,
        direct_mass_published_range: published_status(
            direct_mass.map(|summary| summary.mean),
            range,
        ),
        component_mass_published_range: published_status(
            component_mass.map(|mass| mass.value),
            range,
        ),
        missing_requirements: missing,
        configuration_identified,
        mass_ready,
        cg_ready,
        inertia_ready,
        mass_properties_ready,
        runtime_ready: false,
    })
}

fn configuration_readiness(file: &MassPropertiesFile, missing: &mut Vec<String>) -> bool {
    let campaign = &file.campaign;
    let config = &campaign.operational_configuration;
    let frame = &file.coordinate_frame;
    for (present, blocker) in [
        (campaign.identity.airframe_id.is_some(), "airframe_id"),
        (campaign.measurement_date.is_some(), "measurement_date"),
        (
            !campaign.linked_geometry_campaign_id.is_empty(),
            "linked_geometry_campaign_id",
        ),
        (config.id.is_some(), "operational_configuration_id"),
        (
            config.propulsion_configuration_description.is_some(),
            "propulsion_configuration_description",
        ),
        (
            config.landing_gear_configuration.is_some(),
            "landing_gear_configuration",
        ),
        (
            config.installed_equipment_notes.is_some(),
            "installed_equipment_configuration_notes",
        ),
        (frame.axes_parallel_to_frd, "frd_parallel_coordinate_frame"),
        (
            frame.wing_root_le_center_plane_datum_established,
            "wing_root_le_center_plane_origin_datum",
        ),
        (frame.lateral_datum_established, "lateral_datum"),
        (frame.vertical_datum_established, "vertical_datum"),
        (
            !frame.source_ids.is_empty() && !frame.photograph_ids.is_empty(),
            "coordinate_frame_datum_evidence",
        ),
    ] {
        if !present {
            missing.push(blocker.to_owned());
        }
    }
    missing.is_empty()
}

#[derive(Clone, Copy)]
enum ComponentRequirement {
    Mass,
    Cg,
    Inertia,
}

fn component_path_blockers(
    file: &MassPropertiesFile,
    components: &[ResolvedComponent],
    requirement: ComponentRequirement,
    missing: &mut Vec<String>,
) {
    if !file.raw_observations.component_inventory_complete {
        missing.push("component_inventory_complete".to_owned());
        return;
    }
    if components.is_empty() {
        missing.push("installed_component_inventory_nonempty".to_owned());
        return;
    }
    for component in components {
        if component.mass.is_none() {
            missing.push(format!("component_mass:{}", component.id));
        }
        if matches!(
            requirement,
            ComponentRequirement::Cg | ComponentRequirement::Inertia
        ) && component.position.is_none()
        {
            missing.push(format!("component_cg_position:{}", component.id));
        }
        if matches!(requirement, ComponentRequirement::Inertia)
            && component.intrinsic_inertia.is_none()
        {
            missing.push(format!("component_intrinsic_inertia:{}", component.id));
        }
    }
}

fn scalar_consistency(
    direct: Option<ScalarEstimate>,
    build_up: Option<ScalarEstimate>,
    tolerance: Option<f64>,
    criterion_blocker: &str,
    inconsistency_blocker: &str,
    missing: &mut Vec<String>,
) -> bool {
    let Some((direct, build_up)) = direct.zip(build_up) else {
        return true;
    };
    let Some(tolerance) = tolerance else {
        missing.push(criterion_blocker.to_owned());
        return false;
    };
    if (direct.value - build_up.value).abs()
        <= tolerance + direct.uncertainty.hypot(build_up.uncertainty)
    {
        true
    } else {
        missing.push(inconsistency_blocker.to_owned());
        false
    }
}

fn vector_consistency(
    direct: Option<VectorEstimate>,
    build_up: Option<VectorEstimate>,
    tolerance: Option<f64>,
    criterion_blocker: &str,
    inconsistency_blocker: &str,
    missing: &mut Vec<String>,
) -> bool {
    let Some((direct, build_up)) = direct.zip(build_up) else {
        return true;
    };
    let Some(tolerance) = tolerance else {
        missing.push(criterion_blocker.to_owned());
        return false;
    };
    let difference = norm(subtract(direct.value, build_up.value));
    let uncertainty = direct
        .uncertainty
        .into_iter()
        .chain(build_up.uncertainty)
        .map(|value| value.powi(2))
        .sum::<f64>()
        .sqrt();
    if difference <= tolerance + uncertainty {
        true
    } else {
        missing.push(inconsistency_blocker.to_owned());
        false
    }
}

fn tensor_consistency(
    direct: Option<InertiaEstimate>,
    build_up: Option<InertiaEstimate>,
    tolerance: Option<f64>,
    missing: &mut Vec<String>,
) -> bool {
    let Some((direct, build_up)) = direct.zip(build_up) else {
        return true;
    };
    let Some(tolerance) = tolerance else {
        missing.push("maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2".to_owned());
        return false;
    };
    let difference =
        matrix_frobenius_difference(&direct.matrix_frd_kg_m2, &build_up.matrix_frd_kg_m2);
    let uncertainty = direct
        .uncertainty_kg_m2
        .into_iter()
        .flatten()
        .chain(build_up.uncertainty_kg_m2.into_iter().flatten())
        .map(|value| value.powi(2))
        .sum::<f64>()
        .sqrt();
    if difference <= tolerance + uncertainty {
        true
    } else {
        missing.push("direct_vs_build_up_inertia_consistency".to_owned());
        false
    }
}

fn published_status(
    value: Option<f64>,
    range: &PublishedWeightRangeFile,
) -> PublishedWeightRangeStatus {
    match value {
        Some(value) if value >= range.minimum_kg && value <= range.maximum_kg => {
            PublishedWeightRangeStatus::WithinPublishedRange
        }
        Some(_) => PublishedWeightRangeStatus::OutsidePublishedRange,
        None => PublishedWeightRangeStatus::Unknown,
    }
}

fn is_symmetric(matrix: &[[f64; 3]; 3]) -> bool {
    let scale = matrix
        .iter()
        .flatten()
        .map(|value| value.abs())
        .fold(1.0, f64::max);
    [(0, 1), (0, 2), (1, 2)]
        .into_iter()
        .all(|(a, b)| (matrix[a][b] - matrix[b][a]).abs() <= SYMMETRY_RELATIVE_TOLERANCE * scale)
}

fn is_positive_definite(matrix: &[[f64; 3]; 3]) -> bool {
    if !matrix.iter().flatten().all(|value| value.is_finite()) || !is_symmetric(matrix) {
        return false;
    }
    let a = matrix[0][0];
    if a <= 0.0 {
        return false;
    }
    let l00 = a.sqrt();
    let l10 = matrix[1][0] / l00;
    let l20 = matrix[2][0] / l00;
    let d11 = matrix[1][1] - l10 * l10;
    if d11 <= 0.0 {
        return false;
    }
    let l11 = d11.sqrt();
    let l21 = (matrix[2][1] - l20 * l10) / l11;
    matrix[2][2] - l20 * l20 - l21 * l21 > 0.0
}

fn matrix_frobenius_difference(a: &[[f64; 3]; 3], b: &[[f64; 3]; 3]) -> f64 {
    let mut sum = 0.0;
    for row in 0..3 {
        for column in 0..3 {
            sum += (a[row][column] - b[row][column]).powi(2);
        }
    }
    sum.sqrt()
}

fn subtract(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(value: [f64; 3]) -> f64 {
    dot(value, value).sqrt()
}

fn deduplicate(values: &mut Vec<String>) {
    let mut seen = HashSet::new();
    values.retain(|value| seen.insert(value.clone()));
}
