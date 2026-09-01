//! Strict loading and deterministic evaluation of off-runtime physical survey evidence.
//!
//! A survey is documentary input. It never mutates an [`crate::AircraftModel`], participates in a
//! physics fingerprint, or runs in the simulation stepping path.

use std::{
    collections::HashSet,
    fs,
    path::{Path, PathBuf},
};

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::REFERENCE_SURVEY_SCHEMA_V0;

const SURVEY_ARTIFACT_KIND: &str = "physical_measurement_evidence_not_runtime_configuration";

#[derive(Debug, Error)]
pub enum ReferenceSurveyError {
    #[error("failed to read reference survey {path}: {source}")]
    Io {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("reference survey JSON has invalid structure: {source}")]
    InvalidStructure {
        #[source]
        source: serde_json::Error,
    },
    #[error("unsupported reference survey schema {found:?}")]
    UnsupportedSchema { found: String },
    #[error("invalid reference survey artifact kind {found:?}")]
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
    #[error("{field} contains duplicate reference {reference_id:?}")]
    DuplicateEvidenceReference { field: String, reference_id: String },
    #[error("invalid measurement {field}: {reason}")]
    InvalidMeasurement { field: String, reason: &'static str },
    #[error("invalid survey metadata {field}: {reason}")]
    InvalidMetadata {
        field: &'static str,
        reason: &'static str,
    },
    #[error("physically impossible derived geometry: {field}")]
    PhysicallyImpossibleDerivedGeometry { field: &'static str },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SurveyClassification {
    PhysicalReferenceMeasurement,
    SyntheticNonReference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CrossVariantStatus {
    ConfirmedIdentical,
    ConsistentButNotProven,
    Different,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct MeasurementSummary {
    mean: f64,
    minimum: f64,
    maximum: f64,
    range: f64,
    effective_uncertainty: f64,
}

impl MeasurementSummary {
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

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct BilateralMeasurementSummary {
    left: MeasurementSummary,
    right: MeasurementSummary,
    combined_mean: f64,
    asymmetry_right_minus_left: f64,
    effective_uncertainty: f64,
}

impl BilateralMeasurementSummary {
    pub const fn left(&self) -> &MeasurementSummary {
        &self.left
    }

    pub const fn right(&self) -> &MeasurementSummary {
        &self.right
    }

    pub const fn combined_mean(&self) -> f64 {
        self.combined_mean
    }

    pub const fn asymmetry_right_minus_left(&self) -> f64 {
        self.asymmetry_right_minus_left
    }

    pub const fn effective_uncertainty(&self) -> f64 {
        self.effective_uncertainty
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct DerivedSurveyValue {
    value: f64,
    uncertainty: f64,
}

impl DerivedSurveyValue {
    pub const fn value(&self) -> f64 {
        self.value
    }

    pub const fn uncertainty(&self) -> f64 {
        self.uncertainty
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CrossVariantComparison {
    quantity: &'static str,
    status: CrossVariantStatus,
    absolute_difference: Option<f64>,
}

impl CrossVariantComparison {
    pub const fn quantity(&self) -> &'static str {
        self.quantity
    }

    pub const fn status(&self) -> CrossVariantStatus {
        self.status
    }

    pub const fn absolute_difference(&self) -> Option<f64> {
        self.absolute_difference
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SurveyEvaluation {
    horizontal_tail_root_le_station: Option<DerivedSurveyValue>,
    vertical_tail_root_le_station: Option<DerivedSurveyValue>,
    wing_quarter_chord_station: Option<DerivedSurveyValue>,
    horizontal_tail_quarter_chord_station: Option<DerivedSurveyValue>,
    vertical_tail_quarter_chord_station: Option<DerivedSurveyValue>,
    horizontal_tail_quarter_chord_arm: Option<DerivedSurveyValue>,
    vertical_tail_quarter_chord_arm: Option<DerivedSurveyValue>,
    horizontal_tail_planform_quarter_chord_offset: Option<DerivedSurveyValue>,
    vertical_tail_planform_quarter_chord_offset: Option<DerivedSurveyValue>,
    horizontal_tail_station_bilateral: Option<BilateralMeasurementSummary>,
    cross_variant_comparisons: Vec<CrossVariantComparison>,
    missing_geometry_observations: Vec<&'static str>,
    missing_campaign_observations: Vec<&'static str>,
    geometry_ready: bool,
    campaign_complete: bool,
    runtime_ready: bool,
}

impl SurveyEvaluation {
    pub const fn horizontal_tail_root_le_station(&self) -> Option<DerivedSurveyValue> {
        self.horizontal_tail_root_le_station
    }

    pub const fn vertical_tail_root_le_station(&self) -> Option<DerivedSurveyValue> {
        self.vertical_tail_root_le_station
    }

    pub const fn wing_quarter_chord_station(&self) -> Option<DerivedSurveyValue> {
        self.wing_quarter_chord_station
    }

    pub const fn horizontal_tail_quarter_chord_station(&self) -> Option<DerivedSurveyValue> {
        self.horizontal_tail_quarter_chord_station
    }

    pub const fn vertical_tail_quarter_chord_station(&self) -> Option<DerivedSurveyValue> {
        self.vertical_tail_quarter_chord_station
    }

    pub const fn horizontal_tail_quarter_chord_arm(&self) -> Option<DerivedSurveyValue> {
        self.horizontal_tail_quarter_chord_arm
    }

    pub const fn vertical_tail_quarter_chord_arm(&self) -> Option<DerivedSurveyValue> {
        self.vertical_tail_quarter_chord_arm
    }

    pub const fn horizontal_tail_planform_quarter_chord_offset(
        &self,
    ) -> Option<DerivedSurveyValue> {
        self.horizontal_tail_planform_quarter_chord_offset
    }

    pub const fn vertical_tail_planform_quarter_chord_offset(&self) -> Option<DerivedSurveyValue> {
        self.vertical_tail_planform_quarter_chord_offset
    }

    pub const fn horizontal_tail_station_bilateral(&self) -> Option<&BilateralMeasurementSummary> {
        self.horizontal_tail_station_bilateral.as_ref()
    }

    pub fn cross_variant_comparisons(&self) -> &[CrossVariantComparison] {
        &self.cross_variant_comparisons
    }

    pub fn missing_geometry_observations(&self) -> &[&'static str] {
        &self.missing_geometry_observations
    }

    pub fn missing_campaign_observations(&self) -> &[&'static str] {
        &self.missing_campaign_observations
    }

    pub const fn geometry_ready(&self) -> bool {
        self.geometry_ready
    }

    pub const fn campaign_complete(&self) -> bool {
        self.campaign_complete
    }

    /// M2.2C is evidence ingestion only and can never authorize a runtime model.
    pub const fn runtime_ready(&self) -> bool {
        self.runtime_ready
    }
}

#[derive(Debug, Clone)]
pub struct PhysicalSurvey {
    file: SurveyFile,
    evaluation: SurveyEvaluation,
}

impl PhysicalSurvey {
    pub const fn classification(&self) -> SurveyClassification {
        self.file.campaign.classification
    }

    pub fn campaign_id(&self) -> &str {
        &self.file.campaign.id
    }

    pub fn airframe_id(&self) -> Option<&str> {
        self.file.campaign.identity.airframe_id.as_deref()
    }

    pub const fn evaluation(&self) -> &SurveyEvaluation {
        &self.evaluation
    }
}

pub struct PhysicalSurveyLoader;

impl PhysicalSurveyLoader {
    pub fn from_json_str(json: &str) -> Result<PhysicalSurvey, ReferenceSurveyError> {
        let file: SurveyFile = serde_json::from_str(json)
            .map_err(|source| ReferenceSurveyError::InvalidStructure { source })?;
        validate_survey(&file)?;
        let evaluation = evaluate_survey(&file)?;
        Ok(PhysicalSurvey { file, evaluation })
    }
}

pub fn load_reference_survey(
    path: impl AsRef<Path>,
) -> Result<PhysicalSurvey, ReferenceSurveyError> {
    let path = path.as_ref();
    let json = fs::read_to_string(path).map_err(|source| ReferenceSurveyError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    PhysicalSurveyLoader::from_json_str(&json)
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SurveyFile {
    schema: String,
    artifact_kind: String,
    campaign: CampaignFile,
    datum: DatumFile,
    provenance_sources: Vec<SurveySourceFile>,
    photographs: Vec<PhotographFile>,
    acceptance_criteria: AcceptanceCriteriaFile,
    comparison_baseline: ComparisonBaselineFile,
    raw_observations: RawObservationsFile,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct CampaignFile {
    id: String,
    classification: SurveyClassification,
    identity: SurveyIdentityFile,
    measurement_date: Option<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SurveyIdentityFile {
    manufacturer: String,
    family: String,
    variant: String,
    airframe_id: Option<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct DatumFile {
    wing_root_le_established: bool,
    definition: String,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(rename_all = "snake_case")]
enum SurveySourceKind {
    MeasurementSession,
    InstrumentCalibration,
    ManufacturerDocumentation,
    DerivedReference,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct SurveySourceFile {
    id: String,
    kind: SurveySourceKind,
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
    maximum_station_asymmetry_m: Option<f64>,
    maximum_direct_vs_planform_qc_difference_m: Option<f64>,
    cross_variant_identity_tolerance_m: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ComparisonBaselineFile {
    source_ids: Vec<String>,
    wing_quarter_chord_offset_m: Option<ReferenceValueFile>,
    horizontal_tail: BaselineHorizontalTailFile,
    vertical_tail: BaselineVerticalTailFile,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct BaselineHorizontalTailFile {
    span_m: Option<ReferenceValueFile>,
    root_chord_m: Option<ReferenceValueFile>,
    tip_chord_m: Option<ReferenceValueFile>,
    area_weighted_quarter_chord_aft_root_le_m: Option<ReferenceValueFile>,
    tip_le_offset_aft_root_le_m: Option<ReferenceValueFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct BaselineVerticalTailFile {
    height_m: Option<ReferenceValueFile>,
    root_chord_m: Option<ReferenceValueFile>,
    tip_chord_m: Option<ReferenceValueFile>,
    area_weighted_quarter_chord_aft_root_le_m: Option<ReferenceValueFile>,
    tip_le_offset_aft_root_le_m: Option<ReferenceValueFile>,
}

#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReferenceValueFile {
    value: f64,
    uncertainty: f64,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawObservationsFile {
    horizontal_tail_root_le_aft_wing_le_m: BilateralObservationFile,
    vertical_tail_root_le_aft_wing_le_m: Option<MeasurementSeriesFile>,
    wing_quarter_chord_aft_wing_le_m: Option<MeasurementSeriesFile>,
    direct_horizontal_tail_quarter_chord_aft_wing_le_m: Option<MeasurementSeriesFile>,
    direct_vertical_tail_quarter_chord_aft_wing_le_m: Option<MeasurementSeriesFile>,
    horizontal_tail_planform: HorizontalPlanformFile,
    vertical_tail_planform: VerticalPlanformFile,
    wing_incidence_rad: Option<MeasurementSeriesFile>,
    stabilizer_incidence_rad: Option<MeasurementSeriesFile>,
    motor_thrust_axis_top_view_rad: Option<MeasurementSeriesFile>,
    motor_thrust_axis_side_view_rad: Option<MeasurementSeriesFile>,
    operational_cg_aft_wing_le_m: Option<MeasurementSeriesFile>,
    battery: Option<BatteryFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct BilateralObservationFile {
    left: Option<MeasurementSeriesFile>,
    right: Option<MeasurementSeriesFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct MeasurementSeriesFile {
    readings: [f64; 3],
    instrument_resolution: f64,
    stated_uncertainty: f64,
    datum_definition: String,
    notes: Option<String>,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct HorizontalPlanformFile {
    span_m: Option<MeasurementSeriesFile>,
    root_chord_m: Option<MeasurementSeriesFile>,
    tip_chord_m: Option<MeasurementSeriesFile>,
    tip_le_offset_aft_root_le_m: Option<MeasurementSeriesFile>,
    intermediate_stations: Vec<PlanformStationFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct VerticalPlanformFile {
    height_m: Option<MeasurementSeriesFile>,
    root_chord_m: Option<MeasurementSeriesFile>,
    tip_chord_m: Option<MeasurementSeriesFile>,
    tip_le_offset_aft_root_le_m: Option<MeasurementSeriesFile>,
    intermediate_stations: Vec<PlanformStationFile>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct PlanformStationFile {
    id: String,
    span_or_height_station_m: MeasurementSeriesFile,
    leading_edge_offset_aft_root_le_m: MeasurementSeriesFile,
    chord_m: MeasurementSeriesFile,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct BatteryFile {
    configuration_id: String,
    manufacturer: Option<String>,
    model: Option<String>,
    cell_count: Option<u16>,
    nominal_capacity_ah: Option<f64>,
    location_description: String,
    longitudinal_station_aft_wing_le_m: Option<MeasurementSeriesFile>,
    source_ids: Vec<String>,
    photograph_ids: Vec<String>,
    notes: Option<String>,
}

#[derive(Debug, Clone, Copy)]
enum MeasurementDomain {
    PositiveLength,
    NonNegativeLength,
    SignedLength,
    Angle,
}

fn validate_survey(file: &SurveyFile) -> Result<(), ReferenceSurveyError> {
    if file.schema != REFERENCE_SURVEY_SCHEMA_V0 {
        return Err(ReferenceSurveyError::UnsupportedSchema {
            found: file.schema.clone(),
        });
    }
    if file.artifact_kind != SURVEY_ARTIFACT_KIND {
        return Err(ReferenceSurveyError::InvalidArtifactKind {
            found: file.artifact_kind.clone(),
        });
    }
    validate_stable_id("campaign", &file.campaign.id)?;
    validate_required_text(
        "campaign.identity.manufacturer",
        &file.campaign.identity.manufacturer,
    )?;
    validate_required_text("campaign.identity.family", &file.campaign.identity.family)?;
    validate_required_text("campaign.identity.variant", &file.campaign.identity.variant)?;
    validate_optional_text("campaign.notes", file.campaign.notes.as_deref())?;
    if let Some(id) = file.campaign.identity.airframe_id.as_deref() {
        validate_stable_id("airframe", id)?;
    }
    if let Some(date) = file.campaign.measurement_date.as_deref()
        && !is_iso_date(date)
    {
        return Err(ReferenceSurveyError::InvalidMetadata {
            field: "campaign.measurement_date",
            reason: "expected YYYY-MM-DD",
        });
    }
    validate_required_text("datum.definition", &file.datum.definition)?;

    let source_ids = validate_sources(&file.provenance_sources)?;
    let photograph_ids = validate_photographs(&file.photographs)?;
    validate_refs(
        "datum",
        &file.datum.source_ids,
        &file.datum.photograph_ids,
        &source_ids,
        &photograph_ids,
    )?;
    validate_source_refs(
        "comparison_baseline",
        &file.comparison_baseline.source_ids,
        &source_ids,
    )?;
    validate_acceptance_criteria(&file.acceptance_criteria)?;
    validate_baseline(&file.comparison_baseline)?;
    validate_observations(&file.raw_observations, &source_ids, &photograph_ids)?;
    Ok(())
}

fn validate_sources(sources: &[SurveySourceFile]) -> Result<HashSet<String>, ReferenceSurveyError> {
    let mut ids = HashSet::new();
    for source in sources {
        validate_stable_id("provenance source", &source.id)?;
        if !ids.insert(source.id.clone()) {
            return Err(ReferenceSurveyError::DuplicateStableId {
                kind: "provenance source",
                value: source.id.clone(),
            });
        }
        validate_required_text("provenance_sources.title", &source.title)?;
        validate_optional_text("provenance_sources.url", source.url.as_deref())?;
        validate_optional_text("provenance_sources.notes", source.notes.as_deref())?;
        validate_optional_sha256("provenance_sources.sha256", source.sha256.as_deref())?;
        let _ = source.kind;
    }
    Ok(ids)
}

fn validate_photographs(
    photographs: &[PhotographFile],
) -> Result<HashSet<String>, ReferenceSurveyError> {
    let mut ids = HashSet::new();
    for photograph in photographs {
        validate_stable_id("photograph", &photograph.id)?;
        if !ids.insert(photograph.id.clone()) {
            return Err(ReferenceSurveyError::DuplicateStableId {
                kind: "photograph",
                value: photograph.id.clone(),
            });
        }
        if photograph.path.as_deref().is_none_or(str::is_empty)
            && photograph.url.as_deref().is_none_or(str::is_empty)
        {
            return Err(ReferenceSurveyError::InvalidMetadata {
                field: "photographs",
                reason: "each photograph needs a nonempty path or URL",
            });
        }
        validate_required_text("photographs.description", &photograph.description)?;
        validate_optional_sha256("photographs.sha256", photograph.sha256.as_deref())?;
    }
    Ok(ids)
}

fn validate_acceptance_criteria(
    criteria: &AcceptanceCriteriaFile,
) -> Result<(), ReferenceSurveyError> {
    for (field, value) in [
        (
            "acceptance_criteria.maximum_station_asymmetry_m",
            criteria.maximum_station_asymmetry_m,
        ),
        (
            "acceptance_criteria.maximum_direct_vs_planform_qc_difference_m",
            criteria.maximum_direct_vs_planform_qc_difference_m,
        ),
        (
            "acceptance_criteria.cross_variant_identity_tolerance_m",
            criteria.cross_variant_identity_tolerance_m,
        ),
    ] {
        if let Some(value) = value
            && (!value.is_finite() || value < 0.0)
        {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "tolerance must be finite and nonnegative",
            });
        }
    }
    Ok(())
}

fn validate_baseline(baseline: &ComparisonBaselineFile) -> Result<(), ReferenceSurveyError> {
    let positive_values = [
        (
            "comparison_baseline.wing_quarter_chord_offset_m",
            baseline.wing_quarter_chord_offset_m,
        ),
        (
            "comparison_baseline.horizontal_tail.span_m",
            baseline.horizontal_tail.span_m,
        ),
        (
            "comparison_baseline.horizontal_tail.root_chord_m",
            baseline.horizontal_tail.root_chord_m,
        ),
        (
            "comparison_baseline.horizontal_tail.tip_chord_m",
            baseline.horizontal_tail.tip_chord_m,
        ),
        (
            "comparison_baseline.vertical_tail.height_m",
            baseline.vertical_tail.height_m,
        ),
        (
            "comparison_baseline.vertical_tail.root_chord_m",
            baseline.vertical_tail.root_chord_m,
        ),
        (
            "comparison_baseline.vertical_tail.tip_chord_m",
            baseline.vertical_tail.tip_chord_m,
        ),
    ];
    for (field, value) in positive_values {
        if let Some(value) = value
            && (!value.value.is_finite()
                || value.value <= 0.0
                || !value.uncertainty.is_finite()
                || value.uncertainty < 0.0)
        {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "baseline value must be positive and uncertainty nonnegative",
            });
        }
    }
    for (field, value) in [
        (
            "comparison_baseline.horizontal_tail.area_weighted_quarter_chord_aft_root_le_m",
            baseline
                .horizontal_tail
                .area_weighted_quarter_chord_aft_root_le_m,
        ),
        (
            "comparison_baseline.horizontal_tail.tip_le_offset_aft_root_le_m",
            baseline.horizontal_tail.tip_le_offset_aft_root_le_m,
        ),
        (
            "comparison_baseline.vertical_tail.area_weighted_quarter_chord_aft_root_le_m",
            baseline
                .vertical_tail
                .area_weighted_quarter_chord_aft_root_le_m,
        ),
        (
            "comparison_baseline.vertical_tail.tip_le_offset_aft_root_le_m",
            baseline.vertical_tail.tip_le_offset_aft_root_le_m,
        ),
    ] {
        if let Some(value) = value
            && (!value.value.is_finite()
                || !value.uncertainty.is_finite()
                || value.uncertainty < 0.0)
        {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "baseline offset must be finite and uncertainty nonnegative",
            });
        }
    }
    Ok(())
}

fn validate_observations(
    observations: &RawObservationsFile,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
    validate_optional_series(
        "raw_observations.horizontal_tail_root_le_aft_wing_le_m.left",
        observations
            .horizontal_tail_root_le_aft_wing_le_m
            .left
            .as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.horizontal_tail_root_le_aft_wing_le_m.right",
        observations
            .horizontal_tail_root_le_aft_wing_le_m
            .right
            .as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    for (field, series, domain) in [
        (
            "raw_observations.vertical_tail_root_le_aft_wing_le_m",
            observations.vertical_tail_root_le_aft_wing_le_m.as_ref(),
            MeasurementDomain::PositiveLength,
        ),
        (
            "raw_observations.wing_quarter_chord_aft_wing_le_m",
            observations.wing_quarter_chord_aft_wing_le_m.as_ref(),
            MeasurementDomain::PositiveLength,
        ),
        (
            "raw_observations.direct_horizontal_tail_quarter_chord_aft_wing_le_m",
            observations
                .direct_horizontal_tail_quarter_chord_aft_wing_le_m
                .as_ref(),
            MeasurementDomain::PositiveLength,
        ),
        (
            "raw_observations.direct_vertical_tail_quarter_chord_aft_wing_le_m",
            observations
                .direct_vertical_tail_quarter_chord_aft_wing_le_m
                .as_ref(),
            MeasurementDomain::PositiveLength,
        ),
        (
            "raw_observations.wing_incidence_rad",
            observations.wing_incidence_rad.as_ref(),
            MeasurementDomain::Angle,
        ),
        (
            "raw_observations.stabilizer_incidence_rad",
            observations.stabilizer_incidence_rad.as_ref(),
            MeasurementDomain::Angle,
        ),
        (
            "raw_observations.motor_thrust_axis_top_view_rad",
            observations.motor_thrust_axis_top_view_rad.as_ref(),
            MeasurementDomain::Angle,
        ),
        (
            "raw_observations.motor_thrust_axis_side_view_rad",
            observations.motor_thrust_axis_side_view_rad.as_ref(),
            MeasurementDomain::Angle,
        ),
        (
            "raw_observations.operational_cg_aft_wing_le_m",
            observations.operational_cg_aft_wing_le_m.as_ref(),
            MeasurementDomain::NonNegativeLength,
        ),
    ] {
        validate_optional_series(field, series, domain, source_ids, photograph_ids)?;
    }
    validate_horizontal_planform(
        &observations.horizontal_tail_planform,
        source_ids,
        photograph_ids,
    )?;
    validate_vertical_planform(
        &observations.vertical_tail_planform,
        source_ids,
        photograph_ids,
    )?;
    if let Some(battery) = observations.battery.as_ref() {
        validate_stable_id("battery configuration", &battery.configuration_id)?;
        validate_required_text(
            "raw_observations.battery.location_description",
            &battery.location_description,
        )?;
        validate_optional_text(
            "raw_observations.battery.manufacturer",
            battery.manufacturer.as_deref(),
        )?;
        validate_optional_text("raw_observations.battery.model", battery.model.as_deref())?;
        validate_optional_text("raw_observations.battery.notes", battery.notes.as_deref())?;
        if battery.cell_count == Some(0) {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: "raw_observations.battery.cell_count".to_owned(),
                reason: "cell count must be positive",
            });
        }
        if let Some(capacity) = battery.nominal_capacity_ah
            && (!capacity.is_finite() || capacity <= 0.0)
        {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: "raw_observations.battery.nominal_capacity_ah".to_owned(),
                reason: "capacity must be finite and positive",
            });
        }
        validate_refs(
            "raw_observations.battery",
            &battery.source_ids,
            &battery.photograph_ids,
            source_ids,
            photograph_ids,
        )?;
        validate_optional_series(
            "raw_observations.battery.longitudinal_station_aft_wing_le_m",
            battery.longitudinal_station_aft_wing_le_m.as_ref(),
            MeasurementDomain::NonNegativeLength,
            source_ids,
            photograph_ids,
        )?;
    }
    Ok(())
}

fn validate_horizontal_planform(
    planform: &HorizontalPlanformFile,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
    validate_optional_series(
        "raw_observations.horizontal_tail_planform.span_m",
        planform.span_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.horizontal_tail_planform.root_chord_m",
        planform.root_chord_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.horizontal_tail_planform.tip_chord_m",
        planform.tip_chord_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.horizontal_tail_planform.tip_le_offset_aft_root_le_m",
        planform.tip_le_offset_aft_root_le_m.as_ref(),
        MeasurementDomain::SignedLength,
        source_ids,
        photograph_ids,
    )?;
    validate_planform_stations(
        "raw_observations.horizontal_tail_planform",
        &planform.intermediate_stations,
        planform
            .span_m
            .as_ref()
            .map(|series| summarize(series).mean / 2.0),
        source_ids,
        photograph_ids,
    )
}

fn validate_vertical_planform(
    planform: &VerticalPlanformFile,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
    validate_optional_series(
        "raw_observations.vertical_tail_planform.height_m",
        planform.height_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.vertical_tail_planform.root_chord_m",
        planform.root_chord_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.vertical_tail_planform.tip_chord_m",
        planform.tip_chord_m.as_ref(),
        MeasurementDomain::PositiveLength,
        source_ids,
        photograph_ids,
    )?;
    validate_optional_series(
        "raw_observations.vertical_tail_planform.tip_le_offset_aft_root_le_m",
        planform.tip_le_offset_aft_root_le_m.as_ref(),
        MeasurementDomain::SignedLength,
        source_ids,
        photograph_ids,
    )?;
    validate_planform_stations(
        "raw_observations.vertical_tail_planform",
        &planform.intermediate_stations,
        planform
            .height_m
            .as_ref()
            .map(|series| summarize(series).mean),
        source_ids,
        photograph_ids,
    )
}

fn validate_planform_stations(
    field: &str,
    stations: &[PlanformStationFile],
    extent: Option<f64>,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
    let mut ids = HashSet::new();
    let mut prior = 0.0;
    for station in stations {
        validate_stable_id("planform station", &station.id)?;
        if !ids.insert(station.id.clone()) {
            return Err(ReferenceSurveyError::DuplicateStableId {
                kind: "planform station",
                value: station.id.clone(),
            });
        }
        let prefix = format!("{field}.intermediate_stations.{}", station.id);
        validate_series(
            &format!("{prefix}.span_or_height_station_m"),
            &station.span_or_height_station_m,
            MeasurementDomain::PositiveLength,
            source_ids,
            photograph_ids,
        )?;
        validate_series(
            &format!("{prefix}.leading_edge_offset_aft_root_le_m"),
            &station.leading_edge_offset_aft_root_le_m,
            MeasurementDomain::SignedLength,
            source_ids,
            photograph_ids,
        )?;
        validate_series(
            &format!("{prefix}.chord_m"),
            &station.chord_m,
            MeasurementDomain::PositiveLength,
            source_ids,
            photograph_ids,
        )?;
        let position = summarize(&station.span_or_height_station_m).mean;
        if position <= prior || extent.is_some_and(|extent| position >= extent) {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: format!("{prefix}.span_or_height_station_m"),
                reason: "intermediate stations must be strictly ordered inside the measured extent",
            });
        }
        prior = position;
    }
    Ok(())
}

fn validate_optional_series(
    field: &str,
    series: Option<&MeasurementSeriesFile>,
    domain: MeasurementDomain,
    source_ids: &HashSet<String>,
    photograph_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
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
) -> Result<(), ReferenceSurveyError> {
    if !series.instrument_resolution.is_finite() || series.instrument_resolution <= 0.0 {
        return Err(ReferenceSurveyError::InvalidMeasurement {
            field: field.to_owned(),
            reason: "instrument resolution must be finite and positive",
        });
    }
    if !series.stated_uncertainty.is_finite() || series.stated_uncertainty < 0.0 {
        return Err(ReferenceSurveyError::InvalidMeasurement {
            field: field.to_owned(),
            reason: "stated uncertainty must be finite and nonnegative",
        });
    }
    for value in series.readings {
        let valid = match domain {
            MeasurementDomain::PositiveLength => value.is_finite() && value > 0.0,
            MeasurementDomain::NonNegativeLength => value.is_finite() && value >= 0.0,
            MeasurementDomain::SignedLength => value.is_finite(),
            MeasurementDomain::Angle => {
                value.is_finite() && value.abs() <= std::f64::consts::FRAC_PI_2
            }
        };
        if !valid {
            return Err(ReferenceSurveyError::InvalidMeasurement {
                field: field.to_owned(),
                reason: "reading is non-finite or outside its physical domain",
            });
        }
    }
    validate_required_text("measurement.datum_definition", &series.datum_definition)?;
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
) -> Result<(), ReferenceSurveyError> {
    validate_source_refs(field, source_refs, source_ids)?;
    validate_reference_list(
        field,
        photograph_refs,
        |id| photograph_ids.contains(id),
        |id| ReferenceSurveyError::UnresolvedPhotographReference {
            field: field.to_owned(),
            photograph_id: id.to_owned(),
        },
    )
}

fn validate_source_refs(
    field: &str,
    refs: &[String],
    source_ids: &HashSet<String>,
) -> Result<(), ReferenceSurveyError> {
    validate_reference_list(
        field,
        refs,
        |id| source_ids.contains(id),
        |id| ReferenceSurveyError::UnresolvedSourceReference {
            field: field.to_owned(),
            source_id: id.to_owned(),
        },
    )
}

fn validate_reference_list<F, E>(
    field: &str,
    refs: &[String],
    exists: F,
    unresolved: E,
) -> Result<(), ReferenceSurveyError>
where
    F: Fn(&str) -> bool,
    E: Fn(&str) -> ReferenceSurveyError,
{
    let mut seen = HashSet::new();
    for reference in refs {
        if !is_valid_stable_id(reference) {
            return Err(ReferenceSurveyError::InvalidStableId {
                kind: "evidence reference",
                value: reference.clone(),
            });
        }
        if !exists(reference) {
            return Err(unresolved(reference));
        }
        if !seen.insert(reference) {
            return Err(ReferenceSurveyError::DuplicateEvidenceReference {
                field: field.to_owned(),
                reference_id: reference.clone(),
            });
        }
    }
    Ok(())
}

fn validate_stable_id(kind: &'static str, value: &str) -> Result<(), ReferenceSurveyError> {
    if is_valid_stable_id(value) {
        Ok(())
    } else {
        Err(ReferenceSurveyError::InvalidStableId {
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

fn validate_required_text(field: &'static str, value: &str) -> Result<(), ReferenceSurveyError> {
    if value.trim().is_empty() {
        Err(ReferenceSurveyError::InvalidMetadata {
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
) -> Result<(), ReferenceSurveyError> {
    if value.is_some_and(|value| value.trim().is_empty()) {
        Err(ReferenceSurveyError::InvalidMetadata {
            field,
            reason: "present text must not be empty or whitespace",
        })
    } else {
        Ok(())
    }
}

fn validate_optional_sha256(
    field: &'static str,
    value: Option<&str>,
) -> Result<(), ReferenceSurveyError> {
    if value.is_some_and(|value| {
        value.len() != 64 || !value.bytes().all(|byte| byte.is_ascii_hexdigit())
    }) {
        Err(ReferenceSurveyError::InvalidMetadata {
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
    let leap_year = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    let days_in_month = match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap_year => 29,
        2 => 28,
        _ => return false,
    };
    year != 0 && day != 0 && day <= days_in_month
}

fn summarize(series: &MeasurementSeriesFile) -> MeasurementSummary {
    let [a, b, c] = series.readings;
    let minimum = a.min(b).min(c);
    let maximum = a.max(b).max(c);
    let range = maximum - minimum;
    MeasurementSummary {
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

fn summarize_optional(series: Option<&MeasurementSeriesFile>) -> Option<DerivedSurveyValue> {
    series.map(|series| {
        let summary = summarize(series);
        DerivedSurveyValue {
            value: summary.mean,
            uncertainty: summary.effective_uncertainty,
        }
    })
}

fn summarize_bilateral(
    observation: &BilateralObservationFile,
) -> Option<BilateralMeasurementSummary> {
    let left = summarize(observation.left.as_ref()?);
    let right = summarize(observation.right.as_ref()?);
    let asymmetry = right.mean - left.mean;
    Some(BilateralMeasurementSummary {
        combined_mean: (left.mean + right.mean) / 2.0,
        effective_uncertainty: ((left.effective_uncertainty.powi(2)
            + right.effective_uncertainty.powi(2))
        .sqrt()
            / 2.0)
            .max(asymmetry.abs() / 2.0),
        asymmetry_right_minus_left: asymmetry,
        left,
        right,
    })
}

#[derive(Clone, Copy)]
struct PlanformPoint {
    station: f64,
    leading_edge: f64,
    chord: f64,
    station_uncertainty: f64,
    leading_edge_uncertainty: f64,
    chord_uncertainty: f64,
}

fn horizontal_planform_offset(planform: &HorizontalPlanformFile) -> Option<DerivedSurveyValue> {
    planform_offset(
        planform.span_m.as_ref(),
        0.5,
        planform.root_chord_m.as_ref(),
        planform.tip_chord_m.as_ref(),
        planform.tip_le_offset_aft_root_le_m.as_ref(),
        &planform.intermediate_stations,
    )
}

fn vertical_planform_offset(planform: &VerticalPlanformFile) -> Option<DerivedSurveyValue> {
    planform_offset(
        planform.height_m.as_ref(),
        1.0,
        planform.root_chord_m.as_ref(),
        planform.tip_chord_m.as_ref(),
        planform.tip_le_offset_aft_root_le_m.as_ref(),
        &planform.intermediate_stations,
    )
}

fn planform_offset(
    extent: Option<&MeasurementSeriesFile>,
    extent_factor: f64,
    root_chord: Option<&MeasurementSeriesFile>,
    tip_chord: Option<&MeasurementSeriesFile>,
    tip_le_offset: Option<&MeasurementSeriesFile>,
    intermediate_stations: &[PlanformStationFile],
) -> Option<DerivedSurveyValue> {
    let extent = summarize(extent?);
    let root_chord = summarize(root_chord?);
    let tip_chord = summarize(tip_chord?);
    let tip_le_offset = summarize(tip_le_offset?);
    let mut points = Vec::with_capacity(intermediate_stations.len() + 2);
    points.push(PlanformPoint {
        station: 0.0,
        leading_edge: 0.0,
        chord: root_chord.mean,
        station_uncertainty: 0.0,
        leading_edge_uncertainty: 0.0,
        chord_uncertainty: root_chord.effective_uncertainty,
    });
    for station in intermediate_stations {
        let position = summarize(&station.span_or_height_station_m);
        let leading_edge = summarize(&station.leading_edge_offset_aft_root_le_m);
        let chord = summarize(&station.chord_m);
        points.push(PlanformPoint {
            station: position.mean,
            leading_edge: leading_edge.mean,
            chord: chord.mean,
            station_uncertainty: position.effective_uncertainty,
            leading_edge_uncertainty: leading_edge.effective_uncertainty,
            chord_uncertainty: chord.effective_uncertainty,
        });
    }
    points.push(PlanformPoint {
        station: extent.mean * extent_factor,
        leading_edge: tip_le_offset.mean,
        chord: tip_chord.mean,
        station_uncertainty: extent.effective_uncertainty * extent_factor,
        leading_edge_uncertainty: tip_le_offset.effective_uncertainty,
        chord_uncertainty: tip_chord.effective_uncertainty,
    });
    let nominal = integrate_quarter_chord(&points)?;
    let mut squared_uncertainty = 0.0;
    for index in 0..points.len() {
        for component in 0..3 {
            let uncertainty = match component {
                0 => points[index].station_uncertainty,
                1 => points[index].leading_edge_uncertainty,
                _ => points[index].chord_uncertainty,
            };
            if uncertainty == 0.0 {
                continue;
            }
            let mut plus = points.clone();
            let mut minus = points.clone();
            match component {
                0 => {
                    plus[index].station += uncertainty;
                    minus[index].station -= uncertainty;
                }
                1 => {
                    plus[index].leading_edge += uncertainty;
                    minus[index].leading_edge -= uncertainty;
                }
                _ => {
                    plus[index].chord += uncertainty;
                    minus[index].chord -= uncertainty;
                }
            }
            let deviation = [
                integrate_quarter_chord(&plus),
                integrate_quarter_chord(&minus),
            ]
            .into_iter()
            .flatten()
            .map(|value| (value - nominal).abs())
            .fold(0.0, f64::max);
            squared_uncertainty += deviation.powi(2);
        }
    }
    Some(DerivedSurveyValue {
        value: nominal,
        uncertainty: squared_uncertainty.sqrt(),
    })
}

fn integrate_quarter_chord(points: &[PlanformPoint]) -> Option<f64> {
    let mut area = 0.0;
    let mut first_moment = 0.0;
    for pair in points.windows(2) {
        let [a, b] = [pair[0], pair[1]];
        let width = b.station - a.station;
        if width <= 0.0 || a.chord <= 0.0 || b.chord <= 0.0 {
            return None;
        }
        let dx = b.leading_edge - a.leading_edge;
        let dc = b.chord - a.chord;
        let integral_x_times_chord =
            a.leading_edge * a.chord + (a.leading_edge * dc + a.chord * dx) / 2.0 + dx * dc / 3.0;
        let integral_chord_squared = a.chord.powi(2) + a.chord * dc + dc.powi(2) / 3.0;
        area += width * (a.chord + b.chord) / 2.0;
        first_moment += width * (integral_x_times_chord + 0.25 * integral_chord_squared);
    }
    (area > 0.0).then_some(first_moment / area)
}

fn add_values(a: DerivedSurveyValue, b: DerivedSurveyValue) -> DerivedSurveyValue {
    DerivedSurveyValue {
        value: a.value + b.value,
        uncertainty: a.uncertainty.hypot(b.uncertainty),
    }
}

fn subtract_values(a: DerivedSurveyValue, b: DerivedSurveyValue) -> DerivedSurveyValue {
    DerivedSurveyValue {
        value: a.value - b.value,
        uncertainty: a.uncertainty.hypot(b.uncertainty),
    }
}

fn evaluate_survey(file: &SurveyFile) -> Result<SurveyEvaluation, ReferenceSurveyError> {
    let observations = &file.raw_observations;
    let bilateral = summarize_bilateral(&observations.horizontal_tail_root_le_aft_wing_le_m);
    let horizontal_root = bilateral.as_ref().map(|summary| DerivedSurveyValue {
        value: summary.combined_mean,
        uncertainty: summary.effective_uncertainty,
    });
    let vertical_root =
        summarize_optional(observations.vertical_tail_root_le_aft_wing_le_m.as_ref());
    let wing_qc = summarize_optional(observations.wing_quarter_chord_aft_wing_le_m.as_ref());
    let horizontal_planform_offset =
        horizontal_planform_offset(&observations.horizontal_tail_planform);
    let vertical_planform_offset = vertical_planform_offset(&observations.vertical_tail_planform);
    let horizontal_planform_station = horizontal_root
        .zip(horizontal_planform_offset)
        .map(|(root, offset)| add_values(root, offset));
    let vertical_planform_station = vertical_root
        .zip(vertical_planform_offset)
        .map(|(root, offset)| add_values(root, offset));
    let horizontal_direct = summarize_optional(
        observations
            .direct_horizontal_tail_quarter_chord_aft_wing_le_m
            .as_ref(),
    );
    let vertical_direct = summarize_optional(
        observations
            .direct_vertical_tail_quarter_chord_aft_wing_le_m
            .as_ref(),
    );

    let mut missing_geometry = Vec::new();
    require_campaign_identity(file, &mut missing_geometry);
    if !file.datum.wing_root_le_established || !datum_has_evidence(&file.datum) {
        missing_geometry.push("wing_root_le_datum_with_evidence");
    }
    if observations
        .horizontal_tail_root_le_aft_wing_le_m
        .left
        .is_none()
    {
        missing_geometry.push("horizontal_tail_root_le_left_three_readings");
    }
    if observations
        .horizontal_tail_root_le_aft_wing_le_m
        .right
        .is_none()
    {
        missing_geometry.push("horizontal_tail_root_le_right_three_readings");
    }
    match (
        bilateral.as_ref(),
        file.acceptance_criteria.maximum_station_asymmetry_m,
    ) {
        (_, None) => missing_geometry.push("maximum_station_asymmetry_m_acceptance_criterion"),
        (Some(summary), Some(tolerance))
            if summary.asymmetry_right_minus_left.abs() > tolerance =>
        {
            missing_geometry.push("horizontal_tail_station_asymmetry_within_tolerance");
        }
        _ => {}
    }
    require_series_evidence(
        "horizontal_tail_root_le_left_evidence",
        observations
            .horizontal_tail_root_le_aft_wing_le_m
            .left
            .as_ref(),
        &mut missing_geometry,
    );
    require_series_evidence(
        "horizontal_tail_root_le_right_evidence",
        observations
            .horizontal_tail_root_le_aft_wing_le_m
            .right
            .as_ref(),
        &mut missing_geometry,
    );
    require_value_and_evidence(
        "vertical_tail_root_le",
        observations.vertical_tail_root_le_aft_wing_le_m.as_ref(),
        &mut missing_geometry,
    );
    require_value_and_evidence(
        "wing_quarter_chord",
        observations.wing_quarter_chord_aft_wing_le_m.as_ref(),
        &mut missing_geometry,
    );

    let horizontal_qc = select_qc_station(
        "horizontal_tail",
        horizontal_direct,
        horizontal_planform_station,
        observations
            .direct_horizontal_tail_quarter_chord_aft_wing_le_m
            .as_ref(),
        horizontal_planform_has_evidence(&observations.horizontal_tail_planform),
        &file.acceptance_criteria,
        &mut missing_geometry,
    );
    let vertical_qc = select_qc_station(
        "vertical_tail",
        vertical_direct,
        vertical_planform_station,
        observations
            .direct_vertical_tail_quarter_chord_aft_wing_le_m
            .as_ref(),
        vertical_planform_has_evidence(&observations.vertical_tail_planform),
        &file.acceptance_criteria,
        &mut missing_geometry,
    );
    let horizontal_arm = horizontal_qc
        .zip(wing_qc)
        .map(|(tail, wing)| subtract_values(tail, wing));
    let vertical_arm = vertical_qc
        .zip(wing_qc)
        .map(|(tail, wing)| subtract_values(tail, wing));
    for (field, value) in [
        ("horizontal_tail_quarter_chord_arm", horizontal_arm),
        ("vertical_tail_quarter_chord_arm", vertical_arm),
    ] {
        if value.is_some_and(|value| value.value <= 0.0) {
            return Err(ReferenceSurveyError::PhysicallyImpossibleDerivedGeometry { field });
        }
    }
    if horizontal_arm.is_none() {
        missing_geometry.push("horizontal_tail_quarter_chord_arm");
    }
    if vertical_arm.is_none() {
        missing_geometry.push("vertical_tail_quarter_chord_arm");
    }
    deduplicate(&mut missing_geometry);

    let mut missing_campaign = missing_geometry.clone();
    require_horizontal_planform_campaign(
        &observations.horizontal_tail_planform,
        &mut missing_campaign,
    );
    require_vertical_planform_campaign(&observations.vertical_tail_planform, &mut missing_campaign);
    require_value_and_evidence(
        "wing_incidence",
        observations.wing_incidence_rad.as_ref(),
        &mut missing_campaign,
    );
    require_value_and_evidence(
        "stabilizer_incidence",
        observations.stabilizer_incidence_rad.as_ref(),
        &mut missing_campaign,
    );
    require_value_and_evidence(
        "motor_thrust_axis_top_view",
        observations.motor_thrust_axis_top_view_rad.as_ref(),
        &mut missing_campaign,
    );
    require_value_and_evidence(
        "motor_thrust_axis_side_view",
        observations.motor_thrust_axis_side_view_rad.as_ref(),
        &mut missing_campaign,
    );
    require_value_and_evidence(
        "operational_cg_station",
        observations.operational_cg_aft_wing_le_m.as_ref(),
        &mut missing_campaign,
    );
    require_battery_campaign(observations.battery.as_ref(), &mut missing_campaign);
    deduplicate(&mut missing_campaign);

    Ok(SurveyEvaluation {
        horizontal_tail_root_le_station: horizontal_root,
        vertical_tail_root_le_station: vertical_root,
        wing_quarter_chord_station: wing_qc,
        horizontal_tail_quarter_chord_station: horizontal_qc,
        vertical_tail_quarter_chord_station: vertical_qc,
        horizontal_tail_quarter_chord_arm: horizontal_arm,
        vertical_tail_quarter_chord_arm: vertical_arm,
        horizontal_tail_planform_quarter_chord_offset: horizontal_planform_offset,
        vertical_tail_planform_quarter_chord_offset: vertical_planform_offset,
        horizontal_tail_station_bilateral: bilateral,
        cross_variant_comparisons: compare_to_baseline(file),
        geometry_ready: missing_geometry.is_empty(),
        campaign_complete: missing_campaign.is_empty(),
        missing_geometry_observations: missing_geometry,
        missing_campaign_observations: missing_campaign,
        runtime_ready: false,
    })
}

fn select_qc_station(
    prefix: &'static str,
    direct: Option<DerivedSurveyValue>,
    planform: Option<DerivedSurveyValue>,
    direct_series: Option<&MeasurementSeriesFile>,
    planform_evidence: bool,
    criteria: &AcceptanceCriteriaFile,
    missing: &mut Vec<&'static str>,
) -> Option<DerivedSurveyValue> {
    match (direct, planform) {
        (Some(direct), Some(planform)) => {
            let consistent = criteria
                .maximum_direct_vs_planform_qc_difference_m
                .is_some_and(|tolerance| {
                    (direct.value - planform.value).abs()
                        <= tolerance + direct.uncertainty.hypot(planform.uncertainty)
                });
            if !consistent {
                missing.push(match prefix {
                    "horizontal_tail" => "horizontal_tail_direct_vs_planform_qc_consistency",
                    _ => "vertical_tail_direct_vs_planform_qc_consistency",
                });
                None
            } else if direct_series.is_some_and(series_has_evidence) && planform_evidence {
                Some(direct)
            } else {
                missing.push(match prefix {
                    "horizontal_tail" => "horizontal_tail_quarter_chord_evidence",
                    _ => "vertical_tail_quarter_chord_evidence",
                });
                None
            }
        }
        (Some(direct), None) if direct_series.is_some_and(series_has_evidence) => Some(direct),
        (None, Some(planform)) if planform_evidence => Some(planform),
        _ => {
            missing.push(match prefix {
                "horizontal_tail" => "horizontal_tail_direct_qc_or_complete_planform",
                _ => "vertical_tail_direct_qc_or_complete_planform",
            });
            None
        }
    }
}

fn require_campaign_identity(file: &SurveyFile, missing: &mut Vec<&'static str>) {
    if file.campaign.identity.airframe_id.is_none() {
        missing.push("airframe_id");
    }
    if file.campaign.measurement_date.is_none() {
        missing.push("measurement_date");
    }
}

fn datum_has_evidence(datum: &DatumFile) -> bool {
    !datum.source_ids.is_empty() && !datum.photograph_ids.is_empty()
}

fn series_has_evidence(series: &MeasurementSeriesFile) -> bool {
    !series.source_ids.is_empty() && !series.photograph_ids.is_empty()
}

fn require_series_evidence(
    blocker: &'static str,
    series: Option<&MeasurementSeriesFile>,
    missing: &mut Vec<&'static str>,
) {
    if series.is_some_and(|series| !series_has_evidence(series)) {
        missing.push(blocker);
    }
}

fn require_value_and_evidence(
    blocker: &'static str,
    series: Option<&MeasurementSeriesFile>,
    missing: &mut Vec<&'static str>,
) {
    if !series.is_some_and(series_has_evidence) {
        missing.push(blocker);
    }
}

fn horizontal_planform_has_evidence(planform: &HorizontalPlanformFile) -> bool {
    [
        planform.span_m.as_ref(),
        planform.root_chord_m.as_ref(),
        planform.tip_chord_m.as_ref(),
        planform.tip_le_offset_aft_root_le_m.as_ref(),
    ]
    .into_iter()
    .all(|series| series.is_some_and(series_has_evidence))
        && planform.intermediate_stations.iter().all(|station| {
            series_has_evidence(&station.span_or_height_station_m)
                && series_has_evidence(&station.leading_edge_offset_aft_root_le_m)
                && series_has_evidence(&station.chord_m)
        })
}

fn vertical_planform_has_evidence(planform: &VerticalPlanformFile) -> bool {
    [
        planform.height_m.as_ref(),
        planform.root_chord_m.as_ref(),
        planform.tip_chord_m.as_ref(),
        planform.tip_le_offset_aft_root_le_m.as_ref(),
    ]
    .into_iter()
    .all(|series| series.is_some_and(series_has_evidence))
        && planform.intermediate_stations.iter().all(|station| {
            series_has_evidence(&station.span_or_height_station_m)
                && series_has_evidence(&station.leading_edge_offset_aft_root_le_m)
                && series_has_evidence(&station.chord_m)
        })
}

fn require_horizontal_planform_campaign(
    planform: &HorizontalPlanformFile,
    missing: &mut Vec<&'static str>,
) {
    for (blocker, series) in [
        ("horizontal_tail_span", planform.span_m.as_ref()),
        ("horizontal_tail_root_chord", planform.root_chord_m.as_ref()),
        ("horizontal_tail_tip_chord", planform.tip_chord_m.as_ref()),
        (
            "horizontal_tail_leading_edge_geometry",
            planform.tip_le_offset_aft_root_le_m.as_ref(),
        ),
    ] {
        require_value_and_evidence(blocker, series, missing);
    }
}

fn require_vertical_planform_campaign(
    planform: &VerticalPlanformFile,
    missing: &mut Vec<&'static str>,
) {
    for (blocker, series) in [
        ("vertical_tail_height", planform.height_m.as_ref()),
        ("vertical_tail_root_chord", planform.root_chord_m.as_ref()),
        ("vertical_tail_tip_chord", planform.tip_chord_m.as_ref()),
        (
            "vertical_tail_leading_edge_geometry",
            planform.tip_le_offset_aft_root_le_m.as_ref(),
        ),
    ] {
        require_value_and_evidence(blocker, series, missing);
    }
}

fn require_battery_campaign(battery: Option<&BatteryFile>, missing: &mut Vec<&'static str>) {
    let Some(battery) = battery else {
        missing.push("battery_configuration_and_location");
        return;
    };
    if battery.source_ids.is_empty()
        || battery.photograph_ids.is_empty()
        || !battery
            .longitudinal_station_aft_wing_le_m
            .as_ref()
            .is_some_and(series_has_evidence)
    {
        missing.push("battery_configuration_and_location");
    }
}

fn deduplicate(values: &mut Vec<&'static str>) {
    let mut seen = HashSet::new();
    values.retain(|value| seen.insert(*value));
}

fn compare_to_baseline(file: &SurveyFile) -> Vec<CrossVariantComparison> {
    let observations = &file.raw_observations;
    let baseline = &file.comparison_baseline;
    let tolerance = file.acceptance_criteria.cross_variant_identity_tolerance_m;
    let measured = [
        (
            "wing_quarter_chord_offset_m",
            summarize_optional(observations.wing_quarter_chord_aft_wing_le_m.as_ref()),
            baseline.wing_quarter_chord_offset_m,
        ),
        (
            "horizontal_tail.span_m",
            summarize_optional(observations.horizontal_tail_planform.span_m.as_ref()),
            baseline.horizontal_tail.span_m,
        ),
        (
            "horizontal_tail.root_chord_m",
            summarize_optional(observations.horizontal_tail_planform.root_chord_m.as_ref()),
            baseline.horizontal_tail.root_chord_m,
        ),
        (
            "horizontal_tail.tip_chord_m",
            summarize_optional(observations.horizontal_tail_planform.tip_chord_m.as_ref()),
            baseline.horizontal_tail.tip_chord_m,
        ),
        (
            "horizontal_tail.area_weighted_quarter_chord_aft_root_le_m",
            horizontal_planform_offset(&observations.horizontal_tail_planform),
            baseline
                .horizontal_tail
                .area_weighted_quarter_chord_aft_root_le_m,
        ),
        (
            "horizontal_tail.tip_le_offset_aft_root_le_m",
            summarize_optional(
                observations
                    .horizontal_tail_planform
                    .tip_le_offset_aft_root_le_m
                    .as_ref(),
            ),
            baseline.horizontal_tail.tip_le_offset_aft_root_le_m,
        ),
        (
            "vertical_tail.height_m",
            summarize_optional(observations.vertical_tail_planform.height_m.as_ref()),
            baseline.vertical_tail.height_m,
        ),
        (
            "vertical_tail.root_chord_m",
            summarize_optional(observations.vertical_tail_planform.root_chord_m.as_ref()),
            baseline.vertical_tail.root_chord_m,
        ),
        (
            "vertical_tail.tip_chord_m",
            summarize_optional(observations.vertical_tail_planform.tip_chord_m.as_ref()),
            baseline.vertical_tail.tip_chord_m,
        ),
        (
            "vertical_tail.area_weighted_quarter_chord_aft_root_le_m",
            vertical_planform_offset(&observations.vertical_tail_planform),
            baseline
                .vertical_tail
                .area_weighted_quarter_chord_aft_root_le_m,
        ),
        (
            "vertical_tail.tip_le_offset_aft_root_le_m",
            summarize_optional(
                observations
                    .vertical_tail_planform
                    .tip_le_offset_aft_root_le_m
                    .as_ref(),
            ),
            baseline.vertical_tail.tip_le_offset_aft_root_le_m,
        ),
    ];
    measured
        .into_iter()
        .map(|(quantity, measurement, reference)| {
            compare_one(quantity, measurement, reference, tolerance)
        })
        .collect()
}

fn compare_one(
    quantity: &'static str,
    measured: Option<DerivedSurveyValue>,
    reference: Option<ReferenceValueFile>,
    identity_tolerance: Option<f64>,
) -> CrossVariantComparison {
    let Some((measured, reference)) = measured.zip(reference) else {
        return CrossVariantComparison {
            quantity,
            status: CrossVariantStatus::Unknown,
            absolute_difference: None,
        };
    };
    let difference = (measured.value - reference.value).abs();
    let combined_uncertainty = measured.uncertainty.hypot(reference.uncertainty);
    let status = match identity_tolerance {
        Some(tolerance) if difference <= tolerance => CrossVariantStatus::ConfirmedIdentical,
        Some(tolerance) if difference <= tolerance + combined_uncertainty => {
            CrossVariantStatus::ConsistentButNotProven
        }
        Some(_) => CrossVariantStatus::Different,
        None if difference <= combined_uncertainty => CrossVariantStatus::ConsistentButNotProven,
        None => CrossVariantStatus::Different,
    };
    CrossVariantComparison {
        quantity,
        status,
        absolute_difference: Some(difference),
    }
}
