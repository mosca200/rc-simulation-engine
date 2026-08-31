//! Serializable authoring representation for aircraft-model schema version 2.
//!
//! Version 2 adds classification and optional reference-aircraft documentary data while
//! preserving the v1 physical simulation fields.

use crate::{
    v0::{
        AerodynamicsFileV0, ControlsFileV0, PresentationFileV0, PropulsionFileV0, RigidBodyFileV0,
    },
    v1::ControlSurfaceBindingFileV1,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV2 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV0,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV0>,
    pub presentation: Option<PresentationFileV0>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AircraftClassificationFileV2 {
    SyntheticTest,
    ReferenceAircraft,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceAircraftFileV2 {
    pub identity: ReferenceAircraftIdentityFileV2,
    pub physical_specification: ReferencePhysicalSpecificationFileV2,
    pub provenance_sources: Vec<ProvenanceSourceFileV2>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceAircraftIdentityFileV2 {
    pub manufacturer: Option<String>,
    pub aircraft_name: Option<String>,
    pub variant: Option<String>,
    pub stable_reference_id: Option<String>,
    pub notes: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferencePhysicalSpecificationFileV2 {
    pub wingspan_m: Option<ReferenceScalarFileV2>,
    pub reference_wing_area_m2: Option<ReferenceScalarFileV2>,
    pub aircraft_length_m: Option<ReferenceScalarFileV2>,
    /// Evidence for the simulation-authoritative `rigid_body.mass_kg` value.
    pub mass: Option<ReferenceParameterEvidenceFileV2>,
    pub cg_location: Option<ReferenceCgLocationFileV2>,
    pub aerodynamic_reference_chord_m: Option<ReferenceScalarFileV2>,
    pub wing_incidence_rad: Option<ReferenceScalarFileV2>,
    pub horizontal_tail_incidence_rad: Option<ReferenceScalarFileV2>,
    pub wing_dihedral_rad: Option<ReferenceScalarFileV2>,
    /// Evidence attached to simulation-authoritative binding/servo travel limits.
    pub control_surface_travel_limits: Vec<ReferenceControlSurfaceTravelFileV2>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceScalarFileV2 {
    pub value: f64,
    pub status: ParameterQualityFileV2,
    pub source_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceParameterEvidenceFileV2 {
    pub status: ParameterQualityFileV2,
    pub source_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceCgLocationFileV2 {
    pub position_m_from_reference: [f64; 3],
    pub reference: CgReferenceFileV2,
    pub status: ParameterQualityFileV2,
    pub source_ids: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CgReferenceFileV2 {
    pub kind: CgReferenceKindFileV2,
    pub description: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CgReferenceKindFileV2 {
    BodyFrameOriginFrd,
    WingRootLeadingEdge,
    MeanAerodynamicChordLeadingEdge,
    ManufacturerDatum,
    Other,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReferenceControlSurfaceTravelFileV2 {
    pub control_surface_binding_id: String,
    pub status: ParameterQualityFileV2,
    pub source_ids: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ParameterQualityFileV2 {
    Measured,
    ManufacturerSpec,
    Published,
    Derived,
    Estimated,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProvenanceSourceFileV2 {
    pub id: String,
    pub source_type: ProvenanceSourceTypeFileV2,
    pub title: Option<String>,
    pub url: Option<String>,
    pub bibliographic_reference: Option<String>,
    pub notes: Option<String>,
    pub publication_date: Option<String>,
    pub retrieval_date: Option<String>,
    pub confidence: Option<ProvenanceConfidenceFileV2>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProvenanceSourceTypeFileV2 {
    ManufacturerDocumentation,
    Measured,
    PublishedResearch,
    AirfoilDatabase,
    NumericalAnalysis,
    Derived,
    Estimated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProvenanceConfidenceFileV2 {
    Low,
    Medium,
    High,
}
