#![forbid(unsafe_code)]
//! Strict JSON aircraft-model loading and immutable resolved runtime configuration.

mod loader;
mod reference;
mod reference_survey;
mod runtime;
pub mod v0;
pub mod v1;
pub mod v2;

pub use loader::{AircraftModelLoader, ModelLoadError, load_aircraft_model};
pub use reference::{
    AircraftClassification, CgReferenceKind, ParameterQuality, ProvenanceConfidence,
    ProvenanceSource, ProvenanceSourceType, ReferenceAircraftIdentity, ReferenceAircraftMetadata,
    ReferenceCgLocation, ReferenceControlSurfaceTravel, ReferenceParameterEvidence,
    ReferencePhysicalSpecification, ReferenceScalar,
};
pub use reference_survey::{
    BilateralMeasurementSummary, CrossVariantComparison, CrossVariantStatus, DerivedSurveyValue,
    MeasurementSummary, PhysicalSurvey, PhysicalSurveyLoader, ReferenceSurveyError,
    SurveyClassification, SurveyEvaluation, load_reference_survey,
};
pub use runtime::{
    AircraftModel, AircraftModelFingerprint, ControlActuator, PresentationMetadata,
    RuntimeAeroElement, RuntimeControlSurfaceBinding, RuntimeElectricPropulsion, RuntimePolar,
};

pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V0: u32 = 0;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V1: u32 = 1;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V2: u32 = 2;
pub const REFERENCE_SURVEY_SCHEMA_V0: &str = "reference_aircraft_physical_survey_v0";
