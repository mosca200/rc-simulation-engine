#![forbid(unsafe_code)]
//! Strict JSON aircraft-model loading and immutable resolved runtime configuration.

mod loader;
mod reference;
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
pub use runtime::{
    AircraftModel, AircraftModelFingerprint, ControlActuator, PresentationMetadata,
    RuntimeAeroElement, RuntimeControlSurfaceBinding, RuntimeElectricPropulsion, RuntimePolar,
};

pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V0: u32 = 0;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V1: u32 = 1;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V2: u32 = 2;
