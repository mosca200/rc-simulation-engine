#![forbid(unsafe_code)]
//! Strict JSON aircraft-model loading and immutable resolved runtime configuration.

mod loader;
mod reference;
mod reference_aerodynamics;
mod reference_mass_properties;
mod reference_propulsion;
mod reference_survey;
mod reference_xfoil;
mod reference_xfoil_campaign;
mod reference_xfoil_convergence;
mod reference_xfoil_evidence;
mod reference_xfoil_evidence_json;
mod reference_xfoil_runtime;
mod runtime;
pub mod v0;
pub mod v1;
pub mod v2;
pub mod v3;
pub mod v4;
pub mod v5;
pub mod v6;
pub mod v7;
pub mod v8;
mod xfoil_aircraft_family_binding;

pub use loader::{AircraftModelLoader, ModelLoadError, load_aircraft_model};
pub use reference::{
    AircraftClassification, CgReferenceKind, ParameterQuality, ProvenanceConfidence,
    ProvenanceSource, ProvenanceSourceType, ReferenceAircraftIdentity, ReferenceAircraftMetadata,
    ReferenceCgLocation, ReferenceControlSurfaceTravel, ReferenceParameterEvidence,
    ReferencePhysicalSpecification, ReferenceScalar,
};
pub use reference_aerodynamics::{
    AerodynamicDatasetSummary, AerodynamicEvidence, AerodynamicEvidenceClass,
    AerodynamicEvidenceEvaluation, AerodynamicEvidenceLoader, ConvergenceStatus, CoveragePoint,
    ReferenceAerodynamicEvidenceError, load_reference_aerodynamic_evidence,
};
pub use reference_mass_properties::{
    InertiaEstimate, MassMeasurementSummary, MassPropertiesCampaign, MassPropertiesEvaluation,
    MassPropertiesLoader, PublishedWeightRangeStatus, ReferenceMassPropertiesError, ScalarEstimate,
    VectorEstimate, load_reference_mass_properties, x_aft_to_frd_x,
};
pub use reference_propulsion::{
    ApcPerformanceData, ApcPerformanceDataLoader, ApcPerformanceRow, ApcRpmBlock,
    ConfigurationClaimSummary, PropulsionConfigurationEvidenceClass, PropulsionEvidence,
    PropulsionEvidenceEvaluation, PropulsionEvidenceLoader, ReferencePropulsionEvidenceError,
    load_reference_propulsion_evidence,
};
pub use reference_survey::{
    BilateralMeasurementSummary, CrossVariantComparison, CrossVariantStatus, DerivedSurveyValue,
    MeasurementSummary, PhysicalSurvey, PhysicalSurveyLoader, ReferenceSurveyError,
    SurveyClassification, SurveyEvaluation, load_reference_survey,
};
pub use reference_xfoil::{
    InvalidMetadataReason, MetadataBuilder, XfoilPolarImport, XfoilPolarImportError,
    XfoilPolarSample, XfoilSolverMetadata, parse_xfoil_polar,
};
pub use reference_xfoil_campaign::{
    XfoilCampaignCoverage, XfoilCampaignCoverageBlocker, XfoilCampaignCoverageRequest,
    XfoilCampaignCoverageStatus, XfoilCampaignDatasetCoverage, XfoilEvidenceCampaign,
    XfoilEvidenceCampaignBuilder, XfoilEvidenceCampaignError,
};
pub use reference_xfoil_convergence::{
    SweepConvergenceBlocker, SweepConvergenceStatus, SweepExpectation, SweepExpectationError,
    XfoilSweepConvergenceQualification, qualify_sweep_convergence,
};
pub use reference_xfoil_evidence::{
    XfoilEvidenceBridgeError, XfoilEvidenceDataset, XfoilEvidenceDatasetBuilder,
};
pub use reference_xfoil_evidence_json::{
    XfoilEvidenceJsonError, build_xfoil_reynolds_polar_family_from_json,
    build_xfoil_reynolds_polar_family_from_json_str,
};
pub use reference_xfoil_runtime::{
    XfoilRuntimePolarFamily, XfoilRuntimePolarFamilyError, build_xfoil_reynolds_polar_family,
};
pub use runtime::{
    AircraftModel, AircraftModelFingerprint, ControlActuator, PresentationMetadata,
    RuntimeAeroDownwashInteraction, RuntimeAeroElement, RuntimeAeroPolarBinding,
    RuntimeAeroSurface, RuntimeControlSurfaceBinding, RuntimeElectricPropulsion,
    RuntimeLandingGearContact, RuntimePolar, RuntimePropellerSlipstreamInteraction,
    RuntimeReynoldsPolarFamily,
};
pub use xfoil_aircraft_family_binding::{
    XfoilEvidenceBindingError, XfoilEvidenceBindingResult, bind_xfoil_evidence_to_reynolds_family,
};

pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V0: u32 = 0;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V1: u32 = 1;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V2: u32 = 2;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V3: u32 = 3;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V4: u32 = 4;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V5: u32 = 5;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V6: u32 = 6;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V7: u32 = 7;
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V8: u32 = 8;
pub const REFERENCE_AERODYNAMIC_EVIDENCE_SCHEMA_V0: &str =
    "reference_aircraft_aerodynamic_evidence_v0";
pub const REFERENCE_MASS_PROPERTIES_SCHEMA_V0: &str = "reference_aircraft_mass_properties_v0";
pub const REFERENCE_PROPULSION_EVIDENCE_SCHEMA_V0: &str =
    "reference_aircraft_propulsion_evidence_v0";
pub const REFERENCE_SURVEY_SCHEMA_V0: &str = "reference_aircraft_physical_survey_v0";
