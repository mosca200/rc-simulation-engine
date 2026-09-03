use crate::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V0, AIRCRAFT_MODEL_SCHEMA_VERSION_V1,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V2, AIRCRAFT_MODEL_SCHEMA_VERSION_V3,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V4, AIRCRAFT_MODEL_SCHEMA_VERSION_V5,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V6, AIRCRAFT_MODEL_SCHEMA_VERSION_V7,
    reference::{
        AircraftClassification, CgReferenceKind, ParameterQuality, ProvenanceConfidence,
        ProvenanceSource, ProvenanceSourceType, ReferenceAircraftIdentity,
        ReferenceAircraftMetadata, ReferenceCgLocation, ReferenceControlSurfaceTravel,
        ReferenceParameterEvidence, ReferencePhysicalSpecification, ReferenceScalar,
    },
    runtime::{
        AircraftModel, ControlActuator, PresentationMetadata, RuntimeAeroDownwashInteraction,
        RuntimeAeroElement, RuntimeAeroSurface, RuntimeControlSurfaceBinding,
        RuntimeElectricPropulsion, RuntimePolar, RuntimePropellerSlipstreamInteraction,
        RuntimeReynoldsPolarFamily,
    },
    v0::{
        AerodynamicsFileV0, AircraftModelFileV0, AxisResponseFileV0, PropellerSpinDirectionFileV0,
        ServoFileV0,
    },
    v1::{AircraftModelFileV1, ControlActuatorFileV1, ControlSurfaceBindingFileV1},
    v2::{
        AircraftClassificationFileV2, AircraftModelFileV2, CgReferenceKindFileV2,
        ParameterQualityFileV2, ProvenanceConfidenceFileV2, ProvenanceSourceFileV2,
        ProvenanceSourceTypeFileV2, ReferenceAircraftFileV2, ReferenceCgLocationFileV2,
        ReferenceParameterEvidenceFileV2, ReferencePhysicalSpecificationFileV2,
        ReferenceScalarFileV2,
    },
    v3::{AeroPolarBindingFileV3, AircraftModelFileV3},
    v4::{AircraftModelFileV4, PropellerCoefficientSourceFileV4, PropulsionFileV4},
    v5::{AeroSurfaceFileV5, AircraftModelFileV5},
    v6::{AeroDownwashInteractionFileV6, AircraftModelFileV6},
    v7::{AircraftModelFileV7, PropellerSlipstreamInteractionFileV7},
};
use serde::Deserialize;
use sim_core::{
    AeroElement, AeroElementError, AxisResponseConfig, BatteryConfig, BatteryConfigError,
    ControlActuatorConfig, ControlConfigError, ControlResponseConfig, ControlSystemConfig,
    ElectricPropulsionConfig, EscConfig, EscConfigError, MotorConfig, MotorConfigError,
    ParameterError, PolarError, PolarSample, PolarTable, PropellerCoefficientError,
    PropellerCoefficientMap, PropellerCoefficientMapError, PropellerCoefficientNode,
    PropellerCoefficientSource, PropellerCoefficientTable, PropellerConfig, PropellerConfigError,
    PropellerSample, PropellerSpinDirection, ReynoldsPolar, ReynoldsPolarFamily,
    ReynoldsPolarFamilyError, RigidBodyParams, ServoConfig,
};
use sim_math::{Mat3, Orientation, Quaternion, Vec3};
use std::{fs, io, path::Path};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ModelLoadError {
    #[error("failed to read aircraft model {path}: {source}")]
    Io {
        path: String,
        #[source]
        source: io::Error,
    },
    #[error("aircraft model is not valid JSON: {source}")]
    JsonParse {
        #[source]
        source: serde_json::Error,
    },
    #[error("aircraft model structure is invalid: {source}")]
    InvalidStructure {
        #[source]
        source: serde_json::Error,
    },
    #[error("unsupported aircraft-model schema version {found}")]
    UnsupportedSchemaVersion { found: u64 },
    #[error("invalid model_id {value:?}; expected nonempty [a-z0-9_-]+")]
    InvalidModelId { value: String },
    #[error("invalid {kind} ID {value:?} at index {index}; expected nonempty [a-z0-9_-]+")]
    InvalidStableId {
        kind: &'static str,
        index: usize,
        value: String,
    },
    #[error(
        "duplicate {kind} ID {id:?} at index {duplicate_index}; first declared at index {first_index}"
    )]
    DuplicateStableId {
        kind: &'static str,
        id: String,
        first_index: usize,
        duplicate_index: usize,
    },
    #[error("invalid rigid-body parameters: {source}")]
    InvalidRigidBody {
        #[source]
        source: ParameterError,
    },
    #[error("invalid polar {id:?} at index {index}: {source}")]
    InvalidPolar {
        id: String,
        index: usize,
        #[source]
        source: PolarError,
    },
    #[error("kinematic_viscosity_m2_s must be finite and greater than zero, got {value:?}")]
    InvalidKinematicViscosity { value: f64 },
    #[error(
        "invalid polar table for Reynolds family {family_id:?} at family index {family_index}, node index {node_index}: {source}"
    )]
    InvalidReynoldsFamilyPolar {
        family_id: String,
        family_index: usize,
        node_index: usize,
        #[source]
        source: PolarError,
    },
    #[error(
        "invalid Reynolds family {family_id:?} at family index {family_index}, node index {node_index:?}: {source}"
    )]
    InvalidReynoldsPolarFamily {
        family_id: String,
        family_index: usize,
        node_index: Option<usize>,
        #[source]
        source: ReynoldsPolarFamilyError,
    },
    #[error("invalid aerodynamic element {id:?} at index {index}: {source}")]
    InvalidAeroElement {
        id: String,
        index: usize,
        #[source]
        source: AeroElementError,
    },
    #[error(
        "aerodynamic element {element_id:?} at index {element_index} references unknown polar {polar_id:?}"
    )]
    UnresolvedPolarReference {
        element_id: String,
        element_index: usize,
        polar_id: String,
    },
    #[error(
        "aerodynamic element {element_id:?} at index {element_index} references unknown Reynolds family {family_id:?}"
    )]
    UnresolvedReynoldsFamilyReference {
        element_id: String,
        element_index: usize,
        family_id: String,
    },
    #[error(
        "control-surface binding {binding_id:?} at index {binding_index} references unknown aerodynamic element {element_id:?}"
    )]
    UnresolvedControlSurfaceElementReference {
        binding_id: String,
        binding_index: usize,
        element_id: String,
    },
    #[error(
        "aerodynamic element {element_id:?} is controlled by binding {binding_id:?} at index {duplicate_index}, but was already controlled by binding {first_binding_id:?} at index {first_index}"
    )]
    DuplicateControlledAeroElement {
        element_id: String,
        first_binding_id: String,
        first_index: usize,
        binding_id: String,
        duplicate_index: usize,
    },
    #[error(
        "invalid deflection_gain {value:?} for control-surface binding {binding_id:?} at index {binding_index}; expected a finite nonzero value"
    )]
    InvalidControlSurfaceDeflectionGain {
        binding_id: String,
        binding_index: usize,
        value: f64,
    },
    #[error("invalid controls component {component}: {source}")]
    InvalidControls {
        component: &'static str,
        #[source]
        source: ControlConfigError,
    },
    #[error("invalid propulsion battery: {source}")]
    InvalidBattery {
        #[source]
        source: BatteryConfigError,
    },
    #[error("invalid propulsion ESC: {source}")]
    InvalidEsc {
        #[source]
        source: EscConfigError,
    },
    #[error("invalid propulsion motor: {source}")]
    InvalidMotor {
        #[source]
        source: MotorConfigError,
    },
    #[error("invalid propulsion propeller: {source}")]
    InvalidPropeller {
        #[source]
        source: PropellerConfigError,
    },
    #[error("invalid propulsion coefficient table: {source}")]
    InvalidPropellerCoefficientTable {
        #[source]
        source: PropellerCoefficientError,
    },
    #[error("invalid propulsion coefficient map node {node_index:?}: {source}")]
    InvalidPropellerCoefficientMap {
        node_index: Option<usize>,
        #[source]
        source: PropellerCoefficientMapError,
    },
    #[error(
        "invalid presentation GLB path {path:?}: expected a nonempty relative path without '..'"
    )]
    InvalidPresentationAssetPath { path: String },
    #[error("synthetic_test model must not contain reference_aircraft metadata")]
    UnexpectedReferenceAircraftMetadata,
    #[error("reference_aircraft model requires a reference_aircraft metadata object")]
    MissingReferenceAircraftMetadata,
    #[error("invalid optional text at {field}: expected nonempty text when present")]
    InvalidReferenceText { field: String },
    #[error("invalid stable reference ID {value:?}; expected nonempty [a-z0-9_-]+")]
    InvalidReferenceId { value: String },
    #[error("invalid reference physical value at {field}: {value:?}; {requirement}")]
    InvalidReferencePhysicalValue {
        field: &'static str,
        value: f64,
        requirement: &'static str,
    },
    #[error("invalid reference CG position: all coordinates must be finite")]
    InvalidReferenceCgPosition,
    #[error("CG reference kind {kind:?} requires a nonempty description")]
    InvalidReferenceCgDefinition { kind: CgReferenceKindFileV2 },
    #[error("reference parameter {parameter} references unknown provenance source {source_id:?}")]
    UnresolvedProvenanceReference {
        parameter: String,
        source_id: String,
    },
    #[error(
        "reference parameter {parameter} contains duplicate provenance source reference {source_id:?}"
    )]
    DuplicateProvenanceReference {
        parameter: String,
        source_id: String,
    },
    #[error(
        "reference control travel at index {index} references unknown control-surface binding {binding_id:?}"
    )]
    UnresolvedReferenceControlSurfaceBinding { index: usize, binding_id: String },
    #[error(
        "reference control travel at index {duplicate_index} duplicates binding {binding_id:?} first declared at index {first_index}"
    )]
    DuplicateReferenceControlSurfaceBinding {
        binding_id: String,
        first_index: usize,
        duplicate_index: usize,
    },
    #[error("aerodynamic surface {surface_id:?} at index {surface_index} has empty element_ids")]
    EmptySurfaceMembership {
        surface_id: String,
        surface_index: usize,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} references unknown element {element_id:?}"
    )]
    UnresolvedSurfaceElementReference {
        surface_id: String,
        surface_index: usize,
        element_id: String,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} contains duplicate element {element_id:?}"
    )]
    DuplicateSurfaceElement {
        surface_id: String,
        surface_index: usize,
        element_id: String,
    },
    #[error(
        "aerodynamic element {element_id:?} is assigned to both surface {first_surface_id:?} at index {first_surface_index} and surface {surface_id:?} at index {surface_index}"
    )]
    CrossSurfaceDuplicateElement {
        element_id: Box<str>,
        first_surface_id: Box<str>,
        first_surface_index: usize,
        surface_id: Box<str>,
        surface_index: usize,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has invalid span_axis_body: {reason}"
    )]
    InvalidSurfaceSpanAxis {
        surface_id: String,
        surface_index: usize,
        reason: &'static str,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has invalid span_m {value:?}; expected finite and greater than zero"
    )]
    InvalidSurfaceSpan {
        surface_id: String,
        surface_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has invalid span_efficiency_factor {value:?}; expected finite and greater than zero"
    )]
    InvalidSurfaceSpanEfficiency {
        surface_id: String,
        surface_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has non-finite derived area {value:?}"
    )]
    NonFiniteSurfaceArea {
        surface_id: String,
        surface_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has non-positive derived area {value:?}"
    )]
    NonPositiveSurfaceArea {
        surface_id: String,
        surface_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic surface {surface_id:?} at index {surface_index} has invalid derived aspect_ratio {value:?}; expected finite and greater than zero"
    )]
    InvalidSurfaceAspectRatio {
        surface_id: String,
        surface_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic downwash interaction {interaction_id:?} at index {interaction_index} references unknown source surface {surface_id:?}"
    )]
    UnresolvedDownwashSourceSurface {
        interaction_id: Box<str>,
        interaction_index: usize,
        surface_id: Box<str>,
    },
    #[error(
        "aerodynamic downwash interaction {interaction_id:?} at index {interaction_index} references unknown target surface {surface_id:?}"
    )]
    UnresolvedDownwashTargetSurface {
        interaction_id: Box<str>,
        interaction_index: usize,
        surface_id: Box<str>,
    },
    #[error(
        "aerodynamic downwash interaction {interaction_id:?} at index {interaction_index} uses surface {surface_id:?} as both source and target"
    )]
    DownwashSelfInteraction {
        interaction_id: Box<str>,
        interaction_index: usize,
        surface_id: Box<str>,
    },
    #[error(
        "aerodynamic downwash interaction {interaction_id:?} at index {interaction_index} has invalid downwash_factor {value:?}; expected finite and non-negative"
    )]
    InvalidDownwashFactor {
        interaction_id: Box<str>,
        interaction_index: usize,
        value: f64,
    },
    #[error(
        "aerodynamic downwash interaction {interaction_id:?} at index {interaction_index} targets surface {surface_id:?}, already targeted by interaction {first_interaction_id:?} at index {first_interaction_index}"
    )]
    DuplicateDownwashTarget {
        interaction_id: Box<str>,
        interaction_index: usize,
        surface_id: Box<str>,
        first_interaction_id: Box<str>,
        first_interaction_index: usize,
    },
    #[error(
        "aerodynamic downwash graph is chained: surface {surface_id:?} is both a source and a target"
    )]
    ChainedDownwashSurface { surface_id: Box<str> },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} has no target elements"
    )]
    EmptySlipstreamTargets {
        interaction_id: Box<str>,
        interaction_index: usize,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} has invalid slipstream_velocity_factor {value:?}; expected finite and non-negative"
    )]
    InvalidSlipstreamVelocityFactor {
        interaction_id: Box<str>,
        interaction_index: usize,
        value: f64,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} has invalid swirl_velocity_factor {value:?}; expected finite and non-negative"
    )]
    InvalidSwirlVelocityFactor {
        interaction_id: Box<str>,
        interaction_index: usize,
        value: f64,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} requires propulsion"
    )]
    SlipstreamInteractionWithoutPropulsion {
        interaction_id: Box<str>,
        interaction_index: usize,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} references unknown target element {element_id:?} at target index {target_index}"
    )]
    UnresolvedSlipstreamTargetElement {
        interaction_id: Box<str>,
        interaction_index: usize,
        target_index: usize,
        element_id: Box<str>,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} repeats target element {element_id:?} at target index {duplicate_target_index}; first declared at target index {first_target_index}"
    )]
    DuplicateSlipstreamTargetWithinInteraction {
        interaction_id: Box<str>,
        interaction_index: usize,
        element_id: Box<str>,
        first_target_index: usize,
        duplicate_target_index: usize,
    },
    #[error(
        "propeller slipstream interaction {interaction_id:?} at index {interaction_index} targets element {element_id:?}, already targeted by interaction {first_interaction_id:?} at index {first_interaction_index}"
    )]
    DuplicateSlipstreamTarget {
        interaction_id: Box<str>,
        interaction_index: usize,
        element_id: Box<str>,
        first_interaction_id: Box<str>,
        first_interaction_index: usize,
    },
}

#[derive(Debug, Default, Clone, Copy)]
pub struct AircraftModelLoader;

impl AircraftModelLoader {
    pub fn from_json_str(json: &str) -> Result<AircraftModel, ModelLoadError> {
        let version = serde_json::from_str::<VersionProbe>(json)
            .map_err(classify_probe_error)?
            .schema_version;
        match version {
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V0) => {
                let file: AircraftModelFileV0 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V0);
                resolve_v0(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V1) => {
                let file: AircraftModelFileV1 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V1);
                resolve_v1(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V2) => {
                let file: AircraftModelFileV2 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V2);
                resolve_v2(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V3) => {
                let file: AircraftModelFileV3 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V3);
                resolve_v3(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V4) => {
                let file: AircraftModelFileV4 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V4);
                resolve_v4(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V5) => {
                let file: AircraftModelFileV5 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V5);
                resolve_v5(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V6) => {
                let file: AircraftModelFileV6 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V6);
                resolve_v6(file)
            }
            version if version == u64::from(AIRCRAFT_MODEL_SCHEMA_VERSION_V7) => {
                let file: AircraftModelFileV7 = serde_json::from_str(json)
                    .map_err(|source| ModelLoadError::InvalidStructure { source })?;
                debug_assert_eq!(file.schema_version, AIRCRAFT_MODEL_SCHEMA_VERSION_V7);
                resolve_v7(file)
            }
            found => Err(ModelLoadError::UnsupportedSchemaVersion { found }),
        }
    }
}

pub fn load_aircraft_model(path: impl AsRef<Path>) -> Result<AircraftModel, ModelLoadError> {
    let path = path.as_ref();
    let json = fs::read_to_string(path).map_err(|source| ModelLoadError::Io {
        path: path.display().to_string(),
        source,
    })?;
    AircraftModelLoader::from_json_str(&json)
}

#[derive(Deserialize)]
struct VersionProbe {
    schema_version: u64,
}

fn classify_probe_error(source: serde_json::Error) -> ModelLoadError {
    match source.classify() {
        serde_json::error::Category::Syntax | serde_json::error::Category::Eof => {
            ModelLoadError::JsonParse { source }
        }
        serde_json::error::Category::Data | serde_json::error::Category::Io => {
            ModelLoadError::InvalidStructure { source }
        }
    }
}

fn resolve_v0(file: AircraftModelFileV0) -> Result<AircraftModel, ModelLoadError> {
    resolve_common(file, AIRCRAFT_MODEL_SCHEMA_VERSION_V0)
}

fn resolve_v1(file: AircraftModelFileV1) -> Result<AircraftModel, ModelLoadError> {
    resolve_v1_fields(file, AIRCRAFT_MODEL_SCHEMA_VERSION_V1)
}

fn resolve_v1_fields(
    file: AircraftModelFileV1,
    runtime_schema_version: u32,
) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV1 {
        schema_version,
        model_id,
        display_name,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    } = file;
    let common_file = AircraftModelFileV0 {
        schema_version,
        model_id,
        display_name,
        rigid_body,
        aerodynamics,
        controls,
        propulsion,
        presentation,
    };
    let model = resolve_common(common_file, runtime_schema_version)?;
    let bindings = resolve_control_surface_bindings(&model, control_surface_bindings)?;
    Ok(model.with_control_surface_bindings(bindings))
}

fn resolve_v2(file: AircraftModelFileV2) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV2 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    } = file;
    let v1_file = AircraftModelFileV1 {
        schema_version,
        model_id,
        display_name,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    };
    let model = resolve_v1_fields(v1_file, AIRCRAFT_MODEL_SCHEMA_VERSION_V2)?;
    resolve_reference_framework(model, classification, reference_aircraft)
}

fn resolve_reference_framework(
    model: AircraftModel,
    classification: AircraftClassificationFileV2,
    reference_aircraft: Option<ReferenceAircraftFileV2>,
) -> Result<AircraftModel, ModelLoadError> {
    let classification = match classification {
        AircraftClassificationFileV2::SyntheticTest => AircraftClassification::SyntheticTest,
        AircraftClassificationFileV2::ReferenceAircraft => {
            AircraftClassification::ReferenceAircraft
        }
    };
    let reference_aircraft = match (classification, reference_aircraft) {
        (AircraftClassification::SyntheticTest, None) => None,
        (AircraftClassification::SyntheticTest, Some(_)) => {
            return Err(ModelLoadError::UnexpectedReferenceAircraftMetadata);
        }
        (AircraftClassification::ReferenceAircraft, None) => {
            return Err(ModelLoadError::MissingReferenceAircraftMetadata);
        }
        (AircraftClassification::ReferenceAircraft, Some(file)) => {
            Some(resolve_reference_aircraft(&model, file)?)
        }
    };
    Ok(model.with_reference_framework(classification, reference_aircraft))
}

fn resolve_v3(file: AircraftModelFileV3) -> Result<AircraftModel, ModelLoadError> {
    resolve_v3_fields(file, AIRCRAFT_MODEL_SCHEMA_VERSION_V3)
}

fn resolve_v3_fields(
    file: AircraftModelFileV3,
    runtime_schema_version: u32,
) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV3 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    } = file;
    if !aerodynamics.kinematic_viscosity_m2_s.is_finite()
        || aerodynamics.kinematic_viscosity_m2_s <= 0.0
    {
        return Err(ModelLoadError::InvalidKinematicViscosity {
            value: aerodynamics.kinematic_viscosity_m2_s,
        });
    }

    let common_file = AircraftModelFileV0 {
        schema_version,
        model_id,
        display_name,
        rigid_body,
        aerodynamics: AerodynamicsFileV0 {
            polars: aerodynamics.polars,
            elements: Vec::new(),
        },
        controls,
        propulsion,
        presentation,
    };
    let mut model = resolve_common(common_file, runtime_schema_version)?;

    let mut families = Vec::with_capacity(aerodynamics.polar_families.len());
    for (family_index, family_file) in aerodynamics.polar_families.into_iter().enumerate() {
        validate_unique_id(
            "Reynolds polar family",
            family_index,
            &family_file.id,
            families.iter().map(RuntimeReynoldsPolarFamily::id),
        )?;
        let mut nodes = Vec::with_capacity(family_file.nodes.len());
        for (node_index, node_file) in family_file.nodes.into_iter().enumerate() {
            let samples = node_file
                .samples
                .into_iter()
                .map(|sample| PolarSample {
                    alpha_rad: sample.alpha_rad,
                    cl: sample.cl,
                    cd: sample.cd,
                    cm: sample.cm,
                })
                .collect();
            let table = PolarTable::new(samples).map_err(|source| {
                ModelLoadError::InvalidReynoldsFamilyPolar {
                    family_id: family_file.id.clone(),
                    family_index,
                    node_index,
                    source,
                }
            })?;
            nodes.push(
                ReynoldsPolar::new(node_file.reynolds_number, table).map_err(|source| {
                    ModelLoadError::InvalidReynoldsPolarFamily {
                        family_id: family_file.id.clone(),
                        family_index,
                        node_index: Some(node_index),
                        source,
                    }
                })?,
            );
        }
        let family = ReynoldsPolarFamily::new(nodes).map_err(|source| {
            ModelLoadError::InvalidReynoldsPolarFamily {
                family_id: family_file.id.clone(),
                family_index,
                node_index: None,
                source,
            }
        })?;
        families.push(RuntimeReynoldsPolarFamily::new(family_file.id, family));
    }

    let mut elements = Vec::with_capacity(aerodynamics.elements.len());
    for (element_index, element_file) in aerodynamics.elements.into_iter().enumerate() {
        validate_unique_id(
            "aerodynamic element",
            element_index,
            &element_file.id,
            elements.iter().map(RuntimeAeroElement::id),
        )?;
        let element = AeroElement::new(
            vector(element_file.position_body_m),
            orientation(element_file.orientation_body_from_element_wxyz),
            element_file.area_m2,
            element_file.chord_m,
        )
        .map_err(|source| ModelLoadError::InvalidAeroElement {
            id: element_file.id.clone(),
            index: element_index,
            source,
        })?;
        let runtime_element = match element_file.polar_binding {
            AeroPolarBindingFileV3::Polar { polar_id } => {
                let polar_index = model
                    .aero_polars()
                    .iter()
                    .position(|polar| polar.id() == polar_id)
                    .ok_or_else(|| ModelLoadError::UnresolvedPolarReference {
                        element_id: element_file.id.clone(),
                        element_index,
                        polar_id,
                    })?;
                RuntimeAeroElement::new(element_file.id, element, polar_index)
            }
            AeroPolarBindingFileV3::ReynoldsFamily { family_id } => {
                let family_index = families
                    .iter()
                    .position(|family| family.id() == family_id)
                    .ok_or_else(|| ModelLoadError::UnresolvedReynoldsFamilyReference {
                        element_id: element_file.id.clone(),
                        element_index,
                        family_id,
                    })?;
                RuntimeAeroElement::new_reynolds_family(element_file.id, element, family_index)
            }
        };
        elements.push(runtime_element);
    }

    model =
        model.with_reynolds_aerodynamics(aerodynamics.kinematic_viscosity_m2_s, families, elements);
    let bindings = resolve_control_surface_bindings(&model, control_surface_bindings)?;
    model = model.with_control_surface_bindings(bindings);
    resolve_reference_framework(model, classification, reference_aircraft)
}

fn resolve_v4(file: AircraftModelFileV4) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV4 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    } = file;
    let runtime_propulsion = propulsion.map(resolve_propulsion_v4).transpose()?;
    let v3_file = AircraftModelFileV3 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion: None,
        presentation,
    };
    Ok(
        resolve_v3_fields(v3_file, AIRCRAFT_MODEL_SCHEMA_VERSION_V4)?
            .with_propulsion(runtime_propulsion),
    )
}

fn resolve_v5(file: AircraftModelFileV5) -> Result<AircraftModel, ModelLoadError> {
    resolve_v5_fields(file, AIRCRAFT_MODEL_SCHEMA_VERSION_V5)
}

fn resolve_v5_fields(
    file: AircraftModelFileV5,
    runtime_schema_version: u32,
) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV5 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    } = file;
    let runtime_propulsion = propulsion.map(resolve_propulsion_v5).transpose()?;
    let surfaces = aerodynamics.surfaces.clone();
    let v3_file = AircraftModelFileV3 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics: crate::v3::AerodynamicsFileV3 {
            kinematic_viscosity_m2_s: aerodynamics.kinematic_viscosity_m2_s,
            polars: aerodynamics.polars,
            polar_families: aerodynamics.polar_families,
            elements: aerodynamics.elements,
        },
        controls,
        control_surface_bindings,
        propulsion: None,
        presentation,
    };
    let model =
        resolve_v3_fields(v3_file, runtime_schema_version)?.with_propulsion(runtime_propulsion);
    let runtime_surfaces = resolve_aero_surfaces(&model, surfaces)?;
    Ok(model.with_aero_surfaces(runtime_surfaces))
}

fn resolve_v6(file: AircraftModelFileV6) -> Result<AircraftModel, ModelLoadError> {
    resolve_v6_fields(file, AIRCRAFT_MODEL_SCHEMA_VERSION_V6)
}

fn resolve_v6_fields(
    file: AircraftModelFileV6,
    runtime_schema_version: u32,
) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV6 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        aero_downwash_interactions,
        propulsion,
        presentation,
    } = file;
    let v5_file = AircraftModelFileV5 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        propulsion,
        presentation,
    };
    let model = resolve_v5_fields(v5_file, runtime_schema_version)?;
    let interactions = resolve_aero_downwash_interactions(&model, aero_downwash_interactions)?;
    Ok(model.with_aero_downwash_interactions(interactions))
}

fn resolve_v7(file: AircraftModelFileV7) -> Result<AircraftModel, ModelLoadError> {
    let AircraftModelFileV7 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        aero_downwash_interactions,
        propeller_slipstream_interactions,
        propulsion,
        presentation,
    } = file;
    let v6_file = AircraftModelFileV6 {
        schema_version,
        model_id,
        display_name,
        classification,
        reference_aircraft,
        rigid_body,
        aerodynamics,
        controls,
        control_surface_bindings,
        aero_downwash_interactions,
        propulsion,
        presentation,
    };
    let model = resolve_v6_fields(v6_file, AIRCRAFT_MODEL_SCHEMA_VERSION_V7)?;
    let interactions =
        resolve_propeller_slipstream_interactions(&model, propeller_slipstream_interactions)?;
    Ok(model.with_propeller_slipstream_interactions(interactions))
}

fn resolve_propeller_slipstream_interactions(
    model: &AircraftModel,
    interaction_files: Vec<PropellerSlipstreamInteractionFileV7>,
) -> Result<Vec<RuntimePropellerSlipstreamInteraction>, ModelLoadError> {
    let mut interactions = Vec::with_capacity(interaction_files.len());
    for (interaction_index, file) in interaction_files.into_iter().enumerate() {
        validate_unique_id(
            "propeller slipstream interaction",
            interaction_index,
            &file.id,
            interactions
                .iter()
                .map(RuntimePropellerSlipstreamInteraction::id),
        )?;
        if !file.slipstream_velocity_factor.is_finite() || file.slipstream_velocity_factor < 0.0 {
            return Err(ModelLoadError::InvalidSlipstreamVelocityFactor {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
                value: file.slipstream_velocity_factor,
            });
        }
        if !file.swirl_velocity_factor.is_finite() || file.swirl_velocity_factor < 0.0 {
            return Err(ModelLoadError::InvalidSwirlVelocityFactor {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
                value: file.swirl_velocity_factor,
            });
        }
        if model.propulsion().is_none() {
            return Err(ModelLoadError::SlipstreamInteractionWithoutPropulsion {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
            });
        }
        if file.target_element_ids.is_empty() {
            return Err(ModelLoadError::EmptySlipstreamTargets {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
            });
        }

        let mut target_element_indices = Vec::with_capacity(file.target_element_ids.len());
        for (target_index, element_id) in file.target_element_ids.iter().enumerate() {
            let element_index = model
                .aero_elements()
                .iter()
                .position(|element| element.id() == element_id)
                .ok_or_else(|| ModelLoadError::UnresolvedSlipstreamTargetElement {
                    interaction_id: file.id.clone().into_boxed_str(),
                    interaction_index,
                    target_index,
                    element_id: element_id.clone().into_boxed_str(),
                })?;
            if let Some(first_target_index) = target_element_indices
                .iter()
                .position(|&index| index == element_index)
            {
                return Err(ModelLoadError::DuplicateSlipstreamTargetWithinInteraction {
                    interaction_id: file.id.into_boxed_str(),
                    interaction_index,
                    element_id: element_id.clone().into_boxed_str(),
                    first_target_index,
                    duplicate_target_index: target_index,
                });
            }
            if let Some((first_interaction_index, first)) =
                interactions.iter().enumerate().find(|(_, interaction)| {
                    interaction
                        .target_element_indices()
                        .contains(&element_index)
                })
            {
                return Err(ModelLoadError::DuplicateSlipstreamTarget {
                    interaction_id: file.id.into_boxed_str(),
                    interaction_index,
                    element_id: element_id.clone().into_boxed_str(),
                    first_interaction_id: first.id().into(),
                    first_interaction_index,
                });
            }
            target_element_indices.push(element_index);
        }
        interactions.push(RuntimePropellerSlipstreamInteraction::new(
            file.id,
            target_element_indices,
            file.slipstream_velocity_factor,
            file.swirl_velocity_factor,
        ));
    }
    Ok(interactions)
}

fn resolve_aero_downwash_interactions(
    model: &AircraftModel,
    interaction_files: Vec<AeroDownwashInteractionFileV6>,
) -> Result<Vec<RuntimeAeroDownwashInteraction>, ModelLoadError> {
    let mut interactions = Vec::with_capacity(interaction_files.len());
    for (interaction_index, file) in interaction_files.into_iter().enumerate() {
        validate_unique_id(
            "aerodynamic downwash interaction",
            interaction_index,
            &file.id,
            interactions.iter().map(RuntimeAeroDownwashInteraction::id),
        )?;
        if !file.downwash_factor.is_finite() || file.downwash_factor < 0.0 {
            return Err(ModelLoadError::InvalidDownwashFactor {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
                value: file.downwash_factor,
            });
        }
        let source_surface_index = model
            .aero_surfaces()
            .iter()
            .position(|surface| surface.id() == file.source_surface_id)
            .ok_or_else(|| ModelLoadError::UnresolvedDownwashSourceSurface {
                interaction_id: file.id.clone().into_boxed_str(),
                interaction_index,
                surface_id: file.source_surface_id.clone().into_boxed_str(),
            })?;
        let target_surface_index = model
            .aero_surfaces()
            .iter()
            .position(|surface| surface.id() == file.target_surface_id)
            .ok_or_else(|| ModelLoadError::UnresolvedDownwashTargetSurface {
                interaction_id: file.id.clone().into_boxed_str(),
                interaction_index,
                surface_id: file.target_surface_id.clone().into_boxed_str(),
            })?;
        if source_surface_index == target_surface_index {
            return Err(ModelLoadError::DownwashSelfInteraction {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
                surface_id: file.source_surface_id.into_boxed_str(),
            });
        }
        if let Some((first_interaction_index, first)) = interactions
            .iter()
            .enumerate()
            .find(|(_, interaction)| interaction.target_surface_index() == target_surface_index)
        {
            return Err(ModelLoadError::DuplicateDownwashTarget {
                interaction_id: file.id.into_boxed_str(),
                interaction_index,
                surface_id: file.target_surface_id.into_boxed_str(),
                first_interaction_id: first.id().into(),
                first_interaction_index,
            });
        }
        interactions.push(RuntimeAeroDownwashInteraction::new(
            file.id,
            source_surface_index,
            target_surface_index,
            file.downwash_factor,
        ));
    }

    for (surface_index, surface) in model.aero_surfaces().iter().enumerate() {
        let is_source = interactions
            .iter()
            .any(|interaction| interaction.source_surface_index() == surface_index);
        let is_target = interactions
            .iter()
            .any(|interaction| interaction.target_surface_index() == surface_index);
        if is_source && is_target {
            return Err(ModelLoadError::ChainedDownwashSurface {
                surface_id: surface.id().into(),
            });
        }
    }

    Ok(interactions)
}

fn resolve_propulsion_v5(
    file: crate::v5::PropulsionFileV5,
) -> Result<RuntimeElectricPropulsion, ModelLoadError> {
    let v4_file = PropulsionFileV4 {
        battery: file.battery,
        esc: file.esc,
        motor: file.motor,
        propeller: file.propeller,
        coefficient_source: file.coefficient_source,
    };
    resolve_propulsion_v4(v4_file)
}

fn resolve_aero_surfaces(
    model: &AircraftModel,
    surface_files: Vec<AeroSurfaceFileV5>,
) -> Result<Vec<RuntimeAeroSurface>, ModelLoadError> {
    let mut runtime_surfaces = Vec::with_capacity(surface_files.len());
    let mut assigned_elements: Vec<(usize, String, usize)> = Vec::new();

    for (surface_index, surface_file) in surface_files.into_iter().enumerate() {
        validate_unique_id(
            "aerodynamic surface",
            surface_index,
            &surface_file.id,
            runtime_surfaces.iter().map(RuntimeAeroSurface::id),
        )?;

        if surface_file.element_ids.is_empty() {
            return Err(ModelLoadError::EmptySurfaceMembership {
                surface_id: surface_file.id,
                surface_index,
            });
        }

        let mut element_indices = Vec::with_capacity(surface_file.element_ids.len());
        for element_id in &surface_file.element_ids {
            let element_index = model
                .aero_elements()
                .iter()
                .position(|element| element.id() == *element_id)
                .ok_or_else(|| ModelLoadError::UnresolvedSurfaceElementReference {
                    surface_id: surface_file.id.clone(),
                    surface_index,
                    element_id: element_id.clone(),
                })?;

            if element_indices.contains(&element_index) {
                return Err(ModelLoadError::DuplicateSurfaceElement {
                    surface_id: surface_file.id.clone(),
                    surface_index,
                    element_id: element_id.clone(),
                });
            }

            if let Some((_, first_surface_id, first_surface_index)) = assigned_elements
                .iter()
                .find(|(idx, _, _)| *idx == element_index)
            {
                return Err(ModelLoadError::CrossSurfaceDuplicateElement {
                    element_id: element_id.as_str().into(),
                    first_surface_id: first_surface_id.as_str().into(),
                    first_surface_index: *first_surface_index,
                    surface_id: surface_file.id.as_str().into(),
                    surface_index,
                });
            }

            element_indices.push(element_index);
            assigned_elements.push((element_index, surface_file.id.clone(), surface_index));
        }

        let span_axis = vector(surface_file.span_axis_body);
        let norm = span_axis.norm();
        if !norm.is_finite() || norm <= 1.0e-12 {
            return Err(ModelLoadError::InvalidSurfaceSpanAxis {
                surface_id: surface_file.id,
                surface_index,
                reason: "expected finite vector with norm greater than 1e-12",
            });
        }
        let span_axis_body = span_axis.normalize();

        if !surface_file.span_m.is_finite() || surface_file.span_m <= 0.0 {
            return Err(ModelLoadError::InvalidSurfaceSpan {
                surface_id: surface_file.id,
                surface_index,
                value: surface_file.span_m,
            });
        }

        if !surface_file.span_efficiency_factor.is_finite()
            || surface_file.span_efficiency_factor <= 0.0
        {
            return Err(ModelLoadError::InvalidSurfaceSpanEfficiency {
                surface_id: surface_file.id,
                surface_index,
                value: surface_file.span_efficiency_factor,
            });
        }

        let area_m2: f64 = element_indices
            .iter()
            .map(|&idx| model.aero_elements()[idx].element().area_m2())
            .sum();

        if !area_m2.is_finite() {
            return Err(ModelLoadError::NonFiniteSurfaceArea {
                surface_id: surface_file.id,
                surface_index,
                value: area_m2,
            });
        }

        if area_m2 <= 0.0 {
            return Err(ModelLoadError::NonPositiveSurfaceArea {
                surface_id: surface_file.id,
                surface_index,
                value: area_m2,
            });
        }

        let aspect_ratio = surface_file.span_m * surface_file.span_m / area_m2;

        if !aspect_ratio.is_finite() || aspect_ratio <= 0.0 {
            return Err(ModelLoadError::InvalidSurfaceAspectRatio {
                surface_id: surface_file.id,
                surface_index,
                value: aspect_ratio,
            });
        }

        runtime_surfaces.push(RuntimeAeroSurface::new(
            surface_file.id,
            element_indices,
            span_axis_body,
            surface_file.span_m,
            surface_file.span_efficiency_factor,
            area_m2,
            aspect_ratio,
        ));
    }

    Ok(runtime_surfaces)
}

fn resolve_propulsion_v4(
    file: PropulsionFileV4,
) -> Result<RuntimeElectricPropulsion, ModelLoadError> {
    let battery = BatteryConfig::new(
        file.battery.open_circuit_voltage_v,
        file.battery.internal_resistance_ohm,
    )
    .map_err(|source| ModelLoadError::InvalidBattery { source })?;
    let esc = EscConfig::new(file.esc.series_resistance_ohm)
        .map_err(|source| ModelLoadError::InvalidEsc { source })?;
    let motor = MotorConfig::new(
        file.motor.kv_rpm_per_v,
        file.motor.winding_resistance_ohm,
        file.motor.no_load_current_a,
    )
    .map_err(|source| ModelLoadError::InvalidMotor { source })?;
    let spin_direction = match file.propeller.spin_direction {
        PropellerSpinDirectionFileV0::PositiveAboutLocalX => {
            PropellerSpinDirection::PositiveAboutLocalX
        }
        PropellerSpinDirectionFileV0::NegativeAboutLocalX => {
            PropellerSpinDirection::NegativeAboutLocalX
        }
    };
    let propeller = PropellerConfig::new(
        vector(file.propeller.position_body_m),
        orientation(file.propeller.orientation_body_from_prop_wxyz),
        file.propeller.diameter_m,
        spin_direction,
    )
    .map_err(|source| ModelLoadError::InvalidPropeller { source })?;
    let coefficient_source = match file.coefficient_source {
        PropellerCoefficientSourceFileV4::FixedTable { samples } => {
            PropellerCoefficientSource::FixedTable(resolve_propeller_table(samples)?)
        }
        PropellerCoefficientSourceFileV4::ShaftSpeedMap { nodes } => {
            let mut runtime_nodes = Vec::with_capacity(nodes.len());
            for (node_index, node) in nodes.into_iter().enumerate() {
                let table = resolve_propeller_table(node.samples)?;
                runtime_nodes.push(
                    PropellerCoefficientNode::new(node.shaft_speed_rad_s, table).map_err(
                        |source| ModelLoadError::InvalidPropellerCoefficientMap {
                            node_index: Some(node_index),
                            source,
                        },
                    )?,
                );
            }
            PropellerCoefficientSource::ShaftSpeedMap(
                PropellerCoefficientMap::new(runtime_nodes).map_err(|source| {
                    ModelLoadError::InvalidPropellerCoefficientMap {
                        node_index: None,
                        source,
                    }
                })?,
            )
        }
    };
    Ok(RuntimeElectricPropulsion::new(
        ElectricPropulsionConfig::new_with_esc(battery, esc, motor, propeller),
        coefficient_source,
    ))
}

fn resolve_propeller_table(
    samples: Vec<crate::v0::PropellerSampleFileV0>,
) -> Result<PropellerCoefficientTable, ModelLoadError> {
    PropellerCoefficientTable::new(
        samples
            .into_iter()
            .map(|sample| PropellerSample {
                advance_ratio_j: sample.advance_ratio_j,
                ct: sample.ct,
                cq: sample.cq,
            })
            .collect(),
    )
    .map_err(|source| ModelLoadError::InvalidPropellerCoefficientTable { source })
}

fn resolve_common(
    file: AircraftModelFileV0,
    schema_version: u32,
) -> Result<AircraftModel, ModelLoadError> {
    if !is_valid_stable_id(&file.model_id) {
        return Err(ModelLoadError::InvalidModelId {
            value: file.model_id,
        });
    }

    let inertia = file.rigid_body.inertia_body_kg_m2;
    let rigid_body = RigidBodyParams::new(
        file.rigid_body.mass_kg,
        Mat3::new(
            inertia[0][0],
            inertia[0][1],
            inertia[0][2],
            inertia[1][0],
            inertia[1][1],
            inertia[1][2],
            inertia[2][0],
            inertia[2][1],
            inertia[2][2],
        ),
    )
    .map_err(|source| ModelLoadError::InvalidRigidBody { source })?;

    let mut aero_polars = Vec::with_capacity(file.aerodynamics.polars.len());
    for (index, polar_file) in file.aerodynamics.polars.into_iter().enumerate() {
        validate_unique_id(
            "polar",
            index,
            &polar_file.id,
            aero_polars.iter().map(RuntimePolar::id),
        )?;
        let samples = polar_file
            .samples
            .into_iter()
            .map(|sample| PolarSample {
                alpha_rad: sample.alpha_rad,
                cl: sample.cl,
                cd: sample.cd,
                cm: sample.cm,
            })
            .collect();
        let table = PolarTable::new(samples).map_err(|source| ModelLoadError::InvalidPolar {
            id: polar_file.id.clone(),
            index,
            source,
        })?;
        aero_polars.push(RuntimePolar::new(polar_file.id, table));
    }

    let mut aero_elements = Vec::with_capacity(file.aerodynamics.elements.len());
    for (index, element_file) in file.aerodynamics.elements.into_iter().enumerate() {
        validate_unique_id(
            "aerodynamic element",
            index,
            &element_file.id,
            aero_elements.iter().map(RuntimeAeroElement::id),
        )?;
        let polar_index = aero_polars
            .iter()
            .position(|polar| polar.id() == element_file.polar_id)
            .ok_or_else(|| ModelLoadError::UnresolvedPolarReference {
                element_id: element_file.id.clone(),
                element_index: index,
                polar_id: element_file.polar_id.clone(),
            })?;
        let element = AeroElement::new(
            vector(element_file.position_body_m),
            orientation(element_file.orientation_body_from_element_wxyz),
            element_file.area_m2,
            element_file.chord_m,
        )
        .map_err(|source| ModelLoadError::InvalidAeroElement {
            id: element_file.id.clone(),
            index,
            source,
        })?;
        aero_elements.push(RuntimeAeroElement::new(
            element_file.id,
            element,
            polar_index,
        ));
    }

    let response_file = file.controls.response;
    let response = ControlResponseConfig::new(
        axis("response.roll", response_file.roll)?,
        axis("response.pitch", response_file.pitch)?,
        axis("response.yaw", response_file.yaw)?,
    );
    let servos_file = file.controls.servos;
    let actuators = ControlActuatorConfig::new(
        servo("servos.aileron", servos_file.aileron)?,
        servo("servos.elevator", servos_file.elevator)?,
        servo("servos.rudder", servos_file.rudder)?,
    );
    let controls = ControlSystemConfig::new(response, actuators);

    let propulsion = file
        .propulsion
        .map(|propulsion_file| {
            let battery = BatteryConfig::new(
                propulsion_file.battery.open_circuit_voltage_v,
                propulsion_file.battery.internal_resistance_ohm,
            )
            .map_err(|source| ModelLoadError::InvalidBattery { source })?;
            let motor = MotorConfig::new(
                propulsion_file.motor.kv_rpm_per_v,
                propulsion_file.motor.winding_resistance_ohm,
                propulsion_file.motor.no_load_current_a,
            )
            .map_err(|source| ModelLoadError::InvalidMotor { source })?;
            let spin_direction = match propulsion_file.propeller.spin_direction {
                PropellerSpinDirectionFileV0::PositiveAboutLocalX => {
                    PropellerSpinDirection::PositiveAboutLocalX
                }
                PropellerSpinDirectionFileV0::NegativeAboutLocalX => {
                    PropellerSpinDirection::NegativeAboutLocalX
                }
            };
            let propeller = PropellerConfig::new(
                vector(propulsion_file.propeller.position_body_m),
                orientation(propulsion_file.propeller.orientation_body_from_prop_wxyz),
                propulsion_file.propeller.diameter_m,
                spin_direction,
            )
            .map_err(|source| ModelLoadError::InvalidPropeller { source })?;
            let samples = propulsion_file
                .coefficient_table
                .samples
                .into_iter()
                .map(|sample| PropellerSample {
                    advance_ratio_j: sample.advance_ratio_j,
                    ct: sample.ct,
                    cq: sample.cq,
                })
                .collect();
            let coefficient_table = PropellerCoefficientTable::new(samples)
                .map_err(|source| ModelLoadError::InvalidPropellerCoefficientTable { source })?;
            Ok(RuntimeElectricPropulsion::new_legacy(
                ElectricPropulsionConfig::new(battery, motor, propeller),
                coefficient_table,
            ))
        })
        .transpose()?;

    let presentation = file
        .presentation
        .map(|presentation_file| {
            if !is_valid_relative_asset_path(&presentation_file.glb_path) {
                return Err(ModelLoadError::InvalidPresentationAssetPath {
                    path: presentation_file.glb_path,
                });
            }
            Ok(PresentationMetadata::new(presentation_file.glb_path))
        })
        .transpose()?;

    Ok(AircraftModel::new(
        schema_version,
        file.model_id,
        file.display_name,
        rigid_body,
        aero_polars,
        aero_elements,
        controls,
        Vec::new(),
        propulsion,
        presentation,
    ))
}

fn resolve_control_surface_bindings(
    model: &AircraftModel,
    file_bindings: Vec<ControlSurfaceBindingFileV1>,
) -> Result<Vec<RuntimeControlSurfaceBinding>, ModelLoadError> {
    let mut bindings = Vec::with_capacity(file_bindings.len());
    for (binding_index, file) in file_bindings.into_iter().enumerate() {
        validate_unique_id(
            "control-surface binding",
            binding_index,
            &file.id,
            bindings.iter().map(RuntimeControlSurfaceBinding::id),
        )?;
        if !file.deflection_gain.is_finite() || file.deflection_gain == 0.0 {
            return Err(ModelLoadError::InvalidControlSurfaceDeflectionGain {
                binding_id: file.id,
                binding_index,
                value: file.deflection_gain,
            });
        }
        let element_index = model
            .aero_elements()
            .iter()
            .position(|element| element.id() == file.element_id)
            .ok_or_else(
                || ModelLoadError::UnresolvedControlSurfaceElementReference {
                    binding_id: file.id.clone(),
                    binding_index,
                    element_id: file.element_id.clone(),
                },
            )?;
        if let Some(first_index) = bindings
            .iter()
            .position(|binding| binding.element_index() == element_index)
        {
            return Err(ModelLoadError::DuplicateControlledAeroElement {
                element_id: file.element_id,
                first_binding_id: bindings[first_index].id().to_owned(),
                first_index,
                binding_id: file.id,
                duplicate_index: binding_index,
            });
        }
        let actuator = match file.actuator {
            ControlActuatorFileV1::Aileron => ControlActuator::Aileron,
            ControlActuatorFileV1::Elevator => ControlActuator::Elevator,
            ControlActuatorFileV1::Rudder => ControlActuator::Rudder,
        };
        bindings.push(RuntimeControlSurfaceBinding::new(
            file.id,
            element_index,
            actuator,
            file.deflection_gain,
        ));
    }
    Ok(bindings)
}

fn resolve_reference_aircraft(
    model: &AircraftModel,
    file: ReferenceAircraftFileV2,
) -> Result<ReferenceAircraftMetadata, ModelLoadError> {
    let identity_file = file.identity;
    let manufacturer =
        validate_optional_reference_text("identity.manufacturer", identity_file.manufacturer)?;
    let aircraft_name =
        validate_optional_reference_text("identity.aircraft_name", identity_file.aircraft_name)?;
    let variant = validate_optional_reference_text("identity.variant", identity_file.variant)?;
    let stable_reference_id = identity_file
        .stable_reference_id
        .map(|value| {
            if is_valid_stable_id(&value) {
                Ok(value)
            } else {
                Err(ModelLoadError::InvalidReferenceId { value })
            }
        })
        .transpose()?;
    let notes = validate_optional_reference_text("identity.notes", identity_file.notes)?;
    let identity = ReferenceAircraftIdentity {
        manufacturer,
        aircraft_name,
        variant,
        stable_reference_id,
        notes,
    };

    let mut provenance_sources = Vec::with_capacity(file.provenance_sources.len());
    for (index, source) in file.provenance_sources.into_iter().enumerate() {
        validate_unique_id(
            "provenance source",
            index,
            &source.id,
            provenance_sources.iter().map(ProvenanceSource::id),
        )?;
        provenance_sources.push(resolve_provenance_source(index, source)?);
    }

    let physical_specification = resolve_reference_physical_specification(
        model,
        file.physical_specification,
        &provenance_sources,
    )?;
    Ok(ReferenceAircraftMetadata {
        identity,
        physical_specification,
        provenance_sources,
    })
}

fn resolve_provenance_source(
    index: usize,
    file: ProvenanceSourceFileV2,
) -> Result<ProvenanceSource, ModelLoadError> {
    let prefix = format!("provenance_sources[{index}]");
    Ok(ProvenanceSource {
        id: file.id,
        source_type: match file.source_type {
            ProvenanceSourceTypeFileV2::ManufacturerDocumentation => {
                ProvenanceSourceType::ManufacturerDocumentation
            }
            ProvenanceSourceTypeFileV2::Measured => ProvenanceSourceType::Measured,
            ProvenanceSourceTypeFileV2::PublishedResearch => {
                ProvenanceSourceType::PublishedResearch
            }
            ProvenanceSourceTypeFileV2::AirfoilDatabase => ProvenanceSourceType::AirfoilDatabase,
            ProvenanceSourceTypeFileV2::NumericalAnalysis => {
                ProvenanceSourceType::NumericalAnalysis
            }
            ProvenanceSourceTypeFileV2::Derived => ProvenanceSourceType::Derived,
            ProvenanceSourceTypeFileV2::Estimated => ProvenanceSourceType::Estimated,
        },
        title: validate_optional_reference_text(&format!("{prefix}.title"), file.title)?,
        url: validate_optional_reference_text(&format!("{prefix}.url"), file.url)?,
        bibliographic_reference: validate_optional_reference_text(
            &format!("{prefix}.bibliographic_reference"),
            file.bibliographic_reference,
        )?,
        notes: validate_optional_reference_text(&format!("{prefix}.notes"), file.notes)?,
        publication_date: validate_optional_reference_text(
            &format!("{prefix}.publication_date"),
            file.publication_date,
        )?,
        retrieval_date: validate_optional_reference_text(
            &format!("{prefix}.retrieval_date"),
            file.retrieval_date,
        )?,
        confidence: file.confidence.map(|confidence| match confidence {
            ProvenanceConfidenceFileV2::Low => ProvenanceConfidence::Low,
            ProvenanceConfidenceFileV2::Medium => ProvenanceConfidence::Medium,
            ProvenanceConfidenceFileV2::High => ProvenanceConfidence::High,
        }),
    })
}

fn resolve_reference_physical_specification(
    model: &AircraftModel,
    file: ReferencePhysicalSpecificationFileV2,
    sources: &[ProvenanceSource],
) -> Result<ReferencePhysicalSpecification, ModelLoadError> {
    let positive = |field, value| resolve_reference_scalar(field, value, sources, true);
    let finite = |field, value| resolve_reference_scalar(field, value, sources, false);

    let mass = file
        .mass
        .map(|evidence| {
            resolve_reference_evidence("physical_specification.mass", evidence, sources)
        })
        .transpose()?;
    let cg_location = file
        .cg_location
        .map(|cg| resolve_reference_cg(cg, sources))
        .transpose()?;

    let mut control_surface_travel_limits =
        Vec::with_capacity(file.control_surface_travel_limits.len());
    for (index, travel) in file.control_surface_travel_limits.into_iter().enumerate() {
        let binding_index = model
            .control_surface_bindings()
            .iter()
            .position(|binding| binding.id() == travel.control_surface_binding_id)
            .ok_or_else(
                || ModelLoadError::UnresolvedReferenceControlSurfaceBinding {
                    index,
                    binding_id: travel.control_surface_binding_id.clone(),
                },
            )?;
        if let Some(first_index) = control_surface_travel_limits.iter().position(
            |existing: &ReferenceControlSurfaceTravel| existing.binding_index == binding_index,
        ) {
            return Err(ModelLoadError::DuplicateReferenceControlSurfaceBinding {
                binding_id: travel.control_surface_binding_id,
                first_index,
                duplicate_index: index,
            });
        }
        let evidence = resolve_reference_evidence(
            &format!("physical_specification.control_surface_travel_limits[{index}]"),
            ReferenceParameterEvidenceFileV2 {
                status: travel.status,
                source_ids: travel.source_ids,
            },
            sources,
        )?;
        control_surface_travel_limits.push(ReferenceControlSurfaceTravel {
            binding_index,
            evidence,
        });
    }

    Ok(ReferencePhysicalSpecification {
        wingspan_m: positive("physical_specification.wingspan_m", file.wingspan_m)?,
        reference_wing_area_m2: positive(
            "physical_specification.reference_wing_area_m2",
            file.reference_wing_area_m2,
        )?,
        aircraft_length_m: positive(
            "physical_specification.aircraft_length_m",
            file.aircraft_length_m,
        )?,
        mass,
        cg_location,
        aerodynamic_reference_chord_m: positive(
            "physical_specification.aerodynamic_reference_chord_m",
            file.aerodynamic_reference_chord_m,
        )?,
        wing_incidence_rad: finite(
            "physical_specification.wing_incidence_rad",
            file.wing_incidence_rad,
        )?,
        horizontal_tail_incidence_rad: finite(
            "physical_specification.horizontal_tail_incidence_rad",
            file.horizontal_tail_incidence_rad,
        )?,
        wing_dihedral_rad: finite(
            "physical_specification.wing_dihedral_rad",
            file.wing_dihedral_rad,
        )?,
        control_surface_travel_limits,
    })
}

fn resolve_reference_scalar(
    field: &'static str,
    file: Option<ReferenceScalarFileV2>,
    sources: &[ProvenanceSource],
    must_be_positive: bool,
) -> Result<Option<ReferenceScalar>, ModelLoadError> {
    file.map(|file| {
        if !file.value.is_finite() || (must_be_positive && file.value <= 0.0) {
            return Err(ModelLoadError::InvalidReferencePhysicalValue {
                field,
                value: file.value,
                requirement: if must_be_positive {
                    "expected a finite value greater than zero"
                } else {
                    "expected a finite value"
                },
            });
        }
        let evidence = resolve_reference_evidence(
            field,
            ReferenceParameterEvidenceFileV2 {
                status: file.status,
                source_ids: file.source_ids,
            },
            sources,
        )?;
        Ok(ReferenceScalar {
            value: file.value,
            evidence,
        })
    })
    .transpose()
}

fn resolve_reference_cg(
    file: ReferenceCgLocationFileV2,
    sources: &[ProvenanceSource],
) -> Result<ReferenceCgLocation, ModelLoadError> {
    if !file
        .position_m_from_reference
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(ModelLoadError::InvalidReferenceCgPosition);
    }
    let reference_kind = match file.reference.kind {
        CgReferenceKindFileV2::BodyFrameOriginFrd => CgReferenceKind::BodyFrameOriginFrd,
        CgReferenceKindFileV2::WingRootLeadingEdge => CgReferenceKind::WingRootLeadingEdge,
        CgReferenceKindFileV2::MeanAerodynamicChordLeadingEdge => {
            CgReferenceKind::MeanAerodynamicChordLeadingEdge
        }
        CgReferenceKindFileV2::ManufacturerDatum => CgReferenceKind::ManufacturerDatum,
        CgReferenceKindFileV2::Other => CgReferenceKind::Other,
    };
    let description = validate_optional_reference_text(
        "physical_specification.cg_location.reference.description",
        file.reference.description,
    )?;
    if matches!(
        file.reference.kind,
        CgReferenceKindFileV2::ManufacturerDatum | CgReferenceKindFileV2::Other
    ) && description.is_none()
    {
        return Err(ModelLoadError::InvalidReferenceCgDefinition {
            kind: file.reference.kind,
        });
    }
    let evidence = resolve_reference_evidence(
        "physical_specification.cg_location",
        ReferenceParameterEvidenceFileV2 {
            status: file.status,
            source_ids: file.source_ids,
        },
        sources,
    )?;
    Ok(ReferenceCgLocation {
        position_m_from_reference: file.position_m_from_reference,
        reference_kind,
        reference_description: description,
        evidence,
    })
}

fn resolve_reference_evidence(
    parameter: &str,
    file: ReferenceParameterEvidenceFileV2,
    sources: &[ProvenanceSource],
) -> Result<ReferenceParameterEvidence, ModelLoadError> {
    let mut source_indices = Vec::with_capacity(file.source_ids.len());
    for (index, source_id) in file.source_ids.into_iter().enumerate() {
        if !is_valid_stable_id(&source_id) {
            return Err(ModelLoadError::InvalidStableId {
                kind: "provenance source reference",
                index,
                value: source_id,
            });
        }
        let source_index = sources
            .iter()
            .position(|source| source.id() == source_id)
            .ok_or_else(|| ModelLoadError::UnresolvedProvenanceReference {
                parameter: parameter.to_owned(),
                source_id: source_id.clone(),
            })?;
        if source_indices.contains(&source_index) {
            return Err(ModelLoadError::DuplicateProvenanceReference {
                parameter: parameter.to_owned(),
                source_id,
            });
        }
        source_indices.push(source_index);
    }
    Ok(ReferenceParameterEvidence {
        quality: match file.status {
            ParameterQualityFileV2::Measured => ParameterQuality::Measured,
            ParameterQualityFileV2::ManufacturerSpec => ParameterQuality::ManufacturerSpec,
            ParameterQualityFileV2::Published => ParameterQuality::Published,
            ParameterQualityFileV2::Derived => ParameterQuality::Derived,
            ParameterQualityFileV2::Estimated => ParameterQuality::Estimated,
            ParameterQualityFileV2::Unknown => ParameterQuality::Unknown,
        },
        source_indices,
    })
}

fn validate_optional_reference_text(
    field: &str,
    value: Option<String>,
) -> Result<Option<String>, ModelLoadError> {
    if value.as_ref().is_some_and(|text| text.trim().is_empty()) {
        return Err(ModelLoadError::InvalidReferenceText {
            field: field.to_owned(),
        });
    }
    Ok(value)
}

fn axis(
    component: &'static str,
    file: AxisResponseFileV0,
) -> Result<AxisResponseConfig, ModelLoadError> {
    AxisResponseConfig::new(file.rate, file.expo)
        .map_err(|source| ModelLoadError::InvalidControls { component, source })
}

fn servo(component: &'static str, file: ServoFileV0) -> Result<ServoConfig, ModelLoadError> {
    ServoConfig::new(
        file.min_angle_rad,
        file.neutral_angle_rad,
        file.max_angle_rad,
        file.max_speed_rad_s,
        file.reversed,
    )
    .map_err(|source| ModelLoadError::InvalidControls { component, source })
}

fn validate_unique_id<'a>(
    kind: &'static str,
    index: usize,
    id: &str,
    mut previous_ids: impl Iterator<Item = &'a str>,
) -> Result<(), ModelLoadError> {
    if !is_valid_stable_id(id) {
        return Err(ModelLoadError::InvalidStableId {
            kind,
            index,
            value: id.to_owned(),
        });
    }
    if let Some(first_index) = previous_ids.position(|previous| previous == id) {
        return Err(ModelLoadError::DuplicateStableId {
            kind,
            id: id.to_owned(),
            first_index,
            duplicate_index: index,
        });
    }
    Ok(())
}

fn is_valid_stable_id(id: &str) -> bool {
    !id.is_empty()
        && id.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'-' | b'_')
        })
}

fn is_valid_relative_asset_path(path: &str) -> bool {
    if path.trim().is_empty()
        || path.starts_with('/')
        || path.starts_with('\\')
        || path.as_bytes().get(1) == Some(&b':')
    {
        return false;
    }
    !path.split(['/', '\\']).any(|component| component == "..")
}

fn vector(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

fn orientation(values: [f64; 4]) -> Orientation {
    let [w, x, y, z] = values;
    Orientation::new_unchecked(Quaternion::new(w, x, y, z))
}
