use crate::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V0, AIRCRAFT_MODEL_SCHEMA_VERSION_V1,
    runtime::{
        AircraftModel, ControlActuator, PresentationMetadata, RuntimeAeroElement,
        RuntimeControlSurfaceBinding, RuntimeElectricPropulsion, RuntimePolar,
    },
    v0::{AircraftModelFileV0, AxisResponseFileV0, PropellerSpinDirectionFileV0, ServoFileV0},
    v1::{AircraftModelFileV1, ControlActuatorFileV1, ControlSurfaceBindingFileV1},
};
use serde::Deserialize;
use sim_core::{
    AeroElement, AeroElementError, AxisResponseConfig, BatteryConfig, BatteryConfigError,
    ControlActuatorConfig, ControlConfigError, ControlResponseConfig, ControlSystemConfig,
    ElectricPropulsionConfig, MotorConfig, MotorConfigError, ParameterError, PolarError,
    PolarSample, PolarTable, PropellerCoefficientError, PropellerCoefficientTable, PropellerConfig,
    PropellerConfigError, PropellerSample, PropellerSpinDirection, RigidBodyParams, ServoConfig,
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
    #[error(
        "invalid presentation GLB path {path:?}: expected a nonempty relative path without '..'"
    )]
    InvalidPresentationAssetPath { path: String },
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
    let model = resolve_common(common_file, AIRCRAFT_MODEL_SCHEMA_VERSION_V1)?;
    let bindings = resolve_control_surface_bindings(&model, control_surface_bindings)?;
    Ok(model.with_control_surface_bindings(bindings))
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
            Ok(RuntimeElectricPropulsion::new(
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
