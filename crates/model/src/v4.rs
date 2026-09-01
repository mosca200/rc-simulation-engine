//! Serializable authoring representation for aircraft-model schema version 4.
//!
//! Version 4 preserves v3 Reynolds aerodynamics and makes the electric drivetrain and
//! propeller-coefficient source explicit.

use crate::{
    v0::{
        BatteryFileV0, ControlsFileV0, MotorFileV0, PresentationFileV0, PropellerFileV0,
        PropellerSampleFileV0, RigidBodyFileV0,
    },
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v3::AerodynamicsFileV3,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV4 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV3,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV4>,
    pub presentation: Option<PresentationFileV0>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EscFileV4 {
    pub series_resistance_ohm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropulsionFileV4 {
    pub battery: BatteryFileV0,
    pub esc: EscFileV4,
    pub motor: MotorFileV0,
    pub propeller: PropellerFileV0,
    pub coefficient_source: PropellerCoefficientSourceFileV4,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum PropellerCoefficientSourceFileV4 {
    FixedTable {
        samples: Vec<PropellerSampleFileV0>,
    },
    ShaftSpeedMap {
        nodes: Vec<PropellerCoefficientNodeFileV4>,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerCoefficientNodeFileV4 {
    pub shaft_speed_rad_s: f64,
    pub samples: Vec<PropellerSampleFileV0>,
}
