//! Serializable authoring representation for aircraft-model schema version 1.
//!
//! Version 1 preserves every v0 field and adds an explicit, ordered mapping
//! from the three conventional servos to aerodynamic elements.

use crate::v0::{
    AerodynamicsFileV0, ControlsFileV0, PresentationFileV0, PropulsionFileV0, RigidBodyFileV0,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV1 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV0,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV0>,
    pub presentation: Option<PresentationFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlSurfaceBindingFileV1 {
    pub id: String,
    pub element_id: String,
    pub actuator: ControlActuatorFileV1,
    pub deflection_gain: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlActuatorFileV1 {
    Aileron,
    Elevator,
    Rudder,
}
