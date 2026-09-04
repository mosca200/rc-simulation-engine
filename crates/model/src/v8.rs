//! Serializable authoring representation for aircraft-model schema version 8.
//!
//! Version 8 preserves v7 slipstream semantics exactly and adds an optional
//! ordered `landing_gear` array. Absent gear means "no gear configured": the
//! aircraft stays a pure airborne configuration and no invisible wheels exist.

use crate::{
    v0::{ControlsFileV0, PresentationFileV0, RigidBodyFileV0},
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v5::{AerodynamicsFileV5, PropulsionFileV5},
    v6::AeroDownwashInteractionFileV6,
    v7::PropellerSlipstreamInteractionFileV7,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV8 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV5,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub aero_downwash_interactions: Vec<AeroDownwashInteractionFileV6>,
    pub propeller_slipstream_interactions: Vec<PropellerSlipstreamInteractionFileV7>,
    pub propulsion: Option<PropulsionFileV5>,
    #[serde(default)]
    pub landing_gear: Vec<LandingGearContactFileV8>,
    pub presentation: Option<PresentationFileV0>,
}

/// One authored wheel/skid contact in FRD body coordinates, SI units.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LandingGearContactFileV8 {
    pub id: String,
    pub position_body_m: [f64; 3],
    #[serde(default)]
    pub wheel_radius_m: f64,
    pub normal_stiffness_n_per_m: f64,
    pub normal_damping_n_s_per_m: f64,
    #[serde(default = "default_longitudinal_mu")]
    pub longitudinal_friction_coefficient: f64,
    #[serde(default = "default_lateral_mu")]
    pub lateral_friction_coefficient: f64,
    #[serde(default = "default_rolling_mu")]
    pub rolling_resistance_coefficient: f64,
    #[serde(default)]
    pub max_brake_friction_coefficient: f64,
    #[serde(default)]
    pub steering: SteeringSourceFileV8,
    #[serde(default)]
    pub max_steer_angle_rad: f64,
    #[serde(default)]
    pub steerable: bool,
    #[serde(default)]
    pub braked: bool,
}

fn default_longitudinal_mu() -> f64 {
    0.8
}

fn default_lateral_mu() -> f64 {
    0.8
}

fn default_rolling_mu() -> f64 {
    0.02
}

/// Steering command source. Only rudder coupling exists in v8.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SteeringSourceFileV8 {
    #[default]
    Fixed,
    Rudder,
}
