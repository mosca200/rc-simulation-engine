//! Serializable authoring representation for aircraft-model schema version 9.
//!
//! Version 9 preserves v8 landing-gear semantics and adds ordered, optional
//! body-space structural contact points.

use crate::{
    v0::{ControlsFileV0, PresentationFileV0, RigidBodyFileV0},
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v5::{AerodynamicsFileV5, PropulsionFileV5},
    v6::AeroDownwashInteractionFileV6,
    v7::PropellerSlipstreamInteractionFileV7,
    v8::LandingGearContactFileV8,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV9 {
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
    #[serde(default)]
    pub airframe_contacts: Vec<AirframeContactFileV9>,
    pub presentation: Option<PresentationFileV0>,
}

/// One structural point in FRD body coordinates, SI units.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AirframeContactFileV9 {
    pub id: String,
    pub position_body_m: [f64; 3],
    pub normal_stiffness_n_per_m: f64,
    pub normal_damping_n_s_per_m: f64,
    pub friction_coefficient: f64,
}
