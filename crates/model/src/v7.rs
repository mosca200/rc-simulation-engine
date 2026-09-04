//! Serializable authoring representation for aircraft-model schema version 7.
//!
//! Version 7 preserves v6 finite-wing and downwash semantics and adds explicit,
//! one-way propeller-slipstream interactions targeting resolved aerodynamic elements,
//! including an optional M2.8E rotational-wake factor that defaults to zero.

use crate::{
    v0::{ControlsFileV0, PresentationFileV0, RigidBodyFileV0},
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v5::{AerodynamicsFileV5, PropulsionFileV5},
    v6::AeroDownwashInteractionFileV6,
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV7 {
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
    pub presentation: Option<PresentationFileV0>,
}

/// Explicit coupling from the aircraft's single propeller to selected aero elements.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerSlipstreamInteractionFileV7 {
    pub id: String,
    pub target_element_ids: Vec<String>,
    pub slipstream_velocity_factor: f64,
    /// Optional tangential wake speed as a multiple of actuator-disk induced velocity.
    ///
    /// The default preserves the original M2.8D schema-v7 axial-only behavior.
    #[serde(default)]
    pub swirl_velocity_factor: f64,
}
