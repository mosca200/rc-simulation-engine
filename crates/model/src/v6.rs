//! Serializable authoring representation for aircraft-model schema version 6.
//!
//! Version 6 preserves v5 finite-wing surfaces and adds explicit, one-way
//! aerodynamic downwash interactions between resolved surfaces.

use crate::{
    v0::{ControlsFileV0, PresentationFileV0, RigidBodyFileV0},
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v5::{AerodynamicsFileV5, PropulsionFileV5},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV6 {
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
    pub propulsion: Option<PropulsionFileV5>,
    pub presentation: Option<PresentationFileV0>,
}

/// Authored one-way wake coupling from an upstream finite-wing surface to a
/// downstream finite-wing surface.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroDownwashInteractionFileV6 {
    pub id: String,
    pub source_surface_id: String,
    pub target_surface_id: String,
    pub downwash_factor: f64,
}
