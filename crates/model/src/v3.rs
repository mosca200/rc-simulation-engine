//! Serializable authoring representation for aircraft-model schema version 3.
//!
//! Version 3 preserves the v2 documentary and assembly fields while adding explicit
//! Reynolds-family aerodynamics and a physics-authoritative kinematic viscosity.

use crate::{
    v0::{
        ControlsFileV0, PolarFileV0, PolarSampleFileV0, PresentationFileV0, PropulsionFileV0,
        RigidBodyFileV0,
    },
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV3 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV3,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV0>,
    pub presentation: Option<PresentationFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AerodynamicsFileV3 {
    pub kinematic_viscosity_m2_s: f64,
    pub polars: Vec<PolarFileV0>,
    pub polar_families: Vec<ReynoldsPolarFamilyFileV3>,
    pub elements: Vec<AeroElementFileV3>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReynoldsPolarFamilyFileV3 {
    pub id: String,
    pub nodes: Vec<ReynoldsPolarNodeFileV3>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReynoldsPolarNodeFileV3 {
    pub reynolds_number: f64,
    pub samples: Vec<PolarSampleFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroElementFileV3 {
    pub id: String,
    pub position_body_m: [f64; 3],
    pub orientation_body_from_element_wxyz: [f64; 4],
    pub area_m2: f64,
    pub chord_m: f64,
    pub polar_binding: AeroPolarBindingFileV3,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum AeroPolarBindingFileV3 {
    Polar { polar_id: String },
    ReynoldsFamily { family_id: String },
}
