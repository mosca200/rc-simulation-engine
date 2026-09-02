//! Serializable authoring representation for aircraft-model schema version 5.
//!
//! Version 5 preserves v4 propulsion semantics and v3 Reynolds aerodynamics, and adds
//! an explicit aerodynamic-surface grouping for future finite-wing physics (M2.8B).
//!
//! Surfaces group existing aerodynamic elements. The surface representation is
//! generic — no semantic categories (wing/tail) are imposed. Surface area and aspect
//! ratio are derived from member elements and authored span; no duplicated authored
//! values are permitted.

use crate::{
    v0::{
        BatteryFileV0, ControlsFileV0, MotorFileV0, PolarFileV0, PresentationFileV0,
        PropellerFileV0, RigidBodyFileV0,
    },
    v1::ControlSurfaceBindingFileV1,
    v2::{AircraftClassificationFileV2, ReferenceAircraftFileV2},
    v3::{AeroElementFileV3, ReynoldsPolarFamilyFileV3},
    v4::{EscFileV4, PropellerCoefficientSourceFileV4},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV5 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV5,
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV5>,
    pub presentation: Option<PresentationFileV0>,
}

/// V5 aerodynamics preserves v3/v4 fields and adds surface grouping.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AerodynamicsFileV5 {
    pub kinematic_viscosity_m2_s: f64,
    pub polars: Vec<PolarFileV0>,
    pub polar_families: Vec<ReynoldsPolarFamilyFileV3>,
    pub elements: Vec<AeroElementFileV3>,
    pub surfaces: Vec<AeroSurfaceFileV5>,
}

/// A finite aerodynamic surface grouping existing aero elements.
///
/// Semantics:
/// - `id`: unique, non-empty stable identifier
/// - `element_ids`: member aero-element IDs; must resolve to existing elements
/// - `span_axis_body`: body-frame direction of the surface span (normalized at load)
/// - `span_m`: physical span (finite, > 0)
/// - `span_efficiency_factor`: finite-wing span-efficiency parameter (finite, > 0, no upper cap)
///
/// Surface area is derived as the sum of member element areas.
/// Aspect ratio is derived as `span_m^2 / surface_area_m2`.
///
/// Polar interpretation contract for M2.8B: member element polar bindings represent
/// LOCAL SECTION / quasi-2D aerodynamic data. No 3D finite-wing polar mode exists.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroSurfaceFileV5 {
    pub id: String,
    pub element_ids: Vec<String>,
    pub span_axis_body: [f64; 3],
    pub span_m: f64,
    pub span_efficiency_factor: f64,
}

/// V5 propulsion is identical to v4.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropulsionFileV5 {
    pub battery: BatteryFileV0,
    pub esc: EscFileV4,
    pub motor: MotorFileV0,
    pub propeller: PropellerFileV0,
    pub coefficient_source: PropellerCoefficientSourceFileV4,
}
