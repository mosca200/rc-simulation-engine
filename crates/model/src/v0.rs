//! Serializable authoring representation for aircraft-model schema version 0.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AircraftModelFileV0 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV0,
    pub controls: ControlsFileV0,
    pub propulsion: Option<PropulsionFileV0>,
    pub presentation: Option<PresentationFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RigidBodyFileV0 {
    pub mass_kg: f64,
    pub inertia_body_kg_m2: [[f64; 3]; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AerodynamicsFileV0 {
    pub polars: Vec<PolarFileV0>,
    pub elements: Vec<AeroElementFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolarFileV0 {
    pub id: String,
    pub samples: Vec<PolarSampleFileV0>,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolarSampleFileV0 {
    pub alpha_rad: f64,
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AeroElementFileV0 {
    pub id: String,
    pub position_body_m: [f64; 3],
    pub orientation_body_from_element_wxyz: [f64; 4],
    pub area_m2: f64,
    pub chord_m: f64,
    pub polar_id: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AxisResponseFileV0 {
    pub rate: f64,
    pub expo: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlResponseFileV0 {
    pub roll: AxisResponseFileV0,
    pub pitch: AxisResponseFileV0,
    pub yaw: AxisResponseFileV0,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServoFileV0 {
    pub min_angle_rad: f64,
    pub neutral_angle_rad: f64,
    pub max_angle_rad: f64,
    pub max_speed_rad_s: f64,
    pub reversed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServosFileV0 {
    pub aileron: ServoFileV0,
    pub elevator: ServoFileV0,
    pub rudder: ServoFileV0,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlsFileV0 {
    pub response: ControlResponseFileV0,
    pub servos: ServosFileV0,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BatteryFileV0 {
    pub open_circuit_voltage_v: f64,
    pub internal_resistance_ohm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorFileV0 {
    pub kv_rpm_per_v: f64,
    pub winding_resistance_ohm: f64,
    pub no_load_current_a: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PropellerSpinDirectionFileV0 {
    PositiveAboutLocalX,
    NegativeAboutLocalX,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerFileV0 {
    pub position_body_m: [f64; 3],
    pub orientation_body_from_prop_wxyz: [f64; 4],
    pub diameter_m: f64,
    pub spin_direction: PropellerSpinDirectionFileV0,
    /// Rotor polar moment of inertia about propeller local `+X`.
    ///
    /// Absent values preserve the pre-M2.8F zero-inertia behavior.
    #[serde(default)]
    pub propeller_rotational_inertia_kg_m2: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerSampleFileV0 {
    pub advance_ratio_j: f64,
    pub ct: f64,
    pub cq: f64,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropellerCoefficientTableFileV0 {
    pub samples: Vec<PropellerSampleFileV0>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropulsionFileV0 {
    pub battery: BatteryFileV0,
    pub motor: MotorFileV0,
    pub propeller: PropellerFileV0,
    pub coefficient_table: PropellerCoefficientTableFileV0,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PresentationFileV0 {
    pub glb_path: String,
}
