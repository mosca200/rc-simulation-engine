//! Shared synthetic fixtures for deterministic ground-contact tests.
#![allow(dead_code)]
use aircraft::AircraftSimulationConfig;
use model::AircraftModelLoader;
use serde_json::{Value, json};
use sim_core::{AeroEnvironment, RigidBodyState};
use sim_math::{Orientation, Vec3};

pub const GROUND_TEST_MASS_KG: f64 = 10.0;
pub const GROUND_TEST_WEIGHT_N: f64 = GROUND_TEST_MASS_KG * 9.80665;
pub const GROUND_TEST_DT_S: f64 = 0.002;

/// Tricycle gear sized for a 10 kg airframe at 500 Hz RK4.
pub fn tricycle_gear() -> Value {
    json!([
        {
            "id": "nose-gear",
            "position_body_m": [0.6, 0.0, 0.35],
            "wheel_radius_m": 0.05,
            "normal_stiffness_n_per_m": 12000.0,
            "normal_damping_n_s_per_m": 800.0,
            "longitudinal_friction_coefficient": 0.6,
            "lateral_friction_coefficient": 0.9,
            "rolling_resistance_coefficient": 0.02,
            "max_brake_friction_coefficient": 0.0,
            "steering": "rudder",
            "max_steer_angle_rad": 0.45,
            "steerable": true,
            "braked": false
        },
        {
            "id": "left-main",
            "position_body_m": [-0.25, -0.45, 0.35],
            "wheel_radius_m": 0.06,
            "normal_stiffness_n_per_m": 12000.0,
            "normal_damping_n_s_per_m": 800.0,
            "longitudinal_friction_coefficient": 0.6,
            "lateral_friction_coefficient": 0.9,
            "rolling_resistance_coefficient": 0.02,
            "max_brake_friction_coefficient": 0.8,
            "steering": "fixed",
            "max_steer_angle_rad": 0.0,
            "steerable": false,
            "braked": true
        },
        {
            "id": "right-main",
            "position_body_m": [-0.25, 0.45, 0.35],
            "wheel_radius_m": 0.06,
            "normal_stiffness_n_per_m": 12000.0,
            "normal_damping_n_s_per_m": 800.0,
            "longitudinal_friction_coefficient": 0.6,
            "lateral_friction_coefficient": 0.9,
            "rolling_resistance_coefficient": 0.02,
            "max_brake_friction_coefficient": 0.8,
            "steering": "fixed",
            "max_steer_angle_rad": 0.0,
            "steerable": false,
            "braked": true
        }
    ])
}

/// Minimal v8 model: brick fuselage + one analytic wing + powertrain.
pub fn ground_test_model_value() -> Value {
    json!({
        "schema_version": 8,
        "model_id": "ground-test-synthetic",
        "display_name": "Ground Test Synthetic",
        "classification": "synthetic_test",
        "reference_aircraft": null,
        "rigid_body": {
            "mass_kg": 10.0,
            "inertia_body_kg_m2": [
                [1.2, 0.0, 0.0],
                [0.0, 1.6, 0.0],
                [0.0, 0.0, 2.2]
            ]
        },
        "aerodynamics": {
            "kinematic_viscosity_m2_s": 0.000015,
            "polars": [
                {
                    "id": "flat-plate",
                    "samples": [
                        { "alpha_rad": -0.30, "cl": -0.9, "cd": 0.20, "cm": 0.0 },
                        { "alpha_rad": 0.0, "cl": 0.35, "cd": 0.03, "cm": 0.0 },
                        { "alpha_rad": 0.20, "cl": 1.1, "cd": 0.10, "cm": 0.0 }
                    ]
                },
                {
                    "id": "symmetric-tail",
                    "samples": [
                        { "alpha_rad": -0.40, "cl": -1.2, "cd": 0.025, "cm": 0.0 },
                        { "alpha_rad": 0.0, "cl": 0.0, "cd": 0.015, "cm": 0.0 },
                        { "alpha_rad": 0.40, "cl": 1.2, "cd": 0.025, "cm": 0.0 }
                    ]
                }
            ],
            "polar_families": [],
            "elements": [
                {
                    "id": "main-wing",
                    "position_body_m": [0.1, 0.0, -0.1],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.8,
                    "chord_m": 0.3,
                    "polar_binding": { "kind": "polar", "polar_id": "flat-plate" }
                },
                {
                    "id": "horizontal-tail-elevator",
                    "position_body_m": [-0.9, 0.0, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.2,
                    "chord_m": 0.2,
                    "polar_binding": { "kind": "polar", "polar_id": "symmetric-tail" }
                }
            ],
            "surfaces": []
        },
        "controls": {
            "response": {
                "roll": { "rate": 1.0, "expo": 0.0 },
                "pitch": { "rate": 1.0, "expo": 0.0 },
                "yaw": { "rate": 1.0, "expo": 0.0 }
            },
            "servos": {
                "aileron": {
                    "min_angle_rad": -0.5, "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5, "max_speed_rad_s": 6.0, "reversed": false
                },
                "elevator": {
                    "min_angle_rad": -0.5, "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5, "max_speed_rad_s": 6.0, "reversed": false
                },
                "rudder": {
                    "min_angle_rad": -0.5, "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5, "max_speed_rad_s": 6.0, "reversed": false
                }
            }
        },
        "control_surface_bindings": [
            {
                "id": "elevator-binding",
                "element_id": "horizontal-tail-elevator",
                "actuator": "elevator",
                "deflection_gain": -1.0
            }
        ],
        "aero_downwash_interactions": [],
        "propeller_slipstream_interactions": [],
        "propulsion": {
            "battery": { "open_circuit_voltage_v": 22.2, "internal_resistance_ohm": 0.02 },
            "esc": { "series_resistance_ohm": 0.005 },
            "motor": {
                "kv_rpm_per_v": 800.0,
                "winding_resistance_ohm": 0.03,
                "no_load_current_a": 1.0
            },
            "propeller": {
                "position_body_m": [0.7, 0.0, 0.0],
                "orientation_body_from_prop_wxyz": [0.0, 1.0, 0.0, 0.0],
                "diameter_m": 0.6,
                "spin_direction": "positive_about_local_x"
            },
            "coefficient_source": {
                "kind": "fixed_table",
                "samples": [
                    { "advance_ratio_j": 0.0, "ct": 0.16, "cq": 0.008 },
                    { "advance_ratio_j": 0.6, "ct": 0.12, "cq": 0.007 },
                    { "advance_ratio_j": 1.2, "ct": 0.05, "cq": 0.005 }
                ]
            }
        },
        "landing_gear": tricycle_gear(),
        "presentation": null
    })
}

pub fn load_ground_test_model() -> model::AircraftModel {
    let value = ground_test_model_value();
    AircraftModelLoader::from_json_str(&serde_json::to_string(&value).unwrap())
        .expect("synthetic ground-test model must load")
}

pub fn ground_test_config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        Vec3::new(0.0, 0.0, 9.80665),
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

/// Level rest attitude above the flat plane at `z = 0`.
pub fn parked_state(altitude_offset_m: f64) -> RigidBodyState {
    // Lowest wheel bottom is body z = 0.35 + 0.06 = 0.41 below CG,
    // so CG at z = -0.41 rests exactly on the plane.
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -0.41 - altitude_offset_m),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}
