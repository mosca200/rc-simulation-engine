#![allow(dead_code)]

use model::{AircraftModel, AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};

pub fn valid_model_value() -> Value {
    json!({
        "schema_version": 0,
        "model_id": "test-aircraft_01",
        "display_name": "Test Aircraft",
        "rigid_body": {
            "mass_kg": 2.5,
            "inertia_body_kg_m2": [
                [0.12, 0.01, -0.002],
                [0.01, 0.15, 0.003],
                [-0.002, 0.003, 0.20]
            ]
        },
        "aerodynamics": {
            "polars": [
                {
                    "id": "polar-first",
                    "samples": [
                        { "alpha_rad": -0.20, "cl": -0.70, "cd": 0.080, "cm": 0.030 },
                        { "alpha_rad":  0.00, "cl":  0.10, "cd": 0.020, "cm": 0.010 },
                        { "alpha_rad":  0.25, "cl":  1.00, "cd": 0.110, "cm": -0.040 }
                    ]
                },
                {
                    "id": "polar-second",
                    "samples": [
                        { "alpha_rad": -0.15, "cl": -0.45, "cd": 0.060, "cm": 0.020 },
                        { "alpha_rad":  0.18, "cl":  0.75, "cd": 0.070, "cm": -0.025 }
                    ]
                }
            ],
            "elements": [
                {
                    "id": "element-first",
                    "position_body_m": [0.35, -0.42, 0.08],
                    "orientation_body_from_element_wxyz": [0.5, 0.5, 0.5, 0.5],
                    "area_m2": 0.31,
                    "chord_m": 0.19,
                    "polar_id": "polar-second"
                },
                {
                    "id": "element-second",
                    "position_body_m": [-0.22, 0.37, -0.04],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.27,
                    "chord_m": 0.16,
                    "polar_id": "polar-first"
                }
            ]
        },
        "controls": {
            "response": {
                "roll":  { "rate": 0.80, "expo": 0.10 },
                "pitch": { "rate": 0.70, "expo": 0.20 },
                "yaw":   { "rate": 0.60, "expo": 0.30 }
            },
            "servos": {
                "aileron": {
                    "min_angle_rad": -0.40,
                    "neutral_angle_rad": 0.01,
                    "max_angle_rad": 0.50,
                    "max_speed_rad_s": 4.0,
                    "reversed": false
                },
                "elevator": {
                    "min_angle_rad": -0.30,
                    "neutral_angle_rad": -0.02,
                    "max_angle_rad": 0.45,
                    "max_speed_rad_s": 3.5,
                    "reversed": true
                },
                "rudder": {
                    "min_angle_rad": -0.50,
                    "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.55,
                    "max_speed_rad_s": 2.5,
                    "reversed": false
                }
            }
        },
        "propulsion": {
            "battery": {
                "open_circuit_voltage_v": 14.8,
                "internal_resistance_ohm": 0.025
            },
            "motor": {
                "kv_rpm_per_v": 920.0,
                "winding_resistance_ohm": 0.041,
                "no_load_current_a": 1.2
            },
            "propeller": {
                "position_body_m": [0.30, 0.01, -0.02],
                "orientation_body_from_prop_wxyz": [0.0, 1.0, 0.0, 0.0],
                "diameter_m": 0.33,
                "spin_direction": "negative_about_local_x"
            },
            "coefficient_table": {
                "samples": [
                    { "advance_ratio_j": 0.0, "ct": 0.12, "cq": 0.018 },
                    { "advance_ratio_j": 0.5, "ct": 0.08, "cq": 0.012 },
                    { "advance_ratio_j": 1.0, "ct": 0.02, "cq": 0.005 }
                ]
            }
        },
        "presentation": {
            "glb_path": "assets/test-aircraft.glb"
        }
    })
}

pub fn minimal_model_value() -> Value {
    let mut value = valid_model_value();
    value["aerodynamics"]["polars"]
        .as_array_mut()
        .expect("polars array")
        .truncate(1);
    value["aerodynamics"]["elements"]
        .as_array_mut()
        .expect("elements array")
        .truncate(1);
    value["aerodynamics"]["elements"][0]["polar_id"] = json!("polar-first");
    value["propulsion"] = Value::Null;
    value["presentation"] = Value::Null;
    value
}

pub fn valid_v1_model_value() -> Value {
    let mut value = valid_model_value();
    value["schema_version"] = json!(1);
    value.as_object_mut().expect("model root object").insert(
        "control_surface_bindings".to_owned(),
        json!([
            {
                "id": "aileron-first",
                "element_id": "element-first",
                "actuator": "aileron",
                "deflection_gain": -1.0
            },
            {
                "id": "elevator-second",
                "element_id": "element-second",
                "actuator": "elevator",
                "deflection_gain": 0.75
            }
        ]),
    );
    value
}

pub fn valid_v2_reference_model_value() -> Value {
    let mut value = valid_v1_model_value();
    value["schema_version"] = json!(2);
    value
        .as_object_mut()
        .expect("model root object")
        .insert("classification".to_owned(), json!("reference_aircraft"));
    value.as_object_mut().expect("model root object").insert(
        "reference_aircraft".to_owned(),
        json!({
            "identity": {
                "manufacturer": "Fixture Manufacturer",
                "aircraft_name": "Reference Fixture",
                "variant": "test-only",
                "stable_reference_id": "reference-fixture-01",
                "notes": "Unit-test data, not a real aircraft"
            },
            "physical_specification": {
                "wingspan_m": {
                    "value": 1.8,
                    "status": "manufacturer_spec",
                    "source_ids": ["manufacturer-sheet"]
                },
                "reference_wing_area_m2": {
                    "value": 0.52,
                    "status": "derived",
                    "source_ids": ["calculation-note"]
                },
                "aircraft_length_m": null,
                "mass": {
                    "status": "measured",
                    "source_ids": ["scale-measurement"]
                },
                "cg_location": {
                    "position_m_from_reference": [0.12, 0.0, 0.0],
                    "reference": {
                        "kind": "wing_root_leading_edge",
                        "description": null
                    },
                    "status": "measured",
                    "source_ids": ["scale-measurement"]
                },
                "aerodynamic_reference_chord_m": null,
                "wing_incidence_rad": {
                    "value": 0.02,
                    "status": "published",
                    "source_ids": ["manufacturer-sheet"]
                },
                "horizontal_tail_incidence_rad": null,
                "wing_dihedral_rad": {
                    "value": 0.06,
                    "status": "estimated",
                    "source_ids": []
                },
                "control_surface_travel_limits": [
                    {
                        "control_surface_binding_id": "aileron-first",
                        "status": "manufacturer_spec",
                        "source_ids": ["manufacturer-sheet"]
                    }
                ]
            },
            "provenance_sources": [
                {
                    "id": "manufacturer-sheet",
                    "source_type": "manufacturer_documentation",
                    "title": "Fixture specification sheet",
                    "url": "https://example.invalid/fixture",
                    "bibliographic_reference": null,
                    "notes": null,
                    "publication_date": "2024-01-02",
                    "retrieval_date": "2026-01-03",
                    "confidence": "high"
                },
                {
                    "id": "scale-measurement",
                    "source_type": "measured",
                    "title": "Test measurement",
                    "url": null,
                    "bibliographic_reference": null,
                    "notes": "Test-only measurement",
                    "publication_date": null,
                    "retrieval_date": null,
                    "confidence": "medium"
                },
                {
                    "id": "calculation-note",
                    "source_type": "derived",
                    "title": null,
                    "url": null,
                    "bibliographic_reference": "Fixture calculation A",
                    "notes": null,
                    "publication_date": null,
                    "retrieval_date": null,
                    "confidence": "low"
                }
            ]
        }),
    );
    value
}

pub fn load_value(value: &Value) -> Result<AircraftModel, ModelLoadError> {
    let json = serde_json::to_string(value).expect("test model serializes");
    AircraftModelLoader::from_json_str(&json)
}

pub fn set(value: &mut Value, pointer: &str, replacement: Value) {
    *value
        .pointer_mut(pointer)
        .unwrap_or_else(|| panic!("missing JSON pointer {pointer}")) = replacement;
}

pub fn add_f64(value: &mut Value, pointer: &str, delta: f64) {
    let current = value
        .pointer(pointer)
        .and_then(Value::as_f64)
        .unwrap_or_else(|| panic!("missing f64 JSON pointer {pointer}"));
    set(value, pointer, json!(current + delta));
}
