mod common;

use common::{add_f64, load_value, set, valid_model_value};
use model::{AircraftModelFingerprint, AircraftModelLoader};
use serde_json::{Value, json};

fn fingerprint(value: &Value) -> AircraftModelFingerprint {
    load_value(value)
        .expect("fingerprint test mutation must remain a valid model")
        .physics_fingerprint()
}

#[test]
fn v0_fingerprint_remains_byte_identical_to_s6_baseline() {
    assert_eq!(
        fingerprint(&valid_model_value()).as_bytes(),
        &[
            0x07, 0x3c, 0x7e, 0x94, 0x77, 0x25, 0x56, 0x1e, 0xea, 0xbb, 0xf6, 0x0b, 0xe6, 0x8f,
            0x5c, 0xd5, 0x17, 0x1f, 0x8f, 0xf2, 0x7d, 0x20, 0x99, 0x25, 0x3c, 0xdf, 0xed, 0xa7,
            0x3c, 0x40, 0x31, 0xc2,
        ]
    );
}

fn assert_valid_mutation_changes_fingerprint(pointer: &str, delta: f64) {
    let baseline = valid_model_value();
    let mut changed = baseline.clone();
    add_f64(&mut changed, pointer, delta);

    assert_ne!(fingerprint(&baseline), fingerprint(&changed));
}

#[test]
fn m1_json_formatting_does_not_change_fingerprint() {
    let value = valid_model_value();
    let compact = serde_json::to_string(&value).expect("compact JSON");
    let pretty = serde_json::to_string_pretty(&value).expect("pretty JSON");

    assert_ne!(compact, pretty);
    let compact_model = AircraftModelLoader::from_json_str(&compact).expect("compact model");
    let pretty_model = AircraftModelLoader::from_json_str(&pretty).expect("pretty model");
    assert_eq!(
        compact_model.physics_fingerprint(),
        pretty_model.physics_fingerprint()
    );
}

#[test]
fn m2_mass_change_changes_fingerprint() {
    assert_valid_mutation_changes_fingerprint("/rigid_body/mass_kg", 0.125);
}

#[test]
fn m3_inertia_change_changes_fingerprint() {
    assert_valid_mutation_changes_fingerprint("/rigid_body/inertia_body_kg_m2/0/0", 0.001);
}

#[test]
fn m4_each_aerodynamic_polar_coefficient_changes_fingerprint() {
    for pointer in [
        "/aerodynamics/polars/0/samples/1/cl",
        "/aerodynamics/polars/0/samples/1/cd",
        "/aerodynamics/polars/0/samples/1/cm",
    ] {
        assert_valid_mutation_changes_fingerprint(pointer, 0.001);
    }
}

#[test]
fn m5_aero_element_position_change_changes_fingerprint() {
    assert_valid_mutation_changes_fingerprint("/aerodynamics/elements/0/position_body_m/1", 0.001);
}

#[test]
fn m6_servo_parameter_change_changes_fingerprint() {
    assert_valid_mutation_changes_fingerprint("/controls/servos/aileron/max_speed_rad_s", 0.125);
}

#[test]
fn m7_motor_and_propeller_changes_change_fingerprint() {
    assert_valid_mutation_changes_fingerprint("/propulsion/motor/kv_rpm_per_v", 1.0);
    assert_valid_mutation_changes_fingerprint("/propulsion/propeller/diameter_m", 0.001);

    let baseline = valid_model_value();
    let mut changed_spin = baseline.clone();
    set(
        &mut changed_spin,
        "/propulsion/propeller/spin_direction",
        json!("positive_about_local_x"),
    );
    assert_ne!(fingerprint(&baseline), fingerprint(&changed_spin));
}

#[test]
fn m8_display_name_change_does_not_change_physics_fingerprint() {
    let baseline = valid_model_value();
    let mut renamed = baseline.clone();
    renamed["display_name"] = json!("A Completely Different Human Name");

    assert_eq!(fingerprint(&baseline), fingerprint(&renamed));
}

#[test]
fn m9_glb_path_change_does_not_change_physics_fingerprint() {
    let baseline = valid_model_value();
    let mut changed = baseline.clone();
    changed["presentation"]["glb_path"] = json!("different/visual-only.glb");

    assert_eq!(fingerprint(&baseline), fingerprint(&changed));
}

#[test]
fn presentation_presence_does_not_change_physics_fingerprint() {
    let baseline = valid_model_value();
    let mut without_presentation = baseline.clone();
    without_presentation["presentation"] = Value::Null;

    assert_eq!(fingerprint(&baseline), fingerprint(&without_presentation));
}

#[test]
fn remaining_declared_physics_fields_are_fingerprinted() {
    for (pointer, delta) in [
        ("/aerodynamics/polars/0/samples/1/alpha_rad", 0.001),
        ("/aerodynamics/elements/0/area_m2", 0.001),
        ("/aerodynamics/elements/0/chord_m", 0.001),
        ("/controls/response/pitch/rate", 0.001),
        ("/controls/response/pitch/expo", 0.001),
        ("/controls/servos/rudder/min_angle_rad", -0.001),
        ("/propulsion/battery/open_circuit_voltage_v", 0.01),
        ("/propulsion/battery/internal_resistance_ohm", 0.001),
        ("/propulsion/motor/winding_resistance_ohm", 0.001),
        ("/propulsion/motor/no_load_current_a", 0.01),
        ("/propulsion/coefficient_table/samples/1/ct", 0.001),
        ("/propulsion/coefficient_table/samples/1/cq", 0.001),
    ] {
        assert_valid_mutation_changes_fingerprint(pointer, delta);
    }
}

#[test]
fn resolved_polar_relationship_is_fingerprinted() {
    let baseline = valid_model_value();
    let mut changed = baseline.clone();
    changed["aerodynamics"]["elements"][0]["polar_id"] = json!("polar-first");

    assert_ne!(fingerprint(&baseline), fingerprint(&changed));
}

#[test]
fn nonphysical_model_id_is_excluded_from_physics_fingerprint() {
    let baseline = valid_model_value();
    let mut renamed = baseline.clone();
    renamed["model_id"] = json!("new-stable-id");

    assert_eq!(fingerprint(&baseline), fingerprint(&renamed));
}
