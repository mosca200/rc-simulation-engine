//! Schema v8 landing-gear migration and fingerprint tests.
use model::{AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};

fn v7_fixture_as_v8() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap();
    value["schema_version"] = json!(8);
    value
}

fn load(value: &Value) -> Result<model::AircraftModel, ModelLoadError> {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

fn valid_gear_contact() -> Value {
    json!({
        "id": "nose-gear",
        "position_body_m": [0.6, 0.0, 0.35],
        "wheel_radius_m": 0.05,
        "normal_stiffness_n_per_m": 8000.0,
        "normal_damping_n_s_per_m": 400.0,
        "longitudinal_friction_coefficient": 0.6,
        "lateral_friction_coefficient": 0.9,
        "rolling_resistance_coefficient": 0.02,
        "max_brake_friction_coefficient": 0.0,
        "steering": "rudder",
        "max_steer_angle_rad": 0.45,
        "steerable": true,
        "braked": false
    })
}

#[test]
fn v8_without_gear_loads_with_empty_runtime_gear() {
    let model = load(&v7_fixture_as_v8()).expect("v8 without gear must load");
    assert_eq!(model.schema_version(), 8);
    assert!(model.landing_gear().is_empty());
}

#[test]
fn v8_with_tricycle_gear_resolves_in_order() {
    let mut value = v7_fixture_as_v8();
    value["landing_gear"] = json!([
        valid_gear_contact(),
        {
            "id": "left-main",
            "position_body_m": [-0.25, -0.45, 0.35],
            "wheel_radius_m": 0.06,
            "normal_stiffness_n_per_m": 8000.0,
            "normal_damping_n_s_per_m": 400.0,
            "steerable": false,
            "braked": false
        }
    ]);
    let model = load(&value).expect("tricycle subset must load");
    assert_eq!(model.landing_gear().len(), 2);
    assert_eq!(model.landing_gear()[0].id(), "nose-gear");
    assert_eq!(model.landing_gear()[1].id(), "left-main");
    // Defaulted friction path still validates.
    assert_eq!(model.gear_contacts().len(), 2);
}

#[test]
fn gear_id_is_a_non_physical_label_in_the_fingerprint() {
    let mut baseline = v7_fixture_as_v8();
    baseline["landing_gear"] = json!([valid_gear_contact()]);
    let mut changed = baseline.clone();
    changed["landing_gear"][0]["id"] = json!("renamed-nose-gear");
    let baseline_fp = load(&baseline).unwrap().physics_fingerprint();
    let changed_fp = load(&changed).unwrap().physics_fingerprint();
    assert_eq!(baseline_fp, changed_fp);
}

#[test]
fn every_gear_physics_parameter_and_declaration_order_changes_fingerprint() {
    let mut contact = valid_gear_contact();
    contact["max_brake_friction_coefficient"] = json!(0.7);
    contact["braked"] = json!(true);

    let mut baseline = v7_fixture_as_v8();
    baseline["landing_gear"] = json!([contact]);
    let baseline_fp = load(&baseline).unwrap().physics_fingerprint();

    let mutations = [
        ("position_body_m", json!([0.7, 0.0, 0.35])),
        ("wheel_radius_m", json!(0.06)),
        ("normal_stiffness_n_per_m", json!(9000.0)),
        ("normal_damping_n_s_per_m", json!(450.0)),
        ("longitudinal_friction_coefficient", json!(0.5)),
        ("lateral_friction_coefficient", json!(0.8)),
        ("rolling_resistance_coefficient", json!(0.03)),
        ("max_brake_friction_coefficient", json!(0.6)),
        ("max_steer_angle_rad", json!(0.35)),
    ];
    for (field, replacement) in mutations {
        let mut changed = baseline.clone();
        changed["landing_gear"][0][field] = replacement;
        assert_ne!(
            baseline_fp,
            load(&changed).unwrap().physics_fingerprint(),
            "changing {field} must change the physics fingerprint"
        );
    }

    let mut fixed = baseline.clone();
    fixed["landing_gear"][0]["steering"] = json!("fixed");
    fixed["landing_gear"][0]["max_steer_angle_rad"] = json!(0.0);
    fixed["landing_gear"][0]["steerable"] = json!(false);
    assert_ne!(baseline_fp, load(&fixed).unwrap().physics_fingerprint());

    let mut unbraked = baseline.clone();
    unbraked["landing_gear"][0]["max_brake_friction_coefficient"] = json!(0.0);
    unbraked["landing_gear"][0]["braked"] = json!(false);
    assert_ne!(baseline_fp, load(&unbraked).unwrap().physics_fingerprint());

    let first = baseline["landing_gear"][0].clone();
    let mut second = first.clone();
    second["id"] = json!("main-gear");
    second["position_body_m"] = json!([-0.25, 0.0, 0.35]);
    let mut ordered = v7_fixture_as_v8();
    ordered["landing_gear"] = json!([first.clone(), second.clone()]);
    let mut reordered = v7_fixture_as_v8();
    reordered["landing_gear"] = json!([second, first]);
    assert_ne!(
        load(&ordered).unwrap().physics_fingerprint(),
        load(&reordered).unwrap().physics_fingerprint(),
        "gear declaration order is part of deterministic physics ordering"
    );
}

#[test]
fn no_gear_fabricates_no_wheels_and_keeps_v7_fingerprint_stream() {
    // A v8 model without gear must hash identically to its v7 ancestor except
    // for the schema-version domain (v7 loader path is untouched).
    let v8 = load(&v7_fixture_as_v8()).unwrap();
    assert!(v8.gear_contacts().is_empty());
}

#[test]
fn invalid_gear_is_rejected_at_load() {
    // Negative stiffness.
    let mut value = v7_fixture_as_v8();
    let mut bad = valid_gear_contact();
    bad["normal_stiffness_n_per_m"] = json!(-100.0);
    value["landing_gear"] = json!([bad]);
    assert!(matches!(
        load(&value),
        Err(ModelLoadError::InvalidLandingGearContact { .. })
    ));
    // Steering source without the steerable flag.
    let mut value = v7_fixture_as_v8();
    let mut bad = valid_gear_contact();
    bad["steerable"] = json!(false);
    value["landing_gear"] = json!([bad]);
    assert!(matches!(
        load(&value),
        Err(ModelLoadError::InvalidLandingGearContact { .. })
    ));
    // Unknown fields are never silently accepted.
    let mut value = v7_fixture_as_v8();
    let mut bad = valid_gear_contact();
    bad["suspension_travel_m"] = json!(0.1);
    value["landing_gear"] = json!([bad]);
    assert!(matches!(
        load(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
    // Duplicate gear IDs are rejected.
    let mut value = v7_fixture_as_v8();
    value["landing_gear"] = json!([valid_gear_contact(), valid_gear_contact()]);
    assert!(load(&value).is_err());
}

#[test]
fn v7_documents_reject_gear_fields() {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap();
    value["landing_gear"] = json!([valid_gear_contact()]);
    assert!(matches!(
        load(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}
