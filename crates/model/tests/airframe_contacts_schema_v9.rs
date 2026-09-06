use model::{AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};

fn fixture() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap();
    value["schema_version"] = json!(9);
    value
}

fn valid_contact() -> Value {
    json!({
        "id": "belly",
        "position_body_m": [-0.1, 0.0, 0.08],
        "normal_stiffness_n_per_m": 8000.0,
        "normal_damping_n_s_per_m": 350.0,
        "friction_coefficient": 0.45
    })
}

fn load(value: &Value) -> Result<model::AircraftModel, ModelLoadError> {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

#[test]
fn absent_contacts_are_empty_and_authored_contacts_preserve_order() {
    let empty = load(&fixture()).unwrap();
    assert!(empty.airframe_contacts().is_empty());

    let mut value = fixture();
    let first = valid_contact();
    let mut second = first.clone();
    second["id"] = json!("left-tip");
    second["position_body_m"] = json!([-0.05, -0.9, 0.01]);
    value["airframe_contacts"] = json!([first, second]);
    let model = load(&value).unwrap();
    assert_eq!(model.airframe_contacts()[0].id(), "belly");
    assert_eq!(model.airframe_contacts()[1].id(), "left-tip");
}

#[test]
fn structural_physics_and_order_are_fingerprinted_but_ids_are_not() {
    let mut baseline = fixture();
    let first = valid_contact();
    let mut second = first.clone();
    second["id"] = json!("tip");
    second["position_body_m"] = json!([0.0, 0.9, 0.01]);
    baseline["airframe_contacts"] = json!([first.clone(), second.clone()]);
    let baseline_fp = load(&baseline).unwrap().physics_fingerprint();

    let mut renamed = baseline.clone();
    renamed["airframe_contacts"][0]["id"] = json!("renamed");
    assert_eq!(baseline_fp, load(&renamed).unwrap().physics_fingerprint());

    for (field, replacement) in [
        ("position_body_m", json!([-0.2, 0.0, 0.08])),
        ("normal_stiffness_n_per_m", json!(9000.0)),
        ("normal_damping_n_s_per_m", json!(400.0)),
        ("friction_coefficient", json!(0.5)),
    ] {
        let mut changed = baseline.clone();
        changed["airframe_contacts"][0][field] = replacement;
        assert_ne!(baseline_fp, load(&changed).unwrap().physics_fingerprint());
    }

    let mut reordered = fixture();
    reordered["airframe_contacts"] = json!([second, first]);
    assert_ne!(baseline_fp, load(&reordered).unwrap().physics_fingerprint());
}

#[test]
fn invalid_duplicate_and_unknown_contacts_fail_closed() {
    let mut invalid = fixture();
    let mut bad = valid_contact();
    bad["normal_stiffness_n_per_m"] = json!(-1.0);
    invalid["airframe_contacts"] = json!([bad]);
    assert!(matches!(
        load(&invalid),
        Err(ModelLoadError::InvalidAirframeContact { .. })
    ));

    let mut duplicate = fixture();
    duplicate["airframe_contacts"] = json!([valid_contact(), valid_contact()]);
    assert!(load(&duplicate).is_err());

    let mut unknown = fixture();
    let mut contact = valid_contact();
    contact["radius_m"] = json!(0.02);
    unknown["airframe_contacts"] = json!([contact]);
    assert!(matches!(
        load(&unknown),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn v8_documents_reject_airframe_contact_fields() {
    let mut value = fixture();
    value["schema_version"] = json!(8);
    value["airframe_contacts"] = json!([valid_contact()]);
    assert!(matches!(
        load(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}
