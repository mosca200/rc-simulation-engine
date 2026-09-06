//! OA2B regression coverage for the production synthetic aircraft.
use model::{AIRCRAFT_MODEL_SCHEMA_VERSION_V9, load_aircraft_model};
use serde_json::{Value, json};
use std::path::PathBuf;

// Recorded from the schema-v2 Acro Electric 01 airborne fields before OA2B.
const LEGACY_AIRBORNE_DATA_BLAKE3: &str =
    "43414f68b131b418da4a994eef3341cb76ab6c439cdb02f27d0327e2d8551214";

fn acro_model_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/acro_electric_01/model.json")
}

fn airborne_data_projection(value: &Value) -> Value {
    let elements = value["aerodynamics"]["elements"]
        .as_array()
        .expect("production model has aero elements")
        .iter()
        .map(|element| {
            let mut legacy = element.clone();
            let polar_id = legacy["polar_binding"]["polar_id"].clone();
            legacy
                .as_object_mut()
                .expect("aero element is an object")
                .remove("polar_binding");
            legacy["polar_id"] = polar_id;
            legacy
        })
        .collect::<Vec<_>>();
    json!({
        "rigid_body": value["rigid_body"].clone(),
        "aerodynamics": {
            "polars": value["aerodynamics"]["polars"].clone(),
            "elements": elements,
        },
        "controls": value["controls"].clone(),
        "control_surface_bindings": value["control_surface_bindings"].clone(),
        "propulsion": {
            "battery": value["propulsion"]["battery"].clone(),
            "motor": value["propulsion"]["motor"].clone(),
            "propeller": value["propulsion"]["propeller"].clone(),
            "coefficient_table": {
                "samples": value["propulsion"]["coefficient_source"]["samples"].clone(),
            },
        },
    })
}

#[test]
fn acro_electric_01_is_v9_with_symmetric_gear_and_airframe_contacts() {
    let path = acro_model_path();
    let model = load_aircraft_model(&path).expect("OA2B Acro Electric 01 must load");
    assert_eq!(model.schema_version(), AIRCRAFT_MODEL_SCHEMA_VERSION_V9);
    assert_eq!(model.model_id(), "acro-electric-01");
    assert_eq!(model.landing_gear().len(), 3);
    assert_eq!(model.airframe_contacts().len(), 3);
    let left_main = model
        .landing_gear()
        .iter()
        .find(|gear| gear.id() == "left-main")
        .unwrap();
    let right_main = model
        .landing_gear()
        .iter()
        .find(|gear| gear.id() == "right-main")
        .unwrap();
    assert_eq!(
        left_main.contact().position_body_m.x,
        right_main.contact().position_body_m.x
    );
    assert_eq!(
        left_main.contact().position_body_m.y,
        -right_main.contact().position_body_m.y
    );
    assert_eq!(
        left_main.contact().position_body_m.z,
        right_main.contact().position_body_m.z
    );
    assert_eq!(model.airframe_contacts()[0].id(), "belly");
    let left_tip = model
        .airframe_contacts()
        .iter()
        .find(|contact| contact.id() == "left-wing-tip")
        .unwrap()
        .contact();
    let right_tip = model
        .airframe_contacts()
        .iter()
        .find(|contact| contact.id() == "right-wing-tip")
        .unwrap()
        .contact();
    assert_eq!(left_tip.position_body_m.x, right_tip.position_body_m.x);
    assert_eq!(left_tip.position_body_m.y, -right_tip.position_body_m.y);
    assert_eq!(left_tip.position_body_m.z, right_tip.position_body_m.z);
    assert_eq!(left_tip.stiffness_n_per_m, right_tip.stiffness_n_per_m);
    assert_eq!(left_tip.damping_n_s_per_m, right_tip.damping_n_s_per_m);
    assert_eq!(left_tip.friction_mu, right_tip.friction_mu);
}

#[test]
fn migration_preserves_legacy_airborne_aero_control_and_propulsion_data() {
    let value: Value = serde_json::from_str(&std::fs::read_to_string(acro_model_path()).unwrap())
        .expect("production model JSON is valid");
    let canonical = serde_json::to_vec(&airborne_data_projection(&value)).unwrap();
    let digest = blake3::hash(&canonical).to_hex().to_string();
    assert_eq!(digest, LEGACY_AIRBORNE_DATA_BLAKE3);
}
