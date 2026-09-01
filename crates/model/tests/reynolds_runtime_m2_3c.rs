mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V3, AircraftClassification, AircraftModel, AircraftModelLoader,
    ModelLoadError, RuntimeAeroPolarBinding,
};
use serde_json::{Value, json};

const SYNTHETIC_V3: &str =
    include_str!("../../../tests/fixtures/synthetic_non_reference_reynolds_v3.json");

fn value_v3() -> Value {
    serde_json::from_str(SYNTHETIC_V3).unwrap()
}

fn load_v3(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

#[test]
fn m2_3c_01_zero_viscosity_is_rejected() {
    let mut value = value_v3();
    value["aerodynamics"]["kinematic_viscosity_m2_s"] = json!(0.0);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&value).unwrap()),
        Err(ModelLoadError::InvalidKinematicViscosity { value: 0.0 })
    ));
}

#[test]
fn m2_3c_02_negative_viscosity_is_rejected() {
    let mut value = value_v3();
    value["aerodynamics"]["kinematic_viscosity_m2_s"] = json!(-0.0001);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&value).unwrap()),
        Err(ModelLoadError::InvalidKinematicViscosity { value }) if value == -0.0001
    ));
}

#[test]
fn m2_3c_16_legacy_v0_v1_v2_parse_without_reynolds_fields() {
    for value in [
        valid_model_value(),
        valid_v1_model_value(),
        valid_v2_reference_model_value(),
    ] {
        let model = load_value(&value).unwrap();
        assert!(model.kinematic_viscosity_m2_s().is_none());
        assert!(model.aero_polar_families().is_empty());
        assert!(model.aero_elements().iter().all(|element| matches!(
            element.polar_binding(),
            RuntimeAeroPolarBinding::Polar { .. }
        )));
    }
}

#[test]
fn m2_3c_17_legacy_fingerprints_remain_byte_identical() {
    let v0 = load_value(&valid_model_value()).unwrap();
    assert_eq!(
        v0.physics_fingerprint().as_bytes(),
        &[
            0x07, 0x3c, 0x7e, 0x94, 0x77, 0x25, 0x56, 0x1e, 0xea, 0xbb, 0xf6, 0x0b, 0xe6, 0x8f,
            0x5c, 0xd5, 0x17, 0x1f, 0x8f, 0xf2, 0x7d, 0x20, 0x99, 0x25, 0x3c, 0xdf, 0xed, 0xa7,
            0x3c, 0x40, 0x31, 0xc2,
        ]
    );
    let v1 = load_value(&valid_v1_model_value()).unwrap();
    let v2 = load_value(&valid_v2_reference_model_value()).unwrap();
    assert_eq!(v1.physics_fingerprint(), v2.physics_fingerprint());
}

#[test]
fn m2_3c_18_all_new_reynolds_physics_changes_v3_fingerprint() {
    let baseline = value_v3();
    let baseline_fingerprint = load_v3(&baseline).physics_fingerprint();
    for (pointer, replacement) in [
        ("/aerodynamics/kinematic_viscosity_m2_s", json!(0.00011)),
        (
            "/aerodynamics/polar_families/0/nodes/0/reynolds_number",
            json!(110000.0),
        ),
        (
            "/aerodynamics/polar_families/0/nodes/0/samples/0/alpha_rad",
            json!(-0.45),
        ),
        (
            "/aerodynamics/polar_families/0/nodes/0/samples/0/cl",
            json!(-0.55),
        ),
        (
            "/aerodynamics/polar_families/0/nodes/0/samples/0/cd",
            json!(0.025),
        ),
        (
            "/aerodynamics/polar_families/0/nodes/0/samples/0/cm",
            json!(0.045),
        ),
    ] {
        let mut changed = baseline.clone();
        *changed.pointer_mut(pointer).unwrap() = replacement;
        assert_ne!(
            baseline_fingerprint,
            load_v3(&changed).physics_fingerprint()
        );
    }

    let mut two_families = baseline.clone();
    let mut second = two_families["aerodynamics"]["polar_families"][0].clone();
    second["id"] = json!("synthetic-family-second");
    second["nodes"][0]["samples"][0]["cl"] = json!(-0.6);
    two_families["aerodynamics"]["polar_families"]
        .as_array_mut()
        .unwrap()
        .push(second);
    let first_binding = load_v3(&two_families).physics_fingerprint();
    two_families["aerodynamics"]["elements"][0]["polar_binding"]["family_id"] =
        json!("synthetic-family-second");
    assert_ne!(first_binding, load_v3(&two_families).physics_fingerprint());
}

#[test]
fn m2_3c_21_fixture_is_synthetic_and_introduces_no_reference_aircraft_runtime() {
    let model = load_v3(&value_v3());
    assert_eq!(model.schema_version(), AIRCRAFT_MODEL_SCHEMA_VERSION_V3);
    assert_eq!(
        model.classification(),
        AircraftClassification::SyntheticTest
    );
    assert!(model.reference_aircraft().is_none());
    assert!(!SYNTHETIC_V3.to_ascii_lowercase().contains("kadet"));
    assert!(!SYNTHETIC_V3.to_ascii_lowercase().contains("clark"));
}
