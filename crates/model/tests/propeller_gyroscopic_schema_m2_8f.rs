//! M2.8F propeller rotational-inertia schema/runtime behavior. Synthetic inputs only.

use model::{AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap()
}

fn load(value: &Value) -> Result<model::AircraftModel, ModelLoadError> {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

#[test]
fn absent_and_zero_inertia_preserve_the_existing_model_and_fingerprint() {
    let absent = load(&fixture()).unwrap();
    assert_eq!(
        absent
            .propulsion()
            .unwrap()
            .propeller_rotational_inertia_kg_m2(),
        0.0
    );

    let mut explicit_zero_value = fixture();
    explicit_zero_value["propulsion"]["propeller"]["propeller_rotational_inertia_kg_m2"] =
        json!(0.0);
    let explicit_zero = load(&explicit_zero_value).unwrap();
    assert_eq!(
        absent.physics_fingerprint(),
        explicit_zero.physics_fingerprint()
    );

    explicit_zero_value["propulsion"]["propeller"]["propeller_rotational_inertia_kg_m2"] =
        json!(0.0035);
    let nonzero = load(&explicit_zero_value).unwrap();
    assert_eq!(
        nonzero
            .propulsion()
            .unwrap()
            .propeller_rotational_inertia_kg_m2(),
        0.0035
    );
    assert_ne!(absent.physics_fingerprint(), nonzero.physics_fingerprint());
}

#[test]
fn negative_and_non_finite_inertia_are_rejected() {
    let mut negative = fixture();
    negative["propulsion"]["propeller"]["propeller_rotational_inertia_kg_m2"] = json!(-0.001);
    assert!(matches!(
        load(&negative),
        Err(ModelLoadError::InvalidPropellerRotationalInertia { value }) if value == -0.001
    ));

    let non_finite = serde_json::to_string(&fixture()).unwrap().replace(
        "\"diameter_m\":0.276",
        "\"diameter_m\":0.276,\"propeller_rotational_inertia_kg_m2\":1e999",
    );
    assert!(AircraftModelLoader::from_json_str(&non_finite).is_err());
}
