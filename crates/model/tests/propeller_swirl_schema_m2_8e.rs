//! M2.8E schema-v7 swirl extension. All inputs are synthetic.

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
fn absent_and_zero_swirl_preserve_m2_8d_schema_semantics() {
    let absent = load(&fixture()).unwrap();
    assert_eq!(
        absent.propeller_slipstream_interactions()[0].swirl_velocity_factor(),
        0.0
    );

    let mut explicit_zero = fixture();
    explicit_zero["propeller_slipstream_interactions"][0]["swirl_velocity_factor"] = json!(0.0);
    let explicit_zero = load(&explicit_zero).unwrap();
    assert_eq!(
        absent.physics_fingerprint(),
        explicit_zero.physics_fingerprint()
    );
}

#[test]
fn finite_non_negative_swirl_is_resolved_and_changes_the_fingerprint() {
    let baseline = load(&fixture()).unwrap();
    let mut value = fixture();
    value["propeller_slipstream_interactions"][0]["swirl_velocity_factor"] = json!(0.625);
    let swirled = load(&value).unwrap();
    let interaction = &swirled.propeller_slipstream_interactions()[0];
    assert_eq!(interaction.target_element_indices(), &[1]);
    assert_eq!(interaction.swirl_velocity_factor(), 0.625);
    assert_ne!(
        baseline.physics_fingerprint(),
        swirled.physics_fingerprint()
    );

    value["propeller_slipstream_interactions"][0]["swirl_velocity_factor"] = json!(1.25);
    assert_ne!(
        swirled.physics_fingerprint(),
        load(&value).unwrap().physics_fingerprint()
    );
}

#[test]
fn negative_and_non_finite_swirl_are_rejected() {
    let mut negative = fixture();
    negative["propeller_slipstream_interactions"][0]["swirl_velocity_factor"] = json!(-0.01);
    assert!(matches!(
        load(&negative),
        Err(ModelLoadError::InvalidSwirlVelocityFactor {
            interaction_index: 0,
            value,
            ..
        }) if value == -0.01
    ));

    let non_finite = serde_json::to_string(&fixture()).unwrap().replace(
        "\"slipstream_velocity_factor\":1.0",
        "\"slipstream_velocity_factor\":1.0,\"swirl_velocity_factor\":1e999",
    );
    assert!(AircraftModelLoader::from_json_str(&non_finite).is_err());
}
