//! M2.8D schema-v7 and immutable runtime resolution. All values are synthetic.

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
fn valid_v7_resolves_ordered_target_indices_and_zero_factor_is_valid() {
    let model = load(&fixture()).unwrap();
    assert_eq!(model.schema_version(), 7);
    let interaction = &model.propeller_slipstream_interactions()[0];
    assert_eq!(interaction.id(), "propeller-to-tail");
    assert_eq!(interaction.target_element_indices(), &[1]);
    assert_eq!(interaction.slipstream_velocity_factor(), 1.0);

    let mut zero = fixture();
    zero["propeller_slipstream_interactions"][0]["slipstream_velocity_factor"] = json!(0.0);
    assert_eq!(
        load(&zero).unwrap().propeller_slipstream_interactions()[0].slipstream_velocity_factor(),
        0.0
    );
}

#[test]
fn interaction_ids_are_valid_and_unique_and_targets_are_nonempty() {
    let mut invalid = fixture();
    invalid["propeller_slipstream_interactions"][0]["id"] = json!("");
    assert!(matches!(
        load(&invalid),
        Err(ModelLoadError::InvalidStableId {
            kind: "propeller slipstream interaction",
            ..
        })
    ));

    let mut duplicate = fixture();
    let repeated = duplicate["propeller_slipstream_interactions"][0].clone();
    duplicate["propeller_slipstream_interactions"]
        .as_array_mut()
        .unwrap()
        .push(repeated);
    assert!(matches!(
        load(&duplicate),
        Err(ModelLoadError::DuplicateStableId {
            kind: "propeller slipstream interaction",
            ..
        })
    ));

    let mut empty = fixture();
    empty["propeller_slipstream_interactions"][0]["target_element_ids"] = json!([]);
    assert!(matches!(
        load(&empty),
        Err(ModelLoadError::EmptySlipstreamTargets { .. })
    ));
}

#[test]
fn unknown_duplicate_and_multiply_assigned_targets_are_rejected() {
    let mut unknown = fixture();
    unknown["propeller_slipstream_interactions"][0]["target_element_ids"] =
        json!(["missing-element"]);
    assert!(matches!(
        load(&unknown),
        Err(ModelLoadError::UnresolvedSlipstreamTargetElement { .. })
    ));

    let mut within = fixture();
    within["propeller_slipstream_interactions"][0]["target_element_ids"] =
        json!(["synthetic-tail", "synthetic-tail"]);
    assert!(matches!(
        load(&within),
        Err(ModelLoadError::DuplicateSlipstreamTargetWithinInteraction { .. })
    ));

    let mut across = fixture();
    across["propeller_slipstream_interactions"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "second-propeller-to-tail",
            "target_element_ids": ["synthetic-tail"],
            "slipstream_velocity_factor": 2.0
        }));
    assert!(matches!(
        load(&across),
        Err(ModelLoadError::DuplicateSlipstreamTarget { .. })
    ));
}

#[test]
fn propulsion_is_required_and_factor_must_be_finite_non_negative() {
    let mut absent = fixture();
    absent["propulsion"] = Value::Null;
    assert!(matches!(
        load(&absent),
        Err(ModelLoadError::SlipstreamInteractionWithoutPropulsion { .. })
    ));

    let mut negative = fixture();
    negative["propeller_slipstream_interactions"][0]["slipstream_velocity_factor"] = json!(-0.1);
    assert!(matches!(
        load(&negative),
        Err(ModelLoadError::InvalidSlipstreamVelocityFactor { .. })
    ));

    let nonfinite = serde_json::to_string(&fixture()).unwrap().replace(
        "\"slipstream_velocity_factor\":1.0",
        "\"slipstream_velocity_factor\":1e999",
    );
    assert!(AircraftModelLoader::from_json_str(&nonfinite).is_err());
}

#[test]
fn explicit_empty_list_is_valid_missing_list_is_structural_and_v6_still_loads() {
    let mut empty = fixture();
    empty["propeller_slipstream_interactions"] = json!([]);
    assert!(
        load(&empty)
            .unwrap()
            .propeller_slipstream_interactions()
            .is_empty()
    );
    let mut empty_without_propulsion = empty.clone();
    empty_without_propulsion["propulsion"] = Value::Null;
    assert!(
        load(&empty_without_propulsion)
            .unwrap()
            .propeller_slipstream_interactions()
            .is_empty()
    );

    let mut missing = fixture();
    missing
        .as_object_mut()
        .unwrap()
        .remove("propeller_slipstream_interactions");
    assert!(matches!(
        load(&missing),
        Err(ModelLoadError::InvalidStructure { .. })
    ));

    let v6 = include_str!("../../../tests/fixtures/synthetic_downwash_v6.json");
    assert_eq!(
        AircraftModelLoader::from_json_str(v6)
            .unwrap()
            .schema_version(),
        6
    );
}

#[test]
fn fingerprint_covers_factor_and_resolved_target_membership() {
    let baseline = load(&fixture()).unwrap().physics_fingerprint();
    let mut factor = fixture();
    factor["propeller_slipstream_interactions"][0]["slipstream_velocity_factor"] = json!(1.25);
    assert_ne!(baseline, load(&factor).unwrap().physics_fingerprint());

    let mut target = fixture();
    target["propeller_slipstream_interactions"][0]["target_element_ids"] =
        json!(["synthetic-untargeted"]);
    assert_ne!(baseline, load(&target).unwrap().physics_fingerprint());
}
