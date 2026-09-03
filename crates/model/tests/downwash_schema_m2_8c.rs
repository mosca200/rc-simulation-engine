//! M2.8C schema and immutable-runtime validation. All data is synthetic.

use model::{AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};

fn valid_v6() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_finite_wing_v5.json"
    ))
    .unwrap();
    value["schema_version"] = json!(6);
    value["aerodynamics"]["surfaces"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "horizontal-tail",
            "element_ids": ["synthetic-elevator-tail"],
            "span_axis_body": [0.0, 1.0, 0.0],
            "span_m": 0.35,
            "span_efficiency_factor": 0.85
        }));
    value["aero_downwash_interactions"] = json!([{
        "id": "wing-to-tail",
        "source_surface_id": "synthetic-wing-surface",
        "target_surface_id": "horizontal-tail",
        "downwash_factor": 1.5
    }]);
    value
}

fn load(value: &Value) -> Result<model::AircraftModel, ModelLoadError> {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

#[test]
fn valid_v6_resolves_surface_indices_and_factor() {
    let model = load(&valid_v6()).unwrap();
    assert_eq!(model.schema_version(), 6);
    assert_eq!(model.aero_surfaces().len(), 2);
    assert_eq!(model.aero_downwash_interactions().len(), 1);
    let interaction = &model.aero_downwash_interactions()[0];
    assert_eq!(interaction.id(), "wing-to-tail");
    assert_eq!(interaction.source_surface_index(), 0);
    assert_eq!(interaction.target_surface_index(), 1);
    assert_eq!(interaction.downwash_factor(), 1.5);
}

#[test]
fn interaction_ids_are_nonempty_unique_stable_ids() {
    let mut empty = valid_v6();
    empty["aero_downwash_interactions"][0]["id"] = json!("");
    assert!(matches!(
        load(&empty),
        Err(ModelLoadError::InvalidStableId {
            kind: "aerodynamic downwash interaction",
            ..
        })
    ));

    let mut duplicate = valid_v6();
    let first = duplicate["aero_downwash_interactions"][0].clone();
    duplicate["aero_downwash_interactions"]
        .as_array_mut()
        .unwrap()
        .push(first);
    assert!(matches!(
        load(&duplicate),
        Err(ModelLoadError::DuplicateStableId {
            kind: "aerodynamic downwash interaction",
            ..
        })
    ));
}

#[test]
fn unknown_and_identical_surfaces_are_rejected() {
    let mut unknown_source = valid_v6();
    unknown_source["aero_downwash_interactions"][0]["source_surface_id"] = json!("missing");
    assert!(matches!(
        load(&unknown_source),
        Err(ModelLoadError::UnresolvedDownwashSourceSurface { .. })
    ));

    let mut unknown_target = valid_v6();
    unknown_target["aero_downwash_interactions"][0]["target_surface_id"] = json!("missing");
    assert!(matches!(
        load(&unknown_target),
        Err(ModelLoadError::UnresolvedDownwashTargetSurface { .. })
    ));

    let mut same = valid_v6();
    same["aero_downwash_interactions"][0]["target_surface_id"] = json!("synthetic-wing-surface");
    assert!(matches!(
        load(&same),
        Err(ModelLoadError::DownwashSelfInteraction { .. })
    ));
}

#[test]
fn factor_must_be_finite_and_non_negative() {
    let mut negative = valid_v6();
    negative["aero_downwash_interactions"][0]["downwash_factor"] = json!(-0.1);
    assert!(matches!(
        load(&negative),
        Err(ModelLoadError::InvalidDownwashFactor { .. })
    ));

    let nonfinite = serde_json::to_string(&valid_v6())
        .unwrap()
        .replace("\"downwash_factor\":1.5", "\"downwash_factor\":1e999");
    assert!(AircraftModelLoader::from_json_str(&nonfinite).is_err());
}

#[test]
fn duplicate_targets_and_chained_graphs_are_rejected() {
    let mut duplicate_target = valid_v6();
    duplicate_target["aero_downwash_interactions"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "second-to-tail",
            "source_surface_id": "synthetic-wing-surface",
            "target_surface_id": "horizontal-tail",
            "downwash_factor": 0.5
        }));
    assert!(matches!(
        load(&duplicate_target),
        Err(ModelLoadError::DuplicateDownwashTarget { .. })
    ));

    let mut chained = valid_v6();
    chained["aero_downwash_interactions"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "tail-to-wing",
            "source_surface_id": "horizontal-tail",
            "target_surface_id": "synthetic-wing-surface",
            "downwash_factor": 0.5
        }));
    assert!(matches!(
        load(&chained),
        Err(ModelLoadError::ChainedDownwashSurface { .. })
    ));
}

#[test]
fn empty_interactions_are_explicit_and_factor_changes_physics_identity() {
    let mut missing = valid_v6();
    missing
        .as_object_mut()
        .unwrap()
        .remove("aero_downwash_interactions");
    assert!(matches!(
        load(&missing),
        Err(ModelLoadError::InvalidStructure { .. })
    ));

    let mut empty = valid_v6();
    empty["aero_downwash_interactions"] = json!([]);
    let empty_model = load(&empty).unwrap();
    assert!(empty_model.aero_downwash_interactions().is_empty());

    let baseline = load(&valid_v6()).unwrap().physics_fingerprint();
    let mut changed = valid_v6();
    changed["aero_downwash_interactions"][0]["downwash_factor"] = json!(1.25);
    assert_ne!(baseline, load(&changed).unwrap().physics_fingerprint());
}
