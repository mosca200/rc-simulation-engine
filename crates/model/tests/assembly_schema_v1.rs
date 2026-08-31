mod common;

use common::{load_value, valid_model_value, valid_v1_model_value};
use model::{ControlActuator, ModelLoadError};
use serde_json::json;

#[test]
fn as1_v0_continues_to_load_with_no_control_surface_bindings() {
    let model = load_value(&valid_model_value()).expect("S6 v0 fixture must remain accepted");

    assert_eq!(model.schema_version(), 0);
    assert!(model.control_surface_bindings().is_empty());
}

#[test]
fn as2_valid_v1_loads_ordered_resolved_control_surface_bindings() {
    let model = load_value(&valid_v1_model_value()).expect("valid schema-v1 model");

    assert_eq!(model.schema_version(), 1);
    let bindings = model.control_surface_bindings();
    assert_eq!(bindings.len(), 2);
    assert_eq!(bindings[0].id(), "aileron-first");
    assert_eq!(bindings[0].element_index(), 0);
    assert_eq!(bindings[0].actuator(), ControlActuator::Aileron);
    assert_eq!(
        bindings[0].deflection_gain().to_bits(),
        (-1.0_f64).to_bits()
    );
    assert_eq!(bindings[1].id(), "elevator-second");
    assert_eq!(bindings[1].element_index(), 1);
    assert_eq!(bindings[1].actuator(), ControlActuator::Elevator);
    assert_eq!(bindings[1].deflection_gain().to_bits(), 0.75_f64.to_bits());
}

#[test]
fn as3_unknown_binding_element_is_rejected() {
    let mut value = valid_v1_model_value();
    value["control_surface_bindings"][0]["element_id"] = json!("missing-element");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::UnresolvedControlSurfaceElementReference {
            binding_id,
            binding_index: 0,
            element_id,
        }) if binding_id == "aileron-first" && element_id == "missing-element"
    ));
}

#[test]
fn as4_duplicate_controlled_element_is_rejected() {
    let mut value = valid_v1_model_value();
    value["control_surface_bindings"][1]["element_id"] = json!("element-first");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::DuplicateControlledAeroElement {
            element_id,
            first_binding_id,
            first_index: 0,
            binding_id,
            duplicate_index: 1,
        }) if element_id == "element-first"
            && first_binding_id == "aileron-first"
            && binding_id == "elevator-second"
    ));
}

#[test]
fn as5_zero_binding_gain_is_rejected() {
    for gain in [0.0, -0.0] {
        let mut value = valid_v1_model_value();
        value["control_surface_bindings"][0]["deflection_gain"] = json!(gain);

        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidControlSurfaceDeflectionGain {
                binding_id,
                binding_index: 0,
                value,
            }) if binding_id == "aileron-first" && value == 0.0
        ));
    }
}

#[test]
fn nonfinite_binding_gain_and_unknown_actuator_are_rejected_structurally() {
    let json = serde_json::to_string(&valid_v1_model_value()).expect("valid fixture serializes");
    let nonfinite = json.replacen("\"deflection_gain\":-1.0", "\"deflection_gain\":1e400", 1);
    assert!(matches!(
        model::AircraftModelLoader::from_json_str(&nonfinite),
        Err(ModelLoadError::InvalidStructure { .. })
    ));

    let mut unknown_actuator = valid_v1_model_value();
    unknown_actuator["control_surface_bindings"][0]["actuator"] = json!("flap");
    assert!(matches!(
        load_value(&unknown_actuator),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn invalid_and_duplicate_binding_ids_are_rejected() {
    let mut invalid = valid_v1_model_value();
    invalid["control_surface_bindings"][0]["id"] = json!("Aileron Left");
    assert!(matches!(
        load_value(&invalid),
        Err(ModelLoadError::InvalidStableId {
            kind: "control-surface binding",
            index: 0,
            ..
        })
    ));

    let mut duplicate = valid_v1_model_value();
    duplicate["control_surface_bindings"][1]["id"] = json!("aileron-first");
    assert!(matches!(
        load_value(&duplicate),
        Err(ModelLoadError::DuplicateStableId {
            kind: "control-surface binding",
            first_index: 0,
            duplicate_index: 1,
            ..
        })
    ));
}

#[test]
fn v1_binding_section_is_required_and_binding_objects_are_strict() {
    let mut missing = valid_v1_model_value();
    missing
        .as_object_mut()
        .expect("root object")
        .remove("control_surface_bindings");
    assert!(matches!(
        load_value(&missing),
        Err(ModelLoadError::InvalidStructure { .. })
    ));

    let mut unknown_field = valid_v1_model_value();
    unknown_field["control_surface_bindings"][0]
        .as_object_mut()
        .expect("binding object")
        .insert("hinge_axis".to_owned(), json!([0.0, 1.0, 0.0]));
    assert!(matches!(
        load_value(&unknown_field),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn as6_v1_fingerprint_covers_binding_target_actuator_gain_and_order() {
    let baseline = valid_v1_model_value();
    let baseline_fingerprint = load_value(&baseline)
        .expect("baseline v1")
        .physics_fingerprint();

    let mut changed_gain = baseline.clone();
    changed_gain["control_surface_bindings"][0]["deflection_gain"] = json!(-1.25);
    assert_ne!(
        baseline_fingerprint,
        load_value(&changed_gain)
            .expect("valid changed gain")
            .physics_fingerprint()
    );

    let mut changed_target = baseline.clone();
    changed_target["control_surface_bindings"][0]["element_id"] = json!("element-second");
    changed_target["control_surface_bindings"][1]["element_id"] = json!("element-first");
    assert_ne!(
        baseline_fingerprint,
        load_value(&changed_target)
            .expect("valid changed targets")
            .physics_fingerprint()
    );

    let mut changed_actuator = baseline.clone();
    changed_actuator["control_surface_bindings"][0]["actuator"] = json!("rudder");
    assert_ne!(
        baseline_fingerprint,
        load_value(&changed_actuator)
            .expect("valid changed actuator")
            .physics_fingerprint()
    );

    let mut changed_order = baseline;
    changed_order["control_surface_bindings"]
        .as_array_mut()
        .expect("bindings array")
        .reverse();
    assert_ne!(
        baseline_fingerprint,
        load_value(&changed_order)
            .expect("valid changed order")
            .physics_fingerprint()
    );
}

#[test]
fn v1_fingerprint_excludes_binding_ids_and_presentation_metadata() {
    let baseline = valid_v1_model_value();
    let baseline_fingerprint = load_value(&baseline)
        .expect("baseline v1")
        .physics_fingerprint();

    let mut renamed = baseline.clone();
    renamed["control_surface_bindings"][0]["id"] = json!("renamed-binding");
    assert_eq!(
        baseline_fingerprint,
        load_value(&renamed)
            .expect("renamed nonphysical binding")
            .physics_fingerprint()
    );

    let mut presentation_changed = baseline;
    presentation_changed["presentation"]["glb_path"] = json!("visual/v1-only.glb");
    assert_eq!(
        baseline_fingerprint,
        load_value(&presentation_changed)
            .expect("changed v1 presentation")
            .physics_fingerprint()
    );
}
