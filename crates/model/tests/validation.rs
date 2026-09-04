mod common;

use common::{load_value, minimal_model_value, set, valid_model_value, valid_v1_model_value};
use model::{AircraftModelLoader, ModelLoadError};
use serde_json::{Value, json};
use sim_core::{
    AeroElementError, BatteryConfigError, ControlConfigError, MotorConfigError, ParameterError,
    PolarError, PropellerCoefficientError, PropellerConfigError,
};

#[test]
fn valid_minimal_model_loads() {
    let model = load_value(&minimal_model_value()).expect("minimal v0 model must load");

    assert_eq!(model.schema_version(), 0);
    assert_eq!(model.aero_polars().len(), 1);
    assert_eq!(model.aero_elements().len(), 1);
    assert!(model.propulsion().is_none());
    assert!(model.presentation().is_none());
}

#[test]
fn malformed_json_is_a_parse_error() {
    let result = AircraftModelLoader::from_json_str(r#"{"schema_version": 0,"#);

    assert!(matches!(result, Err(ModelLoadError::JsonParse { .. })));
}

#[test]
fn unsupported_v9_schema_is_rejected_before_structure_validation() {
    let mut value = valid_model_value();
    value["schema_version"] = json!(9);
    value
        .as_object_mut()
        .expect("root object")
        .insert("future_v1_field".to_owned(), json!(true));

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::UnsupportedSchemaVersion { found: 9 })
    ));
}

#[test]
fn missing_or_wrong_type_schema_version_is_structural_error() {
    let mut missing = valid_model_value();
    missing
        .as_object_mut()
        .expect("root object")
        .remove("schema_version");
    assert!(matches!(
        load_value(&missing),
        Err(ModelLoadError::InvalidStructure { .. })
    ));

    let mut wrong_type = valid_model_value();
    wrong_type["schema_version"] = json!("zero");
    assert!(matches!(
        load_value(&wrong_type),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn unknown_root_field_is_rejected() {
    let mut value = valid_model_value();
    value
        .as_object_mut()
        .expect("root object")
        .insert("unknown_root".to_owned(), json!(1));

    let error = load_value(&value).expect_err("unknown root field must fail");
    match error {
        ModelLoadError::InvalidStructure { source } => {
            assert!(source.to_string().contains("unknown field"));
        }
        other => panic!("unexpected error: {other}"),
    }
}

#[test]
fn unknown_nested_field_is_rejected() {
    let mut value = valid_model_value();
    value["controls"]["response"]["roll"]
        .as_object_mut()
        .expect("roll response object")
        .insert("rates".to_owned(), json!(0.5));

    let error = load_value(&value).expect_err("unknown nested field must fail");
    match error {
        ModelLoadError::InvalidStructure { source } => {
            assert!(source.to_string().contains("unknown field"));
        }
        other => panic!("unexpected error: {other}"),
    }
}

#[test]
fn missing_required_field_is_rejected() {
    let mut value = valid_model_value();
    value["rigid_body"]
        .as_object_mut()
        .expect("rigid body object")
        .remove("mass_kg");

    let error = load_value(&value).expect_err("missing mass must fail");
    match error {
        ModelLoadError::InvalidStructure { source } => {
            assert!(source.to_string().contains("missing field"));
        }
        other => panic!("unexpected error: {other}"),
    }
}

#[test]
fn wrong_field_type_is_rejected() {
    let mut value = valid_model_value();
    value["rigid_body"]["mass_kg"] = json!("two kilograms");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn duplicate_json_keys_are_rejected_as_structural_errors() {
    let json = serde_json::to_string(&valid_model_value()).expect("valid fixture serializes");
    for (unique, duplicate) in [
        (
            r#""schema_version":0"#,
            r#""schema_version":0,"schema_version":0"#,
        ),
        (r#""mass_kg":2.5"#, r#""mass_kg":2.5,"mass_kg":2.5"#),
    ] {
        assert_eq!(
            json.matches(unique).count(),
            1,
            "fixture key must be unique"
        );
        let duplicated = json.replacen(unique, duplicate, 1);
        assert!(matches!(
            AircraftModelLoader::from_json_str(&duplicated),
            Err(ModelLoadError::InvalidStructure { .. })
        ));
    }
}

#[test]
fn invalid_model_ids_are_rejected_without_rewriting() {
    for invalid in ["", "Uppercase", "has space", "non-ascii-\u{e9}", "has.dot"] {
        let mut value = valid_model_value();
        value["model_id"] = json!(invalid);

        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidModelId { value }) if value == invalid
        ));
    }

    let mut allowed = minimal_model_value();
    allowed["model_id"] = json!("a-z_09");
    assert!(load_value(&allowed).is_ok());
}

#[test]
fn invalid_polar_and_element_ids_are_rejected() {
    let mut invalid_polar = valid_model_value();
    invalid_polar["aerodynamics"]["polars"][0]["id"] = json!("Polar First");
    assert!(matches!(
        load_value(&invalid_polar),
        Err(ModelLoadError::InvalidStableId {
            kind: "polar",
            index: 0,
            ..
        })
    ));

    let mut invalid_element = valid_model_value();
    invalid_element["aerodynamics"]["elements"][0]["id"] = json!("element.first");
    assert!(matches!(
        load_value(&invalid_element),
        Err(ModelLoadError::InvalidStableId {
            kind: "aerodynamic element",
            index: 0,
            ..
        })
    ));
}

#[test]
fn duplicate_polar_id_is_rejected_with_both_indices() {
    let mut value = valid_model_value();
    let duplicate = value["aerodynamics"]["polars"][0].clone();
    value["aerodynamics"]["polars"]
        .as_array_mut()
        .expect("polars array")
        .push(duplicate);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::DuplicateStableId {
            kind: "polar",
            id,
            first_index: 0,
            duplicate_index: 2,
        }) if id == "polar-first"
    ));
}

#[test]
fn duplicate_aero_element_id_is_rejected_with_both_indices() {
    let mut value = valid_model_value();
    let duplicate = value["aerodynamics"]["elements"][0].clone();
    value["aerodynamics"]["elements"]
        .as_array_mut()
        .expect("elements array")
        .push(duplicate);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::DuplicateStableId {
            kind: "aerodynamic element",
            id,
            first_index: 0,
            duplicate_index: 2,
        }) if id == "element-first"
    ));
}

#[test]
fn missing_polar_reference_is_rejected() {
    let mut value = valid_model_value();
    value["aerodynamics"]["elements"][0]["polar_id"] = json!("missing-polar");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::UnresolvedPolarReference {
            element_id,
            element_index: 0,
            polar_id,
        }) if element_id == "element-first" && polar_id == "missing-polar"
    ));
}

#[test]
fn declaration_order_is_preserved_and_polar_references_are_resolved_to_indices() {
    let model = load_value(&valid_model_value()).expect("valid model");

    let polar_ids: Vec<_> = model.aero_polars().iter().map(|polar| polar.id()).collect();
    let element_ids: Vec<_> = model
        .aero_elements()
        .iter()
        .map(|element| element.id())
        .collect();
    let polar_indices: Vec<_> = model
        .aero_elements()
        .iter()
        .map(|element| element.polar_index())
        .collect();

    assert_eq!(polar_ids, ["polar-first", "polar-second"]);
    assert_eq!(element_ids, ["element-first", "element-second"]);
    assert_eq!(polar_indices, [1, 0]);
}

#[test]
fn invalid_mass_is_rejected_by_rigid_body_validation() {
    let mut value = valid_model_value();
    value["rigid_body"]["mass_kg"] = json!(0.0);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidRigidBody {
            source: ParameterError::InvalidMass
        })
    ));
}

#[test]
fn non_positive_definite_inertia_is_rejected_by_rigid_body_validation() {
    let mut value = valid_model_value();
    value["rigid_body"]["inertia_body_kg_m2"] =
        json!([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidRigidBody {
            source: ParameterError::NonPositiveDefiniteInertia
        })
    ));
}

#[test]
fn invalid_quaternion_is_rejected_without_normalization() {
    let mut value = valid_model_value();
    value["aerodynamics"]["elements"][0]["orientation_body_from_element_wxyz"] =
        json!([2.0, 0.0, 0.0, 0.0]);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidAeroElement {
            id,
            index: 0,
            source: AeroElementError::InvalidOrientation,
        }) if id == "element-first"
    ));
}

#[test]
fn invalid_polar_is_rejected_without_sorting_or_clamping() {
    let mut negative_drag = valid_model_value();
    negative_drag["aerodynamics"]["polars"][0]["samples"][1]["cd"] = json!(-0.01);
    assert!(matches!(
        load_value(&negative_drag),
        Err(ModelLoadError::InvalidPolar {
            index: 0,
            source: PolarError::NegativeDragCoefficient { index: 1 },
            ..
        })
    ));

    let mut unordered = valid_model_value();
    unordered["aerodynamics"]["polars"][0]["samples"][1]["alpha_rad"] = json!(-0.25);
    assert!(matches!(
        load_value(&unordered),
        Err(ModelLoadError::InvalidPolar {
            index: 0,
            source: PolarError::NonIncreasingAlpha { index: 1 },
            ..
        })
    ));
}

#[test]
fn invalid_aero_element_is_rejected() {
    let mut value = valid_model_value();
    value["aerodynamics"]["elements"][0]["area_m2"] = json!(0.0);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidAeroElement {
            index: 0,
            source: AeroElementError::InvalidArea,
            ..
        })
    ));
}

#[test]
fn invalid_rate_and_expo_are_rejected_without_clamping() {
    for (pointer, replacement, expected) in [
        (
            "/controls/response/roll/rate",
            json!(1.01),
            ControlConfigError::InvalidAxisRate,
        ),
        (
            "/controls/response/roll/expo",
            json!(-0.01),
            ControlConfigError::InvalidAxisExpo,
        ),
    ] {
        let mut value = valid_model_value();
        set(&mut value, pointer, replacement);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidControls {
                component: "response.roll",
                source,
            }) if source == expected
        ));
    }
}

#[test]
fn invalid_servo_travel_and_speed_are_rejected() {
    for (pointer, replacement, expected) in [
        (
            "/controls/servos/elevator/min_angle_rad",
            json!(-0.02),
            ControlConfigError::InvalidServoTravel,
        ),
        (
            "/controls/servos/elevator/max_speed_rad_s",
            json!(0.0),
            ControlConfigError::InvalidServoSpeed,
        ),
    ] {
        let mut value = valid_model_value();
        set(&mut value, pointer, replacement);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidControls {
                component: "servos.elevator",
                source,
            }) if source == expected
        ));
    }
}

#[test]
fn invalid_battery_config_is_rejected() {
    let mut value = valid_model_value();
    value["propulsion"]["battery"]["open_circuit_voltage_v"] = json!(0.0);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidBattery {
            source: BatteryConfigError::InvalidOpenCircuitVoltage
        })
    ));
}

#[test]
fn invalid_motor_config_is_rejected() {
    let mut value = valid_model_value();
    value["propulsion"]["motor"]["kv_rpm_per_v"] = json!(0.0);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidMotor {
            source: MotorConfigError::InvalidKv
        })
    ));
}

#[test]
fn invalid_propeller_config_is_rejected() {
    let mut value = valid_model_value();
    value["propulsion"]["propeller"]["diameter_m"] = json!(0.0);

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidPropeller {
            source: PropellerConfigError::InvalidDiameter
        })
    ));
}

#[test]
fn only_explicit_propeller_spin_names_are_accepted() {
    let mut value = valid_model_value();
    value["propulsion"]["propeller"]["spin_direction"] = json!("cw");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn invalid_propeller_coefficient_table_is_rejected() {
    for (pointer, replacement, expected) in [
        (
            "/propulsion/coefficient_table/samples/1/ct",
            json!(-0.01),
            PropellerCoefficientError::NegativeThrustCoefficient { index: 1 },
        ),
        (
            "/propulsion/coefficient_table/samples/1/cq",
            json!(-0.01),
            PropellerCoefficientError::NegativeTorqueCoefficient { index: 1 },
        ),
        (
            "/propulsion/coefficient_table/samples/1/advance_ratio_j",
            json!(0.0),
            PropellerCoefficientError::NonIncreasingAdvanceRatio { index: 1 },
        ),
    ] {
        let mut value = valid_model_value();
        set(&mut value, pointer, replacement);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidPropellerCoefficientTable { source })
                if source == expected
        ));
    }
}

#[test]
fn absolute_presentation_paths_are_rejected_portably() {
    for invalid in [
        "/absolute/aircraft.glb",
        r"C:\absolute\aircraft.glb",
        r"\\server\share\aircraft.glb",
    ] {
        let mut value = valid_model_value();
        value["presentation"]["glb_path"] = json!(invalid);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidPresentationAssetPath { path }) if path == invalid
        ));
    }
}

#[test]
fn parent_traversal_presentation_paths_are_rejected_portably() {
    for invalid in [
        "../aircraft.glb",
        "assets/../aircraft.glb",
        r"..\aircraft.glb",
        r"assets\..\aircraft.glb",
    ] {
        let mut value = valid_model_value();
        value["presentation"]["glb_path"] = json!(invalid);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidPresentationAssetPath { path }) if path == invalid
        ));
    }
}

#[test]
fn valid_relative_presentation_path_is_preserved() {
    let mut value = valid_model_value();
    value["presentation"]["glb_path"] = json!("visual/aircraft-v0.glb");

    let model = load_value(&value).expect("valid relative path");
    assert_eq!(
        model.presentation().expect("presentation").glb_path(),
        "visual/aircraft-v0.glb"
    );
}

fn explicit_articulation() -> Value {
    json!([{
        "visual_primitive_index": 7,
        "surface": "left_aileron",
        "control_surface_binding_id": "aileron-first",
        "hinge_origin_render_body_m": [0.1, 0.2, 0.3],
        "hinge_axis_render_body": [2.0, 0.0, 0.0],
        "visual_gain": -1.5
    }])
}

#[test]
fn explicit_presentation_metadata_is_validated_and_excluded_from_fingerprint() {
    let without = valid_v1_model_value();
    let baseline = load_value(&without).unwrap();
    let mut with = without;
    with["presentation"]["articulated_surfaces"] = explicit_articulation();
    let presented = load_value(&with).unwrap();
    let surface = &presented.presentation().unwrap().articulated_surfaces()[0];
    assert_eq!(surface.visual_primitive_index(), 7);
    assert_eq!(surface.control_surface_binding_id(), "aileron-first");
    assert_eq!(surface.hinge_origin_render_body_m(), [0.1, 0.2, 0.3]);
    assert_eq!(surface.hinge_axis_render_body(), [2.0, 0.0, 0.0]);
    assert_eq!(surface.visual_gain(), -1.5);
    assert_eq!(
        baseline.physics_fingerprint(),
        presented.physics_fingerprint()
    );
}

#[test]
fn invalid_or_implicit_presentation_bindings_fail_closed() {
    let mut unknown = valid_v1_model_value();
    unknown["presentation"]["articulated_surfaces"] = explicit_articulation();
    unknown["presentation"]["articulated_surfaces"][0]["control_surface_binding_id"] =
        json!("not-a-binding");
    assert!(matches!(
        load_value(&unknown),
        Err(ModelLoadError::UnresolvedPresentationControlSurfaceBinding { .. })
    ));

    let mut wrong_actuator = valid_v1_model_value();
    wrong_actuator["presentation"]["articulated_surfaces"] = explicit_articulation();
    wrong_actuator["presentation"]["articulated_surfaces"][0]["surface"] = json!("rudder");
    assert!(matches!(
        load_value(&wrong_actuator),
        Err(ModelLoadError::PresentationBindingActuatorMismatch { .. })
    ));

    let mut duplicate = valid_v1_model_value();
    let mapping = explicit_articulation()[0].clone();
    duplicate["presentation"]["articulated_surfaces"] = json!([mapping.clone(), mapping]);
    assert!(matches!(
        load_value(&duplicate),
        Err(ModelLoadError::DuplicatePresentationVisualPrimitive { .. })
    ));

    let mut zero_axis = valid_v1_model_value();
    zero_axis["presentation"]["articulated_surfaces"] = explicit_articulation();
    zero_axis["presentation"]["articulated_surfaces"][0]["hinge_axis_render_body"] =
        json!([0.0, 0.0, 0.0]);
    assert!(matches!(
        load_value(&zero_axis),
        Err(ModelLoadError::InvalidPresentationArticulation { .. })
    ));
}

#[test]
fn propulsion_may_be_null_or_absent() {
    let mut null = valid_model_value();
    null["propulsion"] = Value::Null;
    assert!(
        load_value(&null)
            .expect("null propulsion")
            .propulsion()
            .is_none()
    );

    let mut absent = valid_model_value();
    absent
        .as_object_mut()
        .expect("root object")
        .remove("propulsion");
    assert!(
        load_value(&absent)
            .expect("absent propulsion")
            .propulsion()
            .is_none()
    );
}

#[test]
fn present_propulsion_must_be_complete() {
    let mut value = valid_model_value();
    value["propulsion"]
        .as_object_mut()
        .expect("propulsion object")
        .remove("motor");

    assert!(matches!(
        load_value(&value),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}
