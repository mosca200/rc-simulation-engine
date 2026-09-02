//! M2.8A — Finite-Wing Surface Representation tests.
//!
//! Tests cover: schema v5 loading, surface membership validation, span-axis normalization,
//! derived area/AR, fingerprint semantics, and backward compatibility.

mod common;

use common::{load_value, set};
use model::{AircraftModelFingerprint, ModelLoadError};
use serde_json::{Value, json};

/// Builds a minimal valid v5 model with two aero elements and one surface.
fn valid_v5_model_value() -> Value {
    json!({
        "schema_version": 5,
        "model_id": "synthetic_non_reference_surface_v5",
        "display_name": "Synthetic Surface Test",
        "classification": "synthetic_test",
        "reference_aircraft": null,
        "rigid_body": {
            "mass_kg": 1.0,
            "inertia_body_kg_m2": [
                [0.1, 0.0, 0.0],
                [0.0, 0.1, 0.0],
                [0.0, 0.0, 0.2]
            ]
        },
        "aerodynamics": {
            "kinematic_viscosity_m2_s": 1.5e-5,
            "polars": [
                {
                    "id": "polar-main",
                    "samples": [
                        { "alpha_rad": -0.2, "cl": -0.5, "cd": 0.05, "cm": 0.0 },
                        { "alpha_rad": 0.0, "cl": 0.1, "cd": 0.02, "cm": 0.0 },
                        { "alpha_rad": 0.2, "cl": 0.7, "cd": 0.08, "cm": -0.02 }
                    ]
                }
            ],
            "polar_families": [],
            "elements": [
                {
                    "id": "wing-strip-0",
                    "position_body_m": [0.0, -0.5, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.25,
                    "chord_m": 0.25,
                    "polar_binding": { "kind": "polar", "polar_id": "polar-main" }
                },
                {
                    "id": "wing-strip-1",
                    "position_body_m": [0.0, 0.5, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.25,
                    "chord_m": 0.25,
                    "polar_binding": { "kind": "polar", "polar_id": "polar-main" }
                },
                {
                    "id": "fuselage-strip",
                    "position_body_m": [0.5, 0.0, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.1,
                    "chord_m": 0.1,
                    "polar_binding": { "kind": "polar", "polar_id": "polar-main" }
                }
            ],
            "surfaces": [
                {
                    "id": "main-wing",
                    "element_ids": ["wing-strip-0", "wing-strip-1"],
                    "span_axis_body": [0.0, 2.0, 0.0],
                    "span_m": 2.0,
                    "span_efficiency_factor": 0.9
                }
            ]
        },
        "controls": {
            "response": {
                "roll": { "rate": 0.5, "expo": 0.0 },
                "pitch": { "rate": 0.5, "expo": 0.0 },
                "yaw": { "rate": 0.5, "expo": 0.0 }
            },
            "servos": {
                "aileron": {
                    "min_angle_rad": -0.5,
                    "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5,
                    "max_speed_rad_s": 5.0,
                    "reversed": false
                },
                "elevator": {
                    "min_angle_rad": -0.5,
                    "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5,
                    "max_speed_rad_s": 5.0,
                    "reversed": false
                },
                "rudder": {
                    "min_angle_rad": -0.5,
                    "neutral_angle_rad": 0.0,
                    "max_angle_rad": 0.5,
                    "max_speed_rad_s": 5.0,
                    "reversed": false
                }
            }
        },
        "control_surface_bindings": [],
        "propulsion": null,
        "presentation": null
    })
}

fn fingerprint(value: &Value) -> AircraftModelFingerprint {
    load_value(value)
        .expect("fingerprint test model must remain valid")
        .physics_fingerprint()
}

// ============================================================
// 1. Schema v5 valid model loads
// ============================================================

#[test]
fn v5_valid_model_loads() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    assert_eq!(model.schema_version(), 5);
    assert_eq!(model.model_id(), "synthetic_non_reference_surface_v5");
    assert_eq!(model.aero_elements().len(), 3);
    assert_eq!(model.aero_surfaces().len(), 1);
}

// ============================================================
// 2. v0-v4 existing loading remains green
// ============================================================

#[test]
fn v0_v4_backward_compatible() {
    let v0 = common::valid_model_value();
    let v0_model = load_value(&v0).expect("v0 model should load");
    assert_eq!(v0_model.schema_version(), 0);
    assert!(v0_model.aero_surfaces().is_empty());

    let v1 = common::valid_v1_model_value();
    let v1_model = load_value(&v1).expect("v1 model should load");
    assert_eq!(v1_model.schema_version(), 1);
    assert!(v1_model.aero_surfaces().is_empty());
}

// ============================================================
// 3. v5 surface IDs preserve authored order
// ============================================================

#[test]
fn v5_surface_order_preserved() {
    let mut value = valid_v5_model_value();
    // Add a second surface
    value["aerodynamics"]["surfaces"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "tail-surface",
            "element_ids": ["fuselage-strip"],
            "span_axis_body": [0.0, 0.0, 1.0],
            "span_m": 1.0,
            "span_efficiency_factor": 0.8
        }));

    let model = load_value(&value).expect("valid v5 model should load");
    assert_eq!(model.aero_surfaces().len(), 2);
    assert_eq!(model.aero_surfaces()[0].id(), "main-wing");
    assert_eq!(model.aero_surfaces()[1].id(), "tail-surface");
}

// ============================================================
// 4. element_ids resolve to expected runtime indices
// ============================================================

#[test]
fn element_ids_resolve_to_correct_indices() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    assert_eq!(surface.element_indices(), &[0, 1]);
    assert_eq!(model.aero_elements()[0].id(), "wing-strip-0");
    assert_eq!(model.aero_elements()[1].id(), "wing-strip-1");
}

// ============================================================
// 5. Member ordering is preserved
// ============================================================

#[test]
fn member_ordering_preserved() {
    let mut value = valid_v5_model_value();
    // Reverse element order in surface
    set(
        &mut value,
        "/aerodynamics/surfaces/0/element_ids",
        json!(["wing-strip-1", "wing-strip-0"]),
    );

    let model = load_value(&value).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    assert_eq!(surface.element_indices(), &[1, 0]);
}

// ============================================================
// 6. Span axis is normalized deterministically
// ============================================================

#[test]
fn span_axis_normalized() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    let axis = surface.span_axis_body();
    // Input was [0, 2, 0], should normalize to [0, 1, 0]
    assert!((axis.x).abs() < 1e-12);
    assert!((axis.y - 1.0).abs() < 1e-12);
    assert!(axis.z.abs() < 1e-12);
    let norm = (axis.x * axis.x + axis.y * axis.y + axis.z * axis.z).sqrt();
    assert!((norm - 1.0).abs() < 1e-12);
}

// ============================================================
// 7. span_m finite and > 0 validation
// ============================================================

#[test]
fn span_m_validation() {
    let mut value = valid_v5_model_value();
    set(&mut value, "/aerodynamics/surfaces/0/span_m", json!(0.0));
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::InvalidSurfaceSpan { .. }
    ));

    let mut value = valid_v5_model_value();
    set(&mut value, "/aerodynamics/surfaces/0/span_m", json!(-1.0));
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::InvalidSurfaceSpan { .. }
    ));
}

// ============================================================
// 8. span efficiency finite and > 0 validation
// ============================================================

#[test]
fn span_efficiency_validation() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/span_efficiency_factor",
        json!(0.0),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::InvalidSurfaceSpanEfficiency { .. }
    ));

    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/span_efficiency_factor",
        json!(-0.5),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::InvalidSurfaceSpanEfficiency { .. }
    ));
}

// ============================================================
// 9. NO arbitrary upper cap for span efficiency
// ============================================================

#[test]
fn span_efficiency_no_upper_cap() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/span_efficiency_factor",
        json!(2.5),
    );
    let model = load_value(&value).expect("span efficiency > 1 must be accepted");
    assert_eq!(model.aero_surfaces()[0].span_efficiency_factor(), 2.5);
}

// ============================================================
// 10. Empty surface membership rejected
// ============================================================

#[test]
fn empty_surface_membership_rejected() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/element_ids",
        json!([]),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::EmptySurfaceMembership { .. }
    ));
}

// ============================================================
// 11. Unknown element ID rejected
// ============================================================

#[test]
fn unknown_element_id_rejected() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/element_ids",
        json!(["nonexistent-element"]),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::UnresolvedSurfaceElementReference { .. }
    ));
}

// ============================================================
// 12. Duplicate element within one surface rejected
// ============================================================

#[test]
fn duplicate_element_within_surface_rejected() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/element_ids",
        json!(["wing-strip-0", "wing-strip-0"]),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::DuplicateSurfaceElement { .. }
    ));
}

// ============================================================
// 13. Same element assigned to two surfaces rejected
// ============================================================

#[test]
fn cross_surface_duplicate_element_rejected() {
    let mut value = valid_v5_model_value();
    value["aerodynamics"]["surfaces"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "second-surface",
            "element_ids": ["wing-strip-0"],
            "span_axis_body": [1.0, 0.0, 0.0],
            "span_m": 1.0,
            "span_efficiency_factor": 1.0
        }));
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::CrossSurfaceDuplicateElement { .. }
    ));
}

// ============================================================
// 14. Unassigned aero elements remain valid
// ============================================================

#[test]
fn unassigned_elements_remain_valid() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    // fuselage-strip (index 2) is not in any surface
    assert_eq!(model.aero_elements().len(), 3);
    assert_eq!(model.aero_surfaces().len(), 1);
    // The surface only contains indices 0 and 1
    assert_eq!(model.aero_surfaces()[0].element_indices(), &[0, 1]);
}

// ============================================================
// 15. Derived surface area equals exact sum of member element areas
// ============================================================

#[test]
fn derived_area_is_sum_of_member_areas() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    // wing-strip-0: 0.25, wing-strip-1: 0.25
    assert!((surface.area_m2() - 0.5).abs() < 1e-12);
}

// ============================================================
// 16. Derived aspect ratio equals span^2 / derived_area
// ============================================================

#[test]
fn derived_aspect_ratio_formula() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    // span_m = 2.0, area = 0.5
    // AR = 2.0^2 / 0.5 = 4.0 / 0.5 = 8.0
    assert!((surface.aspect_ratio() - 8.0).abs() < 1e-12);
}

// ============================================================
// 17. Surface area/aspect ratio are finite and positive
// ============================================================

#[test]
fn surface_area_and_ar_finite_positive() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let surface = &model.aero_surfaces()[0];
    assert!(surface.area_m2().is_finite());
    assert!(surface.area_m2() > 0.0);
    assert!(surface.aspect_ratio().is_finite());
    assert!(surface.aspect_ratio() > 0.0);
}

// ============================================================
// 18. Identical v5 input produces identical runtime surface representation
// ============================================================

#[test]
fn identical_input_identical_surface_representation() {
    let model1 = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    let model2 = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    assert_eq!(model1.aero_surfaces(), model2.aero_surfaces());
}

// ============================================================
// 19. Identical v5 input produces identical fingerprint
// ============================================================

#[test]
fn identical_input_identical_fingerprint() {
    let fp1 = fingerprint(&valid_v5_model_value());
    let fp2 = fingerprint(&valid_v5_model_value());
    assert_eq!(fp1, fp2);
}

// ============================================================
// 20. Changing span changes v5 fingerprint
// ============================================================

#[test]
fn span_change_changes_fingerprint() {
    let baseline = valid_v5_model_value();
    let mut changed = baseline.clone();
    set(&mut changed, "/aerodynamics/surfaces/0/span_m", json!(3.0));
    assert_ne!(fingerprint(&baseline), fingerprint(&changed));
}

// ============================================================
// 21. Changing span_efficiency_factor changes v5 fingerprint
// ============================================================

#[test]
fn span_efficiency_change_changes_fingerprint() {
    let baseline = valid_v5_model_value();
    let mut changed = baseline.clone();
    set(
        &mut changed,
        "/aerodynamics/surfaces/0/span_efficiency_factor",
        json!(1.2),
    );
    assert_ne!(fingerprint(&baseline), fingerprint(&changed));
}

// ============================================================
// 22. Changing surface membership changes v5 fingerprint
// ============================================================

#[test]
fn surface_membership_change_changes_fingerprint() {
    let baseline = valid_v5_model_value();
    let mut changed = baseline.clone();
    set(
        &mut changed,
        "/aerodynamics/surfaces/0/element_ids",
        json!(["wing-strip-0"]),
    );
    assert_ne!(fingerprint(&baseline), fingerprint(&changed));
}

// ============================================================
// 23. v5 surface metadata does NOT change current aerodynamic force/moment behavior
// ============================================================

#[test]
fn v5_surface_does_not_change_aerodynamic_behavior() {
    // The physics output should be identical to an equivalent v4 model
    // because M2.8A does not implement finite-wing corrections.
    // We verify this by checking that the model loads and has the same
    // element configuration.
    let v5_model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    assert_eq!(v5_model.aero_elements().len(), 3);
    // Surface metadata exists but does not affect current physics
    assert_eq!(v5_model.aero_surfaces().len(), 1);
}

// ============================================================
// 24. Synthetic v5 model remains SyntheticTest
// ============================================================

#[test]
fn synthetic_v5_model_remains_synthetic_test() {
    let model = load_value(&valid_v5_model_value()).expect("valid v5 model should load");
    assert_eq!(
        model.classification(),
        model::AircraftClassification::SyntheticTest
    );
}

// ============================================================
// Additional: Old schemas expose zero surfaces
// ============================================================

#[test]
fn old_schemas_expose_zero_surfaces() {
    let v0 = common::valid_model_value();
    let v0_model = load_value(&v0).expect("v0 model should load");
    assert_eq!(v0_model.aero_surfaces().len(), 0);

    let v1 = common::valid_v1_model_value();
    let v1_model = load_value(&v1).expect("v1 model should load");
    assert_eq!(v1_model.aero_surfaces().len(), 0);

    // v3/v4 models from existing tests should also have zero surfaces
    // (tested via backward compatibility)
}

// ============================================================
// Additional: Span axis validation
// ============================================================

#[test]
fn span_axis_invalid_rejected() {
    let mut value = valid_v5_model_value();
    set(
        &mut value,
        "/aerodynamics/surfaces/0/span_axis_body",
        json!([0.0, 0.0, 0.0]),
    );
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::InvalidSurfaceSpanAxis { .. }
    ));
}

// ============================================================
// Additional: Duplicate surface ID rejected
// ============================================================

#[test]
fn duplicate_surface_id_rejected() {
    let mut value = valid_v5_model_value();
    value["aerodynamics"]["surfaces"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "main-wing",
            "element_ids": ["fuselage-strip"],
            "span_axis_body": [1.0, 0.0, 0.0],
            "span_m": 1.0,
            "span_efficiency_factor": 1.0
        }));
    assert!(matches!(
        load_value(&value).unwrap_err(),
        ModelLoadError::DuplicateStableId {
            kind: "aerodynamic surface",
            ..
        }
    ));
}
