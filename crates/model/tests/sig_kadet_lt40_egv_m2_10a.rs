//! M2.10A — SIG Kadet LT-40 EGV loadable reference aircraft model.
//!
//! Validates that the first real reference aircraft model loads, has the expected
//! classification and metadata, exposes documented geometry, and produces a
//! deterministic physics fingerprint.

use model::{
    AircraftClassification, AircraftModelLoader, ControlActuator, RuntimeAeroPolarBinding,
    load_aircraft_model,
};
use std::path::Path;

fn model_path() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("models/sig_kadet_lt40_egv/model.json")
}

fn load_lt40() -> model::AircraftModel {
    load_aircraft_model(model_path()).expect("SIG Kadet LT-40 EGV model must load")
}

#[test]
fn lt40_model_loads_successfully() {
    let model = load_lt40();
    assert_eq!(model.schema_version(), 7);
    assert_eq!(model.model_id(), "sig-kadet-lt40-egv");
    assert_eq!(model.display_name(), "SIG Kadet LT-40 EGV");
}

#[test]
fn lt40_classification_and_reference_metadata() {
    let model = load_lt40();
    assert_eq!(
        model.classification(),
        AircraftClassification::ReferenceAircraft
    );
    let reference = model
        .reference_aircraft()
        .expect("reference_aircraft must be present for reference_aircraft classification");
    assert_eq!(
        reference.identity().manufacturer(),
        Some("SIG Manufacturing")
    );
    assert_eq!(reference.identity().aircraft_name(), Some("Kadet LT-40"));
    assert_eq!(reference.identity().variant(), Some("EGV ARF"));
    assert_eq!(
        reference.identity().stable_reference_id(),
        Some("sig-kadet-lt40-egv")
    );
}

#[test]
fn lt40_reference_physical_specification() {
    let model = load_lt40();
    let reference = model.reference_aircraft().unwrap();
    let spec = reference.physical_specification();

    let wingspan = spec.wingspan_m().expect("wingspan must be present");
    assert!((wingspan.value() - 1.778).abs() < 1e-10);

    let area = spec
        .reference_wing_area_m2()
        .expect("wing area must be present");
    assert!((area.value() - 0.580644).abs() < 1e-10);

    let length = spec
        .aircraft_length_m()
        .expect("aircraft length must be present");
    assert!((length.value() - 1.447).abs() < 1e-10);
}

#[test]
fn lt40_rigid_body_mass_and_inertia() {
    let model = load_lt40();
    let rb = model.rigid_body();
    assert!((rb.mass_kg() - 2.778).abs() < 1e-10);
    let inertia = rb.inertia_body_kg_m2();
    assert!((inertia[(0, 0)] - 0.30).abs() < 1e-10);
    assert!((inertia[(1, 1)] - 0.35).abs() < 1e-10);
    assert!((inertia[(2, 2)] - 0.55).abs() < 1e-10);
    assert!(inertia[(0, 1)].abs() < 1e-15);
    assert!(inertia[(0, 2)].abs() < 1e-15);
    assert!(inertia[(1, 2)].abs() < 1e-15);
}

#[test]
fn lt40_aero_element_count_and_ids() {
    let model = load_lt40();
    let elements = model.aero_elements();
    assert_eq!(elements.len(), 8);

    let expected_ids = [
        "wing-left-fixed",
        "wing-left-aileron",
        "wing-right-fixed",
        "wing-right-aileron",
        "horizontal-tail-fixed",
        "elevator",
        "vertical-tail-fixed",
        "rudder",
    ];
    for (element, expected_id) in elements.iter().zip(&expected_ids) {
        assert_eq!(element.id(), *expected_id);
    }
}

#[test]
fn lt40_reynolds_family_bindings() {
    let model = load_lt40();
    let families = model.aero_polar_families();
    assert_eq!(families.len(), 2);
    assert_eq!(families[0].id(), "wing-clark-y-provisional");
    assert_eq!(families[1].id(), "tail-symmetric-provisional");

    let wing_family = &families[0];
    assert_eq!(wing_family.family().nodes().len(), 3);
    assert!((wing_family.family().nodes()[0].reynolds_number() - 200_000.0).abs() < 1.0);
    assert!((wing_family.family().nodes()[1].reynolds_number() - 300_000.0).abs() < 1.0);
    assert!((wing_family.family().nodes()[2].reynolds_number() - 500_000.0).abs() < 1.0);

    let tail_family = &families[1];
    assert_eq!(tail_family.family().nodes().len(), 2);

    for element in model.aero_elements() {
        let binding = element.polar_binding();
        assert!(
            matches!(binding, RuntimeAeroPolarBinding::ReynoldsFamily { .. }),
            "element {} must use Reynolds family binding",
            element.id()
        );
    }
}

#[test]
fn lt40_aero_surfaces() {
    let model = load_lt40();
    let surfaces = model.aero_surfaces();
    assert_eq!(surfaces.len(), 3);

    let surface_ids: Vec<&str> = surfaces.iter().map(|s| s.id()).collect();
    assert!(surface_ids.contains(&"main-wing"));
    assert!(surface_ids.contains(&"horizontal-tail"));
    assert!(surface_ids.contains(&"vertical-tail"));

    let wing_surface = surfaces.iter().find(|s| s.id() == "main-wing").unwrap();
    assert_eq!(wing_surface.element_indices().len(), 4);
    assert!((wing_surface.span_m() - 1.778).abs() < 1e-10);
    assert!(wing_surface.area_m2() > 0.0);
    assert!(wing_surface.aspect_ratio() > 0.0);
}

#[test]
fn lt40_wing_geometry_matches_evidence() {
    let model = load_lt40();
    let elements = model.aero_elements();

    let wing_left_fixed = elements
        .iter()
        .find(|e| e.id() == "wing-left-fixed")
        .unwrap();
    assert!((wing_left_fixed.element().area_m2() - 0.25805).abs() < 1e-4);
    assert!((wing_left_fixed.element().chord_m() - 0.326).abs() < 1e-3);

    let wing_left_aileron = elements
        .iter()
        .find(|e| e.id() == "wing-left-aileron")
        .unwrap();
    assert!((wing_left_aileron.element().area_m2() - 0.0280).abs() < 1e-4);

    let total_wing_area: f64 = elements
        .iter()
        .filter(|e| e.id().starts_with("wing-"))
        .map(|e| e.element().area_m2())
        .sum();
    assert!((total_wing_area - 0.5721).abs() < 0.001);
}

#[test]
fn lt40_tail_geometry_matches_evidence() {
    let model = load_lt40();
    let elements = model.aero_elements();

    let ht_fixed = elements
        .iter()
        .find(|e| e.id() == "horizontal-tail-fixed")
        .unwrap();
    assert!((ht_fixed.element().area_m2() - 0.10248).abs() < 1e-4);

    let elevator = elements.iter().find(|e| e.id() == "elevator").unwrap();
    assert!((elevator.element().area_m2() - 0.03484).abs() < 1e-4);

    let vt_fixed = elements
        .iter()
        .find(|e| e.id() == "vertical-tail-fixed")
        .unwrap();
    assert!((vt_fixed.element().area_m2() - 0.03778).abs() < 1e-4);

    let rudder = elements.iter().find(|e| e.id() == "rudder").unwrap();
    assert!((rudder.element().area_m2() - 0.01586).abs() < 1e-4);
}

#[test]
fn lt40_control_surface_bindings() {
    let model = load_lt40();
    let bindings = model.control_surface_bindings();
    assert_eq!(bindings.len(), 4);

    let binding_ids: Vec<&str> = bindings.iter().map(|b| b.id()).collect();
    assert!(binding_ids.contains(&"aileron-left"));
    assert!(binding_ids.contains(&"aileron-right"));
    assert!(binding_ids.contains(&"elevator"));
    assert!(binding_ids.contains(&"rudder"));

    let aileron_left = bindings.iter().find(|b| b.id() == "aileron-left").unwrap();
    assert_eq!(aileron_left.actuator(), ControlActuator::Aileron);
    assert!((aileron_left.deflection_gain() - 1.0).abs() < 1e-15);

    let aileron_right = bindings.iter().find(|b| b.id() == "aileron-right").unwrap();
    assert!((aileron_right.deflection_gain() - (-1.0)).abs() < 1e-15);
}

#[test]
fn lt40_propulsion_present() {
    let model = load_lt40();
    let propulsion = model.propulsion().expect("propulsion must be present");
    let config = propulsion.config();

    assert!((config.battery().open_circuit_voltage_v() - 12.6).abs() < 1e-10);
    assert!((config.motor().kv_rpm_per_v() - 1000.0).abs() < 1e-10);
    assert!((config.motor().winding_resistance_ohm() - 0.020).abs() < 1e-10);
    assert!((config.motor().no_load_current_a() - 2.6).abs() < 1e-10);
    assert!((config.propeller().diameter_m() - 0.2794).abs() < 1e-10);

    let coeff_samples = propulsion.coefficient_table().samples();
    assert!(coeff_samples.len() >= 5);
    assert!((coeff_samples[1].advance_ratio_j - 0.0).abs() < 1e-10);
    assert!((coeff_samples[1].ct - 0.1097).abs() < 1e-4);
}

#[test]
fn lt40_downwash_interaction() {
    let model = load_lt40();
    let downwash = model.aero_downwash_interactions();
    assert_eq!(downwash.len(), 1);
    assert_eq!(downwash[0].id(), "wing-to-horizontal-tail");
    assert!((downwash[0].downwash_factor() - 0.6).abs() < 1e-10);
}

#[test]
fn lt40_kinematic_viscosity() {
    let model = load_lt40();
    assert_eq!(model.kinematic_viscosity_m2_s(), Some(0.000015));
}

#[test]
fn lt40_deterministic_fingerprint() {
    let first = load_lt40();
    let second = load_lt40();
    assert_eq!(first.physics_fingerprint(), second.physics_fingerprint());
    assert_eq!(first.physics_fingerprint().as_bytes().len(), 32);
}

#[test]
fn lt40_json_round_trip_deterministic() {
    let json = std::fs::read_to_string(model_path()).expect("read model JSON");
    let first = AircraftModelLoader::from_json_str(&json).expect("first parse must succeed");
    let second = AircraftModelLoader::from_json_str(&json).expect("second parse must succeed");
    assert_eq!(first.physics_fingerprint(), second.physics_fingerprint());
}
