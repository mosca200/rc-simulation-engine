mod common;

use common::{load_value, valid_model_value};
use model::{AircraftClassification, ModelLoadError, load_aircraft_model};
use sim_core::PropellerSpinDirection;
use sim_math::Vec3;
use std::path::Path;

fn assert_f64_bits(actual: f64, expected: f64) {
    assert_eq!(actual.to_bits(), expected.to_bits());
}

#[test]
fn parsed_runtime_model_exactly_matches_declared_semantics() {
    let model = load_value(&valid_model_value()).expect("valid semantic fixture");

    assert_eq!(model.schema_version(), 0);
    assert_eq!(model.model_id(), "test-aircraft_01");
    assert_eq!(model.display_name(), "Test Aircraft");
    assert_eq!(
        model.classification(),
        AircraftClassification::SyntheticTest
    );
    assert!(model.reference_aircraft().is_none());

    let rigid_body = model.rigid_body();
    assert_f64_bits(rigid_body.mass_kg(), 2.5);
    let expected_inertia = [
        [0.12, 0.01, -0.002],
        [0.01, 0.15, 0.003],
        [-0.002, 0.003, 0.20],
    ];
    for (row, expected_row) in expected_inertia.iter().enumerate() {
        for (column, expected) in expected_row.iter().enumerate() {
            assert_f64_bits(rigid_body.inertia_body_kg_m2()[(row, column)], *expected);
        }
    }

    let polars = model.aero_polars();
    assert_eq!(polars.len(), 2);
    assert_eq!(polars[0].id(), "polar-first");
    assert_eq!(polars[1].id(), "polar-second");
    let first_samples = polars[0].table().samples();
    assert_eq!(first_samples.len(), 3);
    for (sample, expected) in first_samples.iter().zip([
        [-0.20, -0.70, 0.080, 0.030],
        [0.00, 0.10, 0.020, 0.010],
        [0.25, 1.00, 0.110, -0.040],
    ]) {
        for (actual, expected) in [sample.alpha_rad, sample.cl, sample.cd, sample.cm]
            .into_iter()
            .zip(expected)
        {
            assert_f64_bits(actual, expected);
        }
    }

    let elements = model.aero_elements();
    assert_eq!(elements.len(), 2);
    assert_eq!(elements[0].id(), "element-first");
    assert_eq!(elements[0].polar_index(), 1);
    let first_element = elements[0].element();
    assert_eq!(
        *first_element.position_body_m(),
        Vec3::new(0.35, -0.42, 0.08)
    );
    let orientation = first_element.orientation_body_from_element().quaternion();
    for (actual, expected) in [orientation.w, orientation.i, orientation.j, orientation.k]
        .into_iter()
        .zip([0.5, 0.5, 0.5, 0.5])
    {
        assert_f64_bits(actual, expected);
    }
    assert_f64_bits(first_element.area_m2(), 0.31);
    assert_f64_bits(first_element.chord_m(), 0.19);
    assert_eq!(elements[1].id(), "element-second");
    assert_eq!(elements[1].polar_index(), 0);
    assert_eq!(
        *elements[1].element().position_body_m(),
        Vec3::new(-0.22, 0.37, -0.04)
    );

    let response = model.controls().response();
    for (axis, expected) in [response.roll(), response.pitch(), response.yaw()]
        .into_iter()
        .zip([(0.80, 0.10), (0.70, 0.20), (0.60, 0.30)])
    {
        assert_f64_bits(axis.rate(), expected.0);
        assert_f64_bits(axis.expo(), expected.1);
    }
    let actuators = model.controls().actuators();
    let expected_servos = [
        (-0.40, 0.01, 0.50, 4.0, false),
        (-0.30, -0.02, 0.45, 3.5, true),
        (-0.50, 0.0, 0.55, 2.5, false),
    ];
    for (servo, expected) in [
        actuators.aileron(),
        actuators.elevator(),
        actuators.rudder(),
    ]
    .into_iter()
    .zip(expected_servos)
    {
        assert_f64_bits(servo.min_angle_rad(), expected.0);
        assert_f64_bits(servo.neutral_angle_rad(), expected.1);
        assert_f64_bits(servo.max_angle_rad(), expected.2);
        assert_f64_bits(servo.max_speed_rad_s(), expected.3);
        assert_eq!(servo.reversed(), expected.4);
    }

    let propulsion = model.propulsion().expect("declared propulsion");
    let config = propulsion.config();
    assert_f64_bits(config.battery().open_circuit_voltage_v(), 14.8);
    assert_f64_bits(config.battery().internal_resistance_ohm(), 0.025);
    assert_f64_bits(config.motor().kv_rpm_per_v(), 920.0);
    assert_f64_bits(config.motor().winding_resistance_ohm(), 0.041);
    assert_f64_bits(config.motor().no_load_current_a(), 1.2);
    let propeller = config.propeller();
    assert_eq!(*propeller.position_body_m(), Vec3::new(0.30, 0.01, -0.02));
    let orientation = propeller.orientation_body_from_prop().quaternion();
    for (actual, expected) in [orientation.w, orientation.i, orientation.j, orientation.k]
        .into_iter()
        .zip([0.0, 1.0, 0.0, 0.0])
    {
        assert_f64_bits(actual, expected);
    }
    assert_f64_bits(propeller.diameter_m(), 0.33);
    assert_eq!(
        propeller.spin_direction(),
        PropellerSpinDirection::NegativeAboutLocalX
    );
    let coefficient_samples = propulsion.coefficient_table().samples();
    assert_eq!(coefficient_samples.len(), 3);
    for (sample, expected) in
        coefficient_samples
            .iter()
            .zip([[0.0, 0.12, 0.018], [0.5, 0.08, 0.012], [1.0, 0.02, 0.005]])
    {
        for (actual, expected) in [sample.advance_ratio_j, sample.ct, sample.cq]
            .into_iter()
            .zip(expected)
        {
            assert_f64_bits(actual, expected);
        }
    }

    assert_eq!(
        model.presentation().expect("presentation").glb_path(),
        "assets/test-aircraft.glb"
    );
}

#[test]
fn acro_electric_01_repository_model_smoke_test() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("models/acro_electric_01/model.json");
    let first = load_aircraft_model(&path).expect("repository Acro Electric 01 must remain valid");
    let second = load_aircraft_model(&path).expect("second deterministic model load");

    assert_eq!(first.schema_version(), 2);
    assert_eq!(
        first.classification(),
        AircraftClassification::SyntheticTest
    );
    assert!(first.reference_aircraft().is_none());
    assert!(!first.aero_polars().is_empty());
    assert_eq!(first.aero_elements().len(), 8);
    assert_eq!(first.control_surface_bindings().len(), 4);
    assert!(first.controls().response().roll().rate().is_finite());
    assert!(first.propulsion().is_some());
    assert_eq!(first.physics_fingerprint(), second.physics_fingerprint());
    assert_eq!(first.physics_fingerprint().as_bytes().len(), 32);
}

#[test]
fn filesystem_helper_reports_io_error_with_path_context() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("definitely-missing-s6-model.json");

    let error = load_aircraft_model(&path).expect_err("missing model must fail");
    match error {
        ModelLoadError::Io {
            path: reported_path,
            ..
        } => assert_eq!(reported_path, path.display().to_string()),
        other => panic!("unexpected error: {other}"),
    }
}
