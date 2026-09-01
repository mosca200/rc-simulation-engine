use sim_core::{
    AeroElement, AeroEnvironment, PolarSample, PolarTable, ReynoldsCalculationError, ReynoldsPolar,
    ReynoldsPolarFamily, ReynoldsRangeStatus, RigidBodyState, calculate_reynolds_number,
    evaluate_aero_element, evaluate_reynolds_aero_element,
};
use sim_math::{Orientation, Vec3};
use std::f64::consts::FRAC_PI_2;

const NU: f64 = 0.0001;

fn table(cl: f64, cd: f64, cm: f64) -> PolarTable {
    PolarTable::new(vec![
        PolarSample {
            alpha_rad: -1.0,
            cl,
            cd,
            cm,
        },
        PolarSample {
            alpha_rad: 1.0,
            cl,
            cd,
            cm,
        },
    ])
    .unwrap()
}

fn family() -> ReynoldsPolarFamily {
    ReynoldsPolarFamily::new(vec![
        ReynoldsPolar::new(100_000.0, table(1.0, 0.02, 0.1)).unwrap(),
        ReynoldsPolar::new(400_000.0, table(3.0, 0.10, -0.3)).unwrap(),
    ])
    .unwrap()
}

fn element(position: Vec3, orientation: Orientation, chord_m: f64) -> AeroElement {
    AeroElement::new(position, orientation, 1.0, chord_m).unwrap()
}

fn state(linear_velocity_world_mps: Vec3, angular_velocity_body_radps: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps,
    }
}

fn environment() -> AeroEnvironment {
    AeroEnvironment::new(1.2, Vec3::zeros()).unwrap()
}

#[test]
fn m2_3c_03_nonfinite_viscosity_is_rejected() {
    for viscosity in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            calculate_reynolds_number(20.0, 0.5, viscosity),
            Err(ReynoldsCalculationError::InvalidKinematicViscosity)
        );
    }
}

#[test]
fn m2_3c_04_reynolds_equation_is_correct() {
    assert_eq!(calculate_reynolds_number(40.0, 0.5, NU), Ok(200_000.0));
    for speed in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            calculate_reynolds_number(speed, 0.5, NU),
            Err(ReynoldsCalculationError::InvalidSectionAirspeed)
        );
    }
    for chord in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            calculate_reynolds_number(40.0, chord, NU),
            Err(ReynoldsCalculationError::InvalidChord)
        );
    }
    assert_eq!(
        calculate_reynolds_number(f64::MAX, f64::MAX, f64::MIN_POSITIVE),
        Err(ReynoldsCalculationError::InvalidResult)
    );
}

#[test]
fn m2_3c_05_zero_speed_has_zero_reynolds_and_zero_wrench() {
    let family = family();
    let output = evaluate_reynolds_aero_element(
        &state(Vec3::zeros(), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(output.local_reynolds, 0.0);
    assert_eq!(
        output.reynolds_sample.range_status,
        ReynoldsRangeStatus::BelowRange
    );
    assert_eq!(output.aero.section_airspeed_mps, 0.0);
    assert_eq!(output.aero.wrench_body.force_body_n, Vec3::zeros());
    assert_eq!(output.aero.wrench_body.moment_body_nm, Vec3::zeros());
}

#[test]
fn m2_3c_06_exact_reynolds_node_is_preserved() {
    let family = family();
    let output = evaluate_reynolds_aero_element(
        &state(Vec3::new(20.0, 0.0, 0.0), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(output.local_reynolds, 100_000.0);
    assert_eq!(
        output.aero.coefficients,
        family.nodes()[0].table().sample_clamped(0.0)
    );
    assert_eq!(output.reynolds_sample.interpolation_fraction, 0.0);
}

#[test]
fn m2_3c_07_between_node_sampling_uses_local_reynolds() {
    let family = family();
    let output = evaluate_reynolds_aero_element(
        &state(Vec3::new(40.0, 0.0, 0.0), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(output.local_reynolds, 200_000.0);
    assert!((output.reynolds_sample.interpolation_fraction - 0.5).abs() < 1.0e-15);
    assert_eq!(output.aero.coefficients.cl, 2.0);
}

#[test]
fn m2_3c_08_below_family_range_is_diagnostic() {
    let family = family();
    let output = evaluate_reynolds_aero_element(
        &state(Vec3::new(10.0, 0.0, 0.0), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(output.local_reynolds, 50_000.0);
    assert_eq!(
        output.reynolds_sample.range_status,
        ReynoldsRangeStatus::BelowRange
    );
    assert_eq!(
        output.reynolds_sample.lower_reynolds.reynolds_number(),
        100_000.0
    );
    assert_eq!(
        output.reynolds_sample.upper_reynolds.reynolds_number(),
        100_000.0
    );
}

#[test]
fn m2_3c_09_above_family_range_is_diagnostic() {
    let family = family();
    let output = evaluate_reynolds_aero_element(
        &state(Vec3::new(100.0, 0.0, 0.0), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(output.local_reynolds, 500_000.0);
    assert_eq!(
        output.reynolds_sample.range_status,
        ReynoldsRangeStatus::AboveRange
    );
    assert_eq!(
        output.reynolds_sample.lower_reynolds.reynolds_number(),
        400_000.0
    );
    assert_eq!(
        output.reynolds_sample.upper_reynolds.reynolds_number(),
        400_000.0
    );
}

#[test]
fn m2_3c_10_local_chord_changes_reynolds() {
    let state = state(Vec3::new(20.0, 0.0, 0.0), Vec3::zeros());
    let family = family();
    let short = evaluate_reynolds_aero_element(
        &state,
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    let long = evaluate_reynolds_aero_element(
        &state,
        &element(Vec3::zeros(), Orientation::identity(), 1.0),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(long.local_reynolds, 2.0 * short.local_reynolds);
}

#[test]
fn m2_3c_11_rotational_local_velocity_changes_reynolds() {
    let body_state = state(Vec3::new(20.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -10.0));
    let family = family();
    let at_cg = evaluate_reynolds_aero_element(
        &body_state,
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    let offset = evaluate_reynolds_aero_element(
        &body_state,
        &element(Vec3::new(0.0, 1.0, 0.0), Orientation::identity(), 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(at_cg.aero.section_airspeed_mps, 20.0);
    assert_eq!(offset.aero.section_airspeed_mps, 30.0);
    assert_eq!(offset.local_reynolds, 1.5 * at_cg.local_reynolds);
}

#[test]
fn m2_3c_12_local_element_orientation_path_remains_correct() {
    let orientation = Orientation::from_axis_angle(&Vec3::y_axis(), FRAC_PI_2);
    let body_state = state(Vec3::new(20.0, 0.0, 0.0), Vec3::zeros());
    let family = family();
    let expected_element_velocity =
        orientation.inverse_transform_vector(&Vec3::new(20.0, 0.0, 0.0));
    let output = evaluate_reynolds_aero_element(
        &body_state,
        &element(Vec3::zeros(), orientation, 0.5),
        &environment(),
        &family,
        NU,
    );
    assert_eq!(
        output.aero.air_relative_velocity_element_mps,
        expected_element_velocity
    );
    assert!((output.aero.section_airspeed_mps - 20.0).abs() < 1.0e-14);
    assert!((output.local_reynolds - 100_000.0).abs() < 1.0e-10);
}

#[test]
fn m2_3c_14_reynolds_aero_hot_path_allocates_nothing() {
    let body_state = state(Vec3::new(40.0, 0.0, 2.0), Vec3::new(0.0, 0.1, 0.0));
    let element = element(Vec3::new(-0.2, 0.5, 0.0), Orientation::identity(), 0.5);
    let environment = environment();
    let family = family();
    let mut checksum = 0.0;
    let allocation_info = allocation_counter::measure(|| {
        for _ in 0..1_000 {
            let output =
                evaluate_reynolds_aero_element(&body_state, &element, &environment, &family, NU);
            checksum += output.local_reynolds + output.aero.wrench_body.force_body_n.x;
        }
    });
    assert!(checksum.is_finite());
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}

#[test]
fn m2_3c_15_legacy_fixed_polar_path_is_unchanged() {
    let output = evaluate_aero_element(
        &state(Vec3::new(20.0, 0.0, 0.0), Vec3::zeros()),
        &element(Vec3::zeros(), Orientation::identity(), 0.5),
        &environment(),
        &table(1.0, 0.1, -0.2),
    );
    assert_eq!(output.dynamic_pressure_pa, 240.0);
    assert_eq!(output.force_element_n, Vec3::new(-24.0, 0.0, -240.0));
    assert_eq!(output.wrench_body.force_body_n, output.force_element_n);
    assert_eq!(
        output.wrench_body.moment_body_nm,
        Vec3::new(0.0, -24.0, 0.0)
    );
}
