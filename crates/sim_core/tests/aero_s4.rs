use sim_core::{
    AeroElement, AeroElementError, AeroEnvironment, AeroEnvironmentError, BodyWrench, PolarError,
    PolarSample, PolarTable, RigidBodyParams, RigidBodyState, Rk4Integrator, evaluate_aero_element,
    evaluate_derivative,
};
use sim_math::{Mat3, Orientation, Quaternion, Vec3};
use std::f64::consts::{FRAC_PI_2, PI};

const ALGEBRA_TOLERANCE: f64 = 64.0 * f64::EPSILON;
const INTEGRATION_TOLERANCE: f64 = 1.0e-12;

fn state_with_velocity(linear_velocity_world_mps: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn element(position_body_m: Vec3) -> AeroElement {
    AeroElement::new(position_body_m, Orientation::identity(), 2.0, 0.5).unwrap()
}

fn environment(density: f64) -> AeroEnvironment {
    AeroEnvironment::new(density, Vec3::zeros()).unwrap()
}

fn constant_polar(cl: f64, cd: f64, cm: f64) -> PolarTable {
    PolarTable::new(vec![
        PolarSample {
            alpha_rad: -PI,
            cl,
            cd,
            cm,
        },
        PolarSample {
            alpha_rad: PI,
            cl,
            cd,
            cm,
        },
    ])
    .unwrap()
}

fn assert_scalar_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual:e}, expected={expected:e}, tolerance={tolerance:e}"
    );
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance:e}"
    );
}

fn assert_output_finite(output: &sim_core::AeroElementOutput) {
    assert!(
        output
            .air_relative_velocity_element_mps
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(output.section_airspeed_mps.is_finite());
    assert!(output.alpha_rad.is_finite());
    assert!(output.beta_rad.is_finite());
    assert!(output.dynamic_pressure_pa.is_finite());
    assert!(output.coefficients.cl.is_finite());
    assert!(output.coefficients.cd.is_finite());
    assert!(output.coefficients.cm.is_finite());
    assert!(output.force_element_n.iter().all(|value| value.is_finite()));
    assert!(
        output
            .wrench_body
            .force_body_n
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(
        output
            .wrench_body
            .moment_body_nm
            .iter()
            .all(|value| value.is_finite())
    );
}

#[test]
fn a1_zero_airspeed_has_zero_finite_wrench() {
    let output = evaluate_aero_element(
        &state_with_velocity(Vec3::zeros()),
        &element(Vec3::zeros()),
        &environment(1.225),
        &constant_polar(1.0, 0.1, 0.2),
    );
    assert_eq!(output.force_element_n, Vec3::zeros());
    assert_eq!(output.wrench_body, BodyWrench::zero());
    assert_output_finite(&output);
}

#[test]
fn a2_zero_density_has_zero_wrench_at_nonzero_speed() {
    let output = evaluate_aero_element(
        &state_with_velocity(Vec3::new(20.0, 0.0, 3.0)),
        &element(Vec3::zeros()),
        &environment(0.0),
        &constant_polar(1.0, 0.1, 0.2),
    );
    assert!(output.section_airspeed_mps > 0.0);
    assert_eq!(output.dynamic_pressure_pa, 0.0);
    assert_eq!(output.wrench_body, BodyWrench::zero());
}

#[test]
fn a3_local_point_velocity_includes_omega_cross_r() {
    let mut state = state_with_velocity(Vec3::new(1.0, 2.0, 3.0));
    state.angular_velocity_body_radps = Vec3::new(0.0, 0.0, 2.0);
    let output = evaluate_aero_element(
        &state,
        &element(Vec3::new(1.0, 0.0, 0.0)),
        &environment(1.0),
        &constant_polar(0.0, 0.0, 0.0),
    );
    assert_vec_close(
        output.air_relative_velocity_element_mps,
        Vec3::new(1.0, 4.0, 3.0),
        ALGEBRA_TOLERANCE,
    );
}

#[test]
fn a4_world_body_and_element_frame_transforms_are_explicit() {
    let orientation_world_from_body = Orientation::from_axis_angle(&Vec3::z_axis(), FRAC_PI_2);
    let orientation_body_from_element = Orientation::from_axis_angle(&Vec3::x_axis(), -FRAC_PI_2);
    let expected_body_velocity = Vec3::new(10.0, 2.0, 3.0);
    let wind_world = Vec3::new(-2.0, 1.0, 0.5);
    let mut state = state_with_velocity(
        orientation_world_from_body.transform_vector(&expected_body_velocity) + wind_world,
    );
    state.orientation_world_from_body = orientation_world_from_body;
    let aero_element =
        AeroElement::new(Vec3::zeros(), orientation_body_from_element, 1.0, 0.5).unwrap();
    let output = evaluate_aero_element(
        &state,
        &aero_element,
        &AeroEnvironment::new(1.0, wind_world).unwrap(),
        &constant_polar(0.0, 0.0, 0.0),
    );
    let expected_element_velocity =
        orientation_body_from_element.inverse_transform_vector(&expected_body_velocity);
    assert_vec_close(
        output.air_relative_velocity_element_mps,
        expected_element_velocity,
        2.0e-14,
    );
}

#[test]
fn a5_alpha_sign_follows_positive_element_down_velocity() {
    let aero_element = element(Vec3::zeros());
    let environment = environment(1.0);
    let polar = constant_polar(0.0, 0.0, 0.0);
    let positive = evaluate_aero_element(
        &state_with_velocity(Vec3::new(10.0, 0.0, 2.0)),
        &aero_element,
        &environment,
        &polar,
    );
    let negative = evaluate_aero_element(
        &state_with_velocity(Vec3::new(10.0, 0.0, -2.0)),
        &aero_element,
        &environment,
        &polar,
    );
    assert!(positive.alpha_rad > 0.0);
    assert!(negative.alpha_rad < 0.0);
}

#[test]
fn a6_polar_exact_sample_is_preserved() {
    let table = test_polar();
    assert_eq!(
        table.sample_clamped(0.0),
        sim_core::PolarCoefficients {
            cl: 0.25,
            cd: 0.04,
            cm: -0.02,
        }
    );
}

#[test]
fn a7_polar_midpoint_is_linearly_interpolated() {
    let coefficients = test_polar().sample_clamped(0.5);
    assert_eq!(
        coefficients,
        sim_core::PolarCoefficients {
            cl: 0.625,
            cd: 0.07,
            cm: -0.06,
        }
    );
}

#[test]
fn a8_polar_clamps_beyond_both_endpoints() {
    let table = test_polar();
    assert_eq!(table.sample_clamped(-100.0), table.samples()[0].into());
    assert_eq!(table.sample_clamped(100.0), table.samples()[2].into());
}

#[test]
fn a9_invalid_polars_are_rejected() {
    assert_eq!(PolarTable::new(Vec::new()), Err(PolarError::TooFewSamples));
    let sample = PolarSample {
        alpha_rad: 0.0,
        cl: 0.0,
        cd: 0.1,
        cm: 0.0,
    };
    assert_eq!(
        PolarTable::new(vec![sample, sample]),
        Err(PolarError::NonIncreasingAlpha { index: 1 })
    );
    assert_eq!(
        PolarTable::new(vec![
            sample,
            PolarSample {
                alpha_rad: -1.0,
                ..sample
            },
        ]),
        Err(PolarError::NonIncreasingAlpha { index: 1 })
    );
    assert_eq!(
        PolarTable::new(vec![
            sample,
            PolarSample {
                alpha_rad: 1.0,
                cl: f64::NAN,
                ..sample
            },
        ]),
        Err(PolarError::NonFiniteSample { index: 1 })
    );
    assert_eq!(
        PolarTable::new(vec![
            sample,
            PolarSample {
                alpha_rad: 1.0,
                cm: f64::INFINITY,
                ..sample
            },
        ]),
        Err(PolarError::NonFiniteSample { index: 1 })
    );
    assert_eq!(
        PolarTable::new(vec![
            sample,
            PolarSample {
                alpha_rad: 1.0,
                cd: -0.1,
                ..sample
            },
        ]),
        Err(PolarError::NegativeDragCoefficient { index: 1 })
    );
}

#[test]
fn a10_drag_opposes_forward_section_velocity() {
    let output = evaluate_aero_element(
        &state_with_velocity(Vec3::new(10.0, 0.0, 2.0)),
        &element(Vec3::zeros()),
        &environment(1.0),
        &constant_polar(0.0, 0.2, 0.0),
    );
    let section_velocity = Vec3::new(10.0, 0.0, 2.0);
    assert!(output.force_element_n.x < 0.0);
    assert!(output.force_element_n.dot(&section_velocity) <= 0.0);
}

#[test]
fn a11_positive_lift_points_toward_element_up() {
    let output = evaluate_aero_element(
        &state_with_velocity(Vec3::new(10.0, 0.0, 0.0)),
        &element(Vec3::zeros()),
        &environment(1.0),
        &constant_polar(1.0, 0.0, 0.0),
    );
    assert_eq!(output.force_element_n, Vec3::new(0.0, 0.0, -100.0));
}

#[test]
fn a12_lift_is_perpendicular_to_section_velocity() {
    let velocity = Vec3::new(12.0, 0.0, 5.0);
    let output = evaluate_aero_element(
        &state_with_velocity(velocity),
        &element(Vec3::zeros()),
        &environment(1.0),
        &constant_polar(1.0, 0.0, 0.0),
    );
    assert_scalar_close(
        output.force_element_n.dot(&velocity),
        0.0,
        INTEGRATION_TOLERANCE,
    );
}

#[test]
fn a13_force_at_body_point_uses_right_handed_moment_arm() {
    let mut wrench = BodyWrench::zero();
    wrench.add_force_at_body_point(Vec3::new(0.0, 0.0, -10.0), Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(wrench.force_body_n, Vec3::new(0.0, 0.0, -10.0));
    assert_eq!(wrench.moment_body_nm, Vec3::new(0.0, 10.0, 0.0));
}

#[test]
fn a14_intrinsic_cm_has_expected_positive_and_negative_y_sign() {
    for cm in [0.2, -0.2] {
        let output = evaluate_aero_element(
            &state_with_velocity(Vec3::new(10.0, 0.0, 0.0)),
            &element(Vec3::zeros()),
            &environment(1.0),
            &constant_polar(0.0, 0.0, cm),
        );
        assert_eq!(
            output.wrench_body.moment_body_nm,
            Vec3::new(0.0, 10.0 * cm / 0.2, 0.0)
        );
    }
}

#[test]
fn a15_complete_identity_frame_case_matches_manual_solution() {
    let output = evaluate_aero_element(
        &state_with_velocity(Vec3::new(10.0, 0.0, 0.0)),
        &element(Vec3::new(1.0, 0.0, 0.0)),
        &environment(1.2),
        &constant_polar(0.5, 0.1, -0.2),
    );
    assert_eq!(output.dynamic_pressure_pa, 60.0);
    assert_eq!(output.alpha_rad, 0.0);
    assert_eq!(output.force_element_n, Vec3::new(-12.0, 0.0, -60.0));
    assert_eq!(output.wrench_body.force_body_n, output.force_element_n);
    assert_eq!(output.wrench_body.moment_body_nm, Vec3::new(0.0, 48.0, 0.0));
}

#[test]
fn a16_rotational_local_velocity_changes_alpha_in_expected_direction() {
    let mut positive_state = state_with_velocity(Vec3::new(10.0, 0.0, 0.0));
    positive_state.angular_velocity_body_radps = Vec3::new(1.0, 0.0, 0.0);
    let mut negative_state = positive_state;
    negative_state.angular_velocity_body_radps.x = -1.0;
    let aero_element = element(Vec3::new(0.0, 1.0, 0.0));
    let environment = environment(1.0);
    let polar = constant_polar(0.0, 0.0, 0.0);
    assert!(
        evaluate_aero_element(&positive_state, &aero_element, &environment, &polar).alpha_rad > 0.0
    );
    assert!(
        evaluate_aero_element(&negative_state, &aero_element, &environment, &polar).alpha_rad < 0.0
    );
}

#[test]
fn a17_real_aero_evaluator_recomputes_wrench_at_all_rk4_stages() {
    let initial = state_with_velocity(Vec3::new(20.0, 0.0, 0.0));
    let body_params = RigidBodyParams::new(1.0, Mat3::identity()).unwrap();
    let aero_element = AeroElement::new(Vec3::zeros(), Orientation::identity(), 1.0, 1.0).unwrap();
    let environment = environment(1.0);
    let polar = constant_polar(0.0, 0.5, 0.0);
    let gravity = Vec3::zeros();
    let mut stage_force_x = [0.0; 4];
    let mut stage_velocity_x = [0.0; 4];
    let mut calls = 0;

    let final_state = Rk4Integrator::step(&initial, 0.1, |stage_state| {
        let aero = evaluate_aero_element(stage_state, &aero_element, &environment, &polar);
        stage_velocity_x[calls] = stage_state.linear_velocity_world_mps.x;
        stage_force_x[calls] = aero.wrench_body.force_body_n.x;
        calls += 1;
        evaluate_derivative(stage_state, &body_params, &aero.wrench_body, &gravity)
    });

    assert_eq!(calls, 4);
    assert!(stage_velocity_x.windows(2).all(|pair| pair[0] != pair[1]));
    assert!(stage_force_x.windows(2).all(|pair| pair[0] != pair[1]));
    assert!(final_state.linear_velocity_world_mps.x < initial.linear_velocity_world_mps.x);
}

#[test]
fn a18_representative_grid_outputs_are_finite() {
    let aero_element = AeroElement::new(
        Vec3::new(0.3, 0.8, -0.2),
        Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.05)),
        0.7,
        0.25,
    )
    .unwrap();
    let environment = AeroEnvironment::new(1.225, Vec3::new(2.0, -1.0, 0.5)).unwrap();
    let polar = test_polar();
    for alpha_rad in [-PI, -1.0, -0.1, 0.0, 0.2, 1.0, PI] {
        for speed_mps in [0.0, 1.0e-12, 0.1, 10.0, 80.0] {
            for angular_rate_radps in [-5.0, 0.0, 7.0] {
                let mut state = state_with_velocity(Vec3::new(
                    speed_mps * alpha_rad.cos(),
                    0.3 * speed_mps,
                    speed_mps * alpha_rad.sin(),
                ));
                state.angular_velocity_body_radps = Vec3::new(angular_rate_radps, -0.2, 0.4);
                let output = evaluate_aero_element(&state, &aero_element, &environment, &polar);
                assert_output_finite(&output);
            }
        }
    }
}

#[test]
fn drag_only_aerodynamics_never_adds_local_kinetic_power() {
    let aero_element = AeroElement::new(
        Vec3::new(0.2, 0.7, -0.1),
        Orientation::from_scaled_axis(Vec3::new(0.2, -0.1, 0.05)),
        1.2,
        0.4,
    )
    .unwrap();
    let environment = environment(1.225);
    let polar = constant_polar(0.0, 0.15, 0.0);
    for velocity in [
        Vec3::new(20.0, 0.0, 0.0),
        Vec3::new(12.0, 5.0, 4.0),
        Vec3::new(-8.0, -3.0, 2.0),
    ] {
        let mut state = state_with_velocity(velocity);
        state.angular_velocity_body_radps = Vec3::new(0.5, -0.3, 0.2);
        let output = evaluate_aero_element(&state, &aero_element, &environment, &polar);
        let local_velocity_body = aero_element
            .orientation_body_from_element()
            .transform_vector(&output.air_relative_velocity_element_mps);
        let power_w = output.wrench_body.force_body_n.dot(&local_velocity_body);
        assert!(power_w <= ALGEBRA_TOLERANCE, "drag power={power_w:e}");
    }
}

#[test]
fn aero_configuration_validation_rejects_invalid_values() {
    assert_eq!(
        AeroElement::new(
            Vec3::new(f64::NAN, 0.0, 0.0),
            Orientation::identity(),
            1.0,
            1.0,
        ),
        Err(AeroElementError::NonFinitePosition)
    );
    assert_eq!(
        AeroElement::new(
            Vec3::zeros(),
            Orientation::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 0.0)),
            1.0,
            1.0,
        ),
        Err(AeroElementError::InvalidOrientation)
    );
    assert_eq!(
        AeroElement::new(Vec3::zeros(), Orientation::identity(), 0.0, 1.0),
        Err(AeroElementError::InvalidArea)
    );
    assert_eq!(
        AeroElement::new(Vec3::zeros(), Orientation::identity(), 1.0, -1.0),
        Err(AeroElementError::InvalidChord)
    );
    assert_eq!(
        AeroEnvironment::new(-1.0, Vec3::zeros()),
        Err(AeroEnvironmentError::InvalidAirDensity)
    );
    assert_eq!(
        AeroEnvironment::new(1.0, Vec3::new(0.0, f64::INFINITY, 0.0)),
        Err(AeroEnvironmentError::NonFiniteWind)
    );
}

#[test]
fn aero_evaluation_allocates_nothing_after_initialization() {
    let state = state_with_velocity(Vec3::new(20.0, 1.0, 3.0));
    let aero_element = element(Vec3::new(0.2, 0.7, -0.1));
    let environment = environment(1.225);
    let polar = test_polar();
    let mut checksum = 0.0;
    let allocation_info = allocation_counter::measure(|| {
        for _ in 0..100 {
            checksum += evaluate_aero_element(&state, &aero_element, &environment, &polar)
                .force_element_n
                .x;
        }
    });
    assert!(checksum.is_finite());
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}

#[test]
fn aero_rk4_step_allocates_nothing_after_initialization() {
    let mut state = state_with_velocity(Vec3::new(20.0, 0.0, 2.0));
    let body_params = RigidBodyParams::new(2.0, Mat3::identity()).unwrap();
    let aero_element = element(Vec3::new(0.2, 0.7, -0.1));
    let environment = environment(1.225);
    let polar = test_polar();
    let gravity = Vec3::zeros();
    let allocation_info = allocation_counter::measure(|| {
        state = Rk4Integrator::step(&state, 0.002, |stage_state| {
            let aero = evaluate_aero_element(stage_state, &aero_element, &environment, &polar);
            evaluate_derivative(stage_state, &body_params, &aero.wrench_body, &gravity)
        });
    });
    assert!(state.validate().is_ok());
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}

fn test_polar() -> PolarTable {
    PolarTable::new(vec![
        PolarSample {
            alpha_rad: -1.0,
            cl: -0.5,
            cd: 0.02,
            cm: 0.1,
        },
        PolarSample {
            alpha_rad: 0.0,
            cl: 0.25,
            cd: 0.04,
            cm: -0.02,
        },
        PolarSample {
            alpha_rad: 1.0,
            cl: 1.0,
            cd: 0.10,
            cm: -0.10,
        },
    ])
    .unwrap()
}
