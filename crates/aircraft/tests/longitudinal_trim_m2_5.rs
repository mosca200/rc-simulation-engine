use aircraft::{
    AircraftSimulationConfig, LongitudinalTrimFailureReason, LongitudinalTrimRequest,
    LongitudinalTrimRequestError, LongitudinalTrimTolerances, LongitudinalTrimVariables,
    TrimBounds, effective_aero_elements_for_positions, evaluate_aircraft_aero_element,
    evaluate_longitudinal_trim_candidate, solve_longitudinal_trim,
};
use model::{AircraftClassification, AircraftModel, AircraftModelLoader};
use sim_core::AeroEnvironment;
use sim_math::Vec3;

const FIXTURE: &str = include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");

fn model() -> AircraftModel {
    AircraftModelLoader::from_json_str(FIXTURE).unwrap()
}

fn model_with_esc_resistance(resistance: f64) -> AircraftModel {
    let changed = FIXTURE.replace(
        "\"series_resistance_ohm\": 0.010",
        &format!("\"series_resistance_ohm\": {resistance}"),
    );
    assert_ne!(changed, FIXTURE);
    AircraftModelLoader::from_json_str(&changed).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        Vec3::new(0.0, 0.0, 9.80665),
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn request(speed_mps: f64) -> LongitudinalTrimRequest {
    LongitudinalTrimRequest::new(
        speed_mps,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
        40,
    )
    .unwrap()
}

#[test]
fn request_validation_fails_closed_and_initial_guess_is_clamped() {
    for (lower, upper) in [
        (f64::NAN, 1.0),
        (0.0, f64::INFINITY),
        (1.0, 0.0),
        (1.0, 1.0),
    ] {
        assert_eq!(
            TrimBounds::new(lower, upper),
            Err(LongitudinalTrimRequestError::InvalidBounds)
        );
    }
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for variables in [
            LongitudinalTrimVariables::new(value, 0.0, 0.5),
            LongitudinalTrimVariables::new(0.0, value, 0.5),
            LongitudinalTrimVariables::new(0.0, 0.0, value),
        ] {
            assert_eq!(
                variables,
                Err(LongitudinalTrimRequestError::NonFiniteInitialGuess)
            );
        }
        assert_eq!(
            LongitudinalTrimTolerances::new(value, 0.1),
            Err(LongitudinalTrimRequestError::InvalidTolerance)
        );
        assert_eq!(
            LongitudinalTrimTolerances::new(0.1, value),
            Err(LongitudinalTrimRequestError::InvalidTolerance)
        );
    }
    for value in [0.0, -1.0] {
        assert_eq!(
            LongitudinalTrimTolerances::new(value, 0.1),
            Err(LongitudinalTrimRequestError::InvalidTolerance)
        );
    }

    let alpha = TrimBounds::new(-0.2, 0.3).unwrap();
    let elevator = TrimBounds::new(-0.8, 0.8).unwrap();
    let throttle = TrimBounds::new(0.0, 1.0).unwrap();
    let guess = LongitudinalTrimVariables::new(1.0, -1.0, 2.0).unwrap();
    let tolerance = LongitudinalTrimTolerances::new(0.01, 0.001).unwrap();
    for speed in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            LongitudinalTrimRequest::new(speed, alpha, elevator, throttle, guess, tolerance, 20),
            Err(LongitudinalTrimRequestError::InvalidTargetAirspeed)
        );
    }
    assert_eq!(
        LongitudinalTrimRequest::new(
            18.0,
            alpha,
            TrimBounds::new(-1.1, 0.8).unwrap(),
            throttle,
            guess,
            tolerance,
            20
        ),
        Err(LongitudinalTrimRequestError::InvalidElevatorBounds)
    );
    assert_eq!(
        LongitudinalTrimRequest::new(
            18.0,
            alpha,
            TrimBounds::new(-0.8, 1.1).unwrap(),
            throttle,
            guess,
            tolerance,
            20
        ),
        Err(LongitudinalTrimRequestError::InvalidElevatorBounds)
    );
    assert_eq!(
        LongitudinalTrimRequest::new(
            18.0,
            alpha,
            elevator,
            TrimBounds::new(-0.1, 0.8).unwrap(),
            guess,
            tolerance,
            20
        ),
        Err(LongitudinalTrimRequestError::InvalidThrottleBounds)
    );
    assert_eq!(
        LongitudinalTrimRequest::new(
            18.0,
            alpha,
            elevator,
            TrimBounds::new(0.1, 1.1).unwrap(),
            guess,
            tolerance,
            20
        ),
        Err(LongitudinalTrimRequestError::InvalidThrottleBounds)
    );
    assert_eq!(
        LongitudinalTrimRequest::new(18.0, alpha, elevator, throttle, guess, tolerance, 0),
        Err(LongitudinalTrimRequestError::InvalidIterationLimit)
    );
    let valid = LongitudinalTrimRequest::new(18.0, alpha, elevator, throttle, guess, tolerance, 20)
        .unwrap();
    assert_eq!(valid.initial_guess().alpha_rad, 0.3);
    assert_eq!(valid.initial_guess().elevator_command, -0.8);
    assert_eq!(valid.initial_guess().throttle, 1.0);
}

#[test]
fn synthetic_trim_converges_is_deterministic_and_re_evaluates_through_runtime_physics() {
    let model = model();
    let config = config();
    let request = request(18.0);
    let first = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let second = solve_longitudinal_trim(&model, &config, &request).unwrap();
    assert_eq!(first, second);
    assert!(first.evaluation.residuals.is_within(request.tolerances()));
    assert!(request.alpha_bounds_rad().lower() <= first.evaluation.variables.alpha_rad);
    assert!(first.evaluation.variables.alpha_rad <= request.alpha_bounds_rad().upper());
    assert!(request.elevator_bounds().lower() <= first.evaluation.variables.elevator_command);
    assert!(first.evaluation.variables.elevator_command <= request.elevator_bounds().upper());
    assert!(request.throttle_bounds().lower() <= first.evaluation.variables.throttle);
    assert!(first.evaluation.variables.throttle <= request.throttle_bounds().upper());

    let independent =
        evaluate_longitudinal_trim_candidate(&model, &config, &request, first.evaluation.variables)
            .unwrap();
    assert_eq!(independent, first.evaluation);
    assert!(independent.residuals.is_within(request.tolerances()));
}

#[test]
fn elevator_and_throttle_perturbations_are_physically_connected() {
    let model = model();
    let config = config();
    let request = request(18.0);
    let trim = solve_longitudinal_trim(&model, &config, &request)
        .unwrap()
        .evaluation;
    let mut elevator = trim.variables;
    elevator.elevator_command += 0.02;
    let elevator_evaluation =
        evaluate_longitudinal_trim_candidate(&model, &config, &request, elevator).unwrap();
    assert!(
        (elevator_evaluation.residuals.pitch_moment_nm - trim.residuals.pitch_moment_nm).abs()
            > 1.0e-3
    );
    let mut throttle = trim.variables;
    throttle.throttle += 0.02;
    let throttle_evaluation =
        evaluate_longitudinal_trim_candidate(&model, &config, &request, throttle).unwrap();
    assert!(
        (throttle_evaluation.residuals.longitudinal_force_n - trim.residuals.longitudinal_force_n)
            .abs()
            > 1.0e-3
    );
}

#[test]
fn bounded_infeasible_case_returns_a_finite_diagnostic_failure() {
    let model = model();
    let config = config();
    let request = LongitudinalTrimRequest::new(
        24.0,
        TrimBounds::new(-0.1, 0.2).unwrap(),
        TrimBounds::new(-0.8, 0.8).unwrap(),
        TrimBounds::new(0.0, 0.03).unwrap(),
        LongitudinalTrimVariables::new(0.05, 0.0, 0.01).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
        30,
    )
    .unwrap();
    let failure = solve_longitudinal_trim(&model, &config, &request).unwrap_err();
    assert!(matches!(
        failure.reason,
        LongitudinalTrimFailureReason::NoFeasibleSolution
            | LongitudinalTrimFailureReason::SingularJacobian
            | LongitudinalTrimFailureReason::IterationLimit
    ));
    let last = failure.last_evaluation.unwrap();
    assert!(!last.residuals.is_within(request.tolerances()));
    assert!(last.residuals.longitudinal_force_n.is_finite());
    assert!(last.residuals.vertical_force_n.is_finite());
    assert!(last.residuals.pitch_moment_nm.is_finite());
}

#[test]
fn reynolds_and_m2_4b_propulsion_both_change_trim() {
    let base_model = model();
    let config = config();
    let slow_request = request(15.0);
    let fast_request = request(21.0);
    let slow = solve_longitudinal_trim(&base_model, &config, &slow_request)
        .unwrap()
        .evaluation;
    let fast = solve_longitudinal_trim(&base_model, &config, &fast_request)
        .unwrap()
        .evaluation;
    assert_ne!(slow.variables, fast.variables);

    let slow_elements =
        effective_aero_elements_for_positions(&base_model, &slow.control_surface_positions);
    let fast_elements =
        effective_aero_elements_for_positions(&base_model, &fast.control_surface_positions);
    let slow_re = evaluate_aircraft_aero_element(
        &slow.state,
        &slow_elements[0],
        &base_model.aero_elements()[0],
        &base_model,
        config.aero_environment(),
    )
    .reynolds()
    .unwrap()
    .local_reynolds;
    let fast_re = evaluate_aircraft_aero_element(
        &fast.state,
        &fast_elements[0],
        &base_model.aero_elements()[0],
        &base_model,
        config.aero_environment(),
    )
    .reynolds()
    .unwrap()
    .local_reynolds;
    assert!(fast_re > slow_re);

    let lossy_model = model_with_esc_resistance(0.035);
    let base_trim = solve_longitudinal_trim(&base_model, &config, &request(18.0))
        .unwrap()
        .evaluation;
    let lossy_trim = solve_longitudinal_trim(&lossy_model, &config, &request(18.0))
        .unwrap()
        .evaluation;
    assert_ne!(base_trim.variables, lossy_trim.variables);
    assert!(lossy_trim.variables.throttle > base_trim.variables.throttle);
}

#[test]
fn fixture_is_synthetic_and_does_not_promote_reference_aircraft() {
    let model = model();
    assert_eq!(
        model.classification(),
        AircraftClassification::SyntheticTest
    );
    assert!(model.reference_aircraft().is_none());
    let lower = FIXTURE.to_ascii_lowercase();
    for forbidden in ["sig", "kadet", "lt-40", "apc", "himax", "castle"] {
        assert!(!lower.contains(forbidden));
    }
}
