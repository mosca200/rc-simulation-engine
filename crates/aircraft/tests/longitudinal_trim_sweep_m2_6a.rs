use aircraft::{
    AircraftSimulationConfig, LongitudinalTrimEvaluation, LongitudinalTrimFailureReason,
    LongitudinalTrimRequest, LongitudinalTrimRequestError, LongitudinalTrimResiduals,
    LongitudinalTrimSolution, LongitudinalTrimSweepError, LongitudinalTrimSweepOutcome,
    LongitudinalTrimSweepRequest, LongitudinalTrimTolerances, LongitudinalTrimVariables,
    ReEvaluationMismatchDetail, ReEvaluationUnverifiableDetail, TrimBounds,
    evaluate_longitudinal_trim_candidate, solve_longitudinal_trim, solve_longitudinal_trim_sweep,
};
use model::{AircraftClassification, AircraftModel, AircraftModelLoader};
use sim_core::AeroEnvironment;
use sim_math::Vec3;

const FIXTURE: &str = include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");

fn model() -> AircraftModel {
    AircraftModelLoader::from_json_str(FIXTURE).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        Vec3::new(0.0, 0.0, 9.80665),
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn sweep_template() -> (
    TrimBounds,
    TrimBounds,
    TrimBounds,
    LongitudinalTrimVariables,
    LongitudinalTrimTolerances,
    usize,
) {
    (
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
        40,
    )
}

fn sweep_request(speeds: Vec<f64>) -> LongitudinalTrimSweepRequest {
    let (alpha, elevator, throttle, guess, tolerance, iters) = sweep_template();
    LongitudinalTrimSweepRequest::new(speeds, alpha, elevator, throttle, guess, tolerance, iters)
        .unwrap()
}

#[test]
fn sweep_request_validation_fails_closed_on_invalid_speed_list() {
    let (alpha, elevator, throttle, guess, tolerance, iters) = sweep_template();
    assert_eq!(
        LongitudinalTrimSweepRequest::new(
            vec![],
            alpha,
            elevator,
            throttle,
            guess,
            tolerance,
            iters
        ),
        Err(LongitudinalTrimSweepError::EmptyTargetSpeeds)
    );
    for (index, value) in [(0_usize, f64::NAN), (2_usize, f64::INFINITY)] {
        let mut speeds = vec![15.0, 18.0, 21.0, 24.0];
        speeds[index] = value;
        assert_eq!(
            LongitudinalTrimSweepRequest::new(
                speeds, alpha, elevator, throttle, guess, tolerance, iters
            ),
            Err(LongitudinalTrimSweepError::NonFiniteTargetAirspeed { index })
        );
    }
    for (index, value) in [
        (0_usize, 0.0),
        (1_usize, -1.0),
        (3_usize, f64::NEG_INFINITY),
    ] {
        let mut speeds = vec![15.0, 18.0, 21.0, 24.0];
        speeds[index] = value;
        let expected = if value.is_finite() {
            LongitudinalTrimSweepError::NonPositiveTargetAirspeed { index }
        } else {
            LongitudinalTrimSweepError::NonFiniteTargetAirspeed { index }
        };
        assert_eq!(
            LongitudinalTrimSweepRequest::new(
                speeds, alpha, elevator, throttle, guess, tolerance, iters
            ),
            Err(expected)
        );
    }
    // Shared-request validation propagates M2.5's own request errors. We trigger
    // `InvalidIterationLimit` (0 iterations) because `TrimBounds::new` is the only public
    // constructor and cannot yield an invalid `TrimBounds` value at the call site.
    assert_eq!(
        LongitudinalTrimSweepRequest::new(
            vec![18.0],
            alpha,
            elevator,
            throttle,
            guess,
            tolerance,
            0
        ),
        Err(LongitudinalTrimSweepError::InvalidSharedRequest(
            LongitudinalTrimRequestError::InvalidIterationLimit
        ))
    );
}

#[test]
fn sweep_produces_one_ordered_result_per_requested_speed() {
    let model = model();
    let config = config();
    let speeds = vec![12.0, 15.0, 18.0, 21.0, 24.0];
    let request = sweep_request(speeds.clone());
    assert_eq!(request.target_airspeeds_mps(), speeds.as_slice());
    let sweep = solve_longitudinal_trim_sweep(&model, &config, &request).unwrap();
    assert_eq!(sweep.len(), speeds.len());
    assert!(!sweep.is_empty());
    let collected: Vec<f64> = sweep.target_airspeeds_mps().collect();
    assert_eq!(collected, speeds);
    for (index, point) in sweep.points().iter().enumerate() {
        assert_eq!(point.target_airspeed_mps, speeds[index]);
    }
}

#[test]
fn successful_points_independently_re_evaluate_within_trim_tolerances() {
    let model = model();
    let config = config();
    let sweep =
        solve_longitudinal_trim_sweep(&model, &config, &sweep_request(vec![15.0, 18.0, 21.0]))
            .unwrap();
    let (alpha, elevator, throttle, guess, tolerance, iters) = sweep_template();
    for point in sweep.points() {
        if let LongitudinalTrimSweepOutcome::Success { solution } = &point.outcome {
            let solution: &LongitudinalTrimSolution = solution;
            assert!(
                solution.evaluation.residuals.is_within(tolerance),
                "converged solution at {} mps exceeded tolerance",
                point.target_airspeed_mps,
            );
            let request = LongitudinalTrimRequest::new(
                point.target_airspeed_mps,
                alpha,
                elevator,
                throttle,
                guess,
                tolerance,
                iters,
            )
            .unwrap();
            let independent = evaluate_longitudinal_trim_candidate(
                &model,
                &config,
                &request,
                solution.evaluation.variables,
            )
            .unwrap();
            assert_eq!(
                independent, solution.evaluation,
                "independent re-evaluation diverged at {} mps",
                point.target_airspeed_mps,
            );
            assert!(independent.residuals.is_within(tolerance));
        }
    }
    assert_eq!(sweep.re_evaluation_mismatch_count(), 0);
    assert_eq!(sweep.re_evaluation_unverifiable_count(), 0);
}

#[test]
fn identical_sweeps_produce_identical_structured_results() {
    let model = model();
    let config = config();
    let speeds = vec![15.0, 18.0, 21.0];
    let first =
        solve_longitudinal_trim_sweep(&model, &config, &sweep_request(speeds.clone())).unwrap();
    let second =
        solve_longitudinal_trim_sweep(&model, &config, &sweep_request(speeds.clone())).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.success_count(), second.success_count());
    assert_eq!(first.trim_failure_count(), second.trim_failure_count());
    assert_eq!(
        first.re_evaluation_mismatch_count(),
        second.re_evaluation_mismatch_count(),
    );
    let third = solve_longitudinal_trim_sweep(
        &model,
        &config,
        &sweep_request(vec![15.0, 18.0, 21.0, 24.0]),
    )
    .unwrap();
    assert_ne!(
        first, third,
        "changing the speed set must change the result"
    );
}

#[test]
fn changing_speed_changes_successful_trim_solutions() {
    let model = model();
    let config = config();
    let (alpha, elevator, throttle, guess, tolerance, iters) = sweep_template();
    let fast_variables = solve_longitudinal_trim(
        &model,
        &config,
        &LongitudinalTrimRequest::new(21.0, alpha, elevator, throttle, guess, tolerance, iters)
            .unwrap(),
    )
    .unwrap()
    .evaluation
    .variables;
    let slow_variables = solve_longitudinal_trim(
        &model,
        &config,
        &LongitudinalTrimRequest::new(15.0, alpha, elevator, throttle, guess, tolerance, iters)
            .unwrap(),
    )
    .unwrap()
    .evaluation
    .variables;
    assert_ne!(fast_variables, slow_variables);

    let sweep =
        solve_longitudinal_trim_sweep(&model, &config, &sweep_request(vec![15.0, 21.0])).unwrap();
    let mut variables_by_speed = Vec::new();
    for point in sweep.points() {
        let LongitudinalTrimSweepOutcome::Success { solution } = &point.outcome else {
            panic!("both 15 and 21 mps should converge for the synthetic fixture");
        };
        let solution: &LongitudinalTrimSolution = solution;
        variables_by_speed.push((point.target_airspeed_mps, solution.evaluation.variables));
    }
    assert_eq!(
        variables_by_speed,
        vec![(15.0, slow_variables), (21.0, fast_variables)]
    );
}

#[test]
fn deliberately_bounded_infeasible_point_is_a_deterministic_trim_failure_point() {
    let model = model();
    let config = config();
    let tolerance = LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap();
    let iterations = 30;
    // Construct a sweep request whose shared template intentionally makes every point
    // bounded/infeasible (tight throttle and alpha). The sweep MUST continue across every
    // requested speed and report each as a structured trim failure rather than aborting.
    let sweep_request = LongitudinalTrimSweepRequest::new(
        vec![18.0, 24.0, 21.0],
        TrimBounds::new(-0.1, 0.2).unwrap(),
        TrimBounds::new(-0.8, 0.8).unwrap(),
        TrimBounds::new(0.0, 0.03).unwrap(),
        LongitudinalTrimVariables::new(0.05, 0.0, 0.01).unwrap(),
        tolerance,
        iterations,
    )
    .unwrap();
    let infeasible_outcome = solve_longitudinal_trim(
        &model,
        &config,
        &LongitudinalTrimRequest::new(
            24.0,
            TrimBounds::new(-0.1, 0.2).unwrap(),
            TrimBounds::new(-0.8, 0.8).unwrap(),
            TrimBounds::new(0.0, 0.03).unwrap(),
            LongitudinalTrimVariables::new(0.05, 0.0, 0.01).unwrap(),
            tolerance,
            iterations,
        )
        .unwrap(),
    )
    .unwrap_err();
    assert!(matches!(
        infeasible_outcome.reason,
        LongitudinalTrimFailureReason::NoFeasibleSolution
            | LongitudinalTrimFailureReason::SingularJacobian
            | LongitudinalTrimFailureReason::IterationLimit
    ));

    let sweep = solve_longitudinal_trim_sweep(&model, &config, &sweep_request).unwrap();
    assert_eq!(sweep.len(), 3);
    assert_eq!(sweep.success_count(), 0);
    assert_eq!(sweep.trim_failure_count(), 3);
    assert_eq!(sweep.re_evaluation_mismatch_count(), 0);
    let failed = sweep
        .points()
        .iter()
        .find(|point| point.target_airspeed_mps == 24.0)
        .expect("the 24 mps point must be present");
    let LongitudinalTrimSweepOutcome::TrimFailure { failure } = &failed.outcome else {
        panic!("the 24 mps point must be a trim failure");
    };
    assert_eq!(failure.reason, infeasible_outcome.reason);
    assert!(failure.last_evaluation.is_some());
    let last_evaluation = failure.last_evaluation.as_ref().unwrap();
    assert!(!last_evaluation.residuals.is_within(tolerance));
    let collected: Vec<f64> = sweep.target_airspeeds_mps().collect();
    assert_eq!(
        collected,
        vec![18.0, 24.0, 21.0],
        "input order must be preserved"
    );
}

#[test]
fn deterministic_ordering_is_explicit_and_partial_results_are_not_built_for_invalid_request() {
    let model = model();
    let config = config();
    let invalid = LongitudinalTrimSweepRequest::new(
        vec![15.0, f64::NAN, 21.0],
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
        40,
    )
    .unwrap_err();
    assert_eq!(
        invalid,
        LongitudinalTrimSweepError::NonFiniteTargetAirspeed { index: 1 }
    );
    let sweep = solve_longitudinal_trim_sweep(
        &model,
        &config,
        &sweep_request(vec![24.0, 21.0, 18.0, 15.0]),
    )
    .unwrap();
    let collected: Vec<f64> = sweep.target_airspeeds_mps().collect();
    assert_eq!(collected, vec![24.0, 21.0, 18.0, 15.0]);
}

#[test]
fn sweep_outcome_helpers_match_variants() {
    let (alpha, elevator, throttle, guess, tolerance, iters) = sweep_template();
    let success = LongitudinalTrimSweepOutcome::Success {
        solution: Box::new(
            solve_longitudinal_trim(
                &model(),
                &config(),
                &LongitudinalTrimRequest::new(
                    18.0, alpha, elevator, throttle, guess, tolerance, iters,
                )
                .unwrap(),
            )
            .unwrap(),
        ),
    };
    assert!(success.is_success());
    assert!(!success.is_trim_failure());
    assert!(!success.is_re_evaluation_mismatch());
    assert!(!success.is_re_evaluation_unverifiable());
}

#[test]
fn residuals_helper_is_within_matches_shared_request_tolerances() {
    let tolerance = LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap();
    let within = LongitudinalTrimResiduals {
        longitudinal_force_n: 5.0e-7,
        vertical_force_n: -3.0e-7,
        pitch_moment_nm: 4.0e-8,
    };
    assert!(within.is_within(tolerance));
    let outside = LongitudinalTrimResiduals {
        longitudinal_force_n: 5.0e-5,
        vertical_force_n: 0.0,
        pitch_moment_nm: 0.0,
    };
    assert!(!outside.is_within(tolerance));
}

#[test]
fn fixture_remains_synthetic_test_and_does_not_promote_reference_aircraft() {
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
    let sweep =
        solve_longitudinal_trim_sweep(&model, &config(), &sweep_request(vec![15.0, 18.0, 21.0]))
            .unwrap();
    assert_eq!(sweep.success_count(), 3);
}

/// Compile-time and import coverage for the public integrity-detail types and their
/// required accessors.
///
/// M2.6A's public re-exports place `ReEvaluationMismatchDetail` and
/// `ReEvaluationUnverifiableDetail` at the crate root, but the integrity variants are
/// constructed only through the validated sweep path. The synthetic fixture used by every
/// other test never produces a real mismatch or unverifiable outcome, and the in-crate
/// test-only constructors are `pub(crate)` and therefore unreachable from an integration
/// test. We therefore exercise the public API surface (re-exports + accessor signatures)
/// at the compile / signature level here, and leave the runtime construction path to the
/// in-crate unit tests. A real `LongitudinalTrimSweep` from a public sweep call is also
/// walked to demonstrate that the public `Success` outcome's boxed payload is consumable
/// end-to-end from outside the module.
#[test]
fn public_integrity_detail_api_is_reachable_from_outside_the_module() {
    // Type-level reachability: the detail types are re-exported from `aircraft`.
    // Referencing them as types proves the re-export is in place and gives a
    // compile-time guarantee that the public names are stable.
    let _mismatch_detail: ReEvaluationMismatchDetail;
    let _unverifiable_detail: ReEvaluationUnverifiableDetail;
    let _evaluation: LongitudinalTrimEvaluation;

    // Accessor-signature coverage: bind function pointers to the required accessors so
    // that a missing or wrongly-typed public method would fail to compile.
    let _mismatch_iteration: fn(&ReEvaluationMismatchDetail) -> usize =
        ReEvaluationMismatchDetail::iteration_count;
    let _mismatch_solver: fn(&ReEvaluationMismatchDetail) -> &LongitudinalTrimEvaluation =
        ReEvaluationMismatchDetail::solver_evaluation;
    let _mismatch_independent: fn(&ReEvaluationMismatchDetail) -> &LongitudinalTrimEvaluation =
        ReEvaluationMismatchDetail::independent_evaluation;

    let _unverifiable_iteration: fn(&ReEvaluationUnverifiableDetail) -> usize =
        ReEvaluationUnverifiableDetail::iteration_count;
    let _unverifiable_solver: fn(&ReEvaluationUnverifiableDetail) -> &LongitudinalTrimEvaluation =
        ReEvaluationUnverifiableDetail::solver_evaluation;

    // End-to-end reachability: drive the public sweep and walk a `Success` outcome
    // through the public `LongitudinalTrimSweepOutcome` API path. This is the only
    // outcome the public validated sweep can produce on the synthetic fixture, and
    // reaching it through the public type proves the `Success.solution` boxed payload
    // and the standard accessors are consumable from outside the `trim_sweep` module.
    let sweep =
        solve_longitudinal_trim_sweep(&model(), &config(), &sweep_request(vec![15.0, 18.0, 21.0]))
            .unwrap();
    let success: LongitudinalTrimSolution = sweep
        .points()
        .iter()
        .find_map(|point| match &point.outcome {
            LongitudinalTrimSweepOutcome::Success { solution } => Some(**solution),
            _ => None,
        })
        .expect("the public sweep must produce a Success outcome on the synthetic fixture");
    assert!(
        success
            .evaluation
            .residuals
            .is_within(LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap())
    );
}
