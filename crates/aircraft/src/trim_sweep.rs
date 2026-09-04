//! M2.6A deterministic longitudinal trim sweep validation primitive.
//!
//! Builds on the M2.5 deterministic bounded Newton trim solver by evaluating an explicitly ordered
//! set of target airspeeds. For every requested speed the primitive invokes
//! [`solve_longitudinal_trim`] and, when the solver converges, independently calls
//! [`evaluate_longitudinal_trim_candidate`] to confirm the returned solution is physically
//! re-evaluable through the same runtime path.
//!
//! This slice is offline validation infrastructure only. It MUST NOT modify the M2.5 trim solver
//! or any aircraft runtime physics.

use crate::{
    AircraftSimulationConfig,
    trim::{
        LongitudinalTrimEvaluation, LongitudinalTrimFailure, LongitudinalTrimRequest,
        LongitudinalTrimRequestError, LongitudinalTrimSolution, LongitudinalTrimTolerances,
        LongitudinalTrimVariables, TrimBounds, evaluate_longitudinal_trim_candidate,
        evaluations_bitwise_equal, solve_longitudinal_trim,
    },
};
use model::AircraftModel;
use thiserror::Error;

/// Fail-closed sweep request errors. These are distinct from M2.5 trim failures: a sweep request
/// error means the caller asked for something the primitive cannot interpret, not that a bounded
/// physical problem lacked a feasible solution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum LongitudinalTrimSweepError {
    #[error("longitudinal trim sweep request must contain at least one target airspeed")]
    EmptyTargetSpeeds,
    #[error("longitudinal trim sweep target airspeed at index {index} is not finite")]
    NonFiniteTargetAirspeed { index: usize },
    #[error("longitudinal trim sweep target airspeed at index {index} is not greater than zero")]
    NonPositiveTargetAirspeed { index: usize },
    #[error("longitudinal trim sweep shared request parameters are invalid: {0}")]
    InvalidSharedRequest(#[from] LongitudinalTrimRequestError),
}

/// Integrity-level detail: the M2.5 solver produced a solution whose independent runtime
/// re-evaluation exists but disagrees with the solver-cached residuals. Boxed to keep the
/// outcome enum compact, following the [`LongitudinalTrimFailure`] precedent.
///
/// The fields are private: an instance can only be produced through the validated sweep
/// path. Downstream reporting layers read the fields through the public read-only
/// accessors below.
#[derive(Debug, Clone, PartialEq)]
pub struct ReEvaluationMismatchDetail {
    iteration_count: usize,
    solver_evaluation: LongitudinalTrimEvaluation,
    independent_evaluation: LongitudinalTrimEvaluation,
}

impl ReEvaluationMismatchDetail {
    /// M2.5 solver iteration count at the point the divergence was recorded.
    #[must_use]
    pub const fn iteration_count(&self) -> usize {
        self.iteration_count
    }

    /// The evaluation the M2.5 solver cached as converged.
    #[must_use]
    pub fn solver_evaluation(&self) -> &LongitudinalTrimEvaluation {
        &self.solver_evaluation
    }

    /// The independent runtime re-evaluation that disagreed with the solver-cached
    /// evaluation.
    #[must_use]
    pub fn independent_evaluation(&self) -> &LongitudinalTrimEvaluation {
        &self.independent_evaluation
    }
}

/// Integrity-level detail: the M2.5 solver produced a solution whose independent runtime
/// re-evaluation could not be produced because the runtime path produced non-finite values
/// for a state the solver accepted as converged. No independent evaluation exists; the
/// absence is represented truthfully by this variant rather than by copying the solver
/// evaluation into an `Option`. Boxed to keep the outcome enum compact.
///
/// The fields are private: an instance can only be produced through the validated sweep
/// path. Downstream reporting layers read the fields through the public read-only
/// accessors below.
#[derive(Debug, Clone, PartialEq)]
pub struct ReEvaluationUnverifiableDetail {
    iteration_count: usize,
    solver_evaluation: LongitudinalTrimEvaluation,
}

impl ReEvaluationUnverifiableDetail {
    /// M2.5 solver iteration count at the point the unverifiable outcome was recorded.
    #[must_use]
    pub const fn iteration_count(&self) -> usize {
        self.iteration_count
    }

    /// The evaluation the M2.5 solver cached as converged. The runtime path returned
    /// `None` when asked to independently re-evaluate this state, so no independent
    /// evaluation exists to expose.
    #[must_use]
    pub fn solver_evaluation(&self) -> &LongitudinalTrimEvaluation {
        &self.solver_evaluation
    }
}

/// Per-point result of a longitudinal trim sweep. Determinism, ordering, and independent
/// re-evaluation are all expressed through this enum.
///
/// * [`LongitudinalTrimSweepOutcome::Success`] — bounded Newton converged and the returned
///   solution re-evaluates identically through the runtime path.
/// * [`LongitudinalTrimSweepOutcome::TrimFailure`] — bounded physical problem lacked a feasible
///   solution, or another M2.5 trim failure was reported. This is NOT a software error.
/// * [`LongitudinalTrimSweepOutcome::ReEvaluationMismatch`] — M2.5 solver produced a solution
///   that, when independently re-evaluated through the runtime path, disagreed with the cached
///   residuals. This SHOULD never happen if M2.5 is internally consistent and is recorded as a
///   distinct, integrity-level outcome so reporting layers can flag it.
/// * [`LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable`] — M2.5 solver produced a
///   solution but the independent runtime re-evaluation could not be produced (the runtime
///   path returned `None` because the candidate produced non-finite values for a state the
///   solver already accepted as converged). This is also an integrity-level outcome and is
///   distinct from [`Self::ReEvaluationMismatch`] precisely because no independent
///   evaluation exists to compare against the solver-cached one.
#[derive(Debug, Clone, PartialEq)]
pub enum LongitudinalTrimSweepOutcome {
    Success {
        solution: Box<LongitudinalTrimSolution>,
    },
    TrimFailure {
        failure: LongitudinalTrimFailure,
    },
    ReEvaluationMismatch(Box<ReEvaluationMismatchDetail>),
    ReEvaluationUnverifiable(Box<ReEvaluationUnverifiableDetail>),
}

impl LongitudinalTrimSweepOutcome {
    #[must_use]
    pub const fn is_success(&self) -> bool {
        matches!(self, Self::Success { .. })
    }

    #[must_use]
    pub const fn is_trim_failure(&self) -> bool {
        matches!(self, Self::TrimFailure { .. })
    }

    #[must_use]
    pub const fn is_re_evaluation_mismatch(&self) -> bool {
        matches!(self, Self::ReEvaluationMismatch { .. })
    }

    #[must_use]
    pub const fn is_re_evaluation_unverifiable(&self) -> bool {
        matches!(self, Self::ReEvaluationUnverifiable { .. })
    }
}

/// One sweep point: the original target airspeed paired with its evaluated outcome.
///
/// The target airspeed is stored alongside the outcome so callers can correlate a sweep row with
/// its input even if they did not retain the input slice.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimSweepPoint {
    pub target_airspeed_mps: f64,
    pub outcome: LongitudinalTrimSweepOutcome,
}

/// Ordered sweep result. Points are stored in the same order as the request's target airspeeds;
/// indexing is therefore deterministic and stable.
///
/// Construction is private: the only validated path that produces a [`LongitudinalTrimSweep`]
/// is [`solve_longitudinal_trim_sweep`]. External callers cannot bypass the validated
/// execution path and forge an arbitrary or empty result.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimSweep {
    points: Vec<LongitudinalTrimSweepPoint>,
}

impl LongitudinalTrimSweep {
    /// Private constructor used only by [`solve_longitudinal_trim_sweep`]. Not exposed to
    /// external callers so a [`LongitudinalTrimSweep`] can only be produced through the
    /// validated execution path.
    #[must_use]
    const fn from_points(points: Vec<LongitudinalTrimSweepPoint>) -> Self {
        Self { points }
    }

    #[must_use]
    pub fn points(&self) -> &[LongitudinalTrimSweepPoint] {
        &self.points
    }

    #[must_use]
    pub fn into_points(self) -> Vec<LongitudinalTrimSweepPoint> {
        self.points
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Iterates the target airspeeds in sweep order.
    pub fn target_airspeeds_mps(&self) -> impl Iterator<Item = f64> + '_ {
        self.points.iter().map(|point| point.target_airspeed_mps)
    }

    /// Counts how many points converged successfully.
    #[must_use]
    pub fn success_count(&self) -> usize {
        self.points
            .iter()
            .filter(|point| point.outcome.is_success())
            .count()
    }

    /// Counts how many points failed through the M2.5 trim failure path.
    #[must_use]
    pub fn trim_failure_count(&self) -> usize {
        self.points
            .iter()
            .filter(|point| point.outcome.is_trim_failure())
            .count()
    }

    /// Counts how many points exhibited a re-evaluation mismatch (independent evaluation
    /// exists but disagrees with the solver-cached evaluation). A non-zero value indicates
    /// an internal inconsistency between the M2.5 solver and the runtime path.
    #[must_use]
    pub fn re_evaluation_mismatch_count(&self) -> usize {
        self.points
            .iter()
            .filter(|point| point.outcome.is_re_evaluation_mismatch())
            .count()
    }

    /// Counts how many points were unverifiable: the M2.5 solver produced a solution but
    /// the independent runtime re-evaluation could not be produced (the runtime path
    /// returned `None` because the candidate produced non-finite values for a state the
    /// solver already accepted as converged). A non-zero value indicates an internal
    /// inconsistency between the M2.5 solver and the runtime path; this is a separate
    /// integrity counter from [`Self::re_evaluation_mismatch_count`] because the absence
    /// of an independent evaluation is itself the signal — no comparison is possible.
    #[must_use]
    pub fn re_evaluation_unverifiable_count(&self) -> usize {
        self.points
            .iter()
            .filter(|point| point.outcome.is_re_evaluation_unverifiable())
            .count()
    }
}

/// Sweep request describing the shared per-speed trim template and the ordered target airspeed
/// list. Built once and consumed by [`solve_longitudinal_trim_sweep`].
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimSweepRequest {
    target_airspeeds_mps: Vec<f64>,
    alpha_bounds_rad: TrimBounds,
    elevator_bounds: TrimBounds,
    throttle_bounds: TrimBounds,
    initial_guess: LongitudinalTrimVariables,
    tolerances: LongitudinalTrimTolerances,
    maximum_iterations: usize,
}

impl LongitudinalTrimSweepRequest {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        target_airspeeds_mps: Vec<f64>,
        alpha_bounds_rad: TrimBounds,
        elevator_bounds: TrimBounds,
        throttle_bounds: TrimBounds,
        initial_guess: LongitudinalTrimVariables,
        tolerances: LongitudinalTrimTolerances,
        maximum_iterations: usize,
    ) -> Result<Self, LongitudinalTrimSweepError> {
        if target_airspeeds_mps.is_empty() {
            return Err(LongitudinalTrimSweepError::EmptyTargetSpeeds);
        }
        for (index, speed) in target_airspeeds_mps.iter().enumerate() {
            if !speed.is_finite() {
                return Err(LongitudinalTrimSweepError::NonFiniteTargetAirspeed { index });
            }
            if *speed <= 0.0 {
                return Err(LongitudinalTrimSweepError::NonPositiveTargetAirspeed { index });
            }
        }
        // Validate the shared trim template up front by constructing a throwaway request with one
        // of the speeds. This reuses M2.5's existing fail-closed validation and surfaces any
        // invalid bounds/tolerances/iterations before the sweep begins.
        LongitudinalTrimRequest::new(
            target_airspeeds_mps[0],
            alpha_bounds_rad,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            maximum_iterations,
        )?;
        Ok(Self {
            target_airspeeds_mps,
            alpha_bounds_rad,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            maximum_iterations,
        })
    }

    #[must_use]
    pub fn target_airspeeds_mps(&self) -> &[f64] {
        &self.target_airspeeds_mps
    }

    #[must_use]
    pub const fn alpha_bounds_rad(&self) -> TrimBounds {
        self.alpha_bounds_rad
    }

    #[must_use]
    pub const fn elevator_bounds(&self) -> TrimBounds {
        self.elevator_bounds
    }

    #[must_use]
    pub const fn throttle_bounds(&self) -> TrimBounds {
        self.throttle_bounds
    }

    #[must_use]
    pub const fn initial_guess(&self) -> LongitudinalTrimVariables {
        self.initial_guess
    }

    #[must_use]
    pub const fn tolerances(&self) -> LongitudinalTrimTolerances {
        self.tolerances
    }

    #[must_use]
    pub const fn maximum_iterations(&self) -> usize {
        self.maximum_iterations
    }
}

/// Evaluates the M2.5 longitudinal trim for each requested airspeed in input order.
///
/// The sweep preserves input ordering and determinism: identical inputs produce identical
/// structured results. A point whose bounded physical problem lacks a feasible solution is
/// recorded as [`LongitudinalTrimSweepOutcome::TrimFailure`] without aborting the sweep.
/// A point whose M2.5 solution exists but whose independent runtime re-evaluation disagrees
/// with the solver-cached evaluation is recorded as
/// [`LongitudinalTrimSweepOutcome::ReEvaluationMismatch`]. A point whose M2.5 solution
/// exists but whose independent runtime re-evaluation could not be produced (the runtime
/// path returned `None` because the candidate produced non-finite values for a state the
/// solver already accepted as converged) is recorded as
/// [`LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable`]; the sweep also continues in
/// these integrity cases. Invalid sweep requests fail closed through
/// [`LongitudinalTrimSweepError`] and produce no partial results.
pub fn solve_longitudinal_trim_sweep(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep_request: &LongitudinalTrimSweepRequest,
) -> Result<LongitudinalTrimSweep, LongitudinalTrimSweepError> {
    let mut points = Vec::with_capacity(sweep_request.target_airspeeds_mps.len());
    for &speed in &sweep_request.target_airspeeds_mps {
        let request = LongitudinalTrimRequest::new(
            speed,
            sweep_request.alpha_bounds_rad,
            sweep_request.elevator_bounds,
            sweep_request.throttle_bounds,
            sweep_request.initial_guess,
            sweep_request.tolerances,
            sweep_request.maximum_iterations,
        )?;
        let outcome = evaluate_one(model, config, &request);
        points.push(LongitudinalTrimSweepPoint {
            target_airspeed_mps: speed,
            outcome,
        });
    }
    Ok(LongitudinalTrimSweep::from_points(points))
}

fn evaluate_one(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
) -> LongitudinalTrimSweepOutcome {
    match solve_longitudinal_trim(model, config, request) {
        Ok(solution) => match evaluate_longitudinal_trim_candidate(
            model,
            config,
            request,
            solution.evaluation.variables,
        ) {
            Some(independent) if evaluations_bitwise_equal(&independent, &solution.evaluation) => {
                LongitudinalTrimSweepOutcome::Success {
                    solution: Box::new(solution),
                }
            }
            Some(independent) => LongitudinalTrimSweepOutcome::ReEvaluationMismatch(Box::new(
                ReEvaluationMismatchDetail {
                    iteration_count: solution.iteration_count,
                    solver_evaluation: solution.evaluation,
                    independent_evaluation: independent,
                },
            )),
            // Independent evaluation only fails when runtime physics produces non-finite values
            // for a point that the M2.5 solver accepted as converged. Treat that as a distinct
            // unverifiable outcome so the absence of an independent evaluation is represented
            // truthfully rather than by copying the solver evaluation into the mismatch slot.
            None => LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(Box::new(
                ReEvaluationUnverifiableDetail {
                    iteration_count: solution.iteration_count,
                    solver_evaluation: solution.evaluation,
                },
            )),
        },
        Err(failure) => LongitudinalTrimSweepOutcome::TrimFailure { failure },
    }
}

/// Test-only seam: exercises the SAME production comparison logic as [`evaluate_one`]
/// (solve → re-evaluate → `evaluations_bitwise_equal` → outcome), but injects a signed-zero
/// flip into the solver-cached evaluation before the comparison. This makes the production
/// comparator deterministically reject the pair, proving the real sweep integrity path
/// catches `+0.0` vs `-0.0` differences that `PartialEq` would accept.
///
/// This is NOT a duplicate of the comparator — it reuses `evaluations_bitwise_equal` directly
/// and produces the same `LongitudinalTrimSweepOutcome` variants as the production path.
#[cfg(test)]
fn evaluate_one_with_signed_zero_reevaluation_probe(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
) -> LongitudinalTrimSweepOutcome {
    match solve_longitudinal_trim(model, config, request) {
        Ok(solution) => {
            // Inject a signed-zero flip into the cached evaluation.
            // position_world_m is always Vec3::zeros() from evaluate_candidate, so x is +0.0.
            let mut altered_solution = solution;
            assert_eq!(
                altered_solution
                    .evaluation
                    .state
                    .position_world_m
                    .x
                    .to_bits(),
                0.0_f64.to_bits(),
                "precondition: position_world_m.x must be +0.0"
            );
            altered_solution.evaluation.state.position_world_m.x = -0.0;

            // Re-evaluate independently through the production runtime path.
            match evaluate_longitudinal_trim_candidate(
                model,
                config,
                request,
                altered_solution.evaluation.variables,
            ) {
                // The production comparator must reject the signed-zero mismatch.
                Some(independent)
                    if evaluations_bitwise_equal(&independent, &altered_solution.evaluation) =>
                {
                    // This branch must NOT be reached: the bitwise comparator must reject
                    // the +0.0 vs -0.0 difference.
                    panic!(
                        "evaluations_bitwise_equal must reject signed-zero mismatch in sweep probe"
                    );
                }
                Some(independent) => LongitudinalTrimSweepOutcome::ReEvaluationMismatch(Box::new(
                    ReEvaluationMismatchDetail {
                        iteration_count: altered_solution.iteration_count,
                        solver_evaluation: altered_solution.evaluation,
                        independent_evaluation: independent,
                    },
                )),
                None => LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(Box::new(
                    ReEvaluationUnverifiableDetail {
                        iteration_count: altered_solution.iteration_count,
                        solver_evaluation: altered_solution.evaluation,
                    },
                )),
            }
        }
        Err(failure) => LongitudinalTrimSweepOutcome::TrimFailure { failure },
    }
}

#[cfg(test)]
impl LongitudinalTrimSweepOutcome {
    /// Test-only constructor for the unverifiable variant. The public M2.6A path constructs
    /// this variant only when the M2.5 solver converged and `evaluate_longitudinal_trim_candidate`
    /// returned `None`; the constructor exists so unit tests can exercise the new variant
    /// without contriving a runtime scenario that produces non-finite values.
    pub(crate) fn re_evaluation_unverifiable_for_test(
        iteration_count: usize,
        solver_evaluation: LongitudinalTrimEvaluation,
    ) -> Self {
        Self::ReEvaluationUnverifiable(Box::new(ReEvaluationUnverifiableDetail {
            iteration_count,
            solver_evaluation,
        }))
    }

    /// Test-only constructor for the mismatch variant. The public M2.6A path constructs
    /// this variant only when the M2.5 solver converged and the independent runtime
    /// re-evaluation diverged from the cached one; the constructor exists so unit tests
    /// can exercise the new variant's public accessors without contriving a runtime
    /// scenario that produces a divergence.
    pub(crate) fn re_evaluation_mismatch_for_test(
        iteration_count: usize,
        solver_evaluation: LongitudinalTrimEvaluation,
        independent_evaluation: LongitudinalTrimEvaluation,
    ) -> Self {
        Self::ReEvaluationMismatch(Box::new(ReEvaluationMismatchDetail {
            iteration_count,
            solver_evaluation,
            independent_evaluation,
        }))
    }
}

#[cfg(test)]
mod tests {
    //! Internal tests for the sweep outcome representation. These cover shape-level invariants
    //! that the integration tests in `crates/aircraft/tests/` cannot reach directly, such as
    //! the absence of a fabricated `independent_evaluation` in the unverifiable variant.

    use super::*;
    use crate::trim::{LongitudinalTrimFailure, LongitudinalTrimFailureReason};

    const FIXTURE: &str =
        include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");

    fn aircraft() -> AircraftModel {
        model::AircraftModelLoader::from_json_str(FIXTURE).unwrap()
    }

    fn sim_config() -> AircraftSimulationConfig {
        AircraftSimulationConfig::new(
            0.002,
            sim_math::Vec3::new(0.0, 0.0, 9.80665),
            sim_core::AeroEnvironment::new(1.225, sim_math::Vec3::zeros()).unwrap(),
        )
        .unwrap()
    }

    fn well_formed_evaluation() -> LongitudinalTrimEvaluation {
        // Build a real, well-formed `LongitudinalTrimEvaluation` by running one real trim
        // solve. The shape-level invariants tested here do not care about its values, only
        // that the type-level construction is correct.
        let (alpha, elevator, throttle, guess, tolerance, iters) = (
            TrimBounds::new(-0.15, 0.30).unwrap(),
            TrimBounds::new(-0.9, 0.9).unwrap(),
            TrimBounds::new(0.02, 1.0).unwrap(),
            LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
            LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
            40,
        );
        let request =
            LongitudinalTrimRequest::new(18.0, alpha, elevator, throttle, guess, tolerance, iters)
                .unwrap();
        solve_longitudinal_trim(&aircraft(), &sim_config(), &request)
            .unwrap()
            .evaluation
    }

    #[test]
    fn unverifiable_outcome_does_not_carry_an_independent_evaluation() {
        let solver = well_formed_evaluation();
        let outcome = LongitudinalTrimSweepOutcome::re_evaluation_unverifiable_for_test(3, solver);
        assert!(outcome.is_re_evaluation_unverifiable());
        assert!(!outcome.is_success());
        assert!(!outcome.is_trim_failure());
        assert!(!outcome.is_re_evaluation_mismatch());
        match outcome {
            LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(detail) => {
                assert_eq!(detail.iteration_count(), 3);
                // The accessor round-trips the solver-cached evaluation it was built
                // with, and the struct itself has no `independent_evaluation` field or
                // accessor. The absence of such a field is the type-level guarantee that
                // no fabricated evaluation can be smuggled in: the variant encodes "no
                // independent evaluation exists" by construction.
                assert_eq!(detail.solver_evaluation(), &solver);
            }
            other => panic!("expected ReEvaluationUnverifiable, got {other:?}"),
        }
    }

    #[test]
    fn outcome_predicates_are_disjoint() {
        let success = LongitudinalTrimSweepOutcome::Success {
            solution: Box::new(LongitudinalTrimSolution {
                evaluation: well_formed_evaluation(),
                iteration_count: 1,
            }),
        };
        let failure = LongitudinalTrimSweepOutcome::TrimFailure {
            failure: LongitudinalTrimFailure {
                reason: LongitudinalTrimFailureReason::NoFeasibleSolution,
                iteration_count: 0,
                last_evaluation: None,
            },
        };
        let unverifiable = LongitudinalTrimSweepOutcome::re_evaluation_unverifiable_for_test(
            1,
            well_formed_evaluation(),
        );
        let mismatch = LongitudinalTrimSweepOutcome::re_evaluation_mismatch_for_test(
            2,
            well_formed_evaluation(),
            well_formed_evaluation(),
        );
        for outcome in [&success, &failure, &unverifiable, &mismatch] {
            let mut true_count = 0;
            if outcome.is_success() {
                true_count += 1;
            }
            if outcome.is_trim_failure() {
                true_count += 1;
            }
            if outcome.is_re_evaluation_mismatch() {
                true_count += 1;
            }
            if outcome.is_re_evaluation_unverifiable() {
                true_count += 1;
            }
            assert_eq!(
                true_count, 1,
                "exactly one outcome predicate must hold at a time; got {true_count} for {outcome:?}"
            );
        }
    }

    #[test]
    fn mismatch_detail_accessors_round_trip_inputs() {
        let solver = well_formed_evaluation();
        let independent = well_formed_evaluation();
        let outcome =
            LongitudinalTrimSweepOutcome::re_evaluation_mismatch_for_test(5, solver, independent);
        match outcome {
            LongitudinalTrimSweepOutcome::ReEvaluationMismatch(detail) => {
                assert_eq!(detail.iteration_count(), 5);
                assert_eq!(detail.solver_evaluation(), &solver);
                assert_eq!(detail.independent_evaluation(), &independent);
            }
            other => panic!("expected ReEvaluationMismatch, got {other:?}"),
        }
    }

    // -----------------------------------------------------------------------
    // Integration test: sweep signed-zero proof through qualification
    // -----------------------------------------------------------------------

    use crate::AircraftSimulationConfig;
    use crate::trim_qualification::{
        LongitudinalTrimQualificationLimits, LongitudinalTrimQualificationOutcome,
        QualificationUnavailableReason, qualify_longitudinal_trim_sweep,
    };
    use sim_core::AeroEnvironment;
    use sim_math::Vec3;

    fn sweep_sim_config() -> AircraftSimulationConfig {
        AircraftSimulationConfig::new(
            0.002,
            Vec3::new(0.0, 0.0, 9.80665),
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap()
    }

    fn sweep_generous_limits() -> LongitudinalTrimQualificationLimits {
        LongitudinalTrimQualificationLimits::new(1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6).unwrap()
    }

    #[test]
    fn sweep_real_path_rejects_signed_zero_via_production_comparator() {
        // Build a real trim request.
        let request = LongitudinalTrimRequest::new(
            18.0,
            TrimBounds::new(-0.15, 0.30).unwrap(),
            TrimBounds::new(-0.9, 0.9).unwrap(),
            TrimBounds::new(0.02, 1.0).unwrap(),
            LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
            LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
            40,
        )
        .unwrap();

        // Exercise the REAL sweep comparison path with a signed-zero probe.
        // This calls solve_longitudinal_trim → evaluate_longitudinal_trim_candidate →
        // evaluations_bitwise_equal (the production comparator), with a +0.0/-0.0
        // difference injected into the cached evaluation.
        let outcome = evaluate_one_with_signed_zero_reevaluation_probe(
            &aircraft(),
            &sweep_sim_config(),
            &request,
        );

        // The production comparator must have rejected the signed-zero mismatch,
        // producing ReEvaluationMismatch (NOT Success).
        let (solver_eval, independent_eval) = match &outcome {
            LongitudinalTrimSweepOutcome::ReEvaluationMismatch(detail) => {
                (detail.solver_evaluation(), detail.independent_evaluation())
            }
            other => panic!("expected ReEvaluationMismatch from real sweep path, got {other:?}"),
        };

        // Proof: PartialEq accepts the signed-zero difference (the bug).
        assert_eq!(
            solver_eval, independent_eval,
            "PartialEq must treat +0.0 and -0.0 as equal — this is the bug the bitwise fix addresses"
        );

        // Proof: the production bitwise comparator rejected it through the real sweep path.
        assert!(
            !evaluations_bitwise_equal(solver_eval, independent_eval),
            "evaluations_bitwise_equal must reject the signed-zero mismatch"
        );

        // Now build a sweep containing this real mismatch and qualify it.
        let sweep = LongitudinalTrimSweep::from_points(vec![LongitudinalTrimSweepPoint {
            target_airspeed_mps: 18.0,
            outcome,
        }]);

        let qualification = qualify_longitudinal_trim_sweep(
            &aircraft(),
            &sweep_sim_config(),
            &sweep,
            &sweep_generous_limits(),
        );

        assert_eq!(qualification.len(), 1);
        match &qualification.points()[0].outcome {
            LongitudinalTrimQualificationOutcome::QualificationUnavailable {
                reason: QualificationUnavailableReason::SweepReEvaluationMismatch,
                diagnostics: None,
            } => {
                // Expected: the real sweep mismatch propagates to QualificationUnavailable.
            }
            other => panic!(
                "expected QualificationUnavailable::SweepReEvaluationMismatch, got {other:?}"
            ),
        }
    }
}
