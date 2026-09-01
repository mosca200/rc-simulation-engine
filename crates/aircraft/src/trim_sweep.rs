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
        solve_longitudinal_trim,
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
#[allow(
    clippy::large_enum_variant,
    reason = "ReEvaluationMismatch is an exceptional integrity path; its larger size keeps the success/failure outcomes unboxed and the public API ergonomic"
)]
#[derive(Debug, Clone, PartialEq)]
pub enum LongitudinalTrimSweepOutcome {
    Success {
        solution: LongitudinalTrimSolution,
    },
    TrimFailure {
        failure: LongitudinalTrimFailure,
    },
    ReEvaluationMismatch {
        iteration_count: usize,
        solver_evaluation: LongitudinalTrimEvaluation,
        independent_evaluation: LongitudinalTrimEvaluation,
    },
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
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimSweep {
    points: Vec<LongitudinalTrimSweepPoint>,
}

impl LongitudinalTrimSweep {
    #[must_use]
    pub const fn from_points(points: Vec<LongitudinalTrimSweepPoint>) -> Self {
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

    /// Counts how many points exhibited a re-evaluation mismatch. A non-zero value indicates an
    /// internal inconsistency between the M2.5 solver and the runtime path.
    #[must_use]
    pub fn re_evaluation_mismatch_count(&self) -> usize {
        self.points
            .iter()
            .filter(|point| point.outcome.is_re_evaluation_mismatch())
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
/// A point whose M2.5 solution cannot be re-evaluated through the runtime path is recorded as
/// [`LongitudinalTrimSweepOutcome::ReEvaluationMismatch`]; the sweep also continues in that
/// case. Invalid sweep requests fail closed through [`LongitudinalTrimSweepError`] and produce
/// no partial results.
pub fn solve_longitudinal_trim_sweep(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep_request: &LongitudinalTrimSweepRequest,
) -> Result<LongitudinalTrimSweep, LongitudinalTrimSweepError> {
    let speeds = sweep_request.target_airspeeds_mps.clone();
    let mut points = Vec::with_capacity(speeds.len());
    for speed in speeds {
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
            Some(independent) if independent == solution.evaluation => {
                LongitudinalTrimSweepOutcome::Success { solution }
            }
            Some(independent) => LongitudinalTrimSweepOutcome::ReEvaluationMismatch {
                iteration_count: solution.iteration_count,
                solver_evaluation: solution.evaluation,
                independent_evaluation: independent,
            },
            // Independent evaluation only fails when runtime physics produces non-finite values
            // for a point that the M2.5 solver accepted as converged. Treat that the same as a
            // re-evaluation mismatch so the sweep completes and reporting can flag it.
            None => LongitudinalTrimSweepOutcome::ReEvaluationMismatch {
                iteration_count: solution.iteration_count,
                solver_evaluation: solution.evaluation,
                independent_evaluation: solution.evaluation,
            },
        },
        Err(failure) => LongitudinalTrimSweepOutcome::TrimFailure { failure },
    }
}
