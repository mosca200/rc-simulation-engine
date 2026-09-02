//! M2.7 local longitudinal trim characterization.
//!
//! Computes finite-difference pitch-moment derivatives around verified M2.6A trim sweep points:
//!
//! - local trim pitch stiffness: `dMy/dAlpha` (N·m/rad)
//! - local elevator control effectiveness: `dMy/dElevatorCommand` (N·m per normalized command)
//!
//! Uses symmetric central differences through the existing M2.5 runtime evaluator
//! ([`evaluate_longitudinal_trim_candidate`]). Perturbations are frozen-control: only the
//! perturbed variable changes; all others remain exactly at the verified trim values.
//!
//! This slice does NOT compute static margin, neutral point, aerodynamic center, or any
//! normalized aerodynamic coefficient derivative.

use crate::{
    AircraftSimulationConfig,
    trim::{
        LongitudinalTrimRequest, LongitudinalTrimVariables, evaluate_longitudinal_trim_candidate,
    },
    trim_sweep::{
        LongitudinalTrimSweep, LongitudinalTrimSweepOutcome, LongitudinalTrimSweepRequest,
    },
};
use model::AircraftModel;
use thiserror::Error;

/// Errors produced when constructing [`LongitudinalTrimCharacterizationSteps`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum CharacterizationStepsError {
    #[error("alpha step must be finite and strictly greater than zero")]
    InvalidAlphaStep,
    #[error("elevator step must be finite and strictly greater than zero")]
    InvalidElevatorStep,
}

/// Validated finite-difference step sizes for M2.7 characterization.
///
/// These steps have different semantics from the M2.5 solver Jacobian steps and MUST remain
/// explicit — no implicit defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimCharacterizationSteps {
    alpha_step_rad: f64,
    elevator_step: f64,
}

impl LongitudinalTrimCharacterizationSteps {
    /// Fails closed unless both steps are finite and strictly positive.
    pub fn new(
        alpha_step_rad: f64,
        elevator_step: f64,
    ) -> Result<Self, CharacterizationStepsError> {
        if !alpha_step_rad.is_finite() || alpha_step_rad <= 0.0 {
            return Err(CharacterizationStepsError::InvalidAlphaStep);
        }
        if !elevator_step.is_finite() || elevator_step <= 0.0 {
            return Err(CharacterizationStepsError::InvalidElevatorStep);
        }
        Ok(Self {
            alpha_step_rad,
            elevator_step,
        })
    }

    #[must_use]
    pub const fn alpha_step_rad(&self) -> f64 {
        self.alpha_step_rad
    }

    #[must_use]
    pub const fn elevator_step(&self) -> f64 {
        self.elevator_step
    }
}

/// Errors produced by [`characterize_longitudinal_trim_sweep`].
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum LongitudinalTrimCharacterizationError {
    #[error("sweep length ({sweep_len}) does not match request length ({request_len})")]
    SweepLengthMismatch {
        sweep_len: usize,
        request_len: usize,
    },
    #[error(
        "sweep target airspeed at index {index} ({} m/s) does not match request ({} m/s)",
        sweep_speed,
        request_speed
    )]
    SweepTargetAirspeedMismatch {
        index: usize,
        sweep_speed: f64,
        request_speed: f64,
    },
}

/// Reason a successfully trimmed point could not be characterized.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum CharacterizationUnavailableReason {
    #[error("alpha perturbation [{alpha_minus}, {alpha_plus}] exceeds bounds [{lower}, {upper}]")]
    AlphaPerturbationOutOfBounds {
        alpha_minus: f64,
        alpha_plus: f64,
        lower: f64,
        upper: f64,
    },
    #[error(
        "elevator perturbation [{elevator_minus}, {elevator_plus}] exceeds bounds [{lower}, {upper}]"
    )]
    ElevatorPerturbationOutOfBounds {
        elevator_minus: f64,
        elevator_plus: f64,
        lower: f64,
        upper: f64,
    },
    #[error("alpha perturbation evaluation returned non-finite ({side})")]
    AlphaPerturbationNonFinite { side: PerturbationSide },
    #[error("elevator perturbation evaluation returned non-finite ({side})")]
    ElevatorPerturbationNonFinite { side: PerturbationSide },
    #[error("derived pitch stiffness is not finite")]
    NonFinitePitchStiffness,
    #[error("derived elevator effectiveness is not finite")]
    NonFiniteElevatorEffectiveness,
}

/// Which side of a central-difference perturbation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PerturbationSide {
    Minus,
    Plus,
}

impl core::fmt::Display for PerturbationSide {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Minus => f.write_str("minus"),
            Self::Plus => f.write_str("plus"),
        }
    }
}

/// Per-point outcome of M2.7 characterization.
#[derive(Debug, Clone, PartialEq)]
pub enum LongitudinalTrimCharacterizationPointOutcome {
    /// Local derivatives computed successfully.
    Characterized(LongitudinalTrimCharacterizationData),
    /// Point was a trim failure — no derivatives computed.
    NotCharacterizedTrimFailure,
    /// Point was a re-evaluation mismatch — no derivatives computed.
    NotCharacterizedReEvaluationMismatch,
    /// Point was a re-evaluation unverifiable — no derivatives computed.
    NotCharacterizedReEvaluationUnverifiable,
    /// Point was a verified trim but characterization could not proceed.
    CharacterizationUnavailable(CharacterizationUnavailableReason),
}

/// Successful characterization data for one sweep point.
///
/// All fields are finite. The four sampled pitch moments are stored so the derivative is
/// auditable rather than returning only two opaque scalar derivatives.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimCharacterizationData {
    pub target_airspeed_mps: f64,
    pub alpha_rad: f64,
    pub elevator_command: f64,
    pub throttle: f64,
    pub alpha_step_rad: f64,
    pub elevator_step: f64,
    pub pitch_moment_at_trim_nm: f64,
    pub alpha_minus_pitch_moment_nm: f64,
    pub alpha_plus_pitch_moment_nm: f64,
    pub elevator_minus_pitch_moment_nm: f64,
    pub elevator_plus_pitch_moment_nm: f64,
    pub pitch_stiffness_nm_per_rad: f64,
    pub elevator_effectiveness_nm_per_command: f64,
}

/// One characterization point: target airspeed paired with its outcome.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimCharacterizationPoint {
    pub target_airspeed_mps: f64,
    pub outcome: LongitudinalTrimCharacterizationPointOutcome,
}

/// Ordered characterization result. Points are stored in the same order as the input sweep.
///
/// Construction is private: the only validated path is [`characterize_longitudinal_trim_sweep`].
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimCharacterization {
    points: Vec<LongitudinalTrimCharacterizationPoint>,
}

impl LongitudinalTrimCharacterization {
    #[must_use]
    const fn from_points(points: Vec<LongitudinalTrimCharacterizationPoint>) -> Self {
        Self { points }
    }

    #[must_use]
    pub fn points(&self) -> &[LongitudinalTrimCharacterizationPoint] {
        &self.points
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    #[must_use]
    pub fn characterized_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::Characterized(_)
                )
            })
            .count()
    }

    #[must_use]
    pub fn trim_failure_not_characterized_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedTrimFailure
                )
            })
            .count()
    }

    #[must_use]
    pub fn re_evaluation_mismatch_not_characterized_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationMismatch
                )
            })
            .count()
    }

    #[must_use]
    pub fn re_evaluation_unverifiable_not_characterized_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationUnverifiable
                )
            })
            .count()
    }

    #[must_use]
    pub fn characterization_unavailable_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(_)
                )
            })
            .count()
    }
}

/// Computes local longitudinal trim characterization for each point in a verified M2.6A sweep.
///
/// Only [`LongitudinalTrimSweepOutcome::Success`] points produce derivatives. All other
/// outcomes are recorded as explicit non-characterized states without fabricating values.
///
/// # Errors
///
/// Returns [`LongitudinalTrimCharacterizationError`] if the sweep does not correspond to the
/// supplied request (length mismatch or target airspeed mismatch).
pub fn characterize_longitudinal_trim_sweep(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep_request: &LongitudinalTrimSweepRequest,
    sweep: &LongitudinalTrimSweep,
    steps: LongitudinalTrimCharacterizationSteps,
) -> Result<LongitudinalTrimCharacterization, LongitudinalTrimCharacterizationError> {
    let request_speeds = sweep_request.target_airspeeds_mps();
    let sweep_points = sweep.points();

    if sweep_points.len() != request_speeds.len() {
        return Err(LongitudinalTrimCharacterizationError::SweepLengthMismatch {
            sweep_len: sweep_points.len(),
            request_len: request_speeds.len(),
        });
    }

    for (index, (sweep_point, &request_speed)) in
        sweep_points.iter().zip(request_speeds.iter()).enumerate()
    {
        if sweep_point.target_airspeed_mps != request_speed {
            return Err(
                LongitudinalTrimCharacterizationError::SweepTargetAirspeedMismatch {
                    index,
                    sweep_speed: sweep_point.target_airspeed_mps,
                    request_speed,
                },
            );
        }
    }

    let mut points = Vec::with_capacity(sweep_points.len());
    for sweep_point in sweep_points {
        let outcome = characterize_one(model, config, sweep_request, sweep_point, steps);
        points.push(LongitudinalTrimCharacterizationPoint {
            target_airspeed_mps: sweep_point.target_airspeed_mps,
            outcome,
        });
    }

    Ok(LongitudinalTrimCharacterization::from_points(points))
}

fn characterize_one(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep_request: &LongitudinalTrimSweepRequest,
    sweep_point: &crate::trim_sweep::LongitudinalTrimSweepPoint,
    steps: LongitudinalTrimCharacterizationSteps,
) -> LongitudinalTrimCharacterizationPointOutcome {
    let LongitudinalTrimSweepOutcome::Success { solution } = &sweep_point.outcome else {
        return match &sweep_point.outcome {
            LongitudinalTrimSweepOutcome::TrimFailure { .. } => {
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedTrimFailure
            }
            LongitudinalTrimSweepOutcome::ReEvaluationMismatch(_) => {
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationMismatch
            }
            LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(_) => {
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationUnverifiable
            }
            LongitudinalTrimSweepOutcome::Success { .. } => unreachable!(),
        };
    };

    let trim_eval = &solution.evaluation;
    let alpha0 = trim_eval.variables.alpha_rad;
    let elevator0 = trim_eval.variables.elevator_command;
    let throttle0 = trim_eval.variables.throttle;
    let target_airspeed = sweep_point.target_airspeed_mps;

    let alpha_bounds = sweep_request.alpha_bounds_rad();
    let elevator_bounds = sweep_request.elevator_bounds();

    let alpha_minus = alpha0 - steps.alpha_step_rad;
    let alpha_plus = alpha0 + steps.alpha_step_rad;
    let elevator_minus = elevator0 - steps.elevator_step;
    let elevator_plus = elevator0 + steps.elevator_step;

    if alpha_minus < alpha_bounds.lower() || alpha_plus > alpha_bounds.upper() {
        return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
            CharacterizationUnavailableReason::AlphaPerturbationOutOfBounds {
                alpha_minus,
                alpha_plus,
                lower: alpha_bounds.lower(),
                upper: alpha_bounds.upper(),
            },
        );
    }

    if elevator_minus < elevator_bounds.lower() || elevator_plus > elevator_bounds.upper() {
        return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
            CharacterizationUnavailableReason::ElevatorPerturbationOutOfBounds {
                elevator_minus,
                elevator_plus,
                lower: elevator_bounds.lower(),
                upper: elevator_bounds.upper(),
            },
        );
    }

    let request = match LongitudinalTrimRequest::new(
        target_airspeed,
        alpha_bounds,
        elevator_bounds,
        sweep_request.throttle_bounds(),
        sweep_request.initial_guess(),
        sweep_request.tolerances(),
        sweep_request.maximum_iterations(),
    ) {
        Ok(r) => r,
        Err(_) => {
            return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                CharacterizationUnavailableReason::AlphaPerturbationOutOfBounds {
                    alpha_minus,
                    alpha_plus,
                    lower: alpha_bounds.lower(),
                    upper: alpha_bounds.upper(),
                },
            );
        }
    };

    let base_pitch_moment = trim_eval.residuals.pitch_moment_nm;

    let alpha_minus_vars = LongitudinalTrimVariables {
        alpha_rad: alpha_minus,
        elevator_command: elevator0,
        throttle: throttle0,
    };
    let alpha_plus_vars = LongitudinalTrimVariables {
        alpha_rad: alpha_plus,
        elevator_command: elevator0,
        throttle: throttle0,
    };
    let elevator_minus_vars = LongitudinalTrimVariables {
        alpha_rad: alpha0,
        elevator_command: elevator_minus,
        throttle: throttle0,
    };
    let elevator_plus_vars = LongitudinalTrimVariables {
        alpha_rad: alpha0,
        elevator_command: elevator_plus,
        throttle: throttle0,
    };

    let alpha_minus_eval =
        match evaluate_longitudinal_trim_candidate(model, config, &request, alpha_minus_vars) {
            Some(eval) => eval,
            None => {
                return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                    CharacterizationUnavailableReason::AlphaPerturbationNonFinite {
                        side: PerturbationSide::Minus,
                    },
                );
            }
        };

    let alpha_plus_eval =
        match evaluate_longitudinal_trim_candidate(model, config, &request, alpha_plus_vars) {
            Some(eval) => eval,
            None => {
                return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                    CharacterizationUnavailableReason::AlphaPerturbationNonFinite {
                        side: PerturbationSide::Plus,
                    },
                );
            }
        };

    let elevator_minus_eval =
        match evaluate_longitudinal_trim_candidate(model, config, &request, elevator_minus_vars) {
            Some(eval) => eval,
            None => {
                return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                    CharacterizationUnavailableReason::ElevatorPerturbationNonFinite {
                        side: PerturbationSide::Minus,
                    },
                );
            }
        };

    let elevator_plus_eval =
        match evaluate_longitudinal_trim_candidate(model, config, &request, elevator_plus_vars) {
            Some(eval) => eval,
            None => {
                return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                    CharacterizationUnavailableReason::ElevatorPerturbationNonFinite {
                        side: PerturbationSide::Plus,
                    },
                );
            }
        };

    let my_alpha_minus = alpha_minus_eval.residuals.pitch_moment_nm;
    let my_alpha_plus = alpha_plus_eval.residuals.pitch_moment_nm;
    let my_elevator_minus = elevator_minus_eval.residuals.pitch_moment_nm;
    let my_elevator_plus = elevator_plus_eval.residuals.pitch_moment_nm;

    let dmy_dalpha = (my_alpha_plus - my_alpha_minus) / (2.0 * steps.alpha_step_rad);
    let dmy_delevator = (my_elevator_plus - my_elevator_minus) / (2.0 * steps.elevator_step);

    if !dmy_dalpha.is_finite() {
        return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
            CharacterizationUnavailableReason::NonFinitePitchStiffness,
        );
    }

    if !dmy_delevator.is_finite() {
        return LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
            CharacterizationUnavailableReason::NonFiniteElevatorEffectiveness,
        );
    }

    let data = LongitudinalTrimCharacterizationData {
        target_airspeed_mps: target_airspeed,
        alpha_rad: alpha0,
        elevator_command: elevator0,
        throttle: throttle0,
        alpha_step_rad: steps.alpha_step_rad,
        elevator_step: steps.elevator_step,
        pitch_moment_at_trim_nm: base_pitch_moment,
        alpha_minus_pitch_moment_nm: my_alpha_minus,
        alpha_plus_pitch_moment_nm: my_alpha_plus,
        elevator_minus_pitch_moment_nm: my_elevator_minus,
        elevator_plus_pitch_moment_nm: my_elevator_plus,
        pitch_stiffness_nm_per_rad: dmy_dalpha,
        elevator_effectiveness_nm_per_command: dmy_delevator,
    };

    LongitudinalTrimCharacterizationPointOutcome::Characterized(data)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trim::{LongitudinalTrimTolerances, LongitudinalTrimVariables, TrimBounds};
    use crate::trim_sweep::{LongitudinalTrimSweepOutcome, LongitudinalTrimSweepRequest};

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

    fn sweep_request_and_sweep() -> (
        LongitudinalTrimSweepRequest,
        crate::trim_sweep::LongitudinalTrimSweep,
    ) {
        let model = aircraft();
        let config = sim_config();
        let speeds = vec![15.0, 18.0, 21.0];
        let alpha_bounds = TrimBounds::new(0.02, 0.20).unwrap();
        let elevator_bounds = TrimBounds::new(-0.5, 0.5).unwrap();
        let throttle_bounds = TrimBounds::new(0.0, 1.0).unwrap();
        let initial_guess = LongitudinalTrimVariables::new(0.05, 0.0, 0.5).unwrap();
        let tolerances = LongitudinalTrimTolerances::new(5.0, 2.0).unwrap();
        let request = LongitudinalTrimSweepRequest::new(
            speeds,
            alpha_bounds,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            50,
        )
        .unwrap();
        let sweep =
            crate::trim_sweep::solve_longitudinal_trim_sweep(&model, &config, &request).unwrap();
        (request, sweep)
    }

    #[test]
    fn alpha_step_must_be_finite_and_positive() {
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(0.0, 0.01).unwrap_err(),
            CharacterizationStepsError::InvalidAlphaStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(-0.01, 0.01).unwrap_err(),
            CharacterizationStepsError::InvalidAlphaStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(f64::NAN, 0.01).unwrap_err(),
            CharacterizationStepsError::InvalidAlphaStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(f64::INFINITY, 0.01).unwrap_err(),
            CharacterizationStepsError::InvalidAlphaStep
        );
    }

    #[test]
    fn elevator_step_must_be_finite_and_positive() {
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(0.01, 0.0).unwrap_err(),
            CharacterizationStepsError::InvalidElevatorStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(0.01, -0.01).unwrap_err(),
            CharacterizationStepsError::InvalidElevatorStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(0.01, f64::NAN).unwrap_err(),
            CharacterizationStepsError::InvalidElevatorStep
        );
        assert_eq!(
            LongitudinalTrimCharacterizationSteps::new(0.01, f64::INFINITY).unwrap_err(),
            CharacterizationStepsError::InvalidElevatorStep
        );
    }

    #[test]
    fn valid_steps_construct_successfully() {
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();
        assert_eq!(steps.alpha_step_rad(), 0.001);
        assert_eq!(steps.elevator_step(), 0.01);
    }

    #[test]
    fn sweep_length_mismatch_fails_closed() {
        let model = aircraft();
        let config = sim_config();
        let (request, _sweep) = sweep_request_and_sweep();

        let shorter_speeds = vec![15.0, 18.0];
        let shorter_request = LongitudinalTrimSweepRequest::new(
            shorter_speeds,
            request.alpha_bounds_rad(),
            request.elevator_bounds(),
            request.throttle_bounds(),
            request.initial_guess(),
            request.tolerances(),
            request.maximum_iterations(),
        )
        .unwrap();
        let shorter_sweep =
            crate::trim_sweep::solve_longitudinal_trim_sweep(&model, &config, &shorter_request)
                .unwrap();

        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();
        let result =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &shorter_sweep, steps);
        assert!(matches!(
            result,
            Err(LongitudinalTrimCharacterizationError::SweepLengthMismatch {
                sweep_len: 2,
                request_len: 3,
            })
        ));
    }

    #[test]
    fn sweep_target_airspeed_mismatch_fails_closed() {
        let model = aircraft();
        let config = sim_config();
        let (request, _sweep) = sweep_request_and_sweep();

        let different_speeds = vec![15.0, 18.0, 25.0];
        let different_request = LongitudinalTrimSweepRequest::new(
            different_speeds,
            request.alpha_bounds_rad(),
            request.elevator_bounds(),
            request.throttle_bounds(),
            request.initial_guess(),
            request.tolerances(),
            request.maximum_iterations(),
        )
        .unwrap();
        let different_sweep =
            crate::trim_sweep::solve_longitudinal_trim_sweep(&model, &config, &different_request)
                .unwrap();

        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();
        let result = characterize_longitudinal_trim_sweep(
            &model,
            &config,
            &request,
            &different_sweep,
            steps,
        );
        assert!(matches!(
            result,
            Err(
                LongitudinalTrimCharacterizationError::SweepTargetAirspeedMismatch { index: 2, .. }
            )
        ));
    }

    #[test]
    fn successful_sweep_produces_characterization_in_order() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        assert_eq!(characterization.len(), 3);
        let speeds: Vec<f64> = characterization
            .points()
            .iter()
            .map(|p| p.target_airspeed_mps)
            .collect();
        assert_eq!(speeds, vec![15.0, 18.0, 21.0]);

        for point in characterization.points() {
            assert!(
                matches!(
                    point.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::Characterized(_)
                ),
                "expected Characterized, got {:?}",
                point.outcome
            );
        }
    }

    #[test]
    fn all_characterized_derivatives_are_finite() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for point in characterization.points() {
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                assert!(data.pitch_stiffness_nm_per_rad.is_finite());
                assert!(data.elevator_effectiveness_nm_per_command.is_finite());
                assert!(data.alpha_minus_pitch_moment_nm.is_finite());
                assert!(data.alpha_plus_pitch_moment_nm.is_finite());
                assert!(data.elevator_minus_pitch_moment_nm.is_finite());
                assert!(data.elevator_plus_pitch_moment_nm.is_finite());
                assert!(data.pitch_moment_at_trim_nm.is_finite());
            }
        }
    }

    #[test]
    fn dmy_dalpha_equals_central_difference_formula() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for point in characterization.points() {
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                let expected = (data.alpha_plus_pitch_moment_nm - data.alpha_minus_pitch_moment_nm)
                    / (2.0 * data.alpha_step_rad);
                assert!(
                    (data.pitch_stiffness_nm_per_rad - expected).abs() < 1e-12,
                    "dMy/dAlpha mismatch at {} m/s: {} vs {}",
                    data.target_airspeed_mps,
                    data.pitch_stiffness_nm_per_rad,
                    expected
                );
            }
        }
    }

    #[test]
    fn dmy_delevator_equals_central_difference_formula() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for point in characterization.points() {
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                let expected = (data.elevator_plus_pitch_moment_nm
                    - data.elevator_minus_pitch_moment_nm)
                    / (2.0 * data.elevator_step);
                assert!(
                    (data.elevator_effectiveness_nm_per_command - expected).abs() < 1e-12,
                    "dMy/dElevator mismatch at {} m/s: {} vs {}",
                    data.target_airspeed_mps,
                    data.elevator_effectiveness_nm_per_command,
                    expected
                );
            }
        }
    }

    #[test]
    fn alpha_perturbation_freezes_elevator_and_throttle() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for point in characterization.points() {
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                assert_eq!(
                    data.elevator_command,
                    sweep.points()[characterization
                        .points()
                        .iter()
                        .position(|p| p.target_airspeed_mps == data.target_airspeed_mps)
                        .unwrap()]
                    .outcome
                    .as_success()
                    .unwrap()
                    .evaluation
                    .variables
                    .elevator_command
                );
                assert_eq!(
                    data.throttle,
                    sweep.points()[characterization
                        .points()
                        .iter()
                        .position(|p| p.target_airspeed_mps == data.target_airspeed_mps)
                        .unwrap()]
                    .outcome
                    .as_success()
                    .unwrap()
                    .evaluation
                    .variables
                    .throttle
                );
            }
        }
    }

    #[test]
    fn elevator_perturbation_freezes_alpha_and_throttle() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for point in characterization.points() {
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                let sweep_idx = characterization
                    .points()
                    .iter()
                    .position(|p| p.target_airspeed_mps == data.target_airspeed_mps)
                    .unwrap();
                let trim_vars = &sweep.points()[sweep_idx]
                    .outcome
                    .as_success()
                    .unwrap()
                    .evaluation
                    .variables;
                assert_eq!(data.alpha_rad, trim_vars.alpha_rad);
                assert_eq!(data.throttle, trim_vars.throttle);
            }
        }
    }

    #[test]
    fn target_airspeed_unchanged_during_perturbations() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        for (idx, point) in characterization.points().iter().enumerate() {
            assert_eq!(
                point.target_airspeed_mps,
                sweep.points()[idx].target_airspeed_mps
            );
            if let LongitudinalTrimCharacterizationPointOutcome::Characterized(data) =
                &point.outcome
            {
                assert_eq!(data.target_airspeed_mps, point.target_airspeed_mps);
            }
        }
    }

    #[test]
    fn too_large_alpha_step_produces_unavailable() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.5, 0.01).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        let unavailable_count = characterization
            .points()
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                        CharacterizationUnavailableReason::AlphaPerturbationOutOfBounds { .. }
                    )
                )
            })
            .count();
        assert!(
            unavailable_count > 0,
            "expected at least one AlphaPerturbationOutOfBounds"
        );
    }

    #[test]
    fn too_large_elevator_step_produces_unavailable() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.8).unwrap();

        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        let unavailable_count = characterization
            .points()
            .iter()
            .filter(|p| {
                matches!(
                    p.outcome,
                    LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                        CharacterizationUnavailableReason::ElevatorPerturbationOutOfBounds { .. }
                    )
                )
            })
            .count();
        assert!(
            unavailable_count > 0,
            "expected at least one ElevatorPerturbationOutOfBounds"
        );
    }

    #[test]
    fn trim_failure_point_is_not_characterized() {
        let model = aircraft();
        let config = sim_config();
        let speeds = vec![18.0];
        let alpha_bounds = TrimBounds::new(0.0, 0.000001).unwrap();
        let elevator_bounds = TrimBounds::new(-0.5, 0.5).unwrap();
        let throttle_bounds = TrimBounds::new(0.0, 1.0).unwrap();
        let initial_guess = LongitudinalTrimVariables::new(0.0, 0.0, 0.5).unwrap();
        let tolerances = LongitudinalTrimTolerances::new(0.001, 0.0001).unwrap();
        let request = LongitudinalTrimSweepRequest::new(
            speeds,
            alpha_bounds,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            5,
        )
        .unwrap();
        let sweep =
            crate::trim_sweep::solve_longitudinal_trim_sweep(&model, &config, &request).unwrap();

        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();
        let characterization =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        assert_eq!(characterization.len(), 1);
        assert_eq!(characterization.trim_failure_not_characterized_count(), 1);
        assert_eq!(characterization.characterized_count(), 0);

        match &characterization.points()[0].outcome {
            LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedTrimFailure => {}
            other => panic!("expected NotCharacterizedTrimFailure, got {:?}", other),
        }
    }

    #[test]
    fn identical_inputs_produce_identical_outputs() {
        let model = aircraft();
        let config = sim_config();
        let (request, sweep) = sweep_request_and_sweep();
        let steps = LongitudinalTrimCharacterizationSteps::new(0.001, 0.01).unwrap();

        let c1 =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();
        let c2 =
            characterize_longitudinal_trim_sweep(&model, &config, &request, &sweep, steps).unwrap();

        assert_eq!(c1, c2);
    }

    #[test]
    fn synthetic_fixture_remains_synthetic_test() {
        let model = aircraft();
        assert_eq!(model.model_id(), "synthetic_non_reference_trim_v4");
    }

    impl LongitudinalTrimSweepOutcome {
        fn as_success(&self) -> Option<&crate::trim::LongitudinalTrimSolution> {
            match self {
                Self::Success { solution } => Some(solution),
                _ => None,
            }
        }
    }
}
