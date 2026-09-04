//! Deterministic bounded longitudinal trim over the complete aircraft runtime physics.

use crate::{
    AircraftSimulationConfig, effective_aero_elements_for_positions,
    evaluate_aircraft_instantaneous,
};
use model::AircraftModel;
use sim_core::{
    BodyWrench, ControlSurfacePositions, PilotInput, RigidBodyDerivative, RigidBodyState,
    evaluate_steady_controls,
};
use sim_math::{Mat3, Orientation, Quaternion, Vec3};
use thiserror::Error;

const FINITE_DIFFERENCE_RELATIVE_STEP: f64 = 1.0e-5;
const FINITE_DIFFERENCE_ABSOLUTE_FLOOR: f64 = 1.0e-7;
const JACOBIAN_RELATIVE_SINGULAR_VALUE_LIMIT: f64 = 1.0e-10;
const MAX_LINE_SEARCH_HALVINGS: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum LongitudinalTrimRequestError {
    #[error("trim bounds must be finite and satisfy lower < upper")]
    InvalidBounds,
    #[error("target airspeed must be finite and greater than zero")]
    InvalidTargetAirspeed,
    #[error("elevator command bounds must lie within [-1, 1]")]
    InvalidElevatorBounds,
    #[error("throttle bounds must lie within [0, 1]")]
    InvalidThrottleBounds,
    #[error("initial guess must contain only finite values")]
    NonFiniteInitialGuess,
    #[error("force and moment tolerances must be finite and greater than zero")]
    InvalidTolerance,
    #[error("maximum iteration count must be greater than zero")]
    InvalidIterationLimit,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrimBounds {
    lower: f64,
    upper: f64,
}

impl TrimBounds {
    pub fn new(lower: f64, upper: f64) -> Result<Self, LongitudinalTrimRequestError> {
        if !lower.is_finite() || !upper.is_finite() || lower >= upper {
            return Err(LongitudinalTrimRequestError::InvalidBounds);
        }
        Ok(Self { lower, upper })
    }

    #[must_use]
    pub const fn lower(&self) -> f64 {
        self.lower
    }

    #[must_use]
    pub const fn upper(&self) -> f64 {
        self.upper
    }

    fn clamp(self, value: f64) -> f64 {
        value.clamp(self.lower, self.upper)
    }

    fn span(self) -> f64 {
        self.upper - self.lower
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimVariables {
    pub alpha_rad: f64,
    pub elevator_command: f64,
    pub throttle: f64,
}

impl LongitudinalTrimVariables {
    pub fn new(
        alpha_rad: f64,
        elevator_command: f64,
        throttle: f64,
    ) -> Result<Self, LongitudinalTrimRequestError> {
        if ![alpha_rad, elevator_command, throttle]
            .into_iter()
            .all(f64::is_finite)
        {
            return Err(LongitudinalTrimRequestError::NonFiniteInitialGuess);
        }
        Ok(Self {
            alpha_rad,
            elevator_command,
            throttle,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimTolerances {
    pub force_n: f64,
    pub pitch_moment_nm: f64,
}

impl LongitudinalTrimTolerances {
    pub fn new(force_n: f64, pitch_moment_nm: f64) -> Result<Self, LongitudinalTrimRequestError> {
        if !force_n.is_finite()
            || force_n <= 0.0
            || !pitch_moment_nm.is_finite()
            || pitch_moment_nm <= 0.0
        {
            return Err(LongitudinalTrimRequestError::InvalidTolerance);
        }
        Ok(Self {
            force_n,
            pitch_moment_nm,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimRequest {
    target_airspeed_mps: f64,
    alpha_bounds_rad: TrimBounds,
    elevator_bounds: TrimBounds,
    throttle_bounds: TrimBounds,
    initial_guess: LongitudinalTrimVariables,
    tolerances: LongitudinalTrimTolerances,
    maximum_iterations: usize,
}

impl LongitudinalTrimRequest {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        target_airspeed_mps: f64,
        alpha_bounds_rad: TrimBounds,
        elevator_bounds: TrimBounds,
        throttle_bounds: TrimBounds,
        initial_guess: LongitudinalTrimVariables,
        tolerances: LongitudinalTrimTolerances,
        maximum_iterations: usize,
    ) -> Result<Self, LongitudinalTrimRequestError> {
        if !target_airspeed_mps.is_finite() || target_airspeed_mps <= 0.0 {
            return Err(LongitudinalTrimRequestError::InvalidTargetAirspeed);
        }
        if elevator_bounds.lower < -1.0 || elevator_bounds.upper > 1.0 {
            return Err(LongitudinalTrimRequestError::InvalidElevatorBounds);
        }
        if throttle_bounds.lower < 0.0 || throttle_bounds.upper > 1.0 {
            return Err(LongitudinalTrimRequestError::InvalidThrottleBounds);
        }
        if maximum_iterations == 0 {
            return Err(LongitudinalTrimRequestError::InvalidIterationLimit);
        }
        let initial_guess = LongitudinalTrimVariables {
            alpha_rad: alpha_bounds_rad.clamp(initial_guess.alpha_rad),
            elevator_command: elevator_bounds.clamp(initial_guess.elevator_command),
            throttle: throttle_bounds.clamp(initial_guess.throttle),
        };
        Ok(Self {
            target_airspeed_mps,
            alpha_bounds_rad,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            maximum_iterations,
        })
    }

    #[must_use]
    pub const fn target_airspeed_mps(&self) -> f64 {
        self.target_airspeed_mps
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

    fn bounds(&self) -> [TrimBounds; 3] {
        [
            self.alpha_bounds_rad,
            self.elevator_bounds,
            self.throttle_bounds,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimResiduals {
    /// `mass * a_world_x`, including gravity through the runtime derivative.
    pub longitudinal_force_n: f64,
    /// `mass * a_world_z` in NED, including gravity through the runtime derivative.
    pub vertical_force_n: f64,
    /// Total body-Y moment about the model CG.
    pub pitch_moment_nm: f64,
}

impl LongitudinalTrimResiduals {
    #[must_use]
    pub fn is_within(&self, tolerances: LongitudinalTrimTolerances) -> bool {
        self.longitudinal_force_n.abs() <= tolerances.force_n
            && self.vertical_force_n.abs() <= tolerances.force_n
            && self.pitch_moment_nm.abs() <= tolerances.pitch_moment_nm
    }

    fn scaled(self, tolerances: LongitudinalTrimTolerances) -> Vec3 {
        Vec3::new(
            self.longitudinal_force_n / tolerances.force_n,
            self.vertical_force_n / tolerances.force_n,
            self.pitch_moment_nm / tolerances.pitch_moment_nm,
        )
    }

    #[must_use]
    pub fn scaled_infinity_norm(self, tolerances: LongitudinalTrimTolerances) -> f64 {
        self.scaled(tolerances).abs().max()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimEvaluation {
    pub variables: LongitudinalTrimVariables,
    pub pitch_attitude_rad: f64,
    pub state: RigidBodyState,
    pub control_surface_positions: ControlSurfacePositions,
    pub body_wrench: BodyWrench,
    pub derivative: RigidBodyDerivative,
    pub residuals: LongitudinalTrimResiduals,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimSolution {
    pub evaluation: LongitudinalTrimEvaluation,
    pub iteration_count: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LongitudinalTrimFailureReason {
    NoFeasibleSolution,
    SingularJacobian,
    IterationLimit,
    NonFiniteEvaluation,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimFailure {
    pub reason: LongitudinalTrimFailureReason,
    pub iteration_count: usize,
    pub last_evaluation: Option<Box<LongitudinalTrimEvaluation>>,
}

/// Solves bounded `[alpha, elevator command, throttle]` using a deterministic finite-difference
/// Newton method and a fixed halving line search.
pub fn solve_longitudinal_trim(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
) -> Result<LongitudinalTrimSolution, LongitudinalTrimFailure> {
    let mut variables = request.initial_guess;
    let Some(mut evaluation) = evaluate_candidate(model, config, request, variables) else {
        return Err(failure(
            LongitudinalTrimFailureReason::NonFiniteEvaluation,
            0,
            None,
        ));
    };
    if evaluation.residuals.is_within(request.tolerances) {
        return Ok(LongitudinalTrimSolution {
            evaluation,
            iteration_count: 0,
        });
    }

    for iteration in 0..request.maximum_iterations {
        let Some(jacobian) = finite_difference_jacobian(model, config, request, variables) else {
            return Err(failure(
                LongitudinalTrimFailureReason::NonFiniteEvaluation,
                iteration,
                Some(evaluation),
            ));
        };
        let singular_values = jacobian.svd(false, false).singular_values;
        let largest = singular_values.max();
        let smallest = singular_values.min();
        if !largest.is_finite()
            || !smallest.is_finite()
            || largest == 0.0
            || smallest <= largest * JACOBIAN_RELATIVE_SINGULAR_VALUE_LIMIT
        {
            return Err(failure(
                LongitudinalTrimFailureReason::SingularJacobian,
                iteration,
                Some(evaluation),
            ));
        }
        let scaled_residual = evaluation.residuals.scaled(request.tolerances);
        let Some(newton_step) = jacobian.lu().solve(&(-scaled_residual)) else {
            return Err(failure(
                LongitudinalTrimFailureReason::SingularJacobian,
                iteration,
                Some(evaluation),
            ));
        };
        if !newton_step.iter().all(|value| value.is_finite()) {
            return Err(failure(
                LongitudinalTrimFailureReason::NonFiniteEvaluation,
                iteration,
                Some(evaluation),
            ));
        }

        let current_norm = evaluation
            .residuals
            .scaled_infinity_norm(request.tolerances);
        let mut accepted = None;
        let mut saw_non_finite = false;
        for halving in 0..=MAX_LINE_SEARCH_HALVINGS {
            let damping = 0.5_f64.powi(i32::try_from(halving).expect("small line-search index"));
            let candidate_variables =
                clamp_variables(add_step(variables, newton_step * damping), request.bounds());
            if candidate_variables == variables {
                continue;
            }
            match evaluate_candidate(model, config, request, candidate_variables) {
                Some(candidate)
                    if candidate.residuals.scaled_infinity_norm(request.tolerances)
                        < current_norm =>
                {
                    accepted = Some((candidate_variables, candidate));
                    break;
                }
                Some(_) => {}
                None => saw_non_finite = true,
            }
        }
        let Some((next_variables, next_evaluation)) = accepted else {
            return Err(failure(
                if saw_non_finite {
                    LongitudinalTrimFailureReason::NonFiniteEvaluation
                } else {
                    LongitudinalTrimFailureReason::NoFeasibleSolution
                },
                iteration + 1,
                Some(evaluation),
            ));
        };
        variables = next_variables;
        evaluation = next_evaluation;
        if evaluation.residuals.is_within(request.tolerances) {
            return Ok(LongitudinalTrimSolution {
                evaluation,
                iteration_count: iteration + 1,
            });
        }
    }

    Err(failure(
        LongitudinalTrimFailureReason::IterationLimit,
        request.maximum_iterations,
        Some(evaluation),
    ))
}

/// Independently evaluates one bounded candidate through steady controls and runtime stage physics.
#[must_use]
pub fn evaluate_longitudinal_trim_candidate(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
    variables: LongitudinalTrimVariables,
) -> Option<LongitudinalTrimEvaluation> {
    let variables = clamp_variables(variables, request.bounds());
    evaluate_candidate(model, config, request, variables)
}

fn evaluate_candidate(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
    variables: LongitudinalTrimVariables,
) -> Option<LongitudinalTrimEvaluation> {
    let pitch_attitude_rad = variables.alpha_rad;
    let state = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: config.aero_environment().wind_velocity_world_mps()
            + Vec3::new(request.target_airspeed_mps, 0.0, 0.0),
        orientation_world_from_body: Orientation::from_axis_angle(
            &Vec3::y_axis(),
            pitch_attitude_rad,
        ),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let input = PilotInput::new(0.0, variables.elevator_command, 0.0, variables.throttle);
    let positions = evaluate_steady_controls(model.controls(), &input);
    let elements = effective_aero_elements_for_positions(model, &positions);
    // Trim is an airborne equilibrium solve: gear contact is excluded so a
    // runway-parked configuration cannot masquerade as a trimmed free-flight state.
    let instantaneous =
        evaluate_aircraft_instantaneous(&state, &elements, model, positions.throttle(), config);
    let derivative = *instantaneous.derivative();
    let body_wrench = *instantaneous.total_wrench();
    let mass_kg = model.rigid_body().mass_kg();
    let residuals = LongitudinalTrimResiduals {
        longitudinal_force_n: mass_kg * derivative.linear_velocity_world_mps2.x,
        vertical_force_n: mass_kg * derivative.linear_velocity_world_mps2.z,
        pitch_moment_nm: body_wrench.moment_body_nm.y,
    };
    let output = LongitudinalTrimEvaluation {
        variables,
        pitch_attitude_rad,
        state,
        control_surface_positions: positions,
        body_wrench,
        derivative,
        residuals,
    };
    if evaluation_is_finite(&output) {
        Some(output)
    } else {
        None
    }
}

fn finite_difference_jacobian(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    request: &LongitudinalTrimRequest,
    variables: LongitudinalTrimVariables,
) -> Option<Mat3> {
    let bounds = request.bounds();
    let values = variable_vector(variables);
    let mut columns = [Vec3::zeros(); 3];
    for index in 0..3 {
        let step = (bounds[index].span() * FINITE_DIFFERENCE_RELATIVE_STEP)
            .max(FINITE_DIFFERENCE_ABSOLUTE_FLOOR);
        let lower_value = (values[index] - step).max(bounds[index].lower);
        let upper_value = (values[index] + step).min(bounds[index].upper);
        if lower_value == upper_value {
            return None;
        }
        let mut lower = values;
        lower[index] = lower_value;
        let mut upper = values;
        upper[index] = upper_value;
        let lower_evaluation =
            evaluate_candidate(model, config, request, variables_from_vector(lower))?;
        let upper_evaluation =
            evaluate_candidate(model, config, request, variables_from_vector(upper))?;
        columns[index] = (upper_evaluation.residuals.scaled(request.tolerances)
            - lower_evaluation.residuals.scaled(request.tolerances))
            / (upper_value - lower_value);
    }
    Some(Mat3::from_columns(&columns))
}

fn clamp_variables(
    variables: LongitudinalTrimVariables,
    bounds: [TrimBounds; 3],
) -> LongitudinalTrimVariables {
    LongitudinalTrimVariables {
        alpha_rad: bounds[0].clamp(variables.alpha_rad),
        elevator_command: bounds[1].clamp(variables.elevator_command),
        throttle: bounds[2].clamp(variables.throttle),
    }
}

fn add_step(variables: LongitudinalTrimVariables, step: Vec3) -> LongitudinalTrimVariables {
    LongitudinalTrimVariables {
        alpha_rad: variables.alpha_rad + step.x,
        elevator_command: variables.elevator_command + step.y,
        throttle: variables.throttle + step.z,
    }
}

fn variable_vector(variables: LongitudinalTrimVariables) -> Vec3 {
    Vec3::new(
        variables.alpha_rad,
        variables.elevator_command,
        variables.throttle,
    )
}

fn variables_from_vector(values: Vec3) -> LongitudinalTrimVariables {
    LongitudinalTrimVariables {
        alpha_rad: values.x,
        elevator_command: values.y,
        throttle: values.z,
    }
}

fn evaluation_is_finite(evaluation: &LongitudinalTrimEvaluation) -> bool {
    let orientation_derivative = &evaluation.derivative.orientation_world_from_body_per_s;
    evaluation.state.validate().is_ok()
        && evaluation
            .body_wrench
            .force_body_n
            .iter()
            .chain(evaluation.body_wrench.moment_body_nm.iter())
            .chain(evaluation.derivative.linear_velocity_world_mps2.iter())
            .chain(evaluation.derivative.angular_velocity_body_radps2.iter())
            .all(|value| value.is_finite())
        && [
            orientation_derivative.w,
            orientation_derivative.i,
            orientation_derivative.j,
            orientation_derivative.k,
        ]
        .into_iter()
        .all(f64::is_finite)
        && [
            evaluation.residuals.longitudinal_force_n,
            evaluation.residuals.vertical_force_n,
            evaluation.residuals.pitch_moment_nm,
        ]
        .into_iter()
        .all(f64::is_finite)
}

fn failure(
    reason: LongitudinalTrimFailureReason,
    iteration_count: usize,
    last_evaluation: Option<LongitudinalTrimEvaluation>,
) -> LongitudinalTrimFailure {
    LongitudinalTrimFailure {
        reason,
        iteration_count,
        last_evaluation: last_evaluation.map(Box::new),
    }
}

/// Bitwise comparison of two [`LongitudinalTrimEvaluation`] values.
///
/// Unlike `PartialEq` (derived), which uses `f64::eq` where `+0.0 == -0.0`, this
/// function compares every floating-point field via `f64::to_bits()`. A sign-different
/// zero pair (`+0.0` vs `-0.0`) is therefore treated as distinct, matching the M2.6C
/// re-evaluation contract: a cached evaluation and an independent re-evaluation must
/// be bitwise identical, not merely numerically equal.
///
/// Covers every floating-point field in the evaluation: trim variables, pitch attitude,
/// rigid-body state (position, velocity, Hamilton quaternion `wxyz`, angular velocity),
/// control surface positions, body wrench, derivative (including quaternion derivative
/// `wxyz`), and residuals. No allocations, no tolerances, no serialization.
#[must_use]
pub fn evaluations_bitwise_equal(
    a: &LongitudinalTrimEvaluation,
    b: &LongitudinalTrimEvaluation,
) -> bool {
    f64_bits_eq(a.variables.alpha_rad, b.variables.alpha_rad)
        && f64_bits_eq(a.variables.elevator_command, b.variables.elevator_command)
        && f64_bits_eq(a.variables.throttle, b.variables.throttle)
        && f64_bits_eq(a.pitch_attitude_rad, b.pitch_attitude_rad)
        && vec3_bits_eq(&a.state.position_world_m, &b.state.position_world_m)
        && vec3_bits_eq(
            &a.state.linear_velocity_world_mps,
            &b.state.linear_velocity_world_mps,
        )
        && orientation_bits_eq(
            &a.state.orientation_world_from_body,
            &b.state.orientation_world_from_body,
        )
        && vec3_bits_eq(
            &a.state.angular_velocity_body_radps,
            &b.state.angular_velocity_body_radps,
        )
        && f64_bits_eq(
            a.control_surface_positions.aileron_angle_rad(),
            b.control_surface_positions.aileron_angle_rad(),
        )
        && f64_bits_eq(
            a.control_surface_positions.elevator_angle_rad(),
            b.control_surface_positions.elevator_angle_rad(),
        )
        && f64_bits_eq(
            a.control_surface_positions.rudder_angle_rad(),
            b.control_surface_positions.rudder_angle_rad(),
        )
        && f64_bits_eq(
            a.control_surface_positions.throttle(),
            b.control_surface_positions.throttle(),
        )
        && vec3_bits_eq(&a.body_wrench.force_body_n, &b.body_wrench.force_body_n)
        && vec3_bits_eq(&a.body_wrench.moment_body_nm, &b.body_wrench.moment_body_nm)
        && vec3_bits_eq(
            &a.derivative.position_world_mps,
            &b.derivative.position_world_mps,
        )
        && vec3_bits_eq(
            &a.derivative.linear_velocity_world_mps2,
            &b.derivative.linear_velocity_world_mps2,
        )
        && quaternion_bits_eq(
            a.derivative.orientation_world_from_body_per_s,
            b.derivative.orientation_world_from_body_per_s,
        )
        && vec3_bits_eq(
            &a.derivative.angular_velocity_body_radps2,
            &b.derivative.angular_velocity_body_radps2,
        )
        && f64_bits_eq(
            a.residuals.longitudinal_force_n,
            b.residuals.longitudinal_force_n,
        )
        && f64_bits_eq(a.residuals.vertical_force_n, b.residuals.vertical_force_n)
        && f64_bits_eq(a.residuals.pitch_moment_nm, b.residuals.pitch_moment_nm)
}

#[inline]
const fn f64_bits_eq(a: f64, b: f64) -> bool {
    a.to_bits() == b.to_bits()
}

#[inline]
fn vec3_bits_eq(a: &Vec3, b: &Vec3) -> bool {
    f64_bits_eq(a.x, b.x) && f64_bits_eq(a.y, b.y) && f64_bits_eq(a.z, b.z)
}

#[inline]
fn quaternion_bits_eq(a: Quaternion<f64>, b: Quaternion<f64>) -> bool {
    f64_bits_eq(a.w, b.w) && f64_bits_eq(a.i, b.i) && f64_bits_eq(a.j, b.j) && f64_bits_eq(a.k, b.k)
}

#[inline]
fn orientation_bits_eq(a: &Orientation, b: &Orientation) -> bool {
    quaternion_bits_eq(*a.quaternion(), *b.quaternion())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AircraftSimulationConfig;
    use sim_core::AeroEnvironment;

    const FIXTURE: &str =
        include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");

    fn aircraft() -> model::AircraftModel {
        model::AircraftModelLoader::from_json_str(FIXTURE).unwrap()
    }

    fn sim_config() -> AircraftSimulationConfig {
        AircraftSimulationConfig::new(
            0.002,
            Vec3::new(0.0, 0.0, 9.80665),
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap()
    }

    fn solved_evaluation() -> LongitudinalTrimEvaluation {
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
    fn bitwise_equal_identical_evaluations() {
        let eval = solved_evaluation();
        let copy = eval;
        assert!(
            evaluations_bitwise_equal(&eval, &copy),
            "two evaluations from the same solve must be bitwise identical"
        );
        assert_eq!(
            eval, copy,
            "PartialEq must also agree for truly identical evaluations"
        );
    }

    #[test]
    fn bitwise_equal_detects_signed_zero_difference_in_state_position() {
        let eval = solved_evaluation();
        // position_world_m is Vec3::zeros() from evaluate_candidate — all +0.0.
        // Flip x to -0.0: PartialEq still says equal (+0.0 == -0.0), but bitwise must not.
        let mut flipped = eval;
        flipped.state.position_world_m.x = -0.0;

        assert_eq!(
            eval, flipped,
            "sanity: PartialEq treats +0.0 and -0.0 as equal (the bug we are fixing)"
        );
        assert!(
            !evaluations_bitwise_equal(&eval, &flipped),
            "bitwise comparison MUST reject +0.0 vs -0.0 in position_world_m.x"
        );
    }

    #[test]
    fn bitwise_equal_detects_signed_zero_difference_in_residual() {
        let eval = solved_evaluation();
        let mut flipped = eval;
        flipped.residuals.longitudinal_force_n = -0.0;
        // If the solved residual was already +0.0, flipping to -0.0 must be caught.
        // If it was non-zero, this test still verifies that a real difference is caught.
        let mut zeroed = eval;
        zeroed.residuals.longitudinal_force_n = 0.0;
        if flipped.residuals.longitudinal_force_n.to_bits()
            != zeroed.residuals.longitudinal_force_n.to_bits()
        {
            assert!(
                !evaluations_bitwise_equal(&zeroed, &flipped),
                "bitwise comparison MUST reject +0.0 vs -0.0 in residuals"
            );
        }
    }

    #[test]
    fn bitwise_equal_detects_signed_zero_difference_in_quaternion() {
        let eval = solved_evaluation();
        let mut flipped = eval;
        flipped.derivative.orientation_world_from_body_per_s.w = -0.0;
        let mut reference = eval;
        reference.derivative.orientation_world_from_body_per_s.w = 0.0;
        if flipped
            .derivative
            .orientation_world_from_body_per_s
            .w
            .to_bits()
            != reference
                .derivative
                .orientation_world_from_body_per_s
                .w
                .to_bits()
        {
            assert!(
                !evaluations_bitwise_equal(&reference, &flipped),
                "bitwise comparison MUST reject +0.0 vs -0.0 in quaternion derivative w"
            );
        }
    }

    #[test]
    fn bitwise_equal_rejects_nonzero_difference() {
        let eval = solved_evaluation();
        let mut different = eval;
        different.variables.alpha_rad += 1.0e-3;
        assert!(
            !evaluations_bitwise_equal(&eval, &different),
            "any real numeric difference must be rejected"
        );
    }

    #[test]
    fn partial_eq_and_bitwise_diverge_on_signed_zero() {
        // This is the core discriminant test: PartialEq says +0.0 == -0.0,
        // but the bitwise comparison must NOT.
        let eval = solved_evaluation();
        let mut flipped = eval;
        flipped.state.angular_velocity_body_radps.y = -0.0;

        // PartialEq treats them as equal (the bug).
        assert_eq!(eval, flipped);
        // Bitwise comparison correctly rejects.
        assert!(!evaluations_bitwise_equal(&eval, &flipped));
    }
}
