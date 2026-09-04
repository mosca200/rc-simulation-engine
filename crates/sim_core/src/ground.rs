#![forbid(unsafe_code)]
//! Native compliant ground-contact model (phase 2).
//!
//! Canonical `RigidBodyState` stays the sole owner of aircraft state.
//! This module only evaluates `F(state)` for the existing RK4 stages.
//! No second rigid body, no external authority, no broad-phase engine.
//!
//! NED/FRD: world North-East-Down (`+Z` down). Flat plane `z = height`,
//! free air `z < height`. Penetration `contact_z - height` (positive
//! below). Outward normal (world) is `[0,0,-1]` (up).

use crate::{PilotInput, RigidBodyState};
use sim_math::{Vec3, body_to_world, world_to_body};
use std::f64::consts::FRAC_PI_2;
use thiserror::Error;

/// Max supported gear contacts.
pub const MAX_GEAR_CONTACTS: usize = 16;
/// Coulomb regularization speed (m/s).
pub const FRICTION_REGULARIZATION_SPEED_MPS: f64 = 0.25;
/// Rejection bound: stiffness above this fails validation.
pub const MAX_NORMAL_STIFFNESS_N_PER_M: f64 = 20_000.0;
/// Rejection bound: damping above this fails validation.
pub const MAX_NORMAL_DAMPING_N_S_PER_M: f64 = 1_500.0;
/// Guideline stiffness: re-validate 500 Hz stability above this.
pub const RECOMMENDED_MAX_NORMAL_STIFFNESS_N_PER_M: f64 = 12_000.0;
/// Guideline damping: re-validate 500 Hz stability above this.
pub const RECOMMENDED_MAX_NORMAL_DAMPING_N_S_PER_M: f64 = 800.0;

/// Flat infinite ground plane at fixed NED down coordinate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlatGroundPlane {
    ground_height_world_down_m: f64,
}

impl FlatGroundPlane {
    pub fn new(height: f64) -> Result<Self, GroundConfigError> {
        if !height.is_finite() {
            return Err(GroundConfigError::NonFiniteGroundHeight);
        }
        Ok(Self {
            ground_height_world_down_m: height,
        })
    }
    #[must_use]
    pub const fn ground_height_world_down_m(&self) -> f64 {
        self.ground_height_world_down_m
    }
    /// Outward (up) normal in NED world.
    #[must_use]
    pub fn outward_normal_world() -> Vec3 {
        Vec3::new(0.0, 0.0, -1.0)
    }
}

impl Default for FlatGroundPlane {
    fn default() -> Self {
        Self {
            ground_height_world_down_m: 0.0,
        }
    }
}

/// Minimal surface abstraction; only flat plane is populated.
/// Future terrain adds variants behind `height_and_normal`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GroundSurface {
    Flat(FlatGroundPlane),
}

impl GroundSurface {
    /// Returns `(height_down_m, outward_normal_world)` for a query point.
    #[must_use]
    pub fn height_and_normal(&self, _position_world_m: &Vec3) -> (f64, Vec3) {
        match self {
            Self::Flat(plane) => (
                plane.ground_height_world_down_m,
                FlatGroundPlane::outward_normal_world(),
            ),
        }
    }
}

impl Default for GroundSurface {
    fn default() -> Self {
        Self::Flat(FlatGroundPlane::default())
    }
}

impl From<FlatGroundPlane> for GroundSurface {
    fn from(plane: FlatGroundPlane) -> Self {
        Self::Flat(plane)
    }
}

/// Steering command source. `Fixed` keeps wheel aligned to airframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SteeringSource {
    Fixed,
    Rudder,
}

impl SteeringSource {
    #[must_use]
    pub const fn is_steerable(self) -> bool {
        matches!(self, Self::Rudder)
    }
}

/// One validated wheel/skid contact. All params SI, validated at load.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GearContact {
    pub position_body_m: Vec3,
    pub wheel_radius_m: f64,
    pub stiffness_n_per_m: f64,
    pub damping_n_s_per_m: f64,
    pub long_mu: f64,
    pub lat_mu: f64,
    pub rolling_mu: f64,
    pub brake_mu: f64,
    pub steering: SteeringSource,
    pub max_steer_rad: f64,
    pub steerable: bool,
    pub braked: bool,
}

/// Config errors are all load-time; hot loop never validates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum GroundConfigError {
    #[error("ground height must be finite")]
    NonFiniteGroundHeight,
    #[error("gear contact position must be finite")]
    NonFiniteContactPosition,
    #[error("wheel radius must be finite and non-negative")]
    InvalidWheelRadius,
    #[error("normal stiffness must be finite, positive, within bounds")]
    InvalidNormalStiffness,
    #[error("normal damping must be finite, non-negative, within bounds")]
    InvalidNormalDamping,
    #[error("friction coefficient must be finite and non-negative")]
    InvalidFriction,
    #[error("max steer angle must be finite within [0, pi/2]")]
    InvalidSteerAngle,
    #[error("steering source requires steerable flag")]
    SteeringWithoutFlag,
    #[error("steerable flag requires a steering source")]
    SteerableWithoutSource,
    #[error("brake authority requires braked flag")]
    BrakeAuthorityWithoutFlag,
    #[error("too many gear contacts")]
    TooManyContacts,
}

/// Validates one contact; called once at model load, never per step.
pub fn validate_gear_contact(contact: &GearContact) -> Result<(), GroundConfigError> {
    if !contact.position_body_m.iter().all(|v| v.is_finite()) {
        return Err(GroundConfigError::NonFiniteContactPosition);
    }
    if !contact.wheel_radius_m.is_finite() || contact.wheel_radius_m < 0.0 {
        return Err(GroundConfigError::InvalidWheelRadius);
    }
    let stiff_ok = contact.stiffness_n_per_m.is_finite()
        && contact.stiffness_n_per_m > 0.0
        && contact.stiffness_n_per_m <= MAX_NORMAL_STIFFNESS_N_PER_M;
    if !stiff_ok {
        return Err(GroundConfigError::InvalidNormalStiffness);
    }
    let damp_ok = contact.damping_n_s_per_m.is_finite()
        && contact.damping_n_s_per_m >= 0.0
        && contact.damping_n_s_per_m <= MAX_NORMAL_DAMPING_N_S_PER_M;
    if !damp_ok {
        return Err(GroundConfigError::InvalidNormalDamping);
    }
    for mu in [
        contact.long_mu,
        contact.lat_mu,
        contact.rolling_mu,
        contact.brake_mu,
    ] {
        if !mu.is_finite() || mu < 0.0 {
            return Err(GroundConfigError::InvalidFriction);
        }
    }
    let steer_ok = contact.max_steer_rad.is_finite()
        && contact.max_steer_rad >= 0.0
        && contact.max_steer_rad <= FRAC_PI_2;
    if !steer_ok {
        return Err(GroundConfigError::InvalidSteerAngle);
    }
    if !contact.steerable && contact.steering.is_steerable() {
        return Err(GroundConfigError::SteeringWithoutFlag);
    }
    if contact.steerable && !contact.steering.is_steerable() {
        return Err(GroundConfigError::SteerableWithoutSource);
    }
    if !contact.braked && contact.brake_mu > 0.0 {
        return Err(GroundConfigError::BrakeAuthorityWithoutFlag);
    }
    Ok(())
}

/// Per-contact stage-local diagnostic (allocation-free, Copy).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundContactSolution {
    pub in_contact: bool,
    pub penetration_m: f64,
    pub normal_force_n: f64,
    pub longitudinal_force_n: f64,
    pub lateral_force_n: f64,
    pub steer_angle_rad: f64,
    pub contact_velocity_body_mps: Vec3,
    pub contact_position_world_m: Vec3,
}

impl GroundContactSolution {
    #[must_use]
    pub fn air() -> Self {
        Self {
            in_contact: false,
            penetration_m: 0.0,
            normal_force_n: 0.0,
            longitudinal_force_n: 0.0,
            lateral_force_n: 0.0,
            steer_angle_rad: 0.0,
            contact_velocity_body_mps: Vec3::zeros(),
            contact_position_world_m: Vec3::zeros(),
        }
    }
}

/// Aggregate stage-local ground solution over the fixed array.
#[derive(Debug, Clone, PartialEq)]
pub struct GroundEvaluation {
    pub force_body_n: Vec3,
    pub moment_body_nm: Vec3,
    pub total_normal_force_n: f64,
    pub total_tangential_force_n: f64,
    pub active_contacts: usize,
    pub contacts: [GroundContactSolution; MAX_GEAR_CONTACTS],
}

impl GroundEvaluation {
    #[must_use]
    pub fn zero() -> Self {
        Self {
            force_body_n: Vec3::zeros(),
            moment_body_nm: Vec3::zeros(),
            total_normal_force_n: 0.0,
            total_tangential_force_n: 0.0,
            active_contacts: 0,
            contacts: [GroundContactSolution::air(); MAX_GEAR_CONTACTS],
        }
    }
    /// Deterministic weight-on-wheels derived from actual contact solution.
    #[must_use]
    pub const fn weight_on_wheels(&self) -> bool {
        self.active_contacts > 0
    }
}

/// Stage-local steering/brake command bundle (held constant across RK4 stages).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundCommand {
    pub rudder_command: f64,
    pub brake_command: f64,
}

impl GroundCommand {
    #[must_use]
    pub fn new(rudder_command: f64, brake_command: f64) -> Self {
        Self {
            rudder_command: clamp_sym(rudder_command),
            brake_command: clamp_unit(brake_command),
        }
    }
    #[must_use]
    pub fn from_pilot(input: &PilotInput, brake_command: f64) -> Self {
        Self::new(input.yaw(), brake_command)
    }
}

fn clamp_sym(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(-1.0, 1.0)
    } else {
        0.0
    }
}

fn clamp_unit(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// World velocity of a body-fixed point: `V_cg + omega_world x r_world`.
#[must_use]
pub fn contact_point_velocity_world(
    state: &RigidBodyState,
    contact_position_body_m: &Vec3,
) -> Vec3 {
    let offset_world = body_to_world(&state.orientation_world_from_body, contact_position_body_m);
    let omega_world = body_to_world(
        &state.orientation_world_from_body,
        &state.angular_velocity_body_radps,
    );
    state.linear_velocity_world_mps + omega_world.cross(&offset_world)
}

/// Steering angle for one contact from the held stage command.
#[must_use]
pub fn steering_angle_rad(contact: &GearContact, command: &GroundCommand) -> f64 {
    if !contact.steerable {
        return 0.0;
    }
    match contact.steering {
        SteeringSource::Fixed => 0.0,
        SteeringSource::Rudder => command.rudder_command * contact.max_steer_rad,
    }
}

/// Regularized Coulomb: linear below the speed, saturated above.
/// Exactly zero at zero slip; never NaN.
fn regularized_coulomb(slip_mps: f64, cap_n: f64) -> f64 {
    if cap_n <= 0.0 || !slip_mps.is_finite() {
        return 0.0;
    }
    let magnitude = slip_mps.abs();
    if magnitude >= FRICTION_REGULARIZATION_SPEED_MPS {
        cap_n * slip_mps.signum()
    } else {
        cap_n * (magnitude / FRICTION_REGULARIZATION_SPEED_MPS) * slip_mps.signum()
    }
}

/// Evaluates the deterministic ground wrench for one RK4 stage state.
/// Pure function of `(stage_state, gear, surface, command)`.
/// Allocation-free: iterates the caller slice, writes the fixed array.
#[must_use]
pub fn evaluate_ground_wrench(
    state: &RigidBodyState,
    gear: &[GearContact],
    surface: &GroundSurface,
    command: &GroundCommand,
) -> GroundEvaluation {
    let mut evaluation = GroundEvaluation::zero();
    if gear.is_empty() {
        return evaluation;
    }
    let mut force_body = Vec3::zeros();
    let mut moment_body = Vec3::zeros();
    let mut tangential = 0.0;
    let count = gear.len().min(MAX_GEAR_CONTACTS);
    for (index, contact) in gear.iter().enumerate().take(count) {
        let steer = steering_angle_rad(contact, command);
        let solution = evaluate_single_contact(state, contact, surface, steer, command);
        evaluation.contacts[index] = solution;
        if solution.in_contact {
            evaluation.active_contacts += 1;
            evaluation.total_normal_force_n += solution.normal_force_n;
            tangential += solution.longitudinal_force_n.abs() + solution.lateral_force_n.abs();
            let force_world = contact_force_world(solution, state, steer);
            let force_b = world_to_body(&state.orientation_world_from_body, &force_world);
            force_body += force_b;
            moment_body += contact.position_body_m.cross(&force_b);
        }
    }
    evaluation.force_body_n = force_body;
    evaluation.moment_body_nm = moment_body;
    evaluation.total_tangential_force_n = tangential;
    evaluation
}

fn evaluate_single_contact(
    state: &RigidBodyState,
    contact: &GearContact,
    surface: &GroundSurface,
    steer_angle_rad: f64,
    command: &GroundCommand,
) -> GroundContactSolution {
    let mut solution = GroundContactSolution::air();
    solution.steer_angle_rad = steer_angle_rad;
    // Wheel-bottom point: axle center plus radius along body-down (+Z body).
    let bottom_body = contact.position_body_m + Vec3::new(0.0, 0.0, contact.wheel_radius_m);
    let bottom_world =
        state.position_world_m + body_to_world(&state.orientation_world_from_body, &bottom_body);
    solution.contact_position_world_m = bottom_world;
    let (ground_height, _) = surface.height_and_normal(&bottom_world);
    let penetration = bottom_world.z - ground_height;
    if penetration <= 0.0 {
        return solution;
    }
    solution.penetration_m = penetration;
    let velocity_world = contact_point_velocity_world(state, &bottom_body);
    let velocity_body = world_to_body(&state.orientation_world_from_body, &velocity_world);
    solution.contact_velocity_body_mps = velocity_body;
    // Unilateral compliant normal law along world-down:
    // F_n = k * penetration + c * v_down_world, clamped to push-only.
    // v_down_world = +d(penetration)/dt, so sinking (v_down > 0) increases
    // the upward push (damping opposes penetration rate) while fast
    // separation drives F_n negative and clamps to zero (never pulls).
    let spring = contact.stiffness_n_per_m * penetration;
    let damper = contact.damping_n_s_per_m * velocity_world.z;
    let normal = (spring + damper).max(0.0);
    if normal <= 0.0 {
        return solution;
    }
    solution.in_contact = true;
    solution.normal_force_n = normal;
    // Wheel-frame slip: rotate body slip about body-Z by steer angle.
    let (sin_s, cos_s) = steer_angle_rad.sin_cos();
    let long_mps = cos_s * velocity_body.x + sin_s * velocity_body.y;
    let lat_mps = -sin_s * velocity_body.x + cos_s * velocity_body.y;
    let long_cap = contact.long_mu * normal;
    let lat_cap = contact.lat_mu * normal;
    let roll_cap = (contact.rolling_mu * normal).min(long_cap);
    let brake_cap = if contact.braked {
        contact.brake_mu * command.brake_command * normal
    } else {
        0.0
    };
    // Brake augments the longitudinal envelope only (never lateral grip).
    let mut long_force =
        -regularized_coulomb(long_mps, long_cap) - regularized_coulomb(long_mps, roll_cap);
    long_force += -regularized_coulomb(long_mps, brake_cap);
    let lat_force = -regularized_coulomb(lat_mps, lat_cap);
    // Clamp longitudinal total so resistance/brake can never propel.
    let combined = long_cap + roll_cap + brake_cap;
    long_force = long_force.clamp(-combined, combined);
    solution.longitudinal_force_n = long_force;
    solution.lateral_force_n = lat_force;
    solution
}

fn contact_force_world(
    solution: GroundContactSolution,
    state: &RigidBodyState,
    steer_angle_rad: f64,
) -> Vec3 {
    // Normal points up (world -Z). Wheel-frame tangential forces rotate back
    // by steer angle into body axes, then into world coordinates.
    let (sin_s, cos_s) = steer_angle_rad.sin_cos();
    let fx = cos_s * solution.longitudinal_force_n - sin_s * solution.lateral_force_n;
    let fy = sin_s * solution.longitudinal_force_n + cos_s * solution.lateral_force_n;
    let tangential_world =
        body_to_world(&state.orientation_world_from_body, &Vec3::new(fx, fy, 0.0));
    Vec3::new(0.0, 0.0, -solution.normal_force_n) + tangential_world
}
