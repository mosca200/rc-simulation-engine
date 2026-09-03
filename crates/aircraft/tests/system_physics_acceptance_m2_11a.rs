//! M2.11A — System-level physics acceptance oracles.
//!
//! Deterministic, system-level tests that distinguish correctness of the simulation
//! engine/runtime physics from fidelity of any provisional aircraft model.
//!
//! The oracles use purpose-built synthetic laboratory aircraft whose mass, geometry,
//! aerodynamic coefficients, propulsion/control setup and equilibrium are explicit and
//! auditable. The SIG Kadet LT-40 provisional model is NOT used as a physical oracle.
//!
//! Expected values derive from:
//! - Analytic mechanics (trim equilibrium, energy invariance)
//! - Symmetry (zero-sideslip cancellation and non-zero mirrored lateral response)
//! - Scaling laws (dynamic pressure ∝ ρV²)
//! - Explicit synthetic fixture construction
//! - Deterministic invariants (identical inputs → identical outputs)
//!
//! ## Exact trim-hold fixture design
//!
//! At `rho = 1.225 kg/m^3` and `V = 15 m/s`, `q = 137.8125 Pa`. The `1.5 kg`
//! aircraft needs `L = mg = 14.709975 N`, so its `0.5 m^2` wing uses the authored
//! exact-trim sample `CL = L/(qS) = 0.21347809523809524` at `alpha = 0.1 rad`.
//! The wing is at the CG, with zero drag and intrinsic `Cm`, so it has no pitch
//! moment and its lift exactly balances gravity in level flight.
//!
//! The horizontal tail is at `x = -0.75 m`, which is aft of the CG in body FRD
//! (`+x` forward, `+y` right, `+z` down). Its incidence is `-0.1 rad`, making its
//! local alpha, lift, and moment zero at trim with a neutral elevator. Above trim
//! alpha it produces upward force aft of the CG and therefore negative body-Y
//! pitch moment: `dMy/dAlpha < 0`, the restoring sign for this convention.
//! Propulsion is absent and all drag coefficients are zero, making the neutral
//! control state an exact unpowered equilibrium rather than an actuator transient.

use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, LongitudinalTrimRequest,
    LongitudinalTrimTolerances, LongitudinalTrimVariables, TrimBounds,
    effective_aero_elements_for_positions, evaluate_aircraft_wrench,
    evaluate_longitudinal_trim_candidate, solve_longitudinal_trim,
};
use model::AircraftModelLoader;
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState, evaluate_steady_controls};
use sim_math::{Orientation, Vec3};

const GENERAL_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_acceptance_lab_v4.json");
const TRIM_HOLD_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_trim_hold_lab_v4.json");
const TRIM_AIRSPEED_MPS: f64 = 15.0;
const TRIM_ALPHA_RAD: f64 = 0.1;
const TRIM_WING_AREA_M2: f64 = 0.5;
const TRIM_WING_CL: f64 = 0.213_478_095_238_095_24;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(GENERAL_FIXTURE).unwrap()
}

fn trim_hold_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(TRIM_HOLD_FIXTURE).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::from_physics_hz(
        500,
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn config_with_density(density: f64) -> AircraftSimulationConfig {
    AircraftSimulationConfig::from_physics_hz(
        500,
        AeroEnvironment::new(density, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

/// State at a given pitch attitude and horizontal airspeed, zero angular rates.
fn state_at_attitude(airspeed_mps: f64, pitch_rad: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(airspeed_mps, 0.0, 0.0),
        orientation_world_from_body: Orientation::from_axis_angle(&Vec3::y_axis(), pitch_rad),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn trim_request(airspeed_mps: f64) -> LongitudinalTrimRequest {
    LongitudinalTrimRequest::new(
        airspeed_mps,
        TrimBounds::new(-0.10, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.05, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-8, 1.0e-9).unwrap(),
        50,
    )
    .unwrap()
}

fn exact_trim_request() -> LongitudinalTrimRequest {
    LongitudinalTrimRequest::new(
        TRIM_AIRSPEED_MPS,
        TrimBounds::new(0.05, 0.15).unwrap(),
        TrimBounds::new(-0.2, 0.2).unwrap(),
        TrimBounds::new(0.0, 1.0).unwrap(),
        LongitudinalTrimVariables::new(TRIM_ALPHA_RAD, 0.0, 0.0).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-10, 1.0e-10).unwrap(),
        50,
    )
    .unwrap()
}

fn trim_input(trim: &aircraft::LongitudinalTrimEvaluation) -> PilotInput {
    PilotInput::new(
        0.0,
        trim.variables.elevator_command,
        0.0,
        trim.variables.throttle,
    )
}

fn run_steps(
    sim: &mut AircraftSimulation,
    input: &PilotInput,
    n: u64,
) -> Vec<aircraft::AircraftSnapshot> {
    (0..n).map(|_| sim.step(input)).collect()
}

/// Total aerodynamic wrench through the runtime path.
fn aero_wrench(
    state: &RigidBodyState,
    model: &model::AircraftModel,
    input: &PilotInput,
    config: &AircraftSimulationConfig,
) -> sim_core::BodyWrench {
    let positions = evaluate_steady_controls(model.controls(), input);
    let elements = effective_aero_elements_for_positions(model, &positions);
    evaluate_aircraft_wrench(
        state,
        &elements,
        model,
        positions.throttle(),
        config.aero_environment(),
    )
}

/// Extract pitch attitude from quaternion using `R[2][0] = 2(ik - wj) = -sin(theta)`.
fn pitch_from_quaternion(q: &sim_math::Quaternion<f64>) -> f64 {
    (2.0 * (q.w * q.j - q.k * q.i)).asin()
}

/// Transform world-frame velocity to body frame.
fn velocity_body(state: &RigidBodyState) -> Vec3 {
    state
        .orientation_world_from_body
        .inverse_transform_vector(&state.linear_velocity_world_mps)
}

// ---------------------------------------------------------------------------
// Oracle 1: Trim Dynamic Hold
// ---------------------------------------------------------------------------

#[test]
fn trim_dynamic_hold() {
    const HOLD_STEPS: u64 = 5_000;
    const AIRSPEED_TOLERANCE_MPS: f64 = 1.0e-8;
    const VERTICAL_VELOCITY_TOLERANCE_MPS: f64 = 1.0e-8;
    const PITCH_TOLERANCE_RAD: f64 = 2.0e-8;
    const ANGULAR_RATE_TOLERANCE_RADPS: f64 = 5.0e-9;

    let model = trim_hold_model();
    let config = config();
    let request = exact_trim_request();

    // Analytic equilibrium authored in the fixture, not inferred from runtime output:
    // q = 1/2 rho V^2 = 137.8125 Pa
    // L = mg = 14.709975 N
    // CL = L/(qS) = 0.21347809523809524 at alpha = 0.1 rad.
    let dynamic_pressure_pa = 0.5 * 1.225 * TRIM_AIRSPEED_MPS.powi(2);
    let required_lift_n = 1.5 * sim_core::DEFAULT_GRAVITY_MPS2;
    let required_cl = required_lift_n / (dynamic_pressure_pa * TRIM_WING_AREA_M2);
    assert_eq!(dynamic_pressure_pa, 137.8125);
    assert_eq!(required_lift_n, 14.709_975);
    assert_eq!(required_cl, TRIM_WING_CL);

    let solution = solve_longitudinal_trim(&model, &config, &request)
        .expect("analytic laboratory equilibrium must solve");
    let trim = solution.evaluation;
    assert_eq!(
        solution.iteration_count, 0,
        "analytic initial guess must already be trim"
    );
    assert!((trim.variables.alpha_rad - TRIM_ALPHA_RAD).abs() <= 1.0e-14);
    assert!(
        trim.variables.elevator_command.abs() <= 1.0e-14,
        "exact equilibrium must use neutral elevator, got {}",
        trim.variables.elevator_command
    );
    assert_eq!(
        trim.variables.throttle, 0.0,
        "unpowered equilibrium must retain zero throttle"
    );

    // Frozen-control restoring derivative. In body FRD, positive My is nose-up
    // by the right-hand rule. The tail is aft (x = -0.75 m), so positive
    // delta-alpha creates negative My: a nose-down restoring moment.
    // With tail lift slope 4/rad, mechanics gives:
    // dMy/dAlpha = -q S_tail CL_alpha l_tail cos(alpha_trim) cos(delta).
    let delta = 0.01;
    let minus = evaluate_longitudinal_trim_candidate(
        &model,
        &config,
        &request,
        LongitudinalTrimVariables::new(
            trim.variables.alpha_rad - delta,
            trim.variables.elevator_command,
            trim.variables.throttle,
        )
        .unwrap(),
    )
    .unwrap();
    let plus = evaluate_longitudinal_trim_candidate(
        &model,
        &config,
        &request,
        LongitudinalTrimVariables::new(
            trim.variables.alpha_rad + delta,
            trim.variables.elevator_command,
            trim.variables.throttle,
        )
        .unwrap(),
    )
    .unwrap();
    let pitch_stiffness_nm_per_rad =
        (plus.residuals.pitch_moment_nm - minus.residuals.pitch_moment_nm) / (2.0 * delta);
    let expected_pitch_stiffness =
        -dynamic_pressure_pa * 0.10 * 4.0 * 0.75 * TRIM_ALPHA_RAD.cos() * delta.cos();
    assert!(
        (pitch_stiffness_nm_per_rad - expected_pitch_stiffness).abs() <= 1.0e-10,
        "pitch stiffness {pitch_stiffness_nm_per_rad:.12} differs from mechanics {expected_pitch_stiffness:.12}"
    );
    assert!(
        pitch_stiffness_nm_per_rad < -1.0,
        "dMy/dAlpha must be materially negative/restoring, got {pitch_stiffness_nm_per_rad}"
    );

    let mut sim = AircraftSimulation::new(model.clone(), config, trim.state)
        .expect("valid trim state must initialize");
    let input = trim_input(&trim);
    assert!(
        (pitch_from_quaternion(trim.state.orientation_world_from_body.quaternion())
            - TRIM_ALPHA_RAD)
            .abs()
            <= 1.0e-14
    );
    let mut max_airspeed_deviation_mps = 0.0_f64;
    let mut max_vertical_velocity_mps = 0.0_f64;
    let mut max_pitch_deviation_rad = 0.0_f64;
    let mut max_angular_rate_radps = 0.0_f64;

    // Inspect every committed step from step 1 through 5000 (10 seconds); there
    // is no skipped actuator-settling interval. The 1e-10 N/Nm trim residuals
    // bound ten-second free accumulation by 6.7e-10 m/s translationally,
    // 2.9e-9 rad/s rotationally, and 1.5e-8 rad in attitude. The asserted
    // limits add only a small allowance for RK4 and floating-point accumulation.
    for step in 1..=HOLD_STEPS {
        let snap = sim.step(&input);
        let rb = snap.rigid_body_state();
        let airspeed = rb.linear_velocity_world_mps.norm();
        max_airspeed_deviation_mps =
            max_airspeed_deviation_mps.max((airspeed - TRIM_AIRSPEED_MPS).abs());
        max_vertical_velocity_mps =
            max_vertical_velocity_mps.max(rb.linear_velocity_world_mps.z.abs());
        let q = rb.orientation_world_from_body.quaternion();
        let pitch = pitch_from_quaternion(q);
        max_pitch_deviation_rad = max_pitch_deviation_rad.max((pitch - TRIM_ALPHA_RAD).abs());
        max_angular_rate_radps = max_angular_rate_radps.max(rb.angular_velocity_body_radps.norm());
        assert!(rb.validate().is_ok(), "invalid state at hold step {step}");
    }

    println!(
        "trim alpha={:.17} elevator={:.17} throttle={:.17} dMy/dAlpha={:.12}",
        trim.variables.alpha_rad,
        trim.variables.elevator_command,
        trim.variables.throttle,
        pitch_stiffness_nm_per_rad
    );
    println!(
        "10 s hold maxima: airspeed={max_airspeed_deviation_mps:.12e} m/s, vertical_velocity={max_vertical_velocity_mps:.12e} m/s, pitch={max_pitch_deviation_rad:.12e} rad, angular_rate={max_angular_rate_radps:.12e} rad/s"
    );
    assert!(max_airspeed_deviation_mps <= AIRSPEED_TOLERANCE_MPS);
    assert!(max_vertical_velocity_mps <= VERTICAL_VELOCITY_TOLERANCE_MPS);
    assert!(max_pitch_deviation_rad <= PITCH_TOLERANCE_RAD);
    assert!(max_angular_rate_radps <= ANGULAR_RATE_TOLERANCE_RADPS);
}

// ---------------------------------------------------------------------------
// Oracle 2: Small Longitudinal Perturbation
// ---------------------------------------------------------------------------

#[test]
fn small_longitudinal_perturbation() {
    const SPEED_DEVIATION_LIMIT_MPS: f64 = 0.25;
    const VERTICAL_VELOCITY_LIMIT_MPS: f64 = 0.50;
    const PITCH_DEVIATION_LIMIT_RAD: f64 = 0.05;
    const ANGULAR_RATE_LIMIT_RADPS: f64 = 0.25;

    let model = trim_hold_model();
    let config = config();
    let request = exact_trim_request();
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let trim = solution.evaluation;

    let mut sim = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();
    let delta_elevator = 0.01;
    let pert_cmd = trim.variables.elevator_command + delta_elevator;
    assert!(pert_cmd.abs() <= 1.0);

    // Apply a 1%-command elevator pulse for 0.1 s, then release to the exact
    // neutral-control equilibrium for 4.9 s. The envelope permits 1.7% speed
    // change, 2.9 degrees attitude change, and 0.25 rad/s body rate: broad
    // relative to this deliberately small impulse, but far below divergence.
    let perturbation_input = PilotInput::new(0.0, pert_cmd, 0.0, trim.variables.throttle);
    let release_input = trim_input(&trim);
    let mut snapshots = run_steps(&mut sim, &perturbation_input, 50);
    snapshots.extend(run_steps(&mut sim, &release_input, 2_450));

    let mut max_speed_deviation_mps = 0.0_f64;
    let mut max_vertical_velocity_mps = 0.0_f64;
    let mut max_pitch_deviation_rad = 0.0_f64;
    let mut max_angular_rate_radps = 0.0_f64;
    for snap in &snapshots {
        let rb = snap.rigid_body_state();
        assert!(rb.validate().is_ok());
        let airspeed = rb.linear_velocity_world_mps.norm();
        max_speed_deviation_mps = max_speed_deviation_mps.max((airspeed - TRIM_AIRSPEED_MPS).abs());
        max_vertical_velocity_mps =
            max_vertical_velocity_mps.max(rb.linear_velocity_world_mps.z.abs());
        let q = rb.orientation_world_from_body.quaternion();
        let pitch = pitch_from_quaternion(q);
        max_pitch_deviation_rad = max_pitch_deviation_rad.max((pitch - TRIM_ALPHA_RAD).abs());
        max_angular_rate_radps = max_angular_rate_radps.max(rb.angular_velocity_body_radps.norm());
    }

    println!(
        "perturbation maxima: airspeed={max_speed_deviation_mps:.12e} m/s, vertical_velocity={max_vertical_velocity_mps:.12e} m/s, pitch={max_pitch_deviation_rad:.12e} rad, angular_rate={max_angular_rate_radps:.12e} rad/s"
    );
    assert!(
        max_pitch_deviation_rad > 1.0e-4,
        "elevator perturbation produced no meaningful pitch response \
         (max deviation = {max_pitch_deviation_rad:.6} rad)"
    );
    assert!(max_speed_deviation_mps <= SPEED_DEVIATION_LIMIT_MPS);
    assert!(max_vertical_velocity_mps <= VERTICAL_VELOCITY_LIMIT_MPS);
    assert!(max_pitch_deviation_rad <= PITCH_DEVIATION_LIMIT_RAD);
    assert!(max_angular_rate_radps <= ANGULAR_RATE_LIMIT_RADPS);
}

// ---------------------------------------------------------------------------
// Oracle 3: Symmetry
// ---------------------------------------------------------------------------

#[test]
fn symmetry_no_spontaneous_lateral_forces() {
    let model = model();
    let config = config();

    // Symmetric flight: zero α, zero sideslip, zero lateral input.
    let state = state_at_attitude(15.0, 0.0);
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let wrench = aero_wrench(&state, &model, &input, &config);

    // No spontaneous lateral force or roll/yaw moment.
    assert!(
        wrench.force_body_n.y.abs() < 1.0e-10,
        "spontaneous sideforce: {:.2e} N",
        wrench.force_body_n.y
    );
    assert!(
        wrench.moment_body_nm.x.abs() < 1.0e-10,
        "spontaneous roll moment: {:.2e} Nm",
        wrench.moment_body_nm.x
    );
    assert!(
        wrench.moment_body_nm.z.abs() < 1.0e-10,
        "spontaneous yaw moment: {:.2e} Nm",
        wrench.moment_body_nm.z
    );
}

#[test]
fn symmetry_mirrored_lateral_response() {
    let model = model();
    let config = config();
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);

    // Sideslip ratio vy/V = 0.10 → β = atan(0.10) ≈ 0.0997 rad ≈ 5.7°.
    let sideslip_ratio = 0.10;

    let state_pos = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(15.0, sideslip_ratio * 15.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let state_neg = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(15.0, -sideslip_ratio * 15.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };

    let w_pos = aero_wrench(&state_pos, &model, &input, &config);
    let w_neg = aero_wrench(&state_neg, &model, &input, &config);

    // The dedicated vertical tail is symmetric, vertical, and aft of the CG.
    // Positive body-Y velocity becomes positive local vertical-tail alpha,
    // producing negative body-Y force and positive body-Z yaw moment. Mirroring
    // sideslip reverses both. Assert material magnitudes before comparing signs
    // so zero/zero can never satisfy this oracle.
    assert!(
        w_pos.force_body_n.y.abs() > 1.0,
        "vertical-tail sideforce is not material: {} N",
        w_pos.force_body_n.y
    );
    assert!(
        w_pos.moment_body_nm.z.abs() > 0.5,
        "vertical-tail yaw moment is not material: {} Nm",
        w_pos.moment_body_nm.z
    );
    assert!(w_pos.force_body_n.y < -1.0 && w_neg.force_body_n.y > 1.0);
    assert!(w_pos.moment_body_nm.z > 0.5 && w_neg.moment_body_nm.z < -0.5);

    let sideforce_sum = (w_pos.force_body_n.y + w_neg.force_body_n.y).abs();
    let roll_sum = (w_pos.moment_body_nm.x + w_neg.moment_body_nm.x).abs();
    let yaw_sum = (w_pos.moment_body_nm.z + w_neg.moment_body_nm.z).abs();
    let longitudinal_force_difference = (w_pos.force_body_n.x - w_neg.force_body_n.x).abs();
    let vertical_force_difference = (w_pos.force_body_n.z - w_neg.force_body_n.z).abs();

    println!(
        "lateral mirror: +beta Fy={:.12} N Mz={:.12} Nm; -beta Fy={:.12} N Mz={:.12} Nm",
        w_pos.force_body_n.y, w_pos.moment_body_nm.z, w_neg.force_body_n.y, w_neg.moment_body_nm.z
    );
    assert!(
        sideforce_sum < 1.0e-10,
        "mirrored sideforce asymmetry: {:.2e}",
        sideforce_sum
    );
    assert!(
        roll_sum < 1.0e-10,
        "mirrored roll moment asymmetry: {:.2e}",
        roll_sum
    );
    assert!(
        yaw_sum < 1.0e-10,
        "mirrored yaw moment asymmetry: {:.2e}",
        yaw_sum
    );
    assert!(longitudinal_force_difference < 1.0e-10);
    assert!(vertical_force_difference < 1.0e-10);
}

// ---------------------------------------------------------------------------
// Oracle 4: Aerodynamic Scaling
// ---------------------------------------------------------------------------

#[test]
fn aerodynamic_scaling_velocity_quadruples_force_and_moment() {
    let model = model();
    let config = config();
    let alpha = 0.10; // safely inside polar interpolation range
    let input = PilotInput::neutral();

    let state_v = state_at_attitude(15.0, alpha);
    let state_2v = state_at_attitude(30.0, alpha);

    let w_v = aero_wrench(&state_v, &model, &input, &config);
    let w_2v = aero_wrench(&state_2v, &model, &input, &config);

    // With a fixed polar (no Reynolds dependence), F ∝ q·S ∝ V² and M ∝ V².
    // Test force and moment independently to avoid dimensionally invalid sums.

    let force_ratio = w_2v.force_body_n.norm() / w_v.force_body_n.norm();
    assert!(
        w_v.force_body_n.norm() > 0.1,
        "reference force too small for scaling check"
    );
    assert!(
        (force_ratio - 4.0).abs() < 1.0e-6,
        "force ratio V→2V: {force_ratio:.10}, expected 4.0"
    );

    let moment_ratio = w_2v.moment_body_nm.norm() / w_v.moment_body_nm.norm();
    assert!(
        w_v.moment_body_nm.norm() > 0.001,
        "reference moment too small for scaling check"
    );
    assert!(
        (moment_ratio - 4.0).abs() < 1.0e-6,
        "moment ratio V→2V: {moment_ratio:.10}, expected 4.0"
    );
}

#[test]
fn aerodynamic_scaling_density_doubles_force_and_moment() {
    let model = model();
    let config_base = config();
    let config_double = config_with_density(2.450);
    let alpha = 0.10;
    let input = PilotInput::neutral();

    let state = state_at_attitude(15.0, alpha);

    let w_base = aero_wrench(&state, &model, &input, &config_base);
    let w_double = aero_wrench(&state, &model, &input, &config_double);

    // With a fixed polar, F ∝ ρ and M ∝ ρ.
    let force_ratio = w_double.force_body_n.norm() / w_base.force_body_n.norm();
    assert!(
        w_base.force_body_n.norm() > 0.1,
        "reference force too small for scaling check"
    );
    assert!(
        (force_ratio - 2.0).abs() < 1.0e-6,
        "force ratio ρ→2ρ: {force_ratio:.10}, expected 2.0"
    );

    let moment_ratio = w_double.moment_body_nm.norm() / w_base.moment_body_nm.norm();
    assert!(
        w_base.moment_body_nm.norm() > 0.001,
        "reference moment too small for scaling check"
    );
    assert!(
        (moment_ratio - 2.0).abs() < 1.0e-6,
        "moment ratio ρ→2ρ: {moment_ratio:.10}, expected 2.0"
    );
}

// ---------------------------------------------------------------------------
// Oracle 5: Energy / Drag Invariant
// ---------------------------------------------------------------------------

#[test]
fn energy_drag_invariant() {
    let model = model();
    let config = config();

    // Unpowered flight at a positive angle of attack.
    // Angular velocity is zero: this test checks translational aerodynamic power only.
    let state = state_at_attitude(20.0, 0.10);
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let wrench = aero_wrench(&state, &model, &input, &config);

    // Instantaneous aerodynamic power in a single consistent frame (body):
    //   P = F_body · V_body
    // In the stability frame this equals −D·V (drag opposes velocity).
    // For any polar with Cd > 0 the power must be non-positive.
    let v_body = velocity_body(&state);
    let power = wrench.force_body_n.dot(&v_body);
    assert!(
        power <= 0.0,
        "aerodynamic force adds energy: power = {power:.6} W"
    );
    // Verify the test is not vacuous.
    assert!(
        wrench.force_body_n.norm() > 0.1,
        "aerodynamic force too small for a meaningful check"
    );
}

// ---------------------------------------------------------------------------
// Oracle 6: Determinism
// ---------------------------------------------------------------------------

#[test]
fn determinism_exact_replay() {
    let model = model();
    let config = config();
    let request = trim_request(15.0);
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let trim = solution.evaluation;

    // Non-trivial deterministic input schedule with integer step boundaries:
    //   Phase 0 [0, 500):      trim (1.0 s)
    //   Phase 1 [500, 600):    pitch-up perturbation (0.2 s)
    //   Phase 2 [600, 1000):   release to trim (0.8 s)
    //   Phase 3 [1000, 1100):  throttle bump (0.2 s)
    //   Phase 4 [1100, 1500):  release to trim (0.8 s)
    let trim_input_val = trim_input(&trim);
    let pitch_up = PilotInput::new(
        0.0,
        (trim.variables.elevator_command + 0.05).clamp(-1.0, 1.0),
        0.0,
        trim.variables.throttle,
    );
    let throttle_bump = PilotInput::new(
        0.0,
        trim.variables.elevator_command,
        0.0,
        (trim.variables.throttle + 0.1).clamp(0.0, 1.0),
    );

    let schedule: Vec<(&PilotInput, u64)> = vec![
        (&trim_input_val, 500),
        (&pitch_up, 100),
        (&trim_input_val, 400),
        (&throttle_bump, 100),
        (&trim_input_val, 400),
    ];

    let mut sim_a = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();
    let mut sim_b = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();

    for (input, steps) in &schedule {
        for _ in 0..*steps {
            let snap_a = sim_a.step(input);
            let snap_b = sim_b.step(input);
            assert_eq!(
                snap_a,
                snap_b,
                "non-deterministic divergence at step {}",
                snap_a.step_index()
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Oracle 7: Long Finite Run
// ---------------------------------------------------------------------------

#[test]
fn long_finite_run() {
    let model = model();
    let config = config();
    let request = trim_request(15.0);
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let trim = solution.evaluation;

    let mut sim = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();
    let input = trim_input(&trim);
    let total_steps = 5000; // 10 s at 500 Hz

    for step in 0..total_steps {
        let snap = sim.step(&input);
        let rb = snap.rigid_body_state();

        assert!(
            rb.position_world_m.iter().all(|v| v.is_finite()),
            "step {step}: non-finite position"
        );
        assert!(
            rb.linear_velocity_world_mps.iter().all(|v| v.is_finite()),
            "step {step}: non-finite velocity"
        );
        assert!(
            rb.angular_velocity_body_radps.iter().all(|v| v.is_finite()),
            "step {step}: non-finite angular velocity"
        );
        let q = rb.orientation_world_from_body.quaternion();
        assert!(
            [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite()),
            "step {step}: non-finite quaternion"
        );
        assert!(
            rb.validate().is_ok(),
            "step {step}: invalid state (quaternion norm drift or non-finite)"
        );
    }
}
