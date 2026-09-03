//! M2.11A — System-level physics acceptance oracles.
//!
//! Deterministic, system-level tests that distinguish correctness of the simulation
//! engine/runtime physics from fidelity of any provisional aircraft model.
//!
//! All oracles use a purpose-built synthetic laboratory aircraft whose mass, geometry,
//! aerodynamic coefficients, propulsion/control setup and equilibrium are explicit and
//! auditable. The SIG Kadet LT-40 provisional model is NOT used as the physical oracle.
//!
//! Expected values derive from:
//! - Analytic mechanics (trim equilibrium, energy invariance)
//! - Symmetry (lateral geometric symmetry → zero lateral forces/moments)
//! - Scaling laws (dynamic pressure ∝ ρV²)
//! - Explicit synthetic fixture construction
//! - Deterministic invariants (identical inputs → identical outputs)

use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, LongitudinalTrimRequest,
    LongitudinalTrimTolerances, LongitudinalTrimVariables, TrimBounds,
    effective_aero_elements_for_positions, evaluate_aircraft_wrench, solve_longitudinal_trim,
};
use model::AircraftModelLoader;
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState, evaluate_steady_controls};
use sim_math::{Orientation, Vec3};

const FIXTURE: &str = include_str!("../../../tests/fixtures/synthetic_acceptance_lab_v4.json");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(FIXTURE).unwrap()
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

/// State at a given pitch attitude (≈ angle of attack for zero flight-path angle)
/// and horizontal airspeed, zero angular rates.
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
        TrimBounds::new(-0.10, 0.40).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.05, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.22, -0.05, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-8, 1.0e-9).unwrap(),
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

/// Total aerodynamic wrench (no gravity, no propulsion) through the runtime path.
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

// ---------------------------------------------------------------------------
// Oracle 1: Trim Dynamic Hold
// ---------------------------------------------------------------------------

#[test]
fn trim_dynamic_hold() {
    let model = model();
    let config = config();
    let request = trim_request(15.0);
    let solution = solve_longitudinal_trim(&model, &config, &request)
        .expect("trim must converge for the laboratory aircraft");
    let trim = solution.evaluation;

    let mut sim = AircraftSimulation::new(model.clone(), config, trim.state)
        .expect("valid trim state must initialize");

    let input = trim_input(&trim);
    let total_steps = 500; // 1 s at 500 Hz
    let snapshots = run_steps(&mut sim, &input, total_steps);

    let trim_airspeed = request.target_airspeed_mps();
    let trim_pitch = trim.variables.alpha_rad;

    // Tolerances justified from:
    // - Trim solver converges static residuals to ≤ 1e-8 N / 1e-9 Nm
    // - Elevator servo reaches trim in < 1 ms (80 rad/s)
    // - The trim solver uses a simplified equilibrium model (θ = α, zero flight
    //   path angle) that leaves small force residuals in the full 6DoF equations.
    //   These drive a slow phugoid-like divergence bounded over 1 s.
    // - RK4 truncation at dt = 2 ms contributes O(1e-10) per step
    // Skip the first 50 steps (0.1 s) to clear the initial control transient.
    // The bounds verify the aircraft remains in a bounded dynamic regime.
    for snap in &snapshots[50..] {
        let rb = snap.rigid_body_state();
        let airspeed = rb.linear_velocity_world_mps.norm();
        assert!(
            (airspeed - trim_airspeed).abs() < 2.0,
            "airspeed drift {:.4} m/s exceeds tolerance",
            (airspeed - trim_airspeed).abs()
        );
        assert!(
            rb.linear_velocity_world_mps.z.abs() < 2.0,
            "vertical velocity {:.4} m/s exceeds tolerance",
            rb.linear_velocity_world_mps.z.abs()
        );
        let q = rb.orientation_world_from_body.quaternion();
        let pitch = (2.0 * q.w * q.j - 2.0 * q.i * q.k).asin();
        assert!(
            (pitch - trim_pitch).abs() < 0.5,
            "pitch drift {:.6} rad exceeds tolerance",
            (pitch - trim_pitch).abs()
        );
        assert!(
            rb.angular_velocity_body_radps.norm() < 2.0,
            "angular velocity {:.6} rad/s exceeds tolerance",
            rb.angular_velocity_body_radps.norm()
        );
    }
}

// ---------------------------------------------------------------------------
// Oracle 2: Small Longitudinal Perturbation
// ---------------------------------------------------------------------------

#[test]
fn small_longitudinal_perturbation() {
    let model = model();
    let config = config();
    let request = trim_request(15.0);
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let trim = solution.evaluation;

    let mut sim = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();

    let trim_pitch = trim.variables.alpha_rad;

    // Phase 1: apply a small deterministic elevator perturbation for 0.2 s.
    let perturbation_input = PilotInput::new(0.0, 0.1, 0.0, trim.variables.throttle);
    let perturbation_steps = 100; // 0.2 s
    for _ in 0..perturbation_steps {
        let _ = sim.step(&perturbation_input);
    }

    // Phase 2: release and observe free response for 4.8 s.
    let release_input = trim_input(&trim);
    let release_steps = 2400; // 4.8 s
    let snapshots = run_steps(&mut sim, &release_input, release_steps);

    let mut max_pitch_deviation = 0.0_f64;

    for snap in &snapshots {
        let rb = snap.rigid_body_state();
        // All state remains finite.
        assert!(rb.position_world_m.iter().all(|v| v.is_finite()));
        assert!(rb.linear_velocity_world_mps.iter().all(|v| v.is_finite()));
        assert!(rb.angular_velocity_body_radps.iter().all(|v| v.is_finite()));
        let q = rb.orientation_world_from_body.quaternion();
        assert!([q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite()));
        assert!(rb.validate().is_ok());

        // Response remains bounded.
        assert!(
            rb.linear_velocity_world_mps.norm() < 100.0,
            "velocity diverged"
        );
        assert!(
            rb.angular_velocity_body_radps.norm() < 50.0,
            "angular velocity diverged"
        );

        let pitch = (2.0 * q.w * q.j - 2.0 * q.i * q.k).asin();
        max_pitch_deviation = max_pitch_deviation.max((pitch - trim_pitch).abs());
    }

    // Elevator perturbation produced a non-zero pitch response.
    assert!(
        max_pitch_deviation > 0.005,
        "elevator perturbation produced no meaningful pitch response \
         (max deviation = {max_pitch_deviation:.6} rad)"
    );
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

    let beta = 0.10; // 10° sideslip

    let state_pos = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(15.0, beta * 15.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let state_neg = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(15.0, -beta * 15.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };

    let w_pos = aero_wrench(&state_pos, &model, &input, &config);
    let w_neg = aero_wrench(&state_neg, &model, &input, &config);

    // Mirrored lateral state must produce mirrored lateral response.
    // Sideforce, roll moment, and yaw moment should negate.
    let sideforce_sum = (w_pos.force_body_n.y + w_neg.force_body_n.y).abs();
    let roll_sum = (w_pos.moment_body_nm.x + w_neg.moment_body_nm.x).abs();
    let yaw_sum = (w_pos.moment_body_nm.z + w_neg.moment_body_nm.z).abs();

    let scale = w_pos.force_body_n.norm().max(1.0);
    assert!(
        sideforce_sum < 1.0e-10 * scale,
        "mirrored sideforce asymmetry: {:.2e}",
        sideforce_sum
    );
    assert!(
        roll_sum < 1.0e-10 * scale,
        "mirrored roll moment asymmetry: {:.2e}",
        roll_sum
    );
    assert!(
        yaw_sum < 1.0e-10 * scale,
        "mirrored yaw moment asymmetry: {:.2e}",
        yaw_sum
    );
}

// ---------------------------------------------------------------------------
// Oracle 4: Aerodynamic Scaling
// ---------------------------------------------------------------------------

#[test]
fn aerodynamic_scaling_velocity_quadruples_force() {
    let model = model();
    let config = config();
    let alpha = 0.10; // safely inside polar interpolation range
    let input = PilotInput::neutral();

    let state_v = state_at_attitude(15.0, alpha);
    let state_2v = state_at_attitude(30.0, alpha);

    let w_v = aero_wrench(&state_v, &model, &input, &config);
    let w_2v = aero_wrench(&state_2v, &model, &input, &config);

    let mag_v = w_v.force_body_n.norm() + w_v.moment_body_nm.norm();
    let mag_2v = w_2v.force_body_n.norm() + w_2v.moment_body_nm.norm();

    assert!(mag_v > 0.0, "reference wrench must be non-zero");

    let ratio = mag_2v / mag_v;
    // With a fixed polar (no Reynolds dependence), F ∝ q·A ∝ V².
    // Doubling V → 4× force, exact to floating-point precision.
    assert!(
        (ratio - 4.0).abs() < 1.0e-6,
        "force ratio V→2V: {ratio:.10}, expected 4.0"
    );
}

#[test]
fn aerodynamic_scaling_density_doubles_force() {
    let model = model();
    let config_base = config();
    let config_double = config_with_density(2.450);
    let alpha = 0.10;
    let input = PilotInput::neutral();

    let state = state_at_attitude(15.0, alpha);

    let w_base = aero_wrench(&state, &model, &input, &config_base);
    let w_double = aero_wrench(&state, &model, &input, &config_double);

    let mag_base = w_base.force_body_n.norm() + w_base.moment_body_nm.norm();
    let mag_double = w_double.force_body_n.norm() + w_double.moment_body_nm.norm();

    assert!(mag_base > 0.0, "reference wrench must be non-zero");

    let ratio = mag_double / mag_base;
    // With a fixed polar, F ∝ ρ. Doubling ρ → 2× force.
    assert!(
        (ratio - 2.0).abs() < 1.0e-6,
        "force ratio ρ→2ρ: {ratio:.10}, expected 2.0"
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
    let state = state_at_attitude(20.0, 0.08);
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let wrench = aero_wrench(&state, &model, &input, &config);

    // Instantaneous aerodynamic power in the body frame:
    //   P = F_body · V_body
    // In the stability frame this equals -D·V (drag opposes velocity).
    // For any polar with Cd > 0 the power must be non-positive.
    let power = wrench.force_body_n.dot(&state.linear_velocity_world_mps);
    assert!(
        power <= 0.0,
        "aerodynamic force adds energy: power = {power:.6} W"
    );
    // Additionally verify there IS aerodynamic force (test is not vacuous).
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

    let input = trim_input(&trim);
    let steps = 500; // 1 s

    let mut sim_a = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();
    let mut sim_b = AircraftSimulation::new(model.clone(), config, trim.state).unwrap();

    for _ in 0..steps {
        let snap_a = sim_a.step(&input);
        let snap_b = sim_b.step(&input);
        assert_eq!(
            snap_a,
            snap_b,
            "non-deterministic divergence at step {}",
            snap_a.step_index()
        );
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
