//! M2.12A — Full runtime soak, determinism, allocation, and performance diagnostics.
//!
//! Validates runtime robustness over prolonged execution:
//! - 60-second deterministic soak (30 000 steps at 500 Hz)
//! - Exact step-by-step determinism between two identical runs
//! - Zero-allocation hot path after warm-up
//! - State/hash consistency at checkpoints
//!
//! This is NOT a physics-fidelity slice. No assertions about realistic flight.

use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::AircraftModelLoader;
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState, SimSnapshot};
use sim_math::{Orientation, Vec3};

const FIXTURE: &str = include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");

const PHYSICS_HZ: u32 = 500;
const SOAK_DURATION_S: u64 = 60;
const TOTAL_STEPS: u64 = PHYSICS_HZ as u64 * SOAK_DURATION_S; // 30 000

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(FIXTURE).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::from_physics_hz(
        PHYSICS_HZ,
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(20.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

/// Deterministic non-trivial input schedule with integer step boundaries.
/// Each entry: (PilotInput, step_count).
/// Total = 30 000 steps = 60 s at 500 Hz.
///
/// The test fixture (`synthetic_non_reference_trim_v4`) binds only the
/// elevator aerodynamic control.  Therefore:
/// - Elevator and throttle phases exercise full aircraft physics
///   (aerodynamics + propulsion + rigid-body dynamics).
/// - Aileron and rudder phases exercise control-system state only
///   (actuator deflections change, but aerodynamic geometry is unaffected).
fn input_schedule() -> Vec<(PilotInput, u64)> {
    vec![
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 5000), // 10 s: neutral, half throttle
        (PilotInput::new(0.0, 0.1, 0.0, 0.50), 1000), //  2 s: small pitch-up
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000), //  6 s: release
        (PilotInput::new(0.1, 0.0, 0.0, 0.50), 1000), //  2 s: small aileron
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000), //  6 s: release
        (PilotInput::new(0.0, 0.0, 0.05, 0.50), 1000), //  2 s: small rudder
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000), //  6 s: release
        (PilotInput::new(0.0, -0.08, 0.0, 0.60), 2000), //  4 s: pitch-down + throttle up
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000), //  6 s: release
        (PilotInput::new(0.05, 0.05, -0.03, 0.55), 3000), //  6 s: combined small inputs
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 5000), // 10 s: neutral
    ]
}

fn assert_schedule_total() {
    let total: u64 = input_schedule().iter().map(|(_, n)| n).sum();
    assert_eq!(
        total, TOTAL_STEPS,
        "input schedule must sum to {TOTAL_STEPS} steps"
    );
}

fn validate_state(snap: &aircraft::AircraftSnapshot, step: u64, dt_s: f64) {
    let rb = snap.rigid_body_state();
    let q = rb.orientation_world_from_body.quaternion();

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
    assert!(
        [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite()),
        "step {step}: non-finite quaternion"
    );
    assert!(
        rb.validate().is_ok(),
        "step {step}: invalid rigid-body state"
    );

    let csp = snap.control_surface_positions();
    assert!(
        [
            csp.aileron_angle_rad(),
            csp.elevator_angle_rad(),
            csp.rudder_angle_rad(),
            csp.throttle(),
        ]
        .iter()
        .all(|v| v.is_finite()),
        "step {step}: non-finite control surface positions"
    );

    assert_eq!(snap.step_index(), step + 1);
    let expected_time = (step + 1) as f64 * dt_s;
    assert!(
        (snap.sim_time_s() - expected_time).abs() < 1.0e-9,
        "step {step}: sim_time mismatch"
    );
}

fn to_sim_snapshot(snap: &aircraft::AircraftSnapshot, dt_s: f64) -> SimSnapshot {
    SimSnapshot::from_state(snap.step_index(), dt_s, snap.rigid_body_state())
}

// ---------------------------------------------------------------------------
// Oracle 1: Long Deterministic Soak
// ---------------------------------------------------------------------------

#[test]
fn long_deterministic_soak_state_validity() {
    assert_schedule_total();

    let model = model();
    let config = config();
    let mut sim = AircraftSimulation::new(model, config, initial_state()).unwrap();

    let schedule = input_schedule();
    let dt_s = config.dt_s();
    let mut global_step = 0_u64;

    for (input, steps) in &schedule {
        for _ in 0..*steps {
            let snap = sim.step(input);
            validate_state(&snap, global_step, dt_s);
            global_step += 1;
        }
    }

    assert_eq!(global_step, TOTAL_STEPS);
}

// ---------------------------------------------------------------------------
// Oracle 2: Exact Long-Run Determinism
// ---------------------------------------------------------------------------

#[test]
fn exact_long_run_determinism() {
    assert_schedule_total();

    let schedule = input_schedule();

    let mut sim_a = AircraftSimulation::new(model(), config(), initial_state()).unwrap();
    let mut sim_b = AircraftSimulation::new(model(), config(), initial_state()).unwrap();

    let mut global_step = 0_u64;

    for (input, steps) in &schedule {
        for _ in 0..*steps {
            let snap_a = sim_a.step(input);
            let snap_b = sim_b.step(input);
            assert_eq!(
                snap_a, snap_b,
                "non-deterministic divergence at step {global_step}"
            );
            global_step += 1;
        }
    }

    assert_eq!(global_step, TOTAL_STEPS);
}

// ---------------------------------------------------------------------------
// Oracle 3: Full Aircraft Hot-Path Allocations
// ---------------------------------------------------------------------------

#[test]
fn full_aircraft_step_allocates_nothing_after_warmup() {
    let model = model();
    let config = config();
    let mut sim = AircraftSimulation::new(model, config, initial_state()).unwrap();

    // Warm-up: run the full input schedule once so that any lazy
    // initialisation triggered by varying aerodynamic conditions completes
    // before the measurement region.
    let schedule = input_schedule();
    for (input, steps) in &schedule {
        for _ in 0..*steps {
            std::hint::black_box(sim.step(std::hint::black_box(input)));
        }
    }

    // Measure allocations over a second pass of the identical schedule.
    let allocations = allocation_counter::measure(|| {
        for (input, steps) in &schedule {
            for _ in 0..*steps {
                std::hint::black_box(sim.step(std::hint::black_box(input)));
            }
        }
    });

    assert_eq!(
        allocations.count_total, 0,
        "AircraftSimulation::step() performed {} heap allocation(s) after warm-up: {allocations:?}",
        allocations.count_total
    );
}

// ---------------------------------------------------------------------------
// Oracle 4: State / Hash Consistency
// ---------------------------------------------------------------------------

#[test]
fn state_hash_consistency_across_identical_runs() {
    assert_schedule_total();

    let schedule = input_schedule();
    let dt = config().dt_s();

    let mut sim_a = AircraftSimulation::new(model(), config(), initial_state()).unwrap();
    let mut sim_b = AircraftSimulation::new(model(), config(), initial_state()).unwrap();

    // Check hashes at every 1000-step checkpoint.
    let checkpoint_interval = 1000_u64;
    let mut global_step = 0_u64;

    for (input, steps) in &schedule {
        for _ in 0..*steps {
            let snap_a = sim_a.step(input);
            let snap_b = sim_b.step(input);
            global_step += 1;

            if global_step.is_multiple_of(checkpoint_interval) {
                let hash_a = to_sim_snapshot(&snap_a, dt).state_hash();
                let hash_b = to_sim_snapshot(&snap_b, dt).state_hash();
                assert_eq!(hash_a, hash_b, "state hash mismatch at step {global_step}");
            }
        }
    }

    // Final hash must also match.
    let final_a = SimSnapshot::from_state(sim_a.step_index(), dt, sim_a.state().rigid_body());
    let final_b = SimSnapshot::from_state(sim_b.step_index(), dt, sim_b.state().rigid_body());
    assert_eq!(
        final_a.state_hash(),
        final_b.state_hash(),
        "final state hash mismatch"
    );

    // Verify the hash is not the trivial zero hash (sanity).
    assert_ne!(
        final_a.state_hash(),
        blake3::Hash::from([0u8; 32]),
        "final hash is trivially zero"
    );
}
