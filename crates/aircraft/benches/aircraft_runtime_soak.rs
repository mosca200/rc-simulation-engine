//! M2.12A.4 — Full runtime soak performance diagnostic.
//!
//! Measures the complete AircraftSimulation::step() hot path over a long
//! deterministic schedule.  Reports wall time, mean step time, steps/s,
//! and realtime factor relative to 500 Hz.
//!
//! This is a DIAGNOSTIC benchmark — results are hardware-dependent and
//! must NOT be used as a CI pass/fail gate.

use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::AircraftModelLoader;
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::hint::black_box;
use std::time::Instant;

const FIXTURE: &str = include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");
const PHYSICS_HZ: u32 = 500;
const BENCHMARK_STEPS: u64 = 30_000; // 60 s at 500 Hz

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

/// The test fixture (`synthetic_non_reference_trim_v4`) binds only the
/// elevator aerodynamic control.  Elevator and throttle phases exercise
/// full aircraft physics; aileron and rudder phases exercise control-system
/// state only (actuator deflections change, aerodynamic geometry does not).
fn input_schedule() -> Vec<(PilotInput, u64)> {
    vec![
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 5000),
        (PilotInput::new(0.0, 0.1, 0.0, 0.50), 1000),
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000),
        (PilotInput::new(0.1, 0.0, 0.0, 0.50), 1000),
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000),
        (PilotInput::new(0.0, 0.0, 0.05, 0.50), 1000),
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000),
        (PilotInput::new(0.0, -0.08, 0.0, 0.60), 2000),
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 3000),
        (PilotInput::new(0.05, 0.05, -0.03, 0.55), 3000),
        (PilotInput::new(0.0, 0.0, 0.0, 0.50), 5000),
    ]
}

/// Run with: `cargo bench -p aircraft --bench aircraft_runtime_soak`
///
/// Prints a human-readable performance report to stdout.
/// This is NOT a criterion benchmark — it's a standalone diagnostic.
fn main() {
    let model = model();
    let config = config();
    let schedule = input_schedule();

    let total_scheduled_steps: u64 = schedule.iter().map(|(_, n)| n).sum();
    assert_eq!(total_scheduled_steps, BENCHMARK_STEPS);

    let mut sim = AircraftSimulation::new(model, config, initial_state()).unwrap();

    // Warm-up pass (not timed).
    for (input, steps) in &schedule {
        for _ in 0..*steps {
            black_box(sim.step(black_box(input)));
        }
    }

    // Post-warmup state sanity: verify the simulation has not diverged.
    let post_warmup_state = sim.state().rigid_body();
    assert!(
        post_warmup_state
            .position_world_m
            .iter()
            .all(|v| v.is_finite())
            && post_warmup_state
                .linear_velocity_world_mps
                .iter()
                .all(|v| v.is_finite())
            && post_warmup_state
                .angular_velocity_body_radps
                .iter()
                .all(|v| v.is_finite()),
        "benchmark: non-finite rigid-body state after warm-up"
    );
    let q = post_warmup_state.orientation_world_from_body.quaternion();
    assert!(
        [q.w, q.i, q.j, q.k].iter().all(|v| v.is_finite()),
        "benchmark: non-finite quaternion after warm-up"
    );
    assert!(
        post_warmup_state.validate().is_ok(),
        "benchmark: invalid rigid-body state after warm-up"
    );

    // Re-create for the timed run to ensure identical initial conditions.
    let mut sim = AircraftSimulation::new(
        AircraftModelLoader::from_json_str(FIXTURE).unwrap(),
        config,
        initial_state(),
    )
    .unwrap();

    let start = Instant::now();
    let mut step_count = 0_u64;

    for (input, steps) in &schedule {
        for _ in 0..*steps {
            black_box(sim.step(black_box(input)));
            step_count += 1;
        }
    }

    let elapsed = start.elapsed();

    // Post-timing state sanity: verify the timed simulation remains valid.
    let final_state = sim.state().rigid_body();
    assert!(
        final_state.position_world_m.iter().all(|v| v.is_finite())
            && final_state
                .linear_velocity_world_mps
                .iter()
                .all(|v| v.is_finite())
            && final_state
                .angular_velocity_body_radps
                .iter()
                .all(|v| v.is_finite()),
        "benchmark: non-finite rigid-body state after timed run"
    );
    let q_f = final_state.orientation_world_from_body.quaternion();
    assert!(
        [q_f.w, q_f.i, q_f.j, q_f.k].iter().all(|v| v.is_finite()),
        "benchmark: non-finite quaternion after timed run"
    );
    assert!(
        final_state.validate().is_ok(),
        "benchmark: invalid rigid-body state after timed run"
    );
    let elapsed_s = elapsed.as_secs_f64();
    let mean_step_s = elapsed_s / step_count as f64;
    let steps_per_second = step_count as f64 / elapsed_s;
    let timestep_budget_s = 1.0 / PHYSICS_HZ as f64;
    let realtime_factor = timestep_budget_s / mean_step_s;

    let model_id = sim.model().model_id();

    println!();
    println!("RC Simulation Engine");
    println!("mode: aircraft-runtime-benchmark");
    println!("model_id: {model_id}");
    println!("physics_hz: {PHYSICS_HZ}");
    println!("steps: {step_count}");
    println!(
        "simulated_duration_s: {:.1}",
        step_count as f64 / PHYSICS_HZ as f64
    );
    println!("elapsed_wall_time_s: {elapsed_s:.4}");
    println!("mean_step_wall_time_s: {mean_step_s:.9}");
    println!("steps_per_second: {steps_per_second:.1}");
    println!("realtime_factor: {realtime_factor:.2}");
    println!();
    println!("NOTE: performance data is hardware-dependent and diagnostic only.");
    println!("realtime_factor > 1 means faster than realtime at {PHYSICS_HZ} Hz on this machine.");
}
