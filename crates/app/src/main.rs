#![forbid(unsafe_code)]

use replay::ReplayRecorder;
use sim_core::{
    DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyParams, RigidBodyState, Simulation, SimulationConfig,
};
use sim_math::{Mat3, Orientation, Vec3};
use std::{env, error::Error, time::Instant};
use telemetry::{PerformanceDiagnostics, TelemetryFrame};
use tracing::info;
use tracing_subscriber::EnvFilter;

fn main() -> Result<(), Box<dyn Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_target(false)
        .compact()
        .try_init()
        .ok();

    let options = Options::parse(env::args().skip(1))?;
    let config = SimulationConfig::from_physics_hz(options.physics_hz)?;
    let initial_state = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(12.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::new(0.0, 0.0, 0.05),
    };
    let params = RigidBodyParams::new(2.0, Mat3::from_diagonal(&Vec3::new(0.08, 0.12, 0.16)))?;
    let mut simulation = Simulation::new(config, params, initial_state)?;
    let input = PilotInput::neutral();
    let mut recorder = ReplayRecorder::with_capacity(&simulation, options.steps as usize)?;

    info!(
        physics_hz = options.physics_hz,
        steps = options.steps,
        "starting headless simulation"
    );
    let started = Instant::now();
    let mut final_snapshot = simulation.snapshot();
    for step_index in 0..options.steps {
        recorder.record(step_index, input)?;
        final_snapshot = simulation.step(&input);
    }
    let elapsed = started.elapsed();

    let diagnostics = PerformanceDiagnostics {
        elapsed_wall_time_s: elapsed.as_secs_f64(),
        average_step_time_s: if options.steps == 0 {
            0.0
        } else {
            elapsed.as_secs_f64() / options.steps as f64
        },
    };
    let _final_telemetry = TelemetryFrame {
        snapshot: final_snapshot,
        pilot_input: input,
    };
    let _recording = recorder.finish();
    let quaternion = final_snapshot.orientation_world_from_body.quaternion();

    println!("RC Simulation Engine");
    println!("mode: headless");
    println!("physics_hz: {}", options.physics_hz);
    println!("steps: {}", options.steps);
    println!("simulated_time_s: {:.6}", final_snapshot.sim_time_s);
    println!(
        "final_position_ned_m: {:?}",
        final_snapshot.position_world_m
    );
    println!(
        "final_velocity_ned_mps: {:?}",
        final_snapshot.linear_velocity_world_mps
    );
    println!(
        "final_orientation_world_from_body_wxyz: [{:.12}, {:.12}, {:.12}, {:.12}]",
        quaternion.w, quaternion.i, quaternion.j, quaternion.k
    );
    println!("state_hash: {}", final_snapshot.state_hash().to_hex());
    println!(
        "elapsed_wall_time: {:.6} s",
        diagnostics.elapsed_wall_time_s
    );
    println!(
        "average_step_time: {:.3} ns",
        diagnostics.average_step_time_s * 1.0e9
    );
    Ok(())
}

#[derive(Debug, Clone, Copy)]
struct Options {
    steps: u64,
    physics_hz: u32,
}

impl Options {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self {
            steps: 5_000,
            physics_hz: DEFAULT_PHYSICS_HZ,
        };
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--steps" => {
                    options.steps = parse_value("--steps", args.next())?;
                }
                "--physics-hz" => {
                    options.physics_hz = parse_value("--physics-hz", args.next())?;
                }
                "--help" | "-h" => {
                    println!("Usage: rcsim-app [--steps N] [--physics-hz HZ]");
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown argument: {argument}")),
            }
        }
        Ok(options)
    }
}

fn parse_value<T>(flag: &str, value: Option<String>) -> Result<T, String>
where
    T: std::str::FromStr,
{
    value
        .ok_or_else(|| format!("missing value for {flag}"))?
        .parse()
        .map_err(|_| format!("invalid value for {flag}"))
}
