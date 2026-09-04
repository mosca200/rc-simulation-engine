#![forbid(unsafe_code)]

mod benchmark_app;
mod controller_app;
mod controller_profile_app;
mod first_slice_app;
mod input_app;
mod propulsion_bench_app;
mod render_app;
mod render_snapshot;
mod replay_app;
mod telemetry_app;
mod telemetry_experiment;
mod trim_characterization_app;
mod trim_sweep_validation_app;
mod validation_app;
mod xfoil_campaign_app;
mod xfoil_evidence_bundle_app;
mod xfoil_runner_app;

use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use benchmark_app::{AircraftBenchmarkOptions, run_aircraft_benchmark};
use controller_app::{ControllerCommand, run_controller};
use first_slice_app::{FirstSliceOptions, run_first_slice_validation};
use input_app::run_input_list;
use model::{AircraftModelFingerprint, load_aircraft_model};
use propulsion_bench_app::{PropulsionBenchOptions, run_propulsion_bench};
use render_app::{RenderOptions, run_render};
use replay::ReplayRecorder;
use replay_app::{ReplayOptions, run_replay};
use sim_core::{
    AeroEnvironment, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyParams, RigidBodyState, Simulation,
    SimulationConfig,
};
use sim_math::{Mat3, Orientation, Vec3};
use std::{env, error::Error, path::PathBuf, time::Instant};
use telemetry::{PerformanceDiagnostics, TelemetryFrame};
use telemetry_app::{TelemetryOptions, run_telemetry};
use tracing::info;
use tracing_subscriber::EnvFilter;
use validation_app::{ValidationOptions, run_validation};

fn main() -> Result<(), Box<dyn Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .with_target(false)
        .compact()
        .try_init()
        .ok();

    let mut arguments = env::args().skip(1);
    match arguments.next() {
        Some(command) if command == "controller" => {
            run_controller(ControllerCommand::parse(arguments)?)?;
            Ok(())
        }
        Some(command) if command == "input" => match arguments.next().as_deref() {
            Some("list") if arguments.next().is_none() => {
                run_input_list()?;
                Ok(())
            }
            Some(input_command) => Err(format!("unknown input command: {input_command}").into()),
            None => Err("missing input command; expected `list`".into()),
        },
        Some(command) if command == "benchmark" => match arguments.next().as_deref() {
            Some("aircraft") => {
                run_aircraft_benchmark(AircraftBenchmarkOptions::parse(arguments)?)?;
                Ok(())
            }
            Some(benchmark_command) => {
                Err(format!("unknown benchmark command: {benchmark_command}").into())
            }
            None => Err("missing benchmark command; expected `aircraft`".into()),
        },
        Some(command) if command == "propulsion" => match arguments.next().as_deref() {
            Some("bench") => {
                run_propulsion_bench(PropulsionBenchOptions::parse(arguments)?)?;
                Ok(())
            }
            Some(propulsion_command) => {
                Err(format!("unknown propulsion command: {propulsion_command}").into())
            }
            None => Err("missing propulsion command; expected `bench`".into()),
        },
        Some(command) if command == "replay" => {
            run_replay(ReplayOptions::parse(arguments)?)?;
            Ok(())
        }
        Some(command) if command == "telemetry" => {
            run_telemetry(TelemetryOptions::parse(arguments)?)?;
            Ok(())
        }
        Some(command) if command == "render" => {
            run_render(RenderOptions::parse(arguments)?)?;
            Ok(())
        }
        Some(command) if command == "xfoil" => match arguments.next().as_deref() {
            Some("run-campaign") => {
                let options = xfoil_runner_app::XfoilRunnerOptions::parse(arguments)?;
                match xfoil_runner_app::run_xfoil_campaign(options) {
                    Ok(xfoil_runner_app::XfoilRunnerStatus::Completed) => Ok(()),
                    Ok(xfoil_runner_app::XfoilRunnerStatus::Incomplete) => {
                        eprintln!(
                            "XFOIL campaign execution is incomplete; see xfoil_execution.md"
                        );
                        std::process::exit(2);
                    }
                    Err(error) => {
                        eprintln!("{error}");
                        std::process::exit(1);
                    }
                }
            }
            Some("build-evidence-bundle") => {
                let options =
                    xfoil_evidence_bundle_app::XfoilEvidenceBundleOptions::parse(arguments)?;
                match xfoil_evidence_bundle_app::run_xfoil_evidence_bundle(options) {
                    Ok(xfoil_evidence_bundle_app::XfoilEvidenceBundleStatus::Built) => Ok(()),
                    Ok(xfoil_evidence_bundle_app::XfoilEvidenceBundleStatus::NotPromotable) => {
                        eprintln!(
                            "XFOIL execution output is not promotable to an evidence bundle"
                        );
                        std::process::exit(2);
                    }
                    Err(error) => {
                        eprintln!("{error}");
                        std::process::exit(1);
                    }
                }
            }
            Some(command) => Err(format!("unknown XFOIL command: {command}").into()),
            None => Err(
                "missing XFOIL command; expected `run-campaign` or `build-evidence-bundle`"
                    .into(),
            ),
        },
        Some(command) if command == "analyze" => match arguments.next().as_deref() {
            Some("trim-characterization") => {
                let options =
                    trim_characterization_app::TrimCharacterizationOptions::parse(arguments)?;
                trim_characterization_app::run_trim_characterization(options)?;
                Ok(())
            }
            Some(analysis_target) => Err(format!("unknown analysis target: {analysis_target}").into()),
            None => Err("missing analysis target; expected `trim-characterization`".into()),
        },
        Some(command) if command == "validate" => match arguments.next().as_deref() {
            Some("xfoil-campaign") => {
                let options = xfoil_campaign_app::XfoilCampaignOptions::parse(arguments)?;
                match xfoil_campaign_app::run_xfoil_campaign_validation(options) {
                    Ok(xfoil_campaign_app::XfoilCampaignRunStatus::Qualified) => Ok(()),
                    Ok(xfoil_campaign_app::XfoilCampaignRunStatus::NotQualified) => {
                        eprintln!(
                            "XFOIL campaign analysis completed with status Not Qualified; see xfoil_campaign.md"
                        );
                        std::process::exit(2);
                    }
                    Err(error) => {
                        eprintln!("{error}");
                        std::process::exit(1);
                    }
                }
            }
            Some("first-slice") => {
                run_first_slice_validation(FirstSliceOptions::parse(arguments)?)?;
                Ok(())
            }
            Some("trim-sweep") => {
                let options =
                    trim_sweep_validation_app::TrimSweepValidationOptions::parse(arguments)?;
                match trim_sweep_validation_app::run_trim_sweep_validation(options) {
                    Ok(()) => Ok(()),
                    Err(trim_sweep_validation_app::TrimSweepValidationError::ValidationFailure {
                        total_points,
                        non_success_points,
                    }) => {
                        eprintln!(
                            "trim sweep validation completed with FAIL: {non_success_points} of {total_points} point(s) are not Success; see trim_sweep.md"
                        );
                        std::process::exit(2);
                    }
                    Err(error) => Err(Box::new(error) as Box<dyn Error>),
                }
            }
            Some(validation_target) => {
                run_validation(ValidationOptions::parse(
                    std::iter::once(validation_target.to_owned()).chain(arguments),
                )?)?;
                Ok(())
            }
            None => Err(
                "missing validation target; expected `acro-electric-01`, `first-slice`, `trim-sweep`, or `xfoil-campaign`".into(),
            ),
        },
        Some(command) if command == "aircraft" => run_aircraft(AircraftOptions::parse(arguments)?),
        Some(first_argument) => run_foundation(Options::parse(
            std::iter::once(first_argument).chain(arguments),
        )?),
        None => run_foundation(Options::parse(std::iter::empty())?),
    }
}

fn run_foundation(options: Options) -> Result<(), Box<dyn Error>> {
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

fn run_aircraft(options: AircraftOptions) -> Result<(), Box<dyn Error>> {
    let model = load_aircraft_model(&options.model_path)?;
    let model_id = model.model_id().to_owned();
    let display_name = model.display_name().to_owned();
    let fingerprint = model.physics_fingerprint();
    let initial_state = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(options.physics_hz, environment)?;
    let mut simulation = AircraftSimulation::new(model, config, initial_state)?;
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.55);

    info!(
        model_id,
        physics_hz = options.physics_hz,
        steps = options.steps,
        "starting headless aircraft simulation"
    );
    let started = Instant::now();
    let mut final_snapshot = None;
    for _ in 0..options.steps {
        final_snapshot = Some(simulation.step(&input));
    }
    let elapsed = started.elapsed();

    let (rigid_state, aileron_rad, elevator_rad, rudder_rad, throttle) =
        if let Some(snapshot) = final_snapshot.as_ref() {
            let positions = snapshot.control_surface_positions();
            (
                *snapshot.rigid_body_state(),
                positions.aileron_angle_rad(),
                positions.elevator_angle_rad(),
                positions.rudder_angle_rad(),
                positions.throttle(),
            )
        } else {
            let controls = simulation.state().controls().actuators();
            (
                *simulation.state().rigid_body(),
                controls.aileron().angle_rad(),
                controls.elevator().angle_rad(),
                controls.rudder().angle_rad(),
                input.throttle(),
            )
        };
    let quaternion = rigid_state.orientation_world_from_body.quaternion();
    let average_step_time_s = if options.steps == 0 {
        0.0
    } else {
        elapsed.as_secs_f64() / options.steps as f64
    };

    println!("RC Simulation Engine");
    println!("mode: aircraft-headless");
    println!("model_path: {}", options.model_path.display());
    println!("model_id: {model_id}");
    println!("display_name: {display_name}");
    print_fingerprint(&fingerprint);
    println!("physics_hz: {}", options.physics_hz);
    println!("steps: {}", options.steps);
    println!("simulated_time_s: {:.6}", simulation.sim_time_s());
    println!("final_position_ned_m: {:?}", rigid_state.position_world_m);
    println!(
        "final_velocity_ned_mps: {:?}",
        rigid_state.linear_velocity_world_mps
    );
    println!(
        "final_orientation_world_from_body_wxyz: [{:.12}, {:.12}, {:.12}, {:.12}]",
        quaternion.w, quaternion.i, quaternion.j, quaternion.k
    );
    println!(
        "final_angular_velocity_body_radps: {:?}",
        rigid_state.angular_velocity_body_radps
    );
    println!(
        "servo_positions_rad: aileron={aileron_rad:.12}, elevator={elevator_rad:.12}, rudder={rudder_rad:.12}"
    );
    println!("throttle: {throttle:.12}");
    println!("elapsed_wall_time: {:.6} s", elapsed.as_secs_f64());
    println!("average_step_time: {:.3} ns", average_step_time_s * 1.0e9);
    Ok(())
}

fn print_fingerprint(fingerprint: &AircraftModelFingerprint) {
    print!("model_physics_fingerprint: ");
    for byte in fingerprint.as_bytes() {
        print!("{byte:02x}");
    }
    println!();
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
                    print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown argument: {argument}")),
            }
        }
        Ok(options)
    }
}

#[derive(Debug, Clone)]
struct AircraftOptions {
    model_path: PathBuf,
    steps: u64,
    physics_hz: u32,
}

impl AircraftOptions {
    fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self {
            model_path: PathBuf::from("models/acro_electric_01/model.json"),
            steps: 1_000,
            physics_hz: DEFAULT_PHYSICS_HZ,
        };
        while let Some(argument) = args.next() {
            match argument.as_str() {
                "--model" => {
                    options.model_path = PathBuf::from(
                        args.next()
                            .ok_or_else(|| "missing value for --model".to_owned())?,
                    );
                }
                "--steps" => {
                    options.steps = parse_value("--steps", args.next())?;
                }
                "--physics-hz" => {
                    options.physics_hz = parse_value("--physics-hz", args.next())?;
                }
                "--help" | "-h" => {
                    print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown aircraft argument: {argument}")),
            }
        }
        Ok(options)
    }
}

fn print_usage() {
    println!("Usage:");
    println!("  rcsim-app [--steps N] [--physics-hz HZ]");
    println!("  rcsim-app aircraft [--model PATH] [--steps N] [--physics-hz HZ]");
    println!(
        "  rcsim-app benchmark aircraft [--model PATH] [--warmup-steps N] [--steps N] [--physics-hz HZ]"
    );
    println!(
        "  rcsim-app propulsion bench [--model PATH] [--throttle V --airspeed-mps MPS | sweep options] [--format table|csv|json] [--output PATH]"
    );
    println!(
        "  rcsim-app render [--model PATH] [--altitude-m M] [--airspeed-mps MPS] [--throttle VALUE] [--controller-profile PATH] [--start-on-ground] [--record-replay PATH] [--scenery none|flying-field] [--camera pilot|chase] [--camera-fov DEG] [--pilot-position X,Y,Z] [--chase-distance-m M] [--chase-height-m M] [--debug-overlays]"
    );
    println!("  rcsim-app controller list");
    println!(
        "  rcsim-app controller monitor [--raw] [--device-id ID] [--samples N] [--duration-seconds N]"
    );
    println!("  rcsim-app controller calibrate --output PATH [--deadzone VALUE]");
    println!("  rcsim-app input list");
    println!(
        "  rcsim-app replay record --model PATH --output PATH --steps N [--roll V] [--pitch V] [--yaw V] [--throttle V]"
    );
    println!("  rcsim-app replay verify --model PATH --input PATH");
    println!(
        "  rcsim-app telemetry run --model PATH --output PATH --steps N [--physics-hz HZ] [--roll V] [--pitch V] [--yaw V] [--throttle V]"
    );
    println!("  rcsim-app telemetry from-replay --model PATH --replay PATH --output PATH");
    println!("  rcsim-app telemetry analyze --input PATH");
    println!("  rcsim-app telemetry experiment --model PATH --schedule PATH --output PATH");
    println!("  rcsim-app validate acro-electric-01 --output-dir PATH");
    println!("  rcsim-app validate first-slice --output-dir PATH");
    println!("  rcsim-app validate xfoil-campaign --manifest PATH --output-dir PATH");
    println!(
        "  rcsim-app xfoil run-campaign --manifest PATH --xfoil-executable PATH --output-dir PATH [--timeout-seconds N]"
    );
    println!("  rcsim-app xfoil build-evidence-bundle --execution-dir PATH --output-dir PATH");
    println!(
        "  rcsim-app analyze trim-characterization --model PATH --speed-mps M [--speed-mps M]... --alpha-min-rad A --alpha-max-rad A --elevator-min A --elevator-max A --throttle-min A --throttle-max A --initial-alpha-rad A --initial-elevator A --initial-throttle A --force-tolerance-n N --moment-tolerance-nm N --max-iterations N --alpha-step-rad A --elevator-step E --output-dir PATH"
    );
    println!(
        "  rcsim-app validate trim-sweep --model PATH --speed-mps M [--speed-mps M]... --alpha-min-rad A --alpha-max-rad A --elevator-min A --elevator-max A --throttle-min A --throttle-max A --initial-alpha-rad A --initial-elevator A --initial-throttle A --force-tolerance-n N --moment-tolerance-nm N --max-iterations N --output-dir PATH"
    );
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
