use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError};
use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use replay::{
    AircraftReplayError, AircraftReplayPlayer, AircraftReplayRecording, AircraftSnapshotHash,
};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState,
    SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{io, path::PathBuf, time::Instant};
use telemetry::{
    AircraftTelemetryRecorder, AircraftTelemetryRecording, ScalarRange, ScalarStatistics,
    TelemetryCaptureError, TelemetryFinalState, TelemetrySummary,
};
use thiserror::Error;

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_THROTTLE: f64 = 0.55;
const DEFAULT_STEPS: u64 = 2_000;

#[derive(Debug, Clone)]
pub enum TelemetryOptions {
    Run(RunOptions),
    FromReplay(FromReplayOptions),
    Analyze(AnalyzeOptions),
}

impl TelemetryOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        match arguments.next().as_deref() {
            Some("run") => Ok(Self::Run(RunOptions::parse(arguments)?)),
            Some("from-replay") => Ok(Self::FromReplay(FromReplayOptions::parse(arguments)?)),
            Some("analyze") => Ok(Self::Analyze(AnalyzeOptions::parse(arguments)?)),
            Some(command) => Err(format!("unknown telemetry command: {command}")),
            None => Err(
                "missing telemetry command; expected `run`, `from-replay`, or `analyze`".to_owned(),
            ),
        }
    }
}

#[derive(Debug, Clone)]
pub struct RunOptions {
    model_path: PathBuf,
    output_path: PathBuf,
    steps: u64,
    physics_hz: u32,
    input: PilotInput,
}

impl RunOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut model_path = PathBuf::from(DEFAULT_MODEL_PATH);
        let mut output_path = None;
        let mut steps = DEFAULT_STEPS;
        let mut physics_hz = DEFAULT_PHYSICS_HZ;
        let mut roll = 0.0;
        let mut pitch = 0.0;
        let mut yaw = 0.0;
        let mut throttle = DEFAULT_THROTTLE;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => model_path = PathBuf::from(next_value("--model", &mut arguments)?),
                "--output" => {
                    output_path = Some(PathBuf::from(next_value("--output", &mut arguments)?));
                }
                "--steps" => steps = parse_value("--steps", &mut arguments)?,
                "--physics-hz" => {
                    physics_hz = parse_value("--physics-hz", &mut arguments)?;
                    if physics_hz == 0 {
                        return Err("--physics-hz must be greater than zero".to_owned());
                    }
                }
                "--roll" => roll = parse_value("--roll", &mut arguments)?,
                "--pitch" => pitch = parse_value("--pitch", &mut arguments)?,
                "--yaw" => yaw = parse_value("--yaw", &mut arguments)?,
                "--throttle" => throttle = parse_value("--throttle", &mut arguments)?,
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown telemetry run argument: {argument}")),
            }
        }
        validate_input_value("--roll", roll, -1.0, 1.0)?;
        validate_input_value("--pitch", pitch, -1.0, 1.0)?;
        validate_input_value("--yaw", yaw, -1.0, 1.0)?;
        validate_input_value("--throttle", throttle, 0.0, 1.0)?;
        Ok(Self {
            model_path,
            output_path: output_path.ok_or_else(|| "missing required --output".to_owned())?,
            steps,
            physics_hz,
            input: PilotInput::new(roll, pitch, yaw, throttle),
        })
    }
}

#[derive(Debug, Clone)]
pub struct FromReplayOptions {
    model_path: PathBuf,
    replay_path: PathBuf,
    output_path: PathBuf,
}

impl FromReplayOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut model_path = PathBuf::from(DEFAULT_MODEL_PATH);
        let mut replay_path = None;
        let mut output_path = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => model_path = PathBuf::from(next_value("--model", &mut arguments)?),
                "--replay" => {
                    replay_path = Some(PathBuf::from(next_value("--replay", &mut arguments)?));
                }
                "--output" => {
                    output_path = Some(PathBuf::from(next_value("--output", &mut arguments)?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => {
                    return Err(format!(
                        "unknown telemetry from-replay argument: {argument}"
                    ));
                }
            }
        }
        Ok(Self {
            model_path,
            replay_path: replay_path.ok_or_else(|| "missing required --replay".to_owned())?,
            output_path: output_path.ok_or_else(|| "missing required --output".to_owned())?,
        })
    }
}

#[derive(Debug, Clone)]
pub struct AnalyzeOptions {
    input_path: PathBuf,
}

impl AnalyzeOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut input_path = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--input" => {
                    input_path = Some(PathBuf::from(next_value("--input", &mut arguments)?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown telemetry analyze argument: {argument}")),
            }
        }
        Ok(Self {
            input_path: input_path.ok_or_else(|| "missing required --input".to_owned())?,
        })
    }
}

#[derive(Debug, Error)]
pub enum TelemetryAppError {
    #[error("failed to load aircraft model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to configure telemetry atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure telemetry simulation: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to initialize telemetry aircraft simulation: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error(transparent)]
    Replay(#[from] AircraftReplayError),
    #[error(transparent)]
    Telemetry(#[from] TelemetryCaptureError),
    #[error("step count {0} cannot fit in memory on this platform")]
    StepCountTooLarge(u64),
    #[error("verified replay unexpectedly ended before frame {0}")]
    ReplayEndedEarly(u64),
    #[error("failed to read {kind} from {path}: {source}")]
    Read {
        kind: &'static str,
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write telemetry capture to {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

pub fn run_telemetry(options: TelemetryOptions) -> Result<(), TelemetryAppError> {
    match options {
        TelemetryOptions::Run(options) => run(options),
        TelemetryOptions::FromReplay(options) => from_replay(options),
        TelemetryOptions::Analyze(options) => analyze(options),
    }
}

fn run(options: RunOptions) -> Result<(), TelemetryAppError> {
    let model = load_model(&options.model_path)?;
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(options.physics_hz, environment)?;
    let mut simulation = AircraftSimulation::new(model, config, standard_initial_state())?;
    let capacity = capacity(options.steps)?;
    let mut recorder = AircraftTelemetryRecorder::with_capacity(&simulation, capacity)?;
    for _ in 0..options.steps {
        let started = Instant::now();
        let snapshot = simulation.step(&options.input);
        let wall_time_s = started.elapsed().as_secs_f64();
        recorder.record(&simulation, options.input, &snapshot, Some(wall_time_s))?;
    }
    let recording = recorder.finish();
    write_capture(&options.output_path, &recording)?;
    println!("RC Simulation Engine");
    println!("mode: telemetry-run");
    println!("schema_version: {}", recording.schema_version());
    println!("model_id: {}", recording.model_id());
    println!("physics_hz: {}", options.physics_hz);
    println!("frames_recorded: {}", recording.frames().len());
    println!("output: {}", options.output_path.display());
    Ok(())
}

fn from_replay(options: FromReplayOptions) -> Result<(), TelemetryAppError> {
    let replay_json = read_text("aircraft replay", &options.replay_path)?;
    let replay = AircraftReplayRecording::from_json(&replay_json)?;
    let model = load_model(&options.model_path)?;
    let telemetry = capture_verified_replay(&replay, model)?;
    write_capture(&options.output_path, &telemetry)?;
    println!("RC Simulation Engine");
    println!("mode: telemetry-from-replay");
    println!("replay_schema_version: {}", replay.schema_version());
    println!("telemetry_schema_version: {}", telemetry.schema_version());
    println!("model_id: {}", telemetry.model_id());
    println!("frames_verified_and_recorded: {}", telemetry.frames().len());
    println!("verification: PASS");
    println!("output: {}", options.output_path.display());
    Ok(())
}

fn capture_verified_replay(
    replay: &AircraftReplayRecording,
    model: AircraftModel,
) -> Result<AircraftTelemetryRecording, TelemetryAppError> {
    let mut simulation = replay.reconstruct_simulation(model)?;
    let mut player = AircraftReplayPlayer::new(replay, &simulation)?;
    let mut recorder =
        AircraftTelemetryRecorder::with_capacity(&simulation, replay.frames().len())?;
    for frame in replay.frames() {
        let snapshot = player
            .verify_next(&mut simulation)?
            .ok_or(TelemetryAppError::ReplayEndedEarly(frame.step_index()))?;
        debug_assert_eq!(
            AircraftSnapshotHash::from_snapshot(&snapshot),
            frame.expected_snapshot_hash()
        );
        recorder.record(&simulation, frame.pilot_input(), &snapshot, None)?;
    }
    Ok(recorder.finish())
}

fn analyze(options: AnalyzeOptions) -> Result<(), TelemetryAppError> {
    let json_lines = read_text("telemetry capture", &options.input_path)?;
    let recording = AircraftTelemetryRecording::from_json_lines(&json_lines)?;
    let summary = recording.summary()?;
    print_summary(&recording, &summary, &options.input_path);
    Ok(())
}

fn print_summary(
    recording: &AircraftTelemetryRecording,
    summary: &TelemetrySummary,
    input_path: &std::path::Path,
) {
    let deterministic = &summary.deterministic;
    println!("RC Simulation Engine");
    println!("mode: telemetry-analyze");
    println!("input: {}", input_path.display());
    println!("schema_version: {}", recording.schema_version());
    println!("model_id: {}", recording.model_id());
    println!("frame_count: {}", deterministic.frame_count);
    print_optional_u64("first_step", deterministic.first_step);
    print_optional_u64("last_step", deterministic.last_step);
    println!(
        "simulated_duration_s: {:.12}",
        deterministic.simulated_duration_s
    );
    print_statistics("speed_mps", deterministic.speed_mps);
    print_range("north_m", deterministic.north_m);
    print_range("east_m", deterministic.east_m);
    print_range("down_m", deterministic.down_m);
    print_range("local_altitude_m", deterministic.local_altitude_m);
    print_optional_f64(
        "max_angular_speed_radps",
        deterministic.max_angular_speed_radps,
    );
    println!(
        "max_abs_input: roll={:.12}, pitch={:.12}, yaw={:.12}",
        deterministic.max_abs_roll_input,
        deterministic.max_abs_pitch_input,
        deterministic.max_abs_yaw_input
    );
    print_range("throttle_input", deterministic.throttle_input);
    print_range("aileron_angle_rad", deterministic.aileron_angle_rad);
    print_range("elevator_angle_rad", deterministic.elevator_angle_rad);
    print_range("rudder_angle_rad", deterministic.rudder_angle_rad);
    print_final_state(deterministic.final_state);
    if let Some(timing) = summary.physics_step_wall_time_s {
        println!("physics_step_wall_time_kind: non-deterministic-performance-data");
        print_statistics("physics_step_wall_time_s", Some(timing));
    } else {
        println!("physics_step_wall_time_s: unavailable");
    }
}

fn print_statistics(label: &str, statistics: Option<ScalarStatistics>) {
    if let Some(value) = statistics {
        println!(
            "{label}: min={:.12}, max={:.12}, mean={:.12}",
            value.min, value.max, value.mean
        );
    } else {
        println!("{label}: unavailable");
    }
}

fn print_range(label: &str, range: Option<ScalarRange>) {
    if let Some(value) = range {
        println!("{label}: min={:.12}, max={:.12}", value.min, value.max);
    } else {
        println!("{label}: unavailable");
    }
}

fn print_optional_u64(label: &str, value: Option<u64>) {
    if let Some(value) = value {
        println!("{label}: {value}");
    } else {
        println!("{label}: unavailable");
    }
}

fn print_optional_f64(label: &str, value: Option<f64>) {
    if let Some(value) = value {
        println!("{label}: {value:.12}");
    } else {
        println!("{label}: unavailable");
    }
}

fn print_final_state(final_state: Option<TelemetryFinalState>) {
    if let Some(state) = final_state {
        println!(
            "final_position_world_ned_m: {:?}",
            state.position_world_ned_m
        );
        println!(
            "final_linear_velocity_world_ned_mps: {:?}",
            state.linear_velocity_world_ned_mps
        );
        println!(
            "final_orientation_world_from_body_hamilton_wxyz: {:?}",
            state.orientation_world_from_body_hamilton_wxyz
        );
        println!(
            "final_angular_velocity_body_frd_radps: {:?}",
            state.angular_velocity_body_frd_radps
        );
    } else {
        println!("final_state: unavailable");
    }
}

fn write_capture(
    path: &PathBuf,
    recording: &AircraftTelemetryRecording,
) -> Result<(), TelemetryAppError> {
    let json_lines = recording.to_json_lines()?;
    std::fs::write(path, json_lines).map_err(|source| TelemetryAppError::Write {
        path: path.clone(),
        source,
    })
}

fn read_text(kind: &'static str, path: &PathBuf) -> Result<String, TelemetryAppError> {
    std::fs::read_to_string(path).map_err(|source| TelemetryAppError::Read {
        kind,
        path: path.clone(),
        source,
    })
}

fn load_model(path: &PathBuf) -> Result<AircraftModel, TelemetryAppError> {
    load_aircraft_model(path).map_err(|source| TelemetryAppError::ModelLoad {
        path: path.clone(),
        source,
    })
}

fn standard_initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn capacity(steps: u64) -> Result<usize, TelemetryAppError> {
    usize::try_from(steps).map_err(|_| TelemetryAppError::StepCountTooLarge(steps))
}

fn next_value(flag: &str, arguments: &mut impl Iterator<Item = String>) -> Result<String, String> {
    arguments
        .next()
        .ok_or_else(|| format!("missing value for {flag}"))
}

fn parse_value<T>(flag: &str, arguments: &mut impl Iterator<Item = String>) -> Result<T, String>
where
    T: std::str::FromStr,
{
    next_value(flag, arguments)?
        .parse()
        .map_err(|_| format!("invalid value for {flag}"))
}

fn validate_input_value(flag: &str, value: f64, minimum: f64, maximum: f64) -> Result<(), String> {
    if value.is_finite() && (minimum..=maximum).contains(&value) {
        Ok(())
    } else {
        Err(format!(
            "{flag} must be finite and inside [{minimum}, {maximum}]"
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn telemetry_cli_rejects_invalid_input_without_clamping() {
        let result = TelemetryOptions::parse(
            ["run", "--output", "target/telemetry.jsonl", "--roll", "NaN"]
                .map(str::to_owned)
                .into_iter(),
        );
        assert!(result.is_err());
    }

    #[test]
    fn telemetry_cli_requires_explicit_output_paths() {
        assert!(TelemetryOptions::parse(["run"].map(str::to_owned).into_iter()).is_err());
        assert!(
            TelemetryOptions::parse(
                ["from-replay", "--replay", "input.json"]
                    .map(str::to_owned)
                    .into_iter()
            )
            .is_err()
        );
    }

    #[test]
    fn telemetry_from_replay_stops_on_s8a_hash_divergence() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let replay_path = root.join("tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json");
        let mut json = std::fs::read_to_string(replay_path).unwrap();
        let marker = "\"expected_snapshot_hash\": \"";
        let hash_start = json.find(marker).unwrap() + marker.len();
        json.replace_range(
            hash_start..hash_start + 64,
            "0000000000000000000000000000000000000000000000000000000000000000",
        );
        let replay = AircraftReplayRecording::from_json(&json).unwrap();
        let model = load_aircraft_model(root.join(DEFAULT_MODEL_PATH)).unwrap();
        assert!(matches!(
            capture_verified_replay(&replay, model),
            Err(TelemetryAppError::Replay(
                AircraftReplayError::SnapshotHashMismatch { .. }
            ))
        ));
    }
}
