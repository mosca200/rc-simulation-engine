use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError};
use model::{ModelLoadError, load_aircraft_model};
use replay::{
    AircraftReplayError, AircraftReplayPlayer, AircraftReplayRecorder, AircraftReplayRecording,
};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState,
    SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{io, path::PathBuf};
use thiserror::Error;

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_THROTTLE: f64 = 0.55;

#[derive(Debug, Clone)]
pub enum ReplayOptions {
    Record(RecordOptions),
    Verify(VerifyOptions),
}

impl ReplayOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        match arguments.next().as_deref() {
            Some("record") => Ok(Self::Record(RecordOptions::parse(arguments)?)),
            Some("verify") => Ok(Self::Verify(VerifyOptions::parse(arguments)?)),
            Some(command) => Err(format!("unknown replay command: {command}")),
            None => Err("missing replay command; expected `record` or `verify`".to_owned()),
        }
    }
}

#[derive(Debug, Clone)]
pub struct RecordOptions {
    model_path: PathBuf,
    output_path: PathBuf,
    steps: u64,
    physics_hz: u32,
    input: PilotInput,
}

impl RecordOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut model_path = PathBuf::from(DEFAULT_MODEL_PATH);
        let mut output_path = None;
        let mut steps = None;
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
                "--steps" => steps = Some(parse_value("--steps", &mut arguments)?),
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
                _ => return Err(format!("unknown replay record argument: {argument}")),
            }
        }
        let output_path = output_path.ok_or_else(|| "missing required --output".to_owned())?;
        let steps = steps.ok_or_else(|| "missing required --steps".to_owned())?;
        validate_input_value("--roll", roll, -1.0, 1.0)?;
        validate_input_value("--pitch", pitch, -1.0, 1.0)?;
        validate_input_value("--yaw", yaw, -1.0, 1.0)?;
        validate_input_value("--throttle", throttle, 0.0, 1.0)?;
        Ok(Self {
            model_path,
            output_path,
            steps,
            physics_hz,
            input: PilotInput::new(roll, pitch, yaw, throttle),
        })
    }
}

#[derive(Debug, Clone)]
pub struct VerifyOptions {
    model_path: PathBuf,
    input_path: PathBuf,
}

impl VerifyOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut model_path = PathBuf::from(DEFAULT_MODEL_PATH);
        let mut input_path = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => model_path = PathBuf::from(next_value("--model", &mut arguments)?),
                "--input" => {
                    input_path = Some(PathBuf::from(next_value("--input", &mut arguments)?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown replay verify argument: {argument}")),
            }
        }
        Ok(Self {
            model_path,
            input_path: input_path.ok_or_else(|| "missing required --input".to_owned())?,
        })
    }
}

#[derive(Debug, Error)]
pub enum ReplayAppError {
    #[error("failed to load aircraft model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to configure replay atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure replay simulation: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to initialize replay aircraft simulation: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error(transparent)]
    Replay(#[from] AircraftReplayError),
    #[error("replay step count {0} cannot fit in memory on this platform")]
    StepCountTooLarge(u64),
    #[error("failed to read aircraft replay from {path}: {source}")]
    Read {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write aircraft replay to {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

pub fn run_replay(options: ReplayOptions) -> Result<(), ReplayAppError> {
    match options {
        ReplayOptions::Record(options) => record(options),
        ReplayOptions::Verify(options) => verify(options),
    }
}

fn record(options: RecordOptions) -> Result<(), ReplayAppError> {
    let model = load_model(&options.model_path)?;
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(options.physics_hz, environment)?;
    let mut simulation = AircraftSimulation::new(model, config, standard_initial_state())?;
    let capacity = usize::try_from(options.steps)
        .map_err(|_| ReplayAppError::StepCountTooLarge(options.steps))?;
    let mut recorder = AircraftReplayRecorder::with_capacity(&simulation, capacity)?;
    for step_index in 0..options.steps {
        let _ = recorder.record(&mut simulation, step_index, options.input)?;
    }
    let recording = recorder.finish();
    let json = recording.to_json_pretty()?;
    std::fs::write(&options.output_path, json).map_err(|source| ReplayAppError::Write {
        path: options.output_path.clone(),
        source,
    })?;

    println!("RC Simulation Engine");
    println!("mode: aircraft-replay-record");
    println!("schema_version: {}", recording.schema_version());
    println!("model_id: {}", recording.model_id());
    println!("steps_recorded: {}", recording.frames().len());
    println!("output: {}", options.output_path.display());
    Ok(())
}

fn verify(options: VerifyOptions) -> Result<(), ReplayAppError> {
    let json =
        std::fs::read_to_string(&options.input_path).map_err(|source| ReplayAppError::Read {
            path: options.input_path.clone(),
            source,
        })?;
    let recording = AircraftReplayRecording::from_json(&json)?;
    let model = load_model(&options.model_path)?;
    let mut simulation = recording.reconstruct_simulation(model)?;
    let player = AircraftReplayPlayer::new(&recording, &simulation)?;
    let steps_verified = player.verify_all(&mut simulation)?;

    println!("RC Simulation Engine");
    println!("mode: aircraft-replay-verify");
    println!("schema_version: {}", recording.schema_version());
    println!("model_id: {}", recording.model_id());
    println!("steps_verified: {steps_verified}");
    println!("verification: PASS");
    Ok(())
}

fn load_model(path: &PathBuf) -> Result<model::AircraftModel, ReplayAppError> {
    load_aircraft_model(path).map_err(|source| ReplayAppError::ModelLoad {
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
    fn record_cli_rejects_out_of_range_input_without_clamping() {
        let result = ReplayOptions::parse(
            [
                "record",
                "--output",
                "target/replay.json",
                "--steps",
                "1",
                "--roll",
                "1.1",
            ]
            .map(str::to_owned)
            .into_iter(),
        );
        assert!(result.is_err());
    }
}
