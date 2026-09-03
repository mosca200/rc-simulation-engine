use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError};
use model::{ModelLoadError, load_aircraft_model};
use serde::Deserialize;
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, PilotInput, RigidBodyState, SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{collections::HashMap, io, path::PathBuf};
use telemetry::{
    AIRCRAFT_TELEMETRY_SCHEMA_VERSION, AircraftTelemetryRecorder, AircraftTelemetryRecording,
    TelemetryCaptureError,
};
use thiserror::Error;

pub const FLIGHT_EXPERIMENT_SCHEMA_VERSION: u32 = 1;

/// Deterministic resource policy for an in-memory experiment capture.
const MAX_EXPERIMENT_STEPS: u64 = 1_000_000;
const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";

#[derive(Debug, Clone)]
pub struct ExperimentOptions {
    model_path: PathBuf,
    schedule_path: PathBuf,
    output_path: PathBuf,
}

impl ExperimentOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut model_path = PathBuf::from(DEFAULT_MODEL_PATH);
        let mut schedule_path = None;
        let mut output_path = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => model_path = PathBuf::from(next_value("--model", &mut arguments)?),
                "--schedule" => {
                    schedule_path = Some(PathBuf::from(next_value("--schedule", &mut arguments)?));
                }
                "--output" => {
                    output_path = Some(PathBuf::from(next_value("--output", &mut arguments)?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown telemetry experiment argument: {argument}")),
            }
        }
        Ok(Self {
            model_path,
            schedule_path: schedule_path.ok_or_else(|| "missing required --schedule".to_owned())?,
            output_path: output_path.ok_or_else(|| "missing required --output".to_owned())?,
        })
    }
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct FlightExperimentScheduleDto {
    schema_version: u32,
    physics_hz: u32,
    initial_state: ExperimentInitialStateDto,
    phases: Vec<ExperimentPhaseDto>,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExperimentInitialStateDto {
    altitude_m: f64,
    airspeed_mps: f64,
    pitch_attitude_rad: f64,
    angular_velocity_body_radps: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExperimentPhaseDto {
    name: String,
    steps: u64,
    input: ExperimentInputDto,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExperimentInputDto {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

#[derive(Debug, Clone, PartialEq)]
struct FlightExperimentSchedule {
    physics_hz: u32,
    initial_state: ExperimentInitialStateDto,
    phases: Vec<ExperimentPhase>,
    total_steps: u64,
}

#[derive(Debug, Clone, PartialEq)]
struct ExperimentPhase {
    name: String,
    steps: u64,
    input: PilotInput,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct PhaseSummary {
    name: String,
    steps: u64,
    first_step: u64,
    last_step: u64,
}

struct ExperimentExecution {
    recording: AircraftTelemetryRecording,
    phases: Vec<PhaseSummary>,
}

#[derive(Debug, Error)]
pub enum TelemetryExperimentError {
    #[error("failed to read experiment schedule from {path}: {source}")]
    ReadSchedule {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("experiment schedule JSON is invalid: {0}")]
    InvalidJson(#[from] serde_json::Error),
    #[error("unsupported experiment schema version {0}")]
    UnsupportedSchemaVersion(u32),
    #[error("experiment physics_hz must be greater than zero")]
    ZeroPhysicsRate,
    #[error("experiment phase list must not be empty")]
    EmptyPhases,
    #[error("experiment initial-state field {field} has invalid value {value:?}")]
    InvalidInitialState { field: &'static str, value: f64 },
    #[error("experiment phase {index} has an empty name")]
    EmptyPhaseName { index: usize },
    #[error(
        "experiment phase name {name:?} at index {duplicate_index} duplicates index {first_index}"
    )]
    DuplicatePhaseName {
        name: String,
        first_index: usize,
        duplicate_index: usize,
    },
    #[error("experiment phase {name:?} at index {index} has zero steps")]
    ZeroPhaseSteps { name: String, index: usize },
    #[error("experiment phase {name:?} at index {index} has invalid input {field}={value:?}")]
    InvalidPhaseInput {
        name: String,
        index: usize,
        field: &'static str,
        value: f64,
    },
    #[error("experiment total step count overflows u64")]
    TotalStepOverflow,
    #[error("experiment total step count {steps} exceeds deterministic recorder limit {limit}")]
    RecorderCapacityExceeded { steps: u64, limit: u64 },
    #[error("failed to load aircraft model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to configure experiment atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure experiment simulation: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to initialize experiment aircraft simulation: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error(transparent)]
    Telemetry(#[from] TelemetryCaptureError),
    #[error("failed to write experiment telemetry to {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

pub fn run_experiment(options: ExperimentOptions) -> Result<(), TelemetryExperimentError> {
    let schedule_json = std::fs::read_to_string(&options.schedule_path).map_err(|source| {
        TelemetryExperimentError::ReadSchedule {
            path: options.schedule_path.clone(),
            source,
        }
    })?;
    let schedule = parse_schedule(&schedule_json)?;
    let model = load_aircraft_model(&options.model_path).map_err(|source| {
        TelemetryExperimentError::ModelLoad {
            path: options.model_path.clone(),
            source,
        }
    })?;
    let model_id = model.model_id().to_owned();
    let execution = execute_experiment(model, &schedule)?;
    let json_lines = execution.recording.to_json_lines()?;
    std::fs::write(&options.output_path, json_lines).map_err(|source| {
        TelemetryExperimentError::Write {
            path: options.output_path.clone(),
            source,
        }
    })?;

    println!("RC Simulation Engine");
    println!("mode: telemetry-experiment");
    println!("schema_version: {AIRCRAFT_TELEMETRY_SCHEMA_VERSION}");
    println!("experiment_schema_version: {FLIGHT_EXPERIMENT_SCHEMA_VERSION}");
    println!("model_id: {model_id}");
    println!("physics_hz: {}", schedule.physics_hz);
    println!("phase_count: {}", execution.phases.len());
    println!("total_steps: {}", schedule.total_steps);
    println!("frames_recorded: {}", execution.recording.frames().len());
    println!("output: {}", options.output_path.display());
    for (index, phase) in execution.phases.iter().enumerate() {
        println!(
            "phase[{index}]: {} steps={} range=[{},{}]",
            phase.name, phase.steps, phase.first_step, phase.last_step
        );
    }
    Ok(())
}

fn parse_schedule(json: &str) -> Result<FlightExperimentSchedule, TelemetryExperimentError> {
    let dto: FlightExperimentScheduleDto = serde_json::from_str(json)?;
    validate_schedule(dto)
}

fn validate_schedule(
    dto: FlightExperimentScheduleDto,
) -> Result<FlightExperimentSchedule, TelemetryExperimentError> {
    if dto.schema_version != FLIGHT_EXPERIMENT_SCHEMA_VERSION {
        return Err(TelemetryExperimentError::UnsupportedSchemaVersion(
            dto.schema_version,
        ));
    }
    if dto.physics_hz == 0 {
        return Err(TelemetryExperimentError::ZeroPhysicsRate);
    }
    if dto.phases.is_empty() {
        return Err(TelemetryExperimentError::EmptyPhases);
    }
    validate_initial_state(dto.initial_state)?;

    let mut phases = Vec::with_capacity(dto.phases.len());
    let mut names = HashMap::with_capacity(dto.phases.len());
    let mut total_steps = 0_u64;
    for (index, phase) in dto.phases.into_iter().enumerate() {
        if phase.name.trim().is_empty() {
            return Err(TelemetryExperimentError::EmptyPhaseName { index });
        }
        if let Some(first_index) = names.get(&phase.name) {
            return Err(TelemetryExperimentError::DuplicatePhaseName {
                name: phase.name,
                first_index: *first_index,
                duplicate_index: index,
            });
        }
        if phase.steps == 0 {
            return Err(TelemetryExperimentError::ZeroPhaseSteps {
                name: phase.name,
                index,
            });
        }
        let input = validate_input(&phase.name, index, phase.input)?;
        total_steps = total_steps
            .checked_add(phase.steps)
            .ok_or(TelemetryExperimentError::TotalStepOverflow)?;
        names.insert(phase.name.clone(), index);
        phases.push(ExperimentPhase {
            name: phase.name,
            steps: phase.steps,
            input,
        });
    }
    if total_steps > MAX_EXPERIMENT_STEPS || usize::try_from(total_steps).is_err() {
        return Err(TelemetryExperimentError::RecorderCapacityExceeded {
            steps: total_steps,
            limit: MAX_EXPERIMENT_STEPS,
        });
    }
    Ok(FlightExperimentSchedule {
        physics_hz: dto.physics_hz,
        initial_state: dto.initial_state,
        phases,
        total_steps,
    })
}

fn validate_initial_state(
    state: ExperimentInitialStateDto,
) -> Result<(), TelemetryExperimentError> {
    validate_initial("altitude_m", state.altitude_m, |value| value.is_finite())?;
    validate_initial("airspeed_mps", state.airspeed_mps, |value| {
        value.is_finite() && value >= 0.0
    })?;
    validate_initial("pitch_attitude_rad", state.pitch_attitude_rad, |value| {
        value.is_finite()
    })?;
    for (field, value) in [
        (
            "angular_velocity_body_radps[0]",
            state.angular_velocity_body_radps[0],
        ),
        (
            "angular_velocity_body_radps[1]",
            state.angular_velocity_body_radps[1],
        ),
        (
            "angular_velocity_body_radps[2]",
            state.angular_velocity_body_radps[2],
        ),
    ] {
        validate_initial(field, value, f64::is_finite)?;
    }
    Ok(())
}

fn validate_initial(
    field: &'static str,
    value: f64,
    valid: impl FnOnce(f64) -> bool,
) -> Result<(), TelemetryExperimentError> {
    if valid(value) {
        Ok(())
    } else {
        Err(TelemetryExperimentError::InvalidInitialState { field, value })
    }
}

fn validate_input(
    phase_name: &str,
    phase_index: usize,
    input: ExperimentInputDto,
) -> Result<PilotInput, TelemetryExperimentError> {
    for (field, value, minimum, maximum) in [
        ("roll", input.roll, -1.0, 1.0),
        ("pitch", input.pitch, -1.0, 1.0),
        ("yaw", input.yaw, -1.0, 1.0),
        ("throttle", input.throttle, 0.0, 1.0),
    ] {
        if !value.is_finite() || !(minimum..=maximum).contains(&value) {
            return Err(TelemetryExperimentError::InvalidPhaseInput {
                name: phase_name.to_owned(),
                index: phase_index,
                field,
                value,
            });
        }
    }
    Ok(PilotInput::new(
        input.roll,
        input.pitch,
        input.yaw,
        input.throttle,
    ))
}

fn initial_rigid_body_state(initial: ExperimentInitialStateDto) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -initial.altitude_m),
        linear_velocity_world_mps: Vec3::new(initial.airspeed_mps, 0.0, 0.0),
        orientation_world_from_body: Orientation::from_axis_angle(
            &Vec3::y_axis(),
            initial.pitch_attitude_rad,
        ),
        angular_velocity_body_radps: Vec3::new(
            initial.angular_velocity_body_radps[0],
            initial.angular_velocity_body_radps[1],
            initial.angular_velocity_body_radps[2],
        ),
    }
}

fn execute_experiment(
    model: model::AircraftModel,
    schedule: &FlightExperimentSchedule,
) -> Result<ExperimentExecution, TelemetryExperimentError> {
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(schedule.physics_hz, environment)?;
    let mut simulation = AircraftSimulation::new(
        model,
        config,
        initial_rigid_body_state(schedule.initial_state),
    )?;
    let capacity = usize::try_from(schedule.total_steps).map_err(|_| {
        TelemetryExperimentError::RecorderCapacityExceeded {
            steps: schedule.total_steps,
            limit: MAX_EXPERIMENT_STEPS,
        }
    })?;
    let mut recorder = AircraftTelemetryRecorder::with_capacity(&simulation, capacity)?;
    let mut phase_summaries = Vec::with_capacity(schedule.phases.len());
    let mut completed_steps = 0_u64;
    for phase in &schedule.phases {
        let first_step = completed_steps + 1;
        for _ in 0..phase.steps {
            let snapshot = simulation.step(&phase.input);
            recorder.record(&simulation, phase.input, &snapshot, None)?;
        }
        completed_steps += phase.steps;
        phase_summaries.push(PhaseSummary {
            name: phase.name.clone(),
            steps: phase.steps,
            first_step,
            last_step: completed_steps,
        });
    }
    debug_assert_eq!(completed_steps, schedule.total_steps);
    Ok(ExperimentExecution {
        recording: recorder.finish(),
        phases: phase_summaries,
    })
}

fn next_value(flag: &str, arguments: &mut impl Iterator<Item = String>) -> Result<String, String> {
    arguments
        .next()
        .ok_or_else(|| format!("missing value for {flag}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};

    fn valid_json() -> Value {
        json!({
            "schema_version": 1,
            "physics_hz": 500,
            "initial_state": {
                "altitude_m": 100.0,
                "airspeed_mps": 18.0,
                "pitch_attitude_rad": 0.1,
                "angular_velocity_body_radps": [0.01, -0.02, 0.03]
            },
            "phases": [
                {"name": "neutral", "steps": 10, "input": {
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5
                }},
                {"name": "pulse", "steps": 5, "input": {
                    "roll": 0.0, "pitch": 0.2, "yaw": 0.0, "throttle": 0.5
                }}
            ]
        })
    }

    fn parse_value(value: &Value) -> Result<FlightExperimentSchedule, TelemetryExperimentError> {
        parse_schedule(&serde_json::to_string(value).unwrap())
    }

    fn model() -> model::AircraftModel {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        load_aircraft_model(root.join(DEFAULT_MODEL_PATH)).unwrap()
    }

    #[test]
    fn valid_schedule_parses_and_initial_state_uses_ned_pitch_semantics() {
        let schedule = parse_value(&valid_json()).unwrap();
        assert_eq!(schedule.physics_hz, 500);
        assert_eq!(schedule.total_steps, 15);
        assert_eq!(schedule.phases.len(), 2);
        let state = initial_rigid_body_state(schedule.initial_state);
        assert_eq!(state.position_world_m, Vec3::new(0.0, 0.0, -100.0));
        assert_eq!(state.linear_velocity_world_mps, Vec3::new(18.0, 0.0, 0.0));
        assert_eq!(
            state.angular_velocity_body_radps,
            Vec3::new(0.01, -0.02, 0.03)
        );
        let forward_world = state
            .orientation_world_from_body
            .transform_vector(&Vec3::new(1.0, 0.0, 0.0));
        assert!(forward_world.z < 0.0);
    }

    #[test]
    fn schema_rate_phase_names_and_steps_fail_closed() {
        let mut value = valid_json();
        value["schema_version"] = json!(2);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::UnsupportedSchemaVersion(2))
        ));
        value = valid_json();
        value["physics_hz"] = json!(0);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::ZeroPhysicsRate)
        ));
        value = valid_json();
        value["phases"] = json!([]);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::EmptyPhases)
        ));
        value = valid_json();
        value["phases"][0]["steps"] = json!(0);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::ZeroPhaseSteps { index: 0, .. })
        ));
        value = valid_json();
        value["phases"][1]["name"] = json!("neutral");
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::DuplicatePhaseName {
                first_index: 0,
                duplicate_index: 1,
                ..
            })
        ));
        value = valid_json();
        value["phases"][0]["name"] = json!("   ");
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::EmptyPhaseName { index: 0 })
        ));
    }

    #[test]
    fn input_ranges_are_rejected_without_clamping() {
        for (field, invalid) in [
            ("roll", -1.01),
            ("pitch", 1.01),
            ("yaw", 1.5),
            ("throttle", -0.01),
            ("throttle", 1.01),
        ] {
            let mut value = valid_json();
            value["phases"][0]["input"][field] = json!(invalid);
            assert!(matches!(
                parse_value(&value),
                Err(TelemetryExperimentError::InvalidPhaseInput { .. })
            ));
        }
    }

    #[test]
    fn non_finite_initial_and_input_values_are_rejected_by_construction() {
        let mut dto: FlightExperimentScheduleDto = serde_json::from_value(valid_json()).unwrap();
        dto.initial_state.altitude_m = f64::NAN;
        assert!(matches!(
            validate_schedule(dto),
            Err(TelemetryExperimentError::InvalidInitialState { .. })
        ));

        let mut dto: FlightExperimentScheduleDto = serde_json::from_value(valid_json()).unwrap();
        dto.initial_state.airspeed_mps = -1.0;
        assert!(matches!(
            validate_schedule(dto),
            Err(TelemetryExperimentError::InvalidInitialState {
                field: "airspeed_mps",
                ..
            })
        ));

        let mut dto: FlightExperimentScheduleDto = serde_json::from_value(valid_json()).unwrap();
        dto.phases[0].input.pitch = f64::INFINITY;
        assert!(matches!(
            validate_schedule(dto),
            Err(TelemetryExperimentError::InvalidPhaseInput { field: "pitch", .. })
        ));
    }

    #[test]
    fn malformed_missing_unknown_and_oversized_schedules_are_rejected() {
        assert!(matches!(
            parse_schedule("{"),
            Err(TelemetryExperimentError::InvalidJson(_))
        ));
        let mut value = valid_json();
        value.as_object_mut().unwrap().remove("initial_state");
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::InvalidJson(_))
        ));
        value = valid_json();
        value["unexpected"] = json!(true);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::InvalidJson(_))
        ));
        value = valid_json();
        value["phases"][0]["steps"] = json!(MAX_EXPERIMENT_STEPS);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::RecorderCapacityExceeded { .. })
        ));
        value = valid_json();
        value["phases"][0]["steps"] = json!(u64::MAX);
        assert!(matches!(
            parse_value(&value),
            Err(TelemetryExperimentError::TotalStepOverflow)
        ));
    }

    #[test]
    fn execution_has_exact_phase_boundaries_and_is_byte_deterministic() {
        let schedule = parse_value(&valid_json()).unwrap();
        let first = execute_experiment(model(), &schedule).unwrap();
        let second = execute_experiment(model(), &schedule).unwrap();
        assert_eq!(first.recording.frames().len(), 15);
        assert_eq!(first.phases[0].first_step, 1);
        assert_eq!(first.phases[0].last_step, 10);
        assert_eq!(first.phases[1].first_step, 11);
        assert_eq!(first.phases[1].last_step, 15);
        for frame in &first.recording.frames()[0..10] {
            assert_eq!(frame.pilot_input(), schedule.phases[0].input);
            assert_eq!(frame.physics_step_wall_time_s(), None);
        }
        for frame in &first.recording.frames()[10..15] {
            assert_eq!(frame.pilot_input(), schedule.phases[1].input);
            assert_eq!(frame.physics_step_wall_time_s(), None);
        }
        assert_eq!(
            first.recording.to_json_lines().unwrap(),
            second.recording.to_json_lines().unwrap()
        );
    }

    #[test]
    fn cli_options_require_schedule_and_output() {
        assert!(ExperimentOptions::parse(std::iter::empty()).is_err());
        assert!(
            ExperimentOptions::parse(
                ["--schedule", "schedule.json"]
                    .map(str::to_owned)
                    .into_iter()
            )
            .is_err()
        );
    }
}
