use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError};
use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use replay::{
    AircraftReplayError, AircraftReplayPlayer, AircraftReplayRecorder, AircraftReplayRecording,
};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState,
    SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{fmt::Write as _, io, path::PathBuf};
use telemetry::{
    AircraftTelemetryRecorder, AircraftTelemetryRecording, DeterministicTelemetrySummary,
    ScalarRange, TelemetryCaptureError,
};
use thiserror::Error;

const MODEL_PATH: &str = "models/acro_electric_01/model.json";
const VALIDATION_SUITE_VERSION: u32 = 1;
const INITIAL_SPEED_MPS: f64 = 18.0;
const INITIAL_LOCAL_ALTITUDE_M: f64 = 100.0;

#[derive(Debug, Clone)]
pub struct ValidationOptions {
    output_dir: PathBuf,
}

impl ValidationOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        match arguments.next().as_deref() {
            Some("acro-electric-01") => {}
            Some(model) => return Err(format!("unsupported validation model: {model}")),
            None => return Err("missing validation model; expected `acro-electric-01`".to_owned()),
        }
        let mut output_dir = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--output-dir" => {
                    output_dir =
                        Some(PathBuf::from(arguments.next().ok_or_else(|| {
                            "missing value for --output-dir".to_owned()
                        })?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(format!("unknown validation argument: {argument}")),
            }
        }
        Ok(Self {
            output_dir: output_dir.ok_or_else(|| "missing required --output-dir".to_owned())?,
        })
    }
}

#[derive(Debug, Error)]
pub enum ValidationAppError {
    #[error("failed to load validation model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to configure validation atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure validation simulation: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to initialize validation aircraft simulation: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error(transparent)]
    Replay(#[from] AircraftReplayError),
    #[error(transparent)]
    Telemetry(#[from] TelemetryCaptureError),
    #[error("validation replay verified {actual} steps, expected {expected}")]
    ReplayStepCount { expected: u64, actual: u64 },
    #[error("failed to create validation output directory {path}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write validation artifact {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ManoeuvreKind {
    StraightNeutral,
    ThrottleResponse,
    PitchStep,
    RollStep,
    YawStep,
    ControlReversalRecovery,
    PowerOffGlide,
    HighAngleEntry,
}

#[derive(Debug, Clone, Copy)]
struct ManoeuvreSpec {
    kind: ManoeuvreKind,
    id: &'static str,
    description: &'static str,
    steps: u64,
}

const MANOEUVRES: [ManoeuvreSpec; 8] = [
    ManoeuvreSpec {
        kind: ManoeuvreKind::StraightNeutral,
        id: "straight_neutral",
        description: "neutral controls at moderate power",
        steps: 2_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::ThrottleResponse,
        id: "throttle_response",
        description: "low-to-high throttle step followed by recovery",
        steps: 2_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::PitchStep,
        id: "pitch_step",
        description: "positive pitch command step followed by neutral recovery",
        steps: 2_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::RollStep,
        id: "roll_step",
        description: "positive roll command step followed by neutral recovery",
        steps: 2_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::YawStep,
        id: "yaw_step",
        description: "positive yaw command step followed by neutral recovery",
        steps: 2_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::ControlReversalRecovery,
        id: "control_reversal_recovery",
        description: "positive then negative roll command followed by neutral recovery",
        steps: 2_500,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::PowerOffGlide,
        id: "power_off_glide",
        description: "power-off free-flight characterization",
        steps: 3_000,
    },
    ManoeuvreSpec {
        kind: ManoeuvreKind::HighAngleEntry,
        id: "high_angle_entry",
        description: "large pitch command at reduced power; characterization, not stall validation",
        steps: 2_000,
    },
];

#[derive(Debug, Clone, PartialEq)]
struct ManoeuvreMetrics {
    summary: DeterministicTelemetrySummary,
    peak_angular_speed_time_s: f64,
    final_speed_mps: f64,
    local_altitude_change_m: f64,
    specific_kinetic_energy_change_j_per_kg: f64,
}

struct ManoeuvreCapture {
    replay: AircraftReplayRecording,
    telemetry: AircraftTelemetryRecording,
    metrics: ManoeuvreMetrics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ValidationSuiteEvidence {
    pub(crate) suite_version: u32,
    pub(crate) manoeuvre_count: usize,
    pub(crate) replay_verified: bool,
    pub(crate) telemetry_valid: bool,
}

pub(crate) fn validate_model_in_memory(
    model: &AircraftModel,
) -> Result<ValidationSuiteEvidence, ValidationAppError> {
    for spec in MANOEUVRES {
        let capture = capture_manoeuvre(model, spec)?;
        if capture.replay.frames().len() != spec.steps as usize
            || capture.telemetry.frames().len() != spec.steps as usize
        {
            return Err(ValidationAppError::ReplayStepCount {
                expected: spec.steps,
                actual: capture.replay.frames().len() as u64,
            });
        }
        let _ = capture.telemetry.summary()?;
    }
    Ok(ValidationSuiteEvidence {
        suite_version: VALIDATION_SUITE_VERSION,
        manoeuvre_count: MANOEUVRES.len(),
        replay_verified: true,
        telemetry_valid: true,
    })
}

pub fn run_validation(options: ValidationOptions) -> Result<(), ValidationAppError> {
    let model_path = PathBuf::from(MODEL_PATH);
    let model =
        load_aircraft_model(&model_path).map_err(|source| ValidationAppError::ModelLoad {
            path: model_path,
            source,
        })?;
    std::fs::create_dir_all(&options.output_dir).map_err(|source| {
        ValidationAppError::CreateOutputDirectory {
            path: options.output_dir.clone(),
            source,
        }
    })?;

    let mut markdown = validation_header(&model);
    println!("RC Simulation Engine");
    println!("mode: acro-electric-01-validation");
    println!("suite_version: {VALIDATION_SUITE_VERSION}");
    println!("model_id: {}", model.model_id());
    println!("model_physics_fingerprint: {}", fingerprint_hex(&model));
    println!("manoeuvres: {}", MANOEUVRES.len());

    for spec in MANOEUVRES {
        let capture = capture_manoeuvre(&model, spec)?;
        let replay_path = options.output_dir.join(format!("{}.replay.json", spec.id));
        let telemetry_path = options
            .output_dir
            .join(format!("{}.telemetry.jsonl", spec.id));
        write_text(&replay_path, &capture.replay.to_json_pretty()?)?;
        write_text(&telemetry_path, &capture.telemetry.to_json_lines()?)?;
        append_metrics(&mut markdown, spec, &capture.metrics);
        print_metrics(spec, &capture.metrics);
    }

    let summary_path = options.output_dir.join("summary.md");
    write_text(&summary_path, &markdown)?;
    println!("validation: PASS");
    println!("output_dir: {}", options.output_dir.display());
    println!("summary: {}", summary_path.display());
    Ok(())
}

fn capture_manoeuvre(
    model: &AircraftModel,
    spec: ManoeuvreSpec,
) -> Result<ManoeuvreCapture, ValidationAppError> {
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(DEFAULT_PHYSICS_HZ, environment)?;
    let mut simulation = AircraftSimulation::new(model.clone(), config, initial_state())?;
    let capacity = usize::try_from(spec.steps).expect("validation step counts fit usize");
    let mut replay_recorder = AircraftReplayRecorder::with_capacity(&simulation, capacity)?;
    let mut telemetry_recorder = AircraftTelemetryRecorder::with_capacity(&simulation, capacity)?;
    for step_index in 0..spec.steps {
        let input = manoeuvre_input(spec.kind, step_index);
        let snapshot = replay_recorder.record(&mut simulation, step_index, input)?;
        telemetry_recorder.record(&simulation, input, &snapshot, None)?;
    }
    let replay = replay_recorder.finish();
    let telemetry = telemetry_recorder.finish();

    let mut verification_simulation = replay.reconstruct_simulation(model.clone())?;
    let player = AircraftReplayPlayer::new(&replay, &verification_simulation)?;
    let verified = player.verify_all(&mut verification_simulation)?;
    if verified != spec.steps {
        return Err(ValidationAppError::ReplayStepCount {
            expected: spec.steps,
            actual: verified,
        });
    }
    let metrics = manoeuvre_metrics(&telemetry)?;
    Ok(ManoeuvreCapture {
        replay,
        telemetry,
        metrics,
    })
}

fn manoeuvre_input(kind: ManoeuvreKind, step: u64) -> PilotInput {
    match kind {
        ManoeuvreKind::StraightNeutral => PilotInput::new(0.0, 0.0, 0.0, 0.55),
        ManoeuvreKind::ThrottleResponse => {
            let throttle = if step < 500 {
                0.2
            } else if step < 1_500 {
                0.85
            } else {
                0.55
            };
            PilotInput::new(0.0, 0.0, 0.0, throttle)
        }
        ManoeuvreKind::PitchStep => {
            let pitch = if (250..750).contains(&step) {
                0.35
            } else {
                0.0
            };
            PilotInput::new(0.0, pitch, 0.0, 0.55)
        }
        ManoeuvreKind::RollStep => {
            let roll = if (250..750).contains(&step) { 0.4 } else { 0.0 };
            PilotInput::new(roll, 0.0, 0.0, 0.55)
        }
        ManoeuvreKind::YawStep => {
            let yaw = if (250..750).contains(&step) {
                0.35
            } else {
                0.0
            };
            PilotInput::new(0.0, 0.0, yaw, 0.55)
        }
        ManoeuvreKind::ControlReversalRecovery => {
            let roll = if (250..750).contains(&step) {
                0.45
            } else if (750..1_250).contains(&step) {
                -0.45
            } else {
                0.0
            };
            PilotInput::new(roll, 0.0, 0.0, 0.55)
        }
        ManoeuvreKind::PowerOffGlide => PilotInput::new(0.0, 0.0, 0.0, 0.0),
        ManoeuvreKind::HighAngleEntry => {
            let pitch = if (250..1_250).contains(&step) {
                0.75
            } else {
                0.0
            };
            PilotInput::new(0.0, pitch, 0.0, 0.35)
        }
    }
}

fn manoeuvre_metrics(
    telemetry: &AircraftTelemetryRecording,
) -> Result<ManoeuvreMetrics, TelemetryCaptureError> {
    let summary = telemetry.summary()?.deterministic;
    let peak_frame = telemetry
        .frames()
        .iter()
        .max_by(|left, right| {
            angular_speed(left)
                .total_cmp(&angular_speed(right))
                .then_with(|| right.step_index().cmp(&left.step_index()))
        })
        .expect("validation manoeuvres contain at least one frame");
    let final_state = summary
        .final_state
        .expect("validation manoeuvres contain a final state");
    let final_speed_mps = magnitude(final_state.linear_velocity_world_ned_mps);
    let final_local_altitude_m = -final_state.position_world_ned_m[2];
    Ok(ManoeuvreMetrics {
        summary,
        peak_angular_speed_time_s: peak_frame.sim_time_s(),
        final_speed_mps,
        local_altitude_change_m: final_local_altitude_m - INITIAL_LOCAL_ALTITUDE_M,
        specific_kinetic_energy_change_j_per_kg: 0.5
            * (final_speed_mps * final_speed_mps - INITIAL_SPEED_MPS * INITIAL_SPEED_MPS),
    })
}

fn angular_speed(frame: &telemetry::AircraftTelemetryFrame) -> f64 {
    magnitude(frame.angular_velocity_body_frd_radps())
}

fn magnitude(values: [f64; 3]) -> f64 {
    values
        .into_iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -INITIAL_LOCAL_ALTITUDE_M),
        linear_velocity_world_mps: Vec3::new(INITIAL_SPEED_MPS, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn validation_header(model: &AircraftModel) -> String {
    format!(
        "# Acro Electric 01 S10 deterministic characterization\n\n- Suite version: `{VALIDATION_SUITE_VERSION}`\n- Model ID: `{}`\n- Physics fingerprint: `{}`\n- Physics rate: `{DEFAULT_PHYSICS_HZ} Hz`\n- Initial speed: `{INITIAL_SPEED_MPS} m/s`\n- Initial local altitude: `{INITIAL_LOCAL_ALTITUDE_M} m`\n- Classification: `CHARACTERIZATION`; numerical validation targets are undefined.\n\n",
        model.model_id(),
        fingerprint_hex(model)
    )
}

fn append_metrics(markdown: &mut String, spec: ManoeuvreSpec, metrics: &ManoeuvreMetrics) {
    let summary = &metrics.summary;
    let speed = summary
        .speed_mps
        .expect("non-empty capture has speed metrics");
    let altitude = summary
        .local_altitude_m
        .expect("non-empty capture has altitude metrics");
    writeln!(markdown, "## `{}`\n", spec.id).unwrap();
    writeln!(markdown, "{}\n", spec.description).unwrap();
    writeln!(
        markdown,
        "- Duration: `{:.6} s`",
        summary.simulated_duration_s
    )
    .unwrap();
    writeln!(
        markdown,
        "- Speed min/max/mean: `{:.9}` / `{:.9}` / `{:.9} m/s`",
        speed.min, speed.max, speed.mean
    )
    .unwrap();
    writeln!(
        markdown,
        "- Local altitude min/max/change: `{:.9}` / `{:.9}` / `{:.9} m`",
        altitude.min, altitude.max, metrics.local_altitude_change_m
    )
    .unwrap();
    writeln!(
        markdown,
        "- Peak angular speed/time: `{:.9} rad/s` at `{:.6} s`",
        summary.max_angular_speed_radps.unwrap(),
        metrics.peak_angular_speed_time_s
    )
    .unwrap();
    append_range(markdown, "Aileron range", summary.aileron_angle_rad);
    append_range(markdown, "Elevator range", summary.elevator_angle_rad);
    append_range(markdown, "Rudder range", summary.rudder_angle_rad);
    writeln!(
        markdown,
        "- Final speed: `{:.9} m/s`",
        metrics.final_speed_mps
    )
    .unwrap();
    writeln!(
        markdown,
        "- Derived specific kinetic-energy change: `{:.9} J/kg`",
        metrics.specific_kinetic_energy_change_j_per_kg
    )
    .unwrap();
    writeln!(
        markdown,
        "- Final state: `{:?}`\n",
        summary.final_state.unwrap()
    )
    .unwrap();
}

fn append_range(markdown: &mut String, label: &str, range: Option<ScalarRange>) {
    let range = range.expect("non-empty capture has control range");
    writeln!(
        markdown,
        "- {label}: `{:.9}` to `{:.9} rad`",
        range.min, range.max
    )
    .unwrap();
}

fn print_metrics(spec: ManoeuvreSpec, metrics: &ManoeuvreMetrics) {
    let speed = metrics.summary.speed_mps.unwrap();
    println!(
        "manoeuvre: {} steps={} duration_s={:.3} speed_min={:.6} speed_max={:.6} speed_mean={:.6} altitude_delta_m={:.6} peak_angular_speed_radps={:.6} peak_time_s={:.3} final_speed_mps={:.6} specific_ke_delta_j_per_kg={:.6}",
        spec.id,
        metrics.summary.frame_count,
        metrics.summary.simulated_duration_s,
        speed.min,
        speed.max,
        speed.mean,
        metrics.local_altitude_change_m,
        metrics.summary.max_angular_speed_radps.unwrap(),
        metrics.peak_angular_speed_time_s,
        metrics.final_speed_mps,
        metrics.specific_kinetic_energy_change_j_per_kg
    );
}

fn fingerprint_hex(model: &AircraftModel) -> String {
    let mut output = String::with_capacity(64);
    for byte in model.physics_fingerprint().as_bytes() {
        write!(&mut output, "{byte:02x}").unwrap();
    }
    output
}

fn write_text(path: &PathBuf, contents: &str) -> Result<(), ValidationAppError> {
    std::fs::write(path, contents).map_err(|source| ValidationAppError::Write {
        path: path.clone(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    #[test]
    fn suite_v1_has_all_required_unique_manoeuvres_and_valid_inputs() {
        assert_eq!(VALIDATION_SUITE_VERSION, 1);
        assert_eq!(MANOEUVRES.len(), 8);
        let ids: BTreeSet<_> = MANOEUVRES.iter().map(|spec| spec.id).collect();
        assert_eq!(ids.len(), MANOEUVRES.len());
        for spec in MANOEUVRES {
            assert!(spec.steps >= 2_000);
            for step in 0..spec.steps {
                assert!(manoeuvre_input(spec.kind, step).is_valid());
            }
        }
    }

    #[test]
    fn suite_boundaries_apply_documented_steps_without_ambiguity() {
        assert_eq!(
            manoeuvre_input(ManoeuvreKind::ThrottleResponse, 499).throttle(),
            0.2
        );
        assert_eq!(
            manoeuvre_input(ManoeuvreKind::ThrottleResponse, 500).throttle(),
            0.85
        );
        assert_eq!(
            manoeuvre_input(ManoeuvreKind::ThrottleResponse, 1_500).throttle(),
            0.55
        );
        assert_eq!(manoeuvre_input(ManoeuvreKind::PitchStep, 250).pitch(), 0.35);
        assert_eq!(manoeuvre_input(ManoeuvreKind::RollStep, 250).roll(), 0.4);
        assert_eq!(manoeuvre_input(ManoeuvreKind::YawStep, 250).yaw(), 0.35);
        assert_eq!(
            manoeuvre_input(ManoeuvreKind::ControlReversalRecovery, 750).roll(),
            -0.45
        );
    }

    #[test]
    fn manoeuvre_capture_is_repeatable_and_does_not_change_model_fingerprint() {
        let model = load_aircraft_model(
            PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("../../")
                .join(MODEL_PATH),
        )
        .unwrap();
        let fingerprint = model.physics_fingerprint();
        let first = capture_manoeuvre(&model, MANOEUVRES[2]).unwrap();
        let second = capture_manoeuvre(&model, MANOEUVRES[2]).unwrap();
        assert_eq!(first.replay, second.replay);
        assert_eq!(first.telemetry, second.telemetry);
        assert_eq!(first.metrics, second.metrics);
        assert_eq!(model.physics_fingerprint(), fingerprint);
    }
}
