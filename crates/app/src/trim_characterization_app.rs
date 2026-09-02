//! M2.7B deterministic longitudinal trim characterization CLI and reporting.

use aircraft::{
    AircraftSimulationConfig, CharacterizationUnavailableReason, LongitudinalTrimCharacterization,
    LongitudinalTrimCharacterizationData, LongitudinalTrimCharacterizationError,
    LongitudinalTrimCharacterizationPoint, LongitudinalTrimCharacterizationPointOutcome,
    LongitudinalTrimCharacterizationSteps, LongitudinalTrimSweepError,
    LongitudinalTrimSweepRequest, LongitudinalTrimTolerances, LongitudinalTrimVariables,
    PerturbationSide, TrimBounds, characterize_longitudinal_trim_sweep,
    solve_longitudinal_trim_sweep,
};
use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use serde::{Deserialize, Serialize};
use sim_core::{AeroEnvironment, DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ};
use sim_math::Vec3;
use std::{
    fmt::Write as _,
    fs, io,
    path::{Path, PathBuf},
};
use thiserror::Error;

const TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION: u32 = 1;
const GENERATED_BY: &str = "rcsim-app analyze trim-characterization";
const JSON_REPORT_NAME: &str = "trim_characterization.json";
const MARKDOWN_REPORT_NAME: &str = "trim_characterization.md";

#[derive(Debug, Clone, PartialEq)]
pub struct TrimCharacterizationOptions {
    model_path: PathBuf,
    speeds_mps: Vec<f64>,
    alpha_bounds: TrimBounds,
    elevator_bounds: TrimBounds,
    throttle_bounds: TrimBounds,
    initial_guess: LongitudinalTrimVariables,
    tolerances: LongitudinalTrimTolerances,
    maximum_iterations: usize,
    characterization_steps: LongitudinalTrimCharacterizationSteps,
    output_dir: PathBuf,
}

impl TrimCharacterizationOptions {
    pub fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        ParseState::parse_all(arguments)
    }

    fn model_path(&self) -> &Path {
        &self.model_path
    }

    fn output_dir(&self) -> &Path {
        &self.output_dir
    }
}

#[derive(Default)]
struct ParseState {
    model_path: Option<PathBuf>,
    speeds_mps: Vec<f64>,
    alpha_min: Option<f64>,
    alpha_max: Option<f64>,
    elevator_min: Option<f64>,
    elevator_max: Option<f64>,
    throttle_min: Option<f64>,
    throttle_max: Option<f64>,
    initial_alpha: Option<f64>,
    initial_elevator: Option<f64>,
    initial_throttle: Option<f64>,
    force_tolerance: Option<f64>,
    moment_tolerance: Option<f64>,
    maximum_iterations: Option<usize>,
    alpha_step_rad: Option<f64>,
    elevator_step: Option<f64>,
    output_dir: Option<PathBuf>,
}

impl ParseState {
    fn parse_all<I: Iterator<Item = String>>(
        mut arguments: I,
    ) -> Result<TrimCharacterizationOptions, String> {
        let mut state = Self::default();
        while let Some(argument) = arguments.next() {
            state.process(&argument, &mut arguments)?;
        }
        state.finalize()
    }

    fn process<I: Iterator<Item = String>>(
        &mut self,
        argument: &str,
        arguments: &mut I,
    ) -> Result<(), String> {
        let mut next = |flag: &str| {
            arguments
                .next()
                .ok_or_else(|| format!("missing value for {flag}"))
        };
        match argument {
            "--model" => self.model_path = Some(PathBuf::from(next("--model")?)),
            "--speed-mps" => self
                .speeds_mps
                .push(parse_finite("--speed-mps", &next("--speed-mps")?)?),
            "--alpha-min-rad" => {
                self.alpha_min = Some(parse_finite("--alpha-min-rad", &next("--alpha-min-rad")?)?);
            }
            "--alpha-max-rad" => {
                self.alpha_max = Some(parse_finite("--alpha-max-rad", &next("--alpha-max-rad")?)?);
            }
            "--elevator-min" => {
                self.elevator_min = Some(parse_finite("--elevator-min", &next("--elevator-min")?)?);
            }
            "--elevator-max" => {
                self.elevator_max = Some(parse_finite("--elevator-max", &next("--elevator-max")?)?);
            }
            "--throttle-min" => {
                self.throttle_min = Some(parse_finite("--throttle-min", &next("--throttle-min")?)?);
            }
            "--throttle-max" => {
                self.throttle_max = Some(parse_finite("--throttle-max", &next("--throttle-max")?)?);
            }
            "--initial-alpha-rad" => {
                self.initial_alpha = Some(parse_finite(
                    "--initial-alpha-rad",
                    &next("--initial-alpha-rad")?,
                )?);
            }
            "--initial-elevator" => {
                self.initial_elevator = Some(parse_finite(
                    "--initial-elevator",
                    &next("--initial-elevator")?,
                )?);
            }
            "--initial-throttle" => {
                self.initial_throttle = Some(parse_finite(
                    "--initial-throttle",
                    &next("--initial-throttle")?,
                )?);
            }
            "--force-tolerance-n" => {
                self.force_tolerance = Some(parse_finite(
                    "--force-tolerance-n",
                    &next("--force-tolerance-n")?,
                )?);
            }
            "--moment-tolerance-nm" => {
                self.moment_tolerance = Some(parse_finite(
                    "--moment-tolerance-nm",
                    &next("--moment-tolerance-nm")?,
                )?);
            }
            "--max-iterations" => {
                let value: usize = next("--max-iterations")?
                    .parse()
                    .map_err(|_| "invalid value for --max-iterations".to_owned())?;
                if value == 0 {
                    return Err("--max-iterations must be greater than zero".to_owned());
                }
                self.maximum_iterations = Some(value);
            }
            "--alpha-step-rad" => {
                self.alpha_step_rad = Some(parse_finite(
                    "--alpha-step-rad",
                    &next("--alpha-step-rad")?,
                )?);
            }
            "--elevator-step" => {
                self.elevator_step =
                    Some(parse_finite("--elevator-step", &next("--elevator-step")?)?);
            }
            "--output-dir" => self.output_dir = Some(PathBuf::from(next("--output-dir")?)),
            "--help" | "-h" => {
                super::print_usage();
                std::process::exit(0);
            }
            _ => {
                return Err(format!(
                    "unknown trim-characterization argument: {argument}"
                ));
            }
        }
        Ok(())
    }

    fn finalize(self) -> Result<TrimCharacterizationOptions, String> {
        if self.speeds_mps.is_empty() {
            return Err("at least one --speed-mps is required".to_owned());
        }
        let required = |flag: &str, value: Option<f64>| {
            value.ok_or_else(|| format!("missing required {flag}"))
        };
        let model_path = self
            .model_path
            .ok_or_else(|| "missing required --model".to_owned())?;
        let output_dir = self
            .output_dir
            .ok_or_else(|| "missing required --output-dir".to_owned())?;
        let alpha_bounds = TrimBounds::new(
            required("--alpha-min-rad", self.alpha_min)?,
            required("--alpha-max-rad", self.alpha_max)?,
        )
        .map_err(|error| format!("invalid --alpha-* bounds: {error}"))?;
        let elevator_bounds = TrimBounds::new(
            required("--elevator-min", self.elevator_min)?,
            required("--elevator-max", self.elevator_max)?,
        )
        .map_err(|error| format!("invalid --elevator-* bounds: {error}"))?;
        let throttle_bounds = TrimBounds::new(
            required("--throttle-min", self.throttle_min)?,
            required("--throttle-max", self.throttle_max)?,
        )
        .map_err(|error| format!("invalid --throttle-* bounds: {error}"))?;
        let initial_guess = LongitudinalTrimVariables::new(
            required("--initial-alpha-rad", self.initial_alpha)?,
            required("--initial-elevator", self.initial_elevator)?,
            required("--initial-throttle", self.initial_throttle)?,
        )
        .map_err(|error| format!("invalid initial guess: {error}"))?;
        let tolerances = LongitudinalTrimTolerances::new(
            required("--force-tolerance-n", self.force_tolerance)?,
            required("--moment-tolerance-nm", self.moment_tolerance)?,
        )
        .map_err(|error| format!("invalid tolerances: {error}"))?;
        let maximum_iterations = self
            .maximum_iterations
            .ok_or_else(|| "missing required --max-iterations".to_owned())?;
        let characterization_steps = LongitudinalTrimCharacterizationSteps::new(
            required("--alpha-step-rad", self.alpha_step_rad)?,
            required("--elevator-step", self.elevator_step)?,
        )
        .map_err(|error| format!("invalid characterization steps: {error}"))?;

        Ok(TrimCharacterizationOptions {
            model_path,
            speeds_mps: self.speeds_mps,
            alpha_bounds,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            maximum_iterations,
            characterization_steps,
            output_dir,
        })
    }
}

fn parse_finite(flag: &str, raw: &str) -> Result<f64, String> {
    let value: f64 = raw
        .parse()
        .map_err(|_| format!("invalid value for {flag}"))?;
    if !value.is_finite() {
        return Err(format!("non-finite value for {flag}"));
    }
    Ok(value)
}

#[derive(Debug, Error)]
pub enum TrimCharacterizationError {
    #[error("failed to load trim-characterization model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to build or solve the longitudinal trim sweep: {0}")]
    Sweep(#[from] LongitudinalTrimSweepError),
    #[error("failed to characterize the longitudinal trim sweep: {0}")]
    Characterization(#[from] LongitudinalTrimCharacterizationError),
    #[error("failed to serialize trim-characterization report: {0}")]
    SerializeReport(#[source] serde_json::Error),
    #[error("failed to deserialize trim-characterization report: {0}")]
    DeserializeReport(#[source] serde_json::Error),
    #[error("unsupported trim-characterization report schema version {found}; expected {expected}")]
    UnsupportedReportSchemaVersion { found: u32, expected: u32 },
    #[error("failed to create trim-characterization output directory {path}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write trim-characterization artifact {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

pub fn run_trim_characterization(
    options: TrimCharacterizationOptions,
) -> Result<(), TrimCharacterizationError> {
    let model = load_aircraft_model(options.model_path()).map_err(|source| {
        TrimCharacterizationError::ModelLoad {
            path: options.model_path().to_path_buf(),
            source,
        }
    })?;
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())
        .expect("the deterministic standard atmosphere is valid");
    let config = AircraftSimulationConfig::new(
        1.0 / f64::from(DEFAULT_PHYSICS_HZ),
        Vec3::new(0.0, 0.0, DEFAULT_GRAVITY_MPS2),
        environment,
    )
    .expect("the deterministic characterization physics configuration is valid");
    let sweep_request = LongitudinalTrimSweepRequest::new(
        options.speeds_mps.clone(),
        options.alpha_bounds,
        options.elevator_bounds,
        options.throttle_bounds,
        options.initial_guess,
        options.tolerances,
        options.maximum_iterations,
    )?;
    let sweep = solve_longitudinal_trim_sweep(&model, &config, &sweep_request)?;
    let characterization = characterize_longitudinal_trim_sweep(
        &model,
        &config,
        &sweep_request,
        &sweep,
        options.characterization_steps,
    )?;
    let report = build_report(&model, &options, &config, environment, &characterization);
    write_reports(options.output_dir(), &report)
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TrimCharacterizationReport {
    schema_version: u32,
    generated_by: String,
    model: ModelInfo,
    environment: EnvironmentInfo,
    trim_request: TrimRequestInfo,
    characterization_steps: CharacterizationStepsInfo,
    summary: SummaryInfo,
    points: Vec<PointInfo>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct ModelInfo {
    model_id: String,
    model_physics_fingerprint: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct EnvironmentInfo {
    physics_hz: u32,
    air_density_kg_m3: f64,
    wind_velocity_world_mps: [f64; 3],
    gravity_world_mps2: [f64; 3],
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TrimRequestInfo {
    target_speeds_mps: Vec<f64>,
    alpha_bounds_rad: BoundsInfo,
    elevator_bounds: BoundsInfo,
    throttle_bounds: BoundsInfo,
    initial_guess: VariablesInfo,
    tolerances: TolerancesInfo,
    maximum_iterations: usize,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct BoundsInfo {
    min: f64,
    max: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct VariablesInfo {
    alpha_rad: f64,
    elevator_command: f64,
    throttle: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TolerancesInfo {
    force_n: f64,
    pitch_moment_nm: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct CharacterizationStepsInfo {
    alpha_step_rad: f64,
    elevator_step: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
enum PerturbationSideInfo {
    Minus,
    Plus,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "reason", rename_all = "snake_case", deny_unknown_fields)]
enum UnavailableReasonInfo {
    AlphaPerturbationOutOfBounds {
        alpha_minus: f64,
        alpha_plus: f64,
        lower: f64,
        upper: f64,
    },
    ElevatorPerturbationOutOfBounds {
        elevator_minus: f64,
        elevator_plus: f64,
        lower: f64,
        upper: f64,
    },
    AlphaPerturbationNonFinite {
        side: PerturbationSideInfo,
    },
    ElevatorPerturbationNonFinite {
        side: PerturbationSideInfo,
    },
    NonFinitePitchStiffness,
    NonFiniteElevatorEffectiveness,
}

impl UnavailableReasonInfo {
    fn from_domain(reason: CharacterizationUnavailableReason) -> Self {
        match reason {
            CharacterizationUnavailableReason::AlphaPerturbationOutOfBounds {
                alpha_minus,
                alpha_plus,
                lower,
                upper,
            } => Self::AlphaPerturbationOutOfBounds {
                alpha_minus,
                alpha_plus,
                lower,
                upper,
            },
            CharacterizationUnavailableReason::ElevatorPerturbationOutOfBounds {
                elevator_minus,
                elevator_plus,
                lower,
                upper,
            } => Self::ElevatorPerturbationOutOfBounds {
                elevator_minus,
                elevator_plus,
                lower,
                upper,
            },
            CharacterizationUnavailableReason::AlphaPerturbationNonFinite { side } => {
                Self::AlphaPerturbationNonFinite {
                    side: PerturbationSideInfo::from(side),
                }
            }
            CharacterizationUnavailableReason::ElevatorPerturbationNonFinite { side } => {
                Self::ElevatorPerturbationNonFinite {
                    side: PerturbationSideInfo::from(side),
                }
            }
            CharacterizationUnavailableReason::NonFinitePitchStiffness => {
                Self::NonFinitePitchStiffness
            }
            CharacterizationUnavailableReason::NonFiniteElevatorEffectiveness => {
                Self::NonFiniteElevatorEffectiveness
            }
        }
    }
}

impl From<PerturbationSide> for PerturbationSideInfo {
    fn from(value: PerturbationSide) -> Self {
        match value {
            PerturbationSide::Minus => Self::Minus,
            PerturbationSide::Plus => Self::Plus,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "outcome", rename_all = "snake_case", deny_unknown_fields)]
enum PointInfo {
    Characterized(CharacterizedPointInfo),
    NotCharacterizedTrimFailure {
        target_airspeed_mps: f64,
    },
    NotCharacterizedReEvaluationMismatch {
        target_airspeed_mps: f64,
    },
    NotCharacterizedReEvaluationUnverifiable {
        target_airspeed_mps: f64,
    },
    CharacterizationUnavailable {
        target_airspeed_mps: f64,
        unavailable: UnavailableReasonInfo,
    },
}

impl PointInfo {
    fn target_airspeed_mps(&self) -> f64 {
        match self {
            Self::Characterized(data) => data.target_airspeed_mps,
            Self::NotCharacterizedTrimFailure {
                target_airspeed_mps,
            }
            | Self::NotCharacterizedReEvaluationMismatch {
                target_airspeed_mps,
            }
            | Self::NotCharacterizedReEvaluationUnverifiable {
                target_airspeed_mps,
            }
            | Self::CharacterizationUnavailable {
                target_airspeed_mps,
                ..
            } => *target_airspeed_mps,
        }
    }

    fn outcome_label(&self) -> &'static str {
        match self {
            Self::Characterized(_) => "characterized",
            Self::NotCharacterizedTrimFailure { .. } => "not_characterized_trim_failure",
            Self::NotCharacterizedReEvaluationMismatch { .. } => {
                "not_characterized_re_evaluation_mismatch"
            }
            Self::NotCharacterizedReEvaluationUnverifiable { .. } => {
                "not_characterized_re_evaluation_unverifiable"
            }
            Self::CharacterizationUnavailable { .. } => "characterization_unavailable",
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct CharacterizedPointInfo {
    target_airspeed_mps: f64,
    alpha_rad: f64,
    elevator_command: f64,
    throttle: f64,
    alpha_step_rad: f64,
    elevator_step: f64,
    pitch_moment_at_trim_nm: f64,
    alpha_minus_pitch_moment_nm: f64,
    alpha_plus_pitch_moment_nm: f64,
    elevator_minus_pitch_moment_nm: f64,
    elevator_plus_pitch_moment_nm: f64,
    pitch_stiffness_nm_per_rad: f64,
    elevator_effectiveness_nm_per_command: f64,
}

impl From<LongitudinalTrimCharacterizationData> for CharacterizedPointInfo {
    fn from(data: LongitudinalTrimCharacterizationData) -> Self {
        Self {
            target_airspeed_mps: data.target_airspeed_mps,
            alpha_rad: data.alpha_rad,
            elevator_command: data.elevator_command,
            throttle: data.throttle,
            alpha_step_rad: data.alpha_step_rad,
            elevator_step: data.elevator_step,
            pitch_moment_at_trim_nm: data.pitch_moment_at_trim_nm,
            alpha_minus_pitch_moment_nm: data.alpha_minus_pitch_moment_nm,
            alpha_plus_pitch_moment_nm: data.alpha_plus_pitch_moment_nm,
            elevator_minus_pitch_moment_nm: data.elevator_minus_pitch_moment_nm,
            elevator_plus_pitch_moment_nm: data.elevator_plus_pitch_moment_nm,
            pitch_stiffness_nm_per_rad: data.pitch_stiffness_nm_per_rad,
            elevator_effectiveness_nm_per_command: data.elevator_effectiveness_nm_per_command,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
struct SummaryInfo {
    total_points: usize,
    characterized_count: usize,
    trim_failure_not_characterized_count: usize,
    re_evaluation_mismatch_not_characterized_count: usize,
    re_evaluation_unverifiable_not_characterized_count: usize,
    characterization_unavailable_count: usize,
}

fn build_report(
    model: &AircraftModel,
    options: &TrimCharacterizationOptions,
    config: &AircraftSimulationConfig,
    environment: AeroEnvironment,
    characterization: &LongitudinalTrimCharacterization,
) -> TrimCharacterizationReport {
    TrimCharacterizationReport {
        schema_version: TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION,
        generated_by: GENERATED_BY.to_owned(),
        model: ModelInfo {
            model_id: model.model_id().to_owned(),
            model_physics_fingerprint: fingerprint_hex(model),
        },
        environment: EnvironmentInfo {
            physics_hz: DEFAULT_PHYSICS_HZ,
            air_density_kg_m3: environment.air_density_kg_m3(),
            wind_velocity_world_mps: vec3_to_array(environment.wind_velocity_world_mps()),
            gravity_world_mps2: vec3_to_array(config.gravity_world_mps2()),
        },
        trim_request: TrimRequestInfo {
            target_speeds_mps: options.speeds_mps.clone(),
            alpha_bounds_rad: bounds_info(options.alpha_bounds),
            elevator_bounds: bounds_info(options.elevator_bounds),
            throttle_bounds: bounds_info(options.throttle_bounds),
            initial_guess: VariablesInfo {
                alpha_rad: options.initial_guess.alpha_rad,
                elevator_command: options.initial_guess.elevator_command,
                throttle: options.initial_guess.throttle,
            },
            tolerances: TolerancesInfo {
                force_n: options.tolerances.force_n,
                pitch_moment_nm: options.tolerances.pitch_moment_nm,
            },
            maximum_iterations: options.maximum_iterations,
        },
        characterization_steps: CharacterizationStepsInfo {
            alpha_step_rad: options.characterization_steps.alpha_step_rad(),
            elevator_step: options.characterization_steps.elevator_step(),
        },
        summary: SummaryInfo {
            total_points: characterization.len(),
            characterized_count: characterization.characterized_count(),
            trim_failure_not_characterized_count: characterization
                .trim_failure_not_characterized_count(),
            re_evaluation_mismatch_not_characterized_count: characterization
                .re_evaluation_mismatch_not_characterized_count(),
            re_evaluation_unverifiable_not_characterized_count: characterization
                .re_evaluation_unverifiable_not_characterized_count(),
            characterization_unavailable_count: characterization
                .characterization_unavailable_count(),
        },
        points: characterization
            .points()
            .iter()
            .map(point_info_from_domain)
            .collect(),
    }
}

fn point_info_from_domain(point: &LongitudinalTrimCharacterizationPoint) -> PointInfo {
    match point.outcome {
        LongitudinalTrimCharacterizationPointOutcome::Characterized(data) => {
            PointInfo::Characterized(data.into())
        }
        LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedTrimFailure => {
            PointInfo::NotCharacterizedTrimFailure {
                target_airspeed_mps: point.target_airspeed_mps,
            }
        }
        LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationMismatch => {
            PointInfo::NotCharacterizedReEvaluationMismatch {
                target_airspeed_mps: point.target_airspeed_mps,
            }
        }
        LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationUnverifiable => {
            PointInfo::NotCharacterizedReEvaluationUnverifiable {
                target_airspeed_mps: point.target_airspeed_mps,
            }
        }
        LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(reason) => {
            PointInfo::CharacterizationUnavailable {
                target_airspeed_mps: point.target_airspeed_mps,
                unavailable: UnavailableReasonInfo::from_domain(reason),
            }
        }
    }
}

fn bounds_info(bounds: TrimBounds) -> BoundsInfo {
    BoundsInfo {
        min: bounds.lower(),
        max: bounds.upper(),
    }
}

fn vec3_to_array(value: &Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

fn fingerprint_hex(model: &AircraftModel) -> String {
    let mut output = String::with_capacity(64);
    for byte in model.physics_fingerprint().as_bytes() {
        write!(&mut output, "{byte:02x}").unwrap();
    }
    output
}

impl TrimCharacterizationReport {
    fn to_json_pretty(&self) -> Result<String, TrimCharacterizationError> {
        serde_json::to_string_pretty(self).map_err(TrimCharacterizationError::SerializeReport)
    }

    fn from_json(json: &str) -> Result<Self, TrimCharacterizationError> {
        let report: Self =
            serde_json::from_str(json).map_err(TrimCharacterizationError::DeserializeReport)?;
        if report.schema_version != TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION {
            return Err(TrimCharacterizationError::UnsupportedReportSchemaVersion {
                found: report.schema_version,
                expected: TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION,
            });
        }
        Ok(report)
    }

    fn to_markdown(&self) -> String {
        render_markdown(self)
    }
}

fn write_reports(
    output_dir: &Path,
    report: &TrimCharacterizationReport,
) -> Result<(), TrimCharacterizationError> {
    let json = report.to_json_pretty()?;
    TrimCharacterizationReport::from_json(&json)?;
    let markdown = report.to_markdown();

    fs::create_dir_all(output_dir).map_err(|source| {
        TrimCharacterizationError::CreateOutputDirectory {
            path: output_dir.to_path_buf(),
            source,
        }
    })?;
    let json_path = output_dir.join(JSON_REPORT_NAME);
    let markdown_path = output_dir.join(MARKDOWN_REPORT_NAME);
    fs::write(&json_path, json).map_err(|source| TrimCharacterizationError::Write {
        path: json_path,
        source,
    })?;
    fs::write(&markdown_path, markdown).map_err(|source| TrimCharacterizationError::Write {
        path: markdown_path,
        source,
    })
}

fn render_markdown(report: &TrimCharacterizationReport) -> String {
    let mut output = String::new();
    writeln!(output, "# Longitudinal Trim Characterization\n").unwrap();
    writeln!(output, "- Generated by: `{}`", report.generated_by).unwrap();
    writeln!(output, "- Schema version: `{}`", report.schema_version).unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Model\n").unwrap();
    writeln!(output, "- Model ID: `{}`", report.model.model_id).unwrap();
    writeln!(
        output,
        "- Model physics fingerprint: `{}`",
        report.model.model_physics_fingerprint
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Configuration\n").unwrap();
    writeln!(
        output,
        "- Physics rate: `{} Hz`",
        report.environment.physics_hz
    )
    .unwrap();
    writeln!(
        output,
        "- Air density: `{} kg/m^3`",
        format_f64(report.environment.air_density_kg_m3)
    )
    .unwrap();
    writeln!(
        output,
        "- Wind velocity (world, m/s): `({}, {}, {})`",
        format_f64(report.environment.wind_velocity_world_mps[0]),
        format_f64(report.environment.wind_velocity_world_mps[1]),
        format_f64(report.environment.wind_velocity_world_mps[2])
    )
    .unwrap();
    writeln!(
        output,
        "- Gravity (world, m/s^2): `({}, {}, {})`",
        format_f64(report.environment.gravity_world_mps2[0]),
        format_f64(report.environment.gravity_world_mps2[1]),
        format_f64(report.environment.gravity_world_mps2[2])
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Trim request\n").unwrap();
    writeln!(
        output,
        "- Ordered target speeds (m/s): `{}`",
        report
            .trim_request
            .target_speeds_mps
            .iter()
            .map(|value| format_f64(*value))
            .collect::<Vec<_>>()
            .join(", ")
    )
    .unwrap();
    write_bounds(
        &mut output,
        "Alpha bounds (rad)",
        report.trim_request.alpha_bounds_rad,
    );
    write_bounds(
        &mut output,
        "Elevator bounds",
        report.trim_request.elevator_bounds,
    );
    write_bounds(
        &mut output,
        "Throttle bounds",
        report.trim_request.throttle_bounds,
    );
    writeln!(
        output,
        "- Initial guess (alpha_rad, elevator, throttle): `({}, {}, {})`",
        format_f64(report.trim_request.initial_guess.alpha_rad),
        format_f64(report.trim_request.initial_guess.elevator_command),
        format_f64(report.trim_request.initial_guess.throttle)
    )
    .unwrap();
    writeln!(
        output,
        "- Tolerances (force N, pitch moment N*m): `({}, {})`",
        format_f64(report.trim_request.tolerances.force_n),
        format_f64(report.trim_request.tolerances.pitch_moment_nm)
    )
    .unwrap();
    writeln!(
        output,
        "- Maximum iterations: `{}`",
        report.trim_request.maximum_iterations
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Characterization steps\n").unwrap();
    writeln!(
        output,
        "- Alpha step (rad): `{}`",
        format_f64(report.characterization_steps.alpha_step_rad)
    )
    .unwrap();
    writeln!(
        output,
        "- Elevator-command step: `{}`",
        format_f64(report.characterization_steps.elevator_step)
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(
        output,
        "`pitch_stiffness_nm_per_rad = dMy/dAlpha` and `elevator_effectiveness_nm_per_command = dMy/dElevatorCommand`."
    )
    .unwrap();
    writeln!(
        output,
        "These are local dimensional derivatives around each verified trim point; they are not coefficient derivatives, static margin, neutral point, aerodynamic center, complete stability derivatives, or flight validation."
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Summary\n").unwrap();
    writeln!(output, "- Total points: `{}`", report.summary.total_points).unwrap();
    writeln!(
        output,
        "- Characterized: `{}`",
        report.summary.characterized_count
    )
    .unwrap();
    writeln!(
        output,
        "- Not characterized (trim failure): `{}`",
        report.summary.trim_failure_not_characterized_count
    )
    .unwrap();
    writeln!(
        output,
        "- Not characterized (re-evaluation mismatch): `{}`",
        report
            .summary
            .re_evaluation_mismatch_not_characterized_count
    )
    .unwrap();
    writeln!(
        output,
        "- Not characterized (re-evaluation unverifiable): `{}`",
        report
            .summary
            .re_evaluation_unverifiable_not_characterized_count
    )
    .unwrap();
    writeln!(
        output,
        "- Characterization unavailable: `{}`",
        report.summary.characterization_unavailable_count
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Ordered points\n").unwrap();
    writeln!(
        output,
        "| speed_mps | outcome | alpha_rad | elevator | throttle | dMy/dAlpha_Nm_per_rad | dMy/dElevator_Nm_per_command |"
    )
    .unwrap();
    writeln!(output, "| ---: | :--- | ---: | ---: | ---: | ---: | ---: |").unwrap();
    for point in &report.points {
        write_point_row(&mut output, point);
    }
    writeln!(output).unwrap();
    writeln!(output, "## Detailed diagnostics\n").unwrap();
    for point in &report.points {
        write_point_diagnostic(&mut output, point);
    }
    output
}

fn write_bounds(output: &mut String, label: &str, bounds: BoundsInfo) {
    writeln!(
        output,
        "- {label}: `[{}, {}]`",
        format_f64(bounds.min),
        format_f64(bounds.max)
    )
    .unwrap();
}

fn write_point_row(output: &mut String, point: &PointInfo) {
    match point {
        PointInfo::Characterized(data) => {
            writeln!(
                output,
                "| {} | characterized | {} | {} | {} | {} | {} |",
                format_f64(data.target_airspeed_mps),
                format_f64(data.alpha_rad),
                format_f64(data.elevator_command),
                format_f64(data.throttle),
                format_f64(data.pitch_stiffness_nm_per_rad),
                format_f64(data.elevator_effectiveness_nm_per_command)
            )
            .unwrap();
        }
        _ => {
            writeln!(
                output,
                "| {} | {} |  |  |  |  |  |",
                format_f64(point.target_airspeed_mps()),
                point.outcome_label()
            )
            .unwrap();
        }
    }
}

fn write_point_diagnostic(output: &mut String, point: &PointInfo) {
    match point {
        PointInfo::Characterized(data) => {
            writeln!(
                output,
                "- `characterized` at `{}` m/s:",
                format_f64(data.target_airspeed_mps)
            )
            .unwrap();
            writeln!(
                output,
                "  - trim: alpha_rad=`{}`, elevator=`{}`, throttle=`{}`, My_Nm=`{}`",
                format_f64(data.alpha_rad),
                format_f64(data.elevator_command),
                format_f64(data.throttle),
                format_f64(data.pitch_moment_at_trim_nm)
            )
            .unwrap();
            writeln!(
                output,
                "  - alpha perturbation: step_rad=`{}`, My_minus_Nm=`{}`, My_plus_Nm=`{}`",
                format_f64(data.alpha_step_rad),
                format_f64(data.alpha_minus_pitch_moment_nm),
                format_f64(data.alpha_plus_pitch_moment_nm)
            )
            .unwrap();
            writeln!(
                output,
                "  - elevator perturbation: step=`{}`, My_minus_Nm=`{}`, My_plus_Nm=`{}`",
                format_f64(data.elevator_step),
                format_f64(data.elevator_minus_pitch_moment_nm),
                format_f64(data.elevator_plus_pitch_moment_nm)
            )
            .unwrap();
            writeln!(
                output,
                "  - derivatives: dMy/dAlpha_Nm_per_rad=`{}`, dMy/dElevator_Nm_per_command=`{}`",
                format_f64(data.pitch_stiffness_nm_per_rad),
                format_f64(data.elevator_effectiveness_nm_per_command)
            )
            .unwrap();
        }
        PointInfo::CharacterizationUnavailable {
            target_airspeed_mps,
            unavailable,
        } => {
            writeln!(
                output,
                "- `characterization_unavailable` at `{}` m/s:",
                format_f64(*target_airspeed_mps)
            )
            .unwrap();
            write_unavailable_reason(output, unavailable);
        }
        _ => {
            writeln!(
                output,
                "- `{}` at `{}` m/s",
                point.outcome_label(),
                format_f64(point.target_airspeed_mps())
            )
            .unwrap();
        }
    }
}

fn write_unavailable_reason(output: &mut String, reason: &UnavailableReasonInfo) {
    match reason {
        UnavailableReasonInfo::AlphaPerturbationOutOfBounds {
            alpha_minus,
            alpha_plus,
            lower,
            upper,
        } => writeln!(
            output,
            "  - reason=`alpha_perturbation_out_of_bounds`, alpha_minus=`{}`, alpha_plus=`{}`, lower=`{}`, upper=`{}`",
            format_f64(*alpha_minus),
            format_f64(*alpha_plus),
            format_f64(*lower),
            format_f64(*upper)
        ),
        UnavailableReasonInfo::ElevatorPerturbationOutOfBounds {
            elevator_minus,
            elevator_plus,
            lower,
            upper,
        } => writeln!(
            output,
            "  - reason=`elevator_perturbation_out_of_bounds`, elevator_minus=`{}`, elevator_plus=`{}`, lower=`{}`, upper=`{}`",
            format_f64(*elevator_minus),
            format_f64(*elevator_plus),
            format_f64(*lower),
            format_f64(*upper)
        ),
        UnavailableReasonInfo::AlphaPerturbationNonFinite { side } => writeln!(
            output,
            "  - reason=`alpha_perturbation_non_finite`, side=`{}`",
            side_label(*side)
        ),
        UnavailableReasonInfo::ElevatorPerturbationNonFinite { side } => writeln!(
            output,
            "  - reason=`elevator_perturbation_non_finite`, side=`{}`",
            side_label(*side)
        ),
        UnavailableReasonInfo::NonFinitePitchStiffness => {
            writeln!(output, "  - reason=`non_finite_pitch_stiffness`")
        }
        UnavailableReasonInfo::NonFiniteElevatorEffectiveness => {
            writeln!(output, "  - reason=`non_finite_elevator_effectiveness`")
        }
    }
    .unwrap();
}

fn side_label(side: PerturbationSideInfo) -> &'static str {
    match side {
        PerturbationSideInfo::Minus => "minus",
        PerturbationSideInfo::Plus => "plus",
    }
}

fn format_f64(value: f64) -> String {
    format!("{value:.17e}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use model::AircraftClassification;

    const FIXTURE_RELATIVE_PATH: &str = "../../tests/fixtures/synthetic_non_reference_trim_v4.json";

    fn fixture_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(FIXTURE_RELATIVE_PATH)
    }

    fn base_options(speeds_mps: Vec<f64>) -> TrimCharacterizationOptions {
        TrimCharacterizationOptions {
            model_path: fixture_path(),
            speeds_mps,
            alpha_bounds: TrimBounds::new(0.02, 0.20).unwrap(),
            elevator_bounds: TrimBounds::new(-0.5, 0.5).unwrap(),
            throttle_bounds: TrimBounds::new(0.0, 1.0).unwrap(),
            initial_guess: LongitudinalTrimVariables::new(0.05, 0.0, 0.5).unwrap(),
            tolerances: LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
            maximum_iterations: 50,
            characterization_steps: LongitudinalTrimCharacterizationSteps::new(0.001, 0.01)
                .unwrap(),
            output_dir: std::env::temp_dir().join("rcsim_m2_7b_unit_default"),
        }
    }

    fn domain_and_report(
        options: &TrimCharacterizationOptions,
    ) -> (LongitudinalTrimCharacterization, TrimCharacterizationReport) {
        let model = load_aircraft_model(options.model_path()).unwrap();
        let environment = AeroEnvironment::new(1.225, Vec3::zeros()).unwrap();
        let config = AircraftSimulationConfig::new(
            1.0 / f64::from(DEFAULT_PHYSICS_HZ),
            Vec3::new(0.0, 0.0, DEFAULT_GRAVITY_MPS2),
            environment,
        )
        .unwrap();
        let request = LongitudinalTrimSweepRequest::new(
            options.speeds_mps.clone(),
            options.alpha_bounds,
            options.elevator_bounds,
            options.throttle_bounds,
            options.initial_guess,
            options.tolerances,
            options.maximum_iterations,
        )
        .unwrap();
        let sweep = solve_longitudinal_trim_sweep(&model, &config, &request).unwrap();
        let characterization = characterize_longitudinal_trim_sweep(
            &model,
            &config,
            &request,
            &sweep,
            options.characterization_steps,
        )
        .unwrap();
        let report = build_report(&model, options, &config, environment, &characterization);
        (characterization, report)
    }

    fn valid_args() -> Vec<String> {
        [
            "--model",
            "synthetic.json",
            "--speed-mps",
            "21",
            "--speed-mps",
            "18",
            "--alpha-min-rad",
            "0.02",
            "--alpha-max-rad",
            "0.20",
            "--elevator-min",
            "-0.5",
            "--elevator-max",
            "0.5",
            "--throttle-min",
            "0.0",
            "--throttle-max",
            "1.0",
            "--initial-alpha-rad",
            "0.05",
            "--initial-elevator",
            "0.0",
            "--initial-throttle",
            "0.5",
            "--force-tolerance-n",
            "5.0",
            "--moment-tolerance-nm",
            "2.0",
            "--max-iterations",
            "50",
            "--alpha-step-rad",
            "0.001",
            "--elevator-step",
            "0.01",
            "--output-dir",
            "out",
        ]
        .into_iter()
        .map(str::to_owned)
        .collect()
    }

    fn assert_bits_equal(actual: f64, expected: f64, field: &str) {
        assert_eq!(
            actual.to_bits(),
            expected.to_bits(),
            "bit mismatch for {field}"
        );
    }

    fn assert_characterized_mapping(
        report: &CharacterizedPointInfo,
        domain: &LongitudinalTrimCharacterizationData,
    ) {
        for (name, actual, expected) in [
            (
                "target_airspeed_mps",
                report.target_airspeed_mps,
                domain.target_airspeed_mps,
            ),
            ("alpha_rad", report.alpha_rad, domain.alpha_rad),
            (
                "elevator_command",
                report.elevator_command,
                domain.elevator_command,
            ),
            ("throttle", report.throttle, domain.throttle),
            (
                "alpha_step_rad",
                report.alpha_step_rad,
                domain.alpha_step_rad,
            ),
            ("elevator_step", report.elevator_step, domain.elevator_step),
            (
                "pitch_moment_at_trim_nm",
                report.pitch_moment_at_trim_nm,
                domain.pitch_moment_at_trim_nm,
            ),
            (
                "alpha_minus_pitch_moment_nm",
                report.alpha_minus_pitch_moment_nm,
                domain.alpha_minus_pitch_moment_nm,
            ),
            (
                "alpha_plus_pitch_moment_nm",
                report.alpha_plus_pitch_moment_nm,
                domain.alpha_plus_pitch_moment_nm,
            ),
            (
                "elevator_minus_pitch_moment_nm",
                report.elevator_minus_pitch_moment_nm,
                domain.elevator_minus_pitch_moment_nm,
            ),
            (
                "elevator_plus_pitch_moment_nm",
                report.elevator_plus_pitch_moment_nm,
                domain.elevator_plus_pitch_moment_nm,
            ),
            (
                "pitch_stiffness_nm_per_rad",
                report.pitch_stiffness_nm_per_rad,
                domain.pitch_stiffness_nm_per_rad,
            ),
            (
                "elevator_effectiveness_nm_per_command",
                report.elevator_effectiveness_nm_per_command,
                domain.elevator_effectiveness_nm_per_command,
            ),
        ] {
            assert_bits_equal(actual, expected, name);
        }
    }

    fn valid_report_json() -> String {
        domain_and_report(&base_options(vec![15.0, 18.0, 21.0]))
            .1
            .to_json_pretty()
            .unwrap()
    }

    #[test]
    fn parser_preserves_speed_order_and_requires_explicit_steps() {
        let options = TrimCharacterizationOptions::parse(valid_args().into_iter()).unwrap();
        assert_eq!(options.speeds_mps, vec![21.0, 18.0]);
        assert_eq!(options.characterization_steps.alpha_step_rad(), 0.001);
        assert_eq!(options.characterization_steps.elevator_step(), 0.01);

        let mut without_alpha_step = valid_args();
        let index = without_alpha_step
            .iter()
            .position(|argument| argument == "--alpha-step-rad")
            .unwrap();
        without_alpha_step.drain(index..=index + 1);
        assert_eq!(
            TrimCharacterizationOptions::parse(without_alpha_step.into_iter()).unwrap_err(),
            "missing required --alpha-step-rad"
        );

        let mut without_elevator_step = valid_args();
        let index = without_elevator_step
            .iter()
            .position(|argument| argument == "--elevator-step")
            .unwrap();
        without_elevator_step.drain(index..=index + 1);
        assert_eq!(
            TrimCharacterizationOptions::parse(without_elevator_step.into_iter()).unwrap_err(),
            "missing required --elevator-step"
        );
    }

    #[test]
    fn characterized_report_fields_are_bit_exact_copies_of_domain_data() {
        let options = base_options(vec![15.0, 18.0, 21.0]);
        let (domain, report) = domain_and_report(&options);
        assert_eq!(domain.points().len(), report.points.len());
        for (domain_point, report_point) in domain.points().iter().zip(&report.points) {
            match (&domain_point.outcome, report_point) {
                (
                    LongitudinalTrimCharacterizationPointOutcome::Characterized(domain_data),
                    PointInfo::Characterized(report_data),
                ) => assert_characterized_mapping(report_data, domain_data),
                other => panic!("expected corresponding characterized points, got {other:?}"),
            }
        }
    }

    #[test]
    fn runner_writes_the_same_characterized_values_returned_by_the_domain_api() {
        let mut options = base_options(vec![15.0, 18.0, 21.0]);
        let output_dir =
            std::env::temp_dir().join(format!("rcsim_m2_7b_mapping_{}", std::process::id()));
        options.output_dir = output_dir.clone();
        let (domain, _) = domain_and_report(&options);
        run_trim_characterization(options).unwrap();
        let json = fs::read_to_string(output_dir.join(JSON_REPORT_NAME)).unwrap();
        let report = TrimCharacterizationReport::from_json(&json).unwrap();
        for (domain_point, report_point) in domain.points().iter().zip(&report.points) {
            match (&domain_point.outcome, report_point) {
                (
                    LongitudinalTrimCharacterizationPointOutcome::Characterized(domain_data),
                    PointInfo::Characterized(report_data),
                ) => assert_characterized_mapping(report_data, domain_data),
                other => panic!("expected corresponding characterized points, got {other:?}"),
            }
        }
        let _ = fs::remove_dir_all(output_dir);
    }

    #[test]
    fn unavailable_reason_preserves_structured_domain_data() {
        let mut options = base_options(vec![15.0, 18.0, 21.0]);
        options.characterization_steps =
            LongitudinalTrimCharacterizationSteps::new(0.5, 0.01).unwrap();
        let (domain, report) = domain_and_report(&options);
        assert!(report.summary.characterization_unavailable_count > 0);
        for (domain_point, report_point) in domain.points().iter().zip(&report.points) {
            match (&domain_point.outcome, report_point) {
                (
                    LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                        domain_reason,
                    ),
                    PointInfo::CharacterizationUnavailable {
                        target_airspeed_mps,
                        unavailable,
                    },
                ) => {
                    assert_bits_equal(
                        *target_airspeed_mps,
                        domain_point.target_airspeed_mps,
                        "unavailable target airspeed",
                    );
                    assert_eq!(
                        *unavailable,
                        UnavailableReasonInfo::from_domain(*domain_reason)
                    );
                }
                other => panic!("expected corresponding unavailable points, got {other:?}"),
            }
        }
    }

    #[test]
    fn every_non_characterized_outcome_maps_without_fabricated_derivatives() {
        let cases = [
            (
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedTrimFailure,
                "not_characterized_trim_failure",
            ),
            (
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationMismatch,
                "not_characterized_re_evaluation_mismatch",
            ),
            (
                LongitudinalTrimCharacterizationPointOutcome::NotCharacterizedReEvaluationUnverifiable,
                "not_characterized_re_evaluation_unverifiable",
            ),
            (
                LongitudinalTrimCharacterizationPointOutcome::CharacterizationUnavailable(
                    CharacterizationUnavailableReason::AlphaPerturbationNonFinite {
                        side: PerturbationSide::Minus,
                    },
                ),
                "characterization_unavailable",
            ),
        ];
        for (outcome, expected_label) in cases {
            let point = LongitudinalTrimCharacterizationPoint {
                target_airspeed_mps: 18.0,
                outcome,
            };
            let mapped = point_info_from_domain(&point);
            assert_eq!(mapped.outcome_label(), expected_label);
            assert!(!matches!(mapped, PointInfo::Characterized(_)));
        }
    }

    #[test]
    fn report_summary_and_points_preserve_domain_order_and_counts() {
        let options = base_options(vec![21.0, 15.0, 18.0]);
        let (domain, report) = domain_and_report(&options);
        assert_eq!(
            report
                .points
                .iter()
                .map(PointInfo::target_airspeed_mps)
                .collect::<Vec<_>>(),
            vec![21.0, 15.0, 18.0]
        );
        assert_eq!(report.summary.total_points, domain.len());
        assert_eq!(
            report.summary.characterized_count,
            domain.characterized_count()
        );
        assert_eq!(
            report.summary.trim_failure_not_characterized_count,
            domain.trim_failure_not_characterized_count()
        );
        assert_eq!(
            report
                .summary
                .re_evaluation_mismatch_not_characterized_count,
            domain.re_evaluation_mismatch_not_characterized_count()
        );
        assert_eq!(
            report
                .summary
                .re_evaluation_unverifiable_not_characterized_count,
            domain.re_evaluation_unverifiable_not_characterized_count()
        );
        assert_eq!(
            report.summary.characterization_unavailable_count,
            domain.characterization_unavailable_count()
        );
    }

    #[test]
    fn current_report_schema_version_is_accepted() {
        let report = TrimCharacterizationReport::from_json(&valid_report_json()).unwrap();
        assert_eq!(
            report.schema_version,
            TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION
        );
    }

    #[test]
    fn unsupported_report_schema_versions_fail_closed() {
        let json = valid_report_json();
        for found in [0, 2, 999] {
            let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
            value["schema_version"] = serde_json::json!(found);
            match TrimCharacterizationReport::from_json(&value.to_string()) {
                Err(TrimCharacterizationError::UnsupportedReportSchemaVersion {
                    found: actual,
                    expected,
                }) => {
                    assert_eq!(actual, found);
                    assert_eq!(expected, TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION);
                }
                other => panic!("expected unsupported schema version {found}, got {other:?}"),
            }
        }
    }

    #[test]
    fn malformed_report_json_is_rejected() {
        match TrimCharacterizationReport::from_json("{") {
            Err(TrimCharacterizationError::DeserializeReport(_)) => {}
            other => panic!("expected report decode error, got {other:?}"),
        }
    }

    #[test]
    fn unknown_report_root_field_is_rejected() {
        let mut value: serde_json::Value = serde_json::from_str(&valid_report_json()).unwrap();
        value["unknown"] = serde_json::json!(true);
        match TrimCharacterizationReport::from_json(&value.to_string()) {
            Err(TrimCharacterizationError::DeserializeReport(_)) => {}
            other => panic!("expected unknown-field decode error, got {other:?}"),
        }
    }

    #[test]
    fn invalid_report_outcome_enum_is_rejected() {
        let mut value: serde_json::Value = serde_json::from_str(&valid_report_json()).unwrap();
        value["points"][0]["outcome"] = serde_json::json!("bogus");
        match TrimCharacterizationReport::from_json(&value.to_string()) {
            Err(TrimCharacterizationError::DeserializeReport(_)) => {}
            other => panic!("expected invalid-outcome decode error, got {other:?}"),
        }
    }

    #[test]
    fn valid_report_round_trip_is_exact() {
        let (_, report) = domain_and_report(&base_options(vec![15.0, 18.0, 21.0]));
        let decoded =
            TrimCharacterizationReport::from_json(&report.to_json_pretty().unwrap()).unwrap();
        assert_eq!(decoded, report);
    }

    #[test]
    fn serialization_and_markdown_are_deterministic() {
        let options = base_options(vec![15.0, 18.0, 21.0]);
        let first = domain_and_report(&options).1;
        let second = domain_and_report(&options).1;
        assert_eq!(
            first.to_json_pretty().unwrap().as_bytes(),
            second.to_json_pretty().unwrap().as_bytes()
        );
        assert_eq!(
            first.to_markdown().as_bytes(),
            second.to_markdown().as_bytes()
        );
    }

    #[test]
    fn markdown_documents_dimensional_local_derivative_semantics() {
        let markdown = domain_and_report(&base_options(vec![18.0])).1.to_markdown();
        for required in [
            "pitch_stiffness_nm_per_rad = dMy/dAlpha",
            "elevator_effectiveness_nm_per_command = dMy/dElevatorCommand",
            "local dimensional derivatives",
            "not coefficient derivatives",
            "static margin",
            "flight validation",
        ] {
            assert!(markdown.contains(required), "missing `{required}`");
        }
    }

    #[test]
    fn production_report_source_uses_no_runtime_nondeterminism() {
        let source = include_str!("trim_characterization_app.rs");
        let test_module = source.find("#[cfg(test)]").unwrap_or(source.len());
        let production_source = &source[..test_module];
        for forbidden in [
            "SystemTime::now",
            "Instant::now",
            "Utc::now",
            "std::process::id",
            "rand::thread_rng",
            "rand::random",
        ] {
            assert!(
                !production_source.contains(forbidden),
                "production report source must not use `{forbidden}`"
            );
        }
    }

    #[test]
    fn fixture_remains_synthetic_and_unpromoted() {
        let model = load_aircraft_model(fixture_path()).unwrap();
        assert_eq!(
            model.classification(),
            AircraftClassification::SyntheticTest
        );
        assert!(model.reference_aircraft().is_none());
    }
}
