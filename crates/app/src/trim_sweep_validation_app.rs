//! M2.6B — deterministic longitudinal trim sweep CLI & reporting.
use aircraft::{
    AircraftSimulationConfig, LongitudinalTrimFailureReason, LongitudinalTrimResiduals,
    LongitudinalTrimSweep, LongitudinalTrimSweepError, LongitudinalTrimSweepOutcome,
    LongitudinalTrimSweepPoint, LongitudinalTrimSweepRequest, LongitudinalTrimTolerances,
    LongitudinalTrimVariables, ReEvaluationMismatchDetail, ReEvaluationUnverifiableDetail,
    TrimBounds, solve_longitudinal_trim_sweep,
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

pub const TRIM_SWEEP_REPORT_SCHEMA_VERSION: u32 = 1;
const GENERATED_BY: &str = "rcsim-app validate trim-sweep";
#[cfg(test)]
const SYNTHETIC_FIXTURE_RELATIVE_PATH: &str =
    "../../tests/fixtures/synthetic_non_reference_trim_v4.json";
const TRIM_SWEEP_JSON_NAME: &str = "trim_sweep.json";
const TRIM_SWEEP_MARKDOWN_NAME: &str = "trim_sweep.md";
#[cfg(test)]
const NON_FINITE_FORBIDDEN_TOKENS: &[&str] = &[
    "timestamp",
    "wall_clock",
    "wallclock",
    "datetime",
    "date_utc",
    "unix_time",
    "nonce",
    "process_id",
    "random_id",
    "uuid",
    "guid",
];

#[derive(Debug, Clone, PartialEq)]
pub struct TrimSweepValidationOptions {
    pub(crate) model_path: PathBuf,
    pub(crate) speeds_mps: Vec<f64>,
    pub(crate) alpha_bounds: TrimBounds,
    pub(crate) elevator_bounds: TrimBounds,
    pub(crate) throttle_bounds: TrimBounds,
    pub(crate) initial_guess: LongitudinalTrimVariables,
    pub(crate) tolerances: LongitudinalTrimTolerances,
    pub(crate) maximum_iterations: usize,
    pub(crate) output_dir: PathBuf,
}

impl TrimSweepValidationOptions {
    /// Parses the CLI arguments for the `validate trim-sweep` subcommand.
    pub fn parse(arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        ParseState::parse_all(arguments)
    }

    #[must_use]
    pub fn model_path(&self) -> &Path {
        &self.model_path
    }
    #[must_use]
    pub fn speeds_mps(&self) -> &[f64] {
        &self.speeds_mps
    }
    #[must_use]
    pub fn output_dir(&self) -> &Path {
        &self.output_dir
    }
    #[must_use]
    pub(crate) fn alpha_bounds(&self) -> TrimBounds {
        self.alpha_bounds
    }
    #[must_use]
    pub(crate) fn elevator_bounds(&self) -> TrimBounds {
        self.elevator_bounds
    }
    #[must_use]
    pub(crate) fn throttle_bounds(&self) -> TrimBounds {
        self.throttle_bounds
    }
    #[must_use]
    pub(crate) fn initial_guess(&self) -> LongitudinalTrimVariables {
        self.initial_guess
    }
    #[must_use]
    pub(crate) fn tolerances(&self) -> LongitudinalTrimTolerances {
        self.tolerances
    }
    #[must_use]
    pub(crate) fn maximum_iterations(&self) -> usize {
        self.maximum_iterations
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
    max_iterations: Option<usize>,
    output_dir: Option<PathBuf>,
}

impl ParseState {
    fn parse_all<I: Iterator<Item = String>>(
        mut arguments: I,
    ) -> Result<TrimSweepValidationOptions, String> {
        let mut state = ParseState::default();
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
        let mut next = |arg: &str| {
            arguments
                .next()
                .ok_or_else(|| format!("missing value for {arg}"))
        };
        match argument {
            "--model" => {
                self.model_path = Some(PathBuf::from(next("--model")?));
            }
            "--speed-mps" => {
                self.speeds_mps
                    .push(parse_finite("--speed-mps", &next("--speed-mps")?)?);
            }
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
                let raw = next("--max-iterations")?;
                let value: usize = raw
                    .parse()
                    .map_err(|_| "invalid value for --max-iterations".to_owned())?;
                if value == 0 {
                    return Err("--max-iterations must be greater than zero".to_owned());
                }
                self.max_iterations = Some(value);
            }
            "--output-dir" => {
                self.output_dir = Some(PathBuf::from(next("--output-dir")?));
            }
            "--help" | "-h" => {
                super::print_usage();
                std::process::exit(0);
            }
            _ => return Err(format!("unknown trim-sweep argument: {argument}")),
        }
        Ok(())
    }

    fn finalize(self) -> Result<TrimSweepValidationOptions, String> {
        if self.speeds_mps.is_empty() {
            return Err("at least one --speed-mps is required".to_owned());
        }
        let required = |flag: &str, value: Option<f64>| -> Result<f64, String> {
            value.ok_or_else(|| format!("missing required {flag}"))
        };
        let model_path = self
            .model_path
            .ok_or_else(|| "missing required --model".to_owned())?;
        let output_dir = self
            .output_dir
            .ok_or_else(|| "missing required --output-dir".to_owned())?;
        let alpha_min = required("--alpha-min-rad", self.alpha_min)?;
        let alpha_max = required("--alpha-max-rad", self.alpha_max)?;
        let elevator_min = required("--elevator-min", self.elevator_min)?;
        let elevator_max = required("--elevator-max", self.elevator_max)?;
        let throttle_min = required("--throttle-min", self.throttle_min)?;
        let throttle_max = required("--throttle-max", self.throttle_max)?;
        let initial_alpha = required("--initial-alpha-rad", self.initial_alpha)?;
        let initial_elevator = required("--initial-elevator", self.initial_elevator)?;
        let initial_throttle = required("--initial-throttle", self.initial_throttle)?;
        let force_tolerance = required("--force-tolerance-n", self.force_tolerance)?;
        let moment_tolerance = required("--moment-tolerance-nm", self.moment_tolerance)?;
        let max_iterations = self
            .max_iterations
            .ok_or_else(|| "missing required --max-iterations".to_owned())?;
        let alpha_bounds = TrimBounds::new(alpha_min, alpha_max)
            .map_err(|error| format!("invalid --alpha-* bounds: {error}"))?;
        let elevator_bounds = TrimBounds::new(elevator_min, elevator_max)
            .map_err(|error| format!("invalid --elevator-* bounds: {error}"))?;
        let throttle_bounds = TrimBounds::new(throttle_min, throttle_max)
            .map_err(|error| format!("invalid --throttle-* bounds: {error}"))?;
        let initial_guess =
            LongitudinalTrimVariables::new(initial_alpha, initial_elevator, initial_throttle)
                .map_err(|error| format!("invalid initial guess: {error}"))?;
        let tolerances = LongitudinalTrimTolerances::new(force_tolerance, moment_tolerance)
            .map_err(|error| format!("invalid tolerances: {error}"))?;
        Ok(TrimSweepValidationOptions {
            model_path,
            speeds_mps: self.speeds_mps,
            alpha_bounds,
            elevator_bounds,
            throttle_bounds,
            initial_guess,
            tolerances,
            maximum_iterations: max_iterations,
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
pub enum TrimSweepValidationError {
    #[error("failed to load trim-sweep validation model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to build the M2.6A longitudinal trim sweep request: {0}")]
    SweepRequest(#[from] LongitudinalTrimSweepError),
    #[error("M2.6A longitudinal trim sweep returned no points; this is a bug")]
    EmptySweep,
    #[error("failed to create trim-sweep output directory {path}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write trim-sweep artifact {path}: {source}")]
    Write {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to serialize trim-sweep report: {0}")]
    SerializeReport(#[source] serde_json::Error),
    #[error("failed to deserialize trim-sweep report: {0}")]
    DeserializeReport(#[source] serde_json::Error),
    #[error("unsupported trim-sweep report schema version {found}; expected {expected}")]
    UnsupportedReportSchemaVersion { found: u32, expected: u32 },
    /// Dedicated non-PASS outcome. Reports are written before this variant is constructed.
    #[error(
        "trim-sweep validation completed with FAIL: {non_success_points} of {total_points} point(s) are not Success"
    )]
    ValidationFailure {
        total_points: usize,
        non_success_points: usize,
    },
}

pub fn run_trim_sweep_validation(
    options: TrimSweepValidationOptions,
) -> Result<(), TrimSweepValidationError> {
    let model = load_aircraft_model(options.model_path()).map_err(|source| {
        TrimSweepValidationError::ModelLoad {
            path: options.model_path().to_path_buf(),
            source,
        }
    })?;
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())
        .expect("the standard atmosphere with zero wind is valid");
    let simulation_config = AircraftSimulationConfig::new(
        1.0 / f64::from(DEFAULT_PHYSICS_HZ),
        Vec3::new(0.0, 0.0, DEFAULT_GRAVITY_MPS2),
        environment,
    )
    .expect("the standard physics configuration is valid");
    let sweep_request = LongitudinalTrimSweepRequest::new(
        options.speeds_mps().to_vec(),
        options.alpha_bounds(),
        options.elevator_bounds(),
        options.throttle_bounds(),
        options.initial_guess(),
        options.tolerances(),
        options.maximum_iterations(),
    )?;
    let sweep = solve_longitudinal_trim_sweep(&model, &simulation_config, &sweep_request)
        .map_err(TrimSweepValidationError::SweepRequest)?;
    if sweep.is_empty() {
        return Err(TrimSweepValidationError::EmptySweep);
    }
    let report = build_report(&model, &options, &sweep, &simulation_config, environment);
    write_reports(options.output_dir(), &report)?;
    if report.summary.overall_status == OverallStatus::Pass {
        Ok(())
    } else {
        let non_success_points = report.summary.total_points - report.summary.success_count;
        Err(TrimSweepValidationError::ValidationFailure {
            total_points: report.summary.total_points,
            non_success_points,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum OverallStatus {
    Pass,
    Fail,
}

impl OverallStatus {
    const fn label(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TrimSweepReport {
    schema_version: u32,
    generated_by: String,
    model: ModelInfo,
    environment: EnvironmentInfo,
    request: RequestInfo,
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
struct RequestInfo {
    target_speeds_mps: Vec<f64>,
    alpha_bounds_rad: BoundsDto,
    elevator_bounds: BoundsDto,
    throttle_bounds: BoundsDto,
    initial_guess: VariablesDto,
    tolerances: TolerancesDto,
    maximum_iterations: usize,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct BoundsDto {
    min: f64,
    max: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct VariablesDto {
    alpha_rad: f64,
    elevator_command: f64,
    throttle: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TolerancesDto {
    force_n: f64,
    pitch_moment_nm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "outcome", deny_unknown_fields)]
enum PointInfo {
    #[serde(rename = "success")]
    Success(SuccessPoint),
    #[serde(rename = "trim_failure")]
    TrimFailure(TrimFailurePoint),
    #[serde(rename = "re_evaluation_mismatch")]
    ReEvaluationMismatch(ReEvaluationMismatchPoint),
    #[serde(rename = "re_evaluation_unverifiable")]
    ReEvaluationUnverifiable(ReEvaluationUnverifiablePoint),
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct SuccessPoint {
    target_airspeed_mps: f64,
    iteration_count: usize,
    alpha_rad: f64,
    elevator_command: f64,
    throttle: f64,
    longitudinal_force_residual_n: f64,
    vertical_force_residual_n: f64,
    pitch_moment_residual_nm: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct TrimFailurePoint {
    target_airspeed_mps: f64,
    failure_reason: FailureReasonDto,
    iteration_count: usize,
    last_evaluation: Option<ResidualsDto>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum FailureReasonDto {
    NoFeasibleSolution,
    SingularJacobian,
    NonFiniteEvaluation,
    IterationLimit,
}

impl FailureReasonDto {
    const fn label(self) -> &'static str {
        match self {
            Self::NoFeasibleSolution => "NO_FEASIBLE_SOLUTION",
            Self::SingularJacobian => "SINGULAR_JACOBIAN",
            Self::NonFiniteEvaluation => "NON_FINITE_EVALUATION",
            Self::IterationLimit => "ITERATION_LIMIT",
        }
    }
    fn from_domain(reason: LongitudinalTrimFailureReason) -> Self {
        match reason {
            LongitudinalTrimFailureReason::NoFeasibleSolution => Self::NoFeasibleSolution,
            LongitudinalTrimFailureReason::SingularJacobian => Self::SingularJacobian,
            LongitudinalTrimFailureReason::NonFiniteEvaluation => Self::NonFiniteEvaluation,
            LongitudinalTrimFailureReason::IterationLimit => Self::IterationLimit,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct ReEvaluationMismatchPoint {
    target_airspeed_mps: f64,
    iteration_count: usize,
    solver_variables: VariablesDto,
    solver_residuals: ResidualsDto,
    independent_variables: VariablesDto,
    independent_residuals: ResidualsDto,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct ReEvaluationUnverifiablePoint {
    target_airspeed_mps: f64,
    iteration_count: usize,
    solver_variables: VariablesDto,
    solver_residuals: ResidualsDto,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct ResidualsDto {
    longitudinal_force_n: f64,
    vertical_force_n: f64,
    pitch_moment_nm: f64,
}

impl From<LongitudinalTrimResiduals> for ResidualsDto {
    fn from(value: LongitudinalTrimResiduals) -> Self {
        Self {
            longitudinal_force_n: value.longitudinal_force_n,
            vertical_force_n: value.vertical_force_n,
            pitch_moment_nm: value.pitch_moment_nm,
        }
    }
}

impl From<LongitudinalTrimVariables> for VariablesDto {
    fn from(value: LongitudinalTrimVariables) -> Self {
        Self {
            alpha_rad: value.alpha_rad,
            elevator_command: value.elevator_command,
            throttle: value.throttle,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct SummaryInfo {
    total_points: usize,
    success_count: usize,
    trim_failure_count: usize,
    re_evaluation_mismatch_count: usize,
    re_evaluation_unverifiable_count: usize,
    overall_status: OverallStatus,
}

fn build_report(
    model: &AircraftModel,
    options: &TrimSweepValidationOptions,
    sweep: &LongitudinalTrimSweep,
    config: &AircraftSimulationConfig,
    environment: AeroEnvironment,
) -> TrimSweepReport {
    let points: Vec<PointInfo> = sweep.points().iter().map(point_info_from).collect();
    let mut success_count = 0;
    let mut trim_failure_count = 0;
    let mut re_evaluation_mismatch_count = 0;
    let mut re_evaluation_unverifiable_count = 0;
    for point in &points {
        match point {
            PointInfo::Success(_) => success_count += 1,
            PointInfo::TrimFailure(_) => trim_failure_count += 1,
            PointInfo::ReEvaluationMismatch(_) => re_evaluation_mismatch_count += 1,
            PointInfo::ReEvaluationUnverifiable(_) => re_evaluation_unverifiable_count += 1,
        }
    }
    let total_points = points.len();
    let overall_status = if success_count == total_points {
        OverallStatus::Pass
    } else {
        OverallStatus::Fail
    };
    TrimSweepReport {
        schema_version: TRIM_SWEEP_REPORT_SCHEMA_VERSION,
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
        request: RequestInfo {
            target_speeds_mps: options.speeds_mps.clone(),
            alpha_bounds_rad: BoundsDto {
                min: options.alpha_bounds.lower(),
                max: options.alpha_bounds.upper(),
            },
            elevator_bounds: BoundsDto {
                min: options.elevator_bounds.lower(),
                max: options.elevator_bounds.upper(),
            },
            throttle_bounds: BoundsDto {
                min: options.throttle_bounds.lower(),
                max: options.throttle_bounds.upper(),
            },
            initial_guess: options.initial_guess.into(),
            tolerances: TolerancesDto {
                force_n: options.tolerances.force_n,
                pitch_moment_nm: options.tolerances.pitch_moment_nm,
            },
            maximum_iterations: options.maximum_iterations,
        },
        summary: SummaryInfo {
            total_points,
            success_count,
            trim_failure_count,
            re_evaluation_mismatch_count,
            re_evaluation_unverifiable_count,
            overall_status,
        },
        points,
    }
}

fn point_info_from(point: &LongitudinalTrimSweepPoint) -> PointInfo {
    match &point.outcome {
        LongitudinalTrimSweepOutcome::Success { solution } => PointInfo::Success(SuccessPoint {
            target_airspeed_mps: point.target_airspeed_mps,
            iteration_count: solution.iteration_count,
            alpha_rad: solution.evaluation.variables.alpha_rad,
            elevator_command: solution.evaluation.variables.elevator_command,
            throttle: solution.evaluation.variables.throttle,
            longitudinal_force_residual_n: solution.evaluation.residuals.longitudinal_force_n,
            vertical_force_residual_n: solution.evaluation.residuals.vertical_force_n,
            pitch_moment_residual_nm: solution.evaluation.residuals.pitch_moment_nm,
        }),
        LongitudinalTrimSweepOutcome::TrimFailure { failure } => {
            PointInfo::TrimFailure(TrimFailurePoint {
                target_airspeed_mps: point.target_airspeed_mps,
                failure_reason: FailureReasonDto::from_domain(failure.reason),
                iteration_count: failure.iteration_count,
                last_evaluation: failure
                    .last_evaluation
                    .as_deref()
                    .map(|evaluation| evaluation.residuals.into()),
            })
        }
        LongitudinalTrimSweepOutcome::ReEvaluationMismatch(detail) => {
            mismatch_point_from(point.target_airspeed_mps, detail)
        }
        LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(detail) => {
            unverifiable_point_from(point.target_airspeed_mps, detail)
        }
    }
}

fn mismatch_point_from(speed: f64, detail: &ReEvaluationMismatchDetail) -> PointInfo {
    let solver = detail.solver_evaluation();
    let independent = detail.independent_evaluation();
    PointInfo::ReEvaluationMismatch(ReEvaluationMismatchPoint {
        target_airspeed_mps: speed,
        iteration_count: detail.iteration_count(),
        solver_variables: solver.variables.into(),
        solver_residuals: solver.residuals.into(),
        independent_variables: independent.variables.into(),
        independent_residuals: independent.residuals.into(),
    })
}

fn unverifiable_point_from(speed: f64, detail: &ReEvaluationUnverifiableDetail) -> PointInfo {
    let solver = detail.solver_evaluation();
    PointInfo::ReEvaluationUnverifiable(ReEvaluationUnverifiablePoint {
        target_airspeed_mps: speed,
        iteration_count: detail.iteration_count(),
        solver_variables: solver.variables.into(),
        solver_residuals: solver.residuals.into(),
    })
}

fn vec3_to_array(v: &Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn fingerprint_hex(model: &AircraftModel) -> String {
    let mut output = String::with_capacity(64);
    for byte in model.physics_fingerprint().as_bytes() {
        write!(&mut output, "{byte:02x}").unwrap();
    }
    output
}

impl TrimSweepReport {
    fn to_json_pretty(&self) -> Result<String, TrimSweepValidationError> {
        serde_json::to_string_pretty(self).map_err(TrimSweepValidationError::SerializeReport)
    }

    fn from_json(json: &str) -> Result<Self, TrimSweepValidationError> {
        let report: Self =
            serde_json::from_str(json).map_err(TrimSweepValidationError::DeserializeReport)?;
        if report.schema_version != TRIM_SWEEP_REPORT_SCHEMA_VERSION {
            return Err(TrimSweepValidationError::UnsupportedReportSchemaVersion {
                found: report.schema_version,
                expected: TRIM_SWEEP_REPORT_SCHEMA_VERSION,
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
    report: &TrimSweepReport,
) -> Result<(), TrimSweepValidationError> {
    let json = report.to_json_pretty()?;
    TrimSweepReport::from_json(&json)?;
    let markdown = report.to_markdown();

    fs::create_dir_all(output_dir).map_err(|source| {
        TrimSweepValidationError::CreateOutputDirectory {
            path: output_dir.to_path_buf(),
            source,
        }
    })?;
    let json_path = output_dir.join(TRIM_SWEEP_JSON_NAME);
    let markdown_path = output_dir.join(TRIM_SWEEP_MARKDOWN_NAME);
    fs::write(&json_path, json).map_err(|source| TrimSweepValidationError::Write {
        path: json_path,
        source,
    })?;
    fs::write(&markdown_path, markdown).map_err(|source| TrimSweepValidationError::Write {
        path: markdown_path,
        source,
    })
}

fn render_markdown(report: &TrimSweepReport) -> String {
    let mut output = String::new();
    writeln!(output, "# Longitudinal Trim Sweep Validation\n").unwrap();
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
        report.environment.air_density_kg_m3
    )
    .unwrap();
    writeln!(
        output,
        "- Wind velocity (world, m/s): `({}, {}, {})`",
        report.environment.wind_velocity_world_mps[0],
        report.environment.wind_velocity_world_mps[1],
        report.environment.wind_velocity_world_mps[2]
    )
    .unwrap();
    writeln!(
        output,
        "- Gravity (world, m/s^2): `({}, {}, {})`",
        report.environment.gravity_world_mps2[0],
        report.environment.gravity_world_mps2[1],
        report.environment.gravity_world_mps2[2]
    )
    .unwrap();
    writeln!(
        output,
        "- Alpha bounds (rad): `[{}, {}]`",
        report.request.alpha_bounds_rad.min, report.request.alpha_bounds_rad.max
    )
    .unwrap();
    writeln!(
        output,
        "- Elevator bounds: `[{}, {}]`",
        report.request.elevator_bounds.min, report.request.elevator_bounds.max
    )
    .unwrap();
    writeln!(
        output,
        "- Throttle bounds: `[{}, {}]`",
        report.request.throttle_bounds.min, report.request.throttle_bounds.max
    )
    .unwrap();
    writeln!(
        output,
        "- Initial guess (alpha_rad, elevator, throttle): `({}, {}, {})`",
        report.request.initial_guess.alpha_rad,
        report.request.initial_guess.elevator_command,
        report.request.initial_guess.throttle
    )
    .unwrap();
    writeln!(
        output,
        "- Tolerances (force N, pitch moment N·m): `({}, {})`",
        report.request.tolerances.force_n, report.request.tolerances.pitch_moment_nm
    )
    .unwrap();
    writeln!(
        output,
        "- Max iterations: `{}`",
        report.request.maximum_iterations
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Summary\n").unwrap();
    writeln!(output, "- Total points: `{}`", report.summary.total_points).unwrap();
    writeln!(output, "- Success: `{}`", report.summary.success_count).unwrap();
    writeln!(
        output,
        "- Trim failure: `{}`",
        report.summary.trim_failure_count
    )
    .unwrap();
    writeln!(
        output,
        "- Re-evaluation mismatch: `{}`",
        report.summary.re_evaluation_mismatch_count
    )
    .unwrap();
    writeln!(
        output,
        "- Re-evaluation unverifiable: `{}`",
        report.summary.re_evaluation_unverifiable_count
    )
    .unwrap();
    writeln!(
        output,
        "- Overall status: `{}`",
        report.summary.overall_status.label()
    )
    .unwrap();
    writeln!(output).unwrap();
    writeln!(output, "## Ordered points\n").unwrap();
    writeln!(output, "| speed_mps | outcome | iterations | alpha_rad | elevator | throttle | Fx_N | Fz_N | My_Nm |").unwrap();
    writeln!(
        output,
        "| ---: | :--- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"
    )
    .unwrap();
    for point in &report.points {
        write_point_row(&mut output, point);
    }
    for point in &report.points {
        if !matches!(point, PointInfo::Success(_)) {
            writeln!(output).unwrap();
            write_failure_diagnostic(&mut output, point);
        }
    }
    output
}

fn write_point_row(output: &mut String, point: &PointInfo) {
    match point {
        PointInfo::Success(s) => {
            writeln!(
                output,
                "| {} | success | {} | {} | {} | {} | {} | {} | {} |",
                format_f64(s.target_airspeed_mps),
                s.iteration_count,
                format_f64(s.alpha_rad),
                format_f64(s.elevator_command),
                format_f64(s.throttle),
                format_f64(s.longitudinal_force_residual_n),
                format_f64(s.vertical_force_residual_n),
                format_f64(s.pitch_moment_residual_nm),
            )
            .unwrap();
        }
        PointInfo::TrimFailure(t) => {
            writeln!(
                output,
                "| {} | trim_failure | {} |  |  |  |  |  |  |",
                format_f64(t.target_airspeed_mps),
                t.iteration_count
            )
            .unwrap();
        }
        PointInfo::ReEvaluationMismatch(m) => {
            writeln!(
                output,
                "| {} | re_evaluation_mismatch | {} |  |  |  |  |  |  |",
                format_f64(m.target_airspeed_mps),
                m.iteration_count
            )
            .unwrap();
        }
        PointInfo::ReEvaluationUnverifiable(u) => {
            writeln!(
                output,
                "| {} | re_evaluation_unverifiable | {} |  |  |  |  |  |  |",
                format_f64(u.target_airspeed_mps),
                u.iteration_count
            )
            .unwrap();
        }
    }
}

fn write_failure_diagnostic(output: &mut String, point: &PointInfo) {
    match point {
        PointInfo::Success(_) => {}
        PointInfo::TrimFailure(t) => {
            writeln!(
                output,
                "- `trim_failure` at `{}` mps: reason=`{}`, iteration_count=`{}`",
                format_f64(t.target_airspeed_mps),
                t.failure_reason.label(),
                t.iteration_count
            )
            .unwrap();
            if let Some(residuals) = t.last_evaluation {
                writeln!(
                    output,
                    "  - last finite residuals: Fx_N=`{}`, Fz_N=`{}`, My_Nm=`{}`",
                    format_f64(residuals.longitudinal_force_n),
                    format_f64(residuals.vertical_force_n),
                    format_f64(residuals.pitch_moment_nm)
                )
                .unwrap();
            } else {
                writeln!(output, "  - no finite last evaluation").unwrap();
            }
        }
        PointInfo::ReEvaluationMismatch(m) => {
            writeln!(
                output,
                "- `re_evaluation_mismatch` at `{}` mps, iteration_count=`{}`",
                format_f64(m.target_airspeed_mps),
                m.iteration_count
            )
            .unwrap();
            writeln!(
                output,
                "  - solver variables: alpha_rad=`{}`, elevator=`{}`, throttle=`{}`",
                format_f64(m.solver_variables.alpha_rad),
                format_f64(m.solver_variables.elevator_command),
                format_f64(m.solver_variables.throttle)
            )
            .unwrap();
            writeln!(
                output,
                "  - solver residuals: Fx_N=`{}`, Fz_N=`{}`, My_Nm=`{}`",
                format_f64(m.solver_residuals.longitudinal_force_n),
                format_f64(m.solver_residuals.vertical_force_n),
                format_f64(m.solver_residuals.pitch_moment_nm)
            )
            .unwrap();
            writeln!(
                output,
                "  - independent variables: alpha_rad=`{}`, elevator=`{}`, throttle=`{}`",
                format_f64(m.independent_variables.alpha_rad),
                format_f64(m.independent_variables.elevator_command),
                format_f64(m.independent_variables.throttle)
            )
            .unwrap();
            writeln!(
                output,
                "  - independent residuals: Fx_N=`{}`, Fz_N=`{}`, My_Nm=`{}`",
                format_f64(m.independent_residuals.longitudinal_force_n),
                format_f64(m.independent_residuals.vertical_force_n),
                format_f64(m.independent_residuals.pitch_moment_nm)
            )
            .unwrap();
        }
        PointInfo::ReEvaluationUnverifiable(u) => {
            writeln!(
                output,
                "- `re_evaluation_unverifiable` at `{}` mps, iteration_count=`{}`",
                format_f64(u.target_airspeed_mps),
                u.iteration_count
            )
            .unwrap();
            writeln!(
                output,
                "  - solver variables: alpha_rad=`{}`, elevator=`{}`, throttle=`{}`",
                format_f64(u.solver_variables.alpha_rad),
                format_f64(u.solver_variables.elevator_command),
                format_f64(u.solver_variables.throttle)
            )
            .unwrap();
            writeln!(
                output,
                "  - solver residuals: Fx_N=`{}`, Fz_N=`{}`, My_Nm=`{}`",
                format_f64(u.solver_residuals.longitudinal_force_n),
                format_f64(u.solver_residuals.vertical_force_n),
                format_f64(u.solver_residuals.pitch_moment_nm)
            )
            .unwrap();
        }
    }
}

fn format_f64(value: f64) -> String {
    format!("{value:.17e}")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn synthetic_fixture_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(SYNTHETIC_FIXTURE_RELATIVE_PATH)
    }

    fn base_template(speeds: Vec<f64>) -> TrimSweepValidationOptions {
        TrimSweepValidationOptions {
            model_path: synthetic_fixture_path(),
            speeds_mps: speeds,
            alpha_bounds: TrimBounds::new(-0.15, 0.30).unwrap(),
            elevator_bounds: TrimBounds::new(-0.9, 0.9).unwrap(),
            throttle_bounds: TrimBounds::new(0.02, 1.0).unwrap(),
            initial_guess: LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
            tolerances: LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
            maximum_iterations: 40,
            output_dir: std::env::temp_dir().join("rcsim_trim_sweep_m2_6b_test_default"),
        }
    }

    fn parse_with_model_and_output(
        model: &str,
        output: &str,
        extra: &[&str],
    ) -> Result<TrimSweepValidationOptions, String> {
        let mut all: Vec<String> = vec![
            "--model".to_owned(),
            model.to_owned(),
            "--output-dir".to_owned(),
            output.to_owned(),
        ];
        for arg in extra {
            all.push((*arg).to_owned());
        }
        TrimSweepValidationOptions::parse(all.into_iter())
    }

    fn point_speed(point: &PointInfo) -> f64 {
        match point {
            PointInfo::Success(p) => p.target_airspeed_mps,
            PointInfo::TrimFailure(p) => p.target_airspeed_mps,
            PointInfo::ReEvaluationMismatch(p) => p.target_airspeed_mps,
            PointInfo::ReEvaluationUnverifiable(p) => p.target_airspeed_mps,
        }
    }

    fn run_synthetic_sweep(
        options: &TrimSweepValidationOptions,
    ) -> (TrimSweepReport, AircraftSimulationConfig, AeroEnvironment) {
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
        let report = build_report(&model, options, &sweep, &config, environment);
        (report, config, environment)
    }

    fn synthetic_report_json() -> String {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        run_synthetic_sweep(&options).0.to_json_pretty().unwrap()
    }

    // ---- 1. Repeated --speed-mps preserves order exactly ----

    #[test]
    fn repeated_speed_mps_preserves_order_exactly() {
        let parsed = parse_with_model_and_output(
            "x.model.json",
            "out",
            &[
                "--speed-mps",
                "21",
                "--speed-mps",
                "18",
                "--speed-mps",
                "15",
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
                "--max-iterations",
                "20",
            ],
        )
        .unwrap();
        assert_eq!(parsed.speeds_mps(), &[21.0, 18.0, 15.0]);
    }

    // ---- 2. No speed fails closed ----

    #[test]
    fn no_speed_fails_closed() {
        let err = parse_with_model_and_output(
            "x.model.json",
            "out",
            &[
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
                "--max-iterations",
                "20",
            ],
        )
        .unwrap_err();
        assert!(err.contains("--speed-mps"));
    }

    // ---- 3. Missing required args fail ----

    #[test]
    fn missing_required_args_fail() {
        let err = TrimSweepValidationOptions::parse(std::iter::empty()).unwrap_err();
        assert!(!err.is_empty());
        let err_iters = parse_with_model_and_output(
            "x",
            "y",
            &[
                "--speed-mps",
                "15",
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
            ],
        )
        .unwrap_err();
        assert!(err_iters.contains("--max-iterations"));
    }

    // ---- 4. Malformed numeric value fails ----

    #[test]
    fn malformed_numeric_value_fails() {
        let err = parse_with_model_and_output(
            "x",
            "y",
            &[
                "--speed-mps",
                "abc",
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
                "--max-iterations",
                "20",
            ],
        )
        .unwrap_err();
        assert!(err.contains("--speed-mps"));
    }

    // ---- 5. Non-finite numeric value fails ----

    #[test]
    fn non_finite_numeric_value_fails() {
        let err = parse_with_model_and_output(
            "x",
            "y",
            &[
                "--speed-mps",
                "nan",
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
                "--max-iterations",
                "20",
            ],
        )
        .unwrap_err();
        assert!(err.contains("non-finite"));
    }

    // ---- 6. Unknown argument fails ----

    #[test]
    fn unknown_argument_fails() {
        let err = parse_with_model_and_output(
            "x",
            "y",
            &[
                "--banana",
                "1",
                "--speed-mps",
                "15",
                "--alpha-min-rad",
                "-0.1",
                "--alpha-max-rad",
                "0.2",
                "--elevator-min",
                "-0.5",
                "--elevator-max",
                "0.5",
                "--throttle-min",
                "0.1",
                "--throttle-max",
                "0.9",
                "--initial-alpha-rad",
                "0.05",
                "--initial-elevator",
                "0.0",
                "--initial-throttle",
                "0.4",
                "--force-tolerance-n",
                "1e-6",
                "--moment-tolerance-nm",
                "1e-7",
                "--max-iterations",
                "20",
            ],
        )
        .unwrap_err();
        assert!(err.contains("unknown"));
    }

    // ---- 7. Successful synthetic sweep produces ordered success rows ----

    #[test]
    fn successful_synthetic_sweep_produces_ordered_success_rows() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let (report, _config, _env) = run_synthetic_sweep(&options);
        assert_eq!(report.points.len(), 3);
        let speeds: Vec<f64> = report.points.iter().map(point_speed).collect();
        assert_eq!(speeds, vec![15.0, 18.0, 21.0]);
        for p in &report.points {
            assert!(matches!(p, PointInfo::Success(_)));
        }
    }

    // ---- 8. JSON counters equal M2.6A counters ----

    #[test]
    fn json_counters_equal_m2_6a_counters() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let (report, _, _) = run_synthetic_sweep(&options);
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
        let json = report.to_json_pretty().unwrap();
        let decoded = TrimSweepReport::from_json(&json).unwrap();
        assert_eq!(decoded.summary.success_count, sweep.success_count());
        assert_eq!(
            decoded.summary.trim_failure_count,
            sweep.trim_failure_count()
        );
        assert_eq!(
            decoded.summary.re_evaluation_mismatch_count,
            sweep.re_evaluation_mismatch_count()
        );
        assert_eq!(
            decoded.summary.re_evaluation_unverifiable_count,
            sweep.re_evaluation_unverifiable_count()
        );
    }

    // ---- 9. Markdown counters equal M2.6A counters ----

    #[test]
    fn markdown_counters_equal_m2_6a_counters() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let (report, _, _) = run_synthetic_sweep(&options);
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
        let md = report.to_markdown();
        for (name, count) in [
            ("Success", sweep.success_count()),
            ("Trim failure", sweep.trim_failure_count()),
            (
                "Re-evaluation mismatch",
                sweep.re_evaluation_mismatch_count(),
            ),
            (
                "Re-evaluation unverifiable",
                sweep.re_evaluation_unverifiable_count(),
            ),
        ] {
            let needle = format!("- {name}: `{count}`");
            assert!(md.contains(&needle), "missing markdown line {needle}");
        }
    }

    // ---- 10. Identical execution produces byte-identical JSON ----

    #[test]
    fn identical_execution_produces_byte_identical_json() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let a = run_synthetic_sweep(&options).0.to_json_pretty().unwrap();
        let b = run_synthetic_sweep(&options).0.to_json_pretty().unwrap();
        assert_eq!(a, b);
    }

    // ---- 11. Identical execution produces byte-identical Markdown ----

    #[test]
    fn identical_execution_produces_byte_identical_markdown() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let a = run_synthetic_sweep(&options).0.to_markdown();
        let b = run_synthetic_sweep(&options).0.to_markdown();
        assert_eq!(a, b);
    }

    // ---- 12. Deliberately infeasible bounded sweep completes and yields FAIL ----

    #[test]
    fn deliberately_infeasible_bounded_sweep_completes_and_yields_fail() {
        let mut options = base_template(vec![18.0]);
        options.alpha_bounds = TrimBounds::new(0.20, 0.21).unwrap();
        options.tolerances = LongitudinalTrimTolerances::new(1.0e-12, 1.0e-13).unwrap();
        options.maximum_iterations = 3;
        let tmp = std::env::temp_dir().join(format!(
            "rcsim_trim_sweep_m2_6b_fail_{}",
            std::process::id()
        ));
        options.output_dir = tmp.clone();
        let (report, _, _) = run_synthetic_sweep(&options);
        let has_trim_failure = report
            .points
            .iter()
            .any(|p| matches!(p, PointInfo::TrimFailure(_)));
        assert!(has_trim_failure);
        assert_eq!(report.summary.overall_status, OverallStatus::Fail);
        match run_trim_sweep_validation(options) {
            Err(TrimSweepValidationError::ValidationFailure { .. }) => {}
            other => panic!("expected ValidationFailure, got {other:?}"),
        }
        assert!(tmp.join(TRIM_SWEEP_JSON_NAME).exists());
        assert!(tmp.join(TRIM_SWEEP_MARKDOWN_NAME).exists());
        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ---- 13. Report fingerprint equals loaded model fingerprint ----

    #[test]
    fn report_fingerprint_equals_loaded_model_fingerprint() {
        let options = base_template(vec![18.0]);
        let (report, _, _) = run_synthetic_sweep(&options);
        let model = load_aircraft_model(options.model_path()).unwrap();
        assert_eq!(
            report.model.model_physics_fingerprint,
            fingerprint_hex(&model)
        );
    }

    // ---- 14. Reports contain no timestamp or wall-clock fields ----

    #[test]
    fn reports_contain_no_timestamp_or_wall_clock_fields() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let (report, _, _) = run_synthetic_sweep(&options);
        let json = report.to_json_pretty().unwrap();
        let md = report.to_markdown();
        for token in NON_FINITE_FORBIDDEN_TOKENS {
            assert!(!json.contains(token), "JSON must not contain `{token}`");
            assert!(
                !md.to_lowercase().contains(token),
                "Markdown must not contain `{token}`"
            );
        }
    }

    // ---- 15. Synthetic fixture remains synthetic_test ----

    #[test]
    fn synthetic_fixture_remains_synthetic_test_and_is_not_promoted() {
        use model::AircraftClassification;
        let options = base_template(vec![15.0]);
        let model = load_aircraft_model(options.model_path()).unwrap();
        assert_eq!(
            model.classification(),
            AircraftClassification::SyntheticTest
        );
        assert!(model.reference_aircraft().is_none());
        let lower = std::fs::read_to_string(options.model_path())
            .unwrap()
            .to_ascii_lowercase();
        for forbidden in ["sig", "kadet", "lt-40", "apc", "himax", "castle"] {
            assert!(!lower.contains(forbidden));
        }
    }

    // ---- full pipeline writes two artifacts on PASS ----

    #[test]
    fn full_pipeline_writes_two_artifacts_on_pass() {
        let mut options = base_template(vec![15.0, 18.0, 21.0]);
        let tmp = std::env::temp_dir().join(format!(
            "rcsim_trim_sweep_m2_6b_pass_{}",
            std::process::id()
        ));
        options.output_dir = tmp.clone();
        run_trim_sweep_validation(options).unwrap();
        assert!(tmp.join(TRIM_SWEEP_JSON_NAME).exists());
        assert!(tmp.join(TRIM_SWEEP_MARKDOWN_NAME).exists());
        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ---- runner creates missing output directories ----

    #[test]
    fn runner_creates_missing_output_directory() {
        let mut options = base_template(vec![18.0]);
        let tmp = std::env::temp_dir().join(format!(
            "rcsim_trim_sweep_m2_6b_mkdir_{}",
            std::process::id()
        ));
        let nested = tmp.join("nested").join("directory");
        options.output_dir = nested.clone();
        run_trim_sweep_validation(options).unwrap();
        assert!(nested.join(TRIM_SWEEP_JSON_NAME).exists());
        let _ = std::fs::remove_dir_all(&tmp);
    }

    // ---- JSON decoding is strict and versioned ----

    #[test]
    fn current_report_schema_version_is_accepted() {
        let json = synthetic_report_json();
        let report = TrimSweepReport::from_json(&json).unwrap();
        assert_eq!(report.schema_version, TRIM_SWEEP_REPORT_SCHEMA_VERSION);
    }

    #[test]
    fn unsupported_report_schema_versions_fail_closed() {
        let json = synthetic_report_json();
        for found in [0, 2, 999] {
            let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
            value["schema_version"] = serde_json::json!(found);
            match TrimSweepReport::from_json(&value.to_string()) {
                Err(TrimSweepValidationError::UnsupportedReportSchemaVersion {
                    found: actual,
                    expected,
                }) => {
                    assert_eq!(actual, found);
                    assert_eq!(expected, TRIM_SWEEP_REPORT_SCHEMA_VERSION);
                }
                other => panic!("expected unsupported schema version {found}, got {other:?}"),
            }
        }
    }

    #[test]
    fn malformed_report_json_is_rejected_as_a_decode_error() {
        match TrimSweepReport::from_json("{") {
            Err(TrimSweepValidationError::DeserializeReport(_)) => {}
            other => panic!("expected report decode error, got {other:?}"),
        }
    }

    #[test]
    fn unknown_report_root_field_is_rejected() {
        let json = synthetic_report_json();
        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        value["unknown_field"] = serde_json::json!(true);
        match TrimSweepReport::from_json(&value.to_string()) {
            Err(TrimSweepValidationError::DeserializeReport(_)) => {}
            other => panic!("expected unknown-field decode error, got {other:?}"),
        }
    }

    #[test]
    fn invalid_report_outcome_enum_is_rejected() {
        let json = synthetic_report_json();
        let mut bad_outcome: serde_json::Value = serde_json::from_str(&json).unwrap();
        bad_outcome["points"][0]["outcome"] = serde_json::json!("bogus");
        match TrimSweepReport::from_json(&bad_outcome.to_string()) {
            Err(TrimSweepValidationError::DeserializeReport(_)) => {}
            other => panic!("expected invalid-outcome decode error, got {other:?}"),
        }
    }

    #[test]
    fn valid_report_round_trip_remains_exact() {
        let options = base_template(vec![15.0, 18.0, 21.0]);
        let report = run_synthetic_sweep(&options).0;
        let json = report.to_json_pretty().unwrap();
        let decoded = TrimSweepReport::from_json(&json).unwrap();
        assert_eq!(decoded, report);
    }

    // ---- source file forbids runtime nondeterminism APIs ----

    #[test]
    fn report_source_uses_no_runtime_nondeterminism() {
        let source = include_str!("trim_sweep_validation_app.rs");
        // Restrict the check to the function bodies of the production code path, i.e.
        // everything above the `#[cfg(test)] mod tests` line. The test module is
        // allowed to mention forbidden tokens as string literals.
        let cfg_test_idx = source.find("#[cfg(test)]").unwrap_or(source.len());
        let production_source = &source[..cfg_test_idx];
        // Check for actual API usage patterns, not bare identifier mentions, so
        // that a literal "SystemTime" in a doc comment or token list cannot trip the
        // guard.
        let forbidden_patterns = [
            ("SystemTime::now", "use of SystemTime::now()"),
            ("Instant::now", "use of Instant::now()"),
            ("Utc::now", "use of Utc::now()"),
            ("std::process::id", "use of std::process::id()"),
            ("rand::thread_rng", "use of rand::thread_rng()"),
            ("rand::random", "use of rand::random()"),
        ];
        for (pattern, description) in forbidden_patterns {
            assert!(
                !production_source.contains(pattern),
                "production source must not reference {description}"
            );
        }
    }

    // ---- synthetic fixture is reachable from the app manifest dir ----

    #[test]
    fn synthetic_fixture_is_reachable_from_app_manifest() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(SYNTHETIC_FIXTURE_RELATIVE_PATH);
        assert!(path.exists(), "synthetic fixture must exist at {path:?}");
    }
}
