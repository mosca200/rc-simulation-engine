use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use serde::Serialize;
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, PropellerCoefficientSource, PropulsionOutput,
    RigidBodyState, ShaftSpeedRangeStatus, evaluate_electric_propulsion_with_source,
};
use sim_math::{Orientation, Vec3};
use std::{fmt::Write as _, fs::OpenOptions, io::Write as _, path::PathBuf};
use thiserror::Error;

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_AIR_DENSITY_KG_M3: f64 = 1.225;
const DEFAULT_THROTTLE_START: f64 = 0.0;
const DEFAULT_THROTTLE_END: f64 = 1.0;
const DEFAULT_THROTTLE_STEP: f64 = 0.25;
const DEFAULT_AIRSPEED_START_MPS: f64 = 0.0;
const DEFAULT_AIRSPEED_END_MPS: f64 = 25.0;
const DEFAULT_AIRSPEED_STEP_MPS: f64 = 5.0;
const MAX_AXIS_POINTS: usize = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OutputFormat {
    Table,
    Csv,
    Json,
}

impl OutputFormat {
    fn parse(value: &str) -> Result<Self, PropulsionBenchError> {
        match value {
            "table" => Ok(Self::Table),
            "csv" => Ok(Self::Csv),
            "json" => Ok(Self::Json),
            _ => Err(PropulsionBenchError::InvalidFormat(value.to_owned())),
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct PropulsionBenchOptions {
    model_path: PathBuf,
    format: OutputFormat,
    output_path: Option<PathBuf>,
    throttle: Option<f64>,
    airspeed_mps: Option<f64>,
    throttle_start: f64,
    throttle_end: f64,
    throttle_step: f64,
    airspeed_start_mps: f64,
    airspeed_end_mps: f64,
    airspeed_step_mps: f64,
    sweep_requested: bool,
}

impl PropulsionBenchOptions {
    pub(crate) fn parse(
        mut arguments: impl Iterator<Item = String>,
    ) -> Result<Self, PropulsionBenchError> {
        let mut options = Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            format: OutputFormat::Table,
            output_path: None,
            throttle: None,
            airspeed_mps: None,
            throttle_start: DEFAULT_THROTTLE_START,
            throttle_end: DEFAULT_THROTTLE_END,
            throttle_step: DEFAULT_THROTTLE_STEP,
            airspeed_start_mps: DEFAULT_AIRSPEED_START_MPS,
            airspeed_end_mps: DEFAULT_AIRSPEED_END_MPS,
            airspeed_step_mps: DEFAULT_AIRSPEED_STEP_MPS,
            sweep_requested: false,
        };
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => {
                    options.model_path = PathBuf::from(next_value(&argument, &mut arguments)?)
                }
                "--format" => {
                    options.format = OutputFormat::parse(&next_value(&argument, &mut arguments)?)?
                }
                "--output" => {
                    options.output_path =
                        Some(PathBuf::from(next_value(&argument, &mut arguments)?))
                }
                "--throttle" => options.throttle = Some(parse_number(&argument, &mut arguments)?),
                "--airspeed-mps" => {
                    options.airspeed_mps = Some(parse_number(&argument, &mut arguments)?)
                }
                "--throttle-start" => {
                    options.throttle_start = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--throttle-end" => {
                    options.throttle_end = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--throttle-step" => {
                    options.throttle_step = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--airspeed-start-mps" => {
                    options.airspeed_start_mps = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--airspeed-end-mps" => {
                    options.airspeed_end_mps = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--airspeed-step-mps" => {
                    options.airspeed_step_mps = parse_number(&argument, &mut arguments)?;
                    options.sweep_requested = true;
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(PropulsionBenchError::UnknownArgument(argument)),
            }
        }
        options.validate()?;
        Ok(options)
    }

    fn validate(&self) -> Result<(), PropulsionBenchError> {
        if self.sweep_requested && (self.throttle.is_some() || self.airspeed_mps.is_some()) {
            return Err(PropulsionBenchError::MixedSinglePointAndSweep);
        }
        if let Some(throttle) = self.throttle {
            validate_throttle(throttle)?;
        }
        if let Some(airspeed_mps) = self.airspeed_mps {
            validate_airspeed(airspeed_mps)?;
        }
        if self.sweep_requested || (self.throttle.is_none() && self.airspeed_mps.is_none()) {
            validate_range(
                "throttle",
                self.throttle_start,
                self.throttle_end,
                self.throttle_step,
            )?;
            validate_throttle(self.throttle_start)?;
            validate_throttle(self.throttle_end)?;
            validate_range(
                "airspeed",
                self.airspeed_start_mps,
                self.airspeed_end_mps,
                self.airspeed_step_mps,
            )?;
            validate_airspeed(self.airspeed_start_mps)?;
            validate_airspeed(self.airspeed_end_mps)?;
        }
        Ok(())
    }

    fn axes(&self) -> Result<(Vec<f64>, Vec<f64>), PropulsionBenchError> {
        if self.sweep_requested || (self.throttle.is_none() && self.airspeed_mps.is_none()) {
            Ok((
                inclusive_range(self.throttle_start, self.throttle_end, self.throttle_step)?,
                inclusive_range(
                    self.airspeed_start_mps,
                    self.airspeed_end_mps,
                    self.airspeed_step_mps,
                )?,
            ))
        } else {
            Ok((
                vec![self.throttle.unwrap_or(1.0)],
                vec![self.airspeed_mps.unwrap_or(0.0)],
            ))
        }
    }
}

fn next_value(
    flag: &str,
    arguments: &mut impl Iterator<Item = String>,
) -> Result<String, PropulsionBenchError> {
    arguments
        .next()
        .ok_or_else(|| PropulsionBenchError::MissingValue(flag.to_owned()))
}

fn parse_number(
    flag: &str,
    arguments: &mut impl Iterator<Item = String>,
) -> Result<f64, PropulsionBenchError> {
    next_value(flag, arguments)?
        .parse()
        .map_err(|_| PropulsionBenchError::InvalidNumber(flag.to_owned()))
}

fn validate_throttle(throttle: f64) -> Result<(), PropulsionBenchError> {
    if throttle.is_finite() && (0.0..=1.0).contains(&throttle) {
        Ok(())
    } else {
        Err(PropulsionBenchError::InvalidThrottle)
    }
}

fn validate_airspeed(airspeed_mps: f64) -> Result<(), PropulsionBenchError> {
    if airspeed_mps.is_finite() && airspeed_mps >= 0.0 {
        Ok(())
    } else {
        Err(PropulsionBenchError::InvalidAirspeed)
    }
}

fn validate_range(
    name: &'static str,
    start: f64,
    end: f64,
    step: f64,
) -> Result<(), PropulsionBenchError> {
    if !start.is_finite() || !end.is_finite() || !step.is_finite() || step <= 0.0 || start > end {
        Err(PropulsionBenchError::InvalidRange(name))
    } else {
        Ok(())
    }
}

fn inclusive_range(start: f64, end: f64, step: f64) -> Result<Vec<f64>, PropulsionBenchError> {
    let mut values = Vec::new();
    for index in 0..MAX_AXIS_POINTS {
        let value = (index as f64).mul_add(step, start);
        let tolerance = 16.0 * f64::EPSILON * end.abs().max(start.abs()).max(1.0);
        if value > end + tolerance {
            return Ok(values);
        }
        if (value - end).abs() <= tolerance {
            values.push(end);
            return Ok(values);
        }
        values.push(value);
    }
    Err(PropulsionBenchError::TooManySweepPoints)
}

#[derive(Debug, Error)]
pub(crate) enum PropulsionBenchError {
    #[error("missing value for propulsion bench option {0}")]
    MissingValue(String),
    #[error("invalid numeric value for propulsion bench option {0}")]
    InvalidNumber(String),
    #[error("unknown propulsion bench argument: {0}")]
    UnknownArgument(String),
    #[error("invalid output format {0}; expected table, csv, or json")]
    InvalidFormat(String),
    #[error("single-point options cannot be combined with sweep options")]
    MixedSinglePointAndSweep,
    #[error("throttle must be finite and within [0, 1]")]
    InvalidThrottle,
    #[error("airspeed must be finite and non-negative")]
    InvalidAirspeed,
    #[error("invalid {0} sweep: start/end must be finite and ordered and step must be positive")]
    InvalidRange(&'static str),
    #[error("propulsion bench sweep axis exceeds {MAX_AXIS_POINTS} points")]
    TooManySweepPoints,
    #[error("failed to load propulsion bench model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("model {0} has no propulsion configuration")]
    MissingPropulsion(String),
    #[error("failed to configure propulsion bench atmosphere: {0}")]
    Atmosphere(#[from] AeroEnvironmentError),
    #[error("production propulsion output {0} is non-finite")]
    NonFiniteOutput(&'static str),
    #[error("failed to serialize propulsion bench JSON: {0}")]
    Json(#[from] serde_json::Error),
    #[error(
        "failed to create propulsion bench output {path}; existing files are never overwritten: {source}"
    )]
    WriteOutput {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct PropulsionBenchReport {
    schema_version: u32,
    model_id: String,
    model_physics_fingerprint: String,
    coefficient_source: String,
    air_density_kg_m3: f64,
    operating_points: Vec<PropulsionBenchPoint>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub(crate) struct PropulsionBenchPoint {
    throttle: f64,
    airspeed_mps: f64,
    axial_inflow_mps: f64,
    battery_open_circuit_voltage_v: f64,
    battery_terminal_voltage_v: f64,
    battery_current_a: f64,
    battery_terminal_electrical_power_w: f64,
    esc_loss_power_w: f64,
    motor_voltage_v: f64,
    motor_current_a: f64,
    motor_electrical_input_power_w: f64,
    shaft_speed_rad_s: f64,
    shaft_speed_rpm: f64,
    motor_torque_nm: f64,
    propeller_torque_nm: f64,
    thrust_n: f64,
    advance_ratio_j: f64,
    ct: f64,
    cq: f64,
    mechanical_shaft_power_w: f64,
    useful_propulsive_power_w: f64,
    drive_efficiency: Option<f64>,
    propulsive_efficiency: Option<f64>,
    coefficient_lower_shaft_speed_rad_s: f64,
    coefficient_upper_shaft_speed_rad_s: f64,
    coefficient_interpolation_fraction: f64,
    coefficient_shaft_speed_range_status: String,
}

pub(crate) fn run_propulsion_bench(
    options: PropulsionBenchOptions,
) -> Result<(), PropulsionBenchError> {
    let model = load_aircraft_model(&options.model_path).map_err(|source| {
        PropulsionBenchError::ModelLoad {
            path: options.model_path.clone(),
            source,
        }
    })?;
    let report = build_report(&model, &options)?;
    let rendered = render_report(&report, options.format)?;
    if let Some(path) = options.output_path {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&path)
            .map_err(|source| PropulsionBenchError::WriteOutput {
                path: path.clone(),
                source,
            })?;
        file.write_all(rendered.as_bytes()).map_err(|source| {
            PropulsionBenchError::WriteOutput {
                path: path.clone(),
                source,
            }
        })?;
        println!("propulsion bench output written: {}", path.display());
    } else {
        print!("{rendered}");
    }
    Ok(())
}

fn build_report(
    model: &AircraftModel,
    options: &PropulsionBenchOptions,
) -> Result<PropulsionBenchReport, PropulsionBenchError> {
    let runtime = model
        .propulsion()
        .ok_or_else(|| PropulsionBenchError::MissingPropulsion(model.model_id().to_owned()))?;
    let coefficient_source = match runtime.coefficient_source() {
        PropellerCoefficientSource::FixedTable(_) => "fixed_table",
        PropellerCoefficientSource::ShaftSpeedMap(_) => "shaft_speed_map",
    };
    let environment = AeroEnvironment::new(DEFAULT_AIR_DENSITY_KG_M3, Vec3::zeros())?;
    let (throttles, airspeeds) = options.axes()?;
    let mut operating_points = Vec::with_capacity(throttles.len() * airspeeds.len());
    for throttle in throttles {
        for &airspeed_mps in &airspeeds {
            operating_points.push(evaluate_bench_point(
                model,
                &environment,
                throttle,
                airspeed_mps,
            )?);
        }
    }
    Ok(PropulsionBenchReport {
        schema_version: 1,
        model_id: model.model_id().to_owned(),
        model_physics_fingerprint: fingerprint_hex(model),
        coefficient_source: coefficient_source.to_owned(),
        air_density_kg_m3: environment.air_density_kg_m3(),
        operating_points,
    })
}

fn bench_state(
    model: &AircraftModel,
    airspeed_mps: f64,
) -> Result<RigidBodyState, PropulsionBenchError> {
    let runtime = model
        .propulsion()
        .ok_or_else(|| PropulsionBenchError::MissingPropulsion(model.model_id().to_owned()))?;
    let axial_body = runtime
        .config()
        .propeller()
        .orientation_body_from_prop()
        .transform_vector(&Vec3::new(1.0, 0.0, 0.0));
    Ok(RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: axial_body * airspeed_mps,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    })
}

fn evaluate_production_output(
    model: &AircraftModel,
    environment: &AeroEnvironment,
    throttle: f64,
    airspeed_mps: f64,
) -> Result<PropulsionOutput, PropulsionBenchError> {
    let runtime = model
        .propulsion()
        .ok_or_else(|| PropulsionBenchError::MissingPropulsion(model.model_id().to_owned()))?;
    let state = bench_state(model, airspeed_mps)?;
    Ok(evaluate_electric_propulsion_with_source(
        &state,
        throttle,
        runtime.config(),
        environment,
        runtime.coefficient_source(),
    ))
}

fn evaluate_bench_point(
    model: &AircraftModel,
    environment: &AeroEnvironment,
    throttle: f64,
    airspeed_mps: f64,
) -> Result<PropulsionBenchPoint, PropulsionBenchError> {
    let runtime = model
        .propulsion()
        .ok_or_else(|| PropulsionBenchError::MissingPropulsion(model.model_id().to_owned()))?;
    let output = evaluate_production_output(model, environment, throttle, airspeed_mps)?;
    point_from_output(
        output,
        airspeed_mps,
        runtime.config().battery().open_circuit_voltage_v(),
    )
}

fn point_from_output(
    output: PropulsionOutput,
    airspeed_mps: f64,
    battery_open_circuit_voltage_v: f64,
) -> Result<PropulsionBenchPoint, PropulsionBenchError> {
    let mechanical_shaft_power_w = output.motor_torque_nm * output.shaft_speed_rad_s;
    let useful_propulsive_power_w = output.thrust_n * output.axial_airspeed_mps;
    let drive_efficiency = positive_ratio(
        mechanical_shaft_power_w,
        output.battery_terminal_electrical_power_w,
    );
    let propulsive_efficiency = if output.axial_airspeed_mps > 0.0 {
        positive_ratio(useful_propulsive_power_w, mechanical_shaft_power_w)
    } else {
        None
    };
    let point = PropulsionBenchPoint {
        throttle: output.throttle,
        airspeed_mps,
        axial_inflow_mps: output.axial_airspeed_mps,
        battery_open_circuit_voltage_v,
        battery_terminal_voltage_v: output.battery_terminal_voltage_v,
        battery_current_a: output.battery_current_a,
        battery_terminal_electrical_power_w: output.battery_terminal_electrical_power_w,
        esc_loss_power_w: output.esc_loss_power_w,
        motor_voltage_v: output.motor_voltage_v,
        motor_current_a: output.motor_current_a,
        motor_electrical_input_power_w: output.motor_electrical_input_power_w,
        shaft_speed_rad_s: output.shaft_speed_rad_s,
        shaft_speed_rpm: output.shaft_speed_rpm,
        motor_torque_nm: output.motor_torque_nm,
        propeller_torque_nm: output.propeller_load_torque_nm,
        thrust_n: output.thrust_n,
        advance_ratio_j: output.advance_ratio_j,
        ct: output.coefficients.ct,
        cq: output.coefficients.cq,
        mechanical_shaft_power_w,
        useful_propulsive_power_w,
        drive_efficiency,
        propulsive_efficiency,
        coefficient_lower_shaft_speed_rad_s: output.coefficient_map_sample.lower_shaft_speed_rad_s,
        coefficient_upper_shaft_speed_rad_s: output.coefficient_map_sample.upper_shaft_speed_rad_s,
        coefficient_interpolation_fraction: output.coefficient_map_sample.interpolation_fraction,
        coefficient_shaft_speed_range_status: range_status_label(
            output.coefficient_map_sample.range_status,
        )
        .to_owned(),
    };
    validate_finite_point(&point)?;
    Ok(point)
}

fn positive_ratio(numerator: f64, denominator: f64) -> Option<f64> {
    if denominator > 0.0 {
        Some(numerator / denominator)
    } else {
        None
    }
}

fn validate_finite_point(point: &PropulsionBenchPoint) -> Result<(), PropulsionBenchError> {
    let values = [
        ("throttle", point.throttle),
        ("airspeed_mps", point.airspeed_mps),
        ("axial_inflow_mps", point.axial_inflow_mps),
        (
            "battery_open_circuit_voltage_v",
            point.battery_open_circuit_voltage_v,
        ),
        (
            "battery_terminal_voltage_v",
            point.battery_terminal_voltage_v,
        ),
        ("battery_current_a", point.battery_current_a),
        (
            "battery_terminal_electrical_power_w",
            point.battery_terminal_electrical_power_w,
        ),
        ("esc_loss_power_w", point.esc_loss_power_w),
        ("motor_voltage_v", point.motor_voltage_v),
        ("motor_current_a", point.motor_current_a),
        (
            "motor_electrical_input_power_w",
            point.motor_electrical_input_power_w,
        ),
        ("shaft_speed_rad_s", point.shaft_speed_rad_s),
        ("shaft_speed_rpm", point.shaft_speed_rpm),
        ("motor_torque_nm", point.motor_torque_nm),
        ("propeller_torque_nm", point.propeller_torque_nm),
        ("thrust_n", point.thrust_n),
        ("advance_ratio_j", point.advance_ratio_j),
        ("ct", point.ct),
        ("cq", point.cq),
        ("mechanical_shaft_power_w", point.mechanical_shaft_power_w),
        ("useful_propulsive_power_w", point.useful_propulsive_power_w),
        (
            "coefficient_lower_shaft_speed_rad_s",
            point.coefficient_lower_shaft_speed_rad_s,
        ),
        (
            "coefficient_upper_shaft_speed_rad_s",
            point.coefficient_upper_shaft_speed_rad_s,
        ),
        (
            "coefficient_interpolation_fraction",
            point.coefficient_interpolation_fraction,
        ),
    ];
    for (name, value) in values {
        if !value.is_finite() {
            return Err(PropulsionBenchError::NonFiniteOutput(name));
        }
    }
    for (name, value) in [
        ("drive_efficiency", point.drive_efficiency),
        ("propulsive_efficiency", point.propulsive_efficiency),
    ] {
        if value.is_some_and(|value| !value.is_finite()) {
            return Err(PropulsionBenchError::NonFiniteOutput(name));
        }
    }
    Ok(())
}

fn range_status_label(status: ShaftSpeedRangeStatus) -> &'static str {
    match status {
        ShaftSpeedRangeStatus::BelowRange => "below_range_clamped",
        ShaftSpeedRangeStatus::ExactOrInRange => "exact_or_in_range",
        ShaftSpeedRangeStatus::AboveRange => "above_range_clamped",
    }
}

fn fingerprint_hex(model: &AircraftModel) -> String {
    let mut output = String::with_capacity(64);
    for byte in model.physics_fingerprint().as_bytes() {
        write!(output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn render_report(
    report: &PropulsionBenchReport,
    format: OutputFormat,
) -> Result<String, PropulsionBenchError> {
    match format {
        OutputFormat::Table => Ok(render_table(report)),
        OutputFormat::Csv => Ok(render_csv(report)),
        OutputFormat::Json => {
            let mut output = serde_json::to_string_pretty(report)?;
            output.push('\n');
            Ok(output)
        }
    }
}

fn render_table(report: &PropulsionBenchReport) -> String {
    let mut output = String::new();
    writeln!(output, "RC Simulation Engine - Propulsion Bench").unwrap();
    writeln!(output, "model_id: {}", report.model_id).unwrap();
    writeln!(
        output,
        "model_physics_fingerprint: {}",
        report.model_physics_fingerprint
    )
    .unwrap();
    writeln!(output, "coefficient_source: {}", report.coefficient_source).unwrap();
    writeln!(output, "air_density_kg_m3: {:.6}", report.air_density_kg_m3).unwrap();
    writeln!(
        output,
        "operating_points: {}",
        report.operating_points.len()
    )
    .unwrap();
    writeln!(
        output,
        " thr   Vax   Voc  Vterm  Ibatt  Vmot  Imot    rad/s      rpm   Tmot  Tprop  thrust      J      Ct      Cq   Pbat  Pesc   Pmot Pshaft   Puse etaDrv etaProp  mapLow mapHigh mapFrac mapStatus"
    )
    .unwrap();
    for point in &report.operating_points {
        writeln!(
            output,
            "{:>4.2} {:>6.2} {:>5.2} {:>6.2} {:>6.2} {:>5.2} {:>5.2} {:>8.2} {:>8.1} {:>6.3} {:>6.3} {:>7.3} {:>6.3} {:>7.5} {:>7.5} {:>6.1} {:>5.1} {:>6.1} {:>6.1} {:>6.1} {:>6} {:>7} {:>7.1} {:>7.1} {:>7.4} {}",
            point.throttle,
            point.axial_inflow_mps,
            point.battery_open_circuit_voltage_v,
            point.battery_terminal_voltage_v,
            point.battery_current_a,
            point.motor_voltage_v,
            point.motor_current_a,
            point.shaft_speed_rad_s,
            point.shaft_speed_rpm,
            point.motor_torque_nm,
            point.propeller_torque_nm,
            point.thrust_n,
            point.advance_ratio_j,
            point.ct,
            point.cq,
            point.battery_terminal_electrical_power_w,
            point.esc_loss_power_w,
            point.motor_electrical_input_power_w,
            point.mechanical_shaft_power_w,
            point.useful_propulsive_power_w,
            optional_table(point.drive_efficiency),
            optional_table(point.propulsive_efficiency),
            point.coefficient_lower_shaft_speed_rad_s,
            point.coefficient_upper_shaft_speed_rad_s,
            point.coefficient_interpolation_fraction,
            point.coefficient_shaft_speed_range_status,
        )
        .unwrap();
    }
    output
}

fn optional_table(value: Option<f64>) -> String {
    value.map_or_else(|| "N/A".to_owned(), |value| format!("{value:.4}"))
}

fn render_csv(report: &PropulsionBenchReport) -> String {
    let mut output = String::from(
        "schema_version,model_id,model_physics_fingerprint,coefficient_source,air_density_kg_m3,throttle,airspeed_mps,axial_inflow_mps,battery_open_circuit_voltage_v,battery_terminal_voltage_v,battery_current_a,battery_terminal_electrical_power_w,esc_loss_power_w,motor_voltage_v,motor_current_a,motor_electrical_input_power_w,shaft_speed_rad_s,shaft_speed_rpm,motor_torque_nm,propeller_torque_nm,thrust_n,advance_ratio_j,ct,cq,mechanical_shaft_power_w,useful_propulsive_power_w,drive_efficiency,propulsive_efficiency,coefficient_lower_shaft_speed_rad_s,coefficient_upper_shaft_speed_rad_s,coefficient_interpolation_fraction,coefficient_shaft_speed_range_status\n",
    );
    for point in &report.operating_points {
        writeln!(
            output,
            "{},{},{},{},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{:.17},{},{},{:.17},{:.17},{:.17},{}",
            report.schema_version,
            csv_escape(&report.model_id),
            report.model_physics_fingerprint,
            report.coefficient_source,
            report.air_density_kg_m3,
            point.throttle,
            point.airspeed_mps,
            point.axial_inflow_mps,
            point.battery_open_circuit_voltage_v,
            point.battery_terminal_voltage_v,
            point.battery_current_a,
            point.battery_terminal_electrical_power_w,
            point.esc_loss_power_w,
            point.motor_voltage_v,
            point.motor_current_a,
            point.motor_electrical_input_power_w,
            point.shaft_speed_rad_s,
            point.shaft_speed_rpm,
            point.motor_torque_nm,
            point.propeller_torque_nm,
            point.thrust_n,
            point.advance_ratio_j,
            point.ct,
            point.cq,
            point.mechanical_shaft_power_w,
            point.useful_propulsive_power_w,
            optional_csv(point.drive_efficiency),
            optional_csv(point.propulsive_efficiency),
            point.coefficient_lower_shaft_speed_rad_s,
            point.coefficient_upper_shaft_speed_rad_s,
            point.coefficient_interpolation_fraction,
            point.coefficient_shaft_speed_range_status,
        )
        .unwrap();
    }
    output
}

fn optional_csv(value: Option<f64>) -> String {
    value.map_or_else(String::new, |value| format!("{value:.17}"))
}

fn csv_escape(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aircraft::{AircraftSimulation, AircraftSimulationConfig, evaluate_aircraft_instantaneous};
    use model::AircraftModelLoader;
    use sim_core::{PilotInput, PropellerCoefficientSource};
    use std::{f64::consts::TAU, path::Path};

    fn repository_path(relative: &str) -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    fn load_model(relative: &str) -> AircraftModel {
        load_aircraft_model(repository_path(relative)).unwrap()
    }

    fn single_options(throttle: f64, airspeed_mps: f64) -> PropulsionBenchOptions {
        PropulsionBenchOptions::parse(
            [
                "--throttle".to_owned(),
                throttle.to_string(),
                "--airspeed-mps".to_owned(),
                airspeed_mps.to_string(),
            ]
            .into_iter(),
        )
        .unwrap()
    }

    #[test]
    fn synthetic_fixed_table_matches_independent_static_analytic_oracle() {
        let model = load_model("tests/fixtures/synthetic_propulsion_bench_ground_v8.json");
        let report = build_report(&model, &single_options(0.75, 0.0)).unwrap();
        let point = &report.operating_points[0];
        let runtime = model.propulsion().unwrap();
        let config = runtime.config();
        let battery = config.battery();
        let esc = config.esc();
        let motor = config.motor();
        let propeller = config.propeller();
        let cq = 0.01;
        let resistance = motor.winding_resistance_ohm()
            + esc.series_resistance_ohm()
            + 0.75_f64.powi(2) * battery.internal_resistance_ohm();
        let load_quadratic =
            cq * DEFAULT_AIR_DENSITY_KG_M3 * propeller.diameter_m().powi(5) / TAU.powi(2);
        let linear =
            motor.torque_constant_nm_per_a() * motor.back_emf_constant_v_per_rad_s() / resistance;
        let constant = motor.torque_constant_nm_per_a()
            * (0.75 * battery.open_circuit_voltage_v() / resistance - motor.no_load_current_a());
        let expected_rad_s = (-linear
            + linear
                .mul_add(linear, 4.0 * load_quadratic * constant)
                .sqrt())
            / (2.0 * load_quadratic);
        let expected_thrust = 0.1
            * DEFAULT_AIR_DENSITY_KG_M3
            * (expected_rad_s / TAU).powi(2)
            * propeller.diameter_m().powi(4);
        assert!((point.shaft_speed_rad_s - expected_rad_s).abs() < 1.0e-10);
        assert!((point.thrust_n - expected_thrust).abs() < 1.0e-11);
        assert_eq!(point.advance_ratio_j, 0.0);
        assert_eq!(point.ct, 0.1);
        assert_eq!(point.cq, cq);
        assert_eq!(point.useful_propulsive_power_w, 0.0);
        assert_eq!(point.propulsive_efficiency, None);
    }

    #[test]
    fn bench_and_aircraft_stage_use_bit_identical_production_propulsion() {
        let model = load_model("models/acro_electric_01/model.json");
        let environment = AeroEnvironment::new(DEFAULT_AIR_DENSITY_KG_M3, Vec3::zeros()).unwrap();
        let config = AircraftSimulationConfig::from_physics_hz(500, environment).unwrap();
        let state = bench_state(&model, 15.0).unwrap();
        let elements = model
            .aero_elements()
            .iter()
            .map(|element| *element.element())
            .collect::<Vec<_>>();
        let aircraft_output =
            *evaluate_aircraft_instantaneous(&state, &elements, &model, 0.5, &config)
                .propulsion()
                .unwrap();
        let bench_output = evaluate_production_output(&model, &environment, 0.5, 15.0).unwrap();
        assert_eq!(bench_output, aircraft_output);
    }

    #[test]
    fn fixed_table_and_shaft_speed_map_both_use_production_source_semantics() {
        let fixed = load_model("models/acro_electric_01/model.json");
        let fixed_report = build_report(&fixed, &single_options(1.0, 0.0)).unwrap();
        assert_eq!(fixed_report.coefficient_source, "fixed_table");
        let fixed_table = match fixed.propulsion().unwrap().coefficient_source() {
            PropellerCoefficientSource::FixedTable(table) => table,
            PropellerCoefficientSource::ShaftSpeedMap(_) => panic!("expected fixed table"),
        };
        let zero_knot = fixed_table
            .samples()
            .iter()
            .find(|sample| sample.advance_ratio_j == 0.0)
            .unwrap();
        assert_eq!(fixed_report.operating_points[0].ct, zero_knot.ct);
        assert_eq!(fixed_report.operating_points[0].cq, zero_knot.cq);

        let mapped = load_model("tests/fixtures/synthetic_non_reference_propulsion_v4.json");
        let mapped_report = build_report(&mapped, &single_options(0.5, 10.0)).unwrap();
        assert_eq!(mapped_report.coefficient_source, "shaft_speed_map");
        assert!(mapped_report.operating_points[0].coefficient_interpolation_fraction >= 0.0);
    }

    #[test]
    fn out_of_range_advance_ratio_uses_authored_endpoint_clamp() {
        let model = load_model("tests/fixtures/synthetic_fixed_table_narrow_j_v4.json");
        let report = build_report(&model, &single_options(0.25, 100.0)).unwrap();
        let point = &report.operating_points[0];
        let table = match model.propulsion().unwrap().coefficient_source() {
            PropellerCoefficientSource::FixedTable(table) => table,
            PropellerCoefficientSource::ShaftSpeedMap(_) => panic!("expected fixed table"),
        };
        let last = table.samples().last().unwrap();
        assert!(point.advance_ratio_j > last.advance_ratio_j);
        assert_eq!(point.ct, last.ct);
        assert_eq!(point.cq, last.cq);
    }

    #[test]
    fn default_sweep_is_ordered_deterministic_finite_and_power_bounded() {
        let options = PropulsionBenchOptions::parse(std::iter::empty()).unwrap();
        let model = load_model("models/acro_electric_01/model.json");
        let first = build_report(&model, &options).unwrap();
        let second = build_report(&model, &options).unwrap();
        assert_eq!(first, second);
        assert_eq!(first.operating_points.len(), 30);
        for (index, point) in first.operating_points.iter().enumerate() {
            let throttle_index = index / 6;
            let airspeed_index = index % 6;
            assert_eq!(point.throttle, throttle_index as f64 * 0.25);
            assert_eq!(point.airspeed_mps, airspeed_index as f64 * 5.0);
            validate_finite_point(point).unwrap();
            assert!((0.0..=1.0).contains(&point.throttle));
            assert!(point.battery_current_a >= 0.0);
            assert!(point.motor_current_a >= 0.0);
            assert!(point.shaft_speed_rad_s >= 0.0);
            assert!(point.battery_terminal_voltage_v >= 0.0);
            let tolerance = 1.0e-9 * point.motor_electrical_input_power_w.abs().max(1.0);
            assert!(
                point.mechanical_shaft_power_w <= point.motor_electrical_input_power_w + tolerance
            );
            let electrical_tolerance =
                1.0e-9 * point.battery_terminal_electrical_power_w.abs().max(1.0);
            assert!(
                point.motor_electrical_input_power_w + point.esc_loss_power_w
                    <= point.battery_terminal_electrical_power_w + electrical_tolerance
            );
            let torque_tolerance = 1.0e-10 * point.motor_torque_nm.abs().max(1.0);
            assert!((point.motor_torque_nm - point.propeller_torque_nm).abs() < torque_tolerance);
        }
        assert_eq!(
            render_report(&first, OutputFormat::Json).unwrap(),
            render_report(&second, OutputFormat::Json).unwrap()
        );
        assert_eq!(
            render_report(&first, OutputFormat::Csv).unwrap(),
            render_report(&second, OutputFormat::Csv).unwrap()
        );
        assert_eq!(
            render_report(&first, OutputFormat::Table).unwrap(),
            render_report(&second, OutputFormat::Table).unwrap()
        );
    }

    #[test]
    fn parser_rejects_invalid_and_mixed_operating_point_requests() {
        for arguments in [
            vec!["--throttle", "NaN"],
            vec!["--throttle", "1.1"],
            vec!["--airspeed-mps", "-1"],
            vec!["--throttle-step", "0"],
            vec!["--throttle", "0.5", "--throttle-start", "0"],
        ] {
            assert!(
                PropulsionBenchOptions::parse(arguments.into_iter().map(str::to_owned)).is_err()
            );
        }
    }

    #[test]
    fn production_propulsion_accelerates_the_ground_fixture_on_its_wheels() {
        let model = load_model("tests/fixtures/synthetic_propulsion_bench_ground_v8.json");
        let environment = AeroEnvironment::new(DEFAULT_AIR_DENSITY_KG_M3, Vec3::zeros()).unwrap();
        let config = AircraftSimulationConfig::from_physics_hz(500, environment).unwrap();
        let initial_state = RigidBodyState {
            position_world_m: Vec3::new(0.0, 0.0, -0.42),
            linear_velocity_world_mps: Vec3::zeros(),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let mut simulation = AircraftSimulation::new(model, config, initial_state).unwrap();
        let idle = PilotInput::new(0.0, 0.0, 0.0, 0.0);
        for _ in 0..500 {
            let _ = simulation.step(&idle);
        }
        let initial_speed_mps = simulation.state().rigid_body().linear_velocity_world_mps.x;
        let full_throttle = PilotInput::new(0.0, 0.0, 0.0, 1.0);
        let mut final_snapshot = simulation.step(&full_throttle);
        for _ in 1..750 {
            final_snapshot = simulation.step(&full_throttle);
        }
        let final_speed_mps = final_snapshot
            .rigid_body_state()
            .linear_velocity_world_mps
            .x;
        assert!(final_speed_mps > initial_speed_mps + 0.5);
        assert!(final_snapshot.weight_on_wheels());
        let runtime = simulation.model().propulsion().unwrap();
        let propulsion = evaluate_electric_propulsion_with_source(
            final_snapshot.rigid_body_state(),
            1.0,
            runtime.config(),
            config.aero_environment(),
            runtime.coefficient_source(),
        );
        let point = point_from_output(
            propulsion,
            propulsion.axial_airspeed_mps,
            runtime.config().battery().open_circuit_voltage_v(),
        )
        .unwrap();
        assert!(point.thrust_n > 0.0);
        assert!(point.battery_current_a > 0.0);
        assert!(point.shaft_speed_rpm > 0.0);
        println!(
            "ground_run initial_mps={initial_speed_mps:.6} final_mps={final_speed_mps:.6} thrust_n={:.6} current_a={:.6} rpm={:.3} weight_on_wheels={}",
            point.thrust_n,
            point.battery_current_a,
            point.shaft_speed_rpm,
            final_snapshot.weight_on_wheels()
        );
    }

    #[test]
    fn synthetic_fixture_is_strictly_loadable_and_explicitly_synthetic() {
        let text =
            include_str!("../../../tests/fixtures/synthetic_propulsion_bench_ground_v8.json");
        let model = AircraftModelLoader::from_json_str(text).unwrap();
        assert_eq!(model.model_id(), "synthetic-propulsion-bench-ground-v8");
        assert!(model.reference_aircraft().is_none());
    }
}
