//! Deterministic quasi-static electric propulsion for one fixed-pitch propeller.

use crate::{AeroEnvironment, BodyWrench, RigidBodyState};
use sim_math::{Orientation, Vec3, world_to_body};
use std::f64::consts::TAU;
use thiserror::Error;

/// Fixed iteration count used by the quasi-static motor/propeller solver.
pub const PROPULSION_BISECTION_ITERATIONS: usize = 48;
/// Below this magnitude the shaft is treated as stopped for advance-ratio evaluation.
pub const MIN_SHAFT_SPEED_RAD_S: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EscConfig {
    series_resistance_ohm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum EscConfigError {
    #[error("ESC series resistance must be finite and non-negative")]
    InvalidSeriesResistance,
}

impl EscConfig {
    pub fn new(series_resistance_ohm: f64) -> Result<Self, EscConfigError> {
        if !series_resistance_ohm.is_finite() || series_resistance_ohm < 0.0 {
            return Err(EscConfigError::InvalidSeriesResistance);
        }
        Ok(Self {
            series_resistance_ohm,
        })
    }

    #[must_use]
    pub const fn ideal() -> Self {
        Self {
            series_resistance_ohm: 0.0,
        }
    }

    #[must_use]
    pub const fn series_resistance_ohm(&self) -> f64 {
        self.series_resistance_ohm
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryConfig {
    open_circuit_voltage_v: f64,
    internal_resistance_ohm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum BatteryConfigError {
    #[error("battery open-circuit voltage must be finite and greater than zero")]
    InvalidOpenCircuitVoltage,
    #[error("battery internal resistance must be finite and non-negative")]
    InvalidInternalResistance,
}

impl BatteryConfig {
    pub fn new(
        open_circuit_voltage_v: f64,
        internal_resistance_ohm: f64,
    ) -> Result<Self, BatteryConfigError> {
        if !open_circuit_voltage_v.is_finite() || open_circuit_voltage_v <= 0.0 {
            return Err(BatteryConfigError::InvalidOpenCircuitVoltage);
        }
        if !internal_resistance_ohm.is_finite() || internal_resistance_ohm < 0.0 {
            return Err(BatteryConfigError::InvalidInternalResistance);
        }
        Ok(Self {
            open_circuit_voltage_v,
            internal_resistance_ohm,
        })
    }

    #[must_use]
    pub const fn open_circuit_voltage_v(&self) -> f64 {
        self.open_circuit_voltage_v
    }

    #[must_use]
    pub const fn internal_resistance_ohm(&self) -> f64 {
        self.internal_resistance_ohm
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorConfig {
    kv_rpm_per_v: f64,
    winding_resistance_ohm: f64,
    no_load_current_a: f64,
    kv_rad_s_per_v: f64,
    back_emf_constant_v_per_rad_s: f64,
    torque_constant_nm_per_a: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum MotorConfigError {
    #[error("motor Kv must be finite, greater than zero, and representable in SI")]
    InvalidKv,
    #[error("motor winding resistance must be finite and greater than zero")]
    InvalidWindingResistance,
    #[error("motor no-load current must be finite and non-negative")]
    InvalidNoLoadCurrent,
}

impl MotorConfig {
    pub fn new(
        kv_rpm_per_v: f64,
        winding_resistance_ohm: f64,
        no_load_current_a: f64,
    ) -> Result<Self, MotorConfigError> {
        let kv_rad_s_per_v = kv_rpm_per_v * TAU / 60.0;
        let back_emf_constant_v_per_rad_s = 1.0 / kv_rad_s_per_v;
        if !kv_rpm_per_v.is_finite()
            || kv_rpm_per_v <= 0.0
            || !kv_rad_s_per_v.is_finite()
            || kv_rad_s_per_v <= 0.0
            || !back_emf_constant_v_per_rad_s.is_finite()
            || back_emf_constant_v_per_rad_s <= 0.0
        {
            return Err(MotorConfigError::InvalidKv);
        }
        if !winding_resistance_ohm.is_finite() || winding_resistance_ohm <= 0.0 {
            return Err(MotorConfigError::InvalidWindingResistance);
        }
        if !no_load_current_a.is_finite() || no_load_current_a < 0.0 {
            return Err(MotorConfigError::InvalidNoLoadCurrent);
        }
        Ok(Self {
            kv_rpm_per_v,
            winding_resistance_ohm,
            no_load_current_a,
            kv_rad_s_per_v,
            back_emf_constant_v_per_rad_s,
            torque_constant_nm_per_a: back_emf_constant_v_per_rad_s,
        })
    }

    #[must_use]
    pub const fn kv_rpm_per_v(&self) -> f64 {
        self.kv_rpm_per_v
    }

    #[must_use]
    pub const fn winding_resistance_ohm(&self) -> f64 {
        self.winding_resistance_ohm
    }

    #[must_use]
    pub const fn no_load_current_a(&self) -> f64 {
        self.no_load_current_a
    }

    #[must_use]
    pub const fn kv_rad_s_per_v(&self) -> f64 {
        self.kv_rad_s_per_v
    }

    #[must_use]
    pub const fn back_emf_constant_v_per_rad_s(&self) -> f64 {
        self.back_emf_constant_v_per_rad_s
    }

    #[must_use]
    pub const fn torque_constant_nm_per_a(&self) -> f64 {
        self.torque_constant_nm_per_a
    }
}

/// Unambiguous shaft angular-velocity direction in the right-handed propeller frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropellerSpinDirection {
    PositiveAboutLocalX,
    NegativeAboutLocalX,
}

impl PropellerSpinDirection {
    const fn sign(self) -> f64 {
        match self {
            Self::PositiveAboutLocalX => 1.0,
            Self::NegativeAboutLocalX => -1.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerConfig {
    position_body_m: Vec3,
    orientation_body_from_prop: Orientation,
    diameter_m: f64,
    spin_direction: PropellerSpinDirection,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PropellerConfigError {
    #[error("propeller position must contain only finite values")]
    NonFinitePosition,
    #[error("propeller orientation must be finite and unit length")]
    InvalidOrientation,
    #[error("propeller diameter must be finite and greater than zero")]
    InvalidDiameter,
}

impl PropellerConfig {
    pub fn new(
        position_body_m: Vec3,
        orientation_body_from_prop: Orientation,
        diameter_m: f64,
        spin_direction: PropellerSpinDirection,
    ) -> Result<Self, PropellerConfigError> {
        if !position_body_m.iter().all(|value| value.is_finite()) {
            return Err(PropellerConfigError::NonFinitePosition);
        }
        let quaternion = orientation_body_from_prop.quaternion();
        if ![quaternion.w, quaternion.i, quaternion.j, quaternion.k]
            .into_iter()
            .all(f64::is_finite)
            || (quaternion.norm_squared() - 1.0).abs() > 1.0e-12
        {
            return Err(PropellerConfigError::InvalidOrientation);
        }
        if !diameter_m.is_finite() || diameter_m <= 0.0 {
            return Err(PropellerConfigError::InvalidDiameter);
        }
        Ok(Self {
            position_body_m,
            orientation_body_from_prop,
            diameter_m,
            spin_direction,
        })
    }

    #[must_use]
    pub const fn position_body_m(&self) -> &Vec3 {
        &self.position_body_m
    }

    #[must_use]
    pub const fn orientation_body_from_prop(&self) -> &Orientation {
        &self.orientation_body_from_prop
    }

    #[must_use]
    pub const fn diameter_m(&self) -> f64 {
        self.diameter_m
    }

    #[must_use]
    pub const fn spin_direction(&self) -> PropellerSpinDirection {
        self.spin_direction
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerSample {
    pub advance_ratio_j: f64,
    pub ct: f64,
    pub cq: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerCoefficients {
    pub ct: f64,
    pub cq: f64,
}

impl From<PropellerSample> for PropellerCoefficients {
    fn from(sample: PropellerSample) -> Self {
        Self {
            ct: sample.ct,
            cq: sample.cq,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PropellerCoefficientTable {
    samples: Vec<PropellerSample>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PropellerCoefficientError {
    #[error("propeller coefficient table requires at least two samples")]
    TooFewSamples,
    #[error("propeller sample {index} contains a non-finite value")]
    NonFiniteSample { index: usize },
    #[error("propeller advance ratio must be strictly increasing at sample {index}")]
    NonIncreasingAdvanceRatio { index: usize },
    #[error("propeller thrust coefficient must be non-negative at sample {index}")]
    NegativeThrustCoefficient { index: usize },
    #[error("propeller torque coefficient must be non-negative at sample {index}")]
    NegativeTorqueCoefficient { index: usize },
}

impl PropellerCoefficientTable {
    pub fn new(samples: Vec<PropellerSample>) -> Result<Self, PropellerCoefficientError> {
        if samples.len() < 2 {
            return Err(PropellerCoefficientError::TooFewSamples);
        }
        for (index, sample) in samples.iter().enumerate() {
            if ![sample.advance_ratio_j, sample.ct, sample.cq]
                .into_iter()
                .all(f64::is_finite)
            {
                return Err(PropellerCoefficientError::NonFiniteSample { index });
            }
            if sample.ct < 0.0 {
                return Err(PropellerCoefficientError::NegativeThrustCoefficient { index });
            }
            if sample.cq < 0.0 {
                return Err(PropellerCoefficientError::NegativeTorqueCoefficient { index });
            }
            if index > 0 && sample.advance_ratio_j <= samples[index - 1].advance_ratio_j {
                return Err(PropellerCoefficientError::NonIncreasingAdvanceRatio { index });
            }
        }
        Ok(Self { samples })
    }

    /// Deterministic piecewise-linear interpolation with exact samples and endpoint clamping.
    #[must_use]
    pub fn sample_clamped(&self, advance_ratio_j: f64) -> PropellerCoefficients {
        debug_assert!(advance_ratio_j.is_finite());
        let first = self.samples[0];
        if advance_ratio_j <= first.advance_ratio_j {
            return first.into();
        }
        let last = self.samples[self.samples.len() - 1];
        if advance_ratio_j >= last.advance_ratio_j {
            return last.into();
        }

        let mut lower = 0;
        let mut upper = self.samples.len() - 1;
        while upper - lower > 1 {
            let middle = lower + (upper - lower) / 2;
            if advance_ratio_j < self.samples[middle].advance_ratio_j {
                upper = middle;
            } else {
                lower = middle;
            }
        }

        let lower_sample = self.samples[lower];
        if advance_ratio_j == lower_sample.advance_ratio_j {
            return lower_sample.into();
        }
        let upper_sample = self.samples[upper];
        if advance_ratio_j == upper_sample.advance_ratio_j {
            return upper_sample.into();
        }
        let fraction = (advance_ratio_j - lower_sample.advance_ratio_j)
            / (upper_sample.advance_ratio_j - lower_sample.advance_ratio_j);
        PropellerCoefficients {
            ct: lower_sample.ct + fraction * (upper_sample.ct - lower_sample.ct),
            cq: lower_sample.cq + fraction * (upper_sample.cq - lower_sample.cq),
        }
    }

    #[must_use]
    pub fn samples(&self) -> &[PropellerSample] {
        &self.samples
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PropellerCoefficientNode {
    shaft_speed_rad_s: f64,
    table: PropellerCoefficientTable,
}

impl PropellerCoefficientNode {
    pub fn new(
        shaft_speed_rad_s: f64,
        table: PropellerCoefficientTable,
    ) -> Result<Self, PropellerCoefficientMapError> {
        if !shaft_speed_rad_s.is_finite() {
            return Err(PropellerCoefficientMapError::NonFiniteShaftSpeed);
        }
        if shaft_speed_rad_s <= 0.0 {
            return Err(PropellerCoefficientMapError::NonPositiveShaftSpeed);
        }
        Ok(Self {
            shaft_speed_rad_s,
            table,
        })
    }

    #[must_use]
    pub const fn shaft_speed_rad_s(&self) -> f64 {
        self.shaft_speed_rad_s
    }

    #[must_use]
    pub const fn table(&self) -> &PropellerCoefficientTable {
        &self.table
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PropellerCoefficientMapError {
    #[error("propeller coefficient map requires at least one node")]
    Empty,
    #[error("propeller map shaft speed must be finite")]
    NonFiniteShaftSpeed,
    #[error("propeller map shaft speed must be greater than zero")]
    NonPositiveShaftSpeed,
    #[error("propeller map shaft speeds must be strictly increasing at node {index}")]
    NonIncreasingShaftSpeed { index: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShaftSpeedRangeStatus {
    BelowRange,
    ExactOrInRange,
    AboveRange,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropellerCoefficientMapSample {
    pub coefficients: PropellerCoefficients,
    pub lower_shaft_speed_rad_s: f64,
    pub upper_shaft_speed_rad_s: f64,
    pub interpolation_fraction: f64,
    pub range_status: ShaftSpeedRangeStatus,
}

#[derive(Debug, Clone, PartialEq)]
pub struct PropellerCoefficientMap {
    nodes: Vec<PropellerCoefficientNode>,
}

impl PropellerCoefficientMap {
    pub fn new(nodes: Vec<PropellerCoefficientNode>) -> Result<Self, PropellerCoefficientMapError> {
        if nodes.is_empty() {
            return Err(PropellerCoefficientMapError::Empty);
        }
        for index in 1..nodes.len() {
            if nodes[index].shaft_speed_rad_s <= nodes[index - 1].shaft_speed_rad_s {
                return Err(PropellerCoefficientMapError::NonIncreasingShaftSpeed { index });
            }
        }
        Ok(Self { nodes })
    }

    #[must_use]
    pub fn nodes(&self) -> &[PropellerCoefficientNode] {
        &self.nodes
    }

    /// Samples both bracketing tables at the same advance ratio, then interpolates linearly in
    /// shaft speed. Requests outside the node range clamp to an endpoint without extrapolation.
    #[must_use]
    pub fn sample(
        &self,
        shaft_speed_rad_s: f64,
        advance_ratio_j: f64,
    ) -> PropellerCoefficientMapSample {
        debug_assert!(shaft_speed_rad_s.is_finite() && shaft_speed_rad_s >= 0.0);
        debug_assert!(advance_ratio_j.is_finite());
        if self.nodes.len() == 1 {
            return self.sample_single(0, advance_ratio_j, ShaftSpeedRangeStatus::ExactOrInRange);
        }
        match self
            .nodes
            .binary_search_by(|node| node.shaft_speed_rad_s.total_cmp(&shaft_speed_rad_s))
        {
            Ok(index) => self.sample_single(
                index,
                advance_ratio_j,
                ShaftSpeedRangeStatus::ExactOrInRange,
            ),
            Err(0) => self.sample_single(0, advance_ratio_j, ShaftSpeedRangeStatus::BelowRange),
            Err(upper) if upper == self.nodes.len() => self.sample_single(
                self.nodes.len() - 1,
                advance_ratio_j,
                ShaftSpeedRangeStatus::AboveRange,
            ),
            Err(upper) => {
                let lower = upper - 1;
                let lower_node = &self.nodes[lower];
                let upper_node = &self.nodes[upper];
                let fraction = (shaft_speed_rad_s - lower_node.shaft_speed_rad_s)
                    / (upper_node.shaft_speed_rad_s - lower_node.shaft_speed_rad_s);
                let lower_coefficients = lower_node.table.sample_clamped(advance_ratio_j);
                let upper_coefficients = upper_node.table.sample_clamped(advance_ratio_j);
                PropellerCoefficientMapSample {
                    coefficients: PropellerCoefficients {
                        ct: lower_coefficients.ct
                            + fraction * (upper_coefficients.ct - lower_coefficients.ct),
                        cq: lower_coefficients.cq
                            + fraction * (upper_coefficients.cq - lower_coefficients.cq),
                    },
                    lower_shaft_speed_rad_s: lower_node.shaft_speed_rad_s,
                    upper_shaft_speed_rad_s: upper_node.shaft_speed_rad_s,
                    interpolation_fraction: fraction,
                    range_status: ShaftSpeedRangeStatus::ExactOrInRange,
                }
            }
        }
    }

    fn sample_single(
        &self,
        index: usize,
        advance_ratio_j: f64,
        range_status: ShaftSpeedRangeStatus,
    ) -> PropellerCoefficientMapSample {
        let node = &self.nodes[index];
        PropellerCoefficientMapSample {
            coefficients: node.table.sample_clamped(advance_ratio_j),
            lower_shaft_speed_rad_s: node.shaft_speed_rad_s,
            upper_shaft_speed_rad_s: node.shaft_speed_rad_s,
            interpolation_fraction: 0.0,
            range_status,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PropellerCoefficientSource {
    FixedTable(PropellerCoefficientTable),
    ShaftSpeedMap(PropellerCoefficientMap),
}

impl PropellerCoefficientSource {
    fn as_ref(&self) -> PropellerCoefficientSourceRef<'_> {
        match self {
            Self::FixedTable(table) => PropellerCoefficientSourceRef::FixedTable(table),
            Self::ShaftSpeedMap(map) => PropellerCoefficientSourceRef::ShaftSpeedMap(map),
        }
    }
}

#[derive(Clone, Copy)]
enum PropellerCoefficientSourceRef<'a> {
    FixedTable(&'a PropellerCoefficientTable),
    ShaftSpeedMap(&'a PropellerCoefficientMap),
}

impl PropellerCoefficientSourceRef<'_> {
    fn sample(
        &self,
        shaft_speed_rad_s: f64,
        advance_ratio_j: f64,
    ) -> PropellerCoefficientMapSample {
        match self {
            Self::FixedTable(table) => PropellerCoefficientMapSample {
                coefficients: table.sample_clamped(advance_ratio_j),
                lower_shaft_speed_rad_s: shaft_speed_rad_s,
                upper_shaft_speed_rad_s: shaft_speed_rad_s,
                interpolation_fraction: 0.0,
                range_status: ShaftSpeedRangeStatus::ExactOrInRange,
            },
            Self::ShaftSpeedMap(map) => map.sample(shaft_speed_rad_s, advance_ratio_j),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricPropulsionConfig {
    battery: BatteryConfig,
    esc: EscConfig,
    motor: MotorConfig,
    propeller: PropellerConfig,
}

impl ElectricPropulsionConfig {
    #[must_use]
    pub const fn new(
        battery: BatteryConfig,
        motor: MotorConfig,
        propeller: PropellerConfig,
    ) -> Self {
        Self {
            battery,
            esc: EscConfig::ideal(),
            motor,
            propeller,
        }
    }

    #[must_use]
    pub const fn new_with_esc(
        battery: BatteryConfig,
        esc: EscConfig,
        motor: MotorConfig,
        propeller: PropellerConfig,
    ) -> Self {
        Self {
            battery,
            esc,
            motor,
            propeller,
        }
    }

    #[must_use]
    pub const fn battery(&self) -> &BatteryConfig {
        &self.battery
    }

    #[must_use]
    pub const fn esc(&self) -> &EscConfig {
        &self.esc
    }

    #[must_use]
    pub const fn motor(&self) -> &MotorConfig {
        &self.motor
    }

    #[must_use]
    pub const fn propeller(&self) -> &PropellerConfig {
        &self.propeller
    }
}

/// Electrical operating point at a known shaft speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElectricalDriveOutput {
    pub battery_terminal_voltage_v: f64,
    pub battery_current_a: f64,
    pub battery_terminal_electrical_power_w: f64,
    pub esc_loss_power_w: f64,
    pub motor_voltage_v: f64,
    pub motor_current_a: f64,
    pub motor_electrical_input_power_w: f64,
    pub motor_torque_nm: f64,
}

/// Analytic one-quadrant battery/ESC/motor evaluation at a known shaft speed.
#[must_use]
pub fn evaluate_electrical_drive(
    throttle: f64,
    shaft_speed_rad_s: f64,
    battery: &BatteryConfig,
    motor: &MotorConfig,
) -> ElectricalDriveOutput {
    evaluate_electrical_drive_with_esc(
        throttle,
        shaft_speed_rad_s,
        battery,
        &EscConfig::ideal(),
        motor,
    )
}

/// Analytic one-quadrant battery/series-loss ESC/motor evaluation at a known shaft speed.
#[must_use]
pub fn evaluate_electrical_drive_with_esc(
    throttle: f64,
    shaft_speed_rad_s: f64,
    battery: &BatteryConfig,
    esc: &EscConfig,
    motor: &MotorConfig,
) -> ElectricalDriveOutput {
    debug_assert!(throttle.is_finite() && (0.0..=1.0).contains(&throttle));
    debug_assert!(shaft_speed_rad_s.is_finite() && shaft_speed_rad_s >= 0.0);

    let denominator_ohm = motor.winding_resistance_ohm
        + esc.series_resistance_ohm
        + throttle * throttle * battery.internal_resistance_ohm;
    let motor_current_raw_a = (throttle * battery.open_circuit_voltage_v
        - motor.back_emf_constant_v_per_rad_s * shaft_speed_rad_s)
        / denominator_ohm;
    let motor_current_a = motor_current_raw_a.max(0.0);
    let battery_current_a = throttle * motor_current_a;
    let battery_terminal_voltage_v =
        battery.open_circuit_voltage_v - battery_current_a * battery.internal_resistance_ohm;
    let esc_output_ideal_voltage_v = throttle * battery_terminal_voltage_v;
    let motor_voltage_v = esc_output_ideal_voltage_v - motor_current_a * esc.series_resistance_ohm;
    let battery_terminal_electrical_power_w = battery_terminal_voltage_v * battery_current_a;
    let esc_loss_power_w = motor_current_a * motor_current_a * esc.series_resistance_ohm;
    let motor_electrical_input_power_w = motor_voltage_v * motor_current_a;
    let motor_torque_nm =
        motor.torque_constant_nm_per_a * (motor_current_a - motor.no_load_current_a).max(0.0);

    ElectricalDriveOutput {
        battery_terminal_voltage_v,
        battery_current_a,
        battery_terminal_electrical_power_w,
        esc_loss_power_w,
        motor_voltage_v,
        motor_current_a,
        motor_electrical_input_power_w,
        motor_torque_nm,
    }
}

#[derive(Debug, Clone, Copy)]
struct PropellerLoad {
    advance_ratio_j: f64,
    coefficients: PropellerCoefficients,
    coefficient_map_sample: PropellerCoefficientMapSample,
    load_torque_nm: f64,
    thrust_n: f64,
}

fn evaluate_propeller_load(
    shaft_speed_rad_s: f64,
    axial_airspeed_mps: f64,
    air_density_kg_m3: f64,
    propeller: &PropellerConfig,
    source: PropellerCoefficientSourceRef<'_>,
) -> PropellerLoad {
    if shaft_speed_rad_s <= MIN_SHAFT_SPEED_RAD_S {
        let coefficient_map_sample = source.sample(shaft_speed_rad_s, 0.0);
        return PropellerLoad {
            advance_ratio_j: 0.0,
            coefficients: coefficient_map_sample.coefficients,
            coefficient_map_sample,
            load_torque_nm: 0.0,
            thrust_n: 0.0,
        };
    }

    let revolutions_per_s = shaft_speed_rad_s / TAU;
    let advance_denominator_mps = revolutions_per_s * propeller.diameter_m;
    let raw_advance_ratio = axial_airspeed_mps / advance_denominator_mps;
    let advance_ratio_j = if raw_advance_ratio.is_finite() {
        raw_advance_ratio
    } else if axial_airspeed_mps > 0.0 {
        f64::MAX
    } else if axial_airspeed_mps < 0.0 {
        f64::MIN
    } else {
        0.0
    };
    let coefficient_map_sample = source.sample(shaft_speed_rad_s, advance_ratio_j);
    let coefficients = coefficient_map_sample.coefficients;
    let revolutions_per_s_squared = revolutions_per_s * revolutions_per_s;
    let diameter_squared_m2 = propeller.diameter_m * propeller.diameter_m;
    let diameter_fourth_m4 = diameter_squared_m2 * diameter_squared_m2;
    let thrust_n =
        coefficients.ct * air_density_kg_m3 * revolutions_per_s_squared * diameter_fourth_m4;
    let load_torque_nm = coefficients.cq
        * air_density_kg_m3
        * revolutions_per_s_squared
        * diameter_fourth_m4
        * propeller.diameter_m;
    PropellerLoad {
        advance_ratio_j,
        coefficients,
        coefficient_map_sample,
        load_torque_nm,
        thrust_n,
    }
}

fn torque_residual_nm(
    shaft_speed_rad_s: f64,
    throttle: f64,
    axial_airspeed_mps: f64,
    air_density_kg_m3: f64,
    config: &ElectricPropulsionConfig,
    source: PropellerCoefficientSourceRef<'_>,
) -> f64 {
    let electrical = evaluate_electrical_drive_with_esc(
        throttle,
        shaft_speed_rad_s,
        &config.battery,
        &config.esc,
        &config.motor,
    );
    let propeller = evaluate_propeller_load(
        shaft_speed_rad_s,
        axial_airspeed_mps,
        air_density_kg_m3,
        &config.propeller,
        source,
    );
    electrical.motor_torque_nm - propeller.load_torque_nm
}

/// Solves the quasi-static motor/propeller equilibrium using exactly 48 bisection iterations.
#[must_use]
pub fn solve_quasi_static_shaft_speed(
    throttle: f64,
    axial_airspeed_mps: f64,
    air_density_kg_m3: f64,
    config: &ElectricPropulsionConfig,
    table: &PropellerCoefficientTable,
) -> f64 {
    solve_quasi_static_shaft_speed_impl(
        throttle,
        axial_airspeed_mps,
        air_density_kg_m3,
        config,
        PropellerCoefficientSourceRef::FixedTable(table),
    )
}

/// Solves with either a fixed table or a shaft-speed map, sampling the candidate speed inside
/// every residual evaluation.
#[must_use]
pub fn solve_quasi_static_shaft_speed_with_source(
    throttle: f64,
    axial_airspeed_mps: f64,
    air_density_kg_m3: f64,
    config: &ElectricPropulsionConfig,
    source: &PropellerCoefficientSource,
) -> f64 {
    solve_quasi_static_shaft_speed_impl(
        throttle,
        axial_airspeed_mps,
        air_density_kg_m3,
        config,
        source.as_ref(),
    )
}

fn solve_quasi_static_shaft_speed_impl(
    throttle: f64,
    axial_airspeed_mps: f64,
    air_density_kg_m3: f64,
    config: &ElectricPropulsionConfig,
    source: PropellerCoefficientSourceRef<'_>,
) -> f64 {
    debug_assert!(throttle.is_finite() && (0.0..=1.0).contains(&throttle));
    debug_assert!(axial_airspeed_mps.is_finite());
    debug_assert!(air_density_kg_m3.is_finite() && air_density_kg_m3 >= 0.0);
    if throttle == 0.0 {
        return 0.0;
    }

    let mut lower_rad_s = 0.0;
    let mut upper_rad_s = throttle * config.battery.open_circuit_voltage_v
        / config.motor.back_emf_constant_v_per_rad_s;
    if torque_residual_nm(
        lower_rad_s,
        throttle,
        axial_airspeed_mps,
        air_density_kg_m3,
        config,
        source,
    ) <= 0.0
    {
        return 0.0;
    }

    for _ in 0..PROPULSION_BISECTION_ITERATIONS {
        let midpoint_rad_s = lower_rad_s + 0.5 * (upper_rad_s - lower_rad_s);
        if torque_residual_nm(
            midpoint_rad_s,
            throttle,
            axial_airspeed_mps,
            air_density_kg_m3,
            config,
            source,
        ) > 0.0
        {
            lower_rad_s = midpoint_rad_s;
        } else {
            upper_rad_s = midpoint_rad_s;
        }
    }
    lower_rad_s + 0.5 * (upper_rad_s - lower_rad_s)
}

/// Complete by-value output of one quasi-static propulsion evaluation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropulsionOutput {
    pub throttle: f64,
    pub air_relative_velocity_prop_mps: Vec3,
    pub axial_airspeed_mps: f64,
    pub battery_terminal_voltage_v: f64,
    pub battery_current_a: f64,
    pub battery_terminal_electrical_power_w: f64,
    pub esc_loss_power_w: f64,
    pub motor_voltage_v: f64,
    pub motor_current_a: f64,
    pub motor_electrical_input_power_w: f64,
    pub shaft_speed_rad_s: f64,
    pub shaft_speed_rpm: f64,
    pub motor_torque_nm: f64,
    pub advance_ratio_j: f64,
    pub coefficients: PropellerCoefficients,
    pub coefficient_map_sample: PropellerCoefficientMapSample,
    pub propeller_load_torque_nm: f64,
    pub thrust_n: f64,
    pub force_prop_n: Vec3,
    pub wrench_body: BodyWrench,
}

/// Evaluates one battery/ESC/motor/propeller assembly from the current RK4 stage state.
#[must_use]
pub fn evaluate_electric_propulsion(
    state: &RigidBodyState,
    throttle: f64,
    config: &ElectricPropulsionConfig,
    environment: &AeroEnvironment,
    table: &PropellerCoefficientTable,
) -> PropulsionOutput {
    evaluate_electric_propulsion_impl(
        state,
        throttle,
        config,
        environment,
        PropellerCoefficientSourceRef::FixedTable(table),
    )
}

/// Evaluates one assembly with a fixed table or shaft-speed coefficient map.
#[must_use]
pub fn evaluate_electric_propulsion_with_source(
    state: &RigidBodyState,
    throttle: f64,
    config: &ElectricPropulsionConfig,
    environment: &AeroEnvironment,
    source: &PropellerCoefficientSource,
) -> PropulsionOutput {
    evaluate_electric_propulsion_impl(state, throttle, config, environment, source.as_ref())
}

fn evaluate_electric_propulsion_impl(
    state: &RigidBodyState,
    throttle: f64,
    config: &ElectricPropulsionConfig,
    environment: &AeroEnvironment,
    source: PropellerCoefficientSourceRef<'_>,
) -> PropulsionOutput {
    debug_assert!(state.validate().is_ok());
    debug_assert!(throttle.is_finite() && (0.0..=1.0).contains(&throttle));

    let air_relative_velocity_world_mps =
        state.linear_velocity_world_mps - environment.wind_velocity_world_mps();
    let air_relative_velocity_body_at_cg_mps = world_to_body(
        &state.orientation_world_from_body,
        &air_relative_velocity_world_mps,
    );
    let rotational_velocity_body_mps = state
        .angular_velocity_body_radps
        .cross(&config.propeller.position_body_m);
    let air_relative_velocity_body_at_prop_mps =
        air_relative_velocity_body_at_cg_mps + rotational_velocity_body_mps;
    let air_relative_velocity_prop_mps = config
        .propeller
        .orientation_body_from_prop
        .inverse_transform_vector(&air_relative_velocity_body_at_prop_mps);
    let axial_airspeed_mps = air_relative_velocity_prop_mps.x;

    let shaft_speed_rad_s = solve_quasi_static_shaft_speed_impl(
        throttle,
        axial_airspeed_mps,
        environment.air_density_kg_m3(),
        config,
        source,
    );
    let electrical = evaluate_electrical_drive_with_esc(
        throttle,
        shaft_speed_rad_s,
        &config.battery,
        &config.esc,
        &config.motor,
    );
    let propeller = evaluate_propeller_load(
        shaft_speed_rad_s,
        axial_airspeed_mps,
        environment.air_density_kg_m3(),
        &config.propeller,
        source,
    );

    let force_prop_n = Vec3::new(propeller.thrust_n, 0.0, 0.0);
    let force_body_n = config
        .propeller
        .orientation_body_from_prop
        .transform_vector(&force_prop_n);
    let reaction_moment_prop_nm = Vec3::new(
        -config.propeller.spin_direction.sign() * propeller.load_torque_nm,
        0.0,
        0.0,
    );
    let reaction_moment_body_nm = config
        .propeller
        .orientation_body_from_prop
        .transform_vector(&reaction_moment_prop_nm);
    let mut wrench_body = BodyWrench::zero();
    wrench_body.add_force_at_body_point(force_body_n, config.propeller.position_body_m);
    wrench_body.add_moment_body(reaction_moment_body_nm);

    PropulsionOutput {
        throttle,
        air_relative_velocity_prop_mps,
        axial_airspeed_mps,
        battery_terminal_voltage_v: electrical.battery_terminal_voltage_v,
        battery_current_a: electrical.battery_current_a,
        battery_terminal_electrical_power_w: electrical.battery_terminal_electrical_power_w,
        esc_loss_power_w: electrical.esc_loss_power_w,
        motor_voltage_v: electrical.motor_voltage_v,
        motor_current_a: electrical.motor_current_a,
        motor_electrical_input_power_w: electrical.motor_electrical_input_power_w,
        shaft_speed_rad_s,
        shaft_speed_rpm: shaft_speed_rad_s * 60.0 / TAU,
        motor_torque_nm: electrical.motor_torque_nm,
        advance_ratio_j: propeller.advance_ratio_j,
        coefficients: propeller.coefficients,
        coefficient_map_sample: propeller.coefficient_map_sample,
        propeller_load_torque_nm: propeller.load_torque_nm,
        thrust_n: propeller.thrust_n,
        force_prop_n,
        wrench_body,
    }
}
