//! Deterministic pilot-command shaping, conventional mixing, and servo dynamics.

use crate::PilotInput;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum ControlConfigError {
    #[error("axis rate must be finite and in [0, 1]")]
    InvalidAxisRate,
    #[error("axis expo must be finite and in [0, 1]")]
    InvalidAxisExpo,
    #[error("servo configuration must contain only finite values")]
    NonFiniteServoConfig,
    #[error("servo travel must satisfy min < neutral < max")]
    InvalidServoTravel,
    #[error("servo maximum speed must be greater than zero")]
    InvalidServoSpeed,
    #[error("initial servo angle must be finite and within configured travel")]
    InvalidInitialServoAngle,
}

/// Per-axis rate and cubic-expo response, both normalized to `[0, 1]`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisResponseConfig {
    rate: f64,
    expo: f64,
}

impl AxisResponseConfig {
    pub fn new(rate: f64, expo: f64) -> Result<Self, ControlConfigError> {
        if !(0.0..=1.0).contains(&rate) {
            return Err(ControlConfigError::InvalidAxisRate);
        }
        if !(0.0..=1.0).contains(&expo) {
            return Err(ControlConfigError::InvalidAxisExpo);
        }
        Ok(Self { rate, expo })
    }

    #[must_use]
    pub const fn rate(&self) -> f64 {
        self.rate
    }

    #[must_use]
    pub const fn expo(&self) -> f64 {
        self.expo
    }

    /// Applies `rate * ((1 - expo) * x + expo * x^3)` to a normalized command.
    #[must_use]
    pub fn shape(&self, command: f64) -> f64 {
        debug_assert!(command.is_finite() && (-1.0..=1.0).contains(&command));
        self.rate * ((1.0 - self.expo) * command + self.expo * command * command * command)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlResponseConfig {
    roll: AxisResponseConfig,
    pitch: AxisResponseConfig,
    yaw: AxisResponseConfig,
}

impl ControlResponseConfig {
    #[must_use]
    pub const fn new(
        roll: AxisResponseConfig,
        pitch: AxisResponseConfig,
        yaw: AxisResponseConfig,
    ) -> Self {
        Self { roll, pitch, yaw }
    }

    #[must_use]
    pub const fn roll(&self) -> &AxisResponseConfig {
        &self.roll
    }

    #[must_use]
    pub const fn pitch(&self) -> &AxisResponseConfig {
        &self.pitch
    }

    #[must_use]
    pub const fn yaw(&self) -> &AxisResponseConfig {
        &self.yaw
    }
}

/// Validated, by-value output of rates/expo shaping.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShapedPilotCommand {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

impl ShapedPilotCommand {
    #[must_use]
    pub const fn roll(&self) -> f64 {
        self.roll
    }

    #[must_use]
    pub const fn pitch(&self) -> f64 {
        self.pitch
    }

    #[must_use]
    pub const fn yaw(&self) -> f64 {
        self.yaw
    }

    #[must_use]
    pub const fn throttle(&self) -> f64 {
        self.throttle
    }
}

#[must_use]
pub fn shape_pilot_input(input: &PilotInput, config: &ControlResponseConfig) -> ShapedPilotCommand {
    ShapedPilotCommand {
        roll: config.roll.shape(input.roll()),
        pitch: config.pitch.shape(input.pitch()),
        yaw: config.yaw.shape(input.yaw()),
        throttle: input.throttle(),
    }
}

/// Logical conventional-aircraft commands, independent of physical servo installation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlTargets {
    aileron: f64,
    elevator: f64,
    rudder: f64,
    throttle: f64,
}

impl ControlTargets {
    #[must_use]
    pub const fn aileron(&self) -> f64 {
        self.aileron
    }

    #[must_use]
    pub const fn elevator(&self) -> f64 {
        self.elevator
    }

    #[must_use]
    pub const fn rudder(&self) -> f64 {
        self.rudder
    }

    #[must_use]
    pub const fn throttle(&self) -> f64 {
        self.throttle
    }
}

/// Conventional fixed-wing mixer: roll/pitch/yaw map directly to aileron/elevator/rudder.
#[must_use]
pub const fn mix_conventional(command: &ShapedPilotCommand) -> ControlTargets {
    ControlTargets {
        aileron: command.roll,
        elevator: command.pitch,
        rudder: command.yaw,
        throttle: command.throttle,
    }
}

/// Physical installation and travel limits of one rotational servo.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoConfig {
    min_angle_rad: f64,
    neutral_angle_rad: f64,
    max_angle_rad: f64,
    max_speed_rad_s: f64,
    reversed: bool,
}

impl ServoConfig {
    pub fn new(
        min_angle_rad: f64,
        neutral_angle_rad: f64,
        max_angle_rad: f64,
        max_speed_rad_s: f64,
        reversed: bool,
    ) -> Result<Self, ControlConfigError> {
        if ![
            min_angle_rad,
            neutral_angle_rad,
            max_angle_rad,
            max_speed_rad_s,
        ]
        .into_iter()
        .all(f64::is_finite)
        {
            return Err(ControlConfigError::NonFiniteServoConfig);
        }
        if !(min_angle_rad < neutral_angle_rad && neutral_angle_rad < max_angle_rad) {
            return Err(ControlConfigError::InvalidServoTravel);
        }
        if max_speed_rad_s <= 0.0 {
            return Err(ControlConfigError::InvalidServoSpeed);
        }
        Ok(Self {
            min_angle_rad,
            neutral_angle_rad,
            max_angle_rad,
            max_speed_rad_s,
            reversed,
        })
    }

    #[must_use]
    pub const fn min_angle_rad(&self) -> f64 {
        self.min_angle_rad
    }

    #[must_use]
    pub const fn neutral_angle_rad(&self) -> f64 {
        self.neutral_angle_rad
    }

    #[must_use]
    pub const fn max_angle_rad(&self) -> f64 {
        self.max_angle_rad
    }

    #[must_use]
    pub const fn max_speed_rad_s(&self) -> f64 {
        self.max_speed_rad_s
    }

    #[must_use]
    pub const fn reversed(&self) -> bool {
        self.reversed
    }

    /// Maps a normalized semantic command to a physical angle with asymmetric travel support.
    #[must_use]
    pub fn target_angle_rad(&self, command: f64) -> f64 {
        debug_assert!(command.is_finite() && (-1.0..=1.0).contains(&command));
        let effective_command = if self.reversed { -command } else { command };
        if effective_command >= 0.0 {
            self.neutral_angle_rad
                + effective_command * (self.max_angle_rad - self.neutral_angle_rad)
        } else {
            self.neutral_angle_rad
                + (-effective_command) * (self.min_angle_rad - self.neutral_angle_rad)
        }
    }
}

/// Dynamic state of one servo, deliberately separate from `RigidBodyState`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ServoState {
    angle_rad: f64,
}

impl ServoState {
    #[must_use]
    pub const fn neutral(config: &ServoConfig) -> Self {
        Self {
            angle_rad: config.neutral_angle_rad,
        }
    }

    pub fn from_angle(config: &ServoConfig, angle_rad: f64) -> Result<Self, ControlConfigError> {
        if !angle_rad.is_finite()
            || !(config.min_angle_rad..=config.max_angle_rad).contains(&angle_rad)
        {
            return Err(ControlConfigError::InvalidInitialServoAngle);
        }
        Ok(Self { angle_rad })
    }

    #[must_use]
    pub const fn angle_rad(&self) -> f64 {
        self.angle_rad
    }
}

/// Advances one servo by a deterministic first-order rate limit and returns its new angle.
#[must_use]
pub fn advance_servo(state: &mut ServoState, config: &ServoConfig, command: f64, dt_s: f64) -> f64 {
    debug_assert!(dt_s.is_finite() && dt_s > 0.0);
    let target_angle_rad = config.target_angle_rad(command);
    let max_delta_rad = config.max_speed_rad_s * dt_s;
    let error_rad = target_angle_rad - state.angle_rad;
    state.angle_rad = if error_rad.abs() <= max_delta_rad {
        target_angle_rad
    } else {
        state.angle_rad + error_rad.signum() * max_delta_rad
    };
    state.angle_rad
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlActuatorConfig {
    aileron: ServoConfig,
    elevator: ServoConfig,
    rudder: ServoConfig,
}

impl ControlActuatorConfig {
    #[must_use]
    pub const fn new(aileron: ServoConfig, elevator: ServoConfig, rudder: ServoConfig) -> Self {
        Self {
            aileron,
            elevator,
            rudder,
        }
    }

    #[must_use]
    pub const fn aileron(&self) -> &ServoConfig {
        &self.aileron
    }

    #[must_use]
    pub const fn elevator(&self) -> &ServoConfig {
        &self.elevator
    }

    #[must_use]
    pub const fn rudder(&self) -> &ServoConfig {
        &self.rudder
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlActuatorState {
    aileron: ServoState,
    elevator: ServoState,
    rudder: ServoState,
}

impl ControlActuatorState {
    #[must_use]
    pub const fn neutral(config: &ControlActuatorConfig) -> Self {
        Self {
            aileron: ServoState::neutral(&config.aileron),
            elevator: ServoState::neutral(&config.elevator),
            rudder: ServoState::neutral(&config.rudder),
        }
    }

    #[must_use]
    pub const fn aileron(&self) -> &ServoState {
        &self.aileron
    }

    #[must_use]
    pub const fn elevator(&self) -> &ServoState {
        &self.elevator
    }

    #[must_use]
    pub const fn rudder(&self) -> &ServoState {
        &self.rudder
    }
}

/// Physical actuator angles plus the logical throttle command for future propulsion use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlSurfacePositions {
    aileron_angle_rad: f64,
    elevator_angle_rad: f64,
    rudder_angle_rad: f64,
    throttle: f64,
}

impl ControlSurfacePositions {
    #[must_use]
    pub const fn aileron_angle_rad(&self) -> f64 {
        self.aileron_angle_rad
    }

    #[must_use]
    pub const fn elevator_angle_rad(&self) -> f64 {
        self.elevator_angle_rad
    }

    #[must_use]
    pub const fn rudder_angle_rad(&self) -> f64 {
        self.rudder_angle_rad
    }

    #[must_use]
    pub const fn throttle(&self) -> f64 {
        self.throttle
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlSystemConfig {
    response: ControlResponseConfig,
    actuators: ControlActuatorConfig,
}

impl ControlSystemConfig {
    #[must_use]
    pub const fn new(response: ControlResponseConfig, actuators: ControlActuatorConfig) -> Self {
        Self {
            response,
            actuators,
        }
    }

    #[must_use]
    pub const fn response(&self) -> &ControlResponseConfig {
        &self.response
    }

    #[must_use]
    pub const fn actuators(&self) -> &ControlActuatorConfig {
        &self.actuators
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlSystemState {
    actuators: ControlActuatorState,
}

impl ControlSystemState {
    #[must_use]
    pub const fn neutral(config: &ControlSystemConfig) -> Self {
        Self {
            actuators: ControlActuatorState::neutral(&config.actuators),
        }
    }

    #[must_use]
    pub const fn actuators(&self) -> &ControlActuatorState {
        &self.actuators
    }
}

/// Runs shaping, conventional mixing, target mapping, and servo rate limiting once.
#[must_use]
pub fn advance_controls(
    state: &mut ControlSystemState,
    config: &ControlSystemConfig,
    input: &PilotInput,
    dt_s: f64,
) -> ControlSurfacePositions {
    let shaped = shape_pilot_input(input, &config.response);
    let targets = mix_conventional(&shaped);
    let aileron_angle_rad = advance_servo(
        &mut state.actuators.aileron,
        &config.actuators.aileron,
        targets.aileron,
        dt_s,
    );
    let elevator_angle_rad = advance_servo(
        &mut state.actuators.elevator,
        &config.actuators.elevator,
        targets.elevator,
        dt_s,
    );
    let rudder_angle_rad = advance_servo(
        &mut state.actuators.rudder,
        &config.actuators.rudder,
        targets.rudder,
        dt_s,
    );
    ControlSurfacePositions {
        aileron_angle_rad,
        elevator_angle_rad,
        rudder_angle_rad,
        throttle: targets.throttle,
    }
}

/// Evaluates the steady actuator positions for a constant pilot input.
///
/// This follows the same response shaping, conventional mixer, servo reversal, and asymmetric
/// travel mapping as [`advance_controls`], but places each actuator directly at its eventual
/// rate-limited target. It is intended for static equilibrium calculations, not time stepping.
#[must_use]
pub fn evaluate_steady_controls(
    config: &ControlSystemConfig,
    input: &PilotInput,
) -> ControlSurfacePositions {
    let shaped = shape_pilot_input(input, &config.response);
    let targets = mix_conventional(&shaped);
    ControlSurfacePositions {
        aileron_angle_rad: config.actuators.aileron.target_angle_rad(targets.aileron),
        elevator_angle_rad: config.actuators.elevator.target_angle_rad(targets.elevator),
        rudder_angle_rad: config.actuators.rudder.target_angle_rad(targets.rudder),
        throttle: targets.throttle,
    }
}
