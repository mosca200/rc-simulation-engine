#![forbid(unsafe_code)]
//! Deterministic, headless rigid-body simulation core.

mod aero;
mod controls;
mod dynamics;
mod input;
mod integrator;
mod parameters;
mod propulsion;
mod reynolds_polar;
mod simulation;
mod snapshot;
mod state;

pub use aero::{
    AeroElement, AeroElementError, AeroElementOutput, AeroEnvironment, AeroEnvironmentError,
    MIN_SECTION_AIRSPEED_MPS, PolarCoefficients, PolarError, PolarSample, PolarTable,
    ReynoldsAeroElementOutput, ReynoldsCalculationError, calculate_reynolds_number,
    evaluate_aero_element, evaluate_reynolds_aero_element,
};
pub use controls::{
    AxisResponseConfig, ControlActuatorConfig, ControlActuatorState, ControlConfigError,
    ControlResponseConfig, ControlSurfacePositions, ControlSystemConfig, ControlSystemState,
    ControlTargets, ServoConfig, ServoState, ShapedPilotCommand, advance_controls, advance_servo,
    mix_conventional, shape_pilot_input,
};
pub use dynamics::{BodyWrench, RigidBodyDerivative, evaluate_derivative};
pub use input::PilotInput;
pub use integrator::Rk4Integrator;
pub use parameters::{ParameterError, RigidBodyParams};
pub use propulsion::{
    BatteryConfig, BatteryConfigError, ElectricPropulsionConfig, ElectricalDriveOutput,
    MIN_SHAFT_SPEED_RAD_S, MotorConfig, MotorConfigError, PROPULSION_BISECTION_ITERATIONS,
    PropellerCoefficientError, PropellerCoefficientTable, PropellerCoefficients, PropellerConfig,
    PropellerConfigError, PropellerSample, PropellerSpinDirection, PropulsionOutput,
    evaluate_electric_propulsion, evaluate_electrical_drive, solve_quasi_static_shaft_speed,
};
pub use reynolds_polar::{
    ReynoldsPolar, ReynoldsPolarFamily, ReynoldsPolarFamilyError, ReynoldsPolarSample,
    ReynoldsRangeStatus,
};
pub use simulation::{
    DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ, Simulation, SimulationConfig, SimulationConfigError,
    SimulationError,
};
pub use snapshot::{SimSnapshot, SnapshotBuffer, SnapshotBufferError};
pub use state::{RigidBodyState, StateError};
