#![forbid(unsafe_code)]
//! Deterministic, headless rigid-body simulation core.

mod aero;
mod dynamics;
mod input;
mod integrator;
mod parameters;
mod simulation;
mod snapshot;
mod state;

pub use aero::{
    AeroElement, AeroElementError, AeroElementOutput, AeroEnvironment, AeroEnvironmentError,
    MIN_SECTION_AIRSPEED_MPS, PolarCoefficients, PolarError, PolarSample, PolarTable,
    evaluate_aero_element,
};
pub use dynamics::{BodyWrench, RigidBodyDerivative, evaluate_derivative};
pub use input::PilotInput;
pub use integrator::Rk4Integrator;
pub use parameters::{ParameterError, RigidBodyParams};
pub use simulation::{
    DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ, Simulation, SimulationConfig, SimulationConfigError,
    SimulationError,
};
pub use snapshot::{SimSnapshot, SnapshotBuffer, SnapshotBufferError};
pub use state::{RigidBodyState, StateError};
