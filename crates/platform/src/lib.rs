#![forbid(unsafe_code)]
//! Headless-capable hardware and keyboard input boundary for normalized pilot commands.

mod backend;
mod mapping;
mod state;

pub use backend::{GilrsInputBackend, InputDeviceInfo};
pub use mapping::{
    AxisMapping, ControllerAxes, InputMapping, normalize_centered_axis, normalize_throttle_axis,
};
pub use state::{InputSource, InputState, KeyboardInputState, KeyboardKey};

use thiserror::Error;

#[derive(Debug, Clone, PartialEq, Error)]
pub enum InputError {
    #[error("input deadzone must be finite and inside [0, 1)")]
    InvalidDeadzone,
    #[error("raw {axis} axis value must be finite")]
    NonFiniteRawAxis { axis: &'static str },
    #[error("input sampling timestep must be finite and greater than zero")]
    InvalidSamplingTimestep,
    #[error("initial keyboard throttle must be finite and inside [0, 1]")]
    InvalidInitialThrottle,
    #[error("failed to initialize gilrs input backend: {0}")]
    BackendInitialization(String),
}
