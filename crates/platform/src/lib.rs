#![forbid(unsafe_code)]
//! Headless-capable hardware and keyboard input boundary for normalized pilot commands.

mod axes;
mod backend;
mod calibration;
mod identity;
mod mapping;
mod profile;
mod state;

pub use axes::{Control, HardwareAxis, RawControllerState};
pub use backend::{GilrsInputBackend, InputDeviceInfo};
pub use calibration::{CenteredCalibration, MIN_CALIBRATION_SPAN, ThrottleCalibration};
pub use identity::{DeviceIdentity, DeviceLink, DeviceLinkStatus, match_device};
pub use mapping::{
    AxisMapping, ControllerAxes, InputMapping, normalize_centered_axis, normalize_throttle_axis,
};
pub use profile::{
    CONTROLLER_PROFILE_SCHEMA_VERSION, CenteredAxisProfile, ControllerProfile, ProfileAxes,
    ThrottleAxisProfile,
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
    #[error("no input devices are connected")]
    NoDevices,
    #[error("requested controller device was not found among connected devices")]
    RequestedDeviceNotFound,
    #[error(
        "controller device match is ambiguous: {candidates} candidates match the requested identity"
    )]
    AmbiguousDeviceMatch { candidates: usize },
    #[error("controller profile is invalid: {0}")]
    InvalidControllerProfile(String),
    #[error("unsupported controller profile schema version {found} (supported: {supported})")]
    UnsupportedProfileVersion { found: u32, supported: u32 },
    #[error("{control} calibration contains non-finite values")]
    NonFiniteCalibration { control: Control },
    #[error("{control} calibration endpoints are out of order")]
    InvalidCalibrationOrder { control: Control },
    #[error("{control} calibration span is degenerate (must cover at least {min_span})")]
    DegenerateCalibrationSpan { control: Control, min_span: f64 },
    #[error("hardware axis {axis} is assigned to more than one control")]
    DuplicateAxisAssignment { axis: HardwareAxis },
    #[error("hardware axis {axis} is not available on the selected device")]
    UnavailableHardwareAxis { axis: HardwareAxis },
    #[error("unknown hardware axis identifier: {0}")]
    UnknownHardwareAxis(String),
}
