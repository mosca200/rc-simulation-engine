use serde::{Deserialize, Serialize};
use sim_core::PilotInput;

use crate::{
    CenteredCalibration, Control, DeviceIdentity, HardwareAxis, InputError, RawControllerState,
    ThrottleCalibration,
};

/// Schema version persisted in controller profiles.
pub const CONTROLLER_PROFILE_SCHEMA_VERSION: u32 = 1;

/// One centered control assigned to a hardware axis with its calibration.
///
/// Decoding from JSON is intentionally unavailable on this type; profiles are
/// only decoded through [`ControllerProfile::from_json`], which enforces the
/// validation boundary.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct CenteredAxisProfile {
    source: HardwareAxis,
    #[serde(flatten)]
    calibration: CenteredCalibration,
}

impl CenteredAxisProfile {
    #[must_use]
    pub const fn new(source: HardwareAxis, calibration: CenteredCalibration) -> Self {
        Self {
            source,
            calibration,
        }
    }

    #[must_use]
    pub const fn source(&self) -> HardwareAxis {
        self.source
    }

    #[must_use]
    pub const fn calibration(&self) -> &CenteredCalibration {
        &self.calibration
    }
}

/// The throttle control assigned to a hardware axis with its endpoint calibration.
///
/// Decoding from JSON is intentionally unavailable on this type; profiles are
/// only decoded through [`ControllerProfile::from_json`], which enforces the
/// validation boundary.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ThrottleAxisProfile {
    source: HardwareAxis,
    #[serde(flatten)]
    calibration: ThrottleCalibration,
}

impl ThrottleAxisProfile {
    #[must_use]
    pub const fn new(source: HardwareAxis, calibration: ThrottleCalibration) -> Self {
        Self {
            source,
            calibration,
        }
    }

    #[must_use]
    pub const fn source(&self) -> HardwareAxis {
        self.source
    }

    #[must_use]
    pub const fn calibration(&self) -> &ThrottleCalibration {
        &self.calibration
    }
}

/// The complete hardware-axis assignment of one controller profile.
///
/// Every public construction path only accepts already-validated
/// calibrations. The type deliberately does not implement
/// `serde::Deserialize`, so an assignment can never be decoded bypassing
/// [`ControllerProfile::from_json`].
///
/// ```compile_fail
/// use platform::ProfileAxes;
///
/// let axes: ProfileAxes = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ProfileAxes {
    roll: CenteredAxisProfile,
    pitch: CenteredAxisProfile,
    yaw: CenteredAxisProfile,
    throttle: ThrottleAxisProfile,
}

impl ProfileAxes {
    #[must_use]
    pub const fn new(
        roll: CenteredAxisProfile,
        pitch: CenteredAxisProfile,
        yaw: CenteredAxisProfile,
        throttle: ThrottleAxisProfile,
    ) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }

    #[must_use]
    pub const fn roll(&self) -> &CenteredAxisProfile {
        &self.roll
    }

    #[must_use]
    pub const fn pitch(&self) -> &CenteredAxisProfile {
        &self.pitch
    }

    #[must_use]
    pub const fn yaw(&self) -> &CenteredAxisProfile {
        &self.yaw
    }

    #[must_use]
    pub const fn throttle(&self) -> &ThrottleAxisProfile {
        &self.throttle
    }

    /// Validates every calibration and rejects duplicate hardware-axis assignments.
    pub fn validate(&self) -> Result<(), InputError> {
        self.roll.calibration.validate(Control::Roll)?;
        self.pitch.calibration.validate(Control::Pitch)?;
        self.yaw.calibration.validate(Control::Yaw)?;
        self.throttle.calibration.validate()?;
        self.reject_duplicate_sources()
    }

    /// Converts raw device state into calibrated pilot input.
    ///
    /// A hardware axis assigned by the profile but missing from `state` is an
    /// error; missing axes are never treated as zero.
    pub fn to_pilot_input(&self, state: &RawControllerState) -> Result<PilotInput, InputError> {
        let roll = centered_value(state, &self.roll, Control::Roll)?;
        let pitch = centered_value(state, &self.pitch, Control::Pitch)?;
        let yaw = centered_value(state, &self.yaw, Control::Yaw)?;
        let raw_throttle =
            state
                .get(self.throttle.source())
                .ok_or(InputError::UnavailableHardwareAxis {
                    axis: self.throttle.source(),
                })?;
        let throttle = self.throttle.calibration.apply(raw_throttle)?;
        Ok(PilotInput::new(roll, pitch, yaw, throttle))
    }

    fn reject_duplicate_sources(&self) -> Result<(), InputError> {
        let assignments = [
            self.roll.source(),
            self.pitch.source(),
            self.yaw.source(),
            self.throttle.source(),
        ];
        for (index, &axis) in assignments.iter().enumerate() {
            if assignments[..index].contains(&axis) {
                return Err(InputError::DuplicateAxisAssignment { axis });
            }
        }
        Ok(())
    }
}

fn centered_value(
    state: &RawControllerState,
    profile: &CenteredAxisProfile,
    control: Control,
) -> Result<f64, InputError> {
    let raw = state
        .get(profile.source())
        .ok_or(InputError::UnavailableHardwareAxis {
            axis: profile.source(),
        })?;
    profile.calibration.apply(raw, control)
}

/// Versioned, JSON-serializable controller profile.
///
/// Every reachable instance is validated: [`Self::new`] validates the axis
/// assignment and [`Self::from_json`] is the only JSON decode entry point,
/// rejecting unsupported schema versions, invalid calibrations, and duplicate
/// axis assignments with typed errors. The type deliberately does not
/// implement `serde::Deserialize`, so decoding can never bypass the
/// validation boundary; decoding happens once at load time, never in the
/// frame loop. `to_json` renders the stable pretty-printed format consumed by
/// the application layer. The profile layer never reads or writes files; path
/// policy belongs to the application.
///
/// ```compile_fail
/// use platform::ControllerProfile;
///
/// let profile: ControllerProfile = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ControllerProfile {
    schema_version: u32,
    device: DeviceIdentity,
    axes: ProfileAxes,
}

impl ControllerProfile {
    /// Constructs a schema version 1 profile after validating the assignment.
    pub fn new(device: DeviceIdentity, axes: ProfileAxes) -> Result<Self, InputError> {
        axes.validate()?;
        Ok(Self {
            schema_version: CONTROLLER_PROFILE_SCHEMA_VERSION,
            device,
            axes,
        })
    }

    /// Validates schema version, calibrations, and axis assignments.
    pub fn validate(&self) -> Result<(), InputError> {
        if self.schema_version != CONTROLLER_PROFILE_SCHEMA_VERSION {
            return Err(InputError::UnsupportedProfileVersion {
                found: self.schema_version,
                supported: CONTROLLER_PROFILE_SCHEMA_VERSION,
            });
        }
        self.axes.validate()
    }

    /// Decodes and validates a profile from JSON text.
    ///
    /// This is the only JSON decode path: `ControllerProfile` does not
    /// implement `serde::Deserialize`, so decoding cannot bypass validation.
    /// Structural problems map to [`InputError::InvalidControllerProfile`],
    /// unsupported schema versions and invalid assignments to their typed
    /// variants.
    pub fn from_json(text: &str) -> Result<Self, InputError> {
        let wire: ProfileWire = serde_json::from_str(text)
            .map_err(|error| InputError::InvalidControllerProfile(error.to_string()))?;
        if wire.schema_version != CONTROLLER_PROFILE_SCHEMA_VERSION {
            return Err(InputError::UnsupportedProfileVersion {
                found: wire.schema_version,
                supported: CONTROLLER_PROFILE_SCHEMA_VERSION,
            });
        }
        let axes = ProfileAxes::new(
            wire.axes.roll.into_profile(Control::Roll)?,
            wire.axes.pitch.into_profile(Control::Pitch)?,
            wire.axes.yaw.into_profile(Control::Yaw)?,
            ThrottleAxisProfile::new(
                wire.axes.throttle.source,
                ThrottleCalibration::new(
                    wire.axes.throttle.raw_min,
                    wire.axes.throttle.raw_max,
                    wire.axes.throttle.inverted,
                )?,
            ),
        );
        Self::new(wire.device, axes)
    }

    /// Encodes the profile as stable pretty-printed JSON text.
    pub fn to_json(&self) -> Result<String, InputError> {
        serde_json::to_string_pretty(self)
            .map_err(|error| InputError::InvalidControllerProfile(error.to_string()))
    }

    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    #[must_use]
    pub const fn device(&self) -> &DeviceIdentity {
        &self.device
    }

    #[must_use]
    pub const fn axes(&self) -> &ProfileAxes {
        &self.axes
    }

    /// Converts raw device state into calibrated `PilotInput` using this profile.
    pub fn to_pilot_input(&self, state: &RawControllerState) -> Result<PilotInput, InputError> {
        self.axes.to_pilot_input(state)
    }
}

/// Private decode representation of the profile JSON schema.
///
/// The wire types mirror the schema field by field but carry no validation
/// invariants and are never exposed. They exist only so
/// [`ControllerProfile::from_json`] can decode text while keeping
/// `serde::Deserialize` off every public profile type.
#[derive(Deserialize)]
struct ProfileWire {
    #[serde(default)]
    schema_version: u32,
    device: DeviceIdentity,
    axes: AxesWire,
}

#[derive(Deserialize)]
struct AxesWire {
    roll: CenteredAxisWire,
    pitch: CenteredAxisWire,
    yaw: CenteredAxisWire,
    throttle: ThrottleAxisWire,
}

#[derive(Deserialize)]
struct CenteredAxisWire {
    source: HardwareAxis,
    raw_min: f64,
    raw_center: f64,
    raw_max: f64,
    inverted: bool,
    deadzone: f64,
}

impl CenteredAxisWire {
    fn into_profile(self, control: Control) -> Result<CenteredAxisProfile, InputError> {
        Ok(CenteredAxisProfile::new(
            self.source,
            CenteredCalibration::new(
                control,
                self.raw_min,
                self.raw_center,
                self.raw_max,
                self.inverted,
                self.deadzone,
            )?,
        ))
    }
}

#[derive(Deserialize)]
struct ThrottleAxisWire {
    source: HardwareAxis,
    raw_min: f64,
    raw_max: f64,
    inverted: bool,
}
