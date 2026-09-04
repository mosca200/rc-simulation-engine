use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize};
use sim_core::PilotInput;

use crate::{
    CenteredCalibration, Control, DeviceIdentity, HardwareAxis, InputError, RawControllerState,
    ThrottleCalibration,
};

/// Schema version persisted in controller profiles.
pub const CONTROLLER_PROFILE_SCHEMA_VERSION: u32 = 1;

/// One centered control assigned to a hardware axis with its calibration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
/// `from_json` is the only decode entry point and rejects unsupported schema
/// versions, invalid calibrations, and duplicate axis assignments with typed
/// errors. `to_json` renders the stable pretty-printed format consumed by the
/// application layer. The profile layer never reads or writes files; path
/// policy belongs to the application.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct ControllerProfile {
    #[serde(default)]
    schema_version: u32,
    device: DeviceIdentity,
    axes: ProfileAxes,
}

#[derive(Deserialize)]
struct UnvalidatedControllerProfile {
    #[serde(default)]
    schema_version: u32,
    device: DeviceIdentity,
    axes: ProfileAxes,
}

impl UnvalidatedControllerProfile {
    fn into_profile(self) -> ControllerProfile {
        ControllerProfile {
            schema_version: self.schema_version,
            device: self.device,
            axes: self.axes,
        }
    }
}

impl<'de> Deserialize<'de> for ControllerProfile {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let profile = UnvalidatedControllerProfile::deserialize(deserializer)?.into_profile();
        profile.validate().map_err(D::Error::custom)?;
        Ok(profile)
    }
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
    pub fn from_json(text: &str) -> Result<Self, InputError> {
        let profile = serde_json::from_str::<UnvalidatedControllerProfile>(text)
            .map_err(|error| InputError::InvalidControllerProfile(error.to_string()))?
            .into_profile();
        profile.validate()?;
        Ok(profile)
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
