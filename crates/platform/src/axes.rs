use std::collections::BTreeMap;
use std::fmt;
use std::str::FromStr;

use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::InputError;

/// The four normalized pilot control channels produced by the input layer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Control {
    Roll,
    Pitch,
    Yaw,
    Throttle,
}

impl Control {
    /// All controls in fixed channel order.
    pub const ALL: [Self; 4] = [Self::Roll, Self::Pitch, Self::Yaw, Self::Throttle];

    /// Stable lowercase label used in errors and profiles.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Roll => "roll",
            Self::Pitch => "pitch",
            Self::Yaw => "yaw",
            Self::Throttle => "throttle",
        }
    }
}

impl fmt::Display for Control {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.label())
    }
}

/// Stable hardware axis identifier used in controller profiles.
///
/// Each variant corresponds to one explicitly supported `gilrs::Axis` variant.
/// `gilrs::Axis::Unknown` is deliberately excluded and is never reported by the
/// backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum HardwareAxis {
    LeftStickX,
    LeftStickY,
    LeftZ,
    RightStickX,
    RightStickY,
    RightZ,
    DPadX,
    DPadY,
}

impl HardwareAxis {
    /// All supported hardware axes in a fixed deterministic order.
    pub const ALL: [Self; 8] = [
        Self::LeftStickX,
        Self::LeftStickY,
        Self::LeftZ,
        Self::RightStickX,
        Self::RightStickY,
        Self::RightZ,
        Self::DPadX,
        Self::DPadY,
    ];

    /// Stable serialized name used in controller profiles.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::LeftStickX => "left_stick_x",
            Self::LeftStickY => "left_stick_y",
            Self::LeftZ => "left_z",
            Self::RightStickX => "right_stick_x",
            Self::RightStickY => "right_stick_y",
            Self::RightZ => "right_z",
            Self::DPadX => "dpad_x",
            Self::DPadY => "dpad_y",
        }
    }
}

impl fmt::Display for HardwareAxis {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl FromStr for HardwareAxis {
    type Err = InputError;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text {
            "left_stick_x" => Ok(Self::LeftStickX),
            "left_stick_y" => Ok(Self::LeftStickY),
            "left_z" => Ok(Self::LeftZ),
            "right_stick_x" => Ok(Self::RightStickX),
            "right_stick_y" => Ok(Self::RightStickY),
            "right_z" => Ok(Self::RightZ),
            "dpad_x" => Ok(Self::DPadX),
            "dpad_y" => Ok(Self::DPadY),
            other => Err(InputError::UnknownHardwareAxis(other.to_owned())),
        }
    }
}

impl Serialize for HardwareAxis {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(self.as_str())
    }
}

impl<'de> Deserialize<'de> for HardwareAxis {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = String::deserialize(deserializer)?;
        text.parse().map_err(D::Error::custom)
    }
}

/// Raw axis values reported by one device snapshot.
///
/// Only axes that the device actually reports are present. Querying an axis
/// that is absent returns `None` so callers never consume an invented default
/// value.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RawControllerState {
    values: BTreeMap<HardwareAxis, f64>,
}

impl RawControllerState {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Inserts one raw axis value, rejecting non-finite values.
    pub fn insert(&mut self, axis: HardwareAxis, value: f64) -> Result<(), InputError> {
        if !value.is_finite() {
            return Err(InputError::NonFiniteRawAxis {
                axis: axis.as_str(),
            });
        }
        self.values.insert(axis, value);
        Ok(())
    }

    #[must_use]
    pub fn get(&self, axis: HardwareAxis) -> Option<f64> {
        self.values.get(&axis).copied()
    }

    #[must_use]
    pub fn contains(&self, axis: HardwareAxis) -> bool {
        self.values.contains_key(&axis)
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.values.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    /// All present axes in deterministic order.
    pub fn axes(&self) -> impl Iterator<Item = HardwareAxis> + '_ {
        self.values.keys().copied()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hardware_axis_names_round_trip_and_reject_unknown() {
        for axis in HardwareAxis::ALL {
            assert_eq!(axis.as_str().parse::<HardwareAxis>().unwrap(), axis);
        }
        assert_eq!(
            "stick_99".parse::<HardwareAxis>(),
            Err(InputError::UnknownHardwareAxis("stick_99".to_owned()))
        );
    }

    #[test]
    fn raw_state_rejects_non_finite_values_and_tracks_axes() {
        let mut state = RawControllerState::new();
        assert!(state.is_empty());
        state.insert(HardwareAxis::LeftStickX, 0.25).unwrap();
        state.insert(HardwareAxis::RightZ, -0.5).unwrap();
        assert_eq!(state.get(HardwareAxis::LeftStickX), Some(0.25));
        assert_eq!(state.get(HardwareAxis::DPadX), None);
        assert!(!state.contains(HardwareAxis::DPadX));
        assert_eq!(state.len(), 2);
        assert_eq!(
            state.axes().collect::<Vec<_>>(),
            vec![HardwareAxis::LeftStickX, HardwareAxis::RightZ]
        );
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(
                state.insert(HardwareAxis::DPadY, bad),
                Err(InputError::NonFiniteRawAxis { axis: "dpad_y" })
            );
        }
        assert_eq!(state.len(), 2);
    }
}
