use crate::InputError;
use sim_core::PilotInput;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerAxes {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub throttle: f64,
}

impl ControllerAxes {
    #[must_use]
    pub const fn new(roll: f64, pitch: f64, yaw: f64, throttle: f64) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisMapping {
    deadzone: f64,
    inverted: bool,
}

impl AxisMapping {
    pub fn new(deadzone: f64, inverted: bool) -> Result<Self, InputError> {
        validate_deadzone(deadzone)?;
        Ok(Self { deadzone, inverted })
    }

    #[must_use]
    pub const fn deadzone(&self) -> f64 {
        self.deadzone
    }

    #[must_use]
    pub const fn inverted(&self) -> bool {
        self.inverted
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InputMapping {
    roll: AxisMapping,
    pitch: AxisMapping,
    yaw: AxisMapping,
    throttle: AxisMapping,
}

impl InputMapping {
    #[must_use]
    pub const fn new(
        roll: AxisMapping,
        pitch: AxisMapping,
        yaw: AxisMapping,
        throttle: AxisMapping,
    ) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            throttle,
        }
    }

    pub fn map_axes(&self, axes: ControllerAxes) -> Result<PilotInput, InputError> {
        let roll = normalize_centered_axis(axes.roll, self.roll.deadzone, self.roll.inverted)
            .map_err(|_| InputError::NonFiniteRawAxis { axis: "roll" })?;
        let pitch = normalize_centered_axis(axes.pitch, self.pitch.deadzone, self.pitch.inverted)
            .map_err(|_| InputError::NonFiniteRawAxis { axis: "pitch" })?;
        let yaw = normalize_centered_axis(axes.yaw, self.yaw.deadzone, self.yaw.inverted)
            .map_err(|_| InputError::NonFiniteRawAxis { axis: "yaw" })?;
        let throttle = normalize_throttle_axis(
            axes.throttle,
            self.throttle.deadzone,
            self.throttle.inverted,
        )
        .map_err(|_| InputError::NonFiniteRawAxis { axis: "throttle" })?;
        Ok(PilotInput::new(roll, pitch, yaw, throttle))
    }
}

impl Default for InputMapping {
    fn default() -> Self {
        Self::new(
            AxisMapping::new(0.08, false).expect("the fixed roll mapping is valid"),
            AxisMapping::new(0.08, true).expect("the fixed pitch mapping is valid"),
            AxisMapping::new(0.08, false).expect("the fixed yaw mapping is valid"),
            AxisMapping::new(0.0, true).expect("the fixed throttle mapping is valid"),
        )
    }
}

/// Continuous centered-axis deadzone with rescaling to the full normalized range.
pub fn normalize_centered_axis(raw: f64, deadzone: f64, inverted: bool) -> Result<f64, InputError> {
    validate_deadzone(deadzone)?;
    if !raw.is_finite() {
        return Err(InputError::NonFiniteRawAxis { axis: "centered" });
    }
    let clamped = raw.clamp(-1.0, 1.0);
    let directed = if inverted { -clamped } else { clamped };
    let magnitude = directed.abs();
    if magnitude <= deadzone {
        return Ok(0.0);
    }
    let rescaled = (magnitude - deadzone) / (1.0 - deadzone);
    Ok(directed.signum() * rescaled.clamp(0.0, 1.0))
}

/// Maps a typical `[-1, 1]` hardware axis into the simulator throttle `[0, 1]`.
pub fn normalize_throttle_axis(raw: f64, deadzone: f64, inverted: bool) -> Result<f64, InputError> {
    let centered = normalize_centered_axis(raw, deadzone, inverted)?;
    Ok(((centered + 1.0) * 0.5).clamp(0.0, 1.0))
}

fn validate_deadzone(deadzone: f64) -> Result<(), InputError> {
    if deadzone.is_finite() && (0.0..1.0).contains(&deadzone) {
        Ok(())
    } else {
        Err(InputError::InvalidDeadzone)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centered_axis_handles_zero_positive_negative_and_clamp() {
        assert_eq!(normalize_centered_axis(0.0, 0.1, false).unwrap(), 0.0);
        assert!(normalize_centered_axis(0.6, 0.1, false).unwrap() > 0.0);
        assert!(normalize_centered_axis(-0.6, 0.1, false).unwrap() < 0.0);
        assert_eq!(normalize_centered_axis(4.0, 0.0, false).unwrap(), 1.0);
        assert_eq!(normalize_centered_axis(-4.0, 0.0, false).unwrap(), -1.0);
    }

    #[test]
    fn deadzone_is_zero_inside_continuous_and_rescaled_outside() {
        let deadzone = 0.2;
        assert_eq!(normalize_centered_axis(0.2, deadzone, false).unwrap(), 0.0);
        let just_outside = normalize_centered_axis(0.2 + 1.0e-9, deadzone, false).unwrap();
        assert!(just_outside > 0.0 && just_outside < 2.0e-9);
        assert!((normalize_centered_axis(0.6, deadzone, false).unwrap() - 0.5).abs() < 1.0e-15);
        assert_eq!(normalize_centered_axis(1.0, deadzone, false).unwrap(), 1.0);
    }

    #[test]
    fn inversion_changes_sign_and_throttle_maps_endpoints() {
        assert_eq!(normalize_centered_axis(0.75, 0.0, true).unwrap(), -0.75);
        assert_eq!(normalize_throttle_axis(-1.0, 0.0, false).unwrap(), 0.0);
        assert_eq!(normalize_throttle_axis(1.0, 0.0, false).unwrap(), 1.0);
        assert_eq!(normalize_throttle_axis(-1.0, 0.0, true).unwrap(), 1.0);
        assert_eq!(normalize_throttle_axis(1.0, 0.0, true).unwrap(), 0.0);
    }

    #[test]
    fn mapping_is_deterministic_and_rejects_nonfinite_values() {
        let mapping = InputMapping::default();
        let axes = ControllerAxes::new(0.5, -0.25, 0.75, -0.5);
        assert_eq!(
            mapping.map_axes(axes).unwrap(),
            mapping.map_axes(axes).unwrap()
        );
        for axes in [
            ControllerAxes::new(f64::NAN, 0.0, 0.0, 0.0),
            ControllerAxes::new(0.0, f64::INFINITY, 0.0, 0.0),
            ControllerAxes::new(0.0, 0.0, f64::NEG_INFINITY, 0.0),
            ControllerAxes::new(0.0, 0.0, 0.0, f64::NAN),
        ] {
            assert!(mapping.map_axes(axes).is_err());
        }
        assert!(AxisMapping::new(1.0, false).is_err());
        assert!(AxisMapping::new(f64::NAN, false).is_err());
    }
}
