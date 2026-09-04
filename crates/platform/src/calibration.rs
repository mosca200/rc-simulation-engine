use serde::Serialize;

use crate::{Control, InputError};

/// Minimum required span for a calibrated axis.
///
/// gilrs raw axis samples originate from f32 device values with a typical
/// integer resolution of about 1/32768, so a calibrated span smaller than this
/// is numerically degenerate and cannot represent usable travel.
pub const MIN_CALIBRATION_SPAN: f64 = 1.0e-6;

/// Deterministic calibration for a centered control axis.
///
/// Maps `raw_min` to -1, `raw_center` to 0, and `raw_max` to +1. The two half
/// spans are normalized independently, so asymmetric travel around the
/// physical center is supported.
///
/// Every instance is validated at construction. The type deliberately does not
/// implement `serde::Deserialize`, so unvalidated values can never reach
/// [`Self::apply`]; persisted calibrations are only decoded through
/// `ControllerProfile::from_json`.
///
/// ```compile_fail
/// use platform::CenteredCalibration;
///
/// let calibration: CenteredCalibration = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct CenteredCalibration {
    raw_min: f64,
    raw_center: f64,
    raw_max: f64,
    inverted: bool,
    deadzone: f64,
}

impl CenteredCalibration {
    /// Constructs and validates a calibration for one control channel.
    pub fn new(
        control: Control,
        raw_min: f64,
        raw_center: f64,
        raw_max: f64,
        inverted: bool,
        deadzone: f64,
    ) -> Result<Self, InputError> {
        let calibration = Self {
            raw_min,
            raw_center,
            raw_max,
            inverted,
            deadzone,
        };
        calibration.validate(control)?;
        Ok(calibration)
    }

    /// Validates the calibration values for one control channel.
    pub fn validate(&self, control: Control) -> Result<(), InputError> {
        if !self.raw_min.is_finite() || !self.raw_center.is_finite() || !self.raw_max.is_finite() {
            return Err(InputError::NonFiniteCalibration { control });
        }
        validate_deadzone_value(self.deadzone)?;
        if self.raw_min >= self.raw_center || self.raw_center >= self.raw_max {
            return Err(InputError::InvalidCalibrationOrder { control });
        }
        if self.raw_center - self.raw_min < MIN_CALIBRATION_SPAN
            || self.raw_max - self.raw_center < MIN_CALIBRATION_SPAN
        {
            return Err(InputError::DegenerateCalibrationSpan {
                control,
                min_span: MIN_CALIBRATION_SPAN,
            });
        }
        Ok(())
    }

    /// Calibrates one raw device value into `[-1, 1]`.
    ///
    /// `raw_min` maps to -1, `raw_center` to 0, and `raw_max` to +1. Values
    /// outside `[raw_min, raw_max]` saturate at the endpoints. The deadzone is
    /// centered on the calibrated center and the travel outside it is rescaled
    /// continuously onto the remaining range. Inversion flips the final sign.
    pub fn apply(&self, raw: f64, control: Control) -> Result<f64, InputError> {
        if !raw.is_finite() {
            return Err(InputError::NonFiniteRawAxis {
                axis: control.label(),
            });
        }
        let clamped = raw.clamp(self.raw_min, self.raw_max);
        let normalized = if clamped <= self.raw_center {
            (clamped - self.raw_min) / (self.raw_center - self.raw_min) - 1.0
        } else {
            (clamped - self.raw_center) / (self.raw_max - self.raw_center)
        };
        let magnitude = normalized.abs();
        let responsive = if magnitude <= self.deadzone {
            0.0
        } else {
            let rescaled = (magnitude - self.deadzone) / (1.0 - self.deadzone);
            normalized.signum() * rescaled.clamp(0.0, 1.0)
        };
        Ok(if self.inverted {
            -responsive
        } else {
            responsive
        })
    }

    #[must_use]
    pub const fn raw_min(&self) -> f64 {
        self.raw_min
    }

    #[must_use]
    pub const fn raw_center(&self) -> f64 {
        self.raw_center
    }

    #[must_use]
    pub const fn raw_max(&self) -> f64 {
        self.raw_max
    }

    #[must_use]
    pub const fn inverted(&self) -> bool {
        self.inverted
    }

    #[must_use]
    pub const fn deadzone(&self) -> f64 {
        self.deadzone
    }
}

/// Deterministic endpoint calibration for the throttle channel.
///
/// Maps `raw_min` to 0 and `raw_max` to 1. No endpoint deadband is applied:
/// at calibrated endpoints the physical stop is the authoritative reference,
/// and a deadband would only mask endpoint calibration errors.
///
/// Every instance is validated at construction. The type deliberately does not
/// implement `serde::Deserialize`, so unvalidated values can never reach
/// [`Self::apply`]; persisted calibrations are only decoded through
/// `ControllerProfile::from_json`.
///
/// ```compile_fail
/// use platform::ThrottleCalibration;
///
/// let calibration: ThrottleCalibration = serde_json::from_str("{}").unwrap();
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct ThrottleCalibration {
    raw_min: f64,
    raw_max: f64,
    inverted: bool,
}

impl ThrottleCalibration {
    /// Constructs and validates a throttle calibration.
    pub fn new(raw_min: f64, raw_max: f64, inverted: bool) -> Result<Self, InputError> {
        let calibration = Self {
            raw_min,
            raw_max,
            inverted,
        };
        calibration.validate()?;
        Ok(calibration)
    }

    /// Validates the calibration values.
    pub fn validate(&self) -> Result<(), InputError> {
        if !self.raw_min.is_finite() || !self.raw_max.is_finite() {
            return Err(InputError::NonFiniteCalibration {
                control: Control::Throttle,
            });
        }
        if self.raw_min >= self.raw_max {
            return Err(InputError::InvalidCalibrationOrder {
                control: Control::Throttle,
            });
        }
        if self.raw_max - self.raw_min < MIN_CALIBRATION_SPAN {
            return Err(InputError::DegenerateCalibrationSpan {
                control: Control::Throttle,
                min_span: MIN_CALIBRATION_SPAN,
            });
        }
        Ok(())
    }

    /// Calibrates one raw throttle value into `[0, 1]`.
    ///
    /// Values outside `[raw_min, raw_max]` clamp to the nearest endpoint.
    /// Inversion exchanges the idle and full endpoints.
    pub fn apply(&self, raw: f64) -> Result<f64, InputError> {
        if !raw.is_finite() {
            return Err(InputError::NonFiniteRawAxis {
                axis: Control::Throttle.label(),
            });
        }
        let position = ((raw - self.raw_min) / (self.raw_max - self.raw_min)).clamp(0.0, 1.0);
        Ok(if self.inverted {
            1.0 - position
        } else {
            position
        })
    }

    #[must_use]
    pub const fn raw_min(&self) -> f64 {
        self.raw_min
    }

    #[must_use]
    pub const fn raw_max(&self) -> f64 {
        self.raw_max
    }

    #[must_use]
    pub const fn inverted(&self) -> bool {
        self.inverted
    }
}

fn validate_deadzone_value(deadzone: f64) -> Result<(), InputError> {
    if deadzone.is_finite() && (0.0..1.0).contains(&deadzone) {
        Ok(())
    } else {
        Err(InputError::InvalidDeadzone)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration(
        raw_min: f64,
        raw_center: f64,
        raw_max: f64,
        inverted: bool,
        deadzone: f64,
    ) -> CenteredCalibration {
        CenteredCalibration::new(
            Control::Roll,
            raw_min,
            raw_center,
            raw_max,
            inverted,
            deadzone,
        )
        .unwrap()
    }

    #[test]
    fn calibration_is_deterministic_across_repeated_applies() {
        let centered = calibration(-0.9, -0.1, 0.7, true, 0.1);
        let throttle = ThrottleCalibration::new(-1.0, 1.0, true).unwrap();
        for raw in [-2.0, -0.9, -0.1, 0.0, 0.4, 0.7, 2.0] {
            let first = centered.apply(raw, Control::Roll).unwrap();
            let second = centered.apply(raw, Control::Roll).unwrap();
            assert_eq!(first, second);
            let first_throttle = throttle.apply(raw).unwrap();
            let second_throttle = throttle.apply(raw).unwrap();
            assert_eq!(first_throttle, second_throttle);
        }
    }
}
