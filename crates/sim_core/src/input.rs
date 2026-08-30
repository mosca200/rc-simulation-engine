use serde::{Deserialize, Deserializer, Serialize, de::Error as _};

/// Normalized radio input before rates, expo, mixing, or servo dynamics.
///
/// Positive commands follow the FRD body axes: roll is right-wing-down, pitch
/// is nose-up, and yaw is nose-right. Throttle is zero at idle and one at full
/// command.
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct PilotInput {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

impl PilotInput {
    /// Constructs an input and clamps it once at the external-input boundary.
    #[must_use]
    pub fn new(roll: f64, pitch: f64, yaw: f64, throttle: f64) -> Self {
        Self {
            roll: clamp_finite(roll, -1.0, 1.0),
            pitch: clamp_finite(pitch, -1.0, 1.0),
            yaw: clamp_finite(yaw, -1.0, 1.0),
            throttle: clamp_finite(throttle, 0.0, 1.0),
        }
    }

    #[must_use]
    pub const fn neutral() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            throttle: 0.0,
        }
    }

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

    #[must_use]
    pub fn is_valid(&self) -> bool {
        (-1.0..=1.0).contains(&self.roll)
            && (-1.0..=1.0).contains(&self.pitch)
            && (-1.0..=1.0).contains(&self.yaw)
            && (0.0..=1.0).contains(&self.throttle)
    }
}

#[derive(Deserialize)]
struct SerializedPilotInput {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

impl<'de> Deserialize<'de> for PilotInput {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let input = SerializedPilotInput::deserialize(deserializer)?;
        let input = Self {
            roll: input.roll,
            pitch: input.pitch,
            yaw: input.yaw,
            throttle: input.throttle,
        };
        if !input.is_valid() {
            return Err(D::Error::custom(
                "pilot input must be finite with roll/pitch/yaw in [-1, 1] and throttle in [0, 1]",
            ));
        }
        Ok(input)
    }
}

fn clamp_finite(value: f64, minimum: f64, maximum: f64) -> f64 {
    if value.is_finite() {
        value.clamp(minimum, maximum)
    } else {
        0.0
    }
}

impl Default for PilotInput {
    fn default() -> Self {
        Self::neutral()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_construction_boundary() {
        let input = PilotInput::new(-2.0, 2.0, f64::NAN, 3.0);
        assert_eq!(input.roll(), -1.0);
        assert_eq!(input.pitch(), 1.0);
        assert_eq!(input.yaw(), 0.0);
        assert_eq!(input.throttle(), 1.0);
    }
}
