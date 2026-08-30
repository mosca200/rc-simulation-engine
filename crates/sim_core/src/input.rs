use serde::{Deserialize, Serialize};

/// Normalized radio input before rates, expo, mixing, or servo dynamics.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PilotInput {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
    pub throttle: f64,
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
    pub fn is_valid(&self) -> bool {
        (-1.0..=1.0).contains(&self.roll)
            && (-1.0..=1.0).contains(&self.pitch)
            && (-1.0..=1.0).contains(&self.yaw)
            && (0.0..=1.0).contains(&self.throttle)
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
        assert_eq!(
            PilotInput::new(-2.0, 2.0, f64::NAN, 3.0),
            PilotInput {
                roll: -1.0,
                pitch: 1.0,
                yaw: 0.0,
                throttle: 1.0,
            }
        );
    }
}
