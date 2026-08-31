use std::time::Duration;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum FixedStepAccumulatorError {
    #[error("physics timestep must be non-zero")]
    ZeroPhysicsTimestep,
    #[error("maximum frame delta must be non-zero")]
    ZeroMaximumFrameDelta,
    #[error("maximum physics steps per frame must be greater than zero")]
    ZeroMaximumSteps,
}

/// Result of feeding one wall-clock frame duration into the fixed-step scheduler.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedStepPlan {
    physics_steps: u32,
    remainder: Duration,
    dropped_time_s: f64,
}

impl FixedStepPlan {
    #[must_use]
    pub const fn physics_steps(&self) -> u32 {
        self.physics_steps
    }

    #[must_use]
    pub const fn remainder(&self) -> Duration {
        self.remainder
    }

    #[must_use]
    pub const fn dropped_time_s(&self) -> f64 {
        self.dropped_time_s
    }
}

/// Integer-nanosecond wall-clock accumulator that never changes the physics timestep.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedStepAccumulator {
    physics_dt: Duration,
    physics_dt_ns: u128,
    maximum_frame_delta_ns: u128,
    maximum_steps_per_frame: u32,
    remainder_ns: u128,
}

impl FixedStepAccumulator {
    pub fn new(
        physics_dt: Duration,
        maximum_frame_delta: Duration,
        maximum_steps_per_frame: u32,
    ) -> Result<Self, FixedStepAccumulatorError> {
        if physics_dt.is_zero() {
            return Err(FixedStepAccumulatorError::ZeroPhysicsTimestep);
        }
        if maximum_frame_delta.is_zero() {
            return Err(FixedStepAccumulatorError::ZeroMaximumFrameDelta);
        }
        if maximum_steps_per_frame == 0 {
            return Err(FixedStepAccumulatorError::ZeroMaximumSteps);
        }
        Ok(Self {
            physics_dt,
            physics_dt_ns: physics_dt.as_nanos(),
            maximum_frame_delta_ns: maximum_frame_delta.as_nanos(),
            maximum_steps_per_frame,
            remainder_ns: 0,
        })
    }

    #[must_use]
    pub const fn physics_dt(&self) -> Duration {
        self.physics_dt
    }

    #[must_use]
    pub fn remainder(&self) -> Duration {
        duration_from_nanos(self.remainder_ns)
    }

    /// Clamps pathological frame time, returns a fixed-step count, and discards excess whole steps.
    pub fn advance(&mut self, frame_delta: Duration) -> FixedStepPlan {
        let frame_delta_ns = frame_delta.as_nanos();
        let accepted_ns = frame_delta_ns.min(self.maximum_frame_delta_ns);
        let clipped_ns = frame_delta_ns - accepted_ns;
        let accumulated_ns = self.remainder_ns.saturating_add(accepted_ns);
        let available_steps = accumulated_ns / self.physics_dt_ns;
        self.remainder_ns = accumulated_ns % self.physics_dt_ns;

        let executed_steps = available_steps.min(u128::from(self.maximum_steps_per_frame));
        let discarded_step_ns =
            (available_steps - executed_steps).saturating_mul(self.physics_dt_ns);
        let dropped_ns = clipped_ns.saturating_add(discarded_step_ns);
        let physics_steps = executed_steps as u32;

        FixedStepPlan {
            physics_steps,
            remainder: self.remainder(),
            dropped_time_s: dropped_ns as f64 * 1.0e-9,
        }
    }
}

fn duration_from_nanos(nanoseconds: u128) -> Duration {
    let seconds = (nanoseconds / 1_000_000_000) as u64;
    let subsecond_nanoseconds = (nanoseconds % 1_000_000_000) as u32;
    Duration::new(seconds, subsecond_nanoseconds)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn accumulator(maximum_steps: u32) -> FixedStepAccumulator {
        FixedStepAccumulator::new(
            Duration::from_millis(2),
            Duration::from_millis(250),
            maximum_steps,
        )
        .unwrap()
    }

    #[test]
    fn below_dt_produces_zero_steps() {
        let mut accumulator = accumulator(8);
        let plan = accumulator.advance(Duration::from_millis(1));
        assert_eq!(plan.physics_steps(), 0);
        assert_eq!(plan.remainder(), Duration::from_millis(1));
    }

    #[test]
    fn exactly_dt_produces_one_step() {
        let mut accumulator = accumulator(8);
        let plan = accumulator.advance(Duration::from_millis(2));
        assert_eq!(plan.physics_steps(), 1);
        assert_eq!(plan.remainder(), Duration::ZERO);
        assert_eq!(accumulator.physics_dt(), Duration::from_millis(2));
    }

    #[test]
    fn multiple_steps_preserve_fractional_remainder() {
        let mut accumulator = accumulator(8);
        let plan = accumulator.advance(Duration::from_micros(5_500));
        assert_eq!(plan.physics_steps(), 2);
        assert_eq!(plan.remainder(), Duration::from_micros(1_500));
    }

    #[test]
    fn remainder_is_conserved_across_frames() {
        let mut accumulator = accumulator(8);
        assert_eq!(
            accumulator
                .advance(Duration::from_millis(1))
                .physics_steps(),
            0
        );
        let plan = accumulator.advance(Duration::from_millis(1));
        assert_eq!(plan.physics_steps(), 1);
        assert_eq!(plan.remainder(), Duration::ZERO);
    }

    #[test]
    fn step_cap_discards_whole_backlog_without_changing_dt() {
        let mut accumulator = accumulator(4);
        let plan = accumulator.advance(Duration::from_millis(20));
        assert_eq!(plan.physics_steps(), 4);
        assert_eq!(plan.remainder(), Duration::ZERO);
        assert!((plan.dropped_time_s() - 0.012).abs() < 1.0e-15);
        assert_eq!(accumulator.physics_dt(), Duration::from_millis(2));
    }

    #[test]
    fn pathological_frame_delta_is_clamped_and_reported() {
        let mut accumulator = accumulator(4);
        let plan = accumulator.advance(Duration::from_secs(1));
        assert_eq!(plan.physics_steps(), 4);
        assert_eq!(plan.remainder(), Duration::ZERO);
        assert!((plan.dropped_time_s() - 0.992).abs() < 1.0e-12);
    }
}
