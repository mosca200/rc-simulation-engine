use crate::{ControllerAxes, InputError, InputMapping};
use sim_core::PilotInput;

const KEYBOARD_THROTTLE_RATE_PER_S: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KeyboardKey {
    RollLeft,
    RollRight,
    PitchUp,
    PitchDown,
    YawLeft,
    YawRight,
    ThrottleIncrease,
    ThrottleDecrease,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyboardInputState {
    pressed: [bool; 8],
    throttle: f64,
}

impl KeyboardInputState {
    pub fn new(initial_throttle: f64) -> Result<Self, InputError> {
        if !initial_throttle.is_finite() || !(0.0..=1.0).contains(&initial_throttle) {
            return Err(InputError::InvalidInitialThrottle);
        }
        Ok(Self {
            pressed: [false; 8],
            throttle: initial_throttle,
        })
    }

    pub fn set_key(&mut self, key: KeyboardKey, pressed: bool) {
        self.pressed[key as usize] = pressed;
    }

    pub fn sample(&mut self, dt_s: f64) -> Result<PilotInput, InputError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(InputError::InvalidSamplingTimestep);
        }
        let roll = digital_axis(
            self.pressed[KeyboardKey::RollLeft as usize],
            self.pressed[KeyboardKey::RollRight as usize],
        );
        let pitch = digital_axis(
            self.pressed[KeyboardKey::PitchDown as usize],
            self.pressed[KeyboardKey::PitchUp as usize],
        );
        let yaw = digital_axis(
            self.pressed[KeyboardKey::YawLeft as usize],
            self.pressed[KeyboardKey::YawRight as usize],
        );
        let throttle_direction = digital_axis(
            self.pressed[KeyboardKey::ThrottleDecrease as usize],
            self.pressed[KeyboardKey::ThrottleIncrease as usize],
        );
        self.throttle = (self.throttle + throttle_direction * KEYBOARD_THROTTLE_RATE_PER_S * dt_s)
            .clamp(0.0, 1.0);
        Ok(PilotInput::new(roll, pitch, yaw, self.throttle))
    }

    #[must_use]
    pub const fn throttle(&self) -> f64 {
        self.throttle
    }
}

impl Default for KeyboardInputState {
    fn default() -> Self {
        Self::new(0.55).expect("the fixed default keyboard throttle is valid")
    }
}

pub trait InputSource {
    fn sample(&mut self, physics_dt_s: f64) -> Result<PilotInput, InputError>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InputState {
    mapping: InputMapping,
    keyboard: KeyboardInputState,
    controller_axes: Option<ControllerAxes>,
}

impl InputState {
    #[must_use]
    pub const fn new(mapping: InputMapping, keyboard: KeyboardInputState) -> Self {
        Self {
            mapping,
            keyboard,
            controller_axes: None,
        }
    }

    pub fn set_key(&mut self, key: KeyboardKey, pressed: bool) {
        self.keyboard.set_key(key, pressed);
    }

    pub fn set_controller_axes(&mut self, axes: Option<ControllerAxes>) {
        self.controller_axes = axes;
    }

    #[must_use]
    pub const fn has_controller(&self) -> bool {
        self.controller_axes.is_some()
    }
}

impl InputSource for InputState {
    fn sample(&mut self, physics_dt_s: f64) -> Result<PilotInput, InputError> {
        if !physics_dt_s.is_finite() || physics_dt_s <= 0.0 {
            return Err(InputError::InvalidSamplingTimestep);
        }
        match self.controller_axes {
            Some(axes) => self.mapping.map_axes(axes),
            None => self.keyboard.sample(physics_dt_s),
        }
    }
}

impl Default for InputState {
    fn default() -> Self {
        Self::new(InputMapping::default(), KeyboardInputState::default())
    }
}

fn digital_axis(negative: bool, positive: bool) -> f64 {
    f64::from(u8::from(positive)) - f64::from(u8::from(negative))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keyboard_axes_support_press_release_and_simultaneous_keys() {
        let mut keyboard = KeyboardInputState::default();
        keyboard.set_key(KeyboardKey::RollLeft, true);
        keyboard.set_key(KeyboardKey::PitchUp, true);
        keyboard.set_key(KeyboardKey::YawRight, true);
        let input = keyboard.sample(0.002).unwrap();
        assert_eq!(input.roll(), -1.0);
        assert_eq!(input.pitch(), 1.0);
        assert_eq!(input.yaw(), 1.0);

        keyboard.set_key(KeyboardKey::RollRight, true);
        assert_eq!(keyboard.sample(0.002).unwrap().roll(), 0.0);
        keyboard.set_key(KeyboardKey::RollLeft, false);
        assert_eq!(keyboard.sample(0.002).unwrap().roll(), 1.0);
        keyboard.set_key(KeyboardKey::RollRight, false);
        assert_eq!(keyboard.sample(0.002).unwrap().roll(), 0.0);
    }

    #[test]
    fn keyboard_throttle_changes_per_fixed_sample_and_respects_release() {
        let mut keyboard = KeyboardInputState::new(0.5).unwrap();
        keyboard.set_key(KeyboardKey::ThrottleIncrease, true);
        assert_eq!(keyboard.sample(0.002).unwrap().throttle(), 0.501);
        assert_eq!(keyboard.sample(0.002).unwrap().throttle(), 0.502);
        keyboard.set_key(KeyboardKey::ThrottleIncrease, false);
        assert_eq!(keyboard.sample(0.002).unwrap().throttle(), 0.502);
        keyboard.set_key(KeyboardKey::ThrottleDecrease, true);
        assert_eq!(keyboard.sample(0.002).unwrap().throttle(), 0.501);
    }

    #[test]
    fn input_sampling_is_deterministic_for_repeated_physics_steps() {
        let mut first = InputState::default();
        let mut second = InputState::default();
        for state in [&mut first, &mut second] {
            state.set_key(KeyboardKey::PitchDown, true);
            state.set_key(KeyboardKey::ThrottleIncrease, true);
        }
        for _ in 0..100 {
            assert_eq!(first.sample(0.002).unwrap(), second.sample(0.002).unwrap());
        }
    }

    #[test]
    fn absent_controller_uses_keyboard_and_present_controller_is_mapped() {
        let mut state = InputState::default();
        state.set_key(KeyboardKey::RollRight, true);
        assert_eq!(state.sample(0.002).unwrap().roll(), 1.0);
        state.set_controller_axes(Some(ControllerAxes::new(-1.0, 0.0, 0.0, -1.0)));
        let controller_input = state.sample(0.002).unwrap();
        assert_eq!(controller_input.roll(), -1.0);
        assert_eq!(controller_input.throttle(), 1.0);
        state.set_controller_axes(None);
        assert_eq!(state.sample(0.002).unwrap().roll(), 1.0);
    }
}
