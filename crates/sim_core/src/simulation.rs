use crate::{
    BodyWrench, ParameterError, PilotInput, RigidBodyParams, RigidBodyState, Rk4Integrator,
    SimSnapshot, StateError,
};
use serde::{Deserialize, Serialize};
use sim_math::Vec3;
use thiserror::Error;

pub const DEFAULT_PHYSICS_HZ: u32 = 500;
pub const DEFAULT_GRAVITY_MPS2: f64 = 9.80665;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SimulationConfig {
    dt_s: f64,
    gravity_world_mps2: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum SimulationConfigError {
    #[error("physics frequency must be greater than zero")]
    ZeroFrequency,
    #[error("simulation timestep must be finite and greater than zero")]
    InvalidTimestep,
    #[error("gravity must contain only finite values")]
    InvalidGravity,
}

impl SimulationConfig {
    pub fn from_physics_hz(physics_hz: u32) -> Result<Self, SimulationConfigError> {
        if physics_hz == 0 {
            return Err(SimulationConfigError::ZeroFrequency);
        }
        Self::new(
            1.0 / f64::from(physics_hz),
            Vec3::new(0.0, 0.0, DEFAULT_GRAVITY_MPS2),
        )
    }

    pub fn new(dt_s: f64, gravity_world_mps2: Vec3) -> Result<Self, SimulationConfigError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(SimulationConfigError::InvalidTimestep);
        }
        if !gravity_world_mps2.iter().all(|value| value.is_finite()) {
            return Err(SimulationConfigError::InvalidGravity);
        }
        Ok(Self {
            dt_s,
            gravity_world_mps2,
        })
    }

    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.dt_s
    }

    #[must_use]
    pub const fn gravity_world_mps2(&self) -> &Vec3 {
        &self.gravity_world_mps2
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self::from_physics_hz(DEFAULT_PHYSICS_HZ).expect("default frequency is valid")
    }
}

#[derive(Debug, Error)]
pub enum SimulationError {
    #[error(transparent)]
    Parameters(#[from] ParameterError),
    #[error(transparent)]
    InitialState(#[from] StateError),
    #[error(transparent)]
    Configuration(#[from] SimulationConfigError),
}

/// Principal mutable owner of canonical physics state.
#[derive(Debug, Clone)]
pub struct Simulation {
    config: SimulationConfig,
    body_params: RigidBodyParams,
    state: RigidBodyState,
    step_index: u64,
}

impl Simulation {
    pub fn new(
        config: SimulationConfig,
        body_params: RigidBodyParams,
        initial_state: RigidBodyState,
    ) -> Result<Self, SimulationError> {
        initial_state.validate()?;
        SimulationConfig::new(config.dt_s, config.gravity_world_mps2)?;
        Ok(Self {
            config,
            body_params,
            state: initial_state,
            step_index: 0,
        })
    }

    /// Advances exactly one fixed physics step and returns the resulting post-step snapshot.
    #[must_use]
    pub fn step(&mut self, _input: &PilotInput) -> SimSnapshot {
        self.state = Rk4Integrator::step(
            &self.state,
            &self.body_params,
            &BodyWrench::zero(),
            self.config.gravity_world_mps2(),
            self.config.dt_s(),
        );
        self.step_index += 1;
        debug_assert!(self.state.validate().is_ok());
        SimSnapshot::from_state(self.step_index, self.config.dt_s(), &self.state)
    }

    #[must_use]
    pub const fn state(&self) -> &RigidBodyState {
        &self.state
    }

    #[must_use]
    pub const fn body_params(&self) -> &RigidBodyParams {
        &self.body_params
    }

    #[must_use]
    pub const fn config(&self) -> &SimulationConfig {
        &self.config
    }

    #[must_use]
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    #[must_use]
    pub fn sim_time_s(&self) -> f64 {
        self.step_index as f64 * self.config.dt_s()
    }

    #[must_use]
    pub fn snapshot(&self) -> SimSnapshot {
        SimSnapshot::from_state(self.step_index, self.config.dt_s(), &self.state)
    }
}
