use sim_core::{
    AeroEnvironment, DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ, SimulationConfig,
    SimulationConfigError,
};
use sim_math::Vec3;

/// Fixed-step physics and constant atmospheric environment for one aircraft run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftSimulationConfig {
    rigid_body: SimulationConfig,
    aero_environment: AeroEnvironment,
}

impl AircraftSimulationConfig {
    pub fn new(
        dt_s: f64,
        gravity_world_mps2: Vec3,
        aero_environment: AeroEnvironment,
    ) -> Result<Self, SimulationConfigError> {
        Ok(Self {
            rigid_body: SimulationConfig::new(dt_s, gravity_world_mps2)?,
            aero_environment,
        })
    }

    pub fn from_physics_hz(
        physics_hz: u32,
        aero_environment: AeroEnvironment,
    ) -> Result<Self, SimulationConfigError> {
        Ok(Self {
            rigid_body: SimulationConfig::from_physics_hz(physics_hz)?,
            aero_environment,
        })
    }

    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.rigid_body.dt_s()
    }

    #[must_use]
    pub const fn gravity_world_mps2(&self) -> &Vec3 {
        self.rigid_body.gravity_world_mps2()
    }

    #[must_use]
    pub const fn aero_environment(&self) -> &AeroEnvironment {
        &self.aero_environment
    }
}

impl Default for AircraftSimulationConfig {
    fn default() -> Self {
        Self::new(
            1.0 / f64::from(DEFAULT_PHYSICS_HZ),
            Vec3::new(0.0, 0.0, DEFAULT_GRAVITY_MPS2),
            AeroEnvironment::new(1.225, Vec3::zeros()).expect("the standard atmosphere is valid"),
        )
        .expect("the default aircraft simulation configuration is valid")
    }
}
