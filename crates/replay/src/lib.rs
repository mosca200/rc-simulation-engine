#![forbid(unsafe_code)]
//! Versioned, input-based deterministic replay recording and playback.

use serde::{Deserialize, Serialize};
use sim_core::{PilotInput, RigidBodyState, Simulation, SimulationConfig};
use thiserror::Error;

pub const REPLAY_SCHEMA_VERSION: u32 = 1;

/// BLAKE3 identity of every value that determines the initial simulation evolution.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SimulationFingerprint([u8; 32]);

impl SimulationFingerprint {
    /// Encodes integers little-endian and floating-point values through their IEEE-754 bits.
    #[must_use]
    pub fn from_simulation(simulation: &Simulation) -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"rcsim:simulation-fingerprint:v1");
        hasher.update(&REPLAY_SCHEMA_VERSION.to_le_bytes());
        update_f64(&mut hasher, simulation.config().dt_s());
        update_vector(
            &mut hasher,
            simulation.config().gravity_world_mps2().as_slice(),
        );
        update_f64(&mut hasher, simulation.body_params().mass_kg());

        // Matrix order is explicit regardless of nalgebra's internal storage order.
        let inertia = simulation.body_params().inertia_body_kg_m2();
        for row in 0..3 {
            for column in 0..3 {
                update_f64(&mut hasher, inertia[(row, column)]);
            }
        }

        let state = simulation.state();
        update_vector(&mut hasher, state.position_world_m.as_slice());
        update_vector(&mut hasher, state.linear_velocity_world_mps.as_slice());
        let orientation = state.orientation_world_from_body.quaternion();
        for value in [orientation.w, orientation.i, orientation.j, orientation.k] {
            update_f64(&mut hasher, value);
        }
        update_vector(&mut hasher, state.angular_velocity_body_radps.as_slice());
        hasher.update(&simulation.step_index().to_le_bytes());

        Self(*hasher.finalize().as_bytes())
    }

    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

fn update_f64(hasher: &mut blake3::Hasher, value: f64) {
    hasher.update(&value.to_bits().to_le_bytes());
}

fn update_vector(hasher: &mut blake3::Hasher, values: &[f64]) {
    for &value in values {
        update_f64(hasher, value);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ReplayFrame {
    /// Pre-step index: frame 0 is the input used to advance snapshot 0 to snapshot 1.
    pub step_index: u64,
    pub pilot_input: PilotInput,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ReplayRecording {
    pub schema_version: u32,
    pub simulation_fingerprint: SimulationFingerprint,
    pub simulation_config: SimulationConfig,
    pub initial_state: RigidBodyState,
    pub frames: Vec<ReplayFrame>,
}

impl ReplayRecording {
    pub fn to_json_pretty(&self) -> Result<String, ReplayError> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    pub fn from_json(json: &str) -> Result<Self, ReplayError> {
        let recording: Self = serde_json::from_str(json)?;
        validate_recording(&recording)?;
        Ok(recording)
    }
}

#[derive(Debug, Clone)]
pub struct ReplayRecorder {
    recording: ReplayRecording,
}

impl ReplayRecorder {
    pub fn new(simulation: &Simulation) -> Result<Self, ReplayError> {
        Self::with_capacity(simulation, 0)
    }

    pub fn with_capacity(
        simulation: &Simulation,
        frame_capacity: usize,
    ) -> Result<Self, ReplayError> {
        if simulation.step_index() != 0 {
            return Err(ReplayError::SimulationNotAtInitialStep(
                simulation.step_index(),
            ));
        }
        Ok(Self {
            recording: ReplayRecording {
                schema_version: REPLAY_SCHEMA_VERSION,
                simulation_fingerprint: SimulationFingerprint::from_simulation(simulation),
                simulation_config: *simulation.config(),
                initial_state: *simulation.state(),
                frames: Vec::with_capacity(frame_capacity),
            },
        })
    }

    pub fn record(&mut self, step_index: u64, pilot_input: PilotInput) -> Result<(), ReplayError> {
        let expected = self.recording.frames.len() as u64;
        if step_index != expected {
            return Err(ReplayError::NonContiguousStep {
                expected,
                actual: step_index,
            });
        }
        if !pilot_input.is_valid() {
            return Err(ReplayError::InvalidPilotInput);
        }
        self.recording.frames.push(ReplayFrame {
            step_index,
            pilot_input,
        });
        Ok(())
    }

    #[must_use]
    pub fn finish(self) -> ReplayRecording {
        self.recording
    }
}

#[derive(Debug, Clone)]
pub struct ReplayPlayer<'a> {
    recording: &'a ReplayRecording,
    next_frame: usize,
}

impl<'a> ReplayPlayer<'a> {
    pub fn new(
        recording: &'a ReplayRecording,
        simulation: &Simulation,
    ) -> Result<Self, ReplayError> {
        validate_recording(recording)?;
        if simulation.step_index() != 0 {
            return Err(ReplayError::SimulationNotAtInitialStep(
                simulation.step_index(),
            ));
        }
        let actual = SimulationFingerprint::from_simulation(simulation);
        if actual != recording.simulation_fingerprint {
            return Err(ReplayError::SimulationFingerprintMismatch);
        }
        Ok(Self {
            recording,
            next_frame: 0,
        })
    }

    #[must_use]
    pub fn next_input(&mut self) -> Option<ReplayFrame> {
        let frame = self.recording.frames.get(self.next_frame).copied();
        if frame.is_some() {
            self.next_frame += 1;
        }
        frame
    }

    #[must_use]
    pub fn remaining(&self) -> usize {
        self.recording.frames.len() - self.next_frame
    }
}

fn validate_recording(recording: &ReplayRecording) -> Result<(), ReplayError> {
    if recording.schema_version != REPLAY_SCHEMA_VERSION {
        return Err(ReplayError::UnsupportedSchema(recording.schema_version));
    }
    recording
        .initial_state
        .validate()
        .map_err(|_| ReplayError::InvalidInitialState)?;
    SimulationConfig::new(
        recording.simulation_config.dt_s(),
        *recording.simulation_config.gravity_world_mps2(),
    )
    .map_err(|_| ReplayError::InvalidSimulationConfig)?;
    for (expected, frame) in recording.frames.iter().enumerate() {
        let expected = expected as u64;
        if frame.step_index != expected {
            return Err(ReplayError::NonContiguousStep {
                expected,
                actual: frame.step_index,
            });
        }
        if !frame.pilot_input.is_valid() {
            return Err(ReplayError::InvalidPilotInput);
        }
    }
    Ok(())
}

#[derive(Debug, Error)]
pub enum ReplayError {
    #[error("unsupported replay schema version {0}")]
    UnsupportedSchema(u32),
    #[error("non-contiguous replay step: expected {expected}, got {actual}")]
    NonContiguousStep { expected: u64, actual: u64 },
    #[error("replay contains invalid normalized pilot input")]
    InvalidPilotInput,
    #[error("replay contains an invalid initial rigid-body state")]
    InvalidInitialState,
    #[error("replay contains an invalid simulation configuration")]
    InvalidSimulationConfig,
    #[error("simulation must be at initial step zero, got step {0}")]
    SimulationNotAtInitialStep(u64),
    #[error("reconstructed simulation fingerprint differs from the recording")]
    SimulationFingerprintMismatch,
    #[error("replay JSON error: {0}")]
    Json(#[from] serde_json::Error),
}
