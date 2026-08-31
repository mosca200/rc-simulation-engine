use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError, AircraftSnapshot,
};
use model::{AircraftModel, AircraftModelFingerprint};
use serde::{Deserialize, Serialize};
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Quaternion, Vec3};
use std::fmt;
use thiserror::Error;

pub const AIRCRAFT_REPLAY_SCHEMA_VERSION: u32 = 1;
const SNAPSHOT_HASH_DOMAIN: &[u8] = b"rcsim:aircraft-snapshot:v1";

/// Canonical BLAKE3 digest of one committed post-step aircraft snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AircraftSnapshotHash([u8; 32]);

impl AircraftSnapshotHash {
    #[must_use]
    pub fn from_snapshot(snapshot: &AircraftSnapshot) -> Self {
        let mut hasher = blake3::Hasher::new();
        hasher.update(SNAPSHOT_HASH_DOMAIN);
        hasher.update(&snapshot.step_index().to_le_bytes());
        update_f64(&mut hasher, snapshot.sim_time_s());

        let rigid_body = snapshot.rigid_body_state();
        update_vec3(&mut hasher, &rigid_body.position_world_m);
        update_vec3(&mut hasher, &rigid_body.linear_velocity_world_mps);
        let quaternion = rigid_body.orientation_world_from_body.quaternion();
        for value in [quaternion.w, quaternion.i, quaternion.j, quaternion.k] {
            update_f64(&mut hasher, value);
        }
        update_vec3(&mut hasher, &rigid_body.angular_velocity_body_radps);

        let controls = snapshot.control_surface_positions();
        for value in [
            controls.aileron_angle_rad(),
            controls.elevator_angle_rad(),
            controls.rudder_angle_rad(),
            controls.throttle(),
        ] {
            update_f64(&mut hasher, value);
        }
        Self(*hasher.finalize().as_bytes())
    }

    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    #[must_use]
    pub fn to_hex(self) -> String {
        encode_hex(&self.0)
    }
}

impl fmt::Display for AircraftSnapshotHash {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&encode_hex(&self.0))
    }
}

/// Exact 32-byte model physics identity stored independently from `model_id`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AircraftModelPhysicsFingerprint([u8; 32]);

impl AircraftModelPhysicsFingerprint {
    #[must_use]
    pub fn from_model_fingerprint(fingerprint: AircraftModelFingerprint) -> Self {
        Self(*fingerprint.as_bytes())
    }

    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }

    #[must_use]
    pub fn to_hex(self) -> String {
        encode_hex(&self.0)
    }
}

impl fmt::Display for AircraftModelPhysicsFingerprint {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(&encode_hex(&self.0))
    }
}

/// Schema-v1 simulation settings required to reconstruct `AircraftSimulation` exactly.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftReplaySimulationConfig {
    dt_s: f64,
    gravity_world_mps2: [f64; 3],
    air_density_kg_m3: f64,
    wind_velocity_world_mps: [f64; 3],
}

impl AircraftReplaySimulationConfig {
    #[must_use]
    pub fn from_runtime(config: &AircraftSimulationConfig) -> Self {
        Self {
            dt_s: config.dt_s(),
            gravity_world_mps2: vec3_to_array(config.gravity_world_mps2()),
            air_density_kg_m3: config.aero_environment().air_density_kg_m3(),
            wind_velocity_world_mps: vec3_to_array(
                config.aero_environment().wind_velocity_world_mps(),
            ),
        }
    }

    pub fn to_runtime(self) -> Result<AircraftSimulationConfig, AircraftReplayError> {
        let environment = AeroEnvironment::new(
            self.air_density_kg_m3,
            array_to_vec3(self.wind_velocity_world_mps),
        )
        .map_err(|error| AircraftReplayError::InvalidSimulationConfig(error.to_string()))?;
        AircraftSimulationConfig::new(
            self.dt_s,
            array_to_vec3(self.gravity_world_mps2),
            environment,
        )
        .map_err(|error| AircraftReplayError::InvalidSimulationConfig(error.to_string()))
    }

    #[must_use]
    pub const fn dt_s(&self) -> f64 {
        self.dt_s
    }

    #[must_use]
    pub const fn gravity_world_mps2(&self) -> [f64; 3] {
        self.gravity_world_mps2
    }

    #[must_use]
    pub const fn air_density_kg_m3(&self) -> f64 {
        self.air_density_kg_m3
    }

    #[must_use]
    pub const fn wind_velocity_world_mps(&self) -> [f64; 3] {
        self.wind_velocity_world_mps
    }
}

/// One pre-step input and the expected hash of its post-step snapshot.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftReplayFrame {
    step_index: u64,
    pilot_input: PilotInput,
    expected_snapshot_hash: AircraftSnapshotHash,
}

impl AircraftReplayFrame {
    #[must_use]
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    #[must_use]
    pub const fn pilot_input(&self) -> PilotInput {
        self.pilot_input
    }

    #[must_use]
    pub const fn expected_snapshot_hash(&self) -> AircraftSnapshotHash {
        self.expected_snapshot_hash
    }
}

/// Input-based aircraft replay with per-step deterministic regression oracles.
#[derive(Debug, Clone, PartialEq)]
pub struct AircraftReplayRecording {
    schema_version: u32,
    model_id: String,
    model_physics_fingerprint: AircraftModelPhysicsFingerprint,
    simulation_config: AircraftReplaySimulationConfig,
    initial_rigid_body_state: RigidBodyState,
    frames: Vec<AircraftReplayFrame>,
}

impl AircraftReplayRecording {
    pub fn to_json_pretty(&self) -> Result<String, AircraftReplayError> {
        validate_recording(self)?;
        Ok(serde_json::to_string_pretty(&RecordingDto::from(self))?)
    }

    pub fn from_json(json: &str) -> Result<Self, AircraftReplayError> {
        let dto: RecordingDto = serde_json::from_str(json)?;
        let recording = Self::try_from(dto)?;
        validate_recording(&recording)?;
        Ok(recording)
    }

    pub fn reconstruct_simulation(
        &self,
        model: AircraftModel,
    ) -> Result<AircraftSimulation, AircraftReplayError> {
        validate_recording(self)?;
        validate_model(self, &model)?;
        AircraftSimulation::new(
            model,
            self.simulation_config.to_runtime()?,
            self.initial_rigid_body_state,
        )
        .map_err(AircraftReplayError::SimulationCreation)
    }

    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    #[must_use]
    pub fn model_id(&self) -> &str {
        &self.model_id
    }

    #[must_use]
    pub const fn model_physics_fingerprint(&self) -> AircraftModelPhysicsFingerprint {
        self.model_physics_fingerprint
    }

    #[must_use]
    pub const fn simulation_config(&self) -> &AircraftReplaySimulationConfig {
        &self.simulation_config
    }

    #[must_use]
    pub const fn initial_rigid_body_state(&self) -> &RigidBodyState {
        &self.initial_rigid_body_state
    }

    #[must_use]
    pub fn frames(&self) -> &[AircraftReplayFrame] {
        &self.frames
    }
}

/// Recorder that exclusively owns mutable access to the simulation while frames are captured.
pub struct AircraftReplayRecorder {
    recording: AircraftReplayRecording,
}

impl AircraftReplayRecorder {
    pub fn new(simulation: &AircraftSimulation) -> Result<Self, AircraftReplayError> {
        Self::with_capacity(simulation, 0)
    }

    pub fn with_capacity(
        simulation: &AircraftSimulation,
        frame_capacity: usize,
    ) -> Result<Self, AircraftReplayError> {
        if simulation.step_index() != 0 {
            return Err(AircraftReplayError::SimulationNotAtInitialStep(
                simulation.step_index(),
            ));
        }
        let recording = AircraftReplayRecording {
            schema_version: AIRCRAFT_REPLAY_SCHEMA_VERSION,
            model_id: simulation.model().model_id().to_owned(),
            model_physics_fingerprint: AircraftModelPhysicsFingerprint::from_model_fingerprint(
                simulation.model().physics_fingerprint(),
            ),
            simulation_config: AircraftReplaySimulationConfig::from_runtime(simulation.config()),
            initial_rigid_body_state: *simulation.state().rigid_body(),
            frames: Vec::with_capacity(frame_capacity),
        };
        Ok(Self { recording })
    }

    /// Records pre-step `step_index`, advances once, and binds the resulting post-step hash.
    pub fn record(
        &mut self,
        simulation: &mut AircraftSimulation,
        step_index: u64,
        pilot_input: PilotInput,
    ) -> Result<AircraftSnapshot, AircraftReplayError> {
        let expected = self.recording.frames.len() as u64;
        if step_index != expected || simulation.step_index() != expected {
            return Err(AircraftReplayError::NonContiguousStep {
                expected,
                actual: step_index.max(simulation.step_index()),
            });
        }
        validate_model(&self.recording, simulation.model())?;
        if !config_bits_equal(
            &self.recording.simulation_config,
            &AircraftReplaySimulationConfig::from_runtime(simulation.config()),
        ) {
            return Err(AircraftReplayError::SimulationConfigMismatch);
        }
        if step_index == 0
            && !state_bits_equal(
                &self.recording.initial_rigid_body_state,
                simulation.state().rigid_body(),
            )
        {
            return Err(AircraftReplayError::InitialStateMismatch);
        }
        if !pilot_input.is_valid() {
            return Err(AircraftReplayError::InvalidPilotInput { step_index });
        }
        let snapshot = simulation.step(&pilot_input);
        let expected_snapshot_step = step_index + 1;
        if snapshot.step_index() != expected_snapshot_step {
            return Err(AircraftReplayError::SnapshotStepMismatch {
                frame_step_index: step_index,
                expected_snapshot_step,
                actual_snapshot_step: snapshot.step_index(),
            });
        }
        self.recording.frames.push(AircraftReplayFrame {
            step_index,
            pilot_input,
            expected_snapshot_hash: AircraftSnapshotHash::from_snapshot(&snapshot),
        });
        Ok(snapshot)
    }

    #[must_use]
    pub fn finish(self) -> AircraftReplayRecording {
        self.recording
    }
}

/// Step-by-step verifier that stops at the first deterministic divergence.
pub struct AircraftReplayPlayer<'a> {
    recording: &'a AircraftReplayRecording,
    next_frame: usize,
}

impl<'a> AircraftReplayPlayer<'a> {
    pub fn new(
        recording: &'a AircraftReplayRecording,
        simulation: &AircraftSimulation,
    ) -> Result<Self, AircraftReplayError> {
        validate_recording(recording)?;
        validate_simulation_setup(recording, simulation)?;
        Ok(Self {
            recording,
            next_frame: 0,
        })
    }

    pub fn verify_next(
        &mut self,
        simulation: &mut AircraftSimulation,
    ) -> Result<Option<AircraftSnapshot>, AircraftReplayError> {
        let Some(frame) = self.recording.frames.get(self.next_frame).copied() else {
            return Ok(None);
        };
        if simulation.step_index() != frame.step_index {
            return Err(AircraftReplayError::PlaybackStepMismatch {
                expected: frame.step_index,
                actual: simulation.step_index(),
            });
        }
        let snapshot = simulation.step(&frame.pilot_input);
        let expected_snapshot_step = frame.step_index + 1;
        if snapshot.step_index() != expected_snapshot_step {
            return Err(AircraftReplayError::SnapshotStepMismatch {
                frame_step_index: frame.step_index,
                expected_snapshot_step,
                actual_snapshot_step: snapshot.step_index(),
            });
        }
        let actual = AircraftSnapshotHash::from_snapshot(&snapshot);
        if actual != frame.expected_snapshot_hash {
            return Err(AircraftReplayError::SnapshotHashMismatch {
                frame_step_index: frame.step_index,
                snapshot_step_index: snapshot.step_index(),
                expected: frame.expected_snapshot_hash,
                actual,
            });
        }
        self.next_frame += 1;
        Ok(Some(snapshot))
    }

    pub fn verify_all(
        mut self,
        simulation: &mut AircraftSimulation,
    ) -> Result<u64, AircraftReplayError> {
        let mut verified = 0;
        while self.verify_next(simulation)?.is_some() {
            verified += 1;
        }
        Ok(verified)
    }

    #[must_use]
    pub fn remaining(&self) -> usize {
        self.recording.frames.len() - self.next_frame
    }
}

#[derive(Debug, Error)]
pub enum AircraftReplayError {
    #[error("unsupported aircraft replay schema version {0}")]
    UnsupportedSchema(u32),
    #[error("aircraft simulation must be at initial step zero, got step {0}")]
    SimulationNotAtInitialStep(u64),
    #[error("non-contiguous aircraft replay step: expected {expected}, got {actual}")]
    NonContiguousStep { expected: u64, actual: u64 },
    #[error("aircraft replay frame {step_index} contains invalid pilot input")]
    InvalidPilotInput { step_index: u64 },
    #[error("aircraft replay contains an invalid simulation configuration: {0}")]
    InvalidSimulationConfig(String),
    #[error("aircraft replay contains an invalid initial rigid-body state: {0}")]
    InvalidInitialState(String),
    #[error("aircraft replay model ID mismatch: expected `{expected}`, got `{actual}`")]
    ModelIdMismatch { expected: String, actual: String },
    #[error(
        "aircraft replay model physics fingerprint mismatch: expected {expected}, got {actual}"
    )]
    ModelPhysicsFingerprintMismatch {
        expected: AircraftModelPhysicsFingerprint,
        actual: AircraftModelPhysicsFingerprint,
    },
    #[error("aircraft replay simulation configuration differs from the recording")]
    SimulationConfigMismatch,
    #[error("aircraft replay initial rigid-body state differs from the recording")]
    InitialStateMismatch,
    #[error("aircraft replay playback step mismatch: expected {expected}, got {actual}")]
    PlaybackStepMismatch { expected: u64, actual: u64 },
    #[error(
        "aircraft replay frame {frame_step_index} expected post-step snapshot {expected_snapshot_step}, got {actual_snapshot_step}"
    )]
    SnapshotStepMismatch {
        frame_step_index: u64,
        expected_snapshot_step: u64,
        actual_snapshot_step: u64,
    },
    #[error(
        "aircraft replay diverged at pre-step frame {frame_step_index}, post-step snapshot {snapshot_step_index}: expected {expected}, got {actual}"
    )]
    SnapshotHashMismatch {
        frame_step_index: u64,
        snapshot_step_index: u64,
        expected: AircraftSnapshotHash,
        actual: AircraftSnapshotHash,
    },
    #[error("malformed aircraft snapshot hash `{0}`; expected 64 lowercase hexadecimal characters")]
    MalformedSnapshotHash(String),
    #[error(
        "malformed model physics fingerprint `{0}`; expected 64 lowercase hexadecimal characters"
    )]
    MalformedModelPhysicsFingerprint(String),
    #[error("failed to construct aircraft simulation from replay: {0}")]
    SimulationCreation(#[source] AircraftSimulationError),
    #[error("aircraft replay JSON error: {0}")]
    Json(#[from] serde_json::Error),
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RecordingDto {
    schema_version: u32,
    model_id: String,
    model_physics_fingerprint: String,
    simulation_config: SimulationConfigDto,
    initial_rigid_body_state: InitialStateDto,
    frames: Vec<FrameDto>,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct SimulationConfigDto {
    dt_s: f64,
    gravity_world_mps2: [f64; 3],
    air_density_kg_m3: f64,
    wind_velocity_world_mps: [f64; 3],
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct InitialStateDto {
    position_world_m: [f64; 3],
    linear_velocity_world_mps: [f64; 3],
    orientation_world_from_body_wxyz: [f64; 4],
    angular_velocity_body_radps: [f64; 3],
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrameDto {
    step_index: u64,
    pilot_input: PilotInputDto,
    expected_snapshot_hash: String,
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PilotInputDto {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

impl From<&AircraftReplayRecording> for RecordingDto {
    fn from(recording: &AircraftReplayRecording) -> Self {
        Self {
            schema_version: recording.schema_version,
            model_id: recording.model_id.clone(),
            model_physics_fingerprint: recording.model_physics_fingerprint.to_hex(),
            simulation_config: recording.simulation_config.into(),
            initial_rigid_body_state: recording.initial_rigid_body_state.into(),
            frames: recording.frames.iter().map(FrameDto::from).collect(),
        }
    }
}

impl TryFrom<RecordingDto> for AircraftReplayRecording {
    type Error = AircraftReplayError;

    fn try_from(dto: RecordingDto) -> Result<Self, Self::Error> {
        let fingerprint_text = dto.model_physics_fingerprint;
        let fingerprint = decode_hex_32(&fingerprint_text).ok_or_else(|| {
            AircraftReplayError::MalformedModelPhysicsFingerprint(fingerprint_text.clone())
        })?;
        Ok(Self {
            schema_version: dto.schema_version,
            model_id: dto.model_id,
            model_physics_fingerprint: AircraftModelPhysicsFingerprint(fingerprint),
            simulation_config: dto.simulation_config.into(),
            initial_rigid_body_state: dto.initial_rigid_body_state.try_into()?,
            frames: dto
                .frames
                .into_iter()
                .map(AircraftReplayFrame::try_from)
                .collect::<Result<_, _>>()?,
        })
    }
}

impl From<AircraftReplaySimulationConfig> for SimulationConfigDto {
    fn from(config: AircraftReplaySimulationConfig) -> Self {
        Self {
            dt_s: config.dt_s,
            gravity_world_mps2: config.gravity_world_mps2,
            air_density_kg_m3: config.air_density_kg_m3,
            wind_velocity_world_mps: config.wind_velocity_world_mps,
        }
    }
}

impl From<SimulationConfigDto> for AircraftReplaySimulationConfig {
    fn from(dto: SimulationConfigDto) -> Self {
        Self {
            dt_s: dto.dt_s,
            gravity_world_mps2: dto.gravity_world_mps2,
            air_density_kg_m3: dto.air_density_kg_m3,
            wind_velocity_world_mps: dto.wind_velocity_world_mps,
        }
    }
}

impl From<RigidBodyState> for InitialStateDto {
    fn from(state: RigidBodyState) -> Self {
        let quaternion = state.orientation_world_from_body.quaternion();
        Self {
            position_world_m: vec3_to_array(&state.position_world_m),
            linear_velocity_world_mps: vec3_to_array(&state.linear_velocity_world_mps),
            orientation_world_from_body_wxyz: [
                quaternion.w,
                quaternion.i,
                quaternion.j,
                quaternion.k,
            ],
            angular_velocity_body_radps: vec3_to_array(&state.angular_velocity_body_radps),
        }
    }
}

impl TryFrom<InitialStateDto> for RigidBodyState {
    type Error = AircraftReplayError;

    fn try_from(dto: InitialStateDto) -> Result<Self, Self::Error> {
        let [w, x, y, z] = dto.orientation_world_from_body_wxyz;
        let state = Self {
            position_world_m: array_to_vec3(dto.position_world_m),
            linear_velocity_world_mps: array_to_vec3(dto.linear_velocity_world_mps),
            orientation_world_from_body: Orientation::new_unchecked(Quaternion::new(w, x, y, z)),
            angular_velocity_body_radps: array_to_vec3(dto.angular_velocity_body_radps),
        };
        state
            .validate()
            .map_err(|error| AircraftReplayError::InvalidInitialState(error.to_string()))?;
        Ok(state)
    }
}

impl From<&AircraftReplayFrame> for FrameDto {
    fn from(frame: &AircraftReplayFrame) -> Self {
        Self {
            step_index: frame.step_index,
            pilot_input: frame.pilot_input.into(),
            expected_snapshot_hash: frame.expected_snapshot_hash.to_hex(),
        }
    }
}

impl TryFrom<FrameDto> for AircraftReplayFrame {
    type Error = AircraftReplayError;

    fn try_from(dto: FrameDto) -> Result<Self, Self::Error> {
        let hash_text = dto.expected_snapshot_hash;
        let hash = decode_hex_32(&hash_text)
            .ok_or_else(|| AircraftReplayError::MalformedSnapshotHash(hash_text.clone()))?;
        Ok(Self {
            step_index: dto.step_index,
            pilot_input: dto.pilot_input.try_into_with_step(dto.step_index)?,
            expected_snapshot_hash: AircraftSnapshotHash(hash),
        })
    }
}

impl From<PilotInput> for PilotInputDto {
    fn from(input: PilotInput) -> Self {
        Self {
            roll: input.roll(),
            pitch: input.pitch(),
            yaw: input.yaw(),
            throttle: input.throttle(),
        }
    }
}

impl PilotInputDto {
    fn try_into_with_step(self, step_index: u64) -> Result<PilotInput, AircraftReplayError> {
        let valid = [self.roll, self.pitch, self.yaw, self.throttle]
            .into_iter()
            .all(f64::is_finite)
            && (-1.0..=1.0).contains(&self.roll)
            && (-1.0..=1.0).contains(&self.pitch)
            && (-1.0..=1.0).contains(&self.yaw)
            && (0.0..=1.0).contains(&self.throttle);
        if !valid {
            return Err(AircraftReplayError::InvalidPilotInput { step_index });
        }
        Ok(PilotInput::new(
            self.roll,
            self.pitch,
            self.yaw,
            self.throttle,
        ))
    }
}

fn validate_recording(recording: &AircraftReplayRecording) -> Result<(), AircraftReplayError> {
    if recording.schema_version != AIRCRAFT_REPLAY_SCHEMA_VERSION {
        return Err(AircraftReplayError::UnsupportedSchema(
            recording.schema_version,
        ));
    }
    recording.simulation_config.to_runtime()?;
    recording
        .initial_rigid_body_state
        .validate()
        .map_err(|error| AircraftReplayError::InvalidInitialState(error.to_string()))?;
    for (expected, frame) in recording.frames.iter().enumerate() {
        let expected = expected as u64;
        if frame.step_index != expected {
            return Err(AircraftReplayError::NonContiguousStep {
                expected,
                actual: frame.step_index,
            });
        }
        if !frame.pilot_input.is_valid() {
            return Err(AircraftReplayError::InvalidPilotInput {
                step_index: frame.step_index,
            });
        }
    }
    Ok(())
}

fn validate_model(
    recording: &AircraftReplayRecording,
    model: &AircraftModel,
) -> Result<(), AircraftReplayError> {
    if model.model_id() != recording.model_id {
        return Err(AircraftReplayError::ModelIdMismatch {
            expected: recording.model_id.clone(),
            actual: model.model_id().to_owned(),
        });
    }
    let actual =
        AircraftModelPhysicsFingerprint::from_model_fingerprint(model.physics_fingerprint());
    if actual != recording.model_physics_fingerprint {
        return Err(AircraftReplayError::ModelPhysicsFingerprintMismatch {
            expected: recording.model_physics_fingerprint,
            actual,
        });
    }
    Ok(())
}

fn validate_simulation_setup(
    recording: &AircraftReplayRecording,
    simulation: &AircraftSimulation,
) -> Result<(), AircraftReplayError> {
    if simulation.step_index() != 0 {
        return Err(AircraftReplayError::SimulationNotAtInitialStep(
            simulation.step_index(),
        ));
    }
    validate_model(recording, simulation.model())?;
    if !config_bits_equal(
        &recording.simulation_config,
        &AircraftReplaySimulationConfig::from_runtime(simulation.config()),
    ) {
        return Err(AircraftReplayError::SimulationConfigMismatch);
    }
    if !state_bits_equal(
        &recording.initial_rigid_body_state,
        simulation.state().rigid_body(),
    ) {
        return Err(AircraftReplayError::InitialStateMismatch);
    }
    Ok(())
}

fn config_bits_equal(
    left: &AircraftReplaySimulationConfig,
    right: &AircraftReplaySimulationConfig,
) -> bool {
    left.dt_s.to_bits() == right.dt_s.to_bits()
        && array_bits_equal(left.gravity_world_mps2, right.gravity_world_mps2)
        && left.air_density_kg_m3.to_bits() == right.air_density_kg_m3.to_bits()
        && array_bits_equal(left.wind_velocity_world_mps, right.wind_velocity_world_mps)
}

fn state_bits_equal(left: &RigidBodyState, right: &RigidBodyState) -> bool {
    let left_quaternion = left.orientation_world_from_body.quaternion();
    let right_quaternion = right.orientation_world_from_body.quaternion();
    array_bits_equal(
        vec3_to_array(&left.position_world_m),
        vec3_to_array(&right.position_world_m),
    ) && array_bits_equal(
        vec3_to_array(&left.linear_velocity_world_mps),
        vec3_to_array(&right.linear_velocity_world_mps),
    ) && [
        left_quaternion.w,
        left_quaternion.i,
        left_quaternion.j,
        left_quaternion.k,
    ]
    .into_iter()
    .zip([
        right_quaternion.w,
        right_quaternion.i,
        right_quaternion.j,
        right_quaternion.k,
    ])
    .all(|(left, right)| left.to_bits() == right.to_bits())
        && array_bits_equal(
            vec3_to_array(&left.angular_velocity_body_radps),
            vec3_to_array(&right.angular_velocity_body_radps),
        )
}

fn array_bits_equal(left: [f64; 3], right: [f64; 3]) -> bool {
    left.into_iter()
        .zip(right)
        .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn update_f64(hasher: &mut blake3::Hasher, value: f64) {
    hasher.update(&value.to_bits().to_le_bytes());
}

fn update_vec3(hasher: &mut blake3::Hasher, vector: &Vec3) {
    for value in [vector.x, vector.y, vector.z] {
        update_f64(hasher, value);
    }
}

fn vec3_to_array(vector: &Vec3) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

fn array_to_vec3(values: [f64; 3]) -> Vec3 {
    Vec3::new(values[0], values[1], values[2])
}

fn encode_hex(bytes: &[u8; 32]) -> String {
    use fmt::Write as _;
    let mut text = String::with_capacity(64);
    for byte in bytes {
        write!(&mut text, "{byte:02x}").expect("writing into a String cannot fail");
    }
    text
}

fn decode_hex_32(text: &str) -> Option<[u8; 32]> {
    if text.len() != 64
        || !text
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    {
        return None;
    }
    let mut output = [0_u8; 32];
    for (index, byte) in output.iter_mut().enumerate() {
        let offset = index * 2;
        *byte = u8::from_str_radix(&text[offset..offset + 2], 16).ok()?;
    }
    Some(output)
}
