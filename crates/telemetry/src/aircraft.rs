use aircraft::{AircraftSimulation, AircraftSnapshot};
use serde::{Deserialize, Serialize};
use sim_core::PilotInput;
use std::fmt::Write as _;
use thiserror::Error;

pub const AIRCRAFT_TELEMETRY_SCHEMA_VERSION: u32 = 1;
const HEADER_RECORD_TYPE: &str = "aircraft_telemetry_header";
const FRAME_RECORD_TYPE: &str = "aircraft_telemetry_frame";

/// One post-step aircraft observation and the exact input applied at its pre-step.
#[derive(Debug, Clone, PartialEq)]
pub struct AircraftTelemetryFrame {
    schema_version: u32,
    step_index: u64,
    sim_time_s: f64,
    model_id: String,
    model_physics_fingerprint: String,
    air_density_kg_m3: f64,
    wind_velocity_world_ned_mps: [f64; 3],
    pilot_input: PilotInput,
    position_world_ned_m: [f64; 3],
    linear_velocity_world_ned_mps: [f64; 3],
    orientation_world_from_body_hamilton_wxyz: [f64; 4],
    angular_velocity_body_frd_radps: [f64; 3],
    aileron_angle_rad: f64,
    elevator_angle_rad: f64,
    rudder_angle_rad: f64,
    throttle: f64,
    physics_step_wall_time_s: Option<f64>,
}

impl AircraftTelemetryFrame {
    pub fn from_snapshot(
        simulation: &AircraftSimulation,
        pilot_input: PilotInput,
        snapshot: &AircraftSnapshot,
        physics_step_wall_time_s: Option<f64>,
    ) -> Result<Self, TelemetryCaptureError> {
        let model = simulation.model();
        let environment = simulation.config().aero_environment();
        let rigid_body = snapshot.rigid_body_state();
        let quaternion = rigid_body.orientation_world_from_body.quaternion();
        let controls = snapshot.control_surface_positions();
        let frame = Self {
            schema_version: AIRCRAFT_TELEMETRY_SCHEMA_VERSION,
            step_index: snapshot.step_index(),
            sim_time_s: snapshot.sim_time_s(),
            model_id: model.model_id().to_owned(),
            model_physics_fingerprint: encode_hex(model.physics_fingerprint().as_bytes()),
            air_density_kg_m3: environment.air_density_kg_m3(),
            wind_velocity_world_ned_mps: vec3_to_array(environment.wind_velocity_world_mps()),
            pilot_input,
            position_world_ned_m: vec3_to_array(&rigid_body.position_world_m),
            linear_velocity_world_ned_mps: vec3_to_array(&rigid_body.linear_velocity_world_mps),
            orientation_world_from_body_hamilton_wxyz: [
                quaternion.w,
                quaternion.i,
                quaternion.j,
                quaternion.k,
            ],
            angular_velocity_body_frd_radps: vec3_to_array(&rigid_body.angular_velocity_body_radps),
            aileron_angle_rad: controls.aileron_angle_rad(),
            elevator_angle_rad: controls.elevator_angle_rad(),
            rudder_angle_rad: controls.rudder_angle_rad(),
            throttle: controls.throttle(),
            physics_step_wall_time_s,
        };
        validate_frame(&frame)?;
        Ok(frame)
    }

    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    #[must_use]
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    #[must_use]
    pub const fn sim_time_s(&self) -> f64 {
        self.sim_time_s
    }

    #[must_use]
    pub fn model_id(&self) -> &str {
        &self.model_id
    }

    #[must_use]
    pub fn model_physics_fingerprint(&self) -> &str {
        &self.model_physics_fingerprint
    }

    #[must_use]
    pub const fn air_density_kg_m3(&self) -> f64 {
        self.air_density_kg_m3
    }

    #[must_use]
    pub const fn wind_velocity_world_ned_mps(&self) -> [f64; 3] {
        self.wind_velocity_world_ned_mps
    }

    #[must_use]
    pub const fn pilot_input(&self) -> PilotInput {
        self.pilot_input
    }

    #[must_use]
    pub const fn position_world_ned_m(&self) -> [f64; 3] {
        self.position_world_ned_m
    }

    #[must_use]
    pub const fn linear_velocity_world_ned_mps(&self) -> [f64; 3] {
        self.linear_velocity_world_ned_mps
    }

    #[must_use]
    pub const fn orientation_world_from_body_hamilton_wxyz(&self) -> [f64; 4] {
        self.orientation_world_from_body_hamilton_wxyz
    }

    #[must_use]
    pub const fn angular_velocity_body_frd_radps(&self) -> [f64; 3] {
        self.angular_velocity_body_frd_radps
    }

    #[must_use]
    pub const fn aileron_angle_rad(&self) -> f64 {
        self.aileron_angle_rad
    }

    #[must_use]
    pub const fn elevator_angle_rad(&self) -> f64 {
        self.elevator_angle_rad
    }

    #[must_use]
    pub const fn rudder_angle_rad(&self) -> f64 {
        self.rudder_angle_rad
    }

    #[must_use]
    pub const fn throttle(&self) -> f64 {
        self.throttle
    }

    /// Non-deterministic performance datum, excluded from deterministic summaries.
    #[must_use]
    pub const fn physics_step_wall_time_s(&self) -> Option<f64> {
        self.physics_step_wall_time_s
    }
}

/// Versioned in-memory telemetry capture. Replay remains the deterministic source of truth.
#[derive(Debug, Clone, PartialEq)]
pub struct AircraftTelemetryRecording {
    schema_version: u32,
    model_id: String,
    model_physics_fingerprint: String,
    physics_dt_s: f64,
    frames: Vec<AircraftTelemetryFrame>,
}

impl AircraftTelemetryRecording {
    pub fn to_json_lines(&self) -> Result<String, TelemetryCaptureError> {
        validate_recording(self)?;
        let header = HeaderDto::from(self);
        let mut output = serde_json::to_string(&header)?;
        output.push('\n');
        for frame in &self.frames {
            output.push_str(&serde_json::to_string(&FrameDto::from(frame))?);
            output.push('\n');
        }
        Ok(output)
    }

    pub fn from_json_lines(json_lines: &str) -> Result<Self, TelemetryCaptureError> {
        let mut lines = json_lines.lines().enumerate();
        let Some((_, header_line)) = lines.next() else {
            return Err(TelemetryCaptureError::MissingHeader);
        };
        if header_line.trim().is_empty() {
            return Err(TelemetryCaptureError::MissingHeader);
        }
        let header: HeaderDto = serde_json::from_str(header_line)
            .map_err(|source| TelemetryCaptureError::JsonLine { line: 1, source })?;
        if header.record_type != HEADER_RECORD_TYPE {
            return Err(TelemetryCaptureError::UnexpectedRecordType {
                line: 1,
                expected: HEADER_RECORD_TYPE,
                actual: header.record_type,
            });
        }
        let mut frames = Vec::new();
        for (line_index, line) in lines {
            let line_number = line_index + 1;
            if line.trim().is_empty() {
                return Err(TelemetryCaptureError::BlankLine(line_number));
            }
            let dto: FrameDto =
                serde_json::from_str(line).map_err(|source| TelemetryCaptureError::JsonLine {
                    line: line_number,
                    source,
                })?;
            if dto.record_type != FRAME_RECORD_TYPE {
                return Err(TelemetryCaptureError::UnexpectedRecordType {
                    line: line_number,
                    expected: FRAME_RECORD_TYPE,
                    actual: dto.record_type,
                });
            }
            frames.push(AircraftTelemetryFrame::try_from(dto)?);
        }
        let recording = Self {
            schema_version: header.schema_version,
            model_id: header.model_id,
            model_physics_fingerprint: header.model_physics_fingerprint,
            physics_dt_s: header.physics_dt_s,
            frames,
        };
        validate_recording(&recording)?;
        Ok(recording)
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
    pub fn model_physics_fingerprint(&self) -> &str {
        &self.model_physics_fingerprint
    }

    #[must_use]
    pub const fn physics_dt_s(&self) -> f64 {
        self.physics_dt_s
    }

    #[must_use]
    pub fn frames(&self) -> &[AircraftTelemetryFrame] {
        &self.frames
    }

    pub fn summary(&self) -> Result<TelemetrySummary, TelemetryCaptureError> {
        validate_recording(self)?;
        Ok(summarize(&self.frames))
    }
}

/// Capture helper kept outside `AircraftSimulation::step()` and its allocation-free hot loop.
pub struct AircraftTelemetryRecorder {
    recording: AircraftTelemetryRecording,
}

impl AircraftTelemetryRecorder {
    pub fn new(simulation: &AircraftSimulation) -> Result<Self, TelemetryCaptureError> {
        Self::with_capacity(simulation, 0)
    }

    pub fn with_capacity(
        simulation: &AircraftSimulation,
        frame_capacity: usize,
    ) -> Result<Self, TelemetryCaptureError> {
        if simulation.step_index() != 0 {
            return Err(TelemetryCaptureError::SimulationNotAtInitialStep(
                simulation.step_index(),
            ));
        }
        let model = simulation.model();
        Ok(Self {
            recording: AircraftTelemetryRecording {
                schema_version: AIRCRAFT_TELEMETRY_SCHEMA_VERSION,
                model_id: model.model_id().to_owned(),
                model_physics_fingerprint: encode_hex(model.physics_fingerprint().as_bytes()),
                physics_dt_s: simulation.config().dt_s(),
                frames: Vec::with_capacity(frame_capacity),
            },
        })
    }

    /// Associates pre-step input `N` with the already-committed post-step snapshot `N + 1`.
    pub fn record(
        &mut self,
        simulation: &AircraftSimulation,
        pilot_input: PilotInput,
        snapshot: &AircraftSnapshot,
        physics_step_wall_time_s: Option<f64>,
    ) -> Result<(), TelemetryCaptureError> {
        let expected_snapshot_step = self.recording.frames.len() as u64 + 1;
        if snapshot.step_index() != expected_snapshot_step {
            return Err(TelemetryCaptureError::NonContiguousStep {
                expected: expected_snapshot_step,
                actual: snapshot.step_index(),
            });
        }
        if simulation.step_index() != snapshot.step_index() {
            return Err(TelemetryCaptureError::SimulationSnapshotStepMismatch {
                simulation: simulation.step_index(),
                snapshot: snapshot.step_index(),
            });
        }
        if simulation.model().model_id() != self.recording.model_id
            || encode_hex(simulation.model().physics_fingerprint().as_bytes())
                != self.recording.model_physics_fingerprint
        {
            return Err(TelemetryCaptureError::SimulationIdentityMismatch);
        }
        if simulation.config().dt_s().to_bits() != self.recording.physics_dt_s.to_bits() {
            return Err(TelemetryCaptureError::SimulationConfigMismatch);
        }
        let frame = AircraftTelemetryFrame::from_snapshot(
            simulation,
            pilot_input,
            snapshot,
            physics_step_wall_time_s,
        )?;
        let expected_time = snapshot.step_index() as f64 * self.recording.physics_dt_s;
        if frame.sim_time_s.to_bits() != expected_time.to_bits() {
            return Err(TelemetryCaptureError::InconsistentSimTime {
                step_index: frame.step_index,
            });
        }
        self.recording.frames.push(frame);
        Ok(())
    }

    #[must_use]
    pub fn finish(self) -> AircraftTelemetryRecording {
        self.recording
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScalarRange {
    pub min: f64,
    pub max: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScalarStatistics {
    pub min: f64,
    pub max: f64,
    pub mean: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryFinalState {
    pub position_world_ned_m: [f64; 3],
    pub linear_velocity_world_ned_mps: [f64; 3],
    pub orientation_world_from_body_hamilton_wxyz: [f64; 4],
    pub angular_velocity_body_frd_radps: [f64; 3],
}

/// Physics-only regression summary; deliberately excludes wall-clock measurements.
#[derive(Debug, Clone, PartialEq)]
pub struct DeterministicTelemetrySummary {
    pub frame_count: u64,
    pub first_step: Option<u64>,
    pub last_step: Option<u64>,
    pub simulated_duration_s: f64,
    pub speed_mps: Option<ScalarStatistics>,
    pub north_m: Option<ScalarRange>,
    pub east_m: Option<ScalarRange>,
    pub down_m: Option<ScalarRange>,
    pub local_altitude_m: Option<ScalarRange>,
    pub max_angular_speed_radps: Option<f64>,
    pub max_abs_roll_input: f64,
    pub max_abs_pitch_input: f64,
    pub max_abs_yaw_input: f64,
    pub throttle_input: Option<ScalarRange>,
    pub aileron_angle_rad: Option<ScalarRange>,
    pub elevator_angle_rad: Option<ScalarRange>,
    pub rudder_angle_rad: Option<ScalarRange>,
    pub final_state: Option<TelemetryFinalState>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TelemetrySummary {
    pub deterministic: DeterministicTelemetrySummary,
    /// Non-deterministic performance data, computed only from present timing samples.
    pub physics_step_wall_time_s: Option<ScalarStatistics>,
}

#[derive(Debug, Error)]
pub enum TelemetryCaptureError {
    #[error("telemetry capture is missing its header")]
    MissingHeader,
    #[error("telemetry capture contains a blank line at line {0}")]
    BlankLine(usize),
    #[error("unsupported aircraft telemetry schema version {0}")]
    UnsupportedSchema(u32),
    #[error(
        "unexpected telemetry record type at line {line}: expected `{expected}`, got `{actual}`"
    )]
    UnexpectedRecordType {
        line: usize,
        expected: &'static str,
        actual: String,
    },
    #[error("invalid telemetry JSON at line {line}: {source}")]
    JsonLine {
        line: usize,
        #[source]
        source: serde_json::Error,
    },
    #[error("telemetry JSON serialization failed: {0}")]
    Json(#[from] serde_json::Error),
    #[error("aircraft simulation must be at initial step zero, got step {0}")]
    SimulationNotAtInitialStep(u64),
    #[error("non-contiguous telemetry step: expected {expected}, got {actual}")]
    NonContiguousStep { expected: u64, actual: u64 },
    #[error("simulation step {simulation} does not match snapshot step {snapshot}")]
    SimulationSnapshotStepMismatch { simulation: u64, snapshot: u64 },
    #[error("telemetry recorder aircraft identity differs from the simulation")]
    SimulationIdentityMismatch,
    #[error("telemetry recorder simulation configuration differs from the simulation")]
    SimulationConfigMismatch,
    #[error("telemetry model ID is empty")]
    EmptyModelId,
    #[error("malformed model physics fingerprint `{0}`")]
    MalformedModelPhysicsFingerprint(String),
    #[error("telemetry frame {step_index} contains invalid {field}")]
    InvalidFrame {
        step_index: u64,
        field: &'static str,
    },
    #[error("telemetry frame {step_index} has simulation time inconsistent with its fixed step")]
    InconsistentSimTime { step_index: u64 },
    #[error("telemetry frame {step_index} model identity differs from the capture header")]
    FrameIdentityMismatch { step_index: u64 },
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct HeaderDto {
    record_type: String,
    schema_version: u32,
    model_id: String,
    model_physics_fingerprint: String,
    physics_dt_s: f64,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FrameDto {
    record_type: String,
    schema_version: u32,
    step_index: u64,
    sim_time_s: f64,
    model_id: String,
    model_physics_fingerprint: String,
    environment: EnvironmentDto,
    pilot_input: PilotInputDto,
    rigid_body: RigidBodyDto,
    controls: ControlsDto,
    #[serde(skip_serializing_if = "Option::is_none")]
    physics_step_wall_time_s: Option<f64>,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct PilotInputDto {
    roll: f64,
    pitch: f64,
    yaw: f64,
    throttle: f64,
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct EnvironmentDto {
    air_density_kg_m3: f64,
    wind_velocity_world_ned_mps: [f64; 3],
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RigidBodyDto {
    position_world_ned_m: [f64; 3],
    linear_velocity_world_ned_mps: [f64; 3],
    orientation_world_from_body_hamilton_wxyz: [f64; 4],
    angular_velocity_body_frd_radps: [f64; 3],
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ControlsDto {
    aileron_angle_rad: f64,
    elevator_angle_rad: f64,
    rudder_angle_rad: f64,
    throttle: f64,
}

impl From<&AircraftTelemetryRecording> for HeaderDto {
    fn from(recording: &AircraftTelemetryRecording) -> Self {
        Self {
            record_type: HEADER_RECORD_TYPE.to_owned(),
            schema_version: recording.schema_version,
            model_id: recording.model_id.clone(),
            model_physics_fingerprint: recording.model_physics_fingerprint.clone(),
            physics_dt_s: recording.physics_dt_s,
        }
    }
}

impl From<&AircraftTelemetryFrame> for FrameDto {
    fn from(frame: &AircraftTelemetryFrame) -> Self {
        Self {
            record_type: FRAME_RECORD_TYPE.to_owned(),
            schema_version: frame.schema_version,
            step_index: frame.step_index,
            sim_time_s: frame.sim_time_s,
            model_id: frame.model_id.clone(),
            model_physics_fingerprint: frame.model_physics_fingerprint.clone(),
            environment: EnvironmentDto {
                air_density_kg_m3: frame.air_density_kg_m3,
                wind_velocity_world_ned_mps: frame.wind_velocity_world_ned_mps,
            },
            pilot_input: PilotInputDto {
                roll: frame.pilot_input.roll(),
                pitch: frame.pilot_input.pitch(),
                yaw: frame.pilot_input.yaw(),
                throttle: frame.pilot_input.throttle(),
            },
            rigid_body: RigidBodyDto {
                position_world_ned_m: frame.position_world_ned_m,
                linear_velocity_world_ned_mps: frame.linear_velocity_world_ned_mps,
                orientation_world_from_body_hamilton_wxyz: frame
                    .orientation_world_from_body_hamilton_wxyz,
                angular_velocity_body_frd_radps: frame.angular_velocity_body_frd_radps,
            },
            controls: ControlsDto {
                aileron_angle_rad: frame.aileron_angle_rad,
                elevator_angle_rad: frame.elevator_angle_rad,
                rudder_angle_rad: frame.rudder_angle_rad,
                throttle: frame.throttle,
            },
            physics_step_wall_time_s: frame.physics_step_wall_time_s,
        }
    }
}

impl TryFrom<FrameDto> for AircraftTelemetryFrame {
    type Error = TelemetryCaptureError;

    fn try_from(dto: FrameDto) -> Result<Self, Self::Error> {
        let raw_input = [
            dto.pilot_input.roll,
            dto.pilot_input.pitch,
            dto.pilot_input.yaw,
            dto.pilot_input.throttle,
        ];
        let valid_input = raw_input.into_iter().all(f64::is_finite)
            && (-1.0..=1.0).contains(&dto.pilot_input.roll)
            && (-1.0..=1.0).contains(&dto.pilot_input.pitch)
            && (-1.0..=1.0).contains(&dto.pilot_input.yaw)
            && (0.0..=1.0).contains(&dto.pilot_input.throttle);
        if !valid_input {
            return Err(TelemetryCaptureError::InvalidFrame {
                step_index: dto.step_index,
                field: "pilot_input",
            });
        }
        let frame = Self {
            schema_version: dto.schema_version,
            step_index: dto.step_index,
            sim_time_s: dto.sim_time_s,
            model_id: dto.model_id,
            model_physics_fingerprint: dto.model_physics_fingerprint,
            air_density_kg_m3: dto.environment.air_density_kg_m3,
            wind_velocity_world_ned_mps: dto.environment.wind_velocity_world_ned_mps,
            pilot_input: PilotInput::new(
                dto.pilot_input.roll,
                dto.pilot_input.pitch,
                dto.pilot_input.yaw,
                dto.pilot_input.throttle,
            ),
            position_world_ned_m: dto.rigid_body.position_world_ned_m,
            linear_velocity_world_ned_mps: dto.rigid_body.linear_velocity_world_ned_mps,
            orientation_world_from_body_hamilton_wxyz: dto
                .rigid_body
                .orientation_world_from_body_hamilton_wxyz,
            angular_velocity_body_frd_radps: dto.rigid_body.angular_velocity_body_frd_radps,
            aileron_angle_rad: dto.controls.aileron_angle_rad,
            elevator_angle_rad: dto.controls.elevator_angle_rad,
            rudder_angle_rad: dto.controls.rudder_angle_rad,
            throttle: dto.controls.throttle,
            physics_step_wall_time_s: dto.physics_step_wall_time_s,
        };
        validate_frame(&frame)?;
        Ok(frame)
    }
}

fn validate_recording(recording: &AircraftTelemetryRecording) -> Result<(), TelemetryCaptureError> {
    if recording.schema_version != AIRCRAFT_TELEMETRY_SCHEMA_VERSION {
        return Err(TelemetryCaptureError::UnsupportedSchema(
            recording.schema_version,
        ));
    }
    validate_identity(&recording.model_id, &recording.model_physics_fingerprint)?;
    if !recording.physics_dt_s.is_finite() || recording.physics_dt_s <= 0.0 {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: 0,
            field: "physics_dt_s",
        });
    }
    for (index, frame) in recording.frames.iter().enumerate() {
        validate_frame(frame)?;
        let expected = index as u64 + 1;
        if frame.step_index != expected {
            return Err(TelemetryCaptureError::NonContiguousStep {
                expected,
                actual: frame.step_index,
            });
        }
        if frame.schema_version != recording.schema_version
            || frame.model_id != recording.model_id
            || frame.model_physics_fingerprint != recording.model_physics_fingerprint
        {
            return Err(TelemetryCaptureError::FrameIdentityMismatch {
                step_index: frame.step_index,
            });
        }
        let expected_time = frame.step_index as f64 * recording.physics_dt_s;
        if frame.sim_time_s.to_bits() != expected_time.to_bits() {
            return Err(TelemetryCaptureError::InconsistentSimTime {
                step_index: frame.step_index,
            });
        }
    }
    Ok(())
}

fn validate_identity(model_id: &str, fingerprint: &str) -> Result<(), TelemetryCaptureError> {
    if model_id.is_empty() {
        return Err(TelemetryCaptureError::EmptyModelId);
    }
    if fingerprint.len() != 64
        || !fingerprint
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    {
        return Err(TelemetryCaptureError::MalformedModelPhysicsFingerprint(
            fingerprint.to_owned(),
        ));
    }
    Ok(())
}

fn validate_frame(frame: &AircraftTelemetryFrame) -> Result<(), TelemetryCaptureError> {
    if frame.schema_version != AIRCRAFT_TELEMETRY_SCHEMA_VERSION {
        return Err(TelemetryCaptureError::UnsupportedSchema(
            frame.schema_version,
        ));
    }
    validate_identity(&frame.model_id, &frame.model_physics_fingerprint)?;
    if frame.step_index == 0 {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "post_step_index",
        });
    }
    if !frame.sim_time_s.is_finite() || frame.sim_time_s < 0.0 {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "sim_time_s",
        });
    }
    if !frame.air_density_kg_m3.is_finite()
        || frame.air_density_kg_m3 < 0.0
        || !frame
            .wind_velocity_world_ned_mps
            .iter()
            .all(|value| value.is_finite())
    {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "environment",
        });
    }
    if !frame.pilot_input.is_valid() {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "pilot_input",
        });
    }
    for (field, values) in [
        ("position_world_ned_m", &frame.position_world_ned_m[..]),
        (
            "linear_velocity_world_ned_mps",
            &frame.linear_velocity_world_ned_mps[..],
        ),
        (
            "orientation_world_from_body_hamilton_wxyz",
            &frame.orientation_world_from_body_hamilton_wxyz[..],
        ),
        (
            "angular_velocity_body_frd_radps",
            &frame.angular_velocity_body_frd_radps[..],
        ),
    ] {
        if !values.iter().all(|value| value.is_finite()) {
            return Err(TelemetryCaptureError::InvalidFrame {
                step_index: frame.step_index,
                field,
            });
        }
    }
    let quaternion_norm_squared = frame
        .orientation_world_from_body_hamilton_wxyz
        .iter()
        .map(|value| value * value)
        .sum::<f64>();
    if quaternion_norm_squared <= f64::EPSILON || (quaternion_norm_squared - 1.0).abs() > 1.0e-12 {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "orientation_world_from_body_hamilton_wxyz",
        });
    }
    if ![
        frame.aileron_angle_rad,
        frame.elevator_angle_rad,
        frame.rudder_angle_rad,
        frame.throttle,
    ]
    .into_iter()
    .all(f64::is_finite)
        || !(0.0..=1.0).contains(&frame.throttle)
    {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "controls",
        });
    }
    if frame
        .physics_step_wall_time_s
        .is_some_and(|value| !value.is_finite() || value < 0.0)
    {
        return Err(TelemetryCaptureError::InvalidFrame {
            step_index: frame.step_index,
            field: "physics_step_wall_time_s",
        });
    }
    Ok(())
}

fn summarize(frames: &[AircraftTelemetryFrame]) -> TelemetrySummary {
    let mut speed = StatisticsAccumulator::default();
    let mut north = RangeAccumulator::default();
    let mut east = RangeAccumulator::default();
    let mut down = RangeAccumulator::default();
    let mut altitude = RangeAccumulator::default();
    let mut angular_speed = RangeAccumulator::default();
    let mut input_throttle = RangeAccumulator::default();
    let mut aileron = RangeAccumulator::default();
    let mut elevator = RangeAccumulator::default();
    let mut rudder = RangeAccumulator::default();
    let mut timing = StatisticsAccumulator::default();
    let mut max_abs_roll: f64 = 0.0;
    let mut max_abs_pitch: f64 = 0.0;
    let mut max_abs_yaw: f64 = 0.0;

    for frame in frames {
        speed.add(magnitude(frame.linear_velocity_world_ned_mps));
        north.add(frame.position_world_ned_m[0]);
        east.add(frame.position_world_ned_m[1]);
        down.add(frame.position_world_ned_m[2]);
        altitude.add(-frame.position_world_ned_m[2]);
        angular_speed.add(magnitude(frame.angular_velocity_body_frd_radps));
        max_abs_roll = max_abs_roll.max(frame.pilot_input.roll().abs());
        max_abs_pitch = max_abs_pitch.max(frame.pilot_input.pitch().abs());
        max_abs_yaw = max_abs_yaw.max(frame.pilot_input.yaw().abs());
        input_throttle.add(frame.pilot_input.throttle());
        aileron.add(frame.aileron_angle_rad);
        elevator.add(frame.elevator_angle_rad);
        rudder.add(frame.rudder_angle_rad);
        if let Some(value) = frame.physics_step_wall_time_s {
            timing.add(value);
        }
    }

    let final_state = frames.last().map(|frame| TelemetryFinalState {
        position_world_ned_m: frame.position_world_ned_m,
        linear_velocity_world_ned_mps: frame.linear_velocity_world_ned_mps,
        orientation_world_from_body_hamilton_wxyz: frame.orientation_world_from_body_hamilton_wxyz,
        angular_velocity_body_frd_radps: frame.angular_velocity_body_frd_radps,
    });
    TelemetrySummary {
        deterministic: DeterministicTelemetrySummary {
            frame_count: frames.len() as u64,
            first_step: frames.first().map(|frame| frame.step_index),
            last_step: frames.last().map(|frame| frame.step_index),
            simulated_duration_s: frames.last().map_or(0.0, |frame| frame.sim_time_s),
            speed_mps: speed.finish(),
            north_m: north.finish(),
            east_m: east.finish(),
            down_m: down.finish(),
            local_altitude_m: altitude.finish(),
            max_angular_speed_radps: angular_speed.finish().map(|range| range.max),
            max_abs_roll_input: max_abs_roll,
            max_abs_pitch_input: max_abs_pitch,
            max_abs_yaw_input: max_abs_yaw,
            throttle_input: input_throttle.finish(),
            aileron_angle_rad: aileron.finish(),
            elevator_angle_rad: elevator.finish(),
            rudder_angle_rad: rudder.finish(),
            final_state,
        },
        physics_step_wall_time_s: timing.finish(),
    }
}

#[derive(Default)]
struct RangeAccumulator {
    range: Option<ScalarRange>,
}

impl RangeAccumulator {
    fn add(&mut self, value: f64) {
        if let Some(range) = &mut self.range {
            range.min = range.min.min(value);
            range.max = range.max.max(value);
        } else {
            self.range = Some(ScalarRange {
                min: value,
                max: value,
            });
        }
    }

    fn finish(self) -> Option<ScalarRange> {
        self.range
    }
}

#[derive(Default)]
struct StatisticsAccumulator {
    range: RangeAccumulator,
    sum: f64,
    compensation: f64,
    count: u64,
}

impl StatisticsAccumulator {
    fn add(&mut self, value: f64) {
        self.range.add(value);
        let corrected = value - self.compensation;
        let new_sum = self.sum + corrected;
        self.compensation = (new_sum - self.sum) - corrected;
        self.sum = new_sum;
        self.count += 1;
    }

    fn finish(self) -> Option<ScalarStatistics> {
        let range = self.range.finish()?;
        Some(ScalarStatistics {
            min: range.min,
            max: range.max,
            mean: self.sum / self.count as f64,
        })
    }
}

fn magnitude(vector: [f64; 3]) -> f64 {
    vector
        .into_iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt()
}

fn vec3_to_array(vector: &sim_math::Vec3) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

fn encode_hex(bytes: &[u8; 32]) -> String {
    let mut text = String::with_capacity(64);
    for byte in bytes {
        write!(&mut text, "{byte:02x}").expect("writing into a String cannot fail");
    }
    text
}
