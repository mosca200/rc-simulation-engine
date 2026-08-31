use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSnapshot};
use model::{AircraftModel, load_aircraft_model};
#[cfg(not(target_os = "windows"))]
use replay::AircraftReplayRecorder;
use replay::{AircraftReplayPlayer, AircraftReplayRecording};
use serde_json::{Value, json};
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::path::PathBuf;
use telemetry::{
    AIRCRAFT_TELEMETRY_SCHEMA_VERSION, AircraftTelemetryRecorder, AircraftTelemetryRecording,
    TelemetryCaptureError,
};

const DATASET_STEPS: usize = 2_000;

fn repository_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn load_model() -> AircraftModel {
    load_aircraft_model(repository_root().join("models/acro_electric_01/model.json")).unwrap()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn simulation() -> AircraftSimulation {
    AircraftSimulation::new(
        load_model(),
        AircraftSimulationConfig::new(
            0.002,
            Vec3::new(-0.0, 0.0, 9.80665),
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap(),
        initial_state(),
    )
    .unwrap()
}

fn capture(
    steps: usize,
    input: PilotInput,
    with_timing: bool,
) -> (AircraftTelemetryRecording, Option<AircraftSnapshot>) {
    let mut simulation = simulation();
    let mut recorder = AircraftTelemetryRecorder::with_capacity(&simulation, steps).unwrap();
    let mut final_snapshot = None;
    for index in 0..steps {
        let snapshot = simulation.step(&input);
        recorder
            .record(
                &simulation,
                input,
                &snapshot,
                with_timing.then_some((index + 1) as f64 * 1.0e-6),
            )
            .unwrap();
        final_snapshot = Some(snapshot);
    }
    (recorder.finish(), final_snapshot)
}

fn json_lines_with_mutation(
    recording: &AircraftTelemetryRecording,
    line_index: usize,
    mutate: impl FnOnce(&mut Value),
) -> String {
    let mut lines: Vec<Value> = recording
        .to_json_lines()
        .unwrap()
        .lines()
        .map(|line| serde_json::from_str(line).unwrap())
        .collect();
    mutate(&mut lines[line_index]);
    let mut output = lines
        .iter()
        .map(|line| serde_json::to_string(line).unwrap())
        .collect::<Vec<_>>()
        .join("\n");
    output.push('\n');
    output
}

#[test]
fn frame_maps_exact_post_step_state_input_identity_and_units() {
    let input = PilotInput::new(0.3, -0.2, 0.1, 0.65);
    let mut simulation = simulation();
    let fingerprint = simulation.model().physics_fingerprint();
    let mut recorder = AircraftTelemetryRecorder::new(&simulation).unwrap();
    let snapshot = simulation.step(&input);
    recorder
        .record(&simulation, input, &snapshot, Some(0.000_123))
        .unwrap();
    let recording = recorder.finish();
    let frame = &recording.frames()[0];
    let rigid = snapshot.rigid_body_state();
    let quaternion = rigid.orientation_world_from_body.quaternion();
    let controls = snapshot.control_surface_positions();

    assert_eq!(
        recording.schema_version(),
        AIRCRAFT_TELEMETRY_SCHEMA_VERSION
    );
    assert_eq!(frame.schema_version(), AIRCRAFT_TELEMETRY_SCHEMA_VERSION);
    assert_eq!(frame.step_index(), 1);
    assert_eq!(frame.sim_time_s().to_bits(), 0.002_f64.to_bits());
    assert_eq!(frame.model_id(), simulation.model().model_id());
    assert_eq!(frame.model_physics_fingerprint().len(), 64);
    assert_eq!(
        recording.model_physics_fingerprint(),
        frame.model_physics_fingerprint()
    );
    assert_eq!(fingerprint.as_bytes().len(), 32);
    assert_eq!(frame.air_density_kg_m3(), 1.225);
    assert_eq!(frame.wind_velocity_world_ned_mps(), [0.0, 0.0, 0.0]);
    assert_eq!(frame.pilot_input(), input);
    assert_eq!(
        frame.position_world_ned_m(),
        [
            rigid.position_world_m.x,
            rigid.position_world_m.y,
            rigid.position_world_m.z
        ]
    );
    assert_eq!(
        frame.linear_velocity_world_ned_mps(),
        [
            rigid.linear_velocity_world_mps.x,
            rigid.linear_velocity_world_mps.y,
            rigid.linear_velocity_world_mps.z
        ]
    );
    assert_eq!(
        frame.orientation_world_from_body_hamilton_wxyz(),
        [quaternion.w, quaternion.i, quaternion.j, quaternion.k]
    );
    assert_eq!(
        frame.angular_velocity_body_frd_radps(),
        [
            rigid.angular_velocity_body_radps.x,
            rigid.angular_velocity_body_radps.y,
            rigid.angular_velocity_body_radps.z
        ]
    );
    assert_eq!(frame.aileron_angle_rad(), controls.aileron_angle_rad());
    assert_eq!(frame.elevator_angle_rad(), controls.elevator_angle_rad());
    assert_eq!(frame.rudder_angle_rad(), controls.rudder_angle_rad());
    assert_eq!(frame.throttle(), controls.throttle());
    assert_eq!(frame.physics_step_wall_time_s(), Some(0.000_123));
}

#[test]
fn jsonl_roundtrip_is_versioned_strict_and_stream_friendly() {
    let (recording, _) = capture(3, PilotInput::new(0.1, -0.2, 0.3, 0.7), false);
    let json_lines = recording.to_json_lines().unwrap();
    assert_eq!(json_lines.lines().count(), 4);
    assert!(json_lines.starts_with("{\"record_type\":\"aircraft_telemetry_header\""));
    assert!(json_lines.contains("\"record_type\":\"aircraft_telemetry_frame\""));
    assert_eq!(
        AircraftTelemetryRecording::from_json_lines(&json_lines).unwrap(),
        recording
    );

    let unknown_header = json_lines_with_mutation(&recording, 0, |header| {
        header["unknown"] = json!(true);
    });
    assert!(AircraftTelemetryRecording::from_json_lines(&unknown_header).is_err());
    let unknown_nested = json_lines_with_mutation(&recording, 1, |frame| {
        frame["rigid_body"]["unknown"] = json!(0);
    });
    assert!(AircraftTelemetryRecording::from_json_lines(&unknown_nested).is_err());
    let unsupported = json_lines_with_mutation(&recording, 0, |header| {
        header["schema_version"] = json!(99);
    });
    assert!(matches!(
        AircraftTelemetryRecording::from_json_lines(&unsupported),
        Err(TelemetryCaptureError::UnsupportedSchema(99))
    ));
}

#[test]
fn invalid_ranges_nonfinite_quaternion_and_steps_are_rejected_without_clamping() {
    let (recording, _) = capture(2, PilotInput::new(0.1, 0.2, 0.3, 0.6), false);
    let bad_input = json_lines_with_mutation(&recording, 1, |frame| {
        frame["pilot_input"]["roll"] = json!(1.1);
    });
    assert!(AircraftTelemetryRecording::from_json_lines(&bad_input).is_err());
    let bad_quaternion = json_lines_with_mutation(&recording, 1, |frame| {
        frame["rigid_body"]["orientation_world_from_body_hamilton_wxyz"] =
            json!([2.0, 0.0, 0.0, 0.0]);
    });
    assert!(AircraftTelemetryRecording::from_json_lines(&bad_quaternion).is_err());
    let bad_step = json_lines_with_mutation(&recording, 2, |frame| {
        frame["step_index"] = json!(4);
    });
    assert!(matches!(
        AircraftTelemetryRecording::from_json_lines(&bad_step),
        Err(TelemetryCaptureError::NonContiguousStep {
            expected: 2,
            actual: 4
        })
    ));
    let nonfinite = recording.to_json_lines().unwrap().replacen(
        "\"sim_time_s\":0.002",
        "\"sim_time_s\":1e999",
        1,
    );
    assert!(AircraftTelemetryRecording::from_json_lines(&nonfinite).is_err());
}

#[test]
fn zero_step_capture_has_a_valid_header_and_empty_summary() {
    let simulation = simulation();
    let recording = AircraftTelemetryRecorder::new(&simulation)
        .unwrap()
        .finish();
    let json_lines = recording.to_json_lines().unwrap();
    assert_eq!(json_lines.lines().count(), 1);
    let decoded = AircraftTelemetryRecording::from_json_lines(&json_lines).unwrap();
    assert!(decoded.frames().is_empty());
    let summary = decoded.summary().unwrap();
    assert_eq!(summary.deterministic.frame_count, 0);
    assert_eq!(summary.deterministic.simulated_duration_s, 0.0);
    assert!(summary.deterministic.final_state.is_none());
}

#[test]
fn analyzer_covers_two_thousand_frames_and_ned_altitude_semantics() {
    let input = PilotInput::new(0.12, -0.08, 0.05, 0.55);
    let (recording, final_snapshot) = capture(DATASET_STEPS, input, false);
    let summary = recording.summary().unwrap().deterministic;
    assert_eq!(summary.frame_count, DATASET_STEPS as u64);
    assert_eq!(summary.first_step, Some(1));
    assert_eq!(summary.last_step, Some(DATASET_STEPS as u64));
    assert_eq!(summary.simulated_duration_s.to_bits(), 4.0_f64.to_bits());
    let speed = summary.speed_mps.unwrap();
    assert!(speed.min <= speed.mean && speed.mean <= speed.max);
    let down = summary.down_m.unwrap();
    let altitude = summary.local_altitude_m.unwrap();
    assert_eq!(altitude.min.to_bits(), (-down.max).to_bits());
    assert_eq!(altitude.max.to_bits(), (-down.min).to_bits());
    assert!(summary.max_angular_speed_radps.unwrap().is_finite());
    assert_eq!(summary.max_abs_roll_input, input.roll().abs());
    assert_eq!(summary.max_abs_pitch_input, input.pitch().abs());
    assert_eq!(summary.max_abs_yaw_input, input.yaw().abs());
    assert_eq!(summary.throttle_input.unwrap().min, input.throttle());
    assert!(summary.aileron_angle_rad.is_some());
    assert!(summary.elevator_angle_rad.is_some());
    assert!(summary.rudder_angle_rad.is_some());

    let expected = final_snapshot.unwrap();
    let final_state = summary.final_state.unwrap();
    let rigid = expected.rigid_body_state();
    assert_eq!(
        final_state.position_world_ned_m,
        [
            rigid.position_world_m.x,
            rigid.position_world_m.y,
            rigid.position_world_m.z
        ]
    );
    assert_eq!(
        final_state.linear_velocity_world_ned_mps,
        [
            rigid.linear_velocity_world_mps.x,
            rigid.linear_velocity_world_mps.y,
            rigid.linear_velocity_world_mps.z
        ]
    );
}

#[test]
fn telemetry_observation_does_not_change_simulation_results() {
    let input = PilotInput::new(0.2, -0.15, 0.1, 0.6);
    let mut observed = simulation();
    let mut baseline = simulation();
    let mut recorder = AircraftTelemetryRecorder::with_capacity(&observed, 200).unwrap();
    for _ in 0..200 {
        let observed_snapshot = observed.step(&input);
        recorder
            .record(&observed, input, &observed_snapshot, None)
            .unwrap();
        let baseline_snapshot = baseline.step(&input);
        assert_eq!(observed_snapshot, baseline_snapshot);
    }
    assert_eq!(observed.state(), baseline.state());
}

#[test]
fn wall_clock_is_excluded_from_deterministic_summary() {
    let input = PilotInput::new(0.1, 0.0, -0.1, 0.5);
    let (with_timing, _) = capture(50, input, true);
    let (without_timing, _) = capture(50, input, false);
    let first = with_timing.summary().unwrap();
    let second = without_timing.summary().unwrap();
    assert_eq!(first.deterministic, second.deterministic);
    assert!(first.physics_step_wall_time_s.is_some());
    assert!(second.physics_step_wall_time_s.is_none());
}

#[test]
fn committed_replay_produces_repeatable_two_thousand_frame_telemetry() {
    let replay = load_committed_replay();
    assert_eq!(replay.frames().len(), DATASET_STEPS);

    #[cfg(not(target_os = "windows"))]
    let replay = {
        let mut simulation = replay.reconstruct_simulation(load_model()).unwrap();
        let mut recorder =
            AircraftReplayRecorder::with_capacity(&simulation, replay.frames().len()).unwrap();
        for frame in replay.frames() {
            recorder
                .record(&mut simulation, frame.step_index(), frame.pilot_input())
                .unwrap();
        }
        recorder.finish()
    };

    let first = capture_replay(&replay).unwrap();
    let second = capture_replay(&replay).unwrap();
    assert_eq!(first, second);
    assert_eq!(
        first.summary().unwrap().deterministic,
        second.summary().unwrap().deterministic
    );
    assert_eq!(first.frames().len(), DATASET_STEPS);
    assert_eq!(first.model_id(), replay.model_id());
    assert_eq!(
        first.model_physics_fingerprint(),
        replay.model_physics_fingerprint().to_hex()
    );
}

#[test]
fn replay_divergence_stops_before_telemetry_is_emitted_for_bad_step() {
    let replay_path =
        repository_root().join("tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json");
    let mut value: Value =
        serde_json::from_str(&std::fs::read_to_string(replay_path).unwrap()).unwrap();
    value["frames"][0]["expected_snapshot_hash"] =
        json!("0000000000000000000000000000000000000000000000000000000000000000");
    let replay =
        AircraftReplayRecording::from_json(&serde_json::to_string(&value).unwrap()).unwrap();
    let mut simulation = replay.reconstruct_simulation(load_model()).unwrap();
    let mut player = AircraftReplayPlayer::new(&replay, &simulation).unwrap();
    let recorder = AircraftTelemetryRecorder::new(&simulation).unwrap();
    assert!(player.verify_next(&mut simulation).is_err());
    assert!(recorder.finish().frames().is_empty());
}

#[test]
fn telemetry_crate_has_no_renderer_platform_or_gpu_dependency() {
    let manifest =
        std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml"))
            .unwrap();
    for forbidden in ["renderer", "wgpu", "winit", "platform", "gilrs"] {
        assert!(
            !manifest.lines().any(|line| {
                line.split_once('=')
                    .is_some_and(|(name, _)| name.trim() == forbidden)
            }),
            "forbidden telemetry dependency {forbidden}"
        );
    }
}

fn load_committed_replay() -> AircraftReplayRecording {
    let path =
        repository_root().join("tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json");
    AircraftReplayRecording::from_json(&std::fs::read_to_string(path).unwrap()).unwrap()
}

fn capture_replay(
    replay: &AircraftReplayRecording,
) -> Result<AircraftTelemetryRecording, replay::AircraftReplayError> {
    let mut simulation = replay.reconstruct_simulation(load_model())?;
    let mut player = AircraftReplayPlayer::new(replay, &simulation)?;
    let mut recorder =
        AircraftTelemetryRecorder::with_capacity(&simulation, replay.frames().len()).unwrap();
    for replay_frame in replay.frames() {
        let snapshot = player.verify_next(&mut simulation)?.unwrap();
        recorder
            .record(&simulation, replay_frame.pilot_input(), &snapshot, None)
            .unwrap();
    }
    Ok(recorder.finish())
}
