use replay::{ReplayPlayer, ReplayRecorder};
use sim_core::{PilotInput, RigidBodyParams, RigidBodyState, Simulation, SimulationConfig};
use sim_math::{Mat3, Orientation, Vec3};

#[test]
fn t11_json_roundtrip_replays_identical_snapshot_hashes() {
    const STEPS: u64 = 2_000;
    let config = SimulationConfig::from_physics_hz(500).unwrap();
    let initial_state = RigidBodyState {
        position_world_m: Vec3::new(5.0, -3.0, -25.0),
        linear_velocity_world_mps: Vec3::new(10.0, 1.0, -2.0),
        orientation_world_from_body: Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.3)),
        angular_velocity_body_radps: Vec3::new(0.2, -0.1, 0.4),
    };
    let make_params =
        || RigidBodyParams::new(3.0, Mat3::from_diagonal(&Vec3::new(0.2, 0.3, 0.4))).unwrap();

    let mut original = Simulation::new(config, make_params(), initial_state).unwrap();
    let mut recorder = ReplayRecorder::with_capacity(&original, STEPS as usize).unwrap();
    let mut expected_hashes = Vec::with_capacity(STEPS as usize);
    for step_index in 0..STEPS {
        let phase = step_index as f64 * 0.013;
        let input = PilotInput::new(
            phase.sin(),
            (phase * 0.7).cos(),
            (phase * 0.3).sin(),
            (0.5 + 0.4 * phase.sin()).clamp(0.0, 1.0),
        );
        recorder.record(step_index, input).unwrap();
        expected_hashes.push(original.step(&input).state_hash());
    }

    let json = recorder.finish().to_json_pretty().unwrap();
    let decoded = replay::ReplayRecording::from_json(&json).unwrap();
    assert_eq!(decoded.initial_state, initial_state);
    assert_eq!(decoded.simulation_config.dt_s(), config.dt_s());

    let mut replayed = Simulation::new(
        decoded.simulation_config,
        make_params(),
        decoded.initial_state,
    )
    .unwrap();
    let mut player = ReplayPlayer::new(&decoded, &replayed).unwrap();
    for expected_hash in expected_hashes {
        let frame = player.next_input().unwrap();
        let actual_hash = replayed.step(&frame.pilot_input).state_hash();
        assert_eq!(actual_hash, expected_hash, "step {}", frame.step_index);
    }
    assert_eq!(player.remaining(), 0);
}

#[test]
fn replay_deserialization_rejects_invalid_pilot_input_instead_of_clamping() {
    let (mut recording, _, _) = fingerprint_recording();
    recording.frames.push(replay::ReplayFrame {
        step_index: 0,
        pilot_input: PilotInput::neutral(),
    });
    let valid_json = recording.to_json_pretty().unwrap();
    for (field, invalid_value) in [
        ("roll", "1.01"),
        ("pitch", "-1.01"),
        ("throttle", "1.01"),
        ("yaw", "1e400"),
        ("roll", "NaN"),
    ] {
        let original = format!(r#""{field}": 0.0"#);
        let replacement = format!(r#""{field}": {invalid_value}"#);
        let json = valid_json.replacen(&original, &replacement, 1);
        assert_ne!(json, valid_json, "missing replay field {field}");
        assert!(
            replay::ReplayRecording::from_json(&json).is_err(),
            "accepted {field}={invalid_value}"
        );
    }
}

#[test]
fn valid_pilot_input_json_roundtrip_preserves_bits_and_field_names() {
    let expected = PilotInput::new(-0.75, 0.125, -0.0, 0.625);
    let json = serde_json::to_string(&expected).unwrap();
    assert!(json.contains(r#""roll""#));
    assert!(json.contains(r#""pitch""#));
    assert!(json.contains(r#""yaw""#));
    assert!(json.contains(r#""throttle""#));
    let actual: PilotInput = serde_json::from_str(&json).unwrap();
    assert_eq!(actual.roll().to_bits(), expected.roll().to_bits());
    assert_eq!(actual.pitch().to_bits(), expected.pitch().to_bits());
    assert_eq!(actual.yaw().to_bits(), expected.yaw().to_bits());
    assert_eq!(actual.throttle().to_bits(), expected.throttle().to_bits());
}

#[test]
fn replay_rejects_different_mass_before_playback() {
    let (recording, config, initial_state) = fingerprint_recording();
    let different_mass =
        RigidBodyParams::new(3.1, Mat3::from_diagonal(&Vec3::new(0.2, 0.3, 0.4))).unwrap();
    let simulation = Simulation::new(config, different_mass, initial_state).unwrap();

    assert!(matches!(
        ReplayPlayer::new(&recording, &simulation),
        Err(replay::ReplayError::SimulationFingerprintMismatch)
    ));
}

#[test]
fn replay_rejects_different_inertia_before_playback() {
    let (recording, config, initial_state) = fingerprint_recording();
    let different_inertia =
        RigidBodyParams::new(3.0, Mat3::from_diagonal(&Vec3::new(0.2, 0.3, 0.5))).unwrap();
    let simulation = Simulation::new(config, different_inertia, initial_state).unwrap();

    assert!(matches!(
        ReplayPlayer::new(&recording, &simulation),
        Err(replay::ReplayError::SimulationFingerprintMismatch)
    ));
}

#[test]
fn replay_rejects_different_timestep_before_playback() {
    let (recording, _, initial_state) = fingerprint_recording();
    let different_timestep = SimulationConfig::from_physics_hz(250).unwrap();
    let simulation = Simulation::new(
        different_timestep,
        RigidBodyParams::new(3.0, Mat3::from_diagonal(&Vec3::new(0.2, 0.3, 0.4))).unwrap(),
        initial_state,
    )
    .unwrap();

    assert!(matches!(
        ReplayPlayer::new(&recording, &simulation),
        Err(replay::ReplayError::SimulationFingerprintMismatch)
    ));
}

fn fingerprint_recording() -> (replay::ReplayRecording, SimulationConfig, RigidBodyState) {
    let config = SimulationConfig::from_physics_hz(500).unwrap();
    let initial_state = RigidBodyState {
        position_world_m: Vec3::new(5.0, -3.0, -25.0),
        linear_velocity_world_mps: Vec3::new(10.0, 1.0, -2.0),
        orientation_world_from_body: Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.3)),
        angular_velocity_body_radps: Vec3::new(0.2, -0.1, 0.4),
    };
    let params = RigidBodyParams::new(3.0, Mat3::from_diagonal(&Vec3::new(0.2, 0.3, 0.4))).unwrap();
    let simulation = Simulation::new(config, params, initial_state).unwrap();
    let recording = ReplayRecorder::new(&simulation).unwrap().finish();
    (recording, config, initial_state)
}
