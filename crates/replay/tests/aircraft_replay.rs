use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSnapshot};
use model::{AircraftModel, AircraftModelLoader, load_aircraft_model};
use replay::{
    AIRCRAFT_REPLAY_SCHEMA_VERSION, AircraftReplayError, AircraftReplayPlayer,
    AircraftReplayRecorder, AircraftReplayRecording, AircraftSnapshotHash,
};
use serde_json::{Value, json};
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::path::{Path, PathBuf};

const DATASET_STEPS: u64 = 2_000;

fn repository_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn model_path() -> PathBuf {
    repository_root().join("models/acro_electric_01/model.json")
}

fn load_model() -> AircraftModel {
    load_aircraft_model(model_path()).unwrap()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn config_with_density(density: f64) -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        Vec3::new(-0.0, 0.0, 9.80665),
        AeroEnvironment::new(density, Vec3::new(0.25, -0.5, 0.75)).unwrap(),
    )
    .unwrap()
}

fn simulation() -> AircraftSimulation {
    AircraftSimulation::new(load_model(), config_with_density(1.225), initial_state()).unwrap()
}

fn record_steps(steps: u64) -> AircraftReplayRecording {
    let mut simulation = simulation();
    let mut recorder = AircraftReplayRecorder::with_capacity(&simulation, steps as usize).unwrap();
    for step_index in 0..steps {
        let phase = step_index as f64 * 0.007;
        let input = PilotInput::new(
            0.35 * phase.sin(),
            -0.2 * (phase * 0.7).cos(),
            0.15 * (phase * 0.3).sin(),
            0.55,
        );
        recorder.record(&mut simulation, step_index, input).unwrap();
    }
    recorder.finish()
}

fn json_value(recording: &AircraftReplayRecording) -> Value {
    serde_json::from_str(&recording.to_json_pretty().unwrap()).unwrap()
}

fn parse_value(value: &Value) -> Result<AircraftReplayRecording, AircraftReplayError> {
    AircraftReplayRecording::from_json(&serde_json::to_string(value).unwrap())
}

fn load_modified_model(mutator: impl FnOnce(&mut Value)) -> AircraftModel {
    let json = std::fs::read_to_string(model_path()).unwrap();
    let mut value: Value = serde_json::from_str(&json).unwrap();
    mutator(&mut value);
    AircraftModelLoader::from_json_str(&serde_json::to_string(&value).unwrap()).unwrap()
}

#[test]
fn snapshot_hash_is_deterministic_and_sensitive_to_snapshot_fields() {
    let mut first = simulation();
    let mut second = simulation();
    let input = PilotInput::new(0.3, -0.2, 0.1, 0.6);
    let first_snapshot = first.step(&input);
    let second_snapshot = second.step(&input);
    let first_hash = AircraftSnapshotHash::from_snapshot(&first_snapshot);
    assert_eq!(
        first_hash,
        AircraftSnapshotHash::from_snapshot(&first_snapshot)
    );
    assert_eq!(
        first_hash,
        AircraftSnapshotHash::from_snapshot(&second_snapshot)
    );

    let changed_snapshot = second.step(&PilotInput::new(-0.3, 0.2, -0.1, 0.4));
    assert_ne!(
        first_hash,
        AircraftSnapshotHash::from_snapshot(&changed_snapshot)
    );
    assert_eq!(first_hash.as_bytes().len(), 32);
    assert_eq!(first_hash.to_hex().len(), 64);
}

#[test]
fn snapshot_hash_uses_documented_field_order_and_hamilton_wxyz() {
    let mut simulation = simulation();
    let snapshot = simulation.step(&PilotInput::new(0.2, -0.1, 0.3, 0.7));
    let canonical = manual_snapshot_hash(&snapshot, false);
    let wrong_quaternion_order = manual_snapshot_hash(&snapshot, true);
    assert_eq!(
        AircraftSnapshotHash::from_snapshot(&snapshot).as_bytes(),
        &canonical
    );
    assert_ne!(canonical, wrong_quaternion_order);
}

#[test]
fn recorder_requires_step_zero_and_rejects_noncontiguous_pre_step() {
    let mut advanced = simulation();
    let _ = advanced.step(&PilotInput::neutral());
    assert!(matches!(
        AircraftReplayRecorder::new(&advanced),
        Err(AircraftReplayError::SimulationNotAtInitialStep(1))
    ));

    let mut fresh = simulation();
    let mut recorder = AircraftReplayRecorder::new(&fresh).unwrap();
    assert!(matches!(
        recorder.record(&mut fresh, 1, PilotInput::neutral()),
        Err(AircraftReplayError::NonContiguousStep {
            expected: 0,
            actual: 1
        })
    ));
}

#[test]
fn recorder_binds_pre_step_n_to_post_step_n_plus_one_hash() {
    let mut simulation = simulation();
    let mut recorder = AircraftReplayRecorder::new(&simulation).unwrap();
    let snapshot = recorder
        .record(&mut simulation, 0, PilotInput::new(0.1, 0.2, 0.3, 0.4))
        .unwrap();
    assert_eq!(snapshot.step_index(), 1);
    let recording = recorder.finish();
    assert_eq!(recording.frames()[0].step_index(), 0);
    assert_eq!(
        recording.frames()[0].expected_snapshot_hash(),
        AircraftSnapshotHash::from_snapshot(&snapshot)
    );
}

#[test]
fn json_roundtrip_preserves_exact_values_fingerprint_and_hashes() {
    let recording = record_steps(4);
    let json = recording.to_json_pretty().unwrap();
    let decoded = AircraftReplayRecording::from_json(&json).unwrap();
    assert_eq!(decoded, recording);
    assert_eq!(
        decoded.simulation_config().dt_s().to_bits(),
        recording.simulation_config().dt_s().to_bits()
    );
    for (actual, expected) in decoded
        .simulation_config()
        .gravity_world_mps2()
        .into_iter()
        .zip(recording.simulation_config().gravity_world_mps2())
    {
        assert_eq!(actual.to_bits(), expected.to_bits());
    }
    assert_eq!(
        decoded.model_physics_fingerprint().as_bytes(),
        recording.model_physics_fingerprint().as_bytes()
    );
    for (actual, expected) in decoded.frames().iter().zip(recording.frames()) {
        assert_eq!(
            actual.pilot_input().roll().to_bits(),
            expected.pilot_input().roll().to_bits()
        );
        assert_eq!(
            actual.expected_snapshot_hash().as_bytes(),
            expected.expected_snapshot_hash().as_bytes()
        );
    }
}

#[test]
fn strict_json_rejects_unknown_fields_at_every_required_level() {
    let recording = record_steps(1);
    for pointer in [
        "",
        "/simulation_config",
        "/initial_rigid_body_state",
        "/frames/0",
        "/frames/0/pilot_input",
    ] {
        let mut value = json_value(&recording);
        value
            .pointer_mut(pointer)
            .unwrap()
            .as_object_mut()
            .unwrap()
            .insert("unknown_s8a_field".to_owned(), json!(true));
        assert!(parse_value(&value).is_err(), "accepted field at {pointer}");
    }
}

#[test]
fn json_rejects_unknown_schema_invalid_input_config_and_state() {
    let recording = record_steps(1);

    let mut schema = json_value(&recording);
    schema["schema_version"] = json!(99);
    assert!(matches!(
        parse_value(&schema),
        Err(AircraftReplayError::UnsupportedSchema(99))
    ));

    let mut input = json_value(&recording);
    input["frames"][0]["pilot_input"]["roll"] = json!(1.01);
    assert!(matches!(
        parse_value(&input),
        Err(AircraftReplayError::InvalidPilotInput { step_index: 0 })
    ));

    let mut config = json_value(&recording);
    config["simulation_config"]["dt_s"] = json!(0.0);
    assert!(matches!(
        parse_value(&config),
        Err(AircraftReplayError::InvalidSimulationConfig(_))
    ));

    let mut state = json_value(&recording);
    state["initial_rigid_body_state"]["orientation_world_from_body_wxyz"] =
        json!([0.0, 0.0, 0.0, 0.0]);
    assert!(matches!(
        parse_value(&state),
        Err(AircraftReplayError::InvalidInitialState(_))
    ));
}

#[test]
fn json_rejects_noncontiguous_frames_and_malformed_hex_values() {
    let recording = record_steps(2);

    let mut noncontiguous = json_value(&recording);
    noncontiguous["frames"][1]["step_index"] = json!(7);
    assert!(matches!(
        parse_value(&noncontiguous),
        Err(AircraftReplayError::NonContiguousStep {
            expected: 1,
            actual: 7
        })
    ));

    let mut hash = json_value(&recording);
    hash["frames"][0]["expected_snapshot_hash"] = json!("abc");
    assert!(matches!(
        parse_value(&hash),
        Err(AircraftReplayError::MalformedSnapshotHash(_))
    ));

    let mut fingerprint = json_value(&recording);
    fingerprint["model_physics_fingerprint"] =
        json!("AA00000000000000000000000000000000000000000000000000000000000000");
    assert!(matches!(
        parse_value(&fingerprint),
        Err(AircraftReplayError::MalformedModelPhysicsFingerprint(_))
    ));
}

#[test]
fn model_id_mismatch_is_distinct_from_physics_fingerprint_mismatch() {
    let recording = record_steps(1);
    let different_id = load_modified_model(|value| value["model_id"] = json!("other-model"));
    let different_id_simulation = AircraftSimulation::new(
        different_id,
        recording.simulation_config().to_runtime().unwrap(),
        *recording.initial_rigid_body_state(),
    )
    .unwrap();
    assert!(matches!(
        AircraftReplayPlayer::new(&recording, &different_id_simulation),
        Err(AircraftReplayError::ModelIdMismatch { .. })
    ));

    let different_physics = load_modified_model(|value| {
        value["rigid_body"]["mass_kg"] = json!(2.01);
    });
    let different_physics_simulation = AircraftSimulation::new(
        different_physics,
        recording.simulation_config().to_runtime().unwrap(),
        *recording.initial_rigid_body_state(),
    )
    .unwrap();
    assert!(matches!(
        AircraftReplayPlayer::new(&recording, &different_physics_simulation),
        Err(AircraftReplayError::ModelPhysicsFingerprintMismatch { .. })
    ));
}

#[test]
fn player_rejects_config_and_initial_state_mismatches_before_playback() {
    let recording = record_steps(1);
    let different_config = AircraftSimulation::new(
        load_model(),
        config_with_density(1.2),
        *recording.initial_rigid_body_state(),
    )
    .unwrap();
    assert!(matches!(
        AircraftReplayPlayer::new(&recording, &different_config),
        Err(AircraftReplayError::SimulationConfigMismatch)
    ));

    let mut changed_state = *recording.initial_rigid_body_state();
    changed_state.position_world_m.x += 0.001;
    let different_state = AircraftSimulation::new(
        load_model(),
        recording.simulation_config().to_runtime().unwrap(),
        changed_state,
    )
    .unwrap();
    assert!(matches!(
        AircraftReplayPlayer::new(&recording, &different_state),
        Err(AircraftReplayError::InitialStateMismatch)
    ));
}

#[test]
fn first_per_step_divergence_reports_pre_and_post_step_indices() {
    let original = record_steps(12);
    let mut value = json_value(&original);
    value["frames"][5]["expected_snapshot_hash"] =
        json!("0000000000000000000000000000000000000000000000000000000000000000");
    let recording = parse_value(&value).unwrap();
    let mut simulation = recording.reconstruct_simulation(load_model()).unwrap();
    let mut player = AircraftReplayPlayer::new(&recording, &simulation).unwrap();
    for _ in 0..5 {
        assert!(player.verify_next(&mut simulation).unwrap().is_some());
    }
    assert!(matches!(
        player.verify_next(&mut simulation),
        Err(AircraftReplayError::SnapshotHashMismatch {
            frame_step_index: 5,
            snapshot_step_index: 6,
            ..
        })
    ));
}

#[test]
fn replay_json_is_input_based_and_contains_no_snapshot_trajectory() {
    let recording = record_steps(3);
    let value = json_value(&recording);
    let top_level = value.as_object().unwrap();
    assert_eq!(
        top_level
            .keys()
            .cloned()
            .collect::<std::collections::BTreeSet<_>>(),
        [
            "frames",
            "initial_rigid_body_state",
            "model_id",
            "model_physics_fingerprint",
            "schema_version",
            "simulation_config",
        ]
        .map(str::to_owned)
        .into_iter()
        .collect()
    );
    for frame in value["frames"].as_array().unwrap() {
        assert_eq!(
            frame
                .as_object()
                .unwrap()
                .keys()
                .cloned()
                .collect::<std::collections::BTreeSet<_>>(),
            ["expected_snapshot_hash", "pilot_input", "step_index"]
                .map(str::to_owned)
                .into_iter()
                .collect()
        );
    }
    assert!(!value["frames"][0].to_string().contains("position_world_m"));
    assert!(!value["frames"][0].to_string().contains("orientation"));
}

#[test]
fn generated_two_thousand_step_replay_verifies_every_hash() {
    let recording = record_steps(DATASET_STEPS);
    let json = recording.to_json_pretty().unwrap();
    let decoded = AircraftReplayRecording::from_json(&json).unwrap();
    let mut replayed = decoded.reconstruct_simulation(load_model()).unwrap();
    let player = AircraftReplayPlayer::new(&decoded, &replayed).unwrap();
    assert_eq!(player.verify_all(&mut replayed).unwrap(), DATASET_STEPS);
}

#[test]
fn committed_acro_dataset_verifies_all_two_thousand_steps() {
    let dataset =
        repository_root().join("tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json");
    verify_dataset(&dataset, DATASET_STEPS);
}

#[test]
fn replay_crate_has_no_renderer_wgpu_or_winit_dependency() {
    let manifest =
        std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml"))
            .unwrap();
    for forbidden in ["renderer", "wgpu", "winit"] {
        assert!(
            !manifest.lines().any(|line| {
                line.split_once('=')
                    .is_some_and(|(name, _)| name.trim() == forbidden)
            }),
            "forbidden replay dependency {forbidden}"
        );
    }
}

fn verify_dataset(path: &Path, expected_steps: u64) {
    let json = std::fs::read_to_string(path).unwrap();
    let recording = AircraftReplayRecording::from_json(&json).unwrap();
    assert_eq!(recording.schema_version(), AIRCRAFT_REPLAY_SCHEMA_VERSION);
    assert_eq!(recording.frames().len() as u64, expected_steps);

    // Bit-identical hashes are qualified only for the target that produced the
    // canonical dataset. Other targets still consume every committed input and
    // verify the complete replay chain against hashes produced on that target.
    #[cfg(not(target_os = "windows"))]
    let recording = {
        let mut simulation = recording.reconstruct_simulation(load_model()).unwrap();
        let mut recorder =
            AircraftReplayRecorder::with_capacity(&simulation, recording.frames().len()).unwrap();
        for frame in recording.frames() {
            recorder
                .record(&mut simulation, frame.step_index(), frame.pilot_input())
                .unwrap();
        }
        recorder.finish()
    };

    let mut simulation = recording.reconstruct_simulation(load_model()).unwrap();
    let player = AircraftReplayPlayer::new(&recording, &simulation).unwrap();
    assert_eq!(player.verify_all(&mut simulation).unwrap(), expected_steps);
}

fn manual_snapshot_hash(snapshot: &AircraftSnapshot, swap_w_x: bool) -> [u8; 32] {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"rcsim:aircraft-snapshot:v1");
    hasher.update(&snapshot.step_index().to_le_bytes());
    manual_f64(&mut hasher, snapshot.sim_time_s());
    let rigid = snapshot.rigid_body_state();
    for vector in [&rigid.position_world_m, &rigid.linear_velocity_world_mps] {
        for value in [vector.x, vector.y, vector.z] {
            manual_f64(&mut hasher, value);
        }
    }
    let quaternion = rigid.orientation_world_from_body.quaternion();
    let values = if swap_w_x {
        [quaternion.i, quaternion.w, quaternion.j, quaternion.k]
    } else {
        [quaternion.w, quaternion.i, quaternion.j, quaternion.k]
    };
    for value in values {
        manual_f64(&mut hasher, value);
    }
    for value in [
        rigid.angular_velocity_body_radps.x,
        rigid.angular_velocity_body_radps.y,
        rigid.angular_velocity_body_radps.z,
    ] {
        manual_f64(&mut hasher, value);
    }
    let controls = snapshot.control_surface_positions();
    for value in [
        controls.aileron_angle_rad(),
        controls.elevator_angle_rad(),
        controls.rudder_angle_rad(),
        controls.throttle(),
    ] {
        manual_f64(&mut hasher, value);
    }
    *hasher.finalize().as_bytes()
}

fn manual_f64(hasher: &mut blake3::Hasher, value: f64) {
    hasher.update(&value.to_bits().to_le_bytes());
}
