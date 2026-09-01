use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::{AircraftModel, AircraftModelLoader};
use replay::{AircraftReplayPlayer, AircraftReplayRecorder, AircraftReplayRecording};
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};

const SYNTHETIC_V3: &str =
    include_str!("../../../tests/fixtures/synthetic_non_reference_reynolds_v3.json");

fn model() -> AircraftModel {
    AircraftModelLoader::from_json_str(SYNTHETIC_V3).unwrap()
}

fn simulation() -> AircraftSimulation {
    AircraftSimulation::new(
        model(),
        AircraftSimulationConfig::new(
            0.01,
            Vec3::zeros(),
            AeroEnvironment::new(1.0, Vec3::new(0.5, 0.0, 0.0)).unwrap(),
        )
        .unwrap(),
        RigidBodyState {
            position_world_m: Vec3::zeros(),
            linear_velocity_world_mps: Vec3::new(40.0, 0.0, 0.0),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        },
    )
    .unwrap()
}

#[test]
fn m2_3c_19_synthetic_reynolds_aware_aircraft_replay_is_deterministic() {
    const STEPS: u64 = 250;
    let mut original = simulation();
    let mut recorder = AircraftReplayRecorder::with_capacity(&original, STEPS as usize).unwrap();
    for step in 0..STEPS {
        let phase = step as f64 / STEPS as f64;
        let input = PilotInput::new(0.2 * phase, -0.1 * phase, 0.05 * phase, 0.0);
        recorder.record(&mut original, step, input).unwrap();
    }
    let recording = recorder.finish();
    let json = recording.to_json_pretty().unwrap();
    let decoded = AircraftReplayRecording::from_json(&json).unwrap();
    assert_eq!(decoded, recording);

    let mut replayed = decoded.reconstruct_simulation(model()).unwrap();
    let player = AircraftReplayPlayer::new(&decoded, &replayed).unwrap();
    assert_eq!(player.verify_all(&mut replayed).unwrap(), STEPS);
    assert_eq!(original.state(), replayed.state());
}
