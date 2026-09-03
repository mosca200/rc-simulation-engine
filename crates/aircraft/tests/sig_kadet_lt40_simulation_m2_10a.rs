//! M2.10A.1 — SIG Kadet LT-40 EGV provisional demonstrator simulation tests.
//!
//! Validates that the provisional LT-40 demonstrator (classified as synthetic_test,
//! NOT a validated reference aircraft) can be simulated:
//! - AircraftSimulation initializes
//! - Simulation advances finite state for several seconds at 500 Hz
//! - No NaN/Inf in state
//! - Reynolds-family aero path is exercised
//! - Propulsion initializes
//!
//! Successful simulation does NOT constitute LT-40 flight-fidelity validation.

use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::load_aircraft_model;
use sim_core::{AeroEnvironment, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::path::Path;

fn model() -> model::AircraftModel {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("models/sig_kadet_lt40_egv/model.json");
    load_aircraft_model(&path).expect("LT-40 model must load")
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::from_physics_hz(
        500,
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(25.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[test]
fn lt40_simulation_initializes() {
    let model = model();
    let config = config();
    let state = initial_state();
    let sim = AircraftSimulation::new(model, config, state);
    assert!(sim.is_ok(), "LT-40 simulation must initialize");
}

#[test]
fn lt40_simulation_advances_finite_state() {
    let model = model();
    let config = config();
    let state = initial_state();
    let mut sim = AircraftSimulation::new(model, config, state).unwrap();

    let input = PilotInput::new(0.0, 0.0, 0.0, 0.5);
    let steps = 2500; // 5 seconds at 500 Hz

    for step in 0..steps {
        let snapshot = sim.step(&input);
        let rb = snapshot.rigid_body_state();

        assert!(
            rb.position_world_m.iter().all(|c| c.is_finite()),
            "step {step}: position not finite"
        );
        assert!(
            rb.linear_velocity_world_mps.iter().all(|c| c.is_finite()),
            "step {step}: velocity not finite"
        );
        assert!(
            rb.angular_velocity_body_radps.iter().all(|c| c.is_finite()),
            "step {step}: angular velocity not finite"
        );
        let q = rb.orientation_world_from_body.quaternion();
        assert!(
            [q.w, q.i, q.j, q.k].iter().all(|c| c.is_finite()),
            "step {step}: orientation not finite"
        );
    }

    assert_eq!(sim.step_index(), steps);
    assert!((sim.sim_time_s() - 5.0).abs() < 1e-10);
}

#[test]
fn lt40_reynolds_family_path_exercised() {
    let model = model();
    assert!(
        !model.aero_polar_families().is_empty(),
        "LT-40 must have Reynolds polar families"
    );

    let config = config();
    let state = initial_state();
    let mut sim = AircraftSimulation::new(model.clone(), config, state).unwrap();

    let input = PilotInput::new(0.0, 0.0, 0.0, 0.5);
    for _ in 0..100 {
        let _ = sim.step(&input);
    }

    let elements = sim.effective_aero_elements();
    assert_eq!(elements.len(), model.aero_elements().len());
    for element in elements {
        assert!(element.area_m2() > 0.0);
        assert!(element.chord_m() > 0.0);
    }
}

#[test]
fn lt40_propulsion_initializes() {
    let model = model();
    assert!(model.propulsion().is_some(), "LT-40 must have propulsion");

    let config = config();
    let state = initial_state();
    let sim = AircraftSimulation::new(model, config, state).unwrap();

    assert!(sim.model().propulsion().is_some());
}

#[test]
fn lt40_control_surfaces_bound() {
    let model = model();
    let bindings = model.control_surface_bindings();
    assert_eq!(bindings.len(), 4);

    let config = config();
    let state = initial_state();
    let mut sim = AircraftSimulation::new(model, config, state).unwrap();

    let input = PilotInput::new(0.5, -0.3, 0.2, 0.5);
    let snapshot = sim.step(&input);
    let positions = snapshot.control_surface_positions();

    assert!(positions.aileron_angle_rad().is_finite());
    assert!(positions.elevator_angle_rad().is_finite());
    assert!(positions.rudder_angle_rad().is_finite());
    assert!(positions.throttle().is_finite());
}

#[test]
fn lt40_no_nan_inf_at_trainer_speed() {
    let model = model();
    let config = config();
    let state = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(20.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let mut sim = AircraftSimulation::new(model, config, state).unwrap();

    let input = PilotInput::neutral();
    for _ in 0..500 {
        let snapshot = sim.step(&input);
        let rb = snapshot.rigid_body_state();
        for val in rb.position_world_m.iter() {
            assert!(val.is_finite());
            assert!(!val.is_nan());
        }
        for val in rb.linear_velocity_world_mps.iter() {
            assert!(val.is_finite());
            assert!(!val.is_nan());
        }
    }
}
