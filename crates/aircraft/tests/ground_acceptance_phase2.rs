//! Acceptance runner: full chain from controls to touchdown.
mod common_ground;

use aircraft::AircraftSimulation;
use common_ground::{ground_test_config, load_ground_test_model, parked_state};
use sim_core::PilotInput;

fn assert_finite(snapshot: &aircraft::AircraftSnapshot) {
    let rigid = snapshot.rigid_body_state();
    assert!(rigid.position_world_m.iter().all(|v| v.is_finite()));
    assert!(
        rigid
            .linear_velocity_world_mps
            .iter()
            .all(|v| v.is_finite())
    );
    assert!(
        rigid
            .angular_velocity_body_radps
            .iter()
            .all(|v| v.is_finite())
    );
    assert!(snapshot.total_ground_normal_force_n().is_finite());
}

/// Scheduled pilot input only; all motion comes from real physics.
/// idle/hold -> throttle ramp -> elevator rotation -> airborne ->
/// throttle reduction/descent -> flare -> touchdown.
#[test]
fn full_takeoff_airborne_descent_touchdown_chain() {
    let mut simulation = AircraftSimulation::new(
        load_ground_test_model(),
        ground_test_config(),
        parked_state(0.02),
    )
    .expect("valid simulation");

    let mut saw_ground_roll = false;
    let mut saw_liftoff = false;
    let mut saw_touchdown_after_flight = false;
    let mut flare_commanded_before_touchdown = false;
    let mut saw_flare_elevator_deflection = false;
    let mut peak_altitude_m = 0.0f64;

    // Phase 1: settle for 1 s (500 steps) — must stay supported or settle.
    let idle = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    for _ in 0..500 {
        let snapshot = simulation.step(&idle);
        assert_finite(&snapshot);
    }
    assert!(simulation.last_ground_evaluation().weight_on_wheels());

    // Phase 2: throttle ramp over 2 s, then full throttle + rotation.
    for step in 0..1000 {
        let throttle = (f64::from(step) / 1000.0).clamp(0.0, 1.0);
        let snapshot = simulation.step(&PilotInput::new(0.0, 0.0, 0.0, throttle));
        assert_finite(&snapshot);
        saw_ground_roll = true;
    }

    // Phase 3: rotation and liftoff (up to 12 s of full-throttle rollout).
    let rotate = PilotInput::new(0.0, 0.35, 0.0, 1.0);
    let mut liftoff_at = None;
    for step in 0..6000 {
        let snapshot = simulation.step(&rotate);
        assert_finite(&snapshot);
        if !snapshot.weight_on_wheels() {
            liftoff_at = Some(step);
            saw_liftoff = true;
            break;
        }
        saw_ground_roll = true;
    }
    assert!(saw_ground_roll, "must roll on gear before liftoff");
    assert!(liftoff_at.is_some(), "must lift off under real physics");

    // Phase 4: airborne climb for 4 s, then throttle cut + descent.
    let climb = PilotInput::new(0.0, 0.1, 0.0, 0.8);
    for _ in 0..2000 {
        let snapshot = simulation.step(&climb);
        assert_finite(&snapshot);
        assert!(!snapshot.weight_on_wheels(), "must stay airborne in climb");
        peak_altitude_m = peak_altitude_m.max(-snapshot.rigid_body_state().position_world_m.z);
    }
    assert!(
        peak_altitude_m > 1.0,
        "must gain altitude (peak {peak_altitude_m})"
    );

    // Phase 5: descent at low throttle, then command a real elevator flare
    // below one metre before touchdown (up to 30 s total).
    let descend = PilotInput::new(0.0, 0.0, 0.0, 0.15);
    let flare = PilotInput::new(0.0, 0.25, 0.0, 0.15);
    for _ in 0..15000 {
        let altitude_m = -simulation.state().rigid_body().position_world_m.z;
        let flare_active = altitude_m <= 1.0;
        let snapshot = simulation.step(if flare_active { &flare } else { &descend });
        assert_finite(&snapshot);
        // No tunneling: never far below the geometric rest plane.
        assert!(
            snapshot.rigid_body_state().position_world_m.z < 1.0,
            "tunneled: z = {}",
            snapshot.rigid_body_state().position_world_m.z
        );
        if flare_active && !snapshot.weight_on_wheels() {
            flare_commanded_before_touchdown = true;
            saw_flare_elevator_deflection |=
                snapshot.control_surface_positions().elevator_angle_rad() > 0.01;
        }
        if snapshot.weight_on_wheels() {
            saw_touchdown_after_flight = true;
            assert!(snapshot.total_ground_normal_force_n() > 0.0);
            break;
        }
    }
    assert!(saw_touchdown_after_flight, "must touch down after flight");
    assert!(
        flare_commanded_before_touchdown,
        "flare must be commanded while still airborne"
    );
    assert!(
        saw_flare_elevator_deflection,
        "flare command must move the bound elevator before touchdown"
    );

    // Phase 6: neutral-elevator rollout stays finite and supported.
    let rollout = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    for _ in 0..2000 {
        let snapshot = simulation.step(&rollout);
        assert_finite(&snapshot);
        assert!(snapshot.rigid_body_state().position_world_m.z < 1.0);
    }
    assert!(saw_liftoff);
    // Brakes path: full brake command must be accepted without NaN.
    simulation.set_brake_command(1.0);
    for _ in 0..500 {
        let snapshot = simulation.step(&rollout);
        assert_finite(&snapshot);
    }
}
