//! Phase 2 takeoff, landing, determinism, and allocation tests.
mod common_ground;

use aircraft::AircraftSimulation;
use common_ground::{ground_test_config, load_ground_test_model, parked_state};
use sim_core::PilotInput;
use sim_math::Vec3;

#[test]
fn takeoff_emerges_from_physics_without_trigger() {
    let mut simulation = AircraftSimulation::new(
        load_ground_test_model(),
        ground_test_config(),
        parked_state(0.02),
    )
    .expect("valid simulation");
    // Full throttle + slight nose-up pitch: real thrust accelerates the brick,
    // the flat-plate wing builds lift with speed, and normal loads decay.
    let rollout = PilotInput::new(0.0, 0.35, 0.0, 1.0);
    let mut saw_weight_on_wheels = false;
    let mut normal_decayed = false;
    let mut initial_normal = 0.0;
    let mut liftoff_step = None;
    for step in 0..6000 {
        let snapshot = simulation.step(&rollout);
        assert!(
            snapshot
                .rigid_body_state()
                .position_world_m
                .iter()
                .all(|v| v.is_finite())
        );
        if snapshot.weight_on_wheels() {
            saw_weight_on_wheels = true;
            if initial_normal == 0.0 {
                initial_normal = snapshot.total_ground_normal_force_n();
            }
            if initial_normal > 1.0 && snapshot.total_ground_normal_force_n() < 0.5 * initial_normal
            {
                normal_decayed = true;
            }
        } else if saw_weight_on_wheels && step > 100 {
            liftoff_step = Some(step);
            break;
        }
    }
    assert!(saw_weight_on_wheels, "must start with weight on wheels");
    assert!(normal_decayed, "normal load must decay as lift builds");
    let liftoff = liftoff_step.expect("all contacts must eventually release");
    assert!(
        liftoff > 100,
        "takeoff must come from acceleration, not teleport"
    );
    // After liftoff the aircraft is airborne and climbing away.
    for _ in 0..500 {
        let snapshot = simulation.step(&rollout);
        assert!(!snapshot.weight_on_wheels());
    }
    assert!(simulation.state().rigid_body().position_world_m.z < -0.5);
}

#[test]
fn landing_touchdown_is_finite_and_damped() {
    let mut start = parked_state(3.0);
    start.linear_velocity_world_mps = Vec3::new(14.0, 0.0, 1.2);
    let mut simulation =
        AircraftSimulation::new(load_ground_test_model(), ground_test_config(), start)
            .expect("valid simulation");
    let input = PilotInput::new(0.0, 0.05, 0.0, 0.25);
    let mut touched = false;
    let mut first_touch_z = 0.0;
    let mut deepest_z = f64::NEG_INFINITY;
    for _ in 0..6000 {
        let snapshot = simulation.step(&input);
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
        if snapshot.weight_on_wheels() {
            if !touched {
                touched = true;
                first_touch_z = rigid.position_world_m.z;
            }
            assert!(snapshot.total_ground_normal_force_n() > 0.0);
            // CG must not tunnel: geometric rest is z = -0.41.
            deepest_z = deepest_z.max(rigid.position_world_m.z);
            assert!(
                rigid.position_world_m.z < 0.5,
                "tunneled catastrophically: z = {}",
                rigid.position_world_m.z
            );
        }
        if touched && !snapshot.weight_on_wheels() {
            break;
        }
    }
    assert!(touched, "shallow descent must produce first contact");
    // First contact occurs at physically consistent geometry: the CG must be
    // within 25 cm of the geometric rest height when the wheels first touch.
    assert!(
        (first_touch_z + 0.41).abs() < 0.25,
        "first contact at inconsistent geometry: z = {first_touch_z}"
    );
    assert!(
        deepest_z.is_finite() && deepest_z < -0.41 + 0.20,
        "contact must damp the impact without deep tunneling (deepest z = {deepest_z})"
    );
}

#[test]
fn hard_landing_bounces_but_stays_finite() {
    let mut start = parked_state(6.0);
    start.linear_velocity_world_mps = Vec3::new(12.0, 0.0, 4.5);
    let mut simulation =
        AircraftSimulation::new(load_ground_test_model(), ground_test_config(), start)
            .expect("valid simulation");
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let mut touched = false;
    for _ in 0..4000 {
        let snapshot = simulation.step(&input);
        let rigid = snapshot.rigid_body_state();
        for v in [
            rigid.position_world_m.x,
            rigid.position_world_m.y,
            rigid.position_world_m.z,
        ] {
            assert!(v.is_finite() && v.abs() < 1.0e6);
        }
        assert!(
            rigid
                .linear_velocity_world_mps
                .iter()
                .all(|v| v.is_finite())
        );
        // May bounce; only physical invariants are asserted (finite, no tunneling).
        assert!(rigid.position_world_m.z < 1.0, "tunneled through the plane");
        if snapshot.weight_on_wheels() {
            touched = true;
        }
    }
    assert!(touched, "hard descent must contact the plane");
}

#[test]
fn identical_ground_scenarios_reproduce_bitwise() {
    let run = || {
        let mut simulation = AircraftSimulation::new(
            load_ground_test_model(),
            ground_test_config(),
            parked_state(0.05),
        )
        .expect("valid simulation");
        let input = PilotInput::new(0.05, -0.05, 0.1, 0.7);
        let mut last = simulation.step(&input);
        for _ in 0..999 {
            last = simulation.step(&input);
        }
        last
    };
    let first = run();
    let second = run();
    assert_eq!(first, second);
    assert_eq!(
        first.ground_telemetry(),
        second.ground_telemetry(),
        "ground telemetry must reproduce"
    );
}

#[test]
fn ground_step_holds_zero_allocation_guarantee() {
    let mut simulation = AircraftSimulation::new(
        load_ground_test_model(),
        ground_test_config(),
        parked_state(0.02),
    )
    .expect("valid simulation");
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.6);
    for _ in 0..50 {
        let _ = simulation.step(&input);
    }
    let allocation_info = allocation_counter::measure(|| {
        for _ in 0..200 {
            let _ = simulation.step(&input);
        }
    });
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}
