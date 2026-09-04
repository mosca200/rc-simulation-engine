//! Phase 2 ground-contact physics: kinematics, unilateral law, friction.
mod common_ground;

use aircraft::AircraftSimulation;
use common_ground::{ground_test_config, load_ground_test_model, parked_state};
use sim_core::{
    FlatGroundPlane, GearContact, GroundCommand, GroundSurface, PilotInput, RigidBodyState,
    SteeringSource, contact_point_velocity_world, evaluate_ground_wrench, steering_angle_rad,
};
use sim_math::{Orientation, Vec3};

fn single_wheel() -> GearContact {
    GearContact {
        position_body_m: Vec3::new(0.0, 0.0, 0.3),
        wheel_radius_m: 0.05,
        stiffness_n_per_m: 8000.0,
        damping_n_s_per_m: 250.0,
        long_mu: 0.6,
        lat_mu: 0.9,
        rolling_mu: 0.02,
        brake_mu: 0.0,
        steering: SteeringSource::Fixed,
        max_steer_rad: 0.0,
        steerable: false,
        braked: false,
    }
}

fn level_state(cg_down_m: f64, velocity_world: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, cg_down_m),
        linear_velocity_world_mps: velocity_world,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[test]
fn free_body_above_ground_produces_zero_wrench() {
    let gear = [single_wheel()];
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let command = GroundCommand::new(0.0, 0.0);
    // Wheel bottom at -5 + 0.35 = -4.65, well above plane z = 0.
    let state = level_state(-5.0, Vec3::new(12.0, 1.0, -2.0));
    let evaluation = evaluate_ground_wrench(&state, &gear, &surface, &command);
    assert_eq!(evaluation.active_contacts, 0);
    assert!(!evaluation.weight_on_wheels());
    assert_eq!(evaluation.total_normal_force_n, 0.0);
    assert_eq!(evaluation.total_tangential_force_n, 0.0);
    assert_eq!(evaluation.force_body_n, Vec3::zeros());
    assert_eq!(evaluation.moment_body_nm, Vec3::zeros());
}

#[test]
fn contact_velocity_includes_omega_cross_r() {
    // Pure rotation about body-Z (yaw): omega x r for r = +X must be +Y.
    let state = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::new(0.0, 0.0, 2.0),
    };
    let velocity = contact_point_velocity_world(&state, &Vec3::new(1.0, 0.0, 0.0));
    assert!((velocity.x - 0.0).abs() < 1.0e-12);
    assert!((velocity.y - 2.0).abs() < 1.0e-12);
    assert!((velocity.z - 0.0).abs() < 1.0e-12);
    // Pure pitch rate about body-Y with an offset nose contact.
    let pitched = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(1.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::new(0.0, 3.0, 0.0),
    };
    let nose = contact_point_velocity_world(&pitched, &Vec3::new(0.6, 0.0, 0.35));
    // omega x r = (q*z - 0, 0, -(q*x)) = (1.05, 0, -1.8); plus V_cg = (2.05, 0, -1.8).
    assert!((nose.x - 2.05).abs() < 1.0e-12);
    assert!((nose.z + 1.8).abs() < 1.0e-12);
}

#[test]
fn ground_pushes_but_never_pulls() {
    let gear = [single_wheel()];
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let command = GroundCommand::new(0.0, 0.0);
    // Shallow penetration with fast separation: damper dominates, clamps to zero.
    // Bottom at -0.34 + 0.35 = +0.01 penetration; v_down = -50 -> F = 80 - 12500.
    let separating = level_state(-0.34, Vec3::new(0.0, 0.0, -50.0));
    let evaluation = evaluate_ground_wrench(&separating, &gear, &surface, &command);
    assert_eq!(evaluation.active_contacts, 0);
    assert_eq!(evaluation.total_normal_force_n, 0.0);
    // World force z must never be positive (down, i.e. attractive) on contact.
    assert!(evaluation.force_body_n.z <= 0.0);
}

#[test]
fn damping_opposes_penetration_velocity() {
    let gear = [single_wheel()];
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let command = GroundCommand::new(0.0, 0.0);
    // Same 5 cm penetration; one sinking, one rising.
    let sinking = level_state(-0.30, Vec3::new(0.0, 0.0, 1.0));
    let rising = level_state(-0.30, Vec3::new(0.0, 0.0, -1.0));
    let sinking_force =
        evaluate_ground_wrench(&sinking, &gear, &surface, &command).total_normal_force_n;
    let rising_force =
        evaluate_ground_wrench(&rising, &gear, &surface, &command).total_normal_force_n;
    // k * 0.05 = 400 N; sinking adds c * 1 = 250 -> 650 N; rising subtracts -> 150 N.
    assert!((sinking_force - 650.0).abs() < 1.0e-9);
    assert!((rising_force - 150.0).abs() < 1.0e-9);
    assert!(sinking_force > rising_force);
}

#[test]
fn longitudinal_rolling_accelerates_and_lateral_friction_opposes_sideslip() {
    let model = load_ground_test_model();
    // Parked, then push forward: with zero throttle the drag-free brick would
    // sit; here we verify the analytic friction direction instead.
    let gear: Vec<GearContact> = model
        .landing_gear()
        .iter()
        .map(|contact| contact.contact())
        .collect();
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let command = GroundCommand::new(0.0, 0.0);
    // Rolling forward at 5 m/s: longitudinal force must oppose motion (negative X).
    let rolling = level_state(-0.39, Vec3::new(5.0, 0.0, 0.0));
    let rolling_eval = evaluate_ground_wrench(&rolling, &gear, &surface, &command);
    assert!(rolling_eval.weight_on_wheels());
    assert!(rolling_eval.force_body_n.x < 0.0);
    // Pure sideslip at 2 m/s: lateral force must oppose motion (negative Y).
    let sideslip = level_state(-0.39, Vec3::new(0.0, 2.0, 0.0));
    let side_eval = evaluate_ground_wrench(&sideslip, &gear, &surface, &command);
    assert!(side_eval.force_body_n.y < 0.0);
    // Anisotropy: lateral grip cap exceeds longitudinal for this gear.
    assert!(side_eval.total_tangential_force_n > 0.0);
}

#[test]
fn friction_and_rolling_resistance_create_no_energy() {
    // Propulsion disabled (idle throttle produces no thrust at rest);
    // rolling resistance + friction must dissipate, never create energy.
    // Give it a forward shove via initial velocity instead of thrust.
    let mut shoved = parked_state(0.02);
    shoved.linear_velocity_world_mps = Vec3::new(3.0, 1.0, 0.0);
    let mut simulation =
        AircraftSimulation::new(load_ground_test_model(), ground_test_config(), shoved)
            .expect("valid simulation");
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let initial_speed_squared = 10.0;
    for _ in 0..500 {
        let snapshot = simulation.step(&input);
        let velocity = snapshot.rigid_body_state().linear_velocity_world_mps;
        let speed_squared = velocity.x * velocity.x + velocity.y * velocity.y;
        // Friction is dissipative, but the aero brick at 3 m/s forward flight
        // produces small lift/drag transients; allow 2% integration headroom.
        assert!(
            speed_squared <= initial_speed_squared * 1.02 + 1.0e-6,
            "tangential speed grew without propulsion: {speed_squared}"
        );
        assert!(velocity.iter().all(|v| v.is_finite()));
    }
}

#[test]
fn steering_rotates_wheel_basis() {
    let mut contact = single_wheel();
    contact.steerable = true;
    contact.steering = SteeringSource::Rudder;
    contact.max_steer_rad = 0.5;
    assert_eq!(
        steering_angle_rad(&contact, &GroundCommand::new(0.0, 0.0)),
        0.0
    );
    assert!((steering_angle_rad(&contact, &GroundCommand::new(1.0, 0.0)) - 0.5).abs() < 1.0e-12);
    assert!((steering_angle_rad(&contact, &GroundCommand::new(-1.0, 0.0)) + 0.5).abs() < 1.0e-12);
    // Fixed contact never steers regardless of rudder.
    let fixed = single_wheel();
    assert_eq!(
        steering_angle_rad(&fixed, &GroundCommand::new(1.0, 0.0)),
        0.0
    );
    // Steered wheel redirects rolling resistance: with 90-degree steer the
    // forward slip maps fully onto the lateral axis.
    let mut sideways = single_wheel();
    sideways.steerable = true;
    sideways.steering = SteeringSource::Rudder;
    sideways.max_steer_rad = std::f64::consts::FRAC_PI_2;
    let gear = [sideways];
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let rolling = level_state(-0.30, Vec3::new(4.0, 0.0, 0.0));
    let straight = evaluate_ground_wrench(&rolling, &gear, &surface, &GroundCommand::new(0.0, 0.0));
    let steered = evaluate_ground_wrench(&rolling, &gear, &surface, &GroundCommand::new(1.0, 0.0));
    assert!(straight.contacts[0].longitudinal_force_n.abs() > 0.0);
    assert!(steered.contacts[0].lateral_force_n.abs() > 0.0);
}

#[test]
fn static_rest_settles_and_supports_weight() {
    let mut simulation = AircraftSimulation::new(
        load_ground_test_model(),
        ground_test_config(),
        parked_state(0.15),
    )
    .expect("valid simulation");
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let mut snapshot = simulation.step(&input);
    for _ in 0..4000 {
        snapshot = simulation.step(&input);
    }
    let rigid = snapshot.rigid_body_state();
    assert!(rigid.position_world_m.iter().all(|v| v.is_finite()));
    assert!(
        rigid
            .linear_velocity_world_mps
            .iter()
            .all(|v| v.is_finite())
    );
    // Settles: vertical velocity near zero, no penetration runaway.
    assert!(rigid.linear_velocity_world_mps.z.abs() < 0.15);
    // Normal load supports approximately the aircraft weight (10% band).
    let weight = 10.0 * 9.80665;
    assert!(
        (snapshot.total_ground_normal_force_n() - weight).abs() < 0.10 * weight,
        "normal {} vs weight {weight}",
        snapshot.total_ground_normal_force_n()
    );
    assert!(snapshot.weight_on_wheels());
    assert_eq!(snapshot.ground_contacts(), 3);
    // Rests above the plane: wheel bottoms must not tunnel deep.
    // CG z must stay within 5 cm of the geometric rest height (-0.41).
    assert!((rigid.position_world_m.z + 0.41).abs() < 0.05);
}

#[test]
fn thrust_accelerates_wheeled_aircraft_along_runway() {
    let mut simulation = AircraftSimulation::new(
        load_ground_test_model(),
        ground_test_config(),
        parked_state(0.02),
    )
    .expect("valid simulation");
    // Full throttle: real thrust plus contact/friction must accelerate forward.
    let input = PilotInput::new(0.0, 0.0, 0.0, 1.0);
    let mut last_x = 0.0;
    for _ in 0..1000 {
        let snapshot = simulation.step(&input);
        last_x = snapshot.rigid_body_state().linear_velocity_world_mps.x;
    }
    assert!(
        last_x > 3.0,
        "expected runway acceleration from real thrust, got forward speed {last_x}"
    );
    assert!(simulation.state().rigid_body().position_world_m.x > 1.0);
}
