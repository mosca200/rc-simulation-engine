use sim_core::{
    AirframeContact, FlatGroundPlane, GroundSurface, RigidBodyState,
    evaluate_airframe_ground_wrench, validate_airframe_contact,
};
use sim_math::{Orientation, Vec3};

fn contact(position_body_m: Vec3) -> AirframeContact {
    AirframeContact {
        position_body_m,
        stiffness_n_per_m: 8_000.0,
        damping_n_s_per_m: 350.0,
        friction_mu: 0.45,
    }
}

fn state(cg_down_m: f64, velocity_world_mps: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, cg_down_m),
        linear_velocity_world_mps: velocity_world_mps,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[test]
fn airborne_and_clear_points_produce_exactly_zero_wrench() {
    let contacts = [contact(Vec3::new(0.0, 0.0, 0.05))];
    let evaluation = evaluate_airframe_ground_wrench(
        &state(-2.0, Vec3::new(12.0, -3.0, 1.0)),
        &contacts,
        &GroundSurface::Flat(FlatGroundPlane::default()),
    );
    assert_eq!(evaluation.active_contacts, 0);
    assert_eq!(evaluation.force_body_n, Vec3::zeros());
    assert_eq!(evaluation.moment_body_nm, Vec3::zeros());
}

#[test]
fn point_at_the_surface_without_penetration_produces_no_spurious_force() {
    let contacts = [contact(Vec3::new(0.0, 0.0, 0.05))];
    let evaluation = evaluate_airframe_ground_wrench(
        &state(-0.05, Vec3::new(0.0, 0.0, -3.0)),
        &contacts,
        &GroundSurface::default(),
    );
    assert_eq!(evaluation.active_contacts, 0);
    assert_eq!(evaluation.total_normal_force_n, 0.0);
    assert_eq!(evaluation.total_tangential_force_n, 0.0);
}

#[test]
fn belly_contact_is_unilateral_and_friction_opposes_slip() {
    let contacts = [contact(Vec3::new(-0.12, 0.0, 0.065))];
    let evaluation = evaluate_airframe_ground_wrench(
        &state(-0.05, Vec3::new(2.0, 0.0, 1.0)),
        &contacts,
        &GroundSurface::default(),
    );
    assert_eq!(evaluation.active_contacts, 1);
    assert!((evaluation.contacts[0].penetration_m - 0.015).abs() < 1.0e-12);
    assert!(evaluation.force_body_n.z < 0.0);
    assert!(evaluation.force_body_n.x < 0.0);
    assert!(evaluation.force_body_n.dot(&Vec3::new(2.0, 0.0, 0.0)) <= 0.0);
}

#[test]
fn symmetric_tip_strikes_generate_opposite_roll_moments() {
    let surface = GroundSurface::default();
    let left = contact(Vec3::new(-0.05, -0.9, 0.015));
    let right = contact(Vec3::new(-0.05, 0.9, 0.015));
    let strike = state(-0.005, Vec3::zeros());
    let left_evaluation = evaluate_airframe_ground_wrench(&strike, &[left], &surface);
    let right_evaluation = evaluate_airframe_ground_wrench(&strike, &[right], &surface);

    assert_eq!(left_evaluation.active_contacts, 1);
    assert_eq!(right_evaluation.active_contacts, 1);
    assert!(left_evaluation.moment_body_nm.x > 0.0);
    assert!(right_evaluation.moment_body_nm.x < 0.0);
    assert!((left_evaluation.moment_body_nm.x + right_evaluation.moment_body_nm.x).abs() < 1.0e-12);
    assert!(
        (left_evaluation.total_normal_force_n - right_evaluation.total_normal_force_n).abs()
            < 1.0e-12
    );
}

#[test]
fn invalid_structural_contact_is_rejected_before_the_hot_loop() {
    let mut invalid = contact(Vec3::zeros());
    invalid.friction_mu = -0.1;
    assert!(validate_airframe_contact(&invalid).is_err());
    invalid.friction_mu = 0.2;
    invalid.position_body_m.x = f64::NAN;
    assert!(validate_airframe_contact(&invalid).is_err());
}

#[test]
fn structural_contact_hot_loop_allocates_nothing() {
    let contacts = [
        contact(Vec3::new(-0.12, 0.0, 0.065)),
        contact(Vec3::new(-0.05, -0.9, 0.015)),
        contact(Vec3::new(-0.05, 0.9, 0.015)),
    ];
    let penetrating = state(-0.01, Vec3::new(2.0, 0.5, 1.0));
    let surface = GroundSurface::default();
    let allocations = allocation_counter::measure(|| {
        for _ in 0..1_000 {
            std::hint::black_box(evaluate_airframe_ground_wrench(
                &penetrating,
                &contacts,
                &surface,
            ));
        }
    });
    assert_eq!(allocations.count_total, 0, "{allocations:?}");
}
