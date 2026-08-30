use sim_core::{
    BodyWrench, PilotInput, RigidBodyParams, RigidBodyState, Rk4Integrator, Simulation,
    SimulationConfig,
};
use sim_math::{Mat3, Orientation, Vec3};

const ANALYTIC_TOLERANCE: f64 = 2.0e-11;
const ANGULAR_INTEGRATION_TOLERANCE: f64 = 2.0e-10;

fn params(inertia: Mat3) -> RigidBodyParams {
    RigidBodyParams::new(2.0, inertia).unwrap()
}

fn state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(1.0, -2.0, 3.0),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn integrate(
    mut state: RigidBodyState,
    params: &RigidBodyParams,
    wrench: &BodyWrench,
    gravity: &Vec3,
    dt_s: f64,
    steps: usize,
) -> RigidBodyState {
    for _ in 0..steps {
        state = Rk4Integrator::step(&state, params, wrench, gravity, dt_s);
    }
    state
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    let error = (actual - expected).norm();
    assert!(
        error <= tolerance,
        "error={error:e}, actual={actual:?}, expected={expected:?}"
    );
}

#[test]
fn t1_stationary_body_zero_gravity_is_unchanged() {
    let initial = state();
    let final_state = integrate(
        initial,
        &params(Mat3::identity()),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.002,
        1_000,
    );
    assert_eq!(final_state, initial);
}

#[test]
fn t2_constant_linear_velocity_matches_analytic_solution() {
    let mut initial = state();
    initial.linear_velocity_world_mps = Vec3::new(7.0, -3.0, 0.5);
    let duration_s = 2.0;
    let final_state = integrate(
        initial,
        &params(Mat3::identity()),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.002,
        1_000,
    );
    assert_vec_close(
        final_state.position_world_m,
        initial.position_world_m + initial.linear_velocity_world_mps * duration_s,
        ANALYTIC_TOLERANCE,
    );
    assert_eq!(
        final_state.linear_velocity_world_mps,
        initial.linear_velocity_world_mps
    );
}

#[test]
fn t3_gravity_only_matches_ned_analytic_solution() {
    let mut initial = state();
    initial.linear_velocity_world_mps = Vec3::new(1.0, 2.0, -4.0);
    let gravity = Vec3::new(0.0, 0.0, 9.80665);
    let duration_s = 1.0;
    let final_state = integrate(
        initial,
        &params(Mat3::identity()),
        &BodyWrench::zero(),
        &gravity,
        0.002,
        500,
    );
    assert_vec_close(
        final_state.linear_velocity_world_mps,
        initial.linear_velocity_world_mps + gravity * duration_s,
        ANALYTIC_TOLERANCE,
    );
    assert_vec_close(
        final_state.position_world_m,
        initial.position_world_m
            + initial.linear_velocity_world_mps * duration_s
            + gravity * (0.5 * duration_s * duration_s),
        ANALYTIC_TOLERANCE,
    );
}

#[test]
fn t4_constant_body_force_without_rotation_matches_analytic_solution() {
    let initial = state();
    let wrench = BodyWrench {
        force_body_n: Vec3::new(4.0, -2.0, 6.0),
        moment_body_nm: Vec3::zeros(),
    };
    let duration_s = 0.5;
    let acceleration = wrench.force_body_n / 2.0;
    let final_state = integrate(
        initial,
        &params(Mat3::identity()),
        &wrench,
        &Vec3::zeros(),
        0.001,
        500,
    );
    assert_vec_close(
        final_state.linear_velocity_world_mps,
        acceleration * duration_s,
        ANALYTIC_TOLERANCE,
    );
    assert_vec_close(
        final_state.position_world_m,
        initial.position_world_m + acceleration * (0.5 * duration_s * duration_s),
        ANALYTIC_TOLERANCE,
    );
}

#[test]
fn t7_constant_body_rate_with_spherical_inertia_matches_rotation() {
    let mut initial = state();
    initial.angular_velocity_body_radps = Vec3::new(0.3, -0.2, 0.5);
    let duration_s = 1.0;
    let final_state = integrate(
        initial,
        &params(Mat3::from_diagonal_element(0.7)),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.001,
        1_000,
    );
    assert_vec_close(
        final_state.angular_velocity_body_radps,
        initial.angular_velocity_body_radps,
        ANALYTIC_TOLERANCE,
    );
    let expected = Orientation::from_scaled_axis(initial.angular_velocity_body_radps * duration_s);
    assert!(
        final_state.orientation_world_from_body.angle_to(&expected)
            <= ANGULAR_INTEGRATION_TOLERANCE
    );
}

#[test]
fn t8_quaternion_norm_remains_unity_after_long_run() {
    let mut initial = state();
    initial.angular_velocity_body_radps = Vec3::new(0.7, 0.2, -0.4);
    let final_state = integrate(
        initial,
        &params(Mat3::from_diagonal_element(1.2)),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.002,
        100_000,
    );
    let norm = final_state.orientation_world_from_body.quaternion().norm();
    assert!((norm - 1.0).abs() <= 8.0 * f64::EPSILON, "norm={norm:.17}");
}

#[test]
fn t9_rk4_exhibits_fourth_order_convergence_for_rotation() {
    let mut initial = state();
    initial.angular_velocity_body_radps = Vec3::new(1.1, -0.6, 0.3);
    let expected = Orientation::from_scaled_axis(initial.angular_velocity_body_radps);
    let coarse = integrate(
        initial,
        &params(Mat3::identity()),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.1,
        10,
    );
    let fine = integrate(
        initial,
        &params(Mat3::identity()),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.05,
        20,
    );
    let coarse_error = coarse.orientation_world_from_body.angle_to(&expected);
    let fine_error = fine.orientation_world_from_body.angle_to(&expected);
    assert!(coarse_error > 0.0);
    assert!(
        coarse_error / fine_error > 12.0,
        "coarse={coarse_error:e}, fine={fine_error:e}"
    );
}

#[test]
fn t10_fixed_step_accounting_uses_post_step_semantics() {
    let config = SimulationConfig::from_physics_hz(500).unwrap();
    let mut simulation = Simulation::new(config, params(Mat3::identity()), state()).unwrap();
    let input = PilotInput::neutral();
    let mut snapshot = simulation.snapshot();
    for _ in 0..1_337 {
        snapshot = simulation.step(&input);
    }
    assert_eq!(snapshot.step_index, 1_337);
    assert_eq!(simulation.step_index(), 1_337);
    assert_eq!(snapshot.sim_time_s, 1_337.0 * 0.002);
    assert_eq!(simulation.sim_time_s(), 1_337.0 * 0.002);
}

#[test]
fn simulation_step_allocates_nothing_after_initialization() {
    let config = SimulationConfig::default();
    let mut simulation = Simulation::new(config, params(Mat3::identity()), state()).unwrap();
    let input = PilotInput::neutral();
    let _ = simulation.step(&input);
    let allocation_info = allocation_counter::measure(|| {
        let _ = simulation.step(&input);
    });
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}
