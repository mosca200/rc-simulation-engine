use sim_core::{
    BodyWrench, PilotInput, RigidBodyParams, RigidBodyState, Rk4Integrator, Simulation,
    SimulationConfig, evaluate_derivative,
};
use sim_math::{Mat3, Orientation, Vec3, body_to_world};

const ANALYTIC_TOLERANCE: f64 = 2.0e-11;
const ANGULAR_INTEGRATION_TOLERANCE: f64 = 2.0e-10;
const GYROSCOPIC_TOLERANCE: f64 = 64.0 * f64::EPSILON;
// About 450x the measured 2.2e-14 maximum error over 20,000 steps on the pinned toolchain.
const CONSERVATION_RELATIVE_TOLERANCE: f64 = 1.0e-11;

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
        state = Rk4Integrator::step_with_constant_wrench(&state, params, wrench, gravity, dt_s);
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
fn rk4_evaluator_observes_four_distinct_stage_states_and_state_dependent_force() {
    let mut initial = state();
    initial.position_world_m = Vec3::new(1.0, 0.0, 0.0);
    initial.linear_velocity_world_mps = Vec3::new(1.0, 0.0, 0.0);
    let body_params = params(Mat3::identity());
    let gravity = Vec3::zeros();
    let mut observed_states = [initial; 4];
    let mut observed_force_x_n = [0.0; 4];
    let mut call_count = 0;

    let final_state = Rk4Integrator::step(&initial, 0.1, |stage_state| {
        let force_x_n = 1.0 + stage_state.position_world_m.x;
        observed_states[call_count] = *stage_state;
        observed_force_x_n[call_count] = force_x_n;
        call_count += 1;
        evaluate_derivative(
            stage_state,
            &body_params,
            &BodyWrench {
                force_body_n: Vec3::new(force_x_n, 0.0, 0.0),
                moment_body_nm: Vec3::zeros(),
            },
            &gravity,
        )
    });

    assert_eq!(call_count, 4);
    for stages in observed_states.windows(2) {
        assert_ne!(stages[0], stages[1]);
    }
    for forces in observed_force_x_n.windows(2) {
        assert!((forces[1] - forces[0]).abs() > f64::EPSILON);
    }
    assert!(final_state.linear_velocity_world_mps.x > initial.linear_velocity_world_mps.x);
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
fn nonidentity_orientation_composes_body_rate_delta_on_the_right() {
    let mut initial = state();
    initial.orientation_world_from_body = Orientation::from_scaled_axis(Vec3::new(0.4, -0.2, 0.1));
    initial.angular_velocity_body_radps = Vec3::new(-0.3, 0.6, 0.2);
    let duration_s = 0.75;
    let final_state = integrate(
        initial,
        &params(Mat3::from_diagonal_element(0.9)),
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.0005,
        1_500,
    );

    let delta_q_body =
        Orientation::from_scaled_axis(initial.angular_velocity_body_radps * duration_s);
    let expected = initial.orientation_world_from_body * delta_q_body;
    let wrong_left_composition = delta_q_body * initial.orientation_world_from_body;
    let error = final_state.orientation_world_from_body.angle_to(&expected);
    assert!(error <= 5.0e-12, "right-composition error={error:e}");
    assert!(
        final_state
            .orientation_world_from_body
            .angle_to(&wrong_left_composition)
            > 1.0e-3,
        "test setup does not distinguish q0 * delta_q_body from delta_q_body * q0"
    );
}

#[test]
fn nonspherical_inertia_has_expected_initial_gyroscopic_acceleration() {
    let inertia = Mat3::from_diagonal(&Vec3::new(2.0, 3.0, 4.0));
    let body_params = params(inertia);
    let mut initial = state();
    initial.angular_velocity_body_radps = Vec3::new(1.0, 2.0, 3.0);
    let derivative =
        evaluate_derivative(&initial, &body_params, &BodyWrench::zero(), &Vec3::zeros());

    // -I^-1 * (omega x I*omega) = [-3, 2, -0.5] for this diagonal inertia.
    assert_vec_close(
        derivative.angular_velocity_body_radps2,
        Vec3::new(-3.0, 2.0, -0.5),
        GYROSCOPIC_TOLERANCE,
    );
}

#[test]
fn torque_free_nonspherical_body_conserves_energy_and_world_angular_momentum() {
    let inertia = Mat3::from_diagonal(&Vec3::new(0.7, 1.1, 1.6));
    let body_params = params(inertia);
    let mut initial = state();
    initial.orientation_world_from_body = Orientation::from_scaled_axis(Vec3::new(0.2, -0.3, 0.1));
    initial.angular_velocity_body_radps = Vec3::new(0.7, -0.4, 1.1);

    let initial_momentum_body = inertia * initial.angular_velocity_body_radps;
    let initial_momentum_world =
        body_to_world(&initial.orientation_world_from_body, &initial_momentum_body);
    let initial_energy = 0.5
        * initial
            .angular_velocity_body_radps
            .dot(&initial_momentum_body);

    let final_state = integrate(
        initial,
        &body_params,
        &BodyWrench::zero(),
        &Vec3::zeros(),
        0.0005,
        20_000,
    );
    let final_momentum_body = inertia * final_state.angular_velocity_body_radps;
    let final_momentum_world = body_to_world(
        &final_state.orientation_world_from_body,
        &final_momentum_body,
    );
    let final_energy = 0.5
        * final_state
            .angular_velocity_body_radps
            .dot(&final_momentum_body);

    let relative_energy_error = (final_energy - initial_energy).abs() / initial_energy;
    let relative_momentum_error =
        (final_momentum_world - initial_momentum_world).norm() / initial_momentum_world.norm();
    assert!(
        relative_energy_error <= CONSERVATION_RELATIVE_TOLERANCE,
        "relative energy error={relative_energy_error:e}"
    );
    assert!(
        relative_momentum_error <= CONSERVATION_RELATIVE_TOLERANCE,
        "relative world angular-momentum error={relative_momentum_error:e}"
    );
    assert!(
        (final_state.angular_velocity_body_radps - initial.angular_velocity_body_radps).norm()
            > 0.1,
        "torque-free nonspherical test must exercise non-constant body rates"
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
