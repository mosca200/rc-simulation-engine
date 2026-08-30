use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use sim_core::{
    BodyWrench, PilotInput, RigidBodyParams, RigidBodyState, Rk4Integrator, Simulation,
    SimulationConfig, evaluate_derivative,
};
use sim_math::{Mat3, Orientation, Vec3};
use std::{hint::black_box, time::Duration};

fn state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(12.0, 1.0, -0.5),
        orientation_world_from_body: Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.3)),
        angular_velocity_body_radps: Vec3::new(0.2, -0.1, 0.4),
    }
}

fn params() -> RigidBodyParams {
    RigidBodyParams::new(2.0, Mat3::from_diagonal(&Vec3::new(0.08, 0.12, 0.16))).unwrap()
}

fn benchmarks(criterion: &mut Criterion) {
    let initial_state = state();
    let body_params = params();
    let wrench = BodyWrench {
        force_body_n: Vec3::new(1.0, 2.0, -3.0),
        moment_body_nm: Vec3::new(0.01, -0.02, 0.03),
    };
    let gravity = Vec3::new(0.0, 0.0, 9.80665);

    criterion.bench_function("B1/evaluate_derivative", |bencher| {
        bencher.iter(|| {
            black_box(evaluate_derivative(
                black_box(&initial_state),
                black_box(&body_params),
                black_box(&wrench),
                black_box(&gravity),
            ))
        });
    });
    criterion.bench_function("B2/rk4_step", |bencher| {
        bencher.iter(|| {
            black_box(Rk4Integrator::step(
                black_box(&initial_state),
                black_box(&body_params),
                black_box(&wrench),
                black_box(&gravity),
                black_box(0.002),
            ))
        });
    });

    let mut step_group = criterion.benchmark_group("B3/simulation_step");
    for physics_hz in [250_u32, 500, 1_000] {
        let mut simulation = Simulation::new(
            SimulationConfig::from_physics_hz(physics_hz).unwrap(),
            params(),
            initial_state,
        )
        .unwrap();
        step_group.bench_with_input(
            BenchmarkId::from_parameter(physics_hz),
            &physics_hz,
            |bencher, _| {
                bencher.iter(|| black_box(simulation.step(black_box(&PilotInput::neutral()))));
            },
        );
    }
    step_group.finish();

    let mut simulation =
        Simulation::new(SimulationConfig::default(), params(), initial_state).unwrap();
    let input = PilotInput::neutral();
    let mut loop_group = criterion.benchmark_group("B4/headless_loop");
    loop_group.throughput(Throughput::Elements(100_000));
    loop_group.sample_size(10);
    loop_group.measurement_time(Duration::from_secs(3));
    loop_group.bench_function("100k_steps", |bencher| {
        bencher.iter(|| {
            for _ in 0..100_000 {
                let _ = simulation.step(black_box(&input));
            }
            // Observe the full state-transition chain without hashing inside each physics step.
            black_box(simulation.snapshot().state_hash());
        });
    });
    loop_group.finish();
}

criterion_group!(foundation, benchmarks);
criterion_main!(foundation);
