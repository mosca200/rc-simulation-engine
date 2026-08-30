use criterion::{Criterion, criterion_group, criterion_main};
use sim_core::{
    AeroElement, AeroEnvironment, PolarSample, PolarTable, RigidBodyParams, RigidBodyState,
    Rk4Integrator, evaluate_aero_element, evaluate_derivative,
};
use sim_math::{Mat3, Orientation, Vec3};
use std::hint::black_box;

fn polar() -> PolarTable {
    PolarTable::new(vec![
        PolarSample {
            alpha_rad: -0.5,
            cl: -0.8,
            cd: 0.08,
            cm: 0.04,
        },
        PolarSample {
            alpha_rad: 0.0,
            cl: 0.1,
            cd: 0.02,
            cm: -0.01,
        },
        PolarSample {
            alpha_rad: 0.5,
            cl: 1.0,
            cd: 0.1,
            cm: -0.08,
        },
    ])
    .unwrap()
}

fn state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(25.0, 1.0, 3.0),
        orientation_world_from_body: Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.3)),
        angular_velocity_body_radps: Vec3::new(0.2, -0.1, 0.4),
    }
}

fn benchmarks(criterion: &mut Criterion) {
    let polar = polar();
    let aero_element = AeroElement::new(
        Vec3::new(0.3, 0.8, -0.1),
        Orientation::from_scaled_axis(Vec3::new(0.05, -0.1, 0.02)),
        0.8,
        0.3,
    )
    .unwrap();
    let environment = AeroEnvironment::new(1.225, Vec3::new(2.0, -1.0, 0.5)).unwrap();
    let initial_state = state();
    let body_params =
        RigidBodyParams::new(2.0, Mat3::from_diagonal(&Vec3::new(0.08, 0.12, 0.16))).unwrap();
    let gravity = Vec3::new(0.0, 0.0, 9.80665);

    criterion.bench_function("B5/polar_lookup", |bencher| {
        bencher.iter(|| black_box(polar.sample_clamped(black_box(0.173))));
    });
    criterion.bench_function("B6/aero_element_evaluation", |bencher| {
        bencher.iter(|| {
            black_box(evaluate_aero_element(
                black_box(&initial_state),
                black_box(&aero_element),
                black_box(&environment),
                black_box(&polar),
            ))
        });
    });
    criterion.bench_function("B7/aero_rk4_step", |bencher| {
        bencher.iter(|| {
            black_box(Rk4Integrator::step(
                black_box(&initial_state),
                black_box(0.002),
                |stage_state| {
                    let aero = evaluate_aero_element(
                        stage_state,
                        black_box(&aero_element),
                        black_box(&environment),
                        black_box(&polar),
                    );
                    evaluate_derivative(
                        stage_state,
                        black_box(&body_params),
                        &aero.wrench_body,
                        black_box(&gravity),
                    )
                },
            ))
        });
    });
}

criterion_group!(aero_s4, benchmarks);
criterion_main!(aero_s4);
