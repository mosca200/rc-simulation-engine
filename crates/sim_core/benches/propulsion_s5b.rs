use criterion::{Criterion, criterion_group, criterion_main};
use sim_core::{
    AeroEnvironment, BatteryConfig, ElectricPropulsionConfig, MotorConfig,
    PropellerCoefficientTable, PropellerConfig, PropellerSample, PropellerSpinDirection,
    RigidBodyParams, RigidBodyState, Rk4Integrator, evaluate_derivative,
    evaluate_electric_propulsion, evaluate_electrical_drive,
};
use sim_math::{Mat3, Orientation, Vec3};
use std::hint::black_box;

fn coefficient_table() -> PropellerCoefficientTable {
    PropellerCoefficientTable::new(vec![
        PropellerSample {
            advance_ratio_j: -0.25,
            ct: 0.135,
            cq: 0.019,
        },
        PropellerSample {
            advance_ratio_j: 0.0,
            ct: 0.125,
            cq: 0.018,
        },
        PropellerSample {
            advance_ratio_j: 0.5,
            ct: 0.09,
            cq: 0.013,
        },
        PropellerSample {
            advance_ratio_j: 1.0,
            ct: 0.04,
            cq: 0.007,
        },
        PropellerSample {
            advance_ratio_j: 1.5,
            ct: 0.0,
            cq: 0.002,
        },
    ])
    .unwrap()
}

fn state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(24.0, 1.0, 2.5),
        orientation_world_from_body: Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.3)),
        angular_velocity_body_radps: Vec3::new(0.2, -0.1, 0.4),
    }
}

fn benchmarks(criterion: &mut Criterion) {
    let battery = BatteryConfig::new(16.8, 0.035).unwrap();
    let motor = MotorConfig::new(900.0, 0.045, 1.2).unwrap();
    let propeller = PropellerConfig::new(
        Vec3::new(0.45, 0.05, -0.1),
        Orientation::from_scaled_axis(Vec3::new(0.02, -0.04, 0.01)),
        0.28,
        PropellerSpinDirection::PositiveAboutLocalX,
    )
    .unwrap();
    let config = ElectricPropulsionConfig::new(battery, motor, propeller);
    let table = coefficient_table();
    let environment = AeroEnvironment::new(1.225, Vec3::new(2.0, -0.5, 0.25)).unwrap();
    let initial_state = state();
    let body_params =
        RigidBodyParams::new(2.5, Mat3::from_diagonal(&Vec3::new(0.08, 0.12, 0.16))).unwrap();
    let gravity = Vec3::new(0.0, 0.0, 9.80665);

    criterion.bench_function("B11/propeller_coefficient_lookup", |bencher| {
        bencher.iter(|| black_box(black_box(&table).sample_clamped(black_box(0.437))));
    });
    criterion.bench_function("B12/electrical_known_omega", |bencher| {
        bencher.iter(|| {
            black_box(evaluate_electrical_drive(
                black_box(0.72),
                black_box(650.0),
                black_box(&battery),
                black_box(&motor),
            ))
        });
    });
    criterion.bench_function("B13/electric_propulsion_operating_point", |bencher| {
        bencher.iter(|| {
            black_box(evaluate_electric_propulsion(
                black_box(&initial_state),
                black_box(0.72),
                black_box(&config),
                black_box(&environment),
                black_box(&table),
            ))
        });
    });
    criterion.bench_function("B14/propulsion_rk4_step", |bencher| {
        bencher.iter(|| {
            black_box(Rk4Integrator::step(
                black_box(&initial_state),
                black_box(0.002),
                |stage_state| {
                    let propulsion = evaluate_electric_propulsion(
                        stage_state,
                        black_box(0.72),
                        black_box(&config),
                        black_box(&environment),
                        black_box(&table),
                    );
                    evaluate_derivative(
                        stage_state,
                        black_box(&body_params),
                        &propulsion.wrench_body,
                        black_box(&gravity),
                    )
                },
            ))
        });
    });
}

criterion_group!(propulsion_s5b, benchmarks);
criterion_main!(propulsion_s5b);
