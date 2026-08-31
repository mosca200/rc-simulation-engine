use aircraft::{AircraftSimulation, AircraftSimulationConfig, evaluate_aerodynamic_wrench};
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use model::AircraftModelLoader;
use sim_core::{PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::hint::black_box;

const ACRO_MODEL_JSON: &str = include_str!("../../../models/acro_electric_01/model.json");

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(24.0, 0.4, 1.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::new(0.05, -0.03, 0.02),
    }
}

fn simulation() -> AircraftSimulation {
    AircraftSimulation::new(
        AircraftModelLoader::from_json_str(ACRO_MODEL_JSON).unwrap(),
        AircraftSimulationConfig::default(),
        initial_state(),
    )
    .unwrap()
}

fn benchmarks(criterion: &mut Criterion) {
    let simulation = simulation();
    criterion.bench_function("B17/aggregate_acro_aero_wrench", |bencher| {
        bencher.iter(|| {
            black_box(evaluate_aerodynamic_wrench(
                black_box(simulation.state().rigid_body()),
                black_box(simulation.effective_aero_elements()),
                black_box(simulation.model()),
                black_box(simulation.config().aero_environment()),
            ))
        });
    });

    let input = PilotInput::new(0.15, -0.1, 0.05, 0.62);
    criterion.bench_function("B18/complete_aircraft_step", |bencher| {
        bencher.iter_batched_ref(
            || simulation.clone(),
            |simulation| black_box(simulation.step(black_box(&input))),
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(aircraft_s6_1, benchmarks);
criterion_main!(aircraft_s6_1);
