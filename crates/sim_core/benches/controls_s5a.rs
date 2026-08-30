use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use sim_core::{
    AxisResponseConfig, ControlActuatorConfig, ControlResponseConfig, ControlSystemConfig,
    ControlSystemState, PilotInput, ServoConfig, ServoState, advance_controls, advance_servo,
    shape_pilot_input,
};
use std::hint::black_box;

fn response_config() -> ControlResponseConfig {
    ControlResponseConfig::new(
        AxisResponseConfig::new(0.85, 0.35).unwrap(),
        AxisResponseConfig::new(0.75, 0.45).unwrap(),
        AxisResponseConfig::new(0.65, 0.55).unwrap(),
    )
}

fn servo_config(reversed: bool) -> ServoConfig {
    ServoConfig::new(-0.45, 0.02, 0.55, 3.0, reversed).unwrap()
}

fn control_config() -> ControlSystemConfig {
    ControlSystemConfig::new(
        response_config(),
        ControlActuatorConfig::new(servo_config(false), servo_config(false), servo_config(true)),
    )
}

fn benchmarks(criterion: &mut Criterion) {
    let input = PilotInput::new(0.63, -0.41, 0.27, 0.72);
    let response = response_config();
    let servo = servo_config(false);
    let controls = control_config();

    criterion.bench_function("B8/rates_expo_three_axes", |bencher| {
        bencher.iter(|| black_box(shape_pilot_input(black_box(&input), black_box(&response))));
    });
    criterion.bench_function("B9/servo_single_update", |bencher| {
        bencher.iter_batched(
            || ServoState::neutral(&servo),
            |mut state| {
                black_box(advance_servo(
                    black_box(&mut state),
                    black_box(&servo),
                    black_box(0.63),
                    black_box(0.002),
                ))
            },
            BatchSize::SmallInput,
        );
    });
    criterion.bench_function("B10/complete_controls_pipeline", |bencher| {
        bencher.iter_batched(
            || ControlSystemState::neutral(&controls),
            |mut state| {
                black_box(advance_controls(
                    black_box(&mut state),
                    black_box(&controls),
                    black_box(&input),
                    black_box(0.002),
                ))
            },
            BatchSize::SmallInput,
        );
    });
}

criterion_group!(controls_s5a, benchmarks);
criterion_main!(controls_s5a);
