use sim_core::{
    AxisResponseConfig, ControlActuatorConfig, ControlConfigError, ControlResponseConfig,
    ControlSystemConfig, ControlSystemState, PilotInput, ServoConfig, ServoState, advance_controls,
    advance_servo, evaluate_steady_controls, mix_conventional, shape_pilot_input,
};
use std::hint::black_box;

const TOLERANCE: f64 = 16.0 * f64::EPSILON;

fn axis(rate: f64, expo: f64) -> AxisResponseConfig {
    AxisResponseConfig::new(rate, expo).unwrap()
}

fn response() -> ControlResponseConfig {
    ControlResponseConfig::new(axis(0.8, 0.3), axis(0.7, 0.4), axis(0.6, 0.5))
}

fn servo(reversed: bool) -> ServoConfig {
    ServoConfig::new(-0.4, 0.0, 0.6, 2.0, reversed).unwrap()
}

fn system_config() -> ControlSystemConfig {
    ControlSystemConfig::new(
        response(),
        ControlActuatorConfig::new(servo(false), servo(false), servo(true)),
    )
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= TOLERANCE,
        "actual={actual:.17}, expected={expected:.17}"
    );
}

#[test]
fn c1_pilot_input_clamps_and_neutralizes_realtime_values() {
    let clamped = PilotInput::new(-2.0, 4.0, 1.5, 3.0);
    assert_eq!(clamped.roll(), -1.0);
    assert_eq!(clamped.pitch(), 1.0);
    assert_eq!(clamped.yaw(), 1.0);
    assert_eq!(clamped.throttle(), 1.0);

    let non_finite = PilotInput::new(f64::NAN, f64::INFINITY, f64::NEG_INFINITY, f64::NAN);
    assert_eq!(non_finite, PilotInput::neutral());
}

#[test]
fn c4_zero_expo_is_linear_rate_scaling() {
    let config = axis(0.65, 0.0);
    for command in [-1.0, -0.4, 0.0, 0.25, 1.0] {
        assert_close(config.shape(command), 0.65 * command);
    }
}

#[test]
fn c5_full_expo_is_cubic_rate_scaling() {
    let config = axis(0.75, 1.0);
    for command in [-1.0_f64, -0.4, 0.0, 0.25, 1.0] {
        assert_close(config.shape(command), 0.75 * command.powi(3));
    }
}

#[test]
fn c6_axis_response_has_odd_symmetry_on_deterministic_grid() {
    let config = axis(0.83, 0.61);
    for index in 0..=100 {
        let command = f64::from(index) / 100.0;
        assert_eq!(
            config.shape(-command).to_bits(),
            (-config.shape(command)).to_bits()
        );
    }
    assert_eq!(config.shape(0.0), 0.0);
}

#[test]
fn c7_response_endpoints_equal_signed_rate() {
    let config = axis(0.73, 0.42);
    assert_eq!(config.shape(-1.0), -0.73);
    assert_eq!(config.shape(1.0), 0.73);
}

#[test]
fn c8_conventional_mixer_preserves_logical_semantics() {
    let input = PilotInput::new(0.25, -0.5, 0.75, 0.6);
    let unity = ControlResponseConfig::new(axis(1.0, 0.0), axis(1.0, 0.0), axis(1.0, 0.0));
    let shaped = shape_pilot_input(&input, &unity);
    let targets = mix_conventional(&shaped);
    assert_eq!(targets.aileron(), 0.25);
    assert_eq!(targets.elevator(), -0.5);
    assert_eq!(targets.rudder(), 0.75);
    assert_eq!(targets.throttle(), 0.6);
}

#[test]
fn c9_servo_endpoints_map_to_configured_travel() {
    let config = servo(false);
    assert_eq!(config.target_angle_rad(-1.0), config.min_angle_rad());
    assert_eq!(config.target_angle_rad(0.0), config.neutral_angle_rad());
    assert_eq!(config.target_angle_rad(1.0), config.max_angle_rad());
}

#[test]
fn c10_asymmetric_servo_travel_uses_piecewise_mapping() {
    let config = ServoConfig::new(-0.2, 0.1, 0.7, 1.0, false).unwrap();
    assert_close(config.target_angle_rad(-0.5), -0.05);
    assert_close(config.target_angle_rad(0.5), 0.4);
}

#[test]
fn c11_reversed_servo_inverts_only_the_physical_command() {
    let normal = servo(false);
    let reversed = servo(true);
    assert_eq!(
        reversed.target_angle_rad(0.5),
        normal.target_angle_rad(-0.5)
    );
    assert_eq!(
        reversed.target_angle_rad(-0.5),
        normal.target_angle_rad(0.5)
    );
}

#[test]
fn c12_servo_update_respects_exact_speed_limit() {
    let config = servo(false);
    let mut state = ServoState::neutral(&config);
    let angle = advance_servo(&mut state, &config, 1.0, 0.1);
    assert_eq!(angle, config.max_speed_rad_s() * 0.1);
}

#[test]
fn c13_servo_captures_reachable_target_without_overshoot() {
    let config = servo(false);
    let mut state = ServoState::neutral(&config);
    let target = config.target_angle_rad(0.1);
    let angle = advance_servo(&mut state, &config, 0.1, 1.0);
    assert_eq!(angle.to_bits(), target.to_bits());
}

#[test]
fn c14_servo_converges_monotonically_to_constant_target() {
    let config = servo(false);
    let mut state = ServoState::neutral(&config);
    let target = config.target_angle_rad(1.0);
    let mut previous = state.angle_rad();
    for _ in 0..100 {
        let angle = advance_servo(&mut state, &config, 1.0, 0.01);
        assert!(angle >= previous);
        assert!(angle <= target);
        previous = angle;
    }
    assert_eq!(state.angle_rad(), target);
}

#[test]
fn c15_one_full_step_matches_two_half_steps_before_target_capture() {
    let config = ServoConfig::new(-2.0, 0.0, 2.0, 0.75, false).unwrap();
    let mut full = ServoState::neutral(&config);
    let mut halves = ServoState::neutral(&config);
    let full_angle = advance_servo(&mut full, &config, 1.0, 0.2);
    let _ = advance_servo(&mut halves, &config, 1.0, 0.1);
    let half_angle = advance_servo(&mut halves, &config, 1.0, 0.1);
    assert_close(full_angle, half_angle);
}

#[test]
fn c16_neutral_state_is_stable_for_many_steps() {
    let config = servo(false);
    let mut state = ServoState::neutral(&config);
    for _ in 0..10_000 {
        assert_eq!(advance_servo(&mut state, &config, 0.0, 0.002), 0.0);
    }
}

#[test]
fn c17_complete_pipeline_matches_manual_one_step_calculation() {
    let response = ControlResponseConfig::new(axis(0.8, 0.5), axis(1.0, 0.0), axis(0.5, 1.0));
    let actuators = ControlActuatorConfig::new(
        ServoConfig::new(-0.4, 0.0, 0.6, 10.0, false).unwrap(),
        ServoConfig::new(-0.3, 0.1, 0.5, 100.0, false).unwrap(),
        ServoConfig::new(-0.2, 0.0, 0.4, 100.0, true).unwrap(),
    );
    let config = ControlSystemConfig::new(response, actuators);
    let mut state = ControlSystemState::neutral(&config);
    let input = PilotInput::new(0.5, -0.5, 1.0, 0.7);

    let shaped = shape_pilot_input(&input, config.response());
    assert_eq!(shaped.roll(), 0.25);
    assert_eq!(shaped.pitch(), -0.5);
    assert_eq!(shaped.yaw(), 0.5);
    assert_eq!(shaped.throttle(), 0.7);

    let positions = advance_controls(&mut state, &config, &input, 0.01);
    assert_eq!(positions.aileron_angle_rad(), 0.1);
    assert_close(positions.elevator_angle_rad(), -0.1);
    assert_eq!(positions.rudder_angle_rad(), -0.1);
    assert_eq!(positions.throttle(), 0.7);
}

#[test]
fn c18_throttle_bypasses_servo_lag() {
    let config = system_config();
    let mut state = ControlSystemState::neutral(&config);
    for throttle in [0.0, 0.2, 0.8, 1.0] {
        let positions = advance_controls(
            &mut state,
            &config,
            &PilotInput::new(1.0, -1.0, 1.0, throttle),
            1.0e-6,
        );
        assert_eq!(positions.throttle(), throttle);
    }
}

#[test]
fn c19_invalid_control_configuration_is_rejected() {
    assert_eq!(
        AxisResponseConfig::new(f64::NAN, 0.0),
        Err(ControlConfigError::InvalidAxisRate)
    );
    assert_eq!(
        AxisResponseConfig::new(-0.1, 0.0),
        Err(ControlConfigError::InvalidAxisRate)
    );
    assert_eq!(
        AxisResponseConfig::new(1.1, 0.0),
        Err(ControlConfigError::InvalidAxisRate)
    );
    assert_eq!(
        AxisResponseConfig::new(1.0, f64::INFINITY),
        Err(ControlConfigError::InvalidAxisExpo)
    );
    assert_eq!(
        AxisResponseConfig::new(1.0, -0.1),
        Err(ControlConfigError::InvalidAxisExpo)
    );
    assert_eq!(
        AxisResponseConfig::new(1.0, 1.1),
        Err(ControlConfigError::InvalidAxisExpo)
    );
    assert_eq!(
        ServoConfig::new(f64::NAN, 0.0, 1.0, 1.0, false),
        Err(ControlConfigError::NonFiniteServoConfig)
    );
    assert_eq!(
        ServoConfig::new(0.0, 0.0, 1.0, 1.0, false),
        Err(ControlConfigError::InvalidServoTravel)
    );
    assert_eq!(
        ServoConfig::new(-1.0, 1.0, 1.0, 1.0, false),
        Err(ControlConfigError::InvalidServoTravel)
    );
    assert_eq!(
        ServoConfig::new(-1.0, 0.0, 1.0, 0.0, false),
        Err(ControlConfigError::InvalidServoSpeed)
    );
    let config = servo(false);
    assert_eq!(
        ServoState::from_angle(&config, f64::NAN),
        Err(ControlConfigError::InvalidInitialServoAngle)
    );
    assert_eq!(
        ServoState::from_angle(&config, 0.7),
        Err(ControlConfigError::InvalidInitialServoAngle)
    );
}

#[test]
fn c20_repeated_control_run_is_bit_identical() {
    let config = system_config();
    let run = || {
        let mut state = ControlSystemState::neutral(&config);
        for index in 0..20_000 {
            let phase = f64::from(index) * 0.013;
            let input = PilotInput::new(
                phase.sin(),
                (phase * 0.7).cos(),
                (phase * 0.3).sin(),
                0.5 + 0.4 * phase.sin(),
            );
            let _ = advance_controls(&mut state, &config, &input, 0.002);
        }
        state
    };
    let first = run();
    let second = run();
    assert_eq!(
        first.actuators().aileron().angle_rad().to_bits(),
        second.actuators().aileron().angle_rad().to_bits()
    );
    assert_eq!(
        first.actuators().elevator().angle_rad().to_bits(),
        second.actuators().elevator().angle_rad().to_bits()
    );
    assert_eq!(
        first.actuators().rudder().angle_rad().to_bits(),
        second.actuators().rudder().angle_rad().to_bits()
    );
}

#[test]
fn m2_5_steady_controls_equal_the_eventual_rate_limited_targets() {
    let config = system_config();
    let input = PilotInput::new(0.45, -0.35, 0.25, 0.72);
    let steady = evaluate_steady_controls(&config, &input);
    let mut state = ControlSystemState::neutral(&config);
    let mut dynamic = advance_controls(&mut state, &config, &input, 0.002);
    for _ in 0..1_000 {
        dynamic = advance_controls(&mut state, &config, &input, 0.002);
    }
    assert_eq!(steady, dynamic);
}

#[test]
fn controls_hot_paths_allocate_nothing_after_initialization() {
    let config = system_config();
    let input = PilotInput::new(0.4, -0.3, 0.2, 0.75);
    let shaped = shape_pilot_input(&input, config.response());
    let servo_config = *config.actuators().aileron();
    let mut servo_state = ServoState::neutral(&servo_config);
    let mut system_state = ControlSystemState::neutral(&config);

    let shaping_allocations = allocation_counter::measure(|| {
        black_box(shape_pilot_input(
            black_box(&input),
            black_box(config.response()),
        ));
    });
    let mixer_allocations = allocation_counter::measure(|| {
        black_box(mix_conventional(black_box(&shaped)));
    });
    let servo_allocations = allocation_counter::measure(|| {
        black_box(advance_servo(
            black_box(&mut servo_state),
            black_box(&servo_config),
            black_box(0.4),
            black_box(0.002),
        ));
    });
    let pipeline_allocations = allocation_counter::measure(|| {
        black_box(advance_controls(
            black_box(&mut system_state),
            black_box(&config),
            black_box(&input),
            black_box(0.002),
        ));
    });

    assert_eq!(
        shaping_allocations.count_total, 0,
        "{shaping_allocations:?}"
    );
    assert_eq!(mixer_allocations.count_total, 0, "{mixer_allocations:?}");
    assert_eq!(servo_allocations.count_total, 0, "{servo_allocations:?}");
    assert_eq!(
        pipeline_allocations.count_total, 0,
        "{pipeline_allocations:?}"
    );
}
