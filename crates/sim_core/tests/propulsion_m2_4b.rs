use sim_core::{
    AeroEnvironment, BatteryConfig, ElectricPropulsionConfig, EscConfig, EscConfigError,
    MotorConfig, PropellerCoefficientMap, PropellerCoefficientMapError, PropellerCoefficientNode,
    PropellerCoefficientSource, PropellerCoefficientTable, PropellerConfig, PropellerSample,
    PropellerSpinDirection, RigidBodyState, ShaftSpeedRangeStatus, evaluate_electric_propulsion,
    evaluate_electric_propulsion_with_source, evaluate_electrical_drive,
    evaluate_electrical_drive_with_esc, solve_quasi_static_shaft_speed,
    solve_quasi_static_shaft_speed_with_source,
};
use sim_math::{Orientation, Vec3};
use std::hint::black_box;

fn table(ct0: f64, cq0: f64, end_j: f64, ct1: f64, cq1: f64) -> PropellerCoefficientTable {
    PropellerCoefficientTable::new(vec![
        PropellerSample {
            advance_ratio_j: 0.0,
            ct: ct0,
            cq: cq0,
        },
        PropellerSample {
            advance_ratio_j: end_j,
            ct: ct1,
            cq: cq1,
        },
    ])
    .unwrap()
}

fn map() -> PropellerCoefficientMap {
    PropellerCoefficientMap::new(vec![
        PropellerCoefficientNode::new(200.0, table(0.10, 0.010, 0.5, 0.06, 0.007)).unwrap(),
        PropellerCoefficientNode::new(600.0, table(0.14, 0.020, 1.0, 0.04, 0.006)).unwrap(),
    ])
    .unwrap()
}

fn config(esc_ohm: f64) -> ElectricPropulsionConfig {
    ElectricPropulsionConfig::new_with_esc(
        BatteryConfig::new(12.4, 0.03).unwrap(),
        EscConfig::new(esc_ohm).unwrap(),
        MotorConfig::new(850.0, 0.06, 0.8).unwrap(),
        PropellerConfig::new(
            Vec3::zeros(),
            Orientation::identity(),
            0.29,
            PropellerSpinDirection::PositiveAboutLocalX,
        )
        .unwrap(),
    )
}

fn state(speed_mps: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(speed_mps, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[test]
fn esc_rejects_invalid_resistance_and_zero_is_ideal() {
    for value in [-0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            EscConfig::new(value),
            Err(EscConfigError::InvalidSeriesResistance)
        );
    }
    let battery = BatteryConfig::new(12.0, 0.04).unwrap();
    let motor = MotorConfig::new(900.0, 0.07, 1.0).unwrap();
    assert_eq!(
        evaluate_electrical_drive(0.63, 321.0, &battery, &motor),
        evaluate_electrical_drive_with_esc(0.63, 321.0, &battery, &EscConfig::ideal(), &motor,)
    );
}

#[test]
fn positive_esc_resistance_reports_loss_and_closes_terminal_power_accounting() {
    let lossy_config = config(0.015);
    let output = evaluate_electrical_drive_with_esc(
        0.72,
        300.0,
        lossy_config.battery(),
        lossy_config.esc(),
        lossy_config.motor(),
    );
    assert!(output.esc_loss_power_w > 0.0);
    let accounted = output.motor_electrical_input_power_w + output.esc_loss_power_w;
    assert!(
        (output.battery_terminal_electrical_power_w - accounted).abs()
            <= 2.0 * f64::EPSILON * accounted.abs()
    );

    let source = PropellerCoefficientSource::FixedTable(table(0.12, 0.018, 1.0, 0.02, 0.005));
    let environment = AeroEnvironment::new(1.225, Vec3::zeros()).unwrap();
    let ideal = evaluate_electric_propulsion_with_source(
        &state(0.0),
        0.8,
        &config(0.0),
        &environment,
        &source,
    );
    let lossy = evaluate_electric_propulsion_with_source(
        &state(0.0),
        0.8,
        &config(0.015),
        &environment,
        &source,
    );
    assert!(lossy.shaft_speed_rad_s < ideal.shaft_speed_rad_s);
    assert!(lossy.thrust_n < ideal.thrust_n);
    for value in [
        lossy.battery_terminal_voltage_v,
        lossy.battery_current_a,
        lossy.battery_terminal_electrical_power_w,
        lossy.esc_loss_power_w,
        lossy.motor_voltage_v,
        lossy.motor_current_a,
        lossy.motor_electrical_input_power_w,
        lossy.shaft_speed_rad_s,
        lossy.thrust_n,
    ] {
        assert!(value.is_finite());
    }
}

#[test]
fn map_rejects_empty_invalid_nonincreasing_and_duplicate_speeds() {
    assert_eq!(
        PropellerCoefficientMap::new(Vec::new()),
        Err(PropellerCoefficientMapError::Empty)
    );
    for (speed, expected) in [
        (0.0, PropellerCoefficientMapError::NonPositiveShaftSpeed),
        (-1.0, PropellerCoefficientMapError::NonPositiveShaftSpeed),
        (f64::NAN, PropellerCoefficientMapError::NonFiniteShaftSpeed),
        (
            f64::INFINITY,
            PropellerCoefficientMapError::NonFiniteShaftSpeed,
        ),
    ] {
        assert_eq!(
            PropellerCoefficientNode::new(speed, table(0.1, 0.01, 1.0, 0.0, 0.0)),
            Err(expected)
        );
    }
    for speeds in [[400.0, 200.0], [400.0, 400.0]] {
        let nodes = speeds
            .map(|speed| {
                PropellerCoefficientNode::new(speed, table(0.1, 0.01, 1.0, 0.0, 0.0)).unwrap()
            })
            .to_vec();
        assert_eq!(
            PropellerCoefficientMap::new(nodes),
            Err(PropellerCoefficientMapError::NonIncreasingShaftSpeed { index: 1 })
        );
    }
}

#[test]
fn map_samples_exact_interpolated_different_j_grids_and_clamps_with_diagnostics() {
    let map = map();
    let exact = map.sample(200.0, 0.25);
    assert_eq!(exact.coefficients.ct, 0.08);
    assert_eq!(exact.range_status, ShaftSpeedRangeStatus::ExactOrInRange);

    let interpolated = map.sample(400.0, 0.25);
    assert_eq!(interpolated.lower_shaft_speed_rad_s, 200.0);
    assert_eq!(interpolated.upper_shaft_speed_rad_s, 600.0);
    assert_eq!(interpolated.interpolation_fraction, 0.5);
    assert_eq!(interpolated.coefficients.ct, 0.0975);
    assert_eq!(interpolated.coefficients.cq, 0.0125);

    let below = map.sample(0.0, 0.25);
    assert_eq!(below.coefficients, exact.coefficients);
    assert_eq!(below.range_status, ShaftSpeedRangeStatus::BelowRange);
    let above = map.sample(900.0, 0.25);
    assert!((above.coefficients.ct - 0.115).abs() <= f64::EPSILON);
    assert_eq!(above.range_status, ShaftSpeedRangeStatus::AboveRange);

    let one = PropellerCoefficientMap::new(vec![
        PropellerCoefficientNode::new(500.0, table(0.1, 0.01, 1.0, 0.0, 0.0)).unwrap(),
    ])
    .unwrap();
    assert_eq!(
        one.sample(0.0, 0.4),
        one.sample(50_000.0, 0.4),
        "one node is speed-independent"
    );
}

#[test]
fn bisection_uses_candidate_speed_map_and_stopped_outputs_are_exact_zero() {
    let config = config(0.01);
    let source = PropellerCoefficientSource::ShaftSpeedMap(map());
    let mapped = solve_quasi_static_shaft_speed_with_source(0.8, 8.0, 1.225, &config, &source);
    let frozen = solve_quasi_static_shaft_speed(0.8, 8.0, 1.225, &config, map().nodes()[0].table());
    assert_ne!(mapped.to_bits(), frozen.to_bits());
    let output = evaluate_electric_propulsion_with_source(
        &state(8.0),
        0.8,
        &config,
        &AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        &source,
    );
    assert_eq!(output.shaft_speed_rad_s.to_bits(), mapped.to_bits());
    assert_eq!(
        output.coefficient_map_sample,
        map().sample(mapped, output.advance_ratio_j)
    );

    let stopped = evaluate_electric_propulsion_with_source(
        &state(12.0),
        0.0,
        &config,
        &AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        &source,
    );
    assert_eq!(stopped.shaft_speed_rad_s, 0.0);
    assert_eq!(stopped.thrust_n, 0.0);
    assert_eq!(stopped.propeller_load_torque_nm, 0.0);
    assert!(stopped.coefficients.ct.is_finite());
}

#[test]
fn fixed_source_legacy_path_is_bit_identical_and_map_hot_path_allocates_nothing() {
    let legacy_config = ElectricPropulsionConfig::new(
        *config(0.0).battery(),
        *config(0.0).motor(),
        *config(0.0).propeller(),
    );
    let table = table(0.12, 0.018, 1.0, 0.02, 0.005);
    let source = PropellerCoefficientSource::FixedTable(table.clone());
    let environment = AeroEnvironment::new(1.225, Vec3::zeros()).unwrap();
    let state = state(7.0);
    let legacy = evaluate_electric_propulsion(&state, 0.71, &legacy_config, &environment, &table);
    let explicit = evaluate_electric_propulsion_with_source(
        &state,
        0.71,
        &legacy_config,
        &environment,
        &source,
    );
    assert_eq!(legacy, explicit);

    let map_source = PropellerCoefficientSource::ShaftSpeedMap(map());
    let first = evaluate_electric_propulsion_with_source(
        &state,
        0.71,
        &config(0.01),
        &environment,
        &map_source,
    );
    for _ in 0..50 {
        assert_eq!(
            first,
            evaluate_electric_propulsion_with_source(
                &state,
                0.71,
                &config(0.01),
                &environment,
                &map_source,
            )
        );
    }
    let runtime_config = config(0.01);
    let allocations = allocation_counter::measure(|| {
        black_box(evaluate_electric_propulsion_with_source(
            black_box(&state),
            black_box(0.71),
            black_box(&runtime_config),
            black_box(&environment),
            black_box(&map_source),
        ));
    });
    assert_eq!(allocations.count_total, 0, "{allocations:?}");
}
