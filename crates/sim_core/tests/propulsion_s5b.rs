use sim_core::{
    AeroEnvironment, AxisResponseConfig, BatteryConfig, BatteryConfigError, BodyWrench,
    ControlActuatorConfig, ControlResponseConfig, ControlSystemConfig, ControlSystemState,
    ElectricPropulsionConfig, MotorConfig, MotorConfigError, PROPULSION_BISECTION_ITERATIONS,
    PilotInput, PropellerCoefficientError, PropellerCoefficientTable, PropellerCoefficients,
    PropellerConfig, PropellerConfigError, PropellerSample, PropellerSpinDirection,
    PropulsionOutput, RigidBodyParams, RigidBodyState, Rk4Integrator, ServoConfig,
    advance_controls, evaluate_derivative, evaluate_electric_propulsion, evaluate_electrical_drive,
    solve_quasi_static_shaft_speed,
};
use sim_math::{Mat3, Orientation, Quaternion, Vec3};
use std::{
    f64::consts::{FRAC_PI_2, TAU},
    hint::black_box,
};

const ALGEBRA_EPSILON_FACTOR: f64 = 128.0;

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual:.17e}, expected={expected:.17e}, tolerance={tolerance:.17e}"
    );
}

fn scaled_roundoff_tolerance(reference: f64) -> f64 {
    ALGEBRA_EPSILON_FACTOR * f64::EPSILON * reference.abs().max(1.0)
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance:.17e}"
    );
}

fn state_with_velocity(linear_velocity_world_mps: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps,
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn battery() -> BatteryConfig {
    BatteryConfig::new(12.0, 0.05).unwrap()
}

fn motor() -> MotorConfig {
    MotorConfig::new(1_000.0, 0.1, 1.0).unwrap()
}

fn propeller_at(
    position_body_m: Vec3,
    orientation_body_from_prop: Orientation,
    spin_direction: PropellerSpinDirection,
) -> PropellerConfig {
    PropellerConfig::new(
        position_body_m,
        orientation_body_from_prop,
        0.254,
        spin_direction,
    )
    .unwrap()
}

fn config_with_propeller(propeller: PropellerConfig) -> ElectricPropulsionConfig {
    ElectricPropulsionConfig::new(battery(), motor(), propeller)
}

fn identity_config(spin_direction: PropellerSpinDirection) -> ElectricPropulsionConfig {
    config_with_propeller(propeller_at(
        Vec3::zeros(),
        Orientation::identity(),
        spin_direction,
    ))
}

fn environment(density_kg_m3: f64) -> AeroEnvironment {
    AeroEnvironment::new(density_kg_m3, Vec3::zeros()).unwrap()
}

fn constant_table(ct: f64, cq: f64) -> PropellerCoefficientTable {
    PropellerCoefficientTable::new(vec![
        PropellerSample {
            advance_ratio_j: -10.0,
            ct,
            cq,
        },
        PropellerSample {
            advance_ratio_j: 10.0,
            ct,
            cq,
        },
    ])
    .unwrap()
}

fn varying_table() -> PropellerCoefficientTable {
    PropellerCoefficientTable::new(vec![
        PropellerSample {
            advance_ratio_j: -1.0,
            ct: 0.16,
            cq: 0.035,
        },
        PropellerSample {
            advance_ratio_j: 0.0,
            ct: 0.15,
            cq: 0.03,
        },
        PropellerSample {
            advance_ratio_j: 0.5,
            ct: 0.08,
            cq: 0.015,
        },
        PropellerSample {
            advance_ratio_j: 1.0,
            ct: 0.02,
            cq: 0.005,
        },
    ])
    .unwrap()
}

fn evaluate_identity(
    velocity_body_mps: Vec3,
    throttle: f64,
    config: &ElectricPropulsionConfig,
    density_kg_m3: f64,
    table: &PropellerCoefficientTable,
) -> PropulsionOutput {
    evaluate_electric_propulsion(
        &state_with_velocity(velocity_body_mps),
        throttle,
        config,
        &environment(density_kg_m3),
        table,
    )
}

fn assert_output_finite(output: &PropulsionOutput) {
    assert!(output.throttle.is_finite());
    assert!(
        output
            .air_relative_velocity_prop_mps
            .iter()
            .all(|value| value.is_finite())
    );
    for scalar in [
        output.axial_airspeed_mps,
        output.battery_terminal_voltage_v,
        output.battery_current_a,
        output.motor_voltage_v,
        output.motor_current_a,
        output.shaft_speed_rad_s,
        output.shaft_speed_rpm,
        output.motor_torque_nm,
        output.advance_ratio_j,
        output.coefficients.ct,
        output.coefficients.cq,
        output.propeller_load_torque_nm,
        output.thrust_n,
    ] {
        assert!(scalar.is_finite(), "non-finite propulsion scalar: {scalar}");
    }
    assert!(output.force_prop_n.iter().all(|value| value.is_finite()));
    assert!(
        output
            .wrench_body
            .force_body_n
            .iter()
            .all(|value| value.is_finite())
    );
    assert!(
        output
            .wrench_body
            .moment_body_nm
            .iter()
            .all(|value| value.is_finite())
    );
}

fn scalar_bits(output: &PropulsionOutput) -> [u64; 13] {
    [
        output.throttle.to_bits(),
        output.axial_airspeed_mps.to_bits(),
        output.battery_terminal_voltage_v.to_bits(),
        output.battery_current_a.to_bits(),
        output.motor_voltage_v.to_bits(),
        output.motor_current_a.to_bits(),
        output.shaft_speed_rad_s.to_bits(),
        output.shaft_speed_rpm.to_bits(),
        output.motor_torque_nm.to_bits(),
        output.advance_ratio_j.to_bits(),
        output.coefficients.ct.to_bits(),
        output.coefficients.cq.to_bits(),
        output.propeller_load_torque_nm.to_bits(),
    ]
}

fn vector_bits(vector: &Vec3) -> [u64; 3] {
    [vector.x.to_bits(), vector.y.to_bits(), vector.z.to_bits()]
}

fn assert_output_bit_identical(left: &PropulsionOutput, right: &PropulsionOutput) {
    assert_eq!(scalar_bits(left), scalar_bits(right));
    assert_eq!(left.thrust_n.to_bits(), right.thrust_n.to_bits());
    assert_eq!(
        vector_bits(&left.air_relative_velocity_prop_mps),
        vector_bits(&right.air_relative_velocity_prop_mps)
    );
    assert_eq!(
        vector_bits(&left.force_prop_n),
        vector_bits(&right.force_prop_n)
    );
    assert_eq!(
        vector_bits(&left.wrench_body.force_body_n),
        vector_bits(&right.wrench_body.force_body_n)
    );
    assert_eq!(
        vector_bits(&left.wrench_body.moment_body_nm),
        vector_bits(&right.wrench_body.moment_body_nm)
    );
}

#[test]
fn p1_invalid_battery_config_is_rejected() {
    for invalid_voltage in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            BatteryConfig::new(invalid_voltage, 0.05),
            Err(BatteryConfigError::InvalidOpenCircuitVoltage)
        );
    }
    for invalid_resistance in [-0.01, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            BatteryConfig::new(12.0, invalid_resistance),
            Err(BatteryConfigError::InvalidInternalResistance)
        );
    }
    assert!(BatteryConfig::new(12.0, 0.0).is_ok());
}

#[test]
fn p2_invalid_motor_config_is_rejected() {
    for invalid_kv in [
        0.0,
        -1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::MAX,
    ] {
        assert_eq!(
            MotorConfig::new(invalid_kv, 0.1, 1.0),
            Err(MotorConfigError::InvalidKv)
        );
    }
    for invalid_resistance in [0.0, -0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            MotorConfig::new(1_000.0, invalid_resistance, 1.0),
            Err(MotorConfigError::InvalidWindingResistance)
        );
    }
    for invalid_no_load_current in [-0.1, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            MotorConfig::new(1_000.0, 0.1, invalid_no_load_current),
            Err(MotorConfigError::InvalidNoLoadCurrent)
        );
    }
}

#[test]
fn p3_invalid_propeller_config_is_rejected() {
    let valid_spin = PropellerSpinDirection::PositiveAboutLocalX;
    assert_eq!(
        PropellerConfig::new(
            Vec3::new(f64::NAN, 0.0, 0.0),
            Orientation::identity(),
            0.25,
            valid_spin,
        ),
        Err(PropellerConfigError::NonFinitePosition)
    );
    assert_eq!(
        PropellerConfig::new(
            Vec3::new(0.0, f64::INFINITY, 0.0),
            Orientation::identity(),
            0.25,
            valid_spin,
        ),
        Err(PropellerConfigError::NonFinitePosition)
    );
    for invalid_orientation in [
        Orientation::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 0.0)),
        Orientation::new_unchecked(Quaternion::new(2.0, 0.0, 0.0, 0.0)),
    ] {
        assert_eq!(
            PropellerConfig::new(Vec3::zeros(), invalid_orientation, 0.25, valid_spin),
            Err(PropellerConfigError::InvalidOrientation)
        );
    }
    for invalid_diameter in [0.0, -0.25, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            PropellerConfig::new(
                Vec3::zeros(),
                Orientation::identity(),
                invalid_diameter,
                valid_spin,
            ),
            Err(PropellerConfigError::InvalidDiameter)
        );
    }
}

#[test]
fn p4_invalid_coefficient_table_is_rejected() {
    let sample = PropellerSample {
        advance_ratio_j: 0.0,
        ct: 0.1,
        cq: 0.02,
    };
    assert_eq!(
        PropellerCoefficientTable::new(Vec::new()),
        Err(PropellerCoefficientError::TooFewSamples)
    );
    assert_eq!(
        PropellerCoefficientTable::new(vec![sample]),
        Err(PropellerCoefficientError::TooFewSamples)
    );
    assert_eq!(
        PropellerCoefficientTable::new(vec![sample, sample]),
        Err(PropellerCoefficientError::NonIncreasingAdvanceRatio { index: 1 })
    );
    assert_eq!(
        PropellerCoefficientTable::new(vec![
            sample,
            PropellerSample {
                advance_ratio_j: -1.0,
                ..sample
            },
        ]),
        Err(PropellerCoefficientError::NonIncreasingAdvanceRatio { index: 1 })
    );
    for non_finite in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for invalid_sample in [
            PropellerSample {
                advance_ratio_j: non_finite,
                ..sample
            },
            PropellerSample {
                advance_ratio_j: 1.0,
                ct: non_finite,
                ..sample
            },
            PropellerSample {
                advance_ratio_j: 1.0,
                cq: non_finite,
                ..sample
            },
        ] {
            assert_eq!(
                PropellerCoefficientTable::new(vec![sample, invalid_sample]),
                Err(PropellerCoefficientError::NonFiniteSample { index: 1 })
            );
        }
    }
    assert_eq!(
        PropellerCoefficientTable::new(vec![
            sample,
            PropellerSample {
                advance_ratio_j: 1.0,
                ct: -0.01,
                ..sample
            },
        ]),
        Err(PropellerCoefficientError::NegativeThrustCoefficient { index: 1 })
    );
    assert_eq!(
        PropellerCoefficientTable::new(vec![
            sample,
            PropellerSample {
                advance_ratio_j: 1.0,
                cq: -0.01,
                ..sample
            },
        ]),
        Err(PropellerCoefficientError::NegativeTorqueCoefficient { index: 1 })
    );
}

fn interpolation_table() -> PropellerCoefficientTable {
    PropellerCoefficientTable::new(vec![
        PropellerSample {
            advance_ratio_j: 0.0,
            ct: 0.125,
            cq: 0.03125,
        },
        PropellerSample {
            advance_ratio_j: 1.0,
            ct: 0.375,
            cq: 0.0625,
        },
        PropellerSample {
            advance_ratio_j: 2.0,
            ct: 0.625,
            cq: 0.09375,
        },
    ])
    .unwrap()
}

#[test]
fn p5_exact_coefficient_sample_is_preserved() {
    assert_eq!(
        interpolation_table().sample_clamped(1.0),
        PropellerCoefficients {
            ct: 0.375,
            cq: 0.0625,
        }
    );
}

#[test]
fn p6_midpoint_coefficients_are_linearly_interpolated() {
    assert_eq!(
        interpolation_table().sample_clamped(0.5),
        PropellerCoefficients {
            ct: 0.25,
            cq: 0.046875,
        }
    );
}

#[test]
fn p7_coefficient_sampling_clamps_to_endpoints() {
    let table = interpolation_table();
    assert_eq!(
        table.sample_clamped(-100.0),
        PropellerCoefficients {
            ct: 0.125,
            cq: 0.03125,
        }
    );
    assert_eq!(
        table.sample_clamped(100.0),
        PropellerCoefficients {
            ct: 0.625,
            cq: 0.09375,
        }
    );
}

#[test]
fn p8_zero_throttle_is_exactly_off_at_any_airspeed() {
    let config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let table = varying_table();
    for velocity in [
        Vec3::new(-40.0, 3.0, -2.0),
        Vec3::zeros(),
        Vec3::new(80.0, -5.0, 4.0),
    ] {
        let output = evaluate_identity(velocity, 0.0, &config, 1.225, &table);
        assert_eq!(output.motor_current_a, 0.0);
        assert_eq!(output.battery_current_a, 0.0);
        assert_eq!(output.shaft_speed_rad_s, 0.0);
        assert_eq!(output.shaft_speed_rpm, 0.0);
        assert_eq!(output.motor_torque_nm, 0.0);
        assert_eq!(output.propeller_load_torque_nm, 0.0);
        assert_eq!(output.thrust_n, 0.0);
        assert_eq!(output.force_prop_n, Vec3::zeros());
        assert_eq!(output.wrench_body, BodyWrench::zero());
    }
}

#[test]
fn p9_electrical_current_and_battery_sag_match_analytical_equations() {
    let battery = BatteryConfig::new(12.0, 0.2).unwrap();
    let kv_rpm_per_v = 600.0;
    let motor = MotorConfig::new(kv_rpm_per_v, 0.1, 0.5).unwrap();
    let throttle = 0.5;
    let shaft_speed_rad_s = 200.0;
    let ke = 60.0 / (kv_rpm_per_v * TAU);
    let denominator = 0.1 + throttle * throttle * 0.2;
    let expected_motor_current =
        ((throttle * 12.0 - ke * shaft_speed_rad_s) / denominator).max(0.0);
    let expected_battery_current = throttle * expected_motor_current;
    let expected_battery_voltage = 12.0 - expected_battery_current * 0.2;
    let expected_motor_voltage = throttle * expected_battery_voltage;
    let output = evaluate_electrical_drive(throttle, shaft_speed_rad_s, &battery, &motor);

    assert_close(
        output.motor_current_a,
        expected_motor_current,
        scaled_roundoff_tolerance(expected_motor_current),
    );
    assert_close(
        output.battery_current_a,
        expected_battery_current,
        scaled_roundoff_tolerance(expected_battery_current),
    );
    assert_close(
        output.battery_terminal_voltage_v,
        expected_battery_voltage,
        scaled_roundoff_tolerance(expected_battery_voltage),
    );
    assert_close(
        output.motor_voltage_v,
        expected_motor_voltage,
        scaled_roundoff_tolerance(expected_motor_voltage),
    );

    let above_back_emf_limit = throttle * 12.0 / ke + 1.0;
    let clamped = evaluate_electrical_drive(throttle, above_back_emf_limit, &battery, &motor);
    assert_eq!(clamped.motor_current_a, 0.0);
    assert_eq!(clamped.battery_current_a, 0.0);
    assert_eq!(clamped.motor_torque_nm, 0.0);
    assert_eq!(clamped.battery_terminal_voltage_v, 12.0);
    assert_eq!(clamped.motor_voltage_v, 6.0);
}

#[test]
fn p10_ideal_esc_conserves_electrical_power() {
    let battery = BatteryConfig::new(14.8, 0.08).unwrap();
    let motor = MotorConfig::new(920.0, 0.11, 0.8).unwrap();
    let output = evaluate_electrical_drive(0.63, 250.0, &battery, &motor);
    let battery_power_w = output.battery_terminal_voltage_v * output.battery_current_a;
    let motor_power_w = output.motor_voltage_v * output.motor_current_a;
    assert!(battery_power_w > 0.0);
    assert!(motor_power_w > 0.0);
    assert_close(
        battery_power_w,
        motor_power_w,
        scaled_roundoff_tolerance(battery_power_w),
    );
}

#[test]
fn p11_motor_kv_ke_and_kt_conversion_is_si_consistent() {
    let motor = MotorConfig::new(60.0, 0.1, 0.0).unwrap();
    assert_close(motor.kv_rad_s_per_v(), TAU, scaled_roundoff_tolerance(TAU));
    assert_close(
        motor.back_emf_constant_v_per_rad_s(),
        1.0 / TAU,
        scaled_roundoff_tolerance(1.0 / TAU),
    );
    assert_eq!(
        motor.torque_constant_nm_per_a().to_bits(),
        motor.back_emf_constant_v_per_rad_s().to_bits()
    );
}

#[test]
fn p12_static_solver_matches_independent_quadratic_solution() {
    let open_circuit_voltage_v = 12.0;
    let battery_resistance_ohm = 0.05;
    let kv_rpm_per_v = 1_000.0;
    let motor_resistance_ohm = 0.1;
    let diameter_m: f64 = 0.254;
    let cq = 0.02;
    let density_kg_m3 = 1.225;
    let throttle = 0.7;
    let config = ElectricPropulsionConfig::new(
        BatteryConfig::new(open_circuit_voltage_v, battery_resistance_ohm).unwrap(),
        MotorConfig::new(kv_rpm_per_v, motor_resistance_ohm, 0.0).unwrap(),
        PropellerConfig::new(
            Vec3::zeros(),
            Orientation::identity(),
            diameter_m,
            PropellerSpinDirection::PositiveAboutLocalX,
        )
        .unwrap(),
    );
    let table = constant_table(0.12, cq);

    // These constants are derived directly from the specification, not production helpers.
    let ke = 60.0 / (kv_rpm_per_v * TAU);
    let kt = ke;
    let electrical_denominator =
        motor_resistance_ohm + throttle * throttle * battery_resistance_ohm;
    let a = kt * throttle * open_circuit_voltage_v / electrical_denominator;
    let b = kt * ke / electrical_denominator;
    let c = cq * density_kg_m3 * diameter_m.powi(5) / TAU.powi(2);
    let expected_rad_s = (-b + b.mul_add(b, 4.0 * c * a).sqrt()) / (2.0 * c);
    let actual_rad_s =
        solve_quasi_static_shaft_speed(throttle, 0.0, density_kg_m3, &config, &table);
    let upper_rad_s = throttle * open_circuit_voltage_v / ke;
    let final_bracket_width = upper_rad_s * 2.0_f64.powi(-(PROPULSION_BISECTION_ITERATIONS as i32));
    let tolerance = 2.0 * final_bracket_width + 512.0 * f64::EPSILON * upper_rad_s.abs().max(1.0);
    let error_rad_s = (actual_rad_s - expected_rad_s).abs();
    println!("P12 analytical omega error: {error_rad_s:.17e} rad/s");
    assert_close(actual_rad_s, expected_rad_s, tolerance);
}

#[test]
fn p13_solver_result_satisfies_independently_computed_torque_balance() {
    let throttle = 0.73;
    let density_kg_m3 = 1.225;
    let cq = 0.021;
    let config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let table = constant_table(0.13, cq);
    let output = evaluate_identity(Vec3::zeros(), throttle, &config, density_kg_m3, &table);

    let battery = config.battery();
    let motor = config.motor();
    let diameter_m = config.propeller().diameter_m();
    let ke = 60.0 / (motor.kv_rpm_per_v() * TAU);
    let denominator =
        motor.winding_resistance_ohm() + throttle * throttle * battery.internal_resistance_ohm();
    let expected_current = ((throttle * battery.open_circuit_voltage_v()
        - ke * output.shaft_speed_rad_s)
        / denominator)
        .max(0.0);
    let expected_motor_torque = ke * (expected_current - motor.no_load_current_a()).max(0.0);
    let expected_propeller_torque =
        cq * density_kg_m3 * (output.shaft_speed_rad_s / TAU).powi(2) * diameter_m.powi(5);
    assert_close(
        output.motor_torque_nm,
        expected_motor_torque,
        scaled_roundoff_tolerance(expected_motor_torque),
    );
    assert_close(
        output.propeller_load_torque_nm,
        expected_propeller_torque,
        scaled_roundoff_tolerance(expected_propeller_torque),
    );

    let upper_rad_s = throttle * battery.open_circuit_voltage_v() / ke;
    let bracket_width = upper_rad_s * 2.0_f64.powi(-(PROPULSION_BISECTION_ITERATIONS as i32));
    let motor_slope = ke * ke / denominator;
    let propeller_quadratic = cq * density_kg_m3 * diameter_m.powi(5) / TAU.powi(2);
    let residual_slope = motor_slope + 2.0 * propeller_quadratic * output.shaft_speed_rad_s;
    let numerical_floor = 512.0
        * f64::EPSILON
        * output
            .motor_torque_nm
            .abs()
            .max(output.propeller_load_torque_nm.abs())
            .max(1.0);
    let residual_bound_nm = 2.0 * residual_slope * bracket_width + numerical_floor;
    assert_close(
        output.motor_torque_nm,
        output.propeller_load_torque_nm,
        residual_bound_nm,
    );
}

fn known_dimensional_load_output() -> PropulsionOutput {
    let target_revolutions_per_s = 10.0;
    let target_omega_rad_s = TAU * target_revolutions_per_s;
    let expected_load_torque_nm = 0.2;
    let kv_rpm_per_v = 600.0;
    let expected_ke = 60.0 / (kv_rpm_per_v * TAU);
    let motor_resistance_ohm = 0.1;
    let required_current_a = expected_load_torque_nm / expected_ke;
    let required_voltage_v =
        expected_ke * target_omega_rad_s + motor_resistance_ohm * required_current_a;
    let config = ElectricPropulsionConfig::new(
        BatteryConfig::new(required_voltage_v, 0.0).unwrap(),
        MotorConfig::new(kv_rpm_per_v, motor_resistance_ohm, 0.0).unwrap(),
        PropellerConfig::new(
            Vec3::zeros(),
            Orientation::identity(),
            0.5,
            PropellerSpinDirection::PositiveAboutLocalX,
        )
        .unwrap(),
    );
    evaluate_identity(
        Vec3::zeros(),
        1.0,
        &config,
        1.0,
        &constant_table(0.16, 0.064),
    )
}

#[test]
fn p14_known_static_thrust_matches_dimensional_formula() {
    // Independently chosen values give 0.16 * 1 * 10^2 * 0.5^4 = 1 N.
    let output = known_dimensional_load_output();
    assert_close(output.shaft_speed_rad_s / TAU, 10.0, 2.0e-12);
    assert_close(output.thrust_n, 1.0, 5.0e-13);
}

#[test]
fn p15_known_propeller_torque_matches_dimensional_formula() {
    // Independently chosen values give 0.064 * 1 * 10^2 * 0.5^5 = 0.2 Nm.
    let output = known_dimensional_load_output();
    assert_close(output.shaft_speed_rad_s / TAU, 10.0, 2.0e-12);
    assert_close(output.propeller_load_torque_nm, 0.2, 2.0e-13);
}

#[test]
fn p16_forward_airspeed_changes_advance_ratio_and_operating_point() {
    let config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let table = varying_table();
    let static_output = evaluate_identity(Vec3::zeros(), 0.7, &config, 1.225, &table);
    let forward_output = evaluate_identity(Vec3::new(5.0, 0.0, 0.0), 0.7, &config, 1.225, &table);

    assert_eq!(static_output.advance_ratio_j, 0.0);
    assert!((0.0..1.0).contains(&forward_output.advance_ratio_j));
    assert!(forward_output.coefficients.ct < static_output.coefficients.ct);
    assert!(forward_output.coefficients.cq < static_output.coefficients.cq);
    assert!(forward_output.shaft_speed_rad_s > static_output.shaft_speed_rad_s);
    assert!(forward_output.thrust_n < static_output.thrust_n);
}

#[test]
fn p17_local_propeller_velocity_includes_omega_cross_r() {
    let propeller = propeller_at(
        Vec3::new(1.0, 0.0, 0.0),
        Orientation::identity(),
        PropellerSpinDirection::PositiveAboutLocalX,
    );
    let config = config_with_propeller(propeller);
    let mut state = state_with_velocity(Vec3::new(1.0, 2.0, 3.0));
    state.angular_velocity_body_radps = Vec3::new(0.0, 0.0, 2.0);
    let output = evaluate_electric_propulsion(
        &state,
        0.5,
        &config,
        &environment(1.225),
        &constant_table(0.1, 0.02),
    );
    assert_vec_close(
        output.air_relative_velocity_prop_mps,
        Vec3::new(1.0, 4.0, 3.0),
        scaled_roundoff_tolerance(4.0),
    );
    assert_eq!(output.axial_airspeed_mps, 1.0);
}

#[test]
fn p18_non_identity_aircraft_and_propeller_frames_transform_explicitly() {
    let orientation_world_from_body = Orientation::from_axis_angle(&Vec3::z_axis(), FRAC_PI_2);
    let orientation_body_from_prop = Orientation::from_axis_angle(&Vec3::x_axis(), FRAC_PI_2);
    let config = config_with_propeller(propeller_at(
        Vec3::zeros(),
        orientation_body_from_prop,
        PropellerSpinDirection::PositiveAboutLocalX,
    ));
    let mut state = state_with_velocity(Vec3::new(-2.0, 10.0, 3.0));
    state.orientation_world_from_body = orientation_world_from_body;
    let output = evaluate_electric_propulsion(
        &state,
        0.5,
        &config,
        &environment(1.225),
        &constant_table(0.1, 0.02),
    );

    // +90 deg body yaw maps body [10,2,3] to world [-2,10,3].
    // +90 deg prop roll maps prop [10,3,-2] to body [10,2,3].
    assert_vec_close(
        output.air_relative_velocity_prop_mps,
        Vec3::new(10.0, 3.0, -2.0),
        2.0e-14,
    );
    assert_close(output.axial_airspeed_mps, 10.0, 2.0e-14);
}

#[test]
fn p19_identity_propeller_thrust_points_along_positive_body_x() {
    let output = evaluate_identity(
        Vec3::zeros(),
        0.7,
        &identity_config(PropellerSpinDirection::PositiveAboutLocalX),
        1.225,
        &constant_table(0.1, 0.02),
    );
    assert!(output.thrust_n > 0.0);
    assert_eq!(output.force_prop_n, Vec3::new(output.thrust_n, 0.0, 0.0));
    assert_eq!(
        output.wrench_body.force_body_n,
        Vec3::new(output.thrust_n, 0.0, 0.0)
    );
}

#[test]
fn p20_positive_local_x_spin_reacts_about_negative_x() {
    let output = evaluate_identity(
        Vec3::zeros(),
        0.7,
        &identity_config(PropellerSpinDirection::PositiveAboutLocalX),
        1.225,
        &constant_table(0.1, 0.02),
    );
    assert!(output.propeller_load_torque_nm > 0.0);
    assert_eq!(
        output.wrench_body.moment_body_nm,
        Vec3::new(-output.propeller_load_torque_nm, 0.0, 0.0)
    );
}

#[test]
fn p21_negative_local_x_spin_reacts_about_positive_x() {
    let output = evaluate_identity(
        Vec3::zeros(),
        0.7,
        &identity_config(PropellerSpinDirection::NegativeAboutLocalX),
        1.225,
        &constant_table(0.1, 0.02),
    );
    assert!(output.propeller_load_torque_nm > 0.0);
    assert_eq!(
        output.wrench_body.moment_body_nm,
        Vec3::new(output.propeller_load_torque_nm, 0.0, 0.0)
    );
}

#[test]
fn p22_off_center_thrust_adds_exact_r_cross_f_moment() {
    let config = config_with_propeller(propeller_at(
        Vec3::new(0.0, 1.0, 0.0),
        Orientation::identity(),
        PropellerSpinDirection::PositiveAboutLocalX,
    ));
    let output = evaluate_identity(
        Vec3::zeros(),
        0.7,
        &config,
        1.225,
        &constant_table(0.1, 0.02),
    );
    assert_eq!(output.wrench_body.moment_body_nm.y, 0.0);
    assert_eq!(
        output.wrench_body.moment_body_nm.z.to_bits(),
        (-output.thrust_n).to_bits()
    );
    assert_eq!(
        output.wrench_body.moment_body_nm.x.to_bits(),
        (-output.propeller_load_torque_nm).to_bits()
    );
}

#[test]
fn p23_zero_density_has_zero_propeller_wrench_but_not_forced_zero_rpm() {
    let output = evaluate_identity(
        Vec3::new(30.0, 2.0, -1.0),
        0.8,
        &identity_config(PropellerSpinDirection::PositiveAboutLocalX),
        0.0,
        &varying_table(),
    );
    assert_eq!(output.thrust_n, 0.0);
    assert_eq!(output.propeller_load_torque_nm, 0.0);
    assert_eq!(output.force_prop_n, Vec3::zeros());
    assert_eq!(output.wrench_body, BodyWrench::zero());
    assert!(output.shaft_speed_rad_s > 0.0);
    assert_output_finite(&output);
}

#[test]
fn p24_rk4_recomputes_forward_speed_rpm_and_thrust_at_stage_states() {
    let initial = state_with_velocity(Vec3::new(2.0, 0.0, 0.0));
    let body_params = RigidBodyParams::new(0.2, Mat3::identity()).unwrap();
    let config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let environment = environment(1.225);
    let table = varying_table();
    let gravity = Vec3::zeros();
    let mut axial_speeds = [0.0; 4];
    let mut shaft_speeds = [0.0; 4];
    let mut thrusts = [0.0; 4];
    let mut calls = 0;

    let final_state = Rk4Integrator::step(&initial, 0.1, |stage_state| {
        let output = evaluate_electric_propulsion(stage_state, 0.7, &config, &environment, &table);
        axial_speeds[calls] = output.axial_airspeed_mps;
        shaft_speeds[calls] = output.shaft_speed_rad_s;
        thrusts[calls] = output.thrust_n;
        calls += 1;
        evaluate_derivative(stage_state, &body_params, &output.wrench_body, &gravity)
    });

    assert_eq!(calls, 4);
    assert!(
        axial_speeds.iter().copied().reduce(f64::max).unwrap()
            - axial_speeds.iter().copied().reduce(f64::min).unwrap()
            > 1.0e-6
    );
    assert!(
        shaft_speeds.iter().copied().reduce(f64::max).unwrap()
            - shaft_speeds.iter().copied().reduce(f64::min).unwrap()
            > 1.0e-6
    );
    assert!(
        thrusts.iter().copied().reduce(f64::max).unwrap()
            - thrusts.iter().copied().reduce(f64::min).unwrap()
            > 1.0e-8
    );
    assert!(final_state.linear_velocity_world_mps.x > initial.linear_velocity_world_mps.x);
}

#[test]
fn p25_representative_propulsion_grid_is_finite() {
    let config = config_with_propeller(propeller_at(
        Vec3::new(0.3, 0.2, -0.1),
        Orientation::from_scaled_axis(Vec3::new(0.1, -0.2, 0.05)),
        PropellerSpinDirection::NegativeAboutLocalX,
    ));
    let table = varying_table();
    for throttle in [0.0, 1.0e-12, 0.1, 0.5, 1.0] {
        for airspeed_mps in [-80.0, -1.0e-12, 0.0, 15.0, 100.0] {
            for density_kg_m3 in [0.0, 1.0e-12, 1.225, 5.0] {
                for body_rate_rad_s in [
                    Vec3::new(-20.0, 5.0, 2.0),
                    Vec3::zeros(),
                    Vec3::new(7.0, -3.0, 11.0),
                ] {
                    let mut state = state_with_velocity(Vec3::new(
                        airspeed_mps,
                        0.2 * airspeed_mps,
                        -0.1 * airspeed_mps,
                    ));
                    state.orientation_world_from_body =
                        Orientation::from_scaled_axis(Vec3::new(0.2, -0.1, 0.3));
                    state.angular_velocity_body_radps = body_rate_rad_s;
                    let environment =
                        AeroEnvironment::new(density_kg_m3, Vec3::new(2.0, -1.0, 0.5)).unwrap();
                    let output = evaluate_electric_propulsion(
                        &state,
                        throttle,
                        &config,
                        &environment,
                        &table,
                    );
                    assert_output_finite(&output);
                }
            }
        }
    }
}

#[test]
fn p26_repeated_evaluation_is_bit_identical() {
    let config = config_with_propeller(propeller_at(
        Vec3::new(0.2, -0.3, 0.1),
        Orientation::from_scaled_axis(Vec3::new(0.1, 0.05, -0.2)),
        PropellerSpinDirection::NegativeAboutLocalX,
    ));
    let environment = AeroEnvironment::new(1.17, Vec3::new(1.0, -2.0, 0.5)).unwrap();
    let table = varying_table();
    let mut state = state_with_velocity(Vec3::new(17.0, -3.0, 2.0));
    state.orientation_world_from_body = Orientation::from_scaled_axis(Vec3::new(-0.2, 0.3, 0.1));
    state.angular_velocity_body_radps = Vec3::new(0.4, -0.7, 0.2);
    let first = evaluate_electric_propulsion(&state, 0.713, &config, &environment, &table);
    for _ in 0..1_000 {
        let repeated = evaluate_electric_propulsion(&state, 0.713, &config, &environment, &table);
        assert_output_bit_identical(&first, &repeated);
    }
}

#[test]
fn p27_shaft_power_does_not_exceed_motor_electrical_input() {
    let output = evaluate_identity(
        Vec3::new(5.0, 0.0, 0.0),
        0.8,
        &identity_config(PropellerSpinDirection::PositiveAboutLocalX),
        1.225,
        &varying_table(),
    );
    let shaft_power_w = output.motor_torque_nm * output.shaft_speed_rad_s;
    let motor_electrical_power_w = output.motor_voltage_v * output.motor_current_a;
    assert!(shaft_power_w > 0.0);
    assert!(motor_electrical_power_w > 0.0);
    let tolerance = scaled_roundoff_tolerance(motor_electrical_power_w);
    assert!(
        shaft_power_w <= motor_electrical_power_w + tolerance,
        "shaft={shaft_power_w:.17e} W, electrical={motor_electrical_power_w:.17e} W"
    );
}

fn control_system_config() -> ControlSystemConfig {
    let axis = AxisResponseConfig::new(1.0, 0.0).unwrap();
    let response = ControlResponseConfig::new(axis, axis, axis);
    let servo = ServoConfig::new(-0.5, 0.0, 0.5, 2.0, false).unwrap();
    ControlSystemConfig::new(response, ControlActuatorConfig::new(servo, servo, servo))
}

#[test]
fn p28_s5a_throttle_passes_directly_to_propulsion_without_conversion() {
    let controls_config = control_system_config();
    let mut controls_state = ControlSystemState::neutral(&controls_config);
    let positions = advance_controls(
        &mut controls_state,
        &controls_config,
        &PilotInput::new(0.2, -0.3, 0.4, 0.63),
        0.002,
    );
    let propulsion_config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let environment = environment(1.225);
    let table = varying_table();
    let state = state_with_velocity(Vec3::new(5.0, 0.0, 0.0));
    let direct = evaluate_electric_propulsion(
        &state,
        positions.throttle(),
        &propulsion_config,
        &environment,
        &table,
    );
    let literal =
        evaluate_electric_propulsion(&state, 0.63, &propulsion_config, &environment, &table);
    assert_eq!(positions.throttle().to_bits(), 0.63_f64.to_bits());
    assert_eq!(direct.throttle.to_bits(), positions.throttle().to_bits());
    assert_output_bit_identical(&direct, &literal);
}

#[test]
fn propulsion_hot_paths_allocate_nothing_after_initialization() {
    let config = identity_config(PropellerSpinDirection::PositiveAboutLocalX);
    let table = varying_table();
    let environment = environment(1.225);
    let mut state = state_with_velocity(Vec3::new(12.0, 1.0, -0.5));
    let body_params = RigidBodyParams::new(1.5, Mat3::identity()).unwrap();
    let gravity = Vec3::zeros();

    let lookup_allocations = allocation_counter::measure(|| {
        black_box(table.sample_clamped(black_box(0.37)));
    });
    let electrical_allocations = allocation_counter::measure(|| {
        black_box(evaluate_electrical_drive(
            black_box(0.7),
            black_box(300.0),
            black_box(config.battery()),
            black_box(config.motor()),
        ));
    });
    let solver_allocations = allocation_counter::measure(|| {
        black_box(solve_quasi_static_shaft_speed(
            black_box(0.7),
            black_box(12.0),
            black_box(1.225),
            black_box(&config),
            black_box(&table),
        ));
    });
    let evaluator_allocations = allocation_counter::measure(|| {
        black_box(evaluate_electric_propulsion(
            black_box(&state),
            black_box(0.7),
            black_box(&config),
            black_box(&environment),
            black_box(&table),
        ));
    });
    let rk4_allocations = allocation_counter::measure(|| {
        state = Rk4Integrator::step(black_box(&state), black_box(0.002), |stage_state| {
            let propulsion = evaluate_electric_propulsion(
                black_box(stage_state),
                black_box(0.7),
                black_box(&config),
                black_box(&environment),
                black_box(&table),
            );
            evaluate_derivative(
                stage_state,
                black_box(&body_params),
                black_box(&propulsion.wrench_body),
                black_box(&gravity),
            )
        });
        black_box(state);
    });

    assert_eq!(lookup_allocations.count_total, 0, "{lookup_allocations:?}");
    assert_eq!(
        electrical_allocations.count_total, 0,
        "{electrical_allocations:?}"
    );
    assert_eq!(solver_allocations.count_total, 0, "{solver_allocations:?}");
    assert_eq!(
        evaluator_allocations.count_total, 0,
        "{evaluator_allocations:?}"
    );
    assert_eq!(rk4_allocations.count_total, 0, "{rk4_allocations:?}");
    assert!(state.validate().is_ok());
}
