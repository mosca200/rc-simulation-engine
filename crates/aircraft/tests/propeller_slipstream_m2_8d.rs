//! M2.8D deterministic propeller-slipstream runtime physics. Synthetic data only.

use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, LongitudinalTrimQualificationLimits,
    LongitudinalTrimQualificationOutcome, LongitudinalTrimRequest, LongitudinalTrimTolerances,
    LongitudinalTrimVariables, QualificationBlocker, TrimBounds,
    effective_aero_elements_for_positions, evaluate_aerodynamic_wrench_with_propulsion,
    evaluate_aircraft_instantaneous, evaluate_aircraft_section_kinematics,
    evaluate_aircraft_surface_aerodynamic_state, evaluate_aircraft_wrench, propeller_slipstream,
    qualify_longitudinal_trim_solution, solve_longitudinal_trim,
};
use model::{AircraftModel, AircraftModelLoader};
use serde_json::{Value, json};
use sim_core::{
    AeroEnvironment, PilotInput, RigidBodyState, assemble_aero_element_wrench,
    calculate_reynolds_number, compute_section_kinematics,
    evaluate_electric_propulsion_with_source, evaluate_steady_controls,
};
use sim_math::{Orientation, Vec3};
use std::f64::consts::PI;

const RHO: f64 = 1.225;
const SPEED_MPS: f64 = 8.0;
const ALPHA_RAD: f64 = 0.16;

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap()
}

fn load(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

fn state(speed: f64, alpha: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(speed * alpha.cos(), 0.0, speed * alpha.sin()),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn environment(rho: f64) -> AeroEnvironment {
    AeroEnvironment::new(rho, Vec3::zeros()).unwrap()
}

fn config(rho: f64) -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(0.002, Vec3::zeros(), environment(rho)).unwrap()
}

fn effective(model: &AircraftModel) -> Vec<sim_core::AeroElement> {
    model
        .aero_elements()
        .iter()
        .map(|element| *element.element())
        .collect()
}

fn propulsion(
    model: &AircraftModel,
    stage_state: &RigidBodyState,
    throttle: f64,
    environment: &AeroEnvironment,
) -> sim_core::PropulsionOutput {
    let runtime = model.propulsion().unwrap();
    evaluate_electric_propulsion_with_source(
        stage_state,
        throttle,
        runtime.config(),
        environment,
        runtime.coefficient_source(),
    )
}

fn assert_close(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "actual={actual:.17e}, expected={expected:.17e}, tolerance={tolerance:.3e}"
    );
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance:.3e}"
    );
}

#[test]
fn empty_v7_and_zero_factor_preserve_uncoupled_v6_physics_bit_exactly() {
    let mut v7_empty = fixture();
    v7_empty["propeller_slipstream_interactions"] = json!([]);
    let mut v6 = v7_empty.clone();
    v6["schema_version"] = json!(6);
    v6.as_object_mut()
        .unwrap()
        .remove("propeller_slipstream_interactions");
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let prior = load(&v6);
    let empty = load(&v7_empty);
    assert_eq!(
        evaluate_aircraft_wrench(&stage, &effective(&prior), &prior, 0.65, &env),
        evaluate_aircraft_wrench(&stage, &effective(&empty), &empty, 0.65, &env)
    );

    let mut zero = fixture();
    zero["propeller_slipstream_interactions"][0]["slipstream_velocity_factor"] = json!(0.0);
    let zero = load(&zero);
    assert_eq!(
        evaluate_aircraft_wrench(&stage, &effective(&empty), &empty, 0.65, &env),
        evaluate_aircraft_wrench(&stage, &effective(&zero), &zero, 0.65, &env)
    );
}

#[test]
fn induced_velocity_matches_momentum_theory_and_unsupported_domains_are_zero() {
    let model = load(&fixture());
    let stage = state(SPEED_MPS, 0.0);
    let env = environment(RHO);
    let output = propulsion(&model, &stage, 0.7, &env);
    assert!(output.thrust_n > 0.0);
    let disk_area = PI
        * model
            .propulsion()
            .unwrap()
            .config()
            .propeller()
            .diameter_m()
            .powi(2)
        / 4.0;
    let expected = 0.5
        * ((output.axial_airspeed_mps.powi(2) + 2.0 * output.thrust_n / (RHO * disk_area)).sqrt()
            - output.axial_airspeed_mps);
    let wake = propeller_slipstream(&model, &env, &output);
    assert!(wake.induced_velocity_mps() > 0.0);
    assert_close(wake.induced_velocity_mps(), expected, 2.0e-15);
    assert_eq!(wake.axis_body(), Vec3::new(1.0, 0.0, 0.0));

    let stopped = propulsion(&model, &stage, 0.0, &env);
    assert_eq!(stopped.thrust_n, 0.0);
    assert_eq!(
        propeller_slipstream(&model, &env, &stopped).induced_velocity_mps(),
        0.0
    );
    let mut reverse_thrust = output;
    reverse_thrust.thrust_n = -1.0;
    assert_eq!(
        propeller_slipstream(&model, &env, &reverse_thrust).induced_velocity_mps(),
        0.0
    );
    let vacuum = environment(0.0);
    let vacuum_output = propulsion(&model, &stage, 0.7, &vacuum);
    assert_eq!(vacuum_output.thrust_n, 0.0);
    assert_eq!(
        propeller_slipstream(&model, &vacuum, &vacuum_output).induced_velocity_mps(),
        0.0
    );
}

#[test]
fn target_speed_dynamic_pressure_and_reynolds_use_the_physical_wake_vector() {
    let model = load(&fixture());
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    let output = propulsion(&model, &stage, 0.7, &env);
    let wake = propeller_slipstream(&model, &env, &output);

    let target =
        evaluate_aircraft_section_kinematics(1, &stage, &elements, &model, &env, Some(&output));
    let base_target = compute_section_kinematics(&stage, &elements[1], &env);
    assert_close(
        target.air_relative_velocity_element_mps.x,
        base_target.air_relative_velocity_element_mps.x + wake.induced_velocity_mps(),
        2.0e-15,
    );
    assert!(target.section_airspeed_mps > base_target.section_airspeed_mps);
    assert!(target.dynamic_pressure_pa > base_target.dynamic_pressure_pa);
    let viscosity = model.kinematic_viscosity_m2_s().unwrap();
    let re_before = calculate_reynolds_number(
        base_target.section_airspeed_mps,
        elements[1].chord_m(),
        viscosity,
    )
    .unwrap();
    let re_after = calculate_reynolds_number(
        target.section_airspeed_mps,
        elements[1].chord_m(),
        viscosity,
    )
    .unwrap();
    assert!(re_after > re_before);
    let family = model.aero_polar_families()[0].family();
    assert_ne!(
        family.sample(re_before, target.alpha_rad).coefficients,
        family.sample(re_after, target.alpha_rad).coefficients
    );

    for untargeted in [0, 2] {
        assert_eq!(
            evaluate_aircraft_section_kinematics(
                untargeted,
                &stage,
                &elements,
                &model,
                &env,
                Some(&output),
            ),
            compute_section_kinematics(&stage, &elements[untargeted], &env)
        );
    }

    let mut empty_value = fixture();
    empty_value["propeller_slipstream_interactions"] = json!([]);
    let empty_model = load(&empty_value);
    let empty_elements = effective(&empty_model);
    let empty_output = propulsion(&empty_model, &stage, 0.7, &env);
    assert_eq!(
        evaluate_aircraft_surface_aerodynamic_state(
            0,
            &stage,
            &elements,
            &model,
            &env,
            Some(&output),
        ),
        evaluate_aircraft_surface_aerodynamic_state(
            0,
            &stage,
            &empty_elements,
            &empty_model,
            &env,
            Some(&empty_output),
        )
    );
}

#[test]
fn axial_v_uses_propeller_location_orientation_wind_and_stage_angular_velocity() {
    let mut value = fixture();
    let half_angle = 0.15_f64;
    value["propulsion"]["propeller"]["orientation_body_from_prop_wxyz"] =
        json!([half_angle.cos(), 0.0, half_angle.sin(), 0.0]);
    let model = load(&value);
    let stage = RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(11.0, -0.5, 1.8),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::new(0.2, -0.4, 0.7),
    };
    let env = AeroEnvironment::new(RHO, Vec3::new(1.5, 0.25, -0.3)).unwrap();
    let runtime_propeller = model.propulsion().unwrap().config().propeller();
    let body_at_prop = stage.linear_velocity_world_mps - env.wind_velocity_world_mps()
        + stage
            .angular_velocity_body_radps
            .cross(runtime_propeller.position_body_m());
    let expected_prop_velocity = runtime_propeller
        .orientation_body_from_prop()
        .inverse_transform_vector(&body_at_prop);
    let output = propulsion(&model, &stage, 0.7, &env);
    assert_vec_close(
        output.air_relative_velocity_prop_mps,
        expected_prop_velocity,
        2.0e-15,
    );
    assert_eq!(
        output.axial_airspeed_mps.to_bits(),
        expected_prop_velocity.x.to_bits()
    );
    let wake = propeller_slipstream(&model, &env, &output);
    let expected_axis = runtime_propeller
        .orientation_body_from_prop()
        .transform_vector(&Vec3::new(1.0, 0.0, 0.0));
    assert_vec_close(wake.axis_body(), expected_axis, 2.0e-15);

    let elements = effective(&model);
    let base = compute_section_kinematics(&stage, &elements[1], &env);
    let affected =
        evaluate_aircraft_section_kinematics(1, &stage, &elements, &model, &env, Some(&output));
    let expected_increment = elements[1]
        .orientation_body_from_element()
        .inverse_transform_vector(&(expected_axis * wake.induced_velocity_mps()));
    assert_vec_close(
        affected.air_relative_velocity_element_mps,
        base.air_relative_velocity_element_mps + expected_increment,
        3.0e-15,
    );
}

#[test]
fn authored_factor_scales_vi_and_throttle_enters_only_through_actual_thrust() {
    let stage = state(SPEED_MPS, 0.0);
    let env = environment(RHO);
    let model = load(&fixture());
    let elements = effective(&model);
    for throttle in [0.35, 0.8] {
        let output = propulsion(&model, &stage, throttle, &env);
        let wake = propeller_slipstream(&model, &env, &output);
        let target =
            evaluate_aircraft_section_kinematics(1, &stage, &elements, &model, &env, Some(&output));
        assert_close(
            target.section_airspeed_mps - SPEED_MPS,
            wake.induced_velocity_mps(),
            2.0e-15,
        );
    }

    let mut doubled = fixture();
    doubled["propeller_slipstream_interactions"][0]["slipstream_velocity_factor"] = json!(2.0);
    let doubled = load(&doubled);
    let doubled_elements = effective(&doubled);
    let output = propulsion(&doubled, &stage, 0.8, &env);
    let wake = propeller_slipstream(&doubled, &env, &output);
    let target = evaluate_aircraft_section_kinematics(
        1,
        &stage,
        &doubled_elements,
        &doubled,
        &env,
        Some(&output),
    );
    assert_close(
        target.section_airspeed_mps - SPEED_MPS,
        2.0 * wake.induced_velocity_mps(),
        4.0e-15,
    );
}

#[test]
fn propulsion_wrench_is_combined_exactly_once_and_fixed_polar_targets_work() {
    let mut value = fixture();
    value["aerodynamics"]["elements"][1]["polar_binding"] =
        json!({"kind": "polar", "polar_id": "synthetic-fixed-linear"});
    let model = load(&value);
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    let evaluation = evaluate_aircraft_instantaneous(&stage, &elements, &model, 0.7, &config(RHO));
    let output = evaluation.propulsion().unwrap();
    let aero =
        evaluate_aerodynamic_wrench_with_propulsion(&stage, &elements, &model, &env, Some(output));
    assert_eq!(
        evaluation.total_wrench().force_body_n,
        aero.force_body_n + output.wrench_body.force_body_n
    );
    assert_eq!(
        evaluation.total_wrench().moment_body_nm,
        aero.moment_body_nm + output.wrench_body.moment_body_nm
    );
}

#[test]
fn finite_wing_solution_uses_each_members_actual_qs_and_adjusted_reynolds() {
    let mut value = fixture();
    let mut second = value["aerodynamics"]["elements"][1].clone();
    second["id"] = json!("synthetic-tail-unwashed");
    second["area_m2"] = json!(0.24);
    value["aerodynamics"]["elements"]
        .as_array_mut()
        .unwrap()
        .push(second);
    value["aerodynamics"]["surfaces"][1]["element_ids"] =
        json!(["synthetic-tail", "synthetic-tail-unwashed"]);
    let model = load(&value);
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    let output = propulsion(&model, &stage, 0.7, &env);
    let solution = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &stage,
        &elements,
        &model,
        &env,
        Some(&output),
    );
    let viscosity = model.kinematic_viscosity_m2_s().unwrap();
    let family = model.aero_polar_families()[0].family();
    let mut weighted_cl = 0.0;
    let mut weight = 0.0;
    let mut unweighted_cl = 0.0;
    for index in [1, 3] {
        let kin = evaluate_aircraft_section_kinematics(
            index,
            &stage,
            &elements,
            &model,
            &env,
            Some(&output),
        );
        let re = kin.section_airspeed_mps * elements[index].chord_m() / viscosity;
        let cl = family
            .sample(re, kin.alpha_rad - solution.induced_alpha_rad)
            .coefficients
            .cl;
        let member_weight = kin.dynamic_pressure_pa * elements[index].area_m2();
        weighted_cl += member_weight * cl;
        weight += member_weight;
        unweighted_cl += cl;
        if index == 3 {
            assert_eq!(
                kin,
                compute_section_kinematics(&stage, &elements[index], &env)
            );
        }
    }
    assert_close(solution.surface_cl, weighted_cl / weight, 2.0e-13);
    assert!((solution.surface_cl - unweighted_cl / 2.0).abs() > 1.0e-4);
}

#[test]
fn slipstream_then_downwash_then_self_induction_is_discriminating_and_preserves_speed() {
    let mut value = fixture();
    value["aero_downwash_interactions"] = json!([{
        "id": "synthetic-wing-to-tail",
        "source_surface_id": "synthetic-wing-surface",
        "target_surface_id": "synthetic-tail-surface",
        "downwash_factor": 0.8
    }]);
    let model = load(&value);
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    let output = propulsion(&model, &stage, 0.7, &env);
    let pre_downwash =
        evaluate_aircraft_section_kinematics(1, &stage, &elements, &model, &env, Some(&output));
    let solution = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &stage,
        &elements,
        &model,
        &env,
        Some(&output),
    );
    assert!(solution.source_alpha_i_rad > 0.0);
    assert_close(
        solution.downwash_angle_rad,
        0.8 * solution.source_alpha_i_rad,
        1.0e-15,
    );
    let final_alpha_geom = pre_downwash.alpha_rad - solution.downwash_angle_rad;
    let alpha_sample = final_alpha_geom - solution.induced_alpha_rad;
    assert!(final_alpha_geom < pre_downwash.alpha_rad);
    assert!(alpha_sample < final_alpha_geom);
    assert!(pre_downwash.section_airspeed_mps > SPEED_MPS);
    let rotated_speed = (pre_downwash.section_airspeed_mps * final_alpha_geom.cos())
        .hypot(pre_downwash.section_airspeed_mps * final_alpha_geom.sin());
    assert_close(rotated_speed, pre_downwash.section_airspeed_mps, 2.0e-15);
}

#[test]
fn force_direction_uses_final_physical_flow_and_control_deflection_keeps_slipstream() {
    let mut value = fixture();
    for sample in value["aerodynamics"]["polars"][0]["samples"]
        .as_array_mut()
        .unwrap()
    {
        sample["cl"] = json!(0.0);
        sample["cd"] = json!(0.0);
        sample["cm"] = json!(0.0);
    }
    let model = load(&value);
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    let output = propulsion(&model, &stage, 0.7, &env);
    let kin =
        evaluate_aircraft_section_kinematics(1, &stage, &elements, &model, &env, Some(&output));
    let surface = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &stage,
        &elements,
        &model,
        &env,
        Some(&output),
    );
    let re = kin.section_airspeed_mps * elements[1].chord_m()
        / model.kinematic_viscosity_m2_s().unwrap();
    let mut coeffs = model.aero_polar_families()[0]
        .family()
        .sample(re, kin.alpha_rad - surface.induced_alpha_rad)
        .coefficients;
    coeffs.cd += surface.induced_drag_coefficient;
    let expected = assemble_aero_element_wrench(&elements[1], &kin, &coeffs);
    let actual =
        evaluate_aerodynamic_wrench_with_propulsion(&stage, &elements, &model, &env, Some(&output));
    assert_vec_close(actual.force_body_n, expected.force_body_n, 2.0e-12);
    assert_vec_close(actual.moment_body_nm, expected.moment_body_nm, 2.0e-12);

    let positions =
        evaluate_steady_controls(model.controls(), &PilotInput::new(0.0, 0.6, 0.0, 0.7));
    let deflected = effective_aero_elements_for_positions(&model, &positions);
    let deflected_kin =
        evaluate_aircraft_section_kinematics(1, &stage, &deflected, &model, &env, Some(&output));
    let deflected_base = compute_section_kinematics(&stage, &deflected[1], &env);
    let wake = propeller_slipstream(&model, &env, &output);
    let expected_increment = deflected[1]
        .orientation_body_from_element()
        .inverse_transform_vector(&(wake.axis_body() * wake.induced_velocity_mps()));
    assert_vec_close(
        deflected_kin.air_relative_velocity_element_mps,
        deflected_base.air_relative_velocity_element_mps + expected_increment,
        2.0e-15,
    );
    assert_ne!(deflected_kin.alpha_rad.to_bits(), kin.alpha_rad.to_bits());
}

#[test]
fn evaluation_allocates_nothing_and_repeated_rk4_is_bit_deterministic() {
    let model = load(&fixture());
    let stage = state(SPEED_MPS, ALPHA_RAD);
    let env = environment(RHO);
    let elements = effective(&model);
    std::hint::black_box(evaluate_aircraft_wrench(
        &stage, &elements, &model, 0.7, &env,
    ));
    let allocations = allocation_counter::measure(|| {
        for _ in 0..100 {
            std::hint::black_box(evaluate_aircraft_wrench(
                std::hint::black_box(&stage),
                std::hint::black_box(&elements),
                std::hint::black_box(&model),
                0.7,
                std::hint::black_box(&env),
            ));
        }
    });
    assert_eq!(allocations.count_total, 0, "{allocations:?}");

    let mut first = AircraftSimulation::new(model.clone(), config(RHO), stage).unwrap();
    let mut second = AircraftSimulation::new(model, config(RHO), stage).unwrap();
    let input = PilotInput::new(0.0, 0.2, 0.0, 0.7);
    for _ in 0..100 {
        assert_eq!(first.step(&input), second.step(&input));
    }
}

#[test]
fn trim_qualification_audits_the_exact_runtime_slipstream_flow() {
    let model = load(&fixture());
    let trim_config =
        AircraftSimulationConfig::new(0.002, Vec3::new(0.0, 0.0, 9.80665), environment(RHO))
            .unwrap();
    let request = LongitudinalTrimRequest::new(
        SPEED_MPS,
        TrimBounds::new(-0.2, 0.5).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.2, 0.0, 0.6).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        80,
    )
    .unwrap();
    let solution = solve_longitudinal_trim(&model, &trim_config, &request).unwrap();
    let limits =
        LongitudinalTrimQualificationLimits::new(1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 1.0e6).unwrap();
    let point =
        qualify_longitudinal_trim_solution(&model, &trim_config, &solution, &limits, SPEED_MPS);
    assert!(
        !point
            .outcome
            .blockers()
            .contains(&QualificationBlocker::ReEvaluationFailure)
    );
    let diagnostics = match &point.outcome {
        LongitudinalTrimQualificationOutcome::Qualified(diagnostics)
        | LongitudinalTrimQualificationOutcome::NotQualifiedDomainViolation(diagnostics)
        | LongitudinalTrimQualificationOutcome::NotQualifiedResidualViolation(diagnostics) => {
            diagnostics
        }
        other => panic!("successful trim qualification must retain diagnostics, got {other:?}"),
    };
    let audits = diagnostics.aero_audits();
    let evaluation = &solution.evaluation;
    let elements =
        effective_aero_elements_for_positions(&model, &evaluation.control_surface_positions);
    let output = propulsion(
        &model,
        &evaluation.state,
        evaluation.control_surface_positions.throttle(),
        trim_config.aero_environment(),
    );
    let kin = evaluate_aircraft_section_kinematics(
        1,
        &evaluation.state,
        &elements,
        &model,
        trim_config.aero_environment(),
        Some(&output),
    );
    let surface = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &evaluation.state,
        &elements,
        &model,
        trim_config.aero_environment(),
        Some(&output),
    );
    let audit = &audits[1];
    assert_eq!(
        audit.section_airspeed_mps.to_bits(),
        kin.section_airspeed_mps.to_bits()
    );
    assert_eq!(audit.alpha_geom_rad.to_bits(), kin.alpha_rad.to_bits());
    assert_eq!(
        audit.alpha_sample_rad.to_bits(),
        (kin.alpha_rad - surface.induced_alpha_rad).to_bits()
    );
    let expected_re = kin.section_airspeed_mps * elements[1].chord_m()
        / model.kinematic_viscosity_m2_s().unwrap();
    assert_eq!(
        audit.reynolds_number.unwrap().to_bits(),
        expected_re.to_bits()
    );
}
