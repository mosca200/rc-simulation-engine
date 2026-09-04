//! M2.8C deterministic wing-to-tail downwash physics. All data is synthetic.

use aircraft::{AircraftSimulation, AircraftSimulationConfig, evaluate_aerodynamic_wrench};
use model::{AircraftModel, AircraftModelLoader};
use serde_json::{Value, json};
use sim_core::{
    AeroEnvironment, BodyWrench, PilotInput, RigidBodyState, calculate_reynolds_number,
};
use sim_math::{Orientation, Vec3};
use std::f64::consts::PI;

const SPEED_MPS: f64 = 18.0;
const ALPHA_RAD: f64 = 0.15;
const RHO: f64 = 1.225;

fn fixture_value() -> Value {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_downwash_v6.json"
    ))
    .unwrap()
}

fn load(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

fn state(alpha_rad: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(
            SPEED_MPS * alpha_rad.cos(),
            0.0,
            SPEED_MPS * alpha_rad.sin(),
        ),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn environment() -> AeroEnvironment {
    AeroEnvironment::new(RHO, Vec3::zeros()).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(0.002, Vec3::zeros(), environment()).unwrap()
}

fn effective_elements(model: &AircraftModel) -> Vec<sim_core::AeroElement> {
    model
        .aero_elements()
        .iter()
        .map(|runtime| *runtime.element())
        .collect()
}

fn evaluate(model: &AircraftModel, alpha_rad: f64) -> BodyWrench {
    evaluate_aerodynamic_wrench(
        &state(alpha_rad),
        &effective_elements(model),
        model,
        &environment(),
    )
}

fn section_force(alpha_flow: f64, q: f64, area: f64, cl: f64, cd: f64) -> Vec3 {
    let flow_hat = Vec3::new(alpha_flow.cos(), 0.0, alpha_flow.sin());
    Vec3::y().cross(&flow_hat) * (q * area * cl) - flow_hat * (q * area * cd)
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance:e}"
    );
}

struct AnalyticResult {
    source_alpha_i: f64,
    epsilon: f64,
    target_alpha_geom: f64,
    target_alpha_i: f64,
    source_force: Vec3,
    target_force: Vec3,
    unassigned_force: Vec3,
    total_wrench: BodyWrench,
}

fn analytic_result(model: &AircraftModel, alpha: f64, factor: f64) -> AnalyticResult {
    let q = 0.5 * RHO * SPEED_MPS * SPEED_MPS;
    let source = &model.aero_surfaces()[0];
    let source_slope = 3.0;
    let source_k = source_slope / (PI * source.aspect_ratio() * source.span_efficiency_factor());
    let source_alpha_i = source_k * alpha / (1.0 + source_k);
    let source_cl = source_slope * (alpha - source_alpha_i);
    let source_cdi =
        source_cl * source_cl / (PI * source.aspect_ratio() * source.span_efficiency_factor());
    let source_force = section_force(alpha, q, source.area_m2(), source_cl, 0.01 + source_cdi);

    let epsilon = factor * source_alpha_i;
    let target_alpha_geom = alpha - epsilon;
    let target = &model.aero_surfaces()[1];
    let target_element = &model.aero_elements()[target.element_indices()[0]];
    let reynolds = calculate_reynolds_number(
        SPEED_MPS,
        target_element.element().chord_m(),
        model.kinematic_viscosity_m2_s().unwrap(),
    )
    .unwrap();
    let family = model.aero_polar_families()[0].family();
    let target_slope = family.sample(reynolds, 0.2).coefficients.cl / 0.2;
    let target_k = target_slope / (PI * target.aspect_ratio() * target.span_efficiency_factor());
    let target_alpha_i = target_k * target_alpha_geom / (1.0 + target_k);
    let target_sample_alpha = target_alpha_geom - target_alpha_i;
    let target_coefficients = family.sample(reynolds, target_sample_alpha).coefficients;
    let target_cdi = target_coefficients.cl * target_coefficients.cl
        / (PI * target.aspect_ratio() * target.span_efficiency_factor());
    let target_force = section_force(
        target_alpha_geom,
        q,
        target.area_m2(),
        target_coefficients.cl,
        target_coefficients.cd + target_cdi,
    );

    let unassigned = model.aero_elements()[2].element();
    let unassigned_coefficients = model.aero_polars()[2].table().sample_clamped(alpha);
    let unassigned_force = section_force(
        alpha,
        q,
        unassigned.area_m2(),
        unassigned_coefficients.cl,
        unassigned_coefficients.cd,
    );

    let total_force = source_force + target_force + unassigned_force;
    let total_moment = model.aero_elements()[1]
        .element()
        .position_body_m()
        .cross(&target_force)
        + unassigned.position_body_m().cross(&unassigned_force);
    AnalyticResult {
        source_alpha_i,
        epsilon,
        target_alpha_geom,
        target_alpha_i,
        source_force,
        target_force,
        unassigned_force,
        total_wrench: BodyWrench {
            force_body_n: total_force,
            moment_body_nm: total_moment,
        },
    }
}

#[test]
fn empty_and_zero_interactions_preserve_prior_finite_wing_physics_bit_exactly() {
    let mut v6_empty = fixture_value();
    v6_empty["aero_downwash_interactions"] = json!([]);
    let mut v5 = v6_empty.clone();
    v5["schema_version"] = json!(5);
    v5.as_object_mut()
        .unwrap()
        .remove("aero_downwash_interactions");
    let prior = evaluate(&load(&v5), ALPHA_RAD);
    let empty = evaluate(&load(&v6_empty), ALPHA_RAD);
    assert_eq!(prior, empty);

    let mut zero = fixture_value();
    zero["aero_downwash_interactions"][0]["downwash_factor"] = json!(0.0);
    assert_eq!(empty, evaluate(&load(&zero), ALPHA_RAD));
}

#[test]
fn positive_source_lift_rotates_target_flow_and_composes_target_induced_alpha() {
    let model = load(&fixture_value());
    let actual = evaluate(&model, ALPHA_RAD);
    let expected = analytic_result(&model, ALPHA_RAD, 1.5);

    assert!(expected.source_alpha_i > 0.0);
    assert_eq!(expected.epsilon, 1.5 * expected.source_alpha_i);
    assert!(expected.target_alpha_geom < ALPHA_RAD);
    assert!((expected.target_alpha_geom - (ALPHA_RAD - expected.epsilon)).abs() < 1.0e-15);
    assert!(expected.target_alpha_i > 0.0);
    assert_vec_close(
        actual.force_body_n,
        expected.total_wrench.force_body_n,
        2.0e-9,
    );
    assert_vec_close(
        actual.moment_body_nm,
        expected.total_wrench.moment_body_nm,
        2.0e-9,
    );

    let q = 0.5 * RHO * SPEED_MPS * SPEED_MPS;
    let target = &model.aero_surfaces()[1];
    let target_element = &model.aero_elements()[target.element_indices()[0]];
    let reynolds = calculate_reynolds_number(
        SPEED_MPS,
        target_element.element().chord_m(),
        model.kinematic_viscosity_m2_s().unwrap(),
    )
    .unwrap();
    let family = model.aero_polar_families()[0].family();
    let correct_sample = expected.target_alpha_geom - expected.target_alpha_i;
    let correct = family.sample(reynolds, correct_sample).coefficients;
    let cdi =
        correct.cl * correct.cl / (PI * target.aspect_ratio() * target.span_efficiency_factor());
    let wrong_sample = ALPHA_RAD - expected.target_alpha_i;
    let wrong_coefficients = family.sample(reynolds, wrong_sample).coefficients;
    let wrong_sample_force = section_force(
        expected.target_alpha_geom,
        q,
        target.area_m2(),
        wrong_coefficients.cl,
        wrong_coefficients.cd + cdi,
    );
    assert!((expected.target_force - wrong_sample_force).norm() > 0.1);

    let wrong_direction_force =
        section_force(ALPHA_RAD, q, target.area_m2(), correct.cl, correct.cd + cdi);
    assert!((expected.target_force - wrong_direction_force).norm() > 0.01);

    let inferred_source = actual.force_body_n - expected.target_force - expected.unassigned_force;
    assert_vec_close(inferred_source, expected.source_force, 2.0e-9);
}

#[test]
fn fixed_polar_target_and_negative_source_lift_have_the_expected_direction() {
    let mut fixed = fixture_value();
    fixed["aerodynamics"]["elements"][1]["polar_binding"] =
        json!({"kind": "polar", "polar_id": "target-fixed-linear"});
    let coupled = load(&fixed);
    let mut empty = fixed.clone();
    empty["aero_downwash_interactions"] = json!([]);
    let uncoupled = load(&empty);

    assert!(
        evaluate(&coupled, ALPHA_RAD).force_body_n.z
            > evaluate(&uncoupled, ALPHA_RAD).force_body_n.z
    );
    assert!(
        evaluate(&coupled, -ALPHA_RAD).force_body_n.z
            < evaluate(&uncoupled, -ALPHA_RAD).force_body_n.z
    );
}

#[test]
fn pure_rotation_preserves_target_reynolds_and_uses_the_correct_family_interpolation() {
    let model = load(&fixture_value());
    let target = model.aero_elements()[1].element();
    let reynolds_before = calculate_reynolds_number(
        SPEED_MPS,
        target.chord_m(),
        model.kinematic_viscosity_m2_s().unwrap(),
    )
    .unwrap();
    let expected = analytic_result(&model, ALPHA_RAD, 1.5);
    let rotated_speed = SPEED_MPS.hypot(0.0);
    let reynolds_after = calculate_reynolds_number(
        rotated_speed,
        target.chord_m(),
        model.kinematic_viscosity_m2_s().unwrap(),
    )
    .unwrap();
    assert_eq!(reynolds_before.to_bits(), reynolds_after.to_bits());
    assert_eq!(reynolds_before, 300_000.0);
    assert_vec_close(
        evaluate(&model, ALPHA_RAD).force_body_n,
        expected.total_wrench.force_body_n,
        2.0e-9,
    );
}

#[test]
fn unassigned_element_is_unchanged_and_induced_drag_is_individually_accounted() {
    let model = load(&fixture_value());
    let expected = analytic_result(&model, ALPHA_RAD, 1.5);
    let uncoupled = analytic_result(&model, ALPHA_RAD, 0.0);
    let coupled_actual = evaluate(&model, ALPHA_RAD);
    let mut empty = fixture_value();
    empty["aero_downwash_interactions"] = json!([]);
    let uncoupled_actual = evaluate(&load(&empty), ALPHA_RAD);

    assert_eq!(expected.unassigned_force, uncoupled.unassigned_force);
    assert_vec_close(
        coupled_actual.force_body_n - uncoupled_actual.force_body_n,
        expected.target_force - uncoupled.target_force,
        3.0e-9,
    );
    let q = 0.5 * RHO * SPEED_MPS * SPEED_MPS;
    let source = &model.aero_surfaces()[0];
    let source_cl = 3.0 * (ALPHA_RAD - expected.source_alpha_i);
    let source_without_induced_drag =
        section_force(ALPHA_RAD, q, source.area_m2(), source_cl, 0.01);
    let source_flow_hat = Vec3::new(ALPHA_RAD.cos(), 0.0, ALPHA_RAD.sin());
    assert!((expected.source_force - source_without_induced_drag).dot(&source_flow_hat) < 0.0);

    let target = &model.aero_surfaces()[1];
    let target_without_induced_drag = section_force(
        expected.target_alpha_geom,
        q,
        target.area_m2(),
        model.aero_polar_families()[0]
            .family()
            .sample(
                300_000.0,
                expected.target_alpha_geom - expected.target_alpha_i,
            )
            .coefficients
            .cl,
        model.aero_polar_families()[0]
            .family()
            .sample(
                300_000.0,
                expected.target_alpha_geom - expected.target_alpha_i,
            )
            .coefficients
            .cd,
    );
    let target_flow_hat = Vec3::new(
        expected.target_alpha_geom.cos(),
        0.0,
        expected.target_alpha_geom.sin(),
    );
    assert!((expected.target_force - target_without_induced_drag).dot(&target_flow_hat) < 0.0);
}

#[test]
fn independent_interactions_are_declaration_order_independent() {
    let mut first = fixture_value();
    first["aerodynamics"]["surfaces"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "second-tail",
            "element_ids": ["unassigned-element"],
            "span_axis_body": [0.0, 1.0, 0.0],
            "span_m": 0.5,
            "span_efficiency_factor": 0.85
        }));
    first["aero_downwash_interactions"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "wing-to-second-tail",
            "source_surface_id": "main-wing",
            "target_surface_id": "second-tail",
            "downwash_factor": 0.7
        }));
    let mut reversed = first.clone();
    reversed["aero_downwash_interactions"]
        .as_array_mut()
        .unwrap()
        .reverse();
    assert_eq!(
        evaluate(&load(&first), ALPHA_RAD),
        evaluate(&load(&reversed), ALPHA_RAD)
    );
}

#[test]
fn evaluation_allocates_nothing_and_repeated_rk4_is_bit_deterministic() {
    let model = load(&fixture_value());
    let state = state(ALPHA_RAD);
    let effective = effective_elements(&model);
    let env = environment();
    std::hint::black_box(evaluate_aerodynamic_wrench(
        &state, &effective, &model, &env,
    ));
    let allocation_info = allocation_counter::measure(|| {
        for _ in 0..100 {
            std::hint::black_box(evaluate_aerodynamic_wrench(
                std::hint::black_box(&state),
                std::hint::black_box(&effective),
                std::hint::black_box(&model),
                std::hint::black_box(&env),
            ));
        }
    });
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");

    let mut first = AircraftSimulation::new(model.clone(), config(), state).unwrap();
    let mut second = AircraftSimulation::new(model, config(), state).unwrap();
    let input = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    for _ in 0..100 {
        assert_eq!(first.step(&input), second.step(&input));
    }
}
