//! M2.8E deterministic rotational propwash. All model values are synthetic.

use aircraft::{
    AircraftSimulationConfig, evaluate_aerodynamic_wrench_with_propulsion,
    evaluate_aircraft_instantaneous, evaluate_aircraft_section_kinematics,
    evaluate_aircraft_surface_aerodynamic_state, evaluate_aircraft_wrench, propeller_slipstream,
};
use model::{AircraftModel, AircraftModelLoader};
use serde_json::{Value, json};
use sim_core::{
    AeroEnvironment, RigidBodyState, assemble_aero_element_wrench, calculate_reynolds_number,
    compute_section_kinematics, evaluate_electric_propulsion_with_source,
};
use sim_math::{Orientation, Vec3};

const RHO: f64 = 1.225;
const SPEED_MPS: f64 = 8.0;
const SWIRL_FACTOR: f64 = 0.625;

fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap()
}

fn configured(swirl_factor: Option<f64>) -> Value {
    let mut value = fixture();
    value["aerodynamics"]["elements"][1]["position_body_m"] = json!([-0.75, 0.30, 0.0]);
    if let Some(factor) = swirl_factor {
        value["propeller_slipstream_interactions"][0]["swirl_velocity_factor"] = json!(factor);
    }
    value
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

fn simulation_config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(0.002, Vec3::zeros(), environment()).unwrap()
}

fn elements(model: &AircraftModel) -> Vec<sim_core::AeroElement> {
    model
        .aero_elements()
        .iter()
        .map(|element| *element.element())
        .collect()
}

fn propulsion(model: &AircraftModel, stage: &RigidBodyState) -> sim_core::PropulsionOutput {
    let runtime = model.propulsion().unwrap();
    evaluate_electric_propulsion_with_source(
        stage,
        0.7,
        runtime.config(),
        &environment(),
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
fn zero_swirl_is_bit_exact_m2_8d_and_positive_swirl_is_tangential() {
    let absent = load(&configured(None));
    let zero = load(&configured(Some(0.0)));
    let stage = state(0.0);
    let env = environment();
    assert_eq!(
        evaluate_aircraft_wrench(&stage, &elements(&absent), &absent, 0.7, &env),
        evaluate_aircraft_wrench(&stage, &elements(&zero), &zero, 0.7, &env)
    );

    let model = load(&configured(Some(SWIRL_FACTOR)));
    let effective = elements(&model);
    let output = propulsion(&model, &stage);
    let wake = propeller_slipstream(&model, &env, &output);
    let base = compute_section_kinematics(&stage, &effective[1], &env);
    let affected =
        evaluate_aircraft_section_kinematics(1, &stage, &effective, &model, &env, Some(&output));
    let total_increment =
        affected.air_relative_velocity_element_mps - base.air_relative_velocity_element_mps;
    let swirl_increment = total_increment - wake.axis_body() * wake.induced_velocity_mps();
    assert_vec_close(
        total_increment,
        Vec3::new(
            wake.induced_velocity_mps(),
            0.0,
            SWIRL_FACTOR * wake.induced_velocity_mps(),
        ),
        3.0e-15,
    );
    assert_close(
        swirl_increment.norm(),
        SWIRL_FACTOR * wake.induced_velocity_mps(),
        3.0e-15,
    );
    assert_close(swirl_increment.dot(&wake.axis_body()), 0.0, 2.0e-15);

    assert_eq!(
        evaluate_aircraft_section_kinematics(2, &stage, &effective, &model, &env, Some(&output),),
        compute_section_kinematics(&stage, &effective[2], &env)
    );
}

#[test]
fn spin_sign_and_opposite_radial_positions_reverse_tangential_flow() {
    let mut value = configured(Some(SWIRL_FACTOR));
    let half_sqrt_two = std::f64::consts::FRAC_1_SQRT_2;
    value["propulsion"]["propeller"]["orientation_body_from_prop_wxyz"] =
        json!([half_sqrt_two, 0.0, 0.0, half_sqrt_two]);
    value["aerodynamics"]["elements"][1]["position_body_m"] = json!([0.54, -0.75, 0.0]);
    let mut opposite = value["aerodynamics"]["elements"][1].clone();
    opposite["id"] = json!("synthetic-opposite-tail");
    opposite["position_body_m"] = json!([-0.06, -0.75, 0.0]);
    value["aerodynamics"]["elements"]
        .as_array_mut()
        .unwrap()
        .push(opposite);
    value["propeller_slipstream_interactions"][0]["target_element_ids"] =
        json!(["synthetic-tail", "synthetic-opposite-tail"]);

    let stage = state(0.0);
    let env = environment();
    let positive = load(&value);
    let positive_elements = elements(&positive);
    let positive_output = propulsion(&positive, &stage);
    let positive_wake = propeller_slipstream(&positive, &env, &positive_output);
    assert_vec_close(positive_wake.axis_body(), Vec3::new(0.0, 1.0, 0.0), 3.0e-15);
    let mut positive_swirl = [Vec3::zeros(); 2];
    for (slot, index) in [1, 3].into_iter().enumerate() {
        let base = compute_section_kinematics(&stage, &positive_elements[index], &env);
        let affected = evaluate_aircraft_section_kinematics(
            index,
            &stage,
            &positive_elements,
            &positive,
            &env,
            Some(&positive_output),
        );
        positive_swirl[slot] = affected.air_relative_velocity_element_mps
            - base.air_relative_velocity_element_mps
            - positive_wake.axis_body() * positive_wake.induced_velocity_mps();
    }
    assert!(positive_swirl[0].z < 0.0);
    assert!(positive_swirl[1].z > 0.0);
    assert_vec_close(positive_swirl[0], -positive_swirl[1], 3.0e-15);

    value["propulsion"]["propeller"]["spin_direction"] = json!("negative_about_local_x");
    let negative = load(&value);
    let negative_elements = elements(&negative);
    let negative_output = propulsion(&negative, &stage);
    assert_eq!(
        positive_output.thrust_n.to_bits(),
        negative_output.thrust_n.to_bits()
    );
    let negative_wake = propeller_slipstream(&negative, &env, &negative_output);
    for (slot, index) in [1, 3].into_iter().enumerate() {
        let base = compute_section_kinematics(&stage, &negative_elements[index], &env);
        let affected = evaluate_aircraft_section_kinematics(
            index,
            &stage,
            &negative_elements,
            &negative,
            &env,
            Some(&negative_output),
        );
        let swirl = affected.air_relative_velocity_element_mps
            - base.air_relative_velocity_element_mps
            - negative_wake.axis_body() * negative_wake.induced_velocity_mps();
        assert_vec_close(swirl, -positive_swirl[slot], 3.0e-15);
        assert_close(swirl.dot(&negative_wake.axis_body()), 0.0, 2.0e-15);
    }
}

#[test]
fn final_physical_flow_drives_q_reynolds_and_aerodynamic_wrench() {
    let mut value = configured(Some(SWIRL_FACTOR));
    for sample in value["aerodynamics"]["polars"][0]["samples"]
        .as_array_mut()
        .unwrap()
    {
        sample["cl"] = json!(0.0);
        sample["cd"] = json!(0.0);
        sample["cm"] = json!(0.0);
    }
    let model = load(&value);
    let effective = elements(&model);
    let stage = state(0.08);
    let env = environment();
    let output = propulsion(&model, &stage);
    let kin =
        evaluate_aircraft_section_kinematics(1, &stage, &effective, &model, &env, Some(&output));
    let axial_only = load(&configured(Some(0.0)));
    let axial_elements = elements(&axial_only);
    let axial_output = propulsion(&axial_only, &stage);
    let axial_kin = evaluate_aircraft_section_kinematics(
        1,
        &stage,
        &axial_elements,
        &axial_only,
        &env,
        Some(&axial_output),
    );
    assert!(kin.section_airspeed_mps > axial_kin.section_airspeed_mps);
    assert!(kin.dynamic_pressure_pa > axial_kin.dynamic_pressure_pa);
    let viscosity = model.kinematic_viscosity_m2_s().unwrap();
    let reynolds =
        calculate_reynolds_number(kin.section_airspeed_mps, effective[1].chord_m(), viscosity)
            .unwrap();
    let axial_reynolds = calculate_reynolds_number(
        axial_kin.section_airspeed_mps,
        axial_elements[1].chord_m(),
        viscosity,
    )
    .unwrap();
    assert!(reynolds > axial_reynolds);

    let surface = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &stage,
        &effective,
        &model,
        &env,
        Some(&output),
    );
    let mut coefficients = model.aero_polar_families()[0]
        .family()
        .sample(reynolds, kin.alpha_rad - surface.induced_alpha_rad)
        .coefficients;
    coefficients.cd += surface.induced_drag_coefficient;
    let expected = assemble_aero_element_wrench(&effective[1], &kin, &coefficients);
    let aero = evaluate_aerodynamic_wrench_with_propulsion(
        &stage,
        &effective,
        &model,
        &env,
        Some(&output),
    );
    assert_vec_close(aero.force_body_n, expected.force_body_n, 3.0e-12);
    assert_vec_close(aero.moment_body_nm, expected.moment_body_nm, 3.0e-12);

    let complete =
        evaluate_aircraft_instantaneous(&stage, &effective, &model, 0.7, &simulation_config());
    let actual_propulsion = complete.propulsion().unwrap();
    assert_eq!(
        complete.total_wrench().force_body_n,
        aero.force_body_n + actual_propulsion.wrench_body.force_body_n
    );
    assert_eq!(
        complete.total_wrench().moment_body_nm,
        aero.moment_body_nm + actual_propulsion.wrench_body.moment_body_nm
    );
}

#[test]
fn swirl_precedes_downwash_and_target_self_induction() {
    let mut value = configured(Some(SWIRL_FACTOR));
    value["aero_downwash_interactions"] = json!([{
        "id": "synthetic-wing-to-tail",
        "source_surface_id": "synthetic-wing-surface",
        "target_surface_id": "synthetic-tail-surface",
        "downwash_factor": 0.8
    }]);
    let model = load(&value);
    let effective = elements(&model);
    let stage = state(0.16);
    let env = environment();
    let output = propulsion(&model, &stage);
    let pre_downwash =
        evaluate_aircraft_section_kinematics(1, &stage, &effective, &model, &env, Some(&output));
    let surface = evaluate_aircraft_surface_aerodynamic_state(
        1,
        &stage,
        &effective,
        &model,
        &env,
        Some(&output),
    );
    assert!(surface.source_alpha_i_rad > 0.0);
    assert!(surface.downwash_angle_rad > 0.0);
    let final_alpha_geom = pre_downwash.alpha_rad - surface.downwash_angle_rad;
    assert!(final_alpha_geom < pre_downwash.alpha_rad);

    let reynolds = pre_downwash.section_airspeed_mps * effective[1].chord_m()
        / model.kinematic_viscosity_m2_s().unwrap();
    let expected_cl = model.aero_polar_families()[0]
        .family()
        .sample(reynolds, final_alpha_geom - surface.induced_alpha_rad)
        .coefficients
        .cl;
    assert_close(surface.surface_cl, expected_cl, 2.0e-13);

    let base = compute_section_kinematics(&stage, &effective[1], &env);
    let wake = propeller_slipstream(&model, &env, &output);
    let axial =
        base.air_relative_velocity_element_mps + wake.axis_body() * wake.induced_velocity_mps();
    let (sin_downwash, cos_downwash) = surface.downwash_angle_rad.sin_cos();
    let downwashed_axial = Vec3::new(
        cos_downwash.mul_add(axial.x, sin_downwash * axial.z),
        axial.y,
        (-sin_downwash).mul_add(axial.x, cos_downwash * axial.z),
    );
    let wrong_downwash_then_swirl =
        downwashed_axial + Vec3::new(0.0, 0.0, SWIRL_FACTOR * wake.induced_velocity_mps());
    let wrong_alpha = wrong_downwash_then_swirl
        .z
        .atan2(wrong_downwash_then_swirl.x);
    assert!((wrong_alpha - final_alpha_geom).abs() > 1.0e-4);
}

#[test]
fn repeated_swirl_evaluation_is_bit_deterministic_and_allocation_free() {
    let model = load(&configured(Some(SWIRL_FACTOR)));
    let effective = elements(&model);
    let stage = state(0.11);
    let env = environment();
    let expected = evaluate_aircraft_wrench(&stage, &effective, &model, 0.7, &env);
    for _ in 0..100 {
        assert_eq!(
            evaluate_aircraft_wrench(&stage, &effective, &model, 0.7, &env),
            expected
        );
    }

    let allocations = allocation_counter::measure(|| {
        for _ in 0..100 {
            std::hint::black_box(evaluate_aircraft_wrench(
                std::hint::black_box(&stage),
                std::hint::black_box(&effective),
                std::hint::black_box(&model),
                0.7,
                std::hint::black_box(&env),
            ));
        }
    });
    assert_eq!(allocations.count_total, 0, "{allocations:?}");
}
