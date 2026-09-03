//! M2.8F deterministic propeller gyroscopic reaction. Synthetic inputs only.

use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, evaluate_aerodynamic_wrench_with_propulsion,
    evaluate_aircraft_instantaneous, evaluate_aircraft_wrench,
};
use model::{AircraftModel, AircraftModelLoader};
use serde_json::{Value, json};
use sim_core::{
    AeroEnvironment, PilotInput, PropulsionOutput, RigidBodyState,
    evaluate_electric_propulsion_with_source,
};
use sim_math::{Orientation, Vec3};

const RHO: f64 = 1.225;
const INERTIA_KG_M2: f64 = 0.0035;

fn fixture(inertia: Option<f64>) -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_propeller_slipstream_v7.json"
    ))
    .unwrap();
    if let Some(inertia) = inertia {
        value["propulsion"]["propeller"]["propeller_rotational_inertia_kg_m2"] = json!(inertia);
    }
    value
}

fn load(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

fn environment() -> AeroEnvironment {
    AeroEnvironment::new(RHO, Vec3::zeros()).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(0.002, Vec3::zeros(), environment()).unwrap()
}

fn state(angular_velocity_body_radps: Vec3) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(8.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps,
    }
}

fn elements(model: &AircraftModel) -> Vec<sim_core::AeroElement> {
    model
        .aero_elements()
        .iter()
        .map(|element| *element.element())
        .collect()
}

fn raw_propulsion(
    model: &AircraftModel,
    stage: &RigidBodyState,
    throttle: f64,
) -> PropulsionOutput {
    let propulsion = model.propulsion().unwrap();
    evaluate_electric_propulsion_with_source(
        stage,
        throttle,
        propulsion.config(),
        &environment(),
        propulsion.coefficient_source(),
    )
}

fn evaluated_propulsion(
    model: &AircraftModel,
    stage: &RigidBodyState,
    throttle: f64,
) -> PropulsionOutput {
    *evaluate_aircraft_instantaneous(stage, &elements(model), model, throttle, &config())
        .propulsion()
        .unwrap()
}

fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
    assert!(
        (actual - expected).norm() <= tolerance,
        "actual={actual:?}, expected={expected:?}, tolerance={tolerance:.3e}"
    );
}

#[test]
fn zero_inertia_and_zero_cross_product_conditions_add_no_gyro() {
    let absent = load(&fixture(None));
    let explicit_zero = load(&fixture(Some(0.0)));
    let rotating = state(Vec3::new(0.0, 2.5, -0.7));
    assert_eq!(
        evaluate_aircraft_instantaneous(&rotating, &elements(&absent), &absent, 0.7, &config()),
        evaluate_aircraft_instantaneous(
            &rotating,
            &elements(&explicit_zero),
            &explicit_zero,
            0.7,
            &config(),
        )
    );

    let model = load(&fixture(Some(INERTIA_KG_M2)));
    let stopped = state(Vec3::new(0.0, 2.5, 0.0));
    let stopped_raw = raw_propulsion(&model, &stopped, 0.0);
    let stopped_actual = evaluated_propulsion(&model, &stopped, 0.0);
    assert_eq!(stopped_actual.shaft_speed_rad_s, 0.0);
    assert_eq!(stopped_actual.wrench_body, stopped_raw.wrench_body);

    for stage in [state(Vec3::zeros()), state(Vec3::new(3.0, 0.0, 0.0))] {
        let raw = raw_propulsion(&model, &stage, 0.7);
        let actual = evaluated_propulsion(&model, &stage, 0.7);
        assert_eq!(actual.wrench_body, raw.wrench_body);
    }
}

#[test]
fn orthogonal_body_rate_matches_analytic_moment_without_changing_force_or_reaction_torque() {
    let model = load(&fixture(Some(INERTIA_KG_M2)));
    let stage = state(Vec3::new(0.0, 2.5, 0.0));
    let raw = raw_propulsion(&model, &stage, 0.7);
    let actual = evaluated_propulsion(&model, &stage, 0.7);
    let angular_momentum = INERTIA_KG_M2 * raw.shaft_speed_rad_s;
    let expected_gyro =
        Vec3::new(angular_momentum, 0.0, 0.0).cross(&stage.angular_velocity_body_radps);

    assert_eq!(actual.thrust_n.to_bits(), raw.thrust_n.to_bits());
    assert_eq!(actual.force_prop_n, raw.force_prop_n);
    assert_eq!(
        actual.wrench_body.force_body_n,
        raw.wrench_body.force_body_n
    );
    assert_vec_close(
        actual.wrench_body.moment_body_nm - raw.wrench_body.moment_body_nm,
        expected_gyro,
        2.0e-13,
    );
    assert_eq!(
        actual.wrench_body.moment_body_nm.x.to_bits(),
        raw.wrench_body.moment_body_nm.x.to_bits()
    );
    assert_eq!(
        raw.wrench_body.moment_body_nm.x.to_bits(),
        (-raw.propeller_load_torque_nm).to_bits()
    );

    let effective = elements(&model);
    let complete = evaluate_aircraft_instantaneous(&stage, &effective, &model, 0.7, &config());
    let output = complete.propulsion().unwrap();
    let aero = evaluate_aerodynamic_wrench_with_propulsion(
        &stage,
        &effective,
        &model,
        &environment(),
        Some(output),
    );
    assert_eq!(
        complete.total_wrench().force_body_n,
        aero.force_body_n + output.wrench_body.force_body_n
    );
    assert_eq!(
        complete.total_wrench().moment_body_nm,
        aero.moment_body_nm + output.wrench_body.moment_body_nm
    );
}

#[test]
fn spin_direction_and_propeller_orientation_control_gyro_direction() {
    let stage = state(Vec3::new(0.0, 2.5, 0.0));
    let positive = load(&fixture(Some(INERTIA_KG_M2)));
    let positive_raw = raw_propulsion(&positive, &stage, 0.7);
    let positive_actual = evaluated_propulsion(&positive, &stage, 0.7);
    let positive_gyro =
        positive_actual.wrench_body.moment_body_nm - positive_raw.wrench_body.moment_body_nm;

    let mut negative_value = fixture(Some(INERTIA_KG_M2));
    negative_value["propulsion"]["propeller"]["spin_direction"] = json!("negative_about_local_x");
    let negative = load(&negative_value);
    let negative_raw = raw_propulsion(&negative, &stage, 0.7);
    let negative_actual = evaluated_propulsion(&negative, &stage, 0.7);
    let negative_gyro =
        negative_actual.wrench_body.moment_body_nm - negative_raw.wrench_body.moment_body_nm;
    assert_eq!(
        positive_raw.shaft_speed_rad_s.to_bits(),
        negative_raw.shaft_speed_rad_s.to_bits()
    );
    assert_vec_close(negative_gyro, -positive_gyro, 2.0e-13);

    let mut oriented_value = fixture(Some(INERTIA_KG_M2));
    let half_sqrt_two = std::f64::consts::FRAC_1_SQRT_2;
    oriented_value["propulsion"]["propeller"]["orientation_body_from_prop_wxyz"] =
        json!([half_sqrt_two, 0.0, 0.0, half_sqrt_two]);
    let oriented = load(&oriented_value);
    let oriented_stage = state(Vec3::new(0.0, 0.0, 1.75));
    let oriented_raw = raw_propulsion(&oriented, &oriented_stage, 0.7);
    let oriented_actual = evaluated_propulsion(&oriented, &oriented_stage, 0.7);
    let expected = Vec3::new(0.0, INERTIA_KG_M2 * oriented_raw.shaft_speed_rad_s, 0.0)
        .cross(&oriented_stage.angular_velocity_body_radps);
    assert_vec_close(
        oriented_actual.wrench_body.moment_body_nm - oriented_raw.wrench_body.moment_body_nm,
        expected,
        3.0e-13,
    );
    assert!(expected.x > 0.0);
}

#[test]
fn gyroscopic_rk4_is_bit_deterministic_and_allocation_free() {
    let model = load(&fixture(Some(INERTIA_KG_M2)));
    let initial = state(Vec3::new(0.15, 1.1, -0.4));
    let mut first = AircraftSimulation::new(model.clone(), config(), initial).unwrap();
    let mut second = AircraftSimulation::new(model.clone(), config(), initial).unwrap();
    let input = PilotInput::new(0.0, 0.2, 0.0, 0.7);
    for _ in 0..100 {
        assert_eq!(first.step(&input), second.step(&input));
    }

    let effective = elements(&model);
    std::hint::black_box(evaluate_aircraft_wrench(
        &initial,
        &effective,
        &model,
        0.7,
        &environment(),
    ));
    let env = environment();
    let allocations = allocation_counter::measure(|| {
        for _ in 0..100 {
            std::hint::black_box(evaluate_aircraft_wrench(
                std::hint::black_box(&initial),
                std::hint::black_box(&effective),
                std::hint::black_box(&model),
                0.7,
                std::hint::black_box(&env),
            ));
        }
    });
    assert_eq!(allocations.count_total, 0, "{allocations:?}");
}
