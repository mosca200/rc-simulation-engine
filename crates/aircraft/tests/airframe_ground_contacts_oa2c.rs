use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::{AircraftModel, AircraftModelLoader};
use serde_json::{Value, json};
use sim_core::{
    AeroEnvironment, FlatGroundPlane, GroundCommand, GroundSurface, PilotInput, RigidBodyState,
    evaluate_airframe_ground_wrench, evaluate_ground_wrench,
};
use sim_math::{Orientation, Vec3, body_to_world};

fn production_value() -> Value {
    serde_json::from_str(include_str!("../../../models/acro_electric_01/model.json")).unwrap()
}

fn load(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

fn config(gravity_world_mps2: Vec3) -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        gravity_world_mps2,
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn supported_state(model: &AircraftModel) -> RigidBodyState {
    let gear = model.gear_contacts();
    let minimum_bottom = gear
        .iter()
        .map(|contact| contact.position_body_m.z + contact.wheel_radius_m)
        .fold(f64::INFINITY, f64::min);
    let maximum_bottom = gear
        .iter()
        .map(|contact| contact.position_body_m.z + contact.wheel_radius_m)
        .fold(f64::NEG_INFINITY, f64::max);
    let minimum_stiffness = gear
        .iter()
        .map(|contact| contact.stiffness_n_per_m)
        .fold(f64::INFINITY, f64::min);
    let weight = model.rigid_body().mass_kg() * 9.80665;
    let mut supported = minimum_bottom - weight / minimum_stiffness;
    let mut clear = maximum_bottom;
    for _ in 0..96 {
        let candidate = 0.5 * (supported + clear);
        let normal = gear
            .iter()
            .map(|contact| {
                contact.stiffness_n_per_m
                    * (contact.position_body_m.z + contact.wheel_radius_m - candidate).max(0.0)
            })
            .sum::<f64>();
        if normal > weight {
            supported = candidate;
        } else {
            clear = candidate;
        }
    }
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -0.5 * (supported + clear)),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[test]
fn normal_gear_stance_has_weight_on_wheels_without_airframe_support() {
    let model = load(&production_value());
    let state = supported_state(&model);
    let surface = GroundSurface::Flat(FlatGroundPlane::default());
    let gear = model.gear_contacts();
    let airframe = model
        .airframe_contacts()
        .iter()
        .map(|contact| contact.contact())
        .collect::<Vec<_>>();
    let gear_evaluation =
        evaluate_ground_wrench(&state, &gear, &surface, &GroundCommand::new(0.0, 0.0));
    let airframe_evaluation = evaluate_airframe_ground_wrench(&state, &airframe, &surface);
    assert!(gear_evaluation.weight_on_wheels());
    assert_eq!(airframe_evaluation.active_contacts, 0);
    assert_eq!(airframe_evaluation.force_body_n, Vec3::zeros());

    let mut simulation =
        AircraftSimulation::new(model, config(Vec3::new(0.0, 0.0, 9.80665)), state).unwrap();
    simulation.refresh_ground_diagnostics(GroundCommand::new(0.0, 0.0));
    assert!(simulation.last_ground_evaluation().weight_on_wheels());
    assert_eq!(
        simulation.last_airframe_ground_evaluation().active_contacts,
        0
    );
}

#[test]
fn belly_reaction_is_included_in_rk4_without_fabricated_gear() {
    let mut with_contact = production_value();
    with_contact["landing_gear"] = json!([]);
    let mut without_contact = with_contact.clone();
    without_contact["airframe_contacts"] = json!([]);
    let initial = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -0.05),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let mut protected =
        AircraftSimulation::new(load(&with_contact), config(Vec3::zeros()), initial).unwrap();
    let mut unprotected =
        AircraftSimulation::new(load(&without_contact), config(Vec3::zeros()), initial).unwrap();
    let idle = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let protected_snapshot = protected.step(&idle);
    let unprotected_snapshot = unprotected.step(&idle);

    assert!(protected.last_airframe_ground_evaluation().active_contacts > 0);
    assert_eq!(
        unprotected
            .last_airframe_ground_evaluation()
            .active_contacts,
        0
    );
    assert!(protected_snapshot.ground_contacts() > 0);
    assert!(protected_snapshot.total_ground_normal_force_n() > 0.0);
    assert!(
        protected_snapshot
            .rigid_body_state()
            .linear_velocity_world_mps
            .z
            < unprotected_snapshot
                .rigid_body_state()
                .linear_velocity_world_mps
                .z
    );
    assert!(!protected_snapshot.weight_on_wheels());
}

fn wing_strike_simulation() -> AircraftSimulation {
    let model = load(&production_value());
    let orientation = Orientation::from_axis_angle(&Vec3::x_axis(), 0.2);
    let right_tip = model
        .airframe_contacts()
        .iter()
        .find(|contact| contact.id() == "right-wing-tip")
        .unwrap()
        .contact();
    let tip_offset = body_to_world(&orientation, &right_tip.position_body_m);
    let initial = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, 0.005 - tip_offset.z),
        linear_velocity_world_mps: Vec3::new(4.0, 0.0, 5.0),
        orientation_world_from_body: orientation,
        angular_velocity_body_radps: Vec3::zeros(),
    };
    AircraftSimulation::new(model, config(Vec3::new(0.0, 0.0, 9.80665)), initial).unwrap()
}

#[test]
fn abnormal_tip_impact_is_bounded_finite_and_repeatable_at_500_hz() {
    let mut first = wing_strike_simulation();
    let mut second = wing_strike_simulation();
    let idle = PilotInput::new(0.0, 0.0, 0.0, 0.0);
    let mut maximum_structural_penetration = 0.0_f64;
    for _ in 0..1_000 {
        let first_snapshot = first.step(&idle);
        let second_snapshot = second.step(&idle);
        assert_eq!(first_snapshot, second_snapshot);
        assert_eq!(
            first.last_airframe_ground_evaluation(),
            second.last_airframe_ground_evaluation()
        );
        let rigid = first_snapshot.rigid_body_state();
        assert!(rigid.position_world_m.iter().all(|value| value.is_finite()));
        assert!(
            rigid
                .linear_velocity_world_mps
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            rigid
                .angular_velocity_body_radps
                .iter()
                .all(|value| value.is_finite())
        );
        for contact in &first.last_airframe_ground_evaluation().contacts {
            maximum_structural_penetration =
                maximum_structural_penetration.max(contact.penetration_m);
        }
    }
    assert!(
        maximum_structural_penetration < 0.15,
        "macroscopic penetration reached {maximum_structural_penetration} m"
    );
}
