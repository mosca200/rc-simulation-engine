use aircraft::{
    AircraftAeroElementOutput, AircraftSimulation, AircraftSimulationConfig,
    evaluate_aircraft_aero_element, evaluate_aircraft_wrench,
};
use model::{AircraftModel, AircraftModelLoader, RuntimeAeroPolarBinding};
use sim_core::{
    AeroEnvironment, BodyWrench, PilotInput, PolarSample, PolarTable, RigidBodyState,
    Rk4Integrator, evaluate_aero_element, evaluate_derivative,
};
use sim_math::{Orientation, Vec3};

const SYNTHETIC_V3: &str =
    include_str!("../../../tests/fixtures/synthetic_non_reference_reynolds_v3.json");

fn model() -> AircraftModel {
    AircraftModelLoader::from_json_str(SYNTHETIC_V3).unwrap()
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::zeros(),
        linear_velocity_world_mps: Vec3::new(40.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.1,
        Vec3::zeros(),
        AeroEnvironment::new(1.0, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

#[test]
fn m2_3c_13_rk4_uses_stage_local_reynolds_instead_of_committed_state_reynolds() {
    let model = model();
    let config = config();
    let initial = initial_state();
    let effective_elements: Vec<_> = model
        .aero_elements()
        .iter()
        .map(|runtime| *runtime.element())
        .collect();

    let mut stage_reynolds = [0.0; 4];
    let mut stage_count = 0;
    let expected = Rk4Integrator::step(&initial, config.dt_s(), |stage_state| {
        let output = evaluate_aircraft_aero_element(
            stage_state,
            &effective_elements[0],
            &model.aero_elements()[0],
            &model,
            config.aero_environment(),
        );
        let AircraftAeroElementOutput::ReynoldsFamily(output) = output else {
            panic!("fixture element must use the Reynolds family")
        };
        stage_reynolds[stage_count] = output.local_reynolds;
        stage_count += 1;
        let wrench = evaluate_aircraft_wrench(
            stage_state,
            &effective_elements,
            &model,
            0.0,
            config.aero_environment(),
        );
        evaluate_derivative(
            stage_state,
            model.rigid_body(),
            &wrench,
            config.gravity_world_mps2(),
        )
    });
    assert_eq!(stage_count, 4);
    assert!(stage_reynolds[1] != stage_reynolds[0]);
    assert!(stage_reynolds[2] != stage_reynolds[0]);
    assert!(stage_reynolds[3] != stage_reynolds[0]);

    let mut simulation = AircraftSimulation::new(model.clone(), config, initial).unwrap();
    let actual = *simulation.step(&PilotInput::neutral()).rigid_body_state();
    assert_eq!(actual, expected);

    let initial_reynolds_output = evaluate_aircraft_aero_element(
        &initial,
        &effective_elements[0],
        &model.aero_elements()[0],
        &model,
        config.aero_environment(),
    );
    let frozen_coefficients = initial_reynolds_output.aero().coefficients;
    let frozen_table = PolarTable::new(vec![
        PolarSample {
            alpha_rad: -1.0,
            cl: frozen_coefficients.cl,
            cd: frozen_coefficients.cd,
            cm: frozen_coefficients.cm,
        },
        PolarSample {
            alpha_rad: 1.0,
            cl: frozen_coefficients.cl,
            cd: frozen_coefficients.cd,
            cm: frozen_coefficients.cm,
        },
    ])
    .unwrap();
    let frozen = Rk4Integrator::step(&initial, config.dt_s(), |stage_state| {
        let mut wrench = BodyWrench::zero();
        for (effective, runtime) in effective_elements.iter().zip(model.aero_elements()) {
            let output = match runtime.polar_binding() {
                RuntimeAeroPolarBinding::ReynoldsFamily { .. } => evaluate_aero_element(
                    stage_state,
                    effective,
                    config.aero_environment(),
                    &frozen_table,
                ),
                RuntimeAeroPolarBinding::Polar { polar_index } => evaluate_aero_element(
                    stage_state,
                    effective,
                    config.aero_environment(),
                    model.aero_polars()[polar_index].table(),
                ),
            };
            wrench.force_body_n += output.wrench_body.force_body_n;
            wrench.moment_body_nm += output.wrench_body.moment_body_nm;
        }
        evaluate_derivative(
            stage_state,
            model.rigid_body(),
            &wrench,
            config.gravity_world_mps2(),
        )
    });
    assert_ne!(
        actual, frozen,
        "freezing Reynolds at k1 must change the RK4 result"
    );
}

#[test]
fn m2_3c_20_repeated_reynolds_aware_runs_are_bit_deterministic() {
    let model = model();
    let config = config();
    let initial = initial_state();
    let mut first = AircraftSimulation::new(model.clone(), config, initial).unwrap();
    let mut second = AircraftSimulation::new(model, config, initial).unwrap();
    for step in 0..300 {
        let phase = f64::from(step) / 299.0;
        let input = PilotInput::new(0.2 * phase, -0.1 * phase, 0.05 * phase, 0.0);
        assert_eq!(first.step(&input), second.step(&input));
    }
    assert_eq!(first.state(), second.state());
}
