use crate::AircraftSimulationConfig;
use model::{AircraftModel, ControlActuator, RuntimeAeroElement, RuntimeAeroPolarBinding};
use sim_core::{
    AeroElement, AeroElementOutput, BodyWrench, ControlSurfacePositions, ControlSystemState,
    PilotInput, PropulsionOutput, ReynoldsAeroElementOutput, RigidBodyDerivative, RigidBodyState,
    Rk4Integrator, StateError, advance_controls, evaluate_aero_element, evaluate_derivative,
    evaluate_electric_propulsion_with_source, evaluate_reynolds_aero_element,
};
use sim_math::{Orientation, Vec3};
use thiserror::Error;

/// All mutable aircraft dynamics, deliberately separate from the immutable model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftState {
    rigid_body: RigidBodyState,
    controls: ControlSystemState,
}

impl AircraftState {
    #[must_use]
    pub const fn rigid_body(&self) -> &RigidBodyState {
        &self.rigid_body
    }

    #[must_use]
    pub const fn controls(&self) -> &ControlSystemState {
        &self.controls
    }
}

/// Allocation-free, by-value observation of the committed post-step state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftSnapshot {
    step_index: u64,
    sim_time_s: f64,
    rigid_body_state: RigidBodyState,
    control_surface_positions: ControlSurfacePositions,
}

/// Per-element aerodynamic output, preserving Reynolds diagnostics without hot-path allocation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AircraftAeroElementOutput<'a> {
    Polar(AeroElementOutput),
    ReynoldsFamily(ReynoldsAeroElementOutput<'a>),
}

/// Instantaneous aircraft physics evaluated through the same path used by one RK4 stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftInstantaneousEvaluation {
    total_wrench: BodyWrench,
    derivative: RigidBodyDerivative,
    propulsion: Option<PropulsionOutput>,
}

impl AircraftInstantaneousEvaluation {
    #[must_use]
    pub const fn total_wrench(&self) -> &BodyWrench {
        &self.total_wrench
    }

    #[must_use]
    pub const fn derivative(&self) -> &RigidBodyDerivative {
        &self.derivative
    }

    #[must_use]
    pub const fn propulsion(&self) -> Option<&PropulsionOutput> {
        self.propulsion.as_ref()
    }
}

impl AircraftAeroElementOutput<'_> {
    #[must_use]
    pub const fn aero(&self) -> &AeroElementOutput {
        match self {
            Self::Polar(output) => output,
            Self::ReynoldsFamily(output) => &output.aero,
        }
    }

    #[must_use]
    pub const fn reynolds(&self) -> Option<&ReynoldsAeroElementOutput<'_>> {
        match self {
            Self::Polar(_) => None,
            Self::ReynoldsFamily(output) => Some(output),
        }
    }
}

impl AircraftSnapshot {
    #[must_use]
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    #[must_use]
    pub const fn sim_time_s(&self) -> f64 {
        self.sim_time_s
    }

    #[must_use]
    pub const fn rigid_body_state(&self) -> &RigidBodyState {
        &self.rigid_body_state
    }

    #[must_use]
    pub const fn control_surface_positions(&self) -> &ControlSurfacePositions {
        &self.control_surface_positions
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum AircraftSimulationError {
    #[error("invalid initial rigid-body state: {0}")]
    InvalidInitialState(#[from] StateError),
    #[error("control-surface binding {binding_index} can overflow its effective deflection")]
    NonFiniteControlSurfaceDeflection { binding_index: usize },
}

/// Mutable single-threaded owner of the assembled aircraft's dynamic state.
#[derive(Debug, Clone)]
pub struct AircraftSimulation {
    model: AircraftModel,
    config: AircraftSimulationConfig,
    state: AircraftState,
    effective_aero_elements: Vec<AeroElement>,
    step_index: u64,
}

impl AircraftSimulation {
    pub fn new(
        model: AircraftModel,
        config: AircraftSimulationConfig,
        initial_rigid_state: RigidBodyState,
    ) -> Result<Self, AircraftSimulationError> {
        initial_rigid_state.validate()?;
        for (binding_index, binding) in model.control_surface_bindings().iter().enumerate() {
            let servo = match binding.actuator() {
                ControlActuator::Aileron => model.controls().actuators().aileron(),
                ControlActuator::Elevator => model.controls().actuators().elevator(),
                ControlActuator::Rudder => model.controls().actuators().rudder(),
            };
            let minimum_deflection =
                binding.deflection_gain() * (servo.min_angle_rad() - servo.neutral_angle_rad());
            let maximum_deflection =
                binding.deflection_gain() * (servo.max_angle_rad() - servo.neutral_angle_rad());
            if !minimum_deflection.is_finite() || !maximum_deflection.is_finite() {
                return Err(AircraftSimulationError::NonFiniteControlSurfaceDeflection {
                    binding_index,
                });
            }
        }
        let effective_aero_elements = model
            .aero_elements()
            .iter()
            .map(|runtime_element| *runtime_element.element())
            .collect();
        let controls = ControlSystemState::neutral(model.controls());
        Ok(Self {
            model,
            config,
            state: AircraftState {
                rigid_body: initial_rigid_state,
                controls,
            },
            effective_aero_elements,
            step_index: 0,
        })
    }

    /// Advances controls once, holds actuators/throttle fixed, then evaluates all four RK4 stages.
    #[must_use]
    pub fn step(&mut self, input: &PilotInput) -> AircraftSnapshot {
        self.step_with_stage_observer(input, |_, _, _| {})
    }

    #[must_use]
    pub const fn model(&self) -> &AircraftModel {
        &self.model
    }

    #[must_use]
    pub const fn config(&self) -> &AircraftSimulationConfig {
        &self.config
    }

    #[must_use]
    pub const fn state(&self) -> &AircraftState {
        &self.state
    }

    #[must_use]
    pub fn effective_aero_elements(&self) -> &[AeroElement] {
        &self.effective_aero_elements
    }

    #[must_use]
    pub const fn step_index(&self) -> u64 {
        self.step_index
    }

    #[must_use]
    pub fn sim_time_s(&self) -> f64 {
        self.step_index as f64 * self.config.dt_s()
    }

    fn update_effective_aero_elements(&mut self, positions: &ControlSurfacePositions) {
        apply_control_surface_positions(&self.model, positions, &mut self.effective_aero_elements);
    }

    fn step_with_stage_observer<F>(
        &mut self,
        input: &PilotInput,
        mut observe_stage: F,
    ) -> AircraftSnapshot
    where
        F: FnMut(&RigidBodyState, &[AeroElement], &AircraftInstantaneousEvaluation),
    {
        let control_surface_positions = advance_controls(
            &mut self.state.controls,
            self.model.controls(),
            input,
            self.config.dt_s(),
        );
        self.update_effective_aero_elements(&control_surface_positions);

        let initial_state = self.state.rigid_body;
        let model = &self.model;
        let effective_aero_elements = &self.effective_aero_elements;
        let throttle = control_surface_positions.throttle();
        self.state.rigid_body =
            Rk4Integrator::step(&initial_state, self.config.dt_s(), |stage_state| {
                let evaluation = evaluate_aircraft_instantaneous(
                    stage_state,
                    effective_aero_elements,
                    model,
                    throttle,
                    &self.config,
                );
                debug_assert_eq!(
                    evaluation.propulsion.is_some(),
                    model.propulsion().is_some()
                );
                observe_stage(stage_state, effective_aero_elements, &evaluation);
                *evaluation.derivative()
            });
        self.step_index += 1;
        debug_assert!(self.state.rigid_body.validate().is_ok());

        AircraftSnapshot {
            step_index: self.step_index,
            sim_time_s: self.sim_time_s(),
            rigid_body_state: self.state.rigid_body,
            control_surface_positions,
        }
    }
}

/// Applies physical actuator positions to an existing model-ordered element buffer.
pub fn apply_control_surface_positions(
    model: &AircraftModel,
    positions: &ControlSurfacePositions,
    effective_aero_elements: &mut [AeroElement],
) {
    assert_eq!(effective_aero_elements.len(), model.aero_elements().len());
    for binding in model.control_surface_bindings() {
        let servo_angle_rad = match binding.actuator() {
            ControlActuator::Aileron => positions.aileron_angle_rad(),
            ControlActuator::Elevator => positions.elevator_angle_rad(),
            ControlActuator::Rudder => positions.rudder_angle_rad(),
        };
        let servo_neutral_angle_rad = match binding.actuator() {
            ControlActuator::Aileron => model.controls().actuators().aileron().neutral_angle_rad(),
            ControlActuator::Elevator => {
                model.controls().actuators().elevator().neutral_angle_rad()
            }
            ControlActuator::Rudder => model.controls().actuators().rudder().neutral_angle_rad(),
        };
        let surface_deflection_rad =
            binding.deflection_gain() * (servo_angle_rad - servo_neutral_angle_rad);
        let base = model.aero_elements()[binding.element_index()].element();
        effective_aero_elements[binding.element_index()] =
            deflected_aero_element(base, surface_deflection_rad);
    }
}

/// Builds model-ordered effective elements and applies the supplied steady actuator positions.
#[must_use]
pub fn effective_aero_elements_for_positions(
    model: &AircraftModel,
    positions: &ControlSurfacePositions,
) -> Vec<AeroElement> {
    let mut elements = model
        .aero_elements()
        .iter()
        .map(|runtime| *runtime.element())
        .collect::<Vec<_>>();
    apply_control_surface_positions(model, positions, &mut elements);
    elements
}

/// Applies a hinge rotation in the element's local frame: `base * rotation_about_local_Y`.
#[must_use]
pub fn deflected_aero_element(base: &AeroElement, surface_deflection_rad: f64) -> AeroElement {
    debug_assert!(surface_deflection_rad.is_finite());
    if surface_deflection_rad == 0.0 {
        return *base;
    }
    let delta_orientation = Orientation::from_axis_angle(&Vec3::y_axis(), surface_deflection_rad);
    let effective_orientation = base.orientation_body_from_element() * delta_orientation;
    AeroElement::new(
        *base.position_body_m(),
        effective_orientation,
        base.area_m2(),
        base.chord_m(),
    )
    .expect("a validated element remains valid after a finite unit hinge rotation")
}

/// Re-evaluates the complete non-gravity body wrench for one rigid-body stage state.
#[must_use]
pub fn evaluate_aircraft_wrench(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    throttle: f64,
    environment: &sim_core::AeroEnvironment,
) -> BodyWrench {
    evaluate_stage(
        stage_state,
        effective_aero_elements,
        model,
        throttle,
        environment,
    )
    .total_wrench
}

/// Evaluates wrench and rigid-body derivative without advancing controls or integrating time.
#[must_use]
pub fn evaluate_aircraft_instantaneous(
    state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    throttle: f64,
    config: &AircraftSimulationConfig,
) -> AircraftInstantaneousEvaluation {
    let stage = evaluate_stage(
        state,
        effective_aero_elements,
        model,
        throttle,
        config.aero_environment(),
    );
    let derivative = evaluate_derivative(
        state,
        model.rigid_body(),
        &stage.total_wrench,
        config.gravity_world_mps2(),
    );
    AircraftInstantaneousEvaluation {
        total_wrench: stage.total_wrench,
        derivative,
        propulsion: stage.propulsion,
    }
}

/// Aggregates every S4 element wrench, preserving the model's declaration order.
#[must_use]
pub fn evaluate_aerodynamic_wrench(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
) -> BodyWrench {
    assert_eq!(effective_aero_elements.len(), model.aero_elements().len());
    let mut total_wrench = BodyWrench::zero();
    for (effective_element, runtime_element) in
        effective_aero_elements.iter().zip(model.aero_elements())
    {
        let output = evaluate_aircraft_aero_element(
            stage_state,
            effective_element,
            runtime_element,
            model,
            environment,
        );
        let output = output.aero();
        total_wrench.force_body_n += output.wrench_body.force_body_n;
        total_wrench.moment_body_nm += output.wrench_body.moment_body_nm;
    }
    total_wrench
}

/// Evaluates one resolved model element and exposes Reynolds diagnostics when applicable.
#[must_use]
pub fn evaluate_aircraft_aero_element<'a>(
    stage_state: &RigidBodyState,
    effective_element: &AeroElement,
    runtime_element: &RuntimeAeroElement,
    model: &'a AircraftModel,
    environment: &sim_core::AeroEnvironment,
) -> AircraftAeroElementOutput<'a> {
    match runtime_element.polar_binding() {
        RuntimeAeroPolarBinding::Polar { polar_index } => {
            AircraftAeroElementOutput::Polar(evaluate_aero_element(
                stage_state,
                effective_element,
                environment,
                model.aero_polars()[polar_index].table(),
            ))
        }
        RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
            let viscosity = model
                .kinematic_viscosity_m2_s()
                .expect("Reynolds-family bindings exist only in schema-v3/v4 models");
            AircraftAeroElementOutput::ReynoldsFamily(evaluate_reynolds_aero_element(
                stage_state,
                effective_element,
                environment,
                model.aero_polar_families()[family_index].family(),
                viscosity,
            ))
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct StageEvaluation {
    total_wrench: BodyWrench,
    propulsion: Option<PropulsionOutput>,
}

fn evaluate_stage(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    throttle: f64,
    environment: &sim_core::AeroEnvironment,
) -> StageEvaluation {
    let mut total_wrench =
        evaluate_aerodynamic_wrench(stage_state, effective_aero_elements, model, environment);

    let propulsion = model.propulsion().map(|runtime_propulsion| {
        let output = evaluate_electric_propulsion_with_source(
            stage_state,
            throttle,
            runtime_propulsion.config(),
            environment,
            runtime_propulsion.coefficient_source(),
        );
        total_wrench.force_body_n += output.wrench_body.force_body_n;
        total_wrench.moment_body_nm += output.wrench_body.moment_body_nm;
        output
    });

    StageEvaluation {
        total_wrench,
        propulsion,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AircraftSimulationConfig;
    use model::AircraftModelLoader;
    use sim_core::{
        AeroEnvironment, PilotInput, RigidBodyState, evaluate_aero_element,
        evaluate_electric_propulsion,
    };
    use sim_math::{Orientation, Vec3};
    use std::f64::consts::FRAC_PI_2;

    const ROOT_TEMPLATE: &str = r#"
    {
      "schema_version": 1,
      "model_id": "synthetic-aircraft",
      "display_name": "Synthetic Aircraft",
      "rigid_body": {
        "mass_kg": 2.0,
        "inertia_body_kg_m2": [[0.2, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.4]]
      },
      "aerodynamics": {
        "polars": [{
          "id": "symmetric",
          "samples": [
            {"alpha_rad": -0.6, "cl": -1.0, "cd": 0.08, "cm": 0.0},
            {"alpha_rad":  0.0, "cl":  0.0, "cd": 0.02, "cm": 0.0},
            {"alpha_rad":  0.6, "cl":  1.0, "cd": 0.08, "cm": 0.0}
          ]
        }],
        "elements": [__ELEMENTS__]
      },
      "controls": {
        "response": {
          "roll":  {"rate": 1.0, "expo": 0.0},
          "pitch": {"rate": 1.0, "expo": 0.0},
          "yaw":   {"rate": 1.0, "expo": 0.0}
        },
        "servos": {
          "aileron":  {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 10.0, "reversed": false},
          "elevator": {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 10.0, "reversed": false},
          "rudder":   {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 10.0, "reversed": false}
        }
      },
      "control_surface_bindings": [__BINDINGS__],
      "propulsion": __PROPULSION__,
      "presentation": null
    }
    "#;

    const PROPULSION: &str = r#"{
      "battery": {"open_circuit_voltage_v": 16.8, "internal_resistance_ohm": 0.035},
      "motor": {"kv_rpm_per_v": 900.0, "winding_resistance_ohm": 0.045, "no_load_current_a": 1.2},
      "propeller": {
        "position_body_m": [0.4, 0.0, 0.0],
        "orientation_body_from_prop_wxyz": [1.0, 0.0, 0.0, 0.0],
        "diameter_m": 0.28,
        "spin_direction": "positive_about_local_x"
      },
      "coefficient_table": {"samples": [
        {"advance_ratio_j": -0.25, "ct": 0.135, "cq": 0.019},
        {"advance_ratio_j": 0.0, "ct": 0.125, "cq": 0.018},
        {"advance_ratio_j": 0.5, "ct": 0.09, "cq": 0.013},
        {"advance_ratio_j": 1.0, "ct": 0.04, "cq": 0.007},
        {"advance_ratio_j": 1.5, "ct": 0.0, "cq": 0.002}
      ]}
    }"#;

    fn element(id: &str, position: [f64; 3], orientation: [f64; 4], area: f64) -> String {
        format!(
            r#"{{"id":"{id}","position_body_m":[{},{},{}],"orientation_body_from_element_wxyz":[{},{},{},{}],"area_m2":{area},"chord_m":0.25,"polar_id":"symmetric"}}"#,
            position[0],
            position[1],
            position[2],
            orientation[0],
            orientation[1],
            orientation[2],
            orientation[3]
        )
    }

    fn binding(id: &str, element_id: &str, actuator: &str, gain: f64) -> String {
        format!(
            r#"{{"id":"{id}","element_id":"{element_id}","actuator":"{actuator}","deflection_gain":{gain}}}"#
        )
    }

    fn load_model(elements: &[String], bindings: &[String], propulsion: bool) -> AircraftModel {
        let json = ROOT_TEMPLATE
            .replace("__ELEMENTS__", &elements.join(","))
            .replace("__BINDINGS__", &bindings.join(","))
            .replace(
                "__PROPULSION__",
                if propulsion { PROPULSION } else { "null" },
            );
        AircraftModelLoader::from_json_str(&json).unwrap()
    }

    fn identity_element(id: &str, position: [f64; 3], area: f64) -> String {
        element(id, position, [1.0, 0.0, 0.0, 0.0], area)
    }

    fn state_with_velocity(velocity: Vec3) -> RigidBodyState {
        RigidBodyState {
            position_world_m: Vec3::new(0.0, 0.0, -100.0),
            linear_velocity_world_mps: velocity,
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        }
    }

    fn config(dt_s: f64, density: f64, gravity: Vec3) -> AircraftSimulationConfig {
        AircraftSimulationConfig::new(
            dt_s,
            gravity,
            AeroEnvironment::new(density, Vec3::zeros()).unwrap(),
        )
        .unwrap()
    }

    fn assert_vec_close(actual: Vec3, expected: Vec3, tolerance: f64) {
        assert!(
            (actual - expected).norm() <= tolerance,
            "actual={actual:?}, expected={expected:?}, tolerance={tolerance:e}"
        );
    }

    fn assert_finite(state: &RigidBodyState) {
        assert!(state.validate().is_ok(), "invalid state: {state:?}");
    }

    #[test]
    fn positive_local_y_deflection_produces_positive_element_alpha() {
        let base = AeroElement::new(Vec3::zeros(), Orientation::identity(), 0.5, 0.25).unwrap();
        let effective = deflected_aero_element(&base, 0.2);
        let model = load_model(
            &[identity_element("surface", [0.0, 0.0, 0.0], 0.5)],
            &[],
            false,
        );
        let output = evaluate_aero_element(
            &state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
            &effective,
            &AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
            model.aero_polars()[0].table(),
        );
        assert!(output.alpha_rad > 0.0, "alpha={}", output.alpha_rad);
    }

    #[test]
    fn as7_servo_neutral_preserves_base_orientation_bit_exactly() {
        let elements = [identity_element("elevator", [-1.0, 0.0, 0.0], 0.2)];
        let bindings = [binding("elevator-binding", "elevator", "elevator", -1.0)];
        let json = ROOT_TEMPLATE
            .replace("__ELEMENTS__", &elements.join(","))
            .replace("__BINDINGS__", &bindings.join(","))
            .replace("__PROPULSION__", "null")
            .replace("\"neutral_angle_rad\": 0.0", "\"neutral_angle_rad\": 0.17");
        let model = AircraftModelLoader::from_json_str(&json).unwrap();
        assert_eq!(
            model.controls().actuators().elevator().neutral_angle_rad(),
            0.17
        );
        let base = *model.aero_elements()[0].element();
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
        let _ = simulation.step(&PilotInput::neutral());
        assert_eq!(simulation.effective_aero_elements()[0], base);
    }

    #[test]
    fn as8_nonidentity_base_composes_before_local_y_deflection() {
        let base = AeroElement::new(
            Vec3::new(0.2, -0.4, 0.1),
            Orientation::from_scaled_axis(Vec3::new(0.35, -0.18, 0.27)),
            0.3,
            0.2,
        )
        .unwrap();
        let deflection = 0.31;
        let effective = deflected_aero_element(&base, deflection);
        let expected = base.orientation_body_from_element()
            * Orientation::from_axis_angle(&Vec3::y_axis(), deflection);
        let wrong = Orientation::from_axis_angle(&Vec3::y_axis(), deflection)
            * base.orientation_body_from_element();
        for axis in [Vec3::x(), Vec3::z()] {
            assert_vec_close(
                effective
                    .orientation_body_from_element()
                    .transform_vector(&axis),
                expected.transform_vector(&axis),
                2.0e-15,
            );
            assert!(
                (effective
                    .orientation_body_from_element()
                    .transform_vector(&axis)
                    - wrong.transform_vector(&axis))
                .norm()
                    > 1.0e-3
            );
        }
    }

    #[test]
    fn as9_opposite_aileron_gains_produce_opposite_geometry() {
        let elements = [
            identity_element("left", [0.0, -0.8, 0.0], 0.2),
            identity_element("right", [0.0, 0.8, 0.0], 0.2),
        ];
        let bindings = [
            binding("left-binding", "left", "aileron", 1.0),
            binding("right-binding", "right", "aileron", -1.0),
        ];
        let model = load_model(&elements, &bindings, false);
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
        let _ = simulation.step(&PilotInput::new(1.0, 0.0, 0.0, 0.0));
        let left_x = simulation.effective_aero_elements()[0]
            .orientation_body_from_element()
            .transform_vector(&Vec3::x());
        let right_x = simulation.effective_aero_elements()[1]
            .orientation_body_from_element()
            .transform_vector(&Vec3::x());
        assert!((left_x.x - right_x.x).abs() < 1.0e-15);
        assert!((left_x.z + right_x.z).abs() < 1.0e-15);
        assert!(left_x.z * right_x.z < 0.0);
    }

    #[test]
    fn as10_unbound_fixed_element_does_not_move() {
        let elements = [
            identity_element("controlled", [0.0, 0.8, 0.0], 0.2),
            element(
                "fixed",
                [0.1, -0.4, 0.05],
                [0.5_f64.sqrt(), 0.5_f64.sqrt(), 0.0, 0.0],
                0.3,
            ),
        ];
        let bindings = [binding("controlled-binding", "controlled", "aileron", 1.0)];
        let model = load_model(&elements, &bindings, false);
        let fixed = *model.aero_elements()[1].element();
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
        for _ in 0..20 {
            let _ = simulation.step(&PilotInput::new(1.0, 0.0, 0.0, 0.0));
        }
        assert_eq!(simulation.effective_aero_elements()[1], fixed);
    }

    #[test]
    fn as11_two_element_aero_aggregate_matches_manual_sum() {
        let model = load_model(
            &[
                identity_element("left", [0.1, -0.7, 0.0], 0.3),
                identity_element("right", [-0.2, 0.9, 0.1], 0.45),
            ],
            &[],
            false,
        );
        let simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(23.0, 0.7, 2.5)),
        )
        .unwrap();
        let state = simulation.state().rigid_body();
        let environment = simulation.config().aero_environment();
        let mut expected = BodyWrench::zero();
        for (element, runtime) in simulation
            .effective_aero_elements()
            .iter()
            .zip(simulation.model().aero_elements())
        {
            let output = evaluate_aero_element(
                state,
                element,
                environment,
                simulation.model().aero_polars()[runtime.polar_index()].table(),
            );
            expected.force_body_n += output.wrench_body.force_body_n;
            expected.moment_body_nm += output.wrench_body.moment_body_nm;
        }
        let actual = evaluate_aerodynamic_wrench(
            state,
            simulation.effective_aero_elements(),
            simulation.model(),
            environment,
        );
        assert_eq!(actual, expected);
    }

    #[test]
    fn as12_propulsion_aggregation_is_aero_plus_propulsion() {
        let model = load_model(&[identity_element("wing", [0.0, 0.0, 0.0], 0.4)], &[], true);
        let simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(18.0, 0.0, 1.0)),
        )
        .unwrap();
        let state = simulation.state().rigid_body();
        let environment = simulation.config().aero_environment();
        let aero = evaluate_aerodynamic_wrench(
            state,
            simulation.effective_aero_elements(),
            simulation.model(),
            environment,
        );
        let runtime_propulsion = simulation.model().propulsion().unwrap();
        let prop = evaluate_electric_propulsion(
            state,
            0.7,
            runtime_propulsion.config(),
            environment,
            runtime_propulsion.coefficient_table(),
        );
        let total = evaluate_aircraft_wrench(
            state,
            simulation.effective_aero_elements(),
            simulation.model(),
            0.7,
            environment,
        );
        assert_eq!(
            total.force_body_n,
            aero.force_body_n + prop.wrench_body.force_body_n
        );
        assert_eq!(
            total.moment_body_nm,
            aero.moment_body_nm + prop.wrench_body.moment_body_nm
        );
    }

    #[test]
    fn as13_glider_adds_no_propulsion_wrench() {
        let model = load_model(
            &[identity_element("wing", [0.0, 0.0, 0.0], 0.4)],
            &[],
            false,
        );
        let simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(18.0, 0.0, 1.0)),
        )
        .unwrap();
        let aero = evaluate_aerodynamic_wrench(
            simulation.state().rigid_body(),
            simulation.effective_aero_elements(),
            simulation.model(),
            simulation.config().aero_environment(),
        );
        let total = evaluate_aircraft_wrench(
            simulation.state().rigid_body(),
            simulation.effective_aero_elements(),
            simulation.model(),
            1.0,
            simulation.config().aero_environment(),
        );
        assert_eq!(total, aero);
    }

    #[test]
    fn as14_gravity_is_not_double_counted() {
        let model = load_model(&[], &[], false);
        let dt_s = 0.01;
        let gravity = Vec3::new(0.0, 0.0, 9.80665);
        let mut simulation = AircraftSimulation::new(
            model,
            config(dt_s, 0.0, gravity),
            state_with_velocity(Vec3::zeros()),
        )
        .unwrap();
        let snapshot = simulation.step(&PilotInput::neutral());
        assert_vec_close(
            snapshot.rigid_body_state().linear_velocity_world_mps,
            gravity * dt_s,
            2.0e-17,
        );
        assert_vec_close(
            snapshot.rigid_body_state().position_world_m,
            Vec3::new(0.0, 0.0, -100.0) + gravity * (0.5 * dt_s * dt_s),
            2.0e-14,
        );
    }

    #[test]
    fn as15_controls_advance_exactly_once_per_step() {
        let model = load_model(
            &[identity_element("aileron", [0.0, 0.5, 0.0], 0.2)],
            &[binding("aileron-binding", "aileron", "aileron", 1.0)],
            false,
        );
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(15.0, 0.0, 0.0)),
        )
        .unwrap();
        let snapshot = simulation.step(&PilotInput::new(1.0, 0.0, 0.0, 0.0));
        assert_eq!(
            snapshot.control_surface_positions().aileron_angle_rad(),
            0.02
        );
        assert_eq!(
            simulation
                .state()
                .controls()
                .actuators()
                .aileron()
                .angle_rad(),
            0.02
        );
    }

    #[test]
    fn as16_all_rk4_stages_share_the_same_effective_orientation() {
        let model = load_model(
            &[identity_element("elevator", [-1.0, 0.0, 0.0], 0.2)],
            &[binding("elevator-binding", "elevator", "elevator", -1.0)],
            false,
        );
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.01, 1.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
        let mut observed_axes = [Vec3::zeros(); 4];
        let mut count = 0;
        let _ = simulation.step_with_stage_observer(
            &PilotInput::new(0.0, 1.0, 0.0, 0.0),
            |_, elements, _| {
                observed_axes[count] = elements[0]
                    .orientation_body_from_element()
                    .transform_vector(&Vec3::x());
                count += 1;
            },
        );
        assert_eq!(count, 4);
        assert!(observed_axes[0].z != 0.0);
        for axis in &observed_axes[1..] {
            assert_eq!(*axis, observed_axes[0]);
        }
    }

    #[test]
    fn as17_aero_is_recomputed_from_each_stage_state() {
        let model = load_model(
            &[identity_element("wing", [0.0, 0.0, 0.0], 1.2)],
            &[],
            false,
        );
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.04, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(28.0, 0.0, 2.0)),
        )
        .unwrap();
        let mut forces = [Vec3::zeros(); 4];
        let mut velocities = [Vec3::zeros(); 4];
        let mut count = 0;
        let _ = simulation.step_with_stage_observer(&PilotInput::neutral(), |state, _, output| {
            velocities[count] = state.linear_velocity_world_mps;
            forces[count] = output.total_wrench.force_body_n;
            count += 1;
        });
        assert_eq!(count, 4);
        assert!((velocities[1] - velocities[0]).norm() > 1.0e-6);
        assert!((forces[1] - forces[0]).norm() > 1.0e-6);
        assert!((forces[3] - forces[0]).norm() > 1.0e-6);
    }

    #[test]
    fn as18_propulsion_is_recomputed_from_each_stage_axial_speed() {
        let model = load_model(&[], &[], true);
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.04, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(4.0, 0.0, 0.0)),
        )
        .unwrap();
        let mut axial_speeds = [0.0; 4];
        let mut shaft_speeds = [0.0; 4];
        let mut count = 0;
        let _ = simulation.step_with_stage_observer(
            &PilotInput::new(0.0, 0.0, 0.0, 0.8),
            |_, _, output| {
                let propulsion = output.propulsion.as_ref().unwrap();
                axial_speeds[count] = propulsion.axial_airspeed_mps;
                shaft_speeds[count] = propulsion.shaft_speed_rad_s;
                count += 1;
            },
        );
        assert_eq!(count, 4);
        assert!((axial_speeds[1] - axial_speeds[0]).abs() > 1.0e-8);
        assert!((shaft_speeds[1] - shaft_speeds[0]).abs() > 1.0e-8);
    }

    #[test]
    fn m2_4b_v4_map_is_stage_local_instead_of_freezing_the_k1_operating_point() {
        let model = AircraftModelLoader::from_json_str(include_str!(
            "../../../tests/fixtures/synthetic_non_reference_propulsion_v4.json"
        ))
        .unwrap();
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.08, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(3.0, 0.0, 0.0)),
        )
        .unwrap();
        let mut stages = Vec::with_capacity(4);
        let _ = simulation.step_with_stage_observer(
            &PilotInput::new(0.0, 0.0, 0.0, 0.85),
            |stage_state, _, output| {
                stages.push((*stage_state, output.propulsion.unwrap()));
            },
        );
        assert_eq!(stages.len(), 4);
        let runtime = simulation.model().propulsion().unwrap();
        for (stage_state, observed) in &stages {
            let direct = evaluate_electric_propulsion_with_source(
                stage_state,
                0.85,
                runtime.config(),
                simulation.config().aero_environment(),
                runtime.coefficient_source(),
            );
            assert_eq!(*observed, direct);
        }
        let frozen_k1 = stages[0].1;
        assert_ne!(stages[1].1, frozen_k1);
        assert_ne!(stages[3].1, frozen_k1);
        assert_ne!(
            stages[1].1.coefficient_map_sample,
            frozen_k1.coefficient_map_sample
        );
    }

    #[test]
    fn as19_snapshot_and_simulation_use_post_step_accounting() {
        let model = load_model(&[], &[], false);
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::new(1.0, 0.0, 0.0)),
        )
        .unwrap();
        for expected in 1..=17 {
            let snapshot = simulation.step(&PilotInput::neutral());
            assert_eq!(snapshot.step_index(), expected);
            assert_eq!(snapshot.sim_time_s(), expected as f64 * 0.002);
        }
        assert_eq!(simulation.step_index(), 17);
        assert_eq!(simulation.sim_time_s(), 17.0 * 0.002);
    }

    #[test]
    fn as20_repeat_runs_are_bit_identical() {
        let elements = [
            identity_element("left", [0.0, -0.7, 0.0], 0.25),
            identity_element("right", [0.0, 0.7, 0.0], 0.25),
            identity_element("tail", [-0.8, 0.0, 0.0], 0.15),
        ];
        let bindings = [
            binding("left-binding", "left", "aileron", 1.0),
            binding("right-binding", "right", "aileron", -1.0),
            binding("tail-binding", "tail", "elevator", -1.0),
        ];
        let json_model = || load_model(&elements, &bindings, true);
        let initial = state_with_velocity(Vec3::new(20.0, 0.0, 0.5));
        let run_config = config(0.002, 0.3, Vec3::new(0.0, 0.0, 9.80665));
        let mut first = AircraftSimulation::new(json_model(), run_config, initial).unwrap();
        let mut second = AircraftSimulation::new(json_model(), run_config, initial).unwrap();
        for step in 0..300 {
            let phase = f64::from(step % 41) / 40.0;
            let input = PilotInput::new(
                0.3 * (2.0 * phase - 1.0),
                0.15 * (1.0 - 2.0 * phase),
                0.1 * (2.0 * phase - 1.0),
                0.55,
            );
            let first_snapshot = first.step(&input);
            let second_snapshot = second.step(&input);
            assert_eq!(first_snapshot, second_snapshot);
        }
        assert_eq!(first.state(), second.state());
    }

    fn first_stage_wrench(
        model: AircraftModel,
        input: PilotInput,
        initial_velocity: Vec3,
    ) -> BodyWrench {
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::zeros()),
            state_with_velocity(initial_velocity),
        )
        .unwrap();
        let mut first = None;
        let _ = simulation.step_with_stage_observer(&input, |_, _, output| {
            if first.is_none() {
                first = Some(output.total_wrench);
            }
        });
        first.unwrap()
    }

    #[test]
    fn as21_positive_pitch_input_produces_positive_pitching_moment() {
        let model = load_model(
            &[identity_element("elevator", [-1.0, 0.0, 0.0], 0.35)],
            &[binding("elevator-binding", "elevator", "elevator", -1.0)],
            false,
        );
        let wrench = first_stage_wrench(
            model,
            PilotInput::new(0.0, 1.0, 0.0, 0.0),
            Vec3::new(22.0, 0.0, 0.0),
        );
        assert!(wrench.moment_body_nm.y > 0.0, "wrench={wrench:?}");
    }

    #[test]
    fn as22_positive_roll_input_produces_positive_rolling_moment() {
        let model = load_model(
            &[
                identity_element("left-aileron", [0.0, -0.9, 0.0], 0.25),
                identity_element("right-aileron", [0.0, 0.9, 0.0], 0.25),
            ],
            &[
                binding("left-binding", "left-aileron", "aileron", 1.0),
                binding("right-binding", "right-aileron", "aileron", -1.0),
            ],
            false,
        );
        let wrench = first_stage_wrench(
            model,
            PilotInput::new(1.0, 0.0, 0.0, 0.0),
            Vec3::new(22.0, 0.0, 0.0),
        );
        assert!(wrench.moment_body_nm.x > 0.0, "wrench={wrench:?}");
    }

    #[test]
    fn as23_positive_yaw_input_produces_positive_yawing_moment() {
        let half_sqrt_two = 0.5_f64.sqrt();
        let model = load_model(
            &[element(
                "rudder",
                [-1.0, 0.0, 0.0],
                [half_sqrt_two, half_sqrt_two, 0.0, 0.0],
                0.3,
            )],
            &[binding("rudder-binding", "rudder", "rudder", -1.0)],
            false,
        );
        let wrench = first_stage_wrench(
            model,
            PilotInput::new(0.0, 0.0, 1.0, 0.0),
            Vec3::new(22.0, 0.0, 0.0),
        );
        assert!(wrench.moment_body_nm.z > 0.0, "wrench={wrench:?}");
    }

    #[test]
    fn symmetric_aircraft_cancels_lateral_force_roll_and_yaw() {
        let model = load_model(
            &[
                identity_element("left", [0.0, -0.8, 0.0], 0.3),
                identity_element("right", [0.0, 0.8, 0.0], 0.3),
            ],
            &[],
            false,
        );
        let simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::zeros()),
            state_with_velocity(Vec3::new(24.0, 0.0, 0.0)),
        )
        .unwrap();
        let wrench = evaluate_aerodynamic_wrench(
            simulation.state().rigid_body(),
            simulation.effective_aero_elements(),
            simulation.model(),
            simulation.config().aero_environment(),
        );
        let tolerance = 128.0 * f64::EPSILON * wrench.force_body_n.norm().max(1.0);
        assert!(wrench.force_body_n.y.abs() <= tolerance);
        assert!(wrench.moment_body_nm.x.abs() <= tolerance);
        assert!(wrench.moment_body_nm.z.abs() <= tolerance);
    }

    #[test]
    fn complete_step_updates_all_subsystems_and_allocates_nothing() {
        let elements = [
            identity_element("left", [0.0, -0.7, 0.0], 0.3),
            identity_element("right", [0.0, 0.7, 0.0], 0.3),
            identity_element("elevator", [-0.9, 0.0, 0.0], 0.2),
        ];
        let bindings = [
            binding("left-binding", "left", "aileron", 1.0),
            binding("right-binding", "right", "aileron", -1.0),
            binding("elevator-binding", "elevator", "elevator", -1.0),
        ];
        let model = load_model(&elements, &bindings, true);
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::new(0.0, 0.0, 9.80665)),
            state_with_velocity(Vec3::new(20.0, 0.0, 0.0)),
        )
        .unwrap();
        let input = PilotInput::new(0.3, 0.2, -0.1, 0.6);
        let before = *simulation.state().rigid_body();
        let first = simulation.step(&input);
        assert_eq!(first.step_index(), 1);
        assert_ne!(*first.rigid_body_state(), before);
        assert!(first.control_surface_positions().aileron_angle_rad() > 0.0);
        assert!(first.control_surface_positions().elevator_angle_rad() > 0.0);
        assert_eq!(first.control_surface_positions().throttle(), 0.6);
        assert_finite(first.rigid_body_state());
        assert!(
            (first.rigid_body_state().orientation_world_from_body.norm() - 1.0).abs() < 1.0e-15
        );

        let allocation_info = allocation_counter::measure(|| {
            let snapshot = simulation.step(&input);
            std::hint::black_box(snapshot);
        });
        assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
    }

    #[test]
    fn ten_thousand_step_run_remains_finite_and_servos_stay_in_travel() {
        let elements = [
            identity_element("aileron", [0.0, 0.0, 0.0], 0.01),
            identity_element("elevator", [0.0, 0.0, 0.0], 0.01),
            element(
                "rudder",
                [0.0, 0.0, 0.0],
                [0.5_f64.sqrt(), 0.5_f64.sqrt(), 0.0, 0.0],
                0.01,
            ),
        ];
        let bindings = [
            binding("aileron-binding", "aileron", "aileron", 1.0),
            binding("elevator-binding", "elevator", "elevator", -1.0),
            binding("rudder-binding", "rudder", "rudder", -1.0),
        ];
        let model = load_model(&elements, &bindings, false);
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::new(0.0, 0.0, 9.80665)),
            state_with_velocity(Vec3::new(15.0, 0.0, 0.0)),
        )
        .unwrap();
        for step in 0..10_000 {
            let phase = f64::from(step % 1000) / 999.0;
            let input = PilotInput::new(
                0.2 * (2.0 * phase - 1.0),
                0.15 * (1.0 - 2.0 * phase),
                0.1 * (2.0 * phase - 1.0),
                0.0,
            );
            let snapshot = simulation.step(&input);
            if step % 50 == 0 {
                assert_finite(snapshot.rigid_body_state());
                for angle in [
                    snapshot.control_surface_positions().aileron_angle_rad(),
                    snapshot.control_surface_positions().elevator_angle_rad(),
                    snapshot.control_surface_positions().rudder_angle_rad(),
                ] {
                    assert!((-0.5..=0.5).contains(&angle));
                }
            }
        }
        assert_eq!(simulation.step_index(), 10_000);
        assert_eq!(simulation.sim_time_s(), 20.0);
        assert_finite(simulation.state().rigid_body());
    }

    #[test]
    fn acro_electric_01_headless_smoke_run() {
        let model = AircraftModelLoader::from_json_str(include_str!(
            "../../../models/acro_electric_01/model.json"
        ))
        .unwrap();
        assert_eq!(model.schema_version(), 2);
        assert!(model.control_surface_bindings().len() >= 4);
        assert!(model.propulsion().is_some());
        let initial = state_with_velocity(Vec3::new(22.0, 0.0, 0.0));
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::new(0.0, 0.0, 9.80665)),
            initial,
        )
        .unwrap();
        let input = PilotInput::new(0.02, 0.01, -0.01, 0.6);
        let first = simulation.step(&input);
        assert!(first.control_surface_positions().aileron_angle_rad() != 0.0);
        assert!(
            simulation
                .effective_aero_elements()
                .iter()
                .zip(simulation.model().aero_elements())
                .any(|(effective, base)| effective != base.element())
        );
        let aero = evaluate_aerodynamic_wrench(
            simulation.state().rigid_body(),
            simulation.effective_aero_elements(),
            simulation.model(),
            simulation.config().aero_environment(),
        );
        let total = evaluate_aircraft_wrench(
            simulation.state().rigid_body(),
            simulation.effective_aero_elements(),
            simulation.model(),
            input.throttle(),
            simulation.config().aero_environment(),
        );
        assert!(aero.force_body_n.norm() > 0.0);
        assert!((total.force_body_n - aero.force_body_n).norm() > 0.0);
        for _ in 1..500 {
            let snapshot = simulation.step(&input);
            assert_finite(snapshot.rigid_body_state());
        }
        assert_eq!(simulation.step_index(), 500);
        assert!(
            (simulation.state().rigid_body().position_world_m - initial.position_world_m).norm()
                > 1.0
        );
    }

    #[test]
    fn acro_electric_01_complete_step_allocates_nothing_after_initialization() {
        let model = AircraftModelLoader::from_json_str(include_str!(
            "../../../models/acro_electric_01/model.json"
        ))
        .unwrap();
        let mut simulation = AircraftSimulation::new(
            model,
            config(0.002, 1.225, Vec3::new(0.0, 0.0, 9.80665)),
            state_with_velocity(Vec3::new(18.0, 0.0, 0.0)),
        )
        .unwrap();
        let input = PilotInput::new(0.0, 0.0, 0.0, 0.55);
        std::hint::black_box(simulation.step(&input));

        let allocation_info = allocation_counter::measure(|| {
            for _ in 0..100 {
                std::hint::black_box(simulation.step(std::hint::black_box(&input)));
            }
        });
        assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
    }

    #[test]
    fn huge_finite_gain_that_can_overflow_is_rejected_at_initialization() {
        let elements = [identity_element("surface", [0.0, 0.0, 0.0], 0.2)];
        let bindings = [String::from(
            r#"{"id":"surface-binding","element_id":"surface","actuator":"aileron","deflection_gain":1e308}"#,
        )];
        let json = ROOT_TEMPLATE
            .replace("__ELEMENTS__", &elements.join(","))
            .replace("__BINDINGS__", &bindings.join(","))
            .replace("__PROPULSION__", "null")
            .replace("\"max_angle_rad\": 0.5", "\"max_angle_rad\": 2.0");
        let model = AircraftModelLoader::from_json_str(&json).unwrap();
        let result = AircraftSimulation::new(
            model,
            config(0.002, 0.0, Vec3::zeros()),
            state_with_velocity(Vec3::zeros()),
        );
        assert_eq!(
            result.unwrap_err(),
            AircraftSimulationError::NonFiniteControlSurfaceDeflection { binding_index: 0 }
        );
    }

    #[test]
    fn rudder_fixture_local_y_is_body_vertical() {
        let orientation = Orientation::from_axis_angle(&Vec3::x_axis(), FRAC_PI_2);
        assert_vec_close(orientation.transform_vector(&Vec3::y()), Vec3::z(), 4.0e-16);
    }
}
