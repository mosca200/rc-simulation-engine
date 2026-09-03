use crate::AircraftSimulationConfig;
use model::{
    AircraftModel, ControlActuator, RuntimeAeroElement, RuntimeAeroPolarBinding, RuntimeAeroSurface,
};
use sim_core::{
    AeroElement, AeroElementOutput, BodyWrench, ControlSurfacePositions, ControlSystemState,
    MIN_SECTION_AIRSPEED_MPS, PilotInput, PolarCoefficients, PropulsionOutput,
    ReynoldsAeroElementOutput, RigidBodyDerivative, RigidBodyState, Rk4Integrator,
    SectionKinematics, StateError, advance_controls, assemble_aero_element_wrench,
    calculate_reynolds_number, compute_section_kinematics, evaluate_aero_element,
    evaluate_derivative, evaluate_electric_propulsion_with_source, evaluate_reynolds_aero_element,
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

/// Same-stage actuator-disk result shared by every targeted aerodynamic element.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PropellerSlipstream {
    induced_velocity_mps: f64,
    axis_body: Vec3,
}

impl PropellerSlipstream {
    #[must_use]
    pub const fn induced_velocity_mps(self) -> f64 {
        self.induced_velocity_mps
    }

    /// Body-frame direction corresponding to propeller local `+X` and positive thrust.
    #[must_use]
    pub const fn axis_body(self) -> Vec3 {
        self.axis_body
    }
}

/// Derives ideal actuator-disk induced velocity from actual same-stage thrust.
///
/// For positive thrust, `vi = 0.5 * (sqrt(V^2 + 2T/(rho A)) - V)`, where `V` is
/// the propulsion evaluator's air-relative velocity at the propeller projected on
/// propeller local `+X`. Unsupported or non-finite derived states fail safe to zero.
#[must_use]
pub fn propeller_slipstream(
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    propulsion_output: &PropulsionOutput,
) -> PropellerSlipstream {
    if model.propeller_slipstream_interactions().is_empty()
        || propulsion_output.thrust_n <= 0.0
        || !propulsion_output.thrust_n.is_finite()
    {
        return PropellerSlipstream::default();
    }
    let rho = environment.air_density_kg_m3();
    let Some(runtime_propulsion) = model.propulsion() else {
        return PropellerSlipstream::default();
    };
    let diameter = runtime_propulsion.config().propeller().diameter_m();
    let disk_area = std::f64::consts::PI * diameter * diameter * 0.25;
    if rho <= 0.0 || !rho.is_finite() || disk_area <= 0.0 || !disk_area.is_finite() {
        return PropellerSlipstream::default();
    }
    let axial_velocity = propulsion_output.axial_airspeed_mps;
    if !axial_velocity.is_finite() {
        return PropellerSlipstream::default();
    }
    let radicand = axial_velocity.mul_add(
        axial_velocity,
        2.0 * propulsion_output.thrust_n / (rho * disk_area),
    );
    let induced_velocity_mps = 0.5 * (radicand.sqrt() - axial_velocity);
    if !induced_velocity_mps.is_finite() || induced_velocity_mps <= 0.0 {
        return PropellerSlipstream::default();
    }
    let axis_body = runtime_propulsion
        .config()
        .propeller()
        .orientation_body_from_prop()
        .transform_vector(&Vec3::new(1.0, 0.0, 0.0));
    PropellerSlipstream {
        induced_velocity_mps,
        axis_body,
    }
}

fn slipstream_velocity_factor(element_index: usize, model: &AircraftModel) -> f64 {
    model
        .propeller_slipstream_interactions()
        .iter()
        .find(|interaction| {
            interaction
                .target_element_indices()
                .contains(&element_index)
        })
        .map_or(0.0, |interaction| interaction.slipstream_velocity_factor())
}

/// Canonical physical section flow after the per-element propeller wake increment.
pub(crate) fn physical_section_kinematics(
    element_index: usize,
    stage_state: &RigidBodyState,
    effective_element: &AeroElement,
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    slipstream: PropellerSlipstream,
) -> SectionKinematics {
    let base = compute_section_kinematics(stage_state, effective_element, environment);
    let factor = slipstream_velocity_factor(element_index, model);
    if factor == 0.0 || slipstream.induced_velocity_mps == 0.0 {
        return base;
    }
    let increment_mps = factor * slipstream.induced_velocity_mps;
    if !increment_mps.is_finite() {
        return base;
    }
    let wake_body_mps = slipstream.axis_body * increment_mps;
    let wake_element_mps = effective_element
        .orientation_body_from_element()
        .inverse_transform_vector(&wake_body_mps);
    let velocity = base.air_relative_velocity_element_mps + wake_element_mps;
    if !velocity.iter().all(|component| component.is_finite()) {
        return base;
    }
    let speed_squared = velocity.x.mul_add(velocity.x, velocity.z * velocity.z);
    if !speed_squared.is_finite() {
        return base;
    }
    let section_airspeed_mps = speed_squared.sqrt();
    let beta_rad = velocity.y.atan2(section_airspeed_mps);
    if section_airspeed_mps < MIN_SECTION_AIRSPEED_MPS {
        return SectionKinematics {
            air_relative_velocity_element_mps: velocity,
            section_airspeed_mps,
            alpha_rad: 0.0,
            beta_rad,
            dynamic_pressure_pa: 0.0,
        };
    }
    let alpha_rad = velocity.z.atan2(velocity.x);
    let dynamic_pressure_pa = 0.5 * environment.air_density_kg_m3() * speed_squared;
    if !dynamic_pressure_pa.is_finite() {
        return base;
    }
    SectionKinematics {
        air_relative_velocity_element_mps: velocity,
        section_airspeed_mps,
        alpha_rad,
        beta_rad,
        dynamic_pressure_pa,
    }
}

/// Public diagnostic for the exact pre-downwash physical flow used by schema-v7 physics.
#[must_use]
pub fn evaluate_aircraft_section_kinematics(
    element_index: usize,
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    propulsion_output: Option<&PropulsionOutput>,
) -> SectionKinematics {
    assert_eq!(effective_aero_elements.len(), model.aero_elements().len());
    let slipstream = propulsion_output
        .map(|output| propeller_slipstream(model, environment, output))
        .unwrap_or_default();
    physical_section_kinematics(
        element_index,
        stage_state,
        &effective_aero_elements[element_index],
        model,
        environment,
        slipstream,
    )
}

/// Aggregates every S4 element wrench, preserving the model's declaration order.
///
/// When the model has finite-wing surfaces (schema v5+), each surface is solved
/// independently for a common induced angle of attack using deterministic bracketed
/// bisection. Members assigned to surfaces receive finite-wing corrections (effective
/// alpha for polar sampling, induced drag added to profile drag). Unassigned elements
/// and models with no surfaces follow the exact legacy quasi-2D path.
#[must_use]
pub fn evaluate_aerodynamic_wrench(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
) -> BodyWrench {
    evaluate_aerodynamic_wrench_with_propulsion(
        stage_state,
        effective_aero_elements,
        model,
        environment,
        None,
    )
}

/// Aggregates aerodynamic wrench using an already-evaluated same-stage propulsion output.
///
/// Passing `None` preserves the uncoupled v0-v6 path exactly. Schema-v7 callers provide
/// the actual same-stage output so slipstream is driven by thrust rather than throttle.
#[must_use]
pub fn evaluate_aerodynamic_wrench_with_propulsion(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    propulsion_output: Option<&PropulsionOutput>,
) -> BodyWrench {
    assert_eq!(effective_aero_elements.len(), model.aero_elements().len());
    let slipstream = propulsion_output
        .map(|output| propeller_slipstream(model, environment, output))
        .unwrap_or_default();
    let surfaces = model.aero_surfaces();

    if surfaces.is_empty() {
        return evaluate_legacy_wrench(
            stage_state,
            effective_aero_elements,
            model,
            environment,
            slipstream,
        );
    }

    let mut total_wrench = BodyWrench::zero();

    for (surface_index, surface) in surfaces.iter().enumerate() {
        let wrench = evaluate_surface_wrench(
            surface_index,
            surface,
            stage_state,
            effective_aero_elements,
            model,
            environment,
            slipstream,
        );
        total_wrench.force_body_n += wrench.force_body_n;
        total_wrench.moment_body_nm += wrench.moment_body_nm;
    }

    for (idx, (effective_element, runtime_element)) in effective_aero_elements
        .iter()
        .zip(model.aero_elements())
        .enumerate()
    {
        if is_element_assigned_to_any_surface(surfaces, idx) {
            continue;
        }
        let wrench = evaluate_independent_element_wrench(
            idx,
            stage_state,
            effective_element,
            runtime_element,
            model,
            environment,
            slipstream,
        );
        total_wrench.force_body_n += wrench.force_body_n;
        total_wrench.moment_body_nm += wrench.moment_body_nm;
    }

    total_wrench
}

/// Allocation-free check: is element at `index` assigned to any surface?
fn is_element_assigned_to_any_surface(surfaces: &[RuntimeAeroSurface], index: usize) -> bool {
    surfaces
        .iter()
        .any(|s| s.element_indices().contains(&index))
}

/// Legacy path: every element evaluated independently through the quasi-2D polar path.
/// Used when the model has no finite-wing surfaces (schema v0-v4, or v5+ with surfaces=[]).
fn evaluate_legacy_wrench(
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    slipstream: PropellerSlipstream,
) -> BodyWrench {
    let mut total_wrench = BodyWrench::zero();
    for (element_index, (effective_element, runtime_element)) in effective_aero_elements
        .iter()
        .zip(model.aero_elements())
        .enumerate()
    {
        let wrench = evaluate_independent_element_wrench(
            element_index,
            stage_state,
            effective_element,
            runtime_element,
            model,
            environment,
            slipstream,
        );
        total_wrench.force_body_n += wrench.force_body_n;
        total_wrench.moment_body_nm += wrench.moment_body_nm;
    }
    total_wrench
}

fn evaluate_independent_element_wrench(
    element_index: usize,
    stage_state: &RigidBodyState,
    effective_element: &AeroElement,
    runtime_element: &RuntimeAeroElement,
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    slipstream: PropellerSlipstream,
) -> BodyWrench {
    if slipstream_velocity_factor(element_index, model) == 0.0
        || slipstream.induced_velocity_mps == 0.0
    {
        return evaluate_aircraft_aero_element(
            stage_state,
            effective_element,
            runtime_element,
            model,
            environment,
        )
        .aero()
        .wrench_body;
    }
    let kin = physical_section_kinematics(
        element_index,
        stage_state,
        effective_element,
        model,
        environment,
        slipstream,
    );
    if kin.dynamic_pressure_pa == 0.0 {
        return BodyWrench::zero();
    }
    let coefficients = match runtime_element.polar_binding() {
        RuntimeAeroPolarBinding::Polar { polar_index } => model.aero_polars()[polar_index]
            .table()
            .sample_clamped(kin.alpha_rad),
        RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
            let viscosity = model.kinematic_viscosity_m2_s().unwrap();
            let reynolds = calculate_reynolds_number(
                kin.section_airspeed_mps,
                effective_element.chord_m(),
                viscosity,
            )
            .unwrap_or(0.0);
            model.aero_polar_families()[family_index]
                .family()
                .sample(reynolds, kin.alpha_rad)
                .coefficients
        }
    };
    assemble_aero_element_wrench(effective_element, &kin, &coefficients)
}

/// Number of deterministic bisection iterations for the induced-angle solver.
const INDUCED_ALPHA_BISECTION_ITERATIONS: usize = 40;

/// Samples the CL coefficient for one surface member at a given alpha, without allocation.
pub(crate) fn sample_member_cl(
    runtime_element: &RuntimeAeroElement,
    model: &AircraftModel,
    section_airspeed: f64,
    chord: f64,
    alpha_rad: f64,
) -> f64 {
    match runtime_element.polar_binding() {
        RuntimeAeroPolarBinding::Polar { polar_index } => {
            model.aero_polars()[polar_index]
                .table()
                .sample_clamped(alpha_rad)
                .cl
        }
        RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
            let viscosity = model
                .kinematic_viscosity_m2_s()
                .expect("Reynolds-family bindings require schema v3+ with explicit viscosity");
            let re = calculate_reynolds_number(section_airspeed, chord, viscosity).unwrap_or(0.0);
            model.aero_polar_families()[family_index]
                .family()
                .sample(re, alpha_rad)
                .coefficients
                .cl
        }
    }
}

/// Finds the maximum absolute CL reachable by any sample in a member's polar binding.
fn max_abs_cl_member(runtime_element: &RuntimeAeroElement, model: &AircraftModel) -> f64 {
    match runtime_element.polar_binding() {
        RuntimeAeroPolarBinding::Polar { polar_index } => model.aero_polars()[polar_index]
            .table()
            .samples()
            .iter()
            .map(|s| s.cl.abs())
            .fold(0.0_f64, f64::max),
        RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
            let family = model.aero_polar_families()[family_index].family();
            family
                .nodes()
                .iter()
                .flat_map(|node| node.table().samples().iter().map(|s| s.cl.abs()))
                .fold(0.0_f64, f64::max)
        }
    }
}

/// Solves for the common induced angle of attack of one finite-wing surface using
/// deterministic bracketed bisection.
///
/// The bracket is derived from the maximum absolute CL reachable by any member polar:
/// `alpha_bound = CL_abs_max / (PI * AR * e)`.
///
/// Returns `(alpha_i, CL_surface, CDi_surface)`.
/// Solves one surface using slipstream-adjusted physical member flows, then downwash.
pub(crate) fn solve_surface_induced_alpha_with_physical_flow(
    surface: &RuntimeAeroSurface,
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    downwash_angle_rad: f64,
    slipstream: PropellerSlipstream,
) -> (f64, f64, f64) {
    let ar = surface.aspect_ratio();
    let e = surface.span_efficiency_factor();
    let pi_ar_e = std::f64::consts::PI * ar * e;

    let member_indices = surface.element_indices();

    let cl_abs_max = member_indices
        .iter()
        .map(|&idx| max_abs_cl_member(&model.aero_elements()[idx], model))
        .fold(0.0_f64, f64::max);

    if cl_abs_max == 0.0 {
        return (0.0, 0.0, 0.0);
    }

    let alpha_bound = cl_abs_max / pi_ar_e;
    let mut lo = -alpha_bound;
    let mut hi = alpha_bound;

    for _ in 0..INDUCED_ALPHA_BISECTION_ITERATIONS {
        let mid = 0.5 * (lo + hi);
        let mut weighted_cl_sum = 0.0;
        let mut weight_sum = 0.0;

        for &member_idx in member_indices {
            let kin = downwashed_section_kinematics(
                physical_section_kinematics(
                    member_idx,
                    stage_state,
                    &effective_aero_elements[member_idx],
                    model,
                    environment,
                    slipstream,
                ),
                downwash_angle_rad,
            );
            if kin.dynamic_pressure_pa == 0.0 {
                continue;
            }
            let w = kin.dynamic_pressure_pa * effective_aero_elements[member_idx].area_m2();
            let alpha_eff = kin.alpha_rad - mid;
            let cl = sample_member_cl(
                &model.aero_elements()[member_idx],
                model,
                kin.section_airspeed_mps,
                effective_aero_elements[member_idx].chord_m(),
                alpha_eff,
            );
            weighted_cl_sum += w * cl;
            weight_sum += w;
        }

        let cl_surface = if weight_sum > 0.0 {
            weighted_cl_sum / weight_sum
        } else {
            0.0
        };
        let g = mid - cl_surface / pi_ar_e;

        if g == 0.0 {
            let cdi = cl_surface * cl_surface / pi_ar_e;
            return (mid, cl_surface, cdi);
        } else if g > 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    let alpha_i = 0.5 * (lo + hi);
    let mut weighted_cl_sum = 0.0;
    let mut weight_sum = 0.0;
    for &member_idx in member_indices {
        let kin = downwashed_section_kinematics(
            physical_section_kinematics(
                member_idx,
                stage_state,
                &effective_aero_elements[member_idx],
                model,
                environment,
                slipstream,
            ),
            downwash_angle_rad,
        );
        if kin.dynamic_pressure_pa == 0.0 {
            continue;
        }
        let w = kin.dynamic_pressure_pa * effective_aero_elements[member_idx].area_m2();
        let alpha_eff = kin.alpha_rad - alpha_i;
        let cl = sample_member_cl(
            &model.aero_elements()[member_idx],
            model,
            kin.section_airspeed_mps,
            effective_aero_elements[member_idx].chord_m(),
            alpha_eff,
        );
        weighted_cl_sum += w * cl;
        weight_sum += w;
    }
    let cl_surface = if weight_sum > 0.0 {
        weighted_cl_sum / weight_sum
    } else {
        0.0
    };
    let cdi = cl_surface * cl_surface / pi_ar_e;
    (alpha_i, cl_surface, cdi)
}

/// Source induced angle and target wake rotation for one resolved target surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SurfaceDownwash {
    pub source_alpha_i_rad: f64,
    pub downwash_angle_rad: f64,
}

/// Allocation-free diagnostic of the exact finite-wing solution used at one stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AircraftSurfaceAerodynamicState {
    pub source_alpha_i_rad: f64,
    pub downwash_angle_rad: f64,
    pub induced_alpha_rad: f64,
    pub surface_cl: f64,
    pub induced_drag_coefficient: f64,
}

/// Evaluates one surface's same-stage slipstream/downwash/finite-wing composition.
#[must_use]
pub fn evaluate_aircraft_surface_aerodynamic_state(
    surface_index: usize,
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    propulsion_output: Option<&PropulsionOutput>,
) -> AircraftSurfaceAerodynamicState {
    let slipstream = propulsion_output
        .map(|output| propeller_slipstream(model, environment, output))
        .unwrap_or_default();
    let surface = &model.aero_surfaces()[surface_index];
    let downwash = surface_downwash_with_slipstream(
        surface_index,
        stage_state,
        effective_aero_elements,
        model,
        environment,
        slipstream,
    );
    let (induced_alpha_rad, surface_cl, induced_drag_coefficient) =
        solve_surface_induced_alpha_with_physical_flow(
            surface,
            stage_state,
            effective_aero_elements,
            model,
            environment,
            downwash.downwash_angle_rad,
            slipstream,
        );
    AircraftSurfaceAerodynamicState {
        source_alpha_i_rad: downwash.source_alpha_i_rad,
        downwash_angle_rad: downwash.downwash_angle_rad,
        induced_alpha_rad,
        surface_cl,
        induced_drag_coefficient,
    }
}

pub(crate) fn surface_downwash_with_slipstream(
    target_surface_index: usize,
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    slipstream: PropellerSlipstream,
) -> SurfaceDownwash {
    let Some(interaction) = model
        .aero_downwash_interactions()
        .iter()
        .find(|interaction| interaction.target_surface_index() == target_surface_index)
    else {
        return SurfaceDownwash {
            source_alpha_i_rad: 0.0,
            downwash_angle_rad: 0.0,
        };
    };
    let source = &model.aero_surfaces()[interaction.source_surface_index()];
    let (source_alpha_i_rad, _, _) = solve_surface_induced_alpha_with_physical_flow(
        source,
        stage_state,
        effective_aero_elements,
        model,
        environment,
        0.0,
        slipstream,
    );
    SurfaceDownwash {
        source_alpha_i_rad,
        downwash_angle_rad: interaction.downwash_factor() * source_alpha_i_rad,
    }
}

/// Rotates physical section-relative airflow about local +Y so that
/// `alpha_after = alpha_before - downwash_angle_rad`.
pub(crate) fn downwashed_section_kinematics(
    kinematics: SectionKinematics,
    downwash_angle_rad: f64,
) -> SectionKinematics {
    if downwash_angle_rad == 0.0 || kinematics.section_airspeed_mps == 0.0 {
        return kinematics;
    }
    let (sin_epsilon, cos_epsilon) = downwash_angle_rad.sin_cos();
    let velocity = kinematics.air_relative_velocity_element_mps;
    let rotated = Vec3::new(
        cos_epsilon.mul_add(velocity.x, sin_epsilon * velocity.z),
        velocity.y,
        (-sin_epsilon).mul_add(velocity.x, cos_epsilon * velocity.z),
    );
    SectionKinematics {
        air_relative_velocity_element_mps: rotated,
        section_airspeed_mps: kinematics.section_airspeed_mps,
        alpha_rad: rotated.z.atan2(rotated.x),
        beta_rad: kinematics.beta_rad,
        dynamic_pressure_pa: kinematics.dynamic_pressure_pa,
    }
}

/// Evaluates the total wrench for one finite-wing surface, including induced drag.
///
/// For each member:
/// - Downwash physically rotates target flow before any target finite-wing solve
/// - Polar is sampled at `alpha_geom_downwashed - alpha_i` (effective alpha)
/// - `CDi_surface` is added to the profile drag coefficient
/// - Force directions come from the actual, possibly downwashed local section flow
fn evaluate_surface_wrench(
    surface_index: usize,
    surface: &RuntimeAeroSurface,
    stage_state: &RigidBodyState,
    effective_aero_elements: &[AeroElement],
    model: &AircraftModel,
    environment: &sim_core::AeroEnvironment,
    slipstream: PropellerSlipstream,
) -> BodyWrench {
    let downwash = surface_downwash_with_slipstream(
        surface_index,
        stage_state,
        effective_aero_elements,
        model,
        environment,
        slipstream,
    );
    let (alpha_i, _cl_surface, cdi_surface) = solve_surface_induced_alpha_with_physical_flow(
        surface,
        stage_state,
        effective_aero_elements,
        model,
        environment,
        downwash.downwash_angle_rad,
        slipstream,
    );

    let mut wrench = BodyWrench::zero();

    for &member_idx in surface.element_indices() {
        let effective_element = &effective_aero_elements[member_idx];
        let runtime_element = &model.aero_elements()[member_idx];

        let kin = downwashed_section_kinematics(
            physical_section_kinematics(
                member_idx,
                stage_state,
                effective_element,
                model,
                environment,
                slipstream,
            ),
            downwash.downwash_angle_rad,
        );
        if kin.dynamic_pressure_pa == 0.0 {
            continue;
        }

        let alpha_eff = kin.alpha_rad - alpha_i;
        let coeffs = match runtime_element.polar_binding() {
            RuntimeAeroPolarBinding::Polar { polar_index } => model.aero_polars()[polar_index]
                .table()
                .sample_clamped(alpha_eff),
            RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
                let viscosity = model.kinematic_viscosity_m2_s().unwrap();
                let re = calculate_reynolds_number(
                    kin.section_airspeed_mps,
                    effective_element.chord_m(),
                    viscosity,
                )
                .unwrap_or(0.0);
                model.aero_polar_families()[family_index]
                    .family()
                    .sample(re, alpha_eff)
                    .coefficients
            }
        };

        let cd_total = coeffs.cd + cdi_surface;
        let adjusted = PolarCoefficients {
            cl: coeffs.cl,
            cd: cd_total,
            cm: coeffs.cm,
        };

        let member_wrench = assemble_aero_element_wrench(effective_element, &kin, &adjusted);
        wrench.force_body_n += member_wrench.force_body_n;
        wrench.moment_body_nm += member_wrench.moment_body_nm;
    }

    wrench
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
                .expect("Reynolds-family bindings require schema v3+ with explicit viscosity");
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
    let propulsion = model.propulsion().map(|runtime_propulsion| {
        evaluate_electric_propulsion_with_source(
            stage_state,
            throttle,
            runtime_propulsion.config(),
            environment,
            runtime_propulsion.coefficient_source(),
        )
    });
    let mut total_wrench = evaluate_aerodynamic_wrench_with_propulsion(
        stage_state,
        effective_aero_elements,
        model,
        environment,
        propulsion.as_ref(),
    );
    if let Some(output) = propulsion {
        total_wrench.force_body_n += output.wrench_body.force_body_n;
        total_wrench.moment_body_nm += output.wrench_body.moment_body_nm;
    }

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

    #[test]
    fn m2_8c_downwash_rotation_obeys_sign_and_preserves_speed() {
        let alpha = 0.2_f64;
        let speed = 17.0_f64;
        let original = SectionKinematics {
            air_relative_velocity_element_mps: Vec3::new(
                speed * alpha.cos(),
                1.25,
                speed * alpha.sin(),
            ),
            section_airspeed_mps: speed,
            alpha_rad: alpha,
            beta_rad: 1.25_f64.atan2(speed),
            dynamic_pressure_pa: 123.0,
        };
        let epsilon = 0.075;
        let downwashed = downwashed_section_kinematics(original, epsilon);
        assert!((downwashed.alpha_rad - (alpha - epsilon)).abs() < 2.0e-16);
        assert_eq!(
            downwashed.section_airspeed_mps.to_bits(),
            original.section_airspeed_mps.to_bits()
        );
        assert_eq!(
            downwashed.dynamic_pressure_pa.to_bits(),
            original.dynamic_pressure_pa.to_bits()
        );
        assert_eq!(
            downwashed.air_relative_velocity_element_mps.y.to_bits(),
            original.air_relative_velocity_element_mps.y.to_bits()
        );
        let rotated_section_speed = downwashed
            .air_relative_velocity_element_mps
            .x
            .hypot(downwashed.air_relative_velocity_element_mps.z);
        assert!((rotated_section_speed - speed).abs() < 4.0e-15);
        let opposite = downwashed_section_kinematics(original, -epsilon);
        assert!((opposite.alpha_rad - (alpha + epsilon)).abs() < 2.0e-16);
        assert_eq!(downwashed_section_kinematics(original, 0.0), original);
    }

    #[test]
    fn m2_8c_runtime_diagnostic_uses_source_solution_without_feedback() {
        let fixture = include_str!("../../../tests/fixtures/synthetic_downwash_v6.json");
        let model = AircraftModelLoader::from_json_str(fixture).unwrap();
        let mut uncoupled_value: serde_json::Value = serde_json::from_str(fixture).unwrap();
        uncoupled_value["aero_downwash_interactions"] = serde_json::json!([]);
        let uncoupled_model =
            AircraftModelLoader::from_json_str(&serde_json::to_string(&uncoupled_value).unwrap())
                .unwrap();
        let alpha = 0.15_f64;
        let speed = 18.0_f64;
        let state = state_with_velocity(Vec3::new(speed * alpha.cos(), 0.0, speed * alpha.sin()));
        let effective = model
            .aero_elements()
            .iter()
            .map(|runtime| *runtime.element())
            .collect::<Vec<_>>();
        let uncoupled_effective = uncoupled_model
            .aero_elements()
            .iter()
            .map(|runtime| *runtime.element())
            .collect::<Vec<_>>();
        let environment = AeroEnvironment::new(1.225, Vec3::zeros()).unwrap();
        let diagnostic = surface_downwash_with_slipstream(
            1,
            &state,
            &effective,
            &model,
            &environment,
            PropellerSlipstream::default(),
        );
        assert!(diagnostic.source_alpha_i_rad > 0.0);
        assert_eq!(
            diagnostic.downwash_angle_rad,
            1.5 * diagnostic.source_alpha_i_rad
        );
        let negative_state =
            state_with_velocity(Vec3::new(speed * alpha.cos(), 0.0, -speed * alpha.sin()));
        let negative_diagnostic = surface_downwash_with_slipstream(
            1,
            &negative_state,
            &effective,
            &model,
            &environment,
            PropellerSlipstream::default(),
        );
        assert!(negative_diagnostic.source_alpha_i_rad < 0.0);
        assert!(negative_diagnostic.downwash_angle_rad < 0.0);
        assert_eq!(
            negative_diagnostic.downwash_angle_rad,
            1.5 * negative_diagnostic.source_alpha_i_rad
        );

        let source_coupled = evaluate_surface_wrench(
            0,
            &model.aero_surfaces()[0],
            &state,
            &effective,
            &model,
            &environment,
            PropellerSlipstream::default(),
        );
        let target = evaluate_surface_wrench(
            1,
            &model.aero_surfaces()[1],
            &state,
            &effective,
            &model,
            &environment,
            PropellerSlipstream::default(),
        );
        let source_uncoupled = evaluate_surface_wrench(
            0,
            &uncoupled_model.aero_surfaces()[0],
            &state,
            &uncoupled_effective,
            &uncoupled_model,
            &environment,
            PropellerSlipstream::default(),
        );
        let target_uncoupled = evaluate_surface_wrench(
            1,
            &uncoupled_model.aero_surfaces()[1],
            &state,
            &uncoupled_effective,
            &uncoupled_model,
            &environment,
            PropellerSlipstream::default(),
        );
        assert_eq!(source_coupled, source_uncoupled);
        assert_ne!(target, target_uncoupled);
    }
}
