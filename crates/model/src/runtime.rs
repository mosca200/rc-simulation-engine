use crate::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V0, AIRCRAFT_MODEL_SCHEMA_VERSION_V1,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V2, AIRCRAFT_MODEL_SCHEMA_VERSION_V3,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V4, AIRCRAFT_MODEL_SCHEMA_VERSION_V5,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V6, AIRCRAFT_MODEL_SCHEMA_VERSION_V7, AircraftClassification,
    ReferenceAircraftMetadata,
};
use sim_core::{
    AeroElement, ControlSystemConfig, ElectricPropulsionConfig, PolarTable,
    PropellerCoefficientSource, PropellerCoefficientTable, PropellerSpinDirection,
    ReynoldsPolarFamily, RigidBodyParams,
};

/// Immutable validated aircraft configuration with all file references resolved.
#[derive(Debug, Clone, PartialEq)]
pub struct AircraftModel {
    schema_version: u32,
    model_id: String,
    display_name: String,
    classification: AircraftClassification,
    reference_aircraft: Option<ReferenceAircraftMetadata>,
    rigid_body: RigidBodyParams,
    aero_polars: Vec<RuntimePolar>,
    aero_polar_families: Vec<RuntimeReynoldsPolarFamily>,
    aero_elements: Vec<RuntimeAeroElement>,
    aero_surfaces: Vec<RuntimeAeroSurface>,
    aero_downwash_interactions: Vec<RuntimeAeroDownwashInteraction>,
    propeller_slipstream_interactions: Vec<RuntimePropellerSlipstreamInteraction>,
    kinematic_viscosity_m2_s: Option<f64>,
    controls: ControlSystemConfig,
    control_surface_bindings: Vec<RuntimeControlSurfaceBinding>,
    propulsion: Option<RuntimeElectricPropulsion>,
    presentation: Option<PresentationMetadata>,
}

impl AircraftModel {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        schema_version: u32,
        model_id: String,
        display_name: String,
        rigid_body: RigidBodyParams,
        aero_polars: Vec<RuntimePolar>,
        aero_elements: Vec<RuntimeAeroElement>,
        controls: ControlSystemConfig,
        control_surface_bindings: Vec<RuntimeControlSurfaceBinding>,
        propulsion: Option<RuntimeElectricPropulsion>,
        presentation: Option<PresentationMetadata>,
    ) -> Self {
        Self {
            schema_version,
            model_id,
            display_name,
            classification: AircraftClassification::SyntheticTest,
            reference_aircraft: None,
            rigid_body,
            aero_polars,
            aero_polar_families: Vec::new(),
            aero_elements,
            aero_surfaces: Vec::new(),
            aero_downwash_interactions: Vec::new(),
            propeller_slipstream_interactions: Vec::new(),
            kinematic_viscosity_m2_s: None,
            controls,
            control_surface_bindings,
            propulsion,
            presentation,
        }
    }

    pub(crate) fn with_reference_framework(
        mut self,
        classification: AircraftClassification,
        reference_aircraft: Option<ReferenceAircraftMetadata>,
    ) -> Self {
        self.classification = classification;
        self.reference_aircraft = reference_aircraft;
        self
    }

    pub(crate) fn with_control_surface_bindings(
        mut self,
        control_surface_bindings: Vec<RuntimeControlSurfaceBinding>,
    ) -> Self {
        self.control_surface_bindings = control_surface_bindings;
        self
    }

    pub(crate) fn with_reynolds_aerodynamics(
        mut self,
        kinematic_viscosity_m2_s: f64,
        aero_polar_families: Vec<RuntimeReynoldsPolarFamily>,
        aero_elements: Vec<RuntimeAeroElement>,
    ) -> Self {
        self.kinematic_viscosity_m2_s = Some(kinematic_viscosity_m2_s);
        self.aero_polar_families = aero_polar_families;
        self.aero_elements = aero_elements;
        self
    }

    pub(crate) fn with_propulsion(mut self, propulsion: Option<RuntimeElectricPropulsion>) -> Self {
        self.propulsion = propulsion;
        self
    }

    pub(crate) fn with_aero_surfaces(mut self, aero_surfaces: Vec<RuntimeAeroSurface>) -> Self {
        self.aero_surfaces = aero_surfaces;
        self
    }

    pub(crate) fn with_aero_downwash_interactions(
        mut self,
        interactions: Vec<RuntimeAeroDownwashInteraction>,
    ) -> Self {
        self.aero_downwash_interactions = interactions;
        self
    }

    pub(crate) fn with_propeller_slipstream_interactions(
        mut self,
        interactions: Vec<RuntimePropellerSlipstreamInteraction>,
    ) -> Self {
        self.propeller_slipstream_interactions = interactions;
        self
    }

    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    #[must_use]
    pub fn model_id(&self) -> &str {
        &self.model_id
    }

    #[must_use]
    pub fn display_name(&self) -> &str {
        &self.display_name
    }

    #[must_use]
    pub const fn classification(&self) -> AircraftClassification {
        self.classification
    }

    #[must_use]
    pub const fn reference_aircraft(&self) -> Option<&ReferenceAircraftMetadata> {
        self.reference_aircraft.as_ref()
    }

    #[must_use]
    pub const fn rigid_body(&self) -> &RigidBodyParams {
        &self.rigid_body
    }

    #[must_use]
    pub fn aero_polars(&self) -> &[RuntimePolar] {
        &self.aero_polars
    }

    #[must_use]
    pub fn aero_polar_families(&self) -> &[RuntimeReynoldsPolarFamily] {
        &self.aero_polar_families
    }

    #[must_use]
    pub fn aero_elements(&self) -> &[RuntimeAeroElement] {
        &self.aero_elements
    }

    /// Aerodynamic surface groupings. Empty for schema v0-v4 models.
    #[must_use]
    pub fn aero_surfaces(&self) -> &[RuntimeAeroSurface] {
        &self.aero_surfaces
    }

    /// Ordered, initialization-resolved one-way downwash interactions.
    #[must_use]
    pub fn aero_downwash_interactions(&self) -> &[RuntimeAeroDownwashInteraction] {
        &self.aero_downwash_interactions
    }

    /// Ordered, initialization-resolved propeller-slipstream interactions.
    #[must_use]
    pub fn propeller_slipstream_interactions(&self) -> &[RuntimePropellerSlipstreamInteraction] {
        &self.propeller_slipstream_interactions
    }

    /// Explicit model-authoritative viscosity for schema-v3+ Reynolds aerodynamics.
    #[must_use]
    pub const fn kinematic_viscosity_m2_s(&self) -> Option<f64> {
        self.kinematic_viscosity_m2_s
    }

    #[must_use]
    pub const fn controls(&self) -> &ControlSystemConfig {
        &self.controls
    }

    /// Ordered, initialization-resolved control-surface relationships.
    #[must_use]
    pub fn control_surface_bindings(&self) -> &[RuntimeControlSurfaceBinding] {
        &self.control_surface_bindings
    }

    #[must_use]
    pub const fn propulsion(&self) -> Option<&RuntimeElectricPropulsion> {
        self.propulsion.as_ref()
    }

    #[must_use]
    pub const fn presentation(&self) -> Option<&PresentationMetadata> {
        self.presentation.as_ref()
    }

    /// BLAKE3 of the validated physics semantics, independent of JSON formatting and metadata.
    #[must_use]
    pub fn physics_fingerprint(&self) -> AircraftModelFingerprint {
        AircraftModelFingerprint::from_model(self)
    }

    /// Find a Reynolds polar family by ID and return its index.
    pub(crate) fn find_reynolds_family_index(&self, family_id: &str) -> Option<usize> {
        self.aero_polar_families
            .iter()
            .position(|f| f.id() == family_id)
    }

    /// Replace the Reynolds polar family at `index` in-place, preserving the family ID.
    ///
    /// This keeps the family index stable so existing aero-element bindings
    /// (`RuntimeAeroPolarBinding::ReynoldsFamily { family_index }`) remain valid.
    pub(crate) fn replace_reynolds_polar_family_at(
        &mut self,
        index: usize,
        family: ReynoldsPolarFamily,
    ) {
        let id = self.aero_polar_families[index].id().to_owned();
        self.aero_polar_families[index] = RuntimeReynoldsPolarFamily::new(id, family);
    }
}

/// The conventional S5A servo selected by a schema-v1 surface binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlActuator {
    Aileron,
    Elevator,
    Rudder,
}

/// Immutable resolved mapping from one conventional servo to one aero element.
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeControlSurfaceBinding {
    id: String,
    element_index: usize,
    actuator: ControlActuator,
    deflection_gain: f64,
}

impl RuntimeControlSurfaceBinding {
    pub(crate) fn new(
        id: String,
        element_index: usize,
        actuator: ControlActuator,
        deflection_gain: f64,
    ) -> Self {
        Self {
            id,
            element_index,
            actuator,
            deflection_gain,
        }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Compact resolved handle into `AircraftModel::aero_elements()`.
    #[must_use]
    pub const fn element_index(&self) -> usize {
        self.element_index
    }

    #[must_use]
    pub const fn actuator(&self) -> ControlActuator {
        self.actuator
    }

    #[must_use]
    pub const fn deflection_gain(&self) -> f64 {
        self.deflection_gain
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimePolar {
    id: String,
    table: PolarTable,
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeReynoldsPolarFamily {
    id: String,
    family: ReynoldsPolarFamily,
}

impl RuntimeReynoldsPolarFamily {
    pub(crate) fn new(id: String, family: ReynoldsPolarFamily) -> Self {
        Self { id, family }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    #[must_use]
    pub const fn family(&self) -> &ReynoldsPolarFamily {
        &self.family
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeAeroPolarBinding {
    Polar { polar_index: usize },
    ReynoldsFamily { family_index: usize },
}

impl RuntimePolar {
    pub(crate) fn new(id: String, table: PolarTable) -> Self {
        Self { id, table }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    #[must_use]
    pub const fn table(&self) -> &PolarTable {
        &self.table
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeAeroElement {
    id: String,
    element: AeroElement,
    polar_binding: RuntimeAeroPolarBinding,
}

impl RuntimeAeroElement {
    pub(crate) fn new(id: String, element: AeroElement, polar_index: usize) -> Self {
        Self {
            id,
            element,
            polar_binding: RuntimeAeroPolarBinding::Polar { polar_index },
        }
    }

    pub(crate) fn new_reynolds_family(
        id: String,
        element: AeroElement,
        family_index: usize,
    ) -> Self {
        Self {
            id,
            element,
            polar_binding: RuntimeAeroPolarBinding::ReynoldsFamily { family_index },
        }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    #[must_use]
    pub const fn element(&self) -> &AeroElement {
        &self.element
    }

    /// Compact resolved handle into `AircraftModel::aero_polars()`.
    #[must_use]
    pub const fn polar_index(&self) -> usize {
        match self.polar_binding {
            RuntimeAeroPolarBinding::Polar { polar_index } => polar_index,
            RuntimeAeroPolarBinding::ReynoldsFamily { .. } => {
                panic!("Reynolds-family element has no legacy polar index")
            }
        }
    }

    #[must_use]
    pub const fn fixed_polar_index(&self) -> Option<usize> {
        match self.polar_binding {
            RuntimeAeroPolarBinding::Polar { polar_index } => Some(polar_index),
            RuntimeAeroPolarBinding::ReynoldsFamily { .. } => None,
        }
    }

    #[must_use]
    pub const fn polar_binding(&self) -> RuntimeAeroPolarBinding {
        self.polar_binding
    }
}

/// Immutable resolved aerodynamic surface grouping existing aero elements.
///
/// Created during model loading (schema v5+). Surfaces group existing
/// aerodynamic elements for future finite-wing physics (M2.8B).
///
/// - `element_indices`: compact resolved handles into `AircraftModel::aero_elements()`
/// - `span_axis_body`: normalized body-frame span direction
/// - `span_m`: authored physical span (finite, > 0)
/// - `span_efficiency_factor`: finite-wing span-efficiency parameter (finite, > 0, no upper cap)
/// - `area_m2`: derived as sum of member element areas
/// - `aspect_ratio`: derived as `span_m^2 / area_m2`
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeAeroSurface {
    id: String,
    element_indices: Vec<usize>,
    span_axis_body: sim_math::Vec3,
    span_m: f64,
    span_efficiency_factor: f64,
    area_m2: f64,
    aspect_ratio: f64,
}

impl RuntimeAeroSurface {
    pub(crate) fn new(
        id: String,
        element_indices: Vec<usize>,
        span_axis_body: sim_math::Vec3,
        span_m: f64,
        span_efficiency_factor: f64,
        area_m2: f64,
        aspect_ratio: f64,
    ) -> Self {
        Self {
            id,
            element_indices,
            span_axis_body,
            span_m,
            span_efficiency_factor,
            area_m2,
            aspect_ratio,
        }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Compact resolved handles into `AircraftModel::aero_elements()`.
    /// Preserves author-specified membership ordering.
    #[must_use]
    pub fn element_indices(&self) -> &[usize] {
        &self.element_indices
    }

    /// Normalized body-frame direction describing the surface span.
    #[must_use]
    pub const fn span_axis_body(&self) -> &sim_math::Vec3 {
        &self.span_axis_body
    }

    /// Authored physical surface span (m). Finite and strictly positive.
    #[must_use]
    pub const fn span_m(&self) -> f64 {
        self.span_m
    }

    /// Finite-wing span-efficiency parameter. Finite and strictly positive.
    /// No arbitrary upper cap is imposed.
    #[must_use]
    pub const fn span_efficiency_factor(&self) -> f64 {
        self.span_efficiency_factor
    }

    /// Derived surface area: sum of member element areas. Finite and strictly positive.
    #[must_use]
    pub const fn area_m2(&self) -> f64 {
        self.area_m2
    }

    /// Derived aspect ratio: `span_m^2 / area_m2`. Finite and strictly positive.
    #[must_use]
    pub const fn aspect_ratio(&self) -> f64 {
        self.aspect_ratio
    }
}

/// Immutable one-way aerodynamic downwash interaction with resolved surface handles.
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeAeroDownwashInteraction {
    id: String,
    source_surface_index: usize,
    target_surface_index: usize,
    downwash_factor: f64,
}

impl RuntimeAeroDownwashInteraction {
    pub(crate) fn new(
        id: String,
        source_surface_index: usize,
        target_surface_index: usize,
        downwash_factor: f64,
    ) -> Self {
        Self {
            id,
            source_surface_index,
            target_surface_index,
            downwash_factor,
        }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    #[must_use]
    pub const fn source_surface_index(&self) -> usize {
        self.source_surface_index
    }

    #[must_use]
    pub const fn target_surface_index(&self) -> usize {
        self.target_surface_index
    }

    #[must_use]
    pub const fn downwash_factor(&self) -> f64 {
        self.downwash_factor
    }
}

/// Immutable one-way propeller-slipstream coupling with resolved element handles.
#[derive(Debug, Clone, PartialEq)]
pub struct RuntimePropellerSlipstreamInteraction {
    id: String,
    target_element_indices: Vec<usize>,
    slipstream_velocity_factor: f64,
    swirl_velocity_factor: f64,
}

impl RuntimePropellerSlipstreamInteraction {
    pub(crate) fn new(
        id: String,
        target_element_indices: Vec<usize>,
        slipstream_velocity_factor: f64,
        swirl_velocity_factor: f64,
    ) -> Self {
        Self {
            id,
            target_element_indices,
            slipstream_velocity_factor,
            swirl_velocity_factor,
        }
    }

    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    #[must_use]
    pub fn target_element_indices(&self) -> &[usize] {
        &self.target_element_indices
    }

    #[must_use]
    pub const fn slipstream_velocity_factor(&self) -> f64 {
        self.slipstream_velocity_factor
    }

    /// Tangential wake speed as a multiple of actuator-disk induced velocity.
    #[must_use]
    pub const fn swirl_velocity_factor(&self) -> f64 {
        self.swirl_velocity_factor
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeElectricPropulsion {
    config: ElectricPropulsionConfig,
    coefficient_source: PropellerCoefficientSource,
    propeller_rotational_inertia_kg_m2: f64,
}

impl RuntimeElectricPropulsion {
    pub(crate) const fn new(
        config: ElectricPropulsionConfig,
        coefficient_source: PropellerCoefficientSource,
    ) -> Self {
        Self {
            config,
            coefficient_source,
            propeller_rotational_inertia_kg_m2: 0.0,
        }
    }

    pub(crate) const fn with_propeller_rotational_inertia(
        mut self,
        propeller_rotational_inertia_kg_m2: f64,
    ) -> Self {
        self.propeller_rotational_inertia_kg_m2 = propeller_rotational_inertia_kg_m2;
        self
    }

    pub(crate) const fn new_legacy(
        config: ElectricPropulsionConfig,
        coefficient_table: PropellerCoefficientTable,
    ) -> Self {
        Self::new(
            config,
            PropellerCoefficientSource::FixedTable(coefficient_table),
        )
    }

    #[must_use]
    pub const fn config(&self) -> &ElectricPropulsionConfig {
        &self.config
    }

    #[must_use]
    pub const fn coefficient_source(&self) -> &PropellerCoefficientSource {
        &self.coefficient_source
    }

    /// Rotor polar moment of inertia about the configured propeller axis.
    #[must_use]
    pub const fn propeller_rotational_inertia_kg_m2(&self) -> f64 {
        self.propeller_rotational_inertia_kg_m2
    }

    /// Legacy fixed-table accessor. Schema-v4 map consumers should use `coefficient_source`.
    #[must_use]
    pub const fn coefficient_table(&self) -> &PropellerCoefficientTable {
        match &self.coefficient_source {
            PropellerCoefficientSource::FixedTable(table) => table,
            PropellerCoefficientSource::ShaftSpeedMap(_) => {
                panic!("shaft-speed map has no single coefficient table")
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PresentationMetadata {
    glb_path: String,
}

impl PresentationMetadata {
    pub(crate) fn new(glb_path: String) -> Self {
        Self { glb_path }
    }

    #[must_use]
    pub fn glb_path(&self) -> &str {
        &self.glb_path
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AircraftModelFingerprint([u8; 32]);

impl AircraftModelFingerprint {
    fn from_model(model: &AircraftModel) -> Self {
        let mut hasher = blake3::Hasher::new();
        let fingerprint_schema_version = match model.schema_version {
            AIRCRAFT_MODEL_SCHEMA_VERSION_V0 => {
                hasher.update(b"rcsim:aircraft-model:v0");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V0
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V1 | AIRCRAFT_MODEL_SCHEMA_VERSION_V2 => {
                // V2 adds documentary semantics only. Identical v1/v2 physics intentionally has
                // the same deterministic identity.
                hasher.update(b"rcsim:aircraft-model:v1");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V1
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V3 => {
                hasher.update(b"rcsim:aircraft-model:v3");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V3
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V4 => {
                hasher.update(b"rcsim:aircraft-model:v4");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V4
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V5 => {
                hasher.update(b"rcsim:aircraft-model:v5");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V5
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V6 => {
                hasher.update(b"rcsim:aircraft-model:v6");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V6
            }
            AIRCRAFT_MODEL_SCHEMA_VERSION_V7 => {
                hasher.update(b"rcsim:aircraft-model:v7");
                AIRCRAFT_MODEL_SCHEMA_VERSION_V7
            }
            _ => unreachable!("runtime models are created only from supported schemas"),
        };
        hasher.update(&fingerprint_schema_version.to_le_bytes());

        update_f64(&mut hasher, model.rigid_body.mass_kg());
        let inertia = model.rigid_body.inertia_body_kg_m2();
        for row in 0..3 {
            for column in 0..3 {
                update_f64(&mut hasher, inertia[(row, column)]);
            }
        }

        update_len(&mut hasher, model.aero_polars.len());
        for polar in &model.aero_polars {
            update_len(&mut hasher, polar.table.samples().len());
            for sample in polar.table.samples() {
                for value in [sample.alpha_rad, sample.cl, sample.cd, sample.cm] {
                    update_f64(&mut hasher, value);
                }
            }
        }

        if matches!(
            model.schema_version,
            AIRCRAFT_MODEL_SCHEMA_VERSION_V3
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
        ) {
            update_f64(
                &mut hasher,
                model
                    .kinematic_viscosity_m2_s
                    .expect("schema v3+ models have explicit viscosity"),
            );
            hasher.update(b"reynolds-family:ln-re-clamped:v1");
            update_len(&mut hasher, model.aero_polar_families.len());
            for runtime_family in &model.aero_polar_families {
                update_len(&mut hasher, runtime_family.family.nodes().len());
                for node in runtime_family.family.nodes() {
                    update_f64(&mut hasher, node.reynolds_number());
                    update_len(&mut hasher, node.table().samples().len());
                    for sample in node.table().samples() {
                        for value in [sample.alpha_rad, sample.cl, sample.cd, sample.cm] {
                            update_f64(&mut hasher, value);
                        }
                    }
                }
            }
        }

        update_len(&mut hasher, model.aero_elements.len());
        for runtime_element in &model.aero_elements {
            let element = &runtime_element.element;
            update_vector(&mut hasher, element.position_body_m().as_slice());
            update_orientation(
                &mut hasher,
                element.orientation_body_from_element().quaternion(),
            );
            update_f64(&mut hasher, element.area_m2());
            update_f64(&mut hasher, element.chord_m());
            match runtime_element.polar_binding {
                RuntimeAeroPolarBinding::Polar { polar_index } => {
                    if matches!(
                        model.schema_version,
                        AIRCRAFT_MODEL_SCHEMA_VERSION_V3
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
                    ) {
                        hasher.update(&[0]);
                    }
                    update_len(&mut hasher, polar_index);
                }
                RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
                    debug_assert!(matches!(
                        model.schema_version,
                        AIRCRAFT_MODEL_SCHEMA_VERSION_V3
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                            | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
                    ));
                    hasher.update(&[1]);
                    update_len(&mut hasher, family_index);
                }
            }
        }

        let response = model.controls.response();
        for axis in [response.roll(), response.pitch(), response.yaw()] {
            update_f64(&mut hasher, axis.rate());
            update_f64(&mut hasher, axis.expo());
        }
        let actuators = model.controls.actuators();
        for servo in [
            actuators.aileron(),
            actuators.elevator(),
            actuators.rudder(),
        ] {
            update_f64(&mut hasher, servo.min_angle_rad());
            update_f64(&mut hasher, servo.neutral_angle_rad());
            update_f64(&mut hasher, servo.max_angle_rad());
            update_f64(&mut hasher, servo.max_speed_rad_s());
            hasher.update(&[u8::from(servo.reversed())]);
        }

        match &model.propulsion {
            None => {
                hasher.update(&[0]);
            }
            Some(runtime_propulsion) => {
                hasher.update(&[1]);
                let config = runtime_propulsion.config();
                let battery = config.battery();
                update_f64(&mut hasher, battery.open_circuit_voltage_v());
                update_f64(&mut hasher, battery.internal_resistance_ohm());
                if matches!(
                    model.schema_version,
                    AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                        | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                        | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                        | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
                ) {
                    hasher.update(b"esc:series-resistance:v1");
                    update_f64(&mut hasher, config.esc().series_resistance_ohm());
                }
                let motor = config.motor();
                update_f64(&mut hasher, motor.kv_rpm_per_v());
                update_f64(&mut hasher, motor.winding_resistance_ohm());
                update_f64(&mut hasher, motor.no_load_current_a());
                let propeller = config.propeller();
                update_vector(&mut hasher, propeller.position_body_m().as_slice());
                update_orientation(
                    &mut hasher,
                    propeller.orientation_body_from_prop().quaternion(),
                );
                update_f64(&mut hasher, propeller.diameter_m());
                let spin_tag = match propeller.spin_direction() {
                    PropellerSpinDirection::PositiveAboutLocalX => 0,
                    PropellerSpinDirection::NegativeAboutLocalX => 1,
                };
                hasher.update(&[spin_tag]);
                if runtime_propulsion.propeller_rotational_inertia_kg_m2 != 0.0 {
                    hasher.update(b"propeller-rotational-inertia:v1");
                    update_f64(
                        &mut hasher,
                        runtime_propulsion.propeller_rotational_inertia_kg_m2,
                    );
                }
                match runtime_propulsion.coefficient_source() {
                    PropellerCoefficientSource::FixedTable(table) => {
                        if matches!(
                            model.schema_version,
                            AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
                        ) {
                            hasher
                                .update(b"propeller-coefficients:fixed-table:j-linear-clamped:v1");
                        }
                        update_len(&mut hasher, table.samples().len());
                        for sample in table.samples() {
                            for value in [sample.advance_ratio_j, sample.ct, sample.cq] {
                                update_f64(&mut hasher, value);
                            }
                        }
                    }
                    PropellerCoefficientSource::ShaftSpeedMap(map) => {
                        debug_assert!(matches!(
                            model.schema_version,
                            AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                                | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
                        ));
                        hasher.update(
                            b"propeller-coefficients:shaft-speed-linear:j-linear-clamped:v1",
                        );
                        update_len(&mut hasher, map.nodes().len());
                        for node in map.nodes() {
                            update_f64(&mut hasher, node.shaft_speed_rad_s());
                            update_len(&mut hasher, node.table().samples().len());
                            for sample in node.table().samples() {
                                for value in [sample.advance_ratio_j, sample.ct, sample.cq] {
                                    update_f64(&mut hasher, value);
                                }
                            }
                        }
                    }
                }
            }
        }

        if matches!(
            model.schema_version,
            AIRCRAFT_MODEL_SCHEMA_VERSION_V1
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V2
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V3
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V4
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
        ) {
            update_len(&mut hasher, model.control_surface_bindings.len());
            for binding in &model.control_surface_bindings {
                update_len(&mut hasher, binding.element_index);
                let actuator_tag = match binding.actuator {
                    ControlActuator::Aileron => 0,
                    ControlActuator::Elevator => 1,
                    ControlActuator::Rudder => 2,
                };
                hasher.update(&[actuator_tag]);
                update_f64(&mut hasher, binding.deflection_gain);
            }
        }

        if matches!(
            model.schema_version,
            AIRCRAFT_MODEL_SCHEMA_VERSION_V5
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V6
                | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
        ) {
            hasher.update(b"aero-surfaces:v1");
            update_len(&mut hasher, model.aero_surfaces.len());
            for surface in &model.aero_surfaces {
                update_len(&mut hasher, surface.element_indices.len());
                for &element_index in &surface.element_indices {
                    update_len(&mut hasher, element_index);
                }
                update_vector(&mut hasher, surface.span_axis_body.as_slice());
                update_f64(&mut hasher, surface.span_m);
                update_f64(&mut hasher, surface.span_efficiency_factor);
                update_f64(&mut hasher, surface.area_m2);
                update_f64(&mut hasher, surface.aspect_ratio);
            }
        }

        if matches!(
            model.schema_version,
            AIRCRAFT_MODEL_SCHEMA_VERSION_V6 | AIRCRAFT_MODEL_SCHEMA_VERSION_V7
        ) {
            hasher.update(b"aero-downwash-interactions:v1");
            update_len(&mut hasher, model.aero_downwash_interactions.len());
            for interaction in &model.aero_downwash_interactions {
                update_len(&mut hasher, interaction.source_surface_index);
                update_len(&mut hasher, interaction.target_surface_index);
                update_f64(&mut hasher, interaction.downwash_factor);
            }
        }

        if model.schema_version == AIRCRAFT_MODEL_SCHEMA_VERSION_V7 {
            hasher.update(b"propeller-slipstream-interactions:v1");
            update_len(&mut hasher, model.propeller_slipstream_interactions.len());
            for interaction in &model.propeller_slipstream_interactions {
                update_len(&mut hasher, interaction.target_element_indices.len());
                for &element_index in &interaction.target_element_indices {
                    update_len(&mut hasher, element_index);
                }
                update_f64(&mut hasher, interaction.slipstream_velocity_factor);
            }
            if model
                .propeller_slipstream_interactions
                .iter()
                .any(|interaction| interaction.swirl_velocity_factor != 0.0)
            {
                hasher.update(b"propeller-swirl:v1");
                for interaction in &model.propeller_slipstream_interactions {
                    update_f64(&mut hasher, interaction.swirl_velocity_factor);
                }
            }
        }

        Self(*hasher.finalize().as_bytes())
    }

    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

fn update_len(hasher: &mut blake3::Hasher, length: usize) {
    let length = u64::try_from(length).expect("model collections fit in u64");
    hasher.update(&length.to_le_bytes());
}

fn update_f64(hasher: &mut blake3::Hasher, value: f64) {
    hasher.update(&value.to_bits().to_le_bytes());
}

fn update_vector(hasher: &mut blake3::Hasher, values: &[f64]) {
    for &value in values {
        update_f64(hasher, value);
    }
}

fn update_orientation(hasher: &mut blake3::Hasher, quaternion: &sim_math::Quaternion<f64>) {
    for value in [quaternion.w, quaternion.i, quaternion.j, quaternion.k] {
        update_f64(hasher, value);
    }
}
