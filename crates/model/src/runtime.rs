use crate::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V0, AIRCRAFT_MODEL_SCHEMA_VERSION_V1,
    AIRCRAFT_MODEL_SCHEMA_VERSION_V2, AircraftClassification, ReferenceAircraftMetadata,
};
use sim_core::{
    AeroElement, ControlSystemConfig, ElectricPropulsionConfig, PolarTable,
    PropellerCoefficientTable, PropellerSpinDirection, RigidBodyParams,
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
    aero_elements: Vec<RuntimeAeroElement>,
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
            aero_elements,
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
    pub fn aero_elements(&self) -> &[RuntimeAeroElement] {
        &self.aero_elements
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
    polar_index: usize,
}

impl RuntimeAeroElement {
    pub(crate) fn new(id: String, element: AeroElement, polar_index: usize) -> Self {
        Self {
            id,
            element,
            polar_index,
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
        self.polar_index
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct RuntimeElectricPropulsion {
    config: ElectricPropulsionConfig,
    coefficient_table: PropellerCoefficientTable,
}

impl RuntimeElectricPropulsion {
    pub(crate) const fn new(
        config: ElectricPropulsionConfig,
        coefficient_table: PropellerCoefficientTable,
    ) -> Self {
        Self {
            config,
            coefficient_table,
        }
    }

    #[must_use]
    pub const fn config(&self) -> &ElectricPropulsionConfig {
        &self.config
    }

    #[must_use]
    pub const fn coefficient_table(&self) -> &PropellerCoefficientTable {
        &self.coefficient_table
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
            update_len(&mut hasher, runtime_element.polar_index);
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
                let samples = runtime_propulsion.coefficient_table().samples();
                update_len(&mut hasher, samples.len());
                for sample in samples {
                    for value in [sample.advance_ratio_j, sample.ct, sample.cq] {
                        update_f64(&mut hasher, value);
                    }
                }
            }
        }

        if matches!(
            model.schema_version,
            AIRCRAFT_MODEL_SCHEMA_VERSION_V1 | AIRCRAFT_MODEL_SCHEMA_VERSION_V2
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
