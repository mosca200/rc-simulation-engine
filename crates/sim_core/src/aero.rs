use crate::{BodyWrench, ReynoldsPolarFamily, ReynoldsPolarSample, RigidBodyState};
use sim_math::{Orientation, Vec3, world_to_body};
use thiserror::Error;

/// Below this quasi-2D section speed, aerodynamic directions are treated as singular.
pub const MIN_SECTION_AIRSPEED_MPS: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolarSample {
    pub alpha_rad: f64,
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PolarCoefficients {
    pub cl: f64,
    pub cd: f64,
    pub cm: f64,
}

impl From<PolarSample> for PolarCoefficients {
    fn from(sample: PolarSample) -> Self {
        Self {
            cl: sample.cl,
            cd: sample.cd,
            cm: sample.cm,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PolarTable {
    samples: Vec<PolarSample>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PolarError {
    #[error("polar table requires at least two samples")]
    TooFewSamples,
    #[error("polar sample {index} contains a non-finite value")]
    NonFiniteSample { index: usize },
    #[error("polar alpha must be strictly increasing at sample {index}")]
    NonIncreasingAlpha { index: usize },
    #[error("polar drag coefficient must be non-negative at sample {index}")]
    NegativeDragCoefficient { index: usize },
}

impl PolarTable {
    pub fn new(samples: Vec<PolarSample>) -> Result<Self, PolarError> {
        if samples.len() < 2 {
            return Err(PolarError::TooFewSamples);
        }
        for (index, sample) in samples.iter().enumerate() {
            if ![sample.alpha_rad, sample.cl, sample.cd, sample.cm]
                .into_iter()
                .all(f64::is_finite)
            {
                return Err(PolarError::NonFiniteSample { index });
            }
            if sample.cd < 0.0 {
                return Err(PolarError::NegativeDragCoefficient { index });
            }
            if index > 0 && sample.alpha_rad <= samples[index - 1].alpha_rad {
                return Err(PolarError::NonIncreasingAlpha { index });
            }
        }
        Ok(Self { samples })
    }

    /// Piecewise-linear interpolation with exact endpoint preservation and endpoint clamping.
    #[must_use]
    pub fn sample_clamped(&self, alpha_rad: f64) -> PolarCoefficients {
        debug_assert!(alpha_rad.is_finite());
        let first = self.samples[0];
        if alpha_rad <= first.alpha_rad {
            return first.into();
        }
        let last = self.samples[self.samples.len() - 1];
        if alpha_rad >= last.alpha_rad {
            return last.into();
        }

        let mut lower = 0;
        let mut upper = self.samples.len() - 1;
        while upper - lower > 1 {
            let middle = lower + (upper - lower) / 2;
            if alpha_rad < self.samples[middle].alpha_rad {
                upper = middle;
            } else {
                lower = middle;
            }
        }

        let lower_sample = self.samples[lower];
        if alpha_rad == lower_sample.alpha_rad {
            return lower_sample.into();
        }
        let upper_sample = self.samples[upper];
        if alpha_rad == upper_sample.alpha_rad {
            return upper_sample.into();
        }
        let fraction = (alpha_rad - lower_sample.alpha_rad)
            / (upper_sample.alpha_rad - lower_sample.alpha_rad);
        PolarCoefficients {
            cl: lower_sample.cl + fraction * (upper_sample.cl - lower_sample.cl),
            cd: lower_sample.cd + fraction * (upper_sample.cd - lower_sample.cd),
            cm: lower_sample.cm + fraction * (upper_sample.cm - lower_sample.cm),
        }
    }

    #[must_use]
    pub fn samples(&self) -> &[PolarSample] {
        &self.samples
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroElement {
    position_body_m: Vec3,
    orientation_body_from_element: Orientation,
    area_m2: f64,
    chord_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum AeroElementError {
    #[error("aerodynamic-element position must be finite")]
    NonFinitePosition,
    #[error("aerodynamic-element orientation must be finite and unit length")]
    InvalidOrientation,
    #[error("aerodynamic-element area must be finite and greater than zero")]
    InvalidArea,
    #[error("aerodynamic-element chord must be finite and greater than zero")]
    InvalidChord,
}

impl AeroElement {
    pub fn new(
        position_body_m: Vec3,
        orientation_body_from_element: Orientation,
        area_m2: f64,
        chord_m: f64,
    ) -> Result<Self, AeroElementError> {
        if !position_body_m.iter().all(|value| value.is_finite()) {
            return Err(AeroElementError::NonFinitePosition);
        }
        let quaternion = orientation_body_from_element.quaternion();
        if ![quaternion.w, quaternion.i, quaternion.j, quaternion.k]
            .into_iter()
            .all(f64::is_finite)
            || (quaternion.norm_squared() - 1.0).abs() > 1.0e-12
        {
            return Err(AeroElementError::InvalidOrientation);
        }
        if !area_m2.is_finite() || area_m2 <= 0.0 {
            return Err(AeroElementError::InvalidArea);
        }
        if !chord_m.is_finite() || chord_m <= 0.0 {
            return Err(AeroElementError::InvalidChord);
        }
        Ok(Self {
            position_body_m,
            orientation_body_from_element,
            area_m2,
            chord_m,
        })
    }

    #[must_use]
    pub const fn position_body_m(&self) -> &Vec3 {
        &self.position_body_m
    }

    #[must_use]
    pub const fn orientation_body_from_element(&self) -> &Orientation {
        &self.orientation_body_from_element
    }

    #[must_use]
    pub const fn area_m2(&self) -> f64 {
        self.area_m2
    }

    #[must_use]
    pub const fn chord_m(&self) -> f64 {
        self.chord_m
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroEnvironment {
    air_density_kg_m3: f64,
    wind_velocity_world_mps: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum AeroEnvironmentError {
    #[error("air density must be finite and non-negative")]
    InvalidAirDensity,
    #[error("world-frame wind velocity must be finite")]
    NonFiniteWind,
}

impl AeroEnvironment {
    pub fn new(
        air_density_kg_m3: f64,
        wind_velocity_world_mps: Vec3,
    ) -> Result<Self, AeroEnvironmentError> {
        if !air_density_kg_m3.is_finite() || air_density_kg_m3 < 0.0 {
            return Err(AeroEnvironmentError::InvalidAirDensity);
        }
        if !wind_velocity_world_mps
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(AeroEnvironmentError::NonFiniteWind);
        }
        Ok(Self {
            air_density_kg_m3,
            wind_velocity_world_mps,
        })
    }

    #[must_use]
    pub const fn air_density_kg_m3(&self) -> f64 {
        self.air_density_kg_m3
    }

    #[must_use]
    pub const fn wind_velocity_world_mps(&self) -> &Vec3 {
        &self.wind_velocity_world_mps
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroElementOutput {
    /// Velocity of the element through the air, expressed in the element frame.
    pub air_relative_velocity_element_mps: Vec3,
    pub section_airspeed_mps: f64,
    pub alpha_rad: f64,
    pub beta_rad: f64,
    pub dynamic_pressure_pa: f64,
    pub coefficients: PolarCoefficients,
    pub force_element_n: Vec3,
    pub wrench_body: BodyWrench,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum ReynoldsCalculationError {
    #[error("section airspeed must be finite and non-negative")]
    InvalidSectionAirspeed,
    #[error("section chord must be finite and greater than zero")]
    InvalidChord,
    #[error("kinematic viscosity must be finite and greater than zero")]
    InvalidKinematicViscosity,
    #[error("computed Reynolds number must be finite and non-negative")]
    InvalidResult,
}

/// Allocation-free Reynolds-aware aerodynamic result with borrowed family diagnostics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReynoldsAeroElementOutput<'a> {
    pub aero: AeroElementOutput,
    pub local_reynolds: f64,
    pub reynolds_sample: ReynoldsPolarSample<'a>,
}

/// Computes section Reynolds number from local quasi-2D speed, chord, and explicit viscosity.
pub fn calculate_reynolds_number(
    section_airspeed_mps: f64,
    chord_m: f64,
    kinematic_viscosity_m2_s: f64,
) -> Result<f64, ReynoldsCalculationError> {
    if !section_airspeed_mps.is_finite() || section_airspeed_mps < 0.0 {
        return Err(ReynoldsCalculationError::InvalidSectionAirspeed);
    }
    if !chord_m.is_finite() || chord_m <= 0.0 {
        return Err(ReynoldsCalculationError::InvalidChord);
    }
    if !kinematic_viscosity_m2_s.is_finite() || kinematic_viscosity_m2_s <= 0.0 {
        return Err(ReynoldsCalculationError::InvalidKinematicViscosity);
    }
    let reynolds_number = section_airspeed_mps * chord_m / kinematic_viscosity_m2_s;
    if !reynolds_number.is_finite() || reynolds_number < 0.0 {
        return Err(ReynoldsCalculationError::InvalidResult);
    }
    Ok(reynolds_number)
}

/// Assembles a body wrench from pre-computed section kinematics and polar coefficients.
///
/// This is the shared force-construction primitive used by both the legacy quasi-2D
/// evaluation path and the finite-wing surface evaluation path. The key invariant:
/// force directions come from the ACTUAL local section flow (encoded in `kinematics`),
/// while the coefficients may have been sampled at a different effective alpha.
///
/// Lift direction = spanwise(y) × section-flow-hat
/// Drag direction = -section-flow-hat
/// Force = lift_dir * (q * S * CL) + drag_dir * (q * S * CD)
/// Moment = r × F + intrinsic CM pitch moment
#[must_use]
pub fn assemble_aero_element_wrench(
    element: &AeroElement,
    kinematics: &SectionKinematics,
    coefficients: &PolarCoefficients,
) -> BodyWrench {
    let q = kinematics.dynamic_pressure_pa;
    let s = element.area_m2();

    let velocity_hat_section = if kinematics.section_airspeed_mps > 0.0 {
        Vec3::new(
            kinematics.air_relative_velocity_element_mps.x / kinematics.section_airspeed_mps,
            0.0,
            kinematics.air_relative_velocity_element_mps.z / kinematics.section_airspeed_mps,
        )
    } else {
        Vec3::zeros()
    };
    let drag_direction = -velocity_hat_section;
    let lift_direction = Vec3::y().cross(&velocity_hat_section);

    let lift_n = q * s * coefficients.cl;
    let drag_n = q * s * coefficients.cd;
    let force_element = lift_direction * lift_n + drag_direction * drag_n;
    let force_body = element
        .orientation_body_from_element
        .transform_vector(&force_element);

    let intrinsic_pitch_nm = q * s * element.chord_m * coefficients.cm;
    let intrinsic_moment_body = element
        .orientation_body_from_element
        .transform_vector(&Vec3::new(0.0, intrinsic_pitch_nm, 0.0));

    let mut wrench = BodyWrench::zero();
    wrench.add_force_at_body_point(force_body, element.position_body_m);
    wrench.add_moment_body(intrinsic_moment_body);
    wrench
}

/// Evaluates one immutable quasi-2D aerodynamic element without allocation.
#[must_use]
pub fn evaluate_aero_element(
    state: &RigidBodyState,
    element: &AeroElement,
    environment: &AeroEnvironment,
    polar: &PolarTable,
) -> AeroElementOutput {
    evaluate_aero_element_with_sampler(state, element, environment, |_, alpha_rad| {
        (polar.sample_clamped(alpha_rad), ())
    })
    .0
}

/// Section-plane kinematics for one quasi-2D element at a given RK4 stage state.
///
/// This primitive computes the air-relative velocity decomposition, angle of attack,
/// sideslip, and dynamic pressure without sampling any polar or assembling forces.
/// It is the reusable building block for finite-wing induced-angle solvers that need
/// to evaluate section kinematics repeatedly with different effective alpha values
/// while keeping force directions tied to the actual local flow.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionKinematics {
    pub air_relative_velocity_element_mps: Vec3,
    pub section_airspeed_mps: f64,
    pub alpha_rad: f64,
    pub beta_rad: f64,
    pub dynamic_pressure_pa: f64,
}

/// Computes section-plane kinematics for one element without polar sampling or force assembly.
///
/// The velocity transformation chain is identical to [`evaluate_aero_element`]:
/// world-relative wind → body-frame at CG → add rotational contribution → element frame.
/// The section airspeed uses only the chordwise (u) and normal (w) components (quasi-2D).
/// Below [`MIN_SECTION_AIRSPEED_MPS`], alpha and dynamic pressure are zero.
#[must_use]
pub fn compute_section_kinematics(
    state: &RigidBodyState,
    element: &AeroElement,
    environment: &AeroEnvironment,
) -> SectionKinematics {
    let air_relative_velocity_world_mps =
        state.linear_velocity_world_mps - environment.wind_velocity_world_mps;
    let air_relative_velocity_body_at_cg_mps = world_to_body(
        &state.orientation_world_from_body,
        &air_relative_velocity_world_mps,
    );
    let rotational_velocity_body_mps = state
        .angular_velocity_body_radps
        .cross(&element.position_body_m);
    let air_relative_velocity_body_at_element_mps =
        air_relative_velocity_body_at_cg_mps + rotational_velocity_body_mps;
    let air_relative_velocity_element_mps = element
        .orientation_body_from_element
        .inverse_transform_vector(&air_relative_velocity_body_at_element_mps);

    let u_mps = air_relative_velocity_element_mps.x;
    let spanwise_mps = air_relative_velocity_element_mps.y;
    let w_mps = air_relative_velocity_element_mps.z;
    let section_speed_squared_mps2 = u_mps.mul_add(u_mps, w_mps * w_mps);
    let section_airspeed_mps = section_speed_squared_mps2.sqrt();
    let beta_rad = spanwise_mps.atan2(section_airspeed_mps);

    if section_airspeed_mps < MIN_SECTION_AIRSPEED_MPS {
        return SectionKinematics {
            air_relative_velocity_element_mps,
            section_airspeed_mps,
            alpha_rad: 0.0,
            beta_rad,
            dynamic_pressure_pa: 0.0,
        };
    }

    let alpha_rad = w_mps.atan2(u_mps);
    let dynamic_pressure_pa = 0.5 * environment.air_density_kg_m3 * section_speed_squared_mps2;

    SectionKinematics {
        air_relative_velocity_element_mps,
        section_airspeed_mps,
        alpha_rad,
        beta_rad,
        dynamic_pressure_pa,
    }
}

/// Evaluates one Reynolds-aware quasi-2D element from its local RK4-stage velocity.
#[must_use]
pub fn evaluate_reynolds_aero_element<'a>(
    state: &RigidBodyState,
    element: &AeroElement,
    environment: &AeroEnvironment,
    polar_family: &'a ReynoldsPolarFamily,
    kinematic_viscosity_m2_s: f64,
) -> ReynoldsAeroElementOutput<'a> {
    debug_assert!(kinematic_viscosity_m2_s.is_finite() && kinematic_viscosity_m2_s > 0.0);
    let (aero, (local_reynolds, reynolds_sample)) = evaluate_aero_element_with_sampler(
        state,
        element,
        environment,
        |section_airspeed_mps, alpha_rad| {
            let local_reynolds = calculate_reynolds_number(
                section_airspeed_mps,
                element.chord_m,
                kinematic_viscosity_m2_s,
            )
            .expect("validated stage state, element, and viscosity produce finite Reynolds");
            let sample = polar_family.sample(local_reynolds, alpha_rad);
            (sample.coefficients, (local_reynolds, sample))
        },
    );
    ReynoldsAeroElementOutput {
        aero,
        local_reynolds,
        reynolds_sample,
    }
}

fn evaluate_aero_element_with_sampler<T, F>(
    state: &RigidBodyState,
    element: &AeroElement,
    environment: &AeroEnvironment,
    sample_coefficients: F,
) -> (AeroElementOutput, T)
where
    F: FnOnce(f64, f64) -> (PolarCoefficients, T),
{
    debug_assert!(state.validate().is_ok());

    let air_relative_velocity_world_mps =
        state.linear_velocity_world_mps - environment.wind_velocity_world_mps;
    let air_relative_velocity_body_at_cg_mps = world_to_body(
        &state.orientation_world_from_body,
        &air_relative_velocity_world_mps,
    );
    let rotational_velocity_body_mps = state
        .angular_velocity_body_radps
        .cross(&element.position_body_m);
    let air_relative_velocity_body_at_element_mps =
        air_relative_velocity_body_at_cg_mps + rotational_velocity_body_mps;
    let air_relative_velocity_element_mps = element
        .orientation_body_from_element
        .inverse_transform_vector(&air_relative_velocity_body_at_element_mps);

    let u_mps = air_relative_velocity_element_mps.x;
    let spanwise_mps = air_relative_velocity_element_mps.y;
    let w_mps = air_relative_velocity_element_mps.z;
    let section_speed_squared_mps2 = u_mps.mul_add(u_mps, w_mps * w_mps);
    let section_airspeed_mps = section_speed_squared_mps2.sqrt();
    let beta_rad = spanwise_mps.atan2(section_airspeed_mps);

    if section_airspeed_mps < MIN_SECTION_AIRSPEED_MPS {
        let (coefficients, diagnostic) = sample_coefficients(section_airspeed_mps, 0.0);
        return (
            AeroElementOutput {
                air_relative_velocity_element_mps,
                section_airspeed_mps,
                alpha_rad: 0.0,
                beta_rad,
                dynamic_pressure_pa: 0.0,
                coefficients,
                force_element_n: Vec3::zeros(),
                wrench_body: BodyWrench::zero(),
            },
            diagnostic,
        );
    }

    let alpha_rad = w_mps.atan2(u_mps);
    let (coefficients, diagnostic) = sample_coefficients(section_airspeed_mps, alpha_rad);
    let dynamic_pressure_pa = 0.5 * environment.air_density_kg_m3 * section_speed_squared_mps2;

    let kinematics = SectionKinematics {
        air_relative_velocity_element_mps,
        section_airspeed_mps,
        alpha_rad,
        beta_rad,
        dynamic_pressure_pa,
    };
    let wrench_body = assemble_aero_element_wrench(element, &kinematics, &coefficients);

    let velocity_hat_section = Vec3::new(
        u_mps / section_airspeed_mps,
        0.0,
        w_mps / section_airspeed_mps,
    );
    let force_element_n = Vec3::y().cross(&velocity_hat_section)
        * (dynamic_pressure_pa * element.area_m2 * coefficients.cl)
        + (-velocity_hat_section) * (dynamic_pressure_pa * element.area_m2 * coefficients.cd);

    (
        AeroElementOutput {
            air_relative_velocity_element_mps,
            section_airspeed_mps,
            alpha_rad,
            beta_rad,
            dynamic_pressure_pa,
            coefficients,
            force_element_n,
            wrench_body,
        },
        diagnostic,
    )
}
