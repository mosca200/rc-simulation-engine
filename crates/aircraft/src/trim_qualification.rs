//! M2.6C — Deterministic longitudinal trim domain qualification.
//!
//! A converged trim solution is NOT automatically qualified. This module determines whether a
//! converged trim operating point lies inside the aerodynamic and propulsion evidence domains
//! and whether its off-axis residuals satisfy caller-supplied limits.
//!
//! # Core principle: fail closed
//!
//! Runtime samplers clamp outside their tabulated domains. Qualification distinguishes
//! "runtime can evaluate this value" from "this operating point is inside the supported
//! evidence domain". A solution relying on endpoint clamping is NOT qualified.
//!
//! # Finite-wing alpha_sample
//!
//! For elements belonging to a [`RuntimeAeroSurface`], runtime samples coefficients at
//! `alpha_sample = alpha_geom - alpha_i`. Qualification audits `alpha_sample`, NOT `alpha_geom`.
//! The induced angle `alpha_i` is obtained from the SAME deterministic bisection used by runtime.
//!
//! # Physical Reynolds
//!
//! Reynolds numbers use the PHYSICAL section airspeed, not any effective-alpha-modified velocity.

use crate::{
    AircraftSimulationConfig,
    simulation::solve_surface_induced_alpha,
    trim::{
        LongitudinalTrimSolution, LongitudinalTrimVariables, evaluate_longitudinal_trim_candidate,
    },
};
use model::AircraftModel;
use sim_core::{
    PolarTable, PropellerCoefficientMap, PropellerCoefficientSource, RigidBodyState,
    ShaftSpeedRangeStatus, compute_section_kinematics, evaluate_electric_propulsion_with_source,
};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Range status
// ---------------------------------------------------------------------------

/// Whether a value lies inside, below, or above its evidence support.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeStatus {
    BelowRange,
    InRange,
    AboveRange,
}

// ---------------------------------------------------------------------------
// Qualification blockers
// ---------------------------------------------------------------------------

/// Typed reason a trim point is not qualified.
#[derive(Debug, Clone, PartialEq)]
pub enum QualificationBlocker {
    // Aero alpha
    AerodynamicAlphaBelowRange {
        element_index: usize,
        element_id: String,
        alpha_sample_rad: f64,
        alpha_lower_rad: f64,
    },
    AerodynamicAlphaAboveRange {
        element_index: usize,
        element_id: String,
        alpha_sample_rad: f64,
        alpha_upper_rad: f64,
    },

    // Reynolds
    ReynoldsBelowRange {
        element_index: usize,
        element_id: String,
        reynolds_number: f64,
        reynolds_lower: f64,
    },
    ReynoldsAboveRange {
        element_index: usize,
        element_id: String,
        reynolds_number: f64,
        reynolds_upper: f64,
    },
    ReynoldsContributingNodeAlphaBelowRange {
        element_index: usize,
        element_id: String,
        node_reynolds: f64,
        alpha_sample_rad: f64,
        alpha_lower_rad: f64,
    },
    ReynoldsContributingNodeAlphaAboveRange {
        element_index: usize,
        element_id: String,
        node_reynolds: f64,
        alpha_sample_rad: f64,
        alpha_upper_rad: f64,
    },

    // Propulsion J
    PropellerAdvanceRatioBelowRange {
        advance_ratio_j: f64,
        j_lower: f64,
    },
    PropellerAdvanceRatioAboveRange {
        advance_ratio_j: f64,
        j_upper: f64,
    },

    // Propulsion shaft speed
    PropellerShaftSpeedBelowRange {
        shaft_speed_rad_s: f64,
        shaft_speed_lower_rad_s: f64,
    },
    PropellerShaftSpeedAboveRange {
        shaft_speed_rad_s: f64,
        shaft_speed_upper_rad_s: f64,
    },

    // Off-axis residual limits
    SideForceLimitExceeded {
        fy_body_n: f64,
        limit_n: f64,
    },
    RollMomentLimitExceeded {
        mx_body_nm: f64,
        limit_nm: f64,
    },
    YawMomentLimitExceeded {
        mz_body_nm: f64,
        limit_nm: f64,
    },
    LateralAccelerationLimitExceeded {
        ay_world_mps2: f64,
        limit_mps2: f64,
    },
    RollAngularAccelerationLimitExceeded {
        angular_accel_body_x_rad_s2: f64,
        limit_rad_s2: f64,
    },
    YawAngularAccelerationLimitExceeded {
        angular_accel_body_z_rad_s2: f64,
        limit_rad_s2: f64,
    },

    // Integrity
    NonFiniteAuditValue {
        field: &'static str,
    },
    ReEvaluationFailure,
}

// ---------------------------------------------------------------------------
// Qualification limits
// ---------------------------------------------------------------------------

/// Caller-supplied maxima for off-axis residuals. No hidden defaults.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LongitudinalTrimQualificationLimits {
    max_side_force_n: f64,
    max_roll_moment_nm: f64,
    max_yaw_moment_nm: f64,
    max_lateral_acceleration_mps2: f64,
    max_roll_angular_acceleration_rad_s2: f64,
    max_yaw_angular_acceleration_rad_s2: f64,
}

/// Errors from constructing [`LongitudinalTrimQualificationLimits`].
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum QualificationLimitsError {
    #[error("limit field `{field}` must be finite, got {value}")]
    NonFinite { field: &'static str, value: f64 },
    #[error("limit field `{field}` must be non-negative, got {value}")]
    Negative { field: &'static str, value: f64 },
}

impl LongitudinalTrimQualificationLimits {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        max_side_force_n: f64,
        max_roll_moment_nm: f64,
        max_yaw_moment_nm: f64,
        max_lateral_acceleration_mps2: f64,
        max_roll_angular_acceleration_rad_s2: f64,
        max_yaw_angular_acceleration_rad_s2: f64,
    ) -> Result<Self, QualificationLimitsError> {
        let limits = [
            ("max_side_force_n", max_side_force_n),
            ("max_roll_moment_nm", max_roll_moment_nm),
            ("max_yaw_moment_nm", max_yaw_moment_nm),
            (
                "max_lateral_acceleration_mps2",
                max_lateral_acceleration_mps2,
            ),
            (
                "max_roll_angular_acceleration_rad_s2",
                max_roll_angular_acceleration_rad_s2,
            ),
            (
                "max_yaw_angular_acceleration_rad_s2",
                max_yaw_angular_acceleration_rad_s2,
            ),
        ];
        for &(field, value) in &limits {
            if !value.is_finite() {
                return Err(QualificationLimitsError::NonFinite { field, value });
            }
            if value < 0.0 {
                return Err(QualificationLimitsError::Negative { field, value });
            }
        }
        Ok(Self {
            max_side_force_n,
            max_roll_moment_nm,
            max_yaw_moment_nm,
            max_lateral_acceleration_mps2,
            max_roll_angular_acceleration_rad_s2,
            max_yaw_angular_acceleration_rad_s2,
        })
    }

    #[must_use]
    pub const fn max_side_force_n(&self) -> f64 {
        self.max_side_force_n
    }
    #[must_use]
    pub const fn max_roll_moment_nm(&self) -> f64 {
        self.max_roll_moment_nm
    }
    #[must_use]
    pub const fn max_yaw_moment_nm(&self) -> f64 {
        self.max_yaw_moment_nm
    }
    #[must_use]
    pub const fn max_lateral_acceleration_mps2(&self) -> f64 {
        self.max_lateral_acceleration_mps2
    }
    #[must_use]
    pub const fn max_roll_angular_acceleration_rad_s2(&self) -> f64 {
        self.max_roll_angular_acceleration_rad_s2
    }
    #[must_use]
    pub const fn max_yaw_angular_acceleration_rad_s2(&self) -> f64 {
        self.max_yaw_angular_acceleration_rad_s2
    }
}

// ---------------------------------------------------------------------------
// Full residual audit
// ---------------------------------------------------------------------------

/// Signed raw residual values for one audited trim point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FullResidualAudit {
    // Body wrench
    pub fx_body_n: f64,
    pub fy_body_n: f64,
    pub fz_body_n: f64,
    pub mx_body_nm: f64,
    pub my_body_nm: f64,
    pub mz_body_nm: f64,
    // Linear acceleration (world frame)
    pub linear_accel_world_x_mps2: f64,
    pub linear_accel_world_y_mps2: f64,
    pub linear_accel_world_z_mps2: f64,
    // Angular acceleration (body frame)
    pub angular_accel_body_x_rad_s2: f64,
    pub angular_accel_body_y_rad_s2: f64,
    pub angular_accel_body_z_rad_s2: f64,
    // Trim residuals
    pub longitudinal_force_n: f64,
    pub vertical_force_n: f64,
    pub pitch_moment_nm: f64,
}

// ---------------------------------------------------------------------------
// Aerodynamic element domain audit
// ---------------------------------------------------------------------------

/// Per-element domain audit for one qualified trim point.
#[derive(Debug, Clone, PartialEq)]
pub struct AerodynamicElementDomainAudit {
    pub element_index: usize,
    pub element_id: String,
    pub alpha_geom_rad: f64,
    pub alpha_sample_rad: f64,
    pub alpha_lower_rad: f64,
    pub alpha_upper_rad: f64,
    pub alpha_range_status: RangeStatus,
    pub section_airspeed_mps: f64,
    pub polar_binding_kind: &'static str,
    // Reynolds (populated only for ReynoldsFamily bindings)
    pub reynolds_number: Option<f64>,
    pub reynolds_lower: Option<f64>,
    pub reynolds_upper: Option<f64>,
    pub reynolds_range_status: Option<RangeStatus>,
    pub reynolds_lower_node_alpha_lower_rad: Option<f64>,
    pub reynolds_lower_node_alpha_upper_rad: Option<f64>,
    pub reynolds_upper_node_alpha_lower_rad: Option<f64>,
    pub reynolds_upper_node_alpha_upper_rad: Option<f64>,
}

// ---------------------------------------------------------------------------
// Propulsion domain audit
// ---------------------------------------------------------------------------

/// Propulsion operating-point audit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PropulsionDomainAudit {
    NotPresent,
    Present {
        throttle: f64,
        axial_airspeed_mps: f64,
        shaft_speed_rad_s: f64,
        shaft_speed_rpm: f64,
        advance_ratio_j: f64,
        j_lower: f64,
        j_upper: f64,
        j_range_status: RangeStatus,
        shaft_speed_lower_rad_s: f64,
        shaft_speed_upper_rad_s: f64,
        shaft_speed_range_status: RangeStatus,
    },
}

// ---------------------------------------------------------------------------
// Qualification outcome
// ---------------------------------------------------------------------------

/// Per-point qualification result.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimQualificationPoint {
    pub target_airspeed_mps: f64,
    pub outcome: LongitudinalTrimQualificationOutcome,
}

/// Outcome of qualifying one trim point.
#[derive(Debug, Clone, PartialEq)]
pub enum LongitudinalTrimQualificationOutcome {
    Qualified {
        residual_audit: FullResidualAudit,
        aero_audits: Vec<AerodynamicElementDomainAudit>,
        propulsion_audit: PropulsionDomainAudit,
    },
    NotQualified {
        blockers: Vec<QualificationBlocker>,
        residual_audit: FullResidualAudit,
        aero_audits: Vec<AerodynamicElementDomainAudit>,
        propulsion_audit: PropulsionDomainAudit,
    },
}

impl LongitudinalTrimQualificationOutcome {
    #[must_use]
    pub const fn is_qualified(&self) -> bool {
        matches!(self, Self::Qualified { .. })
    }

    #[must_use]
    pub fn blockers(&self) -> &[QualificationBlocker] {
        match self {
            Self::Qualified { .. } => &[],
            Self::NotQualified { blockers, .. } => blockers,
        }
    }
}

// ---------------------------------------------------------------------------
// Qualification collection
// ---------------------------------------------------------------------------

/// Ordered collection of qualification points.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimQualification {
    points: Vec<LongitudinalTrimQualificationPoint>,
}

impl LongitudinalTrimQualification {
    #[must_use]
    pub fn points(&self) -> &[LongitudinalTrimQualificationPoint] {
        &self.points
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.points.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    #[must_use]
    pub fn qualified_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| p.outcome.is_qualified())
            .count()
    }

    #[must_use]
    pub fn not_qualified_count(&self) -> usize {
        self.points
            .iter()
            .filter(|p| !p.outcome.is_qualified())
            .count()
    }
}

// ---------------------------------------------------------------------------
// Alpha range helper
// ---------------------------------------------------------------------------

fn alpha_range_status(alpha: f64, lower: f64, upper: f64) -> RangeStatus {
    if alpha < lower {
        RangeStatus::BelowRange
    } else if alpha > upper {
        RangeStatus::AboveRange
    } else {
        RangeStatus::InRange
    }
}

fn polar_alpha_bounds(table: &PolarTable) -> (f64, f64) {
    let samples = table.samples();
    (samples[0].alpha_rad, samples[samples.len() - 1].alpha_rad)
}

// ---------------------------------------------------------------------------
// Main qualification entry point
// ---------------------------------------------------------------------------

/// Qualifies one successful trim solution against its evidence domains and off-axis limits.
#[must_use]
pub fn qualify_longitudinal_trim_solution(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    solution: &LongitudinalTrimSolution,
    limits: &LongitudinalTrimQualificationLimits,
    target_airspeed_mps: f64,
) -> LongitudinalTrimQualificationPoint {
    let eval = &solution.evaluation;
    let state = &eval.state;
    let variables = &eval.variables;

    // Re-evaluate to verify deterministic reproducibility
    let request_check = crate::trim::LongitudinalTrimRequest::new(
        target_airspeed_mps,
        crate::trim::TrimBounds::new(variables.alpha_rad - 0.5, variables.alpha_rad + 0.5)
            .unwrap_or_else(|_| crate::trim::TrimBounds::new(-1.0, 1.0).unwrap()),
        crate::trim::TrimBounds::new(-1.0, 1.0).unwrap(),
        crate::trim::TrimBounds::new(0.0, 1.0).unwrap(),
        *variables,
        crate::trim::LongitudinalTrimTolerances::new(1.0, 0.1).unwrap(),
        1,
    );

    let re_eval = match &request_check {
        Ok(req) => evaluate_longitudinal_trim_candidate(model, config, req, *variables),
        Err(_) => None,
    };

    let mut blockers = Vec::new();
    let mut all_finite = true;

    // Build effective aero elements
    let effective = model
        .aero_elements()
        .iter()
        .map(|re| *re.element())
        .collect::<Vec<_>>();

    // Compute per-surface alpha_i values
    let surfaces = model.aero_surfaces();
    let mut surface_alpha_i = vec![0.0f64; surfaces.len()];
    let mut element_surface_map = vec![None; model.aero_elements().len()];
    for (surf_idx, surface) in surfaces.iter().enumerate() {
        let (alpha_i, _, _) = solve_surface_induced_alpha(
            surface,
            state,
            &effective,
            model,
            config.aero_environment(),
        );
        surface_alpha_i[surf_idx] = alpha_i;
        for &elem_idx in surface.element_indices() {
            element_surface_map[elem_idx] = Some(surf_idx);
        }
    }

    // Audit each aero element
    let mut aero_audits = Vec::with_capacity(model.aero_elements().len());
    for (elem_idx, runtime_elem) in model.aero_elements().iter().enumerate() {
        let eff_elem = &effective[elem_idx];
        let kin = compute_section_kinematics(state, eff_elem, config.aero_environment());

        let alpha_geom = kin.alpha_rad;
        let alpha_i = element_surface_map[elem_idx]
            .map(|si| surface_alpha_i[si])
            .unwrap_or(0.0);
        let alpha_sample = alpha_geom - alpha_i;

        let (audit, elem_blockers) = audit_aero_element(
            elem_idx,
            runtime_elem,
            model,
            alpha_geom,
            alpha_sample,
            kin.section_airspeed_mps,
        );
        aero_audits.push(audit);
        blockers.extend(elem_blockers);
    }

    // Propulsion audit
    let propulsion_audit = audit_propulsion(model, config, state, variables, &mut blockers);

    // Full residual audit
    let body_wrench = &eval.body_wrench;
    let derivative = &eval.derivative;
    let residual_audit = FullResidualAudit {
        fx_body_n: body_wrench.force_body_n.x,
        fy_body_n: body_wrench.force_body_n.y,
        fz_body_n: body_wrench.force_body_n.z,
        mx_body_nm: body_wrench.moment_body_nm.x,
        my_body_nm: body_wrench.moment_body_nm.y,
        mz_body_nm: body_wrench.moment_body_nm.z,
        linear_accel_world_x_mps2: derivative.linear_velocity_world_mps2.x,
        linear_accel_world_y_mps2: derivative.linear_velocity_world_mps2.y,
        linear_accel_world_z_mps2: derivative.linear_velocity_world_mps2.z,
        angular_accel_body_x_rad_s2: derivative.angular_velocity_body_radps2.x,
        angular_accel_body_y_rad_s2: derivative.angular_velocity_body_radps2.y,
        angular_accel_body_z_rad_s2: derivative.angular_velocity_body_radps2.z,
        longitudinal_force_n: eval.residuals.longitudinal_force_n,
        vertical_force_n: eval.residuals.vertical_force_n,
        pitch_moment_nm: eval.residuals.pitch_moment_nm,
    };

    // Check finiteness
    let audit_values = [
        ("fx_body_n", residual_audit.fx_body_n),
        ("fy_body_n", residual_audit.fy_body_n),
        ("fz_body_n", residual_audit.fz_body_n),
        ("mx_body_nm", residual_audit.mx_body_nm),
        ("my_body_nm", residual_audit.my_body_nm),
        ("mz_body_nm", residual_audit.mz_body_nm),
        (
            "linear_accel_world_x",
            residual_audit.linear_accel_world_x_mps2,
        ),
        (
            "linear_accel_world_y",
            residual_audit.linear_accel_world_y_mps2,
        ),
        (
            "linear_accel_world_z",
            residual_audit.linear_accel_world_z_mps2,
        ),
        (
            "angular_accel_body_x",
            residual_audit.angular_accel_body_x_rad_s2,
        ),
        (
            "angular_accel_body_y",
            residual_audit.angular_accel_body_y_rad_s2,
        ),
        (
            "angular_accel_body_z",
            residual_audit.angular_accel_body_z_rad_s2,
        ),
    ];
    for &(field, value) in &audit_values {
        if !value.is_finite() {
            all_finite = false;
            blockers.push(QualificationBlocker::NonFiniteAuditValue { field });
        }
    }

    // Check off-axis limits
    if residual_audit.fy_body_n.abs() > limits.max_side_force_n {
        blockers.push(QualificationBlocker::SideForceLimitExceeded {
            fy_body_n: residual_audit.fy_body_n,
            limit_n: limits.max_side_force_n,
        });
    }
    if residual_audit.mx_body_nm.abs() > limits.max_roll_moment_nm {
        blockers.push(QualificationBlocker::RollMomentLimitExceeded {
            mx_body_nm: residual_audit.mx_body_nm,
            limit_nm: limits.max_roll_moment_nm,
        });
    }
    if residual_audit.mz_body_nm.abs() > limits.max_yaw_moment_nm {
        blockers.push(QualificationBlocker::YawMomentLimitExceeded {
            mz_body_nm: residual_audit.mz_body_nm,
            limit_nm: limits.max_yaw_moment_nm,
        });
    }
    if residual_audit.linear_accel_world_y_mps2.abs() > limits.max_lateral_acceleration_mps2 {
        blockers.push(QualificationBlocker::LateralAccelerationLimitExceeded {
            ay_world_mps2: residual_audit.linear_accel_world_y_mps2,
            limit_mps2: limits.max_lateral_acceleration_mps2,
        });
    }
    if residual_audit.angular_accel_body_x_rad_s2.abs()
        > limits.max_roll_angular_acceleration_rad_s2
    {
        blockers.push(QualificationBlocker::RollAngularAccelerationLimitExceeded {
            angular_accel_body_x_rad_s2: residual_audit.angular_accel_body_x_rad_s2,
            limit_rad_s2: limits.max_roll_angular_acceleration_rad_s2,
        });
    }
    if residual_audit.angular_accel_body_z_rad_s2.abs() > limits.max_yaw_angular_acceleration_rad_s2
    {
        blockers.push(QualificationBlocker::YawAngularAccelerationLimitExceeded {
            angular_accel_body_z_rad_s2: residual_audit.angular_accel_body_z_rad_s2,
            limit_rad_s2: limits.max_yaw_angular_acceleration_rad_s2,
        });
    }

    // Re-evaluation check
    if re_eval.is_none() {
        blockers.push(QualificationBlocker::ReEvaluationFailure);
    }

    let outcome = if blockers.is_empty() && all_finite {
        LongitudinalTrimQualificationOutcome::Qualified {
            residual_audit,
            aero_audits,
            propulsion_audit,
        }
    } else {
        LongitudinalTrimQualificationOutcome::NotQualified {
            blockers,
            residual_audit,
            aero_audits,
            propulsion_audit,
        }
    };

    LongitudinalTrimQualificationPoint {
        target_airspeed_mps,
        outcome,
    }
}

// ---------------------------------------------------------------------------
// Aero element audit
// ---------------------------------------------------------------------------

fn audit_aero_element(
    elem_idx: usize,
    runtime_elem: &model::RuntimeAeroElement,
    model: &AircraftModel,
    alpha_geom: f64,
    alpha_sample: f64,
    section_airspeed: f64,
) -> (AerodynamicElementDomainAudit, Vec<QualificationBlocker>) {
    let mut blockers = Vec::new();
    let eff_elem = runtime_elem.element();

    match runtime_elem.polar_binding() {
        model::RuntimeAeroPolarBinding::Polar { polar_index } => {
            let table = &model.aero_polars()[polar_index].table();
            let (alpha_lo, alpha_hi) = polar_alpha_bounds(table);
            let status = alpha_range_status(alpha_sample, alpha_lo, alpha_hi);

            if status == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::AerodynamicAlphaBelowRange {
                    element_index: elem_idx,
                    element_id: runtime_elem.id().to_owned(),
                    alpha_sample_rad: alpha_sample,
                    alpha_lower_rad: alpha_lo,
                });
            } else if status == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::AerodynamicAlphaAboveRange {
                    element_index: elem_idx,
                    element_id: runtime_elem.id().to_owned(),
                    alpha_sample_rad: alpha_sample,
                    alpha_upper_rad: alpha_hi,
                });
            }

            let audit = AerodynamicElementDomainAudit {
                element_index: elem_idx,
                element_id: runtime_elem.id().to_owned(),
                alpha_geom_rad: alpha_geom,
                alpha_sample_rad: alpha_sample,
                alpha_lower_rad: alpha_lo,
                alpha_upper_rad: alpha_hi,
                alpha_range_status: status,
                section_airspeed_mps: section_airspeed,
                polar_binding_kind: "polar",
                reynolds_number: None,
                reynolds_lower: None,
                reynolds_upper: None,
                reynolds_range_status: None,
                reynolds_lower_node_alpha_lower_rad: None,
                reynolds_lower_node_alpha_upper_rad: None,
                reynolds_upper_node_alpha_lower_rad: None,
                reynolds_upper_node_alpha_upper_rad: None,
            };
            (audit, blockers)
        }
        model::RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
            let family = &model.aero_polar_families()[family_index].family();
            let viscosity = model.kinematic_viscosity_m2_s().unwrap();
            let chord = eff_elem.chord_m();
            let re = section_airspeed * chord / viscosity;

            let nodes = family.nodes();
            let re_lo = nodes[0].reynolds_number();
            let re_hi = nodes[nodes.len() - 1].reynolds_number();
            let re_status = if re < re_lo {
                RangeStatus::BelowRange
            } else if re > re_hi {
                RangeStatus::AboveRange
            } else {
                RangeStatus::InRange
            };

            if re_status == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::ReynoldsBelowRange {
                    element_index: elem_idx,
                    element_id: runtime_elem.id().to_owned(),
                    reynolds_number: re,
                    reynolds_lower: re_lo,
                });
            } else if re_status == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::ReynoldsAboveRange {
                    element_index: elem_idx,
                    element_id: runtime_elem.id().to_owned(),
                    reynolds_number: re,
                    reynolds_upper: re_hi,
                });
            }

            // Determine contributing nodes and audit their alpha domains
            let (lower_node, upper_node, _fraction) = find_reynolds_bracket(family, re);

            let mut ln_node_alpha_lo = None;
            let mut ln_node_alpha_hi = None;
            let mut un_node_alpha_lo = None;
            let mut un_node_alpha_hi = None;

            if let Some(ln) = lower_node {
                let (lo, hi) = polar_alpha_bounds(ln.table());
                ln_node_alpha_lo = Some(lo);
                ln_node_alpha_hi = Some(hi);
                // Check alpha in lower node
                if alpha_sample < lo {
                    blockers.push(
                        QualificationBlocker::ReynoldsContributingNodeAlphaBelowRange {
                            element_index: elem_idx,
                            element_id: runtime_elem.id().to_owned(),
                            node_reynolds: ln.reynolds_number(),
                            alpha_sample_rad: alpha_sample,
                            alpha_lower_rad: lo,
                        },
                    );
                } else if alpha_sample > hi {
                    blockers.push(
                        QualificationBlocker::ReynoldsContributingNodeAlphaAboveRange {
                            element_index: elem_idx,
                            element_id: runtime_elem.id().to_owned(),
                            node_reynolds: ln.reynolds_number(),
                            alpha_sample_rad: alpha_sample,
                            alpha_upper_rad: hi,
                        },
                    );
                }
            }
            if let Some(un) = upper_node {
                let (lo, hi) = polar_alpha_bounds(un.table());
                un_node_alpha_lo = Some(lo);
                un_node_alpha_hi = Some(hi);
                // Only check upper node if it's different from lower
                if upper_node.map(|n| n.reynolds_number())
                    != lower_node.map(|n| n.reynolds_number())
                {
                    if alpha_sample < lo {
                        blockers.push(
                            QualificationBlocker::ReynoldsContributingNodeAlphaBelowRange {
                                element_index: elem_idx,
                                element_id: runtime_elem.id().to_owned(),
                                node_reynolds: un.reynolds_number(),
                                alpha_sample_rad: alpha_sample,
                                alpha_lower_rad: lo,
                            },
                        );
                    } else if alpha_sample > hi {
                        blockers.push(
                            QualificationBlocker::ReynoldsContributingNodeAlphaAboveRange {
                                element_index: elem_idx,
                                element_id: runtime_elem.id().to_owned(),
                                node_reynolds: un.reynolds_number(),
                                alpha_sample_rad: alpha_sample,
                                alpha_upper_rad: hi,
                            },
                        );
                    }
                }
            }

            // For alpha range status, use the intersection of contributing node domains
            let alpha_lo = ln_node_alpha_lo
                .unwrap_or(0.0)
                .max(un_node_alpha_lo.unwrap_or(0.0));
            let alpha_hi = ln_node_alpha_hi
                .unwrap_or(0.0)
                .min(un_node_alpha_hi.unwrap_or(0.0));
            let alpha_status = if let (Some(ln), Some(un)) = (lower_node, upper_node) {
                if ln.reynolds_number() == un.reynolds_number() {
                    // Exact node match: use that single node's domain
                    let (lo, hi) = polar_alpha_bounds(ln.table());
                    alpha_range_status(alpha_sample, lo, hi)
                } else {
                    alpha_range_status(alpha_sample, alpha_lo, alpha_hi)
                }
            } else {
                alpha_range_status(alpha_sample, alpha_lo, alpha_hi)
            };

            let audit = AerodynamicElementDomainAudit {
                element_index: elem_idx,
                element_id: runtime_elem.id().to_owned(),
                alpha_geom_rad: alpha_geom,
                alpha_sample_rad: alpha_sample,
                alpha_lower_rad: alpha_lo,
                alpha_upper_rad: alpha_hi,
                alpha_range_status: alpha_status,
                section_airspeed_mps: section_airspeed,
                polar_binding_kind: "reynolds_family",
                reynolds_number: Some(re),
                reynolds_lower: Some(re_lo),
                reynolds_upper: Some(re_hi),
                reynolds_range_status: Some(re_status),
                reynolds_lower_node_alpha_lower_rad: ln_node_alpha_lo,
                reynolds_lower_node_alpha_upper_rad: ln_node_alpha_hi,
                reynolds_upper_node_alpha_lower_rad: un_node_alpha_lo,
                reynolds_upper_node_alpha_upper_rad: un_node_alpha_hi,
            };
            (audit, blockers)
        }
    }
}

/// Find the bracketing Reynolds nodes for a given Reynolds number.
/// Returns (lower_node, upper_node, interpolation_fraction).
/// For exact match or out-of-range, both point to the same node.
fn find_reynolds_bracket(
    family: &sim_core::ReynoldsPolarFamily,
    re: f64,
) -> (
    Option<&sim_core::ReynoldsPolar>,
    Option<&sim_core::ReynoldsPolar>,
    f64,
) {
    let nodes = family.nodes();
    if nodes.len() == 1 {
        return (Some(&nodes[0]), Some(&nodes[0]), 0.0);
    }
    match nodes.binary_search_by(|n| n.reynolds_number().total_cmp(&re)) {
        Ok(idx) => (Some(&nodes[idx]), Some(&nodes[idx]), 0.0),
        Err(0) => (Some(&nodes[0]), Some(&nodes[0]), 0.0),
        Err(upper) if upper == nodes.len() => {
            let last = nodes.len() - 1;
            (Some(&nodes[last]), Some(&nodes[last]), 0.0)
        }
        Err(upper) => {
            let lower = upper - 1;
            let lo = nodes[lower].reynolds_number();
            let hi = nodes[upper].reynolds_number();
            let frac = if (hi - lo).abs() < 1e-30 {
                0.0
            } else {
                (re - lo) / (hi - lo)
            };
            (Some(&nodes[lower]), Some(&nodes[upper]), frac)
        }
    }
}

// ---------------------------------------------------------------------------
// Propulsion audit
// ---------------------------------------------------------------------------

fn audit_propulsion(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    state: &RigidBodyState,
    variables: &LongitudinalTrimVariables,
    blockers: &mut Vec<QualificationBlocker>,
) -> PropulsionDomainAudit {
    let Some(runtime_prop) = model.propulsion() else {
        return PropulsionDomainAudit::NotPresent;
    };

    let prop_output = evaluate_electric_propulsion_with_source(
        state,
        variables.throttle,
        runtime_prop.config(),
        config.aero_environment(),
        runtime_prop.coefficient_source(),
    );

    let j = prop_output.advance_ratio_j;
    let shaft_speed = prop_output.shaft_speed_rad_s;

    let (j_lo, j_hi, j_status, ss_lo, ss_hi, ss_status) = match runtime_prop.coefficient_source() {
        PropellerCoefficientSource::FixedTable(table) => {
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let j_stat = alpha_range_status(j, j_lo, j_hi);
            if j_stat == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                    advance_ratio_j: j,
                    j_lower: j_lo,
                });
            } else if j_stat == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                    advance_ratio_j: j,
                    j_upper: j_hi,
                });
            }
            // Fixed table: shaft speed is always "in range" (no map)
            (j_lo, j_hi, j_stat, 0.0, 0.0, RangeStatus::InRange)
        }
        PropellerCoefficientSource::ShaftSpeedMap(map) => {
            let nodes = map.nodes();
            let ss_lo = nodes[0].shaft_speed_rad_s();
            let ss_hi = nodes[nodes.len() - 1].shaft_speed_rad_s();
            let ss_stat = match ShaftSpeedRangeStatus::from_re(shaft_speed, ss_lo, ss_hi) {
                ShaftSpeedRangeStatus::BelowRange => RangeStatus::BelowRange,
                ShaftSpeedRangeStatus::AboveRange => RangeStatus::AboveRange,
                ShaftSpeedRangeStatus::ExactOrInRange => RangeStatus::InRange,
            };
            if ss_stat == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerShaftSpeedBelowRange {
                    shaft_speed_rad_s: shaft_speed,
                    shaft_speed_lower_rad_s: ss_lo,
                });
            } else if ss_stat == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::PropellerShaftSpeedAboveRange {
                    shaft_speed_rad_s: shaft_speed,
                    shaft_speed_upper_rad_s: ss_hi,
                });
            }

            // J domain: check against contributing node tables
            let (j_lo, j_hi, j_stat) = audit_map_j_domain(map, shaft_speed, j, blockers);
            (j_lo, j_hi, j_stat, ss_lo, ss_hi, ss_stat)
        }
    };

    PropulsionDomainAudit::Present {
        throttle: variables.throttle,
        axial_airspeed_mps: prop_output.axial_airspeed_mps,
        shaft_speed_rad_s: shaft_speed,
        shaft_speed_rpm: prop_output.shaft_speed_rpm,
        advance_ratio_j: j,
        j_lower: j_lo,
        j_upper: j_hi,
        j_range_status: j_status,
        shaft_speed_lower_rad_s: ss_lo,
        shaft_speed_upper_rad_s: ss_hi,
        shaft_speed_range_status: ss_status,
    }
}

/// Audit J domain for a shaft-speed map. Returns (j_lo, j_hi, j_status).
fn audit_map_j_domain(
    map: &PropellerCoefficientMap,
    shaft_speed: f64,
    j: f64,
    blockers: &mut Vec<QualificationBlocker>,
) -> (f64, f64, RangeStatus) {
    let nodes = map.nodes();
    if nodes.len() == 1 {
        let table = nodes[0].table();
        let samples = table.samples();
        let j_lo = samples[0].advance_ratio_j;
        let j_hi = samples[samples.len() - 1].advance_ratio_j;
        let status = alpha_range_status(j, j_lo, j_hi);
        if status == RangeStatus::BelowRange {
            blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                advance_ratio_j: j,
                j_lower: j_lo,
            });
        } else if status == RangeStatus::AboveRange {
            blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                advance_ratio_j: j,
                j_upper: j_hi,
            });
        }
        return (j_lo, j_hi, status);
    }

    // Find bracket
    match nodes.binary_search_by(|n| n.shaft_speed_rad_s().total_cmp(&shaft_speed)) {
        Ok(idx) => {
            // Exact match: check only that node
            let table = nodes[idx].table();
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let status = alpha_range_status(j, j_lo, j_hi);
            if status == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                    advance_ratio_j: j,
                    j_lower: j_lo,
                });
            } else if status == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                    advance_ratio_j: j,
                    j_upper: j_hi,
                });
            }
            (j_lo, j_hi, status)
        }
        Err(0) => {
            let table = nodes[0].table();
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let status = alpha_range_status(j, j_lo, j_hi);
            if status == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                    advance_ratio_j: j,
                    j_lower: j_lo,
                });
            } else if status == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                    advance_ratio_j: j,
                    j_upper: j_hi,
                });
            }
            (j_lo, j_hi, status)
        }
        Err(upper) if upper == nodes.len() => {
            let last = nodes.len() - 1;
            let table = nodes[last].table();
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let status = alpha_range_status(j, j_lo, j_hi);
            if status == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                    advance_ratio_j: j,
                    j_lower: j_lo,
                });
            } else if status == RangeStatus::AboveRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                    advance_ratio_j: j,
                    j_upper: j_hi,
                });
            }
            (j_lo, j_hi, status)
        }
        Err(upper) => {
            let lower = upper - 1;
            let lower_table = nodes[lower].table();
            let upper_table = nodes[upper].table();
            let l_samples = lower_table.samples();
            let u_samples = upper_table.samples();
            let j_lo_lower = l_samples[0].advance_ratio_j;
            let j_hi_lower = l_samples[l_samples.len() - 1].advance_ratio_j;
            let j_lo_upper = u_samples[0].advance_ratio_j;
            let j_hi_upper = u_samples[u_samples.len() - 1].advance_ratio_j;

            // J must be inside BOTH contributing tables
            let j_lo = j_lo_lower.max(j_lo_upper);
            let j_hi = j_hi_lower.min(j_hi_upper);
            let status = alpha_range_status(j, j_lo, j_hi);

            // Also check each individual table
            let j_in_lower = alpha_range_status(j, j_lo_lower, j_hi_lower);
            let j_in_upper = alpha_range_status(j, j_lo_upper, j_hi_upper);

            if j_in_lower == RangeStatus::BelowRange || j_in_upper == RangeStatus::BelowRange {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioBelowRange {
                    advance_ratio_j: j,
                    j_lower: j_lo,
                });
            } else if j_in_lower == RangeStatus::AboveRange || j_in_upper == RangeStatus::AboveRange
            {
                blockers.push(QualificationBlocker::PropellerAdvanceRatioAboveRange {
                    advance_ratio_j: j,
                    j_upper: j_hi,
                });
            }

            (j_lo, j_hi, status)
        }
    }
}

// Helper to convert ShaftSpeedRangeStatus
trait ShaftSpeedStatusExt {
    fn from_re(shaft_speed: f64, lo: f64, hi: f64) -> ShaftSpeedRangeStatus;
}
impl ShaftSpeedStatusExt for ShaftSpeedRangeStatus {
    fn from_re(shaft_speed: f64, lo: f64, hi: f64) -> Self {
        if shaft_speed < lo {
            Self::BelowRange
        } else if shaft_speed > hi {
            Self::AboveRange
        } else {
            Self::ExactOrInRange
        }
    }
}
