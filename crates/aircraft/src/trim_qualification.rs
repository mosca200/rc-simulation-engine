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
//!
//! # Accepted trim operating point
//!
//! Qualification audits the EXACT accepted trim operating point:
//! - Aero elements are built through `effective_aero_elements_for_positions` using the
//!   solution's `control_surface_positions` (deflected geometry, not base geometry).
//! - Propulsion is evaluated with `control_surface_positions.throttle()` (the accepted
//!   control output throttle, not the raw trim variable).
//! - Re-evaluation must match the solver-cached evaluation exactly (M2.6A precedent).

use crate::{
    AircraftSimulationConfig,
    simulation::solve_surface_induced_alpha,
    trim::{LongitudinalTrimSolution, evaluate_longitudinal_trim_candidate},
};
use model::AircraftModel;
use sim_core::{
    PolarTable, PropellerCoefficientMap, PropellerCoefficientSource, RigidBodyState,
    compute_section_kinematics, evaluate_electric_propulsion_with_source,
};
use thiserror::Error;

// ---------------------------------------------------------------------------
// Range status
// ---------------------------------------------------------------------------

/// Whether a value lies inside, below, or above its evidence support.
///
/// The five domain classifications are exhaustive for finite values:
/// - [`RangeStatus::BelowRange`] — strictly below the supported interval (NOT supported;
///   the runtime reaches this value only by clamping).
/// - [`RangeStatus::AtLowerBound`] — bitwise-equal to the supported lower endpoint (supported).
/// - [`RangeStatus::InRange`] — strictly inside the supported interval (supported).
/// - [`RangeStatus::AtUpperBound`] — bitwise-equal to the supported upper endpoint (supported).
/// - [`RangeStatus::AboveRange`] — strictly above the supported interval (NOT supported;
///   the runtime reaches this value only by clamping).
///
/// `NonFinite` is a fail-closed sentinel: NaN / ±Infinity inputs cannot be classified
/// as `InRange`. Qualification remains `NotQualified` through the `NonFiniteAuditValue`
/// integrity blocker when any audited value is non-finite.
///
/// Boundaries use exact bitwise `f64` equality with the authored endpoint; no epsilon is
/// introduced (an epsilon would invent support that the authored data does not declare).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangeStatus {
    NonFinite,
    BelowRange,
    AtLowerBound,
    InRange,
    AtUpperBound,
    AboveRange,
}

/// Classifies `value` against the closed authored support interval `[lower, upper]`.
///
/// Boundary membership uses exact bitwise equality: `value == lower` yields
/// [`RangeStatus::AtLowerBound`] and `value == upper` yields [`RangeStatus::AtUpperBound`].
/// This function is pure, allocation-free, and deterministic; it is the single classifier
/// used for polar alpha domains, Reynolds family domains, and propeller J domains.
#[must_use]
pub fn classify_range_status(value: f64, lower: f64, upper: f64) -> RangeStatus {
    if !value.is_finite() {
        return RangeStatus::NonFinite;
    }
    if value < lower {
        RangeStatus::BelowRange
    } else if value > upper {
        RangeStatus::AboveRange
    } else if value == lower {
        RangeStatus::AtLowerBound
    } else if value == upper {
        RangeStatus::AtUpperBound
    } else {
        RangeStatus::InRange
    }
}

// ---------------------------------------------------------------------------
// Qualification blockers
// ---------------------------------------------------------------------------

/// Typed reason a trim point is not qualified.
///
/// Blocker ordering follows the documented contract:
/// 1. Aero elements in model order (alpha first, Reynolds second per element)
/// 2. Propulsion (shaft-speed first, J second)
/// 3. Residual limits (Fy, Mx, Mz, lateral accel, roll accel, yaw accel)
/// 4. Integrity / non-finite / re-evaluation
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

    // Reynolds contributing node alpha (emitted before element Reynolds range)
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

    // Reynolds range
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

    // Propulsion shaft speed
    PropellerShaftSpeedBelowRange {
        shaft_speed_rad_s: f64,
        shaft_speed_lower_rad_s: f64,
    },
    PropellerShaftSpeedAboveRange {
        shaft_speed_rad_s: f64,
        shaft_speed_upper_rad_s: f64,
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

    // Integrity (always last)
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

/// Shaft-speed domain sub-audit. Only present when the coefficient source has a
/// shaft-speed map. Fixed propeller tables have no shaft-speed domain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ShaftSpeedDomainAudit {
    pub shaft_speed_lower_rad_s: f64,
    pub shaft_speed_upper_rad_s: f64,
    pub shaft_speed_range_status: RangeStatus,
}

/// RPM support classification.
///
/// RPM is ONLY classified against a support range when the authored model/runtime data
/// explicitly declares one. No RPM envelope is ever invented here: without an explicit
/// authored RPM support range the audit reports [`RpmDomainStatus::NotDeclared`] while
/// still recording the runtime RPM value itself.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RpmDomainStatus {
    /// The authored model/runtime data declares no explicit RPM support range. The
    /// runtime RPM value is recorded unclassified; no envelope is invented.
    NotDeclared,
    /// An explicit authored RPM support range exists and was classified against.
    Declared {
        lower_rpm: f64,
        upper_rpm: f64,
        status: RangeStatus,
    },
}

/// Propulsion operating-point audit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PropulsionDomainAudit {
    /// No propulsion system configured on this aircraft.
    NotPresent,
    /// Propulsion system present and audited.
    Present {
        /// Accepted control-output throttle (from `control_surface_positions.throttle()`).
        throttle: f64,
        axial_airspeed_mps: f64,
        shaft_speed_rad_s: f64,
        shaft_speed_rpm: f64,
        advance_ratio_j: f64,
        j_lower: f64,
        j_upper: f64,
        j_range_status: RangeStatus,
        /// Shaft-speed domain audit. `None` for fixed propeller tables (no shaft-speed
        /// map domain exists). `Some` for shaft-speed maps.
        shaft_speed_domain: Option<ShaftSpeedDomainAudit>,
        /// RPM support classification. `NotDeclared` unless the authored data explicitly
        /// declares an RPM support range.
        rpm_domain: RpmDomainStatus,
        /// Thrust produced at the audited operating point [N].
        thrust_n: f64,
        /// Shaft torque produced at the audited operating point [N·m].
        torque_n_m: f64,
    },
}

// ---------------------------------------------------------------------------
// Qualification outcome
// ---------------------------------------------------------------------------

/// Categorization of a [`QualificationBlocker`], used to select the typed outcome variant.
///
/// The mapping is total and deterministic; no string codes are involved.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QualificationBlockerCategory {
    /// Authored-domain violation (aero alpha, Reynolds family, propulsion J/shaft speed).
    Domain,
    /// Off-axis residual-limit violation (caller-supplied limits).
    Residual,
    /// Integrity failure (non-finite audit value, failed deterministic re-evaluation).
    Integrity,
}

impl QualificationBlocker {
    /// Returns the deterministic category of this blocker.
    #[must_use]
    pub const fn category(&self) -> QualificationBlockerCategory {
        match self {
            Self::AerodynamicAlphaBelowRange { .. }
            | Self::AerodynamicAlphaAboveRange { .. }
            | Self::ReynoldsContributingNodeAlphaBelowRange { .. }
            | Self::ReynoldsContributingNodeAlphaAboveRange { .. }
            | Self::ReynoldsBelowRange { .. }
            | Self::ReynoldsAboveRange { .. }
            | Self::PropellerShaftSpeedBelowRange { .. }
            | Self::PropellerShaftSpeedAboveRange { .. }
            | Self::PropellerAdvanceRatioBelowRange { .. }
            | Self::PropellerAdvanceRatioAboveRange { .. } => QualificationBlockerCategory::Domain,
            Self::SideForceLimitExceeded { .. }
            | Self::RollMomentLimitExceeded { .. }
            | Self::YawMomentLimitExceeded { .. }
            | Self::LateralAccelerationLimitExceeded { .. }
            | Self::RollAngularAccelerationLimitExceeded { .. }
            | Self::YawAngularAccelerationLimitExceeded { .. } => {
                QualificationBlockerCategory::Residual
            }
            Self::NonFiniteAuditValue { .. } | Self::ReEvaluationFailure => {
                QualificationBlockerCategory::Integrity
            }
        }
    }
}

/// Complete diagnostics for one audited trim point (a point whose trim evaluation exists
/// and was therefore fully audited). Nothing is zeroed, filtered, or truncated.
#[derive(Debug, Clone, PartialEq)]
pub struct QualificationDiagnostics {
    blockers: Vec<QualificationBlocker>,
    residual_audit: FullResidualAudit,
    aero_audits: Vec<AerodynamicElementDomainAudit>,
    propulsion_audit: PropulsionDomainAudit,
}

impl QualificationDiagnostics {
    /// All blockers in the documented deterministic order (aero elements in model order,
    /// then propulsion, then residual limits, then integrity). ALL blockers are preserved.
    #[must_use]
    pub fn blockers(&self) -> &[QualificationBlocker] {
        &self.blockers
    }

    /// Full signed residual audit.
    #[must_use]
    pub const fn residual_audit(&self) -> &FullResidualAudit {
        &self.residual_audit
    }

    /// Per-element aerodynamic domain audits in model element order.
    #[must_use]
    pub fn aero_audits(&self) -> &[AerodynamicElementDomainAudit] {
        &self.aero_audits
    }

    /// Propulsion domain audit.
    #[must_use]
    pub const fn propulsion_audit(&self) -> &PropulsionDomainAudit {
        &self.propulsion_audit
    }
}

/// Per-point qualification result.
#[derive(Debug, Clone, PartialEq)]
pub struct LongitudinalTrimQualificationPoint {
    pub target_airspeed_mps: f64,
    pub outcome: LongitudinalTrimQualificationOutcome,
}

/// Why qualification could not present trustworthy diagnostics for a point that DID
/// produce a trim evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QualificationUnavailableReason {
    /// An audited value was NaN or ±Infinity; the offending field is named here and by
    /// the matching `NonFiniteAuditValue` blocker in the preserved diagnostics.
    NonFiniteAuditValue { field: &'static str },
    /// The accepted trim evaluation could not be re-evaluated identically through the
    /// authoritative runtime path, so its cached diagnostics cannot be trusted.
    ReEvaluationFailure,
    /// The M2.6A sweep flagged the point as a re-evaluation MISMATCH; the point is not
    /// audited because its solver-cached evaluation is already known to be untrustworthy.
    SweepReEvaluationMismatch,
    /// The M2.6A sweep could not produce an independent re-evaluation for the point, so
    /// there is no trustworthy evaluation to audit.
    SweepReEvaluationUnverifiable,
}

/// Outcome of qualifying one trim point.
///
/// Variant-selection precedence (documented, deterministic):
/// 1. [`LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure`] — the point never
///    produced a trim solution; nothing is audited and NO diagnostics are fabricated.
/// 2. [`LongitudinalTrimQualificationOutcome::QualificationUnavailable`] — for sweep points
///    whose evaluation integrity was already broken (re-evaluation mismatch/unverifiable),
///    NO diagnostics are fabricated.
/// 3. [`LongitudinalTrimQualificationOutcome::NotQualifiedDomainViolation`] — at least one
///    authored-domain blocker exists. ALL blockers (including residual/integrity ones) are
///    preserved in the diagnostics; nothing is dropped at the first violation.
/// 4. [`LongitudinalTrimQualificationOutcome::NotQualifiedResidualViolation`] — no domain
///    blockers, but at least one off-axis residual-limit blocker exists.
/// 5. [`LongitudinalTrimQualificationOutcome::Qualified`] — trim succeeded, every applicable
///    authored domain is supported, and every off-axis residual limit passes.
#[derive(Debug, Clone, PartialEq)]
pub enum LongitudinalTrimQualificationOutcome {
    /// Trim succeeded and every applicable authored domain and off-axis limit is supported.
    Qualified(QualificationDiagnostics),
    /// The trim solver failed for this point. No aerodynamic or propulsion diagnostics
    /// exist and none are fabricated.
    NotQualifiedTrimFailure {
        failure: crate::trim::LongitudinalTrimFailure,
    },
    /// Trim succeeded, but the operating point relies on evidence outside the authored
    /// domains (extrapolation-by-clamping). ALL blockers are preserved.
    NotQualifiedDomainViolation(QualificationDiagnostics),
    /// Trim succeeded, all authored domains are supported, but an off-axis residual
    /// exceeds a caller-supplied limit. ALL blockers are preserved.
    NotQualifiedResidualViolation(QualificationDiagnostics),
    /// The point produced a trim evaluation but its diagnostics cannot be trusted or
    /// presented (integrity failure). For integrity-failed audits the partially valid
    /// diagnostics are preserved (`Some`); for already-untrusted sweep points nothing
    /// is audited (`None`).
    QualificationUnavailable {
        reason: QualificationUnavailableReason,
        diagnostics: Option<QualificationDiagnostics>,
    },
}

impl LongitudinalTrimQualificationOutcome {
    #[must_use]
    pub const fn is_qualified(&self) -> bool {
        matches!(self, Self::Qualified(_))
    }

    /// All preserved blockers for this outcome. Empty for `Qualified` and
    /// `NotQualifiedTrimFailure`; for `QualificationUnavailable` the preserved
    /// diagnostics' blockers are returned when present.
    #[must_use]
    pub fn blockers(&self) -> &[QualificationBlocker] {
        match self {
            Self::Qualified(diagnostics) => diagnostics.blockers(),
            Self::NotQualifiedTrimFailure { .. } => &[],
            Self::NotQualifiedDomainViolation(diagnostics)
            | Self::NotQualifiedResidualViolation(diagnostics) => diagnostics.blockers(),
            Self::QualificationUnavailable { diagnostics, .. } => match diagnostics {
                Some(preserved) => preserved.blockers(),
                None => &[],
            },
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
    /// Private constructor used only by the qualification entry points.
    const fn from_points(points: Vec<LongitudinalTrimQualificationPoint>) -> Self {
        Self { points }
    }

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
// (Range-status classification lives in the public `classify_range_status`.)
// ---------------------------------------------------------------------------

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
    let positions = &eval.control_surface_positions;

    // FIX 3: Re-evaluate to verify deterministic reproducibility (M2.6A precedent).
    let variables = &eval.variables;
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

    let re_eval_matches = match &request_check {
        Ok(req) => match evaluate_longitudinal_trim_candidate(model, config, req, *variables) {
            Some(independent) => independent == *eval,
            None => false,
        },
        Err(_) => false,
    };

    // FIX 1: Build effective aero elements from the ACCEPTED control surface positions.
    // This audits the deflected geometry actually evaluated by the trim solver,
    // not the base geometry.
    let effective = crate::effective_aero_elements_for_positions(model, positions);

    // Compute per-surface alpha_i values using the deflected elements
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

    // -----------------------------------------------------------------------
    // Phase 1: Aero element blockers (in model order)
    // -----------------------------------------------------------------------
    let mut aero_blockers = Vec::new();
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
            eff_elem,
            model,
            alpha_geom,
            alpha_sample,
            kin.section_airspeed_mps,
        );
        aero_audits.push(audit);
        aero_blockers.extend(elem_blockers);
    }

    // -----------------------------------------------------------------------
    // Phase 2: Propulsion blockers (shaft-speed first, then J)
    // -----------------------------------------------------------------------
    // FIX 2: Use the accepted control-output throttle, not the raw trim variable.
    let accepted_throttle = positions.throttle();
    let mut propulsion_blockers = Vec::new();
    let propulsion_audit = audit_propulsion(
        model,
        config,
        state,
        accepted_throttle,
        &mut propulsion_blockers,
    );

    // -----------------------------------------------------------------------
    // Phase 3: Full residual audit and off-axis limit blockers
    // -----------------------------------------------------------------------
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

    let mut residual_limit_blockers = Vec::new();
    if residual_audit.fy_body_n.abs() > limits.max_side_force_n {
        residual_limit_blockers.push(QualificationBlocker::SideForceLimitExceeded {
            fy_body_n: residual_audit.fy_body_n,
            limit_n: limits.max_side_force_n,
        });
    }
    if residual_audit.mx_body_nm.abs() > limits.max_roll_moment_nm {
        residual_limit_blockers.push(QualificationBlocker::RollMomentLimitExceeded {
            mx_body_nm: residual_audit.mx_body_nm,
            limit_nm: limits.max_roll_moment_nm,
        });
    }
    if residual_audit.mz_body_nm.abs() > limits.max_yaw_moment_nm {
        residual_limit_blockers.push(QualificationBlocker::YawMomentLimitExceeded {
            mz_body_nm: residual_audit.mz_body_nm,
            limit_nm: limits.max_yaw_moment_nm,
        });
    }
    if residual_audit.linear_accel_world_y_mps2.abs() > limits.max_lateral_acceleration_mps2 {
        residual_limit_blockers.push(QualificationBlocker::LateralAccelerationLimitExceeded {
            ay_world_mps2: residual_audit.linear_accel_world_y_mps2,
            limit_mps2: limits.max_lateral_acceleration_mps2,
        });
    }
    if residual_audit.angular_accel_body_x_rad_s2.abs()
        > limits.max_roll_angular_acceleration_rad_s2
    {
        residual_limit_blockers.push(QualificationBlocker::RollAngularAccelerationLimitExceeded {
            angular_accel_body_x_rad_s2: residual_audit.angular_accel_body_x_rad_s2,
            limit_rad_s2: limits.max_roll_angular_acceleration_rad_s2,
        });
    }
    if residual_audit.angular_accel_body_z_rad_s2.abs() > limits.max_yaw_angular_acceleration_rad_s2
    {
        residual_limit_blockers.push(QualificationBlocker::YawAngularAccelerationLimitExceeded {
            angular_accel_body_z_rad_s2: residual_audit.angular_accel_body_z_rad_s2,
            limit_rad_s2: limits.max_yaw_angular_acceleration_rad_s2,
        });
    }

    // -----------------------------------------------------------------------
    // Phase 4: Integrity blockers (non-finite audit values, re-evaluation)
    // -----------------------------------------------------------------------
    let mut integrity_blockers = Vec::new();

    // FIX 4: Check finiteness of ALL audit values in deterministic order.

    // Aero audit values (in element order)
    for audit in &aero_audits {
        check_finite_field(
            &mut integrity_blockers,
            "alpha_geom_rad",
            audit.alpha_geom_rad,
        );
        check_finite_field(
            &mut integrity_blockers,
            "alpha_sample_rad",
            audit.alpha_sample_rad,
        );
        check_finite_field(
            &mut integrity_blockers,
            "alpha_lower_rad",
            audit.alpha_lower_rad,
        );
        check_finite_field(
            &mut integrity_blockers,
            "alpha_upper_rad",
            audit.alpha_upper_rad,
        );
        check_finite_field(
            &mut integrity_blockers,
            "section_airspeed_mps",
            audit.section_airspeed_mps,
        );
        if let Some(re) = audit.reynolds_number {
            check_finite_field(&mut integrity_blockers, "reynolds_number", re);
        }
        if let Some(re_lo) = audit.reynolds_lower {
            check_finite_field(&mut integrity_blockers, "reynolds_lower", re_lo);
        }
        if let Some(re_hi) = audit.reynolds_upper {
            check_finite_field(&mut integrity_blockers, "reynolds_upper", re_hi);
        }
        if let Some(ln_alpha_lo) = audit.reynolds_lower_node_alpha_lower_rad {
            check_finite_field(
                &mut integrity_blockers,
                "reynolds_lower_node_alpha_lower_rad",
                ln_alpha_lo,
            );
        }
        if let Some(ln_alpha_hi) = audit.reynolds_lower_node_alpha_upper_rad {
            check_finite_field(
                &mut integrity_blockers,
                "reynolds_lower_node_alpha_upper_rad",
                ln_alpha_hi,
            );
        }
        if let Some(un_alpha_lo) = audit.reynolds_upper_node_alpha_lower_rad {
            check_finite_field(
                &mut integrity_blockers,
                "reynolds_upper_node_alpha_lower_rad",
                un_alpha_lo,
            );
        }
        if let Some(un_alpha_hi) = audit.reynolds_upper_node_alpha_upper_rad {
            check_finite_field(
                &mut integrity_blockers,
                "reynolds_upper_node_alpha_upper_rad",
                un_alpha_hi,
            );
        }
    }

    // Propulsion audit values
    if let PropulsionDomainAudit::Present {
        throttle,
        axial_airspeed_mps,
        shaft_speed_rad_s,
        shaft_speed_rpm,
        advance_ratio_j,
        j_lower,
        j_upper,
        j_range_status: _,
        shaft_speed_domain,
        rpm_domain,
        thrust_n,
        torque_n_m,
    } = &propulsion_audit
    {
        check_finite_field(&mut integrity_blockers, "throttle", *throttle);
        check_finite_field(
            &mut integrity_blockers,
            "axial_airspeed_mps",
            *axial_airspeed_mps,
        );
        check_finite_field(
            &mut integrity_blockers,
            "shaft_speed_rad_s",
            *shaft_speed_rad_s,
        );
        check_finite_field(&mut integrity_blockers, "shaft_speed_rpm", *shaft_speed_rpm);
        check_finite_field(&mut integrity_blockers, "advance_ratio_j", *advance_ratio_j);
        check_finite_field(&mut integrity_blockers, "j_lower", *j_lower);
        check_finite_field(&mut integrity_blockers, "j_upper", *j_upper);
        check_finite_field(&mut integrity_blockers, "thrust_n", *thrust_n);
        check_finite_field(&mut integrity_blockers, "torque_n_m", *torque_n_m);
        if let Some(ss_domain) = shaft_speed_domain {
            check_finite_field(
                &mut integrity_blockers,
                "shaft_speed_lower_rad_s",
                ss_domain.shaft_speed_lower_rad_s,
            );
            check_finite_field(
                &mut integrity_blockers,
                "shaft_speed_upper_rad_s",
                ss_domain.shaft_speed_upper_rad_s,
            );
        }
        if let RpmDomainStatus::Declared {
            lower_rpm,
            upper_rpm,
            status: _,
        } = rpm_domain
        {
            check_finite_field(&mut integrity_blockers, "rpm_lower", *lower_rpm);
            check_finite_field(&mut integrity_blockers, "rpm_upper", *upper_rpm);
        }
    }

    // Residual audit values
    check_finite_field(
        &mut integrity_blockers,
        "fx_body_n",
        residual_audit.fx_body_n,
    );
    check_finite_field(
        &mut integrity_blockers,
        "fy_body_n",
        residual_audit.fy_body_n,
    );
    check_finite_field(
        &mut integrity_blockers,
        "fz_body_n",
        residual_audit.fz_body_n,
    );
    check_finite_field(
        &mut integrity_blockers,
        "mx_body_nm",
        residual_audit.mx_body_nm,
    );
    check_finite_field(
        &mut integrity_blockers,
        "my_body_nm",
        residual_audit.my_body_nm,
    );
    check_finite_field(
        &mut integrity_blockers,
        "mz_body_nm",
        residual_audit.mz_body_nm,
    );
    check_finite_field(
        &mut integrity_blockers,
        "linear_accel_world_x",
        residual_audit.linear_accel_world_x_mps2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "linear_accel_world_y",
        residual_audit.linear_accel_world_y_mps2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "linear_accel_world_z",
        residual_audit.linear_accel_world_z_mps2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "angular_accel_body_x",
        residual_audit.angular_accel_body_x_rad_s2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "angular_accel_body_y",
        residual_audit.angular_accel_body_y_rad_s2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "angular_accel_body_z",
        residual_audit.angular_accel_body_z_rad_s2,
    );
    check_finite_field(
        &mut integrity_blockers,
        "longitudinal_force_n",
        residual_audit.longitudinal_force_n,
    );
    check_finite_field(
        &mut integrity_blockers,
        "vertical_force_n",
        residual_audit.vertical_force_n,
    );
    check_finite_field(
        &mut integrity_blockers,
        "pitch_moment_nm",
        residual_audit.pitch_moment_nm,
    );

    // Re-evaluation integrity
    if !re_eval_matches {
        integrity_blockers.push(QualificationBlocker::ReEvaluationFailure);
    }

    // -----------------------------------------------------------------------
    // Assemble final blocker list in documented order
    // -----------------------------------------------------------------------
    let mut blockers = Vec::with_capacity(
        aero_blockers.len()
            + propulsion_blockers.len()
            + residual_limit_blockers.len()
            + integrity_blockers.len(),
    );
    blockers.extend(aero_blockers);
    blockers.extend(propulsion_blockers);
    blockers.extend(residual_limit_blockers);
    blockers.extend(integrity_blockers);

    let outcome = if blockers
        .iter()
        .any(|b| b.category() == QualificationBlockerCategory::Domain)
    {
        LongitudinalTrimQualificationOutcome::NotQualifiedDomainViolation(
            QualificationDiagnostics {
                blockers,
                residual_audit,
                aero_audits,
                propulsion_audit,
            },
        )
    } else if blockers
        .iter()
        .any(|b| b.category() == QualificationBlockerCategory::Residual)
    {
        LongitudinalTrimQualificationOutcome::NotQualifiedResidualViolation(
            QualificationDiagnostics {
                blockers,
                residual_audit,
                aero_audits,
                propulsion_audit,
            },
        )
    } else if blockers.is_empty() {
        LongitudinalTrimQualificationOutcome::Qualified(QualificationDiagnostics {
            blockers,
            residual_audit,
            aero_audits,
            propulsion_audit,
        })
    } else {
        // Integrity-only failure: every blocker is an integrity blocker. Report WHY
        // trustworthy qualification is unavailable, preserving the audited diagnostics.
        let reason = blockers.iter().find_map(|b| match b {
            QualificationBlocker::ReEvaluationFailure => {
                Some(QualificationUnavailableReason::ReEvaluationFailure)
            }
            QualificationBlocker::NonFiniteAuditValue { field } => {
                Some(QualificationUnavailableReason::NonFiniteAuditValue { field })
            }
            _ => None,
        });
        LongitudinalTrimQualificationOutcome::QualificationUnavailable {
            reason: reason.expect("integrity-only blockers always map to a reason"),
            diagnostics: Some(QualificationDiagnostics {
                blockers,
                residual_audit,
                aero_audits,
                propulsion_audit,
            }),
        }
    };

    LongitudinalTrimQualificationPoint {
        target_airspeed_mps,
        outcome,
    }
}

/// Qualifies a whole longitudinal trim sweep, preserving the sweep's point order exactly.
///
/// The returned collection contains exactly one entry per sweep point, in the same order
/// as the sweep's (and therefore the request's) target airspeeds. For every sweep point:
///
/// - `Success` points are fully audited through [`qualify_longitudinal_trim_solution`].
/// - `TrimFailure` points map to
///   [`LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure`] carrying the typed
///   solver failure. NO element, propulsion, or residual diagnostics are fabricated.
/// - `ReEvaluationMismatch` / `ReEvaluationUnverifiable` points map to
///   [`LongitudinalTrimQualificationOutcome::QualificationUnavailable`] with NO
///   diagnostics: the point's evaluation integrity was already broken at sweep level,
///   so auditing its cached values would fabricate untrustworthy evidence.
#[must_use]
pub fn qualify_longitudinal_trim_sweep(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep: &crate::trim_sweep::LongitudinalTrimSweep,
    limits: &LongitudinalTrimQualificationLimits,
) -> LongitudinalTrimQualification {
    let points = sweep
        .points()
        .iter()
        .map(|sweep_point| {
            let outcome = match &sweep_point.outcome {
                crate::trim_sweep::LongitudinalTrimSweepOutcome::Success { solution } => {
                    qualify_longitudinal_trim_solution(
                        model,
                        config,
                        solution,
                        limits,
                        sweep_point.target_airspeed_mps,
                    )
                    .outcome
                }
                crate::trim_sweep::LongitudinalTrimSweepOutcome::TrimFailure { failure } => {
                    LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure {
                        failure: failure.clone(),
                    }
                }
                crate::trim_sweep::LongitudinalTrimSweepOutcome::ReEvaluationMismatch(_) => {
                    LongitudinalTrimQualificationOutcome::QualificationUnavailable {
                        reason: QualificationUnavailableReason::SweepReEvaluationMismatch,
                        diagnostics: None,
                    }
                }
                crate::trim_sweep::LongitudinalTrimSweepOutcome::ReEvaluationUnverifiable(_) => {
                    LongitudinalTrimQualificationOutcome::QualificationUnavailable {
                        reason: QualificationUnavailableReason::SweepReEvaluationUnverifiable,
                        diagnostics: None,
                    }
                }
            };
            LongitudinalTrimQualificationPoint {
                target_airspeed_mps: sweep_point.target_airspeed_mps,
                outcome,
            }
        })
        .collect();
    LongitudinalTrimQualification::from_points(points)
}

fn check_finite_field(blockers: &mut Vec<QualificationBlocker>, field: &'static str, value: f64) {
    if !value.is_finite() {
        blockers.push(QualificationBlocker::NonFiniteAuditValue { field });
    }
}

// ---------------------------------------------------------------------------
// Aero element audit
// ---------------------------------------------------------------------------

fn audit_aero_element(
    elem_idx: usize,
    runtime_elem: &model::RuntimeAeroElement,
    eff_elem: &sim_core::AeroElement,
    model: &AircraftModel,
    alpha_geom: f64,
    alpha_sample: f64,
    section_airspeed: f64,
) -> (AerodynamicElementDomainAudit, Vec<QualificationBlocker>) {
    let mut blockers = Vec::new();

    match runtime_elem.polar_binding() {
        model::RuntimeAeroPolarBinding::Polar { polar_index } => {
            let table = &model.aero_polars()[polar_index].table();
            let (alpha_lo, alpha_hi) = polar_alpha_bounds(table);
            let status = classify_range_status(alpha_sample, alpha_lo, alpha_hi);

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
            let re_status = classify_range_status(re, re_lo, re_hi);

            // Contributing-node alpha blockers come BEFORE Reynolds range blockers
            let (lower_node, upper_node, _fraction) = find_reynolds_bracket(family, re);

            let mut ln_node_alpha_lo = None;
            let mut ln_node_alpha_hi = None;
            let mut un_node_alpha_lo = None;
            let mut un_node_alpha_hi = None;

            if let Some(ln) = lower_node {
                let (lo, hi) = polar_alpha_bounds(ln.table());
                ln_node_alpha_lo = Some(lo);
                ln_node_alpha_hi = Some(hi);
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

            // Reynolds range blockers come AFTER contributing-node alpha blockers
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

            let alpha_lo = ln_node_alpha_lo
                .unwrap_or(0.0)
                .max(un_node_alpha_lo.unwrap_or(0.0));
            let alpha_hi = ln_node_alpha_hi
                .unwrap_or(0.0)
                .min(un_node_alpha_hi.unwrap_or(0.0));
            let alpha_status = if let (Some(ln), Some(un)) = (lower_node, upper_node) {
                if ln.reynolds_number() == un.reynolds_number() {
                    let (lo, hi) = polar_alpha_bounds(ln.table());
                    classify_range_status(alpha_sample, lo, hi)
                } else {
                    classify_range_status(alpha_sample, alpha_lo, alpha_hi)
                }
            } else {
                classify_range_status(alpha_sample, alpha_lo, alpha_hi)
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
    accepted_throttle: f64,
    blockers: &mut Vec<QualificationBlocker>,
) -> PropulsionDomainAudit {
    let Some(runtime_prop) = model.propulsion() else {
        return PropulsionDomainAudit::NotPresent;
    };

    let prop_output = evaluate_electric_propulsion_with_source(
        state,
        accepted_throttle,
        runtime_prop.config(),
        config.aero_environment(),
        runtime_prop.coefficient_source(),
    );

    let j = prop_output.advance_ratio_j;
    let shaft_speed = prop_output.shaft_speed_rad_s;

    match runtime_prop.coefficient_source() {
        PropellerCoefficientSource::FixedTable(table) => {
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let j_stat = classify_range_status(j, j_lo, j_hi);
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
            // FIX 6: Fixed table has no shaft-speed map domain -> None.
            // No authored RPM support range exists in the model/runtime data -> NotDeclared.
            PropulsionDomainAudit::Present {
                throttle: accepted_throttle,
                axial_airspeed_mps: prop_output.axial_airspeed_mps,
                shaft_speed_rad_s: shaft_speed,
                shaft_speed_rpm: prop_output.shaft_speed_rpm,
                advance_ratio_j: j,
                j_lower: j_lo,
                j_upper: j_hi,
                j_range_status: j_stat,
                shaft_speed_domain: None,
                rpm_domain: RpmDomainStatus::NotDeclared,
                thrust_n: prop_output.thrust_n,
                torque_n_m: prop_output.motor_torque_nm,
            }
        }
        PropellerCoefficientSource::ShaftSpeedMap(map) => {
            let nodes = map.nodes();
            let ss_lo = nodes[0].shaft_speed_rad_s();
            let ss_hi = nodes[nodes.len() - 1].shaft_speed_rad_s();
            let ss_stat = classify_range_status(shaft_speed, ss_lo, ss_hi);
            // Shaft-speed blockers BEFORE J blockers (documented order).
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

            let (j_lo, j_hi, j_stat) = audit_map_j_domain(map, shaft_speed, j, blockers);
            PropulsionDomainAudit::Present {
                throttle: accepted_throttle,
                axial_airspeed_mps: prop_output.axial_airspeed_mps,
                shaft_speed_rad_s: shaft_speed,
                shaft_speed_rpm: prop_output.shaft_speed_rpm,
                advance_ratio_j: j,
                j_lower: j_lo,
                j_upper: j_hi,
                j_range_status: j_stat,
                shaft_speed_domain: Some(ShaftSpeedDomainAudit {
                    shaft_speed_lower_rad_s: ss_lo,
                    shaft_speed_upper_rad_s: ss_hi,
                    shaft_speed_range_status: ss_stat,
                }),
                // No explicit authored RPM support range exists in the model/runtime
                // data (the map's node range is an authored shaft-speed domain, not an
                // RPM support range) -> NotDeclared. No RPM envelope is invented.
                rpm_domain: RpmDomainStatus::NotDeclared,
                thrust_n: prop_output.thrust_n,
                torque_n_m: prop_output.motor_torque_nm,
            }
        }
    }
}

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
        let status = classify_range_status(j, j_lo, j_hi);
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

    match nodes.binary_search_by(|n| n.shaft_speed_rad_s().total_cmp(&shaft_speed)) {
        Ok(idx) => {
            let table = nodes[idx].table();
            let samples = table.samples();
            let j_lo = samples[0].advance_ratio_j;
            let j_hi = samples[samples.len() - 1].advance_ratio_j;
            let status = classify_range_status(j, j_lo, j_hi);
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
            let status = classify_range_status(j, j_lo, j_hi);
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
            let status = classify_range_status(j, j_lo, j_hi);
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

            let j_lo = j_lo_lower.max(j_lo_upper);
            let j_hi = j_hi_lower.min(j_hi_upper);
            let status = classify_range_status(j, j_lo, j_hi);

            let j_in_lower = classify_range_status(j, j_lo_lower, j_hi_lower);
            let j_in_upper = classify_range_status(j, j_lo_upper, j_hi_upper);

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

// ---------------------------------------------------------------------------
// Unit tests for private helpers
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_status_nan_is_non_finite() {
        assert_eq!(
            classify_range_status(f64::NAN, -1.0, 1.0),
            RangeStatus::NonFinite,
            "NaN must NOT classify as InRange"
        );
    }

    #[test]
    fn range_status_infinity_is_non_finite() {
        assert_eq!(
            classify_range_status(f64::INFINITY, -1.0, 1.0),
            RangeStatus::NonFinite
        );
        assert_eq!(
            classify_range_status(f64::NEG_INFINITY, -1.0, 1.0),
            RangeStatus::NonFinite
        );
    }

    #[test]
    fn range_status_finite_values_classify_correctly() {
        assert_eq!(classify_range_status(0.0, -1.0, 1.0), RangeStatus::InRange);
        assert_eq!(
            classify_range_status(-2.0, -1.0, 1.0),
            RangeStatus::BelowRange
        );
        assert_eq!(
            classify_range_status(2.0, -1.0, 1.0),
            RangeStatus::AboveRange
        );
    }

    #[test]
    fn range_status_distinguishes_exact_bounds_from_interior() {
        // Exact bitwise lower endpoint
        assert_eq!(
            classify_range_status(-1.0, -1.0, 1.0),
            RangeStatus::AtLowerBound
        );
        // Exact bitwise upper endpoint
        assert_eq!(
            classify_range_status(1.0, -1.0, 1.0),
            RangeStatus::AtUpperBound
        );
        // Strictly interior on both sides
        assert_eq!(
            classify_range_status(-1.0 + f64::EPSILON, -1.0, 1.0),
            RangeStatus::InRange
        );
        assert_eq!(
            classify_range_status(1.0 - f64::EPSILON, -1.0, 1.0),
            RangeStatus::InRange
        );
        // Just outside is NOT a bound
        assert_eq!(
            classify_range_status(-1.0 - f64::EPSILON, -1.0, 1.0),
            RangeStatus::BelowRange
        );
        assert_eq!(
            classify_range_status(1.0 + f64::EPSILON, -1.0, 1.0),
            RangeStatus::AboveRange
        );
    }

    #[test]
    fn range_status_degenerate_interval_only_admits_exact_endpoint() {
        // Single-point support: only the exact value is inside the closed interval.
        // Documented precedence: the lower endpoint is checked first, so the exact value
        // of a degenerate interval classifies as AtLowerBound.
        assert_eq!(
            classify_range_status(0.5, 0.5, 0.5),
            RangeStatus::AtLowerBound
        );
        assert_eq!(
            classify_range_status(0.5 + f64::EPSILON, 0.5, 0.5),
            RangeStatus::AboveRange
        );
        assert_eq!(
            classify_range_status(0.5 - f64::EPSILON, 0.5, 0.5),
            RangeStatus::BelowRange
        );
    }
}
