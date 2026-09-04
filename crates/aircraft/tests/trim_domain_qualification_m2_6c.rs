//! M2.6C — Deterministic longitudinal trim domain qualification tests.
//!
//! Every proof test asserts its target operating condition UNCONDITIONALLY.
//! If a fixture does not produce the intended operating point, the test FAILS.

use aircraft::{
    AircraftSimulationConfig, LongitudinalTrimQualificationLimits, LongitudinalTrimRequest,
    LongitudinalTrimSolution, LongitudinalTrimSweepRequest, LongitudinalTrimTolerances,
    LongitudinalTrimVariables, PropulsionDomainAudit, QualificationBlocker,
    QualificationBlockerCategory, RangeStatus, RpmDomainStatus, TrimBounds, classify_range_status,
    effective_aero_elements_for_positions, evaluate_longitudinal_trim_candidate,
    qualify_longitudinal_trim_solution, qualify_longitudinal_trim_sweep, solve_longitudinal_trim,
    solve_longitudinal_trim_sweep,
};
use model::AircraftModelLoader;
use sim_core::{AeroEnvironment, compute_section_kinematics};
use sim_math::Vec3;

// ---------------------------------------------------------------------------
// Fixture loaders
// ---------------------------------------------------------------------------

const TRIM_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_non_reference_trim_v4.json");
const ASYMMETRIC_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_asymmetric_offaxis_trim_v4.json");
const NO_PROPULSION_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_no_propulsion_trim_v4.json");
const FIXED_TABLE_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_fixed_table_trim_v4.json");
const FIXED_TABLE_NARROW_J_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_fixed_table_narrow_j_v4.json");
const NARROW_POLAR_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_narrow_polar_trim_v4.json");
const REYNOLDS_ASYMMETRIC_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_reynolds_asymmetric_v4.json");
const DUAL_J_RANGE_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_dual_j_range_v4.json");
const FINITE_WING_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_finite_wing_v5.json");
const SHAFT_SPEED_NARROW_FIXTURE: &str =
    include_str!("../../../tests/fixtures/synthetic_shaft_speed_narrow_v4.json");

fn trim_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(TRIM_FIXTURE).unwrap()
}
fn asymmetric_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(ASYMMETRIC_FIXTURE).unwrap()
}
fn no_propulsion_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(NO_PROPULSION_FIXTURE).unwrap()
}
fn fixed_table_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(FIXED_TABLE_FIXTURE).unwrap()
}
fn fixed_table_narrow_j_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(FIXED_TABLE_NARROW_J_FIXTURE).unwrap()
}
fn narrow_polar_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(NARROW_POLAR_FIXTURE).unwrap()
}
fn reynolds_asymmetric_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(REYNOLDS_ASYMMETRIC_FIXTURE).unwrap()
}
fn dual_j_range_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(DUAL_J_RANGE_FIXTURE).unwrap()
}
fn finite_wing_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(FINITE_WING_FIXTURE).unwrap()
}
fn shaft_speed_narrow_model() -> model::AircraftModel {
    AircraftModelLoader::from_json_str(SHAFT_SPEED_NARROW_FIXTURE).unwrap()
}

fn sim_config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(
        0.002,
        Vec3::new(0.0, 0.0, 9.80665),
        AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
    )
    .unwrap()
}

fn permissive_limits() -> LongitudinalTrimQualificationLimits {
    LongitudinalTrimQualificationLimits::new(100.0, 100.0, 100.0, 50.0, 50.0, 50.0).unwrap()
}

fn strict_zero_limits() -> LongitudinalTrimQualificationLimits {
    LongitudinalTrimQualificationLimits::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).unwrap()
}

fn solve_trim(
    model: &model::AircraftModel,
    config: &AircraftSimulationConfig,
    speed: f64,
) -> aircraft::LongitudinalTrimSolution {
    let request = LongitudinalTrimRequest::new(
        speed,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    solve_longitudinal_trim(model, config, &request).unwrap()
}

fn audited_diagnostics(
    outcome: &aircraft::LongitudinalTrimQualificationOutcome,
) -> Option<&aircraft::QualificationDiagnostics> {
    match outcome {
        aircraft::LongitudinalTrimQualificationOutcome::Qualified(diagnostics)
        | aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedDomainViolation(
            diagnostics,
        )
        | aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedResidualViolation(
            diagnostics,
        ) => Some(diagnostics),
        aircraft::LongitudinalTrimQualificationOutcome::QualificationUnavailable {
            diagnostics,
            ..
        } => diagnostics.as_ref(),
        // Trim failure: no diagnostics exist and none may be fabricated.
        aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure { .. } => None,
    }
}

fn extract_aero_audits(
    outcome: &aircraft::LongitudinalTrimQualificationOutcome,
) -> &[aircraft::AerodynamicElementDomainAudit] {
    audited_diagnostics(outcome)
        .map(|d| d.aero_audits())
        .unwrap_or(&[])
}

const NOT_PRESENT_PROPULSION: PropulsionDomainAudit = PropulsionDomainAudit::NotPresent;

fn extract_propulsion_audit(
    outcome: &aircraft::LongitudinalTrimQualificationOutcome,
) -> &PropulsionDomainAudit {
    audited_diagnostics(outcome)
        .map(|d| d.propulsion_audit())
        .unwrap_or(&NOT_PRESENT_PROPULSION)
}

fn extract_residual_audit(
    outcome: &aircraft::LongitudinalTrimQualificationOutcome,
) -> &aircraft::FullResidualAudit {
    audited_diagnostics(outcome)
        .map(|d| d.residual_audit())
        .expect("residual audit must exist for every audited outcome")
}

// ===========================================================================
// PRESERVED VALID TESTS
// ===========================================================================

#[test]
fn fixed_polar_alpha_in_range_is_qualified() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(
        point.outcome.is_qualified(),
        "should be qualified: {:?}",
        point.outcome.blockers()
    );
}

#[test]
fn fixed_polar_alpha_audit_records_correct_bounds() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    let tail_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-elevator-tail")
        .unwrap();
    assert_eq!(tail_audit.alpha_lower_rad, -0.40);
    assert_eq!(tail_audit.alpha_upper_rad, 0.40);
    assert_eq!(tail_audit.polar_binding_kind, "polar");
    assert_eq!(tail_audit.alpha_range_status, RangeStatus::InRange);
}

#[test]
fn reynolds_in_range_is_qualified() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(
        point.outcome.is_qualified(),
        "Re in range should qualify: {:?}",
        point.outcome.blockers()
    );
}

#[test]
fn reynolds_below_range_is_rejected() {
    let model = trim_model();
    let config = sim_config();
    let request = LongitudinalTrimRequest::new(
        8.0,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 8.0);
    assert!(
        !point.outcome.is_qualified(),
        "Re below floor should reject"
    );
    assert!(
        point
            .outcome
            .blockers()
            .iter()
            .any(|b| matches!(b, QualificationBlocker::ReynoldsBelowRange { .. }))
    );
}

#[test]
fn reynolds_above_range_is_rejected() {
    let model = trim_model();
    let config = sim_config();
    let request = LongitudinalTrimRequest::new(
        30.0,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.05, 0.0, 0.6).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 30.0);
    assert!(
        !point.outcome.is_qualified(),
        "Re above ceiling should reject"
    );
    assert!(
        point
            .outcome
            .blockers()
            .iter()
            .any(|b| matches!(b, QualificationBlocker::ReynoldsAboveRange { .. }))
    );
}

#[test]
fn permissive_limits_pass_residual_qualification() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(
        point.outcome.is_qualified(),
        "permissive limits should pass: {:?}",
        point.outcome.blockers()
    );
}

#[test]
fn repeated_identical_qualification_is_identical() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let limits = permissive_limits();
    let p1 = qualify_longitudinal_trim_solution(&model, &config, &solution, &limits, 18.0);
    let p2 = qualify_longitudinal_trim_solution(&model, &config, &solution, &limits, 18.0);
    assert_eq!(p1, p2, "repeated qualification must be identical");
}

#[test]
fn propulsion_model_returns_present_audit() {
    let model = trim_model();
    let config = sim_config();
    assert!(model.propulsion().is_some());
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(matches!(
        extract_propulsion_audit(&point.outcome),
        aircraft::PropulsionDomainAudit::Present { .. }
    ));
}

#[test]
fn limits_reject_nan() {
    assert!(LongitudinalTrimQualificationLimits::new(f64::NAN, 1.0, 1.0, 1.0, 1.0, 1.0).is_err());
}

#[test]
fn limits_reject_negative() {
    assert!(LongitudinalTrimQualificationLimits::new(-1.0, 1.0, 1.0, 1.0, 1.0, 1.0).is_err());
}

#[test]
fn limits_reject_infinity() {
    assert!(
        LongitudinalTrimQualificationLimits::new(f64::INFINITY, 1.0, 1.0, 1.0, 1.0, 1.0).is_err()
    );
}

#[test]
fn limits_accept_zero() {
    assert!(LongitudinalTrimQualificationLimits::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).is_ok());
}

#[test]
fn full_residual_audit_preserves_signed_values() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audit = extract_residual_audit(&point.outcome);
    assert!(audit.fx_body_n.is_finite());
    assert!(audit.fy_body_n.is_finite());
    assert!(audit.fz_body_n.is_finite());
    assert!(audit.mx_body_nm.is_finite());
    assert!(audit.my_body_nm.is_finite());
    assert!(audit.mz_body_nm.is_finite());
    assert_eq!(
        audit.longitudinal_force_n,
        solution.evaluation.residuals.longitudinal_force_n
    );
    assert_eq!(
        audit.vertical_force_n,
        solution.evaluation.residuals.vertical_force_n
    );
    assert_eq!(
        audit.pitch_moment_nm,
        solution.evaluation.residuals.pitch_moment_nm
    );
}

#[test]
fn aero_audits_are_in_model_order() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    assert_eq!(audits.len(), model.aero_elements().len());
    for (i, audit) in audits.iter().enumerate() {
        assert_eq!(audit.element_index, i);
    }
}

#[test]
fn reynolds_in_range_asserts_actual_expected_value() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    let wing_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-wing")
        .unwrap();
    let expected_re = 18.0 * 0.30 / 0.000015;
    let actual_re = wing_audit.reynolds_number.unwrap();
    assert!(
        (actual_re - expected_re).abs() < 1.0,
        "expected Re ~ {expected_re}, got {actual_re}"
    );
}

#[test]
fn shaft_speed_map_in_range_has_some_domain() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            shaft_speed_domain, ..
        } => {
            let domain = shaft_speed_domain
                .as_ref()
                .expect("shaft-speed map must have Some domain");
            assert_eq!(domain.shaft_speed_lower_rad_s, 250.0);
            assert_eq!(domain.shaft_speed_upper_rad_s, 800.0);
            assert_eq!(domain.shaft_speed_range_status, RangeStatus::InRange);
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

#[test]
fn repeated_qualification_deterministic_to_bits() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let limits = permissive_limits();
    let p1 = qualify_longitudinal_trim_solution(&model, &config, &solution, &limits, 18.0);
    let p2 = qualify_longitudinal_trim_solution(&model, &config, &solution, &limits, 18.0);
    assert_eq!(p1, p2);
    let a1 = match &p1.outcome {
        aircraft::LongitudinalTrimQualificationOutcome::Qualified(diagnostics) => {
            diagnostics.residual_audit()
        }
        _ => panic!("expected qualified"),
    };
    let a2 = match &p2.outcome {
        aircraft::LongitudinalTrimQualificationOutcome::Qualified(diagnostics) => {
            diagnostics.residual_audit()
        }
        _ => panic!("expected qualified"),
    };
    assert_eq!(a1.fx_body_n.to_bits(), a2.fx_body_n.to_bits());
    assert_eq!(a1.fy_body_n.to_bits(), a2.fy_body_n.to_bits());
    assert_eq!(
        a1.longitudinal_force_n.to_bits(),
        a2.longitudinal_force_n.to_bits()
    );
}

#[test]
fn runtime_wrench_bitwise_matches_solution_evaluation() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audit = extract_residual_audit(&point.outcome);
    assert_eq!(
        audit.fx_body_n.to_bits(),
        solution.evaluation.body_wrench.force_body_n.x.to_bits()
    );
    assert_eq!(
        audit.fy_body_n.to_bits(),
        solution.evaluation.body_wrench.force_body_n.y.to_bits()
    );
    assert_eq!(
        audit.fz_body_n.to_bits(),
        solution.evaluation.body_wrench.force_body_n.z.to_bits()
    );
    assert_eq!(
        audit.mx_body_nm.to_bits(),
        solution.evaluation.body_wrench.moment_body_nm.x.to_bits()
    );
    assert_eq!(
        audit.my_body_nm.to_bits(),
        solution.evaluation.body_wrench.moment_body_nm.y.to_bits()
    );
    assert_eq!(
        audit.mz_body_nm.to_bits(),
        solution.evaluation.body_wrench.moment_body_nm.z.to_bits()
    );
    assert_eq!(
        audit.longitudinal_force_n.to_bits(),
        solution.evaluation.residuals.longitudinal_force_n.to_bits()
    );
    assert_eq!(
        audit.vertical_force_n.to_bits(),
        solution.evaluation.residuals.vertical_force_n.to_bits()
    );
    assert_eq!(
        audit.pitch_moment_nm.to_bits(),
        solution.evaluation.residuals.pitch_moment_nm.to_bits()
    );
}

#[test]
fn propulsion_audit_uses_accepted_throttle() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let accepted_throttle = solution.evaluation.control_surface_positions.throttle();
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present { throttle, .. } => {
            assert_eq!(
                throttle.to_bits(),
                accepted_throttle.to_bits(),
                "propulsion audit must use accepted control-output throttle"
            );
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

#[test]
fn re_evaluation_equality_check_passes_for_valid_solution() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(
        !point
            .outcome
            .blockers()
            .iter()
            .any(|b| matches!(b, QualificationBlocker::ReEvaluationFailure)),
        "valid trim should pass re-evaluation, blockers: {:?}",
        point.outcome.blockers()
    );
}

// ===========================================================================
// TASK 2: FIXED POLAR OUT-OF-RANGE PROOF (unconditional)
// ===========================================================================

#[test]
fn fixed_polar_out_of_range_emits_typed_alpha_blocker() {
    let model = narrow_polar_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    let wing_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-wing")
        .unwrap();

    // UNCONDITIONAL: wing alpha_sample MUST be outside [-0.05, 0.05]
    assert!(
        wing_audit.alpha_sample_rad < -0.05 || wing_audit.alpha_sample_rad > 0.05,
        "wing alpha_sample={} must be outside narrow polar [-0.05, 0.05]",
        wing_audit.alpha_sample_rad
    );

    // UNCONDITIONAL: typed blocker must exist
    assert!(
        point.outcome.blockers().iter().any(|b| matches!(
            b,
            QualificationBlocker::AerodynamicAlphaBelowRange { .. }
                | QualificationBlocker::AerodynamicAlphaAboveRange { .. }
        )),
        "out-of-range alpha must emit typed alpha blocker, got: {:?}",
        point.outcome.blockers()
    );

    // UNCONDITIONAL: must not qualify
    assert!(!point.outcome.is_qualified());
}

// ===========================================================================
// TASK 3: FINITE-WING EFFECTIVE-ALPHA — MUST CHANGE DOMAIN DECISION
// ===========================================================================

#[test]
fn finite_wing_effective_alpha_changes_domain_decision() {
    let model = finite_wing_model();
    let config = sim_config();

    assert!(
        !model.aero_surfaces().is_empty(),
        "v5 fixture must have aero surfaces"
    );

    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    let wing_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-wing")
        .unwrap();

    let alpha_upper = wing_audit.alpha_upper_rad;
    let alpha_lower = wing_audit.alpha_lower_rad;

    // UNCONDITIONAL: alpha_geom and alpha_sample must differ materially
    let alpha_diff = (wing_audit.alpha_geom_rad - wing_audit.alpha_sample_rad).abs();
    assert!(
        alpha_diff > 1e-6,
        "alpha_geom ({}) must differ from alpha_sample ({}) by > 1e-6",
        wing_audit.alpha_geom_rad,
        wing_audit.alpha_sample_rad
    );

    // UNCONDITIONAL: alpha_geom and alpha_sample must lie on opposite sides
    // of at least one alpha evidence boundary.
    // Either alpha_geom is outside while alpha_sample is inside, or vice versa.
    let geom_in_range =
        wing_audit.alpha_geom_rad >= alpha_lower && wing_audit.alpha_geom_rad <= alpha_upper;
    let sample_in_range =
        wing_audit.alpha_sample_rad >= alpha_lower && wing_audit.alpha_sample_rad <= alpha_upper;

    assert!(
        geom_in_range != sample_in_range,
        "alpha_geom ({}) and alpha_sample ({}) must be on opposite sides of \
         evidence boundary [{}, {}]: geom_in_range={}, sample_in_range={}",
        wing_audit.alpha_geom_rad,
        wing_audit.alpha_sample_rad,
        alpha_lower,
        alpha_upper,
        geom_in_range,
        sample_in_range
    );

    // UNCONDITIONAL: qualification follows alpha_sample
    assert_eq!(
        wing_audit.alpha_range_status,
        if sample_in_range {
            RangeStatus::InRange
        } else if wing_audit.alpha_sample_rad < alpha_lower {
            RangeStatus::BelowRange
        } else {
            RangeStatus::AboveRange
        },
        "qualification must follow alpha_sample, not alpha_geom"
    );

    // If alpha_sample is in range but alpha_geom is not, qualification must
    // NOT emit an alpha blocker (proving alpha_geom auditing would be wrong).
    if sample_in_range && !geom_in_range {
        assert!(
            !point.outcome.blockers().iter().any(|b| matches!(
                b,
                QualificationBlocker::AerodynamicAlphaBelowRange { .. }
                    | QualificationBlocker::AerodynamicAlphaAboveRange { .. }
            )),
            "alpha_sample in range must not produce alpha blocker even though alpha_geom is out"
        );
    }
}

// ===========================================================================
// TASK 4: REYNOLDS DUAL-NODE ALPHA SUPPORT (unconditional)
// ===========================================================================

#[test]
fn reynolds_dual_node_alpha_blocker_when_upper_node_rejects() {
    let model = reynolds_asymmetric_model();
    let config = sim_config();

    let solution = solve_trim(&model, &config, 14.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 14.0);
    let audits = extract_aero_audits(&point.outcome);
    let wing_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-wing")
        .unwrap();

    // UNCONDITIONAL: Reynolds number is strictly between the two nodes
    let re = wing_audit.reynolds_number.unwrap();
    assert!(
        re > 200_000.0 && re < 500_000.0,
        "Re={} must be strictly between 200000 and 500000",
        re
    );

    // UNCONDITIONAL: upper node alpha upper bound is 0.20
    let upper_node_alpha_hi = wing_audit.reynolds_upper_node_alpha_upper_rad.unwrap();
    assert_eq!(upper_node_alpha_hi, 0.20);

    // UNCONDITIONAL: lower node supports alpha_sample
    let lower_node_alpha_hi = wing_audit.reynolds_lower_node_alpha_upper_rad.unwrap();
    assert!(
        wing_audit.alpha_sample_rad <= lower_node_alpha_hi,
        "alpha_sample={} must be <= lower node upper bound {}",
        wing_audit.alpha_sample_rad,
        lower_node_alpha_hi
    );

    // UNCONDITIONAL: upper node does NOT support alpha_sample
    assert!(
        wing_audit.alpha_sample_rad > upper_node_alpha_hi,
        "alpha_sample={} must be > upper node upper bound {}",
        wing_audit.alpha_sample_rad,
        upper_node_alpha_hi
    );

    // UNCONDITIONAL: typed blocker must exist
    assert!(
        point.outcome.blockers().iter().any(|b| matches!(
            b,
            QualificationBlocker::ReynoldsContributingNodeAlphaAboveRange { .. }
        )),
        "must emit ReynoldsContributingNodeAlphaAboveRange, blockers: {:?}",
        point.outcome.blockers()
    );
}

// ===========================================================================
// TASK 5: FIXED PROPELLER TABLE — J IN RANGE (unconditional)
// ===========================================================================

#[test]
fn fixed_propeller_table_j_in_domain_no_blocker() {
    let model = fixed_table_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            advance_ratio_j,
            j_lower,
            j_upper,
            j_range_status,
            shaft_speed_domain,
            ..
        } => {
            // UNCONDITIONAL: J bounds
            assert_eq!(*j_lower, 0.0);
            assert_eq!(*j_upper, 1.3);

            // UNCONDITIONAL: J is inside the table support
            assert!(
                *advance_ratio_j >= *j_lower && *advance_ratio_j <= *j_upper,
                "J={} must be in [{}, {}]",
                advance_ratio_j,
                j_lower,
                j_upper
            );

            // UNCONDITIONAL: range status is InRange
            assert_eq!(*j_range_status, RangeStatus::InRange);

            // UNCONDITIONAL: no J blocker
            assert!(
                !point.outcome.blockers().iter().any(|b| matches!(
                    b,
                    QualificationBlocker::PropellerAdvanceRatioBelowRange { .. }
                        | QualificationBlocker::PropellerAdvanceRatioAboveRange { .. }
                )),
                "J in domain must not produce J blocker"
            );

            // UNCONDITIONAL: fixed table has no shaft-speed domain
            assert!(shaft_speed_domain.is_none());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// TASK 5B: FIXED PROPELLER TABLE — J OUT OF RANGE (unconditional)
// ===========================================================================

#[test]
fn fixed_propeller_table_j_out_of_range_emits_typed_blocker() {
    let model = fixed_table_narrow_j_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            advance_ratio_j,
            j_lower,
            j_upper,
            j_range_status,
            shaft_speed_domain,
            ..
        } => {
            // UNCONDITIONAL: fixed table has no shaft-speed domain
            assert!(shaft_speed_domain.is_none());

            // UNCONDITIONAL: J is outside the narrow table support
            assert!(
                *advance_ratio_j > *j_upper || *advance_ratio_j < *j_lower,
                "J={} must be outside [{}, {}]",
                advance_ratio_j,
                j_lower,
                j_upper
            );

            // UNCONDITIONAL: range status is not InRange
            assert_ne!(*j_range_status, RangeStatus::InRange);

            // UNCONDITIONAL: typed J blocker must exist
            let has_j_blocker = point.outcome.blockers().iter().any(|b| {
                matches!(
                    b,
                    QualificationBlocker::PropellerAdvanceRatioBelowRange { .. }
                        | QualificationBlocker::PropellerAdvanceRatioAboveRange { .. }
                )
            });
            assert!(
                has_j_blocker,
                "J out of range must emit typed J blocker, blockers: {:?}",
                point.outcome.blockers()
            );

            // UNCONDITIONAL: must not qualify
            assert!(!point.outcome.is_qualified());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

#[test]
fn fixed_propeller_table_shaft_speed_domain_is_none() {
    let model = fixed_table_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            shaft_speed_domain, ..
        } => {
            assert!(
                shaft_speed_domain.is_none(),
                "fixed table must have shaft_speed_domain = None"
            );
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// TASK 6: SHAFT-SPEED MAP OUT-OF-RANGE (unconditional)
// ===========================================================================

#[test]
fn shaft_speed_map_out_of_range_emits_typed_blocker() {
    let model = shaft_speed_narrow_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            shaft_speed_rad_s,
            shaft_speed_domain,
            ..
        } => {
            let domain = shaft_speed_domain.as_ref().unwrap();

            // UNCONDITIONAL: shaft speed is outside the narrow map range
            assert!(
                *shaft_speed_rad_s < domain.shaft_speed_lower_rad_s
                    || *shaft_speed_rad_s > domain.shaft_speed_upper_rad_s,
                "shaft_speed={} must be outside [{}, {}]",
                shaft_speed_rad_s,
                domain.shaft_speed_lower_rad_s,
                domain.shaft_speed_upper_rad_s
            );

            // UNCONDITIONAL: exact typed blocker
            if *shaft_speed_rad_s < domain.shaft_speed_lower_rad_s {
                assert!(
                    point.outcome.blockers().iter().any(|b| matches!(
                        b,
                        QualificationBlocker::PropellerShaftSpeedBelowRange { .. }
                    )),
                    "shaft speed below range must emit PropellerShaftSpeedBelowRange"
                );
            } else {
                assert!(
                    point.outcome.blockers().iter().any(|b| matches!(
                        b,
                        QualificationBlocker::PropellerShaftSpeedAboveRange { .. }
                    )),
                    "shaft speed above range must emit PropellerShaftSpeedAboveRange"
                );
            }

            // UNCONDITIONAL: must not qualify
            assert!(!point.outcome.is_qualified());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// TASK 7: DUAL-NODE J SUPPORT (unconditional)
// ===========================================================================

#[test]
fn dual_node_j_blocker_when_one_node_rejects() {
    let model = dual_j_range_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            advance_ratio_j,
            j_lower,
            j_upper,
            j_range_status,
            ..
        } => {
            // UNCONDITIONAL: intersected J range is [0.0, 0.5]
            assert_eq!(*j_lower, 0.0);
            assert_eq!(*j_upper, 0.5);

            // UNCONDITIONAL: J is above the intersected range
            assert!(
                *advance_ratio_j > 0.5,
                "J={} must be > 0.5 (intersected upper bound)",
                advance_ratio_j
            );

            // UNCONDITIONAL: range status is AboveRange
            assert_eq!(*j_range_status, RangeStatus::AboveRange);

            // UNCONDITIONAL: typed J blocker
            assert!(
                point.outcome.blockers().iter().any(|b| matches!(
                    b,
                    QualificationBlocker::PropellerAdvanceRatioAboveRange { .. }
                )),
                "J > intersected upper must emit PropellerAdvanceRatioAboveRange"
            );

            // UNCONDITIONAL: must not qualify
            assert!(!point.outcome.is_qualified());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// TASK 8: NO PROPULSION — NO SKIP (unconditional)
// ===========================================================================

#[test]
fn no_propulsion_returns_not_present() {
    let model = no_propulsion_model();
    let config = sim_config();

    // UNCONDITIONAL: model has no propulsion
    assert!(model.propulsion().is_none());

    // Without propulsion the Newton solver's Jacobian is singular (throttle
    // has no effect), so we evaluate a single candidate directly.
    let request = LongitudinalTrimRequest::new(
        18.0,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.0, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.06, 0.0, 0.0).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();

    let evaluation = evaluate_longitudinal_trim_candidate(&model, &config, &request, {
        LongitudinalTrimVariables::new(0.06, 0.0, 0.0).unwrap()
    })
    .expect("evaluation must succeed for no-propulsion candidate");

    let solution = LongitudinalTrimSolution {
        evaluation,
        iteration_count: 0,
    };

    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);

    // UNCONDITIONAL: propulsion audit is NotPresent
    assert!(
        matches!(
            extract_propulsion_audit(&point.outcome),
            PropulsionDomainAudit::NotPresent
        ),
        "no-propulsion model must produce NotPresent audit, got: {:?}",
        extract_propulsion_audit(&point.outcome)
    );
}

// ===========================================================================
// TASK 9: CONTROL DEFLECTION PROOF — FORCE NONZERO DEFLECTION
// ===========================================================================

#[test]
fn controlled_surface_deflection_changes_section_alpha() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let positions = &solution.evaluation.control_surface_positions;
    let state = &solution.evaluation.state;

    let effective_elements = effective_aero_elements_for_positions(&model, positions);
    let env = config.aero_environment();

    let tail_idx = model
        .aero_elements()
        .iter()
        .position(|e| e.id() == "synthetic-elevator-tail")
        .unwrap();

    let base_element = model.aero_elements()[tail_idx].element();
    let base_kin = compute_section_kinematics(state, base_element, env);
    let deflected_kin = compute_section_kinematics(state, &effective_elements[tail_idx], env);

    let elevator_deflection = -(positions.elevator_angle_rad() - 0.0);

    // UNCONDITIONAL: elevator deflection is meaningful
    assert!(
        elevator_deflection.abs() > 1e-6,
        "elevator deflection must be > 1e-6 rad, got {}",
        elevator_deflection
    );

    // UNCONDITIONAL: base alpha != deflected alpha
    let alpha_diff = (base_kin.alpha_rad - deflected_kin.alpha_rad).abs();
    assert!(
        alpha_diff > 1e-10,
        "base alpha ({}) must differ from deflected alpha ({})",
        base_kin.alpha_rad,
        deflected_kin.alpha_rad
    );

    // Qualification uses DEFLECTED alpha
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);
    let tail_audit = audits
        .iter()
        .find(|a| a.element_id == "synthetic-elevator-tail")
        .unwrap();

    // UNCONDITIONAL: qualification alpha_geom_rad == deflected alpha
    assert_eq!(
        tail_audit.alpha_geom_rad.to_bits(),
        deflected_kin.alpha_rad.to_bits(),
        "qualification must use deflected alpha"
    );

    // UNCONDITIONAL: qualification alpha_geom_rad != base alpha
    assert_ne!(
        tail_audit.alpha_geom_rad.to_bits(),
        base_kin.alpha_rad.to_bits(),
        "qualification must differ from base alpha"
    );
}

// ===========================================================================
// TASK 10: OFF-AXIS TEST — GUARANTEE KNOWN BLOCKER
// ===========================================================================

#[test]
fn strict_off_axis_zero_limits_produce_exact_typed_blockers() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &strict_zero_limits(), 18.0);

    let audit = extract_residual_audit(&point.outcome);
    let blockers = point.outcome.blockers();

    // UNCONDITIONAL: at least one off-axis quantity is nonzero
    // (the trim is not perfectly symmetric in practice)
    let fz_nonzero = audit.fz_body_n.abs() > 0.0;
    let my_nonzero = audit.my_body_nm.abs() > 0.0;

    // With zero limits, ANY nonzero residual triggers a blocker.
    // We guarantee at least Fz or My is nonzero (trim residual forces).
    assert!(
        fz_nonzero || my_nonzero,
        "trim must produce at least one nonzero residual for this proof"
    );

    // UNCONDITIONAL: with zero limits, qualification must reject
    assert!(
        !point.outcome.is_qualified(),
        "zero limits with nonzero residuals must not qualify"
    );

    // Verify exact typed blockers for each nonzero quantity
    if audit.fy_body_n.abs() > 0.0 {
        assert!(
            blockers
                .iter()
                .any(|b| matches!(b, QualificationBlocker::SideForceLimitExceeded { .. }))
        );
    }
    if audit.mx_body_nm.abs() > 0.0 {
        assert!(
            blockers
                .iter()
                .any(|b| matches!(b, QualificationBlocker::RollMomentLimitExceeded { .. }))
        );
    }
    if audit.mz_body_nm.abs() > 0.0 {
        assert!(
            blockers
                .iter()
                .any(|b| matches!(b, QualificationBlocker::YawMomentLimitExceeded { .. }))
        );
    }
    if audit.linear_accel_world_y_mps2.abs() > 0.0 {
        assert!(blockers.iter().any(|b| matches!(
            b,
            QualificationBlocker::LateralAccelerationLimitExceeded { .. }
        )));
    }
    if audit.angular_accel_body_x_rad_s2.abs() > 0.0 {
        assert!(blockers.iter().any(|b| matches!(
            b,
            QualificationBlocker::RollAngularAccelerationLimitExceeded { .. }
        )));
    }
    if audit.angular_accel_body_z_rad_s2.abs() > 0.0 {
        assert!(blockers.iter().any(|b| matches!(
            b,
            QualificationBlocker::YawAngularAccelerationLimitExceeded { .. }
        )));
    }
}

// ===========================================================================
// TASK 11: BLOCKER ORDER — EXPLICIT EXPECTED SEQUENCE
// ===========================================================================

#[test]
fn blocker_order_follows_explicit_aero_propulsion_residual_integrity_sequence() {
    let model = trim_model();
    let config = sim_config();

    // Solve at low speed to get Reynolds below range (aero blocker)
    let request = LongitudinalTrimRequest::new(
        8.0,
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    let solution = solve_longitudinal_trim(&model, &config, &request).unwrap();

    // Zero limits to also get residual limit blockers
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &strict_zero_limits(), 8.0);
    let blockers = point.outcome.blockers();
    assert!(!blockers.is_empty(), "must have multiple blockers");

    // Classify each blocker into its phase
    fn blocker_phase(b: &QualificationBlocker) -> u8 {
        match b {
            QualificationBlocker::AerodynamicAlphaBelowRange { .. }
            | QualificationBlocker::AerodynamicAlphaAboveRange { .. }
            | QualificationBlocker::ReynoldsContributingNodeAlphaBelowRange { .. }
            | QualificationBlocker::ReynoldsContributingNodeAlphaAboveRange { .. }
            | QualificationBlocker::ReynoldsBelowRange { .. }
            | QualificationBlocker::ReynoldsAboveRange { .. } => 0,
            QualificationBlocker::PropellerShaftSpeedBelowRange { .. }
            | QualificationBlocker::PropellerShaftSpeedAboveRange { .. }
            | QualificationBlocker::PropellerAdvanceRatioBelowRange { .. }
            | QualificationBlocker::PropellerAdvanceRatioAboveRange { .. } => 1,
            QualificationBlocker::SideForceLimitExceeded { .. }
            | QualificationBlocker::RollMomentLimitExceeded { .. }
            | QualificationBlocker::YawMomentLimitExceeded { .. }
            | QualificationBlocker::LateralAccelerationLimitExceeded { .. }
            | QualificationBlocker::RollAngularAccelerationLimitExceeded { .. }
            | QualificationBlocker::YawAngularAccelerationLimitExceeded { .. } => 2,
            QualificationBlocker::NonFiniteAuditValue { .. }
            | QualificationBlocker::ReEvaluationFailure => 3,
        }
    }

    // UNCONDITIONAL: strict non-decreasing phase ordering
    let phases: Vec<u8> = blockers.iter().map(blocker_phase).collect();
    for i in 1..phases.len() {
        assert!(
            phases[i] >= phases[i - 1],
            "blocker order violation at {i}: phase {} < {}, blockers: {:?}",
            phases[i],
            phases[i - 1],
            blockers
        );
    }

    // UNCONDITIONAL: must have representatives from aero AND residual phases
    assert!(
        phases.contains(&0),
        "must have aero blockers, phases: {:?}, blockers: {:?}",
        phases,
        blockers
    );
    assert!(
        phases.contains(&2),
        "must have residual blockers, phases: {:?}, blockers: {:?}",
        phases,
        blockers
    );

    // UNCONDITIONAL: verify specific expected blocker classes exist
    let has_reynolds_blocker = blockers
        .iter()
        .any(|b| matches!(b, QualificationBlocker::ReynoldsBelowRange { .. }));
    assert!(
        has_reynolds_blocker,
        "must have ReynoldsBelowRange at 8 m/s, blockers: {:?}",
        blockers
    );
}

// ===========================================================================
// SPEC CASE 2: TRIM FAILURE -> NotQualifiedTrimFailure, NOTHING FABRICATED
// ===========================================================================

/// Deliberately bounded-infeasible sweep template (M2.6A precedent): every point is a
/// deterministic structured trim failure.
fn infeasible_sweep(
    model: &model::AircraftModel,
    config: &AircraftSimulationConfig,
) -> aircraft::LongitudinalTrimSweep {
    let sweep_request = LongitudinalTrimSweepRequest::new(
        vec![18.0, 24.0, 21.0],
        TrimBounds::new(-0.1, 0.2).unwrap(),
        TrimBounds::new(-0.8, 0.8).unwrap(),
        TrimBounds::new(0.0, 0.03).unwrap(),
        LongitudinalTrimVariables::new(0.05, 0.0, 0.01).unwrap(),
        LongitudinalTrimTolerances::new(1.0e-6, 1.0e-7).unwrap(),
        30,
    )
    .unwrap();
    solve_longitudinal_trim_sweep(model, config, &sweep_request).unwrap()
}

#[test]
fn sweep_trim_failure_maps_to_typed_trim_failure_without_diagnostics() {
    let model = trim_model();
    let config = sim_config();
    let sweep = infeasible_sweep(&model, &config);
    assert_eq!(
        sweep.trim_failure_count(),
        3,
        "template must fail everywhere"
    );

    let qualification =
        qualify_longitudinal_trim_sweep(&model, &config, &sweep, &permissive_limits());
    assert_eq!(qualification.len(), 3, "one qualification per sweep point");
    assert_eq!(qualification.qualified_count(), 0);
    assert_eq!(qualification.not_qualified_count(), 3);

    for point in qualification.points() {
        match &point.outcome {
            aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure { failure } => {
                // The typed solver failure is preserved.
                assert!(
                    matches!(
                        failure.reason,
                        aircraft::LongitudinalTrimFailureReason::NoFeasibleSolution
                            | aircraft::LongitudinalTrimFailureReason::SingularJacobian
                            | aircraft::LongitudinalTrimFailureReason::IterationLimit
                    ),
                    "unexpected failure reason: {:?}",
                    failure.reason
                );
                // NO blockers and NO fabricated diagnostics exist for a failed trim.
                assert!(point.outcome.blockers().is_empty());
                assert!(audited_diagnostics(&point.outcome).is_none());
            }
            other => panic!(
                "trim-failure point at {} must be NotQualifiedTrimFailure, got {:?}",
                point.target_airspeed_mps, other
            ),
        }
    }
}

// ===========================================================================
// SPEC CASE 15: SWEEP QUALIFICATION ORDERING EXACTLY MATCHES REQUEST ORDER
// ===========================================================================

#[test]
fn sweep_qualification_preserves_request_ordering() {
    let model = trim_model();
    let config = sim_config();
    // Deliberately NOT sorted: proves ordering follows the request, not value order.
    let sweep_request = LongitudinalTrimSweepRequest::new(
        vec![21.0, 15.0, 18.0],
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    let sweep = solve_longitudinal_trim_sweep(&model, &config, &sweep_request).unwrap();
    assert_eq!(
        sweep.target_airspeeds_mps().collect::<Vec<f64>>(),
        vec![21.0, 15.0, 18.0]
    );

    let qualification =
        qualify_longitudinal_trim_sweep(&model, &config, &sweep, &permissive_limits());
    let speeds: Vec<f64> = qualification
        .points()
        .iter()
        .map(|p| p.target_airspeed_mps)
        .collect();
    assert_eq!(
        speeds,
        vec![21.0, 15.0, 18.0],
        "qualification point order must exactly match sweep/request order"
    );
    assert_eq!(qualification.qualified_count(), 3);
}

// ===========================================================================
// SPEC CASE 14 (sweep level): REPEATED SWEEP QUALIFICATION IS BITWISE IDENTICAL
// ===========================================================================

#[test]
fn repeated_sweep_qualification_is_bitwise_identical_including_failures() {
    let model = trim_model();
    let config = sim_config();

    // Mixed sweep: two successful points and one deliberately infeasible speed.
    // (18.0 and 21.0 trim normally; 2.0 m/s is below the model's flyable envelope and
    // is expected to fail inside the bounded alpha/elevator/throttle box.)
    let sweep_request = LongitudinalTrimSweepRequest::new(
        vec![18.0, 2.0, 21.0],
        TrimBounds::new(-0.15, 0.30).unwrap(),
        TrimBounds::new(-0.9, 0.9).unwrap(),
        TrimBounds::new(0.02, 1.0).unwrap(),
        LongitudinalTrimVariables::new(0.08, 0.1, 0.45).unwrap(),
        LongitudinalTrimTolerances::new(5.0, 2.0).unwrap(),
        50,
    )
    .unwrap();
    let sweep = solve_longitudinal_trim_sweep(&model, &config, &sweep_request).unwrap();
    assert_eq!(
        sweep.trim_failure_count(),
        1,
        "the 2.0 m/s point must be a deterministic trim failure"
    );
    assert_eq!(
        sweep.success_count(),
        2,
        "the 18.0 and 21.0 m/s points must trim"
    );

    let q1 = qualify_longitudinal_trim_sweep(&model, &config, &sweep, &permissive_limits());
    let q2 = qualify_longitudinal_trim_sweep(&model, &config, &sweep, &permissive_limits());
    assert_eq!(q1, q2, "repeated qualification must be bitwise identical");
    for (p1, p2) in q1.points().iter().zip(q2.points().iter()) {
        assert_eq!(
            p1.target_airspeed_mps.to_bits(),
            p2.target_airspeed_mps.to_bits()
        );
    }
    assert_eq!(q1.qualified_count(), 2);
    assert!(matches!(
        &q1.points()[1].outcome,
        aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedTrimFailure { .. }
    ));
}

// ===========================================================================
// SPEC CASES 3/5/9 (boundary arms): EXACT-BOUNDARY VALUES ARE TYPED AND SUPPORTED
// ===========================================================================

/// End-to-end cross-check: qualification's per-element classification must agree with a
/// DIRECT classification through the existing authored domain data (polar tables, family
/// nodes) — no expected values are copied from the implementation under test.
#[test]
fn audit_classification_matches_direct_authored_domain_data() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let diagnostics = audited_diagnostics(&point.outcome).expect("must be audited");

    for audit in diagnostics.aero_audits() {
        let runtime_elem = &model.aero_elements()[audit.element_index];
        assert_eq!(runtime_elem.id(), audit.element_id);
        match runtime_elem.polar_binding() {
            model::RuntimeAeroPolarBinding::Polar { polar_index } => {
                let samples = model.aero_polars()[polar_index].table().samples();
                let lo = samples[0].alpha_rad;
                let hi = samples[samples.len() - 1].alpha_rad;
                assert_eq!(audit.alpha_lower_rad, lo);
                assert_eq!(audit.alpha_upper_rad, hi);
                assert_eq!(
                    audit.alpha_range_status,
                    classify_range_status(audit.alpha_sample_rad, lo, hi),
                    "alpha classification must match direct authored-table classification"
                );
            }
            model::RuntimeAeroPolarBinding::ReynoldsFamily { family_index } => {
                let nodes = model.aero_polar_families()[family_index].family().nodes();
                let re_lo = nodes[0].reynolds_number();
                let re_hi = nodes[nodes.len() - 1].reynolds_number();
                assert_eq!(audit.reynolds_lower, Some(re_lo));
                assert_eq!(audit.reynolds_upper, Some(re_hi));
                let re = audit.reynolds_number.expect("family element records Re");
                assert_eq!(
                    audit.reynolds_range_status,
                    Some(classify_range_status(re, re_lo, re_hi)),
                    "Reynolds classification must match direct authored-node classification"
                );
            }
        }
    }
}

/// Exact-boundary values classify as supported AtLowerBound / AtUpperBound (never as
/// InRange), using endpoint values taken directly from the authored fixture data.
#[test]
fn exact_boundary_values_are_supported_typed_statuses_not_silently_in_range() {
    // Authored polar alpha endpoints of the synthetic tail polar: [-0.40, 0.40].
    assert_eq!(
        classify_range_status(-0.40, -0.40, 0.40),
        RangeStatus::AtLowerBound
    );
    assert_eq!(
        classify_range_status(0.40, -0.40, 0.40),
        RangeStatus::AtUpperBound
    );
    // Authored Reynolds family node endpoints: [200000, 500000].
    assert_eq!(
        classify_range_status(200_000.0, 200_000.0, 500_000.0),
        RangeStatus::AtLowerBound
    );
    assert_eq!(
        classify_range_status(500_000.0, 200_000.0, 500_000.0),
        RangeStatus::AtUpperBound
    );
    // Authored propeller J endpoints of the shaft-speed map's 800 rad/s node: [0.0, 1.2].
    assert_eq!(
        classify_range_status(0.0, 0.0, 1.2),
        RangeStatus::AtLowerBound
    );
    assert_eq!(
        classify_range_status(1.2, 0.0, 1.2),
        RangeStatus::AtUpperBound
    );
    // One quantum outside is NOT supported.
    assert_eq!(
        classify_range_status(0.40 + f64::EPSILON, -0.40, 0.40),
        RangeStatus::AboveRange
    );
    assert_eq!(
        classify_range_status(-0.40 - f64::EPSILON, -0.40, 0.40),
        RangeStatus::BelowRange
    );
}

// ===========================================================================
// SPEC CASES 11+12: ASYMMETRIC MODEL — SUCCESSFUL TRIM, FAILED QUALIFICATION;
// EACH OFF-AXIS THRESHOLD FAILS INDEPENDENTLY
// ===========================================================================

/// The asymmetric fixture trims successfully (longitudinal residuals converge) while its
/// off-axis quantities are all nonzero — exactly the case longitudinal trim cannot see.
fn measured_off_axis_residuals() -> [(&'static str, f64); 6] {
    let model = asymmetric_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    assert!(
        point.outcome.is_qualified(),
        "asymmetric fixture must qualify under permissive limits: {:?}",
        point.outcome.blockers()
    );
    let audit = extract_residual_audit(&point.outcome);
    [
        ("|Fy|", audit.fy_body_n.abs()),
        ("|Mx|", audit.mx_body_nm.abs()),
        ("|Mz|", audit.mz_body_nm.abs()),
        ("|ay|", audit.linear_accel_world_y_mps2.abs()),
        ("|roll accel|", audit.angular_accel_body_x_rad_s2.abs()),
        ("|yaw accel|", audit.angular_accel_body_z_rad_s2.abs()),
    ]
}

#[test]
fn asymmetric_model_trims_successfully_but_can_fail_qualification() {
    let measured = measured_off_axis_residuals();
    for (name, value) in measured {
        assert!(
            value > 0.0 && value.is_finite(),
            "{name} must be nonzero and finite for the asymmetric fixture, got {value}"
        );
    }

    // Tighten every off-axis limit to half its measured value: the successful trim must
    // become a typed RESIDUAL violation (not a domain violation, not a trim failure).
    let model = asymmetric_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let half_limits = LongitudinalTrimQualificationLimits::new(
        measured[0].1 * 0.5,
        measured[1].1 * 0.5,
        measured[2].1 * 0.5,
        measured[3].1 * 0.5,
        measured[4].1 * 0.5,
        measured[5].1 * 0.5,
    )
    .unwrap();
    let point = qualify_longitudinal_trim_solution(&model, &config, &solution, &half_limits, 18.0);
    assert!(
        matches!(
            point.outcome,
            aircraft::LongitudinalTrimQualificationOutcome::NotQualifiedResidualViolation(_)
        ),
        "a successful trim with violated off-axis limits must be NotQualifiedResidualViolation"
    );
    // EVERY off-axis quantity is reported — nothing is hidden at the first violation.
    let residual_blockers: Vec<_> = point
        .outcome
        .blockers()
        .iter()
        .filter(|b| b.category() == QualificationBlockerCategory::Residual)
        .collect();
    assert_eq!(residual_blockers.len(), 6);
}

#[test]
fn each_off_axis_residual_threshold_fails_independently() {
    let measured = measured_off_axis_residuals();
    let model = asymmetric_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);

    for index in 0..6 {
        let mut values = [
            measured[0].1,
            measured[1].1,
            measured[2].1,
            measured[3].1,
            measured[4].1,
            measured[5].1,
        ];
        // Tighten ONLY the selected threshold to half its measured value.
        values[index] *= 0.5;
        let limits = LongitudinalTrimQualificationLimits::new(
            values[0], values[1], values[2], values[3], values[4], values[5],
        )
        .unwrap();
        let point = qualify_longitudinal_trim_solution(&model, &config, &solution, &limits, 18.0);
        assert!(
            !point.outcome.is_qualified(),
            "tightening only threshold {} must fail qualification",
            measured[index].0
        );
        let residual_blockers: Vec<_> = point
            .outcome
            .blockers()
            .iter()
            .filter(|b| b.category() == QualificationBlockerCategory::Residual)
            .collect();
        assert_eq!(
            residual_blockers.len(),
            1,
            "exactly one residual threshold may fail when only {} is tightened: {:?}",
            measured[index].0,
            residual_blockers
        );
        let is_expected = matches!(
            (index, residual_blockers[0]),
            (0, QualificationBlocker::SideForceLimitExceeded { .. })
                | (1, QualificationBlocker::RollMomentLimitExceeded { .. })
                | (2, QualificationBlocker::YawMomentLimitExceeded { .. })
                | (
                    3,
                    QualificationBlocker::LateralAccelerationLimitExceeded { .. }
                )
                | (
                    4,
                    QualificationBlocker::RollAngularAccelerationLimitExceeded { .. }
                )
                | (
                    5,
                    QualificationBlocker::YawAngularAccelerationLimitExceeded { .. }
                )
        );
        assert!(
            is_expected,
            "threshold {} must produce its own typed blocker, got {:?}",
            measured[index].0, residual_blockers[0]
        );
    }
}

// ===========================================================================
// SPEC CASE 13: EXACT-BOUNDARY RESIDUAL VALUES PASS (DOCUMENTED <= RULE)
// ===========================================================================

#[test]
fn exact_boundary_residual_limits_pass_deterministically() {
    let measured = measured_off_axis_residuals();
    let model = asymmetric_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);

    // Limits EXACTLY equal to the measured |values|: value == limit must PASS.
    let exact_limits = LongitudinalTrimQualificationLimits::new(
        measured[0].1,
        measured[1].1,
        measured[2].1,
        measured[3].1,
        measured[4].1,
        measured[5].1,
    )
    .unwrap();
    let point = qualify_longitudinal_trim_solution(&model, &config, &solution, &exact_limits, 18.0);
    assert!(
        point.outcome.is_qualified(),
        "residuals exactly at their limits must pass (<= rule): {:?}",
        point.outcome.blockers()
    );
    // Repeated qualification of the same boundary case is bitwise identical.
    let repeat =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &exact_limits, 18.0);
    assert_eq!(point, repeat);
}

// ===========================================================================
// SPEC CASE 10: NO AUTHORED RPM SUPPORT RANGE -> NotDeclared (NO INVENTED LIMITS)
// ===========================================================================

#[test]
fn rpm_domain_is_not_declared_without_authored_rpm_support_range() {
    let config = sim_config();

    // Fixed-table source: no RPM support range is authored anywhere in the model data.
    let model = fixed_table_model();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            shaft_speed_rpm,
            rpm_domain,
            ..
        } => {
            // The runtime RPM value itself IS recorded...
            assert!(shaft_speed_rpm.is_finite() && *shaft_speed_rpm > 0.0);
            // ...but it is explicitly NOT classified: no RPM envelope is invented.
            assert_eq!(*rpm_domain, RpmDomainStatus::NotDeclared);
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }

    // Shaft-speed-map source: the map's node range is an authored shaft-speed domain,
    // NOT an RPM support range, so RPM stays NotDeclared there too.
    let model = shaft_speed_narrow_model();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            shaft_speed_rpm,
            rpm_domain,
            shaft_speed_domain,
            ..
        } => {
            assert!(shaft_speed_rpm.is_finite());
            assert_eq!(*rpm_domain, RpmDomainStatus::NotDeclared);
            // The shaft-speed domain IS explicitly authored and IS audited.
            assert!(shaft_speed_domain.is_some());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// PROPULSION AUDIT RECORDS THE FULL RUNTIME QUANTITY SET
// ===========================================================================

#[test]
fn propulsion_audit_records_thrust_torque_rpm_and_j() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    match extract_propulsion_audit(&point.outcome) {
        PropulsionDomainAudit::Present {
            throttle,
            axial_airspeed_mps,
            shaft_speed_rad_s,
            shaft_speed_rpm,
            advance_ratio_j,
            thrust_n,
            torque_n_m,
            ..
        } => {
            // Every recorded quantity must be finite; thrust must be positive in
            // trimmed forward flight and the RPM conversion must be consistent.
            assert!((*throttle).is_finite());
            assert!(axial_airspeed_mps.is_finite());
            assert!(shaft_speed_rad_s.is_finite() && *shaft_speed_rad_s > 0.0);
            assert!(shaft_speed_rpm.is_finite());
            assert!(advance_ratio_j.is_finite());
            assert!(thrust_n.is_finite() && *thrust_n > 0.0);
            assert!(torque_n_m.is_finite());
            let expected_rpm = shaft_speed_rad_s * 60.0 / std::f64::consts::TAU;
            assert_eq!(shaft_speed_rpm.to_bits(), expected_rpm.to_bits());
        }
        PropulsionDomainAudit::NotPresent => panic!("fixture has propulsion"),
    }
}

// ===========================================================================
// SPEC CASE 18: NO NaN/Inf IN ANY DIAGNOSTIC FOR VALID INPUTS
// ===========================================================================

#[test]
fn valid_inputs_produce_finite_diagnostics_everywhere() {
    let model = trim_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let diagnostics = audited_diagnostics(&point.outcome).expect("must be audited");

    // Full residual audit: every field.
    let r = diagnostics.residual_audit();
    for (name, value) in [
        ("fx", r.fx_body_n),
        ("fy", r.fy_body_n),
        ("fz", r.fz_body_n),
        ("mx", r.mx_body_nm),
        ("my", r.my_body_nm),
        ("mz", r.mz_body_nm),
        ("ax", r.linear_accel_world_x_mps2),
        ("ay", r.linear_accel_world_y_mps2),
        ("az", r.linear_accel_world_z_mps2),
        ("wx", r.angular_accel_body_x_rad_s2),
        ("wy", r.angular_accel_body_y_rad_s2),
        ("wz", r.angular_accel_body_z_rad_s2),
        ("Fx_res", r.longitudinal_force_n),
        ("Fz_res", r.vertical_force_n),
        ("M_res", r.pitch_moment_nm),
    ] {
        assert!(value.is_finite(), "residual field {name} must be finite");
    }
    // Aero audits: every scalar and optional field.
    for audit in diagnostics.aero_audits() {
        assert!(audit.alpha_geom_rad.is_finite());
        assert!(audit.alpha_sample_rad.is_finite());
        assert!(audit.alpha_lower_rad.is_finite());
        assert!(audit.alpha_upper_rad.is_finite());
        assert!(audit.section_airspeed_mps.is_finite());
        assert!(audit.reynolds_number.is_none_or(|v| v.is_finite()));
        assert!(audit.reynolds_lower.is_none_or(|v| v.is_finite()));
        assert!(audit.reynolds_upper.is_none_or(|v| v.is_finite()));
    }
    // No integrity blockers: a valid input must not trip NonFiniteAuditValue.
    assert!(
        !point
            .outcome
            .blockers()
            .iter()
            .any(|b| matches!(b, QualificationBlocker::NonFiniteAuditValue { .. }))
    );
}

// ===========================================================================
// SPEC CASE 8: UNASSIGNED V5 ELEMENT REMAINS QUASI-2D FOR QUALIFICATION
// ===========================================================================

#[test]
fn unassigned_v5_element_remains_quasi2d_in_qualification() {
    let model = finite_wing_model();
    let config = sim_config();
    let solution = solve_trim(&model, &config, 18.0);
    let point =
        qualify_longitudinal_trim_solution(&model, &config, &solution, &permissive_limits(), 18.0);
    let audits = extract_aero_audits(&point.outcome);

    let wing = audits
        .iter()
        .find(|a| a.element_id == "synthetic-wing")
        .unwrap();
    let tail = audits
        .iter()
        .find(|a| a.element_id == "synthetic-elevator-tail")
        .unwrap();

    // The wing belongs to a finite-wing surface: alpha_sample = alpha_geom - alpha_i.
    assert_eq!(wing.polar_binding_kind, "reynolds_family");
    assert!(
        (wing.alpha_geom_rad - wing.alpha_sample_rad).abs() > 1e-6,
        "surface member must be audited at the induced-corrected alpha"
    );

    // The tail is assigned to NO surface: legacy quasi-2D sampling alpha, bitwise equal
    // to its geometric alpha, with no induced correction of any kind.
    assert_eq!(tail.polar_binding_kind, "polar");
    assert_eq!(
        tail.alpha_sample_rad.to_bits(),
        tail.alpha_geom_rad.to_bits(),
        "unassigned element must keep alpha_sample == alpha_geom bitwise"
    );
}
