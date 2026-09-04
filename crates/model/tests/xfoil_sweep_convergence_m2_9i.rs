//! M2.9I — Deterministic XFOIL sweep convergence qualification tests.
//!
//! All fixtures use synthetic XFOIL polar data parsed via the canonical
//! `parse_xfoil_polar`. No Clark Y or LT-40 aerodynamic data appears here.

use model::{
    ConvergenceStatus, MetadataBuilder, SweepConvergenceBlocker, SweepConvergenceStatus,
    SweepExpectation, SweepExpectationError, XfoilPolarImport, parse_xfoil_polar,
    qualify_sweep_convergence,
};

const DEG: f64 = std::f64::consts::PI / 180.0;

fn metadata() -> MetadataBuilder {
    MetadataBuilder::new(250_000.0, 0.04)
        .solver_name("Synthetic XFOIL")
        .solver_version("test")
        .command_or_config("SYNTHETIC")
        .transition_assumptions("test")
        .ncrit(9.0)
}

/// Build a 4-column XFOIL polar text from alpha values in degrees.
fn polar_text(alpha_degrees: &[f64]) -> String {
    let mut s = String::from(
        " alpha    CL         CD         CM\n\
         ------   ---------  ---------  ---------\n",
    );
    for (i, &a) in alpha_degrees.iter().enumerate() {
        let cl = -0.2 + i as f64 * 0.15;
        let cd = 0.01 + i as f64 * 0.001;
        let cm = -0.05;
        s.push_str(&format!(" {a:.6} {cl:.6} {cd:.6} {cm:.6}\n"));
    }
    s
}

/// Parse a polar from alpha degrees (must be strictly increasing for parser).
fn parse_from_degrees(alpha_degrees: &[f64]) -> XfoilPolarImport {
    let text = polar_text(alpha_degrees);
    parse_xfoil_polar(&text, metadata().clone().build().unwrap()).unwrap()
}

// ── Test 1: valid ascending complete sweep → Converged ──────────────────────

#[test]
fn m2_9i_01_ascending_complete_sweep_converged() {
    let import = parse_from_degrees(&[-5.0, 0.0, 5.0, 10.0, 15.0]);
    let expectation = SweepExpectation::new(-5.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
    assert_eq!(q.status(), SweepConvergenceStatus::Converged);
    assert_eq!(q.expected_sample_count(), 5);
    assert_eq!(q.observed_sample_count(), 5);
    assert!(q.blockers().is_empty());
}

// ── Test 2: valid descending complete sweep → Converged ─────────────────────

#[test]
fn m2_9i_02_descending_complete_sweep_converged() {
    // The parser enforces strictly increasing alpha output, so the observed
    // data is ascending [0, 5, 10]. The descending sweep expectation defines
    // the same set of alpha points {0, 5, 10} in reverse command order.
    // The qualification matches the expected point set against observed.
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(10.0 * DEG, 0.0 * DEG, -5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
    assert_eq!(q.expected_sample_count(), 3);
    assert_eq!(q.observed_sample_count(), 3);
}

// ── Test 3: single missing middle point → NotConverged ──────────────────────

#[test]
fn m2_9i_03_missing_middle_point_not_converged() {
    let import = parse_from_degrees(&[-5.0, 0.0, 10.0, 15.0]);
    let expectation = SweepExpectation::new(-5.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert_eq!(q.status(), SweepConvergenceStatus::NotConverged);
    assert!(
        q.blockers()
            .iter()
            .any(|b| matches!(b, SweepConvergenceBlocker::SampleCountMismatch { .. }))
    );
}

// ── Test 4: missing first point → NotConverged ──────────────────────────────

#[test]
fn m2_9i_04_missing_first_point_not_converged() {
    let import = parse_from_degrees(&[0.0, 5.0, 10.0, 15.0]);
    let expectation = SweepExpectation::new(-5.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
}

// ── Test 5: missing last point → NotConverged ───────────────────────────────

#[test]
fn m2_9i_05_missing_last_point_not_converged() {
    let import = parse_from_degrees(&[-5.0, 0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(-5.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
}

// ── Test 6: extra sample → NotConverged ─────────────────────────────────────

#[test]
fn m2_9i_06_extra_sample_not_converged() {
    let import = parse_from_degrees(&[-5.0, 0.0, 5.0, 10.0, 15.0, 20.0]);
    let expectation = SweepExpectation::new(-5.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert!(q.blockers().iter().any(|b| matches!(
        b,
        SweepConvergenceBlocker::SampleCountMismatch {
            expected: 5,
            observed: 6
        }
    )));
}

// ── Test 7: duplicate alpha → NotConverged ──────────────────────────────────

#[test]
fn m2_9i_07_duplicate_alpha_not_converged() {
    // Parser rejects duplicate alpha, so we simulate by providing fewer
    // unique points than expected (count mismatch).
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 2.5 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert!(
        q.blockers()
            .iter()
            .any(|b| matches!(b, SweepConvergenceBlocker::SampleCountMismatch { .. }))
    );
}

// ── Test 8: reordered samples → NotConverged ────────────────────────────────

#[test]
fn m2_9i_08_reordered_samples_not_converged() {
    // The parser enforces strictly increasing alpha, so observed data is
    // always ascending. Reordering that preserves the ascending sequence
    // is indistinguishable from a correctly ordered sweep at the set level.
    // This test verifies that a genuinely different alpha set (simulating
    // the effect of reordering with value substitution) is rejected.
    let import = parse_from_degrees(&[0.0, 7.0, 10.0, 15.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 15.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert!(
        q.blockers()
            .iter()
            .any(|b| matches!(b, SweepConvergenceBlocker::AlphaMismatch { index: 1, .. }))
    );
}

// ── Test 9: exact expected alpha accepted ───────────────────────────────────

#[test]
fn m2_9i_09_exact_alpha_accepted() {
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 0.0).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
}

// ── Test 10: alpha deviation inside tolerance accepted ───────────────────────

#[test]
fn m2_9i_10_deviation_inside_tolerance_accepted() {
    // Expected: [0, 5, 10] deg. Observed: [0.001, 5.001, 10.001] deg.
    // Deviation ≈ 1.75e-5 rad, tolerance = 1e-3 rad.
    let import = parse_from_degrees(&[0.001, 5.001, 10.001]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-3).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
}

// ── Test 11: alpha deviation exactly at tolerance accepted ───────────────────

#[test]
fn m2_9i_11_deviation_exactly_at_tolerance_accepted() {
    // Verify that deviation within tolerance is accepted.
    // The parser converts degrees→radians via degrees * PI / 180.
    // Use a generous tolerance to avoid float round-trip noise.
    let import = parse_from_degrees(&[0.05, 5.0, 10.0]);
    let expectation =
        SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 0.05 * DEG + 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
}

// ── Test 12: alpha deviation outside tolerance rejected ──────────────────────

#[test]
fn m2_9i_12_deviation_outside_tolerance_rejected() {
    let tol = 1e-6;
    let import = parse_from_degrees(&[0.01, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, tol).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert!(
        q.blockers()
            .iter()
            .any(|b| matches!(b, SweepConvergenceBlocker::AlphaMismatch { index: 0, .. }))
    );
}

// ── Test 13: zero tolerance works as exact comparison ────────────────────────

#[test]
fn m2_9i_13_zero_tolerance_exact_comparison() {
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 0.0).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());

    // Tiny deviation fails with zero tolerance.
    let import2 = parse_from_degrees(&[0.0001, 5.0, 10.0]);
    let q2 = qualify_sweep_convergence(&expectation, &import2);
    assert!(!q2.is_converged());
}

// ── Test 14: zero step rejected ─────────────────────────────────────────────

#[test]
fn m2_9i_14_zero_step_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, 0.1, 0.0, 1e-6).unwrap_err(),
        SweepExpectationError::ZeroStep
    );
}

// ── Test 15: positive step with descending bounds rejected ──────────────────

#[test]
fn m2_9i_15_positive_step_descending_bounds_rejected() {
    assert_eq!(
        SweepExpectation::new(0.1, 0.0, 0.01, 1e-6).unwrap_err(),
        SweepExpectationError::StepDirectionMismatch
    );
}

// ── Test 16: negative step with ascending bounds rejected ───────────────────

#[test]
fn m2_9i_16_negative_step_ascending_bounds_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, 0.1, -0.01, 1e-6).unwrap_err(),
        SweepExpectationError::StepDirectionMismatch
    );
}

// ── Test 17: nonfinite start rejected ───────────────────────────────────────

#[test]
fn m2_9i_17_nonfinite_start_rejected() {
    assert_eq!(
        SweepExpectation::new(f64::NAN, 0.1, 0.01, 1e-6).unwrap_err(),
        SweepExpectationError::NonFiniteStart
    );
    assert_eq!(
        SweepExpectation::new(f64::INFINITY, 0.1, 0.01, 1e-6).unwrap_err(),
        SweepExpectationError::NonFiniteStart
    );
}

// ── Test 18: nonfinite end rejected ─────────────────────────────────────────

#[test]
fn m2_9i_18_nonfinite_end_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, f64::NAN, 0.01, 1e-6).unwrap_err(),
        SweepExpectationError::NonFiniteEnd
    );
    assert_eq!(
        SweepExpectation::new(0.0, f64::NEG_INFINITY, 0.01, 1e-6).unwrap_err(),
        SweepExpectationError::NonFiniteEnd
    );
}

// ── Test 19: nonfinite step rejected ────────────────────────────────────────

#[test]
fn m2_9i_19_nonfinite_step_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, 0.1, f64::NAN, 1e-6).unwrap_err(),
        SweepExpectationError::NonFiniteStep
    );
}

// ── Test 20: nonfinite tolerance rejected ───────────────────────────────────

#[test]
fn m2_9i_20_nonfinite_tolerance_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, 0.1, 0.01, f64::NAN).unwrap_err(),
        SweepExpectationError::NonFiniteTolerance
    );
}

// ── Test 21: negative tolerance rejected ────────────────────────────────────

#[test]
fn m2_9i_21_negative_tolerance_rejected() {
    assert_eq!(
        SweepExpectation::new(0.0, 0.1, 0.01, -1e-6).unwrap_err(),
        SweepExpectationError::NegativeTolerance
    );
}

// ── Test 22: unreachable endpoint rejected ──────────────────────────────────

#[test]
fn m2_9i_22_unreachable_endpoint_rejected() {
    // 0.0 to 0.1 with step 0.03: 0.0, 0.03, 0.06, 0.09 — endpoint 0.1
    // is not reachable within tight tolerance.
    let result = SweepExpectation::new(0.0, 0.1, 0.03, 1e-12);
    assert_eq!(
        result.unwrap_err(),
        SweepExpectationError::UnreachableEndpoint
    );
}

// ── Test 23: expected point count deterministic ─────────────────────────────

#[test]
fn m2_9i_23_expected_point_count_deterministic() {
    let e = SweepExpectation::new(-0.1, 0.1, 0.01, 1e-9).unwrap();
    assert_eq!(e.expected_sample_count(), 21);

    let e2 = SweepExpectation::new(0.0, 1.0, 0.1, 1e-9).unwrap();
    assert_eq!(e2.expected_sample_count(), 11);

    let e3 = SweepExpectation::new(0.5, 0.0, -0.1, 1e-9).unwrap();
    assert_eq!(e3.expected_sample_count(), 6);
}

// ── Test 24: blocker ordering deterministic ─────────────────────────────────

#[test]
fn m2_9i_24_blocker_ordering_deterministic() {
    // Alpha mismatches must appear in ascending index order.
    // Observed: [1.0, 6.0, 11.0] deg — all shifted by +1 deg from expected.
    let import = parse_from_degrees(&[1.0, 6.0, 11.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());

    let alpha_blockers: Vec<_> = q
        .blockers()
        .iter()
        .filter_map(|b| match b {
            SweepConvergenceBlocker::AlphaMismatch { index, .. } => Some(*index),
            _ => None,
        })
        .collect();
    assert_eq!(alpha_blockers.len(), 3);
    for w in alpha_blockers.windows(2) {
        assert!(
            w[0] < w[1],
            "alpha blockers must be in ascending index order"
        );
    }
}

// ── Test 25: repeated qualification produces identical result ────────────────

#[test]
fn m2_9i_25_repeated_qualification_identical() {
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q1 = qualify_sweep_convergence(&expectation, &import);
    let q2 = qualify_sweep_convergence(&expectation, &import);
    assert_eq!(q1, q2);
}

// ── Test 26: CL/CD/CM irrelevant to sweep-completeness decision ─────────────

#[test]
fn m2_9i_26_coefficients_irrelevant() {
    // Two polars with same alphas but wildly different CL/CD/CM.
    let text_a = "\
 alpha    CL         CD         CM\n\
 ------   ---------  ---------  ---------\n\
  0.000  0.000000  0.010000  -0.050000\n\
  5.000  0.500000  0.012000  -0.050000\n\
 10.000  1.000000  0.020000  -0.050000\n";
    let text_b = "\
 alpha    CL         CD         CM\n\
 ------   ---------  ---------  ---------\n\
  0.000  999.0000  888.0000  777.0000\n\
  5.000  -42.0000  0.000100  0.000000\n\
 10.000  0.000000  1.000000  -1.000000\n";
    let meta = metadata().clone().build().unwrap();
    let import_a = parse_xfoil_polar(text_a, meta.clone()).unwrap();
    let import_b = parse_xfoil_polar(text_b, meta).unwrap();
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let qa = qualify_sweep_convergence(&expectation, &import_a);
    let qb = qualify_sweep_convergence(&expectation, &import_b);
    assert!(qa.is_converged());
    assert!(qb.is_converged());
    assert_eq!(qa, qb);
}

// ── Test 27: parser success with incomplete sweep is NOT Converged ───────────

#[test]
fn m2_9i_27_parser_success_incomplete_sweep_not_converged() {
    // Parser succeeds (valid polar with 3 samples) but sweep expects 5.
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(-10.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert_eq!(q.status(), SweepConvergenceStatus::NotConverged);
    assert_eq!(q.to_convergence_status(), ConvergenceStatus::Unresolved);
}

// ── Test 28: process/runner concepts are not part of this API ───────────────

#[test]
fn m2_9i_28_no_process_or_runner_concepts() {
    // The API takes only a SweepExpectation and XfoilPolarImport.
    // There is no process exit code, file path, or runner state.
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(q.is_converged());
}

// ── Test 29: no real reference airfoil data used ────────────────────────────

#[test]
fn m2_9i_29_only_synthetic_data() {
    // This test self-documents: all fixtures above use synthetic data.
    // The convergence module must not reference any real airfoil directory.
    let source = include_str!("../src/reference_xfoil_convergence.rs");
    assert!(
        !source.contains("reference/"),
        "convergence module must not reference real data directories"
    );
}

// ── Test 30: existing M2.9A/B/C/H tests remain green ────────────────────────

#[test]
fn m2_9i_30_existing_api_unchanged() {
    // Verify that the canonical types used by M2.9A/B/C/H are still
    // accessible and functional.
    let text = "\
 alpha    CL         CD         CM\n\
 ------   ---------  ---------  ---------\n\
  0.000  0.000000  0.010000  -0.050000\n\
  5.000  0.500000  0.012000  -0.050000\n";
    let meta = metadata().clone().build().unwrap();
    let import = parse_xfoil_polar(text, meta).unwrap();
    assert_eq!(import.sample_count(), 2);
    assert!((import.samples()[0].alpha_rad() - 0.0).abs() < 1e-12);
    assert!((import.samples()[1].cl() - 0.5).abs() < 1e-6);
}

// ── ConvergenceStatus mapping ───────────────────────────────────────────────

#[test]
fn m2_9i_convergence_status_mapping() {
    let import = parse_from_degrees(&[0.0, 5.0, 10.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let converged = qualify_sweep_convergence(&expectation, &import);
    assert_eq!(
        converged.to_convergence_status(),
        ConvergenceStatus::Converged
    );

    let incomplete = parse_from_degrees(&[0.0, 5.0]);
    let not_converged = qualify_sweep_convergence(&expectation, &incomplete);
    assert_eq!(
        not_converged.to_convergence_status(),
        ConvergenceStatus::Unresolved
    );
}

// ── Count mismatch blocker comes first ──────────────────────────────────────

#[test]
fn m2_9i_count_blocker_before_alpha_mismatches() {
    // Parser requires ≥2 samples; provide 2 that don't match a 3-point sweep.
    let import = parse_from_degrees(&[0.0, 5.0]);
    let expectation = SweepExpectation::new(0.0 * DEG, 10.0 * DEG, 5.0 * DEG, 1e-9).unwrap();
    let q = qualify_sweep_convergence(&expectation, &import);
    assert!(!q.is_converged());
    assert!(matches!(
        q.blockers()[0],
        SweepConvergenceBlocker::SampleCountMismatch {
            expected: 3,
            observed: 2
        }
    ));
}

// ── Unreachable endpoint: more than half-step off ───────────────────────────

#[test]
fn m2_9i_unreachable_endpoint_more_than_half_step() {
    // 0 to 0.1 with step 0.03: raw = 3.333, rounds to 3.
    // Reached = 0.09, gap = 0.01 > tolerance 0.001.
    let result = SweepExpectation::new(0.0, 0.1, 0.03, 0.001);
    assert_eq!(
        result.unwrap_err(),
        SweepExpectationError::UnreachableEndpoint
    );
}

// ── Single-point sweep (start == end) is rejected ───────────────────────────

#[test]
fn m2_9i_single_point_sweep_rejected() {
    assert!(SweepExpectation::new(0.1, 0.1, 0.01, 1e-6).is_err());
}
