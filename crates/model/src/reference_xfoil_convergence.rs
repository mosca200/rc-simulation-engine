//! M2.9I — Deterministic XFOIL sweep convergence qualification.
//!
//! This module answers: "Did this parsed XFOIL polar actually contain every
//! requested alpha point of the commanded sweep?"
//!
//! It operates on the canonical [`XfoilPolarImport`] produced by
//! [`parse_xfoil_polar`](crate::parse_xfoil_polar). It does NOT parse XFOIL
//! text, execute XFOIL, modify aircraft runtime physics, or alter
//! aerodynamic coefficients.
//!
//! # Convergence definition
//!
//! Complete sweep convergence means every commanded alpha point produced a
//! parseable polar row. It does **not** prove experimental accuracy, airfoil
//! applicability to an aircraft, 3D finite-wing accuracy, stall fidelity
//! beyond XFOIL, transition-model correctness, or LT-40 suitability.
//!
//! # Runtime safety
//!
//! This module is off-runtime evidence processing. It does not participate in
//! the simulation hot path.

use crate::reference_aerodynamics::ConvergenceStatus;
use crate::reference_xfoil::XfoilPolarImport;

/// A validated alpha sweep expectation.
///
/// Defines the exact sequence of alpha points that an XFOIL solver run was
/// commanded to produce. The inclusive sequence is:
///
/// ```text
/// alpha_start, alpha_start + step, alpha_start + 2*step, ..., alpha_end
/// ```
///
/// The endpoint must be reachable by an integral number of steps within the
/// explicit tolerance.
#[derive(Debug, Clone)]
pub struct SweepExpectation {
    alpha_start_rad: f64,
    alpha_end_rad: f64,
    alpha_step_rad: f64,
    alpha_match_tolerance_rad: f64,
}

impl SweepExpectation {
    /// Create a new sweep expectation after validating all invariants.
    ///
    /// # Errors
    ///
    /// Returns [`SweepExpectationError`] when any invariant is violated.
    /// The inputs are never silently modified.
    pub fn new(
        alpha_start_rad: f64,
        alpha_end_rad: f64,
        alpha_step_rad: f64,
        alpha_match_tolerance_rad: f64,
    ) -> Result<Self, SweepExpectationError> {
        if !alpha_start_rad.is_finite() {
            return Err(SweepExpectationError::NonFiniteStart);
        }
        if !alpha_end_rad.is_finite() {
            return Err(SweepExpectationError::NonFiniteEnd);
        }
        if !alpha_step_rad.is_finite() {
            return Err(SweepExpectationError::NonFiniteStep);
        }
        if !alpha_match_tolerance_rad.is_finite() {
            return Err(SweepExpectationError::NonFiniteTolerance);
        }
        if alpha_step_rad == 0.0 {
            return Err(SweepExpectationError::ZeroStep);
        }
        if alpha_match_tolerance_rad < 0.0 {
            return Err(SweepExpectationError::NegativeTolerance);
        }
        let range = alpha_end_rad - alpha_start_rad;
        if (range > 0.0 && alpha_step_rad < 0.0) || (range < 0.0 && alpha_step_rad > 0.0) {
            return Err(SweepExpectationError::StepDirectionMismatch);
        }
        if range == 0.0 {
            return Err(SweepExpectationError::ZeroStep);
        }

        let raw_count_f = range / alpha_step_rad;
        let n_steps = raw_count_f.round();
        if n_steps < 1.0 {
            return Err(SweepExpectationError::UnreachableEndpoint);
        }
        let n_steps_i = n_steps as i64;
        let reached = alpha_start_rad + n_steps_i as f64 * alpha_step_rad;
        if (reached - alpha_end_rad).abs() > alpha_match_tolerance_rad {
            return Err(SweepExpectationError::UnreachableEndpoint);
        }

        Ok(Self {
            alpha_start_rad,
            alpha_end_rad,
            alpha_step_rad,
            alpha_match_tolerance_rad,
        })
    }

    /// First requested alpha in radians.
    pub const fn alpha_start_rad(&self) -> f64 {
        self.alpha_start_rad
    }

    /// Last requested alpha in radians.
    pub const fn alpha_end_rad(&self) -> f64 {
        self.alpha_end_rad
    }

    /// Alpha increment per step in radians (signed).
    pub const fn alpha_step_rad(&self) -> f64 {
        self.alpha_step_rad
    }

    /// Match tolerance in radians.
    pub const fn alpha_match_tolerance_rad(&self) -> f64 {
        self.alpha_match_tolerance_rad
    }

    /// Deterministic expected sample count (inclusive of both endpoints).
    pub fn expected_sample_count(&self) -> usize {
        let range = self.alpha_end_rad - self.alpha_start_rad;
        let n_steps = (range / self.alpha_step_rad).round() as i64;
        (n_steps + 1) as usize
    }

    /// Compute the expected alpha at the given index using
    /// `start + index * step` to avoid floating-point accumulation drift.
    pub fn expected_alpha_rad(&self, index: usize) -> f64 {
        self.alpha_start_rad + index as f64 * self.alpha_step_rad
    }
}

/// Errors that can occur when constructing a [`SweepExpectation`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SweepExpectationError {
    NonFiniteStart,
    NonFiniteEnd,
    NonFiniteStep,
    NonFiniteTolerance,
    ZeroStep,
    NegativeTolerance,
    StepDirectionMismatch,
    UnreachableEndpoint,
}

impl std::fmt::Display for SweepExpectationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFiniteStart => f.write_str("alpha_start_rad is not finite"),
            Self::NonFiniteEnd => f.write_str("alpha_end_rad is not finite"),
            Self::NonFiniteStep => f.write_str("alpha_step_rad is not finite"),
            Self::NonFiniteTolerance => f.write_str("alpha_match_tolerance_rad is not finite"),
            Self::ZeroStep => f.write_str("alpha_step_rad must not be zero"),
            Self::NegativeTolerance => {
                f.write_str("alpha_match_tolerance_rad must be non-negative")
            }
            Self::StepDirectionMismatch => {
                f.write_str("step sign does not move from start toward end")
            }
            Self::UnreachableEndpoint => {
                f.write_str("endpoint not reachable by integral number of steps within tolerance")
            }
        }
    }
}

impl std::error::Error for SweepExpectationError {}

/// Result of qualifying an XFOIL polar against a sweep expectation.
#[derive(Debug, Clone, PartialEq)]
pub struct XfoilSweepConvergenceQualification {
    status: SweepConvergenceStatus,
    expected_sample_count: usize,
    observed_sample_count: usize,
    blockers: Vec<SweepConvergenceBlocker>,
}

/// Sweep convergence status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepConvergenceStatus {
    Converged,
    NotConverged,
}

impl XfoilSweepConvergenceQualification {
    /// Whether the sweep is fully converged.
    pub fn is_converged(&self) -> bool {
        self.status == SweepConvergenceStatus::Converged
    }

    /// Sweep convergence status.
    pub const fn status(&self) -> SweepConvergenceStatus {
        self.status
    }

    /// Expected number of samples in the commanded sweep.
    pub const fn expected_sample_count(&self) -> usize {
        self.expected_sample_count
    }

    /// Observed number of samples in the parsed polar.
    pub const fn observed_sample_count(&self) -> usize {
        self.observed_sample_count
    }

    /// Deterministic blocker list (count mismatch first, then alpha
    /// mismatches in ascending index order).
    pub fn blockers(&self) -> &[SweepConvergenceBlocker] {
        &self.blockers
    }

    /// Map to the repository-wide [`ConvergenceStatus`].
    ///
    /// `Converged` → `ConvergenceStatus::Converged`
    /// `NotConverged` → `ConvergenceStatus::Unresolved`
    ///
    /// M2.9I never maps to `ConvergenceStatus::Failed`; it only proves
    /// complete convergence when evidence is sufficient, otherwise remains
    /// fail-closed / unresolved.
    pub fn to_convergence_status(&self) -> ConvergenceStatus {
        match self.status {
            SweepConvergenceStatus::Converged => ConvergenceStatus::Converged,
            SweepConvergenceStatus::NotConverged => ConvergenceStatus::Unresolved,
        }
    }
}

/// Typed deterministic blocker.
#[derive(Debug, Clone, PartialEq)]
pub enum SweepConvergenceBlocker {
    SampleCountMismatch {
        expected: usize,
        observed: usize,
    },
    AlphaMismatch {
        index: usize,
        expected_alpha_rad: f64,
        observed_alpha_rad: f64,
        tolerance_rad: f64,
    },
}

/// Qualify whether a parsed XFOIL polar contains every requested alpha point
/// of the commanded sweep.
///
/// This is the core M2.9I entry point. It does NOT execute XFOIL, does NOT
/// modify aerodynamic coefficients, and does NOT alter runtime physics.
///
/// For descending sweep expectations (negative step), the expected sequence
/// is compared in reverse against the observed data, since XFOIL output
/// ordering depends on the commanded sweep direction.
pub fn qualify_sweep_convergence(
    expectation: &SweepExpectation,
    import: &XfoilPolarImport,
) -> XfoilSweepConvergenceQualification {
    let expected_count = expectation.expected_sample_count();
    let observed_count = import.sample_count();
    let tolerance = expectation.alpha_match_tolerance_rad();

    if observed_count != expected_count {
        let blockers = vec![SweepConvergenceBlocker::SampleCountMismatch {
            expected: expected_count,
            observed: observed_count,
        }];
        return XfoilSweepConvergenceQualification {
            status: SweepConvergenceStatus::NotConverged,
            expected_sample_count: expected_count,
            observed_sample_count: observed_count,
            blockers,
        };
    }

    let forward_blockers = compute_alpha_blockers(expectation, import, tolerance, false);
    if forward_blockers.is_empty() {
        return XfoilSweepConvergenceQualification {
            status: SweepConvergenceStatus::Converged,
            expected_sample_count: expected_count,
            observed_sample_count: observed_count,
            blockers: vec![],
        };
    }

    if expectation.alpha_step_rad() < 0.0 {
        let reverse_blockers = compute_alpha_blockers(expectation, import, tolerance, true);
        if reverse_blockers.is_empty() {
            return XfoilSweepConvergenceQualification {
                status: SweepConvergenceStatus::Converged,
                expected_sample_count: expected_count,
                observed_sample_count: observed_count,
                blockers: vec![],
            };
        }
    }

    XfoilSweepConvergenceQualification {
        status: SweepConvergenceStatus::NotConverged,
        expected_sample_count: expected_count,
        observed_sample_count: observed_count,
        blockers: forward_blockers,
    }
}

fn compute_alpha_blockers(
    expectation: &SweepExpectation,
    import: &XfoilPolarImport,
    tolerance: f64,
    reverse_expected: bool,
) -> Vec<SweepConvergenceBlocker> {
    let count = expectation
        .expected_sample_count()
        .min(import.sample_count());
    let mut blockers = Vec::new();
    for i in 0..count {
        let expected_idx = if reverse_expected {
            expectation.expected_sample_count() - 1 - i
        } else {
            i
        };
        let expected_alpha = expectation.expected_alpha_rad(expected_idx);
        let observed_alpha = import.samples()[i].alpha_rad();
        if (observed_alpha - expected_alpha).abs() > tolerance {
            blockers.push(SweepConvergenceBlocker::AlphaMismatch {
                index: i,
                expected_alpha_rad: expected_alpha,
                observed_alpha_rad: observed_alpha,
                tolerance_rad: tolerance,
            });
        }
    }
    blockers
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expectation_rejects_zero_step() {
        assert_eq!(
            SweepExpectation::new(0.0, 0.1, 0.0, 1e-6).unwrap_err(),
            SweepExpectationError::ZeroStep
        );
    }

    #[test]
    fn expectation_rejects_nan_start() {
        assert_eq!(
            SweepExpectation::new(f64::NAN, 0.1, 0.01, 1e-6).unwrap_err(),
            SweepExpectationError::NonFiniteStart
        );
    }

    #[test]
    fn expectation_count_is_deterministic() {
        let e = SweepExpectation::new(-0.1, 0.1, 0.01, 1e-9).unwrap();
        assert_eq!(e.expected_sample_count(), 21);
    }
}
