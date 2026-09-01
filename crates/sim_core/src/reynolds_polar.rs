//! Generic deterministic sampling across a discrete Reynolds-indexed family of alpha polars.
//!
//! This primitive is intentionally not wired into [`crate::Simulation`] or aircraft-model
//! loading. Each node preserves its own [`crate::PolarTable`] alpha grid. Sampling first delegates
//! alpha handling to the adjacent tables, then interpolates their coefficients linearly in
//! `ln(Re)` without extrapolating beyond the family endpoints.

use crate::{PolarCoefficients, PolarTable};
use thiserror::Error;

/// One validated Reynolds node and its independently gridded alpha polar.
#[derive(Debug, Clone, PartialEq)]
pub struct ReynoldsPolar {
    reynolds_number: f64,
    table: PolarTable,
}

impl ReynoldsPolar {
    pub fn new(reynolds_number: f64, table: PolarTable) -> Result<Self, ReynoldsPolarFamilyError> {
        if !reynolds_number.is_finite() {
            return Err(ReynoldsPolarFamilyError::NonFiniteReynoldsNumber);
        }
        if reynolds_number <= 0.0 {
            return Err(ReynoldsPolarFamilyError::NonPositiveReynoldsNumber);
        }
        Ok(Self {
            reynolds_number,
            table,
        })
    }

    #[must_use]
    pub const fn reynolds_number(&self) -> f64 {
        self.reynolds_number
    }

    #[must_use]
    pub const fn table(&self) -> &PolarTable {
        &self.table
    }
}

/// Construction failures for a Reynolds-indexed polar family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum ReynoldsPolarFamilyError {
    #[error("Reynolds polar family requires at least one node")]
    Empty,
    #[error("Reynolds number must be finite")]
    NonFiniteReynoldsNumber,
    #[error("Reynolds number must be greater than zero")]
    NonPositiveReynoldsNumber,
    #[error("duplicate Reynolds number at canonical node index {sorted_index}")]
    DuplicateReynoldsNumber { sorted_index: usize },
}

/// Relationship between a requested Reynolds number and the available node range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReynoldsRangeStatus {
    BelowRange,
    ExactOrInRange,
    AboveRange,
}

/// Allocation-free result of Reynolds/alpha polar-family sampling.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReynoldsPolarSample<'a> {
    pub coefficients: PolarCoefficients,
    pub lower_reynolds: &'a ReynoldsPolar,
    pub upper_reynolds: &'a ReynoldsPolar,
    pub interpolation_fraction: f64,
    pub range_status: ReynoldsRangeStatus,
}

/// Immutable, canonically ordered family of alpha polars at discrete Reynolds numbers.
#[derive(Debug, Clone, PartialEq)]
pub struct ReynoldsPolarFamily {
    nodes: Vec<ReynoldsPolar>,
}

impl ReynoldsPolarFamily {
    /// Canonicalizes nodes by increasing Reynolds number and rejects duplicates.
    pub fn new(mut nodes: Vec<ReynoldsPolar>) -> Result<Self, ReynoldsPolarFamilyError> {
        if nodes.is_empty() {
            return Err(ReynoldsPolarFamilyError::Empty);
        }
        nodes.sort_by(|left, right| left.reynolds_number.total_cmp(&right.reynolds_number));
        for index in 1..nodes.len() {
            if nodes[index - 1].reynolds_number == nodes[index].reynolds_number {
                return Err(ReynoldsPolarFamilyError::DuplicateReynoldsNumber {
                    sorted_index: index,
                });
            }
        }
        Ok(Self { nodes })
    }

    #[must_use]
    pub fn nodes(&self) -> &[ReynoldsPolar] {
        &self.nodes
    }

    /// Samples each adjacent alpha polar with its legacy clamping behavior, then linearly
    /// interpolates CL/CD/CM in `ln(Re)`. Reynolds values outside the family are clamped to the
    /// nearest node and reported; no Reynolds extrapolation occurs.
    ///
    /// Callers must provide a finite, non-negative Reynolds number and finite angle of attack, as
    /// they do for the existing `PolarTable` hot-path sampler. Zero is below every valid family
    /// node and therefore clamps to the first node without evaluating a logarithm.
    #[must_use]
    pub fn sample(&self, reynolds_number: f64, alpha_rad: f64) -> ReynoldsPolarSample<'_> {
        debug_assert!(reynolds_number.is_finite() && reynolds_number >= 0.0);
        debug_assert!(alpha_rad.is_finite());

        match self
            .nodes
            .binary_search_by(|node| node.reynolds_number.total_cmp(&reynolds_number))
        {
            Ok(index) => {
                self.sample_single_node(index, alpha_rad, ReynoldsRangeStatus::ExactOrInRange)
            }
            Err(0) => self.sample_single_node(0, alpha_rad, ReynoldsRangeStatus::BelowRange),
            Err(upper) if upper == self.nodes.len() => self.sample_single_node(
                self.nodes.len() - 1,
                alpha_rad,
                ReynoldsRangeStatus::AboveRange,
            ),
            Err(upper) => {
                let lower = upper - 1;
                let lower_node = &self.nodes[lower];
                let upper_node = &self.nodes[upper];
                let denominator = log_ratio(upper_node.reynolds_number, lower_node.reynolds_number);
                let fraction = (log_ratio(reynolds_number, lower_node.reynolds_number)
                    / denominator)
                    .clamp(0.0, 1.0);
                let lower_coefficients = lower_node.table.sample_clamped(alpha_rad);
                let upper_coefficients = upper_node.table.sample_clamped(alpha_rad);
                ReynoldsPolarSample {
                    coefficients: PolarCoefficients {
                        cl: interpolate(lower_coefficients.cl, upper_coefficients.cl, fraction),
                        cd: interpolate(lower_coefficients.cd, upper_coefficients.cd, fraction),
                        cm: interpolate(lower_coefficients.cm, upper_coefficients.cm, fraction),
                    },
                    lower_reynolds: lower_node,
                    upper_reynolds: upper_node,
                    interpolation_fraction: fraction,
                    range_status: ReynoldsRangeStatus::ExactOrInRange,
                }
            }
        }
    }

    fn sample_single_node(
        &self,
        index: usize,
        alpha_rad: f64,
        range_status: ReynoldsRangeStatus,
    ) -> ReynoldsPolarSample<'_> {
        let node = &self.nodes[index];
        ReynoldsPolarSample {
            coefficients: node.table.sample_clamped(alpha_rad),
            lower_reynolds: node,
            upper_reynolds: node,
            interpolation_fraction: 0.0,
            range_status,
        }
    }
}

fn log_ratio(upper: f64, lower: f64) -> f64 {
    let relative_difference = (upper - lower) / lower;
    if relative_difference.is_finite() {
        relative_difference.ln_1p()
    } else {
        upper.ln() - lower.ln()
    }
}

fn interpolate(lower: f64, upper: f64, fraction: f64) -> f64 {
    if lower.is_sign_negative() == upper.is_sign_negative() {
        lower + fraction * (upper - lower)
    } else {
        lower * (1.0 - fraction) + upper * fraction
    }
}
