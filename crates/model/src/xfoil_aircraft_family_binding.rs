//! Deterministic binding of canonical XFOIL evidence to an aircraft runtime family.
//!
//! M2.9L closes the runtime loop: canonical M2.9B evidence JSON → M2.9K loader →
//! replacement of exactly one named `RuntimeReynoldsPolarFamily` in an existing
//! `AircraftModel`. Aero-element bindings reference families by index, so the
//! replacement preserves the family index and all existing bindings remain valid.

use crate::{
    AircraftModel, XfoilEvidenceJsonError, XfoilRuntimePolarFamily,
    build_xfoil_reynolds_polar_family_from_json,
};

/// Result of a successful evidence-to-aircraft-family binding.
#[derive(Debug, Clone)]
pub struct XfoilEvidenceBindingResult {
    family_index: usize,
    family_id: String,
    mach: f64,
    runtime_family: XfoilRuntimePolarFamily,
}

impl XfoilEvidenceBindingResult {
    /// Index of the replaced family in `AircraftModel::aero_polar_families()`.
    #[must_use]
    pub const fn family_index(&self) -> usize {
        self.family_index
    }

    /// Stable ID of the replaced family (preserved from the original model).
    #[must_use]
    pub fn family_id(&self) -> &str {
        &self.family_id
    }

    /// Common Mach number from the evidence datasets.
    #[must_use]
    pub const fn mach(&self) -> f64 {
        self.mach
    }

    /// The runtime polar family built from the evidence.
    #[must_use]
    pub fn runtime_family(&self) -> &XfoilRuntimePolarFamily {
        &self.runtime_family
    }
}

/// Errors from the evidence-to-aircraft-family binding.
#[derive(Debug, thiserror::Error)]
pub enum XfoilEvidenceBindingError {
    #[error("no Reynolds polar family with ID {family_id:?} found in aircraft model")]
    FamilyNotFound { family_id: String },

    #[error("evidence JSON error: {0}")]
    EvidenceJson(#[from] XfoilEvidenceJsonError),
}

/// Replace exactly one named `RuntimeReynoldsPolarFamily` in an `AircraftModel`
/// using canonical XFOIL evidence JSON bytes.
///
/// The family is identified by `family_id`. Its index in
/// `model.aero_polar_families()` is preserved so existing aero-element bindings
/// (`RuntimeAeroPolarBinding::ReynoldsFamily { family_index }`) remain valid.
///
/// Only the targeted family is replaced; all other aircraft configuration,
/// physics, and families are unchanged.
///
/// # Determinism
///
/// Given the same model state and evidence JSON, repeated calls produce
/// identical results. The physics fingerprint changes when and only when the
/// polar data changes.
pub fn bind_xfoil_evidence_to_reynolds_family(
    model: &mut AircraftModel,
    family_id: &str,
    json_bytes: &[u8],
) -> Result<XfoilEvidenceBindingResult, XfoilEvidenceBindingError> {
    let family_index = model.find_reynolds_family_index(family_id).ok_or_else(|| {
        XfoilEvidenceBindingError::FamilyNotFound {
            family_id: family_id.to_owned(),
        }
    })?;

    let runtime_family = build_xfoil_reynolds_polar_family_from_json(json_bytes)?;

    model.replace_reynolds_polar_family_at(family_index, runtime_family.family().clone());

    Ok(XfoilEvidenceBindingResult {
        family_index,
        family_id: family_id.to_owned(),
        mach: runtime_family.mach(),
        runtime_family,
    })
}
