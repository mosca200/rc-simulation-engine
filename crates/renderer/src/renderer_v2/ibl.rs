//! RV2-5 image-based-lighting contract.
//!
//! The concrete persistent texture ownership lives beside the atmosphere
//! generator because both are produced by the same initialization encoder.
//! This module keeps the IBL-facing names and the single roughness-to-mip
//! policy centralized for the V2 shader and tests.

#[allow(dead_code)]
pub(crate) type IblResources = super::atmosphere::EnvironmentTextures;

#[allow(dead_code)]
pub(crate) const PREFILTERED_MIP_COUNT: u32 = super::atmosphere::SPECULAR_MIP_COUNT;

#[must_use]
#[allow(dead_code)]
pub(crate) fn roughness_to_mip(roughness: f32) -> f32 {
    roughness.clamp(0.0, 1.0) * (PREFILTERED_MIP_COUNT - 1) as f32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roughness_maps_to_first_middle_and_last_mip() {
        assert_eq!(roughness_to_mip(0.0), 0.0);
        assert_eq!(roughness_to_mip(0.5), 3.5);
        assert_eq!(roughness_to_mip(1.0), 7.0);
    }
}
