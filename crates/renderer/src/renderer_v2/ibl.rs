//! RV2-5 image-based-lighting contract.
//!
//! The concrete persistent texture ownership lives beside the atmosphere
//! generator because both are produced by the same initialization encoder. This
//! module keeps the IBL-facing names, the single roughness-to-mip policy and the
//! documented convolution sample counts used by the V2 shader and its tests.

#[allow(dead_code)]
pub(crate) type IblResources = super::atmosphere::EnvironmentTextures;

/// Number of prefiltered specular mip levels (`roughness 1.0 -> mip 7`).
#[allow(dead_code)]
pub(crate) const PREFILTERED_MIP_COUNT: u32 = super::atmosphere::SPECULAR_MIP_COUNT;

/// Cosine-weighted hemisphere samples used by the diffuse irradiance pass.
#[allow(dead_code)]
pub(crate) const IRRADIANCE_SAMPLE_COUNT: u32 = 512;
/// GGX importance samples used by every specular prefilter mip.
#[allow(dead_code)]
pub(crate) const PREFILTER_SAMPLE_COUNT: u32 = 256;
/// Split-sum samples used by the BRDF integration LUT.
#[allow(dead_code)]
pub(crate) const BRDF_SAMPLE_COUNT: u32 = 512;

/// Continuous, clamped roughness-to-mip mapping.
///
/// `roughness 0.0 -> mip 0`, `roughness 1.0 -> mip 7`, linear in between, so a
/// material roughness never lands outside the generated mip chain.
#[must_use]
#[allow(dead_code)]
pub(crate) fn roughness_to_mip(roughness: f32) -> f32 {
    roughness.clamp(0.0, 1.0) * (PREFILTERED_MIP_COUNT - 1) as f32
}

/// Roughness baked into a prefiltered mip: `mip / (mip_count - 1)`.
#[must_use]
#[allow(dead_code)]
pub(crate) fn mip_to_roughness(mip: u32) -> f32 {
    mip.min(PREFILTERED_MIP_COUNT - 1) as f32 / (PREFILTERED_MIP_COUNT - 1) as f32
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

    #[test]
    fn roughness_mapping_is_clamped() {
        assert_eq!(roughness_to_mip(-2.0), 0.0);
        assert_eq!(roughness_to_mip(3.0), 7.0);
    }

    #[test]
    fn prefilter_mips_cover_the_full_roughness_range() {
        assert_eq!(mip_to_roughness(0), 0.0);
        assert_eq!(mip_to_roughness(PREFILTERED_MIP_COUNT - 1), 1.0);
        for mip in 0..PREFILTERED_MIP_COUNT {
            let roughness = mip_to_roughness(mip);
            assert!((roughness_to_mip(roughness) - mip as f32).abs() < 1e-6);
        }
    }
}
