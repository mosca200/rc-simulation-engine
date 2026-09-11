//! RV2-6 physical aerial-perspective state and deterministic CPU reference.
//!
//! The production shader integrates the short RC-scale camera-to-fragment
//! segment directly in the scene pass. This module owns only the persistent
//! V2 uniform packing; it creates no texture and has no frame-time lifecycle.
//!
//! RV2-5's sky-view and environment maps remain intentionally static at their
//! 2 m reference observer altitude. RV2-6 does not reuse that approximation
//! for geometry: every view segment derives altitude from the real local
//! camera/fragment positions and the renderer's ground reference. Consequently
//! elevated cameras get correct segment extinction, while the already-approved
//! sky background retains the small near-ground RV2-5 approximation.

use bytemuck::{Pod, Zeroable};

use super::atmosphere::AtmosphereParameters;

pub(crate) const AERIAL_PERSPECTIVE_SAMPLE_COUNT: u32 = 4;
const OZONE_CENTER_M: f32 = 25_000.0;
const OZONE_WIDTH_M: f32 = 15_000.0;

/// GPU-visible atmosphere coefficients used by the V2 scene shader.
///
/// Every member occupies one WGSL `vec4<f32>` slot. Keeping this state apart
/// from `EnvironmentUniform` preserves the V1 bind-group ABI exactly.
#[repr(C, align(16))]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct AerialPerspectiveUniformRaw {
    pub(crate) planet_ground: [f32; 4],
    pub(crate) density_profile: [f32; 4],
    pub(crate) rayleigh_scattering: [f32; 4],
    pub(crate) mie_scattering: [f32; 4],
    pub(crate) mie_extinction_anisotropy: [f32; 4],
    pub(crate) ozone_absorption: [f32; 4],
    /// Validation-only switch. Production construction always stores `1.0`.
    pub(crate) validation_control: [f32; 4],
}

impl AerialPerspectiveUniformRaw {
    /// Pack the validated RV2-5 atmosphere and the renderer-local ground.
    pub(crate) fn new(parameters: AtmosphereParameters, ground_below_render_origin_m: f32) -> Self {
        Self {
            planet_ground: [
                parameters.planet_radius_m,
                parameters.atmosphere_height_m,
                -ground_below_render_origin_m,
                AERIAL_PERSPECTIVE_SAMPLE_COUNT as f32,
            ],
            density_profile: [
                parameters.rayleigh_scale_height_m,
                parameters.mie_scale_height_m,
                OZONE_CENTER_M,
                OZONE_WIDTH_M,
            ],
            rayleigh_scattering: [
                parameters.rayleigh_scattering[0],
                parameters.rayleigh_scattering[1],
                parameters.rayleigh_scattering[2],
                0.0,
            ],
            mie_scattering: [
                parameters.mie_scattering[0],
                parameters.mie_scattering[1],
                parameters.mie_scattering[2],
                0.0,
            ],
            mie_extinction_anisotropy: [
                parameters.mie_extinction[0],
                parameters.mie_extinction[1],
                parameters.mie_extinction[2],
                parameters.mie_anisotropy,
            ],
            ozone_absorption: [
                parameters.ozone_absorption[0],
                parameters.ozone_absorption[1],
                parameters.ozone_absorption[2],
                0.0,
            ],
            validation_control: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Select AP compositing for the controlled visual gate without changing
    /// any RV2-5 physical-environment state or atmospheric coefficient.
    #[must_use]
    pub(crate) const fn with_validation_enabled(mut self, enabled: bool) -> Self {
        self.validation_control[0] = enabled as u32 as f32;
        self
    }

    #[cfg(test)]
    fn is_finite(self) -> bool {
        self.planet_ground
            .into_iter()
            .chain(self.density_profile)
            .chain(self.rayleigh_scattering)
            .chain(self.mie_scattering)
            .chain(self.mie_extinction_anisotropy)
            .chain(self.ozone_absorption)
            .chain(self.validation_control)
            .all(f32::is_finite)
    }
}

#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq)]
struct ReferenceAerialPerspective {
    transmittance: [f32; 3],
    inscattered_radiance: [f32; 3],
}

#[cfg(test)]
fn reference_evaluate(
    state: AerialPerspectiveUniformRaw,
    camera: [f32; 3],
    fragment: [f32; 3],
    sun_direction: [f32; 3],
    sun_irradiance: [f32; 3],
    sun_transmittance: [f32; 3],
    multiple_scattering_transfer: [f32; 3],
) -> ReferenceAerialPerspective {
    let segment = sub3(fragment, camera);
    let distance = length3(segment);
    if distance <= 1e-4 {
        return ReferenceAerialPerspective {
            transmittance: [1.0; 3],
            inscattered_radiance: [0.0; 3],
        };
    }

    let view_direction = scale3(segment, 1.0 / distance);
    let sun_direction = normalize3(sun_direction);
    let cos_theta = dot3(view_direction, sun_direction).clamp(-1.0, 1.0);
    let rayleigh_phase = 3.0 / (16.0 * std::f32::consts::PI) * (1.0 + cos_theta * cos_theta);
    let g = state.mie_extinction_anisotropy[3];
    let phase_denominator = (1.0 + g * g - 2.0 * g * cos_theta).max(1e-4);
    let mie_phase = (1.0 - g * g) / (4.0 * std::f32::consts::PI * phase_denominator.powf(1.5));
    let step_length = distance / AERIAL_PERSPECTIVE_SAMPLE_COUNT as f32;
    let mut optical_depth = [0.0; 3];
    let mut inscattered = [0.0; 3];

    for sample_index in 0..AERIAL_PERSPECTIVE_SAMPLE_COUNT {
        let sample_distance = (sample_index as f32 + 0.5) * step_length;
        let sample = add3(camera, scale3(view_direction, sample_distance));
        let height = (sample[1] - state.planet_ground[2]).clamp(0.0, state.planet_ground[1]);
        let rho_rayleigh = (-height / state.density_profile[0]).exp();
        let rho_mie = (-height / state.density_profile[1]).exp();
        let rho_ozone =
            (1.0 - (height - state.density_profile[2]).abs() / state.density_profile[3]).max(0.0);
        let extinction: [f32; 3] = std::array::from_fn(|channel| {
            state.rayleigh_scattering[channel] * rho_rayleigh
                + state.mie_extinction_anisotropy[channel] * rho_mie
                + state.ozone_absorption[channel] * rho_ozone
        });
        let midpoint_transmittance: [f32; 3] = std::array::from_fn(|channel| {
            (-(optical_depth[channel] + extinction[channel] * step_length * 0.5)).exp()
        });
        let up = sample_up(state, sample, height);
        let sun_mu = dot3(up, sun_direction).clamp(-1.0, 1.0);
        let visibility = if sun_is_visible(state, height, sun_mu) {
            1.0
        } else {
            0.0
        };

        for channel in 0..3 {
            let scattering = state.rayleigh_scattering[channel] * rho_rayleigh
                + state.mie_scattering[channel] * rho_mie;
            let single_source = sun_irradiance[channel]
                * sun_transmittance[channel]
                * visibility
                * (state.rayleigh_scattering[channel] * rho_rayleigh * rayleigh_phase
                    + state.mie_scattering[channel] * rho_mie * mie_phase);
            let multiple_source =
                sun_irradiance[channel] * multiple_scattering_transfer[channel] * scattering;
            inscattered[channel] +=
                midpoint_transmittance[channel] * (single_source + multiple_source) * step_length;
            optical_depth[channel] += extinction[channel] * step_length;
        }
    }

    ReferenceAerialPerspective {
        transmittance: optical_depth.map(|depth| (-depth).exp().clamp(0.0, 1.0)),
        inscattered_radiance: inscattered.map(|value| value.max(0.0)),
    }
}

#[cfg(test)]
fn sun_is_visible(state: AerialPerspectiveUniformRaw, height: f32, sun_mu: f32) -> bool {
    if sun_mu >= 0.0 {
        return true;
    }
    let radius = state.planet_ground[0] + height;
    let horizon_sine = (height * (2.0 * state.planet_ground[0] + height))
        .max(0.0)
        .sqrt()
        / radius.max(1.0);
    sun_mu >= -horizon_sine
}

#[cfg(test)]
fn sample_up(state: AerialPerspectiveUniformRaw, sample: [f32; 3], height: f32) -> [f32; 3] {
    normalize3([sample[0], state.planet_ground[0] + height, sample[2]])
}

#[cfg(test)]
fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|index| a[index] + b[index])
}

#[cfg(test)]
fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|index| a[index] - b[index])
}

#[cfg(test)]
fn scale3(value: [f32; 3], scale: f32) -> [f32; 3] {
    value.map(|component| component * scale)
}

#[cfg(test)]
fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a.into_iter().zip(b).map(|(a, b)| a * b).sum()
}

#[cfg(test)]
fn length3(value: [f32; 3]) -> f32 {
    dot3(value, value).sqrt()
}

#[cfg(test)]
fn normalize3(value: [f32; 3]) -> [f32; 3] {
    let length = length3(value);
    if length > f32::EPSILON && length.is_finite() {
        scale3(value, 1.0 / length)
    } else {
        [0.0, 1.0, 0.0]
    }
}

#[cfg(test)]
mod tests {
    use std::mem::{align_of, size_of};

    use super::*;

    const SUN_DIRECTION: [f32; 3] = [0.423_999_16, 0.847_998_3, -0.317_999_36];
    const SUN_IRRADIANCE: [f32; 3] = [2.4, 2.28, 2.04];
    const SUN_TRANSMITTANCE: [f32; 3] = [0.80, 0.72, 0.58];
    const MULTIPLE_TRANSFER: [f32; 3] = [0.025, 0.035, 0.050];

    fn state() -> AerialPerspectiveUniformRaw {
        AerialPerspectiveUniformRaw::new(AtmosphereParameters::earth(), 2.0)
    }

    fn evaluate(distance: f32) -> ReferenceAerialPerspective {
        reference_evaluate(
            state(),
            [0.0, 0.0, 0.0],
            [distance, 0.0, 0.0],
            SUN_DIRECTION,
            SUN_IRRADIANCE,
            SUN_TRANSMITTANCE,
            MULTIPLE_TRANSFER,
        )
    }

    #[test]
    fn uniform_packing_is_finite_and_wgsl_aligned() {
        let state = state();
        assert_eq!(size_of::<AerialPerspectiveUniformRaw>(), 112);
        assert_eq!(align_of::<AerialPerspectiveUniformRaw>(), 16);
        assert!(state.is_finite());
        assert_eq!(state.planet_ground[2], -2.0);
        assert_eq!(state.planet_ground[3], 4.0);
        assert_eq!(state.validation_control[0], 1.0);
        assert_eq!(
            state.with_validation_enabled(false).validation_control[0],
            0.0
        );
    }

    #[test]
    fn zero_distance_is_identity() {
        let result = evaluate(0.0);
        assert_eq!(result.transmittance, [1.0; 3]);
        assert_eq!(result.inscattered_radiance, [0.0; 3]);
    }

    #[test]
    fn extinction_increases_and_transmittance_stays_bounded() {
        let near = evaluate(100.0);
        let middle = evaluate(500.0);
        let far = evaluate(1_000.0);
        for channel in 0..3 {
            assert!((0.0..=1.0).contains(&near.transmittance[channel]));
            assert!(middle.transmittance[channel] <= near.transmittance[channel]);
            assert!(far.transmittance[channel] <= middle.transmittance[channel]);
        }
    }

    #[test]
    fn outputs_are_finite_non_negative_and_deterministic() {
        let first = evaluate(500.0);
        let second = evaluate(500.0);
        assert_eq!(first, second);
        for value in first
            .transmittance
            .into_iter()
            .chain(first.inscattered_radiance)
        {
            assert!(value.is_finite());
            assert!(value >= 0.0);
        }
    }

    #[test]
    fn inscattering_scales_once_with_sun_irradiance() {
        let base = evaluate(300.0);
        let doubled = reference_evaluate(
            state(),
            [0.0, 0.0, 0.0],
            [300.0, 0.0, 0.0],
            SUN_DIRECTION,
            SUN_IRRADIANCE.map(|value| value * 2.0),
            SUN_TRANSMITTANCE,
            MULTIPLE_TRANSFER,
        );
        for channel in 0..3 {
            let expected = base.inscattered_radiance[channel] * 2.0;
            assert!((doubled.inscattered_radiance[channel] - expected).abs() < 1e-6);
        }
    }

    #[test]
    fn planet_visibility_blocks_below_horizon_sun_at_ground() {
        assert!(!sun_is_visible(state(), 0.0, -0.01));
        assert!(sun_is_visible(state(), 0.0, 0.0));
        assert!(sun_is_visible(state(), 100.0, -0.003));
        assert!(!sun_is_visible(state(), 100.0, -0.01));
    }
}
