//! G2B directional-shadow configuration and stable light-space transforms.
//!
//! This module deliberately contains no `wgpu` types. It keeps the per-frame
//! light-camera calculation small, deterministic, allocation-free, and covered
//! by hardware-independent tests.

use crate::Mat4;

/// Persistent directional shadow-map resolution.
pub(crate) const SHADOW_MAP_RESOLUTION: u32 = 2_048;
/// Fixed square light-frustum half extent around the tracked aircraft.
pub(crate) const SHADOW_HALF_EXTENT_M: f32 = 128.0;
/// Light-camera near plane in render-space metres.
pub(crate) const SHADOW_NEAR_M: f32 = 1.0;
/// Light-camera far plane in render-space metres.
pub(crate) const SHADOW_FAR_M: f32 = 768.0;
/// Distance from the snapped centre to the orthographic light camera.
pub(crate) const SHADOW_LIGHT_EYE_DISTANCE_M: f32 = 384.0;

/// Rasterizer bias, in depth-format units, used while creating the shadow map.
pub(crate) const SHADOW_DEPTH_BIAS_CONSTANT: i32 = 2;
/// Slope-scaled rasterizer bias for the shadow-map caster pass.
pub(crate) const SHADOW_DEPTH_BIAS_SLOPE_SCALE: f32 = 2.0;
/// Small normalized receiver-depth offset used by the comparison sample.
pub(crate) const SHADOW_RECEIVER_DEPTH_BIAS: f32 = 0.000_15;

const WORLD_UP: [f32; 3] = [0.0, 1.0, 0.0];
const ALTERNATE_UP: [f32; 3] = [0.0, 0.0, 1.0];
const PARALLEL_UP_THRESHOLD: f32 = 0.99;

/// Stable directional-light transform for one rendered frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct DirectionalShadowTransform {
    pub(crate) light_view_projection: Mat4,
    pub(crate) snapped_center: [f32; 3],
}

/// World-space width/height represented by one shadow texel.
#[must_use]
pub(crate) const fn shadow_texel_size_m() -> f32 {
    (2.0 * SHADOW_HALF_EXTENT_M) / SHADOW_MAP_RESOLUTION as f32
}

/// Builds the fixed-extent light frustum around `tracked_world_position`.
///
/// `light_direction_toward_light` follows `EnvironmentUniform.light_direction`:
/// it points from shaded surfaces toward the directional light. The tracked
/// centre is projected into the light's right/up axes and snapped to one shadow
/// texel before the view matrix is built. Its depth coordinate stays continuous;
/// depth movement does not alter the projected texel grid.
#[must_use]
pub(crate) fn stable_directional_shadow_transform(
    light_direction_toward_light: [f32; 3],
    tracked_world_position: [f32; 3],
) -> DirectionalShadowTransform {
    let light_direction = normalize_or_default(light_direction_toward_light, WORLD_UP);
    let forward = scale3(light_direction, -1.0);
    let reference_up = if dot3(forward, WORLD_UP).abs() > PARALLEL_UP_THRESHOLD {
        ALTERNATE_UP
    } else {
        WORLD_UP
    };
    let right = normalize_or_default(cross3(forward, reference_up), [1.0, 0.0, 0.0]);
    let up = normalize_or_default(cross3(right, forward), WORLD_UP);

    let center = if tracked_world_position.into_iter().all(f32::is_finite) {
        tracked_world_position
    } else {
        [0.0; 3]
    };
    let texel_size = shadow_texel_size_m();
    let snapped_right = snap_to_texel(dot3(center, right), texel_size);
    let snapped_up = snap_to_texel(dot3(center, up), texel_size);
    let depth = dot3(center, forward);
    let snapped_center = add3(
        add3(scale3(right, snapped_right), scale3(up, snapped_up)),
        scale3(forward, depth),
    );

    let light_eye = add3(
        snapped_center,
        scale3(light_direction, SHADOW_LIGHT_EYE_DISTANCE_M),
    );
    let view = light_view_matrix(light_eye, right, up, forward);
    let projection = webgpu_orthographic(SHADOW_HALF_EXTENT_M, SHADOW_NEAR_M, SHADOW_FAR_M);

    DirectionalShadowTransform {
        light_view_projection: projection * view,
        snapped_center,
    }
}

#[must_use]
fn snap_to_texel(value: f32, texel_size: f32) -> f32 {
    (value / texel_size).round() * texel_size
}

#[must_use]
fn webgpu_orthographic(half_extent: f32, near_m: f32, far_m: f32) -> Mat4 {
    let depth_scale = 1.0 / (near_m - far_m);
    Mat4::from_rows([
        [half_extent.recip(), 0.0, 0.0, 0.0],
        [0.0, half_extent.recip(), 0.0, 0.0],
        [0.0, 0.0, depth_scale, near_m * depth_scale],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

#[must_use]
fn light_view_matrix(eye: [f32; 3], right: [f32; 3], up: [f32; 3], forward: [f32; 3]) -> Mat4 {
    Mat4::from_rows([
        [right[0], right[1], right[2], -dot3(right, eye)],
        [up[0], up[1], up[2], -dot3(up, eye)],
        [-forward[0], -forward[1], -forward[2], dot3(forward, eye)],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

#[must_use]
fn add3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

#[must_use]
fn scale3(vector: [f32; 3], scale: f32) -> [f32; 3] {
    [vector[0] * scale, vector[1] * scale, vector[2] * scale]
}

#[must_use]
fn dot3(left: [f32; 3], right: [f32; 3]) -> f32 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

#[must_use]
fn cross3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

#[must_use]
fn normalize_or_default(vector: [f32; 3], fallback: [f32; 3]) -> [f32; 3] {
    let length_squared = dot3(vector, vector);
    if !length_squared.is_finite() || length_squared <= f32::EPSILON {
        return fallback;
    }
    let inverse_length = length_squared.sqrt().recip();
    scale3(vector, inverse_length)
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_LIGHT_DIRECTION: [f32; 3] = [0.4, 0.8, -0.3];

    #[test]
    fn light_view_projection_is_finite() {
        let transform =
            stable_directional_shadow_transform(TEST_LIGHT_DIRECTION, [18.0, 12.0, -7.0]);
        assert!(transform.light_view_projection.is_finite());
        assert!(transform.snapped_center.into_iter().all(f32::is_finite));
    }

    #[test]
    fn texel_size_is_derived_from_extent_and_resolution() {
        assert_eq!(shadow_texel_size_m(), 0.125);
        assert_eq!(
            shadow_texel_size_m(),
            (2.0 * SHADOW_HALF_EXTENT_M) / SHADOW_MAP_RESOLUTION as f32
        );
    }

    #[test]
    fn sub_texel_motion_does_not_change_snapped_center() {
        let base = stable_directional_shadow_transform([0.0, 1.0, 0.0], [0.0; 3]);
        let moved = stable_directional_shadow_transform([0.0, 1.0, 0.0], [0.06, 0.0, 0.0]);
        assert_eq!(base.snapped_center, moved.snapped_center);
    }

    #[test]
    fn sub_texel_motion_keeps_default_light_space_xy_stable() {
        let base = stable_directional_shadow_transform(TEST_LIGHT_DIRECTION, [0.0; 3]);
        let moved = stable_directional_shadow_transform(TEST_LIGHT_DIRECTION, [0.06, 0.0, 0.0]);
        let base_clip = base
            .light_view_projection
            .transform_homogeneous([0.0, 0.0, 0.0, 1.0]);
        let moved_clip = moved
            .light_view_projection
            .transform_homogeneous([0.0, 0.0, 0.0, 1.0]);
        assert!((base_clip[0] - moved_clip[0]).abs() < 1.0e-6);
        assert!((base_clip[1] - moved_clip[1]).abs() < 1.0e-6);
    }

    #[test]
    fn motion_past_one_texel_updates_snapped_center() {
        let base = stable_directional_shadow_transform([0.0, 1.0, 0.0], [0.0; 3]);
        let moved = stable_directional_shadow_transform([0.0, 1.0, 0.0], [0.13, 0.0, 0.0]);
        assert_ne!(base.snapped_center, moved.snapped_center);
    }

    #[test]
    fn near_parallel_light_direction_uses_a_finite_alternate_up() {
        let transform =
            stable_directional_shadow_transform([0.0, 1.0, 0.000_001], [5.0, 3.0, -2.0]);
        assert!(transform.light_view_projection.is_finite());
    }

    #[test]
    fn shadow_configuration_constants_are_valid() {
        let positive_values = [
            SHADOW_HALF_EXTENT_M,
            SHADOW_NEAR_M,
            SHADOW_FAR_M,
            SHADOW_LIGHT_EYE_DISTANCE_M,
        ];
        assert!(
            positive_values
                .into_iter()
                .all(|value| value.is_finite() && value > 0.0)
        );
        let [near_m, far_m] = [SHADOW_NEAR_M, SHADOW_FAR_M];
        assert!(far_m > near_m);
        let resolution_from_extent_and_texels =
            ((2.0 * SHADOW_HALF_EXTENT_M) / shadow_texel_size_m()) as u32;
        assert_eq!(resolution_from_extent_and_texels, SHADOW_MAP_RESOLUTION);
        let receiver_bias = SHADOW_RECEIVER_DEPTH_BIAS;
        assert!(receiver_bias.is_finite() && receiver_bias >= 0.0);
    }
}
