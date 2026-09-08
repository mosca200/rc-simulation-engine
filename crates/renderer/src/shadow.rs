//! G3E stable cascaded directional-shadow configuration and light-space transforms.
//!
//! The production shadow contract is deliberately narrow: exactly three fixed
//! cascades, one resolution, fixed split distances, stable orthographic
//! projections, and per-cascade texel snapping. This module contains no `wgpu`
//! types, so the per-frame calculation stays deterministic, allocation-free,
//! and covered by hardware-independent tests.

use crate::Mat4;

/// Exactly three cascades: aircraft/runway, field/vegetation, and far field.
pub(crate) const SHADOW_CASCADE_COUNT: usize = 3;
/// Persistent resolution of every layer in the shadow texture array.
pub(crate) const SHADOW_MAP_RESOLUTION: u32 = 2_048;
/// View-distance split far planes in render-space metres.
///
/// The near cascade prioritizes aircraft contact and runway detail, the middle
/// cascade covers the operational field and vegetation, and the far cascade
/// carries low-frequency terrain silhouettes out to typical RC visibility.
pub(crate) const SHADOW_CASCADE_SPLITS_M: [f32; SHADOW_CASCADE_COUNT] = [32.0, 128.0, 512.0];
/// Fixed square light-space half extents. Keeping these fixed prevents scale
/// changes (and therefore shadow swimming) as the camera or aircraft moves.
pub(crate) const SHADOW_CASCADE_HALF_EXTENTS_M: [f32; SHADOW_CASCADE_COUNT] = [40.0, 128.0, 512.0];
/// Extra caster depth on either side of a cascade's tracked centre.
const SHADOW_CASTER_DEPTH_MARGIN_M: f32 = 64.0;
/// Desired world-space receiver offset, converted to normalized light depth
/// independently for each cascade. This stays below two near-cascade texels,
/// preserving aircraft/runway contact while covering depth interpolation noise.
const SHADOW_RECEIVER_BIAS_M: f32 = 0.06;

/// Rasterizer bias, in depth-format units, used while creating each layer.
pub(crate) const SHADOW_DEPTH_BIAS_CONSTANT: i32 = 2;
/// Slope-scaled rasterizer bias for the shadow-map caster passes.
pub(crate) const SHADOW_DEPTH_BIAS_SLOPE_SCALE: f32 = 2.0;

const WORLD_UP: [f32; 3] = [0.0, 1.0, 0.0];
const VIEW_FORWARD_FALLBACK: [f32; 3] = [0.0, 0.0, -1.0];
const ALTERNATE_UP: [f32; 3] = [0.0, 0.0, 1.0];
const PARALLEL_UP_THRESHOLD: f32 = 0.99;

/// One stable production shadow cascade for a rendered frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ShadowCascade {
    pub(crate) light_view_projection: Mat4,
    pub(crate) snapped_center: [f32; 3],
    pub(crate) split_near_m: f32,
    pub(crate) split_far_m: f32,
    pub(crate) half_extent_m: f32,
    pub(crate) texel_size_m: f32,
    pub(crate) receiver_depth_bias: f32,
}

/// World-space width/height represented by one texel in `cascade_index`.
#[must_use]
#[cfg(test)]
pub(crate) const fn shadow_texel_size_m(cascade_index: usize) -> f32 {
    (2.0 * SHADOW_CASCADE_HALF_EXTENTS_M[cascade_index]) / SHADOW_MAP_RESOLUTION as f32
}

/// Select the cascade for a camera-to-receiver distance.
///
/// Non-finite, negative, or beyond-far values are outside the shadow coverage.
#[must_use]
#[cfg(test)]
pub(crate) fn select_shadow_cascade(view_distance_m: f32) -> Option<usize> {
    if !view_distance_m.is_finite() || view_distance_m < 0.0 {
        return None;
    }
    SHADOW_CASCADE_SPLITS_M
        .iter()
        .position(|split| view_distance_m <= *split)
}

/// Build the three fixed-extent cascades along the active camera view.
///
/// `light_direction_toward_light` follows `EnvironmentUniform.light_direction`:
/// it points from shaded surfaces toward the sun. `camera_eye` and
/// `camera_target` define only where each fixed cascade is centred; camera
/// projection/FOV and physics state are not modified. Every centre is snapped
/// independently in the light's right/up plane, while its light-depth
/// coordinate remains continuous and cannot move the projected texel grid.
#[must_use]
pub(crate) fn build_shadow_cascades(
    light_direction_toward_light: [f32; 3],
    camera_eye: [f32; 3],
    camera_target: [f32; 3],
) -> [ShadowCascade; SHADOW_CASCADE_COUNT] {
    let eye = finite_or_default(camera_eye, [0.0; 3]);
    let target = finite_or_default(camera_target, add3(eye, VIEW_FORWARD_FALLBACK));
    let view_forward = normalize_or_default(sub3(target, eye), VIEW_FORWARD_FALLBACK);

    std::array::from_fn(|index| {
        let split_near_m = if index == 0 {
            0.0
        } else {
            SHADOW_CASCADE_SPLITS_M[index - 1]
        };
        let split_far_m = SHADOW_CASCADE_SPLITS_M[index];
        let center_distance_m = 0.5 * (split_near_m + split_far_m);
        let tracked_center = add3(eye, scale3(view_forward, center_distance_m));
        stable_shadow_cascade(
            light_direction_toward_light,
            tracked_center,
            split_near_m,
            split_far_m,
            SHADOW_CASCADE_HALF_EXTENTS_M[index],
        )
    })
}

#[must_use]
fn stable_shadow_cascade(
    light_direction_toward_light: [f32; 3],
    tracked_center: [f32; 3],
    split_near_m: f32,
    split_far_m: f32,
    half_extent_m: f32,
) -> ShadowCascade {
    let light_direction = normalize_or_default(light_direction_toward_light, WORLD_UP);
    let forward = scale3(light_direction, -1.0);
    let reference_up = if dot3(forward, WORLD_UP).abs() > PARALLEL_UP_THRESHOLD {
        ALTERNATE_UP
    } else {
        WORLD_UP
    };
    let right = normalize_or_default(cross3(forward, reference_up), [1.0, 0.0, 0.0]);
    let up = normalize_or_default(cross3(right, forward), WORLD_UP);

    let texel_size_m = (2.0 * half_extent_m) / SHADOW_MAP_RESOLUTION as f32;
    let snapped_right = snap_to_texel(dot3(tracked_center, right), texel_size_m);
    let snapped_up = snap_to_texel(dot3(tracked_center, up), texel_size_m);
    let depth = dot3(tracked_center, forward);
    let snapped_center = add3(
        add3(scale3(right, snapped_right), scale3(up, snapped_up)),
        scale3(forward, depth),
    );

    let light_eye_distance_m = half_extent_m + SHADOW_CASTER_DEPTH_MARGIN_M;
    let light_eye = add3(
        snapped_center,
        scale3(light_direction, light_eye_distance_m),
    );
    let near_m = 1.0;
    let far_m = 2.0 * light_eye_distance_m;
    let view = light_view_matrix(light_eye, right, up, forward);
    let projection = webgpu_orthographic(half_extent_m, near_m, far_m);

    ShadowCascade {
        light_view_projection: projection * view,
        snapped_center,
        split_near_m,
        split_far_m,
        half_extent_m,
        texel_size_m,
        receiver_depth_bias: SHADOW_RECEIVER_BIAS_M / (far_m - near_m),
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
fn finite_or_default(value: [f32; 3], fallback: [f32; 3]) -> [f32; 3] {
    if value.into_iter().all(f32::is_finite) {
        value
    } else {
        fallback
    }
}

#[must_use]
fn add3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

#[must_use]
fn sub3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
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
    fn production_configuration_is_exactly_three_ordered_cascades() {
        assert_eq!(SHADOW_CASCADE_COUNT, 3);
        assert_eq!(SHADOW_CASCADE_SPLITS_M, [32.0, 128.0, 512.0]);
        assert!(
            SHADOW_CASCADE_SPLITS_M
                .windows(2)
                .all(|pair| pair[0] < pair[1])
        );
        assert!(
            SHADOW_CASCADE_HALF_EXTENTS_M
                .windows(2)
                .all(|pair| pair[0] < pair[1])
        );
    }

    #[test]
    fn all_cascade_matrices_and_metadata_are_finite() {
        let cascades =
            build_shadow_cascades(TEST_LIGHT_DIRECTION, [3.0, 2.0, 8.0], [3.0, 1.0, -20.0]);
        for (index, cascade) in cascades.iter().enumerate() {
            assert!(cascade.light_view_projection.is_finite());
            assert!(cascade.snapped_center.into_iter().all(f32::is_finite));
            assert_eq!(cascade.split_far_m, SHADOW_CASCADE_SPLITS_M[index]);
            assert!(cascade.split_near_m < cascade.split_far_m);
            assert!(cascade.half_extent_m.is_finite() && cascade.half_extent_m > 0.0);
            assert!(cascade.texel_size_m.is_finite() && cascade.texel_size_m > 0.0);
            assert!(cascade.receiver_depth_bias.is_finite() && cascade.receiver_depth_bias > 0.0);
        }
    }

    #[test]
    fn texel_sizes_match_extent_and_resolution() {
        assert_eq!(shadow_texel_size_m(0), 0.0390625);
        assert_eq!(shadow_texel_size_m(1), 0.125);
        assert_eq!(shadow_texel_size_m(2), 0.5);
        for (index, half_extent) in SHADOW_CASCADE_HALF_EXTENTS_M.iter().enumerate() {
            assert_eq!(
                shadow_texel_size_m(index),
                (2.0 * half_extent) / SHADOW_MAP_RESOLUTION as f32
            );
        }
    }

    #[test]
    fn sub_texel_camera_translation_keeps_all_light_space_xy_stable() {
        let base = build_shadow_cascades([0.0, 1.0, 0.0], [0.0; 3], [0.0, 0.0, -1.0]);
        let moved = build_shadow_cascades([0.0, 1.0, 0.0], [0.01, 0.0, 0.0], [0.01, 0.0, -1.0]);
        for (base, moved) in base.iter().zip(moved.iter()) {
            assert_eq!(base.snapped_center, moved.snapped_center);
            let base_clip = base
                .light_view_projection
                .transform_homogeneous([0.0, 0.0, 0.0, 1.0]);
            let moved_clip = moved
                .light_view_projection
                .transform_homogeneous([0.0, 0.0, 0.0, 1.0]);
            assert!((base_clip[0] - moved_clip[0]).abs() < 1.0e-6);
            assert!((base_clip[1] - moved_clip[1]).abs() < 1.0e-6);
        }
    }

    #[test]
    fn motion_past_near_texel_updates_near_center() {
        let base = build_shadow_cascades([0.0, 1.0, 0.0], [0.0; 3], [0.0, 0.0, -1.0]);
        let moved = build_shadow_cascades([0.0, 1.0, 0.0], [0.04, 0.0, 0.0], [0.04, 0.0, -1.0]);
        assert_ne!(base[0].snapped_center, moved[0].snapped_center);
    }

    #[test]
    fn cascade_selection_honors_boundaries_and_rejects_outside() {
        assert_eq!(select_shadow_cascade(0.0), Some(0));
        assert_eq!(select_shadow_cascade(32.0), Some(0));
        assert_eq!(select_shadow_cascade(32.001), Some(1));
        assert_eq!(select_shadow_cascade(128.0), Some(1));
        assert_eq!(select_shadow_cascade(128.001), Some(2));
        assert_eq!(select_shadow_cascade(512.0), Some(2));
        assert_eq!(select_shadow_cascade(512.001), None);
        assert_eq!(select_shadow_cascade(-1.0), None);
        assert_eq!(select_shadow_cascade(f32::NAN), None);
    }

    #[test]
    fn near_parallel_light_direction_uses_finite_alternate_up() {
        let cascades =
            build_shadow_cascades([0.0, 1.0, 0.000_001], [5.0, 3.0, -2.0], [5.0, 3.0, -3.0]);
        assert!(
            cascades
                .iter()
                .all(|cascade| cascade.light_view_projection.is_finite())
        );
    }

    #[test]
    fn invalid_camera_input_fails_closed_to_finite_cascades() {
        let cascades = build_shadow_cascades([f32::NAN; 3], [f32::INFINITY; 3], [f32::NAN; 3]);
        assert!(
            cascades
                .iter()
                .all(|cascade| cascade.light_view_projection.is_finite())
        );
    }
}
