use crate::{
    Mat4, RenderPose,
    math::{add3, look_at_rh, scale3, sub3},
    webgpu_perspective,
};

const VERTICAL_FOV_RAD: f32 = 55.0_f32.to_radians();
const NEAR_PLANE_M: f32 = 0.05;
const FAR_PLANE_M: f32 = 5_000.0;
const DISTANCE_BEHIND_M: f32 = 3.5;
const HEIGHT_ABOVE_M: f32 = 1.25;
const LOOK_AHEAD_M: f32 = 1.5;

/// Render-space world-up direction: +Y is up.
///
/// The NED-to-render mapping sends physics Down (NED +Z) to render −Y,
/// so physics Up (NED −Z) maps to render +Y.
pub const RENDER_WORLD_UP: [f32; 3] = [0.0, 1.0, 0.0];

/// Stable world-up chase camera driven only by a render pose.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChaseCamera {
    aspect_ratio: f32,
}

impl ChaseCamera {
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        let aspect_ratio = valid_aspect_ratio(width, height).unwrap_or(1.0);
        Self { aspect_ratio }
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if let Some(aspect_ratio) = valid_aspect_ratio(width, height) {
            self.aspect_ratio = aspect_ratio;
        }
    }

    #[must_use]
    pub const fn aspect_ratio(&self) -> f32 {
        self.aspect_ratio
    }

    #[must_use]
    pub fn eye_and_target(&self, aircraft_pose: &RenderPose) -> ([f32; 3], [f32; 3]) {
        let position = aircraft_pose.translation_render_m();
        let forward = aircraft_pose.transform_direction([0.0, 0.0, -1.0]);
        let horizontal_forward = normalized_horizontal_forward(forward);
        let eye = add3(
            sub3(position, scale3(horizontal_forward, DISTANCE_BEHIND_M)),
            [0.0, HEIGHT_ABOVE_M, 0.0],
        );
        let target = add3(position, scale3(forward, LOOK_AHEAD_M));
        (eye, target)
    }

    /// Camera world-space position (eye) for the given aircraft pose.
    #[must_use]
    pub fn eye_position(&self, aircraft_pose: &RenderPose) -> [f32; 3] {
        self.eye_and_target(aircraft_pose).0
    }

    #[must_use]
    pub fn view_projection(&self, aircraft_pose: &RenderPose) -> Mat4 {
        let (eye, target) = self.eye_and_target(aircraft_pose);
        let view = look_at_rh(eye, target, RENDER_WORLD_UP);
        let projection = webgpu_perspective(
            VERTICAL_FOV_RAD,
            self.aspect_ratio,
            NEAR_PLANE_M,
            FAR_PLANE_M,
        )
        .expect("fixed chase-camera projection parameters are valid");
        projection * view
    }

    /// Inverse of the view-projection matrix.
    ///
    /// Returns `None` if the matrix is singular (should not happen with valid
    /// camera parameters, but handled cleanly for robustness).
    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        self.view_projection(aircraft_pose).inverse()
    }
}

fn normalized_horizontal_forward(forward: [f32; 3]) -> [f32; 3] {
    let horizontal_norm = forward[0].hypot(forward[2]);
    if horizontal_norm <= 1.0e-4 {
        [0.0, 0.0, -1.0]
    } else {
        [
            forward[0] / horizontal_norm,
            0.0,
            forward[2] / horizontal_norm,
        ]
    }
}

fn valid_aspect_ratio(width: u32, height: u32) -> Option<f32> {
    (width > 0 && height > 0).then(|| width as f32 / height as f32)
}

// ---------------------------------------------------------------------------
// G1B atmosphere math helpers — pure functions, CPU-side testable.
// ---------------------------------------------------------------------------

/// View elevation: dot(view_direction, world_up).
///
/// - +1.0 → looking straight up (zenith)
/// -  0.0 → looking at the horizon
/// - −1.0 → looking straight down (nadir)
///
/// Both inputs should be normalized direction vectors.
#[must_use]
pub fn view_elevation(view_direction: [f32; 3], world_up: [f32; 3]) -> f32 {
    view_direction[0] * world_up[0]
        + view_direction[1] * world_up[1]
        + view_direction[2] * world_up[2]
}

/// Exponential distance fog factor.
///
/// Formula: `fog = 1 − exp(−density × distance)`
///
/// - distance = 0 → fog ≈ 0 (no fog)
/// - larger distance → monotonically higher fog
/// - output clamped to [0, 1]
/// - density must be non-negative
#[must_use]
pub fn exponential_fog_factor(distance: f32, density: f32) -> f32 {
    if density <= 0.0 || distance <= 0.0 {
        return 0.0;
    }
    let factor = 1.0 - (-density * distance).exp();
    factor.clamp(0.0, 1.0)
}

/// Sun alignment: dot(view_direction, sun_direction).
///
/// - +1.0 → looking directly at the sun
/// -  0.0 → perpendicular
/// - −1.0 → looking directly away from the sun
///
/// Both inputs should be normalized.
#[must_use]
pub fn sun_alignment(view_direction: [f32; 3], sun_direction: [f32; 3]) -> f32 {
    view_direction[0] * sun_direction[0]
        + view_direction[1] * sun_direction[1]
        + view_direction[2] * sun_direction[2]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::world_ned_pose_to_render;

    fn pose(quaternion: [f64; 4]) -> RenderPose {
        world_ned_pose_to_render([0.0; 3], quaternion, [0.0; 3]).unwrap()
    }

    #[test]
    fn identity_camera_is_behind_and_targets_ahead() {
        let camera = ChaseCamera::new(1_600, 900);
        let (eye, target) = camera.eye_and_target(&pose([1.0, 0.0, 0.0, 0.0]));
        assert_eq!(eye, [0.0, 1.25, 3.5]);
        assert_eq!(target, [0.0, 0.0, -1.5]);
        assert!(
            camera
                .view_projection(&pose([1.0, 0.0, 0.0, 0.0]))
                .is_finite()
        );
    }

    #[test]
    fn inclined_attitude_produces_a_finite_non_degenerate_camera() {
        let camera = ChaseCamera::new(1_280, 720);
        let inclined_pose = pose([0.75, 0.25, -0.35, 0.5]);
        let (eye, target) = camera.eye_and_target(&inclined_pose);
        assert_ne!(eye, target);
        assert!(eye.into_iter().chain(target).all(f32::is_finite));
        let first = camera.view_projection(&inclined_pose);
        let second = camera.view_projection(&inclined_pose);
        assert!(first.is_finite());
        assert_eq!(first, second);
    }

    #[test]
    fn vertical_attitude_uses_a_stable_finite_heading_fallback() {
        let camera = ChaseCamera::new(1_280, 720);
        let vertical_pose = pose([
            std::f64::consts::FRAC_1_SQRT_2,
            0.0,
            -std::f64::consts::FRAC_1_SQRT_2,
            0.0,
        ]);
        let (eye, target) = camera.eye_and_target(&vertical_pose);
        assert_ne!(eye, target);
        assert!(eye.into_iter().chain(target).all(f32::is_finite));
        assert!(camera.view_projection(&vertical_pose).is_finite());
    }

    #[test]
    fn resize_updates_aspect_and_ignores_zero_extent() {
        let mut camera = ChaseCamera::new(800, 600);
        assert!((camera.aspect_ratio() - 4.0 / 3.0).abs() < f32::EPSILON);
        camera.resize(1_920, 1_080);
        assert!((camera.aspect_ratio() - 16.0 / 9.0).abs() < f32::EPSILON);
        camera.resize(0, 0);
        assert!((camera.aspect_ratio() - 16.0 / 9.0).abs() < f32::EPSILON);
    }

    #[test]
    fn eye_position_matches_eye_and_target() {
        let camera = ChaseCamera::new(1_600, 900);
        let test_pose = pose([1.0, 0.0, 0.0, 0.0]);
        let (expected_eye, _) = camera.eye_and_target(&test_pose);
        assert_eq!(camera.eye_position(&test_pose), expected_eye);
    }

    #[test]
    fn inv_view_projection_is_actual_inverse() {
        let camera = ChaseCamera::new(1_600, 900);
        let test_pose = pose([0.75, 0.25, -0.35, 0.5]);
        let vp = camera.view_projection(&test_pose);
        let inv_vp = camera
            .inv_view_projection(&test_pose)
            .expect("non-singular");
        let product = vp * inv_vp;
        let identity = Mat4::identity();
        for (product_row, identity_row) in product.rows().iter().zip(identity.rows().iter()) {
            for (&p, &i) in product_row.iter().zip(identity_row.iter()) {
                assert!(
                    (p - i).abs() < 1.0e-4,
                    "product {product_row:?} != identity row {identity_row:?}"
                );
            }
        }
    }

    #[test]
    fn inv_view_projection_is_finite_for_all_test_attitudes() {
        let camera = ChaseCamera::new(1_280, 720);
        for &quat in &[
            [1.0, 0.0, 0.0, 0.0],
            [0.75, 0.25, -0.35, 0.5],
            [
                std::f64::consts::FRAC_1_SQRT_2,
                0.0,
                -std::f64::consts::FRAC_1_SQRT_2,
                0.0,
            ],
        ] {
            let test_pose = pose(quat);
            let inv = camera
                .inv_view_projection(&test_pose)
                .expect("non-singular VP");
            assert!(inv.is_finite(), "inv VP not finite for quat {quat:?}");
        }
    }

    // -----------------------------------------------------------------------
    // G1B atmosphere math tests
    // -----------------------------------------------------------------------

    #[test]
    fn elevation_zenith_is_one() {
        let elevation = view_elevation([0.0, 1.0, 0.0], RENDER_WORLD_UP);
        assert!((elevation - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn elevation_horizon_is_zero() {
        // Looking along any horizontal direction → elevation ≈ 0.
        for direction in &[[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]] {
            let elevation = view_elevation(*direction, RENDER_WORLD_UP);
            assert!(elevation.abs() < 1.0e-6, "expected ~0 for {direction:?}");
        }
    }

    #[test]
    fn elevation_below_horizon_is_negative() {
        let elevation = view_elevation([0.0, -1.0, 0.0], RENDER_WORLD_UP);
        assert!((elevation + 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn elevation_is_bounded() {
        // Arbitrary normalized direction.
        let dir = [0.577, 0.577, 0.577];
        let elevation = view_elevation(dir, RENDER_WORLD_UP);
        assert!((-1.0..=1.0).contains(&elevation));
    }

    #[test]
    fn fog_factor_zero_distance_is_zero() {
        assert!(exponential_fog_factor(0.0, 0.001).abs() < 1.0e-6);
    }

    #[test]
    fn fog_factor_zero_density_is_zero() {
        assert!(exponential_fog_factor(100.0, 0.0).abs() < 1.0e-6);
    }

    #[test]
    fn fog_factor_negative_distance_is_zero() {
        assert!(exponential_fog_factor(-10.0, 0.001).abs() < 1.0e-6);
    }

    #[test]
    fn fog_factor_monotonically_increases_with_distance() {
        let density = 0.002;
        let mut prev = 0.0_f32;
        for distance_in_meters in [10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0] {
            let fog = exponential_fog_factor(distance_in_meters, density);
            assert!(
                fog > prev,
                "fog should increase: {fog} <= {prev} at {distance_in_meters}"
            );
            assert!((0.0..=1.0).contains(&fog), "fog out of [0,1]: {fog}");
            assert!(fog.is_finite());
            prev = fog;
        }
    }

    #[test]
    fn fog_factor_large_distance_approaches_one() {
        let fog = exponential_fog_factor(100_000.0, 0.01);
        assert!(
            fog > 0.99,
            "expected near 1.0 for very large distance, got {fog}"
        );
    }

    #[test]
    fn fog_factor_all_outputs_finite_and_bounded() {
        for &density in &[0.0001, 0.001, 0.01, 0.1] {
            for &distance in &[0.0, 1.0, 10.0, 100.0, 1000.0, 10000.0] {
                let fog = exponential_fog_factor(distance, density);
                assert!(fog.is_finite());
                assert!((0.0..=1.0).contains(&fog));
            }
        }
    }

    #[test]
    fn sun_alignment_at_sun_is_maximum() {
        let sun_dir: [f32; 3] = [0.4, 0.8, -0.3];
        let len = (sun_dir[0].powi(2) + sun_dir[1].powi(2) + sun_dir[2].powi(2)).sqrt();
        let sun_normalized = [sun_dir[0] / len, sun_dir[1] / len, sun_dir[2] / len];
        let alignment = sun_alignment(sun_normalized, sun_normalized);
        assert!((alignment - 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn sun_alignment_opposite_direction_is_minimum() {
        let sun_dir: [f32; 3] = [0.4, 0.8, -0.3];
        let len = (sun_dir[0].powi(2) + sun_dir[1].powi(2) + sun_dir[2].powi(2)).sqrt();
        let sun_normalized = [sun_dir[0] / len, sun_dir[1] / len, sun_dir[2] / len];
        let opposite = [-sun_normalized[0], -sun_normalized[1], -sun_normalized[2]];
        let alignment = sun_alignment(opposite, sun_normalized);
        assert!((alignment + 1.0).abs() < 1.0e-5);
    }

    #[test]
    fn sun_alignment_perpendicular_is_zero() {
        let sun_dir = [1.0, 0.0, 0.0];
        let perpendicular = [0.0, 1.0, 0.0];
        let alignment = sun_alignment(perpendicular, sun_dir);
        assert!(alignment.abs() < 1.0e-6);
    }
}
