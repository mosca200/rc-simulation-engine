use crate::{
    Mat4, RenderPose,
    math::{add3, look_at_rh, scale3, sub3},
    webgpu_perspective,
};

const DEFAULT_VERTICAL_FOV_RAD: f32 = 55.0_f32.to_radians();
const NEAR_PLANE_M: f32 = 0.05;
const FAR_PLANE_M: f32 = 5_000.0;
const DEFAULT_DISTANCE_BEHIND_M: f32 = 3.5;
const DEFAULT_HEIGHT_ABOVE_M: f32 = 1.25;
const DEFAULT_LOOK_AHEAD_M: f32 = 1.5;
const DEFAULT_PILOT_POSITION_RENDER_M: [f32; 3] = [0.0, 1.8, 20.0];

/// Render-space world-up direction: +Y is up.
///
/// The NED-to-render mapping sends physics Down (NED +Z) to render −Y,
/// so physics Up (NED −Z) maps to render +Y.
pub const RENDER_WORLD_UP: [f32; 3] = [0.0, 1.0, 0.0];

// ---------------------------------------------------------------------------
// Camera configuration (presentation-side only; never part of the physics
// fingerprint).
// ---------------------------------------------------------------------------

/// Presentation-side RC camera configuration.
///
/// All values live purely in render presentation space. Nothing here feeds
/// back into physics, and none of these fields appear in the physics
/// fingerprint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraConfig {
    /// Fixed RC pilot position that automatically looks at the aircraft.
    Pilot {
        position_render_m: [f32; 3],
        /// Vertical field of view in degrees. Narrower FOV is a camera-only
        /// zoom (never scales the aircraft or fakes distance).
        vertical_fov_deg: f32,
    },
    /// Conventional chase camera following the aircraft from behind/above.
    Chase {
        distance_behind_m: f32,
        height_above_m: f32,
        look_ahead_m: f32,
        vertical_fov_deg: f32,
    },
}

impl CameraConfig {
    /// Default pilot camera: fixed point near the flight field.
    #[must_use]
    pub fn pilot_default() -> Self {
        Self::Pilot {
            position_render_m: DEFAULT_PILOT_POSITION_RENDER_M,
            vertical_fov_deg: DEFAULT_VERTICAL_FOV_RAD.to_degrees(),
        }
    }

    /// Default chase camera matching the historic chase behavior.
    #[must_use]
    pub fn chase_default() -> Self {
        Self::Chase {
            distance_behind_m: DEFAULT_DISTANCE_BEHIND_M,
            height_above_m: DEFAULT_HEIGHT_ABOVE_M,
            look_ahead_m: DEFAULT_LOOK_AHEAD_M,
            vertical_fov_deg: DEFAULT_VERTICAL_FOV_RAD.to_degrees(),
        }
    }

    /// Build the concrete camera for a given render surface size.
    #[must_use]
    pub fn build(self, width: u32, height: u32) -> CameraMode {
        match self {
            Self::Pilot {
                position_render_m,
                vertical_fov_deg,
            } => CameraMode::Pilot(PilotCamera::new(
                width,
                height,
                position_render_m,
                vertical_fov_deg,
            )),
            Self::Chase {
                distance_behind_m,
                height_above_m,
                look_ahead_m,
                vertical_fov_deg,
            } => CameraMode::Chase(ChaseCamera::new_with_config(
                width,
                height,
                ChaseCameraConfig {
                    distance_behind_m,
                    height_above_m,
                    look_ahead_m,
                    vertical_fov_deg,
                },
            )),
        }
    }
}

/// Active RC camera mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CameraMode {
    Pilot(PilotCamera),
    Chase(ChaseCamera),
}

impl CameraMode {
    pub fn resize(&mut self, width: u32, height: u32) {
        match self {
            Self::Pilot(camera) => camera.resize(width, height),
            Self::Chase(camera) => camera.resize(width, height),
        }
    }

    #[must_use]
    pub fn aspect_ratio(&self) -> f32 {
        match self {
            Self::Pilot(camera) => camera.aspect_ratio(),
            Self::Chase(camera) => camera.aspect_ratio(),
        }
    }

    #[must_use]
    pub fn eye_and_target(&self, aircraft_pose: &RenderPose) -> ([f32; 3], [f32; 3]) {
        match self {
            Self::Pilot(camera) => camera.eye_and_target(aircraft_pose),
            Self::Chase(camera) => camera.eye_and_target(aircraft_pose),
        }
    }

    #[must_use]
    pub fn eye_position(&self, aircraft_pose: &RenderPose) -> [f32; 3] {
        match self {
            Self::Pilot(camera) => camera.eye_position(aircraft_pose),
            Self::Chase(camera) => camera.eye_position(aircraft_pose),
        }
    }

    #[must_use]
    pub fn view_projection(&self, aircraft_pose: &RenderPose) -> Mat4 {
        match self {
            Self::Pilot(camera) => camera.view_projection(aircraft_pose),
            Self::Chase(camera) => camera.view_projection(aircraft_pose),
        }
    }

    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        match self {
            Self::Pilot(camera) => camera.inv_view_projection(aircraft_pose),
            Self::Chase(camera) => camera.inv_view_projection(aircraft_pose),
        }
    }
}

// ---------------------------------------------------------------------------
// Pilot camera
// ---------------------------------------------------------------------------

/// Fixed RC pilot camera.
///
/// The eye position never moves; the camera always looks at the current
/// aircraft render position. The horizon stays stable because the world-up
/// vector is fixed and the pilot point is constant.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PilotCamera {
    aspect_ratio: f32,
    position_render_m: [f32; 3],
    vertical_fov_rad: f32,
}

impl PilotCamera {
    #[must_use]
    pub fn new(
        width: u32,
        height: u32,
        position_render_m: [f32; 3],
        vertical_fov_deg: f32,
    ) -> Self {
        Self {
            aspect_ratio: valid_aspect_ratio(width, height).unwrap_or(1.0),
            position_render_m,
            vertical_fov_rad: vertical_fov_deg.to_radians(),
        }
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

    /// The pilot eye is always the fixed configured position.
    #[must_use]
    pub fn eye_position(&self, _aircraft_pose: &RenderPose) -> [f32; 3] {
        self.position_render_m
    }

    /// Fixed eye; target is the aircraft render position.
    #[must_use]
    pub fn eye_and_target(&self, aircraft_pose: &RenderPose) -> ([f32; 3], [f32; 3]) {
        (self.position_render_m, aircraft_pose.translation_render_m())
    }

    #[must_use]
    pub fn view_projection(&self, aircraft_pose: &RenderPose) -> Mat4 {
        let (eye, target) = self.eye_and_target(aircraft_pose);
        let view = look_at_rh(eye, target, RENDER_WORLD_UP);
        let projection = webgpu_perspective(
            self.vertical_fov_rad,
            self.aspect_ratio,
            NEAR_PLANE_M,
            FAR_PLANE_M,
        )
        .expect("fixed pilot-camera projection parameters are valid");
        projection * view
    }

    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        self.view_projection(aircraft_pose).inverse()
    }
}

// ---------------------------------------------------------------------------
// Chase camera
// ---------------------------------------------------------------------------

/// Tunable chase-camera parameters (presentation-only).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChaseCameraConfig {
    pub distance_behind_m: f32,
    pub height_above_m: f32,
    pub look_ahead_m: f32,
    pub vertical_fov_deg: f32,
}

impl Default for ChaseCameraConfig {
    fn default() -> Self {
        Self {
            distance_behind_m: DEFAULT_DISTANCE_BEHIND_M,
            height_above_m: DEFAULT_HEIGHT_ABOVE_M,
            look_ahead_m: DEFAULT_LOOK_AHEAD_M,
            vertical_fov_deg: DEFAULT_VERTICAL_FOV_RAD.to_degrees(),
        }
    }
}

/// Stable world-up chase camera driven only by a render pose.
///
/// The eye is derived directly from the physics pose each frame, so tracking
/// is exactly smooth (no artificial lag), fully deterministic, and never
/// feeds back into physics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChaseCamera {
    aspect_ratio: f32,
    config: ChaseCameraConfig,
}

impl ChaseCamera {
    /// Backward-compatible constructor with default tuning.
    #[must_use]
    pub fn new(width: u32, height: u32) -> Self {
        Self::new_with_config(width, height, ChaseCameraConfig::default())
    }

    #[must_use]
    pub fn new_with_config(width: u32, height: u32, config: ChaseCameraConfig) -> Self {
        Self {
            aspect_ratio: valid_aspect_ratio(width, height).unwrap_or(1.0),
            config,
        }
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
    pub const fn config(&self) -> &ChaseCameraConfig {
        &self.config
    }

    #[must_use]
    pub fn eye_and_target(&self, aircraft_pose: &RenderPose) -> ([f32; 3], [f32; 3]) {
        let position = aircraft_pose.translation_render_m();
        let forward = aircraft_pose.transform_direction([0.0, 0.0, -1.0]);
        let horizontal_forward = normalized_horizontal_forward(forward);
        let eye = add3(
            sub3(
                position,
                scale3(horizontal_forward, self.config.distance_behind_m),
            ),
            [0.0, self.config.height_above_m, 0.0],
        );
        let target = add3(position, scale3(forward, self.config.look_ahead_m));
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
            self.config.vertical_fov_deg.to_radians(),
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

    fn translated_pose(translation_ned: [f64; 3], quaternion: [f64; 4]) -> RenderPose {
        world_ned_pose_to_render(translation_ned, quaternion, [0.0; 3]).unwrap()
    }

    // -------------------------------------------------------------------
    // Chase camera
    // -------------------------------------------------------------------

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
    fn chase_config_is_presentation_only_and_tunable() {
        let camera = ChaseCamera::new_with_config(
            1_600,
            900,
            ChaseCameraConfig {
                distance_behind_m: 8.0,
                height_above_m: 3.0,
                look_ahead_m: 4.0,
                vertical_fov_deg: 40.0,
            },
        );
        let (eye, target) = camera.eye_and_target(&pose([1.0, 0.0, 0.0, 0.0]));
        assert_eq!(eye, [0.0, 3.0, 8.0]);
        assert_eq!(target, [0.0, 0.0, -4.0]);
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

    // -------------------------------------------------------------------
    // Pilot camera
    // -------------------------------------------------------------------

    #[test]
    fn pilot_position_remains_fixed_regardless_of_aircraft() {
        let camera = PilotCamera::new(1_600, 900, [10.0, 1.8, 25.0], 55.0);
        let here = pose([1.0, 0.0, 0.0, 0.0]);
        let away = translated_pose([500.0, 200.0, 100.0], [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(camera.eye_position(&here), [10.0, 1.8, 25.0]);
        assert_eq!(camera.eye_position(&away), [10.0, 1.8, 25.0]);
    }

    #[test]
    fn pilot_camera_points_at_aircraft() {
        let camera = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 55.0);
        let aircraft = translated_pose([50.0, 30.0, -12.0], [1.0, 0.0, 0.0, 0.0]);
        let (eye, target) = camera.eye_and_target(&aircraft);
        assert_eq!(eye, [0.0, 1.8, 20.0]);
        assert_eq!(target, aircraft.translation_render_m());
        assert!(camera.view_projection(&aircraft).is_finite());
    }

    #[test]
    fn pilot_camera_handles_degenerate_and_extreme_cases_without_nan() {
        let camera = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 55.0);
        // Aircraft exactly at the pilot position → look_at degenerates, but
        // the result must stay finite (look_at_rh guards the singular case).
        let at_pilot = translated_pose([0.0, 1.8, 20.0], [1.0, 0.0, 0.0, 0.0]);
        assert!(camera.view_projection(&at_pilot).is_finite());
        // Aircraft far away and vertically above.
        let far_overhead = translated_pose([10_000.0, 5_000.0, -8_000.0], [1.0, 0.0, 0.0, 0.0]);
        let vp = camera.view_projection(&far_overhead);
        assert!(vp.is_finite());
        // Aircraft directly overhead of the pilot.
        let overhead = translated_pose([0.0, 1.0, 20.0], [1.0, 0.0, 0.0, 0.0]);
        assert!(camera.view_projection(&overhead).is_finite());
    }

    #[test]
    fn pilot_camera_narrower_fov_zooms_without_moving_positions() {
        let wide = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 55.0);
        let zoomed = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 30.0);
        let aircraft = translated_pose([30.0, 5.0, 0.0], [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(wide.eye_position(&aircraft), zoomed.eye_position(&aircraft));
        assert_eq!(
            wide.eye_and_target(&aircraft).1,
            zoomed.eye_and_target(&aircraft).1
        );
    }

    #[test]
    fn pilot_camera_transform_is_deterministic() {
        let camera = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 55.0);
        let aircraft = translated_pose([40.0, 12.0, -5.0], [0.75, 0.25, -0.35, 0.5]);
        assert_eq!(
            camera.view_projection(&aircraft),
            camera.view_projection(&aircraft)
        );
    }

    #[test]
    fn pilot_camera_handles_render_origin_translation() {
        // The camera operates in render space; a non-zero render origin must
        // be reflected in the target the camera looks at.
        let camera = PilotCamera::new(1_600, 900, [0.0, 1.8, 20.0], 55.0);
        let origin = [100.0, 0.0, -200.0];
        let aircraft =
            world_ned_pose_to_render([50.0, 30.0, -12.0], [1.0, 0.0, 0.0, 0.0], origin).unwrap();
        let (_, target) = camera.eye_and_target(&aircraft);
        assert_eq!(target, aircraft.translation_render_m());
        assert!(target.iter().all(|v| v.is_finite()));
    }

    // -------------------------------------------------------------------
    // CameraMode union
    // -------------------------------------------------------------------

    #[test]
    fn camera_mode_builds_both_variants_and_is_finite() {
        let pilot_mode = CameraConfig::pilot_default().build(1_600, 900);
        let chase_mode = CameraConfig::chase_default().build(1_600, 900);
        let aircraft = pose([1.0, 0.0, 0.0, 0.0]);
        assert!(pilot_mode.view_projection(&aircraft).is_finite());
        assert!(chase_mode.view_projection(&aircraft).is_finite());
        assert!(
            pilot_mode.view_projection(&aircraft).is_finite()
                && chase_mode.view_projection(&aircraft).is_finite()
        );
    }

    #[test]
    fn camera_mode_resize_updates_aspect() {
        let mut mode = CameraConfig::chase_default().build(800, 600);
        assert!((mode.aspect_ratio() - 4.0 / 3.0).abs() < f32::EPSILON);
        mode.resize(1_920, 1_080);
        assert!((mode.aspect_ratio() - 16.0 / 9.0).abs() < f32::EPSILON);
    }

    #[test]
    fn camera_settings_do_not_touch_physics_fingerprint_path() {
        // Camera configs are plain presentation data; assert no NaN and that
        // both defaults build distinct modes deterministically.
        let pilot = CameraConfig::pilot_default();
        let chase = CameraConfig::chase_default();
        assert_ne!(pilot, chase);
        assert!(matches!(pilot, CameraConfig::Pilot { .. }));
        assert!(matches!(chase, CameraConfig::Chase { .. }));
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
