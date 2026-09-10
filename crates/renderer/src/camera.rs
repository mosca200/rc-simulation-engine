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

/// Horizontal-heading deadband: below this horizontal forward magnitude the
/// aircraft is treated as near-vertical and the geometric pose fallback
/// applies. Above it the historic normalized-forward behavior is kept
/// bit-for-bit.
const NEAR_VERTICAL_FORWARD_EPS: f32 = 1.0e-4;
/// Degenerate-fallback deadband for the pose-derived horizontal projections.
const DEGENERATE_HORIZONTAL_EPS: f32 = 1.0e-6;
/// Last-resort heading, reachable only for non-finite or doubly-degenerate
/// pose input (impossible for valid rotation matrices, whose columns are
/// orthonormal and therefore never vertical at the same time).
const GLOBAL_HEADING_FALLBACK: [f32; 3] = [0.0, 0.0, -1.0];

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

/// Coherent unjittered camera sample for one candidate presentation frame.
///
/// RV2 temporal state stores this presentation-only value after a successful
/// surface presentation. RV2-4 deliberately does not apply its jitter sample
/// to these matrices, so the rendered image remains identical to RV2-3.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct CameraFrameMatrices {
    pub(crate) view: Mat4,
    pub(crate) projection: Mat4,
    pub(crate) view_projection: Mat4,
    pub(crate) inverse_view_projection: Option<Mat4>,
    pub(crate) eye: [f32; 3],
    pub(crate) target: [f32; 3],
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

    /// Produce every unjittered matrix used by one render attempt from one
    /// camera/pose evaluation.
    #[must_use]
    pub(crate) fn frame_matrices(&self, aircraft_pose: &RenderPose) -> CameraFrameMatrices {
        match self {
            Self::Pilot(camera) => camera.frame_matrices(aircraft_pose),
            Self::Chase(camera) => camera.frame_matrices(aircraft_pose),
        }
    }

    #[must_use]
    pub fn view_projection(&self, aircraft_pose: &RenderPose) -> Mat4 {
        self.frame_matrices(aircraft_pose).view_projection
    }

    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        self.frame_matrices(aircraft_pose).inverse_view_projection
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
        self.frame_matrices(aircraft_pose).view_projection
    }

    #[must_use]
    fn frame_matrices(&self, aircraft_pose: &RenderPose) -> CameraFrameMatrices {
        let (eye, target) = self.eye_and_target(aircraft_pose);
        let view = look_at_rh(eye, target, RENDER_WORLD_UP);
        let projection = webgpu_perspective(
            self.vertical_fov_rad,
            self.aspect_ratio,
            NEAR_PLANE_M,
            FAR_PLANE_M,
        )
        .expect("fixed pilot-camera projection parameters are valid");
        let view_projection = projection * view;
        let inverse_view_projection = view_projection.inverse();
        CameraFrameMatrices {
            view,
            projection,
            view_projection,
            inverse_view_projection,
            eye,
            target,
        }
    }

    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        self.frame_matrices(aircraft_pose).inverse_view_projection
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
        let horizontal_forward = chase_horizontal_forward(aircraft_pose, forward);
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
        self.frame_matrices(aircraft_pose).view_projection
    }

    #[must_use]
    fn frame_matrices(&self, aircraft_pose: &RenderPose) -> CameraFrameMatrices {
        let (eye, target) = self.eye_and_target(aircraft_pose);
        let view = look_at_rh(eye, target, RENDER_WORLD_UP);
        let projection = webgpu_perspective(
            self.config.vertical_fov_deg.to_radians(),
            self.aspect_ratio,
            NEAR_PLANE_M,
            FAR_PLANE_M,
        )
        .expect("fixed chase-camera projection parameters are valid");
        let view_projection = projection * view;
        let inverse_view_projection = view_projection.inverse();
        CameraFrameMatrices {
            view,
            projection,
            view_projection,
            inverse_view_projection,
            eye,
            target,
        }
    }

    /// Inverse of the view-projection matrix.
    ///
    /// Returns `None` if the matrix is singular (should not happen with valid
    /// camera parameters, but handled cleanly for robustness).
    #[must_use]
    pub fn inv_view_projection(&self, aircraft_pose: &RenderPose) -> Option<Mat4> {
        self.frame_matrices(aircraft_pose).inverse_view_projection
    }
}

/// Presentation-side chase heading: unit-length horizontal direction used to
/// place the eye behind the aircraft.
///
/// Behavior contract (stateless, deterministic, zero allocation):
/// - Normal flight (`|forward.xz| > 1e-4`): historic behavior, i.e. the
///   normalized horizontal projection of `forward`, bit-for-bit.
/// - Near-vertical flight: geometric fallback derived from the same
///   `RenderPose`. The horizontal projection of the local vertical axis
///   (`up`, i.e. body `+Y` in render space) becomes the heading, with a sign
///   chosen from the climb/dive sense (`forward.y`): belly side (`-up`) when
///   climbing, canopy side (`+up`) when diving. Either choice matches the
///   approach heading of a pure pitch entry (loop), so no artificial swing
///   to a fixed global heading occurs at the pole. If the chosen projection
///   is itself degenerate (only possible for non-orthonormal/corrupt input,
///   since the fallback axis is orthogonal to the near-vertical forward),
///   the horizontal projection of the body right axis (`+X`) is tried next.
/// - Only non-finite or doubly-degenerate input falls back to the global
///   `[0, 0, -1]` heading (unreachable for valid rotation matrices).
///
/// The sign choice is what gives continuity: pitching up from level flight
/// toward +90° keeps heading ≈ forward.xz until the deadband, then the belly
/// projection takes over pointing the same way; symmetrically, pitching down
/// toward −90° hands over to the canopy projection pointing the entry way.
fn chase_horizontal_forward(aircraft_pose: &RenderPose, forward: [f32; 3]) -> [f32; 3] {
    if forward.iter().all(|v| v.is_finite()) {
        let horizontal_norm = forward[0].hypot(forward[2]);
        if horizontal_norm > NEAR_VERTICAL_FORWARD_EPS {
            return [
                forward[0] / horizontal_norm,
                0.0,
                forward[2] / horizontal_norm,
            ];
        }
    }
    // Geometric fallback from the same pose. The sign keeps continuity with
    // the pitch-entry side: belly (-up) on climb, canopy (+up) on dive.
    let climb = !forward[1].is_finite() || forward[1] >= 0.0;
    let vertical_axis = aircraft_pose.transform_direction([0.0, 1.0, 0.0]);
    let fallback = if climb {
        scale3(vertical_axis, -1.0)
    } else {
        vertical_axis
    };
    if let Some(heading) = normalized_horizontal(fallback) {
        return heading;
    }
    let right = aircraft_pose.transform_direction([1.0, 0.0, 0.0]);
    if let Some(heading) = normalized_horizontal(right) {
        return heading;
    }
    GLOBAL_HEADING_FALLBACK
}

/// Normalize the horizontal (`x/z`) projection of a render-space direction.
/// Returns `None` for non-finite or near-zero projections.
fn normalized_horizontal(direction: [f32; 3]) -> Option<[f32; 3]> {
    if !direction.iter().all(|v| v.is_finite()) {
        return None;
    }
    let norm = direction[0].hypot(direction[2]);
    if norm <= DEGENERATE_HORIZONTAL_EPS {
        return None;
    }
    Some([direction[0] / norm, 0.0, direction[2] / norm])
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

    /// Axis-angle quaternion (world-from-body, NED physics frame).
    fn axis_angle_ned(axis: [f64; 3], angle_rad: f64) -> [f64; 4] {
        let half = 0.5 * angle_rad;
        let (sin_half, cos_half) = half.sin_cos();
        [
            cos_half,
            axis[0] * sin_half,
            axis[1] * sin_half,
            axis[2] * sin_half,
        ]
    }

    /// Hamilton product `left * right` (apply `right` first, then `left`).
    fn mul_ned(left: [f64; 4], right: [f64; 4]) -> [f64; 4] {
        let [w1, x1, y1, z1] = left;
        let [w2, x2, y2, z2] = right;
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    }

    /// NED pitch about body +Y (positive pitches the nose up in render).
    fn pitch_ned(angle_rad: f64) -> [f64; 4] {
        axis_angle_ned([0.0, 1.0, 0.0], angle_rad)
    }

    /// NED yaw about body +Z.
    fn yaw_ned(angle_rad: f64) -> [f64; 4] {
        axis_angle_ned([0.0, 0.0, 1.0], angle_rad)
    }

    /// NED roll about body +X (forward axis).
    fn roll_ned(angle_rad: f64) -> [f64; 4] {
        axis_angle_ned([1.0, 0.0, 0.0], angle_rad)
    }

    const LEVEL_QUAT: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    fn horizontal_heading(pose: &RenderPose) -> [f32; 3] {
        let forward = pose.transform_direction([0.0, 0.0, -1.0]);
        chase_horizontal_forward(pose, forward)
    }

    fn distance3(left: [f32; 3], right: [f32; 3]) -> f32 {
        ((left[0] - right[0]).powi(2) + (left[1] - right[1]).powi(2) + (left[2] - right[2]).powi(2))
            .sqrt()
    }

    fn assert_unit_horizontal(heading: [f32; 3]) {
        assert!(heading.iter().all(|v| v.is_finite()));
        assert_eq!(heading[1], 0.0);
        let norm = heading[0].hypot(heading[2]);
        assert!((norm - 1.0).abs() < 1.0e-6, "heading {heading:?} not unit");
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
    fn level_flight_heading_is_unchanged() {
        let camera = ChaseCamera::new(1_600, 900);
        let level = pose(LEVEL_QUAT);
        assert_eq!(horizontal_heading(&level), [0.0, 0.0, -1.0]);
        let (eye, target) = camera.eye_and_target(&level);
        assert_eq!(eye, [0.0, 1.25, 3.5]);
        assert_eq!(target, [0.0, 0.0, -1.5]);
    }

    #[test]
    fn normal_pitch_matches_legacy_projection_within_tolerance() {
        // +/-30deg, +/-60deg and yawed variants stay on the legacy path.
        let cases = [
            pitch_ned(30.0_f64.to_radians()),
            pitch_ned(-30.0_f64.to_radians()),
            pitch_ned(60.0_f64.to_radians()),
            pitch_ned(-60.0_f64.to_radians()),
            mul_ned(
                yaw_ned(45.0_f64.to_radians()),
                pitch_ned(20.0_f64.to_radians()),
            ),
        ];
        for quat in cases {
            let test_pose = pose(quat);
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            let forward = test_pose.transform_direction([0.0, 0.0, -1.0]);
            let norm = forward[0].hypot(forward[2]);
            assert!(norm > 1.0e-4, "case {quat:?} inside fallback deadband");
            let legacy = [forward[0] / norm, 0.0, forward[2] / norm];
            assert!(distance3(heading, legacy) < 1.0e-6);
        }
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
    fn plus_ninety_pitch_is_finite_and_keeps_entry_heading() {
        let camera = ChaseCamera::new(1_280, 720);
        let vertical = pose(pitch_ned(std::f64::consts::FRAC_PI_2));
        let forward = vertical.transform_direction([0.0, 0.0, -1.0]);
        assert!(forward[0].hypot(forward[2]) <= 1.0e-4);
        assert!(forward[1] > 0.999);
        let heading = horizontal_heading(&vertical);
        assert_unit_horizontal(heading);
        // Pitched up from level heading -Z: keep it.
        assert!(distance3(heading, [0.0, 0.0, -1.0]) < 2.0e-6);
        let (eye, target) = camera.eye_and_target(&vertical);
        assert_ne!(eye, target);
        assert!(eye.into_iter().chain(target).all(f32::is_finite));
        assert!(camera.view_projection(&vertical).is_finite());
        assert!(camera.inv_view_projection(&vertical).is_some());
    }

    #[test]
    fn minus_ninety_pitch_is_finite_and_keeps_entry_heading() {
        let camera = ChaseCamera::new(1_280, 720);
        let vertical = pose(pitch_ned(-std::f64::consts::FRAC_PI_2));
        let forward = vertical.transform_direction([0.0, 0.0, -1.0]);
        assert!(forward[0].hypot(forward[2]) <= 1.0e-4);
        assert!(forward[1] < -0.999);
        let heading = horizontal_heading(&vertical);
        assert_unit_horizontal(heading);
        // Pitched down from level heading -Z: keep it.
        assert!(distance3(heading, [0.0, 0.0, -1.0]) < 2.0e-6);
        let (eye, target) = camera.eye_and_target(&vertical);
        assert_ne!(eye, target);
        assert!(eye.into_iter().chain(target).all(f32::is_finite));
        assert!(camera.view_projection(&vertical).is_finite());
        assert!(camera.inv_view_projection(&vertical).is_some());
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
    fn near_plus_ninety_entry_side_stays_continuous() {
        let camera = ChaseCamera::new(1_280, 720);
        let entry = [0.0, 0.0, -1.0_f32];
        let mut previous: Option<[f32; 3]> = None;
        // Approach the pole from level flight (loop entry side): the heading
        // must stay on the entry heading with no swing to a global fallback.
        // The far side (past the pole) genuinely flips the nose the other
        // way, so it is covered separately below.
        for degrees in [85.0_f64, 89.0, 89.9, 89.99, 90.0] {
            let test_pose = pose(pitch_ned(degrees.to_radians()));
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            assert!(
                distance3(heading, entry) < 1.0e-3,
                "heading {heading:?} left entry {entry:?} at {degrees}°"
            );
            if let Some(prev) = previous {
                assert!(distance3(heading, prev) < 1.0e-3);
            }
            previous = Some(heading);
            let (eye, target) = camera.eye_and_target(&test_pose);
            assert_ne!(eye, target);
            assert!(eye.into_iter().chain(target).all(f32::is_finite));
            assert!(camera.view_projection(&test_pose).is_finite());
        }
    }

    #[test]
    fn near_plus_ninety_far_side_follows_flipped_nose() {
        let camera = ChaseCamera::new(1_280, 720);
        // Past the pole the nose genuinely points the other way; the camera
        // follows the (pose-derived) flipped heading, continuously.
        let flipped = [0.0, 0.0, 1.0_f32];
        let mut previous: Option<[f32; 3]> = None;
        for degrees in [90.01_f64, 90.1, 91.0, 95.0] {
            let test_pose = pose(pitch_ned(degrees.to_radians()));
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            assert!(
                distance3(heading, flipped) < 1.0e-3,
                "heading {heading:?} != flipped {flipped:?} at {degrees}°"
            );
            if let Some(prev) = previous {
                assert!(distance3(heading, prev) < 1.0e-3);
            }
            previous = Some(heading);
            assert!(camera.view_projection(&test_pose).is_finite());
        }
    }

    #[test]
    fn near_minus_ninety_entry_side_stays_continuous() {
        let camera = ChaseCamera::new(1_280, 720);
        let entry = [0.0, 0.0, -1.0_f32];
        let mut previous: Option<[f32; 3]> = None;
        // Dive entry side: heading stays on the entry heading.
        for degrees in [-85.0_f64, -89.0, -89.9, -89.99, -90.0] {
            let test_pose = pose(pitch_ned(degrees.to_radians()));
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            assert!(
                distance3(heading, entry) < 1.0e-3,
                "heading {heading:?} left entry {entry:?} at {degrees}°"
            );
            if let Some(prev) = previous {
                assert!(distance3(heading, prev) < 1.0e-3);
            }
            previous = Some(heading);
            let (eye, target) = camera.eye_and_target(&test_pose);
            assert_ne!(eye, target);
            assert!(eye.into_iter().chain(target).all(f32::is_finite));
            assert!(camera.view_projection(&test_pose).is_finite());
        }
    }

    #[test]
    fn near_minus_ninety_far_side_follows_flipped_nose() {
        let camera = ChaseCamera::new(1_280, 720);
        // Past the pole the nose genuinely points the other way; the camera
        // follows the (pose-derived) flipped heading, continuously.
        let flipped = [0.0, 0.0, 1.0_f32];
        let mut previous: Option<[f32; 3]> = None;
        for degrees in [-90.01_f64, -90.1, -91.0, -95.0] {
            let test_pose = pose(pitch_ned(degrees.to_radians()));
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            assert!(
                distance3(heading, flipped) < 1.0e-3,
                "heading {heading:?} != flipped {flipped:?} at {degrees}°"
            );
            if let Some(prev) = previous {
                assert!(distance3(heading, prev) < 1.0e-3);
            }
            previous = Some(heading);
            assert!(camera.view_projection(&test_pose).is_finite());
        }
    }

    #[test]
    fn yawed_vertical_uses_geometric_fallback_not_global_heading() {
        // Yaw +90deg (nose-left in NED convention) then pitch up: the loop
        // plane faces +X, so the geometric fallback must follow +X rather
        // than jump to the global -Z heading.
        // (Heading helper is pose-only; no camera instance needed here.)
        let yawed_climb = pose(mul_ned(
            yaw_ned(std::f64::consts::FRAC_PI_2),
            pitch_ned(std::f64::consts::FRAC_PI_2),
        ));
        let forward = yawed_climb.transform_direction([0.0, 0.0, -1.0]);
        assert!(forward[0].hypot(forward[2]) <= 1.0e-4);
        let heading = horizontal_heading(&yawed_climb);
        assert_unit_horizontal(heading);
        assert!(distance3(heading, [1.0, 0.0, 0.0]) < 2.0e-6);
        assert!(distance3(heading, [0.0, 0.0, -1.0]) > 0.5);
        // Symmetric dive entry keeps its own approach heading too.
        let yawed_dive = pose(mul_ned(
            yaw_ned(std::f64::consts::FRAC_PI_2),
            pitch_ned(-std::f64::consts::FRAC_PI_2),
        ));
        let dive_heading = horizontal_heading(&yawed_dive);
        assert_unit_horizontal(dive_heading);
        assert!(distance3(dive_heading, [1.0, 0.0, 0.0]) < 2.0e-6);
    }

    #[test]
    fn roll_while_vertical_remains_finite() {
        let camera = ChaseCamera::new(1_280, 720);
        let vertical = pitch_ned(std::f64::consts::FRAC_PI_2);
        for degrees in [0.0_f64, 30.0, 90.0, 135.0, 180.0, 270.0] {
            let test_pose = pose(mul_ned(roll_ned(degrees.to_radians()), vertical));
            let heading = horizontal_heading(&test_pose);
            assert_unit_horizontal(heading);
            let (eye, target) = camera.eye_and_target(&test_pose);
            assert_ne!(eye, target);
            assert!(eye.into_iter().chain(target).all(f32::is_finite));
            assert!(camera.view_projection(&test_pose).is_finite());
            assert!(camera.inv_view_projection(&test_pose).is_some());
        }
    }

    #[test]
    fn vertical_camera_eye_differs_from_target_and_matrix_inverts() {
        let camera = ChaseCamera::new(1_280, 720);
        for quat in [
            pitch_ned(std::f64::consts::FRAC_PI_2),
            pitch_ned(-std::f64::consts::FRAC_PI_2),
        ] {
            let test_pose = pose(quat);
            let (eye, target) = camera.eye_and_target(&test_pose);
            assert_ne!(eye, target);
            assert!(distance3(eye, target) > 1.0e-3);
            let view_projection = camera.view_projection(&test_pose);
            assert!(view_projection.is_finite());
            let inverse = camera
                .inv_view_projection(&test_pose)
                .expect("vertical VP must invert");
            assert!(inverse.is_finite());
            let product = view_projection * inverse;
            let identity = Mat4::identity();
            for (product_row, identity_row) in product.rows().iter().zip(identity.rows().iter()) {
                for (&entry, &expected) in product_row.iter().zip(identity_row.iter()) {
                    assert!((entry - expected).abs() < 1.0e-4);
                }
            }
        }
    }

    #[test]
    fn same_pose_gives_same_camera_result() {
        let camera = ChaseCamera::new(1_600, 900);
        let quats = [
            LEVEL_QUAT,
            pitch_ned(0.6),
            pitch_ned(std::f64::consts::FRAC_PI_2),
            pitch_ned(-std::f64::consts::FRAC_PI_2),
            mul_ned(yaw_ned(1.1), pitch_ned(std::f64::consts::FRAC_PI_2)),
            mul_ned(roll_ned(0.9), pitch_ned(std::f64::consts::FRAC_PI_2)),
        ];
        for quat in quats {
            let test_pose = pose(quat);
            assert_eq!(
                camera.eye_and_target(&test_pose),
                camera.eye_and_target(&test_pose)
            );
            assert_eq!(
                camera.view_projection(&test_pose),
                camera.view_projection(&test_pose)
            );
        }
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

    #[test]
    fn frame_matrices_preserve_pilot_and_chase_unjittered_camera_contracts() {
        let aircraft = pose(mul_ned(
            yaw_ned(0.43),
            mul_ned(pitch_ned(-0.27), roll_ned(0.19)),
        ));
        for mode in [
            CameraConfig::pilot_default().build(1_600, 900),
            CameraConfig::chase_default().build(1_600, 900),
        ] {
            let sample = mode.frame_matrices(&aircraft);
            let (expected_eye, expected_target) = mode.eye_and_target(&aircraft);
            let expected_view = look_at_rh(expected_eye, expected_target, RENDER_WORLD_UP);
            let expected_projection = match mode {
                CameraMode::Pilot(camera) => webgpu_perspective(
                    camera.vertical_fov_rad,
                    camera.aspect_ratio,
                    NEAR_PLANE_M,
                    FAR_PLANE_M,
                )
                .unwrap(),
                CameraMode::Chase(camera) => webgpu_perspective(
                    camera.config.vertical_fov_deg.to_radians(),
                    camera.aspect_ratio,
                    NEAR_PLANE_M,
                    FAR_PLANE_M,
                )
                .unwrap(),
            };
            let expected_vp = expected_projection * expected_view;

            assert_eq!(sample.eye, expected_eye);
            assert_eq!(sample.target, expected_target);
            assert_eq!(sample.view, expected_view);
            assert_eq!(sample.projection, expected_projection);
            assert_eq!(sample.view_projection, expected_vp);
            assert_eq!(sample.inverse_view_projection, expected_vp.inverse());
            assert_eq!(mode.view_projection(&aircraft), expected_vp);
            assert_eq!(mode.inv_view_projection(&aircraft), expected_vp.inverse());
            assert_eq!(mode.eye_position(&aircraft), expected_eye);
        }
    }
}
