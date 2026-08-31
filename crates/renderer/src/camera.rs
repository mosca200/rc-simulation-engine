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

    #[must_use]
    pub fn view_projection(&self, aircraft_pose: &RenderPose) -> Mat4 {
        let (eye, target) = self.eye_and_target(aircraft_pose);
        let view = look_at_rh(eye, target, [0.0, 1.0, 0.0]);
        let projection = webgpu_perspective(
            VERTICAL_FOV_RAD,
            self.aspect_ratio,
            NEAR_PLANE_M,
            FAR_PLANE_M,
        )
        .expect("fixed chase-camera projection parameters are valid");
        projection * view
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
}
