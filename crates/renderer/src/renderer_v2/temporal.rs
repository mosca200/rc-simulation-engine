//! CPU-only previous-presented state for the Rendering V2 temporal contract.
//!
//! This module deliberately owns no GPU resource and applies no camera jitter.
//! A candidate frame becomes history only after its surface texture has been
//! presented. Physics snapshots, replay state, and [`crate::RenderFrame`] stay
//! outside this presentation-only lifecycle.

use crate::{Mat4, camera::CameraFrameMatrices};

const JITTER_SEQUENCE_LENGTH: u64 = 8;
const CLIP_W_EPSILON: f32 = 1.0e-6;

/// Why the current temporal generation has no usable previous frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InvalidationReason {
    RendererCreated,
    Resize,
    SurfaceReconfigure,
}

/// One deterministic subpixel sample reserved for a future jittered camera.
///
/// Pixel X grows right and pixel Y grows down. `ndc_offset` therefore negates
/// Y when converting to WebGPU clip-space, where positive Y grows up.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct JitterSample {
    pixel_offset: [f32; 2],
    ndc_offset: [f32; 2],
}

impl JitterSample {
    fn for_presented_index(presented_frame_index: u64, extent: Option<[u32; 2]>) -> Self {
        let sequence_index = presented_frame_index % JITTER_SEQUENCE_LENGTH + 1;
        let pixel_offset = [
            halton(sequence_index, 2) - 0.5,
            halton(sequence_index, 3) - 0.5,
        ];
        let ndc_offset = extent.map_or([0.0; 2], |[width, height]| {
            debug_assert!(width > 0 && height > 0);
            [
                2.0 * pixel_offset[0] / width as f32,
                -2.0 * pixel_offset[1] / height as f32,
            ]
        });
        Self {
            pixel_offset,
            ndc_offset,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PresentedFrameState {
    aircraft_root: Mat4,
    camera: CameraFrameMatrices,
    jitter: JitterSample,
}

/// Immutable candidate used while encoding one render attempt.
///
/// Consuming this value in [`TemporalState::commit_presented`] is the only way
/// to advance history. Dropping it leaves every temporal counter unchanged.
#[derive(Debug)]
pub(crate) struct PreparedTemporalFrame {
    current: PresentedFrameState,
    previous: PresentedFrameState,
    history_was_valid: bool,
    presented_frame_index: u64,
    generation: u64,
}

impl PreparedTemporalFrame {
    pub(crate) const fn current_aircraft_root(&self) -> Mat4 {
        self.current.aircraft_root
    }

    pub(crate) const fn current_camera(&self) -> CameraFrameMatrices {
        self.current.camera
    }

    /// Debug-time validation for the staged CPU temporal contract.
    pub(crate) fn contract_is_finite(&self) -> bool {
        let states_are_finite = [self.current, self.previous].into_iter().all(|state| {
            state.aircraft_root.is_finite()
                && state.camera.view.is_finite()
                && state.camera.projection.is_finite()
                && state.camera.view_projection.is_finite()
                && state
                    .camera
                    .inverse_view_projection
                    .is_none_or(|inverse| inverse.is_finite())
                && state.eye_and_target_are_finite()
                && state.jitter.pixel_offset.into_iter().all(f32::is_finite)
                && state.jitter.ndc_offset.into_iter().all(f32::is_finite)
        });
        let invalid_history_has_zero_root_motion = self.history_was_valid
            || self
                .rigid_instance_motion_uv(Mat4::identity(), [0.0, 0.0, 0.0, 1.0])
                .is_none_or(|motion| motion.into_iter().all(|value| value.abs() <= f32::EPSILON));
        states_are_finite && invalid_history_has_zero_root_motion
    }

    /// Future rigid-GLB motion convention using one shared static instance
    /// transform for current and previous presentation states.
    #[must_use]
    pub(crate) fn rigid_instance_motion_uv(
        &self,
        static_instance: Mat4,
        local_position: [f32; 4],
    ) -> Option<[f32; 2]> {
        let current_world =
            (self.current.aircraft_root * static_instance).transform_homogeneous(local_position);
        let previous_world =
            (self.previous.aircraft_root * static_instance).transform_homogeneous(local_position);
        let current_clip = self
            .current
            .camera
            .view_projection
            .transform_homogeneous(current_world);
        let previous_clip = self
            .previous
            .camera
            .view_projection
            .transform_homogeneous(previous_world);
        motion_uv_from_clip(current_clip, previous_clip)
    }
}

impl PresentedFrameState {
    fn eye_and_target_are_finite(&self) -> bool {
        self.camera.eye.into_iter().all(f32::is_finite)
            && self.camera.target.into_iter().all(f32::is_finite)
    }
}

/// Fixed-size V2 presentation history. No member owns heap or GPU storage.
#[derive(Debug)]
pub(crate) struct TemporalState {
    history_valid: bool,
    presented_frame_index: u64,
    previous_presented: Option<PresentedFrameState>,
    extent: Option<[u32; 2]>,
    generation: u64,
    last_invalidation: InvalidationReason,
}

impl TemporalState {
    pub(crate) fn new(width: u32, height: u32) -> Self {
        Self {
            history_valid: false,
            presented_frame_index: 0,
            previous_presented: None,
            extent: valid_extent(width, height),
            generation: 0,
            last_invalidation: InvalidationReason::RendererCreated,
        }
    }

    /// Prepare, but do not commit, the state for one render attempt.
    pub(crate) fn prepare_frame(
        &self,
        aircraft_root: Mat4,
        camera: CameraFrameMatrices,
    ) -> PreparedTemporalFrame {
        debug_assert_eq!(self.history_valid, self.previous_presented.is_some());
        let current = PresentedFrameState {
            aircraft_root,
            camera,
            jitter: JitterSample::for_presented_index(self.presented_frame_index, self.extent),
        };
        let previous = self.previous_presented.unwrap_or(current);
        PreparedTemporalFrame {
            current,
            previous,
            history_was_valid: self.history_valid,
            presented_frame_index: self.presented_frame_index,
            generation: self.generation,
        }
    }

    /// Promote a candidate only after `queue.present(surface_texture)`.
    pub(crate) fn commit_presented(&mut self, prepared: PreparedTemporalFrame) {
        assert_eq!(prepared.generation, self.generation);
        assert_eq!(prepared.presented_frame_index, self.presented_frame_index);
        self.previous_presented = Some(prepared.current);
        self.history_valid = true;
        self.presented_frame_index = self.presented_frame_index.wrapping_add(1);
    }

    /// Track surface extent and invalidate when a valid render history can no
    /// longer describe the next surface presentation.
    pub(crate) fn resize(&mut self, width: u32, height: u32) {
        let next_extent = valid_extent(width, height);
        if next_extent != self.extent {
            self.extent = next_extent;
            self.invalidate(InvalidationReason::Resize);
        }
    }

    pub(crate) fn invalidate(&mut self, reason: InvalidationReason) {
        self.history_valid = false;
        self.presented_frame_index = 0;
        self.previous_presented = None;
        self.generation = self.generation.wrapping_add(1);
        self.last_invalidation = reason;
    }
}

fn valid_extent(width: u32, height: u32) -> Option<[u32; 2]> {
    (width > 0 && height > 0).then_some([width, height])
}

fn halton(mut index: u64, base: u64) -> f32 {
    debug_assert!(base >= 2);
    let mut value = 0.0_f64;
    let mut factor = 1.0_f64;
    while index > 0 {
        factor /= base as f64;
        value += factor * (index % base) as f64;
        index /= base;
    }
    value as f32
}

/// Future motion convention: current NDC minus previous NDC.
fn motion_ndc_from_clip(current_clip: [f32; 4], previous_clip: [f32; 4]) -> Option<[f32; 2]> {
    if !current_clip.into_iter().all(f32::is_finite)
        || !previous_clip.into_iter().all(f32::is_finite)
        || current_clip[3].abs() <= CLIP_W_EPSILON
        || previous_clip[3].abs() <= CLIP_W_EPSILON
    {
        return None;
    }
    let motion = [
        current_clip[0] / current_clip[3] - previous_clip[0] / previous_clip[3],
        current_clip[1] / current_clip[3] - previous_clip[1] / previous_clip[3],
    ];
    motion.into_iter().all(f32::is_finite).then_some(motion)
}

/// Convert NDC motion to top-left-origin texture UV motion.
fn motion_uv_from_clip(current_clip: [f32; 4], previous_clip: [f32; 4]) -> Option<[f32; 2]> {
    motion_ndc_from_clip(current_clip, previous_clip)
        .map(|motion| [0.5 * motion[0], -0.5 * motion[1]])
}

#[cfg(test)]
mod tests {
    use super::*;

    fn camera() -> CameraFrameMatrices {
        CameraFrameMatrices {
            view: Mat4::identity(),
            projection: Mat4::identity(),
            view_projection: Mat4::identity(),
            inverse_view_projection: Some(Mat4::identity()),
            eye: [0.0; 3],
            target: [0.0, 0.0, -1.0],
        }
    }

    fn translation(x: f32, y: f32, z: f32) -> Mat4 {
        Mat4::from_rows([
            [1.0, 0.0, 0.0, x],
            [0.0, 1.0, 0.0, y],
            [0.0, 0.0, 1.0, z],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    #[test]
    fn first_frame_uses_current_as_previous_and_commits_only_explicitly() {
        let mut state = TemporalState::new(1_280, 720);
        let prepared = state.prepare_frame(translation(0.25, 0.0, 0.0), camera());
        assert!(!prepared.history_was_valid);
        assert_eq!(
            prepared.rigid_instance_motion_uv(Mat4::identity(), [0.0, 0.0, 0.0, 1.0]),
            Some([0.0, -0.0])
        );
        assert_eq!(state.presented_frame_index, 0);
        assert!(!state.history_valid);

        state.commit_presented(prepared);
        assert_eq!(state.presented_frame_index, 1);
        assert!(state.history_valid);
    }

    #[test]
    fn dropping_prepared_frame_does_not_advance_history_or_jitter() {
        let state = TemporalState::new(800, 600);
        let first_jitter = state
            .prepare_frame(Mat4::identity(), camera())
            .current
            .jitter;
        let retry = state.prepare_frame(translation(1.0, 0.0, 0.0), camera());
        assert_eq!(retry.current.jitter, first_jitter);
        assert_eq!(retry.presented_frame_index, 0);
        assert!(!retry.history_was_valid);
    }

    #[test]
    fn commit_makes_last_presented_frame_the_next_previous() {
        let mut state = TemporalState::new(800, 600);
        let first = state.prepare_frame(Mat4::identity(), camera());
        state.commit_presented(first);
        let second = state.prepare_frame(translation(0.2, 0.0, 0.0), camera());
        assert!(second.history_was_valid);
        assert_eq!(
            second.rigid_instance_motion_uv(Mat4::identity(), [0.0, 0.0, 0.0, 1.0]),
            Some([0.1, -0.0])
        );
    }

    #[test]
    fn jitter_is_deterministic_bounded_and_repeats_after_eight_presentations() {
        let expected = [
            [0.0, -1.0 / 6.0],
            [-0.25, 1.0 / 6.0],
            [0.25, -7.0 / 18.0],
            [-0.375, -1.0 / 18.0],
            [0.125, 5.0 / 18.0],
            [-0.125, -5.0 / 18.0],
            [0.375, 1.0 / 18.0],
            [-0.4375, 7.0 / 18.0],
        ];
        for (index, expected_pixel) in expected.into_iter().enumerate() {
            let sample = JitterSample::for_presented_index(index as u64, Some([200, 100]));
            for (actual, expected) in sample.pixel_offset.into_iter().zip(expected_pixel) {
                assert!((actual - expected).abs() < 1.0e-6);
                assert!((-0.5..=0.5).contains(&actual));
            }
            assert_eq!(sample.ndc_offset[0], sample.pixel_offset[0] / 100.0);
            assert_eq!(sample.ndc_offset[1], -sample.pixel_offset[1] / 50.0);
            assert_eq!(
                sample,
                JitterSample::for_presented_index(index as u64 + 8, Some([200, 100]))
            );
        }
    }

    #[test]
    fn invalidation_clears_previous_and_restarts_sequence() {
        let mut state = TemporalState::new(800, 600);
        let first = state.prepare_frame(Mat4::identity(), camera());
        state.commit_presented(first);
        state.invalidate(InvalidationReason::SurfaceReconfigure);
        let after = state.prepare_frame(translation(3.0, 0.0, 0.0), camera());
        assert!(!after.history_was_valid);
        assert_eq!(after.presented_frame_index, 0);
        assert_eq!(
            state.last_invalidation,
            InvalidationReason::SurfaceReconfigure
        );
        assert_eq!(
            after.current.jitter,
            JitterSample::for_presented_index(0, Some([800, 600]))
        );
    }

    #[test]
    fn resize_invalidates_only_when_effective_extent_changes() {
        let mut state = TemporalState::new(800, 600);
        let first = state.prepare_frame(Mat4::identity(), camera());
        state.commit_presented(first);
        state.resize(800, 600);
        assert!(state.history_valid);
        state.resize(1_280, 720);
        assert!(!state.history_valid);
        assert_eq!(state.last_invalidation, InvalidationReason::Resize);

        let next = state.prepare_frame(Mat4::identity(), camera());
        assert_eq!(next.current.jitter.ndc_offset[0], 0.0);
        assert!((next.current.jitter.ndc_offset[1] - 1.0 / 2_160.0).abs() < 1.0e-9);
        state.resize(0, 0);
        assert_eq!(state.extent, None);
        assert!(!state.history_valid);
    }

    #[test]
    fn motion_convention_has_expected_x_and_top_left_uv_y_signs() {
        assert_eq!(
            motion_ndc_from_clip([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]),
            Some([0.0, 0.0])
        );
        assert_eq!(
            motion_uv_from_clip([0.2, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]),
            Some([0.1, -0.0])
        );
        assert_eq!(
            motion_uv_from_clip([0.0, 0.2, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]),
            Some([0.0, -0.1])
        );
        assert_eq!(
            motion_uv_from_clip([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            None
        );
    }

    #[test]
    fn rigid_instance_transform_is_shared_by_current_and_previous_roots() {
        let mut state = TemporalState::new(800, 600);
        let first = state.prepare_frame(translation(0.0, 0.0, 0.0), camera());
        state.commit_presented(first);
        let second = state.prepare_frame(translation(0.2, 0.0, 0.0), camera());
        let static_instance = translation(0.3, 0.0, 0.0);
        let motion = second
            .rigid_instance_motion_uv(static_instance, [0.0, 0.0, 0.0, 1.0])
            .unwrap();
        assert!((motion[0] - 0.1).abs() < 1.0e-6);
        assert_eq!(motion[1], -0.0);
        assert!(!std::mem::needs_drop::<TemporalState>());
    }

    #[test]
    fn gpu_frame_path_commits_only_after_present_and_v1_supplies_no_temporal_state() {
        let source = include_str!("../gpu.rs");
        let frame_path = source
            .split_once("fn render_scheduled(")
            .unwrap()
            .1
            .split_once("fn check_asynchronous_gpu_error")
            .unwrap()
            .0;
        let acquire = frame_path.find("acquire_surface_texture()?").unwrap();
        let prepare = frame_path.find("state.prepare_frame(").unwrap();
        let submit = frame_path
            .find(".submit(std::iter::once(encoder.finish()))")
            .unwrap();
        let present = frame_path
            .find("self.device_context.present(surface_texture);")
            .unwrap();
        let commit = frame_path
            .find("temporal.commit_presented(prepared);")
            .unwrap();
        assert!(acquire < prepare && prepare < submit && submit < present && present < commit);
        assert!(source.contains("self.render_scheduled(frame, None, None, None)"));
    }

    #[test]
    fn temporal_frame_path_creates_no_heap_or_gpu_resources() {
        let temporal_source = include_str!("temporal.rs");
        let temporal_production = temporal_source.split_once("#[cfg(test)]").unwrap().0;
        for forbidden in ["Vec<", "HashMap<", "Box<"] {
            assert!(
                !temporal_production.contains(forbidden),
                "TemporalState must remain fixed-size: {forbidden}"
            );
        }

        let gpu_source = include_str!("../gpu.rs");
        let frame_path = gpu_source
            .split_once("fn render_scheduled(")
            .unwrap()
            .1
            .split_once("fn check_asynchronous_gpu_error")
            .unwrap()
            .0;
        for forbidden in [
            ".create_buffer(",
            ".create_texture(",
            ".create_bind_group(",
            ".create_render_pipeline(",
        ] {
            assert!(
                !frame_path.contains(forbidden),
                "render path must not create persistent GPU resources: {forbidden}"
            );
        }
    }
}
