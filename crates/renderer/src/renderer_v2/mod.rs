//! RV2-1 Rendering V2 parity shell.
//!
//! This module is intentionally the *minimum* V2 surface required by RV2-1. It
//! is a parity shell: every presentation operation delegates straight to the
//! existing V1 [`WgpuRenderer`], so `--renderer v2` produces exactly the same
//! [`RenderFrame`] presentation as `--renderer v1` with zero intentional
//! visual change.
//!
//! Later RV2 slices will progressively replace the internals of
//! [`RendererV2Shell`] (render graph, GPU scene, atmosphere, temporal, ...)
//! without changing this module's contract with the shared
//! [`crate::backend::DesktopRenderer`] facade, and without the application
//! ever learning how V2 is implemented internally.
//!
//! RV2-1 deliberately does NOT fork `gpu.rs`, duplicate shaders, or introduce
//! a framework abstraction. The shell owns one V1 renderer and forwards calls.

use std::sync::Arc;

use winit::window::Window;

use crate::{
    CameraConfig, ExposureError, PresentationAsset, RenderFrame, RenderTerrainMode, RendererError,
    SurfaceError, TerrainDebugMode, VegetationDebugMode, WgpuRenderer, scenery::SceneryPreset,
};

/// Parity shell for the Rendering V2 backend.
///
/// In RV2-1 this wraps a V1 [`WgpuRenderer`] and delegates every call. The
/// wrapper exists so the shared facade can dispatch to a distinct V2 backend
/// while the rendered output stays identical to V1.
pub struct RendererV2Shell {
    inner: WgpuRenderer,
}

impl RendererV2Shell {
    /// Construct the parity shell with a presentation asset and optional
    /// scenery, mirroring [`WgpuRenderer::new_with_presentation`] exactly.
    ///
    /// # Errors
    ///
    /// Returns the same [`RendererError`] values as the V1 constructor, since
    /// RV2-1 delegates GPU/surface creation to V1 unchanged.
    pub async fn new_with_presentation(
        window: Arc<Window>,
        asset: PresentationAsset<'_>,
        ground_below_render_origin_m: f32,
        terrain_mode: RenderTerrainMode,
        scenery_preset: Option<SceneryPreset>,
        camera_config: CameraConfig,
    ) -> Result<Self, RendererError> {
        let inner = WgpuRenderer::new_with_presentation(
            window,
            asset,
            ground_below_render_origin_m,
            terrain_mode,
            scenery_preset,
            camera_config,
        )
        .await?;
        Ok(Self { inner })
    }

    /// Present a shared [`RenderFrame`], delegating to the V1 backend.
    ///
    /// # Errors
    ///
    /// Propagates the V1 [`SurfaceError`] unchanged so the application's
    /// surface-event policy (lost / outdated / timeout / out-of-memory /
    /// validation) behaves identically for V1 and V2.
    pub fn render(&mut self, frame: &RenderFrame) -> Result<(), SurfaceError> {
        self.inner.render(frame)
    }

    /// Resize the presentation surface, delegating to the V1 backend.
    pub fn resize(&mut self, width: u32, height: u32) {
        self.inner.resize(width, height);
    }

    /// Recreate the surface after a lost/outdated event, delegating to V1.
    pub fn reconfigure_surface(&mut self) {
        self.inner.reconfigure_surface();
    }

    /// Toggle the presentation-only debug overlays, delegating to V1.
    pub fn set_show_debug_overlays(&mut self, show: bool) {
        self.inner.set_show_debug_overlays(show);
    }

    /// Select the presentation-only terrain debug channel, delegating to V1.
    pub fn set_terrain_debug_mode(&mut self, mode: TerrainDebugMode) {
        self.inner.set_terrain_debug_mode(mode);
    }

    /// Select the presentation-only vegetation debug channel, delegating to V1.
    pub fn set_vegetation_debug_mode(&mut self, mode: VegetationDebugMode) {
        self.inner.set_vegetation_debug_mode(mode);
    }

    /// Apply the presentation-only manual exposure, delegating to V1.
    ///
    /// # Errors
    ///
    /// Propagates the V1 [`ExposureError`] guard unchanged.
    pub fn set_exposure_ev(&mut self, exposure_ev: f32) -> Result<(), ExposureError> {
        self.inner.set_exposure_ev(exposure_ev)
    }
}
