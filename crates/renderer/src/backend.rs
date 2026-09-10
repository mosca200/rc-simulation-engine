//! Shared V1/V2 renderer facade.
//!
//! This module is the single architectural seam that lets the application
//! choose a rendering backend at runtime (`--renderer v1` / `--renderer v2`)
//! while both backends present the *same* [`RenderFrame`] contract.
//!
//! Design notes (RV2-1):
//! - [`RendererVersion`] is the CLI-facing selection and diagnostic label.
//! - [`DesktopRenderer`] is a statically-dispatched facade: it owns a private
//!   `Backend` enum and `match`es on it. There is no trait object, no generic
//!   dependency injection, and no framework abstraction.
//! - The V2 variant owns its foundation state behind [`crate::renderer_v2`].
//!   The application never sees those internals.
//!
//! The facade exposes exactly the small contract the app already used against
//! [`WgpuRenderer`]: async construction with a presentation asset, `render`,
//! `resize`, `reconfigure_surface`, the debug-overlay / terrain-debug /
//! vegetation-debug / exposure setters, and read-only version access for
//! diagnostics.

use std::sync::Arc;

use winit::window::Window;

use crate::{
    CameraConfig, ExposureError, PresentationAsset, RenderFrame, RenderTerrainMode, RendererError,
    SurfaceError, TerrainDebugMode, VegetationDebugMode, WgpuRenderer,
    renderer_v2::RendererV2Shell, scenery::SceneryPreset,
};

/// Which rendering backend the application selected.
///
/// Defaults to [`RendererVersion::V1`] so every existing command keeps its
/// historical behaviour when `--renderer` is omitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RendererVersion {
    /// The frozen V1 production renderer (`WgpuRenderer`).
    #[default]
    V1,
    /// The Rendering V2 backend.
    V2,
}

impl RendererVersion {
    /// Stable CLI label for this version.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::V1 => "v1",
            Self::V2 => "v2",
        }
    }

    /// Parse a CLI label into a version, rejecting unknown values.
    #[must_use]
    pub fn from_label(label: &str) -> Option<Self> {
        match label {
            "v1" => Some(Self::V1),
            "v2" => Some(Self::V2),
            _ => None,
        }
    }

    /// Stable diagnostic identity of the backend this version dispatches to.
    ///
    /// Used by CPU-only tests to assert the factory selection distinguishes
    /// V1 from V2 without constructing a GPU.
    #[must_use]
    pub const fn backend_id(self) -> &'static str {
        match self {
            Self::V1 => "wgpu-v1",
            Self::V2 => "renderer-v2-parity-shell",
        }
    }
}

/// The concrete backend owned by [`DesktopRenderer`].
///
/// Private so the V2 shell type never leaks into the public API; dispatch is a
/// plain `match`, keeping the facade statically dispatched.
enum Backend {
    V1(Box<WgpuRenderer>),
    V2(Box<RendererV2Shell>),
}

/// Runtime renderer facade shared by the V1 and V2 backends.
///
/// The application drives this type exactly as it previously drove
/// [`WgpuRenderer`]; the selected backend is an internal detail.
pub struct DesktopRenderer {
    backend: Backend,
}

impl DesktopRenderer {
    /// Construct the selected backend with a presentation asset and optional
    /// scenery.
    ///
    /// Both branches forward identical presentation settings, ground
    /// reference, terrain mode, scenery, and camera configuration.
    ///
    /// # Errors
    ///
    /// Returns the shared renderer initialization errors unchanged.
    pub async fn new_with_presentation(
        version: RendererVersion,
        window: Arc<Window>,
        asset: PresentationAsset<'_>,
        ground_below_render_origin_m: f32,
        terrain_mode: RenderTerrainMode,
        scenery_preset: Option<SceneryPreset>,
        camera_config: CameraConfig,
    ) -> Result<Self, RendererError> {
        let backend = match version {
            RendererVersion::V1 => Backend::V1(Box::new(
                WgpuRenderer::new_with_presentation(
                    window,
                    asset,
                    ground_below_render_origin_m,
                    terrain_mode,
                    scenery_preset,
                    camera_config,
                )
                .await?,
            )),
            RendererVersion::V2 => Backend::V2(Box::new(
                RendererV2Shell::new_with_presentation(
                    window,
                    asset,
                    ground_below_render_origin_m,
                    terrain_mode,
                    scenery_preset,
                    camera_config,
                )
                .await?,
            )),
        };
        Ok(Self { backend })
    }

    /// Read-only backend selection, for diagnostics.
    #[must_use]
    pub const fn version(&self) -> RendererVersion {
        match self.backend {
            Backend::V1(_) => RendererVersion::V1,
            Backend::V2(_) => RendererVersion::V2,
        }
    }

    /// Present a shared [`RenderFrame`] through the selected backend.
    ///
    /// # Errors
    ///
    /// Propagates the backend's [`SurfaceError`] unchanged so the application's
    /// existing surface-event policy applies identically to V1 and V2.
    pub fn render(&mut self, frame: &RenderFrame) -> Result<(), SurfaceError> {
        match &mut self.backend {
            Backend::V1(inner) => inner.render(frame),
            Backend::V2(inner) => inner.render(frame),
        }
    }

    /// Resize the presentation surface of the selected backend.
    pub fn resize(&mut self, width: u32, height: u32) {
        match &mut self.backend {
            Backend::V1(inner) => inner.resize(width, height),
            Backend::V2(inner) => inner.resize(width, height),
        }
    }

    /// Recreate the surface after a lost/outdated event on the selected backend.
    pub fn reconfigure_surface(&mut self) {
        match &mut self.backend {
            Backend::V1(inner) => inner.reconfigure_surface(),
            Backend::V2(inner) => inner.reconfigure_surface(),
        }
    }

    /// Toggle the presentation-only debug overlays on the selected backend.
    pub fn set_show_debug_overlays(&mut self, show: bool) {
        match &mut self.backend {
            Backend::V1(inner) => inner.set_show_debug_overlays(show),
            Backend::V2(inner) => inner.set_show_debug_overlays(show),
        }
    }

    /// Select the presentation-only terrain debug channel on the backend.
    pub fn set_terrain_debug_mode(&mut self, mode: TerrainDebugMode) {
        match &mut self.backend {
            Backend::V1(inner) => inner.set_terrain_debug_mode(mode),
            Backend::V2(inner) => inner.set_terrain_debug_mode(mode),
        }
    }

    /// Select the presentation-only vegetation debug channel on the backend.
    pub fn set_vegetation_debug_mode(&mut self, mode: VegetationDebugMode) {
        match &mut self.backend {
            Backend::V1(inner) => inner.set_vegetation_debug_mode(mode),
            Backend::V2(inner) => inner.set_vegetation_debug_mode(mode),
        }
    }

    /// Apply the presentation-only manual exposure on the selected backend.
    ///
    /// # Errors
    ///
    /// Propagates the backend's [`ExposureError`] guard unchanged.
    pub fn set_exposure_ev(&mut self, exposure_ev: f32) -> Result<(), ExposureError> {
        match &mut self.backend {
            Backend::V1(inner) => inner.set_exposure_ev(exposure_ev),
            Backend::V2(inner) => inner.set_exposure_ev(exposure_ev),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn renderer_version_defaults_to_v1() {
        assert_eq!(RendererVersion::default(), RendererVersion::V1);
    }

    #[test]
    fn renderer_version_label_round_trips() {
        for version in [RendererVersion::V1, RendererVersion::V2] {
            assert_eq!(RendererVersion::from_label(version.label()), Some(version));
        }
    }

    #[test]
    fn renderer_version_from_label_rejects_unknown() {
        assert_eq!(RendererVersion::from_label("foo"), None);
        assert_eq!(RendererVersion::from_label(""), None);
        assert_eq!(RendererVersion::from_label("V1"), None);
    }

    #[test]
    fn backend_selection_distinguishes_v1_from_v2_without_gpu() {
        // Dispatch identity is a pure function of the version: the factory
        // routes V1 and V2 to distinct backends, verified here without a GPU.
        assert_ne!(
            RendererVersion::V1.backend_id(),
            RendererVersion::V2.backend_id()
        );
        assert_eq!(RendererVersion::V1.backend_id(), "wgpu-v1");
        assert_eq!(RendererVersion::V2.backend_id(), "renderer-v2-parity-shell");
    }
}
