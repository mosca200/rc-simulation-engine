//! Rendering V2 foundation and rigid-GLB GPU scene path.
//!
//! The V2 backend now owns a compiled static render graph and bounded
//! profiling state while reusing the proven draw-recording implementation.
//! RV2-3 additionally routes rigid GLBs through [`scene`] so shared mesh
//! geometry and deterministic scene-node transforms reach the GPU without
//! changing the `RenderFrame` boundary.
//!
//! Later RV2 slices can replace more internals without changing this module's
//! contract with [`crate::backend::DesktopRenderer`].

pub(crate) mod aerial_perspective;
pub(crate) mod atmosphere;
pub(crate) mod ibl;
pub(crate) mod scene;
pub(crate) mod temporal;

use std::sync::Arc;

use winit::window::Window;

use crate::{
    CameraConfig, ExposureError, PresentationAsset, RenderFrame, RenderTerrainMode, RendererError,
    SurfaceError, TerrainDebugMode, VegetationDebugMode, WgpuRenderer, scenery::SceneryPreset,
};
use crate::{profiling::Profiler, render_graph::CompiledGraph};
use temporal::{InvalidationReason, TemporalState};

/// Foundation owner for the Rendering V2 backend.
///
/// It combines the shared renderer resources with the V2-only compiled graph
/// and profiler while keeping the rendered output identical to V1.
pub struct RendererV2Shell {
    inner: WgpuRenderer,
    graph: CompiledGraph,
    profiler: Profiler,
    temporal: TemporalState,
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
        let initial_size = window.inner_size();
        let inner = WgpuRenderer::new_v2_with_presentation(
            window,
            asset,
            ground_below_render_origin_m,
            terrain_mode,
            scenery_preset,
            camera_config,
        )
        .await?;
        let graph = crate::render_graph::build_v2_render_graph()
            .expect("the static RV2 production graph must compile");
        debug_assert_eq!(graph.resource_class_counts(), [3, 2, 1]);
        let profiler = inner.create_v2_profiler();
        let temporal = TemporalState::new(initial_size.width, initial_size.height);
        Ok(Self {
            inner,
            graph,
            profiler,
            temporal,
        })
    }

    /// Present a shared [`RenderFrame`], delegating to the V1 backend.
    ///
    /// # Errors
    ///
    /// Propagates the V1 [`SurfaceError`] unchanged so the application's
    /// surface-event policy (lost / outdated / timeout / out-of-memory /
    /// validation) behaves identically for V1 and V2.
    pub fn render(&mut self, frame: &RenderFrame) -> Result<(), SurfaceError> {
        self.inner
            .render_v2(frame, &self.graph, &mut self.profiler, &mut self.temporal)
    }

    /// Resize the presentation surface, delegating to the V1 backend.
    pub fn resize(&mut self, width: u32, height: u32) {
        self.inner.resize(width, height);
        self.temporal.resize(width, height);
    }

    /// Recreate the surface after a lost/outdated event, delegating to V1.
    pub fn reconfigure_surface(&mut self) {
        self.inner.reconfigure_surface();
        self.temporal
            .invalidate(InvalidationReason::SurfaceReconfigure);
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
