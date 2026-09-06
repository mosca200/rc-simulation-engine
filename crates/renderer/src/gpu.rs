//! G1C: GPU renderer with texture/material support and terrain rendering.
//!
//! # Material Architecture
//!
//! Each material has:
//! - A base color texture (or the persistent white fallback)
//! - A sampler (configured from glTF sampler settings)
//! - A bind group combining texture + sampler
//!
//! Textures, samplers, and bind groups are created once during asset upload
//! and never recreated per frame.
//!
//! # Draw Architecture
//!
//! Each frame is organized as:
//! 1. Directional shadow depth pass (terrain, scenery, aircraft, articulated surfaces)
//! 2. Main scene pass: sky (fullscreen triangle), terrain/scenery (lit + fogged),
//!    optional debug overlays (unlit), and aircraft batches (lit + fogged)
//!
//! # Object Transforms
//!
//! - Aircraft uses a dedicated object uniform buffer updated per frame from
//!   `frame.aircraft_pose().model_matrix()`.
//! - Terrain uses identity (world-local) object transform.
//! - Debug geometry uses identity (reference) object transform.
//!
//! # Surface Presentation
//!
//! After `queue.submit()`, frames are presented by calling
//! `queue.present(surface_texture)` to schedule the acquired surface texture
//! for presentation.

use crate::scenery::{SceneryMesh, SceneryPreset};
use crate::shadow::{
    SHADOW_DEPTH_BIAS_CONSTANT, SHADOW_DEPTH_BIAS_SLOPE_SCALE, SHADOW_MAP_RESOLUTION,
    SHADOW_RECEIVER_DEPTH_BIAS, stable_directional_shadow_transform,
};
use crate::terrain::{DEFAULT_CHUNK_CELLS, TerrainMaterial, generate_centered_terrain_chunks};
use crate::terrain_textures::generated as terrain_assets;
use crate::texture::{SamplerConfig, TextureLoadError, create_staging_buffer, decode_image};
use crate::{
    AircraftMesh, CameraConfig, CameraMode, GlbAsset, Mat4, RenderFrame, Vertex,
    matrix_to_wgsl_columns, reference_grid_and_axes_at,
};
use bytemuck::{Pod, Zeroable};
use std::{
    mem::size_of,
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
};
use thiserror::Error;
use wgpu::util::DeviceExt;
use winit::window::Window;

const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
const GPU_ERROR_NONE: u8 = 0;
const GPU_ERROR_OUT_OF_MEMORY: u8 = 1;
const GPU_ERROR_OTHER: u8 = 2;
pub const SKY_CLEAR_COLOR: [f64; 4] = [0.42, 0.68, 0.92, 1.0];

const DEFAULT_LIGHT_DIRECTION: [f32; 3] = [0.4, 0.8, -0.3];
const DEFAULT_LIGHT_INTENSITY: f32 = 0.80;
const DEFAULT_AMBIENT_RGB: [f32; 3] = [0.30, 0.30, 0.30];

const DEFAULT_ZENITH_RGB: [f32; 3] = [0.16, 0.36, 0.66];
const DEFAULT_HORIZON_RGB: [f32; 3] = [0.68, 0.78, 0.88];
const DEFAULT_GROUND_ATM_RGB: [f32; 3] = [0.38, 0.44, 0.40];
const DEFAULT_HAZE_STRENGTH: f32 = 0.55;
const DEFAULT_FOG_DENSITY: f32 = 0.0015;
const DEFAULT_SUN_COLOR_RGB: [f32; 3] = [1.0, 0.95, 0.85];
const DEFAULT_SUN_COS_ANGULAR_RADIUS: f32 = 0.999_96;

// G1D: material response parameters for procedural geometry (terrain, scenery,
// debug-adjacent fallback, procedural aircraft).
//
// Procedural surfaces must NOT become accidentally chromed: they are explicit
// non-metals with a high roughness so the PBR specular response stays subdued
// and the terrain keeps its matte, readable look.
const PROCEDURAL_METALLIC: f32 = 0.0;
const PROCEDURAL_ROUGHNESS: f32 = 0.85;

/// Default terrain extent for the RC flying field.
const DEFAULT_TERRAIN_EXTENT_M: f32 = 1000.0;
const DEFAULT_TERRAIN_CELL_SPACING_M: f32 = 5.0;

/// Presentation asset: a rigid GLB, explicitly articulated GLB, or procedural fallback.
#[derive(Clone, Copy)]
pub enum PresentationAsset<'a> {
    Glb(&'a GlbAsset),
    ArticulatedGlb {
        asset: &'a GlbAsset,
        articulation: &'a crate::GlbArticulationPlan,
    },
    Procedural(&'a AircraftMesh),
}

/// Terrain visual mode for the renderer.
///
/// The physics ground authority is always the flat NED z=0 plane.
/// This enum only controls the visual terrain mesh shown in the renderer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RenderTerrainMode {
    /// Rolling/hilly terrain for airborne visual demos.
    Rolling,
    /// Flat terrain aligned with the physics ground plane.
    /// Suitable for ground operations (taxi, takeoff, landing).
    Flat,
}

impl RenderTerrainMode {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Rolling => "rolling",
            Self::Flat => "flat",
        }
    }
}

#[derive(Debug, Error)]
pub enum RendererError {
    #[error("failed to create the wgpu surface: {0}")]
    CreateSurface(#[source] wgpu::CreateSurfaceError),
    #[error("no compatible GPU adapter was found: {0}")]
    AdapterNotFound(String),
    #[error("failed to request the GPU device: {0}")]
    RequestDevice(String),
    #[error("the surface reports no supported texture formats")]
    SurfaceWithoutFormats,
    #[error("the surface reports no supported alpha modes")]
    SurfaceWithoutAlphaModes,
    #[error("ground distance below the render origin must be finite and positive")]
    InvalidGroundReference,
    #[error("failed to upload texture to GPU: {0}")]
    TextureUpload(#[source] TextureLoadError),
}

/// Presentation failures normalized from wgpu 30's `CurrentSurfaceTexture` API.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum SurfaceError {
    #[error("GPU memory exhausted")]
    OutOfMemory,
    #[error("surface lost")]
    Lost,
    #[error("surface configuration outdated")]
    Outdated,
    #[error("surface acquisition timed out")]
    Timeout,
    #[error("surface is occluded")]
    Occluded,
    #[error("GPU validation or internal error")]
    Validation,
}

/// GPU camera uniform matching the WGSL `CameraUniform` struct.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_projection: [[f32; 4]; 4],
    inv_view_projection: [[f32; 4]; 4],
    camera_position: [f32; 4],
}

impl CameraUniform {
    fn new(vp: &Mat4, inv_vp: &Mat4, eye: [f32; 3]) -> Self {
        Self {
            view_projection: matrix_to_wgsl_columns(vp),
            inv_view_projection: matrix_to_wgsl_columns(inv_vp),
            camera_position: [eye[0], eye[1], eye[2], 0.0],
        }
    }
}

/// GPU environment uniform matching the WGSL `EnvironmentUniform` struct.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct EnvironmentUniform {
    light_direction: [f32; 4],
    ambient: [f32; 4],
    sky_zenith: [f32; 4],
    sky_horizon: [f32; 4],
    sky_ground: [f32; 4],
    sun_color: [f32; 4],
}

/// G2B: per-frame light matrix and receiver bias for directional shadows.
///
/// The matrix is kept in the existing environment bind-group boundary, beside
/// the shared light direction and atmosphere data. It is the only shadow
/// buffer written in the frame path.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct ShadowUniform {
    light_view_projection: [[f32; 4]; 4],
    receiver_depth_bias_and_padding: [f32; 4],
}

impl ShadowUniform {
    fn from_matrix(light_view_projection: &Mat4) -> Self {
        Self {
            light_view_projection: matrix_to_wgsl_columns(light_view_projection),
            receiver_depth_bias_and_padding: [SHADOW_RECEIVER_DEPTH_BIAS, 0.0, 0.0, 0.0],
        }
    }
}

impl EnvironmentUniform {
    fn default_environment() -> Self {
        let dir = DEFAULT_LIGHT_DIRECTION;
        let length = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        let normalized = if length > f32::EPSILON {
            [dir[0] / length, dir[1] / length, dir[2] / length]
        } else {
            [0.0, 1.0, 0.0]
        };
        Self {
            light_direction: [
                normalized[0],
                normalized[1],
                normalized[2],
                DEFAULT_LIGHT_INTENSITY,
            ],
            ambient: [
                DEFAULT_AMBIENT_RGB[0],
                DEFAULT_AMBIENT_RGB[1],
                DEFAULT_AMBIENT_RGB[2],
                0.0,
            ],
            sky_zenith: [
                DEFAULT_ZENITH_RGB[0],
                DEFAULT_ZENITH_RGB[1],
                DEFAULT_ZENITH_RGB[2],
                0.0,
            ],
            sky_horizon: [
                DEFAULT_HORIZON_RGB[0],
                DEFAULT_HORIZON_RGB[1],
                DEFAULT_HORIZON_RGB[2],
                DEFAULT_HAZE_STRENGTH,
            ],
            sky_ground: [
                DEFAULT_GROUND_ATM_RGB[0],
                DEFAULT_GROUND_ATM_RGB[1],
                DEFAULT_GROUND_ATM_RGB[2],
                DEFAULT_FOG_DENSITY,
            ],
            sun_color: [
                DEFAULT_SUN_COLOR_RGB[0],
                DEFAULT_SUN_COLOR_RGB[1],
                DEFAULT_SUN_COLOR_RGB[2],
                DEFAULT_SUN_COS_ANGULAR_RADIUS,
            ],
        }
    }
}

/// Object uniform for model matrix.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct ObjectUniform {
    columns: [[f32; 4]; 4],
}

impl ObjectUniform {
    fn from_matrix(matrix: &Mat4) -> Self {
        Self {
            columns: matrix_to_wgsl_columns(matrix),
        }
    }
}

/// G1D: per-primitive PBR material parameters (metallic/roughness workflow).
///
/// 16 bytes (vec4 rounding) to satisfy WGSL uniform buffer alignment rules.
/// Created once per material at asset upload time and never written per frame.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct MaterialUniform {
    metallic: f32,
    roughness: f32,
    _reserved: [f32; 2],
}

impl MaterialUniform {
    fn new(metallic: f32, roughness: f32) -> Self {
        Self {
            metallic,
            roughness,
            _reserved: [0.0; 2],
        }
    }
}

/// G3A: terrain material uniform matching the WGSL `TerrainMaterialUniform`
/// struct (four 16-byte vec4 slots, 64 bytes total).
///
/// The per-map UV anchors (in tile units) decorrelate the three samples so
/// their tile borders never align; offsets are added to the world-space UV.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct TerrainMaterialUniform {
    metallic: f32,
    roughness: f32,
    normal_strength: f32,
    _padding: f32,
    albedo_uv_offset: [f32; 2],
    normal_uv_offset: [f32; 2],
    roughness_uv_offset: [f32; 2],
    // 6 floats: keeps the struct at 64 bytes (four WGSL vec4 slots),
    // matching the shader's vec4-aligned `padding2`.
    _padding2: [f32; 6],
}

impl TerrainMaterialUniform {
    fn from_terrain_material(material: &TerrainMaterial) -> Self {
        Self {
            metallic: material.metallic.clamp(0.0, 1.0),
            roughness: material.roughness.clamp(0.0, 1.0),
            normal_strength: material.normal_strength.clamp(0.0, 1.0),
            _padding: 0.0,
            albedo_uv_offset: material.albedo_uv_offset,
            normal_uv_offset: material.normal_uv_offset,
            roughness_uv_offset: material.roughness_uv_offset,
            _padding2: [0.0; 6],
        }
    }
}

struct DepthTarget {
    _texture: wgpu::Texture,
    view: wgpu::TextureView,
}

/// G2B: persistent depth texture sampled by the lit pass and written by the
/// directional shadow caster pass. It is deliberately independent from the
/// resize-dependent scene depth target.
struct ShadowTarget {
    _texture: wgpu::Texture,
    view: wgpu::TextureView,
}

/// G1C: Persistent GPU material resources.
struct GpuMaterial {
    _texture: wgpu::Texture,
    _texture_view: wgpu::TextureView,
    _sampler: wgpu::Sampler,
    // G1D: metallic/roughness uniform buffer (static, written once at upload).
    _material_uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// G3A: persistent GPU terrain material (dedicated bind group at group 4).
///
/// Owns the albedo (sRGB), normal (linear), and roughness (linear) textures
/// plus one shared repeat/linear sampler and the static material uniform.
/// Created once at renderer initialization, never recreated per frame.
struct GpuTerrainMaterial {
    _albedo_texture: wgpu::Texture,
    _albedo_texture_view: wgpu::TextureView,
    _normal_texture: wgpu::Texture,
    _normal_texture_view: wgpu::TextureView,
    _roughness_texture: wgpu::Texture,
    _roughness_texture_view: wgpu::TextureView,
    _sampler: wgpu::Sampler,
    _material_uniform: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

/// G1C: A render batch with its own vertex/index buffers and material.
struct RenderBatch {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    material_index: usize,
}

/// G1E: one articulated surface draw reusing the lit pipeline/materials.
struct SurfaceRenderBatch {
    surface: crate::SurfaceId,
    hinge: crate::SurfaceHinge,
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    material_index: usize,
    object_buffer_index: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct GlbBatchTarget {
    material_index: usize,
    hinge: Option<crate::SurfaceHinge>,
}

fn glb_batch_target(
    plan: Option<&crate::GlbArticulationPlan>,
    primitive_index: usize,
    material_index: usize,
) -> GlbBatchTarget {
    let hinge = plan.and_then(|plan| match plan.part(primitive_index) {
        crate::GlbPrimitivePart::Rigid => None,
        crate::GlbPrimitivePart::Articulated { hinge, .. } => Some(*hinge),
    });
    GlbBatchTarget {
        material_index,
        hinge,
    }
}

/// G1C: Terrain chunk GPU resources.
struct GpuTerrainChunk {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    _bounds: ([f32; 3], [f32; 3]),
}

/// G2A: Scenery GPU resources (merged mesh, single draw call).
struct GpuScenery {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
}

/// Minimal depth-tested wgpu renderer with G1C texture/material support.
pub struct WgpuRenderer {
    _instance: wgpu::Instance,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface_configuration: wgpu::SurfaceConfiguration,
    surface_is_configured: bool,

    sky_pipeline: wgpu::RenderPipeline,
    triangle_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    shadow_pipeline: wgpu::RenderPipeline,
    // G3A: terrain pipeline (dedicated fs_terrain entry, group-4 material).
    terrain_pipeline: wgpu::RenderPipeline,

    _camera_bind_group_layout: wgpu::BindGroupLayout,
    _object_bind_group_layout: wgpu::BindGroupLayout,
    _environment_bind_group_layout: wgpu::BindGroupLayout,
    _shadow_pass_bind_group_layout: wgpu::BindGroupLayout,
    _material_bind_group_layout: wgpu::BindGroupLayout,
    // G3A: extended terrain material layout (albedo/normal/roughness + uniform).
    _terrain_material_bind_group_layout: wgpu::BindGroupLayout,

    // Persistent bind groups.
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,

    // FIX 2: Dedicated aircraft object buffer (updated per frame).
    aircraft_object_buffer: wgpu::Buffer,
    aircraft_object_bind_group: wgpu::BindGroup,

    // Identity object buffer for terrain and debug geometry.
    _identity_object_buffer: wgpu::Buffer,
    identity_object_bind_group: wgpu::BindGroup,

    _environment_buffer: wgpu::Buffer,
    shadow_matrix_buffer: wgpu::Buffer,
    environment_bind_group: wgpu::BindGroup,
    shadow_pass_bind_group: wgpu::BindGroup,
    shadow_light_direction: [f32; 3],

    // G1C: Material system.
    materials: Vec<GpuMaterial>,
    _fallback_material_index: usize,

    // G1C: Aircraft batches (one per primitive).
    aircraft_batches: Vec<RenderBatch>,

    // G1E: persistent articulated procedural or GLB batches.
    surface_batches: Vec<SurfaceRenderBatch>,
    surface_object_buffers: Vec<wgpu::Buffer>,
    surface_object_bind_groups: Vec<wgpu::BindGroup>,

    // G1C: Terrain chunks.
    terrain_chunks: Vec<GpuTerrainChunk>,
    // G3A: textured terrain material (replaces the white-fallback terrain).
    terrain_material: GpuTerrainMaterial,

    // G2A: Scenery.
    scenery: Option<GpuScenery>,
    scenery_material_index: usize,

    // Debug overlays.
    line_vertex_buffer: wgpu::Buffer,
    line_vertex_count: u32,

    depth_target: DepthTarget,
    shadow_target: ShadowTarget,
    _shadow_sampler: wgpu::Sampler,
    camera: CameraMode,
    asynchronous_gpu_error: Arc<AtomicU8>,

    show_debug_overlays: bool,
}

impl WgpuRenderer {
    /// Create a renderer with a presentation asset (GLB or procedural) and
    /// optional scenery.
    ///
    /// This is the primary constructor. The GLB path exercises the full
    /// G1C textured multi-primitive pipeline.
    pub async fn new_with_presentation(
        window: Arc<Window>,
        asset: PresentationAsset<'_>,
        ground_below_render_origin_m: f32,
        terrain_mode: RenderTerrainMode,
        scenery_preset: Option<SceneryPreset>,
        camera_config: CameraConfig,
    ) -> Result<Self, RendererError> {
        if !ground_below_render_origin_m.is_finite() || ground_below_render_origin_m <= 0.0 {
            return Err(RendererError::InvalidGroundReference);
        }

        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let surface = instance
            .create_surface(window)
            .map_err(RendererError::CreateSurface)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: Some(&surface),
                apply_limit_buckets: false,
            })
            .await
            .map_err(|error| RendererError::AdapterNotFound(error.to_string()))?;
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("G1C device"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: wgpu::MemoryHints::Performance,
                trace: wgpu::Trace::Off,
            })
            .await
            .map_err(|error| RendererError::RequestDevice(error.to_string()))?;

        let asynchronous_gpu_error = Arc::new(AtomicU8::new(GPU_ERROR_NONE));
        let callback_error = Arc::clone(&asynchronous_gpu_error);
        device.on_uncaptured_error(Arc::new(move |error| {
            let code = match error {
                wgpu::Error::OutOfMemory { .. } => GPU_ERROR_OUT_OF_MEMORY,
                wgpu::Error::Validation { .. } | wgpu::Error::Internal { .. } => GPU_ERROR_OTHER,
            };
            callback_error.store(code, Ordering::Release);
        }));

        let capabilities = surface.get_capabilities(&adapter);
        let fallback_format = capabilities
            .formats
            .first()
            .copied()
            .ok_or(RendererError::SurfaceWithoutFormats)?;
        let format = capabilities
            .formats
            .iter()
            .copied()
            .find(wgpu::TextureFormat::is_srgb)
            .unwrap_or(fallback_format);
        let alpha_mode = capabilities
            .alpha_modes
            .first()
            .copied()
            .ok_or(RendererError::SurfaceWithoutAlphaModes)?;
        let surface_configuration = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            color_space: wgpu::SurfaceColorSpace::Auto,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::AutoVsync,
            desired_maximum_frame_latency: 2,
            alpha_mode,
            view_formats: Vec::new(),
        };
        let surface_is_configured = size.width > 0 && size.height > 0;
        if surface_is_configured {
            surface.configure(&device, &surface_configuration);
        }

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("G1C sky+material+texture+lighting+atmosphere shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let camera_bind_group_layout = camera_bind_group_layout(&device, "camera layout");
        let object_bind_group_layout = matrix_bind_group_layout(&device, "object layout");
        let environment_bind_group_layout =
            environment_bind_group_layout(&device, "environment layout");
        let shadow_pass_bind_group_layout =
            shadow_pass_bind_group_layout(&device, "directional shadow pass layout");
        let material_bind_group_layout = material_bind_group_layout(&device, "material layout");
        let terrain_material_bind_group_layout =
            terrain_material_bind_group_layout(&device, "G3A terrain material layout");

        let sky_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G1C sky pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout),
                Some(&object_bind_group_layout),
                Some(&environment_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let lit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G1C lit pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout),
                Some(&object_bind_group_layout),
                Some(&environment_bind_group_layout),
                Some(&material_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let unlit_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G1C unlit pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&object_bind_group_layout),
                ],
                immediate_size: 0,
            });
        let shadow_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G2B directional shadow pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&object_bind_group_layout),
                    Some(&shadow_pass_bind_group_layout),
                ],
                immediate_size: 0,
            });
        // G3A: the terrain pipeline shares groups 0-2 with the lit pipeline and
        // carries its own group-4 terrain material layout. Group 3 is unused by
        // the terrain shaders (no shared material slot conflicts).
        let terrain_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G3A terrain pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&object_bind_group_layout),
                    Some(&environment_bind_group_layout),
                    None,
                    Some(&terrain_material_bind_group_layout),
                ],
                immediate_size: 0,
            });

        let sky_pipeline = create_sky_pipeline(&device, &shader, &sky_pipeline_layout, format);
        let triangle_pipeline = create_pipeline(
            &device,
            &shader,
            &lit_pipeline_layout,
            format,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                depth_write_enabled: true,
                label: "G1C lit triangle pipeline",
                fragment_entry_point: "fs_lit",
            },
        );
        let line_pipeline = create_pipeline(
            &device,
            &shader,
            &unlit_pipeline_layout,
            format,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::LineList,
                cull_mode: None,
                depth_write_enabled: false,
                label: "G1C unlit line pipeline",
                fragment_entry_point: "fs_unlit",
            },
        );
        let shadow_pipeline = create_shadow_pipeline(&device, &shader, &shadow_pipeline_layout);

        // G3A: dedicated terrain pipeline — same raster state as the lit
        // triangle pipeline, `fs_terrain` fragment entry, group-4 material.
        let terrain_pipeline = create_pipeline(
            &device,
            &shader,
            &terrain_pipeline_layout,
            format,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                depth_write_enabled: true,
                label: "G3A terrain lit triangle pipeline",
                fragment_entry_point: "fs_terrain",
            },
        );

        // White fallback material.
        let fallback_material =
            create_white_fallback_material(&device, &material_bind_group_layout, &queue);
        let mut materials = vec![fallback_material];
        let fallback_material_index = 0;

        // Upload presentation geometry once. Explicitly mapped GLB primitives
        // become articulated batches; every other GLB primitive remains rigid.
        let mut aircraft_batches = Vec::new();
        let mut surface_batches = Vec::new();
        let mut surface_object_buffers = Vec::new();
        let mut surface_object_bind_groups = Vec::new();

        let glb_and_plan = match asset {
            PresentationAsset::Glb(glb_asset) => Some((glb_asset, None)),
            PresentationAsset::ArticulatedGlb {
                asset: glb_asset,
                articulation,
            } => Some((glb_asset, Some(articulation))),
            PresentationAsset::Procedural(_) => None,
        };
        if let Some((glb_asset, articulation)) = glb_and_plan {
            for (primitive_index, primitive) in glb_asset.primitives.iter().enumerate() {
                let material_index = if let Some(texture) = &primitive.material.base_color_texture {
                    let gpu_material = create_gpu_material(
                        &device,
                        &material_bind_group_layout,
                        &queue,
                        texture,
                        &primitive.material.sampler_config,
                        primitive.material.metallic_factor,
                        primitive.material.roughness_factor,
                    )?;
                    let index = materials.len();
                    materials.push(gpu_material);
                    index
                } else {
                    fallback_material_index
                };
                if primitive.vertices.is_empty() {
                    continue;
                }
                let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("aircraft primitive vertices"),
                    contents: bytemuck::cast_slice(&primitive.vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("aircraft primitive indices"),
                    contents: bytemuck::cast_slice(&primitive.indices),
                    usage: wgpu::BufferUsages::INDEX,
                });
                let target = glb_batch_target(articulation, primitive_index, material_index);
                if let Some(hinge) = target.hinge {
                    let object_buffer =
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some("GLB articulated part object uniform"),
                            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(
                                &crate::Mat4::identity(),
                            )),
                            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                        });
                    let object_bind_group = matrix_bind_group(
                        &device,
                        &object_bind_group_layout,
                        &object_buffer,
                        "GLB articulated part object bind group",
                    );
                    surface_batches.push(SurfaceRenderBatch {
                        surface: hinge.surface(),
                        hinge,
                        vertex_buffer,
                        index_buffer,
                        index_count: primitive.indices.len() as u32,
                        material_index: target.material_index,
                        object_buffer_index: surface_object_buffers.len(),
                    });
                    surface_object_buffers.push(object_buffer);
                    surface_object_bind_groups.push(object_bind_group);
                } else {
                    aircraft_batches.push(RenderBatch {
                        vertex_buffer,
                        index_buffer,
                        index_count: primitive.indices.len() as u32,
                        material_index: target.material_index,
                    });
                }
            }
        } else if let PresentationAsset::Procedural(mesh) = asset {
            if !mesh.vertices().is_empty() {
                let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("aircraft presentation vertices"),
                    contents: bytemuck::cast_slice(mesh.vertices()),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("aircraft presentation indices"),
                    contents: bytemuck::cast_slice(mesh.indices()),
                    usage: wgpu::BufferUsages::INDEX,
                });
                aircraft_batches.push(RenderBatch {
                    vertex_buffer,
                    index_buffer,
                    index_count: mesh.indices().len() as u32,
                    material_index: fallback_material_index,
                });
            }

            let articulated = crate::articulated_aircraft_mesh();
            let surface_binding_table = crate::articulated_binding_table();
            for surface in crate::SurfaceId::control_surfaces() {
                let Some(mesh) = articulated.surface(surface) else {
                    continue;
                };
                let Some(hinge) = surface_binding_table.hinge(surface).copied() else {
                    continue;
                };
                if mesh.vertices().is_empty() {
                    continue;
                }
                let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("control surface vertices"),
                    contents: bytemuck::cast_slice(mesh.vertices()),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("control surface indices"),
                    contents: bytemuck::cast_slice(mesh.indices()),
                    usage: wgpu::BufferUsages::INDEX,
                });
                let object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("control surface object uniform"),
                    contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(
                        &crate::Mat4::identity(),
                    )),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                });
                let object_bind_group = matrix_bind_group(
                    &device,
                    &object_bind_group_layout,
                    &object_buffer,
                    "control surface object bind group",
                );
                surface_batches.push(SurfaceRenderBatch {
                    surface,
                    hinge,
                    vertex_buffer,
                    index_buffer,
                    index_count: mesh.indices().len() as u32,
                    material_index: fallback_material_index,
                    object_buffer_index: surface_object_buffers.len(),
                });
                surface_object_buffers.push(object_buffer);
                surface_object_bind_groups.push(object_bind_group);
            }
        }

        // FIX 3: Generate centered terrain (extends in both +X/-X and +Z/-Z).
        // G2A fix: FlyingField uses flat visual terrain at -ground_below_render_origin_m
        // to avoid rolling-terrain penetration through the runway/field.
        // SceneryPreset::None preserves the existing rolling terrain.
        // Ground-start (RenderTerrainMode::Flat) also forces flat terrain.
        let terrain_cells = (DEFAULT_TERRAIN_EXTENT_M / DEFAULT_TERRAIN_CELL_SPACING_M) as u32;
        let terrain_height_field = match (terrain_mode, scenery_preset) {
            (RenderTerrainMode::Flat, _) | (_, Some(SceneryPreset::FlyingField)) => {
                crate::terrain::generate_flat_terrain(
                    terrain_cells,
                    terrain_cells,
                    DEFAULT_TERRAIN_CELL_SPACING_M,
                    -ground_below_render_origin_m,
                )
            }
            _ => crate::terrain::generate_rolling_terrain(
                terrain_cells,
                terrain_cells,
                DEFAULT_TERRAIN_CELL_SPACING_M,
                -ground_below_render_origin_m,
                3.0,
            ),
        };
        let terrain_material = TerrainMaterial::default();
        // FIX 3: Use centered chunk generation.
        let terrain_chunk_data = generate_centered_terrain_chunks(
            &terrain_height_field,
            DEFAULT_CHUNK_CELLS,
            &terrain_material,
        );

        // FIX 4 (superseded by G3A): the terrain previously reused the white
        // fallback material because the base color was baked into vertex
        // colors. G3A decodes the committed grass maps once at initialization
        // (they are embedded in the binary via include_bytes!) into the
        // dedicated terrain material; the white G2D vertex color still
        // modulates the sampled albedo as the macro-variation carrier.
        let terrain_material_gpu = create_terrain_material(
            &device,
            &terrain_material_bind_group_layout,
            &queue,
            &terrain_material,
        )?;

        let mut terrain_chunks = Vec::with_capacity(terrain_chunk_data.len());
        for chunk in &terrain_chunk_data {
            let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain chunk vertices"),
                contents: bytemuck::cast_slice(&chunk.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("terrain chunk indices"),
                contents: bytemuck::cast_slice(&chunk.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
            terrain_chunks.push(GpuTerrainChunk {
                vertex_buffer,
                index_buffer,
                index_count: chunk.indices.len() as u32,
                _bounds: chunk.bounds,
            });
        }

        // G2A: Generate scenery and upload to GPU.
        let scenery_material_index = fallback_material_index;
        let scenery = scenery_preset.and_then(|preset| match preset {
            SceneryPreset::None => None,
            SceneryPreset::FlyingField => {
                let params = crate::scenery::FlyingFieldParams {
                    ground_y: -ground_below_render_origin_m,
                    ..Default::default()
                };
                let scene = crate::scenery::generate_flying_field(&params);
                Some(upload_scenery_mesh(&device, &scene.mesh))
            }
        });

        // Debug overlays.
        let references = reference_grid_and_axes_at(-ground_below_render_origin_m);
        let line_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("reference grid and axes vertices"),
            contents: bytemuck::cast_slice(references.vertices()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // FIX 2: Create dedicated aircraft object buffer (COPY_DST for per-frame updates).
        let identity_object = ObjectUniform::from_matrix(&Mat4::identity());
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("camera uniform"),
            contents: bytemuck::bytes_of(&CameraUniform::new(
                &Mat4::identity(),
                &Mat4::identity(),
                [0.0; 3],
            )),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        // Aircraft object buffer: updated per frame with aircraft pose.
        let aircraft_object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("aircraft object uniform"),
            contents: bytemuck::bytes_of(&identity_object),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        // Identity object buffer: used for terrain and debug geometry.
        let identity_object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("identity object uniform"),
            contents: bytemuck::bytes_of(&identity_object),
            usage: wgpu::BufferUsages::UNIFORM,
        });

        let default_environment = EnvironmentUniform::default_environment();
        // G2B: derive the shadow camera direction from the exact normalized
        // direction uploaded into EnvironmentUniform, so the sun disk,
        // direct PBR lighting, and shadow map can never diverge.
        let shadow_light_direction = [
            default_environment.light_direction[0],
            default_environment.light_direction[1],
            default_environment.light_direction[2],
        ];
        let initial_shadow_transform =
            stable_directional_shadow_transform(shadow_light_direction, [0.0; 3]);
        let environment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("environment uniform"),
            contents: bytemuck::bytes_of(&default_environment),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("directional shadow matrix uniform"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_matrix(
                &initial_shadow_transform.light_view_projection,
            )),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let shadow_target = create_shadow_target(&device);
        let shadow_sampler = create_shadow_comparison_sampler(&device);

        // Bind groups.
        let camera_bind_group = camera_bind_group(
            &device,
            &camera_bind_group_layout,
            &camera_buffer,
            "camera bind group",
        );
        let aircraft_object_bind_group = matrix_bind_group(
            &device,
            &object_bind_group_layout,
            &aircraft_object_buffer,
            "aircraft object bind group",
        );
        let identity_object_bind_group = matrix_bind_group(
            &device,
            &object_bind_group_layout,
            &identity_object_buffer,
            "identity object bind group",
        );
        let environment_bind_group = create_environment_bind_group(
            &device,
            &environment_bind_group_layout,
            &environment_buffer,
            &shadow_target.view,
            &shadow_sampler,
            &shadow_matrix_buffer,
            "environment bind group",
        );
        let shadow_pass_bind_group = create_shadow_pass_bind_group(
            &device,
            &shadow_pass_bind_group_layout,
            &shadow_matrix_buffer,
            "directional shadow pass bind group",
        );

        let depth_target = create_depth_target(
            &device,
            surface_configuration.width,
            surface_configuration.height,
        );

        Ok(Self {
            _instance: instance,
            surface,
            device,
            queue,
            surface_configuration,
            surface_is_configured,
            sky_pipeline,
            triangle_pipeline,
            line_pipeline,
            shadow_pipeline,
            terrain_pipeline,
            _camera_bind_group_layout: camera_bind_group_layout,
            _object_bind_group_layout: object_bind_group_layout,
            _environment_bind_group_layout: environment_bind_group_layout,
            _shadow_pass_bind_group_layout: shadow_pass_bind_group_layout,
            _material_bind_group_layout: material_bind_group_layout,
            _terrain_material_bind_group_layout: terrain_material_bind_group_layout,
            camera_buffer,
            camera_bind_group,
            aircraft_object_buffer,
            aircraft_object_bind_group,
            _identity_object_buffer: identity_object_buffer,
            identity_object_bind_group,
            _environment_buffer: environment_buffer,
            shadow_matrix_buffer,
            environment_bind_group,
            shadow_pass_bind_group,
            shadow_light_direction,
            materials,
            _fallback_material_index: fallback_material_index,
            aircraft_batches,
            surface_batches,
            surface_object_buffers,
            surface_object_bind_groups,
            terrain_chunks,
            terrain_material: terrain_material_gpu,
            scenery,
            scenery_material_index,
            line_vertex_buffer,
            line_vertex_count: references.vertices().len() as u32,
            depth_target,
            shadow_target,
            _shadow_sampler: shadow_sampler,
            camera: camera_config.build(size.width, size.height),
            asynchronous_gpu_error,
            show_debug_overlays: false,
        })
    }

    /// Legacy constructor for backward compatibility with tests.
    pub async fn new(
        window: Arc<Window>,
        aircraft: &AircraftMesh,
        ground_below_render_origin_m: f32,
    ) -> Result<Self, RendererError> {
        Self::new_with_presentation(
            window,
            PresentationAsset::Procedural(aircraft),
            ground_below_render_origin_m,
            RenderTerrainMode::Rolling,
            None,
            CameraConfig::chase_default(),
        )
        .await
    }

    /// Create a renderer with a full GLB asset (multi-primitive support).
    pub async fn new_with_asset(
        window: Arc<Window>,
        asset: &GlbAsset,
        ground_below_render_origin_m: f32,
    ) -> Result<Self, RendererError> {
        Self::new_with_presentation(
            window,
            PresentationAsset::Glb(asset),
            ground_below_render_origin_m,
            RenderTerrainMode::Rolling,
            None,
            CameraConfig::chase_default(),
        )
        .await
    }

    /// Set debug overlay visibility.
    pub fn set_show_debug_overlays(&mut self, show: bool) {
        self.show_debug_overlays = show;
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            self.surface_is_configured = false;
            return;
        }
        self.surface_configuration.width = width;
        self.surface_configuration.height = height;
        self.camera.resize(width, height);
        self.reconfigure_surface();
        self.depth_target = create_depth_target(&self.device, width, height);
        self.surface_is_configured = true;
    }

    pub fn reconfigure_surface(&mut self) {
        if self.surface_configuration.width > 0 && self.surface_configuration.height > 0 {
            self.surface
                .configure(&self.device, &self.surface_configuration);
            self.surface_is_configured = true;
        }
    }

    pub fn render(&mut self, frame: &RenderFrame) -> Result<(), SurfaceError> {
        self.check_asynchronous_gpu_error()?;
        if !self.surface_is_configured {
            return Ok(());
        }

        let (surface_texture, reconfigure_after_present) = match self.surface.get_current_texture()
        {
            wgpu::CurrentSurfaceTexture::Success(texture) => (texture, false),
            wgpu::CurrentSurfaceTexture::Suboptimal(texture) => (texture, true),
            wgpu::CurrentSurfaceTexture::Timeout => return Err(SurfaceError::Timeout),
            wgpu::CurrentSurfaceTexture::Occluded => return Err(SurfaceError::Occluded),
            wgpu::CurrentSurfaceTexture::Outdated => return Err(SurfaceError::Outdated),
            wgpu::CurrentSurfaceTexture::Lost => return Err(SurfaceError::Lost),
            wgpu::CurrentSurfaceTexture::Validation => return Err(SurfaceError::Validation),
        };

        // Compute camera uniforms on the stack.
        let aircraft_pose = frame.aircraft_pose();
        let vp = self.camera.view_projection(aircraft_pose);
        let eye = self.camera.eye_position(aircraft_pose);
        let identity = Mat4::identity();
        let inv_vp = self
            .camera
            .inv_view_projection(aircraft_pose)
            .unwrap_or(identity);
        let camera_uniform = CameraUniform::new(&vp, &inv_vp, eye);

        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));

        // FIX 2: Update aircraft object uniform with the current pose model matrix.
        let aircraft_model_matrix = aircraft_pose.model_matrix();
        let aircraft_object_uniform = ObjectUniform::from_matrix(&aircraft_model_matrix);
        self.queue.write_buffer(
            &self.aircraft_object_buffer,
            0,
            bytemuck::bytes_of(&aircraft_object_uniform),
        );

        // G2B: the light camera follows the aircraft only on its light-space
        // texel grid. The single matrix write targets a persistent buffer; no
        // shadow GPU resource, bind group, or pipeline is created per frame.
        let shadow_transform = stable_directional_shadow_transform(
            self.shadow_light_direction,
            aircraft_pose.translation_render_m(),
        );
        let shadow_uniform = ShadowUniform::from_matrix(&shadow_transform.light_view_projection);
        self.queue.write_buffer(
            &self.shadow_matrix_buffer,
            0,
            bytemuck::bytes_of(&shadow_uniform),
        );

        // G1E: articulated surface uniforms (`root * local hinge`).
        for batch in &self.surface_batches {
            let composed = aircraft_model_matrix
                * batch
                    .hinge
                    .local_matrix(frame.surfaces().deflection(batch.surface));
            let uniform = ObjectUniform::from_matrix(&composed);
            self.queue.write_buffer(
                &self.surface_object_buffers[batch.object_buffer_index],
                0,
                bytemuck::bytes_of(&uniform),
            );
        }

        let surface_view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("G1C frame encoder"),
            });
        {
            // G2B: directional caster pass. This is intentionally depth-only:
            // terrain, scenery, rigid aircraft geometry, and articulated
            // surfaces are drawn once into the persistent shadow target.
            let depth_attachment = wgpu::RenderPassDepthStencilAttachment {
                view: &self.shadow_target.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            };
            let mut shadow_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("G2B directional shadow depth pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(depth_attachment),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            shadow_pass.set_pipeline(&self.shadow_pipeline);
            // Group 0 is unused by `vs_shadow`, but binding the existing camera
            // group keeps the depth pipeline layout contiguous and portable.
            shadow_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            shadow_pass.set_bind_group(2, &self.shadow_pass_bind_group, &[]);

            shadow_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            for chunk in &self.terrain_chunks {
                shadow_pass.set_vertex_buffer(0, chunk.vertex_buffer.slice(..));
                shadow_pass
                    .set_index_buffer(chunk.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                shadow_pass.draw_indexed(0..chunk.index_count, 0, 0..1);
            }

            if let Some(ref scenery) = self.scenery {
                shadow_pass.set_vertex_buffer(0, scenery.vertex_buffer.slice(..));
                shadow_pass
                    .set_index_buffer(scenery.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                shadow_pass.draw_indexed(0..scenery.index_count, 0, 0..1);
            }

            shadow_pass.set_bind_group(1, &self.aircraft_object_bind_group, &[]);
            for batch in &self.aircraft_batches {
                shadow_pass.set_vertex_buffer(0, batch.vertex_buffer.slice(..));
                shadow_pass
                    .set_index_buffer(batch.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                shadow_pass.draw_indexed(0..batch.index_count, 0, 0..1);
            }

            for batch in &self.surface_batches {
                shadow_pass.set_bind_group(
                    1,
                    &self.surface_object_bind_groups[batch.object_buffer_index],
                    &[],
                );
                shadow_pass.set_vertex_buffer(0, batch.vertex_buffer.slice(..));
                shadow_pass
                    .set_index_buffer(batch.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                shadow_pass.draw_indexed(0..batch.index_count, 0, 0..1);
            }
        }
        {
            let color_attachment = wgpu::RenderPassColorAttachment {
                view: &surface_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: SKY_CLEAR_COLOR[0],
                        g: SKY_CLEAR_COLOR[1],
                        b: SKY_CLEAR_COLOR[2],
                        a: SKY_CLEAR_COLOR[3],
                    }),
                    store: wgpu::StoreOp::Store,
                },
            };
            let depth_attachment = wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_target.view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            };
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("G1C scene pass"),
                color_attachments: &[Some(color_attachment)],
                depth_stencil_attachment: Some(depth_attachment),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });

            // --- Sky pass (background) ---
            render_pass.set_pipeline(&self.sky_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            // Sky uses identity object (group 1) â€” sky is at infinity.
            render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
            render_pass.draw(0..3, 0..1);

            // --- Scene geometry ---
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);

            // G3A: terrain chunks use the dedicated terrain pipeline and its own
            // bind group (identity object transform, world-local).
            render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            render_pass.set_pipeline(&self.terrain_pipeline);
            render_pass.set_bind_group(4, &self.terrain_material.bind_group, &[]);
            for chunk in &self.terrain_chunks {
                render_pass.set_vertex_buffer(0, chunk.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(chunk.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..chunk.index_count, 0, 0..1);
            }

            // G2A: Scenery (flying field, trees, markers). Drawn with the
            // shared lit pipeline — the terrain pipeline is terrain-only.
            if let Some(ref scenery) = self.scenery {
                let scenery_material = &self.materials[self.scenery_material_index];
                render_pass.set_pipeline(&self.triangle_pipeline);
                render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
                render_pass.set_bind_group(3, &scenery_material.bind_group, &[]);
                render_pass.set_vertex_buffer(0, scenery.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(scenery.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..scenery.index_count, 0, 0..1);
            }

            // Debug grid/axes: identity object transform.
            if self.show_debug_overlays {
                render_pass.set_pipeline(&self.line_pipeline);
                render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
                render_pass.set_vertex_buffer(0, self.line_vertex_buffer.slice(..));
                render_pass.draw(0..self.line_vertex_count, 0..1);
                render_pass.set_pipeline(&self.triangle_pipeline);
            }

            // FIX 2: Aircraft batches use the dedicated aircraft object bind group.
            for batch in &self.aircraft_batches {
                let material = &self.materials[batch.material_index];
                render_pass.set_bind_group(1, &self.aircraft_object_bind_group, &[]);
                render_pass.set_bind_group(3, &material.bind_group, &[]);
                render_pass.set_vertex_buffer(0, batch.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(batch.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..batch.index_count, 0, 0..1);
            }

            // G1E: articulated overlays, each with its persistent object uniform.
            for batch in &self.surface_batches {
                let material = &self.materials[batch.material_index];
                render_pass.set_bind_group(
                    1,
                    &self.surface_object_bind_groups[batch.object_buffer_index],
                    &[],
                );
                render_pass.set_bind_group(3, &material.bind_group, &[]);
                render_pass.set_vertex_buffer(0, batch.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(batch.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..batch.index_count, 0, 0..1);
            }
        }

        let _submit_index = self.queue.submit(std::iter::once(encoder.finish()));
        self.queue.present(surface_texture);

        if reconfigure_after_present {
            self.reconfigure_surface();
        }

        self.check_asynchronous_gpu_error()
    }

    fn check_asynchronous_gpu_error(&self) -> Result<(), SurfaceError> {
        match self.asynchronous_gpu_error.load(Ordering::Acquire) {
            GPU_ERROR_NONE => Ok(()),
            GPU_ERROR_OUT_OF_MEMORY => Err(SurfaceError::OutOfMemory),
            _ => Err(SurfaceError::Validation),
        }
    }
}

// ---------------------------------------------------------------------------
// Material creation helpers
// ---------------------------------------------------------------------------

fn create_white_fallback_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
) -> GpuMaterial {
    let size = wgpu::Extent3d {
        width: 1,
        height: 1,
        depth_or_array_layers: 1,
    };
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("fallback white texture"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &[255, 255, 255, 255],
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4),
            rows_per_image: Some(1),
        },
        size,
    );

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("fallback sampler"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::Repeat,
        address_mode_w: wgpu::AddressMode::Repeat,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    // G1D: procedural/fallback materials are explicit non-metals with high
    // roughness so terrain, scenery, and the procedural aircraft never turn
    // accidentally chromatic under the PBR response.
    let material_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("fallback material uniform"),
        contents: bytemuck::bytes_of(&MaterialUniform::new(
            PROCEDURAL_METALLIC,
            PROCEDURAL_ROUGHNESS,
        )),
        usage: wgpu::BufferUsages::UNIFORM,
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("fallback material bind group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: material_uniform_buffer.as_entire_binding(),
            },
        ],
    });

    GpuMaterial {
        _texture: texture,
        _texture_view: texture_view,
        _sampler: sampler,
        _material_uniform: material_uniform_buffer,
        bind_group,
    }
}

#[allow(clippy::too_many_arguments)]
fn create_gpu_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
    texture_data: &crate::texture::DecodedTexture,
    sampler_config: &SamplerConfig,
    metallic: f32,
    roughness: f32,
) -> Result<GpuMaterial, RendererError> {
    let size = wgpu::Extent3d {
        width: texture_data.width,
        height: texture_data.height,
        depth_or_array_layers: 1,
    };

    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("base color texture"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });

    let texture_view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    // FIX 5: create_staging_buffer now returns Result.
    let (staged_data, bytes_per_row) =
        create_staging_buffer(texture_data).map_err(RendererError::TextureUpload)?;

    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &staged_data,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(bytes_per_row),
            rows_per_image: Some(texture_data.height),
        },
        size,
    );

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("material sampler"),
        address_mode_u: sampler_config.wrap_s.to_wgpu(),
        address_mode_v: sampler_config.wrap_t.to_wgpu(),
        address_mode_w: wgpu::AddressMode::Repeat,
        mag_filter: sampler_config.mag_filter.to_wgpu(),
        min_filter: sampler_config.min_filter.to_wgpu(),
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    // G1D: static per-primitive metallic/roughness uniform. Written once at
    // asset upload; never touched per frame.
    let material_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("material uniform"),
        contents: bytemuck::bytes_of(&MaterialUniform::new(metallic, roughness)),
        usage: wgpu::BufferUsages::UNIFORM,
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("material bind group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: material_uniform_buffer.as_entire_binding(),
            },
        ],
    });

    Ok(GpuMaterial {
        _texture: texture,
        _texture_view: texture_view,
        _sampler: sampler,
        _material_uniform: material_uniform_buffer,
        bind_group,
    })
}

// ---------------------------------------------------------------------------
// G3A: terrain material creation
// ---------------------------------------------------------------------------

/// Create the textured terrain material from the embedded grass maps.
///
/// Decodes the committed PNGs once at initialization and uploads three
/// persistent textures: albedo (sRGB, hardware converts on sampling), normal
/// (linear RGBA), roughness (linear R8). One repeat/linear sampler serves all
/// three maps; the static uniform carries the PBR factors and per-map UV
/// anchors. No resource is created or recreated per frame.
fn create_terrain_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
    material: &TerrainMaterial,
) -> Result<GpuTerrainMaterial, RendererError> {
    let albedo = decode_image(terrain_assets::TERRAIN_ALBEDO_PNG)
        .map_err(RendererError::TextureUpload)?;
    let normal = decode_image(terrain_assets::TERRAIN_NORMAL_PNG)
        .map_err(RendererError::TextureUpload)?;
    let roughness = decode_image(terrain_assets::TERRAIN_ROUGHNESS_PNG)
        .map_err(RendererError::TextureUpload)?;

    let size = wgpu::Extent3d {
        width: albedo.width,
        height: albedo.height,
        depth_or_array_layers: 1,
    };

    let albedo_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A terrain albedo texture"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let albedo_texture_view = albedo_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let (albedo_data, albedo_row_bytes) =
        create_staging_buffer(&albedo).map_err(RendererError::TextureUpload)?;
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &albedo_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &albedo_data,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(albedo_row_bytes),
            rows_per_image: Some(albedo.height),
        },
        size,
    );

    let normal_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A terrain normal texture"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let normal_texture_view = normal_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let (normal_data, normal_row_bytes) =
        create_staging_buffer(&normal).map_err(RendererError::TextureUpload)?;
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &normal_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &normal_data,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(normal_row_bytes),
            rows_per_image: Some(normal.height),
        },
        size,
    );

    // Roughness: single R8 channel extracted from the gray PNG decode.
    let roughness_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A terrain roughness texture"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let roughness_texture_view =
        roughness_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let roughness_row_bytes = crate::texture::padded_bytes_per_row_checked(roughness.width)
        .ok_or(RendererError::TextureUpload(TextureLoadError::PaddedRowOverflow {
            width: roughness.width,
        }))?;
    let mut staged_roughness =
        Vec::with_capacity((roughness_row_bytes as usize) * (roughness.height as usize));
    for row in 0..roughness.height as usize {
        let start = row * roughness.width as usize * 4;
        staged_roughness.extend(
            roughness.rgba8[start..start + roughness.width as usize * 4]
                .iter()
                .step_by(4)
                .copied(),
        );
        staged_roughness.extend(std::iter::repeat_n(
            0u8,
            roughness_row_bytes as usize - roughness.width as usize,
        ));
    }
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &roughness_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &staged_roughness,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(roughness_row_bytes),
            rows_per_image: Some(roughness.height),
        },
        size,
    );

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("G3A terrain sampler"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::Repeat,
        address_mode_w: wgpu::AddressMode::Repeat,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    // G3A: terrain PBR factors + UV anchors, written once at load time.
    let material_uniform_buffer =
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("G3A terrain material uniform"),
            contents: bytemuck::bytes_of(&TerrainMaterialUniform::from_terrain_material(material)),
            usage: wgpu::BufferUsages::UNIFORM,
        });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("G3A terrain material bind group"),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&albedo_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(&normal_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(&roughness_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: material_uniform_buffer.as_entire_binding(),
            },
        ],
    });

    Ok(GpuTerrainMaterial {
        _albedo_texture: albedo_texture,
        _albedo_texture_view: albedo_texture_view,
        _normal_texture: normal_texture,
        _normal_texture_view: normal_texture_view,
        _roughness_texture: roughness_texture,
        _roughness_texture_view: roughness_texture_view,
        _sampler: sampler,
        _material_uniform: material_uniform_buffer,
        bind_group,
    })
}

// ---------------------------------------------------------------------------
// Scenery upload helper
// ---------------------------------------------------------------------------

fn upload_scenery_mesh(device: &wgpu::Device, mesh: &SceneryMesh) -> GpuScenery {
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("scenery vertices"),
        contents: bytemuck::cast_slice(&mesh.vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("scenery indices"),
        contents: bytemuck::cast_slice(&mesh.indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    GpuScenery {
        vertex_buffer,
        index_buffer,
        index_count: mesh.indices.len() as u32,
    }
}

// ---------------------------------------------------------------------------
// Bind group layout helpers
// ---------------------------------------------------------------------------

fn camera_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<CameraUniform>() as u64),
            },
            count: None,
        }],
    })
}

fn matrix_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<ObjectUniform>() as u64),
            },
            count: None,
        }],
    })
}

fn environment_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[
            // Existing environment/light/atmosphere uniform.
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(size_of::<EnvironmentUniform>() as u64),
                },
                count: None,
            },
            // G2B: comparison sample resources and light matrix. They extend
            // the established environment boundary while the lit pipeline
            // remains at groups 0..3.
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Comparison),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(size_of::<ShadowUniform>() as u64),
                },
                count: None,
            },
        ],
    })
}

/// G2B caster-pass group 2. It intentionally exposes only binding 3 from the
/// existing shadow uniform contract, so the depth target is never also bound
/// as a sampled texture while the directional pass writes it.
fn shadow_pass_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 3,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<ShadowUniform>() as u64),
            },
            count: None,
        }],
    })
}

fn material_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            // G1D: metallic/roughness material uniform.
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(size_of::<MaterialUniform>() as u64),
                },
                count: None,
            },
        ],
    })
}

/// G3A: terrain material bind group layout (group 4, bindings 0..4).
///
/// One filtering sampler serves all three maps; the uniform carries the PBR
/// factors and per-map world-space UV anchors. Distinct from the shared
/// material layout so the aircraft/scenery pipeline is untouched.
fn terrain_material_bind_group_layout(
    device: &wgpu::Device,
    label: &str,
) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[
            // Albedo (sRGB).
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            // Shared sampler.
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            // Normal (linear).
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            // Roughness (linear).
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            // Terrain material uniform (PBR factors + UV anchors).
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(
                        size_of::<TerrainMaterialUniform>() as u64,
                    ),
                },
                count: None,
            },
        ],
    })
}

fn camera_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: buffer.as_entire_binding(),
        }],
    })
}

fn matrix_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: buffer.as_entire_binding(),
        }],
    })
}

fn create_environment_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    environment_buffer: &wgpu::Buffer,
    shadow_texture_view: &wgpu::TextureView,
    shadow_sampler: &wgpu::Sampler,
    shadow_matrix_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: environment_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(shadow_texture_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(shadow_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: shadow_matrix_buffer.as_entire_binding(),
            },
        ],
    })
}

fn create_shadow_pass_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    shadow_matrix_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 3,
            resource: shadow_matrix_buffer.as_entire_binding(),
        }],
    })
}

// ---------------------------------------------------------------------------
// Pipeline creation helpers
// ---------------------------------------------------------------------------

struct PipelineSpec {
    topology: wgpu::PrimitiveTopology,
    cull_mode: Option<wgpu::Face>,
    depth_write_enabled: bool,
    label: &'static str,
    fragment_entry_point: &'static str,
}

fn create_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    format: wgpu::TextureFormat,
    spec: PipelineSpec,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(spec.label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[Some(wgpu::VertexBufferLayout {
                array_stride: size_of::<Vertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![
                    0 => Float32x3,
                    1 => Float32x3,
                    2 => Float32x4,
                    3 => Float32x2,
                ],
            })],
        },
        primitive: wgpu::PrimitiveState {
            topology: spec.topology,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: spec.cull_mode,
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(spec.depth_write_enabled),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(spec.fragment_entry_point),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// G2B: depth-only caster pipeline for the fixed directional shadow map.
///
/// It intentionally has no color target, material bind group, or fragment
/// entry point. Back-face culling matches the main scene pipeline; unlike a
/// front-face-only shadow pass it keeps thin RC wings and articulated control
/// surfaces from disappearing as casters.
fn create_shadow_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G2B directional shadow depth pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_shadow"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[Some(wgpu::VertexBufferLayout {
                array_stride: size_of::<Vertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![
                    0 => Float32x3,
                    1 => Float32x3,
                    2 => Float32x4,
                    3 => Float32x2,
                ],
            })],
        },
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: Some(wgpu::Face::Back),
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState {
                constant: SHADOW_DEPTH_BIAS_CONSTANT,
                slope_scale: SHADOW_DEPTH_BIAS_SLOPE_SCALE,
                clamp: 0.0,
            },
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: None,
        multiview_mask: None,
        cache: None,
    })
}

fn create_sky_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G1C sky pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_sky_fullscreen"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[],
        },
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: None,
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::Always),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_sky"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

fn create_depth_target(device: &wgpu::Device, width: u32, height: u32) -> DepthTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("depth target"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    DepthTarget {
        _texture: texture,
        view,
    }
}

fn create_shadow_target(device: &wgpu::Device) -> ShadowTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G2B directional shadow map"),
        size: wgpu::Extent3d {
            width: SHADOW_MAP_RESOLUTION,
            height: SHADOW_MAP_RESOLUTION,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    ShadowTarget {
        _texture: texture,
        view,
    }
}

/// Linear comparison filtering provides the small, stable hardware 2x2 PCF
/// footprint without a costly manually expanded fragment kernel.
fn create_shadow_comparison_sampler(device: &wgpu::Device) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("G2B directional shadow comparison sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Nearest,
        compare: Some(wgpu::CompareFunction::LessEqual),
        ..Default::default()
    })
}

#[cfg(test)]
mod glb_articulation_tests {
    use super::*;

    #[test]
    fn articulation_changes_only_transform_target_and_preserves_material_index() {
        let hinge =
            crate::SurfaceHinge::new(crate::SurfaceId::Elevator, [0.0; 3], [1.0, 0.0, 0.0], 1.0)
                .unwrap();
        let plan = crate::GlbArticulationPlan::from_mappings(2, [(1, hinge)]).unwrap();
        assert_eq!(
            glb_batch_target(Some(&plan), 0, 23),
            GlbBatchTarget {
                material_index: 23,
                hinge: None,
            }
        );
        assert_eq!(
            glb_batch_target(Some(&plan), 1, 41),
            GlbBatchTarget {
                material_index: 41,
                hinge: Some(hinge),
            }
        );
    }
}

#[cfg(test)]
mod material_uniform_tests {
    use super::*;

    // G1D: the material uniform must satisfy WGSL uniform buffer alignment.
    // Two f32 parameters plus padding round up to one 16-byte vec4 slot.
    #[test]
    fn material_uniform_layout_is_16_bytes() {
        assert_eq!(
            size_of::<MaterialUniform>(),
            16,
            "MaterialUniform must occupy exactly one WGSL vec4 slot"
        );
    }

    #[test]
    fn material_uniform_stores_finite_values() {
        let uniform = MaterialUniform::new(0.35, 0.7);
        assert_eq!(uniform.metallic, 0.35);
        assert_eq!(uniform.roughness, 0.7);
        assert!(uniform.metallic.is_finite() && uniform.roughness.is_finite());
        let bytes = bytemuck::bytes_of(&uniform);
        assert_eq!(bytes.len(), 16);
    }

    #[test]
    fn material_uniform_roundtrips_through_bytes() {
        let uniform = MaterialUniform::new(0.0, 1.0);
        let decoded: MaterialUniform = *bytemuck::from_bytes(bytemuck::bytes_of(&uniform));
        assert_eq!(decoded.metallic, 0.0);
        assert_eq!(decoded.roughness, 1.0);
        assert!(decoded.metallic.is_finite() && decoded.roughness.is_finite());
    }

    #[test]
    fn procedural_material_parameters_are_non_metal_and_rough() {
        // Terrain/scenery/procedural aircraft must never become chromed.
        assert_eq!(PROCEDURAL_METALLIC, 0.0);
        assert!(
            (0.5..=1.0).contains(&PROCEDURAL_ROUGHNESS),
            "procedural roughness must stay high for a matte response"
        );
    }
}

#[cfg(test)]
mod shader_brdf_regression_tests {
    //! G1D review-fix regression: the WGSL specular division must use an
    //! explicit positive denominator floor, because `ndot_l` is clamped to
    //! `>= 0` and can be exactly zero, which would otherwise leave a 0/0
    //! through the Smith geometry term (`gl = 0 / k`) and poison the final
    //! direct-light product (`0 * NaN = NaN`).
    //!
    //! This module mirrors the WGSL BRDF formulas verbatim and evaluates
    //! them in pure Rust; no GPU execution infrastructure is involved.

    // WGSL constants, mirrored verbatim from shader.wgsl.
    // `PI` is the f32-rounded value of the WGSL `const PI: f32 = 3.141592653589793`.
    const PI: f32 = std::f32::consts::PI;
    const MIN_ROUGHNESS: f32 = 0.06;
    const DIELECTRIC_F0: f32 = 0.04;
    const SPECULAR_CLAMP: f32 = 4.0;
    const SPECULAR_DENOMINATOR_FLOOR: f32 = 1e-4;

    fn mix3(dielectric: [f32; 3], base_color: [f32; 3], metallic: f32) -> [f32; 3] {
        [
            dielectric[0] + (base_color[0] - dielectric[0]) * metallic,
            dielectric[1] + (base_color[1] - dielectric[1]) * metallic,
            dielectric[2] + (base_color[2] - dielectric[2]) * metallic,
        ]
    }

    // Mirrors WGSL `schlick_fresnel`.
    fn schlick_fresnel(f0: [f32; 3], vdot_h: f32) -> [f32; 3] {
        let base = 1.0 - vdot_h;
        let f = base * base * base * base * base;
        [
            f0[0] + (1.0 - f0[0]) * f,
            f0[1] + (1.0 - f0[1]) * f,
            f0[2] + (1.0 - f0[2]) * f,
        ]
    }

    // Mirrors WGSL `ggx_distribution`.
    fn ggx_distribution(ndot_h: f32, alpha: f32) -> f32 {
        let alpha2 = alpha * alpha;
        let denom = ndot_h * ndot_h * (alpha2 - 1.0) + 1.0;
        alpha2 / (PI * denom * denom)
    }

    // Mirrors WGSL `smith_geometry`.
    fn smith_geometry(ndot_v: f32, ndot_l: f32, roughness: f32) -> f32 {
        let k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
        let gv = ndot_v / (ndot_v * (1.0 - k) + k);
        let gl = ndot_l / (ndot_l * (1.0 - k) + k);
        gv * gl
    }

    /// Mirrors the `fs_lit` specular + direct-light block, operand for
    /// operand, including the floored denominator and the `ndot_l` scale on
    /// the final product.
    fn direct_brdf(
        ndot_v: f32,
        ndot_l: f32,
        vdot_h: f32,
        ndot_h: f32,
        roughness: f32,
        metallic: f32,
        intensity: f32,
    ) -> [f32; 3] {
        let base_color = [0.5; 3];
        let f0 = mix3([DIELECTRIC_F0; 3], base_color, metallic);
        let diffuse_albedo = [
            base_color[0] * (1.0 - metallic),
            base_color[1] * (1.0 - metallic),
            base_color[2] * (1.0 - metallic),
        ];
        let alpha = roughness * roughness;
        let distribution = ggx_distribution(ndot_h, alpha);
        let geometry = smith_geometry(ndot_v, ndot_l, roughness);
        let fresnel = schlick_fresnel(f0, vdot_h);
        let specular_denominator = f32::max(4.0 * ndot_v * ndot_l, SPECULAR_DENOMINATOR_FLOOR);
        let specular_value = distribution * geometry / specular_denominator;
        let specular = [
            f32::min(specular_value * fresnel[0], SPECULAR_CLAMP),
            f32::min(specular_value * fresnel[1], SPECULAR_CLAMP),
            f32::min(specular_value * fresnel[2], SPECULAR_CLAMP),
        ];
        let direct_scale = PI * intensity * ndot_l;
        [
            (diffuse_albedo[0] / PI + specular[0]) * direct_scale,
            (diffuse_albedo[1] / PI + specular[1]) * direct_scale,
            (diffuse_albedo[2] / PI + specular[2]) * direct_scale,
        ]
    }

    fn assert_finite(direct: [f32; 3], context: &str) {
        assert!(
            direct.iter().all(|value| value.is_finite()),
            "direct BRDF must stay finite for {context}, got {direct:?}"
        );
    }

    #[test]
    fn direct_brdf_is_finite_and_zero_when_ndot_l_is_exactly_zero() {
        for roughness in [MIN_ROUGHNESS, 0.5, 1.0] {
            for &ndot_v in &[1e-4, 0.25, 0.5, 1.0] {
                let direct = direct_brdf(ndot_v, 0.0, 0.7, 0.8, roughness, 0.35, 0.8);
                assert_finite(
                    direct,
                    &format!("ndot_l=0, ndot_v={ndot_v}, roughness={roughness}"),
                );
                assert!(
                    direct.iter().all(|value| *value == 0.0),
                    "ndot_l=0 must yield an exactly zero direct response, got {direct:?}"
                );
            }
        }
    }

    #[test]
    fn direct_brdf_is_finite_when_ndot_l_is_very_close_to_zero() {
        for &ndot_l in &[1e-7, 1e-6, 1e-5, 1e-4] {
            for roughness in [MIN_ROUGHNESS, 0.06, 0.5, 1.0] {
                let direct = direct_brdf(1e-4, ndot_l, 0.7, 0.8, roughness, 0.35, 0.8);
                assert_finite(direct, &format!("ndot_l={ndot_l}, roughness={roughness}"));
            }
        }
    }

    #[test]
    fn direct_brdf_is_finite_at_ndot_v_floor_across_ndot_l() {
        for &ndot_l in &[0.0, 1e-7, 1e-3, 0.5, 1.0] {
            let direct = direct_brdf(1e-4, ndot_l, 0.0, 1.0, MIN_ROUGHNESS, 0.35, 0.8);
            assert_finite(direct, &format!("ndot_v=1e-4, ndot_l={ndot_l}"));
        }
    }

    #[test]
    fn direct_brdf_is_finite_at_minimum_roughness_across_directions() {
        for &ndot_v in &[1e-4, 0.5, 1.0] {
            for &ndot_l in &[0.0, 1e-7, 0.05, 0.5, 1.0] {
                for &vdot_h in &[0.0, 0.7, 1.0] {
                    let direct = direct_brdf(ndot_v, ndot_l, vdot_h, 0.8, MIN_ROUGHNESS, 0.0, 0.8);
                    assert_finite(
                        direct,
                        &format!("ndot_v={ndot_v}, ndot_l={ndot_l}, vdot_h={vdot_h}"),
                    );
                }
            }
        }
    }

    /// Pins the live WGSL (not just this mirror) to the floored denominator,
    /// so the shader and the reference math cannot drift apart again.
    #[test]
    fn shader_source_keeps_denominator_floor_and_ndot_l_scale() {
        let source = include_str!("shader.wgsl");
        assert!(
            source.contains("let specular_denominator = max(4.0 * ndot_v * ndot_l, 1e-4);"),
            "fs_lit must define the floored specular denominator"
        );
        assert!(
            source.contains("fresnel / specular_denominator,"),
            "fs_lit must divide the specular BRDF by the floored denominator"
        );
        assert!(
            source.contains("* irradiance * ndot_l;"),
            "fs_lit must keep ndot_l as the final direct-light multiplier"
        );
    }
}

#[cfg(test)]
mod directional_shadow_regression_tests {
    //! G2B structural guards. The light-space math itself lives in `shadow.rs`
    //! so it can be tested without wgpu; these tests pin the renderer/shader
    //! integration points that must not drift in later slices.

    #[test]
    fn shadow_uniform_has_matrix_plus_one_vec4_slot() {
        assert_eq!(
            std::mem::size_of::<super::ShadowUniform>(),
            80,
            "ShadowUniform must remain a mat4 plus one aligned vec4 slot"
        );
    }

    #[test]
    fn shadow_caster_pass_avoids_depth_texture_aliasing_and_resource_recreation() {
        let source = include_str!("gpu.rs");
        let (initialization_path, after_render) = source
            .split_once("pub fn render(&mut self, frame: &RenderFrame)")
            .expect("renderer source must expose the frame path");
        let (frame_path, _) = after_render
            .split_once("fn check_asynchronous_gpu_error")
            .expect("frame path must end before asynchronous error handling");
        let (_, after_shadow_pass_label) = frame_path
            .split_once("label: Some(\"G2B directional shadow depth pass\"),")
            .expect("frame path must contain the directional shadow pass");
        let (shadow_pass_path, _) = after_shadow_pass_label
            .split_once("// --- Sky pass")
            .expect("shadow pass must end before the main scene sky pass");

        assert!(
            initialization_path
                .contains("let shadow_pass_bind_group = create_shadow_pass_bind_group("),
            "the matrix-only shadow-pass bind group must be persistent"
        );
        assert!(
            initialization_path.contains("let shadow_target = create_shadow_target(&device);"),
            "the shadow depth target must remain persistent"
        );
        assert!(
            shadow_pass_path.contains("set_bind_group(2, &self.shadow_pass_bind_group, &[]);"),
            "caster pass must bind the matrix-only group at group 2"
        );
        assert!(
            !shadow_pass_path.contains("&self.environment_bind_group"),
            "caster pass must not bind the sampled depth texture while writing it"
        );
        for forbidden_creation in [
            "create_shadow_target(",
            "create_shadow_pipeline(",
            "create_environment_bind_group(",
            "create_shadow_pass_bind_group(",
            "create_texture(",
            "create_bind_group(",
            "create_render_pipeline(",
        ] {
            assert!(
                !frame_path.contains(forbidden_creation),
                "frame path must not recreate {forbidden_creation}"
            );
        }
        assert!(
            frame_path.contains("&self.shadow_matrix_buffer"),
            "frame path should only update the persistent shadow matrix buffer"
        );
    }

    #[test]
    fn shader_keeps_shadows_on_direct_light_only_and_fogs_after_lighting() {
        let source = include_str!("shader.wgsl");
        let direct = source
            .find("let direct = direct_unshadowed * shadow_visibility;")
            .expect("direct PBR lighting must be multiplied by shadow visibility");
        let ambient = source
            .find("let lit_rgb = direct + ambient;")
            .expect("ambient must remain outside the shadow multiplier");
        let fog = source
            .find("let final_rgb = mix(lit_rgb, fog_color, fog);")
            .expect("fog must continue to be applied after lighting");
        assert!(direct < ambient && ambient < fog);
        assert!(
            source.contains("return textureSampleCompare("),
            "directional shadows must use the comparison sampler path"
        );
    }
}

#[cfg(test)]
mod terrain_material_uniform_tests {
    use super::*;

    #[test]
    fn terrain_material_uniform_layout_is_64_bytes() {
        // WGSL: four vec4 slots (metallic/roughness/normal_strength/pad,
        // three vec2 anchors, pad2). Must match the shader struct exactly.
        assert_eq!(
            size_of::<TerrainMaterialUniform>(),
            64,
            "TerrainMaterialUniform must occupy exactly four WGSL vec4 slots"
        );
    }

    #[test]
    fn terrain_material_uniform_roundtrips_through_bytes() {
        let material = TerrainMaterial::default();
        let uniform = TerrainMaterialUniform::from_terrain_material(&material);
        assert_eq!(uniform.metallic, 0.0);
        assert_eq!(uniform.roughness, 0.9);
        assert_eq!(uniform.normal_strength, 1.0);
        assert_eq!(uniform.albedo_uv_offset, [0.0, 0.0]);
        assert_eq!(uniform.normal_uv_offset, [0.271, 0.137]);
        assert_eq!(uniform.roughness_uv_offset, [0.413, 0.303]);

        let decoded: TerrainMaterialUniform =
            *bytemuck::from_bytes(bytemuck::bytes_of(&uniform));
        assert_eq!(decoded.metallic, 0.0);
        assert_eq!(decoded.roughness, 0.9);
        assert_eq!(decoded.normal_uv_offset, [0.271, 0.137]);
        assert!(decoded.roughness.is_finite());
        assert!(decoded.normal_strength.is_finite());
    }

    #[test]
    fn terrain_material_uniform_clamps_factors_at_load() {
        // Factors outside [0, 1] are clamped once at load; the shader then
        // applies its own MIN_ROUGHNESS floor, keeping every response finite.
        let material = TerrainMaterial {
            metallic: 1.7,
            roughness: 0.02,
            normal_strength: 1.4,
            ..Default::default()
        };
        let uniform = TerrainMaterialUniform::from_terrain_material(&material);
        assert_eq!(uniform.metallic, 1.0);
        assert_eq!(uniform.roughness, 0.02);
        assert_eq!(uniform.normal_strength, 1.0);
    }
}

#[cfg(test)]
mod terrain_gpu_integration_guards {
    //! G3A structural guards: the terrain shader/renderer integration points
    //! that must not drift in later slices.

    #[test]
    fn shader_terrain_entry_reuses_pbr_and_stays_textured() {
        let source = include_str!("shader.wgsl");
        assert!(
            source.contains("fn fs_terrain(input: VertexOutput)"),
            "terrain fragment entry must exist"
        );
        assert!(
            source.contains("fn lit_pbr_response("),
            "terrain must reuse the shared PBR response"
        );
        let terrain_block = source
            .split("fn fs_terrain(input: VertexOutput)")
            .nth(1)
            .expect("terrain fragment entry must exist");
        assert!(
            terrain_block.contains("input.color * albedo_rgba"),
            "G2D vertex color must modulate the albedo texture"
        );
        assert!(
            terrain_block.contains("terrain_material.roughness * roughness_sample"),
            "roughness map must scale the material base roughness"
        );
        assert!(
            terrain_block.contains("dpdx(input.world_position)"),
            "terrain TBN must be derivative-based (chunk-independent)"
        );
        assert!(
            terrain_block.contains("directional_shadow_visibility(")
                || terrain_block.contains("lit_pbr_response("),
            "terrain must keep G2B shadow receiving via the shared path"
        );
    }

    #[test]
    fn renderer_terrain_resources_are_created_once() {
        let source = include_str!("gpu.rs");
        assert!(
            source.contains("fn create_terrain_material("),
            "terrain material creation must be a startup helper"
        );
        assert!(
            source.contains("\"fs_terrain\""),
            "terrain pipeline must use the fs_terrain entry point"
        );
        assert!(
            source.contains("set_bind_group(4, &self.terrain_material.bind_group, &[]);"),
            "terrain draws must bind their own material at group 4"
        );

        let (initialization_path, after_render) = source
            .split_once("pub fn render(&mut self, frame: &RenderFrame)")
            .expect("renderer source must expose the frame path");
        let (frame_path, _) = after_render
            .split_once("fn check_asynchronous_gpu_error")
            .expect("frame path must end before asynchronous error handling");
        for (needle, label) in [
            ("create_terrain_material(", "terrain material"),
            ("decode_image(", "terrain map decode"),
            ("terrain_material_bind_group_layout(", "terrain layout"),
            ("create_texture(", "texture"),
            ("create_sampler(", "sampler"),
            ("create_bind_group(", "bind group"),
        ] {
            assert!(
                !frame_path.contains(needle),
                "frame path must not recreate the {label} resource"
            );
        }
        assert!(
            initialization_path.contains("let terrain_material_gpu = create_terrain_material("),
            "terrain material must be created once at startup"
        );
    }
}
