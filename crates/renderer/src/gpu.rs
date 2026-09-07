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
use crate::terrain_textures::{
    TERRAIN_TEXTURE_SIZE, generate_terrain_mip_chain, mip_level_count_for_size,
    terrain_texture_set_from_decoded,
};
use crate::texture::{
    SamplerConfig, TextureLoadError, create_staging_buffer, decode_image,
    padded_bytes_per_row_checked_for_bytes_per_pixel,
};
use crate::{
    AircraftMesh, CameraConfig, CameraMode, GlbAsset, Mat4, RenderFrame, Vertex,
    matrix_to_wgsl_columns, reference_grid_and_axes_at,
};
use bytemuck::{Pod, Zeroable};
use std::f32::consts::PI;
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
// G3B: linear HDR scene target. Opaque geometry, terrain, aircraft, sky and
// lighting write scene-referred linear values here; the postprocess pass
// resolves exposure + tone mapping to the sRGB surface.
const HDR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
const GPU_ERROR_NONE: u8 = 0;
const GPU_ERROR_OUT_OF_MEMORY: u8 = 1;
const GPU_ERROR_OTHER: u8 = 2;
pub const SKY_CLEAR_COLOR: [f64; 4] = [0.42, 0.68, 0.92, 1.0];

const DEFAULT_LIGHT_DIRECTION: [f32; 3] = [0.4, 0.8, -0.3];
const DEFAULT_LIGHT_INTENSITY: f32 = 0.80;

// G3B: deterministic analytic sky response parameters.
// Sky diffuse: hemispherical irradiance scale applied to the procedural
// zenith/horizon/ground gradient, plus a sun-facing lift weight. Chosen so
// the unshadowed field keeps its established energy while shadow sides stay
// readable (no flat fake fill).
const DEFAULT_SKY_DIFFUSE_RGB: [f32; 3] = [0.80, 0.85, 0.95];
const DEFAULT_SKY_DIFFUSE_SUN_LIFT: f32 = 0.15;
// Environment specular: analytic sky reflection tint and strength through the
// PBR path (roughness/metallic aware); pre-wired for future prefiltered IBL.
const DEFAULT_ENV_SPECULAR_RGB: [f32; 3] = [1.0, 1.0, 1.0];
const DEFAULT_ENV_SPECULAR_STRENGTH: f32 = 1.0;

// G3B: manual exposure (EV). Default outdoor value keeps the current scene
// energy; the multiplier is exp2(ev). Values outside the conservative band
// are rejected by the validator (presentation-only, never physics).
pub const DEFAULT_EXPOSURE_EV: f32 = 0.0;
const EXPOSURE_EV_MIN: f32 = -8.0;
const EXPOSURE_EV_MAX: f32 = 8.0;

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
    // G3B: analytic sky response (see WGSL struct docs).
    sky_diffuse: [f32; 4],
    env_specular: [f32; 4],
}

/// G3B: postprocess state (exposure EV) matching the WGSL `PostProcessUniform`.
///
/// The only uniform written in the frame path besides the camera, object and
/// shadow matrices; the buffer is UNIFORM | COPY_DST created once at startup.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct PostProcessUniform {
    exposure_ev: f32,
    padding: [f32; 3],
}

impl PostProcessUniform {
    fn new(exposure_ev: f32) -> Self {
        Self {
            exposure_ev,
            padding: [0.0; 3],
        }
    }
}

/// G3B: exposure validation error. Presentation-only — exposure NEVER enters
/// the physics/model fingerprint.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ExposureError {
    #[error("exposure EV `{0}` is not finite")]
    NotFinite(f32),
    #[error(
        "exposure EV `{0}` is outside the supported range [{EXPOSURE_EV_MIN}, {EXPOSURE_EV_MAX}]"
    )]
    OutOfRange(f32),
}

/// Validates a manual exposure value in EV stops.
///
/// Rejects non-finite inputs and absurd values; the accepted band keeps the
/// tone mapper response monotone and finite for any scene-referred value.
pub fn validate_exposure_ev(exposure_ev: f32) -> Result<f32, ExposureError> {
    if !exposure_ev.is_finite() {
        return Err(ExposureError::NotFinite(exposure_ev));
    }
    if !(EXPOSURE_EV_MIN..=EXPOSURE_EV_MAX).contains(&exposure_ev) {
        return Err(ExposureError::OutOfRange(exposure_ev));
    }
    Ok(exposure_ev)
}

/// G3B: exposure multiplier for a manual EV stop value: `exp2(ev)`.
#[must_use]
pub fn exposure_multiplier(exposure_ev: f32) -> f32 {
    2.0f32.powf(exposure_ev)
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
                // G3B: legacy flat ambient retired — zeroed, the sky-diffuse
                // model owns the non-direct response now.
                0.0, 0.0, 0.0, 0.0,
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
            sky_diffuse: [
                DEFAULT_SKY_DIFFUSE_RGB[0],
                DEFAULT_SKY_DIFFUSE_RGB[1],
                DEFAULT_SKY_DIFFUSE_RGB[2],
                DEFAULT_SKY_DIFFUSE_SUN_LIFT,
            ],
            env_specular: [
                DEFAULT_ENV_SPECULAR_RGB[0],
                DEFAULT_ENV_SPECULAR_RGB[1],
                DEFAULT_ENV_SPECULAR_RGB[2],
                DEFAULT_ENV_SPECULAR_STRENGTH,
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

/// G3A-R: terrain debug presentation mode (uniform-driven, no shader recompiles).
///
/// `Final` is the production default; any other mode bypasses lighting and
/// fog and outputs the selected material channel for inspection. Debug mode
/// is presentation-only — it never feeds back into physics or animation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TerrainDebugMode {
    /// Full lit PBR terrain (default).
    #[default]
    Final = 0,
    /// Composite albedo stack (base + rotated second sample + macro/detail).
    Albedo = 1,
    /// Tangent-space normal after the distance-faded detail blend, linear [0,1] view.
    Normal = 2,
    /// Final roughness scalar used by the BRDF.
    Roughness = 3,
    /// Macro layer albedo sample.
    Macro = 4,
    /// Detail layer albedo sample.
    Detail = 5,
}

impl TerrainDebugMode {
    /// WGSL uniform value (0 = FINAL).
    #[must_use]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }

    /// Map a WGSL uniform value back to a mode.
    #[must_use]
    pub const fn from_u32(value: u32) -> Option<Self> {
        match value {
            0 => Some(Self::Final),
            1 => Some(Self::Albedo),
            2 => Some(Self::Normal),
            3 => Some(Self::Roughness),
            4 => Some(Self::Macro),
            5 => Some(Self::Detail),
            _ => None,
        }
    }

    /// CLI/config label for the mode.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Final => "final",
            Self::Albedo => "albedo",
            Self::Normal => "normal",
            Self::Roughness => "roughness",
            Self::Macro => "macro",
            Self::Detail => "detail",
        }
    }

    /// Parse a CLI/config label.
    #[must_use]
    pub fn from_label(label: &str) -> Option<Self> {
        match label {
            "final" => Some(Self::Final),
            "albedo" => Some(Self::Albedo),
            "normal" => Some(Self::Normal),
            "roughness" => Some(Self::Roughness),
            "macro" => Some(Self::Macro),
            "detail" => Some(Self::Detail),
            _ => None,
        }
    }
}

/// G3A-R: terrain material uniform matching the WGSL `TerrainMaterialUniform`
/// struct (eight 16-byte vec4 slots, 128 bytes total).
///
/// G3A slots carry the PBR factors and per-map world-space UV anchors. G3A-R
/// adds: the three-frequency stack (base/detail/macro scales and per-layer
/// anchors), the anti-repetition rotated second-sample transform
/// (cos/sin angle, scale, offset), the detail-normal distance fade range, and
/// the debug mode selector. All values are world-anchored, chunk-invariant
/// configuration written once at load time (except `debug_mode`, which is
/// updated only when the debug channel changes).
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct TerrainMaterialUniform {
    // Slot 0: PBR factors + debug selector.
    metallic: f32,
    roughness: f32,
    normal_strength: f32,
    debug_mode: u32,
    // Slot 1: three-frequency stack scales (world metres).
    base_scale_m: f32,
    detail_scale_m: f32,
    macro_scale_m: f32,
    _padding1: f32,
    // Slot 2: base albedo + base normal anchors.
    albedo_uv_offset: [f32; 2],
    normal_uv_offset: [f32; 2],
    // Slot 3: base roughness anchor + detail layer anchor.
    roughness_uv_offset: [f32; 2],
    detail_uv_offset: [f32; 2],
    // Slot 4: macro layer anchor + anti-repetition angle (cos, sin).
    macro_uv_offset: [f32; 2],
    ar_angle_cos_sin: [f32; 2],
    // Slot 5: anti-repetition scale + offset (xy) + padding.
    ar_scale_offset: [f32; 4],
    // Slot 6: detail-normal fade near/far (xy) + padding.
    detail_fade_near_far: [f32; 4],
    // Slot 7: reserved.
    _padding2: [f32; 4],
}

impl TerrainMaterialUniform {
    fn from_terrain_material(material: &TerrainMaterial, debug_mode: TerrainDebugMode) -> Self {
        let radians = material.ar_angle_degrees * PI / 180.0;
        // The explicit `debug_mode` argument wins (headless probes); the
        // material-carried default covers the production startup path.
        let resolved = if debug_mode == TerrainDebugMode::Final {
            material.debug_mode
        } else {
            debug_mode
        };
        Self {
            metallic: material.metallic.clamp(0.0, 1.0),
            roughness: material.roughness.clamp(0.0, 1.0),
            normal_strength: material.normal_strength.clamp(0.0, 1.0),
            debug_mode: resolved.as_u32(),
            base_scale_m: material.texture_scale_m.max(1e-3),
            detail_scale_m: material.detail_scale_m.max(1e-3),
            macro_scale_m: material.macro_scale_m.max(1e-3),
            _padding1: 0.0,
            albedo_uv_offset: material.albedo_uv_offset,
            normal_uv_offset: material.normal_uv_offset,
            roughness_uv_offset: material.roughness_uv_offset,
            detail_uv_offset: material.detail_uv_offset,
            macro_uv_offset: material.macro_uv_offset,
            ar_angle_cos_sin: [radians.cos(), radians.sin()],
            ar_scale_offset: [
                material.ar_scale.max(1e-3),
                material.ar_offset[0],
                material.ar_offset[1],
                0.0,
            ],
            detail_fade_near_far: [
                material.detail_normal_fade_near_m,
                material.detail_normal_fade_far_m,
                0.0,
                0.0,
            ],
            _padding2: [0.0; 4],
        }
    }

    /// Rebuild this uniform with a different debug mode (same material).
    fn with_debug_mode(&self, debug_mode: TerrainDebugMode) -> Self {
        let mut updated = *self;
        updated.debug_mode = debug_mode.as_u32();
        updated
    }
}

struct DepthTarget {
    _texture: wgpu::Texture,
    view: wgpu::TextureView,
}

/// G3B: persistent linear HDR scene target (Rgba16Float).
///
/// Opaque geometry, terrain, aircraft, sky and lighting write scene-referred
/// linear values here; the postprocess pass samples it and resolves exposure +
/// tone mapping to the sRGB surface. Created at startup and recreated only on
/// resize — never per frame. The view is re-bound on resize.
struct HdrTarget {
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

/// G3A-R: persistent GPU terrain material (dedicated bind group at group 4).
///
/// Owns the mipmapped albedo (sRGB), normal (linear), and roughness (linear)
/// textures plus one shared repeat/trilinear sampler with anisotropic
/// filtering clamped to the device capability, and the material uniform.
/// Created once at renderer initialization, never recreated per frame. The
/// uniform buffer is rewritten in place only when the debug mode changes
/// (presentation-only, never on the frame path).
struct GpuTerrainMaterial {
    _albedo_texture: wgpu::Texture,
    _albedo_texture_view: wgpu::TextureView,
    _normal_texture: wgpu::Texture,
    _normal_texture_view: wgpu::TextureView,
    _roughness_texture: wgpu::Texture,
    _roughness_texture_view: wgpu::TextureView,
    _sampler: wgpu::Sampler,
    /// Effective sampler anisotropy on this device (1 or 16).
    sampler_anisotropy: u32,
    material_uniform: wgpu::Buffer,
    uniform: TerrainMaterialUniform,
    bind_group: wgpu::BindGroup,
}

impl GpuTerrainMaterial {
    /// Switch the presentation-only debug channel. Rewrites the existing
    /// uniform buffer in place; no resource is created and nothing allocates.
    fn update_debug_mode(&mut self, queue: &wgpu::Queue, debug_mode: TerrainDebugMode) {
        self.uniform = self.uniform.with_debug_mode(debug_mode);
        queue.write_buffer(&self.material_uniform, 0, bytemuck::bytes_of(&self.uniform));
    }
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
    // G3B: fullscreen postprocess pipeline (HDR -> exposure -> tone map).
    postprocess_pipeline: wgpu::RenderPipeline,

    _camera_bind_group_layout: wgpu::BindGroupLayout,
    _object_bind_group_layout: wgpu::BindGroupLayout,
    _environment_bind_group_layout: wgpu::BindGroupLayout,
    _shadow_pass_bind_group_layout: wgpu::BindGroupLayout,
    _material_bind_group_layout: wgpu::BindGroupLayout,
    // G3A: extended terrain material layout (albedo/normal/roughness + uniform).
    _terrain_material_bind_group_layout: wgpu::BindGroupLayout,
    // G3B: dedicated postprocess layout (HDR texture + sampler + uniform).
    _postprocess_bind_group_layout: wgpu::BindGroupLayout,

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
    // G3B: linear HDR scene target, postprocess sampler/uniform/bind group.
    hdr_target: HdrTarget,
    postprocess_sampler: wgpu::Sampler,
    postprocess_uniform_buffer: wgpu::Buffer,
    postprocess_bind_group: wgpu::BindGroup,
    // G3B: presentation-only manual exposure in EV stops (validated).
    exposure_ev: f32,
    _shadow_sampler: wgpu::Sampler,
    camera: CameraMode,
    asynchronous_gpu_error: Arc<AtomicU8>,

    show_debug_overlays: bool,
    // G3A-R: presentation-only terrain debug channel (uniform-driven).
    terrain_debug_mode: TerrainDebugMode,
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
                // G3A: the terrain material bind group lives at group 4, one
                // beyond the WebGPU default `max_bind_groups` of 4. Native
                // backends advertise up to 8; raising the cap to the adapter's
                // own advertised value keeps the pipeline valid on every device.
                required_limits: wgpu::Limits {
                    max_bind_groups: adapter.limits().max_bind_groups,
                    ..wgpu::Limits::default()
                },
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: wgpu::MemoryHints::Performance,
                trace: wgpu::Trace::Off,
            })
            .await
            .map_err(|error| RendererError::RequestDevice(error.to_string()))?;

        // G3A-R: anisotropic filtering is a downlevel capability in wgpu 30.
        // When the backend supports it, request the WebGPU maximum of 16x
        // (the RTX 3090 target supports it natively); otherwise fall back to
        // 1x so less capable adapters stay valid. The sampler is created with
        // this value; wgpu additionally clamps to [1, 16] internally.
        let terrain_sampler_anisotropy = effective_sampler_anisotropy(
            adapter
                .get_downlevel_capabilities()
                .flags
                .contains(wgpu::DownlevelFlags::ANISOTROPIC_FILTERING),
        );

        let asynchronous_gpu_error = Arc::new(AtomicU8::new(GPU_ERROR_NONE));
        let callback_error = Arc::clone(&asynchronous_gpu_error);
        device.on_uncaptured_error(Arc::new(move |error| {
            let code = match error {
                wgpu::Error::OutOfMemory { .. } => GPU_ERROR_OUT_OF_MEMORY,
                wgpu::Error::Validation { .. } | wgpu::Error::Internal { .. } => GPU_ERROR_OTHER,
            };
            // Debug aid: surface the wgpu validation detail that the atomic
            // only collapses to a code. Correctly diagnosed slices have no
            // uncaptured errors, so this line stays silent in production.
            eprintln!("GpuValidation diagnostic: {error}");
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
            terrain_sampler_anisotropy,
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

        // G3B: linear HDR scene target + postprocess pass, created once here
        // (and recreated on resize only). No texture/sampler/bind group/
        // pipeline creation happens on the frame path.
        let hdr_target = create_hdr_target(
            &device,
            surface_configuration.width,
            surface_configuration.height,
        );
        let postprocess_bind_group_layout =
            postprocess_bind_group_layout(&device, "G3B postprocess layout");
        let postprocess_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("G3B postprocess sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            // 1:1 texel mapping from the fullscreen triangle; nearest keeps
            // the resolve deterministic and avoids cross-texel bleed.
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::MipmapFilterMode::Nearest,
            ..Default::default()
        });
        let postprocess_uniform_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("G3B postprocess uniform"),
                contents: bytemuck::bytes_of(&PostProcessUniform::new(DEFAULT_EXPOSURE_EV)),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
        let postprocess_bind_group = create_hdr_scene_bind_group(
            &device,
            &postprocess_bind_group_layout,
            &hdr_target.view,
            &postprocess_sampler,
            &postprocess_uniform_buffer,
            "G3B postprocess bind group",
        );
        // The postprocess rasterizes a fullscreen triangle on the sRGB
        // surface; the HDR target is sampled in its own fragment entry.
        let postprocess_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G3B postprocess pipeline layout"),
                bind_group_layouts: &[Some(&postprocess_bind_group_layout)],
                immediate_size: 0,
            });
        let postprocess_pipeline =
            create_postprocess_pipeline(&device, &shader, &postprocess_pipeline_layout, format);

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
            postprocess_pipeline,
            _camera_bind_group_layout: camera_bind_group_layout,
            _object_bind_group_layout: object_bind_group_layout,
            _environment_bind_group_layout: environment_bind_group_layout,
            _shadow_pass_bind_group_layout: shadow_pass_bind_group_layout,
            _material_bind_group_layout: material_bind_group_layout,
            _terrain_material_bind_group_layout: terrain_material_bind_group_layout,
            _postprocess_bind_group_layout: postprocess_bind_group_layout,
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
            hdr_target,
            postprocess_sampler,
            postprocess_uniform_buffer,
            postprocess_bind_group,
            exposure_ev: DEFAULT_EXPOSURE_EV,
            _shadow_sampler: shadow_sampler,
            camera: camera_config.build(size.width, size.height),
            asynchronous_gpu_error,
            show_debug_overlays: false,
            terrain_debug_mode: TerrainDebugMode::default(),
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

    /// G3A-R: switch the presentation-only terrain debug channel.
    ///
    /// Writes the debug selector into the existing terrain material uniform;
    /// no shader recompile, no resource creation, and no per-frame work. The
    /// default is [`TerrainDebugMode::Final`], which is the production path.
    pub fn set_terrain_debug_mode(&mut self, mode: TerrainDebugMode) {
        self.terrain_debug_mode = mode;
        self.terrain_material.update_debug_mode(&self.queue, mode);
    }

    /// Current terrain debug channel.
    #[must_use]
    pub fn terrain_debug_mode(&self) -> TerrainDebugMode {
        self.terrain_debug_mode
    }

    /// G3B: set the presentation-only manual exposure (EV stops).
    ///
    /// The value is validated (finite, bounded) before it replaces the
    /// current exposure; invalid values are rejected and leave the state
    /// unchanged. Exposure is applied only in the postprocess pass — it never
    /// enters physics, interpolation, or the model fingerprint.
    pub fn set_exposure_ev(&mut self, exposure_ev: f32) -> Result<(), ExposureError> {
        validate_exposure_ev(exposure_ev)?;
        self.exposure_ev = exposure_ev;
        Ok(())
    }

    /// G3B: current manual exposure in EV stops.
    #[must_use]
    pub fn exposure_ev(&self) -> f32 {
        self.exposure_ev
    }

    /// Effective terrain sampler anisotropy on this device
    /// (`1` when the backend lacks `ANISOTROPIC_FILTERING`, `16` otherwise).
    #[must_use]
    pub fn terrain_sampler_anisotropy(&self) -> u32 {
        self.terrain_material.sampler_anisotropy
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
        // G3B: the linear HDR scene target tracks the surface size; the
        // postprocess bind group is re-created to reference the new view.
        // Pipelines, sampler and uniform buffer are NOT recreated here.
        self.hdr_target = create_hdr_target(&self.device, width, height);
        self.postprocess_bind_group = create_hdr_scene_bind_group(
            &self.device,
            &self._postprocess_bind_group_layout,
            &self.hdr_target.view,
            &self.postprocess_sampler,
            &self.postprocess_uniform_buffer,
            "G3B postprocess bind group (resized)",
        );
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

        // G3B: postprocess exposure (presentation-only). One 16-byte write to
        // the persistent uniform buffer; no resource is created per frame.
        self.queue.write_buffer(
            &self.postprocess_uniform_buffer,
            0,
            bytemuck::bytes_of(&PostProcessUniform::new(self.exposure_ev)),
        );

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
            // G3B: the scene pass now renders to the linear HDR target. The
            // surface receives only the resolved postprocess output below.
            let color_attachment = wgpu::RenderPassColorAttachment {
                view: &self.hdr_target.view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    // Scene-referred HDR clear: the procedural sky pass covers
                    // the full viewport, so this is only a safety fill.
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.0,
                        g: 0.0,
                        b: 0.0,
                        a: 1.0,
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
        {
            // G3B: fullscreen postprocess pass — samples the linear HDR scene
            // target, applies the manual exposure and the Khronos PBR Neutral
            // tone mapper, and writes display values to the sRGB surface. The
            // surface sRGB format performs the final linear->sRGB encode.
            let display_attachment = wgpu::RenderPassColorAttachment {
                view: &surface_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            };
            let mut postprocess_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("G3B HDR postprocess pass"),
                color_attachments: &[Some(display_attachment)],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            postprocess_pass.set_pipeline(&self.postprocess_pipeline);
            postprocess_pass.set_bind_group(0, &self.postprocess_bind_group, &[]);
            postprocess_pass.draw(0..3, 0..1);
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
// G3A-R: terrain material creation
// ---------------------------------------------------------------------------

/// Effective terrain sampler anisotropy: 16x when the backend supports
/// anisotropic filtering (the WebGPU maximum, and the RTX 3090 native
/// capability), 1x otherwise so less capable adapters stay valid.
#[must_use]
fn effective_sampler_anisotropy(anisotropic_supported: bool) -> u16 {
    if anisotropic_supported { 16 } else { 1 }
}

/// Upload one mip level with WebGPU row alignment applied for the given
/// `bytes_per_pixel` (4 for RGBA8 maps, 1 for the R8 roughness map).
///
/// Pads each row to `COPY_BYTES_PER_ROW_ALIGNMENT` into a temporary staging
/// buffer. Called once per level at initialization; the frame path never
/// allocates or uploads.
fn upload_terrain_mip_level(
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
    level: u32,
    width: u32,
    height: u32,
    bytes: &[u8],
    bytes_per_pixel: u32,
) -> Result<(), RendererError> {
    let row_bytes = padded_bytes_per_row_checked_for_bytes_per_pixel(width, bytes_per_pixel)
        .ok_or(RendererError::TextureUpload(
            TextureLoadError::PaddedRowOverflow { width },
        ))?;
    let unpadded = width as usize * bytes_per_pixel as usize;
    let padding = row_bytes as usize - unpadded;
    let mut staged = Vec::with_capacity(row_bytes as usize * height as usize);
    for row in 0..height as usize {
        let start = row * unpadded;
        staged.extend_from_slice(&bytes[start..start + unpadded]);
        staged.extend(std::iter::repeat_n(0u8, padding));
    }
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: level,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &staged,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(row_bytes),
            rows_per_image: Some(height),
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    Ok(())
}

/// Create the textured, mipmapped terrain material from the embedded maps.
///
/// Decodes the committed PNGs once at initialization, builds the full
/// deterministic mip chain (512 -> 1, 10 levels: sRGB-correct albedo,
/// renormalized normals, linear roughness averages), and uploads all levels
/// into three persistent textures. One repeat/trilinear sampler with
/// anisotropic filtering (clamped to the device capability) serves all three
/// maps; the uniform carries the PBR factors, the three-frequency stack, the
/// anti-repetition transform, and the debug selector. No resource is created
/// or recreated per frame.
fn create_terrain_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
    material: &TerrainMaterial,
    sampler_anisotropy: u16,
) -> Result<GpuTerrainMaterial, RendererError> {
    let albedo =
        decode_image(terrain_assets::TERRAIN_ALBEDO_PNG).map_err(RendererError::TextureUpload)?;
    let normal =
        decode_image(terrain_assets::TERRAIN_NORMAL_PNG).map_err(RendererError::TextureUpload)?;
    let roughness = decode_image(terrain_assets::TERRAIN_ROUGHNESS_PNG)
        .map_err(RendererError::TextureUpload)?;

    // G3A-R: assemble the base set (the gray roughness decode carries the
    // linear R channel) and generate the deterministic mip chain once, at
    // initialization. Missing mips would silently downgrade sampling to
    // mip-0-only bilinear, so the level count is asserted up front.
    let base_set = terrain_texture_set_from_decoded(&albedo.rgba8, &normal.rgba8, &roughness.rgba8);
    let mip_chain = generate_terrain_mip_chain(&base_set, TERRAIN_TEXTURE_SIZE);
    let mip_levels = mip_chain.albedo.len() as u32;
    debug_assert_eq!(
        mip_levels,
        mip_level_count_for_size(TERRAIN_TEXTURE_SIZE),
        "terrain textures must carry the full mip chain"
    );
    assert!(
        mip_levels >= 2
            && mip_chain.normal.len() == mip_levels as usize
            && mip_chain.roughness.len() == mip_levels as usize,
        "all three terrain maps must carry the full mip chain"
    );

    let size = wgpu::Extent3d {
        width: albedo.width,
        height: albedo.height,
        depth_or_array_layers: 1,
    };

    let albedo_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A-R terrain albedo texture"),
        size,
        mip_level_count: mip_levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        // COPY_SRC: enables GPU readback verification of the committed asset
        // bytes (headless tests); otherwise inert. Never copied in a frame.
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let albedo_texture_view = albedo_texture.create_view(&wgpu::TextureViewDescriptor::default());
    for (level, mip) in mip_chain.albedo.iter().enumerate() {
        upload_terrain_mip_level(
            queue,
            &albedo_texture,
            level as u32,
            mip.width,
            mip.height,
            &mip.bytes,
            4,
        )?;
    }

    let normal_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A-R terrain normal texture"),
        size,
        mip_level_count: mip_levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let normal_texture_view = normal_texture.create_view(&wgpu::TextureViewDescriptor::default());
    for (level, mip) in mip_chain.normal.iter().enumerate() {
        upload_terrain_mip_level(
            queue,
            &normal_texture,
            level as u32,
            mip.width,
            mip.height,
            &mip.bytes,
            4,
        )?;
    }

    // Roughness: single R8 channel, linear data.
    let roughness_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3A-R terrain roughness texture"),
        size,
        mip_level_count: mip_levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::COPY_DST
            | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let roughness_texture_view =
        roughness_texture.create_view(&wgpu::TextureViewDescriptor::default());
    for (level, mip) in mip_chain.roughness.iter().enumerate() {
        upload_terrain_mip_level(
            queue,
            &roughness_texture,
            level as u32,
            mip.width,
            mip.height,
            &mip.bytes,
            1,
        )?;
    }

    // G3A-R: trilinear filtering with anisotropy clamped to the device
    // capability (16x on capable backends, 1x otherwise).
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("G3A-R terrain sampler"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::Repeat,
        address_mode_w: wgpu::AddressMode::Repeat,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        anisotropy_clamp: sampler_anisotropy,
        ..Default::default()
    });

    // G3A-R: PBR factors + the full visual stack configuration. Written once
    // at load time; only `debug_mode` is rewritten later (in place, via
    // `update_debug_mode`) when the presentation debug channel changes.
    let uniform =
        TerrainMaterialUniform::from_terrain_material(material, TerrainDebugMode::default());
    let material_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("G3A-R terrain material uniform"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });

    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("G3A-R terrain material bind group"),
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
        sampler_anisotropy: u32::from(sampler_anisotropy),
        material_uniform: material_uniform_buffer,
        uniform,
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
fn terrain_material_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
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
                        size_of::<TerrainMaterialUniform>() as u64
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

/// G3B: fullscreen postprocess pipeline — `vs_sky_fullscreen` triangle over
/// `fs_postprocess`, no depth test, output to the sRGB surface format.
fn create_postprocess_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G3B HDR postprocess pipeline"),
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
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_postprocess"),
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

/// G3B: persistent linear HDR scene target (Rgba16Float).
///
/// Created at startup and recreated on resize only; sampled by the
/// postprocess pass, never rendered on the frame path creation-wise.
fn create_hdr_target(device: &wgpu::Device, width: u32, height: u32) -> HdrTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3B HDR scene target"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: HDR_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    HdrTarget {
        _texture: texture,
        view,
    }
}

/// G3B: postprocess bind group layout — HDR scene texture (f32 sampleable),
/// nearest sampler, and the postprocess uniform (exposure EV).
fn postprocess_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(16),
                },
                count: None,
            },
        ],
    })
}

/// G3B: postprocess bind group binding the HDR scene view + sampler + uniform.
fn create_hdr_scene_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    hdr_view: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
    uniform_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(hdr_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    })
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
    fn terrain_material_uniform_layout_is_128_bytes() {
        // WGSL: eight vec4 slots (PBR factors + debug, stack scales, five
        // vec2 anchors, ar transform, fade range, padding). Must match the
        // shader struct exactly.
        assert_eq!(
            size_of::<TerrainMaterialUniform>(),
            128,
            "TerrainMaterialUniform must occupy exactly eight WGSL vec4 slots"
        );
        // Rust (repr(C), f32 arrays) and WGSL (vec2/vec4 alignment) must agree
        // on every member offset; a mismatch would corrupt the uniform.
        let uniform = TerrainMaterialUniform::from_terrain_material(
            &TerrainMaterial::default(),
            TerrainDebugMode::default(),
        );
        let base = &uniform as *const TerrainMaterialUniform as usize;
        assert_eq!(&uniform.metallic as *const f32 as usize - base, 0);
        assert_eq!(&uniform.debug_mode as *const u32 as usize - base, 12);
        assert_eq!(&uniform.base_scale_m as *const f32 as usize - base, 16);
        assert_eq!(
            &uniform.albedo_uv_offset as *const [f32; 2] as usize - base,
            32
        );
        assert_eq!(
            &uniform.normal_uv_offset as *const [f32; 2] as usize - base,
            40
        );
        assert_eq!(
            &uniform.roughness_uv_offset as *const [f32; 2] as usize - base,
            48
        );
        assert_eq!(
            &uniform.detail_uv_offset as *const [f32; 2] as usize - base,
            56
        );
        assert_eq!(
            &uniform.macro_uv_offset as *const [f32; 2] as usize - base,
            64
        );
        assert_eq!(
            &uniform.ar_angle_cos_sin as *const [f32; 2] as usize - base,
            72
        );
        assert_eq!(
            &uniform.ar_scale_offset as *const [f32; 4] as usize - base,
            80
        );
        assert_eq!(
            &uniform.detail_fade_near_far as *const [f32; 4] as usize - base,
            96
        );
        assert_eq!(&uniform._padding2 as *const [f32; 4] as usize - base, 112);
    }

    #[test]
    fn terrain_material_uniform_roundtrips_through_bytes() {
        let material = TerrainMaterial::default();
        let uniform =
            TerrainMaterialUniform::from_terrain_material(&material, TerrainDebugMode::default());
        assert_eq!(uniform.metallic, 0.0);
        assert_eq!(uniform.roughness, 0.9);
        assert_eq!(uniform.normal_strength, 1.0);
        assert_eq!(uniform.debug_mode, 0);
        assert_eq!(uniform.base_scale_m, 4.0);
        assert_eq!(uniform.detail_scale_m, 0.40);
        assert_eq!(uniform.macro_scale_m, 48.0);
        assert_eq!(uniform.albedo_uv_offset, [0.0, 0.0]);
        assert_eq!(uniform.normal_uv_offset, [0.271, 0.137]);
        assert_eq!(uniform.roughness_uv_offset, [0.413, 0.303]);
        assert_eq!(uniform.detail_uv_offset, [0.163, 0.037]);
        assert_eq!(uniform.macro_uv_offset, [0.170, 0.390]);
        assert_eq!(uniform.ar_scale_offset[0], 1.370);
        assert_eq!(uniform.ar_scale_offset[1], 0.315);
        assert_eq!(uniform.ar_scale_offset[2], 0.571);
        assert_eq!(
            uniform.ar_angle_cos_sin[0],
            (27.0f32 * PI / 180.0).cos(),
            "ar angle must be stored as cos"
        );
        assert_eq!(
            uniform.ar_angle_cos_sin[1],
            (27.0f32 * PI / 180.0).sin(),
            "ar angle must be stored as sin"
        );
        assert_eq!(uniform.detail_fade_near_far[0], 20.0);
        assert_eq!(uniform.detail_fade_near_far[1], 80.0);

        let decoded: TerrainMaterialUniform = *bytemuck::from_bytes(bytemuck::bytes_of(&uniform));
        assert_eq!(decoded.metallic, 0.0);
        assert_eq!(decoded.roughness, 0.9);
        assert_eq!(decoded.normal_uv_offset, [0.271, 0.137]);
        assert_eq!(decoded.debug_mode, 0);
        assert_eq!(decoded.ar_scale_offset, uniform.ar_scale_offset);
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
        let uniform =
            TerrainMaterialUniform::from_terrain_material(&material, TerrainDebugMode::default());
        assert_eq!(uniform.metallic, 1.0);
        assert_eq!(uniform.roughness, 0.02);
        assert_eq!(uniform.normal_strength, 1.0);
    }

    #[test]
    fn terrain_material_uniform_with_debug_mode_only_changes_selector() {
        let material = TerrainMaterial::default();
        let base =
            TerrainMaterialUniform::from_terrain_material(&material, TerrainDebugMode::default());
        for mode in [
            TerrainDebugMode::Albedo,
            TerrainDebugMode::Normal,
            TerrainDebugMode::Roughness,
            TerrainDebugMode::Macro,
            TerrainDebugMode::Detail,
        ] {
            let updated = base.with_debug_mode(mode);
            assert_eq!(updated.debug_mode, mode.as_u32());
            // Everything else must stay byte-identical apart from the selector.
            let mut expected = base;
            expected.debug_mode = mode.as_u32();
            assert_eq!(
                updated.debug_mode, expected.debug_mode,
                "debug mode selector mismatch"
            );
            assert_eq!(
                updated.ar_scale_offset, base.ar_scale_offset,
                "debug switch must not touch material state"
            );
            assert_eq!(
                updated.detail_fade_near_far, base.detail_fade_near_far,
                "debug switch must not touch material state"
            );
        }
    }
}

#[cfg(test)]
mod terrain_debug_mode_tests {
    use super::*;

    #[test]
    fn terrain_debug_mode_defaults_to_final() {
        assert_eq!(TerrainDebugMode::default(), TerrainDebugMode::Final);
        assert_eq!(TerrainDebugMode::Final.as_u32(), 0);
    }

    #[test]
    fn terrain_debug_mode_u32_mapping_roundtrips() {
        for mode in [
            TerrainDebugMode::Final,
            TerrainDebugMode::Albedo,
            TerrainDebugMode::Normal,
            TerrainDebugMode::Roughness,
            TerrainDebugMode::Macro,
            TerrainDebugMode::Detail,
        ] {
            assert_eq!(TerrainDebugMode::from_u32(mode.as_u32()), Some(mode));
        }
        assert_eq!(TerrainDebugMode::from_u32(6), None);
        assert_eq!(TerrainDebugMode::from_u32(255), None);
    }

    #[test]
    fn terrain_debug_mode_labels_roundtrip() {
        for mode in [
            TerrainDebugMode::Final,
            TerrainDebugMode::Albedo,
            TerrainDebugMode::Normal,
            TerrainDebugMode::Roughness,
            TerrainDebugMode::Macro,
            TerrainDebugMode::Detail,
        ] {
            assert_eq!(
                TerrainDebugMode::from_label(mode.label()),
                Some(mode),
                "label {} must roundtrip",
                mode.label()
            );
        }
        assert_eq!(TerrainDebugMode::from_label("FINAL"), None);
        assert_eq!(TerrainDebugMode::from_label(""), None);
        assert_eq!(TerrainDebugMode::from_label("metal"), None);
    }

    #[test]
    fn effective_sampler_anisotropy_matches_capability() {
        assert_eq!(effective_sampler_anisotropy(true), 16);
        assert_eq!(effective_sampler_anisotropy(false), 1);
    }
}

#[cfg(test)]
mod terrain_gpu_integration_guards {
    //! G3A/G3A-R structural guards: the terrain shader/renderer integration
    //! points that must not drift in later slices.

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
            terrain_block.contains("input.color * vec4<f32>(albedo.rgb, 1.0)"),
            "G2D vertex color must modulate the composite albedo stack"
        );
        assert!(
            terrain_block.contains("terrain_material.roughness * r_stack"),
            "roughness stack must scale the material base roughness"
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
    fn shader_keeps_the_g3a_r_three_frequency_stack() {
        let source = include_str!("shader.wgsl");
        let terrain_block = source
            .split("fn fs_terrain(input: VertexOutput)")
            .nth(1)
            .expect("terrain fragment entry must exist");
        for (needle, label) in [
            (
                "terrain_material.macro_scale_m",
                "macro scale must be uniform-driven",
            ),
            (
                "terrain_material.detail_scale_m",
                "detail scale must be uniform-driven",
            ),
            (
                "smoothstep(fade_near, fade_far, distance)",
                "detail fade must be a continuous smoothstep",
            ),
            (
                "TERRAIN_DETAIL_NORMAL_BLEND",
                "detail normal blend must be a named constant",
            ),
            (
                "terrain_material.debug_mode",
                "debug channel must be uniform-driven",
            ),
            ("mode == 5u", "detail debug channel must exist"),
        ] {
            assert!(
                terrain_block.contains(needle),
                "terrain fragment must use {label}"
            );
        }
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
            ("write_texture(", "texture upload"),
            ("generate_terrain_mip_chain(", "mip chain generation"),
        ] {
            assert!(
                !frame_path.contains(needle),
                "frame path must not recreate/upload the {label} resource"
            );
        }
        assert!(
            initialization_path.contains("let terrain_material_gpu = create_terrain_material("),
            "terrain material must be created once at startup"
        );
    }

    #[test]
    fn terrain_material_uploads_the_full_mip_chain_at_startup_only() {
        let source = include_str!("gpu.rs");
        let (initialization_path, after_init) = source
            .split_once("fn create_terrain_material(")
            .expect("terrain material helper must exist");
        let (creation_path, _) = after_init
            .split_once("// ---------------------------------------------------------------------------\n// Scenery upload helper")
            .or_else(|| after_init.split_once("// Scenery upload helper"))
            .expect("terrain material creation must end before the scenery helper");

        // All mip uploads happen inside the startup helper.
        assert!(
            creation_path.contains("generate_terrain_mip_chain("),
            "mip chain must be generated deterministically at startup"
        );
        assert!(
            creation_path.contains("upload_terrain_mip_level("),
            "every mip level must be uploaded at startup"
        );
        assert!(
            creation_path.contains("mip_level_count: mip_levels"),
            "all three terrain textures must carry the full mip chain"
        );
        assert!(
            creation_path.contains("anisotropy_clamp: sampler_anisotropy"),
            "the terrain sampler must carry the (clamped) anisotropy"
        );
        assert!(
            initialization_path.contains("effective_sampler_anisotropy("),
            "anisotropy must be derived from the device capability"
        );
        // The frame path must not contain any mip upload or mip generation.
        let (_, after_render) = source
            .split_once("pub fn render(&mut self, frame: &RenderFrame)")
            .expect("renderer source must expose the frame path");
        let (frame_path, _) = after_render
            .split_once("fn check_asynchronous_gpu_error")
            .expect("frame path must end before asynchronous error handling");
        for needle in ["upload_terrain_mip_level(", "generate_terrain_mip_chain("] {
            assert!(
                !frame_path.contains(needle),
                "frame path must never touch the mip chain: {needle}"
            );
        }
    }
}

#[cfg(test)]
mod terrain_headless_gpu_tests {
    //! Opt-in GPU tests (run with `cargo test -p renderer --lib -- --ignored`).
    //!
    //! They drive the REAL terrain material upload and the REAL `fs_terrain`
    //! pipeline on a headless wgpu device (no window, no surface), so the
    //! texture content and the lit terrain output can be verified on machines
    //! with a GPU while CPU-only CI stays green. Both tests are `#[ignore]`d;
    //! the upload readback test is also a permanent regression guard for the
    //! G3A-R mip chain upload path.
    use super::*;
    use crate::math::look_at_rh;
    use crate::terrain::{generate_centered_terrain_chunks, generate_flat_terrain};
    use crate::terrain_textures::{generate_terrain_mip_chain, terrain_texture_set_from_decoded};
    use crate::webgpu_perspective;

    fn headless_device() -> (wgpu::Device, wgpu::Queue) {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            apply_limit_buckets: false,
        }))
        .expect("no wgpu adapter available on this machine");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("g3a-r headless test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits {
                max_bind_groups: adapter.limits().max_bind_groups,
                ..wgpu::Limits::default()
            },
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("request_device failed");
        (device, queue)
    }

    fn committed_base_set() -> crate::terrain_textures::TerrainTextureSet {
        let albedo = decode_image(terrain_assets::TERRAIN_ALBEDO_PNG).expect("albedo decode");
        let normal = decode_image(terrain_assets::TERRAIN_NORMAL_PNG).expect("normal decode");
        let roughness =
            decode_image(terrain_assets::TERRAIN_ROUGHNESS_PNG).expect("roughness decode");
        terrain_texture_set_from_decoded(&albedo.rgba8, &normal.rgba8, &roughness.rgba8)
    }

    /// Read one mip level (or a rendered target) back to CPU with WebGPU row
    /// alignment, stripping the per-row padding.
    fn read_texture_level(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        level: u32,
        width: u32,
        height: u32,
        bytes_per_pixel: u32,
    ) -> Vec<u8> {
        let row_bytes = padded_bytes_per_row_checked_for_bytes_per_pixel(width, bytes_per_pixel)
            .expect("row padding must not overflow");
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("g3a-r readback buffer"),
            size: u64::from(row_bytes) * u64::from(height),
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level: level,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row_bytes),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        queue.submit([encoder.finish()]);

        let slice = buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).expect("readback channel");
        });
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("poll failed");
        receiver
            .recv()
            .expect("readback response")
            .expect("map failed");
        let mapped = slice.get_mapped_range().expect("mapped range");
        let unpadded = width as usize * bytes_per_pixel as usize;
        let mut bytes = Vec::with_capacity(unpadded * height as usize);
        for row in 0..height as usize {
            let start = row * row_bytes as usize;
            bytes.extend_from_slice(&mapped[start..start + unpadded]);
        }
        drop(mapped);
        buffer.unmap();
        bytes
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn terrain_upload_readback_matches_committed_assets() {
        let (device, queue) = headless_device();
        let layout = terrain_material_bind_group_layout(&device, "g3a-r readback layout");
        // Real startup path: decode committed PNGs, build the chain, upload.
        let material =
            create_terrain_material(&device, &layout, &queue, &TerrainMaterial::default(), 16)
                .expect("terrain material creation must succeed");

        // CPU expectation from the same committed assets (bit-exact).
        let base = committed_base_set();
        let chain = generate_terrain_mip_chain(&base, TERRAIN_TEXTURE_SIZE);

        // Level 0 and mid/1x1 levels of the sRGB albedo must round-trip.
        for (level, width, height) in [(0u32, 512u32, 512u32), (5, 16, 16), (9, 1, 1)] {
            let gpu = read_texture_level(
                &device,
                &queue,
                &material._albedo_texture,
                level,
                width,
                height,
                4,
            );
            let expected = &chain.albedo[level as usize].bytes;
            assert!(
                gpu.len() >= expected.len(),
                "albedo level {level} readback must span the level bytes"
            );
            assert_eq!(
                &gpu[..expected.len()],
                expected.as_slice(),
                "albedo level {level} must round-trip the committed pixels"
            );
        }

        // Normal levels must also round-trip (linear data).
        for (level, width) in [(0u32, 512u32), (5, 16), (9, 1)] {
            let gpu = read_texture_level(
                &device,
                &queue,
                &material._normal_texture,
                level,
                width,
                width,
                4,
            );
            let expected = &chain.normal[level as usize].bytes;
            assert_eq!(
                &gpu[..expected.len()],
                expected.as_slice(),
                "normal level {level} must round-trip the committed pixels"
            );
        }

        // Roughness (R8) base level.
        let gpu_roughness = read_texture_level(
            &device,
            &queue,
            &material._roughness_texture,
            0,
            512,
            512,
            1,
        );
        assert_eq!(
            &gpu_roughness[..chain.roughness[0].bytes.len()],
            chain.roughness[0].bytes.as_slice(),
            "roughness level 0 must round-trip the committed pixels"
        );
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn fs_terrain_offscreen_renders_lit_green_grass() {
        let pixels = render_offscreen_with_debug_mode(TerrainDebugMode::Final);
        verify_lit_green_grass_bands(&pixels, "final", 15.0, 20.0);
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn fs_terrain_debug_albedo_is_green_dominant() {
        let pixels = render_offscreen_with_debug_mode(TerrainDebugMode::Albedo);
        verify_green_dominant_bands(&pixels, "albedo");
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn fs_terrain_debug_normal_encodes_up_normals() {
        let pixels = render_offscreen_with_debug_mode(TerrainDebugMode::Normal);
        verify_up_normal_bands(&pixels);
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn fs_terrain_debug_roughness_tracks_committed_map() {
        let pixels = render_offscreen_with_debug_mode(TerrainDebugMode::Roughness);
        verify_grey_roughness_bands(&pixels);
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn fs_terrain_debug_macro_matches_g2d_carrier() {
        let pixels = render_offscreen_with_debug_mode(TerrainDebugMode::Macro);
        verify_green_dominant_bands(&pixels, "macro");
    }

    /// Shared headless render used by every `fs_terrain` channel probe:
    /// identical pipeline/geometry/camera, only the debug selector varies.
    /// Returns the dense RGBA8 frame bytes (512x512, sRGB-encoded).
    fn render_offscreen_with_debug_mode(mode: TerrainDebugMode) -> Vec<u8> {
        let (device, queue) = headless_device();
        const SIZE: u32 = 512;
        let format = wgpu::TextureFormat::Rgba8UnormSrgb;

        // --- Bind group layouts / pipeline (identical to the app) ---
        let camera_layout = camera_bind_group_layout(&device, "test camera layout");
        let object_layout = matrix_bind_group_layout(&device, "test object layout");
        let env_layout = environment_bind_group_layout(&device, "test env layout");
        let terrain_layout = terrain_material_bind_group_layout(&device, "test terrain layout");
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("g3a-r headless terrain layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&env_layout),
                None,
                Some(&terrain_layout),
            ],
            immediate_size: 0,
        });
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("g3a-r headless shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let pipeline = create_pipeline(
            &device,
            &shader,
            &pipeline_layout,
            format,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                depth_write_enabled: true,
                label: "g3a-r headless fs_terrain pipeline",
                fragment_entry_point: "fs_terrain",
            },
        );

        // --- Camera: 60 deg vertical FOV looking at a flat 1 km field ---
        let eye = [0.0, 3.0, 10.0];
        let view = look_at_rh(eye, [0.0, 0.0, 0.0], [0.0, 1.0, 0.0]);
        let projection = webgpu_perspective(60.0_f32.to_radians(), 1.0, 0.05, 2_000.0)
            .expect("projection must be valid");
        let view_projection = projection * view;
        let inv_view_projection = view_projection
            .inverse()
            .expect("view-projection invertible");
        let camera_uniform = CameraUniform::new(&view_projection, &inv_view_projection, eye);
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test camera buffer"),
            contents: bytemuck::bytes_of(&camera_uniform),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("test camera bind group"),
            layout: &camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: camera_buffer.as_entire_binding(),
            }],
        });
        let object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test identity object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let object_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("test object bind group"),
            layout: &object_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: object_buffer.as_entire_binding(),
            }],
        });
        let environment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test environment buffer"),
            contents: bytemuck::bytes_of(&EnvironmentUniform::default_environment()),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test shadow matrix buffer"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_target = create_shadow_target(&device);
        let shadow_sampler = create_shadow_comparison_sampler(&device);
        let environment_bind_group = create_environment_bind_group(
            &device,
            &env_layout,
            &environment_buffer,
            &shadow_target.view,
            &shadow_sampler,
            &shadow_matrix_buffer,
            "g3a-r headless env bind group",
        );

        // --- Real terrain material + real chunk geometry ---
        // The selector under test is baked into the uniform at creation so
        // every debug channel renders through the identical production path.
        let material_desc = TerrainMaterial {
            debug_mode: mode,
            ..TerrainMaterial::default()
        };
        let terrain_material =
            create_terrain_material(&device, &terrain_layout, &queue, &material_desc, 16)
                .expect("terrain material creation must succeed");
        let height_field = generate_flat_terrain(200, 200, 5.0, 0.0);
        let chunks =
            generate_centered_terrain_chunks(&height_field, 32, &TerrainMaterial::default());
        let mut gpu_chunks = Vec::with_capacity(chunks.len());
        for chunk in &chunks {
            let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("test terrain vertices"),
                contents: bytemuck::cast_slice(&chunk.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("test terrain indices"),
                contents: bytemuck::cast_slice(&chunk.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
            gpu_chunks.push((vertex_buffer, index_buffer, chunk.indices.len() as u32));
        }

        // --- Offscreen targets ---
        let color_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("g3a-r headless color target"),
            size: wgpu::Extent3d {
                width: SIZE,
                height: SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_target = create_depth_target(&device, SIZE, SIZE);

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("g3a-r headless terrain pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.2,
                            g: 0.2,
                            b: 0.2,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_target.view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&pipeline);
            pass.set_bind_group(0, &camera_bind_group, &[]);
            pass.set_bind_group(1, &object_bind_group, &[]);
            pass.set_bind_group(2, &environment_bind_group, &[]);
            pass.set_bind_group(4, &terrain_material.bind_group, &[]);
            for (vertex_buffer, index_buffer, index_count) in &gpu_chunks {
                pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..*index_count, 0, 0..1);
            }
        }
        queue.submit([encoder.finish()]);

        // --- Read back the frame; per-channel assertions live in helpers ---
        read_texture_level(&device, &queue, &color_texture, 0, SIZE, SIZE, 4)
    }

    /// Lower-half frame mean (flat field, no sky) of dense RGBA8 bytes.
    fn lower_half_mean(pixels: &[u8]) -> [f64; 3] {
        const SIZE: u32 = 512;
        let mut sum = [0.0f64; 3];
        let mut count = 0u64;
        for y in (SIZE / 2)..SIZE {
            for x in 0..SIZE {
                let pixel = &pixels[((y * SIZE + x) * 4) as usize..];
                sum[0] += f64::from(pixel[0]);
                sum[1] += f64::from(pixel[1]);
                sum[2] += f64::from(pixel[2]);
                count += 1;
            }
        }
        [
            sum[0] / count as f64,
            sum[1] / count as f64,
            sum[2] / count as f64,
        ]
    }

    fn log_band_means(pixels: &[u8], label: &str) {
        const SIZE: u32 = 512;
        let mut band_means = Vec::new();
        for band in 0..10 {
            let y0 = (band * SIZE / 10) as usize;
            let y1 = ((band + 1) * SIZE / 10) as usize;
            let mut sum = [0.0f64; 3];
            for y in y0..y1 {
                for x in 0..SIZE as usize {
                    let pixel = &pixels[(y * SIZE as usize + x) * 4..];
                    sum[0] += f64::from(pixel[0]);
                    sum[1] += f64::from(pixel[1]);
                    sum[2] += f64::from(pixel[2]);
                }
            }
            let n = ((y1 - y0) * SIZE as usize) as f64;
            band_means.push(format!(
                "band{band}:[{:.0},{:.0},{:.0}]",
                sum[0] / n,
                sum[1] / n,
                sum[2] / n
            ));
        }
        eprintln!("offscreen {label} band means: {}", band_means.join(" "));
    }

    fn verify_lit_green_grass_bands(
        pixels: &[u8],
        label: &str,
        green_over_red: f64,
        green_over_blue: f64,
    ) {
        log_band_means(pixels, label);
        let mean = lower_half_mean(pixels);
        assert!(
            mean[1] > mean[0] + green_over_red,
            "grass must be green-dominant in G, got mean RGB {mean:?}"
        );
        assert!(
            mean[1] > mean[2] + green_over_blue,
            "grass must be blue-starved, got mean RGB {mean:?}"
        );
        assert!(
            mean[0] > 30.0 && mean[0] < 230.0 && mean[1] > 30.0 && mean[1] < 250.0,
            "lit grass must stay within a sane band, got mean RGB {mean:?}"
        );
    }

    fn verify_green_dominant_bands(pixels: &[u8], label: &str) {
        log_band_means(pixels, label);
        let mean = lower_half_mean(pixels);
        assert!(
            mean[1] > mean[0] + 5.0,
            "{label} channel must be green-dominant, got mean RGB {mean:?}"
        );
        assert!(
            mean[1] > mean[2] + 5.0,
            "{label} channel must be blue-starved, got mean RGB {mean:?}"
        );
    }

    fn verify_up_normal_bands(pixels: &[u8]) {
        log_band_means(pixels, "normal");
        let mean = lower_half_mean(pixels);
        // Perturbed tangent-space normals visualize around the relief-biased
        // mean, not flat 128: blue stays dominant, R/G carry the relief.
        assert!(
            mean[2] > mean[0] + 40.0 && mean[2] > mean[1] + 40.0,
            "normal channel must be blue-dominant (up), got mean RGB {mean:?}"
        );
        assert!(
            mean[0] > 100.0 && mean[0] < 250.0 && mean[1] > 100.0 && mean[1] < 250.0,
            "normal XY must stay in the encoded relief band, got mean RGB {mean:?}"
        );
        // Relief must actually modulate the channel (not a flat fill).
        let (lower_chunks, _) = pixels.as_chunks::<4>();
        let mut min_r = 255u8;
        let mut max_r = 0u8;
        for chunk in lower_chunks.iter().skip(lower_chunks.len() / 2) {
            min_r = min_r.min(chunk[0]);
            max_r = max_r.max(chunk[0]);
        }
        assert!(
            max_r - min_r >= 8,
            "normal channel shows no relief modulation (R range {min_r}..{max_r})"
        );
    }

    fn verify_grey_roughness_bands(pixels: &[u8]) {
        log_band_means(pixels, "roughness");
        let mean = lower_half_mean(pixels);
        let spread = (mean[0] - mean[1]).abs().max((mean[1] - mean[2]).abs());
        assert!(
            spread < 12.0,
            "roughness channel must be grey, got mean RGB {mean:?} (spread {spread:.1})"
        );
        assert!(
            mean[0] > 100.0 && mean[0] < 250.0,
            "roughness level must sit in the committed band, got mean RGB {mean:?}"
        );
    }
}
