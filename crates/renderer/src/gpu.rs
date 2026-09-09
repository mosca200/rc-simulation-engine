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
//! 1. Three stable cascaded shadow depth passes (scenery, instanced vegetation,
//!    aircraft, articulated surfaces; terrain is the persistent receiver)
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
    SHADOW_CASCADE_COUNT, SHADOW_CASCADE_SPLITS_M, SHADOW_DEPTH_BIAS_CONSTANT,
    SHADOW_DEPTH_BIAS_SLOPE_SCALE, SHADOW_MAP_RESOLUTION, ShadowCascade, build_shadow_cascades,
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
use crate::vegetation::{
    DEFAULT_VEGETATION_SEED, GROUP_COUNT, LOD_COUNT, PART_COUNT, VegetationDebugMode,
    VegetationFrameStats, VegetationGpuInstance, VegetationWorld,
};
use crate::vegetation_assets::{VegetationPart, part_metallic, part_roughness};
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
// G3-VR1.1: density tuned down from the VR1 0.0028 trial — at 0.0028 a
// distant aircraft at 300 m sat at ~57 % fog and read washed out. 0.0012
// keeps the aircraft legible (100 m ≈ 11 %, 200 m ≈ 21 %, 300 m ≈ 30 %)
// while still grading the 500 m field edge (~45 %); horizon blending is
// carried primarily by the stronger G3-VR1 haze.
const DEFAULT_HAZE_STRENGTH: f32 = 0.68;
const DEFAULT_FOG_DENSITY: f32 = 0.0012;
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

/// G3E receiver state for all three production shadow cascades.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct ShadowUniform {
    light_view_projection: [[[f32; 4]; 4]; SHADOW_CASCADE_COUNT],
    split_distances_m: [f32; 4],
    receiver_depth_bias: [f32; 4],
    texel_size_uv: [f32; 4],
}

impl ShadowUniform {
    fn from_cascades(cascades: &[ShadowCascade; SHADOW_CASCADE_COUNT]) -> Self {
        Self {
            light_view_projection: std::array::from_fn(|index| {
                matrix_to_wgsl_columns(&cascades[index].light_view_projection)
            }),
            split_distances_m: [
                SHADOW_CASCADE_SPLITS_M[0],
                SHADOW_CASCADE_SPLITS_M[1],
                SHADOW_CASCADE_SPLITS_M[2],
                0.0,
            ],
            receiver_depth_bias: [
                cascades[0].receiver_depth_bias,
                cascades[1].receiver_depth_bias,
                cascades[2].receiver_depth_bias,
                0.0,
            ],
            texel_size_uv: [
                1.0 / SHADOW_MAP_RESOLUTION as f32,
                1.0 / SHADOW_MAP_RESOLUTION as f32,
                0.0,
                0.0,
            ],
        }
    }

    #[cfg(test)]
    fn from_matrix(light_view_projection: &Mat4) -> Self {
        Self {
            light_view_projection: [
                matrix_to_wgsl_columns(light_view_projection),
                matrix_to_wgsl_columns(light_view_projection),
                matrix_to_wgsl_columns(light_view_projection),
            ],
            split_distances_m: [
                SHADOW_CASCADE_SPLITS_M[0],
                SHADOW_CASCADE_SPLITS_M[1],
                SHADOW_CASCADE_SPLITS_M[2],
                0.0,
            ],
            receiver_depth_bias: [0.000_1, 0.000_1, 0.000_1, 0.0],
            texel_size_uv: [
                1.0 / SHADOW_MAP_RESOLUTION as f32,
                1.0 / SHADOW_MAP_RESOLUTION as f32,
                0.0,
                0.0,
            ],
        }
    }
}

/// G3E caster state bound independently for each shadow-array layer.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct ShadowCascadeUniform {
    light_view_projection: [[f32; 4]; 4],
}

impl ShadowCascadeUniform {
    fn from_cascade(cascade: &ShadowCascade) -> Self {
        Self {
            light_view_projection: matrix_to_wgsl_columns(&cascade.light_view_projection),
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

/// G3E: persistent three-layer depth texture sampled by the lit pass and
/// written one layer at a time by the cascade caster passes. It is independent
/// from the resize-dependent scene depth target.
struct ShadowTarget {
    _texture: wgpu::Texture,
    view: wgpu::TextureView,
    cascade_views: [wgpu::TextureView; SHADOW_CASCADE_COUNT],
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

// ── G3D: production vegetation ─────────────────────────────────────────────

/// Presentation-only vegetation state (mirror of the WGSL group-4 uniform).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Pod, Zeroable)]
struct VegetationUniform {
    debug_mode: u32,
    _pad: [u32; 3],
}

impl VegetationUniform {
    fn new(debug_mode: VegetationDebugMode) -> Self {
        Self {
            debug_mode: debug_mode.as_u32(),
            _pad: [0; 3],
        }
    }
}

/// One static (asset, LOD, part) mesh staged for instanced drawing.
///
/// Buffers are created once at startup and never touched again; the
/// instance buffer (slot 1) carries the per-frame transform data.
struct VegetationGpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    /// Index into `WgpuRenderer::materials` (bark or foliage PBR material).
    material_index: usize,
}

/// Persistent GPU vegetation resources (G3D).
///
/// Everything here is created once and reused every frame: static per-asset
/// meshes, the preallocated COPY_DST instance buffer, the presentation-only
/// debug uniform, and the two instanced pipelines (lit HDR scene + depth-only
/// shadow). Per frame the renderer only rewrites the instance buffer contents
/// via `queue.write_buffer` and records index draws per active batch group.
struct GpuVegetation {
    /// `(asset, LOD, part)` flattened:
    /// index = (asset * LOD_COUNT + lod) * PART_COUNT + part.
    meshes: Vec<VegetationGpuMesh>,
    /// Persistent preallocated instance buffer, `capacity * 48` bytes.
    instance_buffer: wgpu::Buffer,
    instance_capacity: usize,
    /// Presentation-only debug state uniform (16 bytes, rewritten on change).
    _uniform_buffer: wgpu::Buffer,
    uniform_bind_group: wgpu::BindGroup,
    uniform: VegetationUniform,
    /// Lit HDR instanced scene pipeline for bark (`vs_vegetation` / `fs_vegetation`).
    pipeline: wgpu::RenderPipeline,
    /// Lit HDR instanced scene pipeline for foliage: identical to `pipeline`
    /// but with backface culling disabled so leaf cards are visible from both
    /// sides (PV1-R alpha-masked foliage).
    foliage_pipeline: wgpu::RenderPipeline,
    /// Depth-only instanced shadow caster (`vs_vegetation_shadow` / `fs_vegetation_shadow`).
    shadow_pipeline: wgpu::RenderPipeline,
    /// PV1-R2: two-sided shadow caster for foliage alpha cards.
    foliage_shadow_pipeline: wgpu::RenderPipeline,
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
    shadow_uniform_buffer: wgpu::Buffer,
    environment_bind_group: wgpu::BindGroup,
    shadow_cascade_uniform_buffers: [wgpu::Buffer; SHADOW_CASCADE_COUNT],
    shadow_pass_bind_groups: [wgpu::BindGroup; SHADOW_CASCADE_COUNT],
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
    // G3D: production vegetation (FlyingField preset only).
    vegetation_world: Option<VegetationWorld>,
    vegetation: Option<GpuVegetation>,
    vegetation_debug_mode: VegetationDebugMode,
    /// Frame counter for the periodic Culling-mode counter log.
    vegetation_frame_counter: u64,
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

        let sky_pipeline = create_sky_pipeline(&device, &shader, &sky_pipeline_layout, HDR_FORMAT);
        let triangle_pipeline = create_pipeline(
            &device,
            &shader,
            &lit_pipeline_layout,
            HDR_FORMAT,
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
            HDR_FORMAT,
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
            HDR_FORMAT,
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

        // G3D: production vegetation — owned by the FlyingField preset only.
        // The world owns the deterministic instance list and the per-frame
        // visibility/LOD selection; the GPU side builds every buffer, material,
        // uniform and pipeline ONCE here. The per-frame loop only rewrites the
        // instance buffer contents and records instanced draws. Nothing in this
        // block runs per frame.
        let (vegetation_world, vegetation) = if scenery_preset == Some(SceneryPreset::FlyingField) {
            let world = VegetationWorld::flying_field(
                DEFAULT_VEGETATION_SEED,
                -ground_below_render_origin_m,
            );
            // Dedicated bark/foliage PBR materials (dielectric, rough bark,
            // slightly glossier foliage — distinct response per part).
            let bark_material = create_white_texture_material(
                &device,
                &material_bind_group_layout,
                &queue,
                part_metallic(VegetationPart::Bark),
                part_roughness(VegetationPart::Bark),
            );
            let bark_material_index = materials.len();
            materials.push(bark_material);
            let foliage_material = create_white_texture_material(
                &device,
                &material_bind_group_layout,
                &queue,
                part_metallic(VegetationPart::Foliage),
                part_roughness(VegetationPart::Foliage),
            );
            let foliage_material_index = materials.len();
            materials.push(foliage_material);

            // Group 4 of the vegetation scene pipeline carries the
            // presentation-only debug uniform (camera/object/environment/
            // material keep the shared lit slots 0-3).
            let vegetation_state_bind_group_layout =
                vegetation_state_bind_group_layout(&device, "G3D vegetation state layout");
            let vegetation_pipeline_layout =
                device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("G3D vegetation pipeline layout"),
                    bind_group_layouts: &[
                        Some(&camera_bind_group_layout),
                        Some(&object_bind_group_layout),
                        Some(&environment_bind_group_layout),
                        Some(&material_bind_group_layout),
                        Some(&vegetation_state_bind_group_layout),
                    ],
                    immediate_size: 0,
                });
            // Depth-only instanced caster layout: camera + identity object +
            // shadow matrix + material (PV1-R2: material group added for
            // alpha-masked foliage shadow discard).
            let vegetation_shadow_pipeline_layout =
                device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("G3D vegetation shadow pipeline layout"),
                    bind_group_layouts: &[
                        Some(&camera_bind_group_layout),
                        Some(&object_bind_group_layout),
                        Some(&shadow_pass_bind_group_layout),
                        Some(&material_bind_group_layout),
                    ],
                    immediate_size: 0,
                });

            let gpu = build_gpu_vegetation(
                &device,
                &queue,
                &shader,
                &world,
                &vegetation_state_bind_group_layout,
                &vegetation_pipeline_layout,
                &vegetation_shadow_pipeline_layout,
                &material_bind_group_layout,
                &mut materials,
                bark_material_index,
                foliage_material_index,
                VegetationDebugMode::default(),
            );
            (Some(world), Some(gpu))
        } else {
            (None, None)
        };

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
        // G3E: derive the shadow camera direction from the exact normalized
        // direction uploaded into EnvironmentUniform, so the sun disk,
        // direct PBR lighting, and all cascade layers can never diverge.
        let shadow_light_direction = [
            default_environment.light_direction[0],
            default_environment.light_direction[1],
            default_environment.light_direction[2],
        ];
        let initial_shadow_cascades =
            build_shadow_cascades(shadow_light_direction, [0.0, 2.0, 8.0], [0.0, 0.0, 0.0]);
        let environment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("environment uniform"),
            contents: bytemuck::bytes_of(&default_environment),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("G3E shadow receiver uniform"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_cascades(&initial_shadow_cascades)),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let shadow_cascade_uniform_buffers: [wgpu::Buffer; SHADOW_CASCADE_COUNT] =
            std::array::from_fn(|index| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(
                        [
                            "G3E near cascade caster uniform",
                            "G3E mid cascade caster uniform",
                            "G3E far cascade caster uniform",
                        ][index],
                    ),
                    contents: bytemuck::bytes_of(&ShadowCascadeUniform::from_cascade(
                        &initial_shadow_cascades[index],
                    )),
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                })
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
            &shadow_uniform_buffer,
            "environment bind group",
        );
        let shadow_pass_bind_groups: [wgpu::BindGroup; SHADOW_CASCADE_COUNT] =
            std::array::from_fn(|index| {
                create_shadow_pass_bind_group(
                    &device,
                    &shadow_pass_bind_group_layout,
                    &shadow_cascade_uniform_buffers[index],
                    [
                        "G3E near cascade pass bind group",
                        "G3E mid cascade pass bind group",
                        "G3E far cascade pass bind group",
                    ][index],
                )
            });

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
                bind_group_layouts: &[
                    None,
                    None,
                    None,
                    None,
                    None,
                    Some(&postprocess_bind_group_layout),
                ],
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
            shadow_uniform_buffer,
            environment_bind_group,
            shadow_cascade_uniform_buffers,
            shadow_pass_bind_groups,
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
            vegetation_world,
            vegetation,
            vegetation_debug_mode: VegetationDebugMode::default(),
            vegetation_frame_counter: 0,
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

    /// G3D: switch the presentation-only vegetation debug channel.
    ///
    /// Only rewrites the 16-byte persistent state uniform on change; no
    /// shader recompile, no resource creation, nothing per frame. The default
    /// is [`VegetationDebugMode::Final`], the production path. No-op when
    /// vegetation is not configured (scenery preset != FlyingField).
    pub fn set_vegetation_debug_mode(&mut self, mode: VegetationDebugMode) {
        self.vegetation_debug_mode = mode;
        if let Some(vegetation) = self.vegetation.as_mut()
            && vegetation.uniform.debug_mode != mode.as_u32()
        {
            vegetation.uniform.debug_mode = mode.as_u32();
            self.queue.write_buffer(
                &vegetation._uniform_buffer,
                0,
                bytemuck::bytes_of(&vegetation.uniform),
            );
        }
    }

    /// Current vegetation debug channel.
    #[must_use]
    pub fn vegetation_debug_mode(&self) -> VegetationDebugMode {
        self.vegetation_debug_mode
    }

    /// G3D: per-frame vegetation visibility counters (presentation-only).
    ///
    /// `None` when the renderer was created without the FlyingField preset.
    #[must_use]
    pub fn vegetation_stats(&self) -> Option<&VegetationFrameStats> {
        self.vegetation_world.as_ref().map(VegetationWorld::stats)
    }

    /// G3D: the instance-buffer capacity (preallocated worst case).
    #[must_use]
    pub fn vegetation_instance_capacity(&self) -> Option<usize> {
        self.vegetation.as_ref().map(|v| v.instance_capacity)
    }

    /// G3D: instance bytes uploaded in the last frame (visible × 48).
    #[must_use]
    pub fn vegetation_last_instance_bytes(&self) -> Option<u64> {
        let visible_count = self
            .vegetation_world
            .as_ref()
            .map(|world| world.visible().len() as u64)?;
        Some(visible_count * size_of::<VegetationGpuInstance>() as u64)
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
        let (eye, camera_target) = self.camera.eye_and_target(aircraft_pose);
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

        // G3E: fixed-extent light cameras follow the active camera view only on
        // their independent light-space texel grids. All buffers, bind groups,
        // texture layers and pipelines are persistent; the frame path performs
        // four bounded uniform writes and command encoding only.
        let shadow_cascades =
            build_shadow_cascades(self.shadow_light_direction, eye, camera_target);
        let shadow_uniform = ShadowUniform::from_cascades(&shadow_cascades);
        self.queue.write_buffer(
            &self.shadow_uniform_buffer,
            0,
            bytemuck::bytes_of(&shadow_uniform),
        );
        for (index, cascade) in shadow_cascades.iter().enumerate() {
            let cascade_uniform = ShadowCascadeUniform::from_cascade(cascade);
            self.queue.write_buffer(
                &self.shadow_cascade_uniform_buffers[index],
                0,
                bytemuck::bytes_of(&cascade_uniform),
            );
        }

        // G3D: per-frame CPU visibility/LOD selection and the instance-buffer
        // rewrite. Everything is persistent and preallocated: `update_visibility`
        // reuses its scratch (clear + swap), and the write targets the same
        // COPY_DST instance buffer every frame. No buffer, bind group, pipeline
        // or shader is created here; the visible list is never cloned.
        if let (Some(world), Some(vegetation)) =
            (self.vegetation_world.as_mut(), self.vegetation.as_ref())
        {
            let visibility_start = std::time::Instant::now();
            world.update_visibility(eye, &vp);
            let visibility_elapsed = visibility_start.elapsed();
            let visible = world.visible();
            if !visible.is_empty() {
                self.queue.write_buffer(
                    &vegetation.instance_buffer,
                    0,
                    bytemuck::cast_slice(visible),
                );
            }
            debug_assert!(
                visible.len() <= vegetation.instance_capacity,
                "instance buffer capacity exceeded"
            );

            // G3D: presentation-only counter log in Culling mode (~1.5 s cadence).
            self.vegetation_frame_counter += 1;
            if self.vegetation_debug_mode == VegetationDebugMode::Culling
                && self.vegetation_frame_counter.is_multiple_of(90)
            {
                let stats = world.stats();
                tracing::info!(
                    vegetation_total = stats.total,
                    vegetation_visible = stats.visible,
                    vegetation_culled_frustum = stats.culled_frustum,
                    vegetation_culled_distance = stats.culled_distance,
                    vegetation_lod0 = stats.lod_counts[0],
                    vegetation_lod1 = stats.lod_counts[1],
                    vegetation_lod2 = stats.lod_counts[2],
                    vegetation_scene_draw_calls = stats.scene_draw_calls,
                    vegetation_shadow_draw_calls =
                        stats.shadow_draw_calls * SHADOW_CASCADE_COUNT as u32,
                    vegetation_instance_bytes_uploaded =
                        visible.len() as u64 * size_of::<VegetationGpuInstance>() as u64,
                    vegetation_cpu_visibility_ms = visibility_elapsed.as_secs_f64() * 1e3,
                    "G3D vegetation visibility/culling counters"
                );
            }
        }

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
        for cascade_index in 0..SHADOW_CASCADE_COUNT {
            // G3E: one depth-only caster pass per persistent array layer.
            // The terrain receives every object shadow but deliberately does
            // not cast into itself. The RC field height mesh spans the entire
            // cascade and its coarse long-range relief otherwise produces a
            // map-sized false occluder over the runway. Scenery, batched
            // vegetation, rigid aircraft geometry, and articulated surfaces
            // retain their established caster paths.
            let depth_attachment = wgpu::RenderPassDepthStencilAttachment {
                view: &self.shadow_target.cascade_views[cascade_index],
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            };
            let mut shadow_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some(
                    [
                        "G3E near cascade shadow depth pass",
                        "G3E mid cascade shadow depth pass",
                        "G3E far cascade shadow depth pass",
                    ][cascade_index],
                ),
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
            shadow_pass.set_bind_group(2, &self.shadow_pass_bind_groups[cascade_index], &[]);

            shadow_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            if let Some(ref scenery) = self.scenery {
                shadow_pass.set_vertex_buffer(0, scenery.vertex_buffer.slice(..));
                shadow_pass
                    .set_index_buffer(scenery.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                shadow_pass.draw_indexed(0..scenery.index_count, 0, 0..1);
            }

            // G3D: instanced vegetation shadows — LOD0/LOD1 cast, LOD2 skips
            // the caster (economical; the far tier is beyond the field's
            // operational shadows anyway). Draw calls depend on active
            // (asset, LOD) batch groups × parts, never on the tree count.
            if let (Some(vegetation), Some(world)) =
                (self.vegetation.as_ref(), self.vegetation_world.as_ref())
                && !world.visible().is_empty()
            {
                shadow_pass.set_pipeline(&vegetation.shadow_pipeline);
                shadow_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
                let ranges = world.batch_ranges();
                let mut current_is_foliage = false;
                for group in 0..GROUP_COUNT {
                    if group % LOD_COUNT > 1 {
                        continue;
                    }
                    let start = ranges[group * 2];
                    let count = ranges[group * 2 + 1];
                    if count == 0 {
                        continue;
                    }
                    let asset = group / LOD_COUNT;
                    let lod = (group % LOD_COUNT) as u8;
                    for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                        // PV1-R2: switch shadow pipeline for foliage (two-sided
                        // + alpha discard) vs bark (backface culled).
                        let is_foliage = matches!(part, VegetationPart::Foliage);
                        if is_foliage != current_is_foliage {
                            if is_foliage {
                                shadow_pass.set_pipeline(&vegetation.foliage_shadow_pipeline);
                            } else {
                                shadow_pass.set_pipeline(&vegetation.shadow_pipeline);
                            }
                            current_is_foliage = is_foliage;
                        }
                        let mesh = &vegetation.meshes[vegetation_mesh_index(asset, lod, part)];
                        let material = &self.materials[mesh.material_index];
                        shadow_pass.set_bind_group(3, &material.bind_group, &[]);
                        shadow_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                        shadow_pass.set_vertex_buffer(1, vegetation.instance_buffer.slice(..));
                        shadow_pass.set_index_buffer(
                            mesh.index_buffer.slice(..),
                            wgpu::IndexFormat::Uint32,
                        );
                        shadow_pass.draw_indexed(0..mesh.index_count, 0, start..start + count);
                    }
                }
            }

            // G3D FIX (23ae238): restore the standard shadow pipeline after instanced
            // vegetation shadow draws. The vegetation shadow pipeline uses a
            // different vertex layout (slot 1 = per-instance transform) and
            // the `vs_vegetation_shadow` entry point; aircraft and surface
            // casters must use the non-instanced `vs_shadow` with their own
            // object transform.
            shadow_pass.set_pipeline(&self.shadow_pipeline);

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

            // G2A: Scenery (flying field, markers). Drawn with the
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

            // G3D: instanced production vegetation (scene pass). Each active
            // (asset, LOD) group draws bark + foliage with their dedicated PBR
            // materials; the instance range comes from `batch_ranges`, so the
            // number of draw calls depends on the batches, never on the tree
            // count. Same linear HDR target, sun, sky, fog and shadow response
            // as every other lit surface.
            if let (Some(vegetation), Some(world)) =
                (self.vegetation.as_ref(), self.vegetation_world.as_ref())
            {
                let visible = world.visible();
                if !visible.is_empty() {
                    render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
                    render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
                    render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
                    render_pass.set_bind_group(4, &vegetation.uniform_bind_group, &[]);
                    let ranges = world.batch_ranges();
                    let mut current_is_foliage = false;
                    render_pass.set_pipeline(&vegetation.pipeline);
                    for group in 0..GROUP_COUNT {
                        let start = ranges[group * 2];
                        let count = ranges[group * 2 + 1];
                        if count == 0 {
                            continue;
                        }
                        let asset = group / LOD_COUNT;
                        let lod = (group % LOD_COUNT) as u8;
                        for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                            // PV1-R: switch to the two-sided foliage pipeline
                            // for leaf cards; bark keeps backface culling.
                            let is_foliage = matches!(part, VegetationPart::Foliage);
                            if is_foliage != current_is_foliage {
                                if is_foliage {
                                    render_pass.set_pipeline(&vegetation.foliage_pipeline);
                                } else {
                                    render_pass.set_pipeline(&vegetation.pipeline);
                                }
                                current_is_foliage = is_foliage;
                            }
                            let mesh = &vegetation.meshes[vegetation_mesh_index(asset, lod, part)];
                            let material = &self.materials[mesh.material_index];
                            render_pass.set_bind_group(3, &material.bind_group, &[]);
                            render_pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                            render_pass.set_vertex_buffer(1, vegetation.instance_buffer.slice(..));
                            render_pass.set_index_buffer(
                                mesh.index_buffer.slice(..),
                                wgpu::IndexFormat::Uint32,
                            );
                            render_pass.draw_indexed(0..mesh.index_count, 0, start..start + count);
                        }
                    }
                }
            }

            // G3D FIX: restore the standard lit pipeline after instanced
            // vegetation draws. The vegetation pipeline uses a different vertex
            // layout (slot 1 = per-instance transform) and different entry
            // points; aircraft and surface batches must never inherit it.
            render_pass.set_pipeline(&self.triangle_pipeline);

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
            postprocess_pass.set_bind_group(5, &self.postprocess_bind_group, &[]);
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
    create_white_texture_material(
        device,
        layout,
        queue,
        PROCEDURAL_METALLIC,
        PROCEDURAL_ROUGHNESS,
    )
}

/// G1D/G3D: white-texture material with explicit PBR factors.
///
/// Used by the shared fallback (procedural aircraft / scenery) and by the G3D
/// bark/foliage vegetation materials, whose meshes carry linear vertex colors
/// and only need a neutral texel to modulate.
fn create_white_texture_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
    metallic: f32,
    roughness: f32,
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
    // accidentally chromatic under the PBR response. G3D passes the same
    // factors through for the bark/foliage vegetation materials.
    let material_uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("white material uniform"),
        contents: bytemuck::bytes_of(&MaterialUniform::new(metallic, roughness)),
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
/// deterministic mip chain (1024 -> 1, 11 levels: sRGB-correct albedo,
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
// G3D: vegetation upload helper
// ---------------------------------------------------------------------------

/// Flattened mesh index for the (asset, LOD, part) triple.
///
/// Group = asset * LOD_COUNT + lod (matches `VegetationWorld::batch_ranges`),
/// then the two parts sit side by side:
/// index = group * PART_COUNT + part.
fn vegetation_mesh_index(asset: usize, lod: u8, part: VegetationPart) -> usize {
    (asset * LOD_COUNT + lod as usize) * PART_COUNT + part.index()
}

/// Build all persistent vegetation GPU resources for the FlyingField preset
/// (G3D). Creates the static per-(asset, LOD, part) mesh buffers, the
/// preallocated COPY_DST instance buffer, the presentation-only debug uniform
/// and the two instanced pipelines. Nothing here is recreated per frame.
///
/// `bark_material_index` / `foliage_material_index` reference the shared
/// `WgpuRenderer::materials` vector (both white-texture PBR materials with
/// the `part_metallic`/`part_roughness` factors from the asset spec).
///
/// PV1-R: when a production asset carries embedded base-color textures
/// (extracted from the GLB by `decode_committed_lod`), per-asset PBR
/// materials are created and pushed onto `materials`. All LODs of the same
/// asset share the LOD0 texture. Assets without textures fall back to the
/// shared `bark_material_index` / `foliage_material_index`.
#[allow(clippy::too_many_arguments)]
fn build_gpu_vegetation(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    shader: &wgpu::ShaderModule,
    world: &VegetationWorld,
    uniform_bind_group_layout: &wgpu::BindGroupLayout,
    pipeline_layout: &wgpu::PipelineLayout,
    shadow_pipeline_layout: &wgpu::PipelineLayout,
    material_bind_group_layout: &wgpu::BindGroupLayout,
    materials: &mut Vec<GpuMaterial>,
    bark_material_index: usize,
    foliage_material_index: usize,
    debug_mode: VegetationDebugMode,
) -> GpuVegetation {
    // Static meshes: one buffer pair per (asset, LOD, part). The Vec index is
    // positional: (asset * LOD_COUNT + lod) * PART_COUNT + part, matching
    // `vegetation_mesh_index`.
    let mut meshes = Vec::with_capacity(world.assets().len() * LOD_COUNT * PART_COUNT);
    for asset in world.assets().assets() {
        // PV1-R: resolve per-asset material indices. If LOD0 carries embedded
        // base-color textures, create dedicated PBR materials; otherwise fall
        // back to the shared vertex-colour materials.
        let lod0 = asset.lods.lod(0).expect("LOD0 always present");
        let (asset_bark_mat, asset_foliage_mat) = if let (Some(bark_tex), Some(foliage_tex)) = (
            lod0.bark_base_color.as_ref(),
            lod0.foliage_base_color.as_ref(),
        ) {
            let bark_mat = create_gpu_material(
                device,
                material_bind_group_layout,
                queue,
                bark_tex,
                &SamplerConfig::default_sampler(),
                crate::vegetation_assets::part_metallic(VegetationPart::Bark),
                crate::vegetation_assets::part_roughness(VegetationPart::Bark),
            )
            .expect("vegetation bark texture uploads");
            let foliage_mat = create_gpu_material(
                device,
                material_bind_group_layout,
                queue,
                foliage_tex,
                &SamplerConfig::default_sampler(),
                crate::vegetation_assets::part_metallic(VegetationPart::Foliage),
                crate::vegetation_assets::part_roughness(VegetationPart::Foliage),
            )
            .expect("vegetation foliage texture uploads");
            let bi = materials.len();
            materials.push(bark_mat);
            let fi = materials.len();
            materials.push(foliage_mat);
            (bi, fi)
        } else {
            (bark_material_index, foliage_material_index)
        };

        for class in 0..LOD_COUNT {
            let lod = asset
                .lods
                .lod(class as u8)
                .expect("LOD class 0..2 always present in the production set");
            for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                let mesh = match part {
                    VegetationPart::Bark => &lod.bark,
                    VegetationPart::Foliage => &lod.foliage,
                };
                let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("vegetation part vertices"),
                    contents: bytemuck::cast_slice(mesh.vertices()),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("vegetation part indices"),
                    contents: bytemuck::cast_slice(mesh.indices()),
                    usage: wgpu::BufferUsages::INDEX,
                });
                meshes.push(VegetationGpuMesh {
                    vertex_buffer,
                    index_buffer,
                    index_count: mesh.indices().len() as u32,
                    material_index: match part {
                        VegetationPart::Bark => asset_bark_mat,
                        VegetationPart::Foliage => asset_foliage_mat,
                    },
                });
            }
        }
    }

    // Persistent preallocated instance buffer (COPY_DST; rewritten per frame).
    let instance_capacity = world.instance_capacity();
    let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("G3D vegetation instance buffer"),
        size: (instance_capacity * size_of::<VegetationGpuInstance>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });

    // Presentation-only debug uniform (16 bytes, rewritten only on change).
    let uniform = VegetationUniform::new(debug_mode);
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("G3D vegetation state uniform"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });
    let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("G3D vegetation state bind group"),
        layout: uniform_bind_group_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform_buffer.as_entire_binding(),
        }],
    });

    let pipeline = create_vegetation_pipeline(device, shader, pipeline_layout);
    let foliage_pipeline = create_vegetation_foliage_pipeline(device, shader, pipeline_layout);
    let shadow_pipeline = create_vegetation_shadow_pipeline(device, shader, shadow_pipeline_layout);
    let foliage_shadow_pipeline =
        create_vegetation_foliage_shadow_pipeline(device, shader, shadow_pipeline_layout);

    // Startup upload of the initial visible set so the first frame is not
    // empty even before the first `update_visibility` call runs.
    let initial = world.visible();
    if !initial.is_empty() {
        queue.write_buffer(&instance_buffer, 0, bytemuck::cast_slice(initial));
    }

    GpuVegetation {
        meshes,
        instance_buffer,
        instance_capacity,
        _uniform_buffer: uniform_buffer,
        uniform_bind_group,
        uniform,
        pipeline,
        foliage_pipeline,
        shadow_pipeline,
        foliage_shadow_pipeline,
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

/// G3D: bind group layout for the 16-byte presentation-only vegetation state
/// uniform (group 4 of the vegetation scene pipeline).
///
/// Deliberately separate from `matrix_bind_group_layout`: that layout declares
/// a 64-byte minimum and VERTEX-only visibility, while the vegetation selector
/// is read by `fs_vegetation` (FRAGMENT) and is exactly 16 bytes. Reusing the
/// matrix layout would fail wgpu validation at bind-group creation.
fn vegetation_state_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<VegetationUniform>() as u64),
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
            // G3E: comparison-sampled depth array and receiver state. They extend
            // the established environment boundary while the lit pipeline
            // remains at groups 0..3.
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2Array,
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

/// G3E caster-pass group 2. It exposes one cascade matrix at binding 4, so the
/// sampled receiver array and its full three-matrix uniform are never bound
/// while a layer is attached for depth writes.
fn shadow_pass_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 4,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<ShadowCascadeUniform>() as u64),
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
    shadow_uniform_buffer: &wgpu::Buffer,
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
                resource: shadow_uniform_buffer.as_entire_binding(),
            },
        ],
    })
}

fn create_shadow_pass_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    shadow_cascade_uniform_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 4,
            resource: shadow_cascade_uniform_buffer.as_entire_binding(),
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

/// G3E: depth-only caster pipeline shared by all three directional cascades.
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
        label: Some("G3E cascaded directional shadow depth pipeline"),
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

/// The instanced vertex layout shared by the G3D vegetation pipelines: the
/// static per-vertex mesh on slot 0 and the per-instance transform on slot 1.
///
/// The instance stride is `size_of::<VegetationGpuInstance>()` (48 bytes);
/// the shader reconstructs `translate * rotY(yaw) * scale` from the three
/// vec4s — no 4x4 matrix per instance.
static VEGETATION_MESH_ATTRIBUTES: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
    0 => Float32x3,
    1 => Float32x3,
    2 => Float32x4,
    3 => Float32x2,
];
static VEGETATION_INSTANCE_ATTRIBUTES: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![
    4 => Float32x4,
    5 => Float32x4,
    6 => Float32x4,
];

fn vegetation_vertex_buffers() -> [Option<wgpu::VertexBufferLayout<'static>>; 2] {
    [
        Some(wgpu::VertexBufferLayout {
            array_stride: size_of::<Vertex>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &VEGETATION_MESH_ATTRIBUTES,
        }),
        Some(wgpu::VertexBufferLayout {
            array_stride: size_of::<VegetationGpuInstance>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &VEGETATION_INSTANCE_ATTRIBUTES,
        }),
    ]
}

/// G3D: lit HDR instanced vegetation pipeline (`vs_vegetation`/`fs_vegetation`).
///
/// Writes scene-referred linear color to the same Rgba16Float target as every
/// other lit surface; exposure + tone mapping stay exclusively in the G3B
/// postprocess pass (no independent tone mapping in this pipeline).
fn create_vegetation_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G3D vegetation lit pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_vegetation"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &vegetation_vertex_buffers(),
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
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_vegetation"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: HDR_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// PV1-R: foliage variant of the vegetation lit pipeline with backface
/// culling disabled. Leaf cards are flat quads that must be visible from
/// both sides; the alpha cutoff in `fs_vegetation` discards transparent
/// fragments so only the leaf silhouette survives.
fn create_vegetation_foliage_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G3D vegetation foliage lit pipeline (two-sided)"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_vegetation"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &vegetation_vertex_buffers(),
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
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_vegetation"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: HDR_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// G3D: depth-only instanced vegetation caster for the fixed directional
/// shadow map (`vs_vegetation_shadow` / `fs_vegetation_shadow`). PV1-R2:
/// includes a fragment stage for alpha-masked discard so foliage cards cast
/// shaped shadows instead of solid quads.
fn create_vegetation_shadow_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G3D vegetation shadow depth pipeline"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_vegetation_shadow"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &vegetation_vertex_buffers(),
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
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_vegetation_shadow"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// PV1-R2: two-sided shadow caster for foliage alpha cards. Same as the bark
/// shadow pipeline but with culling disabled so leaf cards cast from both sides.
fn create_vegetation_foliage_shadow_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G3D vegetation foliage shadow pipeline (two-sided)"),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_vegetation_shadow"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &vegetation_vertex_buffers(),
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
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_vegetation_shadow"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[],
        }),
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
                    min_binding_size: wgpu::BufferSize::new(size_of::<PostProcessUniform>() as u64),
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
        label: Some("G3E three-cascade shadow depth array"),
        size: wgpu::Extent3d {
            width: SHADOW_MAP_RESOLUTION,
            height: SHADOW_MAP_RESOLUTION,
            depth_or_array_layers: SHADOW_CASCADE_COUNT as u32,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor {
        label: Some("G3E shadow receiver array view"),
        dimension: Some(wgpu::TextureViewDimension::D2Array),
        base_array_layer: 0,
        array_layer_count: Some(SHADOW_CASCADE_COUNT as u32),
        ..Default::default()
    });
    let cascade_views = std::array::from_fn(|index| {
        texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some(
                [
                    "G3E near cascade attachment",
                    "G3E mid cascade attachment",
                    "G3E far cascade attachment",
                ][index],
            ),
            dimension: Some(wgpu::TextureViewDimension::D2),
            base_array_layer: index as u32,
            array_layer_count: Some(1),
            ..Default::default()
        })
    });
    ShadowTarget {
        _texture: texture,
        view,
        cascade_views,
    }
}

/// Linear comparison filtering combines with the shader's fixed 3x3 tap grid
/// for a compact, bounded PCF footprint.
fn create_shadow_comparison_sampler(device: &wgpu::Device) -> wgpu::Sampler {
    device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("G3E cascaded shadow comparison sampler"),
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
    //! G3E structural guards. The light-space math itself lives in `shadow.rs`
    //! so it can be tested without wgpu; these tests pin the renderer/shader
    //! integration points that must not drift in later slices.

    #[test]
    fn shadow_uniforms_match_three_cascade_wgsl_layout() {
        assert_eq!(
            std::mem::size_of::<super::ShadowUniform>(),
            240,
            "ShadowUniform must be three mat4 values plus three aligned vec4 slots"
        );
        assert_eq!(
            std::mem::size_of::<super::ShadowCascadeUniform>(),
            64,
            "each caster uniform must contain exactly one mat4"
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
            .split_once("\"G3E near cascade shadow depth pass\",")
            .expect("frame path must contain the three-cascade shadow pass loop");
        let (shadow_pass_path, _) = after_shadow_pass_label
            .split_once("// --- Sky pass")
            .expect("shadow pass must end before the main scene sky pass");

        assert!(
            initialization_path
                .contains("let shadow_pass_bind_groups: [wgpu::BindGroup; SHADOW_CASCADE_COUNT]"),
            "all three matrix-only shadow-pass bind groups must be persistent"
        );
        assert!(
            initialization_path.contains("let shadow_target = create_shadow_target(&device);"),
            "the shadow depth target must remain persistent"
        );
        assert!(
            shadow_pass_path.contains("&self.shadow_pass_bind_groups[cascade_index]"),
            "each caster pass must bind its matrix-only group at group 2"
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
            frame_path.contains("&self.shadow_uniform_buffer")
                && frame_path.contains("&self.shadow_cascade_uniform_buffers[index]"),
            "frame path should only update the persistent receiver and caster buffers"
        );
        assert!(
            frame_path.contains("for cascade_index in 0..SHADOW_CASCADE_COUNT"),
            "the caster path must encode exactly the centralized cascade count"
        );
        assert!(
            !shadow_pass_path.contains("for chunk in &self.terrain_chunks"),
            "the field terrain must receive object shadows without self-casting its coarse far relief"
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
            source.contains("textureSampleCompare("),
            "directional shadows must use the comparison sampler path"
        );
        assert!(source.contains("texture_depth_2d_array"));
        // G3-VR1: the 5x5 percentage-closer kernel is centralized through
        // named WGSL constants; the comparison divisor must use the constant
        // so the tap count stays in one place.
        assert!(source.contains("return visibility / SHADOW_PCF_TAP_COUNT;"));
        assert!(source.contains("const SHADOW_PCF_TAPS: i32 = 2;"));
        assert!(source.contains("const SHADOW_PCF_TAP_COUNT: f32 = 25.0;"));
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
                "TERRAIN_MACRO_AR_BLEND",
                "macro anti-repetition blend must be a named constant",
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
    fn terrain_material_keeps_the_production_color_space_contract() {
        let source = include_str!("gpu.rs");
        let material_creation = source
            .split("fn create_terrain_material(")
            .nth(1)
            .expect("terrain material creation helper must exist");
        for (needle, label) in [
            (
                "format: wgpu::TextureFormat::Rgba8UnormSrgb",
                "albedo must remain hardware sRGB",
            ),
            (
                "format: wgpu::TextureFormat::Rgba8Unorm",
                "normal must remain linear RGBA8",
            ),
            (
                "format: wgpu::TextureFormat::R8Unorm",
                "roughness must remain linear R8",
            ),
        ] {
            assert!(material_creation.contains(needle), "{label}");
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
        read_texture_layer(
            device,
            queue,
            texture,
            level,
            0,
            width,
            height,
            bytes_per_pixel,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn read_texture_layer(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        level: u32,
        array_layer: u32,
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
                origin: wgpu::Origin3d {
                    x: 0,
                    y: 0,
                    z: array_layer,
                },
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
        for (level, width, height) in [(0u32, 1024u32, 1024u32), (5, 32, 32), (10, 1, 1)] {
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
        for (level, width) in [(0u32, 1024u32), (5, 32), (10, 1)] {
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
            1024,
            1024,
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

// ── G3D: vegetation tests ──────────────────────────────────────────────────

/// G3D vegetation tests: CPU/structural checks (always green) plus opt-in
/// headless GPU integration tests (`#[ignore]`, run with
/// `cargo test -p renderer --lib vegetation -- --ignored`).
///
/// The GPU tests drive the REAL production assembly path
/// (`build_gpu_vegetation`, the instanced pipelines, the real asset meshes and
/// `VegetationWorld::update_visibility`) on a headless device, so they
/// discriminate a correct instanced implementation from a no-op: they render
/// actual pixels and assert coverage/transform differences, never just
/// "no panic".
#[cfg(test)]
mod vegetation_tests {
    use super::*;
    use crate::math::look_at_rh;
    use crate::vegetation::VegetationInstance;
    use crate::vegetation::VegetationLodConfig;
    use crate::vegetation_assets::VegetationAssetSet;
    use crate::webgpu_perspective;

    const TEST_SIZE: u32 = 512;

    // ── CPU / structural checks (no GPU) ──────────────────────────────────

    #[test]
    fn vegetation_mesh_index_is_contiguous_and_has_no_gaps() {
        // production assets × 3 LOD × 2 parts = 2×3×3×2 = 36 distinct,
        // contiguous mesh slots.
        let asset_count = VegetationAssetSet::production().len();
        let total = asset_count * LOD_COUNT * PART_COUNT;
        assert_eq!(GROUP_COUNT * PART_COUNT, total);
        let mut seen = vec![false; total];
        for asset in 0..asset_count {
            for lod in 0..LOD_COUNT {
                for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                    let index = vegetation_mesh_index(asset, lod as u8, part);
                    assert!(index < total, "mesh index {index} out of range");
                    assert!(!seen[index], "duplicate mesh index {index}");
                    seen[index] = true;
                }
            }
        }
        assert!(
            seen.iter().all(|&s| s),
            "vegetation mesh slots must be fully covered"
        );
    }

    #[test]
    fn vegetation_mesh_index_matches_group_part_flattening() {
        // The pass loops flatten as group * PART_COUNT + part; the helper must
        // be equivalent for every (asset, lod) group.
        for group in 0..GROUP_COUNT {
            let asset = group / LOD_COUNT;
            let lod = (group % LOD_COUNT) as u8;
            for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                let via_helper = vegetation_mesh_index(asset, lod, part);
                let via_group = group * PART_COUNT + part.index();
                assert_eq!(via_helper, via_group);
            }
        }
    }

    #[test]
    fn vegetation_state_uniform_is_16_bytes() {
        assert_eq!(size_of::<VegetationUniform>(), 16);
        assert_eq!(size_of::<VegetationGpuInstance>(), 48);
        // Instance stride must be a multiple of 16 for vec4 alignment.
        assert_eq!(size_of::<VegetationGpuInstance>() % 16, 0);
    }

    #[test]
    fn draw_call_counts_follow_batches_not_instance_count() {
        // With a fixed visible set, `stats.scene_draw_calls` equals
        // active-groups × parts, and shadow counts only LOD0/1 groups — the
        // same arithmetic the pass loops perform. This pins the draw-call
        // contract: batch-driven, never per-tree.
        let assets = VegetationAssetSet::single_default();
        let config = VegetationLodConfig {
            lod0_max_m: 30.0,
            lod1_max_m: 60.0,
            distance_cull_m: 120.0,
            hysteresis_band: 0.0,
        };
        // Ten instances spread over LOD0 and LOD1 from the test eye.
        let instances = (0..10)
            .map(|i| {
                let z = 10.0 + (i as f32) * 4.0;
                VegetationInstance {
                    position: [0.0, 0.0, z],
                    yaw_rad: 0.0,
                    scale: 1.0,
                    asset_index: 0,
                    tint: [1.0, 1.0, 1.0],
                    zone: 0,
                }
            })
            .collect();
        let mut world = VegetationWorld::new(assets, instances, config);
        let vp = test_view_projection([0.0, 3.0, 14.0]);
        world.update_visibility([0.0, 3.0, 14.0], &vp);
        let stats = *world.stats();
        assert_eq!(stats.total, 10);

        // Recompute the draw loops' arithmetic from the ranges alone.
        let ranges = world.batch_ranges();
        let mut scene_groups = 0u32;
        let mut shadow_groups = 0u32;
        for group in 0..GROUP_COUNT {
            if ranges[group * 2 + 1] > 0 {
                scene_groups += 1;
                if group % LOD_COUNT <= 1 {
                    shadow_groups += 1;
                }
            }
        }
        assert_eq!(stats.scene_draw_calls, scene_groups * PART_COUNT as u32);
        assert_eq!(stats.shadow_draw_calls, shadow_groups * PART_COUNT as u32);
        assert!(
            stats.scene_draw_calls <= (GROUP_COUNT * PART_COUNT) as u32,
            "draw calls must never exceed the batch grid"
        );
    }

    #[test]
    fn vegetation_instance_buffer_bytes_scale_with_visible_count() {
        let assets = VegetationAssetSet::single_default();
        let config = VegetationLodConfig {
            lod0_max_m: 30.0,
            lod1_max_m: 60.0,
            distance_cull_m: 120.0,
            hysteresis_band: 0.0,
        };
        let instances = (0..8)
            .map(|i| VegetationInstance {
                position: [0.0, 0.0, 10.0 + (i as f32) * 4.0],
                yaw_rad: 0.0,
                scale: 1.0,
                asset_index: 0,
                tint: [1.0, 1.0, 1.0],
                zone: 0,
            })
            .collect();
        let mut world = VegetationWorld::new(assets, instances, config);
        let vp = test_view_projection([0.0, 3.0, 14.0]);
        world.update_visibility([0.0, 3.0, 14.0], &vp);
        let bytes = world.visible().len() as u64 * size_of::<VegetationGpuInstance>() as u64;
        assert_eq!(bytes, world.visible().len() as u64 * 48);
        assert!(
            bytes > 0,
            "visible set must not be empty for near instances"
        );
    }

    #[test]
    fn shader_declares_vegetation_entry_points_and_instance_inputs() {
        let source = include_str!("shader.wgsl");
        for entry in ["vs_vegetation", "fs_vegetation", "vs_vegetation_shadow"] {
            assert!(source.contains(entry), "shader must declare {entry}");
        }
        // Instance attributes ride locations 4-6 on slot-1 Instance step mode.
        assert!(
            source.contains("@location(4) instance_position_yaw"),
            "shader must consume the position/yaw instance attribute"
        );
        assert!(
            source.contains("@location(5) instance_scale_tint"),
            "shader must consume the scale/tint instance attribute"
        );
        assert!(
            source.contains("@location(6) instance_lod_class"),
            "shader must consume the LOD-class instance attribute"
        );
        assert!(
            source.contains("vegetation_state.debug_mode"),
            "shader must expose the presentation-only debug selector"
        );
    }

    #[test]
    fn fs_vegetation_has_no_independent_tone_mapping_or_gamma() {
        // The vegetation fragment must reuse the scene PBR chain and leave
        // display encoding to the G3B postprocess pass (no khronos tonemap,
        // no pow() gamma) so trees join the HDR scene, not an LDR side path.
        let source = include_str!("shader.wgsl");
        let body = source
            // PV1-R2: use "fn fs_vegetation(" to avoid matching fs_vegetation_shadow
            .split_once("fn fs_vegetation(")
            .expect("fs_vegetation present")
            .1;
        let body = body
            .split_once("\n}")
            .expect("df: end of fs_vegetation body")
            .0;
        for banned in ["khronos_pbr_neutral", "exp2(", "pow("] {
            assert!(
                !body.contains(banned),
                "fs_vegetation must not {banned} — tone mapping stays in fs_postprocess"
            );
        }
        // ...but it must reuse the shared scene lighting chain.
        for required in ["lit_pbr_response", "apply_distance_fog"] {
            assert!(
                body.contains(required),
                "fs_vegetation must reuse {required}"
            );
        }
    }

    fn test_view_projection(eye: [f32; 3]) -> Mat4 {
        let view = look_at_rh(eye, [0.0; 3], [0.0, 1.0, 0.0]);
        let projection = webgpu_perspective(60.0_f32.to_radians(), 1.0, 0.05, 2_000.0)
            .expect("projection must be valid");
        projection * view
    }

    // ── GPU integration tests (opt-in, real production path) ──────────────

    fn headless_device_gpu() -> (wgpu::Device, wgpu::Queue) {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            apply_limit_buckets: false,
        }))
        .expect("no wgpu adapter available on this machine");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("g3d vegetation headless test device"),
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

    /// Read an offscreen texture back to CPU with WebGPU row alignment,
    /// stripping the per-row padding (same contract as the terrain tests).
    fn read_texture_level(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        level: u32,
        width: u32,
        height: u32,
        bytes_per_pixel: u32,
    ) -> Vec<u8> {
        read_vegetation_texture_layer(
            device,
            queue,
            texture,
            level,
            0,
            width,
            height,
            bytes_per_pixel,
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn read_vegetation_texture_layer(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        level: u32,
        array_layer: u32,
        width: u32,
        height: u32,
        bytes_per_pixel: u32,
    ) -> Vec<u8> {
        let row_bytes = padded_bytes_per_row_checked_for_bytes_per_pixel(width, bytes_per_pixel)
            .expect("row padding must not overflow");
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("g3d vegetation readback buffer"),
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
                origin: wgpu::Origin3d {
                    x: 0,
                    y: 0,
                    z: array_layer,
                },
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

    /// Render `instances` with the REAL production vegetation assembly path
    /// and return the HDR readback of an offscreen 512² target (clear gray
    /// background). `lod_class` selects the LOD forced via the LOD config;
    /// `debug_mode` drives the presentation-only state uniform like the CLI.
    fn render_vegetation_offscreen(
        positions: &[[f32; 3]],
        lod_class: u8,
        debug_mode: VegetationDebugMode,
    ) -> Vec<u8> {
        let (device, queue) = headless_device_gpu();

        let assets = VegetationAssetSet::single_default();
        let instances: Vec<VegetationInstance> = positions
            .iter()
            .map(|&position| VegetationInstance {
                position,
                yaw_rad: 0.0,
                scale: 1.0,
                asset_index: 0,
                tint: [1.0, 1.0, 1.0],
                zone: 0,
            })
            .collect();
        // LOD thresholds tuned so a ~14 m camera distance lands exactly in the
        // requested class with the hysteresis-start state.
        let config = match lod_class {
            0 => VegetationLodConfig {
                lod0_max_m: 30.0,
                lod1_max_m: 60.0,
                distance_cull_m: 120.0,
                hysteresis_band: 0.0,
            },
            1 => VegetationLodConfig {
                lod0_max_m: 10.0,
                lod1_max_m: 50.0,
                distance_cull_m: 120.0,
                hysteresis_band: 0.0,
            },
            _ => VegetationLodConfig {
                lod0_max_m: 5.0,
                lod1_max_m: 10.0,
                distance_cull_m: 120.0,
                hysteresis_band: 0.0,
            },
        };
        let mut world = VegetationWorld::new(assets, instances, config);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("g3d vegetation test shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        // Material bind groups (production path: white texture + part factors).
        let material_layout = material_bind_group_layout(&device, "g3d test material layout");
        let bark_material = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Bark),
            part_roughness(VegetationPart::Bark),
        );
        let foliage_material = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Foliage),
            part_roughness(VegetationPart::Foliage),
        );

        // Pipeline layouts / bind groups (identical to the production wiring).
        let camera_layout = camera_bind_group_layout(&device, "g3d test camera layout");
        let object_layout = matrix_bind_group_layout(&device, "g3d test object layout");
        let env_layout = environment_bind_group_layout(&device, "g3d test env layout");
        let shadow_pass_layout = shadow_pass_bind_group_layout(&device, "g3d test shadow layout");
        let state_layout = vegetation_state_bind_group_layout(&device, "g3d test state layout");
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("g3d vegetation test layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&env_layout),
                Some(&material_layout),
                Some(&state_layout),
            ],
            immediate_size: 0,
        });
        let shadow_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("g3d vegetation test shadow layout"),
                bind_group_layouts: &[
                    Some(&camera_layout),
                    Some(&object_layout),
                    Some(&shadow_pass_layout),
                    Some(&material_layout),
                ],
                immediate_size: 0,
            });

        let eye = [0.0, 3.0, 14.0];
        let view_projection = test_view_projection(eye);
        let camera_uniform = CameraUniform::new(
            &view_projection,
            &view_projection.inverse().expect("vp invertible"),
            eye,
        );
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d test camera buffer"),
            contents: bytemuck::bytes_of(&camera_uniform),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_bind_group =
            camera_bind_group(&device, &camera_layout, &camera_buffer, "g3d camera group");
        let identity_object = ObjectUniform::from_matrix(&Mat4::identity());
        let object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d test object buffer"),
            contents: bytemuck::bytes_of(&identity_object),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let object_bind_group =
            matrix_bind_group(&device, &object_layout, &object_buffer, "g3d object group");
        let environment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d test environment buffer"),
            contents: bytemuck::bytes_of(&EnvironmentUniform::default_environment()),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d test shadow matrix buffer"),
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
            "g3d test environment bind group",
        );
        let _shadow_pass_bind_group = create_shadow_pass_bind_group(
            &device,
            &shadow_pass_layout,
            &shadow_matrix_buffer,
            "g3d test shadow pass group",
        );

        // Production assembly: real `build_gpu_vegetation` (persistent buffers,
        // instance buffer, pipelines).
        let mut materials = vec![bark_material, foliage_material];
        let gpu = build_gpu_vegetation(
            &device,
            &queue,
            &shader,
            &world,
            &state_layout,
            &pipeline_layout,
            &shadow_pipeline_layout,
            &material_layout,
            &mut materials,
            0,
            1,
            debug_mode,
        );
        // NOTE: bark/foliage material indexes in the test are 0/1 (they are the
        // only materials pushed here — the fallback is deliberately absent).

        // Visibility + instance upload (the exact per-frame renderer steps). Two
        // updates because hysteresis ramps 0 → 1 → 2 across frames; classes 0
        // and 1 are stable under the second update.
        world.update_visibility(eye, &view_projection);
        if lod_class >= 2 {
            world.update_visibility(eye, &view_projection);
        }
        let visible = world.visible();
        debug_assert!(!visible.is_empty());
        queue.write_buffer(&gpu.instance_buffer, 0, bytemuck::cast_slice(visible));

        let format = HDR_FORMAT;
        let color_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("g3d test color target"),
            size: wgpu::Extent3d {
                width: TEST_SIZE,
                height: TEST_SIZE,
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
        let depth_target = create_depth_target(&device, TEST_SIZE, TEST_SIZE);

        // Scene pass draw (mirrors `WgpuRenderer::render` vegetation block).
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("g3d vegetation headless scene pass"),
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
            pass.set_pipeline(&gpu.pipeline);
            pass.set_bind_group(0, &camera_bind_group, &[]);
            pass.set_bind_group(1, &object_bind_group, &[]);
            pass.set_bind_group(2, &environment_bind_group, &[]);
            pass.set_bind_group(4, &gpu.uniform_bind_group, &[]);
            let ranges = world.batch_ranges();
            for group in 0..GROUP_COUNT {
                let start = ranges[group * 2];
                let count = ranges[group * 2 + 1];
                if count == 0 {
                    continue;
                }
                let asset = group / LOD_COUNT;
                let lod = (group % LOD_COUNT) as u8;
                for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                    let mesh = &gpu.meshes[vegetation_mesh_index(asset, lod, part)];
                    let material = if part == VegetationPart::Bark {
                        &materials[0]
                    } else {
                        &materials[1]
                    };
                    pass.set_bind_group(3, &material.bind_group, &[]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_vertex_buffer(1, gpu.instance_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count, 0, start..start + count);
                }
            }
        }
        queue.submit([encoder.finish()]);

        read_texture_level(&device, &queue, &color_texture, 0, TEST_SIZE, TEST_SIZE, 8)
    }

    /// Decode one IEEE binary16 (WebGPU half-float texel) to f32.
    fn f16_to_f32(bits: u16) -> f32 {
        let sign = (bits & 0x8000) as u32;
        let exponent = ((bits >> 10) & 0x1f) as u32;
        let mantissa = (bits & 0x03ff) as u32;
        let bits32 = if exponent == 0 {
            // Subnormal half → normalize into f32 (masking sign out above).
            let mut e = -14i32;
            let mut m = mantissa;
            if m == 0 {
                sign << 31
            } else {
                while m & 0x0400 == 0 {
                    m <<= 1;
                    e -= 1;
                }
                let exp = (e + 127) as u32;
                (sign << 31) | (exp << 23) | ((m & 0x03ff) << 13)
            }
        } else if exponent == 0x1f {
            // Inf/NaN passthrough.
            (sign << 31) | (0xff << 23) | (mantissa << 13)
        } else {
            (sign << 31) | ((exponent + 112) << 23) | (mantissa << 13)
        };
        f32::from_bits(bits32)
    }

    /// X centroid (px) of the non-background pixels; `None` when empty.
    fn coverage_centroid_x(pixels: &[u8]) -> Option<u32> {
        let mut sum = 0u64;
        let mut count = 0u64;
        for (index, texel) in pixels.as_chunks::<8>().0.iter().enumerate() {
            let r = f16_to_f32(u16::from_le_bytes([texel[0], texel[1]]));
            let g = f16_to_f32(u16::from_le_bytes([texel[2], texel[3]]));
            let b = f16_to_f32(u16::from_le_bytes([texel[4], texel[5]]));
            let delta = (r - 0.2).abs().max((g - 0.2).abs()).max((b - 0.2).abs());
            if delta > 0.02 {
                count += 1;
                sum += (index as u32 % TEST_SIZE) as u64;
            }
        }
        sum.checked_div(count).map(|centroid| centroid as u32)
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn vegetation_renders_instances_at_distinct_transforms() {
        // Instance A alone (left of centre) vs instance B alone (right) must
        // project to measurably different screen centroids — the instance
        // transform is actually applied, not a no-op.
        let left = render_vegetation_offscreen(&[[-4.0, 0.0, 0.0]], 0, VegetationDebugMode::Final);
        let right = render_vegetation_offscreen(&[[4.0, 0.0, 0.0]], 0, VegetationDebugMode::Final);
        let centroid_left = coverage_centroid_x(&left).expect("left instance must render");
        let centroid_right = coverage_centroid_x(&right).expect("right instance must render");
        assert!(
            centroid_right.saturating_sub(centroid_left) > TEST_SIZE / 6,
            "distinct instance positions must project apart (left {centroid_left} px, right {centroid_right} px)"
        );
    }

    /// Classify every non-background texel against the deterministic LOD debug
    /// palette (LOD0 green, LOD1 yellow, LOD2 orange) and return per-class counts.
    fn classify_lod_debug_pixels(pixels: &[u8]) -> [u64; 3] {
        const PALETTE: [[f32; 3]; 3] = [[0.05, 0.65, 0.15], [0.85, 0.70, 0.10], [0.90, 0.42, 0.08]];
        let mut counts = [0u64; 3];
        for texel in pixels.as_chunks::<8>().0 {
            let rgb = [
                f16_to_f32(u16::from_le_bytes([texel[0], texel[1]])),
                f16_to_f32(u16::from_le_bytes([texel[2], texel[3]])),
                f16_to_f32(u16::from_le_bytes([texel[4], texel[5]])),
            ];
            let mut nearest = 0usize;
            let mut nearest_distance = f32::INFINITY;
            for (class, target) in PALETTE.iter().enumerate() {
                let distance = (rgb[0] - target[0]).powi(2)
                    + (rgb[1] - target[1]).powi(2)
                    + (rgb[2] - target[2]).powi(2);
                if distance < nearest_distance {
                    nearest = class;
                    nearest_distance = distance;
                }
            }
            if nearest_distance < 0.05 {
                counts[nearest] += 1;
            }
        }
        counts
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn vegetation_all_lod_classes_render_with_decreasing_cost() {
        // The debug LOD channel must color each class deterministically
        // (LOD0 green, LOD1 yellow, LOD2 orange): this discriminates the
        // SELECTED LOD per instance, not just "something rendered", and the
        // near/far cost ladder is pinned by the asset triangle tests. The
        // hit counts also prove each class actually renders geometry.
        for (class, expected_class) in [(0usize, 0usize), (1, 1), (2, 2)] {
            let pixels = render_vegetation_offscreen(
                &[[0.0, 0.0, 0.0]],
                class as u8,
                VegetationDebugMode::Lod,
            );
            let counts = classify_lod_debug_pixels(&pixels);
            assert!(
                counts[expected_class] > 0,
                "LOD{class} render must contain real class-{expected_class} pixels, got {counts:?}"
            );
            let dominant = counts
                .iter()
                .enumerate()
                .max_by_key(|(_, count)| **count)
                .map(|(class, _)| class)
                .expect("non-empty frame");
            assert_eq!(
                dominant, expected_class,
                "class {class} render must dominate the {expected_class} palette, got {counts:?}"
            );
        }
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn vegetation_shadow_pass_produces_silhouettes() {
        // The instanced shadow caster must write the tree into the fixed
        // directional depth map: center texels nearer than the far plane.
        let (device, queue) = headless_device_gpu();
        let assets = VegetationAssetSet::single_default();
        let instances = vec![VegetationInstance {
            position: [0.0, 0.0, 0.0],
            yaw_rad: 0.0,
            scale: 1.0,
            asset_index: 0,
            tint: [1.0, 1.0, 1.0],
            zone: 0,
        }];
        let config = VegetationLodConfig {
            lod0_max_m: 30.0,
            lod1_max_m: 60.0,
            distance_cull_m: 120.0,
            hysteresis_band: 0.0,
        };
        let mut world = VegetationWorld::new(assets, instances, config);
        let eye = [0.0, 3.0, 14.0];
        let vp = test_view_projection(eye);
        world.update_visibility(eye, &vp);
        assert_eq!(
            world.stats().lod_counts[0],
            1,
            "the single instance must be LOD0 for the shadow test"
        );

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("g3d vegetation shadow test shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let camera_layout = camera_bind_group_layout(&device, "g3d shadow test camera layout");
        let object_layout = matrix_bind_group_layout(&device, "g3d shadow test object layout");
        let shadow_pass_layout =
            shadow_pass_bind_group_layout(&device, "g3d shadow test shadow layout");
        let material_layout =
            material_bind_group_layout(&device, "g3d shadow test material layout");
        let shadow_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("g3d shadow vegetation pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_layout),
                    Some(&object_layout),
                    Some(&shadow_pass_layout),
                    Some(&material_layout),
                ],
                immediate_size: 0,
            });

        // Build the exact caster pipeline the renderer uses (depth-only,
        // no material/state groups, same shadow depth bias).
        let shadow_pipeline =
            create_vegetation_shadow_pipeline(&device, &shader, &shadow_pipeline_layout);

        // A simpler one-mesh upload for the depth test: LOD0 bark+foliage of
        // asset 0, drawn with the same instance buffer path.
        let asset = world.assets().get(0).expect("asset 0 present");
        let lod = asset.lods.lod(0).expect("lod0 present");
        let meshes = [&lod.bark, &lod.foliage];
        let vertex_buffers: Vec<wgpu::Buffer> = meshes
            .iter()
            .map(|mesh| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("g3d shadow test vertices"),
                    contents: bytemuck::cast_slice(mesh.vertices()),
                    usage: wgpu::BufferUsages::VERTEX,
                })
            })
            .collect();
        let index_buffers: Vec<wgpu::Buffer> = meshes
            .iter()
            .map(|mesh| {
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("g3d shadow test indices"),
                    contents: bytemuck::cast_slice(mesh.indices()),
                    usage: wgpu::BufferUsages::INDEX,
                })
            })
            .collect();
        let index_counts: Vec<u32> = meshes
            .iter()
            .map(|mesh| mesh.indices().len() as u32)
            .collect();

        let instance_capacity = world.instance_capacity();
        let instance_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("g3d shadow test instance buffer"),
            size: (instance_capacity * size_of::<VegetationGpuInstance>()) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        queue.write_buffer(&instance_buffer, 0, bytemuck::cast_slice(world.visible()));

        // Light eye: above and to the side (matches the shipped direction).
        let light_direction = EnvironmentUniform::default_environment().light_direction;
        let light_dir = {
            let len = (light_direction[0] * light_direction[0]
                + light_direction[1] * light_direction[1]
                + light_direction[2] * light_direction[2])
                .sqrt();
            [
                light_direction[0] / len,
                light_direction[1] / len,
                light_direction[2] / len,
            ]
        };
        let light_eye = [
            light_dir[0] * 25.0,
            light_dir[1] * 25.0,
            light_dir[2] * 25.0,
        ];
        let light_view = look_at_rh(light_eye, [0.0; 3], [0.0, 1.0, 0.0]);
        let light_projection =
            webgpu_perspective(35.0_f32.to_radians(), 1.0, 0.1, 80.0).expect("light projection");
        let light_vp = light_projection * light_view;

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d shadow test camera buffer"),
            contents: bytemuck::bytes_of(&CameraUniform::new(&light_vp, &light_vp, light_eye)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_bind_group = camera_bind_group(
            &device,
            &camera_layout,
            &camera_buffer,
            "g3d shadow camera group",
        );
        let object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d shadow test object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let object_bind_group = matrix_bind_group(
            &device,
            &object_layout,
            &object_buffer,
            "g3d shadow object group",
        );
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("g3d shadow test matrix buffer"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_matrix(&light_vp)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_pass_bind_group = create_shadow_pass_bind_group(
            &device,
            &shadow_pass_layout,
            &shadow_matrix_buffer,
            "g3d shadow group",
        );

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("g3d shadow test depth target"),
            size: wgpu::Extent3d {
                width: TEST_SIZE,
                height: TEST_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("g3d vegetation shadow headless pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
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
            pass.set_pipeline(&shadow_pipeline);
            pass.set_bind_group(0, &camera_bind_group, &[]);
            pass.set_bind_group(1, &object_bind_group, &[]);
            pass.set_bind_group(2, &shadow_pass_bind_group, &[]);
            // PV1-R2: the shadow fragment entry samples the material alpha for
            // foliage discard; bind bark/foliage materials as production does.
            let shadow_materials = [
                create_white_texture_material(
                    &device,
                    &material_layout,
                    &queue,
                    part_metallic(VegetationPart::Bark),
                    part_roughness(VegetationPart::Bark),
                ),
                create_white_texture_material(
                    &device,
                    &material_layout,
                    &queue,
                    part_metallic(VegetationPart::Foliage),
                    part_roughness(VegetationPart::Foliage),
                ),
            ];
            for (mesh_index, ((vertex_buffer, index_buffer), index_count)) in vertex_buffers
                .iter()
                .zip(index_buffers.iter())
                .zip(index_counts.iter())
                .enumerate()
            {
                pass.set_bind_group(3, &shadow_materials[mesh_index].bind_group, &[]);
                pass.set_vertex_buffer(0, vertex_buffer.slice(..));
                pass.set_vertex_buffer(1, instance_buffer.slice(..));
                pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..*index_count, 0, 0..1);
            }
        }
        queue.submit([encoder.finish()]);

        let depth_bytes =
            read_texture_level(&device, &queue, &depth_texture, 0, TEST_SIZE, TEST_SIZE, 4);
        // Count texels strictly nearer than the far plane in the centre region
        // (the tree silhouette projects there).
        let mut nearer = 0u64;
        let half = TEST_SIZE / 2;
        for y in (half - half / 4)..(half + half / 4) {
            for x in (half - half / 4)..(half + half / 4) {
                let offset = ((y * TEST_SIZE + x) * 4) as usize;
                let depth = f32::from_le_bytes([
                    depth_bytes[offset],
                    depth_bytes[offset + 1],
                    depth_bytes[offset + 2],
                    depth_bytes[offset + 3],
                ]);
                if depth < 1.0 {
                    nearer += 1;
                }
            }
        }
        assert!(
            nearer > 0,
            "vegetation shadow caster must write a silhouette into the depth map"
        );
    }

    // ── G3D FIX: pipeline-state isolation regression tests ─────────────────
    //
    // These tests reproduce the exact bug where the vegetation pipeline leaked
    // into subsequent aircraft draws in the same render pass. Without the
    // explicit `set_pipeline` restore, the aircraft mesh is drawn through the
    // instanced vegetation entry points and inherits the vegetation instance
    // transform instead of its own object uniform.

    /// Build a minimal "aircraft-like" mesh (one triangle, ~2 m across) and
    /// upload it to the GPU. Returns (vertex_buffer, index_buffer, index_count).
    fn create_test_aircraft_mesh(device: &wgpu::Device) -> (wgpu::Buffer, wgpu::Buffer, u32) {
        let vertices = [
            Vertex {
                position: [-1.0, 0.0, 0.0],
                normal: [0.0, 1.0, 0.0],
                color: [1.0, 0.0, 0.0, 1.0],
                uv: [0.0, 0.0],
            },
            Vertex {
                position: [1.0, 0.0, 0.0],
                normal: [0.0, 1.0, 0.0],
                color: [1.0, 0.0, 0.0, 1.0],
                uv: [1.0, 0.0],
            },
            Vertex {
                position: [0.0, 0.0, -1.5],
                normal: [0.0, 1.0, 0.0],
                color: [1.0, 0.0, 0.0, 1.0],
                uv: [0.5, 1.0],
            },
        ];
        let indices: [u32; 3] = [0, 1, 2];
        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test aircraft vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("test aircraft indices"),
            contents: bytemuck::cast_slice(&indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        (vb, ib, 3)
    }

    /// Scene-pass regression: vegetation draw followed by an aircraft draw in
    /// the SAME pass. The aircraft object transform places it at x = +10; the
    /// sole vegetation instance sits at x = −20. If the vegetation pipeline
    /// leaks, the aircraft triangle is transformed by the instance buffer and
    /// lands near x = −20 instead. The test asserts the aircraft centroid is
    /// clearly right-of-centre (its own transform), not left (vegetation).
    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn scene_pass_aircraft_not_displaced_by_vegetation_pipeline() {
        let (device, queue) = headless_device_gpu();
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("pipeline isolation test shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        // ── Bind group layouts (same as production) ──
        let camera_layout = camera_bind_group_layout(&device, "iso test camera layout");
        let object_layout = matrix_bind_group_layout(&device, "iso test object layout");
        let env_layout = environment_bind_group_layout(&device, "iso test env layout");
        let material_layout = material_bind_group_layout(&device, "iso test material layout");
        let state_layout = vegetation_state_bind_group_layout(&device, "iso test state layout");

        // ── Standard lit pipeline (aircraft path) ──
        let lit_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("iso test lit layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&env_layout),
                Some(&material_layout),
            ],
            immediate_size: 0,
        });
        let triangle_pipeline = create_pipeline(
            &device,
            &shader,
            &lit_layout,
            HDR_FORMAT,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                depth_write_enabled: true,
                label: "iso test lit triangle pipeline",
                fragment_entry_point: "fs_lit",
            },
        );

        // ── Vegetation pipeline ──
        let veg_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("iso test vegetation layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&env_layout),
                Some(&material_layout),
                Some(&state_layout),
            ],
            immediate_size: 0,
        });
        let shadow_pass_layout =
            shadow_pass_bind_group_layout(&device, "iso test shadow pass layout");
        let veg_shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("iso test vegetation shadow layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&shadow_pass_layout),
                Some(&material_layout),
            ],
            immediate_size: 0,
        });

        // ── Vegetation world: one instance far LEFT (x = −20) ──
        let assets = VegetationAssetSet::single_default();
        let instances = vec![VegetationInstance {
            position: [-20.0, 0.0, 0.0],
            yaw_rad: 0.0,
            scale: 1.0,
            asset_index: 0,
            tint: [1.0, 1.0, 1.0],
            zone: 0,
        }];
        let config = VegetationLodConfig {
            lod0_max_m: 60.0,
            lod1_max_m: 120.0,
            distance_cull_m: 300.0,
            hysteresis_band: 0.0,
        };
        let mut world = VegetationWorld::new(assets, instances, config);

        // ── Camera looking at origin from z = +30 ──
        let eye = [0.0, 5.0, 30.0];
        let vp = test_view_projection(eye);
        world.update_visibility(eye, &vp);
        assert!(
            !world.visible().is_empty(),
            "vegetation instance must be visible"
        );

        // ── GPU vegetation assembly ──
        let bark_mat = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Bark),
            part_roughness(VegetationPart::Bark),
        );
        let foliage_mat = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Foliage),
            part_roughness(VegetationPart::Foliage),
        );
        let mut materials = vec![bark_mat, foliage_mat];
        let gpu_veg = build_gpu_vegetation(
            &device,
            &queue,
            &shader,
            &world,
            &state_layout,
            &veg_layout,
            &veg_shadow_layout,
            &material_layout,
            &mut materials,
            0,
            1,
            VegetationDebugMode::Final,
        );
        queue.write_buffer(
            &gpu_veg.instance_buffer,
            0,
            bytemuck::cast_slice(world.visible()),
        );

        // ── Aircraft mesh + object transform at x = +10 ──
        let (aircraft_vb, aircraft_ib, aircraft_ic) = create_test_aircraft_mesh(&device);
        let aircraft_matrix = Mat4::from_rows([
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let aircraft_object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("iso test aircraft object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&aircraft_matrix)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let aircraft_object_bg = matrix_bind_group(
            &device,
            &object_layout,
            &aircraft_object_buffer,
            "iso test aircraft object group",
        );

        // ── Shared bind groups ──
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("iso test camera buffer"),
            contents: bytemuck::bytes_of(&CameraUniform::new(
                &vp,
                &vp.inverse().expect("vp invertible"),
                eye,
            )),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_bg =
            camera_bind_group(&device, &camera_layout, &camera_buffer, "iso camera group");
        let identity_obj_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("iso test identity object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let identity_obj_bg = matrix_bind_group(
            &device,
            &object_layout,
            &identity_obj_buffer,
            "iso identity object group",
        );
        let env_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("iso test environment buffer"),
            contents: bytemuck::bytes_of(&EnvironmentUniform::default_environment()),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("iso test shadow matrix buffer"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_target = create_shadow_target(&device);
        let shadow_sampler = create_shadow_comparison_sampler(&device);
        let env_bg = create_environment_bind_group(
            &device,
            &env_layout,
            &env_buffer,
            &shadow_target.view,
            &shadow_sampler,
            &shadow_matrix_buffer,
            "iso test env group",
        );
        // Aircraft material: white texture, non-metal, moderate roughness.
        let aircraft_mat = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            PROCEDURAL_METALLIC,
            PROCEDURAL_ROUGHNESS,
        );

        // ── Offscreen HDR target ──
        let color_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("iso test color target"),
            size: wgpu::Extent3d {
                width: TEST_SIZE,
                height: TEST_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: HDR_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let depth_target = create_depth_target(&device, TEST_SIZE, TEST_SIZE);

        // ── Render: vegetation THEN aircraft in the SAME pass ──
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("iso test scene pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.1,
                            g: 0.1,
                            b: 0.1,
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

            // 1. Vegetation draw (instanced pipeline, instance buffer on slot 1).
            pass.set_pipeline(&gpu_veg.pipeline);
            pass.set_bind_group(0, &camera_bg, &[]);
            pass.set_bind_group(1, &identity_obj_bg, &[]);
            pass.set_bind_group(2, &env_bg, &[]);
            pass.set_bind_group(4, &gpu_veg.uniform_bind_group, &[]);
            let ranges = world.batch_ranges();
            for group in 0..GROUP_COUNT {
                let start = ranges[group * 2];
                let count = ranges[group * 2 + 1];
                if count == 0 {
                    continue;
                }
                let asset = group / LOD_COUNT;
                let lod = (group % LOD_COUNT) as u8;
                for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                    let mesh = &gpu_veg.meshes[vegetation_mesh_index(asset, lod, part)];
                    let material = if part == VegetationPart::Bark {
                        &materials[0]
                    } else {
                        &materials[1]
                    };
                    pass.set_bind_group(3, &material.bind_group, &[]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_vertex_buffer(1, gpu_veg.instance_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count, 0, start..start + count);
                }
            }

            // 2. THE FIX: restore standard lit pipeline before aircraft draw.
            pass.set_pipeline(&triangle_pipeline);

            // 3. Aircraft draw (non-instanced, own object transform at x=+10).
            pass.set_bind_group(0, &camera_bg, &[]);
            pass.set_bind_group(1, &aircraft_object_bg, &[]);
            pass.set_bind_group(2, &env_bg, &[]);
            pass.set_bind_group(3, &aircraft_mat.bind_group, &[]);
            pass.set_vertex_buffer(0, aircraft_vb.slice(..));
            pass.set_index_buffer(aircraft_ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..aircraft_ic, 0, 0..1);
        }
        queue.submit([encoder.finish()]);

        // ── Readback and centroid analysis ──
        let pixels =
            read_texture_level(&device, &queue, &color_texture, 0, TEST_SIZE, TEST_SIZE, 8);

        // Find the centroid of RED-dominant pixels (the aircraft triangle is
        // pure red [1,0,0]; vegetation is green/brown from the asset colors).
        let mut red_sum_x = 0u64;
        let mut red_count = 0u64;
        for (index, texel) in pixels.as_chunks::<8>().0.iter().enumerate() {
            let r = f16_to_f32(u16::from_le_bytes([texel[0], texel[1]]));
            let g = f16_to_f32(u16::from_le_bytes([texel[2], texel[3]]));
            let b = f16_to_f32(u16::from_le_bytes([texel[4], texel[5]]));
            // Red-dominant and above background.
            if r > 0.15 && r > g * 1.5 && r > b * 1.5 {
                red_count += 1;
                red_sum_x += (index as u32 % TEST_SIZE) as u64;
            }
        }
        assert!(red_count > 0, "aircraft triangle must render red pixels");
        let centroid_x = (red_sum_x / red_count) as u32;
        let centre = TEST_SIZE / 2;

        // The aircraft object transform places it at x = +10 (right of centre);
        // the vegetation instance is at x = −20 (left). If the pipeline leaked,
        // the aircraft would inherit the vegetation instance transform and its
        // red centroid would land LEFT of centre.
        assert!(
            centroid_x > centre,
            "aircraft centroid must be RIGHT of centre ({centroid_x} px > {centre} px) — \
             a leftward shift means the vegetation pipeline leaked into the aircraft draw"
        );
    }

    /// G3E GPU smoke: the production caster pipeline writes all three layers
    /// through their persistent per-cascade uniforms and attachment views.
    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn three_cascade_gpu_smoke_writes_every_depth_layer() {
        let (device, queue) = headless_device_gpu();
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("G3E three-cascade smoke shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });
        let camera_layout = camera_bind_group_layout(&device, "G3E smoke camera layout");
        let object_layout = matrix_bind_group_layout(&device, "G3E smoke object layout");
        let shadow_pass_layout = shadow_pass_bind_group_layout(&device, "G3E smoke pass layout");
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G3E smoke pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&shadow_pass_layout),
            ],
            immediate_size: 0,
        });
        let pipeline = create_shadow_pipeline(&device, &shader, &pipeline_layout);

        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("G3E smoke camera buffer"),
            contents: bytemuck::bytes_of(&CameraUniform::new(
                &Mat4::identity(),
                &Mat4::identity(),
                [0.0, 2.0, 8.0],
            )),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_group = camera_bind_group(
            &device,
            &camera_layout,
            &camera_buffer,
            "G3E smoke camera group",
        );
        let object_matrix = Mat4::from_rows([
            [8.0, 0.0, 0.0, 0.0],
            [0.0, 8.0, 0.0, 0.0],
            [0.0, 0.0, 8.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("G3E smoke object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&object_matrix)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let object_group = matrix_bind_group(
            &device,
            &object_layout,
            &object_buffer,
            "G3E smoke object group",
        );
        let cascades = build_shadow_cascades([0.4, 0.8, -0.3], [0.0, 2.0, 8.0], [0.0, 0.0, 0.0]);
        let cascade_buffers: [wgpu::Buffer; SHADOW_CASCADE_COUNT] = std::array::from_fn(|index| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("G3E smoke cascade buffer"),
                contents: bytemuck::bytes_of(&ShadowCascadeUniform::from_cascade(&cascades[index])),
                usage: wgpu::BufferUsages::UNIFORM,
            })
        });
        let cascade_groups: [wgpu::BindGroup; SHADOW_CASCADE_COUNT] =
            std::array::from_fn(|index| {
                create_shadow_pass_bind_group(
                    &device,
                    &shadow_pass_layout,
                    &cascade_buffers[index],
                    "G3E smoke cascade group",
                )
            });
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("G3E smoke depth array"),
            size: wgpu::Extent3d {
                width: TEST_SIZE,
                height: TEST_SIZE,
                depth_or_array_layers: SHADOW_CASCADE_COUNT as u32,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let layer_views: [wgpu::TextureView; SHADOW_CASCADE_COUNT] = std::array::from_fn(|index| {
            depth_texture.create_view(&wgpu::TextureViewDescriptor {
                label: Some("G3E smoke cascade attachment"),
                dimension: Some(wgpu::TextureViewDimension::D2),
                base_array_layer: index as u32,
                array_layer_count: Some(1),
                ..Default::default()
            })
        });
        let (vertex_buffer, index_buffer, index_count) = create_test_aircraft_mesh(&device);

        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        for index in 0..SHADOW_CASCADE_COUNT {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("G3E smoke cascade pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &layer_views[index],
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
            pass.set_bind_group(0, &camera_group, &[]);
            pass.set_bind_group(1, &object_group, &[]);
            pass.set_bind_group(2, &cascade_groups[index], &[]);
            pass.set_vertex_buffer(0, vertex_buffer.slice(..));
            pass.set_index_buffer(index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..index_count, 0, 0..1);
        }
        queue.submit([encoder.finish()]);

        for layer in 0..SHADOW_CASCADE_COUNT as u32 {
            let depth = read_vegetation_texture_layer(
                &device,
                &queue,
                &depth_texture,
                0,
                layer,
                TEST_SIZE,
                TEST_SIZE,
                4,
            );
            let written = depth
                .as_chunks::<4>()
                .0
                .iter()
                .filter(|texel| f32::from_le_bytes(**texel) < 1.0)
                .count();
            assert!(
                written > 0,
                "cascade layer {layer} must contain caster depth"
            );
        }
    }

    /// Shadow-pass regression: vegetation shadow draw followed by an aircraft
    /// shadow draw in the SAME depth-only pass. Without the pipeline restore
    /// the aircraft caster uses `vs_vegetation_shadow` and the instance buffer,
    /// displacing the shadow silhouette to the vegetation position.
    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn shadow_pass_aircraft_silhouette_not_displaced_by_vegetation() {
        let (device, queue) = headless_device_gpu();
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("shadow isolation test shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let camera_layout = camera_bind_group_layout(&device, "shadow iso camera layout");
        let object_layout = matrix_bind_group_layout(&device, "shadow iso object layout");
        let shadow_pass_layout = shadow_pass_bind_group_layout(&device, "shadow iso shadow layout");
        let material_layout = material_bind_group_layout(&device, "shadow iso material layout");

        // Standard shadow pipeline (aircraft caster path).
        let shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow iso standard layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&shadow_pass_layout),
            ],
            immediate_size: 0,
        });
        let shadow_pipeline = create_shadow_pipeline(&device, &shader, &shadow_layout);

        // Vegetation shadow pipeline.
        let veg_shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow iso vegetation layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&shadow_pass_layout),
                Some(&material_layout),
            ],
            immediate_size: 0,
        });

        // Vegetation world: one instance far LEFT (x = −20).
        let assets = VegetationAssetSet::single_default();
        let instances = vec![VegetationInstance {
            position: [-20.0, 0.0, 0.0],
            yaw_rad: 0.0,
            scale: 1.0,
            asset_index: 0,
            tint: [1.0, 1.0, 1.0],
            zone: 0,
        }];
        let config = VegetationLodConfig {
            lod0_max_m: 60.0,
            lod1_max_m: 120.0,
            distance_cull_m: 300.0,
            hysteresis_band: 0.0,
        };
        let mut world = VegetationWorld::new(assets, instances, config);

        // Light camera: overhead, looking at origin.
        let light_eye = [0.0, 40.0, 10.0];
        let light_view = look_at_rh(light_eye, [0.0; 3], [0.0, 0.0, -1.0]);
        let light_proj =
            webgpu_perspective(50.0_f32.to_radians(), 1.0, 0.5, 200.0).expect("light proj");
        let light_vp = light_proj * light_view;
        world.update_visibility(light_eye, &light_vp);
        assert!(
            !world.visible().is_empty(),
            "vegetation instance must be visible for shadow test"
        );

        // Build vegetation GPU resources (shadow pipeline only needed here).
        let state_layout = vegetation_state_bind_group_layout(&device, "shadow iso state layout");
        let env_layout = environment_bind_group_layout(&device, "shadow iso env layout");
        let veg_scene_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow iso veg scene layout"),
            bind_group_layouts: &[
                Some(&camera_layout),
                Some(&object_layout),
                Some(&env_layout),
                Some(&material_layout),
                Some(&state_layout),
            ],
            immediate_size: 0,
        });
        let bark_m = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Bark),
            part_roughness(VegetationPart::Bark),
        );
        let foliage_m = create_white_texture_material(
            &device,
            &material_layout,
            &queue,
            part_metallic(VegetationPart::Foliage),
            part_roughness(VegetationPart::Foliage),
        );
        let mut mats = vec![bark_m, foliage_m];
        let gpu_veg = build_gpu_vegetation(
            &device,
            &queue,
            &shader,
            &world,
            &state_layout,
            &veg_scene_layout,
            &veg_shadow_layout,
            &material_layout,
            &mut mats,
            0,
            1,
            VegetationDebugMode::Final,
        );
        queue.write_buffer(
            &gpu_veg.instance_buffer,
            0,
            bytemuck::cast_slice(world.visible()),
        );

        // Aircraft mesh + object transform at x = +10.
        let (aircraft_vb, aircraft_ib, aircraft_ic) = create_test_aircraft_mesh(&device);
        let aircraft_matrix = Mat4::from_rows([
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let aircraft_obj_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow iso aircraft object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&aircraft_matrix)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let aircraft_obj_bg = matrix_bind_group(
            &device,
            &object_layout,
            &aircraft_obj_buffer,
            "shadow iso aircraft object group",
        );

        // Shared bind groups.
        let camera_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow iso camera buffer"),
            contents: bytemuck::bytes_of(&CameraUniform::new(
                &light_vp,
                &light_vp.inverse().expect("light vp invertible"),
                light_eye,
            )),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let camera_bg = camera_bind_group(
            &device,
            &camera_layout,
            &camera_buffer,
            "shadow iso camera group",
        );
        let identity_obj_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow iso identity object buffer"),
            contents: bytemuck::bytes_of(&ObjectUniform::from_matrix(&Mat4::identity())),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let identity_obj_bg = matrix_bind_group(
            &device,
            &object_layout,
            &identity_obj_buffer,
            "shadow iso identity object group",
        );
        let shadow_matrix_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("shadow iso matrix buffer"),
            contents: bytemuck::bytes_of(&ShadowUniform::from_matrix(&light_vp)),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let shadow_pass_bg = create_shadow_pass_bind_group(
            &device,
            &shadow_pass_layout,
            &shadow_matrix_buffer,
            "shadow iso pass group",
        );

        // Depth-only target.
        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("shadow iso depth target"),
            size: wgpu::Extent3d {
                width: TEST_SIZE,
                height: TEST_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        // Render: vegetation shadow THEN aircraft shadow in the SAME pass.
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("shadow iso depth pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &depth_view,
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

            // 1. Vegetation shadow (instanced pipeline).
            pass.set_pipeline(&gpu_veg.shadow_pipeline);
            pass.set_bind_group(0, &camera_bg, &[]);
            pass.set_bind_group(1, &identity_obj_bg, &[]);
            pass.set_bind_group(2, &shadow_pass_bg, &[]);
            let ranges = world.batch_ranges();
            for group in 0..GROUP_COUNT {
                if group % LOD_COUNT > 1 {
                    continue;
                }
                let start = ranges[group * 2];
                let count = ranges[group * 2 + 1];
                if count == 0 {
                    continue;
                }
                let asset = group / LOD_COUNT;
                let lod = (group % LOD_COUNT) as u8;
                for part in [VegetationPart::Bark, VegetationPart::Foliage] {
                    let mesh = &gpu_veg.meshes[vegetation_mesh_index(asset, lod, part)];
                    let material = if matches!(part, VegetationPart::Foliage) {
                        &mats[1]
                    } else {
                        &mats[0]
                    };
                    pass.set_bind_group(3, &material.bind_group, &[]);
                    pass.set_vertex_buffer(0, mesh.vertex_buffer.slice(..));
                    pass.set_vertex_buffer(1, gpu_veg.instance_buffer.slice(..));
                    pass.set_index_buffer(mesh.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..mesh.index_count, 0, start..start + count);
                }
            }

            // 2. THE FIX: restore standard shadow pipeline before aircraft.
            pass.set_pipeline(&shadow_pipeline);

            // 3. Aircraft shadow (non-instanced, own object transform at x=+10).
            pass.set_bind_group(1, &aircraft_obj_bg, &[]);
            pass.set_vertex_buffer(0, aircraft_vb.slice(..));
            pass.set_index_buffer(aircraft_ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..aircraft_ic, 0, 0..1);
        }
        queue.submit([encoder.finish()]);

        // Readback depth and find the centroid of non-far-plane texels that
        // belong to the aircraft (right half of the light-space view).
        let depth_bytes =
            read_texture_level(&device, &queue, &depth_texture, 0, TEST_SIZE, TEST_SIZE, 4);

        // The aircraft is at x = +10 in world space; under the overhead light
        // camera its shadow silhouette projects RIGHT of centre. The vegetation
        // at x = −20 projects LEFT. Count near-depth texels in each half.
        let centre = TEST_SIZE / 2;
        let mut right_near = 0u64;
        let mut left_near = 0u64;
        for y in 0..TEST_SIZE {
            for x in 0..TEST_SIZE {
                let offset = ((y * TEST_SIZE + x) * 4) as usize;
                let depth = f32::from_le_bytes([
                    depth_bytes[offset],
                    depth_bytes[offset + 1],
                    depth_bytes[offset + 2],
                    depth_bytes[offset + 3],
                ]);
                if depth < 1.0 {
                    if x >= centre {
                        right_near += 1;
                    } else {
                        left_near += 1;
                    }
                }
            }
        }
        // The aircraft triangle (2 m across at x=+10, light from above) must
        // produce near-depth texels in the RIGHT half. If the vegetation shadow
        // pipeline leaked, the aircraft would be transformed to x=−20 and land
        // in the LEFT half instead.
        assert!(
            right_near > 0,
            "aircraft shadow must write depth in the right half \
             (right={right_near}, left={left_near}) — zero means the vegetation \
             shadow pipeline leaked into the aircraft caster draw"
        );
    }
}
