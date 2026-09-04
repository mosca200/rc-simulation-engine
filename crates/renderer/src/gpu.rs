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
//! The render pass is organized as:
//! 1. Sky pass (fullscreen triangle)
//! 2. Terrain batches (chunked, lit + fogged)
//! 3. Debug overlays (grid/axes, unlit)
//! 4. Aircraft batches (lit + fogged)
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

use crate::terrain::{DEFAULT_CHUNK_CELLS, TerrainMaterial, generate_centered_terrain_chunks};
use crate::texture::{SamplerConfig, TextureLoadError, create_staging_buffer};
use crate::{
    AircraftMesh, ChaseCamera, GlbAsset, Mat4, RenderFrame, Vertex, matrix_to_wgsl_columns,
    reference_grid_and_axes_at,
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

/// Default terrain extent for the RC flying field.
const DEFAULT_TERRAIN_EXTENT_M: f32 = 1000.0;
const DEFAULT_TERRAIN_CELL_SPACING_M: f32 = 5.0;

/// Presentation asset: either a full GLB with per-primitive materials,
/// or a procedural merged mesh (fallback).
pub enum PresentationAsset<'a> {
    Glb(&'a GlbAsset),
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

struct DepthTarget {
    _texture: wgpu::Texture,
    view: wgpu::TextureView,
}

/// G1C: Persistent GPU material resources.
struct GpuMaterial {
    _texture: wgpu::Texture,
    _texture_view: wgpu::TextureView,
    _sampler: wgpu::Sampler,
    bind_group: wgpu::BindGroup,
}

/// G1C: A render batch with its own vertex/index buffers and material.
struct RenderBatch {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    material_index: usize,
}

/// G1C: Terrain chunk GPU resources.
struct GpuTerrainChunk {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    _bounds: ([f32; 3], [f32; 3]),
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

    _camera_bind_group_layout: wgpu::BindGroupLayout,
    _object_bind_group_layout: wgpu::BindGroupLayout,
    _environment_bind_group_layout: wgpu::BindGroupLayout,
    _material_bind_group_layout: wgpu::BindGroupLayout,

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
    environment_bind_group: wgpu::BindGroup,

    // G1C: Material system.
    materials: Vec<GpuMaterial>,
    _fallback_material_index: usize,

    // G1C: Aircraft batches (one per primitive).
    aircraft_batches: Vec<RenderBatch>,

    // G1C: Terrain chunks.
    terrain_chunks: Vec<GpuTerrainChunk>,
    terrain_material_index: usize,

    // Debug overlays.
    line_vertex_buffer: wgpu::Buffer,
    line_vertex_count: u32,

    depth_target: DepthTarget,
    camera: ChaseCamera,
    asynchronous_gpu_error: Arc<AtomicU8>,

    show_debug_overlays: bool,
}

impl WgpuRenderer {
    /// Create a renderer with a presentation asset (GLB or procedural).
    ///
    /// This is the primary constructor. The GLB path exercises the full
    /// G1C textured multi-primitive pipeline.
    pub async fn new_with_presentation(
        window: Arc<Window>,
        asset: PresentationAsset<'_>,
        ground_below_render_origin_m: f32,
        terrain_mode: RenderTerrainMode,
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
        let material_bind_group_layout = material_bind_group_layout(&device, "material layout");

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

        // White fallback material.
        let fallback_material =
            create_white_fallback_material(&device, &material_bind_group_layout, &queue);
        let mut materials = vec![fallback_material];
        let fallback_material_index = 0;

        // Upload aircraft batches from the presentation asset.
        let mut aircraft_batches = Vec::new();
        match asset {
            PresentationAsset::Glb(glb_asset) => {
                for primitive in &glb_asset.primitives {
                    let material_index =
                        if let Some(texture) = &primitive.material.base_color_texture {
                            // Upload texture material.
                            let gpu_material = create_gpu_material(
                                &device,
                                &material_bind_group_layout,
                                &queue,
                                texture,
                                &primitive.material.sampler_config,
                            )?;
                            let index = materials.len();
                            materials.push(gpu_material);
                            index
                        } else {
                            fallback_material_index
                        };

                    if !primitive.vertices.is_empty() {
                        let vertex_buffer =
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("aircraft primitive vertices"),
                                contents: bytemuck::cast_slice(&primitive.vertices),
                                usage: wgpu::BufferUsages::VERTEX,
                            });
                        let index_buffer =
                            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                                label: Some("aircraft primitive indices"),
                                contents: bytemuck::cast_slice(&primitive.indices),
                                usage: wgpu::BufferUsages::INDEX,
                            });
                        aircraft_batches.push(RenderBatch {
                            vertex_buffer,
                            index_buffer,
                            index_count: primitive.indices.len() as u32,
                            material_index,
                        });
                    }
                }
            }
            PresentationAsset::Procedural(mesh) => {
                if !mesh.vertices().is_empty() {
                    let vertex_buffer =
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some("aircraft presentation vertices"),
                            contents: bytemuck::cast_slice(mesh.vertices()),
                            usage: wgpu::BufferUsages::VERTEX,
                        });
                    let index_buffer =
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
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
            }
        }

        // FIX 3: Generate centered terrain (extends in both +X/-X and +Z/-Z).
        let terrain_height_field = match terrain_mode {
            RenderTerrainMode::Rolling => crate::terrain::generate_rolling_terrain(
                (DEFAULT_TERRAIN_EXTENT_M / DEFAULT_TERRAIN_CELL_SPACING_M) as u32,
                (DEFAULT_TERRAIN_EXTENT_M / DEFAULT_TERRAIN_CELL_SPACING_M) as u32,
                DEFAULT_TERRAIN_CELL_SPACING_M,
                -ground_below_render_origin_m,
                3.0,
            ),
            RenderTerrainMode::Flat => crate::terrain::generate_flat_terrain(
                (DEFAULT_TERRAIN_EXTENT_M / DEFAULT_TERRAIN_CELL_SPACING_M) as u32,
                (DEFAULT_TERRAIN_EXTENT_M / DEFAULT_TERRAIN_CELL_SPACING_M) as u32,
                DEFAULT_TERRAIN_CELL_SPACING_M,
                -ground_below_render_origin_m,
            ),
        };
        let terrain_material = TerrainMaterial::default();
        // FIX 3: Use centered chunk generation.
        let terrain_chunk_data = generate_centered_terrain_chunks(
            &terrain_height_field,
            DEFAULT_CHUNK_CELLS,
            &terrain_material,
        );

        // FIX 4: Terrain uses the white fallback material.
        // The terrain base_color_factor is baked into vertex colors, so
        // vertex_color * white_texture = vertex_color = terrain color.
        let terrain_material_index = fallback_material_index;

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
        let environment_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("environment uniform"),
            contents: bytemuck::bytes_of(&default_environment),
            usage: wgpu::BufferUsages::UNIFORM,
        });

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
            "environment bind group",
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
            _camera_bind_group_layout: camera_bind_group_layout,
            _object_bind_group_layout: object_bind_group_layout,
            _environment_bind_group_layout: environment_bind_group_layout,
            _material_bind_group_layout: material_bind_group_layout,
            camera_buffer,
            camera_bind_group,
            aircraft_object_buffer,
            aircraft_object_bind_group,
            _identity_object_buffer: identity_object_buffer,
            identity_object_bind_group,
            _environment_buffer: environment_buffer,
            environment_bind_group,
            materials,
            _fallback_material_index: fallback_material_index,
            aircraft_batches,
            terrain_chunks,
            terrain_material_index,
            line_vertex_buffer,
            line_vertex_count: references.vertices().len() as u32,
            depth_target,
            camera: ChaseCamera::new(size.width, size.height),
            asynchronous_gpu_error,
            show_debug_overlays: true,
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

        let surface_view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("G1C frame encoder"),
            });
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
            // Sky uses identity object (group 1) — sky is at infinity.
            render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
            render_pass.draw(0..3, 0..1);

            // --- Scene geometry ---
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);

            // Terrain chunks: identity object transform (world-local).
            render_pass.set_bind_group(1, &self.identity_object_bind_group, &[]);
            let terrain_material = &self.materials[self.terrain_material_index];
            render_pass.set_bind_group(3, &terrain_material.bind_group, &[]);
            for chunk in &self.terrain_chunks {
                render_pass.set_vertex_buffer(0, chunk.vertex_buffer.slice(..));
                render_pass
                    .set_index_buffer(chunk.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                render_pass.draw_indexed(0..chunk.index_count, 0, 0..1);
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
        ],
    });

    GpuMaterial {
        _texture: texture,
        _texture_view: texture_view,
        _sampler: sampler,
        bind_group,
    }
}

fn create_gpu_material(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    queue: &wgpu::Queue,
    texture_data: &crate::texture::DecodedTexture,
    sampler_config: &SamplerConfig,
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
        ],
    });

    Ok(GpuMaterial {
        _texture: texture,
        _texture_view: texture_view,
        _sampler: sampler,
        bind_group,
    })
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
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<EnvironmentUniform>() as u64),
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
