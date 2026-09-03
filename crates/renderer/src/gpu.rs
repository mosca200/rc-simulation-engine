use crate::{
    AircraftMesh, ChaseCamera, Mat4, RenderFrame, Vertex, ground_plane_at, matrix_to_wgsl_columns,
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

/// Deterministic directional light defaults.
///
/// Light direction: from above (+Y), slightly right (+X), slightly forward (-Z).
/// Intensity: 0.80 — leaves headroom for the ambient term.
/// Ambient: 0.30 — keeps shadowed faces visible without washing out the lit side.
const DEFAULT_LIGHT_DIRECTION: [f32; 3] = [0.4, 0.8, -0.3];
const DEFAULT_LIGHT_INTENSITY: f32 = 0.80;
const DEFAULT_AMBIENT_RGB: [f32; 3] = [0.30, 0.30, 0.30];

// G1B sky/atmosphere defaults.
const DEFAULT_ZENITH_RGB: [f32; 3] = [0.16, 0.36, 0.66];
const DEFAULT_HORIZON_RGB: [f32; 3] = [0.68, 0.78, 0.88];
const DEFAULT_GROUND_ATM_RGB: [f32; 3] = [0.38, 0.44, 0.40];
const DEFAULT_HAZE_STRENGTH: f32 = 0.55;
const DEFAULT_FOG_DENSITY: f32 = 0.0015;
const DEFAULT_SUN_COLOR_RGB: [f32; 3] = [1.0, 0.95, 0.85];
/// Cosine of ~0.5° angular radius (small sun disk).
const DEFAULT_SUN_COS_ANGULAR_RADIUS: f32 = 0.999_96;

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
///
/// G1B: extended with inverse view-projection and camera world position
/// for sky view-direction reconstruction and distance fog.
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
///
/// Single source of truth for lighting and atmosphere parameters.
/// Replaces the G1A `LightUniform` — the light direction drives both
/// aircraft Lambert lighting and the sky sun disk.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct EnvironmentUniform {
    /// xyz = normalized direction TOWARD the light, w = directional intensity.
    light_direction: [f32; 4],
    /// xyz = ambient light color, w = reserved.
    ambient: [f32; 4],
    /// xyz = zenith sky color, w = reserved.
    sky_zenith: [f32; 4],
    /// xyz = horizon/haze color, w = haze strength.
    sky_horizon: [f32; 4],
    /// xyz = below-horizon atmospheric color, w = fog density.
    sky_ground: [f32; 4],
    /// xyz = sun disk color, w = cosine of sun angular radius.
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

    /// All values are finite by construction from fixed constants,
    /// but this is verified in tests.
    #[cfg(test)]
    fn is_finite(&self) -> bool {
        [
            self.light_direction,
            self.ambient,
            self.sky_zenith,
            self.sky_horizon,
            self.sky_ground,
            self.sun_color,
        ]
        .iter()
        .all(|vec| vec.iter().all(|component| component.is_finite()))
    }
}

/// Object uniform — unchanged from G1A.
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

/// Minimal depth-tested wgpu renderer. It owns no simulation-domain values.
pub struct WgpuRenderer {
    _instance: wgpu::Instance,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface_configuration: wgpu::SurfaceConfiguration,
    surface_is_configured: bool,
    // G1B: sky pipeline (fullscreen triangle, no vertex buffer).
    sky_pipeline: wgpu::RenderPipeline,
    triangle_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    aircraft_vertex_buffer: wgpu::Buffer,
    aircraft_index_buffer: wgpu::Buffer,
    aircraft_index_count: u32,
    ground_vertex_buffer: wgpu::Buffer,
    ground_index_buffer: wgpu::Buffer,
    ground_index_count: u32,
    line_vertex_buffer: wgpu::Buffer,
    line_vertex_count: u32,
    camera_buffer: wgpu::Buffer,
    aircraft_object_buffer: wgpu::Buffer,
    _environment_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    aircraft_object_bind_group: wgpu::BindGroup,
    reference_object_bind_group: wgpu::BindGroup,
    environment_bind_group: wgpu::BindGroup,
    depth_target: DepthTarget,
    camera: ChaseCamera,
    asynchronous_gpu_error: Arc<AtomicU8>,
}

impl WgpuRenderer {
    pub async fn new(
        window: Arc<Window>,
        aircraft: &AircraftMesh,
        ground_below_render_origin_m: f32,
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
                label: Some("G1B device"),
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
            label: Some("G1B sky+material+lighting+atmosphere shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        // Bind group layouts.
        let camera_bind_group_layout = camera_bind_group_layout(&device, "camera layout");
        let object_bind_group_layout = matrix_bind_group_layout(&device, "object layout");
        let environment_bind_group_layout =
            environment_bind_group_layout(&device, "environment layout");

        // Pipeline layouts.
        // All pipelines share the same group numbering:
        //   group 0 = camera, group 1 = object, group 2 = environment.
        // The sky pipeline does not read group 1 but the layout slot is present.
        let sky_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G1B sky pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout),
                Some(&object_bind_group_layout),
                Some(&environment_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let lit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G1B lit pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout),
                Some(&object_bind_group_layout),
                Some(&environment_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let unlit_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G1B unlit pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&object_bind_group_layout),
                ],
                immediate_size: 0,
            });

        // Pipelines.
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
                label: "G1B lit triangle pipeline",
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
                label: "G1B unlit line pipeline",
                fragment_entry_point: "fs_unlit",
            },
        );

        // Vertex/index buffers.
        let aircraft_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("aircraft presentation vertices"),
            contents: bytemuck::cast_slice(aircraft.vertices()),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let aircraft_index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("aircraft presentation indices"),
            contents: bytemuck::cast_slice(aircraft.indices()),
            usage: wgpu::BufferUsages::INDEX,
        });
        let ground = ground_plane_at(-ground_below_render_origin_m - 0.04);
        let ground_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("render-only ground vertices"),
            contents: bytemuck::cast_slice(ground.vertices()),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ground_index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("render-only ground indices"),
            contents: bytemuck::cast_slice(ground.indices()),
            usage: wgpu::BufferUsages::INDEX,
        });
        let references = reference_grid_and_axes_at(-ground_below_render_origin_m);
        let line_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("reference grid and axes vertices"),
            contents: bytemuck::cast_slice(references.vertices()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Uniform buffers.
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
        let aircraft_object_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("aircraft object uniform"),
            contents: bytemuck::bytes_of(&identity_object),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });
        let reference_object_buffer =
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("reference object uniform"),
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
        let reference_object_bind_group = matrix_bind_group(
            &device,
            &object_bind_group_layout,
            &reference_object_buffer,
            "reference object bind group",
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
            aircraft_vertex_buffer,
            aircraft_index_buffer,
            aircraft_index_count: aircraft.indices().len() as u32,
            ground_vertex_buffer,
            ground_index_buffer,
            ground_index_count: ground.indices().len() as u32,
            line_vertex_buffer,
            line_vertex_count: references.vertices().len() as u32,
            camera_buffer,
            aircraft_object_buffer,
            _environment_buffer: environment_buffer,
            camera_bind_group,
            aircraft_object_bind_group,
            reference_object_bind_group,
            environment_bind_group,
            depth_target,
            camera: ChaseCamera::new(size.width, size.height),
            asynchronous_gpu_error,
        })
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

        // Compute camera uniforms on the stack — no heap allocation.
        let aircraft_pose = frame.aircraft_pose();
        let vp = self.camera.view_projection(aircraft_pose);
        let eye = self.camera.eye_position(aircraft_pose);
        let identity = Mat4::identity();
        let inv_vp = self
            .camera
            .inv_view_projection(aircraft_pose)
            .unwrap_or(identity);
        let camera_uniform = CameraUniform::new(&vp, &inv_vp, eye);
        let object_uniform = ObjectUniform::from_matrix(&aircraft_pose.model_matrix());

        self.queue
            .write_buffer(&self.camera_buffer, 0, bytemuck::bytes_of(&camera_uniform));
        self.queue.write_buffer(
            &self.aircraft_object_buffer,
            0,
            bytemuck::bytes_of(&object_uniform),
        );

        let surface_view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("G1B frame encoder"),
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
                label: Some("G1B scene pass"),
                color_attachments: &[Some(color_attachment)],
                depth_stencil_attachment: Some(depth_attachment),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });

            // --- Sky pass (background) ---
            // Fullscreen triangle at far-plane depth.
            // depth_write=false, depth_compare=always → does not affect scene depth.
            render_pass.set_pipeline(&self.sky_pipeline);
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            // Group 1 slot exists in the layout but the sky shader does not read it.
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
            render_pass.draw(0..3, 0..1);

            // --- Scene geometry ---
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);

            // Ground plane (lit + fog).
            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(1, &self.reference_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.ground_vertex_buffer.slice(..));
            render_pass.set_index_buffer(
                self.ground_index_buffer.slice(..),
                wgpu::IndexFormat::Uint32,
            );
            render_pass.draw_indexed(0..self.ground_index_count, 0, 0..1);

            // Debug grid/axes (unlit, unfogged — diagnostic overlay).
            render_pass.set_pipeline(&self.line_pipeline);
            render_pass.set_bind_group(1, &self.reference_object_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.line_vertex_buffer.slice(..));
            render_pass.draw(0..self.line_vertex_count, 0..1);

            // Aircraft (lit + fog).
            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(1, &self.aircraft_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.environment_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.aircraft_vertex_buffer.slice(..));
            render_pass.set_index_buffer(
                self.aircraft_index_buffer.slice(..),
                wgpu::IndexFormat::Uint32,
            );
            render_pass.draw_indexed(0..self.aircraft_index_count, 0, 0..1);
        }
        self.queue.submit([encoder.finish()]);
        self.queue.present(surface_texture);
        if reconfigure_after_present {
            self.reconfigure_surface();
        }
        self.check_asynchronous_gpu_error()
    }

    fn check_asynchronous_gpu_error(&self) -> Result<(), SurfaceError> {
        match self
            .asynchronous_gpu_error
            .swap(GPU_ERROR_NONE, Ordering::AcqRel)
        {
            GPU_ERROR_NONE => Ok(()),
            GPU_ERROR_OUT_OF_MEMORY => Err(SurfaceError::OutOfMemory),
            _ => Err(SurfaceError::Validation),
        }
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
                min_binding_size: None,
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
                min_binding_size: None,
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
                min_binding_size: None,
            },
            count: None,
        }],
    })
}

// ---------------------------------------------------------------------------
// Bind group creation helpers
// ---------------------------------------------------------------------------

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
// Pipeline creation
// ---------------------------------------------------------------------------

struct PipelineSpec<'a> {
    topology: wgpu::PrimitiveTopology,
    cull_mode: Option<wgpu::Face>,
    depth_write_enabled: bool,
    label: &'a str,
    fragment_entry_point: &'a str,
}

fn create_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    surface_format: wgpu::TextureFormat,
    specification: PipelineSpec<'_>,
) -> wgpu::RenderPipeline {
    const ATTRIBUTES: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![
        0 => Float32x3,
        1 => Float32x3,
        2 => Float32x4,
        3 => Float32x2
    ];
    let vertex_layout = wgpu::VertexBufferLayout {
        array_stride: size_of::<Vertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRIBUTES,
    };
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(specification.label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[Some(vertex_layout)],
        },
        primitive: wgpu::PrimitiveState {
            topology: specification.topology,
            strip_index_format: None,
            front_face: wgpu::FrontFace::Ccw,
            cull_mode: specification.cull_mode,
            unclipped_depth: false,
            polygon_mode: wgpu::PolygonMode::Fill,
            conservative: false,
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: DEPTH_FORMAT,
            depth_write_enabled: Some(specification.depth_write_enabled),
            depth_compare: Some(if specification.depth_write_enabled {
                wgpu::CompareFunction::Less
            } else {
                wgpu::CompareFunction::LessEqual
            }),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(specification.fragment_entry_point),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: surface_format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// Sky pipeline: fullscreen triangle, no vertex buffer, no depth write.
///
/// The sky is rendered as background before scene geometry.
/// depth_compare = Always ensures the sky always draws.
/// depth_write = false ensures it does not affect the depth buffer.
fn create_sky_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    surface_format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("G1B sky fullscreen pipeline"),
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
                format: surface_format,
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
        label: Some("G1B depth texture"),
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_environment_uniform_contains_only_finite_values() {
        let env = EnvironmentUniform::default_environment();
        assert!(env.is_finite(), "environment uniform has non-finite values");
    }

    #[test]
    fn default_light_direction_is_normalized() {
        let env = EnvironmentUniform::default_environment();
        let dir = env.light_direction;
        let length = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        assert!(
            (length - 1.0).abs() < 1.0e-5,
            "light direction not normalized: length = {length}"
        );
    }

    #[test]
    fn default_light_direction_matches_g1a() {
        let env = EnvironmentUniform::default_environment();
        let dir = env.light_direction;
        let raw = DEFAULT_LIGHT_DIRECTION;
        let raw_len = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]).sqrt();
        let expected = [raw[0] / raw_len, raw[1] / raw_len, raw[2] / raw_len];
        for (actual, &expected) in dir[..3].iter().zip(expected.iter()) {
            assert!((actual - expected).abs() < 1.0e-5);
        }
        assert!((dir[3] - DEFAULT_LIGHT_INTENSITY).abs() < 1.0e-6);
    }

    #[test]
    fn environment_uniform_layout_matches_wgsl() {
        // 6 vec4s = 96 bytes.
        assert_eq!(size_of::<EnvironmentUniform>(), 96);
        assert_eq!(size_of::<EnvironmentUniform>() % 16, 0);
    }

    #[test]
    fn camera_uniform_layout_matches_wgsl() {
        // 2 mat4x4 + 1 vec4 = 64 + 64 + 16 = 144 bytes.
        assert_eq!(size_of::<CameraUniform>(), 144);
        assert_eq!(size_of::<CameraUniform>() % 16, 0);
    }

    #[test]
    fn camera_uniform_is_finite_for_identity() {
        let cu = CameraUniform::new(&Mat4::identity(), &Mat4::identity(), [0.0; 3]);
        assert!(cu.view_projection.iter().flatten().all(|v| v.is_finite()));
        assert!(
            cu.inv_view_projection
                .iter()
                .flatten()
                .all(|v| v.is_finite())
        );
        assert!(cu.camera_position.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn haze_strength_is_bounded() {
        let env = EnvironmentUniform::default_environment();
        let haze = env.sky_horizon[3];
        assert!(
            (0.0..=1.0).contains(&haze),
            "haze strength out of [0,1]: {haze}"
        );
    }

    #[test]
    fn fog_density_is_non_negative() {
        let env = EnvironmentUniform::default_environment();
        let density = env.sky_ground[3];
        assert!(
            density >= 0.0,
            "fog density must be non-negative: {density}"
        );
    }

    #[test]
    fn sun_cos_angular_radius_is_near_one() {
        let env = EnvironmentUniform::default_environment();
        let cos_r = env.sun_color[3];
        assert!(
            cos_r > 0.99 && cos_r <= 1.0,
            "sun cos radius unexpected: {cos_r}"
        );
    }
}
