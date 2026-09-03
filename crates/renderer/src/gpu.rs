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

#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct MatrixUniform {
    columns: [[f32; 4]; 4],
}

impl MatrixUniform {
    fn from_matrix(matrix: &Mat4) -> Self {
        Self {
            columns: matrix_to_wgsl_columns(matrix),
        }
    }
}

/// GPU light uniform matching the WGSL `LightUniform` struct.
///
/// `direction.xyz` is the normalized world-space direction TOWARD the light.
/// `direction.w` is the directional light intensity.
/// `ambient.xyz` is the ambient light color.
/// `ambient.w` is reserved/padding.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct LightUniform {
    direction: [f32; 4],
    ambient: [f32; 4],
}

impl LightUniform {
    fn default_lit() -> Self {
        let dir = DEFAULT_LIGHT_DIRECTION;
        let length = (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]).sqrt();
        let normalized = if length > f32::EPSILON {
            [dir[0] / length, dir[1] / length, dir[2] / length]
        } else {
            [0.0, 1.0, 0.0]
        };
        Self {
            direction: [
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
    camera_bind_group: wgpu::BindGroup,
    aircraft_object_bind_group: wgpu::BindGroup,
    reference_object_bind_group: wgpu::BindGroup,
    light_bind_group: wgpu::BindGroup,
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
                label: Some("RC Simulation Engine G1A device"),
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
            label: Some("G1A material+lighting shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let camera_bind_group_layout = matrix_bind_group_layout(&device, "camera layout");
        let object_bind_group_layout = matrix_bind_group_layout(&device, "object layout");
        let light_bind_group_layout = light_bind_group_layout(&device, "light layout");

        let lit_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("G1A lit pipeline layout"),
            bind_group_layouts: &[
                Some(&camera_bind_group_layout),
                Some(&object_bind_group_layout),
                Some(&light_bind_group_layout),
            ],
            immediate_size: 0,
        });
        let unlit_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("G1A unlit pipeline layout"),
                bind_group_layouts: &[
                    Some(&camera_bind_group_layout),
                    Some(&object_bind_group_layout),
                ],
                immediate_size: 0,
            });

        let triangle_pipeline = create_pipeline(
            &device,
            &shader,
            &lit_pipeline_layout,
            format,
            PipelineSpec {
                topology: wgpu::PrimitiveTopology::TriangleList,
                cull_mode: Some(wgpu::Face::Back),
                depth_write_enabled: true,
                label: "G1A lit triangle pipeline",
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
                label: "G1A unlit line pipeline",
                fragment_entry_point: "fs_unlit",
            },
        );

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

        let identity_uniform = MatrixUniform::from_matrix(&Mat4::identity());
        let camera_buffer = matrix_buffer(&device, "camera uniform", &identity_uniform, true);
        let aircraft_object_buffer =
            matrix_buffer(&device, "aircraft object uniform", &identity_uniform, true);
        let reference_object_buffer = matrix_buffer(
            &device,
            "reference object uniform",
            &identity_uniform,
            false,
        );
        let default_light = LightUniform::default_lit();
        let light_buffer = light_buffer(&device, "light uniform", &default_light);

        let camera_bind_group = matrix_bind_group(
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
        let light_bind_group = create_light_bind_group(
            &device,
            &light_bind_group_layout,
            &light_buffer,
            "light bind group",
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
            camera_bind_group,
            aircraft_object_bind_group,
            reference_object_bind_group,
            light_bind_group,
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

        let camera_uniform =
            MatrixUniform::from_matrix(&self.camera.view_projection(frame.aircraft_pose()));
        let object_uniform = MatrixUniform::from_matrix(&frame.aircraft_pose().model_matrix());
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
                label: Some("G1A frame encoder"),
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
                label: Some("G1A scene pass"),
                color_attachments: &[Some(color_attachment)],
                depth_stencil_attachment: Some(depth_attachment),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);

            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(1, &self.reference_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.light_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.ground_vertex_buffer.slice(..));
            render_pass.set_index_buffer(
                self.ground_index_buffer.slice(..),
                wgpu::IndexFormat::Uint32,
            );
            render_pass.draw_indexed(0..self.ground_index_count, 0, 0..1);

            render_pass.set_pipeline(&self.line_pipeline);
            render_pass.set_bind_group(1, &self.reference_object_bind_group, &[]);
            render_pass.set_vertex_buffer(0, self.line_vertex_buffer.slice(..));
            render_pass.draw(0..self.line_vertex_count, 0..1);

            render_pass.set_pipeline(&self.triangle_pipeline);
            render_pass.set_bind_group(1, &self.aircraft_object_bind_group, &[]);
            render_pass.set_bind_group(2, &self.light_bind_group, &[]);
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

fn light_bind_group_layout(device: &wgpu::Device, label: &str) -> wgpu::BindGroupLayout {
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

fn matrix_buffer(
    device: &wgpu::Device,
    label: &str,
    value: &MatrixUniform,
    copy_destination: bool,
) -> wgpu::Buffer {
    let mut usage = wgpu::BufferUsages::UNIFORM;
    if copy_destination {
        usage |= wgpu::BufferUsages::COPY_DST;
    }
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytemuck::bytes_of(value),
        usage,
    })
}

fn light_buffer(device: &wgpu::Device, label: &str, value: &LightUniform) -> wgpu::Buffer {
    device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(label),
        contents: bytemuck::bytes_of(value),
        usage: wgpu::BufferUsages::UNIFORM,
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

fn create_light_bind_group(
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
        2 => Float32x3,
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

fn create_depth_target(device: &wgpu::Device, width: u32, height: u32) -> DepthTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G1A depth texture"),
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
