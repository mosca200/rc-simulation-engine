//! G3B: HDR outdoor lighting pipeline — discriminating tests.
//!
//! Groups:
//!   1. exposure EV mapping / validation (pure Rust mirror of the WGSL math);
//!   2. Khronos PBR Neutral tone mapper mirror: finite, monotonic, grey-
//!      preserving, highlight compression;
//!   3. structural guards pinning the renderer/shader integration points
//!      (HDR format, pass topology, resources outside the frame loop, resize
//!      recreation, shadow-on-direct-only, terrain debug path);
//!   4. WGSL parse/validation of the postprocess entry points;
//!   5. one headless GPU test (`#[ignore]`) proving the real HDR -> exposure
//!      -> tone map -> sRGB chain on a device (never runs in CI).
//!
//! Physics/determinism fingerprints are untouched by design: the renderer
//! crate already forbids dependencies on the simulation-domain crates (see
//! `dependency_boundary.rs`), and exposure is presentation-only.

use renderer::{DEFAULT_EXPOSURE_EV, ExposureError, exposure_multiplier, validate_exposure_ev};
use wgpu::util::DeviceExt;

// ---------------------------------------------------------------------------
// Exact Rust mirror of the WGSL `khronos_pbr_neutral` reference math.
// ---------------------------------------------------------------------------

/// Khronos PBR Neutral tone mapper (reference math from the Khronos
/// glTF-Sample-Renderer `tonemapping.glsl`) on one linear HDR value's peak
/// channel. Operates channel-wise through the shared peak/desaturation math.
fn khronos_pbr_neutral_rgb(color: [f32; 3]) -> [f32; 3] {
    const START_COMPRESSION: f32 = 0.8 - 0.04;
    const DESATURATION: f32 = 0.15;

    let x = color[0].min(color[1]).min(color[2]);
    let offset = if x < 0.08 { x - 6.25 * x * x } else { 0.04 };
    let c = [color[0] - offset, color[1] - offset, color[2] - offset];

    let peak = c[0].max(c[1]).max(c[2]);
    if peak < START_COMPRESSION {
        return c;
    }
    let d = 1.0 - START_COMPRESSION;
    let new_peak = 1.0 - d * d / (peak + d - START_COMPRESSION);
    let scaled = [
        c[0] * (new_peak / peak),
        c[1] * (new_peak / peak),
        c[2] * (new_peak / peak),
    ];
    let g = 1.0 - 1.0 / (DESATURATION * (peak - new_peak) + 1.0);
    [
        scaled[0] + (new_peak - scaled[0]) * g,
        scaled[1] + (new_peak - scaled[1]) * g,
        scaled[2] + (new_peak - scaled[2]) * g,
    ]
}

fn all_finite(color: [f32; 3]) -> bool {
    color[0].is_finite() && color[1].is_finite() && color[2].is_finite()
}

#[test]
fn exposure_multiplier_maps_ev_stops() {
    assert_eq!(exposure_multiplier(0.0), 1.0);
    assert_eq!(exposure_multiplier(1.0), 2.0);
    assert_eq!(exposure_multiplier(-1.0), 0.5);
    assert!((exposure_multiplier(0.5) - 2.0f32.sqrt()).abs() < 1e-6);
}

#[test]
fn validate_exposure_ev_accepts_default_and_band_edges() {
    assert_eq!(validate_exposure_ev(DEFAULT_EXPOSURE_EV), Ok(0.0));
    assert_eq!(validate_exposure_ev(8.0), Ok(8.0));
    assert_eq!(validate_exposure_ev(-8.0), Ok(-8.0));
}

#[test]
fn validate_exposure_ev_rejects_non_finite() {
    assert!(matches!(
        validate_exposure_ev(f32::NAN),
        Err(ExposureError::NotFinite(v)) if v.is_nan()
    ));
    assert!(matches!(
        validate_exposure_ev(f32::INFINITY),
        Err(ExposureError::NotFinite(_))
    ));
    assert!(matches!(
        validate_exposure_ev(f32::NEG_INFINITY),
        Err(ExposureError::NotFinite(_))
    ));
}

#[test]
fn validate_exposure_ev_rejects_out_of_range() {
    assert_eq!(
        validate_exposure_ev(8.5),
        Err(ExposureError::OutOfRange(8.5))
    );
    assert_eq!(
        validate_exposure_ev(-8.5),
        Err(ExposureError::OutOfRange(-8.5))
    );
}

#[test]
fn neutral_tone_map_is_finite_across_scene_range() {
    for r in [-1.0, -0.1, 0.0, 0.04, 0.5, 0.8, 1.0, 2.0, 4.0, 100.0, 1e6] {
        for g in [0.0, 0.5, 1.0, 4.0] {
            for b in [0.0, 0.25, 0.9, 3.0] {
                let mapped = khronos_pbr_neutral_rgb([r, g, b]);
                assert!(
                    all_finite(mapped),
                    "tone map must stay finite for ({r}, {g}, {b}), got {mapped:?}"
                );
                for channel in mapped {
                    assert!(
                        channel <= 1.0 + 1e-4,
                        "tone map output must stay display-bounded, got {mapped:?}"
                    );
                }
            }
        }
    }
}

#[test]
fn neutral_tone_map_is_monotonic_per_channel() {
    // Domain: scene-referred HDR (>= 0). Negative inputs shift the reference
    // offset (designed for non-negative scene values) and are out of scope.
    let samples = [0.0, 0.04, 0.1, 0.3, 0.76, 1.0, 2.0, 4.0, 10.0];
    for &channel in &[0usize, 1, 2] {
        let mut previous = f32::NEG_INFINITY;
        for &value in &samples {
            let mut input = [0.3; 3];
            input[channel] = value;
            let mapped = khronos_pbr_neutral_rgb(input)[channel];
            assert!(
                mapped >= previous - 1e-5,
                "tone map must be monotone on channel {channel} at {value}: {mapped} < {previous}"
            );
            previous = mapped;
        }
    }
}

#[test]
fn neutral_tone_map_preserves_greys_and_compresses_highlights() {
    for grey in [0.0, 0.1, 0.5, 1.0, 2.0, 5.0] {
        let mapped = khronos_pbr_neutral_rgb([grey, grey, grey]);
        let spread = (mapped[0] - mapped[1])
            .abs()
            .max((mapped[1] - mapped[2]).abs());
        assert!(
            spread < 1e-4,
            "neutral input must stay neutral, got {mapped:?} (spread {spread})"
        );
    }
    // Highlight compression: >1 scene values map below 1 without hard clip.
    let bright = khronos_pbr_neutral_rgb([4.0, 4.0, 4.0]);
    assert!(
        bright[0] < 1.0,
        "highlights must be compressed, got {bright:?}"
    );
    assert!(
        bright[0] > 0.9,
        "highlights must not collapse, got {bright:?}"
    );
    // Midgrey stays mostly untouched (< startCompression: no remap).
    let mid = khronos_pbr_neutral_rgb([0.5, 0.5, 0.5]);
    assert!(
        (mid[0] - 0.46).abs() < 1e-3,
        "midgrey shift must match the reference offset, got {mid:?}"
    );
}

// ---------------------------------------------------------------------------
// Structural guards (string level): the G3B integration points that must not
// drift in later slices.
// ---------------------------------------------------------------------------

#[test]
fn shader_has_hdr_postprocess_contract_without_ldr_clamp() {
    // Normalize line endings: the source may be checked out with CRLF.
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    assert!(
        source.contains("fn khronos_pbr_neutral("),
        "the default tone mapper must be the Khronos PBR Neutral reference"
    );
    assert!(
        source.contains("exp2(postprocess.exposure_ev)"),
        "exposure must apply as exp2(EV) before tone mapping"
    );
    assert!(
        source.contains("@fragment\nfn fs_postprocess("),
        "the postprocess fragment entry must exist"
    );
    // The scene-lit output path must NOT clamp to [0,1]: the sky no longer
    // clamps and the postprocess owns the display range.
    assert!(
        !source.contains("clamp(sky + environment.sun_color.xyz"),
        "the sky must not be LDR-clamped before the postprocess"
    );
    // The scene pass renders to the HDR target; the postprocess samples it.
    assert!(
        source.contains("var hdr_scene_texture: texture_2d<f32>;"),
        "the HDR scene texture binding must exist"
    );
}

#[test]
fn renderer_hdr_target_is_rgba16float_and_created_outside_frame_loop() {
    let source = include_str!("../src/gpu.rs");
    assert!(
        source
            .contains("const HDR_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;"),
        "the HDR scene target format must be Rgba16Float"
    );
    let (initialization, after_render) = source
        .split_once("pub fn render(&mut self, frame: &RenderFrame)")
        .expect("renderer must expose the frame path");
    let (frame_path, _) = after_render
        .split_once("fn check_asynchronous_gpu_error")
        .expect("frame path must be delimited");
    assert!(
        initialization.contains("let hdr_target = create_hdr_target("),
        "the HDR target must be created at startup"
    );
    assert!(
        initialization.contains("let postprocess_pipeline ="),
        "the postprocess pipeline must be created at startup"
    );
    assert!(
        initialization.contains("let postprocess_bind_group = create_hdr_scene_bind_group("),
        "the postprocess bind group must be created at startup"
    );
    for forbidden in [
        "create_hdr_target(",
        "create_postprocess_pipeline(",
        "create_hdr_scene_bind_group(",
        "create_texture(",
        "create_bind_group(",
        "create_render_pipeline(",
        "create_sampler(",
        "create_buffer(",
    ] {
        assert!(
            !frame_path.contains(forbidden),
            "frame path must not create {forbidden}"
        );
    }
    assert!(
        frame_path.contains("&self.postprocess_bind_group"),
        "the frame path must use the persistent postprocess bind group"
    );
    assert!(
        frame_path.contains("&self.postprocess_uniform_buffer"),
        "the frame path must update the persistent postprocess uniform"
    );
}

#[test]
fn resize_recreates_hdr_target_and_bind_group_but_not_pipeline() {
    let source = include_str!("../src/gpu.rs");
    let (_, after_resize) = source
        .split_once("pub fn resize(&mut self, width: u32, height: u32)")
        .expect("resize must exist");
    let (resize_path, _) = after_resize
        .split_once("pub fn reconfigure_surface")
        .expect("resize body must be delimited");
    assert!(
        resize_path.contains("create_hdr_target("),
        "resize must recreate the HDR scene target"
    );
    assert!(
        resize_path.contains("create_hdr_scene_bind_group("),
        "resize must rebind the HDR view"
    );
    assert!(
        !resize_path.contains("create_postprocess_pipeline("),
        "resize must never recreate the postprocess pipeline"
    );
}

#[test]
fn shadow_stays_on_direct_light_only_and_sky_is_directional() {
    let source = include_str!("../src/shader.wgsl");
    let direct = source
        .find("let direct = direct_unshadowed * shadow_visibility;")
        .expect("direct PBR lighting must be multiplied by shadow visibility");
    let sky = source
        .find("fn sky_diffuse_irradiance(")
        .expect("the sky-diffuse model must exist");
    let lit = source
        .find("let lit_rgb = direct + ambient;")
        .expect("ambient must remain outside the shadow multiplier");
    assert!(direct < lit && sky < lit);
    assert!(
        !source
            .split("fn sky_diffuse_irradiance(")
            .nth(1)
            .expect("sky body")
            .split("fn environment_specular_response(")
            .next()
            .expect("sky body end")
            .contains("shadow_visibility"),
        "the sky-diffuse irradiance must never be shadowed"
    );
    // G3A-R terrain stack + debug path preservation.
    let terrain = source
        .split("fn fs_terrain(input: VertexOutput)")
        .nth(1)
        .expect("terrain entry must exist");
    assert!(
        terrain.contains("terrain_material.debug_mode") && terrain.contains("mode == 5u"),
        "terrain debug channels must remain intact"
    );
    assert!(
        terrain.contains("terrain_material.roughness * r_stack"),
        "the G3A-R roughness stack must remain intact"
    );
}

#[test]
fn postprocess_wgsl_parses_with_postprocess_entry_points() {
    let module = naga::front::wgsl::parse_str(include_str!("../src/shader.wgsl"))
        .expect("shader.wgsl must parse under naga");
    for entry in [
        "vs_sky_fullscreen",
        "fs_postprocess",
        "fs_sky",
        "fs_terrain",
        "fs_lit",
    ] {
        assert!(
            module.entry_points.iter().any(|ep| ep.name == entry),
            "entry point {entry} must exist"
        );
    }
    // Postprocess group binding expectations: texture (0), sampler (1),
    // uniform (2) — the layout the pipeline binds.
    assert!(
        include_str!("../src/shader.wgsl").contains("@group(5) @binding(0)"),
        "the postprocess bindings must live at group 5"
    );
}

#[test]
fn exposure_never_leaks_into_physics_boundary() {
    // The renderer crate must not depend on any simulation-domain crate.
    let manifest = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml"))
        .expect("renderer Cargo.toml must be readable");
    for domain in [
        "sim_core",
        "sim_math",
        "aircraft",
        "model",
        "replay",
        "telemetry",
    ] {
        assert!(
            !manifest.contains(&format!("\"{domain}\"")),
            "renderer must not depend on {domain}"
        );
    }
}

// ---------------------------------------------------------------------------
// Headless GPU proof of the HDR chain (offline only, never in CI).
// ---------------------------------------------------------------------------

#[test]
#[ignore = "requires a GPU; run with -- --ignored"]
fn hdr_scene_to_tone_mapped_surface_offscreen() {
    let (device, queue) = headless_device();
    const SIZE: u32 = 4;

    // HDR scene texture filled with known scene-referred linear values.
    let hdr_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("g3b test HDR scene"),
        size: wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let hdr_view = hdr_texture.create_view(&wgpu::TextureViewDescriptor::default());
    // All texels get the same value per channel: (4.0, 2.0, 1.0).
    let mut texels = Vec::with_capacity((SIZE * SIZE * 4) as usize);
    for _ in 0..SIZE * SIZE {
        // IEEE-754 binary16 bit patterns for (4.0, 2.0, 1.0, 1.0).
        texels.extend_from_slice(&[0x4400u16, 0x4000, 0x3c00, 0x3c00]);
    }
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &hdr_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        bytemuck::cast_slice(&texels),
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(SIZE * 4 * 2),
            rows_per_image: Some(SIZE),
        },
        wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
    );

    let display_format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("g3b test shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("../src/shader.wgsl").into()),
    });
    let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("g3b test postprocess bg layout"),
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
                    min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<
                        PostProcessTestUniform,
                    >() as u64),
                },
                count: None,
            },
        ],
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("g3b test postprocess layout"),
        bind_group_layouts: &[None, None, None, None, None, Some(&bind_group_layout)],
        immediate_size: 0,
    });
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("g3b test postprocess sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Nearest,
        min_filter: wgpu::FilterMode::Nearest,
        mipmap_filter: wgpu::MipmapFilterMode::Nearest,
        ..Default::default()
    });
    // EV 0: multiplier 1.0 — the tone mapper does the compression.
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("g3b test postprocess uniform"),
        contents: bytemuck::bytes_of(&PostProcessTestUniform::new(0.0)),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("g3b test postprocess bind group"),
        layout: &bind_group_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&hdr_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    });
    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("g3b test postprocess pipeline"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &shader,
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
            module: &shader,
            entry_point: Some("fs_postprocess"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: display_format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    });
    let display_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("g3b test display target"),
        size: wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: display_format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let display_view = display_texture.create_view(&wgpu::TextureViewDescriptor::default());

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
    {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("g3b test postprocess pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &display_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&pipeline);
        pass.set_bind_group(5, &bind_group, &[]);
        pass.draw(0..3, 0..1);
    }
    queue.submit(std::iter::once(encoder.finish()));

    // Read back the sRGB-encoded display bytes and check channel ordering.
    let unpadded_row_bytes = SIZE * 4;
    let row_bytes = unpadded_row_bytes.div_ceil(wgpu::COPY_BYTES_PER_ROW_ALIGNMENT)
        * wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("g3b test readback"),
        size: (row_bytes * SIZE) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture: &display_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row_bytes),
                rows_per_image: Some(SIZE),
            },
        },
        wgpu::Extent3d {
            width: SIZE,
            height: SIZE,
            depth_or_array_layers: 1,
        },
    );
    queue.submit(std::iter::once(encoder.finish()));

    let slice = buffer.slice(..);
    let (tx, rx) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        tx.send(result).expect("map channel");
    });
    let _ = device.poll(wgpu::PollType::wait_indefinitely());
    rx.recv().expect("map callback").expect("map must succeed");
    let mapped = slice.get_mapped_range().expect("mapped range");
    let bytes = mapped.to_vec();
    drop(mapped);
    buffer.unmap();

    // One texel of the 4x4 fill.
    let rgba = [bytes[0], bytes[1], bytes[2], bytes[3]];
    // (4.0, 2.0, 1.0) @ EV0 -> Khronos Neutral -> sRGB encode.
    // Reference from the Rust mirror:
    let expected = khronos_pbr_neutral_rgb([4.0, 2.0, 1.0]);
    let to_srgb_u8 = |v: f32| {
        let c = v.clamp(0.0, 1.0);
        (if c <= 0.003_130_8 {
            12.92 * c
        } else {
            1.055 * c.powf(1.0 / 2.4) - 0.055
        } * 255.0)
            .round() as u8
    };
    let expect_rgb = [
        to_srgb_u8(expected[0]),
        to_srgb_u8(expected[1]),
        to_srgb_u8(expected[2]),
    ];
    assert!(
        (rgba[0] as i16 - expect_rgb[0] as i16).abs() <= 2
            && (rgba[1] as i16 - expect_rgb[1] as i16).abs() <= 2
            && (rgba[2] as i16 - expect_rgb[2] as i16).abs() <= 2,
        "HDR chain mismatch: gpu {rgba:?} vs expected {expect_rgb:?} ({expected:?})"
    );
    // Highlight must NOT clip to raw white (4.0 -> compressed below 255).
    assert!(
        rgba[0] < 255 && rgba[0] > 240,
        "red highlight must be compressed, not hard-clipped: {rgba:?}"
    );
}

#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct PostProcessTestUniform {
    exposure_ev: f32,
    padding: [f32; 3],
}

impl PostProcessTestUniform {
    fn new(exposure_ev: f32) -> Self {
        Self {
            exposure_ev,
            padding: [0.0; 3],
        }
    }
}

/// Minimal headless device (no surface). Same shape as the app's GPU setup.
fn headless_device() -> (wgpu::Device, wgpu::Queue) {
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
    let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
        power_preference: wgpu::PowerPreference::HighPerformance,
        force_fallback_adapter: false,
        compatible_surface: None,
        apply_limit_buckets: false,
    }))
    .expect("no adapter available");
    pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
        label: Some("g3b headless device"),
        required_features: wgpu::Features::empty(),
        required_limits: wgpu::Limits {
            max_bind_groups: adapter.limits().max_bind_groups,
            ..wgpu::Limits::default()
        },
        experimental_features: wgpu::ExperimentalFeatures::disabled(),
        memory_hints: wgpu::MemoryHints::Performance,
        trace: wgpu::Trace::Off,
    }))
    .expect("no device available")
}
