//! PF1: every GPU resource the Photo Field presentation path owns.
//!
//! Kept out of `gpu.rs` on purpose: the renderer's frame path stays readable
//! when the photographic field's panorama upload, depth-proxy baking, shadow
//! mask target and three dedicated pipelines live behind one `PhotoFieldGpu`
//! value that `WgpuRenderer` either owns or does not.
//!
//! # Lifecycle contract
//!
//! Everything here is built by [`build_photo_field_gpu`] during renderer
//! initialization, or rebuilt by [`resize`] when the surface extent changes.
//! The frame path only binds and draws: no texture, sampler, bind group,
//! buffer or pipeline is created per frame, and the presentation-only uniform
//! is written exactly once because the panorama calibration never moves.
//!
//! # Why the proxies cannot draw colour
//!
//! The depth-proxy pipeline attaches no colour target at all and has no
//! fragment stage, so the invisible photographic stand-ins are structurally
//! incapable of contributing a pixel anywhere. They exist only as camera-space
//! depth: `photo_proxy_depth` decides where the 3D aircraft is occluded by the
//! photograph, and (with depth writes disabled) where a photographed tree
//! suppresses the aircraft's shadow on the photographed grass.

use crate::glb::GlbAsset;
use crate::gpu::{RendererError, upload_terrain_mip_level};
use crate::photo_field::{
    PHOTO_FIELD_DEPTH_PROXY_GLB, PHOTO_FIELD_GROUND_NODE_NAME, PHOTO_FIELD_PANORAMA_HEIGHT,
    PHOTO_FIELD_PANORAMA_JPEG, PHOTO_FIELD_PANORAMA_WIDTH, PhotoFieldConfig,
    embedded_photo_field_config,
};
use crate::terrain_textures::mip_level_count_for_size;
use crate::texture::decode_image;
use bytemuck::{Pod, Zeroable};
use std::mem::size_of;
use std::path::PathBuf;
use wgpu::util::DeviceExt;

/// Depth format of the invisible photographic stand-in target.
const PROXY_DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Single-channel float format of the presentation-only shadow mask.
///
/// `R16Float` is enough for one attenuation factor and, unlike `Rgba16Float`,
/// keeps the mask target at a quarter of the memory. The shader reads it with
/// `textureLoad`, so no float-filtering capability is required.
const SHADOW_MASK_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::R16Float;

/// Label of the embedded depth-proxy asset, used in every typed load error.
const DEPTH_PROXY_LABEL: &str = "photo_field_depth";

/// Mirror of the WGSL `PhotoFieldUniform` struct (group 6, binding 5).
///
/// Presentation-only: the panorama calibration and the photographic shadow
/// attenuation. Exposure deliberately lives in the shared group-5
/// `PostProcessUniform` so the photograph and the tone-mapped aircraft can
/// never drift apart.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct PhotoFieldUniformRaw {
    pub(crate) panorama_yaw_rad: f32,
    pub(crate) panorama_pitch_rad: f32,
    pub(crate) shadow_strength: f32,
    pub(crate) padding_0: f32,
}

impl PhotoFieldUniformRaw {
    fn from_config(config: &PhotoFieldConfig) -> Self {
        Self {
            panorama_yaw_rad: config.panorama_yaw_rad,
            panorama_pitch_rad: config.panorama_pitch_rad,
            shadow_strength: config.shadow_strength,
            padding_0: 0.0,
        }
    }
}

/// Position-only depth-proxy vertex: 12 bytes, one `Float32x3` at location 0.
///
/// The proxies are never shaded, so carrying normals, colours or UVs would be
/// dead weight in both the buffer and the vertex fetch.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub(crate) struct ProxyVertex {
    pub(crate) position: [f32; 3],
}

static PROXY_VERTEX_ATTRIBUTES: [wgpu::VertexAttribute; 1] =
    wgpu::vertex_attr_array![0 => Float32x3];

fn proxy_vertex_buffers() -> [Option<wgpu::VertexBufferLayout<'static>>; 1] {
    [Some(wgpu::VertexBufferLayout {
        array_stride: size_of::<ProxyVertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &PROXY_VERTEX_ATTRIBUTES,
    })]
}

/// One uploaded depth-proxy draw batch.
///
/// `Debug` is derived so the fields stay live: the GPU owns these buffers and
/// nothing on the CPU reads them back.
#[derive(Debug)]
pub(crate) struct ProxyBatch {
    pub(crate) vertex_buffer: wgpu::Buffer,
    pub(crate) index_buffer: wgpu::Buffer,
    pub(crate) index_count: u32,
}

/// The CPU-side depth-proxy geometry, already split by photographic role.
///
/// Pure data so the partition can be unit-tested without a GPU: the ground
/// receiver is the only batch that catches the aircraft shadow, while every
/// other node is an occluder only.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ProxyGeometry {
    /// The `pf1_ground` instance(s).
    pub(crate) ground: ProxyMesh,
    /// Every other proxy node.
    pub(crate) occluders: ProxyMesh,
}

/// One baked, index-addressed triangle mesh in render-world space.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ProxyMesh {
    pub(crate) vertices: Vec<ProxyVertex>,
    pub(crate) indices: Vec<u32>,
}

impl ProxyMesh {
    fn new() -> Self {
        Self {
            vertices: Vec::new(),
            indices: Vec::new(),
        }
    }
}

/// All PF1 GPU resources, owned by `WgpuRenderer` only for the PhotoField preset.
///
/// `Debug` is derived so every field stays live: the backing textures of the
/// three views are retained purely for ownership (a dropped `wgpu::Texture`
/// would invalidate its view), and the manifest configuration is retained as
/// the record of which photograph these resources were built from.
#[derive(Debug)]
pub(crate) struct PhotoFieldGpu {
    /// The validated manifest configuration the pipelines were built from.
    /// Retained as the provenance record of which photograph these resources
    /// describe; the frame path reads the calibration from the uniform buffer.
    pub(crate) _config: PhotoFieldConfig,
    /// Written once at construction; the calibration never moves at runtime.
    pub(crate) uniform_buffer: wgpu::Buffer,
    /// Group 6. Kept alive because the capture path rebuilds the photo
    /// postprocess pipeline against it for a different colour format.
    pub(crate) bind_group_layout: wgpu::BindGroupLayout,
    /// Retained for ownership: dropping the texture would invalidate
    /// `panorama_view`.
    pub(crate) _panorama_texture: wgpu::Texture,
    pub(crate) panorama_view: wgpu::TextureView,
    pub(crate) panorama_sampler: wgpu::Sampler,
    pub(crate) proxy_depth_texture: wgpu::Texture,
    pub(crate) proxy_depth_view: wgpu::TextureView,
    pub(crate) mask_texture: wgpu::Texture,
    pub(crate) mask_view: wgpu::TextureView,
    pub(crate) bind_group: wgpu::BindGroup,
    /// Group 6 reduced to the uniform alone, bound by the shadow-mask pass: that
    /// pass attaches the proxy depth as its depth target, so binding the full
    /// photographic group there would sample the same texture as a RESOURCE
    /// inside the pass that writes it, which WebGPU rejects.
    pub(crate) uniform_bind_group: wgpu::BindGroup,
    pub(crate) proxy_pipeline: wgpu::RenderPipeline,
    pub(crate) mask_pipeline: wgpu::RenderPipeline,
    pub(crate) postprocess_pipeline: wgpu::RenderPipeline,
    /// Photographed trees, houses and garage: occluders, never shadow receivers.
    pub(crate) occluders: ProxyBatch,
    /// The photographic ground plane: the only shadow receiver.
    pub(crate) ground: ProxyBatch,
}

/// The extra group layouts the PF1 postprocess pipeline needs on top of the
/// shared group-5 postprocess layout.
///
/// `fs_postprocess_photo` reconstructs a world view direction, so unlike
/// `fs_postprocess` it reads the camera uniform at group 0 as well as the
/// photographic group 6.
#[derive(Clone, Copy)]
pub(crate) struct PhotoPostprocessLayouts<'a> {
    pub(crate) camera: &'a wgpu::BindGroupLayout,
    pub(crate) photo: &'a wgpu::BindGroupLayout,
}

impl PhotoFieldGpu {
    /// Recreate only the resolution-dependent resources after a surface resize.
    ///
    /// The panorama, the sampler, the baked proxy buffers, the uniform and all
    /// three pipelines are resolution-independent and survive untouched; the
    /// two render targets and the group-6 bind group that references them (and
    /// the new scene depth view) are rebuilt here and nowhere else.
    pub(crate) fn resize(
        &mut self,
        device: &wgpu::Device,
        depth_view: &wgpu::TextureView,
        width: u32,
        height: u32,
    ) {
        let (proxy_depth_texture, proxy_depth_view) =
            create_proxy_depth_target(device, width, height);
        let (mask_texture, mask_view) = create_shadow_mask_target(device, width, height);
        let bind_group = create_photo_field_bind_group(
            device,
            &self.bind_group_layout,
            &self.panorama_view,
            &self.panorama_sampler,
            depth_view,
            &proxy_depth_view,
            &mask_view,
            &self.uniform_buffer,
            "PF1 photo field bind group (resized)",
        );
        self.proxy_depth_texture = proxy_depth_texture;
        self.proxy_depth_view = proxy_depth_view;
        self.mask_texture = mask_texture;
        self.mask_view = mask_view;
        self.bind_group = bind_group;
    }
}

/// Build every PF1 GPU resource from the embedded meadow assets.
///
/// Called once from the renderer constructor. Fails closed — and before any
/// resource is retained — on a rejected manifest, an unexpected panorama
/// extent, an undecodable panorama, an unloadable depth proxy, or a proxy scene
/// that is missing either its ground receiver or its occluders.
///
/// # Errors
///
/// Returns the typed [`RendererError`] that applies.
#[allow(clippy::too_many_arguments)]
pub(crate) fn build_photo_field_gpu(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    shader: &wgpu::ShaderModule,
    camera_layout: &wgpu::BindGroupLayout,
    object_layout: &wgpu::BindGroupLayout,
    environment_layout: &wgpu::BindGroupLayout,
    postprocess_layout: &wgpu::BindGroupLayout,
    surface_format: wgpu::TextureFormat,
    depth_view: &wgpu::TextureView,
    width: u32,
    height: u32,
    anisotropy: u16,
) -> Result<PhotoFieldGpu, RendererError> {
    let config = embedded_photo_field_config()?;

    let (panorama_texture, panorama_view, panorama_sampler) =
        create_panorama(device, queue, anisotropy)?;

    let proxy_asset = crate::glb::load_glb_bytes(PHOTO_FIELD_DEPTH_PROXY_GLB, DEPTH_PROXY_LABEL)?;
    let geometry = bake_proxy_geometry(&proxy_asset)?;
    let occluders = upload_proxy_batch(device, "PF1 photo occluder proxy", &geometry.occluders);
    let ground = upload_proxy_batch(device, "PF1 photo ground proxy", &geometry.ground);

    let (proxy_depth_texture, proxy_depth_view) = create_proxy_depth_target(device, width, height);
    let (mask_texture, mask_view) = create_shadow_mask_target(device, width, height);

    // UNIFORM without COPY_DST on purpose: the buffer is immutable after this
    // point, so a frame path that tried to rewrite the calibration would fail
    // validation instead of silently moving the photograph.
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("PF1 photo field uniform"),
        contents: bytemuck::bytes_of(&PhotoFieldUniformRaw::from_config(&config)),
        usage: wgpu::BufferUsages::UNIFORM,
    });

    let bind_group_layout = photo_field_bind_group_layout(device);
    let bind_group = create_photo_field_bind_group(
        device,
        &bind_group_layout,
        &panorama_view,
        &panorama_sampler,
        depth_view,
        &proxy_depth_view,
        &mask_view,
        &uniform_buffer,
        "PF1 photo field bind group",
    );

    let proxy_pipeline = create_proxy_pipeline(device, shader, camera_layout, object_layout);
    let uniform_only_layout = photo_uniform_only_bind_group_layout(device);
    let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("PF1 photo uniform-only bind group"),
        layout: &uniform_only_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 5,
            resource: uniform_buffer.as_entire_binding(),
        }],
    });
    let mask_pipeline = create_mask_pipeline(
        device,
        shader,
        camera_layout,
        object_layout,
        environment_layout,
        &uniform_only_layout,
    );
    let postprocess_pipeline = create_photo_postprocess_pipeline(
        device,
        shader,
        PhotoPostprocessLayouts {
            camera: camera_layout,
            photo: &bind_group_layout,
        },
        postprocess_layout,
        surface_format,
        "PF1 photo postprocess pipeline",
    );

    Ok(PhotoFieldGpu {
        _config: config,
        uniform_buffer,
        bind_group_layout,
        _panorama_texture: panorama_texture,
        panorama_view,
        panorama_sampler,
        proxy_depth_texture,
        proxy_depth_view,
        mask_texture,
        mask_view,
        bind_group,
        uniform_bind_group,
        proxy_pipeline,
        mask_pipeline,
        postprocess_pipeline,
        occluders,
        ground,
    })
}

// ---------------------------------------------------------------------------
// Panorama
// ---------------------------------------------------------------------------

/// Decode the embedded panorama, build its wrap-on-U mip chain and upload it.
fn create_panorama(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    anisotropy: u16,
) -> Result<(wgpu::Texture, wgpu::TextureView, wgpu::Sampler), RendererError> {
    let decoded = decode_image(PHOTO_FIELD_PANORAMA_JPEG)?;
    if decoded.width != PHOTO_FIELD_PANORAMA_WIDTH || decoded.height != PHOTO_FIELD_PANORAMA_HEIGHT
    {
        return Err(RendererError::PhotoFieldPanoramaExtent {
            width: decoded.width,
            height: decoded.height,
            expected_width: PHOTO_FIELD_PANORAMA_WIDTH,
            expected_height: PHOTO_FIELD_PANORAMA_HEIGHT,
        });
    }

    let chain = panorama_mip_chain(decoded.rgba8, decoded.width, decoded.height);
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("PF1 meadow equirectangular panorama"),
        size: wgpu::Extent3d {
            width: PHOTO_FIELD_PANORAMA_WIDTH,
            height: PHOTO_FIELD_PANORAMA_HEIGHT,
            depth_or_array_layers: 1,
        },
        mip_level_count: chain.len() as u32,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        // COPY_DST is required by `write_texture`; the panorama is never
        // rendered into.
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    for (level, mip) in chain.iter().enumerate() {
        upload_terrain_mip_level(
            queue,
            &texture,
            level as u32,
            mip.width,
            mip.height,
            &mip.bytes,
            4,
        )?;
    }
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

    // Repeat on U is what makes the 360° azimuth seamless; V clamps because the
    // equirectangular poles are single rows, not a wrap boundary.
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("PF1 panorama sampler"),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        anisotropy_clamp: anisotropy,
        ..Default::default()
    });
    Ok((texture, view, sampler))
}

/// One mip level of the panorama: RGBA8, sRGB-encoded texels.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct PanoramaMipLevel {
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) bytes: Vec<u8>,
}

/// Deterministic mip chain of a 2:1 equirectangular panorama.
///
/// Level 0 is the decoded photograph itself; each following level halves the
/// dimensions with a 2x2 box filter that WRAPS on U and CLAMPS on V:
///
/// - the azimuth is periodic, so a level that filtered the u=0/u=1 seam
///   against clamped edge texels would smear a visible meridian band straight
///   through the horizon;
/// - the elevation is not periodic, so the poles must clamp instead of
///   wrapping the zenith into the nadir.
///
/// Filtering runs in linear light (`srgb_to_linear_f64` / `linear_to_srgb_f64`,
/// the crate's single canonical transfer pair) and re-encodes to sRGB, because
/// the texture is uploaded as `Rgba8UnormSrgb` and the hardware decodes on
/// sample. Averaging sRGB-encoded bytes directly would darken every minified
/// level.
///
/// Pure and bitwise deterministic; takes ownership of level 0 so an 8K
/// panorama is never duplicated in memory.
///
/// # Panics
///
/// Panics if `width` is not a positive power of two, `height` is not half of
/// it, or the buffer length does not match `width * height * 4`. The production
/// caller has already rejected any extent other than the surveyed panorama with
/// a typed error before reaching here; the relaxed shape keeps the chain itself
/// unit-testable at a few texels across.
pub(crate) fn panorama_mip_chain(rgba8: Vec<u8>, width: u32, height: u32) -> Vec<PanoramaMipLevel> {
    assert!(
        width.is_power_of_two(),
        "panorama width must be a power of two"
    );
    assert_eq!(
        height * 2,
        width,
        "an equirectangular panorama is exactly 2:1"
    );
    assert_eq!(
        rgba8.len(),
        (width as usize) * (height as usize) * 4,
        "panorama byte count"
    );

    let levels = mip_level_count_for_size(width) as usize;
    let mut chain = Vec::with_capacity(levels);
    chain.push(PanoramaMipLevel {
        width,
        height,
        bytes: rgba8,
    });
    while chain.len() < levels {
        let previous = chain.last().expect("the chain always holds level 0");
        let next = downsample_panorama_level(previous.width, previous.height, &previous.bytes);
        chain.push(next);
    }
    chain
}

/// One wrap-on-U / clamp-on-V 2x2 box-filter step in linear light.
fn downsample_panorama_level(width: u32, height: u32, src: &[u8]) -> PanoramaMipLevel {
    let out_width = (width / 2).max(1);
    let out_height = (height / 2).max(1);
    let mut bytes = vec![0u8; (out_width as usize) * (out_height as usize) * 4];
    for out_y in 0..out_height {
        for out_x in 0..out_width {
            let mut linear = [0.0f64; 3];
            let mut alpha = 0.0f64;
            for (dy, dx) in [(0u32, 0u32), (0, 1), (1, 0), (1, 1)] {
                // Wrap on U, clamp on V: see `panorama_mip_chain`.
                let x = (out_x * 2 + dx) % width;
                let y = (out_y * 2 + dy).min(height - 1);
                let index = ((y * width + x) * 4) as usize;
                for (channel, accumulated) in linear.iter_mut().enumerate() {
                    *accumulated += srgb_u8_to_linear(src[index + channel]);
                }
                alpha += f64::from(src[index + 3]);
            }
            let out_index = ((out_y * out_width + out_x) * 4) as usize;
            for (channel, accumulated) in linear.iter().enumerate() {
                bytes[out_index + channel] = linear_to_srgb_u8(accumulated * 0.25);
            }
            bytes[out_index + 3] = (alpha * 0.25).round().clamp(0.0, 255.0) as u8;
        }
    }
    PanoramaMipLevel {
        width: out_width,
        height: out_height,
        bytes,
    }
}

fn srgb_u8_to_linear(byte: u8) -> f64 {
    crate::texture::srgb_to_linear_f64(f64::from(byte) / 255.0)
}

fn linear_to_srgb_u8(linear: f64) -> u8 {
    (crate::texture::linear_to_srgb_f64(linear) * 255.0)
        .round()
        .clamp(0.0, 255.0) as u8
}

// ---------------------------------------------------------------------------
// Depth proxies
// ---------------------------------------------------------------------------

/// Bake every scene instance's `world_transform` into position-only vertices
/// and split the result into the ground receiver and the occluders.
///
/// Baking on the CPU is what lets both batches draw with the identity object
/// uniform, so the frame path needs no per-proxy matrix upload and no
/// instancing. Fails closed when either role is empty: a field with no ground
/// receiver would drop the aircraft shadow entirely, and a field with no
/// occluder would let the aircraft fly in front of photographed trees.
///
/// # Errors
///
/// Returns a typed error when a role is missing or the baked vertex count
/// exceeds the u32 index range.
pub(crate) fn bake_proxy_geometry(asset: &GlbAsset) -> Result<ProxyGeometry, RendererError> {
    let mut ground = ProxyMesh::new();
    let mut occluders = ProxyMesh::new();

    for instance in &asset.instances {
        let target = if instance.node_name.as_deref() == Some(PHOTO_FIELD_GROUND_NODE_NAME) {
            &mut ground
        } else {
            &mut occluders
        };
        let mesh = &asset.meshes[instance.mesh_index];
        for primitive in &mesh.primitives {
            let base = u32::try_from(target.vertices.len()).map_err(|_| {
                RendererError::PhotoFieldGlb(crate::glb::GlbLoadError::TooManyVertices {
                    path: PathBuf::from(DEPTH_PROXY_LABEL),
                })
            })?;
            target
                .vertices
                .extend(primitive.vertices.iter().map(|vertex| ProxyVertex {
                    position: baked_position(&instance.world_transform, vertex.position),
                }));
            target
                .indices
                .extend(primitive.indices.iter().map(|index| base + index));
        }
    }

    if ground.indices.is_empty() {
        return Err(RendererError::PhotoFieldProxyPartitionEmpty {
            partition: PHOTO_FIELD_GROUND_NODE_NAME,
        });
    }
    if occluders.indices.is_empty() {
        return Err(RendererError::PhotoFieldProxyPartitionEmpty {
            partition: "occluder",
        });
    }
    Ok(ProxyGeometry { ground, occluders })
}

/// Apply a row-major render-world transform to a mesh-space position.
fn baked_position(world_transform: &crate::Mat4, position: [f32; 3]) -> [f32; 3] {
    let transformed =
        world_transform.transform_homogeneous([position[0], position[1], position[2], 1.0]);
    [transformed[0], transformed[1], transformed[2]]
}

fn upload_proxy_batch(device: &wgpu::Device, label: &str, mesh: &ProxyMesh) -> ProxyBatch {
    ProxyBatch {
        vertex_buffer: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytemuck::cast_slice(&mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        }),
        index_buffer: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytemuck::cast_slice(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        }),
        index_count: mesh.indices.len() as u32,
    }
}

// ---------------------------------------------------------------------------
// Targets, group 6 and pipelines
// ---------------------------------------------------------------------------

fn create_proxy_depth_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("PF1 photo depth-proxy target"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: PROXY_DEPTH_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

fn create_shadow_mask_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("PF1 photo shadow mask target"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: SHADOW_MASK_FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

/// Group 6: panorama, its sampler, the two 1:1 depth textures, the shadow mask
/// and the presentation-only uniform.
///
/// The mask is declared NON-filterable because the shader reads all three
/// auxiliary textures with `textureLoad`; declaring them filterable would
/// request a float-filtering capability the renderer does not require.
pub(crate) fn photo_field_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    const FRAGMENT: wgpu::ShaderStages = wgpu::ShaderStages::FRAGMENT;
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("PF1 photo field layout"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Depth,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: wgpu::BufferSize::new(
                        size_of::<PhotoFieldUniformRaw>() as u64
                    ),
                },
                count: None,
            },
        ],
    })
}

/// Group 6 carrying only the presentation uniform (binding 5).
///
/// The shadow-mask pass attaches the proxy depth as its depth target while the
/// full photographic group samples that same texture; WebGPU forbids RESOURCE
/// and DEPTH_STENCIL write usages of one texture inside a single pass. The mask
/// fragment stage reads only `photo_field.shadow_strength`, so this sparse
/// layout is sufficient for it and keeps the conflict structurally impossible.
pub(crate) fn photo_uniform_only_bind_group_layout(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("PF1 photo uniform-only layout"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 5,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: wgpu::BufferSize::new(size_of::<PhotoFieldUniformRaw>() as u64),
            },
            count: None,
        }],
    })
}

#[allow(clippy::too_many_arguments)]
fn create_photo_field_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    panorama_view: &wgpu::TextureView,
    panorama_sampler: &wgpu::Sampler,
    scene_depth_view: &wgpu::TextureView,
    proxy_depth_view: &wgpu::TextureView,
    mask_view: &wgpu::TextureView,
    uniform_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(panorama_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(panorama_sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(scene_depth_view),
            },
            wgpu::BindGroupEntry {
                binding: 3,
                resource: wgpu::BindingResource::TextureView(proxy_depth_view),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::TextureView(mask_view),
            },
            wgpu::BindGroupEntry {
                binding: 5,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    })
}

/// Both proxy pipelines need no backface culling: a coarse photographic
/// stand-in is routinely entered by the camera, and a culled far side would
/// punch a hole in the occlusion the photograph depicts.
fn proxy_primitive_state() -> wgpu::PrimitiveState {
    wgpu::PrimitiveState {
        topology: wgpu::PrimitiveTopology::TriangleList,
        strip_index_format: None,
        front_face: wgpu::FrontFace::Ccw,
        cull_mode: None,
        unclipped_depth: false,
        polygon_mode: wgpu::PolygonMode::Fill,
        conservative: false,
    }
}

/// Depth-only proxy pipeline: no colour target and no fragment stage at all.
fn create_proxy_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    camera_layout: &wgpu::BindGroupLayout,
    object_layout: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("PF1 photo depth-proxy pipeline layout"),
        bind_group_layouts: &[Some(camera_layout), Some(object_layout)],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("PF1 photo depth-proxy pipeline"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_photo_proxy"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &proxy_vertex_buffers(),
        },
        primitive: proxy_primitive_state(),
        depth_stencil: Some(wgpu::DepthStencilState {
            format: PROXY_DEPTH_FORMAT,
            depth_write_enabled: Some(true),
            depth_compare: Some(wgpu::CompareFunction::Less),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        // `color_targets: &[]` with `fragment: None` is the structural guarantee
        // that the invisible proxies cannot write colour anywhere.
        fragment: None,
        multiview_mask: None,
        cache: None,
    })
}

/// Shadow-mask pipeline: writes one attenuation factor per covered texel while
/// depth-testing against the already-rasterized proxies.
fn create_mask_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    camera_layout: &wgpu::BindGroupLayout,
    object_layout: &wgpu::BindGroupLayout,
    environment_layout: &wgpu::BindGroupLayout,
    photo_layout: &wgpu::BindGroupLayout,
) -> wgpu::RenderPipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("PF1 photo shadow mask pipeline layout"),
        bind_group_layouts: &[
            Some(camera_layout),
            Some(object_layout),
            Some(environment_layout),
            None,
            None,
            None,
            Some(photo_layout),
        ],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("PF1 photo shadow mask pipeline"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_photo_shadow_mask"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &proxy_vertex_buffers(),
        },
        primitive: proxy_primitive_state(),
        depth_stencil: Some(wgpu::DepthStencilState {
            format: PROXY_DEPTH_FORMAT,
            // Depth WRITES stay disabled: the mask pass reuses the proxy depth
            // as a read-only occlusion test, and writing would corrupt the
            // depth the final composite compares the aircraft against.
            depth_write_enabled: Some(false),
            depth_compare: Some(wgpu::CompareFunction::LessEqual),
            stencil: wgpu::StencilState::default(),
            bias: wgpu::DepthBiasState::default(),
        }),
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_photo_shadow_mask"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: SHADOW_MASK_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

/// The PF1 final composite, built for one colour format.
///
/// Shared by the surface pipeline (built once at initialization) and the
/// capture pipeline (built per capture against `CAPTURE_FORMAT`), so the
/// presented frame and the captured frame can never be produced by two
/// different composites.
pub(crate) fn create_photo_postprocess_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layouts: PhotoPostprocessLayouts<'_>,
    postprocess_layout: &wgpu::BindGroupLayout,
    color_format: wgpu::TextureFormat,
    label: &'static str,
) -> wgpu::RenderPipeline {
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("PF1 photo postprocess pipeline layout"),
        bind_group_layouts: &[
            Some(layouts.camera),
            None,
            None,
            None,
            None,
            Some(postprocess_layout),
            Some(layouts.photo),
        ],
        immediate_size: 0,
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_sky_fullscreen"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[],
        },
        primitive: proxy_primitive_state(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_postprocess_photo"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: color_format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::texture::padded_bytes_per_row_checked_for_bytes_per_pixel;

    #[test]
    fn the_photo_field_uniform_mirrors_the_wgsl_struct() {
        assert_eq!(size_of::<PhotoFieldUniformRaw>(), 16);
        assert_eq!(size_of::<PhotoFieldUniformRaw>(), 4 * size_of::<f32>());
        // Field offsets must match the WGSL declaration order exactly: four
        // scalar f32s, each 4-byte aligned, so no implicit padding exists and
        // the offsets are simply 0/4/8/12.
        let raw = PhotoFieldUniformRaw {
            panorama_yaw_rad: 1.0,
            panorama_pitch_rad: 2.0,
            shadow_strength: 3.0,
            padding_0: 4.0,
        };
        let bytes = bytemuck::bytes_of(&raw);
        assert_eq!(bytes.len(), 16);
        let words: &[f32] = bytemuck::cast_slice(bytes);
        assert_eq!(words, &[1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn the_uniform_carries_the_manifest_calibration_and_zero_padding() {
        let config = PhotoFieldConfig {
            pilot_position_render_m: [17.469, 1.6, -9.736],
            panorama_yaw_rad: 0.25,
            panorama_pitch_rad: -0.125,
            sun_direction_render: [0.0, 1.0, 0.0],
            sun_intensity: 2.6,
            sun_rgb: [1.0, 0.95, 0.85],
            shadow_strength: 0.45,
        };
        let raw = PhotoFieldUniformRaw::from_config(&config);
        assert_eq!(raw.panorama_yaw_rad, config.panorama_yaw_rad);
        assert_eq!(raw.panorama_pitch_rad, config.panorama_pitch_rad);
        assert_eq!(raw.shadow_strength, config.shadow_strength);
        assert_eq!(raw.padding_0, 0.0);
    }

    #[test]
    fn the_proxy_vertex_is_position_only_at_twelve_bytes() {
        // The vertex buffer layout declares array_stride 12 with a single
        // Float32x3 at offset 0; the struct must stay exactly that.
        assert_eq!(size_of::<ProxyVertex>(), 12);
        let vertex = ProxyVertex {
            position: [1.0, 2.0, 3.0],
        };
        assert_eq!(bytemuck::bytes_of(&vertex).len(), 12);
        assert_eq!(
            proxy_vertex_buffers()[0]
                .as_ref()
                .expect("one vertex buffer slot")
                .array_stride,
            12
        );
    }

    /// A 4x2 synthetic panorama whose two halves differ, with the discontinuity
    /// exactly at the u=0/u=1 seam (column 3 neighbours column 0).
    fn seam_image() -> (u32, u32, Vec<u8>) {
        let (width, height) = (4u32, 2u32);
        // columns: 0 -> black, 1 -> black, 2 -> white, 3 -> white
        let row: Vec<u8> = [0u8, 0, 255, 255]
            .iter()
            .flat_map(|value| [*value, *value, *value, 255])
            .collect();
        let mut bytes = Vec::with_capacity(row.len() * height as usize);
        for _ in 0..height {
            bytes.extend_from_slice(&row);
        }
        (width, height, bytes)
    }

    #[test]
    fn the_full_chain_runs_from_the_base_down_to_one_by_one() {
        // 8x4 exercises the same level count rule as the real 8192x4096
        // panorama (log2(width) + 1) at a size a unit test can afford.
        let (width, height) = (8u32, 4u32);
        let chain = panorama_mip_chain(vec![128u8; (width * height * 4) as usize], width, height);
        assert_eq!(chain.len(), mip_level_count_for_size(width) as usize);
        assert_eq!((chain[0].width, chain[0].height), (width, height));
        for pair in chain.windows(2) {
            let (previous, next) = (&pair[0], &pair[1]);
            assert_eq!(next.width, (previous.width / 2).max(1));
            assert_eq!(next.height, (previous.height / 2).max(1));
            assert_eq!(
                next.bytes.len(),
                (next.width * next.height * 4) as usize,
                "every level must stay tightly packed"
            );
        }
        let last = chain.last().expect("the chain is never empty");
        assert_eq!((last.width, last.height), (1, 1));
    }

    #[test]
    fn the_chain_is_bitwise_deterministic() {
        let (width, height, bytes) = seam_image();
        let first = panorama_mip_chain(bytes.clone(), width, height);
        let second = panorama_mip_chain(bytes, width, height);
        assert_eq!(first, second);
    }

    #[test]
    fn the_wrap_mip_chain_never_bleeds_across_the_u_seam() {
        // The box phase is (2x, 2x+1), so the seam between the last and first
        // column always falls BETWEEN two boxes. A clamp-on-U or off-by-one
        // phase would average column 3 against column 0 and turn both level-1
        // texels mid-grey.
        let (width, height, bytes) = seam_image();
        let level = downsample_panorama_level(width, height, &bytes);
        assert_eq!((level.width, level.height), (2, 1));
        assert_eq!(
            &level.bytes[0..4],
            &[0, 0, 0, 255],
            "the black half must stay black"
        );
        assert_eq!(
            &level.bytes[4..8],
            &[255, 255, 255, 255],
            "the white half must stay white"
        );
    }

    #[test]
    fn u_indexing_wraps_instead_of_reading_past_the_last_column() {
        // A degenerate one-column level makes the wrap load-bearing: without
        // `% width` the second horizontal tap of the box would index one texel
        // past the end of the buffer and panic.
        let (width, height) = (1u32, 1u32);
        let bytes = vec![40u8, 40, 40, 255];
        let level = downsample_panorama_level(width, height, &bytes);
        assert_eq!((level.width, level.height), (1, 1));
        // All four taps land on the same wrapped texel, so the level is that
        // texel re-encoded through the canonical transfer pair.
        for channel in 0..3 {
            assert!(
                level.bytes[channel].abs_diff(40) <= 1,
                "channel {channel} must round-trip, got {}",
                level.bytes[channel]
            );
        }
        assert_eq!(level.bytes[3], 255);
    }

    #[test]
    fn the_wrap_mip_chain_clamps_at_the_poles_instead_of_wrapping_them() {
        // The last level of the real chain is 1x1 produced from a 2x1 source:
        // the vertical box must clamp to the only row rather than wrap the
        // zenith into the nadir.
        let (width, height) = (2u32, 1u32);
        let bytes: Vec<u8> = [0u8, 255]
            .iter()
            .flat_map(|value| [*value, *value, *value, 255])
            .collect();
        let level = downsample_panorama_level(width, height, &bytes);
        assert_eq!((level.width, level.height), (1, 1));
        assert_eq!(level.bytes[3], 255, "alpha stays opaque");
        assert!(
            level.bytes[0] > 0 && level.bytes[0] < 255,
            "the single texel averages both columns, got {}",
            level.bytes[0]
        );
    }

    #[test]
    fn mip_filtering_happens_in_linear_light_not_on_encoded_bytes() {
        let (width, height) = (4u32, 2u32);
        let row: Vec<u8> = [0u8, 128, 128, 255]
            .iter()
            .flat_map(|value| [*value, *value, *value, 255])
            .collect();
        let mut bytes = Vec::with_capacity(row.len() * height as usize);
        for _ in 0..height {
            bytes.extend_from_slice(&row);
        }
        let level = downsample_panorama_level(width, height, &bytes);
        // Level 1 texel 1 averages columns 2 and 3 (128, 255). In linear light
        // that re-encodes to 205; averaging the encoded bytes would give 191.
        let linear_light = level.bytes[4];
        let byte_average = u32::from((128u16 + 255) / 2);
        assert!(
            u32::from(linear_light) > byte_average,
            "linear-light mixing ({linear_light}) must be brighter than byte mixing ({byte_average})"
        );
        assert_eq!(linear_light, 205);
    }

    #[test]
    fn the_transfer_pair_round_trips_every_byte() {
        assert_eq!(srgb_u8_to_linear(0), 0.0);
        assert_eq!(linear_to_srgb_u8(1.0), 255);
        for byte in [0u8, 1, 10, 64, 128, 191, 254, 255] {
            let round_trip = linear_to_srgb_u8(srgb_u8_to_linear(byte));
            assert!(
                round_trip.abs_diff(byte) <= 1,
                "the canonical transfer pair must round-trip {byte}, got {round_trip}"
            );
        }
    }

    #[test]
    fn every_panorama_mip_row_pads_to_the_copy_alignment() {
        let mut width = PHOTO_FIELD_PANORAMA_WIDTH;
        loop {
            let row = padded_bytes_per_row_checked_for_bytes_per_pixel(width, 4)
                .expect("no overflow at any panorama level");
            assert!(row.is_multiple_of(crate::texture::COPY_BYTES_PER_ROW_ALIGNMENT));
            assert!(row >= width * 4);
            if width == 1 {
                break;
            }
            width /= 2;
        }
        assert_eq!(width, 1);
    }

    /// A tiny in-memory stand-in for the proxy GLB: two named instances that
    /// share one mesh, one of them translated, so baking and partitioning can
    /// be checked without a GPU or an asset on disk.
    fn two_instance_asset() -> GlbAsset {
        use crate::Mat4;
        use crate::glb::{GlbMesh, GlbSceneInstance, PrimitiveMaterial, RenderPrimitive};
        use crate::mesh::{SAFE_NORMAL, SAFE_UV, Vertex};

        let unit = |position: [f32; 3]| Vertex {
            position,
            normal: SAFE_NORMAL,
            color: [1.0; 4],
            uv: SAFE_UV,
        };
        let primitive = RenderPrimitive {
            vertices: vec![
                unit([0.0, 0.0, 0.0]),
                unit([1.0, 0.0, 0.0]),
                unit([0.0, 1.0, 0.0]),
            ],
            indices: vec![0, 1, 2],
            material: PrimitiveMaterial {
                base_color_factor: [1.0; 4],
                base_color_texture: None,
                metallic_factor: 0.0,
                roughness_factor: 1.0,
                normal_texture: None,
                normal_texture_scale: 1.0,
                metallic_roughness_texture: None,
                sampler_config: crate::texture::SamplerConfig::default_sampler(),
            },
        };
        GlbAsset {
            primitives: vec![primitive.clone()],
            meshes: vec![GlbMesh {
                gltf_mesh_index: 0,
                primitives: vec![primitive],
            }],
            instances: vec![
                GlbSceneInstance {
                    node_index: 0,
                    node_name: Some(PHOTO_FIELD_GROUND_NODE_NAME.to_string()),
                    mesh_index: 0,
                    world_transform: Mat4::identity(),
                },
                GlbSceneInstance {
                    node_index: 1,
                    node_name: Some("pf1_tree_trunk_0".to_string()),
                    mesh_index: 0,
                    // Translate the occluder by +10 on Y so the bake is visible.
                    world_transform: Mat4::from_rows([
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 10.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ]),
                },
            ],
        }
    }

    #[test]
    fn proxy_partitioning_splits_by_node_name_and_bakes_the_world_transform() {
        let geometry = bake_proxy_geometry(&two_instance_asset()).expect("both roles present");
        assert_eq!(geometry.ground.vertices.len(), 3);
        assert_eq!(geometry.occluders.vertices.len(), 3);
        assert_eq!(geometry.ground.indices, vec![0, 1, 2]);
        // Indices are rebased per primitive so the two batches stay independent.
        assert_eq!(geometry.occluders.indices, vec![0, 1, 2]);
        assert_eq!(geometry.ground.vertices[0].position, [0.0, 0.0, 0.0]);
        assert_eq!(geometry.occluders.vertices[0].position, [0.0, 10.0, 0.0]);
        assert_eq!(geometry.occluders.vertices[1].position, [1.0, 10.0, 0.0]);
    }

    #[test]
    fn proxy_partitioning_is_a_pure_function_of_the_asset() {
        let asset = two_instance_asset();
        assert_eq!(
            bake_proxy_geometry(&asset).expect("first"),
            bake_proxy_geometry(&asset).expect("second")
        );
    }

    #[test]
    fn a_proxy_scene_without_a_ground_receiver_fails_closed() {
        let mut asset = two_instance_asset();
        asset
            .instances
            .retain(|instance| instance.node_name.as_deref() != Some(PHOTO_FIELD_GROUND_NODE_NAME));
        assert!(matches!(
            bake_proxy_geometry(&asset),
            Err(RendererError::PhotoFieldProxyPartitionEmpty { partition })
                if partition == PHOTO_FIELD_GROUND_NODE_NAME
        ));
    }

    #[test]
    fn a_proxy_scene_without_occluders_fails_closed() {
        let mut asset = two_instance_asset();
        asset
            .instances
            .retain(|instance| instance.node_name.as_deref() == Some(PHOTO_FIELD_GROUND_NODE_NAME));
        assert!(matches!(
            bake_proxy_geometry(&asset),
            Err(RendererError::PhotoFieldProxyPartitionEmpty { partition }) if partition == "occluder"
        ));
    }

    #[test]
    fn an_unnamed_proxy_node_is_treated_as_an_occluder_never_as_ground() {
        // Only the authored ground name receives the aircraft shadow; an
        // unnamed or renamed node must not silently become the receiver.
        let mut asset = two_instance_asset();
        for instance in &mut asset.instances {
            instance.node_name = None;
        }
        assert!(matches!(
            bake_proxy_geometry(&asset),
            Err(RendererError::PhotoFieldProxyPartitionEmpty { partition })
                if partition == PHOTO_FIELD_GROUND_NODE_NAME
        ));
    }
}
