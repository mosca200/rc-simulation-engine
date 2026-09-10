//! RV2-5 physical atmosphere and image-based-lighting generation.
//!
//! Every LUT, cubemap and the BRDF integration table is produced **on the GPU**
//! by dedicated render passes recorded into a single initialization command
//! encoder and submitted exactly once. Nothing here touches the frame loop: the
//! resources are persistent for the whole renderer life, are never regenerated
//! on resize, and never carry `COPY_DST`.
//!
//! Resources use `RENDER_ATTACHMENT | TEXTURE_BINDING` only and no storage
//! texture is involved. Cubemaps are `TextureDimension::D2` with
//! `depth_or_array_layers = 6`; generation renders into per-face/per-mip D2
//! views while the scene samples a `Cube` view of the same texture.

use std::f32::consts::PI;

use bytemuck::{Pod, Zeroable};
use wgpu::util::DeviceExt;

// ---------------------------------------------------------------------------
// Resource plan
// ---------------------------------------------------------------------------

pub(crate) const TRANSMITTANCE_SIZE: (u32, u32) = (256, 64);
pub(crate) const MULTI_SCATTERING_SIZE: (u32, u32) = (32, 32);
pub(crate) const SKY_VIEW_SIZE: (u32, u32) = (256, 128);
pub(crate) const ENVIRONMENT_CUBE_SIZE: u32 = 128;
pub(crate) const ENVIRONMENT_CUBE_MIP_COUNT: u32 = 1;
pub(crate) const IRRADIANCE_CUBE_SIZE: u32 = 32;
pub(crate) const SPECULAR_CUBE_SIZE: u32 = 128;
pub(crate) const SPECULAR_MIP_COUNT: u32 = 8;
pub(crate) const BRDF_LUT_SIZE: (u32, u32) = (256, 256);
pub(crate) const CUBE_FACE_COUNT: u32 = 6;

/// Shared format of every physical atmosphere/IBL resource.
pub(crate) const PHYSICAL_TEXTURE_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
/// `Rgba16Float` is four half-float channels.
pub(crate) const RGBA16F_BYTES_PER_TEXEL: u64 = 8;

/// Production usage set: render target during generation, sampled afterwards.
/// Deliberately no `COPY_DST` and no `STORAGE_BINDING`.
pub(crate) const PHYSICAL_TEXTURE_USAGE: wgpu::TextureUsages =
    wgpu::TextureUsages::from_bits_truncate(
        wgpu::TextureUsages::TEXTURE_BINDING.bits() | wgpu::TextureUsages::RENDER_ATTACHMENT.bits(),
    );

/// Reference observer altitude baked into the sky-view LUT and the environment
/// cubemap. The LUTs are generated once, so they cannot follow the per-frame
/// camera altitude; 2 m matches the historic presentation reference altitude.
pub(crate) const OBSERVER_ALTITUDE_M: f32 = 2.0;

/// One persistent physical texture in the RV2-5 plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PhysicalTexturePlan {
    pub(crate) label: &'static str,
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) depth_or_array_layers: u32,
    pub(crate) mip_level_count: u32,
    pub(crate) format: wgpu::TextureFormat,
    pub(crate) usage: wgpu::TextureUsages,
    pub(crate) cube: bool,
}

/// Deterministic, persistent (never resize-dependent) RV2-5 texture plan.
#[must_use]
pub(crate) fn physical_texture_plans() -> [PhysicalTexturePlan; 7] {
    [
        PhysicalTexturePlan {
            label: "RV2-5 physical transmittance LUT",
            width: TRANSMITTANCE_SIZE.0,
            height: TRANSMITTANCE_SIZE.1,
            depth_or_array_layers: 1,
            mip_level_count: 1,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: false,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical multi-scattering LUT",
            width: MULTI_SCATTERING_SIZE.0,
            height: MULTI_SCATTERING_SIZE.1,
            depth_or_array_layers: 1,
            mip_level_count: 1,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: false,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical sky-view LUT",
            width: SKY_VIEW_SIZE.0,
            height: SKY_VIEW_SIZE.1,
            depth_or_array_layers: 1,
            mip_level_count: 1,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: false,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical environment cube",
            width: ENVIRONMENT_CUBE_SIZE,
            height: ENVIRONMENT_CUBE_SIZE,
            depth_or_array_layers: CUBE_FACE_COUNT,
            mip_level_count: ENVIRONMENT_CUBE_MIP_COUNT,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: true,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical diffuse irradiance cube",
            width: IRRADIANCE_CUBE_SIZE,
            height: IRRADIANCE_CUBE_SIZE,
            depth_or_array_layers: CUBE_FACE_COUNT,
            mip_level_count: 1,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: true,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical prefiltered specular cube",
            width: SPECULAR_CUBE_SIZE,
            height: SPECULAR_CUBE_SIZE,
            depth_or_array_layers: CUBE_FACE_COUNT,
            mip_level_count: SPECULAR_MIP_COUNT,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: true,
        },
        PhysicalTexturePlan {
            label: "RV2-5 physical BRDF integration LUT",
            width: BRDF_LUT_SIZE.0,
            height: BRDF_LUT_SIZE.1,
            depth_or_array_layers: 1,
            mip_level_count: 1,
            format: PHYSICAL_TEXTURE_FORMAT,
            usage: PHYSICAL_TEXTURE_USAGE,
            cube: false,
        },
    ]
}

/// Total texel footprint of the RV2-5 physical resources, in bytes.
///
/// Counts the environment cubemap base level only and all eight prefiltered
/// specular mips, i.e. the ~2.68 MiB plan (never "~2 MiB").
#[must_use]
pub(crate) fn physical_environment_bytes() -> u64 {
    physical_texture_plans()
        .iter()
        .map(|plan| {
            let mut texels = 0u64;
            for mip in 0..plan.mip_level_count {
                let width = u64::from((plan.width >> mip).max(1));
                let height = u64::from((plan.height >> mip).max(1));
                texels += width * height * u64::from(plan.depth_or_array_layers);
            }
            texels * RGBA16F_BYTES_PER_TEXEL
        })
        .sum()
}

/// Ordered GPU initialization passes recorded into the single command encoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum InitializationPass {
    TransmittanceLut,
    MultiScatteringLut,
    SkyViewLut,
    EnvironmentCube,
    DiffuseIrradianceCube,
    PrefilteredSpecularCube,
    BrdfIntegrationLut,
}

/// Mandatory generation order; each entry depends only on earlier entries.
pub(crate) const INITIALIZATION_PASSES: [InitializationPass; 7] = [
    InitializationPass::TransmittanceLut,
    InitializationPass::MultiScatteringLut,
    InitializationPass::SkyViewLut,
    InitializationPass::EnvironmentCube,
    InitializationPass::DiffuseIrradianceCube,
    InitializationPass::PrefilteredSpecularCube,
    InitializationPass::BrdfIntegrationLut,
];

/// Explicit physical/fallback selection for the V2 environment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum V2EnvironmentMode {
    Physical,
    AnalyticalFallback,
}

impl V2EnvironmentMode {
    /// Decide the mode from the V2 policy flag and the device-legal capability
    /// probe (never from the adapter-reported snapshot alone).
    #[must_use]
    pub(crate) fn select(v2_feature_policy: bool, device_legal_support: bool) -> Self {
        if v2_feature_policy && device_legal_support {
            Self::Physical
        } else {
            Self::AnalyticalFallback
        }
    }
}

// ---------------------------------------------------------------------------
// Atmosphere parameters
// ---------------------------------------------------------------------------

/// Presentation-only atmosphere parameters expressed in metres and in the
/// physical unit conventions of the Hillaire/Bruneton model.
///
/// The preset follows the wavelength-based Earth defaults of the public
/// reference implementation; no coefficient is tuned per channel for looks.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AtmosphereParameters {
    pub(crate) planet_radius_m: f32,
    pub(crate) atmosphere_height_m: f32,
    pub(crate) rayleigh_scattering: [f32; 3],
    pub(crate) rayleigh_scale_height_m: f32,
    pub(crate) mie_scattering: [f32; 3],
    pub(crate) mie_extinction: [f32; 3],
    pub(crate) mie_scale_height_m: f32,
    pub(crate) mie_anisotropy: f32,
    pub(crate) ozone_absorption: [f32; 3],
    pub(crate) ground_albedo: [f32; 3],
}

impl AtmosphereParameters {
    /// Earth-like outdoor preset: Rayleigh/Mie/ozone coefficients in per metre
    /// and the standard scale heights (Rayleigh 8 km, Mie 1.2 km, ozone tent
    /// centred on 25 km) over a 60 km shell.
    pub(crate) const fn earth() -> Self {
        Self {
            planet_radius_m: 6_360_000.0,
            atmosphere_height_m: 60_000.0,
            rayleigh_scattering: [5.802e-6, 13.558e-6, 33.100e-6],
            rayleigh_scale_height_m: 8_000.0,
            mie_scattering: [3.996e-6; 3],
            mie_extinction: [4.440e-6; 3],
            mie_scale_height_m: 1_200.0,
            mie_anisotropy: 0.76,
            ozone_absorption: [0.000_000_65, 0.000_001_88, 0.000_000_08],
            ground_albedo: [0.20, 0.22, 0.18],
        }
    }

    /// Deterministic rejection of non-physical parameters.
    ///
    /// The result is never discarded: [`create_physical_environment`] rejects
    /// invalid input *before* the first texture or pipeline is created.
    pub(crate) fn validate(self) -> Result<(), &'static str> {
        let scalars = [
            self.planet_radius_m,
            self.atmosphere_height_m,
            self.rayleigh_scale_height_m,
            self.mie_scale_height_m,
            self.mie_anisotropy,
        ];
        if scalars.iter().any(|value| !value.is_finite()) {
            return Err("atmosphere scalar parameters must be finite");
        }
        if self
            .rayleigh_scattering
            .iter()
            .chain(self.mie_scattering.iter())
            .chain(self.mie_extinction.iter())
            .chain(self.ozone_absorption.iter())
            .chain(self.ground_albedo.iter())
            .any(|value| !value.is_finite())
        {
            return Err("atmosphere spectral coefficients must be finite");
        }
        if self.planet_radius_m <= 0.0 {
            return Err("planet radius must be positive");
        }
        if self.atmosphere_height_m <= 0.0 {
            return Err("atmosphere height must be positive");
        }
        if self.rayleigh_scale_height_m <= 0.0 || self.mie_scale_height_m <= 0.0 {
            return Err("scale heights must be positive");
        }
        if self.rayleigh_scale_height_m >= self.atmosphere_height_m
            || self.mie_scale_height_m >= self.atmosphere_height_m
        {
            return Err("scale heights must be shorter than the atmosphere shell");
        }
        if !(0.0..1.0).contains(&self.mie_anisotropy) {
            return Err("Mie anisotropy must lie in [0, 1)");
        }
        if self
            .rayleigh_scattering
            .iter()
            .chain(self.mie_scattering.iter())
            .chain(self.mie_extinction.iter())
            .chain(self.ozone_absorption.iter())
            .any(|value| *value < 0.0)
        {
            return Err("scattering and absorption coefficients must not be negative");
        }
        if self
            .ground_albedo
            .iter()
            .any(|value| !(0.0..=1.0).contains(value))
        {
            return Err("ground albedo must lie in [0, 1]");
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Sun
// ---------------------------------------------------------------------------

/// The single V2 sun description.
///
/// One direction drives the physical sky, the sun disk, direct PBR lighting,
/// all three shadow cascades and the environment generation; one
/// `radiance`/transmittance pair drives the sky, the environment and the direct
/// PBR response, so the sky can never diverge from the lighting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SunState {
    /// Normalized world-space direction *towards* the sun.
    pub(crate) direction: [f32; 3],
    /// Angular **radius** of the solar disk in radians (not the diameter).
    pub(crate) angular_radius_radians: f32,
    /// Scene-referred radiance of the solar disk.
    pub(crate) radiance: [f32; 3],
    /// Atmospheric attenuation along the sun direction at the observer.
    pub(crate) transmittance: [f32; 3],
}

impl SunState {
    /// Mean angular radius of the solar disk seen from Earth, in radians
    /// (0.266 degrees half-angle). Authoritative source: NASA Sun fact sheet
    /// ("the Sun's angular diameter is about 0.53 degrees").
    pub(crate) const EARTH_ANGULAR_RADIUS_RADIANS: f32 = 0.004_65;

    /// Build a sun from a direction and the scene-referred radiance of its
    /// disk. The direction is normalized; a degenerate input falls back to
    /// straight up so every derived value stays finite.
    pub(crate) fn from_direction(direction: [f32; 3], radiance: [f32; 3]) -> Self {
        Self {
            direction: normalize3(direction),
            angular_radius_radians: Self::EARTH_ANGULAR_RADIUS_RADIANS,
            radiance,
            transmittance: [1.0; 3],
        }
    }

    /// Solid angle of the solar disk (small-angle approximation).
    #[must_use]
    pub(crate) fn solid_angle(&self) -> f32 {
        PI * self.angular_radius_radians * self.angular_radius_radians
    }

    /// Irradiance delivered by the disk: `radiance * solid angle`.
    #[must_use]
    pub(crate) fn irradiance(&self) -> [f32; 3] {
        let solid_angle = self.solid_angle();
        std::array::from_fn(|channel| self.radiance[channel] * solid_angle)
    }

    /// Build a sun whose *irradiance* is the requested value.
    #[must_use]
    pub(crate) fn from_irradiance(
        direction: [f32; 3],
        irradiance: [f32; 3],
        angular_radius_radians: f32,
    ) -> Self {
        let solid_angle = PI * angular_radius_radians * angular_radius_radians;
        let mut sun = Self::from_direction(direction, [0.0; 3]);
        sun.angular_radius_radians = angular_radius_radians;
        sun.radiance = std::array::from_fn(|channel| irradiance[channel] / solid_angle);
        sun
    }

    /// Deterministic rejection of a malformed sun state.
    pub(crate) fn validate(self) -> Result<(), &'static str> {
        if self.direction.iter().any(|value| !value.is_finite())
            || self.radiance.iter().any(|value| !value.is_finite())
            || !self.angular_radius_radians.is_finite()
        {
            return Err("sun state must be finite");
        }
        let length = (self.direction[0] * self.direction[0]
            + self.direction[1] * self.direction[1]
            + self.direction[2] * self.direction[2])
            .sqrt();
        if (length - 1.0).abs() > 1e-3 {
            return Err("sun direction must be normalized");
        }
        if self.angular_radius_radians <= 0.0 || self.angular_radius_radians > 0.1 {
            return Err("sun angular radius must lie in (0, 0.1] radians");
        }
        if self.radiance.iter().any(|value| *value < 0.0) {
            return Err("sun radiance must not be negative");
        }
        Ok(())
    }

    /// Fold the clear-air attenuation along the sun direction into the state.
    #[must_use]
    pub(crate) fn with_atmosphere_transmittance(
        mut self,
        parameters: AtmosphereParameters,
    ) -> Self {
        self.transmittance = clear_air_transmittance(parameters, self.direction[1]);
        self
    }
}

fn normalize3(direction: [f32; 3]) -> [f32; 3] {
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    if length > f32::EPSILON && length.is_finite() {
        [
            direction[0] / length,
            direction[1] / length,
            direction[2] / length,
        ]
    } else {
        [0.0, 1.0, 0.0]
    }
}

/// Analytic single-ray transmittance from the ground to the top of the shell
/// along an elevation cosine `mu`, used to seed [`SunState::transmittance`].
#[must_use]
pub(crate) fn clear_air_transmittance(parameters: AtmosphereParameters, mu: f32) -> [f32; 3] {
    const STEPS: i32 = 64;
    let r0 = parameters.planet_radius_m;
    let top = parameters.planet_radius_m + parameters.atmosphere_height_m;
    let cosine = mu.clamp(-1.0, 1.0);
    let discriminant = r0 * r0 * (cosine * cosine - 1.0) + top * top;
    let distance = if discriminant <= 0.0 {
        0.0
    } else {
        (-r0 * cosine + discriminant.sqrt()).max(0.0)
    };
    let dt = distance / STEPS as f32;
    let mut rayleigh = 0.0;
    let mut mie = 0.0;
    let mut ozone = 0.0;
    for step in 0..STEPS {
        let t = (step as f32 + 0.5) * dt;
        let radius = (r0 * r0 + 2.0 * r0 * cosine * t + t * t).max(0.0).sqrt();
        let height = (radius - r0).max(0.0);
        rayleigh += (-height / parameters.rayleigh_scale_height_m).exp() * dt;
        mie += (-height / parameters.mie_scale_height_m).exp() * dt;
        ozone += (1.0 - (height - 25_000.0).abs() / 15_000.0).max(0.0) * dt;
    }
    std::array::from_fn(|channel| {
        let optical_depth = parameters.rayleigh_scattering[channel] * rayleigh
            + parameters.mie_extinction[channel] * mie
            + parameters.ozone_absorption[channel] * ozone;
        (-optical_depth).exp()
    })
}

// ---------------------------------------------------------------------------
// GPU generation
// ---------------------------------------------------------------------------

/// Generation-pass uniform mirroring the WGSL `AtmosphereUniform`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct AtmosphereUniformRaw {
    planet_radius_m: f32,
    atmosphere_height_m: f32,
    rayleigh_scale_height_m: f32,
    mie_scale_height_m: f32,
    mie_anisotropy: f32,
    sun_angular_radius_radians: f32,
    observer_altitude_m: f32,
    _padding: f32,
    sun_direction: [f32; 4],
    sun_irradiance: [f32; 4],
    rayleigh_scattering: [f32; 4],
    mie_scattering: [f32; 4],
    mie_extinction: [f32; 4],
    ozone_absorption: [f32; 4],
    ground_albedo: [f32; 4],
}

/// Per-mip prefilter uniform mirroring the WGSL `PrefilterUniform`.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
struct PrefilterUniformRaw {
    roughness: f32,
    source_resolution: f32,
    _padding: [f32; 2],
}

/// Persistent physical environment resources.
///
/// The scene pass samples the cube views; the owned textures keep every view
/// and the underlying allocation alive for the whole renderer life. Nothing
/// here is ever recreated on resize and nothing allocates per frame.
pub(crate) struct EnvironmentTextures {
    pub(crate) transmittance: wgpu::TextureView,
    pub(crate) multi_scattering: wgpu::TextureView,
    pub(crate) sky_view: wgpu::TextureView,
    pub(crate) environment_cube: wgpu::TextureView,
    pub(crate) irradiance_cube: wgpu::TextureView,
    pub(crate) prefiltered_cube: wgpu::TextureView,
    pub(crate) brdf_lut: wgpu::TextureView,
    pub(crate) cube_sampler: wgpu::Sampler,
    pub(crate) lut_sampler: wgpu::Sampler,
    _textures: Vec<wgpu::Texture>,
}

#[allow(dead_code)]
pub(crate) type AtmosphereResources = EnvironmentTextures;

fn texture_entry(
    binding: u32,
    view_dimension: wgpu::TextureViewDimension,
) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension,
            multisampled: false,
        },
        count: None,
    }
}

fn sampler_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
        count: None,
    }
}

fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Buffer {
            ty: wgpu::BufferBindingType::Uniform,
            has_dynamic_offset: false,
            min_binding_size: None,
        },
        count: None,
    }
}

fn create_layout(
    device: &wgpu::Device,
    label: &'static str,
    entries: &[wgpu::BindGroupLayoutEntry],
) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some(label),
        entries,
    })
}

fn create_fullscreen_pipeline(
    device: &wgpu::Device,
    shader: &wgpu::ShaderModule,
    layout: &wgpu::PipelineLayout,
    label: &'static str,
    fragment_entry_point: &'static str,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_fullscreen"),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            buffers: &[],
        },
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some(fragment_entry_point),
            compilation_options: wgpu::PipelineCompilationOptions::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: PHYSICAL_TEXTURE_FORMAT,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    })
}

fn create_fullscreen_pipeline_layout(
    device: &wgpu::Device,
    label: &'static str,
    layout: &wgpu::BindGroupLayout,
) -> wgpu::PipelineLayout {
    device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some(label),
        bind_group_layouts: &[Some(layout)],
        immediate_size: 0,
    })
}

/// Record one fullscreen-triangle pass.
///
/// `first_instance`/`instances` drive `@builtin(instance_index)` in the vertex
/// shader, which is the cube-face selector of every cube pass: face `N` must
/// be drawn as the single-instance range `N..N+1` so the shader evaluates
/// `cube_direction(N, uv)`. The `D2` array-layer view only decides *where* the
/// texels are written, never *which* direction is evaluated, so the two must
/// stay in lock-step. 2D passes use `first_instance = 0, instances = 1`.
fn render_fullscreen(
    encoder: &mut wgpu::CommandEncoder,
    label: &'static str,
    view: &wgpu::TextureView,
    pipeline: &wgpu::RenderPipeline,
    bind_group: &wgpu::BindGroup,
    first_instance: u32,
    instances: u32,
) {
    let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some(label),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
            view,
            depth_slice: None,
            resolve_target: None,
            ops: wgpu::Operations {
                load: wgpu::LoadOp::Clear(wgpu::Color {
                    r: 0.0,
                    g: 0.0,
                    b: 0.0,
                    a: 0.0,
                }),
                store: wgpu::StoreOp::Store,
            },
        })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    });
    pass.set_pipeline(pipeline);
    pass.set_bind_group(0, bind_group, &[]);
    pass.draw(0..3, first_instance..first_instance + instances);
}

/// Per-face (and per-mip) `D2` render view of one cube face.
fn cube_face_view(texture: &wgpu::Texture, mip: u32, face: u32) -> wgpu::TextureView {
    texture.create_view(&wgpu::TextureViewDescriptor {
        label: Some("RV2-5 physical cube face render view"),
        format: None,
        dimension: Some(wgpu::TextureViewDimension::D2),
        usage: None,
        aspect: wgpu::TextureAspect::All,
        base_mip_level: mip,
        mip_level_count: Some(1),
        base_array_layer: face,
        array_layer_count: Some(1),
    })
}

/// `Cube` sampling view of a cubemap layer array.
fn cube_sampling_view(texture: &wgpu::Texture) -> wgpu::TextureView {
    texture.create_view(&wgpu::TextureViewDescriptor {
        label: Some("RV2-5 physical cube sampling view"),
        format: None,
        dimension: Some(wgpu::TextureViewDimension::Cube),
        usage: None,
        aspect: wgpu::TextureAspect::All,
        base_mip_level: 0,
        mip_level_count: None,
        base_array_layer: 0,
        array_layer_count: Some(CUBE_FACE_COUNT),
    })
}

/// Generate every persistent physical atmosphere/IBL resource on the GPU.
///
/// The parameters and the sun are validated first and rejected deterministically
/// before any resource exists. All passes are recorded into one command encoder
/// and submitted exactly once.
///
/// # Errors
///
/// Returns the validation error string when the parameters or the sun state are
/// not physically usable.
pub(crate) fn create_physical_environment(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    parameters: AtmosphereParameters,
    sun: SunState,
) -> Result<EnvironmentTextures, &'static str> {
    create_physical_environment_with_usage(
        device,
        queue,
        parameters,
        sun,
        wgpu::TextureUsages::empty(),
    )
}

/// As [`create_physical_environment`], but with extra texture usages.
///
/// Production always passes [`wgpu::TextureUsages::empty`], so the physical
/// resources keep exactly `RENDER_ATTACHMENT | TEXTURE_BINDING`. The ignored
/// GPU diagnostics test adds `COPY_SRC` through this seam only, which is why
/// the production usage set itself stays free of copy usages.
pub(crate) fn create_physical_environment_with_usage(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    parameters: AtmosphereParameters,
    sun: SunState,
    extra_usage: wgpu::TextureUsages,
) -> Result<EnvironmentTextures, &'static str> {
    parameters.validate()?;
    sun.validate()?;

    let mut textures = Vec::with_capacity(physical_texture_plans().len());
    for plan in physical_texture_plans() {
        textures.push(device.create_texture(&wgpu::TextureDescriptor {
            label: Some(plan.label),
            size: wgpu::Extent3d {
                width: plan.width,
                height: plan.height,
                depth_or_array_layers: plan.depth_or_array_layers,
            },
            mip_level_count: plan.mip_level_count,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: plan.format,
            usage: plan.usage | extra_usage,
            view_formats: &[],
        }));
    }

    let transmittance = &textures[0];
    let multi_scattering = &textures[1];
    let sky_view = &textures[2];
    let environment_cube = &textures[3];
    let irradiance_cube = &textures[4];
    let prefiltered_cube = &textures[5];
    let brdf_lut = &textures[6];

    let transmittance_view = transmittance.create_view(&wgpu::TextureViewDescriptor::default());
    let multi_scattering_view =
        multi_scattering.create_view(&wgpu::TextureViewDescriptor::default());
    let sky_view_view = sky_view.create_view(&wgpu::TextureViewDescriptor::default());
    let brdf_lut_view = brdf_lut.create_view(&wgpu::TextureViewDescriptor::default());
    let environment_cube_view = cube_sampling_view(environment_cube);
    let irradiance_cube_view = cube_sampling_view(irradiance_cube);
    let prefiltered_cube_view = cube_sampling_view(prefiltered_cube);

    let environment_face_views: Vec<wgpu::TextureView> = (0..CUBE_FACE_COUNT)
        .map(|face| cube_face_view(environment_cube, 0, face))
        .collect();
    let irradiance_face_views: Vec<wgpu::TextureView> = (0..CUBE_FACE_COUNT)
        .map(|face| cube_face_view(irradiance_cube, 0, face))
        .collect();
    let prefiltered_face_views: Vec<wgpu::TextureView> = (0..SPECULAR_MIP_COUNT)
        .flat_map(|mip| {
            (0..CUBE_FACE_COUNT).map(move |face| cube_face_view(prefiltered_cube, mip, face))
        })
        .collect();

    let lut_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("RV2-5 physical atmosphere LUT sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });
    let cube_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("RV2-5 physical environment cube sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });

    // --- Atmosphere generation pipelines ---------------------------------
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("RV2-5 physical atmosphere generation shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("atmosphere.wgsl").into()),
    });

    let uniform_only_layout = create_layout(
        device,
        "RV2-5 atmosphere uniform layout",
        &[uniform_entry(0)],
    );
    let transmittance_input_layout = create_layout(
        device,
        "RV2-5 atmosphere transmittance+sampler layout",
        &[
            uniform_entry(0),
            texture_entry(1, wgpu::TextureViewDimension::D2),
            sampler_entry(4),
        ],
    );
    let atmosphere_lut_layout = create_layout(
        device,
        "RV2-5 atmosphere LUT inputs layout",
        &[
            uniform_entry(0),
            texture_entry(1, wgpu::TextureViewDimension::D2),
            texture_entry(2, wgpu::TextureViewDimension::D2),
            sampler_entry(4),
        ],
    );

    let transmittance_pipeline_layout = create_fullscreen_pipeline_layout(
        device,
        "RV2-5 transmittance pipeline layout",
        &uniform_only_layout,
    );
    let multi_scattering_pipeline_layout = create_fullscreen_pipeline_layout(
        device,
        "RV2-5 multi-scattering pipeline layout",
        &transmittance_input_layout,
    );
    let atmosphere_pipeline_layout = create_fullscreen_pipeline_layout(
        device,
        "RV2-5 sky/environment pipeline layout",
        &atmosphere_lut_layout,
    );

    let transmittance_pipeline = create_fullscreen_pipeline(
        device,
        &shader,
        &transmittance_pipeline_layout,
        "RV2-5 transmittance LUT pipeline",
        "fs_transmittance",
    );
    let multi_scattering_pipeline = create_fullscreen_pipeline(
        device,
        &shader,
        &multi_scattering_pipeline_layout,
        "RV2-5 multi-scattering LUT pipeline",
        "fs_multi_scattering",
    );
    let sky_view_pipeline = create_fullscreen_pipeline(
        device,
        &shader,
        &atmosphere_pipeline_layout,
        "RV2-5 sky-view LUT pipeline",
        "fs_sky_view",
    );
    let environment_pipeline = create_fullscreen_pipeline(
        device,
        &shader,
        &atmosphere_pipeline_layout,
        "RV2-5 environment cube pipeline",
        "fs_environment_cube",
    );

    // --- IBL convolution pipelines ---------------------------------------
    let ibl_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("RV2-5 physical IBL shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("ibl.wgsl").into()),
    });
    let irradiance_layout = create_layout(
        device,
        "RV2-5 irradiance convolution layout",
        &[
            texture_entry(0, wgpu::TextureViewDimension::Cube),
            sampler_entry(2),
        ],
    );
    let prefilter_layout = create_layout(
        device,
        "RV2-5 specular prefilter layout",
        &[
            texture_entry(0, wgpu::TextureViewDimension::Cube),
            uniform_entry(1),
            sampler_entry(2),
        ],
    );
    let brdf_layout = create_layout(device, "RV2-5 BRDF integration layout", &[]);

    let irradiance_pipeline_layout = create_fullscreen_pipeline_layout(
        device,
        "RV2-5 irradiance pipeline layout",
        &irradiance_layout,
    );
    let prefilter_pipeline_layout = create_fullscreen_pipeline_layout(
        device,
        "RV2-5 prefilter pipeline layout",
        &prefilter_layout,
    );
    let brdf_pipeline_layout =
        create_fullscreen_pipeline_layout(device, "RV2-5 BRDF pipeline layout", &brdf_layout);

    let irradiance_pipeline = create_fullscreen_pipeline(
        device,
        &ibl_shader,
        &irradiance_pipeline_layout,
        "RV2-5 diffuse irradiance pipeline",
        "fs_irradiance_cube",
    );
    let prefilter_pipeline = create_fullscreen_pipeline(
        device,
        &ibl_shader,
        &prefilter_pipeline_layout,
        "RV2-5 specular prefilter pipeline",
        "fs_prefiltered_cube",
    );
    let brdf_pipeline = create_fullscreen_pipeline(
        device,
        &ibl_shader,
        &brdf_pipeline_layout,
        "RV2-5 BRDF integration pipeline",
        "fs_brdf_lut",
    );

    // --- Uniforms ---------------------------------------------------------
    let sun_irradiance = sun.irradiance();
    let uniform = AtmosphereUniformRaw {
        planet_radius_m: parameters.planet_radius_m,
        atmosphere_height_m: parameters.atmosphere_height_m,
        rayleigh_scale_height_m: parameters.rayleigh_scale_height_m,
        mie_scale_height_m: parameters.mie_scale_height_m,
        mie_anisotropy: parameters.mie_anisotropy,
        sun_angular_radius_radians: sun.angular_radius_radians,
        observer_altitude_m: OBSERVER_ALTITUDE_M,
        _padding: 0.0,
        sun_direction: [sun.direction[0], sun.direction[1], sun.direction[2], 0.0],
        sun_irradiance: [sun_irradiance[0], sun_irradiance[1], sun_irradiance[2], 0.0],
        rayleigh_scattering: [
            parameters.rayleigh_scattering[0],
            parameters.rayleigh_scattering[1],
            parameters.rayleigh_scattering[2],
            0.0,
        ],
        mie_scattering: [
            parameters.mie_scattering[0],
            parameters.mie_scattering[1],
            parameters.mie_scattering[2],
            0.0,
        ],
        mie_extinction: [
            parameters.mie_extinction[0],
            parameters.mie_extinction[1],
            parameters.mie_extinction[2],
            0.0,
        ],
        ozone_absorption: [
            parameters.ozone_absorption[0],
            parameters.ozone_absorption[1],
            parameters.ozone_absorption[2],
            0.0,
        ],
        ground_albedo: [
            parameters.ground_albedo[0],
            parameters.ground_albedo[1],
            parameters.ground_albedo[2],
            0.0,
        ],
    };
    let uniform_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("RV2-5 atmosphere generation uniform"),
        contents: bytemuck::bytes_of(&uniform),
        usage: wgpu::BufferUsages::UNIFORM,
    });

    let prefilter_uniform_buffers: Vec<wgpu::Buffer> = (0..SPECULAR_MIP_COUNT)
        .map(|mip| {
            let roughness = mip as f32 / (SPECULAR_MIP_COUNT - 1) as f32;
            let raw = PrefilterUniformRaw {
                roughness,
                source_resolution: SPECULAR_CUBE_SIZE as f32,
                _padding: [0.0; 2],
            };
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("RV2-5 specular prefilter mip uniform"),
                contents: bytemuck::bytes_of(&raw),
                usage: wgpu::BufferUsages::UNIFORM,
            })
        })
        .collect();

    // --- Bind groups ------------------------------------------------------
    let transmittance_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("RV2-5 transmittance bind group"),
        layout: &uniform_only_layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform_buffer.as_entire_binding(),
        }],
    });
    let multi_scattering_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("RV2-5 multi-scattering bind group"),
        layout: &transmittance_input_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&transmittance_view),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::Sampler(&lut_sampler),
            },
        ],
    });
    let atmosphere_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("RV2-5 sky/environment bind group"),
        layout: &atmosphere_lut_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&transmittance_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(&multi_scattering_view),
            },
            wgpu::BindGroupEntry {
                binding: 4,
                resource: wgpu::BindingResource::Sampler(&lut_sampler),
            },
        ],
    });
    let irradiance_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("RV2-5 irradiance bind group"),
        layout: &irradiance_layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&environment_cube_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::Sampler(&cube_sampler),
            },
        ],
    });
    let prefilter_bind_groups: Vec<wgpu::BindGroup> = prefilter_uniform_buffers
        .iter()
        .map(|buffer| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("RV2-5 specular prefilter bind group"),
                layout: &prefilter_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&environment_cube_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: buffer.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::Sampler(&cube_sampler),
                    },
                ],
            })
        })
        .collect();
    let brdf_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("RV2-5 BRDF bind group"),
        layout: &brdf_layout,
        entries: &[],
    });

    // --- Single initialization encoder, submitted once --------------------
    // Mandatory order: transmittance -> multi-scattering -> sky-view ->
    // environment cube -> irradiance -> prefiltered specular -> BRDF.
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("RV2-5 physical atmosphere + IBL initialization encoder"),
    });
    render_fullscreen(
        &mut encoder,
        "RV2-5 transmittance LUT pass",
        &transmittance_view,
        &transmittance_pipeline,
        &transmittance_bind_group,
        0,
        1,
    );
    render_fullscreen(
        &mut encoder,
        "RV2-5 multi-scattering LUT pass",
        &multi_scattering_view,
        &multi_scattering_pipeline,
        &multi_scattering_bind_group,
        0,
        1,
    );
    render_fullscreen(
        &mut encoder,
        "RV2-5 sky-view LUT pass",
        &sky_view_view,
        &sky_view_pipeline,
        &atmosphere_bind_group,
        0,
        1,
    );
    // Cube faces: face `N` renders into the layer-`N` D2 view AND draws the
    // instance range `N..N+1`, so `@builtin(instance_index)` inside the shader
    // equals `N` and `cube_direction(N, uv)` evaluates the correct axis.
    for (face, face_view) in environment_face_views.iter().enumerate() {
        render_fullscreen(
            &mut encoder,
            "RV2-5 environment cube face pass",
            face_view,
            &environment_pipeline,
            &atmosphere_bind_group,
            face as u32,
            1,
        );
    }
    for (face, face_view) in irradiance_face_views.iter().enumerate() {
        render_fullscreen(
            &mut encoder,
            "RV2-5 diffuse irradiance cube face pass",
            face_view,
            &irradiance_pipeline,
            &irradiance_bind_group,
            face as u32,
            1,
        );
    }
    for mip in 0..SPECULAR_MIP_COUNT {
        for face in 0..CUBE_FACE_COUNT {
            let index = (mip * CUBE_FACE_COUNT + face) as usize;
            render_fullscreen(
                &mut encoder,
                "RV2-5 prefiltered specular cube face pass",
                &prefiltered_face_views[index],
                &prefilter_pipeline,
                &prefilter_bind_groups[mip as usize],
                face,
                1,
            );
        }
    }
    render_fullscreen(
        &mut encoder,
        "RV2-5 BRDF integration LUT pass",
        &brdf_lut_view,
        &brdf_pipeline,
        &brdf_bind_group,
        0,
        1,
    );
    queue.submit(std::iter::once(encoder.finish()));

    Ok(EnvironmentTextures {
        transmittance: transmittance_view,
        multi_scattering: multi_scattering_view,
        sky_view: sky_view_view,
        environment_cube: environment_cube_view,
        irradiance_cube: irradiance_cube_view,
        prefiltered_cube: prefiltered_cube_view,
        brdf_lut: brdf_lut_view,
        cube_sampler,
        lut_sampler,
        _textures: textures,
    })
}

#[cfg(test)]
impl EnvironmentTextures {
    /// Test-only access to the owned textures for diagnostic readbacks.
    pub(crate) fn test_texture(&self, index: usize) -> &wgpu::Texture {
        &self._textures[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn earth_sun() -> SunState {
        SunState::from_direction([0.4, 0.8, -0.3], [1.0, 0.95, 0.85])
            .with_atmosphere_transmittance(AtmosphereParameters::earth())
    }

    #[test]
    fn earth_preset_is_valid() {
        assert!(AtmosphereParameters::earth().validate().is_ok());
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        let cases: [(&str, AtmosphereParameters); 8] = [
            (
                "nan",
                AtmosphereParameters {
                    planet_radius_m: f32::NAN,
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "infinite",
                AtmosphereParameters {
                    atmosphere_height_m: f32::INFINITY,
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "negative coefficient",
                AtmosphereParameters {
                    rayleigh_scattering: [-1.0, 1.0, 1.0],
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "negative mie extinction",
                AtmosphereParameters {
                    mie_extinction: [1.0, -2.0, 1.0],
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "zero radius",
                AtmosphereParameters {
                    planet_radius_m: 0.0,
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "zero height",
                AtmosphereParameters {
                    atmosphere_height_m: 0.0,
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "anisotropy out of range",
                AtmosphereParameters {
                    mie_anisotropy: 1.0,
                    ..AtmosphereParameters::earth()
                },
            ),
            (
                "ground albedo out of range",
                AtmosphereParameters {
                    ground_albedo: [0.2, 1.4, 0.2],
                    ..AtmosphereParameters::earth()
                },
            ),
        ];
        for (label, parameters) in cases {
            assert!(parameters.validate().is_err(), "{label} must be rejected");
        }
    }

    #[test]
    fn scale_height_must_be_shorter_than_the_shell() {
        let parameters = AtmosphereParameters {
            rayleigh_scale_height_m: 60_000.0,
            ..AtmosphereParameters::earth()
        };
        assert!(parameters.validate().is_err());
    }

    #[test]
    fn sun_direction_is_normalized() {
        let sun = SunState::from_direction([0.4, 0.8, -0.3], [1.0, 0.95, 0.85]);
        let length =
            (sun.direction[0].powi(2) + sun.direction[1].powi(2) + sun.direction[2].powi(2)).sqrt();
        assert!((length - 1.0).abs() < 1e-6);
        assert!(sun.validate().is_ok());
    }

    #[test]
    fn degenerate_sun_direction_falls_back_to_up_and_stays_finite() {
        let sun = SunState::from_direction([0.0, 0.0, 0.0], [1.0; 3]);
        assert_eq!(sun.direction, [0.0, 1.0, 0.0]);
        assert!(sun.validate().is_ok());
    }

    #[test]
    fn angular_radius_is_the_radius_not_the_diameter() {
        let sun = earth_sun();
        let expected = SunState::EARTH_ANGULAR_RADIUS_RADIANS;
        assert!((sun.angular_radius_radians - expected).abs() < 1e-9);
        // The authoritative angular diameter is ~0.53 degrees, i.e. twice the
        // stored half-angle. Storing the diameter would fail this bound.
        let diameter_reference = 0.53_f32.to_radians();
        assert!(sun.angular_radius_radians < 0.6 * diameter_reference);
        assert!(sun.validate().is_ok());
    }

    #[test]
    fn angular_radius_validation_rejects_out_of_range() {
        let mut sun = earth_sun();
        sun.angular_radius_radians = 0.0;
        assert!(sun.validate().is_err());
        let mut sun = earth_sun();
        sun.angular_radius_radians = 0.5;
        assert!(sun.validate().is_err());
    }

    #[test]
    fn irradiance_is_radiance_times_solid_angle() {
        let sun = earth_sun();
        let irradiance = sun.irradiance();
        let solid_angle = sun.solid_angle();
        for (computed, radiance) in irradiance.iter().zip(sun.radiance.iter()) {
            assert!((computed - radiance * solid_angle).abs() < 1e-6);
        }
    }

    #[test]
    fn resource_plan_dimensions_and_format() {
        let plans = physical_texture_plans();
        assert_eq!(plans[0].width, 256);
        assert_eq!(plans[0].height, 64);
        assert_eq!(plans[1].width, 32);
        assert_eq!(plans[1].height, 32);
        assert_eq!(plans[2].width, 256);
        assert_eq!(plans[2].height, 128);
        assert_eq!((plans[3].width, plans[3].height), (128, 128));
        assert_eq!((plans[4].width, plans[4].height), (32, 32));
        assert_eq!((plans[5].width, plans[5].height), (128, 128));
        assert_eq!(plans[6].width, 256);
        assert_eq!(plans[6].height, 256);
        for plan in plans {
            assert_eq!(
                plan.format,
                wgpu::TextureFormat::Rgba16Float,
                "{}",
                plan.label
            );
        }
    }

    #[test]
    fn cube_faces_and_prefilter_mip_count_are_exact() {
        let plans = physical_texture_plans();
        assert_eq!(plans[3].depth_or_array_layers, 6);
        assert_eq!(plans[3].mip_level_count, 1);
        assert!(plans[3].cube);
        assert_eq!(plans[4].depth_or_array_layers, 6);
        assert!(plans[4].cube);
        assert_eq!(plans[5].depth_or_array_layers, 6);
        assert_eq!(plans[5].mip_level_count, 8);
        assert!(plans[5].cube);
        // 8 mips on a 128 texel cube is the full chain (log2(128) + 1).
        assert_eq!(128u32.trailing_zeros() + 1, SPECULAR_MIP_COUNT);
    }

    #[test]
    fn usages_are_exactly_render_attachment_and_texture_binding() {
        let expected =
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        for plan in physical_texture_plans() {
            assert_eq!(plan.usage, expected, "{}", plan.label);
            assert!(
                !plan.usage.contains(wgpu::TextureUsages::COPY_DST),
                "{} must not carry COPY_DST",
                plan.label
            );
            assert!(
                !plan.usage.contains(wgpu::TextureUsages::STORAGE_BINDING),
                "{} must not be a storage texture",
                plan.label
            );
        }
    }

    #[test]
    fn memory_report_counts_all_eight_mips_and_is_not_two_mib() {
        let bytes = physical_environment_bytes();
        let mib = bytes as f64 / (1024.0 * 1024.0);
        assert!(
            (2.6..2.75).contains(&mib),
            "expected the ~2.68 MiB plan, got {mib:.4} MiB"
        );
        assert!(mib > 2.5, "the plan must not be reported as ~2 MiB");
    }

    #[test]
    fn initialization_order_is_the_mandatory_generation_order() {
        assert_eq!(
            INITIALIZATION_PASSES,
            [
                InitializationPass::TransmittanceLut,
                InitializationPass::MultiScatteringLut,
                InitializationPass::SkyViewLut,
                InitializationPass::EnvironmentCube,
                InitializationPass::DiffuseIrradianceCube,
                InitializationPass::PrefilteredSpecularCube,
                InitializationPass::BrdfIntegrationLut,
            ]
        );
    }

    #[test]
    fn mode_selection_requires_v2_policy_and_device_legal_support() {
        assert_eq!(
            V2EnvironmentMode::select(true, true),
            V2EnvironmentMode::Physical
        );
        assert_eq!(
            V2EnvironmentMode::select(true, false),
            V2EnvironmentMode::AnalyticalFallback
        );
        assert_eq!(
            V2EnvironmentMode::select(false, true),
            V2EnvironmentMode::AnalyticalFallback
        );
        assert_eq!(
            V2EnvironmentMode::select(false, false),
            V2EnvironmentMode::AnalyticalFallback
        );
    }

    #[test]
    fn resources_are_persistent_and_never_resize_dependent() {
        // The plan has no frame-driven or surface-size input: it is a pure
        // constant function of the RV2-5 specification.
        assert_eq!(physical_texture_plans(), physical_texture_plans());
        let plans = physical_texture_plans();
        assert!(!plans.iter().any(|plan| plan.mip_level_count == 0));
        assert!(plans.iter().all(|plan| plan.depth_or_array_layers >= 1));
    }

    /// The production half of this module (everything before the test-only
    /// items), so source guards never match their own literals. CRLF is
    /// normalised because `cargo fmt` writes native line endings on Windows.
    fn production_source() -> String {
        include_str!("atmosphere.rs")
            .replace("\r\n", "\n")
            .split("#[cfg(test)]\nimpl EnvironmentTextures")
            .next()
            .expect("the production source must precede the test-only items")
            .to_owned()
    }

    #[test]
    fn cpu_generation_path_is_gone() {
        let source = production_source();
        for forbidden in [
            "generate_transmittance",
            "generate_multi_scattering",
            "generate_sky_view",
            "generate_environment_cube",
            "generate_irradiance_cube",
            "generate_prefiltered_cube",
            "generate_brdf_lut",
            "f32_to_f16",
            "copy_buffer_to_texture",
            "create_buffer_init(&wgpu::util::BufferInitDescriptor {\n        label: Some(\"RV2 atmosphere initialization staging\")",
        ] {
            assert!(
                !source.contains(forbidden),
                "CPU generation artefact `{forbidden}` must be removed from production"
            );
        }
        assert!(
            !source.contains("Vec<[f32; 4]>"),
            "the CPU pixel buffers must not exist any more"
        );
    }

    #[test]
    fn validation_happens_before_the_first_resource_is_created() {
        let source = production_source();
        let validation = source
            .find("parameters.validate()?;")
            .expect("the atmosphere parameters must be validated");
        let first_texture = source
            .find("device.create_texture(")
            .expect("textures must be created on the GPU");
        assert!(
            validation < first_texture,
            "invalid parameters must be rejected before resource creation"
        );
        assert!(
            !source.contains("let _ = parameters.validate();"),
            "validate() must never be discarded"
        );
    }

    #[test]
    fn generation_lives_in_v2_specific_wgsl_modules() {
        let atmosphere = include_str!("atmosphere.wgsl");
        let ibl = include_str!("ibl.wgsl");
        for entry in [
            "fs_transmittance",
            "fs_multi_scattering",
            "fs_sky_view",
            "fs_environment_cube",
        ] {
            assert!(atmosphere.contains(entry), "{entry} must exist");
        }
        for entry in ["fs_irradiance_cube", "fs_prefiltered_cube", "fs_brdf_lut"] {
            assert!(ibl.contains(entry), "{entry} must exist");
        }
        // The physical model must not contain the retired heuristics.
        for forbidden in ["powf(0.35", "* 8.0", "* 14.0", "0.015 * horizon"] {
            assert!(
                !atmosphere.contains(forbidden),
                "heuristic `{forbidden}` must not return"
            );
        }
        // Multi-scattering must be a real spherical integration closed with
        // the Hillaire (2020, Eq. 7-8) energy-compensation series: TWO
        // INDEPENDENT integrals, the unit-source second-order transfer
        // `l_2nd_order` and the medium-transfer ratio `f_ms`.
        assert!(atmosphere.contains("multi_scattering_texel"));
        assert!(
            atmosphere.contains("var l_2nd_order = vec3<f32>(0.0);"),
            "L_2ndOrder must have its own accumulator"
        );
        assert!(
            atmosphere.contains("var f_ms_sum = vec3<f32>(0.0);"),
            "f_ms must have its own accumulator"
        );
        assert!(
            atmosphere.contains("f_ms_sum = f_ms_sum + ms_transfer_integral(r0, mu);"),
            "f_ms must be a distinct spherical integral, never a function of l_2nd_order"
        );
        assert!(
            atmosphere.contains(
                "single_scattering(r0, mu, direction, sun_direction, true, UNIT_IRRADIANCE)"
            ),
            "L_2ndOrder must integrate the single scattering against the unit source E_I = 1"
        );
        assert!(
            atmosphere.contains(
                "let f_ms = clamp(f_ms_sum / f32(SPHERE_SAMPLES), vec3<f32>(0.0), vec3<f32>(0.999));"
            ),
            "f_ms must be the sphere-averaged transfer integral, clamped into [0, 1) before the series"
        );
        assert!(
            atmosphere.contains("return l_2nd_order / (1.0 - f_ms);"),
            "Psi_ms = L_2ndOrder / (1 - f_ms)"
        );
        // The luminance-ratio shortcut (f_ms derived from L_2ndOrder) and the
        // pre-Hillaire scalar series factor must never come back.
        for forbidden in ["luminance3", "sun_luminance", "f_ms_factor"] {
            assert!(
                !atmosphere.contains(forbidden),
                "the forbidden closure term `{forbidden}` must not exist"
            );
        }
        // The ground albedo participates through the ground-bounce radiance,
        // never as the series-denominator term.
        assert!(
            atmosphere.contains(
                "ground_reflection(r0, mu, direction, sun_direction, ground_distance, UNIT_IRRADIANCE)"
            ),
            "the ground-bounce component must feed l_2nd_order with E_I = 1"
        );
        // The retired non-Hillaire closure must not come back: no radiance
        // times ground albedo inside a geometric-series denominator.
        for forbidden in [
            "second_order * albedo / denominator",
            "1.0) - second_order * albedo",
            "second_order",
        ] {
            assert!(
                !atmosphere.contains(forbidden),
                "the pre-Hillaire closure `{forbidden}` must not return"
            );
        }
    }

    #[test]
    fn hillaire_closure_keeps_two_independent_bounded_integrals() {
        let source = include_str!("atmosphere.wgsl").replace("\r\n", "\n");

        // --- Eq. 8 integrand: pure medium transfer -------------------------
        let transfer = source
            .split("fn ms_transfer_integral(")
            .nth(1)
            .expect("the Hillaire Eq. 8 transfer integral must exist");
        // Cut at the function's own closing brace (column 0), so the doc
        // comment of the next function cannot satisfy or fail a guard.
        let transfer = transfer.split("\n}").next().expect("function body");
        for forbidden in [
            // (2) f_ms must not depend on the sun irradiance,
            "sun_irradiance",
            // (3) f_ms must not use any phase function,
            "phase",
            // (4) f_ms must not use the planet-shadow visibility or any
            // directional-light angular dependence,
            "ray_intersects_ground",
            "sun_direction",
            "transmittance_lookup",
        ] {
            assert!(
                !transfer.contains(forbidden),
                "the f_ms integrand must be medium-only and must not contain `{forbidden}`"
            );
        }
        assert!(
            transfer.contains("let sigma_s = atmosphere.rayleigh_scattering.rgb * rho_rayleigh"),
            "L_f must integrate the volume scattering coefficient σ_s"
        );
        assert!(
            transfer.contains("transfer = transfer + sigma_s * exp(-optical_depth) * dt;"),
            "L_f(x, v) = ∫ σ_s(x) T(x, x - t v) dt with T = exp(-τ)"
        );

        // --- Closure: distinct accumulators, bounded series ----------------
        let body = source
            .split("fn multi_scattering_texel(")
            .nth(1)
            .expect("the multi-scattering closure must exist");
        let body = body.split("\n}").next().expect("function body");
        // (1) Two distinct accumulators fed by two distinct integrals.
        let l2_position = body
            .find("single_scattering(r0, mu, direction, sun_direction, true, UNIT_IRRADIANCE)")
            .expect("the Eq. 7 accumulator must integrate the single scattering");
        let fms_position = body
            .find("f_ms_sum = f_ms_sum + ms_transfer_integral(r0, mu);")
            .expect("the Eq. 8 accumulator must integrate the medium transfer");
        assert_ne!(
            l2_position, fms_position,
            "l_2nd_order and f_ms must be distinct integrals"
        );
        // (7) The LUT is built with E_I = 1: the real SunState irradiance
        // never enters the closure, and f_ms is never derived from
        // l_2nd_order through any luminance ratio.
        assert!(
            !body.contains("sun_irradiance"),
            "the transfer LUT must not bake the real SunState irradiance"
        );
        assert!(
            !body.contains("luminance"),
            "f_ms must not be derived from L_2ndOrder"
        );
        // (5) f_ms is bounded into [0, 1) *before* the series is formed.
        let clamp_position = body
            .find("let f_ms = clamp(")
            .expect("f_ms must be clamped at construction");
        let series_position = body
            .find("(1.0 - f_ms)")
            .expect("the series denominator must exist");
        assert!(
            clamp_position < series_position,
            "f_ms must be bounded before the series denominator is evaluated"
        );
        assert!(
            body.contains("vec3<f32>(0.0), vec3<f32>(0.999)"),
            "f_ms must stay inside the physically valid interval [0, 1), per channel"
        );
        // (6) Psi_ms = L_2ndOrder / (1 - f_ms); no radiance quantity ever
        // enters the geometric-series denominator.
        assert!(
            body.contains("return l_2nd_order / (1.0 - f_ms);"),
            "Psi_ms = L_2ndOrder * F_ms with F_ms = 1 / (1 - f_ms)"
        );
        assert!(
            !body.contains("l_2nd_order * albedo"),
            "no radiance may sit in the geometric-series denominator"
        );

        // (8) The real SunState irradiance is read exactly once in the whole
        // module and applied once per term at consumption, LUT included.
        assert_eq!(
            source.matches("atmosphere.sun_irradiance").count(),
            1,
            "the real SunState irradiance must be accessed exactly once, in sky_radiance"
        );
        let sky = source
            .split("fn sky_radiance(")
            .nth(1)
            .expect("the consumption function must exist");
        let sky = sky.split("\n}").next().expect("function body");
        assert!(
            sky.contains("let sun_irradiance = atmosphere.sun_irradiance.rgb;"),
            "the single irradiance access must live in the consumption path"
        );
        assert!(
            sky.contains(") * sun_irradiance;"),
            "the transfer LUT lookup must be scaled by the real irradiance exactly once"
        );
    }

    #[test]
    fn sun_visibility_is_occluded_by_the_ground_sphere() {
        let source = include_str!("atmosphere.wgsl").replace("\r\n", "\n");
        let predicate = source
            .split("fn ray_intersects_ground(")
            .nth(1)
            .expect("the planet-shadow predicate must exist");
        let predicate = predicate.split("\n}").next().expect("function body");
        // Exact-surface band: the degenerate t = 0 tangent root must not hide
        // the positive exit root, so the band decides directly on the horizon.
        assert!(
            predicate
                .contains("if (sample_radius <= planet_radius * (1.0 + SURFACE_BAND_EPSILON)) {"),
            "the surface band must bypass the degenerate quadratic"
        );
        assert!(
            predicate.contains("return sun_mu < 0.0;"),
            "on the surface the sun is occluded exactly when it is below the horizon"
        );
        assert!(
            predicate
                .contains("return ray_sphere_near(sample_radius, sun_mu, planet_radius) > 0.0;"),
            "above the surface band occlusion must stay the ground-sphere intersection test"
        );
        assert!(
            source.contains("const SURFACE_BAND_EPSILON: f32 = 1e-7;"),
            "the surface band width must be an explicit, documented constant"
        );
        let single = source
            .split("fn single_scattering(")
            .nth(1)
            .expect("single scattering must exist");
        let single = single.split("\n}").next().expect("function body");
        let gate = single
            .find("if (!ray_intersects_ground(radius, sun_mu)) {")
            .expect("the sun transmittance must be gated by the planet shadow");
        let lookup = single
            .find("sun_transmittance = transmittance_lookup(height, sun_mu);")
            .expect("the gated lookup must use the transmittance LUT");
        assert!(
            gate < lookup,
            "the ground-truncated transmittance must never be used as sun visibility"
        );
        assert!(
            single.contains("var sun_transmittance = vec3<f32>(0.0);"),
            "an occluded sun must evaluate to exactly zero visibility"
        );
        assert!(
            !single.contains("let sun_transmittance = transmittance_lookup(height, sun_mu);"),
            "the unconditional pre-fix lookup must be gone"
        );
    }

    /// CPU mirror of the WGSL `ray_sphere_near` scalar quadratic.
    fn mirrored_ray_sphere_near(r0: f32, mu: f32, radius: f32) -> f32 {
        let b = r0 * mu;
        let c = r0 * r0 - radius * radius;
        let discriminant = b * b - c;
        if discriminant < 0.0 {
            return -1.0;
        }
        let root = discriminant.sqrt();
        let near_t = -b - root;
        if near_t >= 0.0 {
            return near_t;
        }
        let far_t = -b + root;
        if far_t >= 0.0 {
            return far_t;
        }
        -1.0
    }

    /// CPU mirror of the WGSL `ray_intersects_ground` predicate, including the
    /// exact-surface band where the t = 0 tangent root degenerates the
    /// quadratic and the decision is taken directly on the horizon.
    fn mirrored_ray_intersects_ground(sample_radius: f32, sun_mu: f32, planet_radius: f32) -> bool {
        const SURFACE_BAND_EPSILON: f32 = 1e-7;
        if sample_radius <= planet_radius * (1.0 + SURFACE_BAND_EPSILON) {
            return sun_mu < 0.0;
        }
        mirrored_ray_sphere_near(sample_radius, sun_mu, planet_radius) > 0.0
    }

    #[test]
    fn ground_sphere_occludes_the_sun_below_the_horizon() {
        let planet = AtmosphereParameters::earth().planet_radius_m;
        // A sample 10 km up with the sun 30 degrees below the local horizon
        // (cos = -0.5) is in the planetary shadow: visibility must be zero.
        let occluded = mirrored_ray_intersects_ground(planet + 10_000.0, -0.5, planet);
        assert!(occluded, "sun below the horizon must be occluded");
        // The same sample with the sun above the horizon sees the sun.
        let visible = mirrored_ray_intersects_ground(planet + 10_000.0, 0.5, planet);
        assert!(!visible, "sun above the horizon must be visible");
        // A sample exactly on the surface must not self-occlude when the sun
        // is at or above the horizon (the tangent solution t = 0 is not an
        // occlusion), but MUST be occluded when the sun is below it: the
        // degenerate c = 0 quadratic has a second root at t = -2 r mu > 0
        // that the old nearest-root test could never see.
        assert!(
            !mirrored_ray_intersects_ground(planet, 0.0, planet),
            "the horizon ray from the surface must not be an occlusion"
        );
        assert!(
            !mirrored_ray_intersects_ground(planet, 0.8, planet),
            "an upward sun ray from the surface must not be an occlusion"
        );
        assert!(
            mirrored_ray_intersects_ground(planet, -0.3, planet),
            "a sun below the horizon from the surface must be occluded"
        );
        assert!(
            mirrored_ray_intersects_ground(planet, -1e-6, planet),
            "any strictly-below-horizon sun from the surface must be occluded"
        );
        // A shallow downward ray from high altitude that geometrically
        // escapes past the horizon is not occluded.
        let high = planet + 55_000.0;
        let tangent_mu = -(1.0 - (planet / high).powi(2)).sqrt();
        assert!(
            !mirrored_ray_intersects_ground(high, tangent_mu * 0.5, planet),
            "a ray above the tangent must escape"
        );
        assert!(
            mirrored_ray_intersects_ground(high, tangent_mu * 1.5, planet),
            "a ray below the tangent must hit the ground sphere"
        );
        // Near-horizon finiteness: the predicate stays a clean boolean on
        // both sides of the tangent, never a NaN-producing configuration.
        for mu in [
            tangent_mu - 1e-6,
            tangent_mu,
            tangent_mu + 1e-6,
            -1e-6,
            0.0,
            1e-6,
        ] {
            let t = mirrored_ray_sphere_near(high, mu, planet);
            assert!(t.is_finite(), "near-horizon intersection must be finite");
            assert!(
                mirrored_ray_intersects_ground(high, mu, planet) == (t > 0.0),
                "the predicate must agree with the intersection distance"
            );
        }
    }

    #[test]
    fn planet_visibility_matches_the_required_edge_case_matrix() {
        let planet = AtmosphereParameters::earth().planet_radius_m;
        // Surface + sun below the horizon => OCCLUDED.
        for sun_mu in [-1.0f32, -0.5, -1e-3, -1e-6] {
            assert!(
                mirrored_ray_intersects_ground(planet, sun_mu, planet),
                "surface + sun_mu {sun_mu} below the horizon must be occluded"
            );
        }
        // Surface + tangent or sun above the horizon => visible.
        for sun_mu in [0.0f32, 1e-6, 0.5, 1.0] {
            assert!(
                !mirrored_ray_intersects_ground(planet, sun_mu, planet),
                "surface + sun_mu {sun_mu} at/above the horizon must be visible"
            );
        }
        // f32 evaluation noise around the exact surface (~0.2 m at Earth
        // scale) stays inside the band and follows the horizon rule.
        let surface_noise = planet * (1.0 - 5e-8);
        assert!(
            mirrored_ray_intersects_ground(surface_noise, -0.2, planet),
            "just inside the surface band the horizon rule must still occlude"
        );
        assert!(
            !mirrored_ray_intersects_ground(surface_noise, 0.2, planet),
            "just inside the surface band an upward sun must stay visible"
        );
        // The production observer altitude (2 m) is outside the band and
        // stays on the exact quadratic branch.
        assert!(
            planet + 2.0 > planet * (1.0 + 1e-7),
            "the 2 m observer must stay on the exact quadratic branch"
        );
        // Altitude + ray above the tangent => visible; below => occluded.
        let altitude = planet + 25_000.0;
        let tangent_mu = -(1.0 - (planet / altitude).powi(2)).sqrt();
        assert!(
            !mirrored_ray_intersects_ground(altitude, tangent_mu * 0.5, planet),
            "altitude + ray above the tangent must be visible"
        );
        assert!(
            mirrored_ray_intersects_ground(altitude, tangent_mu * 1.5, planet),
            "altitude + ray below the tangent must be occluded"
        );
    }

    /// CPU mirror of the WGSL `atmosphere_distance` march length.
    fn mirrored_atmosphere_distance(r0: f32, mu: f32, p: &AtmosphereParameters) -> f32 {
        let top = mirrored_ray_sphere_near(r0, mu, p.planet_radius_m + p.atmosphere_height_m);
        let ground = mirrored_ray_sphere_near(r0, mu, p.planet_radius_m);
        if ground > 0.0 && (top < 0.0 || ground < top) {
            ground
        } else {
            top.max(0.0)
        }
    }

    /// CPU mirror of the WGSL `ms_transfer_integral`: the Hillaire (2020)
    /// Eq. 8 integrand `L_f(x, v) = ∫ σ_s(x) T(x, x - t v) dt`. The mirror
    /// takes NO sun input at all — the medium transfer is independent of the
    /// sun irradiance, of the phase function and of the planet-shadow
    /// visibility by construction, exactly like the WGSL original.
    fn mirrored_ms_transfer(r0: f32, mu: f32, p: &AtmosphereParameters) -> [f32; 3] {
        const STEPS: i32 = 32;
        let distance = mirrored_atmosphere_distance(r0, mu, p);
        let dt = distance / STEPS as f32;
        let mut optical_depth = [0.0f32; 3];
        let mut transfer = [0.0f32; 3];
        for step in 0..STEPS {
            let t = (step as f32 + 0.5) * dt;
            let radius = (r0 * r0 + 2.0 * r0 * mu * t + t * t).max(0.0).sqrt();
            let height = (radius - p.planet_radius_m).max(0.0);
            let rho_rayleigh = (-height / p.rayleigh_scale_height_m).exp();
            let rho_mie = (-height / p.mie_scale_height_m).exp();
            let rho_ozone = (1.0 - (height - 25_000.0).abs() / 15_000.0).max(0.0);
            for channel in 0..3 {
                let sigma_s = p.rayleigh_scattering[channel] * rho_rayleigh
                    + p.mie_scattering[channel] * rho_mie;
                let sigma_t = p.rayleigh_scattering[channel] * rho_rayleigh
                    + p.mie_extinction[channel] * rho_mie
                    + p.ozone_absorption[channel] * rho_ozone;
                optical_depth[channel] += sigma_t * dt;
                transfer[channel] += sigma_s * (-optical_depth[channel]).exp() * dt;
            }
        }
        transfer
    }

    /// CPU mirror of the WGSL `fibonacci_direction` deterministic sphere set.
    fn mirrored_fibonacci_direction(index: i32, count: i32) -> [f32; 3] {
        const GOLDEN_ANGLE: f32 = 2.399_963_1;
        let i = index as f32 + 0.5;
        let cos_theta = 1.0 - 2.0 * i / count as f32;
        let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
        let phi = i * GOLDEN_ANGLE;
        [sin_theta * phi.cos(), cos_theta, sin_theta * phi.sin()]
    }

    #[test]
    fn ms_transfer_sphere_average_is_bounded_below_one() {
        let p = AtmosphereParameters::earth();
        // Sphere-averaged f_ms over the exact 32-sample Fibonacci set of the
        // WGSL generation, for representative LUT rows.
        for height in [0.0f32, 2.0, 12_000.0, 40_000.0] {
            let r0 = p.planet_radius_m + height;
            let mut sum = [0.0f32; 3];
            for index in 0..32 {
                let direction = mirrored_fibonacci_direction(index, 32);
                let mu = direction[1].clamp(-1.0, 1.0);
                let transfer = mirrored_ms_transfer(r0, mu, &p);
                for channel in 0..3 {
                    // Per-direction transfer: finite, non-negative, < 1.
                    assert!(
                        transfer[channel].is_finite() && (0.0..1.0).contains(&transfer[channel]),
                        "L_f must be a bounded medium transfer at {height} m, got {transfer:?}"
                    );
                    sum[channel] += transfer[channel];
                }
            }
            for (channel, channel_sum) in sum.iter().enumerate() {
                let raw = *channel_sum / 32.0;
                let f_ms = raw.clamp(0.0, 0.999);
                // Bounded before the series, and the closed Psi_ms factor
                // (6) is finite and never below one.
                assert!(
                    (0.0..1.0).contains(&f_ms),
                    "f_ms[{channel}] = {raw} at {height} m must stay in [0, 1) before the clamp"
                );
                let factor = 1.0 / (1.0 - f_ms);
                assert!(
                    factor.is_finite() && factor >= 1.0,
                    "the series factor must be finite, got {factor}"
                );
            }
        }
    }

    /// CPU mirror of the WGSL `cube_direction(face, uv)` used by every cube
    /// generation pass, with the same v-down uv convention as the fullscreen
    /// vertex shader.
    fn mirrored_cube_direction(face: u32, uv: (f32, f32)) -> [f32; 3] {
        let u = uv.0 * 2.0 - 1.0;
        let v = uv.1 * 2.0 - 1.0;
        let direction = match face {
            0 => [1.0, -v, -u],
            1 => [-1.0, -v, u],
            2 => [u, 1.0, v],
            3 => [u, -1.0, -v],
            4 => [u, -v, 1.0],
            _ => [-u, -v, -1.0],
        };
        let length = (direction[0] * direction[0]
            + direction[1] * direction[1]
            + direction[2] * direction[2])
            .sqrt();
        [
            direction[0] / length,
            direction[1] / length,
            direction[2] / length,
        ]
    }

    #[test]
    fn cube_face_indices_map_to_the_signed_axis_order() {
        // Face centres of the generation convention: instance_index N must
        // evaluate the N-th signed axis, in the WebGPU cube sampling order.
        let expected: [[f32; 3]; 6] = [
            [1.0, 0.0, 0.0],  // face 0 = +X
            [-1.0, 0.0, 0.0], // face 1 = -X
            [0.0, 1.0, 0.0],  // face 2 = +Y
            [0.0, -1.0, 0.0], // face 3 = -Y
            [0.0, 0.0, 1.0],  // face 4 = +Z
            [0.0, 0.0, -1.0], // face 5 = -Z
        ];
        for face in 0..6u32 {
            let direction = mirrored_cube_direction(face, (0.5, 0.5));
            for axis in 0..3 {
                assert!(
                    (direction[axis] - expected[face as usize][axis]).abs() < 1e-6,
                    "face {face} centre must point along {:?}, got {direction:?}",
                    expected[face as usize]
                );
            }
        }
        // Both cube shaders must carry the identical mapping, so the instance
        // index routes the same axis in generation and in convolution.
        for file in ["atmosphere.wgsl", "ibl.wgsl"] {
            let source = include_str!("atmosphere.wgsl");
            let source = if file == "ibl.wgsl" {
                include_str!("ibl.wgsl")
            } else {
                source
            };
            for axis in [
                "direction = vec3<f32>(1.0, -v, -u);",
                "direction = vec3<f32>(-1.0, -v, u);",
                "direction = vec3<f32>(u, 1.0, v);",
                "direction = vec3<f32>(u, -1.0, -v);",
                "direction = vec3<f32>(u, -v, 1.0);",
                "direction = vec3<f32>(-u, -v, -1.0);",
            ] {
                assert!(
                    source.contains(axis),
                    "{file} must keep the signed-axis cube mapping `{axis}`"
                );
            }
            assert!(
                source.contains("output.face = instance_index;"),
                "{file} must route the face through @builtin(instance_index)"
            );
        }
    }

    #[test]
    fn cube_face_passes_draw_their_own_instance_range() {
        let source = production_source();
        // The single draw must expose the instance range, so face N can be
        // drawn as `N..N+1` and @builtin(instance_index) equals N.
        assert!(
            source.contains("pass.draw(0..3, first_instance..first_instance + instances);"),
            "render_fullscreen must draw an explicit first_instance range"
        );
        assert!(
            !source.contains("pass.draw(0..3, 0..instances)"),
            "a 0-based instance range would collapse every face onto face 0"
        );
        // Environment and irradiance face loops pass the face index as
        // first_instance (single-instance draws).
        assert_eq!(
            source.matches("face as u32,\n").count(),
            2,
            "environment and irradiance face loops must route the face index"
        );
        assert!(
            source.contains("&prefilter_bind_groups[mip as usize],\n                face,"),
            "the prefiltered specular loop must route the face index"
        );
        // The face views and the instance ranges must advance together: the
        // N-th view of each cube is created from layer N.
        assert!(
            source.contains(".map(|face| cube_face_view(environment_cube, 0, face))"),
            "environment face views must be created per layer"
        );
        assert!(
            source.contains("base_array_layer: face,"),
            "a cube face render view must target exactly its own layer"
        );
    }

    #[test]
    fn prefilter_source_is_the_single_mip_environment_cube() {
        // Decision: the environment cube stays at ONE mip (minimal solution);
        // the GGX convolution therefore samples the source at LOD 0 and the
        // fake source-mip heuristic must be gone.
        assert_eq!(ENVIRONMENT_CUBE_MIP_COUNT, 1);
        assert_eq!(physical_texture_plans()[3].mip_level_count, 1);
        let ibl = include_str!("ibl.wgsl").replace("\r\n", "\n");
        assert!(
            ibl.contains("textureSampleLevel(environment_cube, environment_sampler, light, 0.0)"),
            "the specular convolution must sample the source at LOD 0"
        );
        for forbidden in [
            "sample_solid_angle",
            "texel_solid_angle",
            "PREFILTER_MIP_COUNT",
        ] {
            assert!(
                !ibl.contains(forbidden),
                "the source-mip heuristic `{forbidden}` must not exist while the environment cube has one mip"
            );
        }
        // The prefiltered cube itself keeps the full 8-mip roughness chain.
        assert_eq!(SPECULAR_MIP_COUNT, 8);
    }

    #[test]
    fn initialization_records_every_pass_into_one_encoder() {
        let source = production_source();
        let body = source
            .split("let mut encoder = device.create_command_encoder(")
            .nth(1)
            .expect("the initialization encoder must exist");
        let submit = body
            .split("queue.submit(std::iter::once(encoder.finish()));")
            .next()
            .expect("exactly one submit must close the encoder");
        assert_eq!(
            body.matches("queue.submit(").count(),
            1,
            "the initialization must submit exactly once"
        );
        assert_eq!(
            body.matches("device.create_command_encoder(").count(),
            0,
            "no nested encoder"
        );
        for label in [
            "RV2-5 transmittance LUT pass",
            "RV2-5 multi-scattering LUT pass",
            "RV2-5 sky-view LUT pass",
            "RV2-5 environment cube face pass",
            "RV2-5 diffuse irradiance cube face pass",
            "RV2-5 prefiltered specular cube face pass",
            "RV2-5 BRDF integration LUT pass",
        ] {
            assert!(submit.contains(label), "{label} must be recorded");
        }
    }

    fn f16_to_f32(half: u16) -> f32 {
        let sign = if half & 0x8000 != 0 { -1.0 } else { 1.0 };
        let exponent = (half >> 10) & 0x1f;
        let fraction = half & 0x3ff;
        let magnitude = match exponent {
            0 => f32::from(fraction) * 2.0f32.powi(-24),
            0x1f => {
                if fraction == 0 {
                    f32::INFINITY
                } else {
                    f32::NAN
                }
            }
            _ => (1.0 + f32::from(fraction) / 1024.0) * 2.0f32.powi(i32::from(exponent) - 15),
        };
        sign * magnitude
    }

    #[allow(clippy::too_many_arguments)]
    fn read_texture_slice(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        width: u32,
        height: u32,
        mip_level: u32,
        array_layer: u32,
    ) -> Vec<f32> {
        let bytes_per_row = (width * RGBA16F_BYTES_PER_TEXEL as u32).div_ceil(256) * 256;
        let size = u64::from(bytes_per_row) * u64::from(height);
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("RV2-5 diagnostic readback buffer"),
            size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture,
                mip_level,
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
                    bytes_per_row: Some(bytes_per_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(std::iter::once(encoder.finish()));
        let slice = buffer.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("diagnostic readback must complete");
        let mapped = slice.get_mapped_range().expect("readback range must map");
        let halfs: &[u16] = bytemuck::cast_slice(&mapped);
        let values = halfs.iter().copied().map(f16_to_f32).collect();
        drop(mapped);
        buffer.unmap();
        values
    }

    fn read_first_face(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        texture: &wgpu::Texture,
        width: u32,
        height: u32,
    ) -> Vec<f32> {
        read_texture_slice(device, queue, texture, width, height, 0, 0)
    }

    /// The deterministic fallback-adapter device shared by the ignored GPU
    /// diagnostics tests.
    fn smoke_device() -> (wgpu::Device, wgpu::Queue) {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            force_fallback_adapter: true,
            compatible_surface: None,
            ..Default::default()
        }))
        .expect("a fallback adapter is required for the ignored GPU test");
        pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("RV2-5 physical environment smoke device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("device creation must succeed")
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn physical_environment_gpu_generation_runs_and_reads_back() {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            force_fallback_adapter: true,
            compatible_surface: None,
            ..Default::default()
        }))
        .expect("a fallback adapter is required for the ignored GPU test");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("RV2-5 physical environment smoke device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("device creation must succeed");

        // COPY_SRC is only added here, through the test seam: production keeps
        // the exact RENDER_ATTACHMENT | TEXTURE_BINDING usage set.
        let textures = create_physical_environment_with_usage(
            &device,
            &queue,
            AtmosphereParameters::earth(),
            earth_sun(),
            wgpu::TextureUsages::COPY_SRC,
        )
        .expect("the Earth preset must generate");
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("the initialization submission must complete without validation errors");

        // Transmittance LUT: finite, inside [0, 1] and not all zero.
        let transmittance = read_first_face(
            &device,
            &queue,
            textures.test_texture(0),
            TRANSMITTANCE_SIZE.0,
            TRANSMITTANCE_SIZE.1,
        );
        let mut positive = 0usize;
        for texel in transmittance.as_chunks::<4>().0 {
            for channel in &texel[..3] {
                assert!(channel.is_finite(), "transmittance must be finite");
                assert!(
                    (-1e-3..=1.0 + 1e-3).contains(channel),
                    "transmittance must stay in [0, 1], got {channel}"
                );
                if *channel > 0.0 {
                    positive += 1;
                }
            }
        }
        assert!(
            positive > transmittance.len() / 4,
            "the transmittance LUT must not be all zero"
        );

        // Multi-scattering LUT: finite and non-negative.
        let multi = read_first_face(
            &device,
            &queue,
            textures.test_texture(1),
            MULTI_SCATTERING_SIZE.0,
            MULTI_SCATTERING_SIZE.1,
        );
        assert!(
            multi
                .iter()
                .all(|value| value.is_finite() && *value >= -1e-3)
        );
        assert!(
            multi.iter().any(|value| *value > 0.0),
            "the multi-scattering LUT must carry energy"
        );

        // BRDF integration LUT: finite, non-negative and not all zero.
        let brdf = read_first_face(
            &device,
            &queue,
            textures.test_texture(6),
            BRDF_LUT_SIZE.0,
            BRDF_LUT_SIZE.1,
        );
        assert!(
            brdf.iter()
                .all(|value| value.is_finite() && *value >= -1e-3),
            "the BRDF LUT must be finite and non-negative"
        );
        let split_sum_texels = brdf
            .as_chunks::<4>()
            .0
            .iter()
            .filter(|texel| texel[0] > 0.0 && texel[1] > 0.0)
            .count();
        assert!(
            split_sum_texels > 0,
            "the BRDF LUT must carry both split-sum terms (max A = {}, max B = {})",
            brdf.as_chunks::<4>()
                .0
                .iter()
                .fold(f32::MIN, |acc, texel| acc.max(texel[0])),
            brdf.as_chunks::<4>()
                .0
                .iter()
                .fold(f32::MIN, |acc, texel| acc.max(texel[1]))
        );

        // Sky-view LUT: finite, non-negative and carrying energy.
        let sky_view = read_first_face(
            &device,
            &queue,
            textures.test_texture(2),
            SKY_VIEW_SIZE.0,
            SKY_VIEW_SIZE.1,
        );
        assert!(
            sky_view
                .iter()
                .all(|value| value.is_finite() && *value >= -1e-3),
            "the sky-view LUT must be finite and non-negative"
        );
        assert!(
            sky_view.iter().any(|value| *value > 0.0),
            "the sky-view LUT must carry energy"
        );

        // Environment cubemap +Z face and diffuse irradiance +Z face: the cube
        // tangent frame degenerates exactly there, so these guard the
        // reference-vector choice in the GGX/cosine helpers.
        for (texture, size, label) in [
            (3usize, ENVIRONMENT_CUBE_SIZE, "environment cube +Z face"),
            (4usize, IRRADIANCE_CUBE_SIZE, "irradiance cube +Z face"),
            (5usize, SPECULAR_CUBE_SIZE, "prefiltered cube +Z face"),
        ] {
            let face = read_texture_slice(
                &device,
                &queue,
                textures.test_texture(texture),
                size,
                size,
                0,
                4,
            );
            assert!(
                face.iter()
                    .all(|value| value.is_finite() && *value >= -1e-3),
                "{label} must be finite and non-negative"
            );
            assert!(
                face.iter().any(|value| *value > 0.0),
                "{label} must carry energy"
            );
        }
    }

    /// Centre texel (RGB) of one read-back face slice.
    fn face_centre(size: u32, face: &[f32]) -> [f32; 3] {
        let texel = ((size / 2 * size + size / 2) * 4) as usize;
        [face[texel], face[texel + 1], face[texel + 2]]
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn transfer_lut_is_sun_independent_and_output_doubles_with_irradiance() {
        let (device, queue) = smoke_device();
        let parameters = AtmosphereParameters::earth();
        let base_sun = earth_sun();
        // The same sun with exactly double the irradiance: same direction and
        // angular radius, doubled radiance.
        let base_irradiance = base_sun.irradiance();
        let doubled_sun = SunState::from_irradiance(
            base_sun.direction,
            [
                base_irradiance[0] * 2.0,
                base_irradiance[1] * 2.0,
                base_irradiance[2] * 2.0,
            ],
            base_sun.angular_radius_radians,
        )
        .with_atmosphere_transmittance(parameters);
        assert!(
            (doubled_sun.irradiance()[1] - 2.0 * base_irradiance[1]).abs()
                < 1e-3 * base_irradiance[1],
            "the doubled sun must carry exactly twice the base irradiance"
        );

        let base = create_physical_environment_with_usage(
            &device,
            &queue,
            parameters,
            base_sun,
            wgpu::TextureUsages::COPY_SRC,
        )
        .expect("the base sun must generate");
        let doubled = create_physical_environment_with_usage(
            &device,
            &queue,
            parameters,
            doubled_sun,
            wgpu::TextureUsages::COPY_SRC,
        )
        .expect("the doubled sun must generate");
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("both initialization submissions must complete");

        // The Hillaire transfer LUT is built with E_I = 1: doubling the real
        // SunState irradiance must not change a single texel of it.
        let lut_base = read_first_face(
            &device,
            &queue,
            base.test_texture(1),
            MULTI_SCATTERING_SIZE.0,
            MULTI_SCATTERING_SIZE.1,
        );
        let lut_doubled = read_first_face(
            &device,
            &queue,
            doubled.test_texture(1),
            MULTI_SCATTERING_SIZE.0,
            MULTI_SCATTERING_SIZE.1,
        );
        assert!(
            lut_base.iter().any(|value| *value > 0.0),
            "the transfer LUT must carry energy"
        );
        assert_eq!(
            lut_base, lut_doubled,
            "the transfer LUT must be bitwise identical under a doubled sun: \
             it is a transfer function, the SunState irradiance is not baked in"
        );

        // Every consumed output is linear in the irradiance: doubling the
        // SunState irradiance doubles the sky-view LUT and the environment
        // cube (the single application point is the consumption in
        // `sky_radiance`, never the LUT itself).
        for (texture_index, size, label) in [
            (2usize, SKY_VIEW_SIZE, "sky-view LUT"),
            (
                3usize,
                (ENVIRONMENT_CUBE_SIZE, ENVIRONMENT_CUBE_SIZE),
                "environment cube face 0",
            ),
        ] {
            let base_values = read_first_face(
                &device,
                &queue,
                base.test_texture(texture_index),
                size.0,
                size.1,
            );
            let doubled_values = read_first_face(
                &device,
                &queue,
                doubled.test_texture(texture_index),
                size.0,
                size.1,
            );
            let mut energetic = 0usize;
            // RGB channels only: alpha is the constant 1.0 coverage flag and
            // is deliberately NOT linear in the irradiance.
            let base_texels = base_values.as_chunks::<4>().0;
            let doubled_texels = doubled_values.as_chunks::<4>().0;
            for (texel_a, texel_b) in base_texels.iter().zip(doubled_texels) {
                for channel in 0..3 {
                    let a = texel_a[channel];
                    let b = texel_b[channel];
                    assert!(a.is_finite() && b.is_finite(), "{label} must stay finite");
                    let expected = 2.0 * a;
                    assert!(
                        (b - expected).abs() <= 2e-7 + 1e-3 * expected.abs(),
                        "{label}: doubling the irradiance must double the output \
                         (got {a} -> {b}, expected {expected})"
                    );
                    if a.abs() > 2e-7 {
                        energetic += 1;
                    }
                }
            }
            assert!(
                energetic > 0,
                "{label} must carry real energy for the ratio to be meaningful"
            );
        }
    }

    /// Per-channel mean (RGB) of one read-back face slice, accumulated in f64.
    fn face_mean(face: &[f32]) -> [f32; 3] {
        let texels = face.as_chunks::<4>().0;
        let mut sum = [0.0f64; 3];
        for texel in texels {
            for channel in 0..3 {
                sum[channel] += f64::from(texel[channel]);
            }
        }
        std::array::from_fn(|channel| (sum[channel] / texels.len() as f64) as f32)
    }

    /// Largest per-channel absolute difference between two RGB triples.
    fn rgb_distance(a: [f32; 3], b: [f32; 3]) -> f32 {
        a.iter()
            .zip(b)
            .map(|(left, right)| (left - right).abs())
            .fold(0.0_f32, f32::max)
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn physical_environment_cube_faces_are_individually_routed() {
        let (device, queue) = smoke_device();
        // The pre-existing asymmetric SunState ([0.4, 0.8, -0.3]) makes the
        // six cube axes physically distinct: the Mie forward-scattering lobe
        // lives inside the +X face, so with correct instance routing every
        // face integrates a different sky, while a face-routing collapse
        // (every pass drawing instance 0) yields six bitwise-identical faces.
        //
        // Discriminator choice: near-horizontal *centre* texels are dominated
        // by the direction-independent multiple-scattering floor (the horizon
        // single scattering sits below one f16 ulp of that floor), so centres
        // alone cannot separate the four horizontal faces; per-face f32 means
        // over the full 12-bit f16 mantissa range can, and bitwise image
        // inequality catches an exact routing collapse directly.
        let textures = create_physical_environment_with_usage(
            &device,
            &queue,
            AtmosphereParameters::earth(),
            earth_sun(),
            wgpu::TextureUsages::COPY_SRC,
        )
        .expect("the Earth preset must generate");
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("the initialization submission must complete without validation errors");

        let read_faces = |texture_index: usize, size: u32, mip: u32| -> Vec<Vec<f32>> {
            (0..CUBE_FACE_COUNT)
                .map(|face| {
                    read_texture_slice(
                        &device,
                        &queue,
                        textures.test_texture(texture_index),
                        size >> mip,
                        size >> mip,
                        mip,
                        face,
                    )
                })
                .collect()
        };

        // --- Environment cube: all six faces, real routing -----------------
        let environment = read_faces(3, ENVIRONMENT_CUBE_SIZE, 0);
        for (index, face) in environment.iter().enumerate() {
            assert!(
                face.iter()
                    .all(|value| value.is_finite() && *value >= -1e-3),
                "environment face {index} must stay finite and non-negative, horizon included"
            );
            assert!(
                face.iter().any(|value| *value > 0.0),
                "environment face {index} must carry energy"
            );
        }
        let centres: Vec<[f32; 3]> = environment
            .iter()
            .map(|face| face_centre(ENVIRONMENT_CUBE_SIZE, face))
            .collect();
        let means: Vec<[f32; 3]> = environment.iter().map(|face| face_mean(face)).collect();
        eprintln!("environment cube face centres: {centres:?}");
        eprintln!("environment cube face means:   {means:?}");
        // +X != -X, +Y != -Y, +Z != -Z: opposite faces sample opposite sky
        // hemispheres under the asymmetric sun and must differ clearly.
        for (positive, negative) in [(0usize, 1usize), (2, 3), (4, 5)] {
            let distance = rgb_distance(means[positive], means[negative]);
            assert!(
                distance > 1e-8,
                "environment faces {positive} and {negative} must not be copies \
                 (means {:?} vs {:?}, distance {distance})",
                means[positive],
                means[negative]
            );
        }
        // Up and down are unambiguous even texel-by-texel: the +Y centre
        // looks into the sun hemisphere, the -Y centre at the ground bounce.
        assert!(
            rgb_distance(centres[2], centres[3]) > 1e-7,
            "+Y and -Y face centres must differ (got {:?} vs {:?})",
            centres[2],
            centres[3]
        );
        // The six faces must not be bitwise/near-identical: under a routing
        // collapse every face image would equal face 0 exactly.
        for other in 1..CUBE_FACE_COUNT as usize {
            assert_ne!(
                environment[0], environment[other],
                "environment face {other} must not be a bitwise copy of face 0"
            );
        }
        let near_duplicates = means
            .iter()
            .filter(|mean| rgb_distance(means[0], **mean) <= 1e-9)
            .count();
        assert_eq!(
            near_duplicates, 1,
            "only face 0 may match itself; all six face means must be distinct, got {means:?}"
        );

        // --- Diffuse irradiance cube: same sanity check ---------------------
        let irradiance = read_faces(4, IRRADIANCE_CUBE_SIZE, 0);
        let irradiance_means: Vec<[f32; 3]> =
            irradiance.iter().map(|face| face_mean(face)).collect();
        eprintln!("irradiance cube face means:      {irradiance_means:?}");
        for (positive, negative) in [(0usize, 1usize), (2, 3), (4, 5)] {
            let distance = rgb_distance(irradiance_means[positive], irradiance_means[negative]);
            assert!(
                distance > 1e-9,
                "irradiance faces {positive} and {negative} must not be copies \
                 (means {:?} vs {:?}, distance {distance})",
                irradiance_means[positive],
                irradiance_means[negative]
            );
        }
        for other in 1..CUBE_FACE_COUNT as usize {
            assert_ne!(
                irradiance[0], irradiance[other],
                "irradiance face {other} must not be a bitwise copy of face 0"
            );
        }

        // --- Prefiltered specular cube, mip 0: same sanity check ------------
        // At roughness 0 the GGX lobe degenerates to the face normal, so
        // mip 0 reproduces the environment cube face and inherits its
        // per-face distinctness.
        let prefiltered = read_faces(5, SPECULAR_CUBE_SIZE, 0);
        let prefiltered_means: Vec<[f32; 3]> =
            prefiltered.iter().map(|face| face_mean(face)).collect();
        eprintln!("prefiltered mip-0 face means:    {prefiltered_means:?}");
        for (positive, negative) in [(0usize, 1usize), (2, 3), (4, 5)] {
            let distance = rgb_distance(prefiltered_means[positive], prefiltered_means[negative]);
            assert!(
                distance > 1e-8,
                "prefiltered mip-0 faces {positive} and {negative} must not be copies \
                 (means {:?} vs {:?}, distance {distance})",
                prefiltered_means[positive],
                prefiltered_means[negative]
            );
        }
        for other in 1..CUBE_FACE_COUNT as usize {
            assert_ne!(
                prefiltered[0], prefiltered[other],
                "prefiltered mip-0 face {other} must not be a bitwise copy of face 0"
            );
        }
    }
}
