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

/// Record one fullscreen-triangle pass; cube faces are drawn as `instances`.
fn render_fullscreen(
    encoder: &mut wgpu::CommandEncoder,
    label: &'static str,
    view: &wgpu::TextureView,
    pipeline: &wgpu::RenderPipeline,
    bind_group: &wgpu::BindGroup,
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
    pass.draw(0..3, 0..instances);
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
        1,
    );
    render_fullscreen(
        &mut encoder,
        "RV2-5 multi-scattering LUT pass",
        &multi_scattering_view,
        &multi_scattering_pipeline,
        &multi_scattering_bind_group,
        1,
    );
    render_fullscreen(
        &mut encoder,
        "RV2-5 sky-view LUT pass",
        &sky_view_view,
        &sky_view_pipeline,
        &atmosphere_bind_group,
        1,
    );
    for face_view in &environment_face_views {
        render_fullscreen(
            &mut encoder,
            "RV2-5 environment cube face pass",
            face_view,
            &environment_pipeline,
            &atmosphere_bind_group,
            1,
        );
    }
    for face_view in &irradiance_face_views {
        render_fullscreen(
            &mut encoder,
            "RV2-5 diffuse irradiance cube face pass",
            face_view,
            &irradiance_pipeline,
            &irradiance_bind_group,
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
        // Multi-scattering must be a real spherical integration.
        assert!(atmosphere.contains("multi_scattering_texel"));
        assert!(atmosphere.contains("second_order * albedo / denominator"));
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
                mip_level: 0,
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
        read_texture_slice(device, queue, texture, width, height, 0)
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
}
