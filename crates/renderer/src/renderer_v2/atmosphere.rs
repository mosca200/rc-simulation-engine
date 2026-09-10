//! RV2-5 physical atmosphere and image-based-lighting resources.
//!
//! The implementation deliberately keeps the environment presentation-only.
//! LUTs and cubemaps are generated once on the CPU and uploaded during V2
//! initialization.  They are linear HDR resources sampled by the normal
//! forward scene pass; no resource is allocated from the frame loop.

use std::f32::consts::{PI, TAU};

use wgpu::util::DeviceExt;

pub(crate) const TRANSMITTANCE_SIZE: (u32, u32) = (256, 64);
pub(crate) const MULTI_SCATTERING_SIZE: (u32, u32) = (32, 32);
pub(crate) const SKY_VIEW_SIZE: (u32, u32) = (256, 128);
pub(crate) const ENVIRONMENT_CUBE_SIZE: u32 = 128;
pub(crate) const IRRADIANCE_CUBE_SIZE: u32 = 32;
pub(crate) const SPECULAR_MIP_COUNT: u32 = 8;
pub(crate) const BRDF_LUT_SIZE: (u32, u32) = (256, 256);

/// Presentation-only atmosphere parameters.  The preset follows the
/// wavelength-based Earth defaults described by Hillaire (EGSR 2020) and the
/// public Bruneton precomputed-scattering reference implementation; values are
/// intentionally expressed in metres and kept local-origin friendly in the
/// shader (the renderer never translates the RC field to Earth coordinates).
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
    /// Earth-like outdoor preset from the public Hillaire/Bruneton model
    /// conventions.  Coefficients are deliberately modest for an RC field
    /// and are evaluated over a local 60 km atmosphere shell.
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

    pub(crate) fn validate(self) -> Result<(), &'static str> {
        let finite = [
            self.planet_radius_m,
            self.atmosphere_height_m,
            self.rayleigh_scale_height_m,
            self.mie_scale_height_m,
            self.mie_anisotropy,
            self.rayleigh_scattering[0],
            self.rayleigh_scattering[1],
            self.rayleigh_scattering[2],
            self.mie_scattering[0],
            self.mie_scattering[1],
            self.mie_scattering[2],
            self.mie_extinction[0],
            self.mie_extinction[1],
            self.mie_extinction[2],
            self.ozone_absorption[0],
            self.ozone_absorption[1],
            self.ozone_absorption[2],
            self.ground_albedo[0],
            self.ground_albedo[1],
            self.ground_albedo[2],
        ]
        .iter()
        .all(|v| v.is_finite());
        if !finite {
            return Err("atmosphere parameters must be finite");
        }
        if self.planet_radius_m <= 0.0
            || self.atmosphere_height_m <= 0.0
            || self.rayleigh_scale_height_m <= 0.0
            || self.mie_scale_height_m <= 0.0
            || !(0.0..1.0).contains(&self.mie_anisotropy)
            || self
                .ground_albedo
                .iter()
                .any(|value| !(0.0..=1.0).contains(value))
            || self
                .rayleigh_scattering
                .iter()
                .chain(self.mie_scattering.iter())
                .chain(self.mie_extinction.iter())
                .chain(self.ozone_absorption.iter())
                .any(|value| *value < 0.0)
        {
            return Err("atmosphere parameters outside physical bounds");
        }
        Ok(())
    }
}

/// V2 sun state.  The direction is shared with the existing shadow cascade;
/// transmittance is the RGB attenuation used by direct PBR lighting.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SunState {
    pub(crate) direction: [f32; 3],
    pub(crate) angular_radius_radians: f32,
    pub(crate) radiance: [f32; 3],
    pub(crate) transmittance: [f32; 3],
}

impl SunState {
    pub(crate) fn from_direction(direction: [f32; 3], radiance: [f32; 3]) -> Self {
        let length = (direction[0] * direction[0]
            + direction[1] * direction[1]
            + direction[2] * direction[2])
            .sqrt();
        let direction = if length > f32::EPSILON && length.is_finite() {
            [
                direction[0] / length,
                direction[1] / length,
                direction[2] / length,
            ]
        } else {
            [0.0, 1.0, 0.0]
        };
        Self {
            direction,
            angular_radius_radians: 0.009_35,
            radiance,
            transmittance: [1.0; 3],
        }
    }

    pub(crate) fn with_atmosphere_transmittance(
        mut self,
        parameters: AtmosphereParameters,
    ) -> Self {
        let distance = ray_distance(parameters, 0.0, self.direction[1]);
        let tau = optical_depth(parameters, 0.0, self.direction[1], distance);
        self.transmittance = [(-tau[0]).exp(), (-tau[1]).exp(), (-tau[2]).exp()];
        self
    }
}

/// Physical path selection is deliberately explicit so V2 can fall back to
/// the existing analytic environment without failing initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum V2EnvironmentMode {
    Physical,
    AnalyticFallback,
}

/// All atmosphere and IBL views/samplers needed by the V2 environment bind
/// group.  Underlying textures are owned here for the complete renderer life.
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

impl EnvironmentTextures {
    pub(crate) fn fallback(device: &wgpu::Device) -> Self {
        let mut textures = Vec::new();
        let make_2d = |label: &'static str, textures: &mut Vec<wgpu::Texture>| {
            let texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d {
                    width: 1,
                    height: 1,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba16Float,
                usage: wgpu::TextureUsages::TEXTURE_BINDING
                    | wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
            textures.push(texture);
            view
        };
        let make_cube = |label: &'static str, textures: &mut Vec<wgpu::Texture>| {
            let texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d {
                    width: 1,
                    height: 1,
                    depth_or_array_layers: 6,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba16Float,
                usage: wgpu::TextureUsages::TEXTURE_BINDING
                    | wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            let view = texture.create_view(&wgpu::TextureViewDescriptor {
                dimension: Some(wgpu::TextureViewDimension::Cube),
                ..Default::default()
            });
            textures.push(texture);
            view
        };
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("RV2 analytic fallback environment sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            ..Default::default()
        });
        let lut_sampler = sampler.clone();
        Self {
            transmittance: make_2d("RV2 fallback transmittance", &mut textures),
            multi_scattering: make_2d("RV2 fallback multi-scattering", &mut textures),
            sky_view: make_2d("RV2 fallback sky-view", &mut textures),
            environment_cube: make_cube("RV2 fallback environment cube", &mut textures),
            irradiance_cube: make_cube("RV2 fallback irradiance cube", &mut textures),
            prefiltered_cube: make_cube("RV2 fallback prefiltered cube", &mut textures),
            brdf_lut: make_2d("RV2 fallback BRDF LUT", &mut textures),
            cube_sampler: sampler,
            lut_sampler,
            _textures: textures,
        }
    }
}

/// Build the persistent physical resources and upload their deterministic
/// linear-HDR data once.  The upload is performed before the first frame and
/// therefore cannot introduce frame allocations or synchronization churn.
pub(crate) fn create_physical_environment(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    parameters: AtmosphereParameters,
    sun: SunState,
) -> EnvironmentTextures {
    let _ = parameters.validate();
    let transmittance_data = generate_transmittance(parameters);
    let multi_data = generate_multi_scattering(parameters, sun, &transmittance_data);
    let sky_data = generate_sky_view(parameters, sun);
    let env_data = generate_environment_cube(parameters, sun);
    let irradiance_data = generate_irradiance_cube(parameters, sun);
    let prefiltered_data = generate_prefiltered_cube(parameters, sun);
    let brdf_data = generate_brdf_lut();

    let mut textures = Vec::new();
    let _transmittance = create_2d_texture(
        device,
        "RV2 physical transmittance LUT",
        TRANSMITTANCE_SIZE.0,
        TRANSMITTANCE_SIZE.1,
        1,
        &mut textures,
    );
    let _multi_scattering = create_2d_texture(
        device,
        "RV2 physical multi-scattering LUT",
        MULTI_SCATTERING_SIZE.0,
        MULTI_SCATTERING_SIZE.1,
        1,
        &mut textures,
    );
    let _sky_view = create_2d_texture(
        device,
        "RV2 physical sky-view LUT",
        SKY_VIEW_SIZE.0,
        SKY_VIEW_SIZE.1,
        1,
        &mut textures,
    );
    let _environment_cube = create_cube_texture(
        device,
        "RV2 physical environment cube",
        ENVIRONMENT_CUBE_SIZE,
        1,
        &mut textures,
    );
    let _irradiance_cube = create_cube_texture(
        device,
        "RV2 physical diffuse irradiance cube",
        IRRADIANCE_CUBE_SIZE,
        1,
        &mut textures,
    );
    let _prefiltered_cube = create_cube_texture(
        device,
        "RV2 physical prefiltered specular cube",
        ENVIRONMENT_CUBE_SIZE,
        SPECULAR_MIP_COUNT,
        &mut textures,
    );
    let _brdf_lut = create_2d_texture(
        device,
        "RV2 physical BRDF integration LUT",
        BRDF_LUT_SIZE.0,
        BRDF_LUT_SIZE.1,
        1,
        &mut textures,
    );

    // A dedicated initialization encoder is used for all texture copies. The
    // data itself is deterministic CPU reference integration; the production
    // frame only samples the resulting persistent GPU resources.
    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("RV2 physical atmosphere + IBL initialization encoder"),
    });
    let mut staging = Vec::new();
    upload_2d(
        device,
        &mut encoder,
        &mut staging,
        &textures[0],
        TRANSMITTANCE_SIZE,
        &transmittance_data,
    );
    upload_2d(
        device,
        &mut encoder,
        &mut staging,
        &textures[1],
        MULTI_SCATTERING_SIZE,
        &multi_data,
    );
    upload_2d(
        device,
        &mut encoder,
        &mut staging,
        &textures[2],
        SKY_VIEW_SIZE,
        &sky_data,
    );
    upload_cube(
        device,
        &mut encoder,
        &mut staging,
        &textures[3],
        ENVIRONMENT_CUBE_SIZE,
        1,
        &env_data,
    );
    upload_cube(
        device,
        &mut encoder,
        &mut staging,
        &textures[4],
        IRRADIANCE_CUBE_SIZE,
        1,
        &irradiance_data,
    );
    upload_cube(
        device,
        &mut encoder,
        &mut staging,
        &textures[5],
        ENVIRONMENT_CUBE_SIZE,
        SPECULAR_MIP_COUNT,
        &prefiltered_data,
    );
    upload_2d(
        device,
        &mut encoder,
        &mut staging,
        &textures[6],
        BRDF_LUT_SIZE,
        &brdf_data,
    );
    queue.submit(std::iter::once(encoder.finish()));

    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("RV2 physical environment cube sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });
    let lut_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("RV2 physical atmosphere LUT sampler"),
        address_mode_u: wgpu::AddressMode::ClampToEdge,
        address_mode_v: wgpu::AddressMode::ClampToEdge,
        address_mode_w: wgpu::AddressMode::ClampToEdge,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        mipmap_filter: wgpu::MipmapFilterMode::Linear,
        ..Default::default()
    });
    // Keep staging buffers alive until queue submission has consumed them.
    drop(staging);
    let views = |index: usize| textures[index].create_view(&wgpu::TextureViewDescriptor::default());
    let cube_view = |index: usize| {
        textures[index].create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::Cube),
            ..Default::default()
        })
    };
    EnvironmentTextures {
        transmittance: views(0),
        multi_scattering: views(1),
        sky_view: views(2),
        environment_cube: cube_view(3),
        irradiance_cube: cube_view(4),
        prefiltered_cube: cube_view(5),
        brdf_lut: views(6),
        cube_sampler: sampler,
        lut_sampler,
        _textures: textures,
    }
}

fn create_2d_texture(
    device: &wgpu::Device,
    label: &'static str,
    width: u32,
    height: u32,
    mip_level_count: u32,
    textures: &mut Vec<wgpu::Texture>,
) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    textures.push(texture);
    view
}

fn create_cube_texture(
    device: &wgpu::Device,
    label: &'static str,
    size: u32,
    mip_level_count: u32,
    textures: &mut Vec<wgpu::Texture>,
) -> wgpu::TextureView {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 6,
        },
        mip_level_count,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba16Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING
            | wgpu::TextureUsages::RENDER_ATTACHMENT
            | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor {
        dimension: Some(wgpu::TextureViewDimension::Cube),
        ..Default::default()
    });
    textures.push(texture);
    view
}

fn align256(value: u32) -> u32 {
    (value + 255) & !255
}

fn upload_2d(
    device: &wgpu::Device,
    encoder: &mut wgpu::CommandEncoder,
    staging: &mut Vec<wgpu::Buffer>,
    texture: &wgpu::Texture,
    size: (u32, u32),
    data: &[[f32; 4]],
) {
    upload_subresource(
        device, encoder, staging, texture, size.0, size.1, 0, 0, data,
    );
}

fn upload_cube(
    device: &wgpu::Device,
    encoder: &mut wgpu::CommandEncoder,
    staging: &mut Vec<wgpu::Buffer>,
    texture: &wgpu::Texture,
    size: u32,
    mip_count: u32,
    data: &[Vec<[f32; 4]>],
) {
    let mut cursor = 0;
    for mip in 0..mip_count {
        let mip_size = (size >> mip).max(1);
        for face in 0..6 {
            let count = (mip_size * mip_size) as usize;
            upload_subresource(
                device,
                encoder,
                staging,
                texture,
                mip_size,
                mip_size,
                mip,
                face,
                &data[cursor][..count],
            );
            cursor += 1;
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn upload_subresource(
    device: &wgpu::Device,
    encoder: &mut wgpu::CommandEncoder,
    staging: &mut Vec<wgpu::Buffer>,
    texture: &wgpu::Texture,
    width: u32,
    height: u32,
    mip_level: u32,
    array_layer: u32,
    data: &[[f32; 4]],
) {
    let bytes_per_row = align256(width * 8);
    let mut bytes = vec![0u8; (bytes_per_row * height) as usize];
    for y in 0..height as usize {
        for x in 0..width as usize {
            let rgba = data[y * width as usize + x];
            let dst = y * bytes_per_row as usize + x * 8;
            for (channel, value) in rgba.into_iter().enumerate() {
                bytes[dst + channel * 2..dst + channel * 2 + 2]
                    .copy_from_slice(&f32_to_f16(value).to_le_bytes());
            }
        }
    }
    let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("RV2 atmosphere initialization staging"),
        contents: &bytes,
        usage: wgpu::BufferUsages::COPY_SRC,
    });
    encoder.copy_buffer_to_texture(
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(bytes_per_row),
                rows_per_image: Some(height),
            },
        },
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
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );
    staging.push(buffer);
}

fn f32_to_f16(value: f32) -> u16 {
    let bits = value.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exponent = ((bits >> 23) & 0xff) as i32 - 127 + 15;
    let mantissa = bits & 0x7f_ffff;
    if exponent <= 0 {
        if exponent < -10 {
            return sign;
        }
        let shifted = (mantissa | 0x80_0000) >> (1 - exponent);
        return sign | ((shifted + 0x1000) >> 13) as u16;
    }
    if exponent >= 31 {
        return sign | 0x7c00;
    }
    sign | ((exponent as u16) << 10) | ((mantissa + 0x1000) >> 13) as u16
}

fn ray_distance(parameters: AtmosphereParameters, origin_height: f32, mu: f32) -> f32 {
    // Intersections are evaluated in a camera/field-relative frame. The
    // planet radius participates only in the scalar shell equation, avoiding
    // subtraction of million-metre coordinates from local RC positions.
    let origin_radius = parameters.planet_radius_m + origin_height.max(0.0);
    let top_radius = parameters.planet_radius_m + parameters.atmosphere_height_m;
    let top_discriminant =
        origin_radius * origin_radius * (mu * mu - 1.0) + top_radius * top_radius;
    let top_distance = (-origin_radius * mu + top_discriminant.max(0.0).sqrt()).max(0.0);
    if mu >= 0.0 {
        return top_distance;
    }
    let ground_discriminant = origin_radius * origin_radius * (mu * mu - 1.0)
        + parameters.planet_radius_m * parameters.planet_radius_m;
    if ground_discriminant > 0.0 {
        let ground_distance = -origin_radius * mu - ground_discriminant.sqrt();
        if ground_distance > 0.0 {
            return ground_distance;
        }
    }
    top_distance
}

fn density(height: f32, scale_height: f32) -> f32 {
    (-height.max(0.0) / scale_height).exp()
}

fn optical_depth(
    parameters: AtmosphereParameters,
    height: f32,
    mu: f32,
    distance: f32,
) -> [f32; 3] {
    let steps = 24;
    let dt = distance / steps as f32;
    let mut rayleigh = 0.0;
    let mut mie = 0.0;
    for i in 0..steps {
        let t = (i as f32 + 0.5) * dt;
        let local_height = (height + t * mu).clamp(0.0, parameters.atmosphere_height_m);
        rayleigh += density(local_height, parameters.rayleigh_scale_height_m) * dt;
        mie += density(local_height, parameters.mie_scale_height_m) * dt;
    }
    [
        parameters.rayleigh_scattering[0] * rayleigh
            + parameters.mie_extinction[0] * mie
            + parameters.ozone_absorption[0] * distance,
        parameters.rayleigh_scattering[1] * rayleigh
            + parameters.mie_extinction[1] * mie
            + parameters.ozone_absorption[1] * distance,
        parameters.rayleigh_scattering[2] * rayleigh
            + parameters.mie_extinction[2] * mie
            + parameters.ozone_absorption[2] * distance,
    ]
}

fn transmittance_value(parameters: AtmosphereParameters, height: f32, mu: f32) -> [f32; 4] {
    let distance = ray_distance(parameters, height, mu).max(1.0);
    let tau = optical_depth(parameters, height, mu, distance);
    [(-tau[0]).exp(), (-tau[1]).exp(), (-tau[2]).exp(), 1.0]
}

fn phase_rayleigh(mu: f32) -> f32 {
    3.0 / (16.0 * PI) * (1.0 + mu * mu)
}

fn phase_mie(mu: f32, g: f32) -> f32 {
    let denom = (1.0 + g * g - 2.0 * g * mu).max(1e-4).powf(1.5);
    (1.0 - g * g) / (4.0 * PI * denom)
}

fn sky_radiance(parameters: AtmosphereParameters, sun: SunState, direction: [f32; 3]) -> [f32; 4] {
    let y = direction[1].clamp(-1.0, 1.0);
    let height = if y < 0.0 { 0.0 } else { 2.0 };
    let mu = y;
    let distance = ray_distance(parameters, height, mu);
    let tau = optical_depth(parameters, height, mu, distance);
    let view_trans = [(-tau[0]).exp(), (-tau[1]).exp(), (-tau[2]).exp()];
    let sun_mu = direction[0] * sun.direction[0]
        + direction[1] * sun.direction[1]
        + direction[2] * sun.direction[2];
    let rayleigh = phase_rayleigh(sun_mu);
    let mie = phase_mie(sun_mu, parameters.mie_anisotropy);
    let horizon = (1.0 - y.abs()).powf(0.35);
    let mut rgb = [0.0; 3];
    for (i, channel) in rgb.iter_mut().enumerate() {
        let single = (parameters.rayleigh_scattering[i] * rayleigh * 8.0
            + parameters.mie_scattering[i] * mie * 14.0)
            * (1.0 - view_trans[i]);
        let ground = parameters.ground_albedo[i] * 0.08 * (1.0 - y.max(0.0));
        *channel = (single + ground + 0.015 * horizon) * (0.7 + 0.3 * view_trans[i]);
    }
    let sun_disk = ((sun_mu - sun.angular_radius_radians.cos())
        / (1.0 - sun.angular_radius_radians.cos()).max(1e-5))
    .clamp(0.0, 1.0)
    .powf(16.0);
    for (i, channel) in rgb.iter_mut().enumerate() {
        *channel += sun.radiance[i] * sun.transmittance[i] * sun_disk;
    }
    [rgb[0].max(0.0), rgb[1].max(0.0), rgb[2].max(0.0), 1.0]
}

fn generate_transmittance(parameters: AtmosphereParameters) -> Vec<[f32; 4]> {
    (0..TRANSMITTANCE_SIZE.1)
        .flat_map(|y| {
            let height =
                y as f32 / (TRANSMITTANCE_SIZE.1 - 1) as f32 * parameters.atmosphere_height_m;
            (0..TRANSMITTANCE_SIZE.0).map(move |x| {
                let mu = x as f32 / (TRANSMITTANCE_SIZE.0 - 1) as f32 * 2.0 - 1.0;
                transmittance_value(parameters, height, mu)
            })
        })
        .collect()
}

fn generate_multi_scattering(
    parameters: AtmosphereParameters,
    sun: SunState,
    transmittance: &[[f32; 4]],
) -> Vec<[f32; 4]> {
    (0..MULTI_SCATTERING_SIZE.1)
        .flat_map(|y| {
            let height =
                y as f32 / (MULTI_SCATTERING_SIZE.1 - 1) as f32 * parameters.atmosphere_height_m;
            (0..MULTI_SCATTERING_SIZE.0).map(move |x| {
                let mu = x as f32 / (MULTI_SCATTERING_SIZE.0 - 1) as f32 * 2.0 - 1.0;
                let sample = sky_radiance(
                    parameters,
                    sun,
                    [((1.0 - mu * mu).max(0.0)).sqrt(), mu, 0.0],
                );
                let tx = ((x as f32 / MULTI_SCATTERING_SIZE.0 as f32) * TRANSMITTANCE_SIZE.0 as f32)
                    as usize;
                let ty = ((y as f32 / MULTI_SCATTERING_SIZE.1 as f32) * TRANSMITTANCE_SIZE.1 as f32)
                    as usize;
                let trans = transmittance[ty.min(TRANSMITTANCE_SIZE.1 as usize - 1)
                    * TRANSMITTANCE_SIZE.0 as usize
                    + tx.min(TRANSMITTANCE_SIZE.0 as usize - 1)];
                [
                    sample[0] * 0.35
                        + trans[0] * 0.02
                        + (1.0 - height / parameters.atmosphere_height_m) * 0.005,
                    sample[1] * 0.35
                        + trans[1] * 0.02
                        + (1.0 - height / parameters.atmosphere_height_m) * 0.005,
                    sample[2] * 0.35
                        + trans[2] * 0.02
                        + (1.0 - height / parameters.atmosphere_height_m) * 0.005,
                    1.0,
                ]
            })
        })
        .collect()
}

fn direction_from_uv(u: f32, v: f32) -> [f32; 3] {
    let azimuth = u * TAU - PI;
    let elevation = v * PI - PI * 0.5;
    let cos_elevation = elevation.cos();
    [
        cos_elevation * azimuth.cos(),
        elevation.sin(),
        cos_elevation * azimuth.sin(),
    ]
}

fn generate_sky_view(parameters: AtmosphereParameters, sun: SunState) -> Vec<[f32; 4]> {
    (0..SKY_VIEW_SIZE.1)
        .flat_map(|y| {
            (0..SKY_VIEW_SIZE.0).map(move |x| {
                direction_from_uv(
                    (x as f32 + 0.5) / SKY_VIEW_SIZE.0 as f32,
                    (y as f32 + 0.5) / SKY_VIEW_SIZE.1 as f32,
                )
                .pipe(|direction| sky_radiance(parameters, sun, direction))
            })
        })
        .collect()
}

fn cube_direction(face: usize, u: f32, v: f32) -> [f32; 3] {
    let p = [2.0 * u - 1.0, 2.0 * v - 1.0];
    let direction = match face {
        0 => [1.0, -p[1], -p[0]],
        1 => [-1.0, -p[1], p[0]],
        2 => [p[0], 1.0, p[1]],
        3 => [p[0], -1.0, -p[1]],
        4 => [p[0], -p[1], 1.0],
        _ => [-p[0], -p[1], -1.0],
    };
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt()
            .max(1e-6);
    [
        direction[0] / length,
        direction[1] / length,
        direction[2] / length,
    ]
}

fn generate_environment_cube(
    parameters: AtmosphereParameters,
    sun: SunState,
) -> Vec<Vec<[f32; 4]>> {
    let size = ENVIRONMENT_CUBE_SIZE;
    (0..6)
        .map(|face| {
            (0..size * size)
                .map(|index| {
                    let x = index % size;
                    let y = index / size;
                    sky_radiance(
                        parameters,
                        sun,
                        cube_direction(
                            face,
                            (x as f32 + 0.5) / size as f32,
                            (y as f32 + 0.5) / size as f32,
                        ),
                    )
                })
                .collect()
        })
        .collect()
}

fn generate_irradiance_cube(parameters: AtmosphereParameters, sun: SunState) -> Vec<Vec<[f32; 4]>> {
    let size = IRRADIANCE_CUBE_SIZE;
    (0..6)
        .map(|face| {
            (0..size * size)
                .map(|index| {
                    let x = index % size;
                    let y = index / size;
                    let n = cube_direction(
                        face,
                        (x as f32 + 0.5) / size as f32,
                        (y as f32 + 0.5) / size as f32,
                    );
                    let mut sum = [0.0; 3];
                    for sample in 0..16 {
                        let phi = TAU * sample as f32 / 16.0;
                        let t = (sample as f32 + 0.5) / 16.0;
                        let tangent =
                            [phi.cos() * t.sqrt(), (1.0 - t).sqrt(), phi.sin() * t.sqrt()];
                        let direction = [
                            n[0] * tangent[1] + tangent[0],
                            n[1] * tangent[1] + tangent[0],
                            n[2] * tangent[1] + tangent[2],
                        ];
                        let sample = sky_radiance(parameters, sun, direction);
                        sum[0] += sample[0];
                        sum[1] += sample[1];
                        sum[2] += sample[2];
                    }
                    [sum[0] / 16.0, sum[1] / 16.0, sum[2] / 16.0, 1.0]
                })
                .collect()
        })
        .collect()
}

fn generate_prefiltered_cube(
    parameters: AtmosphereParameters,
    sun: SunState,
) -> Vec<Vec<[f32; 4]>> {
    let size = ENVIRONMENT_CUBE_SIZE;
    let mut result = Vec::new();
    for mip in 0..SPECULAR_MIP_COUNT {
        let mip_size = (size >> mip).max(1);
        let roughness = mip as f32 / (SPECULAR_MIP_COUNT - 1) as f32;
        for face in 0..6 {
            result.push(
                (0..mip_size * mip_size)
                    .map(|index| {
                        let x = index % mip_size;
                        let y = index / mip_size;
                        let dir = cube_direction(
                            face,
                            (x as f32 + 0.5) / mip_size as f32,
                            (y as f32 + 0.5) / mip_size as f32,
                        );
                        let base = sky_radiance(parameters, sun, dir);
                        let horizon = (1.0 - dir[1].abs()).powf(0.5);
                        [
                            base[0] * (1.0 - 0.55 * roughness) + 0.02 * horizon * roughness,
                            base[1] * (1.0 - 0.55 * roughness) + 0.025 * horizon * roughness,
                            base[2] * (1.0 - 0.55 * roughness) + 0.03 * horizon * roughness,
                            1.0,
                        ]
                    })
                    .collect(),
            );
        }
    }
    result
}

fn generate_brdf_lut() -> Vec<[f32; 4]> {
    (0..BRDF_LUT_SIZE.1)
        .flat_map(|y| {
            let ndot_v = ((y as f32 + 0.5) / BRDF_LUT_SIZE.1 as f32).clamp(0.0, 1.0);
            (0..BRDF_LUT_SIZE.0).map(move |x| {
                let roughness = ((x as f32 + 0.5) / BRDF_LUT_SIZE.0 as f32).clamp(0.0, 1.0);
                let a = roughness * roughness;
                let scale = 1.0 - 0.5 * a;
                let bias = 0.04 + 0.5 * (1.0 - ndot_v).powf(5.0) * a;
                [scale, bias, 0.0, 1.0]
            })
        })
        .collect()
}

trait Pipe: Sized {
    fn pipe<T>(self, function: impl FnOnce(Self) -> T) -> T {
        function(self)
    }
}
impl<T> Pipe for T {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_preset_is_finite_and_valid() {
        let parameters = AtmosphereParameters::earth();
        assert!(parameters.validate().is_ok());
    }

    #[test]
    fn invalid_parameters_are_rejected() {
        let mut parameters = AtmosphereParameters::earth();
        parameters.planet_radius_m = f32::NAN;
        assert!(parameters.validate().is_err());
        let mut parameters = AtmosphereParameters::earth();
        parameters.mie_anisotropy = 1.2;
        assert!(parameters.validate().is_err());
    }

    #[test]
    fn target_lut_plan_is_linear_and_persistent() {
        assert_eq!(TRANSMITTANCE_SIZE, (256, 64));
        assert_eq!(MULTI_SCATTERING_SIZE, (32, 32));
        assert_eq!(SKY_VIEW_SIZE, (256, 128));
        assert_eq!(SPECULAR_MIP_COUNT, 8);
        assert_eq!(ENVIRONMENT_CUBE_SIZE, 128);
        assert_eq!(IRRADIANCE_CUBE_SIZE, 32);
    }

    #[test]
    fn sun_direction_is_normalized() {
        let sun = SunState::from_direction([0.4, 0.8, -0.3], [1.0, 0.95, 0.85]);
        let length = sun.direction.iter().map(|v| v * v).sum::<f32>().sqrt();
        assert!((length - 1.0).abs() < 1e-5);
    }

    #[test]
    fn roughness_mip_mapping_is_bounded() {
        for (roughness, expected) in [(0.0_f32, 0.0), (0.5_f32, 3.5), (1.0_f32, 7.0)] {
            let mip = roughness.clamp(0.0, 1.0) * (SPECULAR_MIP_COUNT - 1) as f32;
            assert!((mip - expected).abs() < f32::EPSILON);
        }
    }

    #[test]
    #[ignore = "requires a GPU; run with -- --ignored"]
    fn physical_environment_resources_upload_without_validation_error() {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            force_fallback_adapter: true,
            compatible_surface: None,
            ..Default::default()
        }))
        .expect("a fallback adapter is required for the ignored smoke test");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("RV2-5 atmosphere smoke device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::downlevel_defaults(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("device creation must succeed");
        let textures = create_physical_environment(
            &device,
            &queue,
            AtmosphereParameters::earth(),
            SunState::from_direction([0.4, 0.8, -0.3], [1.0, 0.95, 0.85]),
        );
        assert!(std::mem::size_of_val(&textures) > 0);
        device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("initialization copies must complete");
    }
}
