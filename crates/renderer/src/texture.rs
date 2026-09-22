//! G1C: GPU texture infrastructure for glTF base-color textures.
//!
//! # Color Space
//!
//! Base-color textures are color data (not linear mask data). They are uploaded
//! to the GPU in sRGB format (`Rgba8UnormSrgb`). The hardware performs the
//! sRGB-to-linear conversion automatically when sampling. No manual gamma
//! correction is applied in the shader.
//!
//! # Alignment
//!
//! WebGPU requires `bytes_per_row` to be aligned to `COPY_BYTES_PER_ROW_ALIGNMENT`
//! (256 bytes). For images whose width * 4 is not a multiple of 256, we pad each
//! row in a staging buffer before upload.
//!
//! # Overflow Safety
//!
//! All size arithmetic uses checked operations. Functions that compute buffer
//! sizes return `Result` with appropriate error variants rather than silently
//! wrapping or panicking.

use image::{ImageError, ImageReader};
use std::io::Cursor;
use thiserror::Error;

/// WebGPU requires texture copy rows to be aligned to this many bytes.
pub const COPY_BYTES_PER_ROW_ALIGNMENT: u32 = 256;

/// RGBA8 pixel data ready for GPU upload.
#[derive(Debug, Clone)]
pub struct DecodedTexture {
    pub width: u32,
    pub height: u32,
    pub rgba8: Vec<u8>,
}

#[derive(Debug, Error)]
pub enum TextureLoadError {
    #[error("image data is empty")]
    EmptyImageData,
    #[error("failed to decode image: {0}")]
    DecodeFailed(#[from] ImageError),
    #[error("image dimensions are zero or overflow: {width}x{height}")]
    InvalidDimensions { width: u32, height: u32 },
    #[error("image byte size overflow")]
    ByteSizeOverflow,
    #[error("padded row byte size overflow for width {width}")]
    PaddedRowOverflow { width: u32 },
    #[error("staging buffer byte size overflow for {width}x{height} texture")]
    StagingBufferOverflow { width: u32, height: u32 },
}

/// Decode an image from raw bytes (PNG or JPEG supported).
///
/// Returns RGBA8 pixel data. The image crate handles format detection.
pub fn decode_image(data: &[u8]) -> Result<DecodedTexture, TextureLoadError> {
    if data.is_empty() {
        return Err(TextureLoadError::EmptyImageData);
    }

    let cursor = Cursor::new(data);
    let reader = ImageReader::new(cursor)
        .with_guessed_format()
        .map_err(|e| TextureLoadError::DecodeFailed(image::ImageError::IoError(e)))?;
    let image = reader.decode()?;

    let rgba = image.to_rgba8();
    let (width, height) = rgba.dimensions();

    if width == 0 || height == 0 {
        return Err(TextureLoadError::InvalidDimensions { width, height });
    }

    // Check for overflow in byte size calculation.
    let pixel_count = (width as u64).checked_mul(height as u64);
    if pixel_count.is_none() {
        return Err(TextureLoadError::ByteSizeOverflow);
    }

    Ok(DecodedTexture {
        width,
        height,
        rgba8: rgba.into_raw(),
    })
}

/// sRGB (IEC 61966-2-1) decode of one normalized channel: [0, 1] -> linear.
///
/// ENV1-A: this is the single canonical definition of the transfer function in
/// this crate. Both the terrain mip chain (`terrain_textures`) and the ENV1
/// photographic source-map processing (`env1_material`) call it, so a color map
/// can never be filtered with two slightly different curves.
#[must_use]
pub(crate) fn srgb_to_linear_f64(channel: f64) -> f64 {
    let c = channel.clamp(0.0, 1.0);
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// sRGB (IEC 61966-2-1) encode of one linear channel: [0, 1] -> [0, 1].
#[must_use]
pub(crate) fn linear_to_srgb_f64(linear: f64) -> f64 {
    let c = linear.clamp(0.0, 1.0);
    if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

/// Compute the padded bytes_per_row for GPU upload.
///
/// WebGPU requires each row to be aligned to `COPY_BYTES_PER_ROW_ALIGNMENT`.
/// For RGBA8 (4 bytes per pixel), if `width * 4` is not a multiple of 256,
/// we must pad to the next multiple.
///
/// Returns `None` if the arithmetic would overflow u32.
#[must_use]
pub fn padded_bytes_per_row_checked(width: u32) -> Option<u32> {
    padded_bytes_per_row_checked_for_bytes_per_pixel(width, 4)
}

/// Compute the padded bytes_per_row for an upload with an arbitrary
/// `bytes_per_pixel` (e.g. 4 for RGBA8, 1 for R8 mip levels).
///
/// WebGPU requires each row to be aligned to `COPY_BYTES_PER_ROW_ALIGNMENT`
/// (256 bytes): the row size is padded up to the next multiple of 256.
///
/// Returns `None` if the arithmetic would overflow u32.
#[must_use]
pub fn padded_bytes_per_row_checked_for_bytes_per_pixel(
    width: u32,
    bytes_per_pixel: u32,
) -> Option<u32> {
    let unpadded = width.checked_mul(bytes_per_pixel)?;
    let remainder = unpadded % COPY_BYTES_PER_ROW_ALIGNMENT;
    if remainder == 0 {
        Some(unpadded)
    } else {
        unpadded.checked_add(COPY_BYTES_PER_ROW_ALIGNMENT - remainder)
    }
}

/// Compute the padded bytes_per_row for GPU upload.
///
/// # Panics
///
/// Panics if the arithmetic would overflow (use `padded_bytes_per_row_checked`
/// for a non-panicking version).
#[must_use]
pub fn padded_bytes_per_row(width: u32) -> u32 {
    padded_bytes_per_row_checked(width).expect("padded_bytes_per_row overflow")
}

/// Compute the padded bytes_per_row for an upload with an arbitrary
/// `bytes_per_pixel`.
///
/// # Panics
///
/// Panics if the arithmetic would overflow (use
/// `padded_bytes_per_row_checked_for_bytes_per_pixel` for a non-panicking
/// version).
#[must_use]
pub fn padded_bytes_per_row_for_bytes_per_pixel(width: u32, bytes_per_pixel: u32) -> u32 {
    padded_bytes_per_row_checked_for_bytes_per_pixel(width, bytes_per_pixel)
        .expect("padded bytes_per_row overflow")
}

/// Create a staging buffer with row padding for GPU upload.
///
/// Returns the padded data and the padded bytes_per_row.
/// All arithmetic is overflow-checked.
pub fn create_staging_buffer(texture: &DecodedTexture) -> Result<(Vec<u8>, u32), TextureLoadError> {
    let padded_row_bytes =
        padded_bytes_per_row_checked(texture.width).ok_or(TextureLoadError::PaddedRowOverflow {
            width: texture.width,
        })?;
    let unpadded_row_bytes =
        texture
            .width
            .checked_mul(4)
            .ok_or(TextureLoadError::PaddedRowOverflow {
                width: texture.width,
            })?;
    let padding_per_row = (padded_row_bytes - unpadded_row_bytes) as usize;

    let total_bytes = (padded_row_bytes as u64)
        .checked_mul(texture.height as u64)
        .ok_or(TextureLoadError::StagingBufferOverflow {
            width: texture.width,
            height: texture.height,
        })?;
    let total_bytes =
        usize::try_from(total_bytes).map_err(|_| TextureLoadError::StagingBufferOverflow {
            width: texture.width,
            height: texture.height,
        })?;

    if padding_per_row == 0 {
        return Ok((texture.rgba8.clone(), padded_row_bytes));
    }

    let mut staged = Vec::with_capacity(total_bytes);
    for row in 0..texture.height as usize {
        let start = row
            .checked_mul(unpadded_row_bytes as usize)
            .expect("row offset overflow");
        let end = start
            .checked_add(unpadded_row_bytes as usize)
            .expect("row end overflow");
        staged.extend_from_slice(&texture.rgba8[start..end]);
        staged.extend(std::iter::repeat_n(0u8, padding_per_row));
    }

    Ok((staged, padded_row_bytes))
}

/// glTF sampler wrap mode mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplerWrap {
    Repeat,
    ClampToEdge,
    MirroredRepeat,
}

impl SamplerWrap {
    /// Map from glTF sampler wrap enum.
    #[must_use]
    pub fn from_gltf(wrap: gltf::texture::WrappingMode) -> Self {
        match wrap {
            gltf::texture::WrappingMode::ClampToEdge => Self::ClampToEdge,
            gltf::texture::WrappingMode::MirroredRepeat => Self::MirroredRepeat,
            gltf::texture::WrappingMode::Repeat => Self::Repeat,
        }
    }

    #[must_use]
    pub fn to_wgpu(self) -> wgpu::AddressMode {
        match self {
            Self::ClampToEdge => wgpu::AddressMode::ClampToEdge,
            Self::MirroredRepeat => wgpu::AddressMode::MirrorRepeat,
            Self::Repeat => wgpu::AddressMode::Repeat,
        }
    }
}

/// glTF sampler min/mag filter mapping: the NEAREST-vs-LINEAR axis only.
///
/// ENV1-A: this no longer throws information away. A glTF `minFilter` carries
/// two independent axes — the minification mode and whether a mip chain is
/// sampled — and the mipmap axis is now preserved separately by
/// `SamplerMipmapFilter`. `SamplerConfig::mipmap_filter` is `None` exactly when
/// the asset asked for single-mip sampling.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplerFilter {
    Nearest,
    Linear,
}

impl SamplerFilter {
    /// Map the minification axis of a glTF min filter.
    ///
    /// The mipmap-selection axis is handled by
    /// `SamplerMipmapFilter::from_gltf_min`; nothing is discarded overall.
    #[must_use]
    pub fn from_gltf_min(filter: gltf::texture::MinFilter) -> Self {
        match filter {
            gltf::texture::MinFilter::Nearest
            | gltf::texture::MinFilter::NearestMipmapNearest
            | gltf::texture::MinFilter::NearestMipmapLinear => Self::Nearest,
            gltf::texture::MinFilter::Linear
            | gltf::texture::MinFilter::LinearMipmapNearest
            | gltf::texture::MinFilter::LinearMipmapLinear => Self::Linear,
        }
    }

    /// Map from glTF mag filter.
    #[must_use]
    pub fn from_gltf_mag(filter: gltf::texture::MagFilter) -> Self {
        match filter {
            gltf::texture::MagFilter::Nearest => Self::Nearest,
            gltf::texture::MagFilter::Linear => Self::Linear,
        }
    }

    #[must_use]
    pub fn to_wgpu(self) -> wgpu::FilterMode {
        match self {
            Self::Nearest => wgpu::FilterMode::Nearest,
            Self::Linear => wgpu::FilterMode::Linear,
        }
    }
}

/// The mipmap-selection axis of a glTF `minFilter`.
///
/// glTF encodes mipmapping inside the min filter: the bare `NEAREST` and
/// `LINEAR` values mean "sample mip level 0 only", while the four
/// `*_MIPMAP_*` values request a mip chain and state how levels are selected.
/// `None` in `SamplerConfig::mipmap_filter` therefore means the asset asked
/// for single-mip sampling, which is a real distinction rather than a missing
/// value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplerMipmapFilter {
    Nearest,
    Linear,
}

impl SamplerMipmapFilter {
    /// Map the mipmap-selection axis of a glTF min filter.
    ///
    /// Returns `None` when the asset requests no mipmapping at all.
    #[must_use]
    pub fn from_gltf_min(filter: gltf::texture::MinFilter) -> Option<Self> {
        match filter {
            gltf::texture::MinFilter::Nearest | gltf::texture::MinFilter::Linear => None,
            gltf::texture::MinFilter::NearestMipmapNearest
            | gltf::texture::MinFilter::LinearMipmapNearest => Some(Self::Nearest),
            gltf::texture::MinFilter::NearestMipmapLinear
            | gltf::texture::MinFilter::LinearMipmapLinear => Some(Self::Linear),
        }
    }

    /// Whether a glTF min filter requests a mip chain at all.
    #[must_use]
    pub fn mipmapping_requested(filter: gltf::texture::MinFilter) -> bool {
        Self::from_gltf_min(filter).is_some()
    }

    #[must_use]
    pub fn to_wgpu(self) -> wgpu::MipmapFilterMode {
        match self {
            Self::Nearest => wgpu::MipmapFilterMode::Nearest,
            Self::Linear => wgpu::MipmapFilterMode::Linear,
        }
    }
}

/// Sampler configuration extracted from a glTF sampler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplerConfig {
    pub wrap_s: SamplerWrap,
    pub wrap_t: SamplerWrap,
    pub min_filter: SamplerFilter,
    pub mag_filter: SamplerFilter,
    /// Mipmap selection, or `None` when the asset asked for single-mip
    /// sampling (glTF `minFilter` of NEAREST or LINEAR, or no minFilter).
    pub mipmap_filter: Option<SamplerMipmapFilter>,
}

impl SamplerConfig {
    /// Default sampler: linear filtering, repeat wrapping, no mipmapping
    /// requested.
    #[must_use]
    pub fn default_sampler() -> Self {
        Self {
            wrap_s: SamplerWrap::Repeat,
            wrap_t: SamplerWrap::Repeat,
            min_filter: SamplerFilter::Linear,
            mag_filter: SamplerFilter::Linear,
            mipmap_filter: None,
        }
    }

    /// Extract from a glTF sampler, preserving both filter axes.
    #[must_use]
    pub fn from_gltf_sampler(sampler: &gltf::texture::Sampler) -> Self {
        let wrap_s = SamplerWrap::from_gltf(sampler.wrap_s());
        let wrap_t = SamplerWrap::from_gltf(sampler.wrap_t());
        let min_filter = sampler
            .min_filter()
            .map(SamplerFilter::from_gltf_min)
            .unwrap_or(SamplerFilter::Linear);
        let mag_filter = sampler
            .mag_filter()
            .map(SamplerFilter::from_gltf_mag)
            .unwrap_or(SamplerFilter::Linear);
        // An absent minFilter means the glTF spec leaves minification
        // implementation-defined, so no mipmapping is assumed.
        let mipmap_filter = sampler
            .min_filter()
            .and_then(SamplerMipmapFilter::from_gltf_min);
        Self {
            wrap_s,
            wrap_t,
            min_filter,
            mag_filter,
            mipmap_filter,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn padded_bytes_per_row_returns_unpadded_when_aligned() {
        assert_eq!(padded_bytes_per_row(64), 256);
    }

    #[test]
    fn padded_bytes_per_row_pads_to_alignment() {
        assert_eq!(padded_bytes_per_row(1), 256);
        assert_eq!(padded_bytes_per_row(63), 256);
        assert_eq!(padded_bytes_per_row(65), 512);
    }

    #[test]
    fn padded_bytes_per_row_checked_matches_unchecked() {
        for width in [0, 1, 63, 64, 65, 128, 256, 1024] {
            assert_eq!(
                padded_bytes_per_row_checked(width),
                Some(padded_bytes_per_row(width))
            );
        }
    }

    #[test]
    fn padded_bytes_per_row_generic_matches_rgba8_specialization() {
        for width in [0, 1, 63, 64, 65, 128, 256, 1024] {
            assert_eq!(
                padded_bytes_per_row_checked_for_bytes_per_pixel(width, 4),
                padded_bytes_per_row_checked(width),
                "the generic helper must agree with the RGBA8 specialization"
            );
        }
    }

    #[test]
    fn padded_bytes_per_row_r8_aligns_to_copy_alignment() {
        // R8 uploads (e.g. roughness mip levels) must respect the same 256-byte
        // row alignment; only widths of 256+ are already aligned.
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(512, 1),
            Some(512)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(256, 1),
            Some(256)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(128, 1),
            Some(256)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(64, 1),
            Some(256)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(8, 1),
            Some(256)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(1, 1),
            Some(256)
        );
        assert_eq!(
            padded_bytes_per_row_checked_for_bytes_per_pixel(u32::MAX, 1),
            None
        );
    }

    #[test]
    fn padded_bytes_per_row_checked_overflow() {
        assert_eq!(padded_bytes_per_row_checked(u32::MAX), None);
        assert_eq!(padded_bytes_per_row_checked(u32::MAX / 4 + 1), None);
    }

    #[test]
    fn create_staging_buffer_no_padding_needed() {
        let texture = DecodedTexture {
            width: 64,
            height: 2,
            rgba8: vec![0xAB; 64 * 2 * 4],
        };
        let (staged, bytes_per_row) = create_staging_buffer(&texture).unwrap();
        assert_eq!(bytes_per_row, 256);
        assert_eq!(staged.len(), 256 * 2);
        assert_eq!(&staged[..256], &texture.rgba8[..256]);
    }

    #[test]
    fn create_staging_buffer_with_padding() {
        let texture = DecodedTexture {
            width: 1,
            height: 2,
            rgba8: vec![0xAB; 8],
        };
        let (staged, bytes_per_row) = create_staging_buffer(&texture).unwrap();
        assert_eq!(bytes_per_row, 256);
        assert_eq!(staged.len(), 256 * 2);
        assert_eq!(&staged[..4], &[0xAB; 4]);
        assert_eq!(&staged[4..256], &[0; 252]);
        assert_eq!(&staged[256..260], &[0xAB; 4]);
        assert_eq!(&staged[260..512], &[0; 252]);
    }

    #[test]
    fn create_staging_buffer_overflow_returns_error() {
        let texture = DecodedTexture {
            width: u32::MAX,
            height: u32::MAX,
            rgba8: vec![],
        };
        let result = create_staging_buffer(&texture);
        assert!(result.is_err());
    }

    #[test]
    fn decode_empty_data_returns_error() {
        let result = decode_image(&[]);
        assert!(matches!(result, Err(TextureLoadError::EmptyImageData)));
    }

    #[test]
    fn decode_invalid_data_returns_error() {
        let result = decode_image(&[0xFF, 0xD8, 0xFF]);
        assert!(matches!(result, Err(TextureLoadError::DecodeFailed(_))));
    }

    #[test]
    fn sampler_wrap_from_gltf_maps_correctly() {
        assert_eq!(
            SamplerWrap::from_gltf(gltf::texture::WrappingMode::Repeat),
            SamplerWrap::Repeat
        );
        assert_eq!(
            SamplerWrap::from_gltf(gltf::texture::WrappingMode::ClampToEdge),
            SamplerWrap::ClampToEdge
        );
        assert_eq!(
            SamplerWrap::from_gltf(gltf::texture::WrappingMode::MirroredRepeat),
            SamplerWrap::MirroredRepeat
        );
    }

    #[test]
    fn sampler_filter_from_gltf_min_collapses_mipmaps() {
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::Nearest),
            SamplerFilter::Nearest
        );
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::NearestMipmapNearest),
            SamplerFilter::Nearest
        );
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::NearestMipmapLinear),
            SamplerFilter::Nearest
        );
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::Linear),
            SamplerFilter::Linear
        );
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::LinearMipmapNearest),
            SamplerFilter::Linear
        );
        assert_eq!(
            SamplerFilter::from_gltf_min(gltf::texture::MinFilter::LinearMipmapLinear),
            SamplerFilter::Linear
        );
    }

    #[test]
    fn sampler_filter_from_gltf_mag_maps_correctly() {
        assert_eq!(
            SamplerFilter::from_gltf_mag(gltf::texture::MagFilter::Nearest),
            SamplerFilter::Nearest
        );
        assert_eq!(
            SamplerFilter::from_gltf_mag(gltf::texture::MagFilter::Linear),
            SamplerFilter::Linear
        );
    }

    #[test]
    fn default_sampler_config_is_linear_repeat() {
        let config = SamplerConfig::default_sampler();
        assert_eq!(config.wrap_s, SamplerWrap::Repeat);
        assert_eq!(config.wrap_t, SamplerWrap::Repeat);
        assert_eq!(config.min_filter, SamplerFilter::Linear);
        assert_eq!(config.mag_filter, SamplerFilter::Linear);
        assert_eq!(config.mipmap_filter, None);
    }

    #[test]
    fn sampler_mipmap_filter_mapping_is_total_over_every_gltf_min_filter() {
        use gltf::texture::MinFilter;
        // The two bare values request mip 0 only; the four `*_MIPMAP_*` values
        // request a chain and say how levels are selected.
        let cases = [
            (MinFilter::Nearest, None),
            (MinFilter::Linear, None),
            (
                MinFilter::NearestMipmapNearest,
                Some(SamplerMipmapFilter::Nearest),
            ),
            (
                MinFilter::LinearMipmapNearest,
                Some(SamplerMipmapFilter::Nearest),
            ),
            (
                MinFilter::NearestMipmapLinear,
                Some(SamplerMipmapFilter::Linear),
            ),
            (
                MinFilter::LinearMipmapLinear,
                Some(SamplerMipmapFilter::Linear),
            ),
        ];
        for (filter, expected) in cases {
            assert_eq!(
                SamplerMipmapFilter::from_gltf_min(filter),
                expected,
                "mipmap mapping for {filter:?}"
            );
            assert_eq!(
                SamplerMipmapFilter::mipmapping_requested(filter),
                expected.is_some(),
                "mipmapping_requested for {filter:?}"
            );
        }
    }

    #[test]
    fn sampler_minification_and_mipmap_axes_are_independent() {
        use gltf::texture::MinFilter;
        // LINEAR_MIPMAP_NEAREST is linear minification with nearest mip
        // selection: the two axes must not collapse into one value.
        assert_eq!(
            SamplerFilter::from_gltf_min(MinFilter::LinearMipmapNearest),
            SamplerFilter::Linear
        );
        assert_eq!(
            SamplerMipmapFilter::from_gltf_min(MinFilter::LinearMipmapNearest),
            Some(SamplerMipmapFilter::Nearest)
        );
        // NEAREST_MIPMAP_LINEAR is the mirror case.
        assert_eq!(
            SamplerFilter::from_gltf_min(MinFilter::NearestMipmapLinear),
            SamplerFilter::Nearest
        );
        assert_eq!(
            SamplerMipmapFilter::from_gltf_min(MinFilter::NearestMipmapLinear),
            Some(SamplerMipmapFilter::Linear)
        );
    }

    #[test]
    fn sampler_mipmap_filter_maps_to_the_matching_wgpu_mode() {
        assert_eq!(
            SamplerMipmapFilter::Nearest.to_wgpu(),
            wgpu::MipmapFilterMode::Nearest
        );
        assert_eq!(
            SamplerMipmapFilter::Linear.to_wgpu(),
            wgpu::MipmapFilterMode::Linear
        );
    }

    #[test]
    fn srgb_transfer_functions_round_trip_at_the_endpoints() {
        for channel in [0.0f64, 1.0] {
            let round_trip = linear_to_srgb_f64(srgb_to_linear_f64(channel));
            assert!(
                (round_trip - channel).abs() < 1e-12,
                "round trip must be exact at {channel}, got {round_trip}"
            );
        }
    }

    #[test]
    fn srgb_transfer_functions_use_the_piecewise_linear_segment() {
        // Below the 0.04045 knee the curve is exactly the 12.92 linear segment,
        // so a dark channel must not pick up any gamma curvature.
        let small = 0.03f64;
        assert!((srgb_to_linear_f64(small) - small / 12.92).abs() < 1e-15);
        let dark_linear = 0.002f64;
        assert!((linear_to_srgb_f64(dark_linear) - dark_linear * 12.92).abs() < 1e-15);
    }

    #[test]
    fn srgb_decode_is_monotonic_and_bounded() {
        let mut previous = f64::NEG_INFINITY;
        for step in 0..=1000 {
            let c = f64::from(step) / 1000.0;
            let linear = srgb_to_linear_f64(c);
            assert!(linear.is_finite() && (0.0..=1.0).contains(&linear));
            assert!(linear >= previous, "decode must be monotonic at {c}");
            previous = linear;
        }
    }

    #[test]
    fn srgb_transfer_functions_clamp_out_of_range_input() {
        // Tolerance, not equality: `1.055 * 1.0 - 0.055` evaluates to
        // 0.9999999999999999 in f64 because neither literal is exactly
        // representable. The 8-bit quantization the mip chain actually uses
        // still lands on 255 (see terrain_textures' round-trip test).
        assert!((srgb_to_linear_f64(-0.5) - 0.0).abs() < 1e-12);
        assert!((srgb_to_linear_f64(1.5) - 1.0).abs() < 1e-12);
        assert!((linear_to_srgb_f64(-0.5) - 0.0).abs() < 1e-12);
        assert!((linear_to_srgb_f64(1.5) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn row_padding_arithmetic_for_various_widths() {
        // Width 1: 4 bytes → padded to 256.
        assert_eq!(padded_bytes_per_row(1), 256);
        // Width 64: 256 bytes → already aligned.
        assert_eq!(padded_bytes_per_row(64), 256);
        // Width 100: 400 bytes → padded to 512.
        assert_eq!(padded_bytes_per_row(100), 512);
        // Width 256: 1024 bytes → already aligned.
        assert_eq!(padded_bytes_per_row(256), 1024);
    }
}
