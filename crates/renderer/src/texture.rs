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

/// Compute the padded bytes_per_row for GPU upload.
///
/// WebGPU requires each row to be aligned to `COPY_BYTES_PER_ROW_ALIGNMENT`.
/// For RGBA8 (4 bytes per pixel), if `width * 4` is not a multiple of 256,
/// we must pad to the next multiple.
#[must_use]
pub fn padded_bytes_per_row(width: u32) -> u32 {
    let unpadded = width * 4;
    let remainder = unpadded % COPY_BYTES_PER_ROW_ALIGNMENT;
    if remainder == 0 {
        unpadded
    } else {
        unpadded + (COPY_BYTES_PER_ROW_ALIGNMENT - remainder)
    }
}

/// Create a staging buffer with row padding for GPU upload.
///
/// Returns the padded data and the padded bytes_per_row.
#[must_use]
pub fn create_staging_buffer(texture: &DecodedTexture) -> (Vec<u8>, u32) {
    let padded_row_bytes = padded_bytes_per_row(texture.width);
    let unpadded_row_bytes = texture.width * 4;
    let padding_per_row = (padded_row_bytes - unpadded_row_bytes) as usize;

    if padding_per_row == 0 {
        // No padding needed.
        return (texture.rgba8.clone(), padded_row_bytes);
    }

    let mut staged = Vec::with_capacity((padded_row_bytes * texture.height) as usize);
    for row in 0..texture.height as usize {
        let start = row * unpadded_row_bytes as usize;
        let end = start + unpadded_row_bytes as usize;
        staged.extend_from_slice(&texture.rgba8[start..end]);
        staged.extend(std::iter::repeat_n(0u8, padding_per_row));
    }

    (staged, padded_row_bytes)
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

/// glTF sampler min/mag filter mapping.
///
/// G1C limitation: Mipmap filters are mapped to their non-mipmap equivalents.
/// A single-mip implementation is used; the distinction between NEAREST and
/// LINEAR for minification is preserved, but mipmap selection is not.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SamplerFilter {
    Nearest,
    Linear,
}

impl SamplerFilter {
    /// Map from glTF min filter, discarding mipmap distinctions.
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

/// Sampler configuration extracted from a glTF sampler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SamplerConfig {
    pub wrap_s: SamplerWrap,
    pub wrap_t: SamplerWrap,
    pub min_filter: SamplerFilter,
    pub mag_filter: SamplerFilter,
}

impl SamplerConfig {
    /// Default sampler: linear filtering, repeat wrapping.
    #[must_use]
    pub fn default_sampler() -> Self {
        Self {
            wrap_s: SamplerWrap::Repeat,
            wrap_t: SamplerWrap::Repeat,
            min_filter: SamplerFilter::Linear,
            mag_filter: SamplerFilter::Linear,
        }
    }

    /// Extract from a glTF sampler, applying G1C mipmap limitations.
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
        Self {
            wrap_s,
            wrap_t,
            min_filter,
            mag_filter,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn padded_bytes_per_row_returns_unpadded_when_aligned() {
        // 64 pixels * 4 bytes = 256 bytes, already aligned.
        assert_eq!(padded_bytes_per_row(64), 256);
    }

    #[test]
    fn padded_bytes_per_row_pads_to_alignment() {
        // 1 pixel * 4 bytes = 4 bytes, needs padding to 256.
        assert_eq!(padded_bytes_per_row(1), 256);
        // 63 pixels * 4 bytes = 252 bytes, needs padding to 256.
        assert_eq!(padded_bytes_per_row(63), 256);
        // 65 pixels * 4 bytes = 260 bytes, needs padding to 512.
        assert_eq!(padded_bytes_per_row(65), 512);
    }

    #[test]
    fn create_staging_buffer_no_padding_needed() {
        let texture = DecodedTexture {
            width: 64,
            height: 2,
            rgba8: vec![0xAB; 64 * 2 * 4],
        };
        let (staged, bytes_per_row) = create_staging_buffer(&texture);
        assert_eq!(bytes_per_row, 256);
        assert_eq!(staged.len(), 256 * 2);
        // Data should be unchanged.
        assert_eq!(&staged[..256], &texture.rgba8[..256]);
    }

    #[test]
    fn create_staging_buffer_with_padding() {
        let texture = DecodedTexture {
            width: 1,
            height: 2,
            rgba8: vec![0xAB; 8], // 4 bytes per row * 2 rows
        };
        let (staged, bytes_per_row) = create_staging_buffer(&texture);
        assert_eq!(bytes_per_row, 256);
        assert_eq!(staged.len(), 256 * 2);
        // First row: 4 bytes of data + 252 bytes of padding.
        assert_eq!(&staged[..4], &[0xAB; 4]);
        assert_eq!(&staged[4..256], &[0; 252]);
        // Second row: same pattern.
        assert_eq!(&staged[256..260], &[0xAB; 4]);
        assert_eq!(&staged[260..512], &[0; 252]);
    }

    #[test]
    fn decode_empty_data_returns_error() {
        let result = decode_image(&[]);
        assert!(matches!(result, Err(TextureLoadError::EmptyImageData)));
    }

    #[test]
    fn decode_invalid_data_returns_error() {
        let result = decode_image(&[0xFF, 0xD8, 0xFF]); // Truncated JPEG header.
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
        // All NEAREST variants map to Nearest.
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
        // All LINEAR variants map to Linear.
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
    }
}
