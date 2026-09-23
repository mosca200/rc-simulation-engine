//! ENV1-A: deterministic processing of CC0 photographic source maps into the
//! runtime terrain material set.
//!
//! # Purpose
//!
//! The G3A/PV2 terrain material was textured with three maps produced by this
//! repository's own procedural generator (`terrain_textures::generate_terrain_textures`).
//! ENV1 replaces that *source content* with a processed photographic PBR material
//! (Poly Haven `sparse_grass`, CC0) while leaving the terrain's geometry, UV
//! mapping, three-frequency stack, anti-repetition transform and shader material
//! path untouched.
//!
//! This module owns the source -> runtime conversion only. It is a
//! development-time recipe: it runs from `src/bin/process_env1_terrain_material.rs`,
//! never from the frame loop, and the renderer consumes its committed output
//! through `include_bytes!`.
//!
//! # Recipe (version 1)
//!
//! The Poly Haven 4k sources are 16-bit PNGs (`bit_depth=16`): `Diffuse` and
//! `nor_gl` are 16-bit truecolor RGB, `Rough` is 16-bit grayscale. Each map is
//! reduced 4096 -> 2048 with a single exact 2x2 box filter computed in the
//! correct color space, then quantized once to 8 bits:
//!
//! - **base color** (`Diffuse`): every source texel is decoded sRGB -> linear,
//!   the four linear values are averaged, and the mean is re-encoded to sRGB
//!   before quantization. Averaging in linear light is what keeps a minified
//!   photograph from darkening; the transfer function is the crate's single
//!   canonical `texture::srgb_to_linear_f64` / `linear_to_srgb_f64`, the same
//!   pair the runtime mip chain uses. Output is RGBA8 with alpha = 255: the
//!   source carries no alpha channel, so there is nothing to preserve.
//! - **normal** (`nor_gl`): linear data, already tangent-space in OpenGL
//!   orientation (+Y up), which is the convention the terrain shader decodes.
//!   The four vectors are decoded to [-1, 1], summed and renormalized — never
//!   averaged as raw bytes and never flattened — then re-encoded. A degenerate
//!   near-zero sum falls back to flat-up `(0, 0, 1)`. Output RGBA8, alpha = 255.
//! - **roughness** (`Rough`): linear single-channel data reduced with exact
//!   `u32` integer arithmetic (sum, half-up divide by 4, then an exact
//!   16-bit -> 8-bit rescale). No floating point is involved, so the result is
//!   bit-identical on every platform and toolchain. Output 8-bit grayscale,
//!   which the runtime uploads as `R8Unorm`.
//!
//! # Deliberately not applied
//!
//! No saturation or contrast boost, no baked shadow, no ambient-occlusion
//! multiplication into the base color, no cosmetic sharpening, no LUT, and no
//! displacement. The only geometric assumption is the 2:1 source-to-target
//! ratio; a source that is not exactly `ENV1_SOURCE_EDGE` square is rejected
//! rather than silently resampled with some other filter.
//!
//! # Determinism
//!
//! The recipe is a pure function of the source pixels: the same 4k inputs
//! always produce byte-identical 2048 outputs. Roughness is integer-exact;
//! base color and normal use `f64` throughout with a single final rounding
//! step, matching the existing `terrain_textures` mip chain.

use crate::terrain_textures::TerrainTextureSet;
use crate::texture::{linear_to_srgb_f64, srgb_to_linear_f64};
use image::{DynamicImage, ImageBuffer, Luma, Rgba};
use std::path::Path;
use thiserror::Error;

/// Edge length of the Poly Haven source maps this recipe consumes (the `4k`
/// variant: 4096x4096, 16-bit PNG).
pub const ENV1_SOURCE_EDGE: u32 = 4096;

/// Edge length of the committed runtime maps (2048x2048).
pub const ENV1_RUNTIME_EDGE: u32 = 2048;

/// Maximum value of a 16-bit PNG sample.
const U16_MAX_F64: f64 = 65535.0;

/// Runtime file names, relative to the ENV1 terrain asset directory.
pub const BASE_COLOR_FILE_NAME: &str = "sparse_grass_base_color.png";
/// Runtime file name of the tangent-space normal map.
pub const NORMAL_FILE_NAME: &str = "sparse_grass_normal.png";
/// Runtime file name of the single-channel roughness map.
pub const ROUGHNESS_FILE_NAME: &str = "sparse_grass_roughness.png";

/// A 16-bit three-channel source map, row-major, `width * height * 3` samples.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rgb16Map {
    /// Source width in texels.
    pub width: u32,
    /// Source height in texels.
    pub height: u32,
    /// Interleaved RGB samples at 16-bit precision.
    pub samples: Vec<u16>,
}

/// A 16-bit single-channel source map, row-major, `width * height` samples.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Luma16Map {
    /// Source width in texels.
    pub width: u32,
    /// Source height in texels.
    pub height: u32,
    /// Grayscale samples at 16-bit precision.
    pub samples: Vec<u16>,
}

/// The processed runtime maps, ready to be written as PNG and embedded.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Env1RuntimeMaps {
    /// Runtime edge length (both maps are square).
    pub edge: u32,
    /// RGBA8 base color, sRGB-encoded, alpha = 255.
    pub base_color_rgba8: Vec<u8>,
    /// RGBA8 tangent-space normal (OpenGL orientation), linear, alpha = 255.
    pub normal_rgba8: Vec<u8>,
    /// R8 roughness, linear.
    pub roughness_r8: Vec<u8>,
}

/// Where the runtime maps were written.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Env1RuntimePaths {
    /// Base color PNG.
    pub base_color: std::path::PathBuf,
    /// Normal PNG.
    pub normal: std::path::PathBuf,
    /// Roughness PNG.
    pub roughness: std::path::PathBuf,
}

/// Every way the ENV1-A source processing can fail. All variants fail closed:
/// a source that is not exactly what the recipe documents is an error, never a
/// silent fallback.
#[derive(Debug, Error)]
pub enum Env1MaterialError {
    /// The source file could not be read.
    #[error("cannot read ENV1 source map {path}: {reason}")]
    Io {
        /// Path that failed.
        path: String,
        /// Underlying I/O reason.
        reason: String,
    },
    /// The file is not a decodable image.
    #[error("cannot decode ENV1 source map {path}: {reason}")]
    Decode {
        /// Path that failed.
        path: String,
        /// Decoder reason.
        reason: String,
    },
    /// The decoded image is not the documented 16-bit layout.
    #[error("ENV1 source map {path} must be {expected}, found {found}")]
    UnexpectedPixelFormat {
        /// Path that failed.
        path: String,
        /// What the recipe requires.
        expected: &'static str,
        /// What the file actually is.
        found: String,
    },
    /// The source is not the 4k square the 2:1 recipe requires.
    #[error(
        "ENV1 source map {path} is {width}x{height}; the recipe requires exactly \
         {expected}x{expected} so the reduction is one exact 2x2 box step"
    )]
    UnexpectedDimensions {
        /// Path that failed.
        path: String,
        /// Actual width.
        width: u32,
        /// Actual height.
        height: u32,
        /// Required edge length.
        expected: u32,
    },
    /// A runtime PNG could not be written.
    #[error("cannot write ENV1 runtime map {path}: {reason}")]
    Write {
        /// Path that failed.
        path: String,
        /// Encoder reason.
        reason: String,
    },
}

/// Load a 16-bit truecolor RGB PNG, rejecting any other layout.
///
/// # Errors
///
/// Fails if the file cannot be read or decoded, or if it is not 16-bit RGB.
pub fn load_rgb16(path: &Path) -> Result<Rgb16Map, Env1MaterialError> {
    let displayed = path.display().to_string();
    let image = open_image(path, &displayed)?;
    match image {
        DynamicImage::ImageRgb16(rgb) => {
            let (width, height) = (rgb.width(), rgb.height());
            Ok(Rgb16Map {
                width,
                height,
                samples: rgb.into_raw(),
            })
        }
        other => Err(Env1MaterialError::UnexpectedPixelFormat {
            path: displayed,
            expected: "16-bit truecolor RGB (PNG bit_depth=16, color_type=2)",
            found: describe(&other),
        }),
    }
}

/// Load a 16-bit grayscale PNG, rejecting any other layout.
///
/// # Errors
///
/// Fails if the file cannot be read or decoded, or if it is not 16-bit gray.
pub fn load_luma16(path: &Path) -> Result<Luma16Map, Env1MaterialError> {
    let displayed = path.display().to_string();
    let image = open_image(path, &displayed)?;
    match image {
        DynamicImage::ImageLuma16(luma) => {
            let (width, height) = (luma.width(), luma.height());
            Ok(Luma16Map {
                width,
                height,
                samples: luma.into_raw(),
            })
        }
        other => Err(Env1MaterialError::UnexpectedPixelFormat {
            path: displayed,
            expected: "16-bit grayscale (PNG bit_depth=16, color_type=0)",
            found: describe(&other),
        }),
    }
}

/// Load an RGB source map at 16-bit precision, accepting either a 16-bit
/// truecolor PNG or an 8-bit truecolor/RGBA PNG.
///
/// Poly Haven publishes 16-bit PNGs for some assets only (`sparse_grass` is
/// 16-bit, `grass_path_3` and `forest_ground_04` are 8-bit). An 8-bit source is
/// expanded to 16 bits by exact code-value replication (`v * 257`), so the
/// decoded linear value of every texel is identical to decoding the 8-bit
/// sample directly: the single 2x2 reduction below stays the only place
/// quantization happens, and one recipe serves every registered asset.
///
/// # Errors
///
/// Fails if the file cannot be read or decoded, or if it is not truecolor.
pub fn load_rgb_source(path: &Path) -> Result<Rgb16Map, Env1MaterialError> {
    let displayed = path.display().to_string();
    let image = open_image(path, &displayed)?;
    match image {
        DynamicImage::ImageRgb16(rgb) => {
            let (width, height) = (rgb.width(), rgb.height());
            Ok(Rgb16Map {
                width,
                height,
                samples: rgb.into_raw(),
            })
        }
        DynamicImage::ImageRgb8(rgb) => {
            let (width, height) = (rgb.width(), rgb.height());
            let samples = rgb
                .into_raw()
                .into_iter()
                .map(|v| u16::from(v) * 257)
                .collect();
            Ok(Rgb16Map {
                width,
                height,
                samples,
            })
        }
        DynamicImage::ImageRgba8(rgba) => {
            let (width, height) = (rgba.width(), rgba.height());
            let raw = rgba.into_raw();
            // Drop the alpha lane: the diffuse source alpha is not transferred
            // (runtime base color is opaque by recipe).
            let samples = raw
                .iter()
                .enumerate()
                .filter(|(index, _)| (index & 3) != 3)
                .map(|(_, &value)| u16::from(value) * 257)
                .collect();
            Ok(Rgb16Map {
                width,
                height,
                samples,
            })
        }
        other => Err(Env1MaterialError::UnexpectedPixelFormat {
            path: displayed,
            expected: "16-bit or 8-bit truecolor RGB (PNG color_type 2 or 6)",
            found: describe(&other),
        }),
    }
}

/// Load a grayscale source map at 16-bit precision, accepting 16-bit or 8-bit.
///
/// See [`load_rgb_source`] for why an 8-bit source is expanded by `v * 257`.
///
/// # Errors
///
/// Fails if the file cannot be read or decoded, or if it is not grayscale.
pub fn load_luma_source(path: &Path) -> Result<Luma16Map, Env1MaterialError> {
    let displayed = path.display().to_string();
    let image = open_image(path, &displayed)?;
    match image {
        DynamicImage::ImageLuma16(luma) => {
            let (width, height) = (luma.width(), luma.height());
            Ok(Luma16Map {
                width,
                height,
                samples: luma.into_raw(),
            })
        }
        DynamicImage::ImageLuma8(luma) => {
            let (width, height) = (luma.width(), luma.height());
            let samples = luma
                .into_raw()
                .into_iter()
                .map(|v| u16::from(v) * 257)
                .collect();
            Ok(Luma16Map {
                width,
                height,
                samples,
            })
        }
        other => Err(Env1MaterialError::UnexpectedPixelFormat {
            path: displayed,
            expected: "16-bit or 8-bit grayscale (PNG color_type 0)",
            found: describe(&other),
        }),
    }
}

fn open_image(path: &Path, displayed: &str) -> Result<DynamicImage, Env1MaterialError> {
    match image::open(path) {
        Ok(image) => Ok(image),
        Err(error) => {
            let reason = error.to_string();
            Err(match error {
                image::ImageError::IoError(_) => Env1MaterialError::Io {
                    path: displayed.to_owned(),
                    reason,
                },
                _ => Env1MaterialError::Decode {
                    path: displayed.to_owned(),
                    reason,
                },
            })
        }
    }
}

fn describe(image: &DynamicImage) -> String {
    let (width, height) = (image.width(), image.height());
    format!("{width}x{height} {:?}", image.color())
}

/// Exact 2x2 box reduction of a 16-bit sRGB RGB map to 8-bit RGBA.
///
/// Each output texel decodes its four source texels to linear light, averages
/// them, and re-encodes once. Alpha is written as 255 because the Poly Haven
/// diffuse source carries no alpha channel.
///
/// # Panics
///
/// Panics if the source is not an even-sided square, or if its sample count
/// does not match `width * height * 3`.
#[must_use]
pub fn halve_base_color_srgb(source: &Rgb16Map) -> Vec<u8> {
    let (width, height) = even_square(source.width, source.height, "base color");
    expect_samples(source.samples.len(), width, height, 3, "base color");
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h * 4) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut acc = [0.0f64; 3];
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                let i = (((2 * oy + dy) * width + (2 * ox + dx)) * 3) as usize;
                for (slot, sample) in acc.iter_mut().zip(&source.samples[i..i + 3]) {
                    *slot += srgb_to_linear_f64(f64::from(*sample) / U16_MAX_F64);
                }
            }
            let oi = ((oy * out_w + ox) * 4) as usize;
            for channel in 0..3 {
                out[oi + channel] = quantize_unit(linear_to_srgb_f64(acc[channel] * 0.25));
            }
            out[oi + 3] = 255;
        }
    }
    out
}

/// Exact 2x2 box reduction of a 16-bit tangent-space normal map to 8-bit RGBA.
///
/// The four vectors are decoded to [-1, 1], summed and renormalized so relief
/// survives minification instead of washing out toward flat. A near-zero sum
/// (only reachable with opposing corners) falls back to flat-up.
///
/// # Panics
///
/// Panics if the source is not an even-sided square, or if its sample count
/// does not match `width * height * 3`.
#[must_use]
pub fn halve_normal(source: &Rgb16Map) -> Vec<u8> {
    let (width, height) = even_square(source.width, source.height, "normal");
    expect_samples(source.samples.len(), width, height, 3, "normal");
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h * 4) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut sum = [0.0f64; 3];
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                let i = (((2 * oy + dy) * width + (2 * ox + dx)) * 3) as usize;
                for (slot, sample) in sum.iter_mut().zip(&source.samples[i..i + 3]) {
                    *slot += (f64::from(*sample) / U16_MAX_F64) * 2.0 - 1.0;
                }
            }
            let length_sq = sum.iter().map(|c| c * c).sum::<f64>();
            let normal = if length_sq > 1e-12 {
                let inv = length_sq.sqrt().recip();
                [sum[0] * inv, sum[1] * inv, sum[2] * inv]
            } else {
                [0.0, 0.0, 1.0]
            };
            let oi = ((oy * out_w + ox) * 4) as usize;
            for channel in 0..3 {
                out[oi + channel] = quantize_unit(normal[channel] * 0.5 + 0.5);
            }
            out[oi + 3] = 255;
        }
    }
    out
}

/// Exact 2x2 box reduction of a 16-bit linear grayscale map to 8-bit R8.
///
/// Pure `u32` integer arithmetic: sum the four samples, divide by four with
/// half-up rounding, then rescale 16-bit -> 8-bit exactly. No floating point,
/// so the output is bit-identical on every platform.
///
/// # Panics
///
/// Panics if the source is not an even-sided square, or if its sample count
/// does not match `width * height`.
#[must_use]
pub fn halve_roughness(source: &Luma16Map) -> Vec<u8> {
    let (width, height) = even_square(source.width, source.height, "roughness");
    expect_samples(source.samples.len(), width, height, 1, "roughness");
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut acc = 0u32;
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                acc += u32::from(source.samples[((2 * oy + dy) * width + (2 * ox + dx)) as usize]);
            }
            // Half-up average, then an exact 65535 -> 255 rescale.
            let mean16 = (acc + 2) / 4;
            out[(oy * out_w + ox) as usize] = ((mean16 * 255 + 32_767) / 65_535) as u8;
        }
    }
    out
}

/// Run the full ENV1-A recipe over the three 4k sources.
///
/// # Errors
///
/// Fails closed if any source is not exactly `ENV1_SOURCE_EDGE` square, or if a
/// sample count does not match its dimensions.
pub fn process_sparse_grass(
    base_color: &Rgb16Map,
    normal: &Rgb16Map,
    roughness: &Luma16Map,
) -> Result<Env1RuntimeMaps, Env1MaterialError> {
    require_source_edge(base_color.width, base_color.height, "Diffuse")?;
    require_source_edge(normal.width, normal.height, "nor_gl")?;
    require_source_edge(roughness.width, roughness.height, "Rough")?;

    Ok(Env1RuntimeMaps {
        edge: ENV1_RUNTIME_EDGE,
        base_color_rgba8: halve_base_color_srgb(base_color),
        normal_rgba8: halve_normal(normal),
        roughness_r8: halve_roughness(roughness),
    })
}

/// Bridge the processed maps into the terrain texture set the runtime mip
/// chain consumes, so the ENV1 material reuses the existing (already tested)
/// `generate_terrain_mip_chain` path with no architectural duplication.
#[must_use]
pub fn runtime_texture_set(maps: &Env1RuntimeMaps) -> TerrainTextureSet {
    TerrainTextureSet {
        albedo_rgba: maps.base_color_rgba8.clone(),
        normal_rgba: maps.normal_rgba8.clone(),
        roughness_r8: maps.roughness_r8.clone(),
    }
}

/// Write the runtime maps as PNGs into `directory`, creating it if needed.
///
/// Base color and normal are written as RGBA8 (PNG color type 6) and roughness
/// as 8-bit grayscale (color type 0), matching the existing committed terrain
/// maps byte-for-byte in layout so the runtime decode path is unchanged.
///
/// # Errors
///
/// Fails if the directory cannot be created or a PNG cannot be encoded.
pub fn write_runtime_maps(
    maps: &Env1RuntimeMaps,
    directory: &Path,
) -> Result<Env1RuntimePaths, Env1MaterialError> {
    write_runtime_maps_with_names(
        maps,
        directory,
        &(BASE_COLOR_FILE_NAME, NORMAL_FILE_NAME, ROUGHNESS_FILE_NAME),
    )
}

/// [`write_runtime_maps`] with explicit runtime file names, so one recipe can
/// serve every registered ground material (Flying Field v1 adds the worn-grass
/// and dry-soil layers alongside the maintained `sparse_grass` base).
///
/// # Errors
///
/// Fails if the directory cannot be created or a PNG cannot be encoded.
pub fn write_runtime_maps_with_names(
    maps: &Env1RuntimeMaps,
    directory: &Path,
    names: &(&str, &str, &str),
) -> Result<Env1RuntimePaths, Env1MaterialError> {
    std::fs::create_dir_all(directory).map_err(|error| Env1MaterialError::Io {
        path: directory.display().to_string(),
        reason: error.to_string(),
    })?;

    let edge = maps.edge;
    let base_color = directory.join(names.0);
    let normal = directory.join(names.1);
    let roughness = directory.join(names.2);

    save_rgba8(&base_color, edge, &maps.base_color_rgba8)?;
    save_rgba8(&normal, edge, &maps.normal_rgba8)?;

    let roughness_image =
        ImageBuffer::<Luma<u8>, _>::from_raw(edge, edge, maps.roughness_r8.clone()).ok_or_else(
            || Env1MaterialError::Write {
                path: roughness.display().to_string(),
                reason: "roughness buffer length does not match edge * edge".to_owned(),
            },
        )?;
    roughness_image
        .save(&roughness)
        .map_err(|error| write_error(&roughness, &error))?;

    Ok(Env1RuntimePaths {
        base_color,
        normal,
        roughness,
    })
}

fn save_rgba8(path: &Path, edge: u32, bytes: &[u8]) -> Result<(), Env1MaterialError> {
    let image =
        ImageBuffer::<Rgba<u8>, _>::from_raw(edge, edge, bytes.to_vec()).ok_or_else(|| {
            Env1MaterialError::Write {
                path: path.display().to_string(),
                reason: "buffer length does not match edge * edge * 4".to_owned(),
            }
        })?;
    image.save(path).map_err(|error| write_error(path, &error))
}

fn write_error(path: &Path, error: &image::ImageError) -> Env1MaterialError {
    Env1MaterialError::Write {
        path: path.display().to_string(),
        reason: error.to_string(),
    }
}

fn require_source_edge(
    width: u32,
    height: u32,
    label: &'static str,
) -> Result<(), Env1MaterialError> {
    if width == ENV1_SOURCE_EDGE && height == ENV1_SOURCE_EDGE {
        Ok(())
    } else {
        Err(Env1MaterialError::UnexpectedDimensions {
            path: label.to_owned(),
            width,
            height,
            expected: ENV1_SOURCE_EDGE,
        })
    }
}

/// Quantize a unit-range value to 8 bits with half-away-from-zero rounding.
fn quantize_unit(value: f64) -> u8 {
    (value.clamp(0.0, 1.0) * 255.0).round() as u8
}

fn even_square(width: u32, height: u32, label: &str) -> (u32, u32) {
    assert_eq!(
        width, height,
        "{label} source must be square, got {width}x{height}"
    );
    assert!(
        width.is_multiple_of(2) && width >= 2,
        "{label} source edge must be an even number >= 2, got {width}"
    );
    (width, height)
}

fn expect_samples(len: usize, width: u32, height: u32, channels: u32, label: &str) {
    let expected = (width as usize) * (height as usize) * (channels as usize);
    assert_eq!(
        len, expected,
        "{label} source must carry {expected} samples ({width}x{height}x{channels}), got {len}"
    );
}

/// Committed ENV1-A runtime maps, embedded at compile time.
///
/// Written by `src/bin/process_env1_terrain_material.rs` from the Poly Haven
/// `sparse_grass` 4k sources; never edited by hand. The constant names mirror
/// `terrain_textures::generated` so the renderer's terrain material path is a
/// one-line repoint rather than a rewrite. Provenance, source digests and the
/// output digests are recorded in `docs/assets/env1/env1_open_assets.json` and
/// re-checked by `tools/env1_asset_pipeline/verify_env1_assets.py`.
pub mod runtime_assets {
    /// Base color, RGBA8, sRGB intent (sampled as `Rgba8UnormSrgb`).
    pub const TERRAIN_ALBEDO_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/sparse_grass/sparse_grass_base_color.png");
    /// Tangent-space normal, RGBA8, linear, OpenGL (+Y) orientation.
    pub const TERRAIN_NORMAL_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/sparse_grass/sparse_grass_normal.png");
    /// Roughness, 8-bit grayscale, linear R channel (uploaded as `R8Unorm`).
    pub const TERRAIN_ROUGHNESS_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/sparse_grass/sparse_grass_roughness.png");
}

/// Committed FFV1 worn-grass companion maps (Poly Haven `grass_path_3`, CC0,
/// ENV1-GND-02), embedded at compile time. Written by
/// `src/bin/process_env1_terrain_material.rs --asset grass_path_3`; provenance
/// in `docs/assets/env1/env1_open_assets.json`.
pub mod runtime_assets_worn {
    /// Base color, RGBA8, sRGB intent (sampled as `Rgba8UnormSrgb`).
    pub const TERRAIN_ALBEDO_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/grass_path_3/grass_path_3_base_color.png");
    /// Tangent-space normal, RGBA8, linear, OpenGL (+Y) orientation.
    pub const TERRAIN_NORMAL_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/grass_path_3/grass_path_3_normal.png");
    /// Roughness, 8-bit grayscale, linear R channel (uploaded as `R8Unorm`).
    pub const TERRAIN_ROUGHNESS_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/grass_path_3/grass_path_3_roughness.png");
}

/// Committed FFV1 dry-soil companion maps (Poly Haven `forest_ground_04`, CC0,
/// ENV1-GND-03), embedded at compile time. Written by
/// `src/bin/process_env1_terrain_material.rs --asset forest_ground_04`;
/// provenance in `docs/assets/env1/env1_open_assets.json`.
pub mod runtime_assets_dry {
    /// Base color, RGBA8, sRGB intent (sampled as `Rgba8UnormSrgb`).
    pub const TERRAIN_ALBEDO_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/forest_ground_04/forest_ground_04_base_color.png");
    /// Tangent-space normal, RGBA8, linear, OpenGL (+Y) orientation.
    pub const TERRAIN_NORMAL_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/forest_ground_04/forest_ground_04_normal.png");
    /// Roughness, 8-bit grayscale, linear R channel (uploaded as `R8Unorm`).
    pub const TERRAIN_ROUGHNESS_PNG: &[u8] =
        include_bytes!("../assets/env1/terrain/forest_ground_04/forest_ground_04_roughness.png");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain_textures::{generate_terrain_mip_chain, mip_level_count_for_size};

    /// A constant-color 16-bit RGB source of the given edge length.
    fn flat_rgb16(edge: u32, rgb: [u16; 3]) -> Rgb16Map {
        let mut samples = Vec::with_capacity((edge * edge * 3) as usize);
        for _ in 0..(edge * edge) {
            samples.extend_from_slice(&rgb);
        }
        Rgb16Map {
            width: edge,
            height: edge,
            samples,
        }
    }

    fn flat_luma16(edge: u32, value: u16) -> Luma16Map {
        Luma16Map {
            width: edge,
            height: edge,
            samples: vec![value; (edge * edge) as usize],
        }
    }

    #[test]
    fn runtime_edge_is_exactly_half_the_source_edge() {
        assert_eq!(ENV1_SOURCE_EDGE, 4096);
        assert_eq!(ENV1_RUNTIME_EDGE, 2048);
        assert_eq!(ENV1_SOURCE_EDGE / 2, ENV1_RUNTIME_EDGE);
        assert!(ENV1_RUNTIME_EDGE.is_power_of_two());
    }

    #[test]
    fn base_color_reduction_halves_both_dimensions() {
        let out = halve_base_color_srgb(&flat_rgb16(8, [0; 3]));
        assert_eq!(out.len(), 4 * 4 * 4);
    }

    #[test]
    fn base_color_reduction_is_bitwise_deterministic() {
        // A non-constant source exercises the real arithmetic path.
        let edge = 16u32;
        let mut source = flat_rgb16(edge, [0; 3]);
        for (index, sample) in source.samples.iter_mut().enumerate() {
            *sample = ((index as u64 * 7919) % 65536) as u16;
        }
        let first = halve_base_color_srgb(&source);
        let second = halve_base_color_srgb(&source);
        assert_eq!(first, second, "processing must be deterministic");
    }

    #[test]
    fn base_color_of_a_flat_source_is_the_exact_16_to_8_bit_rescale() {
        // With four identical texels the linear average is the texel itself, so
        // the result must equal round(v / 257) with no filtering error.
        for value in [0u16, 1, 257, 32_768, 65_535] {
            let out = halve_base_color_srgb(&flat_rgb16(4, [value; 3]));
            let expected = ((f64::from(value) / 257.0).round()) as u8;
            assert_eq!(out[0], expected, "value {value} -> {}", out[0]);
            assert_eq!(out[1], expected);
            assert_eq!(out[2], expected);
        }
    }

    #[test]
    fn base_color_extremes_are_exact() {
        let black = halve_base_color_srgb(&flat_rgb16(4, [0; 3]));
        assert_eq!(&black[..3], &[0, 0, 0]);
        let white = halve_base_color_srgb(&flat_rgb16(4, [65_535; 3]));
        assert_eq!(&white[..3], &[255, 255, 255]);
    }

    #[test]
    fn base_color_averages_in_linear_light_not_in_srgb() {
        // Two black and two white texels: averaging the encoded values would
        // give 128, while averaging in linear light gives the sRGB encoding of
        // 0.5, which is 188. This is the assertion that the color space is
        // actually honoured.
        let mut source = flat_rgb16(2, [0; 3]);
        source.samples = vec![
            0, 0, 0, // (0,0) black
            65_535, 65_535, 65_535, // (1,0) white
            0, 0, 0, // (0,1) black
            65_535, 65_535, 65_535, // (1,1) white
        ];
        let out = halve_base_color_srgb(&source);
        let linear_mean = 0.5f64;
        let expected = quantize_unit(linear_to_srgb_f64(linear_mean));
        assert_eq!(out[0], expected);
        assert!(
            expected > 128,
            "linear-light averaging must be brighter than an sRGB average, got {expected}"
        );
    }

    #[test]
    fn base_color_alpha_is_opaque_because_the_source_has_no_alpha() {
        let out = halve_base_color_srgb(&flat_rgb16(8, [12_345; 3]));
        assert!(
            out.iter().skip(3).step_by(4).all(|&a| a == 255),
            "every base-color alpha must be 255"
        );
    }

    #[test]
    fn normal_reduction_keeps_flat_up_exactly() {
        // (0.5, 0.5, 1.0) encoded at 16 bits is the flat tangent-space normal in
        // the OpenGL convention the terrain shader decodes.
        let out = halve_normal(&flat_rgb16(8, [32_768, 32_768, 65_535]));
        assert_eq!(&out[..3], &[128, 128, 255]);
        assert_eq!(out[3], 255);
    }

    #[test]
    fn normal_reduction_renormalizes_instead_of_averaging_bytes() {
        // +Z and +X are both unit vectors; their mean has length 1/sqrt(2). A
        // byte average would keep that shortened vector and visibly flatten the
        // relief, while a renormalized vector sum must stay on the unit sphere.
        let plus_z: [u16; 3] = [32_768, 32_768, 65_535];
        let plus_x: [u16; 3] = [65_535, 32_768, 32_768];
        let mut source = flat_rgb16(2, [0; 3]);
        source.samples = [plus_z, plus_x, plus_z, plus_x].concat();

        let out = halve_normal(&source);

        // Renormalized: (1, 0, 1)/sqrt(2) -> encode -> 218, with Y at the
        // 16-bit midpoint decoding to ~0 and therefore landing on 128.
        assert_eq!(&out[..3], &[218, 128, 218]);

        // The plain byte average of the same four texels is materially darker,
        // which is exactly the flattening the renormalization avoids.
        let byte_average: Vec<u8> = (0..3)
            .map(|channel| {
                let sum: u32 = source
                    .samples
                    .iter()
                    .skip(channel)
                    .step_by(3)
                    .map(|&value| u32::from(value))
                    .sum();
                let mean16 = (sum + 2) / 4;
                ((mean16 * 255 + 32_767) / 65_535) as u8
            })
            .collect();
        assert_ne!(
            &out[..3],
            byte_average.as_slice(),
            "renormalizing must differ from averaging bytes"
        );

        // And the decoded output vector really is unit length.
        let decoded: Vec<f64> = out[..3]
            .iter()
            .map(|&byte| f64::from(byte) / 255.0 * 2.0 - 1.0)
            .collect();
        let length = decoded.iter().map(|c| c * c).sum::<f64>().sqrt();
        assert!(
            (length - 1.0).abs() < 0.02,
            "output normal must be unit length, got {length}"
        );
    }

    #[test]
    fn normal_reduction_is_bitwise_deterministic_and_bounded() {
        let edge = 16u32;
        let mut source = flat_rgb16(edge, [0; 3]);
        for (index, sample) in source.samples.iter_mut().enumerate() {
            *sample = ((index as u64 * 104_729) % 65536) as u16;
        }
        let first = halve_normal(&source);
        let second = halve_normal(&source);
        assert_eq!(first, second);
        assert!(
            first.iter().skip(3).step_by(4).all(|&a| a == 255),
            "normal alpha must be opaque"
        );
    }

    #[test]
    fn roughness_reduction_is_exact_integer_arithmetic() {
        // Four samples averaging to exactly 32768 must rescale to round(32768/257).
        let mut source = flat_luma16(2, 0);
        source.samples = vec![32_766, 32_767, 32_768, 32_771];
        let out = halve_roughness(&source);
        let mean16 = (32_766u32 + 32_767 + 32_768 + 32_771 + 2) / 4;
        let expected = ((mean16 * 255 + 32_767) / 65_535) as u8;
        assert_eq!(out.len(), 1);
        assert_eq!(out[0], expected);
    }

    #[test]
    fn roughness_extremes_are_exact() {
        assert_eq!(halve_roughness(&flat_luma16(4, 0))[0], 0);
        assert_eq!(halve_roughness(&flat_luma16(4, 65_535))[0], 255);
    }

    #[test]
    fn roughness_is_monotonic_in_the_source_value() {
        let mut previous = -1i32;
        for step in 0..=64u32 {
            let value = (step * 65_535 / 64) as u16;
            let out = halve_roughness(&flat_luma16(2, value))[0];
            assert!(
                i32::from(out) >= previous,
                "roughness must not decrease at source {value}"
            );
            previous = i32::from(out);
        }
    }

    #[test]
    fn roughness_reduction_is_bitwise_deterministic() {
        let edge = 16u32;
        let mut source = flat_luma16(edge, 0);
        for (index, sample) in source.samples.iter_mut().enumerate() {
            *sample = ((index as u64 * 65_537) % 65536) as u16;
        }
        assert_eq!(halve_roughness(&source), halve_roughness(&source));
    }

    #[test]
    fn full_recipe_produces_the_documented_runtime_layout() {
        let base_color = flat_rgb16(ENV1_SOURCE_EDGE / 64, [40_000; 3]);
        let normal = flat_rgb16(ENV1_SOURCE_EDGE / 64, [32_768, 32_768, 65_535]);
        let roughness = flat_luma16(ENV1_SOURCE_EDGE / 64, 50_000);
        // The recipe is driven at 64x64 here so the unit test stays cheap; the
        // dimension gate itself is covered by the rejection tests below.
        let maps = Env1RuntimeMaps {
            edge: ENV1_SOURCE_EDGE / 128,
            base_color_rgba8: halve_base_color_srgb(&base_color),
            normal_rgba8: halve_normal(&normal),
            roughness_r8: halve_roughness(&roughness),
        };
        let pixels = (maps.edge * maps.edge) as usize;
        assert_eq!(maps.base_color_rgba8.len(), pixels * 4);
        assert_eq!(maps.normal_rgba8.len(), pixels * 4);
        assert_eq!(maps.roughness_r8.len(), pixels);
    }

    #[test]
    fn recipe_rejects_a_source_that_is_not_the_documented_4k_square() {
        let base_color = flat_rgb16(1024, [0; 3]);
        let normal = flat_rgb16(1024, [0; 3]);
        let roughness = flat_luma16(1024, 0);
        let error = process_sparse_grass(&base_color, &normal, &roughness)
            .expect_err("a 1024 source must be rejected");
        assert!(
            matches!(error, Env1MaterialError::UnexpectedDimensions { .. }),
            "unexpected error: {error}"
        );
        assert!(error.to_string().contains("4096x4096"));
    }

    #[test]
    fn source_dimension_gate_accepts_only_the_documented_4k_square() {
        // The full 4k reduction is exercised by the offline processor binary and
        // by the committed-asset digest tripwire in the Python provenance suite;
        // running it here would allocate ~200 MB per test for no extra signal.
        require_source_edge(ENV1_SOURCE_EDGE, ENV1_SOURCE_EDGE, "Diffuse").expect("4k is valid");
        for (width, height) in [
            (0u32, 0u32),
            (1024, 1024),
            (2048, 2048),
            (4095, 4095),
            (4096, 2048),
            (8192, 8192),
        ] {
            let error = require_source_edge(width, height, "Diffuse")
                .expect_err("only the 4k square is accepted");
            assert!(
                matches!(error, Env1MaterialError::UnexpectedDimensions { .. }),
                "unexpected error for {width}x{height}: {error}"
            );
        }
    }

    #[test]
    fn runtime_set_feeds_the_existing_mip_chain_down_to_one_by_one() {
        let edge = 64u32;
        let maps = Env1RuntimeMaps {
            edge: edge / 2,
            base_color_rgba8: halve_base_color_srgb(&flat_rgb16(edge, [40_000; 3])),
            normal_rgba8: halve_normal(&flat_rgb16(edge, [32_768, 32_768, 65_535])),
            roughness_r8: halve_roughness(&flat_luma16(edge, 50_000)),
        };
        let set = runtime_texture_set(&maps);
        let chain = generate_terrain_mip_chain(&set, maps.edge);
        assert_eq!(
            chain.albedo.len(),
            mip_level_count_for_size(maps.edge) as usize
        );
        assert_eq!(chain.normal.len(), chain.albedo.len());
        assert_eq!(chain.roughness.len(), chain.albedo.len());
        // The chain must reach 1x1, and every level must halve.
        assert_eq!(
            (
                chain.albedo.last().expect("last level").width,
                chain.albedo.last().expect("last level").height
            ),
            (1, 1)
        );
        assert_eq!(
            (
                chain.roughness.last().expect("last level").width,
                chain.roughness.last().expect("last level").height
            ),
            (1, 1)
        );
        for levels in [&chain.albedo, &chain.normal, &chain.roughness] {
            let mut expected = maps.edge;
            for level in levels {
                assert_eq!(level.width, expected);
                assert_eq!(level.height, expected);
                expected /= 2;
            }
        }
    }

    #[test]
    fn alpha_is_preserved_through_the_runtime_mip_chain() {
        // ENV1-A requirement for future foliage: alpha must survive the mip
        // chain rather than being forced opaque. Build a set whose albedo alpha
        // alternates 0/255 and check the averaged levels stay in between.
        let edge = 8u32;
        let pixels = (edge * edge) as usize;
        let mut albedo = vec![0u8; pixels * 4];
        for pixel in 0..pixels {
            albedo[pixel * 4] = 200;
            albedo[pixel * 4 + 1] = 100;
            albedo[pixel * 4 + 2] = 50;
            albedo[pixel * 4 + 3] = if pixel % 2 == 0 { 0 } else { 255 };
        }
        let set = TerrainTextureSet {
            albedo_rgba: albedo,
            normal_rgba: vec![128u8; pixels * 4],
            roughness_r8: vec![200u8; pixels],
        };
        let chain = generate_terrain_mip_chain(&set, edge);
        assert_eq!(set.albedo_rgba[3], 0);
        assert_eq!(set.albedo_rgba[7], 255);
        // Level 1 averages the alternating alpha to ~127/128, proving alpha is
        // filtered rather than dropped or clamped to 255.
        let level1_alpha: Vec<u8> = chain.albedo[1]
            .bytes
            .iter()
            .skip(3)
            .step_by(4)
            .copied()
            .collect();
        assert!(
            level1_alpha.iter().all(|&a| (120..=135).contains(&a)),
            "level 1 alpha must be the filtered mean, got {level1_alpha:?}"
        );
        // Deeper levels converge to the same mean and never snap to opaque.
        let last = chain.albedo.last().expect("last level");
        let last_alpha = last.bytes[3];
        assert!(
            (120..=135).contains(&last_alpha),
            "the 1x1 alpha must stay the mean, got {last_alpha}"
        );
    }

    #[test]
    #[should_panic(expected = "even")]
    fn base_color_reduction_rejects_an_odd_source() {
        let mut source = flat_rgb16(4, [0; 3]);
        source.width = 3;
        source.height = 3;
        let _ = halve_base_color_srgb(&source);
    }

    #[test]
    #[should_panic(expected = "samples")]
    fn roughness_reduction_rejects_a_truncated_source() {
        let mut source = flat_luma16(4, 0);
        source.samples.pop();
        let _ = halve_roughness(&source);
    }
}
