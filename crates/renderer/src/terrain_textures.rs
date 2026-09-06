//! G3A: deterministic, development-time terrain texture generation.
//!
//! The terrain material is textured with three maps generated off-line by a
//! pure, deterministic generator (this module) and versioned in the repository
//! as PNGs (`crates/renderer/assets/terrain_grass_{albedo,normal,roughness}.png`),
//! embedded into the binary via `include_bytes!`. Generation happens exactly
//! once at development time; the renderer only decodes the embedded PNGs once
//! at initialization. There is no per-frame or per-chunk procedural work.
//!
//! # Seamless tiling
//!
//! Every map is periodic in its own texture space with period `SIZE` texels:
//! each noise octave is sampled on a lattice whose corner hash is periodic
//! (`rem_euclid` on the cell period), and texel values are sampled at texel
//! centers, so texel `SIZE` is bitwise identical to texel `0`. Together with
//! `AddressMode::Repeat` no seam can appear at any tile boundary.
//!
//! # Normal convention
//!
//! The normal map follows the OpenGL/Blender convention: RGB ∈ [0, 1] maps
//! tangent-space XYZ ∈ [-1, 1], flat (pointing +Z in tangent space, which is
//! world-up on the terrain) is `(128, 128, 255)`. The map is stored in a
//! linear (non-sRGB) format; the shader decodes it to [-1, 1] and rebuilds Z
//! after applying `normal_strength`.
//!
//! # Roughness convention
//!
//! The roughness map is single-channel (R) linear data in [0, 1]; the shader
//! multiplies it by the material base roughness.
//!
//! # G3A-R deterministic mip chain
//!
//! `generate_terrain_mip_chain` produces the complete mip pyramid (512 -> 1,
//! 10 levels) with a wrap-free 2x2 box filter in the correct color space:
//! albedo is averaged in linear space and re-encoded to sRGB, normal vectors
//! are decoded, averaged, and renormalized (never flattening), and roughness
//! (linear R8) is averaged directly. Generation is pure and deterministic, so
//! the GPU mips can be produced once at initialization from the committed
//! assets without any per-frame work.

/// Texture edge length in texels for all three maps.
pub const TERRAIN_TEXTURE_SIZE: u32 = 512;

/// Fixed, platform-independent generator seeds. Changing any of these changes
/// the committed assets; the regression test that regenerates and compares
/// against the committed PNGs will fail, which is the intended tripwire.
const SEED_ALBEDO_COARSE: u32 = 0x51A3_9C2D;
const SEED_ALBEDO_DRY: u32 = 0x7C4B_E9F5;
const SEED_ALBEDO_MOTTLE: u32 = 0x2A8D_6B34;
const SEED_NORMAL_COARSE: u32 = 0x6F1E_47A8;
const SEED_NORMAL_MID: u32 = 0xB32C_5D91;
const SEED_NORMAL_FINE: u32 = 0x9E4A_18C6;
const SEED_ROUGHNESS_COARSE: u32 = 0x3D7F_A22E;
const SEED_ROUGHNESS_FINE: u32 = 0xE56B_0C91;

/// Albedo base color of a well-kept grass field (linear intent, stored sRGB).
const GRASS_BASE_RGB: [f32; 3] = [0.30, 0.52, 0.23];
/// Dry/dead grass color for sparse warm patches.
const GRASS_DRY_RGB: [f32; 3] = [0.62, 0.55, 0.30];
/// Luminance around which the albedo field oscillates.
const GRASS_BASE_LUMINANCE: f32 = 0.80;
/// Peak luminance swing of the low-frequency patch field.
const GRASS_LUMINANCE_SWING: f32 = 0.22;
/// Dry-patch noise threshold ([0,1] field, above this the patch is dry).
const DRY_PATCH_START: f32 = 0.60;
const DRY_PATCH_PEAK: f32 = 0.86;
/// Peak per-channel micro-mottle (blade-level tint jitter).
const MOTTLE_SWING: f32 = 0.08;

/// Octave cell sizes (in texels) for the albedo/patch field.
const ALBEDO_CELL_COARSE: u32 = 128;
const ALBEDO_CELL_MID: u32 = 32;
const ALBEDO_CELL_FINE: u32 = 8;
/// Octave cell size for the blade-level grain.
const ALBEDO_CELL_MICRO: u32 = 2;

/// Slope applied to the height-field gradients when building the normal map.
/// Tuned so the encoded XY channels stay within roughly ±30/255 of 128:
/// clearly visible micro-relief under the sun, but no cliff-like response.
const NORMAL_SLOPE: f32 = 6.0;

/// Octave cell sizes for the normal height field.
const NORMAL_CELL_COARSE: u32 = 128;
const NORMAL_CELL_MID: u32 = 32;
const NORMAL_CELL_FINE: u32 = 8;

/// Octave cell sizes for the roughness field.
const ROUGHNESS_CELL_COARSE: u32 = 64;
const ROUGHNESS_CELL_FINE: u32 = 16;

/// Mean roughness and swing of the roughness field (linear data).
const ROUGHNESS_MEAN: f32 = 0.80;
/// Peak per-octave roughness swing; the combined field sweeps roughly
/// [0.54, 1.0] so wet and dry patches are clearly separated.
const ROUGHNESS_SWING: f32 = 0.45;

/// One set of generated terrain maps, raw pixels ready for GPU upload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TerrainTextureSet {
    /// RGBA8 albedo, `SIZE * SIZE * 4` bytes, intended to be sampled as sRGB.
    pub albedo_rgba: Vec<u8>,
    /// RGBA8 tangent-space normal, `SIZE * SIZE * 4` bytes, linear data.
    pub normal_rgba: Vec<u8>,
    /// R8 roughness, `SIZE * SIZE` bytes, linear data.
    pub roughness_r8: Vec<u8>,
}

/// Committed, versioned assets embedded at compile time.
///
/// Regenerated deterministically by `generate_terrain_textures` through the
/// `generate_terrain_textures` binary; never edited by hand.
pub mod generated {
    pub const TERRAIN_ALBEDO_PNG: &[u8] = include_bytes!("../assets/terrain_grass_albedo.png");
    pub const TERRAIN_NORMAL_PNG: &[u8] = include_bytes!("../assets/terrain_grass_normal.png");
    pub const TERRAIN_ROUGHNESS_PNG: &[u8] =
        include_bytes!("../assets/terrain_grass_roughness.png");
}

/// Generate the full deterministic terrain texture set.
///
/// Pure function of `size`: same size → same pixels, bitwise, on every build
/// and platform. Allocates exactly the three output buffers.
#[must_use]
pub fn generate_terrain_textures(size: u32) -> TerrainTextureSet {
    let pixel_count = (size as usize) * (size as usize);

    let mut albedo_rgba = vec![0u8; pixel_count * 4];
    let mut normal_rgba = vec![0u8; pixel_count * 4];
    let mut roughness_r8 = vec![0u8; pixel_count];

    let mut height_field = vec![0.0f32; pixel_count];

    for y in 0..size {
        for x in 0..size {
            let index = (y as usize) * (size as usize) + (x as usize);
            let (albedo, height) = grass_pixel(size, x, y);
            albedo_rgba[index * 4..index * 4 + 4].copy_from_slice(&albedo);
            height_field[index] = height;
        }
    }

    for y in 0..size {
        for x in 0..size {
            let index = (y as usize) * (size as usize) + (x as usize);
            let normal = grass_normal_pixel(size, x, y, &height_field);
            normal_rgba[index * 4..index * 4 + 4].copy_from_slice(&normal);
            let roughness = grass_roughness_pixel(size, x, y);
            roughness_r8[index] = (roughness.clamp(0.0, 1.0) * 255.0).round() as u8;
        }
    }

    TerrainTextureSet {
        albedo_rgba,
        normal_rgba,
        roughness_r8,
    }
}

/// One albedo texel + the height sample used by the normal map.
///
/// Returns `([r, g, b, a], height)` where `a` is always 255.
fn grass_pixel(size: u32, x: u32, y: u32) -> ([u8; 4], f32) {
    // Low-frequency patch field (coarse + mid + fine octaves, smoothstepped
    // value noise sampled at texel centers). Periodicity in each octave gives
    // the combined field period `size` texels.
    let patch = 0.50 * octave_noise(size, ALBEDO_CELL_COARSE, x, y, SEED_ALBEDO_COARSE)
        + 0.32 * octave_noise(size, ALBEDO_CELL_MID, x, y, SEED_ALBEDO_COARSE)
        + 0.18 * octave_noise(size, ALBEDO_CELL_FINE, x, y, SEED_ALBEDO_COARSE);

    let luminance = GRASS_BASE_LUMINANCE + GRASS_LUMINANCE_SWING * (patch - 0.5);

    // Sparse dry patches on an independent coarse lattice.
    let dry_field = octave_noise(size, ALBEDO_CELL_COARSE, x, y, SEED_ALBEDO_DRY);
    let dry = smoothstep(DRY_PATCH_START, DRY_PATCH_PEAK, dry_field);

    // Blade-level mottle: tiny per-channel tint jitter.
    let mottle = octave_noise(size, ALBEDO_CELL_MICRO, x, y, SEED_ALBEDO_MOTTLE) - 0.5;
    let mottle_green =
        (octave_noise(size, ALBEDO_CELL_MICRO, x, y, SEED_ALBEDO_MOTTLE ^ 0x0F) - 0.5) * 0.7;

    let mut rgb = [0.0f32; 3];
    for channel in 0..3 {
        let green = mix_channel(GRASS_BASE_RGB[channel], GRASS_DRY_RGB[channel], dry);
        let mottle_swing = if channel == 1 {
            mottle_green
        } else if channel == 0 {
            mottle * 0.8
        } else {
            mottle * 0.6
        };
        rgb[channel] = (green * luminance + MOTTLE_SWING * mottle_swing).clamp(0.0, 1.0);
    }

    let height = 0.50 * octave_noise(size, NORMAL_CELL_COARSE, x, y, SEED_NORMAL_COARSE)
        + 0.32 * octave_noise(size, NORMAL_CELL_MID, x, y, SEED_NORMAL_MID)
        + 0.18 * octave_noise(size, NORMAL_CELL_FINE, x, y, SEED_NORMAL_FINE);

    (encode_u8x4(rgb), height)
}

/// One normal texel from a periodic height field via central differences.
fn grass_normal_pixel(size: u32, x: u32, y: u32, height_field: &[f32]) -> [u8; 4] {
    let index = |xx: u32, yy: u32| (yy as usize) * (size as usize) + (xx as usize);
    let left = height_field[index((x + size - 1) % size, y)];
    let right = height_field[index((x + 1) % size, y)];
    let down = height_field[index(x % size, (y + size - 1) % size)];
    let up = height_field[index(x % size, (y + 1) % size)];

    let gx = (right - left) * 0.5;
    let gy = (up - down) * 0.5;

    let nx = -gx * NORMAL_SLOPE;
    let ny = -gy * NORMAL_SLOPE;
    let nz = 1.0;

    let length_sq = nx * nx + ny * ny + nz * nz;
    // length_sq >= 1.0 always (nz = 1), so this cannot be zero.
    let inv_length = length_sq.sqrt().recip();
    let normal = [nx * inv_length, ny * inv_length, nz * inv_length];

    // Encode [-1, 1] → [0, 1] → u8. Alpha is unused (255).
    [
        ((normal[0] * 0.5 + 0.5) * 255.0).round() as u8,
        ((normal[1] * 0.5 + 0.5) * 255.0).round() as u8,
        ((normal[2] * 0.5 + 0.5) * 255.0).round() as u8,
        255,
    ]
}

/// One roughness texel: coarse wet/dry patches + fine mottling.
fn grass_roughness_pixel(size: u32, x: u32, y: u32) -> f32 {
    let coarse = octave_noise(size, ROUGHNESS_CELL_COARSE, x, y, SEED_ROUGHNESS_COARSE) - 0.5;
    let fine = octave_noise(size, ROUGHNESS_CELL_FINE, x, y, SEED_ROUGHNESS_FINE) - 0.5;
    (ROUGHNESS_MEAN + ROUGHNESS_SWING * coarse * 0.8 + ROUGHNESS_SWING * fine * 0.35)
        .clamp(0.0, 1.0)
}

/// Value noise at texel `(x, y)` for the octave with `cell_px`-wide cells.
///
/// The field is periodic with period `size` texels: the lattice corner hash
/// wraps with `rem_euclid` over the per-axis cell count, and the texel is
/// sampled at its center, so texel `size` ≡ texel `0` bitwise.
fn octave_noise(size: u32, cell_px: u32, x: u32, y: u32, seed: u32) -> f32 {
    let cells = (size / cell_px) as i64;
    debug_assert!(cells >= 1, "cell size must not exceed the texture size");
    // Lattice coordinate of the texel center, in cell units.
    let sx = (x as f32 + 0.5) / cell_px as f32;
    let sy = (y as f32 + 0.5) / cell_px as f32;
    periodic_value_noise(sx, sy, cells, seed)
}

/// Smoothstep-clamped 2D lattice value noise in [0, 1], periodic in both axes.
fn periodic_value_noise(x: f32, y: f32, cells: i64, seed: u32) -> f32 {
    let x0 = x.floor();
    let y0 = y.floor();
    let fx = x - x0;
    let fy = y - y0;

    let ix0 = (x0 as i64).rem_euclid(cells);
    let iy0 = (y0 as i64).rem_euclid(cells);
    // Neighbor cell wraps at the period (cells ≡ 0).
    let ix1 = (ix0 + 1).rem_euclid(cells);
    let iy1 = (iy0 + 1).rem_euclid(cells);

    let v00 = lattice_hash_unit(ix0, iy0, seed);
    let v10 = lattice_hash_unit(ix1, iy0, seed);
    let v01 = lattice_hash_unit(ix0, iy1, seed);
    let v11 = lattice_hash_unit(ix1, iy1, seed);

    let tx = fx * fx * (3.0 - 2.0 * fx);
    let ty = fy * fy * (3.0 - 2.0 * fy);

    let top = v00 + (v10 - v00) * tx;
    let bottom = v01 + (v11 - v01) * tx;
    top + (bottom - top) * ty
}

/// Deterministic lattice corner hash in [0, 1]. Wrapping integer mixing only:
/// no allocation, no platform state, identical on every build and platform.
/// Inputs are expected to be non-negative (the caller wraps them).
fn lattice_hash_unit(ix: i64, iy: i64, seed: u32) -> f32 {
    let mut h = seed ^ (ix as u32).wrapping_mul(0x85EB_CA6B);
    h = h.wrapping_add((iy as u32).wrapping_mul(0xC2B2_AE35));
    h = h.wrapping_mul(0x9E37_79B9);
    h ^= h >> 16;
    h = h.wrapping_mul(0x85EB_CA6B);
    h ^= h >> 13;
    h = h.wrapping_mul(0xC2B2_AE35);
    h ^= h >> 16;
    ((h >> 8) & 0xFFFF) as f32 * (1.0 / 65_535.0)
}

/// Hermite smoothstep between edge0 and edge1.
fn smoothstep(edge0: f32, edge1: f32, value: f32) -> f32 {
    let t = ((value - edge0) / (edge1 - edge0)).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Linear interpolation used for dry-patch mixing.
fn mix_channel(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

fn encode_u8x4(rgb: [f32; 3]) -> [u8; 4] {
    [
        (rgb[0] * 255.0).round() as u8,
        (rgb[1] * 255.0).round() as u8,
        (rgb[2] * 255.0).round() as u8,
        255,
    ]
}

// ---------------------------------------------------------------------------
// G3A-R: deterministic CPU mip chain
// ---------------------------------------------------------------------------

/// One mip level of a terrain map: base-2 power-of-two dimensions and the raw
/// texel bytes in the same layout as the corresponding base map (RGBA8 for
/// albedo/normal, single-channel R8 for roughness).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MipImage {
    /// Level edge length in texels (X).
    pub width: u32,
    /// Level edge length in texels (Y).
    pub height: u32,
    /// `width * height * channels` bytes, ready for GPU upload.
    pub bytes: Vec<u8>,
}

/// Complete deterministic mip chain for the three terrain maps.
///
/// Level 0 is the committed base texture (bitwise identical to the decoded
/// PNG); each following level halves the dimensions with a 2x2 box filter in
/// the correct color space:
///
/// - albedo: decode sRGB -> linear, average, re-encode to sRGB (hardware
///   samples the sRGB texture and expects sRGB-encoded mip data);
/// - normal: decode each vector to [-1, 1], average, renormalize — never a
///   plain flattening average, which would wash out the relief;
/// - roughness: linear R8, direct average.
///
/// All operations are pure and deterministic; no allocation happens at frame
/// time. The renderer builds this chain once at initialization from the
/// committed assets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TerrainMipChain {
    /// Albedo levels, RGBA8 sRGB intent, level 0 = base texture.
    pub albedo: Vec<MipImage>,
    /// Tangent-space normal levels, RGBA8 linear, level 0 = base texture.
    pub normal: Vec<MipImage>,
    /// Roughness levels, R8 linear, level 0 = base texture.
    pub roughness: Vec<MipImage>,
}

/// Number of mip levels for a power-of-two square texture of the given edge:
/// `log2(size) + 1` (512 -> 10 levels down to 1x1).
///
/// # Panics
///
/// Panics if `size` is not a positive power of two.
#[must_use]
pub fn mip_level_count_for_size(size: u32) -> u32 {
    assert!(
        size >= 1 && size.is_power_of_two(),
        "mip level count requires a positive power-of-two size, got {size}"
    );
    size.trailing_zeros() + 1
}

/// Assemble the base texture set from decoded committed assets.
///
/// `albedo_rgba8` and `normal_rgba8` are the direct RGBA8 decodes of the
/// committed PNGs; `roughness_rgba8` is the gray PNG decode (R == G == B ==
/// the linear roughness value). Extracting the R channel here keeps the GPU
/// upload path and the test path byte-identical.
#[must_use]
pub fn terrain_texture_set_from_decoded(
    albedo_rgba8: &[u8],
    normal_rgba8: &[u8],
    roughness_rgba8: &[u8],
) -> TerrainTextureSet {
    let roughness_r8 = roughness_rgba8.iter().step_by(4).copied().collect();
    TerrainTextureSet {
        albedo_rgba: albedo_rgba8.to_vec(),
        normal_rgba: normal_rgba8.to_vec(),
        roughness_r8,
    }
}

/// Build the full mip chain for a terrain texture set.
///
/// A pure function of the input pixels: the same set always yields the same
/// chain, bitwise, on every build and platform. Level 0 is the input itself;
/// each following level halves both dimensions.
#[must_use]
pub fn generate_terrain_mip_chain(set: &TerrainTextureSet, size: u32) -> TerrainMipChain {
    assert_eq!(
        set.albedo_rgba.len(),
        (size * size * 4) as usize,
        "albedo pixel count must match size"
    );
    assert_eq!(
        set.normal_rgba.len(),
        (size * size * 4) as usize,
        "normal pixel count must match size"
    );
    assert_eq!(
        set.roughness_r8.len(),
        (size * size) as usize,
        "roughness pixel count must match size"
    );

    let level_count = mip_level_count_for_size(size) as usize;
    let mut albedo = Vec::with_capacity(level_count);
    let mut normal = Vec::with_capacity(level_count);
    let mut roughness = Vec::with_capacity(level_count);

    let (mut w, mut h, mut albedo_px) = (size, size, set.albedo_rgba.clone());
    let (mut normal_w, mut normal_h, mut normal_px) = (size, size, set.normal_rgba.clone());
    let (mut rough_w, mut rough_h, mut rough_px) = (size, size, set.roughness_r8.clone());

    albedo.push(MipImage {
        width: w,
        height: h,
        bytes: albedo_px.clone(),
    });
    normal.push(MipImage {
        width: normal_w,
        height: normal_h,
        bytes: normal_px.clone(),
    });
    roughness.push(MipImage {
        width: rough_w,
        height: rough_h,
        bytes: rough_px.clone(),
    });

    while w > 1 {
        (w, h, albedo_px) = downsample_rgba8_srgb(w, h, &albedo_px);
        albedo.push(MipImage {
            width: w,
            height: h,
            bytes: albedo_px.clone(),
        });
        (normal_w, normal_h, normal_px) = downsample_rgba8_normal(normal_w, normal_h, &normal_px);
        normal.push(MipImage {
            width: normal_w,
            height: normal_h,
            bytes: normal_px.clone(),
        });
        (rough_w, rough_h, rough_px) = downsample_r8(rough_w, rough_h, &rough_px);
        roughness.push(MipImage {
            width: rough_w,
            height: rough_h,
            bytes: rough_px.clone(),
        });
    }

    TerrainMipChain {
        albedo,
        normal,
        roughness,
    }
}

/// Half-resolution 2x2 box filter of an RGBA8 sRGB image.
///
/// Every intermediate level of a power-of-two chain is even-sized, so the
/// filter never crosses the tile border and each level stays perfectly
/// periodic. Each texel is decoded to linear light, averaged, and re-encoded
/// to sRGB, so the color space is preserved down the chain.
fn downsample_rgba8_srgb(width: u32, height: u32, src: &[u8]) -> (u32, u32, Vec<u8>) {
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h * 4) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut acc = [0.0f64; 4];
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                let i = (((2 * oy + dy) * width + (2 * ox + dx)) * 4) as usize;
                for channel in 0..3 {
                    acc[channel] += srgb_to_linear(src[i + channel]);
                }
                acc[3] += f64::from(src[i + 3]);
            }
            let oi = ((oy * out_w + ox) * 4) as usize;
            for channel in 0..3 {
                out[oi + channel] = linear_to_srgb_u8(acc[channel] * 0.25);
            }
            out[oi + 3] = (acc[3] * 0.25).round() as u8;
        }
    }
    (out_w, out_h, out)
}

/// Half-resolution 2x2 box filter of an RGBA8 tangent-space normal map.
///
/// The four vectors are decoded to [-1, 1], summed, and renormalized; a
/// near-zero sum (only possible on fully opposing corners, which the terrain
/// normals never reach) falls back to flat-up instead of producing a
/// degenerate level.
fn downsample_rgba8_normal(width: u32, height: u32, src: &[u8]) -> (u32, u32, Vec<u8>) {
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h * 4) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut sum = [0.0f64; 3];
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                let i = (((2 * oy + dy) * width + (2 * ox + dx)) * 4) as usize;
                for channel in 0..3 {
                    sum[channel] += (f64::from(src[i + channel]) / 127.5) - 1.0;
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
                out[oi + channel] = ((normal[channel] * 0.5 + 0.5) * 255.0).round() as u8;
            }
            out[oi + 3] = 255;
        }
    }
    (out_w, out_h, out)
}

/// Half-resolution 2x2 box filter of an R8 linear map (roughness).
fn downsample_r8(width: u32, height: u32, src: &[u8]) -> (u32, u32, Vec<u8>) {
    let out_w = width / 2;
    let out_h = height / 2;
    let mut out = vec![0u8; (out_w * out_h) as usize];
    for oy in 0..out_h {
        for ox in 0..out_w {
            let mut acc = 0u32;
            for (dy, dx) in [(0u32, 0u32), (1, 0), (0, 1), (1, 1)] {
                acc += u32::from(src[((2 * oy + dy) * width + (2 * ox + dx)) as usize]);
            }
            // Nearest with half-up rounding, exact integer arithmetic.
            out[(oy * out_w + ox) as usize] = ((acc + 2) / 4) as u8;
        }
    }
    (out_w, out_h, out)
}

/// sRGB (IEC 61966-2-1) decode of one channel: [0, 255] -> linear [0, 1].
fn srgb_to_linear(channel: u8) -> f64 {
    let c = f64::from(channel) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// sRGB (IEC 61966-2-1) encode of one linear channel: [0, 1] -> [0, 255].
fn linear_to_srgb_u8(linear: f64) -> u8 {
    let c = linear.clamp(0.0, 1.0);
    let encoded = if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    };
    (encoded * 255.0).round() as u8
}

#[cfg(test)]
mod tests {
    use super::generated::{TERRAIN_ALBEDO_PNG, TERRAIN_NORMAL_PNG, TERRAIN_ROUGHNESS_PNG};
    use super::*;
    use crate::texture::decode_image;

    #[test]
    fn generated_size_is_reasonable() {
        assert_eq!(TERRAIN_TEXTURE_SIZE, 512);
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        assert_eq!(set.albedo_rgba.len(), 512 * 512 * 4);
        assert_eq!(set.normal_rgba.len(), 512 * 512 * 4);
        assert_eq!(set.roughness_r8.len(), 512 * 512);
    }

    #[test]
    fn generation_is_bitwise_deterministic() {
        let a = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        let b = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        assert_eq!(a, b);
    }

    #[test]
    fn committed_assets_match_the_generator_bitwise() {
        // The versioned PNGs must be exactly reproducible from this module, so
        // the repository assets can never silently drift from the generator.
        let expected = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);

        let albedo = decode_image(TERRAIN_ALBEDO_PNG).expect("albedo asset must decode");
        assert_eq!(albedo.rgba8, expected.albedo_rgba);

        let normal = decode_image(TERRAIN_NORMAL_PNG).expect("normal asset must decode");
        assert_eq!(normal.rgba8, expected.normal_rgba);

        // The roughness asset is stored as a gray PNG; decode expands it to
        // RGBA with R == G == B == L. Compare against the expected R channel.
        let roughness = decode_image(TERRAIN_ROUGHNESS_PNG).expect("roughness asset must decode");
        for (index, &expected_r) in expected.roughness_r8.iter().enumerate() {
            let rgba = &roughness.rgba8[index * 4..index * 4 + 4];
            assert_eq!(rgba, &[expected_r, expected_r, expected_r, 255]);
        }
    }

    #[test]
    fn embedded_assets_decode_to_committed_dimensions() {
        for (bytes, label) in [
            (TERRAIN_ALBEDO_PNG, "albedo"),
            (TERRAIN_NORMAL_PNG, "normal"),
            (TERRAIN_ROUGHNESS_PNG, "roughness"),
        ] {
            let decoded = decode_image(bytes).unwrap_or_else(|e| panic!("{label} asset: {e}"));
            assert_eq!(
                (decoded.width, decoded.height),
                (TERRAIN_TEXTURE_SIZE, TERRAIN_TEXTURE_SIZE),
                "{label} asset dimensions"
            );
        }
    }

    #[test]
    fn embedded_assets_are_png_files() {
        for (bytes, label) in [
            (TERRAIN_ALBEDO_PNG, "albedo"),
            (TERRAIN_NORMAL_PNG, "normal"),
            (TERRAIN_ROUGHNESS_PNG, "roughness"),
        ] {
            assert_eq!(
                &bytes[..8],
                &[0x89, b'P', b'N', b'G', 0x0D, 0x0A, 0x1A, 0x0A],
                "{label} asset must be a PNG"
            );
        }
    }

    #[test]
    fn albedo_is_grass_like_green_with_variation() {
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        let mut mean = [0.0f64; 3];
        let mut min_channel = [255u8; 3];
        let mut max_channel = [0u8; 3];
        for byte in 0..3 {
            for pixel in 0..set.albedo_rgba.len() / 4 {
                let value = set.albedo_rgba[pixel * 4 + byte];
                mean[byte] += value as f64;
                min_channel[byte] = min_channel[byte].min(value);
                max_channel[byte] = max_channel[byte].max(value);
            }
            mean[byte] /= (set.albedo_rgba.len() / 4) as f64;
        }

        // Green-dominant albedo with a sober dynamic range (no black/white).
        // Expected means from the constants: base (0.30, 0.52, 0.23) at the
        // mean luminance 0.80 → roughly (61, 106, 47) in u8.
        assert!(
            (90.0..=120.0).contains(&mean[1]),
            "green mean out of range: {}",
            mean[1]
        );
        assert!(
            (40.0..=80.0).contains(&mean[0]),
            "red mean out of range: {}",
            mean[0]
        );
        assert!(
            (30.0..=70.0).contains(&mean[2]),
            "blue mean out of range: {}",
            mean[2]
        );
        assert!(
            mean[1] > mean[0] && mean[1] > mean[2],
            "grass albedo must be green-dominant"
        );
        for byte in 0..3 {
            let spread = max_channel[byte] - min_channel[byte];
            assert!(
                spread >= 20,
                "albedo must show real variation, channel {byte}: {}..{} (spread {spread})",
                min_channel[byte],
                max_channel[byte]
            );
        }
    }

    #[test]
    fn albedo_alpha_is_always_255() {
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        for pixel in 0..set.albedo_rgba.len() / 4 {
            assert_eq!(set.albedo_rgba[pixel * 4 + 3], 255);
        }
    }

    #[test]
    fn normal_map_convention_is_flat_up_and_bounded() {
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);

        // Decoded channels. Z encodes tangent-space +Z (world-up on terrain).
        let z_min = set
            .normal_rgba
            .iter()
            .skip(2)
            .step_by(4)
            .copied()
            .min()
            .unwrap();
        let z_mean: f64 = set
            .normal_rgba
            .iter()
            .skip(2)
            .step_by(4)
            .map(|&v| v as f64)
            .sum::<f64>()
            / (set.normal_rgba.len() / 4) as f64;
        // The field is slope-dominated, so Z stays comfortably in the upper
        // half-band (normal keeps pointing mostly up).
        assert!(
            z_min >= 125,
            "normal Z must never flip below tangent-planar"
        );
        assert!(
            z_mean > 200.0,
            "normal must stay mostly up, mean Z {z_mean}"
        );

        // XY shows real micro-relief but stays in a credible amplitude band.
        let xy_dev_min: u8 = set
            .normal_rgba
            .iter()
            .enumerate()
            .filter(|&(i, _)| i % 4 == 0 || i % 4 == 1)
            .map(|(_, &v)| v.abs_diff(128))
            .min()
            .unwrap();
        let xy_dev_max: u8 = set
            .normal_rgba
            .iter()
            .enumerate()
            .filter(|&(i, _)| i % 4 == 0 || i % 4 == 1)
            .map(|(_, &v)| v.abs_diff(128))
            .max()
            .unwrap();
        assert!(
            xy_dev_min <= 6,
            "normal must contain near-flat texels, min XY deviation {xy_dev_min}"
        );
        assert!(
            (10..=70).contains(&xy_dev_max),
            "normal XY amplitude out of credible range: {xy_dev_max}"
        );
    }

    #[test]
    fn normal_texels_are_encoded_unit_vectors() {
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        for pixel in set.normal_rgba.as_chunks::<4>().0 {
            let nx = (pixel[0] as f32 / 127.5) - 1.0;
            let ny = (pixel[1] as f32 / 127.5) - 1.0;
            let nz = (pixel[2] as f32 / 127.5) - 1.0;
            let length = (nx * nx + ny * ny + nz * nz).sqrt();
            assert!(
                (length - 1.0).abs() < 0.04,
                "normal texel not unit length: {length} at pixel {pixel:?}"
            );
            assert_eq!(pixel[3], 255);
        }
    }

    #[test]
    fn roughness_is_linear_in_range_with_variation() {
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        let mut min = 255u8;
        let mut max = 0u8;
        let mut sum = 0u64;
        for &value in &set.roughness_r8 {
            min = min.min(value);
            max = max.max(value);
            sum += value as u64;
        }
        let mean = sum as f64 / set.roughness_r8.len() as f64;
        assert!(
            (170.0..=220.0).contains(&mean),
            "roughness mean {mean} out of range"
        );
        assert!(
            min < 150,
            "roughness must show wet/dry variation, min {min}"
        );
        assert!(
            max > 190,
            "roughness must show wet/dry variation, max {max}"
        );
    }

    #[test]
    fn field_wraps_periodically() {
        // Periodicity gives texel `size` ≡ texel `0` bitwise for every map.
        let n = TERRAIN_TEXTURE_SIZE;
        let mut heights = Vec::with_capacity((n as usize) * (n as usize));
        for y in 0..n {
            for x in 0..n {
                let (_, height) = grass_pixel(n, x, y);
                heights.push(height);
            }
        }
        for y in [0, 31, n / 2, n - 1] {
            assert_eq!(
                grass_pixel(n, n, y),
                grass_pixel(n, 0, y),
                "albedo field must repeat with period {n} at y={y}"
            );
            assert_eq!(
                grass_normal_pixel(n, n, y, &heights),
                grass_normal_pixel(n, 0, y, &heights),
                "normal field must repeat with period {n} at y={y}"
            );
            assert_eq!(
                grass_roughness_pixel(n, n, y),
                grass_roughness_pixel(n, 0, y),
                "roughness field must repeat with period {n} at y={y}"
            );
        }
    }

    #[test]
    fn seam_is_indistinguishable_from_interior() {
        // The wrap seam (texel `n - 1` vs texel `0`) must not jump more than
        // the neighbouring texels anywhere else in the map; anything larger
        // would read as a visible tile edge under Repeat addressing.
        let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        let n = TERRAIN_TEXTURE_SIZE as usize;

        for (data, stride, label) in [
            (&set.albedo_rgba, 4usize, "albedo"),
            (&set.normal_rgba, 4usize, "normal"),
            (&set.roughness_r8, 1usize, "roughness"),
        ] {
            let channels = if stride == 1 { 1 } else { 3 };
            for channel in 0..channels {
                let mut max_interior = 0u8;
                for row in 0..n {
                    for col in 0..n - 1 {
                        let a = data[row * n * stride + col * stride + channel];
                        let b = data[row * n * stride + (col + 1) * stride + channel];
                        max_interior = max_interior.max(a.abs_diff(b));
                    }
                }
                let mut seam = 0u8;
                for row in 0..n {
                    let a = data[row * n * stride + channel];
                    let b = data[row * n * stride + (n - 1) * stride + channel];
                    seam = seam.max(a.abs_diff(b));
                }
                assert!(
                    seam as u16 <= max_interior as u16 + 2,
                    "{label} channel {channel}: seam delta {seam} exceeds interior {max_interior}"
                );
            }
        }
    }

    #[test]
    fn noise_is_bounded_and_periodic() {
        let cells = (TERRAIN_TEXTURE_SIZE / ALBEDO_CELL_MID) as i64;
        for i in 0..=cells {
            // Field at cell coordinate `cells` must match the field at 0.
            let a = periodic_value_noise(i as f32 + 0.25, 0.5, cells, SEED_ALBEDO_COARSE);
            let b = periodic_value_noise((i + cells) as f32 + 0.25, 0.5, cells, SEED_ALBEDO_COARSE);
            assert_eq!(a.to_bits(), b.to_bits(), "period violation at {i}");
            assert!((0.0..=1.0).contains(&a));
        }
    }

    // -----------------------------------------------------------------------
    // G3A-R: deterministic mip chain regression tests
    // -----------------------------------------------------------------------

    fn chain_from_committed_assets() -> TerrainMipChain {
        let albedo = decode_image(TERRAIN_ALBEDO_PNG).expect("albedo asset must decode");
        let normal = decode_image(TERRAIN_NORMAL_PNG).expect("normal asset must decode");
        let roughness = decode_image(TERRAIN_ROUGHNESS_PNG).expect("roughness asset must decode");
        let set = terrain_texture_set_from_decoded(&albedo.rgba8, &normal.rgba8, &roughness.rgba8);
        generate_terrain_mip_chain(&set, TERRAIN_TEXTURE_SIZE)
    }

    #[test]
    fn mip_level_count_is_log2_plus_one() {
        assert_eq!(mip_level_count_for_size(1), 1);
        assert_eq!(mip_level_count_for_size(2), 2);
        assert_eq!(mip_level_count_for_size(64), 7);
        assert_eq!(mip_level_count_for_size(512), 10);
        assert_eq!(mip_level_count_for_size(1024), 11);
    }

    #[test]
    fn mip_chain_has_full_level_count_for_all_maps() {
        let chain = chain_from_committed_assets();
        let levels = mip_level_count_for_size(TERRAIN_TEXTURE_SIZE) as usize;
        assert_eq!(chain.albedo.len(), levels, "albedo level count");
        assert_eq!(chain.normal.len(), levels, "normal level count");
        assert_eq!(chain.roughness.len(), levels, "roughness level count");
        assert!(
            levels > 1,
            "the mip chain must never collapse to mip-0-only"
        );
    }

    #[test]
    fn mip_chain_levels_have_halved_dimensions_and_pixel_counts() {
        let chain = chain_from_committed_assets();
        let mut expected_w = TERRAIN_TEXTURE_SIZE;
        for (level, mip) in chain.albedo.iter().enumerate() {
            assert_eq!(
                (mip.width, mip.height),
                (expected_w, expected_w),
                "albedo level {level} dimensions"
            );
            assert_eq!(
                mip.bytes.len(),
                (mip.width * mip.height * 4) as usize,
                "albedo level {level} byte count"
            );
            expected_w /= 2;
        }
        assert_eq!(expected_w, 0, "chain must terminate at the 1x1 level");

        for (level, mip) in chain.normal.iter().enumerate() {
            assert_eq!(
                (mip.width, mip.height),
                (chain.albedo[level].width, chain.albedo[level].height),
                "normal level {level} must match the albedo dimensions"
            );
            assert_eq!(
                mip.bytes.len(),
                (mip.width * mip.height * 4) as usize,
                "normal level {level} byte count"
            );
        }
        for (level, mip) in chain.roughness.iter().enumerate() {
            assert_eq!(
                (mip.width, mip.height),
                (chain.albedo[level].width, chain.albedo[level].height),
                "roughness level {level} must match the albedo dimensions"
            );
            assert_eq!(
                mip.bytes.len(),
                (mip.width * mip.height) as usize,
                "roughness level {level} must be single-channel"
            );
        }
        let last = chain.albedo.last().expect("chain must be non-empty");
        assert_eq!((last.width, last.height), (1, 1), "last level must be 1x1");
    }

    #[test]
    fn mip_chain_mip_zero_is_the_committed_base() {
        let albedo = decode_image(TERRAIN_ALBEDO_PNG).expect("albedo asset must decode");
        let normal = decode_image(TERRAIN_NORMAL_PNG).expect("normal asset must decode");
        let chain = chain_from_committed_assets();
        assert_eq!(chain.albedo[0].bytes, albedo.rgba8, "albedo base level");
        assert_eq!(chain.normal[0].bytes, normal.rgba8, "normal base level");
    }

    #[test]
    fn mip_chain_generation_is_bitwise_deterministic() {
        let a = chain_from_committed_assets();
        let b = chain_from_committed_assets();
        assert_eq!(a, b, "mip chain generation must be bitwise deterministic");
    }

    #[test]
    fn mip_chain_preserves_periodicity_of_the_base_texture() {
        // The base maps are periodic in their own texture space, and the mip
        // filter never crosses the tile border (every 2x2 quad is interior),
        // so each level inherits the wrap smoothness: the seam delta (edge
        // texel vs the far edge texel that becomes its tiled neighbour) must
        // stay within the interior delta budget, exactly like the base maps.
        let chain = chain_from_committed_assets();
        for (level, mip) in chain.albedo.iter().enumerate().skip(2) {
            let n = mip.width as usize;
            let mut max_interior = 0u8;
            for row in 0..n {
                for col in 0..n - 1 {
                    for channel in 0..3 {
                        let a = mip.bytes[row * n * 4 + col * 4 + channel];
                        let b = mip.bytes[row * n * 4 + (col + 1) * 4 + channel];
                        max_interior = max_interior.max(a.abs_diff(b));
                    }
                }
            }
            let mut max_seam = 0u8;
            for row in 0..n {
                for channel in 0..3 {
                    let a = mip.bytes[row * n * 4 + channel];
                    let b = mip.bytes[row * n * 4 + (n - 1) * 4 + channel];
                    max_seam = max_seam.max(a.abs_diff(b));
                }
            }
            assert!(
                max_seam as u16 <= max_interior as u16 + 2,
                "albedo level {level}: seam delta {max_seam} exceeds interior {max_interior}"
            );
        }
        for (level, mip) in chain.roughness.iter().enumerate().skip(2) {
            let n = mip.width as usize;
            let mut max_interior = 0u8;
            for row in 0..n {
                for col in 0..n - 1 {
                    max_interior = max_interior
                        .max(mip.bytes[row * n + col].abs_diff(mip.bytes[row * n + col + 1]));
                }
            }
            let mut max_seam = 0u8;
            for row in 0..n {
                max_seam = max_seam.max(mip.bytes[row * n].abs_diff(mip.bytes[row * n + n - 1]));
            }
            assert!(
                max_seam as u16 <= max_interior as u16 + 2,
                "roughness level {level}: seam delta {max_seam} exceeds interior {max_interior}"
            );
        }
    }

    #[test]
    fn mip_normal_levels_are_finite_and_normalized() {
        let chain = chain_from_committed_assets();
        for (level, mip) in chain.normal.iter().enumerate() {
            for pixel in mip.bytes.as_chunks::<4>().0 {
                let nx = (f32::from(pixel[0]) / 127.5) - 1.0;
                let ny = (f32::from(pixel[1]) / 127.5) - 1.0;
                let nz = (f32::from(pixel[2]) / 127.5) - 1.0;
                assert!(nx.is_finite() && ny.is_finite() && nz.is_finite());
                let length = (nx * nx + ny * ny + nz * nz).sqrt();
                assert!(
                    (length - 1.0).abs() < 0.04,
                    "normal level {level} texel not unit length: {length} at {pixel:?}"
                );
                assert!(nz > 0.9, "downsampled normals must keep pointing up");
                assert_eq!(pixel[3], 255);
            }
        }
    }

    #[test]
    fn mip_albedo_mean_is_stable_down_the_chain() {
        // The box filter averages in linear light, so the sRGB-space mean can
        // drift only slightly from level to level; a large jump would mean the
        // color space is being ignored.
        let chain = chain_from_committed_assets();
        let base_mean: Vec<f64> = {
            let mip = &chain.albedo[0];
            let mut mean = [0.0f64; 3];
            for pixel in mip.bytes.as_chunks::<4>().0 {
                for (channel, slot) in mean.iter_mut().enumerate() {
                    *slot += f64::from(pixel[channel]);
                }
            }
            for value in &mut mean {
                *value /= (mip.width * mip.height) as f64;
            }
            mean.to_vec()
        };
        for (level, mip) in chain.albedo.iter().enumerate().skip(1) {
            let mut mean = [0.0f64; 3];
            for pixel in mip.bytes.as_chunks::<4>().0 {
                for (channel, slot) in mean.iter_mut().enumerate() {
                    *slot += f64::from(pixel[channel]);
                }
            }
            for (channel, value) in mean.iter_mut().enumerate() {
                *value /= (mip.width * mip.height) as f64;
                assert!(
                    (*value - base_mean[channel]).abs() < 14.0,
                    "albedo level {level} channel {channel} mean drifted: {value:.1} vs {:.1}",
                    base_mean[channel]
                );
            }
        }
    }

    #[test]
    fn mip_roughness_mean_is_stable_down_the_chain() {
        // Integer half-up averaging preserves the mean of linear data.
        let chain = chain_from_committed_assets();
        let base_mean = chain.roughness[0]
            .bytes
            .iter()
            .map(|&v| f64::from(v))
            .sum::<f64>()
            / chain.roughness[0].bytes.len() as f64;
        for (level, mip) in chain.roughness.iter().enumerate().skip(1) {
            let mean =
                mip.bytes.iter().map(|&v| f64::from(v)).sum::<f64>() / mip.bytes.len() as f64;
            assert!(
                (mean - base_mean).abs() < 3.0,
                "roughness level {level} mean drifted: {mean:.1} vs {base_mean:.1}"
            );
        }
    }

    #[test]
    fn mip_chain_from_generated_set_matches_committed_chain() {
        // The chain built from `generate_terrain_textures` must be bitwise
        // identical to the chain built from the committed PNGs, proving the
        // GPU path (decoded assets) and the development path (generator) can
        // never drift apart.
        let generated = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
        let chain_a = generate_terrain_mip_chain(&generated, TERRAIN_TEXTURE_SIZE);

        let albedo = decode_image(TERRAIN_ALBEDO_PNG).expect("albedo asset must decode");
        let normal = decode_image(TERRAIN_NORMAL_PNG).expect("normal asset must decode");
        let roughness = decode_image(TERRAIN_ROUGHNESS_PNG).expect("roughness asset must decode");
        let set = terrain_texture_set_from_decoded(&albedo.rgba8, &normal.rgba8, &roughness.rgba8);
        let chain_b = generate_terrain_mip_chain(&set, TERRAIN_TEXTURE_SIZE);
        assert_eq!(chain_a, chain_b);
    }
}
