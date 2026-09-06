//! G1D: Terrain subsystem for the RC flying field environment.
//!
//! # Architecture
//!
//! The terrain is presentation-only. It does not participate in physics collision.
//! The terrain renderer must NOT become physics authority.
//!
//! # Coordinate Convention
//!
//! Terrain uses render-space coordinates:
//! - +Y = up (elevation)
//! - XZ = horizontal plane
//!
//! The default terrain is centered around the render origin so the aircraft
//! starts above the terrain interior, not at a corner.
//!
//! # Render Origin Offset
//!
//! The height field stores raw elevation data in its own grid coordinate system
//! (indices 0..=width_cells, 0..=depth_cells). When generating chunks, a
//! `render_origin_offset` shifts grid coordinates into render-space so the
//! terrain is centered around the render origin.
//!
//! Future render-origin shifts (floating origin) will require coordinated
//! terrain rebasing. For this milestone, terrain is static relative to the
//! initial render origin.
//!
//! # Chunking
//!
//! Terrain is broken into deterministic chunks for:
//! - Future frustum culling
//! - Future LOD
//! - Manageable GPU buffer sizes
//!
//! Adjacent chunks share exact boundary coordinates (no cracks).
//!
//! # UV Tiling
//!
//! UVs are based on world-space metres, not mesh tessellation. This ensures
//! texture scale is independent of chunk resolution.
//!
//! # Material Strategy
//!
//! G3A: the terrain is a textured PBR surface. `TerrainMaterial` carries the
//! metallic/roughness/normal-strength factors and per-map UV offsets that are
//! uploaded once into the dedicated terrain material bind group; the shader
//! entry `fs_terrain` reuses the G1D PBR response with a sampled albedo,
//! tangent-space normal map, and roughness map, all tiled in world-space
//! metres.
//!
//! The `base_color_factor` is baked into vertex colors at chunk generation
//! time. In the textured configuration the default base is white: the shader
//! multiplies the (G2D-modulated) vertex color by the albedo texture sample,
//! so the texture is the chromatic authority while G2D stays the macro
//! brightness/tint variation on top of it.

use crate::mesh::{SAFE_NORMAL, Vertex};
use std::f32::consts::PI;

/// Default terrain texture scale in metres.
///
/// At 4.0m per tile, a 512x512 texture covers a 2km x 2km field with
/// reasonable ground detail without appearing stretched.
pub const DEFAULT_TERRAIN_TEXTURE_SCALE_M: f32 = 4.0;

// ---------------------------------------------------------------------------
// G3A-R: three-frequency stack tuning (central, documented constants)
// ---------------------------------------------------------------------------

/// Macro layer tile scale in metres: breaks the large uniform field patches
/// (order 30-80 m). The same albedo texture is sampled at this much larger
/// tile scale and blended softly underneath the base layer.
pub const DEFAULT_TERRAIN_MACRO_SCALE_M: f32 = 48.0;

/// Detail layer tile scale in metres: perceptible grass texture close to the
/// camera/runway (order 0.25-0.5 m), faded out with distance.
pub const DEFAULT_TERRAIN_DETAIL_SCALE_M: f32 = 0.40;

/// Macro layer UV anchor (tile units) so its borders never align with the
/// base tile grid.
pub const DEFAULT_TERRAIN_MACRO_UV_OFFSET: [f32; 2] = [0.170, 0.390];

/// Detail layer UV anchor (tile units) so its borders never align with the
/// base tile grid.
pub const DEFAULT_TERRAIN_DETAIL_UV_OFFSET: [f32; 2] = [0.163, 0.037];

/// Anti-repetition second-sample scale factor: the rotated sample tiles at
/// `base_scale * ar_scale` metres, so its tile borders run at a different
/// frequency and angle from the base grid.
pub const DEFAULT_TERRAIN_AR_SCALE: f32 = 1.370;

/// Anti-repetition second-sample rotation (degrees). Non-axis-aligned so the
/// rotated tile borders never line up with either world axis.
pub const DEFAULT_TERRAIN_AR_ANGLE_DEGREES: f32 = 27.0;

/// Anti-repetition second-sample UV offset (tile units).
pub const DEFAULT_TERRAIN_AR_OFFSET: [f32; 2] = [0.315, 0.571];

/// Distance (metres) at which the detail normal layer starts fading.
pub const DEFAULT_TERRAIN_DETAIL_NORMAL_FADE_NEAR_M: f32 = 20.0;

/// Distance (metres) at which the detail normal layer is fully faded out;
/// beyond this only base/macro structure remains.
pub const DEFAULT_TERRAIN_DETAIL_NORMAL_FADE_FAR_M: f32 = 80.0;

/// CPU mirror of the WGSL detail fade: 1.0 at/near `near_m`, 0.0 at/beyond
/// `far_m`, with a smoothstep (zero-derivative) falloff so no popping can
/// occur. Shared documentation authority for the shader's smoothstep.
#[must_use]
pub fn detail_normal_fade_weight(distance_m: f32, near_m: f32, far_m: f32) -> f32 {
    debug_assert!(
        near_m >= 0.0 && far_m > near_m,
        "fade range must be ascending"
    );
    let t = ((distance_m - near_m) / (far_m - near_m).max(1e-6)).clamp(0.0, 1.0);
    1.0 - t * t * (3.0 - 2.0 * t)
}

/// CPU mirror of the WGSL anti-repetition UV transform: rotate the base
/// world-space UV by `angle_degrees`, scale it by `ar_scale`, and shift it
/// by `ar_offset` (tile units). Pure and deterministic — the same world
/// position always resolves to the same second-sample UV, across chunks.
#[must_use]
pub fn rotated_secondary_uv(
    uv: [f32; 2],
    ar_scale: f32,
    angle_degrees: f32,
    ar_offset: [f32; 2],
) -> [f32; 2] {
    let radians = angle_degrees * PI / 180.0;
    let (sin, cos) = radians.sin_cos();
    [
        (cos * uv[0] - sin * uv[1]) * ar_scale + ar_offset[0],
        (sin * uv[0] + cos * uv[1]) * ar_scale + ar_offset[1],
    ]
}

/// Default chunk size in cells.
///
/// 32x32 cells per chunk balances:
/// - GPU buffer size (32*32*4 = 4096 vertices per chunk)
/// - Future culling granularity
/// - Memory overhead
pub const DEFAULT_CHUNK_CELLS: u32 = 32;

/// CPU-side terrain height field representation.
///
/// A regular grid of elevation samples at uniform spacing.
/// The height field uses its own grid coordinate system (0..=width_cells).
/// Render-space placement is applied via the offset parameter during chunk generation.
#[derive(Debug, Clone)]
pub struct TerrainHeightField {
    /// Width (X-axis) in cells.
    pub width_cells: u32,
    /// Depth (Z-axis) in cells.
    pub depth_cells: u32,
    /// Spacing between samples in metres.
    pub sample_spacing_m: f32,
    /// Elevation samples in row-major order (Z-major, then X).
    /// Index = z * (width_cells + 1) + x.
    pub elevations: Vec<f32>,
}

impl TerrainHeightField {
    /// Create a new height field with the given dimensions.
    ///
    /// # Panics
    ///
    /// Panics if dimensions are zero or elevations length doesn't match.
    #[must_use]
    pub fn new(
        width_cells: u32,
        depth_cells: u32,
        sample_spacing_m: f32,
        elevations: Vec<f32>,
    ) -> Self {
        assert!(
            width_cells > 0 && depth_cells > 0,
            "dimensions must be non-zero"
        );
        assert!(
            sample_spacing_m > 0.0 && sample_spacing_m.is_finite(),
            "spacing must be positive and finite"
        );
        let expected_len = (width_cells as usize + 1) * (depth_cells as usize + 1);
        assert_eq!(elevations.len(), expected_len, "elevations length mismatch");
        assert!(
            elevations.iter().all(|e| e.is_finite()),
            "elevations must be finite"
        );

        Self {
            width_cells,
            depth_cells,
            sample_spacing_m,
            elevations,
        }
    }

    /// Get elevation at grid coordinates (x, z) where x in [0, width_cells], z in [0, depth_cells].
    #[must_use]
    pub fn elevation_at(&self, x: u32, z: u32) -> f32 {
        debug_assert!(x <= self.width_cells && z <= self.depth_cells);
        let index = (z as usize) * (self.width_cells as usize + 1) + (x as usize);
        self.elevations[index]
    }

    /// Total width in metres.
    #[must_use]
    pub fn width_m(&self) -> f32 {
        self.width_cells as f32 * self.sample_spacing_m
    }

    /// Total depth in metres.
    #[must_use]
    pub fn depth_m(&self) -> f32 {
        self.depth_cells as f32 * self.sample_spacing_m
    }

    /// Sample elevation at arbitrary world coordinates using bilinear interpolation.
    ///
    /// Coordinates are in the height field's local grid space (0..width_m, 0..depth_m).
    /// Coordinates outside the height field are clamped to the boundary.
    #[must_use]
    pub fn sample_bilinear(&self, local_x: f32, local_z: f32) -> f32 {
        let grid_x = local_x / self.sample_spacing_m;
        let grid_z = local_z / self.sample_spacing_m;

        let x0 = grid_x.floor().clamp(0.0, self.width_cells as f32) as u32;
        let z0 = grid_z.floor().clamp(0.0, self.depth_cells as f32) as u32;
        let x1 = (x0 + 1).min(self.width_cells);
        let z1 = (z0 + 1).min(self.depth_cells);

        let fx = (grid_x - x0 as f32).clamp(0.0, 1.0);
        let fz = (grid_z - z0 as f32).clamp(0.0, 1.0);

        let e00 = self.elevation_at(x0, z0);
        let e10 = self.elevation_at(x1, z0);
        let e01 = self.elevation_at(x0, z1);
        let e11 = self.elevation_at(x1, z1);

        let e0 = e00 * (1.0 - fx) + e10 * fx;
        let e1 = e01 * (1.0 - fx) + e11 * fx;

        e0 * (1.0 - fz) + e1 * fz
    }

    /// Sample elevation at render-space coordinates, accounting for the terrain offset.
    ///
    /// Converts render-space (x, z) to height-field-local coordinates before sampling.
    #[must_use]
    pub fn sample_bilinear_render_space(
        &self,
        render_x: f32,
        render_z: f32,
        render_origin_offset: [f32; 2],
    ) -> f32 {
        let local_x = render_x - render_origin_offset[0];
        let local_z = render_z - render_origin_offset[1];
        self.sample_bilinear(local_x, local_z)
    }
}

/// Terrain material configuration.
///
/// G3A: the default configuration is the textured grass surface. The
/// `base_color_factor` is baked into terrain vertex colors at chunk generation
/// time; the `fs_terrain` shader multiplies the vertex color by the sampled
/// albedo texture, so the texture is the chromatic authority while the G2D
/// macro variation (baked into the vertex color) modulates it.
///
/// G3A-R: the uniform additionally carries the three-frequency stack
/// (macro/base/detail scales and per-layer UV anchors), the anti-repetition
/// second-sample transform (rotation/scale/offset applied in world-space UV),
/// and the detail normal distance fade range. All values are world-anchored
/// metres/tile-units, so they are invariant to camera and chunking.
#[derive(Debug, Clone)]
pub struct TerrainMaterial {
    /// Base color factor (RGBA). Baked into vertex colors during chunk generation.
    pub base_color_factor: [f32; 4],
    /// Texture scale in metres for UV tiling.
    pub texture_scale_m: f32,
    /// G3A: PBR metallic factor (glTF metallic workflow). Terrain is a
    /// dielectric; the default is 0.0.
    pub metallic: f32,
    /// G3A: base perceptual roughness. The sampled roughness map is multiplied
    /// by this factor before the shader's MIN_ROUGHNESS floor.
    pub roughness: f32,
    /// G3A: tangent-space normal strength in [0, 1] applied to the sampled
    /// normal map (1.0 = the committed asset amplitude).
    pub normal_strength: f32,
    /// Presentation-only debug channel selector (FINAL by default).
    pub debug_mode: crate::TerrainDebugMode,
    /// G3A: albedo map UV anchor (in tile units) added to the world-space UV.
    pub albedo_uv_offset: [f32; 2],
    /// G3A: normal map UV anchor (in tile units) added to the world-space UV.
    pub normal_uv_offset: [f32; 2],
    /// G3A: roughness map UV anchor (in tile units) added to the world-space UV.
    pub roughness_uv_offset: [f32; 2],
    /// G3A-R: macro layer tile scale in metres (order 30-80 m).
    pub macro_scale_m: f32,
    /// G3A-R: detail layer tile scale in metres (order 0.25-0.5 m).
    pub detail_scale_m: f32,
    /// G3A-R: macro layer UV anchor (tile units).
    pub macro_uv_offset: [f32; 2],
    /// G3A-R: detail layer UV anchor (tile units).
    pub detail_uv_offset: [f32; 2],
    /// G3A-R: anti-repetition rotated second-sample scale factor.
    pub ar_scale: f32,
    /// G3A-R: anti-repetition rotated second-sample angle (degrees).
    pub ar_angle_degrees: f32,
    /// G3A-R: anti-repetition rotated second-sample UV offset (tile units).
    pub ar_offset: [f32; 2],
    /// G3A-R: distance in metres at which the detail normal layer starts
    /// fading (full detail closer than this).
    pub detail_normal_fade_near_m: f32,
    /// G3A-R: distance in metres at which the detail normal layer is fully
    /// faded out (only base/macro structure remains beyond this).
    pub detail_normal_fade_far_m: f32,
}

impl Default for TerrainMaterial {
    fn default() -> Self {
        Self {
            // G3A: the white base turns the baked vertex color into the G2D
            // macro-variation carrier; the albedo texture is the chromatic
            // authority in the textured pipeline.
            base_color_factor: [1.0, 1.0, 1.0, 1.0],
            texture_scale_m: DEFAULT_TERRAIN_TEXTURE_SCALE_M,
            metallic: 0.0,
            roughness: 0.9,
            normal_strength: 1.0,
            debug_mode: crate::TerrainDebugMode::Final,
            // Deliberately non-integer anchors (in tile units) so the three
            // maps' tile borders never align, breaking perceived repetition.
            albedo_uv_offset: [0.0, 0.0],
            normal_uv_offset: [0.271, 0.137],
            roughness_uv_offset: [0.413, 0.303],
            // G3A-R: three-frequency stack defaults (see the constants above).
            macro_scale_m: DEFAULT_TERRAIN_MACRO_SCALE_M,
            detail_scale_m: DEFAULT_TERRAIN_DETAIL_SCALE_M,
            macro_uv_offset: DEFAULT_TERRAIN_MACRO_UV_OFFSET,
            detail_uv_offset: DEFAULT_TERRAIN_DETAIL_UV_OFFSET,
            ar_scale: DEFAULT_TERRAIN_AR_SCALE,
            ar_angle_degrees: DEFAULT_TERRAIN_AR_ANGLE_DEGREES,
            ar_offset: DEFAULT_TERRAIN_AR_OFFSET,
            detail_normal_fade_near_m: DEFAULT_TERRAIN_DETAIL_NORMAL_FADE_NEAR_M,
            detail_normal_fade_far_m: DEFAULT_TERRAIN_DETAIL_NORMAL_FADE_FAR_M,
        }
    }
}

/// A single terrain chunk ready for GPU upload.
#[derive(Debug, Clone)]
pub struct TerrainChunk {
    /// Chunk grid coordinates (chunk_x, chunk_z).
    pub chunk_coords: (u32, u32),
    /// Render-space origin of this chunk (min X, min Z).
    pub world_origin: [f32; 2],
    /// Chunk dimensions in metres.
    pub size_m: [f32; 2],
    /// Vertices for this chunk (in render-space coordinates).
    pub vertices: Vec<Vertex>,
    /// Indices for this chunk (triangle list).
    pub indices: Vec<u32>,
    /// Axis-aligned bounding box (min, max) in render-space.
    pub bounds: ([f32; 3], [f32; 3]),
}

impl TerrainChunk {
    /// Generate a terrain chunk from a height field.
    ///
    /// # Arguments
    ///
    /// * `height_field` - Source height data (grid coordinates)
    /// * `chunk_x` - Chunk X coordinate (in chunk units)
    /// * `chunk_z` - Chunk Z coordinate (in chunk units)
    /// * `cells_per_chunk` - Cells per chunk dimension
    /// * `material` - Terrain material (base color baked into vertices)
    /// * `render_origin_offset` - Offset applied to grid coordinates to produce
    ///   render-space coordinates. Use `[0.0, 0.0]` for un-offset chunks or
    ///   `[-extent/2, -extent/2]` for centered terrain.
    #[must_use]
    pub fn generate(
        height_field: &TerrainHeightField,
        chunk_x: u32,
        chunk_z: u32,
        cells_per_chunk: u32,
        material: &TerrainMaterial,
        render_origin_offset: [f32; 2],
    ) -> Self {
        let cells_x = cells_per_chunk.min(
            height_field
                .width_cells
                .saturating_sub(chunk_x * cells_per_chunk),
        );
        let cells_z = cells_per_chunk.min(
            height_field
                .depth_cells
                .saturating_sub(chunk_z * cells_per_chunk),
        );

        if cells_x == 0 || cells_z == 0 {
            return Self {
                chunk_coords: (chunk_x, chunk_z),
                world_origin: [0.0; 2],
                size_m: [0.0; 2],
                vertices: Vec::new(),
                indices: Vec::new(),
                bounds: ([0.0; 3], [0.0; 3]),
            };
        }

        let start_x = chunk_x * cells_per_chunk;
        let start_z = chunk_z * cells_per_chunk;
        let spacing = height_field.sample_spacing_m;

        let world_origin_x = start_x as f32 * spacing + render_origin_offset[0];
        let world_origin_z = start_z as f32 * spacing + render_origin_offset[1];
        let size_x = cells_x as f32 * spacing;
        let size_z = cells_z as f32 * spacing;

        let vertex_count = (cells_x + 1) * (cells_z + 1);
        let mut vertices = Vec::with_capacity(vertex_count as usize);
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        // Bake material base_color_factor into vertex color.
        // The shader multiplies vertex_color * texture_sample.
        // With the white fallback texture (1,1,1,1), the result is the baked color.
        let baked_color = material.base_color_factor;

        for local_z in 0..=cells_z {
            let global_z = start_z + local_z;
            let render_z = global_z as f32 * spacing + render_origin_offset[1];

            for local_x in 0..=cells_x {
                let global_x = start_x + local_x;
                let render_x = global_x as f32 * spacing + render_origin_offset[0];

                let elevation = height_field.elevation_at(global_x, global_z);
                min_y = min_y.min(elevation);
                max_y = max_y.max(elevation);

                let normal = compute_terrain_normal(height_field, global_x, global_z);

                // UV based on render-space position for consistent tiling.
                let uv = [
                    render_x / material.texture_scale_m,
                    render_z / material.texture_scale_m,
                ];

                // G2D: deterministic surface variation anchored to render-space
                // coordinates; the geometry fields above are untouched.
                let color =
                    terrain_surface_color(baked_color, render_x, render_z, elevation, normal[1]);

                vertices.push(Vertex {
                    position: [render_x, elevation, render_z],
                    normal,
                    color,
                    uv,
                });
            }
        }

        let index_count = (cells_x * cells_z * 6) as usize;
        let mut indices = Vec::with_capacity(index_count);

        for local_z in 0..cells_z {
            for local_x in 0..cells_x {
                let v0 = local_z * (cells_x + 1) + local_x;
                let v1 = v0 + 1;
                let v2 = v0 + (cells_x + 1);
                let v3 = v2 + 1;

                // G3A-R fix: the quad is emitted with the front face pointing
                // UP (+Y, away from the earth). With `front_face = Ccw` and
                // back-face culling the triangles must wind CCW when viewed
                // from above (+X right, +Z down on screen); the previous
                // (v0, v1, v2) order wound CW from above, so the culled
                // terrain never rasterized despite being in view.
                indices.push(v0);
                indices.push(v2);
                indices.push(v1);

                indices.push(v2);
                indices.push(v3);
                indices.push(v1);
            }
        }

        let bounds = (
            [world_origin_x, min_y, world_origin_z],
            [world_origin_x + size_x, max_y, world_origin_z + size_z],
        );

        Self {
            chunk_coords: (chunk_x, chunk_z),
            world_origin: [world_origin_x, world_origin_z],
            size_m: [size_x, size_z],
            vertices,
            indices,
            bounds,
        }
    }
}

/// Compute terrain normal at grid coordinates using finite differences.
fn compute_terrain_normal(height_field: &TerrainHeightField, x: u32, z: u32) -> [f32; 3] {
    let spacing = height_field.sample_spacing_m;
    let two_spacing = 2.0 * spacing;

    let h_left = if x > 0 {
        height_field.elevation_at(x - 1, z)
    } else {
        height_field.elevation_at(x, z)
    };
    let h_right = if x < height_field.width_cells {
        height_field.elevation_at(x + 1, z)
    } else {
        height_field.elevation_at(x, z)
    };
    let h_down = if z > 0 {
        height_field.elevation_at(x, z - 1)
    } else {
        height_field.elevation_at(x, z)
    };
    let h_up = if z < height_field.depth_cells {
        height_field.elevation_at(x, z + 1)
    } else {
        height_field.elevation_at(x, z)
    };

    let dx = if x > 0 && x < height_field.width_cells {
        (h_right - h_left) / two_spacing
    } else {
        (h_right - h_left) / spacing
    };

    let dz = if z > 0 && z < height_field.depth_cells {
        (h_up - h_down) / two_spacing
    } else {
        (h_up - h_down) / spacing
    };

    let normal = [-dx, 1.0, -dz];
    let length_sq = normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2];

    if length_sq > f32::EPSILON * f32::EPSILON {
        let inv_length = length_sq.sqrt().recip();
        [
            normal[0] * inv_length,
            normal[1] * inv_length,
            normal[2] * inv_length,
        ]
    } else {
        SAFE_NORMAL
    }
}

// ---------------------------------------------------------------------------
// G2D: deterministic world-space terrain surface variation
// ---------------------------------------------------------------------------
//
// All tuning below is deliberately conservative: the terrain must visibly
// stay a background for the aircraft (aircraft readability > terrain
// richness). The worst-case per-channel deviation from the material base
// color is bounded by TERRAIN_MAX_CHANNEL_DEVIATION; typical vertices land
// well below it because the noise layers rarely align.

/// Wavelength of the macroscopic brightness patches, in metres.
const TERRAIN_MACRO_SCALE_M: f32 = 90.0;
/// Wavelength of the medium brightness patches, in metres.
const TERRAIN_MEDIUM_SCALE_M: f32 = 25.0;
/// Peak luminance contribution of the macro layer, as a fraction of base.
const TERRAIN_MACRO_VARIATION: f32 = 0.10;
/// Peak luminance contribution of the medium layer, as a fraction of base.
const TERRAIN_MEDIUM_VARIATION: f32 = 0.05;
/// Peak luminance brightening on a fully vertical wall (normal_y = 0).
const TERRAIN_SLOPE_VARIATION: f32 = 0.04;
/// Elevation bias gradient, in luminance per metre of height.
const TERRAIN_ELEVATION_GRADIENT_1_PER_M: f32 = 0.0015;
/// Cap on the elevation bias, as a fraction of base.
const TERRAIN_ELEVATION_MAX_BIAS: f32 = 0.02;
/// Peak per-channel warm/cool tint, as a fraction of base.
const TERRAIN_TINT_VARIATION: f32 = 0.04;
/// Hard upper bound on any single output channel deviation from the base:
/// the worst exactly-achievable deviation is 0.2584 (luminance swing 0.21
/// combined with the full tint swing), rounded up with margin.
const TERRAIN_MAX_CHANNEL_DEVIATION: f32 = 0.26;
/// Fixed seeds keep the pattern stable across builds and platforms.
const TERRAIN_MACRO_SEED: u32 = 0x2F6E_B9D1;
const TERRAIN_MEDIUM_SEED: u32 = 0x8A4C_3D70;
const TERRAIN_TINT_SEED: u32 = 0x1B5E_9C43;

/// Deterministic terrain surface color for one vertex.
///
/// Pure input -> output mapping computed once at chunk generation:
/// - no allocation, no mutable state, no time, no camera data;
/// - anchored to render-space (x, z), the same coordinates used for
///   `Vertex.position`, so shared boundary vertices between adjacent chunks
///   (and across different chunkings) always resolve to the same color;
/// - `base` (the material `base_color_factor`) stays the chromatic authority.
fn terrain_surface_color(
    base: [f32; 4],
    render_x: f32,
    render_z: f32,
    elevation: f32,
    normal_y: f32,
) -> [f32; 4] {
    // Slope response: a fully flat surface (normal_y = 1) contributes zero;
    // steeper slopes brighten slightly, as if exposing drier grass.
    let slope = (1.0 - normal_y).clamp(0.0, 1.0);
    // Elevation response: gentle lightening with height, hard-capped so the
    // effect stays sober even on large height fields. No global elevation
    // range is assumed; the gradient is deliberately tiny.
    let elevation_bias = (elevation * TERRAIN_ELEVATION_GRADIENT_1_PER_M)
        .clamp(-TERRAIN_ELEVATION_MAX_BIAS, TERRAIN_ELEVATION_MAX_BIAS);

    let luminance = 1.0
        + TERRAIN_MACRO_VARIATION
            * value_noise_2d(
                render_x,
                render_z,
                TERRAIN_MACRO_SCALE_M,
                TERRAIN_MACRO_SEED,
            )
        + TERRAIN_MEDIUM_VARIATION
            * value_noise_2d(
                render_x,
                render_z,
                TERRAIN_MEDIUM_SCALE_M,
                TERRAIN_MEDIUM_SEED,
            )
        + TERRAIN_SLOPE_VARIATION * slope
        + elevation_bias;

    // Tiny warm/cool tint: a positive value lifts R and drops B (drier
    // grass), negative does the opposite (denser green). The amplitude is
    // small enough that no hue jumps are visible.
    let tint = TERRAIN_TINT_VARIATION
        * value_noise_2d(
            render_x,
            render_z,
            TERRAIN_MEDIUM_SCALE_M,
            TERRAIN_TINT_SEED,
        );

    let red = (base[0] * luminance * (1.0 + tint)).clamp(0.0, 1.0);
    let green = (base[1] * luminance * (1.0 - 0.5 * tint)).clamp(0.0, 1.0);
    let blue = (base[2] * luminance * (1.0 - tint)).clamp(0.0, 1.0);

    // Keep the documented deviation bound honest even in debug builds. Sound
    // for any material base in [0, 1]: clamping only shrinks the deviation.
    debug_assert!(
        red.is_finite() && (red - base[0]).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
        "red channel exceeds the documented deviation bound"
    );
    debug_assert!(
        green.is_finite() && (green - base[1]).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
        "green channel exceeds the documented deviation bound"
    );
    debug_assert!(
        blue.is_finite() && (blue - base[2]).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
        "blue channel exceeds the documented deviation bound"
    );

    // Alpha is a material property; G2D never invents alpha variation.
    [red, green, blue, base[3]]
}

/// 2D lattice value noise returning [-1, 1].
///
/// Bilinear interpolation of deterministic per-cell hash values with a
/// smoothstep kernel. `scale_m` is the metric wavelength of the pattern;
/// the function is anchored to render-space metres.
fn value_noise_2d(x: f32, z: f32, scale_m: f32, seed: u32) -> f32 {
    let sx = x / scale_m;
    let sz = z / scale_m;
    let x0 = sx.floor();
    let z0 = sz.floor();
    let fx = sx - x0;
    let fz = sz - z0;

    let ix0 = x0 as i32;
    let iz0 = z0 as i32;

    let v00 = lattice_hash_unit(ix0, iz0, seed);
    let v10 = lattice_hash_unit(ix0 + 1, iz0, seed);
    let v01 = lattice_hash_unit(ix0, iz0 + 1, seed);
    let v11 = lattice_hash_unit(ix0 + 1, iz0 + 1, seed);

    let tx = smoothstep_t(fx);
    let tz = smoothstep_t(fz);

    let top = v00 + (v10 - v00) * tx;
    let bottom = v01 + (v11 - v01) * tx;
    let value = top + (bottom - top) * tz;

    value * 2.0 - 1.0
}

/// Smoothstep interpolation kernel: `t*t*(3 - 2*t)`, zero derivative at both
/// ends for a kink-free blend between lattice cells.
fn smoothstep_t(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

/// Deterministic lattice corner hash in [0, 1].
///
/// Wrapping integer mixing only: no allocation, no platform state. Negative
/// cell indices wrap deterministically through `as u32` (defined behaviour).
fn lattice_hash_unit(x: i32, z: i32, seed: u32) -> f32 {
    let mut h = seed ^ (x as u32).wrapping_mul(0x85EB_CA6B);
    h = h.wrapping_add((z as u32).wrapping_mul(0xC2B2_AE35));
    h = h.wrapping_mul(0x9E37_79B9);
    h ^= h >> 16;
    h = h.wrapping_mul(0x85EB_CA6B);
    h ^= h >> 13;
    h = h.wrapping_mul(0xC2B2_AE35);
    h ^= h >> 16;
    ((h >> 8) & 0xFFFF) as f32 * (1.0 / 65_535.0)
}

/// Generate a flat terrain height field.
#[must_use]
pub fn generate_flat_terrain(
    width_cells: u32,
    depth_cells: u32,
    sample_spacing_m: f32,
    elevation: f32,
) -> TerrainHeightField {
    let elevations = vec![elevation; (width_cells as usize + 1) * (depth_cells as usize + 1)];
    TerrainHeightField::new(width_cells, depth_cells, sample_spacing_m, elevations)
}

/// Generate a rolling terrain height field using deterministic mathematical functions.
#[must_use]
pub fn generate_rolling_terrain(
    width_cells: u32,
    depth_cells: u32,
    sample_spacing_m: f32,
    base_elevation: f32,
    amplitude: f32,
) -> TerrainHeightField {
    let width_m = width_cells as f32 * sample_spacing_m;
    let depth_m = depth_cells as f32 * sample_spacing_m;

    let mut elevations =
        Vec::with_capacity((width_cells as usize + 1) * (depth_cells as usize + 1));

    for z in 0..=depth_cells {
        let world_z = z as f32 * sample_spacing_m;
        for x in 0..=width_cells {
            let world_x = x as f32 * sample_spacing_m;

            let freq_x = 2.0 * PI / width_m;
            let freq_z = 2.0 * PI / depth_m;

            let h1 = (world_x * freq_x * 2.0).sin() * (world_z * freq_z * 3.0).cos();
            let h2 =
                (world_x * freq_x * 5.0 + 1.0).sin() * (world_z * freq_z * 4.0 + 2.0).cos() * 0.5;
            let h3 =
                (world_x * freq_x * 8.0 + 3.0).cos() * (world_z * freq_z * 6.0 + 1.0).sin() * 0.25;

            let elevation = base_elevation + amplitude * (h1 + h2 + h3) / 1.75;
            elevations.push(elevation);
        }
    }

    TerrainHeightField::new(width_cells, depth_cells, sample_spacing_m, elevations)
}

/// Generate all chunks for a terrain height field with a render-space offset.
#[must_use]
pub fn generate_terrain_chunks(
    height_field: &TerrainHeightField,
    cells_per_chunk: u32,
    material: &TerrainMaterial,
    render_origin_offset: [f32; 2],
) -> Vec<TerrainChunk> {
    let chunks_x = height_field.width_cells.div_ceil(cells_per_chunk);
    let chunks_z = height_field.depth_cells.div_ceil(cells_per_chunk);

    let mut chunks = Vec::with_capacity((chunks_x * chunks_z) as usize);

    for chunk_z in 0..chunks_z {
        for chunk_x in 0..chunks_x {
            let chunk = TerrainChunk::generate(
                height_field,
                chunk_x,
                chunk_z,
                cells_per_chunk,
                material,
                render_origin_offset,
            );
            if !chunk.vertices.is_empty() {
                chunks.push(chunk);
            }
        }
    }

    chunks
}

/// Generate terrain chunks centered around the render origin.
///
/// Computes the offset as `[-width_m/2, -depth_m/2]` so the terrain
/// extends equally in both +X/-X and +Z/-Z from the render origin.
#[must_use]
pub fn generate_centered_terrain_chunks(
    height_field: &TerrainHeightField,
    cells_per_chunk: u32,
    material: &TerrainMaterial,
) -> Vec<TerrainChunk> {
    let offset = [-height_field.width_m() / 2.0, -height_field.depth_m() / 2.0];
    generate_terrain_chunks(height_field, cells_per_chunk, material, offset)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    #[test]
    fn flat_terrain_has_correct_dimensions() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 0.0);
        assert_eq!(terrain.width_cells, 10);
        assert_eq!(terrain.depth_cells, 10);
        assert_eq!(terrain.width_m(), 10.0);
        assert_eq!(terrain.depth_m(), 10.0);
    }

    #[test]
    fn flat_terrain_has_correct_vertex_count() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 0.0);
        assert_eq!(terrain.elevations.len(), 121);
    }

    #[test]
    fn flat_terrain_all_elevations_equal() {
        let terrain = generate_flat_terrain(5, 5, 2.0, -10.0);
        assert!(terrain.elevations.iter().all(|&e| e == -10.0));
    }

    #[test]
    fn rolling_terrain_elevations_are_finite() {
        let terrain = generate_rolling_terrain(20, 20, 5.0, 0.0, 10.0);
        assert!(terrain.elevations.iter().all(|e| e.is_finite()));
    }

    #[test]
    fn rolling_terrain_has_no_nan_or_inf() {
        let terrain = generate_rolling_terrain(32, 32, 10.0, -5.0, 15.0);
        assert!(
            terrain
                .elevations
                .iter()
                .all(|e| !e.is_nan() && !e.is_infinite())
        );
    }

    #[test]
    fn terrain_chunk_has_correct_vertex_count() {
        let terrain = generate_flat_terrain(32, 32, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
        assert_eq!(chunk.vertices.len(), 33 * 33);
    }

    #[test]
    fn terrain_chunk_has_correct_index_count() {
        let terrain = generate_flat_terrain(32, 32, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
        assert_eq!(chunk.indices.len(), 32 * 32 * 6);
    }

    #[test]
    fn terrain_chunk_indices_are_in_bounds() {
        let terrain = generate_flat_terrain(32, 32, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
        let vertex_count = chunk.vertices.len() as u32;
        assert!(chunk.indices.iter().all(|&i| i < vertex_count));
    }

    #[test]
    fn terrain_chunk_triangle_front_faces_point_up() {
        // G3A-R: with `front_face = Ccw` and back-face culling, the field's
        // front faces must point UP (+Y, away from the earth) or the culled
        // terrain never rasterizes. The triangle normal via the right-hand
        // rule over the ordered winding must therefore have a positive Y.
        let terrain = generate_flat_terrain(2, 2, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 2, &material, [0.0; 2]);

        let i0 = chunk.indices[0] as usize;
        let i1 = chunk.indices[1] as usize;
        let i2 = chunk.indices[2] as usize;

        let v0 = chunk.vertices[i0].position;
        let v1 = chunk.vertices[i1].position;
        let v2 = chunk.vertices[i2].position;

        let e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

        // cross(e1, e2).y = e1[2] * e2[0] - e1[0] * e2[2].
        let cross_y = e1[2] * e2[0] - e1[0] * e2[2];
        assert!(
            cross_y > 0.0,
            "front face normal must point up (+Y), got cross_y = {}",
            cross_y
        );
    }

    #[test]
    fn flat_terrain_normals_point_upward() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 10, &material, [0.0; 2]);

        for vertex in &chunk.vertices {
            let length =
                (vertex.normal[0].powi(2) + vertex.normal[1].powi(2) + vertex.normal[2].powi(2))
                    .sqrt();
            assert!(
                (length - 1.0).abs() < 1e-5,
                "normal not unit length: {:?}",
                vertex.normal
            );
            assert!(
                vertex.normal[1] > 0.99,
                "flat terrain normal should point up: {:?}",
                vertex.normal
            );
        }
    }

    #[test]
    fn rolling_terrain_normals_are_finite_and_unit_length() {
        let terrain = generate_rolling_terrain(20, 20, 5.0, 0.0, 10.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 20, &material, [0.0; 2]);

        for vertex in &chunk.vertices {
            assert!(
                vertex.normal.iter().all(|c| c.is_finite()),
                "non-finite normal: {:?}",
                vertex.normal
            );
            let length_sq =
                vertex.normal[0].powi(2) + vertex.normal[1].powi(2) + vertex.normal[2].powi(2);
            assert!(
                (length_sq - 1.0).abs() < 1e-4,
                "normal not unit length: {:?}",
                vertex.normal
            );
        }
    }

    #[test]
    fn terrain_uv_scale_independent_of_tessellation() {
        let material = TerrainMaterial {
            texture_scale_m: 10.0,
            ..Default::default()
        };

        let terrain_coarse = generate_flat_terrain(10, 10, 10.0, 0.0);
        let chunk_coarse = TerrainChunk::generate(&terrain_coarse, 0, 0, 10, &material, [0.0; 2]);

        let terrain_fine = generate_flat_terrain(100, 100, 1.0, 0.0);
        let chunk_fine = TerrainChunk::generate(&terrain_fine, 0, 0, 100, &material, [0.0; 2]);

        let corner_coarse = chunk_coarse.vertices.last().unwrap().uv;
        let corner_fine = chunk_fine.vertices.last().unwrap().uv;

        assert!((corner_coarse[0] - corner_fine[0]).abs() < 1e-5);
        assert!((corner_coarse[1] - corner_fine[1]).abs() < 1e-5);
    }

    #[test]
    fn terrain_generation_is_deterministic() {
        let terrain1 = generate_rolling_terrain(32, 32, 5.0, 0.0, 10.0);
        let terrain2 = generate_rolling_terrain(32, 32, 5.0, 0.0, 10.0);

        assert_eq!(terrain1.elevations.len(), terrain2.elevations.len());
        for (a, b) in terrain1.elevations.iter().zip(terrain2.elevations.iter()) {
            assert_eq!(a.to_bits(), b.to_bits(), "elevations differ");
        }
    }

    #[test]
    fn adjacent_chunks_share_boundary_coordinates() {
        let terrain = generate_flat_terrain(64, 64, 1.0, 0.0);
        let material = TerrainMaterial::default();

        let chunk_00 = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
        let chunk_10 = TerrainChunk::generate(&terrain, 1, 0, 32, &material, [0.0; 2]);

        let chunk_00_right_edge: Vec<_> = chunk_00
            .vertices
            .iter()
            .filter(|v| (v.position[0] - 32.0).abs() < 1e-5)
            .collect();

        let chunk_10_left_edge: Vec<_> = chunk_10
            .vertices
            .iter()
            .filter(|v| (v.position[0] - 32.0).abs() < 1e-5)
            .collect();

        assert!(
            !chunk_00_right_edge.is_empty(),
            "chunk 00 should have right edge at x=32"
        );
        assert!(
            !chunk_10_left_edge.is_empty(),
            "chunk 10 should have left edge at x=32"
        );

        let chunk_00_z_values: Vec<_> = chunk_00_right_edge.iter().map(|v| v.position[2]).collect();
        let chunk_10_z_values: Vec<_> = chunk_10_left_edge.iter().map(|v| v.position[2]).collect();

        assert!(chunk_00_z_values.iter().any(|&z| z.abs() < 1e-5));
        assert!(chunk_10_z_values.iter().any(|&z| z.abs() < 1e-5));
    }

    #[test]
    fn terrain_bounds_are_correct() {
        let terrain = generate_rolling_terrain(20, 20, 5.0, 0.0, 10.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 20, &material, [0.0; 2]);

        let (min, max) = chunk.bounds;

        assert!((min[0] - chunk.world_origin[0]).abs() < 1e-5);
        assert!((max[0] - (chunk.world_origin[0] + chunk.size_m[0])).abs() < 1e-5);

        assert!((min[2] - chunk.world_origin[1]).abs() < 1e-5);
        assert!((max[2] - (chunk.world_origin[1] + chunk.size_m[1])).abs() < 1e-5);

        for vertex in &chunk.vertices {
            assert!(vertex.position[1] >= min[1] - 1e-5);
            assert!(vertex.position[1] <= max[1] + 1e-5);
        }
    }

    #[test]
    fn chunk_ordering_is_deterministic() {
        let terrain = generate_flat_terrain(64, 64, 1.0, 0.0);
        let material = TerrainMaterial::default();

        let chunks1 = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);
        let chunks2 = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);

        assert_eq!(chunks1.len(), chunks2.len());
        for (c1, c2) in chunks1.iter().zip(chunks2.iter()) {
            assert_eq!(c1.chunk_coords, c2.chunk_coords);
        }
    }

    #[test]
    fn bilinear_interpolation_at_sample_points_matches_exact() {
        let terrain = generate_rolling_terrain(10, 10, 1.0, 0.0, 5.0);

        for z in 0..=10 {
            for x in 0..=10 {
                let local_x = x as f32;
                let local_z = z as f32;
                let sampled = terrain.sample_bilinear(local_x, local_z);
                let exact = terrain.elevation_at(x, z);
                assert!((sampled - exact).abs() < 1e-5, "mismatch at ({}, {})", x, z);
            }
        }
    }

    #[test]
    fn bilinear_interpolation_clamps_outside_bounds() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 5.0);

        let sampled = terrain.sample_bilinear(-100.0, -100.0);
        assert!((sampled - 5.0).abs() < 1e-5);

        let sampled = terrain.sample_bilinear(100.0, 100.0);
        assert!((sampled - 5.0).abs() < 1e-5);
    }

    #[test]
    fn terrain_vertices_have_no_nan_or_inf() {
        let terrain = generate_rolling_terrain(32, 32, 10.0, -5.0, 15.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);

        for vertex in &chunk.vertices {
            assert!(
                vertex.position.iter().all(|c| c.is_finite()),
                "non-finite position"
            );
            assert!(
                vertex.normal.iter().all(|c| c.is_finite()),
                "non-finite normal"
            );
            assert!(
                vertex.color.iter().all(|c| c.is_finite()),
                "non-finite color"
            );
            assert!(vertex.uv.iter().all(|c| c.is_finite()), "non-finite uv");
        }
    }

    // -----------------------------------------------------------------------
    // G1C review fix tests: centered terrain
    // -----------------------------------------------------------------------

    #[test]
    fn centered_terrain_contains_render_origin() {
        let terrain = generate_rolling_terrain(200, 200, 5.0, -10.0, 3.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);

        // The render origin (0, 0) must be inside the terrain bounds.
        let min_x = chunks
            .iter()
            .map(|c| c.bounds.0[0])
            .fold(f32::INFINITY, f32::min);
        let max_x = chunks
            .iter()
            .map(|c| c.bounds.1[0])
            .fold(f32::NEG_INFINITY, f32::max);
        let min_z = chunks
            .iter()
            .map(|c| c.bounds.0[2])
            .fold(f32::INFINITY, f32::min);
        let max_z = chunks
            .iter()
            .map(|c| c.bounds.1[2])
            .fold(f32::NEG_INFINITY, f32::max);

        assert!(
            min_x < 0.0,
            "terrain must extend below X=0, got min_x={min_x}"
        );
        assert!(
            max_x > 0.0,
            "terrain must extend above X=0, got max_x={max_x}"
        );
        assert!(
            min_z < 0.0,
            "terrain must extend below Z=0, got min_z={min_z}"
        );
        assert!(
            max_z > 0.0,
            "terrain must extend above Z=0, got max_z={max_z}"
        );
    }

    #[test]
    fn centered_terrain_extends_in_both_x_directions() {
        let terrain = generate_flat_terrain(100, 100, 10.0, 0.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);

        let has_negative_x = chunks.iter().any(|c| c.bounds.0[0] < 0.0);
        let has_positive_x = chunks.iter().any(|c| c.bounds.1[0] > 0.0);
        assert!(has_negative_x, "centered terrain must have chunks at -X");
        assert!(has_positive_x, "centered terrain must have chunks at +X");
    }

    #[test]
    fn centered_terrain_extends_in_both_z_directions() {
        let terrain = generate_flat_terrain(100, 100, 10.0, 0.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);

        let has_negative_z = chunks.iter().any(|c| c.bounds.0[2] < 0.0);
        let has_positive_z = chunks.iter().any(|c| c.bounds.1[2] > 0.0);
        assert!(has_negative_z, "centered terrain must have chunks at -Z");
        assert!(has_positive_z, "centered terrain must have chunks at +Z");
    }

    #[test]
    fn centered_adjacent_chunk_seams_match() {
        let terrain = generate_flat_terrain(64, 64, 10.0, 0.0);
        let material = TerrainMaterial::default();
        let offset = [-320.0, -320.0];
        let chunks = generate_terrain_chunks(&terrain, 32, &material, offset);

        assert!(
            chunks.len() >= 4,
            "expected at least 4 chunks for 64x64 with 32-cell chunks"
        );

        // Find two horizontally adjacent chunks and verify seam positions match.
        let first = &chunks[0];
        let second = chunks.iter().find(|c| {
            c.chunk_coords.0 == first.chunk_coords.0 + 1 && c.chunk_coords.1 == first.chunk_coords.1
        });

        if let Some(second) = second {
            let first_max_x = first.bounds.1[0];
            let second_min_x = second.bounds.0[0];
            assert!(
                (first_max_x - second_min_x).abs() < 1e-4,
                "adjacent chunk seam mismatch: {} vs {}",
                first_max_x,
                second_min_x
            );
        }
    }

    #[test]
    fn centered_chunk_ordering_is_deterministic() {
        let terrain = generate_flat_terrain(64, 64, 10.0, 0.0);
        let material = TerrainMaterial::default();

        let chunks1 = generate_centered_terrain_chunks(&terrain, 32, &material);
        let chunks2 = generate_centered_terrain_chunks(&terrain, 32, &material);

        assert_eq!(chunks1.len(), chunks2.len());
        for (c1, c2) in chunks1.iter().zip(chunks2.iter()) {
            assert_eq!(c1.chunk_coords, c2.chunk_coords);
            assert_eq!(c1.world_origin, c2.world_origin);
        }
    }

    #[test]
    fn terrain_material_color_stays_authority_within_bounds() {
        // G2D: the material base_color_factor remains the chromatic authority;
        // vertex colors are now modulated, but only within the explicit G2D
        // deviation bound, and alpha stays exactly the material alpha.
        let terrain = generate_flat_terrain(16, 16, 2.0, 0.0);
        let material = TerrainMaterial {
            base_color_factor: [0.25, 0.5, 0.75, 1.0],
            texture_scale_m: 4.0,
            ..Default::default()
        };
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 16, &material, [0.0; 2]);

        for vertex in &chunk.vertices {
            for (channel, base) in vertex.color[..3].iter().zip(material.base_color_factor) {
                assert!(
                    (channel - base).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
                    "channel {channel} deviates too far from base {base}"
                );
            }
            assert_eq!(
                vertex.color[3], 1.0,
                "alpha must stay exactly the material alpha"
            );
        }
    }

    #[test]
    fn sample_bilinear_render_space_accounts_for_offset() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 5.0);
        let offset = [-5.0, -5.0];

        // At render origin (0, 0), local coords are (5, 5).
        let sampled = terrain.sample_bilinear_render_space(0.0, 0.0, offset);
        assert!((sampled - 5.0).abs() < 1e-5);

        // At render coord (-5, -5), local coords are (0, 0).
        let sampled = terrain.sample_bilinear_render_space(-5.0, -5.0, offset);
        assert!((sampled - 5.0).abs() < 1e-5);
    }

    // -----------------------------------------------------------------------
    // G2D: terrain surface variation regression tests
    // -----------------------------------------------------------------------

    fn find_vertex(chunks: &[TerrainChunk], render_x: f32, render_z: f32) -> &Vertex {
        chunks
            .iter()
            .flat_map(|c| c.vertices.iter())
            .find(|v| {
                (v.position[0] - render_x).abs() < 1e-4 && (v.position[2] - render_z).abs() < 1e-4
            })
            .expect("probe vertex must exist in the generated terrain")
    }

    #[test]
    fn g2d_flat_terrain_full_chunk_counts() {
        let terrain = generate_flat_terrain(64, 64, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunks = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);
        assert_eq!(chunks.len(), 4);
        let vertices: usize = chunks.iter().map(|c| c.vertices.len()).sum();
        let indices: usize = chunks.iter().map(|c| c.indices.len()).sum();
        assert_eq!(vertices, 4 * 33 * 33);
        assert_eq!(indices, 4 * 32 * 32 * 6);
        assert_eq!(indices / 3, 4 * 32 * 32 * 2);
    }

    #[test]
    fn g2d_flat_terrain_elevation_exact_everywhere() {
        let terrain = generate_flat_terrain(16, 16, 1.0, 7.5);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 16, &material, [0.0; 2]);
        assert!(chunk.vertices.iter().all(|v| v.position[1] == 7.5));
    }

    #[test]
    fn g2d_flat_terrain_vertex_positions_correct() {
        let terrain = generate_flat_terrain(8, 8, 2.0, 3.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 8, &material, [0.0; 2]);
        for (i, vertex) in chunk.vertices.iter().enumerate() {
            let local_x = (i % 9) as f32;
            let local_z = (i / 9) as f32;
            assert_eq!(vertex.position, [local_x * 2.0, 3.0, local_z * 2.0]);
        }
    }

    #[test]
    fn g2d_flat_terrain_normals_still_exact_up() {
        let terrain = generate_flat_terrain(64, 64, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunks = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);
        for chunk in &chunks {
            for vertex in &chunk.vertices {
                assert_eq!(vertex.normal, [0.0, 1.0, 0.0]);
            }
        }
    }

    #[test]
    fn g2d_uv_mapping_unchanged() {
        let material = TerrainMaterial {
            texture_scale_m: 4.0,
            ..Default::default()
        };
        let terrain = generate_flat_terrain(8, 8, 2.0, 0.0);
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 8, &material, [0.0; 2]);
        for vertex in &chunk.vertices {
            let expected = [vertex.position[0] / 4.0, vertex.position[2] / 4.0];
            assert_eq!(vertex.uv, expected);
        }
    }

    #[test]
    fn g2d_alpha_is_exactly_material_alpha() {
        // Default terrain.
        let terrain = generate_flat_terrain(16, 16, 1.0, 0.0);
        let chunk =
            TerrainChunk::generate(&terrain, 0, 0, 16, &TerrainMaterial::default(), [0.0; 2]);
        assert!(chunk.vertices.iter().all(|v| v.color[3] == 1.0));
        // Custom alpha.
        let material = TerrainMaterial {
            base_color_factor: [0.2, 0.4, 0.6, 0.4],
            ..Default::default()
        };
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 16, &material, [0.0; 2]);
        assert!(chunk.vertices.iter().all(|v| v.color[3] == 0.4));
    }

    #[test]
    fn g2d_color_function_alpha_is_exactly_base() {
        for base in [
            [0.25, 0.45, 0.18, 1.0],
            [1.0, 1.0, 1.0, 0.0],
            [0.1, 0.2, 0.3, 0.5],
        ] {
            let out = terrain_surface_color(base, 12.5, -3.25, 2.0, 0.9);
            assert_eq!(out[3], base[3]);
        }
    }

    #[test]
    fn g2d_large_flat_terrain_has_color_variation() {
        // 256 m field: the 25 m/90 m layers guarantee many distinct colors.
        let terrain = generate_flat_terrain(128, 128, 2.0, 0.0);
        let material = TerrainMaterial::default();
        let chunks = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);
        let mut distinct = HashSet::new();
        for chunk in &chunks {
            for vertex in &chunk.vertices {
                distinct.insert(vertex.color.map(f32::to_bits));
            }
        }
        assert!(
            distinct.len() > 16,
            "expected visible color variation, got {}",
            distinct.len()
        );
    }

    #[test]
    fn g2d_variation_sober_for_flat_and_rolling() {
        // Covers: colors finite, in [0,1], within the explicit deviation
        // bound, alpha preserved (spec tests 9, 10, 11, 13, 32).
        for terrain in [
            generate_flat_terrain(200, 200, 5.0, -2.0),
            generate_rolling_terrain(200, 200, 5.0, -2.0, 3.0),
        ] {
            let material = TerrainMaterial::default();
            let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);
            for chunk in &chunks {
                for vertex in &chunk.vertices {
                    for (channel, base) in vertex.color[..3].iter().zip(material.base_color_factor)
                    {
                        assert!(channel.is_finite(), "non-finite color channel");
                        assert!(
                            (0.0..=1.0).contains(channel),
                            "color channel out of range: {channel}"
                        );
                        assert!(
                            (channel - base).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
                            "deviation {} exceeds bound for base {base}",
                            (channel - base).abs()
                        );
                    }
                    assert_eq!(vertex.color[3], 1.0);
                }
            }
        }
    }

    #[test]
    fn g2d_color_function_is_deterministic_bitwise() {
        let base = TerrainMaterial::default().base_color_factor;
        let a = terrain_surface_color(base, 123.5, -456.25, 3.75, 0.98);
        let b = terrain_surface_color(base, 123.5, -456.25, 3.75, 0.98);
        assert_eq!(a.map(f32::to_bits), b.map(f32::to_bits));
    }

    #[test]
    fn g2d_macro_noise_deterministic_and_bounded() {
        let a = value_noise_2d(123.5, -456.25, TERRAIN_MACRO_SCALE_M, 3);
        let b = value_noise_2d(123.5, -456.25, TERRAIN_MACRO_SCALE_M, 3);
        assert_eq!(a.to_bits(), b.to_bits());
        assert!((-1.0..=1.0).contains(&a));
    }

    #[test]
    fn g2d_medium_noise_deterministic_and_bounded() {
        let a = value_noise_2d(123.5, -456.25, TERRAIN_MEDIUM_SCALE_M, 9);
        let b = value_noise_2d(123.5, -456.25, TERRAIN_MEDIUM_SCALE_M, 9);
        assert_eq!(a.to_bits(), b.to_bits());
        assert!((-1.0..=1.0).contains(&a));
    }

    #[test]
    fn g2d_full_generation_repeatable_bitwise() {
        let terrain = generate_rolling_terrain(48, 48, 2.0, 0.0, 2.0);
        let material = TerrainMaterial::default();
        let first = generate_terrain_chunks(&terrain, 32, &material, [-48.0, -48.0]);
        let second = generate_terrain_chunks(&terrain, 32, &material, [-48.0, -48.0]);
        assert_eq!(first.len(), second.len());
        for (a, b) in first.iter().zip(second.iter()) {
            assert_eq!(a.chunk_coords, b.chunk_coords);
            assert_eq!(a.indices, b.indices);
            for (va, vb) in a.vertices.iter().zip(b.vertices.iter()) {
                assert_eq!(va.position.map(f32::to_bits), vb.position.map(f32::to_bits));
                assert_eq!(va.normal.map(f32::to_bits), vb.normal.map(f32::to_bits));
                assert_eq!(va.uv.map(f32::to_bits), vb.uv.map(f32::to_bits));
                assert_eq!(va.color.map(f32::to_bits), vb.color.map(f32::to_bits));
            }
        }
    }

    #[test]
    fn g2d_noise_finite_and_bounded_on_positive_coordinates() {
        for i in 0..80 {
            let x = (i as f32) * 13.7 + 0.3;
            let z = (i as f32) * 7.1 + 2.5;
            for scale in [TERRAIN_MACRO_SCALE_M, TERRAIN_MEDIUM_SCALE_M] {
                let v = value_noise_2d(x, z, scale, 5);
                assert!(v.is_finite(), "non-finite noise at ({x}, {z})");
                assert!((-1.0..=1.0).contains(&v));
            }
        }
    }

    #[test]
    fn g2d_noise_finite_and_bounded_on_negative_coordinates() {
        for i in 0..80 {
            let x = -(i as f32) * 13.7 - 0.7;
            let z = -(i as f32) * 7.1 - 1.3;
            for scale in [TERRAIN_MACRO_SCALE_M, TERRAIN_MEDIUM_SCALE_M] {
                let v = value_noise_2d(x, z, scale, 9);
                assert!(v.is_finite(), "non-finite noise at ({x}, {z})");
                assert!((-1.0..=1.0).contains(&v));
            }
        }
    }

    #[test]
    fn g2d_noise_works_at_origin() {
        let h = lattice_hash_unit(0, 0, 42);
        assert!((0.0..=1.0).contains(&h));
        for scale in [TERRAIN_MACRO_SCALE_M, TERRAIN_MEDIUM_SCALE_M] {
            let v = value_noise_2d(0.0, 0.0, scale, 42);
            assert!(v.is_finite());
            assert!((-1.0..=1.0).contains(&v));
        }
    }

    #[test]
    fn g2d_noise_continuous_across_lattice_boundaries() {
        for k in [-3.0, -1.0, 1.0, 5.0, 17.0] {
            let boundary = k * TERRAIN_MEDIUM_SCALE_M;
            let just_before = boundary - 1e-3;
            let va = value_noise_2d(just_before, 3.0, TERRAIN_MEDIUM_SCALE_M, 7);
            let vb = value_noise_2d(boundary, 3.0, TERRAIN_MEDIUM_SCALE_M, 7);
            assert!(va.is_finite() && vb.is_finite(), "non-finite at {boundary}");
            assert!(
                (va - vb).abs() < 1e-3,
                "discontinuity at {boundary}: {va} vs {vb}"
            );
        }
    }

    #[test]
    fn g2d_lattice_hash_bounded_including_negative_indices() {
        for ix in -64..=64 {
            for iz in -64..=64 {
                let v = lattice_hash_unit(ix, iz, 11);
                assert!(
                    v.is_finite() && (0.0..=1.0).contains(&v),
                    "hash out of range at ({ix}, {iz}): {v}"
                );
            }
        }
    }

    #[test]
    fn g2d_adjacent_chunks_share_colors_on_x_and_z_seams() {
        for terrain in [
            generate_flat_terrain(64, 64, 1.0, 0.0),
            generate_rolling_terrain(64, 64, 1.0, 0.5, 1.0),
        ] {
            let material = TerrainMaterial::default();

            // Shared X boundary at render x = 32.
            let a = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
            let b = TerrainChunk::generate(&terrain, 1, 0, 32, &material, [0.0; 2]);
            let a_edge: Vec<_> = a
                .vertices
                .iter()
                .filter(|v| (v.position[0] - 32.0).abs() < 1e-5)
                .collect();
            let b_edge: Vec<_> = b
                .vertices
                .iter()
                .filter(|v| (v.position[0] - 32.0).abs() < 1e-5)
                .collect();
            assert_eq!(a_edge.len(), b_edge.len(), "X seam vertex count");
            for (va, vb) in a_edge.iter().zip(b_edge.iter()) {
                assert_eq!(va.position[2], vb.position[2]);
                assert_eq!(
                    va.color, vb.color,
                    "color seam on X boundary at z={}",
                    va.position[2]
                );
            }

            // Shared Z boundary at render z = 32.
            let c = TerrainChunk::generate(&terrain, 0, 1, 32, &material, [0.0; 2]);
            let a_edge: Vec<_> = a
                .vertices
                .iter()
                .filter(|v| (v.position[2] - 32.0).abs() < 1e-5)
                .collect();
            let c_edge: Vec<_> = c
                .vertices
                .iter()
                .filter(|v| (v.position[2] - 32.0).abs() < 1e-5)
                .collect();
            assert_eq!(a_edge.len(), c_edge.len(), "Z seam vertex count");
            for (va, vc) in a_edge.iter().zip(c_edge.iter()) {
                assert_eq!(va.position[0], vc.position[0]);
                assert_eq!(
                    va.color, vc.color,
                    "color seam on Z boundary at x={}",
                    va.position[0]
                );
            }
        }
    }

    #[test]
    fn g2d_render_origin_offset_does_not_create_seam() {
        // Same world position reached through different grid offsets must
        // produce the same color, because the pattern is render-space based.
        let terrain = generate_flat_terrain(48, 48, 2.0, 0.0);
        let material = TerrainMaterial::default();
        let offset_a = generate_terrain_chunks(&terrain, 24, &material, [0.0, 0.0]);
        let offset_b = generate_terrain_chunks(&terrain, 24, &material, [-40.0, 0.0]);
        for z in [0, 8, 16, 24, 40] {
            let render_z = (z as f32) * 2.0;
            let va = find_vertex(&offset_a, 16.0, render_z);
            let vb = find_vertex(&offset_b, 16.0, render_z);
            assert_eq!(
                va.color, vb.color,
                "render-origin offset seam at render z={render_z}"
            );
            assert_eq!(va.position, vb.position);
        }
    }

    #[test]
    fn g2d_centered_seams_match_world_space_colors() {
        // Centered terrain spans negative render coordinates; seams must
        // still resolve to identical colors on the shared boundaries.
        let terrain = generate_rolling_terrain(64, 64, 1.0, 0.0, 1.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);
        assert_eq!(chunks.len(), 4);
        let c00 = &chunks[0];
        let c10 = &chunks[1];
        let c01 = &chunks[2];

        // Shared X boundary at render x = 0 (grid x = 32 -> 32 - 32 = 0).
        let c00_x: Vec<_> = c00
            .vertices
            .iter()
            .filter(|v| v.position[0] == 0.0)
            .collect();
        let c10_x: Vec<_> = c10
            .vertices
            .iter()
            .filter(|v| v.position[0] == 0.0)
            .collect();
        assert_eq!(c00_x.len(), c10_x.len());
        for (va, vb) in c00_x.iter().zip(c10_x.iter()) {
            assert_eq!(va.position[2], vb.position[2]);
            assert_eq!(va.color, vb.color);
        }

        // Shared Z boundary at render z = 0.
        let c00_z: Vec<_> = c00
            .vertices
            .iter()
            .filter(|v| v.position[2] == 0.0)
            .collect();
        let c01_z: Vec<_> = c01
            .vertices
            .iter()
            .filter(|v| v.position[2] == 0.0)
            .collect();
        assert_eq!(c00_z.len(), c01_z.len());
        for (va, vc) in c00_z.iter().zip(c01_z.iter()) {
            assert_eq!(va.position[0], vc.position[0]);
            assert_eq!(va.color, vc.color);
        }
    }

    #[test]
    fn g2d_chunk_size_does_not_change_color_at_same_world_position() {
        let terrain = generate_rolling_terrain(64, 64, 1.0, 0.0, 1.5);
        let material = TerrainMaterial::default();

        for (offset, probes) in [
            (
                [0.0, 0.0],
                [
                    (16.0, 16.0),
                    (32.0, 32.0),
                    (50.0, 7.0),
                    (63.0, 63.0),
                    (0.0, 0.0),
                    (33.0, 41.0),
                ],
            ),
            (
                [-64.0, -32.0],
                [
                    (-33.0, -2.0),
                    (0.0, 2.0),
                    (-64.0, -32.0),
                    (-50.0, 10.0),
                    (-1.0, -31.0),
                    (-20.0, 0.0),
                ],
            ),
        ] {
            let coarse = generate_terrain_chunks(&terrain, 32, &material, offset);
            let fine = generate_terrain_chunks(&terrain, 16, &material, offset);
            for (px, pz) in probes {
                let va = find_vertex(&coarse, px, pz);
                let vb = find_vertex(&fine, px, pz);
                assert_eq!(
                    va.color, vb.color,
                    "chunk-size seam at ({px}, {pz}) for offset {offset:?}"
                );
                assert_eq!(va.position, vb.position);
            }
        }
    }

    #[test]
    fn g2d_texture_scale_does_not_scale_surface_variation() {
        let terrain = generate_flat_terrain(80, 80, 1.0, 0.0);
        let small_scale = TerrainMaterial::default(); // 4 m
        let large_scale = TerrainMaterial {
            texture_scale_m: 17.0,
            ..Default::default()
        };
        let a = generate_terrain_chunks(&terrain, 32, &small_scale, [0.0; 2]);
        let b = generate_terrain_chunks(&terrain, 32, &large_scale, [0.0; 2]);
        for (px, pz) in [
            (10.0, 20.0),
            (55.0, 5.0),
            (79.0, 79.0),
            (0.0, 34.0),
            (37.0, 41.0),
        ] {
            let va = find_vertex(&a, px, pz);
            let vb = find_vertex(&b, px, pz);
            assert_eq!(
                va.color, vb.color,
                "texture_scale must not resize the G2D variation at ({px}, {pz})"
            );
            assert_ne!(va.uv, vb.uv, "UVs must differ to exercise different scales");
        }
    }

    #[test]
    fn g2d_custom_material_color_is_respected() {
        let terrain = generate_rolling_terrain(64, 64, 2.0, 0.0, 2.0);
        let custom = TerrainMaterial {
            base_color_factor: [0.3, 0.5, 0.2, 0.8],
            ..Default::default()
        };
        let default_material = TerrainMaterial::default();
        let custom_chunks = generate_terrain_chunks(&terrain, 32, &custom, [0.0; 2]);
        let default_chunks = generate_terrain_chunks(&terrain, 32, &default_material, [0.0; 2]);

        for (cc, cd) in custom_chunks.iter().zip(default_chunks.iter()) {
            for (vc, vd) in cc.vertices.iter().zip(cd.vertices.iter()) {
                assert_eq!(
                    vc.position, vd.position,
                    "geometry must not depend on material"
                );
                for (channel, base) in vc.color[..3].iter().zip(custom.base_color_factor) {
                    assert!(
                        (channel - base).abs() <= TERRAIN_MAX_CHANNEL_DEVIATION + 1e-6,
                        "color deviates from custom base: {channel} vs {base}"
                    );
                }
                assert_eq!(vc.color[3], 0.8, "custom alpha must be preserved");
                assert_ne!(
                    vc.color[..3],
                    vd.color[..3],
                    "color must follow the custom base, not the default"
                );
            }
        }
    }

    #[test]
    fn g2d_rolling_elevations_stay_within_expected_band() {
        // The rolling generator is bounded: |sum| <= 1.75 before division,
        // so elevations stay within [base - amplitude, base + amplitude].
        let base = 0.0;
        let amplitude = 10.0;
        let terrain = generate_rolling_terrain(64, 64, 2.0, base, amplitude);
        for &e in &terrain.elevations {
            assert!(e.is_finite());
            assert!((base - amplitude..=base + amplitude).contains(&e));
        }
    }

    #[test]
    fn g2d_flat_terrain_slope_contribution_is_zero() {
        // On flat terrain normal_y == 1.0 exactly, so the slope term is
        // identically zero: the deviation budget tightens by the full slope
        // budget compared with the global TERRAIN_MAX_CHANNEL_DEVIATION.
        let terrain = generate_flat_terrain(200, 200, 5.0, -2.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);
        let no_slope_bound = TERRAIN_MAX_CHANNEL_DEVIATION - TERRAIN_SLOPE_VARIATION;
        for chunk in &chunks {
            for vertex in &chunk.vertices {
                for (channel, base) in vertex.color[..3].iter().zip(material.base_color_factor) {
                    assert!(
                        (channel - base).abs() <= no_slope_bound + 1e-6,
                        "flat terrain exceeded the no-slope bound: {channel} vs {base}"
                    );
                }
            }
        }
    }

    #[test]
    fn g2d_slope_response_bounded_on_inclined_normal() {
        let base = TerrainMaterial::default().base_color_factor;
        let flat = terrain_surface_color(base, 123.0, -45.0, 2.0, 1.0);
        let steep = terrain_surface_color(base, 123.0, -45.0, 2.0, 0.3);
        for (flat_channel, steep_channel) in flat[..3].iter().copied().zip(steep) {
            assert!(steep_channel.is_finite() && flat_channel.is_finite());
            assert!(
                (steep_channel - flat_channel).abs() <= TERRAIN_SLOPE_VARIATION + 1e-5,
                "slope response exceeds its budget"
            );
            assert!(
                steep_channel >= flat_channel,
                "slope must brighten, never darken"
            );
        }
    }

    #[test]
    fn g2d_elevation_bias_finite_and_bounded() {
        let base = TerrainMaterial::default().base_color_factor;
        let low = terrain_surface_color(base, 55.0, 77.0, -10_000.0, 1.0);
        let high = terrain_surface_color(base, 55.0, 77.0, 10_000.0, 1.0);
        let max_swing = 2.0 * TERRAIN_ELEVATION_MAX_BIAS * 1.05;
        for (low_channel, high_channel) in low[..3].iter().copied().zip(high) {
            assert!(low_channel.is_finite() && high_channel.is_finite());
            assert!(
                high_channel >= low_channel,
                "higher terrain must be lighter, never darker"
            );
            assert!(
                (high_channel - low_channel).abs() <= max_swing + 1e-5,
                "elevation swing exceeded its budget"
            );
        }
    }

    #[test]
    fn g2d_geometry_fields_independent_of_color_modulation() {
        let terrain = generate_rolling_terrain(64, 64, 2.0, 0.0, 4.0);
        let material_a = TerrainMaterial {
            base_color_factor: [0.9, 0.1, 0.2, 0.5],
            ..Default::default()
        };
        let material_b = TerrainMaterial::default();
        let chunks_a = generate_terrain_chunks(&terrain, 32, &material_a, [0.0; 2]);
        let chunks_b = generate_terrain_chunks(&terrain, 32, &material_b, [0.0; 2]);

        assert_eq!(chunks_a.len(), chunks_b.len());
        for (a, b) in chunks_a.iter().zip(chunks_b.iter()) {
            assert_eq!(a.chunk_coords, b.chunk_coords);
            assert_eq!(a.world_origin, b.world_origin);
            assert_eq!(a.size_m, b.size_m);
            assert_eq!(a.bounds, b.bounds);
            assert_eq!(a.indices, b.indices);
            for (va, vb) in a.vertices.iter().zip(b.vertices.iter()) {
                assert_eq!(va.position, vb.position);
                assert_eq!(va.normal, vb.normal);
                assert_eq!(va.uv, vb.uv);
                assert_ne!(va.color, vb.color, "colors must differ across materials");
                assert_eq!(va.color[3], 0.5);
                assert_eq!(vb.color[3], 1.0);
            }
        }
    }

    #[test]
    fn g2d_bounds_contain_all_vertices() {
        let terrain = generate_rolling_terrain(200, 200, 5.0, -2.0, 3.0);
        let material = TerrainMaterial::default();
        let chunks = generate_centered_terrain_chunks(&terrain, 32, &material);
        for chunk in &chunks {
            let (min, max) = chunk.bounds;
            for vertex in &chunk.vertices {
                assert!(vertex.position[0] >= min[0] - 1e-4 && vertex.position[0] <= max[0] + 1e-4);
                assert!(vertex.position[1] >= min[1] - 1e-4 && vertex.position[1] <= max[1] + 1e-4);
                assert!(vertex.position[2] >= min[2] - 1e-4 && vertex.position[2] <= max[2] + 1e-4);
            }
        }
    }

    #[test]
    fn g2d_no_unexpected_empty_chunks() {
        let material = TerrainMaterial::default();

        // Non-power-of-two sizes exercise partial edge chunks.
        let terrain = generate_flat_terrain(65, 33, 1.0, 0.0);
        let chunks = generate_terrain_chunks(&terrain, 32, &material, [0.0; 2]);
        assert_eq!(chunks.len(), 3 * 2);
        assert!(
            chunks
                .iter()
                .all(|c| !c.vertices.is_empty() && !c.indices.is_empty())
        );

        // Production-like layout (200 cells, 32-cell chunks -> 49 chunks).
        let production = generate_flat_terrain(200, 200, 5.0, -2.0);
        let chunks = generate_terrain_chunks(&production, 32, &material, [0.0; 2]);
        assert_eq!(chunks.len(), 49);
        assert!(
            chunks
                .iter()
                .all(|c| !c.vertices.is_empty() && !c.indices.is_empty())
        );
    }

    // -----------------------------------------------------------------------
    // G3A: textured terrain material regression tests
    // -----------------------------------------------------------------------

    #[test]
    fn g3a_material_configuration_is_deterministic_and_finite() {
        // Pinned production defaults: dielectric, matte, full-strength normal,
        // world-space tiling at 4 m, white baked base carrying G2D variation.
        let material = TerrainMaterial::default();
        assert_eq!(material.base_color_factor, [1.0, 1.0, 1.0, 1.0]);
        assert_eq!(material.texture_scale_m, DEFAULT_TERRAIN_TEXTURE_SCALE_M);
        assert_eq!(material.metallic, 0.0);
        assert_eq!(material.roughness, 0.9);
        assert_eq!(material.normal_strength, 1.0);
        for offset in [
            material.albedo_uv_offset,
            material.normal_uv_offset,
            material.roughness_uv_offset,
            material.macro_uv_offset,
            material.detail_uv_offset,
            material.ar_offset,
        ] {
            assert!(offset.iter().all(|v| v.is_finite()));
        }

        // Custom configurations stay finite and produce finite chunk output.
        let custom = TerrainMaterial {
            base_color_factor: [0.2, 0.4, 0.6, 0.5],
            texture_scale_m: 8.0,
            metallic: 0.0,
            roughness: 0.75,
            normal_strength: 0.6,
            debug_mode: crate::TerrainDebugMode::Final,
            albedo_uv_offset: [0.1, 0.2],
            normal_uv_offset: [0.3, 0.4],
            roughness_uv_offset: [0.5, 0.6],
            macro_scale_m: 60.0,
            detail_scale_m: 0.5,
            macro_uv_offset: [0.7, 0.8],
            detail_uv_offset: [0.9, 0.1],
            ar_scale: 2.0,
            ar_angle_degrees: 15.0,
            ar_offset: [0.2, 0.3],
            detail_normal_fade_near_m: 10.0,
            detail_normal_fade_far_m: 50.0,
        };
        let terrain = generate_rolling_terrain(16, 16, 2.0, 0.0, 1.0);
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 16, &custom, [0.0; 2]);
        assert!(
            chunk
                .vertices
                .iter()
                .all(|v| v.color.iter().chain(v.uv.iter()).all(|c| c.is_finite()))
        );
    }

    // -----------------------------------------------------------------------
    // G3A-R: three-frequency stack / anti-repetition / distance-fade tests
    // -----------------------------------------------------------------------

    #[test]
    fn g3ar_material_defaults_are_pinned() {
        // Central-tuning authority: changing any value here changes the visual
        // slice contract, so the defaults are asserted explicitly.
        let material = TerrainMaterial::default();
        assert_eq!(material.macro_scale_m, 48.0);
        assert_eq!(material.detail_scale_m, 0.40);
        assert_eq!(material.macro_uv_offset, [0.170, 0.390]);
        assert_eq!(material.detail_uv_offset, [0.163, 0.037]);
        assert_eq!(material.ar_scale, 1.370);
        assert_eq!(material.ar_angle_degrees, 27.0);
        assert_eq!(material.ar_offset, [0.315, 0.571]);
        assert_eq!(material.detail_normal_fade_near_m, 20.0);
        assert_eq!(material.detail_normal_fade_far_m, 80.0);
    }

    #[test]
    fn g3ar_detail_fade_is_monotonic_and_bounded() {
        // Full detail at/inside `near`, zero at/beyond `far`, monotone and
        // zero-derivative at both ends (no popping): the smoothstep contract
        // the shader implements.
        let near = 20.0;
        let far = 80.0;
        assert_eq!(detail_normal_fade_weight(0.0, near, far), 1.0);
        assert_eq!(detail_normal_fade_weight(near, near, far), 1.0);
        assert_eq!(detail_normal_fade_weight(far, near, far), 0.0);
        assert_eq!(detail_normal_fade_weight(10_000.0, near, far), 0.0);

        let mut previous = 1.0f32;
        let mut samples = Vec::new();
        for i in 0..=60 {
            let distance = near + (far - near) * (i as f32 / 60.0);
            let weight = detail_normal_fade_weight(distance, near, far);
            assert!(
                (0.0..=1.0).contains(&weight) && weight.is_finite(),
                "fade weight out of range at {distance}: {weight}"
            );
            assert!(
                weight <= previous + 1e-6,
                "fade weight must be non-increasing: {weight} after {previous}"
            );
            previous = weight;
            samples.push((distance, weight));
        }
        // Monotonicity must be strict somewhere in the middle (a flat plateau
        // at 1.0 or 0.0 would mean the fade does nothing). At t=1/6 the
        // smoothstep has already lost ~7%; at t=5/6 it keeps ~7%.
        let first = samples[10].1;
        let last = samples[50].1;
        assert!(
            first > 0.90 && last < 0.10,
            "fade must actually transition: {first} -> {last}"
        );
        assert!(
            samples[30].1 > 0.25 && samples[30].1 < 0.75,
            "mid-fade must be smooth, got {}",
            samples[30].1
        );
    }

    #[test]
    fn g3ar_fade_is_continuous_around_near_and_far() {
        let near = 20.0;
        let far = 80.0;
        let inside = detail_normal_fade_weight(near - 0.01, near, far);
        let at_near = detail_normal_fade_weight(near, near, far);
        let at_far = detail_normal_fade_weight(far, near, far);
        let outside = detail_normal_fade_weight(far + 0.01, near, far);
        assert!((inside - at_near).abs() < 1e-3, "kink at `near`");
        assert!((at_far - outside).abs() < 1e-3, "kink at `far`");
    }

    #[test]
    fn g3ar_rotated_secondary_uv_is_deterministic() {
        let uv = [12.25, -5.5];
        let a = rotated_secondary_uv(uv, 1.37, 27.0, [0.315, 0.571]);
        let b = rotated_secondary_uv(uv, 1.37, 27.0, [0.315, 0.571]);
        assert_eq!(a.map(f32::to_bits), b.map(f32::to_bits));
        assert!(a.iter().all(|v| v.is_finite()));

        // A zero rotation with identity scale/offset must be the identity.
        let identity = rotated_secondary_uv(uv, 1.0, 0.0, [0.0, 0.0]);
        assert!((identity[0] - uv[0]).abs() < 1e-6);
        assert!((identity[1] - uv[1]).abs() < 1e-6);
    }

    #[test]
    fn g3ar_rotated_secondary_uv_is_chunk_boundary_continuous() {
        // The transform is a pure function of the world-space UV, so the same
        // world position reached through different chunkings must resolve to
        // the same rotated UV — no second-sample seam at chunk borders.
        let terrain = generate_rolling_terrain(128, 128, 2.0, 0.0, 2.0);
        let material = TerrainMaterial::default();
        let coarse = generate_centered_terrain_chunks(&terrain, 64, &material);
        let fine = generate_centered_terrain_chunks(&terrain, 32, &material);

        for world in [(10.0, 20.0), (36.0, -12.0), (0.0, 0.0), (-64.0, 48.0)] {
            let vertex_c = find_vertex(&coarse, world.0, world.1);
            let vertex_f = find_vertex(&fine, world.0, world.1);
            assert_eq!(vertex_c.uv, vertex_f.uv, "uv mismatch at {world:?}");
            let ar_c = rotated_secondary_uv(
                vertex_c.uv,
                material.ar_scale,
                material.ar_angle_degrees,
                material.ar_offset,
            );
            let ar_f = rotated_secondary_uv(
                vertex_f.uv,
                material.ar_scale,
                material.ar_angle_degrees,
                material.ar_offset,
            );
            assert_eq!(
                ar_c.map(f32::to_bits),
                ar_f.map(f32::to_bits),
                "rotated second-sample mismatch at {world:?}"
            );
        }
    }

    #[test]
    fn g3ar_detail_and_macro_stacks_stay_finite_with_custom_material() {
        // The full G3A-R configuration must stay finite through chunk
        // generation and keep the geometry fields untouched.
        let terrain = generate_rolling_terrain(32, 32, 2.0, 0.0, 3.0);
        let custom = TerrainMaterial {
            macro_scale_m: 55.0,
            detail_scale_m: 0.3,
            ar_angle_degrees: 33.0,
            ar_scale: 1.8,
            detail_normal_fade_near_m: 25.0,
            detail_normal_fade_far_m: 120.0,
            ..Default::default()
        };
        let reference = TerrainMaterial::default();
        let chunks_custom = generate_terrain_chunks(&terrain, 16, &custom, [-32.0, -32.0]);
        let chunks_ref = generate_terrain_chunks(&terrain, 16, &reference, [-32.0, -32.0]);
        assert_eq!(chunks_custom.len(), chunks_ref.len());
        for (a, b) in chunks_custom.iter().zip(chunks_ref.iter()) {
            assert_eq!(a.indices, b.indices);
            for (va, vb) in a.vertices.iter().zip(b.vertices.iter()) {
                // Visual-layering parameters must never touch geometry fields.
                assert_eq!(va.position, vb.position);
                assert_eq!(va.normal, vb.normal);
                assert_eq!(va.uv, vb.uv);
                assert!(va.color.iter().all(|c| c.is_finite()));
            }
        }
    }

    #[test]
    fn g3a_uv_mapping_is_world_space_anchored() {
        // uv = render_position / texture_scale_m, unchanged from G2D; the
        // mapping authority is the world position, not the chunk.
        let material = TerrainMaterial {
            texture_scale_m: 4.0,
            ..Default::default()
        };
        let terrain = generate_flat_terrain(8, 8, 2.0, 0.0);
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 8, &material, [0.0; 2]);
        for vertex in &chunk.vertices {
            assert_eq!(
                vertex.uv,
                [vertex.position[0] / 4.0, vertex.position[2] / 4.0]
            );
        }
    }

    #[test]
    fn g3a_uv_and_color_are_chunk_size_independent() {
        // Same world position under two different chunkings must resolve to
        // identical UV and identical G2D color (both anchored to render-space).
        for terrain in [
            generate_flat_terrain(128, 128, 2.0, 0.0),
            generate_rolling_terrain(128, 128, 2.0, 0.0, 2.0),
        ] {
            let material = TerrainMaterial::default();
            let coarse = generate_centered_terrain_chunks(&terrain, 64, &material);
            let fine = generate_centered_terrain_chunks(&terrain, 32, &material);

            // Probe positions must lie exactly on grid vertices (spacing 2 m);
            // centered generation covers negative render coordinates too.
            for world in [(10.0, 20.0), (36.0, -12.0), (0.0, 0.0), (-64.0, 48.0)] {
                let (wx, wz) = world;
                let vertex_c = find_vertex(&coarse, wx, wz);
                let vertex_f = find_vertex(&fine, wx, wz);
                assert_eq!(vertex_c.uv, vertex_f.uv, "uv mismatch at {world:?}");
                assert_eq!(
                    vertex_c.color.map(f32::to_bits),
                    vertex_f.color.map(f32::to_bits),
                    "color mismatch at {world:?}"
                );
            }
        }
    }

    #[test]
    fn g3a_adjacent_chunk_boundary_uv_and_color_match() {
        // Shared boundary vertices between adjacent chunks must carry the same
        // world-space UV and G2D color — no tile or tone seam.
        for terrain in [
            generate_flat_terrain(64, 64, 1.0, 0.0),
            generate_rolling_terrain(64, 64, 1.0, 0.0, 2.0),
        ] {
            let material = TerrainMaterial::default();
            let chunk_00 = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);
            let chunk_10 = TerrainChunk::generate(&terrain, 1, 0, 32, &material, [0.0; 2]);
            let chunk_01 = TerrainChunk::generate(&terrain, 0, 1, 32, &material, [0.0; 2]);

            // X boundary: right edge of (0,0) vs left edge of (1,0).
            let mut pairs = Vec::new();
            for v in &chunk_00.vertices {
                if (v.position[0] - 32.0).abs() < 1e-4 {
                    for w in &chunk_10.vertices {
                        if (w.position[0] - 32.0).abs() < 1e-4 && w.position[2] == v.position[2] {
                            pairs.push((v, w));
                        }
                    }
                }
            }
            // Z boundary: far edge of (0,0) vs near edge of (0,1).
            for v in &chunk_00.vertices {
                if (v.position[2] - 32.0).abs() < 1e-4 {
                    for w in &chunk_01.vertices {
                        if (w.position[2] - 32.0).abs() < 1e-4 && w.position[0] == v.position[0] {
                            pairs.push((v, w));
                        }
                    }
                }
            }
            assert!(
                pairs.len() >= 32,
                "expected shared X and Z boundary vertices, got {}",
                pairs.len()
            );
            for (v, w) in pairs {
                assert_eq!(v.uv, w.uv, "boundary UV mismatch at {:?}", v.position);
                assert_eq!(
                    v.color.map(f32::to_bits),
                    w.color.map(f32::to_bits),
                    "boundary color mismatch at {:?}",
                    v.position
                );
            }
        }
    }

    /// CPU mirror of the `fs_terrain` derivative TBN reconstruction.
    ///
    /// The WGSL fragment shader computes the tangent frame from screen-space
    /// derivatives of `world_position` and `uv`. Its CPU analogue: with
    /// `dp1/duv1` from the +X grid neighbour and `dp2/duv2` from the +Z grid
    /// neighbour, the determinant guard and the `(duv2.y * dp1 - duv1.y * dp2)
    /// / det` reconstruction reduce exactly to `dp1 / duu` and `dp2 / dvv`.
    fn derivative_tbn(
        dp1: [f32; 3],
        dp2: [f32; 3],
        duv1: [f32; 2],
        duv2: [f32; 2],
    ) -> ([f32; 3], [f32; 3], f32) {
        let det = duv1[0] * duv2[1] - duv1[1] * duv2[0];
        let has_basis = det.abs() > 1e-8;
        let inv_det = if has_basis { 1.0 / det } else { 0.0 };
        let normalize_or_world = |v: [f32; 3], fallback: [f32; 3]| -> [f32; 3] {
            let length_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if !has_basis || length_sq <= 0.0 {
                fallback
            } else {
                let inv = length_sq.sqrt().recip();
                [v[0] * inv, v[1] * inv, v[2] * inv]
            }
        };
        let tangent_raw = [
            (duv2[1] * dp1[0] - duv1[1] * dp2[0]) * inv_det,
            (duv2[1] * dp1[1] - duv1[1] * dp2[1]) * inv_det,
            (duv2[1] * dp1[2] - duv1[1] * dp2[2]) * inv_det,
        ];
        let bitangent_raw = [
            (-duv2[0] * dp1[0] + duv1[0] * dp2[0]) * inv_det,
            (-duv2[0] * dp1[1] + duv1[0] * dp2[1]) * inv_det,
            (-duv2[0] * dp1[2] + duv1[0] * dp2[2]) * inv_det,
        ];
        (
            normalize_or_world(tangent_raw, [1.0, 0.0, 0.0]),
            normalize_or_world(bitangent_raw, [0.0, 0.0, 1.0]),
            det,
        )
    }

    #[test]
    fn g3a_terrain_tbn_finite_and_unit_length() {
        // Every interior vertex of a rolling chunk must yield a finite,
        // unit-length tangent/bitangent pair under the derivative
        // reconstruction used by fs_terrain. Note that for a sloped height
        // field dP/du and dP/dv are not mutually orthogonal (that only holds
        // on the flat plane), so no orthonormality is asserted here — the
        // invariant is finiteness, unit length and the +X tracking of the
        // tangent.
        let terrain = generate_rolling_terrain(32, 32, 2.0, 0.0, 2.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material, [0.0; 2]);

        let vertex_at = |x: u32, z: u32| &chunk.vertices[(z * 33 + x) as usize];
        for z in 1..32 {
            for x in 1..32 {
                let p = vertex_at(x, z).position;
                let p_x = vertex_at(x + 1, z).position;
                let p_z = vertex_at(x, z + 1).position;
                let uv = vertex_at(x, z).uv;
                let uv_x = vertex_at(x + 1, z).uv;
                let uv_z = vertex_at(x, z + 1).uv;

                let dp1 = [p_x[0] - p[0], p_x[1] - p[1], p_x[2] - p[2]];
                let dp2 = [p_z[0] - p[0], p_z[1] - p[1], p_z[2] - p[2]];
                let duv1 = [uv_x[0] - uv[0], uv_x[1] - uv[1]];
                let duv2 = [uv_z[0] - uv[0], uv_z[1] - uv[1]];

                let (tangent, bitangent, det) = derivative_tbn(dp1, dp2, duv1, duv2);
                assert!(
                    det.is_finite() && det > 0.0,
                    "det must be positive, got {det}"
                );
                for axis in [tangent, bitangent] {
                    assert!(
                        axis.iter().all(|c| c.is_finite()),
                        "non-finite TBN axis at ({x}, {z})"
                    );
                    let length = (axis[0].powi(2) + axis[1].powi(2) + axis[2].powi(2)).sqrt();
                    assert!(
                        (length - 1.0).abs() < 1e-4,
                        "TBN axis not unit length at ({x}, {z}): {length}"
                    );
                }
                // Tangent follows the +X grid edge (uv.x = x / scale); the
                // rolling amplitude tilts it up to ~0.8 forward, so the
                // hemisphere check stays sober.
                assert!(
                    tangent[0] > 0.75 && tangent[1].abs() < 0.8,
                    "tangent must track +X at ({x}, {z}): {tangent:?}"
                );
            }
        }
    }

    #[test]
    fn g3a_terrain_tbn_is_exact_on_flat_terrain() {
        // On the flat plane dP/du = (spacing, 0, 0)/du and dP/dv normalize to
        // exactly [1, 0, 0] and [0, 0, 1]: unit length, orthogonal.
        let terrain = generate_flat_terrain(8, 8, 2.0, -1.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 8, &material, [0.0; 2]);

        let vertex_at = |x: u32, z: u32| &chunk.vertices[(z * 9 + x) as usize];
        for z in 0..8 {
            for x in 0..8 {
                let p = vertex_at(x, z).position;
                let p_x = vertex_at(x + 1, z).position;
                let p_z = vertex_at(x, z + 1).position;
                let uv = vertex_at(x, z).uv;
                let uv_x = vertex_at(x + 1, z).uv;
                let uv_z = vertex_at(x, z + 1).uv;

                let (tangent, bitangent, det) = derivative_tbn(
                    [p_x[0] - p[0], p_x[1] - p[1], p_x[2] - p[2]],
                    [p_z[0] - p[0], p_z[1] - p[1], p_z[2] - p[2]],
                    [uv_x[0] - uv[0], uv_x[1] - uv[1]],
                    [uv_z[0] - uv[0], uv_z[1] - uv[1]],
                );
                assert!(det > 0.0);
                assert_eq!(tangent, [1.0, 0.0, 0.0]);
                assert_eq!(bitangent, [0.0, 0.0, 1.0]);
            }
        }
    }

    #[test]
    fn g3a_terrain_tbn_degenerate_fallback_is_finite() {
        // A degenerate basis (det ~ 0, e.g. a zero-area fragment) must fall
        // back to the world-aligned frame instead of producing NaN.
        let (tangent, bitangent, det) =
            derivative_tbn([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0], [0.0, 0.0]);
        assert_eq!(det, 0.0);
        assert_eq!(tangent, [1.0, 0.0, 0.0]);
        assert_eq!(bitangent, [0.0, 0.0, 1.0]);
    }
}
