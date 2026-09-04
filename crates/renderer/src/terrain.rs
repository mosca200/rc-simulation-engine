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
//! The terrain material base_color_factor is baked into vertex colors at chunk
//! generation time. The shader's texture * vertex_color pipeline with a white
//! fallback texture produces the correct terrain color. This avoids a separate
//! terrain shading path while keeping the material pipeline uniform.

use crate::mesh::{SAFE_NORMAL, Vertex};
use std::f32::consts::PI;

/// Default terrain texture scale in metres.
///
/// At 4.0m per tile, a 512x512 texture covers a 2km x 2km field with
/// reasonable ground detail without appearing stretched.
pub const DEFAULT_TERRAIN_TEXTURE_SCALE_M: f32 = 4.0;

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
/// The `base_color_factor` is baked into terrain vertex colors at chunk
/// generation time. The shader multiplies vertex color by the (white fallback)
/// texture, producing the correct terrain color without a separate shading path.
#[derive(Debug, Clone)]
pub struct TerrainMaterial {
    /// Base color factor (RGBA). Baked into vertex colors during chunk generation.
    pub base_color_factor: [f32; 4],
    /// Texture scale in metres for UV tiling.
    pub texture_scale_m: f32,
}

impl Default for TerrainMaterial {
    fn default() -> Self {
        Self {
            // Default grass-like green.
            base_color_factor: [0.25, 0.45, 0.18, 1.0],
            texture_scale_m: DEFAULT_TERRAIN_TEXTURE_SCALE_M,
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

                vertices.push(Vertex {
                    position: [render_x, elevation, render_z],
                    normal,
                    color: baked_color,
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

                indices.push(v0);
                indices.push(v1);
                indices.push(v2);

                indices.push(v1);
                indices.push(v3);
                indices.push(v2);
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
    fn terrain_chunk_triangle_winding_is_ccw_from_above() {
        let terrain = generate_flat_terrain(2, 2, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 2, &material, [0.0; 2]);

        let i0 = chunk.indices[0] as usize;
        let i1 = chunk.indices[1] as usize;
        let i2 = chunk.indices[2] as usize;

        let v0 = chunk.vertices[i0].position;
        let v1 = chunk.vertices[i1].position;
        let v2 = chunk.vertices[i2].position;

        let e1 = [v1[0] - v0[0], 0.0, v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], 0.0, v2[2] - v0[2]];

        let cross_y = e1[0] * e2[2] - e1[2] * e2[0];
        assert!(
            cross_y > 0.0,
            "expected CCW winding, got cross_y = {}",
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
    fn terrain_material_color_is_baked_into_vertex_colors() {
        let terrain = generate_flat_terrain(4, 4, 1.0, 0.0);
        let material = TerrainMaterial {
            base_color_factor: [0.25, 0.5, 0.75, 1.0],
            texture_scale_m: 4.0,
        };
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 4, &material, [0.0; 2]);

        for vertex in &chunk.vertices {
            assert!(
                (vertex.color[0] - 0.25).abs() < 1e-6,
                "red channel mismatch: {}",
                vertex.color[0]
            );
            assert!(
                (vertex.color[1] - 0.5).abs() < 1e-6,
                "green channel mismatch: {}",
                vertex.color[1]
            );
            assert!(
                (vertex.color[2] - 0.75).abs() < 1e-6,
                "blue channel mismatch: {}",
                vertex.color[2]
            );
            assert!(
                (vertex.color[3] - 1.0).abs() < 1e-6,
                "alpha channel mismatch: {}",
                vertex.color[3]
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
}
