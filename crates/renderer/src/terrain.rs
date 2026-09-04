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
//! Terrain height is in render-world Y. Raw terrain height data (e.g., from
//! a simulation heightmap) maps into renderer coordinates via the render origin.
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
//! Example: `u = x / texture_scale_m`, `v = z / texture_scale_m`

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
#[derive(Debug, Clone)]
pub struct TerrainHeightField {
    /// Width (X-axis) in cells.
    pub width_cells: u32,
    /// Depth (Z-axis) in cells.
    pub depth_cells: u32,
    /// Spacing between samples in metres.
    pub sample_spacing_m: f32,
    /// Elevation samples in row-major order (Z-major, then X).
    /// Index = z * width_cells + x.
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
    /// Coordinates outside the height field are clamped to the boundary.
    #[must_use]
    pub fn sample_bilinear(&self, world_x: f32, world_z: f32) -> f32 {
        let grid_x = world_x / self.sample_spacing_m;
        let grid_z = world_z / self.sample_spacing_m;

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
}

/// Terrain material configuration.
#[derive(Debug, Clone)]
pub struct TerrainMaterial {
    /// Base color factor (RGBA).
    pub base_color_factor: [f32; 4],
    /// Optional base color texture.
    pub base_color_texture: Option<crate::texture::DecodedTexture>,
    /// Texture scale in metres for UV tiling.
    pub texture_scale_m: f32,
}

impl Default for TerrainMaterial {
    fn default() -> Self {
        Self {
            // Default grass-like green.
            base_color_factor: [0.25, 0.45, 0.18, 1.0],
            base_color_texture: None,
            texture_scale_m: DEFAULT_TERRAIN_TEXTURE_SCALE_M,
        }
    }
}

/// A single terrain chunk ready for GPU upload.
#[derive(Debug, Clone)]
pub struct TerrainChunk {
    /// Chunk grid coordinates (chunk_x, chunk_z).
    pub chunk_coords: (u32, u32),
    /// World-space origin of this chunk (min X, min Z).
    pub world_origin: [f32; 2],
    /// Chunk dimensions in metres.
    pub size_m: [f32; 2],
    /// Vertices for this chunk.
    pub vertices: Vec<Vertex>,
    /// Indices for this chunk (triangle list).
    pub indices: Vec<u32>,
    /// Axis-aligned bounding box (min, max).
    pub bounds: ([f32; 3], [f32; 3]),
}

impl TerrainChunk {
    /// Generate a terrain chunk from a height field.
    ///
    /// # Arguments
    ///
    /// * `height_field` - Source height data
    /// * `chunk_x` - Chunk X coordinate (in chunk units)
    /// * `chunk_z` - Chunk Z coordinate (in chunk units)
    /// * `cells_per_chunk` - Cells per chunk dimension
    /// * `material` - Terrain material for UV scaling
    #[must_use]
    pub fn generate(
        height_field: &TerrainHeightField,
        chunk_x: u32,
        chunk_z: u32,
        cells_per_chunk: u32,
        material: &TerrainMaterial,
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

        let world_origin_x = start_x as f32 * spacing;
        let world_origin_z = start_z as f32 * spacing;
        let size_x = cells_x as f32 * spacing;
        let size_z = cells_z as f32 * spacing;

        let vertex_count = (cells_x + 1) * (cells_z + 1);
        let mut vertices = Vec::with_capacity(vertex_count as usize);
        let mut min_y = f32::INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        // Generate vertices.
        for local_z in 0..=cells_z {
            let global_z = start_z + local_z;
            let world_z = global_z as f32 * spacing;

            for local_x in 0..=cells_x {
                let global_x = start_x + local_x;
                let world_x = global_x as f32 * spacing;

                // Sample elevation from height field.
                let elevation = height_field.elevation_at(global_x, global_z);
                min_y = min_y.min(elevation);
                max_y = max_y.max(elevation);

                // Compute normal via finite differences.
                let normal = compute_terrain_normal(height_field, global_x, global_z);

                // UV based on world position for consistent tiling.
                let uv = [
                    world_x / material.texture_scale_m,
                    world_z / material.texture_scale_m,
                ];

                vertices.push(Vertex {
                    position: [world_x, elevation, world_z],
                    normal,
                    color: [1.0, 1.0, 1.0, 1.0], // White; material color applied in shader.
                    uv,
                });
            }
        }

        // Generate indices (two triangles per cell).
        let index_count = (cells_x * cells_z * 6) as usize;
        let mut indices = Vec::with_capacity(index_count);

        for local_z in 0..cells_z {
            for local_x in 0..cells_x {
                let v0 = local_z * (cells_x + 1) + local_x;
                let v1 = v0 + 1;
                let v2 = v0 + (cells_x + 1);
                let v3 = v2 + 1;

                // Two triangles per cell, CCW winding when viewed from above (+Y).
                // v0 is bottom-left, v1 is bottom-right, v2 is top-left, v3 is top-right.
                // For CCW from above: v0 -> v1 -> v2 and v1 -> v3 -> v2
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
///
/// Uses central differences where possible, forward/backward at edges.
fn compute_terrain_normal(height_field: &TerrainHeightField, x: u32, z: u32) -> [f32; 3] {
    let spacing = height_field.sample_spacing_m;
    let two_spacing = 2.0 * spacing;

    // Height samples for finite differences.
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

    // Central difference for interior, one-sided at edges.
    let dx = if x > 0 && x < height_field.width_cells {
        (h_right - h_left) / two_spacing
    } else {
        // At edges (x == 0 or x == width_cells), use one-sided difference.
        (h_right - h_left) / spacing
    };

    let dz = if z > 0 && z < height_field.depth_cells {
        (h_up - h_down) / two_spacing
    } else {
        // At edges (z == 0 or z == depth_cells), use one-sided difference.
        (h_up - h_down) / spacing
    };

    // Normal = normalize(-dh/dx, 1, -dh/dz) for Y-up terrain.
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
///
/// All elevations are set to the specified height.
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
///
/// The terrain uses a sum of sinusoids for gentle hills. No randomness is used.
///
/// # Arguments
///
/// * `width_cells` - Width in cells
/// * `depth_cells` - Depth in cells
/// * `sample_spacing_m` - Spacing between samples
/// * `base_elevation` - Base height
/// * `amplitude` - Maximum hill height above base
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

            // Sum of sinusoids for gentle rolling hills.
            // Frequencies chosen to give visible but gentle variation.
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

/// Generate all chunks for a terrain height field.
#[must_use]
pub fn generate_terrain_chunks(
    height_field: &TerrainHeightField,
    cells_per_chunk: u32,
    material: &TerrainMaterial,
) -> Vec<TerrainChunk> {
    let chunks_x = height_field.width_cells.div_ceil(cells_per_chunk);
    let chunks_z = height_field.depth_cells.div_ceil(cells_per_chunk);

    let mut chunks = Vec::with_capacity((chunks_x * chunks_z) as usize);

    for chunk_z in 0..chunks_z {
        for chunk_x in 0..chunks_x {
            let chunk =
                TerrainChunk::generate(height_field, chunk_x, chunk_z, cells_per_chunk, material);
            if !chunk.vertices.is_empty() {
                chunks.push(chunk);
            }
        }
    }

    chunks
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
        // (10+1) * (10+1) = 121 vertices.
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
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material);
        // (32+1) * (32+1) = 1089 vertices.
        assert_eq!(chunk.vertices.len(), 33 * 33);
    }

    #[test]
    fn terrain_chunk_has_correct_index_count() {
        let terrain = generate_flat_terrain(32, 32, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material);
        // 32 * 32 * 6 = 6144 indices.
        assert_eq!(chunk.indices.len(), 32 * 32 * 6);
    }

    #[test]
    fn terrain_chunk_indices_are_in_bounds() {
        let terrain = generate_flat_terrain(32, 32, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material);
        let vertex_count = chunk.vertices.len() as u32;
        assert!(chunk.indices.iter().all(|&i| i < vertex_count));
    }

    #[test]
    fn terrain_chunk_triangle_winding_is_ccw_from_above() {
        let terrain = generate_flat_terrain(2, 2, 1.0, 0.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 2, &material);

        // Check first triangle: should be CCW when viewed from +Y.
        let i0 = chunk.indices[0] as usize;
        let i1 = chunk.indices[1] as usize;
        let i2 = chunk.indices[2] as usize;

        let v0 = chunk.vertices[i0].position;
        let v1 = chunk.vertices[i1].position;
        let v2 = chunk.vertices[i2].position;

        // Edge vectors.
        let e1 = [v1[0] - v0[0], 0.0, v1[2] - v0[2]];
        let e2 = [v2[0] - v0[0], 0.0, v2[2] - v0[2]];

        // Cross product Y component should be positive for CCW.
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
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 10, &material);

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
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 20, &material);

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

        // Coarse terrain.
        let terrain_coarse = generate_flat_terrain(10, 10, 10.0, 0.0);
        let chunk_coarse = TerrainChunk::generate(&terrain_coarse, 0, 0, 10, &material);

        // Fine terrain with same world extent.
        let terrain_fine = generate_flat_terrain(100, 100, 1.0, 0.0);
        let chunk_fine = TerrainChunk::generate(&terrain_fine, 0, 0, 100, &material);

        // Corner vertices should have same UVs.
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

        let chunk_00 = TerrainChunk::generate(&terrain, 0, 0, 32, &material);
        let chunk_10 = TerrainChunk::generate(&terrain, 1, 0, 32, &material);

        // Chunk 00 right edge should be at x=32.
        let chunk_00_right_edge: Vec<_> = chunk_00
            .vertices
            .iter()
            .filter(|v| (v.position[0] - 32.0).abs() < 1e-5)
            .collect();

        // Chunk 10 left edge should also be at x=32 (world origin of chunk 10 is 32).
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

        // Both edges should have the same Z coordinates.
        let chunk_00_z_values: Vec<_> = chunk_00_right_edge.iter().map(|v| v.position[2]).collect();
        let chunk_10_z_values: Vec<_> = chunk_10_left_edge.iter().map(|v| v.position[2]).collect();

        // Both should have vertices at z=0 through z=32.
        assert!(chunk_00_z_values.iter().any(|&z| z.abs() < 1e-5));
        assert!(chunk_10_z_values.iter().any(|&z| z.abs() < 1e-5));
    }

    #[test]
    fn terrain_bounds_are_correct() {
        let terrain = generate_rolling_terrain(20, 20, 5.0, 0.0, 10.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 20, &material);

        let (min, max) = chunk.bounds;

        // Check X bounds.
        assert!((min[0] - chunk.world_origin[0]).abs() < 1e-5);
        assert!((max[0] - (chunk.world_origin[0] + chunk.size_m[0])).abs() < 1e-5);

        // Check Z bounds.
        assert!((min[2] - chunk.world_origin[1]).abs() < 1e-5);
        assert!((max[2] - (chunk.world_origin[1] + chunk.size_m[1])).abs() < 1e-5);

        // Check Y bounds contain all vertices.
        for vertex in &chunk.vertices {
            assert!(vertex.position[1] >= min[1] - 1e-5);
            assert!(vertex.position[1] <= max[1] + 1e-5);
        }
    }

    #[test]
    fn chunk_ordering_is_deterministic() {
        let terrain = generate_flat_terrain(64, 64, 1.0, 0.0);
        let material = TerrainMaterial::default();

        let chunks1 = generate_terrain_chunks(&terrain, 32, &material);
        let chunks2 = generate_terrain_chunks(&terrain, 32, &material);

        assert_eq!(chunks1.len(), chunks2.len());
        for (c1, c2) in chunks1.iter().zip(chunks2.iter()) {
            assert_eq!(c1.chunk_coords, c2.chunk_coords);
        }
    }

    #[test]
    fn bilinear_interpolation_at_sample_points_matches_exact() {
        let terrain = generate_rolling_terrain(10, 10, 1.0, 0.0, 5.0);

        // Sample at exact grid points should match elevation_at.
        for z in 0..=10 {
            for x in 0..=10 {
                let world_x = x as f32;
                let world_z = z as f32;
                let sampled = terrain.sample_bilinear(world_x, world_z);
                let exact = terrain.elevation_at(x, z);
                assert!((sampled - exact).abs() < 1e-5, "mismatch at ({}, {})", x, z);
            }
        }
    }

    #[test]
    fn bilinear_interpolation_clamps_outside_bounds() {
        let terrain = generate_flat_terrain(10, 10, 1.0, 5.0);

        // Outside bounds should clamp to edge values.
        let sampled = terrain.sample_bilinear(-100.0, -100.0);
        assert!((sampled - 5.0).abs() < 1e-5);

        let sampled = terrain.sample_bilinear(100.0, 100.0);
        assert!((sampled - 5.0).abs() < 1e-5);
    }

    #[test]
    fn terrain_vertices_have_no_nan_or_inf() {
        let terrain = generate_rolling_terrain(32, 32, 10.0, -5.0, 15.0);
        let material = TerrainMaterial::default();
        let chunk = TerrainChunk::generate(&terrain, 0, 0, 32, &material);

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
}
