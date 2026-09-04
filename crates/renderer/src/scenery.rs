//! G2A: Flying-field scenery foundation.
//!
//! # Architecture
//!
//! Static scenery only — no ECS, no dynamic objects, no scene graph.
//! Each scenery slice is generated deterministically at initialization time
//! and uploaded to the GPU as a single merged mesh. No per-frame allocation,
//! mesh generation, or GPU resource creation.
//!
//! # Coordinate Convention
//!
//! Scenery uses render-space coordinates consistent with the rest of the
//! renderer:
//! - +Y = up (elevation)
//! - XZ = horizontal plane
//!
//! The flying field is centered around the render origin. The runway runs
//! along the X axis so aircraft taking off in the +X direction align with
//! the existing simulation forward axis.
//!
//! # Ground Height
//!
//! All ground-level scenery (runway, grass field) is placed at
//! `ground_y_render_m` which matches the existing ground-plane reference
//! used by the ground-demo architecture.

use crate::mesh::{SAFE_NORMAL, SAFE_UV, Vertex};

// ── Constants ──────────────────────────────────────────────────────────────

/// Default ground Y in render space (matches `DEFAULT_GROUND_Y_RENDER_M`).
pub const DEFAULT_GROUND_Y: f32 = -30.04;

/// Runway half-length along X (total 120 m).
pub const RUNWAY_HALF_LENGTH_M: f32 = 60.0;

/// Runway half-width along Z (total 12 m).
pub const RUNWAY_HALF_WIDTH_M: f32 = 6.0;

/// Runway safety margin beyond the strip edges.
pub const RUNWAY_SAFETY_MARGIN_M: f32 = 3.0;

/// Grass field half-extent (500 m × 500 m).
pub const FIELD_HALF_EXTENT_M: f32 = 250.0;

/// Default tree count for the flying field.
pub const DEFAULT_TREE_COUNT: usize = 50;

/// Default tree seed for deterministic placement.
pub const DEFAULT_TREE_SEED: u64 = 42;

/// Minimum distance from runway safety rectangle to any tree centre.
pub const TREE_MIN_DISTANCE_FROM_RUNWAY_M: f32 = 20.0;

// ── Types ──────────────────────────────────────────────────────────────────

/// Visual classification for a scenery object.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SceneryVisualKind {
    Grass,
    Runway,
    TreeTrunk,
    TreeCanopy,
    Pole,
    Marker,
    Fence,
    Windsock,
}

/// A named scenery configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneryPreset {
    None,
    FlyingField,
}

/// Parameters controlling flying-field generation.
#[derive(Debug, Clone)]
pub struct FlyingFieldParams {
    pub ground_y: f32,
    pub tree_seed: u64,
    pub tree_count: usize,
}

impl Default for FlyingFieldParams {
    fn default() -> Self {
        Self {
            ground_y: DEFAULT_GROUND_Y,
            tree_seed: DEFAULT_TREE_SEED,
            tree_count: DEFAULT_TREE_COUNT,
        }
    }
}

/// A placed scenery object with its transform and visual type.
#[derive(Debug, Clone)]
pub struct SceneryObject {
    pub kind: SceneryVisualKind,
    pub position: [f32; 3],
    pub rotation_yaw_rad: f32,
    pub scale: f32,
}

/// Merged scenery mesh ready for GPU upload.
///
/// All geometry (field, runway, objects) is merged into a single vertex/index
/// buffer pair so the entire scenery is one draw call.
#[derive(Debug, Clone)]
pub struct SceneryMesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
}

/// Complete generated scenery scene.
///
/// Contains the merged GPU-ready mesh and the list of placed objects
/// (retained for testing and debugging).
#[derive(Debug, Clone)]
pub struct SceneryScene {
    pub mesh: SceneryMesh,
    pub objects: Vec<SceneryObject>,
}

// ── Generation entry point ─────────────────────────────────────────────────

/// Generate the flying-field scenery scene.
#[must_use]
pub fn generate_flying_field(params: &FlyingFieldParams) -> SceneryScene {
    let mut all_vertices = Vec::new();
    let mut all_indices = Vec::new();
    let mut objects = Vec::new();

    // Grass field.
    let grass = generate_grass_field(params.ground_y);
    merge_mesh(
        &mut all_vertices,
        &mut all_indices,
        &grass.vertices,
        &grass.indices,
    );

    // Runway.
    let runway = generate_runway(params.ground_y);
    merge_mesh(
        &mut all_vertices,
        &mut all_indices,
        &runway.vertices,
        &runway.indices,
    );

    // Trees.
    let tree_positions = deterministic_tree_positions(
        params.tree_seed,
        params.tree_count,
        FIELD_HALF_EXTENT_M,
        runway_safety_rect(),
        TREE_MIN_DISTANCE_FROM_RUNWAY_M,
    );
    for &[x, z] in &tree_positions {
        let tree = generate_tree(x, params.ground_y, z);
        merge_mesh(
            &mut all_vertices,
            &mut all_indices,
            &tree.vertices,
            &tree.indices,
        );
        objects.push(SceneryObject {
            kind: SceneryVisualKind::TreeTrunk,
            position: [x, params.ground_y, z],
            rotation_yaw_rad: 0.0,
            scale: 1.0,
        });
    }

    // Marker poles along runway edges.
    for i in 0..6_u32 {
        let x = -RUNWAY_HALF_LENGTH_M + (i as f32 + 0.5) * (RUNWAY_HALF_LENGTH_M * 2.0 / 6.0);
        for &z_sign in &[-1.0_f32, 1.0] {
            let z = z_sign * (RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M);
            let pole = generate_marker_pole(x, params.ground_y, z, 2.0);
            merge_mesh(
                &mut all_vertices,
                &mut all_indices,
                &pole.vertices,
                &pole.indices,
            );
            objects.push(SceneryObject {
                kind: SceneryVisualKind::Pole,
                position: [x, params.ground_y, z],
                rotation_yaw_rad: 0.0,
                scale: 1.0,
            });
        }
    }

    // Windsock pole at runway threshold.
    let windsock = generate_windsock(RUNWAY_HALF_LENGTH_M + 10.0, params.ground_y, -15.0);
    merge_mesh(
        &mut all_vertices,
        &mut all_indices,
        &windsock.vertices,
        &windsock.indices,
    );
    objects.push(SceneryObject {
        kind: SceneryVisualKind::Windsock,
        position: [RUNWAY_HALF_LENGTH_M + 10.0, params.ground_y, -15.0],
        rotation_yaw_rad: 0.0,
        scale: 1.0,
    });

    SceneryScene {
        mesh: SceneryMesh {
            vertices: all_vertices,
            indices: all_indices,
        },
        objects,
    }
}

// ── Runway safety rectangle ────────────────────────────────────────────────

/// Runway safety rectangle as `[min_x, min_z, max_x, max_z]`.
#[must_use]
pub fn runway_safety_rect() -> [f32; 4] {
    [
        -(RUNWAY_HALF_LENGTH_M + RUNWAY_SAFETY_MARGIN_M),
        -(RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M),
        RUNWAY_HALF_LENGTH_M + RUNWAY_SAFETY_MARGIN_M,
        RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M,
    ]
}

// ── Grass field ────────────────────────────────────────────────────────────

#[must_use]
fn generate_grass_field(ground_y: f32) -> SceneryMesh {
    let extent = FIELD_HALF_EXTENT_M;
    let base_color = [0.22, 0.42, 0.16, 1.0];
    let alt_color = [0.26, 0.48, 0.20, 1.0];

    let v0 = Vertex {
        position: [-extent, ground_y, -extent],
        normal: SAFE_NORMAL,
        color: base_color,
        uv: SAFE_UV,
    };
    let v1 = Vertex {
        position: [extent, ground_y, -extent],
        normal: SAFE_NORMAL,
        color: alt_color,
        uv: SAFE_UV,
    };
    let v2 = Vertex {
        position: [extent, ground_y, extent],
        normal: SAFE_NORMAL,
        color: base_color,
        uv: SAFE_UV,
    };
    let v3 = Vertex {
        position: [-extent, ground_y, extent],
        normal: SAFE_NORMAL,
        color: alt_color,
        uv: SAFE_UV,
    };

    SceneryMesh {
        vertices: vec![v0, v1, v2, v3],
        indices: vec![0, 1, 2, 0, 2, 3],
    }
}

// ── Runway ─────────────────────────────────────────────────────────────────

#[must_use]
fn generate_runway(ground_y: f32) -> SceneryMesh {
    let lx = RUNWAY_HALF_LENGTH_M;
    let lz = RUNWAY_HALF_WIDTH_M;
    let y = ground_y + 0.02;
    let runway_color = [0.35, 0.33, 0.30, 1.0];
    let edge_color = [0.45, 0.42, 0.38, 1.0];
    let center_color = [0.50, 0.48, 0.44, 1.0];

    let mut vertices = Vec::with_capacity(8);
    let mut indices = Vec::with_capacity(18);

    // Main runway surface (slightly darker).
    vertices.push(Vertex {
        position: [-lx, y, -lz],
        normal: SAFE_NORMAL,
        color: runway_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [lx, y, -lz],
        normal: SAFE_NORMAL,
        color: runway_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [lx, y, lz],
        normal: SAFE_NORMAL,
        color: runway_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [-lx, y, lz],
        normal: SAFE_NORMAL,
        color: runway_color,
        uv: SAFE_UV,
    });
    indices.extend_from_slice(&[0, 1, 2, 0, 2, 3]);

    // Centre-line strip (lighter, narrower).
    let clz = 0.3;
    let base = vertices.len() as u32;
    vertices.push(Vertex {
        position: [-lx, y + 0.005, -clz],
        normal: SAFE_NORMAL,
        color: center_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [lx, y + 0.005, -clz],
        normal: SAFE_NORMAL,
        color: center_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [lx, y + 0.005, clz],
        normal: SAFE_NORMAL,
        color: center_color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: [-lx, y + 0.005, clz],
        normal: SAFE_NORMAL,
        color: center_color,
        uv: SAFE_UV,
    });
    indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);

    // Edge stripes.
    let ew = 0.4;
    for &z_sign in &[-1.0_f32, 1.0] {
        let z_outer = z_sign * lz;
        let z_inner = z_sign * (lz - ew);
        let base = vertices.len() as u32;
        vertices.push(Vertex {
            position: [-lx, y + 0.005, z_outer],
            normal: SAFE_NORMAL,
            color: edge_color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [lx, y + 0.005, z_outer],
            normal: SAFE_NORMAL,
            color: edge_color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [lx, y + 0.005, z_inner],
            normal: SAFE_NORMAL,
            color: edge_color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [-lx, y + 0.005, z_inner],
            normal: SAFE_NORMAL,
            color: edge_color,
            uv: SAFE_UV,
        });
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    SceneryMesh { vertices, indices }
}

// ── Tree ───────────────────────────────────────────────────────────────────

#[must_use]
fn generate_tree(x: f32, ground_y: f32, z: f32) -> SceneryMesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Trunk: 0.15 m radius, 1.5 m tall.
    let trunk = generate_cylinder([x, ground_y, z], 0.15, 1.5, [0.35, 0.22, 0.10, 1.0], 6);
    merge_mesh(&mut vertices, &mut indices, &trunk.vertices, &trunk.indices);

    // Canopy: cone, 1.2 m radius, 3.0 m tall, sitting on top of trunk.
    let canopy = generate_cone([x, ground_y + 1.5, z], 1.2, 3.0, [0.15, 0.38, 0.12, 1.0], 8);
    merge_mesh(
        &mut vertices,
        &mut indices,
        &canopy.vertices,
        &canopy.indices,
    );

    SceneryMesh { vertices, indices }
}

// ── Marker pole ────────────────────────────────────────────────────────────

#[must_use]
fn generate_marker_pole(x: f32, ground_y: f32, z: f32, height: f32) -> SceneryMesh {
    generate_cylinder([x, ground_y, z], 0.05, height, [0.85, 0.25, 0.10, 1.0], 5)
}

// ── Windsock ───────────────────────────────────────────────────────────────

#[must_use]
fn generate_windsock(x: f32, ground_y: f32, z: f32) -> SceneryMesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Pole: 5 m tall.
    let pole = generate_cylinder([x, ground_y, z], 0.06, 5.0, [0.60, 0.60, 0.60, 1.0], 5);
    merge_mesh(&mut vertices, &mut indices, &pole.vertices, &pole.indices);

    // Sock: small cone at top, pointing sideways.
    let sock = generate_cone(
        [x, ground_y + 5.0, z],
        0.25,
        1.0,
        [0.90, 0.45, 0.10, 1.0],
        6,
    );
    merge_mesh(&mut vertices, &mut indices, &sock.vertices, &sock.indices);

    SceneryMesh { vertices, indices }
}

// ── Deterministic tree placement ───────────────────────────────────────────

/// Deterministic tree placement using a simple hash-based PRNG.
///
/// Trees are placed within `[-field_half_extent, field_half_extent]` on both
/// axes, excluding the runway safety rectangle expanded by `min_distance`.
/// The same seed always produces the same layout.
#[must_use]
pub fn deterministic_tree_positions(
    seed: u64,
    count: usize,
    field_half_extent: f32,
    runway_rect: [f32; 4],
    min_distance: f32,
) -> Vec<[f32; 2]> {
    let safe_min_x = runway_rect[0] - min_distance;
    let safe_min_z = runway_rect[1] - min_distance;
    let safe_max_x = runway_rect[2] + min_distance;
    let safe_max_z = runway_rect[3] + min_distance;

    let mut positions = Vec::with_capacity(count);
    let mut state = seed.wrapping_mul(6364136223846793005).wrapping_add(1);

    while positions.len() < count {
        // Two hash calls for x and z.
        let hash_x = scramble(state);
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        let hash_z = scramble(state);
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);

        let x = lerp(-field_half_extent, field_half_extent, to_unit(hash_x));
        let z = lerp(-field_half_extent, field_half_extent, to_unit(hash_z));

        // Reject if inside expanded safety rectangle.
        if x >= safe_min_x && x <= safe_max_x && z >= safe_min_z && z <= safe_max_z {
            continue;
        }

        positions.push([x, z]);
    }

    positions
}

fn scramble(mut x: u64) -> u64 {
    x = x.wrapping_mul(0x517cc1b727220a95);
    x ^= x >> 32;
    x = x.wrapping_mul(0x6c62272e07bb0142);
    x ^= x >> 32;
    x
}

fn to_unit(hash: u64) -> f32 {
    (hash >> 11) as f32 / (1u64 << 53) as f32
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

// ── Procedural geometry helpers ────────────────────────────────────────────

fn generate_cylinder(
    base_centre: [f32; 3],
    radius: f32,
    height: f32,
    color: [f32; 4],
    segments: u32,
) -> SceneryMesh {
    let [cx, cy, cz] = base_centre;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Side vertices: bottom ring + top ring.
    for i in 0..segments {
        let angle = (i as f32 / segments as f32) * 2.0 * std::f32::consts::PI;
        let cos = angle.cos();
        let sin = angle.sin();
        let nx = cos;
        let nz = sin;

        // Bottom.
        vertices.push(Vertex {
            position: [cx + radius * cos, cy, cz + radius * sin],
            normal: [nx, 0.0, nz],
            color,
            uv: SAFE_UV,
        });
        // Top.
        vertices.push(Vertex {
            position: [cx + radius * cos, cy + height, cz + radius * sin],
            normal: [nx, 0.0, nz],
            color,
            uv: SAFE_UV,
        });
    }

    // Side indices.
    for i in 0..segments {
        let b0 = 2 * i;
        let t0 = 2 * i + 1;
        let b1 = 2 * ((i + 1) % segments);
        let t1 = 2 * ((i + 1) % segments) + 1;
        indices.extend_from_slice(&[b0, b1, t0, t0, b1, t1]);
    }

    // Top cap.
    let top_centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [cx, cy + height, cz],
        normal: [0.0, 1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[top_centre, 2 * i + 1, 2 * next + 1]);
    }

    // Bottom cap.
    let bottom_centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [cx, cy, cz],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[bottom_centre, 2 * i, 2 * next]);
    }

    SceneryMesh { vertices, indices }
}

fn generate_cone(
    base_centre: [f32; 3],
    radius: f32,
    height: f32,
    color: [f32; 4],
    segments: u32,
) -> SceneryMesh {
    let [cx, cy, cz] = base_centre;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Apex.
    let apex_idx = 0u32;
    vertices.push(Vertex {
        position: [cx, cy + height, cz],
        normal: [0.0, 1.0, 0.0],
        color,
        uv: SAFE_UV,
    });

    // Base ring.
    let ring_start = 1u32;
    for i in 0..segments {
        let angle = (i as f32 / segments as f32) * 2.0 * std::f32::consts::PI;
        let cos = angle.cos();
        let sin = angle.sin();
        // Approximate cone normal (tilted outward).
        let slope = radius / height;
        let ny = slope / (1.0 + slope * slope).sqrt();
        let nxz = 1.0 / (1.0 + slope * slope).sqrt();
        vertices.push(Vertex {
            position: [cx + radius * cos, cy, cz + radius * sin],
            normal: [nxz * cos, ny, nxz * sin],
            color,
            uv: SAFE_UV,
        });
    }

    // Side triangles.
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[apex_idx, ring_start + i, ring_start + next]);
    }

    // Base cap.
    let base_centre_idx = vertices.len() as u32;
    vertices.push(Vertex {
        position: [cx, cy, cz],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[base_centre_idx, ring_start + next, ring_start + i]);
    }

    SceneryMesh { vertices, indices }
}

// ── Mesh merging ───────────────────────────────────────────────────────────

fn merge_mesh(
    dst_vertices: &mut Vec<Vertex>,
    dst_indices: &mut Vec<u32>,
    src_vertices: &[Vertex],
    src_indices: &[u32],
) {
    let base = dst_vertices.len() as u32;
    dst_vertices.extend_from_slice(src_vertices);
    dst_indices.extend(src_indices.iter().map(|&i| i + base));
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_scene() -> SceneryScene {
        generate_flying_field(&FlyingFieldParams::default())
    }

    #[test]
    fn runway_is_centered_at_origin() {
        let scene = default_scene();
        let runway_verts: Vec<_> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == [0.35, 0.33, 0.30, 1.0])
            .collect();
        assert!(!runway_verts.is_empty(), "runway vertices must exist");

        let min_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::INFINITY, f32::min);
        let max_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max);
        let min_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::INFINITY, f32::min);
        let max_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::NEG_INFINITY, f32::max);

        let centre_x = (min_x + max_x) / 2.0;
        let centre_z = (min_z + max_z) / 2.0;
        assert!(
            centre_x.abs() < 0.1,
            "runway centre X should be ~0, got {centre_x}"
        );
        assert!(
            centre_z.abs() < 0.1,
            "runway centre Z should be ~0, got {centre_z}"
        );
    }

    #[test]
    fn runway_dimensions_match_spec() {
        let scene = default_scene();
        let runway_verts: Vec<_> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == [0.35, 0.33, 0.30, 1.0])
            .collect();

        let min_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::INFINITY, f32::min);
        let max_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max);
        let min_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::INFINITY, f32::min);
        let max_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::NEG_INFINITY, f32::max);

        let length = max_x - min_x;
        let width = max_z - min_z;
        assert!(
            (length - 120.0).abs() < 0.1,
            "runway length should be 120 m, got {length}"
        );
        assert!(
            (width - 12.0).abs() < 0.1,
            "runway width should be 12 m, got {width}"
        );
    }

    #[test]
    fn scenery_transforms_are_deterministic() {
        let params = FlyingFieldParams::default();
        let scene_a = generate_flying_field(&params);
        let scene_b = generate_flying_field(&params);

        assert_eq!(scene_a.objects.len(), scene_b.objects.len());
        for (a, b) in scene_a.objects.iter().zip(scene_b.objects.iter()) {
            assert_eq!(a.kind, b.kind);
            assert_eq!(a.position, b.position);
            assert_eq!(a.rotation_yaw_rad, b.rotation_yaw_rad);
            assert_eq!(a.scale, b.scale);
        }
    }

    #[test]
    fn vegetation_layout_is_deterministic() {
        let params = FlyingFieldParams::default();
        let scene_a = generate_flying_field(&params);
        let scene_b = generate_flying_field(&params);

        assert_eq!(scene_a.mesh.vertices.len(), scene_b.mesh.vertices.len());
        assert_eq!(scene_a.mesh.indices.len(), scene_b.mesh.indices.len());
        for (a, b) in scene_a
            .mesh
            .vertices
            .iter()
            .zip(scene_b.mesh.vertices.iter())
        {
            assert_eq!(a.position, b.position);
            assert_eq!(a.normal, b.normal);
            assert_eq!(a.color, b.color);
        }
    }

    #[test]
    fn same_seed_produces_bit_identical_placement() {
        let positions_a = deterministic_tree_positions(
            DEFAULT_TREE_SEED,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        let positions_b = deterministic_tree_positions(
            DEFAULT_TREE_SEED,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        assert_eq!(positions_a.len(), positions_b.len());
        for (a, b) in positions_a.iter().zip(positions_b.iter()) {
            assert_eq!(a[0].to_bits(), b[0].to_bits());
            assert_eq!(a[1].to_bits(), b[1].to_bits());
        }
    }

    #[test]
    fn no_tree_overlaps_runway_safety_rectangle() {
        let positions = deterministic_tree_positions(
            DEFAULT_TREE_SEED,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        let rect = runway_safety_rect();
        let safe_min_x = rect[0] - TREE_MIN_DISTANCE_FROM_RUNWAY_M;
        let safe_min_z = rect[1] - TREE_MIN_DISTANCE_FROM_RUNWAY_M;
        let safe_max_x = rect[2] + TREE_MIN_DISTANCE_FROM_RUNWAY_M;
        let safe_max_z = rect[3] + TREE_MIN_DISTANCE_FROM_RUNWAY_M;

        for &[x, z] in &positions {
            let inside = x >= safe_min_x && x <= safe_max_x && z >= safe_min_z && z <= safe_max_z;
            assert!(
                !inside,
                "tree at ({x}, {z}) is inside expanded runway safety rect"
            );
        }
    }

    #[test]
    fn all_generated_coordinates_are_finite() {
        let scene = default_scene();
        for vertex in &scene.mesh.vertices {
            assert!(
                vertex.position.iter().all(|c| c.is_finite()),
                "non-finite position: {:?}",
                vertex.position
            );
            assert!(
                vertex.normal.iter().all(|c| c.is_finite()),
                "non-finite normal: {:?}",
                vertex.normal
            );
            assert!(
                vertex.color.iter().all(|c| c.is_finite()),
                "non-finite color: {:?}",
                vertex.color
            );
        }
        for obj in &scene.objects {
            assert!(obj.position.iter().all(|c| c.is_finite()));
            assert!(obj.position[0].is_finite());
            assert!(obj.rotation_yaw_rad.is_finite());
            assert!(obj.scale.is_finite());
        }
    }

    #[test]
    fn terrain_runway_ground_height_matches_visual_ground_plane() {
        let scene = default_scene();
        // Grass field vertices should be at DEFAULT_GROUND_Y.
        let grass_verts: Vec<_> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == [0.22, 0.42, 0.16, 1.0] || v.color == [0.26, 0.48, 0.20, 1.0])
            .collect();
        assert!(!grass_verts.is_empty());
        for v in &grass_verts {
            assert!(
                (v.position[1] - DEFAULT_GROUND_Y).abs() < 0.001,
                "grass vertex Y {} != expected {}",
                v.position[1],
                DEFAULT_GROUND_Y
            );
        }
    }

    #[test]
    fn repeated_generation_produces_identical_scenery() {
        let params = FlyingFieldParams::default();
        let scene_a = generate_flying_field(&params);
        let scene_b = generate_flying_field(&params);

        assert_eq!(scene_a.mesh.vertices.len(), scene_b.mesh.vertices.len());
        assert_eq!(scene_a.mesh.indices.len(), scene_b.mesh.indices.len());

        for (a, b) in scene_a
            .mesh
            .vertices
            .iter()
            .zip(scene_b.mesh.vertices.iter())
        {
            assert_eq!(a.position, b.position);
            assert_eq!(a.normal, b.normal);
            assert_eq!(a.color, b.color);
            assert_eq!(a.uv, b.uv);
        }
        for (a, b) in scene_a.mesh.indices.iter().zip(scene_b.mesh.indices.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn all_indices_are_in_bounds() {
        let scene = default_scene();
        let vertex_count = scene.mesh.vertices.len() as u32;
        assert!(
            scene.mesh.indices.iter().all(|&i| i < vertex_count),
            "index out of bounds"
        );
    }

    #[test]
    fn mesh_has_non_zero_geometry() {
        let scene = default_scene();
        assert!(scene.mesh.vertices.len() > 100);
        assert!(scene.mesh.indices.len() > 100);
        assert!(scene.objects.len() >= DEFAULT_TREE_COUNT);
    }

    #[test]
    fn tree_positions_are_within_field_bounds() {
        let positions = deterministic_tree_positions(
            DEFAULT_TREE_SEED,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        for &[x, z] in &positions {
            assert!((-FIELD_HALF_EXTENT_M..=FIELD_HALF_EXTENT_M).contains(&x));
            assert!((-FIELD_HALF_EXTENT_M..=FIELD_HALF_EXTENT_M).contains(&z));
        }
    }

    #[test]
    fn different_seeds_produce_different_layouts() {
        let positions_a = deterministic_tree_positions(
            1,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        let positions_b = deterministic_tree_positions(
            999,
            DEFAULT_TREE_COUNT,
            FIELD_HALF_EXTENT_M,
            runway_safety_rect(),
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        // At least some positions should differ.
        let different_count = positions_a
            .iter()
            .zip(positions_b.iter())
            .filter(|(a, b)| (a[0] - b[0]).abs() > 0.01 || (a[1] - b[1]).abs() > 0.01)
            .count();
        assert!(
            different_count > DEFAULT_TREE_COUNT / 2,
            "different seeds should produce substantially different layouts"
        );
    }

    #[test]
    fn runway_surface_is_above_grass() {
        let scene = default_scene();
        let grass_y = DEFAULT_GROUND_Y;
        let runway_verts: Vec<_> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == [0.35, 0.33, 0.30, 1.0])
            .collect();
        assert!(!runway_verts.is_empty());
        for v in &runway_verts {
            assert!(v.position[1] >= grass_y, "runway vertex below grass level");
        }
    }

    #[test]
    fn all_triangles_have_ccw_winding_from_above() {
        // Convention: cross_y = e1_xz[0]*e2_xz[2] - e1_xz[2]*e2_xz[0] > 0
        // matches the terrain chunk winding (same as terrain test).
        // Only upward-facing triangles (face normal Y > 0) are checked;
        // downward-facing caps naturally have the opposite winding.
        let scene = default_scene();
        let verts = &scene.mesh.vertices;
        for tri in scene.mesh.indices.as_chunks::<3>().0 {
            let v0 = &verts[tri[0] as usize];
            let v1 = &verts[tri[1] as usize];
            let v2 = &verts[tri[2] as usize];

            let e1 = [
                v1.position[0] - v0.position[0],
                v1.position[1] - v0.position[1],
                v1.position[2] - v0.position[2],
            ];
            let e2 = [
                v2.position[0] - v0.position[0],
                v2.position[1] - v0.position[1],
                v2.position[2] - v0.position[2],
            ];

            // Face normal Y from e1 × e2.
            let face_normal_y = e1[2] * e2[0] - e1[0] * e2[2];

            // Skip downward-facing triangles.
            if face_normal_y > 0.0 {
                continue;
            }

            // 2D cross product in XZ plane (same formula as terrain test).
            let cross_y = e1[0] * e2[2] - e1[2] * e2[0];
            assert!(
                cross_y > -0.01,
                "inconsistent winding: cross_y = {cross_y}, v0 = {:?}, v1 = {:?}, v2 = {:?}",
                v0.position,
                v1.position,
                v2.position
            );
        }
    }
}
