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
//! along the Z axis, aligned with NED North (render −Z) so that aircraft
//! taking off in the −Z direction align with the identity-aircraft forward
//! axis per the approved NED-to-render mapping.
//!
//! # Ground Height
//!
//! All ground-level scenery (runway, trees, poles) is placed at
//! `ground_y_render_m` which matches the existing ground-plane reference.
//! For the FlyingField preset the visual terrain itself is flat at this Y,
//! so no redundant coplanar grass plane is generated.
//!
//! # G2C: Flying-field presentation pass
//!
//! The FlyingField preset is enriched with presentation-only geometry,
//! still generated once at initialization and merged into the single
//! [`SceneryMesh`]:
//! - segmented runway centreline, edge lines, and threshold bars (vertex
//!   colors, no textures);
//! - an RC flightline: one safety fence run and four pilot station markers
//!   beside the runway, outside the runway safety rectangle;
//! - a recognizable windsock (pole, top boom, striped horizontal sock);
//! - deterministic per-tree variation (height, canopy radius, yaw/lean,
//!   silhouette, canopy green) derived from the tree seed and index.

use crate::mesh::{SAFE_NORMAL, SAFE_UV, Vertex};

// ── Constants ──────────────────────────────────────────────────────────────

/// Default ground Y in render space (matches `DEFAULT_GROUND_Y_RENDER_M`).
pub const DEFAULT_GROUND_Y: f32 = -30.04;

/// Runway half-length along Z (total 120 m, long axis parallel to NED North).
pub const RUNWAY_HALF_LENGTH_M: f32 = 60.0;

/// Runway half-width along X (total 12 m).
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

/// Explicit upper bound on merged flying-field geometry in triangles.
///
/// Generation must stay well under this ceiling; a test enforces it. Keeps
/// the single scenery draw call cheap regardless of future presentation
/// additions.
pub const MAX_FLYING_FIELD_TRIANGLES: u32 = 8_000;

/// Vertical offset of runway markings above the surface (z-fighting guard).
const RUNWAY_MARKING_OFFSET_M: f32 = 0.005;

/// Centreline dash length and gap along Z; 10 dashes in total.
const CENTERLINE_DASH_LENGTH_M: f32 = 6.0;
const CENTERLINE_DASH_GAP_M: f32 = 6.0;
const CENTERLINE_DASH_COUNT: u32 = 10;

/// Centreline dash half-width along X.
const CENTERLINE_DASH_HALF_WIDTH_M: f32 = 0.3;

/// Edge marking width along X (solid line inside each runway edge).
const EDGE_MARKING_WIDTH_M: f32 = 0.4;

/// Threshold marking depth along Z (full-width bar between the edge lines).
const THRESHOLD_DEPTH_M: f32 = 1.5;

/// Flightline: fence offset from runway centre, outside the safety rectangle.
const FLIGHTLINE_X_M: f32 = 12.0;

/// Fence run half-length along Z (centred on the runway origin).
const FLIGHTLINE_HALF_LENGTH_M: f32 = 50.0;

/// Fence post spacing and height.
const FENCE_POST_SPACING_M: f32 = 10.0;
const FENCE_HEIGHT_M: f32 = 1.2;

/// Pilot station offset from the runway centre (behind the fence) and its z.
const PILOT_MARKER_X_M: f32 = 14.0;
const PILOT_MARKER_Z_M: [f32; 4] = [-30.0, -10.0, 10.0, 30.0];

/// Windsock pole height and sock length.
const WINDSOCK_POLE_HEIGHT_M: f32 = 6.0;
const WINDSOCK_SOCK_LENGTH_M: f32 = 2.2;

/// Presentation colors. Markings are high-contrast but never emissive.
const MARKING_CENTER: [f32; 4] = [0.90, 0.90, 0.88, 1.0];
const MARKING_EDGE: [f32; 4] = [0.88, 0.88, 0.86, 1.0];
const MARKING_THRESHOLD: [f32; 4] = [0.92, 0.90, 0.88, 1.0];
const FENCE_WHITE: [f32; 4] = [0.86, 0.86, 0.84, 1.0];
const PILOT_ORANGE: [f32; 4] = [0.85, 0.25, 0.10, 1.0];
const WINDSOCK_ORANGE: [f32; 4] = [0.92, 0.44, 0.08, 1.0];
const WINDSOCK_WHITE: [f32; 4] = [0.94, 0.94, 0.92, 1.0];

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

impl SceneryMesh {
    /// Number of triangles in the merged mesh (`indices.len() / 3`).
    #[must_use]
    pub const fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
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

    // Grass field: for FlyingField the visual terrain is flat at ground_y
    // (option A), so no redundant coplanar grass plane is generated here.

    // Runway.
    let runway = generate_runway(params.ground_y);
    merge_mesh(
        &mut all_vertices,
        &mut all_indices,
        &runway.vertices,
        &runway.indices,
    );

    // Trees (deterministic placement + deterministic per-tree variation).
    let tree_positions = deterministic_tree_positions(
        params.tree_seed,
        params.tree_count,
        FIELD_HALF_EXTENT_M,
        runway_safety_rect(),
        TREE_MIN_DISTANCE_FROM_RUNWAY_M,
    );
    for (index, &[x, z]) in tree_positions.iter().enumerate() {
        let variant = deterministic_tree_variant(params.tree_seed, index);
        let tree = generate_tree(x, params.ground_y, z, &variant);
        merge_mesh(
            &mut all_vertices,
            &mut all_indices,
            &tree.vertices,
            &tree.indices,
        );
        objects.push(SceneryObject {
            kind: SceneryVisualKind::TreeTrunk,
            position: [x, params.ground_y, z],
            rotation_yaw_rad: variant.yaw_rad,
            scale: variant.height_scale,
        });
    }

    // Marker poles along runway edges (iterating along Z, placed at ±X).
    for i in 0..6_u32 {
        let z = -RUNWAY_HALF_LENGTH_M + (i as f32 + 0.5) * (RUNWAY_HALF_LENGTH_M * 2.0 / 6.0);
        for &x_sign in &[-1.0_f32, 1.0] {
            let x = x_sign * (RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M);
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

    // RC flightline: safety fence run and four pilot stations, both beside
    // the runway and clear of the runway safety rectangle (fence at +12 m,
    // stations behind it at +14 m). Presentation geometry only.
    append_fence_run(
        &mut all_vertices,
        &mut all_indices,
        FLIGHTLINE_X_M,
        params.ground_y,
        FLIGHTLINE_HALF_LENGTH_M,
        FENCE_HEIGHT_M,
        FENCE_WHITE,
    );
    objects.push(SceneryObject {
        kind: SceneryVisualKind::Fence,
        position: [FLIGHTLINE_X_M, params.ground_y, 0.0],
        rotation_yaw_rad: 0.0,
        scale: 1.0,
    });
    for &z in &PILOT_MARKER_Z_M {
        generate_pilot_marker(
            &mut all_vertices,
            &mut all_indices,
            PILOT_MARKER_X_M,
            params.ground_y,
            z,
            PILOT_ORANGE,
        );
        objects.push(SceneryObject {
            kind: SceneryVisualKind::Marker,
            position: [PILOT_MARKER_X_M, params.ground_y, z],
            rotation_yaw_rad: 0.0,
            scale: 1.0,
        });
    }

    // Windsock pole at runway threshold (+Z end).
    let windsock = generate_windsock(15.0, params.ground_y, RUNWAY_HALF_LENGTH_M + 10.0);
    merge_mesh(
        &mut all_vertices,
        &mut all_indices,
        &windsock.vertices,
        &windsock.indices,
    );
    objects.push(SceneryObject {
        kind: SceneryVisualKind::Windsock,
        position: [15.0, params.ground_y, RUNWAY_HALF_LENGTH_M + 10.0],
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

/// Runway safety rectangle as `[min_x, min_z, max_x, max_z]` (long axis along Z).
#[must_use]
pub fn runway_safety_rect() -> [f32; 4] {
    [
        -(RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M),
        -(RUNWAY_HALF_LENGTH_M + RUNWAY_SAFETY_MARGIN_M),
        RUNWAY_HALF_WIDTH_M + RUNWAY_SAFETY_MARGIN_M,
        RUNWAY_HALF_LENGTH_M + RUNWAY_SAFETY_MARGIN_M,
    ]
}

// ── Runway ─────────────────────────────────────────────────────────────────

#[must_use]
fn generate_runway(ground_y: f32) -> SceneryMesh {
    // Long axis along Z (NED North), short axis along X.
    let lz = RUNWAY_HALF_LENGTH_M;
    let lx = RUNWAY_HALF_WIDTH_M;
    let surface_y = ground_y + 0.02;
    let marking_y = surface_y + RUNWAY_MARKING_OFFSET_M;
    let runway_color = [0.35, 0.33, 0.30, 1.0];

    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Main runway surface (readable asphalt, vertex colors only).
    append_runway_marking(
        &mut vertices,
        &mut indices,
        [-lx, lx],
        [-lz, lz],
        surface_y,
        runway_color,
    );

    // Edge markings: solid high-contrast lines just inside both edges.
    for &x_sign in &[-1.0_f32, 1.0] {
        let outer = x_sign * lx;
        let inner = x_sign * (lx - EDGE_MARKING_WIDTH_M);
        append_runway_marking(
            &mut vertices,
            &mut indices,
            [outer.min(inner), outer.max(inner)],
            [-lz, lz],
            marking_y,
            MARKING_EDGE,
        );
    }

    // Threshold markings: full-width bars at both ends, inside the edge lines.
    for &z_sign in &[-1.0_f32, 1.0] {
        let outer = z_sign * lz;
        let inner = z_sign * (lz - THRESHOLD_DEPTH_M);
        append_runway_marking(
            &mut vertices,
            &mut indices,
            [-(lx - EDGE_MARKING_WIDTH_M), lx - EDGE_MARKING_WIDTH_M],
            [outer.min(inner), outer.max(inner)],
            marking_y,
            MARKING_THRESHOLD,
        );
    }

    // Segmented centreline: 10 dashes between the threshold bars, symmetric
    // about the runway origin.
    let dash_period = CENTERLINE_DASH_LENGTH_M + CENTERLINE_DASH_GAP_M;
    let dash_span = CENTERLINE_DASH_COUNT as f32 * dash_period;
    let first_dash_z = -(dash_span * 0.5 - CENTERLINE_DASH_GAP_M * 0.5);
    for dash in 0..CENTERLINE_DASH_COUNT {
        let z_lo = first_dash_z + dash as f32 * dash_period;
        append_runway_marking(
            &mut vertices,
            &mut indices,
            [-CENTERLINE_DASH_HALF_WIDTH_M, CENTERLINE_DASH_HALF_WIDTH_M],
            [z_lo, z_lo + CENTERLINE_DASH_LENGTH_M],
            marking_y,
            MARKING_CENTER,
        );
    }

    SceneryMesh { vertices, indices }
}

// ── Tree ───────────────────────────────────────────────────────────────────

/// Deterministic per-tree variation, a pure function of `(seed, index)`.
///
/// The same seed and index always produce the same layout and the same
/// geometry; no runtime RNG is involved.
struct TreeVariant {
    height_scale: f32,
    canopy_radius: f32,
    yaw_rad: f32,
    canopy_color: [f32; 4],
    rounded: bool,
}

/// Derive one tree variant from the placement seed and tree index.
#[must_use]
fn deterministic_tree_variant(seed: u64, index: usize) -> TreeVariant {
    let mut state = scramble(
        seed.wrapping_mul(0x9e37_79b9_7f4a_7c15)
            .wrapping_add(index as u64 + 1),
    );
    let mut unit = move || {
        let value = to_unit(scramble(state));
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        value
    };

    let height_unit = unit();
    let radius_unit = unit();
    let yaw_unit = unit();
    let rounded_unit = unit();
    let green_unit = unit();
    let tint_unit = unit();

    TreeVariant {
        // Full tree height scales ~0.8..1.25 (keeps canopies out of
        // "giant" and "tiny" territory).
        height_scale: 0.80 + 0.45 * height_unit,
        // Canopy radius scales ~0.85..1.30 around the 1.2 m base.
        canopy_radius: 0.85 + 0.45 * radius_unit,
        yaw_rad: -0.40 + 0.80 * yaw_unit,
        rounded: rounded_unit < 0.50,
        // Natural dark-green spread via independent channel factors.
        canopy_color: [
            0.15 * (0.80 + 0.35 * green_unit),
            0.38 * (0.85 + 0.30 * radius_unit),
            0.12 * (0.85 + 0.30 * tint_unit),
            1.0,
        ],
    }
}

#[must_use]
fn generate_tree(x: f32, ground_y: f32, z: f32, variant: &TreeVariant) -> SceneryMesh {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    // Trunk: 0.15 m radius, scaled height.
    let trunk_height = 1.5 * variant.height_scale;
    let trunk = generate_cylinder(
        [x, ground_y, z],
        0.15,
        trunk_height,
        [0.35, 0.22, 0.10, 1.0],
        6,
    );
    merge_mesh(&mut vertices, &mut indices, &trunk.vertices, &trunk.indices);

    // Canopy on top of the trunk; yaw tilts the apex for a subtle lean.
    let canopy_radius = 1.2 * variant.canopy_radius;
    let canopy_height = if variant.rounded {
        2.0 * variant.height_scale
    } else {
        3.0 * variant.height_scale
    };
    let canopy = generate_canopy(
        [x, ground_y + trunk_height, z],
        canopy_radius,
        canopy_height,
        variant.canopy_color,
        variant.yaw_rad,
        variant.rounded,
    );
    merge_mesh(
        &mut vertices,
        &mut indices,
        &canopy.vertices,
        &canopy.indices,
    );

    SceneryMesh { vertices, indices }
}

/// One horizontal ring of `segments` vertices around the Y axis.
fn canopy_ring(x: f32, y: f32, z: f32, radius: f32, segments: u32) -> Vec<[f32; 3]> {
    (0..segments)
        .map(|i| {
            let angle = (i as f32 / segments as f32) * 2.0 * std::f32::consts::PI;
            [x + radius * angle.cos(), y, z + radius * angle.sin()]
        })
        .collect()
}

/// Generate a canopy silhouette: a pointed cone (conifer) or a rounded
/// two-ring dome. The apex leans by `yaw_rad` so the tree is not a perfect
/// rotationally symmetric copy of its neighbours.
#[must_use]
fn generate_canopy(
    base: [f32; 3],
    radius: f32,
    height: f32,
    color: [f32; 4],
    yaw_rad: f32,
    rounded: bool,
) -> SceneryMesh {
    const SEGMENTS: u32 = 8;
    let [x, base_y, z] = base;
    let lean = radius * yaw_rad;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    if rounded {
        let bottom = canopy_ring(x, base_y, z, radius, SEGMENTS);
        let mid = canopy_ring(x, base_y + height * 0.55, z, radius * 0.72, SEGMENTS);
        let apex = [x + lean, base_y + height, z];
        for i in 0..SEGMENTS {
            let next = (i + 1) % SEGMENTS;
            push_triangle(
                &mut vertices,
                &mut indices,
                bottom[i as usize],
                bottom[next as usize],
                mid[next as usize],
                color,
            );
            push_triangle(
                &mut vertices,
                &mut indices,
                bottom[i as usize],
                mid[next as usize],
                mid[i as usize],
                color,
            );
            push_triangle(
                &mut vertices,
                &mut indices,
                mid[i as usize],
                mid[next as usize],
                apex,
                color,
            );
        }
        push_cap(&mut vertices, &mut indices, [x, base_y, z], &bottom, color);
    } else {
        let base_ring = canopy_ring(x, base_y, z, radius, SEGMENTS);
        let apex = [x + lean, base_y + height, z];
        for i in 0..SEGMENTS {
            let next = (i + 1) % SEGMENTS;
            push_triangle(
                &mut vertices,
                &mut indices,
                apex,
                base_ring[i as usize],
                base_ring[next as usize],
                color,
            );
        }
        push_cap(
            &mut vertices,
            &mut indices,
            [x, base_y, z],
            &base_ring,
            color,
        );
    }

    SceneryMesh { vertices, indices }
}

/// Push the (hidden, downward-facing) base cap of a canopy.
fn push_cap(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    centre: [f32; 3],
    ring: &[[f32; 3]],
    color: [f32; 4],
) {
    let segments = ring.len();
    for i in 0..segments {
        let next = (i + 1) % segments;
        push_triangle(vertices, indices, centre, ring[next], ring[i], color);
    }
}

/// Push one shaded triangle with its geometric face normal.
fn push_triangle(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    a: [f32; 3],
    b: [f32; 3],
    c: [f32; 3],
    color: [f32; 4],
) {
    let base = vertices.len() as u32;
    let normal = face_normal(a, b, c);
    vertices.push(Vertex {
        position: a,
        normal,
        color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: b,
        normal,
        color,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: c,
        normal,
        color,
        uv: SAFE_UV,
    });
    indices.extend_from_slice(&[base, base + 1, base + 2]);
}

/// Unit face normal from three corners; `SAFE_NORMAL` on degenerate faces.
fn face_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let e1 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let e2 = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let normal = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
    ];
    let length = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if length <= 1.0e-6 {
        return SAFE_NORMAL;
    }
    [normal[0] / length, normal[1] / length, normal[2] / length]
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

    // Pole: 6 m tall.
    let pole = generate_cylinder(
        [x, ground_y, z],
        0.06,
        WINDSOCK_POLE_HEIGHT_M,
        [0.62, 0.62, 0.62, 1.0],
        5,
    );
    merge_mesh(&mut vertices, &mut indices, &pole.vertices, &pole.indices);

    // Small top support: a short boom the sock hangs from.
    append_box(
        &mut vertices,
        &mut indices,
        [x + 0.35, ground_y + WINDSOCK_POLE_HEIGHT_M, z],
        [0.35, 0.035, 0.035],
        [0.60, 0.60, 0.60, 1.0],
    );

    // Sock: clearly horizontal, tapered, with alternating colour bands.
    append_windsock_sock(
        &mut vertices,
        &mut indices,
        [x, ground_y + WINDSOCK_POLE_HEIGHT_M - 0.2, z],
        WINDSOCK_SOCK_LENGTH_M,
        0.28,
        0.14,
        5,
    );

    SceneryMesh { vertices, indices }
}

/// Append a horizontal tapered sock of alternating colour bands.
///
/// A closed ring of `segments` vertices around the X axis is swept from the
/// mouth to the tip, with 3 bands (orange/white/orange). Static presentation
/// geometry; normals face away from the sock axis.
fn append_windsock_sock(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    origin: [f32; 3],
    length: f32,
    mouth_radius: f32,
    tip_radius: f32,
    segments: u32,
) {
    const BAND_COUNT: usize = 3;
    let [x0, y, z] = origin;

    let ring = |x: f32, t: f32| -> Vec<[f32; 3]> {
        let radius = lerp(mouth_radius, tip_radius, t);
        (0..segments)
            .map(|i| {
                let angle = (i as f32 / segments as f32) * 2.0 * std::f32::consts::PI;
                [x, y + radius * angle.sin(), z + radius * angle.cos()]
            })
            .collect()
    };

    for band in 0..BAND_COUNT {
        let t_a = band as f32 / BAND_COUNT as f32;
        let t_b = (band + 1) as f32 / BAND_COUNT as f32;
        let ring_a = ring(x0 + length * t_a, t_a);
        let ring_b = ring(x0 + length * t_b, t_b);
        let color = if band % 2 == 0 {
            WINDSOCK_ORANGE
        } else {
            WINDSOCK_WHITE
        };
        let axis = [(x0 + length * (t_a + t_b) * 0.5), y, z];
        append_tube_band(vertices, indices, &ring_a, &ring_b, color, axis);
    }
}

/// Append one ring band of a tapered tube with outward-facing normals.
fn append_tube_band(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    ring_a: &[[f32; 3]],
    ring_b: &[[f32; 3]],
    color: [f32; 4],
    axis: [f32; 3],
) {
    let segments = ring_a.len();
    for k in 0..segments {
        let next = (k + 1) % segments;
        let corners = [ring_a[k], ring_a[next], ring_b[next], ring_b[k]];
        let normal = outward_face_normal(&corners, axis);
        append_quad(vertices, indices, corners, color, normal);
    }
}

/// Outward unit normal for a four-corner tube band.
fn outward_face_normal(corners: &[[f32; 3]; 4], axis: [f32; 3]) -> [f32; 3] {
    let normal = face_normal(corners[0], corners[1], corners[2]);
    let mut centre = [0.0_f32; 3];
    for corner in corners {
        centre[0] += corner[0];
        centre[1] += corner[1];
        centre[2] += corner[2];
    }
    centre[0] *= 0.25;
    centre[1] *= 0.25;
    centre[2] *= 0.25;
    let outward = [
        centre[0] - axis[0],
        centre[1] - axis[1],
        centre[2] - axis[2],
    ];
    let facing_out = normal[0] * outward[0] + normal[1] * outward[1] + normal[2] * outward[2];
    if facing_out < 0.0 {
        [-normal[0], -normal[1], -normal[2]]
    } else {
        normal
    }
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

// ── G2C presentation helpers ───────────────────────────────────────────────

/// Append one quad as two triangles with a shared color and normal.
///
/// Horizontal quads should be wound CCW when viewed from above (matching the
/// runway/terrain winding convention); vertical faces are unaffected.
fn append_quad(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    corners: [[f32; 3]; 4],
    color: [f32; 4],
    normal: [f32; 3],
) {
    let base = vertices.len() as u32;
    for corner in corners {
        vertices.push(Vertex {
            position: corner,
            normal,
            color,
            uv: SAFE_UV,
        });
    }
    indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
}

/// Append one horizontal runway marking quad at height `y`.
fn append_runway_marking(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    x_bounds: [f32; 2],
    z_bounds: [f32; 2],
    y: f32,
    color: [f32; 4],
) {
    append_quad(
        vertices,
        indices,
        [
            [x_bounds[0], y, z_bounds[0]],
            [x_bounds[1], y, z_bounds[0]],
            [x_bounds[1], y, z_bounds[1]],
            [x_bounds[0], y, z_bounds[1]],
        ],
        color,
        SAFE_NORMAL,
    );
}

/// Append an axis-aligned box with per-face normals.
fn append_box(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    centre: [f32; 3],
    half_extents: [f32; 3],
    color: [f32; 4],
) {
    let [cx, cy, cz] = centre;
    let [hx, hy, hz] = half_extents;
    let corners = [
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz],
    ];
    let faces: [([usize; 4], [f32; 3]); 6] = [
        ([0, 1, 5, 4], [0.0, -1.0, 0.0]),
        ([3, 2, 6, 7], [0.0, 1.0, 0.0]),
        ([0, 3, 7, 4], [-1.0, 0.0, 0.0]),
        ([1, 2, 6, 5], [1.0, 0.0, 0.0]),
        ([0, 1, 2, 3], [0.0, 0.0, -1.0]),
        ([4, 5, 6, 7], [0.0, 0.0, 1.0]),
    ];
    for (face, normal) in faces {
        append_quad(
            vertices,
            indices,
            [
                corners[face[0]],
                corners[face[1]],
                corners[face[2]],
                corners[face[3]],
            ],
            color,
            normal,
        );
    }
}

/// Append a low-poly fence run along Z at `x`: square posts every
/// [`FENCE_POST_SPACING_M`] with two horizontal rails per span.
fn append_fence_run(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    x: f32,
    ground_y: f32,
    z_half_span: f32,
    height: f32,
    color: [f32; 4],
) {
    let post_half = [0.045, height * 0.5, 0.045];
    let mut z = -z_half_span;
    while z <= z_half_span + 0.001 {
        append_box(
            vertices,
            indices,
            [x, ground_y + height * 0.5, z],
            post_half,
            color,
        );
        z += FENCE_POST_SPACING_M;
    }

    let rail_half = [0.035, 0.02, FENCE_POST_SPACING_M * 0.5];
    for rail_y in [height * 0.875, height * 0.45] {
        let mut z0 = -z_half_span;
        while z0 + FENCE_POST_SPACING_M <= z_half_span + 0.001 {
            append_box(
                vertices,
                indices,
                [x, ground_y + rail_y, z0 + FENCE_POST_SPACING_M * 0.5],
                rail_half,
                color,
            );
            z0 += FENCE_POST_SPACING_M;
        }
    }
}

/// Append one pilot-station marker: an orange post with a plate facing the
/// runway.
fn generate_pilot_marker(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    x: f32,
    ground_y: f32,
    z: f32,
    color: [f32; 4],
) {
    append_box(
        vertices,
        indices,
        [x, ground_y + 0.7, z],
        [0.04, 0.7, 0.04],
        color,
    );
    let plate_x = x + 0.06;
    let plate_y0 = ground_y + 1.0;
    let plate_y1 = ground_y + 1.7;
    let half_w = 0.35;
    append_quad(
        vertices,
        indices,
        [
            [plate_x, plate_y0, z - half_w],
            [plate_x, plate_y0, z + half_w],
            [plate_x, plate_y1, z + half_w],
            [plate_x, plate_y1, z - half_w],
        ],
        color,
        [-1.0, 0.0, 0.0],
    );
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
    use std::collections::HashSet;

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

        // Long axis along Z (NED North), short axis along X.
        let length = max_z - min_z;
        let width = max_x - min_x;
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
            assert!(
                vertex.uv.iter().all(|c| c.is_finite()),
                "non-finite uv: {:?}",
                vertex.uv
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
    fn all_scenery_object_bases_use_same_ground_reference() {
        let scene = default_scene();
        let ground_y = DEFAULT_GROUND_Y;
        for obj in &scene.objects {
            assert!(
                (obj.position[1] - ground_y).abs() < 0.001,
                "object {:?} base Y {} != expected {}",
                obj.kind,
                obj.position[1],
                ground_y
            );
        }
    }

    #[test]
    fn runway_long_axis_is_parallel_to_render_z() {
        // Regression test: NED North -> render -Z, identity aircraft forward
        // is render -Z. The runway long axis must be parallel to Z.
        let scene = default_scene();
        let runway_verts: Vec<_> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == [0.35, 0.33, 0.30, 1.0])
            .collect();
        assert!(
            !runway_verts.is_empty(),
            "runway vertices must exist for axis check"
        );

        let min_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::INFINITY, f32::min);
        let max_z = runway_verts
            .iter()
            .map(|v| v.position[2])
            .fold(f32::NEG_INFINITY, f32::max);
        let min_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::INFINITY, f32::min);
        let max_x = runway_verts
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max);

        let z_extent = max_z - min_z;
        let x_extent = max_x - min_x;

        assert!(
            z_extent > x_extent,
            "runway long axis must be along Z (NED North), got Z={z_extent} X={x_extent}"
        );
        assert!(
            (z_extent - 120.0).abs() < 0.1,
            "runway long extent should be 120 m, got {z_extent}"
        );
        assert!(
            (x_extent - 12.0).abs() < 0.1,
            "runway short extent should be 12 m, got {x_extent}"
        );
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

    #[test]
    fn generated_flying_field_is_not_empty_and_reports_geometry() {
        let scene = default_scene();
        println!(
            "flying field: {} vertices, {} indices, {} triangles",
            scene.mesh.vertices.len(),
            scene.mesh.indices.len(),
            scene.mesh.triangle_count()
        );
        assert!(!scene.mesh.vertices.is_empty());
        assert!(!scene.mesh.indices.is_empty());
        assert!(!scene.objects.is_empty());
    }

    #[test]
    fn centerline_markings_are_segmented_dashes() {
        let scene = default_scene();
        let mut dash_edges: Vec<f32> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == MARKING_CENTER)
            .map(|v| v.position[2])
            .collect();
        assert_eq!(
            dash_edges.len() % 4,
            0,
            "each dash quad contributes 4 corners"
        );
        dash_edges.sort_by(|a, b| a.total_cmp(b));
        dash_edges.dedup();
        // CENTERLINE_DASH_COUNT dashes ⇒ 2 * count distinct parallel edges.
        assert_eq!(
            dash_edges.len(),
            (CENTERLINE_DASH_COUNT * 2) as usize,
            "expected the configured number of dash edges"
        );
        // Dashes must not form one continuous strip: consecutive distinct
        // edges sit exactly one dash length apart (6 m), never closer.
        let max_gap = dash_edges
            .windows(2)
            .map(|pair| pair[1] - pair[0])
            .fold(0.0_f32, f32::max);
        assert!(
            max_gap + 0.01 >= CENTERLINE_DASH_LENGTH_M
                && max_gap <= CENTERLINE_DASH_LENGTH_M + CENTERLINE_DASH_GAP_M + 0.01,
            "centreline spacing mismatch: max gap {max_gap}"
        );
    }

    #[test]
    fn threshold_markings_are_present_at_both_runway_ends() {
        let scene = default_scene();
        let threshold_z: Vec<f32> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == MARKING_THRESHOLD)
            .map(|v| v.position[2])
            .collect();
        assert!(
            threshold_z.len() >= 8,
            "threshold bars must exist (got {} vertices)",
            threshold_z.len()
        );
        let min_z = threshold_z.iter().fold(f32::INFINITY, |min, &z| min.min(z));
        let max_z = threshold_z
            .iter()
            .fold(f32::NEG_INFINITY, |max, &z| max.max(z));
        assert!(
            min_z <= -(RUNWAY_HALF_LENGTH_M - 0.5),
            "missing threshold at the -Z end"
        );
        assert!(
            max_z >= RUNWAY_HALF_LENGTH_M - 0.5,
            "missing threshold at the +Z end"
        );
    }

    #[test]
    fn flightline_stays_outside_the_runway_safety_area() {
        let scene = default_scene();
        let safety = runway_safety_rect(); // [min_x, min_z, max_x, max_z]
        assert!(FLIGHTLINE_X_M > safety[2]);
        assert!(PILOT_MARKER_X_M > safety[2]);
        for obj in &scene.objects {
            if matches!(
                obj.kind,
                SceneryVisualKind::Fence | SceneryVisualKind::Marker
            ) {
                assert!(
                    obj.position[0] > safety[2],
                    "{:?} at x={} inside runway safety area",
                    obj.kind,
                    obj.position[0]
                );
            }
        }
        for vertex in &scene.mesh.vertices {
            if vertex.color == FENCE_WHITE {
                assert!(
                    vertex.position[0] >= safety[2],
                    "fence geometry inside runway safety area"
                );
            }
        }
    }

    #[test]
    fn exactly_four_pilot_markers_are_placed() {
        let scene = default_scene();
        let markers: Vec<&SceneryObject> = scene
            .objects
            .iter()
            .filter(|obj| obj.kind == SceneryVisualKind::Marker)
            .collect();
        assert_eq!(markers.len(), 4);
        for marker in markers {
            assert_eq!(marker.position[0], PILOT_MARKER_X_M);
            assert!(
                PILOT_MARKER_Z_M.contains(&marker.position[2]),
                "unexpected pilot station z={}",
                marker.position[2]
            );
        }
    }

    #[test]
    fn windsock_is_above_ground_and_extends_horizontally() {
        let scene = default_scene();
        assert!(
            scene
                .objects
                .iter()
                .any(|obj| obj.kind == SceneryVisualKind::Windsock),
            "windsock object must exist"
        );
        let sock: Vec<&Vertex> = scene
            .mesh
            .vertices
            .iter()
            .filter(|v| v.color == WINDSOCK_ORANGE)
            .collect();
        assert!(!sock.is_empty(), "windsock sock geometry must exist");
        assert!(
            sock.iter().all(|v| v.position[1] > DEFAULT_GROUND_Y + 4.0),
            "windsock sock must sit well above the ground"
        );
        let min_x = sock
            .iter()
            .map(|v| v.position[0])
            .fold(f32::INFINITY, f32::min);
        let max_x = sock
            .iter()
            .map(|v| v.position[0])
            .fold(f32::NEG_INFINITY, f32::max);
        assert!(
            (15.0 - 0.1..=15.0 + 0.3).contains(&min_x),
            "sock must start at the pole, got min_x={min_x}"
        );
        assert!(
            max_x > 15.0 + 1.5,
            "sock must extend horizontally away from the pole, got max_x={max_x}"
        );
        assert!(
            max_x <= 15.0 + WINDSOCK_SOCK_LENGTH_M + 0.1,
            "sock overshoots its length, got max_x={max_x}"
        );
    }

    #[test]
    fn tree_variants_are_deterministic_and_stay_in_range() {
        for index in 0..DEFAULT_TREE_COUNT {
            let a = deterministic_tree_variant(DEFAULT_TREE_SEED, index);
            let b = deterministic_tree_variant(DEFAULT_TREE_SEED, index);
            assert_eq!(a.height_scale.to_bits(), b.height_scale.to_bits());
            assert_eq!(a.canopy_radius.to_bits(), b.canopy_radius.to_bits());
            assert_eq!(a.yaw_rad.to_bits(), b.yaw_rad.to_bits());
            assert_eq!(a.canopy_color, b.canopy_color);
            assert_eq!(a.rounded, b.rounded);
            assert!((0.80..=1.25).contains(&a.height_scale));
            assert!((0.85..=1.30).contains(&a.canopy_radius));
            assert!((-0.40..=0.40).contains(&a.yaw_rad));
        }
        // The same seed + index must also yield identical per-object
        // transforms across independent generations.
        let scene_a = default_scene();
        let scene_b = default_scene();
        for (a, b) in scene_a.objects.iter().zip(scene_b.objects.iter()) {
            if a.kind == SceneryVisualKind::TreeTrunk {
                assert_eq!(a.rotation_yaw_rad.to_bits(), b.rotation_yaw_rad.to_bits());
                assert_eq!(a.scale.to_bits(), b.scale.to_bits());
            }
        }
    }

    #[test]
    fn tree_variation_is_not_uniform() {
        let variants: Vec<TreeVariant> = (0..DEFAULT_TREE_COUNT)
            .map(|index| deterministic_tree_variant(DEFAULT_TREE_SEED, index))
            .collect();
        let unique_heights: HashSet<u32> = variants
            .iter()
            .map(|variant| variant.height_scale.to_bits())
            .collect();
        let unique_yaws: HashSet<u32> = variants
            .iter()
            .map(|variant| variant.yaw_rad.to_bits())
            .collect();
        let unique_greens: HashSet<[u32; 4]> = variants
            .iter()
            .map(|variant| variant.canopy_color.map(f32::to_bits))
            .collect();
        let rounded_count = variants.iter().filter(|variant| variant.rounded).count();
        assert!(
            unique_heights.len() > DEFAULT_TREE_COUNT / 2,
            "heights too uniform ({} unique)",
            unique_heights.len()
        );
        assert!(
            unique_yaws.len() > DEFAULT_TREE_COUNT / 2,
            "yaw too uniform ({} unique)",
            unique_yaws.len()
        );
        assert!(
            unique_greens.len() > DEFAULT_TREE_COUNT / 2,
            "canopy greens too uniform ({} unique)",
            unique_greens.len()
        );
        assert!(
            (1..DEFAULT_TREE_COUNT).contains(&rounded_count),
            "expected both silhouettes, got rounded={rounded_count}"
        );
    }

    #[test]
    fn scenery_geometry_stays_within_an_explicit_budget() {
        let scene = default_scene();
        let triangles = scene.mesh.triangle_count();
        assert!(
            triangles >= 2_500,
            "G2C scene lost presentation richness: {triangles} triangles"
        );
        assert!(
            triangles <= MAX_FLYING_FIELD_TRIANGLES as usize,
            "scene exceeded budget: {triangles} > {MAX_FLYING_FIELD_TRIANGLES}"
        );
    }
}
