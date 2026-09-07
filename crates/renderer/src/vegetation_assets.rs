//! G3D: procedural production tree assets.
//!
//! # Goal
//!
//! Replaces the FlyingField cone/dome placeholders with a small, coherent,
//! deterministic set of original tree assets. Everything here is pure,
//! seed-free procedural geometry: the same build always produces the same
//! meshes, so no DCC pipeline or external download is required. Provenance:
//! generated in-repository by `src/bin/generate_tree_assets.rs`, which also
//! exports the same assets as committed GLB files under
//! `models/assets/scenery`.
//!
//! # Families
//!
//! - [`VegetationSpecies::Deciduous`] — broadleaf field tree. Crown is a
//!   cluster of overlapping ellipsoid foliage blobs around a tapered trunk
//!   with short branch stubs; two hard-coded crown variants.
//! - [`VegetationSpecies::Conifer`] — pine-like tree. Crown is a stack of
//!   tapered tiers with overhanging rims; two hard-coded shape variants.
//!
//! Per-instance variation (scale, yaw, tint, canopy proportion) is applied by
//! the placement layer, NOT by generating dozens of near-identical meshes.
//!
//! # Materials
//!
//! Every LOD is split into two render parts ([`VegetationPart::Bark`] and
//! [`VegetationPart::Foliage`]) with distinct PBR factors: metallic 0,
//! roughness 0.85 (bark) / 0.65 (foliage). Base colors are baked per-vertex
//! (linear factors, consistent with the scenery vertex-color convention); no
//! emission, no fluorescent colors.
//!
//! # LOD policy
//!
//! - LOD0: full silhouette (detailed blobs / tiers + trunk).
//! - LOD1: reduced segment counts and fewer crown lobes (~40–60% of LOD0).
//! - LOD2: single wobbly dome (deciduous) or two-tier silhouette (conifer)
//!   (~10–25% of LOD0). Never a cone and never a flat billboard — the
//!   silhouette stays tree-like at every LOD.
//!
//! LOD thresholds and hysteresis live in `vegetation.rs`; the meshes here are
//! pure assets.

use crate::mesh::{AircraftMesh, MeshError, SAFE_NORMAL, SAFE_UV, Vertex};
use serde_json::json;
use std::f32::consts::{PI, TAU};

// ── Public asset model ─────────────────────────────────────────────────────

/// Tree family used to balance species on the field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VegetationSpecies {
    Deciduous,
    Conifer,
}

/// One render part of a tree LOD. Parts are drawn with distinct materials.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VegetationPart {
    Bark,
    Foliage,
}

impl VegetationPart {
    /// Index of this part within a (asset, LOD) batch pair.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Bark => 0,
            Self::Foliage => 1,
        }
    }

    /// Total number of parts per tree LOD.
    #[must_use]
    pub const fn count() -> usize {
        2
    }
}

/// One LOD level of a tree: two closed, separately-materialed meshes.
#[derive(Debug, Clone)]
pub struct VegetationLod {
    pub bark: AircraftMesh,
    pub foliage: AircraftMesh,
}

impl VegetationLod {
    /// Total triangle count across both parts.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.bark.indices().len() / 3 + self.foliage.indices().len() / 3
    }

    /// Triangle count of a single part.
    #[must_use]
    pub fn part_triangle_count(&self, part: VegetationPart) -> usize {
        match part {
            VegetationPart::Bark => self.bark.indices().len() / 3,
            VegetationPart::Foliage => self.foliage.indices().len() / 3,
        }
    }

    /// Total vertex count across both parts.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.bark.vertices().len() + self.foliage.vertices().len()
    }
}

/// The three LOD levels of one tree asset.
#[derive(Debug, Clone)]
pub struct VegetationLodSet {
    pub lod0: VegetationLod,
    pub lod1: VegetationLod,
    pub lod2: VegetationLod,
}

impl VegetationLodSet {
    /// LOD level by class index.
    #[must_use]
    pub fn lod(&self, class: u8) -> Option<&VegetationLod> {
        match class {
            0 => Some(&self.lod0),
            1 => Some(&self.lod1),
            2 => Some(&self.lod2),
            _ => None,
        }
    }

    /// Triangle count of the mesh a given LOD class actually renders.
    #[must_use]
    pub fn triangle_count(&self, class: u8) -> usize {
        self.lod(class).map_or(0, VegetationLod::triangle_count)
    }
}

/// One production tree asset (species × variant × LOD chain).
#[derive(Debug, Clone)]
pub struct VegetationAsset {
    pub species: VegetationSpecies,
    /// 0-based variant index within the species.
    pub variant: u8,
    /// Stable asset name, also used for the exported GLB file stem.
    pub name: &'static str,
    pub lods: VegetationLodSet,
    /// Ground-relative centre of the tight bounding sphere (built from LOD0).
    pub bounds_center: [f32; 3],
    /// Radius of the tight bounding sphere (built from LOD0), unit scale.
    pub bounds_radius: f32,
    /// Full tree height at unit scale (top of the crown above ground).
    pub height_m: f32,
}

impl VegetationAsset {
    /// Natural-tree extent of this asset at the given instance scale.
    #[must_use]
    pub fn scaled_radius(&self, scale: f32) -> f32 {
        self.bounds_radius * scale
    }

    /// Full height of this asset at the given instance scale.
    #[must_use]
    pub fn scaled_height(&self, scale: f32) -> f32 {
        self.height_m * scale
    }
}

/// The complete committed asset set used by the renderer.
#[derive(Debug, Clone)]
pub struct VegetationAssetSet {
    assets: Vec<VegetationAsset>,
}

impl VegetationAssetSet {
    /// Build the production asset set (deterministic, no I/O).
    #[must_use]
    pub fn production() -> Self {
        let assets = vec![
            build_deciduous_variant(0, "field_deciduous_a"),
            build_deciduous_variant(1, "field_deciduous_b"),
            build_conifer_variant(0, "field_conifer_a"),
            build_conifer_variant(1, "field_conifer_b"),
        ];
        Self { assets }
    }

    /// Minimal one-asset set for tests and the headless GPU probes.
    #[must_use]
    pub fn single_default() -> Self {
        Self {
            assets: vec![build_deciduous_variant(0, "field_deciduous_a")],
        }
    }

    #[must_use]
    pub fn assets(&self) -> &[VegetationAsset] {
        &self.assets
    }

    #[must_use]
    pub fn get(&self, index: usize) -> Option<&VegetationAsset> {
        self.assets.get(index)
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.assets.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.assets.is_empty()
    }
}

/// Species count target enforced by tests (speciess A/B requirement).
pub const SPECIES_TARGET: usize = 2;
/// Minimum visual variants per species (mesh variants, not scale-only).
pub const VARIANTS_PER_SPECIES_TARGET: usize = 2;

// ── Material palette (linear vertex-color factors) ─────────────────────────

/// Bark base color (linear). Warm dark brown, non-metallic.
pub const BARK_BASE: [f32; 4] = [0.17, 0.115, 0.06, 1.0];
/// Foliage base green (linear). Readable in HDR but never fluorescent.
pub const FOLIAGE_BASE: [f32; 4] = [0.10, 0.26, 0.08, 1.0];
/// Deciduous foliage lean: slightly yellower green (linear).
const DECIDUOUS_LEAN: [f32; 3] = [0.10, 0.05, -0.02];
/// Conifer foliage lean: darker, cooler green (linear).
const CONIFER_LEAN: [f32; 3] = [-0.02, 0.02, 0.01];

/// PBR metallic factor per part (trees are fully dielectric).
#[must_use]
pub const fn part_metallic(part: VegetationPart) -> f32 {
    match part {
        VegetationPart::Bark | VegetationPart::Foliage => 0.0,
    }
}

/// PBR roughness per part: bark matte, foliage slightly glossier but never
/// mirror-like.
#[must_use]
pub const fn part_roughness(part: VegetationPart) -> f32 {
    match part {
        VegetationPart::Bark => 0.85,
        VegetationPart::Foliage => 0.65,
    }
}

// ── Variant parameter tables (deterministic, hard-coded) ───────────────────

struct DeciduousParams {
    trunk_height_m: f32,
    trunk_base_radius_m: f32,
    crown_center_y_m: f32,
    crown_height_m: f32,
    crown_radius_m: f32,
    /// (anchor x, anchor z, horizontal factor, vertical offset factor,
    ///  blob radius factor).
    blobs: &'static [([f32; 2], f32, f32, f32)],
}

const DECIDUOUS_A_BLOBS: [([f32; 2], f32, f32, f32); 6] = [
    ([1.0, 0.0], 0.52, 0.10, 0.46),
    ([0.31, 0.95], 0.55, 0.08, 0.48),
    ([-0.81, 0.59], 0.50, 0.12, 0.44),
    ([-0.81, -0.59], 0.53, 0.30, 0.46),
    ([0.31, -0.95], 0.42, 0.34, 0.40),
    ([0.0, 0.0], 0.30, 0.15, 0.42),
];

const DECIDUOUS_B_BLOBS: [([f32; 2], f32, f32, f32); 5] = [
    ([1.0, 0.0], 0.45, 0.12, 0.40),
    ([0.0, 1.0], 0.48, 0.10, 0.42),
    ([-0.9, 0.44], 0.46, 0.16, 0.40),
    ([-0.9, -0.44], 0.44, 0.32, 0.38),
    ([0.0, 0.0], 0.34, 0.20, 0.36),
];

fn deciduous_params(variant: u8) -> DeciduousParams {
    match variant {
        0 => DeciduousParams {
            trunk_height_m: 1.7,
            trunk_base_radius_m: 0.13,
            crown_center_y_m: 3.6,
            crown_height_m: 2.6,
            crown_radius_m: 1.9,
            blobs: &DECIDUOUS_A_BLOBS,
        },
        _ => DeciduousParams {
            // Variant B: narrower, taller crown (different trunk/canopy
            // proportion so the two variants read differently at a glance).
            trunk_height_m: 2.1,
            trunk_base_radius_m: 0.12,
            crown_center_y_m: 4.2,
            crown_height_m: 2.2,
            crown_radius_m: 1.7,
            blobs: &DECIDUOUS_B_BLOBS,
        },
    }
}

struct ConiferParams {
    trunk_height_m: f32,
    trunk_base_radius_m: f32,
    crown_height_m: f32,
    crown_radius_m: f32,
    /// y fractions of tier bases within the crown.
    tiers: &'static [f32],
    /// tier radii as a fraction of the crown radius.
    tier_radii: &'static [f32],
}

const CONIFER_A_BASES: [f32; 4] = [0.0, 0.30, 0.60, 0.85];
const CONIFER_A_RADII: [f32; 4] = [1.0, 0.72, 0.46, 0.24];
const CONIFER_B_BASES: [f32; 5] = [0.0, 0.25, 0.48, 0.70, 0.86];
const CONIFER_B_RADII: [f32; 5] = [0.85, 0.68, 0.50, 0.33, 0.17];

fn conifer_params(variant: u8) -> ConiferParams {
    match variant {
        0 => ConiferParams {
            trunk_height_m: 1.1,
            trunk_base_radius_m: 0.16,
            crown_height_m: 5.6,
            crown_radius_m: 2.25,
            tiers: &CONIFER_A_BASES,
            tier_radii: &CONIFER_A_RADII,
        },
        _ => ConiferParams {
            // Variant B: steeper, narrower spruce-like crown.
            trunk_height_m: 1.2,
            trunk_base_radius_m: 0.15,
            crown_height_m: 6.2,
            crown_radius_m: 1.9,
            tiers: &CONIFER_B_BASES,
            tier_radii: &CONIFER_B_RADII,
        },
    }
}

// ── LOD fidelity tables ────────────────────────────────────────────────────

/// (segments, stacks) of the LOD0 deciduous blob spheres.
const DECIDUOUS_LOD0_SEGMENTS: u32 = 8;
const DECIDUOUS_LOD0_STACKS: u32 = 4;
/// LOD1 keeps fewer lobes at a slightly coarser vertical division.
const DECIDUOUS_LOD1_SEGMENTS: u32 = 8;
const DECIDUOUS_LOD1_STACKS: u32 = 3;
const DECIDUOUS_LOD1_BLOBS: usize = 5;
/// LOD2 dome: segments × stacks of the single wobbly dome.
const DECIDUOUS_LOD2_SEGMENTS: u32 = 8;
const DECIDUOUS_LOD2_STACKS: u32 = 5;

const CONIFER_LOD0_SEGMENTS: u32 = 8;
const CONIFER_LOD1_SEGMENTS: u32 = 6;
const CONIFER_LOD1_TIERS: usize = 3;
const CONIFER_LOD2_SEGMENTS: u32 = 4;

const TRUNK_LOD0_SEGMENTS: u32 = 8;
const TRUNK_LOD1_SEGMENTS: u32 = 6;
const TRUNK_LOD2_SEGMENTS: u32 = 5;

// ── Geometric helpers ──────────────────────────────────────────────────────

fn add3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale3(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn length3(a: [f32; 3]) -> f32 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn normalize3(a: [f32; 3]) -> [f32; 3] {
    let length = length3(a);
    if length <= 1.0e-6 {
        return SAFE_NORMAL;
    }
    scale3(a, length.recip())
}

fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Deterministic per-vertex shade factor for bark: subtle azimuthal noise so
/// the trunk is not a flat cylinder.
fn bark_shade(angle: f32, index: u32) -> f32 {
    0.82 + 0.18 * (index as f32 * 0.618 + angle * 3.0).sin().abs().powf(0.5)
}

/// Deterministic per-vertex foliage shade: darker toward the blob root and
/// depth, lighter on the outer/upper shell. Pure function of blob-local
/// coordinates so generation is reproducible.
fn foliage_shade(vertical: f32, azimuth: f32) -> f32 {
    0.68 + 0.32 * (0.5 + 0.5 * (vertical * 2.1 + azimuth).cos()).powf(0.8)
}

/// Foliage base color plus the species lean, finished to linear RGBA.
const fn shade_color(base: [f32; 4], lean: [f32; 3]) -> [f32; 4] {
    [
        (base[0] + lean[0]).clamp(0.0, 1.0),
        (base[1] + lean[1]).clamp(0.0, 1.0),
        (base[2] + lean[2]).clamp(0.0, 1.0),
        1.0,
    ]
}

/// Merge geometry appends into one validated mesh.
fn assemble(parts: &[(Vec<Vertex>, Vec<u32>)]) -> Result<AircraftMesh, MeshError> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (part_vertices, part_indices) in parts {
        let base = u32::try_from(vertices.len()).unwrap_or(u32::MAX);
        for vertex in part_vertices {
            vertices.push(*vertex);
        }
        for &index in part_indices {
            indices.push(base.saturating_add(index));
        }
    }
    AircraftMesh::new(vertices, indices)
}

/// Tapered cylinder from `y = 0` to `y = height` with smooth side normals
/// (computed from the taper) and a bottom cap.
fn tapered_cylinder(
    base_radius: f32,
    top_radius: f32,
    height: f32,
    segments: u32,
    origin: [f32; 3],
    color: [f32; 4],
    shade: bool,
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(segments >= 3);
    let mut vertices = Vec::with_capacity(segments as usize * 2 + 2);
    let mut indices = Vec::with_capacity(segments as usize * 6 + segments as usize * 3);
    let side_normal_lean = (base_radius - top_radius) / height.max(1.0e-6);
    for i in 0..segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (sin, cos) = angle.sin_cos();
        let shade_factor = if shade { bark_shade(angle, i) } else { 1.0 };
        let shaded = [
            color[0] * shade_factor,
            color[1] * shade_factor,
            color[2] * shade_factor,
            color[3],
        ];
        let normal = normalize3([cos, side_normal_lean, sin]);
        vertices.push(Vertex {
            position: [
                origin[0] + base_radius * cos,
                origin[1],
                origin[2] + base_radius * sin,
            ],
            normal,
            color: shaded,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [
                origin[0] + top_radius * cos,
                origin[1] + height,
                origin[2] + top_radius * sin,
            ],
            normal,
            color: shaded,
            uv: SAFE_UV,
        });
    }
    for i in 0..segments {
        let b0 = 2 * i;
        let t0 = 2 * i + 1;
        let b1 = 2 * ((i + 1) % segments);
        let t1 = 2 * ((i + 1) % segments) + 1;
        indices.extend_from_slice(&[b0, b1, t0, t0, b1, t1]);
    }
    // Bottom cap (ground-facing or hidden; keeps the mesh watertight).
    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [origin[0], origin[1], origin[2]],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[centre, 2 * i, 2 * next]);
    }
    (vertices, indices)
}

/// A closed lat-lon ellipsoid blob shell (deciduous crown lobe).
fn blob_sphere(
    centre: [f32; 3],
    radius: f32,
    squash: f32,
    segments: u32,
    stacks: u32,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(segments >= 3 && stacks >= 2);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for row in 0..=stacks {
        let vertical = 1.0 - 2.0 * (row as f32 / stacks as f32); // 1 top .. -1 bottom
        let latitude = vertical * (PI * 0.5);
        let lat_cos = latitude.cos();
        let ring_y = centre[1] + radius * squash * vertical;
        let ring_radius = radius * lat_cos;
        for column in 0..segments {
            let angle = (column as f32 / segments as f32) * TAU;
            let (sin, cos) = angle.sin_cos();
            let shade = foliage_shade(vertical, angle);
            vertices.push(Vertex {
                position: [
                    centre[0] + ring_radius * cos,
                    ring_y,
                    centre[2] + ring_radius * sin,
                ],
                normal: SAFE_NORMAL, // filled below
                color: [
                    color[0] * shade,
                    color[1] * shade,
                    color[2] * shade,
                    color[3],
                ],
                uv: SAFE_UV,
            });
        }
    }
    // Smooth ellipsoid normals (analytic).
    let rx = 1.0 / radius.max(1.0e-6);
    let ry = 1.0 / (radius * squash).max(1.0e-6);
    for vertex in &mut vertices {
        vertex.normal = normalize3([
            (vertex.position[0] - centre[0]) * rx,
            (vertex.position[1] - centre[1]) * ry,
            (vertex.position[2] - centre[2]) * rx,
        ]);
    }
    for row in 0..stacks {
        for column in 0..segments {
            let next = (column + 1) % segments;
            let a = row * segments + column;
            let b = row * segments + next;
            let c = (row + 1) * segments + column;
            let d = (row + 1) * segments + next;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    (vertices, indices)
}

/// A closed frustum tier (conifer layer) with an overhanging bottom rim.
fn conifer_tier(
    y0: f32,
    y1: f32,
    r0: f32,
    r1: f32,
    rim: f32,
    segments: u32,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0..segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (sin, cos) = angle.sin_cos();
        let shade = foliage_shade(0.5 + 0.3 * (i as f32 / segments as f32), angle);
        let outer = normalize3([cos, 0.0, sin]);
        // Surface normal blends the tier lean toward the up axis.
        let normal = normalize3([outer[0], 0.55 + (r0 - r1) * 0.08, outer[2]]);
        for (y, r, normal) in [
            (y0 - rim, r0.max(r1) * 1.06, normal),
            (y0, r0, normal),
            (y1, r1.max(0.001), normal),
        ] {
            vertices.push(Vertex {
                position: [r * cos, y, r * sin],
                normal,
                color: [
                    color[0] * shade,
                    color[1] * shade,
                    color[2] * shade,
                    color[3],
                ],
                uv: SAFE_UV,
            });
        }
    }
    for i in 0..segments {
        let next = (i + 1) % segments;
        let a0 = 3 * i;
        let a1 = 3 * i + 1;
        let a2 = 3 * i + 2;
        let b0 = 3 * next;
        let b1 = 3 * next + 1;
        let b2 = 3 * next + 2;
        indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
        indices.extend_from_slice(&[a1, b1, b2, a1, b2, a2]);
    }
    // Bottom cap (hidden under the next tier / above the trunk).
    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, y0 - rim, 0.0],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[centre, 3 * i, 3 * next]);
    }
    (vertices, indices)
}

/// Closed tip cone for the conifer crown apex.
fn tip_cone(
    base_y: f32,
    tip_y: f32,
    base_radius: f32,
    segments: u32,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let lean = base_radius / (tip_y - base_y).max(1.0e-6);
    for i in 0..segments {
        let angle = (i as f32 / segments as f32) * TAU;
        let (sin, cos) = angle.sin_cos();
        let shade = foliage_shade(0.85, angle);
        vertices.push(Vertex {
            position: [base_radius * cos, base_y, base_radius * sin],
            normal: normalize3([cos, lean, sin]),
            color: [
                color[0] * shade,
                color[1] * shade,
                color[2] * shade,
                color[3],
            ],
            uv: SAFE_UV,
        });
    }
    let apex = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, tip_y, 0.0],
        normal: [0.0, 1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[apex, i, next]);
    }
    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, base_y, 0.0],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for i in 0..segments {
        let next = (i + 1) % segments;
        indices.extend_from_slice(&[centre, next, i]);
    }
    (vertices, indices)
}

/// LOD2 single "wobbly dome": a lathe whose ring radius follows a fixed
/// lumpy profile, closing at the apex. Reads as a tree silhouette from any
/// view, at a fraction of the LOD0 cost. Never a geometric cone.
fn wobbly_dome(
    crown_centre: [f32; 3],
    radius: f32,
    height: f32,
    segments: u32,
    stacks: u32,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(segments >= 5 && stacks >= 3);
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let profile = |t: f32| -> f32 {
        // Fixed lumpy profile: three overlapping sine bumps, deterministic.
        let bumps = 0.78
            + 0.14 * (t * TAU).sin()
            + 0.10 * (t * TAU * 2.3 + 1.1).sin()
            + 0.08 * (t * TAU * 5.1 + 4.2).cos().abs();
        1.0 + bumps * 0.25
    };
    let crown_base_y = crown_centre[1] - height * 0.5;
    for row in 0..=stacks {
        let t = row as f32 / stacks as f32;
        let y = crown_base_y + height * t;
        let ring_radius = radius * profile(t) * (1.0 - t).powf(0.85);
        for column in 0..segments {
            let angle = (column as f32 / segments as f32) * TAU;
            let (sin, cos) = angle.sin_cos();
            let shade = foliage_shade(t, angle);
            vertices.push(Vertex {
                position: [
                    crown_centre[0] + ring_radius * cos,
                    y,
                    crown_centre[2] + ring_radius * sin,
                ],
                normal: SAFE_NORMAL, // filled below
                color: [
                    color[0] * shade,
                    color[1] * shade,
                    color[2] * shade,
                    color[3],
                ],
                uv: SAFE_UV,
            });
        }
    }
    for vertex in &mut vertices {
        let radial = [
            vertex.position[0] - crown_centre[0],
            0.0,
            vertex.position[2] - crown_centre[2],
        ];
        let radial_length = length3(radial);
        if radial_length <= 1.0e-5 {
            vertex.normal = [0.0, 1.0, 0.0];
            continue;
        }
        let up_fraction = ((vertex.position[1] - crown_base_y) / height).clamp(0.0, 1.0);
        let outward = 0.55 + 0.45 * (1.0 - up_fraction);
        vertex.normal = normalize3([
            radial[0] / radial_length * outward,
            1.0 - outward,
            radial[2] / radial_length * outward,
        ]);
    }
    for row in 0..stacks {
        for column in 0..segments {
            let next = (column + 1) % segments;
            let a = row * segments + column;
            let b = row * segments + next;
            let c = (row + 1) * segments + column;
            let d = (row + 1) * segments + next;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    (vertices, indices)
}

/// A short tapered branch stub angled slightly upward from its base point.
fn branch_stub(base: &[f32; 3], length: f32, segments: u32) -> (Vec<Vertex>, Vec<u32>) {
    let horizontal = length3([base[0], 0.0, base[2]]).max(1.0e-4);
    let direction = normalize3([base[0] / horizontal, 0.45, base[2] / horizontal]);
    let up = if direction[1] < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let right = normalize3(cross3(direction, up));
    let up2 = cross3(right, direction);
    const SUBDIV: usize = 5;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for i in 0..SUBDIV {
        let t = i as f32 / (SUBDIV - 1) as f32;
        let radius = 0.035 * (1.0 - 0.55 * t);
        let centre = [
            base[0] + direction[0] * length * t,
            base[1] + direction[1] * length * t,
            base[2] + direction[2] * length * t,
        ];
        for k in 0..segments {
            let angle = (k as f32 / segments as f32) * TAU;
            let (sin, cos) = angle.sin_cos();
            let offset = add3(scale3(right, cos * radius), scale3(up2, sin * radius));
            let shade = bark_shade(angle, i as u32);
            vertices.push(Vertex {
                position: [
                    centre[0] + offset[0],
                    centre[1] + offset[1],
                    centre[2] + offset[2],
                ],
                normal: scale3(offset, radius.recip().max(1.0)),
                color: [
                    BARK_BASE[0] * shade,
                    BARK_BASE[1] * shade,
                    BARK_BASE[2] * shade,
                    1.0,
                ],
                uv: SAFE_UV,
            });
        }
    }
    for ring in 0..(SUBDIV - 1) {
        for k in 0..segments {
            let next = (k + 1) % segments;
            let ring = ring as u32;
            let a = ring * segments + k;
            let b = ring * segments + next;
            let c = (ring + 1) * segments + k;
            let d = (ring + 1) * segments + next;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    (vertices, indices)
}

// ── Builders ───────────────────────────────────────────────────────────────

/// Full LOD0/LOD1 deciduous tree: tapered trunk + branch stubs + blobs.
fn deciduous_lod(
    params: &DeciduousParams,
    blob_count: usize,
    segments: u32,
    stacks: u32,
    with_branches: bool,
) -> VegetationLod {
    let trunk_segments = if segments >= 8 {
        TRUNK_LOD0_SEGMENTS
    } else {
        TRUNK_LOD1_SEGMENTS
    };
    let trunk = tapered_cylinder(
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.55,
        params.trunk_height_m,
        trunk_segments,
        [0.0, 0.0, 0.0],
        BARK_BASE,
        true,
    );
    let color = shade_color(FOLIAGE_BASE, DECIDUOUS_LEAN);
    let mut foliage_parts = Vec::new();
    for (index, &(anchor, horizontal, vertical, blob_radius)) in
        params.blobs.iter().take(blob_count).enumerate()
    {
        let centre = [
            anchor[0] * params.crown_radius_m * horizontal,
            params.trunk_height_m + params.crown_center_y_m + vertical * params.crown_height_m,
            anchor[1] * params.crown_radius_m * horizontal,
        ];
        let radius = params.crown_radius_m * blob_radius;
        let squash = if index % 2 == 0 { 0.82 } else { 0.68 };
        foliage_parts.push(blob_sphere(centre, radius, squash, segments, stacks, color));
    }
    let mut bark_parts = vec![trunk];
    if with_branches {
        for &(anchor, horizontal, vertical, _blob_radius) in params.blobs.iter().take(3) {
            let stub_base = [
                anchor[0] * params.crown_radius_m * horizontal * 0.7,
                params.trunk_height_m + 0.35 + vertical * params.crown_height_m,
                anchor[1] * params.crown_radius_m * horizontal * 0.7,
            ];
            bark_parts.push(branch_stub(&stub_base, 0.6, segments));
        }
    }
    let bark = assemble(&bark_parts).expect("deciduous bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("deciduous foliage mesh is valid");
    VegetationLod { bark, foliage }
}

/// LOD2 deciduous: single wobbly dome + tapered trunk stub.
fn deciduous_lod2(params: &DeciduousParams) -> VegetationLod {
    let trunk = tapered_cylinder(
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.55,
        params.trunk_height_m,
        TRUNK_LOD2_SEGMENTS,
        [0.0, 0.0, 0.0],
        BARK_BASE,
        true,
    );
    let crown_centre = [0.0, params.trunk_height_m + params.crown_center_y_m, 0.0];
    let dome = wobbly_dome(
        crown_centre,
        params.crown_radius_m,
        params.crown_height_m,
        DECIDUOUS_LOD2_SEGMENTS,
        DECIDUOUS_LOD2_STACKS,
        shade_color(FOLIAGE_BASE, DECIDUOUS_LEAN),
    );
    let bark = assemble(&[trunk]).expect("deciduous LOD2 bark mesh is valid");
    let foliage = assemble(&[dome]).expect("deciduous LOD2 foliage mesh is valid");
    VegetationLod { bark, foliage }
}

fn darken(color: [f32; 4], tier: usize) -> [f32; 4] {
    let factor = 1.0 - 0.10 * tier as f32;
    [
        color[0] * factor,
        color[1] * factor,
        color[2] * factor,
        color[3],
    ]
}

/// Full conifer tree: tapered trunk + tier stack + tip cone.
fn conifer_lod(params: &ConiferParams, tier_count: usize, segments: u32) -> VegetationLod {
    let trunk_segments = if tier_count >= 4 {
        TRUNK_LOD0_SEGMENTS
    } else {
        TRUNK_LOD1_SEGMENTS
    };
    let trunk = tapered_cylinder(
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.5,
        params.trunk_height_m,
        trunk_segments,
        [0.0, 0.0, 0.0],
        BARK_BASE,
        true,
    );
    let color = shade_color(FOLIAGE_BASE, CONIFER_LEAN);
    let bases = &params.tiers[..tier_count.min(params.tiers.len())];
    let radii = &params.tier_radii[..tier_count.min(params.tier_radii.len())];
    let crown_base_y = params.trunk_height_m;
    let mut foliage_parts = Vec::new();
    for (tier_index, &y_fraction) in bases.iter().enumerate() {
        let y0 = crown_base_y + y_fraction * params.crown_height_m;
        let y1_fraction = if tier_index + 1 < bases.len() {
            bases[tier_index + 1] * 0.55 + y_fraction * 0.45
        } else {
            y_fraction + 0.12
        };
        let y1 = crown_base_y + y1_fraction * params.crown_height_m;
        let r0 = params.crown_radius_m * radii[tier_index];
        let r1 = if tier_index + 1 < bases.len() {
            params.crown_radius_m * radii[tier_index + 1]
        } else {
            params.crown_radius_m * radii[tier_index] * 0.3
        };
        foliage_parts.push(conifer_tier(
            y0,
            y1,
            r0,
            r1,
            0.3 * r0,
            segments,
            darken(color, tier_index.min(2)),
        ));
    }
    let tip_start = crown_base_y + params.crown_height_m;
    foliage_parts.push(tip_cone(
        tip_start - 0.02,
        tip_start + params.crown_height_m * 0.16,
        params.crown_radius_m * 0.16,
        segments,
        color,
    ));
    let bark = assemble(&[trunk]).expect("conifer bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("conifer foliage mesh is valid");
    VegetationLod { bark, foliage }
}

/// LOD2 conifer: two coarse tiers; cheap pine silhouette.
fn conifer_lod2(params: &ConiferParams) -> VegetationLod {
    let trunk = tapered_cylinder(
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.5,
        params.trunk_height_m,
        TRUNK_LOD2_SEGMENTS,
        [0.0, 0.0, 0.0],
        BARK_BASE,
        true,
    );
    let color = shade_color(FOLIAGE_BASE, CONIFER_LEAN);
    let crown_base_y = params.trunk_height_m;
    let segments = CONIFER_LOD2_SEGMENTS;
    let mut foliage_parts = Vec::new();
    let last = params.tiers.len() - 1;
    for tier_index in 0..2 {
        let table_index = (tier_index * (last / 2)).min(last);
        let y0 = crown_base_y + params.tiers[table_index] * params.crown_height_m;
        let y1 = crown_base_y + params.tiers[(table_index + 1).min(last)] * params.crown_height_m;
        let r0 = params.crown_radius_m * params.tier_radii[table_index];
        let r1 = params.crown_radius_m
            * params.tier_radii[(table_index + 1).min(last)]
            * if tier_index == 0 { 1.0 } else { 0.5 };
        foliage_parts.push(conifer_tier(
            y0,
            y1.max(y0 + 0.5),
            r0,
            r1.max(0.05),
            0.25 * r0,
            segments,
            darken(color, tier_index),
        ));
    }
    // The upper tier tapers to a near-tip: no separate apex cone at LOD2,
    // keeping the pine silhouette within the far LOD triangle budget.
    let bark = assemble(&[trunk]).expect("conifer LOD2 bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("conifer LOD2 foliage mesh is valid");
    VegetationLod { bark, foliage }
}

/// Build one deciduous asset at all three LODs.
fn build_deciduous_variant(variant: u8, name: &'static str) -> VegetationAsset {
    let params = deciduous_params(variant);
    let lod0 = deciduous_lod(
        &params,
        params.blobs.len(),
        DECIDUOUS_LOD0_SEGMENTS,
        DECIDUOUS_LOD0_STACKS,
        true,
    );
    let lod1 = deciduous_lod(
        &params,
        DECIDUOUS_LOD1_BLOBS.min(params.blobs.len()),
        DECIDUOUS_LOD1_SEGMENTS,
        DECIDUOUS_LOD1_STACKS,
        false,
    );
    let lod2 = deciduous_lod2(&params);
    let (center, radius, height) = bounds_from_lod0(&lod0);
    VegetationAsset {
        species: VegetationSpecies::Deciduous,
        variant,
        name,
        lods: VegetationLodSet { lod0, lod1, lod2 },
        bounds_center: center,
        bounds_radius: radius,
        height_m: height,
    }
}

/// Build one conifer asset at all three LODs.
fn build_conifer_variant(variant: u8, name: &'static str) -> VegetationAsset {
    let params = conifer_params(variant);
    let lod0 = conifer_lod(&params, params.tiers.len(), CONIFER_LOD0_SEGMENTS);
    let lod1 = conifer_lod(&params, CONIFER_LOD1_TIERS, CONIFER_LOD1_SEGMENTS);
    let lod2 = conifer_lod2(&params);
    let (center, radius, height) = bounds_from_lod0(&lod0);
    VegetationAsset {
        species: VegetationSpecies::Conifer,
        variant,
        name,
        lods: VegetationLodSet { lod0, lod1, lod2 },
        bounds_center: center,
        bounds_radius: radius,
        height_m: height,
    }
}

/// Tight bounding sphere (unit scale) from the LOD0 mesh.
fn bounds_from_lod0(lod: &VegetationLod) -> ([f32; 3], f32, f32) {
    let mut max_height = 0.0f32;
    let mut max_horizontal = 0.0f32;
    let mut max_corner: Option<[f32; 3]> = None;
    for vertex in lod.bark.vertices().iter().chain(lod.foliage.vertices()) {
        let horizontal = (vertex.position[0] * vertex.position[0]
            + vertex.position[2] * vertex.position[2])
            .sqrt();
        max_horizontal = max_horizontal.max(horizontal);
        if vertex.position[1] > max_height {
            max_height = vertex.position[1];
            max_corner = Some(vertex.position);
        }
    }
    // Sphere centre: slightly above mid-trunk so the crown dominates the
    // cull test while the trunk stays inside.
    let center_y = max_height * 0.42;
    let mut bounds_radius = (max_horizontal * max_horizontal + center_y * center_y).sqrt();
    // Tighten against the actual farthest vertex from the chosen centre.
    for vertex in lod.bark.vertices().iter().chain(lod.foliage.vertices()) {
        let dx = vertex.position[0];
        let dy = vertex.position[1] - center_y;
        let dz = vertex.position[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();
        bounds_radius = bounds_radius.max(distance);
    }
    let _ = max_corner;
    ([0.0, center_y, 0.0], bounds_radius, max_height)
}

// ── GLB export (deterministic) ─────────────────────────────────────────────

fn align4(offset: usize) -> usize {
    (offset + 3) & !3
}

/// Deterministic GLB export of one asset's LOD0 (bark + foliage primitives
/// with distinct PBR materials). The byte stream is a pure function of the
/// asset, so the committed files under `models/assets/scenery` are
/// reproducible. Written with serde_json's ordered `Map`, so identical input
/// yields identical bytes.
#[must_use]
pub fn export_glb(asset: &VegetationAsset) -> Vec<u8> {
    let parts = [
        (VegetationPart::Bark, &asset.lods.lod0.bark),
        (VegetationPart::Foliage, &asset.lods.lod0.foliage),
    ];
    let mut bin = Vec::new();
    let mut buffer_views = Vec::new();
    let mut accessors = Vec::new();
    let mut meshes = Vec::new();

    for (part, mesh) in parts {
        let vertices = mesh.vertices();
        let indices = mesh.indices();
        let vertex_start = bin.len();
        for vertex in vertices {
            for &component in &vertex.position {
                bin.extend_from_slice(&component.to_le_bytes());
            }
            for &component in &vertex.normal {
                bin.extend_from_slice(&component.to_le_bytes());
            }
            for &component in &vertex.color {
                bin.extend_from_slice(&component.to_le_bytes());
            }
            // Pad to the 48-byte stride declared on the bufferView, or the
            // loader would read vertex i at byte 48*i while the data was
            // packed at 40*i and run past the view end.
            bin.extend_from_slice(&[0; 8]);
        }
        let index_start = align4(bin.len());
        bin.resize(index_start, 0);
        for &index in indices {
            bin.extend_from_slice(&index.to_le_bytes());
        }

        let mut global_min = [f32::MAX; 3];
        let mut global_max = [f32::MIN; 3];
        for vertex in vertices {
            for axis in 0..3 {
                global_min[axis] = global_min[axis].min(vertex.position[axis]);
                global_max[axis] = global_max[axis].max(vertex.position[axis]);
            }
        }

        let vertex_view = buffer_views.len();
        buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": vertex_start,
            "byteLength": index_start - vertex_start,
            "byteStride": 48,
            "target": 34962,
        }));
        let index_view = buffer_views.len();
        buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": index_start,
            "byteLength": bin.len() - index_start,
            "target": 34963,
        }));

        let position_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": vertex_view,
            "byteOffset": 0,
            "componentType": 5126,
            "count": vertices.len(),
            "type": "VEC3",
            "min": global_min,
            "max": global_max,
        }));
        let normal_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": vertex_view,
            "byteOffset": 12,
            "componentType": 5126,
            "count": vertices.len(),
            "type": "VEC3",
        }));
        let color_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": vertex_view,
            "byteOffset": 24,
            "componentType": 5126,
            "count": vertices.len(),
            "type": "VEC4",
        }));
        let index_accessor = accessors.len();
        accessors.push(json!({
            "bufferView": index_view,
            "byteOffset": 0,
            "componentType": 5125,
            "count": indices.len(),
            "type": "SCALAR",
        }));

        let material_index = part.index();
        meshes.push(json!({
            "name": match part {
                VegetationPart::Bark => "bark",
                VegetationPart::Foliage => "foliage",
            },
            "primitives": [{
                "attributes": {
                    "POSITION": position_accessor,
                    "NORMAL": normal_accessor,
                    "COLOR_0": color_accessor,
                },
                "indices": index_accessor,
                "material": material_index,
                "mode": 4,
            }],
        }));
    }

    // Four-part buffer; bin may carry trailing alignment padding.
    let materials = vec![
        json!({
            "pbrMetallicRoughness": {
                "baseColorFactor": [1.0, 1.0, 1.0, 1.0],
                "metallicFactor": part_metallic(VegetationPart::Bark),
                "roughnessFactor": part_roughness(VegetationPart::Bark),
            }
        }),
        json!({
            "pbrMetallicRoughness": {
                "baseColorFactor": [1.0, 1.0, 1.0, 1.0],
                "metallicFactor": part_metallic(VegetationPart::Foliage),
                "roughnessFactor": part_roughness(VegetationPart::Foliage),
            }
        }),
    ];

    let root = json!({
        "asset": {
            "generator": "rc-simulation-engine G3D tree asset generator",
            "version": "2.0",
        },
        "scene": 0,
        "scenes": [{ "nodes": [0, 1] }],
        "nodes": [{ "mesh": 0 }, { "mesh": 1 }],
        "meshes": meshes,
        "materials": materials,
        "buffers": [{ "byteLength": bin.len() }],
        "bufferViews": buffer_views,
        "accessors": accessors,
    });

    // Serialize as a Map (object) with sorted keys => byte-deterministic.
    let mut json_bytes = serde_json::to_vec(&root).expect("tree GLB JSON serializes");
    while !json_bytes.len().is_multiple_of(4) {
        json_bytes.push(b' ');
    }
    let mut glb = Vec::new();
    let total_length = 12 + 8 + json_bytes.len() + 8 + align4(bin.len());
    glb.extend_from_slice(&0x4654_6C67_u32.to_le_bytes()); // "glTF"
    glb.extend_from_slice(&2_u32.to_le_bytes());
    glb.extend_from_slice(&(total_length as u32).to_le_bytes());
    glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x4E4F_534A_u32.to_le_bytes()); // "JSON"
    glb.extend_from_slice(&json_bytes);
    let bin_length = align4(bin.len());
    bin.resize(bin_length, 0);
    glb.extend_from_slice(&(bin_length as u32).to_le_bytes());
    glb.extend_from_slice(&0x004E_4942_u32.to_le_bytes()); // "BIN\0"
    glb.extend_from_slice(&bin);
    glb
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn production_asset_set() -> VegetationAssetSet {
        VegetationAssetSet::production()
    }

    #[test]
    fn production_set_has_two_species_and_two_variants_each() {
        let set = production_asset_set();
        assert_eq!(set.len(), 4);
        let mut deciduous = 0;
        let mut conifer = 0;
        for asset in set.assets() {
            match asset.species {
                VegetationSpecies::Deciduous => deciduous += 1,
                VegetationSpecies::Conifer => conifer += 1,
            }
        }
        assert!(deciduous >= VARIANTS_PER_SPECIES_TARGET);
        assert!(conifer >= VARIANTS_PER_SPECIES_TARGET);
        assert_eq!(
            set.assets().len(),
            SPECIES_TARGET * VARIANTS_PER_SPECIES_TARGET
        );
    }

    #[test]
    fn every_lod_mesh_is_valid_and_finite() {
        for asset in production_asset_set().assets() {
            for class in 0..3_u8 {
                let lod = asset.lods.lod(class).expect("lod class exists");
                for (part, mesh) in [
                    (VegetationPart::Bark, &lod.bark),
                    (VegetationPart::Foliage, &lod.foliage),
                ] {
                    assert!(
                        !mesh.vertices().is_empty(),
                        "{0}/lod{class}/{part:?} has no vertices",
                        asset.name
                    );
                    assert!(
                        !mesh.indices().is_empty(),
                        "{0}/lod{class}/{part:?} has no indices",
                        asset.name
                    );
                    assert!(mesh.indices().len().is_multiple_of(3));
                    let index_max = mesh.indices().iter().copied().max().unwrap_or(0) as usize;
                    assert!(index_max < mesh.vertices().len());
                }
            }
        }
    }

    #[test]
    fn normals_are_unit_length_and_positions_finite() {
        for asset in production_asset_set().assets() {
            for class in 0..3_u8 {
                let lod = asset.lods.lod(class).unwrap();
                for mesh in [&lod.bark, &lod.foliage] {
                    for vertex in mesh.vertices() {
                        assert!(vertex.position.into_iter().all(f32::is_finite));
                        let length = (vertex.normal[0].powi(2)
                            + vertex.normal[1].powi(2)
                            + vertex.normal[2].powi(2))
                        .sqrt();
                        assert!(
                            (length - 1.0).abs() < 1.0e-3,
                            "{}: normal {:?} length {length}",
                            asset.name,
                            vertex.normal
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn trunk_and_canopy_materials_are_distinct() {
        for asset in production_asset_set().assets() {
            for class in 0..3_u8 {
                let lod = asset.lods.lod(class).unwrap();
                let bark = &lod.bark;
                let foliage = &lod.foliage;
                // Distinct part identities imply distinct materials; verify
                // the PBR factors actually differ (roughness) and that bark
                // colors are brown-dominant while foliage is green-dominant.
                assert_ne!(
                    part_roughness(VegetationPart::Bark),
                    part_roughness(VegetationPart::Foliage)
                );
                let bark_g = bark.vertices().iter().map(|v| v.color[1]).sum::<f32>()
                    / bark.vertices().len() as f32;
                let bark_r = bark.vertices().iter().map(|v| v.color[0]).sum::<f32>()
                    / bark.vertices().len() as f32;
                assert!(
                    bark_r > bark_g,
                    "{} bark should be brown-dominant",
                    asset.name
                );
                let foliage_g = foliage.vertices().iter().map(|v| v.color[1]).sum::<f32>()
                    / foliage.vertices().len() as f32;
                let foliage_r = foliage.vertices().iter().map(|v| v.color[0]).sum::<f32>()
                    / foliage.vertices().len() as f32;
                assert!(
                    foliage_g > foliage_r,
                    "{} foliage should be green-dominant",
                    asset.name
                );
            }
        }
    }

    #[test]
    fn lod_triangle_counts_reduce_substantially_and_monotonically() {
        for asset in production_asset_set().assets() {
            let lod0 = asset.lods.triangle_count(0);
            let lod1 = asset.lods.triangle_count(1);
            let lod2 = asset.lods.triangle_count(2);
            assert!(
                lod0 > 0 && lod1 > 0 && lod2 > 0,
                "{}: all LODs must have geometry",
                asset.name
            );
            assert!(
                lod1 < lod0,
                "{}: LOD1 must be cheaper than LOD0",
                asset.name
            );
            assert!(
                lod2 < lod1,
                "{}: LOD2 must be cheaper than LOD1",
                asset.name
            );
            let lod1_ratio = lod1 as f32 / lod0 as f32;
            let lod2_ratio = lod2 as f32 / lod0 as f32;
            assert!(
                (0.35..=0.60).contains(&lod1_ratio),
                "{}: LOD1 ratio {lod1_ratio:.2} outside 35-60%",
                asset.name
            );
            assert!(
                (0.08..=0.28).contains(&lod2_ratio),
                "{}: LOD2 ratio {lod2_ratio:.2} outside 8-28%",
                asset.name
            );
        }
    }

    #[test]
    fn lod0_silhouette_is_not_a_single_cone_or_cylinder() {
        // The old placeholders were a single cone/cylinder per tree. A real
        // tree must have a non-trivial silhouette: verify the canopy has many
        // more triangles than any single cone shell would, and that foliage
        // vertices spread over multiple distinct horizontal extents/rings.
        for asset in production_asset_set().assets() {
            let foliage = &asset.lods.lod0.foliage;
            let horizontal_extents = foliage
                .vertices()
                .iter()
                .filter_map(|v| {
                    let h = (v.position[0].powi(2) + v.position[2].powi(2)).sqrt();
                    (h > 0.05).then_some(h)
                })
                .fold(f32::NEG_INFINITY, f32::max);
            // Foliage must extend beyond the trunk radius (a trunk-only or
            // cone-at-trunk-range canopy would fail this).
            assert!(
                horizontal_extents > 1.2,
                "{} LOD0 foliage must have a real canopy extent, got {horizontal_extents}",
                asset.name
            );
            let canopy_tris = foliage.indices().len() / 3;
            assert!(
                canopy_tris >= 80,
                "{} LOD0 canopy must not be a trivial shell, got {canopy_tris} tris",
                asset.name
            );
        }
    }

    #[test]
    fn tree_heights_are_plausible_and_bounds_are_finite() {
        for asset in production_asset_set().assets() {
            assert!(
                (4.0..=9.0).contains(&asset.height_m),
                "{} height {}",
                asset.name,
                asset.height_m
            );
            assert!(asset.bounds_radius.is_finite() && asset.bounds_radius > 1.5);
            assert!(asset.bounds_center.iter().all(|v| v.is_finite()));
            // The bounding sphere must actually contain every LOD0 vertex.
            for mesh in [&asset.lods.lod0.bark, &asset.lods.lod0.foliage] {
                for vertex in mesh.vertices() {
                    let dx = vertex.position[0] - asset.bounds_center[0];
                    let dy = vertex.position[1] - asset.bounds_center[1];
                    let dz = vertex.position[2] - asset.bounds_center[2];
                    let distance = (dx * dx + dy * dy + dz * dz).sqrt();
                    let tolerance = asset.bounds_radius * 1.01 + 1.0e-3;
                    assert!(
                        distance <= tolerance,
                        "{}: vertex at {distance:.2} exceeds bounds radius {}",
                        asset.name,
                        asset.bounds_radius
                    );
                }
            }
        }
    }

    #[test]
    fn assets_within_each_species_are_not_bit_identical() {
        let set = production_asset_set();
        for species in [VegetationSpecies::Deciduous, VegetationSpecies::Conifer] {
            let variants: Vec<_> = set
                .assets()
                .iter()
                .filter(|a| a.species == species)
                .collect();
            assert!(variants.len() >= 2);
            assert_ne!(
                variants[0].lods.lod0.foliage.indices(),
                variants[1].lods.lod0.foliage.indices(),
                "{species:?} variants must differ in geometry"
            );
            assert_ne!(
                variants[0].height_m, variants[1].height_m,
                "{species:?} variants must differ in proportion"
            );
        }
    }

    #[test]
    fn deterministic_generation_is_reproducible() {
        let a = VegetationAssetSet::production();
        let b = VegetationAssetSet::production();
        for (left, right) in a.assets().iter().zip(b.assets()) {
            assert_eq!(left.name, right.name);
            assert_eq!(left.bounds_radius.to_bits(), right.bounds_radius.to_bits());
            assert_eq!(
                left.lods.lod0.foliage.indices(),
                right.lods.lod0.foliage.indices()
            );
            assert_eq!(
                left.lods.lod0.bark.vertices(),
                right.lods.lod0.bark.vertices()
            );
        }
    }

    #[test]
    fn glb_export_is_deterministic_and_valid_for_every_asset() {
        let set = production_asset_set();
        for asset in set.assets() {
            let bytes = export_glb(asset);
            assert!(!bytes.is_empty());
            assert_eq!(&bytes[0..4], b"glTF");
            assert_eq!(&bytes[4..8], &2_u32.to_le_bytes());
            // Byte-determinism: exporting twice yields identical bytes.
            assert_eq!(bytes, export_glb(asset));
            // File size sanity: a tree GLB is a few tens of kilobytes.
            assert!(bytes.len() > 1_000 && bytes.len() < 500_000);
        }
    }

    #[test]
    fn glb_round_trips_through_the_render_glb_loader() {
        use crate::glb::load_glb_asset;
        let set = production_asset_set();
        for asset in set.assets() {
            let bytes = export_glb(asset);
            let path = std::env::temp_dir().join(format!(
                "g3d_tree_roundtrip_{}_{}.glb",
                asset.name,
                std::process::id()
            ));
            std::fs::write(&path, &bytes).expect("write temp GLB");
            let loaded = load_glb_asset(&path).expect("generated GLB must load");
            assert_eq!(loaded.primitives.len(), 2, "{}: bark + foliage", asset.name);
            for (part_index, primitive) in loaded.primitives.iter().enumerate() {
                let part = match part_index {
                    0 => VegetationPart::Bark,
                    1 => VegetationPart::Foliage,
                    _ => unreachable!(),
                };
                let expected = match part {
                    VegetationPart::Bark => &asset.lods.lod0.bark,
                    VegetationPart::Foliage => &asset.lods.lod0.foliage,
                };
                assert_eq!(primitive.vertices.len(), expected.vertices().len());
                assert_eq!(primitive.indices, expected.indices());
                assert!(
                    primitive
                        .indices
                        .iter()
                        .all(|&i| (i as usize) < primitive.vertices.len())
                );
                assert_eq!(primitive.material.metallic_factor, part_metallic(part));
                assert!(
                    (primitive.material.roughness_factor - part_roughness(part)).abs() < 1.0e-6
                );
            }
            let _ = std::fs::remove_file(&path);
        }
    }

    #[test]
    fn gpu_instance_relevant_meshes_are_cheap_enough_for_an_rc_field() {
        // Budget guard: LOD0 across the production set must stay comfortably
        // below what a 500 m RC field with a few hundred instanced trees
        // needs on a 3090 (a few million tris/second are trivial for the GPU).
        let set = production_asset_set();
        let lod0_tris: usize = set.assets().iter().map(|a| a.lods.triangle_count(0)).sum();
        assert!(
            lod0_tris < 4_000,
            "production LOD0 total {lod0_tris} tris too high"
        );
    }
}
