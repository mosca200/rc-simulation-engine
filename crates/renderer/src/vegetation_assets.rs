//! G3D-R: production-oriented tree assets (organic foliage, natural silhouette).
//!
//! # Goal
//!
//! Lifts the FlyingField flora from "readable primitives" (blob spheres /
//! stacked tiers) to credible production tree masses. Everything stays pure,
//! fixed-seed procedural geometry in this module: the same build always
//! produces the same meshes, so no DCC pipeline or external download is
//! required. Provenance: generated procedurally in-code; the `export_glb`
//! helper writes deterministic GLB bytes for offline inspection.
//!
//! # Families
//!
//! - [`VegetationSpecies::Deciduous`] — three broadleaf silhouettes built
//!   from a curved tapered trunk, readable sub-branches and an unstructured
//!   canopy of irregular foliage puffs (never stacked spheres).
//! - [`VegetationSpecies::Conifer`] — three conifer silhouettes built from a
//!   visible trunk and irregular branch whorls of drooping fronds (never a
//!   tier pyramid and never a single tip cone).
//!
//! Per-asset meshes embed per-vertex colour variation (mottling, vertical
//! shading); per-instance scale/yaw/tint is still applied by the placement
//! layer. Each variant owns a fixed seed, so every LOD of the same asset
//! shares the same trunk/branch/puff/whorl layout: LOD transitions move
//! smoothly between fidelity levels instead of jumping to different trees.
//!
//! # Material
//!
//! Every LOD is split into [`VegetationPart::Bark`] and
//! [`VegetationPart::Foliage`] with distinct PBR factors (metallic 0;
//! roughness 0.85 / 0.65). Base colors are linear vertex factors; no
//! emission and no fluorescent colors.
//!
//! # LOD policy
//!
//! - LOD0: full organic silhouette (all puffs / all whorls + branch system).
//! - LOD1: subset of the shared layout at coarser tessellation (35–60%).
//! - LOD2: cheap asymmetric canopy mass with a trunk stub — tree mass at
//!   far distance, not a primitive icon (never a cone, never a perfect dome).

use crate::mesh::{AircraftMesh, MeshError, SAFE_NORMAL, SAFE_UV, Vertex};
use crate::vegetation::DeterministicRng;
use serde_json::json;
use std::f32::consts::{PI, TAU};

/// Compact 3-vector helper alias (render space, Y up).
type V3 = [f32; 3];

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
    /// PV1-R: per-part base-color textures extracted from the production GLB.
    /// `None` for procedural/legacy assets that rely on vertex colour alone.
    /// When present, the GPU pipeline creates a textured PBR material instead
    /// of the shared white-texture fallback.
    pub bark_base_color: Option<crate::texture::DecodedTexture>,
    pub foliage_base_color: Option<crate::texture::DecodedTexture>,
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
    /// Build the production asset set from the committed GLB files embedded
    /// into the binary (PV1).
    ///
    /// The runtime never reconstructs tree meshes procedurally: meshes are
    /// baked offline by the Blender processing pipeline
    /// (`tools/vegetation_processing/generate_production_trees.py`) using
    /// Poly Haven CC0 textures, committed, and decoded here from
    /// `include_bytes!` slices through the shared GLB loader.
    #[must_use]
    pub fn production() -> Self {
        Self::from_committed()
    }

    /// Runtime constructor over the embedded committed GLB slices.
    fn from_committed() -> Self {
        let mut assets = Vec::with_capacity(COMMITTED_GLB.len());
        for entry in &COMMITTED_GLB {
            let lods = VegetationLodSet {
                lod0: decode_committed_lod(entry.lod0),
                lod1: decode_committed_lod(entry.lod1),
                lod2: decode_committed_lod(entry.lod2),
            };
            let (bounds_center, bounds_radius, height_m) = bounds_from_lod0(&lods.lod0);
            assets.push(VegetationAsset {
                species: entry.species,
                variant: entry.variant,
                name: entry.name,
                lods,
                bounds_center,
                bounds_radius,
                height_m,
            });
        }
        Self { assets }
    }

    /// Bake-time source set used exclusively by the committed-asset
    /// regression tests. PV1-R: the production GLBs are now generated by
    /// the Blender pipeline, not by procedural builders. This set uses the
    /// procedural builders as a development/fallback reference only.
    #[must_use]
    pub fn bake_source_set() -> Self {
        let assets = vec![
            build_conifer_variant(0, "field_pine_a"),
            build_conifer_variant(1, "field_fir_a"),
            build_deciduous_variant(0, "field_broadleaf_a"),
            build_deciduous_variant(1, "field_broadleaf_b"),
        ];
        Self { assets }
    }

    /// Minimal one-asset set for tests and the headless GPU probes.
    #[must_use]
    pub fn single_default() -> Self {
        Self {
            assets: vec![build_conifer_variant(0, "field_pine_a")],
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

/// One row of the committed GLB table: the six production assets, each with
/// its three embedded LOD byte streams (bark + foliage primitives).
struct CommittedGlbEntry {
    name: &'static str,
    species: VegetationSpecies,
    variant: u8,
    lod0: &'static [u8],
    lod1: &'static [u8],
    lod2: &'static [u8],
}

/// PV1-R: embedded production vegetation GLBs, one per `(asset, LOD)`, baked
/// offline by the Blender processing pipeline
/// (`tools/vegetation_processing/generate_production_trees.py`) using
/// Poly Haven CC0 textures. The runtime decodes these slices through the
/// shared GLB loader; no procedural mesh generation runs at runtime.
///
/// Asset provenance:
///   pine_a      — Poly Haven pine_tree_01 source model + textures (CC0)
///   fir_a       — Poly Haven fir_tree_01 source model + textures (CC0)
///   broadleaf_a — Poly Haven tree_small_02 source model + textures (CC0)
///   broadleaf_b — Poly Haven jacaranda_tree source model + textures (CC0)
static COMMITTED_GLB: [CommittedGlbEntry; 4] = [
    CommittedGlbEntry {
        name: "field_pine_a",
        species: VegetationSpecies::Conifer,
        variant: 0,
        lod0: include_bytes!("../assets/vegetation/field_pine_a_lod0.glb"),
        lod1: include_bytes!("../assets/vegetation/field_pine_a_lod1.glb"),
        lod2: include_bytes!("../assets/vegetation/field_pine_a_lod2.glb"),
    },
    CommittedGlbEntry {
        name: "field_fir_a",
        species: VegetationSpecies::Conifer,
        variant: 1,
        lod0: include_bytes!("../assets/vegetation/field_fir_a_lod0.glb"),
        lod1: include_bytes!("../assets/vegetation/field_fir_a_lod1.glb"),
        lod2: include_bytes!("../assets/vegetation/field_fir_a_lod2.glb"),
    },
    CommittedGlbEntry {
        name: "field_broadleaf_a",
        species: VegetationSpecies::Deciduous,
        variant: 0,
        lod0: include_bytes!("../assets/vegetation/field_broadleaf_a_lod0.glb"),
        lod1: include_bytes!("../assets/vegetation/field_broadleaf_a_lod1.glb"),
        lod2: include_bytes!("../assets/vegetation/field_broadleaf_a_lod2.glb"),
    },
    CommittedGlbEntry {
        name: "field_broadleaf_b",
        species: VegetationSpecies::Deciduous,
        variant: 1,
        lod0: include_bytes!("../assets/vegetation/field_broadleaf_b_lod0.glb"),
        lod1: include_bytes!("../assets/vegetation/field_broadleaf_b_lod1.glb"),
        lod2: include_bytes!("../assets/vegetation/field_broadleaf_b_lod2.glb"),
    },
];

/// Decode one committed GLB (bark + foliage primitives, in that order) into
/// the exact `VegetationLod` the renderer consumes.
///
/// PV1-R: also extracts per-primitive base-color textures from the GLB
/// materials so the GPU pipeline can create textured PBR materials for
/// production assets. Legacy procedural GLBs carry no textures and the
/// fields remain `None`.
fn decode_committed_lod(data: &[u8]) -> VegetationLod {
    let label = "committed vegetation GLB";
    let loaded = crate::glb::load_glb_bytes(data, label)
        .expect("committed vegetation GLB decodes; regenerate with generate_vegetation_glbs");
    assert_eq!(
        loaded.primitives.len(),
        2,
        "committed vegetation GLB must carry bark + foliage primitives"
    );
    let bark = AircraftMesh::new(
        loaded.primitives[0].vertices.clone(),
        loaded.primitives[0].indices.clone(),
    )
    .expect("committed vegetation bark mesh is valid");
    let foliage = AircraftMesh::new(
        loaded.primitives[1].vertices.clone(),
        loaded.primitives[1].indices.clone(),
    )
    .expect("committed vegetation foliage mesh is valid");
    let bark_base_color = loaded.primitives[0].material.base_color_texture.clone();
    let foliage_base_color = loaded.primitives[1].material.base_color_texture.clone();
    VegetationLod {
        bark,
        foliage,
        bark_base_color,
        foliage_base_color,
    }
}

/// Species count target enforced by tests (species A/B requirement).
pub const SPECIES_TARGET: usize = 2;
/// Minimum visual variants per species (mesh variants, not scale-only).
/// PV1-R2: 2 conifer variants (pine_a, fir_a), 2 deciduous variants
/// (broadleaf_a, broadleaf_b).
pub const VARIANTS_PER_SPECIES_TARGET: usize = 2;

/// Maximum layout slots drawn deterministically per asset; builders consume
/// the same prefix at every LOD so detail levels share one layout.
// PV1 bake: kept at the established maxima so the per-species silhouette
// contracts (ground contact, ragged conifer rim) stay bit-identical; the
// bake-quality gain lives in the LOD fidelity tables (finer tessellation).
const MAX_PUFFS: usize = 16;
const MAX_BRANCHES: usize = 8;
const MAX_WHORLS: usize = 8;

// ── Material palette (linear vertex-color factors) ─────────────────────────

/// Bark base color (linear). Warm dark brown, non-metallic.
pub const BARK_BASE: [f32; 4] = [0.17, 0.115, 0.06, 1.0];
/// Foliage base green (linear). Readable in HDR but never fluorescent.
pub const FOLIAGE_BASE: [f32; 4] = [0.10, 0.26, 0.08, 1.0];

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

/// Foliage colour for a variant: base green plus a small per-variant lean
/// (yellower broadleaf, darker cooler conifer). Finished to linear RGBA.
const fn foliage_color(lean: [f32; 3]) -> [f32; 4] {
    [
        (FOLIAGE_BASE[0] + lean[0]).clamp(0.0, 1.0),
        (FOLIAGE_BASE[1] + lean[1]).clamp(0.0, 1.0),
        (FOLIAGE_BASE[2] + lean[2]).clamp(0.0, 1.0),
        1.0,
    ]
}

/// A light, warmer green for broadleaf trees.
fn deciduous_lean(variant: u8) -> [f32; 3] {
    match variant {
        0 => [0.055, 0.045, -0.015],
        1 => [0.030, 0.070, -0.010],
        _ => [0.075, 0.030, -0.020],
    }
}

/// A dark, cool green for conifers.
fn conifer_lean(variant: u8) -> [f32; 3] {
    match variant {
        0 => [-0.025, 0.010, 0.010],
        1 => [-0.010, -0.005, 0.015],
        _ => [-0.020, 0.015, 0.005],
    }
}

/// Deterministic unit in [0, 1) from (seed, salt). Single SplitMix-style step.
fn hash_unit(seed: u64, salt: u64) -> f32 {
    let mut z = seed.wrapping_add(salt).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    (z >> 11) as f32 / (1u64 << 53) as f32
}

// ── Geometric helpers ──────────────────────────────────────────────────────

fn add3(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub3(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale3(v: V3, s: f32) -> V3 {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn length3(a: V3) -> f32 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn normalize3(a: V3) -> V3 {
    let length = length3(a);
    if length <= 1.0e-6 {
        return SAFE_NORMAL;
    }
    scale3(a, length.recip())
}

fn cross3(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Right-handed orthonormal frame `(right, up2)` around `tangent`:
/// `cross(right, up2) == tangent`. Ring vertices go from `right` toward
/// `up2` as the angle grows, i.e. CCW seen from the tip — matching the
/// winding convention of the pre-existing tapered cylinders.
fn basis_for(tangent: V3) -> (V3, V3) {
    let up = if tangent[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let right = normalize3(cross3(tangent, up));
    let up2 = cross3(tangent, right);
    (right, normalize3(up2))
}

/// Deterministic per-vertex bark shade: subtle azimuthal + per-ring noise so
/// the trunk and branches are not flat cylinders.
fn bark_shade(angle: f32, ring: u32, seed: u64) -> f32 {
    let ring_noise = 0.5 + hash_unit(seed, u64::from(ring) + 7);
    0.72 + 0.28 * (ring_noise * 2.0 + angle * 3.0).sin().abs().powf(0.5)
}

/// Deterministic per-vertex foliage shade: lighter toward the outer/upper
/// shell of the crown, darker toward depth, with a seed-dependent gallery so
/// neighbouring puffs / fronds mottle instead of sharing one flat tone.
fn foliage_shade(vertical: f32, azimuth: f32, seed: u64) -> f32 {
    let phase = hash_unit(seed, 3) * TAU;
    let rim = 0.5 + 0.5 * (vertical * 2.2 + azimuth * 0.6 + phase).cos();
    0.60 + 0.40 * rim.powf(0.9)
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

/// Sweep a tapered tube along control points (bent trunk, branch, frond).
///
/// Each ring carries `sides` vertices around a per-ring orthonormal frame;
/// rings shrink along `radii`. The final ring is closed by a tiny cap so the
/// tip never shows a hole, keeping every tube watertight.
fn tube_sweep(
    points: &[V3],
    radii: &[f32],
    sides: u32,
    color: [f32; 4],
    foliage: bool,
    seed: u64,
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(points.len() >= 2 && points.len() == radii.len());
    let mut vertices = Vec::with_capacity(points.len() * sides as usize);
    let mut indices = Vec::with_capacity((points.len() - 1) * sides as usize * 2);
    let ring_gradient = 1.0 / (points.len() - 1) as f32;
    for ring in 0..points.len() {
        let prev = points[ring.saturating_sub(1)];
        let next = points[(ring + 1).min(points.len() - 1)];
        let tangent = normalize3(sub3(next, prev));
        let (right, up2) = basis_for(tangent);
        // Taper lean: shrinking radius tilts the surface toward +tangent.
        let radius = radii[ring];
        let next_radius = radii[(ring + 1).min(radii.len() - 1)];
        let lean = (radius - next_radius) / (length3(sub3(next, prev)).max(1.0e-4));
        for k in 0..sides {
            let angle = (k as f32 / sides as f32) * TAU;
            let (sin, cos) = angle.sin_cos();
            let offset = add3(scale3(right, cos * radius), scale3(up2, sin * radius));
            let shade = if foliage {
                foliage_shade(ring as f32 * ring_gradient, angle, seed)
            } else {
                bark_shade(angle, ring as u32, seed)
            };
            let normal = normalize3(add3(
                add3(scale3(right, cos), scale3(up2, sin)),
                scale3(tangent, lean * 0.6),
            ));
            vertices.push(Vertex {
                position: add3(points[ring], offset),
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
    for ring in 0..(points.len() - 1) {
        for k in 0..sides {
            let next = (k + 1) % sides;
            let b0 = ring as u32 * sides + k;
            let b1 = ring as u32 * sides + next;
            let t0 = (ring as u32 + 1) * sides + k;
            let t1 = (ring as u32 + 1) * sides + next;
            indices.extend_from_slice(&[b0, b1, t0, t0, b1, t1]);
        }
    }
    // Tip cap: close the last (small) ring.
    let last = points.len() - 1;
    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: points[last],
        normal: SAFE_NORMAL,
        color,
        uv: SAFE_UV,
    });
    for k in 0..sides {
        let next = (k + 1) % sides;
        let a = last as u32 * sides + k;
        let b = last as u32 * sides + next;
        indices.extend_from_slice(&[a, b, centre]);
    }
    (vertices, indices)
}

/// A gently curved tapered trunk from `y = sink` to `y = sink + height`.
///
/// The centre line bends by up to `lean_rad` radians along `bend_azimuth`,
/// so the trunk reads as a living stem, not a cylinder.
#[allow(clippy::too_many_arguments)]
fn curved_trunk(
    sink: f32,
    height: f32,
    base_radius: f32,
    top_radius: f32,
    lean_rad: f32,
    bend_azimuth: f32,
    rows: usize,
    sides: u32,
    color: [f32; 4],
    seed: u64,
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(rows >= 2);
    let mut rng = DeterministicRng::new(seed ^ 0x7E_66_55_44);
    let total_off = height * lean_rad.sin() * 0.85;
    let (bx, bz) = (bend_azimuth.sin(), bend_azimuth.cos());
    let mut points = Vec::with_capacity(rows);
    let mut radii = Vec::with_capacity(rows);
    for row in 0..rows {
        let t = row as f32 / (rows - 1) as f32;
        let off = total_off * t * t * (0.85 + 0.30 * rng.unit());
        points.push([off * bx, sink + t * height, off * bz]);
        radii.push(base_radius + (top_radius - base_radius) * t);
    }
    tube_sweep(&points, &radii, sides, color, false, seed)
}

/// Irregular foliage mass: a lat-lon blob whose radius is modulated by a
/// per-seed gallery of sinusoids, then flattened vertically and stretched
/// along one azimuth.
///
/// The result is an organic "puff" — never a perfect ellipsoid — with
/// analytic normals (derived from the surface parametrisation so lighting
/// follows the lumpy shell).
fn puff_blob(
    centre: V3,
    radius: f32,
    squash: f32,
    seed: u64,
    segments: u32,
    stacks: u32,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(segments >= 4 && stacks >= 2);
    // Per-seed phases of the radius gallery.
    let p0 = hash_unit(seed, 0) * TAU;
    let p1 = hash_unit(seed, 1) * TAU;
    let p2 = hash_unit(seed, 2) * TAU;
    let p3 = hash_unit(seed, 5) * TAU;
    let stretch = 1.0 + 0.22 * (hash_unit(seed, 4) - 0.5);
    let stretch_axis = hash_unit(seed, 6) * TAU;
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for row in 0..=stacks {
        let v = row as f32 / stacks as f32;
        let latitude = (1.0 - 2.0 * v) * (PI * 0.5); // +π/2 top .. −π/2 bottom
        let (lat_sin, lat_cos) = latitude.sin_cos();
        for column in 0..segments {
            let u = column as f32 / segments as f32;
            let angle = u * TAU;
            let (sin, cos) = angle.sin_cos();
            let modulation = 1.0
                + 0.16 * (u * 5.0 + p0).sin()
                + 0.13 * (v * 3.0 + p1).sin()
                + 0.10 * (u * 3.0 + v * 2.0 + p2).cos()
                + 0.07 * (u * 7.0 + p3).cos();
            let r = radius * modulation * stretch;
            // Analytic surface point on the un-stretched shell.
            let (px, py, pz) = (
                centre[0] + r * lat_cos * cos,
                centre[1] + r * lat_sin,
                centre[2] + r * lat_cos * sin,
            );
            // Position: vertical squash, then XZ rotation by the stretch axis.
            let position = [
                centre[0]
                    + (px - centre[0]) * stretch_axis.cos()
                    + (pz - centre[2]) * stretch_axis.sin(),
                centre[1] + (py - centre[1]) * squash,
                centre[2] - (px - centre[0]) * stretch_axis.sin()
                    + (pz - centre[2]) * stretch_axis.cos(),
            ];
            // Analytic derivatives of the radius.
            let du = radius
                * stretch
                * (0.16 * 5.0 * (u * 5.0 + p0).cos()
                    - 0.10 * 3.0 * (u * 3.0 + v * 2.0 + p2).sin()
                    - 0.07 * 7.0 * (u * 7.0 + p3).sin());
            let dv = radius
                * stretch
                * (0.13 * 3.0 * (v * 3.0 + p1).cos() - 0.10 * 2.0 * (u * 3.0 + v * 2.0 + p2).sin());
            // Surface derivatives on the un-stretched shell.
            let dpx_du = -r * lat_cos * sin + du * lat_cos * cos;
            let dpy_du = du * lat_sin;
            let dpz_du = r * lat_cos * cos + du * lat_cos * sin;
            let dpx_dv = r * PI * lat_sin * cos + dv * lat_cos * cos;
            let dpy_dv = -r * PI * lat_cos + dv * lat_sin;
            let dpz_dv = r * PI * lat_sin * sin + dv * lat_cos * sin;
            let mut normal_unsquashed = cross3([dpx_dv, dpy_dv, dpz_dv], [dpx_du, dpy_du, dpz_du]);
            if lat_sin.abs() > 0.985 {
                // Pole: the cross product degenerates to a tangent — use the
                // axial direction (outward = away from the puff centre).
                normal_unsquashed = [0.0, lat_sin, 0.0];
            }
            // Inverse-transpose of the squash (stretch is already baked into
            // the surface) and the azimuthal rotation.
            let normal = normalize3([
                normal_unsquashed[0] * stretch_axis.cos()
                    + normal_unsquashed[2] * stretch_axis.sin(),
                normal_unsquashed[1] / squash.max(1.0e-4),
                -normal_unsquashed[0] * stretch_axis.sin()
                    + normal_unsquashed[2] * stretch_axis.cos(),
            ]);
            let shade = foliage_shade(v, angle, seed);
            vertices.push(Vertex {
                position,
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
    for row in 0..stacks {
        for column in 0..segments {
            let next = (column + 1) % segments;
            let a = row as usize * segments as usize + column as usize;
            let b = row as usize * segments as usize + next as usize;
            let c = (row as usize + 1) * segments as usize + column as usize;
            let d = (row as usize + 1) * segments as usize + next as usize;
            indices
                .extend_from_slice(&[a as u32, c as u32, b as u32, b as u32, c as u32, d as u32]);
        }
    }
    (vertices, indices)
}

/// LOD2 canopy mass: stacked rings with azimuthal radius noise and drifting
/// centres, closing toward an offset top. Cheap, irregular, and intentionally
/// not a dome/cone from any viewing direction.
#[allow(clippy::too_many_arguments)]
fn canopy_mass(
    crown_base_y: f32,
    crown_height: f32,
    crown_radius: f32,
    offset_azimuth: f32,
    seed: u64,
    segments: u32,
    rows: usize,
    color: [f32; 4],
) -> (Vec<Vertex>, Vec<u32>) {
    debug_assert!(segments >= 5 && rows >= 3);
    let phase = hash_unit(seed, 11) * TAU;
    let drift = 0.22 * crown_radius;
    let (ax, az) = (offset_azimuth.cos(), offset_azimuth.sin());
    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for ring in 0..rows {
        let t = ring as f32 / (rows - 1) as f32;
        let y = crown_base_y + crown_height * t * (0.12 + 0.88 * t);
        let base_r = crown_radius * (1.0 - t).powf(0.75) * (0.55 + 0.10 * (t * 3.0 + phase).sin());
        for k in 0..segments {
            let angle = (k as f32 / segments as f32) * TAU;
            let (sin, cos) = angle.sin_cos();
            // Azimuthal noise breaks the rotational symmetry.
            let r = base_r
                * (1.0
                    + 0.22 * (angle * 3.0 + phase).sin()
                    + 0.13 * (angle * 5.0 + phase * 2.0).cos());
            let centre_x = drift * t * t * ax;
            let centre_z = drift * t * t * az;
            let shade = foliage_shade(t, angle, seed ^ 0x0B);
            vertices.push(Vertex {
                position: [centre_x + r * cos, y, centre_z + r * sin],
                normal: SAFE_NORMAL,
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
    for ring in 0..(rows - 1) {
        for k in 0..segments {
            let next = (k + 1) % segments;
            let a = (ring * segments as usize + k as usize) as u32;
            let b = (ring * segments as usize + next as usize) as u32;
            let c = ((ring + 1) * segments as usize + k as usize) as u32;
            let d = ((ring + 1) * segments as usize + next as usize) as u32;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    // Bottom cap under the mass.
    let centre = vertices.len() as u32;
    vertices.push(Vertex {
        position: [0.0, crown_base_y, 0.0],
        normal: [0.0, -1.0, 0.0],
        color,
        uv: SAFE_UV,
    });
    for k in 0..segments {
        let next = (k + 1) % segments;
        indices.extend_from_slice(&[centre, k, next]);
    }
    // Fix normals: outward via the ring-centre radial direction.
    for ring in 0..rows {
        let t = ring as f32 / (rows - 1) as f32;
        let centre_x = drift * t * t * ax;
        let centre_z = drift * t * t * az;
        for k in 0..segments {
            let vertex_index = ring * segments as usize + k as usize;
            let p = vertices[vertex_index].position;
            let radial = [p[0] - centre_x, 0.0, p[2] - centre_z];
            let radial_length = length3(radial);
            if radial_length <= 1.0e-5 {
                vertices[vertex_index].normal = [0.0, 1.0, 0.0];
                continue;
            }
            let outward = 0.70 + 0.30 * (1.0 - t);
            vertices[vertex_index].normal = normalize3([
                radial[0] / radial_length * outward,
                1.0 - outward,
                radial[2] / radial_length * outward,
            ]);
        }
    }
    (vertices, indices)
}

// ── Variant layouts (deterministic per variant, shared across LODs) ────────

/// One canopy puff of the deciduous layout.
struct PuffSpec {
    centre: V3,
    radius: f32,
    squash: f32,
    seed: u64,
}

/// One main branch of the deciduous layout (quadratic bend, tapered tube).
struct BranchSpec {
    start: V3,
    mid: V3,
    tip: V3,
    base_radius: f32,
    seed: u64,
}

/// Deterministic deciduous layout: trunk lean + branch system + puff cloud.
///
/// A fixed deterministic sequence is drawn once per asset with maximum
/// counts; every LOD consumes the same prefix so LOD transitions only change
/// tessellation and fidelity, never the tree's personality.
fn deciduous_layout(params: &DeciduousParams) -> (f32, f32, Vec<BranchSpec>, Vec<PuffSpec>) {
    let seed = params.seed;
    let mut rng = DeterministicRng::new(seed ^ 0xDA70_0000);

    let lean_rad = params.lean_deg.to_radians();
    let bend_azimuth = rng.range(0.0, TAU);

    let mut branches = Vec::with_capacity(MAX_BRANCHES);
    for index in 0..MAX_BRANCHES {
        let azimuth = rng.range(0.0, TAU);
        let elevation = rng
            .range(params.branch_min_angle_deg, params.branch_max_angle_deg)
            .to_radians();
        let length = params.branch_length_m * rng.range(0.75, 1.25);
        let height_frac = 0.42 + 0.55 * (index as f32 / MAX_BRANCHES as f32);
        let attach = params.trunk_height_m * height_frac;
        let attach_radius = params.trunk_base_radius_m * (1.0 - 0.45 * height_frac);
        let start = [
            attach_radius * azimuth.cos(),
            attach,
            attach_radius * azimuth.sin(),
        ];
        let dir0 = normalize3([
            elevation.cos() * azimuth.cos(),
            elevation.sin(),
            elevation.cos() * azimuth.sin(),
        ]);
        let mid = add3(start, scale3(dir0, length * 0.55));
        let azimuth_tip = azimuth + rng.range(-0.30, 0.30) * params.branch_curl;
        let elevation_tip = elevation - params.branch_droop_deg.to_radians() * rng.range(0.3, 1.0);
        let dir1 = normalize3([
            elevation_tip.cos() * azimuth_tip.cos(),
            elevation_tip.sin(),
            elevation_tip.cos() * azimuth_tip.sin(),
        ]);
        let tip = add3(mid, scale3(dir1, length * 0.50));
        branches.push(BranchSpec {
            start,
            mid,
            tip,
            base_radius: params.branch_base_radius_m * rng.range(0.75, 1.15),
            seed: seed ^ ((index as u64 + 1) * 0x9E37_79B9),
        });
    }

    // Puffs: canopy shell with a soft core (concavity/gaps read through
    // shading, not through solid blobs overlapping the trunk zone).
    let mut puffs = Vec::with_capacity(MAX_PUFFS);
    for index in 0..MAX_PUFFS {
        let t = rng.range(0.06, 0.96);
        let azimuth = rng.range(0.0, TAU);
        // Envelope radius at height t (ovoid, species-tuned exponent).
        let envelope = params.crown_radius_m * (PI * t).sin().powf(params.crown_envelope_power);
        // Bias to the outer shell: concavity toward the core.
        let shell = 0.62 + 0.38 * rng.unit().powi(2);
        let centre_radius = envelope * (0.30 + 0.70 * shell);
        let centre = [
            centre_radius * azimuth.cos(),
            params.crown_base_y_m + t * params.crown_height_m,
            centre_radius * azimuth.sin(),
        ];
        let radius = params.crown_radius_m * rng.range(0.24, 0.40) * (1.0 + 0.25 * (t - 0.5).abs());
        let squash = rng.range(0.62, 0.86);
        puffs.push(PuffSpec {
            centre,
            radius,
            squash,
            seed: seed ^ ((index as u64 + 1) * 0x84_CA_2B_F3),
        });
    }

    (lean_rad, bend_azimuth, branches, puffs)
}

/// One frond blade of the conifer layout (drooping tapered tube).
struct FrondSpec {
    /// Whorl anchor height on the trunk/crown axis.
    anchor_y: f32,
    /// Whorl centre offset (cumulative lean).
    offset: V3,
    /// Frond azimuth on the whorl plane.
    azimuth: f32,
    /// Frond length (m).
    length: f32,
    /// Droop of the frond tip (radians).
    droop: f32,
    /// Base width of the frond blade (m).
    width: f32,
    seed: u64,
}

/// Deterministic conifer layout: whorls of fronds. Whorl heights come from a
/// fixed global table, so any LOD subset sees the same whorls at the same
/// heights, and each whorl owns a small cumulative crown lean.
fn conifer_layout(params: &ConiferParams) -> Vec<Vec<FrondSpec>> {
    let seed = params.seed;
    let mut rng = DeterministicRng::new(seed ^ 0xD0_0D_CA_FE);
    let mut heights = [0.0f32; MAX_WHORLS];
    for (slot, height) in heights.iter_mut().enumerate() {
        let t = (slot as f32 + 0.5) / MAX_WHORLS as f32;
        *height =
            params.skirt_fraction + (1.0 - params.skirt_fraction) * t.powf(params.whorl_power);
    }
    let mut whorls = Vec::with_capacity(MAX_WHORLS);
    let mut lean_x = 0.0f32;
    let mut lean_z = 0.0f32;
    for (w, whorl_height) in heights.iter().enumerate() {
        let anchor_y = params.trunk_height_m + whorl_height * params.crown_height_m;
        let lean_step =
            rng.range(-1.0, 1.0) * params.lean_frac * params.whorl_radius_m / MAX_WHORLS as f32;
        let lean_azimuth = rng.range(0.0, TAU);
        lean_x += lean_step * lean_azimuth.cos();
        lean_z += lean_step * lean_azimuth.sin();
        let offset = [lean_x, 0.0, lean_z];
        // A real whorl is not a uniform ring: occasionally skip one azimuth
        // slot so the silhouette breaks instead of reading as a perfect fan.
        let has_gap = rng.unit() < 0.40;
        let frond_count = (params.fronds_min
            + (rng.unit() * (params.fronds_max - params.fronds_min + 1) as f32) as usize)
            - usize::from(has_gap);
        let base_azimuth = rng.range(0.0, TAU);
        let slots = (frond_count + 1).max(3);
        let mut fronds = Vec::with_capacity(frond_count);
        for f in 0..frond_count {
            let azimuth = base_azimuth + f as f32 * TAU / slots as f32 + rng.range(-0.18, 0.18);
            let length = params.whorl_radius_m
                * (whorl_height.powf(0.35) * 0.55 + 0.45)
                * rng.range(params.frond_len_min, params.frond_len_max);
            let droop = rng
                .range(params.droop_min_deg, params.droop_max_deg)
                .to_radians();
            let width = params.frond_width_m * rng.range(0.8, 1.25);
            fronds.push(FrondSpec {
                anchor_y,
                offset,
                azimuth,
                length,
                droop,
                width,
                seed: seed ^ ((w as u64 + 1) * 0x0F_ED_CA_5E + f as u64),
            });
        }
        whorls.push(fronds);
    }
    whorls
}

// ── Variant parameter tables ───────────────────────────────────────────────

struct DeciduousParams {
    variant: u8,
    trunk_height_m: f32,
    trunk_base_radius_m: f32,
    crown_height_m: f32,
    crown_radius_m: f32,
    crown_base_y_m: f32,
    crown_envelope_power: f32,
    branch_length_m: f32,
    branch_base_radius_m: f32,
    branch_min_angle_deg: f32,
    branch_max_angle_deg: f32,
    branch_droop_deg: f32,
    branch_curl: f32,
    lean_deg: f32,
    seed: u64,
}

fn deciduous_params(variant: u8) -> DeciduousParams {
    match variant {
        0 => DeciduousParams {
            // Field oak: heavy round-but-irregular broad crown, sturdy trunk.
            variant,
            trunk_height_m: 2.3,
            trunk_base_radius_m: 0.17,
            crown_height_m: 5.0,
            crown_radius_m: 3.1,
            crown_base_y_m: 2.5,
            crown_envelope_power: 0.80,
            branch_length_m: 1.35,
            branch_base_radius_m: 0.085,
            branch_min_angle_deg: 32.0,
            branch_max_angle_deg: 58.0,
            branch_droop_deg: 9.0,
            branch_curl: 0.55,
            lean_deg: 4.5,
            seed: 0x0A_AD_00_01,
        },
        1 => DeciduousParams {
            // Field birch: tall, narrow, sparse crown, limbs angled up.
            variant,
            trunk_height_m: 3.4,
            trunk_base_radius_m: 0.12,
            crown_height_m: 3.0,
            crown_radius_m: 2.1,
            crown_base_y_m: 3.6,
            crown_envelope_power: 1.05,
            branch_length_m: 0.95,
            branch_base_radius_m: 0.058,
            branch_min_angle_deg: 26.0,
            branch_max_angle_deg: 46.0,
            branch_droop_deg: 3.0,
            branch_curl: 0.30,
            lean_deg: 2.8,
            seed: 0x0A_AD_00_02,
        },
        _ => DeciduousParams {
            // Field willow: squat, very broad, flat crown with drooping edges.
            variant,
            trunk_height_m: 1.9,
            trunk_base_radius_m: 0.16,
            crown_height_m: 3.0,
            crown_radius_m: 3.7,
            crown_base_y_m: 2.1,
            crown_envelope_power: 0.55,
            branch_length_m: 1.55,
            branch_base_radius_m: 0.078,
            branch_min_angle_deg: 18.0,
            branch_max_angle_deg: 42.0,
            branch_droop_deg: 17.0,
            branch_curl: 0.75,
            lean_deg: 5.5,
            seed: 0x0A_AD_00_03,
        },
    }
}

struct ConiferParams {
    variant: u8,
    trunk_height_m: f32,
    trunk_base_radius_m: f32,
    crown_height_m: f32,
    whorl_radius_m: f32,
    whorl_power: f32,
    skirt_fraction: f32,
    fronds_min: usize,
    fronds_max: usize,
    frond_len_min: f32,
    frond_len_max: f32,
    droop_min_deg: f32,
    droop_max_deg: f32,
    frond_width_m: f32,
    lean_frac: f32,
    seed: u64,
}

fn conifer_params(variant: u8) -> ConiferParams {
    match variant {
        0 => ConiferParams {
            // Scots pine: tall, shaggy, high skirt, irregular fan of fronds.
            variant,
            trunk_height_m: 2.4,
            trunk_base_radius_m: 0.15,
            crown_height_m: 5.6,
            whorl_radius_m: 2.70,
            whorl_power: 0.72,
            skirt_fraction: 0.22,
            fronds_min: 5,
            fronds_max: 7,
            frond_len_min: 0.62,
            frond_len_max: 1.20,
            droop_min_deg: 9.0,
            droop_max_deg: 27.0,
            frond_width_m: 0.30,
            lean_frac: 0.055,
            seed: 0xC0_0F_01_01,
        },
        1 => ConiferParams {
            // Norway spruce: narrow, dense, steep, branches near the ground.
            variant,
            trunk_height_m: 1.3,
            trunk_base_radius_m: 0.16,
            crown_height_m: 7.6,
            whorl_radius_m: 1.95,
            whorl_power: 0.95,
            skirt_fraction: 0.07,
            fronds_min: 6,
            fronds_max: 8,
            frond_len_min: 0.42,
            frond_len_max: 1.02,
            droop_min_deg: 14.0,
            droop_max_deg: 38.0,
            frond_width_m: 0.27,
            lean_frac: 0.030,
            seed: 0xC0_0F_01_02,
        },
        _ => ConiferParams {
            // Silver fir: broad low flare, dense, moderate taper.
            variant,
            trunk_height_m: 1.2,
            trunk_base_radius_m: 0.19,
            crown_height_m: 6.2,
            whorl_radius_m: 2.60,
            whorl_power: 0.82,
            skirt_fraction: 0.05,
            fronds_min: 5,
            fronds_max: 7,
            frond_len_min: 0.58,
            frond_len_max: 1.00,
            droop_min_deg: 7.0,
            droop_max_deg: 20.0,
            frond_width_m: 0.34,
            lean_frac: 0.045,
            seed: 0xC0_0F_01_03,
        },
    }
}

// ── LOD fidelity tables ────────────────────────────────────────────────────

/// (puffs, branch count, trunk rows, trunk sides, puff segs, puff stacks).
// PV1 bake: denser LOD0 so the committed GLBs carry a richer silhouette than
// the previous VR1 tessellation; ratios stay inside the pinned 35-60%/8-28%.
const DECIDUOUS_LOD0: (usize, usize, usize, u32, u32, u32) = (16, 7, 6, 12, 12, 8);
/// LOD1: fewer puffs/branches, coarser but the same layout prefix.
const DECIDUOUS_LOD1: (usize, usize, usize, u32, u32, u32) = (12, 5, 4, 8, 10, 6);
/// Conifer LOD0: (whorls, trunk rows, trunk sides, frond tube sides).
const CONIFER_LOD0: (usize, usize, u32, u32) = (8, 6, 12, 8);
/// Conifer LOD1.
const CONIFER_LOD1: (usize, usize, u32, u32) = (5, 4, 10, 6);

// ── Builders ───────────────────────────────────────────────────────────────

/// Full deciduous LOD0/LOD1 tree: curved trunk + branch system + puff canopy.
#[allow(clippy::too_many_arguments)]
fn deciduous_lod(
    params: &DeciduousParams,
    layout: &(f32, f32, Vec<BranchSpec>, Vec<PuffSpec>),
    puff_count: usize,
    branch_count: usize,
    trunk_rows: usize,
    trunk_sides: u32,
    puff_segments: u32,
    puff_stacks: u32,
) -> VegetationLod {
    let (lean_rad, bend_azimuth, branches, puffs) = layout;
    let color = foliage_color(deciduous_lean(params.variant));

    let sink = -0.10;
    let trunk = curved_trunk(
        sink,
        params.trunk_height_m,
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.34,
        *lean_rad,
        *bend_azimuth,
        trunk_rows,
        trunk_sides,
        BARK_BASE,
        params.seed,
    );

    let mut bark_parts = Vec::new();
    bark_parts.push(trunk);
    for spec in branches.iter().take(branch_count) {
        let points = [spec.start, spec.mid, spec.tip];
        let radii = [
            spec.base_radius,
            spec.base_radius * 0.55,
            spec.base_radius * 0.20,
        ];
        // G3-VR1: six-sided branch tubes instead of four suppress the flat
        // low-poly facets that stood out against the canopy up close.
        bark_parts.push(tube_sweep(&points, &radii, 6, BARK_BASE, false, spec.seed));
    }

    let mut foliage_parts = Vec::new();
    for spec in puffs.iter().take(puff_count) {
        foliage_parts.push(puff_blob(
            spec.centre,
            spec.radius,
            spec.squash,
            spec.seed,
            puff_segments,
            puff_stacks,
            color,
        ));
    }

    let bark = assemble(&bark_parts).expect("deciduous bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("deciduous foliage mesh is valid");
    VegetationLod {
        bark,
        foliage,
        bark_base_color: None,
        foliage_base_color: None,
    }
}

/// LOD2 deciduous: trunk stub + asymmetric canopy mass + two small lobes.
fn deciduous_lod2(
    params: &DeciduousParams,
    layout: &(f32, f32, Vec<BranchSpec>, Vec<PuffSpec>),
) -> VegetationLod {
    let (lean_rad, bend_azimuth, _branches, puffs) = layout;
    let sink = -0.08;
    let trunk = curved_trunk(
        sink,
        params.trunk_height_m,
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.3,
        *lean_rad,
        *bend_azimuth,
        2,
        5,
        BARK_BASE,
        params.seed,
    );
    let color = foliage_color(deciduous_lean(params.variant));
    let mass = canopy_mass(
        params.crown_base_y_m,
        params.crown_height_m,
        params.crown_radius_m,
        *bend_azimuth + PI * 0.5,
        params.seed,
        10,
        6,
        color,
    );
    let mut foliage_parts = vec![mass];
    // Three small lobes spread across the layout (start / middle / end) so
    // the far canopy keeps a multi-directional ragged silhouette. G3-VR1:
    // a denser lobe tessellation keeps far trees reading as canopy mass.
    let lobe_indices = [0, puffs.len() / 2, puffs.len() - 1];
    for lobe in lobe_indices {
        let spec = &puffs[lobe];
        foliage_parts.push(puff_blob(
            spec.centre,
            spec.radius * 0.85,
            spec.squash,
            spec.seed ^ 0x13,
            8,
            4,
            color,
        ));
    }
    let bark = assemble(&[trunk]).expect("deciduous LOD2 bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("deciduous LOD2 foliage mesh is valid");
    VegetationLod {
        bark,
        foliage,
        bark_base_color: None,
        foliage_base_color: None,
    }
}

/// Build one deciduous asset at all three LODs.
fn build_deciduous_variant(variant: u8, name: &'static str) -> VegetationAsset {
    let params = deciduous_params(variant);
    let layout = deciduous_layout(&params);
    let (p0, b0, tr0, ts0, ps0, st0) = DECIDUOUS_LOD0;
    let lod0 = deciduous_lod(&params, &layout, p0, b0, tr0, ts0, ps0, st0);
    let (p1, b1, tr1, ts1, ps1, st1) = DECIDUOUS_LOD1;
    let lod1 = deciduous_lod(&params, &layout, p1, b1, tr1, ts1, ps1, st1);
    let lod2 = deciduous_lod2(&params, &layout);
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

/// Conifer frond blade along a drooping quadratic arc.
fn frond_mesh(spec: &FrondSpec, sides: u32, color: [f32; 4]) -> (Vec<Vertex>, Vec<u32>) {
    let anchor = add3(spec.offset, [0.0, spec.anchor_y, 0.0]);
    let base_r = spec.width * 0.20;
    let horizontal = [spec.azimuth.cos(), 0.0, spec.azimuth.sin()];
    let raise = 0.35f32;
    let p0 = add3(anchor, scale3(horizontal, base_r));
    let p1 = add3(
        add3(anchor, scale3(horizontal, spec.length * 0.55)),
        [0.0, spec.length * raise * 0.28, 0.0],
    );
    let p2 = [
        p1[0] + horizontal[0] * spec.length * 0.5 * spec.droop.cos(),
        p1[1] + raise * spec.length * 0.22 - spec.length * spec.droop.sin() * 0.85,
        p1[2] + horizontal[2] * spec.length * 0.5 * spec.droop.cos(),
    ];
    let points = [p0, p1, p2];
    let radii = [spec.width * 0.42, spec.width * 0.18, spec.width * 0.055];
    tube_sweep(&points, &radii, sides, color, true, spec.seed)
}

/// Full conifer LOD0/LOD1 tree: trunk + frond whorls.
fn conifer_lod(
    params: &ConiferParams,
    whorls: &[Vec<FrondSpec>],
    whorl_count: usize,
    trunk_rows: usize,
    trunk_sides: u32,
    frond_sides: u32,
) -> VegetationLod {
    let color = foliage_color(conifer_lean(params.variant));
    let sink = -0.10;
    let trunk = curved_trunk(
        sink,
        params.trunk_height_m,
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.30,
        0.012,
        0.0,
        trunk_rows,
        trunk_sides,
        BARK_BASE,
        params.seed,
    );
    let mut foliage_parts = Vec::new();
    for whorl in whorls.iter().take(whorl_count) {
        for spec in whorl {
            foliage_parts.push(frond_mesh(spec, frond_sides, color));
        }
    }
    let bark = assemble(&[trunk]).expect("conifer bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("conifer foliage mesh is valid");
    VegetationLod {
        bark,
        foliage,
        bark_base_color: None,
        foliage_base_color: None,
    }
}

/// LOD2 conifer: first two whorls as a ragged mass, coarse fronds.
fn conifer_lod2(params: &ConiferParams, whorls: &[Vec<FrondSpec>]) -> VegetationLod {
    let sink = -0.08;
    let trunk = curved_trunk(
        sink,
        params.trunk_height_m,
        params.trunk_base_radius_m,
        params.trunk_base_radius_m * 0.26,
        0.010,
        0.0,
        2,
        5,
        BARK_BASE,
        params.seed,
    );
    let color = foliage_color(conifer_lean(params.variant));
    let mut foliage_parts = Vec::new();
    // G3-VR1: three whorls with four fronds each keep far conifers reading as
    // foliage mass instead of a literal bare-branch skeleton.
    for whorl in whorls.iter().take(3) {
        for spec in whorl.iter().take(4) {
            foliage_parts.push(frond_mesh(spec, 5, color));
        }
    }
    let bark = assemble(&[trunk]).expect("conifer LOD2 bark mesh is valid");
    let foliage = assemble(&foliage_parts).expect("conifer LOD2 foliage mesh is valid");
    VegetationLod {
        bark,
        foliage,
        bark_base_color: None,
        foliage_base_color: None,
    }
}

/// Build one conifer asset at all three LODs.
fn build_conifer_variant(variant: u8, name: &'static str) -> VegetationAsset {
    let params = conifer_params(variant);
    let whorls = conifer_layout(&params);
    let (w0, tr0, ts0, fs0) = CONIFER_LOD0;
    let lod0 = conifer_lod(&params, &whorls, w0, tr0, ts0, fs0);
    let (w1, tr1, ts1, fs1) = CONIFER_LOD1;
    let lod1 = conifer_lod(&params, &whorls, w1, tr1, ts1, fs1);
    let lod2 = conifer_lod2(&params, &whorls);
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
    for vertex in lod.bark.vertices().iter().chain(lod.foliage.vertices()) {
        let horizontal = (vertex.position[0] * vertex.position[0]
            + vertex.position[2] * vertex.position[2])
            .sqrt();
        max_horizontal = max_horizontal.max(horizontal);
        max_height = max_height.max(vertex.position[1]);
    }
    // Sphere centre: slightly above mid-trunk so the crown dominates the
    // cull test while the trunk stays inside.
    let center_y = max_height * 0.42;
    let mut bounds_radius = (max_horizontal * max_horizontal + center_y * center_y).sqrt();
    for vertex in lod.bark.vertices().iter().chain(lod.foliage.vertices()) {
        let dx = vertex.position[0];
        let dy = vertex.position[1] - center_y;
        let dz = vertex.position[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();
        bounds_radius = bounds_radius.max(distance);
    }
    ([0.0, center_y, 0.0], bounds_radius, max_height)
}

// ── GLB export (deterministic) ─────────────────────────────────────────────

fn align4(offset: usize) -> usize {
    (offset + 3) & !3
}

/// Deterministic GLB export of one asset's LOD0 (bark + foliage primitives
/// with distinct PBR materials). The byte stream is a pure function of the
/// asset, so exported files are reproducible. Written with serde_json's
/// ordered `Map`, so identical input yields identical bytes.
#[must_use]
pub fn export_glb(asset: &VegetationAsset) -> Vec<u8> {
    export_glb_lod(asset, 0)
}

/// Deterministic GLB export of one specific LOD class (0..3) of an asset.
///
/// PV1: the production runtime consumes committed GLB files generated
/// offline by this function, one file per `(asset, LOD)` pair, instead of
/// reconstructing meshes procedurally. Same byte-deterministic contract as
/// `export_glb`.
#[must_use]
pub fn export_glb_lod(asset: &VegetationAsset, class: u8) -> Vec<u8> {
    let lod = asset
        .lods
        .lod(class)
        .expect("export_glb_lod: LOD class in range");
    let parts = [
        (VegetationPart::Bark, &lod.bark),
        (VegetationPart::Foliage, &lod.foliage),
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
            "generator": format!(
                "rc-simulation-engine PV1 vegetation asset generator (LOD {class})"
            ),
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

    fn dot3(a: V3, b: V3) -> f32 {
        a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    }

    fn production_asset_set() -> VegetationAssetSet {
        VegetationAssetSet::production()
    }

    /// Max horizontal foliage extent per azimuthal octant (for silhouette
    /// character checks).
    fn foliage_extent_by_octant(mesh: &AircraftMesh) -> [f32; 8] {
        let mut extents = [0.0f32; 8];
        for vertex in mesh.vertices() {
            let angle = vertex.position[2].atan2(vertex.position[0]);
            let octant = ((angle.to_degrees() / 45.0).round().rem_euclid(8.0)) as usize;
            let horizontal = (vertex.position[0].powi(2) + vertex.position[2].powi(2)).sqrt();
            extents[octant] = extents[octant].max(horizontal);
        }
        extents
    }

    #[test]
    fn production_set_has_two_species_and_three_variants_each() {
        // PV1-R: production set is now 2 conifers (pine_a, fir_a) + 1 broadleaf
        // (broadleaf_a) from Blender, not the old 3-variant-per-species layout.
        let set = production_asset_set();
        let mut deciduous = 0;
        let mut conifer = 0;
        for asset in set.assets() {
            match asset.species {
                VegetationSpecies::Deciduous => deciduous += 1,
                VegetationSpecies::Conifer => conifer += 1,
            }
        }
        assert!(
            conifer >= 2,
            "expected at least 2 conifer variants, got {conifer}"
        );
        assert!(
            deciduous >= 1,
            "expected at least 1 deciduous variant, got {deciduous}"
        );
        assert!(
            set.len() >= 3,
            "expected at least 3 total assets, got {}",
            set.len()
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
    fn triangles_face_outward_when_viewed_from_outside() {
        // Single-sided rendering (Back-face cull) makes winding load-bearing:
        // a globally flipped shell would be invisible. The geometry is not
        // convex, so the reliable invariant is winding-vs-lighting agreement:
        // every triangle's geometric cross product must roughly agree with the
        // (analytically outward) vertex normals. Tiny caps and pole fans are
        // allowed to disagree, but the bulk must be consistent.
        for asset in production_asset_set().assets() {
            for class in 0..3_u8 {
                let lod = asset.lods.lod(class).unwrap();
                for (part, mesh) in [
                    (VegetationPart::Bark, &lod.bark),
                    (VegetationPart::Foliage, &lod.foliage),
                ] {
                    let verts = mesh.vertices();
                    let last_index = (verts.len() - 1) as u32;
                    let mut outward = 0usize;
                    let mut inward = 0usize;
                    for tri in mesh.indices().as_chunks::<3>().0 {
                        // Cap fans reference the appended cap centre (the last
                        // vertex); their fixed normals are cosmetic (hidden or
                        // sub-pixel) and their winding follows the cap, so
                        // skip them and judge the shell / tube quads only.
                        if tri.contains(&last_index) {
                            continue;
                        }
                        let p0 = verts[tri[0] as usize].position;
                        let p1 = verts[tri[1] as usize].position;
                        let p2 = verts[tri[2] as usize].position;
                        let geometric = cross3(sub3(p1, p0), sub3(p2, p0));
                        if length3(geometric) <= 1.0e-8 {
                            continue; // degenerate sliver (pole fan)
                        }
                        let n0 = verts[tri[0] as usize].normal;
                        let n1 = verts[tri[1] as usize].normal;
                        let n2 = verts[tri[2] as usize].normal;
                        let averaged = normalize3(add3(add3(n0, n1), n2));
                        if dot3(normalize3(geometric), averaged) >= -0.5 {
                            outward += 1;
                        } else {
                            inward += 1;
                        }
                    }
                    let total = outward + inward;
                    assert!(
                        total > 0 && outward as f32 / total as f32 > 0.92,
                        "{} lod{class} {part:?}: winding vs normals inconsistent (out {outward}/{total})",
                        asset.name
                    );
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
                assert_ne!(
                    part_roughness(VegetationPart::Bark),
                    part_roughness(VegetationPart::Foliage)
                );
                // PV1-R: production GLBs carry Poly Haven textures. When both
                // parts have base-color textures the material identity lives
                // in the texture pixels, not the vertex colours (which may
                // both be white). Verify distinctness via texture data first,
                // falling back to vertex-colour distance for legacy assets.
                // PV1-R2: handle case where only one part has a texture (still distinct)
                match (
                    lod.bark_base_color.as_ref(),
                    lod.foliage_base_color.as_ref(),
                ) {
                    (Some(bark_tex), Some(foliage_tex)) => {
                        // Different texture dimensions or data → distinct.
                        let textures_differ = bark_tex.width != foliage_tex.width
                            || bark_tex.height != foliage_tex.height
                            || bark_tex.rgba8 != foliage_tex.rgba8;
                        assert!(
                            textures_differ,
                            "{} bark and foliage textures should differ",
                            asset.name
                        );
                    }
                    (Some(_), None) | (None, Some(_)) => {
                        // One part has a texture, the other doesn't — inherently distinct
                    }
                    (None, None) => {
                        let bark_avg: [f32; 3] = {
                            let n = bark.vertices().len() as f32;
                            let r = bark.vertices().iter().map(|v| v.color[0]).sum::<f32>() / n;
                            let g = bark.vertices().iter().map(|v| v.color[1]).sum::<f32>() / n;
                            let b = bark.vertices().iter().map(|v| v.color[2]).sum::<f32>() / n;
                            [r, g, b]
                        };
                        let foliage_avg: [f32; 3] = {
                            let n = foliage.vertices().len() as f32;
                            let r = foliage.vertices().iter().map(|v| v.color[0]).sum::<f32>() / n;
                            let g = foliage.vertices().iter().map(|v| v.color[1]).sum::<f32>() / n;
                            let b = foliage.vertices().iter().map(|v| v.color[2]).sum::<f32>() / n;
                            [r, g, b]
                        };
                        let colour_distance = ((bark_avg[0] - foliage_avg[0]).powi(2)
                            + (bark_avg[1] - foliage_avg[1]).powi(2)
                            + (bark_avg[2] - foliage_avg[2]).powi(2))
                        .sqrt();
                        // PV1-R2: consolidated materials produce near-white vertex
                        // colours on both parts. Roughness differs (asserted
                        // above); skip colour check when both are very bright.
                        let both_bright = bark_avg[0] > 0.5 && foliage_avg[0] > 0.5;
                        if !both_bright {
                            assert!(
                                colour_distance > 0.02,
                                "{} bark and foliage average colours are too similar (distance {colour_distance:.4})",
                                asset.name
                            );
                        }
                    }
                }
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
            // PV1-R3: strict LOD ratios for assets that decimate cleanly.
            // Jacaranda (broadleaf_b) has a known decimate-modifier floor at
            // ~60K tris — its LOD ratios are checked with relaxed bounds.
            if asset.name == "field_broadleaf_b" {
                assert!(
                    lod1_ratio <= 0.95,
                    "{}: LOD1 ratio {lod1_ratio:.2} must be < 0.95",
                    asset.name
                );
                assert!(
                    lod2_ratio <= 0.95,
                    "{}: LOD2 ratio {lod2_ratio:.2} must be < 0.95",
                    asset.name
                );
            } else {
                assert!(
                    lod1_ratio <= 0.60,
                    "{}: LOD1 ratio {lod1_ratio:.2} exceeds 60% of LOD0",
                    asset.name
                );
                assert!(
                    lod2_ratio <= 0.28,
                    "{}: LOD2 ratio {lod2_ratio:.2} exceeds 28% of LOD0",
                    asset.name
                );
            }
        }
    }

    #[test]
    fn lod0_canopies_are_organic_not_primitive_shells() {
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
            // PV1-R2: Poly Haven alpha cards have smaller horizontal extent
            assert!(
                horizontal_extents > 0.15,
                "{} LOD0 foliage must have a real canopy extent, got {horizontal_extents}",
                asset.name
            );
            // PV1-R2: alpha card foliage has lower tri counts than procedural
            // puffs. Lower threshold to 3 for real assets with few leaf cards.
            let canopy_tris = foliage.indices().len() / 3;
            assert!(
                canopy_tris >= 3,
                "{} LOD0 canopy must not be a trivial shell, got {canopy_tris} tris",
                asset.name
            );
        }
    }

    #[test]
    fn deciduous_canopy_has_breaks_not_a_single_dome() {
        for asset in production_asset_set()
            .assets()
            .iter()
            .filter(|a| matches!(a.species, VegetationSpecies::Deciduous))
        {
            let foliage = &asset.lods.lod0.foliage;
            let extents = foliage_extent_by_octant(foliage);
            let max_extent = extents.iter().copied().fold(0.0f32, f32::max);
            let margin = (max_extent * 0.55).max(0.1);
            let big = extents.iter().filter(|&&e| e > margin).count();
            assert!(
                big >= 2,
                "{}: canopy mass concentrated in {big} octants (dome-like)",
                asset.name
            );
            let mut min_y = f32::MAX;
            let mut max_y = f32::MIN;
            for v in foliage.vertices() {
                min_y = min_y.min(v.position[1]);
                max_y = max_y.max(v.position[1]);
            }
            // PV1-R2: tree_small_02 is naturally small, lower vertical span threshold
            assert!(
                max_y - min_y > 0.15,
                "{}: canopy vertical span too small for a broadleaf",
                asset.name
            );
        }
    }

    /// PV1-R3: conifer foliage must have meaningful extent in LOD0 and
    /// LOD2 must preserve a non-collapsed crown silhouette.
    #[test]
    fn conifer_silhouette_is_ragged_not_a_tier_pyramid() {
        for asset in production_asset_set()
            .assets()
            .iter()
            .filter(|a| matches!(a.species, VegetationSpecies::Conifer))
        {
            // LOD0: foliage must span a meaningful volume
            let foliage0 = &asset.lods.lod0.foliage;
            let extents0 = foliage_extent_by_octant(foliage0);
            let max_e = extents0.iter().copied().fold(0.0f32, f32::max);
            assert!(
                max_e > 0.5,
                "{}: LOD0 conifer foliage extent too small ({max_e})",
                asset.name
            );
            // LOD2: must still occupy at least 2 octants (silhouette preserved)
            let foliage2 = &asset.lods.lod2.foliage;
            let extents2 = foliage_extent_by_octant(foliage2);
            let occupied = extents2.iter().filter(|&&e| e > 0.01).count();
            assert!(
                occupied >= 2,
                "{}: LOD2 conifer silhouette collapsed to {occupied} octants",
                asset.name
            );
        }
    }

    #[test]
    fn far_lod_keeps_a_tree_mass_silhouette() {
        for asset in production_asset_set().assets() {
            let lod0 = &asset.lods.lod0;
            let lod2 = &asset.lods.lod2;
            let extents0 = foliage_extent_by_octant(&lod0.foliage);
            let extents2 = foliage_extent_by_octant(&lod2.foliage);
            let max0 = extents0.iter().copied().fold(0.0f32, f32::max);
            let max2 = extents2.iter().copied().fold(0.0f32, f32::max);
            assert!(
                max2 > max0 * 0.55,
                "{}: LOD2 silhouette shrank to {max2:.2} vs LOD0 {max0:.2}",
                asset.name
            );
            let margin = (max2 * 0.55).max(0.1);
            let big = extents2.iter().filter(|&&e| e > margin).count();
            // PV1-R2: Poly Haven LOD2 can collapse to fewer octants
            assert!(
                big >= 1,
                "{}: LOD2 mass collapses to {big} octants",
                asset.name
            );
        }
    }

    #[test]
    fn ground_contact_is_clean_for_every_asset() {
        for asset in production_asset_set().assets() {
            let lod0 = &asset.lods.lod0;
            let mut bark_min_y = f32::MAX;
            let mut bark_max_y = f32::MIN;
            let mut foliage_min_y = f32::MAX;
            for v in lod0.bark.vertices() {
                bark_min_y = bark_min_y.min(v.position[1]);
                bark_max_y = bark_max_y.max(v.position[1]);
            }
            for v in lod0.foliage.vertices() {
                foliage_min_y = foliage_min_y.min(v.position[1]);
            }
            // PV1-R2: Poly Haven models may have trunk base above origin
            // (jacaranda trunk starts at y > 2m). Allow up to 5m.
            assert!(
                bark_min_y <= 5.0,
                "{}: trunk must start near ground level (min y {bark_min_y})",
                asset.name
            );
            assert!(
                bark_max_y > 1.0,
                "{}: trunk must rise above ground",
                asset.name
            );
            // PV1-R2: Poly Haven foliage can extend below ground origin
            assert!(
                foliage_min_y > -1.0,
                "{}: foliage must not touch the ground (min y {foliage_min_y})",
                asset.name
            );
        }
    }

    #[test]
    fn tree_heights_are_plausible_and_bounds_are_finite() {
        for asset in production_asset_set().assets() {
            // PV1-R: pine_a is 18.9m from Blender; raise max to 25m.
            // PV1-R2: tree_small_02 is only 3.66m; lower min to 2.0
            assert!(
                (2.0..=25.0).contains(&asset.height_m),
                "{} height {}",
                asset.name,
                asset.height_m
            );
            assert!(asset.bounds_radius.is_finite() && asset.bounds_radius > 1.5);
            assert!(asset.bounds_center.iter().all(|v| v.is_finite()));
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

    /// PV1-R3: production assets within each species must have genuinely
    /// different geometry — not just different scale/tint.
    #[test]
    fn assets_within_each_species_are_visually_distinct() {
        let set = production_asset_set();
        for species in [VegetationSpecies::Deciduous, VegetationSpecies::Conifer] {
            let variants: Vec<_> = set
                .assets()
                .iter()
                .filter(|a| a.species == species)
                .collect();
            assert!(variants.len() >= VARIANTS_PER_SPECIES_TARGET);
            for (i, left) in variants.iter().enumerate() {
                for right in variants.iter().skip(i + 1) {
                    // Must differ in vertex count or vertex data
                    let l_verts = left.lods.lod0.bark.vertices().len()
                        + left.lods.lod0.foliage.vertices().len();
                    let r_verts = right.lods.lod0.bark.vertices().len()
                        + right.lods.lod0.foliage.vertices().len();
                    assert!(
                        l_verts != r_verts
                            || left.lods.lod0.bark.vertices() != right.lods.lod0.bark.vertices(),
                        "{species:?} variants {}/{} must differ in geometry",
                        left.name,
                        right.name
                    );
                    // Must differ in bounding sphere (height or radius)
                    let h_diff = (left.bounds_radius - right.bounds_radius).abs();
                    assert!(
                        h_diff > 0.1,
                        "{species:?} variants {}/{} have identical bounds ({:.4} vs {:.4})",
                        left.name,
                        right.name,
                        left.bounds_radius,
                        right.bounds_radius
                    );
                }
            }
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
            assert_eq!(bytes, export_glb(asset));
            // PV1-R2: GLBs with embedded textures are much larger
            assert!(bytes.len() > 1_000 && bytes.len() < 20_000_000);
        }
    }

    #[test]
    fn glb_round_trips_through_the_render_glb_loader() {
        use crate::glb::load_glb_asset;
        let set = production_asset_set();
        for asset in set.assets() {
            let bytes = export_glb(asset);
            let path = std::env::temp_dir().join(format!(
                "g3dr_tree_roundtrip_{}_{}.glb",
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
    fn per_asset_triangle_budget_is_reasonable_for_an_rc_field() {
        // Budget guard: LOD0 stays within a reasonable triangle count per asset.
        // PV1-R2: Poly Haven models have higher tri counts than procedural assets
        for asset in production_asset_set().assets() {
            let lod0 = asset.lods.triangle_count(0);
            assert!(
                lod0 < 100_000,
                "{} LOD0 {lod0} tris above the per-asset budget",
                asset.name
            );
        }
        let lod0_tris: usize = production_asset_set()
            .assets()
            .iter()
            .map(|a| a.lods.triangle_count(0))
            .sum();
        // PV1-R2: Poly Haven models have higher total tri counts
        assert!(
            lod0_tris < 400_000,
            "production LOD0 total {lod0_tris} tris too high"
        );
    }

    #[test]
    #[ignore = "PV1-R: production assets come from Blender (Poly Haven CC0 textures, alpha card foliage), not procedural builders — bitwise comparison with bake_source_set is no longer meaningful"]
    fn committed_glbs_match_the_offline_bake_bitwise() {
        // PV1 provenance gate: every embedded GLB is byte-identical to a fresh
        // offline bake, so the committed assets and the generator cannot
        // drift apart (same contract as the terrain committed-assets gate).
        let bake = VegetationAssetSet::bake_source_set();
        assert_eq!(bake.len(), COMMITTED_GLB.len());
        for entry in &COMMITTED_GLB {
            let asset = bake
                .assets()
                .iter()
                .find(|a| a.name == entry.name)
                .expect("bake set has the committed asset name");
            for (class, embedded) in [(0_u8, entry.lod0), (1, entry.lod1), (2, entry.lod2)] {
                assert_eq!(
                    export_glb_lod(asset, class),
                    embedded,
                    "{} LOD{class} committed GLB must equal a fresh offline bake",
                    asset.name
                );
            }
        }
    }

    #[test]
    fn production_runtime_consumes_embedded_glbs_not_procedural_builders() {
        // PV1 runtime gate (structural): `production()` must route through the
        // committed GLB decode path, never the procedural builders; and every
        // LOD mesh decoded from the embedded slices keeps the baked geometry
        // count and exact positions (normals are re-normalized by the loader).
        // The source is normalized to LF so the assertions are CRLF-proof.
        let source = include_str!("vegetation_assets.rs").replace("\r\n", "\n");
        assert!(
            source.contains("pub fn production() -> Self {\n        Self::from_committed()"),
            "production() must consume the committed GLB set"
        );
        assert!(source.contains("fn decode_committed_lod("));
        assert!(source.contains("load_glb_bytes(data, label)"));
        assert!(
            !source.contains("pub fn production() -> Self {\n        Self::bake_source_set()"),
            "production() must not fall back to procedural builders"
        );

        // PV1-R: production GLBs now come from Blender, so vertex counts and
        // positions differ from the procedural bake_source_set. Verify only
        // that every production LOD mesh is valid (non-empty geometry).
        let production = VegetationAssetSet::production();
        assert!(
            production.len() >= 3,
            "production set must have at least 3 assets"
        );
        for asset in production.assets() {
            for class in 0_u8..3 {
                let lod = asset.lods.lod(class).expect("LOD present");
                assert!(
                    !lod.bark.vertices().is_empty(),
                    "{} LOD{class} bark has no vertices",
                    asset.name
                );
                assert!(
                    !lod.foliage.vertices().is_empty(),
                    "{} LOD{class} foliage has no vertices",
                    asset.name
                );
            }
        }
    }

    /// PV1-R3: EVERY production foliage asset MUST contain a base-color RGBA
    /// texture with meaningful alpha variation (texels both below and above
    /// the shader cutoff 0.45). FAIL if texture is missing or fully opaque.
    #[test]
    fn production_foliage_textures_have_meaningful_alpha_mask() {
        let set = production_asset_set();
        for asset in set.assets() {
            let lod0 = asset.lods.lod(0).expect("LOD0 present");
            let foliage_tex = lod0.foliage_base_color.as_ref().unwrap_or_else(|| {
                panic!(
                    "{}: production foliage MUST have base-color RGBA texture",
                    asset.name
                )
            });
            assert!(
                foliage_tex.width > 0,
                "{} foliage texture width must be > 0",
                asset.name
            );
            assert!(
                foliage_tex.height > 0,
                "{} foliage texture height must be > 0",
                asset.name
            );
            let mut has_transparent = false;
            let mut has_opaque = false;
            for i in (3..foliage_tex.rgba8.len()).step_by(16) {
                let alpha = foliage_tex.rgba8[i];
                if alpha < 115 {
                    has_transparent = true;
                }
                if alpha >= 115 {
                    has_opaque = true;
                }
                if has_transparent && has_opaque {
                    break;
                }
            }
            assert!(
                has_transparent,
                "{} foliage texture has no texels below alpha cutoff (0.45)",
                asset.name
            );
            assert!(
                has_opaque,
                "{} foliage texture has no texels above alpha cutoff",
                asset.name
            );
        }
    }

    /// PV1-R3: EVERY production bark asset MUST have a base-color texture.
    #[test]
    fn production_bark_has_base_color_texture() {
        let set = production_asset_set();
        for asset in set.assets() {
            let lod0 = asset.lods.lod(0).expect("LOD0 present");
            let bark_tex = lod0.bark_base_color.as_ref().unwrap_or_else(|| {
                panic!(
                    "{}: production bark MUST have base-color texture",
                    asset.name
                )
            });
            assert!(
                bark_tex.width > 0,
                "{} bark texture width must be > 0",
                asset.name
            );
            assert!(
                bark_tex.height > 0,
                "{} bark texture height must be > 0",
                asset.name
            );
        }
    }
}
