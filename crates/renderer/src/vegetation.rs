//! G3D: production vegetation world (placement, culling, LOD).
//!
//! Owns the deterministic instance list for the FlyingField scenery and the
//! per-frame CPU selection pipeline — frustum culling, distance culling and
//! LOD assignment with hysteresis — plus the presentation-only counters and
//! debug modes. The module is pure Rust (no wgpu): everything here is
//! deterministic and unit-testable, and nothing touches simulation state.
//!
//! # Hot path
//!
//! [`VegetationWorld::update_visibility`] is zero-allocation: all scratch
//! storage (visible compacted list, sorted scratch, per-batch ranges,
//! current-LOD table, debug bounds) is preallocated at construction and
//! reused (clear + swap, never fresh `Vec`). The GPU instance buffer is
//! written by the renderer from the compacted list; no buffers, bind groups
//! or pipelines are created per frame.
//!
//! # Coordinate convention
//!
//! Render space: +Y up, XZ horizontal. Instance positions are ground points
//! (Y = field ground height). Assets are generated at the origin with their
//! base at Y = 0; the yaw rotates around Y and the scale is uniform, so no
//! 4x4 instance matrix is ever built on the CPU or GPU.

use crate::Mat4;
use crate::scenery::{
    DEFAULT_GROUND_Y, FIELD_HALF_EXTENT_M, TREE_MIN_DISTANCE_FROM_RUNWAY_M, runway_safety_rect,
};
use crate::vegetation_assets::{VegetationAssetSet, VegetationSpecies};

/// Number of LOD classes (LOD3 billboard is a documented residual gap).
pub const LOD_COUNT: usize = 3;
/// Render parts per LOD (bark + foliage).
pub const PART_COUNT: usize = 2;
/// Batch-group count: one group per (asset, LOD). PV1-R: 3 production assets
/// (pine_a, fir_a, broadleaf_a) × 3 LOD classes = 9 groups.
pub const GROUP_COUNT: usize = 3 * LOD_COUNT;

/// Default vegetation seed (matches the former `scenery::DEFAULT_TREE_SEED`).
pub const DEFAULT_VEGETATION_SEED: u64 = 42;

// ── Centralized LOD / distance policy (tunable, documented) ────────────────

/// Nominal distance (m) at which instances switch between LOD0 and LOD1.
///
/// Centralized here and re-used by the renderer and the tests — never
/// scattered constants in shaders or draw loops.
pub const DEFAULT_LOD0_MAX_M: f32 = 55.0;
/// Nominal distance (m) at which instances switch between LOD1 and LOD2.
///
/// G3-VR1: raised to keep the richer LOD1 canopy in view through the
/// mid-distance range that previously degraded to the sparse LOD2 shell.
pub const DEFAULT_LOD1_MAX_M: f32 = 160.0;
/// Distance (m) beyond which an instance is culled entirely.
///
/// The far corner of the FlyingField boundary belt (230 m radius from the
/// field centre, camera offset up to ~30 m) stays well inside this budget;
/// beyond it the instance's on-screen contribution is sub-pixel.
pub const DEFAULT_DISTANCE_CULL_M: f32 = 340.0;
/// Fractional hysteresis band applied to each LOD threshold.
///
/// Switching to a cheaper LOD requires `threshold * (1 + band)`; switching
/// back requires `threshold * (1 - band)`. This prevents LOD thrashing for
/// instances orbiting a threshold while keeping the band small enough to be
/// visually transparent.
pub const DEFAULT_HYSTERESIS_BAND: f32 = 0.10;

/// Centralized, documented LOD and culling policy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VegetationLodConfig {
    pub lod0_max_m: f32,
    pub lod1_max_m: f32,
    pub distance_cull_m: f32,
    pub hysteresis_band: f32,
}

impl Default for VegetationLodConfig {
    fn default() -> Self {
        Self {
            lod0_max_m: DEFAULT_LOD0_MAX_M,
            lod1_max_m: DEFAULT_LOD1_MAX_M,
            distance_cull_m: DEFAULT_DISTANCE_CULL_M,
            hysteresis_band: DEFAULT_HYSTERESIS_BAND,
        }
    }
}

impl VegetationLodConfig {
    /// The policy is valid: finite, ordered, positive thresholds.
    #[must_use]
    pub fn validate(&self) -> bool {
        self.lod0_max_m.is_finite()
            && self.lod1_max_m.is_finite()
            && self.distance_cull_m.is_finite()
            && self.hysteresis_band.is_finite()
            && self.hysteresis_band >= 0.0
            && 0.0 < self.lod0_max_m
            && self.lod0_max_m < self.lod1_max_m
            && self.lod1_max_m < self.distance_cull_m
    }

    fn lod0_enter(&self) -> f32 {
        self.lod0_max_m * (1.0 - self.hysteresis_band)
    }

    fn lod0_exit(&self) -> f32 {
        self.lod0_max_m * (1.0 + self.hysteresis_band)
    }

    fn lod1_enter(&self) -> f32 {
        self.lod1_max_m * (1.0 - self.hysteresis_band)
    }

    fn lod1_exit(&self) -> f32 {
        self.lod1_max_m * (1.0 + self.hysteresis_band)
    }
}

// ── Debug modes (presentation-only, never physics) ─────────────────────────

/// Vegetation debug presentation mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum VegetationDebugMode {
    /// Full lit HDR PBR production path (default).
    #[default]
    Final = 0,
    /// Color-code LOD classes per instance (LOD0 green, LOD1 yellow,
    /// LOD2 orange) — lighting and fog bypassed, production behavior off.
    Lod = 1,
    /// No visual change; periodically log visibility counters.
    Culling = 2,
}

impl VegetationDebugMode {
    #[must_use]
    pub const fn as_u32(self) -> u32 {
        self as u32
    }

    /// CLI/config label.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Final => "final",
            Self::Lod => "lod",
            Self::Culling => "culling",
        }
    }

    /// Parse a CLI/config label.
    #[must_use]
    pub fn from_label(label: &str) -> Option<Self> {
        match label {
            "final" => Some(Self::Final),
            "lod" => Some(Self::Lod),
            "culling" => Some(Self::Culling),
            _ => None,
        }
    }
}

// ── Counters (presentation-only) ───────────────────────────────────────────

/// Per-frame vegetation visibility counters, presentation-only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct VegetationFrameStats {
    pub total: u32,
    pub visible: u32,
    pub culled_frustum: u32,
    pub culled_distance: u32,
    pub lod_counts: [u32; LOD_COUNT],
    /// Scene-pass draw calls (active groups × parts).
    pub scene_draw_calls: u32,
    /// Shadow-pass draw calls (active LOD0/1 groups × parts).
    pub shadow_draw_calls: u32,
}

impl VegetationFrameStats {
    /// Total culled (frustum + distance) for reporting.
    #[must_use]
    pub fn culled_total(&self) -> u32 {
        self.culled_frustum + self.culled_distance
    }
}

// ── GPU instance layout (compact, documented) ──────────────────────────────

/// Compact 48-byte GPU instance (three vec4s, stride 48, 16-byte aligned).
///
/// ```text
/// offset 0  position_yaw : xyz world position, w = yaw (radians)
/// offset 16 scale_tint   : x = uniform scale, yzw = color tint (rgb)
/// offset 32 lod_class    : x = LOD class (0..2), y = asset index,
///                          zw = reserved (zero)
/// ```
///
/// The GPU shader composes `T = translate(position) * rotY(yaw) * scale`
/// on the fly, so no 4x4 matrix per instance is stored. The asset index and
/// LOD class ride along for the CPU-side grouping sort and the LOD debug
/// channel; production shading ignores everything except the transform.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VegetationGpuInstance {
    pub position_yaw: [f32; 4],
    pub scale_tint: [f32; 4],
    pub lod_class: [f32; 4],
}

impl VegetationGpuInstance {
    #[must_use]
    pub fn zero() -> Self {
        Self {
            position_yaw: [0.0; 4],
            scale_tint: [0.0; 4],
            lod_class: [0.0; 4],
        }
    }

    /// Build from a placement instance and the LOD class assigned this frame.
    #[must_use]
    pub fn from_placement(instance: &VegetationInstance, lod: u8) -> Self {
        Self {
            position_yaw: [
                instance.position[0],
                instance.position[1],
                instance.position[2],
                instance.yaw_rad,
            ],
            scale_tint: [
                instance.scale,
                instance.tint[0],
                instance.tint[1],
                instance.tint[2],
            ],
            lod_class: [f32::from(lod), instance.asset_index as f32, 0.0, 0.0],
        }
    }

    /// Batch group `asset * LOD_COUNT + lod` for the counting-sort grouping.
    #[must_use]
    pub fn group_index(&self) -> usize {
        self.asset_index() * LOD_COUNT + self.lod_class() as usize
    }

    #[must_use]
    pub fn asset_index(&self) -> usize {
        self.lod_class[1] as usize
    }

    #[must_use]
    pub fn lod_class(&self) -> u8 {
        self.lod_class[0] as u8
    }
}

// ── Deterministic placement ────────────────────────────────────────────────

/// Deterministic PRNG (SplitMix64). Pure, seed-based, no external state.
#[derive(Debug, Clone, Copy)]
pub struct DeterministicRng {
    state: u64,
}

impl DeterministicRng {
    #[must_use]
    pub const fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    pub fn unit(&mut self) -> f32 {
        (self.next_u64() >> 11) as f32 / (1u64 << 53) as f32
    }

    pub fn range(&mut self, low: f32, high: f32) -> f32 {
        low + (high - low) * self.unit()
    }
}

/// One placed tree instance (world data, immutable after construction).
#[derive(Debug, Clone, Copy)]
pub struct VegetationInstance {
    /// Ground position in render space (Y = field ground height).
    pub position: [f32; 3],
    /// Yaw around render Y (radians).
    pub yaw_rad: f32,
    /// Uniform scale.
    pub scale: f32,
    /// Index into the asset set.
    pub asset_index: usize,
    /// Per-channel color tint multiplier (subtle, ±~8%).
    pub tint: [f32; 3],
    /// Placement zone (0 = near field edge, 1 = boundary belt).
    pub zone: u8,
}

// ── Placement policy (documented constants) ────────────────────────────────

/// Near-field cluster centres (render XZ). The flightline is on the +X side
/// (pilot stations at x=14), so the near zone is biased to the −X flank,
/// the runway ends, and the far side; +X holds only a few trees beyond the
/// fence so the flying area stays readable.
const NEAR_CLUSTER_CENTRES: [[f32; 2]; 10] = [
    // −X flank (opposite the flightline, densest).
    [-58.0, -45.0],
    [-64.0, 18.0],
    [-50.0, 72.0],
    // +X flank, beyond the fence / safety strip.
    [46.0, -35.0],
    [54.0, 22.0],
    [44.0, 70.0],
    // Runway-end clusters (beyond the threshold, clear of the approach strip).
    [-22.0, -122.0],
    [22.0, -116.0],
    [-26.0, 122.0],
    [22.0, 118.0],
];

/// Members per near cluster (spread).
const NEAR_CLUSTER_MEMBER_MIN: usize = 8;
const NEAR_CLUSTER_MEMBER_MAX: usize = 17;
/// Near-cluster member scatter radius (m).
const NEAR_CLUSTER_SPREAD_M: f32 = 15.0;
/// Secondary "satellite" trees per near cluster (smaller, offset groups so
/// the near field reads as loose clumps instead of one blob per centre).
const NEAR_SATELLITE_MIN: usize = 1;
const NEAR_SATELLITE_MAX: usize = 3;
/// Satellite offset from the cluster centre (m).
const NEAR_SATELLITE_DISTANCE_M: f32 = 24.0;
/// Minimum centre spacing in the near zone (m).
const NEAR_MIN_SPACING_M: f32 = 7.5;
/// Boundary belt ring (m).
const BOUNDARY_INNER_RADIUS_M: f32 = 160.0;
const BOUNDARY_OUTER_RADIUS_M: f32 = 230.0;
/// Boundary angular slots.
const BOUNDARY_SLOT_COUNT: usize = 16;
/// Probability a boundary slot is left empty (aperture).
const BOUNDARY_APERTURE_PROBABILITY: f32 = 0.14;
/// Minimum centre spacing in the boundary belt (m).
const BOUNDARY_MIN_SPACING_M: f32 = 6.0;
/// Near-zone scale range and boundary scale range.
const NEAR_SCALE_MIN: f32 = 0.85;
const NEAR_SCALE_MAX: f32 = 1.25;
const BOUNDARY_SCALE_MIN: f32 = 0.75;
const BOUNDARY_SCALE_MAX: f32 = 1.15;

/// Total-instance budget guard: enough for credible depth, never a forest.
const TARGET_MIN_INSTANCES: usize = 150;
const TARGET_MAX_INSTANCES: usize = 320;

#[must_use]
fn species_for_zone(zone: u8, rng: &mut DeterministicRng) -> VegetationSpecies {
    match zone {
        // Near field: broadleaf field trees dominate, conifers interspersed.
        0 => {
            if rng.unit() < 0.66 {
                VegetationSpecies::Deciduous
            } else {
                VegetationSpecies::Conifer
            }
        }
        // Boundary: conifers lead, deciduous fill in — an unequal, natural
        // mix rather than a strict species ring.
        _ => {
            if rng.unit() < 0.52 {
                VegetationSpecies::Conifer
            } else {
                VegetationSpecies::Deciduous
            }
        }
    }
}

/// Asset variant weights per species (uneven so the mix does not read as a
/// uniform row of look-alikes). PV1-R: 3 assets — conifer 0/1 (pine/fir),
/// deciduous 2 (broadleaf).
#[must_use]
fn asset_index_for(species: VegetationSpecies, variant_roll: f32) -> usize {
    match species {
        // Asset 2 (broadleaf_a).
        VegetationSpecies::Deciduous => 2,
        // Assets 0..1 (conifer pine_a / fir_a).
        VegetationSpecies::Conifer => {
            if variant_roll < 0.5 {
                0
            } else {
                1
            }
        }
    }
}

/// Deterministic FlyingField vegetation layout.
///
/// Pure function of `seed`: the same seed always produces the same instance
/// list. The runway + safety strip is excluded (plus a clearance margin),
/// the flying area stays open, clusters read as coherent field-edge groups,
/// and minimum spacing is enforced between every accepted pair.
#[must_use]
pub fn flying_field_layout(
    assets: &VegetationAssetSet,
    seed: u64,
    ground_y: f32,
    field_half_extent: f32,
    runway_clearance: f32,
) -> Vec<VegetationInstance> {
    let mut rng = DeterministicRng::new(seed);
    let mut instances: Vec<VegetationInstance> = Vec::new();
    let mut near_accepted: Vec<[f32; 2]> = Vec::new();
    let mut boundary_accepted: Vec<[f32; 2]> = Vec::new();
    let safe_rect = expanded_safety_rect(runway_clearance);

    // Zone 0: near clusters, each followed by loose satellite trees so the
    // field edge reads as natural clumps rather than one dense blob per
    // centre.
    for &[cx, cz] in &NEAR_CLUSTER_CENTRES {
        let members = NEAR_CLUSTER_MEMBER_MIN
            + (rng.unit() * (NEAR_CLUSTER_MEMBER_MAX - NEAR_CLUSTER_MEMBER_MIN) as f32) as usize;
        let spread = NEAR_CLUSTER_SPREAD_M * (0.6 + 0.8 * rng.unit());
        let mut accepted_in_cluster = 0;
        let mut attempts = 0;
        while accepted_in_cluster < members && attempts < members * 10 {
            attempts += 1;
            let x = cx + rng.range(-1.0, 1.0) * spread;
            let z = cz + rng.range(-1.0, 1.0) * spread;
            if !candidate_ok(
                x,
                z,
                field_half_extent,
                &safe_rect,
                &near_accepted,
                &boundary_accepted,
                NEAR_MIN_SPACING_M,
            ) {
                continue;
            }
            near_accepted.push([x, z]);
            accepted_in_cluster += 1;
            let species = species_for_zone(0, &mut rng);
            let asset_index =
                asset_index_for(species, rng.unit()).min(assets.len().saturating_sub(1));
            instances.push(VegetationInstance {
                position: [x, ground_y, z],
                yaw_rad: rng.range(0.0, std::f32::consts::TAU),
                scale: rng.range(NEAR_SCALE_MIN, NEAR_SCALE_MAX),
                asset_index,
                tint: [
                    rng.range(0.94, 1.04),
                    rng.range(0.94, 1.04),
                    rng.range(0.88, 1.00),
                ],
                zone: 0,
            });
        }
        // Satellites: a small secondary clump away from the main blob.
        let satellites = NEAR_SATELLITE_MIN
            + (rng.unit() * (NEAR_SATELLITE_MAX - NEAR_SATELLITE_MIN + 1) as f32) as usize;
        let satellite_azimuth = rng.range(0.0, std::f32::consts::TAU);
        let satellite_distance = NEAR_SATELLITE_DISTANCE_M * rng.range(0.75, 1.25);
        let satellite_cx = cx + satellite_distance * satellite_azimuth.cos();
        let satellite_cz = cz + satellite_distance * satellite_azimuth.sin();
        let satellite_spread = NEAR_CLUSTER_SPREAD_M * 0.45;
        let mut accepted_satellite = 0;
        attempts = 0;
        while accepted_satellite < satellites && attempts < satellites * 12 {
            attempts += 1;
            let x = satellite_cx + rng.range(-1.0, 1.0) * satellite_spread;
            let z = satellite_cz + rng.range(-1.0, 1.0) * satellite_spread;
            if !candidate_ok(
                x,
                z,
                field_half_extent,
                &safe_rect,
                &near_accepted,
                &boundary_accepted,
                NEAR_MIN_SPACING_M,
            ) {
                continue;
            }
            near_accepted.push([x, z]);
            accepted_satellite += 1;
            let species = species_for_zone(0, &mut rng);
            let asset_index =
                asset_index_for(species, rng.unit()).min(assets.len().saturating_sub(1));
            instances.push(VegetationInstance {
                position: [x, ground_y, z],
                yaw_rad: rng.range(0.0, std::f32::consts::TAU),
                scale: rng.range(NEAR_SCALE_MIN * 0.82, NEAR_SCALE_MAX * 0.92),
                asset_index,
                tint: [
                    rng.range(0.94, 1.04),
                    rng.range(0.94, 1.04),
                    rng.range(0.88, 1.00),
                ],
                zone: 0,
            });
        }
    }

    // Zone 1: boundary belt — clumps of unequal width/density with natural
    // apertures and occasional isolated trees.
    let step = std::f32::consts::TAU / BOUNDARY_SLOT_COUNT as f32;
    for slot in 0..BOUNDARY_SLOT_COUNT {
        let anchor = (slot as f32 + 0.5) * step;
        if rng.unit() < BOUNDARY_APERTURE_PROBABILITY {
            continue;
        }
        let cos_anchor = anchor.cos();
        let member_count = if cos_anchor < -0.25 {
            12 + (rng.unit() * 9.0) as usize // dense opposite the flightline
        } else if cos_anchor > 0.10 {
            5 + (rng.unit() * 5.0) as usize // sparse toward flightline
        } else {
            8 + (rng.unit() * 7.0) as usize
        };
        let cluster_radius =
            rng.range(BOUNDARY_INNER_RADIUS_M + 6.0, BOUNDARY_OUTER_RADIUS_M - 6.0);
        let spread_rad = 0.085 * (0.7 + 0.6 * rng.unit());
        for _ in 0..member_count {
            let angle = anchor + rng.range(-1.0, 1.0) * spread_rad;
            let radius = (cluster_radius + rng.range(-1.0, 1.0) * 7.0)
                .clamp(BOUNDARY_INNER_RADIUS_M, BOUNDARY_OUTER_RADIUS_M);
            let x = radius * angle.cos();
            let z = radius * angle.sin();
            if !spacing_ok(
                x,
                z,
                &near_accepted,
                &boundary_accepted,
                BOUNDARY_MIN_SPACING_M,
            ) {
                continue;
            }
            boundary_accepted.push([x, z]);
            let species = species_for_zone(1, &mut rng);
            let asset_index =
                asset_index_for(species, rng.unit()).min(assets.len().saturating_sub(1));
            instances.push(VegetationInstance {
                position: [x, ground_y, z],
                yaw_rad: rng.range(0.0, std::f32::consts::TAU),
                scale: rng.range(BOUNDARY_SCALE_MIN, BOUNDARY_SCALE_MAX),
                asset_index,
                tint: [
                    rng.range(0.92, 1.02),
                    rng.range(0.92, 1.02),
                    rng.range(0.86, 0.98),
                ],
                zone: 1,
            });
        }
    }

    instances
}

/// Runway + clearance rectangle as `[min_x, min_z, max_x, max_z]`.
fn expanded_safety_rect(runway_clearance: f32) -> [f32; 4] {
    let rect = runway_safety_rect();
    [
        rect[0] - runway_clearance,
        rect[1] - runway_clearance,
        rect[2] + runway_clearance,
        rect[3] + runway_clearance,
    ]
}

fn candidate_ok(
    x: f32,
    z: f32,
    field_half_extent: f32,
    safe_rect: &[f32; 4],
    near: &[[f32; 2]],
    boundary: &[[f32; 2]],
    min_spacing: f32,
) -> bool {
    if x.abs() > field_half_extent || z.abs() > field_half_extent {
        return false;
    }
    if x >= safe_rect[0] && x <= safe_rect[2] && z >= safe_rect[1] && z <= safe_rect[3] {
        return false;
    }
    spacing_ok(x, z, near, boundary, min_spacing)
}

/// Minimum-distance check against every previously accepted tree (both
/// zones), so clusters cannot overlap each other across zone boundaries.
fn spacing_ok(x: f32, z: f32, near: &[[f32; 2]], boundary: &[[f32; 2]], min_spacing: f32) -> bool {
    let min_sq = min_spacing * min_spacing;
    if near
        .iter()
        .any(|&[px, pz]| (px - x).powi(2) + (pz - z).powi(2) < min_sq)
    {
        return false;
    }
    if boundary
        .iter()
        .any(|&[px, pz]| (px - x).powi(2) + (pz - z).powi(2) < min_sq)
    {
        return false;
    }
    true
}

/// Whole-placement validation (used by tests and the fail-safe path).
#[must_use]
pub fn placement_is_valid(
    instances: &[VegetationInstance],
    assets: &VegetationAssetSet,
    field_half_extent: f32,
    runway_clearance: f32,
    min_spacing: f32,
) -> bool {
    if instances.is_empty() {
        return false;
    }
    let safe_rect = expanded_safety_rect(runway_clearance);
    for (index, instance) in instances.iter().enumerate() {
        let [x, y, z] = instance.position;
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return false;
        }
        if x.abs() > field_half_extent || z.abs() > field_half_extent {
            return false;
        }
        if instance.asset_index >= assets.len() {
            return false;
        }
        if !instance.scale.is_finite() || !(0.5..=2.0).contains(&instance.scale) {
            return false;
        }
        if !instance.yaw_rad.is_finite() {
            return false;
        }
        if instance
            .tint
            .iter()
            .any(|channel| !channel.is_finite() || *channel < 0.5)
        {
            return false;
        }
        // Near zone must stay outside the expanded runway rectangle; the
        // boundary belt is far outside by construction.
        if instance.zone == 0
            && x >= safe_rect[0]
            && x <= safe_rect[2]
            && z >= safe_rect[1]
            && z <= safe_rect[3]
        {
            return false;
        }
        // Pairwise spacing (whole-field guarantee).
        for other in instances.iter().skip(index + 1) {
            let dx = other.position[0] - x;
            let dz = other.position[2] - z;
            if dx * dx + dz * dz < min_spacing * min_spacing {
                return false;
            }
        }
    }
    true
}

// ── Frustum culling (CPU) ──────────────────────────────────────────────────

/// Six normalized frustum planes (interior = positive side).
///
/// Extracted from a row-major view-projection with the WebGPU clip volume
/// x ∈ [−w, w], y ∈ [−w, w], z ∈ [0, w]:
/// - left/right/bottom/top from rows 0/1 against row 3,
/// - near = row 2 (clip z ≥ 0),
/// - far = row 3 − row 2 (clip z ≤ w).
#[derive(Debug, Clone, Copy)]
pub struct FrustumPlanes {
    planes: [[f32; 4]; 6],
}

impl FrustumPlanes {
    /// Extract from a view-projection matrix.
    #[must_use]
    pub fn from_view_projection(vp: &Mat4) -> Self {
        let r = vp.rows();
        let mut planes = [[0.0f32; 4]; 6];
        planes[0] = combine(r[3], r[0], 1.0, 1.0); // left
        planes[1] = combine(r[3], r[0], 1.0, -1.0); // right
        planes[2] = combine(r[3], r[1], 1.0, 1.0); // bottom
        planes[3] = combine(r[3], r[1], 1.0, -1.0); // top
        planes[4] = r[2]; // near: clip z >= 0
        planes[5] = combine(r[3], r[2], 1.0, -1.0); // far
        for plane in &mut planes {
            let normal_length =
                (plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]).sqrt();
            if normal_length > 1.0e-9 {
                for component in plane.iter_mut() {
                    *component /= normal_length;
                }
            }
        }
        Self { planes }
    }

    /// Classic sphere-vs-frustum inclusion test.
    #[must_use]
    pub fn intersects_sphere(&self, centre: [f32; 3], radius: f32) -> bool {
        self.planes.iter().all(|plane| {
            plane[0] * centre[0] + plane[1] * centre[1] + plane[2] * centre[2] + plane[3] >= -radius
        })
    }
}

fn combine(a: [f32; 4], b: [f32; 4], sign_a: f32, sign_b: f32) -> [f32; 4] {
    [
        sign_a * a[0] + sign_b * b[0],
        sign_a * a[1] + sign_b * b[1],
        sign_a * a[2] + sign_b * b[2],
        sign_a * a[3] + sign_b * b[3],
    ]
}

// ── World (asset set + instance list + per-frame selection) ────────────────

/// The vegetation world: committed assets, static placement, persistent
/// per-frame scratch. Owned by the renderer, presentation-only.
pub struct VegetationWorld {
    assets: VegetationAssetSet,
    instances: Vec<VegetationInstance>,
    config: VegetationLodConfig,
    /// Current LOD class per instance (hysteresis state).
    current_lod: Vec<u8>,
    /// Compacted visible instances (unordered pass-1 output).
    visible: Vec<VegetationGpuInstance>,
    /// Sorted scratch swapped with `visible` each frame (keeps capacity).
    visible_sorted: Vec<VegetationGpuInstance>,
    /// Per-visible bounds (centre xyz + radius), aligned with `visible()`.
    /// Retained for future debug visualisation; the frustum culling test uses
    /// the same bounding-sphere data inline during `update_visibility`.
    visible_bounds: Vec<[f32; 4]>,
    /// Sorted bounds scratch swapped with `visible_bounds` each frame.
    visible_bounds_sorted: Vec<[f32; 4]>,
    /// Per-group (start, count) within `visible`; group = asset*3 + lod.
    batch_ranges: [u32; GROUP_COUNT * 2],
    stats: VegetationFrameStats,
}

impl VegetationWorld {
    /// Build a world from committed assets and a placement list.
    #[must_use]
    pub fn new(
        assets: VegetationAssetSet,
        instances: Vec<VegetationInstance>,
        config: VegetationLodConfig,
    ) -> Self {
        let instance_count = instances.len();
        // Hysteresis starting state: nominal LOD from the field-centre
        // distance, so the first frame starts in a sane class.
        let mut current_lod = Vec::with_capacity(instance_count);
        for instance in &instances {
            let distance = (instance.position[0].powi(2) + instance.position[2].powi(2)).sqrt()
                + camera_height_guess();
            current_lod.push(initial_lod(&config, distance));
        }
        Self {
            assets,
            instances,
            config,
            current_lod,
            visible: Vec::with_capacity(instance_count),
            visible_sorted: Vec::with_capacity(instance_count),
            visible_bounds: Vec::with_capacity(instance_count),
            visible_bounds_sorted: Vec::with_capacity(instance_count),
            batch_ranges: [0; GROUP_COUNT * 2],
            stats: VegetationFrameStats::default(),
        }
    }

    /// Convenience constructor for the FlyingField preset.
    #[must_use]
    pub fn flying_field(seed: u64, ground_y: f32) -> Self {
        let assets = VegetationAssetSet::production();
        let instances = flying_field_layout(
            &assets,
            seed,
            ground_y,
            FIELD_HALF_EXTENT_M,
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
        );
        debug_assert!(
            (TARGET_MIN_INSTANCES..=TARGET_MAX_INSTANCES).contains(&instances.len()),
            "total instances {} outside budget [{TARGET_MIN_INSTANCES}, {TARGET_MAX_INSTANCES}]",
            instances.len()
        );
        Self::new(assets, instances, VegetationLodConfig::default())
    }

    #[must_use]
    pub fn assets(&self) -> &VegetationAssetSet {
        &self.assets
    }

    #[must_use]
    pub fn instances(&self) -> &[VegetationInstance] {
        &self.instances
    }

    #[must_use]
    pub fn config(&self) -> &VegetationLodConfig {
        &self.config
    }

    /// Instance capacity for GPU buffer preallocation (padded, stable).
    #[must_use]
    pub fn instance_capacity(&self) -> usize {
        (self.instances.len() + 63) & !63
    }

    #[must_use]
    pub fn stats(&self) -> &VegetationFrameStats {
        &self.stats
    }

    /// Compacted visible GPU instances, grouped by (asset, LOD), aligned with
    /// [`Self::batch_ranges`].
    #[must_use]
    pub fn visible(&self) -> &[VegetationGpuInstance] {
        &self.visible
    }

    /// `(start, count)` per batch group; group = asset * 3 + lod.
    #[must_use]
    pub fn batch_ranges(&self) -> &[u32] {
        &self.batch_ranges
    }

    /// Per-visible bounds (centre xyz + radius), aligned with `visible()`.
    #[must_use]
    pub fn visible_bounds(&self) -> &[[f32; 4]] {
        &self.visible_bounds
    }

    /// Replace the LOD/culling policy (validated).
    pub fn set_lod_config(&mut self, config: VegetationLodConfig) {
        if config.validate() {
            self.config = config;
        }
    }

    /// Per-frame CPU selection: frustum + distance culling, LOD assignment
    /// with hysteresis, and compaction grouped by (asset, LOD).
    ///
    /// Zero-allocation: all scratch buffers are reused. `eye` is the camera
    /// position and `vp` the camera view-projection.
    pub fn update_visibility(&mut self, eye: [f32; 3], vp: &Mat4) {
        let frustum = FrustumPlanes::from_view_projection(vp);
        self.visible.clear();
        self.visible_bounds.clear();
        let mut counts = [0u32; GROUP_COUNT];
        let mut lod_counts = [0u32; LOD_COUNT];

        let config = &self.config;
        let assets = &self.assets;
        let mut total_visible = 0u32;
        let mut culled_frustum = 0u32;
        let mut culled_distance = 0u32;

        for (index, instance) in self.instances.iter().enumerate() {
            let asset = &assets.assets()[instance.asset_index];
            let scale = instance.scale;
            let centre = [
                instance.position[0] + asset.bounds_center[0] * scale,
                instance.position[1] + asset.bounds_center[1] * scale,
                instance.position[2] + asset.bounds_center[2] * scale,
            ];
            let dx = centre[0] - eye[0];
            let dy = centre[1] - eye[1];
            let dz = centre[2] - eye[2];
            let distance = (dx * dx + dy * dy + dz * dz).sqrt();
            let radius = asset.bounds_radius * scale;

            if distance > config.distance_cull_m {
                culled_distance += 1;
                continue;
            }
            if !frustum.intersects_sphere(centre, radius) {
                culled_frustum += 1;
                continue;
            }

            let lod = next_lod(self.current_lod[index], distance, config);
            self.current_lod[index] = lod;
            let group = instance.asset_index * LOD_COUNT + lod as usize;
            debug_assert!(group < GROUP_COUNT, "asset index out of range");
            counts[group] += 1;
            lod_counts[lod as usize] += 1;
            total_visible += 1;
            self.visible_bounds
                .push([centre[0], centre[1], centre[2], radius]);
            self.visible
                .push(VegetationGpuInstance::from_placement(instance, lod));
        }

        // Prefix sums -> per-group ranges within `visible`.
        let mut base = 0u32;
        for (group, &count) in counts.iter().enumerate() {
            self.batch_ranges[group * 2] = base;
            self.batch_ranges[group * 2 + 1] = count;
            base += count;
        }

        // Counting-sort into the scratch vec so each group's instances are
        // contiguous, then swap (capacities are preserved; zero allocation).
        if total_visible > 1 {
            self.visible_sorted.clear();
            self.visible_bounds_sorted.clear();
            self.visible_sorted
                .resize(total_visible as usize, VegetationGpuInstance::zero());
            self.visible_bounds_sorted
                .resize(total_visible as usize, [0.0; 4]);
            let mut cursors = self.batch_ranges;
            for (position, gpu) in self.visible.iter().enumerate() {
                let group = gpu.group_index();
                let slot = cursors[group * 2] as usize;
                cursors[group * 2] += 1;
                self.visible_sorted[slot] = *gpu;
                self.visible_bounds_sorted[slot] = self.visible_bounds[position];
            }
            std::mem::swap(&mut self.visible, &mut self.visible_sorted);
            std::mem::swap(&mut self.visible_bounds, &mut self.visible_bounds_sorted);
        } else if total_visible == 1 {
            let gpu = self.visible[0];
            let bounds = self.visible_bounds[0];
            self.visible.clear();
            self.visible_bounds.clear();
            self.visible.push(gpu);
            self.visible_bounds.push(bounds);
        }

        // Session stats.
        let mut active_groups = 0u32;
        let mut shadow_active = 0u32;
        for group in 0..GROUP_COUNT {
            let lod = group % LOD_COUNT;
            if self.batch_ranges[group * 2 + 1] > 0 {
                active_groups += 1;
                if lod <= 1 {
                    shadow_active += 1;
                }
            }
        }
        self.stats = VegetationFrameStats {
            total: self.instances.len() as u32,
            visible: total_visible,
            culled_frustum,
            culled_distance,
            lod_counts,
            scene_draw_calls: active_groups * PART_COUNT as u32,
            shadow_draw_calls: shadow_active * PART_COUNT as u32,
        };
    }
}

/// Typical camera height over the field for the first-frame LOD guess.
fn camera_height_guess() -> f32 {
    2.0
}

/// Nominal LOD class by thresholds (no hysteresis — construction state only).
fn initial_lod(config: &VegetationLodConfig, distance: f32) -> u8 {
    if distance < config.lod0_max_m {
        0
    } else if distance < config.lod1_max_m {
        1
    } else {
        2
    }
}

/// Hysteresis-aware LOD transition.
///
/// An instance keeps its current LOD unless the distance leaves the current
/// class band: entering a cheaper LOD requires `threshold * (1 + band)`,
/// returning to a richer LOD requires `threshold * (1 - band)`. Returns the
/// next class in 0..=2; distance culling happens before this is called.
fn next_lod(current: u8, distance: f32, config: &VegetationLodConfig) -> u8 {
    match current {
        0 => {
            if distance > config.lod0_exit() {
                1
            } else {
                0
            }
        }
        1 => {
            if distance < config.lod0_enter() {
                0
            } else if distance > config.lod1_exit() {
                2
            } else {
                1
            }
        }
        _ => {
            if distance < config.lod1_enter() {
                1
            } else {
                2
            }
        }
    }
}

impl Default for VegetationWorld {
    fn default() -> Self {
        Self::flying_field(DEFAULT_VEGETATION_SEED, DEFAULT_GROUND_Y)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::{look_at_rh, webgpu_perspective};
    use crate::mesh::SAFE_NORMAL;

    // Compile-time sanity: the shared "safe" normal must point up.
    const _: () = assert!(SAFE_NORMAL[1] > 0.0, "SAFE_NORMAL sanity");

    fn test_world(seed: u64) -> VegetationWorld {
        VegetationWorld::flying_field(seed, DEFAULT_GROUND_Y)
    }

    #[test]
    fn placement_respects_runway_and_safety_exclusion() {
        let world = test_world(DEFAULT_VEGETATION_SEED);
        let instances = world.instances();
        assert!(!instances.is_empty());
        let safe_rect = expanded_safety_rect(TREE_MIN_DISTANCE_FROM_RUNWAY_M);
        for instance in instances {
            let [x, _, z] = instance.position;
            if x >= safe_rect[0] && x <= safe_rect[2] && z >= safe_rect[1] && z <= safe_rect[3] {
                panic!("tree inside expanded runway safety rect: ({x}, {z})");
            }
        }
    }

    #[test]
    fn placement_bounds_and_scales_are_finite_and_bounded() {
        let world = test_world(DEFAULT_VEGETATION_SEED);
        for instance in world.instances() {
            assert!(instance.position.iter().all(|v| v.is_finite()));
            assert!(instance.yaw_rad.is_finite());
            assert!(instance.scale.is_finite() && (0.5..=2.0).contains(&instance.scale));
            assert!(instance.asset_index < world.assets().len());
            assert!(instance.tint.iter().all(|c| c.is_finite() && *c >= 0.5));
        }
    }

    #[test]
    fn placement_count_is_within_the_field_budget() {
        let total = test_world(DEFAULT_VEGETATION_SEED).instances().len();
        assert!(
            (TARGET_MIN_INSTANCES..=TARGET_MAX_INSTANCES).contains(&total),
            "total instances {total} outside [{TARGET_MIN_INSTANCES}, {TARGET_MAX_INSTANCES}]"
        );
    }

    #[test]
    fn placement_enforces_minimum_spacing_everywhere() {
        let world = test_world(DEFAULT_VEGETATION_SEED);
        assert!(placement_is_valid(
            world.instances(),
            world.assets(),
            FIELD_HALF_EXTENT_M,
            TREE_MIN_DISTANCE_FROM_RUNWAY_M,
            6.0,
        ));
    }

    #[test]
    fn placement_is_deterministic_for_the_same_seed() {
        let a = test_world(7);
        let b = test_world(7);
        assert_eq!(a.instances().len(), b.instances().len());
        for (left, right) in a.instances().iter().zip(b.instances()) {
            assert_eq!(left.position, right.position);
            assert_eq!(left.yaw_rad.to_bits(), right.yaw_rad.to_bits());
            assert_eq!(left.scale.to_bits(), right.scale.to_bits());
            assert_eq!(left.asset_index, right.asset_index);
        }
    }

    #[test]
    fn different_seeds_produce_meaningfully_different_layouts() {
        let a = test_world(1);
        let b = test_world(999_983);
        let moved = a
            .instances()
            .iter()
            .zip(b.instances())
            .filter(|(l, r)| l.position != r.position)
            .count();
        assert!(
            moved >= a.instances().len() / 2,
            "seeds must produce mostly different placements"
        );
    }

    #[test]
    fn both_species_are_present_within_target_ranges() {
        let world = test_world(DEFAULT_VEGETATION_SEED);
        let deciduous = world
            .instances()
            .iter()
            .filter(|i| {
                matches!(
                    world.assets().assets()[i.asset_index].species,
                    VegetationSpecies::Deciduous
                )
            })
            .count();
        let conifer = world.instances().len() - deciduous;
        assert!(
            deciduous > world.instances().len() / 5,
            "deciduous underrepresented"
        );
        assert!(
            conifer > world.instances().len() / 8,
            "conifer underrepresented"
        );
    }

    #[test]
    fn near_trees_populate_and_boundary_belt_stays_in_its_ring() {
        let world = test_world(DEFAULT_VEGETATION_SEED);
        let near: Vec<_> = world.instances().iter().filter(|i| i.zone == 0).collect();
        let boundary: Vec<_> = world.instances().iter().filter(|i| i.zone == 1).collect();
        assert!(near.len() >= 40, "near zone must populate");
        assert!(boundary.len() >= 60, "boundary belt must populate");
        for instance in boundary {
            let radius = (instance.position[0].powi(2) + instance.position[2].powi(2)).sqrt();
            assert!(
                (150.0..=235.0).contains(&radius),
                "boundary tree at radius {radius}"
            );
        }
    }

    #[test]
    fn gpu_instance_layout_is_expected_size_and_aligned() {
        assert_eq!(std::mem::size_of::<VegetationGpuInstance>(), 48);
        assert_eq!(std::mem::align_of::<VegetationGpuInstance>(), 4);
        assert_eq!(std::mem::offset_of!(VegetationGpuInstance, position_yaw), 0);
        assert_eq!(std::mem::offset_of!(VegetationGpuInstance, scale_tint), 16);
        assert_eq!(std::mem::offset_of!(VegetationGpuInstance, lod_class), 32);
    }

    fn simple_vp(eye: [f32; 3], target: [f32; 3]) -> (Mat4, [f32; 3]) {
        let view = look_at_rh(eye, target, [0.0, 1.0, 0.0]);
        let projection = webgpu_perspective(60.0_f32.to_radians(), 16.0 / 9.0, 0.05, 2_000.0)
            .expect("projection valid");
        (projection * view, eye)
    }

    #[test]
    fn frustum_keeps_front_centre_visible_and_blocks_behind_camera() {
        let (vp, _) = simple_vp([0.0, 3.0, 20.0], [0.0, 0.0, 0.0]);
        let frustum = FrustumPlanes::from_view_projection(&vp);
        // 12 m ahead of the camera, inside the view cone.
        assert!(frustum.intersects_sphere([0.0, 0.0, 8.0], 1.0));
        // Behind the camera: culled.
        assert!(!frustum.intersects_sphere([0.0, 0.0, 30.0], 1.0));
        // Far outside the horizontal FOV to the right.
        assert!(!frustum.intersects_sphere([400.0, 0.0, 0.0], 1.0));
        // Large-radius sphere straddling the near plane is still visible.
        assert!(frustum.intersects_sphere([0.0, 3.0, 19.0], 3.0));
    }

    #[test]
    fn frustum_handles_altitude_lookdown_and_extreme_positions() {
        let (vp, _) = simple_vp([0.0, 120.0, 0.0], [0.0, 0.0, 0.0]);
        let frustum = FrustumPlanes::from_view_projection(&vp);
        assert!(frustum.intersects_sphere([0.0, 0.0, 0.0], 5.0));
        // Directly above (behind) the look-down camera is culled.
        assert!(!frustum.intersects_sphere([0.0, 300.0, 0.0], 2.0));
    }

    #[test]
    fn distance_cull_removes_instances_beyond_the_configured_range() {
        let mut world = test_world(DEFAULT_VEGETATION_SEED);
        let mut config = *world.config();
        config.lod0_max_m = 40.0;
        config.lod1_max_m = 60.0;
        config.distance_cull_m = 80.0;
        world.set_lod_config(config);
        let (vp, eye) = simple_vp([0.0, 2.0, 0.0], [0.0, 0.0, -100.0]);
        world.update_visibility(eye, &vp);
        let stats = *world.stats();
        assert!(stats.visible > 0);
        assert!(stats.culled_distance > 0);
        assert_eq!(
            stats.total,
            stats.visible + stats.culled_frustum + stats.culled_distance
        );
    }

    #[test]
    fn lod_thresholds_are_centralized_and_ordered() {
        let config = VegetationLodConfig::default();
        assert!(config.validate());
        assert!(config.lod0_enter() < config.lod0_max_m);
        assert!(config.lod0_exit() > config.lod0_max_m);
        assert!(config.lod0_exit() < config.lod1_enter());
        assert!(config.lod1_exit() < config.distance_cull_m);
    }

    #[test]
    fn lod_hysteresis_does_not_thrash_across_a_boundary() {
        let config = VegetationLodConfig {
            lod0_max_m: 50.0,
            lod1_max_m: 100.0,
            distance_cull_m: 300.0,
            hysteresis_band: 0.10,
        };
        // Start inside LOD1 and orbit the LOD0/LOD1 threshold with small
        // camera drift.
        let mut current = 1u8;
        let mut transitions = 0u32;
        let mut distance = 52.0;
        for step in 0..400 {
            distance += (step as f32 * 0.07).sin() * 1.5;
            let next = next_lod(current, distance, &config);
            if next != current {
                transitions += 1;
                current = next;
            }
        }
        assert!(
            transitions <= 6,
            "hysteresis allowed {transitions} LOD flips; expected <= 6"
        );
    }

    #[test]
    fn lod_transitions_are_monotonic_for_a_slow_zoom() {
        let config = VegetationLodConfig::default();
        let mut current = initial_lod(&config, 4.0);
        assert_eq!(current, 0);
        let mut previous = current;
        let mut transitions = Vec::new();
        let mut distance = 4.0;
        while distance < 330.0 {
            current = next_lod(current, distance, &config);
            if current != previous {
                transitions.push((distance, current));
                previous = current;
            }
            distance += 2.0;
        }
        assert_eq!(
            transitions.len(),
            2,
            "expected LOD0->1->2, got {transitions:?}"
        );
        assert_eq!(transitions[0].1, 1);
        assert_eq!(transitions[1].1, 2);
    }

    #[test]
    fn visibility_is_grouped_by_asset_and_lod_into_batches() {
        let mut world = test_world(DEFAULT_VEGETATION_SEED);
        let (vp, eye) = simple_vp([0.0, 2.0, 0.0], [0.0, 0.0, -200.0]);
        world.update_visibility(eye, &vp);
        let ranges = world.batch_ranges();
        let visible = world.visible();
        let stats = *world.stats();
        assert_eq!(visible.len() as u32, stats.visible);
        for group in 0..GROUP_COUNT {
            let start = ranges[group * 2] as usize;
            let count = ranges[group * 2 + 1] as usize;
            assert!(start + count <= visible.len());
            for instance in &visible[start..start + count] {
                assert_eq!(
                    group,
                    instance.group_index(),
                    "group {group} contains a foreign instance"
                );
            }
        }
        assert_eq!(
            stats.scene_draw_calls as usize,
            (0..GROUP_COUNT).filter(|&g| ranges[g * 2 + 1] > 0).count() * PART_COUNT
        );
    }

    #[test]
    fn visible_scratch_capacity_is_reused_across_frames() {
        let mut world = test_world(DEFAULT_VEGETATION_SEED);
        let (vp1, eye1) = simple_vp([0.0, 2.0, 0.0], [0.0, 0.0, -200.0]);
        let (vp2, eye2) = simple_vp([0.0, 40.0, 0.0], [0.0, 0.0, -100.0]);
        world.update_visibility(eye1, &vp1);
        let capacity_after_first = world.visible.capacity();
        world.update_visibility(eye2, &vp2);
        assert_eq!(
            world.visible.capacity(),
            capacity_after_first,
            "visible scratch must reuse capacity"
        );
    }

    #[test]
    fn debug_modes_map_cleanly() {
        for mode in [
            VegetationDebugMode::Final,
            VegetationDebugMode::Lod,
            VegetationDebugMode::Culling,
        ] {
            let label = mode.label();
            assert_eq!(VegetationDebugMode::from_label(label), Some(mode));
            assert_eq!(mode.as_u32(), mode as u32);
        }
        assert_eq!(VegetationDebugMode::from_label("bogus"), None);
    }

    #[test]
    fn far_boundary_trees_deserve_lod2_not_cull_at_default_range() {
        // Regression guard for the distance-cull choice: at the default
        // budget the farthest boundary-tree centre (235 m) must be culled by
        // DISTANCE only when it actually exceeds the cull range.
        assert!(235.0 < VegetationLodConfig::default().distance_cull_m);
    }
}
