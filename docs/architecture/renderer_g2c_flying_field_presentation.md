# G2C — Flying-Field Presentation Pass

Status: implemented slice (renderer-only, scenery-only). Base commit:
`fac43fb` (G1D final pass). Branch: `feature/g2c-flying-field-presentation`.

## Scope

G2C enriches the `SceneryPreset::FlyingField` from a technical procedural
"strip with uniform trees" into a readable RC aeromodelling field, keeping the
G2A architecture: `generate_flying_field(...)` runs once at initialization,
outputs **one merged `SceneryMesh`** (single vertex/index pair), and is
uploaded with the existing scenery path in `gpu.rs` — unchanged. There is no
per-frame generation, no per-frame allocation, no new GPU pipeline, no new
assets, and no runtime-dependent behavior.

Out of scope (explicitly NOT in this slice, belongs to later slices):

- hangars, buildings, vehicles, people
- dense/massive vegetation, blade grass, billboards, alpha vegetation
- textures, normal maps, procedural terrain materials
- additional lighting, shadows (G2B parallel work), clouds
- gameplay/collision volumes, wind interaction (the windsock is static)

## Visual objective

Hierarchy of readability, from strongest to weakest: **runway → flightline →
windsock → tree boundary/background**. Nothing random is scattered over the
field; every addition has a purpose.

## Runway composition

Still 120 m × 12 m, long axis along Z (NED North = render −Z); surface and
marking colors only (no textures):

| Element        | Geometry                                   | Color                          | Height                 |
| -------------- | ------------------------------------------ | ------------------------------ | ---------------------- |
| Surface        | single quad                                | `[0.35, 0.33, 0.30]` asphalt   | `ground + 0.020 m`     |
| Edge markings  | 2 solid lines, 0.4 m wide, inside edges    | `[0.88, 0.88, 0.86]`           | `ground + 0.025 m`     |
| Threshold bars | 2 full-width bars, 1.5 m deep, between the edge lines | `[0.92, 0.90, 0.88]` | `ground + 0.025 m` |
| Centerline     | 10 dashes (6 m dash, 6 m gap), 0.6 m wide  | `[0.90, 0.90, 0.88]`           | `ground + 0.025 m`     |

The markings sit 5 mm above the surface (`RUNWAY_MARKING_OFFSET_M`) to avoid
z-fighting, a pattern continued from G2A. Dashes start symmetric about the
runway origin and are separated from the threshold bars, so no marking is
coplanar with another marking. All colors are high-contrast but not emissive.

## Flightline

One RC pilot line beside the runway, outside the runway safety rectangle
(safety half-width = 9 m; fence at x = +12 m, stations at x = +14 m), along
Z from −50 m to +50 m:

- **Fence** (`SceneryVisualKind::Fence`): 11 square posts (1.2 m tall, 9 cm)
  every 10 m, two thin rails per span (top 1.05 m, mid 0.54 m). Low and
  open — reads as a safety fence, not a wall.
- **4 pilot stations** (`SceneryVisualKind::Marker`): orange post + plate
  (1.0–1.7 m) facing the runway, at z = −30 / −10 / +10 / +30 m.

Both reuse the existing `SceneryVisualKind` variants. No collision geometry.
Objects are recorded in `SceneryScene.objects` for tests/debugging.

## Windsock

At its established position (x = 15 m, z = runway threshold + 10 m =
+70 m):

- vertical pole, 6 m tall;
- small horizontal boom at the top (the support the sock hangs from);
- sock: clearly **horizontal** tapered 5-segment tube, 2.2 m long
  (mouth 0.28 m → tip 0.14 m), with 3 alternating bands
  (orange / white / orange) via vertex colors — the classic RC-windsock
  silhouette, recognizable from the field.

Static: no animation, no wind coupling, no physics.

## Deterministic vegetation variation

Same seed + same 50 trees, same exclusion (trees keep ≥ 20 m from the
runway safety rectangle), same initialization-only generation. Each tree
gets a `TreeVariant` that is a **pure function of (tree_seed, tree index)**,
derived from the existing hash-based PRNG (`scramble`/`to_unit` — no RNG
crate, no runtime randomness):

| Property      | Range                        | Effect                                |
| ------------- | ---------------------------- | ------------------------------------- |
| height_scale  | 0.80–1.25                    | trunk and canopy height               |
| canopy_radius | 0.85–1.30 (× 1.2 m base)     | canopy width                          |
| yaw_rad       | −0.40…+0.40 rad              | apex lean direction/amount            |
| silhouette    | 50/50 conifer vs rounded     | pointed cone vs two-ring dome         |
| canopy color  | independent per-channel      | natural dark-green spread             |

Total tree height stays in the 2.8–5.6 m band: no giants, no matchsticks,
visible but sober variation. The apex lean makes "yaw" visible on an
otherwise rotationally symmetric canopy.

## Single merged mesh / zero runtime generation

`generate_flying_field` still returns `SceneryScene { mesh, objects }` with
one `SceneryMesh`. `gpu.rs` consumes `scene.mesh` exactly as before.
`SceneryMesh::triangle_count()` is added as a convenience reader.

## Geometry counts

| Metric          | G2A base (fac43fb) | G2C      | Δ      |
| --------------- | ------------------ | -------- | ------ |
| Vertices        | 1 380              | 5 360    | +3 980 |
| Indices         | 6 840              | 9 384    | +2 544 |
| Triangles       | 2 280              | 3 128    | +848   |

Increase is a fraction of the explicit budget
`MAX_FLYING_FIELD_TRIANGLES = 8 000` and remains a single draw call.
Most of the vertex growth comes from per-face normals (flat shading) on the
tree canopies. Breakdown (triangles): runway +22 (30 total), trees +400
(2 400), fence new 372, pilot markers new 56, windsock +30 (62), runway
poles unchanged 240.

## Limits

G2C is **not yet AAA scenery** and does not claim to be:

- no terrain materials, no detail normal maps, no texture usage anywhere;
- no dense vegetation, no ground cover / blade grass / billboards;
- no authored assets (everything is procedural);
- windsock is static, trees are static, flightline has no gameplay;
- no shadows or additional lighting (G2B handles directional shadows in
  parallel on another branch; this branch is scenery-only to avoid
  conflicts).

## Tests

Hardware-independent, in `scenery.rs` (all run in CI):

- generated field is not empty + geometry report (vertices/indices/tris)
- all indices in bounds; all vertex position/normal/color/uv finite
- centerline is 10 separate dashes (20 distinct z edges, spacing = dash
  length, never continuous)
- threshold markings at both runway ends
- flightline/fence and all fence vertices outside the runway safety
  rectangle; exactly 4 pilot markers at the configured stations
- windsock object present; sock above ground and extending horizontally
- tree variants deterministic per (seed, index) and inside their ranges;
  bit-identical per-object transforms across generations
- variation is not uniform (unique heights/yaws/greens ≫ half; both
  silhouettes present)
- no tree inside the expanded runway exclusion
- same params ⇒ same mesh and object layout (existing determinism tests)
- geometry budget below the explicit `MAX_FLYING_FIELD_TRIANGLES` ceiling
  and above a richness floor

No screenshot/pixel tests.