# G2E — Field Boundary Vegetation & Depth Cues

Status: implemented slice (renderer-only, scenery-only). Base commit:
`1575276d` (G2D final). Branch: `feature/g2e-field-boundary-vegetation`.

## Scope

G2E adds a distant, clustered belt of low-poly vegetation that frames the
FlyingField and gives the scene ambient depth, without turning the renderer
into a vegetation engine. It is purely **additive** on top of G2C/G2D:

- runway, centreline, threshold markings: untouched
- flightline fence and 4 pilot markers: untouched
- windsock: untouched
- safety rectangle: untouched
- the 50 deterministic near trees and their placement: untouched

Everything lives in `crates/renderer/src/scenery.rs` (plus this document).
The architecture is preserved exactly: `generate_flying_field(...)` runs once
at initialization, appends the belt into the **same single merged
`SceneryMesh`** (one vertex/index pair, one draw call), and `gpu.rs` is not
touched. No per-frame generation, no new GPU resources, no shaders, no
textures, no new dependencies.

## Visual objective

The field previously read as an empty plane with a handful of isolated trees.
G2E builds a convincing distant vegetation frame — the kind of tree line a
real RC field sits inside — with the readability hierarchy unchanged:
**aircraft → runway → flightline → windsock → distant vegetation**. The belt
is background scenery: desaturated, low-contrast, and kept spatially far from
everything that matters, so tracking the aircraft is never harder because of
the trees.

## Placement strategy

The belt is an annulus around the field centre:

| Constant | Value | Meaning |
| -------- | ----- | ------- |
| `BOUNDARY_VEGETATION_INNER_RADIUS_M` | 160 m | nearest allowed instance |
| `BOUNDARY_VEGETATION_OUTER_RADIUS_M` | 230 m | farthest allowed instance |
| `FIELD_HALF_EXTENT_M`               | 250 m | field square half-extent, never violated |

The band is "circa 160–230 m" from the centre: close enough to read as a
forest frame, far enough to leave the entire operational area untouched, and
well inside the 500 m field square (no corner overflow).

## Clustering (group → gap → group)

No uniform-random scatter. The ring is split into `BOUNDARY_CLUSTER_COUNT =
18` angular slots; each slot becomes one cluster whose members are scattered
around the slot centre:

- members stay within ±0.075 rad of the slot anchor, so a cluster reads as a
  tight copse (~30 m wide at 195 m radius) separated from the next cluster by
  a guaranteed empty band of ≥ 0.20 rad (~40 m);
- ~16% of slots are left **empty** (`APERTURE_PROBABILITY = 0.16`) — visible
  openings toward the horizon; the layout is never a "green wall";
- **side density modulation**: denser opposite the flightline
  (10–14 members per cluster for `cos θ < −0.25`, the −X side), sparse toward
  the flightline/windsock quadrant (3–5 members for `cos θ > 0.10`), moderate
  elsewhere (6–9). The flightline side stays visually open; the far side
  reads as a tree line.

The result is deterministic group → gap → group structure around the field,
not `tree tree tree tree` every 5 metres.

## Silhouettes

Four scenery-local low-poly silhouettes (no camera-facing billboards, no
alpha, no shaders, plain triangles that work with normal back-face culling).
All rise from the ground — no trunk, which is invisible at 160+ m and only
wastes triangles:

| Silhouette | `BoundarySilhouette` | Geometry | Triangles | Base size |
| ---------- | -------------------- | -------- | --------- | --------- |
| Conifer | `Conifer` | 5-segment cone + base cap | 10 | r 1.3 m, h 6 m |
| Rounded deciduous | `Deciduous` | 5-segment two-ring dome + cap | 20 | r 1.6 m, h 5 m |
| Narrow tall | `Tall` | 5-segment steep cone + cap | 10 | r 0.9 m, h 8 m |
| Shrub / hedge | `Shrub` | 5-segment squat cone + cap | 10 | r 2.0 m, h 2 m |

Average ≈ 12.5 triangles per instance — roughly 4× cheaper per tree than the
G2C near trees (48 triangles).

Per-instance variation, all deterministic from the tree seed:

- `height_scale` 0.80–1.20, `width_scale` 0.85–1.15, `yaw_rad` ±0.15 rad
  (subtle apex lean — no fluorescent, no extreme proportions, no obvious
  repetition);
- silhouette chosen from 4 with a deterministic unit draw;
- green tint is **desaturated and haze-blended** (`boundary_green`): each
  channel is blended toward an atmospheric grey-green by a per-instance
  factor, so the belt sits visually behind the saturated near objects.

## Determinism

Same `tree_seed` + `tree_count` + `FlyingFieldParams` ⇒ same scene, bit for
bit. The layout is a pure function of the seed using the existing
`scramble` / `to_unit` hash-and-LCG strategy — no `rand` crate, no thread
RNG, no time, no camera, no frame index.

## Safety distances

Instances are guaranteed clear of everything operational:

| Feature | Guaranteed clearance |
| ------- | -------------------- |
| Field centre / runway safety rectangle | ≥ 140 m (belt starts at 160 m) |
| Windsock (x 15, z 70) | ≥ 80 m |
| Flightline strip (x ≈ 13, z −50..50) | ≥ 80 m |
| Pilot stations, fence, runway | ≥ 140 m by construction |

Aircraft readability dominates: the belt is confined to the outer 40% of the
field and never occludes the runway-threshold view corridor.

## Metadata

Each belt instance is registered in `SceneryScene.objects` as
`SceneryVisualKind::BoundaryVegetation` with its position, yaw, height scale,
and a `variant_id` (0 Conifer, 1 Deciduous, 2 Tall, 3 Shrub) for
tests/debugging. The `SceneryObject` struct gained the `variant_id` field;
all pre-existing objects carry `variant_id: 0`. No scene graph, no ECS.

## Geometry cost

| Metric | G2D base | G2E | Δ |
| ------ | -------- | --- | - |
| Vertices | 5 360 | 10 100 | +4 740 |
| Indices | 9 384 | 14 124 | +4 740 |
| Triangles | 3 128 | 4 708 | +1 580 |

129 boundary instances add 1 580 triangles (avg 12.2/instance). The result
sits at 4 708 triangles — well under the preferred G2E target (≈ 6 500) and
far under the hard existing ceiling `MAX_FLYING_FIELD_TRIANGLES = 8 000`
which was **not** raised. The single merged `SceneryMesh` / one-draw-call
invariant is unchanged; the belt adds zero per-frame work and zero GPU
resources.

## Tests

Hardware-independent, in `scenery.rs` (all run in CI), on top of the
untouched G2C suite:

- boundary layout is deterministic (bit-identical positions and params)
- different seed ⇒ substantially different boundary layout
- instances stay within `FIELD_HALF_EXTENT_M` and inside the 160–230 m belt
- central operational area, windsock and flightline clearances hold
- belt is denser opposite the flightline (+X side), sparse side exists
- ≥ 3 silhouettes present, exposed both in layout and in object metadata
- height/width/yaw/green variation is non-uniform and inside controlled
  ranges
- cluster/gap structure: multiple gaps ≥ 4, ≥ 8 groups, longest run ≥ 5
- instances registered 1:1 in `SceneryScene.objects` with correct metadata
- merged scene stays ≤ 6 500 triangles (preferred G2E budget; the existing
  test keeps the ≤ 8 000 hard ceiling)

Existing G2C runway/flightline/windsock tests remain green and unchanged.

## Aircraft readability

Readability principle: **AIRCRAFT READABILITY > SCENERY DENSITY**. The belt
is far, muted, clustered with apertures, never uniform, and never enters any
operational zone. If the belt ever threatens readability, the correct fix is
to thin the belt — never to raise the triangle cap.

## Limitations

**G2E is NOT AAA vegetation.** Explicitly missing, by design:

- authored vegetation assets / models
- textures / normal maps / alpha cards
- proper vegetation materials (wind shading, subsurface scatter, PBR leaf
  response)
- LOD, frustum culling, instancing
- wind animation, sway
- dense grass / ground cover blades
- vegetation interaction (no collision, no physics, no gameplay)

Boundary vegetation is static, camera-independent geometry with vertex
colors only; adding any of the above is future work outside this slice.