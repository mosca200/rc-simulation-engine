# G2D Terrain Surface Presentation

Status: implemented slice (renderer-only, terrain-only). Base commit:
`c20c1b1` (G2C final pass). Branch:
`feature/g2d-terrain-surface-presentation`.

## Scope

G2D improves the deterministic ground appearance of the terrain mesh using
**only CPU-side data already computed during chunk generation**. The color
of every terrain vertex becomes the output of a pure, deterministic function
of its render-space position (plus the local elevation and normal). Nothing
else changes:

- no new texture, no shader change, no new GPU resource, no new draw call;
- no per-frame CPU work: colors are baked once at initialization into the
  existing `Vertex.color` field;
- geometry (positions, normals, UVs, indices, bounds, chunk topology) is
  bit-for-bit unchanged;
- physics is untouched: the terrain remains presentation-only.

## Problem before G2D

Before G2D the terrain baked `TerrainMaterial.base_color_factor` verbatim
into every vertex color. The result was technically correct (geometry,
normals, UVs, chunking all present) but perceptually a flat "green sheet":
a single uniform color over the whole field, with no sense of scale, depth,
or natural variation. The scene read as "technical renderer output" rather
than a field.

## Design constraints

- **CPU initialization only** — vertex colors are computed once at chunk
  generation; the renderer pipeline is unchanged.
- **Deterministic** — no RNG, no time, no platform state; same input →
  same color, bit-for-bit.
- **World-space** — the pattern is anchored to the render-space coordinates
  used by `Vertex.position`, never to chunk-local grid coordinates.
- **Low frequency** — the two noise layers have metric wavelengths of
  90 m and 25 m; no high-frequency detail that could alias against the
  terrain tessellation.
- **Bounded / sober** — every output channel stays within ±0.26 of the
  material base color; no black or near-white patches, no hue jumps.
- **Readability** — aircraft readability wins over terrain richness; the
  ground must never compete with the aircraft for attention.

## Surface model

Per vertex:

```
color = base * luminance * tint_factor, clamped to [0,1]
alpha = base.alpha                       (exactly, never modified)
```

where `base` is `TerrainMaterial.base_color_factor` — the material remains
the chromatic authority at every vertex. The modulation is:

1. **macro luminance variation** — slow brightness patches (90 m
   wavelength), breaking the overall uniformity of the field;
2. **medium luminance variation** — gentler patches (25 m wavelength),
   preventing large flat-looking surfaces;
3. **slope response** — `(1 - normal_y)` clamped to `[0,1]`, brightening
   steeper surfaces slightly (exposed/drier grass). Flat ground
   (`normal_y == 1`) contributes exactly zero;
4. **elevation response** — a tiny linear bias with height, hard-capped, so
   higher ground reads a little lighter without assuming any global
   elevation range;
5. **warm/cool tint** — a small per-channel tint (25 m scale, independent
   seed): positive tint lifts R and drops B (drier patches), negative does
   the opposite (denser green). Amplitude is small enough that no hue jumps
   are visible.

## Spatial scales

| Layer        | Wavelength | Value  |
| ------------ | ---------- | ------ |
| Macro        | 90 m       | `TERRAIN_MACRO_SCALE_M` |
| Medium       | 25 m       | `TERRAIN_MEDIUM_SCALE_M` |
| Tint         | 25 m       | (medium scale, separate seed) |

No super-macro layer (200–400 m) was added: the 1 km production field is
already well covered by the 90 m macro layer, and the optional layer did not
improve the design enough to justify the extra evaluation.

## Amplitude limits

All values are fractions of the base color and are conservative on purpose:

| Constant                          | Value | Meaning                                  |
| --------------------------------- | ----- | ---------------------------------------- |
| `TERRAIN_MACRO_VARIATION`         | 0.10  | peak macro luminance swing               |
| `TERRAIN_MEDIUM_VARIATION`        | 0.05  | peak medium luminance swing              |
| `TERRAIN_SLOPE_VARIATION`         | 0.04  | peak slope brightening (fully vertical)  |
| `TERRAIN_ELEVATION_GRADIENT_1_PER_M` | 0.0015 | elevation bias per metre of height    |
| `TERRAIN_ELEVATION_MAX_BIAS`      | 0.02  | cap on the elevation bias                |
| `TERRAIN_TINT_VARIATION`          | 0.04  | peak per-channel tint                    |
| `TERRAIN_MAX_CHANNEL_DEVIATION`   | 0.26  | hard bound on any output channel's deviation from base |

The worst mathematically-achievable per-channel deviation is 0.2584
(luminance swing 0.21 aligned with the full tint swing); the bound of 0.26
adds rounding margin. Typical vertices land well below it, because the noise
layers rarely align. For a default grass base `[0.25, 0.45, 0.18, 1.0]` the
colors stay within roughly `[0.79, 1.21]` of the base in relative terms —
never near black or white.

## Noise/hash strategy

A minimal, self-contained 2D lattice **value noise**:

1. render-space (x, z) divided by the layer's metric scale → lattice cell;
2. four lattice corners hashed with a deterministic integer mixer
   (`lattice_hash_unit`): wrapping multiply/xor mixing of the cell indices
   and a fixed per-layer seed → value in [0, 1];
3. bilinear blend of the four corner values with a smoothstep kernel
   `t*t*(3-2*t)` (zero derivative at both ends → no kinks);
4. remap [0, 1] → [-1, 1].

The hash uses wrapping integer arithmetic only: no allocation, no RNG, no
platform state, identical on every build and platform. Negative
coordinates are handled deterministically through `as u32` wrapping
(defined behaviour in Rust). Seeds are fixed constants per layer.

## Chunk seam prevention

The pattern is a pure function of **final render-space coordinates** — the
same coordinates used for `Vertex.position`. Adjacent chunks share their
boundary vertices at identical f32 render coordinates (the grid-to-render
arithmetic is identical on both sides), so the color function returns
bitwise-identical colors on shared X and Z boundaries. There is no
chunk-local state anywhere in the calculation, so seams cannot appear.

## Chunk-size independence

The pattern authority is the render-space position, not the chunk. Changing
`cells_per_chunk` moves grid indices but never the render-space position of
a vertex, so the color at a given world position is unchanged. This is
verified by a dedicated test that compares colors across two different
chunkings on identical world positions (including negative-coordinate
regions).

`texture_scale_m` is likewise decoupled: UVs remain
`render / texture_scale_m` (unchanged), while the G2D noise uses its own
metric scales. Changing the texture scale resizes the UV tiling only.

## Flat terrain preservation

Visual richness is added **without touching the physical-compatible flat
surface**: elevation, positions, normals, UVs, indices, bounds and chunk
topology stay exactly as before. Because flat-terrain normals are exactly
`[0, 1, 0]`, the slope term contributes exactly zero, and the flat surface
gains only the world-space brightness/tint variation.

## Rolling terrain preservation

`generate_rolling_terrain` is untouched: same frequencies, same amplitudes,
same elevation equations. G2D only *reads* the computed elevation and normal
to tint the color. Dedicated tests assert that geometry fields (including
bounds) are bit-for-bit independent of the color modulation.

## Temporal stability

The surface variation is fully static: it depends only on position,
elevation and normal — never on time, frame index, camera position or any
per-frame input. A given vertex has the same color in every frame, which
removes a whole class of shimmer.

## Aircraft visibility

Ground contrast is deliberately kept well below the aircraft's visual
weight: the hardest guaranteed deviation from the base color is ±0.26 per
channel, no strong patterns, no fine frequencies, no near-black or
near-white patches. The principle is documented and enforced by tests:
**aircraft readability > terrain richness**.

## Performance

- Initialization-only: the color function runs once per vertex during chunk
  generation.
- No new GPU resource, no new texture, no new draw call.
- **Per-frame G2D cost: 0 additional terrain presentation work** — the GPU
  already processes vertex colors.
- The color function is allocation-free: a few integer hashes, a few
  `floor`s, one smoothstep pair and a handful of f32 multiplies per vertex.
  No Vec, String, HashMap, recursion, dynamic dispatch, lock or global
  state.

## Determinism

The whole color path is a pure mapping: same inputs → same output
bit-for-bit (`to_bits` equality is asserted in tests). The integer hash and
the f32 interpolation are IEEE-deterministic given identical inputs, and the
inputs (render coordinates, elevation, normal) are themselves computed
deterministically from the height field.

## Tests

New regression tests in `crates/renderer/src/terrain.rs` (module
`terrain::tests`) cover, among others:

- flat terrain vertex/index/triangle counts, exact requested elevation,
  exact grid positions, exact upward normals, unchanged UV mapping;
- all colors finite, in [0, 1], within the explicit deviation bound;
- alpha preserved exactly (default and custom materials, and at the pure
  function level);
- large terrain produces many distinct colors (variation exists);
- same-input-same-output bitwise determinism of the color function, of both
  noise layers, and of a full double generation;
- noise finite/bounded on positive and negative coordinates, at the origin,
  near lattice boundaries, and continuous across lattice boundaries;
- hash bounded in [0, 1] over a signed index sweep;
- shared X and Z boundary colors identical between adjacent chunks (flat and
  rolling);
- render-origin offset and centered-terrain seams resolve identically;
- chunk-size independence, and texture-scale independence of the noise scale;
- custom material base color respected (authority), including custom alpha;
- slope response zero on flat and bounded on steep normals; elevation bias
  finite/bounded;
- geometry fields bit-for-bit independent of the color modulation;
- bounds contain all vertices; no unexpected empty chunks.

## Limitations

G2D does **not** make the terrain AAA. Still missing:

- authored albedo and aeromodelling-field detail;
- detail normal maps and roughness variation textures;
- material blending / splat maps on the terrain;
- grass rendering and dense vegetation;
- authored terrain and terrain LOD;
- terrain culling optimization;
- physics/visual heightfield convergence.

The improvement here is limited to a sober, low-frequency brightness/tint
variation baked into vertex colors. It is a presentation-layer stopgap, not
a terrain engine.

## Future compatibility

The G2D color function produces a conservative base tint that can coexist
with future texture work: it remains the fallback/albedo base for the
existing `vertex_color * texture` pipeline, and it deliberately does not
require any future rendering architecture. When authored albedo or splat
textures arrive, this layer can be kept underneath or dropped per-region
without touching the rest of the renderer; the function is self-contained
and easy to remove or replace.