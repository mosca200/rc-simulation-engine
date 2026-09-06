# G3A Production Terrain Material Foundation

Status: implemented slice (renderer-only, terrain-only). Base commit:
`365acce` (integration/g2-graphics-convergence). Branch:
`feature/g3a-production-terrain-material`.

## Outcome

The terrain is no longer a flatly colored surface. G3A turns it into a
textured, PBR-lit grass surface that is immediately distinguishable in
screenshots:

- grass/ground **albedo texture** as the chromatic authority;
- tangent-space **detail normal map** perturbing the lighting;
- texture-driven **roughness** around a material base;
- **world-space tiling** that is chunk-size independent, seamless across
  chunk boundaries, and camera independent;
- G2D deterministic macro variation kept on top as the brightness/tint layer;
- full G2B directional-shadow receiving through the shared PBR path;
- zero impact on the flight core (terrain remains presentation-only).

## Design

### One material, one pipeline, reused PBR

The G3A terrain is rendered by a **dedicated terrain pipeline** that is
identical to the established lit triangle pipeline except for its fragment
entry point (`fs_terrain`) and its material bind group (group 4, see GPU
resources). The shader body of `fs_lit` was extracted verbatim into a shared
`lit_pbr_response` helper plus an `apply_distance_fog` helper; both fragment
paths evaluate the exact same operands in the same order, so `fs_lit` is
bit-compatible with the pre-G3A look and G3A cannot introduce per-frame
divergence between terrain and aircraft lighting.

The terrain vertex buffer layout is **unchanged** (`position, normal, color,
uv`). The tangent frame used to apply the normal map is reconstructed per
fragment from screen-space derivatives of the interpolated world position and
UV — no per-vertex tangent attribute, no second vertex layout, no GLB-side
change. The reconstruction is exact on the flat plane (unit, orthogonal
frames), follows the actual surface tilt on rolling terrain, and falls back
to the world-aligned frame on degenerate fragments so the math is always
finite.

### Texture/material design

Three deterministic, seamlessly tileable maps are generated offline
(`crates/renderer/src/terrain_textures.rs`), versioned in the repository at
`crates/renderer/assets/terrain_grass_*.png`, regenerable byte-for-byte with:

```text
cargo run -p renderer --bin generate_terrain_textures
```

and embedded into the renderer binary with `include_bytes!`. Nothing is
generated per frame; the maps are decoded once at renderer initialization.

| Map        | Format             | Content                                    |
| ---------- | ------------------ | ------------------------------------------ |
| albedo     | RGBA8 sRGB (512²)  | multi-octave green grass field, sparse dry |
|            |                    | patches, blade-level mottle, alpha = 255   |
| normal     | RGBA8 linear (512²)| tangent-space relief from a periodic       |
|            |                    | height field, ±~30/255 XY around 128       |
| roughness  | R8 linear (512²)   | wet/dry patch field, mean ≈ 0.80, range    |
|            |                    | ≈ [0.54, 1.0]                              |

The generator is a pure function of the texture size: same size → same pixels
bitwise on every build and platform. A regression test regenerates the set and
compares it pixel-for-pixel against the committed PNGs, so the versioned
assets and the generator cannot drift apart.

**Seamless tiling** is guaranteed by construction: every noise octave uses a
lattice corner hash that is periodic (`rem_euclid` on the per-axis cell
count) and texels are sampled at texel centers, so texel `SIZE` is bitwise
identical to texel `0`. With `AddressMode::Repeat` no tile edge can appear.
A dedicated test asserts the wrap seam never jumps more than the interior
neighbouring-texel delta.

### Coordinate system and tiling scale

- Terrain stays in render space: +Y up, XZ horizontal — unchanged from G1C.
- The world-space UV is `uv = render_position / texture_scale_m` with
  `texture_scale_m = 4.0` by default, untouched from G2D. One tile = 4 m;
  the 512² maps read at ~7.8 mm/texel.
- The normal and roughness maps are sampled with the same tile size but
  **per-map UV anchors** (`normal_uv_offset = (0.271, 0.137)`,
  `roughness_uv_offset = (0.413, 0.303)` in tile units) so their tile borders
  never align with the albedo's, breaking perceived repetition at zero cost.
- Chunk size, chunk origin, and camera never enter the mapping: the shader
  only sees final render-space positions and the shared per-map uniforms, so
  chunk-size independence, seam-free chunk boundaries, and camera stability
  hold by construction and are pinned by tests.

### Normal convention and strength

The normal map follows the OpenGL/Blender convention: RGB ∈ [0, 1] maps
tangent-space XYZ ∈ [-1, 1]; `(128, 128, 255)` is flat, and on the terrain
tangent-space +Z is world-up. The map is stored in a **linear** format (no
hardware sRGB conversion). The shader decodes it to [-1, 1], scales the XY by
`TerrainMaterial.normal_strength` (default 1.0), rebuilds Z for unit length,
and perturbs the geometric normal through the fragment TBN. The committed
amplitude (±30/255) is visible under the sun but credible.

### Roughness

`TerrainMaterial.roughness` (default 0.9) is the material base;
`fs_terrain` multiplies it by the roughness map sample and clamps to
`[MIN_ROUGHNESS, 1]` (MIN_ROUGHNESS = 0.06, shared with `fs_lit`). Terrain
`metallic` is clamped to 0.0 (dielectric). The result is a matte, textured
specular response consistent with the G1D model.

### Relationship with G2D

G2D's deterministic macro variation is preserved exactly as the **carrier
layer**: the material base color factor is now white, so the vertex color
baked at chunk generation carries only the G2D ±~20% brightness/tint
modulation, and `fs_terrain` multiplies `vertex_color × albedo_sample`. The
texture is the chromatic authority; G2D breaks uniformity across tiles; the
worst-case per-channel deviation bound (±0.26) is unchanged and still tested.

### GPU resources

- One **terrain material bind group** (group 4):
  - binding 0: albedo `Rgba8UnormSrgb`
  - binding 1: shared repeat/linear sampler (serves all three maps)
  - binding 2: normal `Rgba8Unorm`
  - binding 3: roughness `R8Unorm`
  - binding 4: uniform `TerrainMaterialUniform` (64 bytes: metallic,
    roughness, normal_strength, three vec2 UV anchors, padding)
- One dedicated **terrain pipeline** (same vertex layout/raster state as the
  lit triangle pipeline, `fs_terrain` fragment, group-4 layout; group 3 unused).
- All resources are created once at renderer initialization; the frame path
  only sets the pipeline and bind group. Texture memory: albedo 1 MiB +
  normal 1 MiB + roughness 256 KiB ≈ 2.25 MiB.
- The device `max_bind_groups` limit is raised from the WebGPU default of 4
  to the adapter's advertised value (native backends advertise up to 8) so a
  fifth bind group is legal.

### Performance cost

- **Draw calls**: unchanged — the terrain keeps the existing chunk batching;
  the only change is one pipeline bind and one bind-group bind for the terrain
  block per frame.
- **Per-frame CPU work**: none added. No allocation, no resource creation in
  the frame path (regression-guarded by source tests).
- **Per-fragment GPU work**: three 2D texture samples, one derivative TBN
  (2× `dpdx`/`dpdy`), one normalize — negligible on the RTX 3090 reference and
  well within class-range GPUs.

### Draw path

Shadow caster pass: unchanged (`vs_shadow`, depth-only), so G3A terrain casts
and receives the G2B directional shadow exactly as before. Main pass:
sky → terrain (`terrain_pipeline`) → scenery (shared lit pipeline, restored
explicitly) → debug overlays → aircraft/surfaces.

## Tests

CPU-only regression coverage in `crates/renderer`:

- `terrain_textures`: bitwise determinism; committed-asset-vs-generator
  equality; 512² decode of the embedded PNGs; format/pixel-format checks;
  normal convention (flat = 128/128/255, unit vectors, XY amplitude bounds);
  roughness range; periodic wrap (texel SIZE ≡ texel 0) and seamless-seam
  bounds for all three maps; bounded periodic noise.
- `terrain` (G3A): pinned deterministic material configuration (white base,
  4 m scale, metallic 0, roughness 0.9, strength 1.0, finite offsets); UV
  anchored to render-space; UV + G2D color identical across chunkings and
  across adjacent chunk boundaries (X and Z); finite/unit TBN on rolling
  terrain, exact `[1,0,0]`/`[0,0,1]` frame on flat terrain, finite
  world-aligned degenerate fallback.
- `gpu` (G3A): uniform struct is exactly 64 bytes and round-trips; factors
  clamped at load; structural guards that the frame path never recreates
  terrain textures/samplers/bind groups/pipelines and that terrain draws bind
  their own material at group 4.
- `tests/shader_wgsl_g3a.rs`: the whole `shader.wgsl` parses **and** validates
  with naga (pinned `=30.0.1`, the wgpu 30 front end), catching WGSL
  regressions on CPU-only runners; plus entry-point structure guards.
- All pre-existing G2D determinism tests still pass unchanged.

## Smoke result

`rcsim-app render --scenery flying-field` (release) initializes the renderer,
builds every pipeline, and renders frames with zero GPU validation errors —
before the `max_bind_groups` fix the run failed with a validation error, after
the fix it ran continuously. The terrain pipeline is exercised every frame;
the result was confirmed visually through a live window capture. Screenshot
persistence was unavailable in the driving environment (capture output was
not written to disk), so the evidence is the clean runtime plus the captured
frame description.

## Current limits

Out of scope for G3A, deliberately deferred to future slices:

- no per-vertex tangents (derivative TBN is fragment-local; a small shading
  discontinuity can appear where a 2×2 pixel quad straddles a chunk diagonal);
- no mipmapping (single 512² mip; minification relies on bilinear filtering);
- no splatting / multi-biome blending; no grass blades, tessellation,
  parallax, displacement;
- no authored terrain art — the maps are procedurally generated and versioned;
- terrain texture scale is global; no per-region material variation.

The G2D color layer, the derivative TBN, and the dedicated terrain pipeline
are all self-contained: a future slice (splat maps, authored albedo, per-region
materials) can replace each piece independently without touching the rest of
the renderer.