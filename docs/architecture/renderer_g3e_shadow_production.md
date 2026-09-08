# G3E - Stable Three-Cascade Production Shadows

Status: implemented renderer-only slice. Exact base: `23ae2381887994eca2f4eeef54274d7fc4f94787`.

## Scope and priorities

G3E replaces the G2B single directional map with exactly three stable cascades.
The implementation is intentionally specific to the existing outdoor sun and
RC field: temporal stability first, aircraft/runway quality second, then
predictable cost and vegetation integration. It does not change physics,
camera behavior, terrain material, the aircraft asset, or vegetation assets.

Out of scope are PCSS, EVSM, temporal shadow filtering, ray tracing, raw Vulkan,
and a general-purpose light/shadow framework. No shadow-debug CLI is exposed in
this slice because a partial cascade visualization would not meet the command's
contract.

## Central cascade policy

`shadow.rs` owns the only cascade count, splits, extents, resolution, snapping,
and bias constants:

| Cascade | Receiver distance | Light-space half extent | World texel |
| --- | ---: | ---: | ---: |
| Near | 0-32 m | 40 m | 0.0390625 m |
| Mid | 32-128 m | 128 m | 0.125 m |
| Far | 128-512 m | 512 m | 0.5 m |

Every layer is 2048 x 2048 `Depth32Float`. Fixed extents avoid projection-scale
changes. The generous overlap between each receiver interval and its square
projection keeps the chase-camera frustum covered at the established field FOV.
Receivers beyond 512 m are explicitly unshadowed.

For each cascade, the tracked center lies halfway through its receiver interval
along the active camera view direction. The center is projected onto the fixed
directional-light right/up basis, independently rounded to that cascade's world
texel grid, and reconstructed before the orthographic matrix is built. Light
depth remains continuous. Translation smaller than a texel therefore cannot
translate the projected sampling grid; tests pin that behavior for all three
cascades and also verify the first update beyond the near texel.

## GPU architecture and lifetime

One persistent 2048 x 2048 x 3 depth texture array is created during renderer
initialization. One D2-array view is sampled by the lit pipelines and three D2
layer views are render attachments. Receiver state is one persistent 240-byte
uniform (three matrices plus split, bias, and texel vectors). Three persistent
64-byte caster uniforms and bind groups isolate the attachment pass from the
sampled texture binding.

The frame path performs four bounded queue writes, builds three stack-only
`ShadowCascade` values, and encodes exactly three depth passes. It creates no
shadow texture, buffer, sampler, pipeline, bind group, or heap collection per
frame. Window resize does not recreate the fixed shadow array.

Each caster pass preserves the established object-transform contract:

- scenery uses the identity object group;
- G3D vegetation uses the existing instanced shadow pipeline and one draw per
  active `(asset, LOD, part)` batch, never one draw per tree;
- LOD0 and LOD1 vegetation cast; economical far LOD2 does not;
- rigid aircraft primitives use the aircraft root object group;
- aileron L/R, elevator, and rudder use their independent articulated object
  groups, after explicitly restoring the non-instanced pipeline required by
  the `23ae238` regression fix.

The terrain remains a normal lit shadow receiver. Its coarse long-range height
mesh does not self-cast: in the base single-map path it could behave as a
map-sized false occluder across the runway, and extending that artifact across
the near cascade made the operational field unreadable. Scenery, aircraft, and
batched vegetation still cast onto the terrain, preserving contact shadows
without a per-terrain-chunk caster cost or a large peter-panning bias.

Raw shadow-array storage is 50,331,648 bytes (48 MiB), versus 16 MiB for the
former single map. The shadow uniforms add 432 bytes. Texture views, bind
groups, sampler, and pipeline driver overhead are implementation-dependent and
are not estimated as texel VRAM.

## Filtering and bias

The receiver shader selects the first split containing the Euclidean
camera-to-fragment distance, transforms with the matching matrix, and samples
the matching array layer. It evaluates a fixed 3 x 3 PCF grid: exactly nine
comparison-sampler calls per shadowed fragment, independent of cascade and
scene content. Sampling outside a selected light frustum returns fully lit.

Caster raster bias is shared and centralized: constant `2`, slope scale `2.0`,
clamp `0.0`. Receiver bias is 0.06 m converted into each cascade's normalized
depth range (approximately 0.000290 near, 0.000157 mid, and 0.000052 far). The
world-space policy stays below two near texels, suppressing interpolation acne
while retaining grounded aircraft and runway contact; terrain self-casting is
not hidden with an excessive receiver bias.

## Draw cost and validation

Per-frame shadow draw calls are:

```text
3 * (optional scenery batch + active LOD0/LOD1 vegetation batch-parts
     + rigid aircraft primitives + articulated aircraft primitives)
```

The FlyingField chase runtime reports vegetation shadow calls after the
three-cascade multiplier. LOD2 is excluded by policy. Structural tests pin the
persistent resource boundary, exact three-pass loop, depth-array sampling,
fixed nine-tap PCF, terrain receiver-only policy, and absence of shadow resource
creation in the frame path. Pure tests cover split ordering and bounds,
cascade selection, finite matrices, texel derivation, and sub-texel snapping.
The ignored GPU suite covers each written array layer, vegetation silhouettes,
and aircraft transform isolation after an instanced vegetation shadow draw.

GPU shadow-pass milliseconds and FPS/1% low require a timestamp/profiling path
not present in this slice; they must be reported as not measured rather than
inferred from CPU or screenshot data.

## Known limits

- Cascade transitions are deterministic hard selections; there is no blend
  band. Fixed extents overlap enough for coverage, not for cross-fading.
- Far-cascade quality is deliberately economical at 1 m per two texels and
  LOD2 vegetation does not cast.
- The terrain receives object shadows but does not cast its own large-scale
  relief, as documented above.
- There is no cascade debug visualization CLI in this bounded slice.
