# G2B - Stable Directional Shadows

Status: implemented slice (renderer-only). Base commit: `fac43fb` (G1D final pass).

## Scope

G2B adds the renderer's first shadow system: one production-oriented
directional shadow map for the existing sun. It prioritizes stable, readable
RC-aircraft presentation and predictable frame cost over cinematic softness.

It does not change simulation, aircraft/model data, application UI/CLI, or the
renderer-to-platform boundary. All `wgpu` resources remain in the `renderer`
crate.

Out of scope:

- cascades, point lights, spot lights, ray tracing, contact shadows, screen-space shadows
- temporal filtering, HDR, IBL, normal maps, additional PBR textures
- new assets, vegetation, UI, physics, model, platform, or OA1 work

## Pass architecture

Each render frame encodes two passes in this order:

1. **Directional shadow pass** - depth-only, clears the persistent shadow depth
   target to `1.0`, then renders terrain, scenery, rigid aircraft primitives,
   and articulated aircraft surfaces with the light camera.
2. **Existing main scene pass** - sky, lit terrain/scenery/aircraft, optional
   debug overlays. The G1D lit fragment comparison-samples the shadow map.

The shadow pipeline has no fragment stage and no color target. Sky and debug
overlays are deliberately not casters. All normal lit/PBR surfaces are
receivers. The direct G1D term is multiplied by shadow visibility; the existing
ambient/readability term remains independent, so shadowed aircraft do not turn
into black silhouettes. Fog is still applied after `ambient + direct`.

The shadow texture, comparison sampler, bind group, matrix buffer, and
depth-only pipeline are created once at renderer initialization. The frame path
only calculates a small matrix, writes the persistent shadow uniform, and
encodes the caster pass. Resizing the window
does not recreate the fixed shadow map.

## Bind-group boundary

G2B extends the existing environment bind group (group 2), keeping the lit
pipeline at its established groups `0..3`:

| Group | Binding | Resource |
| --- | --- | --- |
| 2 | 0 | `EnvironmentUniform` (existing sun, ambient, atmosphere) |
| 2 | 1 | depth texture view for the directional shadow map |
| 2 | 2 | linear comparison sampler |
| 2 | 3 | `ShadowUniform` (light view-projection + receiver bias) |

The main/lighting pass uses this complete group 2 and can therefore sample the
shadow texture normally. The shadow caster pipeline instead uses a persistent,
matrix-only group-2 layout with **only binding 3** (`ShadowUniform`, vertex
visible), backed by the same `shadow_matrix_buffer`. It never binds the full
environment group while its depth target is a render attachment, avoiding a
read/write alias between `RENDER_ATTACHMENT` and `TEXTURE_BINDING`.

The shadow vertex path retains `@group(2) @binding(3)` and uses the object
transform at group 1. Group 0 is bound with the existing camera group only to
keep the pipeline layout contiguous; the shadow vertex shader does not read it.
The pass needs neither material data nor a new global bind group.

## Frustum and temporal stability

The map is a persistent `Depth32Float` target at **2048 x 2048**. Its fixed
orthographic half extent is **128 m**, yielding:

```text
texel size = (2 * 128 m) / 2048 = 0.125 m per texel
```

The frustum tracks the aircraft. Before building the light camera, the tracked
world centre is projected on the light-space right/up axes and each coordinate
is rounded to the 0.125 m texel grid. Its light-space depth stays continuous.
Consequently small aircraft motion cannot translate the shadow projection by a
sub-texel amount and cause shimmering; the XY shadow origin changes only after
a texel boundary is crossed.

The camera direction is the exact normalized value written to
`EnvironmentUniform.light_direction`, so the visible sun, direct G1D light,
and shadow direction agree. A safe alternate up vector is selected when the
light is nearly parallel to render-space world up, preventing a degenerate
light basis or NaN transform.

The orthographic camera is placed 384 m toward the light from the snapped
centre and uses a 1 m near plane and 768 m far plane. This leaves a reasonable
local vertical slice for the aircraft and nearby terrain without adding
cascades.

## Bias and filtering

Caster rasterization uses centralized `wgpu::DepthBiasState` values:

- constant bias: `2` depth units
- slope-scale bias: `2.0`
- clamp: `0.0`

The receiver comparison additionally subtracts a small normalized depth bias
of `0.00015`. Together these conservative values mitigate acne without
front-face culling, which could make thin RC wings or articulated surfaces
disappear from the caster pass.

The shadow map uses a `LessEqual` comparison sampler with linear min/mag
filtering. This gives a compact hardware-filtered (2x2 PCF-style) visibility
edge at low fragment cost. No 5x5/9x9 or temporal kernel is used. Coordinates
outside the fixed light frustum explicitly return visibility `1.0` rather than
sampling an edge texel.

## Performance and lifetime

Expected cost per frame:

- one additional 2048^2 depth-only render pass;
- caster draw calls repeated for terrain chunks, optional scenery, rigid
  aircraft geometry, and articulated surfaces;
- one persistent 2048^2 `Depth32Float` texture;
- one comparison sample in each lit fragment;
- one small shadow-matrix uniform-buffer write.

There are no new main-pass draw calls and no avoidable heap allocation, GPU
resource creation, pipeline creation, or bind-group creation in the render hot
path. The current terrain is drawn as its existing chunks and clipped by the
fixed light frustum; G2B deliberately does not add a shadow-specific culling
system.

## Hardware-independent coverage

Pure renderer tests cover:

- finite light view-projection values;
- texel-size derivation from extent and resolution;
- unchanged snapped centre for sub-texel lateral motion;
- centre update after movement beyond one texel;
- finite result for a nearly world-up light direction;
- valid frustum, map, and bias constants;
- structural guard that the frame path only updates persistent shadow buffers,
  not shadow resources, and that the caster pass binds its matrix-only group
  instead of the full sampled environment group;
- WGSL source guards for direct-only shadow modulation, unshadowed ambient,
  fog after lighting, and comparison sampling.

## Known limits

- One directional map only; no cascades or distant coverage tier.
- Resolution is concentrated in the fixed local 256 m square. Quality degrades
  beyond that region and outside it receivers are intentionally fully lit.
- Filtering is small and stable rather than cinematic; large soft penumbrae
  are not a goal of this slice.
- Bias values are conservative defaults and may require visual tuning on a
  wider range of field assets and GPU drivers.
- This is not an AAA-final shadow solution.
