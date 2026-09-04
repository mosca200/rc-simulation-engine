# G1B — Procedural sky, horizon haze, and distance fog

G1B replaces the flat clear-color background with a procedural outdoor atmosphere foundation
suitable for an RC flight simulator. It adds a view-direction-based sky gradient, a procedural sun
disk, horizon haze, and exponential distance fog applied to scene geometry.

G1B is intentionally simple. It is not a photorealistic atmosphere milestone.

## What G1B adds

1. Procedural sky rendered as a fullscreen triangle background pass.
2. View-direction-based horizon that moves correctly with camera pitch and roll.
3. Horizon haze that softens the sky-to-atmosphere transition.
4. Procedural sun disk coherent with the G1A directional light.
5. Exponential distance fog applied to aircraft and ground.
6. Persistent environment GPU state (single uniform for all atmosphere parameters).
7. Correct integration with existing G1A directional lighting.
8. Zero per-frame heap allocation.

## Procedural sky approach

The sky is rendered as a fullscreen triangle (3 vertices from `vertex_index`, no vertex buffer)
before any scene geometry. The vertex shader outputs clip-space positions at the far plane
(`z = 1.0, w = 1.0` in WebGPU clip depth). The fragment shader reconstructs a world-space view
direction for each pixel.

The sky pipeline uses:
- `depth_write = false`
- `depth_compare = Always`

This ensures the sky always draws as background and never writes to the depth buffer. Scene
geometry then renders normally with depth writes enabled and `Less` comparison.

## View-direction reconstruction

Each sky fragment reconstructs its world-space view direction using the inverse view-projection
matrix:

```wgsl
let clip_far = vec4<f32>(clip_xy, 1.0, 1.0);  // far plane
let world_h = inv_view_projection * clip_far;
let world_pos = world_h.xyz / world_h.w;
let view_dir = normalize(world_pos - camera_position);
```

The CPU computes `inv_view_projection` via a general 4×4 cofactor-expansion inverse
(`Mat4::inverse()`). If the matrix is singular (should not occur with valid camera parameters),
the identity is used as a safe fallback.

Key properties by construction:
- Camera translation does NOT move the sky (the view direction is normalized, so distance is
  irrelevant — the sky is effectively at infinite distance).
- Camera rotation DOES rotate the viewed sky (the view direction changes with camera attitude).
- The horizon moves correctly when the camera pitches up, pitches down, or rolls.

## Renderer world-up convention

The render coordinate system is right-handed:

```text
Render +X = East
Render +Y = Up
Render +Z = South
```

World up is `+Y` (`RENDER_WORLD_UP = [0.0, 1.0, 0.0]`). This is consistent with the NED-to-render
mapping where physics Down (NED +Z) maps to render −Y, so physics Up maps to render +Y.

The world-up vector is used consistently for:
- Sky elevation calculation
- Horizon determination
- Sun position reference
- Chase camera preferred-up axis

## Sky color model

The sky color is derived from the view elevation:

```text
elevation = dot(view_direction, world_up)
```

- `elevation = +1.0` → zenith (straight up)
- `elevation =  0.0` → horizon
- `elevation = −1.0` → nadir (straight down)

Above the horizon, a nonlinear power curve maps elevation to a blend factor:

```wgsl
t = pow(elevation, 0.5)
sky = mix(horizon_color, zenith_color, t)
```

Below the horizon:

```wgsl
t = pow(clamp(-elevation, 0.0, 1.0), 0.7)
sky = mix(horizon_color, ground_atmosphere_color, t)
```

The power curves avoid a banded linear look and produce a natural atmospheric gradient.

Default colors:
- Zenith: `[0.16, 0.36, 0.66]` (deep blue)
- Horizon: `[0.68, 0.78, 0.88]` (light blue-white)
- Ground atmosphere: `[0.38, 0.44, 0.40]` (grey-green)

## Sun disk

The sun direction is derived directly from the environment uniform's `light_direction.xyz` — the
same normalized direction that drives aircraft Lambert lighting. There is no separate sun direction.

```wgsl
let sun_dir = normalize(environment.light_direction.xyz);
let sun_alignment = dot(view_dir, sun_dir);
let disk = smoothstep(cos_radius - 0.0005, cos_radius + 0.0005, sun_alignment);
let halo = smoothstep(cos_radius - 0.06, cos_radius - 0.005, sun_alignment) * 0.25;
sky_color += sun_color * (disk + halo);
```

The default sun angular radius is approximately 0.5° (`cos ≈ 0.99996`). The sun disk is small,
bright, and does not affect aircraft physics. A subtle halo extends slightly beyond the disk edge.

Default sun color: `[1.0, 0.95, 0.85]` (warm white).

## Light direction single source of truth

The `EnvironmentUniform` contains the light direction and intensity. Both the aircraft Lambert
lighting and the sky sun disk read from the same uniform. There is no hardcoded light direction
in any shader.

Default light direction: `normalize([0.4, 0.8, -0.3])` (from above, slightly right, slightly
forward). Intensity: 0.80. Ambient: `[0.30, 0.30, 0.30]`.

## Horizon haze

Horizon haze widens the horizon band by blending the sky color toward the horizon color based on
angular proximity to the horizon:

```wgsl
let haze_falloff = exp(-abs(elevation) * 6.0);
sky = mix(sky, horizon_color, haze_falloff * haze_strength);
```

The exponential falloff is strongest at the horizon (`elevation ≈ 0`) and decays rapidly away
from it. `haze_strength` (default 0.55) controls the overall intensity.

The haze does not turn the entire sky white — it only affects the region near the horizon.

## Distance fog

Exponential distance fog is applied to lit scene geometry (aircraft and ground plane):

```wgsl
let distance = length(world_position - camera_position);
let fog = clamp(1.0 - exp(-density * max(distance, 0.0)), 0.0, 1.0);
let final_rgb = mix(lit_rgb, fog_color, fog);
```

- `density` = 0.0015 (default)
- `fog_color` = `sky_horizon.xyz` (the atmospheric horizon color)
- Fog does NOT affect alpha
- Fog converges visually toward the horizon haze color

The fog uses world-space distance (not clip-space Z) to avoid the nonlinearity of perspective
projection. The camera world position is provided by the camera uniform.

## Fogged objects

| Object | Fogged | Notes |
|--------|--------|-------|
| Aircraft | Yes | Lit color then fogged |
| Ground plane | Yes | Lit color then fogged |
| Grid/axes | No | Unlit, unfogged diagnostic overlay |

## Debug grid/axes policy

The debug grid and axes use the unlit pipeline (`fs_unlit`) which passes through vertex color
without lighting or fog. They remain fully visible at all distances as diagnostic overlays.

This is an intentional choice: debug geometry must remain usable for development and verification.

## Depth and pass ordering

The render pass order within a single encoder:

1. **Clear**: color = `SKY_CLEAR_COLOR` (fallback), depth = 1.0
2. **Sky pass**: fullscreen triangle, `depth_write = false`, `depth_compare = Always`
3. **Ground plane**: lit + fog, `depth_write = true`, `depth_compare = Less`
4. **Grid/axes**: unlit, no fog, `depth_write = false`, `depth_compare = LessEqual`
5. **Aircraft**: lit + fog, `depth_write = true`, `depth_compare = Less`

The sky pass does not write depth, so it cannot occlude scene geometry. The clear color remains
as a technical fallback but is visually covered by the procedural sky.

## GPU resource lifetime

All GPU resources are created at renderer initialization and persist for the renderer's lifetime:

- Sky pipeline (created once)
- Environment uniform buffer (created once, not updated per frame — parameters are fixed defaults)
- Camera uniform buffer (created once, updated per frame via `queue.write_buffer`)
- All bind groups and layouts (created once)

Per-frame work:
- Compute camera uniform on the stack (no heap allocation)
- `queue.write_buffer` for camera and object uniforms (existing pattern)
- No `Vec`, `String`, bind-group, or pipeline recreation per frame

## Environment uniform layout

```text
EnvironmentUniform (96 bytes, 16-byte aligned):
  light_direction: vec4<f32>   // xyz = normalized dir toward light, w = intensity
  ambient:         vec4<f32>   // xyz = ambient RGB, w = reserved
  sky_zenith:      vec4<f32>   // xyz = zenith color, w = reserved
  sky_horizon:     vec4<f32>   // xyz = horizon/haze color, w = haze strength
  sky_ground:      vec4<f32>   // xyz = below-horizon atmosphere, w = fog density
  sun_color:       vec4<f32>   // xyz = sun disk color, w = cos(angular radius)
```

## Camera uniform layout (G1B extended)

```text
CameraUniform (144 bytes, 16-byte aligned):
  view_projection:     mat4x4<f32>   // 64 bytes
  inv_view_projection: mat4x4<f32>   // 64 bytes
  camera_position:     vec4<f32>     // xyz = eye position, w = 0
```

The inverse view-projection is computed CPU-side and uploaded each frame. The camera position is
the chase camera eye position in world space.

## Bind group organization

All pipelines share the same group numbering:

| Group | Binding | Content | Visibility |
|-------|---------|---------|------------|
| 0 | 0 | CameraUniform | Vertex + Fragment |
| 1 | 0 | ObjectUniform | Vertex |
| 2 | 0 | EnvironmentUniform | Fragment |

The sky pipeline layout includes all three groups, but the sky shader only reads groups 0 and 2.
Group 1 is present in the layout but unused by the sky shader.

## Tests

CPU-side tests cover:

1. **Light direction normalization**: default direction is unit-length and matches G1A.
2. **View elevation**: zenith → +1, horizon → 0, nadir → −1, bounded [−1, +1].
3. **Fog factor**: zero distance → 0, zero density → 0, monotonically increasing, bounded [0, 1],
   all outputs finite.
4. **Sun alignment**: same direction → +1, opposite → −1, perpendicular → 0.
5. **Environment uniform finiteness**: all values finite by construction.
6. **Camera inverse**: `VP * inv(VP) ≈ I` for multiple attitudes, all results finite.
7. **Uniform layout sizes**: `EnvironmentUniform` = 96 bytes, `CameraUniform` = 144 bytes.
8. **Matrix inverse**: general 4×4 cofactor expansion verified against identity product.
9. All existing G1A renderer tests continue to pass (75 total renderer tests).

## Intentional limitations

G1B deliberately does NOT include:

- Cloud rendering (no cloud textures, no volumetric clouds)
- HDR or tone mapping
- PBR materials
- Shadows or shadow maps
- Terrain or heightmap
- Weather (rain, snow, wind)
- Bloom, lens flare, god rays
- SSAO or screen-space effects
- Reflections
- Normal mapping or texture materials
- VR/stereo rendering
- Temporal antialiasing

These belong to later renderer slices.

## No physics changes

G1B modifies only the renderer crate. No production physics, aircraft aerodynamics, propulsion,
model, telemetry, or XFOIL code is changed. The renderer dependency boundary is preserved.
