# G1C: World Foundation — Base-Color Textures and Terrain

## Overview

G1C extends the renderer from G1B (sky/haze/fog) with:

1. **glTF base-color texture support** — PNG/JPEG embedded in GLB
2. **Per-primitive material binding** — multiple materials per asset
3. **Persistent GPU resources** — textures, samplers, bind groups created once
4. **Terrain subsystem** — chunked height field with lighting/fog integration
5. **Centralized f64→f32 boundary** — render world origin architecture

## GLB Base-Color Texture Path

### Color Space

Base-color textures are **sRGB** data. They are uploaded to the GPU as `Rgba8UnormSrgb`.
The hardware performs sRGB-to-linear conversion automatically during sampling.
**No manual gamma correction** is applied in the shader.

### Supported Image Forms

- Embedded GLB PNG
- Embedded GLB JPEG

External buffer URIs are **not supported** and return an explicit error.

### Image Decoding

Images are decoded during asset loading using the `image` crate.
Decoded pixels are converted to RGBA8 for GPU upload.

Validation:
- Width > 0, height > 0
- Finite dimensions
- No overflow in byte-size calculations

Malformed images return `GlbLoadError::MalformedImage`, not a silent white fallback.

### Texture Upload and Alignment

WebGPU requires `bytes_per_row` to be aligned to 256 bytes (`COPY_BYTES_PER_ROW_ALIGNMENT`).

For images where `width * 4` is not a multiple of 256, a staging buffer with row padding
is created before upload. The `padded_bytes_per_row()` and `create_staging_buffer()`
functions handle this.

### Texture Coordinates

Only `TEXCOORD_0` is supported. If a primitive requests `TEXCOORD_1` or higher,
`GlbLoadError::UnsupportedTexCoord` is returned.

### Fallback White Texture

A persistent 1×1 white RGBA texture is created for materials without a base-color texture.
This allows a uniform shader path. The fallback is created once and never recreated.

### Sampler Support

glTF sampler settings are mapped as follows:

**Wrapping:**
- `REPEAT` → `AddressMode::Repeat`
- `CLAMP_TO_EDGE` → `AddressMode::ClampToEdge`
- `MIRRORED_REPEAT` → `AddressMode::MirrorRepeat`

**Filtering:**
- `NEAREST` → `FilterMode::Nearest`
- `LINEAR` → `FilterMode::Linear`

**Mipmap limitation:** G1C uses a single-mip implementation. Mipmap filter distinctions
(`NEAREST_MIPMAP_NEAREST`, `LINEAR_MIPMAP_LINEAR`, etc.) are collapsed to their
non-mipmap equivalents. This is documented and deterministic.

## Primitive/Material/Batch Architecture

### CPU Representation

```
GlbAsset
  primitives: Vec<RenderPrimitive>

RenderPrimitive
  vertices: Vec<Vertex>
  indices: Vec<u32>
  material: PrimitiveMaterial

PrimitiveMaterial
  base_color_factor: [f32; 4]
  base_color_texture: Option<DecodedTexture>
  sampler_config: SamplerConfig
```

### GPU Representation

```
WgpuRenderer
  materials: Vec<GpuMaterial>
  aircraft_batches: Vec<RenderBatch>

GpuMaterial
  _texture: wgpu::Texture
  _texture_view: wgpu::TextureView
  _sampler: wgpu::Sampler
  bind_group: wgpu::BindGroup

RenderBatch
  vertex_buffer: wgpu::Buffer
  index_buffer: wgpu::Buffer
  index_count: u32
  material_index: usize
```

### Material Binding Invariant

**Primitive → correct material → correct texture** survives loading and rendering.
Each `RenderBatch` references a `material_index` into the persistent `materials` vec.
The bind group at group 3 is set per-batch during rendering.

### Color Combination Formula

```
texture_rgba = textureSample(base_color_texture, base_color_sampler, uv)
base_rgba = vertex_color * texture_rgba
```

Where `vertex_color` already contains `baseColorFactor * COLOR_0` (baked during loading).

The shader does **not** multiply `baseColorFactor` twice.

## Terrain CPU Representation

### TerrainHeightField

```
TerrainHeightField
  width_cells: u32
  depth_cells: u32
  sample_spacing_m: f32
  elevations: Vec<f32>  // row-major, Z-major then X
```

A regular grid of elevation samples at uniform spacing.

### TerrainChunk

```
TerrainChunk
  chunk_coords: (u32, u32)
  world_origin: [f32; 2]
  size_m: [f32; 2]
  vertices: Vec<Vertex>
  indices: Vec<u32>
  bounds: ([f32; 3], [f32; 3])
```

### Chunking Strategy

- Default chunk size: 32×32 cells
- Default terrain extent: 1000m × 1000m
- Default cell spacing: 5m

Chunks are generated deterministically. Adjacent chunks share exact boundary coordinates
(no cracks).

### Terrain Normal Generation

Normals are computed via finite differences:
- Central differences for interior samples
- One-sided differences at edges

Formula: `normal = normalize(-dh/dx, 1, -dh/dz)` for Y-up terrain.

Flat terrain produces exact upward normals within tolerance.

### Terrain UV Strategy

UVs are based on **world-space metres**, not mesh tessellation:

```
u = world_x / texture_scale_m
v = world_z / texture_scale_m
```

Default `texture_scale_m = 4.0` (4m per texture tile).

This ensures texture scale is independent of chunk resolution.

## Terrain GPU Integration

### Lighting

Terrain uses the **same environment directional light** as aircraft and sun.
The `EnvironmentUniform` at group 2 is the single source of truth.

### Fog

Terrain uses the **G1B distance fog** path. Distant terrain blends into the horizon color.

Formula:
```
fog_factor = 1 - exp(-density * distance)
final_rgb = mix(lit_rgb, fog_color, fog_factor)
```

### Terrain Material

Terrain uses the same lit/fogged material pipeline as aircraft.
The fallback white texture is used for terrain (no terrain-specific texture yet).

## Render World Origin

### f64 Simulation → f32 Rendering

The conversion is centralized in `pose.rs`:

```rust
pub fn world_ned_pose_to_render(
    position_world_ned_m: [f64; 3],
    orientation_world_from_body_wxyz: [f64; 4],
    render_origin_world_ned_m: [f64; 3],
) -> Result<RenderPose, RenderDataError>
```

The render origin is subtracted in f64 **before** casting to f32:

```
relative_position_ned = position_world_ned - render_origin_world_ned
relative_position_render = NED_TO_RENDER * relative_position_ned
translation_render_m = relative_position_render as f32
```

This prevents precision loss from large simulation coordinates.

### Physics/Render Boundary

**Terrain is presentation-only.** The renderer terrain does not participate in physics collision.

Dependency direction:
- `renderer` does **not** depend on `aircraft`, `model`, or `sim_core`
- Physics does **not** query renderer terrain
- If future physics terrain is needed, raw data can be factored into a neutral crate

## Draw Architecture

The render pass is organized as:

1. **Sky pass** — fullscreen triangle, depth_write=false, depth_compare=always
2. **Terrain batches** — chunked, lit + fogged
3. **Debug overlays** — grid/axes, unlit, unfogged (optional via `show_debug_overlays`)
4. **Aircraft batches** — lit + fogged, one batch per primitive

No per-frame allocation or resource creation. All GPU resources are persistent.

## Intentional Limitations

G1C deliberately does **not** implement:

- PBR metallic/roughness shading
- Normal maps, occlusion maps, emissive maps
- Transparency / alpha blending
- HDR, tone mapping, bloom
- Shadows, cascaded shadow maps, SSAO
- Reflections
- Volumetric clouds, weather
- Vegetation, trees, buildings
- Satellite imagery
- Terrain LOD, terrain streaming
- Physics terrain collision

These are deferred to later milestones.

## Tests

### GLB Texture Tests

- Embedded PNG base-color texture loads
- Embedded JPEG base-color texture loads
- Malformed image fails clearly
- TEXCOORD_0 used correctly
- TEXCOORD_1 rejected explicitly
- Repeat/ClampToEdge/MirroredRepeat sampler mapping
- Nearest/Linear filter mapping
- Row-upload padding math
- Multiple primitives retain distinct materials
- Existing G1A/G1B GLB fixtures still load

### Terrain Tests

- Flat terrain dimensions correct
- Terrain vertex/index count correct
- Triangle winding correct (CCW from above)
- Flat normals point upward
- Rolling terrain normals finite and unit length
- UV scale independent of tessellation density
- Deterministic generation
- Adjacent chunks share exact boundary coordinates
- No NaN/Inf
- Bounds are correct
- Chunk ordering deterministic

### World Origin Tests

- Simulation positions translated relative to render origin correctly
- Render origin subtracted before f32 cast
- Repeated conversion is bit-identical

## Quality Gates

All quality gates pass:

```
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test -p renderer
cargo test --workspace --all-targets --all-features
cargo build --workspace
```

## Files Changed

- `Cargo.toml` — added `image` dependency
- `crates/renderer/Cargo.toml` — added `image` dependency
- `crates/renderer/src/lib.rs` — export new modules
- `crates/renderer/src/texture.rs` — **new** image decoding and GPU texture infrastructure
- `crates/renderer/src/terrain.rs` — **new** terrain height field and chunking
- `crates/renderer/src/glb.rs` — refactored for multi-primitive/material support
- `crates/renderer/src/gpu.rs` — refactored for material system and terrain rendering
- `crates/renderer/src/shader.wgsl` — added texture sampling
- `crates/app/src/render_app.rs` — boxed GlbLoadError to reduce error size
