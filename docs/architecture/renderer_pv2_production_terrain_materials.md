# PV2 Production Terrain Materials

Status: renderer-only terrain-material replacement. Base commit:
`535fbe1e0e662bc46443120f95f7094d224b3bd3`. Branch:
`feature/pv2-production-terrain-materials`.

## Outcome

PV2 replaces the visibly prototype G3A-R maps, without changing the terrain
mesh, world-space mapping, derivative TBN, PBR/HDR lighting, terrain draw
path, or any flight/vegetation/aircraft/shadow system. The result is a
production-oriented maintained-grass material: broad tonal variation, stable
directional near grain, independent roughness, and a decorrelated macro
carrier rather than a dotted four-metre tile.

## Asset provenance and reproducibility

All three assets are original RC Simulation Engine material data, generated
offline by the repository's deterministic Rust generator. No external asset,
download, or third-party licence is used.

```text
cargo run -p renderer --bin generate_terrain_textures
```

The generator is pure and its output is committed as:

| Asset | Resolution / format | Colour-space contract | Content |
| --- | --- | --- | --- |
| `terrain_grass_albedo.png` | 1024² RGBA8 PNG | sampled as `Rgba8UnormSrgb` | cool grass, restrained dry wear, directional fibre grain |
| `terrain_grass_normal.png` | 1024² RGBA8 PNG | sampled as linear `Rgba8Unorm` | periodic tangent-space micro relief |
| `terrain_grass_roughness.png` | 1024² L8 PNG | sampled as linear `R8Unorm` | independently seeded moisture/wear response |

The committed-map equality test prevents generator/asset drift. CPU mip
generation remains sRGB-correct for albedo, vector-correct for normals, and
linear for roughness; all resources are created once during renderer startup.

## Sampling policy

- Existing world-space base, macro, and detail scales remain intact: 4 m,
  48 m, and 0.40 m respectively.
- The existing rotated/scaled base sample remains the anti-tiling primary.
- PV2 adds a second, rotated and differently scaled **macro** sample before
  luminance modulation. It prevents its 30–80 m carrier from revealing a
  repeated square in oblique mid/far terrain views.
- Detail albedo and normal still use the existing smooth 20–80 m fade. This
  keeps normal/specular detail out of distant oblique pixels and avoids
  shimmer/crawling.
- Roughness uses independent base/macro/detail samples and is explicitly
  regression-tested not to be a luminance copy of albedo.

No bind group, pipeline, texture, sampler, or buffer is created in the frame
loop; PV2 only changes persistent startup assets and the existing terrain
fragment sampling expression.

## Regression coverage

The terrain suite verifies the committed 1024² PNG dimensions, PNG validity,
generator determinism, exact generator/asset equality, albedo/normal/linear
roughness conventions, seamless periodicity, full 1024→1 mip chains, smooth
detail fading, world-anchored sampling, debug-mode mapping, WGSL parsing and
validation, colour-space texture-format contracts, and absence of terrain
resource creation in the render loop.
