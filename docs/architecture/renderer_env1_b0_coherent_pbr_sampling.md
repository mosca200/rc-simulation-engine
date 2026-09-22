# ENV1-B0 — coherent PBR terrain sampling

ENV1-B0 corrects the sampling contract of the ENV1 Sparse Grass material. It
does not add a material, texture-coordinate system, renderer path, or GPU
resource. The existing terrain pipeline, world-space UVs, texture bindings,
debug selector, lighting paths, atmosphere, shadows, and temporal resolve stay
in place.

## Registered photographic base set

The Sparse Grass albedo, OpenGL tangent normal, and linear roughness maps are
one registered 2.0 m × 2.0 m scan. For a terrain vertex UV of
`world_xz / 2.0 m`, the production shader computes one common coordinate:

```text
base_uv = uv + common_base_offset
```

All three base channels sample `base_uv`. `TerrainMaterial::albedo_uv_offset`
retains its old name for layout compatibility but is the common base offset.
`normal_uv_offset` and `roughness_uv_offset` remain reserved layout fields;
production ENV1 sampling ignores them, and all three defaults are zero. This
prevents a future default from silently sliding one physical channel relative
to the others.

## Coherent anti-repetition pair

The inherited deterministic secondary transform is unchanged:

```text
uv_B = R(+27°) × base_uv × 1.370 + (0.315, 0.571)
```

Sample A and sample B each read the complete albedo/normal/roughness triplet.
The same 0.5 weight blends all three base channels. There is no time,
screen-space noise, camera state, or chunk-local state in the transform.

The B tangent normal is expressed in the rotated UV basis. Before the
normalized linear A/B blend, its XY components return to the primary tangent
frame with the inverse UV rotation:

```text
nx' =  cos(theta) nx + sin(theta) ny
ny' = -sin(theta) nx + cos(theta) ny
nz' =  nz
```

The isotropic positive scale does not change orientation. CPU mirror tests lock
identity, ±90°, 27°, flat-normal invariance, finite output, and unit length so
the sign cannot regress unnoticed. The existing near-field detail normal is a
separate presentation layer applied after the registered base blend.

Roughness A/B samples are linear data and use a linear blend. The default
terrain roughness factor is `1.0`; the photographed roughness map is therefore
the starting authority. Terrain metallic remains `0.0`.

## Retained presentation carriers

The 48.0 m macro carrier and 0.40 m detail carrier are unchanged and are not
new physical scans. Macro albedo and macro roughness now both use the existing
macro A/B transform and weight. No macro normal layer was introduced. FINAL,
albedo, normal, roughness, macro, and detail debug modes remain the existing
observability surface.

`terrain_textures::TERRAIN_TEXTURE_SIZE == 1024` still identifies the legacy
procedural regression fixture. ENV1 production maps remain 2048² and retain
their complete 2048→1 mip chains. No ENV1 asset byte changes in B0.

## Retained limitations

ENV1-B0 does not implement four-material terrain blending, biome/splat maps,
dry/wet/worn zones, dedicated macro or micro scans, 3D grass, ground clutter,
production foliage, alpha-coverage foliage mips, vegetation transmission, GLB
normal/metallic-roughness GPU rendering, the GLB shared-image/different-sampler
cache fix, virtual texturing, texture compression, or terrain/physics height
authority convergence.
