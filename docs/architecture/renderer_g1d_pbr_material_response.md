# G1D — PBR Material Response

Status: implemented slice (renderer-only). Base commit: `77e2ad0` (main).

## Scope

G1D replaces the Lambert-only lighting of the lit path
(`base_color * (ambient + directional * NdotL)`) with a first
metallic/roughness, physically-inspired BRDF, while keeping the renderer
small, cheap, and reversible.

Out of scope (explicitly NOT in this slice):

- `metallicRoughnessTexture`
- normal map (`normalTexture`)
- `occlusionTexture` / `emissiveTexture`
- HDR / tonemapping pipeline
- IBL / environment reflections
- shadow maps
- advanced atmosphere
- production vegetation/terrain shading

## Material data (glTF)

`PrimitiveMaterial` now also carries, per primitive:

| Field              | glTF source                               | Default (spec) | Validation                     |
| ------------------ | ----------------------------------------- | -------------- | ------------------------------ |
| `metallic_factor`  | `pbrMetallicRoughness.metallicFactor`     | `1.0`          | clamped to `[0,1]`, finite     |
| `roughness_factor` | `pbrMetallicRoughness.roughnessFactor`    | `1.0`          | clamped to `[0,1]`, finite     |

Non-finite values fall back to the glTF spec default, so GPU material uniforms
can never contain NaN/Inf. Legacy baseColor-only GLBs keep loading unchanged
(covered by regression tests).

## GPU material uniform

A 16-byte uniform (one WGSL vec4 slot) was added to the existing material bind
group (group 3, binding 2):

```
struct MaterialUniform {
    metallic: f32,
    roughness: f32,
    reserved: vec2<f32>,   // padding for WGSL uniform alignment
}
```

- Created once per material at asset upload time (CPU load → GPU buffer).
- Never written per frame → zero per-frame allocations.
- Reuses the existing material bind group; no new bind group layout.

Procedural geometry (terrain, scenery, procedural aircraft, fallback material)
uses explicit non-chrome parameters: `metallic = 0.0`, `roughness = 0.85`.
Debug overlays remain unlit and are unaffected.

## BRDF implemented (WGSL `fs_lit`)

- **Fresnel**: Schlick, `F = F0 + (1 - F0)(1 - VdotH)^5`
- **NDF**: GGX / Trowbridge-Reitz, `D = a^2 / (PI * ((NdotH^2)(a^2 - 1) + 1)^2)`
  with `a = roughness^2`
- **Geometry**: Smith with Schlick-GGX, direct-lighting `k = (r+1)^2 / 8`
- **F0**: glTF metallic workflow, `F0 = mix(0.04, baseColor, metallic)`
- **Diffuse**: `baseColor * (1 - metallic)` — fully suppressed on metals
- **Specular**: `D * G * F / max(4 * NdotV * NdotL, 1e-4)`

Energy balance: G1D is **legacy-compatible and physically-inspired, not yet
strictly energy-conserving**. The diffuse term is `baseColor * (1 - metallic)`
and deliberately does **not** contain the standard `(1 - Fresnel)` attenuation,
because G1D preserves the legacy Lambert diffuse response. The BRDF/diffuse
model itself is intentionally unchanged in this micro-fix.

Documented numeric/readability guards:

- `MIN_ROUGHNESS = 0.06`: floor for numeric stability and stable highlight
  sizes at RC viewing distances (no aliasing micro-dots).
- `NdotV` floored at `1e-4`.
- The specular denominator is explicitly floored: `NdotL` is clamped to
  `>= 0` but may be exactly zero; without the floor, the Smith geometry term
  (`gl = 0 / k`) makes the `4 * NdotV * NdotL` denominator zero, leaving a
  0/0 in the specular division and propagating NaN into the final
  `* NdotL` product. The implemented guard is
  `specular_denominator = max(4 * NdotV * NdotL, 1e-4)`, applied to the
  specular division only; the `NdotL` used as the final direct-light
  multiplier is unchanged, so back-facing surfaces are never artificially
  illuminated.
- Specular clamped at `4.0` (it would clip to white in LDR output anyway).
- `safe_normalize` returns world-up for degenerate vectors (no NaN/Inf when
  the camera coincides with a shaded point, or `V + L` degenerates).
- **Legacy-compat irradiance scale**: the old Lambert path used
  `albedo * intensity` (no `1/PI`). The physically normalized split is scaled
  by `PI * intensity`, so the diffuse response matches the established look
  exactly and only the specular response is added.
- **Ambient**: still the simple flat ambient (no fake IBL). It is applied to
  the diffuse (non-metal) response; metals receive a flat readability floor of
  `F0 * ambient * 0.5` (`AMBIENT_SPECULAR_SCALE`) so they do not go black at
  distance. This is a documented approximation, not an environment reflection.

Color pipeline preserved: sRGB base-color texture → hardware linearization →
lighting in linear space → fog after lighting → alpha untouched. No double
gamma.

## Tests

Renderer crate:

- metallic/roughness factor parsing from GLB
- glTF spec defaults (1.0/1.0) when factors are absent
- per-primitive distinct factors
- factor clamping to `[0,1]` and non-finite rejection
- `MaterialUniform` layout (16 bytes), finite values, byte round-trip
- procedural material parameters stay non-metal/rough
- legacy baseColor-only GLBs keep loading (regression guard)
- BRDF reference-math regression (pure Rust mirror of the WGSL formulas, no
  GPU): the direct BRDF evaluation stays finite for `NdotL = 0`, `NdotL`
  very close to zero, `NdotV` at its `1e-4` floor, and `MIN_ROUGHNESS`, with
  `NdotL = 0` yielding an exactly zero direct response
- WGSL source-text guard pinning `fs_lit` to the floored specular
  denominator and the unchanged `* NdotL` final multiplier

All pre-existing tests remain green.

## Performance

Qualitative GPU cost (per lit fragment):

- ~30 extra ALU ops vs Lambert (GGX + Smith + Schlick are scalar-vector mixes;
  no divisions beyond one guarded reciprocal).
- One extra uniform read (16 bytes, cached) — no new texture lookups, no new
  render passes, no extra draw calls, still one directional light.
- Memory: one 16-byte uniform buffer per material, allocated once at load.
- Zero per-frame CPU allocations; no per-frame buffer writes for materials.
