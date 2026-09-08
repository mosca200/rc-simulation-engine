# G3-VR1 — Visual Remediation (rendered-field pass)

Status: implemented renderer-only remediation. Exact base:
`8a648ebc8fd15c45090bd8d0d7a9dc11cd308c02` (G3B→G3E convergence).

## Scope and priorities

G3-VR1 is a targeted visual pass over the approved G3D-R / G3E build,
responding to the manual visual gate that failed at `8a648eb`. Every change
exists to move the rendered FlyingField closer to the professional outdoor
target without changing physics, controller, replay, renderer architecture,
G3D instancing/culling/LOD logic, or the G3E cascade configuration.

It does not add LOD3 billboards, wind animation, foliage texturing, splat
mapping, ray tracing, or any new subsystem. Out of scope is anything with a
cost or risk disproportionate to the observed visual gaps.

## Audit findings (driven by a real RTX 3090 runtime + screenshots)

| # | Finding | Root cause |
|---|---------|-----------|
| 1 | Mid/long-distance trees read as bare branch skeletons | DECIDUOUS_LOD1 kept only 8 puffs/3 branches; conifer LOD2 was 2 whorls × 3 fronds — a literal skeleton past ~130 m |
| 2 | Near canopies look low-poly / blobby | LOD0 puff lat-lon 8×5, trunk 8 sides, branches and conifer fronds at 4 tube sides |
| 3 | Terrain tiling / flat look | Tile = 4 m with only 0.20 macro gain and 0.25 detail blend |
| 4 | Flat, synthetic horizon transition | Fog density 0.0015 was ~0.53 at the 500 m field edge, cutting against the sky; haze band decayed at 6/rad |
| 5 | Shadows hard / poorly integrated | Fixed 3×3 PCF (9 taps) and a hard cut to zero direct light in full shadow |
| 6 | Placeholder-orange runway poles dominant | 2.0 m × 0.05 m saturated orange cylinders at ±9 m |
| 7 | Aircraft readability | Already acceptable; kept untouched |

## Changes

### Vegetation LOD fidelity (`vegetation_assets.rs`, `vegetation.rs`)

- `DECIDUOUS_LOD0` now `(14, 6, 6, 10, 10, 6)` (trunk 6×10, puffs 10×6)
  and branches swept at 6 sides: near canopies lose the low-poly faceting.
- `DECIDUOUS_LOD1` now `(10, 4, 4, 8, 8, 4)`: the mid-distance canopy shell
  stays closed instead of exposing branch skeletons.
- `CONIFER_LOD0` `(8, 6, 10, 6)` and `CONIFER_LOD1` `(5, 4, 8, 5)`:
  fronds at 5–6 sides, one extra whorl at LOD1.
- `deciduous_lod2`: canopy mass 10×6 rings, lobes 8×4 — far deciduous trees
  read as canopy mass, not twigs.
- `conifer_lod2`: three whorls × four fronds at 5 sides — far conifers keep a
  foliated mass.
- `DEFAULT_LOD1_MAX_M` 130 → 160 m: the richer LOD1 stays in view through the
  mid range; LOD2 covers 160–340 m.

All triangle-ratio, per-asset budget, total budget, normal/winding,
determinism, silhouette and GLB round-trip tests remain green.

### Terrain / atmosphere composition (`shader.wgsl`, `gpu.rs`)

- `TERRAIN_MACRO_ALBEDO_GAIN` 0.20 → 0.32 and
  `TERRAIN_DETAIL_ALBEDO_BLEND` 0.25 → 0.35: more large-scale tonal
  variation and near grain break the perceived 4 m tile repetition without
  touching the versioned textures or UV anchoring.
- `DEFAULT_FOG_DENSITY` 0.0015 → 0.0028: the 500 m field edge fades to
  ~0.75 fog instead of cutting against the sky; the aircraft (3–20 m) stays
  at <0.06 fog.
- `DEFAULT_HAZE_STRENGTH` 0.55 → 0.68 and sky haze falloff 6/rad → 4/rad: a
  wider horizon band blends terrain and sky through a gradation.

### Shadows (`shader.wgsl`, `gpu.rs`)

- PCF kernel 3×3 → 5×5 (9 → 25 comparison taps) through the named WGSL
  constants `SHADOW_PCF_TAPS = 2` / `SHADOW_PCF_TAP_COUNT = 25.0`; the
  comparison divisor now references the constant so the tap count lives in
  one place. The structural regression test was updated to pin the new
  contract.
- Penumbra floor `SHADOW_MIN_VISIBILITY = 0.30` applied inside
  `directional_shadow_visibility`: fully shadowed receivers keep 30 % direct
  light, integrating shadows with the terrain while deep contact shadows stay
  readable. The `direct_unshadowed * shadow_visibility` line and the ambient
  isolation contract are unchanged.

### Scenery placeholder reduction (`scenery.rs`)

- New dedicated `RUNWAY_POLE_*` palette: height 2.0 → 1.5 m, radius 0.05 →
  0.035 m, saturated orange `[0.85, 0.25, 0.10]` → warmer muted
  `[0.72, 0.30, 0.14]`. The poles stay visible position markers without
  reading as debug placeholders. `PILOT_ORANGE` is untouched (pilot stations
  and windsock keep their established identity).

## Verification

```text
cargo fmt --all -- --check                                    PASS
cargo check --workspace --all-targets                         PASS
cargo clippy --workspace --all-targets -- -D warnings         PASS
cargo test --workspace --all-targets                          87 suites, 0 failures
cargo build --workspace --release                             PASS
```

Runtime smoke on the RTX 3090 reference (release):

- `play --scenery flying-field --exposure-ev 0.0` — clean startup, no GPU
  validation errors; before/after screenshots captured with identical
  configuration.
- `play --scenery flying-field --vegetation-debug lod` — LOD bands visible
  (mid range holds LOD1, far range LOD2 with fuller mass).
- `play --scenery flying-field --exposure-ev 1.0` — clean startup, brighter
  sky/haze response.

Measured frame-time/FPS data is not available: the renderer has no
timestamp/profiling path (documented limitation since G3E), so performance is
reported as not measured rather than inferred. The algorithmic cost added is
bounded (25 vs 9 shadow taps; slightly denser vegetation meshes well below
the pinned budgets).

## Residual gaps (unchanged scope)

- LOD3 billboard / impostor remains a documented gap; far trees are meshes at
  LOD2.
- No wind animation or foliage texture; vertex color + PBR remain the foliage
  content.
- Terrain maps stay the versioned 512² set; repetition is reduced by blend
  tuning only.
- Cascade transitions remain hard selections (no blend band).
- A permanent FPS/1% low measurement path is still outstanding.