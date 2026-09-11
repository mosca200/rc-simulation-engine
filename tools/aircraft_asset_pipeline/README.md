# Aircraft asset pipeline (RV2-7A)

Professional Blender → GLB asset pipeline for RC Simulation Engine aircraft presentation assets.

**Status: foundation only.** This pipeline does **not** replace any runtime asset. The committed
`models/acro_electric_01/aircraft.glb` and `models/acro_electric_01/model.json` are byte-identical
before and after this slice, no renderer/runtime/physics file was touched, and nothing in the
simulation reads any file in this directory.

## Why this exists

Until now the Acro presentation asset was produced by `tools/generate_acro_electric_01_glb.ps1`, a
procedural PowerShell generator. That works, but it is not an editable art source: there is no DCC
document an artist can open, and the primitive ordering that the runtime depends on is an implicit
side effect of the generator's emission order.

This directory adds the missing layer:

```
Blender editable source (.blend, DCC authority)
        │  blender_validate_source.py   (fail closed before anything is exported)
        │  blender_export_glb.py        (ONE pinned production export configuration)
        ▼
GLB 2.0 (runtime authority)
        │  validate_glb.py              (offline, stdlib only, no wgpu)
        ▼
manifest-checked semantic contract
```

`acro_electric_01_manifest.json` is the machine-readable authority for that contract. It is
**presentation only** — it is never read by the simulation.

## Layout

| File | Runs under | Purpose |
|---|---|---|
| `acro_electric_01_manifest.json` | — | Semantic asset contract: components, primitive order, moving surfaces, hinges, materials, bounds, counts, export profiles |
| `asset_contract.py` | system Python **and** Blender Python | Shared manifest loading + render-body ↔ Blender frame conversion + orientation rules |
| `blender_import_reference.py` | Blender | Materialises the editable `.blend` source from the committed reference GLB |
| `blender_validate_source.py` | Blender | Fail-closed validation of the `.blend` source |
| `blender_selftest_source_validation.py` | Blender | Proves the source validator rejects 16 distinct violations |
| `blender_export_glb.py` | Blender | The single production export configuration, with source validation and output validation built in |
| `validate_glb.py` | system Python | Offline glTF 2.0 + manifest validator, and the semantic fingerprint used for reproducibility |
| `test_asset_contract.py` | system Python | 31 contract tests, including fail-closed negative tests |

All Python is standard library only. No `gltf` wheel, no `numpy`, no `wgpu`, no dependency stack.

## Prerequisites

- **Python 3.10+** for the offline validator and the contract tests (developed on 3.12.10).
- **Blender 5.2.1 LTS** for anything touching the `.blend`. Blender is *not* auto-installed by
  this pipeline. On Windows it is commonly at
  `C:\Program Files\Blender Foundation\Blender 5.2\blender.exe`; add it to `PATH` or set:

  ```cmd
  set BLENDER="C:\Program Files\Blender Foundation\Blender 5.2\blender.exe"
  ```

  The expectations recorded in `acro_electric_01_manifest.json -> toolchain` are pinned to that
  Blender build. Re-record them deliberately on upgrade; do not silently relax them.

## Workflow

All commands run from the repository root.

### 1. Materialise the editable source

```cmd
"%BLENDER%" --background --python-exit-code 1 ^
    --python tools/aircraft_asset_pipeline/blender_import_reference.py ^
    -- --asset-id acro_electric_01
```

Writes `models/acro_electric_01/source/acro_electric_01.blend`. It refuses to overwrite an
existing source unless `--force` is given, and it never writes to the reference GLB.

### 2. Validate the source

```cmd
"%BLENDER%" --background models/acro_electric_01/source/acro_electric_01.blend ^
    --python-exit-code 1 ^
    --python tools/aircraft_asset_pipeline/blender_validate_source.py
```

### 3. Export (scratch only)

```cmd
"%BLENDER%" --background models/acro_electric_01/source/acro_electric_01.blend ^
    --python-exit-code 1 ^
    --python tools/aircraft_asset_pipeline/blender_export_glb.py ^
    -- --asset-id acro_electric_01 ^
       --output target/asset_pipeline/acro_electric_01/aircraft.blender.glb ^
       --fingerprint-out target/asset_pipeline/acro_electric_01/fingerprint.json
```

The exporter **refuses** to write onto the production GLB, onto the reference GLB, or anywhere
under `models/`. Scratch output belongs under `target/asset_pipeline/`, which is git-ignored.
It validates the source before exporting and validates the produced GLB afterwards, so a bad
export cannot pass silently. The mutated session is never saved.

### 4. Validate a GLB offline

```cmd
python tools/aircraft_asset_pipeline/validate_glb.py ^
    models/acro_electric_01/aircraft.glb ^
    --manifest tools/aircraft_asset_pipeline/acro_electric_01_manifest.json ^
    --profile production

python tools/aircraft_asset_pipeline/validate_glb.py ^
    target/asset_pipeline/acro_electric_01/aircraft.blender.glb ^
    --manifest tools/aircraft_asset_pipeline/acro_electric_01_manifest.json ^
    --profile blender_export
```

### 5. Contract tests

```cmd
python -m unittest discover -s tools/aircraft_asset_pipeline -p "test_*.py" -v
```

### 6. Prove the source validator is not decorative

```cmd
"%BLENDER%" --background models/acro_electric_01/source/acro_electric_01.blend ^
    --python-exit-code 1 ^
    --python tools/aircraft_asset_pipeline/blender_selftest_source_validation.py
```

Applies 16 contract violations one at a time to the real source in memory and requires the
validator to reject each with the expected diagnostic, reverting to a clean state after every one.
The file is never saved.

## The contract in one page

**Frame.** Runtime render-body and glTF are the same: `+X` aircraft right, `+Y` up, `-Z`
forward/nose, right-handed. Blender is `+X` right, `+Z` up, `-Y` forward. Conversion is
`blender = (x, -z, y)` and `render_body = (x, z, -y)`, implemented once in `asset_contract.py`
and verified against Blender 5.2.1 (a Blender object at `(2, 3, 4)` exports to glTF
translation `(2, 4, -3)`).

**Units.** Metres. Scene unit system `METRIC`, `scale_length = 1.0`, `length_unit = METERS`.

**Hierarchy.** One `ACRO_ROOT` empty at the identity transform; every semantic component is a
direct child. No lights, cameras, armatures or stray empties.

**Objects.** Exactly one mesh object per semantic component, named exactly by its semantic id
(`FUSELAGE`, `AILERON_L`, …). One mesh datablock per object — never shared. Exactly one material
slot, named as declared in the manifest. No modifiers, no shape keys, no vertex-colour dependence.

**Transforms.** Scale exactly `1.0` on every axis (no negative, no unapplied scale) and rotation
exactly identity. Location is free and is the **authoring pivot**: the hinge line for a moving
surface, world origin for a rigid component. Pivots are therefore *not* destroyed by an Apply
Transform requirement — the exporter bakes them at export time instead, which keeps the `.blend`
usable for articulation preview.

**Moving surfaces.** Each is its own object with its own mesh and its own primitive. They are
never merged with rigid geometry, never instanced. Blender names exist for the pipeline only: the
runtime mapping stays explicit in `model.json -> presentation.articulated_surfaces` and is never
inferred from a name.

**Primitive order.** The runtime maps left aileron `6`, right aileron `7`, elevator `9`, rudder
`11`. See "The ordering hazard" below for how the pipeline pins that instead of inheriting it.

## The ordering hazard (measured, not assumed)

Blender's glTF exporter emits nodes — and therefore mesh indices — in **alphabetical object-name
order**, not creation or outliner order. Measured on Blender 5.2.1 LTS / Khronos glTF Blender I/O
v5.2.40 by re-exporting the committed Acro GLB with its original clean names:

| component | production primitive | naive Blender re-export |
|---|---:|---:|
| `LeftAileron` | 6 | 6 (coincidence) |
| `RightAileron` | **7** | **12** |
| `Elevator` | **9** | **3** |
| `Rudder` | **11** | **13** |

A naive Blender export would have silently destroyed the articulation mapping. The pipeline
therefore pins the order **by construction**: `blender_export_glb.py` renames each object to
`{primitive_index:02d}_{SEMANTIC_ID}` in a throwaway session before exporting, and
`validate_glb.py` then re-checks that primitive *i* really is the declared component and fails
closed if it is not. The `.blend` source keeps clean semantic names; the prefix never lands there.

Two related measurements are recorded in the manifest:

- `export_apply` applies **modifiers only** — it does *not* bake object translation (an object at
  `(2, 3, 4)` still exports node translation `(2, 4, -3)`). Because the production loader reads
  baked vertex positions and does not apply glTF node transforms to `GlbAsset::primitives`, the
  exporter bakes every world transform into the mesh data itself.
- Materials are emitted in **first-use order** across the traversal, so a Blender export yields a
  different material *index* order from the legacy generator even though the material names and
  shading values are identical. Both orders are recorded in
  `materials.index_order_profiles`.

## Reproducibility (measured)

Three independent runs of `blender_import_reference.py` produced three **different** `.blend`
files (250 924 – 250 930 bytes; Blender embeds its own save metadata). This is reported rather
than papered over.

All three exports are **byte-identical**:

```
glb sha256 : be523ca41dcbe1dbe6243df604076edd6e44f72b7d8294be1b8b715c7d9d388d
glb bytes  : 206116
```

and their semantic fingerprints are identical too. So reproducibility is asserted on the
deliverable (the GLB), not on the intermediate DCC container. For comparison across Blender
versions or platforms use the fingerprint, not the hash — it covers node order, mesh/primitive/
accessor counts, per-primitive vertex and index counts, material names and factors, bounds and
embedded image hashes:

```cmd
fc /b run_a.fingerprint.json run_b.fingerprint.json
```

`blender_import_reference.py` also deletes the `.blend1` rollback sibling Blender writes when
overwriting, so a regenerable backup can never be committed.

## What a Blender export does and does not match today

Exporting the Acro source with the pinned configuration produces a GLB that satisfies the whole
manifest contract and matches the production asset on everything that matters visually:

| metric | production (PowerShell) | Blender export | match |
|---|---:|---:|---|
| primitives | 21 | 21 | ✓ |
| triangles | 8 270 | 8 270 | ✓ |
| indices | 24 810 | 24 810 | ✓ |
| render-body bounds min | `[-0.92, -0.117, -0.89]` | `[-0.92, -0.117, -0.89]` | ✓ |
| render-body bounds max | `[0.92, 0.795, 0.91]` | `[0.92, 0.795, 0.91]` | ✓ |
| materials (names + factors) | 8 | 8 | ✓ |
| primitive order | authored | pinned by prefix | ✓ |
| vertices | 5 811 | 5 903 | ✗ (+92) |
| bytes | 252 816 | 206 116 | ✗ |
| material index order | authored | first-use | ✗ |
| node names | `Fuselage` … | `00_FUSELAGE` … | ✗ |
| scene nodes | 21 | 22 (+`ACRO_ROOT`) | ✗ |

The +92 vertices are Blender re-splitting vertices along normal/material seams; silhouette,
shading normals and triangle topology are unchanged. Material index order and node names are
deliberate, documented consequences of the pinned export.

**A production cutover is therefore still a separate, future slice.** It would have to reconcile
the material index order and the node names with the positional assertions in
`crates/renderer/tests/aircraft_asset_g3c.rs`. RV2-7A does neither and changes no runtime
behaviour.

## PBR readiness (RV2-8)

The Blender contract is already shaped for a full metallic-roughness asset: base colour, normal,
metallic/roughness, `alphaMode` `MASK`/`BLEND`, `doubleSided` only where genuinely needed, UV0,
and future decals/livery shells. Recommended texture naming:

```
<asset_id>_<part>_basecolor
<asset_id>_<part>_normal
<asset_id>_<part>_orm        R = occlusion, G = roughness, B = metallic
```

Nothing here creates placeholder or fake textures. `attribute_contract.texcoord_0_required` is
`false` for Acro because the asset is untextured; flip it to `true` in the same slice that adds
textures, and both validators will then require UV0 and the matching images. The validator also
rejects `extensionsUsed`/`extensionsRequired`, keeping the asset on the renderer's core path.

## Propeller and control surfaces

`PROP_ASSEMBLY` and `DETAIL_PROP_TIPS` stay **presentation only**. No RPM coupling, no shaft
speed, no propulsion, snapshot or physics change is made or implied. `SurfaceId::Propeller` and
`propeller_angle_rad` remain reserved by the runtime.

No new mixer, no new servo logic, and no runtime inference from object names. The Blender names
serve the asset pipeline; the explicit G1E mapping in `model.json` remains the only articulation
authority.

## Reuse for the SIG Kadet LT-40 (RV2-7B)

The pipeline is generic: every asset-specific fact lives in one manifest, and every script takes
`--asset-id`. To add `models/sig_kadet_lt40_egv`:

1. Copy `acro_electric_01_manifest.json` to `sig_kadet_lt40_egv_manifest.json` and re-declare
   `asset_id`, `paths`, `components`, `moving_surfaces`, `materials`, `bounds`, `counts` and
   `orientation_checks` for the Kadet's real geometry.
2. Use `docs/reference_aircraft/sig_kadet_lt40_egv_geometry_reconstruction.md` and
   `docs/reference_aircraft/data/sig_kadet_lt40_geometry_v0.json` as a **visual** geometric
   reference while modelling in Blender. They must not feed back into the physical data: the
   Kadet's mass properties, aerodynamics, propulsion and survey evidence stay authoritative in
   their own files.
3. Declare the Kadet's real layout honestly, derived from the reference evidence rather than from
   Acro's. Its component set will not look like Acro's, and the `orientation_checks` reference
   components must be re-pointed at components that actually exist. Where the reference data lists
   an entry under `unknowns`, do not invent geometry to fill it.
4. Mind the frames — this is the easy mistake. The reference documentary frame is `x_aft` positive
   aft / `y` positive right / height positive **up**, origin at the wing root leading edge. Its
   published `runtime_body_conversion` lands in the **FRD physics body frame** (`+X` forward, `+Y`
   right, `+Z` down, X re-referenced to the CG) — *not* in the render-body frame the asset needs.
   Reaching render-body takes a second step, whose authority is
   `crates/renderer/src/pose.rs :: NED_TO_RENDER`. Composed:
   `render = (y_right, height_up, -(cg_x_aft - x_aft))`. The reference is also only **2D planform**
   evidence (outlines, areas, centroids, hinge stations — no fuselage shape, no airfoils, no solids),
   and EGV dimensional equality is unproven because the measurements come from the original SIG
   RC-67 kit plans. Note that `models/sig_kadet_lt40_egv/model.json` already declares the four
   `control_surface_bindings` (`aileron-left`, `aileron-right`, `elevator`, `rudder`), so the new
   `presentation.articulated_surfaces` block references existing ids rather than inventing them.
   Full detail: `docs/architecture/renderer_rv2_7a_blender_aircraft_pipeline.md`.
5. Run the same four steps: materialise → validate source → export to scratch → validate GLB.

See `docs/architecture/renderer_rv2_7a_blender_aircraft_pipeline.md` for the full contract and
`docs/architecture/renderer_glb_scene_node_foundation.md` for the loader semantics the export
targets.

## Known limitations

- The Acro `.blend` is a **reorganisation of the existing procedural geometry**, not remodelled
  art. Silhouette and topology are the G3C-B asset's.
- Per-side separation does not exist in the source geometry and is deliberately not invented:
  `WING_MAIN_FIXED` is one full-span object, `HTAIL_FIXED` is one object, `GEAR_MAIN` carries both
  main legs, and `WHEELS` carries all three tires. Splitting them is future art work and would
  change primitive counts, so it belongs to a slice that also updates the runtime mapping.
- No UV maps and no textures yet, so `texcoord_0_required` is `false`.
- No LOD chain, no cockpit/interior, opaque canopy, static propeller — all inherited from G3C-B.
- `counts.blender_export` is pinned to one Blender build; a Blender upgrade requires re-recording.
- No CI job runs Blender, so the Blender-side steps are local/developer-run. `validate_glb.py` and
  `test_asset_contract.py` are pure stdlib and could be added to CI in a later slice.
