# RV2-7A Blender aircraft asset pipeline

## Scope and philosophy

RV2-7A adds a professional DCC asset pipeline for aircraft presentation assets. It replaces the
authoring *method* — procedural PowerShell emission — with an editable Blender source, a
deterministic validated export, and an offline validator, without replacing any shipped asset.

Nothing in this slice changes runtime behaviour. `crates/renderer/src/gpu.rs`, `shader.wgsl`,
`glb.rs`, `texture.rs`, `renderer_v2/**`, `render_graph.rs`, `camera.rs`, the app runtime, model
physics, aircraft physics, `sim_core`, replay and the platform layer are untouched. The committed
production assets are byte-identical before and after:

| file | SHA-256 before | SHA-256 after |
|---|---|---|
| `models/acro_electric_01/aircraft.glb` | `55932887da7e5114846bcf20f5bf1219fed303810ac761043c22275becc470d9` | unchanged |
| `models/acro_electric_01/model.json` | `f0cc5ce4711f07d85dc60c0f6c61b0fba48d04107205ffa0cb42329f00aa0eee` | unchanged |

No material tuning against RV2-6 was performed, and no RV2-6 dependency was introduced: this
branch is cut from `integration/render-v2` directly.

## Authority model

Two authorities, deliberately separated:

- **DCC authority = Blender.** `models/<asset_id>/source/<asset_id>.blend` is the editable art
  source. It is a working document, regenerable from the reference GLB at any time.
- **Runtime authority = the exported GLB.** `models/<asset_id>/aircraft.glb` is what the renderer
  loads. The `.blend` is never a runtime dependency; `test_asset_contract.py` fails if a runtime
  crate ever references it.

A third artefact sits between them: `tools/aircraft_asset_pipeline/<asset_id>_manifest.json` is
the machine-readable **presentation contract** — component semantics, primitive order, moving
surfaces and hinges, materials, bounds, counts and export profiles. It declares
`"consumed_by_simulation": false` and is never read by physics, aerodynamics, propulsion, servos,
mixers, contacts, the scheduler, replay or any fingerprint. The 500 Hz rate, RK4 integrator, `f64`
pose path, NED/FRD frames and replay hashes are untouched.

Articulation authority is unchanged and stays where G1E put it:
`model.json -> presentation.articulated_surfaces`, resolved by explicit primitive index and exact
`control_surface_binding_id`. There is no substring, node-name or declaration-order inference, and
this slice adds none.

## Axes and units

Runtime render-body and glTF share one frame:

- `+X`: aircraft right
- `+Y`: up
- `-Z`: forward / nose
- right-handed, metres

Blender uses `+X` right, `+Z` up, `-Y` forward. The conversion is implemented once, in
`tools/aircraft_asset_pipeline/asset_contract.py`:

```
blender      = (x, -z, y)     for a render-body point (x, y, z)
render_body  = (x,  z, -y)    for a Blender point (x, y, z)
```

It was verified empirically rather than assumed: a Blender object at `(2, 3, 4)` exports to glTF
translation `(2, 4, -3)` on Blender 5.2.1 LTS with `export_yup=True`.

Units are metres with `unit_settings.system = METRIC`, `scale_length = 1.0` and
`length_unit = METERS`. A non-metric scene, or a `scale_length` other than `1.0`, fails validation:
that is the classic route to a 100× scale bug reaching the renderer.

## Semantic object naming

Every semantic component is exactly one mesh object, named exactly by its semantic id, parented to
a single `ACRO_ROOT` empty at the identity transform. Names match
`^[A-Z][A-Z0-9]*(_[A-Z0-9]+)*$`.

Acro's 21 components, adapted to the geometry that actually exists:

| idx | semantic id | family | legacy generator node | material |
|---:|---|---|---|---|
| 0 | `FUSELAGE` | structure | `Fuselage` | Airframe Pearl |
| 1 | `COWL` | structure | `Cowl` | Competition Red |
| 2 | `SPINNER` | structure | `Spinner` | Painted Spinner |
| 3 | `PROP_ASSEMBLY` | propeller | `Propeller` | Carbon Propeller |
| 4 | `CANOPY` | structure | `Canopy` | Tinted Canopy |
| 5 | `WING_MAIN_FIXED` | structure | `MainWingFixed` | Airframe Pearl |
| 6 | `AILERON_L` | control_surface | `LeftAileron` | Competition Red |
| 7 | `AILERON_R` | control_surface | `RightAileron` | Competition Red |
| 8 | `HTAIL_FIXED` | structure | `HorizontalStabilizer` | Airframe Pearl |
| 9 | `ELEVATOR` | control_surface | `Elevator` | Deep Navy Accent |
| 10 | `VTAIL_FIXED` | structure | `VerticalStabilizer` | Airframe Pearl |
| 11 | `RUDDER` | control_surface | `Rudder` | Competition Red |
| 12 | `GEAR_MAIN` | gear | `MainLandingGear` | Gear Metal |
| 13 | `GEAR_NOSE` | gear | `NoseLandingGear` | Gear Metal |
| 14 | `WHEELS` | wheel | `Wheels` | Tire Rubber |
| 15 | `LIVERY_WING_FUSELAGE` | livery | `WingAndFuselageLivery` | Deep Navy Accent |
| 16 | `LIVERY_TOP_RED` | livery | `TopRedLivery` | Competition Red |
| 17 | `LIVERY_UNDERSIDE_NAVY` | livery | `UndersideNavyLivery` | Deep Navy Accent |
| 18 | `DETAIL_CANOPY_FRAME` | detail | `CanopyFrame` | Deep Navy Accent |
| 19 | `DETAIL_WHEEL_HUBS` | detail | `WheelHubs` | Gear Metal |
| 20 | `DETAIL_PROP_TIPS` | detail | `PropellerTips` | Competition Red |

Two adaptations were made instead of following a generic naming template, because inventing
components that are not in the geometry would be worse than naming them honestly:

- The airframe is a **tricycle** undercarriage, so it has `GEAR_NOSE`, not a tail gear. There is no
  `GEAR_TAIL` / `WHEEL_TAIL`.
- `WING_MAIN_FIXED`, `HTAIL_FIXED`, `GEAR_MAIN` and `WHEELS` are single full-span or single
  combined objects because that is how the G3C-B geometry exists. They are *not* split into
  `_L`/`_R` pairs here: splitting changes primitive counts, so it belongs to a slice that also
  updates the runtime mapping.

Each object also carries `rc_semantic_id`, `rc_primitive_index`, `rc_legacy_node_name`,
`rc_family` and `rc_asset_id` custom properties (plus `rc_moving_surface` and
`rc_control_surface_binding_id` on the four control surfaces). These are pipeline metadata only:
`export_extras` is off, so they never reach the GLB, and no runtime code infers anything from them.

## Primitive ordering: the hazard, measured

`GlbAsset::primitives` keeps the historical flat contract — active meshes in glTF mesh-index order,
primitives in document order — and G1E maps moving surfaces by index into it. So the *emission
order of the exporter* is part of the runtime contract.

Blender's glTF exporter emits nodes, and therefore mesh indices, in **alphabetical object-name
order**, not creation or outliner order. This was measured on Blender 5.2.1 LTS / Khronos glTF
Blender I/O v5.2.40 by importing the committed Acro GLB and re-exporting it unchanged:

| component | production primitive | naive Blender re-export |
|---|---:|---:|
| `LeftAileron` | 6 | 6 (coincidence) |
| `RightAileron` | **7** | **12** |
| `Elevator` | **9** | **3** |
| `Rudder` | **11** | **13** |

A naive Blender export would have silently destroyed the articulation mapping while producing a
perfectly valid-looking GLB. This is exactly the accidental dependency RV2-7A exists to remove.

The pipeline pins the order by construction and then verifies it:

1. `blender_export_glb.py` renames every object to `{primitive_index:02d}_{SEMANTIC_ID}` in a
   throwaway in-memory session, so alphabetical order *is* the required primitive order.
2. `validate_glb.py` then re-checks that primitive *i* resolves to the declared node name, that
   every moving surface is owned by exactly one node, and that the manifest's moving-surface
   indices equal `model.json`'s `visual_primitive_index` values. Any mismatch fails closed.

The `.blend` source keeps clean semantic names; the order prefix never lands in it.

Two further exporter behaviours were measured and are recorded in the manifest rather than assumed:

- `export_apply` applies **modifiers only**. It does not bake object translation: an object at
  `(2, 3, 4)` still exports node translation `(2, 4, -3)`.
- Materials are emitted in **first-use order** across the traversal, not in the order the legacy
  generator authored them.

## Transforms and pivots

The production loader reads baked vertex positions and does not apply glTF node transforms to
`GlbAsset::primitives`; `world_transform` lives on `GlbSceneInstance` for the future GPU
instancing path. Therefore an exported production GLB must carry its geometry in render-body space
with identity node transforms.

Because `export_apply` does not do this, `blender_export_glb.py` bakes it explicitly: for each
component it applies `matrix_world` into the mesh data and resets the object to identity, refusing
to proceed if the mesh is shared (which would bake it twice).

The source contract is:

- **scale exactly `1.0`** on every axis — negative or zero scale mirrors geometry and flips
  normals; any other value is unapplied scale and would make exported metres wrong
- **rotation exactly identity** — objects must be axis-aligned with the render-body frame
- **location free** — this is the authoring pivot

That combination is deliberate. Requiring a blanket Apply Transform would destroy the pivots that
make the source useful for articulation work, so pivots are kept in the `.blend` and baked at
export time instead.

Pivot values: the four moving surfaces carry their object origin on the authored hinge line,
converted into Blender space; every rigid component sits at world origin.

| semantic id | hinge origin, render-body m | Blender pivot |
|---|---|---|
| `AILERON_L` | `[0.0, 0.265, 0.11]` | `[0.0, -0.11, 0.265]` |
| `AILERON_R` | `[0.0, 0.265, 0.11]` | `[0.0, -0.11, 0.265]` |
| `ELEVATOR` | `[0.0, 0.367, 0.735]` | `[0.0, -0.735, 0.367]` |
| `RUDDER` | `[0.0, 0.375, 0.735]` | `[0.0, -0.735, 0.375]` |

Baking was proven rather than trusted: after repositioning all four pivots, the source validator
measures render-body bounds `[-0.92, -0.117, -0.89] … [0.92, 0.795, 0.91]`, identical to the
production GLB, and the exported GLB reproduces the production triangle count (8 270) and index
count (24 810) exactly.

Pivots remain a DCC convenience. The runtime never reads a Blender origin; hinge origins and axes
come from `model.json` and are applied by the renderer as `T(pivot) · R(axis, gain·δ) · T(-pivot)`.

## Control surfaces

Each moving surface is its own object, with its own mesh datablock, producing its own primitive.
The source validator rejects a mesh shared by more than one object — over *all* mesh objects,
declared or not — so a control surface can never be merged with rigid geometry or silently
instanced against it.

The aileron hinge axes are deliberately **not unit length**: `[1.0, ±0.054, 0.0]` encodes the wing
dihedral so a deflected aileron follows the wing plane. `renderer::SurfaceHinge::new` requires only
finite components and `|axis| > 1e-9` and normalizes internally, so the authored vector is preserved
verbatim in `model.json`. The manifest mirrors both the authored axis and the normalized axis
(`[0.998545181, ±0.05392144, 0.0]`) and the validator checks both. An earlier draft of the
validator wrongly required unit-length axes and correctly failed on the real production data; the
runtime, not the guess, is the authority.

No new mixer, no new servo logic, and no runtime inference from object names were added. The
propeller stays presentation-only: `PROP_ASSEMBLY` and `DETAIL_PROP_TIPS` are static,
`SurfaceId::Propeller` and `propeller_angle_rad` remain reserved, and no propulsion, shaft-speed,
physics or snapshot code was touched.

## Material contract

Core glTF 2.0 metallic-roughness, no extensions. `validate_glb.py` rejects `extensionsUsed` and
`extensionsRequired` outright, keeping assets on the renderer's existing path.

Eight materials, one material slot per component, names and shading values identical between the
legacy generator and the Blender export:

| material | role | base colour | metallic | roughness |
|---|---|---|---:|---:|
| Airframe Pearl | airframe dielectric | `[0.94, 0.965, 1.0, 1.0]` | 0.00 | 0.26 |
| Competition Red | livery red | `[0.88, 0.018, 0.028, 1.0]` | 0.00 | 0.24 |
| Deep Navy Accent | livery navy | `[0.012, 0.035, 0.11, 1.0]` | 0.00 | 0.30 |
| Tinted Canopy | canopy opaque tint | `[0.018, 0.095, 0.17, 1.0]` | 0.00 | 0.11 |
| Painted Spinner | spinner paint | `[0.92, 0.022, 0.018, 1.0]` | 0.00 | 0.19 |
| Carbon Propeller | propeller carbon | `[0.014, 0.017, 0.024, 1.0]` | 0.12 | 0.30 |
| Gear Metal | gear metal | `[0.42, 0.46, 0.52, 1.0]` | 0.78 | 0.27 |
| Tire Rubber | tire rubber | `[0.012, 0.014, 0.018, 1.0]` | 0.00 | 0.88 |

All are `alphaMode: OPAQUE` and `doubleSided: false`. `alphaMode` `MASK`/`BLEND`, `alphaCutoff` and
`doubleSided` are validated per material, so a future asset that needs them declares them
explicitly instead of drifting.

Material **index** order differs between the two producers, because Blender emits first-use order:

| index | production (authored) | Blender export (first-use) |
|---:|---|---|
| 0 | Airframe Pearl | Airframe Pearl |
| 1 | Competition Red | Competition Red |
| 2 | Deep Navy Accent | Painted Spinner |
| 3 | Tinted Canopy | Carbon Propeller |
| 4 | Painted Spinner | Tinted Canopy |
| 5 | Carbon Propeller | Deep Navy Accent |
| 6 | Gear Metal | Gear Metal |
| 7 | Tire Rubber | Tire Rubber |

Both orders are recorded in `materials.index_order_profiles` and each export profile enforces its
own. This matters because `crates/renderer/tests/aircraft_asset_g3c.rs` asserts *positional*
material indices against the committed production GLB (`materials[0]` airframe, `materials[3]`
canopy, `materials[7]` tire). A future production cutover to a Blender-exported GLB must either
restore the production material order or update those assertions in the same slice. RV2-7A does
neither, because it does not replace the asset.

## UV and texture contract

Acro carries no UV maps and no textures; the livery is material segmentation. The manifest states
this explicitly with `attribute_contract.texcoord_0_required: false`, so the requirement is
declared rather than silently absent. Setting it to `true` makes both validators require `TEXCOORD_0`
on every primitive and reject an asset with no textures — which is the RV2-8 switch, and is covered
by a negative test today.

No placeholder or fake textures are created anywhere in this slice. Recommended naming when real
textures arrive:

```
<asset_id>_<part>_basecolor
<asset_id>_<part>_normal
<asset_id>_<part>_orm        R = occlusion, G = roughness, B = metallic
```

## Export settings

`blender_export_glb.py` holds exactly one production configuration, with every flag stated
explicitly so the result cannot drift with Blender's per-file operator settings. Highlights:
`export_format='GLB'`, `export_yup=True`, `export_normals=True`, `export_texcoords=True`,
`export_tangents=False`, `export_materials='EXPORT'`, `export_vertex_color='NONE'`,
`export_extras=False`, `export_attributes=False`, all animation/skin/morph/camera/light flags off,
Draco, meshopt and gltfpack off, `use_selection=False`, `use_visible=False`,
`will_save_settings=False`.

`use_visible=False` is deliberate: exporting everything regardless of viewport state means a stray
viewport toggle cannot silently drop a production mesh. That is paired with a source-validator rule
that rejects any required component which is hidden, render-hidden, viewport-hidden, unlinked or
inside an excluded collection.

The exporter refuses to write onto the production GLB, onto the reference GLB, or anywhere under
`models/`; scratch output goes to `target/asset_pipeline/`, which is git-ignored. It validates the
source before exporting and validates the produced GLB afterwards. It never saves the mutated
session. The refusal was exercised, not just written: an export aimed at
`models/acro_electric_01/aircraft.glb` exited non-zero with three distinct refusals and left the
asset byte-identical.

## Validation flow

```
.blend source ──▶ blender_validate_source.py ──▶ blender_export_glb.py ──▶ GLB
                        (fail closed)              (bake + pin order)         │
                                                                             ▼
                                                              validate_glb.py --profile
                                                              production | blender_export
```

`blender_validate_source.py` fails with a non-zero exit on: non-metric unit system or incoherent
`scale_length`; non-finite object transforms; negative, zero or unapplied scale; non-identity
rotation; a missing required semantic object; an undeclared mesh object; a mesh shared between
objects (a moving surface merged with or instanced against rigid geometry); a missing UV map where
the manifest requires one; a missing, unassigned, unexpected or multi-slot material; zero-vertex,
zero-polygon or zero-area meshes; absurd bounds; a wrong orientation convention; a hidden or
excluded production mesh; and modifiers or shape keys that would make the export ambiguous.

`validate_glb.py` is pure standard library — no wgpu, no `gltf` wheel, no numpy — and checks the
container (magic, version, declared length, chunk table, 4-byte alignment, JSON/BIN consistency),
the document (buffers, buffer views, accessors, no dangling references, finite min/max), the
geometry (decoded POSITION finiteness, unit-length NORMAL, index range and count, degenerate
triangle ratio, per-primitive and global bounds) and the manifest contract (primitive count and
order, semantic node names at each index, moving-surface separation, material names/factors/alpha
mode/double-sided, per-component material assignment, orientation conventions, counts, size and
SHA-256).

Neither validator is taken on trust. `blender_selftest_source_validation.py` applies 16 contract
violations one at a time to the real source in memory and requires the expected diagnostic for
each, reverting to a verified-clean state after every one; the file is never saved. All 16 were
rejected. `test_asset_contract.py` adds 31 standard-library tests, including ten fail-closed
negative tests (truncated container, corrupted magic, empty file, missing file, renamed material,
swapped node names, wrong moving-surface index, unknown profile, tightened UV contract, tightened
bounds limit, flipped nose convention) and cross-checks the manifest against
`model.json -> presentation` and against `EXPECTED_PARTS` parsed from
`crates/renderer/tests/aircraft_asset_g3c.rs`.

The orientation convention is checked structurally rather than by a free-form expression: the
declared foremost component must own the global minimum Z, the declared "above" component must
out-rise the "below" one on +Y, the declared front component must be forward of the one behind it,
and every `*_L` semantic id must have a `*_R` sibling that it mirrors about `X = 0` and matches on
Y and Z.

## Reproducibility

Measured, and reported honestly where it does not hold.

Three independent runs of `blender_import_reference.py` produced three **different** `.blend` files
(250 924 – 250 930 bytes):

```
884e54352c327fdd210c537785ed88d01cf29c735559b2f3ced375f77a02cdbd
c419f091f0fb000b9cab32a908e73eab271167f9a8bf39af07edd48456b58199
56e0683854f98e0a8b53aa401597dcf0df417fbe356ebada0adccf91d78a3ce1   <- committed
```

Blender embeds its own save metadata in the container. Byte-level determinism of the `.blend` is
therefore **not** claimed and was not faked.

All three exports are **byte-identical**:

```
glb sha256 : be523ca41dcbe1dbe6243df604076edd6e44f72b7d8294be1b8b715c7d9d388d
glb bytes  : 206116
```

and their semantic fingerprints are identical. Reproducibility is thus asserted on the deliverable
(the GLB), including across independently regenerated sources, not on the intermediate DCC
container. For comparison across Blender versions or platforms the fingerprint is the portable
artefact: it covers generator string, node order, scene/mesh/primitive/accessor/buffer-view counts,
per-primitive node names, attributes, material, vertex and index counts and bounds, material names
and factors, embedded image hashes, totals and global bounds.

`blender_import_reference.py` also deletes the `.blend1` rollback sibling Blender writes when
overwriting an existing file, so a regenerable backup can never be committed.

## Current Acro status and limitations

Exporting the Acro source with the pinned configuration yields a GLB that satisfies the entire
manifest contract and matches production on everything visually load-bearing:

| metric | production (PowerShell) | Blender export | match |
|---|---:|---:|---|
| primitives | 21 | 21 | yes |
| triangles | 8 270 | 8 270 | yes |
| indices | 24 810 | 24 810 | yes |
| bounds min | `[-0.92, -0.117, -0.89]` | `[-0.92, -0.117, -0.89]` | yes |
| bounds max | `[0.92, 0.795, 0.91]` | `[0.92, 0.795, 0.91]` | yes |
| materials (names + factors) | 8 | 8 | yes |
| primitive order | authored | pinned by prefix | yes |
| vertices | 5 811 | 5 903 | no (+92) |
| bytes | 252 816 | 206 116 | no |
| material index order | authored | first-use | no |
| node names | `Fuselage` … | `00_FUSELAGE` … | no |
| scene nodes | 21 | 22 (+`ACRO_ROOT`) | no |

The +92 vertices are Blender re-splitting vertices along normal and material seams; silhouette,
shading normals and triangle topology are unchanged.

Limitations, stated plainly:

- The `.blend` is a **semantic reorganisation of the existing G3C-B procedural geometry**, not
  remodelled art. The silhouette, topology and eight-material livery are G3C-B's.
- No UV maps and no textures, so `texcoord_0_required` is `false`.
- No per-side separation of wing, horizontal tail, main gear or wheels, because the source geometry
  has none.
- Inherited from G3C-B: opaque canopy, static propeller, no cockpit or interior, no runtime LOD
  selection, no G3J distance-visibility preservation.
- `counts.blender_export` is pinned to Blender 5.2.1 LTS / glTF I/O v5.2.40 on Windows; a toolchain
  upgrade requires deliberately re-recording it.
- No CI job runs Blender, so the Blender-side steps are developer-run. `validate_glb.py` and
  `test_asset_contract.py` are pure standard library and are CI-ready, but wiring them into
  `.github/workflows/ci.yml` was outside this slice's scope.
- **A production cutover has not happened.** It remains a future slice that must reconcile material
  index order and node names with the positional assertions in `aircraft_asset_g3c.rs`, and re-run
  the full asset and physics-fingerprint test set.

## Path to RV2-7B: SIG Kadet LT-40 EGV

The pipeline is generic by construction: every asset-specific fact lives in one manifest and every
script takes `--asset-id`. RV2-7B does not need new tooling, only a new manifest and a new source.

Current Kadet state, verified: `models/sig_kadet_lt40_egv/` contains only `model.json` and
`README.md` — **there is no presentation asset at all**, and `model.json` declares
`"presentation": null`. So RV2-7B creates the Kadet's first visual asset rather than replacing one,
which means it also has to introduce the `presentation.articulated_surfaces` block and choose its
primitive mapping deliberately. There is no legacy index order to preserve.

Steps:

1. Copy `acro_electric_01_manifest.json` to `sig_kadet_lt40_egv_manifest.json` and re-declare
   `asset_id`, `paths`, `components`, `moving_surfaces`, `materials`, `bounds`, `counts`,
   `attribute_contract` and `orientation_checks` for the Kadet's real geometry.
2. Model in Blender against the reference geometry, then run the same four steps: materialise (or
   author directly), validate source, export to scratch, validate the GLB.
3. Declare the Kadet's real layout honestly, derived from the reference evidence rather than from
   Acro's. Its component set will not resemble Acro's tricycle layout, and the
   `orientation_checks.foremost_component` / `up_reference` / `forward_reference` entries must be
   re-pointed at components that actually exist. Where the reference lists an entry under
   `unknowns`, do not invent geometry to fill it.

Two properties of the reference data must be respected:

- **It is evidence, not configuration.** `docs/reference_aircraft/data/sig_kadet_lt40_geometry_v0.json`
  declares `"artifact_kind": "reference_geometry_evidence_not_runtime_configuration"` and
  `"runtime_ready": false`. Together with
  `docs/reference_aircraft/sig_kadet_lt40_egv_geometry_reconstruction.md` it is a **visual**
  geometric reference only. It must not feed back into mass properties, aerodynamics, propulsion or
  survey evidence, which stay authoritative in their own committed files, and it must not be
  converted into a runtime configuration by this pipeline.
- **It is in a different frame.** The reference uses a documentary origin at the wing root leading
  edge in the wing reference plane, with **x aft positive, y right positive, z down positive**, and
  publishes an explicit `runtime_body_conversion`:
  `x_body_m = cg_x_aft_from_wing_le_m - x_aft_m`, `y_body_m = y_right_m`,
  `z_body_m = -height_up_m`. Note that this conversion already re-references X to the **CG**, not to
  the wing leading edge, while `runtime_origin_relation` is `null` with quality `unknown`. The
  Blender source must be built in the render-body frame (`+X` right, `+Y` up, `-Z` forward) via that
  conversion — never by copying reference coordinates directly — and the visual datum offset has to
  be chosen and documented explicitly, the way Acro's `+0.255 m` datum is, without disturbing the
  physical model.
- **It names surfaces, not components.** The JSON has no component list. Its surface blocks are
  `wing`, `ailerons`, `horizontal_tail`, `elevator`, `vertical_tail` and `rudder`, supported by
  `longitudinal_datums`, `control_travel_geometry`, `propulsion_axis`, `calibration`,
  `consistency_checks` and `unknowns`. A Kadet semantic component set still has to be derived from
  that evidence — and anything listed under `unknowns` (including the elevator hinge-line height,
  rudder hinge coordinates and the tail-arm definition) must not be invented. `unknowns[2]` also
  records that the reference data cannot tell single from split elevator halves, so a
  `ELEVATOR_L`/`ELEVATOR_R` split must not be assumed.

## Verification performed in this slice

| check | result |
|---|---|
| `cargo fmt --all -- --check` | pass (no diff) |
| `cargo check --workspace --all-targets` | pass |
| `cargo clippy --workspace --all-targets -- -D warnings` | pass, zero warnings |
| `cargo test --workspace --all-targets` | pass: 1721 passed, 0 failed, 17 ignored |
| `validate_glb.py` on production GLB, profile `production` | pass, exit 0 |
| `validate_glb.py` on scratch export, profile `blender_export` | pass, exit 0 |
| `blender_validate_source.py` on the `.blend` | pass, exit 0 |
| `blender_selftest_source_validation.py` | 16/16 violations rejected |
| `test_asset_contract.py` | 31/31 pass |
| export reproducibility | 3/3 byte-identical GLB |
| export guard against `models/` | refused, exit non-zero |
| production GLB / `model.json` SHA-256 | unchanged |
