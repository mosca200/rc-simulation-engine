# PV1 — Production Vegetation Assets (committed GLB)

Status: implemented renderer-only slice. Exact base:
`535fbe1e0e662bc46443120f95f7094d224b3bd3` (G3 visual remediation merged
onto the current integration baseline).

## Outcome

The production vegetation no longer reconstructs tree meshes procedurally at
runtime. PV1 bakes the deterministic asset set to committed GLB files, embeds
them into the renderer with `include_bytes!`, and decodes them at startup
through the shared GLB loader. G3D instancing, culling, LOD hysteresis,
batching, shadow passes, PBR materials, and the GPU instance layout are
untouched.

Before PV1 the runtime called `deciduous_lod` / `conifer_lod` and their
`puff_blob` / `tube_sweep` / `frond_mesh` builders every time the asset set
was constructed. After PV1 those builders run exclusively inside the offline
generator `crates/renderer/src/bin/generate_vegetation_glbs.rs`; the runtime
consumes the baked files.

## Provenance

The 18 committed GLB files in `crates/renderer/assets/vegetation/`
(`6 assets × 3 LOD classes`) are project-original, generated in-repo by the
deterministic offline tool:

```text
cargo run -p renderer --bin generate_vegetation_glbs
```

- One file per `(asset, LOD)`: `field_<name>_lod{0,1,2}.glb`, each with two
  primitives ordered `bark` then `foliage`, distinct PBR materials.
- The generator is a pure function of seed-fixed asset parameters: identical
  bytes on every platform/run (fixed little-endian BIN packing, `serde_json`
  ordered `Map`, 4-byte GLB alignment).
- No external files, no third-party downloads, no CC0 imports — provenance is
  the in-repo generator plus the seed-per-variant table documented in
  `vegetation_assets.rs` (`deciduous_params` / `conifer_params`).
- Regression gate: `committed_glbs_match_the_offline_bake_bitwise` regenerates
  the set and compares byte-for-byte against the embedded slices, so the
  committed assets and the generator cannot drift (same contract as the
  terrain committed-asset gate).

## Asset density (bake quality)

The LOD fidelity tables were raised from the VR1 values so the baked
silhouettes read richer while every ratio stays inside the pinned ranges:

| Table | Before (VR1) | PV1 |
| --- | --- | --- |
| `DECIDUOUS_LOD0` | (14, 6, 6, 10, 10, 6) | (16, 7, 6, 12, 12, 8) |
| `DECIDUOUS_LOD1` | (10, 4, 4, 8, 8, 4) | (12, 5, 4, 8, 10, 6) |
| `CONIFER_LOD0` | (8, 6, 10, 6) | (8, 6, 12, 8) |
| `CONIFER_LOD1` | (5, 4, 8, 5) | (5, 4, 10, 6) |

Measured LOD0 (bark + foliage tris): oak-like 3414, pine-like 2172,
spruce-like 2612, fir-like 2132 — all under the 4000/asset budget, total
production LOD0 under 20k, LOD1/LOD2 ratios inside 35–60% / 8–28%.
Per-asset silhouette contracts (ground contact, ragged conifer rim, organic
deciduous canopy) remain pinned by tests and pass on the decoded GLBs.

## Runtime integration

- `VegetationAssetSet::production()` → `from_committed()` iterates the
  embedded `COMMITTED_GLB` table, decodes each LOD with
  `glb::load_glb_bytes`, and reassembles `VegetationLod { bark, foliage }`
  from the two primitives. Bounds/height are derived from the decoded LOD0.
- `VegetationAssetSet::bake_source_set()` stays as the exact procedural
  source used by the offline generator and the provenance tests.
- `glb::load_glb_bytes` was added next to `load_glb_asset` (shared core via
  `gltf::Gltf::from_slice`) so embedded slices load without touching the
  filesystem.
- `export_glb_lod(asset, class)` generalizes the previous LOD0-only export to
  any LOD class for the offline bake.
- Renderer (`gpu.rs`), world/culling/LOD (`vegetation.rs`), and the 48-byte
  instance layout are unchanged.

## Verification

```text
cargo fmt --all -- --check                                    PASS
cargo check --workspace --all-targets                         PASS
cargo clippy --workspace --all-targets -- -D warnings         PASS
cargo test --workspace --all-targets                          0 failures
cargo build --workspace --release                             PASS
```

Runtime smoke on the RTX 3090 reference: `play --scenery flying-field`
(startup, no GPU validation errors), `--vegetation-debug lod` (LOD bands),
`--exposure-ev 1.0`. A/B screenshots against the base capture the denser
canopy silhouette.

## Residual gaps (unchanged scope)

- No wind animation; foliage content is vertex color + PBR as before.
- The baked assets are deterministic in-repo originals; artist-authored or
  external CC0 assets are future provenance options, not this slice.
- A permanent FPS/1% low measurement path is still outstanding.