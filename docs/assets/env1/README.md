# ENV1 open assets — provenance index

This directory records every externally sourced (non-original) asset that ENV1
brings into the repository, with enough metadata to re-acquire, re-verify and
re-process it from scratch.

| File | Purpose |
| --- | --- |
| `env1_open_assets.json` | Machine-readable provenance manifest. The authority for digests. |
| `README.md` | This file. |
| `LICENSES.md` | Licence text and attribution obligations per provider. |

## Scope

ENV1-A registers exactly one asset:

| `asset_id` | Provider | Slug | Licence | Runtime use |
| --- | --- | --- | --- | --- |
| `ENV1-GND-01` | Poly Haven | `sparse_grass` | CC0 | Terrain FINAL path — base color, normal, roughness |

## What the manifest records

Per asset, the manifest carries the fields the ENV1 contract requires and
nothing invented:

- identity: `asset_id`, `provider`, `slug`, `name`, `source_page`, `license`,
  `license_url`, `authors`;
- API provenance: the `info` and `files` endpoint URLs, the SHA-256 of each
  verbatim JSON payload, the API's own `files_hash`, `date_published`,
  `max_resolution`, `dimensions`, `category`, `description`, the fetch timestamp
  and the identifying `User-Agent`;
- source files: for each of the three maps, the download URL, the size and MD5
  the API published, the measured local size, MD5 and **SHA-256**, the
  verification flags, and the measured PNG header (4096x4096, bit depth 16);
- processing: recipe version, the processor command and module, source and
  runtime edge lengths, the reduction filter, per-map colour-space handling, the
  alpha statement, and the explicit `not_applied` list;
- runtime outputs: for each of the three committed maps, the repository path,
  colour-space semantic, channel layout, byte size, **SHA-256** and PNG header.

### Deliberate nulls

`api.dimensions_unit` is `null`. The Poly Haven `/info` endpoint reports
`dimensions: [2000, 2000]` with **no unit**, so the value is recorded verbatim
and the unit is left null rather than guessed; `dimensions_unit_note` states
this. ENV1-A therefore does not derive any physical tiling scale from that
field — the terrain base tile scale is an explicit, documented material
parameter in `crates/renderer/src/terrain.rs`.

`license` is `CC0`, sourced from <https://polyhaven.com/license> rather than
from the API: the `/info` payload carries no licence field. `license_note`
records that basis. This is the same citation the repository already uses in
`tools/vegetation_processing/PROVENANCE.md`.

## Source cache is not committed

The 4k sources total roughly 212 MB and live under the gitignored
`tmp/env1_source_cache/polyhaven/sparse_grass/`. They are re-downloadable and
digest-verified, so the repository carries only their digests. The three
committed runtime maps total roughly 24 MB.

## Reproducing

```bash
# 1. acquire and verify the sources (network; Poly Haven public API)
python -X utf8 tools/env1_asset_pipeline/fetch_polyhaven_asset.py

# 2. regenerate the runtime maps (deterministic)
cargo run -p renderer --bin process_env1_terrain_material

# 3. re-record the provenance manifest
python -X utf8 tools/env1_asset_pipeline/build_manifest.py

# 4. fail-closed verification of the whole chain
python -X utf8 tools/env1_asset_pipeline/verify_env1_assets.py --reprocess
```

Step 4 exits non-zero if any recorded digest no longer matches the bytes on
disk, and `--reprocess` additionally requires the Rust processor to reproduce
the committed runtime maps byte-for-byte from the cached sources.

Powered by Poly Haven (<https://polyhaven.com>).
