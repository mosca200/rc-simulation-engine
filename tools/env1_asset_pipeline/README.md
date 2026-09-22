# ENV1 open-asset pipeline

Reproducible acquisition, deterministic processing and fail-closed provenance
verification for the CC0 assets ENV1 brings into the renderer.

Standard library only, matching the deliberate invariant of
`tools/visual_benchmark`: reading a PNG header needs 33 bytes and
`int.from_bytes`, not an image library. Nothing here invents metadata — every
recorded field is read from the Poly Haven API, measured from a file on disk, or
explicitly `null` with a written reason.

| File | Role |
| --- | --- |
| `env1_assets.py` | Shared library: digests, PNG IHDR parsing, the manifest contract and its validator. |
| `fetch_polyhaven_asset.py` | Acquisition. Queries the API, downloads the documented maps, verifies each against the published size and MD5, records local SHA-256, writes a fetch receipt. |
| `build_manifest.py` | Regenerates `docs/assets/env1/env1_open_assets.json` and validates it against the contract before writing. |
| `verify_env1_assets.py` | Fail-closed verification of the whole chain. |
| `test_env1_assets.py` | 36 unit tests for all of the above. |

The pixel processing itself is **not** here: it is a Rust dev-binary,
`cargo run -p renderer --bin process_env1_terrain_material`, backed by
`crates/renderer/src/env1_material.rs`. That keeps the recipe in the same
language and the same colour-space transfer functions as the runtime mip chain
that consumes its output.

## Usage

```bash
# acquire + verify the 4k sources into the gitignored cache
python -X utf8 tools/env1_asset_pipeline/fetch_polyhaven_asset.py
# reuse the cached API payloads without contacting Poly Haven
python -X utf8 tools/env1_asset_pipeline/fetch_polyhaven_asset.py --offline

# regenerate the committed 2048 runtime maps
cargo run -p renderer --bin process_env1_terrain_material

# re-record provenance
python -X utf8 tools/env1_asset_pipeline/build_manifest.py

# verify (add --reprocess to also require byte-identical reprocessing)
python -X utf8 tools/env1_asset_pipeline/verify_env1_assets.py --reprocess
```

Run from the workspace root. `-X utf8` is required: a redirected Windows stdout
defaults to cp1252 and raises `UnicodeEncodeError` on non-ASCII output.

## Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Verified / acquired / written successfully. |
| 1 | A digest mismatch, a contract violation, or an API payload that does not offer the documented file. Fail closed. |
| 2 | A required input is missing (no manifest, `--offline` with an empty cache, network failure). |

## Fail-closed guarantees

- A download whose size or MD5 does not match the value the API published is
  recorded as unverified and the fetch exits 1. `build_manifest.py` then refuses
  to build a manifest from an unverified receipt.
- A source whose on-disk SHA-256 no longer matches the receipt is rejected by
  `build_manifest.py`, so a mutated cache cannot be laundered into provenance.
- `verify_env1_assets.py` recomputes every committed runtime digest and PNG
  header and exits 1 on any mismatch; a missing runtime map is a failure, not a
  skip.
- Source-cache absence is the one tolerated condition: the cache is gitignored,
  so verification reports those checks as skipped and still enforces the
  committed side. The recorded digests remain the authority.
- The manifest validator rejects a missing required field, a non-CC0 licence, a
  guessed `dimensions_unit`, a duplicate or malformed `asset_id`, a wrong recipe
  version, a wrong runtime edge, a wrong PNG colour type, a malformed digest and
  a shortened `not_applied` list.

## Tests

```bash
python -X utf8 -m unittest tools/env1_asset_pipeline/test_env1_assets.py -v
```

The two tests that need the gitignored source cache (and the `--reprocess`
byte-identity test) skip themselves when it is absent, so the suite is green on
a fresh clone while still exercising every fail-closed path wherever the sources
exist.

Powered by Poly Haven (<https://polyhaven.com>).
