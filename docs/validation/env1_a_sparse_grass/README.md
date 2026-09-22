# ENV1-A — BEFORE/AFTER validation evidence

Canonical VIS reference capture of `aircraft_acro_static_front`, rendered with
`--renderer v2`, on the real GPU. **BEFORE** is the release binary built at the
base SHA `e84876a1b93b9ccf7a4d04af777bf3d4eb2a0882` *before any ENV1-A edit*;
**AFTER** is the release binary carrying the ENV1-A Sparse Grass material. Both
runs use the identical manifest, so camera, scenery, exposure, aircraft state,
warmup and capture frame are held constant by construction — the terrain
material is the only intended variable.

| File | Purpose |
| --- | --- |
| `env1a_evidence.json` | All measurements, verbatim runtime facts, and every unavailable metric with its reason. The authority. |
| `env1a_reference_1440p.json`, `env1a_reference_2160p.json` | Resolution variants of the canonical scene (only `resolution`, `scene_id`, `capture.filename` and `description` differ). 1920x1080 uses `docs/validation/visual_benchmark/vis0_reference_scene.json` unchanged. |

## Candidate captures are not committed

These captures carry `visual_pass = null` and are **not** an approved golden or
beauty baseline, so the raw PNGs are transient validation artifacts and are kept
out of git history. The canonical rule is:

- raw captures stay under the gitignored `tmp/` tree while they are candidates;
- hashes, measurements, receipts, audits and capture evidence **are** committed;
- a PNG becomes permanent only when it is explicitly promoted to an approved
  golden/beauty baseline.

ENV1-A has no such promotion. Each artifact in `env1a_evidence.json` therefore
records `committed_png: null` with a note, plus `local_png` (the `tmp/` path),
`local_png_sha256`, the runtime receipt digest and byte size — so a capture is
fully identified and re-producible without inflating the repository. No broad
`.gitignore` rule was added, so a future explicitly approved baseline can still
be committed.

To regenerate the evidence from the existing `tmp/` runs (no re-capture needed):

```bash
python -X utf8 tools/measure_env1a_sparse_grass.py
```

Pass `--promote-png` only when a capture has actually been approved as a
baseline; it then copies the PNGs into `png/` and records committed paths.

## Reproducing

```bash
cargo build --release --workspace --all-features
python -X utf8 tools/visual_benchmark/run_benchmark.py \
  --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
  --execute --app target/release/rcsim-app.exe --output-dir tmp/env1a_after/1920x1080
# ...repeat for the 1440p and 2160p manifests, then:
python -X utf8 tools/measure_env1a_sparse_grass.py
```

## What is and is not measured

Every number in `env1a_evidence.json` is either copied verbatim from a runtime
artifact (`RuntimeCaptureReceipt`, `RuntimeVisualAudit`, `VisualCaptureEvidence`)
or computed from two PNGs on disk. Four requested metrics have **no source of
truth in this repository** and are recorded as `null` with a written reason
rather than estimated:

| Metric | Status |
| --- | --- |
| GPU per-pass timing | Measured — six `PassId` passes with `gpu_duration_ns`, plus `gpu_timing_status` recorded verbatim. |
| CPU render timing | Measured — `cpu_frame_duration_ns` and per-pass `cpu_duration_ns`. |
| GPU whole-frame time | **Not available.** Only per-pass durations exist; a sum would be our arithmetic, not a measurement. |
| Terrain timing | **Not available.** The terrain is drawn inside the `scene` pass; there is no terrain pass and no terrain-specific instrumentation. The audit's `terrain` block is configuration only. |
| Draw calls | **Vegetation only** (`vegetation.stats.scene_draw_calls` / `shadow_draw_calls`). No whole-frame counter exists. |
| VRAM | **Not available.** No wgpu memory query, no NVML, no `nvidia-smi` scraping anywhere in the repository, and the `VisualCaptureEvidence 1.0.0` schema forbids a `hardware.vram_gb` key. |
| `visual_pass` | **`null`.** Locked to `null` by the schema (`"type": "null"`); capture evidence records facts, never a visual verdict. |

The pixel comparison (changed pixels, peak channel difference, mean absolute
difference, per-band breakdown) is an objective description of *how much* the
frame changed. It is deliberately **not** a quality verdict.
