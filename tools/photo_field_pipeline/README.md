# PF1 photo-field asset pipeline

Offline Python tooling for slice **PF1 - Photo Flying Field v1**. It turns one CC0
photographic 360 panorama (Poly Haven `meadow`) into the three committed runtime
assets the renderer loads, plus the provenance record that explains every number
in them.

The runtime is deliberately dumb about this asset: it decodes a JPEG, samples it
as an equirectangular background, and renders an invisible coarse depth proxy in
front of it. All judgement - tone mapping, exposure, calibration, geometry - is
made here, offline, where it can be measured, reviewed and reproduced.

```
tools/photo_field_pipeline/
  photo_field_assets.py            shared library (constants, digests, RGBE decoder,
                                   tone mapper, sRGB, equirect math, GLB writer)
  fetch_photo_field_sources.py     acquire + verify the Poly Haven sources (gitignored)
  process_photo_field_panorama.py  HDR -> display-referred sRGB JPEG (committed)
  author_photo_field_proxies.py    calibration -> coarse depth-proxy GLB (committed)
  build_photo_field_manifest.py    runtime manifest + provenance record (committed)
  test_photo_field_assets.py       unittest suite (run BY PATH)
```

## Commands

Run from the workspace root, always with `-X utf8`, always by path:

```
python -X utf8 tools/photo_field_pipeline/fetch_photo_field_sources.py
python -X utf8 tools/photo_field_pipeline/process_photo_field_panorama.py
python -X utf8 tools/photo_field_pipeline/author_photo_field_proxies.py
python -X utf8 tools/photo_field_pipeline/build_photo_field_manifest.py
python -X utf8 -m unittest tools/photo_field_pipeline/test_photo_field_assets.py -v
```

Reproducibility checks (each re-derives the committed bytes and fails closed on
any difference):

```
python -X utf8 tools/photo_field_pipeline/process_photo_field_panorama.py --check
python -X utf8 tools/photo_field_pipeline/author_photo_field_proxies.py --check
```

Useful variations:

* `fetch_photo_field_sources.py --offline` - reuse the cached API payloads and
  downloads; fails closed when either is absent, because a receipt that cannot
  cite the API is not provenance.
* `build_photo_field_manifest.py --runtime-only` - write only the runtime
  manifest, which needs no measured input. This is the escape hatch that keeps
  the renderer integration unblocked before the 108 MB source has been fetched.
* `process_photo_field_panorama.py --out PATH` / `author_photo_field_proxies.py
  --out PATH` - derive to a scratch path (a two-run digest comparison).

Order matters: `fetch` -> `process` -> `author` -> `build`. The builder measures
what the two previous steps committed and refuses to invent it.

## Committed outputs

| Path | What it is |
| --- | --- |
| `crates/renderer/assets/photofield/meadow/meadow_panorama_8192x4096.jpg` | the display-referred background, 8192x4096, JPEG q92 4:4:4 |
| `crates/renderer/assets/photofield/meadow/photo_field_depth.glb` | 9 named nodes, 724 triangles, world-space, depth-only |
| `crates/renderer/assets/photofield/meadow/photo_field_manifest.json` | the 11-key manifest `photo_field.rs` deserialises |
| `docs/assets/photofield/pf1_provenance.json` | licence, digests, calibration, and what was deliberately not done |

Nothing else belongs in `crates/renderer/assets/photofield/meadow/`; the test
suite asserts the directory holds exactly those three files.

## The processing recipe

The renderer tonemaps the 3D aircraft with the Khronos PBR Neutral curve at
`exposure_ev = 0.0`, and the photographic background **bypasses** the runtime
tonemapper. The background is therefore tonemapped here with the same curve at
the same exposure, so the aircraft and the photograph are one photometric world:

```
radiance       = decode_rgbe(meadow_8k.hdr)          # 32-bit_rle_rgbe, banded
display_linear = khronos_pbr_neutral(radiance * 1.0) # ported from shader.wgsl
srgb_bytes     = round(linear_to_srgb(display_linear) * 255)   # texture.rs curve
               -> JPEG, quality 92, subsampling 0
```

No resize, no resampling, no LUT, no saturation or contrast boost, no
sharpening, no cropping, no AI depth estimation, no photogrammetry. The
`not_applied` list in the provenance record is the authoritative statement of
what was refused, and the rounding rule is a single documented round-half-to-even.

`khronos_pbr_neutral` in `photo_field_assets.py` is a statement-by-statement port
of the WGSL function; the test suite re-reads the shader source and asserts the
port still matches it, and compares the vectorised port against an independent
scalar transcription of the same math. Note that WGSL
`select(0.04, x - 6.25 * x * x, x < 0.08)` yields the **parabolic** term for
`x < 0.08` - the PF1 prototype in `tmp/pf1_exposure_fit.py` had those two
branches swapped, which is recorded as a caveat on the exposure-fit residuals in
the provenance file.

## The equirectangular convention

One convention, used by the panorama, the calibration and the runtime
(`crates/renderer/src/photo_field.rs`):

```
u = fract(azimuth / 2pi)     azimuth   = atan2(dir.z, dir.x)
v = 0.5 - elevation / pi     elevation = asin(dir.y)
```

Row 0 of the committed JPEG is the **+90 deg zenith**; the `u = 0` column is the
render **+X** axis. So a direction towards panorama longitude/elevation is
`[cos(el) * cos(az), sin(el), cos(el) * sin(az)]`. The prototype analysis
emitted `[cos(el) * sin(lon), sin(el), cos(el) * cos(lon)]`, which belongs to
`azimuth = atan2(x, z)`; the sun direction in the brief was corrected for this
before the manifest was committed, and `test_photo_field_assets.py` pins both
facts (the corrected vector maps onto the solar texel, the superseded one maps
onto `u = 0.826`).

## Depth-proxy geometry

Frame: render world space, y-up, ground plane at `y = 0`, photographic eye at
`pilot_position_render_m`, read from `photo_field_assets.PILOT_EYE_RENDER_M`
(`[15.321, 1.6, -12.856]` at the time of writing). The eye is a manual
calibration: the aircraft spawns at the render origin, so the eye sits
`EYE_TO_SPAWN_DISTANCE_M` = 20 m from the spawn at panorama azimuth
`PILOT_EYE_AZIMUTH_DEG` = 140 deg. That azimuth keeps the aircraft line-of-sight
12 deg clear of the photographed near trunk at 152 deg (whose proxy would
otherwise hide the parked aircraft) while staying inside the brick garage's
measured span 131.75-169.94 deg, so the aircraft still sits in front of a
photographed obstacle and the occlusion test stays expressible with the real
aircraft. Nothing outside `photo_field_assets.py` hard-codes the eye: the
authoring tool, the manifests, the provenance record and the tests all read it
from the constant (or from the committed manifest).

Every proxy is placed relative to that eye:

```
world = eye + distance * (cos(azimuth), 0, sin(azimuth))
```

with `azimuth` the render azimuth the panorama samples at, so a box at azimuth A
occludes exactly the part of the photograph that shows A.

| Node | Geometry | Basis |
| --- | --- | --- |
| `pf1_ground` | 64-segment fan, r = 250 m, y = 0, centred under the eye | manual |
| `pf1_building_brick_garage` | 16.73 x 6.0 x 4.91 m box at 150.87 deg / 25.10 m | **derived** |
| `pf1_house_brick` | 10 x 7 x 4.5 m box at 268 deg / 15 m | manual (crops) |
| `pf1_house_green` | 12 x 8 x 5.5 m box at 286 deg / 32 m | manual (crops) |
| `pf1_tree_trunk_0..3` | 0.7 x 0.7 x 22 m boxes at (152, 5), (270, 7), (285, 9), (300, 6) | manual (crops) |
| `pf1_tree_ring` | 48 boxes, 7 x 5 x 40 m, on a 30 m ring with +/- 3 m jitter | manual |

724 triangles against a 4000 budget. The jitter comes from a 64-bit integer LCG
with a frozen seed: the `random` module and the clock are never used anywhere in
this package, and a test asserts that.

Nodes carry an explicit **identity TRS** because the geometry is baked into world
space: `tools/aircraft_asset_pipeline/blender_export_glb.py` documents that the
production loader reads baked positions and does not apply node transforms, while
`crates/renderer/src/glb.rs`'s scene-graph path composes them. Identity makes
both readings agree. There are no materials, textures or normals - the meshes are
depth-only, and `glb.rs` generates normals for a primitive that carries none.

## The garage derivation (angles to metres)

A single panorama gives angles, not metres, so exactly one real-world assumption
is made: the camera height, 1.6 m. With a flat ground plane and the
colour-segmented angular box of the brick garage
(azimuth `[131.748, 169.937]`, elevation `[-3.647, +7.515]`):

```
distance = 1.6 / tan(3.647 deg)               = 25.0995 m
height   = distance * tan(7.515 deg) + 1.6     =  4.9109 m
width    = distance * radians(38.188 deg)      = 16.7289 m   (arc; chord 16.4217 m)
azimuth  = centroid of the 118048-pixel component = 150.874 deg
```

`garage_derivation()` recomputes these and fails closed if they drift more than
5 cm from the calibrated values, so the geometry can never silently diverge from
the measurement it came from.

## Provenance chain

```
Poly Haven /info + /files  --(verbatim payloads, sha256, files_hash)-->  tmp/.../api/*.json
                              |
                              v
meadow_8k.hdr, meadow_tonemapped.jpg, meadow_1k.hdr
   size + MD5 verified against the API AND against the constants pinned in
   photo_field_assets.py; SHA-256 measured locally
                              |
                              v
tmp/pf1_source_cache/polyhaven/meadow/fetch_receipt_pf1.json   (gitignored)
                              |
                              v
process_photo_field_panorama.py  -> meadow_panorama_8192x4096.jpg  (sha256, bytes, dims)
author_photo_field_proxies.py    -> photo_field_depth.glb          (sha256, bytes, triangles)
                              |
                              v
build_photo_field_manifest.py    -> photo_field_manifest.json  (11 keys, nothing extra)
                                 -> docs/assets/photofield/pf1_provenance.json
```

Every digest in the provenance record is measured from the file it describes;
every angle is a measurement of `meadow_8k.hdr`; every metre that is not derived
from an angle is listed under `manually_calibrated` with its evidence. The
licence is CC0 (`https://polyhaven.com/license`), the author is Sergej Majboroda,
and the attribution recorded in the manifest is
"Powered by Poly Haven (https://polyhaven.com)".

The gitignored source cache is never required to *use* the committed assets, only
to re-derive or re-verify them; tests that need it skip cleanly when it is
absent, mirroring `tools/env1_asset_pipeline/verify_env1_assets.py`.

## Dependencies

Standard library + numpy + Pillow (both already installed in this workspace).
scipy is **not** used: the connected-component solar-disc isolation was a
one-off measurement whose result is recorded as a constant, and the pipeline that
produces committed bytes must not depend on it.
