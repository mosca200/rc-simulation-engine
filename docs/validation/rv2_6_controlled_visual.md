# RV2-6 controlled visual validation

This validation path isolates RV2-6 physical aerial perspective from the
FlyingField vegetation-culling distance and from the renderer-V1/V2 feature
boundary. It is developer-only and does not change the production defaults.

## Architecture

`--rv2-6-validation-scene` replaces the aircraft presentation with one
deterministic, matte, neutral split-tone target and selects a fixed pilot
camera. It also forces flat ground, zero throttle, no scenery, no vegetation,
final material channels, and no debug overlay. The presentation pose is frozen
at its initial state. Target extent scales with distance so the target remains
readable without changing its neutral material or the camera FOV.

The available cases are `near`, `100m`, `500m`, `1000m`, `frontlit`,
`sidelit`, and `backlit`. The distance cameras are at nominal distances of 5,
100, 500, and 1000 metres. The three lighting cases retain the fixed
production sun and place the camera at the documented 100 m azimuths; no
geometry can occlude the target.

`--rv2-6-validation-ap on|off` is accepted only with the validation scene and
only by renderer V2. `on` is the default. Both states initialize the complete
RV2-5 physical atmosphere, LUTs, IBL, render graph, lighting, materials, and
postprocess identically. `off` changes one persistent validation-control value
in the RV2-6 uniform, causing only the geometry aerial-perspective compositing
function to return its unmodified surface radiance. No coefficient is changed
and no resource is created per frame.

## Capture

On Windows, run from the repository root:

```powershell
powershell -ExecutionPolicy Bypass -File tools/capture_rv2_6_controlled_visual.ps1
```

The script builds `rcsim-app` in release mode, opens each real winit viewport,
waits six seconds, and saves its full 1280 x 720 client area as lossless PNG.
It captures AP ON/OFF for all four distances and all three sun-relative cases.
Use `-Force` only when intentionally replacing a complete evidence set.

An individual case can be inspected with:

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --exposure-ev 0.0 --rv2-6-validation-scene 500m --rv2-6-validation-ap on
cargo run -p rcsim-app --release -- render --renderer v2 --exposure-ev 0.0 --rv2-6-validation-scene 500m --rv2-6-validation-ap off
```

The primary evidence directory is:

`docs/validation/rv2_6_controlled_visual/`

The controlled capture produced these 1280 x 720 lossless pairs:

- `AP_ON_near.png` / `AP_OFF_near.png`
- `AP_ON_100m.png` / `AP_OFF_100m.png`
- `AP_ON_500m.png` / `AP_OFF_500m.png`
- `AP_ON_1000m.png` / `AP_OFF_1000m.png`
- `AP_ON_frontlit.png` / `AP_OFF_frontlit.png`
- `AP_ON_sidelit.png` / `AP_OFF_sidelit.png`
- `AP_ON_backlit.png` / `AP_OFF_backlit.png`

Capture review confirmed that the neutral target is present, vegetation is
absent, no target is occluded, and every ON/OFF pair has matching framing.

## Measured separation

`tools/measure_rv2_6_controlled_visual.ps1` compares each lossless pair and
reports the changed pixel count, the peak per-channel difference, and the mean
absolute difference per channel:

```powershell
powershell -ExecutionPolicy Bypass -File tools/measure_rv2_6_controlled_visual.ps1
```

The bottom five rows are excluded from every figure below: the window copy used
for capture paints the rounded corners and shadow over them, and that band is
the only place in the frame where a difference above one 8-bit level appears.

| Case | Changed px | Mean abs / channel | Peak channel difference |
| --- | --- | --- | --- |
| near | 29 128 (3.18%) | 0.0108 | 1 (target box), 3 (corner band) |
| 100m | 20 814 (2.27%) | 0.0077 | 1, 3 |
| 500m | 21 233 (2.32%) | 0.0079 | 1, 3 |
| 1000m | 910 (0.10%) | 0.0007 | 2 (target box), 3 |
| frontlit | 30 321 (3.31%) | 0.0113 | 1 |
| sidelit | 23 903 (2.61%) | 0.0089 | 1, 3 |
| backlit | 19 175 (2.10%) | 0.0071 | 1, 3 |

Two controls bound those numbers:

- **Capture noise floor.** Capturing `500m` with AP `on` twice in a row and
  comparing the two images reports 4 changed pixels, all inside the corner band,
  and 0.0000 mean absolute difference. Scene content is reproducible pixel for
  pixel, so the ON/OFF differences above are signal, not capture noise:
  `tools/measure_rv2_6_controlled_visual.ps1 -Left FIRST.png -Right SECOND.png`.
- **Provenance.** Re-capturing `500m` with AP `on` from a clean release build
  reproduces the committed `AP_ON_500m.png` with 0 changed pixels, and `1000m`
  with AP `on` differs only inside the corner band. The committed pairs
  therefore correspond to the committed code.

The direction of the change matches physical aerial perspective: in every case
the majority of changed pixels are brighter with AP `on` (21 994 of 29 128 at
`near`, 16 946 of 21 233 at `500m`), which is in-scattering added on top of
attenuated surface radiance, and the sky stays bit-identical because the
validation switch only reaches the geometry compositing function.

**The measured separation stays below the 8-bit output quantization floor.** No
scene pixel in any pair changes by more than one level, except for 212 pixels
inside the `1000m` target box that change by two. At these distances, with the
unmodified RV2-5 clear atmosphere, aerial perspective is a sub-quantization
radiometric change rather than a visible one. The pairs therefore demonstrate
that the AP seam is isolated and directionally correct; they do **not**
demonstrate a visible aerial-perspective difference. No correction or
atmospheric tuning was made from these observations.

Recording a visible separation would need a scene whose path length or aerosol
load exceeds one 8-bit level (kilometre-scale terrain, or a denser documented
atmosphere). That is outside this validation path and remains an open finding.

Visual conclusions remain a manual review gate. Neither the capture script nor
the measurement tool assigns PASS or FAIL.
