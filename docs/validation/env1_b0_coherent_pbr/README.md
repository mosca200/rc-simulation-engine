# ENV1-B0 clean validation evidence

`env1b0_evidence.json` is the machine-readable authority for the captures,
receipts, audits, timings, image digests, and pixel comparisons summarized
here. Raw candidate PNGs remain under the gitignored `tmp/` tree and are not
committed. `visual_pass` remains `null`; valid capture evidence is not an
automatic visual-quality verdict.

## Authorities and method

| Side | Commit | Dirty | Build |
| --- | --- | --- | --- |
| BEFORE | `1ec35c7c92dda6e15527ecb5fe50fd5f82d1bb0e` | `false` | release, workspace, all features |
| AFTER | `679eca0bfee9f7d69efdab03f1e1ed14a395bad8` | `false` | release, workspace, all features |

Both sides used the same committed canonical manifests, fixed camera,
aircraft, scenery, exposure, warmup, and capture frame. The device reported by
RuntimeVisualAudit was `NVIDIA GeForce RTX 3090` using Vulkan and the NVIDIA
driver. Each capture completed with a trusted runtime receipt, independently
verified RGBA8 PNG, valid capture evidence, and valid RuntimeVisualAudit.

ENV1-B0 uses one 2.0 m world-space `base_uv` for the photographed albedo,
OpenGL tangent normal, and linear roughness. Sample B applies the retained
1.370 scale, +27 degree UV rotation, and `[0.315, 0.571]` offset to that common
coordinate. All three channels use the same 0.5 A/B weight. Sample B's normal
XY is transformed by `R(-27 degrees)` before a normalized linear normal blend;
roughness is blended linearly. The default roughness factor is 1.0 and
metallic remains 0.0. The inherited 48.0 m macro and 0.40 m detail presentation
carriers remain in place.

## Captured timing facts

Times below are individual RuntimeVisualAudit facts, not a summed whole-GPU
frame time. In all six FINAL captures the GPU sample came from presentation
frame 9 and was one frame old at capture frame 10.

| Resolution | Metric | BEFORE | AFTER | Delta |
| --- | --- | ---: | ---: | ---: |
| 1920x1080 | scene GPU | 1.171456 ms | 1.231872 ms | +0.060416 ms (+5.157%) |
|  | temporal resolve GPU | 0.028672 ms | 0.028672 ms | 0.000000 ms (0.000%) |
|  | postprocess GPU | 0.020480 ms | 0.020480 ms | 0.000000 ms (0.000%) |
|  | CPU frame | 17.233100 ms | 17.421500 ms | +0.188400 ms (+1.093%) |
| 2560x1440 | scene GPU | 1.575936 ms | 1.703936 ms | +0.128000 ms (+8.122%) |
|  | temporal resolve GPU | 0.053248 ms | 0.052224 ms | -0.001024 ms (-1.923%) |
|  | postprocess GPU | 0.033792 ms | 0.032768 ms | -0.001024 ms (-3.030%) |
|  | CPU frame | 19.046900 ms | 19.889800 ms | +0.842900 ms (+4.425%) |
| 3840x2160 | scene GPU | 2.694144 ms | 2.961408 ms | +0.267264 ms (+9.920%) |
|  | temporal resolve GPU | 0.124928 ms | 0.122880 ms | -0.002048 ms (-1.639%) |
|  | postprocess GPU | 0.076800 ms | 0.073728 ms | -0.003072 ms (-4.000%) |
|  | CPU frame | 24.696400 ms | 25.491500 ms | +0.795100 ms (+3.219%) |

These are single deterministic capture-frame observations, not a statistical
benchmark. The added registered normal and roughness B samples affect the
scene pass as expected; temporal resolve and postprocess are effectively
unchanged at the granularity of these samples.

## Image and debug evidence

The FINAL PNG SHA-256 pairs (BEFORE / AFTER) are:

- 1920x1080: `db794b164e693b9010b58a731cddf706a201361d1baf4f2ef40b6650a5cef98e` / `24ad0fa1b322c56777285f8e06672e4554ceecfc2f94d6de47b07af816caacac`
- 2560x1440: `838f7aa2c705bfb1017d8ed960b16a803742780b05b5ba96d957a2a526322e20` / `81e895354bbb3c051a5d405b4f28b715a0880166023e548a6a6080715e5eec62`
- 3840x2160: `63d5be3792e6ffd9cf4902f155df21428c709ec7b5e5d94555c18ad112848d72` / `4185c6ca89bf60e1f656a440205ca49968524543ddc9ed9f1b11c5f439d86731`

All five AFTER debug captures were valid and their audits reported the
requested terrain selector:

| Debug mode | PNG SHA-256 |
| --- | --- |
| albedo | `28721d979f08312833ff42cf4cada9f387b179ee0e868236dac2bfbabe85d6a3` |
| normal | `c1c17bf0b18f72afe5cdd74e7894c719e6f139fde0674a1e739cc2a3ac7b631b` |
| roughness | `1ecd0627f1e43084236306ce8cda166e76b03c0041e3f8839874e3c0fd7c1f70` |
| macro | `21e6dc7dd8604e8780ec275d210150f77df4df8043c482ddcab8dec272c6582d` |
| detail | `d23a35a96d52f011dcf2b8ff1c13a60bf62b8b73989c3855d01e156aba849a6e` |

Manual still-image inspection found the expected terrain-response change and
no obvious new grid or seam. This observation is deliberately not promoted to
a PASS: still captures cannot establish temporal shimmer behaviour, and no
approved visual threshold exists.

## Runtime asset integrity

The committed Sparse Grass files were not regenerated or modified. Their
before/after SHA-256 values are identical:

| Map | SHA-256 |
| --- | --- |
| base color | `3721a8f28dc8a2de86f29b271de8d7f4e7f826265e1e7e0a16b017da29f3d28f` |
| normal | `6af58131ecce8d40017bf534e9f2cb9bb1e0f49b89fe9afeaa4cecaf0ba04575` |
| roughness | `b5bd29f723b8717c1401dfc05afb6209dd0b35857b53bb47f4c84cda1b64188a` |

## Validation

- `cargo fmt --all -- --check`: passed.
- `cargo check --workspace --all-targets`: passed.
- `cargo clippy --workspace --all-targets -- -D warnings`: passed.
- `cargo test --workspace --all-targets`: passed; renderer library summary was
  483 passed and 17 ignored.
- `cargo build --workspace --release`: passed; both capture authorities also
  built release workspace/all-features binaries.
- ENV1 Python pipeline: 45/45 tests passed; normal verification passed; local
  source-cache `--reprocess` reproduced all three runtime outputs byte-for-byte.
- Renderer ignored tests: all 16 GPU render/readback tests passed. The one
  unrelated vegetation offline-bake byte-comparison test failed; the same
  isolated failure reproduces at the untouched BEFORE commit, so it is a
  pre-existing baseline issue rather than an ENV1-B0 regression.

## Retained limitations

ENV1-B0 does not add four-material terrain blending, terrain biome/splat maps,
dry/wet/worn local zones, dedicated macro or micro/detail textures, 3D grass,
ground clutter, production foliage, alpha-coverage-preserving foliage mips,
vegetation transmission, GLB normal/metallic-roughness GPU rendering, the GLB
shared-image/different-sampler cache fix, virtual texturing, texture
compression, or terrain/physics height-authority convergence.
