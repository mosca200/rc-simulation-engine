# Gate 0 — RC Simulation Engine convergence

**Decision:** PARTIAL  
**Branch:** `integration/gate0-convergence`  
**Tested code commit:** `181c74858e355f2634217bc4ddb4de064188b0a8`  
**Date:** 4 September 2026, Europe/Rome

The technical convergence is complete and clean. Physics, validation, presentation, the reviewed
M2.6C delta, mandatory checks, acceptance, both release benchmarks, and two viewer captures are
present on one branch. The gate is not called PASS because the checked-in GLB assets do not expose
articulated surfaces or texture-rich materials to the running viewer. Technical, Manual, and
Real-world gates remain separate.

## Preflight and integration matrix

| Branch | Tip | Dependency/content | Decision |
|---|---|---|---|
| `main` | `b51bece` | M2.6B.1/M2.7B and XFOIL M2.9A/B | No duplicate merge: tree equals approved ancestor `ef2977f` |
| `integration/m2-9c-on-main` | `85c4e99` | deterministic XFOIL campaign coverage | No duplicate merge: tree/patch equals approved ancestor `cfbf842` |
| `integration/m2-approved-stack` | `b02ffcc` | approved physics/validation through M2.12A | Integrated by exact ancestry of complete demo |
| `integration/complete-flight-demo` | `c4864eb` | approved stack plus ground and presentation foundation | Baseline parent |
| `feature/m2-6c-trim-domain-qualification` | `a69641a` | unique offline qualification hardening | Reviewed, merged as `9243aba`, corrected by `181c748` |

Preflight found 17 registered worktrees: one dirty and 16 clean. The dirty primary checkout remains
owned by `primary-working-copy` and was not used for integration. Its `.gitignore`,
`docs/assets/aaa_visual_target_v1.png`, and `docs/reviews/rcsim_project_review_2026-09-04.md`
were not moved, overwritten, stashed, or committed here. The M2.6C worktree was already clean at
`a69641a`; the original branch remains unchanged.

## Duplicate and conflict handling

- `tree(b51bece) == tree(ef2977f)`.
- `tree(85c4e99) == tree(cfbf842)`; both M2.9C commits have patch-id
  `77e2b1308010d9829b60620f19b666e8948b9419`.
- `integration/m2-approved-stack` is an exact ancestor of `integration/complete-flight-demo`.
- A conservative single-base merge simulation exposed the expected genealogical overlap in
  `crates/app/src/main.rs`, `crates/model/src/lib.rs`, plus add/add
  `crates/model/src/reference_xfoil.rs`. The recursive live merge simulation showed that `main +
  complete` resolves cleanly. The chosen order avoids duplicating these outcomes. The retained
  complete versions contain every CLI subcommand and the M2.9A–L/v0–v8 public model APIs; the
  workspace tests and `render --help` verify the resulting surface.
- The actual `complete + a69641a` merge had one textual conflict in
  `crates/aircraft/src/trim_qualification.rs`. Resolution retained the later slipstream/downwash
  runtime helpers and the new offline qualification API. The old `RigidBodyState`/
  `compute_section_kinematics` imports were obsolete on the complete physics path.
- The corrective commit makes invalid range input fail closed, honors bitwise endpoint identity,
  gives integrity failures top-level precedence, preserves all diagnostics, and updates the one
  downstream exhaustive match. It changes no physics parameter or 500 Hz code path.

## Mandatory checks

| Command | Result |
|---|---|
| `cargo fmt --all -- --check` | PASS, exit 0 |
| `cargo clippy --workspace --all-targets -- -D warnings` | PASS, exit 0 |
| `cargo test --workspace --all-targets` | PASS, exit 0 |
| `cargo build --workspace --release` | PASS, exit 0 |
| `cargo run -p rcsim-app --release -- validate first-slice --output-dir target/gate0_first_slice` | Executed successfully, exit 0 |

M2.6C discriminating checks additionally passed: 46 trim qualification integration tests, 7
qualification unit tests, and 11 slipstream/downwash integration tests. The complete suite includes
the zero-allocation, deterministic replay/fingerprint, 500 Hz, NED/FRD, Hamilton quaternion, SI,
and renderer dependency-boundary checks.

First-slice result: **Technical PASS / Manual PARTIAL / Real-world PARTIAL / overall PARTIAL**.
There were 31 criteria: 24 PASS, 1 PARTIAL, 6 NOT_TESTED, 0 FAIL. The Acro fingerprint is
`dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed`.

Generated acceptance artifacts:

- `docs/validation/gate0/first_slice_report.json`
- `docs/validation/gate0/first_slice_report.md`

The generated report's `base_commit_if_known` field is the embedded canonical replay-fixture
provenance, not the Gate branch SHA; `tested_code_commit` in `gate0_evidence.json` identifies the
code that produced this evidence.

## Release benchmarks

Host: AMD Ryzen 7 5800X, Rust 1.98.0, `x86_64-pc-windows-msvc`. Physics budget is 2,000 µs at
500 Hz; both runs used 1,000 warmup and 10,000 measured steps.

| Model | p50 | p95 | p99 | max | Classification | Fingerprint |
|---|---:|---:|---:|---:|---|---|
| Acro Electric 01 | 7.3 µs | 9.1 µs | 9.3 µs | 121.2 µs | PASS | `dedc7981…b8ed` |
| SIG Kadet LT-40 EGV provisional | 119.5 µs | 146.0 µs | 181.5 µs | 304.8 µs | PASS | `328b969e…e0af` |

Exact machine-readable metrics and final snapshot hashes are in
`docs/validation/gate0/benchmark_results.json`.

## Viewer verification

The viewer was launched twice and closed normally with Escape. The ground pass also recorded and
verified a 15,962-step replay. wgpu selected an **NVIDIA GeForce RTX 3090**, driver **NVIDIA
595.97**, backend **Vulkan**. The Windows inventory reports driver `32.0.15.9597`, dated
2026-03-17.

Commands (arguments passed to the release binary):

```text
rcsim-app render --model models/acro_electric_ground_demo/model.json --start-on-ground --throttle 0 --scenery flying-field --camera pilot --pilot-position 0,1.8,20 --camera-fov 55 --debug-overlays --record-replay target/gate0_viewer/viewer_ground_replay.json
rcsim-app render --model models/acro_electric_01/model.json --altitude-m 45 --airspeed-mps 20 --throttle 0.55 --scenery none --camera chase --chase-distance-m 5 --chase-height-m 2 --camera-fov 55
```

| Item | Result | Observable evidence |
|---|---|---|
| Sky/haze/fog | PASS observed | clear procedural sky and horizon haze in both captures |
| Terrain | PASS observed | flat flying-field ground visible; rolling mode confirmed in runtime log |
| Scenery | PASS observed | runway/reference strip, tree line and field markers visible |
| GLB | PASS observed | finite low-poly aircraft visible in both views |
| Materials | PARTIAL | vertex colors, directional light and fog visible; checked-in GLB has no material/texture payload |
| Moving surfaces | NOT OBSERVABLE | ten renderer articulation tests pass, but checked-in models declare no `articulated_surfaces` |
| Pilot camera | PASS observed | fixed field-side capture |
| Chase camera | PASS observed | airborne follow capture |

Evidence:

- `docs/validation/gate0/viewer_ground_pilot_debug.png`
- `docs/validation/gate0/viewer_airborne_chase.png`
- `docs/validation/gate0/viewer_ground_stdout.log`
- `docs/validation/gate0/viewer_chase_stdout.log`
- `docs/validation/gate0/gpu_windows.txt`

The implemented presentation is classified precisely as a **functional presentation foundation**:
procedural sky/haze/fog, flat/rolling terrain, deterministic flying-field scenery, low-poly GLB,
vertex colors/basic lighting, pilot/chase cameras, and an articulation pipeline covered by tests.
It is not PBR/AAA and has no observed production material or moving-surface asset.

## Schema and residual risks

Schema v8 is the single documented authoring head. The engine exposes no model JSON writer; the
loader retains explicit, tested read compatibility for v0 through v8. Older checked-in model files
therefore remain intentional legacy inputs rather than competing write heads.

Residual risks:

- manual moving-surface and texture-rich material verification is blocked by current assets;
- controller/TX16S and real interactive pilot session remain NOT_TESTED;
- LT-40 remains provisional and lacks measured/holdout flight validation;
- all synthetic fixtures demonstrate contracts and determinism, not physical realism;
- GPU timing, shadows, PBR/HDR, high-quality antialiasing and frame-pacing evidence are absent.

No branch or worktree was deleted. Branches whose tips are ancestors of this baseline remain
available and are listed in the Gate handoff; `main` and `integration/m2-9c-on-main` remain available
as explicitly documented equivalent exclusions.

## One next action

Add or approve one checked-in multi-primitive GLB with explicit `articulated_surfaces` and a
texture-bearing material, then repeat the same viewer capture checklist to close the remaining
Manual presentation evidence gap.
