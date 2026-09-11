# RV2-6 physical aerial perspective closure

Status: technically validated; manual visual and measured performance gates remain open.

> **Controlled-validation update:** the FlyingField/V1 matrix recorded below
> is superseded because it mixes vegetation distance culling and the RV2-5
> renderer boundary into the comparison. Use
> [`rv2_6_controlled_visual.md`](rv2_6_controlled_visual.md) and its V2 AP
> ON/OFF evidence instead. The historical commands remain only as provenance.

## Git provenance

- RV2-6 branch: `feature/rv2-6-aerial-perspective`
- RV2-6 origin before convergence: `9034b4d7a1ffd1e9b563ebe67f5fbfbc3cecff45`
- incorporated `integration/render-v2`: `ec3e17fb3205c7eb337ffa6c4c3446940a90a681`
- common merge base: `3358cee5faaac6fb544e02367fb130088194f325`
- convergence merge: `51ba2e34e00b7a5551f69d970f02fb9a191c88da`
- convergence parents, in order: `9034b4d7a1ffd1e9b563ebe67f5fbfbc3cecff45`,
  `ec3e17fb3205c7eb337ffa6c4c3446940a90a681`

The merge completed with the `ort` strategy and no conflicts. Relative to the incorporated
integration branch, the runtime delta remains the five original RV2-6 files:

- `crates/renderer/src/gpu.rs`
- `crates/renderer/src/renderer_v2/aerial_perspective.rs`
- `crates/renderer/src/renderer_v2/mod.rs`
- `crates/renderer/src/shader.wgsl`
- `crates/renderer/tests/rv2_6_physical_aerial_perspective.rs`

RV2-7A contributes authoring tools, documentation, and the non-runtime `.blend` source. No merge
resolution changed RV2-6 behavior.

## Technical validation

Executed on Windows with an NVIDIA GeForce RTX 3090, driver 595.97, 24,576 MiB reported memory.

| Command | Result |
| --- | --- |
| `cargo fmt --all -- --check` | PASS |
| `cargo check --workspace --all-targets --all-features` | PASS |
| `cargo clippy --workspace --all-targets --all-features -- -D warnings` | PASS |
| `cargo test --workspace --all-features` | PASS |
| `cargo test -p renderer --test rv2_6_physical_aerial_perspective --all-features` | PASS, 5/5 |
| `cargo test -p model --test fingerprint --all-features` | PASS, 14/14 |
| `cargo build --workspace --release --all-features` | PASS |
| RV2-7A Python contract tests | PASS, 31/31 |
| physical-environment GPU generation/readback, isolated | PASS |
| transfer-LUT irradiance GPU test, isolated | PASS |
| physical environment cube-face routing GPU test, isolated | PASS |
| HDR-to-surface offscreen GPU test, isolated | PASS |

Running all three ignored atmosphere GPU tests concurrently caused Windows process termination with
`STATUS_ACCESS_VIOLATION` after 60 seconds. Each test passes when run in isolation on the same RTX
3090. The failure therefore remains recorded as a test-harness/driver concurrency finding; no test
was removed, ignored further, weakened, or changed.

The Acro Electric 01 production physics fingerprint remains:

`07c48378ad0f8de786f0927c1bba206681c4153deb50174bf0c518d6eae5ba73`

The full workspace replay, deterministic run, allocation, simulation, model-fingerprint, renderer
dependency-boundary, and RV2-7A presentation-contract tests passed. The renderer remains downstream
of `AircraftRenderSnapshot`; no renderer dependency or value feeds back into simulation.

## Runtime smoke

Executed for approximately one minute:

```powershell
cargo run -p rcsim-app --release -- play --renderer v2 --scenery flying-field --camera chase --exposure-ev 0.0
```

Initialization completed, the 500 Hz ground-start simulation stayed alive without runtime or wgpu
errors, and `nvidia-smi pmon -c 1` identified `rcsim-app.exe` as a live RTX 3090 C+G workload. This is
a runtime smoke only, not a visual-quality or performance measurement.

## Reproducible manual visual gate

Run each command from the repository root. Keep the window size and exposure unchanged, wait at
least five seconds after initialization, and capture the complete simulator window. The aircraft is
held on the supported ground start at zero throttle, making camera-to-aircraft distance repeatable.

### A. Near field

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera chase --chase-distance-m 3.5 --chase-height-m 1.25
```

Check that the aircraft, runway, and nearby vegetation are not washed out and that contact contrast
is preserved.

### B. Approximately 100 m

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position 0,1.8,100 --camera-fov 55
```

Check modest, continuous contrast/saturation loss without destroying aircraft readability.

### C. Approximately 500 m

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position 0,1.8,500 --camera-fov 55
```

Check that extinction and in-scattering are stronger than at 100 m, finite, and free of a linear-fog
edge.

### D. Long distance and horizon

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position 0,20,1000 --camera-fov 55
```

Check the terrain/sky transition for depth, clipping, banding, and implausible colour casts.

The following three 100 m views rotate the camera around the static aircraft relative to the fixed
production sun azimuth. They do not alter atmosphere or lighting state.

### E. Front-lit azimuth

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position 80,1.8,-60 --camera-fov 55
```

### F. Lateral-sun azimuth

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position 60,1.8,80 --camera-fov 55
```

### G. Back-lit/controluce azimuth

```powershell
cargo run -p rcsim-app --release -- render --renderer v2 --start-on-ground --throttle 0 --scenery flying-field --exposure-ev 0.0 --camera pilot --pilot-position -80,1.8,60 --camera-fov 55
```

For E-G, check that forward/lateral/back-lit responses remain continuous, finite, naturally
coloured, and do not erase the aircraft silhouette. For an A/B control, repeat B-D with
`--renderer v1`; change no other argument.

## Evidence and open findings

- The original FlyingField A-G captures are not accepted evidence for RV2-6 isolation. The
  controlled V2 AP ON/OFF captures under `docs/validation/rv2_6_controlled_visual/` replace them.
  Their measured separation is recorded in that document: the seam is isolated and directionally
  correct, but the difference is below the 8-bit quantization floor of the capture, so these are
  isolation captures rather than visual-quality evidence.
- Manual review of the controlled captures remains required. Compilation, GPU smoke, static shader
  tests, and successful capture are not a visual PASS.
- The V2 profiler owns persistent CPU timings and optional GPU timestamp queries, but its latest
  snapshot is not exposed through the CLI, overlay, or an evidence file. PresentMon/CapFrameX was
  not available. **GPU COST: NOT YET INSTRUMENTED FOR REPORTING.** No FPS or frame-time metric is
  inferred from utilization or visual observation.
- The parallel ignored-atmosphere-test access violation described above remains open; isolated GPU
  coverage passes.

## Gate status

- CODE: PASS
- DETERMINISM: PASS
- VISUAL: OPEN
- PERFORMANCE: OPEN
