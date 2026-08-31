# First Vertical Slice

The First Vertical Slice is the smallest end-to-end RC aircraft simulation path in this repository.
It combines the deterministic 500 Hz `f64` flight core, the Acro Electric model, local aerodynamic
elements, electric propulsion, controls and servos, model-bound replay, telemetry, input
abstraction, deterministic characterization, performance evidence, and the presentation-only GLB
viewer. The P3 acceptance harness evaluates these parts without opening a window, polling physical
input hardware, or initializing a GPU.

The slice is an engineering integration milestone. A passing Technical Gate does not claim that the
flight model is calibrated against a real aircraft, that its visual result has been observed, or
that the product is ready for flight-training use.

## Build and smoke commands

Build the complete workspace in release mode:

```powershell
cargo build --workspace --release
```

Run the deterministic foundation and aircraft smokes:

```powershell
cargo run -p rcsim-app --release -- --steps 10
cargo run -p rcsim-app --release -- aircraft --steps 10
```

Verify the canonical 2,000-step replay:

```powershell
cargo run -p rcsim-app --release -- replay verify --model models/acro_electric_01/model.json --input tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json
```

List available input devices without starting the renderer:

```powershell
cargo run -p rcsim-app --release -- input list
```

Generate and analyze 2,000 telemetry frames:

```powershell
cargo run -p rcsim-app --release -- telemetry run --model models/acro_electric_01/model.json --output target/first_slice_telemetry.jsonl --steps 2000
cargo run -p rcsim-app --release -- telemetry analyze --input target/first_slice_telemetry.jsonl
```

Run the S10 deterministic characterization suite:

```powershell
cargo run -p rcsim-app --release -- validate acro-electric-01 --output-dir target/first_slice_s10
```

Run the P2 release benchmark. Its timing diagnostics do not participate in deterministic hashes:

```powershell
cargo run -p rcsim-app --release -- benchmark aircraft --model models/acro_electric_01/model.json --warmup-steps 1000 --steps 10000
```

## P3 acceptance harness

Run the unified, headless acceptance command:

```powershell
cargo run -p rcsim-app --release -- validate first-slice --output-dir target/first_slice_gate
```

The harness calls existing Rust APIs directly. It does not spawn Cargo or duplicate the physics,
replay, telemetry, benchmark, GLB-loader, or S10 implementations. It creates:

- `target/first_slice_gate/report.json`, the strict, versioned machine-readable report;
- `target/first_slice_gate/report.md`, generated from the same structured report.

Criterion statuses are `PASS`, `PARTIAL`, `NOT_TESTED`, and `FAIL`. Any technical failure makes the
overall result `FAIL`. Otherwise, incomplete manual or real-world evidence keeps the overall result
`PARTIAL`; it is not silently promoted to `PASS`.

## Renderer launch and manual verification

The graphical viewer is deliberately outside the headless acceptance path:

```powershell
cargo run -p rcsim-app --release -- render --model models/acro_electric_01/model.json
```

Manual verification still requires a real observation of the aircraft GLB, ground, sky, camera,
pose interpolation, and a basic user flight session. A physical controller must be exercised
separately, including the Radiomaster TX16S if that device is part of the acceptance target. Merely
building the viewer or reporting zero connected devices is not manual evidence.

## Definition of Done

### Technical Gate

The Technical Gate is `PASS` when every automatically verifiable criterion passes: workspace and
stable-Rust contracts, `f64` RK4 dynamics at 500 Hz, model/controls/propulsion assembly, canonical
replay and dataset, telemetry, GLB CPU parsing, simulation/render separation, interpolation,
frame-rate independence, input semantics, live-input recording, performance, allocation evidence,
S10 characterization, and headless execution.

Technical `PASS` means the automated slice is coherent and repeatable. It does not mean the product
is validated, the flight model is calibrated, or the visual result is realistic.

### Manual Gate

The Manual Gate requires an observed viewer check, a real controller test, a Radiomaster TX16S test
when applicable, and a basic interactive flight session. Until those checks are performed and their
evidence is persisted, the applicable criteria remain `PARTIAL` or `NOT_TESTED`.

### Real-world Gate

The Real-world Gate requires execution of the structured pilot-review protocol and comparison with
measured aircraft geometry and inertia, propulsion bench data, and real flight telemetry. The
current protocol is prepared, but pilot review and calibration evidence have not yet been produced;
the gate must therefore remain incomplete.
