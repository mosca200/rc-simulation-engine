# Aircraft real-time baseline P2

## Run context

- Date: 2026-08-31 (Europe/Rome)
- Operating environment: Windows, local Codex workspace
- Build mode: Cargo `release` (`lto = "thin"`, one codegen unit)
- Model: `models/acro_electric_01/model.json` (`acro-electric-01`)
- Physics fingerprint: `dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed`
- Physics rate: 500 Hz
- Physics timestep: 0.002 s
- Per-step budget: 2,000 µs
- Deterministic input: roll/pitch/yaw `0.0`, throttle `0.55`
- Warmup: 5,000 steps, excluded from all timing statistics
- Measurement: 50,000 individual `AircraftSimulation::step()` samples per run

CPU identification was not available through the permitted system-information interface, so no CPU
model is reported. GPU information is irrelevant because this command initializes no renderer.
These measurements are hardware/OS specific and are not a cross-machine performance guarantee.

## Measurement method

The release CLI loads and constructs the aircraft before timing, executes the warmup, then surrounds
each measured physics step with `std::time::Instant`. The timing vector is preallocated. Model I/O,
hardware input, rendering, telemetry serialization, replay hashing, sorting, percentile calculation,
and output formatting are outside each timing sample.

Percentiles use nearest rank. For percentile `p` and `N` sorted samples, the selected one-based rank
is `ceil(p * N)`, clamped to `1..=N`. No samples or outliers are removed; `max` is the actual largest
sample.

## Results

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| measured steps | 50,000 | 50,000 |
| mean | 5.674884 µs | 5.583146 µs |
| p50 | 5.600000 µs | 5.600000 µs |
| p95 | 5.900000 µs | 5.700000 µs |
| p99 | 7.100000 µs | 6.200000 µs |
| max | 27.600000 µs | 24.900000 µs |
| steps/s | 176,215.056 | 179,110.487 |
| mean budget utilization | 0.283744% | 0.279157% |
| p99 budget utilization | 0.355000% | 0.310000% |
| max budget utilization | 1.380000% | 1.245000% |
| classification | PASS | PASS |
| final snapshot hash | `cee385056fa643a05be63e7b6dde812e7cfeffa5c863cf06732451aaef9a7772` | `cee385056fa643a05be63e7b6dde812e7cfeffa5c863cf06732451aaef9a7772` |

## Acceptance classification

- **PASS:** p99 is below the 2,000 µs physics budget.
- **MARGINAL:** mean is below budget, but p99 is at or above budget.
- **FAIL:** mean is at or above budget.

Both runs are PASS with substantial margin. The real maxima are retained, but isolated scheduler
spikes do not override a p99 that remains well inside budget. No P2 optimization was necessary or
applied.
