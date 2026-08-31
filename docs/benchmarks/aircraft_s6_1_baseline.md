# S6.1 aircraft-assembly benchmark baseline

- Date: 2026-08-31
- Git base commit: `b427f20ab8b60b1af6da642195be8151c36019c6` (`Implement S5B electric propulsion`); S6 and S6.1 changes were uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench / optimized Criterion profile
- Verification command: `cargo bench`
- Aircraft fixture: Acro Electric 01 schema v1, 8 aerodynamic elements, 4 controlled elements, electric propulsion present

| Benchmark | 95% confidence interval | Point estimate | Approximate throughput | 2 ms budget |
| --- | ---: | ---: | ---: | ---: |
| B17 aggregate Acro aerodynamic wrench | 391.52-398.12 ns | 393.92 ns | 2.538 million evaluations/s | 0.01970% |
| B18 complete `AircraftSimulation::step` | 5.5385-5.6641 us | 5.5951 us | 178.73 thousand steps/s | 0.27976% |

B17 evaluates and accumulates all eight real S4 aerodynamic elements at neutral surface positions.
The stage state, effective elements, immutable model, environment, and returned `BodyWrench` are
consumed through `std::hint::black_box`.

B18 includes one control-system update, four in-place control-surface updates, all eight aerodynamic
elements, electric propulsion, and all four stage-correct RK4 evaluations. Criterion
`iter_batched_ref` creates a fresh simulation outside each timed batch and also drops it outside the
timed routine, so heap-owning setup and teardown are not attributed to `step`. The pilot input and
returned snapshot are consumed through `std::hint::black_box`.

B19 was optional and is not included in S6.1. These measurements are informational local baselines,
not CI performance gates. The percentage is `point_estimate / 2 ms * 100`.
