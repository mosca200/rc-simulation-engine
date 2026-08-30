# S5B propulsion benchmark baseline

- Date: 2026-08-31
- Git base commit: `ae95b36` (`Implement S5A control pipeline`); S5B changes were uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench / optimized Criterion profile
- Verification command: `cargo bench`
- Stable isolated confirmation: `cargo bench --bench propulsion_s5b`
- Solver iterations: exactly 48 bisection iterations per complete propulsion evaluation

| Benchmark | 95% confidence interval | Point estimate | Approximate throughput |
| --- | ---: | ---: | ---: |
| B11 propeller coefficient lookup | 7.5263–7.9157 ns | 7.7297 ns | 129.37 million evaluations/s |
| B12 known-speed electrical evaluation | 7.2852–7.7464 ns | 7.5329 ns | 132.75 million evaluations/s |
| B13 complete electric propulsion operating point | 927.75–931.00 ns | 929.50 ns | 1.0758 million evaluations/s |
| B14 RK4 step with stage-correct propulsion | 3.8211–3.8441 us | 3.8330 us | 260.89 thousand RK4 steps/s |

All configurations, the coefficient-table `Vec`, environment, and rigid-body fixtures are created
outside timed loops. Inputs and final outputs are consumed with `std::hint::black_box`. B11 uses an
interior, non-tabulated advance ratio. B12 remains in the positive-current regime. B13 includes all
48 fixed bisection iterations. B14 evaluates propulsion inside all four RK4 stages, for 192 total
bisection iterations per measured step, and consumes the final rigid-body state.

The required full `cargo bench` run completed successfully while the host experienced a transient,
suite-wide slowdown also affecting unchanged B1–B10. The table therefore records the immediately
repeated isolated S5B run. Isolated checks of unchanged B5 and B10 returned near their established
baselines, confirming that the full-run slowdown was environmental rather than an S5B regression.
These measurements are informational local baselines, not CI performance gates.
