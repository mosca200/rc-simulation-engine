# S5A controls benchmark baseline

- Date: 2026-08-31
- Git base commit: `bd41c6c` (`Implement S4 aerodynamic element`); S5A changes were uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench / optimized Criterion profile
- Command: `cargo bench`

| Benchmark | 95% confidence interval | Point estimate | Approximate throughput |
| --- | ---: | ---: | ---: |
| B8 rates/expo, three axes | 4.9748–4.9888 ns | 4.9814 ns | 200.75 million evaluations/s |
| B9 one rate-limited servo update | 3.5918–3.6224 ns | 3.6077 ns | 277.18 million updates/s |
| B10 complete controls pipeline | 13.639–13.833 ns | 13.731 ns | 72.83 million pipeline steps/s |

Inputs and outputs are consumed with `std::hint::black_box`. B9 and B10 use Criterion batched
state initialization so every timed invocation performs a real rate-limited state transition and
the setup is excluded from the timed routine. These measurements are informational local
baselines, not CI performance gates.
