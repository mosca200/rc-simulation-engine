# S4 aerodynamic benchmark baseline

- Date: 2026-08-31
- Git base commit: `579100e` (`Harden RK4 numerical boundaries`); S4 changes uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench / optimized Criterion profile
- Command: `cargo bench`
- B5 polar lookup: 3.5135–3.5339 ns (point estimate 3.5230 ns), approximately 283.85 million lookups/s
- B6 single aerodynamic-element evaluation: 52.795–52.969 ns (point estimate 52.874 ns), approximately 18.91 million evaluations/s
- B7 RK4 step with four aerodynamic-element evaluations: 328.38–330.92 ns (point estimate 329.38 ns), approximately 3.04 million RK4 steps/s
- Notes: Informational local baseline; not a CI performance gate.
