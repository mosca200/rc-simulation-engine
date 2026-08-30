# Foundation benchmark baseline

Criterion point estimates and confidence intervals below are a local baseline, not a performance contract.

- Date: 2026-08-30
- Git commit: unavailable (the workspace was initially empty and had no Git repository or commit)
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench (optimized Criterion profile; workspace release profile enables thin LTO and one codegen unit)
- Command: `cargo bench`
- B1 `evaluate_derivative`: 13.210–13.479 ns (estimate 13.326 ns)
- B2 one RK4 step: 97.732–98.187 ns (estimate 97.918 ns)
- B3 `Simulation::step()` at 250 Hz: 107.81–108.18 ns (estimate 107.98 ns)
- B3 `Simulation::step()` at 500 Hz: 107.69–108.49 ns (estimate 108.01 ns)
- B3 `Simulation::step()` at 1000 Hz: 108.12–109.93 ns (estimate 108.91 ns)
- B4 100,000-step headless loop plus one final state hash: 10.027–10.088 ms (estimate 10.054 ms), 9.9126–9.9734 million steps/s (estimate 9.9466 million steps/s)
- Estimated maximum single-core physics steps/s: approximately 9.26 million from the B3 500 Hz point estimate; the direct B4 loop measured 9.95 million steps/s
- 500 Hz budget utilization: approximately 0.00540% of the 2 ms step budget from the B3 point estimate
- Notes: Wall-clock measurements include normal local-system noise and Criterion reported outliers. Benchmarks are not CI pass/fail gates. B4 hashes only the final snapshot of each 100,000-step batch, outside the per-step workload.

## S3.1 anti-optimization audit comparison

Delta is `(after - before) / before`, using the Criterion point estimates from the original S1–S3 baseline and this S3.1 run.

| Benchmark | Before | After | Delta |
| --- | ---: | ---: | ---: |
| B1 derivative | 12.908 ns | 13.326 ns | +3.24% |
| B2 RK4 step | 97.256 ns | 97.918 ns | +0.68% |
| B3 step, 250 Hz | 106.74 ns | 107.98 ns | +1.16% |
| B3 step, 500 Hz | 106.69 ns | 108.01 ns | +1.24% |
| B3 step, 1000 Hz | 107.99 ns | 108.91 ns | +0.85% |
| B4 100,000-step batch | 10.307 ms | 10.054 ms | -2.45% |
| B4 throughput | 9.7018 Mstep/s | 9.9466 Mstep/s | +2.52% |
