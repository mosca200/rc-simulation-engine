# Foundation benchmark baseline

Criterion point estimates and confidence intervals below are a local baseline, not a performance contract.

- Date: 2026-08-31
- Git base commit: `577e8df` (`Foundation S1-S3`); S3.2 changes were uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench (optimized Criterion profile; workspace release profile enables thin LTO and one codegen unit)
- Command: `cargo bench`
- B1 `evaluate_derivative`: 13.033–13.099 ns (estimate 13.061 ns)
- B2 one constant-wrench RK4 step: 99.723–100.26 ns (estimate 99.958 ns)
- B3 `Simulation::step()` at 250 Hz: 110.29–110.67 ns (estimate 110.46 ns)
- B3 `Simulation::step()` at 500 Hz: 110.16–110.36 ns (estimate 110.25 ns)
- B3 `Simulation::step()` at 1000 Hz: 110.19–110.45 ns (estimate 110.31 ns)
- B4 100,000-step headless loop plus one final state hash: 10.324–10.347 ms (estimate 10.337 ms), 9.6645–9.6859 million steps/s (estimate 9.6744 million steps/s)
- Estimated maximum single-core physics steps/s: approximately 9.07 million from the B3 500 Hz point estimate; the direct B4 loop measured 9.67 million steps/s
- 500 Hz budget utilization: approximately 0.00551% of the 2 ms step budget from the B3 point estimate
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

## S3.2 stage-evaluator comparison

Delta compares the S3.2 point estimates against the S3.1 anti-optimization baseline immediately above.

| Benchmark | S3.1 | S3.2 | Delta |
| --- | ---: | ---: | ---: |
| B1 derivative | 13.326 ns | 13.061 ns | -1.99% |
| B2 RK4 step | 97.918 ns | 99.958 ns | +2.08% |
| B3 step, 250 Hz | 107.98 ns | 110.46 ns | +2.30% |
| B3 step, 500 Hz | 108.01 ns | 110.25 ns | +2.07% |
| B3 step, 1000 Hz | 108.91 ns | 110.31 ns | +1.29% |
| B4 100,000-step batch | 10.054 ms | 10.337 ms | +2.81% |
| B4 throughput | 9.9466 Mstep/s | 9.6744 Mstep/s | -2.74% |
