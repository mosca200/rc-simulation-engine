# S6 aircraft-model benchmark baseline

- Date: 2026-08-31
- Git base commit: `b427f20ab8b60b1af6da642195be8151c36019c6` (`Implement S5B electric propulsion`); S6 changes were uncommitted during measurement
- rustc: `rustc 1.98.0 (88d9e12ae 2026-08-18)`, LLVM 22.1.8
- Target: `x86_64-pc-windows-msvc`
- CPU: AMD Ryzen 7 5800X 8-Core Processor
- OS: Microsoft Windows NT 10.0.26200.0
- Build profile: Cargo bench / optimized Criterion profile
- Verification command: `cargo bench`

| Benchmark | 95% confidence interval | Point estimate | Approximate throughput |
| --- | ---: | ---: | ---: |
| B15 parse + validate Acro Electric 01 | 9.3935-9.4671 us | 9.4280 us | 106.07 thousand loads/s |
| B16 model semantic physics fingerprint | 1.8828-1.8964 us | 1.8893 us | 529.30 thousand hashes/s |

B15 consumes the embedded repository `models/acro_electric_01/model.json`, performs strict JSON
parsing, schema-version selection, validation through the existing S4/S5A/S5B constructors, and
reference resolution on every iteration. Both the input and returned immutable model are consumed
with `std::hint::black_box`.

B16 constructs and validates the model outside the timed loop. Each iteration hashes the complete
validated physics semantics using canonical little-endian encodings, and consumes both the model
reference and resulting fingerprint with `std::hint::black_box`. It performs no JSON parsing or
filesystem access.

An earlier local measurement taken before the final strict duplicate-key parsing hardening was
25.409 us for B15 and 1.9472 us for B16. The final points are respectively 62.90% and 2.97% lower;
the B15 improvement follows removal of the intermediate `serde_json::Value` tree and its clone.
These measurements are informational local baselines, not CI performance gates.
