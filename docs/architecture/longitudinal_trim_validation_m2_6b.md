# M2.6B — Deterministic Longitudinal Trim Sweep CLI & Reporting

## Scope

M2.6B adds an application-layer CLI and deterministic reporting primitive for the
M2.6A `solve_longitudinal_trim_sweep` API. The slice is offline validation
infrastructure: it runs in developer / CI shells, produces two structured
artifacts, and exits 0 on PASS or non-zero on FAIL.

M2.6B does NOT modify:
- M2.5 trim solver,
- M2.6A sweep primitive,
- any aircraft runtime physics,
- the 500 Hz hot loop,
- any M2.4, M2.3, M2.2 evidence path.

M2.6B does NOT calibrate any aircraft and introduces no real-aircraft evidence.

## CLI syntax

```
rcsim-app validate trim-sweep \
    --model PATH \
    [--speed-mps VALUE]... \
    --alpha-min-rad VALUE --alpha-max-rad VALUE \
    --elevator-min VALUE --elevator-max VALUE \
    --throttle-min VALUE --throttle-max VALUE \
    --initial-alpha-rad VALUE --initial-elevator VALUE --initial-throttle VALUE \
    --force-tolerance-n VALUE --moment-tolerance-nm VALUE \
    --max-iterations N \
    --output-dir PATH
```

`--speed-mps` may be supplied one or more times; the order is preserved exactly in
the report. The CLI fail-closes on missing required arguments, malformed or
non-finite numeric values, zero `--max-iterations`, invalid bounds/tolerances, and
unknown arguments. No default values are derived from the synthetic trim fixture
— every value is supplied by the caller.

## M2.5 relationship

M2.6B is a pure consumer of the M2.5 trim solver, accessed exclusively through
the M2.6A `solve_longitudinal_trim_sweep` API. The CLI never instantiates a
`LongitudinalTrimRequest` directly; the per-speed request is built by
`LongitudinalTrimSweepRequest::new` from the parsed options. The M2.5 trim
solver, the M2.5 residual / tolerance / iteration contract, and the M2.5
`LongitudinalTrimFailureReason` taxonomy are all unmodified.

## M2.6A relationship

M2.6B is a thin orchestrator over M2.6A. It:

- consumes `LongitudinalTrimSweepRequest::new` (fail-closed shared-template
  validation),
- consumes `solve_longitudinal_trim_sweep` (per-point bounded Newton solve
  + integrity re-evaluation),
- consumes the public M2.6A accessors on
  `ReEvaluationMismatchDetail` (`iteration_count`, `solver_evaluation`,
  `independent_evaluation`) and on `ReEvaluationUnverifiableDetail`
  (`iteration_count`, `solver_evaluation`) to build the integrity diagnostics
  in the report,
- does NOT add new Serialize derives on M2.6A domain types; the report is
  built in application-layer DTOs that mirror the M2.6A outcome enum via a
  serde `tag = "outcome"` discriminated union,
- does NOT modify M2.6A semantics; the M2.6A `LongitudinalTrimSweepOutcome`
  ordering, count accessors, and outcome predicates are the source of truth
  for the report counters.

## Deterministic environment

M2.6B hard-codes a deterministic validation environment identical to the
M2.5/M2.6A test harness:

- physics rate: `DEFAULT_PHYSICS_HZ` (500 Hz, dt = 0.002 s),
- air density: 1.225 kg/m³,
- wind velocity (world): `Vec3::zeros()`,
- gravity (NED): `(0.0, 0.0, 9.80665)` m/s².

No weather, no wind, no time-of-day, no stochastic inputs. The deterministic
environment is part of the report's `environment` block so downstream readers
can verify the physics configuration that produced the artifacts.

## JSON schema (`trim_sweep.json`)

`schema_version` is `1`. The JSON is `serde_json::to_string_pretty` and
`deny_unknown_fields` on every DTO, so adding a top-level field is a deliberate
breaking change.

Top-level keys:

- `schema_version: u32` — currently `1`.
- `generated_by: string` — `"rcsim-app validate trim-sweep"`.
- `model: { model_id, model_physics_fingerprint }` — the loaded model identity
  and 64-character hex physics fingerprint.
- `environment: { physics_hz, air_density_kg_m3, wind_velocity_world_mps[3], gravity_world_mps2[3] }`.
- `request: { target_speeds_mps, alpha_bounds_rad, elevator_bounds, throttle_bounds, initial_guess, tolerances, maximum_iterations }`.
- `summary: { total_points, success_count, trim_failure_count, re_evaluation_mismatch_count, re_evaluation_unverifiable_count, overall_status }` — `overall_status` is `"PASS"` or `"FAIL"`.
- `points: [...]` — one entry per requested speed, in input order.

`points[i]` is a discriminated union with `tag = "outcome"`:

| `outcome` value                  | fields                                                                                                                                            |
|---------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| `"success"`                     | `target_airspeed_mps, iteration_count, alpha_rad, elevator_command, throttle, longitudinal_force_residual_n, vertical_force_residual_n, pitch_moment_residual_nm` |
| `"trim_failure"`                | `target_airspeed_mps, failure_reason, iteration_count, last_evaluation?`                                                                          |
| `"re_evaluation_mismatch"`      | `target_airspeed_mps, iteration_count, solver_variables, solver_residuals, independent_variables, independent_residuals`                  |
| `"re_evaluation_unverifiable"`  | `target_airspeed_mps, iteration_count, solver_variables, solver_residuals` (no independent payload)                                          |

`failure_reason` is one of `"NO_FEASIBLE_SOLUTION"`, `"SINGULAR_JACOBIAN"`,
`"NON_FINITE_EVALUATION"`, `"ITERATION_LIMIT"`. The unverifiable variant has
no `independent_*` field — absence is the truthful representation.

## Markdown report (`trim_sweep.md`)

Sections in order:

1. Title + `generated_by` + `schema_version`.
2. **Model** — `model_id`, `model_physics_fingerprint`.
3. **Configuration** — physics rate, density, wind vector, gravity vector, alpha
   / elevator / throttle bounds, initial guess, tolerances, max iterations.
4. **Summary** — total, success, trim failure, mismatch, unverifiable, overall
   status.
5. **Ordered points** — a markdown table of the form
   `| speed_mps | outcome | iterations | alpha_rad | elevator | throttle | Fx_N | Fz_N | My_Nm |`,
   one row per requested speed in input order. Non-success rows leave the
   variable columns blank.
6. Additional diagnostic paragraphs for every non-success point: failure
   reason and last finite residuals for `trim_failure`; solver / independent
   variables and residuals for `re_evaluation_mismatch`; solver variables and
   residuals for `re_evaluation_unverifiable` (no fabricated independent
   payload).

The Markdown table is the only place where the speed list is restated;
downstream reporters should rely on `points[i].target_airspeed_mps` from the
JSON for authoritative per-point metadata.

## PASS/FAIL rules

- **PASS** if and only if every `points[i]` has `outcome = "success"`. The
  `summary.overall_status` is `"PASS"` and the CLI exits `0`.
- **FAIL** if any `points[i]` has `outcome = "trim_failure"`,
  `"re_evaluation_mismatch"`, or `"re_evaluation_unverifiable"`. A
  `TrimFailure` is a VALID structured physical result — it is NOT an
  application crash. The `summary.overall_status` is `"FAIL"` and the CLI
  exits `2`.

## Exit semantics

- `0` — every point is `Success` (PASS).
- `2` — at least one point is non-Success; the two reports are written
  COMPLETELY before this exit code is produced. This is the dedicated
  validation-failure exit code.
- `1` — ordinary CLI / model / filesystem / sweep-request error. Either the
  two reports were not produced (e.g. parse failure, model load failure) or
  the sweep could not be constructed; the report is not guaranteed to be
  present.
- `--help` / `-h` — prints usage and exits `0`.

## Deterministic artifact guarantee

The JSON and Markdown reports MUST be byte-identical across runs that share:

- the bytes of the loaded aircraft model file,
- the CLI arguments (including the order of `--speed-mps`),
- the validation environment (which M2.6B hard-codes),
- the M2.6A sweep request, the M2.5 trim solver, and the report builder.

There is no timestamp, wall clock time, date, random identifier, or
nondeterministic ordering. A source-level guard test
(`report_source_uses_no_runtime_nondeterminism`) asserts the production code
path does not reference `SystemTime::now`, `Instant::now`, `Utc::now`,
`std::process::id`, `rand::thread_rng`, or `rand::random`. The JSON and
Markdown outputs are also scanned by the
`reports_contain_no_timestamp_or_wall_clock_fields` test, which asserts the
set of forbidden output tokens is absent.

## Architecture boundaries

```
rcsim-app (application layer)
└── trim_sweep_validation_app (CLI + report DTOs)
    ├── CLI parsing (TrimSweepValidationOptions::parse)
    ├── Runner (run_trim_sweep_validation)
    │   ├── load aircraft model (model crate)
    │   ├── build the M2.6A sweep request (aircraft crate, fail-closed)
    │   ├── solve the M2.6A sweep (aircraft crate)
    │   ├── build the application DTO TrimSweepReport
    │   │   ├── M2.6A count accessors (success/trim_failure/mismatch/unverifiable)
    │   │   └── M2.6A integrity detail accessors
    │   │       (ReEvaluationMismatchDetail::{iteration_count, solver_evaluation, independent_evaluation})
    │   │       (ReEvaluationUnverifiableDetail::{iteration_count, solver_evaluation})
    │   └── write trim_sweep.json + trim_sweep.md
    └── Dispatch (main.rs `validate trim-sweep` arm)
        └── maps TrimSweepValidationError::ValidationFailure to exit code 2
```

- The M2.6A public API is consumed only through its re-exports in
  `aircraft::{...}`. The application layer never reaches into M2.6A
  private fields.
- No new dependencies are added; the JSON serializer is `serde_json` and
  the deterministic environment reuses `sim_core::DEFAULT_PHYSICS_HZ` and
  `sim_core::DEFAULT_GRAVITY_MPS2`.
- M2.6B does not depend on `wgpu`, `replay`, `telemetry`, or any 6-DoF
  runtime crate.

## Limitations

- M2.6B is offline validation only. It is not a runtime guard and is not
  on the 500 Hz hot path.
- M2.6B is driven by the M2.6A sweep primitive. The slice cannot detect
  integrity regressions that the underlying M2.5 / M2.6A primitives already
  cover; it can only faithfully report them.
- The determinism guarantee assumes a stable Rust toolchain. Bit-exact
  byte equality across toolchain versions is not asserted.
- M2.6B does NOT calibrate, fit, or modify any aircraft model. It only
  evaluates the model that the caller provides. A model is never promoted
  to `ReferenceAircraft` by running the sweep.
- The PASS/FAIL verdict is reported separately from any per-point
  actionable remediation; downstream reporting layers are responsible for
  mapping `TrimFailure` reasons to user-facing guidance.

## Failure-closed on the synthetic trim fixture

The synthetic trim fixture
(`tests/fixtures/synthetic_non_reference_trim_v4.json`) used in the M2.5/M2.6A
test suites is also the only model the M2.6B test suite exercises. The
synthetic fixture is asserted to remain `AircraftClassification::SyntheticTest`
with `reference_aircraft == None` after the M2.6B pipeline. This is the same
fixture-level invariant the M2.5 and M2.6A tests assert; the constraint is
re-asserted in M2.6B's
`synthetic_fixture_remains_synthetic_test_and_is_not_promoted` test to make
the invariant explicit at the M2.6B boundary.

## M2.6B.1 report-contract hardening

M2.6B.1 hardens the existing application/report boundary without changing
flight physics, trim mathematics, solver configuration, or the report schema
version. It is not M2.6C domain qualification.

The process exit-code contract is covered by direct subprocess integration
tests against the compiled `rcsim-app` binary:

- exit `0`: validation completed and every trim-sweep point is `Success`;
- exit `1`: CLI, input, model, filesystem, or other operational failure;
- exit `2`: validation completed with at least one non-`Success` point.

When validation completes with exit `2`, the reports are still emitted before
the process exits, provided report construction and writing succeeded. The only
canonical artifacts are exactly:

- `trim_sweep.json`
- `trim_sweep.md`

Report decoding fails closed unless `schema_version` is exactly
`TRIM_SWEEP_REPORT_SCHEMA_VERSION` (currently `1`). Malformed JSON, unknown
fields, invalid enum values, and numerically unsupported schema versions are
rejected.

Identical model bytes, CLI inputs, and deterministic validation environment
produce byte-identical JSON and Markdown artifacts. Subprocess tests compare
the raw bytes produced in separate output directories; output paths and other
run-specific identifiers are not serialized.

Both report payloads are fully rendered and the generated JSON is decoded
through the strict current-schema path before either canonical file is written.
The two independent filesystem writes are not pair-atomic: an I/O failure on
the second write can leave the first file present. Cross-platform atomic
replacement of two files is outside the M2.6B.1 scope.
