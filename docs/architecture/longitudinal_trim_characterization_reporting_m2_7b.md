# M2.7B — Longitudinal Trim Characterization Reporting

## Scope

M2.7B exposes the existing M2.7 longitudinal trim characterization API through
a deterministic, headless analysis command. It is an application-layer
orchestrator and report formatter. It does not change flight physics, trim
mathematics, solver configuration, or characterization mathematics.

The command uses only the existing public aircraft APIs:

1. `LongitudinalTrimSweepRequest::new`
2. `solve_longitudinal_trim_sweep`
3. `LongitudinalTrimCharacterizationSteps::new`
4. `characterize_longitudinal_trim_sweep`

No central difference is recalculated in the app.

## Command

```text
rcsim-app analyze trim-characterization \
  --model PATH \
  --speed-mps VALUE [--speed-mps VALUE]... \
  --alpha-min-rad VALUE --alpha-max-rad VALUE \
  --elevator-min VALUE --elevator-max VALUE \
  --throttle-min VALUE --throttle-max VALUE \
  --initial-alpha-rad VALUE \
  --initial-elevator VALUE \
  --initial-throttle VALUE \
  --force-tolerance-n VALUE \
  --moment-tolerance-nm VALUE \
  --max-iterations N \
  --alpha-step-rad VALUE \
  --elevator-step VALUE \
  --output-dir PATH
```

Both characterization steps are required and must be finite and strictly
positive. There are no hidden finite-difference defaults. Repeated
`--speed-mps` arguments are preserved in exact input order.

## Deterministic environment

The command uses the same fixed environment as the trim-sweep validation tool:

- physics rate: `DEFAULT_PHYSICS_HZ` (500 Hz);
- air density: `1.225 kg/m^3`;
- wind velocity in world coordinates: `(0, 0, 0) m/s`;
- gravity in NED coordinates: `(0, 0, 9.80665) m/s^2`.

The environment is serialized in the report. No timestamp, wall clock,
process ID, random identifier, UUID, or output path is serialized.

## Canonical artifacts

The command writes exactly:

- `trim_characterization.json`
- `trim_characterization.md`

It does not create `report.json`, `report.md`, `characterization.json`, or
`results.json`.

Both payloads are completely rendered in memory, and the JSON is decoded
through the strict current-schema path, before either canonical write begins.
The two independent writes are not pair-atomic: an I/O failure during the
second write can leave the first file present. Cross-platform transactional
replacement of two files is outside this slice.

## JSON report

`TRIM_CHARACTERIZATION_REPORT_SCHEMA_VERSION` is `1`. Decoding accepts only
that exact numeric version. Versions `0`, `2`, `999`, and all other unsupported
values fail closed. Malformed JSON, unknown DTO fields, and invalid enum values
also fail closed.

The root contains:

- `schema_version`
- `generated_by`
- `model` with model ID and physics fingerprint
- `environment`
- `trim_request` with ordered speeds, bounds, initial guess, tolerances, and
  maximum iterations
- `characterization_steps`
- `summary`
- ordered `points`

The summary records:

- `total_points`
- `characterized_count`
- `trim_failure_not_characterized_count`
- `re_evaluation_mismatch_not_characterized_count`
- `re_evaluation_unverifiable_not_characterized_count`
- `characterization_unavailable_count`

There is exactly one point for every requested speed, in request order.

## Point outcomes

The report preserves the M2.7 domain outcomes without inventing derivative
values:

- `characterized`
- `not_characterized_trim_failure`
- `not_characterized_re_evaluation_mismatch`
- `not_characterized_re_evaluation_unverifiable`
- `characterization_unavailable`

A characterized point copies these domain values directly:

- target airspeed, trim alpha, elevator command, and throttle;
- alpha and elevator finite-difference steps;
- pitch moment at trim;
- pitch moments at alpha-minus and alpha-plus;
- pitch moments at elevator-minus and elevator-plus;
- pitch stiffness;
- elevator effectiveness.

Unavailable reasons remain typed and retain their structured fields:

- `alpha_perturbation_out_of_bounds` with both perturbations and both bounds;
- `elevator_perturbation_out_of_bounds` with both perturbations and both
  bounds;
- `alpha_perturbation_non_finite` with explicit `minus` or `plus` side;
- `elevator_perturbation_non_finite` with explicit `minus` or `plus` side;
- `non_finite_pitch_stiffness`;
- `non_finite_elevator_effectiveness`.

Unavailable and non-characterized points contain no fabricated zero or null
derivatives.

## Derivative meaning

The two reported derivatives are:

```text
pitch_stiffness_nm_per_rad = dMy/dAlpha
elevator_effectiveness_nm_per_command = dMy/dElevatorCommand
```

They are local dimensional central-difference results around a verified trim
point. Their units are `N*m/rad` and `N*m` per normalized elevator command,
respectively.

They are not `Cma`, `Cm_delta_e`, static margin, neutral point, aerodynamic
center, complete longitudinal stability derivatives, or flight validation.
M2.7B does not infer a stability classification.

The alpha perturbations freeze elevator and throttle at the trim values. The
elevator perturbations freeze alpha and throttle at the trim values. The report
relies on the existing, directly tested M2.7 domain contract for this frozen-
variable behavior; the public characterization data does not expose separate
perturbation-variable vectors, and M2.7B adds no evaluator or core API.

## Markdown report

The Markdown artifact includes model identity and fingerprint, deterministic
configuration, the full trim request, both explicit characterization steps,
all summary counts, and one ordered table row per requested speed.

Characterized rows include trim variables and both derivatives. Detailed
diagnostics include the trim pitch moment and all four perturbation pitch
moments. Other rows and diagnostics use the exact typed outcome or unavailable
reason.

All floating-point values use the deterministic 17-digit scientific formatting
used by the trim reporting path.

## Exit codes

- Exit `0`: the sweep and characterization pipeline completed and both reports
  were written. This remains exit `0` when individual points are trim failures,
  integrity failures, or characterization-unavailable outcomes because those
  are analysis data.
- Exit `1`: CLI parsing, model loading, request construction, serialization,
  filesystem, or another operational failure.

The command never uses validation exit code `2` and never prints a PASS/FAIL
verdict.

## Determinism contract

For identical model bytes and arguments, including repeated-speed order, the
canonical JSON and Markdown bytes are identical. Subprocess integration tests
run the compiled `rcsim-app` binary twice into distinct output directories and
compare raw bytes.

## Physics and evidence boundary

M2.7B modifies only app orchestration/reporting and this document. It does not
modify `aircraft`, `sim_core`, `model`, finite-wing physics, XFOIL data, or the
reference-aircraft evidence pipeline. Tests use only the synthetic trim fixture.
The command does not claim runtime readiness, flight-data validation, static-
stability validation, or completion of a Clark Y evidence campaign.
