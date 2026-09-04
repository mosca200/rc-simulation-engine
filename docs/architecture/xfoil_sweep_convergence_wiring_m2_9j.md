# M2.9J — Wire XFOIL Sweep Convergence into Generated Evidence

## Purpose

M2.9J wires the canonical M2.9I sweep-convergence qualification into the
existing M2.9E execution pipeline so that a successfully parsed XFOIL polar
can be legitimately classified as `ConvergenceStatus::Converged` only when
the actual polar contains the complete requested alpha sweep.

This closes the pipeline:

```
M2.9E execution
→ canonical M2.9A polar parse
→ M2.9I deterministic sweep qualification
→ M2.9B generated evidence
→ Converged or Unresolved
```

## Scope Exclusions

**M2.9J does NOT:**

- Execute real XFOIL
- Modify M2.9A parser semantics
- Modify M2.9H runtime family semantics
- Modify aircraft, sim_core, or reference data
- Revert to process-exit-success = Converged
- Use stdout/stderr text heuristics
- Alter raw `.polar` files
- Change M2.9D validation logic

## Convergence Decision

For each completed + parseable XFOIL run:

1. Parse the polar through canonical `parse_xfoil_polar`
2. Reconstruct the exact alpha sweep from the run specification
3. Build the canonical M2.9I `SweepExpectation`
4. Run canonical `qualify_sweep_convergence`
5. If converged → `ConvergenceStatus::Converged`
6. Otherwise → `ConvergenceStatus::Unresolved`

## Sweep Source of Truth

The expected sweep comes from the execution manifest `RunSpec`:

- `alpha_start_deg`
- `alpha_end_deg`
- `alpha_step_deg`

These are the exact authored values from the campaign definition.

## Unit Conversion

The manifest expresses alpha in degrees. M2.9I expects radians. Conversion
is performed exactly once at the boundary:

```rust
alpha_rad = alpha_deg * (PI / 180.0)
```

## Convergence Tolerance

```
XFOIL_ALPHA_MATCH_TOLERANCE_RAD = 1e-7 radians (≈ 5.7e-6 degrees)
```

**Rationale:** XFOIL serializes polar alpha values in degrees with finite
decimal precision. The M2.9A parser converts degrees to radians via
`degrees * PI / 180.0`. For XFOIL output with ≥6 decimal places in degrees,
the combined serialization + conversion round-trip error is bounded by
~1.5e-8 radians. The 1e-7 tolerance provides ~6.7× headroom above that
bound while remaining tight enough to reject any genuinely incorrect alpha
point.

## Run Result Semantics

| Scenario | Execution Status | Convergence |
|---|---|---|
| Process failure | `ProcessFailed` | `unresolved` |
| Process success + missing polar | `MissingPolarOutput` | `unresolved` |
| Process success + unparseable polar | `UnparseablePolarOutput` | `unresolved` |
| Process success + parseable incomplete sweep | `CompletedParseable` | `unresolved` |
| Process success + parseable complete sweep | `CompletedParseable` | `converged` |

## Validation Manifest

The validation manifest now contains per-dataset `convergence_status`
reflecting M2.9I qualification:

- `"converged"` — complete sweep proven by M2.9I
- `"unresolved"` — incomplete sweep or non-parseable output

M2.9D `require_converged = true` naturally accepts fully complete campaigns
and rejects campaigns containing unresolved datasets.

## Changed Files

| File | Change |
|---|---|
| `crates/app/src/xfoil_runner_app.rs` | Added M2.9I integration |
| `crates/app/tests/xfoil_runner_cli.rs` | Added 30 convergence tests |
| `docs/architecture/xfoil_sweep_convergence_wiring_m2_9j.md` | This document |

## Public API Used

From `model` crate (M2.9I):

- `SweepExpectation::new(start_rad, end_rad, step_rad, tolerance_rad)`
- `qualify_sweep_convergence(expectation, import)`
- `XfoilSweepConvergenceQualification::is_converged()`
