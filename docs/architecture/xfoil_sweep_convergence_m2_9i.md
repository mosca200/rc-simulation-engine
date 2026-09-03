# M2.9I — Deterministic XFOIL Sweep Convergence Qualification

## Purpose

M2.9I provides a deterministic, off-runtime qualification step that answers:

> "Did this parsed XFOIL polar actually contain every requested alpha point
> of the commanded sweep?"

This is the convergence proof between:

- M2.9A parsed XFOIL output
- M2.9E generated evidence (currently `Unresolved`)
- Legitimately `Converged` generated evidence

M2.9I does NOT execute XFOIL. It does NOT modify aircraft runtime physics.

## Background

M2.9E intentionally does NOT equate process exit success with solver
convergence. Generated datasets therefore remain
`ConvergenceStatus::Unresolved`.

M2.9H correctly refuses to promote `Unresolved` evidence into a runtime
`ReynoldsPolarFamily`.

M2.9I provides the deterministic evidence needed to decide when an XFOIL
alpha sweep may legitimately be classified as `Converged`.

## Scope Exclusions

**M2.9I does NOT:**

- Execute XFOIL or any external solver
- Parse XFOIL text (uses canonical `XfoilPolarImport` from M2.9A)
- Modify aircraft runtime physics
- Modify `AircraftModel`, `sim_core::PolarTable`, or runtime polars
- Alter aerodynamic coefficients (CL, CD, CM)
- Automatically mutate `XfoilEvidenceDataset` or `ConvergenceStatus`
- Incorporate M2.8C, M2.9G, or M2.9H implementation
- Use real Clark Y or LT-40 data

## Public API

### `SweepExpectation`

Validated alpha sweep expectation containing:

| Field | Type | Description |
|---|---|---|
| `alpha_start_rad` | `f64` | First requested alpha (radians) |
| `alpha_end_rad` | `f64` | Last requested alpha (radians) |
| `alpha_step_rad` | `f64` | Alpha increment per step (signed, radians) |
| `alpha_match_tolerance_rad` | `f64` | Match tolerance (radians, ≥ 0) |

Construction via `SweepExpectation::new(start, end, step, tolerance)` validates
all invariants and returns `Result<Self, SweepExpectationError>`.

### `SweepExpectationError`

Typed validation errors:

- `NonFiniteStart` / `NonFiniteEnd` / `NonFiniteStep` / `NonFiniteTolerance`
- `ZeroStep`
- `NegativeTolerance`
- `StepDirectionMismatch` — step sign does not move from start toward end
- `UnreachableEndpoint` — endpoint not reachable by integral number of steps
  within tolerance

### `XfoilSweepConvergenceQualification`

Result of qualifying a parsed polar against a sweep expectation:

| Method | Returns | Description |
|---|---|---|
| `is_converged()` | `bool` | Whether sweep is fully converged |
| `status()` | `SweepConvergenceStatus` | Typed status |
| `expected_sample_count()` | `usize` | Commanded point count |
| `observed_sample_count()` | `usize` | Parsed sample count |
| `blockers()` | `&[SweepConvergenceBlocker]` | Deterministic blocker list |
| `to_convergence_status()` | `ConvergenceStatus` | Repository-wide mapping |

### `SweepConvergenceStatus`

- `Converged` — all expected points present and matched
- `NotConverged` — one or more blockers prevent convergence

### `SweepConvergenceBlocker`

Typed deterministic blockers:

- `SampleCountMismatch { expected, observed }` — count differs
- `AlphaMismatch { index, expected_alpha_rad, observed_alpha_rad, tolerance_rad }`
  — alpha at position `index` exceeds tolerance

## Sweep Validation Rules

1. All values (start, end, step, tolerance) must be finite
2. `alpha_step_rad != 0`
3. Step sign must move from start toward end
4. Tolerance must be finite and ≥ 0
5. Endpoint must be reachable by an integral number of steps within tolerance
6. Inputs are never silently modified

## Expected Point Count

The inclusive requested sequence is:

```
alpha_expected(index) = alpha_start + index * alpha_step
```

for `index` in `0..expected_count`, where:

```
expected_count = round((end - start) / step) + 1
```

This avoids floating-point accumulation drift by computing each expected
alpha directly from the index rather than iteratively accumulating.

## Convergence Definition

Complete sweep convergence means every commanded alpha point produced a
parseable polar row. Specifically:

1. Parsed sample count equals expected requested point count
2. Every sample exists in the exact requested sequence position
3. For every index: `|observed_alpha - expected_alpha| <= tolerance`
4. No duplicate/missing/out-of-sequence alpha point can pass
5. First requested point is present
6. Last requested point is present

For descending sweep expectations (negative step), the expected sequence
is compared in reverse against the observed data, since XFOIL output
ordering depends on the commanded sweep direction.

### What convergence does NOT prove

- Experimental accuracy
- Airfoil applicability to an aircraft
- 3D finite-wing accuracy
- Stall fidelity beyond XFOIL
- Transition-model correctness
- LT-40 suitability

## Tolerance Semantics

- `tolerance >= 0` is required
- Match condition: `|observed - expected| <= tolerance` (inclusive)
- Zero tolerance means exact floating-point comparison
- No hidden tolerances are added

## Blocker Ordering

Deterministic ordering:

1. Count blocker first (if count differs)
2. Alpha mismatches in ascending sample index order
3. No `HashMap` ordering

## ConvergenceStatus Mapping

| SweepConvergenceStatus | ConvergenceStatus |
|---|---|
| `Converged` | `ConvergenceStatus::Converged` |
| `NotConverged` | `ConvergenceStatus::Unresolved` |

M2.9I never maps to `ConvergenceStatus::Failed`. It only proves complete
convergence when evidence is sufficient; otherwise remains fail-closed /
unresolved.

## Runtime Safety

This module is off-runtime evidence processing. No simulation stepping
changes. No allocations concern in the flight hot path because this API
is not used there.

## Module Location

- Implementation: `crates/model/src/reference_xfoil_convergence.rs`
- Tests: `crates/model/tests/xfoil_sweep_convergence_m2_9i.rs`
- Public API exported through `crates/model/src/lib.rs`
