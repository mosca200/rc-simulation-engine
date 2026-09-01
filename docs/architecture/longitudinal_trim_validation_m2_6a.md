# M2.6A deterministic longitudinal trim sweep validation

## Scope and assumptions

M2.6A adds an offline validation primitive that drives the M2.5 deterministic bounded Newton trim
solver across an explicitly ordered, finite set of target airspeeds. The primitive is a thin
orchestration layer that:

1. accepts the airspeed list and the shared per-speed trim template once;
2. invokes the existing `solve_longitudinal_trim` for every speed in input order;
3. independently re-evaluates each converged solution through `evaluate_longitudinal_trim_candidate`
   to confirm the cached solver evaluation is physically re-derivable from the runtime path;
4. returns the ordered sequence of point results as a structured value.

M2.6A is offline validation infrastructure only. It does NOT modify the M2.5 trim solver, the
runtime stage path, the model schema, or any physics. It is also not on the 500 Hz hot loop; one
`Vec` of per-point results is allocated per call.

The slice deliberately does NOT add a CLI, a reporting layer, persistence, plot generation, or any
filesystem side effect. Downstream slices consume the structured result.

## Inputs and types

The primitive lives in `crates/aircraft/src/trim_sweep.rs` and exports:

- `LongitudinalTrimSweepError` — fail-closed sweep request errors.
- `LongitudinalTrimSweepRequest` — owns the shared per-speed trim template and the ordered
  airspeed list.
- `LongitudinalTrimSweepPoint` — one target airspeed plus its evaluated outcome.
- `LongitudinalTrimSweepOutcome` — three-variant per-point result.
- `LongitudinalTrimSweep` — the ordered sweep result.
- `solve_longitudinal_trim_sweep` — the entry point.

The shared template is intentionally identical in shape to the M2.5 trim request minus the
target airspeed, because the airspeed is per-point. The template carries alpha / elevator /
throttle bounds, the initial guess, the force/moment tolerances, and the maximum iteration count.
Reusing M2.5's own value types means the sweep primitive cannot drift from the solver's
expectations on bounds, units, or tolerance ordering.

## Fail-closed sweep request validation

`LongitudinalTrimSweepRequest::new` performs three layers of validation up front and produces no
partial results on failure:

1. The speed list is non-empty.
2. Every speed is finite and strictly positive, with the first failing index reported.
3. The shared template successfully constructs one throwaway M2.5
   `LongitudinalTrimRequest` with the first speed; any M2.5 request error
   (`InvalidBounds`, `InvalidElevatorBounds`, `InvalidThrottleBounds`, `NonFiniteInitialGuess`,
   `InvalidTolerance`, `InvalidIterationLimit`) is surfaced as
   `LongitudinalTrimSweepError::InvalidSharedRequest`.

The first two layers guarantee that, if `new` returns `Ok`, every per-point request the sweep
constructs will pass M2.5's per-request validation. The third layer guarantees that any error in
the shared template is reported before any trim work begins.

## Per-point execution

For each requested speed in input order the primitive constructs a fresh M2.5
`LongitudinalTrimRequest` from the shared template, calls `solve_longitudinal_trim`, and
classifies the result:

- Solver success → independently re-evaluate the returned variables through
  `evaluate_longitudinal_trim_candidate`. If the re-evaluation produces an identical
  `LongitudinalTrimEvaluation`, the point is recorded as
  `LongitudinalTrimSweepOutcome::Success { solution }`. If the re-evaluation diverges (or returns
  `None` because runtime physics produced non-finite values for a state the M2.5 solver already
  accepted as converged), the point is recorded as
  `LongitudinalTrimSweepOutcome::ReEvaluationMismatch` with both evaluations preserved so a
  reporting layer can flag the integrity issue.
- Solver failure → the point is recorded as
  `LongitudinalTrimSweepOutcome::TrimFailure { failure }` with the M2.5 failure reason, iteration
  count, and last finite evaluation preserved.

A bounded physical problem with no feasible solution is therefore visible in the structured
result as a `TrimFailure` — the sweep does NOT abort, does NOT panic, and does NOT misclassify a
physical infeasibility as a software error.

## Re-evaluation semantics

M2.5 already establishes that `evaluate_longitudinal_trim_candidate` returns the same
`LongitudinalTrimEvaluation` that the solver's last accepted iteration produced, because both
paths share `evaluate_candidate`. The sweep's independent re-evaluation is therefore a regression
guard: if a future refactor of M2.5 or of `evaluate_candidate` desynchronises the solver-cached
evaluation from the runtime evaluation, the sweep records that point as
`ReEvaluationMismatch` rather than silently trusting the solver output. The M2.6A tests assert
zero mismatches against the M2.5 fixture.

## Determinism, ordering, and performance

The sweep is fully deterministic and stable:

- The point vector preserves input order; `points()[i].target_airspeed_mps` equals the
  caller-supplied `target_airspeeds_mps[i]`.
- Identical inputs (same model, same config, same request) produce identical structured
  results: `assert_eq!(first, second)` is asserted in tests.
- The shared template means the per-point constructor is allocation-free; the only allocation is
  the per-point result buffer itself.

The M2.5 trim types are reused for per-point solutions and failures, so the sweep result can be
consumed without any conversion in the test or reporting layer.

## Evidence boundary and limitations

The dedicated fixture remains classified `synthetic_test`, has `reference_aircraft = null`, and
uses invented architecture-test values. The sweep does not promote the fixture to
`ReferenceAircraft` and does not introduce any SIG Kadet LT-40 evidence. No model schema, no
runtime physics, and no M2.5 trim primitive is modified by M2.6A.

M2.6A does NOT add:

- a CLI, a reporting layer, or any presentation concern;
- persistence, plotting, filesystem side effects, or telemetry;
- a property-based / fuzz harness for the M2.5 solver (a future slice may add one);
- lateral trim, sideslip, coordinated turns, climbs, descents, dynamic trim, autopilot, or any
  control law;
- a coupled 6-DoF trim that solves translational, rotational, and sideslip states together;
- any modification to the M2.5 trim solver, the runtime stage path, or the M2.4B propulsion
  primitive.

M2.6A exists to make the M2.5 deterministic primitive bulk-testable across a flight envelope
before the downstream reporting and CI gates build on it.