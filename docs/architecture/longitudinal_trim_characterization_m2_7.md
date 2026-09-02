# M2.7 — Local Longitudinal Trim Characterization

## Purpose

M2.7 adds deterministic **local** longitudinal characterization around already verified M2.6A trim solutions. The purpose is to establish quantitative baseline observables **before** finite-wing physics is introduced in M2.8.

The two quantities computed by this slice are:

1. **Local trim pitch stiffness**: `dMy/dAlpha` (N·m/rad)
2. **Local elevator control effectiveness**: `dMy/dElevatorCommand` (N·m per normalized elevator command)

These are **local finite-difference derivatives** around a verified trim point. They are **dimensional** quantities — no normalized aerodynamic coefficient derivatives are computed.

## Relation to M2.5

M2.5 provides the bounded Newton trim solver (`solve_longitudinal_trim`) and the runtime evaluator (`evaluate_longitudinal_trim_candidate`). M2.7 **reuses** these existing primitives — it does not create a parallel physics evaluator.

The finite-difference perturbations in M2.7 use the same `evaluate_longitudinal_trim_candidate` path that M2.5 uses for its solver Jacobian, but with **different semantics**:

- M2.5 solver Jacobian steps are relative to bound spans and may be asymmetric
- M2.7 characterization steps are **explicit, symmetric, caller-supplied** central-difference steps
- M2.7 does **not** re-trim the perturbed points — they are frozen-control evaluations

## Relation to M2.6A/M2.6B

M2.6A introduced the sweep primitive (`solve_longitudinal_trim_sweep`) with integrity-level outcomes:

- `Success` — solver converged and independent re-evaluation matched
- `TrimFailure` — bounded physical problem lacked a feasible solution
- `ReEvaluationMismatch` — solver solution disagreed with independent re-evaluation
- `ReEvaluationUnverifiable` — solver converged but independent re-evaluation returned non-finite

M2.7 operates **only** on `Success` outcomes. All other outcomes are recorded as explicit non-characterized states without fabricating derivative values. This preserves the integrity semantics introduced in M2.6A.

M2.6B added CLI and reporting for the sweep. M2.7 does not change the CLI or the M2.6B report schema.

## Exact Derivative Definitions

### dMy/dAlpha (Local Trim Pitch Stiffness)

For a verified successful trim point with:
- `alpha0` — trim angle of attack (rad)
- `elevator0` — trim elevator command (normalized)
- `throttle0` — trim throttle (normalized)
- `V` — target airspeed (m/s)

Calculate:

```
My_minus = My(alpha0 - h_alpha, elevator0, throttle0) at airspeed V
My_plus  = My(alpha0 + h_alpha, elevator0, throttle0) at airspeed V

dMy/dAlpha = (My_plus - My_minus) / (2 * h_alpha)
```

Units: **N·m/rad**

### dMy/dElevatorCommand (Local Elevator Control Effectiveness)

At the same verified trim point:

```
My_minus = My(alpha0, elevator0 - h_elevator, throttle0) at airspeed V
My_plus  = My(alpha0, elevator0 + h_elevator, throttle0) at airspeed V

dMy/dElevator = (My_plus - My_minus) / (2 * h_elevator)
```

Units: **N·m per normalized elevator command**

## Frozen-Variable Semantics

When perturbing **alpha**:
- Target airspeed remains **exactly fixed**
- Elevator command remains **exactly fixed** at `elevator0`
- Throttle remains **exactly fixed** at `throttle0`
- Environment remains **exactly fixed**
- Angular velocity remains whatever the existing steady trim evaluator defines

When perturbing **elevator**:
- Target airspeed remains **exactly fixed**
- Alpha remains **exactly fixed** at `alpha0`
- Throttle remains **exactly fixed** at `throttle0`
- Environment remains **exactly fixed**

**Perturbed points are NOT re-trimmed.** This is a local frozen-control derivative.

## Central-Difference Formulas

Both derivatives use **symmetric central differences**:

```
f'(x) ≈ (f(x + h) - f(x - h)) / (2h)
```

The step sizes `h_alpha` and `h_elevator` are **explicit caller-supplied values** with no implicit defaults. They are validated at construction time to be finite and strictly positive.

## Dimensional Units

M2.7 produces **dimensional** derivatives:

- `dMy/dAlpha`: N·m/rad
- `dMy/dElevator`: N·m per normalized elevator command

There is currently no guaranteed aircraft-wide reference area or reference chord contract suitable for normalized coefficient derivatives. M2.7 does **not** compute:

- `Cm_alpha` (normalized pitch stiffness)
- `Cm_delta_e` (normalized elevator effectiveness)
- Static margin
- Neutral point
- Aerodynamic center

## Bound Handling

Before evaluating a perturbation, M2.7 explicitly verifies:

```
alpha0 - h_alpha >= alpha_lower
alpha0 + h_alpha <= alpha_upper

elevator0 - h_elevator >= elevator_lower
elevator0 + h_elevator <= elevator_upper
```

If a symmetric perturbation does not fit within the bounds:

- **DO NOT clamp**
- **DO NOT silently switch to a one-sided derivative**
- **DO NOT reduce h automatically**

Instead, return an explicit per-point `CharacterizationUnavailable` outcome with reason `AlphaPerturbationOutOfBounds` or `ElevatorPerturbationOutOfBounds`.

This ensures central-difference semantics remain exact and auditable.

## Failure/Unavailable Semantics

### Non-Finite Perturbations

If any required perturbation call to `evaluate_longitudinal_trim_candidate` returns `None` (indicating non-finite values from the runtime path):

- **DO NOT panic**
- **DO NOT fabricate a value**
- **DO NOT substitute the base trim evaluation**

Record the point as `CharacterizationUnavailable` with reason `AlphaPerturbationNonFinite` or `ElevatorPerturbationNonFinite`, including which side (`Minus` or `Plus`) failed.

### Non-Finite Derivatives

If the final derivative itself is non-finite:

- Record explicit `CharacterizationUnavailable` with reason `NonFinitePitchStiffness` or `NonFiniteElevatorEffectiveness`

Nothing non-finite may be returned as a successful derivative.

### M2.6A Non-Success Outcomes

Only `LongitudinalTrimSweepOutcome::Success` points may be characterized. All other outcomes produce explicit non-characterized states:

- `TrimFailure` → `NotCharacterizedTrimFailure`
- `ReEvaluationMismatch` → `NotCharacterizedReEvaluationMismatch`
- `ReEvaluationUnverifiable` → `NotCharacterizedReEvaluationUnverifiable`

**Never fabricate derivatives from:**
- `last_evaluation` (from trim failures)
- Solver-only evaluation (from mismatch/unverifiable)
- Absent independent evaluations

## Determinism

Identical inputs produce identical outputs:

- Same model
- Same simulation config
- Same sweep request
- Same verified sweep
- Same characterization steps

→ Identical structured characterization results

**No:**
- Timestamps
- Random IDs
- Wall clock
- Process IDs
- Unordered output semantics
- Stochastic perturbations
- Adaptive finite-difference step selection

Sweep input order is preserved exactly.

## Sign Convention

The sign of the derivatives is inherited from the current body-Y moment / NED / runtime convention used by `evaluate_aircraft_instantaneous` and the M2.5 trim solver.

M2.7 **reports measured derivatives**. It does **not** interpret the sign as a verdict:

- **DO NOT** decide "stable" / "unstable" based on sign
- **DO NOT** decide "safe" / "unsafe" based on sign
- **DO NOT** decide "acceptable" / "unacceptable" based on sign
- **DO NOT** introduce arbitrary thresholds

M2.7 is a measurement primitive, not a stability assessment.

## Limitations

### What M2.7 Does NOT Prove

M2.7 does **not** establish:

- **Static margin** — requires normalized `Cm_alpha` and knowledge of the neutral point
- **Neutral point** — requires sweeping CG or computing aerodynamic center
- **Aerodynamic center** — requires lifting-surface theory or empirical calibration
- **Full longitudinal stability** — requires dynamic derivatives (`Cm_q`, `Cm_alpha_dot`) and eigenvalue analysis
- **Real-aircraft fidelity** — the synthetic fixture is a test artifact, not a calibrated aircraft
- **Finite-wing correctness** — M2.7 uses the current runtime physics, which does not yet include finite-wing corrections

### Pre-M2.8 Baseline

M2.7 is intended to provide a **pre-M2.8 baseline** so later finite-wing physics changes can be quantitatively compared against the same observable derivatives.

When M2.8 introduces finite-wing corrections (induced angle, induced drag, aspect-ratio effects, Oswald efficiency, etc.), the same characterization API can be re-run on the same sweep to measure how the derivatives change.

This enables **quantitative regression testing** of finite-wing physics: if M2.8 changes `dMy/dAlpha` by 20% at a given speed, that is a measurable, auditable delta.

## Public API

### Types

- `LongitudinalTrimCharacterizationSteps` — validated step sizes
- `LongitudinalTrimCharacterization` — ordered result collection
- `LongitudinalTrimCharacterizationPoint` — per-point target airspeed + outcome
- `LongitudinalTrimCharacterizationPointOutcome` — enum: `Characterized`, `NotCharacterizedTrimFailure`, `NotCharacterizedReEvaluationMismatch`, `NotCharacterizedReEvaluationUnverifiable`, `CharacterizationUnavailable`
- `LongitudinalTrimCharacterizationData` — successful characterization with all samples and derivatives
- `CharacterizationUnavailableReason` — enum: `AlphaPerturbationOutOfBounds`, `ElevatorPerturbationOutOfBounds`, `AlphaPerturbationNonFinite`, `ElevatorPerturbationNonFinite`, `NonFinitePitchStiffness`, `NonFiniteElevatorEffectiveness`
- `PerturbationSide` — enum: `Minus`, `Plus`

### Errors

- `CharacterizationStepsError` — `InvalidAlphaStep`, `InvalidElevatorStep`
- `LongitudinalTrimCharacterizationError` — `SweepLengthMismatch`, `SweepTargetAirspeedMismatch`

### Entry Point

```rust
pub fn characterize_longitudinal_trim_sweep(
    model: &AircraftModel,
    config: &AircraftSimulationConfig,
    sweep_request: &LongitudinalTrimSweepRequest,
    sweep: &LongitudinalTrimSweep,
    steps: LongitudinalTrimCharacterizationSteps,
) -> Result<LongitudinalTrimCharacterization, LongitudinalTrimCharacterizationError>
```

## Tests

M2.7 includes focused unit tests covering:

1. Step validation (finite, positive)
2. Sweep/request length mismatch
3. Sweep/request target airspeed mismatch
4. Successful sweep produces characterization in order
5. All characterized derivatives are finite
6. `dMy/dAlpha` equals central-difference formula
7. `dMy/dElevator` equals central-difference formula
8. Alpha perturbation freezes elevator and throttle
9. Elevator perturbation freezes alpha and throttle
10. Target airspeed unchanged during perturbations
11. Too-large alpha step produces unavailable
12. Too-large elevator step produces unavailable
13. Trim failure point is not characterized
14. Identical inputs produce identical outputs
15. Synthetic fixture remains synthetic test

## Summary

M2.7 is a small, deterministic, local characterization primitive that:

- Reuses the existing M2.5 runtime evaluator
- Operates only on verified M2.6A `Success` outcomes
- Uses exact symmetric central differences
- Does not re-trim perturbed points
- Does not silently clamp
- Preserves integrity semantics
- Produces dimensional derivatives
- Does not interpret sign as verdict
- Does not compute static margin or normalized coefficients
- Provides a pre-M2.8 baseline for finite-wing physics comparison
