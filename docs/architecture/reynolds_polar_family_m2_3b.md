# M2.3B — Reynolds-Aware Polar Family Core Primitive

## Purpose and boundary

M2.3B adds a generic mathematical primitive for sampling aerodynamic coefficients as functions of
angle of attack and Reynolds number. It does not select an aircraft operating envelope, convert
M2.3A evidence into runtime data, create a SIG KADET LT-40 model, or connect a Reynolds-dependent
polar to `AircraftSimulation`.

The existing `PolarTable` API and behavior remain unchanged. Aircraft-model schemas v0, v1, and v2
continue to resolve their legacy polar tables exactly as before.

## Data structure and validation

`ReynoldsPolar` owns one finite, positive `reynolds_number` and one already validated
`PolarTable`. `ReynoldsPolarFamily` owns at least one node. Construction may allocate and sorts the
nodes into strictly increasing Reynolds order using a deterministic total comparison. Equal
Reynolds numbers are rejected after sorting; there is no map iteration or implicit last-value win.

Each node retains its own alpha samples. Families do not require common alpha nodes and do not
resample or rewrite any table.

## Sampling sequence

`sample(reynolds_number, alpha_rad)` performs two explicit steps:

1. sample the lower and upper Reynolds-node `PolarTable` independently at `alpha_rad`;
2. interpolate the resulting CL, CD, and CM values between those Reynolds nodes.

Step 1 preserves the established `PolarTable` semantics exactly: deterministic binary search,
piecewise-linear alpha interpolation, exact sample preservation, and endpoint clamping. Sampling
the two tables before Reynolds interpolation permits different alpha grids without fabricating a
shared grid.

## Reynolds interpolation

For adjacent nodes `Re0 < Re < Re1`, the interpolation fraction is:

```text
t = (ln(Re) - ln(Re0)) / (ln(Re1) - ln(Re0))
```

and each coefficient is:

```text
C = C0 + t (C1 - C0),  C in {CL, CD, CM}
```

The implementation evaluates the logarithmic differences with a numerically stable log-ratio
form for closely spaced finite values. Linear interpolation in `ln(Re)` is an explicit numerical
policy for moving between discrete datasets. It is not a new aerodynamic law and does not justify
or infer any missing LT-40 data.

An exact Reynolds-node request samples only that node and preserves its `PolarTable` result
exactly. The returned `ReynoldsPolarSample` includes the coefficients, references to the lower and
upper nodes, the interpolation fraction, and a `ReynoldsRangeStatus`.

## Endpoint behavior and diagnostics

Reynolds sampling never extrapolates:

- below the smallest node, coefficients clamp to that node and status is `BelowRange`;
- exact-node and between-node requests use `ExactOrInRange`;
- above the largest node, coefficients clamp to that node and status is `AboveRange`.

For an exact or clamped single-node result, lower and upper node references are identical and the
interpolation fraction is zero. A future caller can turn below/above status into a diagnostic or
readiness gate without hidden extrapolation.

## Mach limitation

M2.3B is Reynolds-only. It performs no Mach interpolation, Mach clamping, or Mach-model inference.
M2.3A retains Mach metadata in its separate evidence artifacts; this core primitive neither drops
nor consumes that metadata. Future evidence selection must choose a justified Mach policy before
constructing runtime families.

## Determinism and allocation behavior

Family construction canonicalizes once and may allocate. Sampling uses immutable slices,
deterministic binary search, scalar `f64` arithmetic, and borrowed node references. It allocates no
memory and clones no vectors. Repeated identical requests are bit-deterministic on the supported
same-build/same-target boundary.

## Integration status

`ReynoldsPolarFamily` is exported by `sim_core`, but no `AircraftModel`, `RuntimePolar`,
`AeroElement`, RK4 evaluator, replay record, physics fingerprint, or 500 Hz path contains or calls
it in M2.3B. Connecting evidence to this primitive and integrating it into aircraft simulation
requires a later, separately reviewed M2.3 slice. M2.3C subsequently provides that generic,
schema-v3 integration while leaving the historical M2.3B primitive contract unchanged; see
[`reynolds_runtime_m2_3c.md`](reynolds_runtime_m2_3c.md).
