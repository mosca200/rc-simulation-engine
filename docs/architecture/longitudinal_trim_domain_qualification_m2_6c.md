# M2.6C — Longitudinal Trim Domain Qualification

## Convergence ≠ Qualification

A trim solution that numerically converges is NOT automatically qualified. The Newton solver finds an operating point where longitudinal force, vertical force, and pitch moment residuals are within tolerance. This says nothing about whether the operating point lies inside the aerodynamic or propulsion evidence domains.

M2.6C adds an offline qualification layer that determines whether a converged trim operating point is actually supported by the underlying aerodynamic and propulsion data.

## Clamping ≠ Evidence Support

Runtime aerodynamic and propulsion samplers intentionally clamp outside their tabulated domains. A polar table sampled at alpha = 0.50 when the last sample is at alpha = 0.40 returns the endpoint value. The runtime can evaluate this; the evidence does not support it.

Qualification distinguishes:
- "runtime can evaluate this value" (clamping)
- "this operating point is inside the supported evidence domain" (qualification)

A solution relying on endpoint clamping is NOT qualified.

## Range Status

Every domain comparison produces a typed [`RangeStatus`], computed by the public pure
classifier `classify_range_status(value, lower, upper)` — the single classifier used for
polar alpha domains, Reynolds family domains, and propeller J domains:

| Status | Meaning | Supported? |
|---|---|---|
| `BelowRange` | strictly below the authored interval | NO (clamped) |
| `AtLowerBound` | bitwise-equal to the authored lower endpoint | YES |
| `InRange` | numerically inside the interval, but not bitwise-identical to an endpoint | YES |
| `AtUpperBound` | bitwise-equal to the authored upper endpoint | YES |
| `AboveRange` | strictly above the authored interval | NO (clamped) |
| `NonFinite` | value or bound is NaN / ±Infinity | NO (fail-closed) |
| `InvalidRange` | finite lower bound is greater than upper bound | NO (fail-closed) |

Boundary membership uses EXACT bitwise equality with the authored endpoint. Numerically
equivalent signed zero with different bits remains supported as `InRange`, but is not
mislabeled as the authored endpoint. Non-finite inputs and inverted intervals fail closed.
No epsilon is introduced anywhere: an epsilon would invent support that the authored data
does not declare. Runtime clamping is unchanged; qualification merely reveals when it is
relied on.

## Geometric Alpha vs Finite-Wing Sampled Alpha

For elements belonging to a `RuntimeAeroSurface` (finite-wing members), runtime samples coefficients at:

```
alpha_sample = alpha_geom - alpha_i
```

where `alpha_i` is the induced angle of attack from the M2.8B bisection solver. Qualification audits `alpha_sample`, NOT `alpha_geom`. The induced angle `alpha_i` is obtained from the SAME deterministic bisection used by runtime — no second approximate solver is implemented.

For unassigned elements (not part of any surface), `alpha_sample == alpha_geom` and legacy quasi-2D behavior is preserved.

## Physical Reynolds

Reynolds numbers use the PHYSICAL section airspeed:

```
Re = section_airspeed_mps * chord_m / kinematic_viscosity_m2_s
```

This is NOT any effective-alpha-modified velocity. Physical flow is physical flow; `alpha_i` changes coefficient sampling only.

## Dual-Node Reynolds Alpha-Domain Rule

When the physical Reynolds falls between two family nodes, the canonical runtime interpolates coefficients from BOTH contributing nodes. Qualification requires `alpha_sample` to lie inside BOTH contributing `PolarTable` alpha domains. If `alpha_sample` is inside one node's domain but outside the other's, qualification fails.

At an exact Reynolds node, only that single node's alpha domain is checked.

Below or above the Reynolds family range, Reynolds qualification fails. The alpha audit still runs against the endpoint table actually sampled by runtime clamping.

## Propulsion Domain

### Fixed Propeller Table

J (advance ratio) support is defined by the first and last sample in the `PropellerCoefficientTable`. Endpoints classify as `AtLowerBound`/`AtUpperBound` (supported). Strictly outside = blocker.

The audit also records the runtime quantities available from the authoritative propulsion evaluation: throttle, axial airspeed, shaft speed (rad/s and RPM), advance ratio J, thrust, and shaft torque.

### RPM Support

RPM is classified ONLY when the authored model/runtime data explicitly declares an RPM support range. No such range exists in the current model data, so the audit reports `RpmDomainStatus::NotDeclared` for both fixed tables and shaft-speed maps while still recording the runtime RPM value. No RPM envelope is ever invented.

### Shaft-Speed Map

Shaft speed support is defined by the first and last node's `shaft_speed_rad_s`. Below/above = blocker.

When shaft speed falls between two map nodes, J must be inside BOTH contributing tables' J domains. If J is valid in only one table, qualification fails.

### Stopped Prop

If runtime evaluates J = 0 (stopped shaft), qualification audits whether J = 0 is actually supported by the sampled coefficient table. It is not automatically qualified.

## Full Residual Audit

For every audited trim point, the qualification preserves signed raw values for:

**Body wrench:** Fx, Fy, Fz, Mx, My, Mz  
**Linear acceleration (world):** x, y, z  
**Angular acceleration (body):** x, y, z  
**Trim residuals:** longitudinal force, vertical force, pitch moment  

Values below tolerance are NOT zeroed. The raw signed values are preserved for audit.

## Explicit Limits

The caller must supply finite non-negative maxima for:

| Quantity | Unit |
|---|---|
| \|Fy_body\| | N |
| \|Mx_body\| | N·m |
| \|Mz_body\| | N·m |
| \|a_world_y\| | m/s² |
| \|angular_accel_body_x\| | rad/s² |
| \|angular_accel_body_z\| | rad/s² |

No hidden defaults. NaN, ±Inf, and negative limits are rejected at construction.

The acceptance rule is documented and deterministic: a residual value passes when
`|value| <= limit`. A value EXACTLY equal to its limit passes; no epsilon is applied.

## Point Outcomes

Every qualified point carries a typed outcome; no string status codes are used:

- `Qualified` — trim succeeded, every applicable authored domain is supported, every
  off-axis residual limit passes, all audit values are finite, and the accepted solution
  re-evaluates identically. Carries the full diagnostics.
- `NotQualifiedTrimFailure` — the sweep point never produced a trim solution. Carries the
  typed solver failure and NOTHING else: no element, propulsion, or residual diagnostics
  are fabricated.
- `NotQualifiedDomainViolation` — at least one authored-domain blocker exists (aero alpha,
  Reynolds family, contributing-node alpha, propulsion J/shaft speed). ALL blockers are
  preserved; nothing is dropped at the first violation.
- `NotQualifiedResidualViolation` — no domain blockers, but at least one off-axis
  residual exceeds a caller-supplied limit. ALL blockers are preserved.
- `QualificationUnavailable` — the point produced a trim evaluation whose diagnostics
  cannot be trusted or presented (non-finite audit value, failed deterministic
  re-evaluation) or whose evaluation integrity was already broken at sweep level
  (`SweepReEvaluationMismatch` / `SweepReEvaluationUnverifiable`; no diagnostics are
  fabricated for these).

Variant-selection precedence is deterministic: any Integrity failure > Domain violation >
Residual violation > Qualified. Integrity failures map to `QualificationUnavailable` with
all partially valid diagnostics and blockers preserved.

Blocker categories are typed (`QualificationBlockerCategory::{Domain, Residual,
Integrity}`) via `QualificationBlocker::category()`.

## Sweep Qualification

`qualify_longitudinal_trim_sweep(model, config, sweep, limits)` operates on the existing
M2.6A sweep result and returns exactly one qualification point per sweep point, in the
sweep's (and therefore the request's) input order. Successful points are fully audited;
trim failures map to `NotQualifiedTrimFailure`; sweep-level re-evaluation integrity
outcomes map to `QualificationUnavailable`.

## Qualified Definition

A successful trim point is **Qualified** only when ALL apply:

1. Every aerodynamic `alpha_sample` is inside its polar support
2. Every applicable Reynolds number is inside its family support
3. Reynolds contributing node alpha domains support `alpha_sample`
4. Propulsion J is inside support (when propulsion exists)
5. Shaft speed is inside map support (when a shaft-speed map exists)
6. All explicit off-axis limits pass
7. All audit values are finite
8. Accepted trim solution remains deterministically re-evaluable

Otherwise the point is `NotQualifiedDomainViolation`, `NotQualifiedResidualViolation`, or
`QualificationUnavailable` (see Point Outcomes) with ALL applicable typed blockers
preserved. The qualification does NOT stop at the first blocker.

## Blocker Ordering

Deterministic externally visible order:

1. Aero elements in model order
   - Alpha issue first
   - Reynolds issue second
2. Propulsion
   - Shaft-speed issue first
   - J issue second
3. Residual limits: Fy, Mx, Mz, lateral acceleration, roll acceleration, yaw acceleration
4. Integrity / non-finite issues

No `HashMap`/`HashSet` iteration determines public ordering.

## Zero Runtime-Physics Change

M2.6C does NOT change:
- RK4 integration
- Newton trim algorithm or convergence rules
- Induced-angle equation or 40-step M2.8B bisection
- Induced drag or physical force directions
- Reynolds interpolation or polar clamping
- Propulsion equilibrium solver or propeller sampling
- Controls/servo physics

Qualification is an offline consumer of the same runtime primitives used by the integrated
physics path: `propeller_slipstream`, `surface_downwash_with_slipstream`,
`solve_surface_induced_alpha_with_physical_flow`, and `physical_section_kinematics`. It does
not introduce an alternate aerodynamic or propulsion solver.

## 500 Hz Safety

Qualification is offline infrastructure. No `Vec`, `String`, or report construction is added
to the normal 500 Hz / RK4 hot path. The shared runtime physics helpers remain allocation-free.
