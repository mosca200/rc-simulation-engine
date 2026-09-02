# M2.8B — Deterministic Finite-Wing Induced Physics

## Purpose

M2.8B adds deterministic finite-wing induced-angle physics to the existing
quasi-2D element aerodynamic evaluation. Each `RuntimeAeroSurface` (introduced
in M2.8A) is independently solved for a common induced angle of attack
`alpha_i`, which modifies polar sampling for member elements while preserving
actual-flow force directions.

This is an engineering finite-wing correction, not a full lifting-line or
wake model.

## M2.8A vs M2.8B

| Concern | M2.8A | M2.8B |
|---------|-------|-------|
| Surface representation | Resolved `RuntimeAeroSurface` with span, AR, e | Same |
| Physics effect | None (metadata only) | Induced alpha, effective polar sampling, induced drag |
| Force directions | Quasi-2D local flow | Same (unchanged) |
| Polar sampling | At geometric alpha | At `alpha_geom - alpha_i` |

## Common Induced-Angle Equation

For each surface, solve for `alpha_i` such that:

```
g(alpha_i) = alpha_i - CL_surface(alpha_i) / (PI * AR * e) = 0
```

where:

```
CL_surface(alpha_i) = sum_j(W_j * CL_j(alpha_geom_j - alpha_i)) / sum_j(W_j)
W_j = q_j * S_j
```

- `q_j` = local section dynamic pressure at member j
- `S_j` = member element area
- `CL_j` = lift coefficient sampled from member j's polar at effective alpha
- `AR` = surface aspect ratio (`span_m^2 / area_m2`)
- `e` = surface span efficiency factor

If `sum(W_j) == 0` (all members at zero dynamic pressure), then:
`alpha_i = 0`, `CL_surface = 0`, `CDi_surface = 0`.

## Effective Alpha Semantics

For member j of a surface:

```
alpha_eff_j = alpha_geom_j - alpha_i
```

The effective alpha is used **only for polar coefficient sampling** (CL, CD, CM).
The physical local-flow velocity vector is NOT altered. Therefore:

- Drag direction remains opposite the ACTUAL local section flow
- Lift direction remains perpendicular to the ACTUAL local section flow
- Force application point remains the existing element position
- Body-frame transformation remains unchanged

**This is critical**: the induced effect changes POLAR SAMPLING ONLY, not the
force directions or element geometry.

## Profile vs Induced Drag

After solving the common surface state, `CDi_surface` is added to every
member's sampled profile drag:

```
CD_total_j = CD_profile_j(alpha_eff_j) + CDi_surface
```

where:

```
CDi_surface = CL_surface^2 / (PI * AR * e)
```

This distributes the total surface induced drag under the same local `q*S`
weighting as the profile drag. Induced drag is added exactly once per surface.

## Fixed-Polar and Reynolds-Family Support

Both existing polar binding types work with finite-wing surfaces:

**Fixed polar** (`RuntimeAeroPolarBinding::Polar`):
- CL sampled via `PolarTable::sample_clamped(alpha_eff)`
- CD sampled at `alpha_eff`, then `CDi_surface` added

**Reynolds family** (`RuntimeAeroPolarBinding::ReynoldsFamily`):
- Reynolds number computed from PHYSICAL section speed: `Re = V * chord / nu`
- `alpha_i` does NOT alter the Reynolds number
- CL/CD sampled from the family at `(Re, alpha_eff)`, then `CDi` added to CD

## Deterministic CL-Derived Bracket

The bisection bracket is derived from the actual polar data, not arbitrary
constants:

1. For each member binding, inspect all polar samples (fixed polar) or all
   samples of all Reynolds nodes (Reynolds family)
2. Find `CL_abs_max = max(|CL|)` across all reachable samples
3. Compute `alpha_bound = CL_abs_max / (PI * AR * e)`
4. Use bracket `[-alpha_bound, +alpha_bound]`

If `CL_abs_max == 0`, then `alpha_i = 0` immediately (no bisection needed).

## 40-Step Bisection Rule

The solver uses exactly 40 bisection iterations unless an exact root is found
at an endpoint or midpoint earlier. This provides precision of approximately
`alpha_bound / 2^40`, which is far below any physically meaningful threshold.

- No Newton iteration (polars may be non-monotonic post-stall)
- No wall-clock stopping
- No adaptive behavior
- Fully deterministic: identical inputs produce identical outputs

### Multiple-Root Limitation

For non-monotonic post-stall polars, the bisection selects one deterministic
root based on the bracket and sign-preserving bisection rule. This does NOT
claim physical branch uniqueness. The selected root is the one the bisection
converges to from the symmetric bracket centered at zero.

## Unassigned-Element Behavior

Elements not assigned to any `RuntimeAeroSurface` are evaluated through the
exact existing quasi-2D path. No finite-wing correction is applied to them.
They coexist with surface members in the same model.

## Legacy No-Surface Behavior

Models with no surfaces (schema v0-v4, or schema v5 with `surfaces: []`)
follow the exact legacy path through `evaluate_legacy_wrench()`. This preserves
previous wrench behavior exactly.

## 500 Hz / RK4 Stage-Local Recomputation

The induced-angle solution is computed independently for each RK4 stage from
that stage's `RigidBodyState`. This means:

- Each of the 4 RK4 stages solves its own `alpha_i`
- The `alpha_i` varies across stages as the stage state changes
- Control surface deflections are frozen across all stages (applied once per step)
- The bisection is allocation-free in the inner loop (pre-computed kinematics)

## SectionKinematics Primitive

A new public function `compute_section_kinematics()` in `sim_core::aero` provides
the reusable building block for the induced-angle solver. It computes the
section-plane velocity decomposition (airspeed, alpha, beta, dynamic pressure)
without sampling any polar or assembling forces. The existing
`evaluate_aero_element()` and `evaluate_reynolds_aero_element()` functions
preserve their current behavior unchanged.

## Current Limitations

M2.8B does NOT implement:

- Wing-tail downwash interaction
- Lifting-line spanwise circulation distribution
- Horseshoe vortices
- Prandtl lifting-line theory
- Propwash/slipstream effects
- Ground effect
- Dynamic stall or hysteresis
- Wake history
- Interference between separate surfaces
- Fuselage-wing interference

Each `RuntimeAeroSurface` is solved independently. These effects belong to
later slices.

## Explicit Disclaimers

- M2.8B does not generate aerodynamic coefficients.
- M2.8B does not modify the M2.8A surface representation.
- M2.8B does not alter force directions or element geometry.
- M2.8B does not make any aircraft runtime-ready for real-world validation.
