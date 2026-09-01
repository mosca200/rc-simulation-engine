# S4 single aerodynamic element

## Frames and reference point

The element frame is right-handed: +X is local chord-forward, +Y is positive span, and +Z is local down. `orientation_body_from_element` is an active Hamilton rotation:

```text
v_body = orientation_body_from_element * v_element
v_element = inverse(orientation_body_from_element) * v_body
```

The rigid-body origin is the centre of mass/CG used by the 6DoF equations and inertia tensor. `position_body_m` points from that origin to the element aerodynamic centre. A body force applied there contributes `position_body_m × force_body_n`; the intrinsic aerodynamic moment is added separately.

## Local air-relative velocity

The velocity is the element's velocity through the air, not incoming-wind direction:

```text
V_air_world = linear_velocity_world_mps - wind_velocity_world_mps
V_air_body_cg = world_to_body(orientation_world_from_body, V_air_world)
V_air_body_element = V_air_body_cg + angular_velocity_body_radps × position_body_m
V_air_element = inverse(orientation_body_from_element) * V_air_body_element
```

For `V_air_element = [u, v, w]`, S4 is quasi-2D and uses `V_section = [u, 0, w]`. The spanwise component does not affect CL, CD, CM, or dynamic pressure. It contributes only to diagnostic `beta = atan2(v, sqrt(u²+w²))`. Side force and three-dimensional crossflow are future work.

Angle of attack is `alpha = atan2(w, u)`, so forward velocity with positive local-down velocity has positive alpha. When `sqrt(u²+w²) < 1e-9 m/s`, alpha is defined as zero and force, intrinsic moment, dynamic pressure, and wrench are zero. This handles the direction singularity without aerodynamic stabilization.

## Forces and moment

For section speed `V`:

```text
q = 0.5 * air_density_kg_m3 * V²
Vhat = [u, 0, w] / V
drag_direction_element = -Vhat
lift_direction_element = span_axis_element × Vhat
span_axis_element = [0, 1, 0]
L = q * area_m2 * CL
D = q * area_m2 * CD
force_element_n = lift_direction_element * L + drag_direction_element * D
```

Because the span axis and `Vhat` are perpendicular unit vectors, their cross product is already unit length. Forward flow therefore gives positive lift along element -Z. Drag always opposes section velocity.

The intrinsic pitching moment is:

```text
M_pitch = q * area_m2 * chord_m * CM
intrinsic_moment_element_nm = [0, M_pitch, 0]
moment_body_nm = position_body_m × force_body_n + intrinsic_moment_body_nm
```

Positive CM produces a positive +Y right-hand-rule pitch moment, corresponding to nose-up in FRD.

## Polar sampling and scope

`PolarTable` owns at least two finite samples with strictly increasing radian alpha and non-negative CD. Construction validates order; the hot path performs a deterministic binary search and piecewise-linear interpolation without allocation. Exact table samples and endpoints are preserved. `sample_clamped` returns the first or last coefficients outside the tabulated range; there is no extrapolation or stall heuristic.

M2.3B adds a separate generic `ReynoldsPolarFamily` core primitive while preserving this legacy
table contract. It samples each table with `sample_clamped` before Reynolds interpolation and is
not connected to the S4 element evaluator or aircraft runtime in that slice. See
[`reynolds_polar_family_m2_3b.md`](reynolds_polar_family_m2_3b.md).

S4 intentionally does not model side force, induced drag, finite-wing effects, propwash, propulsion, controls, atmosphere variation, ground effect, turbulence, aircraft assembly, or rendering. The canonical `Simulation` does not yet own aerodynamic elements; stage-correct integration is exercised directly through the generic RK4 evaluator boundary.
