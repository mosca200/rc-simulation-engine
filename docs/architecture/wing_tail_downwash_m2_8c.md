# M2.8C deterministic wing-to-tail downwash

M2.8C adds the first explicit aerodynamic interaction between finite-wing
surfaces. It is a one-way wake coupling from an upstream source surface to a
downstream target surface and is evaluated from every real RK4 stage state.

## Relation to M2.8B

M2.8B solves each finite-wing surface's self-induced angle with the existing
deterministic 40-iteration bisection. That angle changes coefficient-sampling
alpha and adds induced drag, but it does not rotate physical section flow.

M2.8C has a distinct role. For one configured interaction, the source's M2.8B
solution supplies `alpha_i_source`, and the target downwash angle is exactly:

```text
epsilon = downwash_factor * alpha_i_source
```

There is no hidden factor of two. A future evidence-backed model may author a
factor near two, but M2.8C neither assumes nor invents it.

## Sign and physical-flow convention

For positive source lift, `alpha_i_source > 0`. A non-negative authored factor
therefore produces `epsilon > 0`, and the target flow rotation obeys:

```text
alpha_geom_target_downwashed = alpha_geom_target_undisturbed - epsilon
```

The target element-frame airflow vector is rotated about local positive span
axis `+Y` in the X-Z section plane:

```text
u' =  cos(epsilon) * u + sin(epsilon) * w
w' = -sin(epsilon) * u + cos(epsilon) * w
```

This is a physical directional rotation. Lift and drag directions, not only
polar lookup, use the rotated flow.

## Target finite-wing composition

If the target is itself finite-wing, its M2.8B solve runs after the physical
downwash rotation and uses the downwashed member kinematics. The canonical
composition is:

```text
alpha_geom_target = angle of downwashed physical flow
alpha_sample_target = alpha_geom_target - alpha_i_target
```

The source solution always uses undisturbed source flow. Target forces cannot
feed back into it, and source induced angle, lift, wrench, and induced drag are
unchanged by the existence of a downstream interaction.

## Reynolds semantics

The rotation is purely directional. It preserves section-plane speed,
spanwise velocity, dynamic pressure, and therefore section Reynolds number.
Both fixed `Polar` and `ReynoldsFamily` bindings use the target's downwashed
physical flow. Reynolds-family interpolation itself is unchanged and continues
to use physical section speed, chord, and authored kinematic viscosity.

## Schema and runtime representation

Aircraft schema v6 adds the required explicit top-level
`aero_downwash_interactions` array. No interaction is inferred. Each entry has
an ID, source surface ID, target surface ID, and non-negative finite factor.

Loading resolves names once into immutable `RuntimeAeroDownwashInteraction`
values containing source and target surface indices. Runtime evaluation uses
deterministic slice traversal and allocates no `Vec`, map, set, or string in an
RK4 stage.

Loading rejects invalid IDs, duplicate IDs, unknown surfaces, self-coupling,
negative or non-finite factors, and multiple interactions targeting one
surface. A surface may not be both a source and a target anywhere in the graph.
This forbids cycles and chained wake propagation. One source may feed multiple
distinct targets; independent interaction declaration order does not affect
the evaluated wrench.

## Compatibility and scope

Schemas v0-v5 retain their previous paths. Schema v6 with an empty interaction
array and a zero-factor interaction preserve the prior finite-wing wrench; the
zero rotation takes a bit-identical fast path. Unassigned quasi-2D elements are
unchanged.

M2.6C qualification uses the same downwash lookup, physical-flow rotation, and
finite-wing solver as runtime before auditing target geometric and sampling
alpha. It therefore continues to inspect the actual runtime operating point.

This slice is not a lifting-line wake solver, does not propagate chained wakes,
does not iterate source and target jointly, and does not introduce a downwash
velocity-magnitude model. It contains only synthetic test data. No Clark Y or
SIG Kadet LT-40 aerodynamic calibration is claimed or supplied.
