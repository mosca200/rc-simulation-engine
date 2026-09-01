# M2.2D — SIG KADET LT-40 EGV Mass Properties Evidence & Deterministic Derivation

## Scope and authority

M2.2D provides a strict, auditable, off-runtime evidence and derivation layer for the mass
properties of one identified SIG KADET LT-40 EGV operational configuration. It accepts an
unmeasured campaign, preserves unknowns, evaluates independent direct and component-build-up
paths, and reports readiness blockers.

It does not create `models/sig_kadet_lt40_egv/model.json`, construct `RigidBodyParams`, select an
operational mass, or change `AircraftModel`. The module is separate from `AircraftModelLoader`, the
physics fingerprint, and the 500 Hz step. Every M2.2D evaluation reports `runtime_ready = false`.

The machine-readable template is
[`data/sig_kadet_lt40_egv_mass_properties_v0.json`](data/sig_kadet_lt40_egv_mass_properties_v0.json).
Its schema is `reference_aircraft_mass_properties_v0` and its artifact kind is
`mass_properties_evidence_not_runtime_configuration`. All committed operational observations are
`null`; the file contains no inferred LT-40 mass, CG, component position, or inertia.

## Campaign and operational-configuration identity

A campaign records its stable ID, physical/synthetic classification, manufacturer, family, exact
variant, physical airframe ID, measurement date, linked M2.2C geometry campaign ID, and notes. The
operational configuration separately records:

- a stable configuration ID;
- battery configuration ID when applicable;
- propulsion configuration description;
- landing-gear configuration; and
- installed-equipment/configuration notes.

Every installed component is bound to the campaign configuration ID. Changing that ID without
updating the installed inventory fails validation rather than silently reusing evidence. Components
marked `reference_only` are retained as documentary comparisons and never participate in mass,
CG, inertia, or readiness. Thus the historical Himax manufacturer mass cannot become installed
aircraft mass unless a future campaign independently identifies and measures an installed motor.

## Survey frame and the M2.2C bridge

The mass-properties survey frame is right-handed and parallel to simulator body FRD:

- `+X` forward;
- `+Y` right;
- `+Z` down.

The preferred origin is the physical wing-root-leading-edge centre-plane datum used by M2.2C.
The artifact records its origin definition, all three positive directions, establishment status of
the longitudinal origin, lateral datum, and vertical datum, plus source and photograph references.
Three-dimensional CG or inertia cannot close until all three datums are evidenced.

M2.2C longitudinal stations use `x_aft > 0` aft. M2.2D uses `x_FRD > 0` forward. For the same
physical origin, the only deterministic bridge is:

```text
x_FRD = -x_aft
```

The loader exposes this conversion explicitly and rejects non-finite input. No unresolved M2.2C
station is copied into the mass artifact.

## Raw measurement series

Every scalar observation contains exactly three `f64` SI readings, positive instrument
resolution, nonnegative stated uncertainty, a datum or method definition, optional notes, and
source/photo references. A present series is validated even when the campaign remains incomplete.
An observation used to close a path must have both provenance and photograph evidence.

For readings `r1`, `r2`, and `r3`, the deterministic aggregate reports mean, minimum, maximum, and
range. Effective uncertainty follows M2.2C:

```text
u = max(stated_uncertainty, instrument_resolution / 2, reading_range / 2)
```

This is an auditable engineering rule, not a population-statistics confidence interval.

## Direct whole-aircraft evidence

The direct path supports nullable observations for:

- total operational mass;
- operational CG `x`, `y`, and `z` in the evidenced FRD survey frame; and
- the whole-aircraft inertia tensor about that operational CG, with axes parallel to FRD.

Every inertia observation records a method class—physical pendulum, bifilar or trifilar
suspension, evidenced CAD/mass model, or another documented method—plus its method definition and
evidence. No method result exists until its measurement entries and evidence exist.

## Tensor convention and validation

The stored and derived matrix convention matches the matrix consumed by `RigidBodyParams`:

```text
I_FRD = [ Ixx  Ixy  Ixz ]
        [ Iyx  Iyy  Iyz ]
        [ Izx  Izy  Izz ]
```

The off-diagonal fields are matrix entries, not unsigned classical product integrals. For example,
the parallel-axis contribution to `Ixy` is `-m rx ry`; no hidden sign conversion occurs.

The JSON records all nine matrix entries so symmetry can be independently checked. Validation
requires `Ixy = Iyx`, `Ixz = Izx`, and `Iyz = Izy` within a fixed relative tolerance, leaving the
six independent symmetric terms `Ixx`, `Iyy`, `Izz`, `Ixy`, `Ixz`, and `Iyz`. Direct and final
build-up tensors must be finite, symmetric, and positive definite. Nonzero products of inertia are
preserved; the result is never forced diagonal.

## Component inventory

Each component or evidenced group has a stable ID, category, description, status, configuration
binding, mass series, three-dimensional component-CG series, optional intrinsic inertia about its
own CG in FRD-parallel axes, evidence references, and notes. Categories are stable author text so a
campaign can represent fuselage/centre structure, separate wings, tail surfaces, motor,
propeller/spinner, ESC, battery, receiver, individual or grouped servos, landing gear, wheels,
wiring/connectors, fasteners, ballast, and remaining equipment without schema churn.

`component_inventory_complete` is an explicit assertion. Missing content never receives zero mass.
A complete build-up also requires at least one installed component and complete evidence for every
installed entry.

## Mass and CG build-up

For the installed components in declaration order:

```text
M = sum(m_i)
r_CG = sum(m_i r_i) / M
```

Mass uncertainty is root-sum-square across independent component mass uncertainties. CG uses the
deterministic first-order derivatives
`d r_CG / d m_i = (r_i - r_CG) / M` and
`d r_CG / d r_i = m_i / M`, combined by root-sum-square per FRD axis. Missing component mass or
position evidence blocks only the build-up path; it is never assigned a default.

## Full-inertia build-up

For each component, with `r = r_i - r_CG`, the evaluator applies:

```text
I_shifted = I_component_CG
          + m_i ((r dot r) Identity - r r^T)

I_aircraft_CG = sum(I_shifted)
```

All six independent terms are accumulated in deterministic component order. A component lacking
intrinsic inertia prevents the authoritative component-inertia result. M2.2D deliberately exposes
no point-mass-only tensor, so such a diagnostic cannot promote `inertia_ready`.

Inertia uncertainty uses deterministic one-input-at-a-time perturbation. Each component mass,
each position coordinate, and each independent intrinsic tensor term is varied by plus and minus
its effective uncertainty. The larger absolute output deviation for each matrix entry is retained,
then all input contributions are combined by root-sum-square. Invalid negative-mass perturbations
are omitted. The output is an engineering sensitivity band, not a statistical interval.

## Direct versus build-up consistency

Direct and build-up results remain separately visible. M2.2D does not silently choose one when
both exist. Nullable acceptance criteria cover:

- absolute total-mass difference;
- Euclidean three-dimensional CG-position difference; and
- Frobenius norm of the full inertia-matrix difference.

When both paths exist, the corresponding criterion is mandatory. A comparison passes when its
difference is no greater than the explicit criterion plus root-sum-square combined uncertainty.
Missing criteria or disagreement create stable readiness blockers. If only one evidenced path is
available, that path may close its individual gate.

## Published flying-weight range

The SIG EGV range of 2.720–2.835 kg is stored only under
`published_weight_range_comparison`, with manufacturer provenance and the explicit authority value
`comparison_only_never_operational_mass`. Direct and component masses receive independent
`within_published_range`, `outside_published_range`, or `unknown` diagnostics.

The range is never averaged, selected, or substituted. It cannot produce operational mass or make
`mass_ready` true.

## Readiness gates

`configuration_identified` requires physical airframe ID, measurement date, linked geometry
campaign, operational configuration ID and descriptions, an FRD-parallel frame, all three
established datums, and frame source/photo evidence.

`mass_ready` requires an evidenced direct total mass or a complete evidenced installed-component
mass inventory. `cg_ready` similarly requires an evidenced direct 3-D CG or complete component
mass/position build-up. `inertia_ready` requires an evidenced, physically valid direct tensor or a
complete component build-up including every intrinsic tensor. If both paths exist, their explicit
consistency check must pass.

`mass_properties_ready` requires all four preceding gates and no failed direct/build-up
consistency. `runtime_ready` is hard-coded false regardless of parsing or evidence completeness.
The evaluation returns stable blocker identifiers, including component IDs where an installed
component lacks required evidence.

## Remaining real-world blockers

The committed campaign is intentionally unmeasured. It still needs an identified physical EGV
airframe and date; a frozen operational configuration including battery, propulsion, landing gear,
ballast, and installed equipment; evidenced longitudinal, lateral, and vertical FRD datums; direct
mass and 3-D CG measurements or a complete measured component inventory; and either an evidenced
whole-aircraft inertia experiment/model or intrinsic inertia evidence for every installed
component. Any simultaneous direct/build-up paths also need acceptance criteria and agreement.

The M2.2C physical geometry survey remains unresolved, and M2.2D does not pretend otherwise. No
runnable or simulation-authoritative SIG KADET LT-40 model exists after this slice.
