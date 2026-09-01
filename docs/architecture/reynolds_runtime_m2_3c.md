# M2.3C — Stage-Correct Local Reynolds Aerodynamics Integration

## Purpose and boundary

M2.3C connects the generic M2.3B `ReynoldsPolarFamily` primitive to aircraft assembly for synthetic
and future model data. It introduces aircraft-model schema v3, but it does not create a real
SIG KADET LT-40 model, authorize the incomplete M2.3A evidence artifact for runtime use, or change
the existing v0/v1/v2 simulation paths.

Mach interpolation, standard atmosphere, temperature, altitude-dependent viscosity, XFOIL,
propwash, propulsion changes, control-law changes, trim, timestep changes, and renderer behavior
remain outside this slice.

## Local Reynolds equation

Every Reynolds-aware aerodynamic element uses its own quasi-2D section speed and chord:

```text
Re_local = V_section * chord_m / kinematic_viscosity_m2_s
```

`V_section` is the S4 magnitude `sqrt(u² + w²)` after world-to-body transformation, the local
`omega × r` contribution, and body-to-element transformation. Consequently element position,
body angular velocity, and local orientation affect Reynolds through the same kinematic path that
already determines alpha and dynamic pressure.

The pure `calculate_reynolds_number` primitive rejects non-finite or negative speed, non-finite or
non-positive chord and viscosity, and a non-finite result. Zero speed produces zero Reynolds.
At the existing S4 singularity threshold, force and moment remain exactly zero; the zero Reynolds
diagnostic clamps to the first family node without creating a logarithm or a spurious force.

## Viscosity authority

Schema v3 requires the finite, positive field:

```text
aerodynamics.kinematic_viscosity_m2_s
```

The resolved `AircraftModel` owns this value as physics-authoritative configuration. There is no
implicit standard-air viscosity, fallback, temperature conversion, or density-to-viscosity
inference. Legacy v0/v1/v2 models store no viscosity and do not need one.

Air density and wind remain run configuration in `AeroEnvironment`. M2.3C does not reinterpret
density as viscosity and does not add hidden atmospheric coupling.

## Schema v3 and family binding

Schema v3 preserves the v2 classification, reference metadata, controls, propulsion, and
presentation fields. Its `aerodynamics` object adds ordered `polar_families`; each family has a
stable ID and one or more nodes containing a Reynolds number and an existing alpha-sample shape.
Loader resolution constructs validated `PolarTable`, `ReynoldsPolar`, and `ReynoldsPolarFamily`
values before an `AircraftModel` is returned.

Each v3 element contains exactly one tagged `polar_binding`:

```json
{"kind": "polar", "polar_id": "fixed-id"}
```

or:

```json
{"kind": "reynolds_family", "family_id": "family-id"}
```

The tagged, strict object prevents an element from ambiguously declaring both modes. References
are resolved once to compact indices. Runtime iteration performs no string lookup.

## Stage-correct evaluation

`AircraftSimulation` retains the established RK4 architecture. For each of k1, k2, k3, and k4,
the stage callback supplies that stage's `RigidBodyState` to aircraft-wrench aggregation. The
per-element dispatch then:

1. computes local element velocity from the current stage state;
2. computes `V_section` and alpha;
3. computes local Reynolds from that `V_section`, the element chord, and explicit viscosity;
4. samples the resolved family at local Reynolds and alpha; and
5. applies the unchanged S4 force, intrinsic-moment, and lever-arm equations.

Reynolds is therefore never computed once from the committed state and reused across a timestep.
Control positions remain intentionally zero-order-held within the RK4 step, while local velocity,
Reynolds, coefficients, dynamic pressure, and wrench are stage-local.

## Family sampling and diagnostics

M2.3B behavior is unchanged: each adjacent table is sampled at alpha first, then CL/CD/CM are
interpolated linearly in `ln(Re)`. Reynolds values outside the family clamp without extrapolation.

`ReynoldsAeroElementOutput` exposes the local Reynolds and borrowed `ReynoldsPolarSample`, including
the lower and upper nodes, interpolation fraction, and `ReynoldsRangeStatus`. The aircraft-level
`AircraftAeroElementOutput` distinguishes fixed and Reynolds-family evaluations and makes those
diagnostics available to future telemetry or readiness gates. It does not clone a family or its
tables.

## Physics fingerprint and replay

V3 uses a new physics-fingerprint domain. Its canonical stream includes:

- kinematic viscosity;
- the identity of the clamped logarithmic-Reynolds interpolation policy;
- every canonical Reynolds node;
- every alpha, CL, CD, and CM sample in every family;
- every element's fixed-polar or family binding; and
- all previously fingerprinted mass, geometry, controls, propulsion, and assembly semantics.

Changing any of these v3 physics values changes the fingerprint. Stable authoring IDs remain
lookup metadata after they resolve to ordered indices.

The v0 byte stream and the shared v1/v2 byte stream are not extended or reordered, so all legacy
fingerprints remain byte-for-byte unchanged. Aircraft replay already binds recordings to the
model physics fingerprint; a synthetic v3 recording therefore rejects changed viscosity, family
data, or bindings and replays deterministically without a replay-schema change.

## Determinism and allocation behavior

Model loading and family construction may allocate. The hot path uses stable slice order, compact
indices, monomorphized calls, borrowed family nodes, scalar `f64` arithmetic, and stack outputs.
Neither the per-element Reynolds evaluator nor aircraft-wrench aggregation allocates or clones
vectors. Repeated same-build/same-target runs remain bit-identical.

## Remaining limitations

M2.3C supplies generic runtime infrastructure only. The committed LT-40 evidence still lacks an
authorized operating envelope and polar datasets, and remains `runtime_ready = false`. A later
M2.3 slice must close sourced Reynolds/Mach coverage, choose an explicit Mach policy, define
out-of-grid readiness behavior, and only then construct a separately reviewed reference-aircraft
runtime model.
