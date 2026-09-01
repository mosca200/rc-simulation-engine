# M2.3A — SIG KADET LT-40 EGV Clark Y Aerodynamic Evidence Framework

## Scope and authority

M2.3A is a strict, auditable, off-runtime evidence layer for airfoil geometry and aerodynamic
polar families. It records what a source or solver actually provides, validates that evidence, and
reports evidence-level readiness and coverage holes. It does not create or modify a runtime
`PolarTable`, choose an LT-40 operating envelope, interpolate between Reynolds or Mach conditions,
or alter S4 aerodynamic behavior.

The machine-readable artifact is
[`data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json`](data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json).
Its schema is `reference_aircraft_aerodynamic_evidence_v0`; its artifact kind is
`aerodynamic_evidence_not_runtime_configuration`. Parsing never grants runtime authority and every
evaluation reports `runtime_ready = false`.

## Clark Y identity

The target EGV product specification identifies its airfoil as Clark Y. That manufacturer claim is
represented by `sig-lt40-egv-arf-product`. It establishes identity only: it does not provide
coordinates or aerodynamic coefficients.

The coordinate source is the UIUC Airfoil Coordinates Database file `clarky.dat`, published by
Michael S. Selig at the University of Illinois Urbana-Champaign:

- coordinate file: <https://m-selig.ae.illinois.edu/ads/coord_seligFmt/clarky.dat>;
- database index: <https://m-selig.web.engr.illinois.edu/ads/coord_database.html>;
- retrieval date: 2026-09-01;
- retrieved byte count: 2,559;
- exact retrieved-source SHA-256:
  `a30da5120fd7cd95c08541496cfc8607d58ef64f58198e25c799ffe6532f6e4d`.

The source is coordinate evidence only. No UIUC polar is claimed or inferred.

## Coordinate preservation and normalization

The source uses Selig format: one title line followed by normalized `x/c y/c` pairs, starting at the
upper trailing edge, proceeding to the leading edge, and returning along the lower surface to the
trailing edge. The committed JSON preserves all 121 coordinate pairs in source order.

The normalization is syntactic only:

- remove the title line from the numeric array;
- parse each pair as `f64`;
- express source tokens such as `-.0046700` as the numerically identical `-0.00467` JSON number;
- retain source point order and values.

No scaling, smoothing, resampling, interpolation, point insertion, trailing-edge closure, or other
geometry change is performed. The source is already unit-chord normalized. The Clark Y coordinates
contain a single leading-edge point and an open trailing edge; both properties are declared rather
than silently modified.

When coordinates are present, validation requires:

- finite `x/c` and `y/c`;
- normalized bounds, with the leading edge at `x/c = 0` and both trailing-edge endpoints at
  `x/c = 1`;
- strictly decreasing upper-surface `x/c` and strictly increasing lower-surface `x/c`;
- no duplicate coordinate pair;
- an interior, single leading-edge point;
- agreement between the declared open/closed trailing-edge representation and the endpoints;
- a resolved airfoil-database source with publisher, URL, retrieval date, and SHA-256; and
- explicit transformation provenance.

Validation does not repair bad coordinates.

## Polar evidence datasets

`polar_datasets` is an ordered authoring collection of independent datasets. Every dataset has a
stable ID and an explicit evidence class:

- `published` identifies coefficients reported by a traceable external source;
- `generated_solver` identifies coefficients produced by a documented numerical tool run.

Each sample stores `alpha_rad`, `cl`, `cd`, and `cm` together in its dataset. Coefficients never
exist without the dataset's flow conditions, method, and provenance. A dataset records:

- Reynolds and Mach;
- optional density and dynamic or kinematic viscosity;
- transition assumptions, optional Ncrit, and optional upper/lower forced-transition stations;
- a stable method ID;
- solver/tool name, exact version, and command/config when solver-generated;
- convergence status;
- source references; and
- notes.

Published and generated datasets remain distinguishable through the public evaluation API. A
published dataset uses `not_applicable_published` convergence status. A generated dataset uses
`converged`, `unresolved`, or `failed`; it cannot masquerade as published evidence.

## Strict polar validation

Every present dataset must satisfy:

- finite, positive Reynolds;
- finite, nonnegative Mach;
- finite and positive optional density or viscosity values;
- finite, positive optional Ncrit;
- forced-transition locations within `[0, 1]` chord;
- at least two samples;
- finite alpha, CL, CD, and CM;
- strictly increasing alpha; and
- nonnegative CD.

Dataset IDs are unique. The same Reynolds/Mach point may occur more than once only when the
datasets carry distinct explicit method IDs. M2.3A neither extrapolates samples nor invents a stall
model.

## Generated-solver convergence

A generated dataset is evidence-ready only when all of the following are present:

- `convergence_status = converged`;
- solver/tool identity;
- exact tool version;
- exact command or configuration;
- explicit transition assumptions; and
- resolved provenance.

Unresolved and failed runs may remain in the artifact for audit, but produce stable readiness
blockers and cannot satisfy polar or coverage readiness. A method name by itself is not evidence.

## Reynolds/Mach grid and coverage

Evaluation canonicalizes dataset summaries by Reynolds, then Mach, method ID, and dataset ID. This
ordering is deterministic and does not rewrite the source artifact.

An optional `operating_envelope` contains explicitly sourced required Reynolds/Mach points and a
rationale. Coverage is exact evidence-level membership: every required point needs at least one
evidence-ready dataset at the same Reynolds and Mach. Missing points are returned as structured
coverage holes and stable blocker IDs.

The committed artifact deliberately leaves `operating_envelope = null`. Current geometry,
operational speed, and atmosphere evidence do not authorize an LT-40 Reynolds/Mach envelope. M2.3A
does not derive one from generic trainer assumptions and does not interpolate between grid points.

## Readiness gates

The evaluator exposes:

- `airfoil_identity_ready`: the airfoil identity has resolved provenance;
- `coordinates_ready`: coordinate geometry and its traceable source satisfy the coordinate
  contract;
- `polar_evidence_ready`: at least one dataset exists and every committed dataset is
  evidence-ready;
- `coverage_ready`: a sourced envelope exists and has no coverage holes;
- `aerodynamic_evidence_ready`: all four preceding gates are true; and
- `runtime_ready`: always false in M2.3A.

The committed LT-40 artifact has evidenced Clark Y identity and coordinates, but no polar datasets
or operating envelope. Consequently polar, coverage, aggregate, and runtime readiness remain
false. Null means unknown, never zero or a default.

## Runtime boundary

The M2.3A loader is separate from `AircraftModelLoader`, model schema v0/v1/v2, `RuntimePolar`, and
`sim_core::PolarTable`. It never creates an aerodynamic element, changes alpha interpolation or
endpoint clamping, participates in the physics fingerprint, or executes in the 500 Hz step. RK4,
controls, propulsion, replay, renderer, and existing model files remain unchanged.

## Requirements for a future M2.3B

A separately reviewed M2.3B would first need:

- a justified and sourced LT-40 operational speed/atmosphere/chord envelope;
- a reviewed Reynolds/Mach grid derived from that evidence;
- published polars or reproducible, converged solver runs with exact versions and configurations;
- explicit transition policy and convergence acceptance rules;
- evidence coverage across the authorized grid;
- a documented policy for Reynolds/Mach selection or interpolation;
- stall and out-of-grid behavior defined without hidden extrapolation; and
- regression evidence that any new runtime semantics remain deterministic and allocation-free.

M2.3A implements none of those runtime decisions. No runnable SIG KADET LT-40 model exists after
this slice, and M2.4 propulsion is outside its scope.
