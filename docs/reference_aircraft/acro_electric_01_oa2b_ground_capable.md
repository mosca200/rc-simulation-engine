# Acro Electric 01 — OA2B ground-capable migration

`models/acro_electric_01/model.json` is the default playable model. OA2B
migrates it from schema v2 to schema v8, making all required v8 fields
explicit: Reynolds-viscosity and polar bindings, empty
surface/downwash/slipstream collections, ESC resistance, and the fixed-table
propeller coefficient source.

The existing airborne mass properties, aerodynamic polars and elements,
control bindings and servo limits, and electric propulsion values are
preserved. The OA2B regression projects the current representation back onto its
v2 airborne shape and checks its fixed BLAKE3 digest. No aerodynamic or
propulsion tuning is part of this slice.

The model now declares a tricycle landing gear as model data: a fixed nose
wheel and symmetric fixed, braked main wheels. The contacts use the existing
ground-demo baseline and enable a level, stationary, weight-supported start
on the normal flat ground plane. There are no model-specific physics paths,
altitude clamps, teleports, or synthetic ground contacts.

`models/acro_electric_ground_demo/model.json` remains present and loadable as
a synthetic/regression fixture. It is no longer the default for `rcsim-app
play`; `play` is an alias for `render` and defaults to Acro Electric 01.

The schema and landing gear legitimately change the model physics fingerprint.
OA2B's model fingerprint is
`522bb2ed72632fe0d62b02b9ff4237dd02bde47eabaae25d568fd8368ccadeb0`.

## OA2C v2 structural contacts

OA2C v2 advances the production model to schema v9. The schema adds an
optional ordered `airframe_contacts` array, separate from `landing_gear`.
Acro Electric 01 authors three body-space points: belly, left wing tip, and
right wing tip. Each point carries its compliant normal and isotropic sliding
friction parameters; stable IDs are diagnostic labels and are excluded from
the physics fingerprint.

At each RK4 stage, structural points are transformed from FRD body space into
NED world space and evaluated against the same ground surface as the wheels.
The unilateral spring/damper and regularized tangential force contribute
`F` and `r x F` to the aircraft wrench. Storage is resolved contiguously at
initialization and evaluation uses fixed diagnostics with no per-step heap
allocation.

The wheel evaluator and its steering, rolling, braking, diagnostics, and
weight-on-wheels meaning are unchanged. All three structural points remain
clear in the normal supported gear stance, so they do not carry aircraft
weight during ordinary ground start, taxi, takeoff, or wheel landing.
The v9 production physics fingerprint is
`07c48378ad0f8de786f0927c1bba206681c4153deb50174bf0c518d6eae5ba73`;
the canonical airborne replay keeps its unchanged state hashes and records
this updated model identity.
