# Acro Electric 01 — OA2B ground-capable migration

`models/acro_electric_01/model.json` is the default playable model. OA2B
migrates it from schema v2 to the current schema v8, making all required
current-schema fields explicit: Reynolds-viscosity and polar bindings, empty
surface/downwash/slipstream collections, ESC resistance, and the fixed-table
propeller coefficient source.

The existing airborne mass properties, aerodynamic polars and elements,
control bindings and servo limits, and electric propulsion values are
preserved. The OA2B regression projects the v8 representation back onto its
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
