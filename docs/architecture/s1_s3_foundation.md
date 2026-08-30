# S1–S3 foundation

## Crates and ownership

The acyclic internal dependency graph is `sim_math <- sim_core <- {telemetry, replay} <- app`. `sim_core` has no knowledge of consumers. `Simulation` is the sole mutable owner of the canonical `RigidBodyState`, validated rigid-body parameters, fixed configuration, and integer step index.

## Lifecycle and fixed step

`Simulation::new` validates finite state, positive `dt_s`, gravity, mass, symmetric positive-definite inertia, and quaternion validity. It precomputes inverse inertia. A valid simulation then advances infallibly by exactly one configured fixed step per `step` call. The default is 500 Hz (`0.002 s`). No scheduler or wall clock exists in the core.

Snapshots use post-step semantics: after integration the step index is incremented, and the returned snapshot's time is `step_index * dt_s`. A fixed-capacity, preallocated safe-Rust ring overwrites its oldest entry deterministically.

## Numerical method

Rigid-body derivative evaluation is separate from the dedicated RK4 integrator and from `Simulation`. RK4 stages form raw quaternion increments, normalize each intermediate orientation once when constructing a valid stage state, and normalize the final orientation once. No artificial damping or stabilization is used.

## Replay

Replay is input-based and versioned. A recording stores simulation configuration, a BLAKE3 fingerprint of schema/configuration/mass/inertia/initial state, the complete initial rigid-body state, and ordered `(step_index, PilotInput)` frames. JSON is an intentionally reversible, non-hot-path encoding. Initial-state scalars are encoded as IEEE-754 bit integers so deserialization cannot silently renormalize a quaternion or round an initial value. Playback validates schema, state, contiguous indices, and the reconstructed simulation fingerprint before yielding its first input. State hashes verify same-build/same-target playback.

## Change cost

Coordinate frames, quaternion direction, canonical-state meaning, SI units, post-step snapshot semantics, and input-based replay semantics are high-cost contracts. Criterion, tracing implementation, JSON encoding, CLI parsing, and the integrator's internal organization are deliberately replaceable.

No S4+ aerodynamic, propulsion, rendering, collision, ECS, or concurrency framework is present.
