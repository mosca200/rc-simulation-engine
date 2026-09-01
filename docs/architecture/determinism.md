# Determinism policy

S1–S3 expects bit-identical state evolution when executable/build, target, Rust toolchain, input sequence, initial conditions, and configuration are identical. Simulation time is derived from the integer step index; wall-clock time never enters physics.

The core forbids parallel accumulation, Rayon in a step, order-dependent unordered iteration, unseeded randomness, wall clocks, implicit fast-math, data races, and nondeterministic scheduling. The current solver is single-threaded and uses deterministic iteration order.

Schema-v3 Reynolds aerodynamics follows the same boundary. Families are canonicalized once,
element bindings are resolved to indices, and every RK4 stage samples in stable element order from
its own local velocity. Sampling uses no map iteration, allocation, logging, or committed-state
cache. V3 fingerprints include viscosity, family data, binding semantics, and the interpolation
policy identity; legacy v0/v1/v2 fingerprint streams are unchanged.

Snapshot hashing encodes each scalar explicitly in little-endian form. Floating-point values use their IEEE-754 `to_bits()` representation; quaternion values are encoded in canonical `[w, x, y, z]` order. BLAKE3 hashes only step index, position, linear velocity, orientation, and angular velocity. Diagnostic timings, logging, and renderer state are excluded.

Replay setup additionally records a BLAKE3 simulation fingerprint. Its canonical byte stream includes replay schema version, timestep, gravity, mass, the complete row-major inertia matrix, initial canonical state, and initial step index. Playback recomputes this fingerprint from the reconstructed `Simulation` before exposing its first input and rejects any mismatch.

This is not a promise of bit-identical results between Windows and Linux, CPU architectures, Rust toolchains, math-library implementations, or future SIMD paths. Those require separate qualification.
