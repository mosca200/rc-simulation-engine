# S8A Aircraft Replay and Deterministic Regression

## Scope and separation

S8A adds an input-based deterministic replay for `AircraftSimulation`. It is separate from the
foundation `Simulation` replay: `ReplayRecording`, `ReplayRecorder`, `ReplayPlayer`,
`ReplayFrame`, `ReplayError`, and `SimulationFingerprint` retain their existing meaning and
format. Aircraft-specific APIs are prefixed with `AircraftReplay` and use an independent schema
version.

The replay crate remains headless. It depends on `aircraft`, `model`, `sim_core`, and `sim_math`,
but not on `renderer`, `wgpu`, or `winit`. The `rcsim-app replay` routing never enters the render
application or creates an event loop, window, surface, adapter, device, or queue.

## Schema version 1

`AIRCRAFT_REPLAY_SCHEMA_VERSION` is `1`. A recording contains:

- `schema_version`;
- `model_id`;
- the model's 32-byte `model_physics_fingerprint`, encoded as exactly 64 lowercase hexadecimal
  characters;
- `simulation_config` with `dt_s`, NED gravity, air density, and NED wind velocity;
- `initial_rigid_body_state` with NED position and velocity, Hamilton body-to-world quaternion,
  and FRD angular velocity;
- an ordered array of input frames.

The model ID and physics fingerprint are checked independently. An ID mismatch returns
`ModelIdMismatch`; a change to deterministic model physics returns
`ModelPhysicsFingerprintMismatch`.

All `f64` values are serialized as JSON numbers. The replay crate enables serde_json's
`float_roundtrip` parser so serialization and deserialization preserve the exact IEEE-754 bit
pattern for every finite value. Runtime conversion validates the configuration, atmosphere,
initial state, and pilot input without clamping.

## Strict JSON

Every serialized S8A object uses a dedicated DTO with `deny_unknown_fields`: top-level recording,
simulation configuration, initial state, frame, and pilot input. Hashes and fingerprints are
strict scalar hexadecimal strings. Loading rejects unknown fields, unsupported schema versions,
non-finite or invalid values, malformed hashes, malformed fingerprints, and non-contiguous frame
indices.

Foundation replay serialization is not changed by S8A.

## Frame semantics

Frames have frozen pre-step semantics. Frame `N` is valid only while
`simulation.step_index() == N`:

```text
frame.step_index = N
frame.pilot_input
        |
        v
AircraftSimulation::step
        |
        v
AircraftSnapshot.step_index = N + 1
frame.expected_snapshot_hash = hash(post-step snapshot N + 1)
```

`AircraftReplayRecorder::record(simulation, N, input)` validates the simulation identity,
configuration, pre-step index, and initial state, performs exactly one simulation step itself, and
stores the hash of the returned post-step snapshot. This prevents callers from pairing an input
with a snapshot from another step while allowing an application to own simulation and recorder as
separate state fields.

`AircraftReplayPlayer` checks schema, step zero, model ID, model fingerprint, exact configuration,
exact initial rigid-body state, and frame continuity before playback. It then compares every
post-step hash and stops on the first difference. The divergence error reports pre-step frame
index, post-step snapshot index, expected hash, and actual hash.

## Canonical aircraft snapshot hash

`AircraftSnapshotHash` is exactly 32 BLAKE3 bytes. The domain separator is:

```text
rcsim:aircraft-snapshot:v1
```

The canonical byte stream is appended in this order:

1. `step_index`;
2. `sim_time_s`;
3. rigid-body position `x, y, z`;
4. rigid-body linear velocity `x, y, z`;
5. active Hamilton body-to-world quaternion `w, x, y, z`;
6. FRD angular velocity `x, y, z`;
7. aileron angle;
8. elevator angle;
9. rudder angle;
10. throttle.

`u64` values use little-endian bytes. Every `f64` uses `to_bits()` followed by little-endian
`u64` bytes. The implementation never hashes JSON, `Debug` output, formatted numbers, raw memory,
or Rust struct layout. Diagnostic and JSON hashes use 64-character lowercase hexadecimal.

## Input-based replay

Each frame contains only:

```text
step_index
pilot_input
expected_snapshot_hash
```

The file does not store a post-step snapshot, position history, velocity history, orientation
history, servo trajectory, or another playback trajectory. During verification the physics
recalculates the entire flight. Expected hashes are regression oracles, not state used to drive
the simulation.

## Versioned regression dataset

The committed dataset is:

```text
tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json
```

It was generated through `rcsim-app replay record` from the real
`models/acro_electric_01/model.json`, using 500 Hz, the standard aircraft-headless initial state,
constant neutral axes, throttle `0.55`, and 2,000 input frames. Automated CPU tests load this file,
reload the actual aircraft model, and compare every post-step hash.

## Headless CLI

Record a replay:

```powershell
cargo run -p rcsim-app --release -- replay record --model models/acro_electric_01/model.json --output target/s8a_smoke.json --steps 32
```

The optional constant-input flags are `--roll`, `--pitch`, `--yaw`, `--throttle`, and
`--physics-hz`. Normalized inputs are validated before `PilotInput` construction, so replay CLI
loading never silently clamps invalid data.

Verify a replay:

```powershell
cargo run -p rcsim-app --release -- replay verify --model models/acro_electric_01/model.json --input target/s8a_smoke.json
```

Verification succeeds only after every frame matches. The command returns a non-zero error at the
first setup mismatch or step divergence.

## S8A exclusions

S8A does not add visual playback, timeline UI, scrub, pause/rewind, replay interpolation, video,
binary/compressed replay, migration, telemetry persistence, GLB loading, flight-controller input,
networking, terrain, collision, OpenXR, VR, S8B, or S9 functionality. It does not modify physics
equations, aircraft snapshot semantics, the renderer, or the 500 Hz default.
