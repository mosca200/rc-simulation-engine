# S9 telemetry architecture

## Architecture

The S9 path is deliberately one-way:

```text
AircraftSimulation
  -> committed AircraftSnapshot
  -> AircraftTelemetryFrame
  -> AircraftTelemetryRecording (preallocated buffer)
  -> JSON Lines capture
  -> headless analyzer / deterministic regression summary
```

Telemetry observes an already committed result. `AircraftTelemetryRecorder::record` receives an
immutable `AircraftSimulation` reference, the exact applied `PilotInput`, and the returned
`AircraftSnapshot`; it cannot advance or mutate the simulation. Buffer growth, JSON serialization,
and file I/O remain outside `AircraftSimulation::step()`. When the number of steps is known, the app
preallocates the recording buffer.

The telemetry crate depends on aircraft and the deterministic core, but not on `renderer`, `wgpu`,
`winit`, `platform`, or `gilrs`. Only `rcsim-app render` may initialize the graphical path.

## Temporal semantics

Every frame is post-step:

```text
PilotInput selected for pre-step N
  -> AircraftSimulation::step(input N)
  -> AircraftSnapshot with step_index N+1
  -> AircraftTelemetryFrame with step_index N+1 and PilotInput N
```

The recorder requires contiguous snapshot indices beginning at one and verifies that the immutable
simulation and supplied snapshot have the same committed step. `sim_time_s` must be bit-equal to
`step_index * physics_dt_s`.

## Coordinates and units

All physical values use SI units. Explicit JSON names preserve conventions:

- world position and velocity are NED;
- body angular velocity is FRD;
- orientation is the active Hamilton body-to-world quaternion in `w, x, y, z` order;
- local altitude is derived only as `-Down` and is never presented as MSL altitude.

Each frame also carries air density and NED wind from the simulation environment. These are observed
configuration values and require no additional physics evaluation.

## Capture format

`AIRCRAFT_TELEMETRY_SCHEMA_VERSION` is `1`. The UTF-8 JSONL file contains:

1. one `aircraft_telemetry_header` line with schema, model identity, model physics fingerprint, and
   fixed physics timestep;
2. zero or more `aircraft_telemetry_frame` lines.

Each frame contains schema/model identity, step and simulation time, the exact `PilotInput`, explicit
NED/FRD rigid-body values, Hamilton `wxyz` quaternion, actuator angles, throttle, environment context,
and optional `physics_step_wall_time_s`.

All structural DTOs use `serde(deny_unknown_fields)`. Decoding rejects unknown schema or fields,
malformed fingerprints, empty model identity, non-contiguous steps, inconsistent time, non-finite
values, invalid input/control ranges, and invalid unit quaternions. Values are never silently clamped
during telemetry decoding. JSON floating-point numbers are human-readable; replay remains responsible
for exact deterministic reconstruction.

A zero-step capture is a valid header-only JSONL file.

## Replay relation

Replay and telemetry have separate responsibilities:

```text
Replay    = deterministic source: configuration, initial state, input sequence, snapshot hashes
Telemetry = observation: readable states, controls, environment, metrics, diagnostics
```

`telemetry from-replay` reconstructs the simulation through S8A, verifies every expected post-step
snapshot hash, and emits a frame only after that step passes. A divergence stops processing before a
telemetry frame is emitted for the failing step. No replay trajectory is stored or inferred.

## Analyzer metrics

The headless analyzer reports frame count, first/last step, simulated duration, speed min/max/mean,
North/East/Down ranges, local altitude range, maximum angular-speed magnitude, maximum absolute
roll/pitch/yaw input, throttle range, aileron/elevator/rudder ranges, and final position, velocity,
quaternion, and angular velocity.

Means use compensated summation. Empty captures produce zero frames and unavailable ranges/final
state rather than invented values.

## Deterministic and non-deterministic data

`DeterministicTelemetrySummary` contains only simulation-derived data. It excludes wall-clock timing,
device identity, GPU information, real timestamps, and filesystem metadata. It is useful as a readable
secondary regression signal, while S8A per-step hashes remain the primary deterministic gate.

Optional `physics_step_wall_time_s` is measured by the app around the call to `step()`. It is marked as
non-deterministic performance data, never enters snapshot hashes or replay validation, and is not used
as the physics timestep. Mean/max timing is kept outside the deterministic summary.

## CLI

The following commands are headless:

```text
rcsim-app telemetry run --model PATH --output PATH --steps N [input options]
rcsim-app telemetry from-replay --model PATH --replay PATH --output PATH
rcsim-app telemetry analyze --input PATH
```

`telemetry run` defaults to 500 Hz and supports roll, pitch, yaw, and throttle values. Output paths are
always explicit; committed regression datasets are never overwritten automatically.

## Open gaps before vertical-slice gate

The current `AircraftSnapshot` canonically exposes rigid-body and control state, but no uniquely
defined post-step physics-diagnostic snapshot. The following real gaps remain:

- total aerodynamic/body force and total body moment;
- propulsion thrust and torque, motor/propeller RPM, electrical power, and detailed propulsion data;
- local/global angle of attack and sideslip, and dynamic pressure.

Those values are evaluated internally for RK4 stage states. S9 does not label one intermediate stage
as the committed post-step diagnostic, does not reevaluate aero/propulsion after the step, and does not
change operation ordering merely to expose them. A future slice must first define canonical diagnostic
timing and carry already-computed values by value without allocation or numerical changes.

## Exclusions

S9 adds no GUI, plotting stack, database, compression, binary format, cloud upload, model tuning,
physics changes, new aerodynamics, terrain, collisions, networking, hardware-specific input, VR, or
S10 functionality. No renderer source is modified.
