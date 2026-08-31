# P1 presentation boundary

P1 closes the first vertical slice's presentation gap without changing simulation semantics. The
simulation remains the sole owner of canonical `f64` state, and the application remains the
composition root between model, input, replay, telemetry, and renderer crates. Despite the historic
filename `presentation_s11.md`, this document describes P1 only.

## GLB boundary and asset resolution

The application reads `AircraftModel::presentation().glb_path()` and resolves that validated
relative path against the directory containing the selected `model.json`. The current working
directory is therefore not used to reinterpret the asset reference. It then asks the renderer's
CPU-only GLB loader for an `AircraftMesh`; the renderer never depends on `model` or another
simulation-domain crate.

The loader uses the Rust `gltf` crate and deliberately supports a small glTF 2.0 GLB subset:
embedded binary buffers, triangle primitives, `POSITION`, required indices, and optional
`COLOR_0`. It combines triangle primitives into one indexed CPU mesh and validates finite vertices,
complete triangles, and in-range indices before any GPU resource is created. Materials, textures,
animation, skins, morph targets, lights, and a general scene graph are outside P1.

If presentation metadata is absent, the S7 procedural aircraft remains available as a debug
fallback. If metadata declares an asset and that file is missing or malformed, startup returns an
explicit path-bearing error; it never silently substitutes the procedural mesh.

`models/acro_electric_01/aircraft.glb` is a checked-in, low-poly presentation placeholder generated
by `tools/generate_placeholder_aircraft_glb.ps1`. It is not final artwork. The local render-mesh
coordinate contract remains:

- `+X`: aircraft right;
- `+Y`: aircraft up;
- `-Z`: aircraft forward/nose.

No camera compensation or additional orientation conversion is applied. The established
NED/FRD-to-render transform remains the single world/body boundary.

## Minimal outdoor scene

The renderer clears each frame to a stable sky-blue color. A flat 1 km square green ground plane is
drawn ten metres below the initial aircraft render origin. It is render-only: there is no collision,
terrain, ground handling, or change to physics. The existing reference grid is 4 cm above the plane
to avoid z-fighting, and the debug axes remain available.

## Simulation-to-render snapshots

`AircraftRenderSnapshot` is app-owned presentation data containing only `step_index`, `sim_time_s`,
world-NED position in `f64`, and the Hamilton world-from-body quaternion in `f64`. It is constructed
from the same committed post-step `AircraftSnapshot` used by replay hashing and telemetry. A
dedicated initial-state adapter creates the step-zero render snapshot without changing the approved
post-step meaning of `AircraftSnapshot`.

The fixed two-slot buffer always retains `previous` and `current`. It starts with the initial
snapshot in both slots, so the aircraft is renderable before the first physics step. After every
500 Hz physics step, the new post-step snapshot is inserted exactly once.

For a frame, interpolation uses:

```text
alpha = clamp(accumulator_remainder / physics_dt, 0, 1)
```

Position is linearly interpolated in `f64`. Orientation uses normalized shortest-path SLERP. A
negative quaternion dot product flips the second quaternion, so `q` and `-q` remain the same
orientation and interpolation follows the shorter arc. Near-identical quaternions use normalized
linear interpolation to avoid numerical instability.

The precision boundary order is mandatory and implemented as:

```text
physics position f64
  -> interpolation f64
  -> render-origin subtraction f64
  -> NED-to-render conversion f64
  -> f32 cast
  -> GPU
```

This preserves small local displacement at large world coordinates.

## Fixed-step, input, replay, and telemetry relationship

Rendering remains variable-rate while the physics timestep remains exactly `0.002 s` (500 Hz).
The S7 integer-duration accumulator chooses only how many fixed steps execute; no frame delta is
ever passed to `AircraftSimulation::step`. Within every selected step, including intermediate steps
of a multi-step render frame, the application performs the existing S8B input sample once, advances
the simulation once, records the S8A replay input/hash when enabled, and inserts the resulting render
snapshot once. No second hardware input sample and no second physics evaluation are introduced.

The canonical relationships remain:

```text
replay frame N: PilotInput N -> post-step snapshot/hash N+1
telemetry frame N+1: PilotInput N + AircraftSnapshot N+1
render snapshot N+1: presentation subset of the same AircraftSnapshot N+1
```

Standalone S9 telemetry generation and analysis are unchanged. The render command does not invent a
second telemetry stream; it consumes presentation data only. CPU regression tests run equal elapsed
time under 60 Hz-like, 144 Hz-like, and mixed render-delta patterns and require identical physics
step counts, sampled input sequences, snapshot hashes, render-snapshot insertion counts, and final
physics state.

## Remaining First Vertical Slice Gaps

- The aircraft GLB is an engineering presentation placeholder, not production art.
- Lighting is represented only by vertex colors and the clear sky; there is no PBR or texture path.
- The ground is a fixed local visual plane, with no terrain, collision, runway, or ground handling.
- There is no automated visual/GPU screenshot regression; CPU asset and boundary tests remain the CI
  gate.
- There is no in-render telemetry overlay or replay timeline UI.
- Camera controls and presentation settings remain intentionally minimal.

These gaps are visible follow-up work. P1 does not implement Phase 2 functionality.
