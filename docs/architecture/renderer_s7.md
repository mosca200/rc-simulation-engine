# S7 minimal wgpu renderer

S7 adds the first desktop visualization of the real headless aircraft simulation. It is a small
vertex-color renderer, not a game engine or a new simulation owner. The rendered aircraft pose is
copied from the latest committed `AircraftSimulation` state; the renderer cannot mutate physics.

The frozen simulation conventions remain SI, `f64`, NED world, FRD body, Hamilton quaternion,
fixed-step RK4, and 500 Hz. Rendering uses `f32`, an East/Up/South right-handed frame, and an
independent VSync-driven window loop.

## Dependency boundary

The relevant dependency direction is:

```text
sim_math -> sim_core -> model -> aircraft -> rcsim-app
                                      renderer -> rcsim-app
```

`renderer` depends only on `bytemuck`, `thiserror`, `wgpu`, and `winit`. It does not depend on
`sim_math`, `sim_core`, `model`, or `aircraft`. Conversely, none of the simulation crates depends on
`renderer`. The application is the only integration boundary and adapts a `RigidBodyState` into
three raw arrays before calling the render-only conversion function.

The existing `rcsim-app` foundation and `rcsim-app aircraft` paths do not construct an event loop,
window, wgpu instance, surface, or adapter. Automated tests exercise CPU-only renderer modules and
do not require a display or GPU.

## The single `f64` to `f32` boundary

`world_ned_pose_to_render` accepts only raw physics scalars:

```text
position_world_ned_m:                 [f64; 3]
orientation_world_from_body_wxyz:     [f64; 4]
render_origin_world_ned_m:             [f64; 3]
```

It has no knowledge of `RigidBodyState` or `AircraftSimulation`. The application performs that
adaptation. Position conversion deliberately executes:

```text
relative_position_ned_m = position_world_ned_m - render_origin_world_ned_m  // f64
relative_position_render_m = C * relative_position_ned_m                    // f64
translation_render_m = cast_to_f32(relative_position_render_m)              // final step
```

This preserves a small local displacement even when absolute world coordinates are too large for
`f32`. S7 selects the aircraft's initial NED position as a fixed render origin. It does not rebase
that origin dynamically.

`RenderPose` contains only an `f32` relative translation and an `f32` active rotation matrix.
`RenderFrame` contains the latest pose by value and no simulation-owned references or diagnostics.

## Frozen NED/FRD to render mapping

The render frame is right-handed:

```text
Render +X = East
Render +Y = Up
Render +Z = South
```

World vectors are mapped as:

```text
NED North +X -> Render -Z
NED East  +Y -> Render +X
NED Down  +Z -> Render -Y

render_x =  ned_y
render_y = -ned_z
render_z = -ned_x
```

The identical basis change is applied to body coordinates:

```text
FRD Forward +X -> render-body -Z
FRD Right   +Y -> render-body +X
FRD Down    +Z -> render-body -Y
```

With row-vector descriptions of basis axes but active matrices acting on column vectors, the basis
matrix is:

```text
C = [ 0  1  0 ]
    [ 0  0 -1 ]
    [-1  0  0 ]
```

The orientation conversion is exactly:

```text
R_render_world_from_render_body = C * R_ned_world_from_frd_body * C^-1
```

`C` is orthonormal, so `C^-1 = transpose(C)`. The quaternion is first expanded as a Hamilton
active `f64` rotation matrix; its four components are never reinterpreted as a render quaternion.
Physical identity becomes render identity, so the procedural mesh nose points along local `-Z`.
A positive 90-degree physical yaw moves the nose from North/render `-Z` to East/render `+X`.

## GPU matrices and depth range

CPU `Mat4` values are explicitly row-major. `matrix_to_wgsl_columns` transposes their scalar access
into four `[f32; 4]` WGSL columns before uploading a `mat4x4<f32>` uniform. No mathematical type's
native memory layout is assumed.

Camera and object matrices use separate uniform buffers and bind groups. The perspective builder
starts with a right-handed OpenGL `[-1, 1]` depth projection and explicitly premultiplies:

```text
z_webgpu = 0.5 * z_opengl + 0.5 * w
```

The resulting near and far depths map to approximately `0` and `1`. All GPU-facing matrices use
`f32` and are checked for finiteness by CPU tests.

## Scene geometry and pipelines

The static placeholder aircraft is assembled from low-poly boxes with local `+X` right, `+Y` up,
and `-Z` forward. Its bounding box is approximately 1.64 m wide and 1.52 m long. It has a blue
fuselage, red nose, differently colored left and right wings, colored tail surfaces, a green
vertical fin, and an orange top marker. It is not loaded from the model's `glb_path` and makes no
calibration claim.

A `TriangleList` pipeline draws the aircraft with CCW front faces, back-face culling, depth writes,
and `Less` comparison. A `LineList` pipeline draws a 500 m by 500 m XZ grid and positive axes with
depth testing but no depth writes. Major grid lines occur every 50 m and secondary lines every
10 m. Axis colors are East `+X` red, Up `+Y` green, and South `+Z` blue.

The grid lies at render Y = -10 m relative to the fixed render origin. It is only a visual
constant-altitude reference: it is not terrain, creates no collision, does not clamp altitude, and
cannot influence physics.

The pass clears to a sky-blue color and uses a recreated `Depth32Float` target after every non-zero
resize. There are no textures, lights, shadows, MSAA requirement, or post-processing.

## Chase camera

The chase camera uses:

```text
distance behind: 6.0 m
height above:    2.5 m
look-ahead:      2.0 m
vertical FOV:   60 degrees
near:            0.05 m
far:          2000.0 m
```

Aircraft local `-Z` supplies the world-space forward direction. The eye is behind that direction
plus a render-world `+Y` offset, and the target is ahead of the aircraft. The preferred camera up
is always render-world `+Y`; a deterministic alternate up axis is used only for the singular case
where the view direction is virtually parallel to world up. Resize updates only the camera aspect
ratio and surface resources. The camera neither reads nor writes any flight input.

## Fixed-step scheduling

The desktop wall clock determines only an integer count of 2 ms physics steps:

```text
accumulator += accepted_frame_delta
while accumulator >= 2 ms and steps < 16:
    AircraftSimulation::step(fixed_input)
    accumulator -= 2 ms
```

The implementation uses integer nanoseconds. Frame delta is capped at 250 ms and at most 16 steps
are recovered per render frame. If whole-step backlog remains, it is explicitly discarded while
the sub-step remainder is retained and a warning records dropped wall-clock time. Physics `dt`
never changes and frame delta is never passed to RK4, controls, servos, aerodynamics, or propulsion.
S7 renders the latest committed state without interpolation.

## Winit and surface lifecycle

Render mode uses `winit 0.30` `ApplicationHandler` and `EventLoop::run_app`. Model loading and
`AircraftSimulation` initialization happen before window creation. Window and GPU surface creation
occur on `resumed`; surface-owning objects are dropped on `suspended` and recreated on the next
resume. `CloseRequested` and Escape exit through `ActiveEventLoop::exit`.

Zero-sized windows are left unconfigured. A later non-zero resize updates surface dimensions,
camera aspect, and the depth texture. The selected surface format is sRGB when available,
presentation uses `AutoVsync`, and the desired maximum frame latency is two.

`wgpu 30` exposes `CurrentSurfaceTexture` rather than the older `wgpu::SurfaceError`. The renderer
normalizes its statuses into an equivalent public `SurfaceError`: lost/outdated trigger surface
reconfiguration, timeout skips the frame with a warning, occlusion quietly skips rendering,
validation/internal GPU errors terminate with context, and an asynchronous device OOM signal exits
cleanly. Adapter and device request errors and empty surface capabilities are returned from
`WgpuRenderer::new`.

## Running and manual visual verification

From the repository root on a desktop machine:

```powershell
cargo run -p rcsim-app --release -- render --model models/acro_electric_01/model.json
```

Optional fixed flight throttle:

```powershell
cargo run -p rcsim-app --release -- render --model models/acro_electric_01/model.json --throttle 0.55
```

The initial state is NED `[0, 0, -100]` m, 18 m/s North, identity attitude, zero body rate, zero
wind, standard density, and 500 Hz. Roll, pitch, and yaw input remain exactly neutral. Throttle is
validated inside `[0, 1]`.

For a manual check, confirm sky, grid, colored axes, recognizable nose/top/wing/tail geometry,
correct initial North direction, chase motion driven by the simulation, clean resize,
minimize/restore, Escape, and absence of unexpected GPU diagnostics.

## Explicit S7 exclusions and determinism

S7 does not load GLB/glTF, textures, PBR materials, physical lighting, shadows, terrain,
collisions, Rapier, runway, landing gear, crash physics, joystick/gamepad/Radiomaster input, mouse or
keyboard flight input, OpenXR/VR, HUD, egui, menus, telemetry UI, aircraft replay, networking,
audio, propwash, model calibration, schema v2, or any S8 component.

Rendering may alter wall-clock pacing and the number of fixed steps attempted before a displayed
frame, but it does not alter a step's input, `dt`, arithmetic, subsystem order, or deterministic
result. S1-S6.1 equations and ownership remain unchanged.
