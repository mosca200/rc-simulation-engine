# G1E aircraft presentation - moving surfaces

## Scope

Render-only milestone on top of G1C (`ffcc18e`). No physics, aerodynamics,
servo, or mixing changes. Production physics files are untouched.

## Visual binding architecture

- `renderer::surfaces`: `SurfaceId` (left/right aileron, elevator, rudder,
  reserved propeller slot), `SurfaceHinge` (pivot + axis in render-body
  metres, visual gain), `ControlSurfacePresentation` (four deflection
  angles + propeller angle per frame), `SurfaceBindingTable` (the procedural
  fallback), and `GlbArticulationPlan` (explicit primitive-index-to-hinge
  mapping; unmapped primitives stay rigid).
- `renderer::mesh`: `articulated_aircraft_mesh()` and
  `articulated_binding_table()` remain the procedural fallback and its CPU
  test fixture. GLB geometry is never split or rebuilt by the renderer.
- `renderer::pose`: `RenderFrame` carries the root `RenderPose` plus a
  `ControlSurfacePresentation` (`RenderFrame::new` is neutral and rigid).
- `model::PresentationMetadata`: each optional `articulated_surfaces` entry
  names a GLB `visual_primitive_index`, one visual surface slot, the exact
  `control_surface_binding_id`, hinge origin/axis, and visual gain. Binding
  IDs are resolved exactly and checked against the required actuator. There
  is no substring, node-name, or declaration-order inference. Missing
  metadata means every GLB part remains rigid.
- `app::render_snapshot`: follows only those validated presentation entries
  to read the corresponding production control-surface binding and committed
  servo position. Presentation metadata remains outside the physics
  fingerprint.

## Servo state to renderer

`PilotInput -> servo/control system -> ControlSurfacePositions` (committed
servo angles in `AircraftSnapshot`) `-> deflection_gain * (servo - neutral)`
for the explicitly referenced physics binding `-> [f32; 4]` visual
deflections in `AircraftRenderSnapshot::post_step` `->
RenderFrame::with_surfaces` per render frame. No keyboard state is consulted;
no second control system exists. Pose still interpolates in `f64`;
deflections use the current committed servo state.

## Surface transform implementation

Per surface: `local = T(pivot) * R(axis, gain * deflection) * T(-pivot)`
(Rodrigues, right-handed); zero deflection is exactly identity. A mapped GLB
primitive is removed from the root-only batch list and uploaded unchanged as
an articulated batch. Its draw matrix is `root * local`; an unmapped GLB
primitive receives exactly `root`. Both paths reuse the primitive's original
vertex/index data and material index, including its base-color texture.

The GLB is parsed once and all vertex/index buffers, textures, samplers,
pipelines, object buffers, and bind groups are created once during renderer
initialization. Per frame the renderer only updates the persistent root and
articulated object uniforms with `queue.write_buffer`. There is no per-frame
GLB parsing, mesh construction, pipeline construction, or bind-group creation.

The current acro placeholder GLB contains one combined primitive, so its
model intentionally has no articulation entries and remains rigid. An authored
multi-primitive GLB can opt in without changing physics or requiring runtime
vertex-subset hacks.

## Presentation metadata example

```json
"presentation": {
  "glb_path": "aircraft.glb",
  "articulated_surfaces": [
    {
      "visual_primitive_index": 3,
      "surface": "elevator",
      "control_surface_binding_id": "pitch-surface-binding",
      "hinge_origin_render_body_m": [0.0, 0.0, 0.65],
      "hinge_axis_render_body": [1.0, 0.0, 0.0],
      "visual_gain": 1.0
    }
  ]
}
```

The primitive index is the stable order exposed by `GlbAsset::primitives`.
An entry is accepted only when its binding exists and its surface agrees with
the binding actuator. Duplicate and out-of-range primitive mappings fail
closed. Metadata absence never triggers guessing.

## Propeller readiness

`SurfaceId::Propeller` plus `propeller_angle_rad` remain reserved, but no
propeller batch is uploaded or drawn in G1E and no RPM coupling is wired.

## Tests

- `renderer/tests/moving_surfaces_g1e.rs` (10 tests): neutral identity,
  elevator opposition, rudder hinge, differential ailerons, root/local
  composition, explicit mapping of all four GLB slots, unmapped-rigid
  behavior, invalid primitive rejection, hinge validation, and determinism.
- Renderer unit coverage proves switching a primitive to articulated changes
  only its transform target and retains the same material index.
- App tests use opaque binding IDs through the real servo pipeline (600
  steps): roll gives opposite aileron deflections, pitch moves elevator, yaw
  moves rudder, neutral is approximately zero, and metadata changes do not
  affect the fingerprint.
- Model validation tests cover exact binding resolution, actuator
  compatibility, duplicate primitive rejection, finite hinges, and
  fingerprint exclusion.
- Existing renderer/model suites cover GLB textures, multi-material assets,
  root pose, terrain, atmosphere, and deterministic physics boundaries.
