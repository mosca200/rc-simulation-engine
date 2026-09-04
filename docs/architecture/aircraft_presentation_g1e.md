# G1E aircraft presentation — moving surfaces

## Scope

Render-only milestone on top of G1C (`ffcc18e`). No physics, model-schema,
aerodynamics, servo, or mixing changes. Production physics files are
untouched.

## Visual binding architecture

- `renderer::surfaces`: `SurfaceId` (left/right aileron, elevator, rudder,
  reserved propeller slot), `SurfaceHinge` (pivot + axis in render-body
  metres, visual gain), `ControlSurfacePresentation` (four deflection
  angles + propeller angle per frame), `SurfaceBindingTable` (fixed
  hinge-per-slot table, `None` stays rigid).
- `renderer::mesh`: `articulated_aircraft_mesh()` (rigid airframe + four
  separate synthetic panels) and `articulated_binding_table()` (spanwise-X
  hinges for ailerons/elevator, vertical-Y hinge for rudder, unit gains).
  The merged `aircraft_mesh()` and the production GLB are untouched; no GLB
  is split.
- `renderer::pose`: `RenderFrame` carries the root `RenderPose` plus a
  `ControlSurfacePresentation` (`RenderFrame::new` = neutral, rigid).
- `app::render_snapshot`: explicit binding-ID metadata maps physics
  bindings to visual slots (`aileron-left`, `aileron-right`, `elevator`,
  `rudder` substrings; deterministic actuator-order fallback for unknown
  IDs). Presentation metadata never touches the physics fingerprint (no
  model-schema change at all).

## Servo state to renderer

`PilotInput -> servo/control system -> ControlSurfacePositions` (committed
servo angles in `AircraftSnapshot`) `-> deflection_gain * (servo - neutral)`
per resolved binding `-> [f32; 4]` visual deflections in
`AircraftRenderSnapshot::post_step` `-> RenderFrame::with_surfaces` per
render frame. No keyboard state is consulted; no second control system
exists. Pose still interpolates in `f64`; deflections use the current
committed servo state.

## Surface transform implementation

Per surface: `local = T(pivot) * R(axis, gain * deflection) * T(-pivot)`
(Rodrigues, right-handed); zero deflection is exactly identity. Composed
draw matrix is `root * local` where root is exactly
`frame.aircraft_pose().model_matrix()` — the root never changes. The GPU
path adds one persistent object uniform + bind group per surface batch at
upload time (procedural fallback path only; GLB path keeps rigid batches
and empty overlays). Per frame only `queue.write_buffer` updates run: no
texture/bind-group/pipeline creation, no GLB parsing, no mesh rebuilds.

## Propeller readiness

`SurfaceId::Propeller` slot plus `propeller_angle_rad` exist in the state
and binding table, but no propeller batch is uploaded or drawn in G1E and
no RPM coupling is wired. A later slice can add a rotating node behind the
same seam without renumbering the four surfaces.

## Tests

- `renderer/tests/moving_surfaces_g1e.rs` (7 tests): neutral identity,
  elevator +/- opposition + inverse-product identity, rudder vertical axis
  + pivot fixed, differential ailerons opposite, root*local composition +
  neutral root preservation, unbound-rigid + hinge validation, determinism.
- `app::render_snapshot::presentation_comes_from_simulated_servo_state_not_keyboard`:
  Acro model through the real servo pipeline (600 steps): roll gives
  opposite aileron deflections, pitch moves elevator, yaw moves rudder,
  neutral is ~zero, fingerprint unchanged.
- Existing renderer/model suites: no regressions (see quality gates).
