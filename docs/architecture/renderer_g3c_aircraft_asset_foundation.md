# G3C-A production aircraft asset foundation

G3C-A replaces the Acro Electric 01 presentation placeholder without changing the physical model or
renderer architecture. The checked-in GLB is generated deterministically by
`tools/generate_acro_electric_01_glb.ps1`; geometry is authored directly in render-local metres with
`+X` right, `+Y` up, and `-Z` forward.

The existing contracts remain authoritative:

- `model.json.presentation.glb_path` selects the visual asset and remains outside the physics
  fingerprint.
- The production GLB loader preserves indexed triangle primitives and metallic-roughness material
  factors. It does not apply node transforms, so the generator bakes every vertex position.
- G1E binds primitive indices 6, 7, 9, and 11 to the existing left/right aileron, elevator, and
  rudder control bindings. Rigid primitives keep the aircraft root transform; articulated
  primitives receive `root * hinge_transform`.
- The renderer consumes simulation snapshots read-only. No geometry-derived value flows back to
  aerodynamics, mass properties, collision, ground contacts, propulsion, or controls.

The source is organized by semantic part and uses reusable loft, airfoil-panel, strut, and wheel
builders. That preserves a clean path to separately generated LOD assets later, while G3C-A ships
only the active visual replacement. Continuous propeller animation, transparent materials, texture
authoring, renderer LOD selection, and artist-final detailing are intentionally deferred.
