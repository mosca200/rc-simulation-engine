# Acro Electric 01

This model is explicitly classified as `synthetic_test`. Its physical values are simplified,
synthetic regression data rather than measurements from a real aircraft. The presentation asset
does not define or modify mass, inertia, aerodynamics, propulsion, contacts, or control physics.

## G3C-A visual foundation

`aircraft.glb` is the active production presentation asset. It replaces the original six-cuboid,
single-primitive placeholder with an original procedural model of a modern electric RC aerobatic
aircraft. This is a production-usable **asset foundation**, not artist-final DCC artwork.

The GLB contains baked geometry for a lofted/tapered fuselage, separate cowl and spinner, static
two-blade propeller, opaque tinted canopy, airfoil-section main wing and tail, separate left/right
ailerons, elevator and rudder, tricycle landing gear, wheels, and a coherent high-contrast
white/red/navy livery. Eight glTF metallic-roughness materials use only features supported by the
current renderer. The canopy is deliberately opaque because the renderer has no transparency pass.

Local render coordinates are:

- `+X`: aircraft right
- `+Y`: up
- `-Z`: forward / nose

Primitive order is an authored runtime contract because G1E maps moving surfaces explicitly by
primitive index: left aileron `6`, right aileron `7`, elevator `9`, and rudder `11`. Their hinge
origins, axes, physics binding IDs, and visual gains live only under `presentation` in `model.json`.
Neutral transforms are identity; the existing servo-to-presentation path supplies deflections.
The propeller remains separate and ready for a future animation binding, but is static because G1E
reserves the slot without wiring continuous shaft rotation.

Regenerate deterministically from the repository root on PowerShell 5.1 or newer:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/generate_acro_electric_01_glb.ps1
```

The generator emits one named mesh/primitive per authored part, with all positions baked because
the production loader does not apply glTF node transforms. This part-based source layout is the LOD
foundation: later LODs can reuse the same silhouette stations, semantic part split, material set,
and articulation indices without adding unused runtime assets in this slice.

All geometry and materials are original procedural work created for this repository and are covered
by the repository license. Residual limitations are the lack of artist-authored texture maps,
cockpit/interior detail, transparent canopy, animated propeller, and runtime LOD selection.
