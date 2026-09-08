# Acro Electric 01

This model is explicitly classified as `synthetic_test`. Its physical values are simplified,
synthetic regression data rather than measurements from a real aircraft. The presentation asset
does not define or modify mass, inertia, aerodynamics, propulsion, contacts, or control physics.

## G3C-B production visual closure

`aircraft.glb` is the active production presentation asset. It replaces the original six-cuboid,
single-primitive placeholder and advances the G3C-A foundation into a production-oriented modern
electric RC aerobatic aircraft.

The 252816-byte GLB contains 5811 vertices and 8270 triangles across 21 primitives. Selective
geometry density is concentrated in the 48-sided progressive fuselage/cowl/spinner lofts, the
40-sided compound canopy, 17-point airfoil sections, curved wheels, and tapered visibly twisted
propeller blades. No interior or hidden subdivision is generated. Eight glTF metallic-roughness
materials use only the existing renderer path.

The high-contrast livery uses pearl white structure, competition-red leading-edge panels and
controls, deep-navy tips/trim, and a broad dark underside treatment. Top and bottom remain distinct
at RC viewing distances without emissive colors or apparent-scale tricks. The opaque low-roughness
blue canopy is intentional because the renderer has no transparency pass. No textures are embedded:
material segmentation gives the useful visual gain without adding UV/runtime complexity.

Local render coordinates are:

- `+X`: aircraft right
- `+Y`: up
- `-Z`: forward / nose

Geometry is baked with a `+0.255 m` visual Y datum offset so the main tire envelope meets the
physical ground-start plane. Presentation hinge origins carry the same offset; physics is unchanged.

Primitive order is an authored runtime contract because G1E maps moving surfaces explicitly by
primitive index: left aileron `6`, right aileron `7`, elevator `9`, and rudder `11`. The original
first 16 primitive slots remain stable; G3C-B appends top red livery `16`, underside navy `17`,
canopy frame `18`, wheel hubs `19`, and propeller tips `20`. Their hinge
origins, axes, physics binding IDs, and visual gains live only under `presentation` in `model.json`.
Neutral transforms are identity; the existing servo-to-presentation path supplies deflections.
The propeller remains separate and ready for a future animation binding, but is static because G1E
reserves the slot without wiring continuous shaft rotation.

Regenerate deterministically from the repository root on PowerShell 5.1 or newer:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/generate_acro_electric_01_glb.ps1
```

The generator emits one named mesh/primitive per authored part, with all positions baked because
the production loader does not apply glTF node transforms. Two consecutive runs must produce the
same SHA-256. This part-based layout is future-LOD-ready, but this slice intentionally adds no unused
LOD files and no runtime LOD selection.

All geometry and materials are original procedural work created for this repository and are covered
by the repository license. Residual limitations are the lack of texture/normal/ORM maps,
cockpit/interior detail, transparent canopy, animated propeller, runtime LOD selection, and future
G3J distance-visibility preservation.
