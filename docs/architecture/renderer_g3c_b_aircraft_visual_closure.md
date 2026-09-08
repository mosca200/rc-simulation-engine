# G3C-B aircraft production visual closure

## Scope and philosophy

G3C-B improves only the presentation asset for Acro Electric 01. It preserves the existing GLB
loader, material pipeline, articulation boundary, render passes, and physical model. Geometry is
spent where it changes silhouette or shading: progressive body lofts, airfoil edges, canopy,
spinner, propeller, and wheels. The source is deterministic PowerShell with no DCC or runtime
dependency.

## Geometry and silhouette

The fuselage uses eleven progressively tapered 48-segment elliptical stations from the blended cowl
to a narrow tail boom. The cowl and six-station ogive spinner form a continuous nose. The compound
40-segment canopy is longer, higher, and more strongly curved than G3C-A. Main-wing and tail panels
use a 17-point rounded-leading-edge/thin-trailing-edge section with additional span stations and
credible taper. Separate ailerons, elevator, and rudder retain a small readable neutral gap.

The propeller remains a separate static primitive but now has six radial stations per blade,
variable chord, tapered tips, and visible pitch depth. Higher-segment toroidal tires, smaller hubs,
and slender gear geometry improve the ground silhouette.

## Materials and livery

Eight existing-path metallic-roughness materials cover pearl composite paint, red and navy paint,
opaque tinted canopy, painted spinner, carbon propeller, gear metal, and tire rubber. Airframe,
canopy, and tire are dielectric; canopy roughness is lower than airframe and tire roughness. There is
no emissive material.

The top uses a pearl base with symmetric red leading panels, red control surfaces, navy tips, canopy
frames, and tail/nose accents. The underside adds a broad navy span treatment, making roll attitude
different from the top without billboards or apparent-scale changes.

No texture is embedded in G3C-B. The current material segmentation produces deterministic,
distance-readable contrast without UV seams, decoded texture memory, or renderer changes. Future
texture, normal, and ORM work remains outside this slice.

## Coordinate and articulation contract

- `+X`: aircraft right
- `+Y`: up
- `-Z`: forward/nose

The generator bakes a uniform `+0.255 m` visual Y datum offset. The same offset is applied only to
the four presentation hinge origins, placing the main tire envelope on the ground-start plane while
leaving every physical field and the root simulation pose unchanged.

The original first 16 primitives keep their G3C-A order. G1E continues to map left aileron `6`,
right aileron `7`, elevator `9`, and rudder `11`; neutral is identity and positive/negative finite
deflections use the existing presentation hinges. Five decorative primitives are append-only:
top-red livery `16`, underside navy `17`, canopy frame `18`, wheel hubs `19`, propeller tips `20`.

## Reproducibility and budget

Run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/generate_acro_electric_01_glb.ps1
```

| Metric | G3C-A | G3C-B | Delta |
|---|---:|---:|---:|
| GLB bytes | 108480 | 252816 | +144336 (+133.1%) |
| Vertices | 2445 | 5811 | +3366 (+137.7%) |
| Triangles | 3230 | 8270 | +5040 (+156.0%) |
| Primitives | 16 | 21 | +5 |
| Materials | 8 | 8 | 0 |
| Texture bytes / decoded memory | 0 / 0 | 0 / 0 | 0 |

The result stays inside the 8k-30k LOD0 guidance and below a 3x geometry/byte increase, with the
extra cost concentrated in visible curvature and orientation markings.

## Validation and limitations

Asset tests cover loading, indices, finite positions, normalized normals, degenerate-triangle ratio,
bounds/scale/directions, component separation, materials, livery placement, articulation, and the
exact physics fingerprint. Two generator runs are compared byte-for-byte during release audit.

Known limitations are opaque canopy, static propeller, no cockpit/interior, no textures or advanced
maps, no runtime LOD selection, and no G3J distance-visibility preservation. The generator stations,
stable semantic split, and append-only decoration are suitable inputs for future LOD assets without
claiming a runtime LOD feature here.
