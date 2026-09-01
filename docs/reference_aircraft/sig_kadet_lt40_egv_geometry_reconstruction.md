# SIG KADET LT-40 EGV geometry reconstruction (M2.2B)

## 1. Scope and readiness boundary

This document reconstructs traceable geometry for the SIG KADET LT-40 family. The target
operational subject remains the **KADET LT-40 EGV ARF**. The measured drawings are original
RC-67 kit drawings, so every detailed plan measurement remains legacy-kit evidence until EGV
dimensional equality is independently established.

This slice is documentation and reference data only. It does not define a runnable aircraft and
does not add aerodynamic coefficients, polars, mass, inertia, control angles, or propulsion
coefficients. The machine-readable companion is
[`data/sig_kadet_lt40_geometry_v0.json`](data/sig_kadet_lt40_geometry_v0.json); its
`artifact_kind` explicitly excludes runtime use.

Quality labels in this document are exactly `manufacturer_spec`,
`measured_from_validated_plan`, `derived`, `estimated`, or `unknown`. No value is `estimated`.
Unknown values are not back-filled from the synthetic `acro_electric_01` model.

## 2. Source hierarchy

The source priority used here is:

1. SIG manufacturer product documentation;
2. the original SIG construction manual;
3. an independently identified original SIG plan sheet;
4. the EGV ARF manufacturer manual;
5. secondary plan scans as measurement aids;
6. community discussion only as corroboration.

The frozen M2.2A registry remains authoritative. The principal sources used in this slice are:

| ID | Classification | Use |
| --- | --- | --- |
| `sig-lt40-kit-product` | manufacturer documentation | 70 in span, 900 in^2 area, 57 in length, incidence, dihedral, control travel, 6 deg down/6 deg right engine offset |
| `sig-lt40-build-manual` | manufacturer documentation | two-sheet plan inventory, full-size plan instructions, construction facts, 27 in stabilizer trailing-edge member |
| `sig-lt40-egv-arf-product` | manufacturer documentation | target-variant span, area, length, and identity |
| `sig-lt40-egv-arf-manual` | manufacturer-authored retrieval copy | EGV CG range, control travel, rudder measurement point, and motor-mount instructions |
| `lt40-community-plan-sheet-1-scan` | `secondary_plan_scan` | fuselage, fin, rudder, and local construction geometry |
| `lt40-community-plan-sheet-2-scan` | `secondary_plan_scan` | wing, aileron, stabilizer, elevator, and dihedral geometry |

The manufacturer pages are the [SIG LT-40 kit product page](https://sigmfg.com/products/sig-lt-40-trainer-kit)
and the [SIG EGV ARF product page](https://sigmfg.com/collections/sig-arfs-almost-ready-to-fly/products/sig-kadet-lt-40-egv-arf?variant=45138130441).
The plan scans were retrieved from the [AeroBoc community archive](https://aeroboc.webnode.page/plantas/).
The scans were measurement aids, not promoted to manufacturer documentation.

## 3. Plan identification and manufacturer plan facts

The original kit manual lists `PLAN SHEET 1` and `PLAN SHEET 2`. It says the plans are full
scale/actual size and directs the builder to assemble the Wing Panels, Stabilizer, and Fin directly
over them. The explicit exception is the **Wing Front View**, which is drawn half size.

The downloaded one-page raster PDFs identify themselves in their title blocks as:

- `KADET LT-40 RC-67 PLAN SHEET 1 OF 2`, SIG Manufacturing Co., Inc.; and
- `KADET LT-40 RC-67 PLAN SHEET 2 OF 2`, SIG Manufacturing Co., Inc.

Their layout, part names, rib sequence, construction callouts, and sheet numbering agree with the
official manual. That establishes them as scans of the expected original drawings strongly enough
for calibrated measurement. It does not establish the community files as publisher-hosted copies
or grant redistribution rights.

The files were inspected locally but are not committed:

| File | Bytes | SHA-256 |
| --- | ---: | --- |
| `Kadet LT-40 1.pdf` | 4,362,205 | `2c3c95b59012a49e1bd10f86c2890e924f709d38eb508bae353e78ccec8f20d2` |
| `Kadet LT-40 2.pdf` | 3,298,466 | `07feee42c7e394f3c541623f97f391b01fdd1d72e184aa43b4a0bdf9445d7ee5` |

## 4. Scan calibration and measurement method

Both PDFs contain one grayscale JPEG and no usable vector geometry or text layer.

| Check | Sheet 1 | Sheet 2 |
| --- | ---: | ---: |
| PDF page | 2757.31 x 3643.20 pt | 2759.04 x 3636.29 pt |
| Physical page from PDF box | 38.296 x 50.600 in | 38.320 x 50.504 in |
| Embedded raster | 4787 x 6325 px | 4790 x 6313 px |
| Nominal mapping | 125 x 125 px/in | 125 x 125 px/in |

Sheet 2 was calibrated independently along both scan axes:

| Anchor | Observed | Nominal result | Residual | Adopted scale |
| --- | ---: | ---: | ---: | ---: |
| wing root centerline to rounded-tip extreme, expected 35 in | 4377 px | 35.016 in | +0.016 in (+0.0457%) | 125.0571 px/in |
| 27 in stabilizer trailing-edge member | 3357 px | 26.856 in | -0.144 in (-0.5333%) | 124.3333 px/in |

The axis scales differ by about 0.58%, so direct isotropic PDF coordinates are not authoritative.
All Sheet 2 results use the axis-specific correction. Sheet 1's PDF mapping is 125 px/in in both
axes; the drawn 1/2 in fin trailing-edge post spans approximately 65 px including both printed
outlines. Because small member widths are dominated by line thickness, Sheet 1 retains 125 px/in
and uses a larger endpoint uncertainty.

Outer outlines and hinge axes were traced at native raster resolution. Areas and centroids use
shoelace polygon integration. MAC and quarter-chord references use spanwise integration of the
reconstructed chord distribution; tapered or rounded surfaces are not replaced by rough
rectangles. Default uncertainties are:

- Sheet 2 linear coordinates: +/-0.08 to +/-0.10 in (about 2.0 to 2.5 mm);
- Sheet 1 linear coordinates: +/-0.08 to +/-0.12 in (about 2.0 to 3.0 mm);
- wing outline area: +/-5 in^2 (about 0.0032 m^2);
- horizontal-tail area: +/-4 in^2 (about 0.0026 m^2);
- vertical-tail area: +/-2.5 in^2 (about 0.0016 m^2).

Those uncertainty bands cover printed line width, JPEG edge ambiguity, manual vertex selection,
and residual local scan distortion. They do not cover an unverified EGV-versus-kit difference.

## 5. Coordinate system and conversion

The documentary datum is the wing root leading edge in the wing reference plane. Documentary
coordinates are:

- `x_aft`: positive aft from the local leading-edge datum;
- `y`: positive toward the aircraft's right wing;
- `h`: positive upward for the vertical tail.

Horizontal-surface polygon pairs are `[y, x_aft]`. Vertical-surface pairs are `[x_aft, h]`.
These are not raw PDF coordinates.

The simulator body frame is Forward-Right-Down (FRD): +X forward, +Y right, +Z down. For an
eventual runtime model with CG at `x_cg_aft` from the wing leading edge:

```text
X_body = x_cg_aft - x_aft
Y_body = y
Z_body = -h                  (vertical-tail local height only)
```

The operational CG, surface Z positions, and tail root longitudinal positions remain unknown, so
this conversion cannot yet produce complete runtime coordinates.

## 6. Main-wing geometry

Manufacturer values common to the kit and EGV are 70 in (1.778 m) span and 900 in^2
(0.580644 m^2 exact conversion) reference area. The plan confirms a constant-chord center region,
straight leading and trailing-edge lines, and a rounded tip. It therefore **does not confirm an
exact rectangular outline**.

| Datum | Value | Quality | Notes |
| --- | ---: | --- | --- |
| span | 1.778 m | `manufacturer_spec` | kit and EGV agree |
| semi-span | 0.889 m | `derived` | span / 2 |
| measured root outline chord | 0.3307 +/-0.0025 m (13.02 +/-0.10 in) | `measured_from_validated_plan` | leading outer outline to aileron trailing outline |
| conventional trapezoidal tip chord | Unknown | `unknown` | rounded outline closes continuously; no unique station |
| traced gross outline area | 0.5720 +/-0.0032 m^2 (886.6 +/-5 in^2) | `derived` | includes the center/fuselage-intersected reference region |
| fixed area excluding both ailerons | 0.5161 +/-0.0036 m^2 | `derived` | polygon subtraction |
| geometric MAC of traced outline | 0.3260 +/-0.0025 m | `derived` | spanwise chord-squared integration |
| area-weighted quarter-chord aft of wing LE | 0.0820 +/-0.0020 m | `derived` | geometric reference, not an aerodynamic coefficient |
| each panel's area centroid | y = 0.4327 m, x_aft = 0.1634 m | `derived` | mirrored left/right |

The documentary wing centerline is `y = 0`. Across the constant-chord portion, the leading-edge
line is `x_aft = 0` and the gross trailing-edge line is `x_aft = 0.3307 m`; both transition into
the rounded tip captured by the polygon. The aerodynamic reference is the area-weighted
quarter-chord above, not a straight line forced through the rounded tip.

The source polygon is in the JSON artifact. The reference `S/b` chord is 0.326571 m, while the
traced-plan MAC is 0.325952 m. Their difference is -0.000619 m (-0.190%). This agreement does not
turn the rounded wing into a rectangle.

Original-kit incidence is +1.5 deg and dihedral is 3 deg per panel (`manufacturer_spec`). EGV
correspondence for both remains `unknown`.

## 7. Aileron geometry

The plan shows two pre-shaped ailerons, four hinges per aileron, a root-side torque rod, and an
oblique outboard cut matching the rounded tip. The 1-3/8 in torque-rod-hole offset is a construction
location and was not used as aerodynamic span or chord.

For either mirrored aileron:

| Datum | Value | Quality |
| --- | ---: | --- |
| inboard hinge station | 0.0845 +/-0.0020 m from centerline | `measured_from_validated_plan` |
| outboard hinge station | 0.8333 +/-0.0020 m from centerline | `measured_from_validated_plan` |
| hinge span | 0.7489 +/-0.0030 m | `derived` |
| outboard trailing-edge station | 0.8069 +/-0.0020 m | `measured_from_validated_plan` |
| inboard hinge-to-trailing radius | 0.0380 +/-0.0013 m (1.496 +/-0.05 in) | `measured_from_validated_plan` |
| area | 0.0280 +/-0.0013 m^2 (43.33 +/-2 in^2) | `derived` |
| centroid | y = 0.4524 m, x_aft = 0.3116 m | `derived` |

A single scalar tip chord is not reported because the outboard edge is oblique and terminates into
the rounded wing tip. The four polygon vertices, rather than a guessed trapezoid, are authoritative.

## 8. Horizontal-tail geometry

The official manual's 27 in stabilizer trailing-edge member is a construction datum. The calibrated
plan resolves its relationship to the aerodynamic outline: its endpoints align with the complete
stabilizer/elevator span. The reconstructed full span is therefore 0.6858 +/-0.0020 m
(`measured_from_validated_plan`), while the original 27 in stock length remains
`manufacturer_spec`.

The leading edge has a short center plateau/joiner followed by straight swept panels. Polygon
integration gives:

| Datum | Value | Quality |
| --- | ---: | --- |
| total root chord | 0.2529 +/-0.0025 m | `measured_from_validated_plan` |
| total tip chord | 0.1400 +/-0.0025 m | `measured_from_validated_plan` |
| fixed root chord | 0.2021 +/-0.0025 m | `measured_from_validated_plan` |
| fixed tip chord | 0.0892 +/-0.0025 m | `measured_from_validated_plan` |
| gross fixed+moving area | 0.1373 +/-0.0026 m^2 (212.85 +/-4 in^2) | `derived` |
| fixed stabilizer area | 0.1025 +/-0.0026 m^2 | `derived` |
| geometric MAC | 0.2062 +/-0.0030 m | `derived` |
| area-weighted quarter-chord aft of tail root LE | 0.0983 +/-0.0030 m | `derived` |
| gross centroid aft of tail root LE | 0.1498 +/-0.0030 m | `derived` |

Areas include the geometric center region intersected by the fuselage. Original-kit stabilizer
incidence is 0 deg (`manufacturer_spec`); EGV correspondence is `unknown`.

## 9. Elevator geometry

The elevator is continuous across the 27 in span in the plan; no center cutout is shown. Its
pre-shaped stock is labelled 2 in chord, and the calibrated hinge-axis-to-trailing-edge measurement
agrees within printed-line uncertainty.

| Datum | Value | Quality |
| --- | ---: | --- |
| hinge axis aft of tail root LE | 0.2021 +/-0.0025 m | `measured_from_validated_plan` |
| full span | 0.6858 +/-0.0020 m | `measured_from_validated_plan` |
| chord | 0.0508 +/-0.0013 m | `measured_from_validated_plan` |
| movable area | 0.03484 +/-0.0013 m^2 (54.0 +/-2 in^2) | `derived` |
| centroid aft of tail root LE | 0.2275 +/-0.0030 m | `derived` |

## 10. Vertical-fin geometry

The Sheet 1 fin outline is a straight-edged tapered polygon. Values exclude any area hidden below
the drawn fin root/fuselage intersection.

| Datum | Value | Quality |
| --- | ---: | --- |
| height | 0.2266 +/-0.0020 m | `measured_from_validated_plan` |
| gross root chord including rudder | 0.3192 +/-0.0030 m | `measured_from_validated_plan` |
| gross tip chord including rudder | 0.1542 +/-0.0030 m | `measured_from_validated_plan` |
| fixed root chord | 0.2430 +/-0.0030 m | `measured_from_validated_plan` |
| fixed tip chord | 0.0904 +/-0.0030 m | `measured_from_validated_plan` |
| gross area | 0.05363 +/-0.0016 m^2 (83.13 +/-2.5 in^2) | `derived` |
| fixed-fin area | 0.03777 +/-0.0016 m^2 | `derived` |
| gross geometric MAC | 0.2463 +/-0.0040 m | `derived` |
| area-weighted quarter-chord aft of fin root LE | 0.1290 +/-0.0040 m | `derived` |
| gross centroid | x_aft = 0.1906 m, h = 0.1001 m | `derived` |

"Aerodynamic center" in this geometry slice means only the geometric quarter-chord reference. No
aerodynamic center is asserted from coefficients or analysis.

## 11. Rudder geometry

| Datum | Value | Quality |
| --- | ---: | --- |
| hinge axis aft of fin root LE | 0.2430 +/-0.0030 m | `measured_from_validated_plan` |
| height | 0.2266 +/-0.0020 m | `measured_from_validated_plan` |
| bottom/widest chord | 0.0762 +/-0.0013 m (3.00 +/-0.05 in) | `measured_from_validated_plan` |
| top chord | 0.0638 +/-0.0013 m | `measured_from_validated_plan` |
| movable area | 0.01586 +/-0.0010 m^2 (24.58 +/-1.5 in^2) | `derived` |
| centroid | x_aft = 0.2781 m, h = 0.1099 m | `derived` |

The EGV manual explicitly says to measure rudder travel at the bottom of the rudder at its widest
point. The plan therefore resolves the measurement radius as 0.0762 +/-0.0013 m.

## 12. Longitudinal datums and tail arms

The wing leading edge is defined as documentary `x_aft = 0`. The traced wing geometric
quarter-chord reference is 0.0820 +/-0.0020 m aft of it.

The EGV manual gives a CG range of 3-1/2 to 4-1/4 in (0.0889 to 0.10795 m) aft of the wing leading
edge. This confirms the original-kit range for the EGV, but it does not select an operational CG.

Sheet 1's fuselage top and side views contain explicit break marks so the 57 in aircraft cannot fit
on the approximately 50.6 in sheet. The omitted longitudinal distances are not dimensioned. It is
therefore invalid to subtract coordinates across the breaks or scale the broken view to overall
length. These values remain `unknown`:

- horizontal-tail root LE relative to wing LE;
- vertical-tail root LE relative to wing LE;
- horizontal-tail and vertical-tail quarter-chord positions in the wing datum;
- wing-to-horizontal-tail and wing-to-vertical-tail aerodynamic reference arms.

Overall aircraft length is retained only as the 57 in/1.447 m manufacturer specification. It is
not used to estimate a tail arm.

## 13. Control-travel geometry

Published EGV and legacy-kit linear travels agree:

| Control | Linear travel | Radius status | Angle status |
| --- | ---: | --- | --- |
| aileron | +/-0.009525 m (+/-3/8 in) | measurement point/radius `unknown` | `unknown` |
| elevator | +/-0.0142875 m (+/-9/16 in) | measurement point/radius `unknown` | `unknown` |
| rudder | +/-0.0254 m (+/-1 in) | 0.0762 +/-0.0013 m resolved at bottom widest point | `unknown` |

Even for the rudder, the manuals do not say whether the ruler reports perpendicular displacement
from neutral, straight endpoint-to-endpoint chord, or arc length. Those models would respectively
give 19.47 deg (`asin(d/R)`), 19.19 deg (`2 asin(d/(2R))`), and 19.10 deg (`d/R`) for the nominal
1 in/3 in values. These are conditional illustrations, not accepted control angles. No angular
travel is therefore defensibly resolved in M2.2B.

## 14. Propulsion and thrust-axis geometry

The original-kit SIG product documentation specifies 6 deg down and 6 deg right engine offset
(`manufacturer_spec`). This is retained as original-kit evidence only.

The EGV manual specifies the motor thrust-washer longitudinal distance, a square adjustable motor
box/firewall assembly, and scribed thrust-line location marks. It does not publish angular down- or
right-thrust values. "Square in the box" does not prove the box's installed axis relative to the
aircraft datum. EGV down-thrust and right-thrust therefore remain `unknown`.

## 15. Numerical consistency checks

| Check | Calculated | Reference | Absolute residual | Percent residual | Interpretation |
| --- | ---: | ---: | ---: | ---: | --- |
| wing semi-span at nominal 125 px/in | 35.016 in | 35.000 in | +0.016 in | +0.0457% | long-axis calibration anchor |
| 27 in stabilizer member at nominal 125 px/in | 26.856 in | 27.000 in | -0.144 in | -0.5333% | short-axis calibration anchor |
| traced rounded wing area | 886.63 in^2 | 900.00 in^2 | -13.37 in^2 | -1.486% | within reference-definition plus trace uncertainty; not forced |
| measured bounding rectangle | 911.92 in^2 | 900.00 in^2 | +11.92 in^2 | +1.324% | shows why neither a pure rectangle nor rounded outline alone should replace the published reference area |
| traced MAC | 12.8328 in | `S/b` = 12.8571 in | -0.0244 in | -0.190% | close despite rounded tip |
| forward CG / measured root chord | 26.88% | manual approximately 27% | -0.12 percentage point | -0.44% of stated percentage | consistent |
| intermediate CG / measured root chord | 29.76% | manual approximately 30% | -0.24 percentage point | -0.80% | consistent |
| rear CG / measured root chord | 32.64% | manual approximately 33% | -0.36 percentage point | -1.09% | consistent |

For 3 deg per panel and a 35 in panel length, the final symmetric tip rise is
`35 sin(3 deg) = 1.8318 in` (0.046527 m). The half-size front-view construction drawing labels a
3-3/4 in joining gauge with one panel flat and the other raised. A nominal 6 deg included angle
predicts `35 sin(6 deg) = 3.6585 in`, leaving +0.0915 in (+2.44%) to the practical gauge. This is
reasonable construction rounding and does not revise the manufacturer 3 deg/panel specification.

No aircraft-length residual is reported: the only plan views containing the needed longitudinal
extent are deliberately broken, so a numerical result would be fabricated.

## 16. Uncertainty and interpretation limits

The numeric uncertainty is plan-scan uncertainty, not manufacturing tolerance. Built aircraft may
differ through sanding, hinge gaps, covering, control bevels, and assembly. More importantly, the
scans describe the original kit. Matching kit and EGV span, reference area, length, CG range, and
control travel do not prove that every local EGV hinge line and tail polygon is identical.

The published 900 in^2 remains the aerodynamic **reference area**. The 886.6 in^2 result is a
calibrated physical outline trace with rounded tips. Neither silently replaces the other.

## 17. Unresolved data

- direct EGV planform/hinge equality for wing, ailerons, stabilizer, elevator, fin, and rudder;
- horizontal and vertical tail longitudinal positions relative to the wing datum;
- wing-to-tail aerodynamic reference arms;
- aileron and elevator travel measurement points/radii;
- the linear-displacement convention required for every control-angle conversion;
- EGV motor down-thrust and right-thrust angles;
- operational CG, complete mass build-up, and inertia tensor;
- aerodynamic and propulsion data.

## 18. Readiness assessment

The reconstruction is suitable evidence input for M2.2D mass-property planning: it supplies local
surface polygons, areas, and centroids that can support component placement once longitudinal
stations and an operational build are measured. It is **not** sufficient to close the M2.2C
geometry gate, complete M2.2D, or create a runtime LT-40 model. Missing tail arms, operational
mass/CG, inertia, control angles, aerodynamics, propulsion coefficients, and EGV-specific
thrust-axis evidence remain hard gates.
