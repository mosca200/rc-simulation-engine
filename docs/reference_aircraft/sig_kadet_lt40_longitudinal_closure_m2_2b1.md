# SIG KADET LT-40 longitudinal and cross-variant closure (M2.2B.1)

## 1. Objective

This evidence-closure slice asks whether documentary evidence can locate the horizontal and
vertical tails aft of the wing datum, derive their aerodynamic reference arms, establish whether
the detailed original RC-67 kit geometry applies to the KADET LT-40 EGV ARF, and resolve the EGV
propulsion-axis angles.

The result is a **valid blocked result**. The search produced useful structural constraints and a
more explicit kit/EGV comparison, but no source supplies an absolute dimension across the breaks in
the original fuselage drawing. No tail station, tail arm, or EGV thrust angle is estimated. This
artifact is evidence only and is not runtime authority.

The coordinate convention remains the M2.2B convention: the documentary origin is the wing root
leading edge, `x_aft = 0`, and positive X is aft.

## 2. Unresolved state inherited from M2.2B

M2.2B resolved original-kit local planforms and the following geometric reference locations:

| Quantity | Value | Quality | Applicability |
| --- | ---: | --- | --- |
| wing geometric quarter chord aft of wing root LE | 0.081960 +/- 0.002000 m | `derived` | original-kit plan; common EGV span/area do not prove local equality |
| horizontal-tail area-weighted quarter chord aft of its root LE | 0.098255 +/- 0.003000 m | `derived` | original-kit plan |
| vertical-tail area-weighted quarter chord aft of its root LE | 0.129016 +/- 0.004000 m | `derived` | original-kit plan |

It left these quantities `unknown`:

- horizontal-tail root LE relative to wing root LE;
- vertical-tail root LE relative to wing root LE;
- wing-to-horizontal-tail quarter-chord arm;
- wing-to-vertical-tail quarter-chord arm;
- detailed original-kit-to-EGV planform and station equality;
- EGV down- and right-thrust angles.

The blocker was not inadequate raster scale. The side and top fuselage views on RC-67 plan sheet 1
contain explicit `X-X` and `Y-Y` break marks, and the omitted distances are not dimensioned.

## 3. New sources inspected

The search was focused on manufacturer and plan evidence rather than a broad web crawl.

| Source | Classification | What was inspected | Useful result |
| --- | --- | --- | --- |
| [SIG LT-40 kit product page](https://sigmfg.com/products/sig-lt-40-trainer-kit) | manufacturer documentation | published geometry/setup table and plan statement | confirms 70 in span, 900 in^2 area, 57 in length, CAD-drawn plans, and published 6 deg down/6 deg right setup |
| [SIG RC-67 construction manual](https://cdn.shopify.com/s/files/1/2281/6393/files/sigrc67kadetlt40.pdf?1161818709039042179=) | manufacturer documentation | complete parts list; fuselage steps 74-118; tail construction and installation; propulsion installation | identifies structural relationships and stock lengths, but no dimension bridging wing and tail |
| [SIG LT-40 replacement-parts listing](https://sigmfg.com/products/sig-kadet-lt-40-parts) | manufacturer documentation | RC-67 replacement inventory and `SIGKP267` plan availability | confirms the factory plan remains a distinct replacement item; supplies no part drawing or dimensions |
| validated RC-67 plan-sheet scans from M2.2B | secondary measurement source | complete side/top view, break locations, and tail detail re-inspected | confirms that wing and tail lie on separated drawing segments; overlapping tail outlines do not provide a defensible absolute wing datum |
| [SIG EGV ARF product page](https://sigmfg.com/collections/sig-arfs-almost-ready-to-fly/products/sig-kadet-lt-40-egv-arf?variant=45138130441) | manufacturer documentation | specifications, variant wording, and construction description | calls the EGV the latest ARF rendition, says it was redesigned for electric/glow power, and documents a two-piece tube-joined wing |
| [EGV ARF manufacturer manual retrieval copy](https://manuals.plus/m/ff89ac931f6fcc8e935416d04673ba56b31f1ecf2e08465d1e7912a4482a9812.pdf), `sig-lt40-egv-arf-manual` | manufacturer-authored retrieval copy | complete text plus rendered tail and motor-mount pages | gives assembly alignment constraints and a 4-3/8 in propulsion packaging dimension, but no absolute tail station or thrust angle |
| focused searches for unbroken plans, fuselage-side dimensions, CAD files, and dimensional EGV drawings | discovery only | manufacturer, exact part number, plan archive, and construction-term queries | found no accessible unbroken dimensioned drawing or calibrated EGV orthographic source |

The temporarily downloaded RC-67 scan matched the M2.2B SHA-256 exactly. No plan or manual PDF is
committed by this slice.

## 4. Manufacturer structural constraints

The following distinction is essential: a direct dimension fixes a geometric distance between
defined features; an indirect structural constraint describes assembly topology or alignment but
does not necessarily determine a longitudinal station.

| Evidence | Type | Deterministic consequence | Longitudinal closure value |
| --- | --- | --- | --- |
| wing root LE documentary datum | direct datum | `x_wing_LE = 0` by definition | fixes the origin only |
| original-kit 27 in stabilizer trailing-edge member | direct component dimension | validates local stabilizer construction/calibration | does not locate the stabilizer on the fuselage |
| two 26-5/8 in outer pushrod tubes | direct stock dimension | establishes supplied sleeve length | none: installed overlap, emergence, bend/path, and endpoints are not fixed by the stock length |
| two 38 in inner pushrod tubes | direct stock dimension | establishes supplied inner-pushrod stock | none: the manual explicitly trims each at the servo end after installation |
| FS-F glued to FS-R; tabbed formers F1-F8; FT-R/FB-R skins | indirect structural constraint | defines assembly order and adjacency | none without a part drawing or inter-feature dimensions |
| stabilizer glued to F8 and fuselage-side top edges | indirect structural constraint | identifies its mounting structure | F8 has no recoverable station relative to wing LE across the plan break |
| fin inserted through the fuselage top and glued to fuselage/stabilizer | indirect structural constraint | identifies local attachment and perpendicularity | does not establish its root LE station relative to the wing |
| equal wing-tip-to-fuselage-tail and stab-tip diagonals | indirect alignment constraint | squares wing and stabilizer laterally | equality fixes yaw alignment, not the common longitudinal distance |
| 57 in published aircraft length | whole-aircraft specification | permits only a future consistency check | deliberately not used to infer any tail position |

The side elevation near the tail was also inspected for a relative fin/stabilizer root constraint.
The fin, stabilizer, fuselage skin, F6/F8, and hidden outlines overlap. The drawing does not label a
single shared longitudinal feature from which both aerodynamic root leading edges can be read
without interpretation. No additional numeric constraint is therefore extracted from that overlap.

## 5. EGV evidence

### Tail installation

The EGV manual instructs the builder to:

1. bolt the wing to the fuselage;
2. position the stabilizer on its factory mounting platform;
3. compare the two stab-tip-to-wing-trailing-edge diagonal distances and make them equal;
4. view the aircraft from the front and make the stabilizer level with the wing;
5. glue the fin perpendicular to the stabilizer.

These are symmetry, squareness, and incidence-adjacent assembly checks. The manual gives no value
for either diagonal, no wing-LE-to-stabilizer-LE dimension, and no fuselage station. The factory
platform makes the assembled location repeatable, but its location is not dimensioned in the
manual.

### Wing and fuselage interfaces

The EGV wing has an aluminum tube joiner, a center leading-edge tab engaging a front fuselage-former
cutout, and two M6.5 nylon bolts at the rear. These facts identify physical measurement datums, but
the manual does not dimension them relative to the tail platform.

### Image-based cross-check

The manual photographs and diagrams show the conventional LT-40 high-wing/tail layout and are
qualitatively consistent with the original kit. They are perspective assembly photographs, have no
coplanar scale bars or camera calibration, and do not provide two independent known dimensions in
the measurement plane. The product photographs have the same limitation. They are used only for a
qualitative `CONSISTENT_BUT_NOT_PROVEN` assessment, not for a station or ratio measurement.

## 6. Original-kit evidence

The original manual confirms that plan sheet 1 carries the fuselage and fin/rudder construction,
while plan sheet 2 carries the wing and stabilizer/elevator construction. It directs builders to
build the surfaces over the full-size drawings.

For longitudinal placement, however:

- the main side and top fuselage views are broken between their wing/fuselage and tail segments;
- no printed dimension bridges a break;
- the factory-cut FS-F, FS-R, FD, FT-R, and FB-R parts are not dimensioned in the manual;
- the SIG replacement listing offers the plan but no CAD file or replacement-part drawing;
- the manual's pushrod stock is deliberately fitted and trimmed, so it is not a station chain;
- stabilizer and fin installation references F6/F8 and local outlines, not wing LE.

The strongest original-kit propulsion evidence also has an internal documentary tension. The
current SIG product table publishes 6 deg down and 6 deg right. Construction-manual step 83 tells
the builder, while positioning the engine on F1, that zero side thrust is ideal and slight right
thrust is acceptable. The published setup value is retained as `manufacturer_spec` for the
original kit, but this conflict is another reason not to infer an EGV angle from kit instructions.

## 7. Kit/EGV comparison matrix

The classifications mean exactly: `CONFIRMED_IDENTICAL` is explicit common manufacturer evidence;
`CONSISTENT_BUT_NOT_PROVEN` is supportive but insufficient for numeric inheritance; `DIFFERENT` is
an evidenced construction difference; and `UNKNOWN` lacks a defensible comparison.

| Property | Original kit | EGV ARF | Classification | Consequence |
| --- | --- | --- | --- | --- |
| span | 70 in | 70 in | `CONFIRMED_IDENTICAL` | shared global specification only |
| wing area | 900 in^2 | 900 in^2 | `CONFIRMED_IDENTICAL` | shared reference area only |
| overall length | 57 in | 57 in | `CONFIRMED_IDENTICAL` | cannot prove local stations |
| published CG range | 3-1/2 to 4-1/4 in aft wing LE | same range | `CONFIRMED_IDENTICAL` | shared operational envelope, not selected CG |
| published linear control travel | aileron 3/8 in, elevator 9/16 in, rudder 1 in each way | same values | `CONFIRMED_IDENTICAL` | does not prove hinge geometry or angular travel |
| airfoil | not identified by the inspected kit manufacturer sources | Clark Y | `UNKNOWN` | Clark Y is authoritative for EGV, not proof of kit airfoil identity |
| wing mounting concept | one built-up wing retained by twelve #67 rubber bands | two-piece aluminum-tube wing with leading tab and nylon bolts | `DIFFERENT` | local structure and datums cannot be inherited automatically |
| tail layout | built-up stabilizer/fin installed at fuselage tail | factory-built surfaces glued to tail platform | `CONSISTENT_BUT_NOT_PROVEN` | same arrangement, unproven stations |
| visible planform geometry | detailed full-size original-kit plan | perspective photos/diagrams only | `CONSISTENT_BUT_NOT_PROVEN` | no EGV planform coordinates authorized |
| stabilizer/elevator/rudder arrangement | full-span elevator, conventional fin/rudder | visually same arrangement | `CONSISTENT_BUT_NOT_PROVEN` | qualitative continuity only |
| fuselage proportions | dimensioned only by broken plan plus global length | product/manual perspective views plus global length | `CONSISTENT_BUT_NOT_PROVEN` | no quantitative local inheritance |
| manufacturer design statement | LT-40 trainer design | “latest ARF rendition” and “proven aerodynamic design” | `CONSISTENT_BUT_NOT_PROVEN` | family continuity is not exact geometry equality |
| down/right thrust | product table says 6/6 deg; build-step side-thrust wording differs | no angle published | `UNKNOWN` for cross-variant equality | EGV remains unknown |

The cross-variant conclusion is **partial at the global-specification level and unresolved at the
local-geometry level**.

## 8. Longitudinal constraint network

Let:

- `H` be horizontal-tail root LE aft of wing root LE;
- `V` be vertical-tail root LE aft of wing root LE;
- `x_wq = 0.081960 +/- 0.002000 m`;
- `d_hq = 0.098255 +/- 0.003000 m`;
- `d_vq = 0.129016 +/- 0.004000 m`.

The deterministic original-kit relations are:

```text
x_ht_qc = H + d_hq
x_vt_qc = V + d_vq
l_h     = H + d_hq - x_wq
l_v     = V + d_vq - x_wq
```

Therefore the known symbolic arm offsets are:

```text
l_h = H + 0.016295 +/- 0.0036 m
l_v = V + 0.047056 +/- 0.0045 m
```

The uncertainty values are root-sum-square propagation of the local plan measurements. They do not
include an uncertainty for `H` or `V`, because those variables are not measured at all, and they do
not resolve EGV applicability.

The available evidence supplies no independent equation for `H` or `V`. The constraint system is
rank-deficient and cannot produce an absolute station or arm. The 57 in overall length is excluded
from the network because neither its forward nor aft physical endpoint is an aerodynamic datum and
using it would require a guessed nose/tail decomposition.

## 9. Resolved stations

No new absolute longitudinal station is resolved.

The only resolved longitudinal values remain the local, original-kit reference offsets inherited
from M2.2B and the symbolic equations above. They are useful once an absolute tail-root station is
measured, but they are not tail stations by themselves.

## 10. Unresolved stations

| Station | Result | Quality | Exact missing evidence |
| --- | --- | --- | --- |
| horizontal-tail root LE aft of wing root LE | `null` | `unknown` | dimension bridging the original plan break or direct EGV measurement |
| horizontal-tail quarter chord aft of wing root LE | `null` | `unknown` | root-LE station plus EGV-specific local tail geometry or direct QC measurement |
| vertical-tail root LE aft of wing root LE | `null` | `unknown` | dimension bridging the plan break or direct EGV measurement with a defined fin-root datum |
| vertical-tail quarter chord aft of wing root LE | `null` | `unknown` | root-LE station plus EGV-specific local fin geometry or direct QC measurement |

An unbroken, dimensioned SIG CAD export; deterministic dimensions for the factory-cut fuselage
side; or a calibrated physical airframe survey would close the absolute-station gap. A casual side
photograph would not.

## 11. Tail arms

Both aerodynamic reference arms remain `unknown`:

- horizontal tail arm: `null`, `unknown`;
- vertical tail arm: `null`, `unknown`.

No value is inferred from overall length, generic trainer proportions, the pushrod stock, or the
appearance of the photographs. The original-kit symbolic relationships in section 8 are not
promoted to EGV authority.

## 12. Optional tail-volume sanity checks

No horizontal or vertical tail-volume coefficient is calculated. Both formulas require a resolved
tail arm, and using a typical trainer range to reverse-engineer either arm would violate the
evidence policy.

## 13. Thrust-axis assessment

### Original kit

The current manufacturer product table publishes 6 deg down and 6 deg right
(`manufacturer_spec`). Construction-manual side-thrust wording is recorded as a source conflict,
not averaged or reinterpreted.

### EGV ARF

The EGV manual establishes:

- an adjustable fore/aft firewall inside the plywood electric motor box;
- exactly 4-3/8 in from the back of the box to the motor thrust washer after adjustment;
- a requirement that the adjustable firewall be straight and square in the box;
- laser-scribed lines that locate the thrust-line center on the firewall;
- attachment of the completed box to the fuselage with four bolts.

The 4-3/8 in value controls cowling/propeller packaging, not axis angle. “Square in the box” controls
the internal firewall installation, while the manual does not specify the box's installed angular
relation to the wing chord, stabilizer datum, or fuselage center plane. The scribed lines locate the
shaft center; their 45 deg construction does not encode down/right thrust.

Accordingly, EGV down thrust and right thrust both remain `null`, `unknown`. The original-kit 6/6
values are not inherited.

## 14. Uncertainty and physical measurement path

### Documentary uncertainty

The local plan uncertainties from M2.2B cover scan calibration, raster edge ambiguity, and feature
selection. They do not cover the missing distance across a break. A missing constraint cannot be
represented honestly by a large numerical `+/-` interval. EGV cross-variant uncertainty is likewise
categorical: local applicability remains unproven.

### Minimum physical measurement campaign

Documentary closure failed, so an EGV airframe survey is required before the M2.2C geometry gate
can close the tail-arm requirements. Use a fully assembled airframe in the intended operational
configuration, but perform these geometry measurements before selecting mass or inertia values.

| Item | Protocol |
| --- | --- |
| datum | establish `x_aft = 0` at the wing root LE projected to the fuselage center plane; mark it on both fuselage sides |
| leveling | support the airframe without wing/tail deflection; define the longitudinal reference using the wing root chord line and record the measured wing incidence separately |
| horizontal station | measure parallel to the longitudinal reference from wing root LE to horizontal-tail root LE on left and right sides; also measure directly to a marked 25%-local-chord station if the EGV planform is surveyed |
| vertical station | define the fin root as its aerodynamic root-plane intersection, photograph the definition, then measure wing root LE to fin root LE and to a constructed 25%-local-chord station |
| local geometry | measure EGV stabilizer span, root/tip chords and sweep; fin height, root/tip chords and sweep; do not reuse original-kit offsets unless the measurements agree within stated tolerance |
| incidence | use a calibrated digital inclinometer on the wing root chord and stabilizer chord; record their difference |
| thrust axis | remove propeller, fit a straight mandrel to the motor shaft, and measure side/top angular projections relative to the recorded wing/fuselage datums |
| linear precision | target +/-1 mm for direct station/chord measurements; repeat any feature identification whose edge width exceeds this target and report the larger uncertainty |
| angular precision | target +/-0.1 deg for wing/stabilizer incidence and +/-0.25 deg for thrust-axis projections |
| repetitions | three independent setups and readings per quantity; left/right measurements for symmetric features; report mean, range, instrument resolution, and any asymmetry rather than silently averaging it away |
| photographs | one full side, one full top, close-ups of every datum/endpoint, ruler or scale bar coplanar with each measured feature, camera approximately normal to the measurement plane, and labels for airframe identity, motor mount, battery, and date |

Actual operational CG and battery location should be documented in the same campaign under the
M2.2C measurement contract, but they are not geometry substitutes and are not added here.

## 15. Final readiness assessment

M2.2B.1 improves the evidence boundary and makes the constraint failure reproducible, but it does
not close the longitudinal model:

- original-kit and EGV global span, area, length, CG range, and published control travels are
  confirmed identical;
- the wing mounting construction is demonstrably different;
- local planforms, tail stations, and fuselage proportions remain only consistent, not proven
  identical;
- horizontal and vertical tail stations and arms remain unknown;
- EGV propulsion-axis angles remain unknown;
- tail-volume metrics are intentionally absent.

The geometry dataset remains `runtime_ready: false`. Recommendation: **perform targeted geometry
measurement first (option B)**. Acquiring an unbroken dimensioned SIG drawing (option C) would be an
equivalent documentary closure path, but proceeding directly to the M2.2D mass-properties slice
would leave the aerodynamic placement gate unresolved.
