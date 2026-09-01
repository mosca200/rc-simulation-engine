# SIG KADET LT-40 EGV ARF reference dossier (M2.2A)

## Status and scope

This dossier freezes traceable source data for the first real trainer-class aircraft intended for
simulator physics validation. It is documentation only. It does not define a runnable aircraft,
change the current simulator, or assert that the available data are sufficient for a validated
physical model.

All values use one of the M2.1 quality labels: `manufacturer_spec`, `published`, `measured`,
`derived`, `estimated`, or `unknown`. No values in this dossier are `measured` or `estimated`.
Unknown quantities remain explicitly unknown. Canonical documentary units are SI; original units
and manufacturer rounding are retained so that conversions can be audited.

## Reference identity and variant boundary

| Field | Frozen identity |
| --- | --- |
| Manufacturer | SIG Manufacturing |
| Family | KADET LT-40 |
| Target operational variant | KADET LT-40 EGV ARF |
| Reference purpose | First real trainer-class aircraft used for simulator physics validation |

The original SIG LT-40 kit documentation supplies legacy geometry, construction, CG, and control
travel evidence where stated below. The current EGV ARF documentation supplies the target electric
variant's weight and power-system evidence. A value from the original kit is not automatically an
EGV value. Every cross-variant use is identified, and its applicability must be confirmed during
geometry reconstruction rather than silently assumed.

## Source policy and registry

The evidence priority for this work is:

1. current SIG manufacturer product pages;
2. original SIG instruction manual and plan documentation;
3. component-manufacturer documentation;
4. the UIUC airfoil database and published research;
5. APC manufacturer performance data.

Community plans or forum scans may corroborate later reconstruction, but cannot be promoted to
manufacturer evidence unless their identity is independently established. None are used here.

| Stable ID | Type | Title | Publisher / author | URL | Accessed | Parameters supported | Variant applicability / notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `sig-lt40-kit-product` | `manufacturer_documentation` | SIG LT-40 Trainer Kit | SIG Manufacturing | [product page](https://sigmfg.com/products/sig-lt-40-trainer-kit) | 2026-08-31 | Span, area, length, legacy control travel, incidence, dihedral | Original LT-40 kit; legacy evidence only where explicitly cited |
| `sig-lt40-build-manual` | `manufacturer_documentation` | SIG KADET LT-40 assembly manual | SIG Manufacturing | [official PDF](https://cdn.shopify.com/s/files/1/2281/6393/files/sigrc67kadetlt40.pdf?1161818709039042179=) | 2026-08-31 | Three CG locations and handling guidance; construction geometry including the 27 in stabilizer trailing-edge member | Original LT-40 kit; not silently treated as EGV geometry |
| `sig-lt40-egv-arf-product` | `manufacturer_documentation` | SIG KADET LT-40 EGV ARF | SIG Manufacturing | [product page](https://sigmfg.com/collections/sig-arfs-almost-ready-to-fly/products/sig-kadet-lt-40-egv-arf?variant=45138130441) | 2026-08-31 | EGV identity, Clark Y, span, area, length, flying-weight range, general electric recommendation | Target EGV ARF variant |
| `sig-lt40-egv-arf-manual` | `manufacturer_documentation` | SIG KADET LT-40 EG ARF instruction manual | SIG Manufacturing; retrieval mirror | [retrieval copy](https://manuals.plus/m/ff89ac931f6fcc8e935416d04673ba56b31f1ecf2e08465d1e7912a4482a9812.pdf) | 2026-08-31 | Historically flight-tested motor, ESC, propeller, and battery configurations | EGV ARF; the current SIG product page independently identifies and links its instruction manual, while this URL is a mirror and is not treated as the publisher |
| `uiuc-clark-y` | `airfoil_database` | UIUC Airfoil Coordinates Database, `clarky.dat` | Michael S. Selig, UIUC | [coordinate file](https://m-selig.ae.illinois.edu/ads/coord_seligFmt/clarky.dat) ([database index](https://m-selig.web.engr.illinois.edu/ads/coord_database.html)) | 2026-08-31 | Clark Y coordinates for future analysis | Coordinate source only; no polar is generated in M2.2A |
| `himax-hc3528-1000` | `manufacturer_documentation` | Himax Brushless Outrunner Motor HC3528-1000 operating manual | Maxx Products International, Inc.; retrieval mirror | [manual PDF](https://www.manuallib.com/download/HIMAX-BRUSHLESS-OUTRUNNER-MOTOR-HC3528-1000.PDF) | 2026-08-31 | Motor electrical, dimensional, mass, power, and current ratings | Candidate/reference component; dated 2005 manufacturer manual retrieved from a mirror |
| `apc-11x7e-performance` | `manufacturer_documentation` | APC 11x7E and propeller performance data | APC Propellers | [product](https://www.apcprop.com/product/11x7e/) and [`PER3_11x7E.dat`](https://www.apcprop.com/files/PER3_11x7E.dat) | 2026-08-31 | Propeller identity, diameter, pitch, and manufacturer performance coefficients | Reference component; exact manufacturer dataset ingested as off-runtime M2.4A evidence |

Retrieval mirrors preserve manufacturer-authored documents but are weaker locations than current
publisher-hosted files. Their titles, branding, part identifiers, and contents are recorded rather
than treating the mirror itself as a manufacturer.

## Frozen manufacturer data

### Aircraft geometry and weight

| Parameter | Canonical value | Status | Original manufacturer value | Source | Applicability and conversion note |
| --- | ---: | --- | --- | --- | --- |
| Wingspan | 1.778 m | `derived` | 70 in (`manufacturer_spec`); SIG also publishes 1778 mm | `sig-lt40-egv-arf-product`, corroborated by `sig-lt40-kit-product` | EGV and original product specifications agree; exact inch-to-metre conversion |
| Reference wing area | 0.580644 m^2 | `derived` | 900 in^2 (`manufacturer_spec`); SIG also publishes 58.1 dm^2 | `sig-lt40-egv-arf-product`, corroborated by `sig-lt40-kit-product` | Exact conversion of 900 in^2; 58.1 dm^2 is manufacturer-rounded |
| Aircraft length | 1.447 m | `manufacturer_spec` | 57 in; SIG publishes 1447 mm | `sig-lt40-egv-arf-product`, corroborated by `sig-lt40-kit-product` | Canonical value retains SIG's published metric precision; exact conversion of 57 in is 1.4478 m (`derived`) |
| Airfoil | Clark Y | `manufacturer_spec` | Clark Y | `sig-lt40-egv-arf-product` | Target EGV product specification |
| EGV flying-weight range | 2.720-2.835 kg | `manufacturer_spec` | 6.0-6.25 lb; SIG publishes 2720-2835 g | `sig-lt40-egv-arf-product` | Target EGV ARF; manufacturer-rounded metric range, not a selected simulation mass |

No single mass is selected. The eventual simulation-authoritative mass must come from a documented
operational configuration or a traceable mass build-up.

### Wing derivations

The plan/manual evidence supports a constant-chord, rectangular baseline for documentary
derivations. M2.2B must validate the actual plan geometry before this becomes authoritative
aerodynamic geometry.

Using reference area `S = 0.580644 m^2` and span `b = 1.778 m`:

```text
geometric chord = S / b
                = 0.580644 / 1.778
                = 0.3265714286 m

aspect ratio = b^2 / S
             = 1.778^2 / 0.580644
             = 5.4444444444
```

Both results are `derived`. The chord is a reference geometric chord. It must not be called an
exact aerodynamic mean aerodynamic chord (MAC) until the rectangular-wing assumption has been
formally established. For a true rectangular wing those quantities coincide, but that conditional
statement is not evidence that the detailed EGV wing is exact.

### Incidence and dihedral

Original SIG LT-40 documentation gives these legacy geometry values:

| Parameter | Documentary value | SI conversion | Status | Source | Applicability |
| --- | ---: | ---: | --- | --- | --- |
| Wing incidence | +1.5 deg | +0.0261799388 rad | `manufacturer_spec`; radians `derived` | `sig-lt40-kit-product` | Original LT-40; EGV correspondence unresolved |
| Horizontal-stabilizer incidence | 0 deg | 0 rad | `manufacturer_spec`; radians `derived` | `sig-lt40-kit-product` | Original LT-40; EGV correspondence unresolved |
| Wing dihedral | 3 deg per panel | 0.0523598776 rad per panel | `manufacturer_spec`; radians `derived` | `sig-lt40-kit-product` | Original LT-40; EGV correspondence unresolved |

The derived half-span is `1.778 / 2 = 0.889 m`. Treating that half-span as the panel length gives
the consistency check

```text
vertical tip displacement = 0.889 * sin(3 deg)
                          = 0.0465266651 m
                          ~= 0.04653 m
```

This is `derived`, is not an independently sourced dimension, and must not override a future plan
measurement.

### Centre-of-gravity evidence

The original SIG manual defines CG as distance aft of the wing leading edge and gives three
positions:

| Use | Position | SI conversion | Approximate chord | Status | Source |
| --- | ---: | ---: | ---: | --- | --- |
| Beginner / most stable | 3.5 in | 88.9 mm | 27% | `manufacturer_spec` | `sig-lt40-build-manual` |
| Intermediate | 3.875 in | 98.425 mm | 30% | `manufacturer_spec` | `sig-lt40-build-manual` |
| Rear | 4.25 in | 107.95 mm | 33% | `manufacturer_spec` | `sig-lt40-build-manual` |

SIG describes the forward 3.5 in / 27% position as best for beginners, with maximum stability and
self-correcting behavior. Therefore `88.9 mm aft of the wing leading edge` is the planned **initial
trainer validation CG**. It is a future validation target, not a value added to a runnable physics
model in M2.2A. Applicability to the EGV geometry remains a cross-variant item for M2.2B.

### Published control travel

| Control | Published linear travel | SI conversion | Status | Source | Applicability |
| --- | ---: | ---: | --- | --- | --- |
| Ailerons | +/-3/8 in | +/-9.525 mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 legacy setup; EGV correspondence unresolved |
| Elevator | +/-9/16 in | +/-14.2875 mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 legacy setup; EGV correspondence unresolved |
| Rudder | +/-1 in | +/-25.4 mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 legacy setup; EGV correspondence unresolved |

These are linear displacements, not hinge angles. Linear published travel does not determine an
angular deflection without the measurement radius and exact surface/hinge geometry. Angular
conversion is deferred; no servo limit or control effectiveness is inferred here.

### Horizontal-tail partial evidence

The original build manual specifies a `27 in` (`0.6858 m`, exact `derived` conversion) stabilizer
trailing-edge member. Its status is `manufacturer_spec`, but its meaning is strictly a
**construction datum**. It is not labelled as exact stabilizer span or aerodynamic tail width. The
exact horizontal-tail planform remains unknown pending validated plan reconstruction.

## Airfoil reference

The target EGV product identifies the airfoil as Clark Y (`manufacturer_spec`,
`sig-lt40-egv-arf-product`). M2.3A ingests UIUC's original `clarky.dat` (`published`,
`uiuc-clark-y`) as traceable coordinate evidence. M2.2A does not run XFOIL, generate
Reynolds-dependent polars, alter the aerodynamic solver, or create a coefficient table. Those
activities remain future M2.3 work.

## Electric-propulsion references

### General EGV recommendation

SIG's current EGV product page gives the following target-variant recommendation, all
`manufacturer_spec` from `sig-lt40-egv-arf-product`:

| Item | Recommendation |
| --- | --- |
| Electric power | 500-800 W |
| Brushless motor | 800-1000 rpm/V (Kv) |
| ESC | 50-75 A |
| LiPo battery | 3S-4S, 4000-5000 mAh |

This recommendation is a compatibility envelope, not a complete electrical model. Battery
internal resistance, exact battery mass, voltage sag, and ESC losses are `unknown`.

### Historical EGV manual configuration

The EGV instruction manual separately documents an airplane flown with a Maxx Products
HC3528-1000 motor, 50 A Castle Creations ESC, APC 11x7E propeller, and either a 3S 4500 mAh or 4S
4500 mAh LiPo battery. This is retained as a historically flight-tested manufacturer configuration
from `sig-lt40-egv-arf-manual`; it is not silently generalized into the current simulator or
treated as proof of a unique production setup.

### Himax HC3528-1000 candidate/reference motor

| Parameter | Value | Status | Source | Notes |
| --- | ---: | --- | --- | --- |
| Speed constant, Kv | 1000 rpm/V | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference motor |
| Winding resistance, Rm | 0.020 ohm | `manufacturer_spec` | `himax-hc3528-1000` | Manual notation `Rm` |
| No-load current, Io | 2.6 A | `manufacturer_spec` | `himax-hc3528-1000` | Manual notation `Io` |
| Motor mass | 197 g | `manufacturer_spec` | `himax-hc3528-1000` | Motor only |
| Diameter | 35.2 mm | `manufacturer_spec` | `himax-hc3528-1000` | Motor dimension |
| Length | 54.2 mm | `manufacturer_spec` | `himax-hc3528-1000` | Motor dimension |
| Shaft diameter | 5.0 mm | `manufacturer_spec` | `himax-hc3528-1000` | Motor dimension |
| Published maximum power | 450 W | `manufacturer_spec` | `himax-hc3528-1000` | Manual states dependence on several factors |
| Efficient operating current | 15-48 A | `manufacturer_spec` | `himax-hc3528-1000` | Published operating range |
| Short-duration maximum current | 68 A | `manufacturer_spec` | `himax-hc3528-1000` | Maximum for 15 seconds, with cooling limitations |

The motor's manufacturer maximum power is not reconciled here with SIG's broader 500-800 W
airframe recommendation. They are distinct claims with distinct sources and intended meanings.

### APC 11x7E reference propeller

| Parameter | Value | Status | Source |
| --- | ---: | --- | --- |
| Diameter | 11 in = 0.2794 m | Published size `manufacturer_spec`; SI conversion `derived` | `apc-11x7e-performance` |
| Pitch | 7 in = 0.1778 m | Published size `manufacturer_spec`; SI conversion `derived` | `apc-11x7e-performance` |

APC publishes `PER3_11x7E.dat`, which contains performance coefficients across operating
conditions. M2.4A preserves the exact retrieved manufacturer file and parses its declared columns
as off-runtime evidence; it does not resample or simplify the data into the runtime propeller
table. For coefficient handling, the conventional relationship is

```text
Cq = Cp / (2 * pi)
```

This relationship is recorded as `derived`; no new `Ct` or `Cp` values are produced. M2.4A records
the parser interpretation and derived `Cq`; M2.4B supplies a generic calibratable runtime, but an
LT-40 runtime propeller model remains blocked on installed-configuration evidence.

## Mass properties and planned derivation

The only known mass evidence is the manufacturer EGV flying-weight range of 2.720-2.835 kg. Exact
operational mass, `Ixx`, `Iyy`, `Izz`, and all relevant products of inertia are `unknown`. No
plausible inertia or convenient midpoint mass is introduced.

M2.2D defines a strict evidence and derivation path for traceable component masses and locations,
including at least fuselage, left and right wings, horizontal tail, vertical tail, motor, battery,
ESC, servos, receiver, landing gear, wheels, and remaining structure. Conceptually:

```text
CG = sum(m_i * r_i) / sum(m_i)
```

The implemented off-runtime derivation combines each component's own inertia with its translated
contribution using the parallel-axis theorem. The committed campaign remains unmeasured, so this
method supplies no LT-40 numerical mass properties or runtime authority.

## Parameter matrix

`Unknown` means no numerical substitute is authorized. Original-kit evidence is explicitly marked
where EGV applicability still requires verification.

| Parameter | Value | Units | Status | Source | Variant applicability | Simulation use | Notes |
| --- | ---: | --- | --- | --- | --- | --- | --- |
| Wingspan | 1.778 | m | `derived` | `sig-lt40-egv-arf-product`, `sig-lt40-kit-product` | Both published as 70 in | Future geometry reference | Exact conversion |
| Reference wing area | 0.580644 | m^2 | `derived` | `sig-lt40-egv-arf-product`, `sig-lt40-kit-product` | Both published as 900 in^2 | Future aerodynamic reference | Exact conversion |
| Aircraft length | 1.447 | m | `manufacturer_spec` | `sig-lt40-egv-arf-product`, `sig-lt40-kit-product` | Both product specifications | Future geometry reference | Manufacturer-rounded SI; 57 in converts exactly to 1.4478 m |
| Airfoil identity | Clark Y | - | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | M2.3A evidence input | UIUC coordinates are a separate source |
| Flying-weight range | 2.720-2.835 | kg | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | Evidence only | No authoritative single mass selected |
| Reference geometric chord | 0.3265714286 | m | `derived` | This dossier from span and area | Rectangular baseline | Future geometry check | Not yet an exact aerodynamic MAC |
| Aspect ratio | 5.4444444444 | - | `derived` | This dossier from span and area | Rectangular baseline | Future geometry check | `b^2 / S` |
| Half-span | 0.889 | m | `derived` | This dossier from span | Baseline | Consistency check | `b / 2` |
| Wing incidence | +1.5 | deg | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Future geometry candidate | EGV correspondence unresolved |
| Horizontal-tail incidence | 0 | deg | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Future geometry candidate | EGV correspondence unresolved |
| Wing dihedral | 3 per panel | deg | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Future geometry candidate | EGV correspondence unresolved |
| Dihedral tip displacement | 0.04653 approx. | m | `derived` | This dossier | Baseline consistency check | None authoritative | Assumes half-span equals panel length |
| Forward CG | 88.9 aft of wing LE | mm | `manufacturer_spec` | `sig-lt40-build-manual` | Original LT-40 | Planned initial trainer validation target | 3.5 in, approximately 27% chord |
| Intermediate CG | 98.425 aft of wing LE | mm | `manufacturer_spec` | `sig-lt40-build-manual` | Original LT-40 | Future validation target | 3.875 in, approximately 30% chord |
| Rear CG | 107.95 aft of wing LE | mm | `manufacturer_spec` | `sig-lt40-build-manual` | Original LT-40 | Future validation boundary | 4.25 in, approximately 33% chord |
| Aileron linear travel | +/-9.525 | mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Evidence only | Hinge angle unknown |
| Elevator linear travel | +/-14.2875 | mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Evidence only | Hinge angle unknown |
| Rudder linear travel | +/-25.4 | mm | `manufacturer_spec` | `sig-lt40-kit-product` | Original LT-40 | Evidence only | Hinge angle unknown |
| Stabilizer trailing-edge member | 0.6858 | m | `derived` | `sig-lt40-build-manual` | Original construction datum | Geometry reconstruction evidence | Exact conversion of 27 in; not exact tail span |
| Exact horizontal-tail span | Unknown | m | `unknown` | - | EGV | None | Requires validated plan geometry |
| Horizontal-tail planform area | Unknown | m^2 | `unknown` | - | EGV | None | Requires validated plan geometry |
| Horizontal-tail aerodynamic chord | Unknown | m | `unknown` | - | EGV | None | Requires validated plan geometry |
| Elevator exact chord | Unknown | m | `unknown` | - | EGV | None | No estimate authorized |
| Elevator exact area | Unknown | m^2 | `unknown` | - | EGV | None | No estimate authorized |
| Vertical-fin planform | Unknown | - | `unknown` | - | EGV | None | Requires validated plan geometry |
| Vertical-fin area | Unknown | m^2 | `unknown` | - | EGV | None | No estimate authorized |
| Rudder exact area | Unknown | m^2 | `unknown` | - | EGV | None | No estimate authorized |
| Rudder exact chord | Unknown | m | `unknown` | - | EGV | None | No estimate authorized |
| Wing-to-tail aerodynamic moment arm | Unknown | m | `unknown` | - | EGV | None | Aerodynamic reference points unresolved |
| Exact aileron span | Unknown | m | `unknown` | - | EGV | None | No estimate authorized |
| Exact aileron chord | Unknown | m | `unknown` | - | EGV | None | No estimate authorized |
| Aerodynamic-center positions | Unknown | m | `unknown` | - | EGV | None | Wing and tail reference positions unresolved |
| Detailed fuselage aerodynamic geometry | Unknown | - | `unknown` | - | EGV | None | No body-aero surrogate authorized |
| Exact operational mass | Unknown | kg | `unknown` | - | Specific operational build | None | Must not use range midpoint by convenience |
| `Ixx`, `Iyy`, `Izz` | Unknown | kg m^2 | `unknown` | - | Specific operational build | None | M2.2D mass build-up required |
| Products of inertia | Unknown | kg m^2 | `unknown` | - | Specific operational build | None | M2.2D mass build-up required |
| Motor Kv | 1000 | rpm/V | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | Not installed in a runtime model |
| Motor Rm | 0.020 | ohm | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | - |
| Motor Io | 2.6 | A | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | - |
| Motor mass | 0.197 | kg | `derived` | `himax-hc3528-1000` | Historical candidate/reference | Future M2.2D evidence | Exact conversion from 197 g manufacturer spec |
| Motor diameter | 0.0352 | m | `derived` | `himax-hc3528-1000` | Historical candidate/reference | Future fit/mass evidence | Exact conversion from 35.2 mm |
| Motor length | 0.0542 | m | `derived` | `himax-hc3528-1000` | Historical candidate/reference | Future fit/mass evidence | Exact conversion from 54.2 mm |
| Motor shaft diameter | 0.005 | m | `derived` | `himax-hc3528-1000` | Historical candidate/reference | Future component evidence | Exact conversion from 5 mm |
| Motor maximum power | 450 | W | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | Conditional manufacturer rating |
| Motor efficient current | 15-48 | A | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | - |
| Motor 15-second maximum current | 68 | A | `manufacturer_spec` | `himax-hc3528-1000` | Historical candidate/reference | M2.4A evidence | Short-duration limit |
| Propeller diameter | 0.2794 | m | `derived` | `apc-11x7e-performance` | APC 11x7E reference | M2.4A evidence | Exact conversion of 11 in |
| Propeller pitch | 0.1778 | m | `derived` | `apc-11x7e-performance` | APC 11x7E reference | M2.4A evidence | Exact conversion of 7 in |
| Airframe electric power recommendation | 500-800 | W | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | Compatibility evidence only | Not a motor model |
| Airframe motor-Kv recommendation | 800-1000 | rpm/V | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | Compatibility evidence only | - |
| Airframe ESC recommendation | 50-75 | A | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | Compatibility evidence only | - |
| Airframe battery recommendation | 3S-4S, 4000-5000 | mAh | `manufacturer_spec` | `sig-lt40-egv-arf-product` | EGV | Compatibility evidence only | LiPo |
| Battery internal resistance | Unknown | ohm | `unknown` | - | Specific battery | None | No source selected |
| Exact battery mass | Unknown | kg | `unknown` | - | Specific battery | None | No source selected |
| Battery voltage sag | Unknown | V | `unknown` | - | Specific battery/load | None | Requires validated electrical model |
| ESC losses | Unknown | W | `unknown` | - | Specific ESC/operating point | None | Requires validated electrical model |

## Why no runnable LT-40 model exists in M2.2A

The current AircraftModel schema requires complete runtime mass, inertia, aerodynamics, controls,
and propulsion. Those LT-40 quantities are not yet known. Creating
`models/sig_kadet_lt40_egv/model.json` would therefore require copying synthetic values or inventing
inertia, polars, aerodynamic elements, propulsion coefficients, or angular servo travel. Either
would falsely imply physical validity and violate the unknown-data policy.

Accordingly, no runtime LT-40 model is created. `models/acro_electric_01` remains the unchanged
`synthetic_test` regression aircraft.

## Reference-aircraft roadmap

### M2.2B - LT-40 geometry reconstruction

- Obtain and validate a full-size plan or equivalent authoritative geometry.
- Freeze exact wing, horizontal-tail, vertical-tail, aileron, elevator, and rudder planforms.
- Freeze hinge locations and the radii needed to convert published linear travel into angles.
- Define wing, tail, control-surface, and aerodynamic-element placements against an explicit body
  coordinate system, CG datum, and aerodynamic reference datum.
- Record the measurement method, scale calibration, uncertainty, and source for every reconstructed
  quantity.

### M2.2B.1 - LT-40 longitudinal/cross-variant geometry closure investigation

- Investigate unresolved longitudinal stations and tail arms, and determine the evidence status of
  original-kit geometry for EGV applicability.

### M2.2C - SIG KADET LT-40 EGV Physical Measurement Contract & Geometry Closure Gate

- Deterministic physical-survey ingestion, EGV geometry derivation, cross-variant comparison, and
  an explicit non-runtime closure gate.

### M2.2D - LT-40 mass-properties evidence and derivation

- Component mass build-up, operational mass, CG, and full inertia tensor.

### M2.3A - Aerodynamic evidence preparation

- Traceable Clark Y coordinates plus strict, off-runtime polar evidence and coverage gates.

### M2.3B - Reynolds-aware polar family core primitive

- Generic deterministic `ln(Re)` interpolation across independently gridded alpha polars, without
  aircraft-runtime wiring or Mach interpolation.

### M2.3C - Stage-correct local Reynolds aerodynamics integration

- Generic schema-v3 family bindings, explicit kinematic viscosity, per-RK4-stage local Reynolds,
  diagnostics, and fingerprint/replay coverage using synthetic data only.

### M2.3 - Reynolds-dependent Clark Y aerodynamics

- Future auditable Reynolds/Mach operating envelope, polar generation or published-data selection,
  validation, and separately reviewed runtime integration.

### M2.4 - Validated electric propulsion and APC propeller model

- M2.4A: off-runtime configuration/component evidence, exact APC source ingestion, strict parsing,
  provenance, and readiness blockers.
- M2.4B: generic electrical calibration, explicit battery/ESC behavior, speed-dependent
  coefficient maps, and synthetic propulsion-runtime integration are implemented; LT-40 runtime
  configuration remains blocked until installed-configuration evidence closes.

### M2.5 - Trim solver

- Trim definition and solution after geometry, mass properties, aerodynamics, and propulsion are
  credible.

### M2.6 - Automated physics validation

- Reproducible validation scenarios, tolerances, regression evidence, and acceptance gates.
