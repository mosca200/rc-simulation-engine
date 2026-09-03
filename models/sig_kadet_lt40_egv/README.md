# SIG Kadet LT-40 EGV Reference Model (M2.10A)

## Status

This is the first loadable reference aircraft model in the repository. It targets the
**SIG KADET LT-40 EGV ARF** trainer-class aircraft using evidence already present in
`docs/reference_aircraft/`.

The model loads, simulates, and produces a deterministic physics fingerprint. However,
several key parameters remain **provisional** pending physical measurement and XFOIL
execution. This document distinguishes what is source-backed, derived, provisional, and
unresolved.

## Source-backed values

| Parameter | Value | Source | Quality |
|-----------|------:|--------|---------|
| Wingspan | 1.778 m | SIG EGV product page | `manufacturer_spec` |
| Reference wing area | 0.580644 m² | SIG EGV product page (900 in² exact conversion) | `derived` |
| Aircraft length | 1.447 m | SIG EGV product page | `manufacturer_spec` |
| Airfoil | Clark Y | SIG EGV product page | `manufacturer_spec` |
| Wing incidence | +1.5° | SIG kit product page | `manufacturer_spec` (kit; EGV correspondence unknown) |
| Wing dihedral | 3° per panel | SIG kit product page | `manufacturer_spec` (kit; EGV correspondence unknown) |
| HT stabilizer incidence | 0° | SIG kit product page | `manufacturer_spec` (kit; EGV correspondence unknown) |
| Wing root chord | 0.3307 m | Validated plan reconstruction | `measured_from_validated_plan` |
| Wing MAC (geometric) | 0.3260 m | Validated plan reconstruction | `derived` |
| Wing fixed area (both panels) | 0.5161 m² | Validated plan reconstruction | `derived` |
| Each aileron area | 0.0280 m² | Validated plan reconstruction | `derived` |
| HT span | 0.6858 m | Validated plan reconstruction | `measured_from_validated_plan` |
| HT fixed area | 0.10248 m² | Validated plan reconstruction | `derived` |
| Elevator area | 0.03484 m² | Validated plan reconstruction | `derived` |
| VT height | 0.2266 m | Validated plan reconstruction | `measured_from_validated_plan` |
| VT fixed area | 0.03778 m² | Validated plan reconstruction | `derived` |
| Rudder area | 0.01586 m² | Validated plan reconstruction | `derived` |
| CG range | 88.9–107.95 mm aft wing LE | SIG build manual | `manufacturer_spec` |
| Motor Kv | 1000 rpm/V | Himax HC3528-1000 manual | `manufacturer_spec` |
| Motor winding resistance | 0.020 Ω | Himax HC3528-1000 manual | `manufacturer_spec` |
| Motor no-load current | 2.6 A | Himax HC3528-1000 manual | `manufacturer_spec` |
| Propeller diameter | 0.2794 m (11 in) | APC 11x7E | `manufacturer_spec` |
| Propeller pitch | 0.1778 m (7 in) | APC 11x7E | `manufacturer_spec` |
| Propeller Ct/Cq samples | APC PER3 dataset | APC manufacturer data | `manufacturer_data` |

## Derived values

| Parameter | Value | Derivation |
|-----------|------:|------------|
| Wing element areas | per-panel fixed + aileron | Polygon subtraction from plan reconstruction |
| Wing element positions | body-frame centroids | Coordinate conversion from plan datum to CG-relative body frame |
| Tail element positions | provisional body-frame | See "Provisional values" below |
| Propeller Cq | Cp / (2π) | Deterministic conversion from APC published Cp |

## Provisional values

These values are **not source-backed** and are used only to make the model loadable.
They will be replaced by measured or computed values in future slices.

| Parameter | Value | Reason for provisional status |
|-----------|------:|-------------------------------|
| **Mass** | 2.778 kg | Midpoint of SIG EGV flying-weight range (2.720–2.835 kg). No physical measurement exists. |
| **Inertia** | Ixx=0.30, Iyy=0.35, Izz=0.55 kg⋅m² | Estimated from mass and geometry. No pendulum or trifilar measurement exists. Off-diagonals assumed zero. |
| **CG position** | 88.9 mm aft wing LE | Beginner position from SIG manual. Not measured on a physical EGV airframe. |
| **HT longitudinal position** | ~1.0 m aft wing LE | Unknown. No manufacturer dimension bridges wing-to-tail. Provisional estimate from aircraft length proportions. |
| **VT longitudinal position** | ~1.0 m aft wing LE | Unknown. Same blocker as HT. |
| **Tail Z positions** | 0.0 m (HT), −0.10 m (VT) | Unknown vertical positions in the runtime CG frame. |
| **Wing incidence in model** | 0° (identity quaternion) | The documented +1.5° incidence is not yet applied to element orientations. |
| **Wing dihedral in model** | 0° (identity quaternion) | The documented 3° per panel dihedral is not yet applied to element orientations. |
| **Reynolds polar families** | Provisional Clark Y and symmetric polars | No XFOIL-generated polars exist. Placeholder polars use thin-airfoil-theory-like shapes. |
| **Battery voltage** | 12.6 V (3S fully charged) | Historical config uses 3S LiPo; exact pack unknown. |
| **Battery internal resistance** | 0.028 Ω | Unknown. Provisional estimate for a 3S 4500 mAh LiPo. |
| **ESC resistance** | 0.010 Ω | Unknown. Provisional estimate. |
| **Control surface deflection limits** | ±20° aileron, ±25° elevator, ±30° rudder | Angular limits not evidenced. Published linear travels cannot be converted without measurement radii. |
| **Control response rates** | roll 0.70, pitch 0.65, yaw 0.55 | Provisional trainer-class estimates. |
| **Span efficiency factors** | wing 0.85, HT 0.80, VT 0.75 | Provisional estimates for finite-wing induced drag. |
| **Downwash factor** | 0.6 (wing → HT) | Provisional. No measured or computed downwash exists. |
| **Propeller position** | X_body = 0.30 m | Provisional. Exact motor-box-to-CG distance unknown. |
| **Propeller rotational inertia** | 0.0 kg⋅m² | No evidence. Zero preserves pre-M2.8F behavior. |

## Unresolved / blocked

| Parameter | Blocker |
|-----------|---------|
| HT and VT absolute longitudinal stations | No manufacturer dimension bridges wing-to-tail across fuselage-view breaks |
| Wing-to-tail aerodynamic reference arms | Consequence of unknown tail stations |
| EGV equality for kit geometry | Plan measurements are from original kit; EGV dimensional equality unproven |
| Actual airfoil polar data | No XFOIL execution has been performed for the LT-40 Clark Y |
| Operational mass | No physical airframe has been identified and measured |
| Operational CG | No physical measurement on an identified EGV airframe |
| Full inertia tensor | No measurement campaign on an identified airframe |
| Battery internal resistance and voltage sag | No pack identified or measured |
| EGV motor thrust-axis down/right offsets | EGV manual does not publish installation angles |
| Control travel angular conversion | Published linear travels lack measurement radii and convention |

## Model structure

- **Schema version**: 7
- **Classification**: `reference_aircraft`
- **Aero elements**: 8 (4 wing, 2 HT, 2 VT)
- **Aero surfaces**: 3 (main-wing, horizontal-tail, vertical-tail)
- **Reynolds polar families**: 2 (wing-clark-y-provisional, tail-symmetric-provisional)
- **Downwash interactions**: 1 (wing → horizontal-tail)
- **Slipstream interactions**: 0 (provisional; no evidence for slipstream targeting)
- **Control surface bindings**: 4 (aileron L/R, elevator, rudder)
- **Propulsion**: battery + ESC + motor + propeller + fixed Ct/Cq table

## Validation results

| Check | Result |
|-------|--------|
| Model loads with canonical loader | ✅ Pass |
| Physics fingerprint deterministic | ✅ Pass |
| AircraftSimulation initializes | ✅ Pass |
| Finite simulation at 500 Hz (5 s) | ✅ Pass, no NaN/Inf |
| Reynolds-family path exercised | ✅ Pass |
| Control surfaces bound | ✅ Pass |
| Propulsion initializes | ✅ Pass |
| Trim/characterization | ⚠️ Not attempted (provisional polars and tail arm make trim scientifically meaningless) |

## Next steps

1. Execute XFOIL campaign for Clark Y at LT-40-representative Reynolds numbers
2. Bind XFOIL-generated polars to the wing Reynolds family via M2.9 pipeline
3. Measure or source tail longitudinal stations from a physical EGV airframe
4. Conduct physical mass/CG/inertia measurement campaign
5. Identify and measure the installed battery pack for resistance and voltage sag
6. Apply documented wing incidence and dihedral to element orientations
7. Attempt trim/characterization with validated polars and measured mass properties
