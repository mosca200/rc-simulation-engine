# SIG Kadet LT-40 EGV — Provisional Runtime Demonstrator (M2.10A.1)

## THIS MODEL IS A PROVISIONAL RUNTIME DEMONSTRATOR

**Classification:** `synthetic_test`
**Model ID:** `sig-kadet-lt40-egv-provisional`

This model is **NOT** a validated SIG Kadet LT-40 reference aircraft. It is classified as
`synthetic_test` because critical runtime parameters remain unresolved. Successful simulation
does **not** constitute LT-40 flight-fidelity validation.

## What this model IS useful for

- End-to-end simulator pipeline testing (load → simulate → render)
- Renderer and visualization development
- Control/input integration testing
- exercising the Reynolds-family aero path, finite-wing surfaces, downwash, and propulsion
- later replacement of provisional polars through the M2.9 XFOIL pipeline

## What this model is NOT yet suitable for

- LT-40 flight-fidelity claims
- Quantitative trim validation
- Performance prediction
- Stability validation against the real aircraft
- Any claim that provisional numbers represent measured LT-40 physics

## Evidence-backed facts (from `docs/reference_aircraft/`)

| Parameter | Value | Source | Quality |
|-----------|------:|--------|---------|
| Target aircraft | SIG Kadet LT-40 EGV ARF | SIG product page | identity |
| Wingspan | 1.778 m | SIG EGV product page | `manufacturer_spec` |
| Reference wing area | 0.580644 m² | SIG EGV product page (900 in²) | `derived` |
| Aircraft length | 1.447 m | SIG EGV product page | `manufacturer_spec` |
| Airfoil | Clark Y | SIG EGV product page | `manufacturer_spec` |
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
| CG range | 88.9–107.95 mm aft wing LE | SIG build manual | `manufacturer_spec` (documentary only) |
| Motor Kv | 1000 rpm/V | Himax HC3528-1000 manual | `manufacturer_spec` (historical) |
| Motor winding resistance | 0.020 Ω | Himax HC3528-1000 manual | `manufacturer_spec` (historical) |
| Motor no-load current | 2.6 A | Himax HC3528-1000 manual | `manufacturer_spec` (historical) |
| Propeller diameter | 0.2794 m (11 in) | APC 11x7E | `manufacturer_spec` (historical) |
| Propeller pitch | 0.1778 m (7 in) | APC 11x7E | `manufacturer_spec` (historical) |
| Propeller Ct/Cq samples | APC PER3 dataset | APC manufacturer data | `manufacturer_data` (historical) |

## Provisional runtime placeholders

These values were introduced **only to make the model executable**. They are NOT validated
LT-40 physics and must NOT be cited as evidence.

| Parameter | Value | Why provisional |
|-----------|------:|-----------------|
| **Mass** | 2.778 kg | Midpoint of published flying-weight range. No physical measurement exists. The published range is comparison-only per repository evidence policy. |
| **Inertia** | Ixx=0.30, Iyy=0.35, Izz=0.55 kg⋅m² | Estimated from mass and geometry. No pendulum or trifilar measurement. Off-diagonals assumed zero. |
| **CG position** | 88.9 mm aft wing LE | Documentary beginner position from SIG manual. Not measured on a physical EGV airframe. |
| **HT longitudinal position** | ~1.0 m aft wing LE | Unknown. No manufacturer dimension bridges wing-to-tail. |
| **VT longitudinal position** | ~1.0 m aft wing LE | Unknown. Same blocker as HT. |
| **Tail Z positions** | 0.0 m (HT), −0.10 m (VT) | Unknown vertical positions in the runtime CG frame. |
| **Wing incidence in model** | 0° (identity quaternion) | Documented +1.5° incidence not yet applied to element orientations. |
| **Wing dihedral in model** | 0° (identity quaternion) | Documented 3° per panel dihedral not yet applied. |
| **Reynolds polar families** | Provisional Clark Y and symmetric polars | No XFOIL-generated polars exist. Placeholder shapes only. |
| **Battery voltage** | 12.6 V (3S fully charged) | Historical config uses 3S LiPo; exact pack unknown. |
| **Battery internal resistance** | 0.028 Ω | Unknown. Provisional estimate. |
| **ESC resistance** | 0.010 Ω | Unknown. Provisional estimate. |
| **Control deflection limits** | ±20° aileron, ±25° elevator, ±30° rudder | Angular limits not evidenced. Published linear travels cannot be converted without measurement radii. |
| **Control response rates** | roll 0.70, pitch 0.65, yaw 0.55 | Provisional trainer-class estimates. |
| **Span efficiency factors** | wing 0.85, HT 0.80, VT 0.75 | Provisional estimates for finite-wing induced drag. |
| **Downwash factor** | 0.6 (wing → HT) | Provisional. No measured or computed downwash exists. |
| **Propeller position** | X_body = 0.30 m | Provisional. Exact motor-box-to-CG distance unknown. |
| **Propeller rotational inertia** | 0.0 kg⋅m² | No evidence. Zero preserves pre-M2.8F behavior. |

## Unresolved blockers

| Blocker | Impact |
|---------|--------|
| HT/VT absolute longitudinal stations | Tail arm unknown; longitudinal stability cannot be validated |
| Wing-to-tail aerodynamic reference arms | Consequence of unknown tail stations |
| EGV equality for kit geometry | Plan measurements are from original kit; EGV dimensional equality unproven |
| Actual airfoil polar data | No XFOIL execution for LT-40 Clark Y at representative Reynolds numbers |
| Operational mass | No physical airframe identified and measured |
| Operational CG | No physical measurement on an identified EGV airframe |
| Full inertia tensor | No measurement campaign |
| Battery internal resistance and voltage sag | No pack identified or measured |
| EGV motor thrust-axis angles | EGV manual does not publish installation angles |
| Control travel angular conversion | Published linear travels lack measurement radii |

## Model structure

- **Schema version:** 7
- **Classification:** `synthetic_test` (NOT `reference_aircraft`)
- **Aero elements:** 8 (4 wing, 2 HT, 2 VT)
- **Aero surfaces:** 3 (main-wing, horizontal-tail, vertical-tail)
- **Reynolds polar families:** 2 (wing-clark-y-provisional, tail-symmetric-provisional)
- **Downwash interactions:** 1 (wing → horizontal-tail, provisional factor)
- **Slipstream interactions:** 0
- **Control surface bindings:** 4 (aileron L/R, elevator, rudder)
- **Propulsion:** battery + ESC + motor + propeller + fixed Ct/Cq table

## Validation results

| Check | Result |
|-------|--------|
| Model loads with canonical loader | ✅ Pass |
| Classified as `synthetic_test` (NOT reference_aircraft) | ✅ Pass |
| Physics fingerprint deterministic | ✅ Pass |
| AircraftSimulation initializes | ✅ Pass |
| Finite simulation at 500 Hz (5 s) | ✅ Pass, no NaN/Inf |
| Reynolds-family path exercised | ✅ Pass |
| Control surfaces bound | ✅ Pass |
| Propulsion initializes | ✅ Pass |
| Trim/characterization | ⚠️ Not meaningful (provisional polars and tail arm) |

## Path to a validated reference model

1. Execute XFOIL campaign for Clark Y at LT-40-representative Reynolds numbers
2. Bind XFOIL-generated polars via M2.9 pipeline
3. Measure or source tail longitudinal stations from a physical EGV airframe
4. Conduct physical mass/CG/inertia measurement campaign
5. Identify and measure installed battery pack
6. Apply documented wing incidence and dihedral to element orientations
7. Re-classify as `reference_aircraft` when all blockers are resolved
