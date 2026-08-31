# Acro Electric 01 — S10 baseline inventory

## Baseline identity

| Field | Value | Classification |
| --- | --- | --- |
| Model ID | `acro-electric-01` | MEASURED from repository model |
| Display name | `Acro Electric 01` | MEASURED from repository model |
| Model schema | `1` | MEASURED from repository model |
| Physics fingerprint | `dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed` | DERIVED by `AircraftModel::physics_fingerprint()` |
| Model source | `models/acro_electric_01/model.json` | Repository source |
| Calibration status | Initial engineering placeholder; not flight-calibrated | Repository `README.md` |

The repository contains no manufacturer specification, measured geometry, wind-tunnel data,
flight telemetry, target flight envelope, or completed pilot review for this fictional model.
Every physical value below is therefore an existing engineering placeholder, not a validated
real-aircraft parameter.

## Rigid body

- Mass: `2.3 kg`.
- Inertia tensor in FRD body coordinates, `kg·m²`:

```text
[[0.18, 0.00, 0.00],
 [0.00, 0.24, 0.00],
 [0.00, 0.00, 0.39]]
```

## Aerodynamic polar tables

All angles are radians.

### `main-airfoil`

| alpha | Cl | Cd | Cm |
| ---: | ---: | ---: | ---: |
| -0.35 | -0.82 | 0.120 | -0.015 |
| -0.17 | -0.58 | 0.048 | -0.018 |
| 0.00 | 0.08 | 0.025 | -0.020 |
| 0.17 | 0.76 | 0.052 | -0.024 |
| 0.35 | 0.98 | 0.145 | -0.030 |

### `tail-symmetric`

| alpha | Cl | Cd | Cm |
| ---: | ---: | ---: | ---: |
| -0.35 | -0.70 | 0.110 | 0.000 |
| -0.17 | -0.50 | 0.042 | 0.000 |
| 0.00 | 0.00 | 0.022 | 0.000 |
| 0.17 | 0.50 | 0.042 | 0.000 |
| 0.35 | 0.70 | 0.110 | 0.000 |

These tables are piecewise-linear quasi-2D placeholders. They do not establish dynamic-stall,
Reynolds, hysteresis, spin, or post-stall validation.

## Aerodynamic elements

Positions are FRD body metres; orientation is Hamilton `wxyz` from element to body.

| Element | Position | Orientation | Area m² | Chord m | Polar |
| --- | --- | --- | ---: | ---: | --- |
| wing-left-fixed | `[0.04,-0.36,0]` | `[1,0,0,0]` | 0.24 | 0.29 | main-airfoil |
| wing-left-aileron | `[-0.05,-0.70,0]` | `[1,0,0,0]` | 0.10 | 0.20 | main-airfoil |
| wing-right-fixed | `[0.04,0.36,0]` | `[1,0,0,0]` | 0.24 | 0.29 | main-airfoil |
| wing-right-aileron | `[-0.05,0.70,0]` | `[1,0,0,0]` | 0.10 | 0.20 | main-airfoil |
| horizontal-tail-fixed | `[-0.61,0,0.025]` | `[1,0,0,0]` | 0.10 | 0.20 | tail-symmetric |
| elevator | `[-0.69,0,0.025]` | `[1,0,0,0]` | 0.06 | 0.12 | tail-symmetric |
| vertical-tail-fixed | `[-0.56,0,-0.11]` | `[0.7071067812,-0.7071067812,0,0]` | 0.065 | 0.18 | tail-symmetric |
| rudder | `[-0.68,0,-0.11]` | `[0.7071067812,-0.7071067812,0,0]` | 0.04 | 0.12 | tail-symmetric |

Total declared aerodynamic area is `0.905 m²`; this is a sum of discrete model elements and is not
independently measured reference wing area.

## Controls and bindings

| Axis | Rate | Expo |
| --- | ---: | ---: |
| Roll | 0.82 | 0.30 |
| Pitch | 0.76 | 0.28 |
| Yaw | 0.68 | 0.22 |

| Servo | Min rad | Neutral rad | Max rad | Speed rad/s | Reversed |
| --- | ---: | ---: | ---: | ---: | --- |
| Aileron | -0.38 | 0.00 | 0.38 | 4.5 | no |
| Elevator | -0.42 | 0.00 | 0.42 | 4.2 | no |
| Rudder | -0.48 | 0.00 | 0.48 | 3.8 | yes |

| Binding | Element | Actuator | Deflection gain |
| --- | --- | --- | ---: |
| aileron-left | wing-left-aileron | aileron | 1.0 |
| aileron-right | wing-right-aileron | aileron | -1.0 |
| elevator | elevator | elevator | -1.0 |
| rudder | rudder | rudder | -1.0 |

## Electric propulsion

- Battery open-circuit voltage: `16.8 V`.
- Battery internal resistance: `0.035 Ω`.
- Motor Kv: `900 rpm/V`.
- Motor winding resistance: `0.045 Ω`.
- Motor no-load current: `1.2 A`.
- Propeller position: `[0.45,0,0] m` FRD.
- Propeller orientation: `[1,0,0,0]` Hamilton `wxyz`.
- Diameter: `0.28 m`.
- Spin: positive about local +X.

| Advance ratio J | Ct | Cq |
| ---: | ---: | ---: |
| -0.25 | 0.135 | 0.019 |
| 0.00 | 0.125 | 0.018 |
| 0.50 | 0.090 | 0.013 |
| 1.00 | 0.040 | 0.007 |
| 1.50 | 0.000 | 0.002 |

## S10 characterization conditions

- Suite version: `1`.
- Fixed physics rate: `500 Hz`, `dt = 0.002 s`.
- Initial NED position: `[0,0,-100] m`.
- Initial NED velocity: `[18,0,0] m/s`.
- Initial orientation: identity Hamilton quaternion.
- Initial body angular velocity: zero.
- Air density: `1.225 kg/m³`; NED wind: zero.
- Replay artifacts and telemetry captures are generated under `target/s10_validation/`.

No value in `model.json` was changed during S10 because no authoritative reference or documented
target exists. Baseline and final fingerprint are therefore identical.
