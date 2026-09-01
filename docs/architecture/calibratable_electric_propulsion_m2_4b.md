# M2.4B calibratable electric propulsion runtime

M2.4B extends the deterministic S5B electric propulsion model without changing its legacy
meaning. It is a generic runtime capability. It does not identify or configure the propulsion
installed in a SIG Kadet LT-40 EGV.

## Legacy S5B baseline

S5B models a battery, ideal PWM ESC, motor Kv/Kt, and fixed-pitch propeller. A fixed
`PropellerCoefficientTable` supplies `Ct(J)` and `Cq(J)`. The one-quadrant shaft equilibrium uses
exactly 48 bisection iterations and produces thrust, propeller load torque, shaft reaction torque,
and the body-frame wrench. Schema v0-v3 models continue to resolve to this representation with an
ideal ESC (`series_resistance_ohm = 0`) and a fixed coefficient table.

The legacy entry points are thin wrappers over the M2.4B implementation. There is one set of
electrical, equilibrium, propeller-load, and wrench equations.

## ESC equivalent-resistance model

`EscConfig` adds a finite, non-negative series resistance `Resc`. Zero is the ideal S5B case. For
duty `d`, open-circuit battery voltage `Voc`, battery resistance `Rb`, motor resistance `Rm`, back
EMF constant `Ke`, and candidate shaft speed `omega`, the one-quadrant model is:

```text
Imotor_raw = (d Voc - Ke omega) / (Rm + Resc + d^2 Rb)
Imotor     = max(Imotor_raw, 0)
Ibattery   = d Imotor
Vbattery   = Voc - Ibattery Rb
Vmotor     = d Vbattery - Imotor Resc
```

The model exposes battery-terminal electrical power, ESC resistive loss `Imotor^2 Resc`, and motor
electrical input power. These are deterministic derived diagnostics, not integrated states. There
is no switching, arbitrary fixed loss, state of charge, capacity, or thermal model.

## Shaft-speed coefficient map

`PropellerCoefficientSource` statically selects either a legacy `FixedTable` or a
`ShaftSpeedMap`. A `PropellerCoefficientMap` owns one or more strictly increasing, finite,
positive shaft-speed nodes in rad/s. Each node owns an independently validated
`PropellerCoefficientTable`; J grids need not match and are not resampled during initialization.

At `(J, omega)` the map:

1. samples the lower node's table at J using the existing deterministic J interpolation/clamp;
2. samples the upper node's table at the same J;
3. linearly interpolates Ct and Cq in shaft speed.

One node behaves as a speed-independent table. Multiple-node queries below or above the speed
range clamp to the nearest node. `ShaftSpeedRangeStatus` reports `BelowRange`,
`ExactOrInRange`, or `AboveRange`; there is no speed extrapolation. The stopped-shaft rule still
sets thrust and load torque exactly to zero and avoids division by shaft speed.

## Equilibrium and RK4 coupling

Every one of the 48 bisection residual evaluations samples the coefficient source using that
residual's candidate shaft speed. Coefficients are not frozen at a previous or committed RPM.
Aircraft integration remains stage-local:

```text
RK4 stage state
  -> hub air-relative velocity and axial speed
  -> candidate-speed shaft equilibrium
  -> J and Ct/Cq(J, omega)
  -> electrical diagnostics, thrust, torque, and stage wrench
```

The hot path uses enum dispatch, borrowed immutable tables, and by-value diagnostics. It performs
no allocation after configuration construction. Fixed-table/ideal-ESC regression tests preserve
legacy numerical behavior, and repeated mapped evaluations are bit-identical.

## Aircraft-model schema v4

Schema v4 extends the unchanged schema-v3 Reynolds representation. Its propulsion object makes
battery, ESC, motor, propeller, and the tagged `fixed_table` or `shaft_speed_map` coefficient
source explicit. Unknown fields fail closed. The v4 physics fingerprint includes ESC resistance,
source kind, ordered nodes and all samples, plus versioned tags for the J interpolation, speed
interpolation, and clamp policies. Fingerprint semantics for v0-v3 are unchanged.

## Evidence boundary and limitations

The synthetic v4 fixture is architecture-only and contains invented values. The M2.4A
`PropulsionEvidence` loader remains separate from `AircraftModelLoader`, remains
`runtime_ready = false`, and has no conversion into `ElectricPropulsionConfig`. Historical
component and APC material is not runtime configuration; the raw APC file is not read by this
runtime path. No SIG LT-40 model is created.

M2.4B deliberately excludes battery depletion, thermal behavior, rotor inertia and gyroscopic
moment, regeneration, windmilling, reverse or variable pitch, multiple motors, slipstream,
ground effect, automatic fitting, and trim solving.
