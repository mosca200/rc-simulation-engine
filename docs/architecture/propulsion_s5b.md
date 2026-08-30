# S5B electric propulsion

S5B implements one deterministic, quasi-static electric propulsion assembly:

```text
normalized throttle
  -> constant-voltage Thevenin battery
  -> ideal one-quadrant PWM ESC
  -> Kv/Kt equivalent motor
  -> fixed-iteration motor/propeller torque balance
  -> Ct/Cq propeller model
  -> thrust, reaction torque, and body wrench
```

The subsystem is stateless and independent of `Simulation`. Its complete evaluator is suitable
for direct use inside each RK4 stage.

## Battery and ideal PWM ESC

`BatteryConfig` contains constant open-circuit voltage `Voc > 0` and internal resistance `Rb >= 0`.
It is the minimum Thevenin model:

```text
Vbattery = Voc - Ibattery * Rb
```

The ideal, lossless, one-quadrant ESC interprets normalized throttle `d` as PWM duty cycle:

```text
Vmotor   = d * Vbattery
Ibattery = d * Imotor
```

Consequently `Vbattery * Ibattery = Vmotor * Imotor` within floating-point rounding. There is no
ESC dynamic state, switching loss, braking, regeneration, or reverse operation.

Open-circuit voltage is constant. Battery state of charge, capacity, cell curves, balancing,
temperature, recovery, and ageing are deliberately absent.

## Motor conversion and electrical solution

The API accepts the common RC rating in rpm/V and converts it once during configuration:

```text
Kv_si = Kv_rpm_per_v * 2*pi / 60       [rad/s/V]
Ke    = 1 / Kv_si                       [V/(rad/s)]
Kt    = Ke                              [N*m/A]
```

For non-negative shaft-speed magnitude `omega`, the analytic battery-sag solution is:

```text
Imotor_raw = (d * Voc - Ke * omega) / (Rm + d^2 * Rb)
Imotor     = max(Imotor_raw, 0)
Ibattery   = d * Imotor
Vbattery   = Voc - Ibattery * Rb
Vmotor     = d * Vbattery
```

The one-quadrant clamp prevents regenerative current. Available motor torque uses the configured
no-load current as a simple loss model:

```text
motor_torque = Kt * max(Imotor - I0, 0)
```

This equivalent model does not represent phase current, commutation, torque ripple, detailed iron
loss, switching, thermal effects, or a governor.

## Propeller frame and local air velocity

The right-handed propeller frame has +X along positive thrust. `orientation_body_from_prop` maps
propeller coordinates into FRD body coordinates, and `position_body_m` points from the body
origin/CG to the hub.

Rotation direction is named without viewpoint ambiguity:

- `PositiveAboutLocalX`: angular velocity follows the right-hand rule about local +X;
- `NegativeAboutLocalX`: angular velocity points along local -X.

Shaft speed in outputs is a non-negative magnitude; the enum supplies its direction.

`AeroEnvironment` is reused as the validated shared source of air density and world-frame wind.
Despite its S4-era name, its values apply equally to propulsion. Hub air-relative velocity is
recomputed from the current rigid-body stage state:

```text
Vair_world     = Vbody_world - Vwind_world
Vair_body_cg   = world_to_body(q_world_from_body, Vair_world)
Vair_body_prop = Vair_body_cg + omega_body x position_body
Vair_prop      = inverse(q_body_from_prop) * Vair_body_prop
Vaxial         = Vair_prop.x
```

Positive axial airspeed denotes normal forward travel of the propeller through the air. Local Y/Z
crossflow is reported but does not affect coefficients in S5B.

## Coefficients, advance ratio, thrust, and load torque

`PropellerCoefficientTable` owns a setup-time `Vec` with at least two finite samples, strictly
increasing advance ratio, and non-negative `Ct` and `Cq`. Lookup is allocation-free deterministic
binary search followed by piecewise-linear interpolation. Exact samples are preserved and queries
outside the table clamp to endpoints. The non-negative domain explicitly models motoring only.
`Cq` is the torque coefficient used directly by the equation below; a dataset expressed as power
coefficient `Cp` must be converted externally with `Cq = Cp / (2*pi)`.

Above `MIN_SHAFT_SPEED_RAD_S = 1e-9 rad/s`:

```text
n = omega / (2*pi)             [rev/s]
J = Vaxial / (n * D)
```

At or below the threshold, `J`, thrust, and propeller load torque are zero. The threshold only
avoids a singular division and is not an artificial solver stabilizer.

The dimensional coefficient equations are:

```text
T = Ct * rho * n^2 * D^4
Q = Cq * rho * n^2 * D^5
Pprop = omega * Q
```

## Quasi-static operating point

Rotor inertia is deliberately absent. Every evaluation solves:

```text
residual(omega) = motor_torque(omega) - propeller_load_torque(omega)
residual(omega) = 0
```

Throttle zero returns zero shaft speed directly. Otherwise the physical bracket is:

```text
lower = 0
upper = d * Voc / Ke
```

If `residual(0) <= 0`, the stopped endpoint is selected. Otherwise the solver performs exactly
`PROPULSION_BISECTION_ITERATIONS = 48` bisection iterations. A strictly positive residual moves
the lower bound; zero or negative residual moves the upper bound. The fixed iteration count avoids
platform-dependent tolerance exits and also selects the onset of the zero-torque region when the
load is zero. There is no Newton initial guess, allocation, or dynamic dispatch.

## Body wrench and reaction torque

Propeller-frame thrust is:

```text
force_prop = [T, 0, 0]
```

It is transformed to the body frame and contributes the lever-arm moment `position x force`.
The equal-and-opposite shaft reaction applied to the body is:

```text
reaction_prop = [-spin_sign * Q, 0, 0]
```

That moment is also transformed into the body frame. The final wrench is their deterministic sum.

## RK4 staging and deliberate limits

The intended independent-subsystem use is:

```rust,ignore
Rk4Integrator::step(&state, dt_s, |stage_state| {
    let propulsion = evaluate_electric_propulsion(
        stage_state,
        throttle,
        &config,
        &environment,
        &coefficient_table,
    );
    evaluate_derivative(stage_state, &body_params, &propulsion.wrench_body, &gravity)
})
```

Therefore local airspeed, J, equilibrium RPM, Ct/Cq, thrust, and torque are recomputed on all four
RK4 stages. `Simulation`, replay, snapshots, and fingerprints do not own this subsystem yet.

S5B intentionally omits rotor inertia and gyroscopic effects, propwash/slipstream, blade-element
or momentum theory, windmilling, regeneration, reverse thrust, battery SOC, motor/battery thermal
state, multi-motor accumulation, control-to-aero coupling, and full aircraft assembly.

Validated scalar ranges guarantee finite individual configuration values, but do not promise that
arbitrarily extreme combinations remain representable in `f64` (for example `Voc / Ke` or `D^5`).
Physically representative RC configurations remain far inside these numerical limits. A valid but
pathological non-monotonic `Cq(J)` table can also contain multiple equilibria; fixed bisection stays
deterministic but does not assert uniqueness or stability of such a dataset.
