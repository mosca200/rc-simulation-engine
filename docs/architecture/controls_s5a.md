# S5A control pipeline

S5A implements a deterministic, allocation-free control boundary independently of rigid-body
integration, propulsion, and aerodynamics:

```text
PilotInput -> rates/expo -> conventional mixer -> servo targets
           -> rate-limited servo state -> physical actuator positions
```

## Pilot-command contract

`PilotInput` has private fields and validated getters. The real-time constructor clamps roll,
pitch, and yaw to `[-1, +1]`, clamps throttle to `[0, 1]`, and replaces non-finite inputs with the
neutral value zero. Replay deserialization is deliberately stricter: it rejects non-finite or
out-of-range data and never silently clamps a recording.

The command signs are frozen against the right-handed Forward-Right-Down body frame:

- positive roll is positive rotation about +X: right roll, right wing down;
- positive pitch is positive rotation about +Y: nose-up intent;
- positive yaw is positive rotation about +Z: nose-right intent;
- throttle zero is idle/off command and throttle one is full command.

These are logical command meanings, not control-surface trailing-edge conventions.

## Rates and expo

Each attitude axis has validated `rate` and `expo` values in `[0, 1]`. For normalized input `x`,
the exact response is:

```text
shaped = rate * ((1 - expo) * x + expo * x^3)
```

No deadband, trim, or other shaping is applied. Throttle remains linear and bypasses attitude
shaping.

## Conventional mixer

The fixed-wing mixer is an explicit semantic boundary even though its S5A mapping is direct:

```text
aileron  = shaped roll
elevator = shaped pitch
rudder   = shaped yaw
throttle = shaped throttle
```

No general matrix mixer, elevon, V-tail, differential, or other airframe configuration exists in
S5A.

## Logical commands and physical servo angles

`ServoConfig` describes only physical installation: minimum, neutral, and maximum angles in
radians, maximum angular speed in rad/s, and the `reversed` installation flag. It does not encode
the aerodynamic sign or effectiveness of a surface.

For normalized command `c`, reversal is applied first:

```text
effective = reversed ? -c : c
```

Asymmetric travel is mapped piecewise:

```text
effective >= 0: target = neutral + effective * (max - neutral)
effective <  0: target = neutral + (-effective) * (min - neutral)
```

Thus `-1`, `0`, and `+1` reach the configured travel endpoints and neutral point respectively for
a non-reversed installation. `ServoState` stores only the current physical angle and remains
separate from canonical `RigidBodyState`.

## Rate-limited dynamics and timing

For one fixed step:

```text
max_delta = max_speed_rad_s * dt_s
error     = target - current
delta     = clamp(error, -max_delta, +max_delta)
new_angle = current + delta
```

When the target is reachable, it is assigned exactly to avoid overshoot and residual drift.
`ControlSystemState` is advanced once per fixed physics step. S5A does not make it part of
`Simulation` and does not modify RK4. Future aircraft coupling is expected initially to use a
zero-order hold of actuator positions over each 2 ms physics step. Servo dynamics are much slower
than the 500 Hz flight core; sub-stage actuation can be reconsidered if a later model demonstrates
that it is necessary.

Throttle passes through the complete pipeline without servo lag and is reserved for future
propulsion work.

## Deliberate exclusions

S5A does not model servo acceleration, motor inertia, PID control, elasticity, backlash, deadband,
torque saturation, voltage or thermal effects, jitter, propulsion, propwash, stabilization,
autopilot, or HID input. It does not change `AeroElement`, polar coefficients, angle of attack, or
aerodynamic forces. Mapping a physical actuator angle to a surface deflection is a later,
separately validated boundary.
