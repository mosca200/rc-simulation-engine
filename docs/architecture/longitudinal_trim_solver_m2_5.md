# M2.5 deterministic longitudinal trim solver

## Scope and assumptions

M2.5 adds a general aircraft-level equilibrium capability for symmetric, straight-and-level,
steady flight. It solves three unknowns:

```text
alpha_rad
elevator_command
throttle
```

For this first level-flight case, sideslip, angular rates, and air-relative flight-path angle are
zero, and pitch attitude is defined as `theta = alpha`. The candidate air-relative velocity is
horizontal in NED world +X at the requested speed. The configured wind vector is added to the
candidate ground velocity, so the aerodynamic pipeline receives exactly that air-relative
velocity. Position is an arbitrary zero origin because the current environment is spatially
constant.

This slice does not solve lateral trim, sideslip, turns, climbs, descents, dynamic trim, or control
laws.

## Coordinates and residual equations

The solver retains the repository conventions: NED world axes (+X north, +Y east, +Z down) and
FRD body axes (+X forward, +Y right, +Z down). Positive pitch attitude is a positive rotation
about +Y, placing the body nose above the horizontal NED +X direction.

Each candidate is evaluated with zero body angular velocity. The three independent dimensional
residuals are:

```text
Rx = mass * linear_acceleration_world_x       [N]
Rz = mass * linear_acceleration_world_z       [N]
My = total_body_wrench.moment_body_y           [N m]
```

`Rx` and `Rz` are force-equivalent residuals derived from the runtime rigid-body derivative, so
they include gravity. `My` is the pitch moment about the model CG. Convergence requires both force
components independently within `force_n` and the pitch moment within `pitch_moment_nm`; no scalar
objective can hide cancellation between equations.

## Runtime-physics identity

`evaluate_aircraft_instantaneous` is the common static/RK4-stage primitive. The normal fixed-step
simulation and the trim solver both call it. It aggregates the same model-ordered aerodynamic
elements and M2.4B propulsion wrench, then calls the same rigid-body derivative with configured
gravity. The trim search performs no RK4 integration.

For every candidate:

1. a normalized `PilotInput` is formed with neutral roll/yaw;
2. `evaluate_steady_controls` runs rates/expo shaping, conventional mixing, servo reversal, and
   asymmetric travel mapping;
3. each binding applies the resulting steady actuator angle relative to servo neutral;
4. local element velocity drives fixed-polars or M2.3C Reynolds-family sampling;
5. throttle runs through the M2.4B battery, ESC resistance, motor, 48-step shaft equilibrium, and
   fixed-table or candidate-speed coefficient-map path;
6. aerodynamic and propulsion forces, lever arms, intrinsic moments, and shaft reaction torque
   form the body wrench;
7. the rigid-body derivative adds gravity and evaluates accelerations.

The steady control primitive represents the eventual target of the existing rate-limited servo;
it does not equate command with deflection. A regression test proves it equals the settled dynamic
control pipeline.

## Request, bounds, and initial guess

`LongitudinalTrimRequest` contains target airspeed, bounds for all three unknowns, an explicit
initial guess, separate force/moment tolerances, and a nonzero maximum iteration count. Bounds
must be finite and strictly ordered. Elevator bounds must remain within `[-1, 1]`; throttle bounds
must remain within `[0, 1]`. Target speed and tolerances must be finite and positive.

Finite initial guesses are deterministically clamped once into their declared bounds. Every
finite-difference and line-search candidate is also clamped, so returned variables cannot escape
the request domain.

## Deterministic numerical method

The solver uses bounded Newton iterations on the three residuals. The numerical Jacobian uses a
deterministic centered difference where possible and the corresponding asymmetric difference near
a bound. Per-variable perturbation is:

```text
max(bound span * 1e-5, 1e-7)
```

The residual vector is normalized by the request's force and moment tolerances before Jacobian
construction. Thus one newton of force is not implicitly treated as equivalent to one newton-metre;
the scales express the caller's independent acceptance criteria.

The 3x3 Jacobian is checked through its singular values. A smallest-to-largest ratio at or below
`1e-10` is treated as singular/quasi-singular. A successful Newton step is passed through a fixed,
ordered line search: full step followed by at most 12 deterministic halvings. The first bounded
candidate that strictly lowers the scaled infinity norm is accepted. There is no RNG, wall-clock
condition, tolerance-based unbounded loop, or external optimizer.

## Convergence and failures

`solve_longitudinal_trim` returns `Result<LongitudinalTrimSolution, LongitudinalTrimFailure>`.
Solutions contain the final variables, `theta`, exact evaluated state, steady control positions,
body wrench, derivative, dimensional residuals, and iteration count. Failures retain a reason,
iteration count, and the last finite evaluation when one exists.

Failure reasons are:

- `NoFeasibleSolution`: the bounded deterministic line search cannot improve the residual;
- `SingularJacobian`: the three local equations do not independently constrain the unknowns;
- `IterationLimit`: the explicit iteration budget is exhausted;
- `NonFiniteEvaluation`: runtime physics, Jacobian, or Newton step becomes non-finite.

`evaluate_longitudinal_trim_candidate` is public for independent physical re-evaluation and
diagnostics. Tests use it to verify the returned solution rather than trusting cached residuals.

## Determinism, performance, and compatibility

Identical validated inputs follow identical evaluation ordering, finite differences, linear solve,
and line-search decisions. Tests require identical result values on repeated solves. Allocations
for candidate element buffers and small solver linear algebra are acceptable outside the 500 Hz
loop. The existing runtime stage path remains allocation-free; model schemas and physics
fingerprints v0-v4 are unchanged by this solver-only capability.

## Evidence boundary and limitations

The dedicated fixture is classified `synthetic_test`, has `reference_aircraft = null`, and uses
invented architecture-test values. No reference evidence is converted into runtime configuration.
There is no SIG Kadet LT-40 model because its operational mass, CG, inertia, and installed
propulsion remain unresolved.

M2.5 deliberately omits lateral controls, rudder/aileron trim, sideslip, coordinated turns,
climb/descent paths, atmospheric variation, dynamic trim, autopilot/PID, ground interaction, and
automatic parameter fitting. M2.6 may build automated validation around this deterministic
primitive, but M2.5 does not implement M2.6.
