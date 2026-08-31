# Aircraft assembly S6.1

S6.1 is the first deterministic, headless assembly of the previously validated aircraft-model,
controls, aerodynamics, propulsion, RK4, and rigid-body subsystems. It advances a complete RC
aircraft in free flight, but it does not introduce rendering, collision, ground interaction,
propwash, or full-aircraft replay.

The existing conventions remain unchanged: SI units, an NED world frame, an FRD body frame,
Hamilton quaternions, `f64`, a 500 Hz default physics rate, and a fixed timestep derived from an
integer step index.

## Crate boundary

The dependency direction is:

```text
sim_math
   ^
sim_core
   ^
 model
   ^
aircraft
   ^
rcsim-app
```

The `aircraft` crate is the orchestration boundary. It may use `model`, `sim_core`, and `sim_math`,
but `sim_core` does not depend on `model` or `aircraft`, and `model` does not depend on `aircraft`.
The established `sim_core::Simulation` remains the rigid-body foundation and is not reinterpreted
as a full aircraft simulation. No ECS or renderer concerns enter this dependency chain.

## Immutable model and mutable runtime state

`AircraftModel` remains immutable after loading. It owns validated mass properties, ordered polar
tables, base aerodynamic-element geometry, resolved control-surface bindings, the S5A control
configuration, and optional S5B propulsion configuration. The base orientation of every
`AeroElement` describes its neutral geometry.

Dynamic values live separately in `AircraftState`:

```text
AircraftState
  rigid_body: RigidBodyState
  controls:   ControlSystemState
```

Servo state is not added to `RigidBodyState`, and no dynamic state is added to `AircraftModel`.
S5B remains quasi-static, so S6.1 has no propulsion state. `AircraftSimulation` owns the model,
simulation configuration, runtime state, step index, and a preallocated effective-element vector.
Simulation time is always:

```text
sim_time_s = step_index * dt_s
```

The simulation configuration contains the fixed timestep, NED gravity vector, and constant
`AeroEnvironment` for the run. The environment provides air density and NED wind velocity; S6.1
does not model weather dynamics.

## Resolved control-surface bindings

Schema v1 resolves every authoring-time `element_id` into an element index during model loading.
The runtime binding is conceptually:

```text
element_index
actuator        = aileron | elevator | rudder
deflection_gain
```

Bindings retain declaration order. An aerodynamic element may be controlled by at most one
binding, and elements with no binding remain fixed. The physics loop performs no string lookup.

For the selected actuator, the surface angle is computed as:

```text
servo_delta_rad       = servo_angle_rad - servo_neutral_angle_rad
surface_deflection_rad = deflection_gain * servo_delta_rad
```

The subtraction is essential: servo neutral means zero surface deflection even when the configured
neutral servo angle is nonzero. Consequently neutral copies the base element orientation exactly.
`ServoConfig::reversed` determines the physical direction of servo travel, while
`deflection_gain` determines how that travel maps to the surface. The assembly does not hardcode
left/right aileron, elevator trailing-edge, or rudder sign conventions.

## Local +Y hinge and quaternion composition

Every controlled element rotates about its own local `+Y` axis. In the S4 element frame, `+X` is
chord-forward, `+Y` is positive span and the hinge direction, and `+Z` is down. A rudder model
orients its element frame so that local `+Y` coincides with its vertical hinge; no arbitrary hinge
axis is part of schema v1.

For a surface deflection `delta`, define the active Hamilton rotation:

```text
delta_orientation = rotation_about_element_local_positive_Y(delta)
```

The effective element-to-body orientation is exactly:

```text
orientation_body_from_effective_element
    = orientation_body_from_base_element * delta_orientation
```

The post-multiplication is intentional: it applies the hinge rotation in the element's local frame.
`delta_orientation * orientation_body_from_base_element` would instead apply a body-aligned
rotation and is not equivalent when the base orientation is non-identity.

## Effective aerodynamic elements

The immutable base `AeroElement` values in `AircraftModel` are never mutated. During
`AircraftSimulation` initialization, their validated geometry is copied once into an effective
element vector with identical deterministic ordering. At the beginning of each physics step:

1. fixed elements retain or are restored to their base orientation;
2. controlled elements receive `base * local_Y_rotation(surface_deflection)`;
3. positions, areas, chords, and resolved polar indices remain unchanged.

Updates occur in place. The vector is neither cloned nor reallocated per step, and binding targets
are already compact indices.

The S6.1 control-surface model treats each mobile surface as a complete quasi-2D `AeroElement`
whose geometry rotates about local `+Y`. This is a coherent first assembly model, not a detailed
model of hinge moments, gaps, separated-flow interaction, nonlinear control effectiveness, or
partial-chord deformation.

## Exact aircraft step order

`AircraftSimulation::step(input)` performs these operations in order:

1. Advance the complete S5A control pipeline exactly once using `input` and `dt_s`.
2. Subtract each actuator's configured neutral angle and multiply by its binding gain.
3. Update all effective aerodynamic-element orientations in place.
4. Read throttle from the resulting `ControlSurfacePositions`.
5. Run one S3.2 RK4 step. Each of its four stage states receives a fresh aircraft-wrench
   evaluation before `evaluate_derivative` is called.
6. Commit the resulting `RigidBodyState` to `AircraftState`.
7. Increment `step_index` exactly once.
8. Return the post-step by-value aircraft snapshot.

Initialization may fail while validating configuration and initial state. Once initialized,
`step()` is infallible and deterministic.

## Zero-order hold and stage-correct forces

Actuator dynamics are intentionally outside RK4 in S6.1. The values used during a single physics
step follow this policy:

| Quantity | Update policy within one physics step |
| --- | --- |
| Servo state | Advanced once before RK4 |
| Effective surface orientation | Computed once, held for k1/k2/k3/k4 |
| Throttle | Read once, held for k1/k2/k3/k4 |
| Aerodynamic wrench | Recomputed from each RK4 stage state |
| Propulsion operating point and wrench | Recomputed from each RK4 stage state |

This zero-order hold is deliberate. Servo state is not numerically integrated inside RK4. Holding
actuator angles does not mean holding forces: translational velocity, orientation, angular rate,
local element velocity, angle of attack, advance ratio, shaft speed, thrust, and moments can differ
between k1, k2, k3, and k4.

## Per-stage wrench aggregation

For every RK4 stage, aggregation starts from a new `BodyWrench::zero()`. Aerodynamic elements are
visited in their stable model order. Each element uses the real S4 evaluator with the current
stage state, its effective geometry, the shared environment, and its resolved `PolarTable`; its
body force and moment are added to the accumulator. S6.1 does not duplicate angle-of-attack,
lift, drag, pitching-moment, or lever-arm equations.

If the model has electric propulsion, the real S5B evaluator is then called with the same stage
state, held throttle, environment, propulsion configuration, and coefficient table. Its force and
moment are added once. A model with no propulsion simply omits this term; no synthetic zero-thrust
powertrain is created.

The total stage wrench is therefore:

```text
W_total(stage_state)
    = sum_in_declared_order(W_aero_element_i(stage_state))
    + W_propulsion(stage_state)  // only when configured
```

`evaluate_derivative(stage_state, rigid_body_params, W_total, gravity_world)` then computes the
stage derivative. Gravity is not a `BodyWrench` and is never added to `W_total`; it remains the
world-frame acceleration handled by the rigid-body derivative evaluator. This prevents double
counting gravity.

The RK4 stage callback is generic and monomorphized. It uses no trait object, dynamic dispatch,
threading, or per-stage allocation.

## Determinism and hot-loop allocation policy

All collection sizes and reference relationships are established during initialization. A step
uses only fixed-order slice iteration, compact indices, stack values, and the preallocated
effective-element vector. It performs no filesystem access, JSON parsing, string lookup, vector
clone, logging, wall-clock query, or heap allocation. Aerodynamic accumulation remains
single-threaded because floating-point addition order is part of deterministic behavior.

Wall-clock measurements in the CLI and Criterion benchmarks are diagnostics only and never enter
physics. Within the same executable/build, target, Rust toolchain, configuration, initial state,
and input stream, repeated runs are expected to produce bit-identical final aircraft state.

## Minimal headless runner

The additive aircraft mode of the existing application is:

```powershell
cargo run -p rcsim-app --release -- aircraft --model models/acro_electric_01/model.json --steps 1000
```

Its aircraft-mode defaults are Acro Electric 01, 1,000 steps, 500 Hz, zero wind, standard air
density, neutral attitude controls, and moderate throttle. `--physics-hz` remains available. The
runner prints model identity and physics fingerprint, simulated time, final NED position/velocity,
Hamilton orientation, body angular velocity, three servo positions, throttle, and wall-clock
diagnostics only after the run. It does not log each physics step.

Invoking `rcsim-app` without the `aircraft` subcommand preserves the earlier foundation runner and
its replay recording path. Aircraft mode deliberately does not create or consume a foundation
replay.

## Explicit S6.1 exclusions

S6.1 deliberately has:

- no propwash or slipstream velocity added to aerodynamic elements;
- no terrain, runway, landing gear, collision, crash physics, or ground clamp;
- no renderer, GLB loading, joystick, or presentation-side state;
- no modification or partial reuse of the foundation replay schema/player;
- no battery SOC, rotor inertia, thermal state, or multiple propulsors;
- no S10 flight calibration claim for Acro Electric 01.

The aircraft may pass through ground altitude because it is simulated in free air. Aerodynamic
elements see only rigid-body air-relative velocity, the local `omega x r` contribution, and ambient
wind.

## Future work

Later stages may add arbitrary hinge axes and mixers, richer control-effectiveness models,
propwash, multiple propulsion units, battery and rotor dynamics, full-aircraft replay keyed by the
model fingerprint, ground/collision, renderer integration, and flight-data calibration. Those are
separate versioned and testable changes; none is implied by the S6.1 assembly.
