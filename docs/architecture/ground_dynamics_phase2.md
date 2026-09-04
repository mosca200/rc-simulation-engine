# Ground dynamics (phase 2)

## Ownership

The canonical `RigidBodyState` in `sim_core` remains the sole owner of
aircraft flight state. Ground contact is **not** implemented through a
separate authoritative rigid-body engine: no second Rapier body exists, no
external solver owns the aircraft, and no state synchronization between two
authorities occurs. `sim_core::ground` only evaluates a deterministic,
state-dependent ground wrench `W_ground(state)` added to the existing
per-stage wrench accumulator. A flat-ground compliant contact model was
chosen over Rapier: three contacts against one plane need no broad-phase
engine, and a native `f64` model preserves determinism and zero allocation.

## Coordinate convention

World is North-East-Down (NED): `+X` north, `+Y` east, `+Z` down. Gravity is
`[0, 0, +9.80665] m/s²`. Body is Forward-Right-Down (FRD). The flat plane is
`z = ground_height_world_down_m` (default `0.0`); free air is
`z < ground_height`; penetration is `contact_z - ground_height` (positive
below the plane). The outward normal (world) is `[0, 0, -1]` (up).
Wheel-bottom points add `+wheel_radius_m` along body-down before transform.

## Contact kinematics

Stage-local world velocity per contact point:

```text
V_contact = V_CG + omega_world × r_world
```

Cross product runs in world frame so translation, pitch, roll, yaw, and
angular velocity all contribute. Contact is never based on CG velocity alone.

## Normal contact force

Unilateral compliant spring-damper along world-down:

```text
F_normal = max(0, k * penetration + c * v_down_world)
```

`v_down_world = +d(penetration)/dt`: sinking increases the push (damping
opposes penetration rate); fast separation clamps to zero (never pulls).
Params are `f64`, validated once at model load, never in the hot loop.
Stability at dt = 0.002 s: stiffness rejected above 20 000 N/m, damping
above 1 500 N·s/m. Above 12 000 N/m / 800 N·s/m requires re-validation.
Validation gear uses 12 000 N/m + 800 N·s/m: millimetre sag for 10 kg,
survives 4.5 m/s hard landings. No velocity clipping hides instability.

## Friction model

Each wheel defines an anisotropic frame from the held steer angle: at zero
steer, longitudinal is body-forward (+X), lateral is body-right (+Y).
Independent Coulomb caps (`long_mu * F_n`, `lat_mu * F_n`) — never an
isotropic hockey puck. Regularized law per axis: linear below 0.25 m/s,
saturated above, exactly zero at zero slip (no div-by-zero, no NaN).
Rolling resistance opposes only longitudinal motion within
`min(rolling_mu * F_n, long_cap)`, clamped so it never propels. Braking adds
only longitudinal capacity (`brake_mu * brake_command * F_n`).

## Gear representation

Schema v8 adds an optional ordered `landing_gear` array; absent gear means
no gear (zero wrench, no invisible wheels). Each contact: body position,
wheel radius, stiffness/damping, longitudinal/lateral/rolling/brake mus,
steering source, max steer angle, steerable/braked flags. Tricycle,
taildragger, two-wheel, and future skids fit without schema changes.

## Steering and brakes

Steerable contacts declare `steering: "rudder"` + `max_steer_angle_rad`;
angle is `rudder_command * max_steer` from held `PilotInput::yaw`,
constant across RK4 stages. Fixed wheels ignore rudder. Nothing hardwires
nosewheels to rudder; mismatched flag/source pairs fail at load. Brakes:
`set_brake_command([0, 1])` holds one scalar across stages (default 0);
throttle is never hijacked and no input channel is consumed, so a future
brake axis slots in without a schema break.

## RK4 integration

Ground forces are state-dependent and evaluated inside the canonical stage
callback: `evaluate_stage` calls `evaluate_ground_wrench` independently at
k1/k2/k3/k4 with held throttle, held aero geometry, held steering/brake
command, and live stage state. No wrench is held constant across stages; no
second integration path exists. Propulsion and aerodynamics run unmodified
at every stage with or without contact — no flying/ground mode switch.
Takeoff emerges when lift unloads the gear; landing emerges when geometry
intersects the plane. No `if speed > X { flying }` trigger exists. Servos
stay outside RK4 (zero-order hold), matching the existing contract.

## Determinism, allocation, telemetry

Bit-identical snapshots + ground telemetry for identical build/model/inputs
(covered by `identical_ground_scenarios_reproduce_bitwise`). Stable contact
order, no maps, no randomness, no wall clock. `step()` holds the zero heap
allocation guarantee with gear configured (fixed 16-array, preallocated
vectors; covered by allocation-counter test). Snapshot carries
`ground_contacts`, `total_ground_normal_force_n`,
`total_ground_tangential_force_n`, `weight_on_wheels` from the committed
contact solution (never altitude); telemetry schema v2 persists them, with a
separate `AircraftGroundTelemetry` view and per-contact loads in-process.
Replay hashes cover only pre-existing core fields: old recordings stay valid.

## Future terrain and limitations

`GroundSurface::height_and_normal(x, y)` is the terrain seam: only `Flat`
exists today; future variants slot behind the same call. Limitations: flat
ground only; point contacts without tire deformation/shimmy; no weather
coupling on ground; no crash/damage model (off-gear body has no collision
proxy); brakes are a Coulomb scalar, not hydraulics; steering is kinematic,
not caster/trail.



