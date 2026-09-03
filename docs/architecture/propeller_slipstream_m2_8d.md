# M2.8D deterministic propeller slipstream

M2.8D adds aircraft-model schema v7 and explicit one-way coupling from the aircraft's
single propulsion system to selected aerodynamic elements. There is no implicit wake
membership and no aircraft-specific default.

## Schema and resolution

Schema v7 adds `propeller_slipstream_interactions`. Each interaction has a stable `id`,
an ordered non-empty `target_element_ids` list, and a finite non-negative
`slipstream_velocity_factor`. Loading resolves target IDs to immutable element indices.
An element may appear in only one interaction, and interactions require propulsion.
Runtime stage evaluation performs no string lookup and no allocation.

## Actuator-disk model

The existing propulsion evaluator runs first for every RK4 stage. From its actual
positive axial thrust `T`, the induced velocity is

```text
A  = pi D^2 / 4
vi = 0.5 (sqrt(V^2 + 2 T / (rho A)) - V)
```

`D` is the configured propeller diameter. `V` is exactly the existing propulsion
output's `axial_airspeed_mps`: aircraft air-relative velocity at the propeller location,
including translational velocity minus wind and the `omega x r` contribution, transformed
into the propeller frame and projected on propeller local `+X`.

The authored target increment is

```text
delta_v_target = slipstream_velocity_factor * vi
```

Thus factor 0 disables coupling, factor 1 means the actuator-disk increment, and factor 2
means the ideal far-wake increment. The engine does not apply an additional factor of two.

The stored flow convention is aircraft/section motion through air. Positive thrust is
propeller local `+X`, so the wake increment is added along propeller local `+X`, transformed
through body space into each target element's effective frame. For an aligned tractor this
increases positive chordwise air-relative velocity. It changes the physical velocity vector,
not CL/CD/CM.

No wake is produced for absent propulsion, `T <= 0`, `rho <= 0`, invalid disk area, a zero
factor, or a non-finite derived result. Reverse-thrust, windmill, and braking wakes are outside
this slice.

## Stage ordering and composition

For each RK4 stage the runtime:

1. evaluates actual propulsion;
2. derives `vi` and the body-frame propeller-axis wake;
3. adds each authored increment to the selected effective element's physical local flow;
4. applies M2.8C downwash as a pure rotation to that slipstream-adjusted flow;
5. solves M2.8B surface induced alpha using each member's actual `q * S` weight;
6. samples at `alpha_geom_after_physical_effects - alpha_i`;
7. constructs forces from the final physical flow and adds induced drag;
8. adds the propulsion wrench exactly once.

Different members of one finite-wing surface can therefore have different physical speed,
dynamic pressure, and Reynolds number. Downwash preserves the speed created by slipstream.
A downwash source is unchanged unless it is itself an explicit slipstream target.

Control deflection is applied before section flow construction, so wake transformation and
force construction use the current effective element geometry.

## Reynolds, qualification, compatibility, and identity

Reynolds-family targets use the slipstream-adjusted physical section speed:

```text
Re = V_section_physical * chord / nu
```

M2.6C qualification calls the same physical-flow, downwash, and finite-wing helpers as runtime
using the accepted same-stage propulsion output. Its alpha, speed, and Reynolds audit values
therefore match runtime.

Schemas v0-v6 retain their existing paths and fingerprints. Schema v7 with an empty interaction
list, and v7 interactions with factor zero, retain uncoupled physics. The v7 fingerprint records
resolved target membership/order and the velocity factor.

All committed M2.8D fixtures and calibration values are synthetic.
