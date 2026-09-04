# M2.8E deterministic propeller swirl

Schema v7 slipstream interactions optionally accept `swirl_velocity_factor`. Its default is
zero, preserving the M2.8D axial-only behavior and fingerprint. The value must be finite and
non-negative.

M2.8E reuses the M2.8D actuator-disk induced velocity `vi`; it introduces no new momentum
model. The authored tangential speed is:

```text
v_swirl = swirl_velocity_factor * vi
```

Let `a` be propeller local `+X` transformed into the body frame. Let `r` be the target element
position relative to the propeller position, projected onto the plane perpendicular to `a`.
For nonzero `r`, the body-frame swirl direction is:

```text
t = spin_sign * normalize(a cross r)
```

`spin_sign` is `+1` for `positive_about_local_x` and `-1` for
`negative_about_local_x`, matching the existing right-handed shaft-rotation convention. This
is also consistent with the existing propulsion reaction torque being opposite shaft spin.
Targets exactly on the propeller axis receive zero swirl because no unique tangent exists.

The physical-flow order for every RK4 stage is base element flow, axial M2.8D increment,
M2.8E tangential increment, M2.8C downwash rotation, then the M2.8B induced-alpha solve and
polar sample. Consequently swirl changes local velocity, alpha/beta, section dynamic pressure,
Reynolds number, and force direction as the existing quasi-2D element model permits. It never
edits coefficient tables or injects a yaw, roll, or pitch moment. Any moment follows naturally
from the aerodynamic force and the element geometry.

Target IDs remain resolved during loading. Runtime evaluation performs no allocation, string
lookup, filesystem access, serialization, or unsafe operation. Fingerprints retain the exact
M2.8D value when every swirl factor is zero; nonzero swirl semantics and authored factors are
fingerprinted explicitly.
