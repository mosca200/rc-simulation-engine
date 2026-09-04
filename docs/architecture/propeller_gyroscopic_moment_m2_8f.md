# M2.8F deterministic propeller gyroscopic moment

The serialized propeller accepts an optional `propeller_rotational_inertia_kg_m2`. It is the
rotor polar moment of inertia about propeller local `+X`; it must be finite and non-negative.
The default is zero, which preserves all pre-M2.8F model physics and fingerprints exactly.

At every RK4 stage, the existing quasi-static propulsion evaluation supplies the current shaft
speed. With propeller axis `a_body`, rotor inertia `I`, shaft speed `omega_shaft`, and the
existing right-handed spin sign, rotor angular momentum is:

```text
H_body = spin_sign * I * omega_shaft * a_body
```

The gyroscopic reaction moment added to the propulsion wrench is exactly:

```text
M_gyro_body = H_body cross omega_body
```

The addition changes only propulsion-wrench moment. Existing thrust, force, propeller load
torque, and its opposite aircraft reaction torque are unchanged. The complete propulsion
wrench is still combined with the aerodynamic wrench exactly once. Rotor spin-acceleration
torque (`-I * d(omega_shaft)/dt`) is intentionally absent because the shaft model remains
quasi-static.

Zero inertia, zero shaft speed, zero aircraft body rate, or body rate parallel to the propeller
axis produces an exact zero gyroscopic contribution. Runtime evaluation is deterministic and
allocation-free. A nonzero authored inertia is included in the physics fingerprint; zero and
absent inertia retain the prior fingerprint.
