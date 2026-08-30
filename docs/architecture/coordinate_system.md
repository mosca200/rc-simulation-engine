# Coordinate system and units

The simulation world is a right-handed local North-East-Down (NED) frame: +X is north, +Y is east, and +Z is down. The local field is the origin. Gravity is therefore `[0, 0, +9.80665] m/s²` by default.

The rigid-body frame is right-handed Forward-Right-Down (FRD): +X is forward, +Y is right, and +Z is down. Positive roll, pitch, and yaw rates follow the right-hand rule about body X, Y, and Z respectively. Angular velocity is always `angular_velocity_body_radps` in the body frame.

`orientation_world_from_body` is a right-handed Hamilton unit quaternion with conceptual component order `[w, x, y, z]`. It is an active rotation from body to world:

```text
v_world = orientation_world_from_body * v_body
```

For example, identity leaves `[1, 0, 0]` unchanged. A +90° rotation about world/body-down Z at identity maps body-forward `[1, 0, 0]` to world-east `[0, 1, 0]`. With body-frame angular velocity, its derivative is `q_dot = 0.5 * q ⊗ [0, omega_body]`.

The flight core uses SI units exclusively: metres, seconds, kilograms, m/s, m/s², radians, rad/s, newtons, N·m, watts, and kg/m³. Field names carry unit and frame information. Euler angles are not state.

A future renderer must perform an explicit sim-to-render coordinate and precision conversion at its boundary.
