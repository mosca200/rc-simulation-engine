# Ground propulsion world demo

## Launch

From the repository root:

```text
cargo run --release -p rcsim-app -- render --model models/acro_electric_ground_demo/model.json --start-on-ground --throttle 0
```

Controls:

- `R` / `F`: increase / decrease throttle
- `A` / `D`: roll command
- `W` / `S`: pitch command
- `Q` / `E`: yaw command
- `Esc`: exit

## Authority and initialization

`acro_electric_ground_demo` is a `synthetic_test` model with
`reference_aircraft: null`. Its output is computed simulation behavior, not
measured aircraft or component evidence.

`--start-on-ground` requires authored landing gear and fails explicitly when
the selected model has none. Startup keeps the aircraft level with zero linear
and angular velocity. Its CG height is solved deterministically so the actual
landing-gear spring forces support the model weight on the physical flat NED
`z = 0` ground plane. Weight-on-wheels is evaluated from that contact solution
before the viewer starts.

The ground-start renderer uses flat visual terrain at the render-space image
of the same physical `z = 0` plane. The visual mesh is not a second collision
or terrain authority. Normal airborne launches retain rolling visual terrain.

## Known limitations

- The demo uses the production G1C GLB presentation path, including its
  authored materials and textures. The current GLB remains rigid because its
  geometry is not authored as separate movable primitives; G1E articulation
  remains a separate integration step.
- Wheels and suspension compression have no dedicated visual geometry.
- Ground authority is one infinite flat plane; there is no runway material,
  height-field collision, obstacle collision, or deformable terrain.
- The startup solution balances total vertical spring force. It does not solve
  a complete static pitch/roll trim or alter the ground dynamics.
- This synthetic fixture is intended to demonstrate ground contact and the
  production propulsion solver, not to establish real takeoff performance.
