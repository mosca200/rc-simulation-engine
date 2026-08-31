# Aircraft model format v1

Aircraft model format v1 is the additive S6.1 authoring contract for control-surface coupling. It
preserves every physical and presentation field, unit, frame convention, and validation rule from
[aircraft model v0](aircraft_model_v0.md). The only schema addition is the required root array
`control_surface_bindings`.

The loader supports v0, v1, and v2 explicitly. A v0 document is never reinterpreted as v1: it loads
with an empty runtime binding list and retains the S6 v0 physics-fingerprint byte stream. V1 is
likewise not implicitly migrated to [v2](aircraft_model_v2.md).

## Difference from v0

```json
{
  "schema_version": 1,
  "model_id": "control-example",
  "display_name": "Control Example",
  "rigid_body": { "...": "same as v0" },
  "aerodynamics": { "...": "same as v0" },
  "controls": { "...": "same as v0" },
  "control_surface_bindings": [
    {
      "id": "aileron-left",
      "element_id": "wing-left-aileron",
      "actuator": "aileron",
      "deflection_gain": -1.0
    },
    {
      "id": "aileron-right",
      "element_id": "wing-right-aileron",
      "actuator": "aileron",
      "deflection_gain": 1.0
    }
  ]
}
```

The shown `"..."` values are explanatory placeholders, not valid model objects. Refer to the v0
document for the complete rigid-body, aerodynamics, controls, optional propulsion, and optional
presentation shapes.

An empty binding array is valid. Aerodynamic elements absent from the array remain fixed at their
declared base orientation.

## Binding fields

| Field | Type | Meaning and validation |
| --- | --- | --- |
| `id` | string | Unique stable binding ID using nonempty `[a-z0-9_-]+`. |
| `element_id` | string | Exact reference to one declared aerodynamic-element ID. |
| `actuator` | enum | Exactly `aileron`, `elevator`, or `rudder`. |
| `deflection_gain` | `f64` | Finite, nonzero, dimensionless scale; either sign and any finite magnitude are allowed. |

Binding IDs form their own namespace. Declaration order is preserved. The same aerodynamic element
may not be targeted by more than one binding in v1, although one actuator may intentionally drive
multiple distinct elements (for example both ailerons with opposite gains).

The loader rejects invalid or duplicate binding IDs, missing element references, duplicate
controlled elements, unknown actuator names, zero gain, and any non-finite/unrepresentable gain.
No ID, target, actuator, or gain is repaired or defaulted.

## Servo-to-surface semantics

For the actuator selected by a binding:

```text
servo_delta_rad = servo_angle_rad - servo_neutral_angle_rad
surface_deflection_rad = deflection_gain * servo_delta_rad
```

Subtracting neutral is mandatory: neutral servo position produces exactly zero surface deflection,
even when a servo's configured neutral angle is not numerically zero. `ServoConfig.reversed` acts in
the established controls pipeline; the binding gain then expresses the installation-specific
surface sign. V1 does not hardcode an aerodynamic meaning for positive elevator, left aileron, or
right aileron.

## Hinge and quaternion convention

A controlled aerodynamic element rotates around its own local `+Y` axis. In the established S4
element frame, local `+X` is chord-forward, local `+Y` is positive span/hinge direction, and local
`+Z` is down. A model author orients a rudder element so its local `+Y` follows the vertical hinge.

With Hamilton active quaternions in `[w,x,y,z]` order, the effective orientation is:

```text
delta_orientation = rotation_about_local_Y(surface_deflection_rad)
orientation_body_from_effective_element =
    orientation_body_from_base_element * delta_orientation
```

The multiplication order applies the delta in the element-local frame. `delta * base` has different
semantics and is not v1 behavior. The base `AeroElement` describes neutral geometry and remains
immutable.

V1 deliberately does not add an arbitrary hinge axis. Horizontal surfaces use spanwise local `+Y`;
a vertical surface is represented by orienting the entire element frame so local `+Y` is vertical.

## Resolved runtime representation

Loading resolves each `element_id` once into an element index. The immutable runtime form exposes:

```rust,ignore
pub enum ControlActuator {
    Aileron,
    Elevator,
    Rudder,
}

pub struct RuntimeControlSurfaceBinding { /* private fields */ }

binding.id() -> &str
binding.element_index() -> usize
binding.actuator() -> ControlActuator
binding.deflection_gain() -> f64

model.control_surface_bindings() -> &[RuntimeControlSurfaceBinding]
```

The future flight hot path therefore performs no `String -> element` lookup. Binding values are
immutable after initialization, and runtime actuator/aircraft state does not live in
`AircraftModel`.

## Physics fingerprint v1

V0 continues to use the exact S6 domain and byte stream:

```text
rcsim:aircraft-model:v0
```

V1 uses a distinct domain and schema word:

```text
rcsim:aircraft-model:v1
```

It encodes all pre-existing physical model semantics in the same canonical order as v0 and then:

1. binding count as little-endian `u64`;
2. for each binding in declaration order, resolved element index as little-endian `u64`;
3. actuator tag byte (`0` aileron, `1` elevator, `2` rudder);
4. `deflection_gain.to_bits().to_le_bytes()`.

Changing binding order, target relationship, actuator, or gain therefore changes the v1 physics
fingerprint. The non-physical binding ID is excluded, as are `model_id`, display text, polar/element
IDs, and presentation metadata. Presentation presence or `glb_path` still has no physics effect.

## Current approximation and limits

In S6.1 a mobile surface is approximated as a complete S4 `AeroElement` whose orientation rotates
about local `+Y`. This is a compact, physically coherent first assembly model, but it does not model
hinge moment, gaps, separated-flow interaction, nonlinear control effectiveness, flap/airbrake,
elevon, V-tail, arbitrary mixing matrices, or arbitrary hinge axes.

Schema v1 also adds no renderer, GLB loading, ground/collision, propwash, replay integration,
weather dynamics, calibration, or simulation state. Those remain separate later milestones.
