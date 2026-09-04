# Aircraft model format v8

V8 preserves every v7 field, unit, frame convention, and validation rule,
and adds one optional root array: `landing_gear`. A v7 document with
`"schema_version": 8` and no `landing_gear` key loads as a gear-free v8
model (empty runtime gear, zero ground wrench, no invisible wheels).
V7 documents reject `landing_gear` as an unknown field; v8 is never
implicitly downgraded.

```json
{
  "schema_version": 8,
  "landing_gear": [
    {
      "id": "nose-gear",
      "position_body_m": [0.6, 0.0, 0.35],
      "wheel_radius_m": 0.05,
      "normal_stiffness_n_per_m": 12000.0,
      "normal_damping_n_s_per_m": 800.0,
      "longitudinal_friction_coefficient": 0.6,
      "lateral_friction_coefficient": 0.9,
      "rolling_resistance_coefficient": 0.02,
      "max_brake_friction_coefficient": 0.0,
      "steering": "rudder",
      "max_steer_angle_rad": 0.45,
      "steerable": true,
      "braked": false
    }
  ]
}
```

## Contact fields

| Field | Type | Validation |
| --- | --- | --- |
| `id` | string | Unique nonempty `[a-z0-9_-]+`, own namespace, order preserved. |
| `position_body_m` | `[f64; 3]` | Finite FRD body metres (axle center). |
| `wheel_radius_m` | `f64` | Finite, `>= 0` (skids use `0`); defaults to `0`. |
| `normal_stiffness_n_per_m` | `f64` | Finite, `(0, 20000]`; re-validate above 12000 at 500 Hz. |
| `normal_damping_n_s_per_m` | `f64` | Finite, `[0, 1500]`; re-validate above 800 at 500 Hz. |
| `longitudinal_friction_coefficient` | `f64` | Finite, `>= 0`; defaults to `0.8`. |
| `lateral_friction_coefficient` | `f64` | Finite, `>= 0`; defaults to `0.8`. |
| `rolling_resistance_coefficient` | `f64` | Finite, `>= 0`; defaults to `0.02`. |
| `max_brake_friction_coefficient` | `f64` | Finite, `>= 0`; defaults to `0`; requires `braked`. |
| `steering` | enum | `fixed` (default) or `rudder`. |
| `max_steer_angle_rad` | `f64` | Finite, `[0, pi/2]`; defaults to `0`. |
| `steerable` | bool | Defaults `false`; must agree with `steering`. |
| `braked` | bool | Defaults `false`; must agree with brake authority. |

At most 16 contacts. `steering: rudder` without `steerable: true` (and vice
versa) fails; brake authority without `braked: true` fails. Unknown fields
are rejected (`deny_unknown_fields`). All load-time errors surface as
`ModelLoadError::InvalidLandingGearContact` (or structural errors for
unknown/duplicate fields), never as hot-loop checks.

## Fingerprint

Gear physics participates in the model physics fingerprint under the
`landing-gear:v1` domain (IDs, positions, radii, stiffness, damping, all
friction/brake mus, steering source, max steer, flags). Two aircraft
differing only in gear hash differently. Schema version itself is part of
the fingerprint domain, so v7/v8 equivalents do not collide.
