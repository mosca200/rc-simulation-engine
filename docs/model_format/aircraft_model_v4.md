# Aircraft model format v4

Schema v4 is an additive extension of schema v3. Its rigid body, Reynolds aerodynamics, controls,
control-surface bindings, classification, reference metadata, and presentation fields retain the
v3 structure and meaning. Only the optional propulsion authoring representation changes.

```json
{
  "schema_version": 4,
  "classification": "synthetic_test",
  "reference_aircraft": null,
  "propulsion": {
    "battery": {"open_circuit_voltage_v": 13.2, "internal_resistance_ohm": 0.031},
    "esc": {"series_resistance_ohm": 0.012},
    "motor": {"kv_rpm_per_v": 777.0, "winding_resistance_ohm": 0.057, "no_load_current_a": 0.9},
    "propeller": {
      "position_body_m": [0.28, 0.0, 0.0],
      "orientation_body_from_prop_wxyz": [1.0, 0.0, 0.0, 0.0],
      "diameter_m": 0.287,
      "spin_direction": "positive_about_local_x"
    },
    "coefficient_source": {
      "kind": "shaft_speed_map",
      "nodes": [{
        "shaft_speed_rad_s": 240.0,
        "samples": [
          {"advance_ratio_j": 0.0, "ct": 0.101, "cq": 0.0151},
          {"advance_ratio_j": 0.6, "ct": 0.061, "cq": 0.0102}
        ]
      }]
    }
  }
}
```

`esc.series_resistance_ohm` must be finite and non-negative. Zero selects legacy ideal-ESC
behavior.

`coefficient_source` is required when propulsion is present. `fixed_table` has a `samples` array
with the earlier schemas' validation and J-clamping semantics. `shaft_speed_map` has a nonempty
`nodes` array. Node speeds are finite, positive, in rad/s, and strictly increasing without
duplicates. Each node has its own valid J/Ct/Cq samples; J grids may differ. Shaft-speed queries
interpolate linearly between nodes and clamp outside the range with an explicit diagnostic.

All objects reject unknown fields. Schema v3 retains its original propulsion structure and is not
reinterpreted as v4. See
[`calibratable_electric_propulsion_m2_4b.md`](../architecture/calibratable_electric_propulsion_m2_4b.md)
for the runtime equations and policies.
