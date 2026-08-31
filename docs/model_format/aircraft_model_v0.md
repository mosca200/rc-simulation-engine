# Aircraft model format v0

Aircraft model format v0 is the first strict, versioned JSON authoring contract for RC Simulation
Engine. It describes immutable aircraft configuration. It does not contain simulation state.
The current loader continues to support this contract unchanged; the additive control-surface
extension is documented separately in [aircraft model v1](aircraft_model_v1.md).

The architectural boundary is deliberate:

```text
JSON authoring representation
  -> schema-version probe
  -> strict parse
  -> validation and reference resolution
  -> immutable AircraftModel
```

The `model` crate owns parsing and filesystem access. `sim_core` owns the validated physical types
and never reads model files. Loading may allocate; using the resolved model requires no parsing,
filesystem access, or string-based polar lookup.

## Minimal example

This complete example omits the optional propulsion and presentation sections:

```json
{
  "schema_version": 0,
  "model_id": "minimal-glider",
  "display_name": "Minimal Glider",
  "rigid_body": {
    "mass_kg": 1.0,
    "inertia_body_kg_m2": [
      [0.08, 0.0, 0.0],
      [0.0, 0.12, 0.0],
      [0.0, 0.0, 0.16]
    ]
  },
  "aerodynamics": {
    "polars": [
      {
        "id": "wing",
        "samples": [
          { "alpha_rad": -0.1, "cl": -0.5, "cd": 0.04, "cm": 0.0 },
          { "alpha_rad": 0.1, "cl": 0.5, "cd": 0.04, "cm": 0.0 }
        ]
      }
    ],
    "elements": [
      {
        "id": "wing-center",
        "position_body_m": [0.0, 0.0, 0.0],
        "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
        "area_m2": 0.5,
        "chord_m": 0.25,
        "polar_id": "wing"
      }
    ]
  },
  "controls": {
    "response": {
      "roll": { "rate": 1.0, "expo": 0.0 },
      "pitch": { "rate": 1.0, "expo": 0.0 },
      "yaw": { "rate": 1.0, "expo": 0.0 }
    },
    "servos": {
      "aileron": {
        "min_angle_rad": -0.3,
        "neutral_angle_rad": 0.0,
        "max_angle_rad": 0.3,
        "max_speed_rad_s": 4.0,
        "reversed": false
      },
      "elevator": {
        "min_angle_rad": -0.3,
        "neutral_angle_rad": 0.0,
        "max_angle_rad": 0.3,
        "max_speed_rad_s": 4.0,
        "reversed": false
      },
      "rudder": {
        "min_angle_rad": -0.3,
        "neutral_angle_rad": 0.0,
        "max_angle_rad": 0.3,
        "max_speed_rad_s": 4.0,
        "reversed": false
      }
    }
  }
}
```

## Root fields and versioning

| Field | Type | Required | Meaning |
| --- | --- | --- | --- |
| `schema_version` | integer | yes | Must be exactly `0`. |
| `model_id` | string | yes | Stable tooling identifier. |
| `display_name` | string | yes | Human-readable, non-physical label. |
| `rigid_body` | object | yes | Mass and complete inertia tensor. |
| `aerodynamics` | object | yes | Ordered polar tables and aerodynamic elements. |
| `controls` | object | yes | S5A response and conventional servo configuration. |
| `propulsion` | object or `null` | no | One complete S5B electric powertrain. |
| `presentation` | object or `null` | no | Non-physical presentation metadata. |

The loader determines `schema_version` before interpreting the remainder of the object. Version
zero selects this exact contract and is never reinterpreted as a later version. The current loader
also supports the separate schema-v1 and schema-v2 contracts; any other version produces
`UnsupportedSchemaVersion`. There is no implicit migration.

All v0 objects use strict unknown-field rejection. Misspelled, unknown, missing required, and
wrongly typed fields are errors. Duplicate object keys are structural errors rather than
last-value-wins updates. No physical field has an implicit default. JSON object formatting and
field order have no semantic significance, while array order is preserved.

## Units and coordinate frames

Physical field names carry their units. Values use `f64` and SI unless the suffix says otherwise:

- metres: `_m`;
- square metres: `_m2`;
- kilograms: `_kg`;
- inertia: `_kg_m2`;
- radians: `_rad`;
- radians per second: `_rad_s` or `_radps` where already established;
- volts, amperes, and ohms: `_v`, `_a`, and `_ohm`;
- the intentional RC-industry exception is motor `kv_rpm_per_v`.

The world convention remains NED and the rigid-body convention remains FRD: body `+X` is forward,
`+Y` is right, and `+Z` is down. The body origin is the centre of gravity. Every
`position_body_m` vector points from the CG to the component location and is expressed in FRD.

Orientations are active Hamilton unit quaternions. Arrays are always ordered `[w, x, y, z]`:

- `orientation_body_from_element_wxyz` maps an aerodynamic-element vector into body FRD;
- `orientation_body_from_prop_wxyz` maps a propeller-frame vector into body FRD.

Euler angles are not accepted. The loader checks that all four raw components are finite and that
the squared norm differs from one by no more than `1e-12`. It then preserves those exact components;
it never silently normalizes an invalid quaternion.

## Stable IDs and reference resolution

`model_id`, polar IDs, and aerodynamic-element IDs are nonempty ASCII strings containing only:

```text
a-z  0-9  -  _
```

Polar IDs must be unique among polars. Element IDs must be unique among elements. These are
separate namespaces. IDs are never synthesized, case-folded, trimmed, or replaced.

Each aerodynamic element names its polar with `polar_id`. During loading the exact string reference
is resolved to a compact `polar_index` into the ordered runtime polar vector. Missing references are
errors. Evaluation therefore needs neither a `HashMap<String, ...>` nor any other string lookup.
Polar tables, elements, and samples retain declaration order.

## Rigid body

`rigid_body` contains:

| Field | Shape | Validation |
| --- | --- | --- |
| `mass_kg` | scalar | Finite and greater than zero. |
| `inertia_body_kg_m2` | 3 by 3 nested array | Finite, symmetric within the core tolerance, and positive definite. |

The inertia rows and columns are body FRD axes and the complete tensor is supported, including
off-diagonal products of inertia. The loader constructs `RigidBodyParams::new`; the core remains the
single source of inertia validation and inverse-inertia construction. Initial pose, velocity, and
angular velocity belong to a scenario or runtime spawn state and are not model fields.

## Aerodynamic polars

`aerodynamics.polars` is an ordered array. Each entry has a stable `id` and an ordered `samples`
array. Each sample contains:

| Field | Meaning |
| --- | --- |
| `alpha_rad` | Section angle of attack in radians. |
| `cl` | Lift coefficient. |
| `cd` | Drag coefficient. |
| `cm` | Pitching-moment coefficient. |

`PolarTable::new` performs final numerical validation: at least two samples, finite values, strictly
increasing `alpha_rad`, and non-negative `cd`. The loader does not reorder samples, clamp
coefficients, repair alpha, or otherwise alter the table.

## Aerodynamic elements

Every entry in `aerodynamics.elements` contains:

| Field | Meaning and validation |
| --- | --- |
| `id` | Unique stable element ID. |
| `position_body_m` | Finite FRD position `[x, y, z]` from the CG. |
| `orientation_body_from_element_wxyz` | Finite unit Hamilton quaternion `[w, x, y, z]`. |
| `area_m2` | Finite and greater than zero. |
| `chord_m` | Finite and greater than zero. |
| `polar_id` | Exact reference to a declared polar. |

The loader creates the existing validated `AeroElement`; the runtime does not maintain a parallel
copy of its physical geometry. S6 does not add control-surface deflection or connect servo position
to an element.

## Controls

The required `controls` object represents the complete conventional S5A configuration.
`response.roll`, `response.pitch`, and `response.yaw` each contain finite `rate` and `expo` values in
the inclusive range `[0, 1]`.

`servos.aileron`, `servos.elevator`, and `servos.rudder` each contain:

| Field | Validation |
| --- | --- |
| `min_angle_rad` | Finite; strictly below neutral. |
| `neutral_angle_rad` | Finite; strictly between min and max. |
| `max_angle_rad` | Finite; strictly above neutral. |
| `max_speed_rad_s` | Finite and greater than zero. |
| `reversed` | Boolean installation reversal. |

The loader uses `AxisResponseConfig::new`, `ServoConfig::new`, and the existing conventional S5A
configuration types. V0 has no elevon, V-tail, differential, flap, multiple-servo, or general matrix
mixer representation.

## Optional electric propulsion

If `propulsion` is absent or `null`, runtime propulsion is `None`. If present, the entire single S5B
powertrain is required.

### Battery and motor

| Field | Validation |
| --- | --- |
| `battery.open_circuit_voltage_v` | Finite and greater than zero. |
| `battery.internal_resistance_ohm` | Finite and non-negative. |
| `motor.kv_rpm_per_v` | Finite, greater than zero, and representable by the core SI conversion. |
| `motor.winding_resistance_ohm` | Finite and greater than zero. |
| `motor.no_load_current_a` | Finite and non-negative. |

### Propeller

The propeller requires finite `position_body_m`, a valid
`orientation_body_from_prop_wxyz`, and finite positive `diameter_m`. Its `spin_direction` is one of:

```text
positive_about_local_x
negative_about_local_x
```

Viewpoint-dependent `cw` and `ccw` names are deliberately unsupported.

### Coefficient table

`coefficient_table.samples` contains `advance_ratio_j`, `ct`, and `cq`. The existing
`PropellerCoefficientTable::new` requires at least two finite samples, strictly increasing advance
ratio, and non-negative `ct` and `cq`. Samples are neither sorted nor corrected. The runtime owns the
validated `ElectricPropulsionConfig` and coefficient table. Multiple propulsors, rotor inertia,
reverse thrust, windmilling, and battery state are outside v0.

## Optional presentation metadata

`presentation.glb_path` is a nonempty UTF-8 relative-path string. Validation rejects:

- empty or whitespace-only strings;
- paths beginning with `/` or `\`;
- Windows drive prefixes such as `C:`;
- a `..` component using either slash style.

The path is retained verbatim. S6 does not open it, require it to exist, interpret GLB, or depend on
gltf, wgpu, or a renderer. Presentation metadata has no physical effect and is excluded from the
physics fingerprint.

## Loader API and runtime representation

The pure, easily testable entry point is:

```rust,ignore
let model = AircraftModelLoader::from_json_str(json)?;
```

The `model` crate also supplies a thin filesystem helper:

```rust,ignore
let model = load_aircraft_model("models/acro_electric_01/model.json")?;
```

Both return `Result<AircraftModel, ModelLoadError>`. Error variants distinguish filesystem and JSON
syntax failures, structural errors, unsupported versions, invalid or duplicate IDs, invalid rigid
body, polar, element, controls and propulsion configurations, unresolved polar references, and
invalid presentation paths. Context includes IDs, indices, component names, and original validated
core errors where applicable.

`AircraftModel` contains only immutable validated configuration:

```text
model identity and display name
RigidBodyParams
ordered RuntimePolar values
ordered RuntimeAeroElement values with resolved polar_index
ControlSystemConfig
optional RuntimeElectricPropulsion
optional PresentationMetadata
```

It contains no `RigidBodyState`, servo state, step counter, battery SOC, mutable cache, simulation,
or filesystem handle. The public file-v0 types are serializable authoring structures; the resolved
runtime model is intentionally not a JSON round-trip representation.

## Strict validation and no silent correction

Invalid input fails loading. In particular, the loader never:

- normalizes quaternions;
- sorts polar or propeller samples;
- clamps coefficients, rate, expo, or servo travel;
- repairs mass or inertia;
- replaces duplicate IDs;
- ignores missing references or unknown fields;
- substitutes physical defaults.

This policy makes authoring mistakes visible and gives future tooling a stable validation boundary.

## Model physics fingerprint

`AircraftModel::physics_fingerprint()` computes a 32-byte BLAKE3 digest over validated runtime
semantics rather than JSON text. Whitespace, indentation, and JSON object formatting therefore do
not affect it.

The canonical byte stream starts with the ASCII domain:

```text
rcsim:aircraft-model:v0
```

It then encodes, in order:

1. schema version as little-endian `u32`;
2. mass and the nine inertia entries in explicit row-major order;
3. polar count as little-endian `u64`, then each table's sample count and ordered
   `alpha_rad`, `cl`, `cd`, `cm` values;
4. element count, then each ordered position `[x,y,z]`, quaternion `[w,x,y,z]`, area, chord, and
   resolved polar index as `u64`;
5. roll, pitch, and yaw rate/expo pairs in that order;
6. aileron, elevator, and rudder min, neutral, max, speed, and reversal in that order;
7. a propulsion-presence byte;
8. when propulsion exists: battery source parameters, motor source parameters, propeller position,
   quaternion, diameter, spin-direction byte, coefficient-sample count, and ordered J/Ct/Cq samples.

Every `f64` is encoded with `to_bits().to_le_bytes()`. Lengths and indices use little-endian `u64`.
Boolean false/true and propulsion absent/present use `0`/`1`. Propeller spin uses `0` for positive
about local +X and `1` for negative about local +X.

The physics digest excludes `model_id`, `display_name`, polar IDs, element IDs, and all presentation
metadata because they do not alter dynamics. The resolved polar index still captures every physical
element-to-polar relationship. The model fingerprint is not connected to replay or
`SimulationFingerprint`; full-aircraft replay integration is deferred to S8.

## Deliberate v0 limits

S6 defines and validates a model; it does not make the assembled aircraft fly. V0 deliberately has
no aircraft dynamics assembly, aero-force summation, servo-to-aero coupling, control-surface
aerodynamics, propwash, multi-propulsion, environment or spawn state, renderer, GLB loading,
collision, terrain, joystick, schema compiler, editor, importer, or migration implementation.

A future version may add those capabilities through an explicit schema change or migration. A v0
loader never guesses how to interpret such future data.
