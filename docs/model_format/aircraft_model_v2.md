# Aircraft model format v2

Aircraft model format v2 is the M2.1 authoring contract for aircraft classification and documented
reference-aircraft data. It preserves every physical simulation field and control-surface binding
from [v1](aircraft_model_v1.md), then adds two required root fields:

```json
{
  "schema_version": 2,
  "classification": "synthetic_test",
  "reference_aircraft": null
}
```

The fragment is explanatory; a complete model still requires all v1 physical fields. The loader
selects v0, v1, or v2 strictly from `schema_version`. It performs no implicit migration.

## Classification

`classification` is exactly one of:

- `synthetic_test`: architecture/regression fixture, not a validated real aircraft;
- `reference_aircraft`: model intended to be traceable to a real aircraft.

A `synthetic_test` document requires `reference_aircraft: null`. A `reference_aircraft` document
requires the reference object, but every individual identity and physical-reference value may be
unknown. This makes missing evidence explicit without fabricating values.

Legacy v0/v1 models load as `AircraftClassification::SyntheticTest`, preserving compatibility.

## Reference object

The strict `reference_aircraft` object contains:

| Field | Type | Meaning |
| --- | --- | --- |
| `identity` | object | Optional manufacturer, aircraft name, variant, stable reference ID, and notes. |
| `physical_specification` | object | Optional documented geometry and evidence links. |
| `provenance_sources` | array | Normalized registry of documentary sources. |

All identity fields are nullable. A present text field must contain non-whitespace text.
`stable_reference_id`, when present, uses `[a-z0-9_-]+`.

## Provenance registry

Every source has a unique stable `id`, a typed `source_type`, and nullable descriptive fields:

- `title`;
- `url`;
- `bibliographic_reference`;
- `notes`;
- `publication_date`;
- `retrieval_date`;
- `confidence` (`low`, `medium`, or `high`).

`source_type` is one of `manufacturer_documentation`, `measured`, `published_research`,
`airfoil_database`, `numerical_analysis`, `derived`, or `estimated`. Parameter objects reference
registry entries through ordered `source_ids`. Duplicate source IDs, malformed IDs, unresolved
references, and duplicate references within one parameter are rejected. A source object is stored
once, not copied into every scalar.

## Parameter status and unknown values

Every present documented parameter carries `status`, one of `measured`, `manufacturer_spec`,
`published`, `derived`, `estimated`, or `unknown`. `unknown` describes provenance/quality, not a
magic numeric sentinel. A physically unknown parameter is represented by JSON `null`; no numerical
default is substituted.

Independent documented scalars have this strict shape:

```json
{
  "value": 1.0,
  "status": "measured",
  "source_ids": ["measurement-log"]
}
```

The number above demonstrates shape and units only; it is not reference-aircraft data.

## Physical specification

The following nullable scalar fields are supported:

| Field | Validation |
| --- | --- |
| `wingspan_m` | finite and greater than zero |
| `reference_wing_area_m2` | finite and greater than zero |
| `aircraft_length_m` | finite and greater than zero |
| `aerodynamic_reference_chord_m` | finite and greater than zero |
| `wing_incidence_rad` | finite |
| `horizontal_tail_incidence_rad` | finite |
| `wing_dihedral_rad` | finite |

`cg_location` contains a finite FRD position, status/source references, and a typed datum kind:
`body_frame_origin_frd`, `wing_root_leading_edge`,
`mean_aerodynamic_chord_leading_edge`, `manufacturer_datum`, or `other`. The last two require a
nonempty datum description.

### No duplicate authority

The numerical simulation mass remains solely `rigid_body.mass_kg`. Consequently
`physical_specification.mass` stores only status and provenance for that authoritative value; it
does not contain a second mass number.

Likewise, numerical control travel remains defined solely by v1 servo limits and binding gains.
Each `control_surface_travel_limits` entry names a validated `control_surface_binding_id` and
attaches status/provenance to those authoritative limits without duplicating min/max values.
Unknown binding IDs and duplicate travel declarations for one binding are rejected. Invalid servo
travel continues to be rejected by the established controls validation.

## Runtime resolution

The loader validates and resolves every provenance source ID and control binding ID once. Runtime
reference evidence stores compact source/binding indices. `AircraftModel::classification()` and
`AircraftModel::reference_aircraft()` expose immutable query APIs. No source lookup, string parsing,
validation, or allocation occurs during aircraft stepping.

## Fingerprint compatibility

V2 adds no new dynamics. Its physics fingerprint therefore intentionally uses the v1 physical byte
stream and domain when v1 physical fields are identical. The following never affect the physics
fingerprint:

- classification and reference identity;
- reference dimensions, status, provenance, URLs, notes, bibliography, and dates;
- presentation presence and `glb_path`.

Mass, inertia, aerodynamic tables/geometry, control behavior, propulsion, and resolved binding
semantics remain fingerprinted exactly as in v1. A physical change still changes replay identity;
changing documentary evidence or a visual asset does not.
