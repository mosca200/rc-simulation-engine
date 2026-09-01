# Reference Aircraft Framework (M2.1)

## Purpose and policy

M2.1 provides the traceability boundary needed before adding any real reference aircraft. A
`reference_aircraft` model is intended to be reproducible from documented or measured information.
A `synthetic_test` model exists to exercise architecture and deterministic regression behavior.
Synthetic behavior is not evidence of real-world fidelity.

Unknown reference data must remain unknown. Authors must not invent a value to satisfy a schema or
create false precision. Optional values are represented as `null`, never by zero, a sentinel, or a
hidden default.

M2.1 deliberately contains no real-aircraft dataset, aerodynamic solver change, Reynolds-dependent
polar, XFOIL output, trim solver, stall validation, flight-test scenario, or renderer redesign.

## Authoring and runtime boundary

Schema v2 extends the existing v1 document rather than creating a parallel model system:

```text
strict v0/v1/v2 JSON
    -> schema-specific authoring DTO
    -> validation and ID resolution
    -> immutable AircraftModel
    -> AircraftSimulation initialization
```

V0/v1 documents remain supported and are classified as synthetic test models at runtime. V2
requires an explicit typed classification. Reference metadata is loaded into immutable, queryable
runtime structures, but simulation-critical mass, inertia, aerodynamics, controls, and propulsion
retain their established representations.

Provenance source IDs and control-surface binding references are resolved to indices at load time.
No provenance processing occurs in the 500 Hz step, and M2.1 adds no mutable state or steady-state
allocation.

## Identity and provenance

Reference identity supports optional manufacturer, aircraft name, variant, stable reference ID,
and notes. The model's existing `model_id` remains the tooling identity; a reference ID identifies
the documented real-aircraft subject when such an identifier is known.

The normalized source registry stores each source once. Typed source kinds distinguish manufacturer
documentation, measurement, published research, airfoil databases, numerical analysis, derived
work, and estimates. Optional title, URL, bibliography, notes, dates, and confidence preserve
practical audit information without becoming a generic scientific database.

Every present parameter carries a typed quality/status: measured, manufacturer specification,
published, derived, estimated, or unknown. Ordered source references are validated against the
registry.

## Reference physical specification

Optional documented fields cover wingspan, reference wing area, length, CG and its datum,
aerodynamic reference chord, wing/tail incidence, dihedral, mass evidence, and control-travel
evidence. Finite/positive constraints reject impossible authored dimensions while absent values
remain valid.

There is only one simulation-authoritative value for mass: `rigid_body.mass_kg`. Reference mass
metadata attaches quality and provenance to it without storing a second number. There is similarly
one authority for control travel: servo limits plus control binding gain. Reference travel entries
link evidence to a resolved binding instead of duplicating limits. This prevents documentary and
simulation values from silently diverging.

Other reference dimensions describe the real subject but do not currently feed the aerodynamic
solver. A later calibrated model must explicitly translate validated evidence into the existing
simulation-authoritative aero geometry; M2.1 does not perform that calibration.

## Physics and presentation separation

Physical/simulation configuration consists of rigid-body mass properties, the body/CG frame,
aerodynamic elements and data, control behavior, propulsion, and documented reference physical
specification. Presentation consists of GLB path, mesh, materials, and renderer-only metadata.

A GLB or visual mesh never defines or overrides mass, inertia, CG, aerodynamic geometry or
coefficients, control effectiveness, or propulsion. Visual accuracy does not imply physical
accuracy. Presentation is read by the application/renderer boundary only.

## Deterministic fingerprint

`AircraftModel::physics_fingerprint()` remains a BLAKE3 digest of simulation behavior. V2 uses the
same canonical v1 physics stream when physical fields match because classification and reference
documentation do not alter dynamics. Physical changes to mass, inertia, aerodynamic coefficients
or geometry, control behavior, propulsion, or binding relationships continue to change the digest.

Classification, identity, reference notes/dimensions, provenance content, status, bibliography,
dates, confidence, and GLB metadata are excluded. Regression tests cover both directions and retain
the existing `acro_electric_01` replay fingerprint.

## Adding a future real reference aircraft

1. Create a schema-v2 model with `classification: reference_aircraft` and a stable `model_id`.
2. Register every available manufacturer, measurement, research, database, analysis, derivation,
   or estimate source once using a stable source ID.
3. Enter only known identity/specification values; leave every unavailable field `null`.
4. Attach status and source IDs to each present parameter. Document the CG datum explicitly.
5. Put the authoritative simulation mass in `rigid_body.mass_kg`; attach its evidence through the
   reference `mass` object. Link control-travel evidence to existing bindings.
6. Derive simulation aero/control/propulsion data only in later validation work, with explicit
   sources and methodology. Do not infer physics from the GLB.
7. Run strict loader, fingerprint, replay, determinism, and zero-allocation tests before accepting
   the model.

The reference-aircraft roadmap is M2.2A dossier, M2.2B geometry reconstruction, M2.2B.1
longitudinal/cross-variant closure, M2.2C physical measurement contract and geometry gate, M2.2D
mass properties, M2.3A aerodynamic evidence preparation, M2.3B generic Reynolds polar-family
primitive, M2.3C generic stage-correct Reynolds runtime integration, remaining M2.3 reference-data
closure, M2.4 propulsion, M2.5 trim, and M2.6 automated physics validation. M2.1 itself introduces
no real numerical aircraft model.
