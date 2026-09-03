# M2.9L — Bind Canonical XFOIL Evidence to Aircraft Runtime Family

## Purpose

Closes the runtime loop: canonical M2.9B evidence JSON → M2.9K loader →
replacement of exactly one named `RuntimeReynoldsPolarFamily` in an existing
`AircraftModel`. Aero-element bindings reference families by index, so the
replacement preserves the family index and all existing bindings remain valid.

## Scope

### In scope

- Replacing one named Reynolds polar family in an `AircraftModel`
- Reusing M2.9K JSON-to-runtime bridge directly
- Preserving family index for aero-element binding stability
- Returning Mach/provenance metadata for audit
- Physics fingerprint update on polar data change

### Out of scope

- M2.9G bundle manifests or filesystem access
- Schema changes
- Aircraft model creation or structural modification
- Coefficient fitting, smoothing, or resampling
- Clark Y or LT-40 data

## Pipeline

```
(&mut AircraftModel, family_id: &str, json_bytes: &[u8])
  │
  ├─ model.find_reynolds_family_index(family_id)
  │    └─ None → FamilyNotFound error
  ├─ build_xfoil_reynolds_polar_family_from_json(json_bytes)
  │    └─ M2.9K pipeline (deserialize → validate → build)
  └─ model.replace_reynolds_polar_family_at(index, family)
       └─ preserves family ID, replaces ReynoldsPolarFamily in-place
```

## Index Preservation

Aero elements bind to families via `RuntimeAeroPolarBinding::ReynoldsFamily {
family_index }`. The replacement operates in-place at the same index, so all
existing element bindings remain valid without re-resolution.

## Fingerprint Semantics

`AircraftModelFingerprint` hashes all Reynolds family node data (Reynolds
numbers, alpha grids, coefficients). Replacing a family's polar data changes
the fingerprint. Replacing with identical data produces an identical
fingerprint.

## Public API

```rust
pub fn bind_xfoil_evidence_to_reynolds_family(
    model: &mut AircraftModel,
    family_id: &str,
    json_bytes: &[u8],
) -> Result<XfoilEvidenceBindingResult, XfoilEvidenceBindingError>
```

### Result type

```rust
pub struct XfoilEvidenceBindingResult {
    family_index: usize,
    family_id: String,
    mach: f64,
    runtime_family: XfoilRuntimePolarFamily,
}
```

### Error type

```rust
pub enum XfoilEvidenceBindingError {
    FamilyNotFound { family_id: String },
    EvidenceJson(XfoilEvidenceJsonError),
}
```

## Files

| File | Role |
|---|---|
| `crates/model/src/xfoil_aircraft_family_binding.rs` | Binding implementation |
| `crates/model/src/runtime.rs` | `pub(crate)` family lookup/replace methods |
| `crates/model/src/lib.rs` | Module declaration and re-exports |
| `crates/model/tests/xfoil_aircraft_family_binding_m2_9l.rs` | Integration tests |
