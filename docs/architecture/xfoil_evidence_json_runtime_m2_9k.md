# M2.9K — Canonical XFOIL Evidence JSON → Runtime Polar Family

## Purpose

Provides a deterministic bridge from canonical M2.9B evidence JSON (the same
`polar_datasets[]` array written by the M2.9C campaign serializer) directly to
the existing M2.9H `XfoilRuntimePolarFamily`. This eliminates the need for
callers to manually reconstruct `XfoilEvidenceDataset` and
`XfoilEvidenceCampaign` objects before invoking the M2.9H builder.

This is a **JSON entry point only** — no new polar parsing, no coefficient
fitting, no resampling, no filesystem access.

## Scope

### In scope

- Deserializing canonical `polar_datasets[]` JSON
- Validating evidence invariants (convergence, Mach consistency, Reynolds ordering)
- Delegating to the existing M2.9H builder pipeline
- Typed errors for every failure mode

### Out of scope

- M2.9G bundle manifests or filesystem access
- Aircraft model assignment
- Simulation physics modifications
- Coefficient fitting, smoothing, or resampling
- Alpha-grid normalization
- Clark Y or LT-40 data

## Pipeline

```
&[u8] (canonical polar_datasets JSON)
  │
  ├─ serde_json::from_slice → Vec<CanonicalPolarDataset>
  ├─ pre_validate:
  │    ├─ non-empty array
  │    ├─ all convergence_status == Converged
  │    ├─ all mach equal (exact f64 equality to first dataset)
  │    └─ reynolds strictly increasing (no duplicates)
  ├─ construct_evidence_datasets:
  │    ├─ MetadataBuilder from flow_conditions + transition + method
  │    ├─ XfoilPolarImport::from_parts(metadata, samples)
  │    └─ XfoilEvidenceDatasetBuilder::build()
  ├─ XfoilEvidenceCampaignBuilder::build()
  └─ build_xfoil_reynolds_polar_family(&campaign)
       │
       └─ XfoilRuntimePolarFamily { family, mach }
```

## No Duplicated Polar Parsing

The bridge does NOT re-parse XFOIL text. It deserializes the canonical JSON
format directly and constructs `XfoilPolarImport` via a `pub(crate) fn
from_parts` constructor. The existing M2.9H builder performs all runtime
construction (PolarTable, ReynoldsPolar, ReynoldsPolarFamily).

## Convergence Policy — Fail Closed

Only `Converged` datasets are promotable. Pre-validation rejects non-converged
datasets before any type construction occurs.

## Determinism Guarantees

Given the same JSON bytes, the bridge always produces:

- Identical Reynolds node count and ordering
- Identical alpha sample ordering (preserved from JSON array order)
- Identical coefficients (lossless mapping)
- Identical Mach metadata
- Identical error variants and indices for invalid input

No `HashMap` iteration, no filesystem access, no timestamps, no randomness.

## Public API

```rust
pub fn build_xfoil_reynolds_polar_family_from_json(
    json_bytes: &[u8],
) -> Result<XfoilRuntimePolarFamily, XfoilEvidenceJsonError>

pub fn build_xfoil_reynolds_polar_family_from_json_str(
    json_str: &str,
) -> Result<XfoilRuntimePolarFamily, XfoilEvidenceJsonError>
```

### Error type

```rust
pub enum XfoilEvidenceJsonError {
    MalformedJson(serde_json::Error),
    EmptyDatasetArray,
    DatasetNotConverged { index, dataset_id, status },
    InconsistentMach { index, mach, expected_mach },
    DuplicateReynolds { previous_index, index, reynolds },
    ReynoldsNotIncreasing { previous_index, index, previous_reynolds, reynolds },
    InvalidMetadata { index, source },
    EvidenceBridge { index, source },
    CampaignConstruction(XfoilEvidenceCampaignError),
    // Delegated from M2.9H:
    RuntimeDatasetNotConverged { .. },
    RuntimeInconsistentMach { .. },
    PolarTableConstruction { .. },
    ReynoldsPolarConstruction { .. },
    ReynoldsPolarFamilyConstruction(..),
}
```

## Files

| File | Role |
|---|---|
| `crates/model/src/reference_xfoil_evidence_json.rs` | Bridge implementation |
| `crates/model/src/reference_xfoil.rs` | `from_parts` constructors (pub(crate)) |
| `crates/model/src/lib.rs` | Module declaration and re-exports |
| `crates/model/tests/xfoil_evidence_json_m2_9k.rs` | Integration tests |
