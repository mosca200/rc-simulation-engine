# M2.9H — XFOIL Evidence to Reynolds Polar Family Bridge

## Purpose

Provides a deterministic, lossless conversion from validated XFOIL evidence
datasets (`XfoilEvidenceCampaign`) into the existing runtime polar primitive
(`sim_core::ReynoldsPolarFamily`).

This is a **bridge only** — no aircraft wiring, no physics changes, no
calibration claims.

## Scope

### In scope

- Reading canonical XFOIL evidence samples
- Mapping `XfoilPolarSample` → `sim_core::PolarSample`
- Constructing `PolarTable`, `ReynoldsPolar`, and `ReynoldsPolarFamily`
- Convergence gating (fail closed)
- Mach consistency enforcement

### Out of scope

- Aircraft model assignment
- Simulation physics modifications
- Coefficient fitting, smoothing, or resampling
- Alpha-grid normalization
- Trim qualification
- LT-40 calibration

## Conversion Pipeline

```
XfoilEvidenceCampaign
  │
  ├─ for each XfoilEvidenceDataset:
  │    ├─ verify convergence_status == Converged
  │    ├─ verify mach == common_mach (exact f64 equality)
  │    ├─ map samples: XfoilPolarSample → PolarSample
  │    ├─ PolarTable::new(mapped_samples)
  │    └─ ReynoldsPolar::new(dataset.reynolds(), table)
  │
  └─ ReynoldsPolarFamily::new(all_nodes)
       │
       └─ XfoilRuntimePolarFamily { family, mach }
```

## Convergence Policy — Fail Closed

Only datasets with `ConvergenceStatus::Converged` are promoted.

| Status | Behavior |
|---|---|
| `Converged` | Accepted |
| `Unresolved` | Rejected with `DatasetNotConverged` |
| `Failed` | Rejected with `DatasetNotConverged` |

`NotApplicablePublished` cannot appear on `XfoilEvidenceDataset` (rejected by
the dataset builder per M2.9B semantics).

## Mach Consistency

`ReynoldsPolarFamily` is indexed only by Reynolds number. All datasets in a
campaign must share the same Mach value. The bridge enforces exact `f64`
equality against the first dataset's Mach. No averaging, no silent selection.

## Determinism Guarantees

Given the same `XfoilEvidenceCampaign`, the bridge always produces:

- Identical Reynolds node count
- Identical Reynolds node ordering (preserved from campaign order)
- Identical alpha sample ordering (preserved from import order)
- Identical coefficients (lossless mapping)
- Identical Mach metadata

No `HashMap` iteration, no filesystem access, no timestamps, no randomness.

## No Hot-Path Changes

The bridge runs before runtime stepping. It does not modify:

- `ReynoldsPolarFamily::sample`
- `PolarTable::sample_clamped`
- Aerodynamic force evaluation
- RK4 stage physics
- Finite-wing induced physics
- Downwash physics

## Public API

```rust
pub fn build_xfoil_reynolds_polar_family(
    campaign: &XfoilEvidenceCampaign,
) -> Result<XfoilRuntimePolarFamily, XfoilRuntimePolarFamilyError>
```

### Result type

```rust
pub struct XfoilRuntimePolarFamily { /* opaque */ }

impl XfoilRuntimePolarFamily {
    pub fn family(&self) -> &ReynoldsPolarFamily;
    pub fn mach(&self) -> f64;
}
```

### Error type

```rust
pub enum XfoilRuntimePolarFamilyError {
    EmptyCampaign,
    DatasetNotConverged { index: usize, dataset_id: String, status: ConvergenceStatus },
    InconsistentMach { index: usize, mach: f64, expected_mach: f64 },
    PolarTableConstruction { index: usize, source: PolarError },
    ReynoldsPolarConstruction { index: usize, source: ReynoldsPolarFamilyError },
    ReynoldsPolarFamilyConstruction(ReynoldsPolarFamilyError),
}
```

## Files

| File | Role |
|---|---|
| `crates/model/src/reference_xfoil_runtime.rs` | Bridge implementation |
| `crates/model/src/lib.rs` | Module declaration and re-exports |
| `crates/model/tests/xfoil_runtime_family_m2_9h.rs` | Integration tests |
