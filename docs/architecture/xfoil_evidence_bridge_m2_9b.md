# M2.9B — XFOIL-to-Aerodynamic-Evidence Dataset Bridge

## Purpose

M2.9B provides the deterministic off-runtime bridge between a parsed XFOIL
polar import (M2.9A `XfoilPolarImport`) and the repository's existing
aerodynamic-evidence schema (`reference_aircraft_aerodynamic_evidence_v0`).

The bridge produces a single `polar_datasets[]` element as a
`serde_json::Value` that is directly compatible with the existing
`AerodynamicEvidenceLoader`.

## M2.9A vs M2.9B Boundary

| Concern | M2.9A | M2.9B |
|---------|-------|-------|
| Input | Raw XFOIL text file | `XfoilPolarImport` (parsed) |
| Output | `XfoilPolarImport` | `XfoilEvidenceDataset` (JSON) |
| Validates | Text structure, numeric sanity | Evidence schema compatibility |
| Convergence | Does NOT infer | Requires explicit caller supply |
| Runtime | No coupling | No coupling |

M2.9A proves "this text is structurally usable solver output."
M2.9B proves "this parsed output can be expressed in the evidence schema."
Neither proves convergence, physical validity, or runtime readiness.

## Core Type

`XfoilEvidenceDataset` — a deterministic, immutable bridge result that
serializes to exactly one `polar_datasets[]` element.

`XfoilEvidenceDatasetBuilder` — constructs the dataset with explicit
caller-supplied fields:

- `dataset_id` (required, stable ID)
- `method_id` (required, stable ID)
- `convergence_status` (required, explicit)
- `source_ids` (required, non-empty, ordered)
- `notes` (optional)

## Exact Sample Mapping

| XfoilPolarSample field | Evidence sample field | Transformation |
|------------------------|----------------------|----------------|
| `alpha_rad` | `alpha_rad` | None (exact) |
| `cl` | `cl` | None (exact) |
| `cd` | `cd` | None (exact) |
| `cm` | `cm` | None (exact) |

No recomputation, smoothing, interpolation, extrapolation, sorting, or
sample removal is performed. Source sample order is preserved exactly.

## Method Metadata Mapping

| Evidence field | Source |
|---------------|--------|
| `id` | Caller-supplied `method_id` |
| `solver_or_tool` | `XfoilSolverMetadata::solver_name()` |
| `exact_version` | `XfoilSolverMetadata::solver_version()` |
| `command_or_config` | `XfoilSolverMetadata::command_or_config()` |
| `convergence_status` | Caller-supplied (explicit) |

Absent metadata serializes as JSON `null`. The bridge never fabricates
default values.

## Transition-Input Semantics

| Evidence field | Source |
|---------------|--------|
| `assumptions` | `XfoilSolverMetadata::transition_assumptions()` |
| `ncrit` | `XfoilSolverMetadata::ncrit()` |
| `forced_transition_upper_x_over_c` | `XfoilSolverMetadata::forced_transition_upper_x_over_c()` |
| `forced_transition_lower_x_over_c` | `XfoilSolverMetadata::forced_transition_lower_x_over_c()` |

### Why Top_Xtr / Bot_Xtr Are NOT Forced-Transition Inputs

XFOIL outputs `Top_Xtr` and `Bot_Xtr` as **diagnostic results** — they
report where the solver predicted transition occurred for a given
operating point. They are output observations, not input assumptions.

The evidence schema's `forced_transition_upper_x_over_c` and
`forced_transition_lower_x_over_c` are **input parameters** — they
specify where the user forced transition to occur. These are fundamentally
different concepts:

- **XFOIL Top_Xtr/Bot_Xtr**: "the solver computed transition at this x/c"
- **Evidence forced_transition**: "the user commanded transition at this x/c"

M2.9B maps forced-transition inputs only from `XfoilSolverMetadata`,
where the caller explicitly supplied them. XFOIL diagnostic columns
(CDp, Top_Xtr, Bot_Xtr) remain available on the original
`XfoilPolarImport` but are not silently converted into evidence fields.

## Convergence-Status Semantics

The caller must explicitly supply `ConvergenceStatus` for the generated
dataset. Allowed values:

- `Converged` — solver reports convergence
- `Unresolved` — convergence not determined
- `Failed` — solver reports failure

M2.9B does NOT infer convergence from:
- Successful parsing
- Number of samples
- Monotonic alpha
- Finite coefficients
- XFOIL text content
- Absence of malformed rows

`NotApplicablePublished` is not a valid status for generated-solver
datasets (the existing evidence schema enforces this).

## Source-ID Semantics

- Source IDs are preserved in caller-supplied order
- Each ID must be a valid stable ID (`[a-z0-9_-]+`)
- Duplicate source IDs within one dataset are rejected
- Empty source ID lists are rejected

## Flow Conditions

Uses exactly the validated values from `XfoilSolverMetadata`:

- `reynolds`: from metadata (finite, positive)
- `mach`: from metadata (finite, non-negative)

Optional fields (`density_kg_m3`, `dynamic_viscosity_pa_s`,
`kinematic_viscosity_m2_s`) are serialized as `null`. The bridge does not
fabricate physical properties.

## Deterministic Serialization

`XfoilEvidenceDataset::to_json_value()` produces a `serde_json::Value`
matching the exact shape expected by the existing
`reference_aircraft_aerodynamic_evidence_v0` schema.

`XfoilEvidenceDataset::to_json_pretty()` produces a pretty-printed JSON
string.

Identical inputs produce identical outputs (bit-deterministic).

## Compatibility with AerodynamicEvidenceLoader

The bridge output is designed to be embedded directly into a complete
evidence artifact JSON and loaded by the existing
`AerodynamicEvidenceLoader::from_json_str()`. The end-to-end test proves
this compatibility.

## Runtime Boundary

M2.9B does NOT:
- Construct `sim_core::PolarTable`
- Modify `AircraftModel`
- Create `RuntimePolar` or `RuntimeReynoldsPolarFamily`
- Set `runtime_ready = true`
- Participate in the 500 Hz runtime path

## File Locations

- Module: `crates/model/src/reference_xfoil_evidence.rs`
- Tests: `crates/model/tests/xfoil_evidence_bridge_m2_9b.rs`
- Documentation: `docs/architecture/xfoil_evidence_bridge_m2_9b.md`

## Explicit Disclaimers

- M2.9B does not generate aerodynamic coefficients.
- M2.9B does not infer solver convergence.
- M2.9B does not modify runtime physics.
- M2.9B does not make the SIG Kadet LT-40 runtime-ready.

## Remaining Limitations

- The bridge produces a single dataset per import. Multi-polar campaigns
  require multiple bridge invocations.
- The bridge does not validate against an operating envelope. Coverage
  analysis remains a separate concern.
- The bridge does not perform evidence evaluation — that is the existing
  `AerodynamicEvidenceLoader`'s role.
