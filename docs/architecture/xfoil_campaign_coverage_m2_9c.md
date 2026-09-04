# M2.9C — Deterministic XFOIL campaign assembly and coverage audit

## Scope and boundary

M2.9C adds off-runtime infrastructure for assembling multiple synthetic or
future evidence-backed XFOIL datasets into one ordered campaign and auditing an
explicit aerodynamic envelope. The pipeline is:

```text
M2.9A raw XFOIL text
    -> XfoilPolarImport
M2.9B one XfoilPolarImport
    -> one XfoilEvidenceDataset
M2.9C ordered XfoilEvidenceDataset values
    -> XfoilEvidenceCampaign
    -> XfoilCampaignCoverage
    -> ordered polar_datasets JSON array
```

Campaign construction and coverage evaluation do not construct runtime polar
tables or Reynolds polar families. They do not modify aircraft model schemas,
runtime readiness, interpolation, extrapolation, or aerodynamic physics.

## Campaign construction

`XfoilEvidenceCampaignBuilder` requires at least one already-validated M2.9B
dataset. Dataset IDs must be unique, and Reynolds numbers must be strictly
increasing in caller-supplied order. Duplicate Reynolds values and decreasing
sequences are rejected.

The builder deliberately validates rather than sorts. Caller order records the
intended Reynolds-node series and is therefore part of the evidence. Sorting
would conceal an incorrectly assembled campaign and could change public output.
Validated order is preserved by `datasets()`, coverage audits, blocker output,
and JSON assembly. No missing Reynolds node is synthesized.

The first and last nodes provide the campaign minimum and maximum Reynolds
values. Coverage of a requested Reynolds interval means only that these extrema
bracket the request. It does not claim that arbitrary unrepresented internal
nodes are physically sufficient, nor does M2.9C define a new interpolation
model. The ordered node list remains visible through campaign datasets and the
per-dataset audit records.

## Explicit coverage request

`XfoilCampaignCoverageRequest` has no defaults. The caller supplies finite
minimum and maximum Reynolds numbers, finite minimum and maximum alpha values in
radians, and an explicit `require_converged` policy. Reynolds bounds must be
positive and strictly increasing. Alpha bounds must also be strictly
increasing. Invalid requests fail closed before an audit can run. There is no
hidden operating envelope or margin.

## Qualification semantics

A campaign is `Qualified` if and only if all of these statements are true:

1. campaign minimum Reynolds is no greater than required minimum Reynolds;
2. campaign maximum Reynolds is no less than required maximum Reynolds;
3. every campaign dataset's first imported alpha sample is no greater than the
   required minimum alpha;
4. every campaign dataset's last imported alpha sample is no less than the
   required maximum alpha; and
5. when `require_converged` is true, every dataset has the existing M2.9B
   `ConvergenceStatus::Converged` status.

The alpha rule is intentionally conservative. Each dataset is audited against
the complete requested alpha interval. The implementation neither intersects
domains and clips the request nor derives, smooths, interpolates, or
extrapolates additional support.

Parsing success, bridge success, finite samples, and campaign coverage never
imply solver convergence. With `require_converged` enabled, `Unresolved` and
`Failed` produce typed blockers. With it disabled, those statuses are preserved
in dataset audit records and do not block qualification.
`NotApplicablePublished` remains unavailable because M2.9B rejects that status
for generated-solver datasets.

## Audit records and blocker order

The coverage result preserves the request, actual campaign Reynolds range,
overall status, ordered per-dataset records, and every blocker. Each dataset
record includes its campaign index, dataset and method IDs, Reynolds number,
Mach number, convergence status, exact imported alpha bounds, and separate
lower/upper alpha coverage facts.

Blockers are accumulated without stopping at the first failure in this exact
order:

1. global lower-Reynolds blocker;
2. global upper-Reynolds blocker;
3. for each dataset in campaign order:
   1. convergence blocker, when convergence is required;
   2. lower-alpha blocker;
   3. upper-alpha blocker.

The blockers are typed enum values. No hash-map iteration or error-string
parsing determines public ordering.

## Deterministic JSON assembly

`to_polar_datasets_json_value()` returns a JSON array made by calling the
existing `XfoilEvidenceDataset::to_json_value()` on each dataset in exact
campaign order. M2.9C transforms no dataset, recomputes no samples, inserts no
timestamps, and generates no aerodynamic data.

`to_polar_datasets_json_pretty()` applies deterministic `serde_json` pretty
serialization to that array. Identical campaign inputs therefore produce
byte-identical output. An integration test embeds a multi-dataset array in a
synthetic `reference_aircraft_aerodynamic_evidence_v0` artifact and verifies
that the existing `AerodynamicEvidenceLoader` accepts it while runtime readiness
remains false.

## Explicit exclusions

M2.9C performs no filesystem scanning, XFOIL execution or command generation,
network access, runtime conversion, Reynolds interpolation, aerodynamic
interpolation or extrapolation, smoothing, or missing-node synthesis. It makes
no changes to aircraft or simulation crates.

This task does not download Clark Y coordinates, generate Clark Y coefficients,
add LT-40 aerodynamic values, or make any real aircraft runtime-ready. All test
fixtures are explicitly synthetic.
