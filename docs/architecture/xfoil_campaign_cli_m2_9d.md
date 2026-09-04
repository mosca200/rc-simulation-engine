# M2.9D — Deterministic XFOIL campaign CLI and reporting

## Scope and layer boundary

M2.9D is the offline application layer around the existing evidence stages:

```text
M2.9A  raw XFOIL text -> strict parsed import
M2.9B  parsed import -> one generated-solver evidence dataset
M2.9C  ordered datasets -> validated campaign -> coverage audit
M2.9D  manifest + files -> M2.9A/B/C orchestration -> reports
```

The command is:

```text
rcsim-app validate xfoil-campaign --manifest PATH --output-dir PATH
```

It does not run XFOIL, generate command scripts, download data, create runtime
polars, interpolate or extrapolate coefficients, alter aircraft schemas, or
change runtime aerodynamic physics. It does not download Clark Y coordinates,
generate Clark Y coefficients, or invent LT-40 aerodynamic values.

## Strict manifest schema version 1

Every manifest structure rejects unknown fields. Only schema version `1` is
supported. A representative manifest is:

```json
{
  "schema_version": 1,
  "campaign_id": "synthetic-campaign",
  "datasets": [
    {
      "polar_file": "polars/re-100000.txt",
      "dataset_id": "synthetic-re-100000",
      "method_id": "synthetic-xfoil-method",
      "convergence_status": "converged",
      "source_ids": ["synthetic-source"],
      "notes": "optional text",
      "reynolds": 100000.0,
      "mach": 0.03,
      "solver_name": "optional solver name",
      "solver_version": "optional exact version",
      "command_or_config": "optional command or configuration",
      "transition_assumptions": "optional assumptions",
      "ncrit": 9.0,
      "forced_transition_upper_x_over_c": 0.8,
      "forced_transition_lower_x_over_c": 0.7
    }
  ],
  "coverage_request": {
    "required_reynolds_min": 100000.0,
    "required_reynolds_max": 200000.0,
    "required_alpha_min_rad": -0.1,
    "required_alpha_max_rad": 0.1,
    "require_converged": true
  }
}
```

All evidence metadata is explicit. Reynolds, Mach, and convergence are never
inferred from filenames or polar text. Accepted convergence values are
`converged`, `unresolved`, and `failed`. `not_applicable_published` is absent
from the manifest type because M2.9B generated-solver evidence cannot use a
published-data status.

The app passes solver metadata through M2.9A's `MetadataBuilder`, each parsed
import through M2.9B's `XfoilEvidenceDatasetBuilder`, and the complete ordered
vector through M2.9C's `XfoilEvidenceCampaignBuilder`. The dataset array must be
non-empty; dataset IDs must be unique; and Reynolds nodes must be strictly
increasing in manifest order. The app never sorts or silently deduplicates.
The explicit coverage request is validated by
`XfoilCampaignCoverageRequest`.

## Polar path semantics

An absolute `polar_file` path is used directly. A relative path is joined to
the directory containing the manifest, independently of the process working
directory. Successful reports retain the exact manifest-provided path string;
they do not add the resolved machine-specific absolute path.

## Qualification and exit codes

Every dataset must cover both requested alpha endpoints. Global Reynolds
blockers precede dataset blockers. Per dataset, blocker order is convergence,
lower alpha, then upper alpha. With `require_converged: true`, `unresolved` and
`failed` block qualification. With `false`, those statuses remain visible but
do not independently block.

M2.9C audits a Reynolds range whose request bounds must be strictly increasing.
A one-node campaign therefore cannot bracket that range and is
`NotQualified`; M2.9D does not special-case or weaken this inherited rule.

- `0`: completed with status `qualified`.
- `2`: completed with status `not_qualified`; reports are written.
- `1`: CLI, manifest, input, parsing, canonical validation, serialization, or
  I/O error.

Per-dataset errors preserve the manifest index, dataset ID, and polar path.
Only `main.rs` translates the typed `NotQualified` outcome to exit code `2`.

## Deterministic report schema

Every completed analysis writes exactly:

- `xfoil_campaign.json`
- `xfoil_campaign.md`
- `polar_datasets.json`

`xfoil_campaign.json` has strict schema version `1` and contains
`schema_version`, `generated_by`, `campaign_id`, `manifest`, `summary`,
`coverage_request`, `campaign_reynolds_range`, ordered `datasets`, ordered
tagged `blockers`, and typed `status`. `generated_by` is exactly
`rcsim-app validate xfoil-campaign`; statuses are `qualified` and
`not_qualified`. Report deserialization rejects unknown fields and versions
other than `1`.

Dataset rows contain index, identities, manifest polar path, Reynolds, Mach,
convergence status, sample count, alpha bounds, and separate lower/upper alpha
coverage facts. Blocker `kind` values preserve the M2.9C variant identity and
order. Markdown is derived only from this structured report object, without
rerunning analysis.

`polar_datasets.json` is the pretty serialization of
`campaign.to_polar_datasets_json_value()` without transformation. It preserves
manifest order and M2.9B's `generated_solver` evidence class.

All report bytes are constructed before output-directory creation. Visible
ordering comes from structs and vectors, never hash-map iteration. The reports
contain no timestamps, current dates, random IDs, file modification times, or
resolved absolute paths. Identical manifest and polar contents produce
byte-identical outputs.

Coverage qualification only proves the requested evidence-domain rules
implemented by M2.9C; it does not prove solver correctness, airfoil fidelity,
aircraft fidelity, or runtime readiness.
