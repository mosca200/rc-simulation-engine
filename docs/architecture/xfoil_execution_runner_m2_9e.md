# M2.9E — Deterministic external XFOIL execution runner

## Scope and pipeline

M2.9E adds the offline process boundary before the existing evidence pipeline:

```text
M2.9E  airfoil coordinates + execution manifest + explicit executable
       -> deterministic stdin script -> external process -> raw polar
M2.9A  raw polar -> strict parsed import
M2.9B  parsed import -> generated-solver evidence
M2.9C  ordered evidence -> campaign and domain audit
M2.9D  validation manifest -> deterministic qualification reports
```

The command is:

```text
rcsim-app xfoil run-campaign \
  --manifest PATH \
  --xfoil-executable PATH \
  --output-dir PATH \
  [--timeout-seconds N]
```

`--timeout-seconds` defaults explicitly to 60 and must be greater than zero.
The M2.9D `validate xfoil-campaign` command remains available unchanged.

This layer does not create runtime polar tables, modify aircraft models, add
Reynolds interpolation, change finite-wing or trim behavior, or alter runtime
aerodynamic physics. It performs no network access and does not bundle or
generate real-aircraft data. In particular, no real Clark Y campaign or LT-40
dataset is included.

## Execution manifest schema version 1

The strict JSON manifest rejects unknown fields at every structure:

```json
{
  "schema_version": 1,
  "campaign_id": "synthetic-campaign",
  "airfoil_file": "inputs/airfoil.dat",
  "runs": [
    {
      "dataset_id": "synthetic-re-100000",
      "reynolds": 100000.0,
      "mach": 0.03,
      "alpha_start_deg": -10.0,
      "alpha_end_deg": 15.0,
      "alpha_step_deg": 0.5,
      "maximum_iterations": 100,
      "ncrit": 9.0
    }
  ],
  "coverage_request": {
    "required_reynolds_min": 100000.0,
    "required_reynolds_max": 200000.0,
    "required_alpha_min_rad": -0.1,
    "required_alpha_max_rad": 0.1,
    "require_converged": false
  }
}
```

Campaign and airfoil paths must be nonempty and runs must be nonempty. Dataset
IDs are unique stable IDs. Reynolds nodes are finite, positive, and strictly
increasing in manifest order so the generated M2.9D manifest is directly
validatable without sorting. Mach is finite and nonnegative. Ncrit is finite
and positive. Alpha bounds and step are finite; the step is nonzero and its
sign must match the requested direction. Equal start and end values are
rejected because this slice does not implement a separate single-point command.
Iteration limits must be nonzero. Coverage fields use M2.9C's canonical request
validation.

An absolute `airfoil_file` is read directly. A relative value resolves against
the manifest directory, never the process working directory. The successful
reports retain only the caller-supplied string. The input must be readable
UTF-8 and contain non-whitespace text; M2.9E intentionally does not claim to be
a full airfoil geometry validator.

## External executable trust boundary

The executable is always supplied explicitly. A relative executable path is
resolved from the caller's current directory before the child working directory
is changed; PATH is never searched implicitly. Production invokes the resolved
file directly through `std::process::Command`. It does not construct `cmd /C`,
`sh -c`, or any other shell command.

Each run receives its own index-named directory beneath a process-private
staging root inside the output directory. The runner writes the provided
airfoil contents to `airfoil.dat`, sets that directory as the child `current_dir`,
and asks XFOIL to write `polar.out`. The repository root is never the solver
working directory. Staging directory names and captured process output never
enter deterministic artifacts. Staging is removed after every campaign,
including incomplete solver-level campaigns.

Child stdin, stdout, and stderr are piped. The deterministic script is written
to stdin, while separate reader threads drain stdout and stderr to prevent pipe
deadlock. Up to 64 KiB from each stream is retained in memory for the process
boundary; these environment-dependent bytes are not serialized or written as
sidecars. A standard-library polling loop checks process completion every 10 ms.
At the timeout it kills and then waits for the child, ensuring it is reaped;
partial `polar.out` is never accepted.

## Deterministic XFOIL script

The pure script builder emits, in order:

```text
LOAD airfoil.dat
PANE
OPER
VISC <Re>
MACH <Mach>
ITER <maximum_iterations>
VPAR
N <Ncrit>
<blank line leaving VPAR>
PACC
polar.out
<blank dump-file line>
ASEQ <start> <end> <step>
PACC
QUIT
```

All finite floating-point values use Rust's locale-independent scientific
format with 17 digits after the decimal point. This always contains `.` and
round-trips the binary input without Debug formatting. Commands, newlines, and
blank submenu responses are fixed, so identical run specifications produce
byte-identical stdin.

## Process and polar outcomes

Execution states are distinct from aerodynamic convergence:

- `completed_parseable`
- `process_failed`
- `timed_out`
- `missing_polar_output`
- `unparseable_polar_output`

A zero OS exit status means only that the process ended successfully. After
that, `polar.out` must exist, be nonempty UTF-8, and parse through M2.9A's
canonical `parse_xfoil_polar` using metadata built from the exact run. Only then
are its unchanged bytes written as `polars/0000.polar`, `0001.polar`, and so on.
Parse success still says nothing about solver convergence or scientific
quality.

Solver-level failures do not abort independent later runs. Their ordered typed
outcomes are retained, but no completed validation manifest is produced.
Infrastructure failures such as invalid input, an unstartable executable, or
orchestration I/O failure abort with an operational error.

## Outputs and exit codes

A fully completed campaign produces:

```text
xfoil_execution.json
xfoil_execution.md
xfoil_validation_manifest.json
polars/0000.polar
polars/0001.polar
...
```

The execution report has schema version `1`, `generated_by` exactly
`rcsim-app xfoil run-campaign`, campaign identity, supplied airfoil path,
counts, typed overall status, and ordered run rows. Each row records the exact
run request, deterministic final polar path, typed process outcome, optional OS
exit code, and optional parsed sample count. Markdown is derived solely from
the same structured report.

- Exit `0`: every child completed with OS success, every polar passed M2.9A,
  and all deterministic artifacts were written.
- Exit `2`: at least one attempted solver run failed, timed out, omitted its
  polar, or produced an unparseable polar; execution reports are still written.
- Exit `1`: invalid or unreadable input, unsupported schema, executable start
  failure, output/staging error, serialization failure, or orchestration I/O
  failure.

No exit code represents XFOIL convergence.

## Generated M2.9D manifest and determinism

On complete execution, `xfoil_validation_manifest.json` is strict M2.9D schema
version `1`. It preserves run order and dataset IDs, uses index-derived method
and source IDs, references `polars/NNNN.polar`, names the solver `XFOIL`, leaves
its unknown version null, records the exact command script, and truthfully
records free transition with the requested Ncrit and no forced transition.
The caller's coverage request, including `require_converged`, is copied without
change.

Every generated dataset is always `unresolved`. Process success, file presence,
endpoint presence, and parse success are not convergence evidence. A generated
manifest with `require_converged: false` may qualify under M2.9D if its domain
is sufficient. With `true`, M2.9D correctly returns `NotQualified` until a later
explicit evidence process changes convergence status.

Visible ordering uses structs and vectors, never hash-map iteration. Reports
contain no timestamps, UUIDs, filesystem metadata, executable paths, resolved
airfoil paths, staging paths, stdout, or stderr. Given identical manifest and
airfoil contents and identical executable behavior, command scripts, execution
JSON, Markdown, validation manifest, and final polar bytes are byte-identical.

Successful XFOIL process execution and parseable polar output do not establish
solver convergence, scientific validity, airfoil fidelity, aircraft fidelity,
coverage qualification, or runtime readiness.
