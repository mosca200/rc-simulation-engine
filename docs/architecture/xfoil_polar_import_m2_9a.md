# M2.9A — Deterministic XFOIL Polar Import

## Purpose

M2.9A provides a strict, deterministic import layer for textual XFOIL polar
output files. It parses standard XFOIL polar text into an off-runtime,
immutable representation suitable for later integration into
`generated_solver` evidence.

This is infrastructure for the Clark Y polar campaign. It does NOT generate,
synthesize, or authorize any aerodynamic data.

## Scope Exclusions

**M2.9A does NOT:**

- Generate Clark Y polar data
- Authorize an LT-40 operating envelope
- Modify runtime aerodynamics
- Construct `sim_core::PolarTable`
- Modify `AircraftModel`
- Create `RuntimePolar` or `RuntimeReynoldsPolarFamily`
- Set `runtime_ready = true`
- Claim solver convergence
- Claim Clark Y fidelity
- Generate XFOIL command scripts
- Interpolate, extrapolate, smooth, or synthesize missing data

## Supported Textual Table Contract

The parser accepts standard XFOIL polar text output with the following
structure:

1. **Header/prologue**: Arbitrary text lines before the data table. These
   are ignored deterministically.

2. **Column header line**: A line containing the keyword `alpha`
   (case-insensitive). This marks the start of the numeric table.

3. **Optional separator**: A line consisting entirely of dashes and
   whitespace (at least 3 characters), immediately following the column
   header. If present, it is skipped.

4. **Data rows**: Whitespace-delimited numeric rows in source order.
   Each row must contain either:
   - **4 columns**: `alpha  CL  CD  CM`
   - **6 columns**: `alpha  CL  CD  CDp  Top_Xtr  Bot_Xtr`

   Note: XFOIL's standard 6-column output does not include CM. When 6
   columns are present, the parser reads `alpha, CL, CD, CDp, Top_Xtr,
   Bot_Xtr` and CM defaults to 0.0. When 4 columns are present, the
   fourth column is CM.

5. **Termination**: The data section ends at the first blank line or
   end-of-file.

## Degree → Radian Conversion

XFOIL outputs angle of attack in degrees. The parser converts to radians
deterministically:

```
alpha_rad = alpha_deg * PI / 180.0
```

where `PI` is `std::f64::consts::PI`. All other values (CL, CD, CM, CDp,
Top_Xtr, Bot_Xtr) are preserved exactly as they appear in the text.

## Strict Failure Behavior

Once the numeric data table has started, the parser enforces:

- **Malformed rows**: Non-numeric values in data rows cause immediate
  rejection. No silent skipping.
- **Non-finite values**: NaN and Inf are rejected.
- **Alpha ordering**: Alpha must be strictly increasing in source row
  order. The parser does not sort.
- **Duplicate alpha**: Identical alpha values are rejected.
- **Negative CD**: Drag coefficient must be >= 0. Negative zero (-0.0)
  is accepted.
- **Minimum samples**: At least two valid samples are required.
- **Column count**: Each row must have exactly 4 or 7 columns.

If any of these conditions are violated, parsing fails with a specific
error indicating the row number and reason.

## Solver Metadata

The parser requires caller-supplied metadata via `MetadataBuilder`.
Required fields:

- **Reynolds number**: Must be finite and positive (> 0).
- **Mach number**: Must be finite and non-negative (>= 0).

Optional fields:

- **Solver name**: e.g., "XFOIL"
- **Solver version**: Exact version string
- **Command/config text**: The exact XFOIL command script or configuration
- **Transition assumptions**: Free-text description of transition modelling
- **Ncrit**: Critical amplification exponent. When present, must be finite
  and positive (> 0).
- **Forced transition upper x/c**: When present, must be finite within
  [0, 1].
- **Forced transition lower x/c**: When present, must be finite within
  [0, 1].

The parser does NOT invent defaults. All required evidence metadata must
be explicitly supplied by the caller.

## Convergence Boundary

A successful parse means only:

> "This textual solver output is structurally usable."

It does NOT mean:

- The solver converged
- The data is physically valid
- The data is approved for runtime use
- The polar represents a real airfoil

Convergence status is a separate evidence concern that must be established
through other means (e.g., solver logs, residual history, expert review).

## Evidence / Runtime Boundary

M2.9A operates entirely in the **off-runtime evidence domain**:

- **Evidence domain**: `XfoilPolarImport`, `XfoilPolarSample`,
  `XfoilSolverMetadata` — immutable, parse-only representations.
- **Runtime domain**: `sim_core::PolarTable`, `RuntimePolar`,
  `AircraftModel` — mutable, performance-critical simulation state.

The parser has zero coupling to the runtime domain. It does not import,
reference, or depend on any runtime types. A later, separately reviewed
integration slice will bridge evidence to runtime when appropriate.

## Reproducibility

The parser is fully deterministic:

- Identical input text produces identical output structures.
- No randomness, no time-dependent behavior, no filesystem access.
- Floating-point operations use deterministic conversions only.
- Source row order is preserved exactly.

## Limitations

- **No command generation**: M2.9A imports completed solver output only.
  It does not generate XFOIL command scripts.
- **No interpolation**: Missing alpha values are not synthesized.
- **No extrapolation**: Data outside the parsed range is not inferred.
- **No smoothing**: Raw solver output is preserved as-is.
- **No stall synthesis**: Post-stall behavior is not modeled.
- **Single polar per file**: The parser handles one polar table per text
  input. Multi-polar files require separate parsing.
- **Standard format only**: Non-standard XFOIL output formats may not
  parse correctly.

## File Locations

- Module: `crates/model/src/reference_xfoil.rs`
- Tests: `crates/model/tests/xfoil_polar_import_m2_9a.rs`
- Documentation: `docs/architecture/xfoil_polar_import_m2_9a.md`

## Relationship to Other Milestones

- **M2.3A**: Reference aerodynamic evidence framework. M2.9A is independent
  but will later feed into `generated_solver` evidence datasets.
- **M2.8B**: (Future) Clark Y polar campaign. M2.9A provides the import
  infrastructure for M2.8B solver output.
- **Runtime**: M2.9A has no relationship to runtime aerodynamics. Runtime
  integration is a separate, future concern.
