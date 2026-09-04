# Clark Y Reference XFOIL Campaign (M2.9F)

## Why Clark Y

The Clark Y is the first reference airfoil profile in this repository because it is one of the most
widely documented and historically significant airfoil sections in aviation. Its flat-bottom geometry
makes it a natural baseline for polar solver validation: the lower surface is a simple reference plane,
and the aerodynamic behavior is well-characterized across a broad Reynolds range. Using a universally
available, well-documented profile as the first reference campaign ensures that the XFOIL execution
pipeline (M2.9C through M2.9E) can be exercised against a geometry that any practitioner can
independently verify.

This is a **generic Clark Y reference solver campaign**, not an aircraft-specific calibration.

## Source Identity

| Field | Value |
|-------|-------|
| Airfoil | Clark Y |
| Source | UIUC Airfoil Data Site / Michael Selig coordinate database |
| File | `coord_seligFmt/clarky.dat` |
| Format | Selig (upper TE → LE → lower TE) |
| URL | <https://m-selig.ae.illinois.edu/ads/coord_seligFmt/clarky.dat> |
| Points | 121 |
| SHA-256 | `a30da5120fd7cd95c08541496cfc8607d58ef64f58198e25c799ffe6532f6e4d` |

## clarky.dat vs clarkysm.dat

The vendored asset is `clarky.dat`, the original Clark Y coordinates from the Selig database. This is
**not** the same as `clarkysm.dat`, which is a smoothed variant of the Clark Y produced by a different
process. The smoothed variant has different coordinate values and would produce different XFOIL results.
All provenance locks and SHA-256 checksums in this package refer specifically to `clarky.dat`.

## SHA-256 Provenance Lock

The vendored `clarky.dat` is locked to:

```
a30da5120fd7cd95c08541496cfc8607d58ef64f58198e25c799ffe6532f6e4d
```

Tests verify the vendored asset still matches this checksum. If the file is ever modified, all tests
that reference the provenance lock will fail, preventing silent geometry drift.

## Campaign Parameters

### Reynolds Grid

Six Reynolds nodes, logarithmically spaced to cover the low-to-moderate Reynolds range typical of
small RC and general-aviation sections:

| Index | Dataset ID | Reynolds |
|-------|-----------|----------|
| 0 | `clark-y-re-100000` | 100,000 |
| 1 | `clark-y-re-150000` | 150,000 |
| 2 | `clark-y-re-200000` | 200,000 |
| 3 | `clark-y-re-300000` | 300,000 |
| 4 | `clark-y-re-500000` | 500,000 |
| 5 | `clark-y-re-750000` | 750,000 |

### Alpha Sweep

- Start: -12.0 degrees
- End: +18.0 degrees
- Step: 0.5 degrees

XFOIL is not guaranteed to converge over the entire sweep, particularly at high alpha and low
Reynolds. M2.9E intentionally preserves convergence failures truthfully rather than discarding them.

### Mach

Mach = 0.0 for all runs. This is an incompressible / very-low-Mach reference campaign assumption,
not an aircraft-specific measured value.

### Maximum Iterations

200 iterations per alpha point. This is a generous solver budget for a reference campaign.

### Ncrit

Ncrit = 9.0 for all runs. This is an explicit reference-campaign solver assumption. It is **not** a
measured turbulence calibration for any specific aircraft (including the LT-40). Later evidence
campaigns may replace this value with aircraft-specific turbulence measurements.

### Convergence

`require_converged` is set to `false` in the coverage request. M2.9E intentionally emits
`convergence_status = unresolved` for runs where XFOIL does not converge. This reference package
does not weaken M2.9D semantics and does not mark generated runs as converged.

## Coverage Request

The M2.9D-compatible validation coverage request matches the campaign envelope exactly:

- Reynolds: [100000, 750000]
- Alpha: [-12 deg, +18 deg] in radians
- require_converged: false

## How to Execute

### Prerequisites

A caller-supplied XFOIL binary is required. This package does not download, build, or search for
XFOIL automatically.

### Running the Campaign

```bash
rcsim-app xfoil run-campaign \
  --manifest reference/xfoil/clark_y/campaign.json \
  --xfoil-executable /path/to/xfoil \
  --output-dir output/clark-y-reference-v1
```

Replace `/path/to/xfoil` with the actual path to the XFOIL executable on the local machine.

### Running Validation

After execution, the generated validation manifest can be fed into M2.9D:

```bash
rcsim-app validate xfoil-campaign \
  --manifest output/clark-y-reference-v1/xfoil_validation_manifest.json \
  --output-dir output/clark-y-reference-v1-validation
```

## Why This Is Not LT-40 Calibration

The Reynolds grid, Ncrit, and alpha sweep in this campaign are explicit engineering sampling inputs
chosen for broad reference coverage. They do not represent the exact operating envelope of any
specific aircraft. A future LT-40 calibration campaign will use aircraft-specific flight conditions,
measured turbulence levels, and the actual wing section geometry.

## Why Generated Polar Outputs Are Not Committed

This slice provides the **input package** for XFOIL execution, not the output. Polar results depend
on the XFOIL binary version, platform, and runtime convergence behavior. Committing fabricated or
stale polar values would violate the evidence-chain integrity that M2.9C through M2.9E are designed
to protect. Polar outputs are generated by running the campaign with a real XFOIL binary and are
tracked separately.
