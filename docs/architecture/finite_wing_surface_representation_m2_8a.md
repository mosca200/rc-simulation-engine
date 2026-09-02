# M2.8A — Finite-Wing Surface Representation

## Purpose

M2.8A introduces a validated aerodynamic-surface representation for future finite-wing physics. The current solver treats every aerodynamic element independently. That is insufficient for finite-wing physics because induced effects belong to a **lifting surface as a whole**, not independently to arbitrary strips.

M2.8A adds:

- A generic aerodynamic-surface grouping abstraction
- Resolution of string element IDs to compact runtime indices at load time
- Minimum geometric/configuration information required by M2.8B
- Derived surface area from member elements
- Derived aspect ratio from span and derived area
- Deterministic ordering preservation
- Fail-closed validation preventing invalid/double-counted membership

**M2.8A is representation-only. No finite-wing physics is implemented.**

## Why Element-Only Representation Is Insufficient

The current solver evaluates each aerodynamic element independently through the same runtime path:

```
steady controls → effective aero elements → evaluate_aircraft_instantaneous → wrench/derivative
```

Each element produces its own aerodynamic force based on its local polar and geometry. There is no concept of a **lifting surface** that spans multiple elements.

Finite-wing physics requires:

1. **Induced angle**: The wing's own circulation tilts the local flow
2. **Induced drag**: A spanwise lift distribution produces vortex drag
3. **Lift-slope correction**: The effective lift curve slope is reduced from 2D section values
4. **Spanwise coupling**: What happens at one strip affects neighboring strips

These effects are **surface-level properties**, not element-level properties. Without a surface abstraction, M2.8B would have no well-defined entity to apply corrections to.

## Schema V5

### New Constant

```rust
pub const AIRCRAFT_MODEL_SCHEMA_VERSION_V5: u32 = 5;
```

### File Structure

`AircraftModelFileV5` preserves v4 aircraft fields and propulsion semantics, with v5 aerodynamics:

```rust
pub struct AircraftModelFileV5 {
    pub schema_version: u32,
    pub model_id: String,
    pub display_name: String,
    pub classification: AircraftClassificationFileV2,
    pub reference_aircraft: Option<ReferenceAircraftFileV2>,
    pub rigid_body: RigidBodyFileV0,
    pub aerodynamics: AerodynamicsFileV5,  // NEW: includes surfaces
    pub controls: ControlsFileV0,
    pub control_surface_bindings: Vec<ControlSurfaceBindingFileV1>,
    pub propulsion: Option<PropulsionFileV5>,
    pub presentation: Option<PresentationFileV0>,
}
```

### AerodynamicsFileV5

Preserves v3/v4 fields and adds `surfaces`:

```rust
pub struct AerodynamicsFileV5 {
    pub kinematic_viscosity_m2_s: f64,
    pub polars: Vec<PolarFileV0>,
    pub polar_families: Vec<ReynoldsPolarFamilyFileV3>,
    pub elements: Vec<AeroElementFileV3>,
    pub surfaces: Vec<AeroSurfaceFileV5>,  // NEW
}
```

## Generic Surface Semantics

`AeroSurfaceFileV5` represents a finite aerodynamic surface:

```rust
pub struct AeroSurfaceFileV5 {
    pub id: String,
    pub element_ids: Vec<String>,
    pub span_axis_body: [f64; 3],
    pub span_m: f64,
    pub span_efficiency_factor: f64,
}
```

**Design decisions:**

- **No semantic categories** (wing, horizontal_tail, vertical_tail). The physics abstraction is generic — a surface may represent any finite lifting surface.
- **No authored `area_m2`**. Surface area is derived from member element areas.
- **No authored `aspect_ratio`**. AR is derived as `span_m^2 / surface_area_m2`.

## Surface Membership

### Validation Rules

1. **Surface IDs**: non-empty, unique, follow `[a-z0-9_-]+` convention
2. **element_ids**: non-empty
3. **Each element ID** must resolve to an existing aero element
4. **No duplicate member** within the same surface
5. **Cross-surface duplicate membership is rejected** — an element may belong to at most ONE surface

### Why Cross-Surface Rejection Is Critical

M2.8B will apply finite-wing correction to surface members. If an element belonged to multiple surfaces, the correction would be applied multiple times, producing incorrect physics.

### Unassigned Elements Are Valid

Elements that belong to NO surface are allowed and continue behaving exactly as they do today. This enables:

- Fuselage-like aero elements
- Intentionally uncorrected elements
- Partial migration of synthetic models

## Normalized Span Axis

`span_axis_body` is an explicit BODY-FRAME direction describing the surface span direction.

**Validation:**

- All three components finite
- Vector norm finite
- Norm strictly > 1e-12 (numerical zero threshold)

**At runtime, the span axis is stored normalized.** Authoring input does not need to be unit length; normalization occurs deterministically during model loading.

## Span

`span_m` is the physical surface span required by later finite-wing physics.

**Validation:**

- Finite
- Strictly > 0
- No arbitrary upper limit

Span is authored, not derived from element positions. The authored span is the authoritative physical surface span.

## Span Efficiency Factor

`span_efficiency_factor` is the finite-wing span-efficiency parameter to be consumed by M2.8B.

**Validation:**

- Finite
- Strictly > 0
- **No arbitrary upper cap** (e.g., no `e <= 1.0` or `e <= 1.5`)

There is no sufficiently general mathematical reason for a hard schema cap. The parameter is documented conservatively as the finite-wing span-efficiency parameter.

## Derived Area

For each surface:

```
surface_area_m2 = sum(area_m2 of all resolved member elements)
```

**Validation:**

- Result must be finite
- Result must be strictly > 0

Uses deterministic authored member ordering. Does NOT derive planform area from `span * chord`. Does NOT introduce taper/sweep approximations.

## Derived Aspect Ratio

```
aspect_ratio = span_m^2 / surface_area_m2
```

**Validation:**

- Result must be finite
- Result must be strictly > 0

No arbitrary AR limits. No clamping. No "correction" of suspicious values. Fail closed if arithmetic is non-finite.

## Resolved Runtime Indices

Element IDs are resolved to element indices **once during model loading**. Runtime code does not repeatedly search element IDs by String.

Runtime surface membership exposes compact indices into `AircraftModel::aero_elements()`. Author-specified membership ordering is preserved.

## Backward Compatibility

All existing schemas (v0, v1, v2, v3, v4) continue loading exactly as before. Existing fixtures do not require modification.

For old models:

```rust
AircraftModel::aero_surfaces() // returns empty slice
```

No auto-generated surfaces. No heuristic migration.

## Deterministic Ordering

Identical inputs produce identical outputs:

- Same model JSON
- Same surface ordering
- Same member ordering
- Same fingerprint

No timestamps, random IDs, wall clock, or stochastic behavior.

## Fingerprint Semantics

The v5 fingerprint includes, in deterministic order:

- Surface count and order
- Member indices and order
- Normalized span axis
- `span_m`
- `span_efficiency_factor`
- Derived area
- Derived aspect ratio

**Changing a v5 surface's physics-authoritative finite-wing configuration changes the model fingerprint.**

V0-v4 fingerprint results are unchanged.

## Section-2D Polar Contract

For M2.8B finite-wing correction:

> A surface configured for finite-wing correction represents elements whose polar bindings are interpreted as LOCAL SECTION / quasi-2D aerodynamic data.

**M2.8A does NOT introduce:**

- A "finite_wing_3d polar" mode
- Automatic dimensionality detection from CL/CD values

The contract is narrowly defined: member element polars are section data.

## What M2.8A Does NOT Do

M2.8A does **not** implement:

- Induced angle
- Induced drag
- Lift-slope correction
- Downwash
- Wing-tail interaction
- Propwash
- Lifting-line theory
- Finite-wing force modification
- Real-aircraft calibration

**M2.8B will be the first slice allowed to consume the surface representation in runtime aerodynamic physics.**

## Limitations

- No automatic surface inference from element geometry
- No wing/tail semantic classification
- No real-aircraft geometry claims
- No normalized aerodynamic coefficient derivatives
- No static margin computation

## Runtime Representation

```rust
pub struct RuntimeAeroSurface {
    id: String,
    element_indices: Vec<usize>,
    span_axis_body: Vec3,        // normalized
    span_m: f64,
    span_efficiency_factor: f64,
    area_m2: f64,                // derived
    aspect_ratio: f64,           // derived
}
```

Accessors:

- `id() -> &str`
- `element_indices() -> &[usize]`
- `span_axis_body() -> &Vec3`
- `span_m() -> f64`
- `span_efficiency_factor() -> f64`
- `area_m2() -> f64`
- `aspect_ratio() -> f64`

## Summary

M2.8A introduces a validated, deterministic aerodynamic-surface representation:

- Groups existing aerodynamic elements into surfaces
- Resolves string IDs to compact indices at load time
- Validates membership exhaustively (fail-closed)
- Derives area and AR from authoritative sources
- Preserves v0-v4 backward compatibility
- Does NOT change current aerodynamic physics
- Does NOT implement finite-wing correction
- Provides the foundation for M2.8B finite-wing physics
