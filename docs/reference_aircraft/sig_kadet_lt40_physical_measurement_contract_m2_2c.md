# M2.2C — SIG KADET LT-40 EGV Physical Measurement Contract & Geometry Closure Gate

## Scope and authority

M2.2C adds an evidence-ingestion layer for a future physical survey of one identified SIG KADET LT-40 EGV airframe. It does not create a runtime aircraft model, calibrate aerodynamics, select mass properties, or modify the 500 Hz simulation path. LT-40 mass-properties evidence and derivation remain assigned to M2.2D.

The machine-readable campaign is [`data/sig_kadet_lt40_egv_physical_survey_v0.json`](data/sig_kadet_lt40_egv_physical_survey_v0.json). Its `artifact_kind` is deliberately `physical_measurement_evidence_not_runtime_configuration`. The model crate loads it through `PhysicalSurveyLoader`, separate from `AircraftModelLoader`; its contents do not enter an `AircraftModel` or its physics fingerprint. A successfully parsed survey therefore has `runtime_ready = false` in every case.

This contract implements, rather than replaces, the measurement procedure in [M2.2B.1](sig_kadet_lt40_longitudinal_closure_m2_2b1.md). That document remains the field protocol for fixture setup, repeated readings, photographs, and airframe configuration.

## Artifact structure

The strict `reference_aircraft_physical_survey_v0` JSON schema has five conceptual regions:

1. `campaign` identifies the campaign, classification, manufacturer, family, exact variant, physical airframe, measurement date, and notes. `synthetic_non_reference` makes test fixtures unambiguously non-authoritative.
2. `datum` records whether the physical wing-root leading-edge datum was established, its definition, and supporting source and photograph references.
3. `provenance_sources` and `photographs` are registries with stable IDs. Every reference is resolved during loading; duplicate, malformed, or unknown IDs fail validation.
4. `comparison_baseline` contains the reconstructed original-kit values used only for cross-variant comparisons. They are not raw EGV observations and are never fallback geometry.
5. `raw_observations` contains nullable measurement series and battery/configuration metadata. Derived stations, arms, means, ranges, and readiness flags are not stored here; they are computed deterministically into `SurveyEvaluation`.

The committed artifact is an unmeasured campaign template. Observation fields are `null`, not zero and not numerical placeholders. Its original-kit comparison values are explicitly segregated under `comparison_baseline` and cite the M2.2B geometry reconstruction.

## Coordinate and sign conventions

The longitudinal convention is inherited from M2.2B/M2.2B.1:

- the physical EGV wing-root leading edge is `x_aft = 0`;
- positive longitudinal stations are aft;
- `H` is the horizontal-tail root leading-edge station aft of that datum;
- `V` is the explicitly defined vertical-tail root leading-edge station aft of that datum.

The horizontal-tail station is bilateral: left and right observations are retained separately. Top-view motor-axis angle is positive toward aircraft right. Side-view motor-axis angle is positive downward. Wing and stabilizer incidence use the common longitudinal datum and the field procedure defined by M2.2B.1. Angles are stored in radians and lengths in metres.

## Raw observations and validation

Each measurement series contains exactly three `f64` readings, positive instrument resolution, nonnegative stated uncertainty, datum text, optional notes, and source/photo reference lists. Length domains distinguish positive, nonnegative, and signed offsets. Angles must be finite and within ±π/2. Impossible values, non-finite JSON numbers, malformed metadata, invalid battery properties, unordered planform breakpoints, and unresolved evidence references fail closed.

The supported campaign domain is:

- physical wing-root leading-edge datum;
- left and right `H` readings and an asymmetry acceptance criterion;
- `V` readings;
- measured EGV wing quarter-chord station;
- optional direct horizontal- and vertical-tail quarter-chord stations;
- horizontal-tail span, root chord, tip chord, tip leading-edge offset, and optional intermediate breakpoints;
- vertical-tail height, root chord, tip chord, tip leading-edge offset, and optional intermediate breakpoints;
- wing incidence, stabilizer incidence, motor-axis top- and side-view angles;
- operational CG station; and
- battery identity, configuration, location, longitudinal station, and evidence.

Every present physical series is validated even if the campaign is incomplete. Geometry closure additionally requires evidence on each observation that it consumes.

## Repetition, uncertainty, and asymmetry

For readings `r1`, `r2`, and `r3`, evaluation reports the deterministic arithmetic mean, minimum, maximum, and range. The effective series uncertainty is conservative:

```text
u_series = max(stated_uncertainty, instrument_resolution / 2, range / 2)
```

The mean uses a scale-normalized calculation so large finite readings do not overflow merely from
intermediate addition. If a range, derived station, arm, planform integral, or propagated
uncertainty cannot be represented as finite `f64`, loading fails closed rather than emitting a
non-finite result or falling back to another geometry path.

For bilateral `H`, the closure station is the mean of the independently aggregated left and right means. The signed diagnostic remains available as:

```text
asymmetry = H_right - H_left
```

The bilateral uncertainty is the greater of the root-sum-square mean uncertainty and half the absolute asymmetry. Geometry cannot close until an explicit maximum asymmetry criterion exists and the observed magnitude satisfies it. The difference is therefore never silently averaged away.

Independent derived uncertainties use root-sum-square propagation. Planform uncertainty uses deterministic one-at-a-time ±effective-uncertainty perturbations of every input, takes the larger local deviation, and combines those deviations by root-sum-square. This is an auditable engineering closure rule, not a statistical claim about an unmeasured population.

## Deterministic geometry derivation

The horizontal and vertical planforms are piecewise linear in leading-edge offset and chord. The evaluator analytically integrates each segment to calculate the area-weighted local quarter-chord offset:

```text
x_qc,local = integral((x_LE(s) + 0.25 c(s)) c(s) ds)
             / integral(c(s) ds)
```

It then derives:

```text
x_qc,H = H + x_qc,local,H
x_qc,V = V + x_qc,local,V
l_H    = x_qc,H - x_qc,wing
l_V    = x_qc,V - x_qc,wing
```

A direct measured tail quarter-chord station may be used instead. If both direct and planform-derived stations exist, an explicit direct-versus-planform tolerance is required and the two must agree within that tolerance plus combined uncertainty. Nonpositive derived arms are physically impossible and rejected.

No equation reads an original-kit baseline to fill an EGV value. Missing EGV input therefore produces `None`, a missing-observation blocker, and an `UNKNOWN` comparison—not inherited RC-67 geometry.

## Cross-variant comparison

Each measured EGV local value is compared explicitly with its reconstructed original-kit counterpart. The output categories retain the established semantics:

- `CONFIRMED_IDENTICAL`: the absolute difference is within an explicitly supplied identity tolerance;
- `CONSISTENT_BUT_NOT_PROVEN`: it is outside that identity tolerance but within tolerance plus combined uncertainty, or no identity tolerance exists and uncertainty alone overlaps;
- `DIFFERENT`: the difference exceeds the applicable tolerance and uncertainty;
- `UNKNOWN`: either the EGV measurement or comparison reference is absent.

`CONSISTENT_BUT_NOT_PROVEN` is diagnostic only and is never promoted to numerical authority. Cross-variant classification has no path into tail-station or arm derivation.

## Closure and readiness gates

`geometry_ready` requires all simulation-critical placement evidence:

- physical airframe ID and measurement date;
- established wing-root-LE datum with source and photograph evidence;
- three left and three right `H` readings with evidence and acceptable asymmetry;
- three `V` readings with evidence;
- three EGV wing quarter-chord-station readings with evidence;
- a supported horizontal-tail quarter-chord station, from either an evidenced direct observation or an evidenced complete EGV planform;
- a supported vertical-tail quarter-chord station by the same rule; and
- positive derived horizontal and vertical quarter-chord arms.

`campaign_complete` additionally requires all EGV horizontal- and vertical-tail planform fields, both incidence series, both motor-axis angle series, the operational CG series, and an evidenced battery configuration and location. The report returns stable identifiers for every missing observation.

`runtime_ready` is hard-coded false. Later work must review the measurement record, decide whether it is authoritative, and deliberately construct simulation configuration through the existing versioned model path. M2.2C provides no automatic promotion mechanism.

## Remaining blockers after M2.2C

The committed EGV campaign remains deliberately unmeasured. Its current geometry blockers are: physical airframe ID; measurement date; evidenced wing-root-LE datum; left and right three-reading `H` series; an `H` asymmetry acceptance criterion; evidenced `V`; evidenced EGV wing quarter-chord station; evidenced direct or planform-derived horizontal-tail quarter-chord station; evidenced direct or planform-derived vertical-tail quarter-chord station; and consequently both tail arms.

The full campaign also still requires the real EGV horizontal-tail span/chords/leading-edge geometry, vertical-tail height/chords/leading-edge geometry, wing and stabilizer incidence, motor thrust-axis top- and side-view angles, operational CG station, and battery configuration/location evidence. Until those observations are made on an identified airframe under the M2.2B.1 protocol, the EGV geometry is unresolved and no real LT-40 runtime model exists.
