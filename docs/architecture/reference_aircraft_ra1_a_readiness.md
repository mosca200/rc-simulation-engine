# RA1-A: Reference Aircraft readiness gate

RA1-A answers a narrow pre-campaign question: does a *specific physical aircraft in a specific operational configuration* have traceable evidence sufficient to begin physical-model validation? It does not certify that the simulation matches reality or that flight testing is safe.

## API and meaning of READY

`model::evaluate_reference_aircraft_readiness(ReferenceReadinessInput)` is an offline, pure evaluation of an already validated `AircraftModel` and optional artifacts loaded by the existing strict survey, mass-properties, aerodynamic, and propulsion loaders. It does not access the filesystem, alter a model, install evidence into runtime polars, or run in the flight step. The caller supplies the required alpha interval; the gate invents no operating envelope or physical defaults.

The returned `ReferenceAircraftReadiness` includes model/configuration IDs, the unchanged physics fingerprint, eight ordered `DomainReadiness` records, typed `ReadinessEvidence` markers, typed `ReadinessFinding` reasons with optional subject IDs, and flattened findings. `overall_status` refers **only to physical-model evidence readiness**. It is `READY` only when every applicable physical-model domain is ready. `BLOCKED` takes precedence over `INCOMPLETE`; absent but expected evidence is incomplete, while contradictory, synthetic, unresolved, non-finite or incompatible evidence blocks. Propulsion is `NOT_APPLICABLE` only when the validated runtime model has no propulsion.

`FlightTestData` is a separate domain. RA1-A has no loader or qualification protocol for telemetry/flight-test records, so it always reports `INCOMPLETE` with `FlightTestEvidenceNotEvaluated`. It is deliberately excluded from `overall_status`: a physical model can be evidence-ready before the first flight-test dataset exists. `READY` must never be read as flight-test validation readiness.

Runtime validity means the strict model loader accepted the model and the simulation can use its validated runtime data. Reference readiness is stronger and different: it requires physical identity and external evidence. Existing legacy and synthetic models remain loadable, but `SyntheticTest` models and `SyntheticNonReference` artifacts cannot pass the physical gate. Test-only fixtures may exercise the status-combination rule; that does **not** assert a real aircraft is ready.

## Physical identity and domains

The requested airframe ID and operational-configuration ID are explicit. The model's stable reference ID identifies the model/variant, not the individual airframe. Survey airframe identity, mass-campaign airframe/configuration and geometry-campaign link, and exact manufacturer/family/variant must corroborate the model and each other. On powered aircraft, the propulsion campaign and a `SpecificInstalledConfiguration` or `MeasuredConfiguration` claim must identify the same physical airframe, operational configuration and propulsion configuration. A manufacturer recommendation or historical installation is not a physical installation claim. IDs are compared exactly and case-sensitively.

The gate evaluates these domains in this fixed order:

1. Identity: reference metadata, stable model ID, physical airframe/configuration, survey and cross-artifact identity.
2. Geometry: sourced wingspan, wing area and reference chord; survey-qualified tail geometry; resolved runtime aero elements and schema-v5+ surfaces; a sourced, unambiguous CG datum.
3. Mass properties: sourced model mass and CG, campaign configuration, measured/derived total mass and CG, and an evidence-qualified inertia tensor or accepted derivation path from the existing mass evaluator.
4. Aerodynamics: airfoil and coordinate provenance, qualified polar datasets, the evaluator's Reynolds/Mach envelope, and the caller's required alpha interval covered by **every** accepted dataset. Unresolved convergence cannot pass.
5. Propulsion when present: motor, ESC, battery, propeller, required performance dataset, provenance and physical installation identity from the existing propulsion evaluator.
6. Controls: resolved actuator-to-surface bindings, sourced physical travel for every binding, and validated model servo neutral/limits/rate. Radio channel mapping is not aircraft evidence.
7. Model integrity: strict loader validation, supported schema, finite rigid-body mass and physics fingerprint.
8. Flight-test data: separate, currently unassessed and incomplete.

Missing facts remain missing; an absent source is never silently replaced with a manufacturer recommendation, a synthetic fixture or an estimated number. Existing artifact evaluators decide their own qualification rules. RA1-A consumes those decisions and adds cross-artifact identity and alpha-range checks.

## Determinism and blockers

Domain and finding order is fixed by evaluation code, not hash-map iteration. Within aerodynamics, the existing evaluator sorts datasets deterministically; control travel findings follow validated model-binding order. Repeated evaluation of identical loaded inputs produces an equal report. No new serialization format is introduced. `ReadinessReason` is the decision API; strings in finding subjects identify affected datasets/bindings and do not control policy.

The gate is fail-closed about the evidence it can assess. It does not prove that an artifact was honestly classified as physical measurement; that remains a source-provenance review responsibility. The current reference artifact schemas also do not persist a universal numerical link from every runtime parameter/polar to a campaign measurement. RA1-A therefore checks prerequisite evidence and identity, **not** numerical agreement between runtime parameters and measured results. These are explicit review items for subsequent RA1 work, not implicit claims of validation.

Non-finite mass, CG and inertia are rejected by the strict loaders before the gate can observe them: JSON cannot carry NaN/Inf, and every loader validates finiteness on the way in. `NonFiniteModelData` is therefore defence in depth rather than a reachable fixture state; the load-time rejection is what the existing strict-loading tests assert, notably `nonfinite_reference_values_are_rejected_by_strict_json_loading` and `malformed_nonfinite_negative_mass_and_malformed_tensor_fail_closed`.

`SurveyClassification` has only `PhysicalReferenceMeasurement` and `SyntheticNonReference`, and nothing cross-validates that declaration against the cited sources. The `READY` path is therefore demonstrated by the explicitly test-only mock in `reference_readiness_ra1_a_closure.rs`, which *declares* itself measured in order to prove the gate is not structurally incapable of `READY`. That test certifies nothing about any aircraft, and the committed SIG Kadet LT-40 artifacts are deliberately left at their real provisional state.

## Deliberate non-goals and follow-on work

RA1-A does not calibrate coefficients, modify polars or flight behavior, perform system identification or optimization, align telemetry, compare trajectories, or change the renderer. It introduces no new flight model. A future measurement campaign must record reproducible physical observations and configuration changes, qualify flight-test/telemetry evidence separately, and then compare simulation outputs against those observations. Until that happens, no current synthetic fixture or provisional reference template should be reported as a ready real aircraft.
