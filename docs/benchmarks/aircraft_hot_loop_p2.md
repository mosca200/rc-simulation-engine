# Aircraft hot-loop allocation audit P2

## Scope and call graph

The measured path is exactly `AircraftSimulation::step(&PilotInput)`:

```text
AircraftSimulation::step
  -> step_with_stage_observer (monomorphized no-op observer)
    -> advance_controls
      -> rates/expo response
      -> conventional mixer
      -> three servo state updates
    -> update_effective_aero_elements
      -> overwrite bound entries in the pre-sized effective-element Vec
    -> Rk4Integrator::step (monomorphized FnMut stage evaluator)
      -> stage 1
      -> stage 2 from k1 state
      -> stage 3 from k2 state
      -> stage 4 from k3 state
      -> weighted state update and quaternion normalization
      each stage:
        -> evaluate_aerodynamic_wrench
          -> ordered slice iteration over all effective/model aero elements
          -> polar lookup and AeroElement evaluation
          -> by-value BodyWrench accumulation
        -> optional evaluate_electric_propulsion
          -> coefficient lookup
          -> electrical/quasi-static shaft solution
          -> thrust, reaction torque, and BodyWrench accumulation
        -> evaluate_derivative
    -> commit rigid-body state and increment step index
    -> construct AircraftSnapshot by value
```

There is no render-frame delta, interpolation, GPU, window, hardware input, replay hash, telemetry
serialization, file I/O, logging, or formatting in this path.

## Heap-backed structures and initialization

The validated `AircraftModel` owns heap-backed vectors and strings for polars, aerodynamic elements,
control bindings, identifiers, and propulsion coefficient samples. JSON parsing, validation,
reference resolution, strings, and those vector allocations occur during model loading.

`AircraftSimulation::new` allocates `effective_aero_elements` once by collecting the model elements.
Its length is fixed for the simulation lifetime. The P2 CLI also preallocates its timing
`Vec<Duration>` before measurement; that vector belongs to the benchmark harness, not flight core.

## Per-step findings

- Control and servo state are fixed-size values updated in place.
- Bound effective elements overwrite existing vector indices; the vector is never pushed, resized,
  cloned, or replaced during a step.
- Model vectors and strings are accessed only through immutable slices/references.
- RK4 stage states, derivatives, wrenches, propulsion output, and the final snapshot are fixed-size
  by-value values.
- Aerodynamic and propulsion loops traverse existing slices and perform numerical operations only.
- The stage evaluator and observer are generic, monomorphized closures without `Box`, trait objects,
  or dynamic dispatch.
- No `String`, `format!`, `Vec` growth, `HashMap`, `BTreeMap`, serde, JSON, file I/O, or logging API is
  called on the successful per-step path.
- Assertions can format only if an invariant fails; they do not allocate on the successful path.

## Evidence level: A. VERIFIED

The existing test-only `allocation-counter` development dependency counts allocations made by the
current thread. P2 retains the established complete-step allocation test and adds a model-specific
test that warms up Acro Electric 01, then measures 100 consecutive complete steps. The asserted
result is `count_total == 0`.

This is appropriate runtime evidence because `AircraftSimulation::step()` is single-threaded and all
work in scope executes on the measured thread. The allocator instrumentation remains test-only. P2
does not add a global allocator, write unsafe code, weaken `unsafe_code = "deny"`, or introduce a new
allocation dependency.

## Known limitations

- Allocation counting proves the exercised successful paths and current model/configuration; it is
  not a formal proof over every possible future implementation.
- The counter observes only the current thread. The flight step is currently single-threaded, so
  this matches the audited execution model.
- Timing samples include `Instant` call overhead, while allocation tests measure the simulation step
  independently of benchmark sample storage.
- Operating-system scheduling and interrupts affect timing outliers but not physics output or the
  allocation count.
