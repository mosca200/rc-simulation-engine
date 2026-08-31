# S10 — Acro Electric 01 validation and tuning report

## Outcome

S10 produced a deterministic baseline characterization and pilot-review protocol. It did **not**
perform model tuning because the repository contains no authoritative target capable of justifying
a parameter change. This is an evidence-preserving outcome, not a claim that the placeholder model
is physically calibrated.

| Item | Result |
| --- | --- |
| S9 technical gate | PASS on first cycle |
| S10 manoeuvre suite | PASS, version 1 |
| Model parameter changes | none |
| Baseline fingerprint | `dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed` |
| Final fingerprint | `dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed` |
| Canonical replay regenerated | no; model physics did not change |
| Pilot review | prepared, NOT TESTED |
| Real-world validation | NOT TESTED |

## Evidence classification

- **MEASURED:** values emitted by deterministic simulations or read directly from the repository.
- **DERIVED:** arithmetic derived from measured simulation state, with formula stated.
- **ASSUMED:** explicit test conditions such as standard density and initial state.
- **NOT YET VALIDATED:** no authoritative real-world target or comparison exists.

## References available

1. The validated repository model and its schema/fingerprint.
2. The model README, which explicitly classifies values as physically plausible placeholders and
   not flight-calibrated data.
3. S8A input replay and per-step snapshot hashes.
4. S9 strict telemetry captures and deterministic summaries.
5. Existing unit, numerical, allocation and regression tests.

No external aircraft is identified by the model, so unrelated public specifications were not used
as substitute targets.

## References missing

- measured mass distribution and inertia;
- measured geometry and control deflections;
- airfoil/polar source, Reynolds range, wind-tunnel data;
- motor, battery and propeller datasheets or bench curves;
- measured flight envelope and performance targets;
- real flight telemetry;
- completed structured pilot comparison.

Consequently every numerical acceptance target is `TARGET = UNDEFINED`, and every manoeuvre below
is classified as characterization rather than physical validation.

## Methodology

`rcsim-app validate acro-electric-01 --output-dir PATH` runs suite version 1 at 500 Hz. Every
manoeuvre starts independently from `[0,0,-100] m` NED, `[18,0,0] m/s`, identity attitude and zero
body rate, with density `1.225 kg/m³` and zero wind. For each step it:

1. selects a deterministic `PilotInput`;
2. records and advances through S8A;
3. verifies the resulting replay hashes from a reconstructed simulation;
4. records the same post-step snapshot through S9;
5. writes replay and JSONL telemetry outside the hot loop.

Artifacts are work products under `target/s10_validation/`; `summary.md` contains full final states
and control ranges.

## Manoeuvre suite v1

| Manoeuvre | Duration | Input schedule |
| --- | ---: | --- |
| straight_neutral | 4 s | neutral axes, throttle 0.55 |
| throttle_response | 4 s | throttle 0.20 for 1 s, 0.85 for 2 s, 0.55 for 1 s |
| pitch_step | 4 s | pitch +0.35 from 0.5–1.5 s, then neutral |
| roll_step | 4 s | roll +0.40 from 0.5–1.5 s, then neutral |
| yaw_step | 4 s | yaw +0.35 from 0.5–1.5 s, then neutral |
| control_reversal_recovery | 5 s | roll +0.45 for 1 s, then -0.45 for 1 s, then neutral |
| power_off_glide | 6 s | neutral axes, throttle 0 |
| high_angle_entry | 4 s | pitch +0.75 from 0.5–2.5 s, throttle 0.35 |

The high-angle run probes only behaviour of the existing clamped quasi-steady polar. It does not
validate stall, dynamic stall, spin, hysteresis or recovery physics.

## Metrics

All rows are **MEASURED**, except specific kinetic-energy change, which is **DERIVED** as
`0.5 × (final_speed² - initial_speed²)` in `J/kg`. Altitude change uses
`final_local_altitude - 100 m`.

| Manoeuvre | Speed min/max/mean m/s | Altitude Δ m | Peak rate rad/s @ time | Final speed m/s | Specific KE Δ J/kg |
| --- | --- | ---: | --- | ---: | ---: |
| straight_neutral | 18.002887 / 29.780755 / 23.685292 | -37.740570 | 0.300208 @ 0.188 s | 29.780755 | 281.446673 |
| throttle_response | 17.358726 / 32.096788 / 24.618295 | -41.787115 | 0.304553 @ 0.202 s | 32.096788 | 353.101912 |
| pitch_step | 18.002887 / 25.915513 / 21.452521 | -17.186487 | 0.322944 @ 1.614 s | 25.915513 | 173.806917 |
| roll_step | 18.002887 / 32.830849 / 24.570690 | -54.827060 | 1.776913 @ 1.500 s | 32.830849 | 376.932335 |
| yaw_step | 18.002887 / 29.748321 / 23.650163 | -37.615038 | 0.402426 @ 1.622 s | 29.748321 | 280.481295 |
| control_reversal_recovery | 18.002887 / 33.733598 / 25.926015 | -72.264030 | 2.429929 @ 2.500 s | 33.733598 | 406.977827 |
| power_off_glide | 17.282362 / 30.084960 / 22.666580 | -79.498954 | 0.304820 @ 0.202 s | 30.084960 | 290.552424 |
| high_angle_entry | 8.828114 / 17.998444 / 13.647527 | +10.007914 | 0.724124 @ 0.646 s | 9.690543 | -115.046690 |

These figures are regression baselines only. For example, the power-off speed increase and large
altitude loss are observed results, not proof of realistic glide performance.

## Changes applied and before/after

No model parameter was modified.

| Parameter | Before | After | Reference | Decision |
| --- | --- | --- | --- | --- |
| Entire model physics fingerprint | `dedc798…b08b8ed` | `dedc798…b08b8ed` | Repository deterministic identity | unchanged |
| Mass/inertia | existing placeholder | same | no measured aircraft data | rejected tuning |
| Polars/geometry | existing placeholder | same | no aerodynamic reference | rejected tuning |
| Controls/servos | existing placeholder | same | no measured deflections or pilot review | rejected tuning |
| Battery/motor/propeller | existing placeholder | same | no datasheet/bench reference | rejected tuning |

There is therefore no hidden tuning, no side-effect tradeoff and no need to regenerate the canonical
replay dataset. Its existing fingerprint remains correct.

## Open gaps

### Physics

- no dynamic stall, hysteresis, spin model, Reynolds dependency or propwash;
- no terrain, ground, landing gear or collision model;
- no canonical post-step total force/moment, alpha/beta, RPM, thrust, torque or power diagnostics.

### Model data

- all physical parameters remain uncalibrated placeholders;
- no identified real aircraft or authoritative target envelope.

### Telemetry

- required state/control telemetry exists;
- the physics diagnostics listed above remain unavailable without a future canonical diagnostic
  boundary.

### Render and presentation

- no visual validation was performed;
- the referenced GLB is metadata only and no asset is supplied.

### Input hardware

- gilrs backend and synthetic mapping are tested;
- no physical joystick, TX16S, latency or calibration session was tested.

### Real-world validation

- structured pilot protocol exists but no pilot session has occurred;
- no real flight telemetry or matched-aircraft comparison exists.

## Pilot review status

`docs/validation/acro_electric_01_pilot_review.md` is ready for a future structured session. No
fields or scores have been fabricated. Status: **NOT TESTED**.

## Vertical-slice gate assessment

| Criterion | Status | Evidence / limitation |
| --- | --- | --- |
| 500 Hz fixed-step architecture | PASS | fixed `dt=0.002 s`, integer step accounting |
| Headless execution | PASS | foundation, aircraft, replay, input, telemetry, validation CLI |
| No known allocations in hot loop | PASS | allocation tests and unchanged aircraft step |
| Numerical regression | PASS | unit/integration tests and per-step replay hashes |
| Input-based deterministic replay | PASS | S8A and validation manoeuvre replay verification |
| Simulation/render separation | PASS | headless crate boundaries; renderer is presentation-only |
| Telemetry availability | PASS | strict S9 JSONL and analyzer |
| Model versioning | PASS | schema v1 plus physics fingerprint |
| Input pipeline | PARTIAL | CPU/keyboard/backend verified; real hardware NOT TESTED |
| Acro Electric characterization | PASS | eight deterministic manoeuvres and metrics |
| Real pilot review | NOT TESTED | protocol prepared only |
| Flight manoeuvre coverage | PARTIAL | core headless suite covered; landing/ground, real stall, loop and pilot exercises remain |

Overall vertical-slice readiness is **PARTIAL**: the deterministic engineering pipeline is ready,
but physical fidelity and pilot acceptance are not yet validated.
