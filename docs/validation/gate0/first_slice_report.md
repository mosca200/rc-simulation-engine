# First Vertical Slice acceptance report

- Model: `acro-electric-01`
- Physics fingerprint: `dedc79818699d5342ad7c2d770a1957b29d541488635615b8c822135ab08b8ed`
- Overall status: **PARTIAL**
- Technical Gate: **PASS**
- Manual Gate: **PARTIAL**
- Real-world Gate: **PARTIAL**

## Criteria

| ID | Gate | Status | Detail |
|---|---|---|---|
| `workspace_structure` | `Technical` | **PASS** | required workspace crates and canonical model are present |
| `rust_stable_build` | `Technical` | **PASS** | workspace declares Rust 1.98 MSRV and this acceptance binary is executing |
| `f64_flight_core` | `Technical` | **PASS** | canonical timestep, state vectors, quaternion, model coefficients, and dynamics use f64 |
| `fixed_step_500hz` | `Technical` | **PASS** | default physics configuration is exactly 500 Hz / 0.002 s |
| `rk4_integrator` | `Technical` | **PASS** | AircraftSimulation uses the four-stage state-dependent Rk4Integrator path covered by regression tests |
| `local_aerodynamic_elements` | `Technical` | **PASS** | validated model contains ordered local aerodynamic elements and resolved polars |
| `electric_propulsion` | `Technical` | **PASS** | Acro Electric model includes validated battery, ESC/motor, propeller, and coefficient data |
| `control_servo_pipeline` | `Technical` | **PASS** | rates/expo, conventional mixer, servo dynamics, and resolved surface bindings are configured |
| `versioned_model_format` | `Technical` | **PASS** | canonical model loaded through strict schema-v2 validation |
| `deterministic_aircraft_replay` | `Technical` | **PASS** | verified all 2000 canonical replay steps |
| `telemetry_pipeline` | `Technical` | **PASS** | 2,000 contiguous finite replay-derived telemetry frames validated in memory |
| `model_versioning` | `Technical` | **PASS** | schema, model ID, physics fingerprint, and canonical replay identity agree |
| `glb_presentation` | `Technical` | **PASS** | declared GLB exists and parsed to a finite, non-empty indexed triangle mesh |
| `minimal_outdoor_scene` | `Manual` | **PARTIAL** | ground-plane and sky-clear implementations exist, but no visual observation has been performed |
| `sim_render_separation` | `Technical` | **PASS** | renderer dependency boundary excludes simulation ownership and physics remains fixed-step |
| `sim_render_snapshot_interpolation` | `Technical` | **PASS** | two-snapshot f64 interpolation, alpha clamp, shortest-path normalized SLERP, and origin-before-f32 verified |
| `physics_frame_rate_independence` | `Technical` | **PASS** | 60 Hz-like, 144 Hz-like, and variable frame patterns produced identical physics |
| `input_pipeline` | `Technical` | **PASS** | normalization, deadzone, inversion, throttle endpoints, mapping, keyboard fallback, and fixed-step sampling verified without hardware |
| `real_controller_hardware` | `Manual` | **NOT_TESTED** | no physical controller was connected or exercised; devices: 0 is not acceptance evidence |
| `radiomaster_tx16s` | `Manual` | **NOT_TESTED** | Radiomaster TX16S hardware has not been tested |
| `basic_user_flight_session` | `Manual` | **NOT_TESTED** | no real interactive user flight session has been observed and recorded |
| `live_input_replay_recording` | `Technical` | **PASS** | applied PilotInput equals recorded input and pre-step N maps to post-step N+1 hash |
| `physics_performance` | `Technical` | **PASS** | short release acceptance measurement classified PASS |
| `hot_loop_allocations` | `Technical` | **PASS** | P2 evidence level VERIFIED: allocation-counter measured zero allocations across 100 Acro Electric steps after initialization |
| `acro_electric_characterization` | `Technical` | **PASS** | all S10 manoeuvres executed with valid replay and telemetry in memory |
| `pilot_review_protocol` | `RealWorld` | **PASS** | versioned structured protocol exists and explicitly records that it has not been executed |
| `real_pilot_review` | `RealWorld` | **NOT_TESTED** | the structured pilot-review protocol exists but no real pilot session has occurred |
| `real_world_calibration` | `RealWorld` | **NOT_TESTED** | no measured aircraft reference, propulsion bench data, flight telemetry, or calibrated inertia is available |
| `graphical_viewer_verification` | `Manual` | **NOT_TESTED** | the renderer has not been visually observed in a persisted, reviewable verification |
| `headless_execution` | `Technical` | **PASS** | acceptance path uses direct Rust APIs and initializes no GPU, window, renderer object, hardware backend, or child process |
| `regression_dataset` | `Technical` | **PASS** | canonical dataset exists, is versioned, contiguous, identity-bound, and all hashes pass |

## Technical PASS

- `workspace_structure` — required workspace crates and canonical model are present
- `rust_stable_build` — workspace declares Rust 1.98 MSRV and this acceptance binary is executing
- `f64_flight_core` — canonical timestep, state vectors, quaternion, model coefficients, and dynamics use f64
- `fixed_step_500hz` — default physics configuration is exactly 500 Hz / 0.002 s
- `rk4_integrator` — AircraftSimulation uses the four-stage state-dependent Rk4Integrator path covered by regression tests
- `local_aerodynamic_elements` — validated model contains ordered local aerodynamic elements and resolved polars
- `electric_propulsion` — Acro Electric model includes validated battery, ESC/motor, propeller, and coefficient data
- `control_servo_pipeline` — rates/expo, conventional mixer, servo dynamics, and resolved surface bindings are configured
- `versioned_model_format` — canonical model loaded through strict schema-v2 validation
- `deterministic_aircraft_replay` — verified all 2000 canonical replay steps
- `telemetry_pipeline` — 2,000 contiguous finite replay-derived telemetry frames validated in memory
- `model_versioning` — schema, model ID, physics fingerprint, and canonical replay identity agree
- `glb_presentation` — declared GLB exists and parsed to a finite, non-empty indexed triangle mesh
- `sim_render_separation` — renderer dependency boundary excludes simulation ownership and physics remains fixed-step
- `sim_render_snapshot_interpolation` — two-snapshot f64 interpolation, alpha clamp, shortest-path normalized SLERP, and origin-before-f32 verified
- `physics_frame_rate_independence` — 60 Hz-like, 144 Hz-like, and variable frame patterns produced identical physics
- `input_pipeline` — normalization, deadzone, inversion, throttle endpoints, mapping, keyboard fallback, and fixed-step sampling verified without hardware
- `live_input_replay_recording` — applied PilotInput equals recorded input and pre-step N maps to post-step N+1 hash
- `physics_performance` — short release acceptance measurement classified PASS
- `hot_loop_allocations` — P2 evidence level VERIFIED: allocation-counter measured zero allocations across 100 Acro Electric steps after initialization
- `acro_electric_characterization` — all S10 manoeuvres executed with valid replay and telemetry in memory
- `headless_execution` — acceptance path uses direct Rust APIs and initializes no GPU, window, renderer object, hardware backend, or child process
- `regression_dataset` — canonical dataset exists, is versioned, contiguous, identity-bound, and all hashes pass

## Manual NOT_TESTED / PARTIAL

- `minimal_outdoor_scene`: **PARTIAL** — ground-plane and sky-clear implementations exist, but no visual observation has been performed
- `real_controller_hardware`: **NOT_TESTED** — no physical controller was connected or exercised; devices: 0 is not acceptance evidence
- `radiomaster_tx16s`: **NOT_TESTED** — Radiomaster TX16S hardware has not been tested
- `basic_user_flight_session`: **NOT_TESTED** — no real interactive user flight session has been observed and recorded
- `graphical_viewer_verification`: **NOT_TESTED** — the renderer has not been visually observed in a persisted, reviewable verification

## Open gaps

- minimal_outdoor_scene: ground-plane and sky-clear implementations exist, but no visual observation has been performed
- real_controller_hardware: no physical controller was connected or exercised; devices: 0 is not acceptance evidence
- radiomaster_tx16s: Radiomaster TX16S hardware has not been tested
- basic_user_flight_session: no real interactive user flight session has been observed and recorded
- real_pilot_review: the structured pilot-review protocol exists but no real pilot session has occurred
- real_world_calibration: no measured aircraft reference, propulsion bench data, flight telemetry, or calibrated inertia is available
- graphical_viewer_verification: the renderer has not been visually observed in a persisted, reviewable verification
