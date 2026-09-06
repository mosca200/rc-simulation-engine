# G2F Chase Camera Robustness (presentation-only)

Status: implemented slice (renderer-only, `camera.rs` + this doc).
Base commit: `1575276` (G2D final). Branch:
`feature/g2f-chase-camera-robustness`.

## Scope

Allowed files only:

- `crates/renderer/src/camera.rs`
- this doc (`docs/architecture/renderer_g2f_chase_camera_robustness.md`)

Untouched: `gpu.rs`, `shader.wgsl`, `shadow.rs`, `scenery.rs`,
`terrain.rs`, `app/*`, `model/*`, `aircraft/*`, `sim-core/*`. No changes
to distance-behind, height, look-ahead, FOV, or the pilot camera.

## Problem

`ChaseCamera::eye_and_target()` derives the behind-aircraft heading from
`normalized_horizontal_forward(forward)`. When the horizontal projection
of forward is near zero (vertical climb, loop apex, ±90° pitch), the old
code fell back to the fixed global heading `[0, 0, -1]`, which can swing
the chase camera artificially even though the aircraft attitude itself
still carries a well-defined loop-plane direction.

## Fix (geometric fallback, same pose)

`chase_horizontal_forward(pose, forward)` keeps historic behavior
bit-for-bit whenever `|forward.xz| > 1e-4`, and otherwise derives the
heading from the same `RenderPose` local axes:

1. Project the local vertical axis (`up` = body `+Y` in render space)
   onto the horizontal plane, with a sign from the climb/dive sense
   (`forward.y`): belly side (`-up`) on climb, canopy side (`+up`) on
   dive. For a pure pitch entry this matches the approach heading, so
   the camera keeps the loop-plane direction through the pole instead
   of jumping to a global heading.
2. If that projection is degenerate (only possible for
   non-orthonormal/corrupt input, since the fallback axis is orthogonal
   to the near-vertical forward), try the body right axis (`+X`).
3. Only non-finite or doubly-degenerate input uses the global
   `[0, 0, -1]` heading (unreachable for valid rotation matrices).

Properties: presentation-only, no physics feedback, stateless camera
(no history, no smoothing, no lag, no app state), deterministic (pure
function of the pose), zero allocation, no new dependencies, no GPU
changes.

## Vertical behavior

- ±90° pitch: finite eye/target, finite view-projection, invertible
  matrix, heading keeps the pitch-entry direction.
- Near ±90° on the entry side: heading stays on the entry heading with
  step-to-step continuity (no global-heading swing).
- Past the pole: the nose genuinely points the other way; the camera
  follows the pose-derived flipped heading continuously (this flip is
  geometric, not a global jump — a stateless camera cannot distinguish
  "about to loop over" from "coming back" exactly at the pole).
- Yawed vertical (loop plane off the default axis): follows the
  pose-derived loop-plane direction, not the global heading.
- Roll while vertical: finite heading, eye, target, view-projection,
  and inverse for all sampled roll angles.

## Tests (`crates/renderer/src/camera.rs`)

Kept green: all pre-existing camera tests
(`identity_camera_is_behind_and_targets_ahead`,
`inclined_attitude_produces_a_finite_non_degenerate_camera`,
`vertical_attitude_uses_a_stable_finite_heading_fallback`,
`chase_config_is_presentation_only_and_tunable`, resize/eye/inverse/
pilot/fog/sun suites).

Added:

- `level_flight_heading_is_unchanged`
- `normal_pitch_matches_legacy_projection_within_tolerance`
- `plus_ninety_pitch_is_finite_and_keeps_entry_heading`
- `minus_ninety_pitch_is_finite_and_keeps_entry_heading`
- `near_plus_ninety_entry_side_stays_continuous`
- `near_plus_ninety_far_side_follows_flipped_nose`
- `near_minus_ninety_entry_side_stays_continuous`
- `near_minus_ninety_far_side_follows_flipped_nose`
- `yawed_vertical_uses_geometric_fallback_not_global_heading`
- `roll_while_vertical_remains_finite`
- `vertical_camera_eye_differs_from_target_and_matrix_inverts`
- `same_pose_gives_same_camera_result`
