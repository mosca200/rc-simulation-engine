# RV2-VIS0-C2D Runtime Visual Audit

`RuntimeVisualAudit 1.0.0` is a separate, renderer-owned fact artifact for one
presented V2 frame. It does not change `GoldenSceneManifest 1.1.0`,
`RuntimeCaptureReceipt 1.0.0`, or `VisualCaptureEvidence 1.0.0`, and it does not
express a visual-quality verdict.

The app requests the artifact with `--visual-audit-out PATH`. Without that
option the historical render path is unchanged. With it, the app supplies the
zero-based presentation frame index to the V2 profiler, waits until that frame
has been presented (and captured when capture is requested), asks the renderer
for a plain Rust DTO, checks frame and framebuffer identity, then writes JSON
through a sibling temporary file followed by rename. Startup removes the target
and temporary sibling so stale output cannot represent the current run.

The producer reuses renderer state rather than probing the operating system or
creating parallel telemetry:

- `DeviceCapabilities::adapter_info` supplies device and driver identity.
- The post-construction `V2EnvironmentMode` and physical resource selection
  supply atmosphere, IBL, and aerial-perspective facts.
- The HDR target, exposure state, temporal resources, terrain/debug state,
  shadow constants, and vegetation state machine supply their respective facts.
- `ProfileSnapshot` supplies bounded CPU and optional GPU timings for the six
  authoritative `PassId` values.

No `wgpu` type crosses the renderer boundary. Values unavailable from a runtime
source of truth are JSON `null` with a reason. In particular, the WGSL-internal
PCF tap count and the multi-frequency terrain material's nonexistent single
scale are not duplicated in Rust.

GPU timestamps remain asynchronous. The profiler records the presentation
frame associated with each readback slot. The audit reports a current-frame
sample, an explicitly aged previous-frame sample, unsupported timestamps, or a
not-yet-ready result; it never attributes an older sample to the capture frame.

Runner 1.3.0 derives `<scene_dir>/runtime_visual_audit.json`, passes the CLI
option for renderer `v2` only, removes stale audit files, validates the strict
1.0.0 structure, and checks renderer `v2`, presentation frame, and framebuffer
extent against the trusted runtime receipt. Missing or invalid audit data fails
a `v2` runner closed, while `visual_pass` remains `null` and the existing
evidence contract is unchanged.

Audit applicability is renderer-scoped. `rcsim-app` accepts `--visual-audit-out`
only together with `--renderer v2` and rejects it otherwise with
`RenderAppError::VisualAuditRequiresV2`; that guard is correct and stays. The
runner therefore plans the audit from the manifest it already holds instead of
emitting the flag for every executable capture, which would hand a `v1` scene a
command the runtime refuses:

- `capture_plan.audit_required` is true exactly when the scene is executable and
  `renderer.version == "v2"`. `capture_plan.audit_applicability` names the state
  (`required` / `not_applicable`), `capture_plan.audit_applicability_reason`
  states why, and `execution_policy.visual_audit_required` repeats it next to
  `capture_executable`.
- For `v1` the derived argument list is `--capture-receipt-out` and
  `--exit-after-frame` only, the VIS0-C2B capture/receipt/PNG/evidence path is
  untouched, and `runner_success` depends on those historical conditions alone
  (`execution_policy.success_requires` drops the audit entry).
- "Not applicable" is never encoded as a failed audit:
  `runtime_visual_audit_validation` reports `required=false`, `attempted=false`,
  `valid=null` with a `not_applicable_reason`, the file is never read, and
  `run.json` keeps `runtime_visual_audit`, `artifacts.runtime_visual_audit`,
  `execution.runtime_visual_audit_path` and `artifact_paths.runtime_visual_audit`
  null. No audit is fabricated.
- Success is `process/capture/receipt/evidence valid AND (audit not required OR
  audit valid)`, so a required audit that is missing, malformed, or bound to a
  different frame or framebuffer extent still fails the run closed.
- `GoldenSceneManifest 1.1.0` is not narrowed: `renderer.version` keeps
  accepting `"v1" | "v2"` and gains no field, and the audit path stays
  runner-owned. The plan document gained the applicability keys without a
  `plan_version` bump because 1.3.0 is this slice's own, not yet integrated,
  runner version.

The stale-artifact sweep deletes `runtime_visual_audit.json` and its `.tmp`
sibling for every renderer version, including one that requires no audit, so a
leftover audit from an earlier `v2` run cannot survive in a `v1` scene
directory; `v1` metadata could not claim it either way.

The strict reader also rejects an internally contradictory 1.0.0 environment
block: `environment_mode == "physical"` requires `physical_atmosphere_active`
and `physical_ibl_active` to be true, mirroring `environment_from_runtime`,
which derives the mode and both flags from one `V2EnvironmentMode`.
`aerial_perspective_active` is deliberately not implied by physical mode, being
a separately reported runtime path. This rejects contradictory documents rather
than changing the contract, so no version is bumped.

The existing reference manifest is the FINAL diagnostic. The adjacent C2D
manifests vary only one existing `terrain_debug` or `vegetation_debug` selector;
camera, aircraft, exposure, resolution, warmup, and capture frame remain equal
to the reference scene. C2D adds no shader/debug modes and intentionally changes
no production FINAL pixels.
