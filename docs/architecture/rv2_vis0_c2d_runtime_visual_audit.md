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
option, removes stale audit files, validates the strict 1.0.0 structure, and
checks renderer `v2`, presentation frame, and framebuffer extent against the
trusted runtime receipt. Missing or invalid audit data fails the runner closed,
while `visual_pass` remains `null` and the existing evidence contract is
unchanged.

The existing reference manifest is the FINAL diagnostic. The adjacent C2D
manifests vary only one existing `terrain_debug` or `vegetation_debug` selector;
camera, aircraft, exposure, resolution, warmup, and capture frame remain equal
to the reference scene. C2D adds no shader/debug modes and intentionally changes
no production FINAL pixels.
