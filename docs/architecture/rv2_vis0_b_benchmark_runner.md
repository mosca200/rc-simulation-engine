# RV2-VIS0-B — Golden Visual Benchmark Runner

> **Stato attuale (VIS0-C2B).** Questo documento descrive il runner nato con
> VIS0-B ed è stato aggiornato dove le sue affermazioni erano diventate false.
> Dal VIS0-C2B il runner **esegue davvero una capture**: emette il gruppo di
> flag `--capture-frame/--capture-out/--capture-format`, aggiunge i flag derivati
> `--capture-receipt-out`/`--exit-after-frame`, verifica in modo indipendente il
> `RuntimeCaptureReceipt` e il PNG, e pubblica un `VisualCaptureEvidence`
> validato. `RUNNER_VERSION`/`PLAN_VERSION` sono `1.2.0`. Il documento
> normativo per quella integrazione è
> [`rv2_vis0_c2b_capture_evidence_integration.md`](rv2_vis0_c2b_capture_evidence_integration.md);
> il contratto runtime della capture resta
> [`rv2_vis0_c2a_frame_capture.md`](rv2_vis0_c2a_frame_capture.md).

## Purpose

VIS0-B turns the VIS0-A manifest contract into tooling that actually runs.

VIS0-A ([`rv2_vis0_visual_benchmark.md`](rv2_vis0_visual_benchmark.md)) defines *what*
a golden scene is and validates that a manifest is well formed. VIS0-B consumes a
validated manifest and produces:

1. a **deterministic execution plan**,
2. the **exact `rcsim-app` command** that plan implies,
3. optional **real process execution** with captured stdout/stderr/exit code,
4. **run provenance** (`run.json`) sufficient to reproduce or audit the run,
5. since VIS0-C2B, a **verified capture** plus a standalone
   `capture_evidence.json`.

**VIS0 verifies execution and capture reproducibility, NOT visual image quality.**

It never decides a visual PASS/FAIL and never reads a pixel value.
`visual_pass` is written as `null` in every artifact this runner emits. A
byte-verified capture states that an image exists and matches its receipt; it
says nothing about whether the image is good.

## Scope boundary

VIS0-B is deliberately independent of the renderer runtime. It touches only:

- `tools/visual_benchmark/**`
- `docs/validation/visual_benchmark/**`
- `docs/architecture/**` (VIS0 documentation only)

No `crates/**`, `Cargo.toml`, `Cargo.lock`, `.github/**`, `main` or
`integration/render-v2` change. Where the runtime cannot support a manifest
field, this runner **records the gap and refuses to execute** instead of
modifying the runtime or faking the capability.

The VIS0-A contract files stay authoritative and byte-identical:
`golden_scene_manifest.schema.json`, `validate_manifest.py`,
`test_validate_manifest.py`, `vis0_reference_scene.json`. The runner *imports*
`validate_manifest.ManifestValidator` rather than reimplementing any rule.

## Usage

All commands run from the repository root. Python standard library only — no
pip dependency.

### Dry run (the default; starts no application process)

```
python tools/visual_benchmark/run_benchmark.py \
  --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
  --dry-run
```

Omitting both `--dry-run` and `--execute` is also a dry run. Execution is never
the default: it requires an explicit `--execute`.

### Execution

```
python tools/visual_benchmark/run_benchmark.py \
  --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
  --app target/release/rcsim-app \
  --execute
```

Build the app first:

```
cargo build --release -p rcsim-app
```

> `rcsim-app render` still opens a visible winit window; VIS0-C1 did not add a
> headless renderer. Auto-exit is supported and the runner derives
> `--exit-after-frame` from the captured presentation frame, so an unattended run
> has a deterministic frame-bounded lifecycle. Since VIS0-C2A the same frame is
> really written to a lossless PNG plus a `RuntimeCaptureReceipt`; see
> [Capture capability status](#capture-capability-status).

### Persisting the plan

```
python tools/visual_benchmark/run_benchmark.py \
  --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
  --plan-json tmp/plan.json
```

## CLI reference

| Flag | Default | Meaning |
| --- | --- | --- |
| `--manifest PATH` | *(required)* | GoldenSceneManifest JSON to load and validate |
| `--dry-run` | on by default | Build and print the plan; start no application process, write no run artifacts |
| `--execute` | off | Opt in to really launching the app process |
| `--app PATH_OR_COMMAND` | none | `rcsim-app` executable. Required with `--execute` |
| `--output-dir PATH` | `tmp/visual_benchmark_runs` | Root for run artifacts (gitignored) |
| `--timeout-seconds N` | `120` | Kill the app process after N seconds |
| `--run-index N` | `0` | Run index recorded in the plan and `run.json` |
| `--plan-json PATH` | none | Also write the execution plan as JSON |
| `--require-clean-git` | off | Fail with exit 3 unless `git status --porcelain` is empty |

`--dry-run` wins if both `--dry-run` and `--execute` are given; a note is printed
to stderr and no application process starts.

There is no interactive prompt of any kind. Every input arrives as a flag.

### Exit codes

| Code | Meaning |
| --- | --- |
| `0` | Plan built (dry run), or a **fully verified** end-to-end capture |
| `1` | Manifest failed GoldenSceneManifest validation, **or** is formally valid but not runtime-executable under the C2B policy; **no process was started** |
| `2` | Usage/input error: missing file, unreadable JSON, bad flag value |
| `3` | Git policy violation (`--require-clean-git` on a dirty work tree) |
| `4` | Execution failure: app not found, non-zero exit, timeout, untrusted receipt, unverified PNG, or evidence that failed its own validator |
| `5` | Interrupted (Ctrl-C) |

Since VIS0-C2B **`process exit 0` is no longer sufficient for exit `0`**. A run
returns `0` only when all four hold: process exit `0`, trusted
`RuntimeCaptureReceipt`, independently verified PNG, and a
`VisualCaptureEvidence` accepted by `CaptureEvidenceValidator`. Anything else is
exit `4`, even when `rcsim-app` itself succeeded.

Predictable user errors print one `error: ...` line. They never print a
Python traceback.

## Manifest → CLI mapping

Only flags that exist in `crates/app/src/render_app.rs` are emitted. The runner
refuses to build a command containing anything else, and a unit test re-reads the
runtime source to prove every emittable flag is real.

### Mapped to real CLI flags

| Manifest field | Runtime flag | Notes |
| --- | --- | --- |
| `renderer.version` | `--renderer` | `v1` \| `v2` |
| `renderer.terrain_debug` | `--terrain-debug` | optional; omitted when absent |
| `renderer.vegetation_debug` | `--vegetation-debug` | optional; omitted when absent |
| `scenery.preset` | `--scenery` | `none` \| `flying-field` |
| `camera.mode` | `--camera` | `pilot` \| `chase` |
| `camera.vertical_fov_deg` | `--camera-fov` | |
| `camera.pilot_position_render_m` | `--pilot-position` | `[x, y, z]` → `x,y,z` |
| `camera.chase_distance_behind_m` | `--chase-distance-m` | chase mode only |
| `camera.chase_height_above_m` | `--chase-height-m` | chase mode only |
| `exposure_ev` | `--exposure-ev` | |
| `aircraft.model` | `--model` | |
| `aircraft.throttle` | `--throttle` | optional; omitted when absent |
| `aircraft.altitude_m` | `--altitude-m` | required when airborne, forbidden on a ground start; range `(0, 10000]` |
| `aircraft.airspeed_mps` | `--airspeed-mps` | required when airborne, forbidden on a ground start; range `(0, 200]` |
| `aircraft.start_on_ground` | `--start-on-ground` | presence-only flag; emitted only when `true` |
| `resolution.width` | `--render-width` | VIS0-C1 runtime control; enforced fail-closed by the runtime |
| `resolution.height` | `--render-height` | VIS0-C1 runtime control; enforced fail-closed by the runtime |
| `capture.filename` | `--capture-out` | VIS0-C2B; resolved against `<output-dir>/<scene-id>/` and passed as an **absolute** path |
| `capture.format` | `--capture-format` | VIS0-C2B; only `png` is runtime-executable |
| `capture.frame` | `--capture-frame` | VIS0-C2B; zero-based **presentation** frame the runtime reads back |

`--camera <mode>` is always emitted explicitly. This matters: without it the
runtime falls back to a *different* default camera (`Pilot` at `[0, 0.3, 0.85]`,
FOV 70) than the one `--camera pilot` selects (`[0, 1.8, 20]`, FOV 55). Emitting
the mode makes the camera fully determined by the manifest.

`aircraft.altitude_m` and `aircraft.airspeed_mps` were added by VIS0-C1B. Until
then the runner could not emit `--altitude-m` / `--airspeed-mps`, so part of the
airborne initial state still came from the runtime defaults
(`DEFAULT_ALTITUDE_M = 30.0`, `DEFAULT_AIRSPEED_MPS = 18.0`) and the manifest did
not fully determine the scene. The bounds and the ground-start asymmetry are
verified against `RenderOptions::parse_with_defaults` and
`RenderApplication::new` in `crates/app/src/render_app.rs`; see
`docs/architecture/rv2_vis0_visual_benchmark.md` § "Stato iniziale aircraft
completamente esplicito (VIS0-C1B)".

### Runner-derived flags

Two emitted flags do **not** come from a manifest field, and the plan says so
explicitly (`capture_plan.derived_arguments`, `provenance: "runner-derived"`):

| Flag | Value | Derived from |
| --- | --- | --- |
| `--capture-receipt-out` | `<scene-dir>/runtime_capture_receipt.json` | a tooling decision: the runner picks the receipt location so it can parse, trust and cross-check it. No GoldenSceneManifest key names a receipt path. |
| `--exit-after-frame` | equal to `capture.frame` | process lifecycle control. `rcsim-app` rejects `--exit-after-frame < --capture-frame` (`ExitBeforeCaptureFrame`), and the equal-frame case captures, presents, writes the PNG and the receipt, then exits. |

Manifest-driven flags are emitted first, derived flags last, so the argv itself
shows the provenance split. Attributing a derived flag to a manifest field that
does not exist would falsify the plan's provenance.

### Unsupported and derived execution fields

Recorded in `run.json` with an explicit status, `runtime_flag` and reason. Never
silently dropped, never faked.

| Manifest field | Status | Evidence |
| --- | --- | --- |
| `warmup` | **derived** | There is no `--warmup` flag and none is invented. Warmup is realised by the presentation-frame capture relation `warmup == capture.frame`: `--capture-frame N` presents frames `0..N-1` first, which is exactly `N` warmup presentations. It is **not** supported for arbitrary `warmup`/`capture.frame` combinations, which is why the C2B executable path requires them to be equal. |
| `capture.quality` | **unsupported** | Only allowed by the VIS0-A contract alongside `format: "jpg"`, and the VIS0-C2A backend writes lossless PNG, which has no quality parameter. Its presence makes the scene non-executable rather than being ignored. |

### C2B executability gate (manifest valid ≠ runtime executable)

`GoldenSceneManifest` `1.1.0` is **not** narrowed by this tranche. It still
accepts an absent `capture.frame` and still accepts `jpg`/`exr`, while the
VIS0-C2A runtime implements neither. The runner therefore distinguishes two
states and records both in `capture_plan`:

- `executable: true` — the manifest can be honoured by the real runtime;
- `executable: false` plus `blocking_reasons` — formally valid, not runnable.

Blocking conditions: `capture.frame` absent, `capture.frame != warmup`,
`capture.format != png`, `capture.quality` present, `capture.filename` absent.
On `--execute` a blocked scene fails closed with exit `1` **before** any process
starts; a dry run still succeeds and prints the reasons, so the block stays
inspectable. A blocked scene emits **no** capture flag at all, because printing a
command the runtime is known to reject would make the plan a lie.

### Metadata-only fields

Recorded for provenance, with no runtime flag by design:
`schema_version`, `scene_id`, `description`, `tags`, `reference_hardware`.

`capture.filename` and `capture.format` used to be in this list. Since VIS0-C2B
they really drive `rcsim-app`, so labelling them metadata-only would have been
false.

### Flags deliberately not emitted

`--debug-overlays`, `--record-replay`, `--controller-profile` and the
developer-only `--rv2-6-validation-scene` / `--rv2-6-validation-ap` gates exist at
runtime but are **not expressible in the v1 manifest**, so the runner leaves them
at their runtime defaults. The `--rv2-6-validation-*` gates are developer-only and
would override camera, scenery and debug state; emitting them would silently
invalidate a golden scene.

`--altitude-m` and `--airspeed-mps` used to belong to this list. VIS0-C1B made
them expressible (`aircraft.altitude_m` / `aircraft.airspeed_mps`) and therefore
emittable, which removed the last runtime default that silently shaped the
airborne initial state of a golden scene.

## Determinism

- `command_argv` is derived by walking a single fixed list
  (`CANONICAL_FIELD_ORDER`), so manifest key insertion order cannot change it.
- Numbers are formatted with the shortest round-trip representation, which Rust's
  `f32`/`f64` `parse` accepts verbatim. Integers stay integral (`55`, not `55.0`).
- Output naming is `<output-dir>/<scene-id>/` with **no timestamp**, per the VIS0
  naming convention. Timestamps live inside `run.json`, never in a golden name.
- `manifest_sha256` pins the exact manifest bytes the plan was built from.
- The plan is pure: no clock and no randomness enter it. Two dry runs of the same
  manifest produce byte-identical `command_argv` and `field_mapping`.

`command_display` is a `shlex.join` rendering for human reading only.
**`command_argv` is authoritative** and is passed to `subprocess.run` as a list
with `shell=False`, so paths containing spaces and shell metacharacters are
never re-interpreted.

## Output structure

```
<output-dir>/
    <scene-id>/
        <capture.filename>              # real lossless RGBA8 PNG (VIS0-C2A runtime)
        runtime_capture_receipt.json    # RuntimeCaptureReceipt 1.0.0, written by rcsim-app
        capture_evidence.json           # VisualCaptureEvidence 1.0.0 — authoritative artifact
        run.json                        # full provenance + the plan + an embedded evidence copy
        stdout.txt                      # captured app stdout
        stderr.txt                      # captured app stderr
```

A dry run creates **nothing** — no directory, no file, no image, no receipt —
unless `--plan-json` is given. The default `--output-dir` is
`tmp/visual_benchmark_runs`, and `tmp/` is already gitignored, so run artifacts
can never be committed by accident.

`--capture-out` and `--capture-receipt-out` are passed as **absolute** paths, so
the `image_path` the runtime echoes into the receipt is interpretable without
knowing the subprocess working directory. Human-readable metadata may still use
repo-relative display paths.

### Stale artifact safety

Before a real execution the runner removes a pre-existing capture image,
`runtime_capture_receipt.json`, `capture_evidence.json` and their `.tmp`
siblings, and **fails closed** (exit `4`) if any removal is refused. The runtime
has its own stale-output policy, but it cannot help when the process never
starts or dies before its cleanup runs; without this step an image from a
previous run would sit at the expected path and be readable as this run's
result. Only paths inside `<output-dir>/<scene-id>/` are ever touched, so
approved baselines elsewhere are never deleted.

## Capture capability status

**Supported — the VIS0-C2A capture backend really writes an image.**

```json
"capture_backend": {
  "status": "supported",
  "produces_image": true,
  "final_display_referred_capture": true,
  "frame_selection":            {"available": true, "mechanism": "--capture-frame N (zero-based presentation frame)"},
  "runtime_receipt":            {"available": true, "kind": "runtime_capture_receipt", "schema_version": "1.0.0"},
  "process_auto_exit":          {"available": true, "mechanism": "--exit-after-frame N (runner-derived)"},
  "explicit_resolution_enforcement": {"available": true, "mechanism": "--render-width/--render-height"},
  "supported_formats": ["png"],
  "requested_format": "png",
  "requested_format_executable": true,
  "executable_for_this_manifest": true,
  "capture_produced": null
}
```

`capture_produced` stays `null` in the plan: a capability describes the runtime,
while whether *this* run produced an image is a per-run fact reported only after
execution. A dry run therefore correctly declares
`capture backend = supported` **and** `capture produced = false`.

The captured pixels are the final display-referred output of postprocess
(`Rgba8UnormSrgb`, post exposure and Khronos PBR Neutral), encoded as lossless
RGBA8 PNG. See
[`rv2_vis0_c2a_frame_capture.md`](rv2_vis0_c2a_frame_capture.md) for the runtime
contract and
[`rv2_vis0_c2b_capture_evidence_integration.md`](rv2_vis0_c2b_capture_evidence_integration.md)
for the tooling integration.

`expected_output_basename` still resolves the VIS0 golden name and reports
whether the manifest's `capture.filename` matches the
`<scene_id>_<width>x<height>.<format>` convention. A mismatch prints a warning;
it does not fail the run, because VIS0-A owns that rule. Its pre-C2A
`produced: false` / "backend unavailable" wording was removed: it became
semantically false once the runtime started writing PNGs. The block now reports
`planned_path`, `expected_filename`, `format`, `executable`, and a `verified`
field that stays `null` until a real execute confirms the artifact.

The following are explicitly **out of scope** and are not implemented anywhere in
this runner: desktop screenshot automation, Win32 screen scraping,
`PIL`/`ImageGrab`, pixel comparison, SSIM/PSNR, OpenCV, LPIPS, perceptual diff,
baseline comparison and golden promotion. Reading a PNG header (33 fixed bytes)
is not pixel analysis and needs no image library. A unit test asserts the
runner's imports are standard-library only and that no image or metric symbol is
referenced.

## Resolution enforcement status

**Supported — enforced after VIS0-C1 runtime control convergence.**

```json
"resolution_enforcement": {
  "status": "supported",
  "enforced": true,
  "requested": {"width": 1920, "height": 1080},
  "reason": "resolution enforcement: supported via --render-width/--render-height CLI ..."
}
```

Manifest resolution is now mapped to `--render-width` and `--render-height`,
emitted by the runner, and enforced by the runtime fail-closed.

## Provenance (`run.json`)

| Group | Fields |
| --- | --- |
| `manifest` | `schema_version`, `scene_id`, `path`, `path_display`, `sha256` |
| `environment` | `operating_system`, `os_release`, `os_version`, `architecture`, `platform`, `python_version`, `python_implementation`, `python_executable`, `runner_name`, `runner_version` |
| `git` | `commit_sha`, `commit_sha_short`, `branch`, `detached_head`, `upstream`, `remote_origin_url`, `dirty`, `dirty_entry_count`, `dirty_tracked_entry_count`, `dirty_entries`, `errors` |
| `plan` | the full execution plan, embedded verbatim |
| `execution` | `mode`, `app`, `app_resolved`, `command_argv`, `cwd`, `timeout_seconds`, `started_at_utc`, `ended_at_utc`, `duration_seconds`, `exit_code`, `failure_kind`, `failure_message`, `execution_success`, `stdout_path`, `stderr_path`, stream byte counts, `stale_artifacts_removed`, the planned capture/receipt/evidence paths, `artifacts_written` |
| `capabilities` | `capture_backend`, `resolution_enforcement`, `warmup_frames`, `process_auto_exit` |
| `capture_verification` | the independent verification result: `attempted`, `trusted`, `process_ok`, the parsed `receipt`, `receipt_errors`, `expectation_errors`, the verified `image`, `image_errors`, the ordered `checks` list and `failure_reason` (VIS0-C2B) |
| `capture_evidence` | a `VisualCaptureEvidence` document: tooling-supplied leaves always filled, runtime-supplied leaves filled **only** from a trusted receipt and `null` otherwise |
| `capture_evidence_validation` | `valid`, `error_count`, `errors` — the evidence checked against its own contract |
| `capture_evidence_artifact` | `path`, `path_display`, `written`, `note` — whether the standalone authoritative artifact was published |
| `artifacts` | `run_json`, `stdout`, `stderr`, `capture` (verified image path or `null`), `capture_verified`, `capture_reason`, `runtime_capture_receipt`, `capture_evidence` |
| `verdict` | `execution_success`, `capture_success`, `runner_success`, `visual_pass` (always `null`), `visual_pass_reason` |

The embedded `plan` additionally carries `capture_plan` (paths, executability,
blocking reasons, manifest-driven vs runner-derived arguments) and
`capture_evidence_contract`, which names the evidence schema and validator and
lists the producer handshake.

### `execution_success` is not `capture_success` is not `visual_pass`

Three different statements, kept structurally separate:

- `execution_success` — *procedural*: the process was launched and exited `0`.
- `capture_success` — *procedural*: an image artifact was really produced, and
  its receipt and PNG were independently verified.
- `visual_pass` — *visual*: the rendered image meets an approved quality bar.

`visual_pass` is **always `null`**. VIS0 has no approved metric, so it cannot
evaluate a visual result and must not pretend to. `capture_success: true` means
"these bytes exist and match their receipt", never "this image is correct".

### Capture evidence (VIS0-C1B contract, VIS0-C2B producer)

`run.json` carries a `VisualCaptureEvidence` document that is valid against
`tools/visual_benchmark/visual_capture_evidence.schema.json`, and since VIS0-C2B
the same document is also written standalone to `capture_evidence.json` — which
is the **authoritative** artifact, `run.json` merely embedding a convenience
copy.

- On a verified capture: `execution.capture_success` is `true`,
  `execution.failure_reason` is `null`, `capture.actual.*` comes from the
  receipt, and `capture.image.*` comes from the independently measured file.
- On any failure: `capture_success` is `false`, `failure_reason` states
  concretely what happened, and every `capture.image.*` **and**
  `capture.actual.*` leaf is `null` — a failed run never advertises an image.
- `hardware.gpu_adapter_name`, `graphics_backend` and `driver_version` stay
  `null`: `RuntimeCaptureReceipt` `1.0.0` carries no adapter, backend or driver
  field, log text is not parsed as an authority, and the host GPU is never
  guessed. `operating_system`, `os_release` and `architecture` are
  tooling-visible and filled.
- `verdict.visual_pass` is `null`.

Every leaf the contract defines is emitted as an explicit key — the unknown ones
as `null`, never as an omission, because the evidence contract treats a missing
key as an incomplete artifact and only an explicit `null` as an honest
"unavailable" (`test_unexecuted_evidence_omits_no_contract_leaf` enforces this).
The runner validates its own document and records the result in
`capture_evidence_validation`. **If the validator rejects it, the run fails with
exit `4` and no standalone artifact is published**; the rejected document and
its errors stay in `run.json` so the failure is auditable. When git provenance is
unavailable the mandatory `source.commit_sha` cannot be filled: that is reported
as `valid: false`, never raised and never papered over with a placeholder.

See `docs/architecture/rv2_vis0_c1b_capture_evidence.md` for the contract and
`docs/architecture/rv2_vis0_c2b_capture_evidence_integration.md` for the
producer.

### Process and capture failure handling

`subprocess.run` is used with `shell=False` and an argv list. Handled without a
traceback, each producing a coherent runner exit code:

| Condition | `failure_kind` | Runner exit |
| --- | --- | --- |
| Executable missing | `executable_not_found` | `4` |
| Not executable | `permission_denied` | `4` |
| Other OS error | `os_error` | `4` |
| Exceeded `--timeout-seconds` | `timeout` (process killed) | `4` |
| Non-zero exit | `null`, `exit_code` preserved | `4` |
| Scene not runtime-executable (C2B policy) | — (no process started) | `1` |
| Stale artifact could not be removed | — | `4` |
| Exit `0` but receipt missing/invalid/mismatched | `null` | `4` |
| Exit `0` but PNG unverified (size, SHA, header, RGBA8) | `null` | `4` |
| Evidence rejected by its own validator | `null` | `4` |
| Ctrl-C | — | `5` |

Partial stdout/stderr captured before a timeout is still written, and `run.json`
is written even when the run failed, so failed runs remain auditable.

## Git provenance and `--require-clean-git`

Git state is collected with the `git` CLI via `subprocess` — no dependency:

```
git rev-parse HEAD
git rev-parse --short=12 HEAD
git branch --show-current
git rev-parse --abbrev-ref --symbolic-full-name @{u}
git remote get-url origin
git status --porcelain
```

Missing git, a missing upstream, or a detached HEAD are recorded as absences, not
errors, and never block a run.

`--require-clean-git` is opt-in and **strict**: it requires
`git status --porcelain` to be completely empty, so *untracked* entries count as
dirty. If git status is unavailable the flag fails closed rather than passing.

> In this repository the check will always fail: the work tree carries
> pre-existing untracked `.worktrees/` and `build.log`. That is expected, which
> is why the flag is opt-in. `dirty_tracked_entry_count` is recorded separately
> so the distinction is visible in `run.json`.

A dirty repository does **not** block a dry run or a normal execution; it is
simply recorded.

## Validation gating

Manifest validation always runs first, in dry run and in execution alike. An
invalid manifest produces a readable error list and exit `1`, and:

- no process is started,
- no output directory is created,
- the git policy is not even consulted.

The runner adds no validation rules of its own and never relaxes the VIS0-A
contract. It does fail closed in one extra place: if a manifest carries a field
with no runner policy, the runner refuses to plan it rather than silently
ignoring the field.

## Testing

```
python tools/visual_benchmark/validate_manifest.py \
  docs/validation/visual_benchmark/vis0_reference_scene.json

python -X utf8 -m unittest tools/visual_benchmark/test_validate_manifest.py -v
python -X utf8 -m unittest tools/visual_benchmark/test_validate_capture_evidence.py -v
python -X utf8 -m unittest tools/visual_benchmark/test_run_benchmark.py -v
```

Regression guards (the tooling must never perturb the workspace):

```
cargo fmt --all -- --check
cargo test --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
```

The runner suite needs **no graphical process, no window and no GPU**. Real
subprocess coverage uses the Python interpreter as a harmless stand-in
executable, so the execution, non-zero-exit and timeout paths are exercised for
real. The end-to-end capture path is exercised against a test double that
reproduces the VIS0-C2A **artifact contract** — it writes a real RGBA8 PNG
(stdlib `zlib` + `struct`, no image library) to `--capture-out` and a
byte-accurate `RuntimeCaptureReceipt` to `--capture-receipt-out`, with knobs for
a missing receipt, a malformed digest, a wrong frame, a wrong image path, a
corrupt signature and a non-zero exit. Nothing in it renders.

Coverage includes: plan success, invalid-manifest gating, determinism,
pilot/chase/exposure/debug mapping, no-invented-flags (verified against the
runtime source), argv safety with spaces and metacharacters, missing executable,
non-zero exit, timeout, provenance validity, git SHA parsing, the strict
clean-git policy, `visual_pass` never auto-approved, the real capture flag
mapping, runner-derived vs manifest-driven argument provenance, the C2B
executability gate (warmup/frame relation, PNG-only, quality), dry-run side-effect
freedom, stale artifact removal, strict receipt parsing and expectation
checking, independent PNG verification, requested-vs-actual ownership, and
evidence validation gating the runner exit code.

## Capability status after VIS0-C2B

1. ~~**No capture backend.**~~ *Resolved by VIS0-C2A, consumed by VIS0-C2B:*
   `--capture-frame N --capture-out PATH --capture-format png
   [--capture-receipt-out PATH]` writes a lossless RGBA8 PNG of the final
   display-referred frame plus a `RuntimeCaptureReceipt` `1.0.0`.
2. **Auto-exit is resolved; headless rendering is separate.** VIS0-C1's
   `--exit-after-frame` provides deterministic frame-bounded exit, and the
   runner derives it from the captured presentation frame. Rendering is still
   windowed; VIS0-C1 did not implement a headless renderer, and that is not a
   capture blocker.
3. ~~**No resolution control.**~~ *Resolved by VIS0-C1:* `--render-width` and
   `--render-height` request an explicit physical window/client framebuffer
   extent, verified fail-closed before renderer initialization. The authoritative
   `capture.actual` extent still comes from the receipt, never from the request.
4. ~~**No warmup / frame selection for capture.**~~ *Resolved by VIS0-C2A +
   VIS0-C2B:* `--capture-frame` selects a zero-based presentation frame, and
   warmup is derived from the relation `warmup == capture.frame`. There is still
   no `--warmup` flag, and arbitrary warmup/frame combinations remain
   unsupported by design — the runner fails closed on them.
5. ~~**Contract gap: aircraft state is not fully manifest-controlled.**~~
   *Resolved by VIS0-C1B:* `aircraft.altitude_m` and `aircraft.airspeed_mps` are
   now manifest fields mapped to `--altitude-m` / `--airspeed-mps`.
6. **No approved visual metric.** Thresholds are undefined in VIS0-A, so no
   automated PASS/FAIL can exist yet. This is the remaining VIS0-C gap and is
   deliberately untouched here.
7. **No machine-readable GPU/backend/driver handshake.** `RuntimeCaptureReceipt`
   `1.0.0` has no adapter field, so `hardware.gpu_adapter_name`,
   `graphics_backend` and `driver_version` stay `null`.
