# RV2-VIS0-B — Golden Visual Benchmark Runner

## Purpose

VIS0-B turns the VIS0-A manifest contract into tooling that actually runs.

VIS0-A ([`rv2_vis0_visual_benchmark.md`](rv2_vis0_visual_benchmark.md)) defines *what*
a golden scene is and validates that a manifest is well formed. VIS0-B consumes a
validated manifest and produces:

1. a **deterministic execution plan**,
2. the **exact `rcsim-app` command** that plan implies,
3. optional **real process execution** with captured stdout/stderr/exit code,
4. **run provenance** (`run.json`) sufficient to reproduce or audit the run.

**VIS0-B verifies execution reproducibility, NOT visual image quality.**

It never decides a visual PASS/FAIL, never reads pixels, and never produces an
image. `visual_pass` is written as `null` in every artifact this runner emits.

## Scope boundary

VIS0-B is deliberately independent of the renderer runtime. It touches only:

- `tools/visual_benchmark/**`
- `docs/validation/visual_benchmark/**`
- `docs/architecture/**` (VIS0 documentation only)

No `crates/**`, `Cargo.toml`, `Cargo.lock`, `.github/**`, `main` or
`integration/render-v2` change. Where the runtime cannot support a manifest
field, this runner **records the gap** instead of modifying the runtime or
faking the capability.

The VIS0-A contract files stay authoritative and byte-identical:
`golden_scene_manifest.schema.json`, `validate_manifest.py`,
`test_validate_manifest.py`, `vis0_reference_scene.json`. The runner *imports*
`validate_manifest.ManifestValidator` rather than reimplementing any rule.

## Usage

All commands run from the repository root. Python standard library only — no
pip dependency.

### Dry run (the default; starts no process)

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

> **Expect a timeout today.** `rcsim-app render` opens an interactive winit
> window and runs until Escape or window close. It has no headless mode and no
> frame limit, so an unattended run ends in `--timeout-seconds`. This is a
> runtime gap recorded in `run.json` under
> `capabilities.process_auto_exit`, not a runner bug. See
> [Blockers for VIS0-C](#blockers-for-vis0-c).

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
| `--dry-run` | on by default | Build and print the plan; start no process, write no run artifacts |
| `--execute` | off | Opt in to really launching the app process |
| `--app PATH_OR_COMMAND` | none | `rcsim-app` executable. Required with `--execute` |
| `--output-dir PATH` | `tmp/visual_benchmark_runs` | Root for run artifacts (gitignored) |
| `--timeout-seconds N` | `120` | Kill the app process after N seconds |
| `--run-index N` | `0` | Run index recorded in the plan and `run.json` |
| `--plan-json PATH` | none | Also write the execution plan as JSON |
| `--require-clean-git` | off | Fail with exit 3 unless `git status --porcelain` is empty |

`--dry-run` wins if both `--dry-run` and `--execute` are given; a note is printed
to stderr and no process starts.

There is no interactive prompt of any kind. Every input arrives as a flag.

### Exit codes

| Code | Meaning |
| --- | --- |
| `0` | Plan built (dry run), or the app process exited `0` |
| `1` | Manifest failed VIS0-A validation; **no process was started** |
| `2` | Usage/input error: missing file, unreadable JSON, bad flag value |
| `3` | Git policy violation (`--require-clean-git` on a dirty work tree) |
| `4` | Execution failure: app not found, non-zero exit, timeout |
| `5` | Interrupted (Ctrl-C) |

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
| `aircraft.start_on_ground` | `--start-on-ground` | presence-only flag; emitted only when `true` |

`--camera <mode>` is always emitted explicitly. This matters: without it the
runtime falls back to a *different* default camera (`Pilot` at `[0, 0.3, 0.85]`,
FOV 70) than the one `--camera pilot` selects (`[0, 1.8, 20]`, FOV 55). Emitting
the mode makes the camera fully determined by the manifest.

### Unsupported execution fields

Recorded in `run.json` with `status: "unsupported"`, `runtime_flag: null`,
`emitted: false` and an explicit reason. Never silently dropped, never faked.

| Manifest field | Status | Evidence |
| --- | --- | --- |
| `resolution.width`, `resolution.height` | **unsupported** | No CLI flag sets the client framebuffer size. The window is created with a hardcoded `with_inner_size(LogicalSize::new(1_280.0, 720.0))` in `RenderApplication::resumed`. |
| `warmup` | **unsupported** | No CLI flag controls warmup frames; the render loop has no frame counter exposed. |
| `capture.frame` | **unsupported** | No CLI flag selects a capture frame; no image is written at all. |
| `capture.quality` | **unsupported** | JPEG quality only matters to an encoder that does not exist yet. |

### Metadata-only fields

Recorded for provenance, with no runtime flag by design:
`schema_version`, `scene_id`, `description`, `tags`, `reference_hardware`,
`capture.filename`, `capture.format`.

### Flags deliberately not emitted

`--altitude-m`, `--airspeed-mps`, `--debug-overlays`, `--record-replay`,
`--controller-profile` and the developer-only `--rv2-6-validation-scene` /
`--rv2-6-validation-ap` gates exist at runtime but are **not expressible in the
v1 manifest**, so the runner leaves them at their runtime defaults. The
`--rv2-6-validation-*` gates are developer-only and would override camera,
scenery and debug state; emitting them would silently invalidate a golden scene.

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
        run.json      # full provenance + the plan
        stdout.txt    # captured app stdout
        stderr.txt    # captured app stderr
```

A dry run creates **nothing** — no directory, no file — unless `--plan-json` is
given. The default `--output-dir` is `tmp/visual_benchmark_runs`, and `tmp/` is
already gitignored, so run artifacts can never be committed by accident.

No capture image is written, because the runtime cannot produce one.

## Capture capability status

**CAPTURE BACKEND NOT YET AVAILABLE.**

`rcsim-app render` exposes no lossless framebuffer save. The render subcommand
creates a winit window and runs an interactive event loop; it never writes an
image artifact. The `image` crate appears in the workspace only for texture
decoding (`crates/renderer/src/texture.rs`) and the offline terrain texture
generator (`crates/renderer/src/bin/generate_terrain_textures.rs`); the
PNG/JPEG encoders in `crates/renderer/src/glb.rs` are test-only.

VIS0-B therefore stops at execution and provenance, and records the capability
as unavailable in both `plan.json` and `run.json`:

```json
"capture_backend": {
  "status": "unavailable",
  "produces_image": false,
  "reason": "CAPTURE BACKEND NOT YET AVAILABLE: ...",
  "expected_filename": "aircraft_acro_static_front_1920x1080.png",
  "expected_format": "png"
}
```

`expected_output_basename` still resolves the VIS0 golden name and reports
whether the manifest's `capture.filename` matches the
`<scene_id>_<width>x<height>.<format>` convention. A mismatch prints a warning;
it does not fail the run, because VIS0-A owns that rule.

The following are explicitly **out of scope** and are not implemented anywhere in
this runner: desktop screenshot automation, Win32 screen scraping,
`PIL`/`ImageGrab`, pixel comparison, SSIM/PSNR, OpenCV, LPIPS, perceptual diff.
A unit test asserts the runner's imports are standard-library only and that no
image or metric symbol is referenced.

## Resolution enforcement status

**Unsupported — recorded, not enforced.**

```json
"resolution_enforcement": {
  "status": "unsupported",
  "enforced": false,
  "requested": {"width": 1920, "height": 1080},
  "reason": "resolution enforcement: unsupported. `rcsim-app` has no CLI flag ..."
}
```

Any consumer reading `run.json` can see that the requested resolution was *not*
applied. Nothing in the runner claims otherwise.

## Provenance (`run.json`)

| Group | Fields |
| --- | --- |
| `manifest` | `schema_version`, `scene_id`, `path`, `path_display`, `sha256` |
| `environment` | `operating_system`, `os_release`, `os_version`, `architecture`, `platform`, `python_version`, `python_implementation`, `python_executable`, `runner_name`, `runner_version` |
| `git` | `commit_sha`, `commit_sha_short`, `branch`, `detached_head`, `upstream`, `remote_origin_url`, `dirty`, `dirty_entry_count`, `dirty_tracked_entry_count`, `dirty_entries`, `errors` |
| `plan` | the full execution plan, embedded verbatim |
| `execution` | `mode`, `app`, `app_resolved`, `command_argv`, `cwd`, `timeout_seconds`, `started_at_utc`, `ended_at_utc`, `duration_seconds`, `exit_code`, `failure_kind`, `failure_message`, `execution_success`, `stdout_path`, `stderr_path`, stream byte counts, `artifacts_written` |
| `capabilities` | `capture_backend`, `resolution_enforcement`, `warmup_frames`, `process_auto_exit` |
| `artifacts` | `run_json`, `stdout`, `stderr`, `capture` (`null`), `capture_reason` |
| `verdict` | `execution_success`, `visual_pass` (always `null`), `visual_pass_reason` |

### `execution_success` is not `visual_pass`

These are different statements and are kept structurally separate:

- `execution_success` — *procedural*: the process was launched and exited `0`.
- `visual_pass` — *visual*: the rendered image meets an approved quality bar.

`visual_pass` is **always `null`**. VIS0-B has no capture backend and no approved
metric, so it cannot evaluate a visual result and must not pretend to. A visual
verdict requires a lossless capture backend plus human review or an approved
metrics engine (VIS0-C or later).

### Process failure handling

`subprocess.run` is used with `shell=False` and an argv list. Handled without a
traceback, each producing a coherent runner exit code:

| Condition | `failure_kind` | Runner exit |
| --- | --- | --- |
| Executable missing | `executable_not_found` | `4` |
| Not executable | `permission_denied` | `4` |
| Other OS error | `os_error` | `4` |
| Exceeded `--timeout-seconds` | `timeout` (process killed) | `4` |
| Non-zero exit | `null`, `exit_code` preserved | `4` |
| Ctrl-C | — | `5` |

Partial stdout/stderr captured before a timeout is still written, and `run.json`
is written even when the app failed, so failed runs remain auditable.

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

python -m unittest tools/visual_benchmark/test_validate_manifest.py -v
python -m unittest tools/visual_benchmark/test_run_benchmark.py -v
```

Regression guards (unchanged by VIS0-B):

```
cargo fmt --all -- --check
cargo test --workspace --all-features
```

The runner suite needs **no graphical process and no GPU**. Real subprocess
coverage uses the Python interpreter itself as a harmless stand-in executable, so
the execution, non-zero-exit and timeout paths are exercised for real. Coverage
includes: plan success, invalid-manifest gating, determinism, pilot/chase/
exposure/debug mapping, no-invented-flags (verified against the runtime source),
argv safety with spaces and metacharacters, missing executable, non-zero exit,
timeout, provenance validity, git SHA parsing, the strict clean-git policy,
`visual_pass` never auto-approved, and explicit representation of unsupported
resolution/capture.

## Blockers for VIS0-C

Recorded, not implemented. Each one needs a runtime or contract change that is
out of VIS0-B scope.

1. **No capture backend.** `rcsim-app` cannot write a lossless framebuffer. VIS0-C
   needs a real capture CLI (for example `--capture-out PATH --capture-format
   png`) before any golden image can exist.
2. **No headless mode / auto-exit.** The render loop is an interactive winit
   window with `ControlFlow::Poll` and no frame limit. Unattended capture needs a
   headless or exit-after-N-frames mode.
3. **No resolution control.** The window inner size is hardcoded to
   `1280x720` logical. A golden at `1920x1080` is impossible until the framebuffer
   size is CLI-controllable.
4. **No warmup / frame selection.** `warmup` and `capture.frame` cannot be
   honoured, so capture timing is not yet reproducible.
5. **Contract gap: aircraft state is not fully manifest-controlled.**
   `--altitude-m` and `--airspeed-mps` materially change the simulated aircraft
   state but have no manifest field, so the runner leaves them at runtime
   defaults (`30.0` m and `18.0` m/s). Two runs of the same manifest are
   reproducible only because those defaults are stable. Adding them to the v1
   manifest is a VIS0-A schema decision, not a runner one.
6. **No approved visual metric.** Thresholds are undefined in VIS0-A, so no
   automated PASS/FAIL can exist yet.
