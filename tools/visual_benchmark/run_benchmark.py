#!/usr/bin/env python3
"""
RV2-VIS0-B Golden Visual Benchmark Runner

Turns an approved VIS0-A GoldenSceneManifest into a deterministic, reproducible
`rcsim-app render` invocation plus run provenance metadata.

VIS0-B verifies EXECUTION REPRODUCIBILITY, not visual image quality.
It never decides a visual PASS/FAIL, never reads pixels, and never fakes a
capture that the runtime cannot produce.

The manifest contract owned by VIS0-A stays authoritative: this runner reuses
`validate_manifest.ManifestValidator` unchanged and only maps manifest fields
onto CLI flags that really exist in `crates/app/src/render_app.rs`.

Usage:
    # Dry run (the default; no process is started)
    python tools/visual_benchmark/run_benchmark.py \
        --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
        --dry-run

    # Real execution (explicit opt-in; never the default)
    python tools/visual_benchmark/run_benchmark.py \
        --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
        --app target/release/rcsim-app --execute

Exit codes:
    0 - plan built (dry run), or the app process exited 0
    1 - manifest failed VIS0-A validation; no process was started
    2 - usage/input error (missing file, unreadable JSON, bad flag value)
    3 - git policy violation (--require-clean-git against a dirty work tree)
    4 - execution failure (app not found, non-zero exit, timeout)
    5 - interrupted (Ctrl-C)
"""

import argparse
import hashlib
import json
import os
import platform
import shlex
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional

try:  # Imported as part of the tools.visual_benchmark namespace package.
    from tools.visual_benchmark.validate_manifest import ManifestValidator
except ImportError:  # Direct script execution: script directory is sys.path[0].
    from validate_manifest import ManifestValidator


RUNNER_NAME = "rv2-vis0-benchmark-runner"
RUNNER_VERSION = "1.0.0"
PLAN_VERSION = "1.0.0"

EXIT_OK = 0
EXIT_VALIDATION_FAILED = 1
EXIT_INPUT_ERROR = 2
EXIT_GIT_POLICY = 3
EXIT_EXECUTION_FAILED = 4
EXIT_INTERRUPTED = 5

DEFAULT_TIMEOUT_SECONDS = 120
GIT_TIMEOUT_SECONDS = 15

RENDER_SUBCOMMAND = "render"

DEFAULT_OUTPUT_DIR_RELATIVE = Path("tmp") / "visual_benchmark_runs"

# Repository root, derived from this file's location: tools/visual_benchmark/.
REPO_ROOT = Path(__file__).resolve().parent.parent.parent

# The rcsim-app render CLI surface this runner is allowed to emit. Every entry
# was read from RenderOptions::parse_with_defaults and the usage string in
# crates/app/src/main.rs at the VIS0-B base commit. Nothing here is invented.
KNOWN_RENDER_FLAGS = (
    "--model",
    "--throttle",
    "--altitude-m",
    "--airspeed-mps",
    "--record-replay",
    "--controller-profile",
    "--start-on-ground",
    "--debug-overlays",
    "--terrain-debug",
    "--vegetation-debug",
    "--exposure-ev",
    "--renderer",
    "--rv2-6-validation-scene",
    "--rv2-6-validation-ap",
    "--scenery",
    "--camera",
    "--camera-fov",
    "--chase-distance-m",
    "--chase-height-m",
    "--pilot-position",
)

# Flags the runner may actually emit. A subset of KNOWN_RENDER_FLAGS: the
# runner deliberately never emits --altitude-m/--airspeed-mps (not expressible
# in the v1 manifest), --debug-overlays, --record-replay,
# --controller-profile, or the developer-only --rv2-6-validation-* gates.
EMITTABLE_FLAGS = (
    "--renderer",
    "--terrain-debug",
    "--vegetation-debug",
    "--scenery",
    "--camera",
    "--camera-fov",
    "--pilot-position",
    "--chase-distance-m",
    "--chase-height-m",
    "--exposure-ev",
    "--model",
    "--throttle",
    "--start-on-ground",
)

# --- Runtime capability gaps (verified against the runtime source) -----------

CAPTURE_UNAVAILABLE_REASON = (
    "CAPTURE BACKEND NOT YET AVAILABLE: `rcsim-app render` exposes no lossless "
    "framebuffer save CLI. The render subcommand creates a winit window and "
    "runs an interactive event loop; it never writes an image artifact. The "
    "`image` crate is used only for texture decoding "
    "(crates/renderer/src/texture.rs) and the offline terrain texture "
    "generator (crates/renderer/src/bin/generate_terrain_textures.rs). "
    "VIS0-B must not modify renderer/app, so no capture is produced and none "
    "is faked."
)

RESOLUTION_UNSUPPORTED_REASON = (
    "resolution enforcement: unsupported. `rcsim-app` has no CLI flag for the "
    "client framebuffer size; the window is created with a hardcoded "
    "`with_inner_size(LogicalSize::new(1_280.0, 720.0))` in "
    "RenderApplication::resumed (crates/app/src/render_app.rs). Manifest "
    "resolution is therefore recorded as metadata only and is NOT enforced; "
    "enforcement needs a future capture backend (VIS0-C or later)."
)

WARMUP_UNSUPPORTED_REASON = (
    "warmup frame count: unsupported. `rcsim-app` has no CLI flag for warmup "
    "frames or for capture frame selection, and the render loop has no frame "
    "limit or auto-exit. Both depend on a future capture backend."
)

CAPTURE_FRAME_UNSUPPORTED_REASON = (
    "capture frame selection: unsupported. No `rcsim-app` CLI flag selects a "
    "frame, and no image is written at all. Depends on a future capture "
    "backend."
)

CAPTURE_METADATA_ONLY_REASON = (
    "capture backend unavailable; the value is recorded as provenance metadata "
    "only and names the artifact a future capture backend is expected to "
    "produce. Nothing is written by this runner."
)

NO_AUTO_EXIT_REASON = (
    "`rcsim-app render` runs an interactive winit event loop "
    "(ControlFlow::Poll) that exits only on Escape or window close. There is "
    "no headless mode and no frame limit, so a real execution is expected to "
    "end in a timeout unless a human closes the window."
)

# --- Field policy ------------------------------------------------------------

FIELD_STATUS_CLI = "cli"
FIELD_STATUS_METADATA_ONLY = "metadata-only"
FIELD_STATUS_UNSUPPORTED = "unsupported"

# Manifest blocks whose leaves (not the block itself) carry a field policy.
CONTAINER_FIELDS = frozenset(
    {"renderer", "scenery", "camera", "resolution", "aircraft", "capture"}
)


@dataclass(frozen=True)
class FieldPolicy:
    """How one manifest field relates to the real runtime CLI."""

    status: str
    runtime_flag: Optional[str] = None
    reason: Optional[str] = None


# Deterministic order. Also the argv emission order: argv is derived by walking
# this list, so the same manifest always produces the same command and order.
CANONICAL_FIELD_ORDER = (
    "schema_version",
    "scene_id",
    "description",
    "renderer.version",
    "renderer.terrain_debug",
    "renderer.vegetation_debug",
    "scenery.preset",
    "camera.mode",
    "camera.vertical_fov_deg",
    "camera.pilot_position_render_m",
    "camera.chase_distance_behind_m",
    "camera.chase_height_above_m",
    "exposure_ev",
    "aircraft.model",
    "aircraft.throttle",
    "aircraft.start_on_ground",
    "resolution.width",
    "resolution.height",
    "warmup",
    "capture.filename",
    "capture.format",
    "capture.frame",
    "capture.quality",
    "tags",
    "reference_hardware",
)

FIELD_POLICY = {
    "schema_version": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY,
        reason="contract version; recorded in run.json only",
    ),
    "scene_id": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY,
        reason="drives the output directory name and the expected capture basename",
    ),
    "description": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY,
        reason="human-readable text; recorded in run.json only",
    ),
    "renderer.version": FieldPolicy(FIELD_STATUS_CLI, "--renderer"),
    "renderer.terrain_debug": FieldPolicy(FIELD_STATUS_CLI, "--terrain-debug"),
    "renderer.vegetation_debug": FieldPolicy(FIELD_STATUS_CLI, "--vegetation-debug"),
    "scenery.preset": FieldPolicy(FIELD_STATUS_CLI, "--scenery"),
    "camera.mode": FieldPolicy(FIELD_STATUS_CLI, "--camera"),
    "camera.vertical_fov_deg": FieldPolicy(FIELD_STATUS_CLI, "--camera-fov"),
    "camera.pilot_position_render_m": FieldPolicy(FIELD_STATUS_CLI, "--pilot-position"),
    "camera.chase_distance_behind_m": FieldPolicy(FIELD_STATUS_CLI, "--chase-distance-m"),
    "camera.chase_height_above_m": FieldPolicy(FIELD_STATUS_CLI, "--chase-height-m"),
    "exposure_ev": FieldPolicy(FIELD_STATUS_CLI, "--exposure-ev"),
    "aircraft.model": FieldPolicy(FIELD_STATUS_CLI, "--model"),
    "aircraft.throttle": FieldPolicy(FIELD_STATUS_CLI, "--throttle"),
    "aircraft.start_on_ground": FieldPolicy(FIELD_STATUS_CLI, "--start-on-ground"),
    "resolution.width": FieldPolicy(FIELD_STATUS_UNSUPPORTED, reason=RESOLUTION_UNSUPPORTED_REASON),
    "resolution.height": FieldPolicy(FIELD_STATUS_UNSUPPORTED, reason=RESOLUTION_UNSUPPORTED_REASON),
    "warmup": FieldPolicy(FIELD_STATUS_UNSUPPORTED, reason=WARMUP_UNSUPPORTED_REASON),
    "capture.filename": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY, reason=CAPTURE_METADATA_ONLY_REASON
    ),
    "capture.format": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY, reason=CAPTURE_METADATA_ONLY_REASON
    ),
    "capture.frame": FieldPolicy(
        FIELD_STATUS_UNSUPPORTED, reason=CAPTURE_FRAME_UNSUPPORTED_REASON
    ),
    "capture.quality": FieldPolicy(
        FIELD_STATUS_UNSUPPORTED, reason=CAPTURE_FRAME_UNSUPPORTED_REASON
    ),
    "tags": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY, reason="categorisation; recorded in run.json only"
    ),
    "reference_hardware": FieldPolicy(
        FIELD_STATUS_METADATA_ONLY,
        reason="baseline hardware provenance; recorded in run.json only",
    ),
}


class RunnerError(Exception):
    """A predictable user-facing error. Never rendered as a traceback."""

    def __init__(self, message: str, exit_code: int = EXIT_INPUT_ERROR):
        super().__init__(message)
        self.message = message
        self.exit_code = exit_code


@dataclass
class FieldMapping:
    """Resolution of one manifest field into the execution plan."""

    manifest_field: str
    status: str
    runtime_flag: Optional[str]
    manifest_value: Any
    argv_value: Optional[str]
    emitted: bool
    reason: Optional[str] = None

    def to_json(self) -> dict:
        return {
            "manifest_field": self.manifest_field,
            "status": self.status,
            "runtime_flag": self.runtime_flag,
            "manifest_value": self.manifest_value,
            "argv_value": self.argv_value,
            "emitted": self.emitted,
            "reason": self.reason,
        }


@dataclass
class ProcessOutcome:
    """Result of one shell-free subprocess invocation."""

    argv: list
    returncode: Optional[int] = None
    stdout: str = ""
    stderr: str = ""
    failure_kind: Optional[str] = None
    failure_message: Optional[str] = None
    started_at_utc: str = ""
    ended_at_utc: str = ""
    duration_seconds: float = 0.0

    @property
    def succeeded(self) -> bool:
        return self.failure_kind is None and self.returncode == 0


# --- Formatting helpers ------------------------------------------------------


def utc_now_iso() -> str:
    """Second-resolution UTC timestamp in stable ISO-8601 form."""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def format_number(value: Any) -> str:
    """Format a manifest number so Rust's f32/f64 `parse` accepts it verbatim.

    Integers stay integral (`55`, not `55.0`); floats use Python's shortest
    round-trip repr, which Rust parses exactly.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RunnerError(f"expected a number, got {type(value).__name__}")
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def format_vector3(value: Any) -> str:
    """Format `[x, y, z]` as the `x,y,z` string `--pilot-position` expects."""
    if not isinstance(value, list) or len(value) != 3:
        raise RunnerError("--pilot-position needs a 3-element array")
    return ",".join(format_number(component) for component in value)


def sha256_of_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def display_path(path: Path, root: Path) -> str:
    """Repo-relative POSIX path when possible, else the absolute native path."""
    try:
        return path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        return str(path)


def decode_stream(value: Any) -> str:
    """Normalise possibly-binary, possibly-None subprocess stream output."""
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return value


def configure_streams() -> None:
    """Keep output lossless instead of crashing on a legacy console codepage.

    A Windows console or pipe may use cp1252, where any non-ASCII character
    raises UnicodeEncodeError. Runner output is deliberately ASCII, and streams
    are switched to replacement mode so a redirected third-party message can
    never turn a successful run into a traceback.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors="replace")
        except (ValueError, OSError):
            pass


# --- Manifest loading and validation -----------------------------------------


def load_manifest(manifest_path: Path) -> dict:
    if not manifest_path.exists():
        raise RunnerError(f"manifest not found: {manifest_path}")
    if not manifest_path.is_file():
        raise RunnerError(f"manifest is not a file: {manifest_path}")
    try:
        with open(manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
    except json.JSONDecodeError as error:
        raise RunnerError(f"manifest is not valid JSON: {error}")
    except OSError as error:
        raise RunnerError(f"manifest is not readable: {error}")
    if not isinstance(manifest, dict):
        raise RunnerError(
            f"manifest root must be an object, got {type(manifest).__name__}"
        )
    return manifest


def validate_manifest_dict(manifest: dict, base_path: Path) -> list:
    """Run the approved VIS0-A validator. Returns its error list (empty = valid).

    The VIS0-A validator stays the single source of truth; this runner adds no
    rules of its own and never relaxes the contract.
    """
    validator = ManifestValidator(manifest, base_path)
    validator.validate()
    return [str(error) for error in validator.errors]


# --- Field mapping and argv construction -------------------------------------


def get_field(manifest: dict, dotted: str) -> Any:
    """Read a dotted manifest path. Returns None when absent."""
    value: Any = manifest
    for part in dotted.split("."):
        if not isinstance(value, dict) or part not in value:
            return None
        value = value[part]
    return value


def _reject_unmapped_fields(manifest: dict) -> None:
    """Fail closed if the manifest carries a field this runner has no policy for.

    The VIS0-A validator already rejects unknown properties, so this only
    triggers if the contract grows and the runner is not updated - in which
    case silently ignoring a new field would be worse than stopping.
    """
    present = set()
    for key, value in manifest.items():
        if key in CONTAINER_FIELDS:
            # Only the leaf fields are policy-bearing; the container itself is
            # not listed in CANONICAL_FIELD_ORDER.
            if isinstance(value, dict):
                for nested in value:
                    present.add(f"{key}.{nested}")
        else:
            present.add(key)
    unmapped = sorted(present - set(CANONICAL_FIELD_ORDER))
    if unmapped:
        raise RunnerError(
            "manifest field(s) have no runner policy and cannot be mapped "
            f"honestly: {unmapped}. Update CANONICAL_FIELD_ORDER/FIELD_POLICY "
            "before running this manifest."
        )


def build_field_mappings(manifest: dict) -> list:
    """Resolve every present manifest field into a FieldMapping.

    Order follows CANONICAL_FIELD_ORDER, which makes both the mapping list and
    the derived argv deterministic for a given manifest.
    """
    _reject_unmapped_fields(manifest)

    camera_mode = get_field(manifest, "camera.mode")
    mappings = []
    for dotted in CANONICAL_FIELD_ORDER:
        value = get_field(manifest, dotted)
        if value is None:
            continue
        policy = FIELD_POLICY[dotted]
        reason = policy.reason
        argv_value = None
        emitted = False

        if policy.status == FIELD_STATUS_CLI:
            argv_value, emitted, extra_reason = _resolve_cli_value(dotted, value, camera_mode)
            if extra_reason:
                reason = extra_reason

        mappings.append(
            FieldMapping(
                manifest_field=dotted,
                status=policy.status,
                runtime_flag=policy.runtime_flag,
                manifest_value=value,
                argv_value=argv_value,
                emitted=emitted,
                reason=reason,
            )
        )
    return mappings


def _resolve_cli_value(dotted: str, value: Any, camera_mode: Any):
    """Return (argv_value, emitted, reason) for one CLI-mapped field."""
    if dotted == "camera.pilot_position_render_m":
        if camera_mode != "pilot":
            return (
                format_vector3(value),
                False,
                f"present but camera.mode is '{camera_mode}'; rcsim-app rejects "
                "--pilot-position outside pilot mode "
                "(RenderAppError::IncompatibleCameraOption), so it is omitted",
            )
        return format_vector3(value), True, None

    if dotted in ("camera.chase_distance_behind_m", "camera.chase_height_above_m"):
        if camera_mode != "chase":
            return (
                format_number(value),
                False,
                f"present but camera.mode is '{camera_mode}'; rcsim-app rejects "
                "chase options outside chase mode "
                "(RenderAppError::IncompatibleCameraOption), so it is omitted",
            )
        return format_number(value), True, None

    if dotted == "aircraft.start_on_ground":
        # Presence-only flag: emitting it means true, omitting it means false.
        if value is True:
            return "", True, None
        return "", False, "presence-only flag; omitted because the value is false"

    if dotted == "camera.mode":
        return str(value), True, None

    if dotted == "renderer.version":
        return str(value), True, None

    if dotted in ("renderer.terrain_debug", "renderer.vegetation_debug", "scenery.preset"):
        return str(value), True, None

    if dotted == "aircraft.model":
        return str(value), True, None

    if dotted in ("camera.vertical_fov_deg", "exposure_ev", "aircraft.throttle"):
        return format_number(value), True, None

    raise RunnerError(f"no CLI value resolver for manifest field '{dotted}'")


def build_command_argv(app: str, mappings: list) -> list:
    """Build the full argv list from the resolved field mappings.

    Returned as a list, never a shell string, so paths with spaces stay safe
    and no shell is involved.
    """
    argv = [app, RENDER_SUBCOMMAND]
    for mapping in mappings:
        if not mapping.emitted or mapping.runtime_flag is None:
            continue
        if mapping.runtime_flag not in EMITTABLE_FLAGS:
            raise RunnerError(
                f"internal error: {mapping.runtime_flag} is not an emittable flag"
            )
        argv.append(mapping.runtime_flag)
        if mapping.argv_value:
            argv.append(mapping.argv_value)
    _assert_no_invented_flags(argv)
    return argv


def _assert_no_invented_flags(argv: list) -> None:
    """Fail closed rather than pass a flag rcsim-app does not implement."""
    for token in argv[2:]:
        if token.startswith("--"):
            if token not in KNOWN_RENDER_FLAGS:
                raise RunnerError(
                    f"refusing to emit unknown CLI flag '{token}'; it does not "
                    "exist in crates/app/src/render_app.rs"
                )
            if token not in EMITTABLE_FLAGS:
                raise RunnerError(
                    f"refusing to emit '{token}': not expressible from the v1 "
                    "manifest contract"
                )


# --- Execution plan ----------------------------------------------------------


def build_environment_metadata() -> dict:
    return {
        "runner_name": RUNNER_NAME,
        "runner_version": RUNNER_VERSION,
        "operating_system": platform.system(),
        "os_release": platform.release(),
        "os_version": platform.version(),
        "architecture": platform.machine(),
        "platform": platform.platform(),
        "python_version": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "python_executable": sys.executable,
    }


def build_runtime_capabilities(manifest: dict) -> dict:
    """Declare what the runtime can and cannot do for VIS0 today.

    These states are verified against the runtime source at the VIS0-B base
    commit and are surfaced verbatim in plan.json and run.json so no consumer
    can mistake an unenforced value for an enforced one.
    """
    resolution = manifest.get("resolution") or {}
    capture = manifest.get("capture") or {}
    return {
        "capture_backend": {
            "status": "unavailable",
            "produces_image": False,
            "reason": CAPTURE_UNAVAILABLE_REASON,
            "expected_filename": capture.get("filename"),
            "expected_format": capture.get("format"),
        },
        "resolution_enforcement": {
            "status": "unsupported",
            "enforced": False,
            "requested": {
                "width": resolution.get("width"),
                "height": resolution.get("height"),
            },
            "reason": RESOLUTION_UNSUPPORTED_REASON,
        },
        "warmup_frames": {
            "status": "unsupported",
            "enforced": False,
            "requested": manifest.get("warmup"),
            "reason": WARMUP_UNSUPPORTED_REASON,
        },
        "process_auto_exit": {
            "status": "unsupported",
            "reason": NO_AUTO_EXIT_REASON,
        },
    }


def expected_capture_basename(manifest: dict) -> dict:
    """Resolve the VIS0 golden naming convention without producing a file."""
    scene_id = manifest.get("scene_id")
    resolution = manifest.get("resolution") or {}
    capture = manifest.get("capture") or {}
    width = resolution.get("width")
    height = resolution.get("height")
    fmt = capture.get("format")
    filename = capture.get("filename")

    convention = None
    if scene_id and isinstance(width, int) and isinstance(height, int) and fmt:
        convention = f"{scene_id}_{width}x{height}.{fmt}"

    return {
        "from_manifest": filename,
        "per_vis0_convention": convention,
        "matches_vis0_convention": (
            None if convention is None or filename is None else filename == convention
        ),
        "produced": False,
        "reason": CAPTURE_UNAVAILABLE_REASON,
    }


def build_plan(
    manifest: dict,
    manifest_path: Path,
    output_dir: Path,
    app: Optional[str],
    run_index: int,
    dry_run: bool,
    timeout_seconds: Optional[int],
    require_clean_git: bool,
    git_provenance: dict,
) -> dict:
    """Build the deterministic execution plan for one manifest."""
    mappings = build_field_mappings(manifest)
    scene_id = manifest.get("scene_id")
    scene_output_dir = output_dir / scene_id
    app_token = app if app is not None else "<--app not provided>"
    argv = build_command_argv(app_token, mappings)

    schema_version = manifest.get("schema_version") or ""
    major = schema_version.split(".")[0] if schema_version else ""

    return {
        "plan_version": PLAN_VERSION,
        "runner": {"name": RUNNER_NAME, "version": RUNNER_VERSION},
        "run_index": run_index,
        "dry_run": dry_run,
        "execution_requested": not dry_run,
        "scene_id": scene_id,
        "schema_version": schema_version,
        "schema_version_major_supported": major == "1",
        "manifest_path": str(manifest_path),
        "manifest_path_display": display_path(manifest_path, REPO_ROOT),
        "manifest_sha256": sha256_of_file(manifest_path),
        "renderer": get_field(manifest, "renderer.version"),
        "resolution": manifest.get("resolution"),
        "camera": manifest.get("camera"),
        "expected_output_basename": expected_capture_basename(manifest),
        "output_dir": str(output_dir),
        "scene_output_dir": str(scene_output_dir),
        "artifact_paths": {
            "run_json": str(scene_output_dir / "run.json"),
            "stdout": str(scene_output_dir / "stdout.txt"),
            "stderr": str(scene_output_dir / "stderr.txt"),
        },
        "command_argv": argv,
        "command_display": shlex.join(argv),
        "command_display_note": (
            "command_argv is authoritative. command_display uses POSIX quoting "
            "and is for human reading only; no shell is ever involved."
        ),
        "timeout_seconds": None if dry_run else timeout_seconds,
        "environment_metadata": build_environment_metadata(),
        "field_mapping": [mapping.to_json() for mapping in mappings],
        "runtime_capabilities": build_runtime_capabilities(manifest),
        "git": git_provenance,
        "execution_policy": {
            "require_clean_git": require_clean_git,
            "shell": False,
            "default_is_dry_run": True,
            "visual_verdict_automatic": False,
        },
        "visual_pass": None,
        "visual_pass_note": (
            "VIS0-B never sets visual_pass. Execution success is a procedural "
            "statement only; a visual verdict requires a future capture backend "
            "plus human review or an approved metrics engine."
        ),
    }


# --- Git provenance ----------------------------------------------------------


def run_process(
    argv: list,
    cwd: Optional[Path] = None,
    timeout_seconds: Optional[int] = None,
) -> ProcessOutcome:
    """Run argv with no shell, capturing streams. Never raises for user errors."""
    started_at = utc_now_iso()
    monotonic_start = time.monotonic()
    try:
        completed = subprocess.run(
            list(argv),
            cwd=str(cwd) if cwd is not None else None,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout_seconds,
            shell=False,
        )
    except subprocess.TimeoutExpired as error:
        return ProcessOutcome(
            argv=list(argv),
            returncode=None,
            stdout=decode_stream(error.stdout),
            stderr=decode_stream(error.stderr),
            failure_kind="timeout",
            failure_message=(
                f"process did not exit within {timeout_seconds}s and was killed"
            ),
            started_at_utc=started_at,
            ended_at_utc=utc_now_iso(),
            duration_seconds=round(time.monotonic() - monotonic_start, 3),
        )
    except FileNotFoundError as error:
        kind, message = "executable_not_found", str(error)
    except PermissionError as error:
        kind, message = "permission_denied", str(error)
    except OSError as error:
        kind, message = "os_error", str(error)
    else:
        return ProcessOutcome(
            argv=list(argv),
            returncode=completed.returncode,
            stdout=decode_stream(completed.stdout),
            stderr=decode_stream(completed.stderr),
            started_at_utc=started_at,
            ended_at_utc=utc_now_iso(),
            duration_seconds=round(time.monotonic() - monotonic_start, 3),
        )
    return ProcessOutcome(
        argv=list(argv),
        returncode=None,
        failure_kind=kind,
        failure_message=message,
        started_at_utc=started_at,
        ended_at_utc=utc_now_iso(),
        duration_seconds=round(time.monotonic() - monotonic_start, 3),
    )


def collect_git_provenance(
    repo_root: Path, invoke: Optional[Callable[..., ProcessOutcome]] = None
) -> dict:
    """Collect repository provenance with the git CLI. Absence is not fatal."""
    if invoke is None:
        invoke = run_process
    provenance = {
        "available": False,
        "commit_sha": None,
        "commit_sha_short": None,
        "branch": None,
        "detached_head": None,
        "upstream": None,
        "remote_origin_url": None,
        "dirty": None,
        "dirty_entry_count": 0,
        "dirty_tracked_entry_count": 0,
        "dirty_entries": [],
        "errors": [],
    }

    def git(*args: str, optional: bool = False) -> Optional[str]:
        """Run a git subcommand. `optional=True` treats failure as mere absence."""
        outcome = invoke(
            ["git", *args], cwd=repo_root, timeout_seconds=GIT_TIMEOUT_SECONDS
        )
        if outcome.failure_kind is not None:
            if not optional:
                provenance["errors"].append(
                    f"git {' '.join(args)}: {outcome.failure_kind}: {outcome.failure_message}"
                )
            return None
        if outcome.returncode != 0:
            detail = (outcome.stderr or "").strip().splitlines()
            if not optional:
                provenance["errors"].append(
                    f"git {' '.join(args)}: exit {outcome.returncode}"
                    + (f": {detail[0]}" if detail else "")
                )
            return None
        return outcome.stdout

    head = git("rev-parse", "HEAD")
    if head is not None:
        commit = head.strip()
        if commit:
            provenance["available"] = True
            provenance["commit_sha"] = commit
            provenance["commit_sha_short"] = commit[:12]

    short = git("rev-parse", "--short=12", "HEAD")
    if short is not None and short.strip():
        provenance["commit_sha_short"] = short.strip()

    branch = git("branch", "--show-current")
    if branch is not None:
        name = branch.strip()
        if name:
            provenance["branch"] = name
            provenance["detached_head"] = False
        else:
            # `git branch --show-current` prints nothing on a detached HEAD.
            provenance["detached_head"] = True

    # A local branch without an upstream and a repo without an origin remote
    # are both normal; record the absence without calling it an error.
    upstream = git(
        "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}", optional=True
    )
    if upstream is not None and upstream.strip():
        provenance["upstream"] = upstream.strip()

    remote = git("remote", "get-url", "origin", optional=True)
    if remote is not None and remote.strip():
        provenance["remote_origin_url"] = remote.strip()

    status = git("status", "--porcelain")
    if status is not None:
        entries = [line for line in status.splitlines() if line.strip()]
        provenance["dirty"] = len(entries) > 0
        provenance["dirty_entries"] = entries[:200]
        provenance["dirty_entry_count"] = len(entries)
        provenance["dirty_tracked_entry_count"] = sum(
            1 for entry in entries if not entry.startswith("??")
        )

    return provenance


def enforce_git_policy(git_provenance: dict, require_clean: bool) -> None:
    if not require_clean:
        return
    if git_provenance.get("dirty") is None:
        raise RunnerError(
            "--require-clean-git cannot be satisfied: git status is unavailable "
            f"(errors: {git_provenance.get('errors') or 'git not found'})",
            EXIT_GIT_POLICY,
        )
    if git_provenance.get("dirty"):
        entries = git_provenance.get("dirty_entries") or []
        preview = ", ".join(entries[:5])
        more = f" (+{len(entries) - 5} more)" if len(entries) > 5 else ""
        raise RunnerError(
            "--require-clean-git: repository is dirty; `git status --porcelain` "
            f"reported {len(entries)} entr{'y' if len(entries) == 1 else 'ies'}: "
            f"{preview}{more}. Commit, stash or remove them, or drop the flag.",
            EXIT_GIT_POLICY,
        )


# --- App resolution ----------------------------------------------------------


def resolve_app(app: str) -> str:
    """Resolve --app to something subprocess can launch, or fail with a clear message.

    Path-like values are checked up front so the error names the real problem.
    Bare names are left to PATH resolution by the OS.
    """
    looks_like_path = (
        os.sep in app
        or "/" in app
        or "\\" in app
        or app.lower().endswith(".exe")
    )
    if not looks_like_path:
        return app

    candidate = Path(app)
    if candidate.exists():
        return str(candidate)
    if os.name == "nt" and candidate.suffix == "":
        for extension in (".exe", ".cmd", ".bat"):
            with_extension = candidate.with_suffix(extension)
            if with_extension.exists():
                return str(with_extension)
    raise RunnerError(
        f"executable not found: {app} (resolved against the current working "
        "directory; build it first, e.g. `cargo build --release -p rcsim-app`)",
        EXIT_EXECUTION_FAILED,
    )


# --- Execution ---------------------------------------------------------------


def execute_plan(
    plan: dict,
    app_resolved: str,
    timeout_seconds: int,
    invoke: Optional[Callable[..., ProcessOutcome]] = None,
) -> dict:
    """Launch the planned command and capture its streams into the scene directory."""
    if invoke is None:
        invoke = run_process
    scene_output_dir = Path(plan["scene_output_dir"])
    scene_output_dir.mkdir(parents=True, exist_ok=True)

    argv = list(plan["command_argv"])
    argv[0] = app_resolved

    stdout_path = scene_output_dir / "stdout.txt"
    stderr_path = scene_output_dir / "stderr.txt"

    outcome = invoke(argv, cwd=REPO_ROOT, timeout_seconds=timeout_seconds)

    _write_text(stdout_path, outcome.stdout)
    _write_text(stderr_path, outcome.stderr)

    return {
        "mode": "execute",
        "app": plan["command_argv"][0],
        "app_resolved": app_resolved,
        "command_argv": argv,
        "cwd": str(REPO_ROOT),
        "timeout_seconds": timeout_seconds,
        "started_at_utc": outcome.started_at_utc,
        "ended_at_utc": outcome.ended_at_utc,
        "duration_seconds": outcome.duration_seconds,
        "exit_code": outcome.returncode,
        "failure_kind": outcome.failure_kind,
        "failure_message": outcome.failure_message,
        "execution_success": outcome.succeeded,
        "stdout_path": str(stdout_path),
        "stderr_path": str(stderr_path),
        "stdout_bytes": len(outcome.stdout.encode("utf-8", errors="replace")),
        "stderr_bytes": len(outcome.stderr.encode("utf-8", errors="replace")),
        "artifacts_written": [
            str(stdout_path),
            str(stderr_path),
        ],
    }


def build_run_metadata(plan: dict, execution: dict) -> dict:
    """Assemble run.json. `visual_pass` is always null - never auto-approved."""
    return {
        "runner": plan["runner"],
        "plan_version": plan["plan_version"],
        "run_index": plan["run_index"],
        "scene_id": plan["scene_id"],
        "manifest": {
            "schema_version": plan["schema_version"],
            "scene_id": plan["scene_id"],
            "path": plan["manifest_path"],
            "path_display": plan["manifest_path_display"],
            "sha256": plan["manifest_sha256"],
        },
        "environment": plan["environment_metadata"],
        "git": plan["git"],
        "plan": plan,
        "execution": execution,
        "capabilities": plan["runtime_capabilities"],
        "artifacts": {
            "run_json": plan["artifact_paths"]["run_json"],
            "stdout": execution.get("stdout_path"),
            "stderr": execution.get("stderr_path"),
            "capture": None,
            "capture_reason": CAPTURE_UNAVAILABLE_REASON,
        },
        "verdict": {
            "execution_success": execution.get("execution_success"),
            "visual_pass": None,
            "visual_pass_reason": (
                "VIS0-B never evaluates visual quality and never sets "
                "visual_pass. A visual verdict requires a lossless capture "
                "backend plus human review or an approved metrics engine "
                "(VIS0-C or later)."
            ),
        },
    }


def _write_text(path: Path, text: str) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, indent=2, sort_keys=False, ensure_ascii=False)
        handle.write("\n")


# --- Human-readable rendering ------------------------------------------------


def _kv(label: str, value: Any, indent: int = 2) -> str:
    return " " * indent + f"{label:<22} {value}"


def format_plan_human(plan: dict) -> str:
    lines = []
    mode = "DRY-RUN (no process will be started)" if plan["dry_run"] else "EXECUTE"
    lines.append(f"{RUNNER_NAME} {RUNNER_VERSION} (plan {plan['plan_version']})")
    lines.append(f"mode: {mode}")
    lines.append("")

    lines.append("scene")
    lines.append(_kv("scene_id", plan["scene_id"]))
    lines.append(_kv("schema_version", plan["schema_version"]))
    lines.append(_kv("manifest", plan["manifest_path_display"]))
    lines.append(_kv("manifest sha256", plan["manifest_sha256"][:16] + "..."))
    lines.append("")

    lines.append("renderer / scenery / camera")
    lines.append(_kv("renderer", plan["renderer"]))
    for dotted in (
        "renderer.terrain_debug",
        "renderer.vegetation_debug",
        "scenery.preset",
    ):
        value = _find_mapping(plan, dotted)
        if value is not None:
            lines.append(_kv(dotted.split(".")[-1], value))
    camera = plan.get("camera") or {}
    lines.append(_kv("camera.mode", camera.get("mode")))
    lines.append(_kv("camera.vertical_fov_deg", camera.get("vertical_fov_deg")))
    if camera.get("mode") == "pilot":
        lines.append(_kv("camera.pilot_position", camera.get("pilot_position_render_m")))
    elif camera.get("mode") == "chase":
        lines.append(_kv("camera.chase_distance_m", camera.get("chase_distance_behind_m")))
        lines.append(_kv("camera.chase_height_m", camera.get("chase_height_above_m")))
    lines.append("")

    capabilities = plan["runtime_capabilities"]
    resolution = capabilities["resolution_enforcement"]
    requested = resolution["requested"]
    lines.append("resolution")
    lines.append(_kv("requested", f"{requested.get('width')}x{requested.get('height')}"))
    lines.append(_kv("enforcement", f"{resolution['status'].upper()} (enforced={resolution['enforced']})"))
    lines.append("")

    capture = capabilities["capture_backend"]
    basename = plan["expected_output_basename"]
    lines.append("capture")
    lines.append(_kv("backend", capture["status"].upper() + " - " + "CAPTURE BACKEND NOT YET AVAILABLE"))
    lines.append(_kv("expected filename", f"{basename['from_manifest']} (not produced)"))
    lines.append(_kv("VIS0 naming match", basename["matches_vis0_convention"]))
    lines.append("")

    lines.append("command (deterministic order, no shell)")
    for token in plan["command_argv"]:
        lines.append("    " + token)
    lines.append("")

    git = plan["git"]
    lines.append("git provenance")
    lines.append(_kv("commit", git.get("commit_sha") or "unavailable"))
    lines.append(_kv("branch", git.get("branch") or ("(detached HEAD)" if git.get("detached_head") else "unavailable")))
    lines.append(_kv("dirty", git.get("dirty")))
    lines.append(_kv("require_clean_git", plan["execution_policy"]["require_clean_git"]))
    if git.get("errors"):
        for error in git["errors"]:
            lines.append(_kv("git error", error))
    lines.append("")

    lines.append("output")
    lines.append(_kv("scene dir", plan["scene_output_dir"]))
    lines.append(_kv("run.json", plan["artifact_paths"]["run_json"]))
    lines.append(_kv("stdout.txt", plan["artifact_paths"]["stdout"]))
    lines.append(_kv("stderr.txt", plan["artifact_paths"]["stderr"]))
    if plan["dry_run"]:
        lines.append(_kv("written by dry run", "nothing (use --plan-json to persist the plan)"))
    lines.append("")

    lines.append("field mapping (manifest -> rcsim-app render CLI)")
    for mapping in plan["field_mapping"]:
        status = mapping["status"]
        name = mapping["manifest_field"]
        if status == FIELD_STATUS_CLI and mapping["emitted"]:
            detail = f"-> {mapping['runtime_flag']} {mapping['argv_value']}".rstrip()
        elif status == FIELD_STATUS_CLI:
            detail = f"x  {mapping['runtime_flag']} omitted"
        else:
            detail = "-  no runtime flag"
        lines.append(f"  [{status:<13}] {name:<34} {detail}")
    lines.append("")

    lines.append("verdict policy")
    lines.append(_kv("visual_pass", "null (never set by VIS0-B)"))
    return "\n".join(lines)


def _find_mapping(plan: dict, dotted: str) -> Any:
    for mapping in plan["field_mapping"]:
        if mapping["manifest_field"] == dotted:
            return mapping["manifest_value"]
    return None


# --- CLI ---------------------------------------------------------------------


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_benchmark.py",
        description=(
            "RV2-VIS0-B golden visual benchmark runner: validate a VIS0-A "
            "manifest, build a deterministic rcsim-app command, and record run "
            "provenance. Verifies execution reproducibility, NOT visual quality."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--manifest",
        required=True,
        metavar="PATH",
        help="path to a GoldenSceneManifest JSON (validated before anything runs)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "build and print the plan without starting a process. This is the "
            "default; the flag exists so it can override --execute."
        ),
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="opt in to really launching the app process (never the default)",
    )
    parser.add_argument(
        "--output-dir",
        metavar="PATH",
        default=None,
        help=(
            "root directory for run artifacts; defaults to "
            f"{DEFAULT_OUTPUT_DIR_RELATIVE.as_posix()} (gitignored)"
        ),
    )
    parser.add_argument(
        "--app",
        metavar="PATH_OR_COMMAND",
        default=None,
        help=(
            "rcsim-app executable, e.g. target/release/rcsim-app. Required with "
            "--execute; optional in dry run, where it is only substituted into "
            "the printed command."
        ),
    )
    parser.add_argument(
        "--timeout-seconds",
        type=int,
        default=DEFAULT_TIMEOUT_SECONDS,
        metavar="N",
        help=f"kill the app process after N seconds (default {DEFAULT_TIMEOUT_SECONDS})",
    )
    parser.add_argument(
        "--run-index",
        type=int,
        default=0,
        metavar="N",
        help="run index recorded in the plan and run.json (default 0)",
    )
    parser.add_argument(
        "--plan-json",
        metavar="PATH",
        default=None,
        help="also write the execution plan as JSON to PATH",
    )
    parser.add_argument(
        "--require-clean-git",
        action="store_true",
        help=(
            "fail with exit 3 unless `git status --porcelain` is empty. Strict: "
            "untracked entries count as dirty."
        ),
    )
    return parser


def _resolve_output_dir(raw: Optional[str]) -> Path:
    if raw is None:
        return REPO_ROOT / DEFAULT_OUTPUT_DIR_RELATIVE
    candidate = Path(raw)
    if not candidate.is_absolute():
        candidate = Path.cwd() / candidate
    if candidate.exists() and not candidate.is_dir():
        raise RunnerError(f"--output-dir is not a directory: {candidate}")
    return candidate


def main(argv: Optional[list] = None) -> int:
    configure_streams()
    parser = build_arg_parser()
    args = parser.parse_args(argv)

    try:
        return _run(args)
    except RunnerError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code
    except KeyboardInterrupt:
        print("interrupted: no visual verdict was produced", file=sys.stderr)
        return EXIT_INTERRUPTED


def _run(args: argparse.Namespace) -> int:
    if args.timeout_seconds is not None and args.timeout_seconds <= 0:
        raise RunnerError("--timeout-seconds must be a positive integer")
    if args.run_index < 0:
        raise RunnerError("--run-index must be non-negative")

    execute = bool(args.execute) and not bool(args.dry_run)
    dry_run = not execute
    if args.execute and args.dry_run:
        print(
            "note: --dry-run and --execute were both given; dry run wins "
            "(no process started)",
            file=sys.stderr,
        )

    manifest_path = Path(args.manifest)
    if not manifest_path.is_absolute():
        manifest_path = Path.cwd() / manifest_path

    # 1. Load and validate before anything else. An invalid manifest must never
    #    reach a subprocess.
    manifest = load_manifest(manifest_path)
    validation_errors = validate_manifest_dict(manifest, manifest_path.parent)
    if validation_errors:
        print(
            f"[FAIL] Invalid manifest: {manifest_path}\n"
            f"  {len(validation_errors)} error(s); no process was started:",
            file=sys.stderr,
        )
        for error in validation_errors:
            print(f"    - {error}", file=sys.stderr)
        return EXIT_VALIDATION_FAILED
    print(f"[OK] Valid manifest: {manifest_path}")

    output_dir = _resolve_output_dir(args.output_dir)

    app_resolved = None
    if args.app is not None:
        app_resolved = resolve_app(args.app) if execute else args.app
    if execute and app_resolved is None:
        raise RunnerError("--execute requires --app <path-or-command>")

    git_provenance = collect_git_provenance(REPO_ROOT)
    enforce_git_policy(git_provenance, args.require_clean_git)

    plan = build_plan(
        manifest=manifest,
        manifest_path=manifest_path,
        output_dir=output_dir,
        app=app_resolved,
        run_index=args.run_index,
        dry_run=dry_run,
        timeout_seconds=args.timeout_seconds,
        require_clean_git=bool(args.require_clean_git),
        git_provenance=git_provenance,
    )

    basename = plan["expected_output_basename"]
    if basename["matches_vis0_convention"] is False:
        print(
            "warning: capture.filename "
            f"'{basename['from_manifest']}' does not match the VIS0 golden "
            f"naming convention '{basename['per_vis0_convention']}'",
            file=sys.stderr,
        )
    if not plan["schema_version_major_supported"]:
        print(
            f"warning: manifest schema_version '{plan['schema_version']}' is not "
            "major version 1; the runner's field mapping was written for v1",
            file=sys.stderr,
        )

    if args.plan_json:
        plan_json_path = Path(args.plan_json)
        if not plan_json_path.is_absolute():
            plan_json_path = Path.cwd() / plan_json_path
        write_json(plan_json_path, plan)
        print(f"[OK] Plan written: {plan_json_path}")

    print(format_plan_human(plan))
    print("machine-readable plan (JSON):")
    print(json.dumps(plan, indent=2, ensure_ascii=False))

    if dry_run:
        print(
            "\ndry run complete: no process started, no artifacts written. "
            "Re-run with --execute --app <path> to launch rcsim-app."
        )
        return EXIT_OK

    execution = execute_plan(plan, app_resolved, args.timeout_seconds)
    run_json_path = Path(plan["artifact_paths"]["run_json"])
    execution["artifacts_written"].append(str(run_json_path))
    metadata = build_run_metadata(plan, execution)
    write_json(run_json_path, metadata)

    print(
        f"\nexecution_success: {execution['execution_success']} "
        f"(exit code {execution['exit_code']})"
    )
    print("visual_pass: null (VIS0-B never decides a visual verdict)")
    print(f"run.json: {run_json_path}")

    if execution["execution_success"]:
        return EXIT_OK
    if execution["failure_kind"] is not None:
        print(
            f"error: {execution['failure_kind']}: {execution['failure_message']}",
            file=sys.stderr,
        )
    return EXIT_EXECUTION_FAILED


if __name__ == "__main__":
    sys.exit(main())
