#!/usr/bin/env python3
"""
RV2-VIS0-B/C2B Golden Visual Benchmark Runner

Turns an approved VIS0-A GoldenSceneManifest into a deterministic, reproducible
`rcsim-app render` invocation, really runs the VIS0-C2A capture backend, and
records run provenance plus a verified VisualCaptureEvidence artifact.

VIS0 verifies EXECUTION AND CAPTURE REPRODUCIBILITY, not visual image quality.
It never decides a visual PASS/FAIL and never reads pixel values: `visual_pass`
is null in every artifact this runner writes.

Since VIS0-C2B the manifest really drives the capture:

    GoldenSceneManifest
      -> deterministic runner plan
      -> rcsim-app render --capture-frame/--capture-out/--capture-format
      -> real lossless PNG
      -> real RuntimeCaptureReceipt 1.0.0
      -> independent tooling verification (receipt + PNG bytes + PNG header)
      -> VisualCaptureEvidence 1.0.0
      -> capture_evidence.json (authoritative) and run.json (convenience copy)

Three separate contracts are in play and are never conflated:

    GoldenSceneManifest    1.1.0  intent      (requested values)
    RuntimeCaptureReceipt  1.0.0  runtime claim (written by rcsim-app)
    VisualCaptureEvidence  1.0.0  facts       (written by this runner)

The runtime receipt is the ONLY authority for `capture.actual.*` and
`capture.image.*`. Those values are never derived from the manifest, and a
receipt is never trusted merely because it exists: it is re-checked against the
request plan and the PNG is re-measured on disk.

`process exit 0` is no longer enough for a successful run. An execute is
successful only when the process exited 0 AND the receipt is trusted AND the PNG
verified independently AND the evidence document passed its own validator.

The GoldenSceneManifest contract stays authoritative: this runner reuses
`validate_manifest.ManifestValidator` and only maps manifest fields onto CLI
flags that really exist in `crates/app/src/render_app.rs`.

Usage:
    # Dry run (the default; no application process is started)
    python tools/visual_benchmark/run_benchmark.py \
        --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
        --dry-run

    # Real execution (explicit opt-in; never the default)
    python tools/visual_benchmark/run_benchmark.py \
        --manifest docs/validation/visual_benchmark/vis0_reference_scene.json \
        --app target/release/rcsim-app --execute

Exit codes:
    0 - plan built (dry run), or a fully verified end-to-end capture
    1 - manifest failed contract validation, or is formally valid but not
        runtime-executable under the C2B policy; no process was started
    2 - usage/input error (missing file, unreadable JSON, bad flag value)
    3 - git policy violation (--require-clean-git against a dirty work tree)
    4 - execution failure (app not found, non-zero exit, timeout, untrusted
        receipt, unverified PNG, or evidence that failed its own validator)
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
    from tools.visual_benchmark.runtime_capture_receipt import (
        RECEIPT_KIND,
        RECEIPT_SCHEMA_VERSION,
        RECEIPT_SUPPORTED_FORMAT,
        canonical_image_path,
        check_receipt_expectations,
        load_receipt,
        verify_captured_png,
    )
    from tools.visual_benchmark.runtime_visual_audit import (
        AUDIT_KIND,
        AUDIT_SCHEMA_VERSION,
        check_audit_expectations,
        load_runtime_visual_audit,
    )
    from tools.visual_benchmark.validate_capture_evidence import (
        EVIDENCE_KIND,
        EVIDENCE_SCHEMA_PATH,
        EVIDENCE_SCHEMA_VERSION,
        RUNTIME_SUPPLIED_FIELDS,
        TOOLING_SUPPLIED_FIELDS,
        CaptureEvidenceValidator,
    )
    from tools.visual_benchmark.validate_manifest import (
        SUPPORTED_SCHEMA_VERSION,
        ManifestValidator,
    )
except ImportError:  # Direct script execution: script directory is sys.path[0].
    from runtime_capture_receipt import (
        RECEIPT_KIND,
        RECEIPT_SCHEMA_VERSION,
        RECEIPT_SUPPORTED_FORMAT,
        canonical_image_path,
        check_receipt_expectations,
        load_receipt,
        verify_captured_png,
    )
    from runtime_visual_audit import (
        AUDIT_KIND,
        AUDIT_SCHEMA_VERSION,
        check_audit_expectations,
        load_runtime_visual_audit,
    )
    from validate_capture_evidence import (
        EVIDENCE_KIND,
        EVIDENCE_SCHEMA_PATH,
        EVIDENCE_SCHEMA_VERSION,
        RUNTIME_SUPPLIED_FIELDS,
        TOOLING_SUPPLIED_FIELDS,
        CaptureEvidenceValidator,
    )
    from validate_manifest import SUPPORTED_SCHEMA_VERSION, ManifestValidator


RUNNER_NAME = "rv2-vis0-benchmark-runner"
RUNNER_VERSION = "1.3.0"
PLAN_VERSION = "1.3.0"

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

# Artifact names inside <output_dir>/<scene_id>/. The capture image name comes
# from the manifest (capture.filename); these three are tooling-owned.
RUNTIME_RECEIPT_FILENAME = "runtime_capture_receipt.json"
RUNTIME_VISUAL_AUDIT_FILENAME = "runtime_visual_audit.json"
CAPTURE_EVIDENCE_FILENAME = "capture_evidence.json"
RUN_METADATA_FILENAME = "run.json"
STDOUT_FILENAME = "stdout.txt"
STDERR_FILENAME = "stderr.txt"

# The runtime writes each capture output to a sibling `<name>.tmp` and renames
# it only once complete (crates/app/src/render_app.rs::temporary_output_path).
# A crash between write and rename can leave that sibling behind, so the runner
# removes it too rather than letting it look like a partial result.
TEMPORARY_SUFFIX = ".tmp"

# The rcsim-app render CLI surface this runner is allowed to emit. Every entry
# was read from RenderOptions::parse_with_defaults and the usage string in
# crates/app/src/main.rs at the VIS0-C2B base commit. Nothing here is invented.
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
    "--render-width",
    "--render-height",
    "--capture-frame",
    "--capture-out",
    "--capture-format",
    "--capture-receipt-out",
    "--visual-audit-out",
    "--exit-after-frame",
)

# Flags the runner may actually emit. A subset of KNOWN_RENDER_FLAGS: the
# runner deliberately never emits --debug-overlays, --record-replay,
# --controller-profile, or the developer-only --rv2-6-validation-* gates.
# --altitude-m/--airspeed-mps became emittable in VIS0-C1B, when the v1
# manifest gained aircraft.altitude_m/aircraft.airspeed_mps so the airborne
# initial state no longer depends on the runtime defaults.
#
# The VIS0-C2A capture group (--capture-frame/--capture-out/--capture-format)
# became emittable in VIS0-C2B, when the manifest started driving a real
# capture. --capture-receipt-out and --exit-after-frame are kept separate from
# that group on purpose: they are RUNNER-DERIVED controls, not manifest fields.
# No GoldenSceneManifest key names a receipt path, so pretending one does would
# falsify the plan's provenance.
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
    "--altitude-m",
    "--airspeed-mps",
    "--start-on-ground",
    "--render-width",
    "--render-height",
    "--capture-frame",
    "--capture-out",
    "--capture-format",
    "--capture-receipt-out",
    "--visual-audit-out",
    "--exit-after-frame",
)

# Manifest-driven capture flags, in emission order.
MANIFEST_CAPTURE_FLAGS = ("--capture-out", "--capture-format", "--capture-frame")

# Runner-derived flags, in emission order. Always emitted after the
# manifest-driven ones so the argv split between "the manifest asked for this"
# and "the runner decided this" is visible in the command itself.
DERIVED_CAPTURE_FLAGS = (
    "--capture-receipt-out",
    "--visual-audit-out",
    "--exit-after-frame",
)

# --- Runtime capabilities (verified against the runtime source) --------------

CAPTURE_SUPPORTED_REASON = (
    "capture backend: supported via the VIS0-C2A capture group "
    "--capture-frame N --capture-out PATH --capture-format png "
    "[--capture-receipt-out PATH]. On the requested presentation frame the "
    "runtime redirects postprocess to an Rgba8UnormSrgb target, blits it 1:1 to "
    "the surface, reads the framebuffer back through a COPY_DST|MAP_READ staging "
    "buffer, unpads the 256-byte row alignment, presents the frame, and writes a "
    "lossless RGBA8 PNG plus an optional RuntimeCaptureReceipt. The captured "
    "pixels are the final display-referred image (post exposure and tone map), "
    "not an intermediate HDR buffer. produces_image=true."
)

CAPTURE_FRAME_SELECTION_REASON = (
    "frame selection: supported via --capture-frame N. N is a zero-based "
    "PRESENTATION frame index counted by RenderRunControl, so frames 0..N-1 are "
    "presented before frame N is captured. Zero-extent frames, occlusion and "
    "surface-acquisition failures leave the same frame pending and never advance "
    "the counter; a successfully presented frame commits exactly once."
)

CAPTURE_RECEIPT_REASON = (
    "runtime receipt: supported via --capture-receipt-out PATH. The runtime "
    "writes RuntimeCaptureReceipt 1.0.0 with schema_version, "
    "presentation_frame_index, framebuffer_width, framebuffer_height, format, "
    "image_path, image_sha256 and image_byte_size, computed over the PNG bytes "
    "it actually wrote. The receipt is written after the image, and a "
    "receipt-write failure removes the image, so a failed run cannot leave a "
    "seemingly complete pair behind. This receipt is the only runtime authority "
    "for capture.actual.* and capture.image.*."
)

RESOLUTION_SUPPORTED_REASON = (
    "resolution enforcement: supported via --render-width/--render-height CLI "
    "(VIS0-C1 runtime control). These flags request an explicit physical "
    "window/client framebuffer extent; the runtime verifies the physical inner "
    "extent fail-closed before initialising the renderer. The runner maps "
    "resolution.width -> --render-width and resolution.height -> "
    "--render-height. Enforcement is a request-side guarantee only: the "
    "authoritative capture.actual framebuffer extent comes from the runtime "
    "receipt and is verified against the PNG IHDR, never assumed to equal the "
    "requested resolution."
)

WARMUP_DERIVED_REASON = (
    "warmup frame count: derived, not a runtime flag. There is no --warmup "
    "option in rcsim-app and none is invented here. Warmup is realised by the "
    "presentation-frame capture relation warmup == capture.frame: asking for "
    "--capture-frame N means frames 0..N-1 are presented first, which is exactly "
    "N warmup presentations, and frame N is the one captured. This is why the "
    "C2B executable path requires the two manifest values to be equal; warmup is "
    "NOT supported for arbitrary warmup/capture.frame combinations."
)

AUTO_EXIT_SUPPORTED_REASON = (
    "process auto-exit: supported via --exit-after-frame CLI (VIS0-C1 runtime "
    "control). The render loop terminates cleanly after presentation frame N "
    "(zero-based) has been presented. rcsim-app rejects an exit frame earlier "
    "than the capture frame (RenderAppError::ExitBeforeCaptureFrame), so the "
    "runner derives --exit-after-frame from the same presentation frame it asks "
    "to capture: the frame is read back, presented, written to PNG and receipted, "
    "and only then does the process exit."
)

HARDWARE_METADATA_UNAVAILABLE_NOTE = (
    "operating_system, os_release and architecture are tooling-visible and "
    "reported from platform.*. gpu_adapter_name, graphics_backend and "
    "driver_version stay null: RuntimeCaptureReceipt 1.0.0 carries no adapter, "
    "backend or driver field, so there is no machine-readable handshake for "
    "them yet. Textual runtime logging is not parsed as an authority and the "
    "host GPU is never guessed, because a guessed adapter would misattribute "
    "the capture."
)

# --- C2B executability policy -------------------------------------------------

CAPTURE_FRAME_REQUIRED_REASON = (
    "C2B requires explicit capture.frame matching warmup. The manifest omits "
    "capture.frame, which GoldenSceneManifest 1.1.0 still allows, so there is no "
    "presentation frame to ask the runtime to capture; the runner will not "
    "invent one."
)

CAPTURE_FORMAT_UNSUPPORTED_TEMPLATE = (
    "C2B supports png only. capture.format is '{format}', which the VIS0-C2A "
    "runtime capture backend rejects rather than silently converts "
    "(RenderAppError::UnsupportedCaptureFormat). The manifest stays formally "
    "valid - the contract may express formats the runtime does not implement "
    "yet - but this scene is not runtime-executable today."
)

CAPTURE_QUALITY_UNSUPPORTED_TEMPLATE = (
    "capture.quality={quality} is unsupported and is not silently dropped. The "
    "VIS0-C2A backend writes lossless PNG only, which has no quality parameter, "
    "so the request cannot be honoured and the scene is not runtime-executable."
)


def warmup_mismatch_reason(warmup: Any, frame: Any) -> str:
    return (
        f"C2B requires explicit capture.frame matching warmup. The manifest asks "
        f"for warmup={warmup} but capture.frame={frame}. Because --capture-frame "
        "N presents frames 0..N-1 before capturing N, warmup is only realised by "
        "the capture frame itself; a different pair would silently change how "
        "many frames are presented before the capture, so the runner fails "
        "closed instead of guessing."
    )


# --- Capture evidence contract (VIS0-C1B, filled by VIS0-C2B) -----------------

EVIDENCE_NOT_EXECUTED_REASON = (
    "no capture was executed: this plan was built without running rcsim-app, so "
    "there is no runtime receipt and no image. Every runtime-supplied leaf stays "
    "null rather than being inferred from the manifest."
)

EVIDENCE_VISUAL_PASS_NOTE = (
    "visual_pass stays null: capture evidence records facts and never a visual "
    "verdict. A successful, byte-verified capture says an image exists and "
    "matches its receipt; it says nothing about whether the image is good. A "
    "visual PASS/FAIL needs human review or an approved metrics engine, neither "
    "of which exists here."
)

EVIDENCE_HANDSHAKE_NOTE = (
    "Producer handshake, now closed by VIS0-C2B: runtime_supplied_fields are the "
    "leaves only the runtime can state (actual framebuffer extent, actual "
    "presentation frame index, image path/digest/size, capture success, adapter "
    "metadata). They are filled from a RuntimeCaptureReceipt that this tooling "
    "parsed, matched against the request plan and re-verified against the PNG on "
    "disk - never copied from the manifest and never fabricated. "
    "tooling_supplied_fields are what this runner owns (manifest provenance, git "
    "provenance, requested values, process exit code, OS/architecture). "
    "hardware.gpu_adapter_name, hardware.graphics_backend and "
    "hardware.driver_version remain runtime-supplied leaves with no runtime "
    "source yet, so they stay null."
)

# --- Field policy ------------------------------------------------------------

FIELD_STATUS_CLI = "cli"
FIELD_STATUS_METADATA_ONLY = "metadata-only"
FIELD_STATUS_UNSUPPORTED = "unsupported"
# A field the runner turns into runtime behaviour without a flag of its own:
# it is enforced through another mechanism (the presentation-frame capture
# relation) or it constrains how a derived flag is computed. Neither "cli" nor
# "metadata-only" describes that honestly - it really controls the run.
FIELD_STATUS_DERIVED = "derived"

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
    "aircraft.altitude_m",
    "aircraft.airspeed_mps",
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
    "aircraft.altitude_m": FieldPolicy(FIELD_STATUS_CLI, "--altitude-m"),
    "aircraft.airspeed_mps": FieldPolicy(FIELD_STATUS_CLI, "--airspeed-mps"),
    "aircraft.start_on_ground": FieldPolicy(FIELD_STATUS_CLI, "--start-on-ground"),
    "resolution.width": FieldPolicy(FIELD_STATUS_CLI, "--render-width"),
    "resolution.height": FieldPolicy(FIELD_STATUS_CLI, "--render-height"),
    "warmup": FieldPolicy(FIELD_STATUS_DERIVED, reason=WARMUP_DERIVED_REASON),
    "capture.filename": FieldPolicy(
        FIELD_STATUS_CLI, "--capture-out",
        reason=(
            "capture.filename -> --capture-out: the runner resolves the filename "
            "against <output_dir>/<scene_id>/ and passes an ABSOLUTE path, so the "
            "RuntimeCaptureReceipt it gets back does not depend on the subprocess "
            "working directory to be interpreted."
        ),
    ),
    "capture.format": FieldPolicy(
        FIELD_STATUS_CLI, "--capture-format",
        reason=(
            "capture.format -> --capture-format. The VIS0-C2A backend implements "
            "png only; any other value makes the scene formally valid but not "
            "runtime-executable, and the runner fails closed before starting the "
            "process rather than letting rcsim-app reject it mid-run."
        ),
    ),
    "capture.frame": FieldPolicy(
        FIELD_STATUS_CLI, "--capture-frame",
        reason=(
            "capture.frame -> --capture-frame: the zero-based presentation frame "
            "the runtime reads back, presents and writes to PNG. The runner "
            "additionally DERIVES --exit-after-frame from this same value (see "
            "capture_plan.derived_arguments); that derived flag is process "
            "lifecycle control, not a second manifest field."
        ),
    ),
    "capture.quality": FieldPolicy(
        FIELD_STATUS_UNSUPPORTED,
        reason=(
            "capture quality: unsupported. The GoldenSceneManifest contract only "
            "allows capture.quality alongside format='jpg' (VIS0-A JPEG quality "
            "constraint), and the VIS0-C2A backend writes lossless PNG, which has "
            "no quality parameter. The value is never silently ignored: its "
            "presence makes the scene not runtime-executable and the runner says "
            "so in the plan and on stderr."
        ),
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


# --- Capture plan (VIS0-C2B) --------------------------------------------------


def build_capture_plan(manifest: dict, scene_output_dir: Path) -> dict:
    """Turn manifest capture intent into concrete paths, derived args and a gate.

    Two provenance classes are kept visibly apart:

    * `manifest_arguments` come from a real GoldenSceneManifest leaf
      (capture.filename/format/frame).
    * `derived_arguments` are computed by this runner. No manifest key names a
      receipt path, and none asks for a process lifecycle bound, so
      --capture-receipt-out and --exit-after-frame are declared as derived
      rather than being attributed to a field that does not exist.

    `executable` is deliberately distinct from "manifest valid". GoldenSceneManifest
    1.1.0 accepts an absent capture.frame and accepts jpg/exr, while the VIS0-C2A
    runtime implements neither. The schema is not narrowed here; the runner simply
    refuses to start a process it knows cannot honour the request.
    """
    capture = manifest.get("capture") or {}
    filename = capture.get("filename")
    image_format = capture.get("format")
    frame = capture.get("frame")
    quality = capture.get("quality")
    warmup = manifest.get("warmup")

    image_path = scene_output_dir / filename if filename else None
    receipt_path = scene_output_dir / RUNTIME_RECEIPT_FILENAME
    audit_path = scene_output_dir / RUNTIME_VISUAL_AUDIT_FILENAME
    evidence_path = scene_output_dir / CAPTURE_EVIDENCE_FILENAME

    blocking_reasons = []
    if not isinstance(frame, int) or isinstance(frame, bool):
        blocking_reasons.append(CAPTURE_FRAME_REQUIRED_REASON)
    elif frame != warmup:
        blocking_reasons.append(warmup_mismatch_reason(warmup, frame))

    if image_format != RECEIPT_SUPPORTED_FORMAT:
        blocking_reasons.append(
            CAPTURE_FORMAT_UNSUPPORTED_TEMPLATE.format(format=image_format)
        )
    if quality is not None:
        blocking_reasons.append(
            CAPTURE_QUALITY_UNSUPPORTED_TEMPLATE.format(quality=quality)
        )
    if image_path is None:
        blocking_reasons.append(
            "capture.filename is absent, so there is no image path to request"
        )

    executable = not blocking_reasons
    frame_text = format_number(frame) if executable else None

    manifest_arguments = []
    if executable:
        manifest_arguments = [
            {
                "flag": "--capture-out",
                "value": str(image_path),
                "manifest_field": "capture.filename",
                "provenance": "manifest",
            },
            {
                "flag": "--capture-format",
                "value": str(image_format),
                "manifest_field": "capture.format",
                "provenance": "manifest",
            },
            {
                "flag": "--capture-frame",
                "value": frame_text,
                "manifest_field": "capture.frame",
                "provenance": "manifest",
            },
        ]

    derived_arguments = []
    if executable:
        derived_arguments = [
            {
                "flag": "--capture-receipt-out",
                "value": str(receipt_path),
                "manifest_field": None,
                "provenance": "runner-derived",
                "derived_from": (
                    "tooling evidence handshake: the runner chooses the receipt "
                    "location so it can parse, trust and cross-check it. No "
                    "GoldenSceneManifest field names a receipt path."
                ),
            },
            {
                "flag": "--visual-audit-out",
                "value": str(audit_path),
                "manifest_field": None,
                "provenance": "runner-derived",
                "derived_from": (
                    "C2D runtime visual audit handshake: the runner owns the "
                    "artifact path; GoldenSceneManifest remains unchanged."
                ),
            },
            {
                "flag": "--exit-after-frame",
                "value": frame_text,
                "manifest_field": "capture.frame",
                "provenance": "runner-derived",
                "derived_from": (
                    "capture.frame: process lifecycle control equal to the "
                    "captured presentation frame. rcsim-app requires "
                    "--exit-after-frame >= --capture-frame, and the equal-frame "
                    "case captures, presents, writes the PNG and the receipt, "
                    "then exits."
                ),
            },
        ]

    return {
        "executable": executable,
        "blocking_reasons": blocking_reasons,
        "blocking_reason": blocking_reasons[0] if blocking_reasons else None,
        "format": image_format,
        "frame": frame,
        "warmup": warmup,
        "quality": quality,
        "warmup_matches_capture_frame": (
            None if not isinstance(frame, int) or isinstance(frame, bool)
            else frame == warmup
        ),
        "warmup_mechanism": (
            "derived from the presentation-frame capture relation "
            "warmup == capture.frame; there is no --warmup runtime flag"
        ),
        "scene_output_dir": str(scene_output_dir),
        "scene_output_dir_display": display_path(scene_output_dir, REPO_ROOT),
        "expected_filename": filename,
        "image_path": str(image_path) if image_path else None,
        "image_path_display": (
            display_path(image_path, REPO_ROOT) if image_path else None
        ),
        "receipt_path": str(receipt_path),
        "receipt_path_display": display_path(receipt_path, REPO_ROOT),
        "audit_path": str(audit_path),
        "audit_path_display": display_path(audit_path, REPO_ROOT),
        "evidence_path": str(evidence_path),
        "evidence_path_display": display_path(evidence_path, REPO_ROOT),
        "receipt_kind": RECEIPT_KIND,
        "receipt_schema_version": RECEIPT_SCHEMA_VERSION,
        "audit_kind": AUDIT_KIND,
        "audit_schema_version": AUDIT_SCHEMA_VERSION,
        "manifest_arguments": manifest_arguments,
        "derived_arguments": derived_arguments,
        # Absolute paths are handed to the app so the receipt's image_path echo
        # is interpretable without knowing the subprocess working directory.
        "paths_are_absolute": (
            image_path is not None and Path(str(image_path)).is_absolute()
        ),
        "stale_artifact_policy": (
            "before a real execution the runner removes a pre-existing capture "
            "image, runtime_capture_receipt.json, runtime_visual_audit.json, "
            "capture_evidence.json and their "
            "'.tmp' siblings, failing closed if any removal is refused. This "
            "covers the cases the runtime's own stale-output policy cannot: the "
            "process never starting, or crashing before its cleanup ran. Nothing "
            "outside <output_dir>/<scene_id>/ is touched, so approved baselines "
            "are never deleted."
        ),
    }


def enforce_capture_executability(capture_plan: dict) -> None:
    """Refuse to start a process for a scene the runtime cannot honour.

    Called only on the execute path. A dry run reports the same facts in the plan
    instead of raising, so it can still show why the scene is not runnable.
    """
    if capture_plan.get("executable"):
        return
    reasons = capture_plan.get("blocking_reasons") or []
    detail = "\n".join(f"    - {reason}" for reason in reasons)
    raise RunnerError(
        "manifest is formally valid but NOT runtime-executable under the "
        f"VIS0-C2B capture policy; no process was started:\n{detail}",
        EXIT_VALIDATION_FAILED,
    )


def build_field_mappings(manifest: dict, capture_plan: dict) -> list:
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
            argv_value, emitted, extra_reason = _resolve_cli_value(
                dotted, value, camera_mode, capture_plan
            )
            if extra_reason:
                reason = extra_reason
        elif policy.status == FIELD_STATUS_DERIVED and dotted == "warmup":
            if capture_plan.get("warmup_matches_capture_frame") is True:
                reason = policy.reason + " Enforced for this manifest."
            else:
                reason = (
                    policy.reason
                    + " NOT enforceable for this manifest: "
                    + (capture_plan.get("blocking_reason") or "")
                )

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


def _resolve_cli_value(dotted: str, value: Any, camera_mode: Any, capture_plan: dict):
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

    if dotted in ("capture.filename", "capture.format", "capture.frame"):
        return _resolve_capture_cli_value(dotted, value, capture_plan)

    if dotted in (
        "camera.vertical_fov_deg",
        "exposure_ev",
        "aircraft.throttle",
        "aircraft.altitude_m",
        "aircraft.airspeed_mps",
        "resolution.width",
        "resolution.height",
    ):
        return format_number(value), True, None

    raise RunnerError(f"no CLI value resolver for manifest field '{dotted}'")


def _resolve_capture_cli_value(dotted: str, value: Any, capture_plan: dict):
    """Resolve one capture leaf, or refuse to emit it for a non-executable scene.

    The capture group is all-or-nothing: rcsim-app treats --capture-frame,
    --capture-out and --capture-format as one required group and rejects a
    partial one (RenderAppError::IncompleteCaptureOptions), and it rejects a
    format it cannot write. Emitting a command the runtime is known to refuse
    would make the printed plan a lie, so a blocked scene emits no capture flag
    at all and states why.
    """
    flag = FIELD_POLICY[dotted].runtime_flag
    if not capture_plan.get("executable"):
        return (
            None,
            False,
            f"{dotted} -> {flag} not emitted: "
            + (capture_plan.get("blocking_reason") or "capture is not executable"),
        )
    if dotted == "capture.filename":
        return capture_plan["image_path"], True, None
    if dotted == "capture.format":
        return str(value), True, None
    return format_number(value), True, None


def build_command_argv(app: str, mappings: list, capture_plan: dict) -> list:
    """Build the full argv list from the resolved field mappings.

    Manifest-driven flags come first in CANONICAL_FIELD_ORDER, then the
    runner-derived capture controls, so the command itself shows the provenance
    split. Returned as a list, never a shell string, so paths with spaces stay
    safe and no shell is involved.
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
    for argument in capture_plan.get("derived_arguments") or []:
        flag = argument["flag"]
        if flag not in EMITTABLE_FLAGS:
            raise RunnerError(f"internal error: {flag} is not an emittable flag")
        argv.append(flag)
        if argument.get("value"):
            argv.append(str(argument["value"]))
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


def build_runtime_capabilities(manifest: dict, capture_plan: dict) -> dict:
    """Declare what the runtime can and cannot do for this specific manifest.

    Updated for VIS0-C2B: the VIS0-C2A capture backend really writes a PNG and a
    RuntimeCaptureReceipt, so `capture_backend` is supported and produces_image
    is true. Warmup is reported as DERIVED rather than supported: there is no
    --warmup flag, and the presentation-frame relation only realises warmup when
    warmup == capture.frame, which is a property of this manifest and not a
    general runtime capability.
    """
    resolution = manifest.get("resolution") or {}
    executable = bool(capture_plan.get("executable"))
    requested_format = capture_plan.get("format")
    format_is_png = requested_format == RECEIPT_SUPPORTED_FORMAT
    return {
        "capture_backend": {
            "status": "supported",
            "produces_image": True,
            "reason": CAPTURE_SUPPORTED_REASON,
            "final_display_referred_capture": True,
            "frame_selection": {
                "available": True,
                "mechanism": "--capture-frame N (zero-based presentation frame)",
                "reason": CAPTURE_FRAME_SELECTION_REASON,
            },
            "runtime_receipt": {
                "available": True,
                "kind": RECEIPT_KIND,
                "schema_version": RECEIPT_SCHEMA_VERSION,
                "mechanism": "--capture-receipt-out PATH (runner-derived)",
                "reason": CAPTURE_RECEIPT_REASON,
            },
            "process_auto_exit": {
                "available": True,
                "mechanism": "--exit-after-frame N (runner-derived)",
                "reason": AUTO_EXIT_SUPPORTED_REASON,
            },
            "explicit_resolution_enforcement": {
                "available": True,
                "mechanism": "--render-width/--render-height",
                "reason": RESOLUTION_SUPPORTED_REASON,
            },
            "supported_formats": [RECEIPT_SUPPORTED_FORMAT],
            "requested_format": requested_format,
            "requested_format_executable": format_is_png,
            "executable_for_this_manifest": executable,
            "expected_filename": capture_plan.get("expected_filename"),
            "expected_format": requested_format,
            # A capability is a statement about the runtime; whether this run
            # produced anything is a separate fact, reported only after execute.
            "capture_produced": None,
            "blocking_reasons": capture_plan.get("blocking_reasons") or [],
        },
        "resolution_enforcement": {
            "status": "supported",
            "enforced": True,
            "requested": {
                "width": resolution.get("width"),
                "height": resolution.get("height"),
            },
            "reason": RESOLUTION_SUPPORTED_REASON,
        },
        "warmup_frames": {
            "status": "derived",
            "enforced": bool(capture_plan.get("warmup_matches_capture_frame")),
            "requested": capture_plan.get("warmup"),
            "capture_frame": capture_plan.get("frame"),
            "mechanism": capture_plan.get("warmup_mechanism"),
            "runtime_flag": None,
            "reason": WARMUP_DERIVED_REASON,
        },
        "process_auto_exit": {
            "status": "supported",
            "reason": AUTO_EXIT_SUPPORTED_REASON,
        },
    }


def build_expected_capture_metadata(manifest: dict, capture_plan: dict) -> dict:
    """Resolve the VIS0 golden naming convention into planned paths.

    Replaces the pre-C2A `expected_capture_basename`, whose `produced: false`
    and "backend unavailable" wording became semantically false once the runtime
    really started writing PNGs. This reports what the plan intends; whether an
    artifact was verified is stated separately, after execution, in
    `capture_verification` and in the evidence document.
    """
    scene_id = manifest.get("scene_id")
    resolution = manifest.get("resolution") or {}
    width = resolution.get("width")
    height = resolution.get("height")
    image_format = capture_plan.get("format")

    convention = None
    if scene_id and isinstance(width, int) and isinstance(height, int) and image_format:
        convention = f"{scene_id}_{width}x{height}.{image_format}"
    expected_filename = capture_plan.get("expected_filename")

    return {
        "from_manifest": expected_filename,
        "per_vis0_convention": convention,
        "matches_vis0_convention": (
            None if convention is None or expected_filename is None
            else expected_filename == convention
        ),
        "format": image_format,
        "planned_path": capture_plan.get("image_path"),
        "planned_path_display": capture_plan.get("image_path_display"),
        "executable": capture_plan.get("executable"),
        "blocking_reason": capture_plan.get("blocking_reason"),
        # Filled only by a real, independently verified execute. A dry run leaves
        # it null rather than claiming false.
        "verified": None,
        "verified_path": None,
    }


# --- Capture evidence (VIS0-C1B) ---------------------------------------------


def build_capture_evidence_contract(capture_plan: dict) -> dict:
    """Describe the evidence contract, who supplies each leaf, and where it lands.

    The producer handshake is expressed as data: the runtime owns
    `runtime_supplied_fields` through the RuntimeCaptureReceipt, this tooling
    owns `tooling_supplied_fields`, and the split is the same one the evidence
    validator enforces.
    """
    return {
        "kind": EVIDENCE_KIND,
        "schema_version": EVIDENCE_SCHEMA_VERSION,
        "schema_path": EVIDENCE_SCHEMA_PATH,
        "validator": "tools/visual_benchmark/validate_capture_evidence.py",
        "artifact_path": capture_plan.get("evidence_path"),
        "artifact_path_display": capture_plan.get("evidence_path_display"),
        "authoritative_artifact": (
            "capture_evidence.json is the authoritative standalone artifact; "
            "run.json embeds the same document as a convenience copy."
        ),
        "status": "planned" if capture_plan.get("executable") else "blocked",
        "produced": False,
        "produced_note": (
            "the plan itself never writes an evidence artifact. --execute writes "
            "capture_evidence.json and embeds the same document in run.json; a "
            "dry run writes nothing and claims nothing."
        ),
        "capture_backend_available": True,
        "runtime_receipt_contract": {
            "kind": RECEIPT_KIND,
            "schema_version": RECEIPT_SCHEMA_VERSION,
            "distinct_from_evidence": (
                "RuntimeCaptureReceipt is a narrow runtime declaration written by "
                "rcsim-app; VisualCaptureEvidence is the tooling fact layer "
                "written by this runner. They are different contracts with "
                "different owners and their versions are never conflated."
            ),
            "authority": (
                "the receipt is the only runtime authority for capture.actual.* "
                "and capture.image.*; the runner never derives those leaves from "
                "the manifest."
            ),
        },
        "handshake": {
            "note": EVIDENCE_HANDSHAKE_NOTE,
            "runtime_supplied_fields": sorted(RUNTIME_SUPPLIED_FIELDS),
            "tooling_supplied_fields": sorted(TOOLING_SUPPLIED_FIELDS),
        },
        "verdict_policy": {
            "visual_pass_automatic": False,
            "note": EVIDENCE_VISUAL_PASS_NOTE,
        },
    }


def build_capture_evidence(
    plan: dict,
    execution: Optional[dict] = None,
    verification: Optional[dict] = None,
) -> dict:
    """Assemble a VisualCaptureEvidence 1.0.0 document.

    Requested and actual stay strictly separate. `capture.requested.*` comes
    from the manifest; `capture.actual.*` and `capture.image.*` come ONLY from a
    RuntimeCaptureReceipt that was parsed, matched against the request plan and
    re-verified against the PNG bytes and header on disk. Without such a trusted
    receipt every runtime-supplied leaf is null - a value is never copied from
    the request to make a failed run look complete, and a divergence between
    requested and actual is preserved as two facts rather than reconciled.
    """
    execution = execution or {}
    verification = verification or {}
    git = plan.get("git") or {}
    environment = plan.get("environment_metadata") or {}
    resolution = plan.get("resolution") or {}
    camera = plan.get("camera") or {}

    trusted = bool(verification.get("trusted"))
    receipt = verification.get("receipt") or {}
    image = verification.get("image") or {}

    if trusted:
        actual = {
            "framebuffer_width": receipt.get("framebuffer_width"),
            "framebuffer_height": receipt.get("framebuffer_height"),
            "presentation_frame_index": receipt.get("presentation_frame_index"),
        }
        image_block = {
            "path": image.get("path"),
            "sha256": image.get("sha256"),
            "byte_size": image.get("byte_size"),
        }
        capture_success = True
        failure_reason = None
    else:
        # Fail closed: no completely trusted receipt means no runtime fact at all.
        actual = {
            "framebuffer_width": None,
            "framebuffer_height": None,
            "presentation_frame_index": None,
        }
        image_block = {"path": None, "sha256": None, "byte_size": None}
        capture_success = False
        failure_reason = compose_capture_failure_reason(execution, verification)

    return {
        "schema_version": EVIDENCE_SCHEMA_VERSION,
        "scene_id": plan.get("scene_id"),
        "manifest": {
            "path": plan.get("manifest_path"),
            "path_display": plan.get("manifest_path_display"),
            "sha256": plan.get("manifest_sha256"),
        },
        "source": {
            "commit_sha": git.get("commit_sha"),
            "commit_sha_short": git.get("commit_sha_short"),
            "branch": git.get("branch"),
            "detached_head": git.get("detached_head"),
            "dirty": git.get("dirty"),
            "dirty_entry_count": git.get("dirty_entry_count"),
            "runner_name": RUNNER_NAME,
            "runner_version": RUNNER_VERSION,
        },
        "renderer": {
            "version": plan.get("renderer"),
            "exposure_ev": _find_mapping(plan, "exposure_ev"),
            "camera_mode": camera.get("mode"),
            "scenery_preset": _find_mapping(plan, "scenery.preset"),
        },
        "capture": {
            "requested": {
                "width": resolution.get("width"),
                "height": resolution.get("height"),
                "frame_index": _find_mapping(plan, "capture.frame"),
            },
            "actual": actual,
            "format": _find_mapping(plan, "capture.format"),
            "image": image_block,
        },
        "execution": {
            "capture_success": capture_success,
            "process_exit_code": execution.get("exit_code"),
            "failure_reason": failure_reason,
        },
        "hardware": {
            "operating_system": environment.get("operating_system"),
            "os_release": environment.get("os_release"),
            "architecture": environment.get("architecture"),
            # Runtime-supplied, and RuntimeCaptureReceipt 1.0.0 has no such
            # field yet: null rather than parsed out of log text or guessed.
            "gpu_adapter_name": None,
            "graphics_backend": None,
            "driver_version": None,
            "notes": HARDWARE_METADATA_UNAVAILABLE_NOTE,
        },
        "verdict": {
            "visual_pass": None,
            "visual_pass_reason": EVIDENCE_VISUAL_PASS_NOTE,
        },
    }


def compose_capture_failure_reason(execution: dict, verification: dict) -> str:
    """Build one concrete, non-empty explanation of why nothing was captured.

    The evidence contract requires `execution.failure_reason` whenever
    `capture_success` is false, and it must say what actually happened rather
    than restating a capability gap.
    """
    if not execution:
        return EVIDENCE_NOT_EXECUTED_REASON
    if not verification or not verification.get("attempted"):
        return (
            "capture was not verified: no end-to-end receipt/PNG verification was "
            f"performed for this run (process exit code {execution.get('exit_code')})"
        )

    reasons = []
    failure_kind = execution.get("failure_kind")
    if failure_kind is not None:
        reasons.append(
            f"process {failure_kind}: {execution.get('failure_message') or 'no detail'}"
        )
    exit_code = execution.get("exit_code")
    if exit_code is None and failure_kind is None:
        reasons.append("no process exit code was observed")
    elif exit_code not in (0, None):
        reasons.append(f"rcsim-app exited with code {exit_code}")

    receipt_errors = verification.get("receipt_errors") or []
    expectation_errors = verification.get("expectation_errors") or []
    image_errors = verification.get("image_errors") or []
    reasons.extend(receipt_errors)
    reasons.extend(expectation_errors)
    reasons.extend(image_errors)

    if not reasons:
        reasons.append(
            "capture was not verified end-to-end and no specific runtime error "
            "was recorded"
        )
    return "; ".join(reasons)


def validate_evidence_document(evidence: dict) -> dict:
    """Self-check an evidence document against the contract it claims to follow.

    Reported rather than raised here so the failure is recorded in run.json; the
    caller turns an invalid document into a failed run. Missing git provenance
    makes valid evidence impossible (a commit SHA is mandatory), and that is a
    fact worth recording instead of a crash.
    """
    validator = CaptureEvidenceValidator(evidence)
    valid = validator.validate()
    return {
        "valid": valid,
        "error_count": len(validator.errors),
        "errors": [str(error) for error in validator.errors],
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
    output_dir = Path(output_dir)
    if not output_dir.is_absolute():
        # Capture paths are handed to rcsim-app as absolute paths so the receipt
        # it echoes back is interpretable without knowing the subprocess cwd.
        output_dir = Path.cwd() / output_dir
    scene_id = manifest.get("scene_id")
    scene_output_dir = output_dir / scene_id
    capture_plan = build_capture_plan(manifest, scene_output_dir)
    mappings = build_field_mappings(manifest, capture_plan)
    app_token = app if app is not None else "<--app not provided>"
    argv = build_command_argv(app_token, mappings, capture_plan)

    schema_version = manifest.get("schema_version") or ""

    return {
        "plan_version": PLAN_VERSION,
        "runner": {"name": RUNNER_NAME, "version": RUNNER_VERSION},
        "run_index": run_index,
        "dry_run": dry_run,
        "execution_requested": not dry_run,
        "scene_id": scene_id,
        "schema_version": schema_version,
        "schema_version_supported": schema_version == SUPPORTED_SCHEMA_VERSION,
        "manifest_path": str(manifest_path),
        "manifest_path_display": display_path(manifest_path, REPO_ROOT),
        "manifest_sha256": sha256_of_file(manifest_path),
        "renderer": get_field(manifest, "renderer.version"),
        "resolution": manifest.get("resolution"),
        "camera": manifest.get("camera"),
        "capture_plan": capture_plan,
        "expected_output_basename": build_expected_capture_metadata(manifest, capture_plan),
        "output_dir": str(output_dir),
        "scene_output_dir": str(scene_output_dir),
        "artifact_paths": {
            "run_json": str(scene_output_dir / RUN_METADATA_FILENAME),
            "stdout": str(scene_output_dir / STDOUT_FILENAME),
            "stderr": str(scene_output_dir / STDERR_FILENAME),
            "capture_image": capture_plan["image_path"],
            "runtime_capture_receipt": capture_plan["receipt_path"],
            "runtime_visual_audit": capture_plan["audit_path"],
            "capture_evidence": capture_plan["evidence_path"],
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
        "runtime_capabilities": build_runtime_capabilities(manifest, capture_plan),
        "capture_evidence_contract": build_capture_evidence_contract(capture_plan),
        "git": git_provenance,
        "execution_policy": {
            "require_clean_git": require_clean_git,
            "shell": False,
            "default_is_dry_run": True,
            "visual_verdict_automatic": False,
            "capture_executable": bool(capture_plan["executable"]),
            "success_requires": [
                "process exit code 0",
                f"trusted {RECEIPT_KIND} {RECEIPT_SCHEMA_VERSION}",
                f"valid {AUDIT_KIND} {AUDIT_SCHEMA_VERSION} matching receipt frame/extent",
                "PNG independently verified (bytes, SHA-256, IHDR, RGBA8)",
                f"{EVIDENCE_KIND} {EVIDENCE_SCHEMA_VERSION} accepted by its validator",
            ],
            "process_exit_zero_is_sufficient": False,
        },
        "visual_pass": None,
        "visual_pass_note": (
            "VIS0 never sets visual_pass. A verified capture is a procedural "
            "statement only; a visual verdict requires human review or an "
            "approved metrics engine, and no metric of any kind is computed "
            "here."
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


def stale_artifact_paths(plan: dict) -> list:
    """Every artifact a previous run could leave inside the scene directory.

    Includes the runtime's own `<name>.tmp` siblings, which exist when a capture
    crashed between write and rename. Only paths under scene_output_dir are
    ever returned, so an approved baseline elsewhere cannot be deleted.
    """
    capture_plan = plan.get("capture_plan") or {}
    scene_output_dir = Path(plan["scene_output_dir"])
    candidates = []
    for key in ("image_path", "receipt_path", "audit_path", "evidence_path"):
        raw = capture_plan.get(key)
        if not raw:
            continue
        path = Path(raw)
        candidates.append(path)
        candidates.append(path.with_name(path.name + TEMPORARY_SUFFIX))
    # A manifest that changed capture.filename between runs must not leave the
    # previous image behind either, so any stray temporary sibling is included.
    if scene_output_dir.is_dir():
        for path in sorted(scene_output_dir.glob("*" + TEMPORARY_SUFFIX)):
            if path not in candidates:
                candidates.append(path)
    return candidates


def remove_stale_capture_artifacts(plan: dict) -> list:
    """Delete stale capture artifacts before a real execution, failing closed.

    The runtime removes its own stale image and receipt at capture startup, but
    that cleanup never runs if the process does not start or dies first. Without
    this step an image from a previous run would sit at the expected path and be
    interpretable as the result of this one. A removal that is refused (locked
    file, permission denied) stops the run instead of proceeding on a dirty
    directory.
    """
    removed = []
    scene_root = Path(plan["scene_output_dir"]).resolve()
    for path in stale_artifact_paths(plan):
        if not path.exists():
            continue
        # Never follow a path outside the scene directory, whatever the manifest
        # or a leftover artifact claims.
        if scene_root not in path.resolve().parents:
            raise RunnerError(
                f"refusing to remove '{path}': it is not inside the scene output "
                f"directory '{scene_root}'",
                EXIT_EXECUTION_FAILED,
            )
        try:
            path.unlink()
        except OSError as error:
            raise RunnerError(
                f"could not remove stale capture artifact '{path}': {error}. "
                "Refusing to execute over a directory that may still hold a "
                "previous run's image.",
                EXIT_EXECUTION_FAILED,
            )
        removed.append(str(path))
    return removed


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

    # Stale artifacts go first: if the process never starts, the expected image
    # path must be empty rather than holding a previous run's picture.
    stale_removed = remove_stale_capture_artifacts(plan)

    capture_plan = plan.get("capture_plan") or {}
    argv = list(plan["command_argv"])
    argv[0] = app_resolved

    stdout_path = scene_output_dir / STDOUT_FILENAME
    stderr_path = scene_output_dir / STDERR_FILENAME

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
        "stale_artifacts_removed": stale_removed,
        "capture_image_path": capture_plan.get("image_path"),
        "runtime_receipt_path": capture_plan.get("receipt_path"),
        "runtime_visual_audit_path": capture_plan.get("audit_path"),
        "capture_evidence_path": capture_plan.get("evidence_path"),
        "artifacts_written": [
            str(stdout_path),
            str(stderr_path),
        ],
    }


# --- Independent capture verification (VIS0-C2B) ------------------------------


def verify_capture(plan: dict, execution: dict) -> dict:
    """Verify a capture end-to-end without trusting the runtime's own claims.

    Order matters: the receipt is only read after the process exited 0, it is
    only trusted after it parses as strict RuntimeCaptureReceipt 1.0.0 AND
    matches the request plan, and the PNG is only accepted after its real byte
    size, real SHA-256 and real IHDR have been re-measured against that receipt.
    The framebuffer extent is never taken from the manifest - a requested/actual
    divergence is recorded as two facts, not repaired.
    """
    capture_plan = plan.get("capture_plan") or {}
    receipt_path = Path(capture_plan["receipt_path"]) if capture_plan.get("receipt_path") else None
    image_path = Path(capture_plan["image_path"]) if capture_plan.get("image_path") else None
    checks = []

    def record(name: str, passed: bool, detail: str = "") -> bool:
        checks.append({"check": name, "passed": bool(passed), "detail": detail})
        return bool(passed)

    exit_code = execution.get("exit_code")
    process_ok = record(
        "process_exit_code_is_zero",
        execution.get("failure_kind") is None and exit_code == 0,
        f"exit code {exit_code}"
        + (f", failure kind {execution['failure_kind']}"
           if execution.get("failure_kind") else ""),
    )

    verification = {
        "attempted": True,
        "trusted": False,
        "process_ok": process_ok,
        "receipt": None,
        "receipt_path": str(receipt_path) if receipt_path else None,
        "receipt_errors": [],
        "expectation_errors": [],
        "image": None,
        "image_path": str(image_path) if image_path else None,
        "image_errors": [],
        "checks": checks,
        "failure_reason": None,
    }

    if not process_ok:
        verification["failure_reason"] = compose_capture_failure_reason(execution, verification)
        return verification

    receipt, receipt_errors = load_receipt(receipt_path)
    verification["receipt_errors"] = receipt_errors
    receipt_ok = record(
        "runtime_receipt_parses_as_1_0_0",
        receipt is not None,
        "; ".join(receipt_errors) if receipt_errors else f"{receipt_path} parsed",
    )
    if receipt is not None:
        verification["receipt"] = receipt.to_json()

    expectation_errors = []
    if receipt is not None:
        expectation_errors = check_receipt_expectations(
            receipt,
            expected_frame_index=capture_plan.get("frame"),
            expected_format=capture_plan.get("format"),
            expected_image_path=capture_plan.get("image_path"),
            base=REPO_ROOT,
        )
    verification["expectation_errors"] = expectation_errors
    expectations_ok = record(
        "receipt_matches_request_plan",
        receipt is not None and not expectation_errors,
        "; ".join(expectation_errors) if expectation_errors
        else "presentation_frame_index, format and image_path all agree",
    )

    image = None
    image_errors = []
    if receipt is not None:
        image, image_errors = verify_captured_png(image_path, receipt)
    else:
        image_errors = [
            "capture image was not verified because no valid runtime receipt "
            "exists to check it against"
        ]
    verification["image_errors"] = image_errors
    image_ok = record(
        "png_independently_verified",
        image is not None,
        "; ".join(image_errors) if image_errors else (
            f"byte size {image.byte_size}, sha256 {image.sha256[:16]}..., "
            f"IHDR {image.width}x{image.height} bit depth {image.bit_depth} "
            f"colour type {image.color_type}"
        ),
    )
    if image is not None:
        verification["image"] = image.to_json()

    verification["trusted"] = bool(receipt_ok and expectations_ok and image_ok)
    if not verification["trusted"]:
        verification["failure_reason"] = compose_capture_failure_reason(
            execution, verification
        )
    return verification


def verify_runtime_visual_audit(
    plan: dict,
    execution: dict,
    capture_verification: dict,
) -> dict:
    """Validate the separate C2D audit and bind it to the trusted receipt."""
    capture_plan = plan.get("capture_plan") or {}
    audit_path = Path(capture_plan["audit_path"])
    checks = []

    def record(name: str, passed: bool, detail: str) -> bool:
        checks.append({"check": name, "passed": bool(passed), "detail": detail})
        return bool(passed)

    result = {
        "attempted": True,
        "valid": False,
        "path": str(audit_path),
        "audit": None,
        "parse_errors": [],
        "expectation_errors": [],
        "checks": checks,
        "failure_reason": None,
    }
    process_ok = execution.get("failure_kind") is None and execution.get("exit_code") == 0
    if not record("process_exit_code_is_zero", process_ok, f"exit code {execution.get('exit_code')}"):
        result["failure_reason"] = "process did not exit successfully; audit is not trusted"
        return result

    audit, parse_errors = load_runtime_visual_audit(audit_path)
    result["parse_errors"] = parse_errors
    parsed = record(
        "runtime_visual_audit_parses_as_1_0_0",
        audit is not None,
        "; ".join(parse_errors) if parse_errors else f"{audit_path} parsed",
    )
    if audit is not None:
        result["audit"] = audit.to_json()

    receipt = capture_verification.get("receipt") or {}
    expectation_errors = []
    if audit is not None and capture_verification.get("trusted"):
        expectation_errors = check_audit_expectations(
            audit,
            expected_frame=receipt.get("presentation_frame_index"),
            expected_width=receipt.get("framebuffer_width"),
            expected_height=receipt.get("framebuffer_height"),
            expected_renderer="v2",
        )
    elif audit is not None:
        expectation_errors = [
            "runtime capture receipt is not trusted, so audit frame/extent cannot be verified"
        ]
    result["expectation_errors"] = expectation_errors
    matches = record(
        "audit_matches_trusted_receipt_and_renderer_v2",
        audit is not None and capture_verification.get("trusted") and not expectation_errors,
        "; ".join(expectation_errors)
        if expectation_errors
        else "frame, framebuffer extent and renderer v2 agree",
    )
    result["valid"] = bool(parsed and matches)
    if not result["valid"]:
        result["failure_reason"] = "; ".join(parse_errors + expectation_errors) or (
            "runtime visual audit validation failed"
        )
    return result


def build_run_metadata(
    plan: dict,
    execution: dict,
    verification: Optional[dict] = None,
    capture_evidence: Optional[dict] = None,
    evidence_validation: Optional[dict] = None,
    audit_validation: Optional[dict] = None,
) -> dict:
    """Assemble run.json. `visual_pass` is always null - never auto-approved."""
    if capture_evidence is None:
        capture_evidence = build_capture_evidence(plan, execution, verification)
    if evidence_validation is None:
        evidence_validation = validate_evidence_document(capture_evidence)
    if audit_validation is None:
        audit_validation = {
            "attempted": False,
            "valid": False,
            "path": (plan.get("capture_plan") or {}).get("audit_path"),
            "audit": None,
            "failure_reason": "runtime visual audit was not verified",
        }
    capture_plan = plan.get("capture_plan") or {}
    capture_success = bool(
        capture_evidence.get("execution", {}).get("capture_success")
    )
    image_block = capture_evidence.get("capture", {}).get("image", {})
    # Task 21: the pre-execution plan states intent only. After an execute the
    # same metadata is restated with what was really verified, so no field reads
    # as semantically false once a capture exists.
    expected_capture = dict(plan.get("expected_output_basename") or {})
    expected_capture["verified"] = capture_success
    expected_capture["verified_path"] = (
        image_block.get("path") if capture_success else None
    )
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
        "expected_capture": expected_capture,
        "capture_verification": verification,
        "capture_evidence": capture_evidence,
        "capture_evidence_validation": evidence_validation,
        "runtime_visual_audit": audit_validation.get("audit"),
        "runtime_visual_audit_validation": audit_validation,
        "capture_evidence_artifact": {
            "path": capture_plan.get("evidence_path"),
            "path_display": capture_plan.get("evidence_path_display"),
            "written": evidence_validation.get("valid", False),
            "note": (
                "capture_evidence.json is the authoritative standalone artifact "
                "and is only written when CaptureEvidenceValidator accepts it. "
                "The copy embedded above is for convenience."
                if evidence_validation.get("valid")
                else "the evidence document failed its own validator, so no "
                "standalone artifact was published; the rejected document and "
                "its errors are recorded here instead."
            ),
        },
        "artifacts": {
            "run_json": plan["artifact_paths"]["run_json"],
            "stdout": execution.get("stdout_path"),
            "stderr": execution.get("stderr_path"),
            "capture": image_block.get("path") if capture_success else None,
            "capture_verified": capture_success,
            "capture_reason": (
                None if capture_success
                else capture_evidence.get("execution", {}).get("failure_reason")
            ),
            "runtime_capture_receipt": (
                capture_plan.get("receipt_path")
                if (verification or {}).get("receipt") else None
            ),
            "runtime_visual_audit": (
                capture_plan.get("audit_path") if audit_validation.get("valid") else None
            ),
            "capture_evidence": (
                capture_plan.get("evidence_path")
                if evidence_validation.get("valid") else None
            ),
        },
        "verdict": {
            "execution_success": execution.get("execution_success"),
            "capture_success": capture_success,
            "runner_success": bool(
                execution.get("execution_success")
                and capture_success
                and evidence_validation.get("valid")
                and audit_validation.get("valid")
            ),
            "visual_pass": None,
            "visual_pass_reason": (
                "VIS0 never evaluates visual quality and never sets visual_pass. "
                "A verified capture proves an image exists and matches its "
                "receipt; judging it requires human review or an approved "
                "metrics engine, and no metric is computed here."
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
    mode = (
        "DRY-RUN (no application process will be started)"
        if plan["dry_run"]
        else "EXECUTE"
    )
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
    capture_plan = plan["capture_plan"]
    basename = plan["expected_output_basename"]
    lines.append("capture")
    lines.append(_kv("backend", capture["status"].upper() + " (produces_image="
                     + str(capture["produces_image"]).lower() + ")"))
    lines.append(_kv("formats supported", ", ".join(capture["supported_formats"])))
    lines.append(_kv("requested format", capture["requested_format"]))
    lines.append(_kv("executable", capture_plan["executable"]))
    for reason in capture_plan["blocking_reasons"]:
        lines.append(_kv("blocked because", reason))
    lines.append(_kv("capture frame", capture_plan["frame"]))
    lines.append(_kv("warmup", capture_plan["warmup"]))
    lines.append(_kv("warmup mechanism", capture_plan["warmup_mechanism"]))
    lines.append(_kv("expected filename", basename["from_manifest"]))
    lines.append(_kv("VIS0 naming match", basename["matches_vis0_convention"]))
    lines.append(_kv("capture produced", "false (nothing was executed)"
                     if plan["dry_run"] else "see capture_verification"))
    lines.append("")

    lines.append("command (deterministic order, no shell)")
    for token in plan["command_argv"]:
        lines.append("    " + token)
    lines.append("")

    lines.append("argument provenance")
    lines.append("  manifest-driven:")
    for argument in capture_plan["manifest_arguments"]:
        lines.append(
            f"    {argument['flag']} <- {argument['manifest_field']}"
        )
    lines.append("  runner-derived (no manifest field names these):")
    for argument in capture_plan["derived_arguments"]:
        source = argument["manifest_field"] or "tooling decision"
        lines.append(f"    {argument['flag']} <- derived from {source}")
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
    lines.append(_kv("capture image", capture_plan["image_path"]))
    lines.append(_kv("runtime receipt", capture_plan["receipt_path"]))
    lines.append(_kv("runtime visual audit", capture_plan["audit_path"]))
    lines.append(_kv("capture evidence", capture_plan["evidence_path"]))
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
        elif status == FIELD_STATUS_DERIVED:
            detail = "~  enforced through another mechanism, no flag of its own"
        else:
            detail = "-  no runtime flag"
        lines.append(f"  [{status:<13}] {name:<34} {detail}")
    lines.append("")

    lines.append("success criteria (an exit code of 0 from rcsim-app is not enough)")
    for requirement in plan["execution_policy"]["success_requires"]:
        lines.append("    - " + requirement)
    lines.append("")

    lines.append("verdict policy")
    lines.append(_kv("visual_pass", "null (never set by VIS0)"))
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
            "RV2-VIS0 golden visual benchmark runner: validate a VIS0-A "
            "manifest, build a deterministic rcsim-app capture command, verify "
            "the runtime receipt and PNG independently, and record run "
            "provenance plus VisualCaptureEvidence. Verifies execution and "
            "capture reproducibility, NOT visual quality."
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
            "build and print the plan without starting the application process. "
            "This is the default; the flag exists so it can override --execute."
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
            "(no application process started)",
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
            "\ndry run complete: no application process started, no artifacts "
            "written, no image produced, no receipt claimed and no actual value "
            "inferred. Re-run with --execute --app <path> to launch rcsim-app."
        )
        return EXIT_OK

    # 2. Executability gate. A manifest can be formally valid under
    #    GoldenSceneManifest 1.1.0 and still be something the VIS0-C2A runtime
    #    cannot honour; starting a process we know will fail is not fail-closed.
    enforce_capture_executability(plan["capture_plan"])

    execution = execute_plan(plan, app_resolved, args.timeout_seconds)

    # 3. Independent verification. Process exit 0 is necessary but not enough:
    #    the receipt must parse, match the request plan, and the PNG must be
    #    re-measured on disk before any of it is believed.
    verification = verify_capture(plan, execution)
    audit_validation = verify_runtime_visual_audit(plan, execution, verification)
    evidence = build_capture_evidence(plan, execution, verification)
    evidence_validation = validate_evidence_document(evidence)

    # 4. The standalone evidence artifact is authoritative and is only published
    #    when its own validator accepts it. A rejected document is recorded in
    #    run.json with its errors instead of being written out as if valid.
    if evidence_validation["valid"]:
        evidence_path = Path(plan["capture_plan"]["evidence_path"])
        write_json(evidence_path, evidence)
        execution["artifacts_written"].append(str(evidence_path))

    run_json_path = Path(plan["artifact_paths"]["run_json"])
    execution["artifacts_written"].append(str(run_json_path))
    metadata = build_run_metadata(
        plan,
        execution,
        verification,
        evidence,
        evidence_validation,
        audit_validation,
    )
    write_json(run_json_path, metadata)

    capture_success = bool(evidence["execution"]["capture_success"])
    runner_success = bool(metadata["verdict"]["runner_success"])

    print(
        f"\nexecution_success: {execution['execution_success']} "
        f"(exit code {execution['exit_code']})"
    )
    print(f"capture_success:   {capture_success}")
    print(f"receipt_trusted:   {bool(verification['trusted'])}")
    print(f"evidence_valid:    {evidence_validation['valid']}")
    print(f"visual_audit_valid:{audit_validation['valid']}")
    print(f"runner_success:    {runner_success}")
    if capture_success:
        print(f"capture image:     {evidence['capture']['image']['path']}")
        print(f"  sha256:          {evidence['capture']['image']['sha256']}")
        print(f"  byte_size:       {evidence['capture']['image']['byte_size']}")
        print(
            "  framebuffer:     "
            f"{evidence['capture']['actual']['framebuffer_width']}x"
            f"{evidence['capture']['actual']['framebuffer_height']} "
            f"(presentation frame "
            f"{evidence['capture']['actual']['presentation_frame_index']})"
        )
        requested = evidence["capture"]["requested"]
        actual = evidence["capture"]["actual"]
        if (requested["width"], requested["height"]) != (
            actual["framebuffer_width"], actual["framebuffer_height"]
        ):
            print(
                "  note: requested and actual framebuffer extents differ; both "
                "are recorded as facts and neither was overwritten"
            )
    print(
        "capture evidence:  "
        f"{plan['capture_plan']['evidence_path']}"
        + ("" if evidence_validation["valid"] else "  (NOT PUBLISHED: rejected by its validator)")
    )
    print(f"run.json:          {run_json_path}")
    print("visual_pass:       null (VIS0 never decides a visual verdict)")

    if runner_success:
        return EXIT_OK

    if not evidence_validation["valid"]:
        print(
            "error: the produced VisualCaptureEvidence failed its own validator; "
            "no standalone evidence artifact was published:",
            file=sys.stderr,
        )
        for error in evidence_validation["errors"]:
            print(f"    - {error}", file=sys.stderr)
    if not audit_validation["valid"]:
        print(
            "error: RuntimeVisualAudit validation failed: "
            f"{audit_validation['failure_reason']}",
            file=sys.stderr,
        )
    if execution["failure_kind"] is not None:
        print(
            f"error: {execution['failure_kind']}: {execution['failure_message']}",
            file=sys.stderr,
        )
    elif not capture_success:
        print(
            "error: capture was not verified end-to-end: "
            f"{evidence['execution']['failure_reason']}",
            file=sys.stderr,
        )
    return EXIT_EXECUTION_FAILED


if __name__ == "__main__":
    sys.exit(main())
