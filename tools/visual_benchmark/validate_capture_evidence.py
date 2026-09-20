#!/usr/bin/env python3
"""
RV2-VIS0-C1B Visual Capture Evidence Validator

Validates a `VisualCaptureEvidence` artifact: the machine-readable record of
what a capture ACTUALLY produced. This is the counterpart of the VIS0-A
`GoldenSceneManifest`, which records what we ASKED for.

    GoldenSceneManifest    -> intent      (requested values)
    VisualCaptureEvidence  -> facts       (actual values)

The evidence contract records FACTS ONLY. It never decides whether an image is
good: `verdict.visual_pass` must stay null until a human review or an approved
metrics engine exists (see docs/architecture/rv2_vis0_c1b_capture_evidence.md).

NO CAPTURE BACKEND EXISTS YET. Nothing in `integration/render-v2` writes an
image, so no conforming evidence artifact can be produced today except one that
honestly reports `capture_success: false`. This validator exists first so the
future runtime capture (LINEA 1) has a stable contract to hand data to.

Unavailability policy
---------------------
`null` is the ONLY marker for "not available". A measurement the runtime cannot
report must be written as null - never as 0, -1 or an empty string, because
those read as real values - and a real value must never be invented to fill a
field the runtime cannot supply. Dimensions and byte sizes are therefore
validated as >= 1, which makes the placeholder trick impossible.

Presence policy
---------------
Every leaf of the v1 shape must be PRESENT. The only two conforming states are
"present with a real value" and "present with null"; MISSING is never accepted.
That distinction is what makes the artifact auditable: `null` means the producer
followed the contract and explicitly declared the datum unavailable, whereas an
omitted key means the artifact is incomplete - or was written by a producer that
does not implement this contract version. Omitting a field must not be a way to
say "unavailable", and omitting `verdict.visual_pass` must not be a way to leave
the verdict open. The JSON Schema enforces the same rule by listing every
property in its enclosing object's `required` array, so schema and validator
cannot disagree about presence.

Usage:
    python tools/visual_benchmark/validate_capture_evidence.py <evidence.json>
    python tools/visual_benchmark/validate_capture_evidence.py <evidence.json> \
        --manifest docs/validation/visual_benchmark/vis0_reference_scene.json

Exit codes:
    0 - Valid evidence
    1 - Invalid evidence (validation errors)
    2 - File not found, parse error or usage error
"""

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Any, Optional

try:  # Imported as part of the tools.visual_benchmark namespace package.
    from tools.visual_benchmark.validate_manifest import is_real_integer, is_real_number
except ImportError:  # Direct script execution: script directory is sys.path[0].
    from validate_manifest import is_real_integer, is_real_number


EVIDENCE_KIND = "visual_capture_evidence"
EVIDENCE_SCHEMA_VERSION = "1.0.0"
EVIDENCE_SCHEMA_PATH = "tools/visual_benchmark/visual_capture_evidence.schema.json"
SUPPORTED_SCHEMA_MAJOR = 1

SEMVER_PATTERN = r"^[0-9]+\.[0-9]+\.[0-9]+$"
SCENE_ID_PATTERN = r"^[a-z][a-z0-9_]*$"
SHA256_PATTERN = r"^[0-9a-f]{64}$"
# git object names are 40 hex chars in SHA-1 repositories and 64 in SHA-256 ones.
COMMIT_SHA_PATTERN = r"^(?:[0-9a-f]{40}|[0-9a-f]{64})$"

UNAVAILABLE_POLICY = (
    "null is the only marker for 'not available'; 0, -1 and empty strings are "
    "rejected because they read as real measurements"
)

PRESENCE_POLICY = (
    "every leaf of the v1 shape must be PRESENT. A missing field is an "
    "incomplete, non-conforming artifact; an explicit null is the producer "
    "stating that it followed the contract and the value was genuinely "
    "unavailable. The two are never equivalent: write the key with null, do "
    "not omit it."
)


class _Absent:
    """Sentinel telling 'key missing' apart from 'key present with null'.

    `dict.get(key)` collapses the two into None, which would let an incomplete
    artifact pass as an honest 'unavailable'. Call sites pass this as the
    default instead, and the `_require_*` helpers reject it explicitly.
    """

    __slots__ = ()

    def __repr__(self) -> str:
        return "<absent>"


ABSENT = _Absent()

VISUAL_PASS_LOCKED_REASON = (
    "verdict.visual_pass must stay null in VisualCaptureEvidence v1: capture "
    "evidence records facts and never a visual verdict. A visual PASS/FAIL "
    "requires human review or an approved metrics engine, neither of which "
    "exists in this repository."
)


# --- LINEA 1 handshake -------------------------------------------------------
#
# Which producer owns each leaf field. The split is the contract between the
# future runtime capture (LINEA 1) and this tooling: tooling must never
# fabricate a runtime-supplied value, and the runtime is not expected to know
# anything about manifest or git provenance.

RUNTIME_SUPPLIED_FIELDS = frozenset({
    "capture.actual.framebuffer_width",
    "capture.actual.framebuffer_height",
    "capture.actual.presentation_frame_index",
    "capture.image.path",
    "capture.image.sha256",
    "capture.image.byte_size",
    "execution.capture_success",
    "hardware.gpu_adapter_name",
    "hardware.graphics_backend",
    "hardware.driver_version",
})

TOOLING_SUPPLIED_FIELDS = frozenset({
    "schema_version",
    "scene_id",
    "manifest.path",
    "manifest.path_display",
    "manifest.sha256",
    "source.commit_sha",
    "source.commit_sha_short",
    "source.branch",
    "source.detached_head",
    "source.dirty",
    "source.dirty_entry_count",
    "source.runner_name",
    "source.runner_version",
    "renderer.version",
    "renderer.exposure_ev",
    "renderer.camera_mode",
    "renderer.scenery_preset",
    "capture.requested.width",
    "capture.requested.height",
    "capture.requested.frame_index",
    "capture.format",
    # The tooling launches the process and observes its exit status directly,
    # so this leaf needs no runtime cooperation.
    "execution.process_exit_code",
    "execution.failure_reason",
    "hardware.operating_system",
    "hardware.os_release",
    "hardware.architecture",
    "hardware.notes",
    "verdict.visual_pass",
    "verdict.visual_pass_reason",
})

ALL_LEAF_FIELDS = RUNTIME_SUPPLIED_FIELDS | TOOLING_SUPPLIED_FIELDS


class ValidationError:
    def __init__(self, path: str, message: str):
        self.path = path
        self.message = message

    def __str__(self):
        return f"{self.path}: {self.message}"


class CaptureEvidenceValidator:
    """Validates one VisualCaptureEvidence document."""

    ALLOWED_TOP_LEVEL = {
        "schema_version", "scene_id", "manifest", "source", "renderer",
        "capture", "execution", "hardware", "verdict"
    }
    ALLOWED_MANIFEST = {"path", "path_display", "sha256"}
    ALLOWED_SOURCE = {
        "commit_sha", "commit_sha_short", "branch", "detached_head", "dirty",
        "dirty_entry_count", "runner_name", "runner_version"
    }
    ALLOWED_RENDERER = {"version", "exposure_ev", "camera_mode", "scenery_preset"}
    ALLOWED_CAPTURE = {"requested", "actual", "format", "image"}
    ALLOWED_CAPTURE_REQUESTED = {"width", "height", "frame_index"}
    ALLOWED_CAPTURE_ACTUAL = {
        "framebuffer_width", "framebuffer_height", "presentation_frame_index"
    }
    ALLOWED_CAPTURE_IMAGE = {"path", "sha256", "byte_size"}
    ALLOWED_EXECUTION = {"capture_success", "process_exit_code", "failure_reason"}
    ALLOWED_HARDWARE = {
        "operating_system", "os_release", "architecture", "gpu_adapter_name",
        "graphics_backend", "driver_version", "notes"
    }
    ALLOWED_VERDICT = {"visual_pass", "visual_pass_reason"}

    RENDERER_VERSIONS = {"v1", "v2"}
    CAMERA_MODES = {"pilot", "chase"}
    SCENERY_PRESETS = {"none", "flying-field"}
    IMAGE_FORMATS = {"png", "jpg", "exr"}

    # Mirrors the VIS0-A manifest contract so the two artifacts stay aligned.
    MAXIMUM_WIDTH = 7680
    MAXIMUM_HEIGHT = 4320
    MINIMUM_WIDTH = 320
    MINIMUM_HEIGHT = 240

    def __init__(self, evidence: dict, manifest: Optional[dict] = None):
        self.evidence = evidence
        self.manifest = manifest
        self.errors: list = []

    def error(self, path: str, message: str):
        self.errors.append(ValidationError(path, message))

    def validate(self) -> bool:
        """Validate the evidence. Returns True if valid, False otherwise."""
        self._validate_unknown("", self.evidence, self.ALLOWED_TOP_LEVEL)
        self._validate_schema_version()
        self._validate_scene_id()
        self._validate_manifest_block()
        self._validate_source_block()
        self._validate_renderer_block()
        self._validate_capture_block()
        self._validate_execution_block()
        self._validate_hardware_block()
        self._validate_verdict_block()
        self._validate_capture_consistency()
        if self.manifest is not None:
            self._validate_against_manifest()
        return len(self.errors) == 0

    # --- structural helpers -------------------------------------------------

    def _validate_unknown(self, prefix: str, block: Any, allowed: set):
        """Reject unknown properties. Policy: fail closed, like the manifest."""
        if not isinstance(block, dict):
            return
        for key in block:
            if key not in allowed:
                path = f"{prefix}{key}"
                self.error(
                    path,
                    f"unknown property (allowed: {sorted(allowed)}). Unknown "
                    "fields are rejected rather than ignored so an artifact "
                    "from a newer contract cannot be silently misread.")

    def _require_object(self, path: str, value: Any) -> Optional[dict]:
        if value is ABSENT:
            self.error(path, f"required object is missing. {PRESENCE_POLICY}")
            return None
        if not isinstance(value, dict):
            got = type(value).__name__ if value is not None else "null"
            self.error(path, f"must be an object, got {got}")
            return None
        return value

    def _require_string(self, path: str, value: Any, allow_null: bool = False,
                        pattern: Optional[str] = None, label: str = "a string"):
        """Validate a string leaf. Empty/whitespace-only strings are rejected."""
        if value is ABSENT:
            self.error(path, f"required field is missing. {PRESENCE_POLICY}")
            return False
        if value is None:
            if allow_null:
                return True
            self.error(path, f"must be {label}, got null ({UNAVAILABLE_POLICY})")
            return False
        if not isinstance(value, str):
            self.error(path, f"must be {label}, got {type(value).__name__}")
            return False
        if not value.strip():
            self.error(path, f"must be a non-empty {label} ({UNAVAILABLE_POLICY})")
            return False
        if pattern is not None and not re.match(pattern, value):
            self.error(path, f"must match {pattern}, got '{value}'")
            return False
        return True

    def _require_enum(self, path: str, value: Any, allowed: set, allow_null: bool = False):
        if value is ABSENT:
            self.error(path, f"required field is missing. {PRESENCE_POLICY}")
            return False
        if value is None:
            if allow_null:
                return True
            self.error(path, f"must be one of {sorted(allowed)}, got null")
            return False
        if not isinstance(value, str):
            self.error(path, f"must be one of {sorted(allowed)}, got {type(value).__name__}")
            return False
        if value not in allowed:
            self.error(path, f"must be one of {sorted(allowed)}, got '{value}'")
            return False
        return True

    def _require_bool(self, path: str, value: Any, allow_null: bool = False):
        if value is ABSENT:
            self.error(path, f"required field is missing. {PRESENCE_POLICY}")
            return False
        if value is None:
            if allow_null:
                return True
            self.error(path, f"must be a boolean, got null ({UNAVAILABLE_POLICY})")
            return False
        if not isinstance(value, bool):
            self.error(path, f"must be a boolean, got {type(value).__name__}")
            return False
        return True

    def _require_int(self, path: str, value: Any, minimum: Optional[int],
                     maximum: Optional[int] = None, allow_null: bool = False):
        """Validate an integer leaf with an inclusive lower bound.

        `minimum` is what makes a placeholder impossible: dimensions and byte
        sizes start at 1, so 0 cannot be smuggled in as "unavailable". Pass
        None for no lower bound (an exit code may legitimately be negative).
        """
        if value is ABSENT:
            self.error(path, f"required field is missing. {PRESENCE_POLICY}")
            return False
        if value is None:
            if allow_null:
                return True
            self.error(path, f"must be an integer, got null ({UNAVAILABLE_POLICY})")
            return False
        if not is_real_integer(value):
            self.error(path, f"must be an integer, got {type(value).__name__}")
            return False
        if minimum is not None and value < minimum:
            self.error(path, f"must be >= {minimum}, got {value} ({UNAVAILABLE_POLICY})")
            return False
        if maximum is not None and value > maximum:
            self.error(path, f"must be <= {maximum}, got {value}")
            return False
        return True

    def _require_number(self, path: str, value: Any, allow_null: bool = False):
        if value is ABSENT:
            self.error(path, f"required field is missing. {PRESENCE_POLICY}")
            return False
        if value is None:
            if allow_null:
                return True
            self.error(path, f"must be a number, got null ({UNAVAILABLE_POLICY})")
            return False
        if not is_real_number(value):
            self.error(path, f"must be a number, got {type(value).__name__}")
            return False
        if math.isnan(value):
            self.error(path, "must be finite (got NaN)")
            return False
        if math.isinf(value):
            self.error(path, "must be finite (got Infinity)")
            return False
        return True

    # --- leaf blocks --------------------------------------------------------

    def _validate_schema_version(self):
        version = self.evidence.get("schema_version", ABSENT)
        if not self._require_string("schema_version", version,
                                    pattern=SEMVER_PATTERN,
                                    label="a semantic version (X.Y.Z)"):
            return
        major = version.split(".")[0]
        if major != str(SUPPORTED_SCHEMA_MAJOR):
            self.error(
                "schema_version",
                f"major version {major} is not supported by this validator "
                f"(supports major {SUPPORTED_SCHEMA_MAJOR})")

    def _validate_scene_id(self):
        scene_id = self.evidence.get("scene_id", ABSENT)
        if not self._require_string("scene_id", scene_id, pattern=SCENE_ID_PATTERN,
                                    label="a scene id"):
            return
        if len(scene_id) < 3:
            self.error("scene_id", f"must be at least 3 characters, got {len(scene_id)}")
        if len(scene_id) > 128:
            self.error("scene_id", f"must be at most 128 characters, got {len(scene_id)}")

    def _validate_manifest_block(self):
        block = self._require_object("manifest", self.evidence.get("manifest", ABSENT))
        if block is None:
            return
        self._validate_unknown("manifest.", block, self.ALLOWED_MANIFEST)
        # Provenance is the point of the artifact, so the digest is mandatory.
        self._require_string("manifest.sha256", block.get("sha256", ABSENT),
                             pattern=SHA256_PATTERN,
                             label="a lowercase hex SHA-256 of the manifest")
        self._require_string("manifest.path", block.get("path", ABSENT), allow_null=True)
        self._require_string("manifest.path_display", block.get("path_display", ABSENT),
                             allow_null=True)

    def _validate_source_block(self):
        block = self._require_object("source", self.evidence.get("source", ABSENT))
        if block is None:
            return
        self._validate_unknown("source.", block, self.ALLOWED_SOURCE)
        self._require_string("source.commit_sha", block.get("commit_sha", ABSENT),
                             pattern=COMMIT_SHA_PATTERN,
                             label="a lowercase hex git commit SHA")
        self._require_string("source.commit_sha_short",
                             block.get("commit_sha_short", ABSENT), allow_null=True)
        # branch is legitimately null on a detached HEAD, but the key must exist.
        self._require_string("source.branch", block.get("branch", ABSENT), allow_null=True)
        self._require_bool("source.detached_head", block.get("detached_head", ABSENT),
                           allow_null=True)
        self._require_bool("source.dirty", block.get("dirty", ABSENT), allow_null=True)
        self._require_int("source.dirty_entry_count",
                          block.get("dirty_entry_count", ABSENT),
                          minimum=0, allow_null=True)
        self._require_string("source.runner_name", block.get("runner_name", ABSENT))
        self._require_string("source.runner_version", block.get("runner_version", ABSENT),
                             pattern=SEMVER_PATTERN,
                             label="a semantic version (X.Y.Z)")

    def _validate_renderer_block(self):
        block = self._require_object("renderer", self.evidence.get("renderer", ABSENT))
        if block is None:
            return
        self._validate_unknown("renderer.", block, self.ALLOWED_RENDERER)
        self._require_enum("renderer.version", block.get("version", ABSENT),
                           self.RENDERER_VERSIONS, allow_null=True)
        self._require_number("renderer.exposure_ev", block.get("exposure_ev", ABSENT),
                             allow_null=True)
        self._require_enum("renderer.camera_mode", block.get("camera_mode", ABSENT),
                           self.CAMERA_MODES, allow_null=True)
        self._require_enum("renderer.scenery_preset", block.get("scenery_preset", ABSENT),
                           self.SCENERY_PRESETS, allow_null=True)

    def _validate_capture_block(self):
        block = self._require_object("capture", self.evidence.get("capture", ABSENT))
        if block is None:
            return
        self._validate_unknown("capture.", block, self.ALLOWED_CAPTURE)

        requested = self._require_object("capture.requested", block.get("requested", ABSENT))
        if requested is not None:
            self._validate_unknown("capture.requested.", requested,
                                   self.ALLOWED_CAPTURE_REQUESTED)
            # Requested values come from the manifest, so they are mandatory.
            self._require_int("capture.requested.width",
                              requested.get("width", ABSENT),
                              minimum=self.MINIMUM_WIDTH, maximum=self.MAXIMUM_WIDTH)
            self._require_int("capture.requested.height",
                              requested.get("height", ABSENT),
                              minimum=self.MINIMUM_HEIGHT, maximum=self.MAXIMUM_HEIGHT)
            self._require_int("capture.requested.frame_index",
                              requested.get("frame_index", ABSENT),
                              minimum=0, allow_null=True)

        # Actual values are runtime-supplied and stay null until a capture
        # backend exists. They are kept in a separate object from `requested`
        # so a divergence can never be overwritten or confused with intent.
        actual = self._require_object("capture.actual", block.get("actual", ABSENT))
        if actual is not None:
            self._validate_unknown("capture.actual.", actual,
                                   self.ALLOWED_CAPTURE_ACTUAL)
            self._require_int("capture.actual.framebuffer_width",
                              actual.get("framebuffer_width", ABSENT), minimum=1,
                              allow_null=True)
            self._require_int("capture.actual.framebuffer_height",
                              actual.get("framebuffer_height", ABSENT), minimum=1,
                              allow_null=True)
            self._require_int("capture.actual.presentation_frame_index",
                              actual.get("presentation_frame_index", ABSENT), minimum=0,
                              allow_null=True)

        self._require_enum("capture.format", block.get("format", ABSENT),
                           self.IMAGE_FORMATS)

        image = self._require_object("capture.image", block.get("image", ABSENT))
        if image is not None:
            self._validate_unknown("capture.image.", image, self.ALLOWED_CAPTURE_IMAGE)
            self._require_string("capture.image.path", image.get("path", ABSENT),
                                 allow_null=True)
            self._require_string("capture.image.sha256", image.get("sha256", ABSENT),
                                 allow_null=True, pattern=SHA256_PATTERN,
                                 label="a lowercase hex SHA-256 of the image")
            self._require_int("capture.image.byte_size", image.get("byte_size", ABSENT),
                              minimum=1, allow_null=True)

    def _validate_execution_block(self):
        block = self._require_object("execution", self.evidence.get("execution", ABSENT))
        if block is None:
            return
        self._validate_unknown("execution.", block, self.ALLOWED_EXECUTION)
        self._require_bool("execution.capture_success",
                           block.get("capture_success", ABSENT))
        # Null when no process ran at all; the key must still be present. An
        # exit code may legitimately be negative, so there is no lower bound.
        self._require_int("execution.process_exit_code",
                          block.get("process_exit_code", ABSENT),
                          minimum=None, allow_null=True)
        self._require_string("execution.failure_reason",
                             block.get("failure_reason", ABSENT), allow_null=True)

    def _validate_hardware_block(self):
        block = self._require_object("hardware", self.evidence.get("hardware", ABSENT))
        if block is None:
            return
        self._validate_unknown("hardware.", block, self.ALLOWED_HARDWARE)
        # Every hardware leaf may be null - an unavailable adapter, backend or
        # driver must be reported as null, never guessed and never omitted.
        for field in ("operating_system", "os_release", "architecture",
                      "gpu_adapter_name", "graphics_backend", "driver_version",
                      "notes"):
            self._require_string(f"hardware.{field}", block.get(field, ABSENT),
                                 allow_null=True)

    def _validate_verdict_block(self):
        block = self._require_object("verdict", self.evidence.get("verdict", ABSENT))
        if block is None:
            return
        self._validate_unknown("verdict.", block, self.ALLOWED_VERDICT)
        visual_pass = block.get("visual_pass", ABSENT)
        if visual_pass is ABSENT:
            self.error("verdict.visual_pass",
                       f"required field is missing. {PRESENCE_POLICY} "
                       "Omitting the verdict is not a way to leave it open: "
                       "the key must be present with the value null.")
        elif visual_pass is not None:
            self.error("verdict.visual_pass", VISUAL_PASS_LOCKED_REASON)
        self._require_string("verdict.visual_pass_reason",
                             block.get("visual_pass_reason", ABSENT), allow_null=True)

    # --- cross-field invariants ---------------------------------------------

    def _validate_capture_consistency(self):
        """Tie the image facts to `execution.capture_success`.

        A successful capture must be verifiable (path, digest, size and the real
        framebuffer extent); a failed one must not carry an image, otherwise the
        artifact would advertise a picture that was never written.
        """
        execution = self.evidence.get("execution")
        capture = self.evidence.get("capture")
        if not isinstance(execution, dict) or not isinstance(capture, dict):
            return
        success = execution.get("capture_success")
        if not isinstance(success, bool):
            return

        image = capture.get("image") if isinstance(capture.get("image"), dict) else {}
        actual = capture.get("actual") if isinstance(capture.get("actual"), dict) else {}

        if success:
            for field in ("path", "sha256", "byte_size"):
                if image.get(field) is None:
                    self.error(
                        f"capture.image.{field}",
                        "required when execution.capture_success is true: a "
                        "declared capture must be verifiable")
            for field in ("framebuffer_width", "framebuffer_height",
                          "presentation_frame_index"):
                if actual.get(field) is None:
                    self.error(
                        f"capture.actual.{field}",
                        "required when execution.capture_success is true: the "
                        "runtime must report what it really produced")
            exit_code = execution.get("process_exit_code")
            if is_real_integer(exit_code) and exit_code != 0:
                self.error(
                    "execution.process_exit_code",
                    f"cannot be {exit_code} when execution.capture_success is true")
            if execution.get("failure_reason") is not None:
                self.error(
                    "execution.failure_reason",
                    "must be null when execution.capture_success is true")
        else:
            for field in ("path", "sha256", "byte_size"):
                if image.get(field) is not None:
                    self.error(
                        f"capture.image.{field}",
                        "must be null when execution.capture_success is false: "
                        "a failed capture must not advertise an image")
            if execution.get("failure_reason") is None:
                self.error(
                    "execution.failure_reason",
                    "required when execution.capture_success is false: state why "
                    "nothing was captured instead of leaving it implicit")

    def _validate_against_manifest(self):
        """Cross-check the recorded facts against the manifest that was run.

        Only applies when a manifest is supplied. This is where a scene_id
        mismatch or a stale manifest digest is caught.
        """
        manifest = self.manifest
        if not isinstance(manifest, dict):
            self.error("manifest", "cross-check requested but the manifest is not an object")
            return

        scene_id = manifest.get("scene_id")
        if isinstance(scene_id, str) and self.evidence.get("scene_id") != scene_id:
            self.error(
                "scene_id",
                f"does not match the manifest scene_id '{scene_id}': the "
                "evidence describes a different scene than the one supplied")

        resolution = manifest.get("resolution") if isinstance(manifest.get("resolution"), dict) else {}
        camera = manifest.get("camera") if isinstance(manifest.get("camera"), dict) else {}
        scenery = manifest.get("scenery") if isinstance(manifest.get("scenery"), dict) else {}
        renderer = manifest.get("renderer") if isinstance(manifest.get("renderer"), dict) else {}
        capture = manifest.get("capture") if isinstance(manifest.get("capture"), dict) else {}

        evidence_capture = self.evidence.get("capture")
        requested = {}
        evidence_format = None
        if isinstance(evidence_capture, dict):
            if isinstance(evidence_capture.get("requested"), dict):
                requested = evidence_capture["requested"]
            evidence_format = evidence_capture.get("format")

        pairs = (
            ("capture.requested.width", requested.get("width"), resolution.get("width")),
            ("capture.requested.height", requested.get("height"), resolution.get("height")),
            ("capture.requested.frame_index", requested.get("frame_index"), capture.get("frame")),
            ("capture.format", evidence_format, capture.get("format")),
        )
        for path, evidence_value, manifest_value in pairs:
            if manifest_value is None or evidence_value is None:
                continue
            if evidence_value != manifest_value:
                self.error(
                    path,
                    f"records {evidence_value!r} but the supplied manifest says "
                    f"{manifest_value!r}")

        evidence_renderer = self.evidence.get("renderer")
        if isinstance(evidence_renderer, dict):
            renderer_pairs = (
                ("renderer.version", evidence_renderer.get("version"), renderer.get("version")),
                ("renderer.camera_mode", evidence_renderer.get("camera_mode"), camera.get("mode")),
                ("renderer.scenery_preset",
                 evidence_renderer.get("scenery_preset"), scenery.get("preset")),
                ("renderer.exposure_ev",
                 evidence_renderer.get("exposure_ev"), manifest.get("exposure_ev")),
            )
            for path, evidence_value, manifest_value in renderer_pairs:
                if manifest_value is None or evidence_value is None:
                    continue
                if evidence_value != manifest_value:
                    self.error(
                        path,
                        f"records {evidence_value!r} but the supplied manifest "
                        f"says {manifest_value!r}")


def sha256_of_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json_object(path: Path, label: str):
    """Load a JSON document that must be an object. Returns (data, exit_code)."""
    if not path.exists():
        print(f"ERROR: {label} file not found: {path}", file=sys.stderr)
        return None, 2
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except json.JSONDecodeError as exc:
        print(f"ERROR: {label} is not valid JSON: {exc}", file=sys.stderr)
        return None, 2
    except OSError as exc:
        print(f"ERROR: {label} could not be read: {exc}", file=sys.stderr)
        return None, 2
    if not isinstance(data, dict):
        print(f"ERROR: {label} root must be an object, got {type(data).__name__}",
              file=sys.stderr)
        return None, 1
    return data, 0


def validate_capture_evidence(evidence_path: Path,
                              manifest_path: Optional[Path] = None) -> int:
    """Validate an evidence artifact. Returns the process exit code."""
    evidence, exit_code = load_json_object(evidence_path, "evidence")
    if evidence is None:
        return exit_code

    manifest = None
    if manifest_path is not None:
        manifest, exit_code = load_json_object(manifest_path, "manifest")
        if manifest is None:
            return exit_code
        recorded = evidence.get("manifest")
        recorded_sha = recorded.get("sha256") if isinstance(recorded, dict) else None
        actual_sha = sha256_of_file(manifest_path)
        if isinstance(recorded_sha, str) and recorded_sha != actual_sha:
            print(f"[INVALID] capture evidence: {evidence_path}")
            print("  1 error(s):")
            print(f"    - manifest.sha256: records '{recorded_sha}' but the "
                  f"supplied manifest hashes to '{actual_sha}'; the evidence "
                  "was not produced from this manifest")
            return 1

    validator = CaptureEvidenceValidator(evidence, manifest)
    if validator.validate():
        print(f"[OK] valid capture evidence: {evidence_path}")
        return 0

    print(f"[INVALID] capture evidence: {evidence_path}")
    print(f"  {len(validator.errors)} error(s):")
    for error in validator.errors:
        print(f"    - {error}")
    return 1


def configure_streams() -> None:
    """Stay lossless instead of crashing on a legacy console codepage."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is None:
            continue
        try:
            reconfigure(errors="replace")
        except (ValueError, OSError):
            pass


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="validate_capture_evidence.py",
        description="Validate a VisualCaptureEvidence artifact (facts, not a "
                    "visual verdict).")
    parser.add_argument("evidence", help="path to the capture evidence JSON")
    parser.add_argument(
        "--manifest",
        default=None,
        help="optional GoldenSceneManifest to cross-check scene_id, the "
             "recorded manifest SHA-256 and every requested value against")
    return parser


def main(argv: Optional[list] = None) -> int:
    configure_streams()
    args = build_arg_parser().parse_args(argv)
    manifest_path = Path(args.manifest) if args.manifest else None
    return validate_capture_evidence(Path(args.evidence), manifest_path)


if __name__ == "__main__":
    sys.exit(main())
