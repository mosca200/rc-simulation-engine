#!/usr/bin/env python3
"""
Tests for the RV2-VIS0-B Golden Visual Benchmark Runner.

Run with:
    python -m unittest tools/visual_benchmark/test_run_benchmark.py -v

No graphical process is ever started. Real subprocess coverage uses the Python
interpreter itself as a harmless stand-in executable, so execution, non-zero
exit and timeout paths are exercised for real without a GPU or a window.
"""

import ast
import contextlib
import hashlib
import io
import json
import re
import shlex
import struct
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

from tools.visual_benchmark import run_benchmark as rb
from tools.visual_benchmark import runtime_capture_receipt as rcr
from tools.visual_benchmark import runtime_visual_audit as rva
from tools.visual_benchmark.validate_capture_evidence import (
    ALL_LEAF_FIELDS,
    RUNTIME_SUPPLIED_FIELDS,
    validate_capture_evidence,
)
from tools.visual_benchmark.validate_manifest import SUPPORTED_SCHEMA_VERSION


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
RUNNER_SOURCE = Path(rb.__file__)
REFERENCE_MANIFEST = (
    REPO_ROOT / "docs" / "validation" / "visual_benchmark" / "vis0_reference_scene.json"
)
RENDER_APP_SOURCE = REPO_ROOT / "crates" / "app" / "src" / "render_app.rs"
APP_MAIN_SOURCE = REPO_ROOT / "crates" / "app" / "src" / "main.rs"

FAKE_GIT = {
    "available": True,
    "commit_sha": "a" * 40,
    "commit_sha_short": "a" * 12,
    "branch": "feature/test",
    "detached_head": False,
    "upstream": None,
    "remote_origin_url": None,
    "dirty": False,
    "dirty_entry_count": 0,
    "dirty_tracked_entry_count": 0,
    "dirty_entries": [],
    "errors": [],
}


def base_manifest() -> dict:
    """A valid manifest aligned with the real runtime CLI."""
    return {
        "schema_version": SUPPORTED_SCHEMA_VERSION,
        "scene_id": "test_scene",
        "description": "Test scene for benchmark runner coverage",
        "renderer": {"version": "v2", "terrain_debug": "final", "vegetation_debug": "final"},
        "scenery": {"preset": "flying-field"},
        "camera": {
            "mode": "pilot",
            "vertical_fov_deg": 55,
            "pilot_position_render_m": [0.0, 1.8, 20.0],
        },
        "resolution": {"width": 1920, "height": 1080},
        "exposure_ev": 0,
        "aircraft": {
            "model": "models/acro_electric_01/model.json",
            "throttle": 0.0,
            "start_on_ground": True,
        },
        "warmup": 10,
        "capture": {"filename": "test_scene_1920x1080.png", "format": "png", "frame": 10},
        "tags": ["test"],
    }


def airborne_manifest() -> dict:
    """base_manifest() starts on the ground; this exercises the airborne branch.

    The airborne branch of RenderApplication::new is the one that reads
    altitude_m/airspeed_mps, and the VIS0-C1B contract requires both.
    """
    manifest = base_manifest()
    manifest["scene_id"] = "test_scene_airborne"
    manifest["aircraft"] = {
        "model": "models/acro_electric_01/model.json",
        "throttle": 0.55,
        "altitude_m": 100.0,
        "airspeed_mps": 18.0,
    }
    manifest["capture"]["filename"] = "test_scene_airborne_1920x1080.png"
    return manifest


def small_manifest(width: int = 320, height: int = 240, frame=10, warmup=10,
                   image_format: str = "png", filename=None, quality=None) -> dict:
    """base_manifest() at the smallest extent the VIS0-A contract allows.

    320x240 is MINIMUM_WIDTH x MINIMUM_HEIGHT, so real end-to-end tests stay
    fast without asking the manifest validator for something it must reject.
    `frame=None` omits capture.frame entirely, which GoldenSceneManifest 1.1.0
    still permits and the C2B executability policy does not.
    """
    manifest = base_manifest()
    manifest["resolution"] = {"width": width, "height": height}
    manifest["warmup"] = warmup
    capture = {
        "filename": filename or f"test_scene_{width}x{height}.{image_format}",
        "format": image_format,
    }
    if frame is not None:
        capture["frame"] = frame
    if quality is not None:
        capture["quality"] = quality
    manifest["capture"] = capture
    return manifest


# --- Real PNG and receipt fixtures -------------------------------------------
#
# Built with the standard library only (zlib + struct): a PNG header is 33 bytes
# of fixed layout, so no image library is needed to produce or to read one. These
# helpers exist so the tests exercise the verification code against REAL bytes
# rather than mocks that agree with whatever the code happens to do.


def png_chunk(tag: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def make_png(width: int, height: int, color=(10, 20, 30, 255),
             bit_depth: int = 8, color_type: int = 6) -> bytes:
    """Encode a real, decodable RGBA8 PNG of the given extent."""
    ihdr = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    channels = {6: 4, 2: 3, 0: 1}.get(color_type, 4)
    row = b"\x00" + bytes(color[:channels]) * width
    idat = zlib.compress(row * height, 9)
    return (b"\x89PNG\r\n\x1a\n" + png_chunk(b"IHDR", ihdr)
            + png_chunk(b"IDAT", idat) + png_chunk(b"IEND", b""))


def receipt_for(image_path, png_bytes: bytes, width: int, height: int,
                frame: int = 10, image_format: str = "png",
                schema_version: str = "1.0.0") -> dict:
    """A byte-accurate RuntimeCaptureReceipt 1.0.0 for a real PNG."""
    return {
        "schema_version": schema_version,
        "presentation_frame_index": frame,
        "framebuffer_width": width,
        "framebuffer_height": height,
        "format": image_format,
        "image_path": str(image_path),
        "image_sha256": hashlib.sha256(png_bytes).hexdigest(),
        "image_byte_size": len(png_bytes),
    }


def audit_for(frame: int = 10, width: int = 320, height: int = 240) -> dict:
    passes = [{
        "pass_id": pass_id,
        "label": pass_id,
        "cpu_duration_ns": 1,
        "gpu_duration_ns": None,
    } for pass_id in rva.PASS_IDS]
    return {
        "schema_version": "1.0.0",
        "identity": {
            "presentation_frame_index": frame,
            "framebuffer_width": width,
            "framebuffer_height": height,
            "renderer_version": "v2",
        },
        "device": {
            "adapter_name": "NVIDIA GeForce RTX 3090",
            "backend": "vulkan",
            "driver": "NVIDIA",
            "driver_info": "test",
        },
        "environment": {
            "environment_mode": "physical",
            "physical_atmosphere_active": True,
            "physical_ibl_active": True,
            "aerial_perspective_active": True,
        },
        "image_pipeline": {
            "hdr_scene_format": "Rgba16Float",
            "exposure_ev": 0.0,
            "tone_mapper": "Khronos PBR Neutral",
            "temporal_resolve_active": True,
        },
        "shadows": {
            "path_active": True,
            "cascade_count": 3,
            "map_resolution": 2048,
            "split_distances_m": [32.0, 128.0, 512.0],
            "filtering": "PCF",
            "filter_tap_count": None,
            "filter_tap_count_unavailable_reason": "not runtime state",
        },
        "terrain": {
            "render_mode": "flat",
            "debug_mode": "final",
            "material_path_active": True,
            "sampler_anisotropy": 16,
            "material_scale": None,
            "material_scale_unavailable_reason": "multi-frequency material",
        },
        "vegetation": {
            "vegetation_present": True,
            "debug_mode": "final",
            "stats": {
                "total": 10,
                "visible": 6,
                "culled_frustum": 3,
                "culled_distance": 1,
                "lod_counts": [1, 2, 3],
                "scene_draw_calls": 4,
                "shadow_draw_calls": 9,
                "uploaded_instance_bytes": 288,
            },
        },
        "profiling": {
            "presentation_frame_index": frame,
            "cpu_frame_duration_ns": 100,
            "gpu_timing_status": "timestamp_query_unsupported",
            "gpu_timing_source_presentation_frame_index": None,
            "gpu_timing_frame_age": None,
            "gpu_timing_unavailable_reason": "timestamp queries unsupported",
            "passes": passes,
        },
    }


# A stand-in for `rcsim-app render` that reproduces the VIS0-C2A artifact
# contract: it writes a real RGBA8 PNG to --capture-out and a byte-accurate
# RuntimeCaptureReceipt to --capture-receipt-out. Behaviour knobs are baked in
# by textual substitution so no environment plumbing is needed, and the defaults
# mirror the real runtime (extent from --render-width/--render-height, frame from
# --capture-frame). It is a test double for the ARTIFACT contract only; nothing
# here renders anything.
FAKE_RUNTIME_SOURCE = '''\
import hashlib
import json
import struct
import sys
import zlib

WIDTH = __WIDTH__
HEIGHT = __HEIGHT__
FRAME = __FRAME__
RECEIPT_FRAME = __RECEIPT_FRAME__
RECEIPT_WIDTH = __RECEIPT_WIDTH__
RECEIPT_HEIGHT = __RECEIPT_HEIGHT__
RECEIPT_IMAGE_PATH = __RECEIPT_IMAGE_PATH__
RECEIPT_SHA = __RECEIPT_SHA__
RECEIPT_SIZE = __RECEIPT_SIZE__
EXIT_CODE = __EXIT_CODE__
SKIP_RECEIPT = __SKIP_RECEIPT__
SKIP_AUDIT = __SKIP_AUDIT__
CORRUPT_PNG = __CORRUPT_PNG__


def option(name):
    argv = sys.argv[1:]
    if name in argv:
        index = argv.index(name)
        if index + 1 < len(argv):
            return argv[index + 1]
    return None


def chunk(tag, data):
    return (struct.pack(">I", len(data)) + tag + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))


def build_png(width, height):
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    row = b"\\x00" + bytes((10, 20, 30, 255)) * width
    return (b"\\x89PNG\\r\\n\\x1a\\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(row * height, 9))
            + chunk(b"IEND", b""))


def main():
    out = option("--capture-out")
    receipt_out = option("--capture-receipt-out")
    audit_out = option("--visual-audit-out")
    if out is None:
        print("fake runtime: no --capture-out", file=sys.stderr)
        return 2
    width = WIDTH if WIDTH is not None else int(option("--render-width") or 320)
    height = HEIGHT if HEIGHT is not None else int(option("--render-height") or 240)
    frame = FRAME if FRAME is not None else int(option("--capture-frame") or 0)
    png = build_png(width, height)
    if CORRUPT_PNG:
        png = b"\\x00" * 8 + png[8:]
    with open(out, "wb") as handle:
        handle.write(png)
    if SKIP_RECEIPT or receipt_out is None:
        return EXIT_CODE
    digest = RECEIPT_SHA if RECEIPT_SHA is not None else hashlib.sha256(png).hexdigest()
    size = RECEIPT_SIZE if RECEIPT_SIZE is not None else len(png)
    receipt = {
        "schema_version": "1.0.0",
        "presentation_frame_index": (
            RECEIPT_FRAME if RECEIPT_FRAME is not None else frame),
        "framebuffer_width": RECEIPT_WIDTH if RECEIPT_WIDTH is not None else width,
        "framebuffer_height": (
            RECEIPT_HEIGHT if RECEIPT_HEIGHT is not None else height),
        "format": "png",
        "image_path": RECEIPT_IMAGE_PATH if RECEIPT_IMAGE_PATH is not None else out,
        "image_sha256": digest,
        "image_byte_size": size,
    }
    with open(receipt_out, "w", encoding="utf-8") as handle:
        json.dump(receipt, handle, indent=2)
        handle.write("\\n")
    if not SKIP_AUDIT and audit_out is not None:
        passes = [{
            "pass_id": pass_id,
            "label": pass_id,
            "cpu_duration_ns": 1,
            "gpu_duration_ns": None,
        } for pass_id in (
            "shadow_near", "shadow_mid", "shadow_far", "scene",
            "temporal_resolve", "postprocess",
        )]
        audit = {
            "schema_version": "1.0.0",
            "identity": {
                "presentation_frame_index": receipt["presentation_frame_index"],
                "framebuffer_width": receipt["framebuffer_width"],
                "framebuffer_height": receipt["framebuffer_height"],
                "renderer_version": "v2",
            },
            "device": {
                "adapter_name": "test adapter", "backend": "test backend",
                "driver": "test driver", "driver_info": "test driver info",
            },
            "environment": {
                "environment_mode": "physical",
                "physical_atmosphere_active": True,
                "physical_ibl_active": True,
                "aerial_perspective_active": True,
            },
            "image_pipeline": {
                "hdr_scene_format": "Rgba16Float", "exposure_ev": 0.0,
                "tone_mapper": "Khronos PBR Neutral",
                "temporal_resolve_active": True,
            },
            "shadows": {
                "path_active": True, "cascade_count": 3,
                "map_resolution": 2048,
                "split_distances_m": [32.0, 128.0, 512.0],
                "filtering": "PCF", "filter_tap_count": None,
                "filter_tap_count_unavailable_reason": "not runtime state",
            },
            "terrain": {
                "render_mode": "flat", "debug_mode": "final",
                "material_path_active": True, "sampler_anisotropy": 16,
                "material_scale": None,
                "material_scale_unavailable_reason": "multi-frequency material",
            },
            "vegetation": {
                "vegetation_present": True, "debug_mode": "final",
                "stats": {
                    "total": 1, "visible": 1, "culled_frustum": 0,
                    "culled_distance": 0, "lod_counts": [1, 0, 0],
                    "scene_draw_calls": 1, "shadow_draw_calls": 3,
                    "uploaded_instance_bytes": 48,
                },
            },
            "profiling": {
                "presentation_frame_index": receipt["presentation_frame_index"],
                "cpu_frame_duration_ns": 1,
                "gpu_timing_status": "timestamp_query_unsupported",
                "gpu_timing_source_presentation_frame_index": None,
                "gpu_timing_frame_age": None,
                "gpu_timing_unavailable_reason": "unsupported in test adapter",
                "passes": passes,
            },
        }
        with open(audit_out, "w", encoding="utf-8") as handle:
            json.dump(audit, handle, indent=2)
            handle.write("\\n")
    return EXIT_CODE


sys.exit(main())
'''

FAKE_RUNTIME_DEFAULTS = {
    "WIDTH": None,
    "HEIGHT": None,
    "FRAME": None,
    "RECEIPT_FRAME": None,
    "RECEIPT_WIDTH": None,
    "RECEIPT_HEIGHT": None,
    "RECEIPT_IMAGE_PATH": None,
    "RECEIPT_SHA": None,
    "RECEIPT_SIZE": None,
    "EXIT_CODE": 0,
    "SKIP_RECEIPT": False,
    "SKIP_AUDIT": False,
    "CORRUPT_PNG": False,
}


class RunnerTestCase(unittest.TestCase):
    """Shared temp-dir, manifest-writing and patching helpers."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.tmp = Path(directory.name)
        # The VIS0-A validator checks that aircraft.model exists relative to the
        # manifest directory, so materialise a stand-in model file.
        models = self.tmp / "models" / "acro_electric_01"
        models.mkdir(parents=True)
        (models / "model.json").write_text("{}", encoding="utf-8")

    def patch(self, name, value):
        original = getattr(rb, name)
        setattr(rb, name, value)
        self.addCleanup(setattr, rb, name, original)

    def write_manifest(self, manifest=None, name="manifest.json") -> Path:
        payload = base_manifest() if manifest is None else manifest
        path = self.tmp / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def build_plan(self, manifest=None, app="rcsim-app", dry_run=True, **kwargs):
        path = self.write_manifest(manifest)
        loaded = json.loads(path.read_text(encoding="utf-8"))
        return rb.build_plan(
            manifest=loaded,
            manifest_path=path,
            output_dir=self.tmp / "out",
            app=app,
            run_index=kwargs.pop("run_index", 0),
            dry_run=dry_run,
            timeout_seconds=kwargs.pop("timeout_seconds", None),
            require_clean_git=kwargs.pop("require_clean_git", False),
            git_provenance=kwargs.pop("git_provenance", dict(FAKE_GIT)),
        )

    def plan_for(self, manifest=None, output_dir=None, **kwargs):
        """Build a plan against an explicit output directory."""
        path = self.write_manifest(manifest)
        loaded = json.loads(path.read_text(encoding="utf-8"))
        return rb.build_plan(
            manifest=loaded,
            manifest_path=path,
            output_dir=output_dir or (self.tmp / "out"),
            app=kwargs.pop("app", "rcsim-app"),
            run_index=0,
            dry_run=kwargs.pop("dry_run", True),
            timeout_seconds=None,
            require_clean_git=False,
            git_provenance=dict(FAKE_GIT),
        )

    def run_main(self, argv):
        """Invoke rb.main capturing both streams. Returns (exit_code, out, err)."""
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = rb.main(argv)
        return code, out.getvalue(), err.getvalue()

    @staticmethod
    def argv_value(plan, flag):
        """Return the token following `flag` in command_argv, or None."""
        argv = plan["command_argv"]
        return argv[argv.index(flag) + 1] if flag in argv else None

    def synthetic_plan(self, argv, manifest=None):
        """A real plan whose command_argv is replaced by a harmless stand-in.

        execute_plan needs the full plan shape (scene_output_dir, capture_plan,
        artifact_paths), so tests substitute only the command, never the plan.
        """
        plan = self.build_plan(manifest)
        plan["command_argv"] = list(argv)
        return plan

    def write_fake_runtime(self, name="fake_runtime.py", **overrides):
        """Materialise the rcsim-app artifact stand-in with baked-in behaviour."""
        values = dict(FAKE_RUNTIME_DEFAULTS)
        unknown = sorted(set(overrides) - set(values))
        if unknown:
            raise AssertionError(f"unknown fake runtime knob(s): {unknown}")
        values.update(overrides)
        source = FAKE_RUNTIME_SOURCE
        for key, value in values.items():
            source = source.replace(f"__{key}__", repr(value))
        path = self.tmp / name
        path.write_text(source, encoding="utf-8")
        return path

    def install_runtime_shim(self):
        """Launch whatever `--app` names through the interpreter, exactly once.

        Patching per call would nest the shims, so a test that executes twice
        would hand the second run an argv the first shim already rewrote.
        """
        if getattr(self, "_runtime_shim_installed", False):
            return
        real_run_process = rb.run_process

        def invoke(argv, cwd=None, timeout_seconds=None):
            return real_run_process([sys.executable, argv[0]] + list(argv[1:]),
                                    cwd=cwd, timeout_seconds=timeout_seconds)

        self.patch("run_process", invoke)
        self._runtime_shim_installed = True

    def execute_with_fake_runtime(self, manifest=None, runtime=None,
                                  output_dir=None, extra_args=(), **overrides):
        """Run the whole --execute path against the artifact stand-in.

        `--app` receives the script path; run_process is shimmed to launch it
        with the interpreter, exactly like the existing real-subprocess tests.
        No GPU, no window and no rendering are involved - what is exercised for
        real is the runner's execution, receipt, PNG and evidence pipeline.
        """
        script = runtime if runtime is not None else self.write_fake_runtime(**overrides)
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        self.install_runtime_shim()
        manifest_path = self.write_manifest(manifest)
        out = output_dir if output_dir is not None else (self.tmp / "out")
        code, stdout, stderr = self.run_main([
            "--manifest", str(manifest_path), "--execute", "--app", str(script),
            "--output-dir", str(out), "--timeout-seconds", "60", *extra_args,
        ])
        return code, stdout, stderr, out

    def scene_dir(self, output_dir=None, scene_id="test_scene") -> Path:
        return (output_dir or (self.tmp / "out")) / scene_id

    @staticmethod
    def read_json(path: Path) -> dict:
        return json.loads(Path(path).read_text(encoding="utf-8"))

    @staticmethod
    def trusted_verification(receipt: dict, image: dict) -> dict:
        """A verification result shaped like the one verify_capture produces."""
        return {
            "attempted": True,
            "trusted": True,
            "process_ok": True,
            "receipt": dict(receipt),
            "receipt_errors": [],
            "expectation_errors": [],
            "image": dict(image),
            "image_errors": [],
            "checks": [],
            "failure_reason": None,
        }


class TestPlanConstruction(RunnerTestCase):
    """A. valid manifest -> plan success."""

    def test_valid_manifest_builds_plan(self):
        plan = self.build_plan()
        self.assertEqual(plan["scene_id"], "test_scene")
        self.assertEqual(plan["renderer"], "v2")
        self.assertEqual(plan["command_argv"][1], "render")
        self.assertEqual(plan["dry_run"], True)
        self.assertEqual(plan["visual_pass"], None)

    def test_reference_manifest_builds_plan(self):
        """The approved VIS0-A reference manifest must plan without warnings."""
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        plan = rb.build_plan(
            manifest=manifest,
            manifest_path=REFERENCE_MANIFEST,
            output_dir=self.tmp / "out",
            app="target/release/rcsim-app",
            run_index=0,
            dry_run=True,
            timeout_seconds=None,
            require_clean_git=False,
            git_provenance=dict(FAKE_GIT),
        )
        self.assertEqual(plan["scene_id"], "aircraft_acro_static_front")
        self.assertEqual(plan["schema_version"], SUPPORTED_SCHEMA_VERSION)
        self.assertTrue(plan["expected_output_basename"]["matches_vis0_convention"])
        self.assertTrue(plan["schema_version_supported"])

    def test_validation_reuses_approved_validator(self):
        manifest = base_manifest()
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])
        manifest["renderer"]["version"] = "v3"
        errors = rb.validate_manifest_dict(manifest, self.tmp)
        self.assertTrue(any("renderer.version" in error for error in errors))

    def test_runner_rejects_legacy_manifest_version_through_validator(self):
        manifest = base_manifest()
        manifest["schema_version"] = "1.0.0"
        self.assertEqual(
            rb.validate_manifest_dict(manifest, self.tmp),
            [
                "schema_version: unsupported schema version 1.0.0; "
                f"supported version is {SUPPORTED_SCHEMA_VERSION}"
            ],
        )

    def test_output_paths_use_scene_id_without_timestamp(self):
        plan = self.build_plan()
        self.assertTrue(Path(plan["scene_output_dir"]).name == "test_scene")
        for key, value in plan["artifact_paths"].items():
            self.assertTrue(value.endswith((".json", ".txt", ".png")), key)
        self.assertNotRegex(plan["scene_output_dir"], r"\d{4}-\d{2}-\d{2}")

    def test_manifest_sha256_is_recorded(self):
        path = self.write_manifest()
        plan = self.build_plan()
        self.assertEqual(plan["manifest_sha256"], rb.sha256_of_file(path))
        self.assertEqual(len(plan["manifest_sha256"]), 64)


class TestDeterminism(RunnerTestCase):
    """C. same manifest -> same command and order."""

    def test_same_manifest_produces_identical_command(self):
        first = self.build_plan()
        second = self.build_plan()
        self.assertEqual(first["command_argv"], second["command_argv"])
        self.assertEqual(first["manifest_sha256"], second["manifest_sha256"])

    def test_field_mapping_order_is_stable(self):
        first = [m["manifest_field"] for m in self.build_plan()["field_mapping"]]
        second = [m["manifest_field"] for m in self.build_plan()["field_mapping"]]
        self.assertEqual(first, second)
        # Mapping order must follow the declared canonical order.
        canonical = [f for f in rb.CANONICAL_FIELD_ORDER if f in first]
        self.assertEqual(first, canonical)

    def test_manifest_key_insertion_order_does_not_change_argv(self):
        forward = base_manifest()
        backward = {key: forward[key] for key in reversed(list(forward))}
        self.assertEqual(
            self.build_plan(forward)["command_argv"],
            self.build_plan(backward)["command_argv"],
        )

    def test_plan_json_round_trips(self):
        plan = self.build_plan()
        self.assertEqual(json.loads(json.dumps(plan)), plan)


class TestCameraMapping(RunnerTestCase):
    """D. pilot mapping, E. chase mapping."""

    def test_pilot_camera_mapping(self):
        plan = self.build_plan()
        argv = plan["command_argv"]
        self.assertEqual(self.argv_value(plan, "--camera"), "pilot")
        self.assertEqual(self.argv_value(plan, "--camera-fov"), "55")
        self.assertEqual(self.argv_value(plan, "--pilot-position"), "0.0,1.8,20.0")
        self.assertNotIn("--chase-distance-m", argv)
        self.assertNotIn("--chase-height-m", argv)

    def test_chase_camera_mapping(self):
        manifest = base_manifest()
        manifest["camera"] = {
            "mode": "chase",
            "vertical_fov_deg": 62.5,
            "chase_distance_behind_m": 8.25,
            "chase_height_above_m": 2.5,
        }
        plan = self.build_plan(manifest)
        argv = plan["command_argv"]
        self.assertEqual(self.argv_value(plan, "--camera"), "chase")
        self.assertEqual(self.argv_value(plan, "--camera-fov"), "62.5")
        self.assertEqual(self.argv_value(plan, "--chase-distance-m"), "8.25")
        self.assertEqual(self.argv_value(plan, "--chase-height-m"), "2.5")
        self.assertNotIn("--pilot-position", argv)

    def test_float_fov_is_not_truncated_to_int(self):
        manifest = base_manifest()
        manifest["camera"]["vertical_fov_deg"] = 47.5
        self.assertEqual(self.argv_value(self.build_plan(manifest), "--camera-fov"), "47.5")

    def test_cross_mode_field_is_omitted_not_emitted(self):
        """A stale field for the other mode must never reach argv."""
        manifest = base_manifest()
        # pilot mode, but a chase field smuggled in (validator rejects this; the
        # runner must still refuse to emit the incompatible flag).
        manifest["camera"]["chase_distance_behind_m"] = 8.0
        plan = self.build_plan(manifest)
        self.assertNotIn("--chase-distance-m", plan["command_argv"])
        mapping = next(
            m
            for m in plan["field_mapping"]
            if m["manifest_field"] == "camera.chase_distance_behind_m"
        )
        self.assertFalse(mapping["emitted"])
        self.assertIn("IncompatibleCameraOption", mapping["reason"])


class TestExposureAndDebugMapping(RunnerTestCase):
    """F. exposure mapping, G. terrain/vegetation debug mapping."""

    def test_exposure_mapping_integer(self):
        self.assertEqual(self.argv_value(self.build_plan(), "--exposure-ev"), "0")

    def test_exposure_mapping_negative_float(self):
        manifest = base_manifest()
        manifest["exposure_ev"] = -2.5
        self.assertEqual(self.argv_value(self.build_plan(manifest), "--exposure-ev"), "-2.5")

    def test_exposure_extremes(self):
        for value, expected in ((8, "8"), (-8, "-8"), (0.25, "0.25")):
            manifest = base_manifest()
            manifest["exposure_ev"] = value
            self.assertEqual(
                self.argv_value(self.build_plan(manifest), "--exposure-ev"), expected
            )

    def test_terrain_and_vegetation_debug_mapping(self):
        manifest = base_manifest()
        manifest["renderer"]["terrain_debug"] = "albedo"
        manifest["renderer"]["vegetation_debug"] = "lod"
        plan = self.build_plan(manifest)
        self.assertEqual(self.argv_value(plan, "--terrain-debug"), "albedo")
        self.assertEqual(self.argv_value(plan, "--vegetation-debug"), "lod")

    def test_every_debug_mode_is_mappable(self):
        for mode in rb.ManifestValidator.TERRAIN_DEBUG_MODES:
            manifest = base_manifest()
            manifest["renderer"]["terrain_debug"] = mode
            self.assertEqual(self.argv_value(self.build_plan(manifest), "--terrain-debug"), mode)
        for mode in rb.ManifestValidator.VEGETATION_DEBUG_MODES:
            manifest = base_manifest()
            manifest["renderer"]["vegetation_debug"] = mode
            self.assertEqual(
                self.argv_value(self.build_plan(manifest), "--vegetation-debug"), mode
            )

    def test_absent_optional_debug_fields_are_not_emitted(self):
        manifest = base_manifest()
        del manifest["renderer"]["terrain_debug"]
        del manifest["renderer"]["vegetation_debug"]
        del manifest["aircraft"]["throttle"]
        argv = self.build_plan(manifest)["command_argv"]
        self.assertNotIn("--terrain-debug", argv)
        self.assertNotIn("--vegetation-debug", argv)
        self.assertNotIn("--throttle", argv)

    def test_start_on_ground_is_presence_only(self):
        argv = self.build_plan()["command_argv"]
        index = argv.index("--start-on-ground")
        # Presence-only flag: nothing may follow it except another flag.
        self.assertTrue(
            index == len(argv) - 1 or argv[index + 1].startswith("--"),
            f"--start-on-ground consumed a value: {argv[index:index + 2]}",
        )

        manifest = base_manifest()
        manifest["aircraft"]["start_on_ground"] = False
        plan = self.build_plan(manifest)
        self.assertNotIn("--start-on-ground", plan["command_argv"])
        mapping = next(
            m for m in plan["field_mapping"] if m["manifest_field"] == "aircraft.start_on_ground"
        )
        self.assertFalse(mapping["emitted"])
        self.assertIn("presence-only", mapping["reason"])


class TestNoInventedFlags(RunnerTestCase):
    """H. no invented CLI flags."""

    @staticmethod
    def runtime_flags_from_source():
        pattern = re.compile(r'"(--[a-z0-9][a-z0-9-]*)"')
        found = set()
        for source in (RENDER_APP_SOURCE, APP_MAIN_SOURCE):
            if source.exists():
                found.update(pattern.findall(source.read_text(encoding="utf-8")))
        return found

    def test_emittable_flags_exist_in_runtime_source(self):
        found = self.runtime_flags_from_source()
        if not found:
            self.skipTest("runtime sources not available")
        for flag in rb.EMITTABLE_FLAGS:
            self.assertIn(flag, found, f"{flag} is not implemented by rcsim-app")

    def test_known_flags_exist_in_runtime_source(self):
        found = self.runtime_flags_from_source()
        if not found:
            self.skipTest("runtime sources not available")
        for flag in rb.KNOWN_RENDER_FLAGS:
            self.assertIn(flag, found, f"{flag} drifted from the runtime source")

    def test_emittable_is_subset_of_known(self):
        self.assertTrue(set(rb.EMITTABLE_FLAGS).issubset(set(rb.KNOWN_RENDER_FLAGS)))

    def test_every_emitted_flag_is_known(self):
        for manifest in (base_manifest(), self._chase_manifest(), self._minimal_manifest()):
            plan = self.build_plan(manifest)
            for token in plan["command_argv"][2:]:
                if token.startswith("--"):
                    self.assertIn(token, rb.KNOWN_RENDER_FLAGS)
                    self.assertIn(token, rb.EMITTABLE_FLAGS)

    def test_unknown_flag_is_refused(self):
        with self.assertRaises(rb.RunnerError):
            rb._assert_no_invented_flags(["rcsim-app", "render", "--ssim-threshold", "0.9"])

    def test_non_emittable_flag_is_refused(self):
        """Flags that exist at runtime but are not manifest-expressible stay out."""
        with self.assertRaises(rb.RunnerError):
            rb._assert_no_invented_flags(["rcsim-app", "render", "--debug-overlays"])

    def test_unmapped_manifest_field_fails_closed(self):
        manifest = base_manifest()
        manifest["weather"] = "clear"
        with self.assertRaises(rb.RunnerError) as caught:
            self.build_plan(manifest)
        self.assertIn("weather", caught.exception.message)

    def test_unmapped_nested_manifest_field_fails_closed(self):
        manifest = base_manifest()
        manifest["renderer"]["msaa_samples"] = 4
        with self.assertRaises(rb.RunnerError) as caught:
            self.build_plan(manifest)
        self.assertIn("renderer.msaa_samples", caught.exception.message)

    def test_no_invented_capture_or_warmup_flag_is_ever_emitted(self):
        """Only the four real C2A capture flags plus the real exit flag appear."""
        plan = self.build_plan()
        argv = plan["command_argv"]
        for forbidden in ("--warmup", "--warmup-frames", "--screenshot", "--output",
                          "--capture", "--capture-image", "--capture-png",
                          "--capture-receipt", "--quality"):
            self.assertNotIn(forbidden, argv)
        emitted = [token for token in argv[2:] if token.startswith("--")]
        self.assertEqual(
            sorted(set(emitted) & {
                "--capture-frame", "--capture-out", "--capture-format",
                "--capture-receipt-out", "--exit-after-frame",
            }),
            ["--capture-format", "--capture-frame", "--capture-out",
             "--capture-receipt-out", "--exit-after-frame"],
        )
        for token in emitted:
            self.assertIn(token, rb.EMITTABLE_FLAGS, token)

    def test_capture_flags_really_exist_in_the_runtime_source(self):
        if not RENDER_APP_SOURCE.exists():
            self.skipTest("runtime source not available")
        text = RENDER_APP_SOURCE.read_text(encoding="utf-8")
        for flag in ("--capture-frame", "--capture-out", "--capture-format",
                     "--capture-receipt-out", "--exit-after-frame"):
            self.assertIn(f'"{flag}" =>', text, flag)

    def _chase_manifest(self):
        manifest = base_manifest()
        manifest["camera"] = {
            "mode": "chase",
            "vertical_fov_deg": 60,
            "chase_distance_behind_m": 5.0,
            "chase_height_above_m": 1.5,
        }
        return manifest

    def _minimal_manifest(self):
        manifest = base_manifest()
        del manifest["renderer"]["terrain_debug"]
        del manifest["renderer"]["vegetation_debug"]
        del manifest["aircraft"]["throttle"]
        del manifest["aircraft"]["start_on_ground"]
        del manifest["capture"]["frame"]
        return manifest


class TestArgvSafety(RunnerTestCase):
    """I. argv quoting and paths containing spaces stay safe."""

    def test_app_path_with_spaces_is_one_argv_element(self):
        spacey = r"C:\my build dir\out release\rcsim-app.exe"
        plan = self.build_plan(app=spacey)
        self.assertEqual(plan["command_argv"][0], spacey)
        self.assertNotIn(" ", "".join(plan["command_argv"][1:2]))

    def test_posix_app_path_with_spaces_round_trips_through_display(self):
        spacey = "/opt/my build dir/rcsim-app"
        plan = self.build_plan(app=spacey)
        self.assertEqual(plan["command_argv"][0], spacey)
        self.assertEqual(shlex.split(plan["command_display"]), plan["command_argv"])

    def test_output_dir_with_spaces_is_preserved(self):
        spacey = self.tmp / "dir with spaces"
        resolved = rb._resolve_output_dir(str(spacey))
        self.assertEqual(resolved, spacey)
        self.assertIn("dir with spaces", str(resolved))

    def test_command_argv_is_a_list_of_strings(self):
        argv = self.build_plan()["command_argv"]
        self.assertIsInstance(argv, list)
        for token in argv:
            self.assertIsInstance(token, str)

    def test_subprocess_is_invoked_without_a_shell(self):
        """execute_plan must hand a list to the launcher, never a shell string."""
        recorded = {}

        def fake_invoke(argv, cwd=None, timeout_seconds=None):
            recorded["argv"] = argv
            recorded["cwd"] = cwd
            return rb.ProcessOutcome(
                argv=list(argv), returncode=0, started_at_utc="t0", ended_at_utc="t1"
            )

        spacey = r"C:\my build dir\rcsim-app.exe"
        plan = self.build_plan(app=spacey)
        rb.execute_plan(plan, spacey, 30, invoke=fake_invoke)
        self.assertIsInstance(recorded["argv"], list)
        self.assertEqual(recorded["argv"][0], spacey)
        self.assertNotIn("--debug-overlays", recorded["argv"])

    def test_no_shell_metacharacter_splitting(self):
        """A value with shell metacharacters stays a single argv element."""
        manifest = base_manifest()
        manifest["camera"]["pilot_position_render_m"] = [0.0, 1.8, 20.0]
        plan = self.build_plan(app="rcsim-app && rm -rf /")
        self.assertEqual(plan["command_argv"][0], "rcsim-app && rm -rf /")
        self.assertEqual(plan["command_argv"][1], "render")


class TestExecutionFailures(RunnerTestCase):
    """J. missing executable, K. non-zero exit, L. timeout."""

    def test_missing_executable_path_fails_cleanly(self):
        missing = str(self.tmp / "definitely missing" / "rcsim-app")
        with self.assertRaises(rb.RunnerError) as caught:
            rb.resolve_app(missing)
        self.assertEqual(caught.exception.exit_code, rb.EXIT_EXECUTION_FAILED)
        self.assertIn("executable not found", caught.exception.message)

    def test_missing_executable_via_main_returns_exit_4(self):
        path = self.write_manifest()
        missing = str(self.tmp / "nope" / "rcsim-app")
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--execute", "--app", missing,
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED)
        self.assertIn("executable not found", err)
        self.assertNotIn("Traceback", err)

    def test_run_process_reports_executable_not_found(self):
        outcome = rb.run_process(["this-executable-does-not-exist-xyz", "render"])
        self.assertEqual(outcome.failure_kind, "executable_not_found")
        self.assertIsNone(outcome.returncode)
        self.assertFalse(outcome.succeeded)

    def test_non_zero_exit_is_captured_with_streams(self):
        plan = self._synthetic_plan([
            sys.executable, "-c",
            "import sys;sys.stdout.write('OUT');sys.stderr.write('ERR');sys.exit(3)",
        ])
        execution = rb.execute_plan(plan, sys.executable, 30)
        self.assertEqual(execution["exit_code"], 3)
        self.assertFalse(execution["execution_success"])
        self.assertIsNone(execution["failure_kind"])
        self.assertEqual(Path(execution["stdout_path"]).read_text(encoding="utf-8"), "OUT")
        self.assertEqual(Path(execution["stderr_path"]).read_text(encoding="utf-8"), "ERR")

    def test_successful_exit_is_reported_as_execution_success(self):
        plan = self._synthetic_plan([sys.executable, "-c", "print('ok')"])
        execution = rb.execute_plan(plan, sys.executable, 30)
        self.assertEqual(execution["exit_code"], 0)
        self.assertTrue(execution["execution_success"])
        self.assertIn("ok", Path(execution["stdout_path"]).read_text(encoding="utf-8"))

    def test_timeout_is_handled_without_raising(self):
        plan = self._synthetic_plan([sys.executable, "-c", "import time;time.sleep(30)"])
        execution = rb.execute_plan(plan, sys.executable, 1)
        self.assertEqual(execution["failure_kind"], "timeout")
        self.assertIsNone(execution["exit_code"])
        self.assertFalse(execution["execution_success"])
        self.assertIn("did not exit within 1s", execution["failure_message"])

    def test_timeout_via_main_returns_exit_4(self):
        path = self.write_manifest()
        sleeper = self.tmp / "sleeper.py"
        sleeper.write_text("import time\ntime.sleep(30)\n", encoding="utf-8")
        synthetic = self.synthetic_plan([sys.executable, str(sleeper)])
        # Capture the real implementation before patching, or the lambda would
        # recurse into itself.
        real_execute_plan = rb.execute_plan
        self.patch(
            "execute_plan",
            lambda *_a, **_k: real_execute_plan(synthetic, sys.executable, 1),
        )
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--execute", "--app", sys.executable,
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED)
        self.assertIn("timeout", err)
        self.assertNotIn("Traceback", err)

    def test_execute_without_app_is_a_usage_error(self):
        path = self.write_manifest()
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--execute", "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("--execute requires --app", err)
        self.assertNotIn("Traceback", err)

    def _synthetic_plan(self, argv):
        return self.synthetic_plan(argv)


class TestInvalidManifestGating(RunnerTestCase):
    """B. invalid manifest -> no execution."""

    def test_invalid_manifest_returns_exit_1(self):
        manifest = base_manifest()
        manifest["renderer"]["version"] = "v3"
        path = self.write_manifest(manifest)
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--execute", "--app", sys.executable,
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("no process was started", err)
        self.assertIn("renderer.version", err)

    def test_invalid_manifest_starts_no_process(self):
        manifest = base_manifest()
        manifest["scenery"]["preset"] = "city"
        path = self.write_manifest(manifest)
        calls = []

        def recorder(*args, **kwargs):
            calls.append((args, kwargs))
            return rb.ProcessOutcome(argv=["git"], returncode=0, stdout="", stderr="")

        self.patch("run_process", recorder)
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--execute", "--app", sys.executable,
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        for args, _kwargs in calls:
            self.assertNotEqual(args[0][1:2], ["render"])

    def test_invalid_manifest_creates_no_output_directory(self):
        manifest = base_manifest()
        manifest["camera"]["vertical_fov_deg"] = 500
        path = self.write_manifest(manifest)
        output = self.tmp / "out"
        self.run_main(
            ["--manifest", str(path), "--execute", "--app", sys.executable,
             "--output-dir", str(output)]
        )
        self.assertFalse(output.exists())

    def test_missing_manifest_returns_exit_2(self):
        code, _out, err = self.run_main(["--manifest", str(self.tmp / "gone.json")])
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("manifest not found", err)
        self.assertNotIn("Traceback", err)

    def test_malformed_json_returns_exit_2(self):
        path = self.tmp / "broken.json"
        path.write_text("{not json", encoding="utf-8")
        code, _out, err = self.run_main(["--manifest", str(path)])
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("not valid JSON", err)

    def test_non_object_manifest_returns_exit_2(self):
        path = self.tmp / "list.json"
        path.write_text("[]", encoding="utf-8")
        code, _out, err = self.run_main(["--manifest", str(path)])
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("must be an object", err)

    def test_validation_runs_before_git_policy(self):
        """A dirty repo must not mask an invalid manifest."""
        manifest = base_manifest()
        manifest["renderer"]["version"] = "v9"
        path = self.write_manifest(manifest)
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--require-clean-git"]
        )
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)


class TestGitProvenance(RunnerTestCase):
    """N. git SHA parsing, O. dirty repo strict policy."""

    @staticmethod
    def fake_git(responses):
        def invoke(argv, cwd=None, timeout_seconds=None):
            key = " ".join(argv[1:])
            entry = responses.get(key)
            if entry is None:
                return rb.ProcessOutcome(
                    argv=list(argv), returncode=128, stdout="", stderr="fatal: unknown"
                )
            stdout, code = entry
            return rb.ProcessOutcome(
                argv=list(argv), returncode=code, stdout=stdout, stderr=""
            )

        return invoke

    def test_commit_sha_and_short_sha_are_parsed(self):
        sha = "ae9757fd1c4502ed794cda870e2effa2e7c818fe"
        invoke = self.fake_git({
            "rev-parse HEAD": (sha + "\n", 0),
            "rev-parse --short=12 HEAD": (sha[:12] + "\n", 0),
            "branch --show-current": ("feature/rv2-vis0-benchmark-runner\n", 0),
            "status --porcelain": ("", 0),
        })
        provenance = rb.collect_git_provenance(self.tmp, invoke=invoke)
        self.assertEqual(provenance["commit_sha"], sha)
        self.assertEqual(len(provenance["commit_sha"]), 40)
        self.assertEqual(provenance["commit_sha_short"], sha[:12])
        self.assertEqual(provenance["branch"], "feature/rv2-vis0-benchmark-runner")
        self.assertTrue(provenance["available"])
        self.assertFalse(provenance["dirty"])

    def test_detached_head_is_reported(self):
        invoke = self.fake_git({
            "rev-parse HEAD": ("b" * 40 + "\n", 0),
            "rev-parse --short=12 HEAD": ("b" * 12 + "\n", 0),
            "branch --show-current": ("", 0),
            "status --porcelain": ("", 0),
        })
        provenance = rb.collect_git_provenance(self.tmp, invoke=invoke)
        self.assertIsNone(provenance["branch"])
        self.assertTrue(provenance["detached_head"])

    def test_dirty_entries_are_counted_and_split(self):
        invoke = self.fake_git({
            "rev-parse HEAD": ("c" * 40 + "\n", 0),
            "rev-parse --short=12 HEAD": ("c" * 12 + "\n", 0),
            "branch --show-current": ("main\n", 0),
            "status --porcelain": ("?? build.log\n M crates/app/src/main.rs\n", 0),
        })
        provenance = rb.collect_git_provenance(self.tmp, invoke=invoke)
        self.assertTrue(provenance["dirty"])
        self.assertEqual(provenance["dirty_entry_count"], 2)
        self.assertEqual(provenance["dirty_tracked_entry_count"], 1)

    def test_missing_git_is_not_fatal(self):
        def invoke(argv, cwd=None, timeout_seconds=None):
            return rb.ProcessOutcome(
                argv=list(argv), failure_kind="executable_not_found",
                failure_message="git not installed",
            )

        provenance = rb.collect_git_provenance(self.tmp, invoke=invoke)
        self.assertFalse(provenance["available"])
        self.assertIsNone(provenance["commit_sha"])
        self.assertIsNone(provenance["dirty"])
        self.assertTrue(provenance["errors"])

    def test_no_upstream_is_not_an_error(self):
        invoke = self.fake_git({
            "rev-parse HEAD": ("d" * 40 + "\n", 0),
            "rev-parse --short=12 HEAD": ("d" * 12 + "\n", 0),
            "branch --show-current": ("local-only\n", 0),
            "status --porcelain": ("", 0),
        })
        provenance = rb.collect_git_provenance(self.tmp, invoke=invoke)
        self.assertEqual(provenance["errors"], [])
        self.assertIsNone(provenance["upstream"])

    def test_require_clean_git_rejects_dirty_repo(self):
        dirty = dict(FAKE_GIT, dirty=True, dirty_entry_count=1,
                     dirty_entries=[" M crates/renderer/src/gpu.rs"])
        with self.assertRaises(rb.RunnerError) as caught:
            rb.enforce_git_policy(dirty, require_clean=True)
        self.assertEqual(caught.exception.exit_code, rb.EXIT_GIT_POLICY)
        self.assertIn("repository is dirty", caught.exception.message)
        self.assertIn("gpu.rs", caught.exception.message)

    def test_require_clean_git_accepts_clean_repo(self):
        rb.enforce_git_policy(dict(FAKE_GIT, dirty=False), require_clean=True)

    def test_require_clean_git_fails_when_status_unavailable(self):
        unknown = dict(FAKE_GIT, dirty=None, errors=["git status --porcelain: failed"])
        with self.assertRaises(rb.RunnerError) as caught:
            rb.enforce_git_policy(unknown, require_clean=True)
        self.assertEqual(caught.exception.exit_code, rb.EXIT_GIT_POLICY)
        self.assertIn("cannot be satisfied", caught.exception.message)

    def test_clean_policy_is_not_enforced_by_default(self):
        rb.enforce_git_policy(dict(FAKE_GIT, dirty=True), require_clean=False)

    def test_require_clean_git_via_main_returns_exit_3(self):
        path = self.write_manifest()
        dirty = dict(FAKE_GIT, dirty=True, dirty_entry_count=1, dirty_entries=["?? x"])
        self.patch("collect_git_provenance", lambda _root: dirty)
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--require-clean-git",
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_GIT_POLICY)
        self.assertIn("repository is dirty", err)

    def test_real_repository_provenance_is_collected(self):
        provenance = rb.collect_git_provenance(REPO_ROOT)
        if not provenance["available"]:
            self.skipTest("git unavailable in this environment")
        self.assertRegex(provenance["commit_sha"], r"^[0-9a-f]{40}$")
        self.assertIsNotNone(provenance["dirty"])


class TestProvenanceMetadata(RunnerTestCase):
    """M. provenance JSON valid."""

    def run_execution(self, argv=None):
        argv = argv or [sys.executable, "-c", "print('hello')"]
        plan = self.synthetic_plan(argv)
        execution = rb.execute_plan(plan, argv[0], 30)
        run_json = Path(plan["artifact_paths"]["run_json"])
        execution["artifacts_written"].append(str(run_json))
        metadata = rb.build_run_metadata(plan, execution)
        rb.write_json(run_json, metadata)
        return run_json, metadata

    def test_run_json_is_valid_json_with_required_provenance(self):
        run_json, _ = self.run_execution()
        payload = json.loads(run_json.read_text(encoding="utf-8"))
        self.assertEqual(payload["manifest"]["schema_version"], SUPPORTED_SCHEMA_VERSION)
        self.assertEqual(payload["manifest"]["scene_id"], "test_scene")
        self.assertEqual(len(payload["manifest"]["sha256"]), 64)
        self.assertEqual(len(payload["git"]["commit_sha"]), 40)
        self.assertEqual(payload["execution"]["exit_code"], 0)
        self.assertTrue(payload["execution"]["execution_success"])
        self.assertIsInstance(payload["execution"]["command_argv"], list)

    def test_environment_provenance_fields(self):
        _run_json, metadata = self.run_execution()
        environment = metadata["environment"]
        for key in ("operating_system", "architecture", "python_version",
                    "python_executable", "runner_version", "platform"):
            self.assertIn(key, environment)
            self.assertTrue(environment[key], key)

    def test_timestamps_are_iso8601_and_ordered(self):
        _run_json, metadata = self.run_execution()
        execution = metadata["execution"]
        pattern = r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\+00:00$"
        self.assertRegex(execution["started_at_utc"], pattern)
        self.assertRegex(execution["ended_at_utc"], pattern)
        self.assertLessEqual(execution["started_at_utc"], execution["ended_at_utc"])
        self.assertGreaterEqual(execution["duration_seconds"], 0.0)

    def test_artifact_paths_are_recorded_and_exist(self):
        run_json, metadata = self.run_execution()
        self.assertTrue(run_json.exists())
        self.assertTrue(Path(metadata["execution"]["stdout_path"]).exists())
        self.assertTrue(Path(metadata["execution"]["stderr_path"]).exists())
        # No capture verification was performed, so no image may be advertised.
        self.assertIsNone(metadata["artifacts"]["capture"])
        self.assertFalse(metadata["artifacts"]["capture_verified"])
        self.assertIn("not verified", metadata["artifacts"]["capture_reason"])

    def test_plan_json_flag_writes_a_file(self):
        path = self.write_manifest()
        plan_path = self.tmp / "plan.json"
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--plan-json", str(plan_path),
             "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)
        payload = json.loads(plan_path.read_text(encoding="utf-8"))
        self.assertEqual(payload["plan_version"], rb.PLAN_VERSION)
        self.assertTrue(payload["dry_run"])

    def test_dry_run_writes_no_run_artifacts(self):
        path = self.write_manifest()
        output = self.tmp / "out"
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(output)]
        )
        self.assertFalse(output.exists())

    def test_run_json_is_written_on_execution(self):
        path = self.write_manifest()
        output = self.tmp / "out"
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--execute", "--app", sys.executable,
             "--output-dir", str(output), "--timeout-seconds", "30"]
        )
        # sys.executable receives "render" as a script path and exits non-zero:
        # a real process ran and its failure was captured, not swallowed.
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED)
        run_json = output / "test_scene" / "run.json"
        self.assertTrue(run_json.exists())
        payload = json.loads(run_json.read_text(encoding="utf-8"))
        self.assertFalse(payload["execution"]["execution_success"])
        self.assertIsNotNone(payload["execution"]["exit_code"])
        self.assertIsNone(payload["verdict"]["visual_pass"])


class TestVisualVerdictPolicy(RunnerTestCase):
    """P. visual_pass is never auto-approved."""

    def test_plan_visual_pass_is_null(self):
        self.assertIsNone(self.build_plan()["visual_pass"])

    def test_successful_execution_does_not_set_visual_pass(self):
        plan = self.build_plan()
        execution = {
            "mode": "execute",
            "exit_code": 0,
            "execution_success": True,
            "failure_kind": None,
            "failure_message": None,
            "stdout_path": None,
            "stderr_path": None,
            "artifacts_written": [],
        }
        metadata = rb.build_run_metadata(plan, execution)
        self.assertIsNone(metadata["verdict"]["visual_pass"])
        self.assertTrue(metadata["verdict"]["execution_success"])
        self.assertIn("never", metadata["verdict"]["visual_pass_reason"].lower())

    def test_failed_execution_does_not_set_visual_pass(self):
        plan = self.build_plan()
        execution = {"execution_success": False, "exit_code": 1, "stdout_path": None,
                     "stderr_path": None, "artifacts_written": []}
        self.assertIsNone(rb.build_run_metadata(plan, execution)["verdict"]["visual_pass"])

    def test_execution_success_and_visual_pass_are_distinct_keys(self):
        plan = self.build_plan()
        execution = {"execution_success": True, "exit_code": 0, "stdout_path": None,
                     "stderr_path": None, "artifacts_written": []}
        verdict = rb.build_run_metadata(plan, execution)["verdict"]
        self.assertIn("execution_success", verdict)
        self.assertIn("visual_pass", verdict)
        self.assertIsNotNone(verdict["execution_success"])
        self.assertIsNone(verdict["visual_pass"])

    def test_source_never_assigns_visual_pass_true(self):
        source = RUNNER_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn('"visual_pass": True', source)
        self.assertNotIn("'visual_pass': True", source)
        self.assertNotIn("visual_pass = True", source)
        self.assertNotRegex(source, r"\bvisual_pass\b\s*[:=]\s*true", re.IGNORECASE)

    def test_runner_depends_only_on_the_standard_library(self):
        """No pip dependency, and no image/metric library of any kind."""
        allowed = {
            "argparse", "hashlib", "json", "os", "platform", "shlex", "subprocess",
            "sys", "time", "dataclasses", "datetime", "pathlib", "typing",
            # The approved VIS0-A validator, imported as a sibling module.
            "validate_manifest", "tools",
            # The VIS0-C1B capture evidence contract, also a sibling module.
            "validate_capture_evidence",
            # The VIS0-C2B runtime receipt reader, also a sibling module.
            "runtime_capture_receipt",
            # The separate C2D runtime visual audit reader.
            "runtime_visual_audit",
        }
        self.assert_stdlib_only(RUNNER_SOURCE, allowed)

    def test_capture_evidence_module_is_also_standard_library_only(self):
        """Allowlisting a sibling must not open a door to a pip dependency."""
        source = RUNNER_SOURCE.with_name("validate_capture_evidence.py")
        allowed = {
            "argparse", "hashlib", "json", "math", "re", "sys", "pathlib",
            "typing", "validate_manifest", "tools",
        }
        self.assert_stdlib_only(source, allowed)

    def test_receipt_module_is_also_standard_library_only(self):
        """Reading a PNG header must not become an excuse for a pip dependency."""
        source = RUNNER_SOURCE.with_name("runtime_capture_receipt.py")
        allowed = {
            "hashlib", "json", "os", "re", "dataclasses", "pathlib", "typing",
            "validate_manifest", "tools",
        }
        self.assert_stdlib_only(source, allowed)

    def test_visual_audit_module_is_also_standard_library_only(self):
        source = RUNNER_SOURCE.with_name("runtime_visual_audit.py")
        allowed = {"json", "math", "dataclasses", "pathlib", "typing"}
        self.assert_stdlib_only(source, allowed)

    def assert_stdlib_only(self, source: Path, allowed: set):
        tree = ast.parse(Path(source).read_text(encoding="utf-8"))
        roots = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                roots.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
                roots.add(node.module.split(".")[0])
        self.assertTrue(roots)
        for root in sorted(roots):
            self.assertIn(root, allowed, f"unexpected dependency: {root}")

    def test_no_image_or_metric_symbol_is_referenced(self):
        forbidden = {"ssim", "psnr", "lpips", "cv2", "ImageGrab", "ImageChops",
                     "numpy", "np", "mss", "skimage", "PIL", "compare_images"}
        for source in (RUNNER_SOURCE,
                       RUNNER_SOURCE.with_name("runtime_capture_receipt.py")):
            tree = ast.parse(source.read_text(encoding="utf-8"))
            names = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Name):
                    names.add(node.id)
                elif isinstance(node, ast.Attribute):
                    names.add(node.attr)
            self.assertEqual(sorted(names & forbidden), [], source.name)


class TestRuntimeCapabilities(RunnerTestCase):
    """Q. capabilities reflect the real VIS0-C1 + VIS0-C2A runtime surface."""

    def test_capture_backend_is_declared_supported(self):
        capture = self.build_plan()["runtime_capabilities"]["capture_backend"]
        self.assertEqual(capture["status"], "supported")
        self.assertTrue(capture["produces_image"])
        self.assertTrue(capture["final_display_referred_capture"])
        self.assertEqual(capture["supported_formats"], ["png"])

    def test_capture_backend_documents_every_c2a_capability(self):
        capture = self.build_plan()["runtime_capabilities"]["capture_backend"]
        for key in ("frame_selection", "runtime_receipt", "process_auto_exit",
                    "explicit_resolution_enforcement"):
            self.assertIn(key, capture, key)
            self.assertTrue(capture[key]["available"], key)
            self.assertTrue(capture[key]["mechanism"], key)
            self.assertTrue(capture[key]["reason"], key)
        self.assertEqual(capture["frame_selection"]["mechanism"],
                         "--capture-frame N (zero-based presentation frame)")
        self.assertEqual(capture["runtime_receipt"]["kind"], "runtime_capture_receipt")
        self.assertEqual(capture["runtime_receipt"]["schema_version"], "1.0.0")

    def test_no_stale_capture_unavailable_wording_survives(self):
        source = RUNNER_SOURCE.read_text(encoding="utf-8")
        for stale in ("CAPTURE BACKEND NOT YET AVAILABLE", "no framebuffer readback",
                      "no image output", "no PNG writer", "capture backend unavailable"):
            self.assertNotIn(stale, source, stale)

    def test_capability_declaration_does_not_claim_a_capture_was_produced(self):
        """A capability is about the runtime; production is a per-run fact."""
        capture = self.build_plan()["runtime_capabilities"]["capture_backend"]
        self.assertIsNone(capture["capture_produced"])

    def test_resolution_enforcement_is_declared_supported(self):
        capability = self.build_plan()["runtime_capabilities"]["resolution_enforcement"]
        self.assertEqual(capability["status"], "supported")
        self.assertTrue(capability["enforced"])
        self.assertEqual(capability["requested"], {"width": 1920, "height": 1080})
        self.assertIn("--render-width", capability["reason"])

    def test_warmup_is_declared_derived_not_a_flag(self):
        capability = self.build_plan()["runtime_capabilities"]["warmup_frames"]
        self.assertEqual(capability["status"], "derived")
        self.assertTrue(capability["enforced"])
        self.assertEqual(capability["requested"], 10)
        self.assertEqual(capability["capture_frame"], 10)
        self.assertIsNone(capability["runtime_flag"])
        self.assertIn("no --warmup", capability["reason"])
        self.assertIn("NOT supported for arbitrary", capability["reason"])

    def test_warmup_capability_is_not_enforced_when_the_relation_breaks(self):
        manifest = base_manifest()
        manifest["warmup"] = 12
        capability = self.build_plan(manifest)["runtime_capabilities"]["warmup_frames"]
        self.assertEqual(capability["status"], "derived")
        self.assertFalse(capability["enforced"])

    def test_process_auto_exit_is_declared_supported(self):
        capability = self.build_plan()["runtime_capabilities"]["process_auto_exit"]
        self.assertEqual(capability["status"], "supported")
        self.assertIn("--exit-after-frame", capability["reason"])

    def test_resolution_fields_are_mapped_as_cli(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        for field_name in ("resolution.width", "resolution.height"):
            self.assertEqual(mapping[field_name]["status"], "cli", field_name)
            self.assertIsNotNone(mapping[field_name]["runtime_flag"], field_name)
            self.assertTrue(mapping[field_name]["emitted"], field_name)

    def test_capture_frame_is_mapped_to_the_real_capture_flag(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        field = mapping["capture.frame"]
        self.assertEqual(field["status"], "cli")
        self.assertEqual(field["runtime_flag"], "--capture-frame")
        self.assertEqual(field["argv_value"], "10")
        self.assertTrue(field["emitted"])
        self.assertIn("--exit-after-frame", field["reason"])

    def test_capture_filename_and_format_are_cli_mapped(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        self.assertEqual(mapping["capture.filename"]["runtime_flag"], "--capture-out")
        self.assertEqual(mapping["capture.format"]["runtime_flag"], "--capture-format")
        self.assertTrue(mapping["capture.filename"]["emitted"])
        self.assertTrue(mapping["capture.format"]["emitted"])

    def test_warmup_has_derived_status_and_no_flag(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        field = mapping["warmup"]
        self.assertEqual(field["status"], "derived")
        self.assertIsNone(field["runtime_flag"])
        self.assertFalse(field["emitted"])

    def test_metadata_only_fields_are_labelled(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        for field_name in ("schema_version", "scene_id", "description", "tags"):
            self.assertEqual(mapping[field_name]["status"], "metadata-only", field_name)
            self.assertIsNone(mapping[field_name]["runtime_flag"], field_name)

    def test_no_field_that_controls_the_runtime_is_labelled_metadata_only(self):
        """C2B: capture.filename/format/frame really drive rcsim-app now."""
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        for field_name in ("capture.filename", "capture.format", "capture.frame"):
            self.assertNotEqual(mapping[field_name]["status"], "metadata-only", field_name)

    def test_expected_capture_metadata_follows_vis0_convention(self):
        basename = self.build_plan()["expected_output_basename"]
        self.assertEqual(basename["per_vis0_convention"], "test_scene_1920x1080.png")
        self.assertTrue(basename["matches_vis0_convention"])
        self.assertTrue(basename["executable"])
        self.assertTrue(basename["planned_path"].endswith("test_scene_1920x1080.png"))
        # Nothing was executed, so nothing may be reported as verified.
        self.assertIsNone(basename["verified"])
        self.assertIsNone(basename["verified_path"])

    def test_expected_capture_metadata_carries_no_stale_produced_claim(self):
        basename = self.build_plan()["expected_output_basename"]
        self.assertNotIn("produced", basename)
        self.assertNotIn("reason", basename)

    def test_naming_convention_mismatch_is_flagged(self):
        manifest = base_manifest()
        manifest["capture"]["filename"] = "wrong_name.png"
        basename = self.build_plan(manifest)["expected_output_basename"]
        self.assertFalse(basename["matches_vis0_convention"])

    def test_capability_states_survive_into_run_json(self):
        plan = self.build_plan()
        execution = {"execution_success": True, "exit_code": 0, "stdout_path": None,
                     "stderr_path": None, "artifacts_written": []}
        capabilities = rb.build_run_metadata(plan, execution)["capabilities"]
        self.assertEqual(capabilities["capture_backend"]["status"], "supported")
        self.assertTrue(capabilities["capture_backend"]["produces_image"])
        self.assertEqual(capabilities["resolution_enforcement"]["status"], "supported")
        self.assertEqual(capabilities["warmup_frames"]["status"], "derived")
        self.assertEqual(capabilities["process_auto_exit"]["status"], "supported")


class TestCliSurface(RunnerTestCase):
    """Runner CLI contract: safe defaults and predictable errors."""

    def test_default_is_dry_run(self):
        path = self.write_manifest()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, out, _err = self.run_main(
            ["--manifest", str(path), "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)
        self.assertIn("DRY-RUN", out)
        self.assertIn("no application process started", out)

    def test_dry_run_overrides_execute(self):
        path = self.write_manifest()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, out, err = self.run_main(
            ["--manifest", str(path), "--execute", "--dry-run",
             "--app", sys.executable, "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)
        self.assertIn("DRY-RUN", out)
        self.assertIn("dry run wins", err)

    def test_dry_run_prints_machine_readable_plan(self):
        path = self.write_manifest()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        _code, out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        self.assertIn("machine-readable plan (JSON):", out)
        payload = json.loads(out.split("machine-readable plan (JSON):", 1)[1]
                            .split("dry run complete", 1)[0])
        self.assertEqual(payload["scene_id"], "test_scene")

    def test_dry_run_prints_human_readable_plan(self):
        path = self.write_manifest()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        _code, out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        for fragment in ("field mapping", "--pilot-position", "SUPPORTED",
                         "visual_pass", "argument provenance",
                         "--capture-receipt-out", "--exit-after-frame",
                         "runtime receipt", "capture evidence",
                         "success criteria"):
            self.assertIn(fragment, out)
        # The stale "no capture backend" wording must be gone for good.
        self.assertNotIn("CAPTURE BACKEND NOT YET AVAILABLE", out)
        self.assertNotIn("no PNG writer", out)
        self.assertNotIn("no framebuffer readback", out)
        self.assertNotIn("no image output", out)

    def test_manifest_is_required(self):
        with self.assertRaises(SystemExit) as caught:
            rb.build_arg_parser().parse_args([])
        self.assertEqual(caught.exception.code, 2)

    def test_invalid_timeout_is_rejected(self):
        path = self.write_manifest()
        code, _out, err = self.run_main(
            ["--manifest", str(path), "--timeout-seconds", "0"]
        )
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("--timeout-seconds must be a positive integer", err)
        self.assertNotIn("Traceback", err)

    def test_negative_run_index_is_rejected(self):
        path = self.write_manifest()
        code, _out, err = self.run_main(["--manifest", str(path), "--run-index", "-1"])
        self.assertEqual(code, rb.EXIT_INPUT_ERROR)
        self.assertIn("--run-index must be non-negative", err)

    def test_run_index_is_recorded(self):
        self.assertEqual(self.build_plan(run_index=7)["run_index"], 7)

    def test_no_interactive_prompt_is_used(self):
        source = RUNNER_SOURCE.read_text(encoding="utf-8")
        self.assertNotIn("getpass", source)
        self.assertNotIn("shell=True", source)
        tree = ast.parse(source)
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                self.assertNotEqual(node.func.id, "input")

    def test_number_formatting_is_rust_parseable(self):
        for value, expected in ((55, "55"), (0, "0"), (0.0, "0.0"), (1.8, "1.8"),
                                (-2.5, "-2.5"), (62.5, "62.5"), (1000.0, "1000.0")):
            self.assertEqual(rb.format_number(value), expected, value)
            float(expected)  # must be a plain decimal Rust can parse

    def test_number_formatting_rejects_bool(self):
        with self.assertRaises(rb.RunnerError):
            rb.format_number(True)

    def test_vector3_formatting(self):
        self.assertEqual(rb.format_vector3([0.0, 1.8, 20.0]), "0.0,1.8,20.0")
        self.assertEqual(rb.format_vector3([1, 2, 3]), "1,2,3")
        with self.assertRaises(rb.RunnerError):
            rb.format_vector3([1.0, 2.0])


class TestVis0APreservation(RunnerTestCase):
    """The approved VIS0-A contract files must stay authoritative and intact."""

    def test_reference_manifest_still_validates(self):
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(rb.validate_manifest_dict(manifest, REFERENCE_MANIFEST.parent), [])

    def test_runner_imports_the_approved_validator(self):
        from tools.visual_benchmark.validate_manifest import ManifestValidator

        self.assertIs(rb.ManifestValidator, ManifestValidator)

    def test_every_canonical_field_has_a_policy(self):
        for field_name in rb.CANONICAL_FIELD_ORDER:
            self.assertIn(field_name, rb.FIELD_POLICY, field_name)

    def test_every_policy_field_is_in_canonical_order(self):
        self.assertEqual(set(rb.FIELD_POLICY), set(rb.CANONICAL_FIELD_ORDER))

    def test_cli_mapped_fields_all_declare_a_real_flag(self):
        for field_name, policy in rb.FIELD_POLICY.items():
            if policy.status == rb.FIELD_STATUS_CLI:
                self.assertIsNotNone(policy.runtime_flag, field_name)
                self.assertIn(policy.runtime_flag, rb.EMITTABLE_FLAGS, field_name)
            else:
                self.assertIsNone(policy.runtime_flag, field_name)
                self.assertTrue(policy.reason, field_name)


class TestAircraftInitialStateMapping(RunnerTestCase):
    """VIS0-C1B: the airborne initial state reaches the real runtime flags."""

    def test_altitude_maps_to_the_real_flag(self):
        plan = self.build_plan(airborne_manifest())
        self.assertEqual(self.argv_value(plan, "--altitude-m"), "100.0")

    def test_airspeed_maps_to_the_real_flag(self):
        plan = self.build_plan(airborne_manifest())
        self.assertEqual(self.argv_value(plan, "--airspeed-mps"), "18.0")

    def test_both_fields_carry_a_cli_policy(self):
        for field, flag in (("aircraft.altitude_m", "--altitude-m"),
                            ("aircraft.airspeed_mps", "--airspeed-mps")):
            policy = rb.FIELD_POLICY[field]
            self.assertEqual(policy.status, rb.FIELD_STATUS_CLI, field)
            self.assertEqual(policy.runtime_flag, flag, field)

    def test_both_fields_are_in_canonical_order(self):
        self.assertIn("aircraft.altitude_m", rb.CANONICAL_FIELD_ORDER)
        self.assertIn("aircraft.airspeed_mps", rb.CANONICAL_FIELD_ORDER)

    def test_new_flags_are_known_and_emittable(self):
        for flag in ("--altitude-m", "--airspeed-mps"):
            self.assertIn(flag, rb.KNOWN_RENDER_FLAGS, flag)
            self.assertIn(flag, rb.EMITTABLE_FLAGS, flag)

    def test_ground_start_emits_neither_flag(self):
        """On a ground start the runtime ignores both, so the runner omits them."""
        plan = self.build_plan()
        self.assertIsNone(self.argv_value(plan, "--altitude-m"))
        self.assertIsNone(self.argv_value(plan, "--airspeed-mps"))
        self.assertIn("--start-on-ground", plan["command_argv"])

    def test_integer_values_stay_integral_for_rust_parse(self):
        manifest = airborne_manifest()
        manifest["aircraft"]["altitude_m"] = 120
        manifest["aircraft"]["airspeed_mps"] = 20
        plan = self.build_plan(manifest)
        self.assertEqual(self.argv_value(plan, "--altitude-m"), "120")
        self.assertEqual(self.argv_value(plan, "--airspeed-mps"), "20")

    def test_no_flag_is_invented_for_the_new_fields(self):
        """The mapping must reuse the existing CLI, not add new surface."""
        plan = self.build_plan(airborne_manifest())
        self.assertNotIn("--initial-altitude", plan["command_argv"])
        self.assertNotIn("--altitude", plan["command_argv"])
        self.assertNotIn("--airspeed", plan["command_argv"])

    def test_airborne_reference_manifest_emits_both_flags(self):
        path = (REPO_ROOT / "docs" / "validation" / "visual_benchmark"
                / "vis0_reference_scene_airborne.json")
        if not path.exists():
            self.skipTest("airborne reference manifest not available")
        manifest = json.loads(path.read_text(encoding="utf-8"))
        plan = self.build_plan(manifest)
        self.assertEqual(plan["scene_id"], "aircraft_acro_airborne_cruise")
        self.assertEqual(self.argv_value(plan, "--altitude-m"), "100.0")
        self.assertEqual(self.argv_value(plan, "--airspeed-mps"), "18.0")

    def test_runtime_source_really_implements_both_flags(self):
        if not RENDER_APP_SOURCE.exists():
            self.skipTest("runtime source not available")
        text = RENDER_APP_SOURCE.read_text(encoding="utf-8")
        self.assertIn('"--altitude-m" =>', text)
        self.assertIn('"--airspeed-mps" =>', text)

    def test_airborne_manifest_plans_end_to_end(self):
        path = self.write_manifest(airborne_manifest(), name="airborne.json")
        code, _, err = self.run_main(["--manifest", str(path), "--dry-run"])
        self.assertEqual(code, rb.EXIT_OK, err)

    def test_airborne_manifest_with_implicit_state_is_refused(self):
        """Omitting altitude_m would leave the state to DEFAULT_ALTITUDE_M."""
        manifest = airborne_manifest()
        del manifest["aircraft"]["altitude_m"]
        path = self.write_manifest(manifest, name="implicit.json")
        code, _, _ = self.run_main(["--manifest", str(path), "--dry-run"])
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)

    def test_ground_start_with_airborne_state_is_refused(self):
        """The runtime would silently ignore them, so the contract forbids them."""
        manifest = base_manifest()
        manifest["aircraft"]["altitude_m"] = 100.0
        path = self.write_manifest(manifest, name="contradictory.json")
        code, _, _ = self.run_main(["--manifest", str(path), "--dry-run"])
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)


class TestCaptureEvidenceContract(RunnerTestCase):
    """VIS0-C1B: the runner knows the evidence contract without claiming capture."""

    def metadata(self, manifest=None, execution=None):
        plan = self.build_plan(manifest)
        return plan, rb.build_run_metadata(
            plan, execution if execution is not None else {"exit_code": 0})

    def test_plan_declares_evidence_planned_not_produced(self):
        contract = self.build_plan()["capture_evidence_contract"]
        self.assertEqual(contract["kind"], "visual_capture_evidence")
        self.assertEqual(contract["schema_version"], "1.0.0")
        self.assertEqual(contract["status"], "planned")
        self.assertFalse(contract["produced"])
        self.assertTrue(contract["capture_backend_available"])
        self.assertTrue(contract["artifact_path"].endswith("capture_evidence.json"))

    def test_blocked_scene_declares_evidence_blocked(self):
        manifest = base_manifest()
        manifest["warmup"] = 5
        contract = self.build_plan(manifest)["capture_evidence_contract"]
        self.assertEqual(contract["status"], "blocked")
        self.assertFalse(contract["produced"])

    def test_contract_keeps_receipt_and_evidence_versions_apart(self):
        contract = self.build_plan()["capture_evidence_contract"]
        self.assertEqual(contract["schema_version"], "1.0.0")
        receipt = contract["runtime_receipt_contract"]
        self.assertEqual(receipt["kind"], "runtime_capture_receipt")
        self.assertEqual(receipt["schema_version"], "1.0.0")
        self.assertIn("never conflated", receipt["distinct_from_evidence"])
        self.assertEqual(self.build_plan()["schema_version"], "1.1.0")

    def test_contract_points_at_real_schema_and_validator_files(self):
        contract = self.build_plan()["capture_evidence_contract"]
        self.assertTrue((REPO_ROOT / contract["schema_path"]).exists(),
                        contract["schema_path"])
        self.assertTrue((REPO_ROOT / contract["validator"]).exists(),
                        contract["validator"])

    def test_handshake_split_is_disjoint_and_populated(self):
        handshake = self.build_plan()["capture_evidence_contract"]["handshake"]
        runtime = set(handshake["runtime_supplied_fields"])
        tooling = set(handshake["tooling_supplied_fields"])
        self.assertEqual(set(), runtime & tooling)
        self.assertTrue(runtime)
        self.assertTrue(tooling)

    def test_unexecuted_evidence_validates_against_the_evidence_contract(self):
        _, metadata = self.metadata()
        self.assertTrue(metadata["capture_evidence_validation"]["valid"],
                        metadata["capture_evidence_validation"]["errors"])
        self.assertEqual(metadata["capture_evidence_validation"]["error_count"], 0)

    def test_unexecuted_evidence_validates_through_the_standalone_validator(self):
        _, metadata = self.metadata()
        path = self.tmp / "capture_evidence.json"
        path.write_text(json.dumps(metadata["capture_evidence"]), encoding="utf-8")
        self.assertEqual(validate_capture_evidence(path), 0)

    def test_unexecuted_evidence_omits_no_contract_leaf(self):
        """Presence policy: the runner must emit every leaf, nulling not dropping.

        An omitted key would mean "incomplete artifact"; a null means "contract
        followed, value genuinely unavailable". An unexecuted document may only
        say the second, so it has to carry all 39 leaves explicitly.
        """
        _, metadata = self.metadata()
        evidence = metadata["capture_evidence"]
        for dotted in sorted(ALL_LEAF_FIELDS):
            with self.subTest(field=dotted):
                parts = dotted.split(".")
                node = evidence
                for part in parts[:-1]:
                    self.assertIsInstance(node, dict, dotted)
                    self.assertIn(part, node, f"{dotted}: container key omitted")
                    node = node[part]
                self.assertIsInstance(node, dict, dotted)
                self.assertIn(parts[-1], node, f"{dotted} is omitted, not null")

    def test_unexecuted_nulls_are_explicit_keys_not_absences(self):
        """A runtime-supplied leaf must be a present null, never a missing key."""
        _, metadata = self.metadata()
        evidence = metadata["capture_evidence"]
        self.assertIn("gpu_adapter_name", evidence["hardware"])
        self.assertIsNone(evidence["hardware"]["gpu_adapter_name"])
        self.assertIn("framebuffer_width", evidence["capture"]["actual"])
        self.assertIsNone(evidence["capture"]["actual"]["framebuffer_width"])
        self.assertIn("sha256", evidence["capture"]["image"])
        self.assertIsNone(evidence["capture"]["image"]["sha256"])
        self.assertIn("visual_pass", evidence["verdict"])
        self.assertIsNone(evidence["verdict"]["visual_pass"])

    def test_unexecuted_evidence_claims_no_capture(self):
        _, metadata = self.metadata()
        evidence = metadata["capture_evidence"]
        self.assertFalse(evidence["execution"]["capture_success"])
        for leaf in ("path", "sha256", "byte_size"):
            self.assertIsNone(evidence["capture"]["image"][leaf], leaf)
        for leaf in ("framebuffer_width", "framebuffer_height",
                     "presentation_frame_index"):
            self.assertIsNone(evidence["capture"]["actual"][leaf], leaf)

    def test_unexecuted_evidence_leaves_every_runtime_leaf_null(self):
        """Tooling must not fabricate a value only the runtime can know."""
        _, metadata = self.metadata()
        evidence = metadata["capture_evidence"]

        def resolve(dotted):
            node = evidence
            for part in dotted.split("."):
                node = node[part]
            return node

        for dotted in sorted(RUNTIME_SUPPLIED_FIELDS):
            with self.subTest(field=dotted):
                if dotted == "execution.capture_success":
                    self.assertIs(resolve(dotted), False)
                else:
                    self.assertIsNone(resolve(dotted))

    def test_unexecuted_evidence_keeps_visual_pass_null(self):
        _, metadata = self.metadata()
        self.assertIsNone(metadata["capture_evidence"]["verdict"]["visual_pass"])
        self.assertTrue(metadata["capture_evidence"]["verdict"]["visual_pass_reason"])

    def test_unexecuted_failure_reason_is_concrete_not_a_capability_gap(self):
        _, metadata = self.metadata(execution={"exit_code": 0})
        reason = metadata["capture_evidence"]["execution"]["failure_reason"]
        self.assertIn("not verified", reason)
        self.assertNotIn("NOT YET AVAILABLE", reason)

    def test_successful_execution_still_produces_no_visual_verdict(self):
        _, metadata = self.metadata(
            execution={"execution_success": True, "exit_code": 0})
        self.assertIsNone(metadata["verdict"]["visual_pass"])
        self.assertIsNone(metadata["capture_evidence"]["verdict"]["visual_pass"])
        self.assertFalse(metadata["capture_evidence"]["execution"]["capture_success"])

    def test_evidence_carries_manifest_and_git_provenance(self):
        plan, metadata = self.metadata()
        evidence = metadata["capture_evidence"]
        self.assertEqual(evidence["scene_id"], plan["scene_id"])
        self.assertEqual(evidence["manifest"]["sha256"], plan["manifest_sha256"])
        self.assertEqual(len(evidence["manifest"]["sha256"]), 64)
        self.assertEqual(evidence["source"]["commit_sha"], plan["git"]["commit_sha"])
        self.assertEqual(evidence["source"]["runner_name"], rb.RUNNER_NAME)
        self.assertEqual(evidence["source"]["runner_version"], rb.RUNNER_VERSION)

    def test_evidence_records_requested_values_from_the_manifest(self):
        _, metadata = self.metadata(airborne_manifest())
        evidence = metadata["capture_evidence"]
        self.assertEqual(evidence["capture"]["requested"]["width"], 1920)
        self.assertEqual(evidence["capture"]["requested"]["height"], 1080)
        self.assertEqual(evidence["capture"]["requested"]["frame_index"], 10)
        self.assertEqual(evidence["capture"]["format"], "png")
        self.assertEqual(evidence["renderer"]["version"], "v2")
        self.assertEqual(evidence["renderer"]["camera_mode"], "pilot")
        self.assertEqual(evidence["renderer"]["scenery_preset"], "flying-field")
        self.assertEqual(evidence["renderer"]["exposure_ev"], 0)

    def test_evidence_reports_hardware_it_cannot_know_as_null(self):
        _, metadata = self.metadata()
        hardware = metadata["capture_evidence"]["hardware"]
        self.assertIsNone(hardware["gpu_adapter_name"])
        self.assertIsNone(hardware["graphics_backend"])
        self.assertIsNone(hardware["driver_version"])
        self.assertTrue(hardware["notes"])
        self.assertIn("RuntimeCaptureReceipt 1.0.0 carries no adapter",
                      hardware["notes"])
        self.assertTrue(hardware["operating_system"])
        self.assertTrue(hardware["architecture"])

    def test_evidence_survives_json_round_trip(self):
        _, metadata = self.metadata()
        self.assertEqual(json.loads(json.dumps(metadata))["capture_evidence"],
                         metadata["capture_evidence"])

    def test_missing_git_provenance_is_reported_not_raised(self):
        """Without a commit SHA honest evidence is impossible; say so."""
        plan = self.build_plan()
        plan["git"] = dict(FAKE_GIT, commit_sha=None, available=False)
        validation = rb.validate_evidence_document(rb.build_capture_evidence(plan, {}))
        self.assertFalse(validation["valid"])
        self.assertTrue(any("source.commit_sha" in error
                            for error in validation["errors"]), validation["errors"])

    def test_capability_states_are_reported_for_the_c2a_runtime(self):
        _, metadata = self.metadata()
        capabilities = metadata["capabilities"]
        self.assertEqual(capabilities["capture_backend"]["status"], "supported")
        self.assertTrue(capabilities["capture_backend"]["produces_image"])
        self.assertEqual(capabilities["resolution_enforcement"]["status"], "supported")
        self.assertTrue(capabilities["resolution_enforcement"]["enforced"])
        self.assertEqual(capabilities["warmup_frames"]["status"], "derived")
        self.assertTrue(capabilities["warmup_frames"]["enforced"])
        self.assertEqual(capabilities["process_auto_exit"]["status"], "supported")

    def test_artifacts_report_no_capture_without_a_verification(self):
        _, metadata = self.metadata()
        self.assertIsNone(metadata["artifacts"]["capture"])
        self.assertFalse(metadata["artifacts"]["capture_verified"])
        self.assertTrue(metadata["artifacts"]["capture_reason"])


class TestConvergenceBehavior(RunnerTestCase):
    """Task 4: combined Line-1 + Line-2 convergence behavior."""

    def test_resolution_produces_correct_argv_flags(self):
        plan = self.build_plan()
        argv = plan["command_argv"]
        self.assertIn("--render-width", argv)
        self.assertIn("--render-height", argv)
        width_idx = argv.index("--render-width")
        height_idx = argv.index("--render-height")
        self.assertEqual(argv[width_idx + 1], "1920")
        self.assertEqual(argv[height_idx + 1], "1080")

    def test_resolution_flags_are_adjacent_and_emitted_together(self):
        plan = self.build_plan()
        argv = plan["command_argv"]
        width_idx = argv.index("--render-width")
        height_idx = argv.index("--render-height")
        # Both must be present; order is determined by CANONICAL_FIELD_ORDER.
        self.assertIsNotNone(width_idx)
        self.assertIsNotNone(height_idx)
        self.assertEqual(argv[width_idx + 1], "1920")
        self.assertEqual(argv[height_idx + 1], "1080")

    def test_capture_frame_produces_exit_after_frame(self):
        plan = self.build_plan()
        argv = plan["command_argv"]
        self.assertIn("--exit-after-frame", argv)
        idx = argv.index("--exit-after-frame")
        self.assertEqual(argv[idx + 1], "10")

    def test_process_auto_exit_is_supported(self):
        capabilities = self.build_plan()["runtime_capabilities"]
        self.assertEqual(capabilities["process_auto_exit"]["status"], "supported")

    def test_resolution_enforcement_is_supported(self):
        capabilities = self.build_plan()["runtime_capabilities"]
        self.assertEqual(capabilities["resolution_enforcement"]["status"], "supported")
        self.assertTrue(capabilities["resolution_enforcement"]["enforced"])

    def test_capture_backend_is_supported(self):
        capabilities = self.build_plan()["runtime_capabilities"]
        self.assertEqual(capabilities["capture_backend"]["status"], "supported")
        self.assertTrue(capabilities["capture_backend"]["produces_image"])

    def test_capture_success_is_false_without_an_execution(self):
        plan = self.build_plan()
        evidence = rb.build_capture_evidence(plan, {})
        self.assertFalse(evidence["execution"]["capture_success"])

    def test_actual_framebuffer_dimensions_are_null_without_a_receipt(self):
        plan = self.build_plan()
        evidence = rb.build_capture_evidence(plan, {})
        self.assertIsNone(evidence["capture"]["actual"]["framebuffer_width"])
        self.assertIsNone(evidence["capture"]["actual"]["framebuffer_height"])

    def test_actual_presentation_frame_index_is_null_without_a_receipt(self):
        plan = self.build_plan()
        evidence = rb.build_capture_evidence(plan, {})
        self.assertIsNone(evidence["capture"]["actual"]["presentation_frame_index"])

    def test_visual_pass_is_null(self):
        plan = self.build_plan()
        evidence = rb.build_capture_evidence(plan, {})
        self.assertIsNone(evidence["verdict"]["visual_pass"])

    def test_warmup_is_derived_and_never_a_runtime_flag(self):
        capabilities = self.build_plan()["runtime_capabilities"]
        warmup = capabilities["warmup_frames"]
        self.assertEqual(warmup["status"], "derived")
        self.assertIsNone(warmup["runtime_flag"])
        self.assertIn("no --warmup", warmup["reason"])
        self.assertNotIn("--warmup", self.build_plan()["command_argv"])

    def test_manifest_1_1_0_validates(self):
        validator = rb.ManifestValidator(base_manifest(), rb.REPO_ROOT)
        validator.validate()
        self.assertEqual(len(validator.errors), 0)

    def test_manifest_1_0_0_is_rejected(self):
        manifest = base_manifest()
        manifest["schema_version"] = "1.0.0"
        validator = rb.ManifestValidator(manifest, rb.REPO_ROOT)
        validator.validate()
        self.assertGreater(len(validator.errors), 0)
        self.assertTrue(any("1.0.0" in str(e) for e in validator.errors))

    def test_airborne_manifest_emits_altitude_and_airspeed(self):
        manifest = airborne_manifest()
        plan = self.build_plan(manifest)
        argv = " ".join(plan["command_argv"])
        self.assertIn("--altitude-m", argv)
        self.assertIn("--airspeed-mps", argv)

    def test_ground_manifest_does_not_emit_altitude_or_airspeed(self):
        manifest = base_manifest()
        plan = self.build_plan(manifest)
        argv = " ".join(plan["command_argv"])
        self.assertNotIn("--altitude-m", argv)
        self.assertNotIn("--airspeed-mps", argv)

    def test_no_invented_capability(self):
        capabilities = self.build_plan()["runtime_capabilities"]
        known_keys = {"capture_backend", "resolution_enforcement", "warmup_frames", "process_auto_exit"}
        self.assertEqual(set(capabilities.keys()), known_keys)

    def test_capture_image_fields_are_null(self):
        plan = self.build_plan()
        evidence = rb.build_capture_evidence(plan, {})
        self.assertIsNone(evidence["capture"]["image"]["path"])
        self.assertIsNone(evidence["capture"]["image"]["sha256"])
        self.assertIsNone(evidence["capture"]["image"]["byte_size"])

    def test_capture_evidence_contract_is_planned_not_produced(self):
        plan = self.build_plan()
        contract = plan["capture_evidence_contract"]
        self.assertEqual(contract["status"], "planned")
        self.assertFalse(contract["produced"])


class TestCaptureCliMapping(RunnerTestCase):
    """C2B tasks 1-2 and 17: the manifest really drives the C2A capture CLI."""

    def test_known_flags_include_the_real_capture_group(self):
        for flag in ("--capture-frame", "--capture-out", "--capture-format",
                     "--capture-receipt-out", "--visual-audit-out"):
            self.assertIn(flag, rb.KNOWN_RENDER_FLAGS, flag)

    def test_emittable_flags_include_the_real_capture_group(self):
        for flag in ("--capture-frame", "--capture-out", "--capture-format",
                     "--capture-receipt-out", "--visual-audit-out"):
            self.assertIn(flag, rb.EMITTABLE_FLAGS, flag)

    def test_capture_group_and_exit_after_frame_stay_separate(self):
        """--exit-after-frame is process lifecycle, not part of the capture group."""
        self.assertIn("--exit-after-frame", rb.EMITTABLE_FLAGS)
        self.assertNotIn("--exit-after-frame", rb.MANIFEST_CAPTURE_FLAGS)
        self.assertEqual(
            list(rb.MANIFEST_CAPTURE_FLAGS),
            ["--capture-out", "--capture-format", "--capture-frame"],
        )
        self.assertEqual(
            list(rb.DERIVED_CAPTURE_FLAGS),
            ["--capture-receipt-out", "--visual-audit-out", "--exit-after-frame"],
        )

    def test_reference_manifest_emits_capture_frame_ten(self):
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        plan = self.build_plan(manifest)
        self.assertEqual(self.argv_value(plan, "--capture-frame"), "10")

    def test_reference_manifest_emits_capture_out(self):
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        plan = self.build_plan(manifest)
        self.assertIsNotNone(self.argv_value(plan, "--capture-out"))

    def test_capture_output_lives_inside_the_scene_output_dir(self):
        plan = self.build_plan()
        scene_dir = Path(plan["scene_output_dir"])
        image = Path(self.argv_value(plan, "--capture-out"))
        self.assertEqual(image.parent, scene_dir)
        self.assertEqual(image.name, "test_scene_1920x1080.png")

    def test_capture_out_and_receipt_out_are_absolute(self):
        plan = self.build_plan()
        self.assertTrue(Path(self.argv_value(plan, "--capture-out")).is_absolute())
        self.assertTrue(Path(self.argv_value(plan, "--capture-receipt-out")).is_absolute())
        self.assertTrue(Path(self.argv_value(plan, "--visual-audit-out")).is_absolute())
        self.assertTrue(plan["capture_plan"]["paths_are_absolute"])

    def test_emits_capture_format_png(self):
        plan = self.build_plan()
        self.assertEqual(self.argv_value(plan, "--capture-format"), "png")

    def test_emits_runner_derived_capture_receipt_out(self):
        plan = self.build_plan()
        receipt = self.argv_value(plan, "--capture-receipt-out")
        self.assertEqual(Path(receipt).name, "runtime_capture_receipt.json")
        self.assertEqual(Path(receipt).parent, Path(plan["scene_output_dir"]))

    def test_emits_runner_derived_visual_audit_out(self):
        plan = self.build_plan()
        audit = self.argv_value(plan, "--visual-audit-out")
        self.assertEqual(Path(audit).name, "runtime_visual_audit.json")
        self.assertEqual(Path(audit).parent, Path(plan["scene_output_dir"]))

    def test_emits_runner_derived_exit_after_frame(self):
        plan = self.build_plan()
        self.assertEqual(self.argv_value(plan, "--exit-after-frame"), "10")

    def test_derived_flags_are_labelled_runner_derived_in_the_plan(self):
        """Provenance must not attribute them to manifest fields that do not exist."""
        derived = self.build_plan()["capture_plan"]["derived_arguments"]
        by_flag = {item["flag"]: item for item in derived}
        self.assertEqual(set(by_flag), set(rb.DERIVED_CAPTURE_FLAGS))
        for flag, item in by_flag.items():
            self.assertEqual(item["provenance"], "runner-derived", flag)
            self.assertTrue(item["derived_from"], flag)
        self.assertIsNone(by_flag["--capture-receipt-out"]["manifest_field"])
        self.assertIsNone(by_flag["--visual-audit-out"]["manifest_field"])
        self.assertEqual(by_flag["--exit-after-frame"]["manifest_field"], "capture.frame")

    def test_manifest_driven_flags_are_labelled_manifest_in_the_plan(self):
        manifest_arguments = self.build_plan()["capture_plan"]["manifest_arguments"]
        by_flag = {item["flag"]: item for item in manifest_arguments}
        self.assertEqual(set(by_flag), set(rb.MANIFEST_CAPTURE_FLAGS))
        for flag, item in by_flag.items():
            self.assertEqual(item["provenance"], "manifest", flag)
            self.assertIn(item["manifest_field"], rb.CANONICAL_FIELD_ORDER, flag)

    def test_argv_is_deterministic_across_rebuilds(self):
        first = self.build_plan()["command_argv"]
        second = self.build_plan()["command_argv"]
        self.assertEqual(first, second)

    def test_argv_is_deterministic_across_manifest_key_order(self):
        manifest = base_manifest()
        reordered = {key: manifest[key] for key in reversed(list(manifest))}
        self.assertEqual(self.build_plan(manifest)["command_argv"],
                         self.build_plan(reordered)["command_argv"])

    def test_manifest_driven_flags_precede_derived_flags(self):
        argv = self.build_plan()["command_argv"]
        positions = {flag: argv.index(flag) for flag in
                     ("--capture-out", "--capture-format", "--capture-frame",
                      "--capture-receipt-out", "--exit-after-frame")}
        self.assertLess(positions["--capture-frame"], positions["--capture-receipt-out"])
        self.assertLess(positions["--capture-out"], positions["--capture-receipt-out"])
        self.assertLess(positions["--capture-format"], positions["--exit-after-frame"])

    def test_exit_after_frame_is_never_before_capture_frame(self):
        """rcsim-app rejects ExitBeforeCaptureFrame, so the plan must not either."""
        for frame in (0, 1, 10, 42):
            manifest = small_manifest(frame=frame, warmup=frame)
            plan = self.build_plan(manifest)
            self.assertEqual(self.argv_value(plan, "--capture-frame"), str(frame))
            self.assertEqual(self.argv_value(plan, "--exit-after-frame"), str(frame))

    def test_no_shell_is_used_for_execution(self):
        source = RUNNER_SOURCE.read_text(encoding="utf-8")
        self.assertIn("shell=False", source)
        self.assertNotIn("shell=True", source)
        self.assertNotIn("os.system(", source)
        self.assertNotIn("os.popen(", source)
        self.assertNotIn("subprocess.call(", source)
        plan = self.build_plan()
        self.assertFalse(plan["execution_policy"]["shell"])

    def test_canonical_reference_argv(self):
        """Task 17: the exact planned command for the reference manifest."""
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        plan = self.build_plan(manifest, app="target/release/rcsim-app")
        scene_dir = Path(plan["scene_output_dir"])
        self.assertEqual(plan["command_argv"], [
            "target/release/rcsim-app",
            "render",
            "--renderer", "v2",
            "--terrain-debug", "final",
            "--vegetation-debug", "final",
            "--scenery", "flying-field",
            "--camera", "pilot",
            "--camera-fov", "55",
            "--pilot-position", "0.0,1.8,20.0",
            "--exposure-ev", "0",
            "--model", "models/acro_electric_01/model.json",
            "--throttle", "0.0",
            "--start-on-ground",
            "--render-width", "1920",
            "--render-height", "1080",
            "--capture-out", str(scene_dir / "aircraft_acro_static_front_1920x1080.png"),
            "--capture-format", "png",
            "--capture-frame", "10",
            "--capture-receipt-out", str(scene_dir / "runtime_capture_receipt.json"),
            "--visual-audit-out", str(scene_dir / "runtime_visual_audit.json"),
            "--exit-after-frame", "10",
        ])

    def test_command_display_round_trips_to_the_same_argv(self):
        plan = self.build_plan()
        self.assertEqual(shlex.split(plan["command_display"]), plan["command_argv"])


class TestCaptureExecutabilityPolicy(RunnerTestCase):
    """C2B tasks 3-4: manifest valid is not the same as runtime executable."""

    def refuse_to_start(self, manifest):
        """Run --execute with a tripwire launcher. Returns (code, stderr)."""
        script = self.write_fake_runtime()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))

        def invoke(argv, cwd=None, timeout_seconds=None):
            self.fail(f"a process was started for a non-executable manifest: {argv}")

        self.patch("run_process", invoke)
        path = self.write_manifest(manifest)
        _code, _out, err = self.run_main([
            "--manifest", str(path), "--execute", "--app", str(script),
            "--output-dir", str(self.tmp / "out"),
        ])
        return _code, err

    def test_matching_warmup_and_frame_is_executable(self):
        plan = self.build_plan(small_manifest(frame=10, warmup=10))
        self.assertTrue(plan["capture_plan"]["executable"])
        self.assertEqual(plan["capture_plan"]["blocking_reasons"], [])
        self.assertTrue(plan["capture_plan"]["warmup_matches_capture_frame"])

    def test_warmup_zero_and_frame_zero_is_executable(self):
        plan = self.build_plan(small_manifest(frame=0, warmup=0))
        self.assertTrue(plan["capture_plan"]["executable"])
        self.assertEqual(self.argv_value(plan, "--capture-frame"), "0")
        self.assertEqual(self.argv_value(plan, "--exit-after-frame"), "0")

    def test_warmup_mismatch_is_not_executable(self):
        plan = self.build_plan(small_manifest(frame=10, warmup=12))
        self.assertFalse(plan["capture_plan"]["executable"])
        self.assertFalse(plan["capture_plan"]["warmup_matches_capture_frame"])
        self.assertIn("C2B requires explicit capture.frame matching warmup",
                      plan["capture_plan"]["blocking_reason"])

    def test_warmup_mismatch_fails_closed_on_execute(self):
        code, err = self.refuse_to_start(small_manifest(frame=10, warmup=12))
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("C2B requires explicit capture.frame matching warmup", err)
        self.assertNotIn("Traceback", err)

    def test_warmup_mismatch_emits_no_capture_flag_in_the_plan(self):
        argv = self.build_plan(small_manifest(frame=10, warmup=12))["command_argv"]
        for flag in ("--capture-frame", "--capture-out", "--capture-format",
                     "--capture-receipt-out", "--exit-after-frame"):
            self.assertNotIn(flag, argv)

    def test_missing_capture_frame_is_still_a_valid_manifest(self):
        """The GoldenSceneManifest schema is NOT narrowed by this tranche."""
        manifest = small_manifest(frame=None)
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])

    def test_missing_capture_frame_is_not_executable(self):
        plan = self.build_plan(small_manifest(frame=None))
        self.assertFalse(plan["capture_plan"]["executable"])
        self.assertIsNone(plan["capture_plan"]["warmup_matches_capture_frame"])
        self.assertIn("C2B requires explicit capture.frame matching warmup",
                      plan["capture_plan"]["blocking_reason"])

    def test_missing_capture_frame_fails_closed_on_execute(self):
        code, err = self.refuse_to_start(small_manifest(frame=None))
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("C2B requires explicit capture.frame matching warmup", err)

    def test_dry_run_of_a_blocked_scene_still_succeeds_and_explains(self):
        """Dry run reports the block instead of raising, so it stays inspectable."""
        path = self.write_manifest(small_manifest(frame=10, warmup=12))
        code, out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)
        self.assertIn("executable             False", out)
        self.assertIn("C2B requires explicit capture.frame matching warmup", out)

    def test_png_is_executable(self):
        plan = self.build_plan(small_manifest(image_format="png"))
        self.assertTrue(plan["capture_plan"]["executable"])
        self.assertTrue(
            plan["runtime_capabilities"]["capture_backend"]["requested_format_executable"]
        )

    def test_jpg_is_valid_but_not_executable(self):
        manifest = small_manifest(image_format="jpg")
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])
        plan = self.build_plan(manifest)
        self.assertFalse(plan["capture_plan"]["executable"])
        self.assertIn("C2B supports png only", plan["capture_plan"]["blocking_reason"])
        self.assertFalse(
            plan["runtime_capabilities"]["capture_backend"]["requested_format_executable"]
        )

    def test_jpg_fails_closed_on_execute(self):
        code, err = self.refuse_to_start(small_manifest(image_format="jpg"))
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("C2B supports png only", err)

    def test_exr_is_valid_but_not_executable(self):
        manifest = small_manifest(image_format="exr")
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])
        plan = self.build_plan(manifest)
        self.assertFalse(plan["capture_plan"]["executable"])
        self.assertIn("C2B supports png only", plan["capture_plan"]["blocking_reason"])

    def test_exr_fails_closed_on_execute(self):
        code, err = self.refuse_to_start(small_manifest(image_format="exr"))
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("C2B supports png only", err)

    def test_quality_is_unsupported_and_not_silently_ignored(self):
        manifest = small_manifest(image_format="jpg", quality=90)
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])
        plan = self.build_plan(manifest)
        self.assertFalse(plan["capture_plan"]["executable"])
        self.assertEqual(plan["capture_plan"]["quality"], 90)
        joined = " ".join(plan["capture_plan"]["blocking_reasons"])
        self.assertIn("capture.quality=90 is unsupported", joined)
        self.assertIn("not silently dropped", joined)

    def test_quality_field_policy_is_unsupported(self):
        policy = rb.FIELD_POLICY["capture.quality"]
        self.assertEqual(policy.status, rb.FIELD_STATUS_UNSUPPORTED)
        self.assertIn("unsupported", policy.reason)

    def test_quality_blocks_execution(self):
        code, err = self.refuse_to_start(small_manifest(image_format="jpg", quality=90))
        self.assertEqual(code, rb.EXIT_VALIDATION_FAILED)
        self.assertIn("capture.quality=90", err)

    def test_executability_is_recorded_in_the_execution_policy(self):
        plan = self.build_plan()
        self.assertTrue(plan["execution_policy"]["capture_executable"])
        self.assertFalse(plan["execution_policy"]["process_exit_zero_is_sufficient"])
        self.assertEqual(len(plan["execution_policy"]["success_requires"]), 5)

    def test_enforce_capture_executability_raises_for_a_blocked_plan(self):
        plan = self.build_plan(small_manifest(frame=10, warmup=3))
        with self.assertRaises(rb.RunnerError) as caught:
            rb.enforce_capture_executability(plan["capture_plan"])
        self.assertEqual(caught.exception.exit_code, rb.EXIT_VALIDATION_FAILED)

    def test_enforce_capture_executability_accepts_an_executable_plan(self):
        plan = self.build_plan()
        self.assertIsNone(rb.enforce_capture_executability(plan["capture_plan"]))


class TestDryRunSideEffects(RunnerTestCase):
    """C2B task 16: the default stays dry and honestly claims nothing."""

    def test_dry_run_produces_zero_image_side_effects(self):
        output = self.tmp / "out"
        path = self.write_manifest(small_manifest())
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(output)]
        )
        self.assertEqual(code, rb.EXIT_OK)
        self.assertFalse(output.exists())
        self.assertEqual(list(self.tmp.rglob("*.png")), [])
        self.assertEqual(list(self.tmp.rglob("runtime_capture_receipt.json")), [])
        self.assertEqual(list(self.tmp.rglob("runtime_visual_audit.json")), [])
        self.assertEqual(list(self.tmp.rglob("capture_evidence.json")), [])

    def test_dry_run_starts_no_process(self):
        path = self.write_manifest()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))

        def invoke(argv, cwd=None, timeout_seconds=None):
            if argv and argv[0] != "git":
                self.fail(f"dry run started a process: {argv}")
            return rb.ProcessOutcome(argv=list(argv), returncode=0, stdout="",
                                     started_at_utc="t0", ended_at_utc="t1")

        self.patch("run_process", invoke)
        code, _out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)

    def test_dry_run_shows_every_planned_path_and_the_derived_args(self):
        path = self.write_manifest(small_manifest())
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        _code, out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        scene = self.scene_dir()
        for fragment in (str(scene / "test_scene_320x240.png"),
                         str(scene / "runtime_capture_receipt.json"),
                         str(scene / "capture_evidence.json"),
                         "--capture-receipt-out", "--exit-after-frame",
                         "runner-derived"):
            self.assertIn(fragment, out)

    def test_dry_run_declares_supported_backend_but_no_capture_produced(self):
        path = self.write_manifest(small_manifest())
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        _code, out, _err = self.run_main(
            ["--manifest", str(path), "--dry-run", "--output-dir", str(self.tmp / "out")]
        )
        self.assertIn("backend                SUPPORTED (produces_image=true)", out)
        self.assertIn("capture produced       false (nothing was executed)", out)
        self.assertIn("visual_pass            null", out)

    def test_dry_run_does_not_invent_actual_values(self):
        path = self.write_manifest(small_manifest())
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        plan_path = self.tmp / "plan.json"
        self.run_main(["--manifest", str(path), "--dry-run",
                       "--output-dir", str(self.tmp / "out"),
                       "--plan-json", str(plan_path)])
        plan = self.read_json(plan_path)
        self.assertIsNone(plan["runtime_capabilities"]["capture_backend"]["capture_produced"])
        self.assertIsNone(plan["expected_output_basename"]["verified"])

    def test_default_remains_dry_run(self):
        path = self.write_manifest(small_manifest())
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        code, out, _err = self.run_main(
            ["--manifest", str(path), "--output-dir", str(self.tmp / "out")]
        )
        self.assertEqual(code, rb.EXIT_OK)
        self.assertIn("DRY-RUN", out)


class TestStaleArtifactSafety(RunnerTestCase):
    """C2B task 6: a previous run's image must never survive into this one."""

    def plant_stale_artifacts(self, plan):
        scene = Path(plan["scene_output_dir"])
        scene.mkdir(parents=True, exist_ok=True)
        capture_plan = plan["capture_plan"]
        planted = []
        for raw in (capture_plan["image_path"], capture_plan["receipt_path"],
                    capture_plan["audit_path"],
                    capture_plan["evidence_path"]):
            path = Path(raw)
            path.write_bytes(b"stale from a previous run")
            planted.append(path)
            temporary = path.with_name(path.name + ".tmp")
            temporary.write_bytes(b"stale temporary sibling")
            planted.append(temporary)
        return scene, planted

    def test_stale_image_receipt_and_evidence_are_removed_before_execute(self):
        plan = self.synthetic_plan([sys.executable, "-c", "raise SystemExit(3)"],
                                   manifest=small_manifest())
        _scene, planted = self.plant_stale_artifacts(plan)
        for path in planted:
            self.assertTrue(path.exists(), path)
        execution = rb.execute_plan(plan, sys.executable, 30)
        for path in planted:
            self.assertFalse(path.exists(), f"{path} survived the cleanup")
        self.assertEqual(sorted(execution["stale_artifacts_removed"]),
                         sorted(str(path) for path in planted))

    def test_stale_removal_happens_even_when_the_process_never_starts(self):
        plan = self.synthetic_plan([sys.executable, "-c", "raise SystemExit(3)"],
                                   manifest=small_manifest())
        _scene, planted = self.plant_stale_artifacts(plan)

        def invoke(argv, cwd=None, timeout_seconds=None):
            return rb.ProcessOutcome(
                argv=list(argv), returncode=None, failure_kind="executable_not_found",
                failure_message="simulated", started_at_utc="t0", ended_at_utc="t1")

        rb.execute_plan(plan, sys.executable, 30, invoke=invoke)
        for path in planted:
            self.assertFalse(path.exists(), path)

    def test_stale_removal_leaves_unrelated_files_alone(self):
        plan = self.synthetic_plan([sys.executable, "-c", "raise SystemExit(3)"],
                                   manifest=small_manifest())
        scene, _planted = self.plant_stale_artifacts(plan)
        keep = scene / "approved_baseline.png"
        keep.write_bytes(b"must survive")
        outside = self.tmp / "approved_baseline.png"
        outside.write_bytes(b"must survive too")
        rb.execute_plan(plan, sys.executable, 30)
        self.assertTrue(keep.exists())
        self.assertTrue(outside.exists())

    def test_stale_removal_is_skipped_when_nothing_is_stale(self):
        plan = self.synthetic_plan([sys.executable, "-c", "print('ok')"],
                                   manifest=small_manifest())
        execution = rb.execute_plan(plan, sys.executable, 30)
        self.assertEqual(execution["stale_artifacts_removed"], [])

    def test_stale_removal_fails_closed_when_a_file_cannot_be_removed(self):
        plan = self.synthetic_plan([sys.executable, "-c", "print('ok')"],
                                   manifest=small_manifest())
        scene, _planted = self.plant_stale_artifacts(plan)
        image = Path(plan["capture_plan"]["image_path"])

        def refuse(_self, *args, **kwargs):
            raise PermissionError("simulated lock")

        original = Path.unlink
        self.addCleanup(setattr, Path, "unlink", original)
        setattr(Path, "unlink", refuse)
        with self.assertRaises(rb.RunnerError) as caught:
            rb.execute_plan(plan, sys.executable, 30)
        self.assertEqual(caught.exception.exit_code, rb.EXIT_EXECUTION_FAILED)
        self.assertIn("stale capture artifact", caught.exception.message)
        self.assertTrue(image.exists())
        self.assertTrue(scene.is_dir())

    def test_a_second_execute_overwrites_rather_than_reuses_the_first_image(self):
        """End-to-end: the image on disk after run 2 is the one run 2 wrote."""
        first = self.write_fake_runtime(name="first.py", WIDTH=320, HEIGHT=240)
        code, _out, _err, out = self.execute_with_fake_runtime(
            manifest=small_manifest(), runtime=first)
        self.assertEqual(code, rb.EXIT_OK)
        image = Path(self.argv_value(
            self.plan_for(small_manifest(), output_dir=out), "--capture-out"))
        first_bytes = image.read_bytes()

        second = self.write_fake_runtime(name="second.py", WIDTH=400, HEIGHT=300)
        code, _out, _err, _out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), runtime=second, output_dir=out)
        self.assertEqual(code, rb.EXIT_OK)
        self.assertNotEqual(image.read_bytes(), first_bytes)

    def test_stale_policy_is_documented_in_the_plan(self):
        plan = self.build_plan()
        policy = plan["capture_plan"]["stale_artifact_policy"]
        self.assertIn("removes a pre-existing capture image", policy)
        self.assertIn("crashing before its cleanup ran", policy)
        self.assertIn("approved baselines are never deleted", policy)


class TestRuntimeCaptureReceiptParsing(RunnerTestCase):
    """C2B tasks 9-10: a strict, fail-closed RuntimeCaptureReceipt 1.0.0 reader."""

    def valid_receipt(self, directory=None, **overrides):
        directory = directory or self.tmp
        png = make_png(4, 3)
        image_path = directory / "capture.png"
        image_path.write_bytes(png)
        payload = receipt_for(image_path, png, 4, 3, frame=10)
        payload.update(overrides)
        return payload, image_path, png

    def parse(self, payload):
        return rcr.parse_receipt(payload)

    def test_valid_receipt_is_accepted(self):
        payload, image_path, png = self.valid_receipt()
        receipt, errors = self.parse(payload)
        self.assertEqual(errors, [])
        self.assertIsNotNone(receipt)
        self.assertEqual(receipt.schema_version, "1.0.0")
        self.assertEqual(receipt.presentation_frame_index, 10)
        self.assertEqual(receipt.framebuffer_width, 4)
        self.assertEqual(receipt.framebuffer_height, 3)
        self.assertEqual(receipt.format, "png")
        self.assertEqual(receipt.image_path, str(image_path))
        self.assertEqual(receipt.image_sha256, hashlib.sha256(png).hexdigest())
        self.assertEqual(receipt.image_byte_size, len(png))

    def test_receipt_field_set_matches_the_runtime_struct_exactly(self):
        if not RENDER_APP_SOURCE.exists():
            self.skipTest("runtime source not available")
        text = RENDER_APP_SOURCE.read_text(encoding="utf-8")
        start = text.index("struct RuntimeCaptureReceipt {")
        body = text[start:text.index("}", start)]
        for field in rcr.RECEIPT_FIELD_ORDER:
            self.assertIn(f"{field}:", body, field)
        self.assertEqual(len(rcr.RECEIPT_REQUIRED_FIELDS), 8)

    def test_every_missing_field_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        for field in rcr.RECEIPT_FIELD_ORDER:
            broken = dict(payload)
            del broken[field]
            receipt, errors = self.parse(broken)
            self.assertIsNone(receipt, field)
            self.assertTrue(any("missing required field" in e for e in errors), field)

    def test_unknown_field_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["gpu_adapter_name"] = "NVIDIA GeForce"
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertTrue(any("unknown field" in e for e in errors), errors)

    def test_every_wrong_schema_version_is_rejected(self):
        for version in ("0.9.0", "1.0.1", "1.1.0", "2.0.0", "1.0", "v1.0.0"):
            payload, _image, _png = self.valid_receipt(schema_version=version)
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, version)
            self.assertTrue(any("schema_version" in e for e in errors), version)

    def test_negative_frame_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["presentation_frame_index"] = -1
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertTrue(any("presentation_frame_index" in e for e in errors), errors)

    def test_non_integer_frame_is_rejected(self):
        for frame in (10.0, "10", None, True):
            payload, _image, _png = self.valid_receipt()
            payload["presentation_frame_index"] = frame
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, repr(frame))
            self.assertTrue(
                any("presentation_frame_index" in e for e in errors), repr(frame))

    def test_zero_width_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["framebuffer_width"] = 0
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertTrue(any("framebuffer_width" in e for e in errors), errors)

    def test_zero_height_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["framebuffer_height"] = 0
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertTrue(any("framebuffer_height" in e for e in errors), errors)

    def test_negative_extent_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["framebuffer_width"] = -320
        payload["framebuffer_height"] = -240
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertEqual(len(errors), 2, errors)

    def test_non_png_format_is_rejected(self):
        for image_format in ("jpg", "jpeg", "exr", "PNG", ""):
            payload, _image, _png = self.valid_receipt()
            payload["format"] = image_format
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, image_format)
            self.assertTrue(any("format" in e for e in errors), image_format)

    def test_malformed_sha_is_rejected(self):
        for digest in ("z" * 64, "A" * 64, "abcd", "", "0" * 63, "0" * 65):
            payload, _image, _png = self.valid_receipt()
            payload["image_sha256"] = digest
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, digest)
            self.assertTrue(any("image_sha256" in e for e in errors), digest)

    def test_zero_byte_size_is_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["image_byte_size"] = 0
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertTrue(any("image_byte_size" in e for e in errors), errors)

    def test_empty_image_path_is_rejected(self):
        for path in ("", "   "):
            payload, _image, _png = self.valid_receipt()
            payload["image_path"] = path
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, repr(path))
            self.assertTrue(any("image_path" in e for e in errors), repr(path))

    def test_wrong_types_are_rejected(self):
        payload, _image, _png = self.valid_receipt()
        payload["framebuffer_width"] = "4"
        payload["image_byte_size"] = [1]
        payload["schema_version"] = 1.0
        receipt, errors = self.parse(payload)
        self.assertIsNone(receipt)
        self.assertEqual(len(errors), 3, errors)

    def test_non_object_root_is_rejected(self):
        for payload in ([], "receipt", 42, None, True):
            receipt, errors = self.parse(payload)
            self.assertIsNone(receipt, repr(payload))
            self.assertTrue(errors)

    def test_malformed_json_file_is_rejected(self):
        path = self.tmp / "runtime_capture_receipt.json"
        path.write_text("{ not json", encoding="utf-8")
        receipt, errors = rcr.load_receipt(path)
        self.assertIsNone(receipt)
        self.assertTrue(any("not valid JSON" in e for e in errors), errors)

    def test_missing_receipt_file_is_rejected(self):
        receipt, errors = rcr.load_receipt(self.tmp / "absent.json")
        self.assertIsNone(receipt)
        self.assertTrue(any("not found" in e for e in errors), errors)

    def test_receipt_round_trips_through_json(self):
        payload, _image, _png = self.valid_receipt()
        path = self.tmp / "runtime_capture_receipt.json"
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        receipt, errors = rcr.load_receipt(path)
        self.assertEqual(errors, [])
        self.assertEqual(receipt.to_json(), payload)


class TestReceiptExpectationCheck(RunnerTestCase):
    """C2B task 10: a receipt is trusted only if it matches the request plan."""

    def receipt(self, **overrides):
        png = make_png(4, 3)
        image_path = self.tmp / "capture.png"
        image_path.write_bytes(png)
        payload = receipt_for(image_path, png, 4, 3, frame=10)
        payload.update(overrides)
        receipt, errors = rcr.parse_receipt(payload)
        self.assertEqual(errors, [])
        return receipt, str(image_path)

    def expect(self, receipt, image_path, frame=10, image_format="png"):
        return rcr.check_receipt_expectations(
            receipt,
            expected_frame_index=frame,
            expected_format=image_format,
            expected_image_path=image_path,
        )

    def test_matching_receipt_has_no_expectation_errors(self):
        receipt, image_path = self.receipt()
        self.assertEqual(self.expect(receipt, image_path), [])

    def test_frame_mismatch_is_rejected(self):
        receipt, image_path = self.receipt()
        errors = self.expect(receipt, image_path, frame=11)
        self.assertEqual(len(errors), 1)
        self.assertIn("presentation_frame_index", errors[0])
        self.assertIn("receipt declares 10", errors[0])

    def test_format_mismatch_is_rejected(self):
        receipt, image_path = self.receipt()
        errors = self.expect(receipt, image_path, image_format="jpg")
        self.assertEqual(len(errors), 1)
        self.assertIn("format", errors[0])

    def test_image_path_mismatch_is_rejected(self):
        receipt, _image_path = self.receipt()
        errors = self.expect(receipt, str(self.tmp / "somewhere_else.png"))
        self.assertEqual(len(errors), 1)
        self.assertIn("image_path", errors[0])

    def test_image_path_comparison_is_canonical(self):
        """Separator and case spelling must not create a false mismatch."""
        receipt, image_path = self.receipt()
        for variant in (image_path.replace("\\", "/"), image_path.upper(),
                        str(Path(image_path).parent / "." / Path(image_path).name)):
            with self.subTest(variant=variant):
                self.assertEqual(self.expect(receipt, variant), [])

    def test_relative_receipt_path_is_canonicalised_against_a_base(self):
        png = make_png(2, 2)
        (self.tmp / "capture.png").write_bytes(png)
        payload = receipt_for("capture.png", png, 2, 2, frame=0)
        receipt, errors = rcr.parse_receipt(payload)
        self.assertEqual(errors, [])
        self.assertEqual(
            rcr.check_receipt_expectations(
                receipt, expected_frame_index=0, expected_format="png",
                expected_image_path=str(self.tmp / "capture.png"), base=self.tmp),
            [])

    def test_extent_is_deliberately_not_compared_against_the_request(self):
        """requested != actual is a fact to record, not an error to repair."""
        receipt, image_path = self.receipt()
        self.assertEqual(self.expect(receipt, image_path), [])
        self.assertEqual(receipt.framebuffer_width, 4)
        self.assertNotEqual(receipt.framebuffer_width, 1920)

    def test_absent_expected_frame_is_reported_not_silently_accepted(self):
        receipt, image_path = self.receipt()
        errors = self.expect(receipt, image_path, frame=None)
        self.assertEqual(len(errors), 1)
        self.assertIn("no explicit capture frame index", errors[0])


class TestPngVerification(RunnerTestCase):
    """C2B task 11: the PNG is re-measured, never taken on the receipt's word."""

    def write_pair(self, png=None, width=4, height=3, frame=10, receipt_overrides=None,
                   name="capture.png"):
        png = make_png(width, height) if png is None else png
        image_path = self.tmp / name
        image_path.write_bytes(png)
        payload = receipt_for(image_path, png, width, height, frame=frame)
        payload.update(receipt_overrides or {})
        receipt, errors = rcr.parse_receipt(payload)
        self.assertEqual(errors, [], errors)
        return image_path, receipt, png

    def test_valid_runtime_style_rgba8_png_is_accepted(self):
        image_path, receipt, png = self.write_pair()
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertEqual(errors, [])
        self.assertIsNotNone(image)
        self.assertEqual(image.width, 4)
        self.assertEqual(image.height, 3)
        self.assertEqual(image.bit_depth, 8)
        self.assertEqual(image.color_type, 6)
        self.assertEqual(image.byte_size, len(png))
        self.assertEqual(image.sha256, hashlib.sha256(png).hexdigest())

    def test_a_real_320x240_capture_is_accepted(self):
        image_path, receipt, _png = self.write_pair(width=320, height=240)
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertEqual(errors, [])
        self.assertEqual((image.width, image.height), (320, 240))

    def test_missing_image_is_rejected(self):
        _image_path, receipt, _png = self.write_pair()
        image, errors = rcr.verify_captured_png(self.tmp / "absent.png", receipt)
        self.assertIsNone(image)
        self.assertTrue(any("not found" in e for e in errors), errors)

    def test_a_directory_is_not_a_regular_file(self):
        _image_path, receipt, _png = self.write_pair()
        directory = self.tmp / "capture.png.d"
        directory.mkdir()
        image, errors = rcr.verify_captured_png(directory, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("not a regular file" in e for e in errors), errors)

    def test_byte_size_mismatch_is_rejected(self):
        image_path, receipt, png = self.write_pair()
        image_path.write_bytes(png + b"\x00")
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("byte size" in e for e in errors), errors)

    def test_sha_mismatch_is_rejected(self):
        image_path, receipt, png = self.write_pair()
        # Flip one byte so the digest changes while the length does not: this
        # isolates the SHA check from the byte-size check.
        same_length = png[:-1] + bytes([(png[-1] + 1) % 256])
        self.assertEqual(len(same_length), len(png))
        image_path.write_bytes(same_length)
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("sha256" in e for e in errors), errors)
        self.assertFalse(any("byte size" in e for e in errors), errors)

    def test_bad_png_signature_is_rejected(self):
        png = b"\x89PNX\r\n\x1a\n" + make_png(4, 3)[8:]
        image_path, receipt, _png = self.write_pair(png=png)
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("invalid PNG signature" in e for e in errors), errors)

    def test_missing_ihdr_chunk_is_rejected(self):
        png = make_png(4, 3)
        broken = png[:12] + b"IDAT" + png[16:]
        image_path, receipt, _png = self.write_pair(png=broken)
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("IHDR" in e for e in errors), errors)

    def test_truncated_file_is_rejected(self):
        image_path, receipt, png = self.write_pair()
        image_path.write_bytes(png[:20])
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(errors)

    def test_ihdr_width_mismatch_is_rejected(self):
        png = make_png(5, 3)
        image_path = self.tmp / "capture.png"
        image_path.write_bytes(png)
        # The receipt declares the bytes it really wrote, but a different extent.
        payload = receipt_for(image_path, png, 4, 3)
        receipt, errors = rcr.parse_receipt(payload)
        self.assertEqual(errors, [])
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("image width" in e for e in errors), errors)

    def test_ihdr_height_mismatch_is_rejected(self):
        png = make_png(4, 5)
        image_path = self.tmp / "capture.png"
        image_path.write_bytes(png)
        payload = receipt_for(image_path, png, 4, 3)
        receipt, errors = rcr.parse_receipt(payload)
        self.assertEqual(errors, [])
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("image height" in e for e in errors), errors)

    def test_non_eight_bit_png_is_rejected(self):
        image_path, receipt, _png = self.write_pair(
            png=make_png(4, 3, bit_depth=16))
        image, errors = rcr.verify_captured_png(image_path, receipt)
        self.assertIsNone(image)
        self.assertTrue(any("bit depth is 16" in e for e in errors), errors)

    def test_non_rgba_png_is_rejected(self):
        for color_type, label in ((0, "greyscale"), (2, "truecolour"),
                                  (3, "palette"), (4, "greyscale+alpha")):
            image_path, receipt, _png = self.write_pair(
                png=make_png(4, 3, color_type=color_type), name=f"c{color_type}.png")
            image, errors = rcr.verify_captured_png(image_path, receipt)
            self.assertIsNone(image, label)
            self.assertTrue(
                any(f"colour type is {color_type}" in e for e in errors), label)

    def test_verification_reads_the_header_only_and_never_decodes_pixels(self):
        """No decompression, no metric, no baseline comparison anywhere."""
        source = (RUNNER_SOURCE.with_name("runtime_capture_receipt.py")
                  .read_text(encoding="utf-8"))
        for forbidden in ("import zlib", "decompress", "baseline", "ssim", "psnr",
                          "mean_squared", "histogram"):
            self.assertNotIn(forbidden, source, forbidden)
        # Only the fixed 33-byte prefix is needed to establish every PNG fact.
        self.assertEqual(rcr.PNG_MINIMUM_LENGTH, 33)


class TestEvidenceFromRuntimeReceipt(RunnerTestCase):
    """C2B tasks 12-15 and 18: actual comes from the receipt, never the manifest."""

    def evidence_for(self, receipt, image, manifest=None):
        plan = self.build_plan(manifest or small_manifest())
        execution = {"execution_success": True, "exit_code": 0, "failure_kind": None,
                     "artifacts_written": []}
        verification = self.trusted_verification(receipt, image)
        return plan, rb.build_capture_evidence(plan, execution, verification)

    def real_pair(self, width=320, height=240, frame=10):
        png = make_png(width, height)
        image_path = self.tmp / "out" / "test_scene" / f"test_scene_{width}x{height}.png"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        image_path.write_bytes(png)
        receipt = receipt_for(image_path, png, width, height, frame=frame)
        image = {
            "path": str(image_path),
            "byte_size": len(png),
            "sha256": hashlib.sha256(png).hexdigest(),
            "width": width,
            "height": height,
            "bit_depth": 8,
            "color_type": 6,
        }
        return receipt, image

    def test_successful_run_populates_actual_from_the_receipt(self):
        receipt, image = self.real_pair()
        _plan, evidence = self.evidence_for(receipt, image)
        actual = evidence["capture"]["actual"]
        self.assertEqual(actual["framebuffer_width"], receipt["framebuffer_width"])
        self.assertEqual(actual["framebuffer_height"], receipt["framebuffer_height"])
        self.assertEqual(actual["presentation_frame_index"],
                         receipt["presentation_frame_index"])
        self.assertTrue(evidence["execution"]["capture_success"])
        self.assertIsNone(evidence["execution"]["failure_reason"])
        self.assertEqual(evidence["execution"]["process_exit_code"], 0)

    def test_actual_width_is_not_copied_from_requested_width(self):
        receipt, image = self.real_pair(width=640, height=240)
        plan, evidence = self.evidence_for(receipt, image)
        self.assertEqual(evidence["capture"]["requested"]["width"], 320)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_width"], 640)
        self.assertNotEqual(evidence["capture"]["requested"]["width"],
                            evidence["capture"]["actual"]["framebuffer_width"])
        self.assertEqual(plan["resolution"]["width"], 320)

    def test_actual_height_is_not_copied_from_requested_height(self):
        receipt, image = self.real_pair(width=320, height=480)
        _plan, evidence = self.evidence_for(receipt, image)
        self.assertEqual(evidence["capture"]["requested"]["height"], 240)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_height"], 480)

    def test_actual_frame_comes_from_the_receipt_not_the_manifest(self):
        receipt, image = self.real_pair(frame=7)
        _plan, evidence = self.evidence_for(receipt, image)
        self.assertEqual(evidence["capture"]["requested"]["frame_index"], 10)
        self.assertEqual(evidence["capture"]["actual"]["presentation_frame_index"], 7)

    def test_requested_and_actual_stay_in_separate_objects(self):
        receipt, image = self.real_pair(width=640, height=480, frame=7)
        _plan, evidence = self.evidence_for(receipt, image)
        capture = evidence["capture"]
        self.assertEqual(set(capture["requested"]),
                         {"width", "height", "frame_index"})
        self.assertEqual(set(capture["actual"]),
                         {"framebuffer_width", "framebuffer_height",
                          "presentation_frame_index"})

    def test_image_hash_and_size_come_from_the_verified_file(self):
        receipt, image = self.real_pair()
        _plan, evidence = self.evidence_for(receipt, image)
        block = evidence["capture"]["image"]
        self.assertEqual(block["sha256"], image["sha256"])
        self.assertEqual(block["byte_size"], image["byte_size"])
        self.assertEqual(block["path"], image["path"])
        self.assertEqual(len(block["sha256"]), 64)

    def test_successful_evidence_validates(self):
        receipt, image = self.real_pair()
        _plan, evidence = self.evidence_for(receipt, image)
        validation = rb.validate_evidence_document(evidence)
        self.assertTrue(validation["valid"], validation["errors"])
        path = self.tmp / "capture_evidence.json"
        path.write_text(json.dumps(evidence), encoding="utf-8")
        self.assertEqual(validate_capture_evidence(path), 0)

    def test_successful_evidence_keeps_visual_pass_null(self):
        receipt, image = self.real_pair()
        _plan, evidence = self.evidence_for(receipt, image)
        self.assertIsNone(evidence["verdict"]["visual_pass"])
        self.assertTrue(evidence["verdict"]["visual_pass_reason"])

    def test_failed_evidence_nulls_every_image_field(self):
        plan = self.build_plan(small_manifest())
        execution = {"execution_success": True, "exit_code": 0, "failure_kind": None,
                     "artifacts_written": []}
        verification = {
            "attempted": True, "trusted": False, "process_ok": True,
            "receipt": None, "receipt_errors": ["runtime capture receipt not found"],
            "expectation_errors": [], "image": None,
            "image_errors": ["capture image was not verified"], "checks": [],
            "failure_reason": None,
        }
        evidence = rb.build_capture_evidence(plan, execution, verification)
        self.assertFalse(evidence["execution"]["capture_success"])
        for leaf in ("path", "sha256", "byte_size"):
            self.assertIsNone(evidence["capture"]["image"][leaf], leaf)
        for leaf in ("framebuffer_width", "framebuffer_height",
                     "presentation_frame_index"):
            self.assertIsNone(evidence["capture"]["actual"][leaf], leaf)
        self.assertIn("runtime capture receipt not found",
                      evidence["execution"]["failure_reason"])

    def test_failed_evidence_still_validates(self):
        plan = self.build_plan(small_manifest())
        execution = {"execution_success": False, "exit_code": 3, "failure_kind": None,
                     "artifacts_written": []}
        evidence = rb.build_capture_evidence(plan, execution, None)
        validation = rb.validate_evidence_document(evidence)
        self.assertTrue(validation["valid"], validation["errors"])

    def test_an_untrusted_receipt_never_leaks_a_partially_verified_value(self):
        """Fail closed: a mismatched receipt contributes nothing to actual.*."""
        receipt, image = self.real_pair(frame=7)
        plan = self.build_plan(small_manifest())
        execution = {"execution_success": True, "exit_code": 0, "failure_kind": None,
                     "artifacts_written": []}
        verification = self.trusted_verification(receipt, image)
        verification["trusted"] = False
        verification["expectation_errors"] = ["presentation_frame_index: mismatch"]
        evidence = rb.build_capture_evidence(plan, execution, verification)
        self.assertFalse(evidence["execution"]["capture_success"])
        self.assertIsNone(evidence["capture"]["actual"]["presentation_frame_index"])
        self.assertIsNone(evidence["capture"]["image"]["sha256"])


class TestRuntimeVisualAuditContract(RunnerTestCase):
    def test_valid_audit_is_accepted_with_required_groups(self):
        audit, errors = rva.parse_runtime_visual_audit(audit_for())
        self.assertIsNotNone(audit, errors)
        self.assertEqual(audit.to_json()["schema_version"], "1.0.0")
        self.assertEqual(set(audit.to_json()), rva.ROOT_FIELDS)

    def test_missing_audit_is_rejected(self):
        audit, errors = rva.load_runtime_visual_audit(self.tmp / "absent.json")
        self.assertIsNone(audit)
        self.assertTrue(any("not found" in error for error in errors), errors)

    def test_malformed_audit_is_rejected(self):
        path = self.tmp / "runtime_visual_audit.json"
        path.write_text("{broken", encoding="utf-8")
        audit, errors = rva.load_runtime_visual_audit(path)
        self.assertIsNone(audit)
        self.assertTrue(any("not valid JSON" in error for error in errors), errors)

    def test_frame_mismatch_is_rejected(self):
        audit, errors = rva.parse_runtime_visual_audit(audit_for(frame=9))
        self.assertFalse(errors)
        mismatches = rva.check_audit_expectations(audit, 10, 320, 240)
        self.assertTrue(any("presentation_frame_index" in error for error in mismatches))

    def test_extent_mismatch_is_rejected(self):
        audit, errors = rva.parse_runtime_visual_audit(audit_for(width=640))
        self.assertFalse(errors)
        mismatches = rva.check_audit_expectations(audit, 10, 320, 240)
        self.assertTrue(any("framebuffer_width" in error for error in mismatches))

    def test_gpu_timing_unavailable_is_null_not_zero(self):
        audit, errors = rva.parse_runtime_visual_audit(audit_for())
        self.assertFalse(errors)
        profiling = audit.to_json()["profiling"]
        self.assertEqual(profiling["gpu_timing_status"], "timestamp_query_unsupported")
        self.assertTrue(all(item["gpu_duration_ns"] is None for item in profiling["passes"]))
        broken = audit_for()
        broken["profiling"]["passes"][0]["gpu_duration_ns"] = 0
        parsed, errors = rva.parse_runtime_visual_audit(broken)
        self.assertIsNone(parsed)
        self.assertTrue(any("must be null" in error for error in errors), errors)

    def test_previous_gpu_sample_has_explicit_frame_association(self):
        payload = audit_for(frame=10)
        profiling = payload["profiling"]
        profiling["gpu_timing_status"] = "previous_frame_sample"
        profiling["gpu_timing_source_presentation_frame_index"] = 8
        profiling["gpu_timing_frame_age"] = 2
        profiling["gpu_timing_unavailable_reason"] = None
        for item in profiling["passes"]:
            item["gpu_duration_ns"] = 5.0
        audit, errors = rva.parse_runtime_visual_audit(payload)
        self.assertIsNotNone(audit, errors)

    def test_vegetation_absent_requires_null_stats(self):
        payload = audit_for()
        payload["vegetation"] = {
            "vegetation_present": False,
            "debug_mode": "final",
            "stats": None,
        }
        audit, errors = rva.parse_runtime_visual_audit(payload)
        self.assertIsNotNone(audit, errors)

    def test_execute_fails_closed_when_audit_is_missing(self):
        code, _out, _err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), SKIP_AUDIT=True
        )
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED)
        metadata = self.read_json(self.scene_dir(out_dir) / "run.json")
        self.assertFalse(metadata["runtime_visual_audit_validation"]["valid"])
        self.assertIsNone(metadata["verdict"]["visual_pass"])


class TestEndToEndCaptureExecution(RunnerTestCase):
    """C2B tasks 13-15 and 18, exercised against the real runner entry point."""

    def test_verified_capture_is_a_successful_run(self):
        code, out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(code, rb.EXIT_OK, err)
        self.assertIn("capture_success:   True", out)
        self.assertIn("receipt_trusted:   True", out)
        self.assertIn("evidence_valid:    True", out)
        self.assertIn("runner_success:    True", out)
        self.assertIn("visual_pass:       null", out)
        scene = self.scene_dir(out_dir)
        self.assertTrue((scene / "test_scene_320x240.png").exists())
        self.assertTrue((scene / "runtime_capture_receipt.json").exists())
        self.assertTrue((scene / "capture_evidence.json").exists())
        self.assertTrue((scene / "run.json").exists())

    def test_standalone_evidence_matches_the_evidence_embedded_in_run_json(self):
        _code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(_code, rb.EXIT_OK, err)
        scene = self.scene_dir(out_dir)
        standalone = self.read_json(scene / "capture_evidence.json")
        run_json = self.read_json(scene / "run.json")
        self.assertEqual(standalone, run_json["capture_evidence"])
        self.assertTrue(run_json["capture_evidence_validation"]["valid"])
        self.assertTrue(run_json["capture_evidence_artifact"]["written"])

    def test_standalone_evidence_passes_the_independent_validator_cli(self):
        _code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(_code, rb.EXIT_OK, err)
        scene = self.scene_dir(out_dir)
        manifest_path = self.tmp / "manifest.json"
        self.assertEqual(
            validate_capture_evidence(scene / "capture_evidence.json", manifest_path), 0)

    def test_evidence_image_matches_the_bytes_on_disk(self):
        _code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(_code, rb.EXIT_OK, err)
        scene = self.scene_dir(out_dir)
        evidence = self.read_json(scene / "capture_evidence.json")
        image = scene / "test_scene_320x240.png"
        payload = image.read_bytes()
        self.assertEqual(evidence["capture"]["image"]["path"], str(image))
        self.assertEqual(evidence["capture"]["image"]["sha256"],
                         hashlib.sha256(payload).hexdigest())
        self.assertEqual(evidence["capture"]["image"]["byte_size"], len(payload))
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_width"], 320)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_height"], 240)
        self.assertEqual(evidence["capture"]["actual"]["presentation_frame_index"], 10)
        self.assertEqual(evidence["capture"]["requested"]["width"], 320)
        self.assertIsNone(evidence["verdict"]["visual_pass"])

    def test_requested_and_actual_divergence_is_preserved_end_to_end(self):
        """The runtime really presented something else; both facts survive."""
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), WIDTH=640, HEIGHT=480)
        self.assertEqual(code, rb.EXIT_OK, err)
        evidence = self.read_json(self.scene_dir(out_dir) / "capture_evidence.json")
        self.assertEqual(evidence["capture"]["requested"]["width"], 320)
        self.assertEqual(evidence["capture"]["requested"]["height"], 240)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_width"], 640)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_height"], 480)

    def test_process_exit_zero_without_a_receipt_is_a_runner_failure(self):
        code, out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), SKIP_RECEIPT=True)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        self.assertIn("capture_success:   False", out)
        self.assertIn("runner_success:    False", out)
        scene = self.scene_dir(out_dir)
        self.assertFalse((scene / "runtime_capture_receipt.json").exists())
        evidence = self.read_json(scene / "capture_evidence.json")
        self.assertFalse(evidence["execution"]["capture_success"])
        self.assertEqual(evidence["execution"]["process_exit_code"], 0)
        self.assertIn("not found", evidence["execution"]["failure_reason"])
        for leaf in ("path", "sha256", "byte_size"):
            self.assertIsNone(evidence["capture"]["image"][leaf], leaf)
        self.assertIsNone(evidence["verdict"]["visual_pass"])

    def test_process_exit_zero_with_an_invalid_receipt_is_a_runner_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), RECEIPT_SHA="not-a-sha")
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        run_json = self.read_json(self.scene_dir(out_dir) / "run.json")
        self.assertFalse(run_json["capture_verification"]["trusted"])
        self.assertTrue(run_json["capture_verification"]["receipt_errors"])
        self.assertFalse(run_json["capture_evidence"]["execution"]["capture_success"])

    def test_process_exit_zero_with_a_hash_mismatch_is_a_runner_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), RECEIPT_SHA="f" * 64)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        run_json = self.read_json(self.scene_dir(out_dir) / "run.json")
        self.assertFalse(run_json["capture_verification"]["trusted"])
        self.assertTrue(run_json["capture_verification"]["receipt_errors"] == [])
        self.assertTrue(
            any("sha256" in error
                for error in run_json["capture_verification"]["image_errors"]),
            run_json["capture_verification"]["image_errors"])

    def test_receipt_frame_mismatch_is_a_runner_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), RECEIPT_FRAME=9)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        verification = self.read_json(
            self.scene_dir(out_dir) / "run.json")["capture_verification"]
        self.assertFalse(verification["trusted"])
        self.assertTrue(any("presentation_frame_index" in error
                            for error in verification["expectation_errors"]),
                        verification["expectation_errors"])

    def test_receipt_image_path_mismatch_is_a_runner_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(),
            RECEIPT_IMAGE_PATH=str(self.tmp / "elsewhere.png"))
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        verification = self.read_json(
            self.scene_dir(out_dir) / "run.json")["capture_verification"]
        self.assertFalse(verification["trusted"])
        self.assertTrue(any("image_path" in error
                            for error in verification["expectation_errors"]),
                        verification["expectation_errors"])

    def test_corrupt_png_is_a_runner_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), CORRUPT_PNG=True)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        verification = self.read_json(
            self.scene_dir(out_dir) / "run.json")["capture_verification"]
        self.assertFalse(verification["trusted"])
        self.assertTrue(verification["image_errors"])

    def test_non_zero_exit_reports_a_concrete_failure_reason(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), EXIT_CODE=3)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        evidence = self.read_json(self.scene_dir(out_dir) / "capture_evidence.json")
        self.assertFalse(evidence["execution"]["capture_success"])
        self.assertEqual(evidence["execution"]["process_exit_code"], 3)
        self.assertIn("exited with code 3", evidence["execution"]["failure_reason"])
        self.assertIsNone(evidence["verdict"]["visual_pass"])

    def test_failed_attempt_still_publishes_honest_standalone_evidence(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), SKIP_RECEIPT=True)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        scene = self.scene_dir(out_dir)
        self.assertTrue((scene / "capture_evidence.json").exists())
        self.assertEqual(validate_capture_evidence(scene / "capture_evidence.json"), 0)
        run_json = self.read_json(scene / "run.json")
        self.assertEqual(self.read_json(scene / "capture_evidence.json"),
                         run_json["capture_evidence"])

    def test_evidence_that_fails_its_own_validator_fails_the_run(self):
        script = self.write_fake_runtime()
        self.patch("collect_git_provenance", lambda _root: dict(FAKE_GIT))
        real_build = rb.build_capture_evidence

        def poisoned(plan, execution=None, verification=None):
            evidence = real_build(plan, execution, verification)
            evidence["verdict"]["visual_pass"] = True
            return evidence

        self.patch("build_capture_evidence", poisoned)
        self.install_runtime_shim()
        manifest_path = self.write_manifest(small_manifest())
        out = self.tmp / "out"
        code, stdout, stderr = self.run_main([
            "--manifest", str(manifest_path), "--execute", "--app", str(script),
            "--output-dir", str(out), "--timeout-seconds", "60",
        ])
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, stderr)
        self.assertIn("evidence_valid:    False", stdout)
        scene = self.scene_dir(out)
        # The rejected document is never published as the authoritative artifact.
        self.assertFalse((scene / "capture_evidence.json").exists())
        run_json = self.read_json(scene / "run.json")
        self.assertFalse(run_json["capture_evidence_validation"]["valid"])
        self.assertFalse(run_json["capture_evidence_artifact"]["written"])
        self.assertTrue(any("visual_pass" in error
                            for error in run_json["capture_evidence_validation"]["errors"]))
        self.assertTrue(run_json["capture_evidence"]["verdict"]["visual_pass"])

    def test_run_json_expected_capture_reports_the_verified_artifact(self):
        """Task 21: after an execute the metadata says what was really verified."""
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(code, rb.EXIT_OK, err)
        run_json = self.read_json(self.scene_dir(out_dir) / "run.json")
        expected = run_json["expected_capture"]
        self.assertTrue(expected["verified"])
        self.assertTrue(expected["verified_path"].endswith("test_scene_320x240.png"))
        self.assertEqual(expected["from_manifest"], "test_scene_320x240.png")
        self.assertEqual(expected["format"], "png")
        self.assertEqual(expected["planned_path"], expected["verified_path"])
        # The pre-execution plan itself must not claim a verification.
        self.assertIsNone(run_json["plan"]["expected_output_basename"]["verified"])

    def test_run_json_expected_capture_reports_no_verification_on_failure(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest(), SKIP_RECEIPT=True)
        self.assertEqual(code, rb.EXIT_EXECUTION_FAILED, err)
        expected = self.read_json(self.scene_dir(out_dir) / "run.json")["expected_capture"]
        self.assertFalse(expected["verified"])
        self.assertIsNone(expected["verified_path"])
        self.assertIsNotNone(expected["planned_path"])

    def test_verification_records_every_check_it_ran(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(code, rb.EXIT_OK, err)
        verification = self.read_json(
            self.scene_dir(out_dir) / "run.json")["capture_verification"]
        self.assertEqual(
            [check["check"] for check in verification["checks"]],
            ["process_exit_code_is_zero", "runtime_receipt_parses_as_1_0_0",
             "receipt_matches_request_plan", "png_independently_verified"])
        self.assertTrue(all(check["passed"] for check in verification["checks"]))
        self.assertTrue(verification["trusted"])

    def test_hardware_fields_stay_null_after_a_real_capture(self):
        code, _out, err, out_dir = self.execute_with_fake_runtime(
            manifest=small_manifest())
        self.assertEqual(code, rb.EXIT_OK, err)
        hardware = self.read_json(
            self.scene_dir(out_dir) / "capture_evidence.json")["hardware"]
        self.assertIsNone(hardware["gpu_adapter_name"])
        self.assertIsNone(hardware["graphics_backend"])
        self.assertIsNone(hardware["driver_version"])
        self.assertTrue(hardware["operating_system"])
        self.assertIn("RuntimeCaptureReceipt 1.0.0 carries no adapter",
                      hardware["notes"])

    def test_runner_and_plan_versions_are_1_3_0(self):
        self.assertEqual(rb.RUNNER_VERSION, "1.3.0")
        self.assertEqual(rb.PLAN_VERSION, "1.3.0")
        plan = self.build_plan()
        self.assertEqual(plan["plan_version"], "1.3.0")
        self.assertEqual(plan["runner"]["version"], "1.3.0")

    def test_contract_versions_are_untouched(self):
        self.assertEqual(SUPPORTED_SCHEMA_VERSION, "1.1.0")
        self.assertEqual(rb.EVIDENCE_SCHEMA_VERSION, "1.0.0")
        self.assertEqual(rcr.RECEIPT_SCHEMA_VERSION, "1.0.0")
        self.assertEqual(self.build_plan()["schema_version_supported"], True)


if __name__ == "__main__":
    unittest.main()
