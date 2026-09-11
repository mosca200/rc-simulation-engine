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
import io
import json
import re
import shlex
import sys
import tempfile
import unittest
from pathlib import Path

from tools.visual_benchmark import run_benchmark as rb


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
        "schema_version": "1.0.0",
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
        self.assertTrue(plan["expected_output_basename"]["matches_vis0_convention"])
        self.assertTrue(plan["schema_version_major_supported"])

    def test_validation_reuses_approved_validator(self):
        manifest = base_manifest()
        self.assertEqual(rb.validate_manifest_dict(manifest, self.tmp), [])
        manifest["renderer"]["version"] = "v3"
        errors = rb.validate_manifest_dict(manifest, self.tmp)
        self.assertTrue(any("renderer.version" in error for error in errors))

    def test_output_paths_use_scene_id_without_timestamp(self):
        plan = self.build_plan()
        self.assertTrue(Path(plan["scene_output_dir"]).name == "test_scene")
        for key, value in plan["artifact_paths"].items():
            self.assertTrue(value.endswith((".json", ".txt")), key)
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

    def test_no_resolution_or_capture_flag_is_ever_emitted(self):
        plan = self.build_plan()
        argv = " ".join(plan["command_argv"])
        for forbidden in ("--width", "--height", "--resolution", "--capture",
                          "--warmup", "--frame", "--screenshot", "--output"):
            self.assertNotIn(forbidden, argv)

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
        synthetic = self._synthetic_plan([sys.executable, str(sleeper)])
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
        scene_dir = self.tmp / "out" / "synthetic_scene"
        return {"scene_output_dir": str(scene_dir), "command_argv": list(argv)}


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
        plan = self._synthetic_plan(argv)
        execution = rb.execute_plan(plan, argv[0], 30)
        full_plan = self.build_plan()
        full_plan["scene_output_dir"] = plan["scene_output_dir"]
        full_plan["artifact_paths"]["run_json"] = str(
            Path(plan["scene_output_dir"]) / "run.json"
        )
        full_plan["command_argv"] = list(argv)
        execution["artifacts_written"].append(full_plan["artifact_paths"]["run_json"])
        metadata = rb.build_run_metadata(full_plan, execution)
        run_json = Path(full_plan["artifact_paths"]["run_json"])
        rb.write_json(run_json, metadata)
        return run_json, metadata

    def _synthetic_plan(self, argv):
        return {
            "scene_output_dir": str(self.tmp / "out" / "test_scene"),
            "command_argv": list(argv),
        }

    def test_run_json_is_valid_json_with_required_provenance(self):
        run_json, _ = self.run_execution()
        payload = json.loads(run_json.read_text(encoding="utf-8"))
        self.assertEqual(payload["manifest"]["schema_version"], "1.0.0")
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
        self.assertIsNone(metadata["artifacts"]["capture"])
        self.assertIn("CAPTURE BACKEND NOT YET AVAILABLE",
                      metadata["artifacts"]["capture_reason"])

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
        tree = ast.parse(RUNNER_SOURCE.read_text(encoding="utf-8"))
        allowed = {
            "argparse", "hashlib", "json", "os", "platform", "shlex", "subprocess",
            "sys", "time", "dataclasses", "datetime", "pathlib", "typing",
            # The approved VIS0-A validator, imported as a sibling module.
            "validate_manifest", "tools",
        }
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
        tree = ast.parse(RUNNER_SOURCE.read_text(encoding="utf-8"))
        forbidden = {"ssim", "psnr", "lpips", "cv2", "ImageGrab", "ImageChops",
                     "numpy", "np", "mss", "skimage", "PIL", "compare_images"}
        names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                names.add(node.id)
            elif isinstance(node, ast.Attribute):
                names.add(node.attr)
        self.assertEqual(sorted(names & forbidden), [])


class TestRuntimeCapabilities(RunnerTestCase):
    """Q. unsupported resolution/capture are explicitly represented."""

    def test_capture_backend_is_declared_unavailable(self):
        capture = self.build_plan()["runtime_capabilities"]["capture_backend"]
        self.assertEqual(capture["status"], "unavailable")
        self.assertFalse(capture["produces_image"])
        self.assertIn("CAPTURE BACKEND NOT YET AVAILABLE", capture["reason"])

    def test_resolution_enforcement_is_declared_unsupported(self):
        capability = self.build_plan()["runtime_capabilities"]["resolution_enforcement"]
        self.assertEqual(capability["status"], "unsupported")
        self.assertFalse(capability["enforced"])
        self.assertEqual(capability["requested"], {"width": 1920, "height": 1080})
        self.assertIn("unsupported", capability["reason"])

    def test_warmup_is_declared_unsupported(self):
        capability = self.build_plan()["runtime_capabilities"]["warmup_frames"]
        self.assertEqual(capability["status"], "unsupported")
        self.assertEqual(capability["requested"], 10)

    def test_no_auto_exit_is_declared(self):
        capability = self.build_plan()["runtime_capabilities"]["process_auto_exit"]
        self.assertEqual(capability["status"], "unsupported")
        self.assertIn("interactive winit event loop", capability["reason"])

    def test_unsupported_fields_are_labelled_in_field_mapping(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        for field_name in ("resolution.width", "resolution.height", "warmup",
                           "capture.frame"):
            self.assertEqual(mapping[field_name]["status"], "unsupported", field_name)
            self.assertIsNone(mapping[field_name]["runtime_flag"], field_name)
            self.assertFalse(mapping[field_name]["emitted"], field_name)
            self.assertTrue(mapping[field_name]["reason"], field_name)

    def test_metadata_only_fields_are_labelled(self):
        mapping = {m["manifest_field"]: m for m in self.build_plan()["field_mapping"]}
        for field_name in ("schema_version", "scene_id", "description", "tags",
                           "capture.filename", "capture.format"):
            self.assertEqual(mapping[field_name]["status"], "metadata-only", field_name)
            self.assertIsNone(mapping[field_name]["runtime_flag"], field_name)

    def test_expected_capture_basename_follows_vis0_convention(self):
        basename = self.build_plan()["expected_output_basename"]
        self.assertEqual(basename["per_vis0_convention"], "test_scene_1920x1080.png")
        self.assertTrue(basename["matches_vis0_convention"])
        self.assertFalse(basename["produced"])

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
        self.assertEqual(capabilities["capture_backend"]["status"], "unavailable")
        self.assertEqual(capabilities["resolution_enforcement"]["status"], "unsupported")


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
        self.assertIn("no process started", out)

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
        for fragment in ("field mapping", "--pilot-position", "CAPTURE BACKEND NOT YET AVAILABLE",
                         "UNSUPPORTED", "visual_pass"):
            self.assertIn(fragment, out)

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


if __name__ == "__main__":
    unittest.main()
