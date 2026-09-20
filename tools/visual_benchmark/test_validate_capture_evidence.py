#!/usr/bin/env python3
"""
Tests for the RV2-VIS0-C1B Visual Capture Evidence validator.

Run with:
    python -X utf8 -m unittest tools/visual_benchmark/test_validate_capture_evidence.py -v

`-X utf8` keeps the redirected stream lossless on a cp1252 console; the module
itself only emits ASCII markers.
"""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from tools.visual_benchmark.validate_capture_evidence import (
    ALL_LEAF_FIELDS,
    RUNTIME_SUPPLIED_FIELDS,
    TOOLING_SUPPLIED_FIELDS,
    CaptureEvidenceValidator,
    sha256_of_file,
    validate_capture_evidence,
)


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
TOOL_DIR = REPO_ROOT / "tools" / "visual_benchmark"
EVIDENCE_SCHEMA = TOOL_DIR / "visual_capture_evidence.schema.json"
REFERENCE_MANIFEST = (
    REPO_ROOT / "docs" / "validation" / "visual_benchmark" / "vis0_reference_scene.json"
)


def honest_failure_evidence() -> dict:
    """The only artifact today's runtime can truthfully produce.

    `integration/render-v2` has no capture backend, so capture_success is false,
    every image leaf is null and every runtime-supplied measurement is null.
    """
    return {
        "schema_version": "1.0.0",
        "scene_id": "aircraft_acro_static_front",
        "manifest": {
            "path": "docs/validation/visual_benchmark/vis0_reference_scene.json",
            "path_display": "docs/validation/visual_benchmark/vis0_reference_scene.json",
            "sha256": "a" * 64,
        },
        "source": {
            "commit_sha": "b" * 40,
            "commit_sha_short": "b" * 12,
            "branch": "integration/render-v2",
            "detached_head": False,
            "dirty": False,
            "dirty_entry_count": 0,
            "runner_name": "rv2-vis0-benchmark-runner",
            "runner_version": "1.1.0",
        },
        "renderer": {
            "version": "v2",
            "exposure_ev": 0.0,
            "camera_mode": "pilot",
            "scenery_preset": "flying-field",
        },
        "capture": {
            "requested": {"width": 1920, "height": 1080, "frame_index": 10},
            "actual": {
                "framebuffer_width": None,
                "framebuffer_height": None,
                "presentation_frame_index": None,
            },
            "format": "png",
            "image": {"path": None, "sha256": None, "byte_size": None},
        },
        "execution": {
            "capture_success": False,
            "process_exit_code": None,
            "failure_reason": "no capture backend in integration/render-v2",
        },
        "hardware": {
            "operating_system": "Windows",
            "os_release": "11",
            "architecture": "AMD64",
            "gpu_adapter_name": None,
            "graphics_backend": None,
            "driver_version": None,
            "notes": "adapter metadata is not reported by the runtime",
        },
        "verdict": {
            "visual_pass": None,
            "visual_pass_reason": "evidence records facts only; no visual verdict exists",
        },
    }


def successful_capture_evidence() -> dict:
    """What a future LINEA 1 capture backend will hand over."""
    evidence = honest_failure_evidence()
    evidence["capture"]["actual"] = {
        "framebuffer_width": 1920,
        "framebuffer_height": 1080,
        "presentation_frame_index": 10,
    }
    evidence["capture"]["image"] = {
        "path": "tmp/visual_benchmark_runs/aircraft_acro_static_front/capture.png",
        "sha256": "c" * 64,
        "byte_size": 204800,
    }
    evidence["execution"] = {
        "capture_success": True,
        "process_exit_code": 0,
        "failure_reason": None,
    }
    evidence["hardware"]["gpu_adapter_name"] = "NVIDIA GeForce RTX 4090"
    evidence["hardware"]["graphics_backend"] = "vulkan"
    evidence["hardware"]["driver_version"] = "546.33"
    return evidence


class EvidenceTestCase(unittest.TestCase):
    """Shared temp-file writing and error-path helpers."""

    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.tmp = Path(directory.name)

    def write_evidence(self, evidence=None, name="evidence.json") -> Path:
        payload = honest_failure_evidence() if evidence is None else evidence
        path = self.tmp / name
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path

    def error_paths(self, evidence: dict) -> list:
        validator = CaptureEvidenceValidator(copy.deepcopy(evidence))
        validator.validate()
        return [error.path for error in validator.errors]

    def assertInvalid(self, evidence: dict, path: str):
        paths = self.error_paths(evidence)
        self.assertIn(path, paths, f"expected an error on {path}, got {paths}")

    def assertValid(self, evidence: dict):
        validator = CaptureEvidenceValidator(copy.deepcopy(evidence))
        self.assertTrue(
            validator.validate(),
            "expected valid evidence, got: "
            + "; ".join(str(error) for error in validator.errors))

    def mutate(self, path: str, value):
        """Return a deep copy of the fixture with a dotted path overwritten."""
        evidence = honest_failure_evidence()
        keys = path.split(".")
        node = evidence
        for key in keys[:-1]:
            node = node[key]
        node[keys[-1]] = value
        return evidence


class TestValidEvidence(EvidenceTestCase):
    def test_honest_failure_evidence_is_valid(self):
        """No capture backend => capture_success false and every image null."""
        self.assertEqual(validate_capture_evidence(self.write_evidence()), 0)

    def test_successful_capture_evidence_is_valid(self):
        self.assertEqual(validate_capture_evidence(self.write_evidence(
            successful_capture_evidence())), 0)

    def test_all_null_hardware_is_acceptable(self):
        """Unavailable hardware metadata is honest, not an error."""
        evidence = honest_failure_evidence()
        for field in ("operating_system", "os_release", "architecture",
                      "gpu_adapter_name", "graphics_backend", "driver_version",
                      "notes"):
            evidence["hardware"][field] = None
        self.assertValid(evidence)
        self.assertEqual(validate_capture_evidence(self.write_evidence(evidence)), 0)

    def test_nullable_source_leaves_accepted(self):
        """A detached HEAD with no git metadata still validates."""
        evidence = honest_failure_evidence()
        evidence["source"].update({
            "commit_sha_short": None,
            "branch": None,
            "detached_head": True,
            "dirty": None,
            "dirty_entry_count": None,
        })
        self.assertValid(evidence)

    def test_sha256_commit_repository_accepted(self):
        evidence = self.mutate("source.commit_sha", "d" * 64)
        self.assertValid(evidence)

    def test_requested_frame_index_may_be_null(self):
        """A manifest without capture.frame has no requested index to record."""
        evidence = self.mutate("capture.requested.frame_index", None)
        self.assertValid(evidence)

    def test_visual_pass_may_be_absent(self):
        evidence = honest_failure_evidence()
        del evidence["verdict"]["visual_pass"]
        self.assertValid(evidence)

    def test_validation_never_fills_visual_pass(self):
        """The validator reads facts; it must not write a verdict."""
        evidence = honest_failure_evidence()
        CaptureEvidenceValidator(evidence).validate()
        self.assertIsNone(evidence["verdict"]["visual_pass"])


class TestRequestedVersusActual(EvidenceTestCase):
    def test_divergent_resolution_is_representable(self):
        """The real gap: 1280x720 presented against 1920x1080 requested."""
        evidence = honest_failure_evidence()
        evidence["capture"]["requested"] = {
            "width": 1920, "height": 1080, "frame_index": 10}
        evidence["capture"]["actual"] = {
            "framebuffer_width": 1280,
            "framebuffer_height": 720,
            "presentation_frame_index": 10,
        }
        evidence["execution"] = {
            "capture_success": True, "process_exit_code": 0, "failure_reason": None}
        evidence["capture"]["image"] = {
            "path": "capture.png", "sha256": "c" * 64, "byte_size": 900000}
        self.assertValid(evidence)
        # Both pairs stay readable and distinct - neither overwrites the other.
        self.assertEqual(evidence["capture"]["requested"]["width"], 1920)
        self.assertEqual(evidence["capture"]["actual"]["framebuffer_width"], 1280)

    def test_divergent_frame_index_is_representable(self):
        """Requested frame 10, actually presented frame 12: both recorded."""
        evidence = successful_capture_evidence()
        evidence["capture"]["requested"]["frame_index"] = 10
        evidence["capture"]["actual"]["presentation_frame_index"] = 12
        self.assertValid(evidence)
        self.assertNotEqual(
            evidence["capture"]["requested"]["frame_index"],
            evidence["capture"]["actual"]["presentation_frame_index"])

    def test_requested_width_is_mandatory(self):
        """Intent always comes from the manifest, so it cannot be null."""
        self.assertInvalid(self.mutate("capture.requested.width", None),
                           "capture.requested.width")

    def test_actual_may_stay_null_on_failure(self):
        evidence = honest_failure_evidence()
        self.assertIsNone(evidence["capture"]["actual"]["framebuffer_width"])
        self.assertValid(evidence)


class TestPlaceholderRejection(EvidenceTestCase):
    """null is the only 'unavailable' marker; 0/-1/'' must not sneak through."""

    def test_zero_framebuffer_width_rejected(self):
        self.assertInvalid(self.mutate("capture.actual.framebuffer_width", 0),
                           "capture.actual.framebuffer_width")

    def test_zero_framebuffer_height_rejected(self):
        self.assertInvalid(self.mutate("capture.actual.framebuffer_height", 0),
                           "capture.actual.framebuffer_height")

    def test_negative_framebuffer_width_rejected(self):
        self.assertInvalid(self.mutate("capture.actual.framebuffer_width", -1),
                           "capture.actual.framebuffer_width")

    def test_negative_framebuffer_height_rejected(self):
        self.assertInvalid(self.mutate("capture.actual.framebuffer_height", -1920),
                           "capture.actual.framebuffer_height")

    def test_zero_byte_size_rejected(self):
        evidence = successful_capture_evidence()
        evidence["capture"]["image"]["byte_size"] = 0
        self.assertInvalid(evidence, "capture.image.byte_size")

    def test_negative_presentation_frame_index_rejected(self):
        self.assertInvalid(
            self.mutate("capture.actual.presentation_frame_index", -1),
            "capture.actual.presentation_frame_index")

    def test_empty_string_hardware_field_rejected(self):
        self.assertInvalid(self.mutate("hardware.gpu_adapter_name", ""),
                           "hardware.gpu_adapter_name")

    def test_whitespace_only_string_rejected(self):
        self.assertInvalid(self.mutate("hardware.graphics_backend", "   "),
                           "hardware.graphics_backend")

    def test_float_dimension_rejected(self):
        self.assertInvalid(self.mutate("capture.actual.framebuffer_width", 1920.0),
                           "capture.actual.framebuffer_width")

    def test_requested_width_below_manifest_minimum_rejected(self):
        self.assertInvalid(self.mutate("capture.requested.width", 100),
                           "capture.requested.width")


class TestSchemaVersion(EvidenceTestCase):
    def test_non_semantic_version_rejected(self):
        self.assertInvalid(self.mutate("schema_version", "v1.0"), "schema_version")

    def test_unsupported_major_rejected(self):
        self.assertInvalid(self.mutate("schema_version", "2.0.0"), "schema_version")

    def test_missing_schema_version_rejected(self):
        evidence = honest_failure_evidence()
        del evidence["schema_version"]
        self.assertInvalid(evidence, "schema_version")

    def test_null_schema_version_rejected(self):
        self.assertInvalid(self.mutate("schema_version", None), "schema_version")

    def test_minor_bump_within_major_one_accepted(self):
        self.assertValid(self.mutate("schema_version", "1.1.0"))


class TestSceneId(EvidenceTestCase):
    def test_invalid_pattern_rejected(self):
        self.assertInvalid(self.mutate("scene_id", "Invalid-Scene"), "scene_id")

    def test_too_short_rejected(self):
        self.assertInvalid(self.mutate("scene_id", "ab"), "scene_id")

    def test_mismatch_against_manifest_rejected(self):
        """scene_id must describe the manifest that was actually run."""
        if not REFERENCE_MANIFEST.exists():
            self.skipTest("reference manifest not available")
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        evidence = honest_failure_evidence()
        evidence["manifest"]["sha256"] = sha256_of_file(REFERENCE_MANIFEST)
        evidence["scene_id"] = "some_other_scene"
        validator = CaptureEvidenceValidator(evidence, manifest)
        self.assertFalse(validator.validate())
        self.assertIn("scene_id", [error.path for error in validator.errors])

    def test_matching_scene_id_passes_cross_check(self):
        if not REFERENCE_MANIFEST.exists():
            self.skipTest("reference manifest not available")
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        evidence = honest_failure_evidence()
        evidence["manifest"]["sha256"] = sha256_of_file(REFERENCE_MANIFEST)
        evidence["scene_id"] = manifest["scene_id"]
        self.assertTrue(CaptureEvidenceValidator(evidence, manifest).validate())


class TestManifestProvenance(EvidenceTestCase):
    def test_malformed_sha256_rejected(self):
        self.assertInvalid(self.mutate("manifest.sha256", "not-a-hash"),
                           "manifest.sha256")

    def test_uppercase_sha256_rejected(self):
        self.assertInvalid(self.mutate("manifest.sha256", "A" * 64),
                           "manifest.sha256")

    def test_short_sha256_rejected(self):
        self.assertInvalid(self.mutate("manifest.sha256", "a" * 63),
                           "manifest.sha256")

    def test_null_sha256_rejected(self):
        """Provenance is the point of the artifact, so the digest is mandatory."""
        self.assertInvalid(self.mutate("manifest.sha256", None), "manifest.sha256")

    def test_digest_mismatch_against_real_manifest_rejected(self):
        """A stale digest means the evidence came from a different manifest."""
        if not REFERENCE_MANIFEST.exists():
            self.skipTest("reference manifest not available")
        evidence = honest_failure_evidence()
        evidence["manifest"]["sha256"] = "f" * 64
        path = self.write_evidence(evidence)
        self.assertEqual(validate_capture_evidence(path, REFERENCE_MANIFEST), 1)

    def test_digest_match_against_real_manifest_accepted(self):
        if not REFERENCE_MANIFEST.exists():
            self.skipTest("reference manifest not available")
        evidence = honest_failure_evidence()
        evidence["manifest"]["sha256"] = sha256_of_file(REFERENCE_MANIFEST)
        path = self.write_evidence(evidence)
        self.assertEqual(validate_capture_evidence(path, REFERENCE_MANIFEST), 0)

    def test_requested_values_cross_checked_against_manifest(self):
        if not REFERENCE_MANIFEST.exists():
            self.skipTest("reference manifest not available")
        manifest = json.loads(REFERENCE_MANIFEST.read_text(encoding="utf-8"))
        evidence = honest_failure_evidence()
        evidence["manifest"]["sha256"] = sha256_of_file(REFERENCE_MANIFEST)
        evidence["scene_id"] = manifest["scene_id"]
        evidence["capture"]["requested"]["width"] = 3840
        validator = CaptureEvidenceValidator(evidence, manifest)
        self.assertFalse(validator.validate())
        self.assertIn("capture.requested.width",
                      [error.path for error in validator.errors])

    def test_null_manifest_path_accepted(self):
        evidence = self.mutate("manifest.path", None)
        evidence["manifest"]["path_display"] = None
        self.assertValid(evidence)


class TestSourceProvenance(EvidenceTestCase):
    def test_short_commit_sha_rejected(self):
        self.assertInvalid(self.mutate("source.commit_sha", "b" * 39),
                           "source.commit_sha")

    def test_non_hex_commit_sha_rejected(self):
        self.assertInvalid(self.mutate("source.commit_sha", "z" * 40),
                           "source.commit_sha")

    def test_uppercase_commit_sha_rejected(self):
        self.assertInvalid(self.mutate("source.commit_sha", "B" * 40),
                           "source.commit_sha")

    def test_null_commit_sha_rejected(self):
        self.assertInvalid(self.mutate("source.commit_sha", None), "source.commit_sha")

    def test_non_semantic_runner_version_rejected(self):
        self.assertInvalid(self.mutate("source.runner_version", "1.1"),
                           "source.runner_version")

    def test_missing_runner_name_rejected(self):
        evidence = honest_failure_evidence()
        del evidence["source"]["runner_name"]
        self.assertInvalid(evidence, "source.runner_name")


class TestCaptureSuccessInvariants(EvidenceTestCase):
    def test_success_without_image_sha256_rejected(self):
        """A declared capture must be verifiable."""
        evidence = successful_capture_evidence()
        evidence["capture"]["image"]["sha256"] = None
        self.assertInvalid(evidence, "capture.image.sha256")

    def test_success_without_image_path_rejected(self):
        evidence = successful_capture_evidence()
        evidence["capture"]["image"]["path"] = None
        self.assertInvalid(evidence, "capture.image.path")

    def test_success_without_byte_size_rejected(self):
        evidence = successful_capture_evidence()
        evidence["capture"]["image"]["byte_size"] = None
        self.assertInvalid(evidence, "capture.image.byte_size")

    def test_success_without_actual_framebuffer_rejected(self):
        evidence = successful_capture_evidence()
        evidence["capture"]["actual"]["framebuffer_width"] = None
        self.assertInvalid(evidence, "capture.actual.framebuffer_width")

    def test_success_without_actual_frame_index_rejected(self):
        evidence = successful_capture_evidence()
        evidence["capture"]["actual"]["presentation_frame_index"] = None
        self.assertInvalid(evidence, "capture.actual.presentation_frame_index")

    def test_success_with_nonzero_exit_code_rejected(self):
        evidence = successful_capture_evidence()
        evidence["execution"]["process_exit_code"] = 4
        self.assertInvalid(evidence, "execution.process_exit_code")

    def test_success_with_failure_reason_rejected(self):
        evidence = successful_capture_evidence()
        evidence["execution"]["failure_reason"] = "should not be here"
        self.assertInvalid(evidence, "execution.failure_reason")

    def test_failure_with_false_image_rejected(self):
        """A failed capture must not advertise an image that was never written."""
        evidence = honest_failure_evidence()
        evidence["capture"]["image"]["sha256"] = "c" * 64
        self.assertInvalid(evidence, "capture.image.sha256")

    def test_failure_with_false_image_path_rejected(self):
        evidence = honest_failure_evidence()
        evidence["capture"]["image"]["path"] = "capture.png"
        self.assertInvalid(evidence, "capture.image.path")

    def test_failure_with_false_byte_size_rejected(self):
        evidence = honest_failure_evidence()
        evidence["capture"]["image"]["byte_size"] = 1234
        self.assertInvalid(evidence, "capture.image.byte_size")

    def test_failure_without_reason_rejected(self):
        evidence = honest_failure_evidence()
        evidence["execution"]["failure_reason"] = None
        self.assertInvalid(evidence, "execution.failure_reason")

    def test_missing_capture_success_rejected(self):
        evidence = honest_failure_evidence()
        del evidence["execution"]["capture_success"]
        self.assertInvalid(evidence, "execution.capture_success")

    def test_non_boolean_capture_success_rejected(self):
        self.assertInvalid(self.mutate("execution.capture_success", "yes"),
                           "execution.capture_success")


class TestEvidenceIsNotAVerdict(EvidenceTestCase):
    def test_visual_pass_true_rejected(self):
        self.assertInvalid(self.mutate("verdict.visual_pass", True),
                           "verdict.visual_pass")

    def test_visual_pass_false_rejected(self):
        """Even a negative verdict is a verdict, and v1 forbids both."""
        self.assertInvalid(self.mutate("verdict.visual_pass", False),
                           "verdict.visual_pass")

    def test_visual_pass_string_rejected(self):
        self.assertInvalid(self.mutate("verdict.visual_pass", "pass"),
                           "verdict.visual_pass")

    def test_capture_success_does_not_imply_visual_pass(self):
        """A fully successful capture still leaves the visual verdict null."""
        evidence = successful_capture_evidence()
        self.assertTrue(evidence["execution"]["capture_success"])
        self.assertIsNone(evidence["verdict"]["visual_pass"])
        self.assertValid(evidence)

    def test_no_metric_threshold_constants_exist(self):
        """No SSIM/PSNR/LPIPS threshold may hide in the validator."""
        source = Path(
            __file__).with_name("validate_capture_evidence.py").read_text(encoding="utf-8")
        lowered = source.lower()
        for token in ("ssim", "psnr", "lpips", "threshold"):
            self.assertNotIn(token, lowered,
                             f"visual metric '{token}' leaked into the evidence contract")

    def test_schema_locks_visual_pass_to_null(self):
        if not EVIDENCE_SCHEMA.exists():
            self.skipTest("evidence schema not available")
        schema = json.loads(EVIDENCE_SCHEMA.read_text(encoding="utf-8"))
        verdict = schema["properties"]["verdict"]["properties"]["visual_pass"]
        self.assertEqual(verdict["type"], "null")


class TestUnknownFieldPolicy(EvidenceTestCase):
    def test_unknown_top_level_field_rejected(self):
        evidence = honest_failure_evidence()
        evidence["visual_score"] = 0.98
        self.assertInvalid(evidence, "visual_score")

    def test_unknown_nested_field_rejected(self):
        evidence = honest_failure_evidence()
        evidence["capture"]["actual"]["unknown_leaf"] = 1
        self.assertInvalid(evidence, "capture.actual.unknown_leaf")

    def test_unknown_hardware_field_rejected(self):
        evidence = honest_failure_evidence()
        evidence["hardware"]["vram_gb"] = 24
        self.assertInvalid(evidence, "hardware.vram_gb")

    def test_unknown_field_error_documents_the_policy(self):
        evidence = honest_failure_evidence()
        evidence["visual_score"] = 0.98
        validator = CaptureEvidenceValidator(evidence)
        validator.validate()
        message = " ".join(str(error) for error in validator.errors)
        self.assertIn("unknown property", message)
        self.assertIn("rejected", message)

    def test_schema_forbids_additional_properties_everywhere(self):
        """The documented policy is fail-closed at every object level."""
        if not EVIDENCE_SCHEMA.exists():
            self.skipTest("evidence schema not available")
        schema = json.loads(EVIDENCE_SCHEMA.read_text(encoding="utf-8"))

        def walk(node, path):
            if not isinstance(node, dict):
                return
            if "properties" in node:
                self.assertIs(
                    node.get("additionalProperties"), False,
                    f"{path or '<root>'} allows unknown properties")
                for key, sub in node["properties"].items():
                    walk(sub, f"{path}.{key}" if path else key)
            for key in ("items",):
                if key in node:
                    walk(node[key], f"{path}[]")

        walk(schema, "")


class TestRootAndFileHandling(EvidenceTestCase):
    def test_missing_file_returns_two(self):
        self.assertEqual(
            validate_capture_evidence(self.tmp / "absent.json"), 2)

    def test_invalid_json_returns_two(self):
        path = self.tmp / "broken.json"
        path.write_text("{not json", encoding="utf-8")
        self.assertEqual(validate_capture_evidence(path), 2)

    def test_array_root_returns_one(self):
        path = self.tmp / "array.json"
        path.write_text("[]", encoding="utf-8")
        self.assertEqual(validate_capture_evidence(path), 1)

    def test_missing_manifest_file_returns_two(self):
        path = self.write_evidence()
        self.assertEqual(
            validate_capture_evidence(path, self.tmp / "absent-manifest.json"), 2)

    def test_main_returns_exit_code(self):
        from tools.visual_benchmark.validate_capture_evidence import main
        path = self.write_evidence()
        self.assertEqual(main([str(path)]), 0)


class TestLineOneHandshake(EvidenceTestCase):
    """TASK 5: the producer split must be explicit, disjoint and complete."""

    def test_runtime_and_tooling_fields_are_disjoint(self):
        overlap = RUNTIME_SUPPLIED_FIELDS & TOOLING_SUPPLIED_FIELDS
        self.assertEqual(set(), overlap,
                         f"field(s) claimed by both producers: {sorted(overlap)}")

    def test_schema_leaves_match_the_handshake_exactly(self):
        """Every schema leaf is owned by exactly one producer - no orphans."""
        if not EVIDENCE_SCHEMA.exists():
            self.skipTest("evidence schema not available")
        schema = json.loads(EVIDENCE_SCHEMA.read_text(encoding="utf-8"))

        def leaves(node, prefix=""):
            found = set()
            properties = node.get("properties")
            if not isinstance(properties, dict):
                return found
            for key, sub in properties.items():
                dotted = f"{prefix}{key}"
                nested = leaves(sub, f"{dotted}.")
                if nested:
                    found |= nested
                else:
                    found.add(dotted)
            return found

        declared = leaves(schema)
        self.assertEqual(declared, ALL_LEAF_FIELDS,
                         f"schema-only: {sorted(declared - ALL_LEAF_FIELDS)}, "
                         f"handshake-only: {sorted(ALL_LEAF_FIELDS - declared)}")

    def test_runtime_supplied_fields_are_nullable_in_the_schema(self):
        """Tooling must be able to leave every runtime leaf honestly null."""
        if not EVIDENCE_SCHEMA.exists():
            self.skipTest("evidence schema not available")
        schema = json.loads(EVIDENCE_SCHEMA.read_text(encoding="utf-8"))

        def resolve(dotted):
            node = schema
            for part in dotted.split("."):
                node = node["properties"][part]
            return node

        for dotted in sorted(RUNTIME_SUPPLIED_FIELDS):
            with self.subTest(field=dotted):
                node = resolve(dotted)
                if dotted == "execution.capture_success":
                    # The runtime always answers this; a missing answer would be
                    # a lie by omission, so it stays a mandatory boolean.
                    self.assertEqual(node["type"], "boolean")
                    continue
                declared = node["type"]
                self.assertIsInstance(declared, list, f"{dotted} cannot be null")
                self.assertIn("null", declared, f"{dotted} cannot be null")

    def test_honest_failure_evidence_uses_only_tooling_values(self):
        """Today's artifact must leave every runtime leaf null or false."""
        evidence = honest_failure_evidence()

        def resolve(dotted):
            node = evidence
            for part in dotted.split("."):
                node = node[part]
            return node

        for dotted in sorted(RUNTIME_SUPPLIED_FIELDS):
            with self.subTest(field=dotted):
                value = resolve(dotted)
                if dotted == "execution.capture_success":
                    self.assertIs(value, False)
                else:
                    self.assertIsNone(
                        value, f"{dotted} carries a value no runtime reported")


if __name__ == "__main__":
    unittest.main()
