#!/usr/bin/env python3
"""
RV2-7A aircraft asset pipeline contract tests.

Standard library only, no wgpu, no Blender required. Run from the repository root:

    python -m unittest discover -s tools/aircraft_asset_pipeline -p "test_*.py" -v
    python tools/aircraft_asset_pipeline/test_asset_contract.py

These tests guard the boundary between the presentation asset pipeline and the
runtime: they prove the semantic manifest agrees with the committed production
GLB, with model.json's presentation mapping and with the renderer's authored
part order, and that the validators actually fail closed.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
import tempfile
import unittest

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import asset_contract as ac  # noqa: E402
import validate_glb  # noqa: E402

ASSET_ID = ac.DEFAULT_ASSET_ID
PRODUCTION_GLB = ac.repo_relative(f"models/{ASSET_ID}/aircraft.glb")
MODEL_JSON = ac.repo_relative(f"models/{ASSET_ID}/model.json")
BLENDER_SOURCE = ac.repo_relative(f"models/{ASSET_ID}/source/{ASSET_ID}.blend")
RENDERER_ASSET_TEST = os.path.join(ac.REPO_ROOT, "crates", "renderer", "tests", "aircraft_asset_g3c.rs")


def sha256_of(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


class ManifestInternals(unittest.TestCase):
    """The manifest must be self-consistent before it can be an authority."""

    @classmethod
    def setUpClass(cls):
        cls.manifest = ac.load_manifest(ASSET_ID)
        cls.components = ac.components(cls.manifest)

    def test_primitive_indices_are_dense_and_ascending(self):
        indices = [c["primitive_index"] for c in self.components]
        self.assertEqual(indices, list(range(len(self.components))))

    def test_semantic_ids_are_unique_and_well_formed(self):
        pattern = re.compile(ac.SEMANTIC_ID_PATTERN)
        seen = set()
        for component in self.components:
            semantic_id = component["semantic_id"]
            self.assertNotIn(semantic_id, seen, f"duplicate semantic id {semantic_id}")
            seen.add(semantic_id)
            self.assertRegex(semantic_id, pattern)

    def test_legacy_node_names_are_unique(self):
        names = [c["legacy_node_name"] for c in self.components]
        self.assertEqual(len(names), len(set(names)))

    def test_export_node_names_pin_the_primitive_index_and_sort_into_order(self):
        names = []
        for component in self.components:
            expected = ac.export_node_name(component)
            self.assertEqual(component["export_node_name"], expected)
            prefix, semantic_id = expected.split("_", 1)
            self.assertEqual(int(prefix), component["primitive_index"])
            self.assertEqual("_".join(expected.split("_")[1:]), component["semantic_id"])
            names.append(expected)
        self.assertEqual(sorted(names), names, "export names must sort into primitive order")

    def test_every_component_material_is_in_the_catalog(self):
        catalog = ac.material_catalog(self.manifest)
        self.assertTrue(catalog)
        for component in self.components:
            self.assertIn(component["material_name"], catalog)

    def test_material_index_order_profiles_are_permutations_of_the_catalog(self):
        catalog = sorted(ac.material_catalog(self.manifest))
        profiles = self.manifest["materials"]["index_order_profiles"]
        for name, order in profiles.items():
            self.assertEqual(sorted(order), catalog, f"profile {name!r} is not a permutation")
            self.assertEqual(len(order), len(set(order)), f"profile {name!r} repeats a material")

    def test_moving_surfaces_agree_with_the_component_table(self):
        by_semantic = {c["semantic_id"]: c for c in self.components}
        indices = []
        for surface in ac.moving_surfaces(self.manifest):
            component = by_semantic.get(surface["semantic_id"])
            self.assertIsNotNone(component, f"{surface['semantic_id']} has no component")
            self.assertEqual(component["primitive_index"], surface["primitive_index"])
            self.assertEqual(component["moving_surface"], surface["surface_id"])
            self.assertEqual(component["family"], ac.CONTROL_SURFACE_FAMILY)
            indices.append(surface["primitive_index"])
        self.assertEqual(len(indices), len(set(indices)), "surfaces must own distinct primitives")

    def test_blender_pivots_are_the_render_body_hinges_converted(self):
        for surface in ac.moving_surfaces(self.manifest):
            expected = ac.render_body_to_blender(surface["hinge_origin_render_body_m"])
            for axis in range(3):
                self.assertAlmostEqual(
                    surface["blender_pivot"][axis], expected[axis], places=9,
                    msg=f"{surface['semantic_id']} pivot axis {axis}",
                )

    def test_normalized_hinge_axes_match_the_authored_axes(self):
        import math

        for surface in ac.moving_surfaces(self.manifest):
            axis = surface["hinge_axis_render_body"]
            length = math.sqrt(sum(v * v for v in axis))
            self.assertGreater(length, 1.0e-9, f"{surface['semantic_id']} hinge axis is degenerate")
            for index in range(3):
                self.assertAlmostEqual(
                    surface["hinge_axis_normalized_render_body"][index], axis[index] / length,
                    places=6, msg=f"{surface['semantic_id']} axis component {index}",
                )

    def test_frame_conversion_round_trips(self):
        for point in ([0.0, 0.0, 0.0], [1.0, -2.0, 3.0], [-0.92, 0.795, 0.91]):
            back = ac.blender_to_render_body(ac.render_body_to_blender(point))
            for axis in range(3):
                self.assertAlmostEqual(back[axis], point[axis], places=12)

    def test_manifest_declares_itself_presentation_only(self):
        self.assertIs(self.manifest["consumed_by_simulation"], False)
        self.assertEqual(self.manifest["purpose"], "presentation_only")
        self.assertIs(self.manifest["runtime_mapping_authority"]["changed_by_rv2_7a"], False)


class ManifestAgainstRuntimeContract(unittest.TestCase):
    """The manifest must not drift from the runtime authorities it describes."""

    @classmethod
    def setUpClass(cls):
        cls.manifest = ac.load_manifest(ASSET_ID)
        cls.components = ac.components(cls.manifest)
        with open(MODEL_JSON, "r", encoding="utf-8") as handle:
            cls.model_json = json.load(handle)

    def test_moving_surfaces_match_model_json_presentation_exactly(self):
        presentation = self.model_json["presentation"]
        self.assertEqual(presentation["glb_path"], os.path.basename(PRODUCTION_GLB))
        declared = {
            entry["surface"]: entry for entry in presentation["articulated_surfaces"]
        }
        surfaces = ac.moving_surfaces(self.manifest)
        self.assertEqual(len(declared), len(surfaces))
        for surface in surfaces:
            entry = declared.get(surface["surface_id"])
            self.assertIsNotNone(entry, f"model.json has no surface {surface['surface_id']!r}")
            self.assertEqual(
                entry["visual_primitive_index"], surface["primitive_index"],
                f"{surface['surface_id']} primitive index",
            )
            self.assertEqual(
                entry["control_surface_binding_id"], surface["control_surface_binding_id"]
            )
            self.assertEqual(entry["hinge_origin_render_body_m"], surface["hinge_origin_render_body_m"])
            self.assertEqual(entry["hinge_axis_render_body"], surface["hinge_axis_render_body"])
            self.assertEqual(entry["visual_gain"], surface["visual_gain"])

    def test_runtime_mapping_authority_block_matches_model_json(self):
        authority = self.manifest["runtime_mapping_authority"]["model_json_presentation"]
        for entry in self.model_json["presentation"]["articulated_surfaces"]:
            self.assertEqual(authority[entry["surface"]], entry["visual_primitive_index"])
        self.assertEqual(len(authority), len(self.model_json["presentation"]["articulated_surfaces"]))

    def test_legacy_node_names_match_the_renderer_expected_parts_order(self):
        with open(RENDERER_ASSET_TEST, "r", encoding="utf-8") as handle:
            source = handle.read()
        match = re.search(
            r"const EXPECTED_PARTS:\s*\[&str;\s*(\d+)\]\s*=\s*\[(.*?)\];", source, re.DOTALL
        )
        self.assertIsNotNone(match, "EXPECTED_PARTS not found in aircraft_asset_g3c.rs")
        count = int(match.group(1))
        parts = re.findall(r'"([^"]+)"', match.group(2))
        self.assertEqual(len(parts), count)
        self.assertEqual([c["legacy_node_name"] for c in self.components], parts)


class ProductionAssetGuard(unittest.TestCase):
    """The committed runtime asset must stay exactly as the manifest records it."""

    @classmethod
    def setUpClass(cls):
        cls.manifest = ac.load_manifest(ASSET_ID)
        cls.counts = cls.manifest["counts"]["production"]

    def test_production_glb_is_present(self):
        self.assertTrue(os.path.isfile(PRODUCTION_GLB))

    def test_production_glb_sha256_and_size_match_the_manifest(self):
        self.assertEqual(sha256_of(PRODUCTION_GLB), self.counts["sha256"])
        self.assertEqual(os.path.getsize(PRODUCTION_GLB), self.counts["byte_size"])

    def test_production_glb_passes_the_offline_validator(self):
        report = validate_glb.validate(
            PRODUCTION_GLB, ac.manifest_path(ASSET_ID), "production"
        )
        self.assertTrue(report.ok, "\n".join(report.errors))

    def test_validator_cli_reports_success_with_exit_code_zero(self):
        completed = subprocess.run(
            [
                sys.executable, os.path.join(SCRIPT_DIR, "validate_glb.py"),
                PRODUCTION_GLB, "--manifest", ac.manifest_path(ASSET_ID),
                "--profile", "production", "--quiet",
            ],
            capture_output=True, text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)

    def test_blender_source_is_not_a_runtime_dependency(self):
        needles = ("acro_electric_01.blend", "models/acro_electric_01/source")
        hits = []
        crates_root = os.path.join(ac.REPO_ROOT, "crates")
        for directory, _dirs, files in os.walk(crates_root):
            if os.sep + "target" + os.sep in directory + os.sep:
                continue
            for name in files:
                if not name.endswith((".rs", ".toml", ".json", ".wgsl")):
                    continue
                path = os.path.join(directory, name)
                try:
                    with open(path, "r", encoding="utf-8") as handle:
                        text = handle.read()
                except (OSError, UnicodeDecodeError):
                    continue
                for needle in needles:
                    if needle in text:
                        hits.append(f"{os.path.relpath(path, ac.REPO_ROOT)}: {needle}")
        self.assertEqual(hits, [], f".blend source leaked into runtime crates: {hits}")


class ValidatorFailsClosed(unittest.TestCase):
    """Negative tests: a broken asset or a broken manifest must not pass."""

    @classmethod
    def setUpClass(cls):
        cls.manifest_path = ac.manifest_path(ASSET_ID)
        with open(PRODUCTION_GLB, "rb") as handle:
            cls.raw = handle.read()
        cls.temp_dir = tempfile.mkdtemp(prefix="rv2_7a_validator_")

    def _write(self, name: str, data: bytes) -> str:
        path = os.path.join(self.temp_dir, name)
        with open(path, "wb") as handle:
            handle.write(data)
        return path

    def _assert_fails(self, path: str, manifest_path: str | None = None, profile: str = "production"):
        report = validate_glb.validate(path, manifest_path or self.manifest_path, profile)
        self.assertFalse(report.ok, f"expected failure for {path}, got PASS")
        return report

    def test_truncated_container_fails(self):
        report = self._assert_fails(self._write("truncated.glb", self.raw[:-64]))
        self.assertTrue(any("length" in e or "chunk" in e for e in report.errors), report.errors)

    def test_corrupted_magic_fails(self):
        self.assertEqual(self.raw[:4], b"glTF")
        self._assert_fails(self._write("bad_magic.glb", b"\x00\x00\x00\x00" + self.raw[4:]))

    def test_empty_file_fails(self):
        self._assert_fails(self._write("empty.glb", b""))

    def test_missing_file_fails(self):
        report = validate_glb.validate(
            os.path.join(self.temp_dir, "does_not_exist.glb"), self.manifest_path
        )
        self.assertFalse(report.ok)

    def test_unknown_material_name_fails(self):
        needle = b'"Tire Rubber"'
        self.assertIn(needle, self.raw)
        patched = self.raw.replace(needle, b'"Tire RubbeX"', 1)
        self.assertEqual(len(patched), len(self.raw))
        report = self._assert_fails(self._write("renamed_material.glb", patched))
        self.assertTrue(
            any("Tire RubbeX" in e for e in report.errors), report.errors
        )

    def test_reordered_node_names_break_the_primitive_mapping(self):
        # Swap two equally long node names so the container stays byte-aligned.
        # 'Rudder' (primitive 11) and 'Canopy' (primitive 4) are both 6 chars;
        # find() lands on the opening quote, so the name bytes start one later.
        first = self.raw.find(b'"Rudder"')
        second = self.raw.find(b'"Canopy"')
        self.assertGreater(first, 0)
        self.assertGreater(second, 0)
        patched = bytearray(self.raw)
        patched[first + 1:first + 7] = b"Canopy"
        patched[second + 1:second + 7] = b"Rudder"
        self.assertEqual(len(patched), len(self.raw))
        report = self._assert_fails(self._write("swapped_nodes.glb", bytes(patched)))
        self.assertTrue(
            any("primitive 11" in e and "Rudder" in e for e in report.errors), report.errors
        )
        self.assertTrue(
            any("primitive 4" in e and "Canopy" in e for e in report.errors), report.errors
        )

    def test_wrong_moving_surface_index_in_the_manifest_fails(self):
        with open(self.manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        for surface in manifest["moving_surfaces"]:
            if surface["surface_id"] == "elevator":
                surface["primitive_index"] = 10
                break
        for component in manifest["components"]:
            if component["semantic_id"] == "ELEVATOR":
                component["primitive_index"] = 10
        broken_path = os.path.join(self.temp_dir, "broken_manifest.json")
        with open(broken_path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle)
        report = self._assert_fails(PRODUCTION_GLB, broken_path)
        self.assertTrue(
            any("elevator" in e for e in report.errors), report.errors
        )

    def test_unknown_profile_fails(self):
        report = validate_glb.validate(PRODUCTION_GLB, self.manifest_path, "does_not_exist")
        self.assertFalse(report.ok)

    def test_texcoord_contract_can_be_tightened(self):
        with open(self.manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        manifest["attribute_contract"]["texcoord_0_required"] = True
        strict_path = os.path.join(self.temp_dir, "uv_required.json")
        with open(strict_path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle)
        report = self._assert_fails(PRODUCTION_GLB, strict_path)
        self.assertTrue(
            any("TEXCOORD_0" in e for e in report.errors), report.errors
        )

    def test_absurd_bounds_limit_is_enforced(self):
        with open(self.manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        manifest["bounds"]["absurd_extent_limit_m"] = 0.5
        tight_path = os.path.join(self.temp_dir, "tight_bounds.json")
        with open(tight_path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle)
        report = self._assert_fails(PRODUCTION_GLB, tight_path)
        self.assertTrue(
            any("absurd-bounds" in e for e in report.errors), report.errors
        )

    def test_orientation_convention_is_enforced(self):
        with open(self.manifest_path, "r", encoding="utf-8") as handle:
            manifest = json.load(handle)
        manifest["orientation_checks"]["foremost_component"] = "VTAIL_FIXED"
        wrong_path = os.path.join(self.temp_dir, "wrong_nose.json")
        with open(wrong_path, "w", encoding="utf-8") as handle:
            json.dump(manifest, handle)
        report = self._assert_fails(PRODUCTION_GLB, wrong_path)
        self.assertTrue(
            any("nose_forward_minus_z" in e for e in report.errors), report.errors
        )


class FingerprintIsStable(unittest.TestCase):
    def test_fingerprint_of_the_production_asset_is_deterministic(self):
        first = validate_glb.fingerprint(PRODUCTION_GLB)
        second = validate_glb.fingerprint(PRODUCTION_GLB)
        self.assertEqual(first, second)
        self.assertEqual(first["primitive_count"], 21)
        self.assertEqual(first["material_names"][0], "Airframe Pearl")
        self.assertEqual(first["bounds_min"], [-0.92, -0.117, -0.89])
        self.assertEqual(first["bounds_max"], [0.92, 0.795, 0.91])
        self.assertEqual(
            [p["node_names"] for p in first["primitives"]][6], ["LeftAileron"]
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
