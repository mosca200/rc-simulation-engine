"""Tests for the ENV1 open-asset provenance tooling.

Run from the workspace root:

    python -X utf8 -m unittest tools/env1_asset_pipeline/test_env1_assets.py -v

Standard library only. The tests that need the gitignored Poly Haven source
cache skip themselves when it is absent, so the suite is green on a fresh clone
while still exercising the fail-closed paths wherever the sources exist.
"""

from __future__ import annotations

import io
import json
import pathlib
import struct
import sys
import tempfile
import unittest
import zlib
from contextlib import redirect_stderr, redirect_stdout

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import build_manifest  # noqa: E402
import env1_assets  # noqa: E402
import verify_env1_assets  # noqa: E402

REPO_ROOT = env1_assets.repo_root()
MANIFEST = env1_assets.manifest_path()
SOURCE_CACHE = env1_assets.source_cache_dir() / env1_assets.SOURCE_RESOLUTION


def png_bytes(width: int, height: int, bit_depth: int, color_type: int) -> bytes:
    """Build the smallest byte string whose IHDR carries the given fields."""
    ihdr = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    chunk = struct.pack(">I", len(ihdr)) + b"IHDR" + ihdr
    chunk += struct.pack(">I", zlib.crc32(b"IHDR" + ihdr) & 0xFFFFFFFF)
    return b"\x89PNG\r\n\x1a\n" + chunk


class TestPngHeader(unittest.TestCase):
    def test_header_fields_are_read_without_an_image_library(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "probe.png"
            path.write_bytes(png_bytes(2048, 2048, 8, 6))
            header = env1_assets.read_png_header(path)
        self.assertEqual(header["width"], 2048)
        self.assertEqual(header["height"], 2048)
        self.assertEqual(header["bit_depth"], 8)
        self.assertEqual(header["color_type"], 6)
        self.assertEqual(header["interlace"], 0)

    def test_non_png_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "not_a_png.png"
            path.write_bytes(b"definitely not a png, but long enough to read")
            with self.assertRaises(env1_assets.Env1AssetError):
                env1_assets.read_png_header(path)

    def test_truncated_header_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "short.png"
            path.write_bytes(b"\x89PNG\r\n\x1a\n")
            with self.assertRaises(env1_assets.Env1AssetError):
                env1_assets.read_png_header(path)

    def test_digests_are_lowercase_hex_of_the_right_width(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "payload.bin"
            path.write_bytes(b"env1-a")
            digest = env1_assets.sha256_file(path)
        self.assertRegex(digest, r"^[0-9a-f]{64}$")
        self.assertEqual(digest, env1_assets.sha256_bytes(b"env1-a"))


class TestCommittedManifest(unittest.TestCase):
    """The committed manifest is itself part of the contract."""

    @classmethod
    def setUpClass(cls) -> None:
        if not MANIFEST.is_file():
            raise unittest.SkipTest(f"no committed manifest at {MANIFEST}")
        cls.manifest = env1_assets.load_manifest(MANIFEST)

    def test_committed_manifest_satisfies_its_own_contract(self) -> None:
        self.assertEqual(env1_assets.validate_manifest(self.manifest), [])

    def test_sparse_grass_is_registered_with_the_required_provenance(self) -> None:
        asset = self.manifest["assets"][0]
        self.assertEqual(asset["asset_id"], "ENV1-GND-01")
        self.assertEqual(asset["provider"], "Poly Haven")
        self.assertEqual(asset["slug"], "sparse_grass")
        self.assertEqual(asset["source_page"], "https://polyhaven.com/a/sparse_grass")
        self.assertEqual(asset["license"], "CC0")
        self.assertEqual(asset["authors"], {"Amal Kumar": "All"})
        self.assertEqual(asset["source_resolution"], "4k")
        self.assertEqual(asset["source_format"], "png")

    def test_every_source_file_carries_a_local_sha256_and_api_metadata(self) -> None:
        sources = self.manifest["assets"][0]["source_files"]
        self.assertEqual(len(sources), 3)
        for entry in sources:
            self.assertRegex(entry["local_sha256"], r"^[0-9a-f]{64}$")
            self.assertRegex(entry["api_md5"], r"^[0-9a-f]{32}$")
            self.assertEqual(entry["api_size"], entry["local_size"])
            self.assertEqual(entry["local_md5"], entry["api_md5"])
            self.assertTrue(entry["size_verified"])
            self.assertTrue(entry["md5_verified"])
            self.assertEqual(entry["png_bit_depth"], 16, "sources are 16-bit")
            self.assertEqual((entry["width"], entry["height"]), (4096, 4096))

    def test_runtime_outputs_record_paths_and_digests(self) -> None:
        outputs = self.manifest["assets"][0]["runtime_outputs"]
        by_role = {entry["role"]: entry for entry in outputs}
        self.assertEqual(set(by_role), {"base_color", "normal", "roughness"})
        for role, entry in by_role.items():
            self.assertRegex(entry["sha256"], r"^[0-9a-f]{64}$")
            self.assertTrue(entry["path"].startswith("crates/renderer/assets/env1/terrain/sparse_grass/"))
            self.assertEqual((entry["width"], entry["height"]), (2048, 2048))
            self.assertEqual(entry["png_bit_depth"], 8)
            self.assertEqual(
                entry["png_color_type"],
                env1_assets.EXPECTED_COLOR_TYPE[role],
                f"{role} must use the documented PNG colour type",
            )
        self.assertEqual(by_role["base_color"]["color_space"], "srgb")
        self.assertEqual(by_role["normal"]["color_space"], "linear")
        self.assertEqual(by_role["roughness"]["color_space"], "linear")
        self.assertEqual(by_role["roughness"]["channels"], "r8")

    def test_the_api_dimensions_carry_the_documented_millimetre_unit(self) -> None:
        api = self.manifest["assets"][0]["api"]
        # Raw API value preserved verbatim, provider-documented unit preserved,
        # derived metres explicit. No guessed unit and no hidden conversion.
        self.assertEqual(api["dimensions"], [2000, 2000], "recorded verbatim from the API")
        self.assertEqual(api["dimensions_unit"], "mm")
        self.assertEqual(api["physical_dimensions_m"], [2.0, 2.0])
        self.assertIn("millimetre", api["dimensions_unit_note"].lower())
        self.assertEqual(api["files_hash"], "1293431c7316b89282b883ff760be963802f9000")

    def test_the_terrain_tile_scale_is_bound_to_the_physical_span(self) -> None:
        asset = self.manifest["assets"][0]
        binding = asset["runtime_binding"]
        self.assertEqual(binding["terrain_base_tile_scale_m"], 2.0)
        self.assertEqual(binding["constant"], "DEFAULT_TERRAIN_TEXTURE_SCALE_M")
        self.assertEqual(binding["defined_in"], "crates/renderer/src/terrain.rs")
        self.assertEqual(
            binding["terrain_base_tile_scale_m"],
            asset["api"]["physical_dimensions_m"][0],
            "one texture tile must cover exactly the scanned area",
        )

    def test_the_compiled_tile_scale_matches_the_manifest(self) -> None:
        source = (REPO_ROOT / "crates/renderer/src/terrain.rs").read_text(encoding="utf-8")
        match = verify_env1_assets.TERRAIN_SCALE_PATTERN.search(source)
        self.assertIsNotNone(match, "DEFAULT_TERRAIN_TEXTURE_SCALE_M must exist")
        self.assertEqual(float(match.group(1)), 2.0)

    def test_the_physical_metre_conversion_is_explicit_and_fail_closed(self) -> None:
        self.assertEqual(env1_assets.physical_dimensions_m([2000, 2000]), [2.0, 2.0])
        self.assertEqual(env1_assets.physical_dimensions_m([8192, 4096]), [8.192, 4.096])
        with self.assertRaises(env1_assets.Env1AssetError):
            env1_assets.physical_dimensions_m([2000, 2000], "cm")
        with self.assertRaises(env1_assets.Env1AssetError):
            env1_assets.physical_dimensions_m([2000, 2000], "")

    def test_attribution_required_by_the_api_terms_is_recorded(self) -> None:
        self.assertIn("Poly Haven", self.manifest["attribution"])
        self.assertIn("Powered by", self.manifest["attribution"])


class TestManifestContractEnforcement(unittest.TestCase):
    """Mutating a valid manifest must always be caught."""

    @classmethod
    def setUpClass(cls) -> None:
        if not MANIFEST.is_file():
            raise unittest.SkipTest(f"no committed manifest at {MANIFEST}")
        cls.valid = env1_assets.load_manifest(MANIFEST)

    def mutate(self, callback) -> dict:
        clone = json.loads(json.dumps(self.valid))
        callback(clone)
        return clone

    def assertInvalid(self, manifest: dict, needle: str) -> None:
        errors = env1_assets.validate_manifest(manifest)
        self.assertTrue(errors, "the mutation should have been rejected")
        self.assertTrue(
            any(needle in error for error in errors),
            f"expected an error mentioning {needle!r}, got {errors}",
        )

    def test_a_missing_required_field_is_reported(self) -> None:
        for field in env1_assets.REQUIRED_ASSET_FIELDS:
            with self.subTest(field=field):
                self.assertInvalid(
                    self.mutate(lambda m, f=field: m["assets"][0].pop(f)), field
                )

    def test_a_missing_source_file_field_is_reported(self) -> None:
        for field in env1_assets.REQUIRED_SOURCE_FILE_FIELDS:
            with self.subTest(field=field):
                self.assertInvalid(
                    self.mutate(lambda m, f=field: m["assets"][0]["source_files"][0].pop(f)),
                    f"source_files[0].{field}",
                )

    def test_a_missing_runtime_field_is_reported(self) -> None:
        for field in env1_assets.REQUIRED_RUNTIME_FIELDS:
            with self.subTest(field=field):
                self.assertInvalid(
                    self.mutate(lambda m, f=field: m["assets"][0]["runtime_outputs"][0].pop(f)),
                    f"runtime_outputs[0].{field}",
                )

    def test_an_unverified_source_download_fails_closed(self) -> None:
        for flag in ("size_verified", "md5_verified"):
            with self.subTest(flag=flag):
                self.assertInvalid(
                    self.mutate(
                        lambda m, f=flag: m["assets"][0]["source_files"][0].__setitem__(f, False)
                    ),
                    flag,
                )

    def test_a_missing_local_source_digest_fails_closed(self) -> None:
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["source_files"][0].__setitem__("local_sha256", None)
            ),
            "local_sha256",
        )

    def test_a_non_cc0_license_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(lambda m: m["assets"][0].__setitem__("license", "CC-BY-4.0")),
            "license",
        )

    def test_a_unit_other_than_millimetres_is_rejected(self) -> None:
        for unit in ("cm", "m", "inches", "", None):
            with self.subTest(unit=unit):
                self.assertInvalid(
                    self.mutate(
                        lambda m, u=unit: m["assets"][0]["api"].__setitem__(
                            "dimensions_unit", u
                        )
                    ),
                    "dimensions_unit",
                )

    def test_an_inconsistent_unit_and_raw_value_pair_is_rejected(self) -> None:
        # [2000, 2000] with "cm" would silently claim a 20 m scan.
        mutated = self.mutate(
            lambda m: m["assets"][0]["api"].__setitem__("dimensions_unit", "cm")
        )
        errors = env1_assets.validate_manifest(mutated)
        self.assertTrue(any("dimensions_unit" in e for e in errors), errors)

    def test_a_wrong_derived_metre_value_is_rejected(self) -> None:
        for bad in ([20.0, 20.0], [2.0, 3.0], [0.2, 0.2], [2000.0, 2000.0]):
            with self.subTest(physical=bad):
                self.assertInvalid(
                    self.mutate(
                        lambda m, b=bad: m["assets"][0]["api"].__setitem__(
                            "physical_dimensions_m", b
                        )
                    ),
                    "physical_dimensions_m",
                )

    def test_a_missing_derived_metre_value_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(lambda m: m["assets"][0]["api"].pop("physical_dimensions_m")),
            "physical_dimensions_m",
        )

    def test_a_tile_scale_that_disagrees_with_the_physical_span_is_rejected(self) -> None:
        for bad in (4.0, 1.0, 20.0, 0.0):
            with self.subTest(scale=bad):
                self.assertInvalid(
                    self.mutate(
                        lambda m, b=bad: m["assets"][0]["runtime_binding"].__setitem__(
                            "terrain_base_tile_scale_m", b
                        )
                    ),
                    "runtime_binding",
                )

    def test_a_missing_runtime_binding_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(lambda m: m["assets"][0].pop("runtime_binding")),
            "runtime_binding",
        )

    def test_an_incomplete_runtime_binding_is_rejected(self) -> None:
        for field in ("constant", "defined_in", "relationship"):
            with self.subTest(field=field):
                self.assertInvalid(
                    self.mutate(
                        lambda m, f=field: m["assets"][0]["runtime_binding"].pop(f)
                    ),
                    f"runtime_binding.{field}",
                )

    def test_an_asset_id_outside_the_scheme_is_rejected(self) -> None:
        for bad in ("GROUND-01", "env1-gnd-01", "ENV1-GND-1", "ENV1-GROUND-01"):
            with self.subTest(asset_id=bad):
                self.assertInvalid(
                    self.mutate(lambda m, b=bad: m["assets"][0].__setitem__("asset_id", b)),
                    "asset_id",
                )

    def test_a_duplicate_asset_id_is_rejected(self) -> None:
        def duplicate(manifest: dict) -> None:
            manifest["assets"].append(json.loads(json.dumps(manifest["assets"][0])))

        self.assertInvalid(self.mutate(duplicate), "duplicate")

    def test_a_wrong_recipe_version_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["processing"].__setitem__("recipe_version", 999)
            ),
            "recipe_version",
        )

    def test_a_wrong_runtime_edge_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["runtime_outputs"][0].__setitem__("width", 1024)
            ),
            "width",
        )

    def test_a_wrong_png_colour_type_is_rejected(self) -> None:
        # Roughness must stay single-channel so the runtime can upload R8.
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["runtime_outputs"][2].__setitem__("png_color_type", 6)
            ),
            "png_color_type",
        )

    def test_a_malformed_digest_is_rejected(self) -> None:
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["runtime_outputs"][0].__setitem__("sha256", "not-a-digest")
            ),
            "sha256",
        )

    def test_silently_dropping_a_cosmetic_edit_guard_is_rejected(self) -> None:
        # `not_applied` is the record that no baked AO / sharpening / LUT was
        # applied; shortening it must be caught rather than accepted.
        self.assertInvalid(
            self.mutate(
                lambda m: m["assets"][0]["processing"].__setitem__("not_applied", [])
            ),
            "not_applied",
        )

    def test_the_wrong_manifest_version_is_rejected(self) -> None:
        self.assertInvalid(self.mutate(lambda m: m.__setitem__("manifest_version", 2)), "manifest_version")


class TestFailClosedVerification(unittest.TestCase):
    def run_verifier(self, manifest: dict) -> tuple[int, str, str]:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "env1_open_assets.json"
            path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = verify_env1_assets.main(["--manifest", str(path)])
            return code, out.getvalue(), err.getvalue()

    def test_committed_state_verifies_clean(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        code, out, _ = self.run_verifier(env1_assets.load_manifest(MANIFEST))
        self.assertEqual(code, 0, out)
        self.assertIn("all ENV1-A provenance checks passed", out)

    def test_a_tampered_runtime_digest_fails_closed(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        manifest = env1_assets.load_manifest(MANIFEST)
        manifest["assets"][0]["runtime_outputs"][0]["sha256"] = "0" * 64
        code, out, _ = self.run_verifier(manifest)
        self.assertEqual(code, 1)
        self.assertIn("[FAIL]", out)

    def test_a_tampered_runtime_size_fails_closed(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        manifest = env1_assets.load_manifest(MANIFEST)
        manifest["assets"][0]["runtime_outputs"][1]["byte_size"] = 1
        code, out, _ = self.run_verifier(manifest)
        self.assertEqual(code, 1)
        self.assertIn("[FAIL]", out)

    def test_a_missing_runtime_map_fails_closed(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        manifest = env1_assets.load_manifest(MANIFEST)
        manifest["assets"][0]["runtime_outputs"][2]["path"] = (
            "crates/renderer/assets/env1/terrain/sparse_grass/does_not_exist.png"
        )
        manifest["assets"][0]["runtime_outputs"][2]["file_name"] = "does_not_exist.png"
        manifest["assets"][0]["runtime_outputs"][2]["sha256"] = "0" * 64
        code, out, _ = self.run_verifier(manifest)
        self.assertEqual(code, 1)
        self.assertIn("missing", out)

    @unittest.skipUnless(
        SOURCE_CACHE.is_dir() and any(SOURCE_CACHE.glob("*.png")),
        "the gitignored Poly Haven source cache is not populated",
    )
    def test_a_source_sha_mismatch_fails_closed(self) -> None:
        manifest = env1_assets.load_manifest(MANIFEST)
        manifest["assets"][0]["source_files"][0]["local_sha256"] = "f" * 64
        code, out, _ = self.run_verifier(manifest)
        self.assertEqual(code, 1)
        self.assertIn("SOURCE SHA MISMATCH", out)

    @unittest.skipUnless(
        SOURCE_CACHE.is_dir() and any(SOURCE_CACHE.glob("*.png")),
        "the gitignored Poly Haven source cache is not populated",
    )
    def test_sources_verify_against_the_recorded_digests(self) -> None:
        manifest = env1_assets.load_manifest(MANIFEST)
        code, out, _ = self.run_verifier(manifest)
        self.assertEqual(code, 0, out)
        self.assertIn("matches the manifest and the API digest", out)

    def test_a_missing_manifest_exits_with_a_distinct_code(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing = pathlib.Path(temporary) / "absent.json"
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = verify_env1_assets.main(["--manifest", str(missing)])
        self.assertEqual(code, 2)
        self.assertIn("manifest not found", err.getvalue())

    def test_a_malformed_manifest_is_not_a_crash(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "broken.json"
            path.write_text("{ this is not json", encoding="utf-8")
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = verify_env1_assets.main(["--manifest", str(path)])
        self.assertEqual(code, 1)
        self.assertIn("invalid JSON", err.getvalue())


class TestDeterministicProcessing(unittest.TestCase):
    """The runtime digests must be reproducible from the recorded sources."""

    @unittest.skipUnless(
        SOURCE_CACHE.is_dir() and any(SOURCE_CACHE.glob("*.png")),
        "the gitignored Poly Haven source cache is not populated",
    )
    def test_reprocessing_reproduces_the_committed_bytes(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        out, err = io.StringIO(), io.StringIO()
        with redirect_stdout(out), redirect_stderr(err):
            code = verify_env1_assets.main(["--reprocess"])
        self.assertEqual(code, 0, out.getvalue() + err.getvalue())
        self.assertIn("reprocessing is byte-identical", out.getvalue())
        self.assertEqual(out.getvalue().count("byte-identical"), 3)

    def test_the_manifest_digests_are_stable_across_readings(self) -> None:
        if not MANIFEST.is_file():
            self.skipTest("no committed manifest")
        manifest = env1_assets.load_manifest(MANIFEST)
        for entry in manifest["assets"][0]["runtime_outputs"]:
            path = REPO_ROOT / entry["path"]
            with self.subTest(role=entry["role"]):
                self.assertEqual(env1_assets.sha256_file(path), entry["sha256"])
                self.assertEqual(path.stat().st_size, entry["byte_size"])

    def test_building_the_manifest_is_idempotent_apart_from_the_timestamp(self) -> None:
        if not (SOURCE_CACHE.is_dir() and any(SOURCE_CACHE.glob("*.png"))):
            self.skipTest("the gitignored Poly Haven source cache is not populated")
        first = build_manifest.build_manifest_document(REPO_ROOT, env1_assets.source_cache_dir())
        second = build_manifest.build_manifest_document(REPO_ROOT, env1_assets.source_cache_dir())
        for document in (first, second):
            document.pop("generated_at_utc")
        self.assertEqual(first, second)
        self.assertEqual(env1_assets.validate_manifest(first), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
