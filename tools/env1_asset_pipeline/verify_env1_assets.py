"""ENV1-A: fail-closed verification of the open-asset provenance chain.

Checks, in order:

1. the committed manifest satisfies its own contract;
2. every committed runtime PNG still matches the recorded size, SHA-256 and
   PNG header (dimensions, bit depth, colour type);
3. if the gitignored source cache is populated, every source file still matches
   its recorded SHA-256 and the MD5/size the Poly Haven API published —
   a mismatch is a hard failure, never a warning;
4. with ``--reprocess``, that re-running the Rust processor over the cached
   sources reproduces the committed runtime bytes exactly.

Exit codes: 0 verified, 1 mismatch or contract violation, 2 missing input.

Usage:
    python -X utf8 tools/env1_asset_pipeline/verify_env1_assets.py
    python -X utf8 tools/env1_asset_pipeline/verify_env1_assets.py --reprocess
"""

from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import struct
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from env1_assets import (  # noqa: E402
    EXPECTED_BIT_DEPTH,
    EXPECTED_COLOR_TYPE,
    SOURCE_FORMAT,
    SOURCE_RESOLUTION,
    Env1AssetError,
    configure_streams,
    describe_file,
    load_manifest,
    manifest_path,
    md5_file,
    repo_root,
    sha256_file,
    source_cache_dir_for,
    validate_manifest,
)

RECEIPT_NAME = f"fetch_receipt_{SOURCE_RESOLUTION}_{SOURCE_FORMAT}.json"


class Reporter:
    """Collects pass/fail lines and remembers whether anything failed."""

    def __init__(self) -> None:
        self.failures: list[str] = []
        self.skipped: list[str] = []

    def ok(self, message: str) -> None:
        print(f"  [OK]   {message}")

    def fail(self, message: str) -> None:
        self.failures.append(message)
        print(f"  [FAIL] {message}")

    def skip(self, message: str) -> None:
        self.skipped.append(message)
        print(f"  [SKIP] {message}")

    @property
    def failed(self) -> bool:
        return bool(self.failures)


def verify_runtime_outputs(root: pathlib.Path, asset: dict, report: Reporter) -> None:
    print("\nruntime outputs (committed):")
    for entry in asset.get("runtime_outputs", []):
        role = entry.get("role")
        path = root / entry.get("path", "")
        if not path.is_file():
            report.fail(f"{role}: committed runtime map is missing at {entry.get('path')}")
            continue
        measured = describe_file(path)
        if measured["sha256"] != entry.get("sha256"):
            report.fail(
                f"{role}: sha256 {measured['sha256'][:16]}... != recorded "
                f"{str(entry.get('sha256'))[:16]}..."
            )
            continue
        if measured["byte_size"] != entry.get("byte_size"):
            report.fail(
                f"{role}: byte_size {measured['byte_size']} != recorded "
                f"{entry.get('byte_size')}"
            )
            continue
        for field in ("width", "height", "png_bit_depth", "png_color_type"):
            if measured[field] != entry.get(field):
                report.fail(f"{role}: {field} {measured[field]} != recorded {entry.get(field)}")
                break
        else:
            expected_ct = EXPECTED_COLOR_TYPE.get(role)
            expected_bd = EXPECTED_BIT_DEPTH["runtime"]
            if expected_ct is not None and measured["png_color_type"] != expected_ct:
                report.fail(
                    f"{role}: PNG color type {measured['png_color_type']} != the "
                    f"{expected_ct} the recipe documents"
                )
            elif measured["png_bit_depth"] != expected_bd:
                report.fail(
                    f"{role}: PNG bit depth {measured['png_bit_depth']} != {expected_bd}"
                )
            else:
                report.ok(
                    f"{role}: {path.name} {measured['width']}x{measured['height']} "
                    f"depth={measured['png_bit_depth']} ct={measured['png_color_type']} "
                    f"{measured['byte_size']} bytes sha256={measured['sha256'][:16]}..."
                )


def verify_sources(root: pathlib.Path, asset: dict, report: Reporter) -> None:
    print("\nsource maps (gitignored cache):")
    slug = asset.get("slug") or "sparse_grass"
    cache = source_cache_dir_for(root, slug)
    present = [
        (cache / SOURCE_RESOLUTION / entry["file_name"]).is_file()
        for entry in asset.get("source_files", [])
    ]
    if not any(present):
        report.skip(
            f"source cache not populated at {cache}; source digests could not be "
            "re-verified. Run fetch_polyhaven_asset.py to restore them. The "
            "recorded digests remain the authority for what was processed."
        )
        return

    for entry in asset.get("source_files", []):
        role = entry.get("role")
        path = cache / SOURCE_RESOLUTION / entry["file_name"]
        if not path.is_file():
            report.fail(f"{role}: source file missing at {path}")
            continue
        actual_sha = sha256_file(path)
        if actual_sha != entry.get("local_sha256"):
            report.fail(
                f"{role}: SOURCE SHA MISMATCH - on disk {actual_sha[:16]}..., "
                f"manifest {str(entry.get('local_sha256'))[:16]}..."
            )
            continue
        actual_size = path.stat().st_size
        if actual_size != entry.get("local_size"):
            report.fail(
                f"{role}: source size {actual_size} != recorded {entry.get('local_size')}"
            )
            continue
        api_md5 = entry.get("api_md5")
        if api_md5 is not None:
            actual_md5 = md5_file(path)
            if actual_md5 != api_md5:
                report.fail(
                    f"{role}: source MD5 {actual_md5} != the value the Poly Haven "
                    f"API published ({api_md5})"
                )
                continue
        report.ok(
            f"{role}: {entry['file_name']} sha256={actual_sha[:16]}... matches the "
            "manifest and the API digest"
        )


def verify_reprocess(root: pathlib.Path, asset: dict, report: Reporter) -> None:
    """Re-run the Rust processor and require byte-identical output."""
    print("\nreprocessing determinism:")
    slug = asset.get("slug") or "sparse_grass"
    cache = source_cache_dir_for(root, slug) / SOURCE_RESOLUTION
    if not cache.is_dir():
        report.skip(f"source cache absent at {cache}; cannot reprocess")
        return
    if shutil.which("cargo") is None:
        report.skip("cargo is not on PATH; cannot reprocess")
        return

    temporary = pathlib.Path(tempfile.mkdtemp(prefix="env1a_reprocess_"))
    try:
        command = [
            "cargo",
            "run",
            "-q",
            "-p",
            "renderer",
            "--bin",
            "process_env1_terrain_material",
            "--",
            "--asset",
            slug,
            "--out-dir",
            str(temporary),
        ]
        completed = subprocess.run(
            command,
            cwd=root,
            capture_output=True,
            text=True,
            check=False,
        )
        if completed.returncode != 0:
            report.fail(
                f"the processor exited {completed.returncode}: "
                f"{completed.stderr.strip()[:400]}"
            )
            return

        expected_dir = root / (
            asset.get("runtime_outputs") or [{}]
        )[0].get("path", "").replace("\\", "/").rsplit("/", 1)[0]
        reproduced = sorted(temporary.glob("*.png"))
        if not reproduced:
            report.fail("the processor wrote no PNGs")
            return
        for produced in reproduced:
            reference = expected_dir / produced.name
            if not reference.is_file():
                report.fail(f"reproduced {produced.name} has no committed counterpart")
                continue
            if sha256_file(produced) != sha256_file(reference):
                report.fail(
                    f"{produced.name}: reprocessing is NOT byte-identical to the "
                    "committed runtime map"
                )
            else:
                report.ok(f"{produced.name}: reprocessing is byte-identical")
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


TERRAIN_SCALE_PATTERN = re.compile(
    r"pub const DEFAULT_TERRAIN_TEXTURE_SCALE_M: f32 = ([0-9.]+);"
)


def verify_tile_scale_binding(root: pathlib.Path, asset: dict, report: Reporter) -> None:
    """Prove the manifest's tile scale is the one the renderer actually compiles.

    The link is checked here and by a Rust regression test, never by parsing
    JSON at render time.
    """
    print("\nterrain tile scale binding:")
    binding = asset.get("runtime_binding") or {}
    declared = binding.get("terrain_base_tile_scale_m")
    api = asset.get("api") or {}
    physical = api.get("physical_dimensions_m")
    constant = binding.get("constant") or "DEFAULT_TERRAIN_TEXTURE_SCALE_M"
    defined_in = binding.get("defined_in") or "crates/renderer/src/terrain.rs"
    source = root / defined_in
    if not source.is_file():
        report.fail(f"{source} is missing; the tile scale cannot be verified")
        return
    pattern = re.compile(rf"pub const {re.escape(constant)}: f32 = ([0-9.]+);")
    match = pattern.search(source.read_text(encoding="utf-8"))
    if match is None:
        report.fail(f"{constant} was not found in {defined_in}")
        return
    compiled = float(match.group(1))
    # The renderer compiles an f32 tile scale; the manifest records the exact
    # f64 derivation of the provider's millimetre dimensions. Compare at f32
    # precision on BOTH sides: the Rust literal `3.15` becomes the f32 image
    # 3.1500000953674316, and a span such as 3150.0000095 mm has no exact f32
    # image either. The sub-micrometre residue is meaningless at tile scale.
    to_f32 = lambda value: struct.unpack("<f", struct.pack("<f", value))[0]
    compiled_f32 = to_f32(compiled)
    declared_f32 = to_f32(float(declared or 0.0))
    if declared is None or abs(compiled_f32 - declared_f32) > 1e-9:
        report.fail(
            f"{defined_in} compiles {constant} = {compiled} m but the "
            f"manifest declares {declared} m"
        )
        return
    if not (isinstance(physical, list) and len(physical) == 2):
        report.fail("api.physical_dimensions_m is missing; the binding cannot be checked")
        return
    for axis, span in enumerate(physical):
        span_f32 = to_f32(float(span))
        if abs(compiled_f32 - span_f32) > 1e-9:
            report.fail(
                f"tile scale {compiled} m does not equal the asset's physical span "
                f"{span} m on axis {axis}"
            )
            return
    report.ok(
        f"{constant} = {compiled} m == the asset's physical span "
        f"{api.get('dimensions')} {api.get('dimensions_unit')} == {physical} m"
    )


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="verify_env1_assets.py",
        description="Fail-closed verification of the ENV1 open-asset chain.",
    )
    parser.add_argument(
        "--manifest",
        metavar="PATH",
        default=None,
        help=f"provenance manifest (default {manifest_path()})",
    )
    parser.add_argument(
        "--reprocess",
        action="store_true",
        help=(
            "also re-run the Rust processor over the cached sources and require "
            "byte-identical output (needs cargo and a populated source cache)"
        ),
    )
    args = parser.parse_args(argv)

    path = (
        pathlib.Path(args.manifest).resolve() if args.manifest else manifest_path()
    )
    root = repo_root()
    report = Reporter()

    try:
        manifest = load_manifest(path)
    except Env1AssetError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code

    print("ENV1-A provenance verification")
    print(f"  manifest: {path}")
    print(f"  workspace root: {root}")

    print("\nmanifest contract:")
    errors = validate_manifest(manifest)
    if errors:
        for error in errors:
            report.fail(error)
    else:
        report.ok(f"manifest_version {manifest.get('manifest_version')} contract satisfied")

    for asset in manifest.get("assets", []) if isinstance(manifest.get("assets"), list) else []:
        print(f"\nasset {asset.get('asset_id')} ({asset.get('slug')}, {asset.get('license')})")
        verify_runtime_outputs(root, asset, report)
        verify_tile_scale_binding(root, asset, report)
        verify_sources(root, asset, report)
        if args.reprocess:
            verify_reprocess(root, asset, report)

    print()
    if report.skipped:
        print(f"{len(report.skipped)} check(s) skipped")
    if report.failed:
        print(f"FAIL: {len(report.failures)} verification failure(s)", file=sys.stderr)
        return 1
    print("all ENV1-A provenance checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
