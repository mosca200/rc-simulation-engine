"""ENV1-A: build the committed open-asset provenance manifest.

Reads the cached Poly Haven API payloads and fetch receipt, measures the
committed runtime PNGs, and writes ``docs/assets/env1/env1_open_assets.json``.
The result is validated against the manifest contract before it is written, so
an incomplete manifest can never be committed.

Requires the gitignored source cache to be populated (run
``fetch_polyhaven_asset.py`` first): source digests are mandatory provenance and
are never invented.

Usage:
    python -X utf8 tools/env1_asset_pipeline/build_manifest.py
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from env1_assets import (  # noqa: E402
    ATTRIBUTION,
    LICENSE,
    LICENSE_URL,
    MANIFEST_VERSION,
    NOT_APPLIED,
    PROVIDER,
    DIMENSIONS_UNIT,
    RECIPE_VERSION,
    RUNTIME_EDGE,
    SOURCE_EDGE,
    SOURCE_FORMAT,
    SOURCE_RESOLUTION,
    Env1AssetError,
    OpenAssetDescriptor,
    configure_streams,
    describe_file,
    descriptor_for_slug,
    load_manifest,
    manifest_path,
    physical_dimensions_m,
    runtime_dir_for,
    source_cache_dir_for,
    validate_manifest,
)

RECEIPT_NAME = f"fetch_receipt_{SOURCE_RESOLUTION}_{SOURCE_FORMAT}.json"
PROCESSOR_COMMAND = "cargo run -p renderer --bin process_env1_terrain_material"

#: The Poly Haven public API's OpenAPI schema defines a texture asset's
#: `dimensions` as "an array with the dimensions of this asset on each axis in
#: millimeters". The unit is documented by the provider, so it is recorded as
#: such and the metre equivalent is derived explicitly - never guessed, never
#: converted silently.
DIMENSIONS_UNIT_NOTE = (
    "The Poly Haven public API's OpenAPI schema defines a texture asset's "
    "`dimensions` as an array with the dimensions of the asset on each axis in "
    "millimetres. `dimensions` is therefore recorded verbatim in mm, "
    "`dimensions_unit` carries the provider-documented unit, and "
    "`physical_dimensions_m` is the explicit derived value "
    "(mm / 1000). ENV1-A ties the terrain base tile scale to this measurement: "
    "one texture tile covers exactly the scanned area."
)

LICENSE_NOTE = (
    "The Poly Haven /info API payload carries no license field. CC0 is recorded "
    "from https://polyhaven.com/license, the same citation the repository "
    "already uses in tools/vegetation_processing/PROVENANCE.md."
)


def read_json(path: pathlib.Path) -> dict:
    if not path.is_file():
        raise Env1AssetError(
            f"missing {path}. Run tools/env1_asset_pipeline/fetch_polyhaven_asset.py "
            "first: source provenance is mandatory and is never invented.",
            exit_code=2,
        )
    return json.loads(path.read_text(encoding="utf-8"))


def build_source_files(
    cache: pathlib.Path, receipt: dict, descriptor: OpenAssetDescriptor
) -> list[dict]:
    """Merge the verified fetch receipt with measured PNG header facts."""
    by_role = {entry["role"]: entry for entry in receipt.get("files", [])}
    expected_roles = [role for _, role, _ in descriptor.source_maps]
    if sorted(by_role) != sorted(expected_roles):
        raise Env1AssetError(
            f"the fetch receipt covers roles {sorted(by_role)} but the recipe "
            f"requires {sorted(expected_roles)}"
        )

    sources: list[dict] = []
    for map_type, role, file_name in descriptor.source_maps:
        entry = dict(by_role[role])
        if entry.get("map_type") != map_type or entry.get("file_name") != file_name:
            raise Env1AssetError(
                f"receipt entry for {role!r} disagrees with the documented "
                f"source map ({map_type!r}, {file_name!r})"
            )
        local = cache / SOURCE_RESOLUTION / file_name
        measured = describe_file(local)
        entry["local_path"] = (
            f"{descriptor.source_cache_relative}/{SOURCE_RESOLUTION}/{file_name}"
        )
        entry["local_path_note"] = (
            "Under the gitignored source cache; not committed. Re-acquire with "
            "fetch_polyhaven_asset.py and re-verify against local_sha256."
        )
        entry["width"] = measured["width"]
        entry["height"] = measured["height"]
        entry["png_bit_depth"] = measured["png_bit_depth"]
        entry["png_color_type"] = measured["png_color_type"]
        entry["png_interlace"] = measured["png_interlace"]
        if measured["sha256"] != entry.get("local_sha256"):
            raise Env1AssetError(
                f"{file_name}: the file on disk no longer matches the fetch "
                f"receipt digest ({entry.get('local_sha256')}); re-run the fetch"
            )
        if (measured["width"], measured["height"]) != (SOURCE_EDGE, SOURCE_EDGE):
            raise Env1AssetError(
                f"{file_name}: expected a {SOURCE_EDGE}x{SOURCE_EDGE} source, "
                f"got {measured['width']}x{measured['height']}"
            )
        if measured["png_bit_depth"] not in (8, 16):
            raise Env1AssetError(
                f"{file_name}: the recipe requires an 8- or 16-bit source, got "
                f"bit depth {measured['png_bit_depth']}"
            )
        sources.append(entry)
    return sources


def build_runtime_outputs(root: pathlib.Path, descriptor: OpenAssetDescriptor) -> list[dict]:
    """Measure the committed runtime maps."""
    outputs: list[dict] = []
    directory = runtime_dir_for(root, descriptor.slug)
    for role, file_name, color_space, channels in descriptor.runtime_outputs:
        path = directory / file_name
        measured = describe_file(path)
        outputs.append(
            {
                "role": role,
                "file_name": file_name,
                "path": f"{descriptor.runtime_dir_relative}/{file_name}",
                "color_space": color_space,
                "channels": channels,
                "byte_size": measured["byte_size"],
                "sha256": measured["sha256"],
                "width": measured["width"],
                "height": measured["height"],
                "png_bit_depth": measured["png_bit_depth"],
                "png_color_type": measured["png_color_type"],
                "png_interlace": measured["png_interlace"],
                "embedded_by": "crates/renderer/src/env1_material.rs (include_bytes!)",
            }
        )
    return outputs


def build_asset_entry(
    root: pathlib.Path, cache: pathlib.Path, descriptor: OpenAssetDescriptor
) -> dict:
    info = read_json(cache / "api" / "info.json")
    receipt = read_json(cache / RECEIPT_NAME)
    if receipt.get("all_verified") is not True:
        raise Env1AssetError(
            f"the fetch receipt of {descriptor.slug} reports an unverified "
            "download; refusing to build a manifest from unverified sources"
        )
    physical = physical_dimensions_m(info.get("dimensions") or [])
    span = physical[0]
    return {
        "asset_id": descriptor.asset_id,
        "provider": PROVIDER,
        "slug": descriptor.slug,
        "name": info.get("name"),
        "source_page": descriptor.source_page,
        "license": LICENSE,
        "license_url": LICENSE_URL,
        "license_note": LICENSE_NOTE,
        "authors": info.get("authors"),
        "api": {
            "info_url": descriptor.info_url,
            "files_url": descriptor.files_url,
            "info_payload_sha256": receipt["api_payloads"]["info"]["sha256"],
            "files_payload_sha256": receipt["api_payloads"]["files"]["sha256"],
            "files_hash": info.get("files_hash"),
            "date_published_unix": info.get("date_published"),
            "max_resolution": info.get("max_resolution"),
            "dimensions": info.get("dimensions"),
            "dimensions_unit": DIMENSIONS_UNIT,
            "physical_dimensions_m": physical,
            "dimensions_unit_note": DIMENSIONS_UNIT_NOTE,
            "category": info.get("category"),
            "description": info.get("description"),
            "fetched_utc": receipt.get("fetched_utc"),
            "user_agent": receipt.get("user_agent"),
        },
        "source_resolution": SOURCE_RESOLUTION,
        "source_format": SOURCE_FORMAT,
        "source_files": build_source_files(cache, receipt, descriptor),
        "processing": {
            "recipe_version": RECIPE_VERSION,
            "processor": f"{PROCESSOR_COMMAND} --asset {descriptor.slug}",
            "processor_module": "crates/renderer/src/env1_material.rs",
            "source_edge": SOURCE_EDGE,
            "runtime_edge": RUNTIME_EDGE,
            "reduction": (
                "single exact 2x2 box filter, 4096 -> 2048; a source that "
                "is not exactly 4096x4096 is rejected rather than "
                "resampled with a different filter"
            ),
            "color_space_handling": {
                "base_color": (
                    "sRGB source texels decoded to linear, averaged, "
                    "re-encoded to sRGB, quantized once to 8 bits"
                ),
                "normal": (
                    "linear tangent-space vectors decoded to [-1, 1], "
                    "summed and renormalized, then re-encoded; OpenGL "
                    "(+Y) orientation preserved from the nor_gl source"
                ),
                "roughness": (
                    "linear 16-bit grayscale reduced with exact u32 "
                    "integer arithmetic and an exact 65535 -> 255 rescale"
                ),
            },
            "alpha": (
                "Runtime base color and normal maps are opaque RGBA8 with "
                "alpha = 255. Where a diffuse source carries an alpha lane "
                "(PNG color type 6) it is not transferred: ground materials "
                "are opaque by recipe. Alpha preservation through the mip "
                "chain is implemented and tested for future foliage sources."
            ),
            "not_applied": list(NOT_APPLIED),
            "determinism": (
                "pure function of the source bytes; re-running the "
                "processor produces byte-identical PNGs"
            ),
        },
        "runtime_outputs": build_runtime_outputs(root, descriptor),
        "runtime_binding": {
            "terrain_base_tile_scale_m": span,
            "constant": descriptor.binding_constant,
            "defined_in": descriptor.binding_defined_in,
            "relationship": (
                f"One tile of this material layer covers exactly the asset's "
                f"full physical span, so the photograph is reproduced at true "
                f"size: {descriptor.slug} is a {span} m x {span} m scan "
                f"({info.get('dimensions')} {DIMENSIONS_UNIT} per axis) and the "
                f"renderer compiles {descriptor.binding_constant} = {span} m "
                f"for this layer. The macro (48 m) and detail (0.40 m) "
                f"frequencies of the maintained base layer resolve to absolute "
                f"world metres in the shader and are invariant to it."
            ),
            "enforced_by": [
                "tools/env1_asset_pipeline/env1_assets.py::_validate_runtime_binding",
                "tools/env1_asset_pipeline/verify_env1_assets.py::verify_tile_scale_binding",
            ],
        },
    }


def build_manifest_document(
    root: pathlib.Path, slugs: list[str], preserve: dict | None
) -> dict:
    entries: list[dict] = []
    preserved = {
        entry.get("slug"): entry
        for entry in (preserve or {}).get("assets", [])
        if isinstance(entry, dict)
    }
    for slug in slugs:
        descriptor = descriptor_for_slug(slug)
        cache = source_cache_dir_for(root, slug)
        if not (cache / RECEIPT_NAME).is_file() and slug in preserved:
            entries.append(preserved[slug])
            continue
        entries.append(build_asset_entry(root, cache, descriptor))
    # Keep the registry order stable and retain assets this run did not rebuild.
    ordered: list[dict] = []
    seen: set[str] = set()
    for entry in entries:
        ordered.append(entry)
        seen.add(entry.get("slug"))
    for slug, entry in preserved.items():
        if slug not in seen and slug not in slugs:
            ordered.append(entry)

    return {
        "manifest_version": MANIFEST_VERSION,
        "slice": "ENV1-A",
        "generated_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "attribution": ATTRIBUTION,
        "api_terms_note": (
            "The Poly Haven public API was used for all metadata and download "
            "URLs; no HTML was scraped. The API terms require the attribution "
            "recorded in `attribution`."
        ),
        "assets": ordered,
    }


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="build_manifest.py",
        description="Build the ENV1 open-asset provenance manifest.",
    )
    parser.add_argument(
        "--root",
        metavar="PATH",
        default=None,
        help="workspace root (default: inferred from this file's location)",
    )
    parser.add_argument(
        "--out",
        metavar="PATH",
        default=None,
        help=f"manifest destination (default {manifest_path()})",
    )
    parser.add_argument(
        "--slugs",
        metavar="SLUG[,SLUG...]",
        default="sparse_grass",
        help=(
            "comma-separated registered assets to (re)build from their source "
            "cache; assets already in the manifest and not listed are preserved "
            "verbatim (default: sparse_grass)"
        ),
    )
    args = parser.parse_args(argv)

    root = pathlib.Path(args.root).resolve() if args.root else pathlib.Path(
        __file__
    ).resolve().parent.parent.parent
    out = pathlib.Path(args.out).resolve() if args.out else manifest_path(root)
    slugs = [slug.strip() for slug in args.slugs.split(",") if slug.strip()]

    preserve = None
    if out.is_file():
        preserve = load_manifest(out)

    try:
        document = build_manifest_document(root, slugs, preserve)
        errors = validate_manifest(document)
        if errors:
            print("error: the generated manifest violates its own contract:", file=sys.stderr)
            for error in errors:
                print(f"  - {error}", file=sys.stderr)
            return 1
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {out}")
        for entry in document["assets"]:
            print(f"  asset_id: {entry['asset_id']} ({entry['slug']}, {entry['license']})")
            for output in entry["runtime_outputs"]:
                print(
                    f"    {output['role']:11} {output['file_name']:34} "
                    f"{output['byte_size']:>9} bytes  {output['sha256'][:16]}..."
                )
        print(ATTRIBUTION)
        return 0
    except Env1AssetError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code


if __name__ == "__main__":
    sys.exit(main())
