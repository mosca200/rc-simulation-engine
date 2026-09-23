"""ENV1 open-asset provenance: shared library.

Owns the digest, PNG-header and manifest primitives used by the ENV1 asset
acquisition, manifest-building and verification tools.

Standard library only, matching the deliberate design invariant of
``tools/visual_benchmark``: reading a PNG header needs 33 bytes and
``int.from_bytes``, not an image library.

Nothing here invents metadata. Every field in a manifest is either read from
the Poly Haven API payload, measured from a file on disk, or explicitly set to
``None`` with a written reason.
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import re
from typing import Any, Optional

# ---------------------------------------------------------------------------
# Versions and identity
# ---------------------------------------------------------------------------

MANIFEST_VERSION = 1
RECIPE_VERSION = 1
ASSET_ID_SPARSE_GRASS = "ENV1-GND-01"
PROVIDER = "Poly Haven"
SLUG = "sparse_grass"
SOURCE_PAGE = f"https://polyhaven.com/{'a'}/{SLUG}"
LICENSE = "CC0"
LICENSE_URL = "https://polyhaven.com/license"
ATTRIBUTION = "Powered by Poly Haven (https://polyhaven.com)"

INFO_URL = f"https://api.polyhaven.com/info/{SLUG}"
FILES_URL = f"https://api.polyhaven.com/files/{SLUG}"

#: Identifying User-Agent required by the Poly Haven API terms.
USER_AGENT = (
    "rc-simulation-engine/ENV1-A (CC0 asset provenance; "
    "repo mosca200/rc-simulation-engine)"
)

#: Poly Haven map-type key -> (runtime role, source file stem).
SOURCE_MAPS: tuple[tuple[str, str, str], ...] = (
    ("Diffuse", "base_color", "sparse_grass_diff_4k.png"),
    ("nor_gl", "normal", "sparse_grass_nor_gl_4k.png"),
    ("Rough", "roughness", "sparse_grass_rough_4k.png"),
)

SOURCE_RESOLUTION = "4k"
SOURCE_FORMAT = "png"
SOURCE_EDGE = 4096
RUNTIME_EDGE = 2048

#: Runtime output role -> (file name, color space, channel layout).
RUNTIME_OUTPUTS: tuple[tuple[str, str, str, str], ...] = (
    ("base_color", "sparse_grass_base_color.png", "srgb", "rgba8"),
    ("normal", "sparse_grass_normal.png", "linear", "rgba8"),
    ("roughness", "sparse_grass_roughness.png", "linear", "r8"),
)

RUNTIME_DIR_RELATIVE = "crates/renderer/assets/env1/terrain/sparse_grass"
SOURCE_CACHE_RELATIVE = "tmp/env1_source_cache/polyhaven/sparse_grass"
MANIFEST_RELATIVE = "docs/assets/env1/env1_open_assets.json"

#: Expected PNG IHDR color types per role (see the recipe documentation).
EXPECTED_COLOR_TYPE = {"base_color": 6, "normal": 6, "roughness": 0}
EXPECTED_BIT_DEPTH = {"source": 16, "runtime": 8}

#: The Poly Haven public API's OpenAPI schema defines a texture asset's
#: `dimensions` as "an array with the dimensions of this asset on each axis in
#: millimeters". The unit is therefore documented by the provider, not inferred
#: and not absent: `sparse_grass` reports [2000, 2000], i.e. a 2.0 m x 2.0 m
#: scanned area.
DIMENSIONS_UNIT = "mm"
MILLIMETRES_PER_METRE = 1000.0

#: Things the recipe deliberately never does. Recorded in the manifest so a
#: future reviewer can see the absence was a decision, not an oversight.
NOT_APPLIED = (
    "saturation_or_contrast_boost",
    "baked_shadow",
    "ambient_occlusion_multiplied_into_base_color",
    "cosmetic_sharpening",
    "colour_lut",
    "displacement",
)

_CHUNK = 1 << 22
_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


# ---------------------------------------------------------------------------
# Open-asset registry
# ---------------------------------------------------------------------------
#
# ENV1-A shipped a single asset (``sparse_grass``) through module-level
# constants. Flying Field v1 adds two further CC0 ground materials, so every
# per-asset fact now lives in one descriptor table; the module-level constants
# above remain the ``sparse_grass`` descriptor's values and are kept as the
# backwards-compatible aliases every existing tool and test imports.


class OpenAssetDescriptor:
    """The per-asset facts the acquisition/manifest/verification chain needs."""

    __slots__ = (
        "slug",
        "asset_id",
        "source_maps",
        "runtime_outputs",
        "runtime_dir_relative",
        "source_cache_relative",
        "binding_constant",
        "binding_defined_in",
    )

    def __init__(
        self,
        slug: str,
        asset_id: str,
        source_maps: tuple,
        runtime_outputs: tuple,
        runtime_dir_relative: str,
        source_cache_relative: str,
        binding_constant: str,
        binding_defined_in: str,
    ) -> None:
        self.slug = slug
        self.asset_id = asset_id
        self.source_maps = source_maps
        self.runtime_outputs = runtime_outputs
        self.runtime_dir_relative = runtime_dir_relative
        self.source_cache_relative = source_cache_relative
        self.binding_constant = binding_constant
        self.binding_defined_in = binding_defined_in

    @property
    def info_url(self) -> str:
        return f"https://api.polyhaven.com/info/{self.slug}"

    @property
    def files_url(self) -> str:
        return f"https://api.polyhaven.com/files/{self.slug}"

    @property
    def source_page(self) -> str:
        return f"https://polyhaven.com/a/{self.slug}"


OPEN_ASSETS: tuple[OpenAssetDescriptor, ...] = (
    OpenAssetDescriptor(
        slug=SLUG,
        asset_id=ASSET_ID_SPARSE_GRASS,
        source_maps=SOURCE_MAPS,
        runtime_outputs=RUNTIME_OUTPUTS,
        runtime_dir_relative=RUNTIME_DIR_RELATIVE,
        source_cache_relative=SOURCE_CACHE_RELATIVE,
        binding_constant="DEFAULT_TERRAIN_TEXTURE_SCALE_M",
        binding_defined_in="crates/renderer/src/terrain.rs",
    ),
    OpenAssetDescriptor(
        slug="grass_path_3",
        asset_id="ENV1-GND-02",
        source_maps=(
            ("Diffuse", "base_color", "grass_path_3_diff_4k.png"),
            ("nor_gl", "normal", "grass_path_3_nor_gl_4k.png"),
            ("Rough", "roughness", "grass_path_3_rough_4k.png"),
        ),
        runtime_outputs=(
            ("base_color", "grass_path_3_base_color.png", "srgb", "rgba8"),
            ("normal", "grass_path_3_normal.png", "linear", "rgba8"),
            ("roughness", "grass_path_3_roughness.png", "linear", "r8"),
        ),
        runtime_dir_relative="crates/renderer/assets/env1/terrain/grass_path_3",
        source_cache_relative="tmp/env1_source_cache/polyhaven/grass_path_3",
        binding_constant="FFV1_TERRAIN_WORN_TILE_SCALE_M",
        binding_defined_in="crates/renderer/src/terrain.rs",
    ),
    OpenAssetDescriptor(
        slug="forest_ground_04",
        asset_id="ENV1-GND-03",
        source_maps=(
            ("Diffuse", "base_color", "forest_ground_04_diff_4k.png"),
            ("nor_gl", "normal", "forest_ground_04_nor_gl_4k.png"),
            ("Rough", "roughness", "forest_ground_04_rough_4k.png"),
        ),
        runtime_outputs=(
            ("base_color", "forest_ground_04_base_color.png", "srgb", "rgba8"),
            ("normal", "forest_ground_04_normal.png", "linear", "rgba8"),
            ("roughness", "forest_ground_04_roughness.png", "linear", "r8"),
        ),
        runtime_dir_relative="crates/renderer/assets/env1/terrain/forest_ground_04",
        source_cache_relative="tmp/env1_source_cache/polyhaven/forest_ground_04",
        binding_constant="FFV1_TERRAIN_DRY_TILE_SCALE_M",
        binding_defined_in="crates/renderer/src/terrain.rs",
    ),
)


def descriptor_for_slug(slug: str) -> OpenAssetDescriptor:
    """The descriptor of one registered open asset, failing closed."""
    for descriptor in OPEN_ASSETS:
        if descriptor.slug == slug:
            return descriptor
    raise Env1AssetError(
        f"unknown open asset {slug!r}; registered slugs are "
        f"{[d.slug for d in OPEN_ASSETS]}",
        exit_code=2,
    )


def source_cache_dir_for(root: Optional[pathlib.Path] = None, slug: str = SLUG) -> pathlib.Path:
    """Gitignored Poly Haven source cache of one registered asset."""
    return (root or repo_root()) / descriptor_for_slug(slug).source_cache_relative


def runtime_dir_for(root: Optional[pathlib.Path] = None, slug: str = SLUG) -> pathlib.Path:
    """Committed runtime asset directory of one registered asset."""
    return (root or repo_root()) / descriptor_for_slug(slug).runtime_dir_relative


class Env1AssetError(Exception):
    """Raised for every fail-closed condition in the ENV1 asset tooling."""

    def __init__(self, message: str, exit_code: int = 1) -> None:
        super().__init__(message)
        self.message = message
        self.exit_code = exit_code


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------


def repo_root() -> pathlib.Path:
    """Workspace root: the parent of ``tools/``."""
    return pathlib.Path(__file__).resolve().parent.parent.parent


def source_cache_dir(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Gitignored Poly Haven source cache for ``sparse_grass``."""
    return (root or repo_root()) / SOURCE_CACHE_RELATIVE


def runtime_dir(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed runtime asset directory."""
    return (root or repo_root()) / RUNTIME_DIR_RELATIVE


def manifest_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Path of the committed provenance manifest."""
    return (root or repo_root()) / MANIFEST_RELATIVE


# ---------------------------------------------------------------------------
# Digests and PNG headers
# ---------------------------------------------------------------------------


def sha256_file(path: pathlib.Path) -> str:
    """SHA-256 of a file's bytes, streamed."""
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(_CHUNK), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(payload: bytes) -> str:
    """SHA-256 of an in-memory payload."""
    return hashlib.sha256(payload).hexdigest()


def md5_file(path: pathlib.Path) -> str:
    """MD5 of a file's bytes.

    Poly Haven publishes MD5 (not SHA-256) per download leaf, so this is the
    only way to verify a download against the API. It is used for source
    verification only; every digest recorded for provenance is SHA-256.
    """
    digest = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(_CHUNK), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_png_header(path: pathlib.Path) -> dict[str, int]:
    """Parse the IHDR of a PNG without an image library.

    Returns ``width``, ``height``, ``bit_depth``, ``color_type`` and
    ``interlace``. Fails closed on a bad signature or a truncated header.
    """
    with path.open("rb") as handle:
        head = handle.read(33)
    if len(head) < 33:
        raise Env1AssetError(f"{path}: file is shorter than a PNG header")
    if head[:8] != _PNG_SIGNATURE:
        raise Env1AssetError(f"{path}: not a PNG (bad signature)")
    width = int.from_bytes(head[16:20], "big")
    height = int.from_bytes(head[20:24], "big")
    return {
        "width": width,
        "height": height,
        "bit_depth": head[24],
        "color_type": head[25],
        "interlace": head[28],
    }


def describe_file(path: pathlib.Path) -> dict[str, Any]:
    """Measured facts about a PNG on disk: size, digest and IHDR fields."""
    if not path.is_file():
        raise Env1AssetError(f"missing expected file: {path}")
    header = read_png_header(path)
    return {
        "byte_size": path.stat().st_size,
        "sha256": sha256_file(path),
        "width": header["width"],
        "height": header["height"],
        "png_bit_depth": header["bit_depth"],
        "png_color_type": header["color_type"],
        "png_interlace": header["interlace"],
    }


# ---------------------------------------------------------------------------
# Manifest handling
# ---------------------------------------------------------------------------

#: Fields every asset entry must carry. Missing (as opposed to null) is an
#: error: an unknown field is a schema drift, a null is an honest "unavailable".
REQUIRED_ASSET_FIELDS = (
    "asset_id",
    "provider",
    "slug",
    "name",
    "source_page",
    "license",
    "license_url",
    "authors",
    "api",
    "source_resolution",
    "source_format",
    "source_files",
    "processing",
    "runtime_outputs",
    "runtime_binding",
)

REQUIRED_SOURCE_FILE_FIELDS = (
    "map_type",
    "role",
    "url",
    "file_name",
    "api_size",
    "api_md5",
    "local_path",
    "local_size",
    "local_md5",
    "local_sha256",
    "size_verified",
    "md5_verified",
)

REQUIRED_RUNTIME_FIELDS = (
    "role",
    "path",
    "file_name",
    "color_space",
    "channels",
    "byte_size",
    "sha256",
    "width",
    "height",
    "png_bit_depth",
    "png_color_type",
)

REQUIRED_PROCESSING_FIELDS = (
    "recipe_version",
    "processor",
    "source_edge",
    "runtime_edge",
    "reduction",
    "color_space_handling",
    "not_applied",
)


def physical_dimensions_m(dimensions: list, unit: str = DIMENSIONS_UNIT) -> list[float]:
    """Convert the API's millimetre dimensions to metres, failing closed.

    The conversion is explicit and lives in exactly one place, so a manifest can
    never carry a derived value that disagrees with the raw one.
    """
    if unit != DIMENSIONS_UNIT:
        raise Env1AssetError(
            f"the Poly Haven OpenAPI schema defines `dimensions` in {DIMENSIONS_UNIT}; "
            f"refusing to convert from {unit!r}"
        )
    return [float(axis) / MILLIMETRES_PER_METRE for axis in dimensions]


def load_manifest(path: pathlib.Path) -> dict[str, Any]:
    """Load the provenance manifest, failing closed on malformed JSON."""
    if not path.is_file():
        raise Env1AssetError(f"manifest not found: {path}", exit_code=2)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise Env1AssetError(f"{path}: invalid JSON: {error}") from error


def validate_manifest(manifest: dict[str, Any]) -> list[str]:
    """Return a list of human-readable contract violations (empty == valid)."""
    errors: list[str] = []

    version = manifest.get("manifest_version")
    if version != MANIFEST_VERSION:
        errors.append(f"manifest_version must be {MANIFEST_VERSION}, got {version!r}")

    assets = manifest.get("assets")
    if not isinstance(assets, list) or not assets:
        errors.append("assets must be a non-empty list")
        return errors

    seen_ids: set[str] = set()
    for index, asset in enumerate(assets):
        where = f"assets[{index}]"
        if not isinstance(asset, dict):
            errors.append(f"{where} must be an object")
            continue
        for field in REQUIRED_ASSET_FIELDS:
            if field not in asset:
                errors.append(f"{where}.{field}: required field missing")
        asset_id = asset.get("asset_id")
        if isinstance(asset_id, str):
            if asset_id in seen_ids:
                errors.append(f"{where}.asset_id: duplicate {asset_id!r}")
            seen_ids.add(asset_id)
            if not re.fullmatch(r"ENV1-[A-Z]{3}-[0-9]{2}", asset_id):
                errors.append(
                    f"{where}.asset_id: must match ENV1-<AAA>-<NN>, got {asset_id!r}"
                )
        if asset.get("license") is not None and asset.get("license") != LICENSE:
            errors.append(
                f"{where}.license: only {LICENSE} assets are accepted by ENV1-A, "
                f"got {asset.get('license')!r}"
            )
        errors.extend(_validate_api_block(asset.get("api"), where))
        errors.extend(_validate_source_files(asset.get("source_files"), where))
        errors.extend(_validate_processing(asset.get("processing"), where))
        errors.extend(_validate_runtime_outputs(asset.get("runtime_outputs"), where))
        errors.extend(
            _validate_runtime_binding(asset.get("runtime_binding"), asset.get("api"), where)
        )
    return errors


def _validate_api_block(api: Any, where: str) -> list[str]:
    errors: list[str] = []
    if not isinstance(api, dict):
        errors.append(f"{where}.api must be an object")
        return errors
    for field in (
        "info_url",
        "files_url",
        "info_payload_sha256",
        "files_payload_sha256",
        "files_hash",
    ):
        if field not in api:
            errors.append(f"{where}.api.{field}: required field missing")
    errors.extend(_validate_dimensions(api, where))
    return errors


def _validate_dimensions(api: dict, where: str) -> list[str]:
    """Enforce the raw value, the API-documented unit and the derived metres.

    All three must be present and mutually consistent, so neither a guessed
    unit nor a hidden conversion can slip through.
    """
    errors: list[str] = []
    for field in ("dimensions", "dimensions_unit", "physical_dimensions_m"):
        if field not in api:
            errors.append(f"{where}.api.{field}: required field missing")
    if not api.get("dimensions_unit_note"):
        errors.append(
            f"{where}.api.dimensions_unit_note: must cite the provider schema "
            "that defines the unit"
        )
    if errors:
        return errors

    unit = api["dimensions_unit"]
    if unit != DIMENSIONS_UNIT:
        errors.append(
            f"{where}.api.dimensions_unit: the Poly Haven OpenAPI schema defines "
            f"`dimensions` in millimetres, so it must be {DIMENSIONS_UNIT!r}, got {unit!r}"
        )
    dimensions = api["dimensions"]
    physical = api["physical_dimensions_m"]
    if not (isinstance(dimensions, list) and len(dimensions) == 2):
        errors.append(f"{where}.api.dimensions must be a two-element array")
        return errors
    if not (isinstance(physical, list) and len(physical) == 2):
        errors.append(f"{where}.api.physical_dimensions_m must be a two-element array")
        return errors
    for axis in range(2):
        raw, derived = dimensions[axis], physical[axis]
        if not isinstance(raw, (int, float)) or isinstance(raw, bool):
            errors.append(f"{where}.api.dimensions[{axis}] must be a number")
            continue
        if not isinstance(derived, (int, float)) or isinstance(derived, bool):
            errors.append(f"{where}.api.physical_dimensions_m[{axis}] must be a number")
            continue
        expected = raw / MILLIMETRES_PER_METRE
        if abs(derived - expected) > 1e-9:
            errors.append(
                f"{where}.api.physical_dimensions_m[{axis}]: {raw} mm is {expected} m, "
                f"but the manifest records {derived} m"
            )
    return errors


def _validate_runtime_binding(binding: Any, api: Any, where: str) -> list[str]:
    """Tie the terrain base tile scale to the asset's physical span."""
    errors: list[str] = []
    if not isinstance(binding, dict):
        errors.append(f"{where}.runtime_binding must be an object")
        return errors
    for field in (
        "terrain_base_tile_scale_m",
        "constant",
        "defined_in",
        "relationship",
    ):
        if field not in binding:
            errors.append(f"{where}.runtime_binding.{field}: required field missing")
    scale = binding.get("terrain_base_tile_scale_m")
    if not isinstance(scale, (int, float)) or isinstance(scale, bool):
        errors.append(f"{where}.runtime_binding.terrain_base_tile_scale_m must be a number")
        return errors
    physical = (api or {}).get("physical_dimensions_m")
    if not (isinstance(physical, list) and len(physical) == 2):
        errors.append(
            f"{where}.runtime_binding: cannot check the tile scale without "
            "api.physical_dimensions_m"
        )
        return errors
    for axis in range(2):
        if abs(float(scale) - float(physical[axis])) > 1e-9:
            errors.append(
                f"{where}.runtime_binding.terrain_base_tile_scale_m is {scale} m but the "
                f"asset's physical span on axis {axis} is {physical[axis]} m: one texture "
                "tile must cover exactly the scanned area"
            )
    return errors


def _validate_source_files(files: Any, where: str) -> list[str]:
    errors: list[str] = []
    if not isinstance(files, list) or not files:
        errors.append(f"{where}.source_files must be a non-empty list")
        return errors
    roles = set()
    for index, entry in enumerate(files):
        item = f"{where}.source_files[{index}]"
        if not isinstance(entry, dict):
            errors.append(f"{item} must be an object")
            continue
        for field in REQUIRED_SOURCE_FILE_FIELDS:
            if field not in entry:
                errors.append(f"{item}.{field}: required field missing")
        role = entry.get("role")
        if role in roles:
            errors.append(f"{item}.role: duplicate {role!r}")
        roles.add(role)
        for flag in ("size_verified", "md5_verified"):
            if entry.get(flag) is not True:
                errors.append(
                    f"{item}.{flag}: must be true; an unverified source download "
                    "is a fail-closed condition"
                )
        if entry.get("local_sha256") is None:
            errors.append(f"{item}.local_sha256: the local digest is mandatory")
    expected_roles = {role for _, role, _ in SOURCE_MAPS}
    if roles != expected_roles:
        errors.append(
            f"{where}.source_files: roles must be exactly {sorted(expected_roles)}, "
            f"got {sorted(str(r) for r in roles)}"
        )
    return errors


def _validate_processing(processing: Any, where: str) -> list[str]:
    errors: list[str] = []
    if not isinstance(processing, dict):
        errors.append(f"{where}.processing must be an object")
        return errors
    for field in REQUIRED_PROCESSING_FIELDS:
        if field not in processing:
            errors.append(f"{where}.processing.{field}: required field missing")
    if processing.get("recipe_version") != RECIPE_VERSION:
        errors.append(
            f"{where}.processing.recipe_version must be {RECIPE_VERSION}, "
            f"got {processing.get('recipe_version')!r}"
        )
    if processing.get("source_edge") != SOURCE_EDGE:
        errors.append(f"{where}.processing.source_edge must be {SOURCE_EDGE}")
    if processing.get("runtime_edge") != RUNTIME_EDGE:
        errors.append(f"{where}.processing.runtime_edge must be {RUNTIME_EDGE}")
    not_applied = processing.get("not_applied")
    if not isinstance(not_applied, list) or tuple(not_applied) != NOT_APPLIED:
        errors.append(
            f"{where}.processing.not_applied must be exactly {list(NOT_APPLIED)}"
        )
    return errors


def _validate_runtime_outputs(outputs: Any, where: str) -> list[str]:
    errors: list[str] = []
    if not isinstance(outputs, list) or not outputs:
        errors.append(f"{where}.runtime_outputs must be a non-empty list")
        return errors
    roles = set()
    for index, entry in enumerate(outputs):
        item = f"{where}.runtime_outputs[{index}]"
        if not isinstance(entry, dict):
            errors.append(f"{item} must be an object")
            continue
        for field in REQUIRED_RUNTIME_FIELDS:
            if field not in entry:
                errors.append(f"{item}.{field}: required field missing")
        role = entry.get("role")
        if role in roles:
            errors.append(f"{item}.role: duplicate {role!r}")
        roles.add(role)
        if role in EXPECTED_COLOR_TYPE and entry.get("png_color_type") is not None:
            if entry["png_color_type"] != EXPECTED_COLOR_TYPE[role]:
                errors.append(
                    f"{item}.png_color_type: {role} must be PNG color type "
                    f"{EXPECTED_COLOR_TYPE[role]}, got {entry['png_color_type']}"
                )
        if entry.get("png_bit_depth") not in (None, EXPECTED_BIT_DEPTH["runtime"]):
            errors.append(
                f"{item}.png_bit_depth: runtime maps must be 8-bit, "
                f"got {entry.get('png_bit_depth')}"
            )
        for field in ("width", "height"):
            if entry.get(field) != RUNTIME_EDGE:
                errors.append(f"{item}.{field} must be {RUNTIME_EDGE}")
        digest = entry.get("sha256")
        if not (isinstance(digest, str) and re.fullmatch(r"[0-9a-f]{64}", digest)):
            errors.append(f"{item}.sha256 must be 64 lowercase hex characters")
    expected_roles = {role for role, _, _, _ in RUNTIME_OUTPUTS}
    if roles != expected_roles:
        errors.append(
            f"{where}.runtime_outputs: roles must be exactly {sorted(expected_roles)}, "
            f"got {sorted(str(r) for r in roles)}"
        )
    return errors


def configure_streams() -> None:
    """Make redirected stdout/stderr UTF-8 and loss-tolerant.

    Windows redirects default to cp1252, which raises ``UnicodeEncodeError`` on
    non-ASCII markers and fails every test in a suite.
    """
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
