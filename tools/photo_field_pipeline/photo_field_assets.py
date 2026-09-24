"""PF1 photo-field asset pipeline: shared library.

Owns every fact and every primitive the PF1 acquisition, processing, proxy
authoring, manifest building and verification tools share:

* the Poly Haven provenance constants and the pinned source digests;
* streamed SHA-256 / MD5 digests and a dependency-free JPEG header reader;
* the Radiance ``32-bit_rle_rgbe`` decoder (new-style RLE only, band-wise so an
  8192x4096 source never materialises more than one band of float64);
* the Khronos PBR Neutral tone mapper, ported statement-by-statement from
  ``crates/renderer/src/shader.wgsl``;
* the sRGB transfer functions of ``crates/renderer/src/texture.rs``;
* the equirectangular direction -> UV mapping of
  ``crates/renderer/src/photo_field.rs``;
* a minimal deterministic glTF 2.0 binary (GLB) writer/reader.

Nothing here invents metadata. Every recorded value is either read from the
Poly Haven API payload, measured from a file on disk, derived in code from a
documented measurement, or explicitly flagged as a manual calibration.

Standard library + numpy (Pillow is only needed by the panorama processor, and
scipy is not used at all). Fail closed: every unexpected condition raises
:class:`PhotoFieldError` with an exit code, never a warning.
"""

from __future__ import annotations

import hashlib
import json
import math
import pathlib
import struct
from typing import Any, Iterator, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Versions and identity
# ---------------------------------------------------------------------------

#: The only runtime manifest schema the renderer understands
#: (``crates/renderer/src/photo_field.rs``: PHOTO_FIELD_MANIFEST_SCHEMA_VERSION).
MANIFEST_SCHEMA_VERSION = 1
PROVENANCE_SCHEMA_VERSION = 1

ASSET_ID = "pf1-meadow"
PROVIDER = "Poly Haven"
SLUG = "meadow"
ASSET_NAME = "Meadow"
SOURCE_PAGE = f"https://polyhaven.com/a/{SLUG}"
LICENSE = "CC0"
LICENSE_URL = "https://polyhaven.com/license"
ATTRIBUTION = "Powered by Poly Haven (https://polyhaven.com)"
AUTHORS = {"Sergej Majboroda": "All"}

INFO_URL = f"https://api.polyhaven.com/info/{SLUG}"
FILES_URL = f"https://api.polyhaven.com/files/{SLUG}"

#: Identifying User-Agent required by the Poly Haven API terms.
USER_AGENT = (
    "rc-simulation-engine/PF1 (CC0 asset provenance; "
    "repo mosca200/rc-simulation-engine)"
)

#: The provider's largest published resolution for this asset, and the
#: resolution PF1 actually acquires (8k == 8192x4096 equirectangular).
SOURCE_DIMENSIONS = [16384, 8192]
ACQUIRED_RESOLUTION = "8k"
ACQUIRED_DIMENSIONS = [8192, 4096]
PROBE_RESOLUTION = "1k"
PROBE_DIMENSIONS = [1024, 512]

RUNTIME_DIR_RELATIVE = "crates/renderer/assets/photofield/meadow"
SOURCE_CACHE_RELATIVE = "tmp/pf1_source_cache/polyhaven/meadow"
RUNTIME_MANIFEST_RELATIVE = f"{RUNTIME_DIR_RELATIVE}/photo_field_manifest.json"
PROVENANCE_MANIFEST_RELATIVE = "docs/assets/photofield/pf1_provenance.json"

PANORAMA_FILE_NAME = "meadow_panorama_8192x4096.jpg"
DEPTH_PROXY_FILE_NAME = "photo_field_depth.glb"
RECEIPT_NAME = "fetch_receipt_pf1.json"

#: Display-referred derivative encoding. The runtime can only decode PNG/JPEG
#: (the `image` crate is built with `png` + `jpeg`), and a panorama is a
#: photograph: JPEG at high quality with NO chroma subsampling keeps the sky
#: gradient and the horizon band clean while staying ~30 MB instead of ~200 MB
#: of PNG. `subsampling=0` (4:4:4) is what makes the derivative subsample-free.
JPEG_QUALITY = 92
JPEG_SUBSAMPLING = 0
JPEG_FORMAT = "jpeg"

#: Things the recipe deliberately never does. Recorded in the provenance so a
#: reviewer can see the absence was a decision, not an oversight.
NOT_APPLIED = (
    "ai_depth_estimation",
    "photogrammetry",
    "virtual_texturing",
    "texture_streaming",
    "resampling",
    "saturation_or_contrast_boost",
    "colour_lut",
)

_CHUNK = 1 << 22


class SourceFile:
    """One Poly Haven source file: what to fetch and what it must match.

    ``api_path`` is the tuple of keys that reaches the leaf in the ``/files``
    payload; ``url``/``api_size``/``api_md5`` are the values the API published
    when PF1 was calibrated (2026-09-24) and are pinned so an offline run can
    still fail closed on a corrupted cache. The live payload is authoritative
    when it is available: ``fetch_photo_field_sources.py`` requires the two to
    agree.
    """

    __slots__ = (
        "role",
        "file_name",
        "api_path",
        "url",
        "api_size",
        "api_md5",
        "pinned_sha256",
        "purpose",
    )

    def __init__(
        self,
        role: str,
        file_name: str,
        api_path: tuple,
        url: str,
        api_size: int,
        api_md5: str,
        pinned_sha256: Optional[str],
        purpose: str,
    ) -> None:
        self.role = role
        self.file_name = file_name
        self.api_path = api_path
        self.url = url
        self.api_size = api_size
        self.api_md5 = api_md5
        self.pinned_sha256 = pinned_sha256
        self.purpose = purpose


SOURCE_FILES: tuple[SourceFile, ...] = (
    SourceFile(
        role="source_hdr",
        file_name="meadow_8k.hdr",
        api_path=("hdri", "8k", "hdr"),
        url="https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/8k/meadow_8k.hdr",
        api_size=108_457_266,
        api_md5="c1e25ad9fb1aba9ebc8babb952292727",
        pinned_sha256=(
            "9d947c59de8464a04fd22ecef8ed548750e2bbb07072f44ef61b0fbb2d738c9d"
        ),
        purpose=(
            "scene-referred Radiance RGBE source: the panorama derivative and "
            "every calibration measurement are derived from this file"
        ),
    ),
    SourceFile(
        role="look_reference_tonemapped_jpg",
        file_name="meadow_tonemapped.jpg",
        api_path=("tonemapped",),
        url=(
            "https://dl.polyhaven.org/file/ph-assets/HDRIs/extra/"
            "Tonemapped%20JPG/meadow.jpg"
        ),
        api_size=57_575_185,
        api_md5="9508d4679483dfc17466ff4c9ada44d9",
        pinned_sha256=(
            "8b9765e45ffc2106f4181a491244fd7113078cd26dcc615cacc11fb347724ee7"
        ),
        purpose=(
            "the provider's own photographic rendition: look reference and "
            "exposure-fit evidence only, never a runtime input"
        ),
    ),
    SourceFile(
        role="decode_probe_hdr",
        file_name="meadow_1k.hdr",
        api_path=("hdri", "1k", "hdr"),
        url="https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/1k/meadow_1k.hdr",
        api_size=1_819_485,
        api_md5="955f6b479ce79e67c0e224d3cc409ec6",
        # No pinned SHA-256: the 1k probe is a test fixture, not a processing
        # input. Its digest is measured and recorded by the fetch receipt.
        pinned_sha256=None,
        purpose=(
            "small Radiance source used by the test suite to exercise the RGBE "
            "decoder without touching the 108 MB 8k file"
        ),
    ),
)


def source_file_for_role(role: str) -> SourceFile:
    """The descriptor of one registered source file, failing closed."""
    for entry in SOURCE_FILES:
        if entry.role == role:
            return entry
    raise PhotoFieldError(
        f"unknown PF1 source role {role!r}; registered roles are "
        f"{[entry.role for entry in SOURCE_FILES]}",
        exit_code=2,
    )


# ---------------------------------------------------------------------------
# Calibration: the photographic eye, the sun and the exposure
# ---------------------------------------------------------------------------

#: The fixed photographic eye, in render metres (y-up, ground plane at y = 0).
#:
#: MANUALLY CALIBRATED. The aircraft always spawns at the render-world origin,
#: so the surveyed panorama eye is placed 20 m from the spawn along azimuth
#: 140 deg: ``-20 * (cos(140 deg), 0, sin(140 deg))``. Azimuth 140 deg keeps the
#: aircraft line-of-sight 12 deg clear of the photographed near tree trunk at
#: 152 deg (whose proxy would otherwise hide the parked aircraft) while staying
#: on the brick garage (measured span 131.75-169.94 deg). The aircraft then
#: sits just IN FRONT of the 25.1 m garage proxy and INSIDE the 30 m tree-ring
#: proxy, which is what makes the mandatory occlusion test (aircraft nearer vs
#: farther than a photographed obstacle) expressible with the real aircraft.
PILOT_EYE_RENDER_M = (15.321, 1.6, -12.856)
EYE_TO_SPAWN_DISTANCE_M = 20.0
#: Panorama azimuth at which the eye was placed, measured from the aircraft
#: spawn. Kept as a constant next to the eye so every derived reference (the
#: provenance record, the tests) reads the placement, never a remembered number.
PILOT_EYE_AZIMUTH_DEG = 140.0
CAMERA_HEIGHT_M = 1.6

#: Solar disc position measured in ``meadow_8k.hdr`` by connected-component
#: isolation of the brightest region (see the provenance calibration block for
#: the derivation and for the 0.9*max cross-check at 152.722 / 68.903 deg).
SUN_LONGITUDE_DEG = 153.027
SUN_ELEVATION_DEG = 68.936

#: ``sun_direction_render`` as pinned by the PF1 brief, and the tolerance the
#: f64 recomputation from the two angles above must satisfy.
#:
#: The render convention is ``azimuth = atan2(dir.z, dir.x)``, so the direction
#: towards a panorama longitude/elevation is
#: ``[cos(el) * cos(lon), sin(el), cos(el) * sin(lon)]``. The pinned literal is
#: a five-decimal evaluation of that formula; the exact f64 evaluation is
#: ``[-0.320314, 0.933180, 0.163018]``, i.e. the two agree to 3e-5 (about
#: 0.001 deg of azimuth). Both map to the solar texel well inside the 1e-5
#: tolerance ``photo_field.rs`` uses.
SUN_DIRECTION_RENDER = (-0.32032, 0.93318, 0.16299)
SUN_DIRECTION_TOLERANCE = 1e-4
SUN_DIRECTION_EXACT_F64 = (-0.320314, 0.933180, 0.163018)

#: Direct-light level and chromaticity, MANUALLY CALIBRATED against the
#: photographed scene (the disc radiance measured below is the evidence).
SUN_INTENSITY = 2.6
SUN_RGB = (1.0, 0.95, 0.85)
SHADOW_STRENGTH = 0.45

#: The panorama is used exactly as photographed: no rotation, no tilt.
PANORAMA_YAW_DEG = 0.0
PANORAMA_PITCH_DEG = 0.0

#: Measured radiance statistics of ``meadow_8k.hdr`` (luminance = Rec.709).
LUM_MAX = 39.44605255126953
SKY_UPPER_HEMISPHERE_MEAN = 1.250892996788025
ZENITH_TOP_1_64_MEAN = 3.151242971420288
GROUND_LOWER_45_PERCENT_MEAN = 0.16443490982055664
NEAR_SUN_SKY_MEAN = 2.1031928062438965
MEAN_RGB = (0.7370936520201212, 0.718630185213442, 0.699419742712962)
SUN_DISC_MEAN_RGB = (50.86206817626953, 34.16379165649414, 27.10344886779785)
SUN_DISC_PEAK_RGB = (54.0, 36.25, 28.5)
HORIZON_ROW_8K = 1910

#: Documented cross-check band for the 1k probe: FFV1's independent offline
#: look-development analysis of this exact asset recorded "sky mean 1.26".
SKY_MEAN_CROSS_CHECK_BAND = (1.15, 1.35)

#: Exposure decision. The renderer tonemaps the 3D aircraft with
#: ``khronos_pbr_neutral(hdr * exp2(exposure_ev))`` at exposure_ev = 0.0, and
#: the photographic background bypasses the tonemapper, so the derivative is
#: produced offline with the SAME curve at the SAME exposure: k = 1.0.
EXPOSURE_SCALE = 1.0
EXPOSURE_EV = 0.0

#: Residual evidence from the prototype exposure fit against the provider's
#: tonemapped JPG (``tmp/pf1_exposure_fit.log``). Recorded as historical
#: evidence only - see ``EXPOSURE_FIT_CAVEAT``.
EXPOSURE_FIT_MAE_AT_PINNED_SCALE = 0.07849
EXPOSURE_FIT_RMSE_AT_PINNED_SCALE = 0.09621
EXPOSURE_FIT_REFINED_SCALE = 1.1790625
EXPOSURE_FIT_REFINED_EV = 0.23764019503538478
EXPOSURE_FIT_REFINED_MAE = 0.07181839109904041
EXPOSURE_FIT_CORRELATION_SUBSAMPLED = 0.9720202616649115
EXPOSURE_FIT_CAVEAT = (
    "The prototype fit in tmp/pf1_exposure_fit.py evaluated WGSL "
    "`select(0.04, x - 6.25 * x * x, x < 0.08)` with its two branches swapped "
    "(it used the constant 0.04 for x < 0.08 and x - 6.25*x*x otherwise), so "
    "these residuals characterise the prototype curve, not the shipped one. "
    "photo_field_assets.khronos_pbr_neutral is a verbatim port of "
    "crates/renderer/src/shader.wgsl. The k = 1.0 decision does not depend on "
    "the fit: it is pinned to the renderer's exposure_ev = 0.0 so that the "
    "photographic background and the tonemapped aircraft share one curve."
)

#: Brick garage: the largest colour-segmented component of the photographed
#: building, as an angular box in panorama space (degrees).
GARAGE_ANGULAR_BOX_AZIMUTH_DEG = (131.748046875, 169.9365234375)
GARAGE_ANGULAR_BOX_ELEVATION_DEG = (-3.6474609375, 7.5146484375)
GARAGE_CENTROID_AZIMUTH_DEG = 150.87425614540803
GARAGE_CENTROID_ELEVATION_DEG = 1.8927557457427326
GARAGE_SEGMENT_AREA_PX = 118_048

#: Depth of the garage proxy along the viewing direction. Not measurable from a
#: single panorama: MANUALLY CALIBRATED from the crop previews.
GARAGE_DEPTH_M = 6.0

#: The brief's rounded derivations, used as regression bounds on the code that
#: re-derives them from the angular box and the assumed camera height.
GARAGE_DISTANCE_M_EXPECTED = 25.1
GARAGE_HEIGHT_M_EXPECTED = 4.91
GARAGE_WIDTH_M_EXPECTED = 16.7
GARAGE_DERIVATION_TOLERANCE_M = 0.05

# ---------------------------------------------------------------------------
# Calibration: coarse depth-proxy geometry contract
# ---------------------------------------------------------------------------

GROUND_RADIUS_M = 250.0
GROUND_SEGMENTS = 64

MAX_PROXY_TRIANGLES = 4000
EXPECTED_PROXY_TRIANGLES = 724

NODE_GROUND = "pf1_ground"
NODE_GARAGE = "pf1_building_brick_garage"
NODE_HOUSE_BRICK = "pf1_house_brick"
NODE_HOUSE_GREEN = "pf1_house_green"
NODE_TREE_RING = "pf1_tree_ring"
NODE_TRUNK_PREFIX = "pf1_tree_trunk_"
TRUNK_COUNT = 4

#: Every node name the committed depth proxy must carry, in authoring order.
PROXY_NODE_NAMES: tuple[str, ...] = (
    NODE_GROUND,
    NODE_GARAGE,
    NODE_HOUSE_BRICK,
    NODE_HOUSE_GREEN,
    f"{NODE_TRUNK_PREFIX}0",
    f"{NODE_TRUNK_PREFIX}1",
    f"{NODE_TRUNK_PREFIX}2",
    f"{NODE_TRUNK_PREFIX}3",
    NODE_TREE_RING,
)

#: Near trunks, MANUALLY CALIBRATED from the perspective crops:
#: (azimuth deg, distance m). Trunk cross-section 0.7 m x 0.7 m, 22 m tall.
TRUNK_PLACEMENTS: tuple[tuple[float, float], ...] = (
    (152.0, 5.0),
    (270.0, 7.0),
    (285.0, 9.0),
    (300.0, 6.0),
)
TRUNK_WIDTH_M = 0.7
TRUNK_DEPTH_M = 0.7
TRUNK_HEIGHT_M = 22.0

#: Distant obstacles, MANUALLY CALIBRATED from the perspective crops:
#: (node name, azimuth deg, distance m, width m, depth m, height m).
OBSTACLE_PLACEMENTS: tuple[tuple[str, float, float, float, float, float], ...] = (
    (NODE_HOUSE_BRICK, 268.0, 15.0, 10.0, 7.0, 4.5),
    (NODE_HOUSE_GREEN, 286.0, 32.0, 12.0, 8.0, 5.5),
)

#: Distant tree line: one merged node of 48 overlapping boxes on a 30 m ring.
TREE_RING_COUNT = 48
TREE_RING_RADIUS_M = 30.0
TREE_RING_JITTER_M = 3.0
TREE_RING_WIDTH_M = 7.0
TREE_RING_DEPTH_M = 5.0
TREE_RING_HEIGHT_M = 40.0
#: Frozen LCG seed. Arbitrary, but the committed GLB bytes depend on it, so it
#: is a constant and never read from the clock or from `random`.
TREE_RING_JITTER_SEED = 1


class PhotoFieldError(Exception):
    """Raised for every fail-closed condition in the PF1 asset tooling."""

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
    """Gitignored Poly Haven source cache (``tmp/``)."""
    return (root or repo_root()) / SOURCE_CACHE_RELATIVE


def runtime_dir(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed runtime asset directory of the PF1 photo field."""
    return (root or repo_root()) / RUNTIME_DIR_RELATIVE


def runtime_manifest_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed runtime manifest parsed by ``photo_field.rs``."""
    return (root or repo_root()) / RUNTIME_MANIFEST_RELATIVE


def provenance_manifest_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed provenance record."""
    return (root or repo_root()) / PROVENANCE_MANIFEST_RELATIVE


def source_path(role: str, root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Cache path of one registered source file."""
    return source_cache_dir(root) / source_file_for_role(role).file_name


def panorama_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed display-referred panorama derivative."""
    return runtime_dir(root) / PANORAMA_FILE_NAME


def depth_proxy_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Committed coarse depth-proxy GLB."""
    return runtime_dir(root) / DEPTH_PROXY_FILE_NAME


def receipt_path(root: Optional[pathlib.Path] = None) -> pathlib.Path:
    """Fetch receipt inside the gitignored source cache."""
    return source_cache_dir(root) / RECEIPT_NAME


def require_file(path: pathlib.Path, hint: str) -> pathlib.Path:
    """Return ``path`` or fail closed with an actionable message."""
    if not path.is_file():
        raise PhotoFieldError(f"missing required file: {path}\n  {hint}", exit_code=2)
    return path


# ---------------------------------------------------------------------------
# Digests and headers
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
    """MD5 of a file's bytes, streamed.

    Poly Haven publishes MD5 (not SHA-256) per download leaf, so this is the
    only way to verify a download against the API. It is used for source
    verification only; every digest recorded for provenance is SHA-256.
    """
    digest = hashlib.md5()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(_CHUNK), b""):
            digest.update(chunk)
    return digest.hexdigest()


def file_facts(path: pathlib.Path) -> dict[str, Any]:
    """Measured facts about a file on disk: size and both digests."""
    require_file(path, "expected an existing file to measure")
    return {
        "byte_size": path.stat().st_size,
        "sha256": sha256_file(path),
        "md5": md5_file(path),
    }


#: Standalone JPEG markers (no length field) that the scanner must skip.
_JPEG_STANDALONE_MARKERS = frozenset(
    {0x01, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8}
)
#: SOFn markers that are NOT frame headers (DHT, JPG extensions, DAC).
_JPEG_NON_SOF_MARKERS = frozenset({0xC4, 0xC8, 0xCC})


def read_jpeg_header(path: pathlib.Path) -> dict[str, Any]:
    """Parse the first SOFn of a baseline JPEG without an image library.

    Returns ``width``, ``height``, ``precision``, ``components`` and
    ``sof_marker``. Reading a JPEG header is a marker walk, and keeping it in
    the standard library means the contract tests do not depend on the same
    image library that produced the file. Fails closed on a bad signature, on
    entropy data reached before a frame header, and on a truncated segment.
    """
    data = path.read_bytes()
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        raise PhotoFieldError(f"{path}: not a JPEG (missing SOI marker)")
    position = 2
    while True:
        if position >= len(data):
            raise PhotoFieldError(f"{path}: ended before a frame header (SOFn)")
        if data[position] != 0xFF:
            raise PhotoFieldError(
                f"{path}: byte {position} is 0x{data[position]:02X}, not a JPEG marker prefix"
            )
        while position < len(data) and data[position] == 0xFF:
            position += 1  # 0xFF fill bytes are legal between markers
        if position >= len(data):
            raise PhotoFieldError(f"{path}: truncated marker prefix")
        marker = data[position]
        position += 1
        if marker in _JPEG_STANDALONE_MARKERS:
            continue
        if marker == 0xD9:
            raise PhotoFieldError(f"{path}: reached EOI without a frame header")
        if position + 2 > len(data):
            raise PhotoFieldError(f"{path}: truncated segment length")
        length = int.from_bytes(data[position : position + 2], "big")
        if length < 2 or position + length > len(data):
            raise PhotoFieldError(
                f"{path}: segment 0x{marker:02X} declares {length} bytes, which does "
                f"not fit the {len(data) - position} remaining"
            )
        if 0xC0 <= marker <= 0xCF and marker not in _JPEG_NON_SOF_MARKERS:
            if length < 8:
                raise PhotoFieldError(f"{path}: frame header is shorter than 8 bytes")
            return {
                "width": int.from_bytes(data[position + 5 : position + 7], "big"),
                "height": int.from_bytes(data[position + 3 : position + 5], "big"),
                "precision": data[position + 2],
                "components": data[position + 7],
                "sof_marker": marker,
            }
        position += length


def describe_jpeg(path: pathlib.Path) -> dict[str, Any]:
    """Measured facts about a JPEG on disk: size, digests and header fields."""
    facts = file_facts(path)
    facts.update(read_jpeg_header(path))
    facts["dimensions"] = [facts["width"], facts["height"]]
    return facts


def load_json(path: pathlib.Path, hint: str) -> Any:
    """Load a JSON document, failing closed on absence or malformed JSON."""
    require_file(path, hint)
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise PhotoFieldError(f"{path}: invalid JSON: {error}") from error


def dump_json(document: Any, path: pathlib.Path) -> None:
    """Write a JSON document with the repository's usual 2-space layout."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")


def configure_streams() -> None:
    """Make redirected stdout/stderr UTF-8 and loss-tolerant.

    Windows redirects default to cp1252, which raises ``UnicodeEncodeError`` on
    non-ASCII markers and fails every test in a suite. PF1 additionally prints
    ASCII only, so this is a belt-and-braces guard rather than a licence to
    print non-ASCII.
    """
    import sys

    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass


# ---------------------------------------------------------------------------
# Radiance RGBE (.hdr) decoding
# ---------------------------------------------------------------------------

RGBE_FORMAT = "32-bit_rle_rgbe"
_RGBE_EXPONENT_BIAS = 128 + 8


def parse_rgbe_header(buffer: bytes) -> dict[str, Any]:
    """Parse a Radiance text header.

    Returns ``width``, ``height``, ``variables`` and ``data_offset`` (the first
    byte after the resolution line). Only ``-Y ... +X ...`` scanline order is
    accepted: it is what Poly Haven publishes and what puts row 0 at the top of
    the image, which the equirectangular convention depends on.
    """
    if not buffer.startswith(b"#?RADIANCE") and not buffer.startswith(b"#?RGBE"):
        raise PhotoFieldError("not a Radiance RGBE file (bad magic)")
    end = buffer.find(b"\n\n")
    if end < 0:
        raise PhotoFieldError("truncated Radiance header (no blank line)")
    variables: dict[str, str] = {}
    for line in buffer[:end].decode("ascii", "replace").splitlines()[1:]:
        if "=" in line:
            key, _, value = line.partition("=")
            variables[key.strip()] = value.strip()
    if variables.get("FORMAT") != RGBE_FORMAT:
        raise PhotoFieldError(
            f"unsupported Radiance FORMAT {variables.get('FORMAT')!r}; PF1 decodes "
            f"{RGBE_FORMAT!r} only"
        )

    rest = buffer[end + 2 :]
    newline = rest.find(b"\n")
    if newline < 0:
        raise PhotoFieldError("missing Radiance resolution line")
    tokens = rest[:newline].decode("ascii", "replace").split()
    if len(tokens) != 4:
        raise PhotoFieldError(f"malformed Radiance resolution line {tokens!r}")
    y_sign, y_count, x_sign, x_count = tokens
    if y_sign not in ("-Y", "+Y") or x_sign not in ("-X", "+X"):
        raise PhotoFieldError(f"unsupported resolution orientation {tokens!r}")
    if y_sign != "-Y":
        raise PhotoFieldError(
            "+Y (bottom-to-top) scanline order would flip the panorama; PF1 "
            "requires -Y so that row 0 is the zenith"
        )
    try:
        height, width = int(y_count), int(x_count)
    except ValueError as error:
        raise PhotoFieldError(f"non-integer Radiance resolution {tokens!r}") from error
    if width <= 0 or height <= 0:
        raise PhotoFieldError(f"non-positive Radiance resolution {tokens!r}")
    return {
        "width": width,
        "height": height,
        "variables": variables,
        "data_offset": end + 2 + newline + 1,
    }


def decode_rgbe_channel(buffer: bytes, position: int, width: int) -> tuple[bytes, int]:
    """Decode one new-style RLE channel of one scanline.

    Promoted verbatim from the validated PF1 prototype: a run byte above 128
    repeats the next byte ``count - 128`` times, otherwise ``count`` literal
    bytes follow. Any overrun or truncation fails closed.
    """
    out = bytearray()
    append = out.extend
    total = len(buffer)
    while len(out) < width:
        if position >= total:
            raise PhotoFieldError("unexpected end of data inside an RLE channel")
        count = buffer[position]
        position += 1
        if count > 128:
            run = count - 128
            if position >= total:
                raise PhotoFieldError("unexpected end of data inside an RLE run")
            append(bytes([buffer[position]]) * run)
            position += 1
        else:
            if count == 0:
                raise PhotoFieldError("zero-length RLE literal run")
            append(buffer[position : position + count])
            position += count
    if len(out) != width:
        raise PhotoFieldError(f"channel overran the scanline: {len(out)} > {width}")
    return bytes(out), position


def rgbe_to_float(rgbe: np.ndarray) -> np.ndarray:
    """Convert uint8 RGBE samples to float32 radiance.

    Radiance's own ``rgbe2float``: ``f = ldexp(1.0, e - (128 + 8))`` and
    ``rgb = rgbe[:3] * f``. An exponent of zero encodes exactly zero.
    """
    exponent = rgbe[..., 3].astype(np.int32)
    scale = np.zeros(exponent.shape, dtype=np.float64)
    nonzero = exponent > 0
    scale[nonzero] = np.ldexp(1.0, exponent[nonzero] - _RGBE_EXPONENT_BIAS)
    return (rgbe[..., :3].astype(np.float64) * scale[..., None]).astype(np.float32)


def iter_rgbe_bands(
    path: pathlib.Path, band_rows: int = 256
) -> Iterator[tuple[int, np.ndarray, dict[str, Any]]]:
    """Decode a Radiance ``.hdr`` file, yielding ``(row_start, band, meta)``.

    ``band`` is a float32 ``(rows, width, 3)`` radiance array. Decoding in bands
    bounds peak memory: an 8192x4096 panorama is ~100 MB of RGBE bytes but
    ~800 MB of float64 temporaries if converted in one piece.
    """
    if band_rows <= 0:
        raise PhotoFieldError(f"band_rows must be positive, got {band_rows}")
    require_file(path, "run tools/photo_field_pipeline/fetch_photo_field_sources.py first")
    buffer = path.read_bytes()
    header = parse_rgbe_header(buffer)
    width, height = header["width"], header["height"]
    meta = {
        "width": width,
        "height": height,
        "variables": header["variables"],
        "byte_size": len(buffer),
        "format": header["variables"].get("FORMAT"),
    }
    position = header["data_offset"]
    for start in range(0, height, band_rows):
        rows = min(band_rows, height - start)
        rgbe = np.zeros((rows, width, 4), dtype=np.uint8)
        for row in range(rows):
            scanline = start + row
            marker = buffer[position : position + 4]
            if len(marker) < 4:
                raise PhotoFieldError(f"truncated scanline marker at row {scanline}")
            if marker[0] != 2 or marker[1] != 2:
                raise PhotoFieldError(
                    f"scanline {scanline} is not new-style RLE (marker {list(marker)}); "
                    "old-style RLE and uncompressed RGBE are not supported"
                )
            declared = (marker[2] << 8) | marker[3]
            if declared != width:
                raise PhotoFieldError(
                    f"scanline {scanline} declares width {declared} but the header "
                    f"says {width}"
                )
            position += 4
            for channel in range(4):
                raw, position = decode_rgbe_channel(buffer, position, width)
                rgbe[row, :, channel] = np.frombuffer(raw, dtype=np.uint8)
        band = rgbe_to_float(rgbe)
        if not bool(np.isfinite(band).all()):
            raise PhotoFieldError(
                f"decoded radiance is not finite in rows [{start}, {start + rows}) of "
                f"{path.name}; refusing to process a corrupt or extreme source"
            )
        yield start, band, meta


def decode_rgbe(path: pathlib.Path, band_rows: int = 256) -> tuple[np.ndarray, dict]:
    """Decode a whole Radiance ``.hdr`` into one float32 array.

    Convenience wrapper over :func:`iter_rgbe_bands` for small sources (the 1k
    decode probe). The 8k pipeline consumes the generator directly.
    """
    bands = []
    meta: dict[str, Any] = {}
    for _, band, meta in iter_rgbe_bands(path, band_rows=band_rows):
        bands.append(band)
    if not bands:
        raise PhotoFieldError(f"{path}: decoded zero scanline bands")
    return np.concatenate(bands, axis=0), meta


def luminance(rgb: np.ndarray) -> np.ndarray:
    """Rec.709 luminance of an ``(..., 3)`` radiance array."""
    return 0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2]


# ---------------------------------------------------------------------------
# Colour: the renderer's tone mapper and transfer functions
# ---------------------------------------------------------------------------


def khronos_pbr_neutral(color: np.ndarray) -> np.ndarray:
    """Vectorised port of ``fn khronos_pbr_neutral`` in ``shader.wgsl``.

    Statement-by-statement, so the offline derivative and the GPU aircraft are
    tonemapped by one curve::

        let start_compression = 0.8 - 0.04;
        let desaturation = 0.15;
        let x = min(color.r, min(color.g, color.b));
        let offset = select(0.04, x - 6.25 * x * x, x < 0.08);
        var c = color - vec3<f32>(offset);
        let peak = max(c.r, max(c.g, c.b));
        if (peak < start_compression) { return c; }
        let d = 1.0 - start_compression;
        let new_peak = 1.0 - d * d / (peak + d - start_compression);
        c = c * vec3<f32>(new_peak / peak);
        let g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
        return mix(c, vec3<f32>(new_peak), g);

    WGSL ``select(f, t, cond)`` yields ``t`` when ``cond`` is true, so the
    offset is ``x - 6.25 * x * x`` for ``x < 0.08`` and the constant ``0.04``
    otherwise (the two branches meet at exactly 0.04 when x == 0.08).

    Evaluated in float64 from float32 radiance. The GPU evaluates the same math
    in f32, so the derivative can differ from a framebuffer readback by ~1e-7
    relative - three orders of magnitude below the 8-bit quantisation it is
    about to be rounded to.
    """
    start_compression = 0.8 - 0.04
    desaturation = 0.15

    values = np.asarray(color, dtype=np.float64)
    x = np.minimum(np.minimum(values[..., 0], values[..., 1]), values[..., 2])
    offset = np.where(x < 0.08, x - 6.25 * x * x, 0.04)
    c = values - offset[..., None]

    peak = np.maximum(np.maximum(c[..., 0], c[..., 1]), c[..., 2])
    below = peak < start_compression

    d = 1.0 - start_compression
    # The early-out branch never uses these; substituting 1.0 for `peak` keeps
    # the division finite so `np.where` can select the returned branch.
    safe_peak = np.where(below, 1.0, peak)
    new_peak = 1.0 - d * d / (safe_peak + d - start_compression)
    scaled = c * (new_peak / safe_peak)[..., None]

    g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0)
    mixed = scaled * (1.0 - g)[..., None] + new_peak[..., None] * g[..., None]
    return np.where(below[..., None], c, mixed)


def linear_to_srgb(linear: np.ndarray) -> np.ndarray:
    """sRGB (IEC 61966-2-1) encode, matching ``linear_to_srgb_f64`` in texture.rs.

    Clamped to [0, 1] first, exactly like the Rust definition.
    """
    values = np.clip(np.asarray(linear, dtype=np.float64), 0.0, 1.0)
    return np.where(
        values <= 0.0031308,
        values * 12.92,
        1.055 * np.power(values, 1.0 / 2.4) - 0.055,
    )


def srgb_to_linear(srgb: np.ndarray) -> np.ndarray:
    """sRGB (IEC 61966-2-1) decode, matching ``srgb_to_linear_f64`` in texture.rs."""
    values = np.clip(np.asarray(srgb, dtype=np.float64), 0.0, 1.0)
    return np.where(
        values <= 0.04045,
        values / 12.92,
        np.power((values + 0.055) / 1.055, 2.4),
    )


def display_referred_u8(display_linear: np.ndarray) -> np.ndarray:
    """Quantise display-referred linear values to the committed 8-bit bytes.

    ``round(linear_to_srgb(x) * 255)`` with round-half-to-even, which is both
    Python's ``round()`` and numpy's ``np.rint`` - a single documented rounding
    rule, so the derivative is byte-reproducible.
    """
    encoded = linear_to_srgb(display_linear) * 255.0
    return np.rint(encoded).astype(np.uint8)


# ---------------------------------------------------------------------------
# Equirectangular mapping (mirrors crates/renderer/src/photo_field.rs)
# ---------------------------------------------------------------------------


def equirect_uv_from_direction(
    direction: Sequence[float], yaw_deg: float = 0.0, pitch_deg: float = 0.0
) -> tuple[float, float]:
    """Panorama UV of a render-space direction, mirroring the runtime exactly.

    ``photo_field.rs`` documents and implements::

        u = fract(azimuth / 2pi)      azimuth  = atan2(dir.z, dir.x)
        v = 0.5 - elevation / pi      elevation = asin(dir.y), v = 0 at the top

    with the world -> panorama calibration applied as a yaw about +Y followed by
    a pitch about the rotated +X. The yaw is SUBTRACTED (as in the runtime): a
    calibration of +yaw puts panorama longitude 0 on world azimuth +yaw. Row 0 of
    the committed panorama is the zenith. The zero vector maps to the panorama
    centre row rather than to a NaN, exactly as the runtime does.
    """
    dx, dy, dz = (float(component) for component in direction)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length > 1e-9:
        dx, dy, dz = dx / length, dy / length, dz / length
    else:
        dx, dy, dz = 0.0, 1.0, 0.0

    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    sin_yaw, cos_yaw = math.sin(yaw), math.cos(yaw)
    x1 = dx * cos_yaw + dz * sin_yaw
    z1 = -dx * sin_yaw + dz * cos_yaw
    sin_pitch, cos_pitch = math.sin(pitch), math.cos(pitch)
    y2 = dy * cos_pitch + z1 * sin_pitch
    z2 = -dy * sin_pitch + z1 * cos_pitch

    azimuth = math.atan2(z2, x1)
    u = azimuth / (2.0 * math.pi)
    u = u - math.floor(u)  # fract()
    elevation = math.asin(min(1.0, max(-1.0, y2)))
    v = min(1.0, max(0.0, 0.5 - elevation / math.pi))
    return u, v


def direction_from_azimuth_elevation(
    azimuth_deg: float, elevation_deg: float
) -> tuple[float, float, float]:
    """Render-space unit direction of a panorama azimuth/elevation, in degrees.

    The inverse of :func:`equirect_uv_from_direction` at zero calibration:
    ``azimuth = atan2(z, x)`` gives ``x = cos(el) * cos(az)`` and
    ``z = cos(el) * sin(az)``; ``elevation = asin(y)`` gives ``y = sin(el)``.
    """
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    cos_elevation = math.cos(elevation)
    return (
        cos_elevation * math.cos(azimuth),
        math.sin(elevation),
        cos_elevation * math.sin(azimuth),
    )


def sun_direction_render_from_angles(
    longitude_deg: float = SUN_LONGITUDE_DEG, elevation_deg: float = SUN_ELEVATION_DEG
) -> tuple[float, float, float]:
    """Direction towards the photographed sun, in render world space.

    The panorama longitude IS the render azimuth under the documented
    convention, so this is :func:`direction_from_azimuth_elevation` with the
    measured solar disc position.
    """
    return direction_from_azimuth_elevation(longitude_deg, elevation_deg)


def horizontal_direction(azimuth_deg: float) -> tuple[float, float, float]:
    """Unit vector on the ground plane at a render azimuth (y == 0)."""
    azimuth = math.radians(azimuth_deg)
    return (math.cos(azimuth), 0.0, math.sin(azimuth))


def proxy_origin_from_eye(azimuth_deg: float, distance_m: float) -> tuple[float, float]:
    """World (x, z) of a proxy placed ``distance_m`` from the eye at an azimuth.

    Every proxy position is expressed relative to the photographic eye, so the
    coarse depth geometry agrees with the panorama it occludes.
    """
    direction = horizontal_direction(azimuth_deg)
    return (
        PILOT_EYE_RENDER_M[0] + distance_m * direction[0],
        PILOT_EYE_RENDER_M[2] + distance_m * direction[2],
    )


def eye_to_spawn_azimuth_deg() -> float:
    """Panorama azimuth, seen from the photographic eye, of the aircraft spawn.

    Derived from ``PILOT_EYE_RENDER_M`` rather than stored, so the provenance
    record can never quote a placement the eye no longer has.
    """
    to_spawn = (-PILOT_EYE_RENDER_M[0], 0.0, -PILOT_EYE_RENDER_M[2])
    u, _ = equirect_uv_from_direction(to_spawn)
    return u * 360.0


# ---------------------------------------------------------------------------
# Garage derivation: angular box + assumed camera height -> metres
# ---------------------------------------------------------------------------


def derive_box_from_angular_box(
    base_elevation_deg: float,
    top_elevation_deg: float,
    azimuth_span_deg: float,
    camera_height_m: float = CAMERA_HEIGHT_M,
) -> dict[str, float]:
    """Derive distance, height and tangential width of a photographed box.

    With the camera at ``camera_height_m`` over a flat ground plane, an object
    whose base sits at elevation ``-b`` is at ``distance = h / tan(b)``, and a
    top at elevation ``+t`` then implies ``height = distance * tan(t) + h``.
    The width is the ARC length subtended by the azimuth span at that distance
    (``distance * span_rad``); the straight chord is slightly shorter and is
    recorded alongside so the choice is visible rather than implicit.
    """
    if camera_height_m <= 0.0:
        raise PhotoFieldError(f"camera height must be positive, got {camera_height_m}")
    if not 0.0 < base_elevation_deg < 90.0:
        raise PhotoFieldError(
            "the base elevation must be a small depression angle below the horizon, "
            f"got {base_elevation_deg}"
        )
    if not 0.0 <= top_elevation_deg < 90.0:
        raise PhotoFieldError(f"invalid top elevation {top_elevation_deg}")
    if not 0.0 < azimuth_span_deg < 360.0:
        raise PhotoFieldError(f"invalid azimuth span {azimuth_span_deg}")

    distance_m = camera_height_m / math.tan(math.radians(base_elevation_deg))
    height_m = distance_m * math.tan(math.radians(top_elevation_deg)) + camera_height_m
    span_rad = math.radians(azimuth_span_deg)
    return {
        "camera_height_m": float(camera_height_m),
        "distance_m": float(distance_m),
        "height_m": float(height_m),
        "width_arc_m": float(distance_m * span_rad),
        "width_chord_m": float(2.0 * distance_m * math.sin(span_rad / 2.0)),
    }


def garage_derivation() -> dict[str, float]:
    """The brick garage's metric box, derived from its measured angular box."""
    azimuth_low, azimuth_high = GARAGE_ANGULAR_BOX_AZIMUTH_DEG
    base_elevation, top_elevation = GARAGE_ANGULAR_BOX_ELEVATION_DEG
    derived = derive_box_from_angular_box(
        base_elevation_deg=-base_elevation,
        top_elevation_deg=top_elevation,
        azimuth_span_deg=azimuth_high - azimuth_low,
    )
    for key, expected in (
        ("distance_m", GARAGE_DISTANCE_M_EXPECTED),
        ("height_m", GARAGE_HEIGHT_M_EXPECTED),
        ("width_arc_m", GARAGE_WIDTH_M_EXPECTED),
    ):
        if abs(derived[key] - expected) > GARAGE_DERIVATION_TOLERANCE_M:
            raise PhotoFieldError(
                f"the garage derivation moved: {key} = {derived[key]:.4f} m but the "
                f"calibrated value is {expected} m (+/- {GARAGE_DERIVATION_TOLERANCE_M})"
            )
    derived["azimuth_deg"] = float(GARAGE_CENTROID_AZIMUTH_DEG)
    return derived


# ---------------------------------------------------------------------------
# Deterministic jitter (no `random`, no wall clock)
# ---------------------------------------------------------------------------


class DeterministicLcg:
    """A 64-bit linear congruential generator (Knuth's MMIX constants).

    The committed GLB bytes depend on the tree-ring jitter, so the sequence must
    be reproducible on every machine and every Python version. The standard
    library's ``random`` is deliberately NOT used: its algorithm is an
    implementation detail that may change, and it invites seeding from the clock.
    """

    MULTIPLIER = 6364136223846793005
    INCREMENT = 1442695040888963407
    MODULUS = 1 << 64

    def __init__(self, seed: int) -> None:
        if not isinstance(seed, int) or isinstance(seed, bool):
            raise PhotoFieldError(f"the LCG seed must be an int, got {seed!r}")
        self._state = seed % self.MODULUS

    def next_unit(self) -> float:
        """The next value in [0, 1), from the top 53 bits of the state."""
        self._state = (self.MULTIPLIER * self._state + self.INCREMENT) % self.MODULUS
        return (self._state >> 11) / float(1 << 53)

    def next_symmetric(self) -> float:
        """The next value in [-1, 1)."""
        return self.next_unit() * 2.0 - 1.0


# ---------------------------------------------------------------------------
# Minimal deterministic glTF 2.0 binary (GLB) writer
# ---------------------------------------------------------------------------

GLB_MAGIC = 0x46546C67
GLB_VERSION = 2
GLB_CHUNK_JSON = 0x4E4F534A
GLB_CHUNK_BIN = 0x004E4942
GLB_COMPONENT_FLOAT32 = 5126
GLB_COMPONENT_UINT32 = 5125
GLB_TARGET_ARRAY_BUFFER = 34962
GLB_TARGET_ELEMENT_ARRAY_BUFFER = 34963
GLB_MODE_TRIANGLES = 4
GLB_GENERATOR = "rc-simulation-engine PF1 author_photo_field_proxies.py (glTF 2.0)"


class GlbMesh:
    """One named triangle mesh in render world space.

    ``positions`` are float triples, ``indices`` uint32-representable integers
    forming triangles (a multiple of three). No materials, no textures, no
    normals: the runtime uses these meshes depth-only, and ``glb.rs`` falls back
    to generated normals when a primitive carries none.
    """

    __slots__ = ("name", "positions", "indices")

    def __init__(
        self,
        name: str,
        positions: Sequence[Sequence[float]],
        indices: Sequence[int],
    ) -> None:
        self.name = name
        self.positions = [(float(p[0]), float(p[1]), float(p[2])) for p in positions]
        self.indices = [int(index) for index in indices]
        self._validate()

    def _validate(self) -> None:
        if not self.name:
            raise PhotoFieldError("a GLB mesh name must not be empty")
        if not self.positions:
            raise PhotoFieldError(f"{self.name}: a mesh needs at least one position")
        if len(self.indices) % 3 != 0:
            raise PhotoFieldError(
                f"{self.name}: index count {len(self.indices)} is not a multiple of 3"
            )
        if not self.indices:
            raise PhotoFieldError(f"{self.name}: a mesh needs at least one triangle")
        highest = max(self.indices)
        if min(self.indices) < 0 or highest >= len(self.positions):
            raise PhotoFieldError(
                f"{self.name}: index range [{min(self.indices)}, {highest}] does not fit "
                f"{len(self.positions)} vertices"
            )
        for position in self.positions:
            if not all(math.isfinite(component) for component in position):
                raise PhotoFieldError(f"{self.name}: non-finite position {position}")

    @property
    def triangle_count(self) -> int:
        return len(self.indices) // 3

    @property
    def vertex_count(self) -> int:
        return len(self.positions)

    def bounds(self) -> tuple[list[float], list[float]]:
        """Exact float32 (min, max) of the stored positions, per axis."""
        flat = struct.pack("<%df" % (3 * len(self.positions)), *[c for p in self.positions for c in p])
        values = struct.unpack("<%df" % (3 * len(self.positions)), flat)
        minimum = [min(values[axis::3]) for axis in range(3)]
        maximum = [max(values[axis::3]) for axis in range(3)]
        return minimum, maximum


def write_glb(meshes: Sequence[GlbMesh], generator: str = GLB_GENERATOR) -> bytes:
    """Serialise named world-space meshes into one deterministic GLB 2.0 file.

    Layout: a 12-byte header, a space-padded JSON chunk and a zero-padded BIN
    chunk holding one buffer. Every mesh gets its own POSITION accessor
    (float32 VEC3 with exact min/max, as the glTF spec requires), its own
    uint32 index accessor, and its own node carrying an explicit identity TRS.

    The identity TRS is deliberate: ``tools/aircraft_asset_pipeline/
    blender_export_glb.py`` documents that this repository's production loader
    reads baked vertex positions and does NOT apply glTF node transforms, while
    ``glb.rs``'s scene-graph path does compose them. Authoring in render world
    space with an identity node transform makes the two readings agree.

    Deterministic by construction: no timestamps, no dictionaries iterated in
    insertion-dependent order (``sort_keys``), and byte offsets that are exact
    multiples of four so the BIN chunk needs no padding.
    """
    if not meshes:
        raise PhotoFieldError("write_glb needs at least one mesh")
    names = [mesh.name for mesh in meshes]
    if len(set(names)) != len(names):
        duplicates = sorted({name for name in names if names.count(name) > 1})
        raise PhotoFieldError(f"duplicate GLB node names: {duplicates}")

    bin_data = bytearray()
    buffer_views: list[dict[str, Any]] = []
    accessors: list[dict[str, Any]] = []
    gltf_meshes: list[dict[str, Any]] = []
    nodes: list[dict[str, Any]] = []

    for mesh in meshes:
        position_offset = len(bin_data)
        bin_data.extend(
            struct.pack(
                "<%df" % (3 * len(mesh.positions)),
                *[component for position in mesh.positions for component in position],
            )
        )
        position_length = len(bin_data) - position_offset
        index_offset = len(bin_data)
        bin_data.extend(struct.pack("<%dI" % len(mesh.indices), *mesh.indices))
        index_length = len(bin_data) - index_offset
        if position_length % 4 or index_length % 4:
            raise PhotoFieldError(
                f"{mesh.name}: internal error, a buffer view is not 4-byte aligned"
            )

        minimum, maximum = mesh.bounds()
        position_view = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": position_offset,
                "byteLength": position_length,
                "target": GLB_TARGET_ARRAY_BUFFER,
            }
        )
        index_view = len(buffer_views)
        buffer_views.append(
            {
                "buffer": 0,
                "byteOffset": index_offset,
                "byteLength": index_length,
                "target": GLB_TARGET_ELEMENT_ARRAY_BUFFER,
            }
        )
        position_accessor = len(accessors)
        accessors.append(
            {
                "bufferView": position_view,
                "componentType": GLB_COMPONENT_FLOAT32,
                "count": mesh.vertex_count,
                "type": "VEC3",
                "min": minimum,
                "max": maximum,
            }
        )
        index_accessor = len(accessors)
        accessors.append(
            {
                "bufferView": index_view,
                "componentType": GLB_COMPONENT_UINT32,
                "count": len(mesh.indices),
                "type": "SCALAR",
            }
        )
        gltf_meshes.append(
            {
                "name": mesh.name,
                "primitives": [
                    {
                        "attributes": {"POSITION": position_accessor},
                        "indices": index_accessor,
                        "mode": GLB_MODE_TRIANGLES,
                    }
                ],
            }
        )
        nodes.append(
            {
                "name": mesh.name,
                "mesh": len(gltf_meshes) - 1,
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0, 1.0],
                "scale": [1.0, 1.0, 1.0],
            }
        )

    if len(bin_data) % 4:
        raise PhotoFieldError("internal error: the BIN chunk is not 4-byte aligned")

    document = {
        "asset": {"version": "2.0", "generator": generator},
        "scene": 0,
        "scenes": [{"name": "pf1-depth-proxies", "nodes": list(range(len(nodes)))}],
        "nodes": nodes,
        "meshes": gltf_meshes,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(bin_data)}],
    }
    json_chunk = json.dumps(document, separators=(",", ":"), sort_keys=True).encode("utf-8")
    json_chunk += b" " * ((4 - len(json_chunk) % 4) % 4)

    total = 12 + 8 + len(json_chunk) + 8 + len(bin_data)
    return (
        struct.pack("<III", GLB_MAGIC, GLB_VERSION, total)
        + struct.pack("<II", len(json_chunk), GLB_CHUNK_JSON)
        + json_chunk
        + struct.pack("<II", len(bin_data), GLB_CHUNK_BIN)
        + bytes(bin_data)
    )


def read_glb(data: bytes) -> tuple[dict[str, Any], bytes]:
    """Split a GLB container into its JSON document and BIN chunk, failing closed."""
    if len(data) < 12:
        raise PhotoFieldError(f"file too small for a GLB header ({len(data)} bytes)")
    magic, version, length = struct.unpack_from("<III", data, 0)
    if magic != GLB_MAGIC:
        raise PhotoFieldError(f"bad GLB magic 0x{magic:08X}, expected 0x{GLB_MAGIC:08X}")
    if version != GLB_VERSION:
        raise PhotoFieldError(f"GLB version {version}, expected {GLB_VERSION}")
    if length != len(data):
        raise PhotoFieldError(f"GLB header length {length} != file size {len(data)}")

    offset = 12
    document: Optional[dict[str, Any]] = None
    bin_chunk: Optional[bytes] = None
    while offset < len(data):
        if offset + 8 > len(data):
            raise PhotoFieldError(f"truncated GLB chunk header at byte {offset}")
        chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
        start = offset + 8
        end = start + chunk_length
        if end > len(data):
            raise PhotoFieldError(
                f"GLB chunk 0x{chunk_type:08X} declares {chunk_length} bytes but only "
                f"{len(data) - start} remain"
            )
        payload = data[start:end]
        if chunk_type == GLB_CHUNK_JSON:
            if document is not None:
                raise PhotoFieldError("more than one JSON chunk")
            document = json.loads(payload.decode("utf-8"))
        elif chunk_type == GLB_CHUNK_BIN:
            if bin_chunk is not None:
                raise PhotoFieldError("more than one BIN chunk")
            bin_chunk = payload
        else:
            raise PhotoFieldError(f"unknown GLB chunk type 0x{chunk_type:08X}")
        offset = end

    if document is None:
        raise PhotoFieldError("the GLB has no JSON chunk")
    if bin_chunk is None:
        raise PhotoFieldError("the GLB has no BIN chunk")
    return document, bin_chunk


def _glb_array(document: dict[str, Any], key: str) -> list:
    value = document.get(key, [])
    if not isinstance(value, list):
        raise PhotoFieldError(f"GLB top-level {key!r} must be an array")
    return value


def _glb_entry(items: list, index: Any, where: str) -> dict:
    """One bounds-checked object of a glTF array, failing closed."""
    if isinstance(index, bool) or not isinstance(index, int):
        raise PhotoFieldError(f"{where}: index {index!r} is not an integer")
    if not 0 <= index < len(items):
        raise PhotoFieldError(f"{where}: index {index} is out of range ({len(items)})")
    entry = items[index]
    if not isinstance(entry, dict):
        raise PhotoFieldError(f"{where}: entry {index} is not an object")
    return entry


def glb_summary(document: dict[str, Any]) -> dict[str, Any]:
    """Measured facts about a parsed GLB document: names, counts, bounds."""
    nodes = _glb_array(document, "nodes")
    meshes = _glb_array(document, "meshes")
    accessors = _glb_array(document, "accessors")
    buffer_views = _glb_array(document, "bufferViews")
    buffers = _glb_array(document, "buffers")

    node_names = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            raise PhotoFieldError(f"nodes[{index}] is not an object")
        name = node.get("name")
        if not isinstance(name, str) or not name:
            raise PhotoFieldError(f"nodes[{index}] has no usable name")
        node_names.append(name)

    triangles = 0
    vertices = 0
    for index, mesh in enumerate(meshes):
        primitives = mesh.get("primitives") if isinstance(mesh, dict) else None
        if not isinstance(primitives, list) or not primitives:
            raise PhotoFieldError(f"meshes[{index}] has no primitives")
        for primitive_index, primitive in enumerate(primitives):
            where = f"meshes[{index}].primitives[{primitive_index}]"
            if not isinstance(primitive, dict):
                raise PhotoFieldError(f"{where} is not an object")
            if primitive.get("mode", GLB_MODE_TRIANGLES) != GLB_MODE_TRIANGLES:
                raise PhotoFieldError(f"{where} is not TRIANGLES")
            attributes = primitive.get("attributes")
            if not isinstance(attributes, dict) or "POSITION" not in attributes:
                raise PhotoFieldError(f"{where} has no POSITION attribute")
            position = _glb_entry(accessors, attributes["POSITION"], f"{where}.POSITION")
            vertices += int(position["count"])
            if "indices" not in primitive:
                raise PhotoFieldError(f"{where} has no indices")
            index_accessor = _glb_entry(accessors, primitive["indices"], f"{where}.indices")
            count = int(index_accessor["count"])
            if count % 3:
                raise PhotoFieldError(f"{where} index count {count} is not a multiple of 3")
            triangles += count // 3

    return {
        "node_names": node_names,
        "mesh_count": len(meshes),
        "node_count": len(nodes),
        "accessor_count": len(accessors),
        "buffer_view_count": len(buffer_views),
        "buffer_count": len(buffers),
        "vertex_count": vertices,
        "triangle_count": triangles,
    }


def glb_positions(document: dict[str, Any], bin_chunk: bytes, mesh_index: int) -> list:
    """Decode one mesh's POSITION accessor back to float triples."""
    meshes = _glb_array(document, "meshes")
    accessors = _glb_array(document, "accessors")
    buffer_views = _glb_array(document, "bufferViews")
    where = f"meshes[{mesh_index}]"
    mesh = _glb_entry(meshes, mesh_index, where)
    primitives = mesh.get("primitives")
    if not isinstance(primitives, list) or not primitives:
        raise PhotoFieldError(f"{where} has no primitives")
    primitive = primitives[0]
    attributes = primitive.get("attributes") if isinstance(primitive, dict) else None
    if not isinstance(attributes, dict) or "POSITION" not in attributes:
        raise PhotoFieldError(f"{where} has no POSITION attribute")
    accessor = _glb_entry(accessors, attributes["POSITION"], f"{where}.POSITION")
    view = _glb_entry(buffer_views, accessor.get("bufferView"), f"{where}.bufferView")
    if accessor.get("componentType") != GLB_COMPONENT_FLOAT32:
        raise PhotoFieldError(f"{where} POSITION is not float32")
    if accessor.get("type") != "VEC3":
        raise PhotoFieldError(f"{where} POSITION is not VEC3")
    start = int(view.get("byteOffset", 0)) + int(accessor.get("byteOffset", 0))
    count = int(accessor["count"]) * 3
    end = start + count * 4
    if end > len(bin_chunk):
        raise PhotoFieldError(
            f"meshes[{mesh_index}] POSITION needs bytes [{start}, {end}) but the BIN "
            f"chunk is {len(bin_chunk)} bytes"
        )
    values = struct.unpack_from("<%df" % count, bin_chunk, start)
    return [tuple(values[axis : axis + 3]) for axis in range(0, count, 3)]


# ---------------------------------------------------------------------------
# Runtime manifest contract
# ---------------------------------------------------------------------------

#: The exact key set of the committed runtime manifest. ``photo_field.rs``
#: deserialises with ``deny_unknown_fields``, so one extra key is a hard failure.
RUNTIME_MANIFEST_KEYS: tuple[str, ...] = (
    "schema_version",
    "id",
    "panorama",
    "depth_proxy",
    "pilot_position_render_m",
    "panorama_yaw_deg",
    "panorama_pitch_deg",
    "sun_direction_render",
    "sun_intensity",
    "sun_rgb",
    "shadow_strength",
)


def build_runtime_manifest() -> dict[str, Any]:
    """The committed runtime manifest, with the sun direction re-derived.

    Fails closed unless the derived direction agrees with the pinned literal and
    the literal maps back onto the measured solar texel.
    """
    derived = sun_direction_render_from_angles()
    for axis, (pinned, exact) in enumerate(
        zip(SUN_DIRECTION_RENDER, SUN_DIRECTION_EXACT_F64)
    ):
        if abs(pinned - derived[axis]) > SUN_DIRECTION_TOLERANCE:
            raise PhotoFieldError(
                f"sun_direction_render[{axis}] = {pinned} disagrees with the "
                f"recomputed {derived[axis]:.6f} from longitude "
                f"{SUN_LONGITUDE_DEG} / elevation {SUN_ELEVATION_DEG}"
            )
        if abs(exact - derived[axis]) > 1e-5:
            raise PhotoFieldError(
                f"the documented exact evaluation {SUN_DIRECTION_EXACT_F64} no longer "
                f"matches the recomputation {derived}"
            )

    length = math.sqrt(sum(component * component for component in SUN_DIRECTION_RENDER))
    if abs(length - 1.0) > SUN_DIRECTION_TOLERANCE:
        raise PhotoFieldError(f"sun_direction_render is not unit length (|v| = {length})")

    # The pinned literal must land on the measured solar texel. The exact f64
    # derivation lands on it to ~1e-12; the five-decimal literal is ~7e-6 away in
    # u, so the two checks use the tolerances they can actually hold.
    u, v = equirect_uv_from_direction(SUN_DIRECTION_RENDER)
    expected_u = (SUN_LONGITUDE_DEG / 360.0) % 1.0
    expected_v = 0.5 - SUN_ELEVATION_DEG / 180.0
    if abs(u - expected_u) > 1e-4 or abs(v - expected_v) > 1e-4:
        raise PhotoFieldError(
            f"sun_direction_render maps to u={u:.6f} v={v:.6f}, not to the measured "
            f"solar texel u={expected_u:.6f} v={expected_v:.6f}: the direction and the "
            "panorama longitude/elevation disagree about the convention"
        )
    exact_u, exact_v = equirect_uv_from_direction(derived)
    if abs(exact_u - expected_u) > 1e-9 or abs(exact_v - expected_v) > 1e-9:
        raise PhotoFieldError(
            f"the recomputed sun direction maps to u={exact_u:.9f} v={exact_v:.9f}, "
            f"not to u={expected_u:.9f} v={expected_v:.9f}"
        )

    return {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "id": ASSET_ID,
        "panorama": PANORAMA_FILE_NAME,
        "depth_proxy": DEPTH_PROXY_FILE_NAME,
        "pilot_position_render_m": list(PILOT_EYE_RENDER_M),
        "panorama_yaw_deg": PANORAMA_YAW_DEG,
        "panorama_pitch_deg": PANORAMA_PITCH_DEG,
        "sun_direction_render": list(SUN_DIRECTION_RENDER),
        "sun_intensity": SUN_INTENSITY,
        "sun_rgb": list(SUN_RGB),
        "shadow_strength": SHADOW_STRENGTH,
    }


def validate_runtime_manifest(manifest: Any) -> list[str]:
    """Human-readable contract violations of a runtime manifest (empty == valid).

    Mirrors the checks ``PhotoFieldManifest::validate`` performs in Rust, so the
    committed file is rejected here before the renderer ever sees it.
    """
    errors: list[str] = []
    if not isinstance(manifest, dict):
        return ["the runtime manifest must be a JSON object"]

    keys = set(manifest)
    expected = set(RUNTIME_MANIFEST_KEYS)
    for missing in sorted(expected - keys):
        errors.append(f"{missing}: required field missing")
    for extra in sorted(keys - expected):
        errors.append(f"{extra}: unknown field (photo_field.rs denies unknown fields)")
    if errors:
        return errors

    if manifest["schema_version"] != MANIFEST_SCHEMA_VERSION:
        errors.append(
            f"schema_version must be {MANIFEST_SCHEMA_VERSION}, got "
            f"{manifest['schema_version']!r}"
        )
    if manifest["id"] != ASSET_ID:
        errors.append(f"id must be {ASSET_ID!r}, got {manifest['id']!r}")
    for field, expected_name in (
        ("panorama", PANORAMA_FILE_NAME),
        ("depth_proxy", DEPTH_PROXY_FILE_NAME),
    ):
        value = manifest[field]
        if not isinstance(value, str) or not value:
            errors.append(f"{field} must be a non-empty file name, got {value!r}")
            continue
        # photo_field.rs rejects any separator or parent reference; PF1 also pins
        # the two file names, because the panorama's name states its dimensions.
        if "/" in value or "\\" in value or value.startswith("."):
            errors.append(f"{field} must be a single relative file name, got {value!r}")
        if value != expected_name:
            errors.append(f"{field} must be {expected_name!r}, got {value!r}")

    eye = manifest["pilot_position_render_m"]
    if not (isinstance(eye, list) and len(eye) == 3) or list(eye) != list(PILOT_EYE_RENDER_M):
        errors.append(
            f"pilot_position_render_m must be {list(PILOT_EYE_RENDER_M)}, got {eye!r}"
        )
    for field in ("panorama_yaw_deg", "panorama_pitch_deg"):
        value = manifest[field]
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            errors.append(f"{field} must be a number, got {value!r}")
        elif not math.isfinite(value):
            errors.append(f"{field} must be finite")
    if abs(manifest["panorama_yaw_deg"]) > 360.0:
        errors.append("panorama_yaw_deg must lie in [-360, 360]")
    if abs(manifest["panorama_pitch_deg"]) > 90.0:
        errors.append("panorama_pitch_deg must lie in [-90, 90]")

    sun = manifest["sun_direction_render"]
    if not (isinstance(sun, list) and len(sun) == 3):
        errors.append(f"sun_direction_render must be a three-element array, got {sun!r}")
    elif not all(
        isinstance(component, (int, float)) and not isinstance(component, bool)
        for component in sun
    ):
        errors.append(f"sun_direction_render must be numeric, got {sun!r}")
    elif not all(math.isfinite(component) for component in sun):
        errors.append("sun_direction_render must be finite")
    else:
        length = math.sqrt(sum(component * component for component in sun))
        if length <= 1e-6:
            errors.append("sun_direction_render must not be the zero vector")
        elif abs(length - 1.0) > SUN_DIRECTION_TOLERANCE:
            errors.append(f"sun_direction_render must be unit length, |v| = {length:.9f}")
        elif list(sun) != list(SUN_DIRECTION_RENDER):
            errors.append(
                f"sun_direction_render must be {list(SUN_DIRECTION_RENDER)}, got {list(sun)}"
            )

    intensity = manifest["sun_intensity"]
    if not isinstance(intensity, (int, float)) or isinstance(intensity, bool):
        errors.append(f"sun_intensity must be a number, got {intensity!r}")
    elif not math.isfinite(intensity) or intensity <= 0.0:
        errors.append(f"sun_intensity must be finite and positive, got {intensity!r}")

    sun_rgb = manifest["sun_rgb"]
    if not (isinstance(sun_rgb, list) and len(sun_rgb) == 3):
        errors.append(f"sun_rgb must be a three-element array, got {sun_rgb!r}")
    elif not all(
        isinstance(component, (int, float))
        and not isinstance(component, bool)
        and math.isfinite(component)
        and component >= 0.0
        for component in sun_rgb
    ):
        errors.append(f"sun_rgb must be three finite non-negative numbers, got {sun_rgb!r}")

    shadow = manifest["shadow_strength"]
    if not isinstance(shadow, (int, float)) or isinstance(shadow, bool):
        errors.append(f"shadow_strength must be a number, got {shadow!r}")
    elif not math.isfinite(shadow) or not 0.0 <= shadow <= 1.0:
        errors.append(f"shadow_strength must lie in [0, 1], got {shadow!r}")

    return errors


# ---------------------------------------------------------------------------
# Provenance contract
# ---------------------------------------------------------------------------

REQUIRED_PROVENANCE_FIELDS: tuple[str, ...] = (
    "schema_version",
    "slice",
    "asset_id",
    "provider",
    "slug",
    "name",
    "authors",
    "license",
    "license_url",
    "attribution",
    "source_page",
    "acquisition",
    "processing",
    "depth_proxy",
    "runtime_manifest",
    "calibration",
    "not_applied",
)

REQUIRED_ACQUISITION_FIELDS: tuple[str, ...] = (
    "acquired_utc",
    "acquisition_date",
    "user_agent",
    "info_url",
    "files_url",
    "info_payload_sha256",
    "files_payload_sha256",
    "files_hash",
    "source_dimensions",
    "acquired_dimensions",
    "source_files",
)

REQUIRED_SOURCE_FILE_RECORD_FIELDS: tuple[str, ...] = (
    "role",
    "file_name",
    "url",
    "api_size",
    "api_md5",
    "local_size",
    "local_md5",
    "local_sha256",
    "size_verified",
    "md5_verified",
)

REQUIRED_PROCESSING_FIELDS: tuple[str, ...] = (
    "command",
    "recipe",
    "tonemap",
    "colour_space",
    "exposure_scale",
    "exposure_ev",
    "jpeg",
    "runtime_derivative",
)

REQUIRED_CALIBRATION_FIELDS: tuple[str, ...] = (
    "equirect_convention",
    "pilot_eye",
    "sun",
    "radiance",
    "exposure_calibration",
    "brick_garage",
    "manually_estimated_obstacles",
    "depth_proxy_geometry",
    "physically_derived",
    "manually_calibrated",
)


def validate_provenance(provenance: Any) -> list[str]:
    """Human-readable contract violations of a provenance record (empty == valid)."""
    errors: list[str] = []
    if not isinstance(provenance, dict):
        return ["the provenance record must be a JSON object"]

    keys = set(provenance)
    for missing in sorted(set(REQUIRED_PROVENANCE_FIELDS) - keys):
        errors.append(f"{missing}: required field missing")
    if errors:
        return errors

    if provenance["schema_version"] != PROVENANCE_SCHEMA_VERSION:
        errors.append(
            f"schema_version must be {PROVENANCE_SCHEMA_VERSION}, got "
            f"{provenance['schema_version']!r}"
        )
    if provenance["license"] != LICENSE:
        errors.append(f"license must be {LICENSE!r}; PF1 accepts CC0 sources only")
    if provenance["not_applied"] != list(NOT_APPLIED):
        errors.append(f"not_applied must be exactly {list(NOT_APPLIED)}")

    acquisition = provenance["acquisition"]
    if not isinstance(acquisition, dict):
        errors.append("acquisition must be an object")
    else:
        for field in REQUIRED_ACQUISITION_FIELDS:
            if field not in acquisition:
                errors.append(f"acquisition.{field}: required field missing")
        for field in ("info_payload_sha256", "files_payload_sha256"):
            value = acquisition.get(field)
            if not (isinstance(value, str) and len(value) == 64):
                errors.append(
                    f"acquisition.{field} must be a 64-character hex digest, got {value!r}"
                )
        records = acquisition.get("source_files")
        if not isinstance(records, list) or len(records) != len(SOURCE_FILES):
            errors.append(
                f"acquisition.source_files must list {len(SOURCE_FILES)} entries"
            )
        else:
            for index, record in enumerate(records):
                where = f"acquisition.source_files[{index}]"
                if not isinstance(record, dict):
                    errors.append(f"{where} must be an object")
                    continue
                for field in REQUIRED_SOURCE_FILE_RECORD_FIELDS:
                    if field not in record:
                        errors.append(f"{where}.{field}: required field missing")
                for flag in ("size_verified", "md5_verified"):
                    if record.get(flag) is not True:
                        errors.append(
                            f"{where}.{flag}: must be true; an unverified source is a "
                            "fail-closed condition"
                        )

    processing = provenance["processing"]
    if not isinstance(processing, dict):
        errors.append("processing must be an object")
    else:
        for field in REQUIRED_PROCESSING_FIELDS:
            if field not in processing:
                errors.append(f"processing.{field}: required field missing")
        derivative = processing.get("runtime_derivative")
        if isinstance(derivative, dict):
            for field in ("path", "file_name", "format", "sha256", "byte_size", "dimensions"):
                if field not in derivative:
                    errors.append(f"processing.runtime_derivative.{field}: missing")
            if derivative.get("dimensions") != ACQUIRED_DIMENSIONS:
                errors.append(
                    f"processing.runtime_derivative.dimensions must be "
                    f"{ACQUIRED_DIMENSIONS}, got {derivative.get('dimensions')!r}"
                )
        if processing.get("exposure_scale") != EXPOSURE_SCALE:
            errors.append(
                f"processing.exposure_scale must be {EXPOSURE_SCALE} (the renderer's "
                f"exposure_ev = {EXPOSURE_EV})"
            )

    proxy = provenance["depth_proxy"]
    if not isinstance(proxy, dict):
        errors.append("depth_proxy must be an object")
    else:
        for field in ("path", "file_name", "sha256", "byte_size", "triangle_count", "node_names"):
            if field not in proxy:
                errors.append(f"depth_proxy.{field}: required field missing")
        if proxy.get("triangle_count") != EXPECTED_PROXY_TRIANGLES:
            errors.append(
                f"depth_proxy.triangle_count must be {EXPECTED_PROXY_TRIANGLES}, got "
                f"{proxy.get('triangle_count')!r}"
            )
        if list(proxy.get("node_names") or []) != list(PROXY_NODE_NAMES):
            errors.append(
                f"depth_proxy.node_names must be exactly {list(PROXY_NODE_NAMES)}"
            )

    calibration = provenance["calibration"]
    if not isinstance(calibration, dict):
        errors.append("calibration must be an object")
    else:
        for field in REQUIRED_CALIBRATION_FIELDS:
            if field not in calibration:
                errors.append(f"calibration.{field}: required field missing")
        sun = calibration.get("sun")
        if isinstance(sun, dict):
            if sun.get("longitude_deg") != SUN_LONGITUDE_DEG:
                errors.append("calibration.sun.longitude_deg does not match the measurement")
            if sun.get("elevation_deg") != SUN_ELEVATION_DEG:
                errors.append("calibration.sun.elevation_deg does not match the measurement")
            if list(sun.get("direction_render") or []) != list(SUN_DIRECTION_RENDER):
                errors.append(
                    "calibration.sun.direction_render must equal the manifest's "
                    "sun_direction_render"
                )
            if not sun.get("convention"):
                errors.append("calibration.sun.convention must state the mapping used")

    return errors


def processing_command(root: Optional[pathlib.Path] = None) -> list[str]:
    """The canonical, machine-independent argv that derives the panorama.

    Recorded in provenance verbatim. Relative paths keep the record stable
    across worktrees; ``sys.argv`` would embed a developer's absolute path.
    """
    del root  # the command is workspace-relative by design
    return [
        "python",
        "-X",
        "utf8",
        "tools/photo_field_pipeline/process_photo_field_panorama.py",
    ]


def authoring_command() -> list[str]:
    """The canonical argv that authors the depth-proxy GLB."""
    return [
        "python",
        "-X",
        "utf8",
        "tools/photo_field_pipeline/author_photo_field_proxies.py",
    ]


def manifest_command() -> list[str]:
    """The canonical argv that builds both manifests."""
    return [
        "python",
        "-X",
        "utf8",
        "tools/photo_field_pipeline/build_photo_field_manifest.py",
    ]


def fetch_command(offline: bool = False) -> list[str]:
    """The canonical argv that acquires and verifies the Poly Haven sources."""
    command = [
        "python",
        "-X",
        "utf8",
        "tools/photo_field_pipeline/fetch_photo_field_sources.py",
    ]
    if offline:
        command.append("--offline")
    return command
