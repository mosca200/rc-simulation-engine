#!/usr/bin/env python3
"""
RV2-VIS0-C2B RuntimeCaptureReceipt reader and independent PNG verification

The runtime capture backend (VIS0-C2A) writes two artifacts: a lossless PNG of
the presented frame and, when `--capture-receipt-out` is given, a
`RuntimeCaptureReceipt` describing what it really produced. This module is the
tooling side of that handshake.

    RuntimeCaptureReceipt  -> a narrow runtime declaration  (schema 1.0.0)
    VisualCaptureEvidence  -> the tooling fact layer        (schema 1.0.0)

They are different contracts with different owners and are never conflated: the
receipt is written by `rcsim-app`, the evidence is written by the benchmark
runner. The receipt is the ONLY runtime authority for `capture.actual.*` and
`capture.image.*`; the runner never derives those values from the manifest.

Trust model
-----------
A receipt is not trusted because it exists. It is trusted only after:

1. it parses as strict `RuntimeCaptureReceipt` 1.0.0 - exact field set, no
   unknown key, no missing key, no placeholder value;
2. it agrees with the request plan on `presentation_frame_index`, `format` and
   `image_path` (compared in canonical form);
3. the PNG it points at is re-verified independently: real byte size, real
   SHA-256, valid signature, readable IHDR, IHDR extent equal to the receipt
   extent, bit depth 8 and colour type 6 (truecolour with alpha).

Everything here fails closed and reports concrete errors instead of raising, so
the runner can turn a rejection into honest failure evidence. Only the Python
standard library is used: reading a PNG header needs 33 bytes and `int.from_bytes`,
not an image library, and this module deliberately performs no pixel analysis.
"""

import hashlib
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

try:  # Imported as part of the tools.visual_benchmark namespace package.
    from tools.visual_benchmark.validate_manifest import is_real_integer
except ImportError:  # Direct script execution: script directory is sys.path[0].
    from validate_manifest import is_real_integer


RECEIPT_KIND = "runtime_capture_receipt"
RECEIPT_SCHEMA_VERSION = "1.0.0"
RECEIPT_SUPPORTED_FORMAT = "png"

# Declaration order matches the Rust struct in crates/app/src/render_app.rs, so
# a serialized receipt reads the same in both directions.
RECEIPT_FIELD_ORDER = (
    "schema_version",
    "presentation_frame_index",
    "framebuffer_width",
    "framebuffer_height",
    "format",
    "image_path",
    "image_sha256",
    "image_byte_size",
)
RECEIPT_REQUIRED_FIELDS = frozenset(RECEIPT_FIELD_ORDER)

SHA256_PATTERN = r"^[0-9a-f]{64}$"
SHA256_REGEX = re.compile(SHA256_PATTERN)

# --- PNG container facts (RFC 2083) ------------------------------------------

PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
PNG_IHDR_TYPE = b"IHDR"
PNG_IHDR_DATA_LENGTH = 13
# signature (8) + length (4) + type (4) + IHDR data (13) + CRC (4)
PNG_MINIMUM_LENGTH = 33
PNG_REQUIRED_BIT_DEPTH = 8
PNG_COLOR_TYPE_RGBA = 6

UNAVAILABLE_POLICY = (
    "null is the only marker for 'not available'; 0, -1 and empty strings are "
    "rejected because they read as real measurements"
)


@dataclass(frozen=True)
class RuntimeCaptureReceipt:
    """One parsed, structurally valid RuntimeCaptureReceipt 1.0.0."""

    schema_version: str
    presentation_frame_index: int
    framebuffer_width: int
    framebuffer_height: int
    format: str
    image_path: str
    image_sha256: str
    image_byte_size: int

    def to_json(self) -> dict:
        return {field: getattr(self, field) for field in RECEIPT_FIELD_ORDER}


@dataclass(frozen=True)
class VerifiedImage:
    """A PNG whose bytes and header were checked against a trusted receipt."""

    path: str
    byte_size: int
    sha256: str
    width: int
    height: int
    bit_depth: int
    color_type: int

    def to_json(self) -> dict:
        return {
            "path": self.path,
            "byte_size": self.byte_size,
            "sha256": self.sha256,
            "width": self.width,
            "height": self.height,
            "bit_depth": self.bit_depth,
            "color_type": self.color_type,
        }


def sha256_of_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def canonical_image_path(value: Any, base: Optional[Path] = None) -> str:
    """Canonicalise an image path so two spellings of one file compare equal.

    The runtime echoes back exactly the string it was given on `--capture-out`,
    so a runner that passes an absolute path gets an absolute path back. Case
    and separator spelling still differ across platforms, and a relative receipt
    path is only meaningful against the subprocess working directory, which the
    runner knows and passes as `base`.
    """
    if not isinstance(value, str):
        return ""
    path = Path(value)
    if not path.is_absolute() and base is not None:
        path = Path(base) / path
    return os.path.normcase(os.path.normpath(str(path)))


def _require_integer(errors: list, path: str, value: Any, minimum: int) -> Optional[int]:
    """Validate one receipt integer leaf. Returns the value or None."""
    if not is_real_integer(value):
        got = type(value).__name__ if value is not None else "null"
        errors.append(f"{path}: must be an integer, got {got}")
        return None
    if value < minimum:
        errors.append(f"{path}: must be >= {minimum}, got {value} ({UNAVAILABLE_POLICY})")
        return None
    return value


def parse_receipt(payload: Any) -> tuple:
    """Parse a RuntimeCaptureReceipt 1.0.0 payload.

    Returns `(receipt, errors)`. On any rejection `receipt` is None and
    `errors` names every problem found, so a producer bug is diagnosable from
    the runner output alone. Nothing is coerced and nothing is defaulted.
    """
    errors: list = []
    if not isinstance(payload, dict):
        got = type(payload).__name__ if payload is not None else "null"
        return None, [f"receipt root must be an object, got {got}"]

    unknown = sorted(set(payload) - RECEIPT_REQUIRED_FIELDS)
    if unknown:
        errors.append(
            f"receipt: unknown field(s) {unknown}; RuntimeCaptureReceipt "
            f"{RECEIPT_SCHEMA_VERSION} declares exactly "
            f"{list(RECEIPT_FIELD_ORDER)}"
        )
    missing = sorted(RECEIPT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"receipt: missing required field(s) {missing}")

    schema_version = payload.get("schema_version")
    if not isinstance(schema_version, str):
        got = type(schema_version).__name__ if schema_version is not None else "null"
        errors.append(f"schema_version: must be a string, got {got}")
    elif schema_version != RECEIPT_SCHEMA_VERSION:
        errors.append(
            f"schema_version: unsupported runtime receipt version "
            f"'{schema_version}'; this tooling reads {RECEIPT_SCHEMA_VERSION}"
        )

    frame = _require_integer(errors, "presentation_frame_index",
                             payload.get("presentation_frame_index"), minimum=0)
    width = _require_integer(errors, "framebuffer_width",
                             payload.get("framebuffer_width"), minimum=1)
    height = _require_integer(errors, "framebuffer_height",
                              payload.get("framebuffer_height"), minimum=1)
    byte_size = _require_integer(errors, "image_byte_size",
                                 payload.get("image_byte_size"), minimum=1)

    image_format = payload.get("format")
    if not isinstance(image_format, str):
        got = type(image_format).__name__ if image_format is not None else "null"
        errors.append(f"format: must be a string, got {got}")
    elif image_format != RECEIPT_SUPPORTED_FORMAT:
        errors.append(
            f"format: must be '{RECEIPT_SUPPORTED_FORMAT}', got '{image_format}'; "
            "the VIS0-C2A runtime capture backend writes lossless PNG only"
        )

    image_path = payload.get("image_path")
    if not isinstance(image_path, str):
        got = type(image_path).__name__ if image_path is not None else "null"
        errors.append(f"image_path: must be a string, got {got}")
    elif not image_path.strip():
        errors.append(f"image_path: must be a non-empty path ({UNAVAILABLE_POLICY})")

    image_sha256 = payload.get("image_sha256")
    if not isinstance(image_sha256, str):
        got = type(image_sha256).__name__ if image_sha256 is not None else "null"
        errors.append(f"image_sha256: must be a string, got {got}")
    elif not SHA256_REGEX.match(image_sha256):
        errors.append(
            f"image_sha256: must be 64 lowercase hexadecimal characters, got "
            f"'{image_sha256}'"
        )

    if errors:
        return None, errors
    return RuntimeCaptureReceipt(
        schema_version=schema_version,
        presentation_frame_index=frame,
        framebuffer_width=width,
        framebuffer_height=height,
        format=image_format,
        image_path=image_path,
        image_sha256=image_sha256,
        image_byte_size=byte_size,
    ), []


def load_receipt(path: Path) -> tuple:
    """Read and parse a receipt file. Returns `(receipt, errors)`."""
    if not path.exists():
        return None, [f"runtime capture receipt not found: {path}"]
    if not path.is_file():
        return None, [f"runtime capture receipt is not a regular file: {path}"]
    try:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except json.JSONDecodeError as error:
        return None, [f"runtime capture receipt is not valid JSON: {error}"]
    except OSError as error:
        return None, [f"runtime capture receipt could not be read: {error}"]
    return parse_receipt(payload)


def check_receipt_expectations(
    receipt: RuntimeCaptureReceipt,
    expected_frame_index: Any,
    expected_format: Any,
    expected_image_path: Any,
    base: Optional[Path] = None,
) -> list:
    """Compare a parsed receipt against the request plan.

    Only the three fields the runner actually requested are compared:
    `presentation_frame_index`, `format` and `image_path`. The framebuffer
    extent is deliberately NOT compared against the manifest resolution - the
    receipt is the authority for what was really presented, and a divergence
    between requested and actual is a fact to record, not an error to repair.

    Returns a list of mismatch descriptions; empty means the receipt matches.
    """
    errors: list = []
    if not is_real_integer(expected_frame_index):
        errors.append(
            "request plan has no explicit capture frame index to check the "
            "receipt against"
        )
    elif receipt.presentation_frame_index != expected_frame_index:
        errors.append(
            f"presentation_frame_index: receipt declares "
            f"{receipt.presentation_frame_index} but the request asked for "
            f"{expected_frame_index}"
        )

    if not isinstance(expected_format, str):
        errors.append("request plan has no capture format to check the receipt against")
    elif receipt.format != expected_format:
        errors.append(
            f"format: receipt declares '{receipt.format}' but the request asked "
            f"for '{expected_format}'"
        )

    if not isinstance(expected_image_path, str) or not expected_image_path.strip():
        errors.append("request plan has no capture image path to check the receipt against")
    else:
        declared = canonical_image_path(receipt.image_path, base)
        requested = canonical_image_path(expected_image_path, base)
        if declared != requested:
            errors.append(
                f"image_path: receipt declares '{receipt.image_path}' but the "
                f"request planned '{expected_image_path}'"
            )
    return errors


def parse_png_header(payload: bytes) -> tuple:
    """Read the IHDR fields of a PNG byte string.

    Returns `(header, errors)` where header is a dict of the raw IHDR values.
    No image library is involved: the signature is 8 fixed bytes and IHDR is
    the mandatory first chunk, so 33 bytes are enough to establish that a file
    really is an RGBA8 PNG of a given extent.
    """
    if len(payload) < PNG_MINIMUM_LENGTH:
        return None, [
            f"file is {len(payload)} byte(s), too short to hold a PNG signature "
            f"and IHDR chunk (minimum {PNG_MINIMUM_LENGTH})"
        ]
    errors: list = []
    if payload[:8] != PNG_SIGNATURE:
        errors.append(
            "invalid PNG signature: expected 89 50 4e 47 0d 0a 1a 0a, got "
            + " ".join(f"{byte:02x}" for byte in payload[:8])
        )
    length_bytes = payload[8:12]
    chunk_type = payload[12:16]
    declared_length = int.from_bytes(length_bytes, "big")
    if chunk_type != PNG_IHDR_TYPE:
        errors.append(
            f"first chunk is '{chunk_type.decode('latin-1')}', not 'IHDR'; a PNG "
            "must start with the image header chunk"
        )
    elif declared_length != PNG_IHDR_DATA_LENGTH:
        errors.append(
            f"IHDR declares a data length of {declared_length}, expected "
            f"{PNG_IHDR_DATA_LENGTH}; the header is not readable"
        )
    if errors:
        return None, errors

    header = {
        "width": int.from_bytes(payload[16:20], "big"),
        "height": int.from_bytes(payload[20:24], "big"),
        "bit_depth": payload[24],
        "color_type": payload[25],
        "compression": payload[26],
        "filter": payload[27],
        "interlace": payload[28],
    }
    if header["bit_depth"] != PNG_REQUIRED_BIT_DEPTH:
        errors.append(
            f"bit depth is {header['bit_depth']}, expected "
            f"{PNG_REQUIRED_BIT_DEPTH}: the runtime captures display-referred RGBA8"
        )
    if header["color_type"] != PNG_COLOR_TYPE_RGBA:
        errors.append(
            f"colour type is {header['color_type']}, expected "
            f"{PNG_COLOR_TYPE_RGBA} (truecolour with alpha): the runtime "
            "captures RGBA8"
        )
    if errors:
        return None, errors
    return header, []


def verify_captured_png(path: Path, receipt: RuntimeCaptureReceipt) -> tuple:
    """Independently verify the PNG a receipt points at.

    The receipt is a claim; this function re-measures the file. Returns
    `(image, errors)` where `image` is a VerifiedImage built from the observed
    bytes and header, or None when any check fails. Errors are accumulated so a
    single run reports every divergence rather than only the first.
    """
    if not path.exists():
        return None, [f"capture image not found: {path}"]
    if not path.is_file():
        return None, [f"capture image is not a regular file: {path}"]
    try:
        with open(path, "rb") as handle:
            payload = handle.read()
    except OSError as error:
        return None, [f"capture image could not be read: {error}"]

    errors: list = []
    byte_size = len(payload)
    if byte_size != receipt.image_byte_size:
        errors.append(
            f"image byte size: file is {byte_size} byte(s) but the receipt "
            f"declares {receipt.image_byte_size}"
        )
    digest = sha256_of_bytes(payload)
    if digest != receipt.image_sha256:
        errors.append(
            f"image sha256: file hashes to {digest} but the receipt declares "
            f"{receipt.image_sha256}"
        )

    header, header_errors = parse_png_header(payload)
    errors.extend(header_errors)
    if header is not None:
        if header["width"] != receipt.framebuffer_width:
            errors.append(
                f"image width: PNG IHDR says {header['width']} but the receipt "
                f"declares framebuffer_width {receipt.framebuffer_width}"
            )
        if header["height"] != receipt.framebuffer_height:
            errors.append(
                f"image height: PNG IHDR says {header['height']} but the receipt "
                f"declares framebuffer_height {receipt.framebuffer_height}"
            )

    if errors:
        return None, errors
    return VerifiedImage(
        path=str(path),
        byte_size=byte_size,
        sha256=digest,
        width=header["width"],
        height=header["height"],
        bit_depth=header["bit_depth"],
        color_type=header["color_type"],
    ), []
