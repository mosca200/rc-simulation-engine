"""PF1: derive the committed display-referred panorama from the HDR source.

The runtime can only decode PNG/JPEG (the ``image`` crate is built with ``png``
and ``jpeg``), and it tonemaps the 3D aircraft with the Khronos PBR Neutral curve
at ``exposure_ev = 0.0`` while the photographic background BYPASSES the
tonemapper. The background must therefore be tonemapped offline with the very
same curve at the very same exposure, or the aircraft and the photograph would be
two different renderings of two different worlds.

The whole recipe is::

    radiance       = decode_rgbe(meadow_8k.hdr)              # scene-referred
    display_linear = khronos_pbr_neutral(radiance * 1.0)      # shader.wgsl
    srgb_bytes     = round(linear_to_srgb(display_linear) * 255)  # texture.rs
    -> JPEG, quality 92, subsampling 0, NO resize, NO other processing

Nothing else happens: no resampling, no colour LUT, no saturation or contrast
boost, no sharpening, no cropping. Decoding runs band-wise so an 8192x4096 source
never materialises a full float64 copy, and the quantisation rule is a single
documented round-half-to-even.

The source is verified against the size and MD5 the Poly Haven API published
before a single scanline is decoded, so a corrupted cache can never become a
committed asset.

Byte reproducibility: ``--check`` re-derives the panorama and requires
byte-identity with the committed file. The comparison happens in memory rather
than through a temporary file so a failed check can never leave a partial asset
behind; ``--out`` writes a real second copy when a two-run digest comparison is
wanted instead.

Usage:
    python -X utf8 tools/photo_field_pipeline/process_photo_field_panorama.py
    python -X utf8 tools/photo_field_pipeline/process_photo_field_panorama.py --check
"""

from __future__ import annotations

import argparse
import io
import pathlib
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from photo_field_assets import (  # noqa: E402
    ACQUIRED_DIMENSIONS,
    EXPOSURE_EV,
    EXPOSURE_SCALE,
    JPEG_FORMAT,
    JPEG_QUALITY,
    JPEG_SUBSAMPLING,
    PhotoFieldError,
    configure_streams,
    display_referred_u8,
    file_facts,
    iter_rgbe_bands,
    khronos_pbr_neutral,
    panorama_path,
    require_file,
    sha256_bytes,
    source_file_for_role,
    source_path,
)

SOURCE_ROLE = "source_hdr"
BAND_ROWS = 256


def verify_source(path: pathlib.Path) -> dict[str, object]:
    """Verify the HDR source against its pinned API size and MD5, failing closed."""
    descriptor = source_file_for_role(SOURCE_ROLE)
    require_file(
        path,
        "run tools/photo_field_pipeline/fetch_photo_field_sources.py first: the "
        "panorama is derived from the Poly Haven source, never from a substitute",
    )
    facts = file_facts(path)
    if facts["byte_size"] != descriptor.api_size:
        raise PhotoFieldError(
            f"{path.name} is {facts['byte_size']} bytes but the Poly Haven API "
            f"publishes {descriptor.api_size}: the cached source is not the asset PF1 "
            "was calibrated against"
        )
    if facts["md5"] != descriptor.api_md5:
        raise PhotoFieldError(
            f"{path.name} MD5 {facts['md5']} != the API's {descriptor.api_md5}"
        )
    if descriptor.pinned_sha256 is not None and facts["sha256"] != descriptor.pinned_sha256:
        raise PhotoFieldError(
            f"{path.name} SHA-256 {facts['sha256']} != the pinned {descriptor.pinned_sha256}"
        )
    print(f"source verified: {path.name}")
    print(f"  byte_size {facts['byte_size']}")
    print(f"  md5       {facts['md5']} (API)")
    print(f"  sha256    {facts['sha256']}")
    return facts


def derive_panorama_bytes(source: pathlib.Path) -> tuple[bytes, dict[str, object]]:
    """Decode, tonemap, encode. Returns the JPEG payload and measured statistics."""
    width = height = None
    pixels: np.ndarray | None = None
    bands = 0
    radiance_min = float("inf")
    radiance_max = float("-inf")

    for row_start, band, meta in iter_rgbe_bands(source, band_rows=BAND_ROWS):
        if width is None:
            width, height = int(meta["width"]), int(meta["height"])
            if [width, height] != ACQUIRED_DIMENSIONS:
                raise PhotoFieldError(
                    f"{source.name} is {width}x{height} but PF1 commits a "
                    f"{ACQUIRED_DIMENSIONS[0]}x{ACQUIRED_DIMENSIONS[1]} derivative "
                    f"(and the file name says so); refusing to resize"
                )
            pixels = np.zeros((height, width, 3), dtype=np.uint8)
            print(f"decoding {width}x{height} {meta['format']} in {BAND_ROWS}-row bands")
        assert pixels is not None  # for type checkers; the branch above always runs first
        radiance_min = min(radiance_min, float(band.min()))
        radiance_max = max(radiance_max, float(band.max()))

        # The renderer's chain, at the renderer's exposure: exp2(0.0) == 1.0.
        exposed = band.astype(np.float64) * EXPOSURE_SCALE
        display_linear = khronos_pbr_neutral(exposed)
        pixels[row_start : row_start + band.shape[0], :, :] = display_referred_u8(display_linear)

        bands += 1
        print(f"  row {row_start:5d} / {height} processed", end="\r", flush=True)

    if pixels is None or width is None or height is None:
        raise PhotoFieldError(f"{source}: decoded no scanline bands")
    print(" " * 40, end="\r", flush=True)
    if bands * BAND_ROWS < height:
        raise PhotoFieldError(
            f"decoded {bands} bands covering {bands * BAND_ROWS} rows of {height}"
        )

    image = Image.fromarray(pixels, mode="RGB")
    if list(image.size) != ACQUIRED_DIMENSIONS:
        raise PhotoFieldError(
            f"Pillow reports {image.size} for a {tuple(ACQUIRED_DIMENSIONS)} array"
        )
    buffer = io.BytesIO()
    image.save(
        buffer,
        format=JPEG_FORMAT.upper(),
        quality=JPEG_QUALITY,
        subsampling=JPEG_SUBSAMPLING,
        optimize=False,
        progressive=False,
    )
    payload = buffer.getvalue()

    statistics = {
        "bands": bands,
        "radiance_min": radiance_min,
        "radiance_max": radiance_max,
        "srgb_mean": float(pixels.mean()) / 255.0,
        "srgb_min": int(pixels.min()),
        "srgb_max": int(pixels.max()),
    }
    return payload, statistics


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="process_photo_field_panorama.py",
        description=(
            "Derive the committed display-referred sRGB JPEG panorama from the "
            "Poly Haven HDR source using the renderer's own tone mapper."
        ),
    )
    parser.add_argument(
        "--source",
        metavar="PATH",
        default=None,
        help=f"Radiance HDR source (default {source_path(SOURCE_ROLE)})",
    )
    parser.add_argument(
        "--out",
        metavar="PATH",
        default=None,
        help=f"destination JPEG (default {panorama_path()})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help=(
            "re-derive the panorama and require byte-identity with the committed "
            "file instead of writing it"
        ),
    )
    args = parser.parse_args(argv)

    try:
        source = (
            pathlib.Path(args.source).resolve() if args.source else source_path(SOURCE_ROLE)
        )
        destination = (
            pathlib.Path(args.out).resolve() if args.out else panorama_path()
        )
        # Fail before the expensive part: a --check with nothing to check against
        # must not spend two minutes re-deriving 33.5 megapixels first.
        if args.check and not destination.is_file():
            raise PhotoFieldError(
                f"--check found no committed panorama at {destination}; run without "
                "--check first",
                exit_code=2,
            )
        print("PF1 panorama processing")
        print(f"  recipe: khronos_pbr_neutral(radiance * {EXPOSURE_SCALE}) -> sRGB -> JPEG")
        print(f"  exposure_ev {EXPOSURE_EV} (the renderer's pinned exposure)")
        print(f"  JPEG quality {JPEG_QUALITY}, subsampling {JPEG_SUBSAMPLING} (4:4:4)")
        verify_source(source)

        payload, statistics = derive_panorama_bytes(source)
        digest = sha256_bytes(payload)

        print("\nradiance in:  min {radiance_min:.6f}  max {radiance_max:.4f}".format(**statistics))
        print(
            "sRGB out:     mean {srgb_mean:.6f}  min {srgb_min}  max {srgb_max}".format(
                **statistics
            )
        )

        if args.check:
            committed = destination.read_bytes()
            committed_digest = sha256_bytes(committed)
            if committed != payload:
                print(
                    f"FAIL: re-processing is NOT byte-identical to {destination.name}\n"
                    f"  committed  {len(committed)} bytes sha256 {committed_digest}\n"
                    f"  re-derived {len(payload)} bytes sha256 {digest}",
                    file=sys.stderr,
                )
                return 1
            print(f"\n--check: {destination.name} is byte-identical ({len(payload)} bytes)")
            print(f"sha256 {digest}")
            return 0

        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        print(f"\nwrote {destination}")
        print(f"  byte_size  {len(payload)}")
        print(f"  sha256     {digest}")
        print(f"  dimensions {ACQUIRED_DIMENSIONS[0]}x{ACQUIRED_DIMENSIONS[1]}")
        print(f"  format     {JPEG_FORMAT} (quality {JPEG_QUALITY}, subsampling {JPEG_SUBSAMPLING})")
        return 0
    except PhotoFieldError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code


if __name__ == "__main__":
    sys.exit(main())
