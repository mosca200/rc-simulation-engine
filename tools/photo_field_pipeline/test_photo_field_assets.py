"""Tests for the PF1 photo-field asset pipeline.

Run BY PATH from the workspace root (never by discovery):

    python -X utf8 -m unittest tools/photo_field_pipeline/test_photo_field_assets.py -v

Standard library + numpy (Pillow only for the panorama reprocessing check).

Three groups of tests, by what they need:

* pure tests (always run): the RGBE decoder against hand-built Radiance files,
  the tone-mapper port against a scalar transcription of the WGSL, the sRGB
  transfer functions, the equirect mapping (anchored to values the Rust runtime
  itself printed), the GLB writer, and the proxy authoring contract;
* committed-asset tests (skip when the pipeline has not been run yet): the
  runtime manifest, the panorama header and digest, the depth proxy's node names
  and triangle count, and the provenance record;
* source-cache tests (skip when the gitignored Poly Haven cache is absent): the
  1k decode cross-check against FFV1's independent measurement, and the
  byte-reproducibility ``--check`` of the 8k processing.
"""

from __future__ import annotations

import io
import json
import math
import pathlib
import struct
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import author_photo_field_proxies as authoring  # noqa: E402
import build_photo_field_manifest as builder  # noqa: E402
import photo_field_assets as pfa  # noqa: E402

try:  # Pillow is a documented PF1 dependency; skip its tests when it is absent.
    import process_photo_field_panorama as processing
except ImportError:  # pragma: no cover - environment dependent
    processing = None  # type: ignore[assignment]

PACKAGE_DIR = pathlib.Path(__file__).resolve().parent
REPO_ROOT = pfa.repo_root()
PROBE_HDR = pfa.source_path("decode_probe_hdr")
SOURCE_HDR_8K = pfa.source_path("source_hdr")
PANORAMA = pfa.panorama_path()
DEPTH_PROXY = pfa.depth_proxy_path()
RUNTIME_MANIFEST = pfa.runtime_manifest_path()
PROVENANCE = pfa.provenance_manifest_path()


# ---------------------------------------------------------------------------
# Helpers: a synthetic Radiance file, a synthetic JPEG, and the WGSL reference
# ---------------------------------------------------------------------------


def encode_literal(data: bytes) -> bytes:
    """New-style RLE literal runs (count <= 128) for one channel of one row."""
    out = b""
    for start in range(0, len(data), 128):
        chunk = data[start : start + 128]
        out += bytes([len(chunk)]) + chunk
    return out


def encode_run(value: int, count: int) -> bytes:
    """A new-style RLE repeat run: ``128 + count`` then the repeated byte."""
    if not 1 <= count <= 128:
        raise ValueError(f"an RLE run holds 1..128 bytes, got {count}")
    return bytes([128 + count, value])


def rgbe_scanline(width: int, encoded_channels: list[bytes]) -> bytes:
    """One new-style RLE scanline: the 2 2 marker, then four encoded channels."""
    if len(encoded_channels) != 4:
        raise ValueError("a scanline carries exactly four channels")
    out = struct.pack(">BBH", 2, 2, width)
    for channel in encoded_channels:
        out += channel
    return out


def literal_channels(*channels: bytes) -> list[bytes]:
    """Encode raw channel bytes as new-style RLE literal runs."""
    return [encode_literal(channel) for channel in channels]


def write_rgbe(
    path: pathlib.Path,
    width: int,
    height: int,
    scanlines: list[bytes],
    header: bytes = b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n",
    resolution: str | None = None,
) -> pathlib.Path:
    """Write a minimal Radiance RGBE file from pre-encoded scanlines."""
    line = resolution if resolution is not None else f"-Y {height} +X {width}"
    path.write_bytes(header + line.encode("ascii") + b"\n" + b"".join(scanlines))
    return path


def synthetic_jpeg(
    width: int,
    height: int,
    precision: int = 8,
    components: int = 3,
    extra_segments: bytes = b"",
) -> bytes:
    """The smallest byte string whose SOF0 carries the given frame header."""
    payload = struct.pack(">BHHB", precision, height, width, components)
    for index in range(components):
        payload += bytes([index + 1, 0x11, 0])
    sof0 = b"\xff\xc0" + struct.pack(">H", len(payload) + 2) + payload
    app0_payload = b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
    app0 = b"\xff\xe0" + struct.pack(">H", len(app0_payload) + 2) + app0_payload
    return b"\xff\xd8" + app0 + extra_segments + sof0 + b"\xff\xd9"


def wgsl_khronos_pbr_neutral(color: tuple[float, float, float]) -> tuple[float, float, float]:
    """Scalar transcription of ``fn khronos_pbr_neutral`` in shader.wgsl.

    Written statement-by-statement from the WGSL source (including the early
    return) so it is an independent check on the vectorised numpy port, not a
    copy of it. WGSL ``select(f, t, cond)`` yields ``t`` when ``cond`` is true,
    hence the parabolic offset for ``x < 0.08`` and the constant 0.04 otherwise.
    """
    start_compression = 0.8 - 0.04
    desaturation = 0.15

    red, green, blue = (float(value) for value in color)
    x = min(red, min(green, blue))
    offset = (x - 6.25 * x * x) if x < 0.08 else 0.04
    c = (red - offset, green - offset, blue - offset)

    peak = max(c[0], max(c[1], c[2]))
    if peak < start_compression:
        return c

    d = 1.0 - start_compression
    new_peak = 1.0 - d * d / (peak + d - start_compression)
    scale = new_peak / peak
    c = (c[0] * scale, c[1] * scale, c[2] * scale)
    g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0)
    # mix(c, vec3(new_peak), g)
    return tuple(channel * (1.0 - g) + new_peak * g for channel in c)


def port(color: tuple[float, float, float]) -> tuple[float, float, float]:
    """The library's vectorised port evaluated on one colour."""
    result = pfa.khronos_pbr_neutral(np.array([color], dtype=np.float64))
    return tuple(float(value) for value in result[0])


def box_extents(
    positions: list[tuple[float, float, float]], centre: tuple[float, float], azimuth_deg: float
) -> tuple[float, float, float]:
    """Measured (tangential width, radial depth, height) of an authored box."""
    radial = pfa.horizontal_direction(azimuth_deg)
    tangential = (-radial[2], 0.0, radial[0])
    half_t = max(
        abs((p[0] - centre[0]) * tangential[0] + (p[2] - centre[1]) * tangential[2])
        for p in positions
    )
    half_r = max(
        abs((p[0] - centre[0]) * radial[0] + (p[2] - centre[1]) * radial[2]) for p in positions
    )
    ys = [p[1] for p in positions]
    return 2.0 * half_t, 2.0 * half_r, max(ys) - min(ys)


def capture(callable_, *argv: str) -> tuple[int, str, str]:
    """Run a tool's main() with redirected streams; returns (code, out, err)."""
    out, err = io.StringIO(), io.StringIO()
    with redirect_stdout(out), redirect_stderr(err):
        code = callable_(list(argv))
    return code, out.getvalue(), err.getvalue()


# ---------------------------------------------------------------------------
# Radiance RGBE decoding (pure: hand-built files)
# ---------------------------------------------------------------------------


class TestRadianceDecodeSynthetic(unittest.TestCase):
    """The decoder is checked against files built byte by byte from rgbe.c."""

    def test_literal_and_run_channels_decode_to_exact_radiance(self) -> None:
        width, height = 4, 2
        # Row 0: exponent 132 == 128 + 8 - 4, so scale = 2 ** -4 = 0.0625 and the
        # mantissas 128/64/32/16 become exactly 8.0/4.0/2.0/1.0. The exponent
        # channel is a repeat run, the colour channels are literal runs.
        row0 = rgbe_scanline(
            width,
            literal_channels(
                bytes([128, 64, 32, 16]),
                bytes([128, 128, 128, 128]),
                bytes([0, 0, 0, 0]),
            )
            + [encode_run(132, width)],
        )
        # Row 1: exponent 0 encodes exactly zero regardless of the mantissas.
        row1 = rgbe_scanline(
            width,
            literal_channels(
                bytes([255, 1, 2, 3]),
                bytes([7, 7, 7, 7]),
                bytes([9, 9, 9, 9]),
            )
            + [encode_run(0, width)],
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = write_rgbe(pathlib.Path(temporary) / "tiny.hdr", width, height, [row0, row1])
            decoded, meta = pfa.decode_rgbe(path)

        self.assertEqual((meta["width"], meta["height"]), (width, height))
        self.assertEqual(meta["format"], pfa.RGBE_FORMAT)
        self.assertEqual(decoded.shape, (height, width, 3))
        self.assertEqual(decoded.dtype, np.float32)
        expected = np.array(
            [
                [[8.0, 8.0, 0.0], [4.0, 8.0, 0.0], [2.0, 8.0, 0.0], [1.0, 8.0, 0.0]],
                [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            ],
            dtype=np.float32,
        )
        self.assertTrue(np.array_equal(decoded, expected), decoded.tolist())

    def test_banded_decode_matches_the_whole_decode_exactly(self) -> None:
        width, height = 4, 5
        scanlines = [
            rgbe_scanline(
                width,
                literal_channels(
                    bytes([10 * (row + 1)] * width),
                    bytes([20 * (row + 1)] * width),
                    bytes([30 * (row + 1)] * width),
                )
                + [encode_run(130 + row, width)],
            )
            for row in range(height)
        ]
        with tempfile.TemporaryDirectory() as temporary:
            path = write_rgbe(pathlib.Path(temporary) / "bands.hdr", width, height, scanlines)
            whole, _ = pfa.decode_rgbe(path)
            banded = np.concatenate(
                [band for _, band, _ in pfa.iter_rgbe_bands(path, band_rows=2)], axis=0
            )
        self.assertEqual(whole.shape, banded.shape)
        self.assertTrue(np.array_equal(whole, banded))

    def test_header_parsing_rejects_anything_but_the_supported_variant(self) -> None:
        cases = {
            "bad magic": b"#?NOTRADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2 +X 2\n",
            "no blank line": b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n-Y 2 +X 2\n",
            "wrong format": b"#?RADIANCE\nFORMAT=32-bit_rle_xyz\n\n-Y 2 +X 2\n",
            "bottom-up order": b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n+Y 2 +X 2\n",
            "bad resolution": b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2\n",
            "zero width": b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2 +X 0\n",
        }
        for name, buffer in cases.items():
            with self.subTest(name), self.assertRaises(pfa.PhotoFieldError):
                pfa.parse_rgbe_header(buffer)

    def test_scanline_framing_is_enforced(self) -> None:
        width, height = 4, 1
        good = rgbe_scanline(
            width,
            literal_channels(
                bytes([1] * width), bytes([2] * width), bytes([3] * width)
            )
            + [encode_run(136, width)],
        )
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)

            # An old-style (or uncompressed) scanline does not start with 2 2.
            old_style = write_rgbe(
                root / "old.hdr",
                width,
                height,
                [b"\x01\x02\x00\x04" + good[4:]],
            )
            with self.assertRaises(pfa.PhotoFieldError) as caught:
                list(pfa.iter_rgbe_bands(old_style))
            self.assertIn("new-style RLE", str(caught.exception.message))

            # A marker that declares the wrong width is rejected, not trusted.
            wrong_width = write_rgbe(
                root / "width.hdr",
                width,
                height,
                [struct.pack(">BBH", 2, 2, width + 1) + good[4:]],
            )
            with self.assertRaises(pfa.PhotoFieldError) as caught:
                list(pfa.iter_rgbe_bands(wrong_width))
            self.assertIn("declares width", str(caught.exception.message))

            # Truncated pixel data fails closed instead of decoding a short row.
            truncated = write_rgbe(root / "short.hdr", width, height, [good[:6]])
            with self.assertRaises(pfa.PhotoFieldError):
                list(pfa.iter_rgbe_bands(truncated))

    def test_channel_decoder_rejects_a_zero_length_literal_run(self) -> None:
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.decode_rgbe_channel(bytes([0, 1, 2, 3]), 0, 4)

    def test_channel_decoder_rejects_truncated_data(self) -> None:
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.decode_rgbe_channel(bytes([4, 1, 2]), 0, 4)


class TestRadianceDecodeOfTheProbeSource(unittest.TestCase):
    """Cross-check against the committed FFV1 measurement of this exact asset."""

    @classmethod
    def setUpClass(cls) -> None:
        if not PROBE_HDR.is_file():
            raise unittest.SkipTest(
                f"the gitignored Poly Haven source cache has no {PROBE_HDR.name}; run "
                "tools/photo_field_pipeline/fetch_photo_field_sources.py"
            )
        cls.rgb, cls.meta = pfa.decode_rgbe(PROBE_HDR)

    def test_dimensions_match_the_1k_resolution(self) -> None:
        self.assertEqual((self.meta["width"], self.meta["height"]), tuple(pfa.PROBE_DIMENSIONS))
        self.assertEqual(self.rgb.shape, (pfa.PROBE_DIMENSIONS[1], pfa.PROBE_DIMENSIONS[0], 3))

    def test_radiance_is_finite_and_non_negative(self) -> None:
        self.assertTrue(bool(np.isfinite(self.rgb).all()))
        self.assertGreaterEqual(float(self.rgb.min()), 0.0)

    def test_sky_mean_reproduces_the_ffv1_look_dev_measurement(self) -> None:
        """FFV1's independent offline analysis recorded a sky mean of 1.26."""
        lum = pfa.luminance(self.rgb)
        height = lum.shape[0]
        sky_mean = float(lum[: height // 2, :].mean())
        low, high = pfa.SKY_MEAN_CROSS_CHECK_BAND
        self.assertGreaterEqual(sky_mean, low, f"sky mean {sky_mean} below the cross-check band")
        self.assertLessEqual(sky_mean, high, f"sky mean {sky_mean} above the cross-check band")

    def test_the_photographed_ground_is_much_darker_than_the_sky(self) -> None:
        lum = pfa.luminance(self.rgb)
        height = lum.shape[0]
        sky_mean = float(lum[: height // 2, :].mean())
        ground_mean = float(lum[int(height * 0.55) :, :].mean())
        self.assertGreater(sky_mean / max(ground_mean, 1e-9), 2.0)

    def test_the_brightest_pixel_is_sky_not_ground(self) -> None:
        """Row 0 is the zenith, so a sky maximum also checks the row order.

        Deliberately does NOT assert the maximum's longitude: at 1k the solar
        disc is ~1.5 pixels wide and a bright cloud can out-shine the downsampled
        sun. The disc position is calibrated at 8k by connected components.
        """
        lum = pfa.luminance(self.rgb)
        height = lum.shape[0]
        row, _ = np.unravel_index(int(np.argmax(lum)), lum.shape)
        self.assertLess(int(row), height // 2, "the brightest pixel must be above the horizon")


# ---------------------------------------------------------------------------
# The tone mapper: a verbatim port of the WGSL
# ---------------------------------------------------------------------------


class TestKhronosPbrNeutralPort(unittest.TestCase):
    """Expected values come from the WGSL math, not from the numpy port."""

    def test_dark_values_use_the_parabolic_offset(self) -> None:
        # x = 0.02 < 0.08 -> offset = 0.02 - 6.25 * 0.02 ** 2 = 0.0175
        # c = 0.0025, peak < 0.76 -> returned unchanged.
        for channel, expected in ((0.02, 0.0025), (0.04, 0.01)):
            with self.subTest(channel=channel):
                result = port((channel, channel, channel))
                for value in result:
                    self.assertAlmostEqual(value, expected, delta=1e-12)

    def test_low_range_is_the_identity_minus_the_constant_offset(self) -> None:
        # x = 0.3 >= 0.08 -> offset = 0.04; peak 0.46 < 0.76 -> returned as is.
        result = port((0.5, 0.4, 0.3))
        for value, expected in zip(result, (0.46, 0.36, 0.26)):
            self.assertAlmostEqual(value, expected, delta=1e-12)

    def test_the_compression_knee_is_a_fixed_point(self) -> None:
        # c = 0.8 - 0.04 = 0.76 == start_compression, so new_peak = 1 - d = 0.76.
        result = port((0.8, 0.8, 0.8))
        for value in result:
            self.assertAlmostEqual(value, 0.76, delta=1e-12)

    def test_the_curve_is_continuous_across_the_knee(self) -> None:
        """A wrong `d` (0.04 instead of 0.24) would jump by ~0.2 right here."""
        previous = None
        worst_jump = 0.0
        steps = 200
        for index in range(steps + 1):
            value = 0.7 + 0.2 * index / steps
            out = port((value, value, value))[0]
            if previous is not None:
                worst_jump = max(worst_jump, abs(out - previous))
            previous = out
        # The input step is 1e-3 and the curve is 1-Lipschitz below the knee, so
        # a continuous curve shows adjacent deltas of about one step; a broken
        # knee shows ~0.2. Two steps separates the two cases cleanly.
        self.assertLess(worst_jump, 2e-3, f"the curve jumps by {worst_jump} at the knee")

    def test_grey_ramp_is_monotone_and_bounded(self) -> None:
        previous = -1.0
        for index in range(81):
            value = index * 0.05
            out = port((value, value, value))[0]
            self.assertGreaterEqual(out, previous - 1e-12, f"non-monotone at {value}")
            self.assertGreaterEqual(out, 0.0)
            self.assertLess(out, 1.0)
            previous = out

    def test_offset_branches_are_continuous_at_the_0_08_boundary(self) -> None:
        at_boundary = port((0.08, 0.5, 0.5))
        just_below = port((0.08 - 1e-9, 0.5, 0.5))
        for boundary, below in zip(at_boundary, just_below):
            self.assertAlmostEqual(boundary, below, delta=1e-6)

    def test_zero_maps_to_zero(self) -> None:
        self.assertEqual(port((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))

    def test_saturated_highlight_compresses_towards_the_new_peak(self) -> None:
        result = port((10.0, 1.0, 0.5))
        peak = 10.0 - 0.04
        d = 1.0 - 0.76
        new_peak = 1.0 - d * d / (peak + d - 0.76)
        # The brightest channel WAS the peak, so it lands exactly on new_peak.
        self.assertAlmostEqual(result[0], new_peak, delta=1e-12)
        self.assertGreater(result[0], result[1])
        self.assertGreater(result[1], result[2])
        for value in result:
            self.assertGreater(value, 0.0)
            self.assertLess(value, 1.0)

    def test_extreme_highlights_stay_finite_below_one(self) -> None:
        result = port((65472.0, 65472.0, 65472.0))
        for value in result:
            self.assertTrue(math.isfinite(value))
            self.assertGreater(value, 0.999)
            self.assertLessEqual(value, 1.0)

    def test_the_measured_sun_disc_peak_is_compressed_not_clipped(self) -> None:
        """(54.0, 36.25, 28.5) is the brightest pixel measured in meadow_8k.hdr."""
        color = tuple(pfa.SUN_DISC_PEAK_RGB)
        result = port(color)
        expected = wgsl_khronos_pbr_neutral(color)
        for channel in range(3):
            self.assertAlmostEqual(result[channel], expected[channel], delta=1e-12)
            self.assertGreater(result[channel], 0.0)
            self.assertLessEqual(result[channel], 1.0)
        self.assertGreater(result[0], result[1], "the red channel stays the brightest")
        # `mix(c, new_peak, g)` pulls the channels towards one another, so a
        # ratio below 1 rises towards 1: extreme highlights desaturate.
        self.assertGreater(result[1] / result[0], color[1] / color[0])
        self.assertLess(result[1] / result[0], 1.0)

    def test_vectorised_port_matches_the_scalar_wgsl_transcription(self) -> None:
        table = [
            (0.0, 0.0, 0.0),
            (1e-6, 2e-6, 3e-6),
            (0.0031308, 0.0031308, 0.0031308),
            (0.02, 0.5, 0.9),
            (0.0799999, 0.0799999, 0.0799999),
            (0.08, 0.08, 0.08),
            (0.5, 0.4, 0.3),
            (0.76, 0.76, 0.76),
            (0.8, 0.8, 0.8),
            (1.0, 1.0, 1.0),
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (2.6, 2.47, 2.21),
            (10.0, 1.0, 0.5),
            (39.446, 39.446, 39.446),
            (54.0, 36.25, 28.5),
            (128.0, 0.25, 0.0625),
            (65472.0, 65472.0, 65472.0),
        ]
        for color in table:
            with self.subTest(color=color):
                expected = wgsl_khronos_pbr_neutral(color)
                actual = port(color)
                for channel in range(3):
                    self.assertAlmostEqual(actual[channel], expected[channel], delta=1e-12)

    def test_a_whole_band_is_tonemapped_pixelwise(self) -> None:
        """The port must not mix channels between pixels (band-shaped input)."""
        band = np.array(
            [
                [[0.5, 0.4, 0.3], [10.0, 1.0, 0.5]],
                [[0.02, 0.02, 0.02], [0.8, 0.8, 0.8]],
            ],
            dtype=np.float64,
        )
        result = pfa.khronos_pbr_neutral(band)
        self.assertEqual(result.shape, band.shape)
        for row in range(2):
            for column in range(2):
                expected = wgsl_khronos_pbr_neutral(tuple(band[row, column]))
                for channel in range(3):
                    self.assertAlmostEqual(
                        float(result[row, column, channel]), expected[channel], delta=1e-12
                    )


class TestSrgbTransfer(unittest.TestCase):
    """Matching ``crates/renderer/src/texture.rs`` channel by channel."""

    def test_encode_matches_the_rust_definition(self) -> None:
        self.assertAlmostEqual(
            float(pfa.linear_to_srgb(np.array([0.5]))[0]),
            1.055 * 0.5 ** (1.0 / 2.4) - 0.055,
            delta=1e-15,
        )
        self.assertAlmostEqual(
            float(pfa.linear_to_srgb(np.array([0.0031308]))[0]),
            0.0031308 * 12.92,
            delta=1e-15,
        )

    def test_decode_matches_the_rust_definition(self) -> None:
        self.assertAlmostEqual(
            float(pfa.srgb_to_linear(np.array([0.5]))[0]),
            ((0.5 + 0.055) / 1.055) ** 2.4,
            delta=1e-15,
        )
        self.assertAlmostEqual(
            float(pfa.srgb_to_linear(np.array([0.04045]))[0]),
            0.04045 / 12.92,
            delta=1e-15,
        )

    def test_round_trip_is_the_identity_inside_the_gamut(self) -> None:
        values = np.linspace(0.0, 1.0, 1001)
        restored = pfa.srgb_to_linear(pfa.linear_to_srgb(values))
        self.assertTrue(np.allclose(restored, values, atol=1e-12))

    def test_both_directions_clamp_like_the_rust_definitions(self) -> None:
        self.assertEqual(float(pfa.linear_to_srgb(np.array([-1.0]))[0]), 0.0)
        # 1.055 * 1 - 0.055 is 0.9999999999999999 in IEEE754, in Python exactly
        # as in the Rust `linear_to_srgb_f64` this mirrors; compare with a
        # tolerance, not for bitwise 1.0.
        self.assertAlmostEqual(float(pfa.linear_to_srgb(np.array([5.0]))[0]), 1.0, delta=1e-12)
        self.assertEqual(float(pfa.srgb_to_linear(np.array([-0.5]))[0]), 0.0)
        self.assertEqual(float(pfa.srgb_to_linear(np.array([2.0]))[0]), 1.0)

    def test_quantisation_is_round_half_to_even_at_the_endpoints(self) -> None:
        self.assertEqual(int(pfa.display_referred_u8(np.array([0.0]))[0]), 0)
        self.assertEqual(int(pfa.display_referred_u8(np.array([1.0]))[0]), 255)
        self.assertEqual(int(pfa.display_referred_u8(np.array([2.0]))[0]), 255)
        self.assertEqual(int(pfa.display_referred_u8(np.array([-0.5]))[0]), 0)
        ramp = np.linspace(0.0, 1.0, 257)
        expected = np.rint(pfa.linear_to_srgb(ramp) * 255.0).astype(np.uint8)
        self.assertTrue(np.array_equal(pfa.display_referred_u8(ramp), expected))
        self.assertEqual(int(pfa.display_referred_u8(np.array([0.5]))[0]), 188)


# ---------------------------------------------------------------------------
# The equirectangular mapping, anchored to the runtime's own outputs
# ---------------------------------------------------------------------------


class TestEquirectMapping(unittest.TestCase):
    """Mirrors ``photo_field.rs::equirect_uv_from_direction`` exactly."""

    def uv(self, direction, yaw_deg=0.0, pitch_deg=0.0):
        return pfa.equirect_uv_from_direction(direction, yaw_deg, pitch_deg)

    def test_cardinal_directions_map_to_the_documented_uv(self) -> None:
        cases = [
            ((1.0, 0.0, 0.0), (0.0, 0.5)),
            ((0.0, 0.0, 1.0), (0.25, 0.5)),
            ((-1.0, 0.0, 0.0), (0.5, 0.5)),
            ((0.0, 0.0, -1.0), (0.75, 0.5)),
        ]
        for direction, expected in cases:
            with self.subTest(direction=direction):
                actual = self.uv(direction)
                self.assertAlmostEqual(actual[0], expected[0], delta=1e-12)
                self.assertAlmostEqual(actual[1], expected[1], delta=1e-12)

    def test_zenith_is_row_zero_and_nadir_is_the_last_row(self) -> None:
        zenith = self.uv((0.0, 1.0, 0.0))
        nadir = self.uv((0.0, -1.0, 0.0))
        self.assertAlmostEqual(zenith[1], 0.0, delta=1e-12)
        self.assertAlmostEqual(nadir[1], 1.0, delta=1e-12)
        self.assertAlmostEqual(zenith[0], 0.0, delta=1e-12)
        self.assertAlmostEqual(nadir[0], 0.0, delta=1e-12)

    def test_the_u_seam_wraps_seamlessly(self) -> None:
        step = 0.2 / 360.0
        before = self.uv(pfa.direction_from_azimuth_elevation(359.9, 0.0))
        after = self.uv(pfa.direction_from_azimuth_elevation(0.1, 0.0))
        self.assertGreater(before[0], 1.0 - 2.0 * step)
        self.assertLess(after[0], 2.0 * step)
        self.assertAlmostEqual((1.0 - before[0]) + after[0], step, delta=1e-12)

    def test_poles_clamp_instead_of_producing_nan(self) -> None:
        for direction in ((0.0, 1.5, 0.0), (0.0, -2.0, 0.0), (1e-12, 1.0, 1e-12)):
            with self.subTest(direction=direction):
                u, v = self.uv(direction)
                self.assertTrue(math.isfinite(u) and math.isfinite(v))
                self.assertGreaterEqual(v, 0.0)
                self.assertLessEqual(v, 1.0)
        self.assertAlmostEqual(self.uv((0.0, 1.5, 0.0))[1], 0.0, delta=1e-12)
        self.assertAlmostEqual(self.uv((0.0, -2.0, 0.0))[1], 1.0, delta=1e-12)

    def test_the_zero_vector_maps_to_the_pole_not_to_a_nan(self) -> None:
        u, v = self.uv((0.0, 0.0, 0.0))
        self.assertTrue(math.isfinite(u) and math.isfinite(v))
        self.assertAlmostEqual(v, 0.0, delta=1e-12)

    def test_unnormalised_directions_are_normalised(self) -> None:
        self.assertAlmostEqual(self.uv((3.0, 0.0, 0.0))[0], 0.0, delta=1e-12)
        self.assertAlmostEqual(self.uv((0.0, 0.0, -7.5))[0], 0.75, delta=1e-12)

    def test_direction_and_uv_are_inverses_at_zero_calibration(self) -> None:
        for azimuth in (0.0, 37.5, 90.0, 153.027, 180.0, 270.0, 359.5):
            for elevation in (-89.0, -3.647, 0.0, 45.0, 68.936, 89.0):
                with self.subTest(azimuth=azimuth, elevation=elevation):
                    direction = pfa.direction_from_azimuth_elevation(azimuth, elevation)
                    u, v = self.uv(direction)
                    self.assertAlmostEqual(u, (azimuth / 360.0) % 1.0, delta=1e-12)
                    self.assertAlmostEqual(v, 0.5 - elevation / 180.0, delta=1e-12)

    def test_the_yaw_calibration_subtracts_like_the_runtime(self) -> None:
        """Mirrors ``yaw_calibration_rotates_the_panorama_about_the_vertical_axis``.

        A calibration of +yaw puts panorama longitude 0 on world azimuth +yaw, so
        the world direction at azimuth 30 deg samples u = 0 when yaw = 30 deg.
        PF1 pins yaw = 0, but the port must reproduce the runtime as it is.
        """
        for world_azimuth, expected_u in ((30.0, 0.0), (120.0, 0.25)):
            with self.subTest(world_azimuth=world_azimuth):
                direction = pfa.direction_from_azimuth_elevation(world_azimuth, 0.0)
                u, v = self.uv(direction, yaw_deg=30.0)
                self.assertAlmostEqual(u, expected_u, delta=1e-12)
                self.assertAlmostEqual(v, 0.5, delta=1e-12)

    def test_the_pitch_calibration_is_a_rigid_rotation(self) -> None:
        """Mirrors ``pitch_calibration_is_a_rigid_rotation_of_the_sampling_frame``."""
        pitch_deg = 12.0
        pitch = math.radians(pitch_deg)
        world = (0.0, -math.sin(pitch), math.cos(pitch))
        u, v = self.uv(world, pitch_deg=pitch_deg)
        self.assertAlmostEqual(u, 0.25, delta=1e-12)
        self.assertAlmostEqual(v, 0.5, delta=1e-12)

    def test_the_180_degree_meridian_maps_to_half_way_round_u(self) -> None:
        """azimuth = atan2(z, x) puts the +/-180 deg meridian at u = 0.5."""
        azimuth = math.radians(-179.9)
        u, v = self.uv((math.cos(azimuth), 0.0, math.sin(azimuth)))
        self.assertAlmostEqual(u, 0.5 + 0.1 / 360.0, delta=1e-12)
        self.assertAlmostEqual(v, 0.5, delta=1e-12)


class TestSunCalibration(unittest.TestCase):
    def test_the_recomputed_direction_is_unit_length(self) -> None:
        direction = pfa.sun_direction_render_from_angles()
        length = math.sqrt(sum(component * component for component in direction))
        self.assertAlmostEqual(length, 1.0, delta=1e-12)

    def test_the_pinned_literal_agrees_with_the_recomputation(self) -> None:
        derived = pfa.sun_direction_render_from_angles()
        for axis in range(3):
            self.assertAlmostEqual(
                pfa.SUN_DIRECTION_RENDER[axis], derived[axis], delta=pfa.SUN_DIRECTION_TOLERANCE
            )
            self.assertAlmostEqual(pfa.SUN_DIRECTION_EXACT_F64[axis], derived[axis], delta=1e-5)

    def test_the_pinned_literal_maps_onto_the_measured_solar_texel(self) -> None:
        """Tolerance mirrors the 1e-4 the runtime uses; the residue is ~7e-6."""
        u, v = pfa.equirect_uv_from_direction(pfa.SUN_DIRECTION_RENDER)
        self.assertAlmostEqual(u, (pfa.SUN_LONGITUDE_DEG / 360.0) % 1.0, delta=5e-5)
        self.assertAlmostEqual(v, 0.5 - pfa.SUN_ELEVATION_DEG / 180.0, delta=5e-5)

    def test_the_exact_recomputation_maps_onto_the_solar_texel_exactly(self) -> None:
        u, v = pfa.equirect_uv_from_direction(pfa.sun_direction_render_from_angles())
        self.assertAlmostEqual(u, (pfa.SUN_LONGITUDE_DEG / 360.0) % 1.0, delta=1e-9)
        self.assertAlmostEqual(v, 0.5 - pfa.SUN_ELEVATION_DEG / 180.0, delta=1e-9)

    def test_the_old_scratch_convention_vector_does_not_point_at_the_sun(self) -> None:
        """Pins why the brief's first vector had to be superseded.

        ``[0.16497, 0.93297, -0.31992]`` is ``[cos(el)sin(lon), sin(el),
        cos(el)cos(lon)]``, i.e. azimuth = atan2(x, z). Under the runtime's
        azimuth = atan2(z, x) it samples u = 0.8258 - the value the Rust test
        printed when it failed - not the solar texel at u = 0.4251.
        """
        u, v = pfa.equirect_uv_from_direction((0.16497, 0.93297, -0.31992))
        self.assertAlmostEqual(u, 0.82577324, delta=1e-5)
        self.assertAlmostEqual(v, 0.117206424, delta=1e-5)
        self.assertGreater(abs(u - pfa.SUN_LONGITUDE_DEG / 360.0), 0.3)


# ---------------------------------------------------------------------------
# The garage derivation
# ---------------------------------------------------------------------------


class TestGarageDerivation(unittest.TestCase):
    def test_the_metric_box_follows_from_the_angular_box_and_the_camera_height(self) -> None:
        derived = pfa.garage_derivation()
        base = -pfa.GARAGE_ANGULAR_BOX_ELEVATION_DEG[0]
        top = pfa.GARAGE_ANGULAR_BOX_ELEVATION_DEG[1]
        distance = pfa.CAMERA_HEIGHT_M / math.tan(math.radians(base))
        height = distance * math.tan(math.radians(top)) + pfa.CAMERA_HEIGHT_M
        span = math.radians(
            pfa.GARAGE_ANGULAR_BOX_AZIMUTH_DEG[1] - pfa.GARAGE_ANGULAR_BOX_AZIMUTH_DEG[0]
        )
        self.assertAlmostEqual(derived["distance_m"], distance, delta=1e-9)
        self.assertAlmostEqual(derived["height_m"], height, delta=1e-9)
        self.assertAlmostEqual(derived["width_arc_m"], distance * span, delta=1e-9)
        self.assertLess(derived["width_chord_m"], derived["width_arc_m"])

    def test_the_derivation_still_matches_the_calibrated_metres(self) -> None:
        derived = pfa.garage_derivation()
        self.assertAlmostEqual(derived["distance_m"], 25.1, delta=0.05)
        self.assertAlmostEqual(derived["height_m"], 4.91, delta=0.05)
        self.assertAlmostEqual(derived["width_arc_m"], 16.7, delta=0.05)

    def test_an_impossible_angular_box_fails_closed(self) -> None:
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.derive_box_from_angular_box(-1.0, 5.0, 20.0)
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.derive_box_from_angular_box(3.0, 5.0, 20.0, camera_height_m=0.0)


# ---------------------------------------------------------------------------
# The GLB writer
# ---------------------------------------------------------------------------


class TestGlbWriter(unittest.TestCase):
    def triangle(self) -> pfa.GlbMesh:
        return pfa.GlbMesh(
            "pf1_test_triangle",
            [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)],
            [0, 1, 2],
        )

    def test_a_mesh_round_trips_through_the_container(self) -> None:
        mesh = self.triangle()
        payload = pfa.write_glb([mesh])
        document, bin_chunk = pfa.read_glb(payload)
        summary = pfa.glb_summary(document)
        self.assertEqual(summary["node_names"], ["pf1_test_triangle"])
        self.assertEqual(summary["triangle_count"], 1)
        self.assertEqual(summary["vertex_count"], 3)
        positions = pfa.glb_positions(document, bin_chunk, 0)
        self.assertEqual(len(positions), 3)
        for actual, expected in zip(positions, mesh.positions):
            for axis in range(3):
                self.assertAlmostEqual(actual[axis], expected[axis], delta=1e-6)

    def test_the_container_is_a_well_formed_glb_2(self) -> None:
        payload = pfa.write_glb([self.triangle()])
        magic, version, length = struct.unpack_from("<III", payload, 0)
        self.assertEqual(magic, pfa.GLB_MAGIC)
        self.assertEqual(version, 2)
        self.assertEqual(length, len(payload))
        json_length, json_type = struct.unpack_from("<II", payload, 12)
        self.assertEqual(json_type, pfa.GLB_CHUNK_JSON)
        self.assertEqual(json_length % 4, 0)
        json_chunk = payload[20 : 20 + json_length]
        stripped = json_chunk.rstrip(b" ")
        self.assertLess(json_length - len(stripped), 4, "padding must be under four bytes")
        bin_offset = 20 + json_length
        bin_length, bin_type = struct.unpack_from("<II", payload, bin_offset)
        self.assertEqual(bin_type, pfa.GLB_CHUNK_BIN)
        self.assertEqual(bin_length % 4, 0)
        self.assertEqual(bin_offset + 8 + bin_length, len(payload))
        document = json.loads(stripped.decode("utf-8"))
        self.assertEqual(document["asset"]["version"], "2.0")
        self.assertEqual(document["buffers"][0]["byteLength"], bin_length)
        self.assertEqual(document["scene"], 0)
        self.assertEqual(document["scenes"][0]["nodes"], [0])
        self.assertEqual(document["nodes"][0]["translation"], [0.0, 0.0, 0.0])
        self.assertEqual(document["nodes"][0]["rotation"], [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(document["nodes"][0]["scale"], [1.0, 1.0, 1.0])
        accessor = document["accessors"][0]
        self.assertEqual(accessor["componentType"], pfa.GLB_COMPONENT_FLOAT32)
        self.assertEqual(accessor["type"], "VEC3")
        self.assertEqual(accessor["min"], [0.0, 0.0, 0.0])
        self.assertEqual(accessor["max"], [1.0, 1.0, 0.0])
        self.assertEqual(document["accessors"][1]["componentType"], pfa.GLB_COMPONENT_UINT32)
        self.assertNotIn("materials", document)
        self.assertNotIn("images", document)
        self.assertNotIn("textures", document)

    def test_writing_is_byte_for_byte_deterministic(self) -> None:
        """Two independent authoring runs must produce identical bytes."""
        self.assertEqual(
            pfa.write_glb(authoring.build_proxy_meshes()),
            pfa.write_glb(authoring.build_proxy_meshes()),
        )

    def test_bad_meshes_are_rejected(self) -> None:
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.write_glb([])
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.write_glb([self.triangle(), self.triangle()])  # duplicate node names
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.GlbMesh("x", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], [0, 1])  # not a multiple of 3
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.GlbMesh("x", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], [0, 1, 5])  # index overflow
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.GlbMesh("", [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)], [0, 1, 2])
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.GlbMesh(
                "x",
                [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (float("nan"), 1.0, 0.0)],
                [0, 1, 2],
            )

    def test_a_corrupt_container_fails_closed(self) -> None:
        payload = bytearray(pfa.write_glb([self.triangle()]))
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.read_glb(bytes(payload[:8]))
        payload[0] = 0x00
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.read_glb(bytes(payload))


# ---------------------------------------------------------------------------
# The depth-proxy authoring contract
# ---------------------------------------------------------------------------


class TestProxyAuthoring(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.meshes = authoring.build_proxy_meshes()
        cls.by_name = {mesh.name: mesh for mesh in cls.meshes}

    def test_the_node_names_and_triangle_budget_match_the_contract(self) -> None:
        self.assertEqual([mesh.name for mesh in self.meshes], list(pfa.PROXY_NODE_NAMES))
        triangles = sum(mesh.triangle_count for mesh in self.meshes)
        self.assertEqual(triangles, pfa.EXPECTED_PROXY_TRIANGLES)
        self.assertLess(triangles, pfa.MAX_PROXY_TRIANGLES)

    def test_every_box_is_a_closed_twelve_triangle_box(self) -> None:
        boxes = [
            pfa.NODE_GARAGE,
            pfa.NODE_HOUSE_BRICK,
            pfa.NODE_HOUSE_GREEN,
            *(f"{pfa.NODE_TRUNK_PREFIX}{index}" for index in range(pfa.TRUNK_COUNT)),
        ]
        for name in boxes:
            with self.subTest(node=name):
                mesh = self.by_name[name]
                self.assertEqual(mesh.triangle_count, 12)
                self.assertEqual(mesh.vertex_count, 24)
                self.assertEqual(len(set(mesh.indices)), 24, "every vertex is used")
                self.assertEqual(min(mesh.indices), 0)
                self.assertEqual(max(mesh.indices), 23)

    def test_the_ground_disc_is_a_coarse_fan_under_the_photographic_eye(self) -> None:
        mesh = self.by_name[pfa.NODE_GROUND]
        self.assertEqual(mesh.triangle_count, pfa.GROUND_SEGMENTS)
        self.assertEqual(mesh.vertex_count, pfa.GROUND_SEGMENTS + 1)
        eye_x, _, eye_z = pfa.PILOT_EYE_RENDER_M
        self.assertEqual(mesh.positions[0], (eye_x, 0.0, eye_z))
        for position in mesh.positions:
            self.assertEqual(position[1], 0.0)
        for position in mesh.positions[1:]:
            radius = math.hypot(position[0] - eye_x, position[2] - eye_z)
            self.assertAlmostEqual(radius, pfa.GROUND_RADIUS_M, delta=1e-9)

    def test_the_garage_sits_at_its_derived_distance_and_size(self) -> None:
        derived = pfa.garage_derivation()
        mesh = self.by_name[pfa.NODE_GARAGE]
        centre = self.centre(mesh)
        expected = pfa.proxy_origin_from_eye(derived["azimuth_deg"], derived["distance_m"])
        self.assertAlmostEqual(centre[0], expected[0], delta=1e-9)
        self.assertAlmostEqual(centre[1], expected[1], delta=1e-9)
        width, depth, height = box_extents(
            mesh.positions, centre, pfa.GARAGE_CENTROID_AZIMUTH_DEG
        )
        self.assertAlmostEqual(width, derived["width_arc_m"], delta=1e-9)
        self.assertAlmostEqual(depth, pfa.GARAGE_DEPTH_M, delta=1e-9)
        self.assertAlmostEqual(height, derived["height_m"], delta=1e-9)
        ys = [position[1] for position in mesh.positions]
        self.assertAlmostEqual(min(ys), 0.0, delta=1e-9)
        self.assertAlmostEqual(max(ys), derived["height_m"], delta=1e-9)

    def test_the_manually_calibrated_obstacles_sit_where_the_contract_says(self) -> None:
        for name, azimuth, distance, width, depth, height in pfa.OBSTACLE_PLACEMENTS:
            with self.subTest(node=name):
                self.assert_placement(name, azimuth, distance, width, depth, height)
        for index, (azimuth, distance) in enumerate(pfa.TRUNK_PLACEMENTS):
            with self.subTest(node=f"{pfa.NODE_TRUNK_PREFIX}{index}"):
                self.assert_placement(
                    f"{pfa.NODE_TRUNK_PREFIX}{index}",
                    azimuth,
                    distance,
                    pfa.TRUNK_WIDTH_M,
                    pfa.TRUNK_DEPTH_M,
                    pfa.TRUNK_HEIGHT_M,
                )

    def assert_placement(
        self,
        name: str,
        azimuth_deg: float,
        distance_m: float,
        width_m: float,
        depth_m: float,
        height_m: float,
    ) -> None:
        mesh = self.by_name[name]
        centre = self.centre(mesh)
        expected = pfa.proxy_origin_from_eye(azimuth_deg, distance_m)
        self.assertAlmostEqual(centre[0], expected[0], delta=1e-9)
        self.assertAlmostEqual(centre[1], expected[1], delta=1e-9)
        measured = box_extents(mesh.positions, centre, azimuth_deg)
        for actual, expected_value, label in zip(
            measured, (width_m, depth_m, height_m), ("width", "depth", "height")
        ):
            self.assertAlmostEqual(actual, expected_value, delta=1e-9, msg=f"{name} {label}")
        ys = [position[1] for position in mesh.positions]
        self.assertAlmostEqual(min(ys), 0.0, delta=1e-9)

    def test_the_tree_ring_is_48_jittered_boxes_on_the_30_m_ring(self) -> None:
        mesh = self.by_name[pfa.NODE_TREE_RING]
        count = pfa.TREE_RING_COUNT
        self.assertEqual(mesh.triangle_count, 12 * count)
        self.assertEqual(mesh.vertex_count, 24 * count)
        eye_x, _, eye_z = pfa.PILOT_EYE_RENDER_M
        radii = []
        for index in range(count):
            vertices = mesh.positions[index * 24 : (index + 1) * 24]
            centre_x = sum(vertex[0] for vertex in vertices) / 24.0
            centre_z = sum(vertex[2] for vertex in vertices) / 24.0
            delta_x = centre_x - eye_x
            delta_z = centre_z - eye_z
            radius = math.hypot(delta_x, delta_z)
            radii.append(radius)
            self.assertGreaterEqual(radius, pfa.TREE_RING_RADIUS_M - pfa.TREE_RING_JITTER_M - 1e-9)
            self.assertLessEqual(radius, pfa.TREE_RING_RADIUS_M + pfa.TREE_RING_JITTER_M + 1e-9)
            azimuth = math.degrees(math.atan2(delta_z, delta_x)) % 360.0
            expected_azimuth = 360.0 * index / count
            # Compared as a signed angular difference: index 0 sits exactly on the
            # 0/360 wrap, where a raw subtraction would report ~360 degrees.
            difference = (azimuth - expected_azimuth + 180.0) % 360.0 - 180.0
            self.assertAlmostEqual(difference, 0.0, delta=1e-9)
            ys = [vertex[1] for vertex in vertices]
            self.assertAlmostEqual(min(ys), 0.0, delta=1e-9)
            self.assertAlmostEqual(max(ys), pfa.TREE_RING_HEIGHT_M, delta=1e-9)
        self.assertEqual(len(set(round(radius, 9) for radius in radii)), count, "jitter applied")

    def test_adjacent_tree_ring_boxes_overlap(self) -> None:
        """A ring that did not overlap would leak the sky through the tree line."""
        mesh = self.by_name[pfa.NODE_TREE_RING]
        centres = []
        for index in range(pfa.TREE_RING_COUNT):
            vertices = mesh.positions[index * 24 : (index + 1) * 24]
            centres.append(
                (
                    sum(vertex[0] for vertex in vertices) / 24.0,
                    sum(vertex[2] for vertex in vertices) / 24.0,
                )
            )
        gaps = [
            math.hypot(
                centres[(index + 1) % len(centres)][0] - centre[0],
                centres[(index + 1) % len(centres)][1] - centre[1],
            )
            for index, centre in enumerate(centres)
        ]
        self.assertGreater(min(gaps), 0.0, "two ring boxes must not share a centre")

        # The worst case is analytic: two adjacent boxes at the opposite extremes
        # of the jitter band, one angular step apart.
        outer = pfa.TREE_RING_RADIUS_M + pfa.TREE_RING_JITTER_M
        inner = pfa.TREE_RING_RADIUS_M - pfa.TREE_RING_JITTER_M
        step = math.radians(360.0 / pfa.TREE_RING_COUNT)
        worst_case = math.sqrt(
            outer * outer + inner * inner - 2.0 * outer * inner * math.cos(step)
        )
        self.assertLessEqual(max(gaps), worst_case + 1e-9, f"widest gap {max(gaps)} m")

        ordered = sorted(gaps)
        middle = len(ordered) // 2
        median = (ordered[middle - 1] + ordered[middle]) / 2.0
        self.assertLess(
            median,
            pfa.TREE_RING_WIDTH_M,
            f"the median gap {median} m is not narrower than a {pfa.TREE_RING_WIDTH_M} m box",
        )

    def test_the_jitter_is_a_frozen_integer_lcg_not_the_random_module(self) -> None:
        first = [pfa.DeterministicLcg(pfa.TREE_RING_JITTER_SEED).next_symmetric() for _ in range(1)]
        generator = pfa.DeterministicLcg(pfa.TREE_RING_JITTER_SEED)
        sequence = [generator.next_symmetric() for _ in range(pfa.TREE_RING_COUNT)]
        self.assertEqual(sequence[0], first[0])
        replay = pfa.DeterministicLcg(pfa.TREE_RING_JITTER_SEED)
        self.assertEqual([replay.next_symmetric() for _ in sequence], sequence)
        for value in sequence:
            self.assertGreaterEqual(value, -1.0)
            self.assertLess(value, 1.0)
        with self.assertRaises(pfa.PhotoFieldError):
            pfa.DeterministicLcg(1.5)

    def test_the_contract_fails_closed_when_the_geometry_moves(self) -> None:
        original = authoring.EXPECTED_PROXY_TRIANGLES
        authoring.EXPECTED_PROXY_TRIANGLES = original + 1
        self.addCleanup(setattr, authoring, "EXPECTED_PROXY_TRIANGLES", original)
        with self.assertRaises(pfa.PhotoFieldError):
            authoring.build_proxy_meshes()

        original_trunks = authoring.TRUNK_PLACEMENTS
        authoring.TRUNK_PLACEMENTS = original_trunks[:-1]
        self.addCleanup(setattr, authoring, "TRUNK_PLACEMENTS", original_trunks)
        with self.assertRaises(pfa.PhotoFieldError):
            authoring.build_proxy_meshes()

    def test_a_degenerate_box_is_rejected(self) -> None:
        with self.assertRaises(pfa.PhotoFieldError):
            authoring.box_geometry(10.0, 0.0, 1.0, 1.0, 1.0)
        with self.assertRaises(pfa.PhotoFieldError):
            authoring.box_geometry(10.0, 5.0, -1.0, 1.0, 1.0)

    @staticmethod
    def centre(mesh: pfa.GlbMesh) -> tuple[float, float]:
        count = len(mesh.positions)
        return (
            sum(position[0] for position in mesh.positions) / count,
            sum(position[2] for position in mesh.positions) / count,
        )


class TestAuthoringCommandLine(unittest.TestCase):
    def test_authoring_into_a_temporary_file_reports_the_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            destination = pathlib.Path(temporary) / "photo_field_depth.glb"
            code, out, err = capture(authoring.main, "--out", str(destination))
            self.assertEqual(code, 0, err)
            self.assertTrue(destination.is_file())
            self.assertIn("pf1_ground", out)
            self.assertIn(str(pfa.EXPECTED_PROXY_TRIANGLES), out)
            document, _ = pfa.read_glb(destination.read_bytes())
            self.assertEqual(
                pfa.glb_summary(document)["node_names"], list(pfa.PROXY_NODE_NAMES)
            )

    def test_check_mode_compares_against_the_written_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            destination = pathlib.Path(temporary) / "photo_field_depth.glb"
            self.assertEqual(capture(authoring.main, "--out", str(destination))[0], 0)
            code, out, err = capture(authoring.main, "--out", str(destination), "--check")
            self.assertEqual(code, 0, err + out)
            self.assertIn("byte-identical", out)

            destination.write_bytes(destination.read_bytes() + b"\x00\x00\x00\x00")
            code, out, err = capture(authoring.main, "--out", str(destination), "--check")
            self.assertEqual(code, 1)
            self.assertIn("NOT byte-identical", err)

    def test_check_mode_fails_closed_without_a_committed_file(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing = pathlib.Path(temporary) / "absent.glb"
            code, _, err = capture(authoring.main, "--out", str(missing), "--check")
            self.assertEqual(code, 2)
            self.assertIn("no committed GLB", err)


# ---------------------------------------------------------------------------
# The runtime manifest
# ---------------------------------------------------------------------------


class TestRuntimeManifestContract(unittest.TestCase):
    def test_the_key_set_is_exactly_what_the_renderer_deserialises(self) -> None:
        manifest = pfa.build_runtime_manifest()
        self.assertEqual(set(manifest), set(pfa.RUNTIME_MANIFEST_KEYS))
        self.assertEqual(len(manifest), len(pfa.RUNTIME_MANIFEST_KEYS))
        self.assertEqual(pfa.validate_runtime_manifest(manifest), [])

    def test_every_value_is_the_calibrated_one(self) -> None:
        manifest = pfa.build_runtime_manifest()
        self.assertEqual(manifest["schema_version"], 1)
        self.assertEqual(manifest["id"], "pf1-meadow")
        self.assertEqual(manifest["panorama"], "meadow_panorama_8192x4096.jpg")
        self.assertEqual(manifest["depth_proxy"], "photo_field_depth.glb")
        self.assertEqual(
            manifest["pilot_position_render_m"], list(pfa.PILOT_EYE_RENDER_M)
        )
        self.assertEqual(manifest["panorama_yaw_deg"], 0.0)
        self.assertEqual(manifest["panorama_pitch_deg"], 0.0)
        self.assertEqual(manifest["sun_direction_render"], [-0.32032, 0.93318, 0.16299])
        self.assertEqual(manifest["sun_intensity"], 2.6)
        self.assertEqual(manifest["sun_rgb"], [1.0, 0.95, 0.85])
        self.assertEqual(manifest["shadow_strength"], 0.45)

    def test_the_sun_direction_is_unit_length(self) -> None:
        manifest = pfa.build_runtime_manifest()
        length = math.sqrt(
            sum(component * component for component in manifest["sun_direction_render"])
        )
        self.assertAlmostEqual(length, 1.0, delta=1e-4)

    def test_the_photographic_eye_is_20_m_from_the_spawn_and_clear_of_the_trunks(self) -> None:
        """Reads the eye from the module constant; nothing here is hard-coded."""
        eye = pfa.PILOT_EYE_RENDER_M
        self.assertAlmostEqual(eye[1], pfa.CAMERA_HEIGHT_M, delta=1e-12)
        # The committed eye is rounded to millimetres, so the 20 m design distance
        # and the placement azimuth both come back with a millimetre-scale residue.
        distance = math.hypot(eye[0], eye[2])
        self.assertAlmostEqual(distance, pfa.EYE_TO_SPAWN_DISTANCE_M, delta=5e-3)
        azimuth = pfa.eye_to_spawn_azimuth_deg()
        self.assertAlmostEqual(azimuth, pfa.PILOT_EYE_AZIMUTH_DEG, delta=0.05)
        # The aircraft line-of-sight must stay on the photographed garage ...
        low, high = pfa.GARAGE_ANGULAR_BOX_AZIMUTH_DEG
        self.assertGreater(azimuth, low)
        self.assertLess(azimuth, high)
        # ... and clear of every photographed near trunk, or a trunk proxy would
        # hide the parked aircraft in the hero capture.
        clearance = min(
            abs((trunk_azimuth - azimuth + 180.0) % 360.0 - 180.0)
            for trunk_azimuth, _ in pfa.TRUNK_PLACEMENTS
        )
        self.assertGreaterEqual(clearance, 10.0)

    def test_bad_values_are_rejected(self) -> None:
        def mutated(**changes) -> list[str]:
            manifest = pfa.build_runtime_manifest()
            manifest.update(changes)
            return pfa.validate_runtime_manifest(manifest)

        self.assertIn("schema_version", " ".join(mutated(schema_version=2)))
        self.assertTrue(mutated(surprise=42))
        self.assertTrue(mutated(id=""))
        self.assertTrue(mutated(panorama="sub/dir.jpg"))
        self.assertTrue(mutated(depth_proxy="../escape.glb"))
        self.assertTrue(mutated(sun_direction_render=[0.0, 0.0, 0.0]))
        # The superseded scratch-convention vector must not be accepted: under
        # azimuth = atan2(z, x) it samples u = 0.826, not the solar texel.
        self.assertTrue(mutated(sun_direction_render=[0.16497, 0.93297, -0.31992]))
        self.assertTrue(mutated(sun_intensity=-1.0))
        self.assertTrue(mutated(shadow_strength=1.5))
        self.assertTrue(mutated(panorama_yaw_deg=720.0))
        self.assertTrue(mutated(panorama_pitch_deg=float("nan")))
        self.assertTrue(mutated(pilot_position_render_m=[0.0, 1.6, 0.0]))
        self.assertTrue(mutated(sun_rgb=[1.0, -0.5, 0.85]))
        self.assertIn(
            "must be a JSON object", " ".join(pfa.validate_runtime_manifest("not a manifest"))
        )
        self.assertIn(
            "required field missing",
            " ".join(pfa.validate_runtime_manifest({"schema_version": 1})),
        )


@unittest.skipUnless(RUNTIME_MANIFEST.is_file(), f"no committed runtime manifest at {RUNTIME_MANIFEST}")
class TestCommittedRuntimeManifest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = json.loads(RUNTIME_MANIFEST.read_text(encoding="utf-8"))

    def test_the_committed_file_satisfies_the_contract(self) -> None:
        self.assertEqual(pfa.validate_runtime_manifest(self.manifest), [])

    def test_the_committed_file_is_what_the_builder_produces(self) -> None:
        self.assertEqual(self.manifest, pfa.build_runtime_manifest())

    def test_the_committed_file_names_only_its_two_siblings(self) -> None:
        for field, path in (
            ("panorama", PANORAMA),
            ("depth_proxy", DEPTH_PROXY),
        ):
            self.assertEqual(self.manifest[field], path.name)


# ---------------------------------------------------------------------------
# The JPEG header reader (pure)
# ---------------------------------------------------------------------------


class TestJpegHeaderReader(unittest.TestCase):
    def read(self, payload: bytes) -> dict:
        with tempfile.TemporaryDirectory() as temporary:
            path = pathlib.Path(temporary) / "probe.jpg"
            path.write_bytes(payload)
            return pfa.read_jpeg_header(path)

    def test_dimensions_are_read_without_an_image_library(self) -> None:
        header = self.read(synthetic_jpeg(8192, 4096))
        self.assertEqual(header["width"], 8192)
        self.assertEqual(header["height"], 4096)
        self.assertEqual(header["precision"], 8)
        self.assertEqual(header["components"], 3)
        self.assertEqual(header["sof_marker"], 0xC0)

    def test_segments_before_the_frame_header_are_skipped(self) -> None:
        dht = b"\xff\xc4" + struct.pack(">H", 5) + b"\x00\x00\x00"
        dqt = b"\xff\xdb" + struct.pack(">H", 5) + b"\x00\x01\x02"
        header = self.read(synthetic_jpeg(64, 32, extra_segments=dht + dqt))
        self.assertEqual((header["width"], header["height"]), (64, 32))

    def test_a_non_jpeg_or_a_truncated_one_fails_closed(self) -> None:
        for name, payload in {
            "not a jpeg": b"definitely not a jpeg at all",
            "soi only": b"\xff\xd8",
            "eoi before sof": b"\xff\xd8" + b"\xff\xd9",
            "truncated segment": b"\xff\xd8" + b"\xff\xe0" + struct.pack(">H", 200) + b"xy",
            "garbage marker": b"\xff\xd8" + b"\x00\x01\x02\x03",
        }.items():
            with self.subTest(name), self.assertRaises(pfa.PhotoFieldError):
                self.read(payload)


# ---------------------------------------------------------------------------
# Committed assets and the provenance record
# ---------------------------------------------------------------------------


@unittest.skipUnless(
    PANORAMA.is_file() and DEPTH_PROXY.is_file(),
    "the committed panorama and depth proxy are needed (run the processing and "
    "authoring tools first)",
)
class TestCommittedRuntimeAssets(unittest.TestCase):
    def test_the_directory_holds_exactly_the_three_committed_files(self) -> None:
        names = sorted(path.name for path in pfa.runtime_dir().iterdir() if path.is_file())
        self.assertEqual(
            names,
            sorted(
                [
                    pfa.PANORAMA_FILE_NAME,
                    pfa.DEPTH_PROXY_FILE_NAME,
                    "photo_field_manifest.json",
                ]
            ),
        )
        self.assertEqual(
            [path.name for path in pfa.runtime_dir().iterdir() if path.is_dir()], []
        )

    @unittest.skipUnless(PANORAMA.is_file(), "the panorama derivative has not been processed yet")
    def test_the_panorama_is_an_8192x4096_eight_bit_rgb_jpeg(self) -> None:
        header = pfa.read_jpeg_header(PANORAMA)
        self.assertEqual([header["width"], header["height"]], pfa.ACQUIRED_DIMENSIONS)
        self.assertEqual(header["precision"], 8)
        self.assertEqual(header["components"], 3)
        self.assertGreater(PANORAMA.stat().st_size, 1_000_000)

    @unittest.skipUnless(DEPTH_PROXY.is_file(), "the depth proxy has not been authored yet")
    def test_the_depth_proxy_matches_the_authoring_contract(self) -> None:
        document, bin_chunk = pfa.read_glb(DEPTH_PROXY.read_bytes())
        summary = pfa.glb_summary(document)
        self.assertEqual(summary["node_names"], list(pfa.PROXY_NODE_NAMES))
        self.assertEqual(summary["triangle_count"], pfa.EXPECTED_PROXY_TRIANGLES)
        self.assertLess(summary["triangle_count"], pfa.MAX_PROXY_TRIANGLES)
        self.assertEqual(document["asset"]["version"], "2.0")
        self.assertEqual(document["buffers"][0]["byteLength"], len(bin_chunk))
        for index in range(summary["mesh_count"]):
            for position in pfa.glb_positions(document, bin_chunk, index):
                for component in position:
                    self.assertTrue(math.isfinite(component))
        ground = pfa.glb_positions(document, bin_chunk, 0)
        for position in ground:
            self.assertEqual(position[1], 0.0, "the ground proxy must lie on y = 0")

    def test_the_committed_proxy_bytes_are_reproducible(self) -> None:
        """Re-authoring in memory must reproduce the committed GLB exactly."""
        code, out, err = capture(authoring.main, "--check")
        self.assertEqual(code, 0, err + out)
        self.assertIn("byte-identical", out)
        self.assertIn(pfa.sha256_file(DEPTH_PROXY), out)


@unittest.skipUnless(PROVENANCE.is_file(), f"no provenance record at {PROVENANCE}")
class TestProvenanceRecord(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.provenance = json.loads(PROVENANCE.read_text(encoding="utf-8"))

    def test_the_record_satisfies_its_own_contract(self) -> None:
        self.assertEqual(pfa.validate_provenance(self.provenance), [])

    def test_the_licence_and_attribution_are_recorded(self) -> None:
        self.assertEqual(self.provenance["license"], "CC0")
        self.assertEqual(self.provenance["license_url"], pfa.LICENSE_URL)
        self.assertEqual(self.provenance["attribution"], pfa.ATTRIBUTION)
        self.assertEqual(self.provenance["authors"], {"Sergej Majboroda": "All"})
        self.assertEqual(self.provenance["source_page"], "https://polyhaven.com/a/meadow")

    def test_the_acquisition_cites_both_api_payloads(self) -> None:
        acquisition = self.provenance["acquisition"]
        self.assertEqual(acquisition["info_url"], pfa.INFO_URL)
        self.assertEqual(acquisition["files_url"], pfa.FILES_URL)
        self.assertRegex(acquisition["info_payload_sha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(acquisition["files_payload_sha256"], r"^[0-9a-f]{64}$")
        self.assertEqual(acquisition["source_dimensions"], [16384, 8192])
        self.assertEqual(acquisition["acquired_dimensions"], [8192, 4096])
        self.assertEqual(len(acquisition["source_files"]), len(pfa.SOURCE_FILES))
        for entry in acquisition["source_files"]:
            self.assertRegex(entry["local_sha256"], r"^[0-9a-f]{64}$")
            self.assertTrue(entry["size_verified"])
            self.assertTrue(entry["md5_verified"])

    def test_the_processing_record_names_the_command_and_the_derivative(self) -> None:
        processing_record = self.provenance["processing"]
        self.assertEqual(processing_record["command"], pfa.processing_command())
        self.assertEqual(processing_record["exposure_scale"], 1.0)
        self.assertEqual(processing_record["exposure_ev"], 0.0)
        derivative = processing_record["runtime_derivative"]
        self.assertEqual(derivative["dimensions"], [8192, 4096])
        self.assertRegex(derivative["sha256"], r"^[0-9a-f]{64}$")
        if PANORAMA.is_file():
            self.assertEqual(derivative["sha256"], pfa.sha256_file(PANORAMA))
            self.assertEqual(derivative["byte_size"], PANORAMA.stat().st_size)

    def test_the_depth_proxy_record_matches_the_committed_glb(self) -> None:
        proxy = self.provenance["depth_proxy"]
        self.assertEqual(proxy["triangle_count"], pfa.EXPECTED_PROXY_TRIANGLES)
        self.assertEqual(proxy["node_names"], list(pfa.PROXY_NODE_NAMES))
        if DEPTH_PROXY.is_file():
            self.assertEqual(proxy["sha256"], pfa.sha256_file(DEPTH_PROXY))
            self.assertEqual(proxy["byte_size"], DEPTH_PROXY.stat().st_size)

    def test_the_calibration_splits_derived_from_manual_numbers(self) -> None:
        calibration = self.provenance["calibration"]
        self.assertEqual(calibration["equirect_convention"], builder.EQUIRECT_CONVENTION)
        self.assertEqual(
            calibration["pilot_eye"]["position_render_m"], list(pfa.PILOT_EYE_RENDER_M)
        )
        self.assertAlmostEqual(
            calibration["pilot_eye"]["azimuth_from_eye_to_spawn_deg"],
            pfa.eye_to_spawn_azimuth_deg(),
            delta=1e-9,
        )
        reason = calibration["pilot_eye"]["reason"]
        self.assertIn("occlusion", reason)
        # The recorded reason must name the placement azimuth and the trunk it
        # stays clear of, both read from the constants rather than remembered.
        self.assertIn(str(int(pfa.PILOT_EYE_AZIMUTH_DEG)), reason)
        self.assertIn(str(int(pfa.TRUNK_PLACEMENTS[0][0])), reason)
        sun = calibration["sun"]
        self.assertEqual(sun["longitude_deg"], pfa.SUN_LONGITUDE_DEG)
        self.assertEqual(sun["elevation_deg"], pfa.SUN_ELEVATION_DEG)
        self.assertEqual(sun["direction_render"], list(pfa.SUN_DIRECTION_RENDER))
        self.assertIn("atan2", sun["convention"])
        self.assertIn("0.8", sun["derived_by"])
        exposure = calibration["exposure_calibration"]
        self.assertEqual(exposure["exposure_scale"], 1.0)
        evidence = exposure["prototype_fit_evidence"]
        self.assertEqual(evidence["mae_srgb_at_pinned_scale"], 0.07849)
        self.assertEqual(evidence["correlation_subsampled"], 0.9720202616649115)
        self.assertEqual(evidence["refined_scale"], 1.1790625)
        self.assertEqual(evidence["refined_mae_srgb"], 0.07181839109904041)
        self.assertIn("swapped", evidence["caveat"])
        radiance = calibration["radiance"]
        self.assertAlmostEqual(radiance["sky_upper_hemisphere_mean"], 1.2509, delta=1e-4)
        self.assertAlmostEqual(radiance["ground_lower_45_percent_mean"], 0.1644, delta=1e-4)
        garage = calibration["brick_garage"]
        self.assertAlmostEqual(garage["derived_distance_m"], 25.1, delta=0.05)
        self.assertEqual(garage["camera_height_m"], 1.6)
        self.assertTrue(calibration["physically_derived"])
        self.assertTrue(calibration["manually_calibrated"])
        self.assertIn(
            str(pfa.PILOT_EYE_RENDER_M[0]),
            " ".join(calibration["manually_calibrated"]),
            "the eye is a manual choice",
        )

    def test_not_applied_lists_everything_the_recipe_refuses_to_do(self) -> None:
        self.assertEqual(self.provenance["not_applied"], list(pfa.NOT_APPLIED))
        self.assertIn("ai_depth_estimation", self.provenance["not_applied"])
        self.assertIn("colour_lut", self.provenance["not_applied"])


# ---------------------------------------------------------------------------
# The panorama processor
# ---------------------------------------------------------------------------


class TestPanoramaProcessorGuards(unittest.TestCase):
    @unittest.skipUnless(processing is not None, "Pillow is not installed")
    def test_a_substitute_source_is_rejected_before_decoding(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            substitute = pathlib.Path(temporary) / "meadow_8k.hdr"
            substitute.write_bytes(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2 +X 2\n")
            with self.assertRaises(pfa.PhotoFieldError):
                processing.verify_source(substitute)

    @unittest.skipUnless(processing is not None, "Pillow is not installed")
    def test_a_missing_source_points_at_the_fetch_tool(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            missing = pathlib.Path(temporary) / "absent.hdr"
            with self.assertRaises(pfa.PhotoFieldError) as caught:
                processing.verify_source(missing)
            self.assertIn("fetch_photo_field_sources.py", str(caught.exception.message))

    @unittest.skipUnless(processing is not None, "Pillow is not installed")
    def test_check_mode_needs_a_committed_derivative(self) -> None:
        """--check fails closed before decoding anything when there is nothing to compare."""
        with tempfile.TemporaryDirectory() as temporary:
            missing = pathlib.Path(temporary) / "absent.jpg"
            code, _, err = capture(processing.main, "--out", str(missing), "--check")
            self.assertEqual(code, 2)
            self.assertIn("no committed panorama", err)

    @unittest.skipUnless(processing is not None, "Pillow is not installed")
    def test_the_recipe_constants_are_the_pinned_ones(self) -> None:
        self.assertEqual(processing.SOURCE_ROLE, "source_hdr")
        self.assertEqual(pfa.EXPOSURE_SCALE, 1.0)
        self.assertEqual(pfa.EXPOSURE_EV, 0.0)
        self.assertEqual(pfa.JPEG_QUALITY, 92)
        self.assertEqual(pfa.JPEG_SUBSAMPLING, 0)
        self.assertEqual(pfa.ACQUIRED_DIMENSIONS, [8192, 4096])


@unittest.skipUnless(
    processing is not None and SOURCE_HDR_8K.is_file() and PANORAMA.is_file(),
    "the 8k source cache and the committed panorama are both needed",
)
class TestPanoramaReproducibility(unittest.TestCase):
    """Slow: re-decodes the 108 MB source and re-encodes 33.5 megapixels."""

    def test_reprocessing_is_byte_identical_to_the_committed_derivative(self) -> None:
        code, out, err = capture(processing.main, "--check")
        self.assertEqual(code, 0, err + out)
        self.assertIn("byte-identical", out)
        self.assertIn(pfa.sha256_file(PANORAMA), out)


# ---------------------------------------------------------------------------
# House rules
# ---------------------------------------------------------------------------


class TestHouseRules(unittest.TestCase):
    SOURCES = (
        "photo_field_assets.py",
        "fetch_photo_field_sources.py",
        "process_photo_field_panorama.py",
        "author_photo_field_proxies.py",
        "build_photo_field_manifest.py",
        "test_photo_field_assets.py",
    )

    def test_every_module_is_ascii_only(self) -> None:
        """Redirected stdout is cp1252 on Windows, so nothing here prints non-ASCII."""
        for name in self.SOURCES:
            path = PACKAGE_DIR / name
            with self.subTest(module=name):
                self.assertTrue(path.is_file(), f"{name} is missing")
                offenders = sorted({byte for byte in path.read_bytes() if byte > 127})
                self.assertEqual(offenders, [], f"{name} contains non-ASCII bytes {offenders}")

    def test_the_deterministic_chain_never_reads_the_clock_or_random(self) -> None:
        for name in (
            "photo_field_assets.py",
            "author_photo_field_proxies.py",
            "build_photo_field_manifest.py",
            "process_photo_field_panorama.py",
        ):
            text = (PACKAGE_DIR / name).read_text(encoding="utf-8")
            with self.subTest(module=name):
                for banned in ("import random", "from random", "random.", "datetime.now"):
                    self.assertNotIn(banned, text)
        # Only the fetch tool may stamp a wall clock, and only into the receipt.
        for name in (
            "photo_field_assets.py",
            "author_photo_field_proxies.py",
            "build_photo_field_manifest.py",
            "process_photo_field_panorama.py",
        ):
            text = (PACKAGE_DIR / name).read_text(encoding="utf-8")
            with self.subTest(module=name):
                self.assertNotIn("time.time()", text)
                self.assertNotIn("time.strftime", text)

    def test_the_recorded_commands_are_workspace_relative(self) -> None:
        for command in (
            pfa.fetch_command(),
            pfa.processing_command(),
            pfa.authoring_command(),
            pfa.manifest_command(),
        ):
            self.assertEqual(command[:3], ["python", "-X", "utf8"])
            self.assertTrue(command[-1].startswith("tools/photo_field_pipeline/"))
            for element in command:
                self.assertNotIn("\\", element)
                self.assertFalse(pathlib.Path(element).is_absolute())

    def test_the_runtime_manifest_contract_matches_the_rust_struct(self) -> None:
        """The key set is read back out of photo_field.rs, not remembered here."""
        source = (REPO_ROOT / "crates/renderer/src/photo_field.rs").read_text(encoding="utf-8")
        start = source.index("pub struct PhotoFieldManifest {")
        body = source[start : source.index("}", start)]
        # [1:] drops the `pub struct ... {` line itself; what remains is the
        # field list, in declaration order, which is the manifest key order.
        fields = [
            line.split("pub")[1].split(":")[0].strip()
            for line in body.splitlines()[1:]
            if line.strip().startswith("pub ")
        ]
        self.assertEqual(tuple(fields), pfa.RUNTIME_MANIFEST_KEYS)

    def test_the_tone_mapper_port_matches_the_wgsl_source(self) -> None:
        """The constants the port must not drift from, read out of shader.wgsl."""
        source = (REPO_ROOT / "crates/renderer/src/shader.wgsl").read_text(encoding="utf-8")
        start = source.index("fn khronos_pbr_neutral(")
        body = source[start : source.index("\n}", start)]
        for fragment in (
            "let start_compression = 0.8 - 0.04;",
            "let desaturation = 0.15;",
            "let offset = select(0.04, x - 6.25 * x * x, x < 0.08);",
            "if (peak < start_compression) {",
            "let d = 1.0 - start_compression;",
            "let new_peak = 1.0 - d * d / (peak + d - start_compression);",
            "let g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);",
            "return mix(c, vec3<f32>(new_peak), g);",
        ):
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, body)
        self.assertNotIn("65472", body, "the repo's WGSL does not clamp the input")

    def test_the_srgb_thresholds_match_texture_rs(self) -> None:
        source = (REPO_ROOT / "crates/renderer/src/texture.rs").read_text(encoding="utf-8")
        self.assertIn("c <= 0.04045", source)
        self.assertIn("c <= 0.003_130_8", source)
        library = (PACKAGE_DIR / "photo_field_assets.py").read_text(encoding="utf-8")
        self.assertIn("values <= 0.04045", library)
        self.assertIn("values <= 0.0031308", library)


if __name__ == "__main__":
    unittest.main(verbosity=2)
