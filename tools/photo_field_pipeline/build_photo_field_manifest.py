"""PF1: build the two committed photo-field manifests.

Writes, in this order:

1. the RUNTIME manifest ``crates/renderer/assets/photofield/meadow/
   photo_field_manifest.json``, deserialised by ``crates/renderer/src/
   photo_field.rs`` with ``deny_unknown_fields``. It needs no measured input:
   every value is a calibrated constant, and the sun direction is re-derived
   from the measured solar longitude/elevation and checked against the pinned
   literal before it is written. It is written first so the renderer integration
   is never blocked on the provenance step.

2. the PROVENANCE record ``docs/assets/photofield/pf1_provenance.json``. It is
   measured, never invented: source digests come from the fetch receipt, the
   derivative's digest and dimensions from the committed JPEG, the proxy's
   digest and triangle count from the committed GLB. Missing inputs fail closed
   with the command that produces them.

Both documents are validated against their contract before being written.

Usage:
    python -X utf8 tools/photo_field_pipeline/build_photo_field_manifest.py
    python -X utf8 tools/photo_field_pipeline/build_photo_field_manifest.py --runtime-only
"""

from __future__ import annotations

import argparse
import pathlib
import platform
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from photo_field_assets import (  # noqa: E402
    ACQUIRED_DIMENSIONS,
    ASSET_ID,
    ASSET_NAME,
    ATTRIBUTION,
    AUTHORS,
    CAMERA_HEIGHT_M,
    EYE_TO_SPAWN_DISTANCE_M,
    EXPOSURE_EV,
    EXPOSURE_SCALE,
    EXPOSURE_FIT_CAVEAT,
    EXPOSURE_FIT_CORRELATION_SUBSAMPLED,
    EXPOSURE_FIT_MAE_AT_PINNED_SCALE,
    EXPOSURE_FIT_REFINED_EV,
    EXPOSURE_FIT_REFINED_MAE,
    EXPOSURE_FIT_REFINED_SCALE,
    EXPOSURE_FIT_RMSE_AT_PINNED_SCALE,
    FILES_URL,
    GARAGE_ANGULAR_BOX_AZIMUTH_DEG,
    GARAGE_ANGULAR_BOX_ELEVATION_DEG,
    GARAGE_CENTROID_AZIMUTH_DEG,
    GARAGE_CENTROID_ELEVATION_DEG,
    GARAGE_DEPTH_M,
    GARAGE_SEGMENT_AREA_PX,
    GROUND_RADIUS_M,
    GROUND_SEGMENTS,
    HORIZON_ROW_8K,
    INFO_URL,
    JPEG_FORMAT,
    JPEG_QUALITY,
    JPEG_SUBSAMPLING,
    LICENSE,
    LICENSE_URL,
    LUM_MAX,
    MAX_PROXY_TRIANGLES,
    MEAN_RGB,
    NEAR_SUN_SKY_MEAN,
    NOT_APPLIED,
    OBSTACLE_PLACEMENTS,
    PANORAMA_PITCH_DEG,
    PANORAMA_YAW_DEG,
    PILOT_EYE_RENDER_M,
    PROVENANCE_SCHEMA_VERSION,
    PROVIDER,
    PROXY_NODE_NAMES,
    RUNTIME_DIR_RELATIVE,
    SHADOW_STRENGTH,
    SKY_MEAN_CROSS_CHECK_BAND,
    SKY_UPPER_HEMISPHERE_MEAN,
    SLUG,
    SOURCE_DIMENSIONS,
    SOURCE_FILES,
    SOURCE_PAGE,
    SUN_DIRECTION_EXACT_F64,
    SUN_DIRECTION_RENDER,
    SUN_DIRECTION_TOLERANCE,
    SUN_DISC_MEAN_RGB,
    SUN_DISC_PEAK_RGB,
    SUN_ELEVATION_DEG,
    SUN_INTENSITY,
    SUN_LONGITUDE_DEG,
    SUN_RGB,
    TRUNK_DEPTH_M,
    TRUNK_HEIGHT_M,
    TRUNK_PLACEMENTS,
    TRUNK_WIDTH_M,
    TREE_RING_COUNT,
    TREE_RING_DEPTH_M,
    TREE_RING_HEIGHT_M,
    TREE_RING_JITTER_M,
    TREE_RING_JITTER_SEED,
    TREE_RING_RADIUS_M,
    TREE_RING_WIDTH_M,
    USER_AGENT,
    ZENITH_TOP_1_64_MEAN,
    GROUND_LOWER_45_PERCENT_MEAN,
    PhotoFieldError,
    authoring_command,
    build_runtime_manifest,
    configure_streams,
    depth_proxy_path,
    describe_jpeg,
    dump_json,
    eye_to_spawn_azimuth_deg,
    fetch_command,
    file_facts,
    garage_derivation,
    glb_summary,
    load_json,
    manifest_command,
    panorama_path,
    processing_command,
    provenance_manifest_path,
    read_glb,
    receipt_path,
    repo_root,
    require_file,
    runtime_manifest_path,
    source_file_for_role,
    sun_direction_render_from_angles,
    validate_provenance,
    validate_runtime_manifest,
)

#: The equirectangular convention sentence, recorded wherever an angle or a
#: direction is committed, so a reader never has to guess which way round it is.
EQUIRECT_CONVENTION = (
    "u = fract(azimuth / 2pi) with azimuth = atan2(dir.z, dir.x); "
    "v = 0.5 - elevation / pi with elevation = asin(dir.y). Row 0 of the "
    "committed panorama is the +90 deg zenith, the last row the nadir, and the "
    "u = 0 column is the render +X axis. This is exactly "
    "crates/renderer/src/photo_field.rs::equirect_uv_from_direction with "
    "panorama_yaw_deg = panorama_pitch_deg = 0."
)

SUN_DERIVATION = (
    "Connected-component isolation of the brightest region of meadow_8k.hdr "
    "(lum_max = 39.4461). The PF1 calibration quotes the 0.8 * lum_max isolation, "
    f"which puts the solar disc at panorama longitude {SUN_LONGITUDE_DEG} deg at "
    f"elevation {SUN_ELEVATION_DEG} deg. The prototype run recorded in "
    "tmp/pf1_sun.json isolated at 0.9 * lum_max and selected a compact 29-pixel "
    "component at 152.722 / 68.903 deg, so the two thresholds agree to 0.3 deg "
    "(about 7 texels at 8k), and FFV1's independent offline analysis of this same "
    "asset recorded 'sun ~69 deg elevation'."
)

SUN_CONVENTION_NOTE = (
    "The prototype analysis emitted its direction as "
    "[cos(el)*sin(lon), sin(el), cos(el)*cos(lon)], which corresponds to "
    "azimuth = atan2(x, z) and is therefore NOT the render convention: under "
    "azimuth = atan2(z, x) the same longitude/elevation is "
    "[cos(el)*cos(lon), sin(el), cos(el)*sin(lon)]. PF1 commits the render form. "
    f"The f64 evaluation is {list(SUN_DIRECTION_EXACT_F64)}; the committed literal "
    f"{list(SUN_DIRECTION_RENDER)} is a five-decimal evaluation of the same formula. "
    "They agree to 3e-5 (about 0.001 deg of azimuth) and both map onto the solar "
    "texel inside the 1e-5 tolerance photo_field.rs uses: the exact evaluation to "
    "~1e-12, the literal to ~7e-6."
)

PILOT_EYE_REASON = (
    "The aircraft always spawns at the render-world origin, so the surveyed "
    "photographic eye is placed 20 m from the spawn along azimuth 140 deg: "
    "eye = -20 * (cos(140 deg), 0, sin(140 deg)) = (15.321, 0, -12.856), with "
    "y = 1.6 m the assumed camera height. Azimuth 140 deg keeps the aircraft "
    "line-of-sight 12 deg clear of the photographed near tree trunk at 152 deg "
    "(whose proxy would otherwise hide the parked aircraft) while staying on "
    "the brick garage, whose measured span is 131.75-169.94 deg. From that eye "
    "the spawn - and therefore the parked aircraft - lies at 20 m, just IN "
    "FRONT of the 25.1 m garage proxy, and INSIDE the 30 m tree-ring proxy, so "
    "the mandatory occlusion test (aircraft nearer vs farther than a "
    "photographed obstacle) is expressible with the real aircraft."
)

GARAGE_DERIVATION_NOTE = (
    "A single panorama gives angles, not metres, so exactly one real-world "
    f"assumption is made: the camera height ({CAMERA_HEIGHT_M} m). With a flat "
    "ground plane, the building's base at elevation "
    f"{GARAGE_ANGULAR_BOX_ELEVATION_DEG[0]} deg is at distance "
    f"{CAMERA_HEIGHT_M} / tan({-GARAGE_ANGULAR_BOX_ELEVATION_DEG[0]:.6f} deg), its top "
    f"at {GARAGE_ANGULAR_BOX_ELEVATION_DEG[1]} deg then gives height "
    "distance * tan(top) + camera height, and its azimuth span gives the "
    "tangential width as the arc length at that distance."
)


def environment() -> dict[str, object]:
    """The tools that produced the committed bytes.

    Recorded because a JPEG's bytes depend on the encoder behind Pillow: a
    byte-reproducibility claim is only checkable against a named encoder.
    """
    def version(module_name: str) -> object:
        try:
            module = __import__(module_name)
        except ImportError:
            return None
        return getattr(module, "__version__", None)

    return {
        "python": platform.python_version(),
        "python_implementation": platform.python_implementation(),
        "platform": platform.platform(),
        "numpy": version("numpy"),
        "pillow": version("PIL"),
        "note": (
            "The derivative's bytes depend on the JPEG encoder behind Pillow "
            "(libjpeg-turbo); the --check mode of process_photo_field_panorama.py "
            "is the authority on whether this machine reproduces them."
        ),
    }


def build_acquisition(receipt: dict) -> dict[str, object]:
    """The acquisition block, taken from the fetch receipt and cross-checked."""
    for field in (
        "info_payload_sha256",
        "files_payload_sha256",
        "files_hash",
        "fetched_utc",
        "user_agent",
        "slug",
        "provider",
    ):
        if receipt.get(field) in (None, ""):
            raise PhotoFieldError(
                f"the fetch receipt has no {field!r}; re-run "
                "tools/photo_field_pipeline/fetch_photo_field_sources.py WITHOUT "
                "--offline so both API payloads are captured verbatim",
                exit_code=2,
            )
    if receipt.get("all_verified") is not True:
        raise PhotoFieldError(
            "the fetch receipt reports at least one unverified source download; "
            "provenance is not built on an unverified cache",
            exit_code=1,
        )
    if receipt.get("slug") != SLUG or receipt.get("provider") != PROVIDER:
        raise PhotoFieldError(
            f"the receipt is for {receipt.get('provider')}/{receipt.get('slug')}, "
            f"not {PROVIDER}/{SLUG}"
        )
    if receipt.get("user_agent") != USER_AGENT:
        raise PhotoFieldError("the receipt was not produced with the PF1 User-Agent")

    records = receipt.get("files")
    if not isinstance(records, list) or len(records) != len(SOURCE_FILES):
        raise PhotoFieldError(
            f"the receipt lists {len(records) if isinstance(records, list) else 'no'} "
            f"files; PF1 acquires {len(SOURCE_FILES)}",
            exit_code=2,
        )

    source_files = []
    for descriptor in SOURCE_FILES:
        entry = next(
            (item for item in records if item.get("role") == descriptor.role), None
        )
        if entry is None:
            raise PhotoFieldError(
                f"the receipt has no entry for the {descriptor.role} source", exit_code=2
            )
        for field, expected in (
            ("url", descriptor.url),
            ("api_size", descriptor.api_size),
            ("api_md5", descriptor.api_md5),
        ):
            if entry.get(field) != expected:
                raise PhotoFieldError(
                    f"the receipt's {descriptor.role}.{field} = {entry.get(field)!r} "
                    f"disagrees with the pinned {expected!r}: the Poly Haven leaf moved "
                    "and the calibration constants must be reviewed",
                    exit_code=1,
                )
        for flag in ("size_verified", "md5_verified"):
            if entry.get(flag) is not True:
                raise PhotoFieldError(
                    f"the receipt's {descriptor.role}.{flag} is not true", exit_code=1
                )
        if descriptor.pinned_sha256 is not None and (
            entry.get("local_sha256") != descriptor.pinned_sha256
        ):
            raise PhotoFieldError(
                f"the receipt's {descriptor.role} SHA-256 {entry.get('local_sha256')} "
                f"!= the pinned {descriptor.pinned_sha256}",
                exit_code=1,
            )
        source_files.append(
            {
                "role": descriptor.role,
                "file_name": descriptor.file_name,
                "purpose": descriptor.purpose,
                "api_path": list(descriptor.api_path),
                "url": entry["url"],
                "api_size": entry["api_size"],
                "api_md5": entry["api_md5"],
                "local_size": entry.get("local_size"),
                "local_md5": entry.get("local_md5"),
                "local_sha256": entry.get("local_sha256"),
                "size_verified": entry.get("size_verified"),
                "md5_verified": entry.get("md5_verified"),
            }
        )

    fetched_utc = str(receipt["fetched_utc"])
    return {
        "acquired_utc": fetched_utc,
        "acquisition_date": fetched_utc[:10],
        "user_agent": receipt["user_agent"],
        "info_url": INFO_URL,
        "files_url": FILES_URL,
        "info_payload_sha256": receipt["info_payload_sha256"],
        "files_payload_sha256": receipt["files_payload_sha256"],
        "files_hash": receipt["files_hash"],
        "asset_name": receipt.get("asset_name") or ASSET_NAME,
        "authors": receipt.get("authors") or AUTHORS,
        "source_dimensions": list(SOURCE_DIMENSIONS),
        "source_dimensions_note": (
            "the provider's largest published resolution for this asset; PF1 "
            f"acquires {ACQUIRED_DIMENSIONS[0]}x{ACQUIRED_DIMENSIONS[1]} and never "
            "resamples it"
        ),
        "acquired_dimensions": list(ACQUIRED_DIMENSIONS),
        "source_files": source_files,
    }


def build_processing(root: pathlib.Path, receipt: dict) -> dict[str, object]:
    """The processing block, measured from the committed derivative."""
    path = panorama_path(root)
    if not path.is_file():
        raise PhotoFieldError(
            f"missing {path}. Run: {' '.join(processing_command())}", exit_code=2
        )
    measured = describe_jpeg(path)
    if measured["dimensions"] != ACQUIRED_DIMENSIONS:
        raise PhotoFieldError(
            f"{path.name} is {measured['width']}x{measured['height']}, not "
            f"{ACQUIRED_DIMENSIONS[0]}x{ACQUIRED_DIMENSIONS[1]}"
        )
    if measured["precision"] != 8 or measured["components"] != 3:
        raise PhotoFieldError(
            f"{path.name} must be 8-bit 3-channel JPEG, got precision "
            f"{measured['precision']} with {measured['components']} components"
        )
    source = source_file_for_role("source_hdr")
    return {
        "command": processing_command(),
        "source": {
            "role": source.role,
            "file_name": source.file_name,
            "sha256": measured_source_sha256(receipt, source.role),
            "dimensions": list(ACQUIRED_DIMENSIONS),
            "format": "Radiance RGBE (32-bit_rle_rgbe)",
        },
        "recipe": (
            "display_linear = khronos_pbr_neutral(radiance * 1.0); "
            "srgb_bytes = round(linear_to_srgb(display_linear) * 255); "
            "JPEG quality 92, subsampling 0. No resize, no other processing."
        ),
        "tonemap": (
            "khronos_pbr_neutral, a statement-by-statement port of "
            "crates/renderer/src/shader.wgsl (the same curve the renderer applies "
            "to the 3D aircraft; the photographic background bypasses the runtime "
            "tonemapper, so it is applied here instead)"
        ),
        "colour_space": (
            "sRGB (IEC 61966-2-1) via linear_to_srgb, matching "
            "crates/renderer/src/texture.rs::linear_to_srgb_f64"
        ),
        "exposure_scale": EXPOSURE_SCALE,
        "exposure_ev": EXPOSURE_EV,
        "quantisation": "round-half-to-even of (sRGB * 255), numpy.rint",
        "jpeg": {
            "quality": JPEG_QUALITY,
            "subsampling": JPEG_SUBSAMPLING,
            "progressive": False,
            "optimize": False,
        },
        "runtime_derivative": {
            "path": f"{RUNTIME_DIR_RELATIVE}/{path.name}",
            "file_name": path.name,
            "format": JPEG_FORMAT,
            "sha256": measured["sha256"],
            "byte_size": measured["byte_size"],
            "dimensions": measured["dimensions"],
            "bit_depth": measured["precision"],
            "channels": measured["components"],
        },
        "reproducibility_check": (
            "python -X utf8 tools/photo_field_pipeline/process_photo_field_panorama.py --check"
        ),
        "environment": environment(),
    }


def measured_source_sha256(receipt: dict, role: str) -> str:
    """The receipt's SHA-256 of one source file, failing closed when absent."""
    for entry in receipt.get("files") or []:
        if entry.get("role") == role:
            digest = entry.get("local_sha256")
            if isinstance(digest, str) and len(digest) == 64:
                return digest
    raise PhotoFieldError(
        f"the fetch receipt carries no SHA-256 for the {role} source", exit_code=2
    )


def build_depth_proxy(root: pathlib.Path) -> dict[str, object]:
    """The depth-proxy block, measured from the committed GLB."""
    path = depth_proxy_path(root)
    if not path.is_file():
        raise PhotoFieldError(
            f"missing {path}. Run: {' '.join(authoring_command())}", exit_code=2
        )
    facts = file_facts(path)
    document, _bin_chunk = read_glb(path.read_bytes())
    summary = glb_summary(document)
    if summary["node_names"] != list(PROXY_NODE_NAMES):
        raise PhotoFieldError(
            f"{path.name} carries the nodes {summary['node_names']}, not the contract "
            f"{list(PROXY_NODE_NAMES)}"
        )
    if summary["triangle_count"] > MAX_PROXY_TRIANGLES:
        raise PhotoFieldError(
            f"{path.name} carries {summary['triangle_count']} triangles, over the "
            f"{MAX_PROXY_TRIANGLES} budget"
        )
    return {
        "path": f"{RUNTIME_DIR_RELATIVE}/{path.name}",
        "file_name": path.name,
        "format": "glTF 2.0 binary (GLB)",
        "sha256": facts["sha256"],
        "byte_size": facts["byte_size"],
        "triangle_count": summary["triangle_count"],
        "vertex_count": summary["vertex_count"],
        "node_count": summary["node_count"],
        "mesh_count": summary["mesh_count"],
        "node_names": summary["node_names"],
        "command": authoring_command(),
        "reproducibility_check": (
            "python -X utf8 tools/photo_field_pipeline/author_photo_field_proxies.py --check"
        ),
    }


def build_calibration() -> dict[str, object]:
    """Every number the runtime is asked to believe, with its basis."""
    derived = sun_direction_render_from_angles()
    deviation = max(
        abs(pinned - exact) for pinned, exact in zip(SUN_DIRECTION_RENDER, derived)
    )

    garage = garage_derivation()

    obstacles = []
    for name, azimuth_deg, distance_m, width_m, depth_m, height_m in OBSTACLE_PLACEMENTS:
        obstacles.append(
            {
                "node": name,
                "azimuth_deg": azimuth_deg,
                "distance_m": distance_m,
                "width_m": width_m,
                "depth_m": depth_m,
                "height_m": height_m,
                "evidence": (
                    "visual estimate from the perspective crops of the provider's "
                    "tonemapped JPG (tmp/pf1_preview/crop_*.jpg); not measurable "
                    "from a single panorama"
                ),
            }
        )
    for index, (azimuth_deg, distance_m) in enumerate(TRUNK_PLACEMENTS):
        obstacles.append(
            {
                "node": f"pf1_tree_trunk_{index}",
                "azimuth_deg": azimuth_deg,
                "distance_m": distance_m,
                "width_m": TRUNK_WIDTH_M,
                "depth_m": TRUNK_DEPTH_M,
                "height_m": TRUNK_HEIGHT_M,
                "evidence": (
                    "visual estimate of a near tree trunk from the perspective "
                    "crops; trunk radius ~0.35 m gives the 0.7 m cross-section"
                ),
            }
        )

    return {
        "equirect_convention": EQUIRECT_CONVENTION,
        "pilot_eye": {
            "position_render_m": list(PILOT_EYE_RENDER_M),
            "distance_from_spawn_m": EYE_TO_SPAWN_DISTANCE_M,
            "azimuth_from_eye_to_spawn_deg": eye_to_spawn_azimuth_deg(),
            "camera_height_m": CAMERA_HEIGHT_M,
            "reason": PILOT_EYE_REASON,
        },
        "sun": {
            "longitude_deg": SUN_LONGITUDE_DEG,
            "elevation_deg": SUN_ELEVATION_DEG,
            "direction_render": list(SUN_DIRECTION_RENDER),
            "direction_render_exact_f64": list(derived),
            "direction_render_pinned_deviation": deviation,
            "convention": EQUIRECT_CONVENTION,
            "convention_note": SUN_CONVENTION_NOTE,
            "derived_by": SUN_DERIVATION,
            "cross_check_0p9_threshold": {
                "longitude_deg": 152.72222865998236,
                "elevation_deg": 68.9027490562396,
                "component_area_px": 29,
                "source": "tmp/pf1_sun.json",
            },
            "disc_mean_radiance_rgb": list(SUN_DISC_MEAN_RGB),
            "disc_peak_radiance_rgb": list(SUN_DISC_PEAK_RGB),
            "near_sun_sky_mean": NEAR_SUN_SKY_MEAN,
            "sun_intensity": SUN_INTENSITY,
            "sun_rgb": list(SUN_RGB),
            "shadow_strength": SHADOW_STRENGTH,
            "lighting_note": (
                "sun_intensity, sun_rgb and shadow_strength are presentation "
                "constants chosen so the tonemapped aircraft reads as lit by this "
                "photograph; they are not measurements of it"
            ),
        },
        "radiance": {
            "source": "meadow_8k.hdr",
            "luminance_definition": "Rec.709: 0.2126 R + 0.7152 G + 0.0722 B",
            "lum_max": LUM_MAX,
            "sky_upper_hemisphere_mean": SKY_UPPER_HEMISPHERE_MEAN,
            "zenith_top_1_64_mean": ZENITH_TOP_1_64_MEAN,
            "ground_lower_45_percent_mean": GROUND_LOWER_45_PERCENT_MEAN,
            "mean_rgb": list(MEAN_RGB),
            "horizon_row_8k": HORIZON_ROW_8K,
            "cross_check": (
                "FFV1's independent offline analysis of this asset recorded "
                "'sun ~69 deg elevation, sky mean 1.26'; this measurement gives "
                f"{SUN_ELEVATION_DEG} deg and {SKY_UPPER_HEMISPHERE_MEAN:.4f}. The "
                "test suite re-measures the 1k probe and requires the upper-half "
                f"mean inside {list(SKY_MEAN_CROSS_CHECK_BAND)}."
            ),
        },
        "exposure_calibration": {
            "decision": (
                "exposure_scale = 1.0 (exposure_ev = 0.0), identical to the "
                "renderer's pinned postprocess exposure, so the photographic "
                "background and the tonemapped aircraft share one curve"
            ),
            "exposure_scale": EXPOSURE_SCALE,
            "exposure_ev": EXPOSURE_EV,
            "prototype_fit_evidence": {
                "source": "tmp/pf1_exposure_fit.log and tmp/pf1_exposure_fit.json",
                "reference": (
                    "the provider's own tonemapped JPG (meadow_tonemapped.jpg), "
                    "used as a look reference only and never as a runtime input"
                ),
                "mae_srgb_at_pinned_scale": EXPOSURE_FIT_MAE_AT_PINNED_SCALE,
                "rmse_srgb_at_pinned_scale": EXPOSURE_FIT_RMSE_AT_PINNED_SCALE,
                "refined_scale": EXPOSURE_FIT_REFINED_SCALE,
                "refined_exposure_ev": EXPOSURE_FIT_REFINED_EV,
                "refined_mae_srgb": EXPOSURE_FIT_REFINED_MAE,
                "correlation_subsampled": EXPOSURE_FIT_CORRELATION_SUBSAMPLED,
                "caveat": EXPOSURE_FIT_CAVEAT,
            },
        },
        "brick_garage": {
            "node": "pf1_building_brick_garage",
            "angular_box_azimuth_deg": list(GARAGE_ANGULAR_BOX_AZIMUTH_DEG),
            "angular_box_elevation_deg": list(GARAGE_ANGULAR_BOX_ELEVATION_DEG),
            "centroid_azimuth_deg": GARAGE_CENTROID_AZIMUTH_DEG,
            "centroid_elevation_deg": GARAGE_CENTROID_ELEVATION_DEG,
            "segment_area_px": GARAGE_SEGMENT_AREA_PX,
            "segmented_by": (
                "colour segmentation of the provider's tonemapped JPG "
                "(warm red brick: r > 1.22 g, g > 0.92 b) plus connected-component "
                "labelling; the largest component is the garage"
            ),
            "camera_height_m": CAMERA_HEIGHT_M,
            "derived_distance_m": garage["distance_m"],
            "derived_height_m": garage["height_m"],
            "derived_width_arc_m": garage["width_arc_m"],
            "derived_width_chord_m": garage["width_chord_m"],
            "proxy_depth_m": GARAGE_DEPTH_M,
            "derivation": GARAGE_DERIVATION_NOTE,
        },
        "manually_estimated_obstacles": obstacles,
        "depth_proxy_geometry": {
            "frame": "render world space, y-up, ground plane at y = 0",
            "origin_of_placement": list(PILOT_EYE_RENDER_M),
            "placement_rule": (
                "world = eye + distance * (cos(azimuth), 0, sin(azimuth)), with "
                "azimuth the render-space azimuth the panorama samples at"
            ),
            "ground": {
                "node": PROXY_NODE_NAMES[0],
                "radius_m": GROUND_RADIUS_M,
                "segments": GROUND_SEGMENTS,
                "centre": [PILOT_EYE_RENDER_M[0], 0.0, PILOT_EYE_RENDER_M[2]],
                "y": 0.0,
            },
            "tree_ring": {
                "node": "pf1_tree_ring",
                "box_count": TREE_RING_COUNT,
                "radius_m": TREE_RING_RADIUS_M,
                "jitter_m": TREE_RING_JITTER_M,
                "jitter_generator": (
                    f"a 64-bit integer LCG (Knuth MMIX constants) seeded with "
                    f"{TREE_RING_JITTER_SEED}; the standard library `random` module "
                    "is deliberately not used anywhere in this package"
                ),
                "width_m": TREE_RING_WIDTH_M,
                "depth_m": TREE_RING_DEPTH_M,
                "height_m": TREE_RING_HEIGHT_M,
            },
            "boxes": "closed 12-triangle boxes, 24 vertices, outward-facing winding",
            "node_transforms": (
                "every node carries an explicit identity TRS because the geometry "
                "is authored in render world space: the repository's production GLB "
                "loader reads baked vertex positions and does not apply node "
                "transforms (documented in "
                "tools/aircraft_asset_pipeline/blender_export_glb.py), while "
                "crates/renderer/src/glb.rs's scene-graph path composes them. An "
                "identity transform makes both readings agree."
            ),
            "materials": (
                "none: the runtime uses these meshes depth-only, and glb.rs falls "
                "back to generated normals for a primitive that carries none"
            ),
            "triangle_budget": MAX_PROXY_TRIANGLES,
        },
        "physically_derived": [
            (
                f"sun_direction_render {list(SUN_DIRECTION_RENDER)} from the measured "
                f"solar longitude {SUN_LONGITUDE_DEG} deg / elevation {SUN_ELEVATION_DEG} deg "
                "under the documented equirect convention"
            ),
            (
                f"brick garage distance {garage['distance_m']:.4f} m = "
                f"{CAMERA_HEIGHT_M} m / tan({-GARAGE_ANGULAR_BOX_ELEVATION_DEG[0]:.6f} deg), "
                "from the segmented base elevation and the assumed camera height"
            ),
            (
                f"brick garage height {garage['height_m']:.4f} m = "
                f"{garage['distance_m']:.4f} * tan({GARAGE_ANGULAR_BOX_ELEVATION_DEG[1]:.6f} deg) "
                f"+ {CAMERA_HEIGHT_M} m, from the segmented top elevation"
            ),
            (
                f"brick garage width {garage['width_arc_m']:.4f} m = arc length at that "
                f"distance over the segmented azimuth span "
                f"{GARAGE_ANGULAR_BOX_AZIMUTH_DEG[1] - GARAGE_ANGULAR_BOX_AZIMUTH_DEG[0]:.6f} deg "
                f"(straight chord {garage['width_chord_m']:.4f} m)"
            ),
            (
                f"brick garage azimuth {GARAGE_CENTROID_AZIMUTH_DEG:.5f} deg = centroid of the "
                f"{GARAGE_SEGMENT_AREA_PX}-pixel segmented component"
            ),
            (
                f"exposure scale {EXPOSURE_SCALE} = exp2({EXPOSURE_EV}), pinned to the "
                "renderer's postprocess exposure rather than fitted"
            ),
            (
                f"radiance means sky {SKY_UPPER_HEMISPHERE_MEAN:.4f}, zenith "
                f"{ZENITH_TOP_1_64_MEAN:.4f}, ground {GROUND_LOWER_45_PERCENT_MEAN:.4f}, "
                f"lum_max {LUM_MAX:.4f}: measured from meadow_8k.hdr"
            ),
            (
                "the panorama derivative's bytes: a deterministic function of the "
                "verified source, the ported tone mapper and the named encoder"
            ),
        ],
        "manually_calibrated": [
            (
                f"pilot_position_render_m {list(PILOT_EYE_RENDER_M)}: {PILOT_EYE_REASON}"
            ),
            (
                f"camera height {CAMERA_HEIGHT_M} m: the single real-world assumption "
                "every metric garage dimension scales with"
            ),
            (
                f"brick garage depth {GARAGE_DEPTH_M} m: a radial extent is not "
                "measurable from one panorama"
            ),
            (
                "pf1_house_brick at azimuth 268 deg / 15 m, 10 x 7 x 4.5 m, and "
                "pf1_house_green at azimuth 286 deg / 32 m, 12 x 8 x 5.5 m: visual "
                "estimates from the perspective crops"
            ),
            (
                "four near trunks at (152 deg, 5 m), (270 deg, 7 m), (285 deg, 9 m), "
                f"(300 deg, 6 m), {TRUNK_WIDTH_M} x {TRUNK_DEPTH_M} x {TRUNK_HEIGHT_M} m: "
                "visual estimates from the perspective crops"
            ),
            (
                f"the distant tree line: {TREE_RING_COUNT} boxes on a "
                f"{TREE_RING_RADIUS_M} m ring with +/- {TREE_RING_JITTER_M} m deterministic "
                f"jitter (LCG seed {TREE_RING_JITTER_SEED}), "
                f"{TREE_RING_WIDTH_M} x {TREE_RING_DEPTH_M} x {TREE_RING_HEIGHT_M} m"
            ),
            (
                f"the ground disc: {GROUND_RADIUS_M} m radius, {GROUND_SEGMENTS} segments, "
                "centred under the photographic eye"
            ),
            (
                f"sun_intensity {SUN_INTENSITY}, sun_rgb {list(SUN_RGB)}, "
                f"shadow_strength {SHADOW_STRENGTH}: presentation constants matched to "
                "the photograph by eye"
            ),
            (
                f"panorama_yaw_deg {PANORAMA_YAW_DEG} and panorama_pitch_deg "
                f"{PANORAMA_PITCH_DEG}: the panorama is used exactly as photographed"
            ),
            (
                f"JPEG quality {JPEG_QUALITY} with subsampling {JPEG_SUBSAMPLING}: an "
                "encoding choice, not a measurement"
            ),
        ],
    }


def build_provenance(root: pathlib.Path) -> dict[str, object]:
    """The whole provenance record, measured from disk and the fetch receipt."""
    receipt_file = receipt_path(root)
    receipt = load_json(
        receipt_file,
        "run tools/photo_field_pipeline/fetch_photo_field_sources.py first (without "
        "--offline if the API payloads are not cached yet): the acquisition digests "
        "are mandatory provenance and are never invented",
    )
    runtime_file = runtime_manifest_path(root)
    require_file(
        runtime_file,
        "the runtime manifest is written by this same tool before the provenance "
        "record; run it without --runtime-only",
    )
    runtime_facts = file_facts(runtime_file)
    return {
        "schema_version": PROVENANCE_SCHEMA_VERSION,
        "slice": "PF1 - Photo Flying Field v1",
        "asset_id": ASSET_ID,
        "provider": PROVIDER,
        "slug": SLUG,
        "name": ASSET_NAME,
        "authors": dict(AUTHORS),
        "license": LICENSE,
        "license_url": LICENSE_URL,
        "license_note": (
            "The Poly Haven /info payload carries no license field; CC0 is recorded "
            "from https://polyhaven.com/license, the same citation "
            "tools/env1_asset_pipeline/build_manifest.py uses."
        ),
        "attribution": ATTRIBUTION,
        "source_page": SOURCE_PAGE,
        "acquisition": build_acquisition(receipt),
        "processing": build_processing(root, receipt),
        "depth_proxy": build_depth_proxy(root),
        "runtime_manifest": {
            "path": f"{RUNTIME_DIR_RELATIVE}/{runtime_file.name}",
            "sha256": runtime_facts["sha256"],
            "byte_size": runtime_facts["byte_size"],
            "consumed_by": "crates/renderer/src/photo_field.rs (PhotoFieldManifest)",
            "commands": {
                "fetch": fetch_command(),
                "process": processing_command(),
                "author": authoring_command(),
                "build": manifest_command(),
            },
        },
        "calibration": build_calibration(),
        "not_applied": list(NOT_APPLIED),
    }


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="build_photo_field_manifest.py",
        description="Build the PF1 runtime manifest and its provenance record.",
    )
    parser.add_argument(
        "--runtime-only",
        action="store_true",
        help=(
            "write only the runtime manifest (it needs no measured input); use when "
            "the derivative, the proxy or the fetch receipt is not available yet"
        ),
    )
    args = parser.parse_args(argv)

    root = repo_root()
    try:
        manifest = build_runtime_manifest()
        errors = validate_runtime_manifest(manifest)
        if errors:
            for error in errors:
                print(f"  [FAIL] {error}", file=sys.stderr)
            raise PhotoFieldError(
                f"the runtime manifest violates its own contract ({len(errors)} error(s))"
            )
        runtime_path = runtime_manifest_path(root)
        dump_json(manifest, runtime_path)
        facts = file_facts(runtime_path)
        print("PF1 runtime manifest")
        print(f"  wrote {runtime_path}")
        print(f"  byte_size {facts['byte_size']}  sha256 {facts['sha256']}")
        print(f"  id {manifest['id']}  schema_version {manifest['schema_version']}")
        print(f"  pilot_position_render_m {manifest['pilot_position_render_m']}")
        print(f"  sun_direction_render {manifest['sun_direction_render']}")
        derived = sun_direction_render_from_angles()
        print(
            f"  re-derived from longitude {SUN_LONGITUDE_DEG} / elevation "
            f"{SUN_ELEVATION_DEG}: "
            f"[{derived[0]:.6f}, {derived[1]:.6f}, {derived[2]:.6f}] "
            f"(deviation {max(abs(a - b) for a, b in zip(SUN_DIRECTION_RENDER, derived)):.2e}, "
            f"tolerance {SUN_DIRECTION_TOLERANCE:.0e})"
        )
        print(f"  keys {sorted(manifest)}")

        if args.runtime_only:
            print("\n--runtime-only: the provenance record was not touched")
            return 0

        print("\nPF1 provenance record")
        provenance = build_provenance(root)
        errors = validate_provenance(provenance)
        if errors:
            for error in errors:
                print(f"  [FAIL] {error}", file=sys.stderr)
            raise PhotoFieldError(
                f"the provenance record violates its own contract ({len(errors)} error(s))"
            )
        provenance_file = provenance_manifest_path(root)
        dump_json(provenance, provenance_file)
        provenance_facts = file_facts(provenance_file)
        print(f"  wrote {provenance_file}")
        print(
            f"  byte_size {provenance_facts['byte_size']}  sha256 {provenance_facts['sha256']}"
        )
        print(f"  runtime derivative sha256 {provenance['processing']['runtime_derivative']['sha256']}")
        print(f"  depth proxy sha256 {provenance['depth_proxy']['sha256']}")
        print(f"  depth proxy triangles {provenance['depth_proxy']['triangle_count']}")
        print(f"\n{ATTRIBUTION}")
        return 0
    except PhotoFieldError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code


if __name__ == "__main__":
    sys.exit(main())
