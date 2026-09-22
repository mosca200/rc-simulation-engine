"""ENV1-A: measure and collect the BEFORE/AFTER capture evidence.

Compares the base-SHA capture against the ENV1-A capture of the same canonical
scene at three resolutions, folds in the runtime receipts, the C2D visual
audits and the runner's capture evidence, and writes one machine-readable
document plus the PNG artifacts under
``docs/validation/env1_a_sparse_grass/``.

Every metric in the output is either read from a runtime artifact or computed
from two PNGs on disk. Nothing is estimated: a metric the runtime does not
produce is recorded as ``null`` together with the reason it is unavailable, and
``visual_pass`` stays ``null`` because this repository has no approved visual
verdict.

Uses Pillow/numpy on purpose. ``tools/visual_benchmark`` is deliberately
standard-library only and this script is kept outside that package so its
invariant is not weakened.

Usage:
    python -X utf8 tools/measure_env1a_sparse_grass.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import pathlib
import shutil
import sys

import numpy as np
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent.parent
BEFORE_ROOT = ROOT / "tmp" / "env1a_before"
AFTER_ROOT = ROOT / "tmp" / "env1a_after"
C2D_ROOT = ROOT / "tmp" / "env1a_after_c2d"
OUT_DIR = ROOT / "docs" / "validation" / "env1_a_sparse_grass"
PNG_DIR = OUT_DIR / "png"
EVIDENCE_PATH = OUT_DIR / "env1a_evidence.json"

RESOLUTIONS = (
    ("1920x1080", "aircraft_acro_static_front", "docs/validation/visual_benchmark/vis0_reference_scene.json"),
    ("2560x1440", "env1a_reference_1440p", "docs/validation/env1_a_sparse_grass/env1a_reference_1440p.json"),
    ("3840x2160", "env1a_reference_2160p", "docs/validation/env1_a_sparse_grass/env1a_reference_2160p.json"),
)

C2D_CHANNELS = ("albedo", "normal", "roughness", "macro", "detail")

VISUAL_PASS_REASON = (
    "visual_pass stays null: capture evidence records facts and never a visual "
    "verdict. No SSIM/PSNR/LPIPS threshold and no automatic PASS/FAIL exists in "
    "this repository; a visual verdict needs human review or an approved metrics "
    "engine, neither of which exists here."
)

UNAVAILABLE = {
    "gpu_frame_duration_ns": (
        "RuntimeVisualAudit 1.0.0 exposes six per-pass GPU timestamp durations "
        "but no whole-frame GPU timestamp. Summing the passes would be this "
        "script's arithmetic, not a measurement, and the passes are not proven "
        "to be contiguous."
    ),
    "terrain_duration_ns": (
        "The terrain is drawn inside the `scene` pass. There is no terrain pass "
        "in PassId and no terrain-specific timing instrumentation; the audit's "
        "terrain block is configuration only."
    ),
    "total_draw_calls": (
        "Only vegetation draw calls are counted (vegetation.stats.scene_draw_calls "
        "and shadow_draw_calls). Terrain, aircraft, sky and postprocess draws are "
        "not instrumented."
    ),
    "vram_bytes": (
        "No VRAM introspection exists anywhere in this repository (no wgpu memory "
        "query, no NVML, no nvidia-smi scraping), and the VisualCaptureEvidence "
        "1.0.0 schema forbids a hardware.vram_gb key. No authoritative "
        "measurement exists, so none is reported."
    ),
    "frame_timing_frame_age_note": (
        "The profiler polls GPU timestamps without ever busy-waiting, so the "
        "capture frame may legitimately report an older sample or none at all. "
        "gpu_timing_status is recorded verbatim rather than treated as a failure."
    ),
}


def load_json(path: pathlib.Path) -> dict:
    if not path.is_file():
        raise SystemExit(f"missing required artifact: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def run_artifacts(root: pathlib.Path, resolution: str, scene_id: str) -> dict:
    scene = root / resolution / scene_id
    pngs = sorted(scene.glob("*.png"))
    if len(pngs) != 1:
        raise SystemExit(f"expected exactly one capture PNG in {scene}, found {len(pngs)}")
    return {
        "scene_dir": scene,
        "png": pngs[0],
        "receipt": load_json(scene / "runtime_capture_receipt.json"),
        "audit": load_json(scene / "runtime_visual_audit.json"),
        "evidence": load_json(scene / "capture_evidence.json"),
        "run": load_json(scene / "run.json"),
    }


def compare_pngs(before: pathlib.Path, after: pathlib.Path) -> dict:
    """Per-pixel comparison of two lossless RGBA captures."""
    left = np.asarray(Image.open(before).convert("RGBA"), dtype=np.int32)
    right = np.asarray(Image.open(after).convert("RGBA"), dtype=np.int32)
    if left.shape != right.shape:
        raise SystemExit(f"shape mismatch: {left.shape} vs {right.shape}")

    height, width, _ = left.shape
    rgb_left, rgb_right = left[..., :3], right[..., :3]
    delta = np.abs(rgb_left - rgb_right)
    per_pixel_max = delta.max(axis=2)
    changed = int(np.count_nonzero(per_pixel_max))
    above_one = int(np.count_nonzero(per_pixel_max > 1))
    peak = int(per_pixel_max.max())
    peak_y, peak_x = (int(v) for v in np.unravel_index(per_pixel_max.argmax(), per_pixel_max.shape))
    luminance_left = rgb_left.mean(axis=2)
    luminance_right = rgb_right.mean(axis=2)

    # Horizontal bands. The canonical camera looks slightly down at the parked
    # aircraft from the pilot seat, so the top band is sky and the lower bands
    # are terrain. A material change confined to the terrain must leave the sky
    # band untouched; this is measured, not assumed.
    bands = {}
    band_count = 4
    for index in range(band_count):
        top = height * index // band_count
        bottom = height * (index + 1) // band_count
        band_delta = per_pixel_max[top:bottom]
        band_pixels = int(band_delta.size)
        band_changed = int(np.count_nonzero(band_delta))
        bands[f"rows_{top}_to_{bottom}"] = {
            "pixel_count": band_pixels,
            "changed_pixels": band_changed,
            "changed_percent": round(100.0 * band_changed / band_pixels, 4) if band_pixels else None,
            "mean_abs_diff": round(float(delta[top:bottom].mean()), 6),
        }

    return {
        "width": width,
        "height": height,
        "pixel_count": width * height,
        "identical": changed == 0,
        "changed_pixels": changed,
        "changed_percent": round(100.0 * changed / float(width * height), 4),
        "pixels_above_one_level": above_one,
        "pixels_above_one_level_percent": round(100.0 * above_one / float(width * height), 4),
        "peak_channel_diff": peak,
        "peak_channel_diff_at_xy": [peak_x, peak_y],
        "mean_abs_diff_per_channel": [
            round(float(delta[..., channel].mean()), 6) for channel in range(3)
        ],
        "mean_abs_diff_overall": round(float(delta.mean()), 6),
        "before_brighter_pixels": int(np.count_nonzero(luminance_left > luminance_right)),
        "after_brighter_pixels": int(np.count_nonzero(luminance_right > luminance_left)),
        "mean_luminance_before": round(float(luminance_left.mean()), 6),
        "mean_luminance_after": round(float(luminance_right.mean()), 6),
        "alpha_identical": bool(np.array_equal(left[..., 3], right[..., 3])),
        "horizontal_bands": bands,
    }


def audit_summary(audit: dict) -> dict:
    """Copy the audit facts verbatim; never recompute or aggregate them."""
    profiling = audit.get("profiling", {})
    return {
        "identity": audit.get("identity"),
        "device": audit.get("device"),
        "terrain": audit.get("terrain"),
        "vegetation": audit.get("vegetation"),
        "image_pipeline": audit.get("image_pipeline"),
        "shadows": audit.get("shadows"),
        "profiling": {
            "presentation_frame_index": profiling.get("presentation_frame_index"),
            "cpu_frame_duration_ns": profiling.get("cpu_frame_duration_ns"),
            "gpu_timing_status": profiling.get("gpu_timing_status"),
            "gpu_timing_source_presentation_frame_index": profiling.get(
                "gpu_timing_source_presentation_frame_index"
            ),
            "gpu_timing_frame_age": profiling.get("gpu_timing_frame_age"),
            "gpu_timing_unavailable_reason": profiling.get("gpu_timing_unavailable_reason"),
            "passes": [
                {
                    "pass_id": entry.get("pass_id"),
                    "label": entry.get("label"),
                    "cpu_duration_ns": entry.get("cpu_duration_ns"),
                    "gpu_duration_ns": entry.get("gpu_duration_ns"),
                }
                for entry in profiling.get("passes", [])
            ],
        },
    }


CANDIDATE_NOTE = (
    "Candidate capture: raw PNGs stay under the gitignored tmp/ tree and are "
    "referenced here by path and SHA-256. A PNG is committed only when it is "
    "explicitly promoted to an approved golden/beauty baseline; ENV1-A has no "
    "such promotion (visual_pass is null), so committed_png is null."
)


def artifact_entry(local: pathlib.Path, promoted_name: str, promote: bool) -> dict:
    """Record a capture artifact without ever implying a committed PNG exists."""
    return {
        "committed_png": (
            f"docs/validation/env1_a_sparse_grass/png/{promoted_name}" if promote else None
        ),
        "committed_png_note": (
            "Explicitly promoted to a committed baseline." if promote else CANDIDATE_NOTE
        ),
        "local_png": str(local.relative_to(ROOT)).replace("\\", "/"),
        "local_png_sha256": sha256_file(local),
    }


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 22), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="measure_env1a_sparse_grass.py",
        description="Measure and collect the ENV1-A BEFORE/AFTER capture evidence.",
    )
    parser.add_argument(
        "--promote-png",
        action="store_true",
        help=(
            "copy the capture PNGs into docs/validation/env1_a_sparse_grass/png/ "
            "and record committed paths. Off by default: candidate captures stay "
            "under tmp/ and are referenced by digest only."
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if not BEFORE_ROOT.is_dir() or not AFTER_ROOT.is_dir():
        raise SystemExit(
            f"expected BEFORE runs under {BEFORE_ROOT} and AFTER runs under {AFTER_ROOT}"
        )

    promote = bool(args.promote_png)
    if promote:
        PNG_DIR.mkdir(parents=True, exist_ok=True)
    resolutions = []

    for resolution, scene_id, manifest in RESOLUTIONS:
        before = run_artifacts(BEFORE_ROOT, resolution, scene_id)
        after = run_artifacts(AFTER_ROOT, resolution, scene_id)
        print(f"=== {resolution} ({scene_id}) ===")

        comparison = compare_pngs(before["png"], after["png"])
        print(
            f"  changed {comparison['changed_pixels']} px "
            f"({comparison['changed_percent']}%), peak {comparison['peak_channel_diff']}, "
            f"mean {comparison['mean_abs_diff_overall']}"
        )

        before_name = f"BEFORE_{resolution}.png"
        after_name = f"AFTER_{resolution}.png"
        if promote:
            shutil.copyfile(before["png"], PNG_DIR / before_name)
            shutil.copyfile(after["png"], PNG_DIR / after_name)

        resolutions.append(
            {
                "resolution": resolution,
                "scene_id": scene_id,
                "manifest": manifest,
                "before": {
                    **artifact_entry(before["png"], before_name, promote),
                    "image_sha256": before["receipt"]["image_sha256"],
                    "image_byte_size": before["receipt"]["image_byte_size"],
                    "framebuffer": [
                        before["receipt"]["framebuffer_width"],
                        before["receipt"]["framebuffer_height"],
                    ],
                    "presentation_frame_index": before["receipt"]["presentation_frame_index"],
                    "git_commit_sha": before["evidence"]["source"]["commit_sha"],
                    "git_dirty": before["evidence"]["source"]["dirty"],
                    "runner_success": before["run"]["verdict"]["runner_success"],
                    "capture_evidence_valid": before["run"]["capture_evidence_validation"]["valid"],
                    "visual_audit_valid": before["run"]["runtime_visual_audit_validation"]["valid"],
                    "audit": audit_summary(before["audit"]),
                },
                "after": {
                    **artifact_entry(after["png"], after_name, promote),
                    "image_sha256": after["receipt"]["image_sha256"],
                    "image_byte_size": after["receipt"]["image_byte_size"],
                    "framebuffer": [
                        after["receipt"]["framebuffer_width"],
                        after["receipt"]["framebuffer_height"],
                    ],
                    "presentation_frame_index": after["receipt"]["presentation_frame_index"],
                    "git_commit_sha": after["evidence"]["source"]["commit_sha"],
                    "git_dirty": after["evidence"]["source"]["dirty"],
                    "runner_success": after["run"]["verdict"]["runner_success"],
                    "capture_evidence_valid": after["run"]["capture_evidence_validation"]["valid"],
                    "visual_audit_valid": after["run"]["runtime_visual_audit_validation"]["valid"],
                    "audit": audit_summary(after["audit"]),
                },
                "pixel_comparison": comparison,
                "unavailable_metrics": {
                    key: {"value": None, "reason": reason}
                    for key, reason in UNAVAILABLE.items()
                },
            }
        )

    # The five C2D terrain debug channels must stay capturable with the new
    # material; their PNGs are the proof, and the audits carry the selector.
    debug_channels = []
    for channel in C2D_CHANNELS:
        channel_root = C2D_ROOT / channel
        scenes = sorted(entry for entry in channel_root.glob("*") if entry.is_dir())
        if len(scenes) != 1:
            raise SystemExit(f"expected one scene directory under {channel_root}, found {len(scenes)}")
        scene = scenes[0]
        pngs = sorted(scene.glob("*.png"))
        if len(pngs) != 1:
            raise SystemExit(f"expected one C2D capture for {channel} in {scene}")
        receipt = load_json(scene / "runtime_capture_receipt.json")
        audit = load_json(scene / "runtime_visual_audit.json")
        run = load_json(scene / "run.json")
        name = f"AFTER_c2d_terrain_{channel}.png"
        if promote:
            shutil.copyfile(pngs[0], PNG_DIR / name)
        reported = audit["terrain"]["debug_mode"]
        if reported != channel:
            raise SystemExit(
                f"C2D {channel}: the audit reports debug_mode {reported!r}"
            )
        print(f"  C2D {channel:11} debug_mode={reported} sha256={receipt['image_sha256'][:16]}...")
        debug_channels.append(
            {
                "terrain_debug": channel,
                "manifest": f"docs/validation/visual_benchmark/c2d_terrain_{channel}.json",
                **artifact_entry(pngs[0], name, promote),
                "image_sha256": receipt["image_sha256"],
                "image_byte_size": receipt["image_byte_size"],
                "framebuffer": [receipt["framebuffer_width"], receipt["framebuffer_height"]],
                "audit_debug_mode": reported,
                "audit_material_path_active": audit["terrain"]["material_path_active"],
                "audit_sampler_anisotropy": audit["terrain"]["sampler_anisotropy"],
                "runner_success": run["verdict"]["runner_success"],
                "visual_audit_valid": run["runtime_visual_audit_validation"]["valid"],
            }
        )

    document = {
        "schema_version": 1,
        "slice": "ENV1-A",
        "title": "Photorealistic flying field: Sparse Grass production material BEFORE/AFTER",
        "method": (
            "The canonical VIS0 reference scene was captured with the VIS0-B "
            "runner (rv2-vis0-benchmark-runner 1.3.0) driving rcsim-app render "
            "--renderer v2. BEFORE was produced by the release binary built at "
            "the base SHA before any ENV1-A edit; AFTER by the release binary "
            "with the ENV1-A material. Both runs use the identical manifest, so "
            "camera, scenery, exposure, aircraft state, warmup and capture frame "
            "are held constant by construction."
        ),
        "hardware_note": (
            "adapter_name/backend/driver are copied verbatim from "
            "RuntimeVisualAudit.device. VisualCaptureEvidence.hardware keeps "
            "them null by contract (no runtime source feeds those leaves), so "
            "the audit is the authority here."
        ),
        "verdict": {"visual_pass": None, "visual_pass_reason": VISUAL_PASS_REASON},
        "resolutions": resolutions,
        "c2d_terrain_debug_channels": debug_channels,
        "reproduction": {
            "before": (
                "check out the base SHA e84876a1b93b9ccf7a4d04af777bf3d4eb2a0882, "
                "cargo build --release --workspace --all-features, then run "
                "tools/visual_benchmark/run_benchmark.py --execute with each manifest"
            ),
            "after": (
                "cargo build --release --workspace --all-features on the ENV1-A "
                "HEAD, then run tools/visual_benchmark/run_benchmark.py --execute "
                "with the same manifests"
            ),
            "measure": "python -X utf8 tools/measure_env1a_sparse_grass.py",
        },
    }

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    EVIDENCE_PATH.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"\nwrote {EVIDENCE_PATH}")
    if promote:
        print(f"png artifacts promoted into {PNG_DIR}")
    else:
        print("png artifacts left under tmp/ (candidate); evidence records digests only")
    return 0


if __name__ == "__main__":
    sys.exit(main())
