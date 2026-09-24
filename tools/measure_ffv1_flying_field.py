"""FFV1 Flying Field v1: BEFORE/AFTER measurement and evidence assembly.

Compares the base-SHA (224bad6) captures against the slice captures of the
same authoritative hero scene at 1080p / 1440p / 4K, folds in the runtime
receipts and RuntimeVisualAudit 1.0.0 facts, and writes
``docs/validation/ffv1_flying_field/ffv1_evidence.json``.

Every metric here is either read from a runtime artifact or computed from two
PNGs on disk. Nothing is estimated. Metrics the repository cannot produce
(whole-frame GPU time, VRAM, total draw calls, FPS percentiles) are recorded
as ``null`` with a written reason, and ``visual_pass`` stays ``null``: capture
evidence records facts and never a visual verdict.

Candidate PNGs are NOT committed (project rule): the evidence records
``committed_png: null`` plus the local ``tmp/`` path and SHA-256.

Usage:
    python -X utf8 tools/measure_ffv1_flying_field.py
"""
from __future__ import annotations

import hashlib
import json
import pathlib
import statistics
import sys

import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "validation" / "ffv1_flying_field"
EVIDENCE = OUT_DIR / "ffv1_evidence.json"

RESOLUTIONS = (
    ("1920x1080", "ffv1_hero_1080p"),
    ("2560x1440", "ffv1_hero_1440p"),
    ("3840x2160", "ffv1_hero_2160p"),
)

UNAVAILABLE = {
    "gpu_frame_duration_ns": (
        "the runtime exposes only the six per-pass gpu_duration_ns values; a "
        "whole-frame GPU time would be this script's arithmetic, not a "
        "measurement, and the passes are not proven contiguous"
    ),
    "vram_bytes": (
        "no wgpu memory introspection, no NVML and no nvidia-smi scraping; the "
        "VisualCaptureEvidence 1.0.0 schema forbids a hardware.vram_gb key"
    ),
    "total_draw_calls": (
        "only vegetation draw calls are instrumented "
        "(vegetation.stats.scene_draw_calls / shadow_draw_calls)"
    ),
    "fps_percentiles": (
        "no FPS sampler exists; the audit records a single deterministic "
        "capture-frame cpu_frame_duration_ns plus per-pass GPU timings"
    ),
}


def sha256_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 22), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_run(side: str, resolution: str, scene_id: str) -> dict:
    base = REPO_ROOT / "tmp" / f"ffv1_{side}" / resolution / scene_id
    run = json.loads((base / "run.json").read_text(encoding="utf-8"))
    receipt = json.loads((base / "runtime_capture_receipt.json").read_text(encoding="utf-8"))
    audit = json.loads((base / "runtime_visual_audit.json").read_text(encoding="utf-8"))
    evidence = json.loads((base / "capture_evidence.json").read_text(encoding="utf-8"))
    manifest = json.loads(
        (OUT_DIR / f"{scene_id}.json").read_text(encoding="utf-8")
    )
    png = base / manifest["capture"]["filename"]
    return {
        "run": run,
        "receipt": receipt,
        "audit": audit,
        "evidence": evidence,
        "png": png,
    }


def side_entry(side: str, resolution: str, scene_id: str) -> dict:
    bundle = load_run(side, resolution, scene_id)
    png = bundle["png"]
    receipt = bundle["receipt"]
    audit = bundle["audit"]
    return {
        "png": str(png),
        "committed_png": None,
        "committed_png_note": (
            "candidate capture: not committed per project rule; hashes and the "
            "receipt/audit are the committed evidence"
        ),
        "local_png": str(png.relative_to(REPO_ROOT)).replace("\\", "/"),
        "local_png_sha256": sha256_file(png),
        "image_sha256": receipt["image_sha256"],
        "image_byte_size": receipt["image_byte_size"],
        "framebuffer": [receipt["framebuffer_width"], receipt["framebuffer_height"]],
        "presentation_frame_index": receipt["presentation_frame_index"],
        "git_commit_sha": bundle["evidence"]["source"]["commit_sha"],
        "git_dirty": bundle["evidence"]["source"]["dirty"],
        "runner_success": bundle["run"]["verdict"]["runner_success"],
        "capture_evidence_valid": bundle["run"]["capture_evidence_validation"]["valid"],
        "visual_audit_valid": bundle["run"]["runtime_visual_audit_validation"]["valid"],
        "audit": {
            "identity": audit["identity"],
            "device": audit["device"],
            "terrain": audit["terrain"],
            "vegetation": audit["vegetation"],
            "image_pipeline": audit["image_pipeline"],
            "shadows": audit["shadows"],
            "profiling": audit["profiling"],
        },
    }


def compare_pngs(before: pathlib.Path, after: pathlib.Path) -> dict:
    a = np.asarray(Image.open(before).convert("RGB"), dtype=np.int16)
    b = np.asarray(Image.open(after).convert("RGB"), dtype=np.int16)
    height, width, _ = a.shape
    diff = np.abs(a - b)
    changed = np.any(diff > 0, axis=2)
    bands = {}
    for index in range(4):
        top = index * height // 4
        bottom = (index + 1) * height // 4
        band_changed = changed[top:bottom]
        bands[f"rows_{top}_to_{bottom}"] = {
            "pixel_count": int(band_changed.size),
            "changed_pixels": int(band_changed.sum()),
            "changed_percent": round(100.0 * float(band_changed.mean()), 4),
            "mean_abs_diff": round(float(diff[top:bottom].mean()), 6),
        }
    lum_a = (0.2126 * a[..., 0] + 0.7152 * a[..., 1] + 0.0722 * a[..., 2]) / 255.0
    lum_b = (0.2126 * b[..., 0] + 0.7152 * b[..., 1] + 0.0722 * b[..., 2]) / 255.0
    return {
        "width": width,
        "height": height,
        "pixel_count": width * height,
        "identical": bool(np.array_equal(a, b)),
        "changed_pixels": int(changed.sum()),
        "changed_percent": round(100.0 * float(changed.mean()), 4),
        "peak_channel_diff": int(diff.max()),
        "mean_abs_diff_per_channel": [round(float(diff[..., c].mean()), 6) for c in range(3)],
        "mean_abs_diff_overall": round(float(diff.mean()), 6),
        "mean_luminance_before": round(float(lum_a.mean()), 6),
        "mean_luminance_after": round(float(lum_b.mean()), 6),
        "horizontal_bands": bands,
    }


def repeated_scene_timings(side: str, resolution: str, scene_id: str) -> dict:
    """Read four clean, independent capture runs of the authoritative manifest."""
    roots = [REPO_ROOT / "tmp" / f"ffv1_{side}" / resolution / scene_id]
    roots.extend(
        REPO_ROOT / "tmp" / "ffv1_perf_final" / side / resolution
        / f"run{index}" / scene_id
        for index in range(1, 4)
    )
    samples = []
    for run_index, root in enumerate(roots):
        audit = json.loads((root / "runtime_visual_audit.json").read_text(encoding="utf-8"))
        evidence = json.loads((root / "capture_evidence.json").read_text(encoding="utf-8"))
        run = json.loads((root / "run.json").read_text(encoding="utf-8"))
        if evidence["source"]["dirty"] or not run["verdict"]["runner_success"]:
            raise ValueError(f"unclean or failed timing sample: {root}")
        profiling = audit["profiling"]
        scene = next(p for p in profiling["passes"] if p["pass_id"] == "scene")
        samples.append({
            "run_index": run_index,
            "git_commit_sha": evidence["source"]["commit_sha"],
            "git_dirty": evidence["source"]["dirty"],
            "presentation_frame_index": profiling["presentation_frame_index"],
            "gpu_timing_status": profiling["gpu_timing_status"],
            "gpu_timing_source_presentation_frame_index": profiling[
                "gpu_timing_source_presentation_frame_index"
            ],
            "gpu_timing_frame_age": profiling["gpu_timing_frame_age"],
            "scene_gpu_ms": scene["gpu_duration_ns"] / 1_000_000,
        })
    if len({sample["git_commit_sha"] for sample in samples}) != 1:
        raise ValueError(f"mixed commits in {side} timing samples at {resolution}")
    values = [sample["scene_gpu_ms"] for sample in samples]
    return {
        "method": (
            "four independent process launches of the authoritative frame-30 "
            "manifest; run 0 is the hero capture, runs 1-3 are repeats; "
            "GPU timestamps may represent frame 30 or the preceding frame"
        ),
        "samples": samples,
        "median_scene_gpu_ms": round(statistics.median(values), 6),
        "min_scene_gpu_ms": round(min(values), 6),
        "max_scene_gpu_ms": round(max(values), 6),
    }


def main() -> int:
    resolutions = []
    for resolution, scene_id in RESOLUTIONS:
        before = side_entry("before", resolution, scene_id)
        after = side_entry("after", resolution, scene_id)
        resolutions.append(
            {
                "resolution": resolution,
                "scene_id": scene_id,
                "manifest": f"docs/validation/ffv1_flying_field/{scene_id}.json",
                "before": before,
                "after": after,
                "pixel_comparison": compare_pngs(before["png"], after["png"]),
                "repeated_scene_timings": {
                    "before": repeated_scene_timings("before", resolution, scene_id),
                    "after": repeated_scene_timings("after", resolution, scene_id),
                },
                "unavailable_metrics": {
                    key: {"value": None, "reason": reason}
                    for key, reason in UNAVAILABLE.items()
                },
            }
        )
        print(f"{resolution}: changed {resolutions[-1]['pixel_comparison']['changed_percent']}%")

    document = {
        "schema_version": 1,
        "slice": "FFV1",
        "title": "Flying Field visual vertical slice v1 - BEFORE/AFTER evidence",
        "method": (
            "rv2-vis0-benchmark-runner 1.3.0 executed the authoritative hero "
            "manifest at three resolutions against release binaries built "
            "from clean base 224bad6 and FFV1 capture-commit checkouts; "
            "this script compares the verified PNGs and folds the "
            "runtime receipts and RuntimeVisualAudit 1.0.0 facts into one "
            "document. No metric is estimated."
        ),
        "hardware_note": (
            "GPU identity comes from RuntimeVisualAudit.device (adapter_name, "
            "backend, driver); the evidence schema carries no VRAM field."
        ),
        "verdict": {
            "visual_pass": None,
            "visual_pass_reason": (
                "visual_pass stays null: capture evidence records facts and "
                "never a visual verdict; the visual gate is human review."
            ),
        },
        "resolutions": resolutions,
        "reproduction": {
            "before": (
                "check out 224bad6588465b6828ed1f7236d479c04788a6f4, cargo build "
                "--release --workspace, then run tools/visual_benchmark/"
                "run_benchmark.py --execute with each ffv1_hero manifest"
            ),
            "after": (
                "check out the slice HEAD, cargo build --release --workspace, "
                "then run the same manifests"
            ),
            "measure": "python -X utf8 tools/measure_ffv1_flying_field.py",
        },
    }
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    EVIDENCE.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {EVIDENCE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
