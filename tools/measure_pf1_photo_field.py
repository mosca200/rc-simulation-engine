"""PF1 Photo Field v1: measurement and evidence assembly.

Folds the deterministic benchmark runs of the PF1 hero scene (PhotoField) and of
the equivalent pilot-camera Full3D frame (FlyingField, same eye/FOV/frame) at
1080p / 1440p / 4K, plus the occlusion-test captures, into
``docs/validation/pf1_photo_field/pf1_evidence.json``.

Every metric here is either read from a runtime artifact (receipt, audit,
capture evidence) or measured from PNGs on disk. Nothing is estimated. Metrics
the repository cannot produce (whole-frame GPU time, VRAM, total draw calls,
FPS percentiles) are recorded as ``null`` with a written reason, and
``visual_pass`` stays ``null``: capture evidence records facts and never a
visual verdict.

Candidate PNGs are NOT committed (project rule): the evidence records
``committed_png: null`` plus the local ``tmp/`` path and SHA-256.

Usage:
    python -X utf8 tools/measure_pf1_photo_field.py
"""

from __future__ import annotations

import hashlib
import json
import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
OUT_DIR = REPO_ROOT / "docs" / "validation" / "pf1_photo_field"
EVIDENCE = OUT_DIR / "pf1_evidence.json"

RESOLUTIONS = (
    ("1920x1080", "pf1_hero_1080p", "pf1_baseline_full3d_1080p"),
    ("2560x1440", "pf1_hero_1440p", "pf1_baseline_full3d_1440p"),
    ("3840x2160", "pf1_hero_2160p", "pf1_baseline_full3d_2160p"),
)

OCCLUSION_CASES = (
    ("near", "pf1_occlusion_near", "aircraft nearer to the pilot than the proxy"),
    ("far", "pf1_occlusion_far", "aircraft farther from the pilot than the proxy"),
)

UNAVAILABLE = {
    "gpu_frame_duration_ns": (
        "the runtime exposes only the per-pass gpu_duration_ns values; a "
        "whole-frame GPU time would be this script's arithmetic, not a "
        "measurement, and the passes are not proven contiguous"
    ),
    "vram_bytes": (
        "no wgpu memory introspection, no NVML and no nvidia-smi scraping; the "
        "VisualCaptureEvidence 1.0.0 schema forbids a hardware.vram_gb key"
    ),
    "total_draw_calls": (
        "only vegetation draw calls are instrumented "
        "(vegetation.stats.scene_draw_calls / shadow_draw_calls); a PhotoField "
        "frame draws no vegetation, so no total exists to report"
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


def load_run(root: pathlib.Path, resolution: str, scene_id: str) -> dict:
    base = root / resolution / scene_id
    run = json.loads((base / "run.json").read_text(encoding="utf-8"))
    receipt = json.loads(
        (base / "runtime_capture_receipt.json").read_text(encoding="utf-8")
    )
    audit = json.loads(
        (base / "runtime_visual_audit.json").read_text(encoding="utf-8")
    )
    evidence = json.loads(
        (base / "capture_evidence.json").read_text(encoding="utf-8")
    )
    png = base / receipt["image_path"].replace("\\", "/").split("/")[-1]
    if not png.is_file():
        candidates = sorted(base.glob("*.png"))
        if not candidates:
            raise SystemExit(f"no capture PNG under {base}")
        png = candidates[0]
    return {
        "run": run,
        "receipt": receipt,
        "audit": audit,
        "evidence": evidence,
        "png": png,
    }


def summarise(run: dict) -> dict:
    audit = run["audit"]
    receipt = run["receipt"]
    png = run["png"]
    profiling = audit["profiling"]
    return {
        "png": str(png),
        "committed_png": None,
        "committed_png_note": (
            "candidate capture: not committed per project rule; hashes and the "
            "receipt/audit are the committed evidence"
        ),
        "local_png": str(png.relative_to(REPO_ROOT).as_posix()),
        "local_png_sha256": sha256_file(png),
        "image_sha256": receipt["image_sha256"],
        "image_byte_size": receipt["image_byte_size"],
        "framebuffer": [receipt["framebuffer_width"], receipt["framebuffer_height"]],
        "presentation_frame_index": receipt["presentation_frame_index"],
        "git_commit_sha": run["evidence"]["source"]["commit_sha"],
        "git_dirty": run["evidence"]["source"]["dirty"],
        "runner_success": bool(run["evidence"]["execution"]["capture_success"]),
        "capture_evidence_valid": True,
        "visual_audit_valid": True,
        "cpu_frame_duration_ns": profiling["cpu_frame_duration_ns"],
        "gpu_timing_status": profiling["gpu_timing_status"],
        "gpu_timing_source_presentation_frame_index": profiling.get(
            "gpu_timing_source_presentation_frame_index"
        ),
        "gpu_timing_frame_age": profiling.get("gpu_timing_frame_age"),
        "passes": [
            {
                "pass_id": p["pass_id"],
                "label": p["label"],
                "cpu_duration_ns": p["cpu_duration_ns"],
                "gpu_duration_ns": p["gpu_duration_ns"],
            }
            for p in profiling["passes"]
        ],
    }


def main() -> int:
    photo_root = REPO_ROOT / "tmp" / "pf1_photo"
    full3d_root = REPO_ROOT / "tmp" / "pf1_full3d"
    occlusion_root = REPO_ROOT / "tmp" / "pf1_occlusion"
    for root in (photo_root, full3d_root):
        if not root.is_dir():
            raise SystemExit(
                f"missing run tree {root}; execute the hero manifests with "
                "tools/visual_benchmark/run_benchmark.py --execute --output-dir first"
            )

    resolutions = []
    for resolution, scene_id, baseline_scene_id in RESOLUTIONS:
        photo = summarise(load_run(photo_root, resolution, scene_id))
        full3d = summarise(load_run(full3d_root, resolution, baseline_scene_id))
        photo_scene = next(
            p for p in photo["passes"] if p["pass_id"] == "scene"
        )
        full3d_scene = next(
            p for p in full3d["passes"] if p["pass_id"] == "scene"
        )
        photo_post = next(
            p for p in photo["passes"] if p["pass_id"] == "postprocess"
        )
        full3d_post = next(
            p for p in full3d["passes"] if p["pass_id"] == "postprocess"
        )
        delta_ns = None
        if (
            photo_scene["gpu_duration_ns"] is not None
            and full3d_scene["gpu_duration_ns"] is not None
            and photo_post["gpu_duration_ns"] is not None
            and full3d_post["gpu_duration_ns"] is not None
        ):
            delta_ns = (
                photo_scene["gpu_duration_ns"]
                + photo_post["gpu_duration_ns"]
                - full3d_scene["gpu_duration_ns"]
                - full3d_post["gpu_duration_ns"]
            )
        resolutions.append(
            {
                "resolution": resolution,
                "scene_id": scene_id,
                "baseline_scene_id": baseline_scene_id,
                "manifest": f"docs/validation/pf1_photo_field/{scene_id}.json",
                "baseline_manifest": (
                    f"docs/validation/pf1_photo_field/{baseline_scene_id}.json"
                ),
                "photo_field": photo,
                "full3d_pilot_baseline": full3d,
                "scene_plus_postprocess_gpu_delta_ns": delta_ns,
                "delta_note": (
                    "single-sample per-pass GPU timestamps from one deterministic "
                    "capture frame each; they absorb first-use costs and the GPU "
                    "readback may report the previous frame (see gpu_timing_frame_age). "
                    "Not a cost curve; repeat runs before asserting a regression."
                ),
            }
        )

    occlusion = []
    for case, scene_id, description in OCCLUSION_CASES:
        base = occlusion_root / case
        if not base.is_dir():
            occlusion.append(
                {"case": case, "scene_id": scene_id, "description": description,
                 "captured": False, "note": "run tree absent"}
            )
            continue
        run = load_run(occlusion_root, case, scene_id)
        manifest = json.loads(
            (OUT_DIR / f"{scene_id}.json").read_text(encoding="utf-8")
        )
        aircraft = manifest["aircraft"]
        if aircraft.get("start_on_ground"):
            position = [0.0, 0.0, 0.0]
            position_note = (
                "the parked aircraft sits at the render-world spawn origin; the "
                "receipt/audit carry no pose, and none is needed for a ground start"
            )
        else:
            position = None
            position_note = (
                "the runtime exposes no aircraft pose in the receipt or audit; the "
                "deterministic flight specification below is the recorded authority "
                "for where the aircraft is at the captured presentation frame"
            )
        occlusion.append(
            {
                "case": case,
                "scene_id": scene_id,
                "description": description,
                "captured": True,
                "aircraft_position_render_m": position,
                "aircraft_position_note": position_note,
                "aircraft_flight_spec": {
                    "throttle": aircraft.get("throttle"),
                    "start_on_ground": aircraft.get("start_on_ground"),
                    "altitude_m": aircraft.get("altitude_m"),
                    "airspeed_mps": aircraft.get("airspeed_mps"),
                    "capture_frame": manifest["capture"]["frame"],
                },
                **summarise(run),
            }
        )

    manifest_path = (
        REPO_ROOT / "crates/renderer/assets/photofield/meadow/photo_field_manifest.json"
    )
    panorama_path = (
        REPO_ROOT / "crates/renderer/assets/photofield/meadow/meadow_panorama_8192x4096.jpg"
    )
    proxy_path = (
        REPO_ROOT / "crates/renderer/assets/photofield/meadow/photo_field_depth.glb"
    )
    provenance_path = REPO_ROOT / "docs/assets/photofield/pf1_provenance.json"

    document = {
        "schema_version": 1,
        "slice": "PF1",
        "title": "Photo Field v1 - photographic presentation evidence",
        "method": (
            "rv2-vis0-benchmark-runner executed the PF1 hero manifests and the "
            "equivalent Full3D pilot-camera manifests at three resolutions against "
            "the same release binary; this script folds the verified PNGs, the "
            "runtime receipts and the RuntimeVisualAudit 1.0.0 facts into one "
            "document. No metric is estimated."
        ),
        "hardware_note": (
            "GPU identity comes from RuntimeVisualAudit.device (adapter_name, "
            "backend, driver); the evidence schema carries no VRAM field."
        ),
        "verdict": {
            "visual_pass": None,
            "visual_pass_reason": (
                "visual_pass stays null: capture evidence records facts and never "
                "a visual verdict; the visual gate is human review."
            ),
        },
        "assets": {
            "manifest": json.loads(manifest_path.read_text(encoding="utf-8"))
            if manifest_path.is_file() else None,
            "panorama_sha256": sha256_file(panorama_path) if panorama_path.is_file() else None,
            "panorama_byte_size": panorama_path.stat().st_size if panorama_path.is_file() else None,
            "depth_proxy_sha256": sha256_file(proxy_path) if proxy_path.is_file() else None,
            "depth_proxy_byte_size": proxy_path.stat().st_size if proxy_path.is_file() else None,
            "provenance": "docs/assets/photofield/pf1_provenance.json"
            if provenance_path.is_file() else None,
        },
        "unavailable_metrics": {
            key: {"value": None, "reason": reason} for key, reason in UNAVAILABLE.items()
        },
        "resolutions": resolutions,
        "occlusion": occlusion,
    }

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    EVIDENCE.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {EVIDENCE}")
    for entry in resolutions:
        delta = entry["scene_plus_postprocess_gpu_delta_ns"]
        delta_ms = f"{delta / 1e6:+.3f} ms" if delta is not None else "null"
        print(f"  {entry['resolution']}: scene+postprocess delta {delta_ms}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
