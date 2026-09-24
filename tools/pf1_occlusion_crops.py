"""PF1: deterministic occlusion-evidence crops from a hero capture.

The mandatory PF1 acceptance criterion is one photographed obstacle with a
corresponding invisible proxy, demonstrated with the aircraft (A) nearer to the
pilot than the proxy and (B) farther. Because the RC aircraft is small (~1.3 m
wingspan, ~3.7 deg at 20 m) a single deterministic parked-aircraft capture can
show both at once: the aircraft sits IN FRONT of the photographed brick garage
(proxy at 25.1 m) and BEHIND a photographed near tree trunk (proxy at ~5 m).

This tool projects the two proxy centres through the exact pilot camera
(fixed eye, look-at the aircraft spawn at the render origin, vertical FOV 55)
and cuts zoomed crops around them from the captured PNG, so a reviewer can see
the occlusion relationship without guessing where to look. Nothing is inferred:
the projection is the same pinhole math the renderer's PilotCamera uses.

Usage:
    python -X utf8 tools/pf1_occlusion_crops.py --png PATH [--out DIR]
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np
from PIL import Image

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

PILOT_EYE = np.array([15.321, 1.6, -12.856])
AIRCRAFT_SPAWN = np.array([0.0, 0.0, 0.0])
VERTICAL_FOV_DEG = 55.0
WORLD_UP = np.array([0.0, 1.0, 0.0])

# Obstacles as authored by tools/photo_field_pipeline/author_photo_field_proxies.py:
# (name, azimuth deg from the photographic eye, distance m, height m).
OBSTACLES = (
    ("building_brick_garage", 150.87, 25.1, 4.91),
    ("tree_trunk_0", 152.0, 5.0, 22.0),
)


def camera_basis() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = AIRCRAFT_SPAWN - PILOT_EYE
    forward = forward / np.linalg.norm(forward)
    right = np.cross(forward, WORLD_UP)
    right = right / np.linalg.norm(right)
    up = np.cross(right, forward)
    return forward, right, up


def project(point: np.ndarray, width: int, height: int) -> tuple[float, float, float]:
    forward, right, up = camera_basis()
    d = point - PILOT_EYE
    depth = float(np.dot(d, forward))
    tan_v = math.tan(math.radians(VERTICAL_FOV_DEG) / 2.0)
    aspect = width / height
    x = float(np.dot(d, right)) / (depth * tan_v * aspect)
    y = float(np.dot(d, up)) / (depth * tan_v)
    u = (x * 0.5 + 0.5) * width
    v = (0.5 - y * 0.5) * height
    return u, v, depth


def main() -> int:
    parser = argparse.ArgumentParser(prog="pf1_occlusion_crops.py")
    parser.add_argument("--png", required=True, type=pathlib.Path)
    parser.add_argument("--out", type=pathlib.Path, default=None)
    args = parser.parse_args()

    png: pathlib.Path = args.png
    if not png.is_file():
        print(f"error: {png} not found", file=sys.stderr)
        return 2
    out_dir = args.out or (REPO_ROOT / "tmp" / "pf1_occlusion_crops")
    out_dir.mkdir(parents=True, exist_ok=True)

    with Image.open(png) as im:
        im.load()
        frame = np.asarray(im, dtype=np.uint8)
    height, width = frame.shape[:2]
    print(f"capture {png.name}: {width}x{height}")

    record = {"capture": str(png), "framebuffer": [width, height],
              "pilot_eye_render_m": PILOT_EYE.tolist(), "obstacles": []}
    for name, azimuth_deg, distance_m, height_m in OBSTACLES:
        azimuth = math.radians(azimuth_deg)
        centre = PILOT_EYE + distance_m * np.array(
            [math.cos(azimuth), 0.0, math.sin(azimuth)]
        )
        top = centre + np.array([0.0, height_m * 0.5, 0.0])
        u, v, depth = project(centre, width, height)
        _, v_top, _ = project(top, width, height)
        half = max(abs(v - v_top), 24.0) * 1.6
        x0 = int(max(0, u - half))
        x1 = int(min(width, u + half))
        y0 = int(max(0, v - half))
        y1 = int(min(height, v + half))
        crop = frame[y0:y1, x0:x1]
        dest = out_dir / f"occlusion_{name}.png"
        Image.fromarray(crop).save(dest)
        aircraft_depth = float(np.linalg.norm(AIRCRAFT_SPAWN - PILOT_EYE))
        record["obstacles"].append({
            "name": name,
            "proxy_distance_from_eye_m": distance_m,
            "aircraft_distance_from_eye_m": round(aircraft_depth, 3),
            "aircraft_in_front_of_proxy": aircraft_depth < distance_m,
            "projected_pixel": [round(u, 1), round(v, 1)],
            "crop": str(dest.relative_to(REPO_ROOT).as_posix()),
        })
        print(f"  {name}: proxy {distance_m} m, aircraft {aircraft_depth:.1f} m -> "
              f"{'AIRCRAFT IN FRONT' if aircraft_depth < distance_m else 'AIRCRAFT BEHIND'}"
              f"  crop {dest.name}")

    # The aircraft-centred crop is the one a reviewer reads first: it shows the
    # 3D aircraft against the photographed backdrop it stands in front of.
    u, v, depth = project(AIRCRAFT_SPAWN, width, height)
    half = 150.0
    x0 = int(max(0, u - half))
    x1 = int(min(width, u + half))
    y0 = int(max(0, v - half))
    y1 = int(min(height, v + half))
    dest = out_dir / "occlusion_aircraft.png"
    Image.fromarray(frame[y0:y1, x0:x1]).save(dest)
    record["aircraft"] = {
        "position_render_m": AIRCRAFT_SPAWN.tolist(),
        "distance_from_eye_m": round(depth, 3),
        "projected_pixel": [round(u, 1), round(v, 1)],
        "crop": str(dest.relative_to(REPO_ROOT).as_posix()),
    }
    print(f"  aircraft: {depth:.1f} m  crop {dest.name}")

    dest = out_dir / "occlusion_evidence.json"
    dest.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
