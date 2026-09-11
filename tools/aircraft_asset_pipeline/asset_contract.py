#!/usr/bin/env python3
"""
Shared asset-contract helpers for the RV2-7A aircraft asset pipeline.

Used by both the Blender-side scripts (which run inside Blender's bundled
Python) and `validate_glb.py` (which runs under the system Python). Deliberately
standard-library only so it imports anywhere without a dependency stack.

The single authority for what an aircraft asset must contain is the per-asset
manifest JSON; this module only knows how to read it and how to convert between
the runtime render-body frame and Blender's frame.
"""

from __future__ import annotations

import json
import os

PIPELINE_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(PIPELINE_DIR))

DEFAULT_ASSET_ID = "acro_electric_01"
SEMANTIC_ID_PATTERN = "^[A-Z][A-Z0-9]*(_[A-Z0-9]+)*$"

# Families that must never be merged into a single object with a moving surface.
CONTROL_SURFACE_FAMILY = "control_surface"


def manifest_path(asset_id: str = DEFAULT_ASSET_ID) -> str:
    return os.path.join(PIPELINE_DIR, f"{asset_id}_manifest.json")


def repo_relative(path: str) -> str:
    """Resolve a manifest-relative path against the repository root."""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(REPO_ROOT, path))


def load_manifest(asset_id: str = DEFAULT_ASSET_ID) -> dict:
    path = manifest_path(asset_id)
    with open(path, "r", encoding="utf-8") as handle:
        manifest = json.load(handle)
    if manifest.get("asset_id") != asset_id:
        raise ValueError(
            f"manifest at {path} declares asset_id {manifest.get('asset_id')!r}, expected {asset_id!r}"
        )
    return manifest


def components(manifest: dict) -> list[dict]:
    """Components ordered by primitive index."""
    return sorted(manifest.get("components", []), key=lambda c: c["primitive_index"])


def component_by_semantic_id(manifest: dict) -> dict[str, dict]:
    return {c["semantic_id"]: c for c in components(manifest)}


def component_by_legacy_name(manifest: dict) -> dict[str, dict]:
    return {c["legacy_node_name"]: c for c in components(manifest)}


def moving_surfaces(manifest: dict) -> list[dict]:
    return list(manifest.get("moving_surfaces", []))


def material_catalog(manifest: dict) -> dict[str, dict]:
    return {m["name"]: m for m in manifest.get("materials", {}).get("catalog", [])}


def export_node_name(component: dict) -> str:
    """The glTF node name the production export must emit for this component.

    Blender's glTF exporter orders nodes (and therefore mesh indices) by object
    name, so the required primitive index is pinned into the name itself rather
    than inherited from an unpredictable traversal order.
    """
    return f"{component['primitive_index']:02d}_{component['semantic_id']}"


# ---------------------------------------------------------------------------
# frame conversion
# ---------------------------------------------------------------------------
# runtime render-body:  +X right, +Y up,    -Z forward
# Blender:              +X right, +Z up,    -Y forward
# glTF (export_yup):    +X right, +Y up,    -Z forward   (== render-body)
#
# Verified empirically on Blender 5.2.1 LTS: a Blender object at (2, 3, 4)
# exports to glTF translation (2, 4, -3).

def render_body_to_blender(point) -> tuple[float, float, float]:
    x, y, z = point
    return (x, -z, y)


def blender_to_render_body(point) -> tuple[float, float, float]:
    x, y, z = point
    return (x, z, -y)


def blender_bounds_to_render_body(corners) -> tuple[list[float], list[float]]:
    """Convert an iterable of Blender-space corners to render-body min/max."""
    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3
    for corner in corners:
        point = blender_to_render_body(corner)
        for axis in range(3):
            value = point[axis]
            if value < minimum[axis]:
                minimum[axis] = value
            if value > maximum[axis]:
                maximum[axis] = value
    return minimum, maximum


def check_orientation(minimum, maximum, spec: dict, bounds_by_semantic: dict) -> list[str]:
    """Run the manifest orientation convention rules in the render-body frame.

    Returns a list of human-readable failure strings (empty == pass). Shared by
    the Blender source validator and any offline consumer so both frames are
    held to the same convention.
    """
    failures: list[str] = []
    tolerance = spec.get("tolerance_m", 0.002)

    def bounds_of(semantic_id):
        return bounds_by_semantic.get(semantic_id)

    foremost = spec.get("foremost_component")
    if foremost:
        got = bounds_of(foremost)
        if got is None:
            failures.append(f"orientation: foremost component {foremost!r} has no bounds")
        else:
            global_min_z = min(b[0][2] for b in bounds_by_semantic.values())
            if got[0][2] - global_min_z > tolerance:
                failures.append(
                    f"orientation 'nose_forward_minus_z': {foremost} min Z {got[0][2]:.6f} is not "
                    f"the global minimum Z {global_min_z:.6f}"
                )

    up = spec.get("up_reference") or {}
    if up.get("above") and up.get("below"):
        high, low = bounds_of(up["above"]), bounds_of(up["below"])
        if high is None or low is None:
            failures.append("orientation 'up_is_plus_y': missing component bounds")
        elif high[1][1] <= low[1][1]:
            failures.append(
                f"orientation 'up_is_plus_y': {up['above']} max Y {high[1][1]:.6f} is not above "
                f"{up['below']} max Y {low[1][1]:.6f}"
            )

    forward = spec.get("forward_reference") or {}
    if forward.get("front") and forward.get("behind"):
        front, behind = bounds_of(forward["front"]), bounds_of(forward["behind"])
        if front is None or behind is None:
            failures.append("orientation 'forward_reference': missing component bounds")
        elif front[0][2] >= behind[0][2]:
            failures.append(
                f"orientation 'forward_reference': {forward['front']} min Z {front[0][2]:.6f} is "
                f"not forward of {forward['behind']} min Z {behind[0][2]:.6f}"
            )

    if spec.get("mirror_pairs_by_suffix"):
        for semantic_id in sorted(bounds_by_semantic):
            if not semantic_id.endswith("_L"):
                continue
            sibling = semantic_id[:-2] + "_R"
            if sibling not in bounds_by_semantic:
                failures.append(f"orientation 'mirror_pairs': {semantic_id} has no {sibling} sibling")
                continue
            left, right = bounds_by_semantic[semantic_id], bounds_by_semantic[sibling]
            if left[1][0] >= right[0][0]:
                failures.append(
                    f"orientation 'mirror_pairs': {semantic_id} max X {left[1][0]:.6f} is not left "
                    f"of {sibling} min X {right[0][0]:.6f}; +X must be aircraft right"
                )
            for bound, other in ((0, 1), (1, 0)):
                want = -left[bound][0]
                got = right[other][0]
                if abs(got - want) > tolerance:
                    failures.append(
                        f"orientation 'mirror_pairs': {semantic_id}/{sibling} are not mirrored "
                        f"about X=0 ({want:.6f} expected, {got:.6f} measured)"
                    )
            for axis in (1, 2):
                for bound, label in ((0, "min"), (1, "max")):
                    if abs(left[bound][axis] - right[bound][axis]) > tolerance:
                        failures.append(
                            f"orientation 'mirror_pairs': {semantic_id}/{sibling} differ on "
                            f"{label} {'XYZ'[axis]} ({left[bound][axis]:.6f} vs "
                            f"{right[bound][axis]:.6f})"
                        )
    return failures
