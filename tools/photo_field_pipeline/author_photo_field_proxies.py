"""PF1: author the coarse invisible depth-proxy geometry for the photo field.

The committed panorama is a photograph: it carries no usable per-pixel depth, and
deriving one (AI depth estimation, photogrammetry, virtual texturing) is exactly
what PF1 refuses to do. Instead the photographed obstacles are represented by a
handful of coarse boxes and one ground disc, authored offline from measured
angles plus one documented real-world assumption each, and rendered depth-only so
the 3D aircraft is occluded by the photographed building, houses and tree line.

Everything is deterministic: geometry is computed in pure Python float arithmetic
from frozen constants, the tree-ring jitter comes from a small integer LCG (never
the ``random`` module, never the clock), and the GLB writer emits no timestamps.
Re-running the tool must reproduce the committed bytes exactly (``--check``).

Frame: render world space, y-up, ground plane at y = 0, photographic eye at
``PILOT_EYE_RENDER_M``. Every proxy is placed relative to that eye, because the
panorama is only valid from it:

    world = eye + distance * (cos(azimuth), 0, sin(azimuth))

with ``azimuth`` the render-space azimuth (``atan2(z, x)``) that the panorama
samples at, so a box at azimuth A occludes exactly the part of the photograph
that shows A.

Usage:
    python -X utf8 tools/photo_field_pipeline/author_photo_field_proxies.py
    python -X utf8 tools/photo_field_pipeline/author_photo_field_proxies.py --check
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from photo_field_assets import (  # noqa: E402
    EXPECTED_PROXY_TRIANGLES,
    GARAGE_DEPTH_M,
    GROUND_RADIUS_M,
    GROUND_SEGMENTS,
    MAX_PROXY_TRIANGLES,
    NODE_GARAGE,
    NODE_GROUND,
    NODE_TREE_RING,
    NODE_TRUNK_PREFIX,
    OBSTACLE_PLACEMENTS,
    PILOT_EYE_RENDER_M,
    PROXY_NODE_NAMES,
    TRUNK_COUNT,
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
    PhotoFieldError,
    configure_streams,
    depth_proxy_path,
    garage_derivation,
    proxy_origin_from_eye,
    sha256_bytes,
    write_glb,
)
from photo_field_assets import DeterministicLcg, GlbMesh  # noqa: E402

#: Local box frame: X = tangential, Y = up, Z = -radial. That ordering is
#: right-handed (det = +1), so outward-facing winding in local space stays
#: outward-facing in world space.
_UP = (0.0, 1.0, 0.0)

#: (normal axis, normal sign, u axis, v axis) per cube face, chosen so that
#: ``u x v == n``; corners are then emitted (-u-v, +u-v, +u+v, -u+v) and
#: triangulated (0,1,2) (0,2,3), which is counter-clockwise seen from outside.
_BOX_FACES: tuple[tuple[int, int, int, int], ...] = (
    (0, +1, 1, 2),
    (0, -1, 2, 1),
    (1, +1, 2, 0),
    (1, -1, 0, 2),
    (2, +1, 0, 1),
    (2, -1, 1, 0),
)
_CORNER_SIGNS: tuple[tuple[int, int], ...] = ((-1, -1), (+1, -1), (+1, +1), (-1, +1))


def box_geometry(
    azimuth_deg: float,
    distance_m: float,
    width_m: float,
    depth_m: float,
    height_m: float,
    base_y: float = 0.0,
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """A closed 12-triangle box placed relative to the photographic eye.

    ``width_m`` spans the tangential direction, ``depth_m`` the radial one and
    ``height_m`` runs up from ``base_y``. Returns 24 vertices (four per face, so
    each face keeps its own plane) and 36 indices.
    """
    for name, value in (
        ("distance_m", distance_m),
        ("width_m", width_m),
        ("depth_m", depth_m),
        ("height_m", height_m),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise PhotoFieldError(f"box {name} must be a positive finite metre value, got {value}")
    if not math.isfinite(azimuth_deg):
        raise PhotoFieldError(f"box azimuth must be finite, got {azimuth_deg}")

    centre_x, centre_z = proxy_origin_from_eye(azimuth_deg, distance_m)
    azimuth = math.radians(azimuth_deg)
    radial = (math.cos(azimuth), 0.0, math.sin(azimuth))
    tangential = (-radial[2], 0.0, radial[0])
    inward = (-radial[0], -radial[1], -radial[2])
    axes = (tangential, _UP, inward)
    half = (width_m / 2.0, height_m / 2.0, depth_m / 2.0)
    centre = (centre_x, base_y + height_m / 2.0, centre_z)

    positions: list[tuple[float, float, float]] = []
    indices: list[int] = []
    for normal_axis, normal_sign, u_axis, v_axis in _BOX_FACES:
        first = len(positions)
        for u_sign, v_sign in _CORNER_SIGNS:
            offset = [0.0, 0.0, 0.0]
            offset[normal_axis] += normal_sign * half[normal_axis]
            offset[u_axis] += u_sign * half[u_axis]
            offset[v_axis] += v_sign * half[v_axis]
            positions.append(
                (
                    centre[0] + sum(offset[axis] * axes[axis][0] for axis in range(3)),
                    centre[1] + sum(offset[axis] * axes[axis][1] for axis in range(3)),
                    centre[2] + sum(offset[axis] * axes[axis][2] for axis in range(3)),
                )
            )
        indices.extend((first, first + 1, first + 2, first, first + 2, first + 3))
    return positions, indices


def merged_geometry(
    geometries: list[tuple[list, list]],
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """Concatenate box geometries into one mesh, re-basing every index."""
    positions: list[tuple[float, float, float]] = []
    indices: list[int] = []
    for box_positions, box_indices in geometries:
        base = len(positions)
        positions.extend(box_positions)
        indices.extend(base + index for index in box_indices)
    return positions, indices


def ground_disc_geometry(
    centre_x: float, centre_z: float, radius_m: float, segments: int
) -> tuple[list[tuple[float, float, float]], list[int]]:
    """A flat triangle fan at y = 0, wound so its outward normal is +Y.

    Coarse on purpose: at 250 m radius a 64-segment fan has ~24.5 m chords, and
    the disc exists only to give the aircraft a ground plane to intersect, not to
    be silhouetted.
    """
    if radius_m <= 0.0 or segments < 3:
        raise PhotoFieldError(f"invalid ground disc: radius {radius_m}, segments {segments}")
    positions: list[tuple[float, float, float]] = [(centre_x, 0.0, centre_z)]
    for index in range(segments):
        phi = 2.0 * math.pi * index / segments
        positions.append(
            (centre_x + radius_m * math.cos(phi), 0.0, centre_z + radius_m * math.sin(phi))
        )
    indices: list[int] = []
    for index in range(segments):
        current = 1 + index
        following = 1 + (index + 1) % segments
        indices.extend((0, following, current))
    return positions, indices


def build_proxy_meshes() -> list[GlbMesh]:
    """The nine committed depth-proxy meshes, in contract order."""
    meshes: list[GlbMesh] = []

    # 1. the ground plane, centred under the photographic eye.
    positions, indices = ground_disc_geometry(
        PILOT_EYE_RENDER_M[0], PILOT_EYE_RENDER_M[2], GROUND_RADIUS_M, GROUND_SEGMENTS
    )
    meshes.append(GlbMesh(NODE_GROUND, positions, indices))

    # 2. the brick garage: the one obstacle whose metric box is DERIVED from its
    #    measured angular box plus the documented 1.6 m camera height.
    garage = garage_derivation()
    positions, indices = box_geometry(
        azimuth_deg=garage["azimuth_deg"],
        distance_m=garage["distance_m"],
        width_m=garage["width_arc_m"],
        depth_m=GARAGE_DEPTH_M,
        height_m=garage["height_m"],
    )
    meshes.append(GlbMesh(NODE_GARAGE, positions, indices))

    # 3-4. the two houses, MANUALLY CALIBRATED from the perspective crops.
    for name, azimuth_deg, distance_m, width_m, depth_m, height_m in OBSTACLE_PLACEMENTS:
        positions, indices = box_geometry(azimuth_deg, distance_m, width_m, depth_m, height_m)
        meshes.append(GlbMesh(name, positions, indices))

    # 5-8. the near tree trunks.
    if len(TRUNK_PLACEMENTS) != TRUNK_COUNT:
        raise PhotoFieldError(
            f"{len(TRUNK_PLACEMENTS)} trunk placements are registered but the contract "
            f"names {TRUNK_COUNT} trunk nodes"
        )
    for index, (azimuth_deg, distance_m) in enumerate(TRUNK_PLACEMENTS):
        positions, indices = box_geometry(
            azimuth_deg, distance_m, TRUNK_WIDTH_M, TRUNK_DEPTH_M, TRUNK_HEIGHT_M
        )
        meshes.append(GlbMesh(f"{NODE_TRUNK_PREFIX}{index}", positions, indices))

    # 9. the distant tree line: 48 overlapping boxes on a jittered 30 m ring.
    jitter = DeterministicLcg(TREE_RING_JITTER_SEED)
    ring: list[tuple[list, list]] = []
    for index in range(TREE_RING_COUNT):
        azimuth_deg = 360.0 * index / TREE_RING_COUNT
        radius_m = TREE_RING_RADIUS_M + TREE_RING_JITTER_M * jitter.next_symmetric()
        if radius_m <= TREE_RING_DEPTH_M:
            raise PhotoFieldError(
                f"tree ring box {index} jittered to a non-positive radius {radius_m}"
            )
        ring.append(
            box_geometry(
                azimuth_deg, radius_m, TREE_RING_WIDTH_M, TREE_RING_DEPTH_M, TREE_RING_HEIGHT_M
            )
        )
    positions, indices = merged_geometry(ring)
    meshes.append(GlbMesh(NODE_TREE_RING, positions, indices))

    names = [mesh.name for mesh in meshes]
    if names != list(PROXY_NODE_NAMES):
        raise PhotoFieldError(
            f"the authored node names {names} do not match the contract "
            f"{list(PROXY_NODE_NAMES)}"
        )
    triangles = sum(mesh.triangle_count for mesh in meshes)
    if triangles != EXPECTED_PROXY_TRIANGLES:
        raise PhotoFieldError(
            f"authored {triangles} triangles but the contract expects "
            f"{EXPECTED_PROXY_TRIANGLES}; a proxy changed shape"
        )
    if triangles > MAX_PROXY_TRIANGLES:
        raise PhotoFieldError(
            f"{triangles} triangles exceeds the PF1 budget of {MAX_PROXY_TRIANGLES}"
        )
    return meshes


def describe(meshes: list[GlbMesh]) -> None:
    """Print the authored geometry, one line per node (ASCII only)."""
    print(f"photographic eye: {list(PILOT_EYE_RENDER_M)} (render metres, y-up)")
    print(f"{'node':<28} {'triangles':>9} {'vertices':>9}  bounds x/y/z (m)")
    for mesh in meshes:
        minimum, maximum = mesh.bounds()
        print(
            f"{mesh.name:<28} {mesh.triangle_count:>9} {mesh.vertex_count:>9}  "
            f"[{minimum[0]:9.3f},{maximum[0]:9.3f}] "
            f"[{minimum[1]:7.3f},{maximum[1]:7.3f}] "
            f"[{minimum[2]:9.3f},{maximum[2]:9.3f}]"
        )
    total_triangles = sum(mesh.triangle_count for mesh in meshes)
    total_vertices = sum(mesh.vertex_count for mesh in meshes)
    print(
        f"{'total':<28} {total_triangles:>9} {total_vertices:>9}  "
        f"(budget {MAX_PROXY_TRIANGLES} triangles)"
    )


def main(argv: list[str] | None = None) -> int:
    configure_streams()
    parser = argparse.ArgumentParser(
        prog="author_photo_field_proxies.py",
        description=(
            "Author the coarse depth-proxy GLB of the PF1 photo field from the "
            "measured obstacle calibration."
        ),
    )
    parser.add_argument(
        "--out",
        metavar="PATH",
        default=None,
        help=f"destination GLB (default {depth_proxy_path()})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help=(
            "re-author into memory and require byte-identity with the committed "
            "GLB instead of writing it"
        ),
    )
    args = parser.parse_args(argv)

    try:
        destination = (
            pathlib.Path(args.out).resolve() if args.out else depth_proxy_path()
        )
        if args.check and not destination.is_file():
            raise PhotoFieldError(
                f"--check found no committed GLB at {destination}; run without "
                "--check first",
                exit_code=2,
            )

        meshes = build_proxy_meshes()
        print("PF1 depth-proxy authoring")
        describe(meshes)

        payload = write_glb(meshes)
        digest = sha256_bytes(payload)

        if args.check:
            committed = destination.read_bytes()
            if committed != payload:
                print(
                    f"FAIL: re-authoring is NOT byte-identical to {destination.name} "
                    f"(committed sha256 {sha256_bytes(committed)}, re-authored {digest})",
                    file=sys.stderr,
                )
                return 1
            print(f"\n--check: {destination.name} is byte-identical ({len(payload)} bytes)")
            print(f"sha256 {digest}")
            return 0

        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        print(f"\nwrote {destination}")
        print(f"  byte_size {len(payload)}")
        print(f"  sha256    {digest}")
        print(
            f"  triangles {sum(mesh.triangle_count for mesh in meshes)} "
            f"(budget {MAX_PROXY_TRIANGLES})"
        )
        print(
            "  nodes are world-space with an explicit identity TRS: the repository's "
            "production GLB loader reads baked positions and does not apply node "
            "transforms, so the two readings agree"
        )
        return 0
    except PhotoFieldError as error:
        print(f"error: {error.message}", file=sys.stderr)
        return error.exit_code


if __name__ == "__main__":
    sys.exit(main())
