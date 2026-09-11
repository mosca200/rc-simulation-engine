#!/usr/bin/env python3
"""
RV2-7A Blender source validator for RC Simulation Engine aircraft assets.

Runs inside Blender's bundled Python against an open .blend source file and
fails closed (non-zero exit) on any contract violation. It never mutates the
file and never exports anything.

Invocation:
    blender --background models/acro_electric_01/source/acro_electric_01.blend \
        --python-exit-code 1 \
        --python tools/aircraft_asset_pipeline/blender_validate_source.py \
        -- --asset-id acro_electric_01

The checks cover, at minimum:
  * unit system must be metric with a 1.0 scale length and metre length unit
  * non-finite object transforms
  * negative / unapplied / non-unit pathological scale
  * non-identity rotation (objects must be axis-aligned with the render-body frame)
  * missing required semantic object
  * duplicate semantic object
  * undeclared mesh objects (geometry the manifest does not know about)
  * a moving surface merged with, instanced by, or sharing a mesh with another object
  * missing UV map where the manifest's future production contract requires one
  * missing / unexpected / multi-slot material assignment
  * zero-vertex, zero-polygon or zero-area meshes
  * absurd bounds (unit-system or unapplied-scale symptom)
  * wrong orientation convention (+X right, +Y up, -Z forward)
  * required production meshes hidden or excluded from the export
  * modifiers or shape keys that would make the export ambiguous

Pivots are handled deliberately: the source keeps authoring origins, so this
validator does NOT demand Apply Transform. It demands unit scale, no rotation,
and an origin that matches the pivot the manifest declares.
"""

from __future__ import annotations

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import asset_contract as ac  # noqa: E402

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402

TRANSFORM_TOLERANCE = 1.0e-6
PIVOT_TOLERANCE = 1.0e-6
ZERO_AREA_EPSILON = 1.0e-9


class SourceReport:
    def __init__(self, subject: str):
        self.subject = subject
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.info: list[str] = []
        self.summary: dict = {}

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)

    def note(self, message: str) -> None:
        self.info.append(message)

    @property
    def ok(self) -> bool:
        return not self.errors

    def render(self) -> str:
        lines = [f"blender_validate_source: {self.subject}"]
        for key in sorted(self.summary):
            lines.append(f"  {key}: {self.summary[key]}")
        for message in self.info:
            lines.append(f"  [info] {message}")
        for message in self.warnings:
            lines.append(f"  [warn] {message}")
        for message in self.errors:
            lines.append(f"  [FAIL] {message}")
        lines.append(
            f"blender_validate_source: {'PASS' if self.ok else 'FAIL'} "
            f"({len(self.errors)} error(s), {len(self.warnings)} warning(s))"
        )
        return "\n".join(lines)


def _is_finite_vector(values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _world_corners(ob) -> list[tuple[float, float, float]]:
    return [tuple(ob.matrix_world @ Vector(corner)) for corner in ob.bound_box]


def _visible_in_export(ob, scene) -> tuple[bool, list[str]]:
    """A production mesh must be visible, renderable and linked into the exported scene."""
    reasons: list[str] = []
    if ob.hide_get():
        reasons.append("hide_get() is True (hidden in the viewport)")
    if ob.hide_viewport:
        reasons.append("hide_viewport is True")
    if ob.hide_render:
        reasons.append("hide_render is True")
    linked = any(
        ob.name in collection.objects for collection in _all_collections(scene.collection)
    )
    if not linked:
        reasons.append("not linked into any collection of the exported scene")
    excluded = _excluded_collections(ob, scene)
    if excluded:
        reasons.append(f"inside collection(s) excluded from the view layer: {excluded}")
    if not ob.visible_get():
        reasons.append("visible_get() is False")
    return (not reasons), reasons


def _all_collections(collection):
    yield collection
    for child in collection.children:
        yield from _all_collections(child)


def _excluded_collections(ob, scene) -> list[str]:
    excluded = []
    for view_layer in scene.view_layers:
        for collection in _collections_of(ob):
            layer_collection = _find_layer_collection(view_layer.layer_collection, collection)
            if layer_collection is not None and layer_collection.exclude:
                excluded.append(f"{view_layer.name}:{collection.name}")
    return excluded


def _collections_of(ob) -> list:
    return [collection for collection in bpy.data.collections if ob.name in collection.objects]


def _find_layer_collection(layer_collection, collection):
    if layer_collection.collection == collection:
        return layer_collection
    for child in layer_collection.children:
        found = _find_layer_collection(child, collection)
        if found is not None:
            return found
    return None


def validate_source(manifest: dict, report: SourceReport) -> None:
    root_spec = manifest.get("root_object", {})
    root_name = root_spec.get("blender_name", "ACRO_ROOT")
    components = ac.components(manifest)
    by_semantic = {c["semantic_id"]: c for c in components}
    catalog = ac.material_catalog(manifest)
    surfaces = {s["semantic_id"]: s for s in ac.moving_surfaces(manifest)}
    contract = manifest.get("attribute_contract", {})
    texcoord_required = bool(contract.get("texcoord_0_required", False))
    bounds_spec = manifest.get("bounds", {})
    tolerance = bounds_spec.get("tolerance_m", 0.002)
    absurd = bounds_spec.get("absurd_extent_limit_m", 5.0)

    # --- scene / unit system ----------------------------------------------
    scenes = list(bpy.data.scenes)
    if len(scenes) != 1:
        report.error(
            f"expected exactly one scene, found {len(scenes)}: {[s.name for s in scenes]}"
        )
    if not scenes:
        return
    scene = scenes[0]
    report.summary["scene"] = scene.name
    report.summary["blender_version"] = bpy.app.version_string

    units = scene.unit_settings
    if units.system != "METRIC":
        report.error(f"scene unit system is {units.system!r}, must be METRIC")
    if abs(units.scale_length - 1.0) > TRANSFORM_TOLERANCE:
        report.error(
            f"scene unit scale_length is {units.scale_length}, must be 1.0 "
            f"(1 Blender unit = 1 metre)"
        )
    if units.length_unit != "METERS":
        report.error(f"scene length_unit is {units.length_unit!r}, must be METERS")
    report.note(
        f"units: system={units.system} scale_length={units.scale_length} length_unit={units.length_unit}"
    )

    # --- root object -------------------------------------------------------
    root = bpy.data.objects.get(root_name)
    if root is None:
        report.error(f"missing required root object {root_name!r}")
    else:
        if root.type != "EMPTY":
            report.error(f"root object {root_name!r} must be an EMPTY, is {root.type!r}")
        for row in root.matrix_world:
            for value in row:
                if not math.isfinite(value):
                    report.error(f"root object {root_name!r} has a non-finite matrix_world entry")
                    break
        identity = tuple(tuple(round(v, 6) for v in row) for row in root.matrix_world)
        expected_identity = ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0),
                             (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        if identity != expected_identity:
            report.error(
                f"root object {root_name!r} must have an identity world transform, got {identity}"
            )
        if root.parent is not None:
            report.error(f"root object {root_name!r} must not be parented to {root.parent.name!r}")

    # --- object inventory --------------------------------------------------
    declared_names = set(by_semantic)
    mesh_objects = [ob for ob in bpy.data.objects if ob.type == "MESH"]
    other_objects = [
        ob for ob in bpy.data.objects if ob.type not in ("MESH",) and ob.name != root_name
    ]
    for ob in other_objects:
        report.error(
            f"unexpected non-mesh object {ob.name!r} of type {ob.type!r}; the source may only "
            f"contain {root_name!r} plus the declared semantic mesh objects"
        )

    seen_names: dict[str, list[str]] = {}
    for ob in mesh_objects:
        seen_names.setdefault(ob.name, []).append(ob.name)
    unknown = sorted({ob.name for ob in mesh_objects} - declared_names)
    for name in unknown:
        report.error(
            f"undeclared mesh object {name!r} is not a semantic id in the manifest; production "
            f"geometry must be declared before it can be exported"
        )
    missing = sorted(declared_names - {ob.name for ob in mesh_objects})
    for name in missing:
        report.error(f"missing required semantic object {name!r}")

    report.summary["declared_components"] = len(components)
    report.summary["mesh_objects"] = len(mesh_objects)

    # --- per-object contract ----------------------------------------------
    render_bounds_by_semantic: dict[str, tuple[list[float], list[float]]] = {}
    # Collected over every mesh object, declared or not, so a moving surface that
    # is instanced or linked-duplicated against unknown geometry is still caught.
    mesh_users: dict[str, list[str]] = {}
    for ob in mesh_objects:
        if ob.data is not None:
            mesh_users.setdefault(ob.data.name, []).append(ob.name)
    for ob in mesh_objects:
        if ob.name in unknown:
            continue
        component = by_semantic.get(ob.name)
        if component is None:
            continue
        semantic_id = component["semantic_id"]
        surface = surfaces.get(semantic_id)
        me = ob.data

        # visibility / export inclusion
        visible, reasons = _visible_in_export(ob, scene)
        if not visible:
            report.error(
                f"{semantic_id}: production mesh would be excluded from the export - "
                + "; ".join(reasons)
            )

        # hierarchy
        if root is not None and ob.parent is not root:
            parent_name = ob.parent.name if ob.parent else None
            report.error(
                f"{semantic_id}: must be a direct child of {root_name!r}, parent is {parent_name!r}"
            )

        # transforms
        flat = [value for row in ob.matrix_world for value in row]
        if not _is_finite_vector(flat):
            report.error(f"{semantic_id}: matrix_world contains a non-finite value")
            continue
        location = tuple(ob.location)
        rotation = tuple(ob.rotation_euler)
        scale = tuple(ob.scale)
        if not _is_finite_vector(location + rotation + scale):
            report.error(f"{semantic_id}: location/rotation/scale contains a non-finite value")
            continue

        for axis, value in enumerate(scale):
            if value <= 0.0:
                report.error(
                    f"{semantic_id}: scale[{axis}] is {value}; negative or zero scale mirrors or "
                    f"collapses geometry and flips normals"
                )
            elif abs(value - 1.0) > TRANSFORM_TOLERANCE:
                report.error(
                    f"{semantic_id}: scale[{axis}] is {value}; unapplied scale must be exactly 1.0 "
                    f"so exported metres are real metres"
                )
        for axis, value in enumerate(rotation):
            if abs(value) > TRANSFORM_TOLERANCE:
                report.error(
                    f"{semantic_id}: rotation_euler[{axis}] is {value}; objects must be "
                    f"axis-aligned with the render-body frame (bake the rotation into the mesh)"
                )

        expected_pivot = component.get("blender_pivot", [0.0, 0.0, 0.0])
        if surface is not None:
            expected_pivot = surface.get("blender_pivot", expected_pivot)
        pivot_tolerance = PIVOT_TOLERANCE
        if surface is not None:
            pivot_tolerance = surface.get("hinge_origin_tolerance_m", PIVOT_TOLERANCE)
        for axis in range(3):
            if abs(location[axis] - expected_pivot[axis]) > pivot_tolerance:
                label = "authoring pivot / hinge origin" if surface else "origin"
                report.error(
                    f"{semantic_id}: {label} on axis {'XYZ'[axis]} is {location[axis]:.9f} but the "
                    f"manifest declares {expected_pivot[axis]:.9f} (tolerance {pivot_tolerance})"
                )
        for axis in range(3):
            if abs(location[axis]) > absurd:
                report.error(
                    f"{semantic_id}: origin {location[axis]:.3f} m on axis {'XYZ'[axis]} leaves the "
                    f"{absurd} m sanity box"
                )

        # modifiers / shape keys would make the export ambiguous
        for modifier in ob.modifiers:
            report.error(
                f"{semantic_id}: modifier {modifier.name!r} ({modifier.type}) is not permitted; the "
                f"production export does not apply modifiers, so the mesh data must already be final"
            )
        if me.shape_keys is not None and len(me.shape_keys.key_blocks) > 1:
            report.error(
                f"{semantic_id}: mesh carries {len(me.shape_keys.key_blocks)} shape keys; the "
                f"runtime consumes a single static primitive per component"
            )
        if ob.parent is not None and ob.parent.type == "ARMATURE":
            report.error(f"{semantic_id}: armature-parented meshes are not part of this contract")

        # mesh content
        vertex_count = len(me.vertices)
        polygon_count = len(me.polygons)
        if vertex_count == 0:
            report.error(f"{semantic_id}: mesh has zero vertices")
            continue
        if polygon_count == 0:
            report.error(f"{semantic_id}: mesh has zero polygons (empty or point/edge-only mesh)")
            continue
        if me.validate(verbose=False):
            report.error(f"{semantic_id}: mesh failed Blender's own validity check and needed repair")

        total_area = 0.0
        non_finite_vertex = False
        for vertex in me.vertices:
            co = vertex.co
            if not _is_finite_vector((co.x, co.y, co.z)):
                non_finite_vertex = True
                break
        if non_finite_vertex:
            report.error(f"{semantic_id}: mesh contains a non-finite vertex coordinate")
            continue
        for polygon in me.polygons:
            total_area += polygon.area
        if total_area <= ZERO_AREA_EPSILON:
            report.error(
                f"{semantic_id}: total polygon area {total_area:.3e} m^2 is zero; degenerate mesh"
            )

        # UV contract
        uv_names = [layer.name for layer in me.uv_layers]
        if texcoord_required and not uv_names:
            report.error(
                f"{semantic_id}: manifest requires TEXCOORD_0 but the mesh has no UV map"
            )
        elif texcoord_required and len(uv_names) > 1:
            report.warn(
                f"{semantic_id}: mesh has {len(uv_names)} UV maps {uv_names}; only the active one "
                f"is exported as TEXCOORD_0"
            )
        elif not texcoord_required and uv_names:
            report.warn(
                f"{semantic_id}: mesh has UV map(s) {uv_names} but "
                f"attribute_contract.texcoord_0_required is false; confirm this is intentional"
            )

        # material contract
        slots = [slot.material for slot in ob.material_slots]
        if not slots:
            report.error(f"{semantic_id}: mesh has no material slot")
        elif len(slots) > 1:
            report.error(
                f"{semantic_id}: mesh has {len(slots)} material slots; the contract is exactly one "
                f"material per semantic component (one primitive per component)"
            )
        for index, material in enumerate(slots):
            if material is None:
                report.error(f"{semantic_id}: material slot {index} is unassigned")
                continue
            expected_name = component.get("material_name")
            if material.name != expected_name:
                report.error(
                    f"{semantic_id}: material {material.name!r} != manifest {expected_name!r}"
                )
            if material.name not in catalog:
                report.error(
                    f"{semantic_id}: material {material.name!r} is not in the manifest catalog"
                )

        # custom properties (pipeline metadata)
        if ob.get("rc_semantic_id") not in (None, semantic_id):
            report.error(
                f"{semantic_id}: rc_semantic_id custom property is {ob.get('rc_semantic_id')!r}"
            )
        declared_index = ob.get("rc_primitive_index")
        if declared_index is not None and int(declared_index) != component["primitive_index"]:
            report.error(
                f"{semantic_id}: rc_primitive_index is {declared_index} but the manifest declares "
                f"{component['primitive_index']}"
            )

        # bounds in the render-body frame
        corners = _world_corners(ob)
        if not all(_is_finite_vector(corner) for corner in corners):
            report.error(f"{semantic_id}: world-space bounds contain a non-finite corner")
            continue
        minimum, maximum = ac.blender_bounds_to_render_body(corners)
        render_bounds_by_semantic[semantic_id] = (minimum, maximum)
        expected = component.get("expected_bounds") or {}
        for key, measured in (("min", minimum), ("max", maximum)):
            want = expected.get(key)
            if not isinstance(want, list) or len(want) != 3:
                continue
            for axis in range(3):
                if abs(measured[axis] - want[axis]) > tolerance:
                    report.error(
                        f"{semantic_id}: bounds {key}[{'XYZ'[axis]}] {measured[axis]:.6f} != "
                        f"manifest {want[axis]} (tolerance {tolerance} m)"
                    )

    # --- moving surface separation ----------------------------------------
    for mesh_name, owners in sorted(mesh_users.items()):
        if len(owners) > 1:
            report.error(
                f"mesh {mesh_name!r} is shared by {owners}; every semantic component needs its own "
                f"mesh so a moving surface can never be merged with or instanced against rigid geometry"
            )
    for semantic_id, surface in sorted(surfaces.items()):
        component = by_semantic.get(semantic_id)
        if component is None:
            report.error(f"moving surface {semantic_id!r} has no component entry in the manifest")
            continue
        if bpy.data.objects.get(semantic_id) is None:
            report.error(f"moving surface {semantic_id!r} is not a separate object in the source")
            continue
        for key in ("surface_id", "control_surface_binding_id"):
            if not surface.get(key):
                report.error(f"moving surface {semantic_id} declares no {key}")
        origin = surface.get("hinge_origin_render_body_m")
        pivot = surface.get("blender_pivot")
        if isinstance(origin, list) and isinstance(pivot, list) and len(origin) == 3 and len(pivot) == 3:
            expected_pivot = ac.render_body_to_blender(origin)
            for axis in range(3):
                if abs(pivot[axis] - expected_pivot[axis]) > 1.0e-9:
                    report.error(
                        f"moving surface {semantic_id}: blender_pivot {pivot} does not match "
                        f"hinge_origin_render_body_m {origin} converted to Blender space "
                        f"{tuple(expected_pivot)}"
                    )

    # --- global bounds and orientation ------------------------------------
    if render_bounds_by_semantic:
        global_min = [
            min(b[0][axis] for b in render_bounds_by_semantic.values()) for axis in range(3)
        ]
        global_max = [
            max(b[1][axis] for b in render_bounds_by_semantic.values()) for axis in range(3)
        ]
        report.summary["render_body_bounds_min"] = [round(v, 6) for v in global_min]
        report.summary["render_body_bounds_max"] = [round(v, 6) for v in global_max]
        expected_min = bounds_spec.get("global_min")
        expected_max = bounds_spec.get("global_max")
        if isinstance(expected_min, list) and isinstance(expected_max, list):
            for axis in range(3):
                if abs(global_min[axis] - expected_min[axis]) > tolerance:
                    report.error(
                        f"global bounds min[{'XYZ'[axis]}] {global_min[axis]:.6f} != manifest "
                        f"{expected_min[axis]} (tolerance {tolerance} m)"
                    )
                if abs(global_max[axis] - expected_max[axis]) > tolerance:
                    report.error(
                        f"global bounds max[{'XYZ'[axis]}] {global_max[axis]:.6f} != manifest "
                        f"{expected_max[axis]} (tolerance {tolerance} m)"
                    )
        for axis in range(3):
            extent = global_max[axis] - global_min[axis]
            if extent <= 0.0:
                report.error(f"global {'XYZ'[axis]} extent is non-positive ({extent:.6f} m)")
            elif extent > absurd:
                report.error(
                    f"global {'XYZ'[axis]} extent {extent:.3f} m exceeds the absurd-bounds limit "
                    f"{absurd} m; suspect a non-metric unit system or an unapplied scale"
                )
        for failure in ac.check_orientation(
            global_min, global_max, manifest.get("orientation_checks", {}),
            render_bounds_by_semantic,
        ):
            report.error(failure)

    # --- manifest-level sanity --------------------------------------------
    if manifest.get("consumed_by_simulation") is not False:
        report.error(
            "manifest must declare consumed_by_simulation=false: this contract is presentation only"
        )


def parse_blender_argv(argv: list[str]) -> list[str]:
    if "--" in argv:
        return argv[argv.index("--") + 1:]
    return []


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--asset-id", default=ac.DEFAULT_ASSET_ID)
    arguments = parser.parse_args(
        argv if argv is not None else parse_blender_argv(sys.argv)
    )

    manifest = ac.load_manifest(arguments.asset_id)
    subject = os.path.abspath(bpy.data.filepath or "<unsaved>")
    report = SourceReport(subject)
    validate_source(manifest, report)
    print(report.render(), flush=True)
    return 0 if report.ok else 1


if __name__ == "__main__":
    sys.exit(main())
