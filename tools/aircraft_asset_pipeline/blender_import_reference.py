#!/usr/bin/env python3
"""
RV2-7A Blender source materialiser for RC Simulation Engine aircraft assets.

Turns the committed reference GLB into an editable, semantically organised
Blender source file. The .blend is a DCC working document only: it is never a
runtime dependency and it never replaces the committed production GLB.

What it does:
  1. starts from an empty factory scene with a metric, 1.0-scale unit system
  2. imports the reference GLB declared by the manifest
  3. renames every imported object from its legacy generator part name to its
     stable semantic id, and stamps pipeline custom properties on it
  4. parents every component to a single ACRO_ROOT empty
  5. moves each moving surface's object origin onto its authored hinge line so
     the pivot is usable for DCC authoring and preview
  6. records provenance in an embedded text block
  7. saves models/<asset_id>/source/<asset_id>.blend

It never writes to the reference GLB path and refuses to overwrite an existing
.blend unless --force is given.

Invocation:
    blender --background --python-exit-code 1 \
        --python tools/aircraft_asset_pipeline/blender_import_reference.py \
        -- --asset-id acro_electric_01 [--force]
"""

from __future__ import annotations

import hashlib
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import asset_contract as ac  # noqa: E402

import bpy  # noqa: E402
from mathutils import Matrix, Vector  # noqa: E402

PROVENANCE_TEXT_NAME = "RC_ASSET_PIPELINE"


def _sha256(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def _ensure_single_empty_scene(title: str):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scenes = list(bpy.data.scenes)
    if not scenes:
        scene = bpy.data.scenes.new(title)
        bpy.context.window.scene = scene
    else:
        scene = scenes[0]
    scene.name = title
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"
    return scene


def _clear_orphans() -> None:
    for collection in list(bpy.data.collections):
        if collection.users == 0:
            bpy.data.collections.remove(collection)
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    for material in list(bpy.data.materials):
        if material.users == 0:
            bpy.data.materials.remove(material)


def _write_provenance(manifest: dict, reference_path: str, output_path: str) -> None:
    lines = [
        "RC Simulation Engine - aircraft asset pipeline source",
        "",
        f"asset_id            : {manifest.get('asset_id')}",
        f"purpose             : {manifest.get('purpose')}",
        f"classification      : {manifest.get('classification')}",
        "",
        "THIS .blend IS A DCC SOURCE FILE, NOT A RUNTIME DEPENDENCY.",
        "Nothing in the simulation, physics or renderer reads it. The runtime",
        "authority is the exported GLB declared by the manifest, and the",
        "articulation authority is model.json -> presentation.articulated_surfaces.",
        "",
        f"reference GLB       : {os.path.relpath(reference_path, ac.REPO_ROOT)}",
        f"reference SHA-256   : {_sha256(reference_path)}",
        f"manifest            : {os.path.relpath(ac.manifest_path(manifest.get('asset_id')), ac.REPO_ROOT)}",
        f"output              : {os.path.relpath(output_path, ac.REPO_ROOT)}",
        "",
        "Coordinate contract",
        "  render-body / glTF : +X aircraft right, +Y up, -Z forward (nose)",
        "  Blender            : +X right, +Z up, -Y forward",
        "  units              : metres, 1 Blender unit = 1 m, metric unit system",
        "",
        "Object contract",
        "  one mesh object per semantic component, named exactly by its semantic id",
        "  every component is a direct child of the single ACRO_ROOT empty",
        "  scale is exactly 1.0 and rotation is exactly identity; geometry is baked",
        "  the object origin is the authoring pivot: hinge line for moving surfaces,",
        "    world origin for rigid components",
        "  exactly one material per component, named as declared in the manifest",
        "  no modifiers, no shape keys, no armatures, no lights, no cameras",
        "",
        "Regenerate with:",
        "  blender --background --python-exit-code 1 \\",
        "      --python tools/aircraft_asset_pipeline/blender_import_reference.py \\",
        f"      -- --asset-id {manifest.get('asset_id')} --force",
        "",
        "Validate with:",
        "  blender --background <this file> --python-exit-code 1 \\",
        "      --python tools/aircraft_asset_pipeline/blender_validate_source.py",
    ]
    existing = bpy.data.texts.get(PROVENANCE_TEXT_NAME)
    if existing is not None:
        bpy.data.texts.remove(existing)
    text = bpy.data.texts.new(PROVENANCE_TEXT_NAME)
    text.from_string("\n".join(lines) + "\n")


def materialise(manifest: dict, force: bool = False) -> int:
    asset_id = manifest["asset_id"]
    title = manifest.get("title", asset_id)
    reference_path = ac.repo_relative(manifest["paths"]["reference_glb"])
    output_path = ac.repo_relative(manifest["paths"]["source_blend"])
    root_name = manifest["root_object"]["blender_name"]

    if not os.path.isfile(reference_path):
        print(f"[FAIL] reference GLB not found: {reference_path}", flush=True)
        return 1
    if os.path.exists(output_path) and not force:
        print(
            f"[FAIL] source already exists: {output_path}\n"
            f"       re-run with --force to overwrite it deliberately.",
            flush=True,
        )
        return 1

    components = ac.components(manifest)
    by_legacy = {c["legacy_node_name"]: c for c in components}
    surfaces = {s["semantic_id"]: s for s in ac.moving_surfaces(manifest)}
    catalog = ac.material_catalog(manifest)

    _ensure_single_empty_scene(title)
    print(f"[info] importing reference GLB: {reference_path}", flush=True)
    print(f"[info] reference SHA-256: {_sha256(reference_path)}", flush=True)
    bpy.ops.import_scene.gltf(filepath=reference_path)

    imported = [ob for ob in bpy.data.objects]
    failures: list[str] = []

    unknown = sorted({ob.name for ob in imported} - set(by_legacy))
    for name in unknown:
        failures.append(f"reference GLB contains node {name!r} which the manifest does not declare")
    missing = sorted(set(by_legacy) - {ob.name for ob in imported})
    for name in missing:
        failures.append(f"reference GLB is missing the node {name!r} declared by the manifest")
    non_mesh = sorted(ob.name for ob in imported if ob.type != "MESH")
    for name in non_mesh:
        failures.append(f"reference node {name!r} is a {bpy.data.objects[name].type}, expected MESH")
    if failures:
        for failure in failures:
            print(f"[FAIL] {failure}", flush=True)
        return 1

    root = bpy.data.objects.new(root_name, None)
    root.empty_display_type = "PLAIN_AXES"
    root.empty_display_size = 0.25
    root["rc_role"] = "asset_root"
    root["rc_asset_id"] = asset_id
    bpy.context.scene.collection.objects.link(root)

    print("[info] semantic organisation", flush=True)
    print(f"  {'idx':>3}  {'semantic_id':<24} {'legacy node':<24} {'pivot (Blender m)':<34} material", flush=True)
    for component in components:
        legacy_name = component["legacy_node_name"]
        semantic_id = component["semantic_id"]
        ob = bpy.data.objects[legacy_name]
        ob.name = semantic_id
        if ob.data is not None:
            ob.data.name = semantic_id

        ob.hide_set(False)
        ob.hide_render = False
        ob.hide_viewport = False
        ob.show_in_front = False

        ob["rc_semantic_id"] = semantic_id
        ob["rc_primitive_index"] = component["primitive_index"]
        ob["rc_legacy_node_name"] = legacy_name
        ob["rc_family"] = component.get("family", "")
        ob["rc_asset_id"] = asset_id
        surface = surfaces.get(semantic_id)
        if surface is not None:
            ob["rc_moving_surface"] = surface["surface_id"]
            ob["rc_control_surface_binding_id"] = surface["control_surface_binding_id"]

        # Parent under the single root. The root is at the identity transform,
        # so parenting cannot move the geometry.
        ob.parent = root
        ob.matrix_parent_inverse = Matrix.Identity(4)

        pivot = component.get("blender_pivot", [0.0, 0.0, 0.0])
        if surface is not None:
            pivot = surface["blender_pivot"]
        pivot_vector = Vector(pivot)
        if pivot_vector.length > 0.0:
            # Move the origin onto the authored pivot without moving geometry in
            # world space: shift the mesh data by -pivot, then place the object
            # at +pivot. blender_export_glb.py bakes this back on export.
            ob.data.transform(Matrix.Translation(-pivot_vector))
            ob.matrix_world = Matrix.Translation(pivot_vector)
        else:
            ob.matrix_world = Matrix.Identity(4)

        material_name = component["material_name"]
        if material_name not in catalog:
            print(f"[FAIL] {semantic_id}: manifest material {material_name!r} is not in the catalog", flush=True)
            return 1
        slots = [slot.material.name if slot.material else None for slot in ob.material_slots]
        if slots != [material_name]:
            print(
                f"[FAIL] {semantic_id}: reference material slots {slots} != manifest [{material_name!r}]",
                flush=True,
            )
            return 1

        label = "hinge" if surface is not None else "origin"
        print(
            f"  {component['primitive_index']:>3}  {semantic_id:<24} {legacy_name:<24} "
            f"{tuple(round(v, 6) for v in pivot)!s:<34} {material_name} ({label})",
            flush=True,
        )

    _clear_orphans()
    _write_provenance(manifest, reference_path, output_path)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=output_path)
    # Blender writes a .blend1 rollback sibling when overwriting an existing
    # file. The source is fully regenerable from the reference GLB, so the
    # backup must never be committed.
    backup_path = output_path + "1"
    if os.path.exists(backup_path):
        os.remove(backup_path)
        print(f"[info] removed Blender rollback backup: {backup_path}", flush=True)
    print(f"[info] saved Blender source: {output_path}", flush=True)
    print(f"[info] objects: {len(bpy.data.objects)} (1 root + {len(components)} components)", flush=True)
    print(f"[info] meshes: {len(bpy.data.meshes)}  materials: {len(bpy.data.materials)}", flush=True)
    print(
        "[info] this file is a DCC source only; the runtime asset "
        f"{manifest['paths']['production_glb']} was not touched",
        flush=True,
    )
    return 0


def parse_blender_argv(argv: list[str]) -> list[str]:
    if "--" in argv:
        return argv[argv.index("--") + 1:]
    return []


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--asset-id", default=ac.DEFAULT_ASSET_ID)
    parser.add_argument(
        "--force", action="store_true",
        help="overwrite an existing .blend source file",
    )
    arguments = parser.parse_args(argv if argv is not None else parse_blender_argv(sys.argv))

    manifest = ac.load_manifest(arguments.asset_id)
    return materialise(manifest, force=arguments.force)


if __name__ == "__main__":
    sys.exit(main())
