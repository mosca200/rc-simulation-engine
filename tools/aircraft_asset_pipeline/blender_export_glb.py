#!/usr/bin/env python3
"""
RV2-7A Blender -> glTF 2.0 binary GLB production exporter.

ONE explicit, reproducible export configuration for RC Simulation Engine
aircraft assets. The .blend source is never modified or re-saved: this script
mutates a throwaway in-memory session only.

Why the script renames and bakes before exporting
-------------------------------------------------
Blender's glTF exporter emits nodes (and therefore mesh indices) in alphabetical
object-name order, and the runtime maps moving surfaces by primitive index. A
naive export of clean semantic names would silently reorder the primitives and
break articulation. Verified on Blender 5.2.1 LTS / Khronos glTF Blender I/O
v5.2.40: re-exporting the committed Acro GLB moved ELEVATOR to primitive 3 and
RUDDER to primitive 11 -> 13. This script therefore pins the order by renaming
each object to '{primitive_index:02d}_{SEMANTIC_ID}' for the duration of the
export, and the offline validator re-checks the result and fails closed.

The production loader reads baked vertex positions and does not apply glTF node
transforms to `GlbAsset::primitives`, and `export_apply` only applies modifiers
(verified: it leaves object translation in the node). So every object transform
is baked into the mesh data here, which is also what preserves the authored
Blender pivots in the source file.

RV2-7A guard rails
------------------
This slice must not replace the runtime asset, so the exporter refuses to write
anywhere under models/ and refuses to write the manifest's production GLB path.
Scratch output belongs under target/asset_pipeline/ (already git-ignored).

Invocation:
    blender --background models/acro_electric_01/source/acro_electric_01.blend \
        --python-exit-code 1 \
        --python tools/aircraft_asset_pipeline/blender_export_glb.py \
        -- --asset-id acro_electric_01 \
           --output target/asset_pipeline/acro_electric_01/aircraft.blender.glb
"""

from __future__ import annotations

import argparse
import json
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import asset_contract as ac  # noqa: E402
import validate_glb  # noqa: E402

import bpy  # noqa: E402
from mathutils import Matrix  # noqa: E402

# THE production export configuration. Every flag is stated explicitly so the
# result cannot drift with Blender's per-file operator settings.
PRODUCTION_EXPORT_SETTINGS = {
    "export_format": "GLB",
    # +Y up / -Z forward glTF frame, matching the render-body convention.
    "export_yup": True,
    # Modifiers: none are permitted by the source contract, so nothing to apply.
    # Transforms are baked explicitly by this script instead (see module doc).
    "export_apply": False,
    "export_texcoords": True,
    "export_normals": True,
    # Tangents need a UV map; the renderer path does not consume them.
    "export_tangents": False,
    "export_materials": "EXPORT",
    "export_image_format": "AUTO",
    "export_keep_originals": False,
    "export_shared_accessors": False,
    # Static presentation asset: no animation, skinning, morphs or scene props.
    "export_animations": False,
    "export_nla_strips": False,
    "export_bake_animation": False,
    "export_current_frame": False,
    "export_frame_range": False,
    "export_skins": False,
    "export_morph": False,
    "export_morph_normal": False,
    "export_morph_tangent": False,
    "export_morph_animation": False,
    "export_cameras": False,
    "export_lights": False,
    "export_def_bones": False,
    "export_leaf_bone": False,
    "export_hierarchy_flatten_objs": False,
    "export_hierarchy_flatten_bones": False,
    "export_armature_object_remove": False,
    # No Blender custom properties, no vertex colours, no generic attributes:
    # the runtime consumes core metallic-roughness only.
    "export_extras": False,
    "export_attributes": False,
    "export_vertex_color": "NONE",
    "export_all_vertex_colors": False,
    "export_active_vertex_color_when_no_material": False,
    "export_gn_mesh": False,
    "export_original_specular": False,
    "export_import_convert_lighting_mode": "SPEC",
    # No compression and no external post-processors: deterministic, dependency-free.
    "export_draco_mesh_compression_enable": False,
    "export_meshopt_compression_enable": False,
    "export_use_gltfpack": False,
    "export_unused_images": False,
    "export_unused_textures": False,
    # Export the whole scene regardless of viewport visibility, so a stray
    # viewport state can never silently drop a production mesh. The source
    # validator separately rejects hidden or excluded required components.
    "use_selection": False,
    "use_visible": False,
    "use_renderable": False,
    "use_active_collection": False,
    # Never persist operator settings back into the .blend.
    "will_save_settings": False,
}


class ExportReport:
    def __init__(self):
        self.errors: list[str] = []
        self.info: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def note(self, message: str) -> None:
        self.info.append(message)

    @property
    def ok(self) -> bool:
        return not self.errors


def _refuse_unsafe_output(output_path: str, manifest: dict, report: ExportReport) -> str:
    """Resolve the output path and refuse to overwrite anything the runtime uses."""
    resolved = os.path.abspath(output_path)
    production = os.path.abspath(ac.repo_relative(manifest["paths"]["production_glb"]))
    reference = os.path.abspath(ac.repo_relative(manifest["paths"]["reference_glb"]))
    models_root = os.path.abspath(os.path.join(ac.REPO_ROOT, "models"))

    if resolved == production:
        report.error(
            f"refusing to export onto the production runtime asset {production}; "
            f"RV2-7A must not replace it"
        )
    if resolved == reference:
        report.error(f"refusing to export onto the reference GLB {reference}")
    if resolved.startswith(models_root + os.sep):
        report.error(
            f"refusing to write {resolved} inside models/; scratch exports belong under "
            f"target/asset_pipeline/ so they stay out of the repository"
        )
    if os.path.abspath(bpy.data.filepath or "") == resolved:
        report.error("refusing to overwrite the open .blend source with a GLB")
    if not resolved.lower().endswith(".glb"):
        report.error(f"output must be a .glb file, got {resolved}")
    return resolved


def _prepare_for_export(manifest: dict, report: ExportReport) -> None:
    """Bake world transforms into mesh data and pin export order into object names.

    Runs on a throwaway session: the .blend is never saved afterwards.
    """
    root_name = manifest["root_object"]["blender_name"]
    root = bpy.data.objects.get(root_name)
    if root is not None:
        root.matrix_world = Matrix.Identity(4)

    for component in ac.components(manifest):
        semantic_id = component["semantic_id"]
        ob = bpy.data.objects.get(semantic_id)
        if ob is None:
            report.error(f"{semantic_id}: object is missing, cannot export")
            continue
        me = ob.data
        if me is None:
            report.error(f"{semantic_id}: object has no mesh data")
            continue
        if me.users != 1:
            report.error(
                f"{semantic_id}: mesh {me.name!r} has {me.users} users; baking a shared mesh "
                f"would transform it more than once"
            )
            continue

        world = Matrix(ob.matrix_world)
        if world != Matrix.Identity(4):
            me.transform(world)
            ob.matrix_world = Matrix.Identity(4)
        report.note(
            f"baked {semantic_id}: origin {tuple(round(v, 6) for v in world.translation)} -> identity"
        )

        export_name = ac.export_node_name(component)
        ob.name = export_name
        me.name = export_name

    bpy.context.view_layer.update()


def _export(output_path: str, report: ExportReport) -> None:
    settings = dict(PRODUCTION_EXPORT_SETTINGS)
    directory = os.path.dirname(output_path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    report.note(
        "export settings: "
        + ", ".join(f"{key}={settings[key]!r}" for key in sorted(settings))
    )
    bpy.ops.export_scene.gltf(filepath=output_path, **settings)
    if not os.path.isfile(output_path):
        report.error(f"exporter reported success but {output_path} does not exist")


def export(manifest: dict, output_path: str, skip_source_validation: bool = False,
           fingerprint_out: str | None = None, profile: str = "blender_export") -> int:
    report = ExportReport()
    resolved = _refuse_unsafe_output(output_path, manifest, report)
    if not report.ok:
        for error in report.errors:
            print(f"[FAIL] {error}", flush=True)
        return 1

    source_path = os.path.abspath(bpy.data.filepath or "")
    if not source_path:
        report.error(
            "no .blend source is open; invoke as: blender --background <source.blend> "
            "--python-exit-code 1 --python blender_export_glb.py -- --output <scratch.glb>"
        )
    print(f"[info] source .blend : {source_path}", flush=True)
    print(f"[info] export target : {resolved}", flush=True)
    print(f"[info] profile       : {profile}", flush=True)

    if not skip_source_validation:
        import blender_validate_source

        source_report = blender_validate_source.SourceReport(source_path)
        blender_validate_source.validate_source(manifest, source_report)
        print(source_report.render(), flush=True)
        if not source_report.ok:
            print("[FAIL] source validation failed; nothing was exported", flush=True)
            return 1

    if not report.ok:
        for error in report.errors:
            print(f"[FAIL] {error}", flush=True)
        return 1

    _prepare_for_export(manifest, report)
    if not report.ok:
        for error in report.errors:
            print(f"[FAIL] {error}", flush=True)
        return 1

    _export(resolved, report)
    if not report.ok:
        for error in report.errors:
            print(f"[FAIL] {error}", flush=True)
        return 1

    print(f"[info] wrote {os.path.getsize(resolved)} bytes to {resolved}", flush=True)

    validation = validate_glb.validate(resolved, ac.manifest_path(manifest["asset_id"]), profile)
    print(validation.render(), flush=True)

    if fingerprint_out:
        digest = validate_glb.fingerprint(resolved)
        directory = os.path.dirname(os.path.abspath(fingerprint_out))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(fingerprint_out, "w", encoding="utf-8") as handle:
            json.dump(digest, handle, indent=2, sort_keys=True)
            handle.write("\n")
        print(f"[info] semantic fingerprint: {fingerprint_out}", flush=True)

    # The mutated session is discarded: this script never calls
    # save_as_mainfile, and `blender --background` does not save on exit.

    if not validation.ok:
        print("[FAIL] exported GLB did not satisfy the manifest contract", flush=True)
        return 1
    print("[info] the .blend source was not modified or re-saved", flush=True)
    return 0


def parse_blender_argv(argv: list[str]) -> list[str]:
    if "--" in argv:
        return argv[argv.index("--") + 1:]
    return []


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--asset-id", default=ac.DEFAULT_ASSET_ID)
    parser.add_argument("--output", required=True, help="scratch .glb path (never under models/)")
    parser.add_argument("--profile", default="blender_export")
    parser.add_argument("--fingerprint-out", help="write the semantic fingerprint JSON here")
    parser.add_argument(
        "--skip-source-validation", action="store_true",
        help="export even if blender_validate_source fails (debugging only)",
    )
    arguments = parser.parse_args(argv if argv is not None else parse_blender_argv(sys.argv))

    manifest = ac.load_manifest(arguments.asset_id)
    return export(
        manifest,
        arguments.output,
        skip_source_validation=arguments.skip_source_validation,
        fingerprint_out=arguments.fingerprint_out,
        profile=arguments.profile,
    )


if __name__ == "__main__":
    sys.exit(main())
