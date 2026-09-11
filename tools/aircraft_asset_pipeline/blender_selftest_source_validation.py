#!/usr/bin/env python3
"""
RV2-7A self-test for blender_validate_source.py.

Proves the source validator is not decorative: it opens the real .blend source
in memory, applies one contract violation at a time, and asserts the validator
rejects it with the expected diagnostic. Every mutation is reverted and the
clean state is re-verified, so the run is non-destructive.

The .blend is NEVER saved by this script.

Invocation:
    blender --background models/acro_electric_01/source/acro_electric_01.blend \
        --python-exit-code 1 \
        --python tools/aircraft_asset_pipeline/blender_selftest_source_validation.py \
        -- --asset-id acro_electric_01

Note on duplicate semantic objects: Blender guarantees unique datablock names,
so a "duplicate" cannot be expressed as two objects with the same name. It shows
up as a linked duplicate (a second object sharing the moving surface's mesh) or
as an undeclared object, and both are covered below. Manifest-level duplicate
semantic ids are covered by test_asset_contract.py.
"""

from __future__ import annotations

import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import asset_contract as ac  # noqa: E402
import blender_validate_source as bvs  # noqa: E402

import bpy  # noqa: E402


def _object(name: str):
    found = bpy.data.objects.get(name)
    if found is None:
        raise KeyError(f"scenario requires object {name!r}, which is not in the source")
    return found


def _scene():
    return bpy.context.scene


def _root(manifest: dict):
    return _object(manifest["root_object"]["blender_name"])


def _update() -> None:
    bpy.context.view_layer.update()


def run_validation(manifest: dict):
    report = bvs.SourceReport(bpy.data.filepath or "<unsaved>")
    bvs.validate_source(manifest, report)
    return report


# --- mutations -------------------------------------------------------------

def mutate_imperial_units(_manifest):
    _scene().unit_settings.system = "IMPERIAL"


def revert_imperial_units(_manifest):
    _scene().unit_settings.system = "METRIC"


def mutate_unit_scale(_manifest):
    _scene().unit_settings.scale_length = 0.0254


def revert_unit_scale(_manifest):
    _scene().unit_settings.scale_length = 1.0


def mutate_unapplied_scale(_manifest):
    _object("FUSELAGE").scale = (2.0, 1.0, 1.0)


def revert_unapplied_scale(_manifest):
    _object("FUSELAGE").scale = (1.0, 1.0, 1.0)


def mutate_negative_scale(_manifest):
    _object("WING_MAIN_FIXED").scale = (-1.0, 1.0, 1.0)


def revert_negative_scale(_manifest):
    _object("WING_MAIN_FIXED").scale = (1.0, 1.0, 1.0)


def mutate_rotation(_manifest):
    _object("HTAIL_FIXED").rotation_euler = (0.0, 0.0, 0.5)


def revert_rotation(_manifest):
    _object("HTAIL_FIXED").rotation_euler = (0.0, 0.0, 0.0)


def mutate_non_finite(_manifest):
    _object("FUSELAGE").location = (float("nan"), 0.0, 0.0)


def revert_non_finite(_manifest):
    _object("FUSELAGE").location = (0.0, 0.0, 0.0)


def mutate_absurd_scale(_manifest):
    _object("FUSELAGE").scale = (1000.0, 1000.0, 1000.0)


def revert_absurd_scale(_manifest):
    _object("FUSELAGE").scale = (1.0, 1.0, 1.0)


def mutate_rename_away(_manifest):
    _object("RUDDER").name = "RUDDER_PROBE"


def revert_rename_away(_manifest):
    _object("RUDDER_PROBE").name = "RUDDER"


def mutate_instanced_moving_surface(manifest):
    source = _object("AILERON_L")
    duplicate = source.copy()
    _scene().collection.objects.link(duplicate)
    duplicate.parent = _root(manifest)
    _update()


def revert_instanced_moving_surface(_manifest):
    for name in list(bpy.data.objects.keys()):
        if name.startswith("AILERON_L."):
            duplicate = bpy.data.objects[name]
            bpy.data.objects.remove(duplicate, do_unlink=True)
    _update()


def mutate_hidden_from_export(_manifest):
    _object("ELEVATOR").hide_render = True


def revert_hidden_from_export(_manifest):
    _object("ELEVATOR").hide_render = False


def mutate_missing_material(_manifest):
    _object("CANOPY").material_slots[0].material = None


def revert_missing_material(_manifest):
    _object("CANOPY").material_slots[0].material = bpy.data.materials["Tinted Canopy"]


def mutate_wrong_material(_manifest):
    _object("SPINNER").material_slots[0].material = bpy.data.materials["Gear Metal"]


def revert_wrong_material(_manifest):
    _object("SPINNER").material_slots[0].material = bpy.data.materials["Painted Spinner"]


def mutate_modifier(_manifest):
    _object("CANOPY").modifiers.new("RV2_7A_PROBE", "SUBSURF")


def revert_modifier(_manifest):
    ob = _object("CANOPY")
    modifier = ob.modifiers.get("RV2_7A_PROBE")
    if modifier is not None:
        ob.modifiers.remove(modifier)


def mutate_pivot_drift(_manifest):
    _object("RUDDER").location = (0.05, -0.735, 0.375)


def revert_pivot_drift(_manifest):
    _object("RUDDER").location = (0.0, -0.735, 0.375)


def mutate_empty_mesh_component(_manifest):
    _object("DETAIL_PROP_TIPS").name = "DETAIL_PROP_TIPS_BAK"
    mesh = bpy.data.meshes.new("RV2_7A_EMPTY_PROBE")
    probe = bpy.data.objects.new("DETAIL_PROP_TIPS", mesh)
    _scene().collection.objects.link(probe)
    probe.parent = bpy.data.objects.get("ACRO_ROOT")
    _update()


def revert_empty_mesh_component(_manifest):
    probe = bpy.data.objects.get("DETAIL_PROP_TIPS")
    if probe is not None and probe.data is not None and probe.data.name == "RV2_7A_EMPTY_PROBE":
        mesh = probe.data
        bpy.data.objects.remove(probe, do_unlink=True)
        bpy.data.meshes.remove(mesh)
    renamed = bpy.data.objects.get("DETAIL_PROP_TIPS_BAK")
    if renamed is not None:
        renamed.name = "DETAIL_PROP_TIPS"
    _update()


def mutate_flipped_orientation(manifest):
    root = _root(manifest)
    root.rotation_euler = (0.0, 0.0, math.pi)
    _update()


def revert_flipped_orientation(manifest):
    root = _root(manifest)
    root.rotation_euler = (0.0, 0.0, 0.0)
    _update()


SCENARIOS = [
    ("unit_system_not_metric", "must be METRIC", mutate_imperial_units, revert_imperial_units),
    ("unit_scale_incoherent", "scale_length", mutate_unit_scale, revert_unit_scale),
    ("unapplied_scale", "unapplied scale", mutate_unapplied_scale, revert_unapplied_scale),
    ("negative_scale", "negative or zero scale", mutate_negative_scale, revert_negative_scale),
    ("non_identity_rotation", "axis-aligned", mutate_rotation, revert_rotation),
    ("non_finite_transform", "non-finite", mutate_non_finite, revert_non_finite),
    ("absurd_bounds", "absurd-bounds", mutate_absurd_scale, revert_absurd_scale),
    ("missing_required_semantic_object", "missing required semantic object",
     mutate_rename_away, revert_rename_away),
    ("undeclared_object_instancing_a_moving_surface", "is shared by",
     mutate_instanced_moving_surface, revert_instanced_moving_surface),
    ("hidden_production_mesh_excluded_from_export", "excluded from the export",
     mutate_hidden_from_export, revert_hidden_from_export),
    ("missing_material_assignment", "unassigned", mutate_missing_material, revert_missing_material),
    ("wrong_material_assignment", "!= manifest", mutate_wrong_material, revert_wrong_material),
    ("modifier_present", "modifier", mutate_modifier, revert_modifier),
    ("moving_surface_pivot_drift", "hinge origin", mutate_pivot_drift, revert_pivot_drift),
    ("zero_vertex_component_mesh", "zero vertices",
     mutate_empty_mesh_component, revert_empty_mesh_component),
    ("wrong_orientation_convention", "nose_forward_minus_z",
     mutate_flipped_orientation, revert_flipped_orientation),
]


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument("--asset-id", default=ac.DEFAULT_ASSET_ID)
    arguments = parser.parse_args(
        argv if argv is not None else bvs.parse_blender_argv(sys.argv)
    )

    manifest = ac.load_manifest(arguments.asset_id)
    subject = os.path.abspath(bpy.data.filepath or "<unsaved>")
    print(f"blender_selftest_source_validation: {subject}", flush=True)
    print(f"  blender: {bpy.app.version_string}", flush=True)

    baseline = run_validation(manifest)
    if not baseline.ok:
        print("[FAIL] the source does not validate cleanly before any mutation:", flush=True)
        print(baseline.render(), flush=True)
        return 1
    print("  [info] baseline source validates cleanly", flush=True)

    failures = []
    for scenario_id, expected, mutate, revert in SCENARIOS:
        try:
            mutate(manifest)
            _update()
        except Exception as exc:
            failures.append(f"{scenario_id}: mutation raised {exc!r}")
            try:
                revert(manifest)
                _update()
            except Exception:
                pass
            continue

        report = run_validation(manifest)
        joined = "\n".join(report.errors)
        if report.ok:
            failures.append(f"{scenario_id}: validator PASSED, expected failure ({expected!r})")
        elif expected not in joined:
            failures.append(
                f"{scenario_id}: validator failed but not with {expected!r}; got:\n{joined}"
            )
        else:
            matched = next(e for e in report.errors if expected in e)
            print(f"  [ok]   {scenario_id}: {matched}", flush=True)

        try:
            revert(manifest)
            _update()
        except Exception as exc:
            failures.append(f"{scenario_id}: revert raised {exc!r}")
            continue

        restored = run_validation(manifest)
        if not restored.ok:
            failures.append(
                f"{scenario_id}: source did not return to a clean state after revert:\n"
                + "\n".join(restored.errors)
            )

    if failures:
        print(f"blender_selftest_source_validation: FAIL ({len(failures)} problem(s))", flush=True)
        for failure in failures:
            print(f"  [FAIL] {failure}", flush=True)
        return 1

    print(
        f"blender_selftest_source_validation: PASS "
        f"({len(SCENARIOS)} violation(s) rejected, source unchanged and never saved)",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
