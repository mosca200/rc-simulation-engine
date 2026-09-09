"""
PV1-R2: Production vegetation asset processor.

Imports REAL Poly Haven CC0 .blend source models, preserves authored geometry,
generates runtime LODs via decimation, and exports GLB files with the
2-primitive (bark + foliage) structure the renderer expects.

NO procedural reconstruction. All geometry derives from the authored source.

Source models (CC0 Poly Haven):
  pine_tree_01   — conifer, 3 authored variants with trunk+twig+needle parts
  fir_tree_01    — conifer, similar structure
  tree_small_02  — broadleaf, authored leaf card clusters
  jacaranda_tree — broadleaf, authored crown geometry

Pipeline:
  SOURCE .blend → import → separate bark/foliage by material →
  decimate for LOD → export GLB (2 primitives: bark + foliage)
"""

import bpy
import bmesh
import os
import sys
import json

# ── Paths ──────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# PV1-R2 FIX: robust repo-root resolution.
# SCRIPT_DIR = <repo>/tools/vegetation_processing → 2 levels up = repo root.
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
SOURCE_DIR = os.path.join(SCRIPT_DIR, "source_models")
OUTPUT_DIR = os.path.join(REPO_ROOT, "crates", "renderer", "assets", "vegetation")
LOG_PATH = os.path.join(SCRIPT_DIR, "processing_log.json")

os.makedirs(OUTPUT_DIR, exist_ok=True)

# Verify we're writing to the correct location
assert os.path.isdir(os.path.join(REPO_ROOT, "crates", "renderer")), \
    f"REPO_ROOT resolution failed: {REPO_ROOT} is not the repo root"

LOG = []

def log(msg):
    print(msg, flush=True)
    LOG.append(msg)

# ── Helpers ────────────────────────────────────────────────────────────────

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for block in bpy.data.meshes:
        if block.users == 0:
            bpy.data.meshes.remove(block)
    for block in bpy.data.materials:
        if block.users == 0:
            bpy.data.materials.remove(block)
    for block in bpy.data.collections:
        if block.users == 0:
            bpy.data.collections.remove(block)


def get_mesh_objects():
    return [o for o in bpy.data.objects if o.type == 'MESH']


def join_objects(objects, name):
    """Join a list of objects into one mesh."""
    if not objects:
        return None
    # Make all objects visible and selectable
    for obj in objects:
        obj.hide_set(False)
        obj.hide_select = False
        obj.hide_viewport = False
    bpy.ops.object.select_all(action='DESELECT')
    for obj in objects:
        obj.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    if len(objects) > 1:
        bpy.ops.object.join()
    result = bpy.context.active_object
    result.name = name
    return result


def decimate_mesh(obj, ratio, name=None):
    """Apply decimate modifier and apply it. Removes shape keys first."""
    if obj is None:
        return None
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    # Remove shape keys (they prevent modifier application)
    while obj.data.shape_keys and len(obj.data.shape_keys.key_blocks) > 0:
        obj.shape_key_remove(obj.data.shape_keys.key_blocks[0])
    # Remove any existing modifiers that might interfere
    for mod in list(obj.modifiers):
        obj.modifiers.remove(mod)
    # Apply decimate
    mod = obj.modifiers.new(name="Decimate", type='DECIMATE')
    mod.ratio = max(ratio, 0.001)
    try:
        bpy.ops.object.modifier_apply(modifier=mod.name)
    except RuntimeError:
        # Fallback: try dissolve method
        mod.decimate_type = 'DISSOLVE'
        mod.ratio = max(ratio, 0.001)
        try:
            bpy.ops.object.modifier_apply(modifier=mod.name)
        except RuntimeError:
            log(f"  WARNING: decimate failed for {obj.name}, keeping original")
    if name:
        obj.name = name
    return obj


def count_tris(obj):
    if obj is None:
        return 0
    return len(obj.data.polygons)


def duplicate_obj(obj, name):
    """Duplicate a mesh object."""
    if obj is None:
        return None
    bpy.ops.object.select_all(action='DESELECT')
    obj.hide_set(False)
    obj.hide_viewport = False
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.duplicate()
    dup = bpy.context.active_object
    dup.name = name
    return dup


def consolidate_materials(obj, mat_name):
    """Set all faces of obj to use a single material, creating it if needed."""
    if obj is None:
        return
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = bpy.data.materials.new(name=mat_name)
    # Clear all material slots and assign one
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    # Set all faces to material index 0
    for face in obj.data.polygons:
        face.material_index = 0


def export_glb(filepath, bark_obj, foliage_obj):
    """Export bark + foliage as exactly 2-primitive GLB."""
    # Consolidate materials so each mesh produces exactly 1 primitive
    consolidate_materials(bark_obj, "bark")
    consolidate_materials(foliage_obj, "foliage")
    bpy.ops.object.select_all(action='DESELECT')
    for obj in [bark_obj, foliage_obj]:
        if obj is not None:
            obj.select_set(True)
    bpy.context.view_layer.objects.active = bark_obj or foliage_obj
    bpy.ops.export_scene.gltf(
        filepath=filepath,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
        export_materials='EXPORT',
    )


def safe_remove(obj):
    if obj and obj.name in bpy.data.objects:
        bpy.data.objects.remove(obj, do_unlink=True)


# ── Species processing ─────────────────────────────────────────────────────

def process_conifer(source_name, runtime_name):
    """Process a conifer source model (pine_tree_01 or fir_tree_01)."""
    source_path = os.path.join(SOURCE_DIR, f"{source_name}_1k.blend")
    log(f"\n{'='*60}")
    log(f"Processing: {source_name} → {runtime_name}")
    log(f"Source: {source_path}")

    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=source_path)

    # PV1-R2: Poly Haven .blend files hide objects in disabled collections.
    # Move ALL mesh objects to the scene's active collection so they are
    # selectable and joinable.
    scene_col = bpy.context.scene.collection
    for obj in list(bpy.data.objects):
        if obj.type != 'MESH':
            continue
        # Link to scene collection if not already there
        if scene_col.objects.get(obj.name) is None:
            scene_col.objects.link(obj)
        obj.hide_set(False)
        obj.hide_select = False
        obj.hide_viewport = False
    bpy.context.view_layer.update()

    all_meshes = get_mesh_objects()
    log(f"  Source meshes: {len(all_meshes)}")

    # Classify by material name keywords
    bark_parts = []   # trunk + bark + dead_branches
    foliage_parts = []  # twig + needle + branch (foliage with alpha)

    for obj in all_meshes:
        # Skip the massive combined LOD meshes (a_LOD0, b_LOD0, etc.)
        name = obj.name
        is_combined = False
        for variant in ['_a_', '_b_', '_c_']:
            if variant in name and '_LOD' in name:
                is_combined = True
                break
        if is_combined:
            continue
        if not obj.data.materials:
            continue
        mat_name = obj.data.materials[0].name.lower()
        if any(k in mat_name for k in ['trunk', 'bark', 'dead']):
            bark_parts.append(obj)
        elif any(k in mat_name for k in ['twig', 'needle', 'branch']):
            foliage_parts.append(obj)

    log(f"  Bark parts: {len(bark_parts)}, Foliage parts: {len(foliage_parts)}")

    # Join into 2 primitives
    bark_joined = join_objects(bark_parts, "bark_src")
    foliage_joined = join_objects(foliage_parts, "foliage_src")

    bark_tris = count_tris(bark_joined)
    foliage_tris = count_tris(foliage_joined)
    log(f"  Source bark: {bark_tris} tris, foliage: {foliage_tris} tris")

    # LOD generation — ratios tuned for runtime budget (~15-25K LOD0 target)
    lod_configs = [
        (0.12, "lod0"),   # → ~12K tris for conifers
        (0.05, "lod1"),   # → ~5K
        (0.02, "lod2"),   # → ~2K
    ]

    for ratio, lod_name in lod_configs:
        bark_lod = duplicate_obj(bark_joined, f"bark_{lod_name}")
        foliage_lod = duplicate_obj(foliage_joined, f"foliage_{lod_name}")

        if bark_lod and bark_tris > 100:
            decimate_mesh(bark_lod, ratio)
        if foliage_lod and foliage_tris > 50:
            decimate_mesh(foliage_lod, ratio)

        b_tris = count_tris(bark_lod)
        f_tris = count_tris(foliage_lod)
        log(f"  {lod_name}: bark={b_tris} tris, foliage={f_tris} tris, total={b_tris+f_tris}")

        out_path = os.path.join(OUTPUT_DIR, f"field_{runtime_name}_{lod_name}.glb")
        export_glb(out_path, bark_lod, foliage_lod)
        log(f"  Exported: {out_path}")

        safe_remove(bark_lod)
        safe_remove(foliage_lod)

    safe_remove(bark_joined)
    safe_remove(foliage_joined)


def process_broadleaf(source_name, runtime_name):
    """Process a broadleaf source model (tree_small_02 / jacaranda_tree)."""
    source_path = os.path.join(SOURCE_DIR, f"{source_name}_1k.blend")
    log(f"\n{'='*60}")
    log(f"Processing: {source_name} → {runtime_name}")
    log(f"Source: {source_path}")

    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=source_path)

    # PV1-R2: Poly Haven .blend files hide objects in disabled collections.
    # Move ALL mesh objects to the scene's active collection so they are
    # selectable and joinable.
    scene_col = bpy.context.scene.collection
    for obj in list(bpy.data.objects):
        if obj.type != 'MESH':
            continue
        # Link to scene collection if not already there
        if scene_col.objects.get(obj.name) is None:
            scene_col.objects.link(obj)
        obj.hide_set(False)
        obj.hide_select = False
        obj.hide_viewport = False
    bpy.context.view_layer.update()

    all_meshes = get_mesh_objects()
    log(f"  Source meshes: {len(all_meshes)}")

    bark_parts = []
    foliage_parts = []

    for obj in all_meshes:
        # Skip massive combined LOD meshes (multi-material)
        if len(obj.data.materials) > 1:
            continue
        if not obj.data.materials:
            continue
        mat_name = obj.data.materials[0].name.lower()
        if any(k in mat_name for k in ['leave', 'leaf', 'foliage', 'crown']):
            foliage_parts.append(obj)
        elif any(k in mat_name for k in ['trunk', 'branch', 'bark']):
            bark_parts.append(obj)

    log(f"  Bark parts: {len(bark_parts)}, Foliage parts: {len(foliage_parts)}")

    bark_joined = join_objects(bark_parts, "bark_src")
    foliage_joined = join_objects(foliage_parts, "foliage_src")

    bark_tris = count_tris(bark_joined)
    foliage_tris = count_tris(foliage_joined)
    log(f"  Source bark: {bark_tris} tris, foliage: {foliage_tris} tris")

    # LOD generation — ratios tuned for runtime budget
    lod_configs = [
        (0.15, "lod0"),   # → ~15K for broadleaf
        (0.06, "lod1"),   # → ~6K
        (0.02, "lod2"),   # → ~2K
    ]

    for ratio, lod_name in lod_configs:
        bark_lod = duplicate_obj(bark_joined, f"bark_{lod_name}")
        foliage_lod = duplicate_obj(foliage_joined, f"foliage_{lod_name}")

        if bark_lod and bark_tris > 100:
            decimate_mesh(bark_lod, ratio)
        if foliage_lod and foliage_tris > 50:
            decimate_mesh(foliage_lod, ratio)

        b_tris = count_tris(bark_lod)
        f_tris = count_tris(foliage_lod)
        log(f"  {lod_name}: bark={b_tris} tris, foliage={f_tris} tris, total={b_tris+f_tris}")

        out_path = os.path.join(OUTPUT_DIR, f"field_{runtime_name}_{lod_name}.glb")
        export_glb(out_path, bark_lod, foliage_lod)
        log(f"  Exported: {out_path}")

        safe_remove(bark_lod)
        safe_remove(foliage_lod)

    safe_remove(bark_joined)
    safe_remove(foliage_joined)


# ── Main ───────────────────────────────────────────────────────────────────

def main():
    log("PV1-R2 Production Vegetation Asset Processor")
    log(f"REPO_ROOT: {REPO_ROOT}")
    log(f"SOURCE_DIR: {SOURCE_DIR}")
    log(f"OUTPUT_DIR: {OUTPUT_DIR}")

    # Verify source files exist
    for name in ['pine_tree_01_1k.blend', 'fir_tree_01_1k.blend',
                 'tree_small_02_1k.blend', 'jacaranda_tree_1k.blend']:
        path = os.path.join(SOURCE_DIR, name)
        if not os.path.isfile(path):
            log(f"ERROR: source file missing: {path}")
            return

    # Process conifers (real Poly Haven geometry)
    process_conifer('pine_tree_01', 'pine_a')
    process_conifer('fir_tree_01', 'fir_a')

    # Process broadleaves (real Poly Haven geometry)
    process_broadleaf('tree_small_02', 'broadleaf_a')
    process_broadleaf('jacaranda_tree', 'broadleaf_b')

    # Write processing log
    with open(LOG_PATH, 'w') as f:
        json.dump(LOG, f, indent=2)

    log(f"\n{'='*60}")
    log("All 4 species processed. 12 GLB files exported.")
    log(f"Output: {OUTPUT_DIR}")
    log(f"Log: {LOG_PATH}")
    log(f"{'='*60}")


if __name__ == "__main__":
    main()
