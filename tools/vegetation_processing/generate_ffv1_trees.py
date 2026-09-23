"""
FFV1: Flying Field broadleaf re-bake (Poly Haven tree_small_02, CC0).

The PV1-R3 bake decimated the leaf-card set with the same ratios as the bark
(0.12 / 0.04 / 0.015) and point-sampled the 2k leaf maps into a 512 atlas, so
the committed crowns carried a few hundred eroded cards: at tree-line distance
the trees read as bare branch skeletons (the FFV1 visual gate failure).

This bake fixes both, for the broadleaf_a asset only:

* the leaf atlas is composited at 1024 with a 2x2 AREA (box) filter in linear
  light for RGB and a plain box average for alpha, so leaf-edge coverage
  survives minification instead of aliasing away;
* the LOD0 crown is DENSIFIED: the joined source card set plus two transformed
  copies (a yawed/scaled copy and a mirrored copy) so overlapping card
  orientations form a closed crown silhouette;
* LOD1 keeps the undensified card set and LOD2 a reduced one; the renderer's
  crown-preserving LOD policy draws the LOD0 crown at every distance, so the
  decimated foliage of LOD1/2 is only a memory/LOD-switch artifact;
* bark keeps a decimation policy (0.15 / 0.06 / 0.02).

Geometry and textures both come from the Poly Haven CC0 source (see
PROVENANCE.md and ffv1_tree_source_receipt.json). Run with:

    "<blender>" --background --python tools/vegetation_processing/generate_ffv1_trees.py
"""
import bpy
import json
import os
import random
import sys

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
SOURCE_DIR = os.path.join(SCRIPT_DIR, "source_models")
TEX_DIR = os.path.join(SCRIPT_DIR, "textures", "tree_small_02")
OUTPUT_DIR = os.path.join(REPO_ROOT, "crates", "renderer", "assets", "vegetation")
ATLAS_DIR = os.path.join(SCRIPT_DIR, "textures", "ffv1_broadleaf_a")
os.makedirs(OUTPUT_DIR, exist_ok=True)
os.makedirs(ATLAS_DIR, exist_ok=True)

LEAF_ATLAS = 1024
BARK_ATLAS = 1024

LOG = []


def log(message):
    print(message, flush=True)
    LOG.append(message)


def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        if collection.users == 0:
            bpy.data.collections.remove(collection)
    for material in list(bpy.data.materials):
        if material.users == 0:
            bpy.data.materials.remove(material)
    for mesh in list(bpy.data.meshes):
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)
    for image in list(bpy.data.images):
        if image.users == 0:
            bpy.data.images.remove(image)


def link_all():
    scene_collection = bpy.context.scene.collection
    for obj in list(bpy.data.objects):
        if obj.type != 'MESH':
            continue
        if scene_collection.objects.get(obj.name) is None:
            scene_collection.objects.link(obj)
        obj.hide_set(False)
        obj.hide_select = False
        obj.hide_viewport = False
    bpy.context.view_layer.update()


def join_meshes(objects, name):
    if not objects:
        return None
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
    while len(result.data.materials) > 1:
        result.active_material_index = len(result.data.materials) - 1
        bpy.ops.object.material_slot_remove()
    return result


def tris(obj):
    return len(obj.data.polygons) if obj else 0


def decimate(obj, ratio):
    if obj is None or ratio >= 1.0:
        return obj
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    while obj.data.shape_keys and len(obj.data.shape_keys.key_blocks) > 0:
        obj.shape_key_remove(obj.data.shape_keys.key_blocks[0])
    for modifier in list(obj.modifiers):
        obj.modifiers.remove(modifier)
    modifier = obj.modifiers.new("Dec", 'DECIMATE')
    modifier.ratio = max(ratio, 0.001)
    try:
        bpy.ops.object.modifier_apply(modifier=modifier.name)
    except RuntimeError:
        pass
    return obj


def read_png_linear_rgb(path):
    """Load a PNG and return (width, height, float32 HxWx4) in LINEAR light.

    Blender already linearizes the stored sRGB bytes when exposing
    `Image.pixels` for byte images, so the values are scene-linear as-is:
    linearizing again would double-apply the transfer and darken greens into
    olive-brown (the FFV1 crown-colour bug)."""
    image = bpy.data.images.load(path, check_existing=False)
    width, height = image.size
    buffer = np.empty(width * height * 4, dtype=np.float32)
    image.pixels.foreach_get(buffer)
    buffer = buffer.reshape(height, width, 4)
    bpy.data.images.remove(image)
    return width, height, buffer


def box_downsample_rgba(buffer, width, height, target):
    """Exact 2x2-box cascade from (width, height) down to (target, target).

    RGB is averaged in linear light and re-encoded to sRGB once at the end;
    alpha is averaged directly (coverage must survive minification)."""
    factor = width // target
    assert height // target == factor and (factor & (factor - 1)) == 0
    current = buffer
    cur_w, cur_h = width, height
    while cur_w > target:
        reshaped = current.reshape(cur_h // 2, 2, cur_w // 2, 2, 4)
        current = reshaped.mean(axis=(1, 3))
        cur_w //= 2
        cur_h //= 2
    out = current.copy()
    rgb = out[..., :3]
    out[..., :3] = np.where(
        rgb <= 0.0031308, rgb * 12.92, 1.055 * np.power(np.clip(rgb, 0.0, 1.0), 1.0 / 2.4) - 0.055
    )
    return np.clip(out, 0.0, 1.0)


def write_atlas(path, rgba):
    target = rgba.shape[0]
    image = bpy.data.images.new(
        os.path.basename(path), width=target, height=target, alpha=True
    )
    image.colorspace_settings.name = 'sRGB'
    image.pixels.foreach_set(rgba.astype(np.float32).ravel())
    image.filepath_raw = path
    image.file_format = 'PNG'
    image.save()
    bpy.data.images.remove(image)
    return path


def box_downsample_raw(buffer, width, height, target):
    """Plain 2x2 box average of the stored values, no colour transfer.

    Used for coverage masks: the runtime reads the alpha lane raw, so the
    reduction must average exactly the bytes the renderer will see."""
    current = buffer
    cur_w, cur_h = width, height
    while cur_w > target:
        current = current.reshape(cur_h // 2, 2, cur_w // 2, 2, 4).mean(axis=(1, 3))
        cur_w //= 2
        cur_h //= 2
    return np.clip(current, 0.0, 1.0)


def composite_leaf_atlas():
    """RGB = leaves_diff (linear box), A = leaves_alpha R channel (raw box)."""
    diff_path = os.path.join(TEX_DIR, "tree_small_02_leaves_diff_2k.png")
    alpha_path = os.path.join(TEX_DIR, "tree_small_02_leaves_alpha_2k.png")
    dw, dh, diff = read_png_linear_rgb(diff_path)
    aw, ah, alpha = read_png_linear_rgb(alpha_path)
    diff_down = box_downsample_rgba(diff, dw, dh, LEAF_ATLAS)
    alpha_down = box_downsample_raw(alpha, aw, ah, LEAF_ATLAS)
    combined = diff_down.copy()
    # Poly Haven publishes leaf masks as grayscale images: the coverage lives
    # in the R channel (the PNG alpha lane is opaque everywhere), exactly as
    # the PV1-R3 bake read it.
    combined[..., 3] = alpha_down[..., 0]
    # Part of the source mask is inverted (neutral background flagged opaque),
    # which would render as solid white sheets. Gate the coverage by the
    # diffuse: neutral-white texels are background regardless of the mask.
    whiteness = combined[..., :3].min(axis=2)
    combined[..., 3] = combined[..., 3] * (1.0 - np.clip((whiteness - 0.55) / 0.25, 0.0, 1.0))
    # The diffuse scan has a neutral white background. Mipmapping would bleed
    # that white into the leaf texels and turn distant cards pale grey, so the
    # background RGB is filled with the mean leaf colour (alpha still carries
    # the coverage).
    leaf = combined[..., 3] > 0.5
    if leaf.any():
        mean_rgb = combined[leaf][:, :3].mean(axis=0)
        combined[..., :3] = np.where(leaf[..., None], combined[..., :3], mean_rgb)
    out = os.path.join(ATLAS_DIR, "ffv1_broadleaf_a_leaves_rgba_1k.png")
    write_atlas(out, combined)
    coverage = float((combined[..., 3] > 0.45).mean())
    log(f"  leaf atlas 1024: coverage(alpha>0.45) = {coverage:.3f}")
    return out


def composite_bark_atlas():
    diff_path = os.path.join(TEX_DIR, "tree_small_02_branch_diff_2k.png")
    dw, dh, diff = read_png_linear_rgb(diff_path)
    down = box_downsample_rgba(diff, dw, dh, BARK_ATLAS)
    down[..., 3] = 1.0
    out = os.path.join(ATLAS_DIR, "ffv1_broadleaf_a_branch_rgba_1k.png")
    write_atlas(out, down)
    return out


def make_material(name, image_path, alpha_clip):
    material = bpy.data.materials.new(name=name)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()
    out = nodes.new('ShaderNodeOutputMaterial')
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.inputs['Roughness'].default_value = 0.7
    bsdf.inputs['Metallic'].default_value = 0.0
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])
    image = bpy.data.images.load(image_path, check_existing=False)
    image.pack()
    tex = nodes.new('ShaderNodeTexImage')
    tex.image = image
    tex.interpolation = 'Smart'
    links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    if alpha_clip:
        # The runtime does its own alpha cutoff; this only documents intent
        # for DCC viewers and survives as glTF alphaMode MASK.
        try:
            material.blend_method = 'CLIP'
            material.alpha_threshold = 0.45
        except (AttributeError, TypeError):
            # Blender 4.2+ EEVEE-Next removed blend_method; the glTF exporter
            # falls back to its own mask detection from the alpha channel.
            pass
    return material


def strip_vertex_colors(obj):
    """Drop baked COLOR_0 attributes. The Poly Haven scans carry vertex
    colours that tint the foliage brown/autumnal; the runtime multiplies them
    into the base-color texture, which is exactly what made the crowns read as
    dead wood. The atlas is the colour authority for FFV1 vegetation."""
    if obj is None:
        return
    for attribute in list(obj.data.color_attributes):
        obj.data.color_attributes.remove(attribute)


def duplicate_transformed(obj, name, euler, scale, offset, pivot):
    """Copy the crown card set with a full-axis rotation, uniform scale and
    small translation about the crown pivot. Scattering oriented copies (not
    just yaw spins) is what fills the crown silhouette: a yaw-only copy lands
    inside the same outline and adds no perceived leaf mass."""
    import math
    from mathutils import Matrix
    mesh = obj.data.copy()
    copy = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(copy)
    copy.matrix_world = obj.matrix_world.copy()
    to_pivot = Matrix.Translation(-pivot)
    from_pivot = Matrix.Translation(pivot + offset)
    rot = (
        Matrix.Rotation(euler[0], 4, 'X')
        @ Matrix.Rotation(euler[1], 4, 'Y')
        @ Matrix.Rotation(euler[2], 4, 'Z')
    )
    scl = Matrix.Diagonal((scale, scale, scale, 1.0))
    copy.matrix_world = from_pivot @ rot @ scl @ to_pivot @ copy.matrix_world
    return copy


def crown_pivot(obj):
    from mathutils import Vector
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    centre = sum(corners, Vector()) / 8.0
    return centre


def export_glb(path, bark, foliage):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in [bark, foliage]:
        if obj:
            obj.select_set(True)
    bpy.context.view_layer.objects.active = bark or foliage
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
        export_materials='EXPORT',
    )


def main():
    log("FFV1 broadleaf_a re-bake (tree_small_02, CC0)")
    leaf_atlas = composite_leaf_atlas()
    bark_atlas = composite_bark_atlas()

    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=os.path.join(SOURCE_DIR, "tree_small_02_1k.blend"))
    link_all()

    meshes = [o for o in bpy.data.objects if o.type == 'MESH']
    bark_objects, foliage_objects = [], []
    for obj in meshes:
        if len(obj.data.materials) != 1 or not obj.data.materials:
            continue
        material_name = obj.data.materials[0].name.lower()
        if any(k in material_name for k in ('trunk', 'branch')):
            bark_objects.append(obj)
        elif any(k in material_name for k in ('leave', 'leaf', 'foliage')):
            foliage_objects.append(obj)
    log(f"  source objects: bark={len(bark_objects)} foliage={len(foliage_objects)}")

    bark = join_meshes(bark_objects, "bark")
    foliage = join_meshes(foliage_objects, "foliage")
    source_bark_tris, source_foliage_tris = tris(bark), tris(foliage)
    log(f"  source tris: bark={source_bark_tris} foliage={source_foliage_tris}")

    bark_material = make_material("ffv1_bark", bark_atlas, False)
    foliage_material = make_material("ffv1_foliage", leaf_atlas, True)

    pivot = crown_pivot(foliage) if foliage else None

    lods = [
        # (lod name, bark ratio, foliage copies, foliage decimate)
        ("lod0", 0.15, 8, 1.0),
        ("lod1", 0.06, 4, 1.0),
        ("lod2", 0.02, 1, 0.35),
    ]
    # The source foliage cards are single leaves (~10 cm): sub-pixel at the
    # 150-230 m tree line, where mipmapped alpha discards them and the crown
    # collapses to bare branches. Scaling the card set about the crown pivot
    # turns single leaves into leaf clusters (~30 cm), which is the card size
    # real-time crowns use, so the silhouette keeps its leaf mass at distance.
    FOLIAGE_CARD_SCALE = 2.2
    scatter = random.Random(42)
    from mathutils import Vector
    for lod, bark_ratio, copies, foliage_ratio in lods:
        bark_lod = decimate(
            duplicate_transformed(bark, f"bark_{lod}", (0.0, 0.0, 0.0), 1.0, Vector((0, 0, 0)), pivot)
            if pivot is not None
            else bark,
            bark_ratio,
        )
        foliage_pieces = []
        for index in range(copies):
            if index == 0:
                euler, offset = (0.0, 0.0, 0.0), Vector((0, 0, 0))
                scale = FOLIAGE_CARD_SCALE
            else:
                euler = (
                    scatter.uniform(-0.9, 0.9),
                    scatter.uniform(-0.9, 0.9),
                    scatter.uniform(0.0, 6.283),
                )
                scale = FOLIAGE_CARD_SCALE * scatter.uniform(0.55, 0.95)
                offset = Vector(
                    (
                        scatter.uniform(-0.6, 0.6),
                        scatter.uniform(-0.6, 0.6),
                        scatter.uniform(-0.4, 0.5),
                    )
                )
            piece = duplicate_transformed(
                foliage, f"foliage_{lod}_{index}", euler, scale, offset, pivot
            )
            foliage_pieces.append(piece)
        foliage_lod = join_meshes(foliage_pieces, f"foliage_{lod}")
        if foliage_ratio < 1.0:
            foliage_lod = decimate(foliage_lod, foliage_ratio)

        if bark_lod:
            bark_lod.data.materials.clear()
            bark_lod.data.materials.append(bark_material)
            strip_vertex_colors(bark_lod)
        if foliage_lod:
            foliage_lod.data.materials.clear()
            foliage_lod.data.materials.append(foliage_material)
            strip_vertex_colors(foliage_lod)

        bark_tris, foliage_tris = tris(bark_lod), tris(foliage_lod)
        log(f"  {lod}: bark={bark_tris} foliage={foliage_tris} total={bark_tris + foliage_tris}")
        export_glb(
            os.path.join(OUTPUT_DIR, f"field_broadleaf_a_{lod}.glb"),
            bark_lod,
            foliage_lod,
        )
        if bark_lod and bark_lod.name in bpy.data.objects:
            bpy.data.objects.remove(bark_lod, do_unlink=True)
        if foliage_lod and foliage_lod.name in bpy.data.objects:
            bpy.data.objects.remove(foliage_lod, do_unlink=True)

    with open(os.path.join(SCRIPT_DIR, "ffv1_processing_log.json"), 'w') as handle:
        json.dump(LOG, handle, indent=2)
    log("done")


if __name__ == "__main__":
    main()
