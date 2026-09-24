"""
FFV1: Flying Field broadleaf re-bake (Poly Haven tree_small_02, CC0).

The Poly Haven source stores its complete leafy tree in a Geometry Nodes curve.
The earlier FFV1 bake joined only the eight small leaf prototype meshes and
missed the evaluated crown; those prototypes cannot fill the tree line even
when duplicated. This field-specific bake renders the evaluated source leaves
to a transparent green crown card and embeds crossed cards in the existing
three GLBs. Bark still comes from the CC0 source meshes.

For the broadleaf_a asset only:

* the evaluated source canopy supplies the card silhouette, rather than the
  source's undistributed prototype leaf meshes;
* LOD0/1/2 use four/three/two crossed cards, all from the same source render;
* bark keeps a decimation policy (0.15 / 0.06 / 0.02).

Geometry and textures both come from the Poly Haven CC0 source (see
PROVENANCE.md). Run with:

    "<blender>" --background --python tools/vegetation_processing/generate_ffv1_trees.py
"""
import bpy
import json
import os
import sys

import numpy as np
from mathutils import Vector

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
    # The source's pale branches became near-white lines at pilot distance.
    # A deeper warm-brown bark keeps the trunk legible without leading the view.
    down[..., :3] *= np.array([0.46, 0.39, 0.32], dtype=np.float32)
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
            obj.hide_render = False
            obj.select_set(True)
    bpy.context.view_layer.objects.active = bark or foliage
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
        export_materials='EXPORT',
    )


def render_crown_atlas():
    """Rasterize the evaluated CC0 Geometry Nodes leaves, excluding bark."""
    tree = bpy.data.objects['tree_small_02_geometry_nodes']
    for obj in bpy.data.objects:
        obj.hide_render = obj != tree

    camera_data = bpy.data.cameras.new('ffv1_crown_camera')
    camera = bpy.data.objects.new('ffv1_crown_camera', camera_data)
    bpy.context.scene.collection.objects.link(camera)
    camera.location = (0.0, -12.0, 3.0)
    camera.rotation_euler = (Vector((0.0, 0.0, 3.0)) - camera.location).to_track_quat('-Z', 'Y').to_euler()
    camera_data.type = 'ORTHO'
    camera_data.ortho_scale = 3.6

    sun_data = bpy.data.lights.new('ffv1_crown_sun', 'SUN')
    sun_data.energy = 1.6
    sun = bpy.data.objects.new('ffv1_crown_sun', sun_data)
    bpy.context.scene.collection.objects.link(sun)
    sun.rotation_euler = (0.5, -0.5, -0.5)

    for material in bpy.data.materials:
        nodes = material.node_tree.nodes
        links = material.node_tree.links
        nodes.clear()
        output = nodes.new('ShaderNodeOutputMaterial')
        if 'leaves' in material.name:
            shader = nodes.new('ShaderNodeBsdfPrincipled')
            shader.inputs['Base Color'].default_value = (0.13, 0.35, 0.10, 1.0)
            shader.inputs['Roughness'].default_value = 0.9
        else:
            shader = nodes.new('ShaderNodeBsdfTransparent')
        links.new(shader.outputs['BSDF'], output.inputs['Surface'])

    scene = bpy.context.scene
    scene.camera = camera
    scene.render.engine = 'BLENDER_EEVEE'
    scene.render.resolution_x = 1024
    scene.render.resolution_y = 1024
    scene.render.resolution_percentage = 100
    scene.render.film_transparent = True
    scene.render.image_settings.file_format = 'PNG'
    scene.render.image_settings.color_mode = 'RGBA'
    scene.world.color = (0.3, 0.3, 0.3)
    path = os.path.join(ATLAS_DIR, 'ffv1_broadleaf_a_crown_rgba_1k.png')
    scene.render.filepath = path
    bpy.ops.render.render(write_still=True)
    log(f'  evaluated source crown atlas: {path}')
    return path


def crown_cards(name, material, card_count):
    """Small field-specific crossed-card crown, centred on the source canopy."""
    import math
    vertices, faces, uvs = [], [], []
    for index in range(card_count):
        angle = math.pi * index / card_count
        dx, dy = math.cos(angle) * 1.8, math.sin(angle) * 1.8
        base = len(vertices)
        vertices.extend([
            (-dx, -dy, 1.2), (dx, dy, 1.2),
            (dx, dy, 4.8), (-dx, -dy, 4.8),
        ])
        faces.append((base, base + 1, base + 2, base + 3))
        uvs.extend([(0, 0), (1, 0), (1, 1), (0, 1)])
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    uv_layer = mesh.uv_layers.new(name='UVMap')
    for polygon in mesh.polygons:
        for corner, loop_index in enumerate(polygon.loop_indices):
            uv_layer.data[loop_index].uv = uvs[polygon.index * 4 + corner]
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    obj.data.materials.append(material)
    return obj


def main():
    log("FFV1 broadleaf_a re-bake (tree_small_02, CC0)")
    bark_atlas = composite_bark_atlas()

    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=os.path.join(SOURCE_DIR, "tree_small_02_1k.blend"))
    crown_atlas = render_crown_atlas()
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
    foliage_material = make_material("ffv1_foliage", crown_atlas, True)

    pivot = crown_pivot(foliage) if foliage else None

    lods = [
        # (lod name, bark ratio, crossed crown cards)
        ("lod0", 0.15, 4),
        ("lod1", 0.06, 3),
        ("lod2", 0.02, 2),
    ]
    for lod, bark_ratio, cards in lods:
        bark_lod = decimate(
            duplicate_transformed(bark, f"bark_{lod}", (0.0, 0.0, 0.0), 1.0, Vector((0, 0, 0)), pivot)
            if pivot is not None
            else bark,
            bark_ratio,
        )
        foliage_lod = crown_cards(f"foliage_{lod}", foliage_material, cards)

        if bark_lod:
            bark_lod.data.materials.clear()
            bark_lod.data.materials.append(bark_material)
            strip_vertex_colors(bark_lod)
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
