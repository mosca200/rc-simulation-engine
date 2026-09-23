"""
FFV1: ground-cover cluster bake (Poly Haven grass_medium_01/02, CC0).

Produces two runtime grass-cluster assets (field_grass_a / field_grass_b) with
three LODs each, for the distance-graded ground-cover belt. The source models
are already light, so LOD0 keeps the authored card set untouched; LOD1/LOD2 are
decimated for the mid band and the LOD-switch artifact.

The diffuse/alpha maps are composited at 1024 with a 2x2 AREA (box) filter
(linear-light RGB, plain average alpha) so blade coverage survives
minification. Run with:

    "<blender>" --background --python tools/vegetation_processing/generate_ffv1_grass.py
"""
import bpy
import bmesh
import json
import os

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
SOURCE_DIR = os.path.join(SCRIPT_DIR, "source_models")
TEX_DIR = os.path.join(SCRIPT_DIR, "textures")
OUTPUT_DIR = os.path.join(REPO_ROOT, "crates", "renderer", "assets", "vegetation")
ATLAS = 1024

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
    olive-brown."""
    image = bpy.data.images.load(path, check_existing=False)
    width, height = image.size
    buffer = np.empty(width * height * 4, dtype=np.float32)
    image.pixels.foreach_get(buffer)
    buffer = buffer.reshape(height, width, 4)
    bpy.data.images.remove(image)
    return width, height, buffer


def box_downsample_rgba(buffer, width, height, target):
    current = buffer
    cur_w, cur_h = width, height
    while cur_w > target:
        current = current.reshape(cur_h // 2, 2, cur_w // 2, 2, 4).mean(axis=(1, 3))
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
    image = bpy.data.images.new(os.path.basename(path), width=target, height=target, alpha=True)
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


def composite_atlas(slug):
    diff_path = os.path.join(TEX_DIR, slug, f"{slug}_diff_2k.png")
    alpha_path = os.path.join(TEX_DIR, slug, f"{slug}_alpha_2k.png")
    dw, dh, diff = read_png_linear_rgb(diff_path)
    aw, ah, alpha = read_png_linear_rgb(alpha_path)
    combined = box_downsample_rgba(diff, dw, dh, ATLAS)
    # The coverage mask lives in the R channel of the grayscale mask PNG.
    combined[..., 3] = box_downsample_raw(alpha, aw, ah, ATLAS)[..., 0]
    # Fill the neutral scan background with the mean blade colour so mip
    # filtering cannot bleed white into distant tufts.
    blade = combined[..., 3] > 0.5
    if blade.any():
        mean_rgb = combined[blade][:, :3].mean(axis=0)
        combined[..., :3] = np.where(blade[..., None], combined[..., :3], mean_rgb)
    out = os.path.join(TEX_DIR, f"ffv1_{slug}_rgba_1k.png")
    write_atlas(out, combined)
    log(f"  {slug} atlas coverage(alpha>0.45) = {float((combined[..., 3] > 0.45).mean()):.3f}")
    return out


def make_material(name, image_path):
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
    try:
        material.blend_method = 'CLIP'
        material.alpha_threshold = 0.45
    except (AttributeError, TypeError):
        pass
    return material


def duplicate(obj, name):
    mesh = obj.data.copy()
    copy = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(copy)
    copy.matrix_world = obj.matrix_world.copy()
    return copy


def recenter(obj):
    """Bake any node transform into the mesh, then move the cluster so its
    ground contact sits at z=0 (Blender is Z-up; the glTF exporter converts
    that to Y-up for the renderer) and its X/Y bbox centre at the origin: the
    renderer instances clusters at their placement position, so an off-centre
    source scan would offset every tuft by metres."""
    from mathutils import Matrix, Vector
    if tuple(obj.matrix_world) != tuple(Matrix.Identity(4)):
        obj.data.transform(obj.matrix_world)
        obj.matrix_world = Matrix.Identity(4)
    local = [v.co for v in obj.data.vertices]
    centre_x = 0.5 * (min(c.x for c in local) + max(c.x for c in local))
    centre_y = 0.5 * (min(c.y for c in local) + max(c.y for c in local))
    min_z = min(c.z for c in local)
    obj.data.transform(Matrix.Translation(Vector((-centre_x, -centre_y, -min_z))))


def strip_vertex_colors(obj):
    """Drop baked COLOR_0 attributes: the scans carry vertex colours that tint
    the blades brown, and the runtime multiplies them into the base-color
    texture. The atlas is the colour authority for FFV1 vegetation."""
    if obj is None:
        return
    for attribute in list(obj.data.color_attributes):
        obj.data.color_attributes.remove(attribute)


def split_by_world_height(obj, name, z_cut, keep_below):
    """Copy `obj` keeping only the faces whose world-space centre is below
    (or above) `z_cut`. The renderer's vegetation contract is two primitives
    (bark + foliage); for a grass cluster the lower stem band plays the bark
    role and the blade mass the foliage role."""
    mesh = obj.data.copy()
    copy = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(copy)
    copy.matrix_world = obj.matrix_world.copy()
    bm = bmesh.new()
    bm.from_mesh(mesh)
    doomed = []
    for face in bm.faces:
        centre_z = (copy.matrix_world @ face.calc_center_median()).z
        if keep_below and centre_z >= z_cut:
            doomed.append(face)
        if not keep_below and centre_z < z_cut:
            doomed.append(face)
    bmesh.ops.delete(bm, geom=doomed, context='FACES')
    bm.to_mesh(mesh)
    bm.free()
    return copy


def export_glb(path, bark, foliage):
    bpy.ops.object.select_all(action='DESELECT')
    for obj in (bark, foliage):
        obj.select_set(True)
    bpy.context.view_layer.objects.active = bark
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format='GLB',
        use_selection=True,
        export_apply=True,
        export_materials='EXPORT',
    )


def process(slug, runtime_name, lod_ratios):
    log(f"\n{'=' * 60}\n{slug} -> {runtime_name}")
    atlas = composite_atlas(slug)
    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=os.path.join(SOURCE_DIR, f"{slug}_1k.blend"))
    link_all()
    meshes = [o for o in bpy.data.objects if o.type == 'MESH']
    cluster = join_meshes(meshes, "cluster")
    recenter(cluster)
    post = [v.co for v in cluster.data.vertices]
    log(
        f"  recentered bbox z=[{min(v.z for v in post):.3f}, {max(v.z for v in post):.3f}] "
        f"x=[{min(v.x for v in post):.3f}, {max(v.x for v in post):.3f}]"
    )
    log(f"  source tris={tris(cluster)}")
    material = make_material(f"ffv1_{runtime_name}", atlas)
    cluster.data.materials.clear()
    cluster.data.materials.append(material)

    world_zs = [v.co.z for v in cluster.data.vertices]
    z_min, z_max = min(world_zs), max(world_zs)
    z_cut = z_min + 0.25 * (z_max - z_min)

    for lod, ratio in lod_ratios:
        piece = duplicate(cluster, f"cluster_{lod}")
        if ratio < 1.0:
            piece = decimate(piece, ratio)
        piece.data.materials.clear()
        piece.data.materials.append(material)
        stem = split_by_world_height(piece, f"stem_{lod}", z_cut, True)
        blades = split_by_world_height(piece, f"blades_{lod}", z_cut, False)
        for part in (stem, blades):
            part.data.materials.clear()
            part.data.materials.append(material)
            strip_vertex_colors(part)
        log(f"  {lod}: stem={tris(stem)} blades={tris(blades)} total={tris(stem) + tris(blades)}")
        export_glb(os.path.join(OUTPUT_DIR, f"field_{runtime_name}_{lod}.glb"), stem, blades)
        for part in (piece, stem, blades):
            if part.name in bpy.data.objects:
                bpy.data.objects.remove(part, do_unlink=True)
    bpy.data.objects.remove(cluster, do_unlink=True)


def main():
    log("FFV1 ground-cover bake (grass_medium_01/02, CC0)")
    # The authored scans are far denser than an instanced ground-cover budget
    # allows; the ratios keep LOD0 tufts readable at 2-25 m while staying
    # inside a few thousand triangles per cluster.
    process(
        "grass_medium_01",
        "grass_a",
        [("lod0", 0.25), ("lod1", 0.10), ("lod2", 0.04)],
    )
    process(
        "grass_medium_02",
        "grass_b",
        [("lod0", 0.50), ("lod1", 0.20), ("lod2", 0.08)],
    )
    with open(os.path.join(SCRIPT_DIR, "ffv1_grass_processing_log.json"), 'w') as handle:
        json.dump(LOG, handle, indent=2)
    log("done")


if __name__ == "__main__":
    main()
