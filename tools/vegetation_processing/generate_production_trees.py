"""
PV1-R3: Production vegetation asset processor.

Imports REAL Poly Haven CC0 .blend source models for geometry, loads
CC0 textures from disk, creates proper PBR materials, exports GLB
with embedded base-color textures (bark diffuse + foliage RGBA).
"""
import bpy, bmesh, os, json, struct
from mathutils import Vector

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
SOURCE_DIR = os.path.join(SCRIPT_DIR, "source_models")
TEX_DIR = os.path.join(SCRIPT_DIR, "textures")
OUTPUT_DIR = os.path.join(REPO_ROOT, "crates", "renderer", "assets", "vegetation")
os.makedirs(OUTPUT_DIR, exist_ok=True)

LOG = []
def log(m): print(m, flush=True); LOG.append(m)

# Texture manifest: maps species to texture files on disk
MANIFEST = {
    'pine_a': {
        'bark_diff': os.path.join(TEX_DIR, 'pine_tree_01', 'bark_diff_2k.jpg'),
        'foliage_diff': os.path.join(TEX_DIR, 'pine_tree_01', 'twig_diff_2k.jpg'),
        'foliage_alpha': os.path.join(TEX_DIR, 'pine_tree_01', 'twig_alpha_2k.jpg'),
    },
    'fir_a': {
        'bark_diff': os.path.join(TEX_DIR, 'fir_tree_01', 'bark_diff_2k.jpg'),
        'foliage_diff': os.path.join(TEX_DIR, 'fir_tree_01', 'twig_diff_2k.jpg'),
        'foliage_alpha': os.path.join(TEX_DIR, 'fir_tree_01', 'twig_alpha_2k.jpg'),
    },
    'broadleaf_a': {
        'bark_diff': os.path.join(TEX_DIR, 'tree_small_02', 'branch_diff_2k.jpg'),
        'foliage_diff': os.path.join(TEX_DIR, 'tree_small_02', 'leaves_diff_2k.jpg'),
        'foliage_alpha': os.path.join(TEX_DIR, 'tree_small_02', 'leaves_alpha_2k.jpg'),
    },
    'broadleaf_b': {
        'bark_diff': os.path.join(TEX_DIR, 'jacaranda_tree', 'branches_diff_2k.jpg'),
        'foliage_diff': os.path.join(TEX_DIR, 'jacaranda_tree', 'leaves_diff_2k.jpg'),
        'foliage_alpha': os.path.join(TEX_DIR, 'jacaranda_tree', 'leaves_alpha_2k.jpg'),
    },
}

def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    for c in list(bpy.data.collections):
        if c.users == 0: bpy.data.collections.remove(c)
    for m in list(bpy.data.materials):
        if m.users == 0: bpy.data.materials.remove(m)
    for m in list(bpy.data.meshes):
        if m.users == 0: bpy.data.meshes.remove(m)
    for i in list(bpy.data.images):
        if i.users == 0: bpy.data.images.remove(i)

def link_all():
    sc = bpy.context.scene.collection
    for o in list(bpy.data.objects):
        if o.type != 'MESH': continue
        if sc.objects.get(o.name) is None: sc.objects.link(o)
        o.hide_set(False); o.hide_select = False; o.hide_viewport = False
    bpy.context.view_layer.update()

def join_meshes(objects, name):
    if not objects: return None
    for o in objects:
        o.hide_set(False); o.hide_select = False; o.hide_viewport = False
    bpy.ops.object.select_all(action='DESELECT')
    for o in objects: o.select_set(True)
    bpy.context.view_layer.objects.active = objects[0]
    if len(objects) > 1: bpy.ops.object.join()
    r = bpy.context.active_object; r.name = name
    # Consolidate to single material slot
    while len(r.data.materials) > 1:
        r.active_material_index = len(r.data.materials) - 1
        bpy.ops.object.material_slot_remove()
    return r

def load_image(path):
    img = bpy.data.images.load(path, check_existing=True)
    img.pack()
    return img

def create_pbr_material(name, diff_path, alpha_path=None, tex_size=512):
    """Create a PBR material with base-color texture.
    If alpha_path given, creates combined RGBA (RGB=diff, A=alpha)."""
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    out = nodes.new('ShaderNodeOutputMaterial')
    out.location = (400, 0)
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')
    bsdf.location = (0, 0)
    bsdf.inputs['Roughness'].default_value = 0.7
    bsdf.inputs['Metallic'].default_value = 0.0
    links.new(bsdf.outputs['BSDF'], out.inputs['Surface'])

    diff_img = bpy.data.images.load(diff_path, check_existing=True)

    if alpha_path:
        alpha_img = bpy.data.images.load(alpha_path, check_existing=True)
        # Create combined RGBA image
        rgba = bpy.data.images.new(f"{name}_rgba", width=tex_size, height=tex_size, alpha=True)
        rgba.colorspace_settings.name = 'sRGB'

        dw, dh = diff_img.size
        aw, ah = alpha_img.size
        dp = list(diff_img.pixels)
        ap = list(alpha_img.pixels)

        pixels = [0.0] * (tex_size * tex_size * 4)
        for y in range(tex_size):
            sy = int(y * dh / tex_size)
            ay_ = int(y * ah / tex_size)
            for x in range(tex_size):
                sx = int(x * dw / tex_size)
                si = (sy * dw + sx) * 4
                ti = (y * tex_size + x) * 4
                if si + 2 < len(dp):
                    pixels[ti] = dp[si]
                    pixels[ti+1] = dp[si+1]
                    pixels[ti+2] = dp[si+2]
                ax = int(x * aw / tex_size)
                ai = (ay_ * aw + ax) * 4
                pixels[ti+3] = ap[ai] if ai < len(ap) else 1.0

        rgba.pixels.foreach_set(pixels)
        rgba.pack()
        rgba.update()
        base_img = rgba
    else:
        base_img = diff_img

    tex = nodes.new('ShaderNodeTexImage')
    tex.image = base_img
    tex.location = (-300, 0)
    tex.interpolation = 'Smart'
    links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])

    # Set alpha mode for foliage
    if alpha_path:
        mat.blend_method = 'CLIP'
        mat.alpha_threshold = 0.45

    return mat

def decimate(obj, ratio, target_tris=None):
    """Standard decimate. No remesh fallback (too destructive)."""
    if obj is None or ratio >= 1.0: return obj
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    while obj.data.shape_keys and len(obj.data.shape_keys.key_blocks) > 0:
        obj.shape_key_remove(obj.data.shape_keys.key_blocks[0])
    for m in list(obj.modifiers): obj.modifiers.remove(m)
    mod = obj.modifiers.new("Dec", 'DECIMATE')
    mod.ratio = max(ratio, 0.001)
    try: bpy.ops.object.modifier_apply(modifier=mod.name)
    except: pass
    return obj

def tris(o): return len(o.data.polygons) if o else 0
def dup(o, n):
    if not o: return None
    bpy.ops.object.select_all(action='DESELECT')
    o.hide_set(False); o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.object.duplicate()
    d = bpy.context.active_object; d.name = n; return d
def rm(o):
    if o and o.name in bpy.data.objects: bpy.data.objects.remove(o, do_unlink=True)

def export_glb(path, bark, foliage):
    bpy.ops.object.select_all(action='DESELECT')
    for o in [bark, foliage]:
        if o: o.select_set(True)
    bpy.context.view_layer.objects.active = bark or foliage
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB',
        use_selection=True, export_apply=True, export_materials='EXPORT')

def process(source_name, runtime_name, bark_kw, foliage_kw):
    src = os.path.join(SOURCE_DIR, f"{source_name}_1k.blend")
    tex = MANIFEST[runtime_name]
    log(f"\n{'='*60}\n{source_name} → {runtime_name}")

    clear_scene()
    bpy.ops.wm.open_mainfile(filepath=src)
    link_all()

    meshes = [o for o in bpy.data.objects if o.type == 'MESH']
    bp, fp = [], []
    for o in meshes:
        if len(o.data.materials) != 1 or not o.data.materials: continue
        mn = o.data.materials[0].name.lower()
        if any(k in mn for k in bark_kw): bp.append(o)
        elif any(k in mn for k in foliage_kw): fp.append(o)
    log(f"  Bark:{len(bp)} Foliage:{len(fp)}")

    # Save mesh data BEFORE any join (join destroys non-active objects)
    src_bark_data = [(o.data.name, o.matrix_world.copy()) for o in bp if o.data]
    src_foliage_data = [(o.data.name, o.matrix_world.copy()) for o in fp if o.data]

    bark = join_meshes(bp, "bark")
    foliage = join_meshes(fp, "foliage")

    # Create PBR materials with textures from disk
    bark_mat = create_pbr_material("bark", tex['bark_diff'])
    foliage_mat = create_pbr_material("foliage", tex['foliage_diff'],
                                       tex.get('foliage_alpha'))

    # Assign materials
    if bark:
        bark.data.materials.clear()
        bark.data.materials.append(bark_mat)
    if foliage:
        foliage.data.materials.clear()
        foliage.data.materials.append(foliage_mat)

    bt, ft = tris(bark), tris(foliage)
    log(f"  Source: bark={bt} foliage={ft}")

    for ratio, lod in [(0.12,"lod0"),(0.04,"lod1"),(0.015,"lod2")]:
        target_total = int((bt + ft) * ratio)
        # Create fresh duplicates from source data for each LOD
        bark_dups = []
        for i, (dname, mx) in enumerate(src_bark_data):
            mesh = bpy.data.meshes.get(dname)
            if not mesh: continue
            obj = bpy.data.objects.new(f"bd_{lod}_{i}", mesh.copy())
            bpy.context.scene.collection.objects.link(obj)
            obj.matrix_world = mx
            if tris(obj) > 50: decimate(obj, ratio)
            bark_dups.append(obj)
        foliage_dups = []
        for i, (dname, mx) in enumerate(src_foliage_data):
            mesh = bpy.data.meshes.get(dname)
            if not mesh: continue
            obj = bpy.data.objects.new(f"fd_{lod}_{i}", mesh.copy())
            bpy.context.scene.collection.objects.link(obj)
            obj.matrix_world = mx
            if tris(obj) > 20: decimate(obj, ratio)
            foliage_dups.append(obj)

        bl = join_meshes(bark_dups, f"bark_{lod}")
        fl = join_meshes(foliage_dups, f"foliage_{lod}")

        if bl:
            bl.data.materials.clear(); bl.data.materials.append(bark_mat)
        if fl:
            fl.data.materials.clear(); fl.data.materials.append(foliage_mat)

        b_, f_ = tris(bl), tris(fl)
        log(f"  {lod}: bark={b_} foliage={f_} total={b_+f_}")
        export_glb(os.path.join(OUTPUT_DIR, f"field_{runtime_name}_{lod}.glb"), bl, fl)
        rm(bl); rm(fl)
    rm(bark); rm(foliage)

def main():
    log("PV1-R3 Processor (textures from disk)")
    log(f"REPO_ROOT: {REPO_ROOT}")

    process('pine_tree_01', 'pine_a',
            ['trunk','bark','dead'], ['twig','needle','branch'])
    process('fir_tree_01', 'fir_a',
            ['trunk','bark','dead'], ['twig','needle','branch'])
    process('tree_small_02', 'broadleaf_a',
            ['trunk','branch'], ['leave','leaf','foliage'])
    process('jacaranda_tree', 'broadleaf_b',
            ['trunk','branch'], ['leave','leaf','foliage','crown'])

    log(f"\n{'='*60}\nDone. 12 GLB → {OUTPUT_DIR}")
    with open(os.path.join(SCRIPT_DIR, "processing_log.json"), 'w') as f:
        json.dump(LOG, f, indent=2)

if __name__ == "__main__":
    main()
