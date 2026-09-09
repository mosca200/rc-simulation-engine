# PV1-R2 Vegetation Asset Provenance

## Summary

All vegetation assets derive from **Poly Haven** CC0 3D models. Both geometry
AND textures come from the Poly Haven source .blend files. No procedural
reconstruction is performed — the runtime GLBs contain decimated versions of
the authored Poly Haven mesh topology.

**License:** CC0 (Public Domain) — https://polyhaven.com/license
**Processing date:** 2026-09-09
**Tool:** Blender 4.2.16 LTS portable + Python processing script

---

## Asset Details

### field_pine_a (Conifer)

| Field | Value |
|---|---|
| **Source model** | pine_tree_01 |
| **Source page** | https://polyhaven.com/a/pine_tree_01 |
| **Author** | Poly Haven |
| **License** | CC0 |
| **Source format** | .blend (Blender 404.32) |
| **Source geometry** | ~114K tris (105K bark + 9K foliage, 54 mesh objects) |
| **Source textures** | bark_diff, bark_rough, bark_nor_gl, trunk_a/b/c variants, twig_diff, twig_alpha, twig_rough, twig_nor_gl (1K embedded) |
| **Processing** | Imported .blend → separated bark (trunk+dead branches) / foliage (twigs+needles) by material → decimated for LODs → consolidated to 2 materials → exported GLB |
| **LOD0** | ~24K tris (23K bark + 1K foliage) |
| **LOD1** | ~10K tris |
| **LOD2** | ~4K tris |
| **Runtime GLB sizes** | LOD0: ~4.3MB, LOD1: ~4.3MB, LOD2: ~4.3MB |

### field_fir_a (Conifer)

| Field | Value |
|---|---|
| **Source model** | fir_tree_01 |
| **Source page** | https://polyhaven.com/a/fir_tree_01 |
| **Author** | Poly Haven |
| **License** | CC0 |
| **Source format** | .blend (Blender 404.32) |
| **Source geometry** | ~107K tris (105K bark + 1.3K foliage, 47 mesh objects) |
| **Source textures** | bark_diff, bark_rough, bark_nor_gl, trunk_a/b variants, twig_diff, twig_alpha, twig_rough (1K embedded) |
| **Processing** | Same pipeline as pine_a |
| **LOD0** | ~24K tris (23K bark + 0.3K foliage) |
| **LOD1** | ~10K tris |
| **LOD2** | ~4K tris |
| **Runtime GLB sizes** | LOD0: ~4.0MB, LOD1: ~4.0MB, LOD2: ~4.0MB |

### field_broadleaf_a (Deciduous)

| Field | Value |
|---|---|
| **Source model** | tree_small_02 |
| **Source page** | https://polyhaven.com/a/tree_small_02 |
| **Author** | Poly Haven |
| **License** | CC0 |
| **Source format** | .blend (Blender 404.32) |
| **Source geometry** | ~30K tris (28K trunk + 2.3K leaf cards, 11 mesh objects) |
| **Source textures** | branch_diff, branch_rough, branch_nor_gl, leaves_diff, leaves_alpha, leaves_rough, leaves_nor_gl, trunk_diff, trunk_rough (1K embedded) |
| **Processing** | Imported .blend → separated trunk/branch (bark) / leaf cards (foliage) → decimated for LODs → consolidated to 2 materials → exported GLB |
| **LOD0** | ~5K tris (4K bark + 0.6K foliage) |
| **LOD1** | ~2K tris |
| **LOD2** | ~0.7K tris |
| **Runtime GLB sizes** | LOD0: ~2.4MB, LOD1: ~2.4MB, LOD2: ~2.4MB |

### field_broadleaf_b (Deciduous)

| Field | Value |
|---|---|
| **Source model** | jacaranda_tree |
| **Source page** | https://polyhaven.com/a/jacaranda_tree |
| **Author** | Poly Haven |
| **License** | CC0 |
| **Source format** | .blend (Blender 404.32) |
| **Source geometry** | ~213K tris (213K bark + 0.1K foliage, 15 mesh objects) |
| **Source textures** | branches_diff, branches_rough, trunk_diff, trunk_rough, leaves_diff, leaves_alpha, leaves_rough (1K embedded) |
| **Processing** | Same pipeline; note: decimation limited on this model (LODs similar tri count) |
| **LOD0** | ~58K tris |
| **LOD1** | ~58K tris |
| **LOD2** | ~58K tris |
| **Runtime GLB sizes** | LOD0: large, LOD1: large, LOD2: large |

---

## Regenerating Assets

```bash
# Source .blend files must be in tools/vegetation_processing/source_models/
# Download from Poly Haven (1K .blend format):
#   pine_tree_01: https://dl.polyhaven.org/file/ph-assets/Models/blend/1k/pine_tree_01/pine_tree_01_1k.blend
#   fir_tree_01:  https://dl.polyhaven.org/file/ph-assets/Models/blend/1k/fir_tree_01/fir_tree_01_1k.blend
#   tree_small_02: https://dl.polyhaven.org/file/ph-assets/Models/blend/1k/tree_small_02/tree_small_02_1k.blend
#   jacaranda_tree: https://dl.polyhaven.org/file/ph-assets/Models/blend/1k/jacaranda_tree/jacaranda_tree_1k.blend

# Run Blender in background mode:
blender --background --python tools/vegetation_processing/generate_production_trees.py

# Output: crates/renderer/assets/vegetation/field_*_lod{0,1,2}.glb
```

## Known Limitations

- Jacaranda tree (broadleaf_b) decimation is limited — LOD levels have similar triangle counts
- Poly Haven .blend files are written by Blender 404.32 (newer than portable 4.2.16) — some data may be lost on import
- Textures are embedded at 1K resolution from the source .blend; roughness/normal maps may not export to GLB (only base color is used by the renderer)
