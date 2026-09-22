# ENV1 open asset licences

Every asset recorded in `env1_open_assets.json` is listed here with its licence,
its attribution obligation and the exact files it produces in this repository.

---

## Poly Haven — `sparse_grass`

| Field | Value |
| --- | --- |
| Asset ID | `ENV1-GND-01` |
| Provider | Poly Haven |
| Slug | `sparse_grass` |
| Name | Sparse Grass |
| Source page | <https://polyhaven.com/a/sparse_grass> |
| Author | Amal Kumar (all maps) |
| Licence | **CC0 1.0 Universal (Public Domain Dedication)** |
| Licence URL | <https://polyhaven.com/license> |
| Acquired | 2026-09-22, via the Poly Haven public API |
| Resolution acquired | `4k` (4096x4096), PNG, 16-bit |

### Licence basis

The Poly Haven `/info` API payload for this asset carries **no** `license`
field. CC0 is recorded from <https://polyhaven.com/license>, which is the
citation the repository already uses for its Poly Haven vegetation assets in
`tools/vegetation_processing/PROVENANCE.md`. No licence was inferred from the
asset's appearance, category or description.

### Obligations

CC0 imposes no attribution requirement: the rights holder has waived all rights
to the extent permitted by law, and the work may be copied, modified and
redistributed, including commercially, without permission or attribution.

Two obligations are nonetheless honoured here:

1. **Poly Haven API terms.** All metadata and download URLs were obtained from
   the public API rather than by scraping HTML, an identifying `User-Agent` was
   sent on every request, and the required credit line is reproduced:

   > Powered by Poly Haven (<https://polyhaven.com>)

   The credit appears in `env1_open_assets.json` (`attribution`), in the fetch
   receipt, in the tooling output and in `docs/assets/env1/README.md`.

2. **Authorship record.** CC0 does not erase provenance. The author published by
   the API (`Amal Kumar`) is recorded in the manifest so the material's origin
   stays traceable.

### Files derived from this asset

Committed (2048x2048, produced by
`cargo run -p renderer --bin process_env1_terrain_material`):

| File | Content | Colour space | PNG layout |
| --- | --- | --- | --- |
| `crates/renderer/assets/env1/terrain/sparse_grass/sparse_grass_base_color.png` | base color | sRGB | 8-bit RGBA (color type 6) |
| `crates/renderer/assets/env1/terrain/sparse_grass/sparse_grass_normal.png` | tangent-space normal, OpenGL (+Y) orientation | linear | 8-bit RGBA (color type 6) |
| `crates/renderer/assets/env1/terrain/sparse_grass/sparse_grass_roughness.png` | roughness | linear | 8-bit grayscale (color type 0) |

Not committed (gitignored source cache, re-downloadable and digest-verified):

| File | API map type |
| --- | --- |
| `tmp/env1_source_cache/polyhaven/sparse_grass/4k/sparse_grass_diff_4k.png` | `Diffuse` |
| `tmp/env1_source_cache/polyhaven/sparse_grass/4k/sparse_grass_nor_gl_4k.png` | `nor_gl` |
| `tmp/env1_source_cache/polyhaven/sparse_grass/4k/sparse_grass_rough_4k.png` | `Rough` |

The `AO`, `Mask` and `Displacement` maps published for this asset were **not**
downloaded: ENV1-A has no runtime feature for them, and fetching them would
imply a capability the slice does not deliver. Occlusion is documented as
deferred in `docs/architecture/renderer_env1_a_production_material_foundation.md`.

SHA-256 digests for both the sources and the committed outputs are recorded in
`env1_open_assets.json`, which is the authority; they are deliberately not
duplicated in prose here.

---

## Assets NOT used by ENV1-A

For completeness, the repository's other Poly Haven-derived assets are covered
by `tools/vegetation_processing/PROVENANCE.md` (vegetation: `pine_tree_01`,
`fir_tree_01`, `tree_small_02`, `jacaranda_tree`, all CC0). ENV1-A does not
modify them. The terrain maps replaced by this slice
(`crates/renderer/assets/terrain_grass_*.png`) are original repository content
produced by the deterministic Rust generator and carry no third-party licence;
they remain committed and unchanged.
