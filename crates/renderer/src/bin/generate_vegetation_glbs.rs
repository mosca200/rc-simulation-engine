//! PV1: development-time generator for the committed production vegetation
//! GLB asset set.
//!
//! Rebuilds every `(asset, LOD)` GLB from the deterministic asset builders
//! and writes them into `crates/renderer/assets/vegetation/`:
//!
//! - `<name>_lod0.glb`, `<name>_lod1.glb`, `<name>_lod2.glb` per asset
//!   (bark + foliage primitives, distinct PBR materials).
//!
//! The generator is fully deterministic: running it twice on any platform
//! produces byte-identical GLBs (fixed layout, packed little-endian BIN,
//! `serde_json::Map` with sorted keys). The versioned assets are embedded
//! into the renderer binary with `include_bytes!`; the production runtime
//! loads them from memory and never rebuilds meshes procedurally.
//!
//! Provenance: these assets are project-original, generated in-repo from
//! seed-fixed procedural builders. No external files, no third-party
//! licenses. See `docs/architecture/renderer_pv1_vegetation_assets.md`.
//!
//! Usage:
//!
//! ```text
//! cargo run -p renderer --bin generate_vegetation_glbs
//! ```

use std::io::Write;

use renderer::vegetation_assets::{VegetationAssetSet, export_glb_lod};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let assets_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets/vegetation");
    std::fs::create_dir_all(&assets_dir)?;

    let set = VegetationAssetSet::bake_source_set();
    let mut total_bytes = 0_usize;
    for asset in set.assets() {
        for (lod, _) in [(0_u8, "lod0"), (1, "lod1"), (2, "lod2")] {
            let bytes = export_glb_lod(asset, lod);
            let file_name = format!("{}_{}.glb", asset.name, lod_label(lod));
            std::fs::File::create(assets_dir.join(&file_name))?.write_all(&bytes)?;
            println!(
                "{}: {file_name}: {} bytes (LOD{lod}: b{} f{} tris)",
                asset.name,
                bytes.len(),
                asset
                    .lods
                    .lod(lod)
                    .map(|l| l.bark.indices().len() / 3)
                    .unwrap_or(0),
                asset
                    .lods
                    .lod(lod)
                    .map(|l| l.foliage.indices().len() / 3)
                    .unwrap_or(0),
            );
            total_bytes += bytes.len();
        }
    }
    println!(
        "Wrote {} GLB files, {} bytes total to {}",
        set.len() * 3,
        total_bytes,
        assets_dir.display()
    );
    Ok(())
}

fn lod_label(lod: u8) -> &'static str {
    match lod {
        0 => "lod0",
        1 => "lod1",
        2 => "lod2",
        _ => unreachable!("LOD class in range"),
    }
}
