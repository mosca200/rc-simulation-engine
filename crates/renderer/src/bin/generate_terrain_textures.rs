//! G3A: development-time generator for the deterministic terrain texture set.
//!
//! Regenerates the committed terrain maps from `renderer::terrain_textures`
//! and writes them into `crates/renderer/assets/`:
//!
//! - `terrain_grass_albedo.png`    (RGBA8, sRGB intent)
//! - `terrain_grass_normal.png`    (RGBA8, linear)
//! - `terrain_grass_roughness.png` (L8 gray, linear R channel)
//!
//! The generator is fully deterministic: running it twice on any platform
//! produces byte-identical PNGs (lossless encoding). The versioned assets are
//! embedded into the renderer binary with `include_bytes!`; the renderer never
//! reads them from disk at runtime.
//!
//! Usage:
//!
//! ```text
//! cargo run -p renderer --bin generate_terrain_textures
//! ```

use image::{ImageBuffer, Luma, Rgba};
use renderer::terrain_textures::{TERRAIN_TEXTURE_SIZE, generate_terrain_textures};

fn stats(label: &str, data: &[u8]) {
    if data.is_empty() {
        println!("{label}: empty");
        return;
    }
    let mean: f64 = data.iter().map(|&v| v as f64).sum::<f64>() / data.len() as f64;
    let min = *data.iter().min().unwrap();
    let max = *data.iter().max().unwrap();
    println!("{label}: mean {mean:.1}, min {min}, max {max}");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let assets_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("assets");
    std::fs::create_dir_all(&assets_dir)?;

    let set = generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
    let n = TERRAIN_TEXTURE_SIZE;

    let albedo = ImageBuffer::<Rgba<u8>, _>::from_raw(n, n, set.albedo_rgba.clone())
        .expect("albedo buffer size");
    albedo.save(assets_dir.join("terrain_grass_albedo.png"))?;

    let normal = ImageBuffer::<Rgba<u8>, _>::from_raw(n, n, set.normal_rgba.clone())
        .expect("normal buffer size");
    normal.save(assets_dir.join("terrain_grass_normal.png"))?;

    let roughness = ImageBuffer::<Luma<u8>, _>::from_raw(n, n, set.roughness_r8.clone())
        .expect("roughness buffer size");
    roughness.save(assets_dir.join("terrain_grass_roughness.png"))?;

    // Channel statistics for eyeballing the design.
    for byte in 0..3 {
        let channel: Vec<u8> = set.albedo_rgba[byte..].iter().step_by(4).copied().collect();
        stats(
            &format!("albedo.{}", "rgb".as_bytes()[byte] as char),
            &channel,
        );
    }
    stats(
        "normal.x",
        &set.normal_rgba[0..]
            .iter()
            .step_by(4)
            .copied()
            .collect::<Vec<_>>(),
    );
    stats(
        "normal.y",
        &set.normal_rgba[1..]
            .iter()
            .step_by(4)
            .copied()
            .collect::<Vec<_>>(),
    );
    stats(
        "normal.z",
        &set.normal_rgba[2..]
            .iter()
            .step_by(4)
            .copied()
            .collect::<Vec<_>>(),
    );
    stats("roughness", &set.roughness_r8);

    println!("Wrote {}px terrain maps to {}", n, assets_dir.display());
    Ok(())
}
