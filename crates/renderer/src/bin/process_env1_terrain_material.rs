//! ENV1-A: development-time processor for the Poly Haven `sparse_grass` maps.
//!
//! Reads the 16-bit 4k source PNGs out of the gitignored source cache, applies
//! the deterministic recipe documented in `renderer::env1_material`, and writes
//! the three committed runtime maps:
//!
//! - `sparse_grass_base_color.png` (RGBA8, sRGB intent)
//! - `sparse_grass_normal.png`     (RGBA8, linear, tangent-space OpenGL)
//! - `sparse_grass_roughness.png`  (L8 gray, linear)
//!
//! The processor is fully deterministic: running it twice over the same source
//! bytes produces byte-identical PNGs. The renderer never reads these files at
//! runtime — they are embedded with `include_bytes!`.
//!
//! Usage:
//!
//! ```text
//! cargo run -p renderer --bin process_env1_terrain_material
//! cargo run -p renderer --bin process_env1_terrain_material -- \
//!     --source-dir tmp/env1_source_cache/polyhaven/sparse_grass/4k \
//!     --out-dir crates/renderer/assets/env1/terrain/sparse_grass
//! ```

use renderer::env1_material::{
    ENV1_RUNTIME_EDGE, Env1MaterialError, load_luma16, load_rgb16, process_sparse_grass,
    write_runtime_maps,
};
use std::path::{Path, PathBuf};

/// Source file names as published by the Poly Haven files API for the `4k` PNG
/// variant of `sparse_grass`.
const SOURCE_BASE_COLOR: &str = "sparse_grass_diff_4k.png";
const SOURCE_NORMAL: &str = "sparse_grass_nor_gl_4k.png";
const SOURCE_ROUGHNESS: &str = "sparse_grass_rough_4k.png";

/// Default source cache location, relative to the workspace root. `tmp/` is
/// gitignored: the sources are re-downloadable and are never committed.
const DEFAULT_SOURCE_DIR: &str = "tmp/env1_source_cache/polyhaven/sparse_grass/4k";
/// Default committed runtime asset location, relative to the renderer crate.
const DEFAULT_OUT_DIR: &str = "assets/env1/terrain/sparse_grass";

struct Options {
    source_dir: PathBuf,
    out_dir: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let options = parse_arguments()?;

    println!("ENV1-A sparse_grass material processor");
    println!("  source dir: {}", options.source_dir.display());
    println!("  output dir: {}", options.out_dir.display());

    let base_color = load_rgb16(&options.source_dir.join(SOURCE_BASE_COLOR))?;
    let normal = load_rgb16(&options.source_dir.join(SOURCE_NORMAL))?;
    let roughness = load_luma16(&options.source_dir.join(SOURCE_ROUGHNESS))?;
    report_source("Diffuse", &base_color);
    report_source("nor_gl", &normal);
    println!(
        "  loaded Rough    {}x{} ({} samples, 16-bit grayscale)",
        roughness.width,
        roughness.height,
        roughness.samples.len()
    );

    let maps = process_sparse_grass(&base_color, &normal, &roughness)?;
    assert_eq!(maps.edge, ENV1_RUNTIME_EDGE, "runtime edge must be 2048");

    report_gray16("source Rough", &roughness.samples);
    report_rgb8("runtime base color", &maps.base_color_rgba8);
    report_rgb8("runtime normal", &maps.normal_rgba8);
    report_gray8("runtime roughness", &maps.roughness_r8);

    let paths = write_runtime_maps(&maps, &options.out_dir)?;
    println!("\nwrote {}px runtime maps:", maps.edge);
    for (label, path) in [
        ("base color (RGBA8, sRGB)", &paths.base_color),
        ("normal (RGBA8, linear GL)", &paths.normal),
        ("roughness (L8, linear)", &paths.roughness),
    ] {
        println!("  {label:28} {}", path.display());
    }
    Ok(())
}

fn parse_arguments() -> Result<Options, Box<dyn std::error::Error>> {
    let crate_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut options = Options {
        source_dir: crate_dir.join("..").join("..").join(DEFAULT_SOURCE_DIR),
        out_dir: crate_dir.join(DEFAULT_OUT_DIR),
    };

    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--source-dir" => options.source_dir = require_value(&mut arguments, "--source-dir")?,
            "--out-dir" => options.out_dir = require_value(&mut arguments, "--out-dir")?,
            "--help" | "-h" => {
                print_usage();
                std::process::exit(0);
            }
            other => {
                return Err(Box::new(Env1MaterialError::Decode {
                    path: other.to_owned(),
                    reason: "unknown argument (expected --source-dir, --out-dir or --help)"
                        .to_owned(),
                }));
            }
        }
    }
    Ok(options)
}

fn print_usage() {
    println!(
        "usage: process_env1_terrain_material [--source-dir PATH] [--out-dir PATH]\n\
         \n\
         defaults:\n  \
         --source-dir {DEFAULT_SOURCE_DIR}   (relative to the workspace root)\n  \
         --out-dir    {DEFAULT_OUT_DIR} (relative to crates/renderer)"
    );
}

fn require_value(
    arguments: &mut impl Iterator<Item = String>,
    flag: &str,
) -> Result<PathBuf, Box<dyn std::error::Error>> {
    arguments
        .next()
        .map(PathBuf::from)
        .ok_or_else(|| format!("{flag} requires a path value").into())
}

fn report_source(label: &str, map: &renderer::env1_material::Rgb16Map) {
    println!(
        "  loaded {label:9} {}x{} ({} samples, 16-bit RGB)",
        map.width,
        map.height,
        map.samples.len()
    );
    for channel in 0..3usize {
        let name = "rgb".as_bytes()[channel] as char;
        let values: Vec<u32> = map
            .samples
            .iter()
            .skip(channel)
            .step_by(3)
            .map(|&v| u32::from(v))
            .collect();
        print_stats(&format!("source {label}.{name}"), &values);
    }
}

fn report_rgb8(label: &str, bytes: &[u8]) {
    for channel in 0..3usize {
        let name = "rgb".as_bytes()[channel] as char;
        let values: Vec<u32> = bytes
            .iter()
            .skip(channel)
            .step_by(4)
            .map(|&v| u32::from(v))
            .collect();
        print_stats(&format!("{label}.{name}"), &values);
    }
    let opaque = bytes.iter().skip(3).step_by(4).all(|&value| value == 255);
    println!("  {label}.alpha: every texel 255 = {opaque}");
}

fn report_gray16(label: &str, samples: &[u16]) {
    let values: Vec<u32> = samples.iter().map(|&v| u32::from(v)).collect();
    print_stats(label, &values);
}

fn report_gray8(label: &str, bytes: &[u8]) {
    let values: Vec<u32> = bytes.iter().map(|&v| u32::from(v)).collect();
    print_stats(label, &values);
}

fn print_stats(label: &str, values: &[u32]) {
    let Some(&min) = values.iter().min() else {
        println!("  {label}: empty");
        return;
    };
    let max = values.iter().max().copied().unwrap_or(min);
    let sum: u64 = values.iter().map(|&value| u64::from(value)).sum();
    let mean = sum as f64 / values.len() as f64;
    println!("  {label}: mean {mean:.1}, min {min}, max {max}");
}
