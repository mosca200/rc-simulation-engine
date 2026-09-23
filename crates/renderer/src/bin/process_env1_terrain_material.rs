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
    ENV1_RUNTIME_EDGE, Env1MaterialError, load_luma_source, load_rgb_source, process_sparse_grass,
    write_runtime_maps_with_names,
};
use std::path::{Path, PathBuf};

/// One registered ground material: source file names as published by the Poly
/// Haven files API for the `4k` PNG variant, the committed runtime file names,
/// and the default cache/output locations. Mirrors the Python registry in
/// `tools/env1_asset_pipeline/env1_assets.py::OPEN_ASSETS`.
struct GroundAssetSpec {
    slug: &'static str,
    source_base_color: &'static str,
    source_normal: &'static str,
    source_roughness: &'static str,
    runtime_base_color: &'static str,
    runtime_normal: &'static str,
    runtime_roughness: &'static str,
    default_source_dir: &'static str,
    default_out_dir: &'static str,
}

const GROUND_ASSETS: &[GroundAssetSpec] = &[
    GroundAssetSpec {
        slug: "sparse_grass",
        source_base_color: "sparse_grass_diff_4k.png",
        source_normal: "sparse_grass_nor_gl_4k.png",
        source_roughness: "sparse_grass_rough_4k.png",
        runtime_base_color: "sparse_grass_base_color.png",
        runtime_normal: "sparse_grass_normal.png",
        runtime_roughness: "sparse_grass_roughness.png",
        default_source_dir: "tmp/env1_source_cache/polyhaven/sparse_grass/4k",
        default_out_dir: "assets/env1/terrain/sparse_grass",
    },
    GroundAssetSpec {
        slug: "grass_path_3",
        source_base_color: "grass_path_3_diff_4k.png",
        source_normal: "grass_path_3_nor_gl_4k.png",
        source_roughness: "grass_path_3_rough_4k.png",
        runtime_base_color: "grass_path_3_base_color.png",
        runtime_normal: "grass_path_3_normal.png",
        runtime_roughness: "grass_path_3_roughness.png",
        default_source_dir: "tmp/env1_source_cache/polyhaven/grass_path_3/4k",
        default_out_dir: "assets/env1/terrain/grass_path_3",
    },
    GroundAssetSpec {
        slug: "forest_ground_04",
        source_base_color: "forest_ground_04_diff_4k.png",
        source_normal: "forest_ground_04_nor_gl_4k.png",
        source_roughness: "forest_ground_04_rough_4k.png",
        runtime_base_color: "forest_ground_04_base_color.png",
        runtime_normal: "forest_ground_04_normal.png",
        runtime_roughness: "forest_ground_04_roughness.png",
        default_source_dir: "tmp/env1_source_cache/polyhaven/forest_ground_04/4k",
        default_out_dir: "assets/env1/terrain/forest_ground_04",
    },
];

fn spec_for_slug(slug: &str) -> Result<&'static GroundAssetSpec, Box<dyn std::error::Error>> {
    GROUND_ASSETS
        .iter()
        .find(|spec| spec.slug == slug)
        .ok_or_else(|| {
            format!(
                "unknown asset {slug:?} (registered: sparse_grass, grass_path_3, forest_ground_04)"
            )
            .into()
        })
}

struct Options {
    spec: &'static GroundAssetSpec,
    source_dir: PathBuf,
    out_dir: PathBuf,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let options = parse_arguments()?;
    let spec = options.spec;

    println!("ENV1 {} material processor", spec.slug);
    println!("  source dir: {}", options.source_dir.display());
    println!("  output dir: {}", options.out_dir.display());

    let base_color = load_rgb_source(&options.source_dir.join(spec.source_base_color))?;
    let normal = load_rgb_source(&options.source_dir.join(spec.source_normal))?;
    let roughness = load_luma_source(&options.source_dir.join(spec.source_roughness))?;
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

    let paths = write_runtime_maps_with_names(
        &maps,
        &options.out_dir,
        &(
            spec.runtime_base_color,
            spec.runtime_normal,
            spec.runtime_roughness,
        ),
    )?;
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
    let mut slug = String::from("sparse_grass");
    let mut source_dir: Option<PathBuf> = None;
    let mut out_dir: Option<PathBuf> = None;

    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--asset" => {
                slug = require_value(&mut arguments, "--asset")?
                    .to_string_lossy()
                    .into_owned();
            }
            "--source-dir" => source_dir = Some(require_value(&mut arguments, "--source-dir")?),
            "--out-dir" => out_dir = Some(require_value(&mut arguments, "--out-dir")?),
            "--help" | "-h" => {
                print_usage();
                std::process::exit(0);
            }
            other => {
                return Err(Box::new(Env1MaterialError::Decode {
                    path: other.to_owned(),
                    reason:
                        "unknown argument (expected --asset, --source-dir, --out-dir or --help)"
                            .to_owned(),
                }));
            }
        }
    }

    let spec = spec_for_slug(&slug)?;
    Ok(Options {
        spec,
        source_dir: source_dir.unwrap_or_else(|| {
            crate_dir
                .join("..")
                .join("..")
                .join(spec.default_source_dir)
        }),
        out_dir: out_dir.unwrap_or_else(|| crate_dir.join(spec.default_out_dir)),
    })
}

fn print_usage() {
    println!(
        "usage: process_env1_terrain_material [--asset SLUG] [--source-dir PATH] [--out-dir PATH]\n\
         \n\
         registered assets: sparse_grass (default), grass_path_3, forest_ground_04\n\
         each asset defaults its --source-dir to its gitignored Poly Haven cache\n\
         and its --out-dir to its committed runtime asset directory"
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
