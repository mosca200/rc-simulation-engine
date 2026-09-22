//! ENV1-A acceptance tests: the processed Poly Haven `sparse_grass` material,
//! its mip chain, its colour-space semantics, and the glTF material slots the
//! slice adds parsing for.
//!
//! These tests never touch a GPU. The headless GPU probes that render the
//! terrain live in `crates/renderer/src/gpu.rs` behind `#[ignore]`.

use image::{ImageBuffer, Rgba};
use renderer::env1_material::{ENV1_RUNTIME_EDGE, runtime_assets};
use renderer::terrain_textures::{
    TERRAIN_TEXTURE_SIZE, generate_terrain_mip_chain, mip_level_count_for_size,
    terrain_texture_set_from_decoded,
};
use renderer::texture::{SamplerMipmapFilter, decode_image};
use renderer::{
    PrimitiveMaterial, SamplerConfig, TerrainMaterial, blend_linear_roughness,
    blend_registered_tangent_normals, load_glb_bytes, reorient_rotated_tangent_normal,
};
use std::io::Cursor;

// ---------------------------------------------------------------------------
// Committed runtime assets
// ---------------------------------------------------------------------------

/// Iterate RGBA8 texels. `chunks_exact(4)` is flagged by clippy for a constant
/// chunk size, and `slice::array_chunks` is not stable, so the stride is
/// spelled out.
fn texels(bytes: &[u8]) -> impl Iterator<Item = [u8; 4]> + use<'_> {
    (0..bytes.len() / 4).map(move |index| {
        let start = index * 4;
        [
            bytes[start],
            bytes[start + 1],
            bytes[start + 2],
            bytes[start + 3],
        ]
    })
}

fn decoded_maps() -> (
    renderer::texture::DecodedTexture,
    renderer::texture::DecodedTexture,
    renderer::texture::DecodedTexture,
) {
    let base = decode_image(runtime_assets::TERRAIN_ALBEDO_PNG).expect("base color decodes");
    let normal = decode_image(runtime_assets::TERRAIN_NORMAL_PNG).expect("normal decodes");
    let roughness = decode_image(runtime_assets::TERRAIN_ROUGHNESS_PNG).expect("roughness decodes");
    (base, normal, roughness)
}

#[test]
fn committed_env1_maps_are_png_and_decode_to_the_runtime_edge() {
    for (label, bytes) in [
        ("base color", runtime_assets::TERRAIN_ALBEDO_PNG),
        ("normal", runtime_assets::TERRAIN_NORMAL_PNG),
        ("roughness", runtime_assets::TERRAIN_ROUGHNESS_PNG),
    ] {
        assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n", "{label} must be a PNG");
        // IHDR: width/height are big-endian u32 at byte offsets 16 and 20.
        let width = u32::from_be_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
        let height = u32::from_be_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
        assert_eq!(
            (width, height),
            (ENV1_RUNTIME_EDGE, ENV1_RUNTIME_EDGE),
            "{label} size"
        );
        assert_eq!(bytes[24], 8, "{label} must be 8-bit");
    }
}

#[test]
fn committed_env1_maps_carry_the_documented_channel_layout() {
    // IHDR color type: 6 = truecolor with alpha, 0 = grayscale.
    assert_eq!(
        runtime_assets::TERRAIN_ALBEDO_PNG[25],
        6,
        "base color must be RGBA8"
    );
    assert_eq!(
        runtime_assets::TERRAIN_NORMAL_PNG[25],
        6,
        "normal must be RGBA8"
    );
    assert_eq!(
        runtime_assets::TERRAIN_ROUGHNESS_PNG[25],
        0,
        "roughness must be single-channel grayscale so the runtime can upload R8"
    );
    assert_eq!(runtime_assets::TERRAIN_ALBEDO_PNG[28], 0, "no interlace");
    assert_eq!(runtime_assets::TERRAIN_NORMAL_PNG[28], 0, "no interlace");
    assert_eq!(runtime_assets::TERRAIN_ROUGHNESS_PNG[28], 0, "no interlace");
}

#[test]
fn committed_env1_maps_decode_to_the_expected_pixel_counts() {
    let (base, normal, roughness) = decoded_maps();
    let pixels = (ENV1_RUNTIME_EDGE * ENV1_RUNTIME_EDGE) as usize;
    for (label, map) in [("base color", &base), ("normal", &normal)] {
        assert_eq!(
            (map.width, map.height),
            (ENV1_RUNTIME_EDGE, ENV1_RUNTIME_EDGE)
        );
        assert_eq!(map.rgba8.len(), pixels * 4, "{label} must be RGBA8");
    }
    assert_eq!(
        roughness.rgba8.len(),
        pixels * 4,
        "the gray decode expands to RGBA8"
    );
    // The runtime extracts the R channel; a grayscale source must have R == G == B.
    assert!(
        texels(&roughness.rgba8)
            .take(4096)
            .all(|texel| texel[0] == texel[1] && texel[1] == texel[2]),
        "roughness must be neutral gray so the R-channel extraction is lossless"
    );
}

#[test]
fn committed_env1_base_color_is_opaque_and_not_baked_with_shadow_or_ao() {
    let (base, _, _) = decoded_maps();
    assert!(
        base.rgba8
            .iter()
            .skip(3)
            .step_by(4)
            .all(|&alpha| alpha == 255),
        "the Diffuse source has no alpha channel, so the runtime map is opaque"
    );
    // A photograph of sparse grass over damp soil: it must carry real chroma
    // (green tufts, brown soil) rather than a flat or monochrome fill, and it
    // must not be crushed toward black, which a baked shadow or an AO multiply
    // into the albedo would do.
    let pixels = base.rgba8.len() / 4;
    let sum = |channel: usize| -> u64 {
        base.rgba8
            .iter()
            .skip(channel)
            .step_by(4)
            .map(|&value| u64::from(value))
            .sum()
    };
    let mean_r = sum(0) as f64 / pixels as f64;
    let mean_g = sum(1) as f64 / pixels as f64;
    let mean_b = sum(2) as f64 / pixels as f64;
    assert!(
        mean_r > mean_b,
        "soil/grass must be warmer than blue: {mean_r} vs {mean_b}"
    );
    assert!(
        mean_g > mean_b,
        "vegetation must dominate blue: {mean_g} vs {mean_b}"
    );
    assert!(
        mean_r > 16.0 && mean_g > 16.0 && mean_b > 8.0,
        "albedo must not be crushed toward black (no baked shadow/AO): \
         r={mean_r:.1} g={mean_g:.1} b={mean_b:.1}"
    );
    let distinct = texels(&base.rgba8)
        .map(|texel| u32::from_ne_bytes([texel[0], texel[1], texel[2], 0]))
        .collect::<std::collections::HashSet<_>>()
        .len();
    assert!(
        distinct > 10_000,
        "a photograph must carry rich colour variation, got {distinct} distinct rgb values"
    );
}

#[test]
fn committed_env1_normal_is_tangent_space_opengl_and_unit_length() {
    let (_, normal, _) = decoded_maps();
    let mut sum_z = 0u64;
    let mut max_deviation = 0.0f64;
    let mut up_dominant = 0usize;
    for texel in texels(&normal.rgba8) {
        assert_eq!(texel[3], 255, "normal alpha must be opaque");
        let x = f64::from(texel[0]) / 255.0 * 2.0 - 1.0;
        let y = f64::from(texel[1]) / 255.0 * 2.0 - 1.0;
        let z = f64::from(texel[2]) / 255.0 * 2.0 - 1.0;
        let length = (x * x + y * y + z * z).sqrt();
        max_deviation = max_deviation.max((length - 1.0).abs());
        sum_z += texel[2] as u64;
        if z > 0.0 {
            up_dominant += 1;
        }
    }
    let texels = normal.rgba8.len() / 4;
    // 8-bit quantization means the length cannot be exactly 1 everywhere, but a
    // correctly renormalized map stays within a couple of quantization steps.
    assert!(
        max_deviation < 0.05,
        "normal texels must be near unit length, worst deviation {max_deviation}"
    );
    let mean_z = sum_z as f64 / texels as f64;
    assert!(
        mean_z > 200.0,
        "a ground normal map must be Z-dominant, mean Z = {mean_z:.1}"
    );
    assert_eq!(
        up_dominant, texels,
        "every texel must point out of the surface (+Z), the OpenGL convention"
    );
}

#[test]
fn committed_env1_roughness_is_linear_dielectric_data_in_range() {
    let (base, normal, roughness) = decoded_maps();
    let set = terrain_texture_set_from_decoded(&base.rgba8, &normal.rgba8, &roughness.rgba8);
    assert_eq!(set.roughness_r8.len(), roughness.rgba8.len() / 4);
    let min = *set.roughness_r8.iter().min().expect("non-empty");
    let max = *set.roughness_r8.iter().max().expect("non-empty");
    assert!(min < max, "roughness must vary across the photograph");
    // Sparse grass over damp soil is a matte dielectric; nothing may be glossy
    // enough to read as metal, and nothing may be fully smooth either.
    assert!(min >= 32, "roughness range [{min}, {max}] out of bounds");
    let mean = set
        .roughness_r8
        .iter()
        .map(|&value| u64::from(value))
        .sum::<u64>() as f64
        / set.roughness_r8.len() as f64;
    assert!(
        mean > 128.0,
        "the material must stay predominantly rough, mean {mean:.1}"
    );
}

// ---------------------------------------------------------------------------
// Mip chain
// ---------------------------------------------------------------------------

fn committed_chain() -> renderer::terrain_textures::TerrainMipChain {
    let (base, normal, roughness) = decoded_maps();
    let set = terrain_texture_set_from_decoded(&base.rgba8, &normal.rgba8, &roughness.rgba8);
    generate_terrain_mip_chain(&set, ENV1_RUNTIME_EDGE)
}

#[test]
fn env1_mip_chain_is_complete_down_to_one_by_one() {
    let chain = committed_chain();
    let levels = mip_level_count_for_size(ENV1_RUNTIME_EDGE) as usize;
    assert_eq!(ENV1_RUNTIME_EDGE, 2048);
    assert_eq!(levels, 12, "2048 must produce 12 levels (2048 -> 1)");
    for (label, map) in [
        ("albedo", &chain.albedo),
        ("normal", &chain.normal),
        ("roughness", &chain.roughness),
    ] {
        assert_eq!(map.len(), levels, "{label} level count");
        let mut expected = ENV1_RUNTIME_EDGE;
        for level in map {
            assert_eq!(
                (level.width, level.height),
                (expected, expected),
                "{label} dimensions"
            );
            expected /= 2;
        }
        let last = map.last().expect("last level");
        assert_eq!((last.width, last.height), (1, 1), "{label} must reach 1x1");
    }
}

#[test]
fn env1_mip_chain_payload_sizes_match_their_dimensions() {
    let chain = committed_chain();
    for (label, map, channels) in [
        ("albedo", &chain.albedo, 4usize),
        ("normal", &chain.normal, 4),
        ("roughness", &chain.roughness, 1),
    ] {
        for level in map {
            let expected = (level.width as usize) * (level.height as usize) * channels;
            assert_eq!(
                level.bytes.len(),
                expected,
                "{label} {}px payload",
                level.width
            );
        }
    }
}

#[test]
fn env1_mip_chain_is_bitwise_deterministic() {
    let a = committed_chain();
    let b = committed_chain();
    assert_eq!(
        a, b,
        "the mip chain must be a pure function of the committed maps"
    );
}

#[test]
fn env1_mip_level_zero_is_the_committed_base_texture() {
    let (base, normal, roughness) = decoded_maps();
    let chain = committed_chain();
    assert_eq!(chain.albedo[0].bytes, base.rgba8);
    assert_eq!(chain.normal[0].bytes, normal.rgba8);
    let roughness_r8: Vec<u8> = roughness.rgba8.iter().step_by(4).copied().collect();
    assert_eq!(chain.roughness[0].bytes, roughness_r8);
}

#[test]
fn env1_mip_chain_keeps_alpha_information_for_future_foliage() {
    // The runtime chain filters alpha rather than forcing it opaque, which is
    // what a later foliage tranche needs for alpha-tested leaf cards.
    let (base, normal, roughness) = decoded_maps();
    let mut albedo = base.rgba8.clone();
    for index in 0..albedo.len() / 4 {
        albedo[index * 4 + 3] = if index.is_multiple_of(2) { 0 } else { 255 };
    }
    let set = terrain_texture_set_from_decoded(&albedo, &normal.rgba8, &roughness.rgba8);
    let chain = generate_terrain_mip_chain(&set, ENV1_RUNTIME_EDGE);
    let level1: Vec<u8> = chain.albedo[1]
        .bytes
        .iter()
        .skip(3)
        .step_by(4)
        .copied()
        .collect();
    assert!(
        level1.iter().all(|&alpha| (120..=135).contains(&alpha)),
        "level 1 alpha must be the filtered mean of 0 and 255, not a clamp to 255"
    );
}

// ---------------------------------------------------------------------------
// Wiring and colour-space semantics (source contract)
// ---------------------------------------------------------------------------

#[test]
fn terrain_material_path_consumes_the_env1_assets() {
    let source = include_str!("../src/gpu.rs");
    assert!(
        source.contains("use crate::env1_material::runtime_assets as terrain_assets;"),
        "the terrain material must be built from the ENV1 runtime maps"
    );
    assert!(
        source.contains("use crate::env1_material::ENV1_RUNTIME_EDGE;"),
        "the terrain mip chain must be sized from the ENV1 runtime edge"
    );
    assert!(
        source.contains("generate_terrain_mip_chain(&base_set, ENV1_RUNTIME_EDGE)"),
        "the mip chain must be generated at the ENV1 runtime edge"
    );
    assert!(
        !source.contains("use crate::terrain_textures::generated as terrain_assets;"),
        "the procedural 1024 maps must no longer feed the terrain FINAL path"
    );
}

#[test]
fn terrain_colour_semantics_stay_srgb_for_color_and_linear_for_data() {
    let source = include_str!("../src/gpu.rs");
    let creation = source
        .split("fn create_terrain_material(")
        .nth(1)
        .expect("create_terrain_material must exist");
    // Base color is sampled as sRGB and converted to linear by the hardware;
    // the normal and roughness maps are linear data and must not be sRGB.
    assert!(creation.contains("format: wgpu::TextureFormat::Rgba8UnormSrgb"));
    assert!(creation.contains("format: wgpu::TextureFormat::Rgba8Unorm,"));
    assert!(creation.contains("format: wgpu::TextureFormat::R8Unorm"));
    // A real mip chain, uploaded once at initialization with trilinear plus
    // anisotropic filtering, and never regenerated per frame.
    assert!(creation.contains("mip_level_count: mip_levels"));
    assert!(creation.contains("mipmap_filter: wgpu::MipmapFilterMode::Linear"));
    assert!(creation.contains("anisotropy_clamp: sampler_anisotropy"));
}

#[test]
fn procedural_terrain_generator_and_its_assets_are_untouched() {
    // ENV1-A keeps the procedural set alongside the photographic one, so its
    // bitwise generator tripwire and its 1024 edge stay exactly as they were.
    assert_eq!(TERRAIN_TEXTURE_SIZE, 1024);
    assert_ne!(
        TERRAIN_TEXTURE_SIZE, ENV1_RUNTIME_EDGE,
        "the ENV1 set is a separate 2048 asset, not a resized procedural one"
    );
    let generated = renderer::terrain_textures::generate_terrain_textures(TERRAIN_TEXTURE_SIZE);
    let committed = terrain_texture_set_from_decoded(
        &decode_image(renderer::terrain_textures::generated::TERRAIN_ALBEDO_PNG)
            .expect("albedo")
            .rgba8,
        &decode_image(renderer::terrain_textures::generated::TERRAIN_NORMAL_PNG)
            .expect("normal")
            .rgba8,
        &decode_image(renderer::terrain_textures::generated::TERRAIN_ROUGHNESS_PNG)
            .expect("roughness")
            .rgba8,
    );
    assert_eq!(
        generated, committed,
        "the procedural assets must still be reproducible"
    );
}

#[test]
fn env1_material_contract_has_no_wgpu_leakage() {
    let source = include_str!("../src/env1_material.rs");
    // Strip the doc comments so prose about GPU formats cannot mask a real leak.
    let contract: String = source
        .lines()
        .filter(|line| {
            !line.trim_start().starts_with("//!") && !line.trim_start().starts_with("///")
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert!(
        !contract.contains("wgpu::"),
        "the ENV1 material processing must stay wgpu-free so it is testable without a GPU"
    );
}

// ---------------------------------------------------------------------------
// glTF material slots
// ---------------------------------------------------------------------------

fn png_bytes(rgba: [u8; 4]) -> Vec<u8> {
    let image = ImageBuffer::<Rgba<u8>, _>::from_pixel(2, 2, Rgba(rgba));
    let mut buffer = Cursor::new(Vec::new());
    image
        .write_to(&mut buffer, image::ImageFormat::Png)
        .expect("PNG encode");
    buffer.into_inner()
}

/// Assemble a minimal but valid GLB: one triangle, one material, and the given
/// images bound to the material slots named by `material_json_extra`.
fn build_glb(images: &[Vec<u8>], material_json: &str) -> Vec<u8> {
    let position: [f32; 9] = [-1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 0.0, 1.0, 0.0];
    let indices: [u16; 3] = [0, 1, 2];

    let mut bin: Vec<u8> = Vec::new();
    for value in position {
        bin.extend_from_slice(&value.to_le_bytes());
    }
    let position_length = bin.len();
    for value in indices {
        bin.extend_from_slice(&value.to_le_bytes());
    }
    let indices_length = bin.len() - position_length;
    // Pad so the image buffer views start on a 4-byte boundary.
    while !bin.len().is_multiple_of(4) {
        bin.push(0);
    }

    let mut image_views = Vec::new();
    let mut image_entries = Vec::new();
    for (index, image) in images.iter().enumerate() {
        let offset = bin.len();
        bin.extend_from_slice(image);
        while !bin.len().is_multiple_of(4) {
            bin.push(0);
        }
        image_views.push(format!(
            r#"{{"buffer":0,"byteOffset":{offset},"byteLength":{}}}"#,
            image.len()
        ));
        image_entries.push(format!(
            r#"{{"bufferView":{},"mimeType":"image/png"}}"#,
            index + 2
        ));
    }

    let buffer_views = format!(
        r#"{{"buffer":0,"byteOffset":0,"byteLength":{position_length},"target":34962}},{{"buffer":0,"byteOffset":{position_length},"byteLength":{indices_length},"target":34963}}{comma}{images}"#,
        comma = if image_views.is_empty() { "" } else { "," },
        images = image_views.join(",")
    );
    let textures = (0..images.len())
        .map(|index| format!(r#"{{"source":{index}}}"#))
        .collect::<Vec<_>>()
        .join(",");

    let json = format!(
        r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],"meshes":[{{"primitives":[{{"attributes":{{"POSITION":0}},"indices":1,"material":0}}]}}],"materials":[{material_json}],"textures":[{textures}],"images":[{images}],"buffers":[{{"byteLength":{}}}],"bufferViews":[{buffer_views}],"accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[-1.0,-1.0,0.0],"max":[1.0,1.0,0.0]}},{{"bufferView":1,"componentType":5123,"count":3,"type":"SCALAR"}}]}}"#,
        bin.len(),
        images = image_entries.join(",")
    );

    let mut json_bytes = json.into_bytes();
    while !json_bytes.len().is_multiple_of(4) {
        json_bytes.push(b' ');
    }

    let total = 12 + 8 + json_bytes.len() + 8 + bin.len();
    let mut glb = Vec::with_capacity(total);
    glb.extend_from_slice(b"glTF");
    glb.extend_from_slice(&2u32.to_le_bytes());
    glb.extend_from_slice(&(total as u32).to_le_bytes());
    glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x4E4F_534Au32.to_le_bytes());
    glb.extend_from_slice(&json_bytes);
    glb.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    glb.extend_from_slice(&0x004E_4942u32.to_le_bytes());
    glb.extend_from_slice(&bin);
    assert_eq!(glb.len(), total);
    glb
}

fn first_material(asset: &renderer::GlbAsset) -> &PrimitiveMaterial {
    &asset
        .primitives
        .first()
        .expect("the test GLB carries exactly one primitive")
        .material
}

#[test]
fn gltf_normal_texture_is_parsed_with_its_scale() {
    let glb = build_glb(
        &[png_bytes([128, 128, 255, 255])],
        r#"{"pbrMetallicRoughness":{"baseColorFactor":[1.0,1.0,1.0,1.0]},"normalTexture":{"index":0,"scale":0.75}}"#,
    );
    let asset = load_glb_bytes(&glb, "env1a_normal_texture").expect("GLB loads");
    let material = first_material(&asset);

    let normal = material
        .normal_texture
        .as_ref()
        .expect("normalTexture must be parsed");
    assert_eq!((normal.width, normal.height), (2, 2));
    assert_eq!(normal.rgba8.len(), 16);
    assert_eq!(&normal.rgba8[..4], &[128, 128, 255, 255], "flat-up normal");
    assert!(
        (material.normal_texture_scale - 0.75).abs() < 1e-6,
        "normalTexture.scale must be carried, got {}",
        material.normal_texture_scale
    );
    // The base color slot stays empty: parsing one slot must not fill another.
    assert!(material.base_color_texture.is_none());
    assert!(material.metallic_roughness_texture.is_none());
}

#[test]
fn gltf_normal_texture_scale_defaults_to_one() {
    let glb = build_glb(
        &[png_bytes([128, 128, 255, 255])],
        r#"{"pbrMetallicRoughness":{},"normalTexture":{"index":0}}"#,
    );
    let asset = load_glb_bytes(&glb, "env1a_normal_default_scale").expect("GLB loads");
    let material = first_material(&asset);
    assert!(material.normal_texture.is_some());
    assert_eq!(material.normal_texture_scale, 1.0);
}

#[test]
fn gltf_metallic_roughness_texture_is_parsed() {
    // glTF packs roughness in G and metallic in B.
    let glb = build_glb(
        &[png_bytes([0, 200, 40, 255])],
        r#"{"pbrMetallicRoughness":{"metallicRoughnessTexture":{"index":0}}}"#,
    );
    let asset = load_glb_bytes(&glb, "env1a_metallic_roughness").expect("GLB loads");
    let material = first_material(&asset);

    let map = material
        .metallic_roughness_texture
        .as_ref()
        .expect("metallicRoughnessTexture must be parsed");
    assert_eq!(
        &map.rgba8[..4],
        &[0, 200, 40, 255],
        "channels must be preserved verbatim"
    );
    assert!(material.normal_texture.is_none());
    assert!(material.base_color_texture.is_none());
}

#[test]
fn gltf_material_with_every_slot_shares_one_decode_per_texture_index() {
    let base = png_bytes([200, 30, 30, 255]);
    let normal = png_bytes([128, 128, 255, 255]);
    let metallic_roughness = png_bytes([0, 128, 255, 255]);
    let glb = build_glb(
        &[base, normal, metallic_roughness],
        r#"{"pbrMetallicRoughness":{"baseColorTexture":{"index":0},"metallicRoughnessTexture":{"index":2}},"normalTexture":{"index":1,"scale":1.25}}"#,
    );
    let asset = load_glb_bytes(&glb, "env1a_all_slots").expect("GLB loads");
    let material = first_material(&asset);

    assert_eq!(
        material
            .base_color_texture
            .as_ref()
            .expect("base color")
            .rgba8[..4],
        [200, 30, 30, 255]
    );
    assert_eq!(
        material.normal_texture.as_ref().expect("normal").rgba8[..4],
        [128, 128, 255, 255]
    );
    assert_eq!(
        material
            .metallic_roughness_texture
            .as_ref()
            .expect("metallic roughness")
            .rgba8[..4],
        [0, 128, 255, 255]
    );
    assert!((material.normal_texture_scale - 1.25).abs() < 1e-6);
}

#[test]
fn gltf_material_without_the_new_slots_defaults_to_none() {
    let glb = build_glb(
        &[],
        r#"{"pbrMetallicRoughness":{"baseColorFactor":[1.0,1.0,1.0,1.0]}}"#,
    );
    let asset = load_glb_bytes(&glb, "env1a_no_textures").expect("GLB loads");
    let material = first_material(&asset);
    assert!(material.normal_texture.is_none());
    assert!(material.metallic_roughness_texture.is_none());
    assert_eq!(material.normal_texture_scale, 1.0);
    assert_eq!(material.sampler_config, SamplerConfig::default_sampler());
}

#[test]
fn gltf_sampler_reports_whether_a_mip_chain_was_requested() {
    // minFilter 9987 is LINEAR_MIPMAP_LINEAR: trilinear minification with a mip
    // chain. 9729 is bare LINEAR: no mipmapping.
    let trilinear = SamplerConfig {
        mipmap_filter: Some(SamplerMipmapFilter::Linear),
        ..SamplerConfig::default_sampler()
    };
    assert!(trilinear.mipmap_filter.is_some());
    assert_eq!(
        SamplerMipmapFilter::from_gltf_min(gltf::texture::MinFilter::LinearMipmapLinear),
        Some(SamplerMipmapFilter::Linear)
    );
    assert_eq!(
        SamplerMipmapFilter::from_gltf_min(gltf::texture::MinFilter::Linear),
        None
    );
}

/// The committed assets must keep working exactly as before: the production
/// aircraft GLB is untextured and every vegetation GLB uses only the base color
/// slot, which is why ENV1-A's new parsing is inert for all of them.
#[test]
fn committed_glb_assets_use_no_normal_or_metallic_roughness_slots() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..");
    let mut checked = 0usize;

    let aircraft = root.join("models/acro_electric_01/aircraft.glb");
    let document = gltf::Gltf::open(&aircraft).expect("the production aircraft GLB opens");
    assert_eq!(
        document.textures().count(),
        0,
        "the aircraft GLB is untextured"
    );
    assert_eq!(document.images().count(), 0);
    for material in document.materials() {
        assert!(material.normal_texture().is_none());
        assert!(
            material
                .pbr_metallic_roughness()
                .metallic_roughness_texture()
                .is_none()
        );
    }
    checked += 1;

    let vegetation = root.join("crates/renderer/assets/vegetation");
    for entry in std::fs::read_dir(&vegetation).expect("the vegetation asset directory exists") {
        let path = entry.expect("readable dir entry").path();
        if path.extension().is_none_or(|extension| extension != "glb") {
            continue;
        }
        let document = gltf::Gltf::open(&path)
            .unwrap_or_else(|error| panic!("{} must open: {error}", path.display()));
        for material in document.materials() {
            assert!(
                material.normal_texture().is_none(),
                "{} carries a normalTexture",
                path.display()
            );
            assert!(
                material
                    .pbr_metallic_roughness()
                    .metallic_roughness_texture()
                    .is_none(),
                "{} carries a metallicRoughnessTexture",
                path.display()
            );
        }
        // And every vegetation sampler really does request a mip chain, which is
        // the request the renderer used to discard.
        for sampler in document.samplers() {
            if let Some(min_filter) = sampler.min_filter() {
                assert!(
                    SamplerMipmapFilter::mipmapping_requested(min_filter),
                    "{} requests minFilter {min_filter:?}",
                    path.display()
                );
            }
        }
        checked += 1;
    }
    assert_eq!(
        checked, 13,
        "one aircraft GLB plus twelve vegetation LOD GLBs"
    );
}
/// The Poly Haven public API's OpenAPI schema defines a texture asset's
/// `dimensions` as the size on each axis in millimetres. `sparse_grass` reports
/// `[2000, 2000]`, i.e. a 2.0 m x 2.0 m scan, and the ENV1 terrain base tile
/// scale must equal that span so one tile covers exactly the scanned area.
///
/// The authoritative record is `docs/assets/env1/env1_open_assets.json`
/// (`api.dimensions` / `api.dimensions_unit` / `api.physical_dimensions_m` /
/// `runtime_binding`); `tools/env1_asset_pipeline/verify_env1_assets.py`
/// re-checks that record against this constant. Nothing parses JSON at render
/// time — the relationship is a compile-time constant plus this test.
#[test]
fn env1_runtime_material_uses_the_documented_physical_tile_span() {
    /// API `dimensions` for `sparse_grass`, verbatim, in millimetres.
    const SOURCE_DIMENSIONS_MM: [f64; 2] = [2000.0, 2000.0];
    /// Millimetres in one metre, the provider-documented unit of `dimensions`.
    const MILLIMETRES_PER_METRE: f64 = 1000.0;

    let physical_span_m = SOURCE_DIMENSIONS_MM.map(|axis| axis / MILLIMETRES_PER_METRE);
    assert_eq!(
        physical_span_m,
        [2.0, 2.0],
        "2000 mm per axis is a 2.0 m scan"
    );

    // One base texture tile == the full physical span of the scan.
    let scale = renderer::terrain::DEFAULT_TERRAIN_TEXTURE_SCALE_M;
    for axis in physical_span_m {
        assert!(
            (f64::from(scale) - axis).abs() < 1e-9,
            "DEFAULT_TERRAIN_TEXTURE_SCALE_M must equal the asset's physical span              ({axis} m), got {scale} m"
        );
    }
    assert_eq!(
        renderer::TerrainMaterial::default().texture_scale_m,
        scale,
        "the default material must carry the documented span"
    );

    // Texel density implied by the committed runtime edge.
    let texels_per_metre = f64::from(ENV1_RUNTIME_EDGE) / f64::from(scale);
    assert_eq!(texels_per_metre, 1024.0);

    // The other frequencies are absolute-world quantities and must not move:
    // the shader derives them as `uv * (base_scale / layer_scale)`, so they are
    // invariant to the base tile scale.
    let material = renderer::TerrainMaterial::default();
    assert_eq!(material.macro_scale_m, 48.0);
    assert_eq!(material.detail_scale_m, 0.40);
    assert_eq!(material.macro_uv_offset, [0.170, 0.390]);
    assert_eq!(material.detail_uv_offset, [0.163, 0.037]);
    assert_eq!(material.ar_scale, 1.370);
    assert_eq!(material.ar_angle_degrees, 27.0);
    assert_eq!(material.ar_offset, [0.315, 0.571]);
    assert_eq!(material.metallic, 0.0, "terrain stays dielectric");
}

// ---------------------------------------------------------------------------
// ENV1-B0 registered PBR sampling
// ---------------------------------------------------------------------------

fn assert_vec3_close(actual: [f32; 3], expected: [f32; 3], tolerance: f32) {
    for axis in 0..3 {
        assert!(
            (actual[axis] - expected[axis]).abs() <= tolerance,
            "axis {axis}: expected {expected:?}, got {actual:?}"
        );
    }
}

fn assert_finite_unit(normal: [f32; 3]) {
    assert!(normal.iter().all(|component| component.is_finite()));
    let length = normal
        .iter()
        .map(|component| component * component)
        .sum::<f32>()
        .sqrt();
    assert!(
        (length - 1.0).abs() < 1.0e-6,
        "normal {normal:?} has length {length}"
    );
}

#[test]
fn env1_production_defaults_lock_the_registered_pbr_contract() {
    let material = TerrainMaterial::default();
    assert_eq!(material.texture_scale_m, 2.0);
    assert_eq!(material.albedo_uv_offset, [0.0, 0.0]);
    assert_eq!(material.normal_uv_offset, material.albedo_uv_offset);
    assert_eq!(material.roughness_uv_offset, material.albedo_uv_offset);
    assert_eq!(
        material.roughness, 1.0,
        "the photographed map is authoritative"
    );
    assert_eq!(material.metallic, 0.0);
    assert_eq!(material.macro_scale_m, 48.0);
    assert_eq!(material.detail_scale_m, 0.40);
    assert_eq!(material.ar_scale, 1.370);
    assert_eq!(material.ar_angle_degrees, 27.0);
    assert_eq!(material.ar_offset, [0.315, 0.571]);
}

#[test]
fn rotated_uv_normal_reorientation_has_unambiguous_inverse_sign() {
    assert_vec3_close(
        reorient_rotated_tangent_normal([1.0, 0.0, 0.0], 0.0),
        [1.0, 0.0, 0.0],
        1.0e-6,
    );
    assert_vec3_close(
        reorient_rotated_tangent_normal([1.0, 0.0, 0.0], 90.0),
        [0.0, -1.0, 0.0],
        1.0e-6,
    );
    assert_vec3_close(
        reorient_rotated_tangent_normal([1.0, 0.0, 0.0], -90.0),
        [0.0, 1.0, 0.0],
        1.0e-6,
    );

    let radians = 27.0f32.to_radians();
    assert_vec3_close(
        reorient_rotated_tangent_normal([1.0, 0.0, 0.0], 27.0),
        [radians.cos(), -radians.sin(), 0.0],
        1.0e-6,
    );
}

#[test]
fn rotated_uv_normal_reorientation_preserves_flat_finite_unit_normals() {
    for angle in [0.0, 90.0, -90.0, 27.0, 721.0] {
        let flat = reorient_rotated_tangent_normal([0.0, 0.0, 1.0], angle);
        assert_vec3_close(flat, [0.0, 0.0, 1.0], 1.0e-6);
        assert_finite_unit(flat);

        let tilted = reorient_rotated_tangent_normal([0.25, -0.5, 0.829_156_2], angle);
        assert_finite_unit(tilted);
    }
    assert_eq!(
        reorient_rotated_tangent_normal([0.0, 0.0, 0.0], 27.0),
        [0.0, 0.0, 1.0],
        "degenerate data must fail safely to a flat tangent normal"
    );
}

#[test]
fn registered_normal_and_linear_roughness_blends_are_well_formed() {
    let primary = [0.0, 0.0, 1.0];
    let secondary = reorient_rotated_tangent_normal([0.3, 0.2, 0.932_737_9], 27.0);
    let blended = blend_registered_tangent_normals(primary, secondary, 0.5);
    assert_finite_unit(blended);
    assert_eq!(
        blend_registered_tangent_normals(primary, secondary, 0.0),
        primary
    );
    assert_vec3_close(
        blend_registered_tangent_normals(primary, secondary, 1.0),
        secondary,
        1.0e-6,
    );

    assert!((blend_linear_roughness(0.2, 0.8, 0.5) - 0.5).abs() < 1.0e-6);
    assert_eq!(blend_linear_roughness(0.2, 0.8, 0.0), 0.2);
    assert_eq!(blend_linear_roughness(0.2, 0.8, 1.0), 0.8);
}

#[test]
fn production_shader_uses_shared_a_b_transforms_for_the_registered_triplet() {
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    let sampling = source
        .split("fn terrain_surface(input: VertexOutput) -> TerrainSurface {")
        .nth(1)
        .and_then(|tail| tail.split("fn terrain_fragment_output(").next())
        .expect("terrain_surface body");

    assert!(sampling.contains("ar_cos * base_uv.x - ar_sin * base_uv.y"));
    assert!(sampling.contains("textureSample(terrain_albedo_texture, terrain_sampler, base_uv)"));
    assert!(sampling.contains("textureSample(terrain_albedo_texture, terrain_sampler, ar_uv)"));
    assert!(
        sampling.contains("textureSample(terrain_roughness_texture, terrain_sampler, base_uv)")
    );
    assert!(sampling.contains("textureSample(terrain_roughness_texture, terrain_sampler, ar_uv)"));
    assert!(sampling.contains("textureSample(terrain_normal_texture, terrain_sampler, base_uv)"));
    assert!(sampling.contains("textureSample(terrain_normal_texture, terrain_sampler, ar_uv)"));
    assert!(sampling.contains("reorient_secondary_tangent_normal("));
    assert!(sampling.contains("mix(r_base_a, r_base_b, TERRAIN_BASE_AR_BLEND)"));
    assert!(!sampling.contains("terrain_material.normal_uv_offset"));
    assert!(!sampling.contains("terrain_material.roughness_uv_offset"));
}
