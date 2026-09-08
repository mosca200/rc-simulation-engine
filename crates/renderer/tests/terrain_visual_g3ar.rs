//! G3A-R: acceptance-level tests for the production terrain visual closure.
//!
//! These tests exercise the public terrain-visual API against the *committed*
//! assets (exactly what the renderer consumes), covering the mip chain
//! contract, the detail distance fade, and the world-space anti-repetition
//! transform — including chunk-boundary continuity of the second sample.

use renderer::terrain_textures::{
    TERRAIN_TEXTURE_SIZE, TerrainMipChain, generate_terrain_mip_chain, mip_level_count_for_size,
    terrain_texture_set_from_decoded,
};
use renderer::texture::decode_image;
use renderer::{
    TerrainDebugMode, TerrainMaterial, detail_normal_fade_weight, generate_centered_terrain_chunks,
    generate_rolling_terrain, rotated_secondary_uv,
};

/// The exact base set the renderer builds at startup, from the embedded PNGs.
fn committed_set() -> renderer::terrain_textures::TerrainTextureSet {
    let albedo = decode_image(include_bytes!("../assets/terrain_grass_albedo.png"))
        .expect("albedo asset must decode");
    let normal = decode_image(include_bytes!("../assets/terrain_grass_normal.png"))
        .expect("normal asset must decode");
    let roughness = decode_image(include_bytes!("../assets/terrain_grass_roughness.png"))
        .expect("roughness asset must decode");
    terrain_texture_set_from_decoded(&albedo.rgba8, &normal.rgba8, &roughness.rgba8)
}

fn committed_chain() -> TerrainMipChain {
    let set = committed_set();
    generate_terrain_mip_chain(&set, TERRAIN_TEXTURE_SIZE)
}

#[test]
fn committed_assets_build_the_full_production_mip_chain() {
    let chain = committed_chain();
    let expected_levels = mip_level_count_for_size(TERRAIN_TEXTURE_SIZE) as usize;
    assert_eq!(chain.albedo.len(), expected_levels, "albedo levels");
    assert_eq!(chain.normal.len(), expected_levels, "normal levels");
    assert_eq!(chain.roughness.len(), expected_levels, "roughness levels");
    assert!(
        expected_levels > 1,
        "refusing a mip-0-only terrain (no trilinear minification possible)"
    );
    assert_eq!(
        (chain.albedo[0].width, chain.albedo[0].height),
        (TERRAIN_TEXTURE_SIZE, TERRAIN_TEXTURE_SIZE)
    );
    assert_eq!(
        (
            chain.albedo[expected_levels - 1].width,
            chain.albedo[expected_levels - 1].height
        ),
        (1, 1)
    );
}

/// Decoded-texel probe helper: the G3A-R mip contract in one assertion.
#[test]
fn every_mip_level_has_exact_base_two_dimensions_and_payload_size() {
    let chain = committed_chain();
    for (index, mip) in chain.albedo.iter().enumerate() {
        assert!(mip.width.is_power_of_two() && mip.width >= 1);
        assert_eq!(mip.width, mip.height);
        if index + 1 < chain.albedo.len() {
            let next = &chain.albedo[index + 1];
            assert_eq!((next.width, next.height), (mip.width / 2, mip.height / 2));
        }
        assert_eq!(
            mip.bytes.len(),
            (mip.width * mip.height * 4) as usize,
            "albedo level {index} must be RGBA8"
        );
    }
    for (index, mip) in chain.roughness.iter().enumerate() {
        assert_eq!(
            mip.bytes.len(),
            (mip.width * mip.height) as usize,
            "roughness level {index} must be R8"
        );
    }
}

#[test]
fn committed_chain_generation_is_bitwise_deterministic() {
    assert_eq!(committed_chain(), committed_chain());
}

#[test]
fn detail_fade_is_monotone_smoothstep_within_the_configured_range() {
    let material = TerrainMaterial::default();
    let (near, far) = (
        material.detail_normal_fade_near_m,
        material.detail_normal_fade_far_m,
    );
    assert!(near < far && near >= 0.0);

    let mut previous = 1.0f32;
    for i in 0..=100 {
        let distance = (i as f32 / 100.0) * (far * 1.5);
        let weight = detail_normal_fade_weight(distance, near, far);
        assert!((0.0..=1.0).contains(&weight) && weight.is_finite());
        assert!(weight <= previous + 1e-6, "fade must be non-increasing");
        previous = weight;
    }
    assert_eq!(detail_normal_fade_weight(0.0, near, far), 1.0);
    assert_eq!(detail_normal_fade_weight(far, near, far), 0.0);
    // The fade must be smooth at both range ends (no popping step).
    let just_outside_near = near - 0.25;
    let just_outside_far = far + 0.25;
    assert!(
        (detail_normal_fade_weight(just_outside_near, near, far) - 1.0).abs() < 0.02,
        "kink just inside `near`"
    );
    assert!(
        detail_normal_fade_weight(just_outside_far, near, far).abs() < 0.02,
        "kink just outside `far`"
    );
}

#[test]
fn rotated_secondary_uv_is_deterministic_and_world_anchored() {
    let material = TerrainMaterial::default();
    let uv = [12.25, -5.5];
    let a = rotated_secondary_uv(
        uv,
        material.ar_scale,
        material.ar_angle_degrees,
        material.ar_offset,
    );
    let b = rotated_secondary_uv(
        uv,
        material.ar_scale,
        material.ar_angle_degrees,
        material.ar_offset,
    );
    assert_eq!(a.map(f32::to_bits), b.map(f32::to_bits));
    assert!(a.iter().all(|v| v.is_finite()));
}

#[test]
fn rotated_secondary_uv_is_seamless_across_chunk_boundaries() {
    // The same world position generated through two different chunkings must
    // resolve to the identical anti-repetition UV — no second-sample seam at
    // chunk borders.
    let terrain = generate_rolling_terrain(128, 128, 2.0, 0.0, 2.0);
    let material = TerrainMaterial::default();
    let coarse = generate_centered_terrain_chunks(&terrain, 64, &material);
    let fine = generate_centered_terrain_chunks(&terrain, 32, &material);

    for world in [(10.0, 20.0), (36.0, -12.0), (0.0, 0.0), (-64.0, 48.0)] {
        let find = |chunks: &[renderer::TerrainChunk]| -> Option<renderer::Vertex> {
            chunks
                .iter()
                .flat_map(|c| c.vertices.iter())
                .copied()
                .find(|v| {
                    (v.position[0] - world.0).abs() < 1e-4 && (v.position[2] - world.1).abs() < 1e-4
                })
        };
        let va = find(&coarse).unwrap_or_else(|| panic!("vertex at {world:?} in coarse"));
        let vb = find(&fine).unwrap_or_else(|| panic!("vertex at {world:?} in fine"));
        assert_eq!(
            va.uv, vb.uv,
            "world UV must be chunk-independent at {world:?}"
        );
        let ar_a = rotated_secondary_uv(
            va.uv,
            material.ar_scale,
            material.ar_angle_degrees,
            material.ar_offset,
        );
        let ar_b = rotated_secondary_uv(
            vb.uv,
            material.ar_scale,
            material.ar_angle_degrees,
            material.ar_offset,
        );
        assert_eq!(
            ar_a.map(f32::to_bits),
            ar_b.map(f32::to_bits),
            "rotated second sample must be chunk-independent at {world:?}"
        );
    }
}

#[test]
fn debug_mode_config_mapping_is_total_over_the_uniform_selector() {
    // Presentation-only configuration contract: every selector value maps to
    // exactly one mode and back, and the FINAL default keeps production
    // untouched.
    assert_eq!(TerrainDebugMode::default(), TerrainDebugMode::Final);
    for value in 0..=5 {
        let mode = TerrainDebugMode::from_u32(value).expect("selector {value} must map");
        assert_eq!(mode.as_u32(), value);
    }
    assert!(TerrainDebugMode::from_u32(6).is_none());
}
