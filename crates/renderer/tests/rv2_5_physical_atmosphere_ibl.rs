//! RV2-5: static CPU validation of the physical atmosphere/IBL shaders and of
//! the V1/V2 scene-shader firewall.
//!
//! `cargo check` never sees WGSL (it is compiled by wgpu/naga only at GPU
//! runtime), so these tests run the exact naga 30.0.1 front end and validator
//! over every RV2-5 WGSL module and over the scene shader. They also prove
//! that the analytic/V1 entry points never statically reference the physical
//! bindings, which is what lets V1 keep its previous bind-group layout.

use naga::valid::{Capabilities, ModuleInfo, ValidationFlags, Validator};

const PHYSICAL_BINDING_GROUP: u32 = 2;
const PHYSICAL_BINDING_FIRST: u32 = 12;
const PHYSICAL_BINDING_LAST: u32 = 20;

fn parse_and_validate(source: &str, label: &str) -> (naga::Module, ModuleInfo) {
    let module = naga::front::wgsl::parse_str(source)
        .unwrap_or_else(|error| panic!("{label} must parse: {error}"));
    let mut validator = Validator::new(ValidationFlags::all(), Capabilities::all());
    let info = validator
        .validate(&module)
        .unwrap_or_else(|error| panic!("{label} must validate: {error}"));
    (module, info)
}

fn entry_point_index(module: &naga::Module, name: &str) -> usize {
    module
        .entry_points
        .iter()
        .position(|entry| entry.name == name)
        .unwrap_or_else(|| panic!("entry point {name} must exist"))
}

/// (handle, binding) for every RV2-5 physical binding at group 2.
fn physical_bindings(module: &naga::Module) -> Vec<(naga::Handle<naga::GlobalVariable>, u32)> {
    module
        .global_variables
        .iter()
        .filter_map(|(handle, variable)| {
            variable
                .binding
                .as_ref()
                .filter(|binding| {
                    binding.group == PHYSICAL_BINDING_GROUP
                        && (PHYSICAL_BINDING_FIRST..=PHYSICAL_BINDING_LAST)
                            .contains(&binding.binding)
                })
                .map(|binding| (handle, binding.binding))
        })
        .collect()
}

fn entry_point_uses_any(
    info: &ModuleInfo,
    module: &naga::Module,
    entry: &str,
    handles: &[naga::Handle<naga::GlobalVariable>],
) -> bool {
    let point = info.get_entry_point(entry_point_index(module, entry));
    handles.iter().any(|handle| !point[*handle].is_empty())
}

#[test]
fn atmosphere_generation_wgsl_parses_and_validates() {
    let source = include_str!("../src/renderer_v2/atmosphere.wgsl");
    let (module, _) = parse_and_validate(source, "atmosphere.wgsl");
    for entry in [
        "vs_fullscreen",
        "fs_transmittance",
        "fs_multi_scattering",
        "fs_sky_view",
        "fs_environment_cube",
    ] {
        entry_point_index(&module, entry);
    }
}

#[test]
fn ibl_convolution_wgsl_parses_and_validates() {
    let source = include_str!("../src/renderer_v2/ibl.wgsl");
    let (module, _) = parse_and_validate(source, "ibl.wgsl");
    for entry in [
        "vs_fullscreen",
        "fs_irradiance_cube",
        "fs_prefiltered_cube",
        "fs_brdf_lut",
    ] {
        entry_point_index(&module, entry);
    }
}

#[test]
fn scene_wgsl_parses_and_validates_with_the_physical_entry_points() {
    let source = include_str!("../src/shader.wgsl");
    let (module, _) = parse_and_validate(source, "shader.wgsl");
    for entry in [
        // V1 / analytic entry points must keep existing.
        "fs_sky",
        "fs_lit",
        "fs_terrain",
        "fs_vegetation",
        "fs_postprocess",
        // V2 physical entry points.
        "fs_sky_v2",
        "fs_lit_v2",
        "fs_terrain_v2",
        "fs_vegetation_v2",
    ] {
        entry_point_index(&module, entry);
    }
}

#[test]
fn physical_bindings_occupy_slots_12_to_20() {
    let (module, _) = parse_and_validate(include_str!("../src/shader.wgsl"), "shader.wgsl");
    let mut bindings: Vec<u32> = physical_bindings(&module)
        .into_iter()
        .map(|(_, binding)| binding)
        .collect();
    bindings.sort_unstable();
    assert_eq!(bindings, (12..=20).collect::<Vec<u32>>());
}

#[test]
fn v1_entry_points_never_reference_the_physical_bindings() {
    let (module, info) = parse_and_validate(include_str!("../src/shader.wgsl"), "shader.wgsl");
    let physical: Vec<_> = physical_bindings(&module)
        .into_iter()
        .map(|(handle, _)| handle)
        .collect();
    assert_eq!(physical.len(), 9, "nine physical bindings must be declared");
    for entry in [
        "fs_sky",
        "fs_lit",
        "fs_terrain",
        "fs_vegetation",
        "vs_main",
        "vs_shadow",
        "fs_vegetation_shadow",
    ] {
        assert!(
            !entry_point_uses_any(&info, &module, entry, &physical),
            "{entry} must not depend on the RV2-5 physical bindings"
        );
    }
}

#[test]
fn physical_entry_points_consume_the_physical_bindings() {
    let (module, info) = parse_and_validate(include_str!("../src/shader.wgsl"), "shader.wgsl");
    let physical: Vec<_> = physical_bindings(&module)
        .into_iter()
        .map(|(handle, _)| handle)
        .collect();
    for entry in [
        "fs_sky_v2",
        "fs_lit_v2",
        "fs_terrain_v2",
        "fs_vegetation_v2",
    ] {
        assert!(
            entry_point_uses_any(&info, &module, entry, &physical),
            "{entry} must sample the RV2-5 physical resources"
        );
    }
}

#[test]
fn physical_sky_disk_and_ibl_wiring_are_explicit() {
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    // Physical sky samples the production Sky-View LUT, never a heuristic.
    assert!(
        source.contains("var color = textureSample(sky_view_lut, atmosphere_sampler, sky_uv).rgb;")
    );
    // The sun disk uses the SunState angular radius, not a hard-coded cosine.
    assert!(source.contains("let cos_radius = cos(environment.atmosphere_state.x);"));
    // Direct PBR irradiance comes from the single SunState radiance.
    assert!(
        source.contains(
            "let irradiance = environment.sun_color.xyz * environment.atmosphere_state.y;"
        )
    );
    assert!(source.contains("* environment.sun_transmittance.rgb;"));
    // Specular IBL is the split-sum product, not a tinted placeholder.
    assert!(source.contains("return prefiltered * (fresnel * brdf.x + brdf.y);"));
    // The withdrawn heuristic tint must not come back.
    assert!(!source.contains("mix(prefiltered, environment_sample"));
}

#[test]
fn roughness_to_mip_mapping_is_continuous_and_clamped() {
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    assert!(
        source.contains("let mip = clamp(roughness, 0.0, 1.0) * 7.0;"),
        "roughness 0 -> mip 0 and roughness 1 -> mip 7 must stay explicit"
    );
}

#[test]
fn physical_generation_is_one_shot_and_never_frame_time() {
    let production = include_str!("../src/renderer_v2/atmosphere.rs").replace("\r\n", "\n");
    let production = production
        .split("#[cfg(test)]\nimpl EnvironmentTextures")
        .next()
        .expect("production source");
    assert!(
        !production.contains("create_physical_environment")
            || production.contains("pub(crate) fn create_physical_environment"),
        "the generation entry point is defined once"
    );
    // The generation path performs exactly one submit and no CPU pixel buffer.
    assert_eq!(production.matches("queue.submit(").count(), 1);
    assert!(!production.contains("copy_buffer_to_texture"));
    assert!(!production.contains("f32_to_f16"));
}
