//! RV2-6 static validation for physical aerial perspective and the V1 fence.

use naga::valid::{Capabilities, ModuleInfo, ValidationFlags, Validator};

const ENVIRONMENT_GROUP: u32 = 2;
const AERIAL_PERSPECTIVE_BINDING: u32 = 5;

fn parse_and_validate(source: &str) -> (naga::Module, ModuleInfo) {
    let module = naga::front::wgsl::parse_str(source)
        .unwrap_or_else(|error| panic!("RV2-6 scene WGSL must parse: {error}"));
    let mut validator = Validator::new(ValidationFlags::all(), Capabilities::all());
    let info = validator
        .validate(&module)
        .unwrap_or_else(|error| panic!("RV2-6 scene WGSL must validate: {error}"));
    (module, info)
}

fn entry_point_index(module: &naga::Module, name: &str) -> usize {
    module
        .entry_points
        .iter()
        .position(|entry| entry.name == name)
        .unwrap_or_else(|| panic!("entry point {name} must exist"))
}

fn function_body<'a>(source: &'a str, name: &str) -> &'a str {
    let signature = format!("fn {name}(");
    let start = source
        .find(&signature)
        .unwrap_or_else(|| panic!("function {name} must exist"));
    let rest = &source[start..];
    let end = rest[signature.len()..]
        .find("\n@fragment")
        .map_or(rest.len(), |offset| signature.len() + offset);
    &rest[..end]
}

#[test]
fn scene_wgsl_parses_and_validates_with_rv2_6() {
    let source = include_str!("../src/shader.wgsl");
    let (module, _) = parse_and_validate(source);
    for entry in [
        "fs_lit",
        "fs_terrain",
        "fs_vegetation",
        "fs_sky_v2",
        "fs_lit_v2",
        "fs_terrain_v2",
        "fs_vegetation_v2",
    ] {
        entry_point_index(&module, entry);
    }
}

#[test]
fn aerial_perspective_binding_is_v2_only_and_uses_free_slot_five() {
    let (module, info) = parse_and_validate(include_str!("../src/shader.wgsl"));
    let (handle, _) = module
        .global_variables
        .iter()
        .find(|(_, variable)| {
            variable.binding.as_ref().is_some_and(|binding| {
                binding.group == ENVIRONMENT_GROUP && binding.binding == AERIAL_PERSPECTIVE_BINDING
            })
        })
        .expect("group 2 binding 5 must hold the aerial-perspective uniform");

    for entry in ["fs_lit", "fs_terrain", "fs_vegetation", "fs_sky_v2"] {
        let point = info.get_entry_point(entry_point_index(&module, entry));
        assert!(
            point[handle].is_empty(),
            "{entry} must not depend on aerial perspective"
        );
    }
    for entry in ["fs_lit_v2", "fs_terrain_v2", "fs_vegetation_v2"] {
        let point = info.get_entry_point(entry_point_index(&module, entry));
        assert!(
            !point[handle].is_empty(),
            "{entry} must consume aerial perspective"
        );
    }
}

#[test]
fn v1_keeps_legacy_fog_while_physical_geometry_uses_rv2_6() {
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    for entry in ["fs_lit", "fs_terrain", "fs_vegetation"] {
        assert!(
            function_body(&source, entry).contains("apply_distance_fog("),
            "{entry} must preserve legacy distance fog"
        );
        assert!(!function_body(&source, entry).contains("apply_physical_aerial_perspective("));
    }
    for entry in ["fs_lit_v2", "fs_terrain_v2", "fs_vegetation_v2"] {
        assert!(
            function_body(&source, entry).contains("apply_physical_aerial_perspective("),
            "{entry} must use physical aerial perspective"
        );
        assert!(!function_body(&source, entry).contains("apply_distance_fog("));
    }
    assert!(!function_body(&source, "fs_sky_v2").contains("apply_physical_aerial_perspective("));
    assert!(source.contains(
        "return surface_radiance * physical.transmittance_rgb\n        + physical.inscattered_radiance_rgb;"
    ));
    assert!(!function_body(&source, "apply_physical_aerial_perspective").contains("mix(surface"));
    assert!(source.contains("camera.camera_position.xyz"));
    assert!(source.contains("world_position.y - aerial_perspective.planet_ground.z"));
    assert!(source.contains("const AERIAL_PERSPECTIVE_SAMPLE_COUNT: i32 = 4;"));
    assert!(source.contains("aerial_perspective.validation_control.x < 0.5"));
}

#[test]
fn rv2_5_bindings_and_temporal_graph_remain_stable() {
    let shader = include_str!("../src/shader.wgsl");
    for binding in 12..=20 {
        assert!(
            shader.contains(&format!("@group(2) @binding({binding})")),
            "RV2-5 binding {binding} must not move"
        );
    }

    let graph = include_str!("../src/render_graph.rs").replace("\r\n", "\n");
    assert!(graph.contains("pub(crate) const COUNT: usize = 6;"));
    for pass in [
        "PassId::ShadowNear",
        "PassId::ShadowMid",
        "PassId::ShadowFar",
        "PassId::Scene",
        "PassId::TemporalResolve",
        "PassId::Postprocess",
    ] {
        assert!(graph.contains(pass), "graph must retain {pass}");
    }
}

#[test]
fn fallback_temporal_and_resource_lifecycle_stay_bounded() {
    let gpu = include_str!("../src/gpu.rs").replace("\r\n", "\n");
    assert!(gpu.contains("V2EnvironmentMode::AnalyticalFallback"));
    assert!(gpu.contains("(\"fs_sky\", \"fs_lit\", \"fs_terrain\", \"fs_vegetation\")"));
    assert!(gpu.contains("let aerial_perspective_buffer = use_physical.then(||"));
    assert!(
        gpu.contains(".with_validation_enabled(initialization_policy.aerial_perspective_enabled)")
    );

    let backend = include_str!("../src/backend.rs").replace("\r\n", "\n");
    assert!(backend.contains("new_v2_for_rv2_6_validation"));
    assert!(backend.contains("new_with_presentation_for_rv2_6_validation"));

    let (_, frame_and_after) = gpu
        .split_once("pub fn render(&mut self, frame: &RenderFrame)")
        .expect("renderer frame path must exist");
    let (frame_path, _) = frame_and_after
        .split_once("fn check_asynchronous_gpu_error")
        .expect("renderer frame path boundary must exist");
    for forbidden in [
        "AerialPerspectiveUniformRaw::new",
        "create_buffer_init",
        "create_bind_group",
        "create_texture",
        "create_render_pipeline",
    ] {
        assert!(
            !frame_path.contains(forbidden),
            "frame path must not contain {forbidden}"
        );
    }

    let temporal = include_str!("../src/renderer_v2/temporal.rs");
    assert!(!temporal.contains("aerial_perspective"));
    assert!(!temporal.contains("AerialPerspective"));
}
