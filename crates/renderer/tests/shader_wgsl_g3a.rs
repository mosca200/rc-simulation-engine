//! G3A: static WGSL validation of the renderer shader module.
//!
//! `cargo check` never sees the WGSL (it is authored as a text resource and
//! compiled by wgpu/naga only at GPU runtime). This test runs the same naga
//! front end (pinned to the wgpu 30 lock version) over `shader.wgsl`, so
//! syntax and semantic regressions are caught on CPU-only CI runners.

#[test]
fn shader_wgsl_parses_and_validates() {
    let source = include_str!("../src/shader.wgsl");
    let module = naga::front::wgsl::parse_str(source)
        .unwrap_or_else(|error| panic!("shader.wgsl must parse: {error}"));
    let mut validator = naga::valid::Validator::new(
        naga::valid::ValidationFlags::all(),
        naga::valid::Capabilities::all(),
    );
    validator
        .validate(&module)
        .unwrap_or_else(|error| panic!("shader.wgsl must validate: {error}"));
}

#[test]
fn shader_defines_the_g3a_terrain_entry_points() {
    // Normalize CRLF so the assertions hold on any checkout.
    let source = include_str!("../src/shader.wgsl").replace("\r\n", "\n");
    assert!(
        source.contains("fn fs_terrain("),
        "terrain fragment must exist"
    );
    assert!(
        source.contains("fn lit_pbr_response("),
        "terrain must share the PBR response"
    );
    assert!(
        source.contains("struct TerrainMaterialUniform"),
        "terrain material uniform must exist"
    );
    assert!(
        source.contains("@group(4) @binding(4)\nvar<uniform> terrain_material"),
        "terrain uniform must live at group 4 binding 4"
    );
}
