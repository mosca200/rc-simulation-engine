use std::path::PathBuf;

#[test]
fn platform_manifest_has_no_renderer_or_simulation_owner_dependencies() {
    let manifest =
        std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml"))
            .unwrap();
    for forbidden in ["renderer", "wgpu", "winit", "aircraft", "model", "replay"] {
        assert!(
            !manifest.lines().any(|line| {
                line.split_once('=')
                    .is_some_and(|(name, _)| name.trim() == forbidden)
            }),
            "forbidden platform dependency {forbidden}"
        );
    }
}
