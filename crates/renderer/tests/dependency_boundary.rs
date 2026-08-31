use std::{fs, path::Path};

#[test]
fn renderer_manifest_has_no_physics_model_or_aircraft_dependencies() {
    let manifest_path = Path::new(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
    let manifest = fs::read_to_string(&manifest_path)
        .unwrap_or_else(|error| panic!("failed to read {}: {error}", manifest_path.display()));
    let dependency_section = manifest
        .split_once("[dependencies]")
        .map(|(_, remainder)| remainder.split("\n[").next().unwrap_or(remainder))
        .unwrap_or("");

    for forbidden_dependency in ["sim_core", "sim_math", "model", "aircraft"] {
        assert!(
            !dependency_section.lines().any(|line| {
                line.trim_start()
                    .starts_with(&format!("{forbidden_dependency} "))
            }),
            "renderer must not depend on {forbidden_dependency}"
        );
    }
}
