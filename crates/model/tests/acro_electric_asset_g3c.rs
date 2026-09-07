//! G3C-A presentation contract and physics-isolation regression tests.

use model::{PresentationSurface, load_aircraft_model};
use std::path::{Path, PathBuf};

const EXPECTED_PHYSICS_FINGERPRINT: &str =
    "07c48378ad0f8de786f0927c1bba206681c4153deb50174bf0c518d6eae5ba73";

fn model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("models/acro_electric_01/model.json")
}

fn fingerprint_hex(bytes: &[u8; 32]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[test]
fn production_presentation_maps_all_four_existing_control_bindings() {
    let path = model_path();
    let model = load_aircraft_model(&path).expect("production model must load");
    let presentation = model
        .presentation()
        .expect("production presentation metadata");
    assert_eq!(presentation.glb_path(), "aircraft.glb");
    assert!(
        path.parent()
            .unwrap()
            .join(presentation.glb_path())
            .is_file()
    );

    let expected = [
        (6, PresentationSurface::LeftAileron, "aileron-left"),
        (7, PresentationSurface::RightAileron, "aileron-right"),
        (9, PresentationSurface::Elevator, "elevator"),
        (11, PresentationSurface::Rudder, "rudder"),
    ];
    assert_eq!(presentation.articulated_surfaces().len(), expected.len());
    for (mapping, (primitive, surface, binding)) in
        presentation.articulated_surfaces().iter().zip(expected)
    {
        assert_eq!(mapping.visual_primitive_index(), primitive);
        assert_eq!(mapping.surface(), surface);
        assert_eq!(mapping.control_surface_binding_id(), binding);
        assert!(
            mapping
                .hinge_origin_render_body_m()
                .iter()
                .chain(mapping.hinge_axis_render_body().iter())
                .all(|value| value.is_finite())
        );
        assert!(mapping.visual_gain().is_finite());
    }
}

#[test]
fn presentation_asset_change_preserves_required_physics_fingerprint() {
    let model = load_aircraft_model(model_path()).expect("production model must load");
    assert_eq!(
        fingerprint_hex(model.physics_fingerprint().as_bytes()),
        EXPECTED_PHYSICS_FINGERPRINT
    );
}
