//! G3C-A production aircraft asset validation (CPU-only).

use renderer::{ControlSurfacePresentation, GlbArticulationPlan, Mat4, SurfaceHinge, SurfaceId};
use std::path::{Path, PathBuf};

const EXPECTED_PARTS: [&str; 16] = [
    "Fuselage",
    "Cowl",
    "Spinner",
    "Propeller",
    "Canopy",
    "MainWingFixed",
    "LeftAileron",
    "RightAileron",
    "HorizontalStabilizer",
    "Elevator",
    "VerticalStabilizer",
    "Rudder",
    "MainLandingGear",
    "NoseLandingGear",
    "Wheels",
    "WingAndFuselageLivery",
];

fn asset_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join("models/acro_electric_01/aircraft.glb")
}

fn bounds(primitive: &renderer::RenderPrimitive) -> ([f32; 3], [f32; 3]) {
    let mut minimum = [f32::INFINITY; 3];
    let mut maximum = [f32::NEG_INFINITY; 3];
    for vertex in &primitive.vertices {
        for axis in 0..3 {
            minimum[axis] = minimum[axis].min(vertex.position[axis]);
            maximum[axis] = maximum[axis].max(vertex.position[axis]);
        }
    }
    (minimum, maximum)
}

#[test]
fn production_loader_accepts_detailed_indexed_asset() {
    let path = asset_path();
    assert!(path.is_file(), "production GLB must be checked in");
    let asset = renderer::load_glb_asset(&path).expect("production loader must accept G3C-A GLB");

    assert_eq!(asset.primitives.len(), EXPECTED_PARTS.len());
    assert!(asset.total_vertex_count() >= 2_000);
    assert!(asset.total_index_count() / 3 >= 3_000);

    let mut global_minimum = [f32::INFINITY; 3];
    let mut global_maximum = [f32::NEG_INFINITY; 3];
    for primitive in &asset.primitives {
        assert!(!primitive.vertices.is_empty());
        assert!(!primitive.indices.is_empty());
        assert_eq!(primitive.indices.len() % 3, 0);
        assert!(
            primitive
                .indices
                .iter()
                .all(|&index| (index as usize) < primitive.vertices.len())
        );
        for vertex in &primitive.vertices {
            assert!(vertex.position.iter().all(|value| value.is_finite()));
            assert!(vertex.normal.iter().all(|value| value.is_finite()));
            let length_squared = vertex.normal.iter().map(|value| value * value).sum::<f32>();
            assert!((length_squared - 1.0).abs() < 2.0e-4);
            for axis in 0..3 {
                global_minimum[axis] = global_minimum[axis].min(vertex.position[axis]);
                global_maximum[axis] = global_maximum[axis].max(vertex.position[axis]);
            }
        }
    }

    assert!(global_minimum[0] <= -0.89 && global_maximum[0] >= 0.89);
    assert!(global_minimum[1] < -0.35 && global_maximum[1] > 0.53);
    assert!(global_minimum[2] < -0.87 && global_maximum[2] >= 0.89);
    assert!(global_maximum[0] - global_minimum[0] < 2.0);
    assert!(global_maximum[2] - global_minimum[2] < 2.0);

    // Structural nose-direction check: the compact spinner is the foremost
    // primitive and the propeller lies ahead of the cowl in the -Z direction.
    let (spinner_minimum, spinner_maximum) = bounds(&asset.primitives[2]);
    let (propeller_minimum, propeller_maximum) = bounds(&asset.primitives[3]);
    let (cowl_minimum, _) = bounds(&asset.primitives[1]);
    assert!((spinner_minimum[2] - global_minimum[2]).abs() < 1.0e-5);
    assert!(spinner_maximum[0] - spinner_minimum[0] < 0.25);
    assert!(propeller_minimum[2] < cowl_minimum[2]);
    assert!(propeller_maximum[1] - propeller_minimum[1] > 0.70);
}

#[test]
fn authored_parts_materials_and_surface_primitives_are_distinct() {
    let path = asset_path();
    let document = gltf::Gltf::open(&path).expect("valid glTF 2.0 GLB");
    assert!(document.blob.is_some(), "all asset data must be embedded");
    assert_eq!(document.materials().count(), 8);
    assert_eq!(document.meshes().count(), EXPECTED_PARTS.len());

    assert_eq!(
        document.meshes().flat_map(|mesh| mesh.primitives()).count(),
        EXPECTED_PARTS.len()
    );

    let asset = renderer::load_glb_asset(&path).unwrap();
    let (fuselage_minimum, fuselage_maximum) = bounds(&asset.primitives[0]);
    assert!(fuselage_maximum[2] - fuselage_minimum[2] > 1.25);
    assert!(fuselage_maximum[0] - fuselage_minimum[0] > 0.30);
    let (canopy_minimum, canopy_maximum) = bounds(&asset.primitives[4]);
    assert!(canopy_maximum[1] > 0.30);
    assert!(canopy_maximum[2] - canopy_minimum[2] > 0.60);
    let (wing_minimum, wing_maximum) = bounds(&asset.primitives[5]);
    assert!(wing_maximum[0] - wing_minimum[0] >= 1.79);
    assert!(wing_maximum[2] - wing_minimum[2] > 0.50);

    let (left_minimum, left_maximum) = bounds(&asset.primitives[6]);
    assert!(left_maximum[0] < -0.25 && left_minimum[0] < -0.85);
    let (right_minimum, right_maximum) = bounds(&asset.primitives[7]);
    assert!(right_minimum[0] > 0.25 && right_maximum[0] > 0.85);
    assert!(left_maximum[2] - left_minimum[2] > 0.08);
    assert!(right_maximum[2] - right_minimum[2] > 0.08);

    let (elevator_minimum, elevator_maximum) = bounds(&asset.primitives[9]);
    assert!(elevator_minimum[0] < -0.49 && elevator_maximum[0] > 0.49);
    let (rudder_minimum, rudder_maximum) = bounds(&asset.primitives[11]);
    assert!(rudder_maximum[1] > 0.50);
    assert!(rudder_maximum[2] - rudder_minimum[2] > 0.04);

    let (main_gear_minimum, main_gear_maximum) = bounds(&asset.primitives[12]);
    assert!(main_gear_minimum[1] < -0.29 && main_gear_maximum[0] > 0.30);
    let (nose_gear_minimum, nose_gear_maximum) = bounds(&asset.primitives[13]);
    assert!(nose_gear_minimum[2] < -0.50 && nose_gear_maximum[2] < -0.43);
    let (wheels_minimum, wheels_maximum) = bounds(&asset.primitives[14]);
    assert!(wheels_minimum[1] < -0.35 && wheels_maximum[0] > 0.29);
}

#[test]
fn production_surface_mapping_has_identity_neutral_and_finite_deflections() {
    let mappings = [
        (
            6,
            SurfaceId::LeftAileron,
            [0.0, 0.01, 0.11],
            [1.0, -0.054, 0.0],
        ),
        (
            7,
            SurfaceId::RightAileron,
            [0.0, 0.01, 0.11],
            [1.0, 0.054, 0.0],
        ),
        (9, SurfaceId::Elevator, [0.0, 0.112, 0.735], [1.0, 0.0, 0.0]),
        (11, SurfaceId::Rudder, [0.0, 0.12, 0.735], [0.0, 1.0, 0.0]),
    ]
    .map(|(primitive, surface, origin, axis)| {
        (
            primitive,
            SurfaceHinge::new(surface, origin, axis, 1.0).unwrap(),
        )
    });
    let plan = GlbArticulationPlan::from_mappings(EXPECTED_PARTS.len(), mappings).unwrap();
    let neutral = ControlSurfacePresentation::neutral();
    let positive = ControlSurfacePresentation::new(0.25, 0.25, 0.25, 0.25, 0.0).unwrap();
    let negative = ControlSurfacePresentation::new(-0.25, -0.25, -0.25, -0.25, 0.0).unwrap();

    for primitive in [6, 7, 9, 11] {
        assert_eq!(
            plan.composed_matrix(&Mat4::identity(), primitive, &neutral),
            Mat4::identity()
        );
        for presentation in [&positive, &negative] {
            let matrix = plan.composed_matrix(&Mat4::identity(), primitive, presentation);
            assert_ne!(matrix, Mat4::identity());
            assert!(
                matrix
                    .rows()
                    .iter()
                    .flatten()
                    .all(|value| value.is_finite())
            );
        }
    }
}
