//! G1E moving-surface presentation tests (CPU-only, no GPU required).
use renderer::{
    ControlSurfacePresentation, Mat4, SurfaceBindingTable, SurfaceHinge, SurfaceId,
    articulated_aircraft_mesh, articulated_binding_table,
};

fn close(left: &Mat4, right: &Mat4, tol: f32) {
    for (lr, rr) in left.rows().iter().zip(right.rows().iter()) {
        for (lv, rv) in lr.iter().zip(rr.iter()) {
            assert!((lv - rv).abs() <= tol, "left={left:?} right={right:?}");
        }
    }
}

fn state(la: f32, ra: f32, el: f32, ru: f32) -> ControlSurfacePresentation {
    ControlSurfacePresentation::new(la, ra, el, ru, 0.0).unwrap()
}

#[test]
fn neutral_surface_transforms_are_identity() {
    let table = articulated_binding_table();
    let neutral = ControlSurfacePresentation::neutral();
    for surface in SurfaceId::control_surfaces() {
        assert_eq!(table.local_matrix(surface, &neutral), Mat4::identity());
    }
    let fixture = articulated_aircraft_mesh();
    for surface in SurfaceId::control_surfaces() {
        assert!(fixture.surface(surface).is_some());
    }
    assert!(fixture.surface(SurfaceId::Propeller).is_none());
}

#[test]
fn elevator_positive_and_negative_rotate_oppositely() {
    let table = articulated_binding_table();
    let positive = table.local_matrix(SurfaceId::Elevator, &state(0.0, 0.0, 0.3, 0.0));
    let negative = table.local_matrix(SurfaceId::Elevator, &state(0.0, 0.0, -0.3, 0.0));
    assert_ne!(positive, negative);
    assert_ne!(positive, Mat4::identity());
    assert_ne!(negative, Mat4::identity());
    close(&(positive * negative), &Mat4::identity(), 1.0e-5);
    let up = positive.transform_homogeneous([0.0, 0.0, 0.70, 1.0]);
    let down = negative.transform_homogeneous([0.0, 0.0, 0.70, 1.0]);
    assert!((up[1] - down[1]).abs() > 1.0e-4);
    assert!(up[1] * down[1] < 0.0);
}

#[test]
fn rudder_rotates_about_vertical_axis() {
    let table = articulated_binding_table();
    let hinge = table.hinge(SurfaceId::Rudder).unwrap();
    assert_eq!(hinge.axis_render_body(), [0.0, 1.0, 0.0]);
    let deflected = table.local_matrix(SurfaceId::Rudder, &state(0.0, 0.0, 0.0, 0.25));
    assert_ne!(deflected, Mat4::identity());
    let pivot = hinge.pivot_render_body_m();
    let fixed = deflected.transform_homogeneous([pivot[0], pivot[1], pivot[2], 1.0]);
    assert!((fixed[0] - pivot[0]).abs() < 1.0e-5);
    assert!((fixed[1] - pivot[1]).abs() < 1.0e-5);
    assert!((fixed[2] - pivot[2]).abs() < 1.0e-5);
    let trailing = deflected.transform_homogeneous([0.0, 0.5, 0.69, 1.0]);
    assert!(trailing[0] != 0.0);
}

#[test]
fn differential_ailerons_move_oppositely() {
    let table = articulated_binding_table();
    let roll = state(0.25, -0.25, 0.0, 0.0);
    let left = table.local_matrix(SurfaceId::LeftAileron, &roll);
    let right = table.local_matrix(SurfaceId::RightAileron, &roll);
    assert_ne!(left, Mat4::identity());
    assert_ne!(right, Mat4::identity());
    let left_tip = left.transform_homogeneous([-0.82, 0.0, 0.20, 1.0]);
    let right_tip = right.transform_homogeneous([0.82, 0.0, 0.20, 1.0]);
    assert!(left_tip[1] * right_tip[1] < 0.0);
}

#[test]
fn root_pose_and_local_transform_compose() {
    let table = articulated_binding_table();
    let root = Mat4::from_rows([
        [1.0, 0.0, 0.0, 2.0],
        [0.0, 1.0, 0.0, -3.0],
        [0.0, 0.0, 1.0, 5.0],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    let deflected = state(0.0, 0.0, 0.2, 0.0);
    let composed = table.composed_matrix(&root, SurfaceId::Elevator, &deflected);
    close(
        &composed,
        &(root * table.local_matrix(SurfaceId::Elevator, &deflected)),
        1.0e-6,
    );
    assert_eq!(
        table.composed_matrix(
            &root,
            SurfaceId::Elevator,
            &ControlSurfacePresentation::neutral()
        ),
        root
    );
}

#[test]
fn unbound_slots_stay_rigid_and_hinges_validate() {
    let table = SurfaceBindingTable::empty();
    let deflected = state(0.5, -0.5, 0.5, 0.5);
    for surface in SurfaceId::control_surfaces() {
        assert_eq!(table.local_matrix(surface, &deflected), Mat4::identity());
    }
    assert!(SurfaceHinge::new(SurfaceId::Elevator, [0.0; 3], [0.0; 3], 1.0).is_none());
    assert!(SurfaceHinge::new(SurfaceId::Elevator, [f32::NAN; 3], [1.0, 0.0, 0.0], 1.0).is_none());
    assert!(ControlSurfacePresentation::new(0.0, 0.0, 0.0, f32::INFINITY, 0.0).is_none());
}

#[test]
fn transform_output_is_deterministic() {
    let table = articulated_binding_table();
    let present = state(0.12, -0.12, 0.07, -0.05);
    for surface in SurfaceId::control_surfaces() {
        assert_eq!(
            table.local_matrix(surface, &present),
            table.local_matrix(surface, &present)
        );
    }
}
