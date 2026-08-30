#![forbid(unsafe_code)]
//! Shared mathematical types and frame conversion helpers for the simulation domain.

pub use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

/// Canonical three-dimensional vector type. Units are carried by field names and docs.
pub type Vec3 = Vector3<f64>;
/// Canonical 3x3 matrix type.
pub type Mat3 = Matrix3<f64>;
/// Hamilton unit quaternion used as an active body-to-world rotation.
pub type Orientation = UnitQuaternion<f64>;

/// Transforms a body-frame vector into the NED world frame.
#[must_use]
pub fn body_to_world(orientation_world_from_body: &Orientation, vector_body: &Vec3) -> Vec3 {
    orientation_world_from_body.transform_vector(vector_body)
}

/// Transforms a NED world-frame vector into the FRD body frame.
#[must_use]
pub fn world_to_body(orientation_world_from_body: &Orientation, vector_world: &Vec3) -> Vec3 {
    orientation_world_from_body.inverse_transform_vector(vector_world)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const ALGEBRA_TOLERANCE: f64 = 32.0 * f64::EPSILON;

    fn assert_vec_close(actual: Vec3, expected: Vec3) {
        assert!(
            (actual - expected).norm() <= ALGEBRA_TOLERANCE,
            "actual={actual:?}, expected={expected:?}"
        );
    }

    #[test]
    fn t5_identity_orientation_preserves_body_vector() {
        let vector_body = Vec3::new(2.0, -3.0, 4.0);
        assert_vec_close(
            body_to_world(&Orientation::identity(), &vector_body),
            vector_body,
        );
    }

    #[test]
    fn t6_positive_ninety_degree_yaw_maps_forward_to_east() {
        let orientation_world_from_body =
            Orientation::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert_vec_close(
            body_to_world(&orientation_world_from_body, &Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.0, 1.0, 0.0),
        );
    }
}
