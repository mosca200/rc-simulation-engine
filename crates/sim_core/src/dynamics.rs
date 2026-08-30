use crate::{RigidBodyParams, RigidBodyState};
use sim_math::{Quaternion, Vec3, body_to_world};

/// Force and moment accumulated in the FRD body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyWrench {
    pub force_body_n: Vec3,
    pub moment_body_nm: Vec3,
}

impl BodyWrench {
    #[must_use]
    pub fn zero() -> Self {
        Self {
            force_body_n: Vec3::zeros(),
            moment_body_nm: Vec3::zeros(),
        }
    }
}

impl Default for BodyWrench {
    fn default() -> Self {
        Self::zero()
    }
}

/// Time derivative of the canonical rigid-body state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyDerivative {
    pub position_world_mps: Vec3,
    pub linear_velocity_world_mps2: Vec3,
    pub orientation_world_from_body_per_s: Quaternion<f64>,
    pub angular_velocity_body_radps2: Vec3,
}

/// Evaluates the 6DoF equations without choosing a numerical integration method.
#[must_use]
pub fn evaluate_derivative(
    state: &RigidBodyState,
    params: &RigidBodyParams,
    wrench: &BodyWrench,
    gravity_world_mps2: &Vec3,
) -> RigidBodyDerivative {
    debug_assert!(state.validate().is_ok());
    debug_assert!(gravity_world_mps2.iter().all(|value| value.is_finite()));
    debug_assert!(wrench.force_body_n.iter().all(|value| value.is_finite()));
    debug_assert!(wrench.moment_body_nm.iter().all(|value| value.is_finite()));

    let force_world_n = body_to_world(&state.orientation_world_from_body, &wrench.force_body_n);
    let linear_velocity_world_mps2 = force_world_n / params.mass_kg() + gravity_world_mps2;

    let angular_momentum_body = params.inertia_body_kg_m2() * state.angular_velocity_body_radps;
    let gyroscopic_moment = state
        .angular_velocity_body_radps
        .cross(&angular_momentum_body);
    let angular_velocity_body_radps2 =
        params.inverse_inertia_body_per_kg_m2() * (wrench.moment_body_nm - gyroscopic_moment);

    let orientation = state.orientation_world_from_body.quaternion();
    let omega_quaternion = Quaternion::new(
        0.0,
        state.angular_velocity_body_radps.x,
        state.angular_velocity_body_radps.y,
        state.angular_velocity_body_radps.z,
    );
    let orientation_world_from_body_per_s = (orientation * omega_quaternion) * 0.5;

    RigidBodyDerivative {
        position_world_mps: state.linear_velocity_world_mps,
        linear_velocity_world_mps2,
        orientation_world_from_body_per_s,
        angular_velocity_body_radps2,
    }
}
