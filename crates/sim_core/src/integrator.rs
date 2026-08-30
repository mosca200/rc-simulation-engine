use crate::{
    BodyWrench, RigidBodyDerivative, RigidBodyParams, RigidBodyState, evaluate_derivative,
};
use sim_math::{Orientation, Quaternion, Vec3};

/// Fourth-order Runge-Kutta integrator specialized for the canonical rigid-body state.
#[derive(Debug, Default, Clone, Copy)]
pub struct Rk4Integrator;

impl Rk4Integrator {
    /// Advances one step while re-evaluating the derivative from each RK4 stage state.
    #[must_use]
    pub fn step<F>(state: &RigidBodyState, dt_s: f64, mut evaluate_stage: F) -> RigidBodyState
    where
        F: FnMut(&RigidBodyState) -> RigidBodyDerivative,
    {
        debug_assert!(dt_s.is_finite() && dt_s > 0.0);

        let k1 = evaluate_stage(state);
        let stage2 = offset_state(state, &k1, 0.5 * dt_s);
        let k2 = evaluate_stage(&stage2);
        let stage3 = offset_state(state, &k2, 0.5 * dt_s);
        let k3 = evaluate_stage(&stage3);
        let stage4 = offset_state(state, &k3, dt_s);
        let k4 = evaluate_stage(&stage4);

        weighted_update(state, [&k1, &k2, &k3, &k4], dt_s)
    }

    /// Convenience path for the current rigid body under a wrench constant over one step.
    #[must_use]
    pub fn step_with_constant_wrench(
        state: &RigidBodyState,
        params: &RigidBodyParams,
        wrench: &BodyWrench,
        gravity_world_mps2: &Vec3,
        dt_s: f64,
    ) -> RigidBodyState {
        Self::step(state, dt_s, |stage_state| {
            evaluate_derivative(stage_state, params, wrench, gravity_world_mps2)
        })
    }
}

fn offset_state(
    base: &RigidBodyState,
    derivative: &RigidBodyDerivative,
    scale_s: f64,
) -> RigidBodyState {
    let base_orientation = base.orientation_world_from_body.quaternion();
    let raw_orientation = base_orientation + derivative.orientation_world_from_body_per_s * scale_s;

    RigidBodyState {
        position_world_m: base.position_world_m + derivative.position_world_mps * scale_s,
        linear_velocity_world_mps: base.linear_velocity_world_mps
            + derivative.linear_velocity_world_mps2 * scale_s,
        orientation_world_from_body: Orientation::new_normalize(raw_orientation),
        angular_velocity_body_radps: base.angular_velocity_body_radps
            + derivative.angular_velocity_body_radps2 * scale_s,
    }
}

fn weighted_update(
    base: &RigidBodyState,
    derivatives: [&RigidBodyDerivative; 4],
    dt_s: f64,
) -> RigidBodyState {
    let [k1, k2, k3, k4] = derivatives;
    let sixth_dt = dt_s / 6.0;
    let position_delta = (k1.position_world_mps
        + 2.0 * k2.position_world_mps
        + 2.0 * k3.position_world_mps
        + k4.position_world_mps)
        * sixth_dt;
    let velocity_delta = (k1.linear_velocity_world_mps2
        + 2.0 * k2.linear_velocity_world_mps2
        + 2.0 * k3.linear_velocity_world_mps2
        + k4.linear_velocity_world_mps2)
        * sixth_dt;
    let angular_velocity_delta = (k1.angular_velocity_body_radps2
        + 2.0 * k2.angular_velocity_body_radps2
        + 2.0 * k3.angular_velocity_body_radps2
        + k4.angular_velocity_body_radps2)
        * sixth_dt;
    let quaternion_delta: Quaternion<f64> = (k1.orientation_world_from_body_per_s
        + k2.orientation_world_from_body_per_s * 2.0
        + k3.orientation_world_from_body_per_s * 2.0
        + k4.orientation_world_from_body_per_s)
        * sixth_dt;
    let orientation = Orientation::new_normalize(
        base.orientation_world_from_body.quaternion() + quaternion_delta,
    );

    RigidBodyState {
        position_world_m: base.position_world_m + position_delta,
        linear_velocity_world_mps: base.linear_velocity_world_mps + velocity_delta,
        orientation_world_from_body: orientation,
        angular_velocity_body_radps: base.angular_velocity_body_radps + angular_velocity_delta,
    }
}
