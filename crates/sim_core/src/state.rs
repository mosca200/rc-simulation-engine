use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};
use sim_math::{Orientation, Quaternion, Vec3};
use thiserror::Error;

/// Single source of truth for the rigid body's 6DoF state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RigidBodyState {
    /// Position in the North-East-Down world frame, metres.
    pub position_world_m: Vec3,
    /// Linear velocity in the NED world frame, metres per second.
    pub linear_velocity_world_mps: Vec3,
    /// Active Hamilton rotation from FRD body coordinates into NED world coordinates.
    pub orientation_world_from_body: Orientation,
    /// Angular velocity in the FRD body frame, radians per second.
    pub angular_velocity_body_radps: Vec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum StateError {
    #[error("rigid-body state contains a non-finite scalar")]
    NonFinite,
    #[error("orientation quaternion is invalid")]
    InvalidOrientation,
}

impl RigidBodyState {
    #[must_use]
    pub fn stationary(position_world_m: Vec3) -> Self {
        Self {
            position_world_m,
            linear_velocity_world_mps: Vec3::zeros(),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        }
    }

    pub fn validate(&self) -> Result<(), StateError> {
        let vectors_are_finite = self.position_world_m.iter().all(|value| value.is_finite())
            && self
                .linear_velocity_world_mps
                .iter()
                .all(|value| value.is_finite())
            && self
                .angular_velocity_body_radps
                .iter()
                .all(|value| value.is_finite());
        if !vectors_are_finite {
            return Err(StateError::NonFinite);
        }

        let quaternion = self.orientation_world_from_body.quaternion();
        let quaternion_is_finite = [quaternion.w, quaternion.i, quaternion.j, quaternion.k]
            .into_iter()
            .all(f64::is_finite);
        let norm_squared = quaternion.norm_squared();
        if !quaternion_is_finite
            || norm_squared <= f64::EPSILON
            || (norm_squared - 1.0).abs() > 1.0e-12
        {
            return Err(StateError::InvalidOrientation);
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize)]
struct SerializableRigidBodyState {
    position_world_m_bits: [u64; 3],
    linear_velocity_world_mps_bits: [u64; 3],
    /// Explicit canonical Hamilton component order with exact IEEE-754 payloads.
    orientation_world_from_body_wxyz_bits: [u64; 4],
    angular_velocity_body_radps_bits: [u64; 3],
}

impl Serialize for RigidBodyState {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let quaternion = self.orientation_world_from_body.quaternion();
        SerializableRigidBodyState {
            position_world_m_bits: vector_to_bits(&self.position_world_m),
            linear_velocity_world_mps_bits: vector_to_bits(&self.linear_velocity_world_mps),
            orientation_world_from_body_wxyz_bits: [
                quaternion.w.to_bits(),
                quaternion.i.to_bits(),
                quaternion.j.to_bits(),
                quaternion.k.to_bits(),
            ],
            angular_velocity_body_radps_bits: vector_to_bits(&self.angular_velocity_body_radps),
        }
        .serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for RigidBodyState {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let serialized = SerializableRigidBodyState::deserialize(deserializer)?;
        let [w, x, y, z] = serialized
            .orientation_world_from_body_wxyz_bits
            .map(f64::from_bits);
        let state = Self {
            position_world_m: vector_from_bits(serialized.position_world_m_bits),
            linear_velocity_world_mps: vector_from_bits(serialized.linear_velocity_world_mps_bits),
            orientation_world_from_body: Orientation::new_unchecked(Quaternion::new(w, x, y, z)),
            angular_velocity_body_radps: vector_from_bits(
                serialized.angular_velocity_body_radps_bits,
            ),
        };
        state.validate().map_err(D::Error::custom)?;
        Ok(state)
    }
}

fn vector_to_bits(vector: &Vec3) -> [u64; 3] {
    [vector.x.to_bits(), vector.y.to_bits(), vector.z.to_bits()]
}

fn vector_from_bits(bits: [u64; 3]) -> Vec3 {
    Vec3::new(
        f64::from_bits(bits[0]),
        f64::from_bits(bits[1]),
        f64::from_bits(bits[2]),
    )
}
