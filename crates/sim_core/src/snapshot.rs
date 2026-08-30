use crate::RigidBodyState;
use serde::{Deserialize, Serialize};
use sim_math::{Orientation, Vec3};
use thiserror::Error;

/// Owned immutable export of post-step simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SimSnapshot {
    pub step_index: u64,
    pub sim_time_s: f64,
    pub position_world_m: Vec3,
    pub linear_velocity_world_mps: Vec3,
    pub orientation_world_from_body: Orientation,
    pub angular_velocity_body_radps: Vec3,
}

impl SimSnapshot {
    #[must_use]
    pub fn from_state(step_index: u64, dt_s: f64, state: &RigidBodyState) -> Self {
        Self {
            step_index,
            sim_time_s: step_index as f64 * dt_s,
            position_world_m: state.position_world_m,
            linear_velocity_world_mps: state.linear_velocity_world_mps,
            orientation_world_from_body: state.orientation_world_from_body,
            angular_velocity_body_radps: state.angular_velocity_body_radps,
        }
    }

    /// Stable canonical hash for same-build/same-target deterministic replay checks.
    #[must_use]
    pub fn state_hash(&self) -> blake3::Hash {
        let mut hasher = blake3::Hasher::new();
        hasher.update(&self.step_index.to_le_bytes());
        update_vector(&mut hasher, &self.position_world_m);
        update_vector(&mut hasher, &self.linear_velocity_world_mps);
        let quaternion = self.orientation_world_from_body.quaternion();
        for value in [quaternion.w, quaternion.i, quaternion.j, quaternion.k] {
            hasher.update(&value.to_bits().to_le_bytes());
        }
        update_vector(&mut hasher, &self.angular_velocity_body_radps);
        hasher.finalize()
    }
}

fn update_vector(hasher: &mut blake3::Hasher, vector: &Vec3) {
    for value in vector.iter() {
        hasher.update(&value.to_bits().to_le_bytes());
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum SnapshotBufferError {
    #[error("snapshot buffer capacity must be greater than zero")]
    ZeroCapacity,
}

/// Fixed-capacity, preallocated snapshot ring with deterministic overwrite semantics.
#[derive(Debug, Clone)]
pub struct SnapshotBuffer {
    slots: Vec<Option<SimSnapshot>>,
    next_write: usize,
    len: usize,
}

impl SnapshotBuffer {
    pub fn new(capacity: usize) -> Result<Self, SnapshotBufferError> {
        if capacity == 0 {
            return Err(SnapshotBufferError::ZeroCapacity);
        }
        Ok(Self {
            slots: vec![None; capacity],
            next_write: 0,
            len: 0,
        })
    }

    pub fn push(&mut self, snapshot: SimSnapshot) {
        self.slots[self.next_write] = Some(snapshot);
        self.next_write = (self.next_write + 1) % self.slots.len();
        self.len = (self.len + 1).min(self.slots.len());
    }

    #[must_use]
    pub fn latest(&self) -> Option<&SimSnapshot> {
        if self.len == 0 {
            None
        } else {
            let index = (self.next_write + self.slots.len() - 1) % self.slots.len();
            self.slots[index].as_ref()
        }
    }

    pub fn oldest_first(&self) -> impl ExactSizeIterator<Item = &SimSnapshot> {
        let capacity = self.slots.len();
        let oldest = if self.len == capacity {
            self.next_write
        } else {
            0
        };
        (0..self.len).map(move |offset| {
            let index = (oldest + offset) % capacity;
            self.slots[index]
                .as_ref()
                .expect("occupied range invariant")
        })
    }

    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }
}
