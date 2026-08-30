use sim_core::{RigidBodyState, SimSnapshot, SnapshotBuffer};
use sim_math::Vec3;

fn snapshot(step_index: u64) -> SimSnapshot {
    SimSnapshot::from_state(
        step_index,
        0.002,
        &RigidBodyState::stationary(Vec3::new(step_index as f64, 0.0, 0.0)),
    )
}

#[test]
fn t12_snapshot_ring_wraps_and_preserves_order() {
    let mut buffer = SnapshotBuffer::new(3).unwrap();
    for index in 1..=5 {
        buffer.push(snapshot(index));
    }
    assert_eq!(buffer.len(), 3);
    assert_eq!(buffer.capacity(), 3);
    assert_eq!(buffer.latest().unwrap().step_index, 5);
    let indices: Vec<_> = buffer
        .oldest_first()
        .map(|value| value.step_index)
        .collect();
    assert_eq!(indices, vec![3, 4, 5]);
}
