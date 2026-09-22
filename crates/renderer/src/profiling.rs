//! Bounded CPU and optional GPU timestamp profiling for the V2 path.

use crate::{device::DeviceCapabilities, render_graph::PassId};
use std::{
    sync::{
        Arc,
        atomic::{AtomicU8, Ordering},
    },
    time::{Duration, Instant},
};

pub(crate) const GPU_QUERY_COUNT: u32 = (PassId::COUNT as u32) * 2;
const READBACK_RING_SIZE: usize = 3;
const SLOT_FREE: u8 = 0;
const SLOT_PENDING: u8 = 1;
const SLOT_READY: u8 = 2;
const SLOT_FAILED: u8 = 3;
const QUERY_BYTES: u64 = GPU_QUERY_COUNT as u64 * wgpu::QUERY_SIZE as u64;
const RESOLVE_BUFFER_SIZE: u64 = align_up(QUERY_BYTES, wgpu::QUERY_RESOLVE_BUFFER_ALIGNMENT);

const fn align_up(value: u64, alignment: u64) -> u64 {
    value.div_ceil(alignment) * alignment
}

#[must_use]
pub(crate) const fn resolve_buffer_usage() -> wgpu::BufferUsages {
    wgpu::BufferUsages::QUERY_RESOLVE.union(wgpu::BufferUsages::COPY_SRC)
}

#[must_use]
pub(crate) const fn readback_buffer_usage() -> wgpu::BufferUsages {
    wgpu::BufferUsages::COPY_DST.union(wgpu::BufferUsages::MAP_READ)
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)]
pub(crate) struct ProfileSnapshot {
    pub(crate) presentation_frame_index: Option<u64>,
    pub(crate) cpu_frame: Duration,
    pub(crate) cpu_passes: [Duration; PassId::COUNT],
    pub(crate) gpu_timing_supported: bool,
    pub(crate) gpu_presentation_frame_index: Option<u64>,
    pub(crate) gpu_pass_ns: [Option<f64>; PassId::COUNT],
}

impl Default for ProfileSnapshot {
    fn default() -> Self {
        Self {
            presentation_frame_index: None,
            cpu_frame: Duration::ZERO,
            cpu_passes: [Duration::ZERO; PassId::COUNT],
            gpu_timing_supported: false,
            gpu_presentation_frame_index: None,
            gpu_pass_ns: [None; PassId::COUNT],
        }
    }
}

struct ReadbackSlot {
    buffer: wgpu::Buffer,
    state: Arc<AtomicU8>,
    presentation_frame_index: Option<u64>,
}

struct GpuProfiler {
    query_set: wgpu::QuerySet,
    resolve_buffer: wgpu::Buffer,
    readback_slots: [ReadbackSlot; READBACK_RING_SIZE],
    next_slot: usize,
    active_slot: Option<usize>,
    timestamp_period_ns: f64,
    latest_presentation_frame_index: Option<u64>,
    latest_pass_ns: [Option<f64>; PassId::COUNT],
}

impl GpuProfiler {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Self {
        let query_set = device.create_query_set(&wgpu::QuerySetDescriptor {
            label: Some("RV2 GPU timestamp query set"),
            ty: wgpu::QueryType::Timestamp,
            count: GPU_QUERY_COUNT,
        });
        let resolve_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("RV2 GPU timestamp resolve buffer"),
            size: RESOLVE_BUFFER_SIZE,
            usage: resolve_buffer_usage(),
            mapped_at_creation: false,
        });
        let readback_slots = std::array::from_fn(|index| ReadbackSlot {
            buffer: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(match index {
                    0 => "RV2 GPU timestamp readback 0",
                    1 => "RV2 GPU timestamp readback 1",
                    _ => "RV2 GPU timestamp readback 2",
                }),
                size: RESOLVE_BUFFER_SIZE,
                usage: readback_buffer_usage(),
                mapped_at_creation: false,
            }),
            state: Arc::new(AtomicU8::new(SLOT_FREE)),
            presentation_frame_index: None,
        });
        Self {
            query_set,
            resolve_buffer,
            readback_slots,
            next_slot: 0,
            active_slot: None,
            timestamp_period_ns: f64::from(queue.get_timestamp_period()),
            latest_presentation_frame_index: None,
            latest_pass_ns: [None; PassId::COUNT],
        }
    }

    fn begin_frame(&mut self, device: &wgpu::Device, presentation_frame_index: Option<u64>) {
        self.poll_ready(device);
        self.active_slot = (0..READBACK_RING_SIZE)
            .map(|offset| (self.next_slot + offset) % READBACK_RING_SIZE)
            .find(|index| self.readback_slots[*index].state.load(Ordering::Acquire) == SLOT_FREE);
        if let Some(slot_index) = self.active_slot {
            self.readback_slots[slot_index].presentation_frame_index = presentation_frame_index;
        }
    }

    fn timestamp_writes(&self, pass: PassId) -> Option<wgpu::RenderPassTimestampWrites<'_>> {
        self.active_slot?;
        let first = pass.index() as u32 * 2;
        Some(wgpu::RenderPassTimestampWrites {
            query_set: &self.query_set,
            beginning_of_pass_write_index: Some(first),
            end_of_pass_write_index: Some(first + 1),
        })
    }

    fn resolve(&self, encoder: &mut wgpu::CommandEncoder) {
        let Some(slot_index) = self.active_slot else {
            return;
        };
        encoder.resolve_query_set(&self.query_set, 0..GPU_QUERY_COUNT, &self.resolve_buffer, 0);
        encoder.copy_buffer_to_buffer(
            &self.resolve_buffer,
            0,
            &self.readback_slots[slot_index].buffer,
            0,
            QUERY_BYTES,
        );
    }

    fn after_submit(&mut self) {
        let Some(slot_index) = self.active_slot.take() else {
            return;
        };
        let slot = &self.readback_slots[slot_index];
        slot.state.store(SLOT_PENDING, Ordering::Release);
        let state = Arc::clone(&slot.state);
        slot.buffer
            .slice(..QUERY_BYTES)
            .map_async(wgpu::MapMode::Read, move |result| {
                state.store(
                    if result.is_ok() {
                        SLOT_READY
                    } else {
                        SLOT_FAILED
                    },
                    Ordering::Release,
                );
            });
        self.next_slot = (slot_index + 1) % READBACK_RING_SIZE;
    }

    fn poll_ready(&mut self, device: &wgpu::Device) {
        let _ = device.poll(wgpu::PollType::Poll);
        for slot in &mut self.readback_slots {
            match slot.state.load(Ordering::Acquire) {
                SLOT_READY => {
                    if let Ok(mapped) = slot.buffer.slice(..QUERY_BYTES).get_mapped_range() {
                        let timestamps: &[u64] = bytemuck::cast_slice(&mapped);
                        for pass_index in 0..PassId::COUNT {
                            let start = timestamps[pass_index * 2];
                            let end = timestamps[pass_index * 2 + 1];
                            self.latest_pass_ns[pass_index] =
                                Some(end.saturating_sub(start) as f64 * self.timestamp_period_ns);
                        }
                        self.latest_presentation_frame_index = slot.presentation_frame_index;
                        drop(mapped);
                    }
                    slot.buffer.unmap();
                    slot.presentation_frame_index = None;
                    slot.state.store(SLOT_FREE, Ordering::Release);
                }
                SLOT_FAILED => {
                    slot.presentation_frame_index = None;
                    slot.state.store(SLOT_FREE, Ordering::Release);
                }
                _ => {}
            }
        }
    }
}

/// V2-only profiler. All storage is bounded and all GPU objects are allocated
/// during initialization.
pub(crate) struct Profiler {
    frame_started: Option<Instant>,
    presentation_frame_index: Option<u64>,
    cpu_passes: [Duration; PassId::COUNT],
    gpu: Option<GpuProfiler>,
    latest: ProfileSnapshot,
}

impl Profiler {
    pub(crate) fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        capabilities: &DeviceCapabilities,
    ) -> Self {
        let gpu = capabilities
            .timestamp_query
            .then(|| GpuProfiler::new(device, queue));
        Self {
            frame_started: None,
            presentation_frame_index: None,
            cpu_passes: [Duration::ZERO; PassId::COUNT],
            gpu,
            latest: ProfileSnapshot::default(),
        }
    }

    pub(crate) fn begin_frame(
        &mut self,
        device: &wgpu::Device,
        presentation_frame_index: Option<u64>,
    ) {
        self.cpu_passes.fill(Duration::ZERO);
        self.frame_started = Some(Instant::now());
        self.presentation_frame_index = presentation_frame_index;
        if let Some(gpu) = self.gpu.as_mut() {
            gpu.begin_frame(device, presentation_frame_index);
        }
    }

    #[must_use]
    pub(crate) fn begin_cpu_pass(&self) -> Instant {
        Instant::now()
    }

    pub(crate) fn end_cpu_pass(&mut self, pass: PassId, started: Instant) {
        self.cpu_passes[pass.index()] = started.elapsed();
    }

    pub(crate) fn timestamp_writes(
        &self,
        pass: PassId,
    ) -> Option<wgpu::RenderPassTimestampWrites<'_>> {
        self.gpu.as_ref().and_then(|gpu| gpu.timestamp_writes(pass))
    }

    pub(crate) fn finish_encoding(&self, encoder: &mut wgpu::CommandEncoder) {
        if let Some(gpu) = self.gpu.as_ref() {
            gpu.resolve(encoder);
        }
    }

    pub(crate) fn after_submit(&mut self) {
        if let Some(gpu) = self.gpu.as_mut() {
            gpu.after_submit();
        }
    }

    pub(crate) fn finish_frame(&mut self, device: &wgpu::Device) {
        // Non-blocking refresh only. Capture readback may already have waited
        // for this submission, in which case the matching timestamps can be
        // associated with this frame; normal presentation never busy-waits.
        if let Some(gpu) = self.gpu.as_mut() {
            gpu.poll_ready(device);
        }
        let cpu_frame = self
            .frame_started
            .take()
            .map_or(Duration::ZERO, |started| started.elapsed());
        self.latest = ProfileSnapshot {
            presentation_frame_index: self.presentation_frame_index.take(),
            cpu_frame,
            cpu_passes: self.cpu_passes,
            gpu_timing_supported: self.gpu.is_some(),
            gpu_presentation_frame_index: self
                .gpu
                .as_ref()
                .and_then(|gpu| gpu.latest_presentation_frame_index),
            gpu_pass_ns: self
                .gpu
                .as_ref()
                .map_or([None; PassId::COUNT], |gpu| gpu.latest_pass_ns),
        };
    }

    pub(crate) const fn latest(&self) -> &ProfileSnapshot {
        &self.latest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn production_query_count_is_two_per_real_pass() {
        assert_eq!(PassId::COUNT, 6);
        assert_eq!(GPU_QUERY_COUNT, 12);
    }

    #[test]
    fn resolve_and_readback_usages_are_separated() {
        let resolve = resolve_buffer_usage();
        assert_eq!(
            resolve,
            wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC
        );
        assert!(!resolve.contains(wgpu::BufferUsages::MAP_READ));

        let readback = readback_buffer_usage();
        assert_eq!(
            readback,
            wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ
        );
        assert!(!readback.contains(wgpu::BufferUsages::QUERY_RESOLVE));
    }

    #[test]
    fn resolve_storage_is_alignment_safe() {
        let resolve_size = std::hint::black_box(RESOLVE_BUFFER_SIZE);
        assert_eq!(resolve_size % wgpu::QUERY_RESOLVE_BUFFER_ALIGNMENT, 0);
        assert!(resolve_size >= QUERY_BYTES);
    }
}
