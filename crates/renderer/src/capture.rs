//! Deterministic final display-frame capture types and CPU row handling.

use thiserror::Error;

pub(crate) const CAPTURE_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
pub(crate) const CAPTURE_USAGES: wgpu::TextureUsages = wgpu::TextureUsages::RENDER_ATTACHMENT
    .union(wgpu::TextureUsages::TEXTURE_BINDING)
    .union(wgpu::TextureUsages::COPY_SRC);
const RGBA8_BYTES_PER_PIXEL: u32 = 4;

/// CPU-owned pixels captured from the final display-referred render target.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CapturedFrame {
    pub width: u32,
    pub height: u32,
    pub rgba8: Vec<u8>,
}

/// Result of an explicit render-and-capture call.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CaptureRenderOutcome {
    /// The frame was rendered, captured, read back, and presented.
    CapturedAndPresented(CapturedFrame),
    /// Rendering and capture were skipped because the surface had zero extent.
    SkippedZeroExtent,
}

/// Fail-closed errors from the renderer-owned capture path.
#[derive(Debug, Error)]
pub enum FrameCaptureError {
    #[error("the presentation surface failed before capture: {0}")]
    Surface(#[from] crate::SurfaceError),
    #[error("capture requires an sRGB presentation surface")]
    UnsupportedSurfaceColorEncoding,
    #[error("the required RGBA8 sRGB capture target usages are unsupported")]
    UnsupportedCaptureTarget,
    #[error("capture dimensions must both be non-zero")]
    InvalidDimensions,
    #[error("capture row or buffer size arithmetic overflowed")]
    LayoutOverflow,
    #[error("the GPU readback buffer could not be mapped")]
    MapFailed,
    #[error("GPU polling failed while waiting for capture readback")]
    DevicePollFailed,
    #[error("GPU capture readback exceeded the bounded wait")]
    DevicePollTimeout,
    #[error("the GPU mapping callback did not complete after submission")]
    MapCallbackMissing,
    #[error("the mapped GPU readback range was unavailable")]
    MappedRangeUnavailable,
    #[error("the capture render path completed without producing pixels")]
    CaptureNotProduced,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct CaptureRowLayout {
    pub(crate) unpadded_bytes_per_row: u32,
    pub(crate) padded_bytes_per_row: u32,
    pub(crate) staging_size: u64,
    pub(crate) pixel_bytes: usize,
}

pub(crate) fn capture_row_layout(
    width: u32,
    height: u32,
) -> Result<CaptureRowLayout, FrameCaptureError> {
    if width == 0 || height == 0 {
        return Err(FrameCaptureError::InvalidDimensions);
    }
    let unpadded_bytes_per_row = width
        .checked_mul(RGBA8_BYTES_PER_PIXEL)
        .ok_or(FrameCaptureError::LayoutOverflow)?;
    let alignment = wgpu::COPY_BYTES_PER_ROW_ALIGNMENT;
    let padded_bytes_per_row = unpadded_bytes_per_row
        .checked_add(alignment - 1)
        .ok_or(FrameCaptureError::LayoutOverflow)?
        / alignment
        * alignment;
    let staging_size = u64::from(padded_bytes_per_row)
        .checked_mul(u64::from(height))
        .ok_or(FrameCaptureError::LayoutOverflow)?;
    let pixel_bytes_u64 = u64::from(unpadded_bytes_per_row)
        .checked_mul(u64::from(height))
        .ok_or(FrameCaptureError::LayoutOverflow)?;
    let pixel_bytes =
        usize::try_from(pixel_bytes_u64).map_err(|_| FrameCaptureError::LayoutOverflow)?;
    Ok(CaptureRowLayout {
        unpadded_bytes_per_row,
        padded_bytes_per_row,
        staging_size,
        pixel_bytes,
    })
}

pub(crate) fn unpad_capture_rows(
    mapped: &[u8],
    height: u32,
    layout: CaptureRowLayout,
) -> Result<Vec<u8>, FrameCaptureError> {
    let required =
        usize::try_from(layout.staging_size).map_err(|_| FrameCaptureError::LayoutOverflow)?;
    if mapped.len() < required {
        return Err(FrameCaptureError::MappedRangeUnavailable);
    }
    let padded = usize::try_from(layout.padded_bytes_per_row)
        .map_err(|_| FrameCaptureError::LayoutOverflow)?;
    let unpadded = usize::try_from(layout.unpadded_bytes_per_row)
        .map_err(|_| FrameCaptureError::LayoutOverflow)?;
    let mut pixels = Vec::with_capacity(layout.pixel_bytes);
    for row in 0..usize::try_from(height).map_err(|_| FrameCaptureError::LayoutOverflow)? {
        let start = row
            .checked_mul(padded)
            .ok_or(FrameCaptureError::LayoutOverflow)?;
        let end = start
            .checked_add(unpadded)
            .ok_or(FrameCaptureError::LayoutOverflow)?;
        pixels.extend_from_slice(
            mapped
                .get(start..end)
                .ok_or(FrameCaptureError::MappedRangeUnavailable)?,
        );
    }
    if pixels.len() != layout.pixel_bytes {
        return Err(FrameCaptureError::MappedRangeUnavailable);
    }
    Ok(pixels)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn row_layout_preserves_already_aligned_rows() {
        let layout = capture_row_layout(64, 3).unwrap();
        assert_eq!(layout.unpadded_bytes_per_row, 256);
        assert_eq!(layout.padded_bytes_per_row, 256);
        assert_eq!(layout.staging_size, 768);
    }

    #[test]
    fn row_layout_aligns_non_aligned_rows_to_256_bytes() {
        let layout = capture_row_layout(65, 2).unwrap();
        assert_eq!(layout.unpadded_bytes_per_row, 260);
        assert_eq!(layout.padded_bytes_per_row, 512);
        assert_eq!(layout.staging_size, 1_024);
    }

    #[test]
    fn row_unpadding_removes_every_padding_byte() {
        let layout = capture_row_layout(2, 3).unwrap();
        let mut mapped = vec![0xEE; usize::try_from(layout.staging_size).unwrap()];
        for row in 0..3_usize {
            let start = row * usize::try_from(layout.padded_bytes_per_row).unwrap();
            mapped[start..start + 8].fill(u8::try_from(row + 1).unwrap());
        }
        let pixels = unpad_capture_rows(&mapped, 3, layout).unwrap();
        assert_eq!(pixels.len(), 2 * 3 * 4);
        assert_eq!(&pixels[0..8], &[1; 8]);
        assert_eq!(&pixels[8..16], &[2; 8]);
        assert_eq!(&pixels[16..24], &[3; 8]);
        assert!(!pixels.contains(&0xEE));
    }

    #[test]
    fn row_layout_rejects_zero_and_overflowing_dimensions() {
        assert!(matches!(
            capture_row_layout(0, 1),
            Err(FrameCaptureError::InvalidDimensions)
        ));
        assert!(matches!(
            capture_row_layout(1, 0),
            Err(FrameCaptureError::InvalidDimensions)
        ));
        assert!(matches!(
            capture_row_layout(u32::MAX, 1),
            Err(FrameCaptureError::LayoutOverflow)
        ));
    }

    #[test]
    fn normal_render_dispatch_does_not_request_capture_resources() {
        let source = include_str!("gpu.rs");
        let normal_render = source
            .split_once("pub fn render(&mut self, frame: &RenderFrame)")
            .unwrap()
            .1
            .split_once("pub fn render_and_capture(")
            .unwrap()
            .0;
        assert!(normal_render.contains("render_scheduled(frame, None, None, None, None, false)"));
        assert!(!normal_render.contains("create_capture_frame_resources"));

        let scheduled_render = source
            .split_once("fn render_scheduled(")
            .unwrap()
            .1
            .split_once("fn check_asynchronous_gpu_error")
            .unwrap()
            .0;
        assert!(scheduled_render.contains("let capture_resources = capture_requested"));
        assert!(scheduled_render.contains(".then(||"));
    }

    #[test]
    #[ignore = "requires a GPU; run with `cargo test -p renderer --lib -- --ignored`"]
    fn headless_rgba8_srgb_render_copy_map_and_unpad() {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            apply_limit_buckets: false,
        }))
        .expect("no wgpu adapter available on this machine");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("VIS0-C2A headless capture test device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("request_device failed");

        let width = 65;
        let height = 2;
        let layout = capture_row_layout(width, height).unwrap();
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("VIS0-C2A headless final LDR target"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: CAPTURE_FORMAT,
            usage: CAPTURE_USAGES,
            view_formats: &[],
        });
        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("VIS0-C2A headless padded readback"),
            size: layout.staging_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("VIS0-C2A known red pattern"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 1.0,
                            g: 0.0,
                            b: 0.0,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
        }
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(layout.padded_bytes_per_row),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        let submission = queue.submit([encoder.finish()]);
        let slice = buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        device
            .poll(wgpu::PollType::Wait {
                submission_index: Some(submission),
                timeout: Some(std::time::Duration::from_secs(30)),
            })
            .expect("bounded GPU poll failed");
        receiver
            .try_recv()
            .expect("map callback missing")
            .expect("map failed");
        let mapped = slice.get_mapped_range().expect("mapped range unavailable");
        let rgba8 = unpad_capture_rows(&mapped, height, layout).unwrap();
        drop(mapped);
        buffer.unmap();

        assert_eq!(rgba8.len(), width as usize * height as usize * 4);
        let (pixels, remainder) = rgba8.as_chunks::<4>();
        assert!(remainder.is_empty());
        assert!(pixels.iter().all(|pixel| *pixel == [255, 0, 0, 255]));
    }
}
