//! Explicit resize-dependent render-target lifecycle.
//!
//! The backing textures and their views are created at renderer startup and
//! recreated only after a valid resize. Static scene/material resources stay
//! in `gpu.rs`; this module deliberately is not a generic resource registry.

pub(crate) struct DepthTarget {
    _texture: wgpu::Texture,
    pub(crate) view: wgpu::TextureView,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DepthTargetUsage {
    V1AttachmentOnly,
    V2Sampleable,
}

impl DepthTargetUsage {
    pub(crate) const fn texture_usages(self) -> wgpu::TextureUsages {
        match self {
            Self::V1AttachmentOnly => wgpu::TextureUsages::RENDER_ATTACHMENT,
            Self::V2Sampleable => {
                wgpu::TextureUsages::RENDER_ATTACHMENT.union(wgpu::TextureUsages::TEXTURE_BINDING)
            }
        }
    }
}

pub(crate) struct HdrTarget {
    _texture: wgpu::Texture,
    pub(crate) view: wgpu::TextureView,
}

pub(crate) struct TemporalHistoryTargets {
    slots: [HdrTarget; 2],
}

pub(crate) const fn temporal_history_usages() -> wgpu::TextureUsages {
    wgpu::TextureUsages::RENDER_ATTACHMENT.union(wgpu::TextureUsages::TEXTURE_BINDING)
}

impl TemporalHistoryTargets {
    pub(crate) fn view(&self, index: usize) -> &wgpu::TextureView {
        &self.slots[index].view
    }
}

pub(crate) fn create_depth_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
    usage: DepthTargetUsage,
) -> DepthTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G1C scene depth"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: usage.texture_usages(),
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    DepthTarget {
        _texture: texture,
        view,
    }
}

pub(crate) fn create_temporal_history_targets(
    device: &wgpu::Device,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
) -> TemporalHistoryTargets {
    TemporalHistoryTargets {
        slots: std::array::from_fn(|index| {
            let texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some(match index {
                    0 => "RV2 temporal history A",
                    _ => "RV2 temporal history B",
                }),
                size: wgpu::Extent3d {
                    width,
                    height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: temporal_history_usages(),
                view_formats: &[],
            });
            let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
            HdrTarget {
                _texture: texture,
                view,
            }
        }),
    }
}

pub(crate) fn create_hdr_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
    format: wgpu::TextureFormat,
) -> HdrTarget {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("G3B linear HDR scene target"),
        size: wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    HdrTarget {
        _texture: texture,
        view,
    }
}

pub(crate) fn create_hdr_scene_bind_group(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    hdr_view: &wgpu::TextureView,
    sampler: &wgpu::Sampler,
    uniform_buffer: &wgpu::Buffer,
    label: &str,
) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(hdr_view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(sampler),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: uniform_buffer.as_entire_binding(),
            },
        ],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v1_depth_remains_attachment_only_while_v2_depth_is_sampleable() {
        assert_eq!(
            DepthTargetUsage::V1AttachmentOnly.texture_usages(),
            wgpu::TextureUsages::RENDER_ATTACHMENT
        );
        assert_eq!(
            DepthTargetUsage::V2Sampleable.texture_usages(),
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING
        );
    }

    #[test]
    fn temporal_history_is_renderable_and_sampleable_only() {
        assert_eq!(
            temporal_history_usages(),
            wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING
        );
    }
}
