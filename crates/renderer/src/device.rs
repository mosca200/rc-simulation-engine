//! Shared wgpu device/surface ownership and capability negotiation.
//!
//! V1 and V2 share the same surface lifecycle, while the feature policy stays
//! explicit: V1 requests no optional features and V2 may request timestamp
//! queries when the selected adapter advertises them.

use crate::{RendererError, SurfaceError};
use std::sync::{
    Arc,
    atomic::{AtomicU8, Ordering},
};
use winit::window::Window;

const GPU_ERROR_NONE: u8 = 0;
const GPU_ERROR_OUT_OF_MEMORY: u8 = 1;
const GPU_ERROR_OTHER: u8 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum DeviceFeaturePolicy {
    V1Legacy,
    V2OptionalTimestamp,
}

#[must_use]
pub(crate) fn required_features_for_policy(
    policy: DeviceFeaturePolicy,
    adapter_features: wgpu::Features,
) -> wgpu::Features {
    match policy {
        DeviceFeaturePolicy::V1Legacy => wgpu::Features::empty(),
        DeviceFeaturePolicy::V2OptionalTimestamp
            if adapter_features.contains(wgpu::Features::TIMESTAMP_QUERY) =>
        {
            wgpu::Features::TIMESTAMP_QUERY
        }
        DeviceFeaturePolicy::V2OptionalTimestamp => wgpu::Features::empty(),
    }
}

/// Immutable adapter/device capability snapshot captured during initialization.
#[derive(Debug, Clone)]
pub(crate) struct DeviceCapabilities {
    pub(crate) adapter_info: wgpu::AdapterInfo,
    pub(crate) adapter_features: wgpu::Features,
    pub(crate) adapter_limits: wgpu::Limits,
    pub(crate) device_features: wgpu::Features,
    pub(crate) device_limits: wgpu::Limits,
    pub(crate) downlevel: wgpu::DownlevelCapabilities,
    pub(crate) timestamp_query: bool,
    pub(crate) indirect_execution: bool,
    pub(crate) indirect_first_instance: bool,
    pub(crate) multi_draw_indirect_count: bool,
    pub(crate) anisotropic_filtering: bool,
    /// Rgba16Float capability snapshot (adapter diagnostics + device-legal
    /// verdict). See [`PhysicalEnvironmentCapability`].
    pub(crate) physical_environment: PhysicalEnvironmentCapability,
}

/// Rgba16Float capability snapshot and the production decision derived from it.
///
/// The adapter-reported values are diagnostics only: with `wgpu = 30.0.1`,
/// `adapter.get_texture_format_features` may advertise adapter-specific usages
/// that were never requested as a device feature. Production therefore decides
/// on the **device-legal** guaranteed features
/// (`TextureFormat::guaranteed_format_features(device.features())`), which cover
/// exactly what the created device may actually do without
/// `TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PhysicalEnvironmentCapability {
    /// Adapter-reported Rgba16Float allowed usages (diagnostic snapshot).
    pub(crate) adapter_allowed_usages: wgpu::TextureUsages,
    /// Adapter-reported Rgba16Float filterability (diagnostic snapshot).
    pub(crate) adapter_filterable: bool,
    /// Adapter-reported verdict; never used alone to enable the production path.
    pub(crate) adapter_reported: bool,
    /// Device-legal verdict; the one used for the production decision.
    pub(crate) device_legal: bool,
}

/// Usages the physical atmosphere/IBL resources need from Rgba16Float.
#[must_use]
pub(crate) fn physical_environment_required_usages() -> wgpu::TextureUsages {
    wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::RENDER_ATTACHMENT
}

/// Adapter-reported verdict for Rgba16Float (diagnostics only).
#[must_use]
pub(crate) fn physical_environment_adapter_reported(features: wgpu::TextureFormatFeatures) -> bool {
    features
        .allowed_usages
        .contains(physical_environment_required_usages())
        && features
            .flags
            .contains(wgpu::TextureFormatFeatureFlags::FILTERABLE)
}

/// Device-legal verdict for Rgba16Float.
///
/// Only capabilities guaranteed to the *created device* count, so an adapter
/// that would need `TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES` can never switch
/// the production path on.
#[must_use]
pub(crate) fn physical_environment_device_legal(device_features: wgpu::Features) -> bool {
    let features = wgpu::TextureFormat::Rgba16Float.guaranteed_format_features(device_features);
    features
        .allowed_usages
        .contains(physical_environment_required_usages())
        && features
            .flags
            .contains(wgpu::TextureFormatFeatureFlags::FILTERABLE)
}

impl DeviceCapabilities {
    fn capture(
        adapter: &wgpu::Adapter,
        device: &wgpu::Device,
        adapter_features: wgpu::Features,
        adapter_limits: wgpu::Limits,
        downlevel: wgpu::DownlevelCapabilities,
        rgba16float_features: wgpu::TextureFormatFeatures,
    ) -> Self {
        let device_features = device.features();
        let downlevel_flags = downlevel.flags;
        Self {
            adapter_info: adapter.get_info(),
            adapter_features,
            adapter_limits,
            device_features,
            device_limits: device.limits(),
            downlevel,
            timestamp_query: device_features.contains(wgpu::Features::TIMESTAMP_QUERY),
            indirect_execution: downlevel_flags.contains(wgpu::DownlevelFlags::INDIRECT_EXECUTION),
            indirect_first_instance: adapter_features
                .contains(wgpu::Features::INDIRECT_FIRST_INSTANCE),
            multi_draw_indirect_count: adapter_features
                .contains(wgpu::Features::MULTI_DRAW_INDIRECT_COUNT),
            anisotropic_filtering: downlevel_flags
                .contains(wgpu::DownlevelFlags::ANISOTROPIC_FILTERING),
            physical_environment: PhysicalEnvironmentCapability {
                adapter_allowed_usages: rgba16float_features.allowed_usages,
                adapter_filterable: rgba16float_features
                    .flags
                    .contains(wgpu::TextureFormatFeatureFlags::FILTERABLE),
                adapter_reported: physical_environment_adapter_reported(rgba16float_features),
                device_legal: physical_environment_device_legal(device_features),
            },
        }
    }

    pub(crate) fn log_v2_snapshot(&self) {
        tracing::info!(
            adapter = %self.adapter_info.name,
            backend = ?self.adapter_info.backend,
            adapter_features = ?self.adapter_features,
            adapter_limits = ?self.adapter_limits,
            device_features = ?self.device_features,
            device_limits = ?self.device_limits,
            downlevel = ?self.downlevel,
            timestamp_query = self.timestamp_query,
            indirect_execution = self.indirect_execution,
            indirect_first_instance = self.indirect_first_instance,
            multi_draw_indirect_count = self.multi_draw_indirect_count,
            anisotropic_filtering = self.anisotropic_filtering,
            physical_environment_adapter_usages = ?self.physical_environment.adapter_allowed_usages,
            physical_environment_adapter_filterable = self.physical_environment.adapter_filterable,
            physical_environment_adapter_reported = self.physical_environment.adapter_reported,
            physical_environment_device_legal = self.physical_environment.device_legal,
            "RV2 device capability snapshot"
        );
    }
}

/// Owns the wgpu objects whose lifetimes and error handling are tied to the
/// presentation surface.
pub(crate) struct DeviceContext {
    _instance: wgpu::Instance,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface_configuration: wgpu::SurfaceConfiguration,
    surface_is_configured: bool,
    capabilities: DeviceCapabilities,
    asynchronous_gpu_error: Arc<AtomicU8>,
}

impl DeviceContext {
    pub(crate) async fn new(
        window: Arc<Window>,
        feature_policy: DeviceFeaturePolicy,
    ) -> Result<Self, RendererError> {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let surface = instance
            .create_surface(window)
            .map_err(RendererError::CreateSurface)?;
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: Some(&surface),
                apply_limit_buckets: false,
            })
            .await
            .map_err(|error| RendererError::AdapterNotFound(error.to_string()))?;

        let adapter_features = adapter.features();
        let adapter_limits = adapter.limits();
        let downlevel = adapter.get_downlevel_capabilities();
        // RV2-5 capability probe is intentionally read-only and does not
        // request adapter-specific features or storage textures.
        let rgba16float_features =
            adapter.get_texture_format_features(wgpu::TextureFormat::Rgba16Float);
        let required_features = required_features_for_policy(feature_policy, adapter_features);
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("G1C/RV2 device"),
                required_features,
                required_limits: wgpu::Limits {
                    max_bind_groups: adapter_limits.max_bind_groups,
                    ..wgpu::Limits::default()
                },
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
                memory_hints: wgpu::MemoryHints::Performance,
                trace: wgpu::Trace::Off,
            })
            .await
            .map_err(|error| RendererError::RequestDevice(error.to_string()))?;

        let asynchronous_gpu_error = Arc::new(AtomicU8::new(GPU_ERROR_NONE));
        let callback_error = Arc::clone(&asynchronous_gpu_error);
        device.on_uncaptured_error(Arc::new(move |error| {
            let code = match error {
                wgpu::Error::OutOfMemory { .. } => GPU_ERROR_OUT_OF_MEMORY,
                wgpu::Error::Validation { .. } | wgpu::Error::Internal { .. } => GPU_ERROR_OTHER,
            };
            eprintln!("GpuValidation diagnostic: {error}");
            callback_error.store(code, Ordering::Release);
        }));

        let surface_capabilities = surface.get_capabilities(&adapter);
        let fallback_format = surface_capabilities
            .formats
            .first()
            .copied()
            .ok_or(RendererError::SurfaceWithoutFormats)?;
        let format = surface_capabilities
            .formats
            .iter()
            .copied()
            .find(wgpu::TextureFormat::is_srgb)
            .unwrap_or(fallback_format);
        let alpha_mode = surface_capabilities
            .alpha_modes
            .first()
            .copied()
            .ok_or(RendererError::SurfaceWithoutAlphaModes)?;
        let surface_configuration = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            color_space: wgpu::SurfaceColorSpace::Auto,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::AutoVsync,
            desired_maximum_frame_latency: 2,
            alpha_mode,
            view_formats: Vec::new(),
        };
        let surface_is_configured = size.width > 0 && size.height > 0;
        if surface_is_configured {
            surface.configure(&device, &surface_configuration);
        }

        let capabilities = DeviceCapabilities::capture(
            &adapter,
            &device,
            adapter_features,
            adapter_limits,
            downlevel,
            rgba16float_features,
        );

        Ok(Self {
            _instance: instance,
            surface,
            device,
            queue,
            surface_configuration,
            surface_is_configured,
            capabilities,
            asynchronous_gpu_error,
        })
    }

    pub(crate) const fn device(&self) -> &wgpu::Device {
        &self.device
    }

    pub(crate) const fn queue(&self) -> &wgpu::Queue {
        &self.queue
    }

    pub(crate) const fn capabilities(&self) -> &DeviceCapabilities {
        &self.capabilities
    }

    pub(crate) const fn surface_format(&self) -> wgpu::TextureFormat {
        self.surface_configuration.format
    }

    pub(crate) const fn surface_width(&self) -> u32 {
        self.surface_configuration.width
    }

    pub(crate) const fn surface_height(&self) -> u32 {
        self.surface_configuration.height
    }

    pub(crate) const fn is_surface_configured(&self) -> bool {
        self.surface_is_configured
    }

    /// Update the surface size and configure it. Returns `false` for a zero
    /// extent so callers can preserve their existing render targets.
    pub(crate) fn resize_surface(&mut self, width: u32, height: u32) -> bool {
        if width == 0 || height == 0 {
            self.surface_is_configured = false;
            return false;
        }
        self.surface_configuration.width = width;
        self.surface_configuration.height = height;
        self.reconfigure_surface();
        true
    }

    pub(crate) fn reconfigure_surface(&mut self) {
        if self.surface_configuration.width > 0 && self.surface_configuration.height > 0 {
            self.surface
                .configure(&self.device, &self.surface_configuration);
            self.surface_is_configured = true;
        }
    }

    pub(crate) fn acquire_surface_texture(
        &self,
    ) -> Result<(wgpu::SurfaceTexture, bool), SurfaceError> {
        match self.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(texture) => Ok((texture, false)),
            wgpu::CurrentSurfaceTexture::Suboptimal(texture) => Ok((texture, true)),
            wgpu::CurrentSurfaceTexture::Timeout => Err(SurfaceError::Timeout),
            wgpu::CurrentSurfaceTexture::Occluded => Err(SurfaceError::Occluded),
            wgpu::CurrentSurfaceTexture::Outdated => Err(SurfaceError::Outdated),
            wgpu::CurrentSurfaceTexture::Lost => Err(SurfaceError::Lost),
            wgpu::CurrentSurfaceTexture::Validation => Err(SurfaceError::Validation),
        }
    }

    pub(crate) fn present(&self, surface_texture: wgpu::SurfaceTexture) {
        self.queue.present(surface_texture);
    }

    pub(crate) fn check_asynchronous_gpu_error(&self) -> Result<(), SurfaceError> {
        match self.asynchronous_gpu_error.load(Ordering::Acquire) {
            GPU_ERROR_NONE => Ok(()),
            GPU_ERROR_OUT_OF_MEMORY => Err(SurfaceError::OutOfMemory),
            _ => Err(SurfaceError::Validation),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn v1_feature_policy_is_always_empty() {
        assert_eq!(
            required_features_for_policy(DeviceFeaturePolicy::V1Legacy, wgpu::Features::all(),),
            wgpu::Features::empty()
        );
    }

    #[test]
    fn v2_requests_timestamp_only_when_advertised() {
        assert_eq!(
            required_features_for_policy(
                DeviceFeaturePolicy::V2OptionalTimestamp,
                wgpu::Features::empty(),
            ),
            wgpu::Features::empty()
        );
        assert_eq!(
            required_features_for_policy(
                DeviceFeaturePolicy::V2OptionalTimestamp,
                wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_PASSES,
            ),
            wgpu::Features::TIMESTAMP_QUERY
        );
    }

    #[test]
    fn v2_timestamp_disabled_falls_back_to_empty_features() {
        let adapter_features = wgpu::Features::INDIRECT_FIRST_INSTANCE;
        assert_eq!(
            required_features_for_policy(
                DeviceFeaturePolicy::V2OptionalTimestamp,
                adapter_features,
            ),
            wgpu::Features::empty()
        );
    }

    #[test]
    fn adapter_reported_rgba16float_capability_is_required() {
        let unsupported = wgpu::TextureFormatFeatures {
            allowed_usages: wgpu::TextureUsages::TEXTURE_BINDING,
            flags: wgpu::TextureFormatFeatureFlags::empty(),
        };
        assert!(!physical_environment_adapter_reported(unsupported));
        let supported = wgpu::TextureFormatFeatures {
            allowed_usages: wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::RENDER_ATTACHMENT,
            flags: wgpu::TextureFormatFeatureFlags::FILTERABLE,
        };
        assert!(physical_environment_adapter_reported(supported));
    }

    #[test]
    fn device_legal_decision_ignores_adapter_specific_usages() {
        // The adapter snapshot may advertise usages the created device never
        // received (they need TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES). The
        // production decision must be taken on the guaranteed features.
        let adapter_only = wgpu::TextureFormatFeatures {
            allowed_usages: wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::STORAGE_BINDING,
            flags: wgpu::TextureFormatFeatureFlags::FILTERABLE,
        };
        assert!(physical_environment_adapter_reported(adapter_only));

        // Rgba16Float is renderable, bindable and filterable by the WebGPU
        // spec, so the device-legal verdict is true even with no extra device
        // features requested.
        assert!(physical_environment_device_legal(wgpu::Features::empty()));
        assert!(
            !wgpu::Features::empty()
                .contains(wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES)
        );
    }

    #[test]
    fn required_usages_are_texture_binding_and_render_attachment() {
        assert_eq!(
            physical_environment_required_usages(),
            wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::RENDER_ATTACHMENT
        );
        assert!(!physical_environment_required_usages().contains(wgpu::TextureUsages::COPY_DST));
        assert!(
            !physical_environment_required_usages().contains(wgpu::TextureUsages::STORAGE_BINDING)
        );
    }
}
