//! Plain-data contract for one explicitly requested V2 runtime visual audit.
//!
//! This module intentionally exposes no `wgpu` type. The GPU implementation
//! maps its already-owned runtime state into these DTOs only when the capture
//! application explicitly asks for an audit artifact.

use serde::Serialize;

use crate::{
    profiling::ProfileSnapshot,
    render_graph::PassId,
    renderer_v2::atmosphere::V2EnvironmentMode,
    shadow::{SHADOW_CASCADE_COUNT, SHADOW_CASCADE_SPLITS_M, SHADOW_MAP_RESOLUTION},
    vegetation::{VegetationDebugMode, VegetationFrameStats},
};

/// Versioned facts for the V2 renderer that produced one presentation frame.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAudit {
    pub schema_version: &'static str,
    pub identity: RuntimeVisualAuditIdentity,
    pub device: RuntimeVisualAuditDevice,
    pub environment: RuntimeVisualAuditEnvironment,
    pub image_pipeline: RuntimeVisualAuditImagePipeline,
    pub shadows: RuntimeVisualAuditShadows,
    pub terrain: RuntimeVisualAuditTerrain,
    pub vegetation: RuntimeVisualAuditVegetation,
    pub profiling: RuntimeVisualAuditProfiling,
}

impl RuntimeVisualAudit {
    pub const SCHEMA_VERSION: &'static str = "1.0.0";
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RuntimeVisualAuditIdentity {
    pub presentation_frame_index: u64,
    pub framebuffer_width: u32,
    pub framebuffer_height: u32,
    pub renderer_version: &'static str,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RuntimeVisualAuditDevice {
    pub adapter_name: String,
    pub backend: String,
    pub driver: String,
    pub driver_info: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RuntimeVisualAuditEnvironment {
    pub environment_mode: &'static str,
    pub physical_atmosphere_active: bool,
    pub physical_ibl_active: bool,
    pub aerial_perspective_active: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditImagePipeline {
    pub hdr_scene_format: &'static str,
    pub exposure_ev: f32,
    pub tone_mapper: &'static str,
    pub temporal_resolve_active: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditShadows {
    pub path_active: bool,
    pub cascade_count: usize,
    pub map_resolution: u32,
    pub split_distances_m: Vec<f32>,
    pub filtering: &'static str,
    pub filter_tap_count: Option<u32>,
    pub filter_tap_count_unavailable_reason: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditTerrain {
    pub render_mode: &'static str,
    pub debug_mode: &'static str,
    pub material_path_active: bool,
    pub sampler_anisotropy: u32,
    pub material_scale: Option<f32>,
    pub material_scale_unavailable_reason: Option<&'static str>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditVegetation {
    pub vegetation_present: bool,
    pub debug_mode: &'static str,
    pub stats: Option<RuntimeVisualAuditVegetationStats>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct RuntimeVisualAuditVegetationStats {
    pub total: u32,
    pub visible: u32,
    pub culled_frustum: u32,
    pub culled_distance: u32,
    pub lod_counts: [u32; 3],
    pub scene_draw_calls: u32,
    pub shadow_draw_calls: u32,
    pub uploaded_instance_bytes: u64,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditProfiling {
    pub presentation_frame_index: u64,
    pub cpu_frame_duration_ns: u64,
    pub gpu_timing_status: &'static str,
    pub gpu_timing_source_presentation_frame_index: Option<u64>,
    pub gpu_timing_frame_age: Option<u64>,
    pub gpu_timing_unavailable_reason: Option<&'static str>,
    pub passes: Vec<RuntimeVisualAuditPassTiming>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RuntimeVisualAuditPassTiming {
    pub pass_id: &'static str,
    pub label: &'static str,
    pub cpu_duration_ns: u64,
    pub gpu_duration_ns: Option<f64>,
}

pub(crate) fn environment_from_runtime(
    mode: V2EnvironmentMode,
    aerial_perspective_active: bool,
) -> RuntimeVisualAuditEnvironment {
    let physical = mode == V2EnvironmentMode::Physical;
    RuntimeVisualAuditEnvironment {
        environment_mode: if physical {
            "physical"
        } else {
            "analytical_fallback"
        },
        physical_atmosphere_active: physical,
        physical_ibl_active: physical,
        aerial_perspective_active,
    }
}

pub(crate) fn shadows_from_runtime_source() -> RuntimeVisualAuditShadows {
    RuntimeVisualAuditShadows {
        path_active: true,
        cascade_count: SHADOW_CASCADE_COUNT,
        map_resolution: SHADOW_MAP_RESOLUTION,
        split_distances_m: SHADOW_CASCADE_SPLITS_M.to_vec(),
        filtering: "linear comparison sampler with shader PCF",
        filter_tap_count: None,
        filter_tap_count_unavailable_reason: Some(
            "tap count is defined inside WGSL and has no Rust runtime source of truth",
        ),
    }
}

pub(crate) fn vegetation_from_runtime(
    debug_mode: VegetationDebugMode,
    stats: Option<&VegetationFrameStats>,
    uploaded_instance_bytes: Option<u64>,
) -> RuntimeVisualAuditVegetation {
    RuntimeVisualAuditVegetation {
        vegetation_present: stats.is_some(),
        debug_mode: debug_mode.label(),
        stats: stats.map(|stats| RuntimeVisualAuditVegetationStats {
            total: stats.total,
            visible: stats.visible,
            culled_frustum: stats.culled_frustum,
            culled_distance: stats.culled_distance,
            lod_counts: stats.lod_counts,
            scene_draw_calls: stats.scene_draw_calls,
            shadow_draw_calls: stats.shadow_draw_calls * SHADOW_CASCADE_COUNT as u32,
            uploaded_instance_bytes: uploaded_instance_bytes
                .expect("vegetation upload bytes exist whenever vegetation stats exist"),
        }),
    }
}

pub(crate) fn profiling_from_runtime(
    profile: &ProfileSnapshot,
) -> Option<RuntimeVisualAuditProfiling> {
    let presentation_frame_index = profile.presentation_frame_index?;
    let (status, source, age, reason, expose_gpu) = if !profile.gpu_timing_supported {
        (
            "timestamp_query_unsupported",
            None,
            None,
            Some("the selected device does not support timestamp queries"),
            false,
        )
    } else if let Some(source) = profile.gpu_presentation_frame_index
        && source <= presentation_frame_index
    {
        let age = presentation_frame_index - source;
        (
            if age == 0 {
                "current_frame_sample"
            } else {
                "previous_frame_sample"
            },
            Some(source),
            Some(age),
            None,
            true,
        )
    } else {
        (
            "asynchronous_result_not_ready",
            None,
            None,
            Some("no completed timestamp sample is available for this or an earlier frame"),
            false,
        )
    };
    Some(RuntimeVisualAuditProfiling {
        presentation_frame_index,
        cpu_frame_duration_ns: duration_ns(profile.cpu_frame),
        gpu_timing_status: status,
        gpu_timing_source_presentation_frame_index: source,
        gpu_timing_frame_age: age,
        gpu_timing_unavailable_reason: reason,
        passes: PassId::ALL
            .into_iter()
            .map(|pass| RuntimeVisualAuditPassTiming {
                pass_id: pass.audit_id(),
                label: pass.label(),
                cpu_duration_ns: duration_ns(profile.cpu_passes[pass.index()]),
                gpu_duration_ns: expose_gpu
                    .then_some(profile.gpu_pass_ns[pass.index()])
                    .flatten(),
            })
            .collect(),
    })
}

fn duration_ns(duration: std::time::Duration) -> u64 {
    duration.as_nanos().try_into().unwrap_or(u64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn example() -> RuntimeVisualAudit {
        RuntimeVisualAudit {
            schema_version: RuntimeVisualAudit::SCHEMA_VERSION,
            identity: RuntimeVisualAuditIdentity {
                presentation_frame_index: 10,
                framebuffer_width: 1920,
                framebuffer_height: 1080,
                renderer_version: "v2",
            },
            device: RuntimeVisualAuditDevice {
                adapter_name: "adapter".to_owned(),
                backend: "Vulkan".to_owned(),
                driver: "driver".to_owned(),
                driver_info: "info".to_owned(),
            },
            environment: RuntimeVisualAuditEnvironment {
                environment_mode: "physical",
                physical_atmosphere_active: true,
                physical_ibl_active: true,
                aerial_perspective_active: true,
            },
            image_pipeline: RuntimeVisualAuditImagePipeline {
                hdr_scene_format: "Rgba16Float",
                exposure_ev: 0.0,
                tone_mapper: "Khronos PBR Neutral",
                temporal_resolve_active: true,
            },
            shadows: RuntimeVisualAuditShadows {
                path_active: true,
                cascade_count: 3,
                map_resolution: 2048,
                split_distances_m: vec![32.0, 128.0, 512.0],
                filtering: "linear comparison sampler with shader PCF",
                filter_tap_count: None,
                filter_tap_count_unavailable_reason: Some(
                    "tap count is a WGSL-internal constant, not runtime state",
                ),
            },
            terrain: RuntimeVisualAuditTerrain {
                render_mode: "flat",
                debug_mode: "final",
                material_path_active: true,
                sampler_anisotropy: 16,
                material_scale: None,
                material_scale_unavailable_reason: Some(
                    "no single scalar represents the multi-frequency material",
                ),
            },
            vegetation: RuntimeVisualAuditVegetation {
                vegetation_present: false,
                debug_mode: "final",
                stats: None,
            },
            profiling: RuntimeVisualAuditProfiling {
                presentation_frame_index: 10,
                cpu_frame_duration_ns: 1,
                gpu_timing_status: "timestamp_query_unsupported",
                gpu_timing_source_presentation_frame_index: None,
                gpu_timing_frame_age: None,
                gpu_timing_unavailable_reason: Some(
                    "the selected device does not support timestamp queries",
                ),
                passes: vec![RuntimeVisualAuditPassTiming {
                    pass_id: "scene",
                    label: "G1C scene pass",
                    cpu_duration_ns: 1,
                    gpu_duration_ns: None,
                }],
            },
        }
    }

    #[test]
    fn contract_serializes_with_required_groups_and_version() {
        let value = serde_json::to_value(example()).unwrap();
        assert_eq!(value["schema_version"], "1.0.0");
        for key in [
            "identity",
            "device",
            "environment",
            "image_pipeline",
            "shadows",
            "terrain",
            "vegetation",
            "profiling",
        ] {
            assert!(value.get(key).is_some(), "missing {key}");
        }
    }

    #[test]
    fn public_audit_contract_has_no_wgpu_type_leakage() {
        let contract_source = include_str!("visual_audit.rs")
            .split("#[cfg(test)]")
            .next()
            .unwrap();
        assert!(!contract_source.contains("wgpu::"));
        let device = example().device;
        assert_eq!(device.adapter_name, "adapter");
        assert_eq!(device.backend, "Vulkan");
    }

    #[test]
    fn unavailable_values_serialize_as_null_not_zero() {
        let value = serde_json::to_value(example()).unwrap();
        assert!(value["shadows"]["filter_tap_count"].is_null());
        assert!(value["terrain"]["material_scale"].is_null());
        assert!(value["vegetation"]["stats"].is_null());
        assert!(value["profiling"]["passes"][0]["gpu_duration_ns"].is_null());
    }

    #[test]
    fn physical_and_fallback_map_from_actual_runtime_mode() {
        let physical = environment_from_runtime(V2EnvironmentMode::Physical, true);
        assert_eq!(physical.environment_mode, "physical");
        assert!(physical.physical_atmosphere_active);
        assert!(physical.physical_ibl_active);
        assert!(physical.aerial_perspective_active);

        let fallback = environment_from_runtime(V2EnvironmentMode::AnalyticalFallback, false);
        assert_eq!(fallback.environment_mode, "analytical_fallback");
        assert!(!fallback.physical_atmosphere_active);
        assert!(!fallback.physical_ibl_active);
        assert!(!fallback.aerial_perspective_active);
    }

    #[test]
    fn shadow_audit_reads_authoritative_renderer_constants() {
        let shadows = shadows_from_runtime_source();
        assert_eq!(shadows.cascade_count, SHADOW_CASCADE_COUNT);
        assert_eq!(shadows.map_resolution, SHADOW_MAP_RESOLUTION);
        assert_eq!(shadows.split_distances_m, SHADOW_CASCADE_SPLITS_M);
    }

    #[test]
    fn vegetation_absent_and_present_mapping_preserves_frame_stats() {
        let absent = vegetation_from_runtime(VegetationDebugMode::Final, None, None);
        assert!(!absent.vegetation_present);
        assert!(absent.stats.is_none());

        let source = VegetationFrameStats {
            total: 10,
            visible: 6,
            culled_frustum: 3,
            culled_distance: 1,
            lod_counts: [1, 2, 3],
            scene_draw_calls: 4,
            shadow_draw_calls: 5,
        };
        let present = vegetation_from_runtime(VegetationDebugMode::Lod, Some(&source), Some(288));
        let stats = present.stats.unwrap();
        assert!(present.vegetation_present);
        assert_eq!(present.debug_mode, "lod");
        assert_eq!(stats.lod_counts, source.lod_counts);
        assert_eq!(stats.shadow_draw_calls, 5 * SHADOW_CASCADE_COUNT as u32);
        assert_eq!(stats.uploaded_instance_bytes, 288);
    }

    fn profile(
        frame: u64,
        supported: bool,
        gpu_frame: Option<u64>,
        gpu_ns: Option<f64>,
    ) -> ProfileSnapshot {
        ProfileSnapshot {
            presentation_frame_index: Some(frame),
            cpu_frame: Duration::from_nanos(90),
            cpu_passes: [Duration::from_nanos(10); PassId::COUNT],
            gpu_timing_supported: supported,
            gpu_presentation_frame_index: gpu_frame,
            gpu_pass_ns: [gpu_ns; PassId::COUNT],
        }
    }

    #[test]
    fn unavailable_gpu_timing_is_null_not_zero() {
        let audit = profiling_from_runtime(&profile(10, false, None, None)).unwrap();
        assert_eq!(audit.gpu_timing_status, "timestamp_query_unsupported");
        assert!(
            audit
                .passes
                .iter()
                .all(|pass| pass.gpu_duration_ns.is_none())
        );
        assert!(audit.gpu_timing_unavailable_reason.is_some());
    }

    #[test]
    fn asynchronous_gpu_sample_keeps_its_real_frame_association() {
        let audit = profiling_from_runtime(&profile(10, true, Some(8), Some(5.0))).unwrap();
        assert_eq!(audit.gpu_timing_status, "previous_frame_sample");
        assert_eq!(audit.gpu_timing_source_presentation_frame_index, Some(8));
        assert_eq!(audit.gpu_timing_frame_age, Some(2));
        assert!(
            audit
                .passes
                .iter()
                .all(|pass| pass.gpu_duration_ns == Some(5.0))
        );
    }

    #[test]
    fn supported_but_not_ready_gpu_timing_remains_null() {
        let audit = profiling_from_runtime(&profile(10, true, None, None)).unwrap();
        assert_eq!(audit.gpu_timing_status, "asynchronous_result_not_ready");
        assert!(
            audit
                .passes
                .iter()
                .all(|pass| pass.gpu_duration_ns.is_none())
        );
    }
}
