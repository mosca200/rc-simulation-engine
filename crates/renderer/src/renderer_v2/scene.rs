//! RV2-3 deduplicated rigid-GLB GPU scene.
//!
//! The loader remains the scene-graph authority. This module consumes its
//! deterministic `meshes` and `instances` vectors without repeating traversal
//! or baking node transforms into mesh-local vertices.

use crate::{GlbAsset, Mat4, matrix_to_wgsl_columns};
use bytemuck::{Pod, Zeroable};
use std::{mem::size_of, ops::Range};
use wgpu::util::DeviceExt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PresentationKind {
    RigidGlb,
    ArticulatedGlb,
    Procedural,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ScenePath {
    LegacyFlat,
    V2Instanced,
}

#[must_use]
pub(crate) const fn select_scene_path(is_v2: bool, kind: PresentationKind) -> ScenePath {
    match (is_v2, kind) {
        (true, PresentationKind::RigidGlb) => ScenePath::V2Instanced,
        _ => ScenePath::LegacyFlat,
    }
}

/// Immutable diagnostics for one uploaded V2 rigid scene.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct GpuSceneStats {
    pub(crate) unique_mesh_count: usize,
    pub(crate) primitive_count: usize,
    pub(crate) scene_instance_count: usize,
    pub(crate) instance_buffer_bytes: u64,
    pub(crate) draw_count: usize,
}

/// One deterministic scene-traversal record. `mesh_index` addresses a unique
/// upload definition; `instance_index` addresses the persistent instance
/// buffer in the loader's original traversal order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SceneInstanceRecord {
    pub(crate) node_index: usize,
    pub(crate) mesh_index: usize,
    pub(crate) instance_index: u32,
    pub(crate) world_transform: Mat4,
}

/// CPU-side upload plan, intentionally independent of wgpu so scene semantics
/// and geometry deduplication are testable without a GPU.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct GpuScenePlan {
    pub(crate) mesh_primitive_counts: Box<[usize]>,
    pub(crate) instances: Box<[SceneInstanceRecord]>,
    pub(crate) stats: GpuSceneStats,
}

impl GpuScenePlan {
    #[must_use]
    pub(crate) fn from_asset(asset: &GlbAsset) -> Self {
        let mesh_primitive_counts = asset
            .meshes
            .iter()
            .map(|mesh| mesh.primitives.len())
            .collect::<Vec<_>>()
            .into_boxed_slice();
        let instances = asset
            .instances
            .iter()
            .enumerate()
            .map(|(instance_index, instance)| SceneInstanceRecord {
                node_index: instance.node_index,
                mesh_index: instance.mesh_index,
                instance_index: instance_index as u32,
                world_transform: instance.world_transform,
            })
            .collect::<Vec<_>>()
            .into_boxed_slice();
        let primitive_count = mesh_primitive_counts.iter().sum();
        let draw_count = instances
            .iter()
            .map(|instance| mesh_primitive_counts[instance.mesh_index])
            .sum();
        let stats = GpuSceneStats {
            unique_mesh_count: mesh_primitive_counts.len(),
            primitive_count,
            scene_instance_count: instances.len(),
            instance_buffer_bytes: (instances.len() * size_of::<GpuSceneInstanceRaw>()) as u64,
            draw_count,
        };
        Self {
            mesh_primitive_counts,
            instances,
            stats,
        }
    }
}

/// Per-instance vertex data: four model-matrix columns followed by three
/// inverse-transpose normal-matrix columns. Both are static GLB node data;
/// the aircraft root remains the existing per-frame object uniform.
#[repr(C)]
#[derive(Debug, Clone, Copy, Pod, Zeroable)]
pub(crate) struct GpuSceneInstanceRaw {
    pub(crate) model_columns: [[f32; 4]; 4],
    pub(crate) normal_columns: [[f32; 4]; 3],
}

impl GpuSceneInstanceRaw {
    fn from_transform(transform: &Mat4) -> Self {
        let inverse = transform.inverse().unwrap_or_else(Mat4::identity);
        let inverse_rows = inverse.rows();
        Self {
            model_columns: matrix_to_wgsl_columns(transform),
            // Columns of transpose(inverse(M)) are rows of inverse(M).
            normal_columns: [
                [
                    inverse_rows[0][0],
                    inverse_rows[0][1],
                    inverse_rows[0][2],
                    0.0,
                ],
                [
                    inverse_rows[1][0],
                    inverse_rows[1][1],
                    inverse_rows[1][2],
                    0.0,
                ],
                [
                    inverse_rows[2][0],
                    inverse_rows[2][1],
                    inverse_rows[2][2],
                    0.0,
                ],
            ],
        }
    }
}

pub(crate) struct GpuScenePrimitive {
    pub(crate) vertex_buffer: wgpu::Buffer,
    pub(crate) index_buffer: wgpu::Buffer,
    pub(crate) index_count: u32,
    pub(crate) material_index: usize,
}

pub(crate) struct GpuSceneMesh {
    pub(crate) primitives: Box<[GpuScenePrimitive]>,
}

/// Persistent V2 scene resources. Geometry exists once per `GlbAsset::meshes`
/// primitive regardless of the number of scene-node instances.
pub(crate) struct GpuScene {
    pub(crate) meshes: Box<[GpuSceneMesh]>,
    pub(crate) instances: Box<[SceneInstanceRecord]>,
    pub(crate) instance_buffer: wgpu::Buffer,
    stats: GpuSceneStats,
}

impl GpuScene {
    pub(crate) fn upload(
        device: &wgpu::Device,
        asset: &GlbAsset,
        material_indices: &[Vec<usize>],
    ) -> Self {
        let plan = GpuScenePlan::from_asset(asset);
        debug_assert_eq!(material_indices.len(), asset.meshes.len());
        let meshes = asset
            .meshes
            .iter()
            .enumerate()
            .map(|(mesh_index, mesh)| {
                debug_assert_eq!(material_indices[mesh_index].len(), mesh.primitives.len());
                let primitives = mesh
                    .primitives
                    .iter()
                    .enumerate()
                    .map(|(primitive_index, primitive)| GpuScenePrimitive {
                        vertex_buffer: device.create_buffer_init(
                            &wgpu::util::BufferInitDescriptor {
                                label: Some("RV2 GPU scene mesh vertices"),
                                contents: bytemuck::cast_slice(&primitive.vertices),
                                usage: wgpu::BufferUsages::VERTEX,
                            },
                        ),
                        index_buffer: device.create_buffer_init(
                            &wgpu::util::BufferInitDescriptor {
                                label: Some("RV2 GPU scene mesh indices"),
                                contents: bytemuck::cast_slice(&primitive.indices),
                                usage: wgpu::BufferUsages::INDEX,
                            },
                        ),
                        index_count: primitive.indices.len() as u32,
                        material_index: material_indices[mesh_index][primitive_index],
                    })
                    .collect::<Vec<_>>()
                    .into_boxed_slice();
                GpuSceneMesh { primitives }
            })
            .collect::<Vec<_>>()
            .into_boxed_slice();
        let raw_instances = plan
            .instances
            .iter()
            .map(|instance| GpuSceneInstanceRaw::from_transform(&instance.world_transform))
            .collect::<Vec<_>>();
        let instance_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("RV2 static GLB scene instances"),
            contents: bytemuck::cast_slice(&raw_instances),
            usage: wgpu::BufferUsages::VERTEX,
        });
        Self {
            meshes,
            instances: plan.instances,
            instance_buffer,
            stats: plan.stats,
        }
    }

    #[must_use]
    pub(crate) const fn stats(&self) -> GpuSceneStats {
        self.stats
    }

    #[must_use]
    pub(crate) fn instance_range(instance: &SceneInstanceRecord) -> Range<u32> {
        instance.instance_index..instance.instance_index + 1
    }
}

#[must_use]
#[cfg(test)]
pub(crate) fn compose_instance_model(root: Mat4, instance_transform: Mat4) -> Mat4 {
    root * instance_transform
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        GlbMesh, GlbSceneInstance, PrimitiveMaterial, RenderPrimitive, SamplerConfig, Vertex,
    };

    fn primitive() -> RenderPrimitive {
        RenderPrimitive {
            vertices: vec![Vertex {
                position: [0.0, 0.0, 0.0],
                normal: [0.0, 1.0, 0.0],
                color: [1.0; 4],
                uv: [0.0; 2],
            }],
            indices: vec![0, 0, 0],
            material: PrimitiveMaterial {
                base_color_factor: [1.0; 4],
                base_color_texture: None,
                metallic_factor: 0.0,
                roughness_factor: 0.5,
                sampler_config: SamplerConfig::default_sampler(),
            },
        }
    }

    fn asset_with_instances(transforms: &[Mat4]) -> GlbAsset {
        GlbAsset {
            primitives: vec![primitive()],
            meshes: vec![GlbMesh {
                gltf_mesh_index: 0,
                primitives: vec![primitive()],
            }],
            instances: transforms
                .iter()
                .enumerate()
                .map(|(node_index, transform)| GlbSceneInstance {
                    node_index,
                    node_name: Some(format!("node-{node_index}")),
                    mesh_index: 0,
                    world_transform: *transform,
                })
                .collect(),
        }
    }

    #[test]
    fn one_mesh_one_instance_produces_one_upload_definition() {
        let plan = GpuScenePlan::from_asset(&asset_with_instances(&[Mat4::identity()]));
        assert_eq!(plan.mesh_primitive_counts.as_ref(), [1]);
        assert_eq!(plan.stats.unique_mesh_count, 1);
        assert_eq!(plan.stats.primitive_count, 1);
        assert_eq!(plan.stats.scene_instance_count, 1);
        assert_eq!(plan.stats.instance_buffer_bytes, 112);
        assert_eq!(plan.stats.draw_count, 1);
    }

    #[test]
    fn repeated_mesh_instances_do_not_duplicate_geometry_definitions() {
        let second = Mat4::from_rows([
            [1.0, 0.0, 0.0, 3.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, -2.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let plan = GpuScenePlan::from_asset(&asset_with_instances(&[Mat4::identity(), second]));
        assert_eq!(plan.stats.unique_mesh_count, 1);
        assert_eq!(plan.stats.primitive_count, 1);
        assert_eq!(plan.stats.scene_instance_count, 2);
        assert_eq!(plan.stats.instance_buffer_bytes, 224);
        assert_eq!(plan.stats.draw_count, 2);
        assert_ne!(
            plan.instances[0].world_transform,
            plan.instances[1].world_transform
        );
        assert_eq!(plan.instances[0].node_index, 0);
        assert_eq!(plan.instances[1].node_index, 1);
        assert_eq!(plan.instances[0].mesh_index, plan.instances[1].mesh_index);
    }

    #[test]
    fn instance_order_and_plan_are_deterministic() {
        let asset = asset_with_instances(&[Mat4::identity(), Mat4::identity()]);
        assert_eq!(
            GpuScenePlan::from_asset(&asset),
            GpuScenePlan::from_asset(&asset)
        );
    }

    #[test]
    fn root_times_instance_composition_is_explicit_and_identity_matches_flat() {
        let root = Mat4::from_rows([
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 1.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, -4.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        let instance = Mat4::from_rows([
            [1.0, 0.0, 0.0, 3.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 1.0],
            [0.0, 0.0, 0.0, 1.0],
        ]);
        assert_eq!(compose_instance_model(root, Mat4::identity()), root);
        assert_eq!(compose_instance_model(root, instance).rows()[0][3], 13.0);
        assert_eq!(compose_instance_model(root, instance).rows()[2][3], -3.0);
    }

    #[test]
    fn only_v2_rigid_glb_selects_the_instanced_path() {
        assert_eq!(
            select_scene_path(true, PresentationKind::RigidGlb),
            ScenePath::V2Instanced
        );
        for (is_v2, kind) in [
            (false, PresentationKind::RigidGlb),
            (true, PresentationKind::ArticulatedGlb),
            (false, PresentationKind::ArticulatedGlb),
            (true, PresentationKind::Procedural),
        ] {
            assert_eq!(select_scene_path(is_v2, kind), ScenePath::LegacyFlat);
        }
    }

    #[test]
    fn instance_layout_is_seven_vec4_columns() {
        assert_eq!(size_of::<GpuSceneInstanceRaw>(), 112);
    }

    #[test]
    fn shader_keeps_legacy_and_adds_instanced_forward_and_shadow_entries() {
        let shader = include_str!("../shader.wgsl");
        for entry in [
            "fn vs_main(",
            "fn vs_shadow(",
            "fn vs_gpu_scene(",
            "fn vs_gpu_scene_shadow(",
            "fn fs_lit(",
            "fn fs_postprocess(",
        ] {
            assert!(shader.contains(entry), "missing shader entry {entry}");
        }
        for location in 4..=10 {
            assert!(
                shader.contains(&format!("@location({location})")),
                "missing instance matrix attribute {location}"
            );
        }
    }

    #[test]
    fn gpu_scene_resources_are_initialization_only() {
        let source = include_str!("../gpu.rs");
        let (_, render_and_after) = source
            .split_once("pub fn render(&mut self, frame: &RenderFrame)")
            .expect("legacy render entry must remain present");
        let (frame_path, _) = render_and_after
            .split_once("fn check_asynchronous_gpu_error")
            .expect("frame path boundary must remain present");
        assert!(!frame_path.contains("GpuScene::upload("));
        assert!(!frame_path.contains("create_gpu_scene_pipeline("));
        assert!(!frame_path.contains("create_gpu_scene_shadow_pipeline("));
        assert!(!frame_path.contains("create_buffer_init("));
        assert!(source.contains("entry_point: Some(\"vs_gpu_scene\")"));
        assert!(source.contains("entry_point: Some(\"vs_gpu_scene_shadow\")"));
        assert!(source.contains("array_stride: size_of::<GpuSceneInstanceRaw>() as u64"));
        assert!(source.contains("step_mode: wgpu::VertexStepMode::Instance"));
    }
}
