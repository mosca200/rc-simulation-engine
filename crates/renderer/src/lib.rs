#![forbid(unsafe_code)]
//! Minimal desktop renderer isolated from all simulation-domain crates.
//!
//! # G1C: Base-Color Textures and Terrain Foundation
//!
//! This milestone adds:
//! - glTF base-color texture support (PNG/JPEG embedded in GLB)
//! - Per-primitive material binding with persistent GPU resources
//! - Terrain height field with chunked rendering (centered around render origin)
//! - Terrain UV tiling, normals, and lighting/fog integration
//! - Centralized f64â†’f32 render world origin conversion
//! - Dedicated aircraft object uniform buffer (per-frame updated)

mod accumulator;
mod backend;
mod camera;
mod capture;
mod device;
pub mod env1_material;
mod glb;
mod gpu;
mod math;
mod mesh;
mod photo_field;
mod photo_field_gpu;
mod pose;
mod profiling;
mod render_graph;
mod renderer_v2;
mod resources;
pub mod scenery;
mod shadow;
mod surfaces;
pub mod terrain;
pub mod terrain_textures;
pub mod texture;
pub mod vegetation;
pub mod vegetation_assets;
mod visual_audit;

pub use accumulator::{FixedStepAccumulator, FixedStepAccumulatorError, FixedStepPlan};
pub use backend::{DesktopRenderer, RendererVersion};
pub use camera::{
    CameraConfig, CameraMode, ChaseCamera, ChaseCameraConfig, PilotCamera, RENDER_WORLD_UP,
    exponential_fog_factor, sun_alignment, view_elevation,
};
pub use capture::{CaptureRenderOutcome, CapturedFrame, FrameCaptureError};
pub use glb::{
    GlbAsset, GlbLoadError, GlbMesh, GlbSceneInstance, PrimitiveMaterial, RenderPrimitive,
    load_glb_asset, load_glb_bytes, load_glb_mesh,
};
pub use gpu::{
    DEFAULT_EXPOSURE_EV, ExposureError, PresentationAsset, RenderOutcome, RenderTerrainMode,
    RendererError, SKY_CLEAR_COLOR, SurfaceError, TerrainDebugMode, WgpuRenderer,
    exposure_multiplier, validate_exposure_ev,
};
pub use math::{Mat4, ProjectionError, matrix_to_wgsl_columns, webgpu_perspective};
pub use mesh::{
    AircraftMesh, ArticulatedAircraftMesh, LineMesh, MeshError, SAFE_NORMAL, SAFE_UV, Vertex,
    aircraft_mesh, articulated_aircraft_mesh, articulated_binding_table, ground_plane,
    ground_plane_at, reference_grid_and_axes, reference_grid_and_axes_at,
    rv2_6_validation_target_mesh,
};
pub use photo_field::{
    PHOTO_FIELD_DEPTH_PROXY_FILE_NAME, PHOTO_FIELD_DEPTH_PROXY_GLB, PHOTO_FIELD_GROUND_NODE_NAME,
    PHOTO_FIELD_MANIFEST_JSON, PHOTO_FIELD_MANIFEST_SCHEMA_VERSION, PHOTO_FIELD_PANORAMA_FILE_NAME,
    PHOTO_FIELD_PANORAMA_HEIGHT, PHOTO_FIELD_PANORAMA_JPEG, PHOTO_FIELD_PANORAMA_WIDTH,
    PhotoFieldCameraError, PhotoFieldConfig, PhotoFieldManifest, PhotoFieldManifestError,
    embedded_photo_field_config, equirect_uv_from_direction, fixed_pilot_eye,
    photo_field_default_pilot_position,
};
pub use pose::{RenderDataError, RenderFrame, RenderPose, world_ned_pose_to_render};
pub use surfaces::{
    CONTROL_SURFACE_COUNT, ControlSurfacePresentation, GlbArticulationError, GlbArticulationPlan,
    GlbPrimitivePart, SurfaceBindingTable, SurfaceHinge, SurfaceId, VISUAL_SLOT_COUNT,
};
pub use terrain::{
    TerrainChunk, TerrainHeightField, TerrainMaterial, blend_linear_roughness,
    blend_registered_tangent_normals, detail_normal_fade_weight, generate_centered_terrain_chunks,
    generate_flat_terrain, generate_rolling_terrain, generate_terrain_chunks,
    reorient_rotated_tangent_normal, rotated_secondary_uv,
};
pub use texture::{
    DecodedTexture, SamplerConfig, SamplerFilter, SamplerMipmapFilter, SamplerWrap,
    TextureLoadError, decode_image,
};
pub use vegetation::{
    DEFAULT_DISTANCE_CULL_M, DEFAULT_HYSTERESIS_BAND, DEFAULT_LOD0_MAX_M, DEFAULT_LOD1_MAX_M,
    DEFAULT_VEGETATION_SEED, DeterministicRng, FrustumPlanes, GROUP_COUNT, LOD_COUNT, PART_COUNT,
    VegetationDebugMode, VegetationFrameStats, VegetationGpuInstance, VegetationInstance,
    VegetationLodConfig, VegetationWorld, flying_field_layout, placement_is_valid,
};
pub use vegetation_assets::{
    BARK_BASE, FOLIAGE_BASE, SPECIES_TARGET, VARIANTS_PER_SPECIES_TARGET, VegetationAsset,
    VegetationAssetSet, VegetationLod, VegetationLodSet, VegetationPart, VegetationSpecies,
    export_glb, part_metallic, part_roughness,
};
pub use visual_audit::{
    RuntimeVisualAudit, RuntimeVisualAuditDevice, RuntimeVisualAuditEnvironment,
    RuntimeVisualAuditIdentity, RuntimeVisualAuditImagePipeline, RuntimeVisualAuditPassTiming,
    RuntimeVisualAuditProfiling, RuntimeVisualAuditShadows, RuntimeVisualAuditTerrain,
    RuntimeVisualAuditVegetation, RuntimeVisualAuditVegetationStats,
};
