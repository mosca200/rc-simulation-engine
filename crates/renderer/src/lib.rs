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
//! - Centralized f64→f32 render world origin conversion
//! - Dedicated aircraft object uniform buffer (per-frame updated)

mod accumulator;
mod camera;
mod glb;
mod gpu;
mod math;
mod mesh;
mod pose;
pub mod terrain;
pub mod texture;

pub use accumulator::{FixedStepAccumulator, FixedStepAccumulatorError, FixedStepPlan};
pub use camera::{
    ChaseCamera, RENDER_WORLD_UP, exponential_fog_factor, sun_alignment, view_elevation,
};
pub use glb::{
    GlbAsset, GlbLoadError, PrimitiveMaterial, RenderPrimitive, load_glb_asset, load_glb_mesh,
};
pub use gpu::{PresentationAsset, RendererError, SKY_CLEAR_COLOR, SurfaceError, WgpuRenderer};
pub use math::{Mat4, ProjectionError, matrix_to_wgsl_columns, webgpu_perspective};
pub use mesh::{
    AircraftMesh, LineMesh, MeshError, SAFE_NORMAL, SAFE_UV, Vertex, aircraft_mesh, ground_plane,
    ground_plane_at, reference_grid_and_axes, reference_grid_and_axes_at,
};
pub use pose::{RenderDataError, RenderFrame, RenderPose, world_ned_pose_to_render};
pub use terrain::{
    TerrainChunk, TerrainHeightField, TerrainMaterial, generate_centered_terrain_chunks,
    generate_flat_terrain, generate_rolling_terrain, generate_terrain_chunks,
};
pub use texture::{
    DecodedTexture, SamplerConfig, SamplerFilter, SamplerWrap, TextureLoadError, decode_image,
};
