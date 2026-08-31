#![forbid(unsafe_code)]
//! Minimal desktop renderer isolated from all simulation-domain crates.

mod accumulator;
mod camera;
mod gpu;
mod math;
mod mesh;
mod pose;

pub use accumulator::{FixedStepAccumulator, FixedStepAccumulatorError, FixedStepPlan};
pub use camera::ChaseCamera;
pub use gpu::{RendererError, SurfaceError, WgpuRenderer};
pub use math::{Mat4, ProjectionError, matrix_to_wgsl_columns, webgpu_perspective};
pub use mesh::{AircraftMesh, LineMesh, Vertex, aircraft_mesh, reference_grid_and_axes};
pub use pose::{RenderDataError, RenderFrame, RenderPose, world_ned_pose_to_render};
