use bytemuck::{Pod, Zeroable};

/// Deterministic fallback normal for degenerate or unlit geometry.
pub const SAFE_NORMAL: [f32; 3] = [0.0, 1.0, 0.0];

/// Deterministic fallback UV when texture coordinates are absent.
pub const SAFE_UV: [f32; 2] = [0.0, 0.0];

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
    pub uv: [f32; 2],
}

#[derive(Debug, Clone, PartialEq)]
pub struct AircraftMesh {
    vertices: Vec<Vertex>,
    indices: Vec<u32>,
}

impl AircraftMesh {
    pub fn new(vertices: Vec<Vertex>, indices: Vec<u32>) -> Result<Self, MeshError> {
        if vertices.is_empty() {
            return Err(MeshError::EmptyVertices);
        }
        if indices.is_empty() || !indices.len().is_multiple_of(3) {
            return Err(MeshError::InvalidTriangleIndices);
        }
        if !vertices.iter().all(|vertex| {
            vertex
                .position
                .into_iter()
                .chain(vertex.normal)
                .chain(vertex.color)
                .chain(vertex.uv)
                .all(f32::is_finite)
        }) {
            return Err(MeshError::NonFiniteVertex);
        }
        if indices
            .iter()
            .any(|&index| index as usize >= vertices.len())
        {
            return Err(MeshError::IndexOutOfBounds);
        }
        Ok(Self { vertices, indices })
    }

    #[must_use]
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }

    #[must_use]
    pub fn indices(&self) -> &[u32] {
        &self.indices
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum MeshError {
    #[error("render mesh has no vertices")]
    EmptyVertices,
    #[error("render mesh indices must contain one or more complete triangles")]
    InvalidTriangleIndices,
    #[error("render mesh contains a non-finite vertex component")]
    NonFiniteVertex,
    #[error("render mesh contains an out-of-range vertex index")]
    IndexOutOfBounds,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LineMesh {
    vertices: Vec<Vertex>,
}

impl LineMesh {
    #[must_use]
    pub fn vertices(&self) -> &[Vertex] {
        &self.vertices
    }
}

/// Low-poly S7 placeholder: 1.64 m span and approximately 1.5 m length.
#[must_use]
pub fn aircraft_mesh() -> AircraftMesh {
    let mut vertices = Vec::with_capacity(24 * 9);
    let mut indices = Vec::with_capacity(36 * 9);

    add_box(
        &mut vertices,
        &mut indices,
        [-0.11, -0.10, -0.56],
        [0.11, 0.10, 0.60],
        [0.20, 0.34, 0.82],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [-0.13, -0.11, -0.82],
        [0.13, 0.11, -0.56],
        [0.92, 0.18, 0.10],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [-0.82, -0.035, -0.18],
        [-0.10, 0.035, 0.20],
        [0.94, 0.78, 0.16],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [0.10, -0.035, -0.18],
        [0.82, 0.035, 0.20],
        [0.18, 0.70, 0.94],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [-0.38, -0.025, 0.47],
        [-0.08, 0.025, 0.70],
        [0.86, 0.32, 0.72],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [0.08, -0.025, 0.47],
        [0.38, 0.025, 0.70],
        [0.86, 0.32, 0.72],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [-0.035, 0.08, 0.42],
        [0.035, 0.38, 0.69],
        [0.18, 0.82, 0.26],
    );
    add_box(
        &mut vertices,
        &mut indices,
        [-0.085, 0.10, -0.28],
        [0.085, 0.16, 0.02],
        [1.0, 0.48, 0.08],
    );

    AircraftMesh { vertices, indices }
}

/// G1E: separate movable-surface meshes for the procedural fallback.
#[derive(Debug, Clone, PartialEq)]
pub struct ArticulatedAircraftMesh {
    pub rigid: AircraftMesh,
    pub surfaces: [Option<AircraftMesh>; crate::VISUAL_SLOT_COUNT],
}

impl ArticulatedAircraftMesh {
    #[must_use]
    pub fn surface(&self, surface: crate::SurfaceId) -> Option<&AircraftMesh> {
        self.surfaces[surface.index()].as_ref()
    }
}

#[must_use]
pub fn articulated_aircraft_mesh() -> ArticulatedAircraftMesh {
    let mut rv = Vec::with_capacity(24 * 5);
    let mut ri = Vec::with_capacity(36 * 5);
    add_box(
        &mut rv,
        &mut ri,
        [-0.11, -0.10, -0.56],
        [0.11, 0.10, 0.60],
        [0.20, 0.34, 0.82],
    );
    add_box(
        &mut rv,
        &mut ri,
        [-0.42, -0.035, -0.18],
        [-0.10, 0.035, 0.20],
        [0.94, 0.78, 0.16],
    );
    add_box(
        &mut rv,
        &mut ri,
        [0.10, -0.035, -0.18],
        [0.42, 0.035, 0.20],
        [0.18, 0.70, 0.94],
    );
    add_box(
        &mut rv,
        &mut ri,
        [-0.38, -0.025, 0.30],
        [0.38, 0.025, 0.47],
        [0.86, 0.32, 0.72],
    );
    add_box(
        &mut rv,
        &mut ri,
        [-0.035, 0.08, 0.10],
        [0.035, 0.38, 0.42],
        [0.18, 0.82, 0.26],
    );
    let rigid = AircraftMesh {
        vertices: rv,
        indices: ri,
    };
    let mut surfaces: [Option<AircraftMesh>; crate::VISUAL_SLOT_COUNT] =
        [None, None, None, None, None];
    surfaces[crate::SurfaceId::LeftAileron.index()] = Some(colored_box(
        [-0.82, -0.035, -0.18],
        [-0.10, 0.035, 0.20],
        [0.94, 0.78, 0.16],
    ));
    surfaces[crate::SurfaceId::RightAileron.index()] = Some(colored_box(
        [0.10, -0.035, -0.18],
        [0.82, 0.035, 0.20],
        [0.18, 0.70, 0.94],
    ));
    surfaces[crate::SurfaceId::Elevator.index()] = Some(colored_box(
        [-0.38, -0.025, 0.47],
        [0.38, 0.025, 0.70],
        [0.86, 0.32, 0.72],
    ));
    surfaces[crate::SurfaceId::Rudder.index()] = Some(colored_box(
        [-0.035, 0.38, 0.42],
        [0.035, 0.68, 0.69],
        [0.18, 0.82, 0.26],
    ));
    ArticulatedAircraftMesh { rigid, surfaces }
}

#[must_use]
pub fn articulated_binding_table() -> crate::SurfaceBindingTable {
    crate::SurfaceBindingTable::empty()
        .with_hinge(
            crate::SurfaceHinge::new(
                crate::SurfaceId::LeftAileron,
                [-0.10, 0.0, 0.01],
                [1.0, 0.0, 0.0],
                1.0,
            )
            .expect("hinge"),
        )
        .with_hinge(
            crate::SurfaceHinge::new(
                crate::SurfaceId::RightAileron,
                [0.10, 0.0, 0.01],
                [1.0, 0.0, 0.0],
                1.0,
            )
            .expect("hinge"),
        )
        .with_hinge(
            crate::SurfaceHinge::new(
                crate::SurfaceId::Elevator,
                [0.0, 0.0, 0.47],
                [1.0, 0.0, 0.0],
                1.0,
            )
            .expect("hinge"),
        )
        .with_hinge(
            crate::SurfaceHinge::new(
                crate::SurfaceId::Rudder,
                [0.0, 0.38, 0.42],
                [0.0, 1.0, 0.0],
                1.0,
            )
            .expect("hinge"),
        )
}

fn colored_box(minimum: [f32; 3], maximum: [f32; 3], color: [f32; 3]) -> AircraftMesh {
    let mut vertices = Vec::with_capacity(24);
    let mut indices = Vec::with_capacity(36);
    add_box(&mut vertices, &mut indices, minimum, maximum, color);
    AircraftMesh { vertices, indices }
}

const DEFAULT_GROUND_Y_RENDER_M: f32 = -30.04;
const DEFAULT_GRID_Y_RENDER_M: f32 = -30.0;

/// Flat render-only local ground at the default manual-flight altitude.
#[must_use]
pub fn ground_plane() -> AircraftMesh {
    ground_plane_at(DEFAULT_GROUND_Y_RENDER_M)
}

/// Flat render-only local ground at a caller-selected render-world height.
#[must_use]
pub fn ground_plane_at(ground_y_render_m: f32) -> AircraftMesh {
    debug_assert!(ground_y_render_m.is_finite());
    let up = [0.0_f32, 1.0, 0.0];
    let vertices = vec![
        Vertex {
            position: [-2_000.0, ground_y_render_m, -2_000.0],
            normal: up,
            color: [0.12, 0.30, 0.10, 1.0],
            uv: [0.0, 0.0],
        },
        Vertex {
            position: [2_000.0, ground_y_render_m, -2_000.0],
            normal: up,
            color: [0.12, 0.30, 0.10, 1.0],
            uv: [1.0, 0.0],
        },
        Vertex {
            position: [2_000.0, ground_y_render_m, 2_000.0],
            normal: up,
            color: [0.18, 0.38, 0.14, 1.0],
            uv: [1.0, 1.0],
        },
        Vertex {
            position: [-2_000.0, ground_y_render_m, 2_000.0],
            normal: up,
            color: [0.18, 0.38, 0.14, 1.0],
            uv: [0.0, 1.0],
        },
    ];
    AircraftMesh::new(vertices, vec![0, 2, 1, 0, 3, 2]).expect("static ground mesh is valid")
}

/// Static grid at the default manual-flight altitude reference.
#[must_use]
pub fn reference_grid_and_axes() -> LineMesh {
    reference_grid_and_axes_at(DEFAULT_GRID_Y_RENDER_M)
}

/// Static grid on render XZ plus East/Up/South axes at a selected ground height.
#[must_use]
pub fn reference_grid_and_axes_at(grid_y_render_m: f32) -> LineMesh {
    const EXTENT_M: i32 = 1_000;
    const SPACING_M: usize = 25;
    debug_assert!(grid_y_render_m.is_finite());
    let mut vertices = Vec::with_capacity(340);
    for coordinate in (-EXTENT_M..=EXTENT_M).step_by(SPACING_M) {
        let coordinate = coordinate as f32;
        let is_major = (coordinate as i32) % 100 == 0;
        let color = if is_major {
            [0.78, 0.80, 0.68, 1.0]
        } else {
            [0.36, 0.44, 0.31, 1.0]
        };
        vertices.push(Vertex {
            position: [-EXTENT_M as f32, grid_y_render_m, coordinate],
            normal: SAFE_NORMAL,
            color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [EXTENT_M as f32, grid_y_render_m, coordinate],
            normal: SAFE_NORMAL,
            color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [coordinate, grid_y_render_m, -EXTENT_M as f32],
            normal: SAFE_NORMAL,
            color,
            uv: SAFE_UV,
        });
        vertices.push(Vertex {
            position: [coordinate, grid_y_render_m, EXTENT_M as f32],
            normal: SAFE_NORMAL,
            color,
            uv: SAFE_UV,
        });
    }

    let origin = [0.0, grid_y_render_m, 0.0];
    add_line(
        &mut vertices,
        origin,
        [100.0, grid_y_render_m, 0.0],
        [1.0, 0.08, 0.08],
    );
    add_line(
        &mut vertices,
        origin,
        [0.0, grid_y_render_m + 100.0, 0.0],
        [0.08, 1.0, 0.08],
    );
    add_line(
        &mut vertices,
        origin,
        [0.0, grid_y_render_m, 100.0],
        [0.08, 0.20, 1.0],
    );
    LineMesh { vertices }
}

fn add_line(vertices: &mut Vec<Vertex>, start: [f32; 3], end: [f32; 3], color: [f32; 3]) {
    let rgba = [color[0], color[1], color[2], 1.0];
    vertices.push(Vertex {
        position: start,
        normal: SAFE_NORMAL,
        color: rgba,
        uv: SAFE_UV,
    });
    vertices.push(Vertex {
        position: end,
        normal: SAFE_NORMAL,
        color: rgba,
        uv: SAFE_UV,
    });
}

fn add_box(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u32>,
    minimum: [f32; 3],
    maximum: [f32; 3],
    color: [f32; 3],
) {
    let [minimum_x, minimum_y, minimum_z] = minimum;
    let [maximum_x, maximum_y, maximum_z] = maximum;
    let corners = [
        [minimum_x, minimum_y, minimum_z],
        [minimum_x, maximum_y, minimum_z],
        [minimum_x, minimum_y, maximum_z],
        [minimum_x, maximum_y, maximum_z],
        [maximum_x, minimum_y, minimum_z],
        [maximum_x, maximum_y, minimum_z],
        [maximum_x, minimum_y, maximum_z],
        [maximum_x, maximum_y, maximum_z],
    ];
    let faces: [[usize; 4]; 6] = [
        [4, 5, 7, 6],
        [2, 3, 1, 0],
        [1, 3, 7, 5],
        [2, 0, 4, 6],
        [2, 6, 7, 3],
        [4, 0, 1, 5],
    ];
    let face_normals = [
        [1.0_f32, 0.0, 0.0],
        [-1.0_f32, 0.0, 0.0],
        [0.0_f32, 1.0, 0.0],
        [0.0_f32, -1.0, 0.0],
        [0.0_f32, 0.0, 1.0],
        [0.0_f32, 0.0, -1.0],
    ];
    for (face, face_normal) in faces.iter().zip(face_normals.iter()) {
        debug_assert!(vertices.len() <= u32::MAX as usize - 4);
        let base_index = vertices.len() as u32;
        let rgba = [color[0], color[1], color[2], 1.0];
        for &corner_index in face {
            vertices.push(Vertex {
                position: corners[corner_index],
                normal: *face_normal,
                color: rgba,
                uv: SAFE_UV,
            });
        }
        indices.extend_from_slice(&[
            base_index,
            base_index + 1,
            base_index + 2,
            base_index,
            base_index + 2,
            base_index + 3,
        ]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn procedural_aircraft_mesh_is_valid_and_plausibly_bounded() {
        let mesh = aircraft_mesh();
        assert!(!mesh.vertices().is_empty());
        assert!(!mesh.indices().is_empty());
        assert!(mesh.vertices().iter().all(|vertex| {
            vertex
                .position
                .into_iter()
                .chain(vertex.normal)
                .chain(vertex.color)
                .chain(vertex.uv)
                .all(f32::is_finite)
        }));

        let minimum = [0, 1, 2].map(|axis| {
            mesh.vertices()
                .iter()
                .map(|vertex| vertex.position[axis])
                .fold(f32::INFINITY, f32::min)
        });
        let maximum = [0, 1, 2].map(|axis| {
            mesh.vertices()
                .iter()
                .map(|vertex| vertex.position[axis])
                .fold(f32::NEG_INFINITY, f32::max)
        });
        assert!(minimum[0] < -0.7 && maximum[0] > 0.7);
        assert!(minimum[1] < 0.0 && maximum[1] > 0.3);
        assert!(minimum[2] < -0.7 && maximum[2] > 0.6);
        assert!((1.4..=1.8).contains(&(maximum[0] - minimum[0])));
        assert!((1.2..=1.6).contains(&(maximum[2] - minimum[2])));
    }

    #[test]
    fn procedural_aircraft_indices_are_all_in_bounds() {
        let mesh = aircraft_mesh();
        assert!(!mesh.indices().is_empty());
        assert!(
            mesh.indices()
                .iter()
                .all(|index| (*index as usize) < mesh.vertices().len())
        );
    }

    #[test]
    fn procedural_aircraft_normals_are_unit_length() {
        let mesh = aircraft_mesh();
        for vertex in mesh.vertices() {
            let length =
                (vertex.normal[0].powi(2) + vertex.normal[1].powi(2) + vertex.normal[2].powi(2))
                    .sqrt();
            assert!(
                (length - 1.0).abs() < 1.0e-5,
                "normal {0:?} has length {1}",
                vertex.normal,
                length
            );
        }
    }

    #[test]
    fn reference_grid_and_axes_are_finite_line_pairs() {
        let mesh = reference_grid_and_axes();
        assert!(!mesh.vertices().is_empty());
        assert_eq!(mesh.vertices().len() % 2, 0);
        assert!(mesh.vertices().iter().all(|vertex| {
            vertex
                .position
                .into_iter()
                .chain(vertex.normal)
                .chain(vertex.color)
                .chain(vertex.uv)
                .all(f32::is_finite)
        }));
        assert!(
            mesh.vertices()
                .iter()
                .any(|vertex| vertex.position == [100.0, -30.0, 0.0])
        );
        assert!(
            mesh.vertices()
                .iter()
                .any(|vertex| vertex.position == [0.0, 70.0, 0.0])
        );
        assert!(
            mesh.vertices()
                .iter()
                .any(|vertex| vertex.position == [0.0, -30.0, 100.0])
        );
    }

    #[test]
    fn runtime_ground_and_grid_follow_the_selected_reference_height() {
        let ground = ground_plane_at(-75.04);
        assert!(
            ground
                .vertices()
                .iter()
                .all(|vertex| vertex.position[1] == -75.04)
        );
        let references = reference_grid_and_axes_at(-75.0);
        assert!(
            references
                .vertices()
                .iter()
                .any(|vertex| vertex.position == [0.0, -75.0, 0.0])
        );
    }

    #[test]
    fn ground_is_a_valid_render_only_plane_below_the_grid() {
        let mesh = ground_plane();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.indices().len(), 6);
        assert!(
            mesh.vertices()
                .iter()
                .all(|vertex| vertex.position[1] < -30.0)
        );
        assert!(
            mesh.vertices()
                .iter()
                .all(|vertex| vertex.normal == [0.0, 1.0, 0.0])
        );
        assert!(
            mesh.indices()
                .iter()
                .all(|index| (*index as usize) < mesh.vertices().len())
        );
    }

    #[test]
    fn vertex_layout_has_expected_size_and_alignment() {
        assert_eq!(std::mem::size_of::<Vertex>(), 48);
        assert_eq!(std::mem::align_of::<Vertex>(), 4);
    }
}
