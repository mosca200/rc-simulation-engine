use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Pod, Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
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
                .chain(vertex.color)
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
    let vertices = vec![
        Vertex {
            position: [-2_000.0, ground_y_render_m, -2_000.0],
            color: [0.12, 0.30, 0.10],
        },
        Vertex {
            position: [2_000.0, ground_y_render_m, -2_000.0],
            color: [0.12, 0.30, 0.10],
        },
        Vertex {
            position: [2_000.0, ground_y_render_m, 2_000.0],
            color: [0.18, 0.38, 0.14],
        },
        Vertex {
            position: [-2_000.0, ground_y_render_m, 2_000.0],
            color: [0.18, 0.38, 0.14],
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
            [0.78, 0.80, 0.68]
        } else {
            [0.36, 0.44, 0.31]
        };
        vertices.push(Vertex {
            position: [-EXTENT_M as f32, grid_y_render_m, coordinate],
            color,
        });
        vertices.push(Vertex {
            position: [EXTENT_M as f32, grid_y_render_m, coordinate],
            color,
        });
        vertices.push(Vertex {
            position: [coordinate, grid_y_render_m, -EXTENT_M as f32],
            color,
        });
        vertices.push(Vertex {
            position: [coordinate, grid_y_render_m, EXTENT_M as f32],
            color,
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
    vertices.push(Vertex {
        position: start,
        color,
    });
    vertices.push(Vertex {
        position: end,
        color,
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
    let faces = [
        [4, 5, 7, 6],
        [2, 3, 1, 0],
        [1, 3, 7, 5],
        [2, 0, 4, 6],
        [2, 6, 7, 3],
        [4, 0, 1, 5],
    ];
    for face in faces {
        debug_assert!(vertices.len() <= u32::MAX as usize - 4);
        let base_index = vertices.len() as u32;
        vertices.extend(face.map(|corner_index| Vertex {
            position: corners[corner_index],
            color,
        }));
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
                .chain(vertex.color)
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
    fn reference_grid_and_axes_are_finite_line_pairs() {
        let mesh = reference_grid_and_axes();
        assert!(!mesh.vertices().is_empty());
        assert_eq!(mesh.vertices().len() % 2, 0);
        assert!(mesh.vertices().iter().all(|vertex| {
            vertex
                .position
                .into_iter()
                .chain(vertex.color)
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
            mesh.indices()
                .iter()
                .all(|index| (*index as usize) < mesh.vertices().len())
        );
    }
}
