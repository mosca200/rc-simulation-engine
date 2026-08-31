use crate::{AircraftMesh, MeshError, Vertex};
use gltf::buffer::Source;
use std::path::{Path, PathBuf};
use thiserror::Error;

const DEFAULT_VERTEX_COLOR: [f32; 3] = [0.78, 0.22, 0.12];

#[derive(Debug, Error)]
pub enum GlbLoadError {
    #[error("failed to open or parse GLB asset {path}: {source}")]
    Parse {
        path: PathBuf,
        #[source]
        source: gltf::Error,
    },
    #[error("GLB asset {path} does not contain an embedded binary buffer")]
    MissingBinaryBuffer { path: PathBuf },
    #[error("GLB asset {path} uses an external buffer URI, which P1 does not support")]
    ExternalBuffer { path: PathBuf },
    #[error("GLB asset {path} contains no triangle mesh primitives")]
    MissingTrianglePrimitive { path: PathBuf },
    #[error("GLB asset {path} triangle primitive {primitive_index} has no POSITION attribute")]
    MissingPositions {
        path: PathBuf,
        primitive_index: usize,
    },
    #[error("GLB asset {path} triangle primitive {primitive_index} has no indices")]
    MissingIndices {
        path: PathBuf,
        primitive_index: usize,
    },
    #[error("GLB asset {path} primitive {primitive_index} has a mismatched COLOR_0 count")]
    MismatchedColors {
        path: PathBuf,
        primitive_index: usize,
    },
    #[error("GLB asset {path} exceeds the supported u32 vertex index range")]
    TooManyVertices { path: PathBuf },
    #[error("GLB asset {path} produced an invalid CPU render mesh: {source}")]
    InvalidMesh {
        path: PathBuf,
        #[source]
        source: MeshError,
    },
}

/// Loads the deliberately small P1 subset of glTF 2.0 from a binary GLB.
pub fn load_glb_mesh(path: impl AsRef<Path>) -> Result<AircraftMesh, GlbLoadError> {
    let path = path.as_ref();
    let document = gltf::Gltf::open(path).map_err(|source| GlbLoadError::Parse {
        path: path.to_path_buf(),
        source,
    })?;
    if document
        .buffers()
        .any(|buffer| matches!(buffer.source(), Source::Uri(_)))
    {
        return Err(GlbLoadError::ExternalBuffer {
            path: path.to_path_buf(),
        });
    }
    let binary = document
        .blob
        .as_deref()
        .ok_or_else(|| GlbLoadError::MissingBinaryBuffer {
            path: path.to_path_buf(),
        })?;

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    let mut triangle_primitive_count = 0_usize;

    for (primitive_index, primitive) in document
        .meshes()
        .flat_map(|mesh| mesh.primitives())
        .enumerate()
    {
        if primitive.mode() != gltf::mesh::Mode::Triangles {
            continue;
        }
        triangle_primitive_count += 1;
        let reader = primitive.reader(|buffer| match buffer.source() {
            Source::Bin => Some(binary),
            Source::Uri(_) => None,
        });
        let positions = reader
            .read_positions()
            .ok_or_else(|| GlbLoadError::MissingPositions {
                path: path.to_path_buf(),
                primitive_index,
            })?
            .collect::<Vec<_>>();
        let colors = reader
            .read_colors(0)
            .map(|values| values.into_rgb_f32().collect::<Vec<_>>());
        if colors
            .as_ref()
            .is_some_and(|colors| colors.len() != positions.len())
        {
            return Err(GlbLoadError::MismatchedColors {
                path: path.to_path_buf(),
                primitive_index,
            });
        }
        let base = u32::try_from(vertices.len()).map_err(|_| GlbLoadError::TooManyVertices {
            path: path.to_path_buf(),
        })?;
        vertices.extend(positions.into_iter().enumerate().map(|(index, position)| {
            Vertex {
                position,
                color: colors
                    .as_ref()
                    .map_or(DEFAULT_VERTEX_COLOR, |colors| colors[index]),
            }
        }));
        let primitive_indices =
            reader
                .read_indices()
                .ok_or_else(|| GlbLoadError::MissingIndices {
                    path: path.to_path_buf(),
                    primitive_index,
                })?;
        indices.extend(primitive_indices.into_u32().map(|index| base + index));
    }

    if triangle_primitive_count == 0 {
        return Err(GlbLoadError::MissingTrianglePrimitive {
            path: path.to_path_buf(),
        });
    }
    AircraftMesh::new(vertices, indices).map_err(|source| GlbLoadError::InvalidMesh {
        path: path.to_path_buf(),
        source,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn acro_asset() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../models/acro_electric_01/aircraft.glb")
    }

    #[test]
    fn acro_glb_parses_to_a_finite_indexed_triangle_mesh() {
        let mesh = load_glb_mesh(acro_asset()).unwrap();
        assert!(!mesh.vertices().is_empty());
        assert!(!mesh.indices().is_empty());
        assert_eq!(mesh.indices().len() % 3, 0);
        assert!(mesh.vertices().iter().all(|vertex| {
            vertex
                .position
                .into_iter()
                .chain(vertex.color)
                .all(f32::is_finite)
        }));
        assert!(
            mesh.indices()
                .iter()
                .all(|&index| (index as usize) < mesh.vertices().len())
        );
    }

    #[test]
    fn acro_glb_bounds_follow_render_local_coordinate_contract() {
        let mesh = load_glb_mesh(acro_asset()).unwrap();
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
        assert!(minimum.into_iter().chain(maximum).all(f32::is_finite));
        assert!(
            minimum[0] < -1.0 && maximum[0] > 1.0,
            "+X is right across the wing"
        );
        assert!(maximum[1] > 0.35, "+Y is up along the vertical stabilizer");
        assert!(
            minimum[2] < -1.2,
            "-Z is the clearly extended nose direction"
        );
        assert!(maximum[2] > 0.8, "+Z reaches the tail");
    }

    #[test]
    fn missing_glb_is_an_explicit_parse_error() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("does-not-exist.glb");
        assert!(matches!(
            load_glb_mesh(path),
            Err(GlbLoadError::Parse { .. })
        ));
    }

    #[test]
    fn malformed_glb_is_an_explicit_parse_error() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
        assert!(matches!(
            load_glb_mesh(path),
            Err(GlbLoadError::Parse { .. })
        ));
    }
}
