//! G1C: glTF/GLB asset loading with base-color texture support.
//!
//! Loads the G1A/G1B/G1C subset of glTF 2.0:
//! - POSITION (required)
//! - NORMAL (optional with fallback)
//! - COLOR_0 (optional)
//! - TEXCOORD_0 (optional)
//! - pbrMetallicRoughness.baseColorFactor
//! - pbrMetallicRoughness.baseColorTexture (G1C)
//!
//! # Texture Color Space
//!
//! Base-color textures are uploaded as sRGB (`Rgba8UnormSrgb`). The shader
//! does not apply manual gamma correction — hardware sRGB sampling handles it.
//!
//! # Material Architecture
//!
//! Each primitive preserves its material identity. A primitive may reference:
//! - A base color factor (RGBA)
//! - A base color texture (with sampler settings)
//!
//! The combination formula is:
//! ```text
//! base_rgba = baseColorFactor * vertex_COLOR_0 * textureSample(baseColorTexture, TEXCOORD_0)
//! ```

use crate::mesh::{MeshError, SAFE_NORMAL, SAFE_UV, Vertex};
use crate::texture::{DecodedTexture, SamplerConfig, TextureLoadError, decode_image};
use gltf::buffer::Source;
use std::path::{Path, PathBuf};
use thiserror::Error;

/// glTF default `pbrMetallicRoughness.baseColorFactor` when absent.
const GLTF_DEFAULT_BASE_COLOR: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

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
    #[error("GLB asset {path} uses an external buffer URI, which is not supported")]
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
    #[error("GLB asset {path} primitive {primitive_index} has a mismatched NORMAL count")]
    MismatchedNormals {
        path: PathBuf,
        primitive_index: usize,
    },
    #[error("GLB asset {path} primitive {primitive_index} has a mismatched TEXCOORD_0 count")]
    MismatchedTexCoords {
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
    #[error(
        "GLB asset {path} primitive {primitive_index} requests TEXCOORD_{requested} but only TEXCOORD_0 is supported"
    )]
    UnsupportedTexCoord {
        path: PathBuf,
        primitive_index: usize,
        requested: u32,
    },
    #[error("GLB asset {path} has malformed image data for texture {texture_index}: {source}")]
    MalformedImage {
        path: PathBuf,
        texture_index: usize,
        #[source]
        source: TextureLoadError,
    },
    #[error("GLB asset {path} image {image_index} has no buffer view data")]
    MissingImageBufferView { path: PathBuf, image_index: usize },
}

/// Per-primitive material data extracted from the glTF document.
///
/// G1C: Now includes optional base-color texture with sampler configuration.
#[derive(Debug, Clone)]
pub struct PrimitiveMaterial {
    pub base_color_factor: [f32; 4],
    pub base_color_texture: Option<DecodedTexture>,
    pub sampler_config: SamplerConfig,
}

impl PrimitiveMaterial {
    fn from_gltf_material(
        material: &gltf::Material,
        binary: &[u8],
        path: &Path,
        primitive_index: usize,
    ) -> Result<Self, GlbLoadError> {
        let pbr = material.pbr_metallic_roughness();
        let base_color_factor = pbr.base_color_factor();

        let base_color_texture = if let Some(info) = pbr.base_color_texture() {
            // Check for unsupported texCoord set.
            if info.tex_coord() != 0 {
                return Err(GlbLoadError::UnsupportedTexCoord {
                    path: path.to_path_buf(),
                    primitive_index,
                    requested: info.tex_coord(),
                });
            }

            let texture = info.texture();
            let source = texture.source();
            let texture_index = source.index();
            let source_data = extract_image_data(source, binary, path)?;
            let decoded = decode_image(&source_data).map_err(|decode_error| {
                GlbLoadError::MalformedImage {
                    path: path.to_path_buf(),
                    texture_index,
                    source: decode_error,
                }
            })?;

            Some(decoded)
        } else {
            None
        };

        let sampler_config = if let Some(info) = pbr.base_color_texture() {
            SamplerConfig::from_gltf_sampler(&info.texture().sampler())
        } else {
            SamplerConfig::default_sampler()
        };

        Ok(Self {
            base_color_factor,
            base_color_texture,
            sampler_config,
        })
    }

    fn default_material() -> Self {
        Self {
            base_color_factor: GLTF_DEFAULT_BASE_COLOR,
            base_color_texture: None,
            sampler_config: SamplerConfig::default_sampler(),
        }
    }
}

/// Extract raw image data from a glTF image source.
fn extract_image_data(
    image: gltf::Image,
    binary: &[u8],
    path: &Path,
) -> Result<Vec<u8>, GlbLoadError> {
    match image.source() {
        gltf::image::Source::View { view, .. } => {
            let buffer = view.buffer();
            if !matches!(buffer.source(), Source::Bin) {
                return Err(GlbLoadError::ExternalBuffer {
                    path: path.to_path_buf(),
                });
            }
            let start = view.offset();
            let end = start + view.length();
            if end > binary.len() {
                return Err(GlbLoadError::MissingImageBufferView {
                    path: path.to_path_buf(),
                    image_index: image.index(),
                });
            }
            Ok(binary[start..end].to_vec())
        }
        gltf::image::Source::Uri { .. } => Err(GlbLoadError::ExternalBuffer {
            path: path.to_path_buf(),
        }),
    }
}

/// A single renderable primitive with its own material.
#[derive(Debug, Clone)]
pub struct RenderPrimitive {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    pub material: PrimitiveMaterial,
}

/// A loaded GLB asset with one or more primitives.
///
/// G1C: Replaces the old single-mesh representation. Each primitive preserves
/// its material identity for correct multi-material rendering.
#[derive(Debug, Clone)]
pub struct GlbAsset {
    pub primitives: Vec<RenderPrimitive>,
}

impl GlbAsset {
    /// Total vertex count across all primitives.
    #[must_use]
    pub fn total_vertex_count(&self) -> usize {
        self.primitives.iter().map(|p| p.vertices.len()).sum()
    }

    /// Total index count across all primitives.
    #[must_use]
    pub fn total_index_count(&self) -> usize {
        self.primitives.iter().map(|p| p.indices.len()).sum()
    }
}

/// Loads a GLB asset with full primitive/material/texture preservation.
///
/// This is the primary entry point for G1C asset loading.
pub fn load_glb_asset(path: impl AsRef<Path>) -> Result<GlbAsset, GlbLoadError> {
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

    let mut primitives = Vec::new();
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

        let positions: Vec<[f32; 3]> = reader
            .read_positions()
            .ok_or_else(|| GlbLoadError::MissingPositions {
                path: path.to_path_buf(),
                primitive_index,
            })?
            .collect();

        let primitive_indices: Vec<u32> = reader
            .read_indices()
            .ok_or_else(|| GlbLoadError::MissingIndices {
                path: path.to_path_buf(),
                primitive_index,
            })?
            .into_u32()
            .collect();

        let colors: Option<Vec<[f32; 4]>> = reader
            .read_colors(0)
            .map(|values| values.into_rgba_f32().collect());
        if colors
            .as_ref()
            .is_some_and(|colors| colors.len() != positions.len())
        {
            return Err(GlbLoadError::MismatchedColors {
                path: path.to_path_buf(),
                primitive_index,
            });
        }

        let normals: Option<Vec<[f32; 3]>> = reader.read_normals().map(|values| values.collect());
        if normals
            .as_ref()
            .is_some_and(|normals| normals.len() != positions.len())
        {
            return Err(GlbLoadError::MismatchedNormals {
                path: path.to_path_buf(),
                primitive_index,
            });
        }

        let tex_coords: Option<Vec<[f32; 2]>> = reader
            .read_tex_coords(0)
            .map(|values| values.into_f32().collect());
        if tex_coords
            .as_ref()
            .is_some_and(|tex_coords| tex_coords.len() != positions.len())
        {
            return Err(GlbLoadError::MismatchedTexCoords {
                path: path.to_path_buf(),
                primitive_index,
            });
        }

        // Check for unsupported TEXCOORD_1 or higher on the primitive.
        for attr in primitive.attributes() {
            match attr.0 {
                gltf::Semantic::TexCoords(set) if set != 0 => {
                    return Err(GlbLoadError::UnsupportedTexCoord {
                        path: path.to_path_buf(),
                        primitive_index,
                        requested: set,
                    });
                }
                _ => {}
            }
        }

        let material = primitive
            .material()
            .index()
            .and_then(|index| document.materials().nth(index))
            .map(|material| {
                PrimitiveMaterial::from_gltf_material(&material, binary, path, primitive_index)
            })
            .transpose()?
            .unwrap_or_else(PrimitiveMaterial::default_material);

        let computed_normals = match normals {
            Some(explicit) => explicit.into_iter().map(normalize_or_safe).collect(),
            None => generate_area_weighted_vertex_normals(&positions, &primitive_indices),
        };

        let mut vertices = Vec::with_capacity(positions.len());
        for (index, position) in positions.into_iter().enumerate() {
            let vertex_color = colors
                .as_ref()
                .map_or([1.0_f32, 1.0, 1.0, 1.0], |colors| colors[index]);

            // G1C: Vertex color stores baseColorFactor * vertex_COLOR_0.
            // The texture is sampled separately in the shader.
            let combined_color = [
                material.base_color_factor[0] * vertex_color[0],
                material.base_color_factor[1] * vertex_color[1],
                material.base_color_factor[2] * vertex_color[2],
                material.base_color_factor[3] * vertex_color[3],
            ];

            vertices.push(Vertex {
                position,
                normal: computed_normals[index],
                color: combined_color,
                uv: tex_coords
                    .as_ref()
                    .map_or(SAFE_UV, |tex_coords| tex_coords[index]),
            });
        }

        primitives.push(RenderPrimitive {
            vertices,
            indices: primitive_indices,
            material,
        });
    }

    if triangle_primitive_count == 0 {
        return Err(GlbLoadError::MissingTrianglePrimitive {
            path: path.to_path_buf(),
        });
    }

    Ok(GlbAsset { primitives })
}

/// Legacy API: Load a GLB and merge all primitives into a single mesh.
///
/// This preserves backward compatibility with G1A/G1B code that expects
/// a single `AircraftMesh`. Materials are baked into vertex colors.
///
/// For new code that needs texture support, use `load_glb_asset` instead.
pub fn load_glb_mesh(path: impl AsRef<Path>) -> Result<crate::AircraftMesh, GlbLoadError> {
    let path_ref = path.as_ref();
    let asset = load_glb_asset(path_ref)?;
    merge_primitives_to_mesh(&asset.primitives, path_ref)
}

/// Merge multiple primitives into a single AircraftMesh.
///
/// This is used by the legacy `load_glb_mesh` API. Textures are ignored;
/// only baseColorFactor * vertex_COLOR_0 is baked into vertex colors.
fn merge_primitives_to_mesh(
    primitives: &[RenderPrimitive],
    path: &Path,
) -> Result<crate::AircraftMesh, GlbLoadError> {
    let mut vertices = Vec::new();
    let mut indices = Vec::new();

    for primitive in primitives {
        let base = u32::try_from(vertices.len()).map_err(|_| GlbLoadError::TooManyVertices {
            path: path.to_path_buf(),
        })?;

        vertices.extend(primitive.vertices.iter().copied());
        indices.extend(primitive.indices.iter().map(|&index| base + index));
    }

    crate::AircraftMesh::new(vertices, indices).map_err(|source| GlbLoadError::InvalidMesh {
        path: path.to_path_buf(),
        source,
    })
}

/// Computes area-weighted vertex normals from indexed triangle positions.
///
/// Each triangle's cross product (which has magnitude proportional to twice the
/// triangle area) is accumulated into each of its three vertices. The result is
/// normalized per-vertex. Degenerate triangles (zero-area) contribute nothing.
/// Vertices with zero-length accumulated normals receive `SAFE_NORMAL`.
fn generate_area_weighted_vertex_normals(positions: &[[f32; 3]], indices: &[u32]) -> Vec<[f32; 3]> {
    let mut accumulated = vec![[0.0_f32; 3]; positions.len()];

    for triangle in indices.as_chunks::<3>().0 {
        let i0 = triangle[0] as usize;
        let i1 = triangle[1] as usize;
        let i2 = triangle[2] as usize;
        if i0 >= positions.len() || i1 >= positions.len() || i2 >= positions.len() {
            continue;
        }
        let edge1 = sub3(positions[i1], positions[i0]);
        let edge2 = sub3(positions[i2], positions[i0]);
        let face_normal = cross3(edge1, edge2);
        accumulated[i0] = add3(accumulated[i0], face_normal);
        accumulated[i1] = add3(accumulated[i1], face_normal);
        accumulated[i2] = add3(accumulated[i2], face_normal);
    }

    accumulated.into_iter().map(normalize_or_safe).collect()
}

fn normalize_or_safe(vector: [f32; 3]) -> [f32; 3] {
    let length_sq = vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2];
    if length_sq.is_finite() && length_sq > f32::EPSILON * f32::EPSILON {
        let inv_length = length_sq.sqrt().recip();
        let result = [
            vector[0] * inv_length,
            vector[1] * inv_length,
            vector[2] * inv_length,
        ];
        if result.iter().copied().all(f32::is_finite) {
            return result;
        }
    }
    SAFE_NORMAL
}

fn sub3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn add3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn cross3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

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
                .chain(vertex.normal)
                .chain(vertex.color)
                .chain(vertex.uv)
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
    fn acro_glb_normals_are_finite_and_normalized() {
        let mesh = load_glb_mesh(acro_asset()).unwrap();
        for vertex in mesh.vertices() {
            assert!(vertex.normal.into_iter().all(f32::is_finite));
            let length_sq =
                vertex.normal[0].powi(2) + vertex.normal[1].powi(2) + vertex.normal[2].powi(2);
            assert!(
                (length_sq - 1.0).abs() < 1.0e-4,
                "normal {:?} has squared length {}",
                vertex.normal,
                length_sq
            );
        }
    }

    #[test]
    fn acro_glb_colors_are_finite_and_in_range() {
        let mesh = load_glb_mesh(acro_asset()).unwrap();
        for vertex in mesh.vertices() {
            assert!(
                vertex
                    .color
                    .iter()
                    .all(|c| c.is_finite() && *c >= 0.0 && *c <= 1.0)
            );
        }
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

    #[test]
    fn area_weighted_normals_single_triangle_produce_unit_normal() {
        let positions = vec![[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let indices = vec![0_u32, 1, 2];
        let normals = generate_area_weighted_vertex_normals(&positions, &indices);
        assert_eq!(normals.len(), 3);
        for normal in &normals {
            let length_sq = normal[0].powi(2) + normal[1].powi(2) + normal[2].powi(2);
            assert!((length_sq - 1.0).abs() < 1.0e-5);
            assert!(
                normal[2].abs() > 0.9,
                "expected +Z or -Z normal, got {:?}",
                normal
            );
        }
    }

    #[test]
    fn degenerate_triangle_produces_safe_normal_not_nan() {
        let positions = vec![[0.0_f32, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let indices = vec![0_u32, 1, 2];
        let normals = generate_area_weighted_vertex_normals(&positions, &indices);
        for normal in &normals {
            assert!(
                normal.iter().copied().all(f32::is_finite),
                "normal contains NaN or Inf: {:?}",
                normal
            );
            assert_eq!(*normal, SAFE_NORMAL);
        }
    }

    #[test]
    fn degenerate_collapsed_triangle_edge_produces_safe_normal() {
        let positions = vec![[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        let indices = vec![0_u32, 1, 2];
        let normals = generate_area_weighted_vertex_normals(&positions, &indices);
        for normal in &normals {
            assert!(normal.iter().copied().all(f32::is_finite));
            let length_sq = normal[0].powi(2) + normal[1].powi(2) + normal[2].powi(2);
            assert!((length_sq - 1.0).abs() < 1.0e-5);
        }
    }

    #[test]
    fn material_base_color_factor_defaults_to_white() {
        let material = PrimitiveMaterial::default_material();
        assert_eq!(material.base_color_factor, [1.0, 1.0, 1.0, 1.0]);
        assert!(material.base_color_texture.is_none());
    }

    #[test]
    fn color_combination_material_times_vertex_color_rgba() {
        let material = [0.8_f32, 0.5, 0.2, 0.9];
        let vertex_color = [0.5_f32, 1.0, 0.0, 0.6];
        let combined = [
            material[0] * vertex_color[0],
            material[1] * vertex_color[1],
            material[2] * vertex_color[2],
            material[3] * vertex_color[3],
        ];
        assert!((combined[0] - 0.4).abs() < f32::EPSILON);
        assert!((combined[1] - 0.5).abs() < f32::EPSILON);
        assert!((combined[2] - 0.0).abs() < f32::EPSILON);
        assert!((combined[3] - 0.54).abs() < 1.0e-6);
    }

    #[test]
    fn missing_vertex_color_defaults_to_white_opaque_so_material_passes_through() {
        let material = [0.8_f32, 0.5, 0.2, 0.7];
        let vertex_color = [1.0_f32, 1.0, 1.0, 1.0];
        let combined = [
            material[0] * vertex_color[0],
            material[1] * vertex_color[1],
            material[2] * vertex_color[2],
            material[3] * vertex_color[3],
        ];
        assert!((combined[0] - 0.8).abs() < f32::EPSILON);
        assert!((combined[1] - 0.5).abs() < f32::EPSILON);
        assert!((combined[2] - 0.2).abs() < f32::EPSILON);
        assert!((combined[3] - 0.7).abs() < f32::EPSILON);
    }

    #[test]
    fn material_alpha_is_preserved_through_combination() {
        let material = [1.0_f32, 1.0, 1.0, 0.42];
        let vertex_color = [1.0_f32, 1.0, 1.0, 1.0];
        let combined = [
            material[0] * vertex_color[0],
            material[1] * vertex_color[1],
            material[2] * vertex_color[2],
            material[3] * vertex_color[3],
        ];
        assert!((combined[3] - 0.42).abs() < f32::EPSILON);
    }

    #[test]
    fn explicit_non_unit_normal_is_normalized_to_unit_length() {
        let result = normalize_or_safe([5.0, 0.0, 0.0]);
        let length = (result[0].powi(2) + result[1].powi(2) + result[2].powi(2)).sqrt();
        assert!((length - 1.0).abs() < 1.0e-6);
        assert!((result[0] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn explicit_zero_normal_falls_back_to_safe_normal() {
        assert_eq!(normalize_or_safe([0.0, 0.0, 0.0]), SAFE_NORMAL);
    }

    #[test]
    fn explicit_nan_normal_falls_back_to_safe_normal() {
        assert_eq!(
            normalize_or_safe([f32::NAN, f32::NAN, f32::NAN]),
            SAFE_NORMAL
        );
    }

    #[test]
    fn explicit_inf_normal_falls_back_to_safe_normal() {
        assert_eq!(normalize_or_safe([f32::INFINITY, 0.0, 0.0]), SAFE_NORMAL);
    }

    #[test]
    fn normalize_or_safe_returns_unit_for_valid_vector() {
        let result = normalize_or_safe([3.0, 0.0, 4.0]);
        let length = (result[0].powi(2) + result[1].powi(2) + result[2].powi(2)).sqrt();
        assert!((length - 1.0).abs() < 1.0e-6);
        assert!((result[0] - 0.6).abs() < 1.0e-6);
        assert!((result[2] - 0.8).abs() < 1.0e-6);
    }

    #[test]
    fn normalize_or_safe_returns_safe_normal_for_zero_vector() {
        assert_eq!(normalize_or_safe([0.0, 0.0, 0.0]), SAFE_NORMAL);
    }

    #[test]
    fn normalize_or_safe_returns_safe_normal_for_nan_vector() {
        assert_eq!(normalize_or_safe([f32::NAN, 0.0, 0.0]), SAFE_NORMAL);
    }

    #[test]
    fn multiple_triangles_share_vertex_normals_correctly() {
        let positions = vec![
            [0.0_f32, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        let indices = vec![0_u32, 1, 2, 1, 3, 2];
        let normals = generate_area_weighted_vertex_normals(&positions, &indices);
        assert_eq!(normals.len(), 4);
        for normal in &normals {
            let length_sq = normal[0].powi(2) + normal[1].powi(2) + normal[2].powi(2);
            assert!((length_sq - 1.0).abs() < 1.0e-5);
            assert!(normal[2].abs() > 0.9);
        }
    }

    #[test]
    fn load_glb_asset_preserves_primitive_count() {
        let asset = load_glb_asset(acro_asset()).unwrap();
        assert!(!asset.primitives.is_empty());
    }

    #[test]
    fn load_glb_asset_vertices_are_finite() {
        let asset = load_glb_asset(acro_asset()).unwrap();
        for primitive in &asset.primitives {
            assert!(primitive.vertices.iter().all(|v| {
                v.position
                    .into_iter()
                    .chain(v.normal)
                    .chain(v.color)
                    .chain(v.uv)
                    .all(f32::is_finite)
            }));
        }
    }
}
