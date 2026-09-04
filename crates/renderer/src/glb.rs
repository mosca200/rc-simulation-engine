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
            // FIX 6: Borrow directly from the GLB binary blob instead of copying.
            let source_data = extract_image_data(source, binary, path)?;
            let decoded =
                decode_image(source_data).map_err(|decode_error| GlbLoadError::MalformedImage {
                    path: path.to_path_buf(),
                    texture_index,
                    source: decode_error,
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
///
/// FIX 6: Returns a borrow into the GLB binary blob instead of copying.
/// This avoids an unnecessary `Vec<u8>` allocation for embedded buffer-view images.
fn extract_image_data<'a>(
    image: gltf::Image,
    binary: &'a [u8],
    path: &Path,
) -> Result<&'a [u8], GlbLoadError> {
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
            Ok(&binary[start..end])
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
/// Materials are baked into vertex colors. For new code that needs texture
/// support, use `load_glb_asset` instead.
pub fn load_glb_mesh(path: impl AsRef<Path>) -> Result<crate::AircraftMesh, GlbLoadError> {
    let path_ref = path.as_ref();
    let asset = load_glb_asset(path_ref)?;
    merge_primitives_to_mesh(&asset.primitives, path_ref)
}

/// Merge multiple primitives into a single AircraftMesh.
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
        indices.extend(primitive.indices.iter().map(|index| base + index));
    }

    crate::AircraftMesh::new(vertices, indices).map_err(|source| GlbLoadError::InvalidMesh {
        path: path.to_path_buf(),
        source,
    })
}

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

// ---------------------------------------------------------------------------
// Test GLB builder — generates minimal GLB files in-memory for testing.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod test_glb_builder {
    #![allow(dead_code, clippy::all)]

    /// Build a minimal valid GLB with one mesh, one primitive, and optional texture.
    ///
    /// Returns the complete GLB binary as a `Vec<u8>`.
    pub struct GlbBuilder {
        positions: Vec<[f32; 3]>,
        normals: Option<Vec<[f32; 3]>>,
        colors: Option<Vec<[f32; 4]>>,
        tex_coords: Option<Vec<[f32; 2]>>,
        indices: Vec<u32>,
        base_color_factor: Option<[f32; 4]>,
        image_data: Option<Vec<u8>>,
        image_mime: Option<&'static str>,
        sampler_wrap_s: Option<u32>,
        sampler_wrap_t: Option<u32>,
        sampler_min_filter: Option<u32>,
        sampler_mag_filter: Option<u32>,
        material_index: Option<usize>,
        second_primitive: Option<SecondPrimitive>,
        second_image_data: Option<Vec<u8>>,
        second_image_mime: Option<&'static str>,
    }

    pub struct SecondPrimitive {
        pub positions: Vec<[f32; 3]>,
        pub indices: Vec<u32>,
        pub base_color_factor: Option<[f32; 4]>,
        pub has_texture: bool,
    }

    impl GlbBuilder {
        pub fn new() -> Self {
            Self {
                positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                normals: None,
                colors: None,
                tex_coords: None,
                indices: vec![0, 1, 2],
                base_color_factor: None,
                image_data: None,
                image_mime: None,
                sampler_wrap_s: None,
                sampler_wrap_t: None,
                sampler_min_filter: None,
                sampler_mag_filter: None,
                material_index: None,
                second_primitive: None,
                second_image_data: None,
                second_image_mime: None,
            }
        }

        pub fn with_positions(mut self, positions: Vec<[f32; 3]>) -> Self {
            self.positions = positions;
            self
        }

        pub fn with_normals(mut self, normals: Vec<[f32; 3]>) -> Self {
            self.normals = Some(normals);
            self
        }

        pub fn with_colors(mut self, colors: Vec<[f32; 4]>) -> Self {
            self.colors = Some(colors);
            self
        }

        pub fn with_tex_coords(mut self, tex_coords: Vec<[f32; 2]>) -> Self {
            self.tex_coords = Some(tex_coords);
            self
        }

        pub fn with_indices(mut self, indices: Vec<u32>) -> Self {
            self.indices = indices;
            self
        }

        pub fn with_base_color_factor(mut self, factor: [f32; 4]) -> Self {
            self.base_color_factor = Some(factor);
            self
        }

        pub fn with_png_texture(mut self, data: Vec<u8>) -> Self {
            self.image_data = Some(data);
            self.image_mime = Some("image/png");
            self
        }

        pub fn with_jpeg_texture(mut self, data: Vec<u8>) -> Self {
            self.image_data = Some(data);
            self.image_mime = Some("image/jpeg");
            self
        }

        pub fn with_sampler(
            mut self,
            wrap_s: u32,
            wrap_t: u32,
            min_filter: u32,
            mag_filter: u32,
        ) -> Self {
            self.sampler_wrap_s = Some(wrap_s);
            self.sampler_wrap_t = Some(wrap_t);
            self.sampler_min_filter = Some(min_filter);
            self.sampler_mag_filter = Some(mag_filter);
            self
        }

        pub fn with_second_primitive(mut self, prim: SecondPrimitive) -> Self {
            self.second_primitive = Some(prim);
            self
        }

        pub fn with_second_image(mut self, data: Vec<u8>, mime: &'static str) -> Self {
            self.second_image_data = Some(data);
            self.second_image_mime = Some(mime);
            self
        }

        /// Build the GLB binary.
        pub fn build(self) -> Vec<u8> {
            // Build the binary buffer (BIN chunk) first to know buffer view offsets.
            let mut bin_data = Vec::new();

            // Track buffer view offsets for accessors.
            let mut buffer_views = Vec::new();

            // Position buffer view.
            let pos_offset = bin_data.len();
            for pos in &self.positions {
                bin_data.extend_from_slice(&pos[0].to_le_bytes());
                bin_data.extend_from_slice(&pos[1].to_le_bytes());
                bin_data.extend_from_slice(&pos[2].to_le_bytes());
            }
            buffer_views.push(("position", pos_offset, bin_data.len() - pos_offset));

            // Index buffer view.
            let idx_offset = bin_data.len();
            for &idx in &self.indices {
                bin_data.extend_from_slice(&idx.to_le_bytes());
            }
            buffer_views.push(("index", idx_offset, bin_data.len() - idx_offset));

            // Normal buffer view.
            if let Some(normals) = &self.normals {
                let norm_offset = bin_data.len();
                for n in normals {
                    bin_data.extend_from_slice(&n[0].to_le_bytes());
                    bin_data.extend_from_slice(&n[1].to_le_bytes());
                    bin_data.extend_from_slice(&n[2].to_le_bytes());
                }
                buffer_views.push(("normal", norm_offset, bin_data.len() - norm_offset));
            }

            // Color buffer view.
            if let Some(colors) = &self.colors {
                let col_offset = bin_data.len();
                for c in colors {
                    bin_data.extend_from_slice(&c[0].to_le_bytes());
                    bin_data.extend_from_slice(&c[1].to_le_bytes());
                    bin_data.extend_from_slice(&c[2].to_le_bytes());
                    bin_data.extend_from_slice(&c[3].to_le_bytes());
                }
                buffer_views.push(("color", col_offset, bin_data.len() - col_offset));
            }

            // TexCoord buffer view.
            if let Some(tex_coords) = &self.tex_coords {
                let uv_offset = bin_data.len();
                for uv in tex_coords {
                    bin_data.extend_from_slice(&uv[0].to_le_bytes());
                    bin_data.extend_from_slice(&uv[1].to_le_bytes());
                }
                buffer_views.push(("texcoord", uv_offset, bin_data.len() - uv_offset));
            }

            // Image buffer view.
            let mut image_bv_index: Option<usize> = None;
            if let Some(image_data) = &self.image_data {
                let img_offset = bin_data.len();
                bin_data.extend_from_slice(image_data);
                image_bv_index = Some(buffer_views.len());
                buffer_views.push(("image", img_offset, bin_data.len() - img_offset));
            }

            // Second image buffer view.
            let mut second_image_bv_index: Option<usize> = None;
            if let Some(second_image_data) = &self.second_image_data {
                let img_offset = bin_data.len();
                bin_data.extend_from_slice(second_image_data);
                second_image_bv_index = Some(buffer_views.len());
                buffer_views.push(("image2", img_offset, bin_data.len() - img_offset));
            }

            // Second primitive data.
            let mut second_pos_bv: Option<(usize, usize)> = None;
            let mut second_idx_bv: Option<(usize, usize)> = None;
            if let Some(second) = &self.second_primitive {
                let pos_offset = bin_data.len();
                for pos in &second.positions {
                    bin_data.extend_from_slice(&pos[0].to_le_bytes());
                    bin_data.extend_from_slice(&pos[1].to_le_bytes());
                    bin_data.extend_from_slice(&pos[2].to_le_bytes());
                }
                let pos_len = bin_data.len() - pos_offset;
                let pos_bv_idx = buffer_views.len();
                buffer_views.push(("pos2", pos_offset, pos_len));
                second_pos_bv = Some((pos_bv_idx, second.positions.len()));

                let idx_offset = bin_data.len();
                for &idx in &second.indices {
                    bin_data.extend_from_slice(&idx.to_le_bytes());
                }
                let idx_len = bin_data.len() - idx_offset;
                let idx_bv_idx = buffer_views.len();
                buffer_views.push(("idx2", idx_offset, idx_len));
                second_idx_bv = Some((idx_bv_idx, second.indices.len()));
            }

            // Pad BIN to 4-byte alignment.
            let bin_padding = (4 - bin_data.len() % 4) % 4;
            for _ in 0..bin_padding {
                bin_data.push(0);
            }

            // Build JSON with correct buffer view indices.
            let json = self.build_json_with_views(
                &buffer_views,
                image_bv_index,
                second_image_bv_index,
                second_pos_bv,
                second_idx_bv,
            );
            let json_bytes = json.as_bytes();
            let json_padding = (4 - json_bytes.len() % 4) % 4;
            let json_chunk_len = json_bytes.len() + json_padding;

            // GLB header.
            let total_length = 12 + 8 + json_chunk_len + 8 + bin_data.len();
            let mut glb = Vec::with_capacity(total_length);

            // Header: magic, version, length.
            glb.extend_from_slice(&0x46546C67_u32.to_le_bytes()); // "glTF"
            glb.extend_from_slice(&2_u32.to_le_bytes()); // version
            glb.extend_from_slice(&(total_length as u32).to_le_bytes());

            // JSON chunk.
            glb.extend_from_slice(&(json_chunk_len as u32).to_le_bytes());
            glb.extend_from_slice(&0x4E4F534A_u32.to_le_bytes()); // "JSON"
            glb.extend_from_slice(json_bytes);
            for _ in 0..json_padding {
                glb.push(0x20); // space padding
            }

            // BIN chunk.
            glb.extend_from_slice(&(bin_data.len() as u32).to_le_bytes());
            glb.extend_from_slice(&0x004E4942_u32.to_le_bytes()); // "BIN\0"
            glb.extend_from_slice(&bin_data);

            glb
        }

        fn build_json(&self) -> String {
            self.build_json_with_views(&[], None, None, None, None)
        }

        fn build_json_with_views(
            &self,
            buffer_views: &[(&str, usize, usize)],
            image_bv_index: Option<usize>,
            second_image_bv_index: Option<usize>,
            second_pos_bv: Option<(usize, usize)>,
            second_idx_bv: Option<(usize, usize)>,
        ) -> String {
            let total_bin_len = buffer_views.iter().map(|(_, _, len)| len).sum::<usize>();

            // Compute position bounds.
            let min_pos = [
                self.positions
                    .iter()
                    .map(|p| p[0])
                    .fold(f32::INFINITY, f32::min),
                self.positions
                    .iter()
                    .map(|p| p[1])
                    .fold(f32::INFINITY, f32::min),
                self.positions
                    .iter()
                    .map(|p| p[2])
                    .fold(f32::INFINITY, f32::min),
            ];
            let max_pos = [
                self.positions
                    .iter()
                    .map(|p| p[0])
                    .fold(f32::NEG_INFINITY, f32::max),
                self.positions
                    .iter()
                    .map(|p| p[1])
                    .fold(f32::NEG_INFINITY, f32::max),
                self.positions
                    .iter()
                    .map(|p| p[2])
                    .fold(f32::NEG_INFINITY, f32::max),
            ];

            let mut accessors = Vec::new();
            let mut bv_json_entries = Vec::new();

            // Buffer view 0: positions.
            let pos_bv = buffer_views
                .iter()
                .find(|(name, _, _)| *name == "position")
                .unwrap();
            bv_json_entries.push(format!(
                r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34962}}"#,
                pos_bv.1, pos_bv.2
            ));
            accessors.push(format!(
                r#"{{"bufferView":0,"componentType":5126,"count":{},"type":"VEC3","min":[{},{},{}],"max":[{},{},{}]}}"#,
                self.positions.len(), min_pos[0], min_pos[1], min_pos[2], max_pos[0], max_pos[1], max_pos[2]
            ));

            // Buffer view 1: indices.
            let idx_bv = buffer_views
                .iter()
                .find(|(name, _, _)| *name == "index")
                .unwrap();
            bv_json_entries.push(format!(
                r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34963}}"#,
                idx_bv.1, idx_bv.2
            ));
            accessors.push(format!(
                r#"{{"bufferView":1,"componentType":5125,"count":{},"type":"SCALAR"}}"#,
                self.indices.len()
            ));

            let mut next_bv_idx = 2;
            let mut next_accessor_idx = 2;

            // Normals.
            if self.normals.is_some() {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "normal")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34962}}"#,
                    bv.1, bv.2
                ));
                accessors.push(format!(
                    r#"{{"bufferView":{},"componentType":5126,"count":{},"type":"VEC3"}}"#,
                    next_bv_idx,
                    self.normals.as_ref().unwrap().len()
                ));
                next_bv_idx += 1;
                next_accessor_idx += 1;
            }

            // Colors.
            if self.colors.is_some() {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "color")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34962}}"#,
                    bv.1, bv.2
                ));
                accessors.push(format!(
                    r#"{{"bufferView":{},"componentType":5126,"count":{},"type":"VEC4"}}"#,
                    next_bv_idx,
                    self.colors.as_ref().unwrap().len()
                ));
                next_bv_idx += 1;
                next_accessor_idx += 1;
            }

            // TexCoords.
            if self.tex_coords.is_some() {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "texcoord")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34962}}"#,
                    bv.1, bv.2
                ));
                accessors.push(format!(
                    r#"{{"bufferView":{},"componentType":5126,"count":{},"type":"VEC2"}}"#,
                    next_bv_idx,
                    self.tex_coords.as_ref().unwrap().len()
                ));
                next_bv_idx += 1;
                next_accessor_idx += 1;
            }

            // Image buffer views (don't get accessors).
            if image_bv_index.is_some() {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "image")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{}}}"#,
                    bv.1, bv.2
                ));
                next_bv_idx += 1;
            }
            if second_image_bv_index.is_some() {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "image2")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{}}}"#,
                    bv.1, bv.2
                ));
                next_bv_idx += 1;
            }

            // Second primitive buffer views.
            if let Some((bv_idx, count)) = second_pos_bv {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "pos2")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34962}}"#,
                    bv.1, bv.2
                ));
                // Compute min/max for the second primitive's positions.
                let second = self.second_primitive.as_ref().unwrap();
                let s_min = [
                    second
                        .positions
                        .iter()
                        .map(|p| p[0])
                        .fold(f32::INFINITY, f32::min),
                    second
                        .positions
                        .iter()
                        .map(|p| p[1])
                        .fold(f32::INFINITY, f32::min),
                    second
                        .positions
                        .iter()
                        .map(|p| p[2])
                        .fold(f32::INFINITY, f32::min),
                ];
                let s_max = [
                    second
                        .positions
                        .iter()
                        .map(|p| p[0])
                        .fold(f32::NEG_INFINITY, f32::max),
                    second
                        .positions
                        .iter()
                        .map(|p| p[1])
                        .fold(f32::NEG_INFINITY, f32::max),
                    second
                        .positions
                        .iter()
                        .map(|p| p[2])
                        .fold(f32::NEG_INFINITY, f32::max),
                ];
                accessors.push(format!(
                    r#"{{"bufferView":{},"componentType":5126,"count":{},"type":"VEC3","min":[{},{},{}],"max":[{},{},{}]}}"#,
                    bv_idx, count, s_min[0], s_min[1], s_min[2], s_max[0], s_max[1], s_max[2]
                ));
                next_bv_idx += 1;
                next_accessor_idx += 1;
            }
            if let Some((bv_idx, count)) = second_idx_bv {
                let bv = buffer_views
                    .iter()
                    .find(|(name, _, _)| *name == "idx2")
                    .unwrap();
                bv_json_entries.push(format!(
                    r#"{{"buffer":0,"byteOffset":{},"byteLength":{},"target":34963}}"#,
                    bv.1, bv.2
                ));
                accessors.push(format!(
                    r#"{{"bufferView":{},"componentType":5125,"count":{},"type":"SCALAR"}}"#,
                    bv_idx, count
                ));
                next_bv_idx += 1;
                next_accessor_idx += 1;
            }

            // Build attributes for first primitive.
            let mut attrs = r#""POSITION":0"#.to_string();
            let mut attr_idx = 2;
            if self.normals.is_some() {
                attrs.push_str(&format!(r#","NORMAL":{}"#, attr_idx));
                attr_idx += 1;
            }
            if self.colors.is_some() {
                attrs.push_str(&format!(r#","COLOR_0":{}"#, attr_idx));
                attr_idx += 1;
            }
            if self.tex_coords.is_some() {
                attrs.push_str(&format!(r#","TEXCOORD_0":{}"#, attr_idx));
            }

            // Build materials array.
            let mut materials = Vec::new();
            let has_texture = self.image_data.is_some();

            if has_texture || self.base_color_factor.is_some() {
                let mut mat = String::from(r#"{"pbrMetallicRoughness":{"#);
                if let Some(factor) = &self.base_color_factor {
                    mat.push_str(&format!(
                        r#""baseColorFactor":[{},{},{},"#,
                        factor[0], factor[1], factor[2]
                    ));
                    // Format alpha carefully to avoid trailing zeros issues.
                    let alpha_str = if factor[3] == 1.0 {
                        "1.0".to_string()
                    } else {
                        format!("{}", factor[3])
                    };
                    mat.push_str(&format!(r#"{}]"#, alpha_str));
                    if has_texture {
                        mat.push_str(r##","baseColorTexture":{"index":0}"##);
                    }
                } else if has_texture {
                    mat.push_str(r##""baseColorTexture":{"index":0}"##);
                }
                mat.push_str("}}");
                materials.push(mat);
            }

            // Second material for second primitive.
            if let Some(second) = &self.second_primitive {
                let mut mat = String::from(r#"{"pbrMetallicRoughness":{"#);
                if let Some(factor) = &second.base_color_factor {
                    mat.push_str(&format!(
                        r#""baseColorFactor":[{},{},{},{}]"#,
                        factor[0], factor[1], factor[2], factor[3]
                    ));
                }
                if second.has_texture && second_image_bv_index.is_some() {
                    if second.base_color_factor.is_some() {
                        mat.push_str(",");
                    }
                    mat.push_str(r#""baseColorTexture":{"index":1}"#);
                }
                mat.push_str("}}");
                materials.push(mat);
            }

            // Build images array.
            let mut images = Vec::new();
            if let Some(image_bv_idx) = image_bv_index {
                let mime = self.image_mime.unwrap_or("image/png");
                // Find the actual buffer view index in the JSON array.
                let json_bv_idx = self.find_image_json_bv_idx(
                    image_bv_idx,
                    buffer_views,
                    image_bv_index,
                    second_image_bv_index,
                );
                images.push(format!(
                    r#"{{"bufferView":{},"mimeType":"{}"}}"#,
                    json_bv_idx, mime
                ));
            }
            if let Some(second_image_bv_idx) = second_image_bv_index {
                let mime = self.second_image_mime.unwrap_or("image/png");
                let json_bv_idx = self.find_image_json_bv_idx(
                    second_image_bv_idx,
                    buffer_views,
                    image_bv_index,
                    second_image_bv_index,
                );
                images.push(format!(
                    r#"{{"bufferView":{},"mimeType":"{}"}}"#,
                    json_bv_idx, mime
                ));
            }

            // Build samplers array.
            let mut samplers = Vec::new();
            if self.sampler_wrap_s.is_some() {
                samplers.push(format!(
                    r#"{{"wrapS":{},"wrapT":{},"minFilter":{},"magFilter":{}}}"#,
                    self.sampler_wrap_s.unwrap(),
                    self.sampler_wrap_t.unwrap(),
                    self.sampler_min_filter.unwrap(),
                    self.sampler_mag_filter.unwrap()
                ));
            }

            // Build textures array.
            let mut textures = Vec::new();
            if has_texture {
                let sampler_ref = if samplers.is_empty() {
                    String::new()
                } else {
                    r#","sampler":0"#.to_string()
                };
                textures.push(format!(r#"{{"source":0{}}}"#, sampler_ref));
            }
            if let Some(second) = &self.second_primitive {
                if second.has_texture && second_image_bv_index.is_some() {
                    let sampler_ref = if samplers.is_empty() {
                        String::new()
                    } else {
                        r#","sampler":0"#.to_string()
                    };
                    textures.push(format!(r#"{{"source":1{}}}"#, sampler_ref));
                }
            }

            // Build primitives array.
            let mut primitives_json = Vec::new();
            primitives_json.push(format!(
                r#"{{"attributes":{{{}}},"indices":1{}}}"#,
                attrs,
                if let Some(mat_idx) =
                    self.material_index
                        .or(if materials.is_empty() { None } else { Some(0) })
                {
                    format!(r#","material":{}"#, mat_idx)
                } else {
                    String::new()
                }
            ));

            if let Some(_second) = &self.second_primitive {
                let second_pos_accessor = next_accessor_idx - 2;
                let second_idx_accessor = next_accessor_idx - 1;
                let mat_ref = if materials.len() > 1 {
                    format!(r#","material":{}"#, 1)
                } else {
                    String::new()
                };
                primitives_json.push(format!(
                    r#"{{"attributes":{{"POSITION":{}}},"indices":{}{}}}"#,
                    second_pos_accessor, second_idx_accessor, mat_ref
                ));
            }

            let _ = total_bin_len;
            let _ = next_bv_idx;

            format!(
                r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],"meshes":[{{"primitives":[{}]}}],"buffers":[{{"byteLength":{}}}],"bufferViews":[{}],"accessors":[{}]{}{}{}{}}}"#,
                primitives_json.join(","),
                // Recalculate total bin length from buffer views.
                buffer_views.iter().map(|(_, _, len)| len).sum::<usize>(),
                bv_json_entries.join(","),
                accessors.join(","),
                if materials.is_empty() {
                    String::new()
                } else {
                    format!(r#","materials":[{}]"#, materials.join(","))
                },
                if images.is_empty() {
                    String::new()
                } else {
                    format!(r#","images":[{}]"#, images.join(","))
                },
                if samplers.is_empty() {
                    String::new()
                } else {
                    format!(r#","samplers":[{}]"#, samplers.join(","))
                },
                if textures.is_empty() {
                    String::new()
                } else {
                    format!(r#","textures":[{}]"#, textures.join(","))
                },
            )
        }

        fn find_image_json_bv_idx(
            &self,
            _original_bv_index: usize,
            buffer_views: &[(&str, usize, usize)],
            image_bv_index: Option<usize>,
            second_image_bv_index: Option<usize>,
        ) -> usize {
            let mut idx = 0;
            for (name, _, _) in buffer_views {
                if *name == "image" && image_bv_index.is_some() {
                    return idx;
                }
                if *name == "image2" && second_image_bv_index.is_some() {
                    return idx;
                }
                idx += 1;
            }
            0
        }
    }

    /// Create a minimal valid 2x2 PNG image (RGBA).
    pub fn create_test_png() -> Vec<u8> {
        let mut png_data = Vec::new();
        let encoder = image::codecs::png::PngEncoder::new(&mut png_data);
        use image::ImageEncoder;
        let pixels: Vec<u8> = vec![
            255, 0, 0, 255, // red
            0, 255, 0, 255, // green
            0, 0, 255, 255, // blue
            255, 255, 0, 255, // yellow
        ];
        encoder
            .write_image(&pixels, 2, 2, image::ExtendedColorType::Rgba8)
            .unwrap();
        png_data
    }

    /// Create a minimal valid 2x2 JPEG image (RGB, no alpha).
    pub fn create_test_jpeg() -> Vec<u8> {
        let mut jpeg_data = Vec::new();
        let encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut jpeg_data, 75);
        use image::ImageEncoder;
        let pixels: Vec<u8> = vec![
            255, 0, 0, // red
            0, 255, 0, // green
            0, 0, 255, // blue
            255, 255, 0, // yellow
        ];
        encoder
            .write_image(&pixels, 2, 2, image::ExtendedColorType::Rgb8)
            .unwrap();
        jpeg_data
    }

    /// Write GLB data to a temp file and return the path.
    pub fn write_glb_to_temp(glb_data: &[u8], name: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join("rc_sim_glb_tests");
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join(name);
        std::fs::write(&path, glb_data).unwrap();
        path
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::texture::{SamplerFilter, SamplerWrap};
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
    fn acro_glb_asset_loads_with_primitives() {
        let asset = load_glb_asset(acro_asset()).unwrap();
        assert!(!asset.primitives.is_empty());
        assert!(asset.total_vertex_count() > 0);
        assert!(asset.total_index_count() > 0);
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
    }

    // -----------------------------------------------------------------------
    // G1C FIX 7: GLB texture acceptance tests
    // -----------------------------------------------------------------------

    #[test]
    fn embedded_png_base_color_texture_loads() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_png.glb");
        let asset = load_glb_asset(&path).unwrap();
        assert!(!asset.primitives.is_empty());
        let mat = &asset.primitives[0].material;
        assert!(
            mat.base_color_texture.is_some(),
            "PNG texture should be loaded"
        );
        let tex = mat.base_color_texture.as_ref().unwrap();
        assert_eq!(tex.width, 2);
        assert_eq!(tex.height, 2);
        assert_eq!(tex.rgba8.len(), 2 * 2 * 4);
    }

    #[test]
    fn embedded_jpeg_base_color_texture_loads() {
        let jpeg = test_glb_builder::create_test_jpeg();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_jpeg_texture(jpeg)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_jpeg.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert!(
            mat.base_color_texture.is_some(),
            "JPEG texture should be loaded"
        );
        let tex = mat.base_color_texture.as_ref().unwrap();
        assert_eq!(tex.width, 2);
        assert_eq!(tex.height, 2);
    }

    #[test]
    fn malformed_image_data_produces_explicit_error() {
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(vec![0xFF, 0xD8, 0xFF, 0xE0])
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_malformed.glb");
        let result = load_glb_asset(&path);
        assert!(
            matches!(result, Err(GlbLoadError::MalformedImage { .. })),
            "expected MalformedImage, got {:?}",
            result.err()
        );
    }

    #[test]
    fn texcoord_0_texture_path_loads() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_texcoord0.glb");
        let asset = load_glb_asset(&path).unwrap();
        assert!(asset.primitives[0].material.base_color_texture.is_some());
    }

    #[test]
    fn base_color_factor_is_preserved() {
        let glb = test_glb_builder::GlbBuilder::new()
            .with_base_color_factor([0.8, 0.2, 0.1, 1.0])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_color_factor.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert!((mat.base_color_factor[0] - 0.8).abs() < 0.01);
        assert!((mat.base_color_factor[1] - 0.2).abs() < 0.01);
        assert!((mat.base_color_factor[2] - 0.1).abs() < 0.01);
    }

    #[test]
    fn material_alpha_is_preserved() {
        let glb = test_glb_builder::GlbBuilder::new()
            .with_base_color_factor([1.0, 0.5, 0.0, 0.42])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_alpha.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert!((mat.base_color_factor[3] - 0.42).abs() < 0.01);
    }

    #[test]
    fn sampler_repeat_mapping() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_sampler(10497, 10497, 9729, 9729)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_repeat.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert_eq!(mat.sampler_config.wrap_s, SamplerWrap::Repeat);
        assert_eq!(mat.sampler_config.wrap_t, SamplerWrap::Repeat);
    }

    #[test]
    fn sampler_clamp_to_edge_mapping() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_sampler(33071, 33071, 9729, 9729)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_clamp.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert_eq!(mat.sampler_config.wrap_s, SamplerWrap::ClampToEdge);
        assert_eq!(mat.sampler_config.wrap_t, SamplerWrap::ClampToEdge);
    }

    #[test]
    fn sampler_mirrored_repeat_mapping() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_sampler(33648, 33648, 9729, 9729)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_mirror.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert_eq!(mat.sampler_config.wrap_s, SamplerWrap::MirroredRepeat);
        assert_eq!(mat.sampler_config.wrap_t, SamplerWrap::MirroredRepeat);
    }

    #[test]
    fn sampler_nearest_filter_mapping() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_sampler(10497, 10497, 9728, 9728)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_nearest.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert_eq!(mat.sampler_config.min_filter, SamplerFilter::Nearest);
        assert_eq!(mat.sampler_config.mag_filter, SamplerFilter::Nearest);
    }

    #[test]
    fn sampler_linear_filter_mapping() {
        let png = test_glb_builder::create_test_png();
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_sampler(10497, 10497, 9729, 9729)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_linear.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert_eq!(mat.sampler_config.min_filter, SamplerFilter::Linear);
        assert_eq!(mat.sampler_config.mag_filter, SamplerFilter::Linear);
    }

    #[test]
    fn no_texture_uses_no_base_color_texture() {
        let glb = test_glb_builder::GlbBuilder::new()
            .with_base_color_factor([0.5, 0.5, 0.5, 1.0])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_no_texture.glb");
        let asset = load_glb_asset(&path).unwrap();
        let mat = &asset.primitives[0].material;
        assert!(
            mat.base_color_texture.is_none(),
            "no texture should mean None"
        );
    }

    #[test]
    fn multi_primitive_preserves_distinct_materials() {
        let second = test_glb_builder::SecondPrimitive {
            positions: vec![[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0]],
            indices: vec![0, 1, 2],
            base_color_factor: Some([0.0, 1.0, 0.0, 1.0]),
            has_texture: false,
        };
        let glb = test_glb_builder::GlbBuilder::new()
            .with_base_color_factor([1.0, 0.0, 0.0, 1.0])
            .with_second_primitive(second)
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_multi_prim.glb");
        let asset = load_glb_asset(&path).unwrap();
        assert_eq!(asset.primitives.len(), 2, "should have 2 primitives");

        let mat0 = &asset.primitives[0].material;
        let mat1 = &asset.primitives[1].material;

        assert!(
            (mat0.base_color_factor[0] - 1.0).abs() < 0.01,
            "first prim should be red"
        );
        assert!((mat0.base_color_factor[1]).abs() < 0.01);
        assert!(
            (mat1.base_color_factor[1] - 1.0).abs() < 0.01,
            "second prim should be green"
        );
        assert!((mat1.base_color_factor[0]).abs() < 0.01);
    }

    #[test]
    fn multi_primitive_multi_texture_correctly_assigned() {
        let png1 = test_glb_builder::create_test_png();
        let png2 = test_glb_builder::create_test_png();
        let second = test_glb_builder::SecondPrimitive {
            positions: vec![[2.0, 0.0, 0.0], [3.0, 0.0, 0.0], [2.0, 1.0, 0.0]],
            indices: vec![0, 1, 2],
            base_color_factor: Some([0.0, 1.0, 0.0, 1.0]),
            has_texture: true,
        };
        let glb = test_glb_builder::GlbBuilder::new()
            .with_png_texture(png1)
            .with_tex_coords(vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
            .with_base_color_factor([1.0, 0.0, 0.0, 1.0])
            .with_second_primitive(second)
            .with_second_image(png2, "image/png")
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_multi_tex.glb");
        let asset = load_glb_asset(&path).unwrap();
        assert_eq!(asset.primitives.len(), 2);

        // First primitive should have texture 0.
        assert!(
            asset.primitives[0].material.base_color_texture.is_some(),
            "first primitive should have a texture"
        );
        // Second primitive should have texture 1.
        assert!(
            asset.primitives[1].material.base_color_texture.is_some(),
            "second primitive should have a texture"
        );
    }

    #[test]
    fn vertex_color_contains_base_color_factor_times_vertex_color() {
        let glb = test_glb_builder::GlbBuilder::new()
            .with_base_color_factor([0.8, 0.6, 0.4, 1.0])
            .with_colors(vec![
                [0.5, 0.5, 0.5, 1.0],
                [1.0, 1.0, 1.0, 1.0],
                [0.0, 0.0, 0.0, 1.0],
            ])
            .build();
        let path = test_glb_builder::write_glb_to_temp(&glb, "test_vertex_color.glb");
        let asset = load_glb_asset(&path).unwrap();
        let verts = &asset.primitives[0].vertices;

        // Vertex 0: 0.8 * 0.5 = 0.4
        assert!((verts[0].color[0] - 0.4).abs() < 0.01);
        // Vertex 1: 0.8 * 1.0 = 0.8
        assert!((verts[1].color[0] - 0.8).abs() < 0.01);
        // Vertex 2: 0.8 * 0.0 = 0.0
        assert!((verts[2].color[0]).abs() < 0.01);
    }
}
