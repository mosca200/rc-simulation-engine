use std::ops::Mul;
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mat4 {
    rows: [[f32; 4]; 4],
}

impl Mat4 {
    #[must_use]
    pub const fn from_rows(rows: [[f32; 4]; 4]) -> Self {
        Self { rows }
    }

    #[must_use]
    pub const fn identity() -> Self {
        Self::from_rows([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    #[must_use]
    pub const fn rows(&self) -> &[[f32; 4]; 4] {
        &self.rows
    }

    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.rows.iter().flatten().all(|value| value.is_finite())
    }

    #[must_use]
    pub fn transform_homogeneous(&self, vector: [f32; 4]) -> [f32; 4] {
        self.rows.map(|row| dot4(row, vector))
    }

    /// General 4×4 matrix inverse via cofactor expansion.
    /// Returns `None` if the matrix is singular (determinant ≈ 0).
    #[must_use]
    pub fn inverse(&self) -> Option<Self> {
        let r = &self.rows;
        // Shorthand: r[row][col].
        let m00 = r[0][0];
        let m01 = r[0][1];
        let m02 = r[0][2];
        let m03 = r[0][3];
        let m10 = r[1][0];
        let m11 = r[1][1];
        let m12 = r[1][2];
        let m13 = r[1][3];
        let m20 = r[2][0];
        let m21 = r[2][1];
        let m22 = r[2][2];
        let m23 = r[2][3];
        let m30 = r[3][0];
        let m31 = r[3][1];
        let m32 = r[3][2];
        let m33 = r[3][3];

        // Cofactors for the first two rows (used for determinant + adjugate).
        let c00 = m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31)
            + m13 * (m21 * m32 - m22 * m31);
        let c01 = -(m10 * (m22 * m33 - m23 * m32) - m12 * (m20 * m33 - m23 * m30)
            + m13 * (m20 * m32 - m22 * m30));
        let c02 = m10 * (m21 * m33 - m23 * m31) - m11 * (m20 * m33 - m23 * m30)
            + m13 * (m20 * m31 - m21 * m30);
        let c03 = -(m10 * (m21 * m32 - m22 * m31) - m11 * (m20 * m32 - m22 * m30)
            + m12 * (m20 * m31 - m21 * m30));

        let determinant = m00 * c00 + m01 * c01 + m02 * c02 + m03 * c03;
        if determinant.abs() < 1.0e-12 {
            return None;
        }
        let inv_det = determinant.recip();

        // Remaining cofactors.
        let c10 = -(m01 * (m22 * m33 - m23 * m32) - m02 * (m21 * m33 - m23 * m31)
            + m03 * (m21 * m32 - m22 * m31));
        let c11 = m00 * (m22 * m33 - m23 * m32) - m02 * (m20 * m33 - m23 * m30)
            + m03 * (m20 * m32 - m22 * m30);
        let c12 = -(m00 * (m21 * m33 - m23 * m31) - m01 * (m20 * m33 - m23 * m30)
            + m03 * (m20 * m31 - m21 * m30));
        let c13 = m00 * (m21 * m32 - m22 * m31) - m01 * (m20 * m32 - m22 * m30)
            + m02 * (m20 * m31 - m21 * m30);

        let c20 = m01 * (m12 * m33 - m13 * m32) - m02 * (m11 * m33 - m13 * m31)
            + m03 * (m11 * m32 - m12 * m31);
        let c21 = -(m00 * (m12 * m33 - m13 * m32) - m02 * (m10 * m33 - m13 * m30)
            + m03 * (m10 * m32 - m12 * m30));
        let c22 = m00 * (m11 * m33 - m13 * m31) - m01 * (m10 * m33 - m13 * m30)
            + m03 * (m10 * m31 - m11 * m30);
        let c23 = -(m00 * (m11 * m32 - m12 * m31) - m01 * (m10 * m32 - m12 * m30)
            + m02 * (m10 * m31 - m11 * m30));

        let c30 = -(m01 * (m12 * m23 - m13 * m22) - m02 * (m11 * m23 - m13 * m21)
            + m03 * (m11 * m22 - m12 * m21));
        let c31 = m00 * (m12 * m23 - m13 * m22) - m02 * (m10 * m23 - m13 * m20)
            + m03 * (m10 * m22 - m12 * m20);
        let c32 = -(m00 * (m11 * m23 - m13 * m21) - m01 * (m10 * m23 - m13 * m20)
            + m03 * (m10 * m21 - m11 * m20));
        let c33 = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20)
            + m02 * (m10 * m21 - m11 * m20);

        // Adjugate = transpose of cofactor matrix, then scale by 1/det.
        Some(Self::from_rows([
            [c00 * inv_det, c10 * inv_det, c20 * inv_det, c30 * inv_det],
            [c01 * inv_det, c11 * inv_det, c21 * inv_det, c31 * inv_det],
            [c02 * inv_det, c12 * inv_det, c22 * inv_det, c32 * inv_det],
            [c03 * inv_det, c13 * inv_det, c23 * inv_det, c33 * inv_det],
        ]))
    }
}

impl Mul for Mat4 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        let mut rows = [[0.0; 4]; 4];
        for (row_index, row) in rows.iter_mut().enumerate() {
            for (column_index, value) in row.iter_mut().enumerate() {
                *value = self.rows[row_index][0] * rhs.rows[0][column_index]
                    + self.rows[row_index][1] * rhs.rows[1][column_index]
                    + self.rows[row_index][2] * rhs.rows[2][column_index]
                    + self.rows[row_index][3] * rhs.rows[3][column_index];
            }
        }
        Self::from_rows(rows)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum ProjectionError {
    #[error("vertical field of view must be finite and inside (0, pi)")]
    InvalidVerticalFieldOfView,
    #[error("aspect ratio must be finite and positive")]
    InvalidAspectRatio,
    #[error("projection planes must satisfy 0 < near < far with finite values")]
    InvalidDepthRange,
}

/// Converts an explicitly row-major CPU matrix into four WGSL column vectors.
#[must_use]
pub fn matrix_to_wgsl_columns(matrix: &Mat4) -> [[f32; 4]; 4] {
    let rows = matrix.rows();
    [
        [rows[0][0], rows[1][0], rows[2][0], rows[3][0]],
        [rows[0][1], rows[1][1], rows[2][1], rows[3][1]],
        [rows[0][2], rows[1][2], rows[2][2], rows[3][2]],
        [rows[0][3], rows[1][3], rows[2][3], rows[3][3]],
    ]
}

/// Right-handed perspective matrix with the WebGPU clip-depth range `[0, 1]`.
pub fn webgpu_perspective(
    vertical_fov_rad: f32,
    aspect_ratio: f32,
    near_m: f32,
    far_m: f32,
) -> Result<Mat4, ProjectionError> {
    if !vertical_fov_rad.is_finite()
        || vertical_fov_rad <= 0.0
        || vertical_fov_rad >= std::f32::consts::PI
    {
        return Err(ProjectionError::InvalidVerticalFieldOfView);
    }
    if !aspect_ratio.is_finite() || aspect_ratio <= 0.0 {
        return Err(ProjectionError::InvalidAspectRatio);
    }
    if !near_m.is_finite() || !far_m.is_finite() || near_m <= 0.0 || far_m <= near_m {
        return Err(ProjectionError::InvalidDepthRange);
    }

    let focal_length = 1.0 / (vertical_fov_rad * 0.5).tan();
    let open_gl_projection = Mat4::from_rows([
        [focal_length / aspect_ratio, 0.0, 0.0, 0.0],
        [0.0, focal_length, 0.0, 0.0],
        [
            0.0,
            0.0,
            (far_m + near_m) / (near_m - far_m),
            (2.0 * far_m * near_m) / (near_m - far_m),
        ],
        [0.0, 0.0, -1.0, 0.0],
    ]);
    let open_gl_to_webgpu_depth = Mat4::from_rows([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 0.5, 0.5],
        [0.0, 0.0, 0.0, 1.0],
    ]);
    Ok(open_gl_to_webgpu_depth * open_gl_projection)
}

pub(crate) fn look_at_rh(eye: [f32; 3], target: [f32; 3], preferred_up: [f32; 3]) -> Mat4 {
    let forward = normalize3(sub3(target, eye));
    let up = if dot3(forward, preferred_up).abs() > 0.999 {
        [0.0, 0.0, 1.0]
    } else {
        preferred_up
    };
    let right = normalize3(cross3(forward, up));
    let camera_up = cross3(right, forward);
    Mat4::from_rows([
        [right[0], right[1], right[2], -dot3(right, eye)],
        [
            camera_up[0],
            camera_up[1],
            camera_up[2],
            -dot3(camera_up, eye),
        ],
        [-forward[0], -forward[1], -forward[2], dot3(forward, eye)],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

pub(crate) fn add3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

pub(crate) fn sub3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

pub(crate) fn scale3(vector: [f32; 3], scale: f32) -> [f32; 3] {
    [vector[0] * scale, vector[1] * scale, vector[2] * scale]
}

fn dot3(left: [f32; 3], right: [f32; 3]) -> f32 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn cross3(left: [f32; 3], right: [f32; 3]) -> [f32; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn normalize3(vector: [f32; 3]) -> [f32; 3] {
    let norm = dot3(vector, vector).sqrt();
    if norm <= f32::EPSILON {
        [0.0, 0.0, -1.0]
    } else {
        scale3(vector, norm.recip())
    }
}

fn dot4(left: [f32; 4], right: [f32; 4]) -> f32 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2] + left[3] * right[3]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wgsl_packing_is_explicitly_column_major() {
        let matrix = Mat4::from_rows([
            [1.0, 2.0, 3.0, 4.0],
            [5.0, 6.0, 7.0, 8.0],
            [9.0, 10.0, 11.0, 12.0],
            [13.0, 14.0, 15.0, 16.0],
        ]);
        assert_eq!(
            matrix_to_wgsl_columns(&matrix),
            [
                [1.0, 5.0, 9.0, 13.0],
                [2.0, 6.0, 10.0, 14.0],
                [3.0, 7.0, 11.0, 15.0],
                [4.0, 8.0, 12.0, 16.0],
            ]
        );
    }

    #[test]
    fn webgpu_projection_maps_near_to_zero_and_far_to_one() {
        let near = 0.05;
        let far = 2_000.0;
        let projection = webgpu_perspective(60.0_f32.to_radians(), 16.0 / 9.0, near, far).unwrap();
        let near_clip = projection.transform_homogeneous([0.0, 0.0, -near, 1.0]);
        let far_clip = projection.transform_homogeneous([0.0, 0.0, -far, 1.0]);
        assert!((near_clip[2] / near_clip[3]).abs() < 2.0e-6);
        assert!((far_clip[2] / far_clip[3] - 1.0).abs() < 2.0e-6);
        assert!(projection.is_finite());
    }
}
