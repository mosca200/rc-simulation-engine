use crate::Mat4;
use thiserror::Error;

const NED_TO_RENDER: [[f64; 3]; 3] = [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum RenderDataError {
    #[error("render pose position or origin contains a non-finite scalar")]
    NonFinitePosition,
    #[error("render-relative position cannot be represented as finite f32 values")]
    PositionOutsideF32Range,
    #[error("render pose orientation must be a finite non-zero Hamilton quaternion")]
    InvalidOrientation,
}

/// Render-only relative translation and active render-body-to-render-world rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderPose {
    translation_render_m: [f32; 3],
    rotation_render_world_from_render_body: [[f32; 3]; 3],
}

impl RenderPose {
    #[must_use]
    pub const fn translation_render_m(&self) -> [f32; 3] {
        self.translation_render_m
    }

    #[must_use]
    pub const fn rotation_render_world_from_render_body(&self) -> &[[f32; 3]; 3] {
        &self.rotation_render_world_from_render_body
    }

    #[must_use]
    pub fn model_matrix(&self) -> Mat4 {
        let rotation = self.rotation_render_world_from_render_body;
        let [x, y, z] = self.translation_render_m;
        Mat4::from_rows([
            [rotation[0][0], rotation[0][1], rotation[0][2], x],
            [rotation[1][0], rotation[1][1], rotation[1][2], y],
            [rotation[2][0], rotation[2][1], rotation[2][2], z],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    #[must_use]
    pub fn transform_direction(&self, direction_render_body: [f32; 3]) -> [f32; 3] {
        let rotation = self.rotation_render_world_from_render_body;
        [
            rotation[0][0] * direction_render_body[0]
                + rotation[0][1] * direction_render_body[1]
                + rotation[0][2] * direction_render_body[2],
            rotation[1][0] * direction_render_body[0]
                + rotation[1][1] * direction_render_body[1]
                + rotation[1][2] * direction_render_body[2],
            rotation[2][0] * direction_render_body[0]
                + rotation[2][1] * direction_render_body[1]
                + rotation[2][2] * direction_render_body[2],
        ]
    }
}

/// Latest committed pose and no simulation-owned state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RenderFrame {
    aircraft_pose: RenderPose,
}

impl RenderFrame {
    #[must_use]
    pub const fn new(aircraft_pose: RenderPose) -> Self {
        Self { aircraft_pose }
    }

    #[must_use]
    pub const fn aircraft_pose(&self) -> &RenderPose {
        &self.aircraft_pose
    }
}

/// Converts raw physics scalars after subtracting the render origin in `f64`.
pub fn world_ned_pose_to_render(
    position_world_ned_m: [f64; 3],
    orientation_world_from_body_wxyz: [f64; 4],
    render_origin_world_ned_m: [f64; 3],
) -> Result<RenderPose, RenderDataError> {
    if !position_world_ned_m.into_iter().all(f64::is_finite)
        || !render_origin_world_ned_m.into_iter().all(f64::is_finite)
    {
        return Err(RenderDataError::NonFinitePosition);
    }

    let relative_position_ned_m = [
        position_world_ned_m[0] - render_origin_world_ned_m[0],
        position_world_ned_m[1] - render_origin_world_ned_m[1],
        position_world_ned_m[2] - render_origin_world_ned_m[2],
    ];
    let relative_position_render_m = multiply_matrix_vector(NED_TO_RENDER, relative_position_ned_m);
    let translation_render_m = relative_position_render_m.map(|value| value as f32);
    if !translation_render_m.into_iter().all(f32::is_finite) {
        return Err(RenderDataError::PositionOutsideF32Range);
    }

    let rotation_ned_world_from_frd_body =
        quaternion_to_rotation(orientation_world_from_body_wxyz)?;
    let rotation_render_world_from_render_body = multiply_matrices(
        multiply_matrices(NED_TO_RENDER, rotation_ned_world_from_frd_body),
        transpose(NED_TO_RENDER),
    );

    Ok(RenderPose {
        translation_render_m,
        rotation_render_world_from_render_body: rotation_render_world_from_render_body
            .map(|row| row.map(|value| value as f32)),
    })
}

fn quaternion_to_rotation(quaternion_wxyz: [f64; 4]) -> Result<[[f64; 3]; 3], RenderDataError> {
    if !quaternion_wxyz.into_iter().all(f64::is_finite) {
        return Err(RenderDataError::InvalidOrientation);
    }
    let norm_squared = quaternion_wxyz
        .into_iter()
        .map(|value| value * value)
        .sum::<f64>();
    if !norm_squared.is_finite() || norm_squared <= f64::EPSILON {
        return Err(RenderDataError::InvalidOrientation);
    }
    let inverse_norm = norm_squared.sqrt().recip();
    let [w, x, y, z] = quaternion_wxyz.map(|value| value * inverse_norm);
    Ok([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])
}

fn multiply_matrices(left: [[f64; 3]; 3], right: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut output = [[0.0; 3]; 3];
    for (row_index, row) in output.iter_mut().enumerate() {
        for (column_index, value) in row.iter_mut().enumerate() {
            *value = left[row_index][0] * right[0][column_index]
                + left[row_index][1] * right[1][column_index]
                + left[row_index][2] * right[2][column_index];
        }
    }
    output
}

fn multiply_matrix_vector(matrix: [[f64; 3]; 3], vector: [f64; 3]) -> [f64; 3] {
    matrix.map(|row| row[0] * vector[0] + row[1] * vector[1] + row[2] * vector[2])
}

fn transpose(matrix: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const IDENTITY_QUATERNION: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    fn pose(position: [f64; 3], quaternion: [f64; 4]) -> RenderPose {
        world_ned_pose_to_render(position, quaternion, [0.0; 3]).unwrap()
    }

    fn assert_vector_close(actual: [f32; 3], expected: [f32; 3]) {
        for (actual_value, expected_value) in actual.into_iter().zip(expected) {
            assert!((actual_value - expected_value).abs() < 2.0e-6);
        }
    }

    #[test]
    fn north_maps_to_render_negative_z() {
        assert_eq!(
            pose([1.0, 0.0, 0.0], IDENTITY_QUATERNION).translation_render_m(),
            [0.0, 0.0, -1.0]
        );
    }

    #[test]
    fn east_maps_to_render_positive_x() {
        assert_eq!(
            pose([0.0, 1.0, 0.0], IDENTITY_QUATERNION).translation_render_m(),
            [1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn down_maps_to_render_negative_y() {
        assert_eq!(
            pose([0.0, 0.0, 1.0], IDENTITY_QUATERNION).translation_render_m(),
            [0.0, -1.0, 0.0]
        );
    }

    #[test]
    fn identity_body_orientation_becomes_render_identity() {
        let pose = pose([0.0; 3], IDENTITY_QUATERNION);
        assert_eq!(
            pose.rotation_render_world_from_render_body(),
            &[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        );
    }

    #[test]
    fn body_forward_is_render_local_negative_z() {
        let pose = pose([0.0; 3], IDENTITY_QUATERNION);
        assert_eq!(pose.transform_direction([0.0, 0.0, -1.0]), [0.0, 0.0, -1.0]);
    }

    #[test]
    fn positive_ninety_degree_physical_yaw_turns_nose_from_north_to_east() {
        let half_sqrt_two = 0.5_f64.sqrt();
        let pose = pose([0.0; 3], [half_sqrt_two, 0.0, 0.0, half_sqrt_two]);
        assert_vector_close(pose.transform_direction([0.0, 0.0, -1.0]), [1.0, 0.0, 0.0]);
    }

    #[test]
    fn nontrivial_orientation_obeys_c_r_c_inverse() {
        let quaternion = [0.75, 0.25, -0.35, 0.5];
        let pose = pose([0.0; 3], quaternion);
        let render_body_vector = [0.3_f32, -0.4, 0.8];
        let body_frd_vector = [
            -f64::from(render_body_vector[2]),
            f64::from(render_body_vector[0]),
            -f64::from(render_body_vector[1]),
        ];
        let physical_rotation = quaternion_to_rotation(quaternion).unwrap();
        let expected = multiply_matrix_vector(
            NED_TO_RENDER,
            multiply_matrix_vector(physical_rotation, body_frd_vector),
        )
        .map(|value| value as f32);
        assert_vector_close(pose.transform_direction(render_body_vector), expected);
    }

    #[test]
    fn render_origin_is_subtracted_before_f32_cast() {
        let origin = [1.0e12, -1.0e12, 5.0e11];
        let position = [origin[0] + 0.25, origin[1] + 0.5, origin[2] - 0.75];
        let pose = world_ned_pose_to_render(position, IDENTITY_QUATERNION, origin).unwrap();
        assert_eq!(pose.translation_render_m(), [0.5, 0.75, -0.25]);
        assert_eq!(position[0] as f32, origin[0] as f32);
    }

    #[test]
    fn render_origin_is_applied_componentwise_before_frame_mapping() {
        let pose = world_ned_pose_to_render(
            [101.0, 202.0, 303.0],
            IDENTITY_QUATERNION,
            [100.0, 200.0, 300.0],
        )
        .unwrap();
        assert_eq!(pose.translation_render_m(), [2.0, -3.0, -1.0]);
    }

    #[test]
    fn repeated_conversion_is_bit_identical() {
        let first = world_ned_pose_to_render(
            [123_456.75, -92_001.5, 4_012.25],
            [0.75, 0.25, -0.35, 0.5],
            [123_450.0, -92_000.0, 4_000.0],
        )
        .unwrap();
        let second = world_ned_pose_to_render(
            [123_456.75, -92_001.5, 4_012.25],
            [0.75, 0.25, -0.35, 0.5],
            [123_450.0, -92_000.0, 4_000.0],
        )
        .unwrap();
        assert_eq!(
            first.translation_render_m().map(f32::to_bits),
            second.translation_render_m().map(f32::to_bits)
        );
        assert_eq!(
            first
                .rotation_render_world_from_render_body()
                .map(|row| row.map(f32::to_bits)),
            second
                .rotation_render_world_from_render_body()
                .map(|row| row.map(f32::to_bits))
        );
    }
}
