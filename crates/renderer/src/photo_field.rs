//! PF1: Photo Field presentation path (manifest, calibration and equirect math).
//!
//! A Photo Field replaces the distant 3D world with a real photographic 360°
//! equirectangular panorama while the aircraft stays fully 3D, physically
//! simulated and dynamically lit. Everything in this module is
//! presentation-only: no type here is visible to `sim_core`, the flight
//! dynamics, the aerodynamic model, the propulsion model, the controls or the
//! replay physics state, and the renderer crate has no dependency on any of
//! them (see `tests/dependency_boundary.rs`).
//!
//! # Fixed pilot contract
//!
//! PhotoField only operates with [`CameraConfig::Pilot`]. The pilot eye is the
//! manifest's `pilot_position_render_m` and never translates; the camera may
//! rotate to track the aircraft and may change FOV because the panorama is
//! spherical. [`fixed_pilot_eye`] rejects any other camera cleanly instead of
//! silently allowing a translating eye.
//!
//! # Equirectangular convention
//!
//! The committed panorama stores the zenith on its FIRST row, so
//!
//! ```text
//! u = fract(azimuth / 2pi)          azimuth  = atan2(dir.z, dir.x) of the
//! v = 0.5 - elevation / pi          panorama-space direction, elevation =
//!                                   asin(dir.y), v = 0 at the top row
//! ```
//!
//! with `panorama_yaw_deg` / `panorama_pitch_deg` applied as a rigid rotation
//! of the world direction into panorama space (yaw about +Y, then pitch about
//! the rotated +X). Horizontal wrap is seamless because `u` is taken modulo 1
//! and the runtime sampler wraps on U; the vertical coordinate clamps at the
//! poles.

use serde::Deserialize;
use thiserror::Error;

/// The only manifest schema this renderer understands.
pub const PHOTO_FIELD_MANIFEST_SCHEMA_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// PF1: embedded meadow Photo Field assets
// ---------------------------------------------------------------------------

/// The committed meadow manifest, embedded so the runtime can never drift from
/// the versioned asset directory or read an unvalidated file from disk.
pub const PHOTO_FIELD_MANIFEST_JSON: &[u8] =
    include_bytes!("../assets/photofield/meadow/photo_field_manifest.json");

/// The committed 8192x4096 sRGB equirectangular meadow panorama.
pub const PHOTO_FIELD_PANORAMA_JPEG: &[u8] =
    include_bytes!("../assets/photofield/meadow/meadow_panorama_8192x4096.jpg");

/// The committed coarse depth-proxy GLB (invisible photographic stand-ins).
pub const PHOTO_FIELD_DEPTH_PROXY_GLB: &[u8] =
    include_bytes!("../assets/photofield/meadow/photo_field_depth.glb");

/// File name the manifest's `panorama` field must carry.
///
/// The manifest is the human-editable contract and the embedded bytes are the
/// machine contract; asserting they agree means a renamed asset fails closed
/// at startup instead of silently shading with the wrong photograph.
pub const PHOTO_FIELD_PANORAMA_FILE_NAME: &str = "meadow_panorama_8192x4096.jpg";

/// File name the manifest's `depth_proxy` field must carry.
pub const PHOTO_FIELD_DEPTH_PROXY_FILE_NAME: &str = "photo_field_depth.glb";

/// Exact committed panorama width in texels.
///
/// The equirectangular mapping and the wrap-on-U mip chain are only correct for
/// the surveyed power-of-two 2:1 panorama, so the GPU path rejects any other
/// decode instead of silently resampling a mismatched photograph.
pub const PHOTO_FIELD_PANORAMA_WIDTH: u32 = 8192;

/// Exact committed panorama height in texels.
pub const PHOTO_FIELD_PANORAMA_HEIGHT: u32 = 4096;

/// Instance node name of the depth-proxy ground receiver.
///
/// The proxy GLB partitions into exactly one photographic ground receiver —
/// the surface that catches the aircraft shadow — and every other node, which
/// are occluders only. Partitioning by authored name keeps the split stable
/// across asset re-exports and makes a missing ground plane fail closed.
pub const PHOTO_FIELD_GROUND_NODE_NAME: &str = "pf1_ground";

/// Maximum accepted magnitude of the panorama calibration angles, in degrees.
///
/// A photographic calibration never needs more than a full turn of yaw or a
/// quarter turn of pitch; the bound exists so a malformed manifest fails
/// closed instead of producing a silently rotated world.
const PANORAMA_YAW_DEG_LIMIT: f32 = 360.0;
const PANORAMA_PITCH_DEG_LIMIT: f32 = 90.0;

/// The committed, versioned Photo Field manifest.
///
/// Deliberately small: PF1 needs ONE working photographic field, not a general
/// asset compiler. Units are explicit — positions in metres, angles in
/// degrees, `sun_intensity` in the same scene-referred irradiance scale the
/// renderer's `SunState` consumes, `sun_rgb` a unit-less chromaticity.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhotoFieldManifest {
    pub schema_version: u32,
    pub id: String,
    /// File name of the runtime equirectangular panorama, relative to the
    /// embedded Photo Field asset directory.
    pub panorama: String,
    /// File name of the coarse depth-proxy GLB, same directory.
    pub depth_proxy: String,
    /// The fixed pilot eye in render metres. PhotoField never translates it.
    pub pilot_position_render_m: [f32; 3],
    pub panorama_yaw_deg: f32,
    pub panorama_pitch_deg: f32,
    /// Unit direction *towards* the photographed sun, in render world space.
    pub sun_direction_render: [f32; 3],
    pub sun_intensity: f32,
    pub sun_rgb: [f32; 3],
    /// Attenuation depth of the photographic aircraft ground shadow, in [0, 1].
    pub shadow_strength: f32,
}

/// The validated, unit-converted Photo Field configuration the renderer uses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PhotoFieldConfig {
    pub pilot_position_render_m: [f32; 3],
    pub panorama_yaw_rad: f32,
    pub panorama_pitch_rad: f32,
    /// Normalised direction towards the photographed sun.
    pub sun_direction_render: [f32; 3],
    pub sun_intensity: f32,
    pub sun_rgb: [f32; 3],
    pub shadow_strength: f32,
}

/// Every way a Photo Field manifest can be rejected, failing closed.
#[derive(Debug, Error, PartialEq)]
pub enum PhotoFieldManifestError {
    #[error("photo field manifest schema_version must be {expected}, got {actual}")]
    UnsupportedSchemaVersion { expected: u32, actual: u32 },
    #[error("embedded photo field manifest is not valid manifest JSON: {0}")]
    MalformedManifestJson(String),
    #[error(
        "photo field manifest asset reference `{field}` must name the embedded \
         asset {expected:?}, got {actual:?}"
    )]
    AssetNameMismatch {
        field: &'static str,
        expected: &'static str,
        actual: String,
    },
    #[error("photo field manifest field `{field}` must not be empty")]
    EmptyField { field: &'static str },
    #[error(
        "photo field manifest asset reference `{field}` = {value:?} must be a \
         single relative file name (no separators, no parent references)"
    )]
    InvalidAssetReference { field: &'static str, value: String },
    #[error("photo field manifest field `{field}` must be finite")]
    NotFinite { field: &'static str },
    #[error("photo field manifest sun_direction_render must not be the zero vector")]
    ZeroSunDirection,
    #[error("photo field manifest `{field}` must lie in [{min}, {max}], got {value}")]
    OutOfRange {
        field: &'static str,
        min: f32,
        max: f32,
        value: f32,
    },
}

/// Every way a camera can be incompatible with PhotoField, failing closed.
#[derive(Debug, Error, PartialEq)]
pub enum PhotoFieldCameraError {
    #[error(
        "PhotoField requires CameraConfig::Pilot with a fixed eye; a Chase \
         camera translates with the aircraft and would make the panorama swim"
    )]
    ChaseRejected,
    #[error(
        "PhotoField pilot eye {actual:?} does not equal the manifest pilot \
         position {expected:?}; the photographic perspective is only valid \
         from the surveyed eye"
    )]
    PilotPositionMismatch {
        expected: [f32; 3],
        actual: [f32; 3],
    },
}

impl PhotoFieldManifest {
    /// Validate the manifest and convert it to the runtime configuration.
    ///
    /// Nothing here touches the GPU: validation runs before the first texture
    /// or pipeline is created, so a bad manifest can never leave a half-built
    /// renderer behind.
    pub fn validate(&self) -> Result<PhotoFieldConfig, PhotoFieldManifestError> {
        if self.schema_version != PHOTO_FIELD_MANIFEST_SCHEMA_VERSION {
            return Err(PhotoFieldManifestError::UnsupportedSchemaVersion {
                expected: PHOTO_FIELD_MANIFEST_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.id.is_empty() {
            return Err(PhotoFieldManifestError::EmptyField { field: "id" });
        }
        validate_asset_reference("panorama", &self.panorama)?;
        validate_asset_reference("depth_proxy", &self.depth_proxy)?;
        for (index, component) in self.pilot_position_render_m.iter().enumerate() {
            if !component.is_finite() {
                return Err(PhotoFieldManifestError::NotFinite {
                    field: PILOT_POSITION_FIELDS[index],
                });
            }
        }
        let yaw = validate_angle(
            "panorama_yaw_deg",
            self.panorama_yaw_deg,
            PANORAMA_YAW_DEG_LIMIT,
        )?;
        let pitch = validate_angle(
            "panorama_pitch_deg",
            self.panorama_pitch_deg,
            PANORAMA_PITCH_DEG_LIMIT,
        )?;

        let length = (self.sun_direction_render[0] * self.sun_direction_render[0]
            + self.sun_direction_render[1] * self.sun_direction_render[1]
            + self.sun_direction_render[2] * self.sun_direction_render[2])
            .sqrt();
        if !length.is_finite() {
            return Err(PhotoFieldManifestError::NotFinite {
                field: "sun_direction_render",
            });
        }
        if length <= 1e-6 {
            return Err(PhotoFieldManifestError::ZeroSunDirection);
        }
        let sun_direction_render = [
            self.sun_direction_render[0] / length,
            self.sun_direction_render[1] / length,
            self.sun_direction_render[2] / length,
        ];
        if !self.sun_intensity.is_finite() || self.sun_intensity <= 0.0 {
            return Err(PhotoFieldManifestError::OutOfRange {
                field: "sun_intensity",
                min: 0.0,
                max: f32::INFINITY,
                value: self.sun_intensity,
            });
        }
        for (index, component) in self.sun_rgb.iter().enumerate() {
            if !component.is_finite() || *component < 0.0 {
                return Err(PhotoFieldManifestError::OutOfRange {
                    field: SUN_RGB_FIELDS[index],
                    min: 0.0,
                    max: f32::INFINITY,
                    value: *component,
                });
            }
        }
        if !self.shadow_strength.is_finite() || !(0.0..=1.0).contains(&self.shadow_strength) {
            return Err(PhotoFieldManifestError::OutOfRange {
                field: "shadow_strength",
                min: 0.0,
                max: 1.0,
                value: self.shadow_strength,
            });
        }

        Ok(PhotoFieldConfig {
            pilot_position_render_m: self.pilot_position_render_m,
            panorama_yaw_rad: yaw.to_radians(),
            panorama_pitch_rad: pitch.to_radians(),
            sun_direction_render,
            sun_intensity: self.sun_intensity,
            sun_rgb: self.sun_rgb,
            shadow_strength: self.shadow_strength,
        })
    }
}

const PILOT_POSITION_FIELDS: [&str; 3] = [
    "pilot_position_render_m[0]",
    "pilot_position_render_m[1]",
    "pilot_position_render_m[2]",
];
const SUN_RGB_FIELDS: [&str; 3] = ["sun_rgb[0]", "sun_rgb[1]", "sun_rgb[2]"];

fn validate_asset_reference(
    field: &'static str,
    value: &str,
) -> Result<(), PhotoFieldManifestError> {
    if value.is_empty() {
        return Err(PhotoFieldManifestError::EmptyField { field });
    }
    let single_segment = !value.contains('/')
        && !value.contains('\\')
        && value != "."
        && value != ".."
        && !value.starts_with('.');
    if !single_segment {
        return Err(PhotoFieldManifestError::InvalidAssetReference {
            field,
            value: value.to_string(),
        });
    }
    Ok(())
}

fn validate_angle(
    field: &'static str,
    value: f32,
    limit: f32,
) -> Result<f32, PhotoFieldManifestError> {
    if !value.is_finite() {
        return Err(PhotoFieldManifestError::NotFinite { field });
    }
    if !(-limit..=limit).contains(&value) {
        return Err(PhotoFieldManifestError::OutOfRange {
            field,
            min: -limit,
            max: limit,
            value,
        });
    }
    Ok(value)
}

/// The fixed pilot eye PhotoField will render from, or a clean rejection.
///
/// PhotoField is only meaningful from the surveyed photographic eye: any
/// translation would make the panorama swim against the aircraft, so a Chase
/// camera or a mismatched pilot position is an error, never a fallback.
pub fn fixed_pilot_eye(
    camera: &crate::camera::CameraConfig,
    config: &PhotoFieldConfig,
) -> Result<[f32; 3], PhotoFieldCameraError> {
    match camera {
        crate::camera::CameraConfig::Pilot {
            position_render_m,
            vertical_fov_deg: _,
        } => {
            if *position_render_m == config.pilot_position_render_m {
                Ok(*position_render_m)
            } else {
                Err(PhotoFieldCameraError::PilotPositionMismatch {
                    expected: config.pilot_position_render_m,
                    actual: *position_render_m,
                })
            }
        }
        crate::camera::CameraConfig::Chase { .. } => Err(PhotoFieldCameraError::ChaseRejected),
    }
}

/// Check that a manifest names exactly the assets this binary embeds.
///
/// Runs BEFORE `validate()` so a manifest that points at a file this binary
/// does not contain is reported as an asset mismatch rather than passing
/// validation and then shading with the wrong photograph.
fn check_embedded_asset_names(
    manifest: &PhotoFieldManifest,
) -> Result<(), PhotoFieldManifestError> {
    for (field, actual, expected) in [
        (
            "panorama",
            manifest.panorama.as_str(),
            PHOTO_FIELD_PANORAMA_FILE_NAME,
        ),
        (
            "depth_proxy",
            manifest.depth_proxy.as_str(),
            PHOTO_FIELD_DEPTH_PROXY_FILE_NAME,
        ),
    ] {
        if actual != expected {
            return Err(PhotoFieldManifestError::AssetNameMismatch {
                field,
                expected,
                actual: actual.to_string(),
            });
        }
    }
    Ok(())
}

fn embedded_photo_field_manifest() -> Result<PhotoFieldManifest, PhotoFieldManifestError> {
    let manifest: PhotoFieldManifest = serde_json::from_slice(PHOTO_FIELD_MANIFEST_JSON)
        .map_err(|source| PhotoFieldManifestError::MalformedManifestJson(source.to_string()))?;
    check_embedded_asset_names(&manifest)?;
    Ok(manifest)
}

/// The validated runtime configuration of the embedded meadow Photo Field.
///
/// Fails closed on a malformed manifest, a manifest that names assets this
/// binary does not embed, or any out-of-range calibration value.
///
/// # Errors
///
/// Returns the manifest rejection that applies, never a partial configuration.
pub fn embedded_photo_field_config() -> Result<PhotoFieldConfig, PhotoFieldManifestError> {
    embedded_photo_field_manifest()?.validate()
}

/// The surveyed pilot eye the embedded meadow Photo Field must be rendered from.
///
/// The presentation CLI defaults `--scenery photo-field` to this eye so the
/// photographic perspective is correct without the operator transcribing three
/// floats; an explicit `--pilot-position` still wins and is then checked
/// against the manifest by [`fixed_pilot_eye`].
///
/// # Errors
///
/// Propagates the embedded manifest rejection.
pub fn photo_field_default_pilot_position() -> Result<[f32; 3], PhotoFieldManifestError> {
    Ok(embedded_photo_field_config()?.pilot_position_render_m)
}

/// Equirectangular UV of a world-space direction under the PF1 convention.
///
/// `direction` need not be normalised; the zero vector maps to the panorama
/// centre rather than producing a NaN that would poison a whole frame.
#[must_use]
pub fn equirect_uv_from_direction(direction: [f32; 3], yaw_rad: f32, pitch_rad: f32) -> [f32; 2] {
    let length =
        (direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2])
            .sqrt();
    let dir = if length > 1e-9 {
        [
            direction[0] / length,
            direction[1] / length,
            direction[2] / length,
        ]
    } else {
        [0.0, 1.0, 0.0]
    };

    // World -> panorama space: yaw about +Y, then pitch about the rotated +X.
    // The yaw is SUBTRACTED: a calibration of +yaw puts panorama longitude 0 on
    // world azimuth +yaw, so the world direction at that azimuth samples u = 0.
    let (sy, cy) = yaw_rad.sin_cos();
    let x1 = dir[0] * cy + dir[2] * sy;
    let z1 = -dir[0] * sy + dir[2] * cy;
    let (sp, cp) = pitch_rad.sin_cos();
    let y2 = dir[1] * cp + z1 * sp;
    let z2 = -dir[1] * sp + z1 * cp;

    let azimuth = z2.atan2(x1);
    let u = (azimuth / std::f32::consts::TAU).fract();
    // fract() of a negative azimuth is already in [0, 1); guard the exact -0.0.
    let u = if u < 0.0 { u + 1.0 } else { u };
    let elevation = y2.clamp(-1.0, 1.0).asin();
    let v = (0.5 - elevation / std::f32::consts::PI).clamp(0.0, 1.0);
    [u, v]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::CameraConfig;

    const EPS: f32 = 1e-5;

    fn manifest() -> PhotoFieldManifest {
        PhotoFieldManifest {
            schema_version: PHOTO_FIELD_MANIFEST_SCHEMA_VERSION,
            id: "pf1-meadow".to_string(),
            panorama: "meadow_panorama_8192x4096.jpg".to_string(),
            depth_proxy: "photo_field_depth.glb".to_string(),
            pilot_position_render_m: [0.0, 1.6, 0.0],
            panorama_yaw_deg: 0.0,
            panorama_pitch_deg: 0.0,
            sun_direction_render: [-0.320_32, 0.933_18, 0.162_99],
            sun_intensity: 2.6,
            sun_rgb: [1.0, 0.95, 0.85],
            shadow_strength: 0.45,
        }
    }

    /// A render pose from NED physics scalars, mirroring `camera.rs`'s helper.
    fn pose(translation_ned: [f64; 3], axis: [f64; 3], angle_rad: f64) -> crate::pose::RenderPose {
        let half = 0.5 * angle_rad;
        let (sin_half, cos_half) = half.sin_cos();
        crate::world_ned_pose_to_render(
            translation_ned,
            [
                cos_half,
                axis[0] * sin_half,
                axis[1] * sin_half,
                axis[2] * sin_half,
            ],
            [0.0; 3],
        )
        .expect("finite test pose")
    }

    fn approx_eq(actual: [f32; 2], expected: [f32; 2]) {
        assert!(
            (actual[0] - expected[0]).abs() < EPS && (actual[1] - expected[1]).abs() < EPS,
            "uv {actual:?} != expected {expected:?}"
        );
    }

    #[test]
    fn cardinal_directions_map_to_the_documented_uv() {
        // +X is azimuth 0 -> u 0; +Z azimuth 90 -> u 0.25; -X u 0.5; -Z u 0.75.
        approx_eq(
            equirect_uv_from_direction([1.0, 0.0, 0.0], 0.0, 0.0),
            [0.0, 0.5],
        );
        approx_eq(
            equirect_uv_from_direction([0.0, 0.0, 1.0], 0.0, 0.0),
            [0.25, 0.5],
        );
        approx_eq(
            equirect_uv_from_direction([-1.0, 0.0, 0.0], 0.0, 0.0),
            [0.5, 0.5],
        );
        approx_eq(
            equirect_uv_from_direction([0.0, 0.0, -1.0], 0.0, 0.0),
            [0.75, 0.5],
        );
    }

    #[test]
    fn poles_clamp_to_the_first_and_last_row() {
        approx_eq(
            equirect_uv_from_direction([0.0, 1.0, 0.0], 0.0, 0.0),
            [0.0, 0.0],
        );
        approx_eq(
            equirect_uv_from_direction([0.0, -1.0, 0.0], 0.0, 0.0),
            [0.0, 1.0],
        );
    }

    #[test]
    fn horizontal_wrap_is_seamless_across_the_u_meridian() {
        // u wraps where panorama longitude 0 lies (world azimuth 0 at zero yaw),
        // so the seam test straddles THAT meridian: 0.2 deg either side must land
        // 0.2/360 from the 0/1 boundary, on opposite sides of it. The sampler
        // wraps on U, so a view crossing the meridian sees no discontinuity.
        let step = 0.2 / 360.0;
        let just_before = equirect_uv_from_direction(
            [
                (-0.2_f32).to_radians().cos(),
                0.0,
                (-0.2_f32).to_radians().sin(),
            ],
            0.0,
            0.0,
        );
        let just_after = equirect_uv_from_direction(
            [0.2_f32.to_radians().cos(), 0.0, 0.2_f32.to_radians().sin()],
            0.0,
            0.0,
        );
        assert!(just_before[0] > 1.0 - step * 2.0, "u {just_before:?}");
        assert!(just_after[0] < step * 2.0, "u {just_after:?}");
        assert!(((1.0 - just_before[0]) + just_after[0] - 2.0 * step).abs() < EPS);
    }

    #[test]
    fn yaw_calibration_rotates_the_panorama_about_the_vertical_axis() {
        let yaw = 30.0_f32.to_radians();
        // The world direction at azimuth 30 deg must land on u = 0.
        let dir = [yaw.cos(), 0.0, yaw.sin()];
        approx_eq(equirect_uv_from_direction(dir, yaw, 0.0), [0.0, 0.5]);
        // ...and azimuth 120 deg (90 deg further) on u = 0.25.
        let dir = [
            (yaw + 90.0_f32.to_radians()).cos(),
            0.0,
            (yaw + 90.0_f32.to_radians()).sin(),
        ];
        approx_eq(equirect_uv_from_direction(dir, yaw, 0.0), [0.25, 0.5]);
    }

    #[test]
    fn pitch_calibration_is_a_rigid_rotation_of_the_sampling_frame() {
        let pitch = 12.0_f32.to_radians();
        // The panorama horizon at panorama azimuth 90 deg is the panorama-space
        // direction (0,0,1); tilted by +pitch about X it is the world direction
        // (0, -sin p, cos p), and must still sample at v = 0.5.
        let world = [0.0, -pitch.sin(), pitch.cos()];
        let uv = equirect_uv_from_direction(world, 0.0, pitch);
        assert!((uv[1] - 0.5).abs() < EPS, "v {uv:?}");
        assert!((uv[0] - 0.25).abs() < EPS, "u {uv:?}");
    }

    #[test]
    fn the_measured_sun_direction_maps_onto_its_measured_panorama_texel() {
        // Offline analysis of meadow_8k.hdr put the solar disc at panorama
        // longitude 153.027 deg and elevation +68.936 deg; with a zero
        // calibration the manifest sun direction must reproduce exactly that
        // texel, which ties the Rust mapping to the photographic source.
        let config = manifest().validate().expect("valid manifest");
        let uv = equirect_uv_from_direction(config.sun_direction_render, 0.0, 0.0);
        // The manifest stores the direction at 5 decimal places, so the round
        // trip costs ~1e-5 of a UV; 1e-4 keeps the assertion meaningful.
        assert!((uv[0] - 153.027 / 360.0).abs() < 1e-4, "u {uv:?}");
        assert!((uv[1] - (0.5 - 68.936 / 180.0)).abs() < 1e-4, "v {uv:?}");
    }

    #[test]
    fn unnormalised_and_zero_directions_stay_finite() {
        let uv = equirect_uv_from_direction([0.0, 0.0, 0.0], 0.0, 0.0);
        assert!(uv[0].is_finite() && uv[1].is_finite());
        let scaled = equirect_uv_from_direction([3.0, 0.0, 0.0], 0.0, 0.0);
        approx_eq(scaled, [0.0, 0.5]);
    }

    #[test]
    fn manifest_round_trips_and_validates_deterministically() {
        let json = r#"{
            "schema_version": 1,
            "id": "pf1-meadow",
            "panorama": "meadow_panorama_8192x4096.jpg",
            "depth_proxy": "photo_field_depth.glb",
            "pilot_position_render_m": [0.0, 1.6, 0.0],
            "panorama_yaw_deg": 0.0,
            "panorama_pitch_deg": 0.0,
            "sun_direction_render": [-0.32032, 0.93318, 0.16299],
            "sun_intensity": 2.6,
            "sun_rgb": [1.0, 0.95, 0.85],
            "shadow_strength": 0.45
        }"#;
        let parsed: PhotoFieldManifest = serde_json::from_str(json).expect("parses");
        assert_eq!(parsed, manifest(), "the committed manifest text must match");
        let first = parsed.validate().expect("valid");
        let second = parsed.validate().expect("valid");
        assert_eq!(first, second, "validation must be a pure function");
        assert!(
            (first.sun_direction_render[0].powi(2)
                + first.sun_direction_render[1].powi(2)
                + first.sun_direction_render[2].powi(2)
                - 1.0)
                .abs()
                < 1e-6
        );
    }

    #[test]
    fn schema_version_is_rejected_when_unknown() {
        let mut bad = manifest();
        bad.schema_version = PHOTO_FIELD_MANIFEST_SCHEMA_VERSION + 1;
        assert_eq!(
            bad.validate(),
            Err(PhotoFieldManifestError::UnsupportedSchemaVersion {
                expected: PHOTO_FIELD_MANIFEST_SCHEMA_VERSION,
                actual: PHOTO_FIELD_MANIFEST_SCHEMA_VERSION + 1,
            })
        );
    }

    #[test]
    fn unknown_manifest_keys_are_rejected() {
        let json = r#"{
            "schema_version": 1, "id": "x", "panorama": "a.jpg", "depth_proxy": "b.glb",
            "pilot_position_render_m": [0.0, 1.6, 0.0], "panorama_yaw_deg": 0.0,
            "panorama_pitch_deg": 0.0, "sun_direction_render": [0.0, 1.0, 0.0],
            "sun_intensity": 1.0, "sun_rgb": [1.0, 1.0, 1.0], "shadow_strength": 0.5,
            "surprise": 42
        }"#;
        assert!(serde_json::from_str::<PhotoFieldManifest>(json).is_err());
    }

    #[test]
    fn asset_references_must_be_single_relative_file_names() {
        for bad in ["", "../escape.jpg", "sub/a.jpg", ".hidden.jpg", "a\\b.glb"] {
            let mut m = manifest();
            m.panorama = bad.to_string();
            assert!(
                matches!(
                    m.validate(),
                    Err(PhotoFieldManifestError::EmptyField { .. })
                        | Err(PhotoFieldManifestError::InvalidAssetReference { .. })
                ),
                "panorama {bad:?} must be rejected"
            );
        }
    }

    #[test]
    fn non_finite_and_out_of_range_fields_are_rejected() {
        let mut m = manifest();
        m.pilot_position_render_m[1] = f32::NAN;
        assert!(matches!(
            m.validate(),
            Err(PhotoFieldManifestError::NotFinite { .. })
        ));

        let mut m = manifest();
        m.sun_direction_render = [0.0, 0.0, 0.0];
        assert_eq!(m.validate(), Err(PhotoFieldManifestError::ZeroSunDirection));

        let mut m = manifest();
        m.shadow_strength = 1.5;
        assert!(matches!(
            m.validate(),
            Err(PhotoFieldManifestError::OutOfRange { .. })
        ));

        let mut m = manifest();
        m.panorama_yaw_deg = 720.0;
        assert!(matches!(
            m.validate(),
            Err(PhotoFieldManifestError::OutOfRange { .. })
        ));

        let mut m = manifest();
        m.sun_intensity = -1.0;
        assert!(matches!(
            m.validate(),
            Err(PhotoFieldManifestError::OutOfRange { .. })
        ));
    }

    #[test]
    fn photo_field_rejects_a_chase_camera() {
        let config = manifest().validate().expect("valid");
        assert_eq!(
            fixed_pilot_eye(&CameraConfig::chase_default(), &config),
            Err(PhotoFieldCameraError::ChaseRejected)
        );
    }

    #[test]
    fn photo_field_accepts_only_the_manifest_pilot_eye() {
        let config = manifest().validate().expect("valid");
        let eye = fixed_pilot_eye(
            &CameraConfig::Pilot {
                position_render_m: config.pilot_position_render_m,
                vertical_fov_deg: 55.0,
            },
            &config,
        )
        .expect("manifest eye accepted");
        assert_eq!(eye, config.pilot_position_render_m);

        let moved = CameraConfig::Pilot {
            position_render_m: [0.0, 1.6, 5.0],
            vertical_fov_deg: 55.0,
        };
        assert_eq!(
            fixed_pilot_eye(&moved, &config),
            Err(PhotoFieldCameraError::PilotPositionMismatch {
                expected: config.pilot_position_render_m,
                actual: [0.0, 1.6, 5.0],
            })
        );
    }

    #[test]
    fn the_pilot_eye_never_translates_with_the_aircraft() {
        // The fixed-eye guarantee lives in PilotCamera; PhotoField must inherit
        // it bit-for-bit for many aircraft positions and attitudes.
        let config = manifest().validate().expect("valid");
        let camera = CameraConfig::Pilot {
            position_render_m: config.pilot_position_render_m,
            vertical_fov_deg: 55.0,
        }
        .build(1920, 1080);
        for translation_ned in [
            [0.0, 0.0, 0.0],
            [10.0, 25.0, -40.0],
            [-80.0, 120.0, 60.0],
            [300.0, 5.0, 300.0],
        ] {
            for (axis, angle) in [
                ([1.0, 0.0, 0.0], 0.0),
                ([1.0, 0.0, 0.0], 0.7),
                ([0.0, 0.0, 1.0], -1.2),
                ([0.0, 1.0, 0.0], std::f64::consts::PI),
            ] {
                let pose = pose(translation_ned, axis, angle);
                assert_eq!(
                    camera.eye_position(&pose),
                    config.pilot_position_render_m,
                    "the eye must not translate for pose {translation_ned:?} angle {angle}"
                );
            }
        }
    }

    // -----------------------------------------------------------------
    // PF1: embedded meadow asset contract
    // -----------------------------------------------------------------

    #[test]
    fn the_embedded_manifest_parses_validates_and_names_the_embedded_assets() {
        let config = embedded_photo_field_config().expect("the committed manifest must validate");
        assert!(config.pilot_position_render_m.iter().all(|c| c.is_finite()));
        assert!((0.0..=1.0).contains(&config.shadow_strength));
        // The calibration angles survive the degree -> radian conversion.
        assert!(config.panorama_yaw_rad.abs() <= PANORAMA_YAW_DEG_LIMIT.to_radians());
        assert!(config.panorama_pitch_rad.abs() <= PANORAMA_PITCH_DEG_LIMIT.to_radians());
        assert_eq!(
            photo_field_default_pilot_position().expect("manifest eye"),
            config.pilot_position_render_m
        );
    }

    #[test]
    fn an_asset_name_this_binary_does_not_embed_is_rejected_before_validation() {
        // `validate()` only checks that a reference is a single relative file
        // name; it cannot see the embedded bytes. The name check is what makes
        // a renamed asset fail closed instead of shading the wrong photograph.
        let embedded = manifest();
        assert_eq!(check_embedded_asset_names(&embedded), Ok(()));

        for (field, expected, actual) in [
            (
                "panorama",
                PHOTO_FIELD_PANORAMA_FILE_NAME,
                "some_other_panorama.jpg",
            ),
            (
                "depth_proxy",
                PHOTO_FIELD_DEPTH_PROXY_FILE_NAME,
                "some_other_proxy.glb",
            ),
        ] {
            let mut renamed = embedded.clone();
            if field == "panorama" {
                renamed.panorama = actual.to_string();
            } else {
                renamed.depth_proxy = actual.to_string();
            }
            assert_eq!(
                check_embedded_asset_names(&renamed),
                Err(PhotoFieldManifestError::AssetNameMismatch {
                    field,
                    expected,
                    actual: actual.to_string(),
                }),
                "field {field}"
            );
        }
    }

    #[test]
    fn malformed_manifest_json_is_reported_as_a_typed_error() {
        let parsed = serde_json::from_slice::<PhotoFieldManifest>(b"{ not json");
        let error = parsed
            .map_err(|source| PhotoFieldManifestError::MalformedManifestJson(source.to_string()))
            .expect_err("truncated JSON must not parse");
        assert!(
            matches!(error, PhotoFieldManifestError::MalformedManifestJson(_)),
            "got {error:?}"
        );
    }

    #[test]
    fn the_embedded_panorama_decodes_to_the_surveyed_equirectangular_extent() {
        // 2:1 and power-of-two are what make the wrap-on-U mip chain and the
        // equirectangular texel mapping exact; any other decode must fail here.
        let decoded = crate::texture::decode_image(PHOTO_FIELD_PANORAMA_JPEG)
            .expect("the committed panorama must decode");
        assert_eq!(decoded.width, PHOTO_FIELD_PANORAMA_WIDTH);
        assert_eq!(decoded.height, PHOTO_FIELD_PANORAMA_HEIGHT);
        assert_eq!(
            decoded.rgba8.len(),
            (PHOTO_FIELD_PANORAMA_WIDTH * PHOTO_FIELD_PANORAMA_HEIGHT * 4) as usize
        );
    }

    #[test]
    fn the_embedded_depth_proxy_partitions_into_a_ground_receiver_and_occluders() {
        let asset = crate::glb::load_glb_bytes(PHOTO_FIELD_DEPTH_PROXY_GLB, "photo_field_depth")
            .expect("the committed depth proxy must load");
        let names: Vec<&str> = asset
            .instances
            .iter()
            .filter_map(|instance| instance.node_name.as_deref())
            .collect();
        assert!(
            names.contains(&PHOTO_FIELD_GROUND_NODE_NAME),
            "the proxy scene must carry the ground receiver, got {names:?}"
        );
        assert!(
            names
                .iter()
                .any(|name| *name != PHOTO_FIELD_GROUND_NODE_NAME),
            "a proxy scene with no occluder cannot hide the aircraft, got {names:?}"
        );
        assert!(
            asset
                .instances
                .iter()
                .all(|instance| instance.world_transform.is_finite()),
            "every baked proxy transform must be finite"
        );
    }
}
