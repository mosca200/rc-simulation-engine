use crate::{
    controller_app::{
        ControllerStatusTracker, controller_device_views, format_controller_transition,
        format_viewer_controller_status,
    },
    controller_profile_app::{
        CalibratedControllerState, ControllerProfileFileError, format_device_identity,
        load_controller_profile,
    },
    render_snapshot::{AircraftRenderSnapshot, AircraftRenderSnapshotBuffer, interpolation_alpha},
};
use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError, AircraftSnapshot,
};
use image::{ImageEncoder, codecs::png::PngEncoder};
use model::{
    AircraftModel, AircraftModelFingerprint, ModelLoadError, PresentationMetadata,
    PresentationSurface, load_aircraft_model,
};
use platform::{
    DeviceIdentity, GilrsInputBackend, InputDeviceInfo, InputError, InputMapping, InputSource,
    InputState, KeyboardInputState, KeyboardKey, RawControllerState,
};
use renderer::{
    AircraftMesh, CameraConfig, CaptureRenderOutcome, CapturedFrame, DEFAULT_EXPOSURE_EV,
    DesktopRenderer, ExposureError, FixedStepAccumulator, FixedStepAccumulatorError,
    FrameCaptureError, GlbArticulationError, GlbArticulationPlan, GlbAsset, GlbLoadError,
    PhotoFieldManifestError, PresentationAsset, RenderDataError, RenderOutcome, RenderTerrainMode,
    RendererError, RendererVersion, RuntimeVisualAudit, SurfaceError, SurfaceHinge, SurfaceId,
    TerrainDebugMode, VegetationDebugMode, aircraft_mesh, load_glb_asset,
    photo_field_default_pilot_position, rv2_6_validation_target_mesh, scenery::SceneryPreset,
    validate_exposure_ev,
};
use replay::{AircraftReplayError, AircraftReplayRecorder};
use serde::Serialize;
use sha2::{Digest, Sha256};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ,
    FlatGroundPlane, GroundCommand, GroundEvaluation, GroundSurface, PilotInput, RigidBodyState,
    SimulationConfigError, evaluate_ground_wrench,
};
use sim_math::{Orientation, Vec3};
use std::{
    fs,
    io::{self, Write},
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant},
};
use thiserror::Error;
use tracing::warn;
use winit::{
    application::ApplicationHandler,
    dpi::{LogicalSize, PhysicalSize},
    event::{ElementState, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{Key, KeyCode, NamedKey, PhysicalKey},
    window::{Window, WindowId},
};

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_THROTTLE: f64 = 0.55;
const DEFAULT_ALTITUDE_M: f64 = 30.0;
const DEFAULT_AIRSPEED_MPS: f64 = 18.0;
const PLAY_CHASE_DISTANCE_M: f32 = 3.0;
const PLAY_CHASE_HEIGHT_M: f32 = 0.95;
const EXPLICIT_PILOT_POSITION_RENDER_M: [f32; 3] = [0.0, 1.8, 20.0];
const EXPLICIT_CHASE_DISTANCE_M: f32 = 3.5;
const EXPLICIT_CHASE_HEIGHT_M: f32 = 1.25;
const EXPLICIT_CAMERA_FOV_DEG: f32 = 55.0;
const MAXIMUM_ALTITUDE_M: f64 = 10_000.0;
const MAXIMUM_AIRSPEED_MPS: f64 = 200.0;
const PHYSICS_DT: Duration = Duration::from_millis(2);
const MAXIMUM_FRAME_DELTA: Duration = Duration::from_millis(250);
const MAXIMUM_PHYSICS_STEPS_PER_FRAME: u32 = 16;
const DEFAULT_RENDER_WIDTH_LOGICAL: f64 = 1_280.0;
const DEFAULT_RENDER_HEIGHT_LOGICAL: f64 = 720.0;
const MINIMUM_RENDER_WIDTH: u32 = 320;
const MAXIMUM_RENDER_WIDTH: u32 = 7_680;
const MINIMUM_RENDER_HEIGHT: u32 = 240;
const MAXIMUM_RENDER_HEIGHT: u32 = 4_320;

#[derive(Debug, Clone)]
pub struct RenderOptions {
    model_path: PathBuf,
    throttle: f64,
    altitude_m: f64,
    airspeed_mps: f64,
    replay_output_path: Option<PathBuf>,
    controller_profile_path: Option<PathBuf>,
    start_on_ground: bool,
    scenery: SceneryPreset,
    camera: CameraSelection,
    debug_overlays: bool,
    // G3A-R: presentation-only terrain debug channel (FINAL by default).
    terrain_debug: TerrainDebugMode,
    // G3D: presentation-only vegetation debug channel (FINAL by default).
    vegetation_debug: VegetationDebugMode,
    // G3B: presentation-only manual exposure in EV stops (default outdoor).
    exposure_ev: f32,
    // RV2-1: rendering backend selection (`--renderer v1|v2`); defaults to V1
    // so every existing command keeps its historical behaviour.
    renderer: RendererVersion,
    // Developer-only controlled visual gate. None preserves production.
    rv2_6_validation: Option<Rv26ValidationConfig>,
    // VIS0-C1: physical pixels are opt-in; None preserves the historical
    // 1280x720 logical-size window and its platform DPI behavior.
    render_resolution: Option<RenderResolution>,
    // VIS0-C1: zero-based presentation frame after which the event loop exits.
    exit_after_frame: Option<u64>,
    // VIS0-C2A: complete one-shot display-frame capture configuration.
    capture: Option<CaptureConfig>,
    // VIS0-C2D: runner-owned, one-shot audit path. None is the historical path.
    visual_audit_out: Option<PathBuf>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CaptureFormat {
    Png,
}

impl CaptureFormat {
    const fn label(self) -> &'static str {
        match self {
            Self::Png => "png",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct CaptureConfig {
    presentation_frame_index: u64,
    image_path: PathBuf,
    format: CaptureFormat,
    receipt_path: Option<PathBuf>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
struct RuntimeCaptureReceipt {
    schema_version: &'static str,
    presentation_frame_index: u64,
    framebuffer_width: u32,
    framebuffer_height: u32,
    format: &'static str,
    image_path: String,
    image_sha256: String,
    image_byte_size: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RenderResolution {
    width: u32,
    height: u32,
}

impl RenderResolution {
    const fn new(width: u32, height: u32) -> Self {
        Self { width, height }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RenderResolutionState {
    Disabled,
    Requested(RenderResolution),
    Pending(RenderResolution),
    VerifyCurrent(RenderResolution),
    Verified(RenderResolution),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RenderResolutionRequestOutcome {
    Pending,
    Verified,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RenderResolutionMismatch {
    requested: RenderResolution,
    actual: RenderResolution,
}

impl RenderResolutionState {
    const fn new(requested: Option<RenderResolution>) -> Self {
        match requested {
            Some(resolution) => Self::Requested(resolution),
            None => Self::Disabled,
        }
    }

    const fn requested(self) -> Option<RenderResolution> {
        match self {
            Self::Disabled => None,
            Self::Requested(resolution)
            | Self::Pending(resolution)
            | Self::VerifyCurrent(resolution)
            | Self::Verified(resolution) => Some(resolution),
        }
    }

    const fn is_ready(self) -> bool {
        matches!(self, Self::Disabled | Self::Verified(_))
    }

    const fn request_is_needed(self) -> bool {
        matches!(self, Self::Requested(_))
    }

    const fn current_extent_verification_is_needed(self) -> bool {
        matches!(self, Self::VerifyCurrent(_))
    }

    fn begin_request(&mut self) {
        if let Some(requested) = self.requested() {
            *self = Self::Requested(requested);
        }
    }

    fn resolve_request(
        &mut self,
        actual: Option<RenderResolution>,
    ) -> Result<RenderResolutionRequestOutcome, RenderResolutionMismatch> {
        let requested = self
            .requested()
            .expect("resolution requests exist only when enforcement is enabled");
        match actual {
            Some(actual) if actual == requested => {
                *self = Self::Verified(requested);
                Ok(RenderResolutionRequestOutcome::Verified)
            }
            Some(actual) => Err(RenderResolutionMismatch { requested, actual }),
            None => {
                *self = Self::Pending(requested);
                Ok(RenderResolutionRequestOutcome::Pending)
            }
        }
    }

    fn await_current_extent_verification(&mut self) {
        if let Some(requested) = self.requested() {
            *self = Self::VerifyCurrent(requested);
        }
    }

    fn verify_current_extent(
        &mut self,
        actual: RenderResolution,
    ) -> Result<(), RenderResolutionMismatch> {
        let requested = match *self {
            Self::VerifyCurrent(requested) => requested,
            _ => return Ok(()),
        };
        if actual != requested {
            return Err(RenderResolutionMismatch { requested, actual });
        }
        *self = Self::Verified(requested);
        Ok(())
    }

    fn observe_resize(&mut self, actual: RenderResolution) -> Result<(), RenderResolutionMismatch> {
        let requested = match *self {
            Self::Disabled | Self::Requested(_) => return Ok(()),
            Self::Pending(requested)
            | Self::VerifyCurrent(requested)
            | Self::Verified(requested) => requested,
        };
        if actual != requested {
            return Err(RenderResolutionMismatch { requested, actual });
        }
        *self = Self::Verified(requested);
        Ok(())
    }
}

/// Presentation-only run control for deterministic frame scheduling.
///
/// Frame index zero identifies the first frame that reaches `present()`. A
/// failed acquisition, an occluded surface, or a zero-extent surface does not
/// advance the index. The pending plan is intentionally available before the
/// renderer call so a later capture tranche can select the same frame without
/// redefining or duplicating this lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RenderRunControl {
    resolution: RenderResolutionState,
    exit_after_frame: Option<u64>,
    capture_frame: Option<u64>,
    next_presentation_frame: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PresentationFramePlan {
    index: u64,
    exit_after_present: bool,
    capture_requested: bool,
}

impl RenderRunControl {
    const fn new(
        requested_resolution: Option<RenderResolution>,
        exit_after_frame: Option<u64>,
        capture_frame: Option<u64>,
    ) -> Self {
        Self {
            resolution: RenderResolutionState::new(requested_resolution),
            exit_after_frame,
            capture_frame,
            next_presentation_frame: 0,
        }
    }

    const fn requested_resolution(self) -> Option<RenderResolution> {
        self.resolution.requested()
    }

    const fn resolution_is_ready(self) -> bool {
        self.resolution.is_ready()
    }

    const fn pending_frame(self) -> PresentationFramePlan {
        PresentationFramePlan {
            index: self.next_presentation_frame,
            exit_after_present: matches!(
                self.exit_after_frame,
                Some(frame) if frame == self.next_presentation_frame
            ),
            capture_requested: matches!(
                self.capture_frame,
                Some(frame) if frame == self.next_presentation_frame
            ),
        }
    }

    fn commit_presented(&mut self, frame: PresentationFramePlan) -> bool {
        debug_assert_eq!(frame.index, self.next_presentation_frame);
        self.next_presentation_frame = self.next_presentation_frame.saturating_add(1);
        frame.exit_after_present
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Rv26ValidationConfig {
    case: Rv26ValidationCase,
    aerial_perspective_enabled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Rv26ValidationCase {
    Near,
    Distance100M,
    Distance500M,
    Distance1000M,
    FrontLit,
    SideLit,
    BackLit,
}

impl Rv26ValidationCase {
    fn from_label(label: &str) -> Option<Self> {
        match label {
            "near" => Some(Self::Near),
            "100m" => Some(Self::Distance100M),
            "500m" => Some(Self::Distance500M),
            "1000m" => Some(Self::Distance1000M),
            "frontlit" => Some(Self::FrontLit),
            "sidelit" => Some(Self::SideLit),
            "backlit" => Some(Self::BackLit),
            _ => None,
        }
    }

    const fn camera(self) -> CameraSelection {
        let position_render_m = match self {
            Self::Near => [0.0, 1.8, 5.0],
            Self::Distance100M => [0.0, 1.8, 100.0],
            Self::Distance500M => [0.0, 1.8, 500.0],
            Self::Distance1000M => [0.0, 1.8, 1_000.0],
            Self::FrontLit => [80.0, 1.8, -60.0],
            Self::SideLit => [60.0, 1.8, 80.0],
            Self::BackLit => [-80.0, 1.8, 60.0],
        };
        CameraSelection::Pilot {
            position_render_m,
            vertical_fov_deg: 55.0,
        }
    }

    const fn target_extent_m(self) -> f32 {
        match self {
            Self::Near => 1.0,
            Self::Distance100M | Self::FrontLit | Self::SideLit | Self::BackLit => 3.0,
            Self::Distance500M => 15.0,
            Self::Distance1000M => 30.0,
        }
    }
}

/// Presentation-side camera selection parsed from the CLI.
///
/// `CameraConfig` construction is deferred to `camera_config()` so the
/// renderer can build the concrete camera with the window size.
#[derive(Debug, Clone, Copy, PartialEq)]
enum CameraSelection {
    Pilot {
        position_render_m: [f32; 3],
        vertical_fov_deg: f32,
    },
    Chase {
        distance_behind_m: f32,
        height_above_m: f32,
        vertical_fov_deg: f32,
    },
}

impl CameraSelection {
    fn into_camera_config(self) -> CameraConfig {
        match self {
            Self::Pilot {
                position_render_m,
                vertical_fov_deg,
            } => CameraConfig::Pilot {
                position_render_m,
                vertical_fov_deg,
            },
            Self::Chase {
                distance_behind_m,
                height_above_m,
                vertical_fov_deg,
            } => CameraConfig::Chase {
                distance_behind_m,
                height_above_m,
                look_ahead_m: 1.5,
                vertical_fov_deg,
            },
        }
    }
}

impl Default for CameraSelection {
    fn default() -> Self {
        Self::Pilot {
            position_render_m: [0.0, 0.3, 0.85],
            vertical_fov_deg: 70.0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CameraMode {
    Pilot,
    Chase,
}

impl CameraMode {
    const fn label(self) -> &'static str {
        match self {
            Self::Pilot => "pilot",
            Self::Chase => "chase",
        }
    }
}

#[derive(Debug, Default)]
struct PendingCameraOptions {
    mode: Option<CameraMode>,
    vertical_fov_deg: Option<f32>,
    chase_distance_m: Option<f32>,
    chase_height_m: Option<f32>,
    pilot_position_render_m: Option<[f32; 3]>,
}

impl PendingCameraOptions {
    fn resolve(self, default: CameraSelection) -> Result<CameraSelection, RenderAppError> {
        let base = match self.mode {
            Some(CameraMode::Pilot) => CameraSelection::Pilot {
                position_render_m: EXPLICIT_PILOT_POSITION_RENDER_M,
                vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
            },
            Some(CameraMode::Chase) => CameraSelection::Chase {
                distance_behind_m: EXPLICIT_CHASE_DISTANCE_M,
                height_above_m: EXPLICIT_CHASE_HEIGHT_M,
                vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
            },
            None => default,
        };

        match base {
            CameraSelection::Pilot {
                position_render_m,
                vertical_fov_deg,
            } => {
                if self.chase_distance_m.is_some() {
                    return Err(RenderAppError::IncompatibleCameraOption {
                        option: "--chase-distance-m",
                        required_mode: CameraMode::Chase.label(),
                        actual_mode: CameraMode::Pilot.label(),
                    });
                }
                if self.chase_height_m.is_some() {
                    return Err(RenderAppError::IncompatibleCameraOption {
                        option: "--chase-height-m",
                        required_mode: CameraMode::Chase.label(),
                        actual_mode: CameraMode::Pilot.label(),
                    });
                }
                Ok(CameraSelection::Pilot {
                    position_render_m: self.pilot_position_render_m.unwrap_or(position_render_m),
                    vertical_fov_deg: self.vertical_fov_deg.unwrap_or(vertical_fov_deg),
                })
            }
            CameraSelection::Chase {
                distance_behind_m,
                height_above_m,
                vertical_fov_deg,
            } => {
                if self.pilot_position_render_m.is_some() {
                    return Err(RenderAppError::IncompatibleCameraOption {
                        option: "--pilot-position",
                        required_mode: CameraMode::Pilot.label(),
                        actual_mode: CameraMode::Chase.label(),
                    });
                }
                Ok(CameraSelection::Chase {
                    distance_behind_m: self.chase_distance_m.unwrap_or(distance_behind_m),
                    height_above_m: self.chase_height_m.unwrap_or(height_above_m),
                    vertical_fov_deg: self.vertical_fov_deg.unwrap_or(vertical_fov_deg),
                })
            }
        }
    }
}

impl RenderOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, RenderAppError> {
        Self::parse_with_defaults(Self::render_defaults(), &mut arguments)
    }

    pub fn parse_play(mut arguments: impl Iterator<Item = String>) -> Result<Self, RenderAppError> {
        Self::parse_with_defaults(Self::play_defaults(), &mut arguments)
    }

    fn render_defaults() -> Self {
        Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            throttle: DEFAULT_THROTTLE,
            altitude_m: DEFAULT_ALTITUDE_M,
            airspeed_mps: DEFAULT_AIRSPEED_MPS,
            replay_output_path: None,
            controller_profile_path: None,
            start_on_ground: false,
            scenery: SceneryPreset::None,
            camera: CameraSelection::default(),
            debug_overlays: false,
            terrain_debug: TerrainDebugMode::default(),
            vegetation_debug: VegetationDebugMode::default(),
            exposure_ev: DEFAULT_EXPOSURE_EV,
            renderer: RendererVersion::V1,
            rv2_6_validation: None,
            render_resolution: None,
            exit_after_frame: None,
            capture: None,
            visual_audit_out: None,
        }
    }

    fn play_defaults() -> Self {
        Self {
            throttle: 0.0,
            start_on_ground: true,
            scenery: SceneryPreset::FlyingField,
            camera: CameraSelection::Chase {
                distance_behind_m: PLAY_CHASE_DISTANCE_M,
                height_above_m: PLAY_CHASE_HEIGHT_M,
                vertical_fov_deg: 55.0,
            },
            ..Self::render_defaults()
        }
    }

    fn parse_with_defaults(
        mut options: Self,
        arguments: &mut impl Iterator<Item = String>,
    ) -> Result<Self, RenderAppError> {
        let mut pending_camera = PendingCameraOptions::default();
        let mut rv2_6_validation_case = None;
        let mut rv2_6_validation_ap_enabled = true;
        let mut rv2_6_validation_ap_explicit = false;
        let mut render_width = options.render_resolution.map(|resolution| resolution.width);
        let mut render_height = options
            .render_resolution
            .map(|resolution| resolution.height);
        let mut capture_frame = None;
        let mut capture_out = None;
        let mut capture_format = None;
        let mut capture_receipt_out = None;
        let mut visual_audit_out = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => {
                    options.model_path = PathBuf::from(
                        arguments
                            .next()
                            .ok_or(RenderAppError::MissingArgumentValue("--model"))?,
                    );
                }
                "--throttle" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--throttle"))?;
                    options.throttle = value
                        .parse::<f64>()
                        .map_err(|_| RenderAppError::InvalidThrottle(value.clone()))?;
                    if !options.throttle.is_finite() || !(0.0..=1.0).contains(&options.throttle) {
                        return Err(RenderAppError::InvalidThrottle(value));
                    }
                }
                "--altitude-m" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--altitude-m"))?;
                    options.altitude_m = value
                        .parse::<f64>()
                        .map_err(|_| RenderAppError::InvalidAltitude(value.clone()))?;
                    if !options.altitude_m.is_finite()
                        || options.altitude_m <= 0.0
                        || options.altitude_m > MAXIMUM_ALTITUDE_M
                    {
                        return Err(RenderAppError::InvalidAltitude(value));
                    }
                }
                "--airspeed-mps" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--airspeed-mps"))?;
                    options.airspeed_mps = value
                        .parse::<f64>()
                        .map_err(|_| RenderAppError::InvalidAirspeed(value.clone()))?;
                    if !options.airspeed_mps.is_finite()
                        || options.airspeed_mps <= 0.0
                        || options.airspeed_mps > MAXIMUM_AIRSPEED_MPS
                    {
                        return Err(RenderAppError::InvalidAirspeed(value));
                    }
                }
                "--record-replay" => {
                    options.replay_output_path =
                        Some(PathBuf::from(arguments.next().ok_or(
                            RenderAppError::MissingArgumentValue("--record-replay"),
                        )?));
                }
                "--controller-profile" => {
                    options.controller_profile_path =
                        Some(PathBuf::from(arguments.next().ok_or(
                            RenderAppError::MissingArgumentValue("--controller-profile"),
                        )?));
                }
                "--start-on-ground" => {
                    options.start_on_ground = true;
                }
                "--debug-overlays" => {
                    options.debug_overlays = true;
                }
                "--terrain-debug" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--terrain-debug"))?;
                    options.terrain_debug = TerrainDebugMode::from_label(&value)
                        .ok_or_else(|| RenderAppError::InvalidTerrainDebug(value.clone()))?;
                }
                "--vegetation-debug" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--vegetation-debug"))?;
                    options.vegetation_debug = VegetationDebugMode::from_label(&value)
                        .ok_or_else(|| RenderAppError::InvalidVegetationDebug(value.clone()))?;
                }
                "--exposure-ev" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--exposure-ev"))?;
                    let ev = value
                        .parse::<f32>()
                        .map_err(|_| RenderAppError::InvalidExposureEv(value.clone()))?;
                    // Validated here (finite + bounded); the renderer validates
                    // again defensively. Presentation-only: never physics.
                    options.exposure_ev = validate_exposure_ev(ev)
                        .map_err(|_| RenderAppError::InvalidExposureEv(value))?;
                }
                "--renderer" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--renderer"))?;
                    options.renderer = RendererVersion::from_label(&value)
                        .ok_or_else(|| RenderAppError::InvalidRenderer(value.clone()))?;
                }
                "--render-width" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--render-width"))?;
                    let width = value
                        .parse::<u32>()
                        .map_err(|_| RenderAppError::InvalidRenderWidth(value.clone()))?;
                    if !(MINIMUM_RENDER_WIDTH..=MAXIMUM_RENDER_WIDTH).contains(&width) {
                        return Err(RenderAppError::InvalidRenderWidth(value));
                    }
                    render_width = Some(width);
                }
                "--render-height" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--render-height"))?;
                    let height = value
                        .parse::<u32>()
                        .map_err(|_| RenderAppError::InvalidRenderHeight(value.clone()))?;
                    if !(MINIMUM_RENDER_HEIGHT..=MAXIMUM_RENDER_HEIGHT).contains(&height) {
                        return Err(RenderAppError::InvalidRenderHeight(value));
                    }
                    render_height = Some(height);
                }
                "--exit-after-frame" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--exit-after-frame"))?;
                    options.exit_after_frame = Some(
                        value
                            .parse::<u64>()
                            .map_err(|_| RenderAppError::InvalidExitAfterFrame(value))?,
                    );
                }
                "--capture-frame" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--capture-frame"))?;
                    capture_frame = Some(
                        value
                            .parse::<u64>()
                            .map_err(|_| RenderAppError::InvalidCaptureFrame(value))?,
                    );
                }
                "--capture-out" => {
                    capture_out =
                        Some(PathBuf::from(arguments.next().ok_or(
                            RenderAppError::MissingArgumentValue("--capture-out"),
                        )?));
                }
                "--capture-format" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--capture-format"))?;
                    capture_format = Some(match value.as_str() {
                        "png" => CaptureFormat::Png,
                        _ => return Err(RenderAppError::UnsupportedCaptureFormat(value)),
                    });
                }
                "--capture-receipt-out" => {
                    capture_receipt_out = Some(PathBuf::from(arguments.next().ok_or(
                        RenderAppError::MissingArgumentValue("--capture-receipt-out"),
                    )?));
                }
                "--visual-audit-out" => {
                    visual_audit_out =
                        Some(PathBuf::from(arguments.next().ok_or(
                            RenderAppError::MissingArgumentValue("--visual-audit-out"),
                        )?));
                }
                "--rv2-6-validation-scene" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue(
                            "--rv2-6-validation-scene",
                        ))?;
                    rv2_6_validation_case = Some(
                        Rv26ValidationCase::from_label(&value)
                            .ok_or_else(|| RenderAppError::InvalidRv26ValidationScene(value))?,
                    );
                }
                "--rv2-6-validation-ap" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue(
                            "--rv2-6-validation-ap",
                        ))?;
                    rv2_6_validation_ap_enabled = match value.as_str() {
                        "on" => true,
                        "off" => false,
                        _ => return Err(RenderAppError::InvalidRv26ValidationAp(value)),
                    };
                    rv2_6_validation_ap_explicit = true;
                }
                "--scenery" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--scenery"))?;
                    options.scenery = match value.as_str() {
                        "none" => SceneryPreset::None,
                        "flying-field" => SceneryPreset::FlyingField,
                        "photo-field" => SceneryPreset::PhotoField,
                        _ => return Err(RenderAppError::InvalidScenery(value)),
                    };
                }
                "--camera" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--camera"))?;
                    match value.as_str() {
                        "pilot" => pending_camera.mode = Some(CameraMode::Pilot),
                        "chase" => pending_camera.mode = Some(CameraMode::Chase),
                        _ => return Err(RenderAppError::UnknownCamera(value)),
                    }
                }
                "--camera-fov" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--camera-fov"))?;
                    let fov = value
                        .parse::<f32>()
                        .map_err(|_| RenderAppError::InvalidCameraFov(value.clone()))?;
                    if !fov.is_finite() || !(10.0..=120.0).contains(&fov) {
                        return Err(RenderAppError::InvalidCameraFov(value));
                    }
                    pending_camera.vertical_fov_deg = Some(fov);
                }
                "--chase-distance-m" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--chase-distance-m"))?;
                    let distance = value
                        .parse::<f32>()
                        .map_err(|_| RenderAppError::InvalidChaseDistance(value.clone()))?;
                    if !distance.is_finite() || distance <= 0.0 || distance > 1_000.0 {
                        return Err(RenderAppError::InvalidChaseDistance(value));
                    }
                    pending_camera.chase_distance_m = Some(distance);
                }
                "--chase-height-m" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--chase-height-m"))?;
                    let height = value
                        .parse::<f32>()
                        .map_err(|_| RenderAppError::InvalidChaseHeight(value.clone()))?;
                    if !height.is_finite() || !(-100.0..=1_000.0).contains(&height) {
                        return Err(RenderAppError::InvalidChaseHeight(value));
                    }
                    pending_camera.chase_height_m = Some(height);
                }
                "--pilot-position" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--pilot-position"))?;
                    let position = parse_position(&value)
                        .ok_or_else(|| RenderAppError::InvalidPilotPosition(value.clone()))?;
                    pending_camera.pilot_position_render_m = Some(position);
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(RenderAppError::UnknownArgument(argument)),
            }
        }
        options.render_resolution = match (render_width, render_height) {
            (None, None) => None,
            (Some(width), Some(height)) => Some(RenderResolution::new(width, height)),
            _ => return Err(RenderAppError::IncompleteRenderResolution),
        };
        let capture_requested = capture_frame.is_some()
            || capture_out.is_some()
            || capture_format.is_some()
            || capture_receipt_out.is_some();
        options.capture = if capture_requested {
            let presentation_frame_index =
                capture_frame.ok_or(RenderAppError::IncompleteCaptureOptions("--capture-frame"))?;
            let image_path =
                capture_out.ok_or(RenderAppError::IncompleteCaptureOptions("--capture-out"))?;
            let format = capture_format
                .ok_or(RenderAppError::IncompleteCaptureOptions("--capture-format"))?;
            Some(CaptureConfig {
                presentation_frame_index,
                image_path,
                format,
                receipt_path: capture_receipt_out,
            })
        } else {
            None
        };
        options.visual_audit_out = visual_audit_out;
        if options.visual_audit_out.is_some() && options.renderer != RendererVersion::V2 {
            return Err(RenderAppError::VisualAuditRequiresV2);
        }
        if options.visual_audit_out.is_some()
            && options.capture.is_none()
            && options.exit_after_frame.is_none()
        {
            return Err(RenderAppError::VisualAuditRequiresControlledFrame);
        }
        let mut output_paths = Vec::new();
        if let Some(capture) = options.capture.as_ref() {
            output_paths.push(capture.image_path.clone());
            if let Some(receipt_path) = capture.receipt_path.as_ref() {
                output_paths.push(receipt_path.clone());
            }
        }
        if let Some(audit_path) = options.visual_audit_out.as_ref() {
            output_paths.push(audit_path.clone());
        }
        let temporary_paths = output_paths
            .iter()
            .filter_map(|path| temporary_output_path(path).ok())
            .collect::<Vec<_>>();
        output_paths.extend(temporary_paths);
        for (index, path) in output_paths.iter().enumerate() {
            if output_paths[index + 1..].contains(path) {
                return Err(RenderAppError::ConflictingCaptureOutputs);
            }
        }
        if let (Some(exit_frame), Some(capture)) = (options.exit_after_frame, &options.capture)
            && exit_frame < capture.presentation_frame_index
        {
            return Err(RenderAppError::ExitBeforeCaptureFrame {
                exit_frame,
                capture_frame: capture.presentation_frame_index,
            });
        }
        // Read before `resolve` consumes the pending options: PF1 needs to know
        // whether the operator chose the pilot eye explicitly.
        let pending_camera_explicit_pilot_position = pending_camera.pilot_position_render_m;
        options.camera = pending_camera.resolve(options.camera)?;
        if let Some(case) = rv2_6_validation_case {
            if options.renderer != RendererVersion::V2 {
                return Err(RenderAppError::Rv26ValidationRequiresV2);
            }
            options.throttle = 0.0;
            options.start_on_ground = true;
            options.scenery = SceneryPreset::None;
            options.camera = case.camera();
            options.debug_overlays = false;
            options.terrain_debug = TerrainDebugMode::Final;
            options.vegetation_debug = VegetationDebugMode::Final;
            options.rv2_6_validation = Some(Rv26ValidationConfig {
                case,
                aerial_perspective_enabled: rv2_6_validation_ap_enabled,
            });
        } else if rv2_6_validation_ap_explicit {
            return Err(RenderAppError::Rv26ValidationApRequiresScene);
        }
        // PF1: a photographic field is only valid from the surveyed eye, so the
        // CLI defaults to the manifest pilot position instead of the generic
        // pilot default. An explicit `--pilot-position` wins and is then checked
        // against the manifest by the renderer's `fixed_pilot_eye`, which
        // rejects a mismatch cleanly rather than silently re-anchoring the
        // photograph. A Chase camera is deliberately left for the renderer to
        // reject too (`RendererError::PhotoFieldCamera`): this layer stays a
        // parser and never encodes the fixed-eye rule twice.
        if options.scenery == SceneryPreset::PhotoField
            && pending_camera_explicit_pilot_position.is_none()
            && let CameraSelection::Pilot {
                vertical_fov_deg, ..
            } = options.camera
        {
            options.camera = CameraSelection::Pilot {
                position_render_m: photo_field_default_pilot_position()
                    .map_err(RenderAppError::PhotoFieldManifest)?,
                vertical_fov_deg,
            };
        }
        Ok(options)
    }
}

#[derive(Debug, Error)]
pub enum RenderAppError {
    #[error("missing value for render option {0}")]
    MissingArgumentValue(&'static str),
    #[error("invalid render throttle `{0}`; expected a finite value inside [0, 1]")]
    InvalidThrottle(String),
    #[error("invalid render altitude `{0}`; expected a finite value inside (0, 10000] metres")]
    InvalidAltitude(String),
    #[error("invalid render airspeed `{0}`; expected a finite value inside (0, 200] metres/second")]
    InvalidAirspeed(String),
    #[error("unknown render argument: {0}")]
    UnknownArgument(String),
    #[error("invalid scenery preset `{0}`; expected `none`, `flying-field`, or `photo-field`")]
    InvalidScenery(String),
    #[error("the photo field manifest was rejected: {0}")]
    PhotoFieldManifest(#[from] PhotoFieldManifestError),
    #[error(
        "invalid terrain debug mode `{0}`; expected `final`, `albedo`, `normal`, `roughness`, `macro`, or `detail`"
    )]
    InvalidTerrainDebug(String),
    #[error("invalid vegetation debug mode `{0}`; expected `final`, `lod`, or `culling`")]
    InvalidVegetationDebug(String),
    #[error("invalid exposure EV `{0}`; expected a finite value inside [-8, 8]")]
    InvalidExposureEv(String),
    #[error("invalid renderer `{0}`; expected `v1` or `v2`")]
    InvalidRenderer(String),
    #[error("invalid render width `{0}`; expected an integer inside [320, 7680] physical pixels")]
    InvalidRenderWidth(String),
    #[error("invalid render height `{0}`; expected an integer inside [240, 4320] physical pixels")]
    InvalidRenderHeight(String),
    #[error("`--render-width` and `--render-height` must be specified together")]
    IncompleteRenderResolution,
    #[error("invalid exit frame `{0}`; expected a non-negative integer")]
    InvalidExitAfterFrame(String),
    #[error("invalid capture frame `{0}`; expected a non-negative integer")]
    InvalidCaptureFrame(String),
    #[error("incomplete capture options; missing required {0}")]
    IncompleteCaptureOptions(&'static str),
    #[error("unsupported capture format `{0}`; only `png` is supported")]
    UnsupportedCaptureFormat(String),
    #[error("capture image, receipt, visual audit, and their temporary paths must be distinct")]
    ConflictingCaptureOutputs,
    #[error("`--visual-audit-out` requires `--renderer v2`")]
    VisualAuditRequiresV2,
    #[error("`--visual-audit-out` requires `--capture-frame` or `--exit-after-frame`")]
    VisualAuditRequiresControlledFrame,
    #[error(
        "exit frame {exit_frame} is before capture frame {capture_frame}; exit must be at or after capture"
    )]
    ExitBeforeCaptureFrame { exit_frame: u64, capture_frame: u64 },
    #[error("failed to remove stale capture output {path}: {source}")]
    StaleCaptureOutput {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to remove stale runtime visual audit output {path}: {source}")]
    StaleVisualAuditOutput {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error(
        "invalid RV2-6 validation scene `{0}`; expected `near`, `100m`, `500m`, `1000m`, `frontlit`, `sidelit`, or `backlit`"
    )]
    InvalidRv26ValidationScene(String),
    #[error("invalid RV2-6 validation AP state `{0}`; expected `on` or `off`")]
    InvalidRv26ValidationAp(String),
    #[error("RV2-6 validation scenes require `--renderer v2`")]
    Rv26ValidationRequiresV2,
    #[error("`--rv2-6-validation-ap` requires `--rv2-6-validation-scene`")]
    Rv26ValidationApRequiresScene,
    #[error("unknown camera mode `{0}`; expected `pilot` or `chase`")]
    UnknownCamera(String),
    #[error("invalid camera FOV `{0}`; expected a finite value inside [10, 120] degrees")]
    InvalidCameraFov(String),
    #[error("invalid chase distance `{0}`; expected a finite value inside (0, 1000] metres")]
    InvalidChaseDistance(String),
    #[error("invalid chase height `{0}`; expected a finite value inside [-100, 1000] metres")]
    InvalidChaseHeight(String),
    #[error("invalid pilot position `{0}`; expected three finite numbers `x,y,z`")]
    InvalidPilotPosition(String),
    #[error(
        "camera option {option} requires final camera mode `{required_mode}`, but final mode is `{actual_mode}`"
    )]
    IncompatibleCameraOption {
        option: &'static str,
        required_mode: &'static str,
        actual_mode: &'static str,
    },
    #[error("failed to load render model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to load declared presentation asset {path}: {source}")]
    PresentationAsset {
        path: PathBuf,
        #[source]
        source: Box<GlbLoadError>,
    },
    #[error("render ground start requested for model {model_id:?}, but it has no landing gear")]
    GroundStartWithoutLandingGear { model_id: String },
    #[error("model {model_id:?} cannot form a supported render ground start: {reason}")]
    InvalidGroundStart {
        model_id: String,
        reason: &'static str,
    },
    #[error("invalid articulated GLB presentation mapping: {0}")]
    PresentationArticulation(#[from] GlbArticulationError),
    #[error("failed to initialize AircraftSimulation for render mode: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error("failed to configure the render atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure the 500 Hz render simulation: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to configure render fixed-step scheduling: {0}")]
    FixedStep(#[from] FixedStepAccumulatorError),
    #[error("failed to initialize render input: {0}")]
    Input(#[from] InputError),
    #[error(transparent)]
    ControllerProfile(#[from] ControllerProfileFileError),
    #[error("failed to initialize live aircraft replay recording: {0}")]
    Replay(#[from] AircraftReplayError),
    #[error("failed to create the winit event loop: {0}")]
    EventLoopCreation(#[source] winit::error::EventLoopError),
    #[error("winit event loop failed: {0}")]
    EventLoopRun(#[source] winit::error::EventLoopError),
    #[error("render application terminated after a runtime error: {0}")]
    Runtime(#[from] RenderRuntimeError),
}

#[derive(Debug, Error)]
pub enum RenderRuntimeError {
    #[error("failed to create the desktop render window: {0}")]
    WindowCreation(#[source] winit::error::OsError),
    #[error(
        "requested render size {requested_width}x{requested_height} physical pixels was not applied; actual window size is {actual_width}x{actual_height}"
    )]
    RequestedRenderSizeNotApplied {
        requested_width: u32,
        requested_height: u32,
        actual_width: u32,
        actual_height: u32,
    },
    #[error("failed to preserve the requested physical render size across a DPI change: {0}")]
    RequestedRenderSizeUpdate(#[source] winit::error::ExternalError),
    #[error("failed to initialize wgpu: {0}")]
    RendererInitialization(#[source] RendererError),
    #[error("invalid exposure EV rejected at renderer startup: {0}")]
    ExposureValidation(#[source] ExposureError),
    #[error("failed to convert the committed physics pose for rendering: {0}")]
    RenderPose(#[from] RenderDataError),
    #[error("failed to sample normalized pilot input: {0}")]
    Input(#[from] InputError),
    #[error("failed to initialize render input: {0}")]
    InputInitialization(InputError),
    #[error("failed to reconstruct the aircraft simulation during flight reset: {0}")]
    FlightResetSimulation(#[source] AircraftSimulationError),
    #[error("failed to reset flight input state: {0}")]
    FlightResetInput(#[source] InputError),
    #[error("failed to reset fixed-step scheduling: {0}")]
    FlightResetScheduling(#[source] FixedStepAccumulatorError),
    #[error("failed to load controller profile for render input: {0}")]
    ControllerProfileInitialization(#[source] ControllerProfileFileError),
    #[error("render input backend was not initialized after window creation")]
    InputNotInitialized,
    #[error("failed to record live aircraft replay: {0}")]
    Replay(#[from] AircraftReplayError),
    #[error("failed to write live aircraft replay to {path}: {source}")]
    ReplayWrite {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("GPU ran out of memory")]
    OutOfMemory,
    #[error("unexpected GPU validation or internal error")]
    GpuValidation,
    #[error("deterministic frame capture failed: {0}")]
    FrameCapture(#[source] FrameCaptureError),
    #[error("the application exited before the requested presentation frame was captured")]
    CaptureNotCompleted,
    #[error("captured RGBA8 dimensions do not match the returned pixel byte count")]
    InvalidCapturedFrame,
    #[error("failed to encode captured frame as lossless PNG: {0}")]
    PngEncode(#[source] image::ImageError),
    #[error("failed to write captured image to {path}: {source}")]
    CaptureImageWrite {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to serialize runtime capture receipt: {0}")]
    CaptureReceiptSerialization(#[source] serde_json::Error),
    #[error("failed to write runtime capture receipt to {path}: {source}")]
    CaptureReceiptWrite {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("the V2 renderer did not provide an audit for the presented frame")]
    VisualAuditUnavailable,
    #[error("runtime visual audit frame/extent does not match the captured presentation frame")]
    VisualAuditIdentityMismatch,
    #[error("failed to serialize runtime visual audit: {0}")]
    VisualAuditSerialization(#[source] serde_json::Error),
    #[error("failed to write runtime visual audit to {path}: {source}")]
    VisualAuditWrite {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("the application exited before the requested runtime visual audit was written")]
    VisualAuditNotCompleted,
}

pub fn run_render(options: RenderOptions) -> Result<(), RenderAppError> {
    prepare_capture_outputs(options.capture.as_ref())?;
    prepare_visual_audit_output(options.visual_audit_out.as_deref())?;
    let mut application = RenderApplication::new(options)?;
    let event_loop = EventLoop::new().map_err(RenderAppError::EventLoopCreation)?;
    event_loop.set_control_flow(ControlFlow::Poll);
    event_loop
        .run_app(&mut application)
        .map_err(RenderAppError::EventLoopRun)?;
    if let Some(error) = application.runtime_error.take() {
        return Err(error.into());
    }
    application.save_recording()?;
    Ok(())
}

fn temporary_output_path(path: &Path) -> io::Result<PathBuf> {
    let file_name = path
        .file_name()
        .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "output has no file name"))?;
    let mut temporary_name = file_name.to_os_string();
    temporary_name.push(".tmp");
    Ok(path.with_file_name(temporary_name))
}

fn remove_file_if_present(path: &Path) -> io::Result<()> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(error) if error.kind() == io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(error),
    }
}

fn prepare_capture_outputs(capture: Option<&CaptureConfig>) -> Result<(), RenderAppError> {
    let Some(capture) = capture else {
        return Ok(());
    };
    let mut paths = vec![capture.image_path.clone()];
    if let Some(receipt_path) = capture.receipt_path.as_ref() {
        paths.push(receipt_path.clone());
    }
    let temporary_paths = paths
        .iter()
        .map(|path| temporary_output_path(path))
        .collect::<Result<Vec<_>, _>>()
        .map_err(|source| RenderAppError::StaleCaptureOutput {
            path: capture.image_path.clone(),
            source,
        })?;
    paths.extend(temporary_paths);
    for path in paths {
        remove_file_if_present(&path).map_err(|source| RenderAppError::StaleCaptureOutput {
            path: path.clone(),
            source,
        })?;
    }
    Ok(())
}

fn prepare_visual_audit_output(path: Option<&Path>) -> Result<(), RenderAppError> {
    let Some(path) = path else {
        return Ok(());
    };
    let temporary =
        temporary_output_path(path).map_err(|source| RenderAppError::StaleVisualAuditOutput {
            path: path.to_path_buf(),
            source,
        })?;
    for target in [path, temporary.as_path()] {
        remove_file_if_present(target).map_err(|source| {
            RenderAppError::StaleVisualAuditOutput {
                path: target.to_path_buf(),
                source,
            }
        })?;
    }
    Ok(())
}

fn write_capture_file(path: &Path, bytes: &[u8]) -> Result<(), RenderRuntimeError> {
    let temporary =
        temporary_output_path(path).map_err(|source| RenderRuntimeError::CaptureImageWrite {
            path: path.to_path_buf(),
            source,
        })?;
    if let Err(source) = fs::write(&temporary, bytes) {
        let _ = remove_file_if_present(&temporary);
        return Err(RenderRuntimeError::CaptureImageWrite {
            path: path.to_path_buf(),
            source,
        });
    }
    if let Err(source) = fs::rename(&temporary, path) {
        let _ = remove_file_if_present(&temporary);
        return Err(RenderRuntimeError::CaptureImageWrite {
            path: path.to_path_buf(),
            source,
        });
    }
    Ok(())
}

fn write_receipt_file(path: &Path, bytes: &[u8]) -> Result<(), RenderRuntimeError> {
    let temporary =
        temporary_output_path(path).map_err(|source| RenderRuntimeError::CaptureReceiptWrite {
            path: path.to_path_buf(),
            source,
        })?;
    if let Err(source) = fs::write(&temporary, bytes) {
        let _ = remove_file_if_present(&temporary);
        return Err(RenderRuntimeError::CaptureReceiptWrite {
            path: path.to_path_buf(),
            source,
        });
    }
    if let Err(source) = fs::rename(&temporary, path) {
        let _ = remove_file_if_present(&temporary);
        return Err(RenderRuntimeError::CaptureReceiptWrite {
            path: path.to_path_buf(),
            source,
        });
    }
    Ok(())
}

fn write_visual_audit_file(path: &Path, bytes: &[u8]) -> Result<(), RenderRuntimeError> {
    let temporary =
        temporary_output_path(path).map_err(|source| RenderRuntimeError::VisualAuditWrite {
            path: path.to_path_buf(),
            source,
        })?;
    let result = (|| -> io::Result<()> {
        let mut file = fs::File::create(&temporary)?;
        file.write_all(bytes)?;
        file.sync_all()?;
        drop(file);
        fs::rename(&temporary, path)
    })();
    if let Err(source) = result {
        let _ = remove_file_if_present(&temporary);
        return Err(RenderRuntimeError::VisualAuditWrite {
            path: path.to_path_buf(),
            source,
        });
    }
    Ok(())
}

fn persist_runtime_visual_audit(
    path: &Path,
    audit: &RuntimeVisualAudit,
    expected_frame: u64,
    expected_extent: Option<(u32, u32)>,
) -> Result<(), RenderRuntimeError> {
    if audit.identity.presentation_frame_index != expected_frame
        || audit.profiling.presentation_frame_index != expected_frame
        || expected_extent.is_some_and(|(width, height)| {
            audit.identity.framebuffer_width != width || audit.identity.framebuffer_height != height
        })
    {
        return Err(RenderRuntimeError::VisualAuditIdentityMismatch);
    }
    let mut bytes =
        serde_json::to_vec_pretty(audit).map_err(RenderRuntimeError::VisualAuditSerialization)?;
    bytes.push(b'\n');
    write_visual_audit_file(path, &bytes)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

fn persist_capture_artifacts(
    capture: &CaptureConfig,
    presentation_frame_index: u64,
    frame: &CapturedFrame,
) -> Result<RuntimeCaptureReceipt, RenderRuntimeError> {
    let expected_len = u64::from(frame.width)
        .checked_mul(u64::from(frame.height))
        .and_then(|pixels| pixels.checked_mul(4))
        .and_then(|bytes| usize::try_from(bytes).ok())
        .ok_or(RenderRuntimeError::InvalidCapturedFrame)?;
    if frame.rgba8.len() != expected_len {
        return Err(RenderRuntimeError::InvalidCapturedFrame);
    }
    let mut png_bytes = Vec::new();
    PngEncoder::new(&mut png_bytes)
        .write_image(
            &frame.rgba8,
            frame.width,
            frame.height,
            image::ExtendedColorType::Rgba8,
        )
        .map_err(RenderRuntimeError::PngEncode)?;
    let image_byte_size =
        u64::try_from(png_bytes.len()).map_err(|_| RenderRuntimeError::InvalidCapturedFrame)?;
    let receipt = RuntimeCaptureReceipt {
        schema_version: "1.0.0",
        presentation_frame_index,
        framebuffer_width: frame.width,
        framebuffer_height: frame.height,
        format: capture.format.label(),
        image_path: capture.image_path.to_string_lossy().into_owned(),
        image_sha256: sha256_hex(&png_bytes),
        image_byte_size,
    };
    let receipt_bytes = if capture.receipt_path.is_some() {
        let mut bytes = serde_json::to_vec_pretty(&receipt)
            .map_err(RenderRuntimeError::CaptureReceiptSerialization)?;
        bytes.push(b'\n');
        Some(bytes)
    } else {
        None
    };

    write_capture_file(&capture.image_path, &png_bytes)?;
    if let (Some(receipt_path), Some(receipt_bytes)) =
        (capture.receipt_path.as_ref(), receipt_bytes.as_deref())
        && let Err(error) = write_receipt_file(receipt_path, receipt_bytes)
    {
        let _ = remove_file_if_present(&capture.image_path);
        return Err(error);
    }
    Ok(receipt)
}

enum PresentationModel {
    Glb {
        asset: GlbAsset,
        articulation: GlbArticulationPlan,
    },
    Procedural(AircraftMesh),
}

enum ViewerInputMode {
    Legacy {
        state: InputState,
        status: ControllerStatusTracker,
    },
    Calibrated(Box<CalibratedControllerState>),
}

impl ViewerInputMode {
    fn set_key(&mut self, key: KeyboardKey, pressed: bool) {
        if let Self::Legacy { state, .. } = self {
            state.set_key(key, pressed);
        }
    }

    fn poll_hardware(
        &mut self,
        backend: &mut GilrsInputBackend,
    ) -> Result<Option<&'static str>, InputError> {
        match self {
            Self::Legacy { state, status } => {
                let controller_axes = backend.poll_axes();
                let selected_controller_id = backend.selected_device_id();
                if let Some(event) = status.observe(selected_controller_id) {
                    let devices = backend.devices();
                    let views = controller_device_views(&devices, selected_controller_id);
                    println!("{}", format_controller_transition(event, &views));
                }
                state.set_controller_axes(controller_axes);
                Ok(None)
            }
            Self::Calibrated(state) => poll_calibrated_hardware(state, backend),
        }
    }

    fn sample(&mut self, physics_dt_s: f64) -> Result<PilotInput, InputError> {
        match self {
            Self::Legacy { state, .. } => state.sample(physics_dt_s),
            Self::Calibrated(state) => Ok(state.input()),
        }
    }

    fn reset_flight_controls(&mut self, initial_throttle: f64) -> Result<(), InputError> {
        if let Self::Legacy { state, .. } = self {
            *state = InputState::new(
                InputMapping::default(),
                KeyboardInputState::new(initial_throttle)?,
            );
        }
        Ok(())
    }

    fn diagnostic_label(&self, selected_controller_id: Option<usize>) -> &'static str {
        match self {
            Self::Legacy { .. } if selected_controller_id.is_some() => "legacy controller mapping",
            Self::Legacy { .. } => "keyboard fallback",
            Self::Calibrated(_) => "calibrated controller profile",
        }
    }
}

fn poll_calibrated_hardware(
    state: &mut CalibratedControllerState,
    backend: &mut GilrsInputBackend,
) -> Result<Option<&'static str>, InputError> {
    if !state.is_connected() {
        // Advance the gilrs/WGI event queue before enumerating devices: WGI
        // reports newly connected controllers through event delivery, so a
        // stale device list would keep the calibrated binding stuck in
        // "waiting for requested controller". poll_axes() is used here only
        // as the event pump; its returned axis values are discarded.
        let _ = backend.poll_axes();
        let devices = backend.devices();
        let identities: Vec<DeviceIdentity> =
            devices.iter().map(InputDeviceInfo::identity).collect();
        if state.match_requested_device(&identities).is_err()
            || backend.select_device(state.requested_device()).is_err()
        {
            return Ok(state.neutralize().map(|event| event.message()));
        }
    }

    match backend.poll_raw_axes()? {
        Some(raw_state) => Ok(state
            .accept_raw_state(&raw_state)?
            .map(|event| event.message())),
        None => Ok(state.neutralize().map(|event| event.message())),
    }
}

/// Pure calibrated-startup connection transition.
///
/// Returns `Ok(Some(()))` only when `candidates` contains the requested
/// controller and `raw` (already selected and polled by the caller) contains
/// every assigned axis. Returns `Ok(None)` whenever the controller is not
/// (yet) available — transient WGI enumeration, a wrong-only device list,
/// ambiguity, no raw sample yet, or an early raw sample still missing an
/// assigned axis — so startup stays neutral and [`poll_calibrated_hardware`]
/// keeps retrying the same decision every frame. Absence at startup is never
/// an error; `Err` is reserved for genuine input failures.
fn calibrate_startup_connect(
    state: &mut CalibratedControllerState,
    candidates: &[DeviceIdentity],
    raw: Option<RawControllerState>,
) -> Result<Option<()>, InputError> {
    if state.match_requested_device(candidates).is_err() {
        return Ok(None);
    }
    let Some(raw_state) = raw else {
        return Ok(None);
    };
    state.accept_raw_state(&raw_state)?;
    Ok(state.is_connected().then_some(()))
}

/// Best-effort connect of the requested controller right after profile load.
///
/// Absent/ambiguous devices and incomplete first samples simply yield
/// `Ok(None)` (waiting, neutral); the runtime loop is the single authority
/// for every later connect, disconnect, and reconnect.
fn try_connect_requested_controller(
    state: &mut CalibratedControllerState,
    backend: &mut GilrsInputBackend,
) -> Result<Option<InputDeviceInfo>, InputError> {
    let devices = backend.devices();
    let identities: Vec<DeviceIdentity> = devices.iter().map(InputDeviceInfo::identity).collect();
    if state.match_requested_device(&identities).is_err() {
        return Ok(None);
    }
    let selected = match backend.select_device(state.requested_device()) {
        Ok(selected) => selected,
        Err(_) => return Ok(None),
    };
    let raw_state = match backend.poll_raw_axes()? {
        Some(raw_state) => raw_state,
        None => return Ok(None),
    };
    if calibrate_startup_connect(state, &identities, Some(raw_state))?.is_none() {
        return Ok(None);
    }
    Ok(Some(selected))
}

/// Startup status block for calibrated input.
///
/// `matched` carries the matched device when the requested controller was
/// already available at startup; `None` renders the explicit "waiting for
/// requested controller" state with neutral pilot input.
fn format_calibrated_startup_status(
    profile_path: &Path,
    state: &CalibratedControllerState,
    matched: Option<(usize, DeviceIdentity)>,
) -> String {
    match matched {
        Some((id, identity)) => format!(
            "Controller profile:\n{}\n\
             Profile schema:\n{}\n\
             Input mode:\ncalibrated controller profile\n\
             Requested controller:\n{}\n\
             Matched controller:\nsession_id={id} {}\n\
             Controller status:\nconnected\n\
             Pilot input:\ncalibrated",
            profile_path.display(),
            state.profile().schema_version(),
            format_device_identity(state.requested_device()),
            format_device_identity(&identity),
        ),
        None => format!(
            "Controller profile:\n{}\n\
             Profile schema:\n{}\n\
             Input mode:\ncalibrated controller profile\n\
             Requested controller:\n{}\n\
             Controller status:\nwaiting for requested controller\n\
             Pilot input:\nneutral",
            profile_path.display(),
            state.profile().schema_version(),
            format_device_identity(state.requested_device()),
        ),
    }
}

fn initialize_viewer_input(
    profile_path: Option<&Path>,
    initial_throttle: f64,
    backend: &mut GilrsInputBackend,
) -> Result<(ViewerInputMode, String), RenderRuntimeError> {
    let Some(profile_path) = profile_path else {
        let selected_controller_id = backend.selected_device_id();
        let controller_devices = backend.devices();
        let controller_views = controller_device_views(&controller_devices, selected_controller_id);
        let startup_status =
            format_viewer_controller_status(&controller_views, selected_controller_id);
        return Ok((
            ViewerInputMode::Legacy {
                state: InputState::new(
                    InputMapping::default(),
                    KeyboardInputState::new(initial_throttle)
                        .map_err(RenderRuntimeError::InputInitialization)?,
                ),
                status: ControllerStatusTracker::new(selected_controller_id),
            },
            startup_status,
        ));
    };

    // Profile loading and validation remain fatal: a bad profile is a
    // configuration error. Absence of the requested hardware is not.
    let profile = load_controller_profile(profile_path)
        .map_err(RenderRuntimeError::ControllerProfileInitialization)?;
    let mut state = CalibratedControllerState::new(profile);
    let matched = try_connect_requested_controller(&mut state, backend)
        .map_err(RenderRuntimeError::InputInitialization)?;
    let startup_status = format_calibrated_startup_status(
        profile_path,
        &state,
        matched
            .as_ref()
            .map(|device| (device.id(), device.identity())),
    );
    Ok((ViewerInputMode::Calibrated(Box::new(state)), startup_status))
}

fn print_viewer_controller_diagnostics(backend: &GilrsInputBackend, input_mode: &ViewerInputMode) {
    let devices = backend.devices();
    println!("Viewer input initialized after window creation");
    println!("WGI controllers detected: {}", devices.len());
    if devices.is_empty() {
        println!("WGI controller identities: none");
    } else {
        for device in devices {
            println!(
                "WGI controller identity: session_id={} {}",
                device.id(),
                format_device_identity(&device.identity())
            );
        }
    }
    println!(
        "Input mode: {}",
        input_mode.diagnostic_label(backend.selected_device_id())
    );
}

struct RenderApplication {
    simulation: AircraftSimulation,
    initial_rigid_state: RigidBodyState,
    presentation: PresentationModel,
    scenery_preset: SceneryPreset,
    debug_overlays: bool,
    initial_throttle: f64,
    controller_profile_path: Option<PathBuf>,
    input_mode: Option<ViewerInputMode>,
    input_backend: Option<GilrsInputBackend>,
    terrain_debug: TerrainDebugMode,
    // G3D: presentation-only vegetation debug channel (CLI-provided).
    vegetation_debug: VegetationDebugMode,
    // G3B: presentation-only manual exposure (EV stops), CLI-provided.
    exposure_ev: f32,
    replay_recorder: Option<AircraftReplayRecorder>,
    replay_output_path: Option<PathBuf>,
    render_origin_world_ned_m: [f64; 3],
    ground_below_render_origin_m: f32,
    ground_start: bool,
    terrain_mode: RenderTerrainMode,
    camera_config: CameraConfig,
    render_snapshots: AircraftRenderSnapshotBuffer,
    fixed_step: FixedStepAccumulator,
    last_frame_time: Option<Instant>,
    window: Option<Arc<Window>>,
    renderer: Option<DesktopRenderer>,
    // RV2-1: backend selected on the CLI; the facade dispatches to it.
    renderer_version: RendererVersion,
    rv2_6_validation: Option<Rv26ValidationConfig>,
    run_control: RenderRunControl,
    capture: Option<CaptureConfig>,
    capture_completed: bool,
    visual_audit_out: Option<PathBuf>,
    visual_audit_frame: Option<u64>,
    visual_audit_completed: bool,
    runtime_error: Option<RenderRuntimeError>,
}

impl RenderApplication {
    fn new(options: RenderOptions) -> Result<Self, RenderAppError> {
        let rv2_6_validation = options.rv2_6_validation;
        let capture = options.capture;
        let capture_frame = capture
            .as_ref()
            .map(|capture| capture.presentation_frame_index);
        let visual_audit_out = options.visual_audit_out;
        let visual_audit_frame = visual_audit_out.as_ref().map(|_| {
            capture_frame
                .or(options.exit_after_frame)
                .expect("validated audit frame")
        });
        let run_control = RenderRunControl::new(
            options.render_resolution,
            options.exit_after_frame,
            capture_frame,
        );
        let altitude_m = options.altitude_m;
        let airspeed_mps = options.airspeed_mps;
        let initial_throttle = options.throttle;
        let ground_start = options.start_on_ground;
        let model_path = options.model_path;
        let model =
            load_aircraft_model(&model_path).map_err(|source| RenderAppError::ModelLoad {
                path: model_path.clone(),
                source,
            })?;
        let model_id = model.model_id().to_owned();
        let model_fingerprint = model.physics_fingerprint();
        let (initial_state, ground_below_render_origin_m, terrain_mode, initial_ground) =
            if ground_start {
                let initialized = supported_ground_start(&model)?;
                (
                    initialized.state,
                    initialized.ground_below_render_origin_m,
                    RenderTerrainMode::Flat,
                    initialized.ground_evaluation,
                )
            } else {
                (
                    render_initial_state(altitude_m, airspeed_mps),
                    altitude_m as f32,
                    RenderTerrainMode::Rolling,
                    GroundEvaluation::zero(),
                )
            };
        let render_origin_world_ned_m = vector_to_array(initial_state.position_world_m);
        let presentation = if let Some(validation) = rv2_6_validation {
            PresentationModel::Procedural(rv2_6_validation_target_mesh(
                -ground_below_render_origin_m,
                validation.case.target_extent_m(),
            ))
        } else {
            resolve_presentation_model(&model_path, model.presentation())?
        };
        let render_snapshots =
            AircraftRenderSnapshotBuffer::new(AircraftRenderSnapshot::initial(&initial_state));
        let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
        let config = AircraftSimulationConfig::from_physics_hz(DEFAULT_PHYSICS_HZ, environment)?;
        let mut simulation = AircraftSimulation::new(model, config, initial_state)?;
        if ground_start {
            simulation.refresh_ground_diagnostics(GroundCommand::new(0.0, 0.0));
            debug_assert_eq!(simulation.last_ground_evaluation(), &initial_ground);
        }
        let replay_recorder = options
            .replay_output_path
            .as_ref()
            .map(|_| AircraftReplayRecorder::new(&simulation))
            .transpose()?;
        let fixed_step = FixedStepAccumulator::new(
            PHYSICS_DT,
            MAXIMUM_FRAME_DELTA,
            MAXIMUM_PHYSICS_STEPS_PER_FRAME,
        )?;
        print_manual_flight_startup(
            &model_id,
            &model_fingerprint,
            &initial_state,
            initial_throttle,
            ground_start,
            initial_ground.weight_on_wheels(),
            terrain_mode,
        );
        Ok(Self {
            simulation,
            initial_rigid_state: initial_state,
            presentation,
            scenery_preset: options.scenery,
            debug_overlays: options.debug_overlays,
            initial_throttle,
            controller_profile_path: options.controller_profile_path,
            input_mode: None,
            input_backend: None,
            terrain_debug: options.terrain_debug,
            vegetation_debug: options.vegetation_debug,
            exposure_ev: options.exposure_ev,
            replay_recorder,
            replay_output_path: options.replay_output_path,
            render_origin_world_ned_m,
            ground_below_render_origin_m,
            ground_start,
            terrain_mode,
            camera_config: options.camera.into_camera_config(),
            render_snapshots,
            fixed_step,
            last_frame_time: None,
            window: None,
            renderer: None,
            renderer_version: options.renderer,
            rv2_6_validation,
            run_control,
            capture,
            capture_completed: false,
            visual_audit_out,
            visual_audit_frame,
            visual_audit_completed: false,
            runtime_error: None,
        })
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, error: RenderRuntimeError) {
        self.runtime_error = Some(error);
        event_loop.exit();
    }

    fn initialize_input_after_window(&mut self, event_loop: &ActiveEventLoop) -> bool {
        if self.input_backend.is_some() && self.input_mode.is_some() {
            return true;
        }

        let mut input_backend = match GilrsInputBackend::new() {
            Ok(backend) => backend,
            Err(error) => {
                self.fail(event_loop, error.into());
                return false;
            }
        };
        let (input_mode, controller_startup_status) = match initialize_viewer_input(
            self.controller_profile_path.as_deref(),
            self.initial_throttle,
            &mut input_backend,
        ) {
            Ok(initialized) => initialized,
            Err(error) => {
                self.fail(event_loop, error);
                return false;
            }
        };
        print_viewer_controller_diagnostics(&input_backend, &input_mode);
        println!();
        println!("{controller_startup_status}");
        self.input_backend = Some(input_backend);
        self.input_mode = Some(input_mode);
        true
    }

    fn fail_resolution_mismatch(
        &mut self,
        event_loop: &ActiveEventLoop,
        mismatch: RenderResolutionMismatch,
    ) {
        self.fail(
            event_loop,
            RenderRuntimeError::RequestedRenderSizeNotApplied {
                requested_width: mismatch.requested.width,
                requested_height: mismatch.requested.height,
                actual_width: mismatch.actual.width,
                actual_height: mismatch.actual.height,
            },
        );
    }

    fn request_configured_resolution(&mut self, event_loop: &ActiveEventLoop) {
        debug_assert!(self.run_control.resolution.request_is_needed());
        let requested = self
            .run_control
            .requested_resolution()
            .expect("a requested resolution exists while its request is pending submission");
        let window = Arc::clone(
            self.window
                .as_ref()
                .expect("the window is stored before its resolution is requested"),
        );
        let current = window.inner_size();
        if current == PhysicalSize::new(requested.width, requested.height) {
            self.run_control
                .resolution
                .resolve_request(Some(requested))
                .expect("the current extent was checked against the requested extent");
            self.initialize_rendering_after_resolution_verified(event_loop);
            return;
        }
        let immediate = window
            .request_inner_size(PhysicalSize::new(requested.width, requested.height))
            .map(|actual| RenderResolution::new(actual.width, actual.height));
        match self.run_control.resolution.resolve_request(immediate) {
            Ok(RenderResolutionRequestOutcome::Verified) => {
                self.initialize_rendering_after_resolution_verified(event_loop);
            }
            Ok(RenderResolutionRequestOutcome::Pending) => {}
            Err(mismatch) => self.fail_resolution_mismatch(event_loop, mismatch),
        }
    }

    fn verify_current_resolution(&mut self, event_loop: &ActiveEventLoop) {
        debug_assert!(
            self.run_control
                .resolution
                .current_extent_verification_is_needed()
        );
        let actual = self
            .window
            .as_ref()
            .expect("the window exists while its physical extent is verified")
            .inner_size();
        if let Err(mismatch) = self
            .run_control
            .resolution
            .verify_current_extent(RenderResolution::new(actual.width, actual.height))
        {
            self.fail_resolution_mismatch(event_loop, mismatch);
            return;
        }
        self.initialize_rendering_after_resolution_verified(event_loop);
    }

    fn initialize_rendering_after_resolution_verified(&mut self, event_loop: &ActiveEventLoop) {
        if self.renderer.is_some() {
            return;
        }
        debug_assert!(self.run_control.resolution_is_ready());
        let window = Arc::clone(
            self.window
                .as_ref()
                .expect("the window is stored before resolution verification"),
        );
        // WGI enumeration needs a process-owned, focus-capable window before gilrs is created.
        window.focus_window();
        if !self.initialize_input_after_window(event_loop) {
            return;
        }
        let presentation_asset = match &self.presentation {
            PresentationModel::Glb {
                asset,
                articulation,
            } => PresentationAsset::ArticulatedGlb {
                asset,
                articulation,
            },
            PresentationModel::Procedural(mesh) if self.rv2_6_validation.is_some() => {
                PresentationAsset::ValidationTarget(mesh)
            }
            PresentationModel::Procedural(mesh) => PresentationAsset::Procedural(mesh),
        };
        let renderer_result = if let Some(validation) = self.rv2_6_validation {
            pollster::block_on(DesktopRenderer::new_v2_for_rv2_6_validation(
                Arc::clone(&window),
                presentation_asset,
                self.ground_below_render_origin_m,
                self.terrain_mode,
                Some(self.scenery_preset),
                self.camera_config,
                validation.aerial_perspective_enabled,
            ))
        } else {
            pollster::block_on(DesktopRenderer::new_with_presentation(
                self.renderer_version,
                Arc::clone(&window),
                presentation_asset,
                self.ground_below_render_origin_m,
                self.terrain_mode,
                Some(self.scenery_preset),
                self.camera_config,
            ))
        };
        let mut renderer = match renderer_result {
            Ok(renderer) => renderer,
            Err(error) => {
                self.fail(
                    event_loop,
                    RenderRuntimeError::RendererInitialization(error),
                );
                return;
            }
        };
        renderer.set_show_debug_overlays(self.debug_overlays);
        renderer.set_terrain_debug_mode(self.terrain_debug);
        // G3D: presentation-only vegetation debug channel (final = production).
        renderer.set_vegetation_debug_mode(self.vegetation_debug);
        // G3B: exposure was already validated at CLI parse time; the setter is
        // a defensive no-change-on-invalid guard (presentation-only).
        if let Err(error) = renderer.set_exposure_ev(self.exposure_ev) {
            self.fail(event_loop, RenderRuntimeError::ExposureValidation(error));
            return;
        }
        self.renderer = Some(renderer);
        self.last_frame_time = Some(Instant::now());
        window.request_redraw();
    }

    fn finish_and_exit(&mut self, event_loop: &ActiveEventLoop) {
        if self.capture.is_some() && !self.capture_completed {
            self.runtime_error = Some(RenderRuntimeError::CaptureNotCompleted);
            event_loop.exit();
            return;
        }
        if self.visual_audit_out.is_some() && !self.visual_audit_completed {
            self.runtime_error = Some(RenderRuntimeError::VisualAuditNotCompleted);
            event_loop.exit();
            return;
        }
        if let Err(error) = self.save_recording() {
            self.runtime_error = Some(error);
        }
        event_loop.exit();
    }

    fn save_recording(&mut self) -> Result<(), RenderRuntimeError> {
        let Some(recorder) = self.replay_recorder.take() else {
            return Ok(());
        };
        let recording = recorder.finish();
        let json = recording.to_json_pretty()?;
        let path = self
            .replay_output_path
            .as_ref()
            .expect("a recorder is created only when a replay output path exists");
        std::fs::write(path, json).map_err(|source| RenderRuntimeError::ReplayWrite {
            path: path.clone(),
            source,
        })
    }

    fn persist_visual_audit_for_presented_frame(
        &mut self,
        presentation_frame_index: u64,
        expected_extent: Option<(u32, u32)>,
    ) -> Result<(), RenderRuntimeError> {
        if self.visual_audit_frame != Some(presentation_frame_index) {
            return Ok(());
        }
        let path = self
            .visual_audit_out
            .as_ref()
            .expect("an audit frame is configured only with an output path");
        let audit = self
            .renderer
            .as_ref()
            .and_then(DesktopRenderer::runtime_visual_audit)
            .ok_or(RenderRuntimeError::VisualAuditUnavailable)?;
        persist_runtime_visual_audit(path, &audit, presentation_frame_index, expected_extent)?;
        self.visual_audit_completed = true;
        Ok(())
    }

    fn reset_flight_session(
        &mut self,
        timing_baseline: Instant,
    ) -> Result<FlightResetOutcome, RenderRuntimeError> {
        if self.replay_recorder.is_some() {
            return Ok(FlightResetOutcome::RefusedWhileRecording);
        }

        let mut reset_simulation = AircraftSimulation::new(
            self.simulation.model().clone(),
            *self.simulation.config(),
            self.initial_rigid_state,
        )
        .map_err(RenderRuntimeError::FlightResetSimulation)?;
        if self.ground_start {
            reset_simulation.refresh_ground_diagnostics(GroundCommand::new(0.0, 0.0));
        }
        let reset_fixed_step = FixedStepAccumulator::new(
            PHYSICS_DT,
            MAXIMUM_FRAME_DELTA,
            MAXIMUM_PHYSICS_STEPS_PER_FRAME,
        )
        .map_err(RenderRuntimeError::FlightResetScheduling)?;
        if let Some(input_mode) = self.input_mode.as_mut() {
            input_mode
                .reset_flight_controls(self.initial_throttle)
                .map_err(RenderRuntimeError::FlightResetInput)?;
        }

        self.simulation = reset_simulation;
        self.render_snapshots = AircraftRenderSnapshotBuffer::new(AircraftRenderSnapshot::initial(
            &self.initial_rigid_state,
        ));
        self.fixed_step = reset_fixed_step;
        self.last_frame_time = Some(timing_baseline);
        if let Some(renderer) = self.renderer.as_mut() {
            renderer.invalidate_temporal_history();
        }
        Ok(FlightResetOutcome::Reset)
    }

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        if !self.run_control.resolution_is_ready() {
            return;
        }
        let now = Instant::now();
        let frame_delta = self
            .last_frame_time
            .replace(now)
            .map_or(Duration::ZERO, |previous| {
                now.saturating_duration_since(previous)
            });
        let step_plan = self.fixed_step.advance(frame_delta);
        if self.input_mode.is_none() || self.input_backend.is_none() {
            self.fail(event_loop, RenderRuntimeError::InputNotInitialized);
            return;
        }
        let input_mode = self
            .input_mode
            .as_mut()
            .expect("input mode presence checked above");
        let input_backend = self
            .input_backend
            .as_mut()
            .expect("input backend presence checked above");
        match input_mode.poll_hardware(input_backend) {
            Ok(Some(message)) => println!("{message}"),
            Ok(None) => {}
            Err(error) => {
                self.fail(event_loop, error.into());
                return;
            }
        }
        let physics_steps = if self.rv2_6_validation.is_some() {
            0
        } else {
            step_plan.physics_steps()
        };
        for _ in 0..physics_steps {
            let input = match input_mode.sample(PHYSICS_DT.as_secs_f64()) {
                Ok(input) => input,
                Err(error) => {
                    self.fail(event_loop, error.into());
                    return;
                }
            };
            let snapshot =
                match advance_aircraft(&mut self.simulation, &mut self.replay_recorder, input) {
                    Ok(snapshot) => snapshot,
                    Err(error) => {
                        self.fail(event_loop, error.into());
                        return;
                    }
                };
            self.render_snapshots
                .push(AircraftRenderSnapshot::post_step(
                    &snapshot,
                    self.simulation.model(),
                ));
        }
        if step_plan.dropped_time_s() > 0.0 {
            warn!(
                dropped_time_s = step_plan.dropped_time_s(),
                "render loop discarded wall-clock backlog while preserving fixed physics dt"
            );
        }

        let alpha = interpolation_alpha(step_plan.remainder(), self.fixed_step.physics_dt());
        let snapshot = self.render_snapshots.interpolated_snapshot(alpha);
        let pose = match self
            .render_snapshots
            .interpolated_pose(alpha, self.render_origin_world_ned_m)
        {
            Ok(pose) => pose,
            Err(error) => {
                self.fail(event_loop, error.into());
                return;
            }
        };
        let frame = snapshot.render_frame(pose);
        let pending_frame = self.run_control.pending_frame();
        if pending_frame.capture_requested {
            let audit_requested = self.visual_audit_out.is_some();
            let capture_result = self.renderer.as_mut().map(|renderer| {
                if audit_requested {
                    renderer.render_and_capture_presentation(&frame, pending_frame.index)
                } else {
                    renderer.render_and_capture(&frame)
                }
            });
            match capture_result {
                Some(Ok(CaptureRenderOutcome::CapturedAndPresented(captured))) => {
                    let exit_after_present = self.run_control.commit_presented(pending_frame);
                    let artifact_result = self.capture.as_ref().map_or_else(
                        || Err(RenderRuntimeError::InvalidCapturedFrame),
                        |capture| {
                            persist_capture_artifacts(capture, pending_frame.index, &captured)
                                .map(|_| ())
                        },
                    );
                    if let Err(error) = artifact_result {
                        self.fail(event_loop, error);
                        return;
                    }
                    self.capture_completed = true;
                    if let Err(error) = self.persist_visual_audit_for_presented_frame(
                        pending_frame.index,
                        Some((captured.width, captured.height)),
                    ) {
                        self.fail(event_loop, error);
                        return;
                    }
                    if exit_after_present {
                        self.finish_and_exit(event_loop);
                        return;
                    }
                }
                Some(Ok(CaptureRenderOutcome::SkippedZeroExtent))
                | None
                | Some(Err(FrameCaptureError::Surface(SurfaceError::Occluded))) => {}
                Some(Err(FrameCaptureError::Surface(
                    SurfaceError::Lost | SurfaceError::Outdated,
                ))) => {
                    if let Some(renderer) = self.renderer.as_mut() {
                        renderer.reconfigure_surface();
                    }
                }
                Some(Err(FrameCaptureError::Surface(SurfaceError::Timeout))) => {
                    warn!("surface acquisition timed out; capture frame remains pending");
                }
                Some(Err(FrameCaptureError::Surface(SurfaceError::OutOfMemory))) => {
                    self.fail(event_loop, RenderRuntimeError::OutOfMemory);
                    return;
                }
                Some(Err(FrameCaptureError::Surface(SurfaceError::Validation))) => {
                    self.fail(event_loop, RenderRuntimeError::GpuValidation);
                    return;
                }
                Some(Err(error)) => {
                    self.fail(event_loop, RenderRuntimeError::FrameCapture(error));
                    return;
                }
            }
            if let Some(window) = self.window.as_ref() {
                window.request_redraw();
            }
            return;
        }
        let audit_requested = self.visual_audit_out.is_some();
        let render_result = self.renderer.as_mut().map(|renderer| {
            if audit_requested {
                renderer.render_presentation(&frame, pending_frame.index)
            } else {
                renderer.render(&frame)
            }
        });
        match render_result {
            Some(Ok(RenderOutcome::Presented)) => {
                let exit_after_present = self.run_control.commit_presented(pending_frame);
                let expected_extent = self
                    .window
                    .as_ref()
                    .map(|window| window.inner_size())
                    .map(|extent| (extent.width, extent.height));
                if let Err(error) = self
                    .persist_visual_audit_for_presented_frame(pending_frame.index, expected_extent)
                {
                    self.fail(event_loop, error);
                    return;
                }
                if exit_after_present {
                    self.finish_and_exit(event_loop);
                    return;
                }
            }
            Some(Ok(RenderOutcome::SkippedZeroExtent))
            | None
            | Some(Err(SurfaceError::Occluded)) => {}
            Some(Err(SurfaceError::Lost | SurfaceError::Outdated)) => {
                if let Some(renderer) = self.renderer.as_mut() {
                    renderer.reconfigure_surface();
                }
            }
            Some(Err(SurfaceError::Timeout)) => {
                warn!("surface acquisition timed out; skipping render frame");
            }
            Some(Err(SurfaceError::OutOfMemory)) => {
                self.fail(event_loop, RenderRuntimeError::OutOfMemory);
                return;
            }
            Some(Err(SurfaceError::Validation)) => {
                self.fail(event_loop, RenderRuntimeError::GpuValidation);
                return;
            }
        }
        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
        }
    }
}

impl ApplicationHandler for RenderApplication {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_some() {
            return;
        }
        self.run_control.resolution.begin_request();
        let attributes =
            Window::default_attributes().with_title("RC Simulation Engine — Manual Flight Viewer");
        let attributes = if let Some(resolution) = self.run_control.requested_resolution() {
            attributes
                .with_inner_size(PhysicalSize::new(resolution.width, resolution.height))
                .with_resizable(false)
        } else {
            attributes.with_inner_size(LogicalSize::new(
                DEFAULT_RENDER_WIDTH_LOGICAL,
                DEFAULT_RENDER_HEIGHT_LOGICAL,
            ))
        };
        let window = match event_loop.create_window(attributes) {
            Ok(window) => Arc::new(window),
            Err(error) => {
                self.fail(event_loop, RenderRuntimeError::WindowCreation(error));
                return;
            }
        };
        self.window = Some(Arc::clone(&window));
        if self.run_control.resolution_is_ready() {
            self.initialize_rendering_after_resolution_verified(event_loop);
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        window_id: WindowId,
        event: WindowEvent,
    ) {
        if self
            .window
            .as_ref()
            .is_none_or(|window| window.id() != window_id)
        {
            return;
        }
        match event {
            WindowEvent::CloseRequested => self.finish_and_exit(event_loop),
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && matches!(event.logical_key, Key::Named(NamedKey::Escape)) =>
            {
                self.finish_and_exit(event_loop);
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && !event.repeat
                    && matches!(event.physical_key, PhysicalKey::Code(KeyCode::Backspace)) =>
            {
                match self.reset_flight_session(Instant::now()) {
                    Ok(FlightResetOutcome::Reset) => {
                        println!("Flight session reset to its initial state");
                    }
                    Ok(FlightResetOutcome::RefusedWhileRecording) => {
                        println!(
                            "Backspace reset refused: replay recording is active; exit to save the recording"
                        );
                    }
                    Err(error) => self.fail(event_loop, error),
                }
            }
            WindowEvent::KeyboardInput { event, .. } => {
                if let Some(key) = keyboard_key(event.physical_key)
                    && let Some(input_mode) = self.input_mode.as_mut()
                {
                    input_mode.set_key(key, event.state == ElementState::Pressed);
                }
            }
            WindowEvent::ScaleFactorChanged {
                mut inner_size_writer,
                ..
            } => {
                if let Some(requested) = self.run_control.requested_resolution() {
                    match inner_size_writer
                        .request_inner_size(PhysicalSize::new(requested.width, requested.height))
                    {
                        Ok(()) => self
                            .run_control
                            .resolution
                            .await_current_extent_verification(),
                        Err(error) => self.fail(
                            event_loop,
                            RenderRuntimeError::RequestedRenderSizeUpdate(error),
                        ),
                    }
                }
            }
            WindowEvent::Resized(size) => {
                let actual = RenderResolution::new(size.width, size.height);
                if let Err(mismatch) = self.run_control.resolution.observe_resize(actual) {
                    self.fail_resolution_mismatch(event_loop, mismatch);
                    return;
                }
                if !self.run_control.resolution_is_ready() {
                    return;
                }
                if let Some(renderer) = self.renderer.as_mut() {
                    renderer.resize(size.width, size.height);
                } else {
                    self.initialize_rendering_after_resolution_verified(event_loop);
                }
            }
            WindowEvent::RedrawRequested => self.redraw(event_loop),
            _ => {}
        }
    }

    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        if self
            .run_control
            .resolution
            .current_extent_verification_is_needed()
        {
            self.verify_current_resolution(event_loop);
        }
        if self.run_control.resolution.request_is_needed() {
            self.request_configured_resolution(event_loop);
        }
        if self.run_control.resolution_is_ready()
            && let Some(window) = self.window.as_ref()
        {
            window.request_redraw();
        }
    }

    fn suspended(&mut self, _event_loop: &ActiveEventLoop) {
        self.renderer = None;
        self.window = None;
        self.last_frame_time = None;
    }

    fn exiting(&mut self, _event_loop: &ActiveEventLoop) {
        if self.runtime_error.is_none()
            && let Err(error) = self.save_recording()
        {
            self.runtime_error = Some(error);
        }
    }
}

fn advance_aircraft(
    simulation: &mut AircraftSimulation,
    recorder: &mut Option<AircraftReplayRecorder>,
    input: PilotInput,
) -> Result<AircraftSnapshot, AircraftReplayError> {
    let step_index = simulation.step_index();
    if let Some(recorder) = recorder {
        recorder.record(simulation, step_index, input)
    } else {
        Ok(simulation.step(&input))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FlightResetOutcome {
    Reset,
    RefusedWhileRecording,
}

fn resolve_presentation_path(model_path: &Path, glb_path: &str) -> PathBuf {
    model_path
        .parent()
        .unwrap_or_else(|| Path::new(""))
        .join(glb_path)
}

fn resolve_presentation_model(
    model_path: &Path,
    presentation: Option<&PresentationMetadata>,
) -> Result<PresentationModel, RenderAppError> {
    let Some(presentation) = presentation else {
        return Ok(PresentationModel::Procedural(aircraft_mesh()));
    };
    let path = resolve_presentation_path(model_path, presentation.glb_path());
    let asset = load_glb_asset(&path).map_err(|source| RenderAppError::PresentationAsset {
        path,
        source: Box::new(source),
    })?;
    let articulation = articulation_plan(presentation, asset.primitives.len())?;
    Ok(PresentationModel::Glb {
        asset,
        articulation,
    })
}

fn articulation_plan(
    presentation: &PresentationMetadata,
    primitive_count: usize,
) -> Result<GlbArticulationPlan, GlbArticulationError> {
    let mappings = presentation.articulated_surfaces().iter().map(|mapping| {
        let surface = match mapping.surface() {
            PresentationSurface::LeftAileron => SurfaceId::LeftAileron,
            PresentationSurface::RightAileron => SurfaceId::RightAileron,
            PresentationSurface::Elevator => SurfaceId::Elevator,
            PresentationSurface::Rudder => SurfaceId::Rudder,
        };
        let hinge = SurfaceHinge::new(
            surface,
            mapping.hinge_origin_render_body_m(),
            mapping.hinge_axis_render_body(),
            mapping.visual_gain(),
        )
        .expect("model loading validates presentation hinge metadata");
        (mapping.visual_primitive_index(), hinge)
    });
    GlbArticulationPlan::from_mappings(primitive_count, mappings)
}

#[derive(Debug, Clone, PartialEq)]
struct GroundStartInitialization {
    state: RigidBodyState,
    ground_below_render_origin_m: f32,
    ground_evaluation: GroundEvaluation,
}

/// Places a level, stationary aircraft at the unique vertical compression
/// where the landing-gear springs support its weight on the flat physics plane.
/// The fixed-iteration bisection is startup-only and deterministic.
fn supported_ground_start(
    model: &AircraftModel,
) -> Result<GroundStartInitialization, RenderAppError> {
    let gear = model.landing_gear();
    if gear.is_empty() {
        return Err(RenderAppError::GroundStartWithoutLandingGear {
            model_id: model.model_id().to_owned(),
        });
    }

    let minimum_bottom_body_z = gear
        .iter()
        .map(|contact| {
            let contact = contact.contact();
            contact.position_body_m.z + contact.wheel_radius_m
        })
        .fold(f64::INFINITY, f64::min);
    let maximum_bottom_body_z = gear
        .iter()
        .map(|contact| {
            let contact = contact.contact();
            contact.position_body_m.z + contact.wheel_radius_m
        })
        .fold(f64::NEG_INFINITY, f64::max);
    let minimum_stiffness = gear
        .iter()
        .map(|contact| contact.contact().stiffness_n_per_m)
        .fold(f64::INFINITY, f64::min);
    let weight_n = model.rigid_body().mass_kg() * DEFAULT_GRAVITY_MPS2;

    // Upper bound is just clear of every wheel. The lower bound guarantees
    // at least one spring alone would exceed the aircraft weight.
    let mut unsupported_height_m = maximum_bottom_body_z;
    let mut overcompressed_height_m = minimum_bottom_body_z - weight_n / minimum_stiffness;
    for _ in 0..96 {
        let candidate_height_m = 0.5 * (overcompressed_height_m + unsupported_height_m);
        let normal_force_n = gear
            .iter()
            .map(|contact| {
                let contact = contact.contact();
                let bottom_body_z = contact.position_body_m.z + contact.wheel_radius_m;
                contact.stiffness_n_per_m * (bottom_body_z - candidate_height_m).max(0.0)
            })
            .sum::<f64>();
        if normal_force_n > weight_n {
            overcompressed_height_m = candidate_height_m;
        } else {
            unsupported_height_m = candidate_height_m;
        }
    }
    let cg_height_m = 0.5 * (overcompressed_height_m + unsupported_height_m);
    if !cg_height_m.is_finite() || cg_height_m <= 0.0 || cg_height_m > f64::from(f32::MAX) {
        return Err(RenderAppError::InvalidGroundStart {
            model_id: model.model_id().to_owned(),
            reason: "computed CG height above the ground plane is not finite and positive",
        });
    }
    let state = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -cg_height_m),
        linear_velocity_world_mps: Vec3::zeros(),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let contacts = model.gear_contacts();
    let ground_evaluation = evaluate_ground_wrench(
        &state,
        &contacts,
        &GroundSurface::Flat(FlatGroundPlane::default()),
        &GroundCommand::new(0.0, 0.0),
    );
    if !ground_evaluation.weight_on_wheels() {
        return Err(RenderAppError::InvalidGroundStart {
            model_id: model.model_id().to_owned(),
            reason: "computed state has no active physical ground contact",
        });
    }
    Ok(GroundStartInitialization {
        state,
        ground_below_render_origin_m: cg_height_m as f32,
        ground_evaluation,
    })
}

fn render_initial_state(altitude_m: f64, airspeed_mps: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -altitude_m),
        linear_velocity_world_mps: Vec3::new(airspeed_mps, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn print_manual_flight_startup(
    model_id: &str,
    fingerprint: &AircraftModelFingerprint,
    initial_state: &RigidBodyState,
    throttle: f64,
    ground_start: bool,
    initial_weight_on_wheels: bool,
    terrain_mode: RenderTerrainMode,
) {
    println!("Manual flight controls:");
    println!("A/D = roll");
    println!("W/S = pitch");
    println!("Q/E = yaw");
    println!("R/F = throttle");
    println!("Backspace = reset flight");
    println!("ESC = exit");
    println!();
    println!("model ID: {model_id}");
    print!("physics fingerprint: ");
    for byte in fingerprint.as_bytes() {
        print!("{byte:02x}");
    }
    println!();
    println!("physics rate: {DEFAULT_PHYSICS_HZ} Hz");
    println!(
        "initial altitude: {:.3} m",
        -initial_state.position_world_m.z
    );
    println!(
        "initial airspeed: {:.3} m/s",
        initial_state.linear_velocity_world_mps.norm()
    );
    println!("initial throttle: {throttle:.3}");
    println!("ground_start={ground_start}");
    println!("initial_weight_on_wheels={initial_weight_on_wheels}");
    println!("terrain_mode={}", terrain_mode.as_str());
}

fn keyboard_key(physical_key: PhysicalKey) -> Option<KeyboardKey> {
    match physical_key {
        PhysicalKey::Code(KeyCode::KeyA) => Some(KeyboardKey::RollLeft),
        PhysicalKey::Code(KeyCode::KeyD) => Some(KeyboardKey::RollRight),
        PhysicalKey::Code(KeyCode::KeyW) => Some(KeyboardKey::PitchUp),
        PhysicalKey::Code(KeyCode::KeyS) => Some(KeyboardKey::PitchDown),
        PhysicalKey::Code(KeyCode::KeyQ) => Some(KeyboardKey::YawLeft),
        PhysicalKey::Code(KeyCode::KeyE) => Some(KeyboardKey::YawRight),
        PhysicalKey::Code(KeyCode::KeyR) => Some(KeyboardKey::ThrottleIncrease),
        PhysicalKey::Code(KeyCode::KeyF) => Some(KeyboardKey::ThrottleDecrease),
        _ => None,
    }
}

fn vector_to_array(vector: Vec3) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

// ---------------------------------------------------------------------------
// Camera CLI helpers (presentation-side).
// ---------------------------------------------------------------------------

/// Parse `x,y,z` into a finite `[f32; 3]`, or `None` on malformed input.
fn parse_position(value: &str) -> Option<[f32; 3]> {
    let mut parts = value.split(',');
    let x = parts.next()?.trim().parse::<f32>().ok()?;
    let y = parts.next()?.trim().parse::<f32>().ok()?;
    let z = parts.next()?.trim().parse::<f32>().ok()?;
    if parts.next().is_some() || ![x, y, z].iter().all(|v| v.is_finite()) {
        return None;
    }
    Some([x, y, z])
}

#[cfg(test)]
mod tests {
    use super::*;
    use platform::{
        CenteredAxisProfile, CenteredCalibration, Control, ControllerProfile, HardwareAxis,
        ProfileAxes, RawControllerState, ThrottleAxisProfile, ThrottleCalibration,
    };
    use replay::{AircraftReplayPlayer, AircraftReplayRecording};

    fn repository_model_path(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    fn acro_model_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/acro_electric_01/model.json")
    }

    fn play_options_for_test() -> RenderOptions {
        let mut options = RenderOptions::parse_play(std::iter::empty()).unwrap();
        options.model_path = acro_model_path();
        options
    }

    fn render_options_for_test() -> RenderOptions {
        let mut options = RenderOptions::parse(std::iter::empty()).unwrap();
        options.model_path = acro_model_path();
        options
    }

    fn parse_render_camera(arguments: &[&str]) -> Result<CameraSelection, RenderAppError> {
        RenderOptions::parse(arguments.iter().map(|argument| (*argument).to_owned()))
            .map(|options| options.camera)
    }

    fn assert_camera_orders_equal(first: &[&str], second: &[&str]) {
        assert_eq!(
            parse_render_camera(first).unwrap(),
            parse_render_camera(second).unwrap(),
            "camera result differs for {first:?} and {second:?}"
        );
    }

    fn acro_model_with_presentation_path(glb_path: &str) -> model::AircraftModel {
        let mut value: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(acro_model_path()).unwrap()).unwrap();
        value["presentation"]["glb_path"] = serde_json::json!(glb_path);
        model::AircraftModelLoader::from_json_str(&value.to_string()).unwrap()
    }

    #[test]
    fn throttle_parser_accepts_bounds_and_rejects_invalid_values() {
        for value in ["0", "0.5", "1"] {
            assert!(
                RenderOptions::parse(["--throttle".to_owned(), value.to_owned()].into_iter())
                    .is_ok()
            );
        }
        for value in ["-0.1", "1.1", "NaN", "not-a-number"] {
            assert!(matches!(
                RenderOptions::parse(["--throttle".to_owned(), value.to_owned()].into_iter()),
                Err(RenderAppError::InvalidThrottle(_))
            ));
        }
    }

    #[test]
    fn render_control_defaults_preserve_historical_interactive_window() {
        let options = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(options.render_resolution, None);
        assert_eq!(options.exit_after_frame, None);
        assert_eq!(options.capture, None);
        assert_eq!(options.visual_audit_out, None);
        assert_eq!(DEFAULT_RENDER_WIDTH_LOGICAL, 1_280.0);
        assert_eq!(DEFAULT_RENDER_HEIGHT_LOGICAL, 720.0);
    }

    #[test]
    fn complete_png_capture_cli_group_parses() {
        let options = RenderOptions::parse(
            [
                "--capture-frame",
                "10",
                "--capture-out",
                "capture.png",
                "--capture-format",
                "png",
                "--capture-receipt-out",
                "receipt.json",
                "--exit-after-frame",
                "10",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert_eq!(
            options.capture,
            Some(CaptureConfig {
                presentation_frame_index: 10,
                image_path: PathBuf::from("capture.png"),
                format: CaptureFormat::Png,
                receipt_path: Some(PathBuf::from("receipt.json")),
            })
        );
    }

    #[test]
    fn visual_audit_cli_requires_v2_and_a_controlled_frame() {
        assert!(matches!(
            RenderOptions::parse(
                [
                    "--visual-audit-out",
                    "audit.json",
                    "--exit-after-frame",
                    "10"
                ]
                .map(str::to_owned)
                .into_iter()
            ),
            Err(RenderAppError::VisualAuditRequiresV2)
        ));
        assert!(matches!(
            RenderOptions::parse(
                ["--renderer", "v2", "--visual-audit-out", "audit.json"]
                    .map(str::to_owned)
                    .into_iter()
            ),
            Err(RenderAppError::VisualAuditRequiresControlledFrame)
        ));
        let options = RenderOptions::parse(
            [
                "--renderer",
                "v2",
                "--visual-audit-out",
                "audit.json",
                "--exit-after-frame",
                "10",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert_eq!(options.visual_audit_out, Some(PathBuf::from("audit.json")));
    }

    #[test]
    fn visual_audit_path_must_not_collide_with_capture_outputs() {
        assert!(matches!(
            RenderOptions::parse(
                [
                    "--renderer",
                    "v2",
                    "--capture-frame",
                    "0",
                    "--capture-out",
                    "capture.png",
                    "--capture-format",
                    "png",
                    "--visual-audit-out",
                    "capture.png",
                ]
                .map(str::to_owned)
                .into_iter()
            ),
            Err(RenderAppError::ConflictingCaptureOutputs)
        ));
    }

    #[test]
    fn capture_cli_rejects_each_missing_required_member() {
        for (arguments, missing) in [
            (
                vec!["--capture-out", "capture.png", "--capture-format", "png"],
                "--capture-frame",
            ),
            (
                vec!["--capture-frame", "0", "--capture-format", "png"],
                "--capture-out",
            ),
            (
                vec!["--capture-frame", "0", "--capture-out", "capture.png"],
                "--capture-format",
            ),
        ] {
            assert!(matches!(
                RenderOptions::parse(arguments.into_iter().map(str::to_owned)),
                Err(RenderAppError::IncompleteCaptureOptions(actual)) if actual == missing
            ));
        }
    }

    #[test]
    fn capture_cli_rejects_unsupported_formats() {
        for format in ["jpg", "jpeg", "exr"] {
            assert!(matches!(
                RenderOptions::parse(
                    [
                        "--capture-frame",
                        "0",
                        "--capture-out",
                        "capture.bin",
                        "--capture-format",
                        format,
                    ]
                    .map(str::to_owned)
                    .into_iter()
                ),
                Err(RenderAppError::UnsupportedCaptureFormat(actual)) if actual == format
            ));
        }
    }

    #[test]
    fn capture_cli_rejects_colliding_image_receipt_and_temporary_paths() {
        for (image, receipt) in [
            ("capture.png", "capture.png"),
            ("capture.png", "capture.png.tmp"),
        ] {
            assert!(matches!(
                RenderOptions::parse(
                    [
                        "--capture-frame",
                        "0",
                        "--capture-out",
                        image,
                        "--capture-format",
                        "png",
                        "--capture-receipt-out",
                        receipt,
                    ]
                    .map(str::to_owned)
                    .into_iter()
                ),
                Err(RenderAppError::ConflictingCaptureOutputs)
            ));
        }
    }

    #[test]
    fn capture_exit_order_accepts_equal_or_later_and_rejects_earlier() {
        for exit_frame in [10_u64, 11] {
            assert!(
                RenderOptions::parse(
                    [
                        "--capture-frame",
                        "10",
                        "--capture-out",
                        "capture.png",
                        "--capture-format",
                        "png",
                        "--exit-after-frame",
                        &exit_frame.to_string(),
                    ]
                    .map(str::to_owned)
                    .into_iter()
                )
                .is_ok()
            );
        }
        assert!(matches!(
            RenderOptions::parse(
                [
                    "--capture-frame",
                    "10",
                    "--capture-out",
                    "capture.png",
                    "--capture-format",
                    "png",
                    "--exit-after-frame",
                    "9",
                ]
                .map(str::to_owned)
                .into_iter()
            ),
            Err(RenderAppError::ExitBeforeCaptureFrame {
                exit_frame: 9,
                capture_frame: 10
            })
        ));
    }

    #[test]
    fn explicit_render_resolution_parses_at_contract_bounds_for_v1_and_v2() {
        for (renderer, width, height) in [
            ("v1", MINIMUM_RENDER_WIDTH, MINIMUM_RENDER_HEIGHT),
            ("v2", MAXIMUM_RENDER_WIDTH, MAXIMUM_RENDER_HEIGHT),
        ] {
            let options = RenderOptions::parse(
                [
                    "--renderer".to_owned(),
                    renderer.to_owned(),
                    "--render-width".to_owned(),
                    width.to_string(),
                    "--render-height".to_owned(),
                    height.to_string(),
                ]
                .into_iter(),
            )
            .unwrap();
            assert_eq!(
                options.render_resolution,
                Some(RenderResolution::new(width, height))
            );
        }
    }

    #[test]
    fn explicit_render_resolution_requires_both_dimensions() {
        for arguments in [
            vec!["--render-width", "1920"],
            vec!["--render-height", "1080"],
        ] {
            assert!(matches!(
                RenderOptions::parse(arguments.into_iter().map(str::to_owned)),
                Err(RenderAppError::IncompleteRenderResolution)
            ));
        }
    }

    #[test]
    fn explicit_render_resolution_rejects_invalid_values() {
        for width in ["0", "319", "7681", "-1", "wide"] {
            assert!(matches!(
                RenderOptions::parse(
                    ["--render-width", width, "--render-height", "1080"]
                        .map(str::to_owned)
                        .into_iter()
                ),
                Err(RenderAppError::InvalidRenderWidth(_))
            ));
        }
        for height in ["0", "239", "4321", "-1", "tall"] {
            assert!(matches!(
                RenderOptions::parse(
                    ["--render-width", "1920", "--render-height", height]
                        .map(str::to_owned)
                        .into_iter()
                ),
                Err(RenderAppError::InvalidRenderHeight(_))
            ));
        }
    }

    #[test]
    fn resolution_enforcement_accepts_synchronous_exact_extent() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut state = RenderResolutionState::new(Some(requested));
        assert_eq!(state, RenderResolutionState::Requested(requested));
        assert_eq!(
            state.resolve_request(Some(requested)),
            Ok(RenderResolutionRequestOutcome::Verified)
        );
        assert_eq!(state, RenderResolutionState::Verified(requested));
        assert!(state.is_ready());
    }

    #[test]
    fn resolution_enforcement_waits_for_asynchronous_resize() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut state = RenderResolutionState::new(Some(requested));
        // Window-creation resize events are not treated as the result of a
        // request that has not been submitted yet.
        state
            .observe_resize(RenderResolution::new(1_424, 750))
            .unwrap();
        assert_eq!(state, RenderResolutionState::Requested(requested));
        assert_eq!(
            state.resolve_request(None),
            Ok(RenderResolutionRequestOutcome::Pending)
        );
        assert_eq!(state, RenderResolutionState::Pending(requested));
        assert!(!state.is_ready());
    }

    #[test]
    fn pending_resolution_becomes_verified_only_on_matching_resized_event() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut state = RenderResolutionState::new(Some(requested));
        state.resolve_request(None).unwrap();
        state.observe_resize(requested).unwrap();
        assert_eq!(state, RenderResolutionState::Verified(requested));
        assert!(state.is_ready());
    }

    #[test]
    fn scale_factor_update_is_verified_without_a_resized_event() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut state = RenderResolutionState::Verified(requested);
        state.await_current_extent_verification();
        assert_eq!(state, RenderResolutionState::VerifyCurrent(requested));
        assert!(!state.is_ready());
        assert!(!state.request_is_needed());

        state.verify_current_extent(requested).unwrap();
        assert_eq!(state, RenderResolutionState::Verified(requested));
        assert!(state.is_ready());
        assert!(!state.request_is_needed());
    }

    #[test]
    fn scale_factor_update_can_be_verified_by_a_matching_resized_event() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut state = RenderResolutionState::Verified(requested);
        state.await_current_extent_verification();
        state.observe_resize(requested).unwrap();
        assert_eq!(state, RenderResolutionState::Verified(requested));
    }

    #[test]
    fn resolution_enforcement_rejects_explicit_or_event_extent_mismatch() {
        let requested = RenderResolution::new(1_920, 1_080);
        let wrong = RenderResolution::new(1_280, 720);

        let mut synchronous = RenderResolutionState::new(Some(requested));
        assert_eq!(
            synchronous.resolve_request(Some(wrong)),
            Err(RenderResolutionMismatch {
                requested,
                actual: wrong,
            })
        );

        let mut asynchronous = RenderResolutionState::new(Some(requested));
        asynchronous.resolve_request(None).unwrap();
        assert_eq!(
            asynchronous.observe_resize(wrong),
            Err(RenderResolutionMismatch {
                requested,
                actual: wrong,
            })
        );
        assert_eq!(asynchronous, RenderResolutionState::Pending(requested));

        let mut scale_factor = RenderResolutionState::Verified(requested);
        scale_factor.await_current_extent_verification();
        assert_eq!(
            scale_factor.verify_current_extent(wrong),
            Err(RenderResolutionMismatch {
                requested,
                actual: wrong,
            })
        );
        assert_eq!(
            scale_factor,
            RenderResolutionState::VerifyCurrent(requested)
        );
        assert!(!scale_factor.request_is_needed());
    }

    #[test]
    fn resolution_enforcement_is_disabled_for_historical_default() {
        let mut state = RenderResolutionState::new(None);
        assert_eq!(state, RenderResolutionState::Disabled);
        assert!(state.is_ready());
        state
            .observe_resize(RenderResolution::new(1_424, 750))
            .unwrap();
        assert_eq!(state, RenderResolutionState::Disabled);
    }

    #[test]
    fn resolution_state_transitions_do_not_advance_presentation_counter() {
        let requested = RenderResolution::new(1_920, 1_080);
        let mut control = RenderRunControl::new(Some(requested), Some(0), None);
        let frame_zero = control.pending_frame();

        assert_eq!(
            control.resolution.resolve_request(None),
            Ok(RenderResolutionRequestOutcome::Pending)
        );
        control.resolution.observe_resize(requested).unwrap();
        control.resolution.await_current_extent_verification();
        control.resolution.verify_current_extent(requested).unwrap();

        assert_eq!(control.pending_frame(), frame_zero);
        assert_eq!(control.next_presentation_frame, 0);
    }

    #[test]
    fn exit_after_frame_parser_accepts_zero_and_one_and_rejects_invalid_values() {
        for frame in ["0", "1"] {
            let options =
                RenderOptions::parse(["--exit-after-frame", frame].map(str::to_owned).into_iter())
                    .unwrap();
            assert_eq!(options.exit_after_frame, Some(frame.parse().unwrap()));
        }
        for frame in ["-1", "1.5", "later"] {
            assert!(matches!(
                RenderOptions::parse(["--exit-after-frame", frame].map(str::to_owned).into_iter()),
                Err(RenderAppError::InvalidExitAfterFrame(_))
            ));
        }
    }

    #[test]
    fn presentation_frame_index_is_zero_based_and_committed_only_after_present() {
        let mut control = RenderRunControl::new(None, None, None);
        let frame_zero = control.pending_frame();
        assert_eq!(frame_zero.index, 0);
        assert!(!frame_zero.exit_after_present);

        // A skipped/failed renderer call does not commit and therefore retries
        // the same presentation index.
        assert_eq!(control.pending_frame(), frame_zero);
        assert!(!control.commit_presented(frame_zero));
        assert_eq!(control.pending_frame().index, 1);
    }

    #[test]
    fn auto_exit_schedule_triggers_after_configured_presented_frame() {
        let mut exit_after_zero = RenderRunControl::new(None, Some(0), None);
        let frame_zero = exit_after_zero.pending_frame();
        assert!(frame_zero.exit_after_present);
        assert!(exit_after_zero.commit_presented(frame_zero));

        let mut exit_after_one = RenderRunControl::new(None, Some(1), None);
        let frame_zero = exit_after_one.pending_frame();
        assert!(!exit_after_one.commit_presented(frame_zero));
        let frame_one = exit_after_one.pending_frame();
        assert_eq!(frame_one.index, 1);
        assert!(exit_after_one.commit_presented(frame_one));
    }

    #[test]
    fn capture_schedule_matches_exact_presentation_frame_only_once() {
        let mut control = RenderRunControl::new(None, None, Some(1));
        let before = control.pending_frame();
        assert_eq!(before.index, 0);
        assert!(!before.capture_requested);
        assert!(!control.commit_presented(before));

        let capture = control.pending_frame();
        assert_eq!(capture.index, 1);
        assert!(capture.capture_requested);
        assert_eq!(control.pending_frame(), capture);
        assert!(!control.commit_presented(capture));

        let after = control.pending_frame();
        assert_eq!(after.index, 2);
        assert!(!after.capture_requested);
    }

    fn capture_test_directory(label: &str) -> PathBuf {
        let unique = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        std::env::temp_dir().join(format!(
            "rcsim-vis0-c2a-{label}-{}-{unique}",
            std::process::id()
        ))
    }

    #[test]
    fn capture_startup_removes_stale_targets_and_temporary_files() {
        let directory = capture_test_directory("stale");
        fs::create_dir(&directory).unwrap();
        let image_path = directory.join("capture.png");
        let receipt_path = directory.join("receipt.json");
        let capture = CaptureConfig {
            presentation_frame_index: 0,
            image_path: image_path.clone(),
            format: CaptureFormat::Png,
            receipt_path: Some(receipt_path.clone()),
        };
        for path in [
            image_path.clone(),
            receipt_path.clone(),
            temporary_output_path(&image_path).unwrap(),
            temporary_output_path(&receipt_path).unwrap(),
        ] {
            fs::write(path, b"stale").unwrap();
        }

        prepare_capture_outputs(Some(&capture)).unwrap();

        assert!(!image_path.exists());
        assert!(!receipt_path.exists());
        assert!(!temporary_output_path(&image_path).unwrap().exists());
        assert!(!temporary_output_path(&receipt_path).unwrap().exists());
        fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn visual_audit_startup_removes_stale_target_and_temporary_file() {
        let directory = capture_test_directory("audit-stale");
        fs::create_dir(&directory).unwrap();
        let audit_path = directory.join("runtime_visual_audit.json");
        let temporary = temporary_output_path(&audit_path).unwrap();
        fs::write(&audit_path, b"stale").unwrap();
        fs::write(&temporary, b"partial").unwrap();

        prepare_visual_audit_output(Some(&audit_path)).unwrap();

        assert!(!audit_path.exists());
        assert!(!temporary.exists());
        fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn runtime_receipt_serializes_schema_v1() {
        let receipt = RuntimeCaptureReceipt {
            schema_version: "1.0.0",
            presentation_frame_index: 3,
            framebuffer_width: 2,
            framebuffer_height: 1,
            format: "png",
            image_path: "capture.png".to_owned(),
            image_sha256: "00".repeat(32),
            image_byte_size: 123,
        };
        let value = serde_json::to_value(receipt).unwrap();
        assert_eq!(value["schema_version"], "1.0.0");
        assert_eq!(value["presentation_frame_index"], 3);
        assert_eq!(value["framebuffer_width"], 2);
        assert_eq!(value["framebuffer_height"], 1);
        assert_eq!(value["format"], "png");
    }

    #[test]
    fn capture_receipt_uses_actual_frame_facts_and_png_bytes() {
        let directory = capture_test_directory("receipt");
        fs::create_dir(&directory).unwrap();
        let image_path = directory.join("capture.png");
        let receipt_path = directory.join("receipt.json");
        let capture = CaptureConfig {
            presentation_frame_index: 99,
            image_path: image_path.clone(),
            format: CaptureFormat::Png,
            receipt_path: Some(receipt_path.clone()),
        };
        let frame = CapturedFrame {
            width: 2,
            height: 1,
            rgba8: vec![255, 0, 0, 255, 0, 255, 0, 255],
        };
        prepare_capture_outputs(Some(&capture)).unwrap();
        let receipt = persist_capture_artifacts(&capture, 7, &frame).unwrap();
        let image_bytes = fs::read(&image_path).unwrap();
        let receipt_value: serde_json::Value =
            serde_json::from_slice(&fs::read(&receipt_path).unwrap()).unwrap();

        assert_eq!(receipt.presentation_frame_index, 7);
        assert_eq!(receipt.framebuffer_width, 2);
        assert_eq!(receipt.framebuffer_height, 1);
        assert_eq!(receipt.image_sha256, sha256_hex(&image_bytes));
        assert_eq!(receipt.image_byte_size, image_bytes.len() as u64);
        assert_eq!(receipt_value["presentation_frame_index"], 7);
        assert_eq!(receipt_value["framebuffer_width"], 2);
        assert_eq!(receipt_value["framebuffer_height"], 1);
        assert_eq!(receipt_value["image_sha256"], sha256_hex(&image_bytes));
        assert_eq!(receipt_value["image_byte_size"], image_bytes.len() as u64);
        let decoded = image::load_from_memory_with_format(&image_bytes, image::ImageFormat::Png)
            .unwrap()
            .to_rgba8();
        assert_eq!(decoded.dimensions(), (2, 1));
        assert_eq!(decoded.as_raw(), &frame.rgba8);

        fs::remove_file(image_path).unwrap();
        fs::remove_file(receipt_path).unwrap();
        fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn receipt_is_not_produced_when_captured_pixels_are_invalid() {
        let directory = capture_test_directory("failed");
        fs::create_dir(&directory).unwrap();
        let image_path = directory.join("capture.png");
        let receipt_path = directory.join("receipt.json");
        let capture = CaptureConfig {
            presentation_frame_index: 0,
            image_path: image_path.clone(),
            format: CaptureFormat::Png,
            receipt_path: Some(receipt_path.clone()),
        };
        let frame = CapturedFrame {
            width: 2,
            height: 2,
            rgba8: vec![0; 3],
        };
        fs::write(&receipt_path, b"stale receipt").unwrap();
        prepare_capture_outputs(Some(&capture)).unwrap();

        assert!(matches!(
            persist_capture_artifacts(&capture, 0, &frame),
            Err(RenderRuntimeError::InvalidCapturedFrame)
        ));
        assert!(!image_path.exists());
        assert!(!receipt_path.exists());
        fs::remove_dir(directory).unwrap();
    }

    #[test]
    fn default_run_control_never_auto_exits() {
        let mut control = RenderRunControl::new(None, None, None);
        for expected_index in 0..3 {
            let frame = control.pending_frame();
            assert_eq!(frame.index, expected_index);
            assert!(!control.commit_presented(frame));
        }
    }

    #[test]
    fn existing_render_cli_remains_valid_with_controller_diagnostics_present() {
        assert!(
            RenderOptions::parse(
                [
                    "--model",
                    "models/acro_electric_01/model.json",
                    "--throttle",
                    "0.55",
                    "--camera",
                    "pilot",
                    "--scenery",
                    "flying-field",
                ]
                .map(str::to_owned)
                .into_iter()
            )
            .is_ok()
        );
    }

    #[test]
    fn controller_profile_cli_parses_without_changing_legacy_default() {
        let legacy = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(legacy.controller_profile_path, None);

        let calibrated = RenderOptions::parse(
            ["--controller-profile", "controllers/test-radio.json"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        assert_eq!(
            calibrated.controller_profile_path,
            Some(PathBuf::from("controllers/test-radio.json"))
        );
        assert!(matches!(
            RenderOptions::parse(["--controller-profile".to_owned()].into_iter()),
            Err(RenderAppError::MissingArgumentValue("--controller-profile"))
        ));

        let play = RenderOptions::parse_play(
            ["--controller-profile", "controllers/test-radio.json"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        assert_eq!(
            play.controller_profile_path,
            calibrated.controller_profile_path
        );
    }

    #[test]
    fn legacy_viewer_mode_keeps_keyboard_fallback_semantics() {
        let mut mode = ViewerInputMode::Legacy {
            state: InputState::new(
                InputMapping::default(),
                KeyboardInputState::new(0.4).unwrap(),
            ),
            status: ControllerStatusTracker::new(None),
        };
        mode.set_key(KeyboardKey::RollRight, true);
        mode.set_key(KeyboardKey::ThrottleIncrease, true);
        let input = mode.sample(PHYSICS_DT.as_secs_f64()).unwrap();
        assert_eq!(input.roll(), 1.0);
        assert_eq!(input.throttle(), 0.401);
    }

    #[test]
    fn camera_options_select_pilot_and_chase_modes() {
        let pilot =
            RenderOptions::parse(["--camera", "pilot"].map(str::to_owned).into_iter()).unwrap();
        assert_eq!(
            pilot.camera,
            CameraSelection::Pilot {
                position_render_m: EXPLICIT_PILOT_POSITION_RENDER_M,
                vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
            }
        );

        let chase =
            RenderOptions::parse(["--camera", "chase"].map(str::to_owned).into_iter()).unwrap();
        assert_eq!(
            chase.camera,
            CameraSelection::Chase {
                distance_behind_m: EXPLICIT_CHASE_DISTANCE_M,
                height_above_m: EXPLICIT_CHASE_HEIGHT_M,
                vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
            }
        );

        let default = RenderOptions::parse(std::iter::empty()).unwrap();
        assert!(matches!(default.camera, CameraSelection::Pilot { .. }));
    }

    #[test]
    fn terrain_debug_option_parses_and_defaults_to_final() {
        let default = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(default.terrain_debug, TerrainDebugMode::Final);

        for (label, expected) in [
            ("final", TerrainDebugMode::Final),
            ("albedo", TerrainDebugMode::Albedo),
            ("normal", TerrainDebugMode::Normal),
            ("roughness", TerrainDebugMode::Roughness),
            ("macro", TerrainDebugMode::Macro),
            ("detail", TerrainDebugMode::Detail),
            ("region", TerrainDebugMode::Region),
        ] {
            let options =
                RenderOptions::parse(["--terrain-debug", label].map(str::to_owned).into_iter())
                    .unwrap();
            assert_eq!(options.terrain_debug, expected, "label {label}");
        }
    }

    #[test]
    fn terrain_debug_option_rejects_unknown_modes() {
        for value in ["FINAL", "metal", "", "normal-map", "5"] {
            assert!(
                matches!(
                    RenderOptions::parse(
                        ["--terrain-debug".to_owned(), value.to_owned()].into_iter()
                    ),
                    Err(RenderAppError::InvalidTerrainDebug(_))
                ),
                "value {value:?} must be rejected"
            );
        }
    }

    #[test]
    fn vegetation_debug_option_parses_all_modes_and_rejects_unknown() {
        for (label, expected) in [
            ("final", VegetationDebugMode::Final),
            ("lod", VegetationDebugMode::Lod),
            ("culling", VegetationDebugMode::Culling),
        ] {
            let options =
                RenderOptions::parse(["--vegetation-debug", label].map(str::to_owned).into_iter())
                    .unwrap();
            assert_eq!(options.vegetation_debug, expected, "label {label}");
        }
        for value in ["FINAL", "lods", "", "2", "na", "bounds"] {
            assert!(
                matches!(
                    RenderOptions::parse(
                        ["--vegetation-debug".to_owned(), value.to_owned()].into_iter()
                    ),
                    Err(RenderAppError::InvalidVegetationDebug(_))
                ),
                "value {value:?} must be rejected"
            );
        }
    }

    #[test]
    fn renderer_option_defaults_to_v1_for_render_and_play() {
        // RV2-1: omitting `--renderer` keeps the historical V1 backend for both
        // the `render` and `play` commands.
        assert_eq!(
            RenderOptions::parse(std::iter::empty()).unwrap().renderer,
            RendererVersion::V1
        );
        assert_eq!(
            RenderOptions::parse_play(std::iter::empty())
                .unwrap()
                .renderer,
            RendererVersion::V1
        );
    }

    #[test]
    fn renderer_option_selects_v1_and_v2() {
        assert_eq!(
            RenderOptions::parse(["--renderer", "v1"].map(str::to_owned).into_iter())
                .unwrap()
                .renderer,
            RendererVersion::V1
        );
        assert_eq!(
            RenderOptions::parse(["--renderer", "v2"].map(str::to_owned).into_iter())
                .unwrap()
                .renderer,
            RendererVersion::V2
        );
        // `play` shares the same parser and selection semantics.
        assert_eq!(
            RenderOptions::parse_play(["--renderer", "v2"].map(str::to_owned).into_iter())
                .unwrap()
                .renderer,
            RendererVersion::V2
        );
    }

    #[test]
    fn rv2_6_validation_scene_is_v2_only_fixed_and_defaults_ap_on() {
        for (label, expected_case, expected_camera, extent_m) in [
            (
                "near",
                Rv26ValidationCase::Near,
                CameraSelection::Pilot {
                    position_render_m: [0.0, 1.8, 5.0],
                    vertical_fov_deg: 55.0,
                },
                1.0,
            ),
            (
                "100m",
                Rv26ValidationCase::Distance100M,
                CameraSelection::Pilot {
                    position_render_m: [0.0, 1.8, 100.0],
                    vertical_fov_deg: 55.0,
                },
                3.0,
            ),
            (
                "500m",
                Rv26ValidationCase::Distance500M,
                CameraSelection::Pilot {
                    position_render_m: [0.0, 1.8, 500.0],
                    vertical_fov_deg: 55.0,
                },
                15.0,
            ),
            (
                "1000m",
                Rv26ValidationCase::Distance1000M,
                CameraSelection::Pilot {
                    position_render_m: [0.0, 1.8, 1_000.0],
                    vertical_fov_deg: 55.0,
                },
                30.0,
            ),
        ] {
            let options = RenderOptions::parse(
                ["--renderer", "v2", "--rv2-6-validation-scene", label]
                    .map(str::to_owned)
                    .into_iter(),
            )
            .unwrap();
            assert_eq!(
                options.rv2_6_validation,
                Some(Rv26ValidationConfig {
                    case: expected_case,
                    aerial_perspective_enabled: true,
                })
            );
            assert_eq!(options.camera, expected_camera);
            assert_eq!(expected_case.target_extent_m(), extent_m);
            assert_eq!(options.scenery, SceneryPreset::None);
            assert_eq!(options.throttle, 0.0);
            assert!(options.start_on_ground);
        }
    }

    #[test]
    fn rv2_6_validation_ap_off_is_scoped_to_the_validation_scene() {
        let options = RenderOptions::parse(
            [
                "--renderer",
                "v2",
                "--rv2-6-validation-scene",
                "frontlit",
                "--rv2-6-validation-ap",
                "off",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert_eq!(
            options.rv2_6_validation,
            Some(Rv26ValidationConfig {
                case: Rv26ValidationCase::FrontLit,
                aerial_perspective_enabled: false,
            })
        );

        assert!(matches!(
            RenderOptions::parse(
                ["--rv2-6-validation-ap", "off"]
                    .map(str::to_owned)
                    .into_iter()
            ),
            Err(RenderAppError::Rv26ValidationApRequiresScene)
        ));
        assert!(matches!(
            RenderOptions::parse(
                ["--renderer", "v1", "--rv2-6-validation-scene", "near"]
                    .map(str::to_owned)
                    .into_iter()
            ),
            Err(RenderAppError::Rv26ValidationRequiresV2)
        ));
        assert!(matches!(
            RenderOptions::parse(
                ["--renderer", "v2", "--rv2-6-validation-scene", "unknown"]
                    .map(str::to_owned)
                    .into_iter()
            ),
            Err(RenderAppError::InvalidRv26ValidationScene(_))
        ));
        assert!(matches!(
            RenderOptions::parse(
                [
                    "--renderer",
                    "v2",
                    "--rv2-6-validation-scene",
                    "near",
                    "--rv2-6-validation-ap",
                    "half",
                ]
                .map(str::to_owned)
                .into_iter()
            ),
            Err(RenderAppError::InvalidRv26ValidationAp(_))
        ));
    }

    #[test]
    fn renderer_option_rejects_unknown_value() {
        for value in ["foo", "v3", "V1", "", "v2 "] {
            assert!(
                matches!(
                    RenderOptions::parse(["--renderer".to_owned(), value.to_owned()].into_iter()),
                    Err(RenderAppError::InvalidRenderer(_))
                ),
                "value {value:?} must be rejected"
            );
        }
        // A missing value is a distinct, clear error.
        assert!(matches!(
            RenderOptions::parse(["--renderer".to_owned()].into_iter()),
            Err(RenderAppError::MissingArgumentValue("--renderer"))
        ));
    }

    #[test]
    fn renderer_option_is_order_independent_with_camera_option() {
        // The renderer selection must not depend on its position relative to
        // the camera/presentation options, and must not disturb them.
        let first = RenderOptions::parse(
            ["--renderer", "v2", "--camera", "chase"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        let second = RenderOptions::parse(
            ["--camera", "chase", "--renderer", "v2"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        assert_eq!(first.renderer, RendererVersion::V2);
        assert_eq!(second.renderer, RendererVersion::V2);
        assert_eq!(first.camera, second.camera);
        assert!(matches!(first.camera, CameraSelection::Chase { .. }));
    }

    #[test]
    fn play_defaults_keep_v1_reference_path_with_other_options() {
        // The V1 reference path stays the default even when other play options
        // are supplied without an explicit `--renderer`.
        let options = RenderOptions::parse_play(
            ["--scenery", "flying-field", "--debug-overlays"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        assert_eq!(options.renderer, RendererVersion::V1);
        assert_eq!(options.scenery, SceneryPreset::FlyingField);
        assert!(options.debug_overlays);
    }

    #[test]
    fn camera_options_apply_tuning_and_reject_invalid_values() {
        let tuned = RenderOptions::parse(
            [
                "--camera",
                "chase",
                "--camera-fov",
                "35",
                "--chase-distance-m",
                "8",
                "--chase-height-m",
                "2.5",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        match tuned.camera {
            CameraSelection::Chase {
                distance_behind_m,
                height_above_m,
                vertical_fov_deg,
            } => {
                assert_eq!(distance_behind_m, 8.0);
                assert_eq!(height_above_m, 2.5);
                assert_eq!(vertical_fov_deg, 35.0);
            }
            _ => panic!("expected chase selection"),
        }

        let pilot = RenderOptions::parse(
            ["--camera", "pilot", "--pilot-position", "4,1.7,30"]
                .map(str::to_owned)
                .into_iter(),
        )
        .unwrap();
        match pilot.camera {
            CameraSelection::Pilot {
                position_render_m, ..
            } => assert_eq!(position_render_m, [4.0, 1.7, 30.0]),
            _ => panic!("expected pilot selection"),
        }

        assert!(matches!(
            RenderOptions::parse(["--camera", "orbit"].map(str::to_owned).into_iter()),
            Err(RenderAppError::UnknownCamera(_))
        ));
        assert!(matches!(
            RenderOptions::parse(["--camera-fov", "200"].map(str::to_owned).into_iter()),
            Err(RenderAppError::InvalidCameraFov(_))
        ));
        assert!(matches!(
            RenderOptions::parse(["--chase-distance-m", "0"].map(str::to_owned).into_iter()),
            Err(RenderAppError::InvalidChaseDistance(_))
        ));
        assert!(matches!(
            RenderOptions::parse(["--pilot-position", "1,2"].map(str::to_owned).into_iter()),
            Err(RenderAppError::InvalidPilotPosition(_))
        ));
    }

    #[test]
    fn camera_option_permutations_resolve_to_the_same_selection() {
        assert_camera_orders_equal(
            &["--camera", "pilot", "--camera-fov", "42"],
            &["--camera-fov", "42", "--camera", "pilot"],
        );
        assert_camera_orders_equal(
            &["--camera", "chase", "--chase-distance-m", "8"],
            &["--chase-distance-m", "8", "--camera", "chase"],
        );
        assert_camera_orders_equal(
            &["--camera", "chase", "--chase-height-m", "2.5"],
            &["--chase-height-m", "2.5", "--camera", "chase"],
        );
        assert_camera_orders_equal(
            &["--camera", "pilot", "--pilot-position", "4,1.7,30"],
            &["--pilot-position", "4,1.7,30", "--camera", "pilot"],
        );
    }

    #[test]
    fn three_camera_tunings_are_order_independent() {
        let first = [
            "--camera",
            "chase",
            "--camera-fov",
            "35",
            "--chase-distance-m",
            "8",
            "--chase-height-m",
            "2.5",
        ];
        let second = [
            "--chase-height-m",
            "2.5",
            "--camera-fov",
            "35",
            "--chase-distance-m",
            "8",
            "--camera",
            "chase",
        ];
        assert_camera_orders_equal(&first, &second);
        assert_eq!(
            parse_render_camera(&first).unwrap(),
            CameraSelection::Chase {
                distance_behind_m: 8.0,
                height_above_m: 2.5,
                vertical_fov_deg: 35.0,
            }
        );
    }

    #[test]
    fn mode_specific_camera_options_reject_incompatible_final_modes() {
        for arguments in [
            ["--camera", "pilot", "--chase-distance-m", "8"],
            ["--chase-distance-m", "8", "--camera", "pilot"],
        ] {
            assert!(matches!(
                parse_render_camera(&arguments),
                Err(RenderAppError::IncompatibleCameraOption {
                    option: "--chase-distance-m",
                    required_mode: "chase",
                    actual_mode: "pilot",
                })
            ));
        }
        for arguments in [
            ["--camera", "pilot", "--chase-height-m", "2.5"],
            ["--chase-height-m", "2.5", "--camera", "pilot"],
        ] {
            assert!(matches!(
                parse_render_camera(&arguments),
                Err(RenderAppError::IncompatibleCameraOption {
                    option: "--chase-height-m",
                    required_mode: "chase",
                    actual_mode: "pilot",
                })
            ));
        }
        for arguments in [
            ["--camera", "chase", "--pilot-position", "4,1.7,30"],
            ["--pilot-position", "4,1.7,30", "--camera", "chase"],
        ] {
            assert!(matches!(
                parse_render_camera(&arguments),
                Err(RenderAppError::IncompatibleCameraOption {
                    option: "--pilot-position",
                    required_mode: "pilot",
                    actual_mode: "chase",
                })
            ));
        }

        assert!(matches!(
            parse_render_camera(&["--chase-distance-m", "8"]),
            Err(RenderAppError::IncompatibleCameraOption { .. })
        ));
        assert!(matches!(
            RenderOptions::parse_play(
                ["--pilot-position", "4,1.7,30"]
                    .map(str::to_owned)
                    .into_iter()
            ),
            Err(RenderAppError::IncompatibleCameraOption { .. })
        ));
    }

    #[test]
    fn camera_numeric_options_reject_nonfinite_out_of_range_and_malformed_values() {
        for value in ["10", "120"] {
            assert!(parse_render_camera(&["--camera-fov", value]).is_ok());
        }
        for value in ["0.001", "1000"] {
            assert!(
                parse_render_camera(&["--camera", "chase", "--chase-distance-m", value]).is_ok()
            );
        }
        for value in ["-100", "1000"] {
            assert!(parse_render_camera(&["--camera", "chase", "--chase-height-m", value]).is_ok());
        }
        for value in ["9.9", "120.1", "NaN", "inf", "not-a-number"] {
            assert!(matches!(
                parse_render_camera(&["--camera-fov", value]),
                Err(RenderAppError::InvalidCameraFov(_))
            ));
        }
        for value in ["0", "-1", "1000.1", "NaN", "inf", "not-a-number"] {
            assert!(matches!(
                parse_render_camera(&["--chase-distance-m", value]),
                Err(RenderAppError::InvalidChaseDistance(_))
            ));
        }
        for value in ["-100.1", "1000.1", "NaN", "inf", "not-a-number"] {
            assert!(matches!(
                parse_render_camera(&["--chase-height-m", value]),
                Err(RenderAppError::InvalidChaseHeight(_))
            ));
        }
        for value in ["1,2", "1,2,3,4", "1,NaN,3", "1,inf,3", "bad"] {
            assert!(matches!(
                parse_render_camera(&["--pilot-position", value]),
                Err(RenderAppError::InvalidPilotPosition(_))
            ));
        }
    }

    #[test]
    fn camera_config_conversion_is_presentation_only() {
        let selection = CameraSelection::Pilot {
            position_render_m: [1.0, 2.0, 3.0],
            vertical_fov_deg: 50.0,
        };
        let config = selection.into_camera_config();
        assert!(matches!(config, renderer::CameraConfig::Pilot { .. }));
        let chase = CameraSelection::Chase {
            distance_behind_m: 5.0,
            height_above_m: 2.0,
            vertical_fov_deg: 45.0,
        };
        assert!(matches!(
            chase.into_camera_config(),
            renderer::CameraConfig::Chase { .. }
        ));
    }

    #[test]
    fn altitude_and_airspeed_options_parse_with_manual_flight_defaults_and_overrides() {
        let defaults = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(defaults.model_path, PathBuf::from(DEFAULT_MODEL_PATH));
        assert_eq!(
            defaults.model_path,
            PathBuf::from("models/acro_electric_01/model.json")
        );
        assert_eq!(defaults.altitude_m, 30.0);
        assert_eq!(defaults.airspeed_mps, 18.0);
        assert!(!defaults.start_on_ground);

        let options = RenderOptions::parse(
            [
                "--altitude-m",
                "45.5",
                "--airspeed-mps",
                "22.25",
                "--throttle",
                "0.6",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert_eq!(options.altitude_m, 45.5);
        assert_eq!(options.airspeed_mps, 22.25);
        assert_eq!(options.throttle, 0.6);
    }

    #[test]
    fn play_preset_is_ground_ready_without_changing_render_defaults() {
        let render = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(render.model_path, PathBuf::from(DEFAULT_MODEL_PATH));
        assert_eq!(render.throttle, DEFAULT_THROTTLE);
        assert!(!render.start_on_ground);
        assert_eq!(render.scenery, SceneryPreset::None);
        assert_eq!(render.camera, CameraSelection::default());

        let play = RenderOptions::parse_play(std::iter::empty()).unwrap();
        assert_eq!(play.model_path, PathBuf::from(DEFAULT_MODEL_PATH));
        assert_eq!(play.throttle, 0.0);
        assert!(play.start_on_ground);
        assert_eq!(play.scenery, SceneryPreset::FlyingField);
        assert_eq!(
            play.camera,
            CameraSelection::Chase {
                distance_behind_m: PLAY_CHASE_DISTANCE_M,
                height_above_m: PLAY_CHASE_HEIGHT_M,
                vertical_fov_deg: 55.0,
            }
        );
        assert_eq!(PHYSICS_DT, Duration::from_millis(2));
        assert_eq!(DEFAULT_PHYSICS_HZ, 500);
    }

    #[test]
    fn play_preset_accepts_shared_render_overrides() {
        let play = RenderOptions::parse_play(
            [
                "--throttle",
                "0.2",
                "--scenery",
                "none",
                "--camera",
                "pilot",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert_eq!(play.throttle, 0.2);
        assert_eq!(play.scenery, SceneryPreset::None);
        assert!(matches!(play.camera, CameraSelection::Pilot { .. }));
        assert!(play.start_on_ground);
    }

    #[test]
    fn start_on_ground_flag_parses_explicitly() {
        let options = RenderOptions::parse(["--start-on-ground".to_owned()].into_iter()).unwrap();
        assert!(options.start_on_ground);
    }

    #[test]
    fn altitude_and_airspeed_options_reject_nonfinite_nonpositive_and_excessive_values() {
        for value in ["0", "-1", "NaN", "inf", "10000.1", "not-a-number"] {
            assert!(matches!(
                RenderOptions::parse(["--altitude-m".to_owned(), value.to_owned()].into_iter()),
                Err(RenderAppError::InvalidAltitude(_))
            ));
        }
        for value in ["0", "-1", "NaN", "inf", "200.1", "not-a-number"] {
            assert!(matches!(
                RenderOptions::parse(["--airspeed-mps".to_owned(), value.to_owned()].into_iter()),
                Err(RenderAppError::InvalidAirspeed(_))
            ));
        }
    }

    #[test]
    fn render_initial_conditions_apply_positive_altitude_as_negative_ned_z() {
        let state = render_initial_state(45.5, 22.25);
        assert_eq!(state.position_world_m, Vec3::new(0.0, 0.0, -45.5));
        assert_eq!(state.linear_velocity_world_mps, Vec3::new(22.25, 0.0, 0.0));
        assert_eq!(state.orientation_world_from_body, Orientation::identity());
        assert_eq!(state.angular_velocity_body_radps, Vec3::zeros());
    }

    #[test]
    fn acro_electric_01_starts_supported_on_the_physical_flat_plane() {
        let model_path = repository_model_path("models/acro_electric_01/model.json");
        let model = load_aircraft_model(&model_path).unwrap();
        assert_eq!(model.model_id(), "acro-electric-01");
        assert_eq!(model.landing_gear().len(), 3);
        let left_main = &model.landing_gear()[1];
        let right_main = &model.landing_gear()[2];
        assert_eq!(left_main.id(), "left-main");
        assert_eq!(right_main.id(), "right-main");
        assert_eq!(
            left_main.contact().position_body_m.x,
            right_main.contact().position_body_m.x
        );
        assert_eq!(
            left_main.contact().position_body_m.y,
            -right_main.contact().position_body_m.y
        );
        assert_eq!(
            left_main.contact().position_body_m.z,
            right_main.contact().position_body_m.z
        );

        let initialized = supported_ground_start(&model).unwrap();
        assert!(initialized.state.validate().is_ok());
        assert!(initialized.state.position_world_m.z < 0.0);
        assert_eq!(initialized.state.linear_velocity_world_mps, Vec3::zeros());
        assert_eq!(initialized.state.angular_velocity_body_radps, Vec3::zeros());
        assert!(initialized.ground_evaluation.weight_on_wheels());
        assert!(initialized.ground_evaluation.active_contacts > 0);
        let weight_n = model.rigid_body().mass_kg() * DEFAULT_GRAVITY_MPS2;
        assert!(
            (initialized.ground_evaluation.total_normal_force_n - weight_n).abs()
                <= 1.0e-10 * weight_n
        );
    }

    #[test]
    fn play_initial_ground_diagnostics_match_supported_start() {
        let model = load_aircraft_model(acro_model_path()).unwrap();
        let expected = supported_ground_start(&model).unwrap().ground_evaluation;
        let application = RenderApplication::new(play_options_for_test()).unwrap();

        assert_eq!(application.simulation.last_ground_evaluation(), &expected);
        assert!(
            application
                .simulation
                .last_ground_evaluation()
                .weight_on_wheels()
        );
        assert!(
            application
                .simulation
                .last_ground_evaluation()
                .active_contacts
                > 0
        );
    }

    #[test]
    fn airborne_render_initial_ground_diagnostics_remain_zero() {
        let application = RenderApplication::new(render_options_for_test()).unwrap();
        assert_eq!(
            application.simulation.last_ground_evaluation(),
            &GroundEvaluation::zero()
        );
    }

    #[test]
    fn dedicated_ground_demo_starts_supported_on_the_physical_flat_plane() {
        let model_path = repository_model_path("models/acro_electric_ground_demo/model.json");
        let model = load_aircraft_model(&model_path).unwrap();
        assert_eq!(
            model.classification(),
            model::AircraftClassification::SyntheticTest
        );
        assert!(model.reference_aircraft().is_none());
        let presentation = resolve_presentation_model(&model_path, model.presentation()).unwrap();
        match presentation {
            PresentationModel::Glb { asset, .. } => {
                assert!(!asset.primitives.is_empty());
                assert!(asset.total_vertex_count() > 0);
            }
            PresentationModel::Procedural(_) => {
                panic!("dedicated ground demo must use the production GLB path");
            }
        }

        let initialized = supported_ground_start(&model).unwrap();
        assert!(initialized.state.validate().is_ok());
        assert_eq!(initialized.state.linear_velocity_world_mps.x, 0.0);
        assert_eq!(initialized.state.linear_velocity_world_mps.y, 0.0);
        assert_eq!(initialized.state.linear_velocity_world_mps.z, 0.0);
        assert!(initialized.ground_evaluation.weight_on_wheels());
        assert!(initialized.ground_evaluation.active_contacts > 0);
        let weight_n = model.rigid_body().mass_kg() * DEFAULT_GRAVITY_MPS2;
        assert!(
            (initialized.ground_evaluation.total_normal_force_n - weight_n).abs()
                <= 1.0e-10 * weight_n
        );

        let render_origin = vector_to_array(initialized.state.position_world_m);
        let physical_ground_pose = renderer::world_ned_pose_to_render(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            render_origin,
        )
        .unwrap();
        assert_eq!(
            physical_ground_pose.translation_render_m()[1],
            -initialized.ground_below_render_origin_m
        );
        assert_eq!(RenderTerrainMode::Flat.as_str(), "flat");
    }

    #[test]
    fn initial_render_snapshot_adapter_preserves_raw_pose_semantics() {
        let state = RigidBodyState {
            position_world_m: Vec3::new(101.0, 202.0, 303.0),
            linear_velocity_world_mps: Vec3::zeros(),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let buffer = AircraftRenderSnapshotBuffer::new(AircraftRenderSnapshot::initial(&state));
        let pose = buffer
            .interpolated_pose(0.0, [100.0, 200.0, 300.0])
            .unwrap();
        assert_eq!(pose.translation_render_m(), [2.0, -3.0, -1.0]);
    }

    #[test]
    fn presentation_path_is_resolved_relative_to_model_directory() {
        let model_path = Path::new("models/acro_electric_01/model.json");
        assert_eq!(
            resolve_presentation_path(model_path, "aircraft.glb"),
            Path::new("models/acro_electric_01/aircraft.glb")
        );
    }

    #[test]
    fn resolve_presentation_model_without_glb_returns_procedural() {
        let model_path = Path::new("models/nonexistent/model.json");
        let result = resolve_presentation_model(model_path, None).unwrap();
        assert!(matches!(result, PresentationModel::Procedural(_)));
    }

    #[test]
    fn resolve_presentation_model_with_missing_glb_returns_explicit_error() {
        let model_path = Path::new("models/nonexistent/model.json");
        let model = acro_model_with_presentation_path("missing.glb");
        let result = resolve_presentation_model(model_path, model.presentation());
        assert!(matches!(
            result,
            Err(RenderAppError::PresentationAsset { .. })
        ));
    }

    #[test]
    fn resolve_presentation_model_with_real_glb_returns_glb_asset() {
        let model_path = acro_model_path();
        if !model_path.exists() {
            return; // Skip if model not available in CI.
        }
        let model = load_aircraft_model(&model_path).unwrap();
        let result = resolve_presentation_model(&model_path, model.presentation()).unwrap();
        match result {
            PresentationModel::Glb {
                asset,
                articulation,
            } => {
                assert!(!asset.primitives.is_empty());
                assert!(asset.total_vertex_count() > 0);
                assert_eq!(articulation.len(), asset.primitives.len());
            }
            PresentationModel::Procedural(_) => {
                panic!("expected Glb variant for real GLB model");
            }
        }
    }

    #[test]
    fn declared_valid_missing_and_invalid_assets_have_explicit_outcomes() {
        let model_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let valid_model = load_aircraft_model(&model_path).unwrap();
        let valid = resolve_presentation_model(&model_path, valid_model.presentation()).unwrap();
        match valid {
            PresentationModel::Glb { asset, .. } => assert!(!asset.primitives.is_empty()),
            PresentationModel::Procedural(_) => panic!("expected Glb for valid asset"),
        }
        let missing = acro_model_with_presentation_path("missing.glb");
        assert!(matches!(
            resolve_presentation_model(&model_path, missing.presentation()),
            Err(RenderAppError::PresentationAsset { .. })
        ));
        let invalid = acro_model_with_presentation_path("README.md");
        assert!(matches!(
            resolve_presentation_model(&model_path, invalid.presentation()),
            Err(RenderAppError::PresentationAsset { .. })
        ));
    }

    #[test]
    fn absent_presentation_metadata_uses_procedural_fallback() {
        let result = resolve_presentation_model(Path::new("model.json"), None).unwrap();
        match result {
            PresentationModel::Procedural(mesh) => {
                assert!(!mesh.vertices().is_empty());
                assert!(!mesh.indices().is_empty());
            }
            PresentationModel::Glb { .. } => panic!("expected procedural fallback"),
        }
    }

    #[test]
    fn presentation_asset_does_not_change_acro_physics_fingerprint() {
        let model_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let model = load_aircraft_model(&model_path).unwrap();
        let before = model.physics_fingerprint();
        let _presentation = resolve_presentation_model(&model_path, model.presentation()).unwrap();
        assert_eq!(model.physics_fingerprint(), before);
    }

    #[test]
    fn opaque_metadata_builds_explicit_glb_plan_without_changing_fingerprint() {
        let model_path = acro_model_path();
        let original = load_aircraft_model(&model_path).unwrap();
        let mut value: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&model_path).unwrap()).unwrap();
        for (binding, id) in value["control_surface_bindings"]
            .as_array_mut()
            .unwrap()
            .iter_mut()
            .zip(["p", "q", "r", "s"])
        {
            binding["id"] = serde_json::json!(id);
        }
        value["presentation"]["articulated_surfaces"] = serde_json::json!([
            {"visual_primitive_index":1,"surface":"left_aileron","control_surface_binding_id":"p","hinge_origin_render_body_m":[0.0,0.0,0.0],"hinge_axis_render_body":[1.0,0.0,0.0],"visual_gain":1.0},
            {"visual_primitive_index":2,"surface":"right_aileron","control_surface_binding_id":"q","hinge_origin_render_body_m":[0.0,0.0,0.0],"hinge_axis_render_body":[1.0,0.0,0.0],"visual_gain":1.0},
            {"visual_primitive_index":3,"surface":"elevator","control_surface_binding_id":"r","hinge_origin_render_body_m":[0.0,0.0,0.0],"hinge_axis_render_body":[1.0,0.0,0.0],"visual_gain":1.0},
            {"visual_primitive_index":4,"surface":"rudder","control_surface_binding_id":"s","hinge_origin_render_body_m":[0.0,0.0,0.0],"hinge_axis_render_body":[0.0,1.0,0.0],"visual_gain":1.0}
        ]);
        let explicit = model::AircraftModelLoader::from_json_str(&value.to_string()).unwrap();
        let plan = articulation_plan(explicit.presentation().unwrap(), 6).unwrap();
        assert!(matches!(plan.part(0), renderer::GlbPrimitivePart::Rigid));
        assert!(matches!(plan.part(5), renderer::GlbPrimitivePart::Rigid));
        for (index, surface) in [
            SurfaceId::LeftAileron,
            SurfaceId::RightAileron,
            SurfaceId::Elevator,
            SurfaceId::Rudder,
        ]
        .into_iter()
        .enumerate()
        {
            assert!(matches!(
                plan.part(index + 1),
                renderer::GlbPrimitivePart::Articulated { surface: actual, .. }
                    if *actual == surface
            ));
        }
        assert_eq!(
            explicit.physics_fingerprint(),
            original.physics_fingerprint()
        );
    }

    #[test]
    fn live_recording_uses_exact_sampled_input_and_s8a_step_semantics() {
        let model_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let model = load_aircraft_model(&model_path).unwrap();
        let config = AircraftSimulationConfig::from_physics_hz(
            DEFAULT_PHYSICS_HZ,
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap();
        let initial_state = RigidBodyState {
            position_world_m: Vec3::new(0.0, 0.0, -100.0),
            linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let mut simulation = AircraftSimulation::new(model.clone(), config, initial_state).unwrap();
        let mut recorder = Some(AircraftReplayRecorder::new(&simulation).unwrap());
        let mut input_state = InputState::default();
        input_state.set_key(KeyboardKey::PitchUp, true);
        input_state.set_key(KeyboardKey::ThrottleIncrease, true);
        let mut applied = Vec::new();
        for _ in 0..3 {
            let input = input_state.sample(0.002).unwrap();
            applied.push(input);
            advance_aircraft(&mut simulation, &mut recorder, input).unwrap();
        }
        let recording = recorder.take().unwrap().finish();
        for (step_index, (frame, expected_input)) in
            recording.frames().iter().zip(applied).enumerate()
        {
            assert_eq!(frame.step_index(), step_index as u64);
            assert_eq!(frame.pilot_input(), expected_input);
        }
        let json = recording.to_json_pretty().unwrap();
        let decoded = AircraftReplayRecording::from_json(&json).unwrap();
        let mut replayed = decoded.reconstruct_simulation(model).unwrap();
        let player = AircraftReplayPlayer::new(&decoded, &replayed).unwrap();
        assert_eq!(player.verify_all(&mut replayed).unwrap(), 3);
    }

    #[test]
    fn scenery_parser_accepts_flying_field() {
        let options =
            RenderOptions::parse(["--scenery".to_owned(), "flying-field".to_owned()].into_iter())
                .unwrap();
        assert_eq!(options.scenery, SceneryPreset::FlyingField);
    }

    #[test]
    fn scenery_parser_accepts_none() {
        let options =
            RenderOptions::parse(["--scenery".to_owned(), "none".to_owned()].into_iter()).unwrap();
        assert_eq!(options.scenery, SceneryPreset::None);
    }

    #[test]
    fn scenery_parser_rejects_invalid_value() {
        let result = RenderOptions::parse(["--scenery".to_owned(), "city".to_owned()].into_iter());
        assert!(matches!(result, Err(RenderAppError::InvalidScenery(_))));
    }

    #[test]
    fn scenery_default_is_none() {
        let options = RenderOptions::parse(std::iter::empty()).unwrap();
        assert_eq!(options.scenery, SceneryPreset::None);
    }

    // ── PF1 photo field CLI ────────────────────────────────────────────────
    //
    // Enforcement layer: the CLI is a pure parser. It supplies the surveyed
    // pilot eye as a DEFAULT so the photograph is not silently rendered from
    // the wrong place, but the fixed-eye rule itself lives in
    // `renderer::fixed_pilot_eye`, which the renderer applies when it builds
    // the Photo Field. Every test below therefore asserts both halves.

    fn parse(arguments: &[&str]) -> Result<RenderOptions, RenderAppError> {
        RenderOptions::parse(arguments.iter().map(|value| (*value).to_owned()))
    }

    #[test]
    fn photo_field_defaults_to_the_manifest_pilot_eye() {
        let options = parse(&["--scenery", "photo-field"]).unwrap();
        assert_eq!(options.scenery, SceneryPreset::PhotoField);
        let manifest_eye = renderer::photo_field_default_pilot_position().unwrap();
        match options.camera {
            CameraSelection::Pilot {
                position_render_m, ..
            } => {
                assert_eq!(position_render_m, manifest_eye);
                // Neither the explicit-pilot default nor the generic default
                // pilot eye may leak into a photographic field.
                assert_ne!(position_render_m, EXPLICIT_PILOT_POSITION_RENDER_M);
            }
            other => panic!("photo-field must stay a pilot camera, got {other:?}"),
        }
        // The renderer accepts the eye the CLI just chose.
        let config = renderer::embedded_photo_field_config().unwrap();
        assert_eq!(
            renderer::fixed_pilot_eye(&options.camera.into_camera_config(), &config),
            Ok(manifest_eye)
        );
    }

    #[test]
    fn photo_field_keeps_an_explicit_pilot_camera_mode_and_fov() {
        let options = parse(&[
            "--scenery",
            "photo-field",
            "--camera",
            "pilot",
            "--camera-fov",
            "55",
        ])
        .unwrap();
        assert_eq!(
            options.camera,
            CameraSelection::Pilot {
                position_render_m: renderer::photo_field_default_pilot_position().unwrap(),
                vertical_fov_deg: 55.0,
            }
        );
    }

    #[test]
    fn an_explicit_pilot_position_wins_and_the_renderer_rejects_a_mismatch() {
        // The CLI must not silently re-anchor an operator-supplied eye onto the
        // manifest position; the renderer rejects the mismatch instead.
        let options = parse(&[
            "--scenery",
            "photo-field",
            "--camera",
            "pilot",
            "--pilot-position",
            "1.0,2.0,3.0",
        ])
        .unwrap();
        assert_eq!(
            options.camera,
            CameraSelection::Pilot {
                position_render_m: [1.0, 2.0, 3.0],
                vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
            }
        );
        let config = renderer::embedded_photo_field_config().unwrap();
        assert_eq!(
            renderer::fixed_pilot_eye(&options.camera.into_camera_config(), &config),
            Err(renderer::PhotoFieldCameraError::PilotPositionMismatch {
                expected: config.pilot_position_render_m,
                actual: [1.0, 2.0, 3.0],
            })
        );
    }

    #[test]
    fn photo_field_with_a_chase_camera_parses_and_the_renderer_rejects_it() {
        // A Chase eye translates with the aircraft, which would make the
        // panorama swim. The CLI still parses it — rejecting it here would
        // duplicate the fixed-eye rule in a second layer — and the renderer
        // turns it into `RendererError::PhotoFieldCamera`.
        let options = parse(&["--scenery", "photo-field", "--camera", "chase"]).unwrap();
        assert_eq!(options.scenery, SceneryPreset::PhotoField);
        assert!(matches!(options.camera, CameraSelection::Chase { .. }));
        let config = renderer::embedded_photo_field_config().unwrap();
        assert_eq!(
            renderer::fixed_pilot_eye(&options.camera.into_camera_config(), &config),
            Err(renderer::PhotoFieldCameraError::ChaseRejected)
        );
    }

    #[test]
    fn the_manifest_pilot_eye_default_never_applies_to_another_preset() {
        // FlyingField and `none` keep their historical pilot defaults, so the
        // PF1 default cannot change any existing look.
        for preset in ["flying-field", "none"] {
            let options = parse(&["--scenery", preset, "--camera", "pilot"]).unwrap();
            assert_eq!(
                options.camera,
                CameraSelection::Pilot {
                    position_render_m: EXPLICIT_PILOT_POSITION_RENDER_M,
                    vertical_fov_deg: EXPLICIT_CAMERA_FOV_DEG,
                },
                "preset {preset}"
            );
        }
    }

    // ── Integration tests ──────────────────────────────────────────────────

    #[test]
    fn complete_render_options_parse_together() {
        let options = RenderOptions::parse(
            [
                "--model",
                "models/acro_electric_ground_demo/model.json",
                "--start-on-ground",
                "--scenery",
                "flying-field",
                "--camera",
                "pilot",
                "--throttle",
                "0",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert!(options.start_on_ground);
        assert_eq!(options.scenery, SceneryPreset::FlyingField);
        assert!(matches!(options.camera, CameraSelection::Pilot { .. }));
        assert_eq!(options.throttle, 0.0);
    }

    #[test]
    fn pilot_camera_is_default() {
        let options = RenderOptions::parse(std::iter::empty()).unwrap();
        assert!(matches!(options.camera, CameraSelection::Pilot { .. }));
    }

    #[test]
    fn chase_camera_mode_remains_available() {
        let options =
            RenderOptions::parse(["--camera".to_owned(), "chase".to_owned()].into_iter()).unwrap();
        assert!(matches!(options.camera, CameraSelection::Chase { .. }));
    }

    #[test]
    fn camera_settings_parse_without_affecting_physics_options() {
        let options = RenderOptions::parse(
            [
                "--camera",
                "chase",
                "--camera-fov",
                "90",
                "--chase-distance-m",
                "8",
                "--chase-height-m",
                "3",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(options.camera, CameraSelection::Chase { .. }));
    }

    #[test]
    fn ground_start_with_flying_field_scenery_and_pilot_camera() {
        let options = RenderOptions::parse(
            [
                "--start-on-ground",
                "--scenery",
                "flying-field",
                "--camera",
                "pilot",
                "--throttle",
                "0.5",
            ]
            .map(str::to_owned)
            .into_iter(),
        )
        .unwrap();
        assert!(options.start_on_ground);
        assert_eq!(options.scenery, SceneryPreset::FlyingField);
        assert!(matches!(options.camera, CameraSelection::Pilot { .. }));
        assert!((options.throttle - 0.5).abs() < 1e-9);
    }

    // ── OA1 calibrated startup (hardware-independent) ──────────────────────

    #[test]
    fn flight_reset_reconstructs_all_mutable_flight_state() {
        let mut application = RenderApplication::new(play_options_for_test()).unwrap();
        let initial_rigid_state = application.initial_rigid_state;
        let initial_ground = application.simulation.last_ground_evaluation().clone();
        assert!(initial_ground.weight_on_wheels());
        let expected_simulation = AircraftSimulation::new(
            application.simulation.model().clone(),
            *application.simulation.config(),
            initial_rigid_state,
        )
        .unwrap();
        let expected_snapshots = AircraftRenderSnapshotBuffer::new(
            AircraftRenderSnapshot::initial(&initial_rigid_state),
        );
        application.input_mode = Some(ViewerInputMode::Legacy {
            state: InputState::new(
                InputMapping::default(),
                KeyboardInputState::new(0.0).unwrap(),
            ),
            status: ControllerStatusTracker::new(Some(7)),
        });
        if let Some(ViewerInputMode::Legacy { state, .. }) = application.input_mode.as_mut() {
            state.set_key(KeyboardKey::ThrottleIncrease, true);
            assert!(state.sample(PHYSICS_DT.as_secs_f64()).unwrap().throttle() > 0.0);
        }
        application.simulation.set_brake_command(0.8);
        let snapshot = application
            .simulation
            .step(&PilotInput::new(0.8, -0.6, 0.4, 0.7));
        application
            .render_snapshots
            .push(AircraftRenderSnapshot::post_step(
                &snapshot,
                application.simulation.model(),
            ));
        application.fixed_step.advance(Duration::from_millis(1));
        application.last_frame_time = Some(Instant::now() - Duration::from_secs(1));
        assert_eq!(application.simulation.step_index(), 1);
        assert_ne!(
            application.simulation.state().controls(),
            expected_simulation.state().controls()
        );
        assert_ne!(application.render_snapshots, expected_snapshots);
        assert_ne!(application.fixed_step.remainder(), Duration::ZERO);

        let reset_at = Instant::now();
        assert_eq!(
            application.reset_flight_session(reset_at).unwrap(),
            FlightResetOutcome::Reset
        );
        assert_eq!(application.simulation.step_index(), 0);
        assert_eq!(application.simulation.sim_time_s(), 0.0);
        assert_eq!(
            application.simulation.state().rigid_body(),
            &initial_rigid_state
        );
        assert_eq!(
            application.simulation.state().controls(),
            expected_simulation.state().controls()
        );
        assert_eq!(application.simulation.brake_command(), 0.0);
        assert_eq!(
            application.simulation.last_ground_evaluation(),
            &initial_ground
        );
        assert_eq!(application.render_snapshots, expected_snapshots);
        assert_eq!(application.fixed_step.remainder(), Duration::ZERO);
        assert_eq!(application.fixed_step.physics_dt(), PHYSICS_DT);
        assert_eq!(application.last_frame_time, Some(reset_at));
        match application.input_mode.as_mut().unwrap() {
            ViewerInputMode::Legacy { state, status } => {
                assert_eq!(
                    state.sample(PHYSICS_DT.as_secs_f64()).unwrap(),
                    PilotInput::neutral()
                );
                assert!(status.observe(Some(7)).is_none());
            }
            ViewerInputMode::Calibrated(_) => panic!("expected legacy input"),
        }
    }

    #[test]
    fn flight_reset_is_refused_without_mutation_while_recording() {
        let mut options = play_options_for_test();
        options.replay_output_path = Some(PathBuf::from("unused-reset-policy-test.json"));
        let mut application = RenderApplication::new(options).unwrap();
        let _ = application.simulation.step(&PilotInput::neutral());
        let timing = Instant::now() - Duration::from_secs(1);
        application.last_frame_time = Some(timing);

        assert_eq!(
            application.reset_flight_session(Instant::now()).unwrap(),
            FlightResetOutcome::RefusedWhileRecording
        );
        assert_eq!(application.simulation.step_index(), 1);
        assert_eq!(application.last_frame_time, Some(timing));
        assert!(application.replay_recorder.is_some());
    }

    fn requested_identity() -> DeviceIdentity {
        DeviceIdentity::new(
            "TX16S",
            Some("tx16s-0123456789abcdef".to_owned()),
            Some(0x3511),
            Some(0x0123),
        )
    }

    fn other_identity() -> DeviceIdentity {
        DeviceIdentity::new(
            "Other Radio",
            Some("other-0123456789abcdef".to_owned()),
            Some(0x0001),
            Some(0x0001),
        )
    }

    fn calibrated_profile() -> ControllerProfile {
        ControllerProfile::new(
            requested_identity(),
            ProfileAxes::new(
                CenteredAxisProfile::new(
                    HardwareAxis::LeftStickX,
                    CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
                ),
                CenteredAxisProfile::new(
                    HardwareAxis::LeftStickY,
                    CenteredCalibration::new(Control::Pitch, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
                ),
                CenteredAxisProfile::new(
                    HardwareAxis::RightStickX,
                    CenteredCalibration::new(Control::Yaw, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
                ),
                ThrottleAxisProfile::new(
                    HardwareAxis::RightStickY,
                    ThrottleCalibration::new(-1.0, 1.0, false).unwrap(),
                ),
            ),
        )
        .unwrap()
    }

    fn raw_state(roll: f64, throttle: f64) -> RawControllerState {
        let mut state = RawControllerState::new();
        state.insert(HardwareAxis::LeftStickX, roll).unwrap();
        state.insert(HardwareAxis::LeftStickY, 0.0).unwrap();
        state.insert(HardwareAxis::RightStickX, 0.0).unwrap();
        state.insert(HardwareAxis::RightStickY, throttle).unwrap();
        state
    }

    #[test]
    fn flight_reset_preserves_calibrated_controller_owner_and_live_input() {
        let mut controller = CalibratedControllerState::new(calibrated_profile());
        assert_eq!(
            calibrate_startup_connect(
                &mut controller,
                &[requested_identity()],
                Some(raw_state(0.5, 0.5)),
            ),
            Ok(Some(()))
        );
        let expected_input = controller.input();
        let mut application = RenderApplication::new(play_options_for_test()).unwrap();
        application.input_mode = Some(ViewerInputMode::Calibrated(Box::new(controller)));
        let owner_before = match application.input_mode.as_ref().unwrap() {
            ViewerInputMode::Calibrated(state) => std::ptr::from_ref(state.as_ref()),
            ViewerInputMode::Legacy { .. } => unreachable!(),
        };

        assert_eq!(
            application.reset_flight_session(Instant::now()).unwrap(),
            FlightResetOutcome::Reset
        );
        match application.input_mode.as_ref().unwrap() {
            ViewerInputMode::Calibrated(state) => {
                assert_eq!(std::ptr::from_ref(state.as_ref()), owner_before);
                assert!(state.is_connected());
                assert_eq!(state.requested_device(), &requested_identity());
                assert_eq!(state.input(), expected_input);
            }
            ViewerInputMode::Legacy { .. } => panic!("calibrated mode must not fall back"),
        }
    }

    #[test]
    fn calibrated_startup_with_zero_devices_is_not_fatal_and_stays_neutral() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        // A transient zero-device WGI snapshot must yield "waiting", never an error.
        assert_eq!(calibrate_startup_connect(&mut state, &[], None), Ok(None));
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());
    }

    #[test]
    fn calibrated_input_stays_neutral_while_requested_controller_is_absent() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        assert_eq!(calibrate_startup_connect(&mut state, &[], None), Ok(None));
        assert_eq!(state.input(), PilotInput::neutral());
        // Still absent even when other controllers are enumerable with raw data.
        assert_eq!(
            calibrate_startup_connect(&mut state, &[other_identity()], Some(raw_state(0.5, 0.5))),
            Ok(None)
        );
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());
    }

    #[test]
    fn wrong_device_cannot_take_ownership_of_calibrated_input() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        assert_eq!(
            calibrate_startup_connect(&mut state, &[other_identity()], Some(raw_state(0.9, 0.9))),
            Ok(None)
        );
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());
    }

    #[test]
    fn ambiguous_device_state_remains_fail_closed() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        let duplicated = [requested_identity(), requested_identity()];
        assert_eq!(
            calibrate_startup_connect(&mut state, &duplicated, Some(raw_state(0.9, 0.9))),
            Ok(None)
        );
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());
    }

    #[test]
    fn later_appearance_of_requested_controller_connects_and_inputs_calibrated() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        // Startup: requested controller not yet enumerable (async WGI).
        assert_eq!(calibrate_startup_connect(&mut state, &[], None), Ok(None));
        assert_eq!(state.input(), PilotInput::neutral());
        // The device appears shortly afterwards: the same decision path connects it.
        assert_eq!(
            calibrate_startup_connect(
                &mut state,
                &[requested_identity()],
                Some(raw_state(0.5, 0.5))
            ),
            Ok(Some(()))
        );
        assert!(state.is_connected());
        assert_eq!(state.input().roll(), 0.5);
        assert_eq!(state.input().throttle(), 0.75);
    }

    #[test]
    fn immediately_present_requested_controller_connects_at_startup() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        assert_eq!(
            calibrate_startup_connect(
                &mut state,
                &[requested_identity()],
                Some(raw_state(-0.5, 0.25))
            ),
            Ok(Some(()))
        );
        assert!(state.is_connected());
        assert_eq!(state.input().roll(), -0.5);
        assert_eq!(state.input().throttle(), 0.625);
    }

    #[test]
    fn incomplete_first_sample_waits_and_later_complete_sample_connects() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        // The requested device is already enumerable, but its first WGI raw
        // sample is missing LeftStickX (roll): startup stays neutral and
        // keeps waiting instead of failing.
        let mut incomplete = RawControllerState::new();
        incomplete.insert(HardwareAxis::LeftStickY, 0.0).unwrap();
        incomplete.insert(HardwareAxis::RightStickX, 0.0).unwrap();
        incomplete.insert(HardwareAxis::RightStickY, 0.5).unwrap();
        assert_eq!(
            calibrate_startup_connect(&mut state, &[requested_identity()], Some(incomplete)),
            Ok(None)
        );
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());

        // A later complete sample drives the same decision path to connect.
        assert_eq!(
            calibrate_startup_connect(
                &mut state,
                &[requested_identity()],
                Some(raw_state(0.5, 0.5))
            ),
            Ok(Some(()))
        );
        assert!(state.is_connected());
        assert_eq!(state.input().roll(), 0.5);
        assert_eq!(state.input().throttle(), 0.75);
    }

    #[test]
    fn disconnect_neutralizes_and_same_device_resumes_via_reconnect_path() {
        let mut state = CalibratedControllerState::new(calibrated_profile());
        calibrate_startup_connect(
            &mut state,
            &[requested_identity()],
            Some(raw_state(0.5, 0.5)),
        )
        .unwrap();
        assert!(state.is_connected());
        assert!(state.neutralize().is_some());
        assert!(!state.is_connected());
        assert_eq!(state.input(), PilotInput::neutral());
        // Reconnect of the same requested identity resumes calibrated input.
        assert_eq!(
            calibrate_startup_connect(
                &mut state,
                &[requested_identity()],
                Some(raw_state(0.25, 0.75))
            ),
            Ok(Some(()))
        );
        assert!(state.is_connected());
        assert_eq!(state.input().roll(), 0.25);
        assert_eq!(state.input().throttle(), 0.875);
    }

    #[test]
    fn calibrated_mode_has_no_keyboard_fallback() {
        let mut mode = ViewerInputMode::Calibrated(Box::new(CalibratedControllerState::new(
            calibrated_profile(),
        )));
        mode.set_key(KeyboardKey::RollRight, true);
        mode.set_key(KeyboardKey::ThrottleIncrease, true);
        let input = mode.sample(PHYSICS_DT.as_secs_f64()).unwrap();
        assert_eq!(input, PilotInput::neutral());
    }

    #[test]
    fn waiting_startup_status_diagnostics_are_explicit() {
        let state = CalibratedControllerState::new(calibrated_profile());
        let status =
            format_calibrated_startup_status(Path::new("controllers/tx16s.json"), &state, None);
        assert!(
            status.contains("Controller profile:\ncontrollers\\tx16s.json")
                || status.contains("Controller profile:\ncontrollers/tx16s.json")
        );
        assert!(status.contains("Input mode:\ncalibrated controller profile"));
        assert!(status.contains("Requested controller:\nname=\"TX16S\""));
        assert!(status.contains("Controller status:\nwaiting for requested controller"));
        assert!(status.contains("Pilot input:\nneutral"));
    }

    #[test]
    fn connected_startup_status_diagnostics_include_matched_device() {
        let state = CalibratedControllerState::new(calibrated_profile());
        let status = format_calibrated_startup_status(
            Path::new("controllers/tx16s.json"),
            &state,
            Some((7, requested_identity())),
        );
        assert!(status.contains("Matched controller:\nsession_id=7 name=\"TX16S\""));
        assert!(status.contains("Controller status:\nconnected"));
        assert!(status.contains("Pilot input:\ncalibrated"));
    }

    #[test]
    fn successful_flight_reset_notifies_renderer_temporal_lifecycle() {
        let source = include_str!("render_app.rs");
        let reset_path = source
            .split_once("fn reset_flight_session(")
            .unwrap()
            .1
            .split_once("fn redraw(")
            .unwrap()
            .0;
        let state_install = reset_path
            .find("self.simulation = reset_simulation;")
            .unwrap();
        let invalidation = reset_path
            .find("renderer.invalidate_temporal_history();")
            .unwrap();
        let success = reset_path.find("Ok(FlightResetOutcome::Reset)").unwrap();
        assert!(state_install < invalidation && invalidation < success);
    }
}
