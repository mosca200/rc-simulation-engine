use crate::render_snapshot::{
    AircraftRenderSnapshot, AircraftRenderSnapshotBuffer, interpolation_alpha,
};
use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError, AircraftSnapshot,
};
use model::{
    AircraftModel, AircraftModelFingerprint, ModelLoadError, PresentationMetadata,
    PresentationSurface, load_aircraft_model,
};
use platform::{
    GilrsInputBackend, InputError, InputMapping, InputSource, InputState, KeyboardInputState,
    KeyboardKey,
};
use renderer::{
    AircraftMesh, FixedStepAccumulator, FixedStepAccumulatorError, GlbArticulationError,
    GlbArticulationPlan, GlbAsset, GlbLoadError, PresentationAsset, RenderDataError,
    RenderTerrainMode, RendererError, SurfaceError, SurfaceHinge, SurfaceId, WgpuRenderer,
    aircraft_mesh, load_glb_asset, scenery::SceneryPreset,
};
use replay::{AircraftReplayError, AircraftReplayRecorder};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_GRAVITY_MPS2, DEFAULT_PHYSICS_HZ,
    FlatGroundPlane, GroundCommand, GroundEvaluation, GroundSurface, PilotInput, RigidBodyState,
    SimulationConfigError, evaluate_ground_wrench,
};
use sim_math::{Orientation, Vec3};
use std::{
    io,
    path::{Path, PathBuf},
    sync::Arc,
    time::{Duration, Instant},
};
use thiserror::Error;
use tracing::warn;
use winit::{
    application::ApplicationHandler,
    dpi::LogicalSize,
    event::{ElementState, WindowEvent},
    event_loop::{ActiveEventLoop, ControlFlow, EventLoop},
    keyboard::{Key, KeyCode, NamedKey, PhysicalKey},
    window::{Window, WindowId},
};

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_THROTTLE: f64 = 0.55;
const DEFAULT_ALTITUDE_M: f64 = 30.0;
const DEFAULT_AIRSPEED_MPS: f64 = 18.0;
const MAXIMUM_ALTITUDE_M: f64 = 10_000.0;
const MAXIMUM_AIRSPEED_MPS: f64 = 200.0;
const PHYSICS_DT: Duration = Duration::from_millis(2);
const MAXIMUM_FRAME_DELTA: Duration = Duration::from_millis(250);
const MAXIMUM_PHYSICS_STEPS_PER_FRAME: u32 = 16;

#[derive(Debug, Clone)]
pub struct RenderOptions {
    model_path: PathBuf,
    throttle: f64,
    altitude_m: f64,
    airspeed_mps: f64,
    replay_output_path: Option<PathBuf>,
    start_on_ground: bool,
    scenery: SceneryPreset,
}

impl RenderOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, RenderAppError> {
        let mut options = Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            throttle: DEFAULT_THROTTLE,
            altitude_m: DEFAULT_ALTITUDE_M,
            airspeed_mps: DEFAULT_AIRSPEED_MPS,
            replay_output_path: None,
            start_on_ground: false,
            scenery: SceneryPreset::None,
        };
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
                "--start-on-ground" => {
                    options.start_on_ground = true;
                }
                "--scenery" => {
                    let value = arguments
                        .next()
                        .ok_or(RenderAppError::MissingArgumentValue("--scenery"))?;
                    options.scenery = match value.as_str() {
                        "none" => SceneryPreset::None,
                        "flying-field" => SceneryPreset::FlyingField,
                        _ => return Err(RenderAppError::InvalidScenery(value)),
                    };
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(RenderAppError::UnknownArgument(argument)),
            }
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
    #[error("invalid scenery preset `{0}`; expected `none` or `flying-field`")]
    InvalidScenery(String),
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
    #[error("failed to initialize wgpu: {0}")]
    RendererInitialization(#[source] RendererError),
    #[error("failed to convert the committed physics pose for rendering: {0}")]
    RenderPose(#[from] RenderDataError),
    #[error("failed to sample normalized pilot input: {0}")]
    Input(#[from] InputError),
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
}

pub fn run_render(options: RenderOptions) -> Result<(), RenderAppError> {
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

enum PresentationModel {
    Glb {
        asset: GlbAsset,
        articulation: GlbArticulationPlan,
    },
    Procedural(AircraftMesh),
}

struct RenderApplication {
    simulation: AircraftSimulation,
    presentation: PresentationModel,
    scenery_preset: SceneryPreset,
    input_state: InputState,
    input_backend: GilrsInputBackend,
    replay_recorder: Option<AircraftReplayRecorder>,
    replay_output_path: Option<PathBuf>,
    render_origin_world_ned_m: [f64; 3],
    ground_below_render_origin_m: f32,
    terrain_mode: RenderTerrainMode,
    render_snapshots: AircraftRenderSnapshotBuffer,
    fixed_step: FixedStepAccumulator,
    last_frame_time: Option<Instant>,
    window: Option<Arc<Window>>,
    renderer: Option<WgpuRenderer>,
    runtime_error: Option<RenderRuntimeError>,
}

impl RenderApplication {
    fn new(options: RenderOptions) -> Result<Self, RenderAppError> {
        let altitude_m = options.altitude_m;
        let airspeed_mps = options.airspeed_mps;
        let initial_throttle = options.throttle;
        let model_path = options.model_path;
        let model =
            load_aircraft_model(&model_path).map_err(|source| RenderAppError::ModelLoad {
                path: model_path.clone(),
                source,
            })?;
        let presentation = resolve_presentation_model(&model_path, model.presentation())?;
        let model_id = model.model_id().to_owned();
        let model_fingerprint = model.physics_fingerprint();
        let (initial_state, ground_below_render_origin_m, terrain_mode, initial_ground) =
            if options.start_on_ground {
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
        let render_snapshots =
            AircraftRenderSnapshotBuffer::new(AircraftRenderSnapshot::initial(&initial_state));
        let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
        let config = AircraftSimulationConfig::from_physics_hz(DEFAULT_PHYSICS_HZ, environment)?;
        let simulation = AircraftSimulation::new(model, config, initial_state)?;
        let input_state = InputState::new(
            InputMapping::default(),
            KeyboardInputState::new(initial_throttle)?,
        );
        let input_backend = GilrsInputBackend::new()?;
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
            options.start_on_ground,
            initial_ground.weight_on_wheels(),
            terrain_mode,
        );
        Ok(Self {
            simulation,
            presentation,
            scenery_preset: options.scenery,
            input_state,
            input_backend,
            replay_recorder,
            replay_output_path: options.replay_output_path,
            render_origin_world_ned_m,
            ground_below_render_origin_m,
            terrain_mode,
            render_snapshots,
            fixed_step,
            last_frame_time: None,
            window: None,
            renderer: None,
            runtime_error: None,
        })
    }

    fn fail(&mut self, event_loop: &ActiveEventLoop, error: RenderRuntimeError) {
        self.runtime_error = Some(error);
        event_loop.exit();
    }

    fn finish_and_exit(&mut self, event_loop: &ActiveEventLoop) {
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

    fn redraw(&mut self, event_loop: &ActiveEventLoop) {
        let now = Instant::now();
        let frame_delta = self
            .last_frame_time
            .replace(now)
            .map_or(Duration::ZERO, |previous| {
                now.saturating_duration_since(previous)
            });
        let step_plan = self.fixed_step.advance(frame_delta);
        self.input_state
            .set_controller_axes(self.input_backend.poll_axes());
        for _ in 0..step_plan.physics_steps() {
            let input = match self.input_state.sample(PHYSICS_DT.as_secs_f64()) {
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
        let render_result = self
            .renderer
            .as_mut()
            .map(|renderer| renderer.render(&frame));
        match render_result {
            Some(Ok(())) | None | Some(Err(SurfaceError::Occluded)) => {}
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
        let attributes = Window::default_attributes()
            .with_title("RC Simulation Engine — Manual Flight Viewer")
            .with_inner_size(LogicalSize::new(1_280.0, 720.0));
        let window = match event_loop.create_window(attributes) {
            Ok(window) => Arc::new(window),
            Err(error) => {
                self.fail(event_loop, RenderRuntimeError::WindowCreation(error));
                return;
            }
        };
        let presentation_asset = match &self.presentation {
            PresentationModel::Glb {
                asset,
                articulation,
            } => PresentationAsset::ArticulatedGlb {
                asset,
                articulation,
            },
            PresentationModel::Procedural(mesh) => PresentationAsset::Procedural(mesh),
        };
        let renderer = match pollster::block_on(WgpuRenderer::new_with_presentation(
            Arc::clone(&window),
            presentation_asset,
            self.ground_below_render_origin_m,
            self.terrain_mode,
            Some(self.scenery_preset),
        )) {
            Ok(renderer) => renderer,
            Err(error) => {
                self.fail(
                    event_loop,
                    RenderRuntimeError::RendererInitialization(error),
                );
                return;
            }
        };
        self.renderer = Some(renderer);
        self.window = Some(window);
        self.last_frame_time = Some(Instant::now());
        if let Some(window) = self.window.as_ref() {
            window.request_redraw();
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
            WindowEvent::KeyboardInput { event, .. } => {
                if let Some(key) = keyboard_key(event.physical_key) {
                    self.input_state
                        .set_key(key, event.state == ElementState::Pressed);
                }
            }
            WindowEvent::Resized(size) => {
                if let Some(renderer) = self.renderer.as_mut() {
                    renderer.resize(size.width, size.height);
                }
            }
            WindowEvent::RedrawRequested => self.redraw(event_loop),
            _ => {}
        }
    }

    fn about_to_wait(&mut self, _event_loop: &ActiveEventLoop) {
        if let Some(window) = self.window.as_ref() {
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

#[cfg(test)]
mod tests {
    use super::*;
    use replay::{AircraftReplayPlayer, AircraftReplayRecording};

    fn repository_model_path(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    fn acro_model_path() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models/acro_electric_01/model.json")
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
    fn altitude_and_airspeed_options_parse_with_manual_flight_defaults_and_overrides() {
        let defaults = RenderOptions::parse(std::iter::empty()).unwrap();
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
    fn ground_start_without_landing_gear_fails_explicitly() {
        let model_path = repository_model_path("models/acro_electric_01/model.json");
        let model = load_aircraft_model(&model_path).unwrap();
        assert!(matches!(
            supported_ground_start(&model),
            Err(RenderAppError::GroundStartWithoutLandingGear { model_id })
                if model_id == "acro-electric-01"
        ));
        let options = RenderOptions::parse(
            [
                "--model".to_owned(),
                model_path.display().to_string(),
                "--start-on-ground".to_owned(),
            ]
            .into_iter(),
        )
        .unwrap();
        assert!(matches!(
            RenderApplication::new(options),
            Err(RenderAppError::GroundStartWithoutLandingGear { model_id })
                if model_id == "acro-electric-01"
        ));
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
}
