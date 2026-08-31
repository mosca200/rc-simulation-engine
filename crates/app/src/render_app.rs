use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError};
use model::{ModelLoadError, load_aircraft_model};
use platform::{
    GilrsInputBackend, InputError, InputMapping, InputSource, InputState, KeyboardInputState,
    KeyboardKey,
};
use renderer::{
    FixedStepAccumulator, FixedStepAccumulatorError, RenderDataError, RenderFrame, RendererError,
    SurfaceError, WgpuRenderer, world_ned_pose_to_render,
};
use replay::{AircraftReplayError, AircraftReplayRecorder};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState,
    SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{
    io,
    path::PathBuf,
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
const PHYSICS_DT: Duration = Duration::from_millis(2);
const MAXIMUM_FRAME_DELTA: Duration = Duration::from_millis(250);
const MAXIMUM_PHYSICS_STEPS_PER_FRAME: u32 = 16;

#[derive(Debug, Clone)]
pub struct RenderOptions {
    model_path: PathBuf,
    throttle: f64,
    replay_output_path: Option<PathBuf>,
}

impl RenderOptions {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, RenderAppError> {
        let mut options = Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            throttle: DEFAULT_THROTTLE,
            replay_output_path: None,
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
                "--record-replay" => {
                    options.replay_output_path =
                        Some(PathBuf::from(arguments.next().ok_or(
                            RenderAppError::MissingArgumentValue("--record-replay"),
                        )?));
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
    #[error("unknown render argument: {0}")]
    UnknownArgument(String),
    #[error("failed to load render model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
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

struct RenderApplication {
    simulation: AircraftSimulation,
    input_state: InputState,
    input_backend: GilrsInputBackend,
    replay_recorder: Option<AircraftReplayRecorder>,
    replay_output_path: Option<PathBuf>,
    render_origin_world_ned_m: [f64; 3],
    fixed_step: FixedStepAccumulator,
    last_frame_time: Option<Instant>,
    window: Option<Arc<Window>>,
    renderer: Option<WgpuRenderer>,
    runtime_error: Option<RenderRuntimeError>,
}

impl RenderApplication {
    fn new(options: RenderOptions) -> Result<Self, RenderAppError> {
        let model_path = options.model_path;
        let model =
            load_aircraft_model(&model_path).map_err(|source| RenderAppError::ModelLoad {
                path: model_path,
                source,
            })?;
        let initial_state = RigidBodyState {
            position_world_m: Vec3::new(0.0, 0.0, -100.0),
            linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let render_origin_world_ned_m = vector_to_array(initial_state.position_world_m);
        let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
        let config = AircraftSimulationConfig::from_physics_hz(DEFAULT_PHYSICS_HZ, environment)?;
        let simulation = AircraftSimulation::new(model, config, initial_state)?;
        let input_state = InputState::new(
            InputMapping::default(),
            KeyboardInputState::new(options.throttle)?,
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
        Ok(Self {
            simulation,
            input_state,
            input_backend,
            replay_recorder,
            replay_output_path: options.replay_output_path,
            render_origin_world_ned_m,
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
            if let Err(error) =
                advance_aircraft(&mut self.simulation, &mut self.replay_recorder, input)
            {
                self.fail(event_loop, error.into());
                return;
            }
        }
        if step_plan.dropped_time_s() > 0.0 {
            warn!(
                dropped_time_s = step_plan.dropped_time_s(),
                "render loop discarded wall-clock backlog while preserving fixed physics dt"
            );
        }

        let pose = match rigid_state_to_render_pose(
            self.simulation.state().rigid_body(),
            self.render_origin_world_ned_m,
        ) {
            Ok(pose) => pose,
            Err(error) => {
                self.fail(event_loop, error.into());
                return;
            }
        };
        let frame = RenderFrame::new(pose);
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
            .with_title("RC Simulation Engine — S7 Minimal Renderer")
            .with_inner_size(LogicalSize::new(1_280.0, 720.0));
        let window = match event_loop.create_window(attributes) {
            Ok(window) => Arc::new(window),
            Err(error) => {
                self.fail(event_loop, RenderRuntimeError::WindowCreation(error));
                return;
            }
        };
        let renderer = match pollster::block_on(WgpuRenderer::new(Arc::clone(&window))) {
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
) -> Result<(), AircraftReplayError> {
    let step_index = simulation.step_index();
    if let Some(recorder) = recorder {
        let _ = recorder.record(simulation, step_index, input)?;
    } else {
        let _ = simulation.step(&input);
    }
    Ok(())
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

fn rigid_state_to_render_pose(
    state: &RigidBodyState,
    render_origin_world_ned_m: [f64; 3],
) -> Result<renderer::RenderPose, RenderDataError> {
    let quaternion = state.orientation_world_from_body.quaternion();
    world_ned_pose_to_render(
        vector_to_array(state.position_world_m),
        [quaternion.w, quaternion.i, quaternion.j, quaternion.k],
        render_origin_world_ned_m,
    )
}

fn vector_to_array(vector: Vec3) -> [f64; 3] {
    [vector.x, vector.y, vector.z]
}

#[cfg(test)]
mod tests {
    use super::*;
    use replay::{AircraftReplayPlayer, AircraftReplayRecording};

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
    fn rigid_body_adapter_preserves_raw_pose_semantics() {
        let state = RigidBodyState {
            position_world_m: Vec3::new(101.0, 202.0, 303.0),
            linear_velocity_world_mps: Vec3::zeros(),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let pose = rigid_state_to_render_pose(&state, [100.0, 200.0, 300.0]).unwrap();
        assert_eq!(pose.translation_render_m(), [2.0, -3.0, -1.0]);
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
        let mut simulation = AircraftSimulation::new(model, config, initial_state).unwrap();
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
        let model = load_aircraft_model(model_path).unwrap();
        let mut replayed = decoded.reconstruct_simulation(model).unwrap();
        let player = AircraftReplayPlayer::new(&decoded, &replayed).unwrap();
        assert_eq!(player.verify_all(&mut replayed).unwrap(), 3);
    }
}
