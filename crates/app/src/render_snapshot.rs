use aircraft::{AircraftSimulation, AircraftSimulationConfig, AircraftSnapshot};
use model::{AircraftModel, ControlActuator, PresentationSurface, load_aircraft_model};
use platform::{InputSource, InputState, KeyboardKey};
use renderer::{
    ControlSurfacePresentation, FixedStepAccumulator, RenderDataError, RenderFrame, RenderPose,
    world_ned_pose_to_render,
};
use replay::AircraftSnapshotHash;
use sim_core::{AeroEnvironment, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::{path::Path, time::Duration};

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AircraftRenderSnapshot {
    step_index: u64,
    sim_time_s: f64,
    position_world_ned_m: [f64; 3],
    orientation_world_from_body_wxyz: [f64; 4],
    surfaces_rad: [f32; 4],
}

impl AircraftRenderSnapshot {
    pub(crate) fn initial(state: &RigidBodyState) -> Self {
        Self::from_state(0, 0.0, state)
    }

    pub(crate) fn post_step(snapshot: &AircraftSnapshot, model: &AircraftModel) -> Self {
        let mut base = Self::from_state(
            snapshot.step_index(),
            snapshot.sim_time_s(),
            snapshot.rigid_body_state(),
        );
        base.surfaces_rad = surface_deflections_from_simulation(model, snapshot);
        base
    }

    fn from_state(step_index: u64, sim_time_s: f64, state: &RigidBodyState) -> Self {
        let position = state.position_world_m;
        let quaternion = state.orientation_world_from_body.quaternion();
        Self {
            step_index,
            sim_time_s,
            position_world_ned_m: [position.x, position.y, position.z],
            orientation_world_from_body_wxyz: [
                quaternion.w,
                quaternion.i,
                quaternion.j,
                quaternion.k,
            ],
            surfaces_rad: [0.0; 4],
        }
    }

    const fn from_components(
        step_index: u64,
        sim_time_s: f64,
        position_world_ned_m: [f64; 3],
        orientation_world_from_body_wxyz: [f64; 4],
    ) -> Self {
        Self {
            step_index,
            sim_time_s,
            position_world_ned_m,
            orientation_world_from_body_wxyz,
            surfaces_rad: [0.0; 4],
        }
    }

    pub(crate) fn surfaces(&self) -> ControlSurfacePresentation {
        ControlSurfacePresentation::new(
            self.surfaces_rad[0],
            self.surfaces_rad[1],
            self.surfaces_rad[2],
            self.surfaces_rad[3],
            0.0,
        )
        .expect("render snapshots only store finite simulated deflections")
    }

    pub(crate) fn render_frame(&self, pose: RenderPose) -> RenderFrame {
        RenderFrame::with_surfaces(pose, self.surfaces())
    }
}

/// Derives per-surface simulated deflections from committed servo state.
pub(crate) fn surface_deflections_from_simulation(
    model: &AircraftModel,
    snapshot: &AircraftSnapshot,
) -> [f32; 4] {
    let positions = snapshot.control_surface_positions();
    let actuators = model.controls().actuators();
    let mut out = [0.0f64; 4];
    let Some(presentation) = model.presentation() else {
        return out.map(|value| value as f32);
    };
    for visual_surface in presentation.articulated_surfaces() {
        let binding = model
            .control_surface_bindings()
            .iter()
            .find(|binding| binding.id() == visual_surface.control_surface_binding_id())
            .expect("model loading resolves every presentation binding");
        let (servo_angle_rad, neutral_rad) = match binding.actuator() {
            ControlActuator::Aileron => (
                positions.aileron_angle_rad(),
                actuators.aileron().neutral_angle_rad(),
            ),
            ControlActuator::Elevator => (
                positions.elevator_angle_rad(),
                actuators.elevator().neutral_angle_rad(),
            ),
            ControlActuator::Rudder => (
                positions.rudder_angle_rad(),
                actuators.rudder().neutral_angle_rad(),
            ),
        };
        let deflection_rad = binding.deflection_gain() * (servo_angle_rad - neutral_rad);
        if !deflection_rad.is_finite() {
            continue;
        }
        let slot = match visual_surface.surface() {
            PresentationSurface::LeftAileron => 0,
            PresentationSurface::RightAileron => 1,
            PresentationSurface::Elevator => 2,
            PresentationSurface::Rudder => 3,
        };
        out[slot] = deflection_rad;
    }
    out.map(|value| value as f32)
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AircraftRenderSnapshotBuffer {
    previous: AircraftRenderSnapshot,
    current: AircraftRenderSnapshot,
}

impl AircraftRenderSnapshotBuffer {
    pub(crate) const fn new(initial: AircraftRenderSnapshot) -> Self {
        Self {
            previous: initial,
            current: initial,
        }
    }

    pub(crate) fn push(&mut self, snapshot: AircraftRenderSnapshot) {
        self.previous = self.current;
        self.current = snapshot;
    }

    pub(crate) fn interpolated_pose(
        &self,
        alpha: f64,
        render_origin_world_ned_m: [f64; 3],
    ) -> Result<RenderPose, RenderDataError> {
        let snapshot = interpolate(self.previous, self.current, alpha);
        world_ned_pose_to_render(
            snapshot.position_world_ned_m,
            snapshot.orientation_world_from_body_wxyz,
            render_origin_world_ned_m,
        )
    }

    pub(crate) fn interpolated_snapshot(&self, alpha: f64) -> AircraftRenderSnapshot {
        interpolate(self.previous, self.current, alpha)
    }
}

pub(crate) fn interpolation_alpha(remainder: Duration, physics_dt: Duration) -> f64 {
    interpolation_alpha_seconds(remainder.as_secs_f64(), physics_dt.as_secs_f64())
}

fn interpolation_alpha_seconds(remainder_s: f64, physics_dt_s: f64) -> f64 {
    if !remainder_s.is_finite() || !physics_dt_s.is_finite() || physics_dt_s <= 0.0 {
        return 0.0;
    }
    (remainder_s / physics_dt_s).clamp(0.0, 1.0)
}

fn interpolate(
    previous: AircraftRenderSnapshot,
    current: AircraftRenderSnapshot,
    alpha: f64,
) -> AircraftRenderSnapshot {
    let alpha = if alpha.is_finite() {
        alpha.clamp(0.0, 1.0)
    } else {
        0.0
    };
    let surfaces_rad = if alpha < 1.0 {
        previous.surfaces_rad
    } else {
        current.surfaces_rad
    };
    AircraftRenderSnapshot {
        step_index: if alpha < 1.0 {
            previous.step_index
        } else {
            current.step_index
        },
        sim_time_s: lerp(previous.sim_time_s, current.sim_time_s, alpha),
        position_world_ned_m: [0, 1, 2].map(|axis| {
            lerp(
                previous.position_world_ned_m[axis],
                current.position_world_ned_m[axis],
                alpha,
            )
        }),
        orientation_world_from_body_wxyz: slerp_shortest(
            previous.orientation_world_from_body_wxyz,
            current.orientation_world_from_body_wxyz,
            alpha,
        ),
        surfaces_rad,
    }
}

fn lerp(start: f64, end: f64, alpha: f64) -> f64 {
    start + (end - start) * alpha
}

fn slerp_shortest(mut start: [f64; 4], mut end: [f64; 4], alpha: f64) -> [f64; 4] {
    normalize_quaternion(&mut start);
    normalize_quaternion(&mut end);
    let mut dot = dot_quaternion(start, end);
    if dot < 0.0 {
        end = end.map(|value| -value);
        dot = -dot;
    }
    dot = dot.clamp(-1.0, 1.0);
    if dot > 0.999_5 {
        let mut result = [0, 1, 2, 3].map(|index| lerp(start[index], end[index], alpha));
        normalize_quaternion(&mut result);
        return result;
    }
    let angle = dot.acos();
    let inverse_sine = angle.sin().recip();
    let start_weight = ((1.0 - alpha) * angle).sin() * inverse_sine;
    let end_weight = (alpha * angle).sin() * inverse_sine;
    let mut result =
        [0, 1, 2, 3].map(|index| start[index] * start_weight + end[index] * end_weight);
    normalize_quaternion(&mut result);
    result
}

fn normalize_quaternion(quaternion: &mut [f64; 4]) {
    let inverse_norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt()
        .recip();
    for value in quaternion {
        *value *= inverse_norm;
    }
}

fn dot_quaternion(left: [f64; 4], right: [f64; 4]) -> f64 {
    left.into_iter().zip(right).map(|(a, b)| a * b).sum()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct FrameRateIndependenceEvidence {
    pub(crate) pattern_count: usize,
    pub(crate) physics_steps: u64,
    pub(crate) render_snapshot_insertions: u64,
}

pub(crate) fn interpolation_acceptance_passes() -> bool {
    const IDENTITY: [f64; 4] = [1.0, 0.0, 0.0, 0.0];
    let origin = [1.0e12, -1.0e12, 5.0e11];
    let previous = AircraftRenderSnapshot::from_components(0, 0.0, origin, IDENTITY);
    let half_angle = std::f64::consts::FRAC_PI_4;
    let end = [-half_angle.cos(), 0.0, -half_angle.sin(), 0.0];
    let current = AircraftRenderSnapshot::from_components(
        1,
        0.002,
        [origin[0] + 0.5, origin[1] + 1.0, origin[2] - 1.5],
        end,
    );
    let midpoint = interpolate(previous, current, 0.5);
    let quaternion_norm = midpoint
        .orientation_world_from_body_wxyz
        .into_iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    let buffer = AircraftRenderSnapshotBuffer { previous, current };
    let Ok(pose) = buffer.interpolated_pose(0.5, origin) else {
        return false;
    };
    interpolation_alpha_seconds(-1.0, 0.002) == 0.0
        && interpolation_alpha_seconds(1.0, 0.002) == 1.0
        && midpoint.position_world_ned_m == [origin[0] + 0.25, origin[1] + 0.5, origin[2] - 0.75]
        && (quaternion_norm - 1.0).abs() < 1.0e-12
        && midpoint.orientation_world_from_body_wxyz[0] > 0.0
        && pose.translation_render_m() == [0.5, 0.75, -0.25]
}

pub(crate) fn verify_frame_rate_independence(
    model_path: &Path,
) -> Result<FrameRateIndependenceEvidence, String> {
    let model = load_aircraft_model(model_path).map_err(|error| error.to_string())?;
    let total = Duration::from_secs(1);
    let patterns = [
        frame_pattern(total, &[Duration::from_nanos(16_666_667)]),
        frame_pattern(total, &[Duration::from_nanos(6_944_444)]),
        frame_pattern(
            total,
            &[
                Duration::from_millis(5),
                Duration::from_millis(11),
                Duration::from_millis(7),
                Duration::from_millis(20),
            ],
        ),
    ];
    let baseline = run_pattern(&model, &patterns[0])?;
    if baseline.steps != 500 || baseline.render_insertions != baseline.steps {
        return Err("60 Hz-like baseline did not execute exactly 500 physics steps".to_owned());
    }
    for pattern in &patterns[1..] {
        let actual = run_pattern(&model, pattern)?;
        if actual.steps != baseline.steps
            || actual.render_insertions != actual.steps
            || actual.inputs != baseline.inputs
            || actual.hashes != baseline.hashes
            || actual.final_state != baseline.final_state
        {
            return Err("render-frame pattern changed deterministic physics output".to_owned());
        }
    }
    Ok(FrameRateIndependenceEvidence {
        pattern_count: patterns.len(),
        physics_steps: baseline.steps,
        render_snapshot_insertions: baseline.render_insertions,
    })
}

struct PatternResult {
    steps: u64,
    inputs: Vec<PilotInput>,
    hashes: Vec<AircraftSnapshotHash>,
    final_state: aircraft::AircraftState,
    render_insertions: u64,
}

fn frame_pattern(total: Duration, pattern: &[Duration]) -> Vec<Duration> {
    let mut output = Vec::new();
    let mut elapsed = Duration::ZERO;
    let mut index = 0;
    while elapsed < total {
        let next = pattern[index % pattern.len()].min(total - elapsed);
        output.push(next);
        elapsed += next;
        index += 1;
    }
    output
}

fn run_pattern(model: &AircraftModel, pattern: &[Duration]) -> Result<PatternResult, String> {
    let config = AircraftSimulationConfig::from_physics_hz(
        DEFAULT_PHYSICS_HZ,
        AeroEnvironment::new(1.225, Vec3::zeros()).map_err(|error| error.to_string())?,
    )
    .map_err(|error| error.to_string())?;
    let initial = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let mut simulation = AircraftSimulation::new(model.clone(), config, initial)
        .map_err(|error| error.to_string())?;
    let mut accumulator =
        FixedStepAccumulator::new(Duration::from_millis(2), Duration::from_millis(250), 128)
            .map_err(|error| error.to_string())?;
    let mut input = InputState::default();
    input.set_key(KeyboardKey::PitchUp, true);
    input.set_key(KeyboardKey::ThrottleIncrease, true);
    let mut render_buffer =
        AircraftRenderSnapshotBuffer::new(AircraftRenderSnapshot::initial(&initial));
    let mut inputs = Vec::with_capacity(500);
    let mut hashes = Vec::with_capacity(500);
    let mut render_insertions = 0;
    for frame_delta in pattern {
        let plan = accumulator.advance(*frame_delta);
        if plan.dropped_time_s() != 0.0 {
            return Err("frame pattern triggered backlog drop".to_owned());
        }
        for _ in 0..plan.physics_steps() {
            let sampled = input.sample(0.002).map_err(|error| error.to_string())?;
            let snapshot = simulation.step(&sampled);
            inputs.push(sampled);
            hashes.push(AircraftSnapshotHash::from_snapshot(&snapshot));
            render_buffer.push(AircraftRenderSnapshot::post_step(&snapshot, model));
            render_insertions += 1;
        }
    }
    Ok(PatternResult {
        steps: simulation.step_index(),
        inputs,
        hashes,
        final_state: *simulation.state(),
        render_insertions,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_math::{Orientation, Vec3};
    use std::path::PathBuf;

    const IDENTITY: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    fn snapshot(position: [f64; 3], orientation: [f64; 4]) -> AircraftRenderSnapshot {
        AircraftRenderSnapshot::from_components(0, 0.0, position, orientation)
    }

    fn acro_model_with_explicit_presentation() -> (AircraftModel, AircraftModel) {
        let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let original = load_aircraft_model(&path).expect("acro model must load");
        let mut value: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
        let bindings = value["control_surface_bindings"].as_array_mut().unwrap();
        let opaque_ids = ["v0", "v1", "v2", "v3"];
        for (binding, id) in bindings.iter_mut().zip(opaque_ids) {
            binding["id"] = serde_json::json!(id);
        }
        value["presentation"]["articulated_surfaces"] = serde_json::json!([
            {
                "visual_primitive_index": 0,
                "surface": "left_aileron",
                "control_surface_binding_id": "v0",
                "hinge_origin_render_body_m": [0.0, 0.0, 0.0],
                "hinge_axis_render_body": [1.0, 0.0, 0.0],
                "visual_gain": 1.0
            },
            {
                "visual_primitive_index": 1,
                "surface": "right_aileron",
                "control_surface_binding_id": "v1",
                "hinge_origin_render_body_m": [0.0, 0.0, 0.0],
                "hinge_axis_render_body": [1.0, 0.0, 0.0],
                "visual_gain": 1.0
            },
            {
                "visual_primitive_index": 2,
                "surface": "elevator",
                "control_surface_binding_id": "v2",
                "hinge_origin_render_body_m": [0.0, 0.0, 0.0],
                "hinge_axis_render_body": [1.0, 0.0, 0.0],
                "visual_gain": 1.0
            },
            {
                "visual_primitive_index": 3,
                "surface": "rudder",
                "control_surface_binding_id": "v3",
                "hinge_origin_render_body_m": [0.0, 0.0, 0.0],
                "hinge_axis_render_body": [0.0, 1.0, 0.0],
                "visual_gain": 1.0
            }
        ]);
        let explicit = model::AircraftModelLoader::from_json_str(&value.to_string()).unwrap();
        (original, explicit)
    }

    fn stepped_snapshot(
        model: &AircraftModel,
        input: PilotInput,
        steps: usize,
    ) -> AircraftSnapshot {
        let config = AircraftSimulationConfig::from_physics_hz(
            DEFAULT_PHYSICS_HZ,
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap();
        let initial = RigidBodyState {
            position_world_m: Vec3::new(0.0, 0.0, -100.0),
            linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let mut simulation = AircraftSimulation::new(model.clone(), config, initial).unwrap();
        let mut snapshot = simulation.step(&PilotInput::neutral());
        for _ in 1..steps.max(1) {
            snapshot = simulation.step(&input);
        }
        snapshot
    }

    #[test]
    fn explicit_opaque_metadata_uses_simulated_servo_state_not_keyboard() {
        let (original, model) = acro_model_with_explicit_presentation();
        let before = original.physics_fingerprint();
        let snapshot = stepped_snapshot(&model, PilotInput::new(1.0, 0.0, 0.0, 0.5), 600);
        let deflections = surface_deflections_from_simulation(&model, &snapshot);
        assert!((deflections[0] + deflections[1]).abs() < 1.0e-4);
        assert!(deflections[0].abs() > 0.05);
        let pitch = stepped_snapshot(&model, PilotInput::new(0.0, 1.0, 0.0, 0.5), 600);
        assert!(surface_deflections_from_simulation(&model, &pitch)[2].abs() > 0.05);
        let yaw = stepped_snapshot(&model, PilotInput::new(0.0, 0.0, 1.0, 0.5), 600);
        assert!(surface_deflections_from_simulation(&model, &yaw)[3].abs() > 0.05);
        let neutral = stepped_snapshot(&model, PilotInput::neutral(), 600);
        for value in surface_deflections_from_simulation(&model, &neutral) {
            assert!(value.abs() < 1.0e-6);
        }
        assert_eq!(model.physics_fingerprint(), before);
    }

    #[test]
    fn absent_articulation_metadata_keeps_every_visual_surface_rigid() {
        let (model, _) = acro_model_with_explicit_presentation();
        let snapshot = stepped_snapshot(&model, PilotInput::new(1.0, 1.0, 1.0, 0.5), 600);
        assert_eq!(
            surface_deflections_from_simulation(&model, &snapshot),
            [0.0; 4]
        );
    }

    fn assert_quaternion_close(actual: [f64; 4], expected: [f64; 4]) {
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!(
                (actual - expected).abs() < 1.0e-12,
                "{actual} != {expected}"
            );
        }
    }

    #[test]
    fn alpha_formula_and_clamping_cover_boundaries() {
        assert_eq!(interpolation_alpha_seconds(0.0, 0.002), 0.0);
        assert_eq!(interpolation_alpha_seconds(0.001, 0.002), 0.5);
        assert_eq!(interpolation_alpha_seconds(0.002, 0.002), 1.0);
        assert_eq!(interpolation_alpha_seconds(-1.0, 0.002), 0.0);
        assert_eq!(interpolation_alpha_seconds(1.0, 0.002), 1.0);
    }

    #[test]
    fn endpoints_and_midpoint_interpolate_position_in_f64() {
        let previous = snapshot([10.0, 20.0, 30.0], IDENTITY);
        let current = snapshot([14.0, 26.0, 38.0], IDENTITY);
        assert_eq!(
            interpolate(previous, current, 0.0).position_world_ned_m,
            [10.0, 20.0, 30.0]
        );
        assert_eq!(
            interpolate(previous, current, 0.5).position_world_ned_m,
            [12.0, 23.0, 34.0]
        );
        assert_eq!(
            interpolate(previous, current, 1.0).position_world_ned_m,
            [14.0, 26.0, 38.0]
        );
    }

    #[test]
    fn same_quaternion_and_q_negated_q_are_equivalent() {
        let same = interpolate(
            snapshot([0.0; 3], IDENTITY),
            snapshot([0.0; 3], IDENTITY),
            0.5,
        );
        assert_quaternion_close(same.orientation_world_from_body_wxyz, IDENTITY);
        let negated = interpolate(
            snapshot([0.0; 3], IDENTITY),
            snapshot([0.0; 3], IDENTITY.map(|value| -value)),
            0.5,
        );
        assert_quaternion_close(negated.orientation_world_from_body_wxyz, IDENTITY);
    }

    #[test]
    fn ninety_degree_rotation_uses_normalized_shortest_path_slerp() {
        let half_angle = std::f64::consts::FRAC_PI_4;
        let end = [half_angle.cos(), 0.0, half_angle.sin(), 0.0];
        let midpoint = interpolate(snapshot([0.0; 3], IDENTITY), snapshot([0.0; 3], end), 0.5)
            .orientation_world_from_body_wxyz;
        let quarter_angle = std::f64::consts::PI / 8.0;
        assert_quaternion_close(
            midpoint,
            [quarter_angle.cos(), 0.0, quarter_angle.sin(), 0.0],
        );
        let norm = midpoint
            .into_iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        assert!((norm - 1.0).abs() < 1.0e-12);

        let negated_end = end.map(|value| -value);
        let shortest = interpolate(
            snapshot([0.0; 3], IDENTITY),
            snapshot([0.0; 3], negated_end),
            0.5,
        );
        assert_quaternion_close(shortest.orientation_world_from_body_wxyz, midpoint);
    }

    #[test]
    fn initial_and_single_snapshot_are_renderable_before_first_step() {
        let state = RigidBodyState {
            position_world_m: Vec3::new(1.0, 2.0, 3.0),
            linear_velocity_world_mps: Vec3::zeros(),
            orientation_world_from_body: Orientation::identity(),
            angular_velocity_body_radps: Vec3::zeros(),
        };
        let initial = AircraftRenderSnapshot::initial(&state);
        assert_eq!(initial.step_index, 0);
        assert_eq!(initial.sim_time_s, 0.0);
        let buffer = AircraftRenderSnapshotBuffer::new(initial);
        assert_eq!(buffer.previous, buffer.current);
        let pose = buffer.interpolated_pose(0.75, [0.0; 3]).unwrap();
        assert_eq!(pose.translation_render_m(), [2.0, -3.0, -1.0]);
    }

    #[test]
    fn large_coordinates_keep_small_relative_displacement_and_repeatability() {
        let origin = [1.0e12, -1.0e12, 5.0e11];
        let previous = snapshot(origin, IDENTITY);
        let current = snapshot(
            [origin[0] + 0.5, origin[1] + 1.0, origin[2] - 1.5],
            IDENTITY,
        );
        let buffer = AircraftRenderSnapshotBuffer { previous, current };
        let first = buffer.interpolated_pose(0.5, origin).unwrap();
        let second = buffer.interpolated_pose(0.5, origin).unwrap();
        assert_eq!(first.translation_render_m(), [0.5, 0.75, -0.25]);
        assert_eq!(first, second);
    }

    #[test]
    fn physics_replay_inputs_and_state_are_independent_of_render_frame_pattern() {
        let model_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let evidence = verify_frame_rate_independence(&model_path).unwrap();
        assert_eq!(evidence.pattern_count, 3);
        assert_eq!(evidence.physics_steps, 500);
        assert_eq!(evidence.render_snapshot_insertions, 500);
    }
}
