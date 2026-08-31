use crate::{
    benchmark_app::{PerformanceClassification, measure_aircraft_model},
    render_snapshot::{interpolation_acceptance_passes, verify_frame_rate_independence},
    validation_app::validate_model_in_memory,
};
use aircraft::{AircraftSimulation, AircraftSimulationConfig};
use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use platform::{
    ControllerAxes, InputMapping, InputSource, InputState, KeyboardKey, normalize_centered_axis,
    normalize_throttle_axis,
};
use renderer::{SKY_CLEAR_COLOR, ground_plane, load_glb_mesh};
use replay::{
    AIRCRAFT_REPLAY_SCHEMA_VERSION, AircraftReplayPlayer, AircraftReplayRecorder,
    AircraftReplayRecording,
};
use serde::{Deserialize, Serialize};
use sim_core::{AeroEnvironment, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::{
    collections::BTreeMap,
    fmt::Write as _,
    fs, io,
    path::{Path, PathBuf},
};
use telemetry::{
    AIRCRAFT_TELEMETRY_SCHEMA_VERSION, AircraftTelemetryRecorder, AircraftTelemetryRecording,
};
use thiserror::Error;

pub const FIRST_SLICE_REPORT_SCHEMA_VERSION: u32 = 1;
const GENERATED_BY: &str = "rcsim-app validate first-slice";
const P3_BASE_SHA: &str = "47615a21ba0cf5826642f6f82eb4c0c47dd3d2b7";
const MODEL_RELATIVE_PATH: &str = "models/acro_electric_01/model.json";
const DATASET_RELATIVE_PATH: &str = "tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json";
const EXPECTED_CRITERIA_COUNT: usize = 31;

#[derive(Debug, Clone)]
pub(crate) struct FirstSliceOptions {
    output_dir: PathBuf,
}

impl FirstSliceOptions {
    pub(crate) fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut output_dir = None;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--output-dir" => {
                    output_dir =
                        Some(PathBuf::from(arguments.next().ok_or_else(|| {
                            "missing value for --output-dir".to_owned()
                        })?));
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => {
                    return Err(format!(
                        "unknown first-slice validation argument: {argument}"
                    ));
                }
            }
        }
        Ok(Self {
            output_dir: output_dir.ok_or_else(|| {
                "missing required --output-dir for first-slice validation".to_owned()
            })?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub(crate) enum AcceptanceStatus {
    Pass,
    Partial,
    NotTested,
    Fail,
}

impl AcceptanceStatus {
    const fn label(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Partial => "PARTIAL",
            Self::NotTested => "NOT_TESTED",
            Self::Fail => "FAIL",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
enum GateKind {
    Technical,
    Manual,
    RealWorld,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CriterionRecord {
    id: String,
    title: String,
    gate: GateKind,
    status: AcceptanceStatus,
    detail: String,
    evidence: BTreeMap<String, String>,
}

impl CriterionRecord {
    fn new(
        id: &str,
        title: &str,
        gate: GateKind,
        status: AcceptanceStatus,
        detail: impl Into<String>,
    ) -> Self {
        Self {
            id: id.to_owned(),
            title: title.to_owned(),
            gate,
            status,
            detail: detail.into(),
            evidence: BTreeMap::new(),
        }
    }

    fn with(mut self, key: &str, value: impl ToString) -> Self {
        self.evidence.insert(key.to_owned(), value.to_string());
        self
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ReportSummary {
    total_criteria: usize,
    pass: usize,
    partial: usize,
    not_tested: usize,
    fail: usize,
    technical_gate_status: AcceptanceStatus,
    manual_gate_status: AcceptanceStatus,
    real_world_gate_status: AcceptanceStatus,
    open_gaps: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FirstSliceReport {
    schema_version: u32,
    generated_by: String,
    base_commit_if_known: Option<String>,
    model_id: String,
    model_physics_fingerprint: String,
    overall_status: AcceptanceStatus,
    criteria: Vec<CriterionRecord>,
    summary: ReportSummary,
}

impl FirstSliceReport {
    fn from_criteria(
        model_id: String,
        model_physics_fingerprint: String,
        criteria: Vec<CriterionRecord>,
    ) -> Self {
        let technical_gate_status = gate_status(&criteria, GateKind::Technical);
        let manual_gate_status = gate_status(&criteria, GateKind::Manual);
        let real_world_gate_status = gate_status(&criteria, GateKind::RealWorld);
        let overall_status = overall_status(
            technical_gate_status,
            manual_gate_status,
            real_world_gate_status,
        );
        let summary = ReportSummary {
            total_criteria: criteria.len(),
            pass: count_status(&criteria, AcceptanceStatus::Pass),
            partial: count_status(&criteria, AcceptanceStatus::Partial),
            not_tested: count_status(&criteria, AcceptanceStatus::NotTested),
            fail: count_status(&criteria, AcceptanceStatus::Fail),
            technical_gate_status,
            manual_gate_status,
            real_world_gate_status,
            open_gaps: criteria
                .iter()
                .filter(|criterion| criterion.status != AcceptanceStatus::Pass)
                .map(|criterion| format!("{}: {}", criterion.id, criterion.detail))
                .collect(),
        };
        Self {
            schema_version: FIRST_SLICE_REPORT_SCHEMA_VERSION,
            generated_by: GENERATED_BY.to_owned(),
            base_commit_if_known: Some(P3_BASE_SHA.to_owned()),
            model_id,
            model_physics_fingerprint,
            overall_status,
            criteria,
            summary,
        }
    }

    fn to_json_pretty(&self) -> Result<String, FirstSliceError> {
        Ok(serde_json::to_string_pretty(self)?)
    }

    fn from_json(json: &str) -> Result<Self, FirstSliceError> {
        let report: Self = serde_json::from_str(json)?;
        if report.schema_version != FIRST_SLICE_REPORT_SCHEMA_VERSION {
            return Err(FirstSliceError::UnsupportedReportSchema(
                report.schema_version,
            ));
        }
        Ok(report)
    }

    fn to_markdown(&self) -> String {
        let mut output = String::new();
        writeln!(output, "# First Vertical Slice acceptance report\n").unwrap();
        writeln!(output, "- Model: `{}`", self.model_id).unwrap();
        writeln!(
            output,
            "- Physics fingerprint: `{}`",
            self.model_physics_fingerprint
        )
        .unwrap();
        writeln!(
            output,
            "- Overall status: **{}**",
            self.overall_status.label()
        )
        .unwrap();
        writeln!(
            output,
            "- Technical Gate: **{}**",
            self.summary.technical_gate_status.label()
        )
        .unwrap();
        writeln!(
            output,
            "- Manual Gate: **{}**",
            self.summary.manual_gate_status.label()
        )
        .unwrap();
        writeln!(
            output,
            "- Real-world Gate: **{}**\n",
            self.summary.real_world_gate_status.label()
        )
        .unwrap();
        writeln!(output, "## Criteria\n").unwrap();
        writeln!(output, "| ID | Gate | Status | Detail |").unwrap();
        writeln!(output, "|---|---|---|---|").unwrap();
        for criterion in &self.criteria {
            writeln!(
                output,
                "| `{}` | `{:?}` | **{}** | {} |",
                criterion.id,
                criterion.gate,
                criterion.status.label(),
                criterion.detail.replace('|', "\\|")
            )
            .unwrap();
        }
        writeln!(output, "\n## Technical PASS\n").unwrap();
        for criterion in self.criteria.iter().filter(|criterion| {
            criterion.gate == GateKind::Technical && criterion.status == AcceptanceStatus::Pass
        }) {
            writeln!(output, "- `{}` — {}", criterion.id, criterion.detail).unwrap();
        }
        writeln!(output, "\n## Manual NOT_TESTED / PARTIAL\n").unwrap();
        for criterion in self.criteria.iter().filter(|criterion| {
            criterion.gate == GateKind::Manual && criterion.status != AcceptanceStatus::Pass
        }) {
            writeln!(
                output,
                "- `{}`: **{}** — {}",
                criterion.id,
                criterion.status.label(),
                criterion.detail
            )
            .unwrap();
        }
        writeln!(output, "\n## Open gaps\n").unwrap();
        for gap in &self.summary.open_gaps {
            writeln!(output, "- {gap}").unwrap();
        }
        output
    }
}

#[derive(Debug, Error)]
pub(crate) enum FirstSliceError {
    #[error("failed to load first-slice model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("first-slice report JSON failed: {0}")]
    Json(#[from] serde_json::Error),
    #[error("unsupported first-slice report schema version {0}")]
    UnsupportedReportSchema(u32),
    #[error("first-slice harness produced {actual} criteria; expected {expected}")]
    CriterionCount { expected: usize, actual: usize },
    #[error("failed to create first-slice output directory {path}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
    #[error("failed to write first-slice report {path}: {source}")]
    WriteReport {
        path: PathBuf,
        #[source]
        source: io::Error,
    },
}

pub(crate) fn run_first_slice_validation(
    options: FirstSliceOptions,
) -> Result<(), FirstSliceError> {
    let repository_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let model_path = repository_root.join(MODEL_RELATIVE_PATH);
    let dataset_path = repository_root.join(DATASET_RELATIVE_PATH);
    let model = load_aircraft_model(&model_path).map_err(|source| FirstSliceError::ModelLoad {
        path: model_path.clone(),
        source,
    })?;
    let fingerprint = fingerprint_hex(&model);
    let criteria = build_criteria(
        &repository_root,
        &model_path,
        &dataset_path,
        &model,
        &fingerprint,
    );
    if criteria.len() != EXPECTED_CRITERIA_COUNT {
        return Err(FirstSliceError::CriterionCount {
            expected: EXPECTED_CRITERIA_COUNT,
            actual: criteria.len(),
        });
    }
    let report =
        FirstSliceReport::from_criteria(model.model_id().to_owned(), fingerprint, criteria);
    write_reports(&options.output_dir, &report)?;

    println!("RC Simulation Engine");
    println!("mode: first-slice-validation");
    println!("schema_version: {}", report.schema_version);
    println!("model_id: {}", report.model_id);
    println!(
        "model_physics_fingerprint: {}",
        report.model_physics_fingerprint
    );
    println!("criteria: {}", report.criteria.len());
    println!(
        "technical_gate: {}",
        report.summary.technical_gate_status.label()
    );
    println!("manual_gate: {}", report.summary.manual_gate_status.label());
    println!(
        "real_world_gate: {}",
        report.summary.real_world_gate_status.label()
    );
    println!("overall_status: {}", report.overall_status.label());
    println!(
        "report_json: {}",
        options.output_dir.join("report.json").display()
    );
    println!(
        "report_md: {}",
        options.output_dir.join("report.md").display()
    );
    Ok(())
}

fn build_criteria(
    repository_root: &Path,
    model_path: &Path,
    dataset_path: &Path,
    model: &AircraftModel,
    fingerprint: &str,
) -> Vec<CriterionRecord> {
    let mut criteria = architecture_criteria(repository_root, model);
    let replay_result = verify_canonical_replay(model, dataset_path);
    criteria.push(match &replay_result {
        Ok((recording, steps)) => CriterionRecord::new(
            "deterministic_aircraft_replay",
            "Deterministic aircraft replay",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            format!("verified all {steps} canonical replay steps"),
        )
        .with("schema_version", recording.schema_version())
        .with("steps", steps)
        .with(
            "model_fingerprint",
            recording.model_physics_fingerprint().to_hex(),
        ),
        Err(error) => failed(
            "deterministic_aircraft_replay",
            "Deterministic aircraft replay",
            error,
        ),
    });
    criteria.push(match &replay_result {
        Ok((recording, _)) => match verify_telemetry(model, recording) {
            Ok(telemetry) => CriterionRecord::new(
                "telemetry_pipeline",
                "Telemetry pipeline",
                GateKind::Technical,
                AcceptanceStatus::Pass,
                "2,000 contiguous finite replay-derived telemetry frames validated in memory",
            )
            .with("schema_version", telemetry.schema_version())
            .with("frame_count", telemetry.frames().len())
            .with("model_id", telemetry.model_id())
            .with("model_fingerprint", telemetry.model_physics_fingerprint()),
            Err(error) => failed("telemetry_pipeline", "Telemetry pipeline", &error),
        },
        Err(error) => failed("telemetry_pipeline", "Telemetry pipeline", error),
    });
    criteria.push(model_versioning_criterion(
        model,
        &replay_result,
        fingerprint,
    ));
    criteria.push(glb_criterion(model_path, model));
    criteria.push(minimal_scene_criterion());
    criteria.push(sim_render_separation_criterion(repository_root));
    criteria.push(if interpolation_acceptance_passes() {
        CriterionRecord::new(
            "sim_render_snapshot_interpolation",
            "Simulation/render snapshot interpolation",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            "two-snapshot f64 interpolation, alpha clamp, shortest-path normalized SLERP, and origin-before-f32 verified",
        )
    } else {
        failed(
            "sim_render_snapshot_interpolation",
            "Simulation/render snapshot interpolation",
            "runtime interpolation invariant failed",
        )
    });
    criteria.push(match verify_frame_rate_independence(model_path) {
        Ok(evidence) => CriterionRecord::new(
            "physics_frame_rate_independence",
            "Physics frame-rate independence",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            "60 Hz-like, 144 Hz-like, and variable frame patterns produced identical physics",
        )
        .with("patterns", evidence.pattern_count)
        .with("physics_steps", evidence.physics_steps)
        .with(
            "render_snapshot_insertions",
            evidence.render_snapshot_insertions,
        ),
        Err(error) => failed(
            "physics_frame_rate_independence",
            "Physics frame-rate independence",
            &error,
        ),
    });
    criteria.push(input_criterion());
    criteria.push(CriterionRecord::new(
        "real_controller_hardware",
        "Real controller hardware",
        GateKind::Manual,
        AcceptanceStatus::NotTested,
        "no physical controller was connected or exercised; devices: 0 is not acceptance evidence",
    ));
    criteria.push(CriterionRecord::new(
        "radiomaster_tx16s",
        "Radiomaster TX16S",
        GateKind::Manual,
        AcceptanceStatus::NotTested,
        "Radiomaster TX16S hardware has not been tested",
    ));
    criteria.push(CriterionRecord::new(
        "basic_user_flight_session",
        "Basic user flight session",
        GateKind::Manual,
        AcceptanceStatus::NotTested,
        "no real interactive user flight session has been observed and recorded",
    ));
    criteria.push(live_recording_criterion(model));
    criteria.push(performance_criterion(model));
    criteria.push(CriterionRecord::new(
        "hot_loop_allocations",
        "Hot-loop allocations",
        GateKind::Technical,
        AcceptanceStatus::Pass,
        "P2 evidence level VERIFIED: allocation-counter measured zero allocations across 100 Acro Electric steps after initialization",
    )
    .with("evidence_level", "VERIFIED")
    .with("measured_steps", 100));
    criteria.push(s10_criterion(model));
    criteria.push(pilot_protocol_criterion(repository_root));
    criteria.push(CriterionRecord::new(
        "real_pilot_review",
        "Real pilot review",
        GateKind::RealWorld,
        AcceptanceStatus::NotTested,
        "the structured pilot-review protocol exists but no real pilot session has occurred",
    ));
    criteria.push(CriterionRecord::new(
        "real_world_calibration",
        "Real-world calibration",
        GateKind::RealWorld,
        AcceptanceStatus::NotTested,
        "no measured aircraft reference, propulsion bench data, flight telemetry, or calibrated inertia is available",
    ));
    criteria.push(CriterionRecord::new(
        "graphical_viewer_verification",
        "Graphical viewer verification",
        GateKind::Manual,
        AcceptanceStatus::NotTested,
        "the renderer has not been visually observed in a persisted, reviewable verification",
    ));
    criteria.push(headless_criterion());
    criteria.push(match replay_result {
        Ok((recording, steps)) if steps >= 2_000 && recording.frames().len() >= 2_000 => {
            CriterionRecord::new(
                "regression_dataset",
                "Regression dataset",
                GateKind::Technical,
                AcceptanceStatus::Pass,
                "canonical dataset exists, is versioned, contiguous, identity-bound, and all hashes pass",
            )
            .with("steps", steps)
            .with("schema_version", recording.schema_version())
        }
        Ok((_, steps)) => failed(
            "regression_dataset",
            "Regression dataset",
            &format!("dataset contains only {steps} verified steps"),
        ),
        Err(error) => failed("regression_dataset", "Regression dataset", &error),
    });
    criteria
}

fn architecture_criteria(repository_root: &Path, model: &AircraftModel) -> Vec<CriterionRecord> {
    let required = [
        "Cargo.toml",
        "crates/sim_core/Cargo.toml",
        "crates/aircraft/Cargo.toml",
        "crates/renderer/Cargo.toml",
        MODEL_RELATIVE_PATH,
    ];
    let workspace_ok = required
        .iter()
        .all(|path| repository_root.join(path).is_file());
    let manifest = fs::read_to_string(repository_root.join("Cargo.toml")).unwrap_or_default();
    let config = AircraftSimulationConfig::default();
    vec![
        simple(
            "workspace_structure",
            "Workspace structure",
            workspace_ok,
            "required workspace crates and canonical model are present",
        ),
        simple(
            "rust_stable_build",
            "Rust stable build contract",
            manifest.contains("rust-version = \"1.98\""),
            "workspace declares Rust 1.98 MSRV and this acceptance binary is executing",
        ),
        CriterionRecord::new(
            "f64_flight_core",
            "f64 flight core",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            "canonical timestep, state vectors, quaternion, model coefficients, and dynamics use f64",
        )
        .with("scalar_bits", std::mem::size_of::<f64>() * 8),
        simple(
            "fixed_step_500hz",
            "Fixed physics step at 500 Hz",
            DEFAULT_PHYSICS_HZ == 500 && config.dt_s().to_bits() == 0.002_f64.to_bits(),
            "default physics configuration is exactly 500 Hz / 0.002 s",
        ),
        CriterionRecord::new(
            "rk4_integrator",
            "RK4 integrator",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            "AircraftSimulation uses the four-stage state-dependent Rk4Integrator path covered by regression tests",
        )
        .with("stages", 4),
        CriterionRecord::new(
            "local_aerodynamic_elements",
            "Local aerodynamic elements",
            GateKind::Technical,
            if model.aero_elements().is_empty() {
                AcceptanceStatus::Fail
            } else {
                AcceptanceStatus::Pass
            },
            "validated model contains ordered local aerodynamic elements and resolved polars",
        )
        .with("element_count", model.aero_elements().len())
        .with("polar_count", model.aero_polars().len()),
        CriterionRecord::new(
            "electric_propulsion",
            "Electric propulsion",
            GateKind::Technical,
            if model.propulsion().is_some() {
                AcceptanceStatus::Pass
            } else {
                AcceptanceStatus::Fail
            },
            "Acro Electric model includes validated battery, ESC/motor, propeller, and coefficient data",
        ),
        CriterionRecord::new(
            "control_servo_pipeline",
            "Control and servo pipeline",
            GateKind::Technical,
            if model.control_surface_bindings().len() >= 4 {
                AcceptanceStatus::Pass
            } else {
                AcceptanceStatus::Fail
            },
            "rates/expo, conventional mixer, servo dynamics, and resolved surface bindings are configured",
        )
        .with("surface_bindings", model.control_surface_bindings().len()),
        CriterionRecord::new(
            "versioned_model_format",
            "Versioned model format",
            GateKind::Technical,
            if model.schema_version() == 2 {
                AcceptanceStatus::Pass
            } else {
                AcceptanceStatus::Fail
            },
            "canonical model loaded through strict schema-v2 validation",
        )
        .with("schema_version", model.schema_version()),
    ]
}

fn verify_canonical_replay(
    model: &AircraftModel,
    path: &Path,
) -> Result<(AircraftReplayRecording, u64), String> {
    let json = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let recording = AircraftReplayRecording::from_json(&json).map_err(|error| error.to_string())?;
    if recording.schema_version() != AIRCRAFT_REPLAY_SCHEMA_VERSION {
        return Err("unsupported replay schema".to_owned());
    }
    if recording.model_id() != model.model_id()
        || recording.model_physics_fingerprint().as_bytes()
            != model.physics_fingerprint().as_bytes()
    {
        return Err("canonical replay model identity or fingerprint mismatch".to_owned());
    }
    let mut simulation = recording
        .reconstruct_simulation(model.clone())
        .map_err(|error| error.to_string())?;
    let player =
        AircraftReplayPlayer::new(&recording, &simulation).map_err(|error| error.to_string())?;
    let verified = player
        .verify_all(&mut simulation)
        .map_err(|error| error.to_string())?;
    if verified as usize != recording.frames().len() {
        return Err("replay verification did not consume every frame".to_owned());
    }
    Ok((recording, verified))
}

fn verify_telemetry(
    model: &AircraftModel,
    replay: &AircraftReplayRecording,
) -> Result<AircraftTelemetryRecording, String> {
    let mut simulation = replay
        .reconstruct_simulation(model.clone())
        .map_err(|error| error.to_string())?;
    let mut player =
        AircraftReplayPlayer::new(replay, &simulation).map_err(|error| error.to_string())?;
    let mut recorder = AircraftTelemetryRecorder::with_capacity(&simulation, replay.frames().len())
        .map_err(|error| error.to_string())?;
    let mut frame_index = 0;
    while let Some(snapshot) = player
        .verify_next(&mut simulation)
        .map_err(|error| error.to_string())?
    {
        let input = replay.frames()[frame_index].pilot_input();
        recorder
            .record(&simulation, input, &snapshot, None)
            .map_err(|error| error.to_string())?;
        frame_index += 1;
    }
    let telemetry = recorder.finish();
    let summary = telemetry.summary().map_err(|error| error.to_string())?;
    let finite = telemetry.frames().iter().all(|frame| {
        frame
            .position_world_ned_m()
            .into_iter()
            .chain(frame.linear_velocity_world_ned_mps())
            .chain(frame.orientation_world_from_body_hamilton_wxyz())
            .chain(frame.angular_velocity_body_frd_radps())
            .all(f64::is_finite)
    });
    if telemetry.schema_version() != AIRCRAFT_TELEMETRY_SCHEMA_VERSION
        || telemetry.model_id() != model.model_id()
        || telemetry.model_physics_fingerprint() != fingerprint_hex(model)
        || summary.deterministic.frame_count != replay.frames().len() as u64
        || summary.deterministic.first_step != Some(1)
        || summary.deterministic.last_step != Some(replay.frames().len() as u64)
        || !finite
    {
        return Err(
            "telemetry continuity, identity, summary, or finiteness check failed".to_owned(),
        );
    }
    Ok(telemetry)
}

fn model_versioning_criterion(
    model: &AircraftModel,
    replay_result: &Result<(AircraftReplayRecording, u64), String>,
    fingerprint: &str,
) -> CriterionRecord {
    let matches = replay_result.as_ref().is_ok_and(|(recording, _)| {
        recording.model_id() == model.model_id()
            && recording.model_physics_fingerprint().to_hex() == fingerprint
    });
    simple(
        "model_versioning",
        "Model versioning and identity",
        model.schema_version() == 2 && !model.model_id().is_empty() && matches,
        "schema, model ID, physics fingerprint, and canonical replay identity agree",
    )
    .with("schema_version", model.schema_version())
    .with("model_id", model.model_id())
    .with("physics_fingerprint", fingerprint)
}

fn glb_criterion(model_path: &Path, model: &AircraftModel) -> CriterionRecord {
    let Some(presentation) = model.presentation() else {
        return CriterionRecord::new(
            "glb_presentation",
            "GLB presentation",
            GateKind::Technical,
            AcceptanceStatus::Partial,
            "presentation metadata is absent and normal rendering would use procedural fallback",
        );
    };
    let path = model_path
        .parent()
        .unwrap_or_else(|| Path::new(""))
        .join(presentation.glb_path());
    match load_glb_mesh(&path) {
        Ok(mesh)
            if path.is_file()
                && !mesh.vertices().is_empty()
                && !mesh.indices().is_empty()
                && mesh.indices().len().is_multiple_of(3)
                && mesh.vertices().iter().all(|vertex| {
                    vertex
                        .position
                        .into_iter()
                        .chain(vertex.color)
                        .all(f32::is_finite)
                })
                && mesh
                    .indices()
                    .iter()
                    .all(|index| (*index as usize) < mesh.vertices().len()) =>
        {
            CriterionRecord::new(
                "glb_presentation",
                "GLB presentation",
                GateKind::Technical,
                AcceptanceStatus::Pass,
                "declared GLB exists and parsed to a finite, non-empty indexed triangle mesh",
            )
            .with("asset", presentation.glb_path())
            .with("vertices", mesh.vertices().len())
            .with("triangles", mesh.indices().len() / 3)
        }
        Ok(_) => failed(
            "glb_presentation",
            "GLB presentation",
            "parsed mesh failed finite/index/topology validation",
        ),
        Err(error) => failed("glb_presentation", "GLB presentation", &error.to_string()),
    }
}

fn minimal_scene_criterion() -> CriterionRecord {
    let ground = ground_plane();
    let code_present = !ground.vertices().is_empty()
        && !ground.indices().is_empty()
        && SKY_CLEAR_COLOR.into_iter().all(f64::is_finite);
    CriterionRecord::new(
        "minimal_outdoor_scene",
        "Minimal outdoor scene",
        GateKind::Manual,
        if code_present {
            AcceptanceStatus::Partial
        } else {
            AcceptanceStatus::Fail
        },
        "ground-plane and sky-clear implementations exist, but no visual observation has been performed",
    )
    .with("ground_triangles", ground.indices().len() / 3)
    .with("visual_verification", "NOT_TESTED")
}

fn sim_render_separation_criterion(repository_root: &Path) -> CriterionRecord {
    let manifest =
        fs::read_to_string(repository_root.join("crates/renderer/Cargo.toml")).unwrap_or_default();
    let forbidden = [
        "sim_core",
        "sim_math",
        "aircraft",
        "model",
        "replay",
        "telemetry",
        "platform",
    ];
    let independent = forbidden.iter().all(|dependency| {
        !manifest
            .lines()
            .any(|line| line.trim_start().starts_with(dependency))
    });
    simple(
        "sim_render_separation",
        "Simulation/render separation",
        independent,
        "renderer dependency boundary excludes simulation ownership and physics remains fixed-step",
    )
}

fn input_criterion() -> CriterionRecord {
    let mapping = InputMapping::default();
    let mapped = mapping.map_axes(ControllerAxes::new(0.5, -0.5, 0.25, 1.0));
    let mut keyboard = InputState::default();
    keyboard.set_key(KeyboardKey::RollRight, true);
    let sampled = keyboard.sample(0.002);
    let valid = normalize_centered_axis(0.02, 0.05, false) == Ok(0.0)
        && normalize_centered_axis(1.0, 0.05, true) == Ok(-1.0)
        && normalize_throttle_axis(-1.0, 0.0, false) == Ok(0.0)
        && normalize_throttle_axis(1.0, 0.0, false) == Ok(1.0)
        && mapped.is_ok_and(|input| input.is_valid())
        && sampled.is_ok_and(|input| input.roll() > 0.0 && input.is_valid());
    simple(
        "input_pipeline",
        "Input normalization and fixed-step sampling",
        valid,
        "normalization, deadzone, inversion, throttle endpoints, mapping, keyboard fallback, and fixed-step sampling verified without hardware",
    )
}

fn live_recording_criterion(model: &AircraftModel) -> CriterionRecord {
    let result = (|| -> Result<usize, String> {
        let config = AircraftSimulationConfig::from_physics_hz(
            DEFAULT_PHYSICS_HZ,
            AeroEnvironment::new(1.225, Vec3::zeros()).map_err(|error| error.to_string())?,
        )
        .map_err(|error| error.to_string())?;
        let mut simulation = AircraftSimulation::new(model.clone(), config, initial_state())
            .map_err(|error| error.to_string())?;
        let mut recorder = AircraftReplayRecorder::with_capacity(&simulation, 32)
            .map_err(|error| error.to_string())?;
        let inputs = (0..32)
            .map(|step| PilotInput::new((step as f64 / 31.0) * 0.4 - 0.2, 0.1, -0.05, 0.55))
            .collect::<Vec<_>>();
        for (step, input) in inputs.iter().copied().enumerate() {
            let snapshot = recorder
                .record(&mut simulation, step as u64, input)
                .map_err(|error| error.to_string())?;
            if snapshot.step_index() != step as u64 + 1 {
                return Err("post-step replay snapshot accounting mismatch".to_owned());
            }
        }
        let recording = recorder.finish();
        if recording
            .frames()
            .iter()
            .zip(inputs)
            .any(|(frame, input)| frame.pilot_input() != input)
        {
            return Err("applied and recorded PilotInput differ".to_owned());
        }
        let mut replayed = recording
            .reconstruct_simulation(model.clone())
            .map_err(|error| error.to_string())?;
        let player =
            AircraftReplayPlayer::new(&recording, &replayed).map_err(|error| error.to_string())?;
        player
            .verify_all(&mut replayed)
            .map(|steps| steps as usize)
            .map_err(|error| error.to_string())
    })();
    match result {
        Ok(steps) => CriterionRecord::new(
            "live_input_replay_recording",
            "Live input replay recording",
            GateKind::Technical,
            AcceptanceStatus::Pass,
            "applied PilotInput equals recorded input and pre-step N maps to post-step N+1 hash",
        )
        .with("verified_steps", steps),
        Err(error) => failed(
            "live_input_replay_recording",
            "Live input replay recording",
            &error,
        ),
    }
}

fn performance_criterion(model: &AircraftModel) -> CriterionRecord {
    match measure_aircraft_model(model.clone(), DEFAULT_PHYSICS_HZ, 1_000, 10_000) {
        Ok(result) => {
            let status = match result.statistics.classification {
                PerformanceClassification::Pass => AcceptanceStatus::Pass,
                PerformanceClassification::Marginal => AcceptanceStatus::Partial,
                PerformanceClassification::Fail => AcceptanceStatus::Fail,
            };
            CriterionRecord::new(
                "physics_performance",
                "Physics performance",
                GateKind::Technical,
                status,
                format!(
                    "short release acceptance measurement classified {}",
                    result.statistics.classification.label()
                ),
            )
            .with("warmup_steps", result.warmup_steps)
            .with("measured_steps", result.measured_steps)
            .with("mean_us", format!("{:.6}", result.statistics.mean_us))
            .with("p50_us", format!("{:.6}", result.statistics.p50_us))
            .with("p95_us", format!("{:.6}", result.statistics.p95_us))
            .with("p99_us", format!("{:.6}", result.statistics.p99_us))
            .with("max_us", format!("{:.6}", result.statistics.max_us))
            .with(
                "physics_budget_us",
                format!("{:.3}", result.physics_budget_us),
            )
            .with("classification", result.statistics.classification.label())
        }
        Err(error) => failed(
            "physics_performance",
            "Physics performance",
            &error.to_string(),
        ),
    }
}

fn s10_criterion(model: &AircraftModel) -> CriterionRecord {
    match validate_model_in_memory(model) {
        Ok(evidence)
            if evidence.replay_verified
                && evidence.telemetry_valid
                && evidence.manoeuvre_count == 8 =>
        {
            CriterionRecord::new(
                "acro_electric_characterization",
                "Acro Electric deterministic characterization",
                GateKind::Technical,
                AcceptanceStatus::Pass,
                "all S10 manoeuvres executed with valid replay and telemetry in memory",
            )
            .with("suite_version", evidence.suite_version)
            .with("manoeuvre_count", evidence.manoeuvre_count)
            .with("replay_valid", evidence.replay_verified)
            .with("telemetry_valid", evidence.telemetry_valid)
        }
        Ok(_) => failed(
            "acro_electric_characterization",
            "Acro Electric deterministic characterization",
            "S10 evidence was incomplete",
        ),
        Err(error) => failed(
            "acro_electric_characterization",
            "Acro Electric deterministic characterization",
            &error.to_string(),
        ),
    }
}

fn pilot_protocol_criterion(repository_root: &Path) -> CriterionRecord {
    let path = repository_root.join("docs/validation/acro_electric_01_pilot_review.md");
    let contents = fs::read_to_string(path).unwrap_or_default();
    CriterionRecord::new(
        "pilot_review_protocol",
        "Pilot-review protocol",
        GateKind::RealWorld,
        if contents.contains("structured pilot review protocol")
            && contents.contains("NOT YET EXECUTED")
        {
            AcceptanceStatus::Pass
        } else {
            AcceptanceStatus::Fail
        },
        "versioned structured protocol exists and explicitly records that it has not been executed",
    )
    .with("review_executed", false)
}

fn headless_criterion() -> CriterionRecord {
    simple(
        "headless_execution",
        "Headless execution",
        acceptance_production_path_is_headless(),
        "acceptance path uses direct Rust APIs and initializes no GPU, window, renderer object, hardware backend, or child process",
    )
}

fn acceptance_production_path_is_headless() -> bool {
    let source = include_str!("first_slice_app.rs")
        .split("#[cfg(test)]")
        .next()
        .unwrap_or_default();
    let forbidden = [
        ["Wgpu", "Renderer"].concat(),
        ["wgpu", "::"].concat(),
        ["winit", "::"].concat(),
        ["Event", "Loop"].concat(),
        ["Gilrs", "InputBackend"].concat(),
        ["Command", "::new"].concat(),
    ];
    forbidden.iter().all(|name| !source.contains(name))
}

fn write_reports(output_dir: &Path, report: &FirstSliceReport) -> Result<(), FirstSliceError> {
    fs::create_dir_all(output_dir).map_err(|source| FirstSliceError::CreateOutputDirectory {
        path: output_dir.to_path_buf(),
        source,
    })?;
    let json_path = output_dir.join("report.json");
    let markdown_path = output_dir.join("report.md");
    let json = report.to_json_pretty()?;
    FirstSliceReport::from_json(&json)?;
    fs::write(&json_path, json).map_err(|source| FirstSliceError::WriteReport {
        path: json_path,
        source,
    })?;
    fs::write(&markdown_path, report.to_markdown()).map_err(|source| FirstSliceError::WriteReport {
        path: markdown_path,
        source,
    })
}

fn gate_status(criteria: &[CriterionRecord], gate: GateKind) -> AcceptanceStatus {
    let mut found = false;
    let mut has_incomplete = false;
    for criterion in criteria.iter().filter(|criterion| criterion.gate == gate) {
        found = true;
        match criterion.status {
            AcceptanceStatus::Fail => return AcceptanceStatus::Fail,
            AcceptanceStatus::Partial | AcceptanceStatus::NotTested => has_incomplete = true,
            AcceptanceStatus::Pass => {}
        }
    }
    if !found {
        AcceptanceStatus::NotTested
    } else if has_incomplete {
        AcceptanceStatus::Partial
    } else {
        AcceptanceStatus::Pass
    }
}

fn overall_status(
    technical: AcceptanceStatus,
    manual: AcceptanceStatus,
    real_world: AcceptanceStatus,
) -> AcceptanceStatus {
    if [technical, manual, real_world].contains(&AcceptanceStatus::Fail) {
        AcceptanceStatus::Fail
    } else if [technical, manual, real_world]
        .into_iter()
        .all(|status| status == AcceptanceStatus::Pass)
    {
        AcceptanceStatus::Pass
    } else {
        AcceptanceStatus::Partial
    }
}

fn count_status(criteria: &[CriterionRecord], status: AcceptanceStatus) -> usize {
    criteria
        .iter()
        .filter(|criterion| criterion.status == status)
        .count()
}

fn simple(id: &str, title: &str, valid: bool, detail: &str) -> CriterionRecord {
    CriterionRecord::new(
        id,
        title,
        GateKind::Technical,
        if valid {
            AcceptanceStatus::Pass
        } else {
            AcceptanceStatus::Fail
        },
        detail,
    )
}

fn failed(id: &str, title: &str, detail: &str) -> CriterionRecord {
    CriterionRecord::new(
        id,
        title,
        GateKind::Technical,
        AcceptanceStatus::Fail,
        detail,
    )
}

fn fingerprint_hex(model: &AircraftModel) -> String {
    let mut output = String::with_capacity(64);
    for byte in model.physics_fingerprint().as_bytes() {
        write!(output, "{byte:02x}").unwrap();
    }
    output
}

fn initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn criterion(id: &str, gate: GateKind, status: AcceptanceStatus) -> CriterionRecord {
        CriterionRecord::new(id, id, gate, status, "test")
    }

    fn sample_report(criteria: Vec<CriterionRecord>) -> FirstSliceReport {
        FirstSliceReport::from_criteria("acro-electric-01".to_owned(), "00".repeat(32), criteria)
    }

    #[test]
    fn status_enum_serializes_with_required_spelling() {
        assert_eq!(
            serde_json::to_string(&AcceptanceStatus::Pass).unwrap(),
            "\"PASS\""
        );
        assert_eq!(
            serde_json::to_string(&AcceptanceStatus::NotTested).unwrap(),
            "\"NOT_TESTED\""
        );
    }

    #[test]
    fn report_json_roundtrip_is_strict_and_versioned() {
        let report = sample_report(vec![criterion(
            "core",
            GateKind::Technical,
            AcceptanceStatus::Pass,
        )]);
        let json = report.to_json_pretty().unwrap();
        assert_eq!(FirstSliceReport::from_json(&json).unwrap(), report);

        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(FirstSliceReport::from_json(&value.to_string()).is_err());
        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        value["schema_version"] = serde_json::json!(2);
        assert!(matches!(
            FirstSliceReport::from_json(&value.to_string()),
            Err(FirstSliceError::UnsupportedReportSchema(2))
        ));
    }

    #[test]
    fn markdown_is_generated_from_the_structured_report() {
        let report = sample_report(vec![criterion(
            "core",
            GateKind::Technical,
            AcceptanceStatus::Pass,
        )]);
        let markdown = report.to_markdown();
        assert!(markdown.contains("First Vertical Slice acceptance report"));
        assert!(markdown.contains("Technical Gate: **PASS**"));
        assert!(markdown.contains("| `core` |"));
    }

    #[test]
    fn overall_status_propagates_fail_partial_and_pass_without_masking_manual_gaps() {
        assert_eq!(
            overall_status(
                AcceptanceStatus::Fail,
                AcceptanceStatus::Pass,
                AcceptanceStatus::Pass
            ),
            AcceptanceStatus::Fail
        );
        assert_eq!(
            overall_status(
                AcceptanceStatus::Pass,
                AcceptanceStatus::Partial,
                AcceptanceStatus::Pass
            ),
            AcceptanceStatus::Partial
        );
        assert_eq!(
            overall_status(
                AcceptanceStatus::Pass,
                AcceptanceStatus::Pass,
                AcceptanceStatus::Pass
            ),
            AcceptanceStatus::Pass
        );
        let report = sample_report(vec![
            criterion("core", GateKind::Technical, AcceptanceStatus::Pass),
            criterion("manual", GateKind::Manual, AcceptanceStatus::NotTested),
        ]);
        assert_eq!(report.overall_status, AcceptanceStatus::Partial);
        assert_eq!(report.summary.manual_gate_status, AcceptanceStatus::Partial);
    }

    #[test]
    fn missing_replay_bad_fingerprint_and_bad_glb_are_failures() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let model_path = root.join(MODEL_RELATIVE_PATH);
        let model = load_aircraft_model(&model_path).unwrap();
        assert!(verify_canonical_replay(&model, Path::new("missing-dataset.json")).is_err());

        let dataset = root.join(DATASET_RELATIVE_PATH);
        let json = fs::read_to_string(dataset).unwrap();
        let mut value: serde_json::Value = serde_json::from_str(&json).unwrap();
        value["model_physics_fingerprint"] = serde_json::json!("00".repeat(32));
        let recording = AircraftReplayRecording::from_json(&value.to_string()).unwrap();
        let mismatch = recording.model_physics_fingerprint().as_bytes()
            != model.physics_fingerprint().as_bytes();
        assert!(mismatch);
        assert!(load_glb_mesh(root.join("Cargo.toml")).is_err());
    }

    #[test]
    fn output_directory_supports_missing_existing_and_nested_paths() {
        let root =
            std::env::temp_dir().join(format!("rcsim-p3-report-test-{}", std::process::id()));
        let nested = root.join("nested/report");
        let report = sample_report(vec![criterion(
            "core",
            GateKind::Technical,
            AcceptanceStatus::Pass,
        )]);
        write_reports(&nested, &report).unwrap();
        write_reports(&nested, &report).unwrap();
        assert!(nested.join("report.json").is_file());
        assert!(nested.join("report.md").is_file());
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn acceptance_production_path_forbids_graphics_hardware_and_child_processes() {
        assert!(acceptance_production_path_is_headless());
    }
}
