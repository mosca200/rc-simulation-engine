#![forbid(unsafe_code)]

use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};
use telemetry::AircraftTelemetryRecording;

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new() -> Self {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
        loop {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_10b_experiment_{}_{}",
                std::process::id(),
                id
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("failed to create test directory {path:?}: {error}"),
            }
        }
    }

    fn path(&self) -> &Path {
        &self.0
    }
}

impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.0);
    }
}

fn model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../models/acro_electric_01/model.json")
}

fn experiment_command(schedule: &Path, output: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("telemetry")
        .arg("experiment")
        .arg("--model")
        .arg(model_path())
        .arg("--schedule")
        .arg(schedule)
        .arg("--output")
        .arg(output)
        .output()
        .expect("experiment command should launch")
}

#[test]
fn experiment_cli_is_deterministic_and_analyze_reads_its_capture() {
    let root = TestDirectory::new();
    let schedule_path = root.path().join("schedule.json");
    let first_output_path = root.path().join("first.jsonl");
    let second_output_path = root.path().join("second.jsonl");
    fs::write(
        &schedule_path,
        r#"{
  "schema_version": 1,
  "physics_hz": 500,
  "initial_state": {
    "altitude_m": 100.0,
    "airspeed_mps": 18.0,
    "pitch_attitude_rad": 0.0,
    "angular_velocity_body_radps": [0.0, 0.0, 0.0]
  },
  "phases": [
    {"name": "neutral", "steps": 4, "input": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5}},
    {"name": "pitch_pulse", "steps": 2, "input": {"roll": 0.0, "pitch": 0.2, "yaw": 0.0, "throttle": 0.5}},
    {"name": "release", "steps": 4, "input": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.5}}
  ]
}"#,
    )
    .unwrap();

    let first = experiment_command(&schedule_path, &first_output_path);
    assert!(
        first.status.success(),
        "{}",
        String::from_utf8_lossy(&first.stderr)
    );
    let stdout = String::from_utf8(first.stdout).unwrap();
    assert!(stdout.contains("mode: telemetry-experiment"));
    assert!(stdout.contains("total_steps: 10"));
    assert!(stdout.contains("frames_recorded: 10"));
    assert!(stdout.contains("phase[0]: neutral steps=4 range=[1,4]"));
    assert!(stdout.contains("phase[1]: pitch_pulse steps=2 range=[5,6]"));
    assert!(stdout.contains("phase[2]: release steps=4 range=[7,10]"));

    let second = experiment_command(&schedule_path, &second_output_path);
    assert!(
        second.status.success(),
        "{}",
        String::from_utf8_lossy(&second.stderr)
    );
    let first_bytes = fs::read(&first_output_path).unwrap();
    let second_bytes = fs::read(&second_output_path).unwrap();
    assert_eq!(first_bytes, second_bytes);

    let recording =
        AircraftTelemetryRecording::from_json_lines(std::str::from_utf8(&first_bytes).unwrap())
            .unwrap();
    assert_eq!(recording.frames().len(), 10);
    assert!(
        recording
            .frames()
            .iter()
            .all(|frame| frame.physics_step_wall_time_s().is_none())
    );
    assert!(
        recording.frames()[0..4]
            .iter()
            .all(|frame| frame.pilot_input().pitch() == 0.0)
    );
    assert!(
        recording.frames()[4..6]
            .iter()
            .all(|frame| frame.pilot_input().pitch() == 0.2)
    );
    assert!(
        recording.frames()[6..10]
            .iter()
            .all(|frame| frame.pilot_input().pitch() == 0.0)
    );

    let analyze = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("telemetry")
        .arg("analyze")
        .arg("--input")
        .arg(&first_output_path)
        .output()
        .expect("telemetry analyze command should launch");
    assert!(
        analyze.status.success(),
        "{}",
        String::from_utf8_lossy(&analyze.stderr)
    );
    let analyze_stdout = String::from_utf8(analyze.stdout).unwrap();
    assert!(analyze_stdout.contains("mode: telemetry-analyze"));
    assert!(analyze_stdout.contains("frame_count: 10"));
    assert!(analyze_stdout.contains("physics_step_wall_time_s: unavailable"));
}
