#![forbid(unsafe_code)]

use serde_json::Value;
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new() -> Self {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
        loop {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "rcsim_propulsion_bench_{}_{}",
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

fn repository_path(relative: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(relative)
}

fn bench(model: &Path, arguments: &[&str]) -> Output {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("propulsion")
        .arg("bench")
        .arg("--model")
        .arg(model);
    command.args(arguments);
    command.output().expect("propulsion bench must launch")
}

#[test]
fn single_point_json_reports_static_and_flight_operating_points() {
    let model = repository_path("models/acro_electric_01/model.json");
    let static_output = bench(
        &model,
        &[
            "--throttle",
            "1.0",
            "--airspeed-mps",
            "0",
            "--format",
            "json",
        ],
    );
    assert!(
        static_output.status.success(),
        "{}",
        String::from_utf8_lossy(&static_output.stderr)
    );
    let static_report: Value = serde_json::from_slice(&static_output.stdout).unwrap();
    let static_point = &static_report["operating_points"][0];
    assert_eq!(static_report["model_id"], "acro-electric-01");
    assert_eq!(static_point["throttle"], 1.0);
    assert_eq!(static_point["axial_inflow_mps"], 0.0);
    assert_eq!(static_point["useful_propulsive_power_w"], 0.0);
    assert!(static_point["propulsive_efficiency"].is_null());
    assert!(static_point["thrust_n"].as_f64().unwrap() > 0.0);
    assert!(static_point["battery_current_a"].as_f64().unwrap() > 0.0);
    assert!(static_point["shaft_speed_rpm"].as_f64().unwrap() > 0.0);

    let flight_output = bench(
        &model,
        &[
            "--throttle",
            "0.5",
            "--airspeed-mps",
            "15",
            "--format",
            "json",
        ],
    );
    assert!(flight_output.status.success());
    let flight_report: Value = serde_json::from_slice(&flight_output.stdout).unwrap();
    let flight_point = &flight_report["operating_points"][0];
    assert_eq!(flight_point["throttle"], 0.5);
    assert_eq!(flight_point["airspeed_mps"], 15.0);
    assert!(flight_point["advance_ratio_j"].as_f64().unwrap() > 0.0);
    assert!(flight_point["useful_propulsive_power_w"].as_f64().unwrap() >= 0.0);
}

#[test]
fn default_sweep_is_byte_deterministic_and_ordered() {
    let model = repository_path("models/acro_electric_01/model.json");
    let first = bench(&model, &["--format", "json"]);
    let second = bench(&model, &["--format", "json"]);
    assert!(first.status.success());
    assert!(second.status.success());
    assert_eq!(first.stdout, second.stdout);
    let report: Value = serde_json::from_slice(&first.stdout).unwrap();
    let points = report["operating_points"].as_array().unwrap();
    assert_eq!(points.len(), 30);
    assert_eq!(points.first().unwrap()["throttle"], 0.0);
    assert_eq!(points.first().unwrap()["airspeed_mps"], 0.0);
    assert_eq!(points.last().unwrap()["throttle"], 1.0);
    assert_eq!(points.last().unwrap()["airspeed_mps"], 25.0);
    let text = String::from_utf8(first.stdout).unwrap();
    for forbidden in ["timestamp", "wall_time", "hostname"] {
        assert!(!text.contains(forbidden));
    }
}

#[test]
fn csv_has_stable_field_order_and_map_fixture_executes() {
    let model = repository_path("tests/fixtures/synthetic_non_reference_propulsion_v4.json");
    let output = bench(
        &model,
        &[
            "--throttle",
            "0.5",
            "--airspeed-mps",
            "10",
            "--format",
            "csv",
        ],
    );
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let csv = String::from_utf8(output.stdout).unwrap();
    let mut lines = csv.lines();
    let header = lines.next().unwrap();
    assert!(
        header.starts_with("schema_version,model_id,model_physics_fingerprint,coefficient_source")
    );
    assert!(header.ends_with("coefficient_shaft_speed_range_status"));
    let row = lines.next().unwrap();
    assert!(row.contains("synthetic_non_reference_propulsion_v4"));
    assert!(row.contains("shaft_speed_map"));
    assert!(lines.next().is_none());
}

#[test]
fn output_uses_create_new_and_never_overwrites_existing_file() {
    let root = TestDirectory::new();
    let output_path = root.path().join("bench.json");
    fs::write(&output_path, b"unrelated-content").unwrap();
    let model = repository_path("models/acro_electric_01/model.json");
    let output = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("propulsion")
        .arg("bench")
        .arg("--model")
        .arg(model)
        .arg("--format")
        .arg("json")
        .arg("--output")
        .arg(&output_path)
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert_eq!(fs::read(&output_path).unwrap(), b"unrelated-content");
}
