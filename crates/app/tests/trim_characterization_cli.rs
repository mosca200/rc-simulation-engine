#![forbid(unsafe_code)]

use serde_json::Value;
use std::{
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};

const JSON_REPORT: &str = "trim_characterization.json";
const MARKDOWN_REPORT: &str = "trim_characterization.md";

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
        loop {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_7b_{label}_{}_{}",
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

fn fixture_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/fixtures/synthetic_non_reference_trim_v4.json")
}

fn characterization_command(output_dir: &Path, alpha_step_rad: &str) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("analyze")
        .arg("trim-characterization")
        .arg("--model")
        .arg(fixture_path())
        .arg("--speed-mps")
        .arg("15")
        .arg("--speed-mps")
        .arg("18")
        .arg("--speed-mps")
        .arg("21")
        .arg("--alpha-min-rad")
        .arg("0.02")
        .arg("--alpha-max-rad")
        .arg("0.20")
        .arg("--elevator-min")
        .arg("-0.5")
        .arg("--elevator-max")
        .arg("0.5")
        .arg("--throttle-min")
        .arg("0.0")
        .arg("--throttle-max")
        .arg("1.0")
        .arg("--initial-alpha-rad")
        .arg("0.05")
        .arg("--initial-elevator")
        .arg("0.0")
        .arg("--initial-throttle")
        .arg("0.5")
        .arg("--force-tolerance-n")
        .arg("5.0")
        .arg("--moment-tolerance-nm")
        .arg("2.0")
        .arg("--max-iterations")
        .arg("50")
        .arg("--alpha-step-rad")
        .arg(alpha_step_rad)
        .arg("--elevator-step")
        .arg("0.01")
        .arg("--output-dir")
        .arg(output_dir);
    command
}

fn run_characterized(output_dir: &Path) -> Output {
    characterization_command(output_dir, "0.001")
        .output()
        .expect("the rcsim-app process must start")
}

fn report_names(output_dir: &Path) -> Vec<OsString> {
    let mut names: Vec<_> = fs::read_dir(output_dir)
        .expect("the output directory must exist")
        .map(|entry| {
            entry
                .expect("the output entry must be readable")
                .file_name()
        })
        .collect();
    names.sort();
    names
}

fn assert_exact_reports(output_dir: &Path) {
    assert_eq!(
        report_names(output_dir),
        vec![OsString::from(JSON_REPORT), OsString::from(MARKDOWN_REPORT)]
    );
    for forbidden in [
        "report.md",
        "report.json",
        "characterization.json",
        "results.json",
    ] {
        assert!(!output_dir.join(forbidden).exists());
    }
    for canonical in [JSON_REPORT, MARKDOWN_REPORT] {
        assert!(
            fs::metadata(output_dir.join(canonical))
                .expect("canonical report must exist")
                .len()
                > 0
        );
    }
}

#[test]
fn characterized_process_exits_zero_and_writes_exact_canonical_reports() {
    let root = TestDirectory::new("characterized");
    let output_dir = root.path().join("reports");
    let output = run_characterized(&output_dir);

    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_exact_reports(&output_dir);
    let report: Value = serde_json::from_slice(
        &fs::read(output_dir.join(JSON_REPORT)).expect("JSON report must be readable"),
    )
    .expect("JSON report must decode");
    assert_eq!(report["schema_version"], 1);
    assert_eq!(report["summary"]["total_points"], 3);
    assert_eq!(report["summary"]["characterized_count"], 3);
    assert!(
        report["points"]
            .as_array()
            .unwrap()
            .iter()
            .all(|point| point["outcome"] == "characterized")
    );
}

#[test]
fn unavailable_outcomes_are_data_and_still_exit_zero() {
    let root = TestDirectory::new("unavailable");
    let output_dir = root.path().join("reports");
    let output = characterization_command(&output_dir, "0.5")
        .output()
        .expect("the rcsim-app process must start");

    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_exact_reports(&output_dir);
    let report: Value = serde_json::from_slice(
        &fs::read(output_dir.join(JSON_REPORT)).expect("JSON report must be readable"),
    )
    .expect("JSON report must decode");
    assert_eq!(report["summary"]["characterization_unavailable_count"], 3);
    for point in report["points"].as_array().unwrap() {
        assert_eq!(point["outcome"], "characterization_unavailable");
        assert_eq!(
            point["unavailable"]["reason"],
            "alpha_perturbation_out_of_bounds"
        );
        assert!(point.get("pitch_stiffness_nm_per_rad").is_none());
        assert!(point.get("elevator_effectiveness_nm_per_command").is_none());
    }
}

#[test]
fn operational_error_exits_one_and_never_uses_validation_exit_two() {
    let output = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("analyze")
        .arg("trim-characterization")
        .output()
        .expect("the rcsim-app process must start");

    assert_eq!(output.status.code(), Some(1));
    assert_ne!(output.status.code(), Some(2));
    let stdout = String::from_utf8(output.stdout).expect("stdout must be UTF-8");
    let stderr = String::from_utf8(output.stderr).expect("stderr must be UTF-8");
    assert!(stderr.contains("at least one --speed-mps is required"));
    for verdict in ["PASS", "FAIL"] {
        assert!(!stdout.contains(verdict));
        assert!(!stderr.contains(verdict));
    }
}

#[test]
fn identical_processes_write_byte_identical_json_and_markdown() {
    let root = TestDirectory::new("determinism");
    let run1 = root.path().join("run1");
    let run2 = root.path().join("run2");

    assert_eq!(run_characterized(&run1).status.code(), Some(0));
    assert_eq!(run_characterized(&run2).status.code(), Some(0));
    let json1 = fs::read(run1.join(JSON_REPORT)).unwrap();
    let json2 = fs::read(run2.join(JSON_REPORT)).unwrap();
    let markdown1 = fs::read(run1.join(MARKDOWN_REPORT)).unwrap();
    let markdown2 = fs::read(run2.join(MARKDOWN_REPORT)).unwrap();
    assert_eq!(json1, json2);
    assert_eq!(markdown1, markdown2);

    let json = String::from_utf8(json1).unwrap();
    let markdown = String::from_utf8(markdown1).unwrap();
    for path in [&run1, &run2] {
        let path = path.to_string_lossy();
        assert!(!json.contains(path.as_ref()));
        assert!(!markdown.contains(path.as_ref()));
    }
}
