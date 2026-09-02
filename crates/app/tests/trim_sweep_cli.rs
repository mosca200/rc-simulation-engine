#![forbid(unsafe_code)]

use serde_json::Value;
use std::{
    collections::BTreeSet,
    ffi::OsString,
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};

const JSON_REPORT: &str = "trim_sweep.json";
const MARKDOWN_REPORT: &str = "trim_sweep.md";
const REPORT_SCHEMA_VERSION: u64 = 1;

struct TestDirectory {
    path: PathBuf,
}

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT_ID: AtomicUsize = AtomicUsize::new(0);
        loop {
            let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_6b1_{label}_{}_{}",
                std::process::id(),
                id
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Self { path },
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("failed to create test directory {path:?}: {error}"),
            }
        }
    }

    fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TestDirectory {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

fn fixture_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/fixtures/synthetic_non_reference_trim_v4.json")
}

fn trim_sweep_command(
    output_dir: &Path,
    alpha_min: &str,
    alpha_max: &str,
    force_tolerance: &str,
    moment_tolerance: &str,
    maximum_iterations: &str,
) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("validate")
        .arg("trim-sweep")
        .arg("--model")
        .arg(fixture_path())
        .arg("--speed-mps")
        .arg("18")
        .arg("--alpha-min-rad")
        .arg(alpha_min)
        .arg("--alpha-max-rad")
        .arg(alpha_max)
        .arg("--elevator-min")
        .arg("-0.9")
        .arg("--elevator-max")
        .arg("0.9")
        .arg("--throttle-min")
        .arg("0.02")
        .arg("--throttle-max")
        .arg("1.0")
        .arg("--initial-alpha-rad")
        .arg("0.08")
        .arg("--initial-elevator")
        .arg("0.1")
        .arg("--initial-throttle")
        .arg("0.45")
        .arg("--force-tolerance-n")
        .arg(force_tolerance)
        .arg("--moment-tolerance-nm")
        .arg(moment_tolerance)
        .arg("--max-iterations")
        .arg(maximum_iterations)
        .arg("--output-dir")
        .arg(output_dir);
    command
}

fn run_success(output_dir: &Path) -> Output {
    trim_sweep_command(output_dir, "-0.15", "0.30", "1e-6", "1e-7", "40")
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

fn assert_exact_canonical_reports(output_dir: &Path) {
    assert_eq!(
        report_names(output_dir),
        vec![OsString::from(JSON_REPORT), OsString::from(MARKDOWN_REPORT)]
    );
    assert!(!output_dir.join("report.md").exists());
    assert!(!output_dir.join("report.json").exists());
    for name in [JSON_REPORT, MARKDOWN_REPORT] {
        let metadata = fs::metadata(output_dir.join(name)).expect("canonical report must exist");
        assert!(metadata.len() > 0, "{name} must be non-empty");
    }
}

#[test]
fn successful_process_exits_zero_and_writes_only_canonical_reports() {
    let root = TestDirectory::new("pass");
    let output_dir = root.path().join("reports");
    let output = run_success(&output_dir);

    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_exact_canonical_reports(&output_dir);

    let json_bytes = fs::read(output_dir.join(JSON_REPORT)).expect("JSON report must be readable");
    let report: Value = serde_json::from_slice(&json_bytes).expect("JSON report must be valid");
    assert_eq!(
        report["schema_version"].as_u64(),
        Some(REPORT_SCHEMA_VERSION)
    );
    let keys: BTreeSet<_> = report
        .as_object()
        .expect("report root must be an object")
        .keys()
        .map(String::as_str)
        .collect();
    assert_eq!(
        keys,
        BTreeSet::from([
            "environment",
            "generated_by",
            "model",
            "points",
            "request",
            "schema_version",
            "summary",
        ])
    );
    assert_eq!(report["summary"]["overall_status"], "PASS");
}

#[test]
fn validation_failure_process_exits_two_after_writing_reports() {
    let root = TestDirectory::new("validation_fail");
    let output_dir = root.path().join("reports");
    let output = trim_sweep_command(&output_dir, "0.20", "0.21", "1e-12", "1e-13", "3")
        .output()
        .expect("the rcsim-app process must start");

    assert_eq!(output.status.code(), Some(2));
    assert_exact_canonical_reports(&output_dir);
    let stderr = String::from_utf8(output.stderr).expect("stderr must be UTF-8");
    assert!(stderr.contains("validation completed with FAIL"));
    assert!(stderr.contains(MARKDOWN_REPORT));
    assert!(!stderr.contains("report.md"));

    let report: Value = serde_json::from_slice(
        &fs::read(output_dir.join(JSON_REPORT)).expect("JSON report must be readable"),
    )
    .expect("JSON report must be valid");
    assert_eq!(report["summary"]["overall_status"], "FAIL");
}

#[test]
fn operational_error_process_exits_one_without_false_pass() {
    let output = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("validate")
        .arg("trim-sweep")
        .output()
        .expect("the rcsim-app process must start");

    assert_eq!(output.status.code(), Some(1));
    assert_ne!(output.status.code(), Some(2));
    let stdout = String::from_utf8(output.stdout).expect("stdout must be UTF-8");
    let stderr = String::from_utf8(output.stderr).expect("stderr must be UTF-8");
    assert!(stderr.contains("at least one --speed-mps is required"));
    assert!(!stdout.contains("PASS"));
    assert!(!stderr.contains("PASS"));
}

#[test]
fn identical_successful_processes_write_byte_identical_reports() {
    let root = TestDirectory::new("determinism");
    let run1 = root.path().join("run1");
    let run2 = root.path().join("run2");

    let first = run_success(&run1);
    let second = run_success(&run2);
    assert_eq!(first.status.code(), Some(0));
    assert_eq!(second.status.code(), Some(0));

    let json1 = fs::read(run1.join(JSON_REPORT)).expect("run1 JSON must be readable");
    let json2 = fs::read(run2.join(JSON_REPORT)).expect("run2 JSON must be readable");
    assert_eq!(json1, json2);
    let markdown1 = fs::read(run1.join(MARKDOWN_REPORT)).expect("run1 Markdown must be readable");
    let markdown2 = fs::read(run2.join(MARKDOWN_REPORT)).expect("run2 Markdown must be readable");
    assert_eq!(markdown1, markdown2);

    let json_text = String::from_utf8(json1).expect("JSON report must be UTF-8");
    let markdown_text = String::from_utf8(markdown1).expect("Markdown report must be UTF-8");
    for path in [&run1, &run2] {
        let path = path.to_string_lossy();
        assert!(!json_text.contains(path.as_ref()));
        assert!(!markdown_text.contains(path.as_ref()));
    }
}
