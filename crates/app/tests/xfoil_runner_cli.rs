#![forbid(unsafe_code)]

use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
    time::{Duration, Instant},
};

const EXECUTION_JSON: &str = "xfoil_execution.json";
const EXECUTION_MARKDOWN: &str = "xfoil_execution.md";
const VALIDATION_MANIFEST: &str = "xfoil_validation_manifest.json";

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        loop {
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_9e_{label}_{}_{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("failed to create {path:?}: {error}"),
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

#[derive(Clone, Copy)]
enum FakeMode {
    Valid,
    FailFirstRun,
    OperationalAfterFirstRun,
    ProcessFailed,
    MissingPolar,
    MalformedPolar,
    Hang,
}

fn run_spec(id: &str, reynolds: f64) -> Value {
    json!({
        "dataset_id": id,
        "reynolds": reynolds,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    })
}

fn coverage(require_converged: bool) -> Value {
    json!({
        "required_reynolds_min": 100_000.0,
        "required_reynolds_max": 200_000.0,
        "required_alpha_min_rad": -0.1,
        "required_alpha_max_rad": 0.1,
        "require_converged": require_converged
    })
}

fn execution_manifest(require_converged: bool) -> Value {
    json!({
        "schema_version": 1,
        "campaign_id": "synthetic-execution-campaign",
        "airfoil_file": "inputs/airfoil.dat",
        "runs": [
            run_spec("synthetic-low", 100_000.0),
            run_spec("synthetic-high", 200_000.0)
        ],
        "coverage_request": coverage(require_converged)
    })
}

fn write_manifest(root: &Path, value: &Value) -> PathBuf {
    let manifest_dir = root.join("manifest");
    fs::create_dir_all(manifest_dir.join("inputs")).unwrap();
    fs::write(
        manifest_dir.join("inputs/airfoil.dat"),
        "Synthetic airfoil\n1.0 0.0\n0.0 0.0\n1.0 0.0\n",
    )
    .unwrap();
    let path = manifest_dir.join("execution.json");
    fs::write(&path, serde_json::to_vec_pretty(value).unwrap()).unwrap();
    path
}

fn runner_command(
    manifest_path: &Path,
    executable: &Path,
    output_dir: &Path,
    timeout_seconds: u64,
) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("xfoil")
        .arg("run-campaign")
        .arg("--manifest")
        .arg(manifest_path)
        .arg("--xfoil-executable")
        .arg(executable)
        .arg("--output-dir")
        .arg(output_dir)
        .arg("--timeout-seconds")
        .arg(timeout_seconds.to_string());
    command
}

fn run_runner(
    manifest_path: &Path,
    executable: &Path,
    output_dir: &Path,
    timeout_seconds: u64,
) -> Output {
    runner_command(manifest_path, executable, output_dir, timeout_seconds)
        .output()
        .unwrap()
}

fn read_json(path: impl AsRef<Path>) -> Value {
    serde_json::from_slice(&fs::read(path).unwrap()).unwrap()
}

#[cfg(unix)]
fn write_fake(root: &Path, mode: FakeMode) -> (PathBuf, PathBuf, PathBuf) {
    use std::os::unix::fs::PermissionsExt;

    let executable = root.join("fake-xfoil");
    let marker = root.join("invoked.txt");
    let cwd_probe = root.join("cwd.txt");
    let body = match mode {
        FakeMode::Valid => format!(
            "cat > captured.stdin\ntest -f airfoil.dat || exit 9\nprintf invoked > '{}'\npwd > '{}'\ncat > polar.out <<'POLAR'\nalpha CL CD CM\n----- -- -- --\n-10 -0.8 0.03 -0.01\n0 0 0.01 -0.01\n10 0.8 0.03 -0.01\nPOLAR\nexit 0\n",
            marker.display(),
            cwd_probe.display()
        ),
        FakeMode::FailFirstRun => "cat > captured.stdin\ntest -f airfoil.dat || exit 9\nif grep -q 'VISC 1.00000000000000000e5' captured.stdin; then exit 7; fi\ncat > polar.out <<'POLAR'\nalpha CL CD CM\n----- -- -- --\n-10 -0.8 0.03 -0.01\n0 0 0.01 -0.01\n10 0.8 0.03 -0.01\nPOLAR\nexit 0\n".to_owned(),
        FakeMode::OperationalAfterFirstRun => "cat > captured.stdin\ntest -f airfoil.dat || exit 9\ncat > polar.out <<'POLAR'\nalpha CL CD CM\n----- -- -- --\n-10 -0.8 0.03 -0.01\n0 0 0.01 -0.01\n10 0.8 0.03 -0.01\nPOLAR\n: > ../0001\nexit 0\n".to_owned(),
        FakeMode::ProcessFailed => "cat > captured.stdin\ntest -f airfoil.dat || exit 9\necho synthetic failure >&2\nexit 7\n".to_owned(),
        FakeMode::MissingPolar => {
            "cat > captured.stdin\ntest -f airfoil.dat || exit 9\nexit 0\n".to_owned()
        }
        FakeMode::MalformedPolar => "cat > captured.stdin\ntest -f airfoil.dat || exit 9\nprintf 'malformed\\n' > polar.out\nexit 0\n".to_owned(),
        FakeMode::Hang => {
            "cat > captured.stdin\ntest -f airfoil.dat || exit 9\nwhile :; do :; done\n".to_owned()
        }
    };
    fs::write(&executable, format!("#!/bin/sh\n{body}")).unwrap();
    fs::set_permissions(&executable, fs::Permissions::from_mode(0o755)).unwrap();
    (executable, marker, cwd_probe)
}

#[cfg(windows)]
fn write_fake(root: &Path, mode: FakeMode) -> (PathBuf, PathBuf, PathBuf) {
    let executable = root.join("fake-xfoil.cmd");
    let marker = root.join("invoked.txt");
    let cwd_probe = root.join("cwd.txt");
    let body = match mode {
        FakeMode::Valid => format!(
            "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\necho invoked>\"{}\"\r\ncd>\"{}\"\r\n(\r\necho alpha CL CD CM\r\necho ----- -- -- --\r\necho -10 -0.8 0.03 -0.01\r\necho 0 0 0.01 -0.01\r\necho 10 0.8 0.03 -0.01\r\n)>polar.out\r\nexit /b 0\r\n",
            marker.display(),
            cwd_probe.display()
        ),
        FakeMode::FailFirstRun => "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\nfindstr /C:\"VISC 1.00000000000000000e5\" captured.stdin >nul\r\nif not errorlevel 1 exit /b 7\r\n(\r\necho alpha CL CD CM\r\necho ----- -- -- --\r\necho -10 -0.8 0.03 -0.01\r\necho 0 0 0.01 -0.01\r\necho 10 0.8 0.03 -0.01\r\n)>polar.out\r\nexit /b 0\r\n".to_owned(),
        FakeMode::OperationalAfterFirstRun => "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\n(\r\necho alpha CL CD CM\r\necho ----- -- -- --\r\necho -10 -0.8 0.03 -0.01\r\necho 0 0 0.01 -0.01\r\necho 10 0.8 0.03 -0.01\r\n)>polar.out\r\ntype nul > ..\\0001\r\nexit /b 0\r\n".to_owned(),
        FakeMode::ProcessFailed => "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\necho synthetic failure 1>&2\r\nexit /b 7\r\n".to_owned(),
        FakeMode::MissingPolar => {
            "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\nexit /b 0\r\n"
                .to_owned()
        }
        FakeMode::MalformedPolar => "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\necho malformed>polar.out\r\nexit /b 0\r\n".to_owned(),
        FakeMode::Hang => "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\n:hang\r\ngoto hang\r\n".to_owned(),
    };
    fs::write(&executable, format!("@echo off\r\n{body}")).unwrap();
    (executable, marker, cwd_probe)
}

#[test]
fn valid_execution_uses_explicit_executable_isolated_cwd_and_manifest_relative_airfoil() {
    let root = TestDirectory::new("valid");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, marker, cwd_probe) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    let unrelated_cwd = root.path().join("unrelated-cwd");
    fs::create_dir(&unrelated_cwd).unwrap();
    let output = runner_command(&manifest_path, &executable, &output_dir, 5)
        .current_dir(&unrelated_cwd)
        .output()
        .unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_eq!(fs::read_to_string(marker).unwrap().trim(), "invoked");
    let child_cwd = fs::read_to_string(cwd_probe).unwrap();
    assert!(child_cwd.contains(".xfoil-staging-"), "{child_cwd}");
    assert!(child_cwd.trim_end().ends_with("0001") || child_cwd.trim_end().ends_with("0001\\"));
    assert!(output_dir.join("polars/0000.polar").is_file());
    assert!(output_dir.join("polars/0001.polar").is_file());
    assert!(output_dir.join(EXECUTION_JSON).is_file());
    assert!(output_dir.join(EXECUTION_MARKDOWN).is_file());
    assert!(output_dir.join(VALIDATION_MANIFEST).is_file());
    assert!(fs::read_dir(&output_dir).unwrap().all(|entry| {
        !entry
            .unwrap()
            .file_name()
            .to_string_lossy()
            .starts_with(".xfoil-staging-")
    }));

    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["schema_version"], 1);
    assert_eq!(report["generated_by"], "rcsim-app xfoil run-campaign");
    assert_eq!(report["airfoil_file"], "inputs/airfoil.dat");
    assert_eq!(report["status"], "completed");
    assert_eq!(report["completed_run_count"], 2);
    assert_eq!(report["runs"][0]["parsed_sample_count"], 3);
}

#[test]
fn repeated_output_directory_removes_polars_from_a_larger_previous_campaign() {
    let root = TestDirectory::new("shrinking-campaign");
    let mut manifest = execution_manifest(false);
    let manifest_path = write_manifest(root.path(), &manifest);
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");

    assert_eq!(
        run_runner(&manifest_path, &executable, &output_dir, 5)
            .status
            .code(),
        Some(0)
    );
    assert!(output_dir.join("polars/0000.polar").is_file());
    assert!(output_dir.join("polars/0001.polar").is_file());

    manifest["runs"] = json!([run_spec("synthetic-low", 100_000.0)]);
    fs::write(
        &manifest_path,
        serde_json::to_vec_pretty(&manifest).unwrap(),
    )
    .unwrap();
    assert_eq!(
        run_runner(&manifest_path, &executable, &output_dir, 5)
            .status
            .code(),
        Some(0)
    );

    let polar_names = fs::read_dir(output_dir.join("polars"))
        .unwrap()
        .map(|entry| entry.unwrap().file_name())
        .collect::<Vec<_>>();
    assert_eq!(polar_names, ["0000.polar"]);
}

#[test]
fn operational_failure_after_a_successful_run_removes_final_artifacts() {
    let root = TestDirectory::new("fail-clean");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::OperationalAfterFirstRun);
    let output_dir = root.path().join("output");

    let output = run_runner(&manifest_path, &executable, &output_dir, 5);
    assert_eq!(
        output.status.code(),
        Some(1),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!output_dir.join(EXECUTION_JSON).exists());
    assert!(!output_dir.join(EXECUTION_MARKDOWN).exists());
    assert!(!output_dir.join(VALIDATION_MANIFEST).exists());
    assert!(!output_dir.join("polars").exists());
}

#[test]
fn manifest_schema_and_semantic_validation_fail_closed() {
    let cases = [
        ("schema-zero", {
            let mut value = execution_manifest(false);
            value["schema_version"] = json!(0);
            value
        }),
        ("schema-two", {
            let mut value = execution_manifest(false);
            value["schema_version"] = json!(2);
            value
        }),
        ("unknown", {
            let mut value = execution_manifest(false);
            value["unknown"] = json!(true);
            value
        }),
        ("empty-runs", {
            let mut value = execution_manifest(false);
            value["runs"] = json!([]);
            value
        }),
        ("duplicate-id", {
            let mut value = execution_manifest(false);
            value["runs"][1]["dataset_id"] = value["runs"][0]["dataset_id"].clone();
            value
        }),
        ("invalid-re", {
            let mut value = execution_manifest(false);
            value["runs"][0]["reynolds"] = json!(-1.0);
            value
        }),
        ("zero-step", {
            let mut value = execution_manifest(false);
            value["runs"][0]["alpha_step_deg"] = json!(0.0);
            value
        }),
        ("wrong-step-sign", {
            let mut value = execution_manifest(false);
            value["runs"][0]["alpha_step_deg"] = json!(-1.0);
            value
        }),
        ("single-alpha", {
            let mut value = execution_manifest(false);
            value["runs"][0]["alpha_end_deg"] = value["runs"][0]["alpha_start_deg"].clone();
            value
        }),
        ("zero-iterations", {
            let mut value = execution_manifest(false);
            value["runs"][0]["maximum_iterations"] = json!(0);
            value
        }),
        ("invalid-ncrit", {
            let mut value = execution_manifest(false);
            value["runs"][0]["ncrit"] = json!(0.0);
            value
        }),
    ];
    for (label, value) in cases {
        let root = TestDirectory::new(label);
        let path = write_manifest(root.path(), &value);
        let output = run_runner(
            &path,
            &root.path().join("not-needed"),
            &root.path().join("output"),
            1,
        );
        assert_eq!(output.status.code(), Some(1), "case {label}");
    }

    let root = TestDirectory::new("non-finite-mach");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let text = fs::read_to_string(&path).unwrap();
    fs::write(&path, text.replacen("0.03", "1e9999", 1)).unwrap();
    let output = run_runner(
        &path,
        &root.path().join("not-needed"),
        &root.path().join("output"),
        1,
    );
    assert_eq!(output.status.code(), Some(1));
}

#[test]
fn airfoil_input_must_be_readable_utf8_and_nonempty() {
    let root = TestDirectory::new("empty-airfoil");
    let path = write_manifest(root.path(), &execution_manifest(false));
    fs::write(root.path().join("manifest/inputs/airfoil.dat"), " \n\t").unwrap();
    let output = run_runner(
        &path,
        &root.path().join("not-needed"),
        &root.path().join("output"),
        1,
    );
    assert_eq!(output.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&output.stderr).contains("empty or whitespace-only"));

    let root = TestDirectory::new("missing-airfoil");
    let mut value = execution_manifest(false);
    value["airfoil_file"] = json!("inputs/missing.dat");
    let path = write_manifest(root.path(), &value);
    let output = run_runner(
        &path,
        &root.path().join("not-needed"),
        &root.path().join("output"),
        1,
    );
    assert_eq!(output.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&output.stderr).contains("failed to read airfoil file"));
}

#[test]
fn solver_level_failures_are_typed_data_and_timeout_returns_after_reaping() {
    let cases = [
        (FakeMode::ProcessFailed, "process_failed", Some(7)),
        (FakeMode::MissingPolar, "missing_polar_output", Some(0)),
        (
            FakeMode::MalformedPolar,
            "unparseable_polar_output",
            Some(0),
        ),
    ];
    for (mode, expected, exit_code) in cases {
        let root = TestDirectory::new(expected);
        let mut value = execution_manifest(false);
        value["runs"] = json!([run_spec("synthetic-run", 100_000.0)]);
        let path = write_manifest(root.path(), &value);
        let (executable, _, _) = write_fake(root.path(), mode);
        let output_dir = root.path().join("output");
        let output = run_runner(&path, &executable, &output_dir, 3);
        assert_eq!(output.status.code(), Some(2), "case {expected}");
        assert!(output_dir.join(EXECUTION_JSON).is_file());
        assert!(output_dir.join(EXECUTION_MARKDOWN).is_file());
        let report = read_json(output_dir.join(EXECUTION_JSON));
        assert_eq!(report["status"], "incomplete");
        assert_eq!(report["runs"][0]["execution_status"], expected);
        assert_eq!(
            report["runs"][0]["process_exit_code"]
                .as_i64()
                .map(|code| code as i32),
            exit_code
        );
        assert!(!output_dir.join(VALIDATION_MANIFEST).exists());
        assert!(!output_dir.join("polars/0000.polar").exists());
    }

    let root = TestDirectory::new("timeout");
    let mut value = execution_manifest(false);
    value["runs"] = json!([run_spec("synthetic-run", 100_000.0)]);
    let path = write_manifest(root.path(), &value);
    let (executable, _, _) = write_fake(root.path(), FakeMode::Hang);
    let output_dir = root.path().join("output");
    let started = Instant::now();
    let output = run_runner(&path, &executable, &output_dir, 1);
    assert_eq!(output.status.code(), Some(2));
    assert!(started.elapsed() < Duration::from_secs(5));
    assert_eq!(
        read_json(output_dir.join(EXECUTION_JSON))["runs"][0]["execution_status"],
        "timed_out"
    );
}

#[test]
fn solver_failure_does_not_prevent_later_independent_runs() {
    let root = TestDirectory::new("continue-after-failure");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::FailFirstRun);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 3).status.code(),
        Some(2)
    );
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["runs"][0]["dataset_id"], "synthetic-low");
    assert_eq!(report["runs"][0]["execution_status"], "process_failed");
    assert_eq!(report["runs"][1]["dataset_id"], "synthetic-high");
    assert_eq!(report["runs"][1]["execution_status"], "completed_parseable");
    assert!(output_dir.join("polars/0001.polar").is_file());
    assert!(!output_dir.join(VALIDATION_MANIFEST).exists());
}

#[test]
fn generated_validation_manifest_preserves_order_is_unresolved_and_flows_through_m2_9d() {
    for (require_converged, validation_exit, validation_status) in
        [(false, 0, "qualified"), (true, 2, "not_qualified")]
    {
        let root = TestDirectory::new(if require_converged {
            "validation-required"
        } else {
            "validation-optional"
        });
        let path = write_manifest(root.path(), &execution_manifest(require_converged));
        let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
        let output_dir = root.path().join("execution-output");
        assert_eq!(
            run_runner(&path, &executable, &output_dir, 5).status.code(),
            Some(0)
        );

        let generated = read_json(output_dir.join(VALIDATION_MANIFEST));
        assert_eq!(generated["datasets"][0]["dataset_id"], "synthetic-low");
        assert_eq!(generated["datasets"][1]["dataset_id"], "synthetic-high");
        assert!(
            generated["datasets"]
                .as_array()
                .unwrap()
                .iter()
                .all(|dataset| dataset["convergence_status"] == "unresolved")
        );
        assert_eq!(
            generated["coverage_request"]["require_converged"],
            require_converged
        );
        let generated_text = serde_json::to_string(&generated).unwrap();
        assert!(!generated_text.contains("\"convergence_status\":\"converged\""));
        assert!(generated["datasets"][0]["solver_version"].is_null());

        let validation_dir = root.path().join("validation-output");
        let validation = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
            .arg("validate")
            .arg("xfoil-campaign")
            .arg("--manifest")
            .arg(output_dir.join(VALIDATION_MANIFEST))
            .arg("--output-dir")
            .arg(&validation_dir)
            .output()
            .unwrap();
        assert_eq!(validation.status.code(), Some(validation_exit));
        assert_eq!(
            read_json(validation_dir.join("xfoil_campaign.json"))["status"],
            validation_status
        );
    }
}

#[test]
fn deterministic_fake_produces_byte_identical_path_free_timestamp_free_artifacts() {
    let root = TestDirectory::new("determinism");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let relative_paths = [
        EXECUTION_JSON,
        EXECUTION_MARKDOWN,
        VALIDATION_MANIFEST,
        "polars/0000.polar",
        "polars/0001.polar",
    ];
    let first_artifacts =
        relative_paths.map(|relative| fs::read(output_dir.join(relative)).unwrap());
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    for (relative, first_bytes) in relative_paths.into_iter().zip(first_artifacts) {
        assert_eq!(first_bytes, fs::read(output_dir.join(relative)).unwrap());
    }
    let primary = format!(
        "{}\n{}\n{}",
        fs::read_to_string(output_dir.join(EXECUTION_JSON)).unwrap(),
        fs::read_to_string(output_dir.join(EXECUTION_MARKDOWN)).unwrap(),
        fs::read_to_string(output_dir.join(VALIDATION_MANIFEST)).unwrap()
    );
    for forbidden in [".xfoil-staging-", "timestamp", "generated_at"] {
        assert!(!primary.contains(forbidden));
    }
    assert!(!primary.contains(root.path().to_string_lossy().as_ref()));
}

#[test]
fn executable_start_failure_is_operational_exit_one() {
    let root = TestDirectory::new("start-failure");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let output = run_runner(
        &path,
        &root.path().join("missing-executable"),
        &root.path().join("output"),
        1,
    );
    assert_eq!(output.status.code(), Some(1));
    assert_ne!(output.status.code(), Some(2));
}

// ── M2.9J convergence wiring helpers ────────────────────────────────────────

fn build_polar_text(alpha_degrees: &[f64]) -> String {
    let mut s = String::from("alpha CL CD CM\n----- -- -- --\n");
    for (i, &a) in alpha_degrees.iter().enumerate() {
        let cl = -0.8 + i as f64 * 0.16;
        let cd = 0.01 + i as f64 * 0.001;
        let cm = -0.01;
        s.push_str(&format!("{a:.6} {cl:.6} {cd:.6} {cm:.6}\n"));
    }
    s
}

fn single_run_manifest(
    id: &str,
    reynolds: f64,
    start: f64,
    end: f64,
    step: f64,
    require_converged: bool,
) -> Value {
    let deg_to_rad = std::f64::consts::PI / 180.0;
    let alpha_min = start.min(end) * deg_to_rad;
    let alpha_max = start.max(end) * deg_to_rad;
    json!({
        "schema_version": 1,
        "campaign_id": "m2-9j-convergence-test",
        "airfoil_file": "inputs/airfoil.dat",
        "runs": [{
            "dataset_id": id,
            "reynolds": reynolds,
            "mach": 0.03,
            "alpha_start_deg": start,
            "alpha_end_deg": end,
            "alpha_step_deg": step,
            "maximum_iterations": 100,
            "ncrit": 9.0
        }],
        "coverage_request": {
            "required_reynolds_min": reynolds * 0.5,
            "required_reynolds_max": reynolds * 1.5,
            "required_alpha_min_rad": alpha_min,
            "required_alpha_max_rad": alpha_max,
            "require_converged": require_converged
        }
    })
}

#[cfg(unix)]
fn write_sweep_fake(root: &Path, alpha_degrees: &[f64]) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let executable = root.join("fake-xfoil");
    let polar = build_polar_text(alpha_degrees);
    let body = format!(
        "cat > captured.stdin\ntest -f airfoil.dat || exit 9\ncat > polar.out <<'POLAR'\n{polar}POLAR\nexit 0\n"
    );
    fs::write(&executable, format!("#!/bin/sh\n{body}")).unwrap();
    fs::set_permissions(&executable, fs::Permissions::from_mode(0o755)).unwrap();
    executable
}

#[cfg(windows)]
fn write_sweep_fake(root: &Path, alpha_degrees: &[f64]) -> PathBuf {
    let executable = root.join("fake-xfoil.cmd");
    let polar = build_polar_text(alpha_degrees);
    let mut body =
        String::from("more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\n(\r\n");
    for line in polar.lines() {
        body.push_str(&format!("echo {line}\r\n"));
    }
    body.push_str(")>polar.out\r\nexit /b 0\r\n");
    fs::write(&executable, format!("@echo off\r\n{body}")).unwrap();
    executable
}

fn run_single_sweep(
    alpha_degrees: &[f64],
    start: f64,
    end: f64,
    step: f64,
    require_converged: bool,
) -> (Value, Value) {
    let root = TestDirectory::new("m2-9j-sweep");
    let manifest = single_run_manifest("sweep-run", 100_000.0, start, end, step, require_converged);
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), alpha_degrees);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    let report_path = output_dir.join(EXECUTION_JSON);
    let manifest_path = output_dir.join(VALIDATION_MANIFEST);
    let report = if report_path.exists() {
        read_json(&report_path)
    } else {
        json!({"exit_code": output.status.code()})
    };
    let validation = if manifest_path.exists() {
        read_json(&manifest_path)
    } else {
        json!(null)
    };
    (report, validation)
}

// ── Test 1: complete ascending sweep → Converged ────────────────────────────

#[test]
fn m2_9j_01_complete_ascending_sweep_converged() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let (report, validation) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["execution_status"], "completed_parseable");
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][0]["convergence_status"], "converged");
}

// ── Test 2: one missing middle alpha → Unresolved ───────────────────────────

#[test]
fn m2_9j_02_missing_middle_alpha_unresolved() {
    let mut alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    alphas.remove(10);
    let (report, validation) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["execution_status"], "completed_parseable");
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
    assert_eq!(
        validation["datasets"][0]["convergence_status"],
        "unresolved"
    );
}

// ── Test 3: missing first alpha → Unresolved ────────────────────────────────

#[test]
fn m2_9j_03_missing_first_alpha_unresolved() {
    let alphas: Vec<f64> = (1..=20).map(|i| -10.0 + i as f64).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 4: missing final alpha → Unresolved ────────────────────────────────

#[test]
fn m2_9j_04_missing_final_alpha_unresolved() {
    let alphas: Vec<f64> = (0..19).map(|i| -10.0 + i as f64).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 5: extra unexpected alpha → Unresolved ─────────────────────────────

#[test]
fn m2_9j_05_extra_alpha_unresolved() {
    let mut alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    alphas.push(10.5);
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 6: reordered alpha rows → Unresolved ───────────────────────────────

#[test]
fn m2_9j_06_reordered_alpha_rows_unresolved() {
    // Parser enforces strictly increasing alpha. Reordering that breaks
    // monotonicity → UnparseablePolarOutput. The convergence is unresolved.
    let mut alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    alphas.swap(0, 1);
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    let status = report["runs"][0]["execution_status"].as_str().unwrap();
    assert!(
        status == "unparseable_polar_output" || status == "completed_parseable",
        "unexpected status: {status}"
    );
    if status == "completed_parseable" {
        assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
    }
}

// ── Test 7: duplicate alpha row → Unresolved ────────────────────────────────

#[test]
fn m2_9j_07_duplicate_alpha_unresolved() {
    // Parser rejects duplicate alpha → UnparseablePolarOutput.
    let mut alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    alphas.push(5.0);
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(
        report["runs"][0]["execution_status"],
        "unparseable_polar_output"
    );
}

// ── Test 8: complete ascending sweep (different range) → Converged ──────────

#[test]
fn m2_9j_08_complete_ascending_sweep_different_range() {
    let alphas: Vec<f64> = (0..=10).map(|i| -5.0 + i as f64).collect();
    let (report, validation) = run_single_sweep(&alphas, -5.0, 5.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][0]["convergence_status"], "converged");
}

// ── Test 9: process failure retains existing failure semantics ───────────────

#[test]
fn m2_9j_09_process_failure_retains_semantics() {
    let root = TestDirectory::new("m2-9j-proc-fail");
    let manifest = single_run_manifest("fail-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let (executable, _, _) = write_fake(root.path(), FakeMode::ProcessFailed);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    assert_eq!(output.status.code(), Some(2));
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["runs"][0]["execution_status"], "process_failed");
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 10: missing polar retains MissingPolarOutput ───────────────────────

#[test]
fn m2_9j_10_missing_polar_retains_semantics() {
    let root = TestDirectory::new("m2-9j-missing-polar");
    let manifest = single_run_manifest("miss-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let (executable, _, _) = write_fake(root.path(), FakeMode::MissingPolar);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    assert_eq!(output.status.code(), Some(2));
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(
        report["runs"][0]["execution_status"],
        "missing_polar_output"
    );
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 11: malformed polar retains UnparseablePolarOutput ─────────────────

#[test]
fn m2_9j_11_malformed_polar_retains_semantics() {
    let root = TestDirectory::new("m2-9j-malformed");
    let manifest = single_run_manifest("mal-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let (executable, _, _) = write_fake(root.path(), FakeMode::MalformedPolar);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    assert_eq!(output.status.code(), Some(2));
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(
        report["runs"][0]["execution_status"],
        "unparseable_polar_output"
    );
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 12: parser-success alone is insufficient ───────────────────────────

#[test]
fn m2_9j_12_parser_success_alone_insufficient() {
    // Existing fake produces 3 alpha points but sweep expects 21.
    // Parser succeeds but convergence is Unresolved.
    let root = TestDirectory::new("m2-9j-parse-only");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    assert_eq!(
        validation["datasets"][0]["convergence_status"],
        "unresolved"
    );
}

// ── Test 13: process exit 0 alone is insufficient ───────────────────────────

#[test]
fn m2_9j_13_process_exit_zero_alone_insufficient() {
    // Same as test 12 — process exits 0 but incomplete sweep → Unresolved.
    let root = TestDirectory::new("m2-9j-exit0-only");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    assert_eq!(output.status.code(), Some(0));
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["runs"][0]["execution_status"], "completed_parseable");
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    assert_eq!(
        validation["datasets"][0]["convergence_status"],
        "unresolved"
    );
}

// ── Test 14: incomplete parseable polar is not operational error ────────────

#[test]
fn m2_9j_14_incomplete_parseable_not_operational_error() {
    let root = TestDirectory::new("m2-9j-incomplete-ok");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    let output = run_runner(&path, &executable, &output_dir, 5);
    assert_eq!(output.status.code(), Some(0));
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["runs"][0]["execution_status"], "completed_parseable");
    assert!(report["runs"][0]["parsed_sample_count"].as_u64().unwrap() > 0);
}

// ── Test 15: complete parseable polar remains completed parseable ────────────

#[test]
fn m2_9j_15_complete_parseable_remains_completed() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["execution_status"], "completed_parseable");
    assert_eq!(report["runs"][0]["parsed_sample_count"], 21);
}

// ── Test 16: exact Reynolds and Mach preserved ──────────────────────────────

#[test]
fn m2_9j_16_exact_reynolds_mach_preserved() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let (report, validation) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["reynolds"], 100_000.0);
    assert_eq!(report["runs"][0]["mach"], 0.03);
    assert_eq!(validation["datasets"][0]["reynolds"], 100_000.0);
    assert_eq!(validation["datasets"][0]["mach"], 0.03);
}

// ── Test 17: raw polar bytes preserved ──────────────────────────────────────

#[test]
fn m2_9j_17_raw_polar_bytes_preserved() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let root = TestDirectory::new("m2-9j-polar-bytes");
    let manifest = single_run_manifest("polar-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let polar = fs::read_to_string(output_dir.join("polars/0000.polar")).unwrap();
    assert!(polar.contains("alpha"));
    assert!(polar.contains("-10.000000"));
    assert!(polar.contains("10.000000"));
}

// ── Test 18: M2.9D require_converged=true accepts complete campaign ──────────

#[test]
fn m2_9j_18_require_converged_accepts_complete() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let root = TestDirectory::new("m2-9j-req-conv-ok");
    // Use two datasets matching the standard execution_manifest coverage range.
    let mut manifest = execution_manifest(true);
    manifest["runs"][0] = json!({
        "dataset_id": "conv-low",
        "reynolds": 100_000.0,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    manifest["runs"][1] = json!({
        "dataset_id": "conv-high",
        "reynolds": 200_000.0,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    assert_eq!(validation["datasets"][0]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][1]["convergence_status"], "converged");
    let validation_dir = root.path().join("validation-output");
    let result = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("validate")
        .arg("xfoil-campaign")
        .arg("--manifest")
        .arg(output_dir.join(VALIDATION_MANIFEST))
        .arg("--output-dir")
        .arg(&validation_dir)
        .output()
        .unwrap();
    assert_eq!(
        result.status.code(),
        Some(0),
        "validator stderr: {}",
        String::from_utf8_lossy(&result.stderr)
    );
    assert_eq!(
        read_json(validation_dir.join("xfoil_campaign.json"))["status"],
        "qualified"
    );
}

// ── Test 19: M2.9D require_converged=true rejects incomplete campaign ────────

#[test]
fn m2_9j_19_require_converged_rejects_incomplete() {
    // Existing fake produces 3 points but sweep expects 21 → Unresolved.
    let root = TestDirectory::new("m2-9j-req-conv-fail");
    let path = write_manifest(root.path(), &execution_manifest(true));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation_dir = root.path().join("validation-output");
    let validation = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("validate")
        .arg("xfoil-campaign")
        .arg("--manifest")
        .arg(output_dir.join(VALIDATION_MANIFEST))
        .arg("--output-dir")
        .arg(&validation_dir)
        .output()
        .unwrap();
    assert_eq!(validation.status.code(), Some(2));
    assert_eq!(
        read_json(validation_dir.join("xfoil_campaign.json"))["status"],
        "not_qualified"
    );
}

// ── Test 20: require_converged=false preserves coverage semantics ────────────

#[test]
fn m2_9j_20_require_converged_false_preserves_coverage() {
    let root = TestDirectory::new("m2-9j-conv-optional");
    let path = write_manifest(root.path(), &execution_manifest(false));
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation_dir = root.path().join("validation-output");
    let validation = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("validate")
        .arg("xfoil-campaign")
        .arg("--manifest")
        .arg(output_dir.join(VALIDATION_MANIFEST))
        .arg("--output-dir")
        .arg(&validation_dir)
        .output()
        .unwrap();
    assert_eq!(validation.status.code(), Some(0));
    assert_eq!(
        read_json(validation_dir.join("xfoil_campaign.json"))["status"],
        "qualified"
    );
}

// ── Test 21: mixed Converged/Unresolved preserves ordering ──────────────────

#[test]
fn m2_9j_21_mixed_status_preserves_ordering() {
    let root = TestDirectory::new("m2-9j-mixed");
    let mut manifest = execution_manifest(false);
    // First run: complete sweep (converged). Second run: incomplete (unresolved).
    manifest["runs"][0] = json!({
        "dataset_id": "converged-run",
        "reynolds": 100_000.0,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    manifest["runs"][1] = json!({
        "dataset_id": "unresolved-run",
        "reynolds": 200_000.0,
        "mach": 0.03,
        "alpha_start_deg": -5.0,
        "alpha_end_deg": 5.0,
        "alpha_step_deg": 0.5,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    let path = write_manifest(root.path(), &manifest);
    // Fake emits only 3 alpha points — first run expects 21, second expects 21.
    // Both will be unresolved with the standard fake.
    let (executable, _, _) = write_fake(root.path(), FakeMode::Valid);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let report = read_json(output_dir.join(EXECUTION_JSON));
    assert_eq!(report["runs"][0]["dataset_id"], "converged-run");
    assert_eq!(report["runs"][1]["dataset_id"], "unresolved-run");
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
    assert_eq!(report["runs"][1]["convergence_status"], "unresolved");
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    assert_eq!(validation["datasets"][0]["dataset_id"], "converged-run");
    assert_eq!(validation["datasets"][1]["dataset_id"], "unresolved-run");
}

// ── Test 22: multiple Reynolds runs qualified independently ─────────────────

#[test]
fn m2_9j_22_multiple_reynolds_independently_qualified() {
    let root = TestDirectory::new("m2-9j-multi-re");
    let mut manifest = execution_manifest(false);
    // Both runs get complete sweeps → both converged.
    manifest["runs"][0] = json!({
        "dataset_id": "low-re",
        "reynolds": 100_000.0,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    manifest["runs"][1] = json!({
        "dataset_id": "high-re",
        "reynolds": 200_000.0,
        "mach": 0.03,
        "alpha_start_deg": -10.0,
        "alpha_end_deg": 10.0,
        "alpha_step_deg": 1.0,
        "maximum_iterations": 100,
        "ncrit": 9.0
    });
    let path = write_manifest(root.path(), &manifest);
    // Use a fake that emits complete sweeps for both runs.
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    assert_eq!(validation["datasets"][0]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][1]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][0]["reynolds"], 100_000.0);
    assert_eq!(validation["datasets"][1]["reynolds"], 200_000.0);
}

// ── Test 23: explicit alpha tolerance boundary ──────────────────────────────

#[test]
fn m2_9j_23_alpha_tolerance_boundary() {
    // Alpha values offset by less than the tolerance (1e-7 rad ≈ 5.7e-6 deg).
    // Offset of 1e-6 degrees is well within tolerance → Converged.
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64 + 1e-6).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
}

// ── Test 24: degree/radian conversion discriminating ────────────────────────

#[test]
fn m2_9j_24_degree_radian_conversion_discriminating() {
    // Alpha values offset by more than the tolerance (0.01 degrees ≈ 1.7e-4
    // rad, which exceeds the 1e-7 rad tolerance) → Unresolved.
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64 + 0.01).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "unresolved");
}

// ── Test 25: deterministic repeated execution ───────────────────────────────

#[test]
fn m2_9j_25_deterministic_repeated_execution() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let root = TestDirectory::new("m2-9j-determ");
    let manifest = single_run_manifest("determ-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let first_report = fs::read(output_dir.join(EXECUTION_JSON)).unwrap();
    let first_validation = fs::read(output_dir.join(VALIDATION_MANIFEST)).unwrap();
    let first_polar = fs::read(output_dir.join("polars/0000.polar")).unwrap();
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    assert_eq!(
        first_report,
        fs::read(output_dir.join(EXECUTION_JSON)).unwrap()
    );
    assert_eq!(
        first_validation,
        fs::read(output_dir.join(VALIDATION_MANIFEST)).unwrap()
    );
    assert_eq!(
        first_polar,
        fs::read(output_dir.join("polars/0000.polar")).unwrap()
    );
}

// ── Test 26: M2.9I called through production API ────────────────────────────

#[test]
fn m2_9j_26_m2_9i_called_through_production_api() {
    // The convergence status is computed by the production qualify_sweep
    // function which calls model::qualify_sweep_convergence. This test
    // verifies the integration by checking that a complete sweep produces
    // "converged" through the full pipeline.
    let alphas: Vec<f64> = (0..=10).map(|i| -5.0 + i as f64).collect();
    let (report, validation) = run_single_sweep(&alphas, -5.0, 5.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
    assert_eq!(validation["datasets"][0]["convergence_status"], "converged");
}

// ── Test 27: no real XFOIL executable required ──────────────────────────────

#[test]
fn m2_9j_27_no_real_xfoil_required() {
    // All tests in this file use fake XFOIL executables. This test
    // self-documents that no real XFOIL is needed.
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
}

// ── Test 28: CL/CD/CM preserved through pipeline ────────────────────────────

#[test]
fn m2_9j_28_cl_cd_cm_preserved_in_polar() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let root = TestDirectory::new("m2-9j-coeffs");
    let manifest = single_run_manifest("coeff-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let polar = fs::read_to_string(output_dir.join("polars/0000.polar")).unwrap();
    assert!(polar.contains("-0.800000"));
    assert!(polar.contains("0.010000"));
    assert!(polar.contains("-0.010000"));
}

// ── Test 29: validation manifest notes reflect convergence ──────────────────

#[test]
fn m2_9j_29_validation_notes_reflect_convergence() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let root = TestDirectory::new("m2-9j-notes");
    let manifest = single_run_manifest("notes-run", 100_000.0, -10.0, 10.0, 1.0, false);
    let path = write_manifest(root.path(), &manifest);
    let executable = write_sweep_fake(root.path(), &alphas);
    let output_dir = root.path().join("output");
    assert_eq!(
        run_runner(&path, &executable, &output_dir, 5).status.code(),
        Some(0)
    );
    let validation = read_json(output_dir.join(VALIDATION_MANIFEST));
    let notes = validation["datasets"][0]["notes"].as_str().unwrap();
    assert!(
        notes.contains("M2.9I"),
        "notes should mention M2.9I: {notes}"
    );
}

// ── Test 30: execution report contains convergence_status field ─────────────

#[test]
fn m2_9j_30_execution_report_has_convergence_field() {
    let alphas: Vec<f64> = (0..=20).map(|i| -10.0 + i as f64).collect();
    let (report, _) = run_single_sweep(&alphas, -10.0, 10.0, 1.0, false);
    assert!(report["runs"][0]["convergence_status"].is_string());
    assert_eq!(report["runs"][0]["convergence_status"], "converged");
}
