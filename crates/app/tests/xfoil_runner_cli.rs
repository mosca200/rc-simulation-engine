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
