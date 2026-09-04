#![forbid(unsafe_code)]
//! End-to-end CLI tests for M2.9G — deterministic XFOIL evidence bundle
//! promotion. Tests construct synthetic M2.9E execution output (either via
//! the existing fake XFOIL runner or by directly writing the schema) and
//! then exercise `rcsim-app xfoil build-evidence-bundle`.

use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};

const EXECUTION_JSON: &str = "xfoil_execution.json";
const VALIDATION_MANIFEST: &str = "xfoil_validation_manifest.json";
const BUNDLE_MANIFEST: &str = "xfoil_evidence_bundle.json";
const POLAR_DATASETS: &str = "polar_datasets.json";
const POLAR_DIRECTORY: &str = "polars";

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        loop {
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_9g_{label}_{}_{}",
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

#[cfg(unix)]
fn write_fake(root: &Path) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;

    let executable = root.join("fake-xfoil");
    let body = "cat > captured.stdin\ntest -f airfoil.dat || exit 9\ncat > polar.out <<'POLAR'\nalpha CL CD CM\n----- -- -- --\n-10 -0.8 0.03 -0.01\n0 0 0.01 -0.01\n10 0.8 0.03 -0.01\nPOLAR\nexit 0\n";
    fs::write(&executable, format!("#!/bin/sh\n{body}")).unwrap();
    fs::set_permissions(&executable, fs::Permissions::from_mode(0o755)).unwrap();
    executable
}

#[cfg(windows)]
fn write_fake(root: &Path) -> PathBuf {
    const FAKE_BODY: &str = "more > captured.stdin\r\nif not exist airfoil.dat exit /b 9\r\n(\r\necho alpha CL CD CM\r\necho ----- -- -- --\r\necho -10 -0.8 0.03 -0.01\r\necho 0 0 0.01 -0.01\r\necho 10 0.8 0.03 -0.01\r\n)>polar.out\r\nexit /b 0\r\n";

    let executable = root.join("fake-xfoil.cmd");
    fs::write(&executable, format!("@echo off\r\n{FAKE_BODY}")).unwrap();
    executable
}

fn run_runner(manifest_path: &Path, executable: &Path, output_dir: &Path) -> Output {
    Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("xfoil")
        .arg("run-campaign")
        .arg("--manifest")
        .arg(manifest_path)
        .arg("--xfoil-executable")
        .arg(executable)
        .arg("--output-dir")
        .arg(output_dir)
        .arg("--timeout-seconds")
        .arg("5")
        .output()
        .unwrap()
}

fn read_json(path: PathBuf) -> Value {
    serde_json::from_slice(&fs::read(&path).unwrap()).unwrap()
}

fn sha256_hex(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(digest.len() * 2);
    for byte in digest {
        use std::fmt::Write;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

fn bundle_command(execution_dir: &Path, output_dir: &Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("xfoil")
        .arg("build-evidence-bundle")
        .arg("--execution-dir")
        .arg(execution_dir)
        .arg("--output-dir")
        .arg(output_dir);
    command
}

#[test]
fn completed_single_dataset_promotion_succeeds() {
    let root = TestDirectory::new("single");
    let mut value = execution_manifest(false);
    value["runs"] = json!([run_spec("synthetic-only", 100_000.0)]);
    let manifest_path = write_manifest(root.path(), &value);
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(bundle_dir.join(BUNDLE_MANIFEST).is_file());
    assert!(bundle_dir.join(POLAR_DATASETS).is_file());
    assert!(
        bundle_dir
            .join(POLAR_DIRECTORY)
            .join("0000.polar")
            .is_file()
    );
    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    assert_eq!(bundle["schema_version"], 1);
    assert_eq!(
        bundle["generated_by"],
        "rcsim-app xfoil build-evidence-bundle"
    );
    assert_eq!(bundle["campaign_id"], "synthetic-execution-campaign");
    assert_eq!(bundle["dataset_count"], 1);
    assert_eq!(bundle["datasets"][0]["index"], 0);
    assert_eq!(bundle["datasets"][0]["dataset_id"], "synthetic-only");
    assert_eq!(bundle["datasets"][0]["convergence_status"], "unresolved");
    assert_eq!(bundle["datasets"][0]["polar_file"], "polars/0000.polar");
    assert_eq!(bundle["datasets"][0]["polar_dataset_index"], 0);
    assert_eq!(
        bundle["coverage_request"]["require_converged"], false,
        "coverage request must be preserved exactly"
    );
}

#[test]
fn multi_dataset_promotion_preserves_exact_order() {
    let root = TestDirectory::new("multi");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    assert_eq!(bundle["dataset_count"], 2);
    assert_eq!(bundle["datasets"][0]["index"], 0);
    assert_eq!(bundle["datasets"][0]["dataset_id"], "synthetic-low");
    assert_eq!(bundle["datasets"][0]["reynolds"], 100_000.0);
    assert_eq!(bundle["datasets"][1]["index"], 1);
    assert_eq!(bundle["datasets"][1]["dataset_id"], "synthetic-high");
    assert_eq!(bundle["datasets"][1]["reynolds"], 200_000.0);
}

#[test]
fn polar_datasets_json_is_canonical_evidence() {
    let root = TestDirectory::new("canonical");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    let datasets_text = fs::read_to_string(bundle_dir.join(POLAR_DATASETS)).unwrap();
    let datasets: Value = serde_json::from_str(&datasets_text).unwrap();
    assert!(datasets.is_array());
    assert_eq!(datasets.as_array().unwrap().len(), 2);
    let first = &datasets[0];
    assert_eq!(first["evidence_class"], "generated_solver");
    assert_eq!(first["method"]["convergence_status"], "unresolved");
    for required in [
        "id",
        "evidence_class",
        "flow_conditions",
        "transition",
        "method",
        "source_ids",
        "samples",
    ] {
        assert!(
            first.get(required).is_some(),
            "missing {required} in dataset element"
        );
    }
}

#[test]
fn repeated_build_into_two_dirs_is_byte_identical() {
    let root = TestDirectory::new("deterministic-dirs");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_a = root.path().join("bundle-a");
    let bundle_b = root.path().join("bundle-b");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_a)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    assert_eq!(
        bundle_command(&exec_dir, &bundle_b)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let relative_paths = [
        BUNDLE_MANIFEST,
        POLAR_DATASETS,
        "polars/0000.polar",
        "polars/0001.polar",
    ];
    for relative in relative_paths {
        let bytes_a = fs::read(bundle_a.join(relative)).unwrap();
        let bytes_b = fs::read(bundle_b.join(relative)).unwrap();
        assert_eq!(bytes_a, bytes_b, "divergence in {relative}");
    }
}

#[test]
fn repeated_build_into_same_dir_is_byte_identical_and_stale_free() {
    let root = TestDirectory::new("same-dir");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    let first_bundle_bytes = fs::read(bundle_dir.join(BUNDLE_MANIFEST)).unwrap();
    let first_polar_datasets_bytes = fs::read(bundle_dir.join(POLAR_DATASETS)).unwrap();
    let first_polars: std::collections::HashMap<String, Vec<u8>> =
        fs::read_dir(bundle_dir.join(POLAR_DIRECTORY))
            .unwrap()
            .map(|entry| {
                let entry = entry.unwrap();
                (
                    entry.file_name().to_string_lossy().into_owned(),
                    fs::read(entry.path()).unwrap(),
                )
            })
            .collect();

    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    assert_eq!(
        fs::read(bundle_dir.join(BUNDLE_MANIFEST)).unwrap(),
        first_bundle_bytes
    );
    assert_eq!(
        fs::read(bundle_dir.join(POLAR_DATASETS)).unwrap(),
        first_polar_datasets_bytes
    );
    let second_polars: std::collections::HashMap<String, Vec<u8>> =
        fs::read_dir(bundle_dir.join(POLAR_DIRECTORY))
            .unwrap()
            .map(|entry| {
                let entry = entry.unwrap();
                (
                    entry.file_name().to_string_lossy().into_owned(),
                    fs::read(entry.path()).unwrap(),
                )
            })
            .collect();
    assert_eq!(second_polars.len(), first_polars.len());
    for (name, bytes) in &first_polars {
        assert_eq!(second_polars.get(name), Some(bytes), "{name}");
    }
}

#[test]
fn smaller_bundle_removes_stale_polars_from_larger_previous_bundle() {
    let root = TestDirectory::new("shrinking");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    assert!(bundle_dir.join("polars/0001.polar").exists());
    assert!(bundle_dir.join("polars/0000.polar").exists());

    let mut new_manifest = execution_manifest(false);
    new_manifest["runs"] = json!([run_spec("synthetic-low", 100_000.0)]);
    let new_manifest_path = write_manifest(root.path(), &new_manifest);
    let new_exec_dir = root.path().join("execution-small");
    assert_eq!(
        run_runner(&new_manifest_path, &executable, &new_exec_dir)
            .status
            .code(),
        Some(0)
    );
    assert_eq!(
        bundle_command(&new_exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let mut polar_names: Vec<String> = fs::read_dir(bundle_dir.join(POLAR_DIRECTORY))
        .unwrap()
        .map(|entry| entry.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    polar_names.sort();
    assert_eq!(polar_names, ["0000.polar"]);
}

#[test]
fn incomplete_execution_is_rejected_with_exit_two() {
    let root = TestDirectory::new("incomplete");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // Produce a structurally valid execution report whose status is
    // explicitly `incomplete` according to the M2.9E schema. The validation
    // manifest is left in place so M2.9G sees a complete pair of artifacts
    // and must reject the evidence on semantic (not operational) grounds.
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["status"] = json!("incomplete");
    exec["completed_run_count"] = json!(1);
    exec["runs"][1]["execution_status"] = json!("process_failed");
    exec["runs"][1]["process_exit_code"] = json!(7);
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
    assert!(!bundle_dir.join(POLAR_DATASETS).exists());
    assert!(!bundle_dir.join(POLAR_DIRECTORY).exists());
}

#[test]
fn completed_run_count_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("completed-count-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["completed_run_count"] = json!(1);
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
}

#[test]
fn non_completed_run_status_is_rejected_with_exit_two() {
    let root = TestDirectory::new("non-completed");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["runs"][1]["execution_status"] = json!("process_failed");
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn missing_validation_manifest_is_rejected() {
    let root = TestDirectory::new("missing-validation");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    fs::remove_file(exec_dir.join(VALIDATION_MANIFEST)).unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(output.status.code(), Some(1));
}

#[test]
fn campaign_id_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("campaign-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["campaign_id"] = json!("different-campaign");
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn dataset_count_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("count-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"] = json!([val["datasets"][0].clone()]);
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn dataset_id_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("id-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"][1]["dataset_id"] = json!("synthetic-other");
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn reynolds_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("re-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"][1]["reynolds"] = json!(150_000.0);
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn mach_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("mach-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"][1]["mach"] = json!(0.05);
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn missing_polar_is_rejected_with_exit_two() {
    let root = TestDirectory::new("missing-polar");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    fs::remove_file(exec_dir.join("polars/0001.polar")).unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn malformed_polar_is_rejected_with_exit_two() {
    let root = TestDirectory::new("malformed-polar");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    fs::write(exec_dir.join("polars/0000.polar"), b"this is not a polar").unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn duplicate_dataset_id_is_rejected_with_exit_two() {
    let root = TestDirectory::new("duplicate-id");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // M2.9E rejects duplicate dataset IDs at the campaign manifest level.
    // Tamper a structurally valid completed execution artifact to introduce
    // a duplicate dataset_id at the level M2.9G is responsible for
    // validating. The validation manifest is left in place so M2.9G sees a
    // complete pair of artifacts and must reject on semantic grounds.
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["runs"][1]["dataset_id"] = json!("synthetic-low");
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"][1]["dataset_id"] = json!("synthetic-low");
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn unsupported_schema_is_rejected_with_exit_two() {
    let root = TestDirectory::new("schema-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["schema_version"] = json!(2);
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn absolute_polar_reference_is_rejected_with_exit_two() {
    let root = TestDirectory::new("absolute-polar");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );
    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    val["datasets"][0]["polar_file"] = json!("/etc/passwd");
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn operational_failure_after_partial_work_leaves_no_final_bundle() {
    let root = TestDirectory::new("partial-fail");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    fs::write(exec_dir.join("polars/0001.polar"), b"not a polar").unwrap();

    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
    assert!(!bundle_dir.join(POLAR_DATASETS).exists());
    assert!(!bundle_dir.join(POLAR_DIRECTORY).exists());
}

#[test]
fn stale_unrelated_file_outside_owned_output_is_not_deleted() {
    let root = TestDirectory::new("unrelated");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    let unrelated = bundle_dir.join("unrelated.txt");
    fs::create_dir_all(&bundle_dir).unwrap();
    fs::write(&unrelated, b"keep me").unwrap();

    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );
    assert_eq!(
        fs::read(&unrelated).unwrap(),
        b"keep me",
        "unrelated file must survive promotion"
    );
}

#[test]
fn no_absolute_input_paths_appear_in_bundle_json() {
    let root = TestDirectory::new("no-abs-paths");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let bundle_text = fs::read_to_string(bundle_dir.join(BUNDLE_MANIFEST)).unwrap();
    let root_str = root.path().to_string_lossy().into_owned();
    for forbidden in [root_str.as_str(), "C:\\", "/tmp/"] {
        assert!(
            !bundle_text.contains(forbidden),
            "bundle manifest must not contain {forbidden}"
        );
    }
}

#[test]
fn no_timestamps_or_hostnames_appear_in_bundle_json() {
    let root = TestDirectory::new("no-meta");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let bundle_text = fs::read_to_string(bundle_dir.join(BUNDLE_MANIFEST)).unwrap();
    let lower = bundle_text.to_ascii_lowercase();
    for forbidden in ["timestamp", "generated_at", "hostname"] {
        assert!(
            !lower.contains(forbidden),
            "bundle manifest must not contain {forbidden}"
        );
    }
}

#[test]
fn no_xfoil_executable_path_appears_in_bundle_json() {
    let root = TestDirectory::new("no-xfoil-path");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let bundle_text = fs::read_to_string(bundle_dir.join(BUNDLE_MANIFEST)).unwrap();
    assert!(!bundle_text.contains("fake-xfoil"));
    let exec_path_str = executable.to_string_lossy().into_owned();
    assert!(!bundle_text.contains(&exec_path_str));
}

#[test]
fn raw_polars_are_copied_byte_identically_and_sha256_locked() {
    let root = TestDirectory::new("raw-byte");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    for dataset in bundle["datasets"].as_array().unwrap() {
        let polar_name = dataset["polar_file"].as_str().unwrap().to_owned();
        let expected_sha = dataset["polar_sha256"].as_str().unwrap();
        let bundled_bytes = fs::read(bundle_dir.join(&polar_name)).unwrap();
        let actual_sha = sha256_hex(&bundled_bytes);
        assert_eq!(actual_sha, expected_sha, "{polar_name}");
    }
}

#[test]
fn polar_datasets_hash_matches_bundle_manifest() {
    let root = TestDirectory::new("datasets-hash");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");
    assert_eq!(
        bundle_command(&exec_dir, &bundle_dir)
            .output()
            .unwrap()
            .status
            .code(),
        Some(0)
    );

    let polar_datasets_bytes = fs::read(bundle_dir.join(POLAR_DATASETS)).unwrap();
    let expected_hash = sha256_hex(&polar_datasets_bytes);
    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    assert_eq!(
        bundle["polar_datasets_sha256"].as_str().unwrap(),
        expected_hash
    );
}

/// Helper: tamper convergence_status in both execution report and validation manifest.
fn set_convergence_status(exec_dir: &Path, status: &str) {
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    for run in exec["runs"].as_array_mut().unwrap() {
        run["convergence_status"] = json!(status);
    }
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let val_text = fs::read_to_string(exec_dir.join(VALIDATION_MANIFEST)).unwrap();
    let mut val: Value = serde_json::from_str(&val_text).unwrap();
    for dataset in val["datasets"].as_array_mut().unwrap() {
        dataset["convergence_status"] = json!(status);
    }
    fs::write(
        exec_dir.join(VALIDATION_MANIFEST),
        serde_json::to_vec_pretty(&val).unwrap(),
    )
    .unwrap();
}

#[test]
fn converged_artifact_promotes_as_converged() {
    let root = TestDirectory::new("converged");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // Tamper to "converged" in both execution and validation artifacts
    set_convergence_status(&exec_dir, "converged");

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    // Bundle manifest must reflect converged status
    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    for dataset in bundle["datasets"].as_array().unwrap() {
        assert_eq!(dataset["convergence_status"].as_str().unwrap(), "converged");
    }

    // polar_datasets.json must also reflect converged status
    let datasets_text = fs::read_to_string(bundle_dir.join(POLAR_DATASETS)).unwrap();
    let datasets: Value = serde_json::from_str(&datasets_text).unwrap();
    for dataset in datasets.as_array().unwrap() {
        assert_eq!(
            dataset["method"]["convergence_status"].as_str().unwrap(),
            "converged"
        );
    }
}

#[test]
fn unresolved_artifact_remains_unresolved() {
    let root = TestDirectory::new("unresolved-kept");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // The fake runner produces "unresolved" by default (only 3 of 21 alpha points)
    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    for dataset in bundle["datasets"].as_array().unwrap() {
        assert_eq!(
            dataset["convergence_status"].as_str().unwrap(),
            "unresolved"
        );
    }

    let datasets_text = fs::read_to_string(bundle_dir.join(POLAR_DATASETS)).unwrap();
    let datasets: Value = serde_json::from_str(&datasets_text).unwrap();
    for dataset in datasets.as_array().unwrap() {
        assert_eq!(
            dataset["method"]["convergence_status"].as_str().unwrap(),
            "unresolved"
        );
    }
}

#[test]
fn convergence_mismatch_between_execution_and_validation_is_exit_two() {
    let root = TestDirectory::new("conv-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // Set execution to "converged" but leave validation as "unresolved"
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    for run in exec["runs"].as_array_mut().unwrap() {
        run["convergence_status"] = json!("converged");
    }
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();
    // Validation manifest keeps "unresolved"

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
}

#[test]
fn run_index_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("index-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // Tamper run index to be wrong
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["runs"][1]["index"] = json!(99);
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
}

#[test]
fn run_polar_file_mismatch_is_rejected_with_exit_two() {
    let root = TestDirectory::new("polar-file-mismatch");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    // Tamper polar_file to non-canonical path
    let exec_text = fs::read_to_string(exec_dir.join(EXECUTION_JSON)).unwrap();
    let mut exec: Value = serde_json::from_str(&exec_text).unwrap();
    exec["runs"][0]["polar_file"] = json!("polars/wrong.polar");
    fs::write(
        exec_dir.join(EXECUTION_JSON),
        serde_json::to_vec_pretty(&exec).unwrap(),
    )
    .unwrap();

    let bundle_dir = root.path().join("bundle");
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(2),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(!bundle_dir.join(BUNDLE_MANIFEST).exists());
}

#[test]
fn stale_owned_staging_does_not_contaminate_next_bundle() {
    let root = TestDirectory::new("stale-staging");
    let manifest_path = write_manifest(root.path(), &execution_manifest(false));
    let executable = write_fake(root.path());
    let exec_dir = root.path().join("execution");
    assert_eq!(
        run_runner(&manifest_path, &executable, &exec_dir)
            .status
            .code(),
        Some(0)
    );

    let bundle_dir = root.path().join("bundle");

    // Create a stale staging directory with junk content
    let stale_staging = bundle_dir.join(".xfoil-bundle-staging-work");
    fs::create_dir_all(stale_staging.join("polars")).unwrap();
    fs::write(stale_staging.join("polars/junk.polar"), b"stale junk").unwrap();
    fs::write(
        stale_staging.join("xfoil_evidence_bundle.json"),
        b"stale bundle",
    )
    .unwrap();

    // Now run a successful promotion — stale staging must be cleaned first
    let output = bundle_command(&exec_dir, &bundle_dir).output().unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );

    // The stale staging directory must not exist after successful promotion
    assert!(!stale_staging.exists(), "stale staging must be cleaned");

    // The final bundle must be valid, not contaminated
    let bundle = read_json(bundle_dir.join(BUNDLE_MANIFEST));
    assert_eq!(bundle["dataset_count"], 2);
    assert!(bundle_dir.join("polars/0000.polar").is_file());
    assert!(bundle_dir.join("polars/0001.polar").is_file());
}
