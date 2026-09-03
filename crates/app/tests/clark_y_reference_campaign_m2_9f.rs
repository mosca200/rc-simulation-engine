//! M2.9F — Real production-parser proof for the committed Clark Y campaign.
//!
//! This test feeds the exact committed `reference/xfoil/clark_y/campaign.json`
//! through the real `rcsim-app xfoil run-campaign` CLI path using a
//! cross-platform fake XFOIL executable. It proves:
//!
//! - The production manifest parser accepts the committed campaign.
//! - Relative `clarky.dat` resolves from the manifest directory.
//! - All 6 ordered runs execute.
//! - Exit code is 0 (all runs completed parseable).
//! - Generated execution report has run_count == 6.
//! - Generated validation manifest exists.
//! - Dataset/Reynolds order is preserved.
//! - CWD independence (child process CWD is unrelated to manifest dir).

#![forbid(unsafe_code)]

use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
    sync::atomic::{AtomicUsize, Ordering},
};

const EXECUTION_JSON: &str = "xfoil_execution.json";
const VALIDATION_MANIFEST: &str = "xfoil_validation_manifest.json";

const EXPECTED_REYNOLDS: [f64; 6] = [
    100_000.0, 150_000.0, 200_000.0, 300_000.0, 500_000.0, 750_000.0,
];

const EXPECTED_DATASET_IDS: [&str; 6] = [
    "clark-y-re-100000",
    "clark-y-re-150000",
    "clark-y-re-200000",
    "clark-y-re-300000",
    "clark-y-re-500000",
    "clark-y-re-750000",
];

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        loop {
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_9f_{label}_{}_{}",
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

fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn committed_campaign_path() -> PathBuf {
    project_root()
        .join("reference")
        .join("xfoil")
        .join("clark_y")
        .join("campaign.json")
}

fn read_json(path: impl AsRef<Path>) -> Value {
    serde_json::from_slice(&fs::read(path).unwrap()).unwrap()
}

#[cfg(unix)]
fn write_fake_xfoil(root: &Path) -> PathBuf {
    use std::os::unix::fs::PermissionsExt;

    let executable = root.join("fake-xfoil");
    let script = "\
#!/bin/sh
cat > captured.stdin
test -f airfoil.dat || exit 9
cat > polar.out <<'POLAR'
alpha CL CD CM
----- -- -- --
-12 -1.0 0.04 -0.01
0 0 0.01 -0.01
18 1.2 0.05 -0.01
POLAR
exit 0
";
    fs::write(&executable, script).unwrap();
    fs::set_permissions(&executable, fs::Permissions::from_mode(0o755)).unwrap();
    executable
}

#[cfg(windows)]
fn write_fake_xfoil(root: &Path) -> PathBuf {
    let executable = root.join("fake-xfoil.cmd");
    let script = "\
@echo off
more > captured.stdin
if not exist airfoil.dat exit /b 9
(
echo alpha CL CD CM
echo ----- -- -- --
echo -12 -1.0 0.04 -0.01
echo 0 0 0.01 -0.01
echo 18 1.2 0.05 -0.01
)>polar.out
exit /b 0
";
    fs::write(&executable, script).unwrap();
    executable
}

#[test]
fn committed_clark_y_campaign_runs_through_production_cli() {
    let root = TestDirectory::new("clark-y-prod");
    let campaign_path = committed_campaign_path();
    assert!(
        campaign_path.is_file(),
        "committed campaign.json must exist at {:?}",
        campaign_path
    );

    let executable = write_fake_xfoil(root.path());
    let output_dir = root.path().join("output");
    let unrelated_cwd = root.path().join("unrelated-cwd");
    fs::create_dir(&unrelated_cwd).unwrap();

    let output = Command::new(env!("CARGO_BIN_EXE_rcsim-app"))
        .arg("xfoil")
        .arg("run-campaign")
        .arg("--manifest")
        .arg(&campaign_path)
        .arg("--xfoil-executable")
        .arg(&executable)
        .arg("--output-dir")
        .arg(&output_dir)
        .arg("--timeout-seconds")
        .arg("30")
        .current_dir(&unrelated_cwd)
        .output()
        .unwrap();

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(
        output.status.code(),
        Some(0),
        "production CLI must exit 0 for the committed Clark Y campaign; stderr: {stderr}"
    );

    for i in 0..6 {
        let polar = output_dir.join(format!("polars/{i:04}.polar"));
        assert!(polar.is_file(), "polar {i:04}.polar must exist");
    }

    let report_path = output_dir.join(EXECUTION_JSON);
    assert!(report_path.is_file(), "execution report must exist");
    let report = read_json(&report_path);

    assert_eq!(report["schema_version"], 1);
    assert_eq!(report["campaign_id"], "clark-y-reference-v1");
    assert_eq!(report["airfoil_file"], "clarky.dat");
    assert_eq!(report["run_count"], 6);
    assert_eq!(report["completed_run_count"], 6);
    assert_eq!(report["status"], "completed");

    for (i, &expected_re) in EXPECTED_REYNOLDS.iter().enumerate() {
        assert_eq!(
            report["runs"][i]["reynolds"],
            json!(expected_re),
            "run {i} Reynolds mismatch"
        );
        assert_eq!(
            report["runs"][i]["dataset_id"],
            json!(EXPECTED_DATASET_IDS[i]),
            "run {i} dataset_id mismatch"
        );
        assert_eq!(
            report["runs"][i]["execution_status"],
            json!("completed_parseable"),
            "run {i} must be completed_parseable"
        );
        assert!(
            report["runs"][i]["parsed_sample_count"].as_u64().unwrap() > 0,
            "run {i} must have parsed samples"
        );
    }

    let validation_path = output_dir.join(VALIDATION_MANIFEST);
    assert!(
        validation_path.is_file(),
        "validation manifest must exist after successful campaign"
    );
    let validation = read_json(&validation_path);
    assert_eq!(validation["schema_version"], 1);
    assert_eq!(validation["campaign_id"], "clark-y-reference-v1");
    assert_eq!(validation["datasets"].as_array().unwrap().len(), 6);

    for (i, dataset) in validation["datasets"]
        .as_array()
        .unwrap()
        .iter()
        .enumerate()
    {
        assert_eq!(
            dataset["convergence_status"], "unresolved",
            "run {i} convergence_status must be unresolved"
        );
        assert_eq!(
            dataset["reynolds"],
            json!(EXPECTED_REYNOLDS[i]),
            "validation run {i} Reynolds mismatch"
        );
    }

    assert_eq!(
        validation["coverage_request"]["require_converged"],
        json!(false),
        "coverage require_converged must remain false"
    );
}
