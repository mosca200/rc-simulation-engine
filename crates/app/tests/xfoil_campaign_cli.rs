#![forbid(unsafe_code)]

use serde_json::{Value, json};
use std::{
    fs,
    path::{Path, PathBuf},
    process::{Command, Output},
    sync::atomic::{AtomicUsize, Ordering},
};

const JSON_REPORT: &str = "xfoil_campaign.json";
const MARKDOWN_REPORT: &str = "xfoil_campaign.md";
const POLAR_REPORT: &str = "polar_datasets.json";
const WIDE_POLAR: &str =
    "alpha CL CD CM\n----- -- -- --\n-10 -0.8 0.03 -0.01\n0 0 0.01 -0.01\n10 0.8 0.03 -0.01\n";
const NARROW_POLAR: &str = "alpha CL CD CDp CM Top_Xtr Bot_Xtr\n----- -- -- --- -- ------- -------\n-2 -0.2 0.02 0.01 -0.01 0.5 0.5\n0 0 0.01 0.005 -0.01 0.5 0.5\n2 0.2 0.02 0.01 -0.01 0.5 0.5\n";

struct TestDirectory(PathBuf);

impl TestDirectory {
    fn new(label: &str) -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        loop {
            let path = std::env::temp_dir().join(format!(
                "rcsim_m2_9d_{label}_{}_{}",
                std::process::id(),
                NEXT.fetch_add(1, Ordering::Relaxed)
            ));
            match fs::create_dir(&path) {
                Ok(()) => return Self(path),
                Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
                Err(error) => panic!("cannot create {path:?}: {error}"),
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

fn dataset(file: &str, id: &str, reynolds: f64, status: &str) -> Value {
    json!({
        "polar_file": file,
        "dataset_id": id,
        "method_id": format!("method-{id}"),
        "convergence_status": status,
        "source_ids": [format!("source-{id}")],
        "notes": "synthetic fixture",
        "reynolds": reynolds,
        "mach": 0.03,
        "solver_name": "synthetic solver",
        "solver_version": "test-only",
        "command_or_config": "fixture",
        "transition_assumptions": "synthetic",
        "ncrit": 9.0,
        "forced_transition_upper_x_over_c": 0.8,
        "forced_transition_lower_x_over_c": 0.7
    })
}

fn request(require_converged: bool) -> Value {
    json!({
        "required_reynolds_min": 100_000.0,
        "required_reynolds_max": 200_000.0,
        "required_alpha_min_rad": -0.1,
        "required_alpha_max_rad": 0.1,
        "require_converged": require_converged
    })
}

fn manifest(datasets: Vec<Value>, coverage_request: Value) -> Value {
    json!({
        "schema_version": 1,
        "campaign_id": "synthetic-campaign",
        "datasets": datasets,
        "coverage_request": coverage_request
    })
}

fn qualifying_manifest() -> Value {
    manifest(
        vec![
            dataset("low.txt", "low", 100_000.0, "converged"),
            dataset("high.txt", "high", 200_000.0, "converged"),
        ],
        request(true),
    )
}

fn write_fixture(root: &Path, value: &Value) -> PathBuf {
    let directory = root.join("manifest");
    fs::create_dir(&directory).unwrap();
    fs::write(directory.join("low.txt"), WIDE_POLAR).unwrap();
    fs::write(directory.join("high.txt"), WIDE_POLAR).unwrap();
    fs::write(directory.join("narrow.txt"), NARROW_POLAR).unwrap();
    fs::write(directory.join("bad.txt"), "not a polar\n").unwrap();
    let path = directory.join("campaign.json");
    fs::write(&path, serde_json::to_vec_pretty(value).unwrap()).unwrap();
    path
}

fn command(manifest_path: &Path, output_dir: &Path) -> Command {
    let mut command = Command::new(env!("CARGO_BIN_EXE_rcsim-app"));
    command
        .arg("validate")
        .arg("xfoil-campaign")
        .arg("--manifest")
        .arg(manifest_path)
        .arg("--output-dir")
        .arg(output_dir);
    command
}

fn run(manifest_path: &Path, output_dir: &Path) -> Output {
    command(manifest_path, output_dir).output().unwrap()
}

fn report(output_dir: &Path) -> Value {
    serde_json::from_slice(&fs::read(output_dir.join(JSON_REPORT)).unwrap()).unwrap()
}

fn assert_reports(output_dir: &Path) {
    let mut names: Vec<_> = fs::read_dir(output_dir)
        .unwrap()
        .map(|entry| entry.unwrap().file_name())
        .collect();
    names.sort();
    assert_eq!(names, [POLAR_REPORT, JSON_REPORT, MARKDOWN_REPORT]);
}

#[test]
fn valid_multi_dataset_manifest_is_qualified_and_paths_are_manifest_relative() {
    let root = TestDirectory::new("qualified");
    let path = write_fixture(root.path(), &qualifying_manifest());
    let output_dir = root.path().join("reports");
    let other_cwd = root.path().join("other-cwd");
    fs::create_dir(&other_cwd).unwrap();
    let output = command(&path, &output_dir)
        .current_dir(other_cwd)
        .output()
        .unwrap();
    assert_eq!(
        output.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert_reports(&output_dir);

    let value = report(&output_dir);
    assert_eq!(value["schema_version"], 1);
    assert_eq!(value["generated_by"], "rcsim-app validate xfoil-campaign");
    assert_eq!(value["status"], "qualified");
    assert_eq!(value["summary"]["dataset_count"], 2);
    assert_eq!(value["summary"]["converged_dataset_count"], 2);
    assert_eq!(value["summary"]["blocker_count"], 0);
    assert_eq!(value["datasets"][0]["polar_file"], "low.txt");
    assert_eq!(value["datasets"][1]["polar_file"], "high.txt");
}

#[test]
fn one_node_completes_not_qualified_under_the_inherited_range_rule() {
    let root = TestDirectory::new("one-node");
    let path = write_fixture(
        root.path(),
        &manifest(
            vec![dataset("low.txt", "only", 100_000.0, "converged")],
            request(true),
        ),
    );
    let output_dir = root.path().join("reports");
    assert_eq!(run(&path, &output_dir).status.code(), Some(2));
    assert_reports(&output_dir);
    assert_eq!(report(&output_dir)["status"], "not_qualified");
}

#[test]
fn campaign_order_and_identity_errors_exit_one_without_reports() {
    let cases = [
        (
            "reversed",
            vec![
                dataset("low.txt", "a", 200_000.0, "converged"),
                dataset("high.txt", "b", 100_000.0, "converged"),
            ],
            "not increasing",
        ),
        (
            "duplicate-re",
            vec![
                dataset("low.txt", "a", 100_000.0, "converged"),
                dataset("high.txt", "b", 100_000.0, "converged"),
            ],
            "duplicate campaign Reynolds node",
        ),
        (
            "duplicate-id",
            vec![
                dataset("low.txt", "same", 100_000.0, "converged"),
                dataset("high.txt", "same", 200_000.0, "converged"),
            ],
            "duplicate campaign dataset ID",
        ),
    ];
    for (label, datasets, expected) in cases {
        let root = TestDirectory::new(label);
        let path = write_fixture(root.path(), &manifest(datasets, request(true)));
        let output_dir = root.path().join("reports");
        let output = run(&path, &output_dir);
        assert_eq!(output.status.code(), Some(1));
        assert!(String::from_utf8_lossy(&output.stderr).contains(expected));
        assert!(!output_dir.exists());
    }
}

#[test]
fn malformed_polar_has_dataset_context_and_strict_manifest_rejects_bad_input() {
    let root = TestDirectory::new("bad-polar");
    let path = write_fixture(
        root.path(),
        &manifest(
            vec![
                dataset("low.txt", "low", 100_000.0, "converged"),
                dataset("bad.txt", "broken", 200_000.0, "converged"),
            ],
            request(true),
        ),
    );
    let output = run(&path, &root.path().join("reports"));
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert_eq!(output.status.code(), Some(1));
    for part in ["index 1", "broken", "bad.txt", "header line not found"] {
        assert!(stderr.contains(part), "{stderr}");
    }

    let strict_cases = [
        ("version", {
            let mut value = qualifying_manifest();
            value["schema_version"] = json!(2);
            value
        }),
        ("unknown", {
            let mut value = qualifying_manifest();
            value["future_field"] = json!(true);
            value
        }),
        ("status", {
            let mut value = qualifying_manifest();
            value["datasets"][0]["convergence_status"] = json!("not_applicable_published");
            value
        }),
        ("empty", manifest(Vec::new(), request(true))),
        ("missing-datasets", {
            let mut value = qualifying_manifest();
            value.as_object_mut().unwrap().remove("datasets");
            value
        }),
    ];
    for (label, value) in strict_cases {
        let root = TestDirectory::new(label);
        let path = write_fixture(root.path(), &value);
        assert_eq!(
            run(&path, &root.path().join("reports")).status.code(),
            Some(1),
            "{label}"
        );
    }
}

#[test]
fn explicit_convergence_policy_changes_only_qualification() {
    for (required, exit, status, blockers) in
        [(true, 2, "not_qualified", 1), (false, 0, "qualified", 0)]
    {
        let root = TestDirectory::new(if required { "required" } else { "optional" });
        let path = write_fixture(
            root.path(),
            &manifest(
                vec![
                    dataset("low.txt", "low", 100_000.0, "unresolved"),
                    dataset("high.txt", "high", 200_000.0, "converged"),
                ],
                request(required),
            ),
        );
        let output_dir = root.path().join("reports");
        assert_eq!(run(&path, &output_dir).status.code(), Some(exit));
        assert_reports(&output_dir);
        let value = report(&output_dir);
        assert_eq!(value["status"], status);
        assert_eq!(value["datasets"][0]["convergence_status"], "unresolved");
        assert_eq!(value["summary"]["unresolved_dataset_count"], 1);
        assert_eq!(value["summary"]["blocker_count"], blockers);
    }
}

#[test]
fn all_coverage_gap_kinds_are_structured_and_write_reports() {
    let cases = [
        (
            "re-below",
            vec![
                dataset("low.txt", "low", 110_000.0, "converged"),
                dataset("high.txt", "high", 200_000.0, "converged"),
            ],
            "reynolds_coverage_below_required",
        ),
        (
            "re-above",
            vec![
                dataset("low.txt", "low", 100_000.0, "converged"),
                dataset("high.txt", "high", 190_000.0, "converged"),
            ],
            "reynolds_coverage_above_required",
        ),
        (
            "alpha-below",
            vec![
                dataset("narrow.txt", "low", 100_000.0, "converged"),
                dataset("high.txt", "high", 200_000.0, "converged"),
            ],
            "dataset_alpha_below_required",
        ),
        (
            "alpha-above",
            vec![
                dataset("low.txt", "low", 100_000.0, "converged"),
                dataset("narrow.txt", "high", 200_000.0, "converged"),
            ],
            "dataset_alpha_above_required",
        ),
    ];
    for (label, datasets, kind) in cases {
        let root = TestDirectory::new(label);
        let path = write_fixture(root.path(), &manifest(datasets, request(true)));
        let output_dir = root.path().join("reports");
        assert_eq!(run(&path, &output_dir).status.code(), Some(2));
        assert_reports(&output_dir);
        assert!(
            report(&output_dir)["blockers"]
                .as_array()
                .unwrap()
                .iter()
                .any(|blocker| blocker["kind"] == kind)
        );
        if label == "re-below" {
            assert_eq!(
                report(&output_dir)["blockers"][0],
                json!({
                    "kind": "reynolds_coverage_below_required",
                    "campaign_minimum_reynolds": 110_000.0,
                    "required_minimum_reynolds": 100_000.0
                })
            );
        }
    }
}

#[test]
fn simultaneous_blockers_preserve_exact_model_order_in_json_and_markdown() {
    let root = TestDirectory::new("blocker-order");
    let coverage_request = json!({
        "required_reynolds_min": 90_000.0,
        "required_reynolds_max": 210_000.0,
        "required_alpha_min_rad": -0.1,
        "required_alpha_max_rad": 0.1,
        "require_converged": true
    });
    let path = write_fixture(
        root.path(),
        &manifest(
            vec![
                dataset("narrow.txt", "low", 100_000.0, "unresolved"),
                dataset("narrow.txt", "high", 200_000.0, "failed"),
            ],
            coverage_request,
        ),
    );
    let output_dir = root.path().join("reports");
    assert_eq!(run(&path, &output_dir).status.code(), Some(2));
    let value = report(&output_dir);
    let kinds: Vec<_> = value["blockers"]
        .as_array()
        .unwrap()
        .iter()
        .map(|blocker| blocker["kind"].as_str().unwrap())
        .collect();
    let expected = [
        "reynolds_coverage_below_required",
        "reynolds_coverage_above_required",
        "dataset_not_converged",
        "dataset_alpha_below_required",
        "dataset_alpha_above_required",
        "dataset_not_converged",
        "dataset_alpha_below_required",
        "dataset_alpha_above_required",
    ];
    assert_eq!(kinds, expected);

    let markdown = fs::read_to_string(output_dir.join(MARKDOWN_REPORT)).unwrap();
    let mut cursor = 0;
    for kind in expected {
        let offset = markdown[cursor..].find(kind).unwrap();
        cursor += offset + kind.len();
    }
    assert!(markdown.contains("does not prove solver correctness"));
}

#[test]
fn reports_are_byte_identical_ordered_timestamp_free_generated_solver_evidence() {
    let root = TestDirectory::new("determinism");
    let path = write_fixture(root.path(), &qualifying_manifest());
    let first = root.path().join("first");
    let second = root.path().join("second");
    assert_eq!(run(&path, &first).status.code(), Some(0));
    assert_eq!(run(&path, &second).status.code(), Some(0));
    for name in [JSON_REPORT, MARKDOWN_REPORT, POLAR_REPORT] {
        assert_eq!(
            fs::read(first.join(name)).unwrap(),
            fs::read(second.join(name)).unwrap()
        );
    }

    let json_text = fs::read_to_string(first.join(JSON_REPORT)).unwrap();
    let markdown = fs::read_to_string(first.join(MARKDOWN_REPORT)).unwrap();
    for forbidden in ["timestamp", "generated_at", "current_date"] {
        assert!(!json_text.contains(forbidden));
        assert!(!markdown.contains(forbidden));
    }
    assert!(!json_text.contains(root.path().to_string_lossy().as_ref()));

    let polar: Value =
        serde_json::from_slice(&fs::read(first.join(POLAR_REPORT)).unwrap()).unwrap();
    assert_eq!(polar[0]["id"], "low");
    assert_eq!(polar[1]["id"], "high");
    assert!(
        polar
            .as_array()
            .unwrap()
            .iter()
            .all(|dataset| dataset["evidence_class"] == "generated_solver")
    );
}

#[test]
fn canonical_metadata_and_coverage_validation_fail_operationally() {
    let root = TestDirectory::new("metadata");
    let path = write_fixture(
        root.path(),
        &manifest(
            vec![
                dataset("low.txt", "low", 0.0, "converged"),
                dataset("high.txt", "high", 200_000.0, "converged"),
            ],
            request(true),
        ),
    );
    let output = run(&path, &root.path().join("reports"));
    assert_eq!(output.status.code(), Some(1));
    assert!(
        String::from_utf8_lossy(&output.stderr)
            .contains("Reynolds number must be finite and positive")
    );

    let root = TestDirectory::new("optional-metadata");
    let mut invalid_metadata = qualifying_manifest();
    invalid_metadata["datasets"][0]["ncrit"] = json!(0.0);
    let path = write_fixture(root.path(), &invalid_metadata);
    let output = run(&path, &root.path().join("reports"));
    assert_eq!(output.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&output.stderr).contains("Ncrit must be finite and positive"));

    let root = TestDirectory::new("coverage");
    let invalid_request = json!({
        "required_reynolds_min": 200_000.0,
        "required_reynolds_max": 100_000.0,
        "required_alpha_min_rad": -0.1,
        "required_alpha_max_rad": 0.1,
        "require_converged": true
    });
    let path = write_fixture(
        root.path(),
        &manifest(
            vec![
                dataset("low.txt", "low", 100_000.0, "converged"),
                dataset("high.txt", "high", 200_000.0, "converged"),
            ],
            invalid_request,
        ),
    );
    let output = run(&path, &root.path().join("reports"));
    assert_eq!(output.status.code(), Some(1));
    assert!(String::from_utf8_lossy(&output.stderr).contains("bounds must be strictly increasing"));
}
