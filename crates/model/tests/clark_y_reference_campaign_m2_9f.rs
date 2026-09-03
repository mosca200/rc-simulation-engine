//! M2.9F — Clark Y reference XFOIL campaign package validation.
//!
//! Validates the committed Clark Y reference package: airfoil geometry,
//! provenance record, M2.9E execution manifest, and M2.9D coverage request.

use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};

// ---------------------------------------------------------------------------
// Paths
// ---------------------------------------------------------------------------

fn project_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn package_dir() -> PathBuf {
    project_root()
        .join("reference")
        .join("xfoil")
        .join("clark_y")
}

fn airfoil_path() -> PathBuf {
    package_dir().join("clarky.dat")
}

fn source_path() -> PathBuf {
    package_dir().join("source.json")
}

fn campaign_path() -> PathBuf {
    package_dir().join("campaign.json")
}

// ---------------------------------------------------------------------------
// Manifest structs (mirror M2.9E ExecutionManifest for parse validation)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct ExecutionManifest {
    schema_version: u32,
    campaign_id: String,
    airfoil_file: String,
    runs: Vec<RunSpec>,
    coverage_request: CoverageRequestSpec,
}

#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct RunSpec {
    dataset_id: String,
    reynolds: f64,
    mach: f64,
    alpha_start_deg: f64,
    alpha_end_deg: f64,
    alpha_step_deg: f64,
    maximum_iterations: u32,
    ncrit: f64,
}

#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
struct CoverageRequestSpec {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

// ---------------------------------------------------------------------------
// Provenance struct
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Deserialize)]
#[allow(dead_code)]
struct SourceProvenance {
    schema_version: u32,
    airfoil_id: String,
    display_name: String,
    source_name: String,
    source_file_name: String,
    source_format: String,
    source_locator: String,
    retrieved_asset_sha256: String,
    point_count: u64,
    first_data_coordinate: Coordinate,
    last_data_coordinate: Coordinate,
}

#[derive(Debug, Clone, Deserialize)]
struct Coordinate {
    x: f64,
    y: f64,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

struct ParsedAirfoil {
    #[allow(dead_code)]
    header: String,
    points: Vec<(f64, f64)>,
}

fn parse_airfoil(text: &str) -> ParsedAirfoil {
    let mut lines = text.lines();
    let header = lines
        .next()
        .expect("file must have a header line")
        .to_owned();
    let points: Vec<(f64, f64)> = lines
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            let mut parts = line.split_whitespace();
            let x: f64 = parts
                .next()
                .unwrap_or_else(|| panic!("missing x on line: {line}"))
                .parse()
                .unwrap_or_else(|_| panic!("invalid x on line: {line}"));
            let y: f64 = parts
                .next()
                .unwrap_or_else(|| panic!("missing y on line: {line}"))
                .parse()
                .unwrap_or_else(|_| panic!("invalid y on line: {line}"));
            (x, y)
        })
        .collect();
    ParsedAirfoil { header, points }
}

fn sha256_hex(data: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(data);
    let result = hasher.finalize();
    let mut hex = String::with_capacity(64);
    for byte in result {
        use std::fmt::Write;
        write!(hex, "{byte:02x}").unwrap();
    }
    hex
}

fn load_manifest() -> ExecutionManifest {
    let text = fs::read_to_string(campaign_path()).unwrap();
    serde_json::from_str(&text).unwrap()
}

// ---------------------------------------------------------------------------
// Tests 1–15: Airfoil geometry and provenance
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_01_vendored_clark_y_asset_exists() {
    assert!(airfoil_path().exists(), "clarky.dat must exist");
}

#[test]
fn m2_9f_02_first_line_identifies_clark_y_airfoil() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let header = text.lines().next().unwrap();
    assert!(
        header.trim().eq_ignore_ascii_case("CLARK Y AIRFOIL"),
        "header must identify CLARK Y AIRFOIL, got: {header:?}"
    );
}

#[test]
fn m2_9f_03_selig_format_coordinates_parse_structurally() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    assert!(
        parsed.points.len() >= 10,
        "expected at least 10 coordinate points, got {}",
        parsed.points.len()
    );
}

#[test]
fn m2_9f_04_no_lednicer_point_count_line() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    for line in text.lines().skip(1) {
        if line.trim().is_empty() {
            continue;
        }
        let tokens: Vec<&str> = line.split_whitespace().collect();
        assert!(
            tokens.len() == 2,
            "every data line must have exactly 2 columns (no Lednicer point-count line); \
             found {} tokens on line: {line:?}",
            tokens.len()
        );
    }
}

#[test]
fn m2_9f_05_point_ordering_te_upper_to_le_to_te_lower() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    let points = &parsed.points;

    let first_x = points[0].0;
    let last_x = points[points.len() - 1].0;
    assert!(
        (first_x - 1.0).abs() < 0.01,
        "first point must be near TE (x≈1), got x={first_x}"
    );
    assert!(
        (last_x - 1.0).abs() < 0.01,
        "last point must be near TE (x≈1), got x={last_x}"
    );

    let min_x = points.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let min_idx = points
        .iter()
        .position(|p| (p.0 - min_x).abs() < 1e-12)
        .unwrap();
    assert!(
        min_idx > 0 && min_idx < points.len() - 1,
        "leading edge must be interior, found at index {min_idx} of {}",
        points.len()
    );

    for i in 1..=min_idx {
        assert!(
            points[i].0 <= points[i - 1].0 + 1e-12,
            "upper surface x must be non-increasing toward LE; violated at index {i}"
        );
    }
    for i in (min_idx + 1)..points.len() {
        assert!(
            points[i].0 >= points[i - 1].0 - 1e-12,
            "lower surface x must be non-decreasing away from LE; violated at index {i}"
        );
    }
}

#[test]
fn m2_9f_06_first_numeric_x_approximately_one() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    assert!(
        (parsed.points[0].0 - 1.0).abs() < 0.01,
        "first x must be ≈1.0, got {}",
        parsed.points[0].0
    );
}

#[test]
fn m2_9f_07_leading_edge_x_zero_exists_exactly_once() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    let le_count = parsed
        .points
        .iter()
        .filter(|(x, _)| x.abs() < 1e-10)
        .count();
    assert_eq!(
        le_count, 1,
        "leading edge (x≈0) must appear exactly once, found {le_count}"
    );
}

#[test]
fn m2_9f_08_final_numeric_x_approximately_one() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    let last = parsed.points.last().unwrap();
    assert!(
        (last.0 - 1.0).abs() < 0.01,
        "last x must be ≈1.0, got {}",
        last.0
    );
}

#[test]
fn m2_9f_09_all_coordinates_finite() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    for (i, (x, y)) in parsed.points.iter().enumerate() {
        assert!(x.is_finite(), "x[{i}] is not finite: {x}");
        assert!(y.is_finite(), "y[{i}] is not finite: {y}");
    }
}

#[test]
fn m2_9f_10_normalized_x_within_reference_bounds() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    for (i, &(x, _)) in parsed.points.iter().enumerate() {
        assert!(
            (-0.01..=1.01).contains(&x),
            "x[{i}] = {x} outside expected [0, 1] reference bounds"
        );
    }
}

#[test]
fn m2_9f_11_no_nan_or_inf_coordinates() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);
    for (i, (x, y)) in parsed.points.iter().enumerate() {
        assert!(!x.is_nan() && !x.is_infinite(), "x[{i}] is NaN or Inf");
        assert!(!y.is_nan() && !y.is_infinite(), "y[{i}] is NaN or Inf");
    }
}

#[test]
fn m2_9f_12_sha256_matches_provenance_lock() {
    let airfoil_bytes = fs::read(airfoil_path()).unwrap();
    let computed = sha256_hex(&airfoil_bytes);

    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();

    assert_eq!(
        computed, provenance.retrieved_asset_sha256,
        "SHA-256 of clarky.dat does not match provenance lock"
    );
}

#[test]
fn m2_9f_13_source_file_recorded_as_clarky_dat() {
    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();
    assert_eq!(provenance.source_file_name, "clarky.dat");
}

#[test]
fn m2_9f_14_source_format_recorded_as_selig() {
    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();
    assert_eq!(provenance.source_format, "Selig");
}

#[test]
fn m2_9f_15_source_is_not_clarkysm_dat() {
    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();
    assert_ne!(
        provenance.source_file_name, "clarkysm.dat",
        "must not use the smoothed Clark Y variant"
    );
    assert!(
        !provenance.source_locator.contains("clarkysm"),
        "source locator must not reference clarkysm"
    );
}

// ---------------------------------------------------------------------------
// Tests 16–29: M2.9E execution manifest and coverage request
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_16_manifest_structural_schema_parse() {
    let text = fs::read_to_string(campaign_path()).unwrap();
    let manifest: Result<ExecutionManifest, _> = serde_json::from_str(&text);
    assert!(
        manifest.is_ok(),
        "campaign.json must parse as a structurally valid execution manifest: {:?}",
        manifest.err()
    );
}

#[test]
fn m2_9f_17_campaign_id_exact() {
    let manifest = load_manifest();
    assert_eq!(manifest.campaign_id, "clark-y-reference-v1");
}

#[test]
fn m2_9f_18_run_order_exact() {
    let manifest = load_manifest();
    let expected_ids = [
        "clark-y-re-100000",
        "clark-y-re-150000",
        "clark-y-re-200000",
        "clark-y-re-300000",
        "clark-y-re-500000",
        "clark-y-re-750000",
    ];
    assert_eq!(manifest.runs.len(), expected_ids.len());
    for (i, expected) in expected_ids.iter().enumerate() {
        assert_eq!(
            &manifest.runs[i].dataset_id, expected,
            "run {i} dataset_id mismatch"
        );
    }
}

#[test]
fn m2_9f_19_reynolds_order_exact() {
    let manifest = load_manifest();
    let expected = [
        100_000.0, 150_000.0, 200_000.0, 300_000.0, 500_000.0, 750_000.0,
    ];
    for (i, &exp) in expected.iter().enumerate() {
        assert_eq!(manifest.runs[i].reynolds, exp, "run {i} Reynolds mismatch");
    }
}

#[test]
fn m2_9f_20_no_duplicate_dataset_ids() {
    let manifest = load_manifest();
    let mut seen = HashSet::new();
    for run in &manifest.runs {
        assert!(
            seen.insert(&run.dataset_id),
            "duplicate dataset_id: {}",
            run.dataset_id
        );
    }
}

#[test]
fn m2_9f_21_all_mach_zero() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(run.mach, 0.0, "run {i} mach must be 0.0");
    }
}

#[test]
fn m2_9f_22_all_alpha_start_minus_12() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(
            run.alpha_start_deg, -12.0,
            "run {i} alpha_start_deg must be -12.0"
        );
    }
}

#[test]
fn m2_9f_23_all_alpha_end_18() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(
            run.alpha_end_deg, 18.0,
            "run {i} alpha_end_deg must be 18.0"
        );
    }
}

#[test]
fn m2_9f_24_all_alpha_step_half() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(
            run.alpha_step_deg, 0.5,
            "run {i} alpha_step_deg must be 0.5"
        );
    }
}

#[test]
fn m2_9f_25_all_max_iterations_200() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(
            run.maximum_iterations, 200,
            "run {i} maximum_iterations must be 200"
        );
    }
}

#[test]
fn m2_9f_26_all_ncrit_9() {
    let manifest = load_manifest();
    for (i, run) in manifest.runs.iter().enumerate() {
        assert_eq!(run.ncrit, 9.0, "run {i} ncrit must be 9.0");
    }
}

#[test]
fn m2_9f_27_coverage_reynolds_bounds_match_first_last_run() {
    let manifest = load_manifest();
    let first_re = manifest.runs.first().unwrap().reynolds;
    let last_re = manifest.runs.last().unwrap().reynolds;
    assert_eq!(
        manifest.coverage_request.required_reynolds_min, first_re,
        "coverage required_reynolds_min must match first run Reynolds"
    );
    assert_eq!(
        manifest.coverage_request.required_reynolds_max, last_re,
        "coverage required_reynolds_max must match last run Reynolds"
    );
}

#[test]
fn m2_9f_28_coverage_alpha_bounds_match_sweep_envelope_in_radians() {
    let manifest = load_manifest();
    let alpha_start_rad = (-12.0_f64).to_radians();
    let alpha_end_rad = (18.0_f64).to_radians();
    assert_eq!(
        manifest.coverage_request.required_alpha_min_rad, alpha_start_rad,
        "coverage alpha_min_rad must equal -12 deg in radians"
    );
    assert_eq!(
        manifest.coverage_request.required_alpha_max_rad, alpha_end_rad,
        "coverage alpha_max_rad must equal 18 deg in radians"
    );
}

#[test]
fn m2_9f_29_require_converged_is_false() {
    let manifest = load_manifest();
    assert!(
        !manifest.coverage_request.require_converged,
        "require_converged must be false"
    );
}

// ---------------------------------------------------------------------------
// Tests 30–31: Path resolution and CWD independence
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_30_airfoil_path_resolves_relative_to_manifest_directory() {
    let manifest = load_manifest();
    assert_eq!(manifest.airfoil_file, "clarky.dat");

    let campaign = campaign_path();
    let manifest_dir = campaign.parent().unwrap();
    let resolved = manifest_dir.join(&manifest.airfoil_file);
    assert!(
        resolved.exists(),
        "airfoil must resolve relative to manifest directory: {:?}",
        resolved
    );
}

// ---------------------------------------------------------------------------
// Tests 32–34: Determinism
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_32_repeated_serialization_is_byte_identical() {
    let source_text = fs::read_to_string(source_path()).unwrap();
    let source_value: Value = serde_json::from_str(&source_text).unwrap();
    let reserialized = serde_json::to_string_pretty(&source_value).unwrap();
    let reserialized_twice: String =
        serde_json::to_string_pretty(&serde_json::from_str::<Value>(&reserialized).unwrap())
            .unwrap();
    assert_eq!(
        reserialized, reserialized_twice,
        "source.json must be deterministically serializable"
    );

    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    let campaign_value: Value = serde_json::from_str(&campaign_text).unwrap();
    let re_campaign = serde_json::to_string_pretty(&campaign_value).unwrap();
    let re_campaign_twice: String =
        serde_json::to_string_pretty(&serde_json::from_str::<Value>(&re_campaign).unwrap())
            .unwrap();
    assert_eq!(
        re_campaign, re_campaign_twice,
        "campaign.json must be deterministically serializable"
    );
}

#[test]
fn m2_9f_33_no_absolute_machine_specific_paths_in_artifacts() {
    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    assert!(
        !campaign_text.contains("C:\\")
            && !campaign_text.contains("/Users/")
            && !campaign_text.contains("/home/"),
        "campaign.json must not contain absolute machine-specific paths"
    );

    let source_text = fs::read_to_string(source_path()).unwrap();
    let source_value: Value = serde_json::from_str(&source_text).unwrap();
    if let Some(locator) = source_value.get("source_locator").and_then(|v| v.as_str()) {
        assert!(
            locator.starts_with("http://") || locator.starts_with("https://"),
            "source_locator must be a URL, not a local path"
        );
    }
}

#[test]
fn m2_9f_34_no_timestamps_in_deterministic_artifacts() {
    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    let campaign_value: Value = serde_json::from_str(&campaign_text).unwrap();
    assert!(
        campaign_value.get("timestamp").is_none()
            && campaign_value.get("generated_at").is_none()
            && campaign_value.get("created_at").is_none(),
        "campaign.json must not contain timestamps"
    );
}

// ---------------------------------------------------------------------------
// Tests 35–38: No fabricated results
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_35_no_xfoil_polar_values_committed() {
    let package = package_dir();
    let polar_files: Vec<_> = fs::read_dir(&package)
        .unwrap()
        .filter_map(|e| e.ok())
        .filter(|e| {
            e.path()
                .extension()
                .is_some_and(|ext| ext == "pol" || ext == "out")
        })
        .collect();
    assert!(
        polar_files.is_empty(),
        "no polar output files should be committed in the reference package"
    );
}

#[test]
fn m2_9f_36_no_cl_cd_cm_values_invented() {
    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    let lower = campaign_text.to_lowercase();
    assert!(
        !lower.contains("\"cl\"")
            && !lower.contains("\"cd\"")
            && !lower.contains("\"cm\"")
            && !lower.contains("lift_coefficient")
            && !lower.contains("drag_coefficient"),
        "campaign.json must not contain invented aerodynamic coefficients"
    );
}

#[test]
fn m2_9f_37_no_convergence_status_converged_committed() {
    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    let lower = campaign_text.to_lowercase();
    assert!(
        !lower.contains("\"converged\"") && !lower.contains("convergence_status"),
        "campaign.json must not claim convergence_status: converged"
    );
}

#[test]
fn m2_9f_38_no_lt40_aircraft_operating_point_claim() {
    let campaign_text = fs::read_to_string(campaign_path()).unwrap();
    let lower = campaign_text.to_lowercase();
    assert!(
        !lower.contains("lt-40") && !lower.contains("lt40") && !lower.contains("kadet"),
        "campaign.json must not reference LT-40 or Kadet aircraft"
    );

    let source_text = fs::read_to_string(source_path()).unwrap();
    let lower_source = source_text.to_lowercase();
    assert!(
        !lower_source.contains("lt-40")
            && !lower_source.contains("lt40")
            && !lower_source.contains("kadet"),
        "source.json must not reference LT-40 or Kadet aircraft"
    );
}

// ---------------------------------------------------------------------------
// Test 39: Existing M2.9C/D/E types still construct
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_39_existing_m2_9_model_types_still_construct() {
    use model::{XfoilCampaignCoverageRequest, XfoilEvidenceCampaignBuilder};

    let coverage = XfoilCampaignCoverageRequest::new(100_000.0, 750_000.0, -0.3, 0.3, false);
    assert!(
        coverage.is_ok(),
        "M2.9C coverage request must still construct"
    );

    let empty = XfoilEvidenceCampaignBuilder::new(vec![]).build();
    assert!(
        empty.is_err(),
        "M2.9C empty campaign must still be rejected"
    );
}

// ---------------------------------------------------------------------------
// Test 40: Source asset not modified by tests
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_40_source_asset_not_modified_by_validation() {
    let before = fs::read(airfoil_path()).unwrap();
    let before_hash = sha256_hex(&before);

    let text = fs::read_to_string(airfoil_path()).unwrap();
    let _parsed = parse_airfoil(&text);

    let after = fs::read(airfoil_path()).unwrap();
    let after_hash = sha256_hex(&after);

    assert_eq!(
        before_hash, after_hash,
        "reading and parsing the airfoil must not modify the source asset"
    );
}

// ---------------------------------------------------------------------------
// Additional structural validation
// ---------------------------------------------------------------------------

#[test]
fn m2_9f_provenance_schema_version_is_one() {
    let text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&text).unwrap();
    assert_eq!(provenance.schema_version, 1);
}

#[test]
fn m2_9f_provenance_airfoil_id() {
    let text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&text).unwrap();
    assert_eq!(provenance.airfoil_id, "clark-y");
}

#[test]
fn m2_9f_provenance_point_count_matches_actual() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);

    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();

    assert_eq!(
        parsed.points.len() as u64,
        provenance.point_count,
        "provenance point_count must match actual parsed point count"
    );
}

#[test]
fn m2_9f_provenance_first_last_coordinates_match() {
    let text = fs::read_to_string(airfoil_path()).unwrap();
    let parsed = parse_airfoil(&text);

    let source_text = fs::read_to_string(source_path()).unwrap();
    let provenance: SourceProvenance = serde_json::from_str(&source_text).unwrap();

    let first = parsed.points.first().unwrap();
    let last = parsed.points.last().unwrap();

    assert!(
        (first.0 - provenance.first_data_coordinate.x).abs() < 1e-10
            && (first.1 - provenance.first_data_coordinate.y).abs() < 1e-10,
        "first coordinate mismatch"
    );
    assert!(
        (last.0 - provenance.last_data_coordinate.x).abs() < 1e-10
            && (last.1 - provenance.last_data_coordinate.y).abs() < 1e-10,
        "last coordinate mismatch"
    );
}

#[test]
fn m2_9f_manifest_schema_version_is_one() {
    let manifest = load_manifest();
    assert_eq!(manifest.schema_version, 1);
}

#[test]
fn m2_9f_manifest_airfoil_file_is_relative() {
    let manifest = load_manifest();
    let path = Path::new(&manifest.airfoil_file);
    assert!(
        path.is_relative(),
        "airfoil_file must be relative, got: {:?}",
        manifest.airfoil_file
    );
}

#[test]
fn m2_9f_reynolds_strictly_increasing() {
    let manifest = load_manifest();
    for i in 1..manifest.runs.len() {
        assert!(
            manifest.runs[i].reynolds > manifest.runs[i - 1].reynolds,
            "Reynolds must be strictly increasing: run {} ({}) <= run {} ({})",
            i - 1,
            manifest.runs[i - 1].reynolds,
            i,
            manifest.runs[i].reynolds
        );
    }
}

#[test]
fn m2_9f_dataset_ids_are_stable() {
    let manifest = load_manifest();
    for run in &manifest.runs {
        assert!(
            !run.dataset_id.is_empty()
                && run.dataset_id.bytes().all(|b| b.is_ascii_lowercase()
                    || b.is_ascii_digit()
                    || b == b'_'
                    || b == b'-'),
            "dataset_id {:?} contains invalid characters",
            run.dataset_id
        );
    }
}
