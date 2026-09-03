//! M2.9K — Canonical XFOIL evidence JSON → runtime polar family integration tests.
//!
//! All fixtures use synthetic data. No Clark Y or LT-40 aerodynamic data appears here.

use model::{
    ConvergenceStatus, MetadataBuilder, XfoilEvidenceCampaignBuilder, XfoilEvidenceDatasetBuilder,
    XfoilEvidenceJsonError, build_xfoil_reynolds_polar_family,
    build_xfoil_reynolds_polar_family_from_json, build_xfoil_reynolds_polar_family_from_json_str,
    parse_xfoil_polar,
};

const SOURCE_ID: &str = "synthetic-m2-9k-source";

/// Build a dataset using the existing M2.9B pipeline, then serialize to canonical JSON.
fn canonical_json_from_datasets(datasets: Vec<model::XfoilEvidenceDataset>) -> String {
    let campaign = XfoilEvidenceCampaignBuilder::new(datasets).build().unwrap();
    campaign.to_polar_datasets_json_pretty()
}

fn build_dataset(
    dataset_id: &str,
    reynolds: f64,
    mach: f64,
    status: ConvergenceStatus,
    alpha_degrees: &[f64],
) -> model::XfoilEvidenceDataset {
    assert!(alpha_degrees.len() >= 2);
    let mut polar = String::from(
        " alpha    CL         CD         CM\n\
         ------   ---------  ---------  ---------\n",
    );
    for (index, alpha) in alpha_degrees.iter().enumerate() {
        let cl = -0.2 + index as f64 * 0.25;
        let cd = 0.01 + index as f64 * 0.001;
        let cm = -0.02 - index as f64 * 0.005;
        polar.push_str(&format!(" {alpha:.6} {cl:.6} {cd:.6} {cm:.6}\n"));
    }

    let metadata = MetadataBuilder::new(reynolds, mach)
        .solver_name("Synthetic XFOIL test double")
        .solver_version("test-only-1")
        .command_or_config(format!("SYNTHETIC RE {reynolds:.0} M {mach:.4}"))
        .transition_assumptions("Synthetic free-transition test assumption")
        .ncrit(9.0)
        .build()
        .unwrap();
    let import = parse_xfoil_polar(&polar, metadata).unwrap();

    XfoilEvidenceDatasetBuilder::new(
        import,
        dataset_id,
        format!("method-{dataset_id}"),
        status,
        vec![SOURCE_ID.to_owned()],
    )
    .notes("Synthetic M2.9K test fixture")
    .build()
    .unwrap()
}

fn standard_converged_dataset(dataset_id: &str, reynolds: f64) -> model::XfoilEvidenceDataset {
    build_dataset(
        dataset_id,
        reynolds,
        0.0,
        ConvergenceStatus::Converged,
        &[-5.0, 0.0, 5.0, 10.0],
    )
}

// ── Test 1: valid multi-Re family ────────────────────────────────────────────

#[test]
fn valid_multi_re_family_from_json() {
    let datasets = vec![
        standard_converged_dataset("ds-low", 100_000.0),
        standard_converged_dataset("ds-mid", 300_000.0),
        standard_converged_dataset("ds-high", 500_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);

    let runtime = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap();
    assert_eq!(runtime.family().nodes().len(), 3);
    assert_eq!(runtime.mach(), 0.0);

    let reynolds_values: [f64; 3] = [100_000.0, 300_000.0, 500_000.0];
    for (node, &expected_re) in runtime.family().nodes().iter().zip(&reynolds_values) {
        assert_eq!(node.reynolds_number().to_bits(), expected_re.to_bits());
    }
}

// ── Test 2: malformed JSON ───────────────────────────────────────────────────

#[test]
fn malformed_json_rejected() {
    let err = build_xfoil_reynolds_polar_family_from_json(b"not json at all").unwrap_err();
    assert!(matches!(err, XfoilEvidenceJsonError::MalformedJson(_)));
}

#[test]
fn truncated_json_rejected() {
    let err = build_xfoil_reynolds_polar_family_from_json(b"[{\"id\":").unwrap_err();
    assert!(matches!(err, XfoilEvidenceJsonError::MalformedJson(_)));
}

#[test]
fn wrong_type_json_rejected() {
    let err = build_xfoil_reynolds_polar_family_from_json(b"{\"not\": \"an array\"}").unwrap_err();
    assert!(matches!(err, XfoilEvidenceJsonError::MalformedJson(_)));
}

// ── Test 3: empty list ───────────────────────────────────────────────────────

#[test]
fn empty_dataset_array_rejected() {
    let err = build_xfoil_reynolds_polar_family_from_json(b"[]").unwrap_err();
    assert!(matches!(err, XfoilEvidenceJsonError::EmptyDatasetArray));
}

// ── Test 4: unresolved/failed dataset rejection ──────────────────────────────

#[test]
fn unresolved_dataset_rejected_via_json() {
    let datasets = vec![build_dataset(
        "ds-unresolved",
        100_000.0,
        0.0,
        ConvergenceStatus::Unresolved,
        &[-5.0, 0.0, 5.0],
    )];
    let json = canonical_json_from_datasets(datasets);

    let err = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap_err();
    match err {
        XfoilEvidenceJsonError::DatasetNotConverged {
            index,
            dataset_id,
            status,
        } => {
            assert_eq!(index, 0);
            assert_eq!(dataset_id, "ds-unresolved");
            assert_eq!(status, ConvergenceStatus::Unresolved);
        }
        other => panic!("expected DatasetNotConverged, got {other:?}"),
    }
}

#[test]
fn failed_dataset_rejected_via_json() {
    let datasets = vec![build_dataset(
        "ds-failed",
        100_000.0,
        0.0,
        ConvergenceStatus::Failed,
        &[-5.0, 0.0, 5.0],
    )];
    let json = canonical_json_from_datasets(datasets);

    let err = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap_err();
    match err {
        XfoilEvidenceJsonError::DatasetNotConverged { status, .. } => {
            assert_eq!(status, ConvergenceStatus::Failed);
        }
        other => panic!("expected DatasetNotConverged, got {other:?}"),
    }
}

#[test]
fn mixed_converged_and_unresolved_rejected_at_first_bad() {
    let datasets = vec![
        standard_converged_dataset("ds-good", 100_000.0),
        build_dataset(
            "ds-bad",
            200_000.0,
            0.0,
            ConvergenceStatus::Unresolved,
            &[-5.0, 0.0, 5.0],
        ),
        standard_converged_dataset("ds-after", 300_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);

    let err = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap_err();
    match err {
        XfoilEvidenceJsonError::DatasetNotConverged {
            index,
            dataset_id,
            status,
        } => {
            assert_eq!(index, 1);
            assert_eq!(dataset_id, "ds-bad");
            assert_eq!(status, ConvergenceStatus::Unresolved);
        }
        other => panic!("expected DatasetNotConverged, got {other:?}"),
    }
}

// ── Test 5: mixed Mach rejection ─────────────────────────────────────────────

#[test]
fn mixed_mach_rejected_via_json() {
    let datasets = vec![
        build_dataset(
            "ds-a",
            100_000.0,
            0.0,
            ConvergenceStatus::Converged,
            &[-5.0, 0.0, 5.0],
        ),
        build_dataset(
            "ds-b",
            200_000.0,
            0.3,
            ConvergenceStatus::Converged,
            &[-5.0, 0.0, 5.0],
        ),
    ];
    let json = canonical_json_from_datasets(datasets);

    let err = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap_err();
    match err {
        XfoilEvidenceJsonError::InconsistentMach {
            index,
            mach,
            expected_mach,
        } => {
            assert_eq!(index, 1);
            assert_eq!(mach.to_bits(), 0.3_f64.to_bits());
            assert_eq!(expected_mach.to_bits(), 0.0_f64.to_bits());
        }
        other => panic!("expected InconsistentMach, got {other:?}"),
    }
}

// ── Test 6: duplicate Reynolds rejection ─────────────────────────────────────

#[test]
fn duplicate_reynolds_rejected_via_json() {
    let json = serde_json::json!([
        canonical_dataset_json("ds-a", 100_000.0, 0.0, "converged"),
        canonical_dataset_json("ds-b", 100_000.0, 0.0, "converged"),
    ]);
    let json_str = serde_json::to_string_pretty(&json).unwrap();

    let err = build_xfoil_reynolds_polar_family_from_json(json_str.as_bytes()).unwrap_err();
    match err {
        XfoilEvidenceJsonError::DuplicateReynolds {
            previous_index,
            index,
            reynolds,
        } => {
            assert_eq!(previous_index, 0);
            assert_eq!(index, 1);
            assert_eq!(reynolds, 100_000.0);
        }
        other => panic!("expected DuplicateReynolds, got {other:?}"),
    }
}

#[test]
fn decreasing_reynolds_rejected_via_json() {
    let json = serde_json::json!([
        canonical_dataset_json("ds-a", 300_000.0, 0.0, "converged"),
        canonical_dataset_json("ds-b", 100_000.0, 0.0, "converged"),
    ]);
    let json_str = serde_json::to_string_pretty(&json).unwrap();

    let err = build_xfoil_reynolds_polar_family_from_json(json_str.as_bytes()).unwrap_err();
    assert!(matches!(
        err,
        XfoilEvidenceJsonError::ReynoldsNotIncreasing {
            previous_index: 0,
            index: 1,
            ..
        }
    ));
}

// ── Test 7: preservation of alpha/cl/cd/cm ───────────────────────────────────

#[test]
fn alpha_cl_cd_cm_preserved_exactly_via_json() {
    let dataset = standard_converged_dataset("ds-preserve", 200_000.0);
    let source_samples: Vec<_> = dataset
        .import()
        .samples()
        .iter()
        .map(|s| (s.alpha_rad(), s.cl(), s.cd(), s.cm()))
        .collect();

    let json = canonical_json_from_datasets(vec![dataset]);
    let runtime = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap();

    let samples = runtime.family().nodes()[0].table().samples();
    assert_eq!(samples.len(), source_samples.len());

    for (i, sample) in samples.iter().enumerate() {
        let (expected_alpha, expected_cl, expected_cd, expected_cm) = source_samples[i];
        assert_eq!(
            sample.alpha_rad.to_bits(),
            expected_alpha.to_bits(),
            "alpha_rad mismatch at {i}"
        );
        assert_eq!(
            sample.cl.to_bits(),
            expected_cl.to_bits(),
            "cl mismatch at {i}"
        );
        assert_eq!(
            sample.cd.to_bits(),
            expected_cd.to_bits(),
            "cd mismatch at {i}"
        );
        assert_eq!(
            sample.cm.to_bits(),
            expected_cm.to_bits(),
            "cm mismatch at {i}"
        );
    }
}

#[test]
fn native_alpha_grids_preserved_independent() {
    let ds_fine = build_dataset(
        "ds-fine",
        100_000.0,
        0.0,
        ConvergenceStatus::Converged,
        &[-5.0, -2.5, 0.0, 2.5, 5.0, 7.5, 10.0],
    );
    let ds_coarse = build_dataset(
        "ds-coarse",
        200_000.0,
        0.0,
        ConvergenceStatus::Converged,
        &[-5.0, 5.0],
    );

    let json = canonical_json_from_datasets(vec![ds_fine, ds_coarse]);
    let runtime = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap();

    assert_eq!(runtime.family().nodes()[0].table().samples().len(), 7);
    assert_eq!(runtime.family().nodes()[1].table().samples().len(), 2);
}

// ── Test 8: deterministic repeated result ────────────────────────────────────

#[test]
fn deterministic_repeated_result() {
    let datasets = vec![
        standard_converged_dataset("ds-a", 100_000.0),
        standard_converged_dataset("ds-b", 300_000.0),
        standard_converged_dataset("ds-c", 500_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);
    let json_bytes = json.as_bytes();

    let result_a = build_xfoil_reynolds_polar_family_from_json(json_bytes).unwrap();
    let result_b = build_xfoil_reynolds_polar_family_from_json(json_bytes).unwrap();

    assert_eq!(result_a.mach().to_bits(), result_b.mach().to_bits());
    assert_eq!(result_a.family(), result_b.family());
}

#[test]
fn deterministic_across_str_and_bytes() {
    let datasets = vec![
        standard_converged_dataset("ds-a", 100_000.0),
        standard_converged_dataset("ds-b", 300_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);

    let from_bytes = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap();
    let from_str = build_xfoil_reynolds_polar_family_from_json_str(&json).unwrap();

    assert_eq!(from_bytes.mach().to_bits(), from_str.mach().to_bits());
    assert_eq!(from_bytes.family(), from_str.family());
}

// ── Cross-validation: JSON bridge matches direct campaign builder ────────────

#[test]
fn json_bridge_matches_direct_campaign_builder() {
    let datasets = vec![
        standard_converged_dataset("ds-a", 100_000.0),
        standard_converged_dataset("ds-b", 300_000.0),
        standard_converged_dataset("ds-c", 500_000.0),
    ];

    let campaign = XfoilEvidenceCampaignBuilder::new(datasets).build().unwrap();
    let direct = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    let json = campaign.to_polar_datasets_json_pretty();
    let via_json = build_xfoil_reynolds_polar_family_from_json(json.as_bytes()).unwrap();

    assert_eq!(direct.mach().to_bits(), via_json.mach().to_bits());
    assert_eq!(direct.family(), via_json.family());
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Build a minimal canonical dataset JSON object directly (without going through M2.9B).
fn canonical_dataset_json(
    id: &str,
    reynolds: f64,
    mach: f64,
    convergence_status: &str,
) -> serde_json::Value {
    serde_json::json!({
        "id": id,
        "evidence_class": "generated_solver",
        "flow_conditions": {
            "reynolds": reynolds,
            "mach": mach,
            "density_kg_m3": null,
            "dynamic_viscosity_pa_s": null,
            "kinematic_viscosity_m2_s": null
        },
        "transition": {
            "assumptions": null,
            "ncrit": null,
            "forced_transition_upper_x_over_c": null,
            "forced_transition_lower_x_over_c": null
        },
        "method": {
            "id": format!("method-{id}"),
            "convergence_status": convergence_status,
            "solver_or_tool": null,
            "exact_version": null,
            "command_or_config": null
        },
        "source_ids": [SOURCE_ID],
        "samples": [
            {"alpha_rad": -0.08726646259971647, "cl": -0.2, "cd": 0.01, "cm": -0.02},
            {"alpha_rad": 0.0, "cl": 0.05, "cd": 0.011, "cm": -0.025},
            {"alpha_rad": 0.08726646259971647, "cl": 0.3, "cd": 0.012, "cm": -0.03}
        ],
        "notes": null
    })
}
