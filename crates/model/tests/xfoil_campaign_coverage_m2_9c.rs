//! M2.9C deterministic XFOIL campaign and coverage integration tests.
//!
//! Every fixture is synthetic test evidence. No Clark Y or LT-40 aerodynamic
//! data appears here.

use model::{
    AerodynamicEvidenceClass, AerodynamicEvidenceLoader, ConvergenceStatus, MetadataBuilder,
    XfoilCampaignCoverageBlocker, XfoilCampaignCoverageRequest, XfoilCampaignCoverageStatus,
    XfoilEvidenceCampaign, XfoilEvidenceCampaignBuilder, XfoilEvidenceCampaignError,
    XfoilEvidenceDataset, XfoilEvidenceDatasetBuilder, parse_xfoil_polar,
};
use serde_json::{Value, json};

const SOURCE_ID: &str = "synthetic-xfoil-campaign-source";

fn rad(degrees: f64) -> f64 {
    degrees * std::f64::consts::PI / 180.0
}

fn synthetic_dataset(
    dataset_id: &str,
    reynolds: f64,
    status: ConvergenceStatus,
    alpha_degrees: &[f64],
) -> XfoilEvidenceDataset {
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

    let metadata = MetadataBuilder::new(reynolds, 0.03)
        .solver_name("Synthetic XFOIL test double")
        .solver_version("test-only-1")
        .command_or_config(format!("SYNTHETIC RE {reynolds:.0}"))
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
    .notes("Synthetic M2.9C campaign fixture")
    .build()
    .unwrap()
}

fn standard_dataset(
    dataset_id: &str,
    reynolds: f64,
    status: ConvergenceStatus,
) -> XfoilEvidenceDataset {
    synthetic_dataset(dataset_id, reynolds, status, &[-5.0, 0.0, 10.0])
}

fn campaign(datasets: Vec<XfoilEvidenceDataset>) -> XfoilEvidenceCampaign {
    XfoilEvidenceCampaignBuilder::new(datasets).build().unwrap()
}

fn standard_campaign() -> XfoilEvidenceCampaign {
    campaign(vec![
        standard_dataset("z-low", 100_000.0, ConvergenceStatus::Converged),
        standard_dataset("a-middle", 200_000.0, ConvergenceStatus::Converged),
        standard_dataset("m-high", 300_000.0, ConvergenceStatus::Converged),
    ])
}

fn request(
    reynolds_min: f64,
    reynolds_max: f64,
    alpha_min_degrees: f64,
    alpha_max_degrees: f64,
    require_converged: bool,
) -> XfoilCampaignCoverageRequest {
    XfoilCampaignCoverageRequest::new(
        reynolds_min,
        reynolds_max,
        rad(alpha_min_degrees),
        rad(alpha_max_degrees),
        require_converged,
    )
    .unwrap()
}

#[test]
fn one_valid_dataset_campaign_builds() {
    let campaign = campaign(vec![standard_dataset(
        "only-node",
        200_000.0,
        ConvergenceStatus::Converged,
    )]);
    assert_eq!(campaign.dataset_count(), 1);
    assert_eq!(campaign.datasets()[0].dataset_id(), "only-node");
}

#[test]
fn increasing_reynolds_campaign_preserves_caller_order() {
    let campaign = standard_campaign();
    let ids: Vec<_> = campaign
        .datasets()
        .iter()
        .map(XfoilEvidenceDataset::dataset_id)
        .collect();
    assert_eq!(ids, ["z-low", "a-middle", "m-high"]);
    assert_eq!(
        campaign
            .datasets()
            .iter()
            .map(XfoilEvidenceDataset::reynolds)
            .collect::<Vec<_>>(),
        [100_000.0, 200_000.0, 300_000.0]
    );
}

#[test]
fn empty_campaign_is_rejected() {
    let error = XfoilEvidenceCampaignBuilder::new(Vec::new())
        .build()
        .unwrap_err();
    assert_eq!(error, XfoilEvidenceCampaignError::EmptyCampaign);
}

#[test]
fn duplicate_dataset_id_is_rejected() {
    let error = XfoilEvidenceCampaignBuilder::new(vec![
        standard_dataset("duplicate", 100_000.0, ConvergenceStatus::Converged),
        standard_dataset("duplicate", 200_000.0, ConvergenceStatus::Converged),
    ])
    .build()
    .unwrap_err();
    assert_eq!(
        error,
        XfoilEvidenceCampaignError::DuplicateDatasetId {
            index: 1,
            dataset_id: "duplicate".to_owned(),
        }
    );
}

#[test]
fn duplicate_reynolds_is_rejected() {
    let error = XfoilEvidenceCampaignBuilder::new(vec![
        standard_dataset("first", 200_000.0, ConvergenceStatus::Converged),
        standard_dataset("second", 200_000.0, ConvergenceStatus::Converged),
    ])
    .build()
    .unwrap_err();
    assert_eq!(
        error,
        XfoilEvidenceCampaignError::DuplicateReynolds {
            previous_index: 0,
            index: 1,
            reynolds: 200_000.0,
        }
    );
}

#[test]
fn decreasing_reynolds_is_rejected_without_sorting() {
    let error = XfoilEvidenceCampaignBuilder::new(vec![
        standard_dataset("higher", 500_000.0, ConvergenceStatus::Converged),
        standard_dataset("lower", 300_000.0, ConvergenceStatus::Converged),
    ])
    .build()
    .unwrap_err();
    assert_eq!(
        error,
        XfoilEvidenceCampaignError::ReynoldsNotIncreasing {
            previous_index: 0,
            index: 1,
            previous_reynolds: 500_000.0,
            reynolds: 300_000.0,
        }
    );
}

#[test]
fn minimum_and_maximum_reynolds_are_exact() {
    let campaign = standard_campaign();
    assert_eq!(campaign.minimum_reynolds(), 100_000.0);
    assert_eq!(campaign.maximum_reynolds(), 300_000.0);
}

#[test]
fn reynolds_request_strictly_inside_campaign_range_is_covered() {
    let coverage =
        standard_campaign().audit_coverage(&request(125_000.0, 275_000.0, -4.0, 8.0, true));
    assert!(coverage.is_qualified());
    assert_eq!(coverage.status(), XfoilCampaignCoverageStatus::Qualified);
    assert_eq!(coverage.campaign_minimum_reynolds(), 100_000.0);
    assert_eq!(coverage.campaign_maximum_reynolds(), 300_000.0);
}

#[test]
fn lower_reynolds_gap_has_typed_blocker() {
    let coverage =
        standard_campaign().audit_coverage(&request(50_000.0, 250_000.0, -4.0, 8.0, true));
    assert_eq!(
        coverage.blockers(),
        &[
            XfoilCampaignCoverageBlocker::ReynoldsCoverageBelowRequired {
                campaign_minimum_reynolds: 100_000.0,
                required_minimum_reynolds: 50_000.0,
            }
        ]
    );
}

#[test]
fn upper_reynolds_gap_has_typed_blocker() {
    let coverage =
        standard_campaign().audit_coverage(&request(150_000.0, 350_000.0, -4.0, 8.0, true));
    assert_eq!(
        coverage.blockers(),
        &[
            XfoilCampaignCoverageBlocker::ReynoldsCoverageAboveRequired {
                campaign_maximum_reynolds: 300_000.0,
                required_maximum_reynolds: 350_000.0,
            }
        ]
    );
}

#[test]
fn both_reynolds_gaps_are_reported_lower_then_upper() {
    let coverage =
        standard_campaign().audit_coverage(&request(50_000.0, 350_000.0, -4.0, 8.0, true));
    assert!(matches!(
        coverage.blockers()[0],
        XfoilCampaignCoverageBlocker::ReynoldsCoverageBelowRequired { .. }
    ));
    assert!(matches!(
        coverage.blockers()[1],
        XfoilCampaignCoverageBlocker::ReynoldsCoverageAboveRequired { .. }
    ));
    assert_eq!(coverage.blockers().len(), 2);
}

#[test]
fn every_dataset_covering_alpha_range_passes() {
    let coverage =
        standard_campaign().audit_coverage(&request(100_000.0, 300_000.0, -5.0, 10.0, true));
    assert!(coverage.is_qualified());
    assert!(coverage.datasets().iter().all(|dataset| {
        dataset.covers_required_alpha_min() && dataset.covers_required_alpha_max()
    }));
    assert_eq!(coverage.datasets()[0].alpha_min_rad(), rad(-5.0));
    assert_eq!(coverage.datasets()[2].alpha_max_rad(), rad(10.0));
}

#[test]
fn missing_lower_alpha_names_exact_dataset() {
    let campaign = campaign(vec![
        standard_dataset("full", 100_000.0, ConvergenceStatus::Converged),
        synthetic_dataset(
            "short-low",
            200_000.0,
            ConvergenceStatus::Converged,
            &[-2.0, 0.0, 10.0],
        ),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 200_000.0, -4.0, 8.0, true));
    assert!(matches!(
        &coverage.blockers()[0],
        XfoilCampaignCoverageBlocker::DatasetAlphaBelowRequired {
            index: 1,
            dataset_id,
            ..
        } if dataset_id == "short-low"
    ));
    assert_eq!(coverage.blockers().len(), 1);
}

#[test]
fn missing_upper_alpha_names_exact_dataset() {
    let campaign = campaign(vec![
        synthetic_dataset(
            "short-high",
            100_000.0,
            ConvergenceStatus::Converged,
            &[-5.0, 0.0, 6.0],
        ),
        standard_dataset("full", 200_000.0, ConvergenceStatus::Converged),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 200_000.0, -4.0, 8.0, true));
    assert!(matches!(
        &coverage.blockers()[0],
        XfoilCampaignCoverageBlocker::DatasetAlphaAboveRequired {
            index: 0,
            dataset_id,
            ..
        } if dataset_id == "short-high"
    ));
    assert_eq!(coverage.blockers().len(), 1);
}

#[test]
fn different_alpha_grids_are_audited_independently() {
    let campaign = campaign(vec![
        synthetic_dataset(
            "upper-gap",
            100_000.0,
            ConvergenceStatus::Converged,
            &[-6.0, 0.0, 7.0],
        ),
        synthetic_dataset(
            "full-grid",
            200_000.0,
            ConvergenceStatus::Converged,
            &[-4.0, 1.0, 8.0],
        ),
        synthetic_dataset(
            "lower-gap",
            300_000.0,
            ConvergenceStatus::Converged,
            &[-3.0, 2.0, 9.0],
        ),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 300_000.0, -4.0, 8.0, true));
    assert_eq!(coverage.datasets().len(), 3);
    assert_eq!(
        coverage
            .datasets()
            .iter()
            .map(|item| (
                item.index(),
                item.covers_required_alpha_min(),
                item.covers_required_alpha_max()
            ))
            .collect::<Vec<_>>(),
        [(0, true, false), (1, true, true), (2, false, true)]
    );
}

#[test]
fn unresolved_blocks_when_convergence_is_required() {
    let campaign = campaign(vec![
        standard_dataset("unresolved", 100_000.0, ConvergenceStatus::Unresolved),
        standard_dataset("converged", 300_000.0, ConvergenceStatus::Converged),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 300_000.0, -4.0, 8.0, true));
    assert!(matches!(
        &coverage.blockers()[0],
        XfoilCampaignCoverageBlocker::DatasetNotConverged {
            index: 0,
            dataset_id,
            status: ConvergenceStatus::Unresolved,
        } if dataset_id == "unresolved"
    ));
}

#[test]
fn failed_blocks_when_convergence_is_required() {
    let campaign = campaign(vec![
        standard_dataset("converged", 100_000.0, ConvergenceStatus::Converged),
        standard_dataset("failed", 300_000.0, ConvergenceStatus::Failed),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 300_000.0, -4.0, 8.0, true));
    assert!(matches!(
        coverage.blockers()[0],
        XfoilCampaignCoverageBlocker::DatasetNotConverged {
            status: ConvergenceStatus::Failed,
            ..
        }
    ));
}

#[test]
fn convergence_is_preserved_but_not_blocking_when_not_required() {
    let campaign = campaign(vec![
        standard_dataset("unresolved", 100_000.0, ConvergenceStatus::Unresolved),
        standard_dataset("failed", 200_000.0, ConvergenceStatus::Failed),
    ]);
    let coverage = campaign.audit_coverage(&request(100_000.0, 200_000.0, -4.0, 8.0, false));
    assert!(coverage.is_qualified());
    assert!(coverage.blockers().is_empty());
    assert_eq!(
        coverage
            .datasets()
            .iter()
            .map(|item| item.convergence_status())
            .collect::<Vec<_>>(),
        [ConvergenceStatus::Unresolved, ConvergenceStatus::Failed]
    );
}

#[test]
fn simultaneous_blockers_have_exact_deterministic_order() {
    let campaign = campaign(vec![
        synthetic_dataset(
            "first",
            100_000.0,
            ConvergenceStatus::Unresolved,
            &[-2.0, 0.0, 6.0],
        ),
        synthetic_dataset(
            "second",
            300_000.0,
            ConvergenceStatus::Failed,
            &[-5.0, 0.0, 6.0],
        ),
    ]);
    let coverage = campaign.audit_coverage(&request(50_000.0, 350_000.0, -4.0, 8.0, true));
    let blockers = coverage.blockers();
    assert_eq!(blockers.len(), 7);
    assert!(matches!(
        blockers[0],
        XfoilCampaignCoverageBlocker::ReynoldsCoverageBelowRequired { .. }
    ));
    assert!(matches!(
        blockers[1],
        XfoilCampaignCoverageBlocker::ReynoldsCoverageAboveRequired { .. }
    ));
    assert!(matches!(
        blockers[2],
        XfoilCampaignCoverageBlocker::DatasetNotConverged { index: 0, .. }
    ));
    assert!(matches!(
        blockers[3],
        XfoilCampaignCoverageBlocker::DatasetAlphaBelowRequired { index: 0, .. }
    ));
    assert!(matches!(
        blockers[4],
        XfoilCampaignCoverageBlocker::DatasetAlphaAboveRequired { index: 0, .. }
    ));
    assert!(matches!(
        blockers[5],
        XfoilCampaignCoverageBlocker::DatasetNotConverged { index: 1, .. }
    ));
    assert!(matches!(
        blockers[6],
        XfoilCampaignCoverageBlocker::DatasetAlphaAboveRequired { index: 1, .. }
    ));
}

#[test]
fn polar_datasets_json_is_exact_m2_9b_json_in_order() {
    let first = standard_dataset("z-first", 100_000.0, ConvergenceStatus::Converged);
    let second = standard_dataset("a-second", 200_000.0, ConvergenceStatus::Unresolved);
    let expected = Value::Array(vec![first.to_json_value(), second.to_json_value()]);
    let campaign = campaign(vec![first, second]);
    assert_eq!(campaign.to_polar_datasets_json_value(), expected);
}

#[test]
fn repeated_campaign_serialization_is_byte_identical() {
    let first = standard_campaign().to_polar_datasets_json_pretty();
    let second = standard_campaign().to_polar_datasets_json_pretty();
    assert_eq!(first.as_bytes(), second.as_bytes());
}

#[test]
fn multi_dataset_artifact_is_accepted_by_evidence_loader() {
    let campaign = standard_campaign();
    let artifact = json!({
        "schema": "reference_aircraft_aerodynamic_evidence_v0",
        "artifact_kind": "aerodynamic_evidence_not_runtime_configuration",
        "campaign": {
            "id": "synthetic-m2-9c-loader-test",
            "classification": "synthetic_non_reference",
            "manufacturer": "Synthetic Test Manufacturer",
            "family": "Synthetic Campaign Family",
            "variant": "test-only",
            "notes": "No real-aircraft claim."
        },
        "airfoil_identity": {
            "name": "Synthetic M2.9C Test Airfoil",
            "source_ids": [SOURCE_ID],
            "notes": "Synthetic loader fixture only."
        },
        "coordinates": null,
        "provenance_sources": [{
            "id": SOURCE_ID,
            "kind": "solver_tool",
            "title": "Synthetic XFOIL campaign test source",
            "publisher": "Test suite",
            "url": "https://example.invalid/m2-9c",
            "retrieval_date": "2030-06-01",
            "sha256": null,
            "notes": "Synthetic data generated in memory for tests."
        }],
        "operating_envelope": null,
        "polar_datasets": campaign.to_polar_datasets_json_value()
    });

    let encoded = serde_json::to_string_pretty(&artifact).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(&encoded).unwrap();
    let evaluated = evidence.evaluation();
    assert_eq!(evaluated.datasets().len(), 3);
    assert!(
        evaluated
            .datasets()
            .iter()
            .all(|item| item.evidence_class() == AerodynamicEvidenceClass::GeneratedSolver)
    );
    assert!(
        evaluated
            .datasets()
            .iter()
            .all(|item| item.convergence_status() == ConvergenceStatus::Converged)
    );
    assert!(!evaluated.runtime_ready());
}

#[test]
fn campaign_api_has_no_runtime_type_dependency() {
    let source = include_str!("../src/reference_xfoil_campaign.rs");
    for forbidden in [
        "sim_core::",
        "RuntimePolar",
        "ReynoldsPolarFamily",
        "AircraftModel",
    ] {
        assert!(!source.contains(forbidden), "found forbidden {forbidden}");
    }

    let campaign = standard_campaign();
    let coverage = campaign.audit_coverage(&request(100_000.0, 300_000.0, -4.0, 8.0, true));
    assert!(coverage.is_qualified());
}

#[test]
fn request_rejects_nonfinite_reynolds_bounds() {
    for (minimum, maximum, expected) in [
        (
            f64::NAN,
            200_000.0,
            XfoilEvidenceCampaignError::RequiredReynoldsMinimumNotFinite,
        ),
        (
            f64::NEG_INFINITY,
            200_000.0,
            XfoilEvidenceCampaignError::RequiredReynoldsMinimumNotFinite,
        ),
        (
            100_000.0,
            f64::INFINITY,
            XfoilEvidenceCampaignError::RequiredReynoldsMaximumNotFinite,
        ),
        (
            100_000.0,
            f64::NAN,
            XfoilEvidenceCampaignError::RequiredReynoldsMaximumNotFinite,
        ),
    ] {
        assert_eq!(
            XfoilCampaignCoverageRequest::new(minimum, maximum, -0.1, 0.1, true).unwrap_err(),
            expected
        );
    }
}

#[test]
fn request_rejects_nonpositive_minimum_reynolds() {
    for minimum in [0.0, -1.0] {
        assert_eq!(
            XfoilCampaignCoverageRequest::new(minimum, 200_000.0, -0.1, 0.1, true).unwrap_err(),
            XfoilEvidenceCampaignError::RequiredReynoldsMinimumNotPositive
        );
    }
}

#[test]
fn request_rejects_equal_or_reversed_reynolds_bounds() {
    for (minimum, maximum) in [(100_000.0, 100_000.0), (200_000.0, 100_000.0)] {
        assert_eq!(
            XfoilCampaignCoverageRequest::new(minimum, maximum, -0.1, 0.1, true).unwrap_err(),
            XfoilEvidenceCampaignError::RequiredReynoldsBoundsNotIncreasing
        );
    }
}

#[test]
fn request_rejects_nonfinite_alpha_bounds() {
    for (minimum, maximum, expected) in [
        (
            f64::NAN,
            0.1,
            XfoilEvidenceCampaignError::RequiredAlphaMinimumNotFinite,
        ),
        (
            f64::NEG_INFINITY,
            0.1,
            XfoilEvidenceCampaignError::RequiredAlphaMinimumNotFinite,
        ),
        (
            -0.1,
            f64::INFINITY,
            XfoilEvidenceCampaignError::RequiredAlphaMaximumNotFinite,
        ),
        (
            -0.1,
            f64::NAN,
            XfoilEvidenceCampaignError::RequiredAlphaMaximumNotFinite,
        ),
    ] {
        assert_eq!(
            XfoilCampaignCoverageRequest::new(100_000.0, 200_000.0, minimum, maximum, true)
                .unwrap_err(),
            expected
        );
    }
}

#[test]
fn request_rejects_equal_or_reversed_alpha_bounds() {
    for (minimum, maximum) in [(0.1, 0.1), (0.2, 0.1)] {
        assert_eq!(
            XfoilCampaignCoverageRequest::new(100_000.0, 200_000.0, minimum, maximum, true,)
                .unwrap_err(),
            XfoilEvidenceCampaignError::RequiredAlphaBoundsNotIncreasing
        );
    }
}

#[test]
fn coverage_result_preserves_request_and_dataset_facts() {
    let request = request(125_000.0, 275_000.0, -4.0, 8.0, false);
    let coverage = standard_campaign().audit_coverage(&request);
    assert_eq!(coverage.request(), &request);
    assert_eq!(coverage.request().required_reynolds_min(), 125_000.0);
    assert_eq!(coverage.request().required_reynolds_max(), 275_000.0);
    assert_eq!(coverage.request().required_alpha_min_rad(), rad(-4.0));
    assert_eq!(coverage.request().required_alpha_max_rad(), rad(8.0));
    assert!(!coverage.request().require_converged());

    let middle = &coverage.datasets()[1];
    assert_eq!(middle.index(), 1);
    assert_eq!(middle.dataset_id(), "a-middle");
    assert_eq!(middle.method_id(), "method-a-middle");
    assert_eq!(middle.reynolds(), 200_000.0);
    assert_eq!(middle.mach(), 0.03);
    assert_eq!(middle.convergence_status(), ConvergenceStatus::Converged);
}

#[test]
fn fixture_is_explicitly_synthetic_and_contains_no_real_aircraft_claim() {
    let json = standard_campaign().to_polar_datasets_json_pretty();
    assert!(json.contains("Synthetic"));
    assert!(!json.to_ascii_lowercase().contains("clark y"));
    assert!(!json.to_ascii_lowercase().contains("lt-40"));
}
