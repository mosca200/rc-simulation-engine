//! M2.9H — XFOIL evidence to Reynolds polar family bridge integration tests.
//!
//! All fixtures use synthetic XFOIL polar data. No Clark Y or LT-40
//! aerodynamic data appears here.

use model::{
    ConvergenceStatus, MetadataBuilder, XfoilEvidenceCampaign, XfoilEvidenceCampaignBuilder,
    XfoilEvidenceDataset, XfoilEvidenceDatasetBuilder, XfoilRuntimePolarFamilyError,
    build_xfoil_reynolds_polar_family, parse_xfoil_polar,
};
use sim_core::{PolarSample, PolarTable};

const SOURCE_ID: &str = "synthetic-m2-9h-source";

fn build_dataset(
    dataset_id: &str,
    reynolds: f64,
    mach: f64,
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
    .notes("Synthetic M2.9H test fixture")
    .build()
    .unwrap()
}

fn build_campaign(datasets: Vec<XfoilEvidenceDataset>) -> XfoilEvidenceCampaign {
    XfoilEvidenceCampaignBuilder::new(datasets).build().unwrap()
}

fn standard_converged_dataset(dataset_id: &str, reynolds: f64) -> XfoilEvidenceDataset {
    build_dataset(
        dataset_id,
        reynolds,
        0.0,
        ConvergenceStatus::Converged,
        &[-5.0, 0.0, 5.0, 10.0],
    )
}

// ── Test 1: single converged dataset converts correctly ─────────────────────

#[test]
fn single_converged_dataset_converts_correctly() {
    let dataset = standard_converged_dataset("ds-a", 100_000.0);
    let campaign = build_campaign(vec![dataset]);

    let result = build_xfoil_reynolds_polar_family(&campaign);
    assert!(result.is_ok(), "expected Ok, got {result:?}");

    let runtime = result.unwrap();
    assert_eq!(runtime.family().nodes().len(), 1);
    assert_eq!(runtime.mach(), 0.0);
}

// ── Test 2: multiple converged datasets convert correctly ───────────────────

#[test]
fn multiple_converged_datasets_convert_correctly() {
    let campaign = build_campaign(vec![
        standard_converged_dataset("ds-low", 100_000.0),
        standard_converged_dataset("ds-mid", 300_000.0),
        standard_converged_dataset("ds-high", 500_000.0),
    ]);

    let result = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    assert_eq!(result.family().nodes().len(), 3);
}

// ── Test 3: Reynolds numbers preserved exactly ──────────────────────────────

#[test]
fn reynolds_numbers_preserved_exactly() {
    let reynolds_values = [100_000.0, 250_000.0, 500_000.0];
    let datasets: Vec<_> = reynolds_values
        .iter()
        .enumerate()
        .map(|(i, &re)| standard_converged_dataset(&format!("ds-{i}"), re))
        .collect();
    let campaign = build_campaign(datasets);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    for (node, &expected_re) in runtime.family().nodes().iter().zip(&reynolds_values) {
        assert_eq!(
            node.reynolds_number().to_bits(),
            expected_re.to_bits(),
            "Reynolds mismatch: got {} expected {}",
            node.reynolds_number(),
            expected_re
        );
    }
}

// ── Test 4: alpha samples preserved exactly ─────────────────────────────────

#[test]
fn alpha_samples_preserved_exactly() {
    let alpha_degrees = [-5.0, 0.0, 5.0, 10.0];
    let dataset = standard_converged_dataset("ds-alpha", 100_000.0);
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let samples = runtime.family().nodes()[0].table().samples();

    assert_eq!(samples.len(), alpha_degrees.len());
    for (sample, &deg) in samples.iter().zip(&alpha_degrees) {
        let expected_rad = deg * std::f64::consts::PI / 180.0;
        assert_eq!(
            sample.alpha_rad.to_bits(),
            expected_rad.to_bits(),
            "alpha_rad mismatch at degree {deg}"
        );
    }
}

// ── Test 5: CL preserved exactly ────────────────────────────────────────────

#[test]
fn cl_preserved_exactly() {
    let dataset = standard_converged_dataset("ds-cl", 100_000.0);
    let source_cls: Vec<f64> = dataset.import().samples().iter().map(|s| s.cl()).collect();
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let samples = runtime.family().nodes()[0].table().samples();

    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(
            sample.cl.to_bits(),
            source_cls[index].to_bits(),
            "CL mismatch at index {index}"
        );
    }
}

// ── Test 6: CD preserved exactly ────────────────────────────────────────────

#[test]
fn cd_preserved_exactly() {
    let dataset = standard_converged_dataset("ds-cd", 100_000.0);
    let source_cds: Vec<f64> = dataset.import().samples().iter().map(|s| s.cd()).collect();
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let samples = runtime.family().nodes()[0].table().samples();

    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(
            sample.cd.to_bits(),
            source_cds[index].to_bits(),
            "CD mismatch at index {index}"
        );
    }
}

// ── Test 7: CM preserved exactly ────────────────────────────────────────────

#[test]
fn cm_preserved_exactly() {
    let dataset = standard_converged_dataset("ds-cm", 100_000.0);
    let source_cms: Vec<f64> = dataset.import().samples().iter().map(|s| s.cm()).collect();
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let samples = runtime.family().nodes()[0].table().samples();

    for (index, sample) in samples.iter().enumerate() {
        assert_eq!(
            sample.cm.to_bits(),
            source_cms[index].to_bits(),
            "CM mismatch at index {index}"
        );
    }
}

// ── Test 8: independent alpha grids remain independent ──────────────────────

#[test]
fn independent_alpha_grids_remain_independent() {
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
    let campaign = build_campaign(vec![ds_fine, ds_coarse]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let nodes = runtime.family().nodes();

    assert_eq!(nodes[0].table().samples().len(), 7);
    assert_eq!(nodes[1].table().samples().len(), 2);
}

// ── Test 9: Mach preserved ──────────────────────────────────────────────────

#[test]
fn mach_preserved() {
    let dataset = build_dataset(
        "ds-mach",
        100_000.0,
        0.3,
        ConvergenceStatus::Converged,
        &[-5.0, 0.0, 5.0],
    );
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    assert_eq!(runtime.mach().to_bits(), 0.3_f64.to_bits());
}

// ── Test 10: equal Mach accepted ────────────────────────────────────────────

#[test]
fn equal_mach_accepted() {
    let mach = 0.05;
    let datasets = vec![
        build_dataset(
            "ds-a",
            100_000.0,
            mach,
            ConvergenceStatus::Converged,
            &[-5.0, 0.0, 5.0],
        ),
        build_dataset(
            "ds-b",
            200_000.0,
            mach,
            ConvergenceStatus::Converged,
            &[-5.0, 0.0, 5.0],
        ),
    ];
    let campaign = build_campaign(datasets);

    let result = build_xfoil_reynolds_polar_family(&campaign);
    assert!(result.is_ok(), "expected Ok for equal Mach, got {result:?}");
    assert_eq!(result.unwrap().mach().to_bits(), mach.to_bits());
}

// ── Test 11: differing Mach rejected ────────────────────────────────────────

#[test]
fn differing_mach_rejected() {
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
    let campaign = build_campaign(datasets);

    let err = build_xfoil_reynolds_polar_family(&campaign).unwrap_err();
    match err {
        XfoilRuntimePolarFamilyError::InconsistentMach {
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

// ── Test 12: Unresolved dataset rejected ────────────────────────────────────

#[test]
fn unresolved_dataset_rejected() {
    let dataset = build_dataset(
        "ds-unresolved",
        100_000.0,
        0.0,
        ConvergenceStatus::Unresolved,
        &[-5.0, 0.0, 5.0],
    );
    let campaign = build_campaign(vec![dataset]);

    let err = build_xfoil_reynolds_polar_family(&campaign).unwrap_err();
    match err {
        XfoilRuntimePolarFamilyError::DatasetNotConverged { status, .. } => {
            assert_eq!(status, ConvergenceStatus::Unresolved);
        }
        other => panic!("expected DatasetNotConverged, got {other:?}"),
    }
}

// ── Test 13: Failed dataset rejected ────────────────────────────────────────

#[test]
fn failed_dataset_rejected() {
    let dataset = build_dataset(
        "ds-failed",
        100_000.0,
        0.0,
        ConvergenceStatus::Failed,
        &[-5.0, 0.0, 5.0],
    );
    let campaign = build_campaign(vec![dataset]);

    let err = build_xfoil_reynolds_polar_family(&campaign).unwrap_err();
    match err {
        XfoilRuntimePolarFamilyError::DatasetNotConverged { status, .. } => {
            assert_eq!(status, ConvergenceStatus::Failed);
        }
        other => panic!("expected DatasetNotConverged, got {other:?}"),
    }
}

// ── Test 14: rejection identifies dataset index/id/status ───────────────────

#[test]
fn rejection_identifies_dataset_index_id_status() {
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
    let campaign = build_campaign(datasets);

    let err = build_xfoil_reynolds_polar_family(&campaign).unwrap_err();
    match err {
        XfoilRuntimePolarFamilyError::DatasetNotConverged {
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

// ── Test 15: sampling at exact Reynolds reproduces PolarTable result ────────

#[test]
fn sampling_at_exact_reynolds_reproduces_polar_table() {
    let alpha_deg = 2.5;
    let alpha_rad = alpha_deg * std::f64::consts::PI / 180.0;
    let reynolds = 200_000.0;

    let dataset = build_dataset(
        "ds-sample",
        reynolds,
        0.0,
        ConvergenceStatus::Converged,
        &[-5.0, 0.0, 5.0, 10.0],
    );

    let original_samples: Vec<PolarSample> = dataset
        .import()
        .samples()
        .iter()
        .map(|s| PolarSample {
            alpha_rad: s.alpha_rad(),
            cl: s.cl(),
            cd: s.cd(),
            cm: s.cm(),
        })
        .collect();
    let original_table = PolarTable::new(original_samples).unwrap();
    let expected = original_table.sample_clamped(alpha_rad);

    let campaign = build_campaign(vec![dataset]);
    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    let sample = runtime.family().sample(reynolds, alpha_rad);
    assert_eq!(sample.coefficients.cl, expected.cl);
    assert_eq!(sample.coefficients.cd, expected.cd);
    assert_eq!(sample.coefficients.cm, expected.cm);
}

// ── Test 16: interpolation between two Reynolds nodes ───────────────────────

#[test]
fn interpolation_between_two_reynolds_nodes() {
    let alpha_rad = 0.0_f64;
    let re_low = 100_000.0;
    let re_high = 300_000.0;
    let re_mid = 200_000.0;

    let campaign = build_campaign(vec![
        standard_converged_dataset("ds-low", re_low),
        standard_converged_dataset("ds-high", re_high),
    ]);
    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    let sample = runtime.family().sample(re_mid, alpha_rad);
    assert_eq!(
        sample.range_status,
        sim_core::ReynoldsRangeStatus::ExactOrInRange
    );
    assert!(sample.interpolation_fraction > 0.0);
    assert!(sample.interpolation_fraction < 1.0);
}

// ── Test 17: below-range Reynolds behavior unchanged ────────────────────────

#[test]
fn below_range_reynolds_clamps_to_lowest_node() {
    let campaign = build_campaign(vec![
        standard_converged_dataset("ds-low", 100_000.0),
        standard_converged_dataset("ds-high", 300_000.0),
    ]);
    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    let sample = runtime.family().sample(50_000.0, 0.0);
    assert_eq!(
        sample.range_status,
        sim_core::ReynoldsRangeStatus::BelowRange
    );

    let expected = runtime.family().nodes()[0].table().sample_clamped(0.0);
    assert_eq!(sample.coefficients.cl, expected.cl);
    assert_eq!(sample.coefficients.cd, expected.cd);
    assert_eq!(sample.coefficients.cm, expected.cm);
}

// ── Test 18: above-range Reynolds behavior unchanged ────────────────────────

#[test]
fn above_range_reynolds_clamps_to_highest_node() {
    let campaign = build_campaign(vec![
        standard_converged_dataset("ds-low", 100_000.0),
        standard_converged_dataset("ds-high", 300_000.0),
    ]);
    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    let sample = runtime.family().sample(500_000.0, 0.0);
    assert_eq!(
        sample.range_status,
        sim_core::ReynoldsRangeStatus::AboveRange
    );

    let expected = runtime
        .family()
        .nodes()
        .last()
        .unwrap()
        .table()
        .sample_clamped(0.0);
    assert_eq!(sample.coefficients.cl, expected.cl);
    assert_eq!(sample.coefficients.cd, expected.cd);
    assert_eq!(sample.coefficients.cm, expected.cm);
}

// ── Test 19: no fitting/resampling occurs ───────────────────────────────────

#[test]
fn no_fitting_or_resampling_occurs() {
    let alpha_degrees = [-5.0, 0.0, 5.0, 10.0];
    let dataset = standard_converged_dataset("ds-nofit", 100_000.0);
    let source_samples: Vec<_> = dataset
        .import()
        .samples()
        .iter()
        .map(|s| (s.alpha_rad(), s.cl(), s.cd(), s.cm()))
        .collect();
    let campaign = build_campaign(vec![dataset]);

    let runtime = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let samples = runtime.family().nodes()[0].table().samples();

    assert_eq!(samples.len(), alpha_degrees.len());

    for (index, sample) in samples.iter().enumerate() {
        let (expected_alpha, expected_cl, expected_cd, expected_cm) = source_samples[index];
        assert_eq!(sample.alpha_rad.to_bits(), expected_alpha.to_bits());
        assert_eq!(sample.cl.to_bits(), expected_cl.to_bits());
        assert_eq!(sample.cd.to_bits(), expected_cd.to_bits());
        assert_eq!(sample.cm.to_bits(), expected_cm.to_bits());
    }
}

// ── Test 20: repeated conversion produces identical result ──────────────────

#[test]
fn repeated_conversion_produces_identical_result() {
    let campaign = build_campaign(vec![
        standard_converged_dataset("ds-a", 100_000.0),
        standard_converged_dataset("ds-b", 300_000.0),
        standard_converged_dataset("ds-c", 500_000.0),
    ]);

    let result_a = build_xfoil_reynolds_polar_family(&campaign).unwrap();
    let result_b = build_xfoil_reynolds_polar_family(&campaign).unwrap();

    assert_eq!(result_a.mach().to_bits(), result_b.mach().to_bits());
    assert_eq!(result_a.family(), result_b.family());
}
