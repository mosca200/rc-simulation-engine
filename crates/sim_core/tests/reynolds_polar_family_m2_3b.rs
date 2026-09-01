use sim_core::{
    PolarCoefficients, PolarSample, PolarTable, ReynoldsPolar, ReynoldsPolarFamily,
    ReynoldsPolarFamilyError, ReynoldsRangeStatus,
};

const TOLERANCE: f64 = 1.0e-12;

fn constant_table(cl: f64, cd: f64, cm: f64) -> PolarTable {
    PolarTable::new(vec![
        PolarSample {
            alpha_rad: -1.0,
            cl,
            cd,
            cm,
        },
        PolarSample {
            alpha_rad: 1.0,
            cl,
            cd,
            cm,
        },
    ])
    .unwrap()
}

fn linear_table(samples: &[(f64, f64, f64, f64)]) -> PolarTable {
    PolarTable::new(
        samples
            .iter()
            .map(|&(alpha_rad, cl, cd, cm)| PolarSample {
                alpha_rad,
                cl,
                cd,
                cm,
            })
            .collect(),
    )
    .unwrap()
}

fn node(reynolds_number: f64, cl: f64, cd: f64, cm: f64) -> ReynoldsPolar {
    ReynoldsPolar::new(reynolds_number, constant_table(cl, cd, cm)).unwrap()
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= TOLERANCE,
        "actual={actual:e}, expected={expected:e}"
    );
}

#[test]
fn m2_3b_01_zero_nodes_are_rejected() {
    assert_eq!(
        ReynoldsPolarFamily::new(Vec::new()),
        Err(ReynoldsPolarFamilyError::Empty)
    );
}

#[test]
fn m2_3b_02_zero_reynolds_is_rejected() {
    assert_eq!(
        ReynoldsPolar::new(0.0, constant_table(0.0, 0.0, 0.0)),
        Err(ReynoldsPolarFamilyError::NonPositiveReynoldsNumber)
    );
}

#[test]
fn m2_3b_03_negative_reynolds_is_rejected() {
    assert_eq!(
        ReynoldsPolar::new(-1.0, constant_table(0.0, 0.0, 0.0)),
        Err(ReynoldsPolarFamilyError::NonPositiveReynoldsNumber)
    );
}

#[test]
fn m2_3b_04_nonfinite_reynolds_is_rejected() {
    for reynolds_number in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            ReynoldsPolar::new(reynolds_number, constant_table(0.0, 0.0, 0.0)),
            Err(ReynoldsPolarFamilyError::NonFiniteReynoldsNumber)
        );
    }
}

#[test]
fn m2_3b_05_duplicate_reynolds_is_rejected() {
    assert_eq!(
        ReynoldsPolarFamily::new(vec![
            node(100_000.0, 0.0, 0.01, 0.0),
            node(100_000.0, 1.0, 0.02, -0.1),
        ]),
        Err(ReynoldsPolarFamilyError::DuplicateReynoldsNumber { sorted_index: 1 })
    );
}

#[test]
fn m2_3b_06_construction_canonicalizes_reynolds_order_deterministically() {
    let family = ReynoldsPolarFamily::new(vec![
        node(400_000.0, 4.0, 0.04, -0.4),
        node(100_000.0, 1.0, 0.01, -0.1),
        node(200_000.0, 2.0, 0.02, -0.2),
    ])
    .unwrap();
    assert_eq!(
        family
            .nodes()
            .iter()
            .map(ReynoldsPolar::reynolds_number)
            .collect::<Vec<_>>(),
        vec![100_000.0, 200_000.0, 400_000.0]
    );
}

#[test]
fn m2_3b_07_one_node_family_samples_that_node() {
    let family = ReynoldsPolarFamily::new(vec![node(100_000.0, 1.0, 0.02, -0.1)]).unwrap();
    let sample = family.sample(100_000.0, 0.25);
    assert_eq!(
        sample.coefficients,
        PolarCoefficients {
            cl: 1.0,
            cd: 0.02,
            cm: -0.1
        }
    );
    assert_eq!(sample.lower_reynolds.reynolds_number(), 100_000.0);
    assert_eq!(sample.upper_reynolds.reynolds_number(), 100_000.0);
    assert_eq!(sample.interpolation_fraction, 0.0);
    assert_eq!(sample.range_status, ReynoldsRangeStatus::ExactOrInRange);
}

#[test]
fn m2_3b_08_exact_reynolds_node_preserves_table_result() {
    let exact_table = linear_table(&[
        (-1.0, -2.0, 0.03, 0.2),
        (0.0, 0.25, 0.04, -0.02),
        (1.0, 3.0, 0.12, -0.3),
    ]);
    let expected = exact_table.sample_clamped(0.4);
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, -1.0, 0.01, 0.1),
        ReynoldsPolar::new(200_000.0, exact_table).unwrap(),
        node(400_000.0, 4.0, 0.08, -0.4),
    ])
    .unwrap();
    let sample = family.sample(200_000.0, 0.4);
    assert_eq!(sample.coefficients, expected);
    assert_eq!(sample.lower_reynolds.reynolds_number(), 200_000.0);
    assert_eq!(sample.upper_reynolds.reynolds_number(), 200_000.0);
    assert_eq!(sample.interpolation_fraction, 0.0);
}

#[test]
fn m2_3b_09_midpoint_is_interpolated_in_log_reynolds() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 0.0, 0.02, 0.2),
        node(400_000.0, 2.0, 0.10, -0.2),
    ])
    .unwrap();
    let sample = family.sample(200_000.0, 0.0);
    assert_close(sample.interpolation_fraction, 0.5);
    assert_close(sample.coefficients.cl, 1.0);
}

#[test]
fn m2_3b_10_arbitrary_log_reynolds_fraction_is_preserved() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100.0, 1.0, 0.02, 0.3),
        node(10_000.0, 5.0, 0.10, -0.1),
    ])
    .unwrap();
    let sample = family.sample(1_000.0, 0.0);
    assert_close(sample.interpolation_fraction, 0.5);
    assert_close(sample.coefficients.cl, 3.0);

    let quarter_reynolds = 100.0_f64 * (10_000.0_f64 / 100.0).powf(0.25);
    let quarter = family.sample(quarter_reynolds, 0.0);
    assert_close(quarter.interpolation_fraction, 0.25);
    assert_close(quarter.coefficients.cl, 2.0);

    let huge = ReynoldsPolarFamily::new(vec![
        node(100.0, -f64::MAX, f64::MAX, -f64::MAX),
        node(10_000.0, f64::MAX, f64::MAX, f64::MAX),
    ])
    .unwrap()
    .sample(1_000.0, 0.0)
    .coefficients;
    assert_eq!(huge.cl, 0.0);
    assert_eq!(huge.cd, f64::MAX);
    assert_eq!(huge.cm, 0.0);
}

#[test]
fn m2_3b_11_alpha_interpolation_remains_piecewise_linear() {
    let table = linear_table(&[(-1.0, -1.0, 0.02, 0.1), (1.0, 3.0, 0.06, -0.3)]);
    let expected = table.sample_clamped(0.5);
    let family =
        ReynoldsPolarFamily::new(vec![ReynoldsPolar::new(100_000.0, table).unwrap()]).unwrap();
    assert_eq!(family.sample(100_000.0, 0.5).coefficients, expected);
}

#[test]
fn m2_3b_12_different_alpha_grids_are_sampled_before_reynolds_interpolation() {
    let lower = linear_table(&[(-1.0, -1.0, 0.02, 0.1), (1.0, 3.0, 0.06, -0.3)]);
    let upper = linear_table(&[
        (-2.0, -8.0, 0.10, 0.8),
        (0.0, 0.0, 0.02, 0.0),
        (2.0, 4.0, 0.06, -0.4),
    ]);
    let lower_at_alpha = lower.sample_clamped(0.5);
    let upper_at_alpha = upper.sample_clamped(0.5);
    let family = ReynoldsPolarFamily::new(vec![
        ReynoldsPolar::new(100_000.0, lower).unwrap(),
        ReynoldsPolar::new(400_000.0, upper).unwrap(),
    ])
    .unwrap();
    let sample = family.sample(200_000.0, 0.5);
    assert_close(
        sample.coefficients.cl,
        (lower_at_alpha.cl + upper_at_alpha.cl) / 2.0,
    );
    assert_close(
        sample.coefficients.cd,
        (lower_at_alpha.cd + upper_at_alpha.cd) / 2.0,
    );
    assert_close(
        sample.coefficients.cm,
        (lower_at_alpha.cm + upper_at_alpha.cm) / 2.0,
    );
}

#[test]
fn m2_3b_13_below_range_clamps_and_reports_status() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 1.0, 0.02, -0.1),
        node(200_000.0, 2.0, 0.04, -0.2),
    ])
    .unwrap();
    let sample = family.sample(50_000.0, 0.0);
    assert_eq!(
        sample.coefficients,
        family.nodes()[0].table().sample_clamped(0.0)
    );
    assert_eq!(sample.range_status, ReynoldsRangeStatus::BelowRange);
    assert_eq!(sample.lower_reynolds.reynolds_number(), 100_000.0);
    assert_eq!(sample.upper_reynolds.reynolds_number(), 100_000.0);
}

#[test]
fn m2_3b_14_above_range_clamps_and_reports_status() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 1.0, 0.02, -0.1),
        node(200_000.0, 2.0, 0.04, -0.2),
    ])
    .unwrap();
    let sample = family.sample(400_000.0, 0.0);
    assert_eq!(
        sample.coefficients,
        family.nodes()[1].table().sample_clamped(0.0)
    );
    assert_eq!(sample.range_status, ReynoldsRangeStatus::AboveRange);
    assert_eq!(sample.lower_reynolds.reynolds_number(), 200_000.0);
    assert_eq!(sample.upper_reynolds.reynolds_number(), 200_000.0);
}

#[test]
fn m2_3b_15_out_of_range_sampling_never_extrapolates() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 1.0, 0.02, -0.1),
        node(200_000.0, 2.0, 0.04, -0.2),
    ])
    .unwrap();
    assert_eq!(
        family.sample(1.0, 0.0).coefficients,
        family.sample(100_000.0, 0.0).coefficients
    );
    assert_eq!(
        family.sample(1.0e12, 0.0).coefficients,
        family.sample(200_000.0, 0.0).coefficients
    );
}

#[test]
fn m2_3b_16_drag_coefficient_uses_log_reynolds_interpolation() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 0.0, 0.02, 0.0),
        node(400_000.0, 0.0, 0.10, 0.0),
    ])
    .unwrap();
    assert_close(family.sample(200_000.0, 0.0).coefficients.cd, 0.06);
}

#[test]
fn m2_3b_17_pitching_moment_uses_log_reynolds_interpolation() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 0.0, 0.02, 0.2),
        node(400_000.0, 0.0, 0.02, -0.4),
    ])
    .unwrap();
    assert_close(family.sample(200_000.0, 0.0).coefficients.cm, -0.1);
}

#[test]
fn m2_3b_18_exact_alpha_sample_is_preserved() {
    let table = linear_table(&[
        (-1.0, -1.0, 0.02, 0.1),
        (0.25, 0.75, 0.035, -0.025),
        (1.0, 2.0, 0.08, -0.2),
    ]);
    let expected = table.samples()[1];
    let family =
        ReynoldsPolarFamily::new(vec![ReynoldsPolar::new(100_000.0, table).unwrap()]).unwrap();
    assert_eq!(family.sample(100_000.0, 0.25).coefficients, expected.into());
}

#[test]
fn m2_3b_19_legacy_polar_table_behavior_is_unchanged() {
    let table = linear_table(&[
        (-1.0, -0.5, 0.02, 0.1),
        (0.0, 0.25, 0.04, -0.02),
        (1.0, 1.0, 0.10, -0.10),
    ]);
    assert_eq!(table.sample_clamped(-100.0), table.samples()[0].into());
    assert_eq!(table.sample_clamped(100.0), table.samples()[2].into());
    assert_eq!(
        table.sample_clamped(0.5),
        PolarCoefficients {
            cl: 0.625,
            cd: 0.07,
            cm: -0.06,
        }
    );
}

#[test]
fn m2_3b_20_sampling_allocates_nothing() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, 0.0, 0.02, 0.2),
        node(400_000.0, 2.0, 0.10, -0.2),
    ])
    .unwrap();
    let mut checksum = 0.0_f64;
    let allocation_info = allocation_counter::measure(|| {
        for index in 0..1_000 {
            let sample = family.sample(200_000.0, f64::from(index) / 1_000.0);
            checksum += sample.coefficients.cl + sample.coefficients.cd + sample.coefficients.cm;
        }
    });
    assert!(checksum.is_finite());
    assert_eq!(allocation_info.count_total, 0, "{allocation_info:?}");
}

#[test]
fn m2_3b_21_repeated_sampling_is_bit_deterministic() {
    let family = ReynoldsPolarFamily::new(vec![
        node(100_000.0, -3.0, 0.02, 0.2),
        node(400_000.0, 7.0, 0.10, -0.4),
    ])
    .unwrap();
    let expected = family.sample(237_500.0, 0.125);
    for _ in 0..1_000 {
        let actual = family.sample(237_500.0, 0.125);
        assert_eq!(
            actual.coefficients.cl.to_bits(),
            expected.coefficients.cl.to_bits()
        );
        assert_eq!(
            actual.coefficients.cd.to_bits(),
            expected.coefficients.cd.to_bits()
        );
        assert_eq!(
            actual.coefficients.cm.to_bits(),
            expected.coefficients.cm.to_bits()
        );
        assert_eq!(
            actual.interpolation_fraction.to_bits(),
            expected.interpolation_fraction.to_bits()
        );
        assert_eq!(actual.range_status, expected.range_status);
    }
}
