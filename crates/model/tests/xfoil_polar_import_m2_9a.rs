//! M2.9A — Deterministic XFOIL Polar Import tests.
//!
//! Integration tests covering the public API surface of the XFOIL polar import
//! module. All fixtures are synthetic; no real LT-40 or Clark Y data is used.

mod common;

use model::{
    InvalidMetadataReason, MetadataBuilder, XfoilPolarImport, XfoilPolarImportError,
    XfoilPolarSample, XfoilSolverMetadata, parse_xfoil_polar,
};

const STANDARD_6COL: &str = "\
 XFOIL 6.99


 Calculated polar for: SOME AIRFOIL


 1 1 Reynolds number: 250000    Mach number: 0.04


 Ncrit: 9.0


 alpha    CL         CD         CDp       Top_Xtr   Bot_Xtr
 ------   ---------  ---------  ---------  ---------  ---------
  -3.000  -0.2000    0.02000    0.01000    0.4000    0.7000
  -2.000  -0.0500    0.01200    0.00500    0.5000    0.6000
  -1.000   0.1000    0.00850    0.00300    0.5500    0.5500
   0.000   0.2500    0.00700    0.00200    0.6000    0.5000
   2.000   0.4500    0.00750    0.00250    0.6500    0.4500
   4.000   0.6500    0.01000    0.00400    0.7000    0.4000
   6.000   0.8500    0.01500    0.00700    0.7500    0.3500
";

const MINIMAL_4COL: &str = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
  -2.000  -0.1500    0.01500    0.0200
   0.000   0.0500    0.00800    0.0050
   2.000   0.2500    0.00900   -0.0100
   4.000   0.4500    0.01200   -0.0250
";

fn valid_metadata() -> XfoilSolverMetadata {
    MetadataBuilder::new(250_000.0, 0.04)
        .solver_name("XFOIL")
        .solver_version("6.99")
        .command_or_config("OPER RE 250000 VISC ITER 100")
        .transition_assumptions("Free transition via e^N method, Ncrit=9")
        .ncrit(9.0)
        .build()
        .expect("valid metadata")
}

#[test]
fn standard_6col_full_parse() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 7);
    assert_eq!(import.metadata().reynolds(), 250_000.0);
    assert_eq!(import.metadata().mach(), 0.04);
    assert_eq!(import.metadata().solver_name(), Some("XFOIL"));
    assert_eq!(import.metadata().solver_version(), Some("6.99"));
    assert_eq!(import.metadata().ncrit(), Some(9.0));
}

#[test]
fn minimal_4col_no_diagnostics() {
    let import = parse_xfoil_polar(MINIMAL_4COL, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 4);
    for sample in import.samples() {
        assert!(sample.cd_pressure().is_none());
        assert!(sample.top_xtr().is_none());
        assert!(sample.bot_xtr().is_none());
    }
}

#[test]
fn alpha_deg_to_rad_conversion() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    let factor = std::f64::consts::PI / 180.0;
    let expected = [-3.0, -2.0, -1.0, 0.0, 2.0, 4.0, 6.0];
    for (sample, &deg) in import.samples().iter().zip(&expected) {
        let expected_rad = deg * factor;
        assert!(
            (sample.alpha_rad() - expected_rad).abs() < 1e-14,
            "alpha mismatch for {deg} deg: got {} expected {expected_rad}",
            sample.alpha_rad()
        );
    }
}

#[test]
fn cl_cd_cm_exact_preservation() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    let s = &import.samples()[3];
    assert_eq!(s.cl(), 0.2500);
    assert_eq!(s.cd(), 0.00700);
    assert_eq!(s.cm(), 0.0);
}

#[test]
fn diagnostic_columns_exact_preservation() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    let s = &import.samples()[0];
    assert_eq!(s.cd_pressure(), Some(0.01000));
    assert_eq!(s.top_xtr(), Some(0.4000));
    assert_eq!(s.bot_xtr(), Some(0.7000));
    assert_eq!(s.cm(), 0.0);
}

#[test]
fn source_ordering_preserved() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    for i in 1..import.sample_count() {
        assert!(import.samples()[i].alpha_rad() > import.samples()[i - 1].alpha_rad());
    }
}

#[test]
fn deterministic_reparse() {
    let meta = valid_metadata();
    let a = parse_xfoil_polar(STANDARD_6COL, meta.clone()).unwrap();
    let b = parse_xfoil_polar(STANDARD_6COL, meta).unwrap();
    assert_eq!(a.sample_count(), b.sample_count());
    for (a_s, b_s) in a.samples().iter().zip(b.samples()) {
        assert_eq!(a_s.alpha_rad().to_bits(), b_s.alpha_rad().to_bits());
        assert_eq!(a_s.cl().to_bits(), b_s.cl().to_bits());
        assert_eq!(a_s.cd().to_bits(), b_s.cd().to_bits());
        assert_eq!(a_s.cm().to_bits(), b_s.cm().to_bits());
    }
}

#[test]
fn too_few_samples_zero() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::TooFewSamples { count: 0 }
    ));
}

#[test]
fn too_few_samples_one() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::TooFewSamples { count: 1 }
    ));
}

#[test]
fn duplicate_alpha_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   1.000   0.2000    0.00900   -0.0100
   1.000   0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::DuplicateAlpha { row: 2 }
    ));
}

#[test]
fn decreasing_alpha_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   2.000   0.3000    0.00900   -0.0100
   1.000   0.2000    0.00800    0.0050
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::AlphaNotIncreasing { row: 2 }
    ));
}

#[test]
fn malformed_row_after_start_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
   abc     0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::MalformedRow {
            row: 2,
            reason: "non-numeric value in data row"
        }
    ));
}

#[test]
fn negative_cd_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000   -0.00100    0.0050
   1.000   0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(err, XfoilPolarImportError::NegativeCd { row: 1 }));
}

#[test]
fn nan_value_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   NaN        0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::NonFiniteValue { row: 1 }
    ));
}

#[test]
fn inf_value_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   inf        0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::NonFiniteValue { row: 1 }
    ));
}

#[test]
fn header_not_found() {
    let err = parse_xfoil_polar("no polar data here\n", valid_metadata()).unwrap_err();
    assert!(matches!(err, XfoilPolarImportError::HeaderNotFound));
}

#[test]
fn empty_input() {
    let err = parse_xfoil_polar("", valid_metadata()).unwrap_err();
    assert!(matches!(err, XfoilPolarImportError::HeaderNotFound));
}

#[test]
fn invalid_reynolds_zero() {
    let err = MetadataBuilder::new(0.0, 0.0).build().unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ReynoldsNotFinitePositive)
    ));
}

#[test]
fn invalid_reynolds_negative() {
    let err = MetadataBuilder::new(-100.0, 0.0).build().unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ReynoldsNotFinitePositive)
    ));
}

#[test]
fn invalid_reynolds_nan() {
    let err = MetadataBuilder::new(f64::NAN, 0.0).build().unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ReynoldsNotFinitePositive)
    ));
}

#[test]
fn invalid_reynolds_infinity() {
    let err = MetadataBuilder::new(f64::INFINITY, 0.0)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ReynoldsNotFinitePositive)
    ));
}

#[test]
fn invalid_mach_negative() {
    let err = MetadataBuilder::new(300_000.0, -0.1).build().unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
    ));
}

#[test]
fn invalid_mach_nan() {
    let err = MetadataBuilder::new(300_000.0, f64::NAN)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
    ));
}

#[test]
fn invalid_mach_infinity() {
    let err = MetadataBuilder::new(300_000.0, f64::INFINITY)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
    ));
}

#[test]
fn invalid_ncrit_zero() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .ncrit(0.0)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
    ));
}

#[test]
fn invalid_ncrit_negative() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .ncrit(-5.0)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
    ));
}

#[test]
fn invalid_ncrit_nan() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .ncrit(f64::NAN)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
    ));
}

#[test]
fn invalid_forced_transition_upper_above_one() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .forced_transition_upper(1.5)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ForcedTransitionOutOfRange)
    ));
}

#[test]
fn invalid_forced_transition_lower_negative() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .forced_transition_lower(-0.1)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ForcedTransitionOutOfRange)
    ));
}

#[test]
fn invalid_forced_transition_nan() {
    let err = MetadataBuilder::new(300_000.0, 0.0)
        .forced_transition_upper(f64::NAN)
        .build()
        .unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ForcedTransitionOutOfRange)
    ));
}

#[test]
fn metadata_optional_fields_absent() {
    let meta = MetadataBuilder::new(300_000.0, 0.0).build().unwrap();
    assert!(meta.solver_name().is_none());
    assert!(meta.solver_version().is_none());
    assert!(meta.command_or_config().is_none());
    assert!(meta.transition_assumptions().is_none());
    assert!(meta.ncrit().is_none());
    assert!(meta.forced_transition_upper_x_over_c().is_none());
    assert!(meta.forced_transition_lower_x_over_c().is_none());
}

#[test]
fn metadata_all_fields_present() {
    let meta = MetadataBuilder::new(500_000.0, 0.1)
        .solver_name("XFOIL")
        .solver_version("6.99")
        .command_or_config("OPER RE 500000")
        .transition_assumptions("Free transition")
        .ncrit(9.0)
        .forced_transition_upper(0.1)
        .forced_transition_lower(0.9)
        .build()
        .unwrap();
    assert_eq!(meta.solver_name(), Some("XFOIL"));
    assert_eq!(meta.solver_version(), Some("6.99"));
    assert_eq!(meta.command_or_config(), Some("OPER RE 500000"));
    assert_eq!(meta.transition_assumptions(), Some("Free transition"));
    assert_eq!(meta.ncrit(), Some(9.0));
    assert_eq!(meta.forced_transition_upper_x_over_c(), Some(0.1));
    assert_eq!(meta.forced_transition_lower_x_over_c(), Some(0.9));
    assert_eq!(meta.reynolds(), 500_000.0);
    assert_eq!(meta.mach(), 0.1);
}

#[test]
fn metadata_boundary_values_valid() {
    assert!(MetadataBuilder::new(300_000.0, 0.0).build().is_ok());
    assert!(MetadataBuilder::new(1.0, 0.0).build().is_ok());
    assert!(
        MetadataBuilder::new(300_000.0, 0.0)
            .forced_transition_upper(0.0)
            .forced_transition_lower(1.0)
            .build()
            .is_ok()
    );
    assert!(
        MetadataBuilder::new(300_000.0, 0.0)
            .ncrit(0.001)
            .build()
            .is_ok()
    );
}

#[test]
fn data_stops_at_blank_line() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100

   2.000   0.3000    0.01000   -0.0200
";
    let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 2);
}

#[test]
fn wrong_column_count_rejected() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800
   1.000   0.2000    0.00900   -0.0100
";
    let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
    assert!(matches!(
        err,
        XfoilPolarImportError::MalformedRow {
            row: 1,
            reason: "expected 4 or 6 columns"
        }
    ));
}

#[test]
fn negative_zero_cd_accepted() {
    let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000   -0.00000    0.0050
   1.000   0.2000    0.00900   -0.0100
";
    let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 2);
    assert!(import.samples()[0].cd().is_sign_negative());
}

#[test]
fn import_is_off_runtime_evidence_only() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    let _: &XfoilPolarImport = &import;
    let _: &[XfoilPolarSample] = import.samples();
}

#[test]
fn no_clark_y_data_fabricated() {
    let import = parse_xfoil_polar(STANDARD_6COL, valid_metadata()).unwrap();
    for sample in import.samples() {
        assert!(sample.cl().abs() < 5.0);
        assert!(sample.cd() < 1.0);
        assert!(sample.cm().abs() < 1.0);
    }
}

#[test]
fn does_not_parse_real_evidence_artifact() {
    let content: Option<&str> = std::fs::read_to_string(
        "docs/reference_aircraft/data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json",
    )
    .ok()
    .and_then(|s| {
        s.contains("\"alpha_rad\"")
            .then_some("real evidence artifact exists")
    });
    assert!(content.is_none());
}

#[test]
fn separator_without_spaces_accepted() {
    let text = "\
 alpha    CL    CD    CM
---------------------------
  0.000   0.1   0.01  0.0
  1.000   0.2   0.02 -0.01
";
    let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 2);
}

#[test]
fn no_separator_line() {
    let text = "\
 alpha    CL         CD         CM
   0.000   0.1000    0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100
";
    let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
    assert_eq!(import.sample_count(), 2);
}

#[test]
fn error_display_messages() {
    let err = XfoilPolarImportError::HeaderNotFound;
    assert!(!err.to_string().is_empty());

    let err = XfoilPolarImportError::TooFewSamples { count: 1 };
    assert!(err.to_string().contains("1"));

    let err = XfoilPolarImportError::MalformedRow {
        row: 5,
        reason: "non-numeric value in data row",
    };
    assert!(err.to_string().contains("5"));

    let err =
        XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::ReynoldsNotFinitePositive);
    assert!(err.to_string().contains("Reynolds"));
}
