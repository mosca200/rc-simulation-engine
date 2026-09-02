//! M2.9B — XFOIL-to-Aerodynamic-Evidence Bridge integration tests.
//!
//! All fixtures are synthetic. No real LT-40 or Clark Y data is used.

mod common;

use model::{
    AerodynamicEvidenceClass, AerodynamicEvidenceLoader, ConvergenceStatus, MetadataBuilder,
    XfoilEvidenceBridgeError, XfoilEvidenceDatasetBuilder, parse_xfoil_polar,
};
use serde_json::{Value, json};

const STANDARD_7COL: &str = "\
 XFOIL 6.99


 Calculated polar for: SYNTHETIC TEST AIRFOIL


 1 1 Reynolds number: 250000    Mach number: 0.04


 alpha    CL         CD         CDp        CM         Top_Xtr  Bot_Xtr
 ------   ---------  ---------  ---------  ---------  -------  -------
  -2.000  -0.0414    0.01134    0.00442   -0.0120     0.5412   0.6178
   0.000   0.1593    0.00700    0.00156   -0.0549     0.5812   0.5612
   2.000   0.3593    0.00720    0.00180   -0.0570     0.6200   0.5200
   4.000   0.5593    0.00900    0.00300   -0.0530     0.6500   0.4800
";

fn full_metadata_import() -> model::XfoilPolarImport {
    let metadata = MetadataBuilder::new(250_000.0, 0.04)
        .solver_name("XFOIL")
        .solver_version("6.99")
        .command_or_config("OPER RE 250000 VISC ITER 100")
        .transition_assumptions("Free transition e^N, Ncrit=9")
        .ncrit(9.0)
        .build()
        .unwrap();
    parse_xfoil_polar(STANDARD_7COL, metadata).unwrap()
}

fn valid_builder() -> XfoilEvidenceDatasetBuilder {
    XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "synthetic-xfoil-dataset_01",
        "xfoil-method_01",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    )
}

fn synthetic_evidence_artifact(dataset_json: Value) -> Value {
    json!({
        "schema": "reference_aircraft_aerodynamic_evidence_v0",
        "artifact_kind": "aerodynamic_evidence_not_runtime_configuration",
        "campaign": {
            "id": "synthetic-m2-9b-bridge-test",
            "classification": "synthetic_non_reference",
            "manufacturer": "Synthetic Bridge Test Manufacturer",
            "family": "Bridge Test Family",
            "variant": "integration-test-only",
            "notes": null
        },
        "airfoil_identity": {
            "name": "Synthetic Test Airfoil",
            "source_ids": ["synthetic-solver-source"],
            "notes": null
        },
        "coordinates": {
            "source_id": "synthetic-solver-source",
            "coordinate_format": "selig",
            "normalization": "unit_chord_source_as_published",
            "ordering": "upper_trailing_edge_to_leading_edge_to_lower_trailing_edge",
            "leading_edge_representation": "single_point",
            "trailing_edge_representation": "open",
            "transformation_provenance": "Synthetic five-point fixture for M2.9B bridge test.",
            "points_x_over_c_y_over_c": [
                [1.0, 0.1], [0.5, 0.2], [0.0, 0.0], [0.5, -0.3], [1.0, -0.1]
            ],
            "notes": null
        },
        "provenance_sources": [
            {
                "id": "synthetic-solver-source",
                "kind": "solver_tool",
                "title": "Synthetic XFOIL solver output source",
                "publisher": "Test suite",
                "url": "https://example.invalid/m2-9b",
                "retrieval_date": "2030-06-01",
                "sha256": null,
                "notes": null
            }
        ],
        "operating_envelope": null,
        "polar_datasets": [dataset_json]
    })
}

#[test]
fn valid_bridge_creation() {
    let dataset = valid_builder().build().unwrap();
    assert_eq!(dataset.dataset_id(), "synthetic-xfoil-dataset_01");
    assert_eq!(dataset.method_id(), "xfoil-method_01");
    assert_eq!(dataset.convergence_status(), ConvergenceStatus::Converged);
    assert_eq!(dataset.reynolds(), 250_000.0);
    assert_eq!(dataset.mach(), 0.04);
    assert_eq!(dataset.sample_count(), 4);
    assert_eq!(dataset.source_ids(), &["synthetic-solver-source"]);
}

#[test]
fn generated_solver_class_in_json() {
    let dataset = valid_builder().build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(json["evidence_class"], "generated_solver");
}

#[test]
fn reynolds_mach_exact() {
    let dataset = valid_builder().build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(json["flow_conditions"]["reynolds"], 250_000.0);
    assert_eq!(json["flow_conditions"]["mach"], 0.04);
}

#[test]
fn sample_count_and_ordering() {
    let dataset = valid_builder().build().unwrap();
    let json = dataset.to_json_value();
    let samples = json["samples"].as_array().unwrap();
    assert_eq!(samples.len(), 4);
    for i in 1..samples.len() {
        let prev = samples[i - 1]["alpha_rad"].as_f64().unwrap();
        let curr = samples[i]["alpha_rad"].as_f64().unwrap();
        assert!(curr > prev);
    }
}

#[test]
fn alpha_cl_cd_cm_bitwise_preserved() {
    let import = full_metadata_import();
    let builder = XfoilEvidenceDatasetBuilder::new(
        import.clone(),
        "ds-bits",
        "m-01",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let json = dataset.to_json_value();
    let samples = json["samples"].as_array().unwrap();

    for (i, xfoil_sample) in import.samples().iter().enumerate() {
        let js = &samples[i];
        assert_eq!(
            js["alpha_rad"].as_f64().unwrap().to_bits(),
            xfoil_sample.alpha_rad().to_bits()
        );
        assert_eq!(
            js["cl"].as_f64().unwrap().to_bits(),
            xfoil_sample.cl().to_bits()
        );
        assert_eq!(
            js["cd"].as_f64().unwrap().to_bits(),
            xfoil_sample.cd().to_bits()
        );
        assert_eq!(
            js["cm"].as_f64().unwrap().to_bits(),
            xfoil_sample.cm().to_bits()
        );
    }
}

#[test]
fn convergence_explicit_converged() {
    let dataset = valid_builder().build().unwrap();
    assert_eq!(dataset.convergence_status(), ConvergenceStatus::Converged);
    let json = dataset.to_json_value();
    assert_eq!(json["method"]["convergence_status"], "converged");
}

#[test]
fn convergence_explicit_unresolved() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-ur",
        "m-01",
        ConvergenceStatus::Unresolved,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    assert_eq!(dataset.convergence_status(), ConvergenceStatus::Unresolved);
    let json = dataset.to_json_value();
    assert_eq!(json["method"]["convergence_status"], "unresolved");
}

#[test]
fn convergence_explicit_failed() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-fail",
        "m-01",
        ConvergenceStatus::Failed,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    assert_eq!(dataset.convergence_status(), ConvergenceStatus::Failed);
}

#[test]
fn source_ordering_preserved() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-src",
        "m-01",
        ConvergenceStatus::Converged,
        vec![
            "source-c".to_owned(),
            "source-a".to_owned(),
            "source-b".to_owned(),
        ],
    );
    let dataset = builder.build().unwrap();
    assert_eq!(dataset.source_ids(), &["source-c", "source-a", "source-b"]);
}

#[test]
fn duplicate_source_rejected() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-dup",
        "m-01",
        ConvergenceStatus::Converged,
        vec!["source-a".to_owned(), "source-a".to_owned()],
    );
    let err = builder.build().unwrap_err();
    assert!(matches!(
        err,
        XfoilEvidenceBridgeError::DuplicateSourceId(ref id) if id == "source-a"
    ));
}

#[test]
fn invalid_dataset_id_rejected() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "BAD ID!",
        "m-01",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    );
    assert!(matches!(
        builder.build().unwrap_err(),
        XfoilEvidenceBridgeError::InvalidDatasetId(_)
    ));
}

#[test]
fn invalid_method_id_rejected() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-01",
        "BAD ID!",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    );
    assert!(matches!(
        builder.build().unwrap_err(),
        XfoilEvidenceBridgeError::InvalidMethodId(_)
    ));
}

#[test]
fn transition_metadata_preserved() {
    let dataset = valid_builder().build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(
        json["transition"]["assumptions"],
        "Free transition e^N, Ncrit=9"
    );
    assert_eq!(json["transition"]["ncrit"], 9.0);
    assert_eq!(
        json["transition"]["forced_transition_upper_x_over_c"],
        Value::Null
    );
    assert_eq!(
        json["transition"]["forced_transition_lower_x_over_c"],
        Value::Null
    );
}

#[test]
fn forced_transition_from_metadata_not_from_xtr_diagnostics() {
    let metadata = MetadataBuilder::new(250_000.0, 0.04)
        .solver_name("XFOIL")
        .solver_version("6.99")
        .forced_transition_upper(0.05)
        .forced_transition_lower(0.95)
        .build()
        .unwrap();
    let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
    let builder = XfoilEvidenceDatasetBuilder::new(
        import,
        "ds-ft",
        "m-01",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(json["transition"]["forced_transition_upper_x_over_c"], 0.05);
    assert_eq!(json["transition"]["forced_transition_lower_x_over_c"], 0.95);
}

#[test]
fn missing_optional_metadata_not_fabricated() {
    let metadata = MetadataBuilder::new(250_000.0, 0.04).build().unwrap();
    let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
    let builder = XfoilEvidenceDatasetBuilder::new(
        import,
        "ds-bare",
        "m-bare",
        ConvergenceStatus::Unresolved,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(json["method"]["solver_or_tool"], Value::Null);
    assert_eq!(json["method"]["exact_version"], Value::Null);
    assert_eq!(json["method"]["command_or_config"], Value::Null);
    assert_eq!(json["transition"]["assumptions"], Value::Null);
    assert_eq!(json["transition"]["ncrit"], Value::Null);
}

#[test]
fn deterministic_repeated_serialization() {
    let a = valid_builder().build().unwrap();
    let b = valid_builder().build().unwrap();
    assert_eq!(a.to_json_pretty(), b.to_json_pretty());
    assert_eq!(
        serde_json::to_string(&a.to_json_value()).unwrap(),
        serde_json::to_string(&b.to_json_value()).unwrap()
    );
}

#[test]
fn end_to_end_through_evidence_loader_converged() {
    let dataset = valid_builder().build().unwrap();
    let artifact = synthetic_evidence_artifact(dataset.to_json_value());
    let json_str = serde_json::to_string_pretty(&artifact).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

    let eval = evidence.evaluation();
    assert_eq!(eval.datasets().len(), 1);
    let ds = &eval.datasets()[0];
    assert_eq!(ds.id(), "synthetic-xfoil-dataset_01");
    assert_eq!(
        ds.evidence_class(),
        AerodynamicEvidenceClass::GeneratedSolver
    );
    assert_eq!(ds.reynolds(), 250_000.0);
    assert_eq!(ds.mach(), 0.04);
    assert_eq!(ds.method_id(), "xfoil-method_01");
    assert_eq!(ds.convergence_status(), ConvergenceStatus::Converged);
    assert!(ds.evidence_ready());
    assert!(!eval.runtime_ready());
}

#[test]
fn unresolved_dataset_not_evidence_ready() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-unresolved",
        "m-01",
        ConvergenceStatus::Unresolved,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let artifact = synthetic_evidence_artifact(dataset.to_json_value());
    let json_str = serde_json::to_string_pretty(&artifact).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

    let eval = evidence.evaluation();
    let ds = &eval.datasets()[0];
    assert_eq!(ds.convergence_status(), ConvergenceStatus::Unresolved);
    assert!(!ds.evidence_ready());
    assert!(!eval.runtime_ready());
}

#[test]
fn failed_dataset_not_evidence_ready() {
    let builder = XfoilEvidenceDatasetBuilder::new(
        full_metadata_import(),
        "ds-failed",
        "m-01",
        ConvergenceStatus::Failed,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let artifact = synthetic_evidence_artifact(dataset.to_json_value());
    let json_str = serde_json::to_string_pretty(&artifact).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

    let eval = evidence.evaluation();
    let ds = &eval.datasets()[0];
    assert_eq!(ds.convergence_status(), ConvergenceStatus::Failed);
    assert!(!ds.evidence_ready());
}

#[test]
fn converged_but_missing_solver_metadata_blocks_evidence_ready() {
    let metadata = MetadataBuilder::new(250_000.0, 0.04).build().unwrap();
    let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
    let builder = XfoilEvidenceDatasetBuilder::new(
        import,
        "ds-incomplete",
        "m-inc",
        ConvergenceStatus::Converged,
        vec!["synthetic-solver-source".to_owned()],
    );
    let dataset = builder.build().unwrap();
    let artifact = synthetic_evidence_artifact(dataset.to_json_value());
    let json_str = serde_json::to_string_pretty(&artifact).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

    let eval = evidence.evaluation();
    let ds = &eval.datasets()[0];
    assert_eq!(ds.convergence_status(), ConvergenceStatus::Converged);
    assert!(!ds.evidence_ready());
    assert!(!eval.blockers().is_empty());
}

#[test]
fn no_real_lt40_coefficients_introduced() {
    let dataset = valid_builder().build().unwrap();
    for sample in dataset.import().samples() {
        assert!(sample.cl().abs() < 5.0);
        assert!(sample.cd() < 1.0);
        assert!(sample.cm().abs() < 1.0);
    }
}

#[test]
fn notes_preserved_when_supplied() {
    let builder = valid_builder().notes("Synthetic test dataset for M2.9B.");
    let dataset = builder.build().unwrap();
    assert_eq!(dataset.notes(), Some("Synthetic test dataset for M2.9B."));
    let json = dataset.to_json_value();
    assert_eq!(json["notes"], "Synthetic test dataset for M2.9B.");
}

#[test]
fn notes_null_when_absent() {
    let dataset = valid_builder().build().unwrap();
    let json = dataset.to_json_value();
    assert_eq!(json["notes"], Value::Null);
}
