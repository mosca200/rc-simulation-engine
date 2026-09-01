mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    AerodynamicEvidenceClass, AerodynamicEvidenceLoader, ConvergenceStatus,
    ReferenceAerodynamicEvidenceError, SurveyClassification,
};
use serde_json::{Value, json};

const COMMITTED_EVIDENCE: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json"
);

fn synthetic_campaign() -> Value {
    let mut value: Value = serde_json::from_str(COMMITTED_EVIDENCE).unwrap();
    value["campaign"]["id"] = json!("synthetic-aerodynamic-evidence");
    value["campaign"]["classification"] = json!("synthetic_non_reference");
    value["campaign"]["manufacturer"] = json!("Synthetic Fixture Manufacturer");
    value["campaign"]["family"] = json!("Synthetic Test Airfoil Family");
    value["campaign"]["variant"] = json!("obviously-non-reference-test-only");
    value["airfoil_identity"] = json!({
        "name": "Synthetic asymmetric test section",
        "source_ids": ["synthetic-airfoil-source"],
        "notes": "Test-only identity; not Clark Y or LT-40 evidence."
    });
    value["coordinates"] = json!({
        "source_id": "synthetic-airfoil-source",
        "coordinate_format": "selig",
        "normalization": "unit_chord_source_as_published",
        "ordering": "upper_trailing_edge_to_leading_edge_to_lower_trailing_edge",
        "leading_edge_representation": "single_point",
        "trailing_edge_representation": "open",
        "transformation_provenance": "Synthetic five-point analytic fixture created only for validation tests.",
        "points_x_over_c_y_over_c": [
            [1.0, 0.1], [0.5, 0.2], [0.0, 0.0], [0.5, -0.3], [1.0, -0.1]
        ],
        "notes": "Synthetic non-reference coordinates."
    });
    value["provenance_sources"] = json!([
        {
            "id": "synthetic-airfoil-source",
            "kind": "airfoil_database",
            "title": "Synthetic test-only coordinate source",
            "publisher": "Model test suite",
            "url": "https://example.invalid/synthetic-airfoil",
            "retrieval_date": "2030-01-02",
            "sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "notes": "Not real-aircraft evidence."
        },
        {
            "id": "synthetic-polar-source",
            "kind": "published_research",
            "title": "Synthetic test-only polar source",
            "publisher": "Model test suite",
            "url": "https://example.invalid/synthetic-polar",
            "retrieval_date": "2030-01-02",
            "sha256": null,
            "notes": "Contains no physical results."
        }
    ]);
    value["operating_envelope"] = Value::Null;
    value["polar_datasets"] = json!([]);
    value
}

fn dataset(
    id: &str,
    evidence_class: &str,
    reynolds: f64,
    mach: f64,
    method_id: &str,
    convergence: &str,
) -> Value {
    let generated = evidence_class == "generated_solver";
    json!({
        "id": id,
        "evidence_class": evidence_class,
        "flow_conditions": {
            "reynolds": reynolds,
            "mach": mach,
            "density_kg_m3": null,
            "dynamic_viscosity_pa_s": null,
            "kinematic_viscosity_m2_s": null
        },
        "transition": {
            "assumptions": if generated { Some("Synthetic test-only free transition assumption.") } else { None },
            "ncrit": if generated { Some(7.0) } else { None },
            "forced_transition_upper_x_over_c": null,
            "forced_transition_lower_x_over_c": null
        },
        "method": {
            "id": method_id,
            "solver_or_tool": if generated { Some("Synthetic Solver") } else { None },
            "exact_version": if generated { Some("test-version-0") } else { None },
            "command_or_config": if generated { Some("synthetic --never-runtime") } else { None },
            "convergence_status": convergence
        },
        "source_ids": ["synthetic-polar-source"],
        "samples": [
            {"alpha_rad": -1.0, "cl": -10.0, "cd": 3.0, "cm": 5.0},
            {"alpha_rad": 1.0, "cl": 20.0, "cd": 4.0, "cm": -6.0}
        ],
        "notes": "Obviously synthetic non-reference coefficients."
    })
}

fn load(value: &Value) -> Result<model::AerodynamicEvidence, ReferenceAerodynamicEvidenceError> {
    AerodynamicEvidenceLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

fn set_complete_envelope(value: &mut Value, points: &[(f64, f64)]) {
    value["operating_envelope"] = json!({
        "rationale": "Synthetic test-only coverage requirement; not an LT-40 operating envelope.",
        "source_ids": ["synthetic-polar-source"],
        "required_points": points
            .iter()
            .map(|&(reynolds, mach)| json!({"reynolds": reynolds, "mach": mach}))
            .collect::<Vec<_>>()
    });
}

#[test]
fn committed_clark_y_template_is_traceable_but_polar_and_runtime_unresolved() {
    let evidence = AerodynamicEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    let evaluation = evidence.evaluation();
    assert_eq!(evidence.airfoil_name(), "Clark Y");
    assert_eq!(
        evidence.classification(),
        SurveyClassification::PhysicalReferenceMeasurement
    );
    assert!(evaluation.airfoil_identity_ready());
    assert!(evaluation.coordinates_ready());
    assert!(!evaluation.polar_evidence_ready());
    assert!(!evaluation.coverage_ready());
    assert!(!evaluation.aerodynamic_evidence_ready());
    assert!(!evaluation.runtime_ready());
    assert!(evaluation.datasets().is_empty());
}

#[test]
fn bad_duplicate_and_nonfinite_coordinates_fail_closed() {
    let mut unordered = synthetic_campaign();
    unordered["coordinates"]["points_x_over_c_y_over_c"][1][0] = json!(1.0);
    assert!(matches!(
        load(&unordered),
        Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate { .. })
    ));

    let mut duplicate = synthetic_campaign();
    duplicate["coordinates"]["points_x_over_c_y_over_c"][1] = json!([1.0, 0.1]);
    assert!(matches!(
        load(&duplicate),
        Err(ReferenceAerodynamicEvidenceError::InvalidCoordinate { .. })
    ));

    let nonfinite = serde_json::to_string(&synthetic_campaign())
        .unwrap()
        .replace("[0.5,0.2]", "[1e400,0.2]");
    assert!(AerodynamicEvidenceLoader::from_json_str(&nonfinite).is_err());
}

#[test]
fn duplicate_and_unresolved_provenance_sources_are_rejected() {
    let mut duplicate = synthetic_campaign();
    let first = duplicate["provenance_sources"][0].clone();
    duplicate["provenance_sources"]
        .as_array_mut()
        .unwrap()
        .push(first);
    assert!(matches!(
        load(&duplicate),
        Err(ReferenceAerodynamicEvidenceError::DuplicateStableId { .. })
    ));

    let mut unresolved = synthetic_campaign();
    unresolved["airfoil_identity"]["source_ids"] = json!(["missing-source"]);
    assert!(matches!(
        load(&unresolved),
        Err(ReferenceAerodynamicEvidenceError::UnresolvedSourceReference { .. })
    ));
}

#[test]
fn duplicate_dataset_ids_and_undistinguished_flow_points_are_rejected() {
    let mut duplicate_id = synthetic_campaign();
    duplicate_id["polar_datasets"] = json!([
        dataset(
            "synthetic-a",
            "published",
            10.0,
            0.01,
            "method-a",
            "not_applicable_published"
        ),
        dataset(
            "synthetic-a",
            "published",
            20.0,
            0.01,
            "method-b",
            "not_applicable_published"
        )
    ]);
    assert!(matches!(
        load(&duplicate_id),
        Err(ReferenceAerodynamicEvidenceError::DuplicateStableId { .. })
    ));

    let mut duplicate_point = synthetic_campaign();
    duplicate_point["polar_datasets"] = json!([
        dataset(
            "synthetic-a",
            "published",
            10.0,
            0.01,
            "method-a",
            "not_applicable_published"
        ),
        dataset(
            "synthetic-b",
            "published",
            10.0,
            0.01,
            "method-a",
            "not_applicable_published"
        )
    ]);
    assert!(matches!(
        load(&duplicate_point),
        Err(ReferenceAerodynamicEvidenceError::DuplicateFlowCondition { .. })
    ));
}

#[test]
fn invalid_reynolds_mach_samples_order_and_drag_are_rejected() {
    let base = dataset(
        "synthetic-a",
        "published",
        10.0,
        0.01,
        "method-a",
        "not_applicable_published",
    );
    for (pointer, invalid) in [
        ("/flow_conditions/reynolds", json!(0.0)),
        ("/flow_conditions/mach", json!(-0.01)),
    ] {
        let mut value = synthetic_campaign();
        let mut bad = base.clone();
        *bad.pointer_mut(pointer).unwrap() = invalid;
        value["polar_datasets"] = json!([bad]);
        assert!(matches!(
            load(&value),
            Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset { .. })
        ));
    }

    let mut unordered = synthetic_campaign();
    let mut bad = base.clone();
    bad["samples"][1]["alpha_rad"] = json!(-1.0);
    unordered["polar_datasets"] = json!([bad]);
    assert!(matches!(
        load(&unordered),
        Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset { .. })
    ));

    let mut negative_drag = synthetic_campaign();
    let mut bad = base.clone();
    bad["samples"][0]["cd"] = json!(-3.0);
    negative_drag["polar_datasets"] = json!([bad]);
    assert!(matches!(
        load(&negative_drag),
        Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset { .. })
    ));

    let mut malformed = synthetic_campaign();
    let mut bad = base;
    bad["samples"].as_array_mut().unwrap().truncate(1);
    malformed["polar_datasets"] = json!([bad]);
    assert!(matches!(
        load(&malformed),
        Err(ReferenceAerodynamicEvidenceError::InvalidPolarDataset { .. })
    ));
}

#[test]
fn unconverged_generated_dataset_is_retained_but_not_ready() {
    for status in ["unresolved", "failed"] {
        let mut value = synthetic_campaign();
        value["polar_datasets"] = json!([dataset(
            "synthetic-generated",
            "generated_solver",
            10.0,
            0.01,
            "synthetic-solver-method",
            status
        )]);
        let evaluation = load(&value).unwrap();
        assert!(!evaluation.evaluation().polar_evidence_ready());
        assert!(!evaluation.evaluation().runtime_ready());
        assert!(
            evaluation
                .evaluation()
                .blockers()
                .contains(&"generated_dataset_convergence:synthetic-generated".to_owned())
        );
    }
}

#[test]
fn dataset_ordering_and_complete_synthetic_multi_re_grid_are_deterministic() {
    let mut value = synthetic_campaign();
    value["polar_datasets"] = json!([
        dataset(
            "synthetic-re20",
            "generated_solver",
            20.0,
            0.02,
            "solver-b",
            "converged"
        ),
        dataset(
            "synthetic-re10",
            "published",
            10.0,
            0.01,
            "published-a",
            "not_applicable_published"
        )
    ]);
    set_complete_envelope(&mut value, &[(10.0, 0.01), (20.0, 0.02)]);
    let evidence = load(&value).unwrap();
    let evaluation = evidence.evaluation();
    assert_eq!(
        evidence.classification(),
        SurveyClassification::SyntheticNonReference
    );
    assert_eq!(evaluation.datasets()[0].id(), "synthetic-re10");
    assert_eq!(evaluation.datasets()[1].id(), "synthetic-re20");
    assert_eq!(
        evaluation.datasets()[0].evidence_class(),
        AerodynamicEvidenceClass::Published
    );
    assert_eq!(
        evaluation.datasets()[1].evidence_class(),
        AerodynamicEvidenceClass::GeneratedSolver
    );
    assert_eq!(
        evaluation.datasets()[1].convergence_status(),
        ConvergenceStatus::Converged
    );
    assert!(evaluation.polar_evidence_ready());
    assert!(evaluation.coverage_ready());
    assert!(evaluation.aerodynamic_evidence_ready());
    assert!(!evaluation.runtime_ready());
}

#[test]
fn missing_grid_coverage_is_reported_exactly() {
    let mut value = synthetic_campaign();
    value["polar_datasets"] = json!([dataset(
        "synthetic-re10",
        "published",
        10.0,
        0.01,
        "published-a",
        "not_applicable_published"
    )]);
    set_complete_envelope(&mut value, &[(10.0, 0.01), (30.0, 0.03)]);
    let evidence = load(&value).unwrap();
    assert!(!evidence.evaluation().coverage_ready());
    assert_eq!(evidence.evaluation().coverage_holes().len(), 1);
    assert_eq!(evidence.evaluation().coverage_holes()[0].reynolds(), 30.0);
    assert_eq!(evidence.evaluation().coverage_holes()[0].mach(), 0.03);
}

#[test]
fn aerodynamic_evidence_never_constructs_or_mutates_runtime_polars_or_fingerprints() {
    for value in [
        valid_model_value(),
        valid_v1_model_value(),
        valid_v2_reference_model_value(),
    ] {
        load_value(&value).unwrap();
    }
    let model_before = load_value(&valid_v2_reference_model_value()).unwrap();
    let fingerprint_before = model_before.physics_fingerprint();
    let polar_samples_before: Vec<_> = model_before
        .aero_polars()
        .iter()
        .map(|polar| polar.table().samples().to_vec())
        .collect();

    let evidence = AerodynamicEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    assert!(!evidence.evaluation().runtime_ready());

    let model_after = load_value(&valid_v2_reference_model_value()).unwrap();
    assert_eq!(fingerprint_before, model_after.physics_fingerprint());
    assert_eq!(
        polar_samples_before,
        model_after
            .aero_polars()
            .iter()
            .map(|polar| polar.table().samples().to_vec())
            .collect::<Vec<_>>()
    );
}
