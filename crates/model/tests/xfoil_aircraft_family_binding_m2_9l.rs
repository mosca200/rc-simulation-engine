//! M2.9L — Bind canonical XFOIL evidence to aircraft runtime family integration tests.
//!
//! All fixtures use synthetic data. No Clark Y or LT-40 aerodynamic data appears here.

use std::io::Write;

use model::{
    ConvergenceStatus, MetadataBuilder, RuntimeAeroPolarBinding, XfoilEvidenceCampaignBuilder,
    XfoilEvidenceDatasetBuilder, XfoilEvidenceJsonError, bind_xfoil_evidence_to_reynolds_family,
    load_aircraft_model, parse_xfoil_polar,
};

const SOURCE_ID: &str = "synthetic-m2-9l-source";

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
    .notes("Synthetic M2.9L test fixture")
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

fn canonical_json_from_datasets(datasets: Vec<model::XfoilEvidenceDataset>) -> String {
    let campaign = XfoilEvidenceCampaignBuilder::new(datasets).build().unwrap();
    campaign.to_polar_datasets_json_pretty()
}

/// Minimal v3 model JSON with two Reynolds polar families and elements bound to them.
fn test_model_json() -> String {
    serde_json::json!({
        "schema_version": 3,
        "model_id": "m2-9l-test-model",
        "display_name": "M2.9L Test Model",
        "classification": "synthetic_test",
        "reference_aircraft": null,
        "rigid_body": {
            "mass_kg": 2.0,
            "inertia_body_kg_m2": [
                [0.1, 0.0, 0.0],
                [0.0, 0.2, 0.0],
                [0.0, 0.0, 0.3]
            ]
        },
        "aerodynamics": {
            "kinematic_viscosity_m2_s": 1.5e-5,
            "polars": [],
            "polar_families": [
                {
                    "id": "wing-family",
                    "nodes": [
                        {
                            "reynolds_number": 100000.0,
                            "samples": [
                                {"alpha_rad": -0.1, "cl": -0.3, "cd": 0.02, "cm": -0.01},
                                {"alpha_rad": 0.0, "cl": 0.1, "cd": 0.015, "cm": -0.02},
                                {"alpha_rad": 0.1, "cl": 0.5, "cd": 0.025, "cm": -0.03}
                            ]
                        },
                        {
                            "reynolds_number": 300000.0,
                            "samples": [
                                {"alpha_rad": -0.1, "cl": -0.28, "cd": 0.018, "cm": -0.01},
                                {"alpha_rad": 0.0, "cl": 0.12, "cd": 0.013, "cm": -0.02},
                                {"alpha_rad": 0.1, "cl": 0.52, "cd": 0.022, "cm": -0.03}
                            ]
                        }
                    ]
                },
                {
                    "id": "tail-family",
                    "nodes": [
                        {
                            "reynolds_number": 50000.0,
                            "samples": [
                                {"alpha_rad": -0.1, "cl": -0.2, "cd": 0.01, "cm": 0.0},
                                {"alpha_rad": 0.0, "cl": 0.0, "cd": 0.008, "cm": 0.0},
                                {"alpha_rad": 0.1, "cl": 0.2, "cd": 0.01, "cm": 0.0}
                            ]
                        }
                    ]
                }
            ],
            "elements": [
                {
                    "id": "wing-element",
                    "position_body_m": [0.0, 0.0, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.5,
                    "chord_m": 0.25,
                    "polar_binding": {"kind": "reynolds_family", "family_id": "wing-family"}
                },
                {
                    "id": "tail-element",
                    "position_body_m": [-0.5, 0.0, 0.0],
                    "orientation_body_from_element_wxyz": [1.0, 0.0, 0.0, 0.0],
                    "area_m2": 0.15,
                    "chord_m": 0.15,
                    "polar_binding": {"kind": "reynolds_family", "family_id": "tail-family"}
                }
            ]
        },
        "controls": {
            "response": {
                "roll": {"rate": 1.0, "expo": 0.0},
                "pitch": {"rate": 1.0, "expo": 0.0},
                "yaw": {"rate": 1.0, "expo": 0.0}
            },
            "servos": {
                "aileron": {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 5.0, "reversed": false},
                "elevator": {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 5.0, "reversed": false},
                "rudder": {"min_angle_rad": -0.5, "neutral_angle_rad": 0.0, "max_angle_rad": 0.5, "max_speed_rad_s": 5.0, "reversed": false}
            }
        },
        "control_surface_bindings": [],
        "propulsion": null,
        "presentation": null
    })
    .to_string()
}

fn load_test_model() -> model::AircraftModel {
    let json = test_model_json();
    let dir = std::env::temp_dir().join(format!(
        "rcsim-m2-9l-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("model.json");
    {
        let mut f = std::fs::File::create(&path).unwrap();
        f.write_all(json.as_bytes()).unwrap();
    }
    let model = load_aircraft_model(&path).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    model
}

// ── Test 1: valid replacement ────────────────────────────────────────────────

#[test]
fn valid_replacement() {
    let mut model = load_test_model();
    let original_family_count = model.aero_polar_families().len();
    assert_eq!(original_family_count, 2);

    let datasets = vec![
        standard_converged_dataset("ds-low", 200_000.0),
        standard_converged_dataset("ds-high", 400_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);

    let result =
        bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    assert_eq!(result.family_id(), "wing-family");
    assert_eq!(result.family_index(), 0);
    assert_eq!(result.mach(), 0.0);
    assert_eq!(model.aero_polar_families().len(), original_family_count);
    assert_eq!(model.aero_polar_families()[0].id(), "wing-family");
    assert_eq!(model.aero_polar_families()[0].family().nodes().len(), 2);
}

// ── Test 2: target element still references same family index ────────────────

#[test]
fn target_element_still_references_same_family_index() {
    let mut model = load_test_model();

    let wing_element_before = model.aero_elements()[0].polar_binding();
    assert!(matches!(
        wing_element_before,
        RuntimeAeroPolarBinding::ReynoldsFamily { family_index: 0 }
    ));

    let datasets = vec![standard_converged_dataset("ds-rep", 200_000.0)];
    let json = canonical_json_from_datasets(datasets);
    bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    let wing_element_after = model.aero_elements()[0].polar_binding();
    assert_eq!(wing_element_after, wing_element_before);
}

// ── Test 3: coefficients/alpha/Re exactly from evidence ──────────────────────

#[test]
fn coefficients_alpha_re_exactly_from_evidence() {
    let mut model = load_test_model();

    let datasets = vec![
        standard_converged_dataset("ds-a", 150_000.0),
        standard_converged_dataset("ds-b", 350_000.0),
    ];
    let source_samples_0: Vec<_> = datasets[0]
        .import()
        .samples()
        .iter()
        .map(|s| (s.alpha_rad(), s.cl(), s.cd(), s.cm()))
        .collect();

    let json = canonical_json_from_datasets(datasets);
    bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    let family = &model.aero_polar_families()[0].family();
    assert_eq!(family.nodes().len(), 2);

    assert_eq!(
        family.nodes()[0].reynolds_number().to_bits(),
        150_000.0_f64.to_bits()
    );
    assert_eq!(
        family.nodes()[1].reynolds_number().to_bits(),
        350_000.0_f64.to_bits()
    );

    let samples = family.nodes()[0].table().samples();
    assert_eq!(samples.len(), source_samples_0.len());
    for (i, sample) in samples.iter().enumerate() {
        let (ea, ecl, ecd, ecm) = source_samples_0[i];
        assert_eq!(sample.alpha_rad.to_bits(), ea.to_bits());
        assert_eq!(sample.cl.to_bits(), ecl.to_bits());
        assert_eq!(sample.cd.to_bits(), ecd.to_bits());
        assert_eq!(sample.cm.to_bits(), ecm.to_bits());
    }
}

// ── Test 4: malformed JSON rejected ─────────────────────────────────────────

#[test]
fn malformed_json_rejected() {
    let mut model = load_test_model();
    let err =
        bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", b"not json").unwrap_err();
    assert!(matches!(
        err,
        model::XfoilEvidenceBindingError::EvidenceJson(XfoilEvidenceJsonError::MalformedJson(_))
    ));
    assert_eq!(model.aero_polar_families()[0].family().nodes().len(), 2);
}

// ── Test 5: unresolved evidence rejected ─────────────────────────────────────

#[test]
fn unresolved_evidence_rejected() {
    let mut model = load_test_model();
    let datasets = vec![build_dataset(
        "ds-unresolved",
        200_000.0,
        0.0,
        ConvergenceStatus::Unresolved,
        &[-5.0, 0.0, 5.0],
    )];
    let json = canonical_json_from_datasets(datasets);

    let err = bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes())
        .unwrap_err();
    assert!(matches!(
        err,
        model::XfoilEvidenceBindingError::EvidenceJson(
            XfoilEvidenceJsonError::DatasetNotConverged { .. }
        )
    ));
}

// ── Test 6: missing family ID rejected ───────────────────────────────────────

#[test]
fn missing_family_id_rejected() {
    let mut model = load_test_model();
    let datasets = vec![standard_converged_dataset("ds-a", 200_000.0)];
    let json = canonical_json_from_datasets(datasets);

    let err =
        bind_xfoil_evidence_to_reynolds_family(&mut model, "nonexistent-family", json.as_bytes())
            .unwrap_err();
    assert!(matches!(
        err,
        model::XfoilEvidenceBindingError::FamilyNotFound { ref family_id } if family_id == "nonexistent-family"
    ));
}

// ── Test 7: unrelated families unchanged ─────────────────────────────────────

#[test]
fn unrelated_families_unchanged() {
    let mut model = load_test_model();

    let tail_nodes_before: Vec<_> = model.aero_polar_families()[1]
        .family()
        .nodes()
        .iter()
        .map(|n| (n.reynolds_number().to_bits(), n.table().samples().len()))
        .collect();

    let datasets = vec![standard_converged_dataset("ds-a", 200_000.0)];
    let json = canonical_json_from_datasets(datasets);
    bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    let tail_nodes_after: Vec<_> = model.aero_polar_families()[1]
        .family()
        .nodes()
        .iter()
        .map(|n| (n.reynolds_number().to_bits(), n.table().samples().len()))
        .collect();

    assert_eq!(tail_nodes_before, tail_nodes_after);
    assert_eq!(model.aero_polar_families()[1].id(), "tail-family");
}

// ── Test 8: unrelated aircraft config unchanged ──────────────────────────────

#[test]
fn unrelated_aircraft_config_unchanged() {
    let mut model = load_test_model();

    let mass_before = model.rigid_body().mass_kg();
    let element_count_before = model.aero_elements().len();
    let schema_before = model.schema_version();

    let wing_fingerprint_before = {
        let f = model.aero_polar_families()[0].family();
        (f.nodes().len(), f.nodes()[0].reynolds_number().to_bits())
    };

    let datasets = vec![standard_converged_dataset("ds-a", 200_000.0)];
    let json = canonical_json_from_datasets(datasets);
    bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    assert_eq!(model.rigid_body().mass_kg(), mass_before);
    assert_eq!(model.aero_elements().len(), element_count_before);
    assert_eq!(model.schema_version(), schema_before);

    let wing_fingerprint_after = {
        let f = model.aero_polar_families()[0].family();
        (f.nodes().len(), f.nodes()[0].reynolds_number().to_bits())
    };
    assert_ne!(wing_fingerprint_before, wing_fingerprint_after);
}

// ── Test 9: fingerprint changes with changed polar physics ───────────────────

#[test]
fn fingerprint_changes_with_changed_polar_physics() {
    let mut model = load_test_model();
    let fingerprint_before = model.physics_fingerprint();

    let datasets = vec![
        standard_converged_dataset("ds-a", 200_000.0),
        standard_converged_dataset("ds-b", 400_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);
    bind_xfoil_evidence_to_reynolds_family(&mut model, "wing-family", json.as_bytes()).unwrap();

    let fingerprint_after = model.physics_fingerprint();
    assert_ne!(fingerprint_before, fingerprint_after);
}

#[test]
fn fingerprint_unchanged_when_same_evidence_rebound() {
    let mut model_a = load_test_model();
    let mut model_b = load_test_model();

    let datasets = vec![standard_converged_dataset("ds-a", 200_000.0)];
    let json = canonical_json_from_datasets(datasets);

    bind_xfoil_evidence_to_reynolds_family(&mut model_a, "wing-family", json.as_bytes()).unwrap();
    bind_xfoil_evidence_to_reynolds_family(&mut model_b, "wing-family", json.as_bytes()).unwrap();

    assert_eq!(model_a.physics_fingerprint(), model_b.physics_fingerprint());
}

// ── Test 10: deterministic repeated binding ──────────────────────────────────

#[test]
fn deterministic_repeated_binding() {
    let mut model_a = load_test_model();
    let mut model_b = load_test_model();

    let datasets = vec![
        standard_converged_dataset("ds-a", 200_000.0),
        standard_converged_dataset("ds-b", 400_000.0),
    ];
    let json = canonical_json_from_datasets(datasets);

    let result_a =
        bind_xfoil_evidence_to_reynolds_family(&mut model_a, "wing-family", json.as_bytes())
            .unwrap();
    let result_b =
        bind_xfoil_evidence_to_reynolds_family(&mut model_b, "wing-family", json.as_bytes())
            .unwrap();

    assert_eq!(result_a.family_index(), result_b.family_index());
    assert_eq!(result_a.family_id(), result_b.family_id());
    assert_eq!(result_a.mach().to_bits(), result_b.mach().to_bits());
    assert_eq!(model_a, model_b);
}
