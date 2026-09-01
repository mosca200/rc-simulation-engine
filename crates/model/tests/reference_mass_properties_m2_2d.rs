mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    MassPropertiesLoader, PublishedWeightRangeStatus, ReferenceMassPropertiesError,
    SurveyClassification, x_aft_to_frd_x,
};
use serde_json::{Value, json};

const EMPTY_CAMPAIGN: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_mass_properties_v0.json"
);

fn series(value: f64) -> Value {
    series_readings([value, value, value])
}

fn series_readings(readings: [f64; 3]) -> Value {
    json!({
        "readings": readings,
        "instrument_resolution": 0.01,
        "stated_uncertainty": 0.02,
        "datum_or_method_definition": "Synthetic test-only method in the synthetic FRD fixture.",
        "notes": "Synthetic non-reference observation.",
        "source_ids": ["synthetic-session"],
        "photograph_ids": ["synthetic-photo"]
    })
}

fn tensor(matrix: [[f64; 3]; 3]) -> Value {
    json!({
        "method_class": "evidenced_cad_mass_model",
        "method_definition": "Synthetic analytic tensor fixture; not physical-aircraft evidence.",
        "matrix_entries": {
            "ixx": series(matrix[0][0]),
            "ixy": series(matrix[0][1]),
            "ixz": series(matrix[0][2]),
            "iyx": series(matrix[1][0]),
            "iyy": series(matrix[1][1]),
            "iyz": series(matrix[1][2]),
            "izx": series(matrix[2][0]),
            "izy": series(matrix[2][1]),
            "izz": series(matrix[2][2])
        },
        "source_ids": ["synthetic-session"],
        "photograph_ids": ["synthetic-photo"],
        "notes": "Synthetic non-reference inertia."
    })
}

fn component(id: &str, category: &str, mass: f64, position: [f64; 3], inertia: Value) -> Value {
    json!({
        "id": id,
        "category": category,
        "description": "Obviously synthetic component used only for analytic testing.",
        "status": "installed",
        "configuration_id": "synthetic-config-a",
        "mass_kg": series(mass),
        "cg_position_frd_m": {
            "x": series(position[0]),
            "y": series(position[1]),
            "z": series(position[2])
        },
        "intrinsic_inertia_about_component_cg_frd_kg_m2": inertia,
        "source_ids": ["synthetic-session"],
        "photograph_ids": ["synthetic-photo"],
        "notes": "Synthetic fixture, not an LT-40 component observation."
    })
}

fn complete_component_campaign() -> Value {
    let mut value: Value = serde_json::from_str(EMPTY_CAMPAIGN).unwrap();
    value["campaign"]["id"] = json!("synthetic-mass-properties-campaign");
    value["campaign"]["classification"] = json!("synthetic_non_reference");
    value["campaign"]["identity"]["airframe_id"] = json!("synthetic-airframe");
    value["campaign"]["measurement_date"] = json!("2030-01-02");
    value["campaign"]["operational_configuration"] = json!({
        "id": "synthetic-config-a",
        "battery_configuration_id": "synthetic-battery-a",
        "propulsion_configuration_description": "Synthetic propulsion configuration.",
        "landing_gear_configuration": "Synthetic landing gear configuration.",
        "installed_equipment_notes": "Complete synthetic equipment manifest for tests only."
    });
    value["provenance_sources"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "synthetic-session",
            "kind": "measurement_session",
            "title": "Synthetic test-only mass session",
            "url": null,
            "sha256": null,
            "notes": "Not physical reference-aircraft evidence."
        }));
    value["photographs"] = json!([{
        "id": "synthetic-photo",
        "path": "synthetic/not-an-aircraft-measurement.jpg",
        "url": null,
        "sha256": null,
        "description": "Synthetic evidence reference for tests only."
    }]);
    value["coordinate_frame"]["axes_parallel_to_frd"] = json!(true);
    value["coordinate_frame"]["wing_root_le_center_plane_datum_established"] = json!(true);
    value["coordinate_frame"]["lateral_datum_established"] = json!(true);
    value["coordinate_frame"]["vertical_datum_established"] = json!(true);
    value["coordinate_frame"]["source_ids"] = json!(["synthetic-session"]);
    value["coordinate_frame"]["photograph_ids"] = json!(["synthetic-photo"]);
    value["acceptance_criteria"] = json!({
        "maximum_direct_vs_build_up_mass_difference_kg": 0.1,
        "maximum_direct_vs_build_up_cg_distance_m": 0.1,
        "maximum_direct_vs_build_up_inertia_frobenius_difference_kg_m2": 0.1
    });
    value["raw_observations"]["component_inventory_complete"] = json!(true);
    value["raw_observations"]["components"] = json!([
        component(
            "synthetic-component-a",
            "synthetic-left-group",
            10.0,
            [1.0, 2.0, 3.0],
            tensor([[4.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 6.0]])
        ),
        component(
            "synthetic-component-b",
            "synthetic-right-group",
            20.0,
            [-2.0, 4.0, -1.0],
            tensor([[7.0, 0.0, 0.0], [0.0, 8.0, 0.0], [0.0, 0.0, 9.0]])
        )
    ]);
    value
}

fn load(value: &Value) -> Result<model::MassPropertiesCampaign, ReferenceMassPropertiesError> {
    MassPropertiesLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1.0e-10,
        "{actual} != {expected}"
    );
}

fn expected_component_inertia() -> [[f64; 3]; 3] {
    [
        [144.0 + 1.0 / 3.0, 40.0, -80.0],
        [40.0, 179.0 + 2.0 / 3.0, 53.0 + 1.0 / 3.0],
        [-80.0, 53.0 + 1.0 / 3.0, 101.0 + 2.0 / 3.0],
    ]
}

#[test]
fn committed_unmeasured_template_parses_but_every_authority_gate_is_false() {
    let campaign = MassPropertiesLoader::from_json_str(EMPTY_CAMPAIGN).unwrap();
    let evaluation = campaign.evaluation();
    assert_eq!(
        campaign.classification(),
        SurveyClassification::PhysicalReferenceMeasurement
    );
    assert!(!evaluation.configuration_identified());
    assert!(!evaluation.mass_ready());
    assert!(!evaluation.cg_ready());
    assert!(!evaluation.inertia_ready());
    assert!(!evaluation.mass_properties_ready());
    assert!(!evaluation.runtime_ready());
    assert!(evaluation.direct_mass().is_none());
    assert!(evaluation.component_build_up_mass().is_none());
    assert_eq!(
        evaluation.direct_mass_published_range(),
        PublishedWeightRangeStatus::Unknown
    );
}

#[test]
fn malformed_nonfinite_negative_mass_and_malformed_tensor_fail_closed() {
    let mut negative = complete_component_campaign();
    negative["raw_observations"]["components"][0]["mass_kg"]["readings"] =
        json!([-10.0, -10.0, -10.0]);
    assert!(matches!(
        load(&negative),
        Err(ReferenceMassPropertiesError::InvalidMeasurement { .. })
    ));

    let non_finite = serde_json::to_string(&complete_component_campaign())
        .unwrap()
        .replace("[10.0,10.0,10.0]", "[1e400,1e400,1e400]");
    assert!(MassPropertiesLoader::from_json_str(&non_finite).is_err());

    let mut malformed = complete_component_campaign();
    malformed["raw_observations"]["components"][0]
        ["intrinsic_inertia_about_component_cg_frd_kg_m2"]["matrix_entries"]
        .as_object_mut()
        .unwrap()
        .remove("izz");
    assert!(matches!(
        load(&malformed),
        Err(ReferenceMassPropertiesError::InvalidStructure { .. })
    ));
}

#[test]
fn nonsymmetric_and_non_positive_definite_direct_tensors_are_rejected() {
    let mut nonsymmetric = complete_component_campaign();
    nonsymmetric["raw_observations"]["direct_inertia_about_operational_cg_frd_kg_m2"] =
        tensor([[10.0, 1.0, 0.0], [2.0, 11.0, 0.0], [0.0, 0.0, 12.0]]);
    assert!(matches!(
        load(&nonsymmetric),
        Err(ReferenceMassPropertiesError::NonSymmetricInertia { .. })
    ));

    let mut indefinite = complete_component_campaign();
    indefinite["raw_observations"]["direct_inertia_about_operational_cg_frd_kg_m2"] =
        tensor([[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    assert!(matches!(
        load(&indefinite),
        Err(ReferenceMassPropertiesError::NonPositiveDefiniteInertia { .. })
    ));
}

#[test]
fn three_reading_aggregation_is_deterministic() {
    let mut value = complete_component_campaign();
    value["raw_observations"]["direct_total_mass_kg"] = series_readings([10.0, 12.0, 14.0]);
    let campaign = load(&value).unwrap();
    let mass = campaign.evaluation().direct_mass().unwrap();
    assert_close(mass.mean(), 12.0);
    assert_close(mass.minimum(), 10.0);
    assert_close(mass.maximum(), 14.0);
    assert_close(mass.range(), 4.0);
    assert_close(mass.effective_uncertainty(), 2.0);
}

#[test]
fn complete_synthetic_component_build_up_derives_mass_cg_and_full_parallel_axis_tensor() {
    let campaign = load(&complete_component_campaign()).unwrap();
    let evaluation = campaign.evaluation();
    assert_eq!(
        campaign.classification(),
        SurveyClassification::SyntheticNonReference
    );
    assert_close(evaluation.component_build_up_mass().unwrap().value(), 30.0);
    let cg = evaluation.component_build_up_cg_frd_m().unwrap();
    assert_close(cg.value()[0], -1.0);
    assert_close(cg.value()[1], 10.0 / 3.0);
    assert_close(cg.value()[2], 1.0 / 3.0);
    let inertia = evaluation
        .component_build_up_inertia_frd_kg_m2()
        .unwrap()
        .matrix_frd_kg_m2();
    let expected = expected_component_inertia();
    for row in 0..3 {
        for column in 0..3 {
            assert_close(inertia[row][column], expected[row][column]);
        }
    }
    assert_close(inertia[0][1], 40.0);
    assert_close(inertia[0][2], -80.0);
    assert_close(inertia[1][2], 160.0 / 3.0);
    assert!(evaluation.configuration_identified());
    assert!(evaluation.mass_ready());
    assert!(evaluation.cg_ready());
    assert!(evaluation.inertia_ready());
    assert!(evaluation.mass_properties_ready());
    assert!(!evaluation.runtime_ready());
}

#[test]
fn missing_intrinsic_inertia_cannot_be_promoted_as_a_point_mass_result() {
    let mut value = complete_component_campaign();
    value["raw_observations"]["components"][0]["intrinsic_inertia_about_component_cg_frd_kg_m2"] =
        Value::Null;
    let campaign = load(&value).unwrap();
    let evaluation = campaign.evaluation();
    assert!(evaluation.mass_ready());
    assert!(evaluation.cg_ready());
    assert!(!evaluation.inertia_ready());
    assert!(evaluation.component_build_up_inertia_frd_kg_m2().is_none());
    assert!(
        evaluation
            .missing_requirements()
            .contains(&"component_intrinsic_inertia:synthetic-component-a".to_owned())
    );
}

#[test]
fn evidenced_direct_whole_aircraft_path_can_close_all_non_runtime_gates() {
    let mut value = complete_component_campaign();
    value["raw_observations"]["component_inventory_complete"] = json!(false);
    value["raw_observations"]["components"] = json!([]);
    value["raw_observations"]["direct_total_mass_kg"] = series(30.0);
    value["raw_observations"]["direct_cg_position_frd_m"] = json!({
        "x": series(-1.0), "y": series(10.0 / 3.0), "z": series(1.0 / 3.0)
    });
    value["raw_observations"]["direct_inertia_about_operational_cg_frd_kg_m2"] =
        tensor([[100.0, 1.0, -2.0], [1.0, 110.0, 3.0], [-2.0, 3.0, 120.0]]);
    let evaluation = load(&value).unwrap();
    assert!(evaluation.evaluation().mass_ready());
    assert!(evaluation.evaluation().cg_ready());
    assert!(evaluation.evaluation().inertia_ready());
    assert!(evaluation.evaluation().mass_properties_ready());
    assert!(!evaluation.evaluation().runtime_ready());
    assert!(evaluation.evaluation().missing_requirements().is_empty());
}

#[test]
fn non_positive_definite_final_component_build_up_tensor_is_rejected() {
    let mut value = complete_component_campaign();
    for component in value["raw_observations"]["components"]
        .as_array_mut()
        .unwrap()
    {
        component["cg_position_frd_m"] =
            json!({"x": series(0.0), "y": series(0.0), "z": series(0.0)});
        component["intrinsic_inertia_about_component_cg_frd_kg_m2"] =
            tensor([[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]]);
    }
    assert!(matches!(
        load(&value),
        Err(ReferenceMassPropertiesError::NonPositiveDefiniteInertia { field })
            if field == "component_build_up_inertia_frd_kg_m2"
    ));
}

#[test]
fn direct_vs_build_up_disagreements_are_explicit_readiness_blockers() {
    let mut mass = complete_component_campaign();
    mass["raw_observations"]["direct_total_mass_kg"] = series(60.0);
    let evaluation = load(&mass).unwrap();
    assert!(!evaluation.evaluation().mass_ready());
    assert!(
        evaluation
            .evaluation()
            .missing_requirements()
            .contains(&"direct_vs_build_up_mass_consistency".to_owned())
    );

    let mut cg = complete_component_campaign();
    cg["raw_observations"]["direct_cg_position_frd_m"] = json!({
        "x": series(100.0), "y": series(100.0), "z": series(100.0)
    });
    let evaluation = load(&cg).unwrap();
    assert!(!evaluation.evaluation().cg_ready());
    assert!(
        evaluation
            .evaluation()
            .missing_requirements()
            .contains(&"direct_vs_build_up_cg_consistency".to_owned())
    );

    let mut inertia = complete_component_campaign();
    inertia["raw_observations"]["direct_inertia_about_operational_cg_frd_kg_m2"] =
        tensor([[1000.0, 0.0, 0.0], [0.0, 1100.0, 0.0], [0.0, 0.0, 1200.0]]);
    let evaluation = load(&inertia).unwrap();
    assert!(!evaluation.evaluation().inertia_ready());
    assert!(
        evaluation
            .evaluation()
            .missing_requirements()
            .contains(&"direct_vs_build_up_inertia_consistency".to_owned())
    );
}

#[test]
fn published_range_and_reference_only_components_never_supply_operational_mass() {
    let template = MassPropertiesLoader::from_json_str(EMPTY_CAMPAIGN).unwrap();
    assert!(!template.evaluation().mass_ready());
    assert_eq!(
        template.evaluation().direct_mass_published_range(),
        PublishedWeightRangeStatus::Unknown
    );

    let mut value = complete_component_campaign();
    let mut reference = component(
        "synthetic-reference-only-motor",
        "historical-reference",
        999.0,
        [50.0, 60.0, 70.0],
        Value::Null,
    );
    reference["status"] = json!("reference_only");
    reference["configuration_id"] = Value::Null;
    value["raw_observations"]["components"]
        .as_array_mut()
        .unwrap()
        .push(reference);
    let evaluation = load(&value).unwrap();
    assert_close(
        evaluation
            .evaluation()
            .component_build_up_mass()
            .unwrap()
            .value(),
        30.0,
    );
    assert_eq!(
        evaluation.evaluation().component_mass_published_range(),
        PublishedWeightRangeStatus::OutsidePublishedRange
    );
}

#[test]
fn m2_2c_aft_station_bridge_negates_x_exactly() {
    assert_close(x_aft_to_frd_x(123.0).unwrap(), -123.0);
    assert_eq!(x_aft_to_frd_x(-0.0).unwrap().to_bits(), 0.0_f64.to_bits());
    assert!(x_aft_to_frd_x(f64::NAN).is_err());
}

#[test]
fn evidence_references_and_configuration_binding_are_strict() {
    let mut unresolved = complete_component_campaign();
    unresolved["raw_observations"]["components"][0]["mass_kg"]["source_ids"] =
        json!(["missing-source"]);
    assert!(matches!(
        load(&unresolved),
        Err(ReferenceMassPropertiesError::UnresolvedSourceReference { .. })
    ));

    let mut changed_configuration = complete_component_campaign();
    changed_configuration["campaign"]["operational_configuration"]["id"] =
        json!("synthetic-config-b");
    assert!(matches!(
        load(&changed_configuration),
        Err(ReferenceMassPropertiesError::ComponentConfigurationMismatch { .. })
    ));
}

#[test]
fn existing_model_versions_and_physics_fingerprint_are_unchanged_by_mass_metadata() {
    for value in [
        valid_model_value(),
        valid_v1_model_value(),
        valid_v2_reference_model_value(),
    ] {
        load_value(&value).unwrap();
    }
    let before = load_value(&valid_v2_reference_model_value())
        .unwrap()
        .physics_fingerprint();
    let mut first = complete_component_campaign();
    let mut second = first.clone();
    first["campaign"]["notes"] = json!("Synthetic documentary note A.");
    second["campaign"]["notes"] = json!("Synthetic documentary note B.");
    load(&first).unwrap();
    load(&second).unwrap();
    let after = load_value(&valid_v2_reference_model_value())
        .unwrap()
        .physics_fingerprint();
    assert_eq!(before, after);
}
