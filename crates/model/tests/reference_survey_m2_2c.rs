mod common;

use common::{load_value, valid_v2_reference_model_value};
use model::{CrossVariantStatus, PhysicalSurveyLoader, ReferenceSurveyError, SurveyClassification};
use serde_json::{Value, json};

const EMPTY_CAMPAIGN: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_physical_survey_v0.json"
);

fn series(readings: [f64; 3]) -> Value {
    json!({
        "readings": readings,
        "instrument_resolution": 0.001,
        "stated_uncertainty": 0.002,
        "datum_definition": "Synthetic fixture datum; not reference-aircraft evidence.",
        "notes": "Synthetic non-reference test input.",
        "source_ids": ["synthetic-session"],
        "photograph_ids": ["synthetic-photo"]
    })
}

fn complete_synthetic_campaign() -> Value {
    let mut value: Value = serde_json::from_str(EMPTY_CAMPAIGN).unwrap();
    value["campaign"]["id"] = json!("synthetic-complete-survey");
    value["campaign"]["classification"] = json!("synthetic_non_reference");
    value["campaign"]["identity"]["airframe_id"] = json!("synthetic-airframe");
    value["campaign"]["measurement_date"] = json!("2030-01-02");
    value["provenance_sources"]
        .as_array_mut()
        .unwrap()
        .push(json!({
            "id": "synthetic-session",
            "kind": "measurement_session",
            "title": "Synthetic test session",
            "url": null,
            "sha256": null,
            "notes": "Generated only for deterministic unit testing."
        }));
    value["photographs"] = json!([{
        "id": "synthetic-photo",
        "path": "synthetic/not-a-physical-airframe.jpg",
        "url": null,
        "sha256": null,
        "description": "Synthetic evidence placeholder used only by tests."
    }]);
    value["datum"]["wing_root_le_established"] = json!(true);
    value["datum"]["source_ids"] = json!(["synthetic-session"]);
    value["datum"]["photograph_ids"] = json!(["synthetic-photo"]);
    value["acceptance_criteria"]["maximum_station_asymmetry_m"] = json!(0.3);
    value["acceptance_criteria"]["cross_variant_identity_tolerance_m"] = json!(0.01);

    let observations = &mut value["raw_observations"];
    observations["horizontal_tail_root_le_aft_wing_le_m"]["left"] = series([1.99, 2.0, 2.01]);
    observations["horizontal_tail_root_le_aft_wing_le_m"]["right"] = series([2.19, 2.2, 2.21]);
    observations["vertical_tail_root_le_aft_wing_le_m"] = series([2.4, 2.4, 2.4]);
    observations["wing_quarter_chord_aft_wing_le_m"] = series([0.5, 0.5, 0.5]);

    observations["horizontal_tail_planform"]["span_m"] = series([2.0, 2.0, 2.0]);
    observations["horizontal_tail_planform"]["root_chord_m"] = series([1.0, 1.0, 1.0]);
    observations["horizontal_tail_planform"]["tip_chord_m"] = series([1.0, 1.0, 1.0]);
    observations["horizontal_tail_planform"]["tip_le_offset_aft_root_le_m"] =
        series([0.0, 0.0, 0.0]);

    observations["vertical_tail_planform"]["height_m"] = series([1.0, 1.0, 1.0]);
    observations["vertical_tail_planform"]["root_chord_m"] = series([0.8, 0.8, 0.8]);
    observations["vertical_tail_planform"]["tip_chord_m"] = series([0.8, 0.8, 0.8]);
    observations["vertical_tail_planform"]["tip_le_offset_aft_root_le_m"] = series([0.0, 0.0, 0.0]);

    observations["wing_incidence_rad"] = series([0.01, 0.01, 0.01]);
    observations["stabilizer_incidence_rad"] = series([0.0, 0.0, 0.0]);
    observations["motor_thrust_axis_top_view_rad"] = series([0.0, 0.0, 0.0]);
    observations["motor_thrust_axis_side_view_rad"] = series([-0.02, -0.02, -0.02]);
    observations["operational_cg_aft_wing_le_m"] = series([0.7, 0.7, 0.7]);
    observations["battery"] = json!({
        "configuration_id": "synthetic-battery-config",
        "manufacturer": "Synthetic",
        "model": "Fixture",
        "cell_count": 4,
        "nominal_capacity_ah": 4.0,
        "location_description": "Synthetic longitudinal station fixture.",
        "longitudinal_station_aft_wing_le_m": series([0.6, 0.6, 0.6]),
        "source_ids": ["synthetic-session"],
        "photograph_ids": ["synthetic-photo"],
        "notes": "Not a physical reference-aircraft configuration."
    });
    value
}

fn load(value: &Value) -> Result<model::PhysicalSurvey, ReferenceSurveyError> {
    PhysicalSurveyLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

fn assert_close(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1.0e-12,
        "{actual} != {expected}"
    );
}

#[test]
fn committed_empty_campaign_is_valid_but_unresolved_and_never_runtime_ready() {
    let survey = PhysicalSurveyLoader::from_json_str(EMPTY_CAMPAIGN).unwrap();
    let evaluation = survey.evaluation();

    assert_eq!(
        survey.classification(),
        SurveyClassification::PhysicalReferenceMeasurement
    );
    assert!(survey.airframe_id().is_none());
    assert!(!evaluation.geometry_ready());
    assert!(!evaluation.campaign_complete());
    assert!(!evaluation.runtime_ready());
    assert!(evaluation.horizontal_tail_quarter_chord_arm().is_none());
    assert!(evaluation.vertical_tail_quarter_chord_arm().is_none());
    assert!(
        evaluation
            .missing_geometry_observations()
            .contains(&"wing_root_le_datum_with_evidence")
    );
    assert!(
        evaluation
            .missing_geometry_observations()
            .contains(&"maximum_station_asymmetry_m_acceptance_criterion")
    );
    assert!(
        evaluation
            .cross_variant_comparisons()
            .iter()
            .all(|comparison| comparison.status() == CrossVariantStatus::Unknown)
    );
}

#[test]
fn synthetic_campaign_aggregates_three_readings_and_preserves_bilateral_asymmetry() {
    let value = complete_synthetic_campaign();
    let survey = load(&value).unwrap();
    let bilateral = survey
        .evaluation()
        .horizontal_tail_station_bilateral()
        .unwrap();

    assert_close(bilateral.left().mean(), 2.0);
    assert_close(bilateral.left().range(), 0.02);
    assert_close(bilateral.right().mean(), 2.2);
    assert_close(bilateral.combined_mean(), 2.1);
    assert_close(bilateral.asymmetry_right_minus_left(), 0.2);
    assert_close(bilateral.effective_uncertainty(), 0.1);
}

#[test]
fn complete_synthetic_campaign_closes_geometry_from_egv_observations_only() {
    let survey = load(&complete_synthetic_campaign()).unwrap();
    let evaluation = survey.evaluation();

    assert_eq!(
        survey.classification(),
        SurveyClassification::SyntheticNonReference
    );
    assert_close(
        evaluation
            .horizontal_tail_planform_quarter_chord_offset()
            .unwrap()
            .value(),
        0.25,
    );
    assert_close(
        evaluation
            .vertical_tail_planform_quarter_chord_offset()
            .unwrap()
            .value(),
        0.2,
    );
    assert_close(
        evaluation
            .horizontal_tail_quarter_chord_station()
            .unwrap()
            .value(),
        2.35,
    );
    assert_close(
        evaluation
            .vertical_tail_quarter_chord_station()
            .unwrap()
            .value(),
        2.6,
    );
    assert_close(
        evaluation
            .horizontal_tail_quarter_chord_arm()
            .unwrap()
            .value(),
        1.85,
    );
    assert_close(
        evaluation
            .vertical_tail_quarter_chord_arm()
            .unwrap()
            .value(),
        2.1,
    );
    assert!(evaluation.geometry_ready());
    assert!(evaluation.campaign_complete());
    assert!(!evaluation.runtime_ready());
}

#[test]
fn missing_egv_wing_station_cannot_inherit_the_legacy_comparison_value() {
    let mut value = complete_synthetic_campaign();
    value["raw_observations"]["wing_quarter_chord_aft_wing_le_m"] = Value::Null;
    let survey = load(&value).unwrap();
    let evaluation = survey.evaluation();

    assert!(evaluation.wing_quarter_chord_station().is_none());
    assert!(evaluation.horizontal_tail_quarter_chord_arm().is_none());
    assert!(evaluation.vertical_tail_quarter_chord_arm().is_none());
    assert!(!evaluation.geometry_ready());
    let wing_comparison = evaluation
        .cross_variant_comparisons()
        .iter()
        .find(|comparison| comparison.quantity() == "wing_quarter_chord_offset_m")
        .unwrap();
    assert_eq!(wing_comparison.status(), CrossVariantStatus::Unknown);
}

#[test]
fn incomplete_or_out_of_tolerance_campaigns_do_not_pass_geometry_gate() {
    let mut missing_right = complete_synthetic_campaign();
    missing_right["raw_observations"]["horizontal_tail_root_le_aft_wing_le_m"]["right"] =
        Value::Null;
    assert!(!load(&missing_right).unwrap().evaluation().geometry_ready());

    let mut asymmetric = complete_synthetic_campaign();
    asymmetric["acceptance_criteria"]["maximum_station_asymmetry_m"] = json!(0.05);
    let evaluation = load(&asymmetric).unwrap();
    assert!(!evaluation.evaluation().geometry_ready());
    assert!(
        evaluation
            .evaluation()
            .missing_geometry_observations()
            .contains(&"horizontal_tail_station_asymmetry_within_tolerance")
    );
}

#[test]
fn cross_variant_categories_preserve_identity_semantics() {
    fn span_status(value: &Value) -> CrossVariantStatus {
        load(value)
            .unwrap()
            .evaluation()
            .cross_variant_comparisons()
            .iter()
            .find(|comparison| comparison.quantity() == "horizontal_tail.span_m")
            .unwrap()
            .status()
    }

    let mut confirmed = complete_synthetic_campaign();
    confirmed["acceptance_criteria"]["cross_variant_identity_tolerance_m"] = json!(0.001);
    confirmed["raw_observations"]["horizontal_tail_planform"]["span_m"] =
        series([0.6863, 0.6863, 0.6863]);
    assert_eq!(
        span_status(&confirmed),
        CrossVariantStatus::ConfirmedIdentical
    );

    let mut consistent = complete_synthetic_campaign();
    consistent["acceptance_criteria"]["cross_variant_identity_tolerance_m"] = json!(0.001);
    consistent["raw_observations"]["horizontal_tail_planform"]["span_m"] =
        series([0.6878, 0.6878, 0.6878]);
    assert_eq!(
        span_status(&consistent),
        CrossVariantStatus::ConsistentButNotProven
    );

    let mut different = complete_synthetic_campaign();
    different["acceptance_criteria"]["cross_variant_identity_tolerance_m"] = json!(0.001);
    different["raw_observations"]["horizontal_tail_planform"]["span_m"] = series([0.7, 0.7, 0.7]);
    assert_eq!(span_status(&different), CrossVariantStatus::Different);

    let mut unknown = complete_synthetic_campaign();
    unknown["acceptance_criteria"]["cross_variant_identity_tolerance_m"] = json!(0.001);
    unknown["raw_observations"]["horizontal_tail_planform"]["span_m"] = Value::Null;
    assert_eq!(span_status(&unknown), CrossVariantStatus::Unknown);
}

#[test]
fn invalid_measurements_and_unresolved_evidence_fail_closed() {
    let mut zero_resolution = complete_synthetic_campaign();
    zero_resolution["raw_observations"]["vertical_tail_root_le_aft_wing_le_m"]["instrument_resolution"] =
        json!(0.0);
    assert!(matches!(
        load(&zero_resolution),
        Err(ReferenceSurveyError::InvalidMeasurement { .. })
    ));

    let mut impossible_length = complete_synthetic_campaign();
    impossible_length["raw_observations"]["vertical_tail_root_le_aft_wing_le_m"]["readings"] =
        json!([-1.0, -1.0, -1.0]);
    assert!(matches!(
        load(&impossible_length),
        Err(ReferenceSurveyError::InvalidMeasurement { .. })
    ));

    let mut missing_source = complete_synthetic_campaign();
    missing_source["raw_observations"]["wing_quarter_chord_aft_wing_le_m"]["source_ids"] =
        json!(["absent-source"]);
    assert!(matches!(
        load(&missing_source),
        Err(ReferenceSurveyError::UnresolvedSourceReference { .. })
    ));

    let non_finite = serde_json::to_string(&complete_synthetic_campaign())
        .unwrap()
        .replace("[2.4,2.4,2.4]", "[1e400,1e400,1e400]");
    assert!(PhysicalSurveyLoader::from_json_str(&non_finite).is_err());

    let mut impossible_date = complete_synthetic_campaign();
    impossible_date["campaign"]["measurement_date"] = json!("2030-02-30");
    assert!(matches!(
        load(&impossible_date),
        Err(ReferenceSurveyError::InvalidMetadata {
            field: "campaign.measurement_date",
            ..
        })
    ));
}

#[test]
fn finite_measurements_cannot_overflow_evidence_summaries() {
    let mut stable_large_mean = complete_synthetic_campaign();
    stable_large_mean["raw_observations"]["vertical_tail_root_le_aft_wing_le_m"] =
        series([1.0e308, 1.0e308, 1.0e308]);
    let survey = load(&stable_large_mean).unwrap();
    assert_eq!(
        survey
            .evaluation()
            .vertical_tail_root_le_station()
            .unwrap()
            .value(),
        1.0e308
    );

    let mut unrepresentable_spread = complete_synthetic_campaign();
    unrepresentable_spread["raw_observations"]["horizontal_tail_planform"]["tip_le_offset_aft_root_le_m"] =
        series([-1.0e308, 0.0, 1.0e308]);
    assert!(matches!(
        load(&unrepresentable_spread),
        Err(ReferenceSurveyError::InvalidMeasurement { .. })
    ));
}

#[test]
fn survey_metadata_is_outside_the_aircraft_runtime_fingerprint() {
    let model_before = load_value(&valid_v2_reference_model_value()).unwrap();
    let fingerprint_before = model_before.physics_fingerprint();

    let mut first = complete_synthetic_campaign();
    let mut second = first.clone();
    first["campaign"]["notes"] = json!("First documentary note.");
    second["campaign"]["notes"] = json!("Different documentary note.");
    PhysicalSurveyLoader::from_json_str(&serde_json::to_string(&first).unwrap()).unwrap();
    PhysicalSurveyLoader::from_json_str(&serde_json::to_string(&second).unwrap()).unwrap();

    let model_after = load_value(&valid_v2_reference_model_value()).unwrap();
    assert_eq!(fingerprint_before, model_after.physics_fingerprint());
}
