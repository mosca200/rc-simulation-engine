mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    AerodynamicEvidenceLoader, MassPropertiesLoader, PhysicalConfigurationIdentity,
    PhysicalSurveyLoader, ReadinessDomain, ReadinessReason, ReadinessStatus,
    ReferenceReadinessInput, evaluate_reference_aircraft_readiness,
};
use serde_json::{Value, json};

const SURVEY: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_physical_survey_v0.json"
);
const MASS: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_mass_properties_v0.json"
);
const AERO: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json"
);

fn input<'a>(model: &'a model::AircraftModel) -> ReferenceReadinessInput<'a> {
    ReferenceReadinessInput {
        model,
        physical_configuration: PhysicalConfigurationIdentity {
            airframe_id: "test-only-airframe",
            operational_configuration_id: "test-only-configuration",
            propulsion_configuration_id: Some("test-only-propulsion"),
        },
        survey: None,
        mass_campaign: None,
        aerodynamic_evidence: None,
        propulsion_evidence: None,
        required_alpha_rad: Some((-0.2, 0.2)),
    }
}

fn has(
    report: &model::ReferenceAircraftReadiness,
    domain: ReadinessDomain,
    reason: ReadinessReason,
) -> bool {
    report
        .domain(domain)
        .findings
        .iter()
        .any(|f| f.reason == reason)
}

#[test]
fn synthetic_aircraft_is_never_physical_reference_ready() {
    let model = load_value(&valid_model_value()).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert_eq!(report.overall_status, ReadinessStatus::Blocked);
    assert!(has(
        &report,
        ReadinessDomain::AircraftIdentity,
        ReadinessReason::SyntheticModel
    ));
}

#[test]
fn reference_without_external_evidence_is_incomplete() {
    let model = load_value(&valid_v2_reference_model_value()).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert_eq!(report.overall_status, ReadinessStatus::Incomplete);
    assert!(has(
        &report,
        ReadinessDomain::AircraftIdentity,
        ReadinessReason::PhysicalSurveyMissing
    ));
    assert!(has(
        &report,
        ReadinessDomain::MassProperties,
        ReadinessReason::MassCampaignMissing
    ));
    assert!(has(
        &report,
        ReadinessDomain::Aerodynamics,
        ReadinessReason::AerodynamicEvidenceMissing
    ));
    assert_eq!(
        report.domain(ReadinessDomain::FlightTestData).status,
        ReadinessStatus::Incomplete
    );
}

#[test]
fn powered_aircraft_without_propulsion_evidence_is_incomplete_in_propulsion() {
    let value: Value = serde_json::from_str(include_str!(
        "../../../tests/fixtures/synthetic_non_reference_propulsion_v4.json"
    ))
    .unwrap();
    let model = load_value(&value).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert!(has(
        &report,
        ReadinessDomain::Propulsion,
        ReadinessReason::PropulsionEvidenceMissing
    ));
    assert_eq!(
        report.domain(ReadinessDomain::Propulsion).status,
        ReadinessStatus::Incomplete
    );
}

#[test]
fn glider_propulsion_is_not_applicable() {
    let mut value = valid_v2_reference_model_value();
    value["propulsion"] = Value::Null;
    let model = load_value(&value).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert_eq!(
        report.domain(ReadinessDomain::Propulsion).status,
        ReadinessStatus::NotApplicable
    );
    assert!(
        report
            .domain(ReadinessDomain::Propulsion)
            .findings
            .is_empty()
    );
}

#[test]
fn incomplete_geometry_and_mass_are_distinct() {
    let mut value = valid_v2_reference_model_value();
    value["reference_aircraft"]["physical_specification"]["aerodynamic_reference_chord_m"] = json!({
        "value": 0.3,
        "status": "manufacturer_spec",
        "source_ids": ["manufacturer-sheet"]
    });
    let model = load_value(&value).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert!(!has(
        &report,
        ReadinessDomain::Geometry,
        ReadinessReason::ReferenceChordMissing
    ));
    assert!(has(
        &report,
        ReadinessDomain::MassProperties,
        ReadinessReason::MassCampaignMissing
    ));
}

#[test]
fn unresolved_model_parameter_provenance_blocks() {
    let mut value = valid_v2_reference_model_value();
    value["reference_aircraft"]["physical_specification"]["wingspan_m"]["status"] =
        json!("unknown");
    let model = load_value(&value).unwrap();
    let report = evaluate_reference_aircraft_readiness(input(&model));
    assert!(has(
        &report,
        ReadinessDomain::Geometry,
        ReadinessReason::EvidenceUnresolved
    ));
    assert_eq!(
        report.domain(ReadinessDomain::Geometry).status,
        ReadinessStatus::Blocked
    );
}

#[test]
fn missing_alpha_requirement_fails_closed() {
    let model = load_value(&valid_v2_reference_model_value()).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(AERO).unwrap();
    let mut request = input(&model);
    request.aerodynamic_evidence = Some(&evidence);
    request.required_alpha_rad = None;
    let report = evaluate_reference_aircraft_readiness(request);
    assert!(has(
        &report,
        ReadinessDomain::Aerodynamics,
        ReadinessReason::AlphaEnvelopeMissing
    ));
}

#[test]
fn missing_reynolds_and_polar_coverage_are_separately_visible() {
    let model = load_value(&valid_v2_reference_model_value()).unwrap();
    let evidence = AerodynamicEvidenceLoader::from_json_str(AERO).unwrap();
    let mut request = input(&model);
    request.aerodynamic_evidence = Some(&evidence);
    let report = evaluate_reference_aircraft_readiness(request);
    assert!(has(
        &report,
        ReadinessDomain::Aerodynamics,
        ReadinessReason::ReynoldsCoverageInsufficient
    ));
    assert!(has(
        &report,
        ReadinessDomain::Aerodynamics,
        ReadinessReason::PolarEvidenceUnresolved
    ));
}

#[test]
fn physical_survey_and_mass_template_cannot_be_promoted() {
    let model = load_value(&valid_v2_reference_model_value()).unwrap();
    let survey = PhysicalSurveyLoader::from_json_str(SURVEY).unwrap();
    let mass = MassPropertiesLoader::from_json_str(MASS).unwrap();
    let mut request = input(&model);
    request.survey = Some(&survey);
    request.mass_campaign = Some(&mass);
    let report = evaluate_reference_aircraft_readiness(request);
    assert!(has(
        &report,
        ReadinessDomain::Geometry,
        ReadinessReason::TailSurveyIncomplete
    ));
    assert!(has(
        &report,
        ReadinessDomain::MassProperties,
        ReadinessReason::MassMeasurementMissing
    ));
    assert!(has(
        &report,
        ReadinessDomain::MassProperties,
        ReadinessReason::CgMeasurementMissing
    ));
    assert!(has(
        &report,
        ReadinessDomain::MassProperties,
        ReadinessReason::InertiaEvidenceMissing
    ));
}

#[test]
fn repeated_evaluation_and_finding_order_are_exactly_equal() {
    let model = load_value(&valid_v2_reference_model_value()).unwrap();
    let before = model.physics_fingerprint();
    let a = evaluate_reference_aircraft_readiness(input(&model));
    let b = evaluate_reference_aircraft_readiness(input(&model));
    assert_eq!(a, b);
    assert_eq!(model.physics_fingerprint(), before);
    assert_eq!(a.findings[0].0, ReadinessDomain::AircraftIdentity);
    assert_eq!(
        a.findings[0].1.reason,
        ReadinessReason::PhysicalSurveyMissing
    );
    assert_eq!(
        a.findings.last().unwrap().0,
        ReadinessDomain::FlightTestData
    );
}

#[test]
fn legacy_models_still_load_and_are_not_reference_ready() {
    for value in [valid_model_value(), valid_v1_model_value()] {
        let model = load_value(&value).unwrap();
        assert_eq!(
            evaluate_reference_aircraft_readiness(input(&model)).overall_status,
            ReadinessStatus::Blocked
        );
    }
}

/// Readiness evaluation is metadata only, so the crate that hosts it must stay free of
/// renderer, windowing and gamepad dependencies. Mirrors the telemetry crate's guard.
#[test]
fn model_crate_has_no_renderer_platform_or_gpu_dependency() {
    let manifest_path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml");
    let manifest = std::fs::read_to_string(manifest_path).expect("model manifest readable");
    for forbidden in ["renderer", "wgpu", "winit", "platform", "gilrs"] {
        assert!(
            !manifest.lines().any(|line| line
                .split_once('=')
                .is_some_and(|(name, _)| name.trim() == forbidden)),
            "forbidden model dependency {forbidden}"
        );
    }
}
