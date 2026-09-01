mod common;

use std::path::PathBuf;

use common::valid_model_value;
use model::{
    AircraftModelLoader, ApcPerformanceDataLoader, PropulsionConfigurationEvidenceClass,
    PropulsionEvidenceLoader, ReferencePropulsionEvidenceError, load_reference_propulsion_evidence,
};
use serde_json::{Value, json};

const COMMITTED_EVIDENCE: &str = include_str!(
    "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_propulsion_evidence_v0.json"
);

fn committed_value() -> Value {
    serde_json::from_str(COMMITTED_EVIDENCE).expect("committed propulsion evidence JSON")
}

fn empty_template() -> Value {
    let mut value = committed_value();
    value["campaign"]["id"] = json!("synthetic-empty-propulsion-evidence");
    value["campaign"]["classification"] = json!("synthetic_non_reference");
    value["campaign"]["airframe_id"] = json!("synthetic-airframe");
    value["provenance_sources"] = json!([]);
    value["configuration_claims"] = json!([]);
    value["motors"] = json!([]);
    value["escs"] = json!([]);
    value["batteries"] = json!([]);
    value["propellers"] = json!([]);
    value["spinners"] = json!([]);
    value["propeller_datasets"] = json!([]);
    value
}

fn load(value: &Value) -> Result<model::PropulsionEvidence, ReferencePropulsionEvidenceError> {
    PropulsionEvidenceLoader::from_json_str(&serde_json::to_string(value).unwrap())
}

fn evidence_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../docs/reference_aircraft/data/sig_kadet_lt40_egv_propulsion_evidence_v0.json")
}

fn synthetic_apc_fixture() -> &'static str {
    "SyntheticProp (synthetic.dat)\n\
vTEST-ONLY\n\
Simulation Date: 01/02/2030\n\
DEFINITIONS:\n\
J=V/nD (advance ratio)\n\
Ct=T/(rho * n**2 * D**4) (thrust coef.)\n\
Cp=P/(rho * n**3 * D**5) (power coef.)\n\
PROP RPM = 1000\n\
0.0 0.0 0.0 0.10 0.20 0 0 0 0 0 0 0 0 1 0\n\
1.0 0.1 0.05 0.09 0.18 0 0 0 0 0 0 0 0 1 0\n\
PROP RPM = 2000\n\
0.0 0.0 0.0 0.11 0.22 0 0 0 0 0 0 0 0 1 0\n\
1.0 0.1 0.05 0.10 0.20 0 0 0 0 0 0 0 0 1 0\n"
}

#[test]
fn empty_template_is_valid_and_unresolved() {
    let evidence = load(&empty_template()).unwrap();
    let evaluation = evidence.evaluation();
    assert!(!evaluation.configuration_identified());
    assert!(!evaluation.propulsion_evidence_ready());
    assert!(!evaluation.runtime_ready());
}

#[test]
fn committed_template_loads_linked_apc_data_but_remains_unresolved() {
    let evidence = load_reference_propulsion_evidence(evidence_path()).unwrap();
    let evaluation = evidence.evaluation();
    assert_eq!(
        evidence.campaign_id(),
        "sig-kadet-lt40-egv-propulsion-evidence-m2-4a"
    );
    assert!(evidence.apc_dataset("apc-per3-11x7e-v2022-0915").is_some());
    assert!(!evaluation.configuration_identified());
    assert!(!evaluation.motor_evidence_ready());
    assert!(!evaluation.esc_evidence_ready());
    assert!(!evaluation.battery_evidence_ready());
    assert!(!evaluation.propeller_evidence_ready());
    assert!(!evaluation.propulsion_evidence_ready());
    assert!(!evaluation.runtime_ready());
}

#[test]
fn unknown_is_null_and_not_zero() {
    let mut value = committed_value();
    assert!(value["batteries"][0]["internal_resistance_ohm"].is_null());
    value["batteries"][0]["internal_resistance_ohm"] = json!(0.0);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn duplicate_source_ids_are_rejected() {
    let mut value = committed_value();
    let source = value["provenance_sources"][0].clone();
    value["provenance_sources"]
        .as_array_mut()
        .unwrap()
        .push(source);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::DuplicateStableId { .. })
    ));
}

#[test]
fn duplicate_component_ids_are_rejected() {
    let mut value = committed_value();
    let motor = value["motors"][0].clone();
    value["motors"].as_array_mut().unwrap().push(motor);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::DuplicateStableId { .. })
    ));
}

#[test]
fn unresolved_provenance_is_rejected() {
    let mut value = committed_value();
    value["motors"][0]["source_ids"] = json!(["missing-source"]);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::UnresolvedReference { .. })
    ));
}

#[test]
fn unresolved_photo_is_rejected() {
    let mut value = committed_value();
    value["configuration_claims"][1]["photograph_ids"] = json!(["missing-photo"]);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::UnresolvedReference { .. })
    ));
}

#[test]
fn invalid_kv_is_rejected() {
    let mut value = committed_value();
    value["motors"][0]["kv_rpm_per_v"] = json!(0.0);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn invalid_motor_resistance_is_rejected() {
    let mut value = committed_value();
    value["motors"][0]["winding_resistance_ohm"] = json!(-0.1);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn invalid_current_is_rejected() {
    let mut value = committed_value();
    value["escs"][0]["current_rating_a"] = json!(-50.0);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn invalid_battery_cell_count_is_rejected() {
    let mut value = committed_value();
    value["batteries"][0]["cell_count"] = json!(0);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn invalid_battery_load_point_is_rejected() {
    let mut value = committed_value();
    value["batteries"][0]["voltage_load_points"] = json!([{
        "state_of_charge": 1.2,
        "load_current_a": 10.0,
        "voltage_v": 11.0,
        "temperature_c": 20.0
    }]);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn duplicate_battery_load_points_are_rejected() {
    let mut value = committed_value();
    let point = json!({
        "state_of_charge": 0.5,
        "load_current_a": 10.0,
        "voltage_v": 11.0,
        "temperature_c": 20.0
    });
    value["batteries"][0]["voltage_load_points"] = json!([point.clone(), point]);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn synthetic_apc_like_fixture_parses() {
    let parsed = ApcPerformanceDataLoader::parse_str(synthetic_apc_fixture()).unwrap();
    assert_eq!(parsed.propeller_designation(), "SyntheticProp");
    assert_eq!(parsed.source_version(), "vTEST-ONLY");
    assert_eq!(parsed.blocks().len(), 2);
    assert_eq!(parsed.row_count(), 4);
}

#[test]
fn malformed_apc_row_is_rejected() {
    let malformed = synthetic_apc_fixture().replace(
        "1.0 0.1 0.05 0.09 0.18 0 0 0 0 0 0 0 0 1 0",
        "1.0 0.1 0.05 0.09",
    );
    assert!(matches!(
        ApcPerformanceDataLoader::parse_str(&malformed),
        Err(ReferencePropulsionEvidenceError::MalformedApcData { .. })
    ));
}

#[test]
fn unordered_apc_advance_ratio_is_rejected() {
    let unordered =
        synthetic_apc_fixture().replace("1.0 0.1 0.05 0.09 0.18", "1.0 0.0 0.05 0.09 0.18");
    assert!(matches!(
        ApcPerformanceDataLoader::parse_str(&unordered),
        Err(ReferencePropulsionEvidenceError::MalformedApcData { .. })
    ));
}

#[test]
fn cq_is_deterministically_derived_from_cp() {
    let parsed = ApcPerformanceDataLoader::parse_str(synthetic_apc_fixture()).unwrap();
    let row = &parsed.blocks()[0].rows()[0];
    assert_eq!(row.cp(), Some(0.2));
    assert_eq!(row.cq_derived(), Some(0.2 / (2.0 * std::f64::consts::PI)));
}

#[test]
fn recommendation_is_not_an_installation() {
    let evidence = PropulsionEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    let claims = evidence.evaluation().configuration_claims();
    assert_eq!(
        claims[0].evidence_class(),
        PropulsionConfigurationEvidenceClass::ManufacturerRecommendation
    );
    assert!(claims[0].operational_configuration_id().is_none());
    assert!(!evidence.evaluation().configuration_identified());
}

#[test]
fn historical_configuration_is_not_physical_installation() {
    let evidence = PropulsionEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    let claims = evidence.evaluation().configuration_claims();
    assert_eq!(
        claims[1].evidence_class(),
        PropulsionConfigurationEvidenceClass::HistoricallyFlightTestedConfiguration
    );
    assert!(!evidence.evaluation().configuration_identified());
}

#[test]
fn incompatible_installed_configuration_identity_is_rejected() {
    let mut value = committed_value();
    value["configuration_claims"][1]["evidence_class"] = json!("specific_installed_configuration");
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::IncompatibleConfigurationIdentity { .. })
    ));
}

#[test]
fn source_applicability_must_match_claim_component_identity() {
    let mut value = committed_value();
    value["batteries"][0]["applicable_configuration_claim_ids"] =
        json!(["sig-historical-flight-test-4s"]);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn unsafe_linked_source_path_is_rejected() {
    let mut value = committed_value();
    value["propeller_datasets"][0]["raw_source_path"] = json!("../outside.dat");
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn inconsistent_propeller_dimensions_are_rejected() {
    let mut value = committed_value();
    value["propeller_datasets"][0]["diameter_m"] = json!(0.3);
    assert!(matches!(
        load(&value),
        Err(ReferencePropulsionEvidenceError::InvalidEvidence { .. })
    ));
}

#[test]
fn actual_apc_file_preserves_declared_order_and_counts() {
    let evidence = load_reference_propulsion_evidence(evidence_path()).unwrap();
    let data = evidence.apc_dataset("apc-per3-11x7e-v2022-0915").unwrap();
    assert_eq!(data.propeller_designation(), "11x7E");
    assert_eq!(data.source_version(), "v2022-0915");
    assert_eq!(data.simulation_date(), "09/22/2022");
    assert_eq!(data.blocks().len(), 19);
    assert_eq!(data.row_count(), 570);
    assert_eq!(data.coefficient_row_count(), 564);
    assert_eq!(data.blocks().first().unwrap().rpm(), 1000.0);
    assert_eq!(data.blocks().last().unwrap().rpm(), 19000.0);
}

#[test]
fn artifact_contains_no_recreated_coefficient_rows() {
    let value = committed_value();
    let dataset = &value["propeller_datasets"][0];
    assert!(dataset.get("rows").is_none());
    assert!(dataset.get("samples").is_none());
    assert_eq!(
        dataset["raw_source_path"],
        "sources/APC_PER3_11x7E_v2022-0915.dat"
    );
}

#[test]
fn evaluation_and_parser_ordering_are_deterministic() {
    let first = PropulsionEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    let second = PropulsionEvidenceLoader::from_json_str(COMMITTED_EVIDENCE).unwrap();
    assert_eq!(first.evaluation(), second.evaluation());
    let first_data = ApcPerformanceDataLoader::parse_str(synthetic_apc_fixture()).unwrap();
    let second_data = ApcPerformanceDataLoader::parse_str(synthetic_apc_fixture()).unwrap();
    assert_eq!(first_data, second_data);
}

#[test]
fn runtime_ready_is_always_false() {
    let evidence = load_reference_propulsion_evidence(evidence_path()).unwrap();
    assert!(!evidence.evaluation().runtime_ready());
}

#[test]
fn evidence_loading_does_not_change_existing_physics_fingerprint() {
    let model_json = serde_json::to_string(&valid_model_value()).unwrap();
    let before = AircraftModelLoader::from_json_str(&model_json)
        .unwrap()
        .physics_fingerprint();
    let evidence = load_reference_propulsion_evidence(evidence_path()).unwrap();
    assert!(!evidence.evaluation().runtime_ready());
    let after = AircraftModelLoader::from_json_str(&model_json)
        .unwrap()
        .physics_fingerprint();
    assert_eq!(before, after);
}
