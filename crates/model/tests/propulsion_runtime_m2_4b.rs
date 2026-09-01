mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V4, AircraftClassification, AircraftModel, AircraftModelLoader,
    ModelLoadError,
};
use serde_json::{Value, json};
use sim_core::{EscConfigError, PropellerCoefficientMapError, PropellerCoefficientSource};

const SYNTHETIC_V4: &str =
    include_str!("../../../tests/fixtures/synthetic_non_reference_propulsion_v4.json");

fn value_v4() -> Value {
    serde_json::from_str(SYNTHETIC_V4).unwrap()
}

fn load_v4(value: &Value) -> AircraftModel {
    AircraftModelLoader::from_json_str(&serde_json::to_string(value).unwrap()).unwrap()
}

#[test]
fn v4_synthetic_fixture_loads_explicit_complete_propulsion() {
    let model = load_v4(&value_v4());
    assert_eq!(model.schema_version(), AIRCRAFT_MODEL_SCHEMA_VERSION_V4);
    assert_eq!(
        model.classification(),
        AircraftClassification::SyntheticTest
    );
    assert!(model.reference_aircraft().is_none());
    let propulsion = model.propulsion().unwrap();
    assert_eq!(propulsion.config().esc().series_resistance_ohm(), 0.012);
    assert!(matches!(
        propulsion.coefficient_source(),
        PropellerCoefficientSource::ShaftSpeedMap(map) if map.nodes().len() == 2
    ));
    let lower = SYNTHETIC_V4.to_ascii_lowercase();
    for forbidden in ["sig", "kadet", "lt-40", "apc", "himax", "castle"] {
        assert!(!lower.contains(forbidden));
    }
}

#[test]
fn v4_fixed_table_source_loads_and_unknown_or_missing_fields_fail_closed() {
    let mut fixed = value_v4();
    fixed["propulsion"]["coefficient_source"] = json!({
        "kind": "fixed_table",
        "samples": [
            {"advance_ratio_j": 0.0, "ct": 0.11, "cq": 0.017},
            {"advance_ratio_j": 1.0, "ct": 0.02, "cq": 0.005}
        ]
    });
    assert!(matches!(
        load_v4(&fixed).propulsion().unwrap().coefficient_source(),
        PropellerCoefficientSource::FixedTable(_)
    ));

    let mut missing_esc = fixed.clone();
    missing_esc["propulsion"]
        .as_object_mut()
        .unwrap()
        .remove("esc");
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&missing_esc).unwrap()),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
    let mut unknown = fixed;
    unknown["propulsion"]["coefficient_source"]["extrapolate"] = json!(true);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&unknown).unwrap()),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn v4_invalid_esc_and_map_nodes_report_semantic_errors() {
    let mut invalid_esc = value_v4();
    invalid_esc["propulsion"]["esc"]["series_resistance_ohm"] = json!(-0.01);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&invalid_esc).unwrap()),
        Err(ModelLoadError::InvalidEsc {
            source: EscConfigError::InvalidSeriesResistance
        })
    ));

    let mut duplicate = value_v4();
    duplicate["propulsion"]["coefficient_source"]["nodes"][1]["shaft_speed_rad_s"] = json!(240.0);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&duplicate).unwrap()),
        Err(ModelLoadError::InvalidPropellerCoefficientMap {
            node_index: None,
            source: PropellerCoefficientMapError::NonIncreasingShaftSpeed { index: 1 }
        })
    ));

    let mut empty = value_v4();
    empty["propulsion"]["coefficient_source"]["nodes"] = json!([]);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&serde_json::to_string(&empty).unwrap()),
        Err(ModelLoadError::InvalidPropellerCoefficientMap {
            node_index: None,
            source: PropellerCoefficientMapError::Empty
        })
    ));
}

#[test]
fn v4_fingerprint_covers_esc_source_nodes_and_samples() {
    let baseline = value_v4();
    let fingerprint = load_v4(&baseline).physics_fingerprint();
    for (pointer, replacement) in [
        ("/propulsion/esc/series_resistance_ohm", json!(0.013)),
        (
            "/propulsion/coefficient_source/nodes/0/shaft_speed_rad_s",
            json!(230.0),
        ),
        (
            "/propulsion/coefficient_source/nodes/0/samples/0/ct",
            json!(0.102),
        ),
        (
            "/propulsion/coefficient_source/nodes/1/samples/1/advance_ratio_j",
            json!(0.36),
        ),
    ] {
        let mut changed = baseline.clone();
        *changed.pointer_mut(pointer).unwrap() = replacement;
        assert_ne!(fingerprint, load_v4(&changed).physics_fingerprint());
    }

    let mut fixed = baseline;
    fixed["propulsion"]["coefficient_source"] = json!({
        "kind": "fixed_table",
        "samples": [
            {"advance_ratio_j": 0.0, "ct": 0.101, "cq": 0.0151},
            {"advance_ratio_j": 0.6, "ct": 0.061, "cq": 0.0102}
        ]
    });
    assert_ne!(fingerprint, load_v4(&fixed).physics_fingerprint());
}

#[test]
fn legacy_models_remain_compatible_and_v0_fingerprint_is_unchanged() {
    let v0 = load_value(&valid_model_value()).unwrap();
    assert_eq!(
        v0.physics_fingerprint().as_bytes(),
        &[
            0x07, 0x3c, 0x7e, 0x94, 0x77, 0x25, 0x56, 0x1e, 0xea, 0xbb, 0xf6, 0x0b, 0xe6, 0x8f,
            0x5c, 0xd5, 0x17, 0x1f, 0x8f, 0xf2, 0x7d, 0x20, 0x99, 0x25, 0x3c, 0xdf, 0xed, 0xa7,
            0x3c, 0x40, 0x31, 0xc2,
        ]
    );
    for value in [valid_v1_model_value(), valid_v2_reference_model_value()] {
        let model = load_value(&value).unwrap();
        assert_eq!(
            model
                .propulsion()
                .unwrap()
                .config()
                .esc()
                .series_resistance_ohm(),
            0.0
        );
        assert!(matches!(
            model.propulsion().unwrap().coefficient_source(),
            PropellerCoefficientSource::FixedTable(_)
        ));
    }
}
