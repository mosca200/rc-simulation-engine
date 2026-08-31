mod common;

use common::{load_value, valid_model_value, valid_v1_model_value, valid_v2_reference_model_value};
use model::{
    AircraftClassification, AircraftModelLoader, CgReferenceKind, ModelLoadError, ParameterQuality,
    ProvenanceConfidence, ProvenanceSourceType,
};
use serde_json::{Value, json};

#[test]
fn legacy_v0_and_v1_models_remain_synthetic_and_load_without_reference_metadata() {
    for value in [valid_model_value(), valid_v1_model_value()] {
        let model = load_value(&value).expect("legacy model remains accepted");
        assert_eq!(
            model.classification(),
            AircraftClassification::SyntheticTest
        );
        assert!(model.reference_aircraft().is_none());
    }
}

#[test]
fn schema_v2_synthetic_classification_is_explicit_and_rejects_reference_metadata() {
    let mut synthetic = valid_v1_model_value();
    synthetic["schema_version"] = json!(2);
    synthetic
        .as_object_mut()
        .unwrap()
        .insert("classification".to_owned(), json!("synthetic_test"));
    synthetic
        .as_object_mut()
        .unwrap()
        .insert("reference_aircraft".to_owned(), Value::Null);
    let model = load_value(&synthetic).expect("explicit synthetic v2");
    assert_eq!(
        model.classification(),
        AircraftClassification::SyntheticTest
    );

    synthetic["reference_aircraft"] =
        valid_v2_reference_model_value()["reference_aircraft"].clone();
    assert!(matches!(
        load_value(&synthetic),
        Err(ModelLoadError::UnexpectedReferenceAircraftMetadata)
    ));
}

#[test]
fn reference_aircraft_identity_provenance_and_parameter_status_are_resolved() {
    let model = load_value(&valid_v2_reference_model_value()).expect("valid reference fixture");
    assert_eq!(model.schema_version(), 2);
    assert_eq!(
        model.classification(),
        AircraftClassification::ReferenceAircraft
    );
    let reference = model.reference_aircraft().expect("reference metadata");
    assert_eq!(
        reference.identity().manufacturer(),
        Some("Fixture Manufacturer")
    );
    assert_eq!(
        reference.identity().stable_reference_id(),
        Some("reference-fixture-01")
    );

    let sources = reference.provenance_sources();
    assert_eq!(sources.len(), 3);
    assert_eq!(
        sources[0].source_type(),
        ProvenanceSourceType::ManufacturerDocumentation
    );
    assert_eq!(sources[0].confidence(), Some(ProvenanceConfidence::High));
    assert_eq!(sources[0].publication_date(), Some("2024-01-02"));

    let specification = reference.physical_specification();
    let wingspan = specification.wingspan_m().expect("wingspan");
    assert_eq!(wingspan.value().to_bits(), 1.8_f64.to_bits());
    assert_eq!(
        wingspan.evidence().quality(),
        ParameterQuality::ManufacturerSpec
    );
    assert_eq!(wingspan.evidence().source_indices(), &[0]);
    assert_eq!(
        specification.mass().expect("mass evidence").quality(),
        ParameterQuality::Measured
    );
    let cg = specification.cg_location().expect("CG");
    assert_eq!(cg.reference_kind(), CgReferenceKind::WingRootLeadingEdge);
    assert_eq!(cg.position_m_from_reference(), &[0.12, 0.0, 0.0]);
    assert_eq!(specification.control_surface_travel_limits().len(), 1);
    assert_eq!(
        specification.control_surface_travel_limits()[0].binding_index(),
        0
    );
}

#[test]
fn reference_aircraft_accepts_fully_unknown_optional_specification() {
    let mut value = valid_v2_reference_model_value();
    value["reference_aircraft"]["identity"] = json!({
        "manufacturer": null,
        "aircraft_name": null,
        "variant": null,
        "stable_reference_id": null,
        "notes": null
    });
    value["reference_aircraft"]["physical_specification"] = json!({
        "wingspan_m": null,
        "reference_wing_area_m2": null,
        "aircraft_length_m": null,
        "mass": null,
        "cg_location": null,
        "aerodynamic_reference_chord_m": null,
        "wing_incidence_rad": null,
        "horizontal_tail_incidence_rad": null,
        "wing_dihedral_rad": null,
        "control_surface_travel_limits": []
    });
    value["reference_aircraft"]["provenance_sources"] = json!([]);

    let model = load_value(&value).expect("unknown reference data is valid");
    let reference = model.reference_aircraft().unwrap();
    assert!(reference.identity().manufacturer().is_none());
    assert!(reference.physical_specification().wingspan_m().is_none());
    assert!(reference.provenance_sources().is_empty());
}

#[test]
fn unknown_parameter_quality_is_preserved_as_typed_status() {
    let mut value = valid_v2_reference_model_value();
    value["reference_aircraft"]["physical_specification"]["wing_dihedral_rad"]["status"] =
        json!("unknown");

    let model = load_value(&value).expect("unknown quality is valid");
    let quality = model
        .reference_aircraft()
        .unwrap()
        .physical_specification()
        .wing_dihedral_rad()
        .unwrap()
        .evidence()
        .quality();
    assert_eq!(quality, ParameterQuality::Unknown);
}

#[test]
fn duplicate_malformed_and_unresolved_provenance_ids_are_rejected() {
    let mut duplicate = valid_v2_reference_model_value();
    duplicate["reference_aircraft"]["provenance_sources"][1]["id"] = json!("manufacturer-sheet");
    assert!(matches!(
        load_value(&duplicate),
        Err(ModelLoadError::DuplicateStableId {
            kind: "provenance source",
            ..
        })
    ));

    let mut malformed = valid_v2_reference_model_value();
    malformed["reference_aircraft"]["provenance_sources"][0]["id"] = json!("Bad ID");
    assert!(matches!(
        load_value(&malformed),
        Err(ModelLoadError::InvalidStableId {
            kind: "provenance source",
            ..
        })
    ));

    let mut unresolved = valid_v2_reference_model_value();
    unresolved["reference_aircraft"]["physical_specification"]["wingspan_m"]["source_ids"] =
        json!(["missing-source"]);
    assert!(matches!(
        load_value(&unresolved),
        Err(ModelLoadError::UnresolvedProvenanceReference { source_id, .. })
            if source_id == "missing-source"
    ));
}

#[test]
fn invalid_reference_dimensions_cg_and_control_travel_are_rejected() {
    for invalid in [0.0, -1.0] {
        let mut value = valid_v2_reference_model_value();
        value["reference_aircraft"]["physical_specification"]["wingspan_m"]["value"] =
            json!(invalid);
        assert!(matches!(
            load_value(&value),
            Err(ModelLoadError::InvalidReferencePhysicalValue {
                field: "physical_specification.wingspan_m",
                ..
            })
        ));
    }

    let mut cg = valid_v2_reference_model_value();
    cg["reference_aircraft"]["physical_specification"]["cg_location"]["reference"] = json!({
        "kind": "manufacturer_datum",
        "description": null
    });
    assert!(matches!(
        load_value(&cg),
        Err(ModelLoadError::InvalidReferenceCgDefinition { .. })
    ));

    let mut travel = valid_v2_reference_model_value();
    travel["controls"]["servos"]["aileron"]["min_angle_rad"] = json!(0.01);
    assert!(matches!(
        load_value(&travel),
        Err(ModelLoadError::InvalidControls { .. })
    ));

    let mut unresolved_binding = valid_v2_reference_model_value();
    unresolved_binding["reference_aircraft"]["physical_specification"]["control_surface_travel_limits"]
        [0]["control_surface_binding_id"] = json!("missing-binding");
    assert!(matches!(
        load_value(&unresolved_binding),
        Err(ModelLoadError::UnresolvedReferenceControlSurfaceBinding { .. })
    ));
}

#[test]
fn nonfinite_reference_values_are_rejected_by_strict_json_loading() {
    let json = serde_json::to_string(&valid_v2_reference_model_value()).unwrap();
    let overflow = json.replacen("\"value\":1.8", "\"value\":1e400", 1);
    assert!(matches!(
        AircraftModelLoader::from_json_str(&overflow),
        Err(ModelLoadError::InvalidStructure { .. })
    ));
}

#[test]
fn documentary_and_presentation_changes_do_not_change_physics_fingerprint() {
    let baseline = valid_v2_reference_model_value();
    let fingerprint = load_value(&baseline).unwrap().physics_fingerprint();

    let mut documentary = baseline.clone();
    documentary["reference_aircraft"]["identity"]["notes"] = json!("Different notes");
    documentary["reference_aircraft"]["provenance_sources"][0]["url"] =
        json!("https://example.invalid/changed");
    documentary["reference_aircraft"]["physical_specification"]["wingspan_m"]["value"] = json!(2.1);
    assert_eq!(
        fingerprint,
        load_value(&documentary).unwrap().physics_fingerprint()
    );

    let mut presentation = baseline;
    presentation["presentation"]["glb_path"] = json!("different/visual.glb");
    assert_eq!(
        fingerprint,
        load_value(&presentation).unwrap().physics_fingerprint()
    );
}

#[test]
fn v2_retains_v1_physics_identity_but_physics_mutations_change_it() {
    let v1 = valid_v1_model_value();
    let v2 = valid_v2_reference_model_value();
    let v1_fingerprint = load_value(&v1).unwrap().physics_fingerprint();
    assert_eq!(
        v1_fingerprint,
        load_value(&v2).unwrap().physics_fingerprint()
    );

    let mut changed_mass = v2;
    changed_mass["rigid_body"]["mass_kg"] = json!(2.6);
    assert_ne!(
        v1_fingerprint,
        load_value(&changed_mass).unwrap().physics_fingerprint()
    );
}

#[test]
fn v2_authoring_roundtrip_and_repeated_runtime_loading_are_deterministic() {
    let value = valid_v2_reference_model_value();
    let encoded = serde_json::to_string(&value).unwrap();
    let authoring: model::v2::AircraftModelFileV2 = serde_json::from_str(&encoded).unwrap();
    let roundtrip = serde_json::to_string(&authoring).unwrap();
    let first = AircraftModelLoader::from_json_str(&encoded).unwrap();
    let second = AircraftModelLoader::from_json_str(&roundtrip).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.physics_fingerprint(), second.physics_fingerprint());
}
