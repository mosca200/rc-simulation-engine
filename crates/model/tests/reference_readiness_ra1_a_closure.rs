//! RA1-A test-only mock. No value here is a physical-aircraft observation.
//! The mock artifacts deliberately use the physical-reference classification solely to
//! exercise the complete evaluator path. They must never be distributed as real evidence.

mod common;

use common::{load_value, valid_v2_reference_model_value};
use model::{
    AerodynamicEvidence, AerodynamicEvidenceLoader, AircraftModel, MassPropertiesCampaign,
    MassPropertiesLoader, PhysicalConfigurationIdentity, PhysicalSurvey, PhysicalSurveyLoader,
    ReadinessDomain, ReadinessReason, ReadinessStatus, ReferenceReadinessInput,
    evaluate_reference_aircraft_readiness,
};
use serde_json::{Value, json};

const MOCK_MANUFACTURER: &str = "RA1 MOCK ONLY";
const MOCK_FAMILY: &str = "RA1 Test Glider";
const MOCK_VARIANT: &str = "test-only-v1";
const MOCK_AIRFRAME: &str = "ra1-mock-airframe";
const MOCK_CONFIGURATION: &str = "ra1-mock-unpowered-config";
const MOCK_SURVEY_ID: &str = "ra1-mock-survey";
const MOCK_SOURCE: &str = "ra1-mock-session";
const MOCK_PHOTO: &str = "ra1-mock-photo";

fn model_value() -> Value {
    let mut value = valid_v2_reference_model_value();
    value["propulsion"] = Value::Null;
    value["reference_aircraft"]["identity"] = json!({
        "manufacturer": MOCK_MANUFACTURER,
        "aircraft_name": MOCK_FAMILY,
        "variant": MOCK_VARIANT,
        "stable_reference_id": "ra1-mock-glider",
        "notes": "TEST-ONLY MOCK; no real aircraft exists for this fixture."
    });
    value["reference_aircraft"]["physical_specification"]["aerodynamic_reference_chord_m"] = json!({
        "value": 0.3,
        "status": "manufacturer_spec",
        "source_ids": ["manufacturer-sheet"]
    });
    value["reference_aircraft"]["physical_specification"]["control_surface_travel_limits"] = json!([
        {
            "control_surface_binding_id": "aileron-first",
            "status": "manufacturer_spec",
            "source_ids": ["manufacturer-sheet"]
        },
        {
            "control_surface_binding_id": "elevator-second",
            "status": "manufacturer_spec",
            "source_ids": ["manufacturer-sheet"]
        }
    ]);
    value
}

fn survey_series(value: f64) -> Value {
    json!({
        "readings": [value, value, value],
        "instrument_resolution": 0.001,
        "stated_uncertainty": 0.002,
        "datum_definition": "TEST-ONLY MOCK wing-root datum; not physically measured.",
        "notes": "Synthetic readings generated only for RA1-A test coverage.",
        "source_ids": [MOCK_SOURCE],
        "photograph_ids": [MOCK_PHOTO]
    })
}

fn survey_value() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_physical_survey_v0.json"
    ))
    .unwrap();
    value["campaign"] = json!({
        "id": MOCK_SURVEY_ID,
        "classification": "physical_reference_measurement",
        "identity": {
            "manufacturer": MOCK_MANUFACTURER,
            "family": MOCK_FAMILY,
            "variant": MOCK_VARIANT,
            "airframe_id": MOCK_AIRFRAME
        },
        "measurement_date": "2030-01-02",
        "notes": "TEST-ONLY MOCK classification to exercise the reference gate; not physical evidence."
    });
    value["datum"] = json!({
        "wing_root_le_established": true,
        "definition": "TEST-ONLY MOCK wing-root leading-edge reference.",
        "source_ids": [MOCK_SOURCE],
        "photograph_ids": [MOCK_PHOTO]
    });
    value["provenance_sources"] = json!([{
        "id": MOCK_SOURCE,
        "kind": "measurement_session",
        "title": "TEST-ONLY MOCK survey session",
        "url": "https://example.invalid/ra1-mock-survey",
        "sha256": null,
        "notes": "Generated in this test, not an actual measurement."
    }]);
    value["photographs"] = json!([{
        "id": MOCK_PHOTO,
        "path": "synthetic/ra1-mock-nonexistent-photo.jpg",
        "url": null,
        "sha256": null,
        "description": "TEST-ONLY MOCK photo reference; no photograph exists."
    }]);
    value["comparison_baseline"] = json!({
        "source_ids": [],
        "wing_quarter_chord_offset_m": null,
        "horizontal_tail": {
            "span_m": null,
            "root_chord_m": null,
            "tip_chord_m": null,
            "area_weighted_quarter_chord_aft_root_le_m": null,
            "tip_le_offset_aft_root_le_m": null
        },
        "vertical_tail": {
            "height_m": null,
            "root_chord_m": null,
            "tip_chord_m": null,
            "area_weighted_quarter_chord_aft_root_le_m": null,
            "tip_le_offset_aft_root_le_m": null
        }
    });
    value["acceptance_criteria"]["maximum_station_asymmetry_m"] = json!(0.05);
    let observations = &mut value["raw_observations"];
    observations["horizontal_tail_root_le_aft_wing_le_m"]["left"] = survey_series(1.0);
    observations["horizontal_tail_root_le_aft_wing_le_m"]["right"] = survey_series(1.0);
    observations["vertical_tail_root_le_aft_wing_le_m"] = survey_series(1.1);
    observations["wing_quarter_chord_aft_wing_le_m"] = survey_series(0.2);
    observations["horizontal_tail_planform"]["span_m"] = survey_series(0.6);
    observations["horizontal_tail_planform"]["root_chord_m"] = survey_series(0.2);
    observations["horizontal_tail_planform"]["tip_chord_m"] = survey_series(0.15);
    observations["horizontal_tail_planform"]["tip_le_offset_aft_root_le_m"] = survey_series(0.0);
    observations["vertical_tail_planform"]["height_m"] = survey_series(0.3);
    observations["vertical_tail_planform"]["root_chord_m"] = survey_series(0.2);
    observations["vertical_tail_planform"]["tip_chord_m"] = survey_series(0.1);
    observations["vertical_tail_planform"]["tip_le_offset_aft_root_le_m"] = survey_series(0.0);
    value
}

fn mass_series(value: f64) -> Value {
    json!({
        "readings": [value, value, value],
        "instrument_resolution": 0.001,
        "stated_uncertainty": 0.002,
        "datum_or_method_definition": "TEST-ONLY MOCK FRD series; not a real measurement.",
        "notes": "Synthetic RA1-A test value.",
        "source_ids": [MOCK_SOURCE],
        "photograph_ids": [MOCK_PHOTO]
    })
}

fn mass_value() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_mass_properties_v0.json"
    ))
    .unwrap();
    value["campaign"] = json!({
        "id": "ra1-mock-mass",
        "classification": "physical_reference_measurement",
        "identity": {
            "manufacturer": MOCK_MANUFACTURER,
            "family": MOCK_FAMILY,
            "variant": MOCK_VARIANT,
            "airframe_id": MOCK_AIRFRAME
        },
        "measurement_date": "2030-01-02",
        "linked_geometry_campaign_id": MOCK_SURVEY_ID,
        "operational_configuration": {
            "id": MOCK_CONFIGURATION,
            "battery_configuration_id": null,
            "propulsion_configuration_description": "TEST-ONLY MOCK unpowered glider",
            "landing_gear_configuration": "TEST-ONLY MOCK no gear",
            "installed_equipment_notes": "TEST-ONLY MOCK configuration"
        },
        "notes": "TEST-ONLY MOCK; no physical mass campaign exists."
    });
    value["provenance_sources"] = json!([{
        "id": MOCK_SOURCE,
        "kind": "measurement_session",
        "title": "TEST-ONLY MOCK mass session",
        "url": "https://example.invalid/ra1-mock-mass",
        "sha256": null,
        "notes": "Generated in this test, not an actual measurement."
    }]);
    value["photographs"] = json!([{
        "id": MOCK_PHOTO,
        "path": "synthetic/ra1-mock-nonexistent-photo.jpg",
        "url": null,
        "sha256": null,
        "description": "TEST-ONLY MOCK photo reference; no photograph exists."
    }]);
    value["published_weight_range_comparison"] = json!({
        "minimum_kg": 2.0,
        "maximum_kg": 3.0,
        "source_ids": [MOCK_SOURCE],
        "authority": "comparison_only_never_operational_mass"
    });
    value["coordinate_frame"]["axes_parallel_to_frd"] = json!(true);
    value["coordinate_frame"]["origin_definition"] = json!("TEST-ONLY MOCK FRD origin");
    value["coordinate_frame"]["lateral_datum_definition"] = json!("TEST-ONLY MOCK lateral datum");
    value["coordinate_frame"]["vertical_datum_definition"] = json!("TEST-ONLY MOCK vertical datum");
    value["coordinate_frame"]["wing_root_le_center_plane_datum_established"] = json!(true);
    value["coordinate_frame"]["lateral_datum_established"] = json!(true);
    value["coordinate_frame"]["vertical_datum_established"] = json!(true);
    value["coordinate_frame"]["source_ids"] = json!([MOCK_SOURCE]);
    value["coordinate_frame"]["photograph_ids"] = json!([MOCK_PHOTO]);
    value["raw_observations"]["direct_total_mass_kg"] = mass_series(2.5);
    value["raw_observations"]["direct_cg_position_frd_m"] = json!({
        "x": mass_series(0.12),
        "y": mass_series(0.0),
        "z": mass_series(0.0)
    });
    value["raw_observations"]["direct_inertia_about_operational_cg_frd_kg_m2"] = json!({
        "method_class": "evidenced_cad_mass_model",
        "method_definition": "TEST-ONLY MOCK positive-definite inertia tensor.",
        "matrix_entries": {
            "ixx": mass_series(0.12),
            "ixy": mass_series(0.0),
            "ixz": mass_series(0.0),
            "iyx": mass_series(0.0),
            "iyy": mass_series(0.15),
            "iyz": mass_series(0.0),
            "izx": mass_series(0.0),
            "izy": mass_series(0.0),
            "izz": mass_series(0.20)
        },
        "source_ids": [MOCK_SOURCE],
        "photograph_ids": [MOCK_PHOTO],
        "notes": "Synthetic analytic tensor, never a real-aircraft observation."
    });
    value
}

fn aero_value() -> Value {
    let mut value: Value = serde_json::from_str(include_str!(
        "../../../docs/reference_aircraft/data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json"
    ))
    .unwrap();
    value["campaign"] = json!({
        "id": "ra1-mock-aero",
        "classification": "physical_reference_measurement",
        "manufacturer": MOCK_MANUFACTURER,
        "family": MOCK_FAMILY,
        "variant": MOCK_VARIANT,
        "notes": "TEST-ONLY MOCK; no real airfoil or polar evidence."
    });
    value["airfoil_identity"] = json!({
        "name": "RA1 synthetic five-point section",
        "source_ids": ["ra1-mock-airfoil-source"],
        "notes": "TEST-ONLY MOCK airfoil; not a published physical profile."
    });
    value["coordinates"] = json!({
        "source_id": "ra1-mock-airfoil-source",
        "coordinate_format": "selig",
        "normalization": "unit_chord_source_as_published",
        "ordering": "upper_trailing_edge_to_leading_edge_to_lower_trailing_edge",
        "leading_edge_representation": "single_point",
        "trailing_edge_representation": "open",
        "transformation_provenance": "Generated solely in RA1-A test code.",
        "points_x_over_c_y_over_c": [
            [1.0, 0.1], [0.5, 0.2], [0.0, 0.0], [0.5, -0.3], [1.0, -0.1]
        ],
        "notes": "Synthetic test-only coordinates."
    });
    value["provenance_sources"] = json!([
        {
            "id": "ra1-mock-airfoil-source",
            "kind": "airfoil_database",
            "title": "TEST-ONLY MOCK coordinate source",
            "publisher": "RA1 test suite",
            "url": "https://example.invalid/ra1-mock-airfoil",
            "retrieval_date": "2030-01-02",
            "sha256": "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "notes": "No such source exists."
        },
        {
            "id": "ra1-mock-solver-source",
            "kind": "solver_tool",
            "title": "TEST-ONLY MOCK solver",
            "publisher": "RA1 test suite",
            "url": "https://example.invalid/ra1-mock-solver",
            "retrieval_date": "2030-01-02",
            "sha256": null,
            "notes": "Synthetic coefficients; not physical evidence."
        }
    ]);
    value["operating_envelope"] = json!({
        "rationale": "TEST-ONLY MOCK Reynolds/Mach coverage requirement.",
        "source_ids": ["ra1-mock-solver-source"],
        "required_points": [{"reynolds": 100_000.0, "mach": 0.0}]
    });
    value["polar_datasets"] = json!([{
        "id": "ra1-mock-polar",
        "evidence_class": "generated_solver",
        "flow_conditions": {
            "reynolds": 100_000.0,
            "mach": 0.0,
            "density_kg_m3": null,
            "dynamic_viscosity_pa_s": null,
            "kinematic_viscosity_m2_s": null
        },
        "transition": {
            "assumptions": "TEST-ONLY MOCK free transition.",
            "ncrit": 7.0,
            "forced_transition_upper_x_over_c": null,
            "forced_transition_lower_x_over_c": null
        },
        "method": {
            "id": "ra1-mock-method",
            "solver_or_tool": "TEST-ONLY MOCK solver",
            "exact_version": "test-only-0",
            "command_or_config": "mock --never-runtime",
            "convergence_status": "converged"
        },
        "source_ids": ["ra1-mock-solver-source"],
        "samples": [
            {"alpha_rad": -1.0, "cl": -1.0, "cd": 0.1, "cm": 0.0},
            {"alpha_rad": 1.0, "cl": 1.0, "cd": 0.1, "cm": 0.0}
        ],
        "notes": "TEST-ONLY MOCK coefficients, not measurements."
    }]);
    value
}

struct MockReadyGlider {
    model_value: Value,
    model: AircraftModel,
    survey: PhysicalSurvey,
    mass: MassPropertiesCampaign,
    aero: AerodynamicEvidence,
}

impl MockReadyGlider {
    fn load() -> Self {
        let model_value = model_value();
        let model = load_value(&model_value).expect("mock model loads via production loader");
        let survey = PhysicalSurveyLoader::from_json_str(&survey_value().to_string())
            .expect("mock survey loads via production loader");
        let mass = MassPropertiesLoader::from_json_str(&mass_value().to_string())
            .expect("mock mass campaign loads via production loader");
        let aero = AerodynamicEvidenceLoader::from_json_str(&aero_value().to_string())
            .expect("mock aerodynamic campaign loads via production loader");
        assert!(survey.evaluation().geometry_ready());
        assert!(mass.evaluation().mass_properties_ready());
        assert!(aero.evaluation().aerodynamic_evidence_ready());
        Self {
            model_value,
            model,
            survey,
            mass,
            aero,
        }
    }

    fn request<'a>(&'a self, model: &'a AircraftModel) -> ReferenceReadinessInput<'a> {
        ReferenceReadinessInput {
            model,
            physical_configuration: PhysicalConfigurationIdentity {
                airframe_id: MOCK_AIRFRAME,
                operational_configuration_id: MOCK_CONFIGURATION,
                propulsion_configuration_id: None,
            },
            survey: Some(&self.survey),
            mass_campaign: Some(&self.mass),
            aerodynamic_evidence: Some(&self.aero),
            propulsion_evidence: None,
            required_alpha_rad: Some((-0.2, 0.2)),
        }
    }
}

#[test]
fn test_only_mock_reaches_ready_through_public_evaluator() {
    let fixture = MockReadyGlider::load();
    let before = fixture.model.physics_fingerprint();
    let request = fixture.request(&fixture.model);
    let first = evaluate_reference_aircraft_readiness(request);
    let second = evaluate_reference_aircraft_readiness(request);

    assert_eq!(first.overall_status, ReadinessStatus::Ready, "{first:#?}");
    for domain in [
        ReadinessDomain::AircraftIdentity,
        ReadinessDomain::Geometry,
        ReadinessDomain::MassProperties,
        ReadinessDomain::Aerodynamics,
        ReadinessDomain::Controls,
        ReadinessDomain::ModelIntegrity,
    ] {
        assert_eq!(
            first.domain(domain).status,
            ReadinessStatus::Ready,
            "{domain:?}"
        );
    }
    assert_eq!(
        first.domain(ReadinessDomain::Propulsion).status,
        ReadinessStatus::NotApplicable
    );
    assert_eq!(
        first.domain(ReadinessDomain::FlightTestData).status,
        ReadinessStatus::Incomplete
    );
    assert!(
        first
            .domain(ReadinessDomain::FlightTestData)
            .findings
            .iter()
            .any(|finding| finding.reason == ReadinessReason::FlightTestEvidenceNotEvaluated)
    );
    assert_eq!(fixture.model.physics_fingerprint(), before);
    assert_eq!(first.physics_fingerprint, before);
    assert_eq!(first, second);
}

#[test]
fn test_only_mock_wrong_physical_identity_blocks() {
    let fixture = MockReadyGlider::load();
    let mut request = fixture.request(&fixture.model);
    request.physical_configuration.airframe_id = "different-ra1-mock-airframe";
    let report = evaluate_reference_aircraft_readiness(request);
    assert_eq!(
        report.domain(ReadinessDomain::AircraftIdentity).status,
        ReadinessStatus::Blocked
    );
    assert!(
        report
            .domain(ReadinessDomain::AircraftIdentity)
            .findings
            .iter()
            .any(|finding| finding.reason == ReadinessReason::PhysicalAirframeMismatch)
    );
    assert_eq!(report.overall_status, ReadinessStatus::Blocked);
}

#[test]
fn test_only_mock_missing_surface_travel_is_incomplete() {
    let fixture = MockReadyGlider::load();
    let mut model_value = fixture.model_value.clone();
    model_value["reference_aircraft"]["physical_specification"]["control_surface_travel_limits"]
        .as_array_mut()
        .unwrap()
        .pop();
    let model = load_value(&model_value).expect("missing documentary travel remains loadable");
    let report = evaluate_reference_aircraft_readiness(fixture.request(&model));
    assert_eq!(
        report.domain(ReadinessDomain::Controls).status,
        ReadinessStatus::Incomplete
    );
    assert!(
        report
            .domain(ReadinessDomain::Controls)
            .findings
            .iter()
            .any(|finding| finding.reason == ReadinessReason::SurfaceTravelEvidenceMissing)
    );
    assert_eq!(report.overall_status, ReadinessStatus::Incomplete);
}
