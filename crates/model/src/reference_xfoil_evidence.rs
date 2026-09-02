//! Deterministic bridge from parsed XFOIL polar imports to the aerodynamic
//! evidence schema.
//!
//! M2.9B converts an [`XfoilPolarImport`] (M2.9A) into a JSON dataset
//! compatible with one element of `polar_datasets` inside the existing
//! `reference_aircraft_aerodynamic_evidence_v0` schema.
//!
//! This module does NOT infer convergence, does NOT generate coefficients,
//! does NOT modify runtime physics, and does NOT make any aircraft
//! runtime-ready.

use std::collections::HashSet;

use serde_json::{Value, json};

use crate::reference_aerodynamics::validate_stable_id;
use crate::{ConvergenceStatus, XfoilPolarImport};

/// A deterministic off-runtime evidence dataset bridging an XFOIL polar
/// import to the `reference_aircraft_aerodynamic_evidence_v0` schema.
///
/// The produced dataset serializes as exactly one element of the
/// `polar_datasets` array.
#[derive(Debug, Clone)]
pub struct XfoilEvidenceDataset {
    dataset_id: String,
    method_id: String,
    convergence_status: ConvergenceStatus,
    source_ids: Vec<String>,
    notes: Option<String>,
    import: XfoilPolarImport,
}

impl XfoilEvidenceDataset {
    /// Dataset stable ID.
    pub fn dataset_id(&self) -> &str {
        &self.dataset_id
    }

    /// Method stable ID.
    pub fn method_id(&self) -> &str {
        &self.method_id
    }

    /// Evidence class — always `generated_solver`.
    pub const fn evidence_class(&self) -> ConvergenceStatus {
        self.convergence_status
    }

    /// Explicit convergence status supplied by the caller.
    pub const fn convergence_status(&self) -> ConvergenceStatus {
        self.convergence_status
    }

    /// Reynolds number from the solver metadata.
    pub fn reynolds(&self) -> f64 {
        self.import.metadata().reynolds()
    }

    /// Mach number from the solver metadata.
    pub fn mach(&self) -> f64 {
        self.import.metadata().mach()
    }

    /// Number of polar samples.
    pub fn sample_count(&self) -> usize {
        self.import.sample_count()
    }

    /// Source IDs in caller-supplied order.
    pub fn source_ids(&self) -> &[String] {
        &self.source_ids
    }

    /// Optional dataset notes.
    pub fn notes(&self) -> Option<&str> {
        self.notes.as_deref()
    }

    /// The underlying XFOIL import.
    pub fn import(&self) -> &XfoilPolarImport {
        &self.import
    }

    /// Serialize this dataset as a pretty-printed JSON string matching the
    /// `polar_datasets[]` element shape of the
    /// `reference_aircraft_aerodynamic_evidence_v0` schema.
    pub fn to_json_pretty(&self) -> String {
        serde_json::to_string_pretty(&self.to_json_value()).expect("dataset serializes")
    }

    /// Serialize this dataset as a [`serde_json::Value`] matching the
    /// `polar_datasets[]` element shape.
    pub fn to_json_value(&self) -> Value {
        let metadata = self.import.metadata();

        let samples: Vec<Value> = self
            .import
            .samples()
            .iter()
            .map(|s| {
                json!({
                    "alpha_rad": s.alpha_rad(),
                    "cl": s.cl(),
                    "cd": s.cd(),
                    "cm": s.cm()
                })
            })
            .collect();

        let mut transition = json!({});
        if let Some(assumptions) = metadata.transition_assumptions() {
            transition["assumptions"] = json!(assumptions);
        } else {
            transition["assumptions"] = Value::Null;
        }
        if let Some(ncrit) = metadata.ncrit() {
            transition["ncrit"] = json!(ncrit);
        } else {
            transition["ncrit"] = Value::Null;
        }
        if let Some(v) = metadata.forced_transition_upper_x_over_c() {
            transition["forced_transition_upper_x_over_c"] = json!(v);
        } else {
            transition["forced_transition_upper_x_over_c"] = Value::Null;
        }
        if let Some(v) = metadata.forced_transition_lower_x_over_c() {
            transition["forced_transition_lower_x_over_c"] = json!(v);
        } else {
            transition["forced_transition_lower_x_over_c"] = Value::Null;
        }

        let mut method = json!({
            "id": self.method_id,
            "convergence_status": serde_json::to_string(&self.convergence_status)
                .expect("convergence status serializes")
                .trim_matches('"')
        });
        if let Some(name) = metadata.solver_name() {
            method["solver_or_tool"] = json!(name);
        } else {
            method["solver_or_tool"] = Value::Null;
        }
        if let Some(version) = metadata.solver_version() {
            method["exact_version"] = json!(version);
        } else {
            method["exact_version"] = Value::Null;
        }
        if let Some(config) = metadata.command_or_config() {
            method["command_or_config"] = json!(config);
        } else {
            method["command_or_config"] = Value::Null;
        }

        json!({
            "id": self.dataset_id,
            "evidence_class": "generated_solver",
            "flow_conditions": {
                "reynolds": metadata.reynolds(),
                "mach": metadata.mach(),
                "density_kg_m3": null,
                "dynamic_viscosity_pa_s": null,
                "kinematic_viscosity_m2_s": null
            },
            "transition": transition,
            "method": method,
            "source_ids": self.source_ids,
            "samples": samples,
            "notes": self.notes
        })
    }
}

/// Builder for [`XfoilEvidenceDataset`].
///
/// All required fields must be supplied explicitly. The builder validates
/// IDs, source references, and produces a deterministic dataset.
#[derive(Debug, Clone)]
pub struct XfoilEvidenceDatasetBuilder {
    dataset_id: String,
    method_id: String,
    convergence_status: ConvergenceStatus,
    source_ids: Vec<String>,
    notes: Option<String>,
    import: XfoilPolarImport,
}

impl XfoilEvidenceDatasetBuilder {
    /// Create a new builder from a parsed XFOIL import and required fields.
    pub fn new(
        import: XfoilPolarImport,
        dataset_id: impl Into<String>,
        method_id: impl Into<String>,
        convergence_status: ConvergenceStatus,
        source_ids: Vec<String>,
    ) -> Self {
        Self {
            dataset_id: dataset_id.into(),
            method_id: method_id.into(),
            convergence_status,
            source_ids,
            notes: None,
            import,
        }
    }

    /// Set optional dataset notes.
    pub fn notes(mut self, notes: impl Into<String>) -> Self {
        self.notes = Some(notes.into());
        self
    }

    /// Validate all fields and build the evidence dataset.
    pub fn build(self) -> Result<XfoilEvidenceDataset, XfoilEvidenceBridgeError> {
        validate_stable_id("dataset", &self.dataset_id)
            .map_err(|_| XfoilEvidenceBridgeError::InvalidDatasetId(self.dataset_id.clone()))?;
        validate_stable_id("method", &self.method_id)
            .map_err(|_| XfoilEvidenceBridgeError::InvalidMethodId(self.method_id.clone()))?;

        if self.source_ids.is_empty() {
            return Err(XfoilEvidenceBridgeError::EmptySourceIds);
        }

        let mut seen = HashSet::new();
        for source_id in &self.source_ids {
            validate_stable_id("source", source_id)
                .map_err(|_| XfoilEvidenceBridgeError::InvalidSourceId(source_id.clone()))?;
            if !seen.insert(source_id.as_str()) {
                return Err(XfoilEvidenceBridgeError::DuplicateSourceId(
                    source_id.clone(),
                ));
            }
        }

        Ok(XfoilEvidenceDataset {
            dataset_id: self.dataset_id,
            method_id: self.method_id,
            convergence_status: self.convergence_status,
            source_ids: self.source_ids,
            notes: self.notes,
            import: self.import,
        })
    }
}

/// Errors from the XFOIL-to-evidence bridge.
#[derive(Debug, thiserror::Error)]
pub enum XfoilEvidenceBridgeError {
    #[error("invalid dataset ID {0:?}")]
    InvalidDatasetId(String),

    #[error("invalid method ID {0:?}")]
    InvalidMethodId(String),

    #[error("invalid source ID {0:?}")]
    InvalidSourceId(String),

    #[error("duplicate source ID {0:?}")]
    DuplicateSourceId(String),

    #[error("source ID list must not be empty")]
    EmptySourceIds,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MetadataBuilder, parse_xfoil_polar};

    const STANDARD_7COL: &str = "\
 alpha    CL         CD         CDp        CM         Top_Xtr  Bot_Xtr
 ------   ---------  ---------  ---------  ---------  -------  -------
  -2.000  -0.0414    0.01134    0.00442   -0.0120     0.5412   0.6178
   0.000   0.1593    0.00700    0.00156   -0.0549     0.5812   0.5612
   2.000   0.3593    0.00720    0.00180   -0.0570     0.6200   0.5200
";

    fn import_with_metadata() -> XfoilPolarImport {
        let metadata = MetadataBuilder::new(300_000.0, 0.0)
            .solver_name("XFOIL")
            .solver_version("6.99")
            .command_or_config("OPER RE 300000 VISC")
            .transition_assumptions("Free transition e^N, Ncrit=9")
            .ncrit(9.0)
            .build()
            .unwrap();
        parse_xfoil_polar(STANDARD_7COL, metadata).unwrap()
    }

    fn valid_builder() -> XfoilEvidenceDatasetBuilder {
        XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "test-dataset_01",
            "xfoil-method_01",
            ConvergenceStatus::Converged,
            vec!["synthetic-src".to_owned()],
        )
    }

    #[test]
    fn valid_bridge_creation() {
        let dataset = valid_builder().build().unwrap();
        assert_eq!(dataset.dataset_id(), "test-dataset_01");
        assert_eq!(dataset.method_id(), "xfoil-method_01");
        assert_eq!(dataset.convergence_status(), ConvergenceStatus::Converged);
        assert_eq!(dataset.reynolds(), 300_000.0);
        assert_eq!(dataset.mach(), 0.0);
        assert_eq!(dataset.sample_count(), 3);
    }

    #[test]
    fn generated_solver_class_fixed() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        assert_eq!(json["evidence_class"], "generated_solver");
    }

    #[test]
    fn sample_mapping_exact() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        let samples = json["samples"].as_array().unwrap();
        assert_eq!(samples.len(), 3);

        let s0 = &samples[0];
        let expected_alpha = -2.0_f64.to_radians();
        assert!((s0["alpha_rad"].as_f64().unwrap() - expected_alpha).abs() < 1e-15);
        assert_eq!(s0["cl"].as_f64().unwrap(), -0.0414);
        assert_eq!(s0["cd"].as_f64().unwrap(), 0.01134);
        assert_eq!(s0["cm"].as_f64().unwrap(), -0.0120);
    }

    #[test]
    fn sample_ordering_preserved() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        let samples = json["samples"].as_array().unwrap();
        for i in 1..samples.len() {
            let prev = samples[i - 1]["alpha_rad"].as_f64().unwrap();
            let curr = samples[i]["alpha_rad"].as_f64().unwrap();
            assert!(curr > prev);
        }
    }

    #[test]
    fn convergence_status_preserved_converged() {
        let dataset = valid_builder().build().unwrap();
        assert_eq!(dataset.convergence_status(), ConvergenceStatus::Converged);
        let json = dataset.to_json_value();
        assert_eq!(json["method"]["convergence_status"], "converged");
    }

    #[test]
    fn convergence_status_preserved_unresolved() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-unresolved",
            "m-01",
            ConvergenceStatus::Unresolved,
            vec!["src".to_owned()],
        );
        let dataset = builder.build().unwrap();
        assert_eq!(dataset.convergence_status(), ConvergenceStatus::Unresolved);
        let json = dataset.to_json_value();
        assert_eq!(json["method"]["convergence_status"], "unresolved");
    }

    #[test]
    fn convergence_status_preserved_failed() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-failed",
            "m-01",
            ConvergenceStatus::Failed,
            vec!["src".to_owned()],
        );
        let dataset = builder.build().unwrap();
        assert_eq!(dataset.convergence_status(), ConvergenceStatus::Failed);
        let json = dataset.to_json_value();
        assert_eq!(json["method"]["convergence_status"], "failed");
    }

    #[test]
    fn source_ordering_preserved() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-src",
            "m-01",
            ConvergenceStatus::Converged,
            vec![
                "source-b".to_owned(),
                "source-a".to_owned(),
                "source-c".to_owned(),
            ],
        );
        let dataset = builder.build().unwrap();
        assert_eq!(dataset.source_ids(), &["source-b", "source-a", "source-c"]);
        let json = dataset.to_json_value();
        let ids: Vec<&str> = json["source_ids"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect();
        assert_eq!(ids, vec!["source-b", "source-a", "source-c"]);
    }

    #[test]
    fn duplicate_source_rejected() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
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
    fn empty_source_ids_rejected() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-empty",
            "m-01",
            ConvergenceStatus::Converged,
            vec![],
        );
        let err = builder.build().unwrap_err();
        assert!(matches!(err, XfoilEvidenceBridgeError::EmptySourceIds));
    }

    #[test]
    fn invalid_dataset_id_rejected() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "INVALID ID",
            "m-01",
            ConvergenceStatus::Converged,
            vec!["src".to_owned()],
        );
        let err = builder.build().unwrap_err();
        assert!(matches!(err, XfoilEvidenceBridgeError::InvalidDatasetId(_)));
    }

    #[test]
    fn invalid_method_id_rejected() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-01",
            "INVALID!",
            ConvergenceStatus::Converged,
            vec!["src".to_owned()],
        );
        let err = builder.build().unwrap_err();
        assert!(matches!(err, XfoilEvidenceBridgeError::InvalidMethodId(_)));
    }

    #[test]
    fn invalid_source_id_rejected() {
        let builder = XfoilEvidenceDatasetBuilder::new(
            import_with_metadata(),
            "ds-01",
            "m-01",
            ConvergenceStatus::Converged,
            vec!["BAD SOURCE".to_owned()],
        );
        let err = builder.build().unwrap_err();
        assert!(matches!(err, XfoilEvidenceBridgeError::InvalidSourceId(_)));
    }

    #[test]
    fn transition_assumptions_preserved() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        assert_eq!(
            json["transition"]["assumptions"],
            "Free transition e^N, Ncrit=9"
        );
    }

    #[test]
    fn ncrit_preserved() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        assert_eq!(json["transition"]["ncrit"], 9.0);
    }

    #[test]
    fn forced_transition_preserved_when_supplied() {
        let metadata = MetadataBuilder::new(300_000.0, 0.0)
            .solver_name("XFOIL")
            .solver_version("6.99")
            .forced_transition_upper(0.1)
            .forced_transition_lower(0.9)
            .build()
            .unwrap();
        let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
        let builder = XfoilEvidenceDatasetBuilder::new(
            import,
            "ds-ft",
            "m-01",
            ConvergenceStatus::Converged,
            vec!["src".to_owned()],
        );
        let dataset = builder.build().unwrap();
        let json = dataset.to_json_value();
        assert_eq!(json["transition"]["forced_transition_upper_x_over_c"], 0.1);
        assert_eq!(json["transition"]["forced_transition_lower_x_over_c"], 0.9);
    }

    #[test]
    fn top_xtr_not_used_as_forced_transition() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
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
    fn cdp_does_not_overwrite_cd_or_cm() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        let s0 = &json["samples"][0];
        assert_eq!(s0["cd"].as_f64().unwrap(), 0.01134);
        assert_eq!(s0["cm"].as_f64().unwrap(), -0.0120);
        assert!(s0.get("cdp").is_none());
    }

    #[test]
    fn missing_optional_metadata_not_fabricated() {
        let metadata = MetadataBuilder::new(300_000.0, 0.0).build().unwrap();
        let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
        let builder = XfoilEvidenceDatasetBuilder::new(
            import,
            "ds-bare",
            "m-bare",
            ConvergenceStatus::Unresolved,
            vec!["src".to_owned()],
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
    fn deterministic_serialization() {
        let a = valid_builder().build().unwrap();
        let b = valid_builder().build().unwrap();
        assert_eq!(a.to_json_pretty(), b.to_json_pretty());
    }

    #[test]
    fn flow_conditions_exact() {
        let dataset = valid_builder().build().unwrap();
        let json = dataset.to_json_value();
        assert_eq!(json["flow_conditions"]["reynolds"], 300_000.0);
        assert_eq!(json["flow_conditions"]["mach"], 0.0);
        assert_eq!(json["flow_conditions"]["density_kg_m3"], Value::Null);
        assert_eq!(
            json["flow_conditions"]["dynamic_viscosity_pa_s"],
            Value::Null
        );
        assert_eq!(
            json["flow_conditions"]["kinematic_viscosity_m2_s"],
            Value::Null
        );
    }

    #[test]
    fn no_runtime_object_constructed() {
        let dataset = valid_builder().build().unwrap();
        let _json = dataset.to_json_value();
    }

    #[test]
    fn end_to_end_through_evidence_loader() {
        use crate::AerodynamicEvidenceLoader;

        let dataset = valid_builder().build().unwrap();
        let dataset_json = dataset.to_json_value();

        let artifact = json!({
            "schema": "reference_aircraft_aerodynamic_evidence_v0",
            "artifact_kind": "aerodynamic_evidence_not_runtime_configuration",
            "campaign": {
                "id": "synthetic-m2-9b-e2e",
                "classification": "synthetic_non_reference",
                "manufacturer": "Synthetic Bridge Test",
                "family": "Bridge Test Family",
                "variant": "e2e-test-only",
                "notes": null
            },
            "airfoil_identity": {
                "name": "Synthetic Test Airfoil",
                "source_ids": ["synthetic-src"],
                "notes": null
            },
            "coordinates": {
                "source_id": "synthetic-src",
                "coordinate_format": "selig",
                "normalization": "unit_chord_source_as_published",
                "ordering": "upper_trailing_edge_to_leading_edge_to_lower_trailing_edge",
                "leading_edge_representation": "single_point",
                "trailing_edge_representation": "open",
                "transformation_provenance": "Synthetic five-point fixture for bridge test.",
                "points_x_over_c_y_over_c": [
                    [1.0, 0.1], [0.5, 0.2], [0.0, 0.0], [0.5, -0.3], [1.0, -0.1]
                ],
                "notes": null
            },
            "provenance_sources": [
                {
                    "id": "synthetic-src",
                    "kind": "airfoil_database",
                    "title": "Synthetic source for bridge test",
                    "publisher": "Test suite",
                    "url": "https://example.invalid/bridge",
                    "retrieval_date": "2030-01-01",
                    "sha256": null,
                    "notes": null
                }
            ],
            "operating_envelope": null,
            "polar_datasets": [dataset_json]
        });

        let json_str = serde_json::to_string_pretty(&artifact).unwrap();
        let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

        let eval = evidence.evaluation();
        assert_eq!(eval.datasets().len(), 1);
        let ds = &eval.datasets()[0];
        assert_eq!(ds.id(), "test-dataset_01");
        assert_eq!(
            ds.evidence_class(),
            crate::AerodynamicEvidenceClass::GeneratedSolver
        );
        assert_eq!(ds.reynolds(), 300_000.0);
        assert_eq!(ds.mach(), 0.0);
        assert_eq!(ds.convergence_status(), ConvergenceStatus::Converged);
        assert!(ds.evidence_ready());
    }

    #[test]
    fn unresolved_does_not_become_evidence_ready_without_metadata() {
        use crate::AerodynamicEvidenceLoader;

        let metadata = MetadataBuilder::new(300_000.0, 0.0).build().unwrap();
        let import = parse_xfoil_polar(STANDARD_7COL, metadata).unwrap();
        let builder = XfoilEvidenceDatasetBuilder::new(
            import,
            "ds-unresolved",
            "m-01",
            ConvergenceStatus::Unresolved,
            vec!["synthetic-src".to_owned()],
        );
        let dataset = builder.build().unwrap();
        let dataset_json = dataset.to_json_value();

        let artifact = json!({
            "schema": "reference_aircraft_aerodynamic_evidence_v0",
            "artifact_kind": "aerodynamic_evidence_not_runtime_configuration",
            "campaign": {
                "id": "synthetic-m2-9b-unresolved",
                "classification": "synthetic_non_reference",
                "manufacturer": "Synthetic Bridge Test",
                "family": "Bridge Test Family",
                "variant": "unresolved-test",
                "notes": null
            },
            "airfoil_identity": {
                "name": "Synthetic Test Airfoil",
                "source_ids": ["synthetic-src"],
                "notes": null
            },
            "coordinates": {
                "source_id": "synthetic-src",
                "coordinate_format": "selig",
                "normalization": "unit_chord_source_as_published",
                "ordering": "upper_trailing_edge_to_leading_edge_to_lower_trailing_edge",
                "leading_edge_representation": "single_point",
                "trailing_edge_representation": "open",
                "transformation_provenance": "Synthetic five-point fixture.",
                "points_x_over_c_y_over_c": [
                    [1.0, 0.1], [0.5, 0.2], [0.0, 0.0], [0.5, -0.3], [1.0, -0.1]
                ],
                "notes": null
            },
            "provenance_sources": [
                {
                    "id": "synthetic-src",
                    "kind": "airfoil_database",
                    "title": "Synthetic source",
                    "publisher": "Test suite",
                    "url": "https://example.invalid/bridge",
                    "retrieval_date": "2030-01-01",
                    "sha256": null,
                    "notes": null
                }
            ],
            "operating_envelope": null,
            "polar_datasets": [dataset_json]
        });

        let json_str = serde_json::to_string_pretty(&artifact).unwrap();
        let evidence = AerodynamicEvidenceLoader::from_json_str(&json_str).unwrap();

        let eval = evidence.evaluation();
        let ds = &eval.datasets()[0];
        assert_eq!(ds.convergence_status(), ConvergenceStatus::Unresolved);
        assert!(!ds.evidence_ready());
        assert!(!eval.runtime_ready());
    }
}
