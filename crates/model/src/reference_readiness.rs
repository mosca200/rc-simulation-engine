//! Offline, deterministic RA1-A evidence gate. It never changes runtime model data.

use crate::{
    AIRCRAFT_MODEL_SCHEMA_VERSION_V9, AerodynamicEvidence, AircraftClassification, AircraftModel,
    AircraftModelFingerprint, CgReferenceKind, MassPropertiesCampaign, ParameterQuality,
    PhysicalSurvey, PropulsionConfigurationEvidenceClass, PropulsionEvidence,
    ReferenceParameterEvidence, ReferenceScalar, SurveyClassification,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadinessStatus {
    Ready,
    Incomplete,
    Blocked,
    NotApplicable,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadinessDomain {
    AircraftIdentity,
    Geometry,
    MassProperties,
    Aerodynamics,
    Propulsion,
    Controls,
    ModelIntegrity,
    FlightTestData,
}

/// Stable, typed decision reasons. `subject` identifies a dataset or binding, never drives policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadinessReason {
    SyntheticModel,
    ReferenceMetadataMissing,
    StableReferenceIdMissing,
    ModelIdentityIncomplete,
    AircraftFamilyMismatch,
    PhysicalAirframeIdMissing,
    OperationalConfigurationIdMissing,
    PropulsionConfigurationIdMissing,
    PhysicalSurveyMissing,
    SyntheticEvidence,
    PhysicalAirframeMismatch,
    SurveyMassCampaignMismatch,
    OperationalConfigurationMismatch,
    PhysicalInstallationMismatch,
    WingspanMissing,
    WingAreaMissing,
    ReferenceChordMissing,
    TailSurveyIncomplete,
    AeroElementsMissing,
    AeroSurfacesMissing,
    CgDatumMissing,
    CgDatumAmbiguous,
    MassEvidenceMissing,
    CgEvidenceMissing,
    MassCampaignMissing,
    MassMeasurementMissing,
    CgMeasurementMissing,
    InertiaEvidenceMissing,
    EvidenceUnresolved,
    AerodynamicEvidenceMissing,
    AirfoilProvenanceUnresolved,
    CoordinatesProvenanceUnresolved,
    PolarEvidenceUnresolved,
    ReynoldsCoverageInsufficient,
    AlphaEnvelopeMissing,
    InvalidAlphaEnvelope,
    AlphaCoverageInsufficient,
    PropulsionEvidenceMissing,
    MotorEvidenceMissing,
    EscEvidenceMissing,
    BatteryEvidenceMissing,
    PropellerEvidenceMissing,
    PropulsionDatasetMissing,
    PhysicalInstallationUnproven,
    ControlBindingMissing,
    SurfaceTravelEvidenceMissing,
    UnsupportedSchema,
    NonFiniteModelData,
    FlightTestEvidenceNotEvaluated,
}

impl ReadinessReason {
    const fn is_blocking(self) -> bool {
        matches!(
            self,
            Self::SyntheticModel
                | Self::SyntheticEvidence
                | Self::PhysicalAirframeMismatch
                | Self::AircraftFamilyMismatch
                | Self::SurveyMassCampaignMismatch
                | Self::OperationalConfigurationMismatch
                | Self::PhysicalInstallationMismatch
                | Self::CgDatumAmbiguous
                | Self::EvidenceUnresolved
                | Self::AirfoilProvenanceUnresolved
                | Self::CoordinatesProvenanceUnresolved
                | Self::PolarEvidenceUnresolved
                | Self::UnsupportedSchema
                | Self::NonFiniteModelData
                | Self::InvalidAlphaEnvelope
        )
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReadinessFinding {
    pub reason: ReadinessReason,
    pub subject: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadinessEvidence {
    ValidatedModel,
    PhysicsFingerprint,
    ReferenceMetadata,
    StableReferenceId,
    PhysicalAirframeId,
    OperationalConfigurationId,
    PhysicalSurvey,
    Wingspan,
    WingArea,
    ReferenceChord,
    TailGeometry,
    AeroElements,
    AeroSurfaces,
    CgDatum,
    MassProvenance,
    CgProvenance,
    MassCampaign,
    TotalMass,
    Cg,
    Inertia,
    AerodynamicCampaign,
    AirfoilIdentity,
    AirfoilCoordinates,
    QualifiedPolars,
    ReynoldsEnvelope,
    AlphaEnvelope,
    PropulsionCampaign,
    Motor,
    Esc,
    Battery,
    Propeller,
    PropulsionDataset,
    PhysicalInstallation,
    ControlBindings,
    SurfaceTravel,
    ServoConfiguration,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DomainReadiness {
    pub domain: ReadinessDomain,
    pub status: ReadinessStatus,
    pub present: Vec<ReadinessEvidence>,
    pub findings: Vec<ReadinessFinding>,
}

impl DomainReadiness {
    fn new(domain: ReadinessDomain) -> Self {
        Self {
            domain,
            status: ReadinessStatus::Ready,
            present: Vec::new(),
            findings: Vec::new(),
        }
    }

    fn present(&mut self, evidence: ReadinessEvidence) {
        self.present.push(evidence);
    }

    fn finding(&mut self, reason: ReadinessReason) {
        self.findings.push(ReadinessFinding {
            reason,
            subject: None,
        });
    }

    fn subject(&mut self, reason: ReadinessReason, subject: &str) {
        self.findings.push(ReadinessFinding {
            reason,
            subject: Some(subject.to_owned()),
        });
    }

    fn finish(&mut self) {
        self.status = if self.findings.iter().any(|f| f.reason.is_blocking()) {
            ReadinessStatus::Blocked
        } else if self.findings.is_empty() {
            ReadinessStatus::Ready
        } else {
            ReadinessStatus::Incomplete
        };
    }
}

/// Explicit physical configuration being assessed. IDs are corroborated against loaded artifacts.
#[derive(Debug, Clone, Copy)]
pub struct PhysicalConfigurationIdentity<'a> {
    pub airframe_id: &'a str,
    pub operational_configuration_id: &'a str,
    pub propulsion_configuration_id: Option<&'a str>,
}

/// All inputs are already loaded and validated by their existing strict loaders.
#[derive(Debug, Clone, Copy)]
pub struct ReferenceReadinessInput<'a> {
    pub model: &'a AircraftModel,
    pub physical_configuration: PhysicalConfigurationIdentity<'a>,
    pub survey: Option<&'a PhysicalSurvey>,
    pub mass_campaign: Option<&'a MassPropertiesCampaign>,
    pub aerodynamic_evidence: Option<&'a AerodynamicEvidence>,
    pub propulsion_evidence: Option<&'a PropulsionEvidence>,
    /// Explicit required alpha interval; no physical default is assumed.
    pub required_alpha_rad: Option<(f64, f64)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceAircraftReadiness {
    pub model_id: String,
    pub physical_airframe_id: String,
    pub operational_configuration_id: String,
    pub propulsion_configuration_id: Option<String>,
    pub physics_fingerprint: AircraftModelFingerprint,
    /// Physical-model readiness only; flight-test readiness is independent.
    pub overall_status: ReadinessStatus,
    /// Fixed order: identity, geometry, mass, aerodynamics, propulsion, controls, integrity, flight data.
    pub domains: [DomainReadiness; 8],
    /// Flattened domain findings in the same fixed order.
    pub findings: Vec<(ReadinessDomain, ReadinessFinding)>,
}

impl ReferenceAircraftReadiness {
    pub fn domain(&self, domain: ReadinessDomain) -> &DomainReadiness {
        self.domains
            .iter()
            .find(|item| item.domain == domain)
            .expect("fixed domain exists")
    }
}

fn sourced(evidence: &ReferenceParameterEvidence) -> bool {
    !matches!(
        evidence.quality(),
        ParameterQuality::Unknown | ParameterQuality::Estimated
    ) && !evidence.source_indices().is_empty()
}

fn family_matches(model: &AircraftModel, manufacturer: &str, family: &str, variant: &str) -> bool {
    model.reference_aircraft().is_some_and(|reference| {
        let identity = reference.identity();
        identity.manufacturer() == Some(manufacturer)
            && identity.aircraft_name() == Some(family)
            && identity.variant() == Some(variant)
    })
}

fn scalar(
    domain: &mut DomainReadiness,
    value: Option<&ReferenceScalar>,
    marker: ReadinessEvidence,
    missing: ReadinessReason,
) {
    match value {
        None => domain.finding(missing),
        Some(value) if !value.value().is_finite() => {
            domain.finding(ReadinessReason::NonFiniteModelData)
        }
        Some(value) if !sourced(value.evidence()) => {
            domain.finding(ReadinessReason::EvidenceUnresolved)
        }
        Some(_) => domain.present(marker),
    }
}

/// Pure, fail-closed evaluation. Does not promote evidence into runtime configuration.
pub fn evaluate_reference_aircraft_readiness(
    input: ReferenceReadinessInput<'_>,
) -> ReferenceAircraftReadiness {
    use ReadinessDomain as D;
    use ReadinessEvidence as E;
    use ReadinessReason as R;

    let model = input.model;
    let reference = model.reference_aircraft();
    let specification = reference.map(|r| r.physical_specification());
    let physical = input.physical_configuration;

    let mut identity = DomainReadiness::new(D::AircraftIdentity);
    if model.classification() == AircraftClassification::SyntheticTest {
        identity.finding(R::SyntheticModel);
    }
    if let Some(reference) = reference {
        identity.present(E::ReferenceMetadata);
        if reference.identity().manufacturer().is_none()
            || reference.identity().aircraft_name().is_none()
            || reference.identity().variant().is_none()
        {
            identity.finding(R::ModelIdentityIncomplete);
        }
        if reference
            .identity()
            .stable_reference_id()
            .is_some_and(|s| !s.trim().is_empty())
        {
            identity.present(E::StableReferenceId);
        } else {
            identity.finding(R::StableReferenceIdMissing);
        }
    } else {
        identity.finding(R::ReferenceMetadataMissing);
    }
    if physical.airframe_id.trim().is_empty() {
        identity.finding(R::PhysicalAirframeIdMissing);
    } else {
        identity.present(E::PhysicalAirframeId);
    }
    if physical.operational_configuration_id.trim().is_empty() {
        identity.finding(R::OperationalConfigurationIdMissing);
    } else {
        identity.present(E::OperationalConfigurationId);
    }
    match input.survey {
        None => identity.finding(R::PhysicalSurveyMissing),
        Some(survey) => {
            if survey.classification() != SurveyClassification::PhysicalReferenceMeasurement {
                identity.finding(R::SyntheticEvidence);
            } else {
                identity.present(E::PhysicalSurvey);
            }
            if survey.airframe_id() != Some(physical.airframe_id) {
                identity.finding(R::PhysicalAirframeMismatch);
            }
            if reference.is_some()
                && !family_matches(
                    model,
                    survey.manufacturer(),
                    survey.family(),
                    survey.variant(),
                )
            {
                identity.finding(R::AircraftFamilyMismatch);
            }
        }
    }
    if let Some(campaign) = input.mass_campaign {
        if campaign.airframe_id() != Some(physical.airframe_id) {
            identity.finding(R::PhysicalAirframeMismatch);
        }
        if input
            .survey
            .is_some_and(|survey| campaign.linked_geometry_campaign_id() != survey.campaign_id())
        {
            identity.finding(R::SurveyMassCampaignMismatch);
        }
        if reference.is_some()
            && !family_matches(
                model,
                campaign.manufacturer(),
                campaign.family(),
                campaign.variant(),
            )
        {
            identity.finding(R::AircraftFamilyMismatch);
        }
    }
    if let Some(campaign) = input.aerodynamic_evidence
        && reference.is_some()
        && !family_matches(
            model,
            campaign.manufacturer(),
            campaign.family(),
            campaign.variant(),
        )
    {
        identity.finding(R::AircraftFamilyMismatch);
    }
    if let Some(campaign) = input.propulsion_evidence
        && model.propulsion().is_some()
        && reference.is_some()
        && !family_matches(
            model,
            campaign.manufacturer(),
            campaign.family(),
            campaign.variant(),
        )
    {
        identity.finding(R::AircraftFamilyMismatch);
    }
    identity.finish();

    let mut geometry = DomainReadiness::new(D::Geometry);
    scalar(
        &mut geometry,
        specification.and_then(|s| s.wingspan_m()),
        E::Wingspan,
        R::WingspanMissing,
    );
    scalar(
        &mut geometry,
        specification.and_then(|s| s.reference_wing_area_m2()),
        E::WingArea,
        R::WingAreaMissing,
    );
    scalar(
        &mut geometry,
        specification.and_then(|s| s.aerodynamic_reference_chord_m()),
        E::ReferenceChord,
        R::ReferenceChordMissing,
    );
    match input.survey {
        Some(survey) => {
            if survey.classification() != SurveyClassification::PhysicalReferenceMeasurement {
                geometry.finding(R::SyntheticEvidence);
            }
            if survey.evaluation().geometry_ready() {
                geometry.present(E::TailGeometry);
            } else {
                geometry.finding(R::TailSurveyIncomplete);
            }
        }
        None => geometry.finding(R::TailSurveyIncomplete),
    }
    if model.aero_elements().is_empty() {
        geometry.finding(R::AeroElementsMissing);
    } else {
        geometry.present(E::AeroElements);
    }
    if model.schema_version() >= 5 && model.aero_surfaces().is_empty() {
        geometry.finding(R::AeroSurfacesMissing);
    } else if !model.aero_surfaces().is_empty() {
        geometry.present(E::AeroSurfaces);
    }
    match specification.and_then(|s| s.cg_location()) {
        None => geometry.finding(R::CgDatumMissing),
        Some(cg) if !sourced(cg.evidence()) => geometry.finding(R::EvidenceUnresolved),
        Some(cg)
            if matches!(cg.reference_kind(), CgReferenceKind::Other)
                && cg.reference_description().is_none() =>
        {
            geometry.finding(R::CgDatumAmbiguous)
        }
        Some(_) => geometry.present(E::CgDatum),
    }
    geometry.finish();

    let mut mass = DomainReadiness::new(D::MassProperties);
    match specification.and_then(|s| s.mass()) {
        None => mass.finding(R::MassEvidenceMissing),
        Some(e) if !sourced(e) => mass.finding(R::EvidenceUnresolved),
        Some(_) => mass.present(E::MassProvenance),
    }
    match specification.and_then(|s| s.cg_location()) {
        None => mass.finding(R::CgEvidenceMissing),
        Some(cg) if !sourced(cg.evidence()) => mass.finding(R::EvidenceUnresolved),
        Some(_) => mass.present(E::CgProvenance),
    }
    match input.mass_campaign {
        None => mass.finding(R::MassCampaignMissing),
        Some(campaign) => {
            if campaign.classification() != SurveyClassification::PhysicalReferenceMeasurement {
                mass.finding(R::SyntheticEvidence);
            } else {
                mass.present(E::MassCampaign);
            }
            if campaign.operational_configuration_id()
                != Some(physical.operational_configuration_id)
            {
                mass.finding(R::OperationalConfigurationMismatch);
            }
            let e = campaign.evaluation();
            if e.mass_ready() {
                mass.present(E::TotalMass);
            } else {
                mass.finding(R::MassMeasurementMissing);
            }
            if e.cg_ready() {
                mass.present(E::Cg);
            } else {
                mass.finding(R::CgMeasurementMissing);
            }
            if e.inertia_ready() {
                mass.present(E::Inertia);
            } else {
                mass.finding(R::InertiaEvidenceMissing);
            }
        }
    }
    mass.finish();

    let mut aero = DomainReadiness::new(D::Aerodynamics);
    match input.aerodynamic_evidence {
        None => aero.finding(R::AerodynamicEvidenceMissing),
        Some(campaign) => {
            if campaign.classification() != SurveyClassification::PhysicalReferenceMeasurement {
                aero.finding(R::SyntheticEvidence);
            } else {
                aero.present(E::AerodynamicCampaign);
            }
            let e = campaign.evaluation();
            if e.airfoil_identity_ready() {
                aero.present(E::AirfoilIdentity);
            } else {
                aero.finding(R::AirfoilProvenanceUnresolved);
            }
            if e.coordinates_ready() {
                aero.present(E::AirfoilCoordinates);
            } else {
                aero.finding(R::CoordinatesProvenanceUnresolved);
            }
            if e.polar_evidence_ready() {
                aero.present(E::QualifiedPolars);
            } else {
                aero.finding(R::PolarEvidenceUnresolved);
            }
            if e.coverage_ready() {
                aero.present(E::ReynoldsEnvelope);
            } else {
                aero.finding(R::ReynoldsCoverageInsufficient);
            }
            match input.required_alpha_rad {
                None => aero.finding(R::AlphaEnvelopeMissing),
                Some((min, max)) if !min.is_finite() || !max.is_finite() || min >= max => {
                    aero.finding(R::InvalidAlphaEnvelope)
                }
                Some((min, max)) => {
                    if !e.datasets().is_empty()
                        && e.datasets().iter().all(|d| {
                            d.evidence_ready()
                                && d.alpha_min_rad() <= min
                                && d.alpha_max_rad() >= max
                        })
                    {
                        aero.present(E::AlphaEnvelope);
                    } else {
                        aero.finding(R::AlphaCoverageInsufficient);
                    }
                }
            }
            for dataset in e.datasets().iter().filter(|d| !d.evidence_ready()) {
                aero.subject(R::PolarEvidenceUnresolved, dataset.id());
            }
        }
    }
    aero.finish();

    let mut propulsion = DomainReadiness::new(D::Propulsion);
    if model.propulsion().is_none() {
        propulsion.status = ReadinessStatus::NotApplicable;
    } else {
        if physical
            .propulsion_configuration_id
            .is_none_or(|s| s.trim().is_empty())
        {
            propulsion.finding(R::PropulsionConfigurationIdMissing);
        }
        match input.propulsion_evidence {
            None => propulsion.finding(R::PropulsionEvidenceMissing),
            Some(campaign) => {
                if campaign.classification() != SurveyClassification::PhysicalReferenceMeasurement {
                    propulsion.finding(R::SyntheticEvidence);
                } else {
                    propulsion.present(E::PropulsionCampaign);
                }
                let campaign_identity_matches = campaign.physical_airframe_id()
                    == Some(physical.airframe_id)
                    && campaign.operational_configuration_id()
                        == Some(physical.operational_configuration_id)
                    && campaign.propulsion_configuration_id()
                        == physical.propulsion_configuration_id;
                if !campaign_identity_matches {
                    propulsion.finding(R::PhysicalInstallationMismatch);
                }
                let e = campaign.evaluation();
                if e.motor_evidence_ready() {
                    propulsion.present(E::Motor);
                } else {
                    propulsion.finding(R::MotorEvidenceMissing);
                }
                if e.esc_evidence_ready() {
                    propulsion.present(E::Esc);
                } else {
                    propulsion.finding(R::EscEvidenceMissing);
                }
                if e.battery_evidence_ready() {
                    propulsion.present(E::Battery);
                } else {
                    propulsion.finding(R::BatteryEvidenceMissing);
                }
                if e.propeller_evidence_ready() {
                    propulsion.present(E::Propeller);
                } else {
                    propulsion.finding(R::PropellerEvidenceMissing);
                }
                if e.propulsion_evidence_ready() {
                    propulsion.present(E::PropulsionDataset);
                } else {
                    propulsion.finding(R::PropulsionDatasetMissing);
                }
                let installations: Vec<_> = e
                    .configuration_claims()
                    .iter()
                    .filter(|claim| {
                        claim.evidence_class()
                            == PropulsionConfigurationEvidenceClass::SpecificInstalledConfiguration
                            || claim.evidence_class()
                                == PropulsionConfigurationEvidenceClass::MeasuredConfiguration
                    })
                    .collect();
                if installations.is_empty() {
                    propulsion.finding(R::PhysicalInstallationUnproven);
                } else if campaign_identity_matches
                    && installations.iter().any(|claim| {
                        claim.physical_airframe_id() == Some(physical.airframe_id)
                            && claim.operational_configuration_id()
                                == Some(physical.operational_configuration_id)
                            && claim.propulsion_configuration_id()
                                == physical.propulsion_configuration_id
                    })
                {
                    propulsion.present(E::PhysicalInstallation);
                } else if campaign_identity_matches {
                    propulsion.finding(R::PhysicalInstallationMismatch);
                }
            }
        }
        propulsion.finish();
    }

    let mut controls = DomainReadiness::new(D::Controls);
    if model.control_surface_bindings().is_empty() {
        controls.finding(R::ControlBindingMissing);
    } else {
        controls.present(E::ControlBindings);
    }
    controls.present(E::ServoConfiguration); // Strict model loader validates neutral, limits and rate.
    for (index, binding) in model.control_surface_bindings().iter().enumerate() {
        let travel = specification.and_then(|s| {
            s.control_surface_travel_limits()
                .iter()
                .find(|t| t.binding_index() == index)
        });
        match travel {
            Some(t) if sourced(t.evidence()) => controls.present(E::SurfaceTravel),
            Some(_) => controls.subject(R::EvidenceUnresolved, binding.id()),
            None => controls.subject(R::SurfaceTravelEvidenceMissing, binding.id()),
        }
    }
    controls.finish();

    let mut integrity = DomainReadiness::new(D::ModelIntegrity);
    integrity.present(E::ValidatedModel);
    if model.schema_version() > AIRCRAFT_MODEL_SCHEMA_VERSION_V9 {
        integrity.finding(R::UnsupportedSchema);
    }
    if !model.rigid_body().mass_kg().is_finite() {
        integrity.finding(R::NonFiniteModelData);
    }
    let physics_fingerprint = model.physics_fingerprint();
    integrity.present(E::PhysicsFingerprint);
    integrity.finish();

    let mut flight = DomainReadiness::new(D::FlightTestData);
    flight.finding(R::FlightTestEvidenceNotEvaluated);
    flight.finish();

    let domains = [
        identity, geometry, mass, aero, propulsion, controls, integrity, flight,
    ];
    let overall_status = combine_physical_status(&domains[..7]);
    let findings = domains
        .iter()
        .flat_map(|d| d.findings.iter().cloned().map(move |f| (d.domain, f)))
        .collect();
    ReferenceAircraftReadiness {
        model_id: model.model_id().to_owned(),
        physical_airframe_id: physical.airframe_id.to_owned(),
        operational_configuration_id: physical.operational_configuration_id.to_owned(),
        propulsion_configuration_id: physical.propulsion_configuration_id.map(str::to_owned),
        physics_fingerprint,
        overall_status,
        domains,
        findings,
    }
}

fn combine_physical_status(domains: &[DomainReadiness]) -> ReadinessStatus {
    if domains.iter().any(|d| d.status == ReadinessStatus::Blocked) {
        ReadinessStatus::Blocked
    } else if domains
        .iter()
        .any(|d| d.status == ReadinessStatus::Incomplete)
    {
        ReadinessStatus::Incomplete
    } else {
        ReadinessStatus::Ready
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_only_synthetic_complete_domain_fixture_aggregates_ready_without_certifying_an_aircraft()
    {
        let fixture = [
            ReadinessDomain::AircraftIdentity,
            ReadinessDomain::Geometry,
            ReadinessDomain::MassProperties,
            ReadinessDomain::Aerodynamics,
            ReadinessDomain::Propulsion,
            ReadinessDomain::Controls,
            ReadinessDomain::ModelIntegrity,
        ]
        .map(DomainReadiness::new);
        assert_eq!(combine_physical_status(&fixture), ReadinessStatus::Ready);
        let mut incomplete = fixture;
        incomplete[2].finding(ReadinessReason::InertiaEvidenceMissing);
        incomplete[2].finish();
        assert_eq!(
            combine_physical_status(&incomplete),
            ReadinessStatus::Incomplete
        );
        // No loaded model/evidence is involved in this decision-algebra fixture.
    }
}
