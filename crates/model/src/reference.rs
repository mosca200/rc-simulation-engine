//! Loaded documentary reference-aircraft metadata.
//!
//! These types are resolved once by the model loader and are never consulted by the physics
//! stepping path.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AircraftClassification {
    SyntheticTest,
    ReferenceAircraft,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterQuality {
    Measured,
    ManufacturerSpec,
    Published,
    Derived,
    Estimated,
    Unknown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceSourceType {
    ManufacturerDocumentation,
    Measured,
    PublishedResearch,
    AirfoilDatabase,
    NumericalAnalysis,
    Derived,
    Estimated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceConfidence {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProvenanceSource {
    pub(crate) id: String,
    pub(crate) source_type: ProvenanceSourceType,
    pub(crate) title: Option<String>,
    pub(crate) url: Option<String>,
    pub(crate) bibliographic_reference: Option<String>,
    pub(crate) notes: Option<String>,
    pub(crate) publication_date: Option<String>,
    pub(crate) retrieval_date: Option<String>,
    pub(crate) confidence: Option<ProvenanceConfidence>,
}

impl ProvenanceSource {
    pub fn id(&self) -> &str {
        &self.id
    }
    pub const fn source_type(&self) -> ProvenanceSourceType {
        self.source_type
    }
    pub fn title(&self) -> Option<&str> {
        self.title.as_deref()
    }
    pub fn url(&self) -> Option<&str> {
        self.url.as_deref()
    }
    pub fn bibliographic_reference(&self) -> Option<&str> {
        self.bibliographic_reference.as_deref()
    }
    pub fn notes(&self) -> Option<&str> {
        self.notes.as_deref()
    }
    pub fn publication_date(&self) -> Option<&str> {
        self.publication_date.as_deref()
    }
    pub fn retrieval_date(&self) -> Option<&str> {
        self.retrieval_date.as_deref()
    }
    pub const fn confidence(&self) -> Option<ProvenanceConfidence> {
        self.confidence
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceParameterEvidence {
    pub(crate) quality: ParameterQuality,
    pub(crate) source_indices: Vec<usize>,
}

impl ReferenceParameterEvidence {
    pub const fn quality(&self) -> ParameterQuality {
        self.quality
    }
    pub fn source_indices(&self) -> &[usize] {
        &self.source_indices
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceScalar {
    pub(crate) value: f64,
    pub(crate) evidence: ReferenceParameterEvidence,
}

impl ReferenceScalar {
    pub const fn value(&self) -> f64 {
        self.value
    }
    pub const fn evidence(&self) -> &ReferenceParameterEvidence {
        &self.evidence
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceAircraftIdentity {
    pub(crate) manufacturer: Option<String>,
    pub(crate) aircraft_name: Option<String>,
    pub(crate) variant: Option<String>,
    pub(crate) stable_reference_id: Option<String>,
    pub(crate) notes: Option<String>,
}

impl ReferenceAircraftIdentity {
    pub fn manufacturer(&self) -> Option<&str> {
        self.manufacturer.as_deref()
    }
    pub fn aircraft_name(&self) -> Option<&str> {
        self.aircraft_name.as_deref()
    }
    pub fn variant(&self) -> Option<&str> {
        self.variant.as_deref()
    }
    pub fn stable_reference_id(&self) -> Option<&str> {
        self.stable_reference_id.as_deref()
    }
    pub fn notes(&self) -> Option<&str> {
        self.notes.as_deref()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CgReferenceKind {
    BodyFrameOriginFrd,
    WingRootLeadingEdge,
    MeanAerodynamicChordLeadingEdge,
    ManufacturerDatum,
    Other,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceCgLocation {
    pub(crate) position_m_from_reference: [f64; 3],
    pub(crate) reference_kind: CgReferenceKind,
    pub(crate) reference_description: Option<String>,
    pub(crate) evidence: ReferenceParameterEvidence,
}

impl ReferenceCgLocation {
    pub const fn position_m_from_reference(&self) -> &[f64; 3] {
        &self.position_m_from_reference
    }
    pub const fn reference_kind(&self) -> CgReferenceKind {
        self.reference_kind
    }
    pub fn reference_description(&self) -> Option<&str> {
        self.reference_description.as_deref()
    }
    pub const fn evidence(&self) -> &ReferenceParameterEvidence {
        &self.evidence
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReferenceControlSurfaceTravel {
    pub(crate) binding_index: usize,
    pub(crate) evidence: ReferenceParameterEvidence,
}

impl ReferenceControlSurfaceTravel {
    pub const fn binding_index(&self) -> usize {
        self.binding_index
    }
    pub const fn evidence(&self) -> &ReferenceParameterEvidence {
        &self.evidence
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReferencePhysicalSpecification {
    pub(crate) wingspan_m: Option<ReferenceScalar>,
    pub(crate) reference_wing_area_m2: Option<ReferenceScalar>,
    pub(crate) aircraft_length_m: Option<ReferenceScalar>,
    pub(crate) mass: Option<ReferenceParameterEvidence>,
    pub(crate) cg_location: Option<ReferenceCgLocation>,
    pub(crate) aerodynamic_reference_chord_m: Option<ReferenceScalar>,
    pub(crate) wing_incidence_rad: Option<ReferenceScalar>,
    pub(crate) horizontal_tail_incidence_rad: Option<ReferenceScalar>,
    pub(crate) wing_dihedral_rad: Option<ReferenceScalar>,
    pub(crate) control_surface_travel_limits: Vec<ReferenceControlSurfaceTravel>,
}

impl ReferencePhysicalSpecification {
    pub const fn wingspan_m(&self) -> Option<&ReferenceScalar> {
        self.wingspan_m.as_ref()
    }
    pub const fn reference_wing_area_m2(&self) -> Option<&ReferenceScalar> {
        self.reference_wing_area_m2.as_ref()
    }
    pub const fn aircraft_length_m(&self) -> Option<&ReferenceScalar> {
        self.aircraft_length_m.as_ref()
    }
    /// Evidence for `AircraftModel::rigid_body().mass_kg()`; no duplicate mass value exists.
    pub const fn mass(&self) -> Option<&ReferenceParameterEvidence> {
        self.mass.as_ref()
    }
    pub const fn cg_location(&self) -> Option<&ReferenceCgLocation> {
        self.cg_location.as_ref()
    }
    pub const fn aerodynamic_reference_chord_m(&self) -> Option<&ReferenceScalar> {
        self.aerodynamic_reference_chord_m.as_ref()
    }
    pub const fn wing_incidence_rad(&self) -> Option<&ReferenceScalar> {
        self.wing_incidence_rad.as_ref()
    }
    pub const fn horizontal_tail_incidence_rad(&self) -> Option<&ReferenceScalar> {
        self.horizontal_tail_incidence_rad.as_ref()
    }
    pub const fn wing_dihedral_rad(&self) -> Option<&ReferenceScalar> {
        self.wing_dihedral_rad.as_ref()
    }
    pub fn control_surface_travel_limits(&self) -> &[ReferenceControlSurfaceTravel] {
        &self.control_surface_travel_limits
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ReferenceAircraftMetadata {
    pub(crate) identity: ReferenceAircraftIdentity,
    pub(crate) physical_specification: ReferencePhysicalSpecification,
    pub(crate) provenance_sources: Vec<ProvenanceSource>,
}

impl ReferenceAircraftMetadata {
    pub const fn identity(&self) -> &ReferenceAircraftIdentity {
        &self.identity
    }
    pub const fn physical_specification(&self) -> &ReferencePhysicalSpecification {
        &self.physical_specification
    }
    pub fn provenance_sources(&self) -> &[ProvenanceSource] {
        &self.provenance_sources
    }
}
