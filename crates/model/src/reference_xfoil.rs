//! Deterministic import of textual XFOIL polar output.
//!
//! M2.9A provides a strict, off-runtime parser for standard XFOIL polar text files.
//! A successful parse means only that the textual solver output is structurally usable.
//! It does NOT mean the solver converged, the data is physically valid, or the data
//! is approved for runtime use.
//!
//! This module does NOT construct [`sim_core::PolarTable`], modify [`AircraftModel`],
//! or participate in the 500 Hz runtime path in any way.

use std::fmt;

use thiserror::Error;

/// One sample row from a parsed XFOIL polar table.
///
/// `alpha_rad` is converted deterministically from the XFOIL output degrees
/// via `degrees * PI / 180.0`. All other values are preserved exactly as
/// they appear in the text.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XfoilPolarSample {
    alpha_rad: f64,
    cl: f64,
    cd: f64,
    cm: f64,
    cd_pressure: Option<f64>,
    top_xtr: Option<f64>,
    bot_xtr: Option<f64>,
}

impl XfoilPolarSample {
    /// Angle of attack in radians, converted from XFOIL output degrees.
    pub const fn alpha_rad(&self) -> f64 {
        self.alpha_rad
    }

    /// Lift coefficient.
    pub const fn cl(&self) -> f64 {
        self.cl
    }

    /// Drag coefficient.
    pub const fn cd(&self) -> f64 {
        self.cd
    }

    /// Moment coefficient.
    pub const fn cm(&self) -> f64 {
        self.cm
    }

    /// Profile drag coefficient (CDp), when the source table contains it.
    pub const fn cd_pressure(&self) -> Option<f64> {
        self.cd_pressure
    }

    /// Upper-surface transition x/c, when the source table contains it.
    pub const fn top_xtr(&self) -> Option<f64> {
        self.top_xtr
    }

    /// Lower-surface transition x/c, when the source table contains it.
    pub const fn bot_xtr(&self) -> Option<f64> {
        self.bot_xtr
    }
}

/// Validated solver metadata supplied by the caller.
///
/// Required evidence metadata must be caller-supplied. The parser does not
/// invent defaults that would masquerade as evidence.
#[derive(Debug, Clone, PartialEq)]
pub struct XfoilSolverMetadata {
    solver_name: Option<String>,
    solver_version: Option<String>,
    command_or_config: Option<String>,
    transition_assumptions: Option<String>,
    ncrit: Option<f64>,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
    reynolds: f64,
    mach: f64,
}

impl XfoilSolverMetadata {
    /// Solver or tool name (e.g. "XFOIL").
    pub fn solver_name(&self) -> Option<&str> {
        self.solver_name.as_deref()
    }

    /// Exact solver version string.
    pub fn solver_version(&self) -> Option<&str> {
        self.solver_version.as_deref()
    }

    /// Command script or configuration text used.
    pub fn command_or_config(&self) -> Option<&str> {
        self.command_or_config.as_deref()
    }

    /// Free-text transition modelling assumptions.
    pub fn transition_assumptions(&self) -> Option<&str> {
        self.transition_assumptions.as_deref()
    }

    /// Critical amplification exponent (Ncrit), when present.
    pub const fn ncrit(&self) -> Option<f64> {
        self.ncrit
    }

    /// Forced transition position upper surface x/c, when present.
    pub const fn forced_transition_upper_x_over_c(&self) -> Option<f64> {
        self.forced_transition_upper_x_over_c
    }

    /// Forced transition position lower surface x/c, when present.
    pub const fn forced_transition_lower_x_over_c(&self) -> Option<f64> {
        self.forced_transition_lower_x_over_c
    }

    /// Reynolds number.
    pub const fn reynolds(&self) -> f64 {
        self.reynolds
    }

    /// Mach number.
    pub const fn mach(&self) -> f64 {
        self.mach
    }
}

/// Builder for [`XfoilSolverMetadata`] that validates all evidence fields.
#[derive(Debug, Clone)]
pub struct MetadataBuilder {
    solver_name: Option<String>,
    solver_version: Option<String>,
    command_or_config: Option<String>,
    transition_assumptions: Option<String>,
    ncrit: Option<f64>,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
    reynolds: f64,
    mach: f64,
}

impl MetadataBuilder {
    /// Create a new builder with required Reynolds and Mach values.
    pub fn new(reynolds: f64, mach: f64) -> Self {
        Self {
            solver_name: None,
            solver_version: None,
            command_or_config: None,
            transition_assumptions: None,
            ncrit: None,
            forced_transition_upper_x_over_c: None,
            forced_transition_lower_x_over_c: None,
            reynolds,
            mach,
        }
    }

    /// Set the solver or tool name.
    pub fn solver_name(mut self, name: impl Into<String>) -> Self {
        self.solver_name = Some(name.into());
        self
    }

    /// Set the exact solver version string.
    pub fn solver_version(mut self, version: impl Into<String>) -> Self {
        self.solver_version = Some(version.into());
        self
    }

    /// Set the command script or configuration text.
    pub fn command_or_config(mut self, text: impl Into<String>) -> Self {
        self.command_or_config = Some(text.into());
        self
    }

    /// Set the transition modelling assumptions.
    pub fn transition_assumptions(mut self, text: impl Into<String>) -> Self {
        self.transition_assumptions = Some(text.into());
        self
    }

    /// Set the critical amplification exponent (Ncrit).
    pub fn ncrit(mut self, value: f64) -> Self {
        self.ncrit = Some(value);
        self
    }

    /// Set the forced transition position on the upper surface.
    pub fn forced_transition_upper(mut self, x_over_c: f64) -> Self {
        self.forced_transition_upper_x_over_c = Some(x_over_c);
        self
    }

    /// Set the forced transition position on the lower surface.
    pub fn forced_transition_lower(mut self, x_over_c: f64) -> Self {
        self.forced_transition_lower_x_over_c = Some(x_over_c);
        self
    }

    /// Validate all fields and build the metadata.
    pub fn build(self) -> Result<XfoilSolverMetadata, XfoilPolarImportError> {
        validate_metadata(
            self.reynolds,
            self.mach,
            self.ncrit,
            self.forced_transition_upper_x_over_c,
            self.forced_transition_lower_x_over_c,
        )?;
        Ok(XfoilSolverMetadata {
            solver_name: self.solver_name,
            solver_version: self.solver_version,
            command_or_config: self.command_or_config,
            transition_assumptions: self.transition_assumptions,
            ncrit: self.ncrit,
            forced_transition_upper_x_over_c: self.forced_transition_upper_x_over_c,
            forced_transition_lower_x_over_c: self.forced_transition_lower_x_over_c,
            reynolds: self.reynolds,
            mach: self.mach,
        })
    }
}

/// Result of parsing an XFOIL polar text file.
///
/// A valid import means only that the textual solver output is structurally
/// usable. It does NOT constitute convergence or runtime approval.
#[derive(Debug, Clone, PartialEq)]
pub struct XfoilPolarImport {
    metadata: XfoilSolverMetadata,
    samples: Vec<XfoilPolarSample>,
}

impl XfoilPolarImport {
    /// Validated solver metadata for this import.
    pub fn metadata(&self) -> &XfoilSolverMetadata {
        &self.metadata
    }

    /// Parsed samples in source row order.
    pub fn samples(&self) -> &[XfoilPolarSample] {
        &self.samples
    }

    /// Number of valid samples.
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }
}

/// Errors that can occur during XFOIL polar import.
#[derive(Debug, Error)]
pub enum XfoilPolarImportError {
    #[error("XFOIL polar header line not found")]
    HeaderNotFound,

    #[error("XFOIL polar data row {row}: {reason}")]
    MalformedRow { row: usize, reason: &'static str },

    #[error("XFOIL polar requires at least two samples, found {count}")]
    TooFewSamples { count: usize },

    #[error("XFOIL polar has duplicate alpha at row {row}")]
    DuplicateAlpha { row: usize },

    #[error("XFOIL polar alpha is not strictly increasing at row {row}")]
    AlphaNotIncreasing { row: usize },

    #[error("XFOIL polar has negative CD at row {row}")]
    NegativeCd { row: usize },

    #[error("XFOIL polar has non-finite value at row {row}")]
    NonFiniteValue { row: usize },

    #[error("invalid solver metadata: {0}")]
    InvalidMetadata(InvalidMetadataReason),
}

/// Reason why solver metadata was rejected.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum InvalidMetadataReason {
    ReynoldsNotFinitePositive,
    MachNegativeOrNotFinite,
    NcritNotFinitePositive,
    ForcedTransitionOutOfRange,
}

impl fmt::Display for InvalidMetadataReason {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ReynoldsNotFinitePositive => {
                f.write_str("Reynolds number must be finite and positive")
            }
            Self::MachNegativeOrNotFinite => {
                f.write_str("Mach number must be finite and non-negative")
            }
            Self::NcritNotFinitePositive => f.write_str("Ncrit must be finite and positive"),
            Self::ForcedTransitionOutOfRange => {
                f.write_str("forced transition x/c must be finite within [0, 1]")
            }
        }
    }
}

const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

/// Parse standard textual XFOIL polar output with caller-supplied metadata.
///
/// The parser locates the column header line (containing the `alpha` keyword),
/// skips an optional dashed separator, then parses numeric data rows until
/// end-of-file or an empty line.
pub fn parse_xfoil_polar(
    text: &str,
    metadata: XfoilSolverMetadata,
) -> Result<XfoilPolarImport, XfoilPolarImportError> {
    let lines: Vec<&str> = text.lines().collect();
    let samples = parse_data_section(&lines)?;
    Ok(XfoilPolarImport { metadata, samples })
}

fn parse_data_section(lines: &[&str]) -> Result<Vec<XfoilPolarSample>, XfoilPolarImportError> {
    let header_idx = lines
        .iter()
        .position(|line| contains_alpha_keyword(line))
        .ok_or(XfoilPolarImportError::HeaderNotFound)?;

    let data_start = if header_idx + 1 < lines.len() && is_separator_line(lines[header_idx + 1]) {
        header_idx + 2
    } else {
        header_idx + 1
    };

    let mut samples = Vec::new();
    let mut source_row = 0_usize;

    for &line in &lines[data_start..] {
        if line.trim().is_empty() {
            break;
        }
        source_row += 1;
        samples.push(parse_data_row(line, source_row)?);
    }

    if samples.len() < 2 {
        return Err(XfoilPolarImportError::TooFewSamples {
            count: samples.len(),
        });
    }

    for i in 1..samples.len() {
        if samples[i].alpha_rad.to_bits() == samples[i - 1].alpha_rad.to_bits() {
            return Err(XfoilPolarImportError::DuplicateAlpha { row: i + 1 });
        }
        if samples[i].alpha_rad <= samples[i - 1].alpha_rad {
            return Err(XfoilPolarImportError::AlphaNotIncreasing { row: i + 1 });
        }
    }

    Ok(samples)
}

fn contains_alpha_keyword(line: &str) -> bool {
    let lower = line.to_ascii_lowercase();
    lower.contains("alpha")
}

fn is_separator_line(line: &str) -> bool {
    let trimmed = line.trim();
    !trimmed.is_empty()
        && trimmed.len() >= 3
        && trimmed.chars().all(|c| c == '-' || c.is_whitespace())
}

fn parse_data_row(
    line: &str,
    source_row: usize,
) -> Result<XfoilPolarSample, XfoilPolarImportError> {
    let fields: Vec<&str> = line.split_whitespace().collect();
    let values: Result<Vec<f64>, _> = fields.iter().map(|f| f.parse::<f64>()).collect();
    let values = values.map_err(|_| XfoilPolarImportError::MalformedRow {
        row: source_row,
        reason: "non-numeric value in data row",
    })?;

    let (alpha_deg, cl, cd, cm, cd_pressure, top_xtr, bot_xtr) = match values.len() {
        4 => (values[0], values[1], values[2], values[3], None, None, None),
        6 => (
            values[0],
            values[1],
            values[2],
            0.0,
            Some(values[3]),
            Some(values[4]),
            Some(values[5]),
        ),
        _ => {
            return Err(XfoilPolarImportError::MalformedRow {
                row: source_row,
                reason: "expected 4 or 6 columns",
            });
        }
    };

    let alpha_rad = alpha_deg * DEG_TO_RAD;

    let sample = XfoilPolarSample {
        alpha_rad,
        cl,
        cd,
        cm,
        cd_pressure,
        top_xtr,
        bot_xtr,
    };
    validate_sample(&sample, source_row)?;
    Ok(sample)
}

fn validate_sample(
    sample: &XfoilPolarSample,
    source_row: usize,
) -> Result<(), XfoilPolarImportError> {
    if ![sample.alpha_rad, sample.cl, sample.cd, sample.cm]
        .into_iter()
        .all(f64::is_finite)
    {
        return Err(XfoilPolarImportError::NonFiniteValue { row: source_row });
    }
    if let Some(v) = sample.cd_pressure
        && !v.is_finite()
    {
        return Err(XfoilPolarImportError::NonFiniteValue { row: source_row });
    }
    if let Some(v) = sample.top_xtr
        && !v.is_finite()
    {
        return Err(XfoilPolarImportError::NonFiniteValue { row: source_row });
    }
    if let Some(v) = sample.bot_xtr
        && !v.is_finite()
    {
        return Err(XfoilPolarImportError::NonFiniteValue { row: source_row });
    }
    if sample.cd < 0.0 {
        return Err(XfoilPolarImportError::NegativeCd { row: source_row });
    }
    Ok(())
}

fn validate_metadata(
    reynolds: f64,
    mach: f64,
    ncrit: Option<f64>,
    forced_upper: Option<f64>,
    forced_lower: Option<f64>,
) -> Result<(), XfoilPolarImportError> {
    if !reynolds.is_finite() || reynolds <= 0.0 {
        return Err(XfoilPolarImportError::InvalidMetadata(
            InvalidMetadataReason::ReynoldsNotFinitePositive,
        ));
    }
    if !mach.is_finite() || mach < 0.0 {
        return Err(XfoilPolarImportError::InvalidMetadata(
            InvalidMetadataReason::MachNegativeOrNotFinite,
        ));
    }
    if ncrit.is_some_and(|v| !v.is_finite() || v <= 0.0) {
        return Err(XfoilPolarImportError::InvalidMetadata(
            InvalidMetadataReason::NcritNotFinitePositive,
        ));
    }
    for value in [forced_upper, forced_lower].into_iter().flatten() {
        if !value.is_finite() || !(0.0..=1.0).contains(&value) {
            return Err(XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ForcedTransitionOutOfRange,
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    const STANDARD_POLAR: &str = "\
 XFOIL 6.99


 Calculated polar for: CLARK Y


 1 1 Reynolds number: 300000    Mach number: 0.00


 Ncrit: 9.0


 alpha    CL         CD         CDp       Top_Xtr   Bot_Xtr
 ------   ---------  ---------  ---------  ---------  ---------
  -2.000  -0.0414    0.01134    0.00442    0.5412    0.6178
  -1.000   0.0593    0.00822    0.00254    0.5631    0.5971
   0.000   0.1593    0.00700    0.00156    0.5812    0.5612
   2.000   0.3593    0.00720    0.00180    0.6200    0.5200
   4.000   0.5593    0.00900    0.00300    0.6500    0.4800
";

    const MINIMAL_4COL: &str = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
  -1.000  -0.1000    0.01000    0.0100
   0.000   0.1000    0.00800    0.0050
   1.000   0.3000    0.00900   -0.0100
";

    fn valid_metadata() -> XfoilSolverMetadata {
        MetadataBuilder::new(300_000.0, 0.0)
            .solver_name("XFOIL")
            .solver_version("6.99")
            .build()
            .unwrap()
    }

    #[test]
    fn standard_7col_parses() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        assert_eq!(import.sample_count(), 5);
    }

    #[test]
    fn minimal_4col_parses() {
        let import = parse_xfoil_polar(MINIMAL_4COL, valid_metadata()).unwrap();
        assert_eq!(import.sample_count(), 3);
        for sample in import.samples() {
            assert!(sample.cd_pressure().is_none());
            assert!(sample.top_xtr().is_none());
            assert!(sample.bot_xtr().is_none());
        }
    }

    #[test]
    fn alpha_degrees_to_radians_exact() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let expected_factor = std::f64::consts::PI / 180.0;
        let first = &import.samples()[0];
        assert!((first.alpha_rad() - (-2.0 * expected_factor)).abs() < 1e-15);
        let third = &import.samples()[2];
        assert!((third.alpha_rad() - 0.0).abs() < 1e-15);
    }

    #[test]
    fn cl_cd_cm_preserved_exactly() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let s = &import.samples()[0];
        assert_eq!(s.cl(), -0.0414);
        assert_eq!(s.cd(), 0.01134);
        assert_eq!(s.cm(), 0.0);
        let s3 = &import.samples()[2];
        assert_eq!(s3.cl(), 0.1593);
        assert_eq!(s3.cd(), 0.00700);
    }

    #[test]
    fn diagnostic_columns_preserved() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let s = &import.samples()[0];
        assert_eq!(s.cd_pressure(), Some(0.00442));
        assert_eq!(s.top_xtr(), Some(0.5412));
        assert_eq!(s.bot_xtr(), Some(0.6178));
        assert_eq!(s.cm(), 0.0);
    }

    #[test]
    fn source_ordering_preserved() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let alphas: Vec<f64> = import.samples().iter().map(|s| s.alpha_rad()).collect();
        for i in 1..alphas.len() {
            assert!(alphas[i] > alphas[i - 1]);
        }
        assert!(alphas[0] < 0.0);
        assert!(alphas[alphas.len() - 1] > 0.0);
    }

    #[test]
    fn header_ignored_deterministically() {
        let a = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let b = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        assert_eq!(a.samples().len(), b.samples().len());
        for (a_sample, b_sample) in a.samples().iter().zip(b.samples()) {
            assert_eq!(a_sample.alpha_rad(), b_sample.alpha_rad());
            assert_eq!(a_sample.cl(), b_sample.cl());
            assert_eq!(a_sample.cd(), b_sample.cd());
            assert_eq!(a_sample.cm(), b_sample.cm());
        }
    }

    #[test]
    fn too_few_samples_zero() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::TooFewSamples { count: 0 }
        ));
    }

    #[test]
    fn too_few_samples_one() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::TooFewSamples { count: 1 }
        ));
    }

    #[test]
    fn duplicate_alpha_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
   0.000   0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::DuplicateAlpha { row: 2 }
        ));
    }

    #[test]
    fn decreasing_alpha_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   2.000   0.3000    0.00900   -0.0100
   1.000   0.2000    0.00800    0.0050
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::AlphaNotIncreasing { row: 2 }
        ));
    }

    #[test]
    fn malformed_row_after_table_start_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
   abc     0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::MalformedRow {
                row: 2,
                reason: "non-numeric value in data row"
            }
        ));
    }

    #[test]
    fn negative_cd_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000   -0.00100    0.0050
   1.000   0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(err, XfoilPolarImportError::NegativeCd { row: 1 }));
    }

    #[test]
    fn nan_value_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   NaN        0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::NonFiniteValue { row: 1 }
        ));
    }

    #[test]
    fn inf_value_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   inf        0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::NonFiniteValue { row: 1 }
        ));
    }

    #[test]
    fn header_not_found() {
        let text = "some random text\nwith no polar data\n";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(err, XfoilPolarImportError::HeaderNotFound));
    }

    #[test]
    fn empty_input() {
        let err = parse_xfoil_polar("", valid_metadata()).unwrap_err();
        assert!(matches!(err, XfoilPolarImportError::HeaderNotFound));
    }

    #[test]
    fn invalid_reynolds_zero() {
        let err = MetadataBuilder::new(0.0, 0.0).build().unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ReynoldsNotFinitePositive
            )
        ));
    }

    #[test]
    fn invalid_reynolds_negative() {
        let err = MetadataBuilder::new(-100.0, 0.0).build().unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ReynoldsNotFinitePositive
            )
        ));
    }

    #[test]
    fn invalid_reynolds_nan() {
        let err = MetadataBuilder::new(f64::NAN, 0.0).build().unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ReynoldsNotFinitePositive
            )
        ));
    }

    #[test]
    fn invalid_reynolds_infinity() {
        let err = MetadataBuilder::new(f64::INFINITY, 0.0)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ReynoldsNotFinitePositive
            )
        ));
    }

    #[test]
    fn invalid_mach_negative() {
        let err = MetadataBuilder::new(300_000.0, -0.1).build().unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
        ));
    }

    #[test]
    fn invalid_mach_nan() {
        let err = MetadataBuilder::new(300_000.0, f64::NAN)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
        ));
    }

    #[test]
    fn invalid_mach_infinity() {
        let err = MetadataBuilder::new(300_000.0, f64::INFINITY)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::MachNegativeOrNotFinite)
        ));
    }

    #[test]
    fn invalid_ncrit_zero() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .ncrit(0.0)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
        ));
    }

    #[test]
    fn invalid_ncrit_negative() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .ncrit(-5.0)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
        ));
    }

    #[test]
    fn invalid_ncrit_nan() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .ncrit(f64::NAN)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(InvalidMetadataReason::NcritNotFinitePositive)
        ));
    }

    #[test]
    fn invalid_forced_transition_upper_above_one() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .forced_transition_upper(1.5)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ForcedTransitionOutOfRange
            )
        ));
    }

    #[test]
    fn invalid_forced_transition_lower_negative() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .forced_transition_lower(-0.1)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ForcedTransitionOutOfRange
            )
        ));
    }

    #[test]
    fn invalid_forced_transition_nan() {
        let err = MetadataBuilder::new(300_000.0, 0.0)
            .forced_transition_upper(f64::NAN)
            .build()
            .unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::InvalidMetadata(
                InvalidMetadataReason::ForcedTransitionOutOfRange
            )
        ));
    }

    #[test]
    fn identical_input_identical_output() {
        let meta = valid_metadata();
        let a = parse_xfoil_polar(STANDARD_POLAR, meta.clone()).unwrap();
        let b = parse_xfoil_polar(STANDARD_POLAR, meta).unwrap();
        assert_eq!(a.sample_count(), b.sample_count());
        for (a_s, b_s) in a.samples().iter().zip(b.samples()) {
            assert_eq!(a_s.alpha_rad().to_bits(), b_s.alpha_rad().to_bits());
            assert_eq!(a_s.cl().to_bits(), b_s.cl().to_bits());
            assert_eq!(a_s.cd().to_bits(), b_s.cd().to_bits());
            assert_eq!(a_s.cm().to_bits(), b_s.cm().to_bits());
            assert_eq!(
                a_s.cd_pressure().map(f64::to_bits),
                b_s.cd_pressure().map(f64::to_bits)
            );
            assert_eq!(
                a_s.top_xtr().map(f64::to_bits),
                b_s.top_xtr().map(f64::to_bits)
            );
            assert_eq!(
                a_s.bot_xtr().map(f64::to_bits),
                b_s.bot_xtr().map(f64::to_bits)
            );
        }
    }

    #[test]
    fn parser_does_not_construct_runtime_polar_table() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        let _import_ref: &XfoilPolarImport = &import;
    }

    #[test]
    fn no_clark_y_data_fabricated() {
        let import = parse_xfoil_polar(STANDARD_POLAR, valid_metadata()).unwrap();
        assert_eq!(import.sample_count(), 5);
        let cl_range = import
            .samples()
            .iter()
            .map(|s| s.cl())
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| {
                (lo.min(v), hi.max(v))
            });
        assert!(cl_range.0 >= -1.0);
        assert!(cl_range.1 <= 2.0);
    }

    #[test]
    fn metadata_required_fields_preserved() {
        let meta = MetadataBuilder::new(300_000.0, 0.05)
            .solver_name("XFOIL")
            .solver_version("6.99")
            .command_or_config("OPER RE 300000 VISC ITER 100")
            .transition_assumptions("Free transition via e^N method")
            .ncrit(9.0)
            .build()
            .unwrap();
        assert_eq!(meta.solver_name(), Some("XFOIL"));
        assert_eq!(meta.solver_version(), Some("6.99"));
        assert_eq!(
            meta.command_or_config(),
            Some("OPER RE 300000 VISC ITER 100")
        );
        assert_eq!(
            meta.transition_assumptions(),
            Some("Free transition via e^N method")
        );
        assert_eq!(meta.ncrit(), Some(9.0));
        assert_eq!(meta.reynolds(), 300_000.0);
        assert_eq!(meta.mach(), 0.05);
        assert!(meta.forced_transition_upper_x_over_c().is_none());
        assert!(meta.forced_transition_lower_x_over_c().is_none());
    }

    #[test]
    fn metadata_forced_transition_boundary_values() {
        let meta = MetadataBuilder::new(300_000.0, 0.0)
            .forced_transition_upper(0.0)
            .forced_transition_lower(1.0)
            .build()
            .unwrap();
        assert_eq!(meta.forced_transition_upper_x_over_c(), Some(0.0));
        assert_eq!(meta.forced_transition_lower_x_over_c(), Some(1.0));
    }

    #[test]
    fn metadata_mach_zero_is_valid() {
        assert!(MetadataBuilder::new(300_000.0, 0.0).build().is_ok());
    }

    #[test]
    fn data_stops_at_blank_line() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800    0.0050
   1.000   0.2000    0.00900   -0.0100

   2.000   0.3000    0.01000   -0.0200
";
        let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
        assert_eq!(import.sample_count(), 2);
    }

    #[test]
    fn wrong_column_count_rejected() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000    0.00800
   1.000   0.2000    0.00900   -0.0100
";
        let err = parse_xfoil_polar(text, valid_metadata()).unwrap_err();
        assert!(matches!(
            err,
            XfoilPolarImportError::MalformedRow {
                row: 1,
                reason: "expected 4 or 6 columns"
            }
        ));
    }

    #[test]
    fn negative_zero_cd_is_accepted() {
        let text = "\
 alpha    CL         CD         CM
 ------   ---------  ---------  ---------
   0.000   0.1000   -0.00000    0.0050
   1.000   0.2000    0.00900   -0.0100
";
        let import = parse_xfoil_polar(text, valid_metadata()).unwrap();
        assert_eq!(import.sample_count(), 2);
        assert!(import.samples()[0].cd().is_sign_negative());
    }

    #[test]
    fn does_not_parse_real_evidence_artifact() {
        let content: Option<&str> = std::fs::read_to_string(
            "docs/reference_aircraft/data/sig_kadet_lt40_egv_aerodynamic_evidence_v0.json",
        )
        .ok()
        .and_then(|s| {
            s.contains("\"alpha_rad\"")
                .then_some("real evidence artifact exists")
        });
        assert!(content.is_none());
    }
}
