#![forbid(unsafe_code)]
//! Deterministic headless assembly of an immutable aircraft model and its runtime state.

mod config;
mod simulation;
mod trim;
mod trim_characterization;
mod trim_qualification;
mod trim_sweep;

pub use config::AircraftSimulationConfig;
pub use simulation::{
    AircraftAeroElementOutput, AircraftGroundTelemetry, AircraftInstantaneousEvaluation,
    AircraftSimulation, AircraftSimulationError, AircraftSnapshot, AircraftState,
    AircraftSurfaceAerodynamicState, GroundStageContext, PropellerSlipstream,
    apply_control_surface_positions, deflected_aero_element, effective_aero_elements_for_positions,
    evaluate_aerodynamic_wrench, evaluate_aerodynamic_wrench_with_propulsion,
    evaluate_aircraft_aero_element, evaluate_aircraft_instantaneous,
    evaluate_aircraft_instantaneous_with_ground, evaluate_aircraft_section_kinematics,
    evaluate_aircraft_surface_aerodynamic_state, evaluate_aircraft_wrench,
    evaluate_aircraft_wrench_with_ground, propeller_slipstream,
};
pub use trim::{
    LongitudinalTrimEvaluation, LongitudinalTrimFailure, LongitudinalTrimFailureReason,
    LongitudinalTrimRequest, LongitudinalTrimRequestError, LongitudinalTrimResiduals,
    LongitudinalTrimSolution, LongitudinalTrimTolerances, LongitudinalTrimVariables, TrimBounds,
    evaluate_longitudinal_trim_candidate, solve_longitudinal_trim,
};
pub use trim_characterization::{
    CharacterizationStepsError, CharacterizationUnavailableReason,
    LongitudinalTrimCharacterization, LongitudinalTrimCharacterizationData,
    LongitudinalTrimCharacterizationError, LongitudinalTrimCharacterizationPoint,
    LongitudinalTrimCharacterizationPointOutcome, LongitudinalTrimCharacterizationSteps,
    PerturbationSide, characterize_longitudinal_trim_sweep,
};
pub use trim_qualification::{
    AerodynamicElementDomainAudit, FullResidualAudit, LongitudinalTrimQualification,
    LongitudinalTrimQualificationLimits, LongitudinalTrimQualificationOutcome,
    LongitudinalTrimQualificationPoint, PropulsionDomainAudit, QualificationBlocker,
    QualificationLimitsError, RangeStatus, ShaftSpeedDomainAudit,
    qualify_longitudinal_trim_solution,
};
pub use trim_sweep::{
    LongitudinalTrimSweep, LongitudinalTrimSweepError, LongitudinalTrimSweepOutcome,
    LongitudinalTrimSweepPoint, LongitudinalTrimSweepRequest, ReEvaluationMismatchDetail,
    ReEvaluationUnverifiableDetail, solve_longitudinal_trim_sweep,
};
