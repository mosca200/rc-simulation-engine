#![forbid(unsafe_code)]
//! Deterministic headless assembly of an immutable aircraft model and its runtime state.

mod config;
mod simulation;
mod trim;
mod trim_sweep;

pub use config::AircraftSimulationConfig;
pub use simulation::{
    AircraftAeroElementOutput, AircraftInstantaneousEvaluation, AircraftSimulation,
    AircraftSimulationError, AircraftSnapshot, AircraftState, apply_control_surface_positions,
    deflected_aero_element, effective_aero_elements_for_positions, evaluate_aerodynamic_wrench,
    evaluate_aircraft_aero_element, evaluate_aircraft_instantaneous, evaluate_aircraft_wrench,
};
pub use trim::{
    LongitudinalTrimEvaluation, LongitudinalTrimFailure, LongitudinalTrimFailureReason,
    LongitudinalTrimRequest, LongitudinalTrimRequestError, LongitudinalTrimResiduals,
    LongitudinalTrimSolution, LongitudinalTrimTolerances, LongitudinalTrimVariables, TrimBounds,
    evaluate_longitudinal_trim_candidate, solve_longitudinal_trim,
};
pub use trim_sweep::{
    LongitudinalTrimSweep, LongitudinalTrimSweepError, LongitudinalTrimSweepOutcome,
    LongitudinalTrimSweepPoint, LongitudinalTrimSweepRequest, ReEvaluationMismatchDetail,
    ReEvaluationUnverifiableDetail, solve_longitudinal_trim_sweep,
};
