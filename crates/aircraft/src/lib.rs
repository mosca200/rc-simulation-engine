#![forbid(unsafe_code)]
//! Deterministic headless assembly of an immutable aircraft model and its runtime state.

mod config;
mod simulation;

pub use config::AircraftSimulationConfig;
pub use simulation::{
    AircraftAeroElementOutput, AircraftSimulation, AircraftSimulationError, AircraftSnapshot,
    AircraftState, deflected_aero_element, evaluate_aerodynamic_wrench,
    evaluate_aircraft_aero_element, evaluate_aircraft_wrench,
};
