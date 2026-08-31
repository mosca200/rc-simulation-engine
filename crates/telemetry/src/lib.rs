#![forbid(unsafe_code)]
//! Small deterministic telemetry boundary. Performance diagnostics stay separate.

mod aircraft;

pub use aircraft::{
    AIRCRAFT_TELEMETRY_SCHEMA_VERSION, AircraftTelemetryFrame, AircraftTelemetryRecorder,
    AircraftTelemetryRecording, DeterministicTelemetrySummary, ScalarRange, ScalarStatistics,
    TelemetryCaptureError, TelemetryFinalState, TelemetrySummary,
};

use sim_core::{PilotInput, SimSnapshot};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryFrame {
    pub snapshot: SimSnapshot,
    pub pilot_input: PilotInput,
}

/// Non-deterministic diagnostic data; never included in state hashes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PerformanceDiagnostics {
    pub elapsed_wall_time_s: f64,
    pub average_step_time_s: f64,
}
