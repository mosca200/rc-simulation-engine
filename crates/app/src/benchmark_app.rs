use aircraft::{
    AircraftSimulation, AircraftSimulationConfig, AircraftSimulationError, AircraftSnapshot,
};
use model::{AircraftModel, ModelLoadError, load_aircraft_model};
use replay::{AircraftModelPhysicsFingerprint, AircraftSnapshotHash};
use sim_core::{
    AeroEnvironment, AeroEnvironmentError, DEFAULT_PHYSICS_HZ, PilotInput, RigidBodyState,
    SimulationConfigError,
};
use sim_math::{Orientation, Vec3};
use std::{
    hint::black_box,
    path::PathBuf,
    time::{Duration, Instant},
};
use thiserror::Error;

const DEFAULT_MODEL_PATH: &str = "models/acro_electric_01/model.json";
const DEFAULT_WARMUP_STEPS: usize = 5_000;
const DEFAULT_MEASURED_STEPS: usize = 50_000;

#[derive(Debug, Clone)]
pub(crate) struct AircraftBenchmarkOptions {
    model_path: PathBuf,
    warmup_steps: usize,
    measured_steps: usize,
    physics_hz: u32,
}

impl AircraftBenchmarkOptions {
    pub(crate) fn parse(
        mut arguments: impl Iterator<Item = String>,
    ) -> Result<Self, AircraftBenchmarkError> {
        let mut options = Self {
            model_path: PathBuf::from(DEFAULT_MODEL_PATH),
            warmup_steps: DEFAULT_WARMUP_STEPS,
            measured_steps: DEFAULT_MEASURED_STEPS,
            physics_hz: DEFAULT_PHYSICS_HZ,
        };
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--model" => {
                    options.model_path = PathBuf::from(
                        arguments
                            .next()
                            .ok_or(AircraftBenchmarkError::MissingValue("--model"))?,
                    );
                }
                "--warmup-steps" => {
                    options.warmup_steps = parse_number("--warmup-steps", arguments.next())?;
                }
                "--steps" => {
                    options.measured_steps = parse_number("--steps", arguments.next())?;
                }
                "--physics-hz" => {
                    options.physics_hz = parse_number("--physics-hz", arguments.next())?;
                }
                "--help" | "-h" => {
                    super::print_usage();
                    std::process::exit(0);
                }
                _ => return Err(AircraftBenchmarkError::UnknownArgument(argument)),
            }
        }
        if options.measured_steps == 0 {
            return Err(AircraftBenchmarkError::ZeroMeasuredSteps);
        }
        if options.physics_hz == 0 {
            return Err(AircraftBenchmarkError::ZeroPhysicsRate);
        }
        Ok(options)
    }
}

#[derive(Debug, Error)]
pub(crate) enum AircraftBenchmarkError {
    #[error("missing value for benchmark option {0}")]
    MissingValue(&'static str),
    #[error("invalid numeric value for benchmark option {0}")]
    InvalidNumber(&'static str),
    #[error("unknown aircraft benchmark argument: {0}")]
    UnknownArgument(String),
    #[error("aircraft benchmark requires --steps greater than zero")]
    ZeroMeasuredSteps,
    #[error("aircraft benchmark requires --physics-hz greater than zero")]
    ZeroPhysicsRate,
    #[error("failed to load benchmark model from {path}: {source}")]
    ModelLoad {
        path: PathBuf,
        #[source]
        source: ModelLoadError,
    },
    #[error("failed to configure benchmark atmosphere: {0}")]
    AeroEnvironment(#[from] AeroEnvironmentError),
    #[error("failed to configure benchmark physics: {0}")]
    SimulationConfig(#[from] SimulationConfigError),
    #[error("failed to initialize benchmark aircraft: {0}")]
    AircraftSimulation(#[from] AircraftSimulationError),
    #[error("cannot calculate benchmark statistics from zero timing samples")]
    EmptySamples,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PerformanceClassification {
    Pass,
    Marginal,
    Fail,
}

impl PerformanceClassification {
    pub(crate) const fn label(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Marginal => "MARGINAL",
            Self::Fail => "FAIL",
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct TimingStatistics {
    pub(crate) mean_us: f64,
    pub(crate) p50_us: f64,
    pub(crate) p95_us: f64,
    pub(crate) p99_us: f64,
    pub(crate) max_us: f64,
    pub(crate) steps_per_second: f64,
    pub(crate) mean_budget_utilization_percent: f64,
    pub(crate) p99_budget_utilization_percent: f64,
    pub(crate) max_budget_utilization_percent: f64,
    pub(crate) classification: PerformanceClassification,
}

pub(crate) struct AircraftBenchmarkResult {
    pub(crate) model_id: String,
    pub(crate) model_fingerprint: String,
    pub(crate) physics_hz: u32,
    pub(crate) physics_dt_s: f64,
    pub(crate) physics_budget_us: f64,
    pub(crate) warmup_steps: usize,
    pub(crate) measured_steps: usize,
    pub(crate) statistics: TimingStatistics,
    pub(crate) final_snapshot_hash: String,
}

struct BenchmarkExecution {
    samples: Vec<Duration>,
    final_snapshot: AircraftSnapshot,
}

pub(crate) fn run_aircraft_benchmark(
    options: AircraftBenchmarkOptions,
) -> Result<(), AircraftBenchmarkError> {
    let model = load_aircraft_model(&options.model_path).map_err(|source| {
        AircraftBenchmarkError::ModelLoad {
            path: options.model_path.clone(),
            source,
        }
    })?;
    let result = measure_aircraft_model(
        model,
        options.physics_hz,
        options.warmup_steps,
        options.measured_steps,
    )?;

    println!("RC Simulation Engine");
    println!("mode: aircraft-benchmark");
    println!("model_id: {}", result.model_id);
    println!("model_physics_fingerprint: {}", result.model_fingerprint);
    println!("physics_hz: {}", result.physics_hz);
    println!("physics_dt_s: {:.12}", result.physics_dt_s);
    println!("physics_budget_us: {:.3}", result.physics_budget_us);
    println!("warmup_steps: {}", result.warmup_steps);
    println!("measured_steps: {}", result.measured_steps);
    println!("mean_us: {:.6}", result.statistics.mean_us);
    println!("p50_us: {:.6}", result.statistics.p50_us);
    println!("p95_us: {:.6}", result.statistics.p95_us);
    println!("p99_us: {:.6}", result.statistics.p99_us);
    println!("max_us: {:.6}", result.statistics.max_us);
    println!(
        "steps_per_second: {:.3}",
        result.statistics.steps_per_second
    );
    println!(
        "mean_budget_utilization_percent: {:.6}",
        result.statistics.mean_budget_utilization_percent
    );
    println!(
        "p99_budget_utilization_percent: {:.6}",
        result.statistics.p99_budget_utilization_percent
    );
    println!(
        "max_budget_utilization_percent: {:.6}",
        result.statistics.max_budget_utilization_percent
    );
    println!(
        "classification: {}",
        result.statistics.classification.label()
    );
    println!("final_snapshot_hash: {}", result.final_snapshot_hash);
    Ok(())
}

pub(crate) fn measure_aircraft_model(
    model: AircraftModel,
    physics_hz: u32,
    warmup_steps: usize,
    measured_steps: usize,
) -> Result<AircraftBenchmarkResult, AircraftBenchmarkError> {
    let model_id = model.model_id().to_owned();
    let model_fingerprint =
        AircraftModelPhysicsFingerprint::from_model_fingerprint(model.physics_fingerprint())
            .to_hex();
    let environment = AeroEnvironment::new(1.225, Vec3::zeros())?;
    let config = AircraftSimulationConfig::from_physics_hz(physics_hz, environment)?;
    let physics_dt_s = config.dt_s();
    let simulation = AircraftSimulation::new(model, config, benchmark_initial_state())?;
    let execution = execute_benchmark(
        simulation,
        benchmark_input(),
        warmup_steps,
        measured_steps,
        |simulation, input| {
            let started = Instant::now();
            let snapshot = simulation.step(black_box(input));
            let elapsed = started.elapsed();
            (black_box(snapshot), elapsed)
        },
    )?;
    let physics_budget = Duration::from_secs_f64(physics_dt_s);
    let statistics = timing_statistics(&execution.samples, physics_budget)?;
    Ok(AircraftBenchmarkResult {
        model_id,
        model_fingerprint,
        physics_hz,
        physics_dt_s,
        physics_budget_us: physics_budget.as_secs_f64() * 1.0e6,
        warmup_steps,
        measured_steps,
        statistics,
        final_snapshot_hash: AircraftSnapshotHash::from_snapshot(&execution.final_snapshot)
            .to_hex(),
    })
}

fn execute_benchmark<F>(
    mut simulation: AircraftSimulation,
    input: PilotInput,
    warmup_steps: usize,
    measured_steps: usize,
    mut measure_step: F,
) -> Result<BenchmarkExecution, AircraftBenchmarkError>
where
    F: FnMut(&mut AircraftSimulation, &PilotInput) -> (AircraftSnapshot, Duration),
{
    if measured_steps == 0 {
        return Err(AircraftBenchmarkError::ZeroMeasuredSteps);
    }
    for _ in 0..warmup_steps {
        black_box(simulation.step(black_box(&input)));
    }

    let mut samples = Vec::with_capacity(measured_steps);
    let mut final_snapshot = None;
    for _ in 0..measured_steps {
        let (snapshot, elapsed) = measure_step(&mut simulation, &input);
        samples.push(elapsed);
        final_snapshot = Some(snapshot);
    }
    Ok(BenchmarkExecution {
        samples,
        final_snapshot: final_snapshot.expect("positive measured step count is checked above"),
    })
}

fn timing_statistics(
    samples: &[Duration],
    physics_budget: Duration,
) -> Result<TimingStatistics, AircraftBenchmarkError> {
    if samples.is_empty() {
        return Err(AircraftBenchmarkError::EmptySamples);
    }
    let mut sorted = samples.to_vec();
    sorted.sort_unstable();
    let total_s = samples.iter().map(Duration::as_secs_f64).sum::<f64>();
    let mean_us = total_s * 1.0e6 / samples.len() as f64;
    let p50_us = nearest_rank(&sorted, 0.50).as_secs_f64() * 1.0e6;
    let p95_us = nearest_rank(&sorted, 0.95).as_secs_f64() * 1.0e6;
    let p99_us = nearest_rank(&sorted, 0.99).as_secs_f64() * 1.0e6;
    let max_us = sorted
        .last()
        .expect("nonempty samples checked above")
        .as_secs_f64()
        * 1.0e6;
    let budget_us = physics_budget.as_secs_f64() * 1.0e6;
    let classification = classify(mean_us, p99_us, budget_us);
    Ok(TimingStatistics {
        mean_us,
        p50_us,
        p95_us,
        p99_us,
        max_us,
        steps_per_second: if total_s > 0.0 {
            samples.len() as f64 / total_s
        } else {
            f64::INFINITY
        },
        mean_budget_utilization_percent: mean_us / budget_us * 100.0,
        p99_budget_utilization_percent: p99_us / budget_us * 100.0,
        max_budget_utilization_percent: max_us / budget_us * 100.0,
        classification,
    })
}

fn nearest_rank(sorted_samples: &[Duration], percentile: f64) -> Duration {
    debug_assert!(!sorted_samples.is_empty());
    debug_assert!((0.0..=1.0).contains(&percentile));
    let rank = (percentile * sorted_samples.len() as f64).ceil() as usize;
    sorted_samples[rank.clamp(1, sorted_samples.len()) - 1]
}

fn classify(mean_us: f64, p99_us: f64, budget_us: f64) -> PerformanceClassification {
    if mean_us >= budget_us {
        PerformanceClassification::Fail
    } else if p99_us >= budget_us {
        PerformanceClassification::Marginal
    } else {
        PerformanceClassification::Pass
    }
}

fn benchmark_initial_state() -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(18.0, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn benchmark_input() -> PilotInput {
    PilotInput::new(0.0, 0.0, 0.0, 0.55)
}

fn parse_number<T>(flag: &'static str, value: Option<String>) -> Result<T, AircraftBenchmarkError>
where
    T: std::str::FromStr,
{
    value
        .ok_or(AircraftBenchmarkError::MissingValue(flag))?
        .parse()
        .map_err(|_| AircraftBenchmarkError::InvalidNumber(flag))
}

#[cfg(test)]
mod tests {
    use super::*;
    use model::load_aircraft_model;

    fn simulation() -> AircraftSimulation {
        let model_path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../models/acro_electric_01/model.json");
        let model = load_aircraft_model(model_path).unwrap();
        let config = AircraftSimulationConfig::from_physics_hz(
            DEFAULT_PHYSICS_HZ,
            AeroEnvironment::new(1.225, Vec3::zeros()).unwrap(),
        )
        .unwrap();
        AircraftSimulation::new(model, config, benchmark_initial_state()).unwrap()
    }

    #[test]
    fn nearest_rank_percentiles_support_sorted_unsorted_and_single_samples() {
        let unsorted = [1, 100, 2, 3, 4, 5, 6, 7, 8, 9].map(Duration::from_micros);
        let statistics = timing_statistics(&unsorted, Duration::from_micros(2_000)).unwrap();
        assert_eq!(statistics.p50_us, 5.0);
        assert_eq!(statistics.p95_us, 100.0);
        assert_eq!(statistics.p99_us, 100.0);
        let sorted = [1, 2, 3, 4, 5, 6, 7, 8, 9, 100].map(Duration::from_micros);
        let sorted_statistics = timing_statistics(&sorted, Duration::from_micros(2_000)).unwrap();
        assert_eq!(statistics.p50_us, sorted_statistics.p50_us);
        assert_eq!(statistics.p95_us, sorted_statistics.p95_us);
        assert_eq!(statistics.p99_us, sorted_statistics.p99_us);

        let single =
            timing_statistics(&[Duration::from_micros(42)], Duration::from_micros(2_000)).unwrap();
        assert_eq!(single.p50_us, 42.0);
        assert_eq!(single.p95_us, 42.0);
        assert_eq!(single.p99_us, 42.0);
    }

    #[test]
    fn warmup_is_excluded_and_measured_sample_count_is_exact() {
        let execution = execute_benchmark(
            simulation(),
            benchmark_input(),
            7,
            3,
            |simulation, input| {
                let snapshot = simulation.step(input);
                (snapshot, Duration::from_micros(snapshot.step_index()))
            },
        )
        .unwrap();
        assert_eq!(execution.samples.len(), 3);
        assert_eq!(execution.samples[0], Duration::from_micros(8));
        assert_eq!(execution.final_snapshot.step_index(), 10);
    }

    #[test]
    fn zero_steps_invalid_numbers_and_zero_physics_rate_are_errors() {
        assert!(matches!(
            AircraftBenchmarkOptions::parse(["--steps".to_owned(), "0".to_owned()].into_iter()),
            Err(AircraftBenchmarkError::ZeroMeasuredSteps)
        ));
        assert!(matches!(
            AircraftBenchmarkOptions::parse(
                ["--warmup-steps".to_owned(), "bad".to_owned()].into_iter()
            ),
            Err(AircraftBenchmarkError::InvalidNumber("--warmup-steps"))
        ));
        assert!(matches!(
            AircraftBenchmarkOptions::parse(
                ["--physics-hz".to_owned(), "0".to_owned()].into_iter()
            ),
            Err(AircraftBenchmarkError::ZeroPhysicsRate)
        ));
        assert!(matches!(
            execute_benchmark(
                simulation(),
                benchmark_input(),
                0,
                0,
                |simulation, input| { (simulation.step(input), Duration::ZERO) }
            ),
            Err(AircraftBenchmarkError::ZeroMeasuredSteps)
        ));
    }

    #[test]
    fn budget_utilization_and_all_classifications_are_explicit() {
        let statistics = timing_statistics(
            &[Duration::from_micros(1_000), Duration::from_micros(2_000)],
            Duration::from_micros(2_000),
        )
        .unwrap();
        assert_eq!(statistics.mean_budget_utilization_percent, 75.0);
        assert_eq!(statistics.p99_budget_utilization_percent, 100.0);
        assert_eq!(statistics.max_budget_utilization_percent, 100.0);
        assert_eq!(
            classify(1_000.0, 1_999.0, 2_000.0),
            PerformanceClassification::Pass
        );
        assert_eq!(
            classify(1_000.0, 2_000.0, 2_000.0),
            PerformanceClassification::Marginal
        );
        assert_eq!(
            classify(2_000.0, 2_000.0, 2_000.0),
            PerformanceClassification::Fail
        );
    }

    #[test]
    fn timed_and_untimed_runs_have_the_same_final_snapshot_hash() {
        let warmup_steps = 11;
        let measured_steps = 100;
        let execution = execute_benchmark(
            simulation(),
            benchmark_input(),
            warmup_steps,
            measured_steps,
            |simulation, input| {
                let started = Instant::now();
                let snapshot = simulation.step(input);
                (snapshot, started.elapsed())
            },
        )
        .unwrap();
        let mut untimed = simulation();
        let mut final_snapshot = None;
        for _ in 0..(warmup_steps + measured_steps) {
            final_snapshot = Some(untimed.step(&benchmark_input()));
        }
        assert_eq!(
            AircraftSnapshotHash::from_snapshot(&execution.final_snapshot),
            AircraftSnapshotHash::from_snapshot(&final_snapshot.unwrap())
        );
    }

    #[test]
    fn benchmark_production_module_has_no_graphics_hardware_or_serialization_path() {
        let production_source = include_str!("benchmark_app.rs")
            .split("#[cfg(test)]")
            .next()
            .unwrap();
        for forbidden in [
            "winit::",
            "wgpu::",
            "renderer::",
            "Gilrs",
            "serde::",
            "serde_json::",
        ] {
            assert!(!production_source.contains(forbidden), "found {forbidden}");
        }
    }

    #[test]
    fn missing_and_malformed_models_return_contextual_load_errors() {
        for model_path in [
            PathBuf::from("missing-p2-model.json"),
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("Cargo.toml"),
        ] {
            let result = run_aircraft_benchmark(AircraftBenchmarkOptions {
                model_path: model_path.clone(),
                warmup_steps: 0,
                measured_steps: 1,
                physics_hz: DEFAULT_PHYSICS_HZ,
            });
            assert!(matches!(
                result,
                Err(AircraftBenchmarkError::ModelLoad { path, .. }) if path == model_path
            ));
        }
    }
}
