#![forbid(unsafe_code)]

use std::{
    collections::HashSet,
    fs,
    io::{Read, Write},
    path::{Path, PathBuf},
    process::{Command, ExitStatus, Stdio},
    thread,
    time::{Duration, Instant},
};

use model::{
    MetadataBuilder, XfoilCampaignCoverageRequest, XfoilPolarImportError, parse_xfoil_polar,
};
use serde::{Deserialize, Serialize};
use thiserror::Error;

const MANIFEST_SCHEMA_VERSION: u32 = 1;
const REPORT_SCHEMA_VERSION: u32 = 1;
const GENERATED_BY: &str = "rcsim-app xfoil run-campaign";
const DEFAULT_TIMEOUT_SECONDS: u64 = 60;
const EXECUTION_JSON: &str = "xfoil_execution.json";
const EXECUTION_MARKDOWN: &str = "xfoil_execution.md";
const VALIDATION_MANIFEST: &str = "xfoil_validation_manifest.json";
const POLARS_DIRECTORY: &str = "polars";
const MAX_CAPTURED_STREAM_BYTES: usize = 64 * 1024;

#[derive(Debug, Clone)]
pub struct XfoilRunnerOptions {
    manifest_path: PathBuf,
    xfoil_executable: PathBuf,
    output_dir: PathBuf,
    timeout: Duration,
}

impl XfoilRunnerOptions {
    pub fn parse(mut args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut manifest_path = None;
        let mut xfoil_executable = None;
        let mut output_dir = None;
        let mut timeout_seconds = None;
        while let Some(argument) = args.next() {
            let value = match argument.as_str() {
                "--manifest" | "--xfoil-executable" | "--output-dir" | "--timeout-seconds" => args
                    .next()
                    .ok_or_else(|| format!("missing value for {argument}"))?,
                _ => return Err(format!("unknown XFOIL runner argument: {argument}")),
            };
            match argument.as_str() {
                "--manifest" => set_once(&mut manifest_path, PathBuf::from(value), &argument)?,
                "--xfoil-executable" => {
                    set_once(&mut xfoil_executable, PathBuf::from(value), &argument)?
                }
                "--output-dir" => set_once(&mut output_dir, PathBuf::from(value), &argument)?,
                "--timeout-seconds" => {
                    let parsed = value
                        .parse::<u64>()
                        .map_err(|_| "invalid value for --timeout-seconds".to_owned())?;
                    if parsed == 0 {
                        return Err("--timeout-seconds must be greater than zero".to_owned());
                    }
                    set_once(&mut timeout_seconds, parsed, &argument)?;
                }
                _ => unreachable!(),
            }
        }
        Ok(Self {
            manifest_path: manifest_path.ok_or_else(|| "--manifest PATH is required".to_owned())?,
            xfoil_executable: xfoil_executable
                .ok_or_else(|| "--xfoil-executable PATH is required".to_owned())?,
            output_dir: output_dir.ok_or_else(|| "--output-dir PATH is required".to_owned())?,
            timeout: Duration::from_secs(timeout_seconds.unwrap_or(DEFAULT_TIMEOUT_SECONDS)),
        })
    }
}

fn set_once<T>(slot: &mut Option<T>, value: T, flag: &str) -> Result<(), String> {
    if slot.is_some() {
        Err(format!("{flag} may be supplied only once"))
    } else {
        *slot = Some(value);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum XfoilRunnerStatus {
    Completed,
    Incomplete,
}

#[derive(Debug, Error)]
pub enum XfoilRunnerError {
    #[error("failed to read XFOIL execution manifest {path:?}: {source}")]
    ManifestRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to deserialize XFOIL execution manifest {path:?}: {source}")]
    ManifestDeserialize {
        path: PathBuf,
        #[source]
        source: serde_json::Error,
    },
    #[error(
        "unsupported XFOIL execution manifest schema version {found}; supported version is {supported}"
    )]
    UnsupportedManifestSchemaVersion { found: u32, supported: u32 },
    #[error("invalid XFOIL execution manifest: {reason}")]
    ManifestValidation { reason: String },
    #[error("invalid coverage request in XFOIL execution manifest: {source}")]
    CoverageRequest {
        #[source]
        source: model::XfoilEvidenceCampaignError,
    },
    #[error("invalid solver metadata for run {index}, dataset {dataset_id:?}: {source}")]
    Metadata {
        index: usize,
        dataset_id: String,
        #[source]
        source: XfoilPolarImportError,
    },
    #[error("failed to read airfoil file {path:?}: {source}")]
    AirfoilRead {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("airfoil file {path:?} is empty or whitespace-only")]
    EmptyAirfoil { path: PathBuf },
    #[error("failed to resolve explicit XFOIL executable path {path:?}: {source}")]
    ExecutablePath {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to create output directory {path:?}: {source}")]
    CreateOutputDirectory {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to prepare deterministic output {path:?}: {source}")]
    PrepareOutput {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to create staging directory {path:?}: {source}")]
    CreateStagingDirectory {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to write staging input {path:?}: {source}")]
    WriteStagingInput {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to start XFOIL for run {index}, dataset {dataset_id:?}: {source}")]
    StartProcess {
        index: usize,
        dataset_id: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to write XFOIL stdin for run {index}, dataset {dataset_id:?}: {source}")]
    ProcessStdin {
        index: usize,
        dataset_id: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed while waiting for XFOIL run {index}, dataset {dataset_id:?}: {source}")]
    WaitProcess {
        index: usize,
        dataset_id: String,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to capture {stream} for XFOIL run {index}, dataset {dataset_id:?}")]
    CaptureProcessOutput {
        index: usize,
        dataset_id: String,
        stream: &'static str,
    },
    #[error("failed to read polar output {path:?}: {source}")]
    ReadPolarOutput {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to write final polar output {path:?}: {source}")]
    WritePolarOutput {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to clean staging directory {path:?}: {source}")]
    CleanupStaging {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    #[error("failed to serialize {artifact}: {source}")]
    SerializeArtifact {
        artifact: &'static str,
        #[source]
        source: serde_json::Error,
    },
    #[error("failed to write artifact {path:?}: {source}")]
    WriteArtifact {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExecutionManifest {
    schema_version: u32,
    campaign_id: String,
    airfoil_file: String,
    runs: Vec<RunSpec>,
    coverage_request: CoverageRequestSpec,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
struct RunSpec {
    dataset_id: String,
    reynolds: f64,
    mach: f64,
    alpha_start_deg: f64,
    alpha_end_deg: f64,
    alpha_step_deg: f64,
    maximum_iterations: u32,
    ncrit: f64,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct CoverageRequestSpec {
    required_reynolds_min: f64,
    required_reynolds_max: f64,
    required_alpha_min_rad: f64,
    required_alpha_max_rad: f64,
    require_converged: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum ExecutionStatus {
    CompletedParseable,
    ProcessFailed,
    TimedOut,
    MissingPolarOutput,
    UnparseablePolarOutput,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum CampaignExecutionStatus {
    Completed,
    Incomplete,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct ExecutionReport {
    schema_version: u32,
    generated_by: String,
    campaign_id: String,
    airfoil_file: String,
    run_count: usize,
    completed_run_count: usize,
    status: CampaignExecutionStatus,
    runs: Vec<RunReport>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct RunReport {
    index: usize,
    dataset_id: String,
    reynolds: f64,
    mach: f64,
    alpha_start_deg: f64,
    alpha_end_deg: f64,
    alpha_step_deg: f64,
    maximum_iterations: u32,
    ncrit: f64,
    polar_file: String,
    execution_status: ExecutionStatus,
    process_exit_code: Option<i32>,
    parsed_sample_count: Option<usize>,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct ValidationManifest<'a> {
    schema_version: u32,
    campaign_id: &'a str,
    datasets: Vec<ValidationDataset<'a>>,
    coverage_request: CoverageRequestSpec,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct ValidationDataset<'a> {
    polar_file: String,
    dataset_id: &'a str,
    method_id: String,
    convergence_status: &'static str,
    source_ids: Vec<String>,
    notes: &'static str,
    reynolds: f64,
    mach: f64,
    solver_name: &'static str,
    solver_version: Option<&'static str>,
    command_or_config: String,
    transition_assumptions: String,
    ncrit: f64,
    forced_transition_upper_x_over_c: Option<f64>,
    forced_transition_lower_x_over_c: Option<f64>,
}

struct ProcessOutcome {
    exit_status: Option<ExitStatus>,
    timed_out: bool,
    _stdout: Vec<u8>,
    _stderr: Vec<u8>,
}

pub fn run_xfoil_campaign(
    options: XfoilRunnerOptions,
) -> Result<XfoilRunnerStatus, XfoilRunnerError> {
    let manifest_text = fs::read_to_string(&options.manifest_path).map_err(|source| {
        XfoilRunnerError::ManifestRead {
            path: options.manifest_path.clone(),
            source,
        }
    })?;
    let manifest: ExecutionManifest = serde_json::from_str(&manifest_text).map_err(|source| {
        XfoilRunnerError::ManifestDeserialize {
            path: options.manifest_path.clone(),
            source,
        }
    })?;
    validate_manifest(&manifest)?;

    let manifest_directory = options
        .manifest_path
        .parent()
        .unwrap_or_else(|| Path::new(""));
    let supplied_airfoil_path = PathBuf::from(&manifest.airfoil_file);
    let airfoil_path = if supplied_airfoil_path.is_absolute() {
        supplied_airfoil_path
    } else {
        manifest_directory.join(supplied_airfoil_path)
    };
    let airfoil_text =
        fs::read_to_string(&airfoil_path).map_err(|source| XfoilRunnerError::AirfoilRead {
            path: airfoil_path.clone(),
            source,
        })?;
    if airfoil_text.trim().is_empty() {
        return Err(XfoilRunnerError::EmptyAirfoil { path: airfoil_path });
    }

    let executable = resolve_executable_path(&options.xfoil_executable)?;
    prepare_output_directory(&options.output_dir, manifest.runs.len())?;
    let staging_root = options
        .output_dir
        .join(format!(".xfoil-staging-{}", std::process::id()));
    if staging_root.exists() {
        return Err(XfoilRunnerError::CreateStagingDirectory {
            path: staging_root,
            source: std::io::Error::new(
                std::io::ErrorKind::AlreadyExists,
                "staging directory already exists",
            ),
        });
    }
    fs::create_dir(&staging_root).map_err(|source| XfoilRunnerError::CreateStagingDirectory {
        path: staging_root.clone(),
        source,
    })?;

    let execution_result = execute_runs(
        &manifest,
        &airfoil_text,
        &executable,
        &options.output_dir,
        &staging_root,
        options.timeout,
    );
    fs::remove_dir_all(&staging_root).map_err(|source| XfoilRunnerError::CleanupStaging {
        path: staging_root,
        source,
    })?;
    let runs = execution_result?;

    let completed_run_count = runs
        .iter()
        .filter(|run| run.execution_status == ExecutionStatus::CompletedParseable)
        .count();
    let completed = completed_run_count == manifest.runs.len();
    let report = ExecutionReport {
        schema_version: REPORT_SCHEMA_VERSION,
        generated_by: GENERATED_BY.to_owned(),
        campaign_id: manifest.campaign_id.clone(),
        airfoil_file: manifest.airfoil_file.clone(),
        run_count: manifest.runs.len(),
        completed_run_count,
        status: if completed {
            CampaignExecutionStatus::Completed
        } else {
            CampaignExecutionStatus::Incomplete
        },
        runs,
    };
    write_execution_artifacts(&options.output_dir, &report)?;
    if completed {
        write_validation_manifest(&options.output_dir, &manifest)?;
        Ok(XfoilRunnerStatus::Completed)
    } else {
        Ok(XfoilRunnerStatus::Incomplete)
    }
}

fn validate_manifest(manifest: &ExecutionManifest) -> Result<(), XfoilRunnerError> {
    if manifest.schema_version != MANIFEST_SCHEMA_VERSION {
        return Err(XfoilRunnerError::UnsupportedManifestSchemaVersion {
            found: manifest.schema_version,
            supported: MANIFEST_SCHEMA_VERSION,
        });
    }
    if manifest.campaign_id.trim().is_empty() {
        return invalid_manifest("campaign_id must not be empty or whitespace-only");
    }
    if manifest.airfoil_file.trim().is_empty() {
        return invalid_manifest("airfoil_file must not be empty or whitespace-only");
    }
    if manifest.runs.is_empty() {
        return invalid_manifest("runs must contain at least one run");
    }
    XfoilCampaignCoverageRequest::new(
        manifest.coverage_request.required_reynolds_min,
        manifest.coverage_request.required_reynolds_max,
        manifest.coverage_request.required_alpha_min_rad,
        manifest.coverage_request.required_alpha_max_rad,
        manifest.coverage_request.require_converged,
    )
    .map_err(|source| XfoilRunnerError::CoverageRequest { source })?;

    let mut ids = HashSet::new();
    for (index, run) in manifest.runs.iter().enumerate() {
        if !is_stable_id(&run.dataset_id) {
            return invalid_manifest(format!(
                "run {index} dataset_id must contain only lowercase ASCII letters, digits, '_' or '-'"
            ));
        }
        if !ids.insert(run.dataset_id.as_str()) {
            return invalid_manifest(format!(
                "duplicate dataset_id {:?} at run {index}",
                run.dataset_id
            ));
        }
        MetadataBuilder::new(run.reynolds, run.mach)
            .ncrit(run.ncrit)
            .build()
            .map_err(|source| XfoilRunnerError::Metadata {
                index,
                dataset_id: run.dataset_id.clone(),
                source,
            })?;
        if index > 0 && run.reynolds <= manifest.runs[index - 1].reynolds {
            return invalid_manifest(format!(
                "run {index} Reynolds node must be strictly greater than the preceding run"
            ));
        }
        if !run.alpha_start_deg.is_finite() || !run.alpha_end_deg.is_finite() {
            return invalid_manifest(format!("run {index} alpha bounds must be finite"));
        }
        if !run.alpha_step_deg.is_finite() || run.alpha_step_deg == 0.0 {
            return invalid_manifest(format!(
                "run {index} alpha_step_deg must be finite and non-zero"
            ));
        }
        if run.alpha_start_deg == run.alpha_end_deg {
            return invalid_manifest(format!(
                "run {index} alpha_start_deg and alpha_end_deg must differ"
            ));
        }
        if (run.alpha_end_deg > run.alpha_start_deg && run.alpha_step_deg < 0.0)
            || (run.alpha_end_deg < run.alpha_start_deg && run.alpha_step_deg > 0.0)
        {
            return invalid_manifest(format!(
                "run {index} alpha_step_deg sign does not match sweep direction"
            ));
        }
        if run.maximum_iterations == 0 {
            return invalid_manifest(format!(
                "run {index} maximum_iterations must be greater than zero"
            ));
        }
    }
    Ok(())
}

fn invalid_manifest<T>(reason: impl Into<String>) -> Result<T, XfoilRunnerError> {
    Err(XfoilRunnerError::ManifestValidation {
        reason: reason.into(),
    })
}

fn is_stable_id(value: &str) -> bool {
    !value.is_empty()
        && value
            .bytes()
            .all(|byte| byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"_-".contains(&byte))
}

fn resolve_executable_path(path: &Path) -> Result<PathBuf, XfoilRunnerError> {
    let explicit_path = if path.is_absolute() {
        path.to_owned()
    } else {
        std::env::current_dir()
            .map_err(|source| XfoilRunnerError::ExecutablePath {
                path: path.to_owned(),
                source,
            })?
            .join(path)
    };
    fs::canonicalize(&explicit_path).map_err(|source| XfoilRunnerError::ExecutablePath {
        path: explicit_path,
        source,
    })
}

fn prepare_output_directory(output_dir: &Path, run_count: usize) -> Result<(), XfoilRunnerError> {
    fs::create_dir_all(output_dir).map_err(|source| XfoilRunnerError::CreateOutputDirectory {
        path: output_dir.to_owned(),
        source,
    })?;
    let polars = output_dir.join(POLARS_DIRECTORY);
    fs::create_dir_all(&polars).map_err(|source| XfoilRunnerError::CreateOutputDirectory {
        path: polars.clone(),
        source,
    })?;
    for path in [
        output_dir.join(EXECUTION_JSON),
        output_dir.join(EXECUTION_MARKDOWN),
        output_dir.join(VALIDATION_MANIFEST),
    ] {
        remove_file_if_present(&path)?;
    }
    for index in 0..run_count {
        remove_file_if_present(&polars.join(polar_filename(index)))?;
    }
    Ok(())
}

fn remove_file_if_present(path: &Path) -> Result<(), XfoilRunnerError> {
    match fs::remove_file(path) {
        Ok(()) => Ok(()),
        Err(source) if source.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(source) => Err(XfoilRunnerError::PrepareOutput {
            path: path.to_owned(),
            source,
        }),
    }
}

fn execute_runs(
    manifest: &ExecutionManifest,
    airfoil_text: &str,
    executable: &Path,
    output_dir: &Path,
    staging_root: &Path,
    timeout: Duration,
) -> Result<Vec<RunReport>, XfoilRunnerError> {
    let mut reports = Vec::with_capacity(manifest.runs.len());
    for (index, run) in manifest.runs.iter().enumerate() {
        let run_directory = staging_root.join(format!("{index:04}"));
        fs::create_dir(&run_directory).map_err(|source| {
            XfoilRunnerError::CreateStagingDirectory {
                path: run_directory.clone(),
                source,
            }
        })?;
        let local_airfoil = run_directory.join("airfoil.dat");
        fs::write(&local_airfoil, airfoil_text.as_bytes()).map_err(|source| {
            XfoilRunnerError::WriteStagingInput {
                path: local_airfoil,
                source,
            }
        })?;
        let script = build_command_script(run);
        let outcome = execute_process(
            executable,
            &run_directory,
            script.as_bytes(),
            index,
            run,
            timeout,
        )?;
        let polar_file = format!("{POLARS_DIRECTORY}/{}", polar_filename(index));
        let local_polar = run_directory.join("polar.out");
        let final_polar = output_dir
            .join(POLARS_DIRECTORY)
            .join(polar_filename(index));

        let (execution_status, parsed_sample_count) = if outcome.timed_out {
            (ExecutionStatus::TimedOut, None)
        } else if !outcome.exit_status.is_some_and(|status| status.success()) {
            (ExecutionStatus::ProcessFailed, None)
        } else if !local_polar.is_file() {
            (ExecutionStatus::MissingPolarOutput, None)
        } else {
            validate_and_copy_polar(&local_polar, &final_polar, run, index)?
        };
        reports.push(RunReport {
            index,
            dataset_id: run.dataset_id.clone(),
            reynolds: run.reynolds,
            mach: run.mach,
            alpha_start_deg: run.alpha_start_deg,
            alpha_end_deg: run.alpha_end_deg,
            alpha_step_deg: run.alpha_step_deg,
            maximum_iterations: run.maximum_iterations,
            ncrit: run.ncrit,
            polar_file,
            execution_status,
            process_exit_code: outcome.exit_status.and_then(|status| status.code()),
            parsed_sample_count,
        });
    }
    Ok(reports)
}

fn execute_process(
    executable: &Path,
    run_directory: &Path,
    script: &[u8],
    index: usize,
    run: &RunSpec,
    timeout: Duration,
) -> Result<ProcessOutcome, XfoilRunnerError> {
    let mut child = Command::new(executable)
        .current_dir(run_directory)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|source| XfoilRunnerError::StartProcess {
            index,
            dataset_id: run.dataset_id.clone(),
            source,
        })?;
    let stdout = child.stdout.take().expect("piped stdout must exist");
    let stderr = child.stderr.take().expect("piped stderr must exist");
    let stdout_reader = thread::spawn(move || read_stream(stdout));
    let stderr_reader = thread::spawn(move || read_stream(stderr));
    let stdin_result = child
        .stdin
        .take()
        .expect("piped stdin must exist")
        .write_all(script);
    if let Err(source) = stdin_result {
        let _ = child.kill();
        let _ = child.wait();
        let _ = stdout_reader.join();
        let _ = stderr_reader.join();
        return Err(XfoilRunnerError::ProcessStdin {
            index,
            dataset_id: run.dataset_id.clone(),
            source,
        });
    }

    let started = Instant::now();
    let (exit_status, timed_out) = loop {
        let status = match child.try_wait() {
            Ok(status) => status,
            Err(source) => {
                let _ = child.kill();
                let _ = child.wait();
                let _ = stdout_reader.join();
                let _ = stderr_reader.join();
                return Err(XfoilRunnerError::WaitProcess {
                    index,
                    dataset_id: run.dataset_id.clone(),
                    source,
                });
            }
        };
        if let Some(status) = status {
            break (Some(status), false);
        }
        if started.elapsed() >= timeout {
            child
                .kill()
                .map_err(|source| XfoilRunnerError::WaitProcess {
                    index,
                    dataset_id: run.dataset_id.clone(),
                    source,
                })?;
            let status = child
                .wait()
                .map_err(|source| XfoilRunnerError::WaitProcess {
                    index,
                    dataset_id: run.dataset_id.clone(),
                    source,
                })?;
            break (Some(status), true);
        }
        thread::sleep(Duration::from_millis(10));
    };
    let stdout = join_stream(stdout_reader, index, run, "stdout")?;
    let stderr = join_stream(stderr_reader, index, run, "stderr")?;
    Ok(ProcessOutcome {
        exit_status,
        timed_out,
        _stdout: stdout,
        _stderr: stderr,
    })
}

fn read_stream(mut stream: impl Read) -> std::io::Result<Vec<u8>> {
    let mut bytes = Vec::new();
    let mut buffer = [0_u8; 8 * 1024];
    loop {
        let count = stream.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        let remaining = MAX_CAPTURED_STREAM_BYTES.saturating_sub(bytes.len());
        bytes.extend_from_slice(&buffer[..count.min(remaining)]);
    }
    Ok(bytes)
}

fn join_stream(
    handle: thread::JoinHandle<std::io::Result<Vec<u8>>>,
    index: usize,
    run: &RunSpec,
    stream: &'static str,
) -> Result<Vec<u8>, XfoilRunnerError> {
    handle
        .join()
        .ok()
        .and_then(Result::ok)
        .ok_or_else(|| XfoilRunnerError::CaptureProcessOutput {
            index,
            dataset_id: run.dataset_id.clone(),
            stream,
        })
}

fn validate_and_copy_polar(
    local_polar: &Path,
    final_polar: &Path,
    run: &RunSpec,
    index: usize,
) -> Result<(ExecutionStatus, Option<usize>), XfoilRunnerError> {
    let bytes = fs::read(local_polar).map_err(|source| XfoilRunnerError::ReadPolarOutput {
        path: local_polar.to_owned(),
        source,
    })?;
    let Ok(text) = std::str::from_utf8(&bytes) else {
        return Ok((ExecutionStatus::UnparseablePolarOutput, None));
    };
    if text.trim().is_empty() {
        return Ok((ExecutionStatus::UnparseablePolarOutput, None));
    }
    let metadata = MetadataBuilder::new(run.reynolds, run.mach)
        .solver_name("XFOIL")
        .command_or_config(build_command_script(run))
        .transition_assumptions(transition_assumptions(run.ncrit))
        .ncrit(run.ncrit)
        .build()
        .map_err(|source| XfoilRunnerError::Metadata {
            index,
            dataset_id: run.dataset_id.clone(),
            source,
        })?;
    let Ok(import) = parse_xfoil_polar(text, metadata) else {
        return Ok((ExecutionStatus::UnparseablePolarOutput, None));
    };
    fs::write(final_polar, &bytes).map_err(|source| XfoilRunnerError::WritePolarOutput {
        path: final_polar.to_owned(),
        source,
    })?;
    Ok((
        ExecutionStatus::CompletedParseable,
        Some(import.sample_count()),
    ))
}

fn build_command_script(run: &RunSpec) -> String {
    format!(
        "LOAD airfoil.dat\nPANE\nOPER\nVISC {}\nMACH {}\nITER {}\nVPAR\nN {}\n\nPACC\npolar.out\n\nASEQ {} {} {}\nPACC\nQUIT\n",
        format_number(run.reynolds),
        format_number(run.mach),
        run.maximum_iterations,
        format_number(run.ncrit),
        format_number(run.alpha_start_deg),
        format_number(run.alpha_end_deg),
        format_number(run.alpha_step_deg),
    )
}

fn format_number(value: f64) -> String {
    format!("{value:.17e}")
}

fn transition_assumptions(ncrit: f64) -> String {
    format!(
        "Free transition with Ncrit {} configured through the XFOIL VPAR menu; no forced transition was requested.",
        format_number(ncrit)
    )
}

fn polar_filename(index: usize) -> String {
    format!("{index:04}.polar")
}

fn write_execution_artifacts(
    output_dir: &Path,
    report: &ExecutionReport,
) -> Result<(), XfoilRunnerError> {
    let json = serde_json::to_vec_pretty(report).map_err(|source| {
        XfoilRunnerError::SerializeArtifact {
            artifact: EXECUTION_JSON,
            source,
        }
    })?;
    let markdown = render_markdown(report).into_bytes();
    write_artifact(output_dir.join(EXECUTION_JSON), &json)?;
    write_artifact(output_dir.join(EXECUTION_MARKDOWN), &markdown)
}

fn write_validation_manifest(
    output_dir: &Path,
    manifest: &ExecutionManifest,
) -> Result<(), XfoilRunnerError> {
    let datasets = manifest
        .runs
        .iter()
        .enumerate()
        .map(|(index, run)| ValidationDataset {
            polar_file: format!("{POLARS_DIRECTORY}/{}", polar_filename(index)),
            dataset_id: &run.dataset_id,
            method_id: format!("xfoil-run-{index:04}"),
            convergence_status: "unresolved",
            source_ids: vec![format!("xfoil-run-{index:04}-input")],
            notes: "Generated by M2.9E; aerodynamic convergence remains unresolved.",
            reynolds: run.reynolds,
            mach: run.mach,
            solver_name: "XFOIL",
            solver_version: None,
            command_or_config: build_command_script(run),
            transition_assumptions: transition_assumptions(run.ncrit),
            ncrit: run.ncrit,
            forced_transition_upper_x_over_c: None,
            forced_transition_lower_x_over_c: None,
        })
        .collect();
    let validation = ValidationManifest {
        schema_version: 1,
        campaign_id: &manifest.campaign_id,
        datasets,
        coverage_request: manifest.coverage_request,
    };
    let bytes = serde_json::to_vec_pretty(&validation).map_err(|source| {
        XfoilRunnerError::SerializeArtifact {
            artifact: VALIDATION_MANIFEST,
            source,
        }
    })?;
    write_artifact(output_dir.join(VALIDATION_MANIFEST), &bytes)
}

fn write_artifact(path: PathBuf, bytes: &[u8]) -> Result<(), XfoilRunnerError> {
    fs::write(&path, bytes).map_err(|source| XfoilRunnerError::WriteArtifact { path, source })
}

fn render_markdown(report: &ExecutionReport) -> String {
    let mut text = format!(
        "# XFOIL campaign execution report\n\n\
         - Campaign ID: `{}`\n\
         - Airfoil file: `{}`\n\
         - Status: **{}**\n\
         - Completed runs: {} of {}\n\n\
         ## Ordered runs\n\n\
         | Index | Dataset | Re | Mach | Alpha Start | Alpha End | Alpha Step | Iterations | Ncrit | Outcome | Exit Code | Parsed Samples | Polar File |\n\
         | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | --- |\n",
        markdown_text(&report.campaign_id),
        markdown_text(&report.airfoil_file),
        match report.status {
            CampaignExecutionStatus::Completed => "Completed",
            CampaignExecutionStatus::Incomplete => "Incomplete",
        },
        report.completed_run_count,
        report.run_count,
    );
    for run in &report.runs {
        text.push_str(&format!(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |\n",
            run.index,
            markdown_text(&run.dataset_id),
            run.reynolds,
            run.mach,
            run.alpha_start_deg,
            run.alpha_end_deg,
            run.alpha_step_deg,
            run.maximum_iterations,
            run.ncrit,
            execution_status_label(&run.execution_status),
            optional_display(run.process_exit_code),
            optional_display(run.parsed_sample_count),
            markdown_text(&run.polar_file),
        ));
    }
    text.push_str(
        "\n> **Scope disclaimer:** Successful XFOIL process execution and parseable polar output do not establish solver convergence, scientific validity, airfoil fidelity, aircraft fidelity, coverage qualification, or runtime readiness.\n",
    );
    text
}

fn execution_status_label(status: &ExecutionStatus) -> &'static str {
    match status {
        ExecutionStatus::CompletedParseable => "completed_parseable",
        ExecutionStatus::ProcessFailed => "process_failed",
        ExecutionStatus::TimedOut => "timed_out",
        ExecutionStatus::MissingPolarOutput => "missing_polar_output",
        ExecutionStatus::UnparseablePolarOutput => "unparseable_polar_output",
    }
}

fn optional_display(value: Option<impl std::fmt::Display>) -> String {
    value.map_or_else(|| "—".to_owned(), |value| value.to_string())
}

fn markdown_text(value: &str) -> String {
    value.replace('|', "\\|").replace(['\r', '\n'], " ")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_spec() -> RunSpec {
        RunSpec {
            dataset_id: "synthetic-run".to_owned(),
            reynolds: 100_000.0,
            mach: 0.03,
            alpha_start_deg: -10.0,
            alpha_end_deg: 15.0,
            alpha_step_deg: 0.5,
            maximum_iterations: 100,
            ncrit: 9.0,
        }
    }

    #[test]
    fn command_script_is_exact_and_byte_deterministic() {
        let expected = "LOAD airfoil.dat\nPANE\nOPER\nVISC 1.00000000000000000e5\nMACH 2.99999999999999989e-2\nITER 100\nVPAR\nN 9.00000000000000000e0\n\nPACC\npolar.out\n\nASEQ -1.00000000000000000e1 1.50000000000000000e1 5.00000000000000000e-1\nPACC\nQUIT\n";
        assert_eq!(
            build_command_script(&run_spec()).as_bytes(),
            expected.as_bytes()
        );
        assert_eq!(
            build_command_script(&run_spec()),
            build_command_script(&run_spec())
        );
    }

    #[test]
    fn manifest_validation_rejects_non_finite_mach() {
        let manifest = ExecutionManifest {
            schema_version: 1,
            campaign_id: "campaign".to_owned(),
            airfoil_file: "airfoil.dat".to_owned(),
            runs: vec![RunSpec {
                mach: f64::NAN,
                ..run_spec()
            }],
            coverage_request: CoverageRequestSpec {
                required_reynolds_min: 100_000.0,
                required_reynolds_max: 200_000.0,
                required_alpha_min_rad: -0.1,
                required_alpha_max_rad: 0.1,
                require_converged: false,
            },
        };
        assert!(matches!(
            validate_manifest(&manifest),
            Err(XfoilRunnerError::Metadata { .. })
        ));
    }
}
