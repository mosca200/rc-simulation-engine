use crate::controller_profile_app::{
    ControllerProfileFileError, format_device_identity, save_controller_profile,
};
use platform::{
    CenteredAxisProfile, CenteredCalibration, Control, ControllerAxes, ControllerProfile,
    GilrsInputBackend, HardwareAxis, InputDeviceInfo, InputError, ProfileAxes, RawControllerState,
    ThrottleAxisProfile, ThrottleCalibration,
};
use std::{
    fmt::Write as _,
    io::{self, Write as _},
    path::PathBuf,
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};
use thiserror::Error;

const MONITOR_REFRESH_PERIOD: Duration = Duration::from_millis(100);
const MONITOR_REFRESH_HZ: u32 = 10;

#[derive(Debug, Clone, PartialEq)]
pub enum ControllerCommand {
    List,
    Monitor(ControllerMonitorOptions),
    Calibrate(ControllerCalibrateOptions),
}

impl ControllerCommand {
    pub fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        match arguments.next().as_deref() {
            Some("list") => {
                if let Some(argument) = arguments.next() {
                    return Err(format!("unknown controller list argument: {argument}"));
                }
                Ok(Self::List)
            }
            Some("monitor") => Ok(Self::Monitor(ControllerMonitorOptions::parse(arguments)?)),
            Some("calibrate") => Ok(Self::Calibrate(ControllerCalibrateOptions::parse(
                arguments,
            )?)),
            Some(command) => Err(format!("unknown controller command: {command}")),
            None => Err(
                "missing controller command; expected `list`, `monitor`, or `calibrate`".to_owned(),
            ),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControllerMonitorOptions {
    sample_limit: Option<u64>,
    duration_seconds: Option<u64>,
    raw: bool,
    device_id: Option<usize>,
}

impl ControllerMonitorOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut options = Self::default();
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--samples" => {
                    options.sample_limit =
                        Some(parse_positive_integer("--samples", arguments.next())?);
                }
                "--duration-seconds" => {
                    options.duration_seconds = Some(parse_positive_integer(
                        "--duration-seconds",
                        arguments.next(),
                    )?);
                }
                "--raw" => options.raw = true,
                "--device-id" => {
                    let value = arguments
                        .next()
                        .ok_or_else(|| "missing value for --device-id".to_owned())?;
                    options.device_id = Some(value.parse::<usize>().map_err(|_| {
                        "invalid value for --device-id: expected a non-negative integer".to_owned()
                    })?);
                }
                _ => return Err(format!("unknown controller monitor argument: {argument}")),
            }
        }
        if options.device_id.is_some() && !options.raw {
            return Err("--device-id is valid only with controller monitor --raw".to_owned());
        }
        Ok(options)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct ControllerCalibrateOptions {
    output_path: PathBuf,
    deadzone: f64,
}

impl ControllerCalibrateOptions {
    fn parse(mut arguments: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut output_path = None;
        let mut deadzone = 0.05;
        while let Some(argument) = arguments.next() {
            match argument.as_str() {
                "--output" => {
                    output_path = Some(PathBuf::from(
                        arguments
                            .next()
                            .ok_or_else(|| "missing value for --output".to_owned())?,
                    ));
                }
                "--deadzone" => {
                    let value = arguments
                        .next()
                        .ok_or_else(|| "missing value for --deadzone".to_owned())?;
                    deadzone = value.parse::<f64>().map_err(|_| {
                        "invalid value for --deadzone: expected a finite value inside [0, 1)"
                            .to_owned()
                    })?;
                    if !deadzone.is_finite() || !(0.0..1.0).contains(&deadzone) {
                        return Err(
                            "invalid value for --deadzone: expected a finite value inside [0, 1)"
                                .to_owned(),
                        );
                    }
                }
                _ => return Err(format!("unknown controller calibrate argument: {argument}")),
            }
        }
        let output_path =
            output_path.ok_or_else(|| "controller calibrate requires --output PATH".to_owned())?;
        Ok(Self {
            output_path,
            deadzone,
        })
    }
}

#[derive(Debug, Error)]
pub enum ControllerAppError {
    #[error(transparent)]
    Input(#[from] InputError),
    #[error(transparent)]
    ProfileFile(#[from] ControllerProfileFileError),
    #[error("controller session id {id} is not connected")]
    UnknownDeviceId { id: usize },
    #[error(
        "raw controller monitor found {devices} devices; choose one explicitly with --device-id ID"
    )]
    RawDeviceSelectionRequired { devices: usize },
    #[error("the explicitly selected controller is no longer connected")]
    SelectedDeviceDisconnected,
    #[error("hardware axis {axis} was not present in the selected controller sample")]
    MissingSampledAxis { axis: HardwareAxis },
    #[error("the selected controller exposes no supported raw hardware axes")]
    NoHardwareAxes,
    #[error("controller calibration console I/O failed: {0}")]
    ConsoleIo(#[from] io::Error),
}

fn parse_positive_integer(flag: &str, value: Option<String>) -> Result<u64, String> {
    let value = value.ok_or_else(|| format!("missing value for {flag}"))?;
    let parsed = value
        .parse::<u64>()
        .map_err(|_| format!("invalid value for {flag}: expected a positive integer"))?;
    if parsed == 0 {
        return Err(format!(
            "invalid value for {flag}: expected a positive integer"
        ));
    }
    Ok(parsed)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct ControllerDeviceView {
    id: usize,
    name: String,
    uuid: String,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
    selected: bool,
}

impl ControllerDeviceView {
    fn from_device(device: &InputDeviceInfo, selected_device_id: Option<usize>) -> Self {
        Self {
            id: device.id(),
            name: device.name().to_owned(),
            uuid: device.uuid().to_owned(),
            vendor_id: device.vendor_id(),
            product_id: device.product_id(),
            selected: selected_device_id == Some(device.id()),
        }
    }
}

pub(crate) fn controller_device_views(
    devices: &[InputDeviceInfo],
    selected_device_id: Option<usize>,
) -> Vec<ControllerDeviceView> {
    devices
        .iter()
        .map(|device| ControllerDeviceView::from_device(device, selected_device_id))
        .collect()
}

pub fn run_controller(command: ControllerCommand) -> Result<(), ControllerAppError> {
    match command {
        ControllerCommand::List => run_controller_list(),
        ControllerCommand::Monitor(options) if options.raw => run_controller_raw_monitor(options),
        ControllerCommand::Monitor(options) => run_controller_monitor(options),
        ControllerCommand::Calibrate(options) => run_controller_calibrate(options),
    }
}

fn run_controller_list() -> Result<(), ControllerAppError> {
    let backend = GilrsInputBackend::new()?;
    let selected_device_id = backend.selected_device_id();
    let devices = backend.devices();
    let views = controller_device_views(&devices, selected_device_id);
    print!("{}", format_controller_list(&views, selected_device_id));
    Ok(())
}

fn run_controller_monitor(options: ControllerMonitorOptions) -> Result<(), ControllerAppError> {
    let mut backend = GilrsInputBackend::new()?;
    let initial_device_id = backend.selected_device_id();
    let initial_devices = backend.devices();
    let initial_views = controller_device_views(&initial_devices, initial_device_id);
    let mut status_tracker = ControllerStatusTracker::new(initial_device_id);

    println!("RC Simulation Engine");
    println!("mode: controller-monitor");
    println!("refresh_hz: {MONITOR_REFRESH_HZ}");
    println!("values: legacy logical axes / pre-calibration");
    println!(
        "{}",
        format_viewer_controller_status(&initial_views, initial_device_id)
    );

    let started = Instant::now();
    let mut next_refresh = started;
    let mut sample_index = 0_u64;
    loop {
        if sample_index > 0
            && options
                .duration_seconds
                .is_some_and(|seconds| started.elapsed() >= Duration::from_secs(seconds))
        {
            break;
        }

        let axes = backend.poll_axes();
        let selected_device_id = backend.selected_device_id();
        if let Some(event) = status_tracker.observe(selected_device_id) {
            let devices = backend.devices();
            let views = controller_device_views(&devices, selected_device_id);
            println!("{}", format_controller_transition(event, &views));
        }

        sample_index += 1;
        println!(
            "{}",
            format_monitor_sample(sample_index, selected_device_id, axes)
        );
        if options
            .sample_limit
            .is_some_and(|limit| sample_index >= limit)
        {
            break;
        }

        next_refresh += MONITOR_REFRESH_PERIOD;
        thread::sleep(next_refresh.saturating_duration_since(Instant::now()));
    }
    Ok(())
}

fn run_controller_raw_monitor(options: ControllerMonitorOptions) -> Result<(), ControllerAppError> {
    let mut backend = GilrsInputBackend::new()?;
    let devices = backend.devices();
    let device = select_raw_monitor_device(&devices, options.device_id)?;
    backend.select_device(&device.identity())?;
    let view = ControllerDeviceView::from_device(device, Some(device.id()));

    println!("RC Simulation Engine");
    println!("mode: controller-monitor-raw");
    println!("refresh_hz: {MONITOR_REFRESH_HZ}");
    println!("device: {}", format_device_identity(&device.identity()));

    let started = Instant::now();
    let mut next_refresh = started;
    let mut sample_index = 0_u64;
    loop {
        if sample_index > 0
            && options
                .duration_seconds
                .is_some_and(|seconds| started.elapsed() >= Duration::from_secs(seconds))
        {
            break;
        }

        let state = poll_selected_raw(&mut backend)?;
        sample_index += 1;
        println!("{}", format_raw_monitor_sample(sample_index, &view, &state));
        if options
            .sample_limit
            .is_some_and(|limit| sample_index >= limit)
        {
            break;
        }

        next_refresh += MONITOR_REFRESH_PERIOD;
        thread::sleep(next_refresh.saturating_duration_since(Instant::now()));
    }
    Ok(())
}

fn select_raw_monitor_device(
    devices: &[InputDeviceInfo],
    requested_id: Option<usize>,
) -> Result<&InputDeviceInfo, ControllerAppError> {
    if devices.is_empty() {
        return Err(InputError::NoDevices.into());
    }
    if let Some(id) = requested_id {
        return devices
            .iter()
            .find(|device| device.id() == id)
            .ok_or(ControllerAppError::UnknownDeviceId { id });
    }
    if devices.len() != 1 {
        return Err(ControllerAppError::RawDeviceSelectionRequired {
            devices: devices.len(),
        });
    }
    Ok(&devices[0])
}

fn run_controller_calibrate(options: ControllerCalibrateOptions) -> Result<(), ControllerAppError> {
    let mut backend = GilrsInputBackend::new()?;
    let devices = backend.devices();
    if devices.is_empty() {
        return Err(InputError::NoDevices.into());
    }

    let views = controller_device_views(&devices, backend.selected_device_id());
    print!(
        "{}",
        format_controller_list(&views, backend.selected_device_id())
    );
    let selected_id = prompt_device_id(&devices)?;
    let selected = devices
        .iter()
        .find(|device| device.id() == selected_id)
        .ok_or(ControllerAppError::UnknownDeviceId { id: selected_id })?;
    let identity = selected.identity();
    backend.select_device(&identity)?;

    println!();
    println!("Selected controller: {}", format_device_identity(&identity));
    println!("No transmitter mode or hardware-axis layout is assumed.");
    let available_axes = discover_hardware_axes(&mut backend, selected)?;
    println!("Detected hardware axes:");
    for axis in &available_axes {
        println!("  {axis}");
    }

    let mut assigned = Vec::with_capacity(4);
    let roll_axis = prompt_axis_assignment(Control::Roll, &available_axes, &mut assigned)?;
    let pitch_axis = prompt_axis_assignment(Control::Pitch, &available_axes, &mut assigned)?;
    let yaw_axis = prompt_axis_assignment(Control::Yaw, &available_axes, &mut assigned)?;
    let throttle_axis = prompt_axis_assignment(Control::Throttle, &available_axes, &mut assigned)?;

    let roll = calibrate_centered_axis(&mut backend, Control::Roll, roll_axis, options.deadzone)?;
    let pitch =
        calibrate_centered_axis(&mut backend, Control::Pitch, pitch_axis, options.deadzone)?;
    let yaw = calibrate_centered_axis(&mut backend, Control::Yaw, yaw_axis, options.deadzone)?;
    let throttle = calibrate_throttle_axis(&mut backend, throttle_axis)?;

    let axes = ProfileAxes::new(roll, pitch, yaw, throttle);
    let profile = ControllerProfile::new(identity, axes)?;
    save_controller_profile(&options.output_path, &profile)?;
    println!();
    println!(
        "Controller profile saved: {}",
        options.output_path.display()
    );
    println!("Profile schema: {}", profile.schema_version());
    println!("Validate it before flight, then pass it to render with --controller-profile.");
    Ok(())
}

fn prompt_device_id(devices: &[InputDeviceInfo]) -> Result<usize, ControllerAppError> {
    loop {
        let response = prompt_line("Select controller session_id: ")?;
        let Ok(id) = response.parse::<usize>() else {
            println!("Enter one of the numeric session_id values shown above.");
            continue;
        };
        if devices.iter().any(|device| device.id() == id) {
            return Ok(id);
        }
        println!("Controller session id {id} is not connected.");
    }
}

fn discover_hardware_axes(
    backend: &mut GilrsInputBackend,
    device: &InputDeviceInfo,
) -> Result<Vec<HardwareAxis>, ControllerAppError> {
    println!();
    println!("Move every stick through its travel to identify changing axes.");
    println!("Press ENTER when you are ready to assign controls.");
    let (receiver, input_thread) = enter_signal();
    let view = ControllerDeviceView::from_device(device, Some(device.id()));
    let mut discovered = Vec::new();
    let mut sample_index = 0_u64;
    loop {
        let state = poll_selected_raw(backend)?;
        for axis in state.axes() {
            if !discovered.contains(&axis) {
                discovered.push(axis);
            }
        }
        discovered.sort_unstable();
        sample_index += 1;
        println!("{}", format_raw_monitor_sample(sample_index, &view, &state));
        match receiver.try_recv() {
            Ok(result) => {
                result?;
                break;
            }
            Err(mpsc::TryRecvError::Empty) => thread::sleep(MONITOR_REFRESH_PERIOD),
            Err(mpsc::TryRecvError::Disconnected) => {
                return Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "calibration input reader stopped unexpectedly",
                )
                .into());
            }
        }
    }
    let _ = input_thread.join();
    if discovered.is_empty() {
        return Err(ControllerAppError::NoHardwareAxes);
    }
    Ok(discovered)
}

fn prompt_axis_assignment(
    control: Control,
    available_axes: &[HardwareAxis],
    assigned: &mut Vec<HardwareAxis>,
) -> Result<HardwareAxis, ControllerAppError> {
    loop {
        let response = prompt_line(&format!(
            "Assign {} hardware axis (type its exact name): ",
            control.label()
        ))?;
        let Ok(axis) = response.parse::<HardwareAxis>() else {
            println!("Unknown hardware axis {response:?}; use one of the names listed above.");
            continue;
        };
        if !available_axes.contains(&axis) {
            println!("Axis {axis} is not exposed by the selected controller.");
            continue;
        }
        if assigned.contains(&axis) {
            println!("Axis {axis} is already assigned; each control requires a distinct axis.");
            continue;
        }
        assigned.push(axis);
        return Ok(axis);
    }
}

fn calibrate_centered_axis(
    backend: &mut GilrsInputBackend,
    control: Control,
    axis: HardwareAxis,
    deadzone: f64,
) -> Result<CenteredAxisProfile, ControllerAppError> {
    let (raw_min, raw_max) = capture_axis_travel(backend, control, axis)?;
    prompt_line(&format!(
        "Release the {} control so it is centered, then press ENTER: ",
        control.label()
    ))?;
    let raw_center = sample_axis_average(backend, axis)?;
    let inverted = prompt_inversion(control)?;
    let calibration =
        CenteredCalibration::new(control, raw_min, raw_center, raw_max, inverted, deadzone)?;
    println!(
        "{}: axis={} min={raw_min:+.5} center={raw_center:+.5} max={raw_max:+.5} inverted={} deadzone={deadzone:.3}",
        control.label(),
        axis,
        inverted
    );
    Ok(CenteredAxisProfile::new(axis, calibration))
}

fn calibrate_throttle_axis(
    backend: &mut GilrsInputBackend,
    axis: HardwareAxis,
) -> Result<ThrottleAxisProfile, ControllerAppError> {
    let (raw_min, raw_max) = capture_axis_travel(backend, Control::Throttle, axis)?;
    let inverted = prompt_inversion(Control::Throttle)?;
    let calibration = ThrottleCalibration::new(raw_min, raw_max, inverted)?;
    println!(
        "throttle: axis={} min={raw_min:+.5} max={raw_max:+.5} inverted={inverted}",
        axis
    );
    Ok(ThrottleAxisProfile::new(axis, calibration))
}

fn capture_axis_travel(
    backend: &mut GilrsInputBackend,
    control: Control,
    axis: HardwareAxis,
) -> Result<(f64, f64), ControllerAppError> {
    println!();
    println!(
        "Move {} ({axis}) through its full travel repeatedly, then press ENTER.",
        control.label()
    );
    let (receiver, input_thread) = enter_signal();
    let mut raw_min = f64::INFINITY;
    let mut raw_max = f64::NEG_INFINITY;
    loop {
        let raw = sample_axis(backend, axis)?;
        raw_min = raw_min.min(raw);
        raw_max = raw_max.max(raw);
        println!(
            "{} axis={} raw={raw:+.5} observed_min={raw_min:+.5} observed_max={raw_max:+.5}",
            control.label(),
            axis
        );
        match receiver.try_recv() {
            Ok(result) => {
                result?;
                break;
            }
            Err(mpsc::TryRecvError::Empty) => thread::sleep(MONITOR_REFRESH_PERIOD),
            Err(mpsc::TryRecvError::Disconnected) => {
                return Err(io::Error::new(
                    io::ErrorKind::BrokenPipe,
                    "calibration input reader stopped unexpectedly",
                )
                .into());
            }
        }
    }
    let _ = input_thread.join();
    Ok((raw_min, raw_max))
}

fn sample_axis_average(
    backend: &mut GilrsInputBackend,
    axis: HardwareAxis,
) -> Result<f64, ControllerAppError> {
    const SAMPLE_COUNT: u32 = 10;
    let mut sum = 0.0;
    for sample_index in 0..SAMPLE_COUNT {
        sum += sample_axis(backend, axis)?;
        if sample_index + 1 < SAMPLE_COUNT {
            thread::sleep(Duration::from_millis(20));
        }
    }
    Ok(sum / f64::from(SAMPLE_COUNT))
}

fn sample_axis(
    backend: &mut GilrsInputBackend,
    axis: HardwareAxis,
) -> Result<f64, ControllerAppError> {
    poll_selected_raw(backend)?
        .get(axis)
        .ok_or(ControllerAppError::MissingSampledAxis { axis })
}

fn poll_selected_raw(
    backend: &mut GilrsInputBackend,
) -> Result<RawControllerState, ControllerAppError> {
    backend
        .poll_raw_axes()?
        .ok_or(ControllerAppError::SelectedDeviceDisconnected)
}

fn prompt_inversion(control: Control) -> Result<bool, ControllerAppError> {
    loop {
        let response = prompt_line(&format!("Invert {}? [y/N]: ", control.label()))?;
        match response.to_ascii_lowercase().as_str() {
            "" | "n" | "no" => return Ok(false),
            "y" | "yes" => return Ok(true),
            _ => println!("Enter y/yes or n/no."),
        }
    }
}

fn prompt_line(prompt: &str) -> Result<String, ControllerAppError> {
    print!("{prompt}");
    io::stdout().flush()?;
    let mut response = String::new();
    if io::stdin().read_line(&mut response)? == 0 {
        return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "standard input closed").into());
    }
    Ok(response.trim().to_owned())
}

fn enter_signal() -> (mpsc::Receiver<io::Result<usize>>, thread::JoinHandle<()>) {
    let (sender, receiver) = mpsc::channel();
    let input_thread = thread::spawn(move || {
        let mut response = String::new();
        let _ = sender.send(io::stdin().read_line(&mut response));
    });
    (receiver, input_thread)
}

pub(crate) fn format_raw_monitor_sample(
    sample_index: u64,
    device: &ControllerDeviceView,
    state: &RawControllerState,
) -> String {
    let mut output = String::new();
    if state.is_empty() {
        write!(
            &mut output,
            "raw hardware axes: sample={sample_index} session_id={} name={:?} axes=none",
            device.id, device.name
        )
        .expect("writing to a String cannot fail");
        return output;
    }
    for (index, axis) in state.axes().enumerate() {
        if index > 0 {
            output.push('\n');
        }
        write!(
            &mut output,
            "raw hardware axis: sample={sample_index} session_id={} name={:?} axis={} value={}",
            device.id,
            device.name,
            axis,
            format_axis(
                state
                    .get(axis)
                    .expect("an axis yielded by RawControllerState must have a value")
            )
        )
        .expect("writing to a String cannot fail");
    }
    output
}

pub(crate) fn format_controller_list(
    devices: &[ControllerDeviceView],
    selected_device_id: Option<usize>,
) -> String {
    let mut output = String::new();
    writeln!(&mut output, "RC Simulation Engine").expect("writing to a String cannot fail");
    writeln!(&mut output, "mode: controller-list").expect("writing to a String cannot fail");
    writeln!(&mut output, "devices: {}", devices.len()).expect("writing to a String cannot fail");
    match selected_device_id {
        Some(id) => writeln!(&mut output, "selected_device_id: {id}"),
        None => writeln!(&mut output, "selected_device_id: none"),
    }
    .expect("writing to a String cannot fail");

    if devices.is_empty() {
        writeln!(&mut output, "No controllers detected.").expect("writing to a String cannot fail");
        writeln!(&mut output, "Input mode: keyboard fallback")
            .expect("writing to a String cannot fail");
        return output;
    }

    for device in devices {
        writeln!(
            &mut output,
            "device: session_id={} name={:?} uuid={} vendor_id={} product_id={} auto_selected={}",
            device.id,
            device.name,
            device.uuid,
            optional_hex(device.vendor_id),
            optional_hex(device.product_id),
            if device.selected { "yes" } else { "no" }
        )
        .expect("writing to a String cannot fail");
    }
    output
}

pub(crate) fn format_viewer_controller_status(
    devices: &[ControllerDeviceView],
    selected_device_id: Option<usize>,
) -> String {
    match selected_device_id {
        Some(id) => format!(
            "Controller: {}\nInput mode: legacy controller mapping",
            device_identity(devices, id)
        ),
        None => "Controller: none\nInput mode: keyboard fallback".to_owned(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ControllerStatusEvent {
    Connected(usize),
    Disconnected(usize),
    Changed { from: usize, to: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ControllerStatusTracker {
    selected_device_id: Option<usize>,
}

impl ControllerStatusTracker {
    pub(crate) const fn new(selected_device_id: Option<usize>) -> Self {
        Self { selected_device_id }
    }

    pub(crate) fn observe(
        &mut self,
        selected_device_id: Option<usize>,
    ) -> Option<ControllerStatusEvent> {
        let event = match (self.selected_device_id, selected_device_id) {
            (None, None) => None,
            (None, Some(id)) => Some(ControllerStatusEvent::Connected(id)),
            (Some(id), None) => Some(ControllerStatusEvent::Disconnected(id)),
            (Some(previous), Some(current)) if previous == current => None,
            (Some(from), Some(to)) => Some(ControllerStatusEvent::Changed { from, to }),
        };
        self.selected_device_id = selected_device_id;
        event
    }
}

pub(crate) fn format_controller_transition(
    event: ControllerStatusEvent,
    devices: &[ControllerDeviceView],
) -> String {
    match event {
        ControllerStatusEvent::Connected(id) => format!(
            "Controller connected: {}; input mode: legacy controller mapping",
            device_identity(devices, id)
        ),
        ControllerStatusEvent::Disconnected(id) => {
            format!("Controller disconnected: id {id}; input mode: keyboard fallback")
        }
        ControllerStatusEvent::Changed { from, to } => format!(
            "Controller changed: {} -> {}; input mode: legacy controller mapping",
            device_identity(devices, from),
            device_identity(devices, to)
        ),
    }
}

pub(crate) fn format_monitor_sample(
    sample_index: u64,
    selected_device_id: Option<usize>,
    axes: Option<ControllerAxes>,
) -> String {
    let device = selected_device_id.map_or_else(|| "none".to_owned(), |id| id.to_string());
    match axes {
        Some(axes) => format!(
            "legacy logical axes / pre-calibration: sample={sample_index} device_id={device} roll={} pitch={} yaw={} throttle={}",
            format_axis(axes.roll),
            format_axis(axes.pitch),
            format_axis(axes.yaw),
            format_axis(axes.throttle)
        ),
        None => format!(
            "legacy logical axes / pre-calibration: sample={sample_index} device_id={device} axes=unavailable"
        ),
    }
}

fn device_identity(devices: &[ControllerDeviceView], id: usize) -> String {
    devices.iter().find(|device| device.id == id).map_or_else(
        || format!("metadata unavailable [id {id}]"),
        |device| format!("{} [id {}]", device.name, device.id),
    )
}

fn optional_hex(value: Option<u16>) -> String {
    value.map_or_else(|| "unknown".to_owned(), |value| format!("0x{value:04x}"))
}

fn format_axis(value: f64) -> String {
    if value.is_nan() {
        "NaN".to_owned()
    } else if value == f64::INFINITY {
        "+inf".to_owned()
    } else if value == f64::NEG_INFINITY {
        "-inf".to_owned()
    } else {
        format!("{value:+.4}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn device(id: usize, name: &str, selected: bool) -> ControllerDeviceView {
        ControllerDeviceView {
            id,
            name: name.to_owned(),
            uuid: format!("uuid-{id}"),
            vendor_id: Some(0x1200 + id as u16),
            product_id: Some(0x3400 + id as u16),
            selected,
        }
    }

    #[test]
    fn no_device_formatting_is_explicit_and_uses_keyboard_fallback() {
        let output = format_controller_list(&[], None);
        assert!(output.contains("devices: 0"));
        assert!(output.contains("selected_device_id: none"));
        assert!(output.contains("No controllers detected."));
        assert!(output.contains("Input mode: keyboard fallback"));
    }

    #[test]
    fn one_device_formatting_includes_all_available_identity_fields() {
        let output = format_controller_list(&[device(7, "Test Controller", true)], Some(7));
        assert!(output.contains("devices: 1"));
        assert!(output.contains("session_id=7"));
        assert!(output.contains("name=\"Test Controller\""));
        assert!(output.contains("uuid=uuid-7"));
        assert!(output.contains("vendor_id=0x1207"));
        assert!(output.contains("product_id=0x3407"));
    }

    #[test]
    fn multiple_device_listing_preserves_order_and_selected_marker() {
        let devices = [device(2, "First", false), device(5, "Second", true)];
        let output = format_controller_list(&devices, Some(5));
        assert!(output.find("session_id=2").unwrap() < output.find("session_id=5").unwrap());
        assert!(output.contains(
            "session_id=2 name=\"First\" uuid=uuid-2 vendor_id=0x1202 product_id=0x3402 auto_selected=no"
        ));
        assert!(output.contains(
            "session_id=5 name=\"Second\" uuid=uuid-5 vendor_id=0x1205 product_id=0x3405 auto_selected=yes"
        ));
    }

    #[test]
    fn none_to_some_produces_connected_once() {
        let mut tracker = ControllerStatusTracker::new(None);
        assert_eq!(
            tracker.observe(Some(4)),
            Some(ControllerStatusEvent::Connected(4))
        );
        assert_eq!(tracker.observe(Some(4)), None);
    }

    #[test]
    fn some_to_same_some_produces_no_duplicate_event() {
        let mut tracker = ControllerStatusTracker::new(Some(4));
        assert_eq!(tracker.observe(Some(4)), None);
        assert_eq!(tracker.observe(Some(4)), None);
    }

    #[test]
    fn some_to_none_produces_disconnected_once() {
        let mut tracker = ControllerStatusTracker::new(Some(4));
        assert_eq!(
            tracker.observe(None),
            Some(ControllerStatusEvent::Disconnected(4))
        );
        assert_eq!(tracker.observe(None), None);
    }

    #[test]
    fn some_a_to_some_b_produces_device_change_once() {
        let mut tracker = ControllerStatusTracker::new(Some(4));
        assert_eq!(
            tracker.observe(Some(9)),
            Some(ControllerStatusEvent::Changed { from: 4, to: 9 })
        );
        assert_eq!(tracker.observe(Some(9)), None);
    }

    #[test]
    fn monitor_sample_formatting_labels_legacy_values() {
        let output = format_monitor_sample(
            12,
            Some(3),
            Some(ControllerAxes::new(0.25, -0.5, 0.75, -1.0)),
        );
        assert_eq!(
            output,
            "legacy logical axes / pre-calibration: sample=12 device_id=3 roll=+0.2500 pitch=-0.5000 yaw=+0.7500 throttle=-1.0000"
        );
        assert!(format_monitor_sample(13, None, None).contains("axes=unavailable"));
    }

    #[test]
    fn monitor_sample_formatting_represents_non_finite_values_safely() {
        let output = format_monitor_sample(
            1,
            Some(3),
            Some(ControllerAxes::new(
                f64::NAN,
                f64::INFINITY,
                f64::NEG_INFINITY,
                0.0,
            )),
        );
        assert!(output.contains("roll=NaN"));
        assert!(output.contains("pitch=+inf"));
        assert!(output.contains("yaw=-inf"));
        assert!(output.contains("throttle=+0.0000"));
    }

    #[test]
    fn cli_parser_recognizes_controller_list() {
        assert_eq!(
            ControllerCommand::parse(["list".to_owned()].into_iter()).unwrap(),
            ControllerCommand::List
        );
    }

    #[test]
    fn cli_parser_recognizes_controller_monitor_and_bounds() {
        assert_eq!(
            ControllerCommand::parse(
                [
                    "monitor".to_owned(),
                    "--samples".to_owned(),
                    "25".to_owned(),
                    "--duration-seconds".to_owned(),
                    "3".to_owned(),
                ]
                .into_iter()
            )
            .unwrap(),
            ControllerCommand::Monitor(ControllerMonitorOptions {
                sample_limit: Some(25),
                duration_seconds: Some(3),
                raw: false,
                device_id: None,
            })
        );
    }

    #[test]
    fn cli_parser_recognizes_raw_monitor_and_explicit_device() {
        assert_eq!(
            ControllerCommand::parse(
                [
                    "monitor".to_owned(),
                    "--raw".to_owned(),
                    "--device-id".to_owned(),
                    "7".to_owned(),
                    "--samples".to_owned(),
                    "2".to_owned(),
                ]
                .into_iter()
            )
            .unwrap(),
            ControllerCommand::Monitor(ControllerMonitorOptions {
                sample_limit: Some(2),
                duration_seconds: None,
                raw: true,
                device_id: Some(7),
            })
        );
    }

    #[test]
    fn raw_monitor_format_includes_only_axes_present_in_the_snapshot() {
        let view = device(7, "Test Controller", true);
        let mut state = RawControllerState::new();
        state.insert(HardwareAxis::LeftStickX, 0.25).unwrap();
        state.insert(HardwareAxis::RightZ, -0.75).unwrap();
        let output = format_raw_monitor_sample(4, &view, &state);
        assert!(output.contains("name=\"Test Controller\""));
        assert!(output.contains("axis=left_stick_x value=+0.2500"));
        assert!(output.contains("axis=right_z value=-0.7500"));
        for absent in [
            HardwareAxis::LeftStickY,
            HardwareAxis::LeftZ,
            HardwareAxis::RightStickX,
            HardwareAxis::RightStickY,
            HardwareAxis::DPadX,
            HardwareAxis::DPadY,
        ] {
            assert!(!output.contains(&format!("axis={absent} ")));
        }
    }

    #[test]
    fn cli_parser_recognizes_controller_calibrate() {
        assert_eq!(
            ControllerCommand::parse(
                [
                    "calibrate".to_owned(),
                    "--output".to_owned(),
                    "controllers/tx16s.json".to_owned(),
                    "--deadzone".to_owned(),
                    "0.075".to_owned(),
                ]
                .into_iter()
            )
            .unwrap(),
            ControllerCommand::Calibrate(ControllerCalibrateOptions {
                output_path: PathBuf::from("controllers/tx16s.json"),
                deadzone: 0.075,
            })
        );
        assert!(ControllerCommand::parse(["calibrate".to_owned()].into_iter()).is_err());
        for invalid_deadzone in ["-0.1", "1", "NaN", "not-a-number"] {
            assert!(
                ControllerCommand::parse(
                    [
                        "calibrate".to_owned(),
                        "--output".to_owned(),
                        "profile.json".to_owned(),
                        "--deadzone".to_owned(),
                        invalid_deadzone.to_owned(),
                    ]
                    .into_iter()
                )
                .is_err()
            );
        }
    }

    #[test]
    fn bounded_monitor_arguments_fail_closed() {
        for arguments in [
            vec!["monitor", "--samples", "0"],
            vec!["monitor", "--samples", "not-a-number"],
            vec!["monitor", "--samples"],
            vec!["monitor", "--duration-seconds", "0"],
            vec!["monitor", "--duration-seconds", "1.5"],
            vec!["monitor", "--duration-seconds"],
            vec!["monitor", "--device-id", "2"],
            vec!["monitor", "--raw", "--device-id", "not-a-number"],
        ] {
            assert!(
                ControllerCommand::parse(arguments.iter().map(|argument| (*argument).to_owned()))
                    .is_err(),
                "arguments should fail: {arguments:?}"
            );
        }
    }

    #[test]
    fn viewer_status_formats_selected_and_absent_modes() {
        assert_eq!(
            format_viewer_controller_status(&[device(7, "Test Controller", true)], Some(7)),
            "Controller: Test Controller [id 7]\nInput mode: legacy controller mapping"
        );
        assert_eq!(
            format_viewer_controller_status(&[], None),
            "Controller: none\nInput mode: keyboard fallback"
        );
    }
}
