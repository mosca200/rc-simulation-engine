use platform::{ControllerAxes, GilrsInputBackend, InputDeviceInfo, InputError};
use std::{
    fmt::Write as _,
    thread,
    time::{Duration, Instant},
};

const MONITOR_REFRESH_PERIOD: Duration = Duration::from_millis(100);
const MONITOR_REFRESH_HZ: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControllerCommand {
    List,
    Monitor(ControllerMonitorOptions),
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
            Some(command) => Err(format!("unknown controller command: {command}")),
            None => Err("missing controller command; expected `list` or `monitor`".to_owned()),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControllerMonitorOptions {
    sample_limit: Option<u64>,
    duration_seconds: Option<u64>,
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
                _ => return Err(format!("unknown controller monitor argument: {argument}")),
            }
        }
        Ok(options)
    }
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

pub fn run_controller(command: ControllerCommand) -> Result<(), InputError> {
    match command {
        ControllerCommand::List => run_controller_list(),
        ControllerCommand::Monitor(options) => run_controller_monitor(options),
    }
}

fn run_controller_list() -> Result<(), InputError> {
    let backend = GilrsInputBackend::new()?;
    let selected_device_id = backend.selected_device_id();
    let devices = backend.devices();
    let views = controller_device_views(&devices, selected_device_id);
    print!("{}", format_controller_list(&views, selected_device_id));
    Ok(())
}

fn run_controller_monitor(options: ControllerMonitorOptions) -> Result<(), InputError> {
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
            })
        );
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
