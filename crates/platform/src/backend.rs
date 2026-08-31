use crate::{ControllerAxes, InputError};
use gilrs::{Axis, EventType, GamepadId, Gilrs, GilrsBuilder};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InputDeviceInfo {
    id: usize,
    name: String,
    uuid: String,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
}

impl InputDeviceInfo {
    #[must_use]
    pub const fn id(&self) -> usize {
        self.id
    }

    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[must_use]
    pub fn uuid(&self) -> &str {
        &self.uuid
    }

    #[must_use]
    pub const fn vendor_id(&self) -> Option<u16> {
        self.vendor_id
    }

    #[must_use]
    pub const fn product_id(&self) -> Option<u16> {
        self.product_id
    }
}

/// gilrs backend with deterministic lowest-ID selection and no implicit axis filters.
pub struct GilrsInputBackend {
    gilrs: Gilrs,
    selected: Option<GamepadId>,
}

impl GilrsInputBackend {
    pub fn new() -> Result<Self, InputError> {
        let gilrs = GilrsBuilder::new()
            .with_force_feedback(false)
            .with_default_filters(false)
            .build()
            .map_err(|error| InputError::BackendInitialization(error.to_string()))?;
        let mut backend = Self {
            gilrs,
            selected: None,
        };
        backend.select_lowest_connected();
        Ok(backend)
    }

    #[must_use]
    pub fn devices(&self) -> Vec<InputDeviceInfo> {
        let mut devices: Vec<_> = self
            .gilrs
            .gamepads()
            .map(|(id, gamepad)| InputDeviceInfo {
                id: id.into(),
                name: gamepad.name().to_owned(),
                uuid: encode_uuid(gamepad.uuid()),
                vendor_id: gamepad.vendor_id(),
                product_id: gamepad.product_id(),
            })
            .collect();
        devices.sort_by_key(InputDeviceInfo::id);
        devices
    }

    pub fn poll_axes(&mut self) -> Option<ControllerAxes> {
        let mut selection_may_have_changed = false;
        while let Some(event) = self.gilrs.next_event() {
            selection_may_have_changed |=
                matches!(event.event, EventType::Connected | EventType::Disconnected);
        }
        if selection_may_have_changed
            || self
                .selected
                .and_then(|id| self.gilrs.connected_gamepad(id))
                .is_none()
        {
            self.select_lowest_connected();
        }
        let gamepad = self
            .selected
            .and_then(|id| self.gilrs.connected_gamepad(id))?;
        Some(ControllerAxes::new(
            f64::from(gamepad.value(Axis::LeftStickX)),
            f64::from(gamepad.value(Axis::LeftStickY)),
            f64::from(gamepad.value(Axis::RightStickX)),
            f64::from(gamepad.value(Axis::RightStickY)),
        ))
    }

    #[must_use]
    pub fn selected_device_id(&self) -> Option<usize> {
        self.selected.map(Into::into)
    }

    fn select_lowest_connected(&mut self) {
        self.selected = self
            .gilrs
            .gamepads()
            .map(|(id, _)| id)
            .min_by_key(|id| usize::from(*id));
    }
}

fn encode_uuid(bytes: [u8; 16]) -> String {
    use std::fmt::Write as _;
    let mut output = String::with_capacity(32);
    for byte in bytes {
        write!(&mut output, "{byte:02x}").expect("writing into a String cannot fail");
    }
    output
}
