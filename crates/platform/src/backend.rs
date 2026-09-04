use crate::{
    ControllerAxes, DeviceIdentity, HardwareAxis, InputError, RawControllerState, match_device,
};
use gilrs::{Axis, EventType, Gamepad, GamepadId, Gilrs, GilrsBuilder};

/// Enumeration snapshot of one connected input device.
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

    /// Stable serializable identity derived from this snapshot.
    #[must_use]
    pub fn identity(&self) -> DeviceIdentity {
        DeviceIdentity::new(
            self.name.clone(),
            Some(self.uuid.clone()),
            self.vendor_id,
            self.product_id,
        )
    }
}

/// gilrs backend with deterministic lowest-ID selection and no implicit axis filters.
pub struct GilrsInputBackend {
    gilrs: Gilrs,
    selected: Option<GamepadId>,
    explicit: Option<usize>,
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
            explicit: None,
        };
        backend.select_lowest_connected();
        Ok(backend)
    }

    #[must_use]
    pub fn devices(&self) -> Vec<InputDeviceInfo> {
        let mut devices: Vec<_> = self
            .gilrs
            .gamepads()
            .map(|(id, gamepad)| device_info(id, gamepad))
            .collect();
        devices.sort_by_key(InputDeviceInfo::id);
        devices
    }

    pub fn poll_axes(&mut self) -> Option<ControllerAxes> {
        let selection_may_have_changed = self.drain_events();
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

    /// Explicitly selects the connected device matching `identity`.
    ///
    /// Matching follows [`crate::match_device`]: a usable profile UUID is
    /// decisive, otherwise vendor ID + product ID + exact name, otherwise an
    /// unambiguous exact name. Ambiguous or missing matches are rejected with
    /// typed errors instead of falling back to another device.
    pub fn select_device(
        &mut self,
        identity: &DeviceIdentity,
    ) -> Result<InputDeviceInfo, InputError> {
        self.drain_events();
        let mut device_ids: Vec<GamepadId> = Vec::new();
        let mut devices: Vec<InputDeviceInfo> = Vec::new();
        let mut candidates: Vec<DeviceIdentity> = Vec::new();
        for (id, gamepad) in self.gilrs.gamepads() {
            let device = device_info(id, gamepad);
            device_ids.push(id);
            candidates.push(device.identity());
            devices.push(device);
        }
        let index = match_device(identity, &candidates)?;
        self.explicit = Some(device_ids[index].into());
        Ok(devices[index].clone())
    }

    /// The explicitly selected device, while it remains connected.
    #[must_use]
    pub fn explicit_device(&self) -> Option<InputDeviceInfo> {
        let selected = self.explicit?;
        self.gilrs
            .gamepads()
            .find(|(id, _)| usize::from(*id) == selected)
            .map(|(id, gamepad)| device_info(id, gamepad))
    }

    /// Polls raw axis state of the device selected by [`Self::select_device`].
    ///
    /// Returns `Ok(None)` when the requested device is no longer connected,
    /// and [`InputError::RequestedDeviceNotFound`] when no device has been
    /// selected. The returned state contains only axes the device actually
    /// reports; an axis missing from the state is unavailable, not zero.
    pub fn poll_raw_axes(&mut self) -> Result<Option<RawControllerState>, InputError> {
        let had_explicit_selection = self.explicit.is_some();
        self.drain_events();
        if had_explicit_selection && self.explicit.is_none() {
            return Ok(None);
        }
        let Some(selected) = self.explicit else {
            return Err(InputError::RequestedDeviceNotFound);
        };
        let Some((_, gamepad)) = self
            .gilrs
            .gamepads()
            .find(|(id, _)| usize::from(*id) == selected)
        else {
            return Ok(None);
        };
        let mut state = RawControllerState::new();
        for axis in HardwareAxis::ALL {
            let gilrs_axis = gilrs_axis_of(axis);
            if gamepad.axis_data(gilrs_axis).is_some() {
                state.insert(axis, f64::from(gamepad.value(gilrs_axis)))?;
            }
        }
        Ok(Some(state))
    }

    fn drain_events(&mut self) -> bool {
        let mut selection_may_have_changed = false;
        while let Some(event) = self.gilrs.next_event() {
            let disconnected = matches!(&event.event, EventType::Disconnected);
            selection_may_have_changed |=
                disconnected || matches!(&event.event, EventType::Connected);
            if disconnected && self.explicit == Some(event.id.into()) {
                self.explicit = None;
            }
        }
        selection_may_have_changed
    }

    fn select_lowest_connected(&mut self) {
        self.selected = self
            .gilrs
            .gamepads()
            .map(|(id, _)| id)
            .min_by_key(|id| usize::from(*id));
    }
}

fn device_info(id: GamepadId, gamepad: Gamepad<'_>) -> InputDeviceInfo {
    InputDeviceInfo {
        id: id.into(),
        name: gamepad.name().to_owned(),
        uuid: encode_uuid(gamepad.uuid()),
        vendor_id: gamepad.vendor_id(),
        product_id: gamepad.product_id(),
    }
}

fn gilrs_axis_of(axis: HardwareAxis) -> Axis {
    match axis {
        HardwareAxis::LeftStickX => Axis::LeftStickX,
        HardwareAxis::LeftStickY => Axis::LeftStickY,
        HardwareAxis::LeftZ => Axis::LeftZ,
        HardwareAxis::RightStickX => Axis::RightStickX,
        HardwareAxis::RightStickY => Axis::RightStickY,
        HardwareAxis::RightZ => Axis::RightZ,
        HardwareAxis::DPadX => Axis::DPadX,
        HardwareAxis::DPadY => Axis::DPadY,
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
