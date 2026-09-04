use platform::{ControllerProfile, DeviceIdentity, DeviceLink, InputError, RawControllerState};
use sim_core::PilotInput;
use std::{fs, io, path::Path};
use thiserror::Error;

#[derive(Debug, Error)]
pub(crate) enum ControllerProfileFileError {
    #[error("failed to read controller profile {path}: {source}")]
    Read {
        path: String,
        #[source]
        source: io::Error,
    },
    #[error("failed to decode controller profile {path}: {source}")]
    Decode {
        path: String,
        #[source]
        source: InputError,
    },
    #[error("failed to create controller profile directory {path}: {source}")]
    CreateDirectory {
        path: String,
        #[source]
        source: io::Error,
    },
    #[error("failed to encode controller profile {path}: {source}")]
    Encode {
        path: String,
        #[source]
        source: InputError,
    },
    #[error("failed to write controller profile {path}: {source}")]
    Write {
        path: String,
        #[source]
        source: io::Error,
    },
}

pub(crate) fn load_controller_profile(
    path: &Path,
) -> Result<ControllerProfile, ControllerProfileFileError> {
    let display_path = path.display().to_string();
    let json = fs::read_to_string(path).map_err(|source| ControllerProfileFileError::Read {
        path: display_path.clone(),
        source,
    })?;
    ControllerProfile::from_json(&json).map_err(|source| ControllerProfileFileError::Decode {
        path: display_path,
        source,
    })
}

pub(crate) fn save_controller_profile(
    path: &Path,
    profile: &ControllerProfile,
) -> Result<(), ControllerProfileFileError> {
    let display_path = path.display().to_string();
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent).map_err(|source| {
            ControllerProfileFileError::CreateDirectory {
                path: parent.display().to_string(),
                source,
            }
        })?;
    }
    let json = profile
        .to_json()
        .map_err(|source| ControllerProfileFileError::Encode {
            path: display_path.clone(),
            source,
        })?;
    fs::write(path, format!("{json}\n")).map_err(|source| ControllerProfileFileError::Write {
        path: display_path,
        source,
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CalibratedControllerEvent {
    Disconnected,
    Reconnected,
}

impl CalibratedControllerEvent {
    pub(crate) const fn message(self) -> &'static str {
        match self {
            Self::Disconnected => "Controller disconnected — controls neutralized.",
            Self::Reconnected => "Controller reconnected.",
        }
    }
}

/// App-side fail-closed state for one explicitly requested controller profile.
///
/// JSON and filesystem work happen before this state is created. Its cached
/// `PilotInput` makes the physics-step read allocation-free; matching and raw
/// hardware polling remain render/event-loop work.
pub(crate) struct CalibratedControllerState {
    profile: ControllerProfile,
    link: DeviceLink,
    input: PilotInput,
    connected: bool,
    ever_connected: bool,
}

impl CalibratedControllerState {
    pub(crate) fn new(profile: ControllerProfile) -> Self {
        let link = DeviceLink::new(profile.device().clone());
        Self {
            profile,
            link,
            input: PilotInput::neutral(),
            connected: false,
            ever_connected: false,
        }
    }

    pub(crate) fn profile(&self) -> &ControllerProfile {
        &self.profile
    }

    pub(crate) fn requested_device(&self) -> &DeviceIdentity {
        self.link.target()
    }

    pub(crate) fn match_requested_device(
        &mut self,
        candidates: &[DeviceIdentity],
    ) -> Result<usize, InputError> {
        self.link.update(candidates)
    }

    pub(crate) fn accept_raw_state(
        &mut self,
        state: &RawControllerState,
    ) -> Result<Option<CalibratedControllerEvent>, InputError> {
        let input = match self.profile.to_pilot_input(state) {
            Ok(input) => input,
            Err(error) => {
                self.neutralize();
                return Err(error);
            }
        };
        self.input = input;
        let event = if !self.connected && self.ever_connected {
            Some(CalibratedControllerEvent::Reconnected)
        } else {
            None
        };
        self.connected = true;
        self.ever_connected = true;
        Ok(event)
    }

    pub(crate) fn neutralize(&mut self) -> Option<CalibratedControllerEvent> {
        self.input = PilotInput::neutral();
        let event = self
            .connected
            .then_some(CalibratedControllerEvent::Disconnected);
        self.connected = false;
        event
    }

    pub(crate) const fn input(&self) -> PilotInput {
        self.input
    }

    pub(crate) const fn is_connected(&self) -> bool {
        self.connected
    }
}

pub(crate) fn format_device_identity(identity: &DeviceIdentity) -> String {
    format!(
        "name={:?} uuid={} vendor_id={} product_id={}",
        identity.name(),
        identity.uuid().unwrap_or("unknown"),
        optional_hex(identity.vendor_id()),
        optional_hex(identity.product_id())
    )
}

fn optional_hex(value: Option<u16>) -> String {
    value.map_or_else(|| "unknown".to_owned(), |value| format!("0x{value:04x}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use platform::{
        CONTROLLER_PROFILE_SCHEMA_VERSION, CenteredAxisProfile, CenteredCalibration, Control,
        HardwareAxis, ProfileAxes, ThrottleAxisProfile, ThrottleCalibration,
    };
    use std::{
        path::PathBuf,
        sync::atomic::{AtomicU64, Ordering},
    };

    static TEMP_FILE_SEQUENCE: AtomicU64 = AtomicU64::new(0);

    fn identity(name: &str, uuid: &str) -> DeviceIdentity {
        DeviceIdentity::new(name, Some(uuid.to_owned()), Some(0x1209), Some(0x4f54))
    }

    fn profile() -> ControllerProfile {
        ControllerProfile::new(
            identity("Test Transmitter", "profile-uuid"),
            ProfileAxes::new(
                CenteredAxisProfile::new(
                    HardwareAxis::LeftStickX,
                    CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, 0.05).unwrap(),
                ),
                CenteredAxisProfile::new(
                    HardwareAxis::LeftStickY,
                    CenteredCalibration::new(Control::Pitch, -1.0, 0.0, 1.0, true, 0.05).unwrap(),
                ),
                CenteredAxisProfile::new(
                    HardwareAxis::RightStickX,
                    CenteredCalibration::new(Control::Yaw, -1.0, 0.0, 1.0, false, 0.05).unwrap(),
                ),
                ThrottleAxisProfile::new(
                    HardwareAxis::RightStickY,
                    ThrottleCalibration::new(-1.0, 1.0, false).unwrap(),
                ),
            ),
        )
        .unwrap()
    }

    fn raw_state(roll: f64, throttle: f64) -> RawControllerState {
        let mut state = RawControllerState::new();
        state.insert(HardwareAxis::LeftStickX, roll).unwrap();
        state.insert(HardwareAxis::LeftStickY, 0.0).unwrap();
        state.insert(HardwareAxis::RightStickX, 0.0).unwrap();
        state.insert(HardwareAxis::RightStickY, throttle).unwrap();
        state
    }

    fn unique_temp_path(label: &str) -> PathBuf {
        std::env::temp_dir().join(format!(
            "rcsim-{label}-{}-{}.json",
            std::process::id(),
            TEMP_FILE_SEQUENCE.fetch_add(1, Ordering::Relaxed)
        ))
    }

    #[test]
    fn valid_profile_load_save_round_trip_is_app_owned() {
        let path = unique_temp_path("profile-roundtrip");
        let expected = profile();
        save_controller_profile(&path, &expected).unwrap();
        assert_eq!(load_controller_profile(&path).unwrap(), expected);
        fs::remove_file(path).unwrap();
    }

    #[test]
    fn malformed_json_and_unsupported_schema_are_distinct_load_errors() {
        let malformed_path = unique_temp_path("profile-malformed");
        fs::write(&malformed_path, "{").unwrap();
        assert!(matches!(
            load_controller_profile(&malformed_path),
            Err(ControllerProfileFileError::Decode {
                source: InputError::InvalidControllerProfile(_),
                ..
            })
        ));
        fs::remove_file(malformed_path).unwrap();

        let schema_path = unique_temp_path("profile-schema");
        let invalid = profile().to_json().unwrap().replacen(
            "\"schema_version\": 1",
            "\"schema_version\": 2",
            1,
        );
        fs::write(&schema_path, invalid).unwrap();
        assert!(matches!(
            load_controller_profile(&schema_path),
            Err(ControllerProfileFileError::Decode {
                source: InputError::UnsupportedProfileVersion {
                    found: 2,
                    supported: CONTROLLER_PROFILE_SCHEMA_VERSION
                },
                ..
            })
        ));
        fs::remove_file(schema_path).unwrap();
    }

    #[test]
    fn missing_path_and_invalid_calibration_keep_actionable_context() {
        let missing_path = unique_temp_path("profile-missing");
        let missing = load_controller_profile(&missing_path).unwrap_err();
        match missing {
            ControllerProfileFileError::Read { path, source } => {
                assert!(path.contains("profile-missing"));
                assert_eq!(source.kind(), io::ErrorKind::NotFound);
            }
            other => panic!("expected path-aware read error, got {other:?}"),
        }

        let invalid_path = unique_temp_path("profile-invalid-calibration");
        let invalid =
            profile()
                .to_json()
                .unwrap()
                .replacen("\"raw_center\": 0.0", "\"raw_center\": -2.0", 1);
        fs::write(&invalid_path, invalid).unwrap();
        assert!(matches!(
            load_controller_profile(&invalid_path),
            Err(ControllerProfileFileError::Decode {
                source: InputError::InvalidCalibrationOrder {
                    control: Control::Roll
                },
                ..
            })
        ));
        fs::remove_file(invalid_path).unwrap();
    }

    #[test]
    fn device_not_found_and_ambiguity_remain_fail_closed() {
        let mut state = CalibratedControllerState::new(profile());
        let other = [identity("Other", "other-uuid")];
        assert_eq!(
            state.match_requested_device(&other),
            Err(InputError::RequestedDeviceNotFound)
        );
        assert_eq!(state.input(), PilotInput::neutral());

        let duplicates = [
            identity("Test Transmitter", "profile-uuid"),
            identity("Test Transmitter", "profile-uuid"),
        ];
        assert_eq!(
            state.match_requested_device(&duplicates),
            Err(InputError::AmbiguousDeviceMatch { candidates: 2 })
        );
    }

    #[test]
    fn disconnect_neutralizes_once_and_never_reuses_stale_input() {
        let requested = identity("Test Transmitter", "profile-uuid");
        let mut state = CalibratedControllerState::new(profile());
        assert_eq!(state.match_requested_device(&[requested]), Ok(0));
        assert_eq!(state.accept_raw_state(&raw_state(0.8, 0.7)), Ok(None));
        assert_ne!(state.input(), PilotInput::neutral());

        assert_eq!(
            state.neutralize(),
            Some(CalibratedControllerEvent::Disconnected)
        );
        assert_eq!(state.input(), PilotInput::neutral());
        assert_eq!(state.neutralize(), None);
        assert_eq!(state.input(), PilotInput::neutral());
    }

    #[test]
    fn only_the_same_identity_can_reconnect_and_resume_input() {
        let requested = identity("Test Transmitter", "profile-uuid");
        let other = identity("Other", "other-uuid");
        let mut state = CalibratedControllerState::new(profile());
        state
            .match_requested_device(std::slice::from_ref(&requested))
            .unwrap();
        state.accept_raw_state(&raw_state(0.4, 0.6)).unwrap();
        state.neutralize();

        assert_eq!(
            state.match_requested_device(&[other]),
            Err(InputError::RequestedDeviceNotFound)
        );
        assert_eq!(state.input(), PilotInput::neutral());

        assert_eq!(state.match_requested_device(&[requested]), Ok(0));
        assert_eq!(
            state.accept_raw_state(&raw_state(-0.4, -0.5)),
            Ok(Some(CalibratedControllerEvent::Reconnected))
        );
        assert_ne!(state.input(), PilotInput::neutral());
    }
}
