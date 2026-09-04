use serde::{Deserialize, Serialize};

use crate::InputError;

/// Stable serializable identity of one input device.
///
/// Transient gilrs gamepad IDs are deliberately not persisted: they are
/// re-enumerated on every process start and are not meaningful across
/// sessions or machines.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct DeviceIdentity {
    name: String,
    uuid: Option<String>,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
}

impl DeviceIdentity {
    #[must_use]
    pub fn new(
        name: impl Into<String>,
        uuid: Option<String>,
        vendor_id: Option<u16>,
        product_id: Option<u16>,
    ) -> Self {
        Self {
            name: name.into(),
            uuid,
            vendor_id,
            product_id,
        }
    }

    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[must_use]
    pub fn uuid(&self) -> Option<&str> {
        self.uuid.as_deref()
    }

    #[must_use]
    pub const fn vendor_id(&self) -> Option<u16> {
        self.vendor_id
    }

    #[must_use]
    pub const fn product_id(&self) -> Option<u16> {
        self.product_id
    }

    fn usable_uuid(&self) -> Option<&str> {
        self.uuid.as_deref().filter(|uuid| !uuid.trim().is_empty())
    }
}

/// Deterministically matches a requested identity against connected candidates.
///
/// Matching rules, in priority order:
///
/// 1. If the requested identity carries a usable UUID, only exact
///    (ASCII case-insensitive) UUID matches are considered. The UUID is
///    decisive: there is no fallback to weaker identifiers, which prevents
///    silently binding to a different unit of the same model.
/// 2. Otherwise, if vendor ID and product ID are both present, candidates must
///    match both, plus the exact name when the requested name is non-empty.
/// 3. Otherwise the fallback is an exact name match only.
///
/// Exactly one match is required. Zero matches return
/// [`InputError::RequestedDeviceNotFound`], multiple matches return
/// [`InputError::AmbiguousDeviceMatch`], and an empty candidate list returns
/// [`InputError::NoDevices`].
pub fn match_device(
    target: &DeviceIdentity,
    candidates: &[DeviceIdentity],
) -> Result<usize, InputError> {
    if candidates.is_empty() {
        return Err(InputError::NoDevices);
    }
    let matches: Vec<usize> = candidates
        .iter()
        .enumerate()
        .filter(|(_, candidate)| identity_matches(target, candidate))
        .map(|(index, _)| index)
        .collect();
    match matches.len() {
        0 => Err(InputError::RequestedDeviceNotFound),
        1 => Ok(matches[0]),
        count => Err(InputError::AmbiguousDeviceMatch { candidates: count }),
    }
}

fn identity_matches(target: &DeviceIdentity, candidate: &DeviceIdentity) -> bool {
    if let Some(uuid) = target.usable_uuid() {
        return candidate
            .usable_uuid()
            .is_some_and(|candidate_uuid| candidate_uuid.eq_ignore_ascii_case(uuid));
    }
    if let (Some(vendor_id), Some(product_id)) = (target.vendor_id(), target.product_id()) {
        if candidate.vendor_id() != Some(vendor_id) || candidate.product_id() != Some(product_id) {
            return false;
        }
        return target.name().is_empty() || candidate.name() == target.name();
    }
    !target.name().is_empty() && candidate.name() == target.name()
}

/// Platform-level connection state of a requested device.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceLinkStatus {
    /// The requested device has never matched while this link existed.
    Absent,
    /// The requested device currently matches one connected device.
    Present,
    /// The requested device matched before but does not match right now.
    Disconnected,
    /// The most recent match attempt matched more than one device.
    Ambiguous,
}

/// Hardware-independent tracker for the connection state of one requested device.
///
/// The caller supplies the current candidate identities on every poll (for
/// example from `GilrsInputBackend::devices` mapped through
/// `InputDeviceInfo::identity`) and the link reports whether the requested
/// device is present, absent, disconnected, or ambiguous. Backend
/// initialization failure is reported separately as
/// [`InputError::BackendInitialization`] when the backend is constructed. The
/// link never chooses a fallback policy; that decision belongs to the
/// application layer.
#[derive(Debug, Clone)]
pub struct DeviceLink {
    target: DeviceIdentity,
    ever_connected: bool,
    status: DeviceLinkStatus,
}

impl DeviceLink {
    #[must_use]
    pub fn new(target: DeviceIdentity) -> Self {
        Self {
            target,
            ever_connected: false,
            status: DeviceLinkStatus::Absent,
        }
    }

    #[must_use]
    pub fn target(&self) -> &DeviceIdentity {
        &self.target
    }

    #[must_use]
    pub const fn status(&self) -> DeviceLinkStatus {
        self.status
    }

    /// Re-matches the requested device against `candidates` and updates the status.
    pub fn update(&mut self, candidates: &[DeviceIdentity]) -> Result<usize, InputError> {
        let result = match_device(&self.target, candidates);
        self.status = match &result {
            Ok(_) => {
                self.ever_connected = true;
                DeviceLinkStatus::Present
            }
            Err(InputError::AmbiguousDeviceMatch { .. }) => DeviceLinkStatus::Ambiguous,
            Err(_) => {
                if self.ever_connected {
                    DeviceLinkStatus::Disconnected
                } else {
                    DeviceLinkStatus::Absent
                }
            }
        };
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity(
        name: &str,
        uuid: Option<&str>,
        vendor_id: Option<u16>,
        product_id: Option<u16>,
    ) -> DeviceIdentity {
        DeviceIdentity::new(name, uuid.map(str::to_owned), vendor_id, product_id)
    }

    #[test]
    fn empty_candidate_list_reports_no_devices() {
        let target = identity("Stick", Some("uuid-1"), None, None);
        assert_eq!(match_device(&target, &[]), Err(InputError::NoDevices));
    }

    #[test]
    fn link_status_tracks_presence_disconnection_and_ambiguity() {
        let mut link = DeviceLink::new(identity("TX", Some("uuid-1"), None, None));
        assert_eq!(link.status(), DeviceLinkStatus::Absent);

        let present = [identity("TX", Some("uuid-1"), None, None)];
        assert_eq!(link.update(&present), Ok(0));
        assert_eq!(link.status(), DeviceLinkStatus::Present);

        let other = [identity("Other", Some("uuid-9"), None, None)];
        assert_eq!(
            link.update(&other),
            Err(InputError::RequestedDeviceNotFound)
        );
        assert_eq!(link.status(), DeviceLinkStatus::Disconnected);

        assert_eq!(link.update(&[]), Err(InputError::NoDevices));
        assert_eq!(link.status(), DeviceLinkStatus::Disconnected);

        assert_eq!(link.update(&present), Ok(0));
        assert_eq!(link.status(), DeviceLinkStatus::Present);
    }

    #[test]
    fn link_reports_ambiguous_match_without_forgetting_history() {
        let mut link = DeviceLink::new(identity("TX", None, Some(1), Some(1)));
        let duplicated = [
            identity("TX", None, Some(1), Some(1)),
            identity("TX", None, Some(1), Some(1)),
        ];
        assert_eq!(
            link.update(&duplicated),
            Err(InputError::AmbiguousDeviceMatch { candidates: 2 })
        );
        assert_eq!(link.status(), DeviceLinkStatus::Ambiguous);
    }
}
