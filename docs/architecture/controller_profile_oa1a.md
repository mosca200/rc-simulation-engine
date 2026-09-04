# OA1A Controller Core: Device Identity, Calibration, and Profiles

## Boundary

OA1A extends `crates/platform` so that a real RC transmitter, such as a RadioMaster TX16S
connected over USB joystick/HID, can later be calibrated and used reliably. The slice is
platform-core only: no application CLI and no viewer changes.

The central change is the separation of

```text
RAW HARDWARE AXES            ->        CALIBRATED PILOT CONTROLS
HardwareAxis, RawControllerState       ControllerProfile -> PilotInput
```

The existing `ControllerAxes` + `InputMapping` path (fixed left/right stick mapping) and the
keyboard fallback remain unchanged and keep their public APIs.

## Raw hardware axes

`HardwareAxis` is the stable internal identifier for addressable device axes:

```text
left_stick_x  left_stick_y  left_z
right_stick_x right_stick_y right_z
dpad_x        dpad_y
```

Each variant maps one-to-one to an explicitly supported `gilrs::Axis` variant;
`gilrs::Axis::Unknown` is never used. A `RawControllerState` snapshot contains only axes the
device actually reports (`Gamepad::axis_data` is the gate). An axis missing from the snapshot is
unavailable, never silently zero. `RawControllerState` rejects non-finite values at insertion.

No custom HID parsing exists in OA1A; the backend uses only standard gilrs axis access.

## Device identity and matching

`DeviceIdentity` is the serializable stable identity:

| field        | type          | notes                                             |
|--------------|---------------|---------------------------------------------------|
| `name`       | string        | device-reported name                              |
| `uuid`       | string or null| backend-encoded UUID                              |
| `vendor_id`  | u16 or null   | USB vendor ID when available                      |
| `product_id` | u16 or null   | USB product ID when available                     |

Transient gilrs gamepad IDs are never persisted. `InputDeviceInfo::identity` derives the
identity from a device enumeration snapshot.

`match_device` is deterministic and hardware-independent:

1. If the requested identity carries a usable (non-empty) UUID, only exact
   ASCII case-insensitive UUID matches count. The UUID is decisive; there is no fallback to
   weaker identifiers, which prevents silently binding to a different unit of the same model.
2. Otherwise, vendor ID + product ID must both match, plus the exact name when the requested
   name is non-empty.
3. Otherwise the fallback is an exact name match only.

Exactly one match is required. Zero matches return `RequestedDeviceNotFound` (or `NoDevices` for
an empty candidate list); multiple matches return `AmbiguousDeviceMatch`. No RadioMaster
VID/PID or name strings are hard-coded anywhere in the platform layer.

## Calibration model

All calibration math is pure, deterministic, and independent of gilrs.

Centered axis (`CenteredCalibration`: `raw_min`, `raw_center`, `raw_max`, `inverted`,
`deadzone`):

```text
clamped    = clamp(raw, raw_min, raw_max)
normalized = (clamped - raw_min) / (raw_center - raw_min) - 1     if clamped <= raw_center
           = (clamped - raw_center) / (raw_max - raw_center)      otherwise

magnitude  = abs(normalized)
responsive = 0                                                     if magnitude <= deadzone
           = sign(normalized) * clamp((magnitude - deadzone) / (1 - deadzone), 0, 1)

output     = inverted ? -responsive : responsive
```

- `raw_min` maps to -1, `raw_center` to 0, `raw_max` to +1, exactly.
- Asymmetric travel around the physical center is supported (independent half spans).
- Values outside `[raw_min, raw_max]` saturate.
- The deadzone is centered on the calibrated center; travel outside it is rescaled
  continuously onto the remaining range (continuous at the deadzone boundary).

Throttle (`ThrottleCalibration`: `raw_min`, `raw_max`, `inverted`):

```text
position = clamp((raw - raw_min) / (raw_max - raw_min), 0, 1)
output   = inverted ? 1 - position : position
```

No throttle endpoint deadband is applied: at calibrated endpoints the physical stop is the
authoritative reference, and a deadband would only mask endpoint calibration errors.

Validation rejects non-finite values, invalid ordering (`raw_min < raw_center < raw_max` for
centered axes, `raw_min < raw_max` for throttle), deadzones outside `[0, 1)`, and degenerate
spans (each calibrated span must cover at least `MIN_CALIBRATION_SPAN = 1e-6`, because gilrs
raw samples originate from f32 device values with a typical integer resolution of about
1/32768). There is no smoothing, adaptive filtering, or hidden state.

## Controller profile schema

`ControllerProfile` is versioned JSON, schema version `1`:

```json
{
  "schema_version": 1,
  "device": {
    "name": "Example Transmitter",
    "uuid": "00112233445566778899aabbccddeeff",
    "vendor_id": null,
    "product_id": null
  },
  "axes": {
    "roll": {
      "source": "left_stick_x",
      "raw_min": -1.0,
      "raw_center": 0.0,
      "raw_max": 1.0,
      "inverted": false,
      "deadzone": 0.05
    },
    "pitch": { "source": "left_stick_y", "raw_min": -1.0, "raw_center": 0.0, "raw_max": 1.0, "inverted": true, "deadzone": 0.05 },
    "yaw": { "source": "right_stick_x", "raw_min": -1.0, "raw_center": 0.0, "raw_max": 1.0, "inverted": false, "deadzone": 0.05 },
    "throttle": { "source": "right_stick_y", "raw_min": -1.0, "raw_max": 1.0, "inverted": true }
  }
}
```

- `ControllerProfile::from_json` decodes and validates; `to_json` renders the stable format.
- Unsupported schema versions are rejected with `UnsupportedProfileVersion`.
- Invalid calibrations, malformed JSON, unknown axis identifiers, and duplicate hardware-axis
  assignments (the same axis assigned to more than one of roll/pitch/yaw/throttle) are rejected
  with typed errors.
- The platform layer never reads or writes files; path policy belongs to the application.

## Mapping into PilotInput

```text
RawControllerState + validated ControllerProfile -> PilotInput
```

`ControllerProfile::to_pilot_input` looks up the hardware axis assigned to each control,
applies the control's calibration, and constructs `sim_core::PilotInput`. Outputs are
`roll/pitch/yaw` in `[-1, 1]` and `throttle` in `[0, 1]`; no output is ever NaN or non-finite.
A requested hardware axis missing from the raw state returns `UnavailableHardwareAxis` instead
of inventing a value.

## Device state and errors

`DeviceLink` tracks the requested device across polls with status `Absent` (never matched),
`Present` (currently matched), `Disconnected` (matched before, absent now), and `Ambiguous`
(last match matched multiple devices). Backend initialization failure is reported as
`BackendInitialization` when constructing `GilrsInputBackend`. The platform layer never panics
on device loss and never chooses a fallback policy; OA1B decides whether runtime falls back to
keyboard or neutral input.

`GilrsInputBackend` additions (the legacy lowest-ID `poll_axes` path is unchanged):

- `select_device(&DeviceIdentity)` — explicit deterministic selection with typed errors.
- `poll_raw_axes()` — `Ok(Some(state))` while selected, `Ok(None)` after device loss,
  `Err(RequestedDeviceNotFound)` when nothing was selected.
- `explicit_device()` — metadata of the selected device while connected.

Added `InputError` variants: `NoDevices`, `RequestedDeviceNotFound`, `AmbiguousDeviceMatch`,
`InvalidControllerProfile`, `UnsupportedProfileVersion`, `NonFiniteCalibration`,
`InvalidCalibrationOrder`, `DegenerateCalibrationSpan`, `DuplicateAxisAssignment`,
`UnavailableHardwareAxis`, `UnknownHardwareAxis`.

## Hardware validation status

Real RadioMaster TX16S validation has NOT happened yet. OA1A is verified exclusively by
hardware-independent deterministic tests over synthetic device lists and calibration values.
Whether gilrs' SDL mapping database assigns the TX16S physical sticks/sliders to the gilrs
axes used by a profile is an open hardware question for OA1B and later slices; if an axis is
not reported, the platform surfaces `UnavailableHardwareAxis` rather than guessing.
