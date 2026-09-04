//! Hardware-independent tests for the versioned controller profile and the
//! calibrated raw-state-to-`PilotInput` path.

use platform::{
    CONTROLLER_PROFILE_SCHEMA_VERSION, CenteredAxisProfile, CenteredCalibration, Control,
    ControllerProfile, DeviceIdentity, HardwareAxis, InputError, ProfileAxes, RawControllerState,
    ThrottleAxisProfile, ThrottleCalibration,
};

fn sample_device() -> DeviceIdentity {
    DeviceIdentity::new(
        "Test Transmitter",
        Some("00112233445566778899aabbccddeeff".to_owned()),
        Some(0x1234),
        Some(0x5678),
    )
}

fn sample_axes() -> ProfileAxes {
    ProfileAxes::new(
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickX,
            CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, 0.06).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickY,
            CenteredCalibration::new(Control::Pitch, -0.75, 0.05, 0.8, true, 0.05).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::RightStickX,
            CenteredCalibration::new(Control::Yaw, -0.5, 0.0, 0.5, false, 0.1).unwrap(),
        ),
        ThrottleAxisProfile::new(
            HardwareAxis::RightStickY,
            ThrottleCalibration::new(-1.0, 1.0, true).unwrap(),
        ),
    )
}

fn sample_profile() -> ControllerProfile {
    ControllerProfile::new(sample_device(), sample_axes()).unwrap()
}

#[test]
fn profile_json_round_trips_and_serialization_is_stable() {
    let profile = sample_profile();
    let first_json = profile.to_json().unwrap();
    let decoded = ControllerProfile::from_json(&first_json).unwrap();
    assert_eq!(decoded, profile);
    assert_eq!(decoded.to_json().unwrap(), first_json);
    assert_eq!(decoded.schema_version(), CONTROLLER_PROFILE_SCHEMA_VERSION);
    assert_eq!(decoded.device().name(), "Test Transmitter");
    assert_eq!(
        decoded.device().uuid(),
        Some("00112233445566778899aabbccddeeff")
    );
    assert_eq!(decoded.device().vendor_id(), Some(0x1234));
    assert_eq!(decoded.device().product_id(), Some(0x5678));
}

#[test]
fn suggested_profile_shape_is_accepted() {
    let json = r#"
    {
      "schema_version": 1,
      "device": {
        "name": "Test Transmitter",
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
        "pitch": {
          "source": "left_stick_y",
          "raw_min": -1.0,
          "raw_center": 0.0,
          "raw_max": 1.0,
          "inverted": true,
          "deadzone": 0.05
        },
        "yaw": {
          "source": "right_stick_x",
          "raw_min": -1.0,
          "raw_center": 0.0,
          "raw_max": 1.0,
          "inverted": false,
          "deadzone": 0.05
        },
        "throttle": {
          "source": "right_stick_y",
          "raw_min": -1.0,
          "raw_max": 1.0,
          "inverted": true
        }
      }
    }
    "#;
    let profile = ControllerProfile::from_json(json).unwrap();
    assert_eq!(profile.axes().roll().source(), HardwareAxis::LeftStickX);
    assert!(profile.axes().pitch().calibration().inverted());
    assert_eq!(profile.axes().throttle().calibration().raw_min(), -1.0);
    assert!(profile.axes().yaw().calibration().deadzone() > 0.0);
}

#[test]
fn unsupported_schema_versions_are_rejected() {
    let json = sample_profile().to_json().unwrap();
    for version in ["0", "2", "99"] {
        let mutated = json.replacen(
            "\"schema_version\": 1",
            &format!("\"schema_version\": {version}"),
            1,
        );
        assert_ne!(mutated, json);
        let error = ControllerProfile::from_json(&mutated).unwrap_err();
        assert_eq!(
            error,
            InputError::UnsupportedProfileVersion {
                found: version.parse().unwrap(),
                supported: CONTROLLER_PROFILE_SCHEMA_VERSION,
            }
        );
    }
}

#[test]
fn missing_schema_version_is_rejected_as_unsupported() {
    let json = sample_profile()
        .to_json()
        .unwrap()
        .replacen("\"schema_version\": 1,\n", "", 1);
    let error = ControllerProfile::from_json(&json).unwrap_err();
    assert_eq!(
        error,
        InputError::UnsupportedProfileVersion {
            found: 0,
            supported: CONTROLLER_PROFILE_SCHEMA_VERSION,
        }
    );
}

#[test]
fn duplicate_hardware_axis_assignments_are_rejected() {
    let duplicated = ProfileAxes::new(
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickX,
            CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickY,
            CenteredCalibration::new(Control::Pitch, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickX,
            CenteredCalibration::new(Control::Yaw, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        ThrottleAxisProfile::new(
            HardwareAxis::RightStickY,
            ThrottleCalibration::new(-1.0, 1.0, false).unwrap(),
        ),
    );
    assert_eq!(
        ControllerProfile::new(sample_device(), duplicated),
        Err(InputError::DuplicateAxisAssignment {
            axis: HardwareAxis::LeftStickX,
        })
    );

    let throttle_overlap = ProfileAxes::new(
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickX,
            CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::LeftStickY,
            CenteredCalibration::new(Control::Pitch, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        CenteredAxisProfile::new(
            HardwareAxis::RightStickX,
            CenteredCalibration::new(Control::Yaw, -1.0, 0.0, 1.0, false, 0.0).unwrap(),
        ),
        ThrottleAxisProfile::new(
            HardwareAxis::LeftStickX,
            ThrottleCalibration::new(-1.0, 1.0, false).unwrap(),
        ),
    );
    assert_eq!(
        ControllerProfile::new(sample_device(), throttle_overlap),
        Err(InputError::DuplicateAxisAssignment {
            axis: HardwareAxis::LeftStickX,
        })
    );
}

#[test]
fn duplicate_axis_assignment_is_rejected_when_decoding_json() {
    let json = sample_profile().to_json().unwrap().replacen(
        "\"source\": \"right_stick_x\"",
        "\"source\": \"left_stick_x\"",
        1,
    );
    assert_eq!(
        ControllerProfile::from_json(&json),
        Err(InputError::DuplicateAxisAssignment {
            axis: HardwareAxis::LeftStickX,
        })
    );
}

#[test]
fn invalid_calibration_inside_json_is_rejected_with_typed_errors() {
    let invalid_order =
        sample_profile()
            .to_json()
            .unwrap()
            .replacen("\"raw_min\": -1.0", "\"raw_min\": 0.5", 1);
    assert_eq!(
        ControllerProfile::from_json(&invalid_order),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Roll
        })
    );

    let invalid_deadzone =
        sample_profile()
            .to_json()
            .unwrap()
            .replacen("\"deadzone\": 0.06", "\"deadzone\": 1.5", 1);
    assert_eq!(
        ControllerProfile::from_json(&invalid_deadzone),
        Err(InputError::InvalidDeadzone)
    );
}

#[test]
fn malformed_profile_json_is_rejected_as_invalid_profile() {
    let cases = [
        "{",
        "{}",
        r#"{"schema_version": 1}"#,
        r#"{"schema_version": "one", "device": {}, "axes": {}}"#,
        r#"{
            "schema_version": 1,
            "device": {"name": "X", "uuid": null, "vendor_id": null, "product_id": null},
            "axes": {
                "roll": {"source": "stick_99", "raw_min": -1.0, "raw_center": 0.0, "raw_max": 1.0, "inverted": false, "deadzone": 0.0},
                "pitch": {"source": "left_stick_y", "raw_min": -1.0, "raw_center": 0.0, "raw_max": 1.0, "inverted": false, "deadzone": 0.0},
                "yaw": {"source": "right_stick_x", "raw_min": -1.0, "raw_center": 0.0, "raw_max": 1.0, "inverted": false, "deadzone": 0.0},
                "throttle": {"source": "right_stick_y", "raw_min": -1.0, "raw_max": 1.0, "inverted": false}
            }
        }"#,
    ];
    for case in cases {
        let error = ControllerProfile::from_json(case).unwrap_err();
        assert!(
            matches!(error, InputError::InvalidControllerProfile(_)),
            "expected InvalidControllerProfile for {case}, got {error:?}"
        );
    }
}

#[test]
fn calibrated_raw_state_maps_to_expected_pilot_input() {
    let profile = sample_profile();
    let mut state = RawControllerState::new();
    state.insert(HardwareAxis::LeftStickX, 0.5).unwrap();
    state.insert(HardwareAxis::LeftStickY, 0.25).unwrap();
    state.insert(HardwareAxis::RightStickX, 0.25).unwrap();
    state.insert(HardwareAxis::RightStickY, -1.0).unwrap();
    let input = profile.to_pilot_input(&state).unwrap();
    assert!((input.roll() - (0.5 - 0.06) / (1.0 - 0.06)).abs() < 1.0e-12);
    let pitch_normalized = (0.25 - 0.05) / (0.8 - 0.05);
    let expected_pitch = -((pitch_normalized - 0.05) / (1.0 - 0.05));
    assert!((input.pitch() - expected_pitch).abs() < 1.0e-12);
    assert!((input.yaw() - (0.5 - 0.1) / (1.0 - 0.1)).abs() < 1.0e-12);
    assert_eq!(input.throttle(), 1.0);
    assert!(input.is_valid());
}

#[test]
fn no_resulting_pilot_input_is_nan_or_non_finite() {
    let profile = sample_profile();
    let samples = [
        -1.0e6, -2.0, -1.0, -0.5, -0.1, 0.0, 0.1, 0.5, 1.0, 2.0, 1.0e6,
    ];
    for roll_raw in samples {
        for pitch_raw in samples {
            for yaw_raw in samples {
                for throttle_raw in samples {
                    let mut state = RawControllerState::new();
                    state.insert(HardwareAxis::LeftStickX, roll_raw).unwrap();
                    state.insert(HardwareAxis::LeftStickY, pitch_raw).unwrap();
                    state.insert(HardwareAxis::RightStickX, yaw_raw).unwrap();
                    state
                        .insert(HardwareAxis::RightStickY, throttle_raw)
                        .unwrap();
                    let input = profile.to_pilot_input(&state).unwrap();
                    assert!(input.is_valid());
                    assert!(input.roll().is_finite());
                    assert!(input.pitch().is_finite());
                    assert!(input.yaw().is_finite());
                    assert!(input.throttle().is_finite());
                }
            }
        }
    }
}

#[test]
fn missing_requested_hardware_axis_is_an_error_not_zero() {
    let profile = sample_profile();
    let mut state = RawControllerState::new();
    state.insert(HardwareAxis::LeftStickX, 0.0).unwrap();
    assert_eq!(
        profile.to_pilot_input(&state),
        Err(InputError::UnavailableHardwareAxis {
            axis: HardwareAxis::LeftStickY,
        })
    );

    let mut no_throttle = RawControllerState::new();
    for axis in [
        HardwareAxis::LeftStickX,
        HardwareAxis::LeftStickY,
        HardwareAxis::RightStickX,
    ] {
        no_throttle.insert(axis, 0.0).unwrap();
    }
    assert_eq!(
        profile.to_pilot_input(&no_throttle),
        Err(InputError::UnavailableHardwareAxis {
            axis: HardwareAxis::RightStickY,
        })
    );
}

#[test]
fn raw_state_rejects_non_finite_values_before_mapping() {
    let mut state = RawControllerState::new();
    for bad in [f64::NAN, f64::INFINITY] {
        assert_eq!(
            state.insert(HardwareAxis::LeftStickX, bad),
            Err(InputError::NonFiniteRawAxis {
                axis: "left_stick_x",
            })
        );
    }
}
