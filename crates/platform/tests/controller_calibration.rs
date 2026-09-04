//! Hardware-independent tests for the deterministic calibration math.

use platform::{
    CenteredCalibration, Control, InputError, MIN_CALIBRATION_SPAN, ThrottleCalibration,
};

fn centered(
    raw_min: f64,
    raw_center: f64,
    raw_max: f64,
    inverted: bool,
    deadzone: f64,
) -> CenteredCalibration {
    CenteredCalibration::new(
        Control::Roll,
        raw_min,
        raw_center,
        raw_max,
        inverted,
        deadzone,
    )
    .unwrap()
}

#[test]
fn centered_min_maps_to_minus_one_exactly() {
    assert_eq!(
        centered(-1.0, 0.0, 1.0, false, 0.0).apply(-1.0, Control::Roll),
        Ok(-1.0)
    );
    assert_eq!(
        centered(-0.75, 0.1, 0.6, false, 0.15).apply(-0.75, Control::Roll),
        Ok(-1.0)
    );
}

#[test]
fn centered_center_maps_to_zero_exactly() {
    assert_eq!(
        centered(-1.0, 0.0, 1.0, false, 0.0).apply(0.0, Control::Roll),
        Ok(0.0)
    );
    assert_eq!(
        centered(-0.75, 0.1, 0.6, false, 0.15).apply(0.1, Control::Roll),
        Ok(0.0)
    );
}

#[test]
fn centered_max_maps_to_plus_one_exactly() {
    assert_eq!(
        centered(-1.0, 0.0, 1.0, false, 0.0).apply(1.0, Control::Roll),
        Ok(1.0)
    );
    assert_eq!(
        centered(-0.75, 0.1, 0.6, false, 0.15).apply(0.6, Control::Roll),
        Ok(1.0)
    );
}

#[test]
fn asymmetric_centered_range_normalizes_each_half_span_independently() {
    let calibration = centered(-0.8, 0.1, 0.9, false, 0.0);
    assert_eq!(calibration.apply(-0.8, Control::Roll), Ok(-1.0));
    assert_eq!(calibration.apply(0.1, Control::Roll), Ok(0.0));
    assert_eq!(calibration.apply(0.9, Control::Roll), Ok(1.0));
    let negative_mid = calibration.apply(-0.35, Control::Roll).unwrap();
    let positive_mid = calibration.apply(0.5, Control::Roll).unwrap();
    assert!((negative_mid + 0.5).abs() < 1.0e-12);
    assert!((positive_mid - 0.5).abs() < 1.0e-12);
}

#[test]
fn centered_inversion_flips_the_calibrated_output() {
    let calibration = centered(-1.0, 0.0, 1.0, true, 0.0);
    assert_eq!(calibration.apply(0.5, Control::Roll), Ok(-0.5));
    assert_eq!(calibration.apply(-0.25, Control::Roll), Ok(0.25));
    assert_eq!(calibration.apply(1.0, Control::Roll), Ok(-1.0));
    assert_eq!(calibration.apply(-1.0, Control::Roll), Ok(1.0));
    assert_eq!(calibration.apply(0.0, Control::Roll), Ok(0.0));
}

#[test]
fn centered_output_saturates_below_min() {
    let calibration = centered(-1.0, 0.0, 1.0, false, 0.0);
    assert_eq!(calibration.apply(-5.0, Control::Roll), Ok(-1.0));
    let with_deadzone = centered(-1.0, 0.0, 1.0, false, 0.2);
    assert_eq!(with_deadzone.apply(-42.0, Control::Roll), Ok(-1.0));
}

#[test]
fn centered_output_saturates_above_max() {
    let calibration = centered(-1.0, 0.0, 1.0, false, 0.0);
    assert_eq!(calibration.apply(3.0, Control::Roll), Ok(1.0));
    let with_deadzone = centered(-1.0, 0.0, 1.0, false, 0.2);
    assert_eq!(with_deadzone.apply(42.0, Control::Roll), Ok(1.0));
}

#[test]
fn deadzone_centered_on_calibration_center_is_exactly_zero() {
    let calibration = centered(-1.0, 0.0, 1.0, false, 0.2);
    for raw in [-0.2, -0.1, 0.0, 0.1, 0.2] {
        assert_eq!(calibration.apply(raw, Control::Roll), Ok(0.0));
    }
    let asymmetric = centered(-0.8, 0.1, 0.9, false, 0.25);
    assert_eq!(asymmetric.apply(0.1, Control::Roll), Ok(0.0));
}

#[test]
fn deadzone_boundary_is_continuous_and_rescaled_outside() {
    let calibration = centered(-1.0, 0.0, 1.0, false, 0.2);
    let epsilon = 1.0e-9;
    assert_eq!(calibration.apply(0.2, Control::Roll), Ok(0.0));
    let just_outside = calibration.apply(0.2 + epsilon, Control::Roll).unwrap();
    assert!(just_outside > 0.0);
    assert!(just_outside < 2.0e-9);
    assert!((calibration.apply(0.6, Control::Roll).unwrap() - 0.5).abs() < 1.0e-15);
    assert_eq!(calibration.apply(1.0, Control::Roll), Ok(1.0));
    let just_outside_negative = calibration.apply(-0.2 - epsilon, Control::Roll).unwrap();
    assert!(just_outside_negative < 0.0);
    assert!(just_outside_negative > -2.0e-9);
}

#[test]
fn throttle_min_maps_to_zero_and_max_maps_to_one() {
    let calibration = ThrottleCalibration::new(1000.0, 2000.0, false).unwrap();
    assert_eq!(calibration.apply(1000.0), Ok(0.0));
    assert_eq!(calibration.apply(2000.0), Ok(1.0));
    assert_eq!(calibration.apply(1500.0), Ok(0.5));
    assert_eq!(calibration.apply(500.0), Ok(0.0));
    assert_eq!(calibration.apply(2500.0), Ok(1.0));
}

#[test]
fn throttle_inversion_exchanges_idle_and_full_endpoints() {
    let calibration = ThrottleCalibration::new(1000.0, 2000.0, true).unwrap();
    assert_eq!(calibration.apply(1000.0), Ok(1.0));
    assert_eq!(calibration.apply(2000.0), Ok(0.0));
    assert_eq!(calibration.apply(1500.0), Ok(0.5));
    assert_eq!(calibration.apply(-500.0), Ok(1.0));
    assert_eq!(calibration.apply(5000.0), Ok(0.0));
}

#[test]
fn invalid_min_center_max_ordering_is_rejected() {
    assert_eq!(
        CenteredCalibration::new(Control::Roll, 0.5, 0.0, 1.0, false, 0.0),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Roll
        })
    );
    assert_eq!(
        CenteredCalibration::new(Control::Pitch, -1.0, 0.5, 0.5, false, 0.0),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Pitch
        })
    );
    assert_eq!(
        CenteredCalibration::new(Control::Yaw, 0.0, 0.0, 1.0, false, 0.0),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Yaw
        })
    );
    assert_eq!(
        CenteredCalibration::new(Control::Roll, 1.0, 0.0, -1.0, false, 0.0),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Roll
        })
    );
    assert_eq!(
        ThrottleCalibration::new(2.0, 1.0, false),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Throttle
        })
    );
    assert_eq!(
        ThrottleCalibration::new(1.0, 1.0, false),
        Err(InputError::InvalidCalibrationOrder {
            control: Control::Throttle
        })
    );
}

#[test]
fn non_finite_calibration_values_are_rejected() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            CenteredCalibration::new(Control::Roll, bad, 0.0, 1.0, false, 0.0),
            Err(InputError::NonFiniteCalibration {
                control: Control::Roll
            })
        );
        assert_eq!(
            CenteredCalibration::new(Control::Pitch, -1.0, bad, 1.0, false, 0.0),
            Err(InputError::NonFiniteCalibration {
                control: Control::Pitch
            })
        );
        assert_eq!(
            CenteredCalibration::new(Control::Yaw, -1.0, 0.0, bad, false, 0.0),
            Err(InputError::NonFiniteCalibration {
                control: Control::Yaw
            })
        );
        assert_eq!(
            ThrottleCalibration::new(bad, 1.0, false),
            Err(InputError::NonFiniteCalibration {
                control: Control::Throttle
            })
        );
        assert_eq!(
            ThrottleCalibration::new(-1.0, bad, false),
            Err(InputError::NonFiniteCalibration {
                control: Control::Throttle
            })
        );
    }
}

#[test]
fn invalid_deadzone_values_are_rejected() {
    for deadzone in [-0.1, 1.0, 2.0, f64::NAN, f64::INFINITY] {
        assert_eq!(
            CenteredCalibration::new(Control::Roll, -1.0, 0.0, 1.0, false, deadzone),
            Err(InputError::InvalidDeadzone)
        );
    }
}

#[test]
fn degenerate_calibration_spans_are_rejected() {
    assert_eq!(
        CenteredCalibration::new(Control::Roll, -1.0e-9, 0.0, 1.0e-9, false, 0.0),
        Err(InputError::DegenerateCalibrationSpan {
            control: Control::Roll,
            min_span: MIN_CALIBRATION_SPAN,
        })
    );
    assert_eq!(
        CenteredCalibration::new(Control::Pitch, -1.0, 1.0 - 1.0e-12, 1.0, false, 0.0),
        Err(InputError::DegenerateCalibrationSpan {
            control: Control::Pitch,
            min_span: MIN_CALIBRATION_SPAN,
        })
    );
    assert_eq!(
        ThrottleCalibration::new(0.0, 1.0e-9, false),
        Err(InputError::DegenerateCalibrationSpan {
            control: Control::Throttle,
            min_span: MIN_CALIBRATION_SPAN,
        })
    );
}

#[test]
fn non_finite_raw_values_are_rejected_at_apply_time() {
    let calibration = centered(-1.0, 0.0, 1.0, false, 0.0);
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert_eq!(
            calibration.apply(bad, Control::Roll),
            Err(InputError::NonFiniteRawAxis { axis: "roll" })
        );
    }
    let throttle = ThrottleCalibration::new(-1.0, 1.0, false).unwrap();
    assert_eq!(
        throttle.apply(f64::NAN),
        Err(InputError::NonFiniteRawAxis { axis: "throttle" })
    );
}
