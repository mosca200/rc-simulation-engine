//! Hardware-independent device matching tests over synthetic device lists.

use platform::{DeviceIdentity, InputError, match_device};

fn identity(
    name: &str,
    uuid: Option<&str>,
    vendor_id: Option<u16>,
    product_id: Option<u16>,
) -> DeviceIdentity {
    DeviceIdentity::new(name, uuid.map(str::to_owned), vendor_id, product_id)
}

#[test]
fn exact_uuid_match_wins_regardless_of_other_metadata() {
    let candidates = [
        identity("Alpha", Some("aaaa-1111"), Some(1), Some(2)),
        identity("Beta", Some("bbbb-2222"), Some(1), Some(2)),
    ];
    let target = identity("Gamma", Some("BBBB-2222"), None, None);
    assert_eq!(match_device(&target, &candidates), Ok(1));
}

#[test]
fn usable_uuid_is_decisive_and_never_falls_back() {
    let candidates = [
        identity("Alpha", Some("aaaa-1111"), Some(1), Some(2)),
        identity("Beta", Some("bbbb-2222"), Some(1), Some(2)),
    ];
    let target = identity("Beta", Some("aaaa-1111"), Some(1), Some(2));
    assert_eq!(match_device(&target, &candidates), Ok(0));

    let missing = identity("Alpha", Some("cccc-3333"), Some(1), Some(2));
    assert_eq!(
        match_device(&missing, &candidates),
        Err(InputError::RequestedDeviceNotFound)
    );
}

#[test]
fn empty_uuid_falls_back_to_vendor_product_name() {
    let candidates = [
        identity("Alpha", Some(""), Some(1), Some(2)),
        identity("Beta", None, Some(1), Some(2)),
    ];
    let target = identity("Beta", Some("   "), Some(1), Some(2));
    assert_eq!(match_device(&target, &candidates), Ok(1));
}

#[test]
fn vendor_product_name_fallback_matches_exact_name() {
    let candidates = [
        identity("TX A", None, Some(0x1234), Some(0x5678)),
        identity("TX B", None, Some(0x1234), Some(0x5678)),
    ];
    let target = identity("TX B", None, Some(0x1234), Some(0x5678));
    assert_eq!(match_device(&target, &candidates), Ok(1));

    let wrong_ids = identity("TX A", None, Some(0x9999), Some(0x5678));
    assert_eq!(
        match_device(&wrong_ids, &candidates),
        Err(InputError::RequestedDeviceNotFound)
    );
    let wrong_name = identity("TX C", None, Some(0x1234), Some(0x5678));
    assert_eq!(
        match_device(&wrong_name, &candidates),
        Err(InputError::RequestedDeviceNotFound)
    );
}

#[test]
fn name_only_fallback_requires_an_unambiguous_match() {
    let candidates = [
        identity("Stick One", None, None, None),
        identity("Stick Two", None, None, None),
    ];
    let target = identity("Stick Two", None, None, None);
    assert_eq!(match_device(&target, &candidates), Ok(1));

    let duplicated = [
        identity("Stick", None, None, None),
        identity("Stick", None, None, None),
    ];
    let ambiguous = identity("Stick", None, None, None);
    assert_eq!(
        match_device(&ambiguous, &duplicated),
        Err(InputError::AmbiguousDeviceMatch { candidates: 2 })
    );

    let empty_name = identity("", None, None, None);
    assert_eq!(
        match_device(&empty_name, &candidates),
        Err(InputError::RequestedDeviceNotFound)
    );
}

#[test]
fn ambiguous_matches_are_rejected() {
    let candidates = [
        identity("Same", None, Some(1), Some(1)),
        identity("Same", None, Some(1), Some(1)),
    ];
    let target = identity("Same", None, Some(1), Some(1));
    assert_eq!(
        match_device(&target, &candidates),
        Err(InputError::AmbiguousDeviceMatch { candidates: 2 })
    );

    let duplicated_uuids = [
        identity("A", Some("uuid-x"), None, None),
        identity("B", Some("UUID-X"), None, None),
    ];
    let by_uuid = identity("C", Some("uuid-x"), None, None);
    assert_eq!(
        match_device(&by_uuid, &duplicated_uuids),
        Err(InputError::AmbiguousDeviceMatch { candidates: 2 })
    );
}

#[test]
fn missing_matches_are_rejected() {
    let candidates = [identity("Present", Some("uuid-1"), Some(1), Some(1))];
    let absent = identity("Absent", None, Some(9), Some(9));
    assert_eq!(
        match_device(&absent, &candidates),
        Err(InputError::RequestedDeviceNotFound)
    );
    assert_eq!(match_device(&absent, &[]), Err(InputError::NoDevices));
}

#[test]
fn synthetic_matches_are_deterministic_across_repeated_calls() {
    let candidates = [
        identity("Alpha", Some("aaaa"), Some(1), Some(2)),
        identity("Beta", Some("bbbb"), Some(3), Some(4)),
        identity("Gamma", Some("cccc"), Some(5), Some(6)),
    ];
    let target = identity("Anything", Some("cccc"), None, None);
    for _ in 0..10 {
        assert_eq!(match_device(&target, &candidates), Ok(2));
    }
}
