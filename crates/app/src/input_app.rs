use platform::{GilrsInputBackend, InputError};

pub fn run_input_list() -> Result<(), InputError> {
    let backend = GilrsInputBackend::new()?;
    let devices = backend.devices();
    println!("RC Simulation Engine");
    println!("mode: input-list");
    println!("devices: {}", devices.len());
    for device in devices {
        println!(
            "device: id={} name={:?} uuid={} vendor_id={} product_id={}",
            device.id(),
            device.name(),
            device.uuid(),
            optional_hex(device.vendor_id()),
            optional_hex(device.product_id())
        );
    }
    Ok(())
}

fn optional_hex(value: Option<u16>) -> String {
    value.map_or_else(|| "unknown".to_owned(), |value| format!("0x{value:04x}"))
}
