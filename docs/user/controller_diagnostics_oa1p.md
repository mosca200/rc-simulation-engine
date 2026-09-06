# Controller diagnostics (OA1P)

OA1P exposes the controller information and logical axes already available through the current
`gilrs` input backend. It does not add calibration or persistent controller profiles.

## Windows commands

From the repository root, build the application and list the controllers visible in the current
session:

```powershell
cargo build -p rcsim-app --release
cargo run -p rcsim-app --release -- controller list
```

The reported `session_id` is transient and is meaningful only for the current process/session.

Monitor the automatically selected controller at 10 Hz until interrupted with `Ctrl+C`:

```powershell
cargo run -p rcsim-app --release -- controller monitor
```

Use a bounded run when collecting a short diagnostic sample:

```powershell
cargo run -p rcsim-app --release -- controller monitor --samples 100
cargo run -p rcsim-app --release -- controller monitor --duration-seconds 30
```

If both bounds are supplied, monitoring stops when the first bound is reached. The commands do not
write controller state or calibration files.

With the Windows Gaming Input (WGI) backend, initial device discovery can arrive asynchronously.
`controller list`, `controller monitor --raw`, and `controller calibrate` therefore process gilrs
events for a bounded two-second discovery window before treating the controller set as empty.
The legacy monitor retains its continuous hot-plug observation. No Raw Input or custom HID path is
used by this diagnostic slice.

## Interpretation

The reported `roll`, `pitch`, `yaw`, and `throttle` values are **legacy logical axes /
pre-calibration**. They reflect the existing `gilrs` logical-axis mapping and must not be treated as
calibrated RC controls.

OA1A will add controller profiles, calibration, raw-axis assignment, and persistent device
matching. Those capabilities are intentionally outside OA1P. A physical Radiomaster TX16S has not
been manually validated by this automated slice.
