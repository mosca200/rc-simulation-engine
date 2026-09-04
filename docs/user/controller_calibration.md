# Controller calibration — Operational Alpha OA1

OA1 supports generic USB game controllers through `gilrs`. It does not assume a transmitter
brand, stick mode, USB VID/PID, or hardware-axis layout. A controller profile records the stable
device identity, four explicit hardware-axis assignments, measured endpoints, centered positions,
inversion choices, and centered-control deadzone.

The RadioMaster TX16S workflow below is prepared for a real Windows test, but the TX16S is **not
validated** until that physical test has been run and its result recorded.

## Input modes and safety policy

Without `--controller-profile`, `rcsim-app render` keeps the OA1P legacy policy: logical gilrs axes
are passed through `InputMapping`, and the keyboard is the fallback when no controller is present.

With `--controller-profile`, the viewer is fail-closed. It loads and validates the JSON once,
matches exactly the recorded `DeviceIdentity`, polls only that device's raw axes, and maps them
through `ControllerProfile` to `PilotInput`. It never falls back silently to another controller or
to the keyboard. On disconnect, roll, pitch, yaw, and throttle are immediately set to zero and the
message `Controller disconnected — controls neutralized.` is printed once. Only the same identity
can reconnect; successful recovery prints `Controller reconnected.` once.

## Windows manual test with a TX16S

Run all commands from the repository root in PowerShell.

1. Configure the TX16S USB connection as a joystick/game controller, then connect it by USB.
2. Enumerate devices:

   ```powershell
   cargo run -p rcsim-app --release -- controller list
   ```

3. Identify the TX16S by its reported name, UUID, vendor ID, product ID, and session ID. Do not
   assume a particular ID or axis layout.
4. Inspect the axes actually exposed by that device, replacing `ID` with its session ID:

   ```powershell
   cargo run -p rcsim-app --release -- controller monitor --raw --device-id ID
   ```

5. Move every stick through full travel. Record which `HardwareAxis` names change. Missing axes are
   not synthesized as zeros.
6. Start interactive calibration. The default centered-control deadzone is `0.05`; override it only
   with a deliberate measured value:

   ```powershell
   cargo run -p rcsim-app --release -- controller calibrate --output tx16s.json
   ```

7. Select the device explicitly by its displayed session ID. During axis discovery, move the
   controls and press ENTER when ready. Assign roll, pitch, yaw, and throttle to the observed axis
   names. The procedure does not assume Mode 1 or Mode 2.
8. For roll, pitch, and yaw, move through full travel, press ENTER, release to center, and press
   ENTER again. For throttle, move through full travel and press ENTER. Answer the inversion prompt
   independently for every control.
9. Inspect `tx16s.json`. Confirm `schema_version` is `1`, the identity describes the selected unit,
   all four `source` values are distinct, and the measured ranges are plausible.
10. Launch the viewer with the profile:

    ```powershell
    cargo run -p rcsim-app --release -- render --controller-profile tx16s.json
    ```

11. Confirm startup diagnostics show the profile path, schema, requested controller, matched
    controller, and `Input mode: calibrated controller profile`.
12. Verify roll, pitch, yaw, and throttle directions and full travel in flight.
13. Disconnect the USB cable. Confirm the one-shot neutralization message appears and that no stale
    commands remain active.
14. Reconnect the same TX16S. Confirm the one-shot reconnect message appears and calibrated input
    resumes.
15. If available, connect a different controller while the TX16S is absent. Confirm it does not
    substitute for the requested TX16S.

Keep the command line, Windows version, USB mode, controller firmware, generated profile, and test
result with the manual evidence. Until this checklist is physically completed, OA1 proves the
software path and safety policy only—not TX16S hardware compatibility.

## Expected failures

- A missing profile path reports the path and operating-system read error.
- Malformed JSON is reported as an invalid controller profile.
- Unsupported schema versions, invalid calibration ranges, and duplicate axis assignments are
  rejected during deserialization.
- No match reports that the requested controller was not found. Multiple identity matches report
  ambiguity; the viewer does not choose one implicitly.
- A profile that assigns an axis not exposed by the matched device is rejected rather than treating
  the missing axis as zero.
