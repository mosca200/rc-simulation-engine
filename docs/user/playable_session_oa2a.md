# OA2A playable session

`rcsim-app play` starts the existing manual-flight viewer through a playable-session preset. It does not own a second render or physics loop.

## Defaults

The `play` preset loads `models/acro_electric_01/model.json`, computes a physically supported stationary ground start, uses the flat terrain mode with the `FlyingField` scenery, selects a chase camera 3.0 m behind and 0.95 m above the aircraft, and starts at zero throttle. Physics continues to run at the existing fixed 500 Hz rate.

`rcsim-app render` retains its technical viewer defaults: airborne at 30 m and 18 m/s, throttle 0.55, no scenery, and the default pilot camera. Both commands accept the same explicit render options, which override their respective defaults.

## Input

Without `--controller-profile`, `play` retains the legacy keyboard/controller input behavior. With `--controller-profile PATH`, it uses the frozen OA1 calibrated-controller path: the profile is loaded once, the requested identity is matched, unavailable hardware waits with neutral input, disconnect fails closed to neutral, and the same device can reconnect during the session. Calibrated mode never silently falls back to keyboard input.

No controller-profile auto-discovery is performed.

## Reset

Press Backspace to reconstruct the initial flight state without recreating the window, GPU renderer, WGI backend, or loaded calibrated-controller state. The reset restores the rigid body, neutral servos, initial throttle, brake command, step index and simulation time, ground diagnostics, both render snapshots, fixed-step accumulator, and wall-clock timing baseline. A connected calibrated controller therefore keeps ownership and continues to work after reset.

If `--record-replay PATH` is active, Backspace is refused with a clear console message. Exit normally to finish and save that single uninterrupted recording.
