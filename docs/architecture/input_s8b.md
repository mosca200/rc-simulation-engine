# S8B Input Abstraction and Live Replay Recording

## Boundary

S8B introduces `crates/platform` as the boundary between operating-system input and normalized
simulation commands:

```text
gilrs or keyboard state
        |
        v
raw ControllerAxes
        |
        v
InputMapping: finite validation, clamp, deadzone, rescale, inversion
        |
        v
PilotInput
        |
        v
existing rates, expo, mixer, servos, aerodynamics and propulsion
```

The platform crate depends only on `gilrs`, `sim_core`, and `thiserror`. It does not depend on
`renderer`, `wgpu`, `winit`, `aircraft`, `model`, or `replay`. The renderer does not own input;
`RenderApplication` owns the backend, persistent input state, aircraft simulation, and optional
replay recorder.

## gilrs backend

The first hardware backend is `gilrs 0.11.2`. On Windows the workspace selects its `xinput`
feature with default features disabled. This permits the headless `input list` path without the
focus-window requirement of the default Windows Gaming Input backend.

`GilrsInputBackend`:

- initializes without force feedback;
- disables gilrs default deadzone/jitter filters so normalization occurs exactly once;
- drains events to maintain cached state and hotplug information;
- enumerates connected devices with ID, name, UUID, vendor ID, and product ID;
- deterministically selects the connected controller with the lowest gilrs ID;
- returns `None` when no controller is connected.

The backend uses only standard gilrs axes. It contains no Radiomaster-specific or vendor HID
protocol.

## Gamepad mapping

The default mapping is:

| gilrs axis | Pilot channel | Deadzone | Inverted |
|---|---|---:|---:|
| `LeftStickX` | roll | 0.08 | no |
| `LeftStickY` | pitch | 0.08 | yes |
| `RightStickX` | yaw | 0.08 | no |
| `RightStickY` | throttle | 0.00 | yes |

Final normalized ranges are:

```text
roll     [-1, 1]
pitch    [-1, 1]
yaw      [-1, 1]
throttle [ 0, 1]
```

Raw finite values are clamped to `[-1, 1]`. Non-finite values return `InputError` and never reach
`PilotInput`.

## Continuous deadzone and inversion

For a centered raw axis `x` and deadzone `d`:

```text
clamped = clamp(x, -1, 1)
directed = inverted ? -clamped : clamped

if abs(directed) <= d:
    normalized = 0
else:
    normalized = sign(directed) * (abs(directed) - d) / (1 - d)
```

The boundary is continuous: approaching `d` from outside approaches zero, while the remaining
travel is rescaled to the full normalized range. Valid deadzones are finite values in `[0, 1)`.
There is no smoothing, temporal filter, calibration curve, rate, or expo in the platform layer.

## Throttle conversion

After centered-axis normalization and optional inversion, throttle is mapped explicitly:

```text
throttle = (normalized_axis + 1) / 2
```

Therefore raw `-1` maps to `0`, raw `+1` maps to `1`, and inversion exchanges the endpoints. The
default hardware throttle mapping is inverted because many devices expose the physical lever in
the opposite direction.

## Keyboard fallback

When no controller is selected, `InputState` samples persistent keyboard state:

```text
A / D -> roll -1 / +1
W / S -> pitch +1 / -1
Q / E -> yaw -1 / +1
R / F -> throttle increase / decrease
Escape -> application exit
```

Opposite simultaneous keys cancel to zero. Releasing a key removes its contribution. Keyboard
throttle starts from the render `--throttle` value and changes at `0.5` normalized units per
second, integrated using only the fixed physics `dt`.

Keyboard commands become ordinary `PilotInput` values. They never write servo positions or
aerodynamic deflections directly.

## Sampling and fixed step

The gilrs cache is polled once before processing a render frame's physics backlog. Every individual
physics step then calls `InputState::sample(0.002)` and receives an explicit `PilotInput`.

```text
wall-clock frame delta
        -> fixed-step accumulator decides count only

for each scheduled step:
    sample current input state with dt = 0.002 s
    AircraftSimulation step with that PilotInput
```

Frame delta never enters controls or physics. If one render frame schedules multiple steps, each
step is sampled deterministically; keyboard throttle therefore advances once per 2 ms physics
step. The simulation remains 500 Hz.

## Live S8A replay recording

`rcsim-app render` accepts:

```text
--record-replay PATH
```

The application creates `AircraftReplayRecorder` before step zero. For every physics step it:

1. samples one normalized `PilotInput`;
2. obtains the current pre-step index;
3. calls `AircraftReplayRecorder::record` with that exact input and simulation;
4. lets the recorder perform the aircraft step;
5. stores the hash of the resulting post-step snapshot.

No trajectory or complete snapshot is stored. No file I/O occurs per step. On normal
`CloseRequested`, Escape, or event-loop shutdown, the recorder is finalized, serialized once, and
written to the requested path. The result is accepted directly by `rcsim-app replay verify`.

## Headless device listing and no-device behavior

```powershell
cargo run -p rcsim-app --release -- input list
```

This route initializes only gilrs, prints connected device metadata, and exits. It does not create
a winit event loop, window, surface, GPU adapter, or renderer. Zero connected devices is normal:

```text
mode: input-list
devices: 0
```

In render mode, absence or disconnection selects keyboard fallback without panic or blocking.
Automated tests use synthetic axes and keyboard events; they do not require physical hardware.

## Future backends and exclusions

The pure `ControllerAxes`, `InputMapping`, `InputState`, and `InputSource` boundary allows a future
dedicated HID backend to provide raw axes without changing physics. S8B does not implement
Radiomaster TX16S HID, vendor protocols, Bluetooth, profiles, calibration UI, force feedback,
hardware latency benchmarks, VR controllers, networking, terrain, collision, S9, or physics
changes.
