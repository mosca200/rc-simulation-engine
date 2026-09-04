# Propulsion bench

## Purpose and authority

The propulsion bench is a deterministic engineering view of the propulsion
model already authored in an aircraft JSON file. It calls
`sim_core::evaluate_electric_propulsion_with_source`, the same pure production
evaluator used by every `AircraftSimulation` RK4 stage. The CLI contains no
independent battery, ESC, motor, shaft, propeller, `Ct`, or `Cq` equations.

Results are computed model output. They are not measurements, manufacturer
data, or evidence that a modeled aircraft matches a physical aircraft.
Synthetic and provisional models retain those limitations when evaluated.

## Commands

Single operating points:

```text
rcsim-app propulsion bench --model models/acro_electric_01/model.json --throttle 1.0 --airspeed-mps 0
rcsim-app propulsion bench --model models/acro_electric_01/model.json --throttle 0.5 --airspeed-mps 15 --format json
```

If only `--throttle` is supplied, airspeed defaults to zero. If only
`--airspeed-mps` is supplied, throttle defaults to one.

With no operating-point options, the bench evaluates this matrix in throttle
major, then airspeed order:

- throttle: `0`, `0.25`, `0.5`, `0.75`, `1`
- axial airspeed: `0`, `5`, `10`, `15`, `20`, `25 m/s`

An explicit inclusive sweep can be selected with:

```text
rcsim-app propulsion bench --model models/acro_electric_01/model.json \
  --throttle-start 0.2 --throttle-end 1.0 --throttle-step 0.2 \
  --airspeed-start-mps 0 --airspeed-end-mps 30 --airspeed-step-mps 5
```

Single-point and sweep options cannot be mixed. Throttle must be finite and in
`[0,1]`; airspeed must be finite and non-negative. Range steps must be finite
and positive. Endpoints are included when exactly reachable by the selected
step.

## Input frame and atmosphere

`--airspeed-mps` is a non-negative axial inflow along the modeled propeller's
local `+X` axis. The diagnostic state has identity aircraft attitude, zero body
angular velocity, and zero wind. This keeps the requested airspeed equal to the
production evaluator's propeller-axis inflow even for an oriented propeller.
The current bench atmosphere is fixed at `1.225 kg/m^3` and is reported in all
machine-readable output.

Positive thrust is propeller-local `+X`. Shaft speed and RPM are non-negative
magnitudes under the existing one-quadrant motor convention. Propeller reaction
torque and spin direction remain embodied in the production body wrench; the
reported `motor_torque_nm` and `propeller_torque_nm` are positive shaft/load
magnitudes.

## Reported quantities

Every point reports:

- throttle command, requested airspeed, and actual propeller axial inflow;
- battery open-circuit voltage, loaded terminal voltage/current, and terminal
  electrical power;
- ESC series-loss power;
- motor voltage/current and motor electrical input power;
- shaft speed in rad/s and RPM;
- motor torque, propeller load torque, thrust, advance ratio `J`, `Ct`, and
  `Cq`;
- mechanical shaft power and useful propulsive power;
- drive and propulsive efficiency when their denominators are meaningful;
- shaft-speed-map bracket, interpolation fraction, and clamp/range status.

The current production evaluator does not expose a separate ESC duty or ideal
PWM output-voltage diagnostic. The reported throttle is the production ESC
command and `motor_voltage_v` is the voltage after the modeled ESC series loss;
the bench does not reconstruct an unreported internal quantity.

Power and efficiencies are defined as:

```text
mechanical_shaft_power_w = motor_torque_nm * shaft_speed_rad_s
useful_propulsive_power_w = thrust_n * axial_inflow_mps
drive_efficiency = mechanical_shaft_power_w / battery_terminal_electrical_power_w
propulsive_efficiency = useful_propulsive_power_w / mechanical_shaft_power_w
```

An efficiency is `N/A` in table output, empty in CSV, and `null` in JSON when
its denominator is zero. Propulsive efficiency is also unavailable at static
thrust. Useful propulsive power at zero airspeed is exactly zero. No output
contains NaN or infinity.

## Output formats

`--format table` is the human-readable default. `--format csv` emits a stable
header and one row per operating point. `--format json` emits schema version 1,
model identity, the physics fingerprint, coefficient-source kind, density, and
the ordered operating-point array. Field and row ordering are deterministic;
timestamps, wall-clock measurements, and host metadata are absent.

`--output PATH` writes the selected representation to a new file. The command
uses create-new semantics and refuses to overwrite an existing file.

## Coefficient sources and determinism

Both fixed coefficient tables and shaft-speed-dependent maps flow through the
same production source enum and evaluator. Advance-ratio table sampling is
piecewise linear, exact at authored knots, and clamped to the first/last `J`
sample outside its range. Shaft-speed maps sample both `J` tables at the
candidate shaft speed during the production equilibrium solve, interpolate
between nodes, and clamp outside the authored shaft-speed range. The report
exposes the resulting map bracket and status.

Identical validated model bytes, CLI values, build, and platform produce the
same ordered numeric report. JSON and CSV are suitable for regression capture;
the table intentionally rounds values for reading.

## Limitations

The bench measures the current quasi-static model only. It adds no battery
state-of-charge depletion, thermal behavior, rotor acceleration, new ESC
control physics, blade-element theory, P-factor, propwash, or calibration.
Efficiency reflects the modeled loss chain and authored coefficient data, not
a physical dynamometer result. In particular, outputs from
`models/sig_kadet_lt40_egv/model.json` are provisional computed values and must
not be described as measured LT-40 performance.
