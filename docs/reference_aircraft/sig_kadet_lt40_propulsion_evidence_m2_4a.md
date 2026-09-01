# M2.4A — SIG KADET LT-40 EGV electric-propulsion evidence framework

## Scope and authority

M2.4A is a strict, off-runtime evidence layer. It records propulsion claims and source data without
creating an LT-40 `AircraftModel`, an `ElectricPropulsionConfig`, or a calibrated propulsion model.
The machine-readable artifact is
[`data/sig_kadet_lt40_egv_propulsion_evidence_v0.json`](data/sig_kadet_lt40_egv_propulsion_evidence_v0.json).
Its schema is `reference_aircraft_propulsion_evidence_v0` and its artifact kind is
`propulsion_evidence_not_runtime_configuration`.

The artifact supports null for unavailable measurements. Unknown is not encoded as zero and the
validator rejects zero where a present physical value must be positive. Every M2.4A evaluation
reports `runtime_ready = false`.

## Configuration evidence classes

Configuration claims have four non-interchangeable classes:

- `manufacturer_recommendation`: an airframe compatibility envelope, not a component identity;
- `historically_flight_tested_configuration`: a configuration reported as flown in historical
  manufacturer documentation, not proof of a later airframe installation;
- `specific_installed_configuration`: component identity tied to one identified airframe,
  operational configuration, and propulsion configuration; and
- `measured_configuration`: an identified installation with measurement provenance.

The campaign separates aircraft type identity (`manufacturer`, `family`, and `variant`) from the
nullable `physical_airframe_id`. A product variant is not a serial or identity for a particular
airframe. Installed or measured claims require a physical-airframe ID plus operational- and
propulsion-configuration IDs, must match all three campaign IDs, and must identify motor, ESC,
battery, and propeller. Motor, ESC, battery, propeller, and optional spinner records declare the
exact configuration claims to which they apply. Source and photograph registries are resolved
strictly; applicability is not inferred from a similar model name.

## SIG recommendation and historical configurations

The current SIG product page provides this manufacturer recommendation for the target EGV ARF:

| Item | Recommendation |
| --- | --- |
| Electric power | 500–800 W |
| Motor Kv | 800–1000 rpm/V |
| ESC rating | 50–75 A |
| LiPo battery | 3S–4S, 4000–5000 mAh |

The artifact stores this as `sig-current-recommendation-envelope` with no component IDs. It cannot
make `configuration_identified` true.

The SIG EGV manual separately reports a historically flown Himax HC3528-1000 motor, a 50 A Castle
Creations ESC, an APC 11x7E propeller, and either a 3S 4500 mAh or a 4S 4500 mAh LiPo battery. The
two battery alternatives are separate historical claims. Neither is promoted to the future
physical reference airframe.

No `specific_installed_configuration` or `measured_configuration` claim exists in the committed
template. Its `physical_airframe_id`, operational-configuration ID, propulsion-configuration ID,
and measurement date therefore remain null. Recommendation and historical claims also carry no
physical-airframe ID and cannot identify the future reference installation.

## Motor evidence

Only values already traced through the dossier and the Himax HC3528-1000 manufacturer manual are
transferred:

| Parameter | Evidence value |
| --- | ---: |
| Kv | 1000 rpm/V |
| Winding resistance `Rm` | 0.020 ohm |
| No-load current `Io` | 2.6 A |
| Motor mass | 0.197 kg |
| Diameter | 0.0352 m |
| Length | 0.0542 m |
| Shaft diameter | 0.005 m |
| Conditional maximum power | 450 W |
| Efficient-current range | 15–48 A |
| Short-duration maximum current | 68 A for 15 s |

These are manufacturer data applicable to the two historical claims. Efficiency is null, and the
record does not assert that this motor is installed in a future measured airframe. The 450 W motor
rating and SIG's 500–800 W airframe recommendation remain distinct source claims.

## ESC evidence

The historical manual identifies Castle Creations and a 50 A rating, but not a traceable exact
model. Cell compatibility, resistance, loss model, efficiency, switching frequency, and control
protocol remain null. M2.4A does not infer ideal behavior from the existing S5B ideal PWM ESC and
does not invent losses.

## Battery evidence and blockers

The historical alternatives preserve only what the manual identifies: LiPo chemistry, 3S or 4S,
and 4500 mAh (stored as the exact unit conversion 4.5 Ah). Manufacturer, pack model, nominal-voltage
definition, mass, internal resistance, voltage versus SOC/load, and temperature behavior remain
unknown. No generic LiPo sag curve or battery resistance is introduced.

A future physical campaign must identify the installed pack and measure or source its internal
resistance and voltage-under-load behavior with SOC and temperature metadata. Historical cell
count and capacity alone cannot satisfy battery evidence readiness.

## APC 11x7E manufacturer data

The exact APC manufacturer file is committed at
[`data/sources/APC_PER3_11x7E_v2022-0915.dat`](data/sources/APC_PER3_11x7E_v2022-0915.dat).
It was retrieved from <https://www.apcprop.com/files/PER3_11x7E.dat> on 2026-09-01:

- source version: `v2022-0915`;
- simulation date declared by APC: `09/22/2022`;
- byte count: 131,404;
- line count: 722;
- SHA-256: `f81055914654dd7f04a7fe337fb895f7332a9070813b368afcd8b048c9a17587`;
- 19 ordered RPM blocks from 1000 through 19000 rpm;
- 570 published `V`/`J` rows, of which 564 contain all 15 published columns; and
- 6 terminal rows where APC publishes `V` and `J` but leaves coefficient columns blank.

The parser follows definitions and column labels present in the file itself:

```text
J  = V / (n D)
Ct = T / (rho n^2 D^4)
Cp = P / (rho n^3 D^5)
Pe = Ct J / Cp
```

It preserves RPM-block and row order and extracts the published `V`, `J`, `Pe`, `Ct`, and `Cp`
columns. The six sparse rows remain explicit rows with unavailable coefficients; no value is
filled, extrapolated, or guessed. A numeric row with neither 2 nor 15 columns is malformed.
Independent variables must be finite and strictly ordered, RPM must be positive, and a present
`Cp` must be positive. The artifact records the source format/version, path, dimensions, row and
block counts, provenance, parser interpretation, and source hash.

When loading a linked raw source, the loader computes SHA-256 with `sha2` directly over the exact
bytes returned by the filesystem, before UTF-8 decoding or parsing, encodes the digest as lowercase
hexadecimal, and compares it with the dataset metadata. A mismatch fails as
`LinkedDatasetMismatch { field: "sha256", .. }`. When the referenced provenance source also has a
SHA-256 value, source and dataset hashes must agree case-insensitively before the raw file is
accepted. Byte/line counts and parser checks remain additional gates, not substitutes for the
cryptographic check.

The APC designation supplies 11 in diameter and 7 in pitch. Their exact SI conversions are
0.2794 m and 0.1778 m. Dataset and propeller dimensions must agree.

## Coefficient convention

APC publishes `Cp`; S5B consumes torque coefficient `Cq`. M2.4A exposes only the deterministic
derived relation already documented by the dossier and S5B:

```text
Cq = Cp / (2 pi)
```

The API returns no `Cq` where `Cp` is absent and labels the value as derived. It never generates
`Ct` or `Cp`, does not resample the APC table, and does not convert the manufacturer data into a
runtime `PropellerCoefficientTable`.

## Validation and readiness

Validation fails closed for invalid schema or artifact kind, unknown JSON fields, malformed IDs,
duplicate IDs or references, unresolved sources/photos/components/datasets, unsafe linked-source
paths, invalid dates or SHA-256 syntax, source/dataset hash disagreement, exact-byte digest
mismatch, blank metadata, non-finite/non-positive present physical values, unordered ranges,
invalid battery points, incompatible physical/configuration identity, mismatched component
applicability, inconsistent propeller dimensions, and malformed APC tables. Bad evidence is never
silently repaired.

The evaluation exposes:

- `motor_evidence_ready`;
- `esc_evidence_ready`;
- `battery_evidence_ready`;
- `propeller_evidence_ready`;
- `configuration_identified`;
- `propulsion_evidence_ready`; and
- `runtime_ready`, always false in M2.4A.

For an identified physical configuration, motor readiness requires Kv, winding resistance, and
no-load current; ESC readiness requires rating plus sourced resistance or efficiency; battery
readiness requires identity, capacity, internal resistance, and voltage-under-load evidence; and
propeller readiness requires identity, dimensions, and successfully parsed coefficient evidence.
Aggregate evidence readiness requires all component gates and the configuration identity gate.

The committed artifact has useful historical motor and propeller evidence, but it deliberately
reports every component readiness gate false because no physical installation is identified. ESC
loss evidence and battery electrical/load evidence also remain blockers.

## Runtime isolation

`PropulsionEvidenceLoader` is separate from `AircraftModelLoader`. M2.4A does not enter
`BatteryConfig`, `MotorConfig`, `PropellerConfig`, `PropellerCoefficientTable`,
`ElectricPropulsionConfig`, `RuntimeElectricPropulsion`, the aircraft physics fingerprint, RK4,
the 500 Hz path, or replay. Existing synthetic propulsion behavior and regression fixtures are
unchanged.

## Requirements for M2.4B

M2.4B should remain a separate, reviewed calibration/integration slice. Before any LT-40 runtime
configuration is authorized it needs, at minimum:

- a specific physical airframe and operational/propulsion configuration identity;
- photographs or equivalent evidence for installed motor, ESC, battery, propeller, and spinner;
- traceable installed-component datasheets and serial/model identity where available;
- battery open-circuit and loaded-voltage evidence across relevant SOC and temperature, including
  justified internal resistance or a reviewed replacement model;
- ESC compatibility and loss/efficiency evidence, with an explicit decision on how S5B's ideal ESC
  limitation is handled;
- installed motor electrical evidence and reconciliation of limits with the chosen battery/prop;
- an explicit, reviewed transformation of applicable APC rows into the runtime coefficient domain,
  including treatment of negative `Ct`, sparse terminal rows, RPM dependence, advance-ratio range,
  and endpoint policy;
- static or dynamic bench measurements of voltage, current, RPM, thrust, and torque/power across
  relevant conditions, with uncertainty and ambient metadata;
- validation against the evidence before runtime promotion; and
- deterministic fingerprint, propulsion-regression, stage-local, and replay tests for the new
  runtime model.

M2.4A stops before all of those runtime decisions.
