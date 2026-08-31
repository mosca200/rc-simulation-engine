# Acro Electric 01

This model is an **initial engineering placeholder / not yet flight-calibrated**.

Its values are physically plausible and internally consistent, but they exist only to exercise
aircraft-model schema v1, strict loading, reference resolution, fingerprinting, and S6.1 aircraft
assembly. The eight-element discretization and four control-surface bindings are deliberately
small engineering placeholders. They are not measured or calibrated flight-test data and must not be used to infer
real-aircraft performance or safety margins. Physical calibration belongs to S10.

`model.json` contains a relative presentation reference to `aircraft.glb`. The checked-in asset is a
low-poly **presentation placeholder**, not final artwork. It makes the fuselage, main wing,
horizontal stabilizer, vertical stabilizer, and orange nose direction visible. Its local render
coordinates are `+X` right, `+Y` up, and `-Z` forward/nose. Regenerate it deterministically from the
repository root with `powershell -File tools/generate_placeholder_aircraft_glb.ps1`.
