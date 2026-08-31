# Acro Electric 01

This model is explicitly classified as `synthetic_test`. It exists to exercise the simulator and
support deterministic regression tests. It is **not a validated reproduction of a real aircraft**
and must not be used as evidence that the simulator is realistic.

Its values exist only to exercise strict model loading, reference resolution, fingerprinting, and
aircraft assembly. The eight-element discretization, aerodynamic polars, and four control-surface
bindings are intentionally simplified synthetic data. They are not measured or calibrated
flight-test data and must not be used to infer real-aircraft performance or safety margins.

`model.json` contains a relative presentation reference to `aircraft.glb`. The checked-in asset is a
low-poly **presentation placeholder**, not final artwork. It makes the fuselage, main wing,
horizontal stabilizer, vertical stabilizer, and orange nose direction visible. Its local render
coordinates are `+X` right, `+Y` up, and `-Z` forward/nose. Regenerate it deterministically from the
repository root with `powershell -File tools/generate_placeholder_aircraft_glb.ps1`.
