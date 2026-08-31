# Acro Electric 01

This model is an **initial engineering placeholder / not yet flight-calibrated**.

Its values are physically plausible and internally consistent, but they exist only to exercise
aircraft-model schema v1, strict loading, reference resolution, fingerprinting, and S6.1 aircraft
assembly. The eight-element discretization and four control-surface bindings are deliberately
small engineering placeholders. They are not measured or calibrated flight-test data and must not be used to infer
real-aircraft performance or safety margins. Physical calibration belongs to S10.

`model.json` intentionally contains a relative presentation reference to `aircraft.glb`. S6.1 keeps
that string as non-physical metadata and does not require the referenced file to exist. No GLB asset
is supplied at this stage.
