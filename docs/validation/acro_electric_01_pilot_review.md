# Acro Electric 01 — structured pilot review protocol

## Status

**NOT YET EXECUTED.** This document prepares a future review; it contains no fabricated scores,
observations, hardware verification, or real-world validation.

## Session record

| Field | Entry |
| --- | --- |
| Date/time | |
| Build SHA | |
| Model physics fingerprint | |
| Pilot name/code | |
| Pilot RC experience (years, disciplines) | |
| Experience with real/reference aircraft | |
| Controller/device and connection mode | |
| Controller calibration/profile | |
| Rates/expo configuration | |
| Display/render configuration | |
| Test sequence version | `S10 manoeuvre suite v1` or describe deviation |
| Environmental/session notes | |

Record whether the session uses simulation only or a real reference aircraft comparison. Do not
infer real-aircraft equivalence when no matching aircraft or measured flight data is available.

## Rating scale

Use the same seven-point scale for response-quality questions:

| Score | Meaning |
| ---: | --- |
| 1 | strongly too little / strongly unrealistic |
| 2 | clearly too little |
| 3 | slightly too little |
| 4 | neutral / plausible / no directional correction |
| 5 | slightly too much |
| 6 | clearly too much |
| 7 | strongly too much / strongly unrealistic |

For overall realism, use `1 = not credible`, `4 = mixed`, `7 = highly credible`. Every numerical
score must include a short observation and, when possible, the manoeuvre/time at which it occurred.

## Safety and preparation

1. Verify the exact build SHA and model fingerprint before the session.
2. Record controller mapping, rates and expo; do not change them silently mid-sequence.
3. Begin with neutral response and low-authority inputs.
4. Treat stall, spin, landing and ground behaviour as unsupported where the current physics lacks
   the required phenomena.
5. Save replay and telemetry for every completed simulation run.

## Review sequence

| Order | Exercise | Intended observation | Completed |
| ---: | --- | --- | --- |
| 1 | Neutral flight | trim tendency and neutral stability | |
| 2 | Small pitch steps | pitch response, damping, linearity | |
| 3 | Small roll steps | roll response, damping, symmetry | |
| 4 | Small yaw steps | yaw response and coupling | |
| 5 | Increasing control amplitude | authority and saturation perception | |
| 6 | Positive/negative reversal | reversal and recovery | |
| 7 | Throttle steps | power response and acceleration | |
| 8 | Power-off segment | glide behaviour and speed loss | |
| 9 | High-angle entry | onset cues only; not dynamic-stall validation | |
| 10 | Vertical manoeuvre | vertical performance, if safely observable | |
| 11 | Loop | loop entry, energy retention and symmetry | |
| 12 | Axial roll | roll behaviour and coupling | |
| 13 | Approach | landing-approach feel in free air only; no ground validation | |

## Structured ratings

| Criterion | Score | Reference/comparison | Observation and evidence |
| --- | ---: | --- | --- |
| Neutral stability | | | |
| Pitch response | | | |
| Roll response | | | |
| Yaw response | | | |
| Control authority | | | |
| Control linearity | | | |
| Expo/rates perception | | | |
| Stall onset cues | | | |
| Stall recovery | | | |
| Glide behaviour | | | |
| Power response | | | |
| Vertical performance | | | |
| Loop behaviour | | | |
| Roll behaviour | | | |
| Landing-approach feel | | | |
| Overall inertia feeling | | | |
| Overall realism | | | |

## Unexpected behaviour

For every anomaly record:

- manoeuvre and replay filename;
- simulation time or step index;
- input being applied;
- observed behaviour;
- expected behaviour and its source;
- severity and reproducibility;
- whether telemetry supports the observation.

## Free notes and disposition

```text
Free notes:


Suggested follow-up:


Parameters proposed for investigation (not automatic tuning):


Evidence/reference required before any change:
```

Review completion requires the session record, all applicable ratings, saved evidence, and explicit
sign-off. Until then the project status remains `pilot review: NOT TESTED`.
