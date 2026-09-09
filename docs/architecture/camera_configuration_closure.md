# Camera configuration closure

Status: implemented for the presentation CLI in `crates/app/src/render_app.rs`.

## Scope

Camera configuration is presentation-only. This closure changes only how the
`render` and `play` command-line options are assembled into a
`CameraSelection`; it does not alter renderer camera mathematics, simulation
state, aircraft dynamics, replay, or any visual subsystem.

## Two-phase resolution

The parser first validates and records camera intent without mutating the
active camera selection:

- final mode from `--camera pilot|chase`;
- shared FOV from `--camera-fov`;
- chase distance and height;
- pilot position.

After all arguments have been parsed, one centralized resolver selects the
final mode and applies compatible values. Therefore a valid set of camera
options produces the same `CameraSelection` regardless of option ordering.
When an option is repeated, the last value for that same option remains the
winner, matching the general CLI parsing convention.

## Defaults and explicit modes

With no camera arguments, defaults are unchanged:

- `render`: pilot at `[0.0, 0.3, 0.85]`, vertical FOV 70 degrees;
- `play`: chase at 3.0 m behind and 0.95 m above, vertical FOV 55 degrees.

An explicit `--camera` also retains its established initialization values:

- `--camera pilot`: position `[0.0, 1.8, 20.0]`, vertical FOV 55 degrees;
- `--camera chase`: 3.5 m behind and 1.25 m above, vertical FOV 55 degrees.

Compatible tuning values override those defaults at final resolution.

## Compatibility rules

`--camera-fov` is valid for both modes. Mode-specific options are accepted
only when they match the final mode:

- `--pilot-position` requires `pilot`;
- `--chase-distance-m` and `--chase-height-m` require `chase`.

An incompatible option returns a diagnostic naming the option, its required
mode, and the actual final mode. It is never silently discarded. This applies
equally when the mode comes from an explicit `--camera` argument or from the
`render`/`play` preset.

All previous numeric validation remains at the parse boundary, including
finite-value checks. FOV accepts 10 through 120 degrees, chase distance accepts
values above 0 through 1000 m, chase height accepts -100 through 1000 m, and
pilot position requires exactly three finite comma-separated components.

## Regression coverage

Unit tests pin the exact `render` and `play` defaults, both explicit modes,
shared and mode-specific tuning, multiple pairwise order permutations, a
three-tuning permutation, incompatibility diagnostics in both orders, numeric
bounds, non-finite inputs, and malformed pilot positions.
