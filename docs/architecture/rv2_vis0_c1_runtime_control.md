# RV2-VIS0-C1 deterministic runtime control

RV2-VIS0-C1 adds only the presentation-runtime foundations needed by a future
golden capture. It does not read pixels, encode images, compare images, or
change the VIS0-B runner capability declarations.

## Explicit render resolution

`rcsim-app render` and the shared render-options parser accept:

```text
--render-width <physical-pixels> --render-height <physical-pixels>
```

The flags are a pair: specifying only one is an error. Width is constrained to
`320..=7680` and height to `240..=4320`, matching the VIS0 manifest contract.
When present, the window is created with a non-resizable `PhysicalSize`; the
runtime verifies the actual inner size before initializing wgpu and fails
closed if the platform does not apply it. DPI and resize events reassert the
requested physical extent synchronously; if the platform cannot restore it,
the runtime fails instead of silently weakening resolution enforcement.
`DeviceContext` continues to configure the surface from the window's actual
physical inner size, so the requested value reaches both V1 and V2 surface
configuration.

When the flags are absent, the historical behavior remains unchanged: winit is
asked for a resizable `LogicalSize` of 1280x720. Camera FOV is independent and
is not changed by resolution selection.

## Presentation frame lifecycle

The presentation frame index is zero-based:

- frame `0` is the first frame for which the renderer submits the final
  postprocess work and calls surface `present()`;
- the index advances only after the renderer reports `Presented`;
- surface acquisition failure, occlusion, or a zero-extent surface does not
  consume an index;
- flight reset, suspend/resume, and wall-clock timing do not reset or define
  the process-wide presentation index.

`RenderRunControl::pending_frame` exposes the index before renderer submission,
and `commit_presented` advances it after presentation. This is the minimal seam
for the next capture tranche: it can decide that a pending index is the capture
target, schedule the GPU copy before submission/presentation, and commit the
same index only after successful presentation.

The counter is presentation-only. It does not change the 2 ms fixed physics
step, the accumulator, RK4, replay, or render snapshot semantics.

## Frame-based automatic exit

```text
--exit-after-frame <N>
```

`N` is the zero-based presentation frame index. The event loop exits cleanly
immediately after frame `N` is successfully presented:

- `--exit-after-frame 0` presents one frame, then exits;
- `--exit-after-frame 1` presents frames 0 and 1, then exits;
- without the flag, no frame-based exit is scheduled and interactive behavior
  remains unchanged.

The criterion contains no sleep, timer, or wall-clock threshold. Existing
recording finalization runs through the normal event-loop exit path.

## Warmup and future capture translation

VIS0 continues to own separate `warmup` and `capture.frame` fields; C1 does not
rewrite that contract and does not claim either capability is supported.

For the current v1 manifest, `warmup` is `10` and `capture.frame` is `10`.
With the zero-based lifecycle above, frames `0..9` are the preceding warmup
presentations and frame `10` is the intended capture target. The next tranche
must validate the relationship rather than silently choosing one field, request
capture for pending presentation index 10, and use `--exit-after-frame 10` so
the captured frame is presented before clean exit.

Until GPU readback exists, VIS0-B must continue to report framebuffer capture,
warmup enforcement, capture-frame enforcement, and complete process auto-exit
capture orchestration as unavailable or unsupported.

## Post-tone-map capture audit

The current postprocess pass writes exposure and the neutral tone-mapped result
directly into the sRGB surface. The surface is configured only with
`RENDER_ATTACHMENT`; adapter surface capabilities are not guaranteed to allow
`COPY_SRC` on every backend.

The recommended next step is therefore a final LDR intermediate texture with
`RENDER_ATTACHMENT | TEXTURE_BINDING | COPY_SRC`, followed by a minimal present
pass that samples/blits it to the surface. Readback should copy that
display-referred postprocess output, with aligned rows, into a mapped buffer.
This avoids making golden capture depend on optional surface `COPY_SRC` support
and preserves the existing HDR scene buffer as an internal pre-tone-map target.

C1 deliberately makes no render-pipeline or surface-usage change for this
future strategy.
