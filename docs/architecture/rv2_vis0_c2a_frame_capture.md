# RV2-VIS0-C2A deterministic final frame capture

## Scope and capture point

The runtime exposes one renderer-domain operation, `render_and_capture`, in
addition to the unchanged production `render` call. It is shared by the V1 and
V2 facade paths. The captured pixels are presentation-only; simulation,
replay, interpolation, and fixed-step state are not inputs to capture
scheduling.

The capture point is the final LDR output of the existing postprocess:

`linear HDR / resolved temporal history -> exposure -> Khronos PBR Neutral -> Rgba8UnormSrgb`

The one requested frame redirects postprocess from the surface to a lazily
created `Rgba8UnormSrgb` texture with exactly these usages:

- `RENDER_ATTACHMENT`
- `TEXTURE_BINDING`
- `COPY_SRC`

The texture uses the renderer's current surface width and height. It is
one-shot and is not retained across frames, so a resize before capture is
naturally reflected in its extent and cannot leave a stale-sized target.

## Why capture does not read the swapchain

The surface remains configured only for `RENDER_ATTACHMENT`. Capture does not
require or assume swapchain `COPY_SRC` support and does not involve a desktop
screenshot or the OS compositor. After postprocess, a nearest-filtered
fullscreen pass samples the final LDR texture and writes it 1:1 to the sRGB
surface. Sampling decodes sRGB to linear and the sRGB surface encodes it again;
there is no extra exposure, tone map, grading, sharpening, or manual gamma.
Capture fails closed if the presentation surface is not sRGB.

## GPU readback

For RGBA8, the unpadded row size is `width * 4`. The staging row size is
rounded up explicitly to `wgpu::COPY_BYTES_PER_ROW_ALIGNMENT` (256):

`padded = ((width * 4 + 255) / 256) * 256`

The staging buffer size is `padded * height` and its usages are `COPY_DST |
MAP_READ`. Every multiplication, addition, and host-size conversion is checked
for overflow. Mapping removes padding row by row and returns exactly `width *
height * 4` bytes in RGBA order.

The renderer submits render, blit, and copy commands once, requests an async
map, then calls `Device::poll(PollType::Wait)` for that exact submission with a
30-second timeout. There is no sleep polling, busy loop, detached thread, or
persistent worker. Map, poll, target validation, and surface failures are
typed `FrameCaptureError` results. A failed readback does not return pixels or
present/commit the requested frame.

## Presentation-frame lifecycle

`--capture-frame N` uses `RenderRunControl::pending_frame().index`, where frame
zero is the first frame that is actually presented. It does not introduce a
physics-step, redraw, wall-clock, or second presentation counter. Surface
acquisition failures, occlusion, and zero extent leave the same frame pending;
a successfully presented frame is committed exactly once.

Capture and exit are separate options. If both are supplied,
`--exit-after-frame` must be greater than or equal to `--capture-frame`. For the
canonical equal-frame case, the renderer reads back and presents frame N, the
application writes the PNG and optional receipt, and only then exits. Artifact
failure makes the process fail. Closing a capture-configured run before a
successful capture also fails.

## CLI and artifacts

Capture requires the complete group:

```text
--capture-frame N --capture-out PATH --capture-format png
[--capture-receipt-out PATH]
```

Only lossless PNG is supported. `jpg`, `jpeg`, `exr`, and every other value are
rejected rather than silently converted.

The application owns PNG encoding, filesystem I/O, SHA-256 provenance, and the
receipt. At capture startup it removes an existing image, receipt, and their
deterministic temporary files. PNG and receipt writes go to a sibling `.tmp`
file and are renamed only after the complete byte sequence is written. A
receipt-write failure removes the newly written image, preventing a failed run
from leaving a seemingly complete image/receipt pair. Image, receipt, and
temporary sibling paths must be distinct; colliding CLI paths are rejected.

The optional `RuntimeCaptureReceipt` schema is version `1.0.0`:

```json
{
  "schema_version": "1.0.0",
  "presentation_frame_index": 10,
  "framebuffer_width": 320,
  "framebuffer_height": 240,
  "format": "png",
  "image_path": "capture.png",
  "image_sha256": "<64 lowercase hexadecimal characters>",
  "image_byte_size": 12345
}
```

The frame index is the frame actually captured. Width and height come from the
renderer's `CapturedFrame`, never from requested CLI dimensions. SHA-256 and
byte size are computed over the encoded PNG bytes that are written.

## Normal-path cost and limitations

With capture options disabled, postprocess still writes directly to the
surface. No LDR intermediate, staging buffer, map, copy, blit, PNG work, or
filesystem side effect is created. Capture resources are allocated only after
the requested frame has acquired a surface texture.

The implementation is intentionally one-shot and synchronous on the capture
frame. It supports native wgpu polling and PNG only. Benchmark-runner and
`VisualCaptureEvidence` integration are outside C2A.

The optional headless GPU regression can be run with:

```text
cargo test -p renderer --lib capture::tests::headless_rgba8_srgb_render_copy_map_and_unpad -- --ignored --exact
```
