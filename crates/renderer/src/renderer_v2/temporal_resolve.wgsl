// IQ0-A identity temporal resolve. The dedicated pass and history lifecycle
// are real; accumulation is intentionally deferred to a later slice.

@group(0) @binding(0)
var current_hdr: texture_2d<f32>;

@vertex
fn vs_temporal_resolve(@builtin(vertex_index) vertex_index: u32) -> @builtin(position) vec4<f32> {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    return vec4<f32>(positions[vertex_index], 0.0, 1.0);
}

@fragment
fn fs_temporal_resolve(@builtin(position) position: vec4<f32>) -> @location(0) vec4<f32> {
    return textureLoad(current_hdr, vec2<i32>(position.xy), 0);
}
