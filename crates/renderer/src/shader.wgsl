// G1A: Material + directional lighting foundation.
// Deliberately simple Lambert diffuse + ambient. No PBR.

struct CameraUniform {
    view_projection: mat4x4<f32>,
};

struct ObjectUniform {
    model: mat4x4<f32>,
};

// direction.xyz: normalized world-space direction TOWARD the light.
// direction.w:   directional light intensity.
// ambient.xyz:   ambient light color (typically grey).
// ambient.w:     unused / reserved.
struct LightUniform {
    direction: vec4<f32>,
    ambient: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> object: ObjectUniform;

@group(2) @binding(0)
var<uniform> light: LightUniform;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
    @location(3) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

// Extract the upper-left 3x3 from the model matrix.
// For rigid-body transforms (rotation + uniform scale only),
// this is the correct normal transform (inverse-transpose == the matrix itself).
fn normal_matrix(model: mat4x4<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(
        model[0].xyz,
        model[1].xyz,
        model[2].xyz,
    );
}

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    let world_position = object.model * vec4<f32>(input.position, 1.0);
    output.clip_position = camera.view_projection * world_position;
    output.world_normal = normalize(normal_matrix(object.model) * input.normal);
    output.color = input.color;
    output.uv = input.uv;
    return output;
}

// Lit triangle fragment: ambient + Lambert directional diffuse.
//
// color = base_color * (ambient + directional_intensity * max(dot(N, L), 0))
//
// Deterministic defaults:
//   ambient       = vec3(0.30)
//   direction     = normalize(vec3(0.4, 0.8, 0.3))  (above, right, slightly forward)
//   intensity     = 0.80
@fragment
fn fs_lit(input: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(input.world_normal);
    let l = normalize(light.direction.xyz);
    let diffuse = max(dot(n, l), 0.0);
    let lit_color = input.color * (light.ambient.xyz + light.direction.w * diffuse);
    return vec4<f32>(lit_color, 1.0);
}

// Unlit fragment: pass-through vertex color for debug geometry (grid, axes, lines).
@fragment
fn fs_unlit(input: VertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(input.color, 1.0);
}
