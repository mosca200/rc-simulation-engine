// RV2-5: GPU image-based-lighting convolution.
//
// Runs once during V2 initialization: cosine-weighted diffuse irradiance, a
// GGX importance-sampled specular prefilter (one render view per cube face and
// per mip) and the split-sum BRDF integration LUT. All three consume the
// generated environment cubemap; none allocates in the frame loop.

const PI: f32 = 3.141592653589793;
const IRRADIANCE_SAMPLE_COUNT: i32 = 512;
const PREFILTER_SAMPLE_COUNT: i32 = 256;
const BRDF_SAMPLE_COUNT: i32 = 512;
const PREFILTER_MIP_COUNT: f32 = 8.0;

struct PrefilterUniform {
    roughness: f32,
    source_resolution: f32,
    _padding: vec2<f32>,
};

@group(0) @binding(0)
var environment_cube: texture_cube<f32>;
@group(0) @binding(1)
var<uniform> prefilter: PrefilterUniform;
@group(0) @binding(2)
var environment_sampler: sampler;

struct FullscreenVertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) @interpolate(flat) face: u32,
};

@vertex
fn vs_fullscreen(
    @builtin(vertex_index) vertex_index: u32,
    @builtin(instance_index) instance_index: u32,
) -> FullscreenVertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    var output: FullscreenVertexOutput;
    let position = positions[vertex_index];
    output.position = vec4<f32>(position, 0.0, 1.0);
    output.uv = vec2<f32>(position.x * 0.5 + 0.5, 0.5 - position.y * 0.5);
    output.face = instance_index;
    return output;
}

fn radical_inverse_vdc(bits_in: u32) -> f32 {
    var bits = bits_in;
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return f32(bits) * 2.3283064365386963e-10;
}

fn hammersley(index: u32, count: u32) -> vec2<f32> {
    return vec2<f32>(f32(index) / f32(count), radical_inverse_vdc(index));
}

fn distribution_ggx(ndot_h: f32, roughness: f32) -> f32 {
    let alpha = roughness * roughness;
    let alpha_squared = alpha * alpha;
    let denominator = ndot_h * ndot_h * (alpha_squared - 1.0) + 1.0;
    return alpha_squared / max(PI * denominator * denominator, 1e-6);
}

fn geometry_schlick_ggx(ndot_v: f32, roughness: f32) -> f32 {
    let k = roughness * roughness * 0.5;
    return ndot_v / max(ndot_v * (1.0 - k) + k, 1e-6);
}

fn geometry_smith(ndot_v: f32, ndot_l: f32, roughness: f32) -> f32 {
    return geometry_schlick_ggx(ndot_v, roughness) * geometry_schlick_ggx(ndot_l, roughness);
}

// GGX importance sampling around `n`, matching the half-vector distribution
// used by the split-sum approximation.
fn importance_sample_ggx(xi: vec2<f32>, n: vec3<f32>, roughness: f32) -> vec3<f32> {
    let alpha = roughness * roughness;
    let alpha_squared = alpha * alpha;
    let phi = 2.0 * PI * xi.x;
    let cos_theta = sqrt(max((1.0 - xi.y) / (1.0 + (alpha_squared - 1.0) * xi.y), 0.0));
    let sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));
    let half = vec3<f32>(sin_theta * cos(phi), sin_theta * sin(phi), cos_theta);
    let reference = select(
        vec3<f32>(0.0, 0.0, 1.0),
        vec3<f32>(1.0, 0.0, 0.0),
        abs(n.z) > 0.999,
    );
    let tangent = normalize(cross(reference, n));
    let bitangent = cross(n, tangent);
    return normalize(tangent * half.x + bitangent * half.y + n * half.z);
}

fn cube_direction(face: u32, uv: vec2<f32>) -> vec3<f32> {
    let u = uv.x * 2.0 - 1.0;
    let v = uv.y * 2.0 - 1.0;
    var direction: vec3<f32>;
    switch face {
        case 0u: {
            direction = vec3<f32>(1.0, -v, -u);
        }
        case 1u: {
            direction = vec3<f32>(-1.0, -v, u);
        }
        case 2u: {
            direction = vec3<f32>(u, 1.0, v);
        }
        case 3u: {
            direction = vec3<f32>(u, -1.0, -v);
        }
        case 4u: {
            direction = vec3<f32>(u, -v, 1.0);
        }
        default: {
            direction = vec3<f32>(-u, -v, -1.0);
        }
    }
    return normalize(direction);
}

// Cosine-weighted hemispherical irradiance: sampling with pdf = cos(theta)/PI
// makes the integral equal PI times the mean incoming radiance.
@fragment
fn fs_irradiance_cube(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let n = cube_direction(input.face, input.uv);
    let reference = select(
        vec3<f32>(0.0, 0.0, 1.0),
        vec3<f32>(1.0, 0.0, 0.0),
        abs(n.z) > 0.999,
    );
    let tangent = normalize(cross(reference, n));
    let bitangent = cross(n, tangent);
    var sum = vec3<f32>(0.0);
    for (var sample_index = 0; sample_index < IRRADIANCE_SAMPLE_COUNT; sample_index = sample_index + 1) {
        let xi = hammersley(u32(sample_index), u32(IRRADIANCE_SAMPLE_COUNT));
        let phi = 2.0 * PI * xi.y;
        let radius = sqrt(xi.x);
        let local = vec3<f32>(
            radius * cos(phi),
            radius * sin(phi),
            sqrt(max(1.0 - xi.x, 0.0)),
        );
        let direction = tangent * local.x + bitangent * local.y + n * local.z;
        sum = sum + textureSampleLevel(environment_cube, environment_sampler, direction, 0.0).rgb;
    }
    return vec4<f32>(sum / f32(IRRADIANCE_SAMPLE_COUNT) * PI, 1.0);
}

// GGX prefilter: one roughness per mip, `roughness = mip / (mip_count - 1)`.
// The outgoing split-sum lobe is integrated with importance sampling; V equals
// N (the reflection direction), which is the standard split-sum assumption.
@fragment
fn fs_prefiltered_cube(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let n = cube_direction(input.face, input.uv);
    let roughness = clamp(prefilter.roughness, 0.0, 1.0);
    var sum = vec3<f32>(0.0);
    var weight = 0.0;
    for (var sample_index = 0; sample_index < PREFILTER_SAMPLE_COUNT; sample_index = sample_index + 1) {
        let xi = hammersley(u32(sample_index), u32(PREFILTER_SAMPLE_COUNT));
        let half = importance_sample_ggx(xi, n, roughness);
        let light = normalize(2.0 * dot(n, half) * half - n);
        let ndot_l = dot(n, light);
        if (ndot_l > 0.0) {
            let ndot_h = max(dot(n, half), 1e-4);
            let vdot_h = max(dot(n, half), 1e-4);
            let pdf = distribution_ggx(ndot_h, roughness) * ndot_h / max(4.0 * vdot_h, 1e-4);
            let sample_solid_angle = 1.0 / (f32(PREFILTER_SAMPLE_COUNT) * pdf + 1e-4);
            let texel_solid_angle = 4.0 * PI
                / (6.0 * prefilter.source_resolution * prefilter.source_resolution);
            let mip = select(
                0.0,
                clamp(
                    0.5 * log2(sample_solid_angle / texel_solid_angle),
                    0.0,
                    PREFILTER_MIP_COUNT - 1.0,
                ),
                roughness > 0.0,
            );
            sum = sum
                + textureSampleLevel(environment_cube, environment_sampler, light, mip).rgb * ndot_l;
            weight = weight + ndot_l;
        }
    }
    return vec4<f32>(sum / max(weight, 1e-4), 1.0);
}

// Split-sum BRDF integration. Axes: u = NdotV, v = roughness.
@fragment
fn fs_brdf_lut(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let ndot_v = clamp(input.uv.x, 1e-4, 1.0);
    let roughness = clamp(input.uv.y, 0.0, 1.0);
    let view = vec3<f32>(sqrt(max(1.0 - ndot_v * ndot_v, 0.0)), 0.0, ndot_v);
    let normal = vec3<f32>(0.0, 0.0, 1.0);
    var scale = 0.0;
    var bias = 0.0;
    for (var sample_index = 0; sample_index < BRDF_SAMPLE_COUNT; sample_index = sample_index + 1) {
        let xi = hammersley(u32(sample_index), u32(BRDF_SAMPLE_COUNT));
        let half = importance_sample_ggx(xi, normal, roughness);
        let light = normalize(2.0 * dot(view, half) * half - view);
        let ndot_l = light.z;
        if (ndot_l > 0.0) {
            let ndot_h = max(half.z, 1e-4);
            let vdot_h = max(dot(view, half), 0.0);
            let geometry = geometry_smith(ndot_v, ndot_l, roughness);
            let visibility = geometry * vdot_h / max(ndot_h * ndot_v, 1e-4);
            let fresnel = pow(clamp(1.0 - vdot_h, 0.0, 1.0), 5.0);
            scale = scale + (1.0 - fresnel) * visibility;
            bias = bias + fresnel * visibility;
        }
    }
    return vec4<f32>(scale / f32(BRDF_SAMPLE_COUNT), bias / f32(BRDF_SAMPLE_COUNT), 0.0, 1.0);
}

