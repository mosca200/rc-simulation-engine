// G1C: Base-color texture support + G1B sky/atmosphere + G1A material/lighting.
// Deliberately simple. No PBR, no clouds, no HDR, no shadows.

// ---------------------------------------------------------------------------
// Uniforms
// ---------------------------------------------------------------------------

struct CameraUniform {
    view_projection: mat4x4<f32>,
    inv_view_projection: mat4x4<f32>,
    camera_position: vec4<f32>,
};

struct ObjectUniform {
    model: mat4x4<f32>,
};

// Single source of truth for lighting and atmosphere.
//
// light_direction.xyz: normalized world-space direction TOWARD the light.
// light_direction.w:   directional light intensity.
// ambient.xyz:         ambient light color (typically grey).
// ambient.w:           reserved.
// sky_zenith.xyz:      zenith color (straight up).
// sky_zenith.w:        reserved.
// sky_horizon.xyz:     horizon / haze color.
// sky_horizon.w:       haze strength [0, 1].
// sky_ground.xyz:      below-horizon atmospheric color.
// sky_ground.w:        fog density (exponential fog coefficient).
// sun_color.xyz:       sun disk color.
// sun_color.w:         cosine of sun angular radius.
struct EnvironmentUniform {
    light_direction: vec4<f32>,
    ambient: vec4<f32>,
    sky_zenith: vec4<f32>,
    sky_horizon: vec4<f32>,
    sky_ground: vec4<f32>,
    sun_color: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> object: ObjectUniform;

@group(2) @binding(0)
var<uniform> environment: EnvironmentUniform;

// G1C: Material texture and sampler.
// Group 3 is the material bind group, containing the base color texture and sampler.
@group(3) @binding(0)
var base_color_texture: texture_2d<f32>;
@group(3) @binding(1)
var base_color_sampler: sampler;

// ---------------------------------------------------------------------------
// Vertex IO
// ---------------------------------------------------------------------------

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) world_position: vec3<f32>,
};

struct SkyVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) clip_xy: vec2<f32>,
};

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// Extract the upper-left 3x3 from the model matrix for normal transformation.
//
// Normal transform invariant: the current RenderPose model matrix contains only
// rigid-body rotation + translation (no non-uniform scale, no shear). For a
// pure rotation matrix R, the correct normal transform (inverse-transpose)
// equals R itself. Therefore the upper-left 3x3 is used directly.
//
// If non-uniform scale is ever introduced, this must be replaced with the
// inverse-transpose of the upper-left 3x3.
fn normal_matrix(model: mat4x4<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(
        model[0].xyz,
        model[1].xyz,
        model[2].xyz,
    );
}

// Render-space world-up: +Y is up (NED Down maps to render -Y).
const WORLD_UP: vec3<f32> = vec3<f32>(0.0, 1.0, 0.0);

// Reconstruct a world-space view direction from clip-space coordinates at the
// far plane (depth = 1.0). The result is normalized and camera-translation
// invariant — the sky is effectively at infinite distance.
fn view_direction_from_clip(clip_xy: vec2<f32>) -> vec3<f32> {
    let clip_far = vec4<f32>(clip_xy, 1.0, 1.0);
    let world_h = camera.inv_view_projection * clip_far;
    let world_pos = world_h.xyz / world_h.w;
    return normalize(world_pos - camera.camera_position.xyz);
}

// Compute sky color for a given view direction.
// Shared between the sky pass and (potentially) other shaders.
fn sky_color_for_direction(view_dir: vec3<f32>) -> vec3<f32> {
    let elevation = dot(view_dir, WORLD_UP);

    let zenith = environment.sky_zenith.xyz;
    let horizon = environment.sky_horizon.xyz;
    let ground_atm = environment.sky_ground.xyz;

    // Nonlinear gradient: power curve above horizon, slightly different below.
    // This avoids a banded linear look and gives a natural atmospheric falloff.
    var sky: vec3<f32>;
    if (elevation >= 0.0) {
        let t = pow(elevation, 0.5);
        sky = mix(horizon, zenith, t);
    } else {
        let t = pow(clamp(-elevation, 0.0, 1.0), 0.7);
        sky = mix(horizon, ground_atm, t);
    }

    // Horizon haze: widen the horizon band by blending toward horizon color.
    // haze_falloff is strongest at the horizon (elevation ≈ 0) and decays
    // exponentially away from it.
    let haze_strength = environment.sky_horizon.w;
    let haze_falloff = exp(-abs(elevation) * 6.0);
    sky = mix(sky, horizon, haze_falloff * haze_strength);

    // Sun disk: procedural, coherent with the directional light.
    let sun_dir = normalize(environment.light_direction.xyz);
    let sun_alignment = dot(view_dir, sun_dir);
    let sun_cos_radius = environment.sun_color.w;
    // Smooth disk edge over a tiny angular band.
    let disk = smoothstep(sun_cos_radius - 0.0005, sun_cos_radius + 0.0005, sun_alignment);
    // Subtle halo: fades from disk edge outward.
    let halo = smoothstep(sun_cos_radius - 0.06, sun_cos_radius - 0.005, sun_alignment) * 0.25;
    sky = clamp(sky + environment.sun_color.xyz * (disk + halo), vec3<f32>(0.0), vec3<f32>(1.0));

    return sky;
}

// Exponential distance fog factor.
// fog_factor ∈ [0, 1]: 0 = no fog, 1 = fully fogged.
// Formula: fog = 1 - exp(-density * distance)
fn fog_factor(distance: f32, density: f32) -> f32 {
    return clamp(1.0 - exp(-density * max(distance, 0.0)), 0.0, 1.0);
}

// ---------------------------------------------------------------------------
// Scene vertex shader (aircraft, ground, terrain)
// ---------------------------------------------------------------------------

@vertex
fn vs_main(input: VertexInput) -> VertexOutput {
    var output: VertexOutput;
    let world_position = object.model * vec4<f32>(input.position, 1.0);
    output.clip_position = camera.view_projection * world_position;
    output.world_normal = normalize(normal_matrix(object.model) * input.normal);
    output.color = input.color;
    output.uv = input.uv;
    output.world_position = world_position.xyz;
    return output;
}

// ---------------------------------------------------------------------------
// Lit fragment: texture * vertex_color * lighting + distance fog.
//
// G1C color pipeline:
//   1. Sample base color texture (sRGB, hardware converts to linear).
//   2. Multiply by vertex color (which already contains baseColorFactor * COLOR_0).
//   3. Apply Lambert lighting.
//   4. Apply distance fog.
//
// Formula:
//   texture_rgba = textureSample(base_color_texture, base_color_sampler, uv)
//   base_rgba = input.color * texture_rgba
//   lit_rgb = base_rgba.rgb * (ambient + directional * max(dot(N, L), 0))
//   final_rgb = mix(lit_rgb, fog_color, fog_factor)
//   alpha = base_rgba.a (preserved, not lit or fogged)
//
// Deterministic defaults:
//   ambient       = vec3(0.30)
//   direction     = normalize(vec3(0.4, 0.8, -0.3))  (above, right, slightly forward)
//   intensity     = 0.80
//   fog_density   = 0.0015
//   fog_color     = sky_horizon color
@fragment
fn fs_lit(input: VertexOutput) -> @location(0) vec4<f32> {
    // G1C: Sample base color texture.
    // The texture is sRGB, so hardware converts to linear during sampling.
    let texture_rgba = textureSample(base_color_texture, base_color_sampler, input.uv);

    // Combine: vertex_color (contains baseColorFactor * COLOR_0) * texture.
    let base_rgba = input.color * texture_rgba;

    // Lighting.
    let n = normalize(input.world_normal);
    let l = normalize(environment.light_direction.xyz);
    let diffuse = max(dot(n, l), 0.0);
    let lit_rgb = base_rgba.rgb * (environment.ambient.xyz + environment.light_direction.w * diffuse);

    // Distance fog.
    let camera_pos = camera.camera_position.xyz;
    let distance = length(input.world_position - camera_pos);
    let density = environment.sky_ground.w;
    let fog = fog_factor(distance, density);
    let fog_color = environment.sky_horizon.xyz;
    let final_rgb = mix(lit_rgb, fog_color, fog);

    return vec4<f32>(final_rgb, base_rgba.a);
}

// ---------------------------------------------------------------------------
// Unlit fragment: pass-through vertex color for debug geometry (grid, axes).
// No fog applied — debug overlays remain visible at all distances.
// Alpha is preserved as-is.
@fragment
fn fs_unlit(input: VertexOutput) -> @location(0) vec4<f32> {
    return input.color;
}

// ---------------------------------------------------------------------------
// Sky fullscreen pass
// ---------------------------------------------------------------------------

// Fullscreen triangle from vertex_index only. No vertex buffer needed.
// Vertices at (-1,-1), (3,-1), (-1,3) cover the entire clip-space quad.
@vertex
fn vs_sky_fullscreen(@builtin(vertex_index) vertex_index: u32) -> SkyVertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>( 3.0, -1.0),
        vec2<f32>(-1.0,  3.0),
    );
    var output: SkyVertexOutput;
    let pos = positions[vertex_index];
    // z = 1.0, w = 1.0 → depth = 1.0 (far plane in WebGPU [0,1] clip depth).
    output.clip_position = vec4<f32>(pos, 1.0, 1.0);
    output.clip_xy = pos;
    return output;
}

// Procedural sky fragment.
// Reconstructs a world-space view direction from the inverse view-projection
// matrix, then computes sky color from the view elevation relative to world up.
// Camera translation does NOT move the sky (infinite distance).
// Camera rotation DOES rotate the viewed sky (view direction changes).
@fragment
fn fs_sky(input: SkyVertexOutput) -> @location(0) vec4<f32> {
    let view_dir = view_direction_from_clip(input.clip_xy);
    let color = sky_color_for_direction(view_dir);
    return vec4<f32>(color, 1.0);
}
