// G1C: Base-color texture support + G1B sky/atmosphere + G1A material/lighting.
// G1D: metallic/roughness PBR material response.
//
// Lighting model (G1D):
//   - Schlick Fresnel, GGX/Trowbridge-Reitz NDF, Smith geometry term
//   - glTF metallic workflow: F0 = mix(0.04, baseColor, metallic)
//   - roughness floor MIN_ROUGHNESS for numeric stability and stable highlights
//   - legacy-compat irradiance scale (see fs_lit) so the diffuse response
//     matches the established Lambert look exactly
//
// Still deliberately out of scope: no shadows, no HDR, no IBL, no normal maps,
// no clouds. Ambient is flat and mostly applied to the diffuse response.

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

// G1D: per-primitive PBR material parameters.
// metallic: 0 = dielectric, 1 = metal (glTF metallicFactor).
// roughness: perceptual roughness (glTF roughnessFactor), floored in the
// shader by MIN_ROUGHNESS. reserved keeps the struct at one 16-byte slot.
struct MaterialUniform {
    metallic: f32,
    roughness: f32,
    reserved: vec2<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> object: ObjectUniform;

@group(2) @binding(0)
var<uniform> environment: EnvironmentUniform;

// G1C: Material texture and sampler.
// Group 3 is the material bind group, containing the base color texture,
// sampler, and (G1D) the metallic/roughness uniform.
@group(3) @binding(0)
var base_color_texture: texture_2d<f32>;
@group(3) @binding(1)
var base_color_sampler: sampler;
@group(3) @binding(2)
var<uniform> material: MaterialUniform;

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
// G1D: PBR material response (metallic/roughness)
// ---------------------------------------------------------------------------

const PI: f32 = 3.141592653589793;

// Numeric + readability floor on roughness. Without it, near-zero roughness
// produces (a) near-singular GGX denominators and (b) highlight dots so small
// they alias into instability at RC-aircraft viewing distances. 0.06 keeps
// specular credible without turning surfaces matte.
const MIN_ROUGHNESS: f32 = 0.06;

// Dielectric F0 (generic plastic/paint interface reflectance at normal
// incidence). Metals override this with their albedo via the glTF workflow.
const DIELECTRIC_F0: f32 = 0.04;

// Readability ambient floor for metals. With no IBL, a pure-metal surface lit
// only by the directional term goes almost black outside its highlight,
// hurting aircraft readability at distance. We therefore let metals receive a
// flat (non-directional, non-fresnel) fraction of the existing ambient term.
// This is a documented readability approximation, NOT an environment
// reflection. Non-metals keep the plain diffuse ambient response.
const AMBIENT_SPECULAR_SCALE: f32 = 0.5;

// Upper clamp on the direct specular response. GGX can spike at grazing
// angles on low-roughness surfaces; with LDR output the spike would clip to
// white anyway, so clamping early keeps the math finite and the highlight
// stable without changing the perceived result.
const SPECULAR_CLAMP: f32 = 4.0;

// Normalize with a degenerate-direction guard. Returns WORLD_UP for zero-length
// inputs instead of NaN (e.g. camera exactly on the shaded surface point).
fn safe_normalize(direction: vec3<f32>) -> vec3<f32> {
    let len = length(direction);
    return select(vec3<f32>(WORLD_UP), direction / len, len > 1e-6);
}

// Schlick Fresnel: F = F0 + (1 - F0) * (1 - VdotH)^5.
fn schlick_fresnel(f0: vec3<f32>, vdot_h: f32) -> vec3<f32> {
    let base = 1.0 - vdot_h;
    let f = base * base * base * base * base;
    return f0 + (vec3<f32>(1.0) - f0) * f;
}

// GGX / Trowbridge-Reitz normal distribution.
fn ggx_distribution(ndot_h: f32, alpha: f32) -> f32 {
    let alpha2 = alpha * alpha;
    let denom = ndot_h * ndot_h * (alpha2 - 1.0) + 1.0;
    // denom >= alpha2 > 0 for our inputs, so this stays finite.
    return alpha2 / (PI * denom * denom);
}

// Smith geometry term with the Schlick-GGX approximation (k scaled for direct
// lighting, not IBL).
fn smith_geometry(ndot_v: f32, ndot_l: f32, roughness: f32) -> f32 {
    let k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    let gv = ndot_v / (ndot_v * (1.0 - k) + k);
    let gl = ndot_l / (ndot_l * (1.0 - k) + k);
    return gv * gl;
}

// ---------------------------------------------------------------------------
// Lit fragment: texture * vertex_color * lighting + distance fog.
//
// G1C color pipeline (preserved):
//   1. Sample base color texture (sRGB, hardware converts to linear).
//   2. Multiply by vertex color (which already contains baseColorFactor * COLOR_0).
//   3. Apply lighting (G1D: metallic/roughness BRDF instead of plain Lambert).
//   4. Apply distance fog after lighting.
//
// G1D lighting model:
//   N = world normal, V = normalize(camera_position - world_position),
//   L = directional light, H = normalize(V + L).
//   F0 = mix(0.04, baseColor, metallic)      (glTF metallic workflow)
//   diffuse albedo = baseColor * (1 - metallic)
//   specular = D(h) * G(v,l) * F(v,h) / max(4 * NdotV * NdotL, 1e-4)
//
// Legacy-compat irradiance scale: the pre-G1D Lambert path used
// albedo * intensity (no 1/PI). The physically normalized split
// (albedo/PI + specular) is therefore scaled by PI * intensity, so the
// diffuse response matches the established look exactly while the specular
// term stays energy-consistent with the diffuse.
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

    // G1D: material parameters with documented safety clamps. The roughness
    // floor prevents degenerate highlights; both uniforms are guaranteed
    // finite by the CPU-side clamp at load time.
    let metallic = clamp(material.metallic, 0.0, 1.0);
    let roughness = clamp(material.roughness, MIN_ROUGHNESS, 1.0);

    // BRDF basis vectors (all guarded against zero-length inputs).
    let n = safe_normalize(input.world_normal);
    let l = safe_normalize(environment.light_direction.xyz);
    let v = safe_normalize(camera.camera_position.xyz - input.world_position);
    let h = safe_normalize(v + l);

    let ndot_l = max(dot(n, l), 0.0);
    let ndot_v = max(dot(n, v), 1e-4);
    let ndot_h = max(dot(n, h), 0.0);
    let vdot_h = max(dot(v, h), 0.0);

    // glTF metallic workflow F0.
    let f0 = mix(vec3<f32>(DIELECTRIC_F0), base_rgba.rgb, metallic);
    // Energy-conscious diffuse/specular split: metallic surfaces carry no
    // diffuse term at all.
    let diffuse_albedo = base_rgba.rgb * (1.0 - metallic);

    // Specular BRDF: D * G * F / (4 * NdotV * NdotL). NdotL is clamped to
    // >= 0 but can be exactly zero, which would leave 0/0 through the Smith
    // geometry term, so the denominator carries an explicit positive floor.
    // NdotL itself is unchanged for the final direct-light multiplication.
    let alpha = roughness * roughness;
    let distribution = ggx_distribution(ndot_h, alpha);
    let geometry = smith_geometry(ndot_v, ndot_l, roughness);
    let fresnel = schlick_fresnel(f0, vdot_h);
    let specular_denominator = max(4.0 * ndot_v * ndot_l, 1e-4);
    let specular = min(
        distribution * geometry * fresnel / specular_denominator,
        vec3<f32>(SPECULAR_CLAMP),
    );

    // Direct lighting (see legacy-compat irradiance scale above).
    let irradiance = PI * environment.light_direction.w;
    let direct = (diffuse_albedo / PI + specular) * irradiance * ndot_l;

    // Ambient: applied predominantly to the diffuse (non-metal) response,
    // with the documented readability floor for metals. No fake IBL.
    let ambient_diffuse = diffuse_albedo * environment.ambient.xyz;
    let ambient_specular = f0 * environment.ambient.xyz * AMBIENT_SPECULAR_SCALE;
    let ambient = mix(ambient_diffuse, ambient_specular, metallic);

    let lit_rgb = direct + ambient;

    // Distance fog (after lighting, unchanged from G1B/G1C).
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
