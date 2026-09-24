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
// G2B adds one stable directional shadow map. G3A adds a textured terrain
// material (albedo/normal/roughness maps with world-space tiling) on a
// dedicated terrain pipeline that reuses this exact PBR response; the shared
// `lit_pbr_response` helper keeps the two fragment paths bit-compatible.
// G3A-R upgrades the terrain material to production visual closure: a full
// deterministic mip chain with trilinear + anisotropic sampling, a
// three-frequency stack (macro/base/detail world-space UVs), an
// anti-repetition rotated second sample, a distance-faded detail normal
// layer, multi-scale roughness, and presentation-only debug channels driven
// by a uniform selector (no shader recompiles).
// RV2-5 adds persistent physical atmosphere/IBL resources for the V2 path; the
// legacy analytic entry points remain selected for V1 and for the analytic
// fallback, and never reference the physical bindings.
//
// G3B adds an HDR outdoor lighting pipeline:
//   scene HDR pass (Rgba16Float, linear, no LDR clamp)
//   -> postprocess pass (exposure * Khronos PBR Neutral tone mapping)
//   -> sRGB display surface (hardware encode).
// The flat ambient term is replaced by a deterministic analytic sky model:
// a hemispherical sky-diffuse irradiance (coherent with world up and the
// procedural sky gradient) and a roughness/metallic-aware analytic
// environment specular response. The directional shadow map still modulates
// ONLY the direct sun term; sky light is never shadowed, so shadow sides
// stay readable without a fake flat fill. All new resources (HDR target,
// sampler, bind groups, postprocess pipeline) are created at startup or
// resize — never per frame.

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
// ambient.xyz:         reserved for the legacy flat ambient (kept at zero;
//                      replaced by the G3B sky-diffuse model).
// ambient.w:           reserved.
// sky_zenith.xyz:      zenith color (straight up).
// sky_zenith.w:        reserved.
// sky_horizon.xyz:     horizon / haze color.
// sky_horizon.w:       haze strength [0, 1].
// sky_ground.xyz:      below-horizon atmospheric color.
// sky_ground.w:        fog density (exponential fog coefficient).
// sun_color.xyz:       sun disk color.
// sun_color.w:         cosine of sun angular radius.
// sky_diffuse.xyz:     G3B sky-diffuse irradiance scale factor (applied to
//                      the hemispherical zenith/horizon/ground gradient).
// sky_diffuse.w:       sun-facing lift weight (diffuse irradiance boost on
//                      surfaces facing the sun, keeps the lit side alive).
// env_specular.xyz:    G3B environment specular color (analytic sky
//                      reflection tint, pre-wired for future prefiltered IBL).
// env_specular.w:      environment specular strength scaler.
// sun_transmittance.xyz: physical atmospheric attenuation shared by the direct
//                      sun term and the analytic sun disk.
// atmosphere_state.x:  RV2-5 sun angular radius (radians).
// atmosphere_state.y:  RV2-5 sun disk solid angle (sr); the physical direct
//                      irradiance is sun_color.xyz * atmosphere_state.y.
// atmosphere_state.z:  RV2-5 observer transmittance-LUT v coordinate (baked).
// atmosphere_state.w:  1 when the physical V2 entry points are bound, else 0.
struct EnvironmentUniform {
    light_direction: vec4<f32>,
    ambient: vec4<f32>,
    sky_zenith: vec4<f32>,
    sky_horizon: vec4<f32>,
    sky_ground: vec4<f32>,
    sun_color: vec4<f32>,
    sky_diffuse: vec4<f32>,
    env_specular: vec4<f32>,
    sun_transmittance: vec4<f32>,
    atmosphere_state: vec4<f32>,
};

// RV2-6 V2-only physical aerial-perspective state. The renderer stays in
// local metre coordinates; planet_ground.z is the render-space ground Y and
// the planet radius is used only by numerically stable local horizon math.
struct AerialPerspectiveUniform {
    // x: planet radius, y: atmosphere height, z: ground render Y,
    // w: fixed integration sample count (4).
    planet_ground: vec4<f32>,
    // x: Rayleigh scale height, y: Mie scale height,
    // z: ozone centre, w: ozone half-width.
    density_profile: vec4<f32>,
    rayleigh_scattering: vec4<f32>,
    mie_scattering: vec4<f32>,
    // xyz: Mie extinction, w: Mie anisotropy.
    mie_extinction_anisotropy: vec4<f32>,
    ozone_absorption: vec4<f32>,
    validation_control: vec4<f32>,
};

// G3B: postprocess state for the final display pass.
// exposure_ev: manual exposure in EV stops; the scene value is multiplied by
//   exp2(exposure_ev) BEFORE tone mapping (scene-referred HDR -> display).
//   Finite and bounded by the CPU-side validator.
struct PostProcessUniform {
    exposure_ev: f32,
    padding_0: f32,
    padding_1: f32,
    padding_2: f32,
};

// G3E: three-cascade directional shadow receiver state. Split distances are
// camera-to-receiver distances in render metres; each matrix addresses one
// persistent layer of the depth texture array. Receiver offsets remain
// separate from the caster rasterization bias configured by wgpu.
struct ShadowUniform {
    light_view_projection: array<mat4x4<f32>, 3>,
    split_distances_m: vec4<f32>,
    receiver_depth_bias: vec4<f32>,
    texel_size_uv: vec4<f32>,
};

// One matrix bound by each depth-only cascade pass. Keeping the caster state
// separate prevents sampling the array while one of its layers is attached.
struct ShadowCascadeUniform {
    light_view_projection: mat4x4<f32>,
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

// G3A-R: terrain material state (terrain pipeline only, group 4).
// metallic/roughness/normal_strength: PBR factors; roughness is multiplied by
//   the multi-scale roughness stack, normal_strength scales the tangent-space
//   XY of the base + detail normal stack.
// debug_mode: presentation-only channel selector (0 = FINAL).
// base/detail/macro_scale_m: three-frequency stack tile scales in metres.
// albedo_uv_offset: legacy field name for the common registered base-PBR UV
//   anchor used by albedo, normal, and roughness together.
// normal_uv_offset/roughness_uv_offset: reserved layout slots; production ENV1
//   sampling deliberately ignores them so the photographic triplet stays
//   registered. detail/macro offsets remain presentation-carrier anchors.
// ar_angle_cos_sin: (cos, sin) of the anti-repetition second-sample rotation.
// ar_scale_offset: (scale, offset.x, offset.y) of the rotated second sample.
// detail_fade_near_far: detail-layer distance fade range in metres (xy).
// padding: alignment to 128 bytes (eight vec4 slots).
struct TerrainMaterialUniform {
    metallic: f32,
    roughness: f32,
    normal_strength: f32,
    debug_mode: u32,
    base_scale_m: f32,
    detail_scale_m: f32,
    macro_scale_m: f32,
    padding1: f32,
    albedo_uv_offset: vec2<f32>,
    normal_uv_offset: vec2<f32>,
    roughness_uv_offset: vec2<f32>,
    detail_uv_offset: vec2<f32>,
    macro_uv_offset: vec2<f32>,
    ar_angle_cos_sin: vec2<f32>,
    ar_scale_offset: vec4<f32>,
    detail_fade_near_far: vec4<f32>,
    padding2: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

@group(1) @binding(0)
var<uniform> object: ObjectUniform;

@group(2) @binding(0)
var<uniform> environment: EnvironmentUniform;
@group(2) @binding(1)
var directional_shadow_depth: texture_depth_2d_array;
@group(2) @binding(2)
var directional_shadow_sampler: sampler_comparison;
@group(2) @binding(3)
var<uniform> shadow: ShadowUniform;
@group(2) @binding(4)
var<uniform> shadow_cascade: ShadowCascadeUniform;
@group(2) @binding(5)
var<uniform> aerial_perspective: AerialPerspectiveUniform;
@group(2) @binding(12)
var environment_cube: texture_cube<f32>;
@group(2) @binding(13)
var prefiltered_environment_cube: texture_cube<f32>;
@group(2) @binding(14)
var irradiance_cube: texture_cube<f32>;
@group(2) @binding(15)
var transmittance_lut: texture_2d<f32>;
@group(2) @binding(16)
var multi_scattering_lut: texture_2d<f32>;
@group(2) @binding(17)
var sky_view_lut: texture_2d<f32>;
@group(2) @binding(18)
var environment_sampler: sampler;
@group(2) @binding(19)
var atmosphere_sampler: sampler;
@group(2) @binding(20)
var brdf_lut: texture_2d<f32>;

// G1C: Material texture and sampler.
// Group 3 is the material bind group, containing the base color texture,
// sampler, and (G1D) the metallic/roughness uniform.
@group(3) @binding(0)
var base_color_texture: texture_2d<f32>;
@group(3) @binding(1)
var base_color_sampler: sampler;
@group(3) @binding(2)
var<uniform> material: MaterialUniform;

// G3A: terrain detail maps (bound only by the dedicated terrain pipeline,
// group 4). Albedo is sRGB (hardware converts on sampling); normal and
// roughness are linear data. One sampler serves all three maps.
@group(4) @binding(0)
var terrain_albedo_texture: texture_2d<f32>;
@group(4) @binding(1)
var terrain_sampler: sampler;
@group(4) @binding(2)
var terrain_normal_texture: texture_2d<f32>;
@group(4) @binding(3)
var terrain_roughness_texture: texture_2d<f32>;
@group(4) @binding(4)
var<uniform> terrain_material: TerrainMaterialUniform;

// FFV1: the companion ground materials of the field composition, bound in the
// same terrain group so the region blend needs no extra bind group. The worn
// layer (Poly Haven grass_path_3, compacted/worn grass, 1.0 m scan) carries
// albedo + normal only: compacted traffic reads through a roughness offset on
// the maintained stack, which keeps the terrain fragment stage inside the
// 16-sampled-texture per-stage limit. `dry` is Poly Haven forest_ground_04
// (dry exposed soil, 3.15 m scan); the maintained base layer stays Poly Haven
// sparse_grass (2.0 m scan).
@group(4) @binding(5)
var terrain_worn_albedo_texture: texture_2d<f32>;
@group(4) @binding(6)
var terrain_worn_normal_texture: texture_2d<f32>;
@group(4) @binding(7)
var terrain_dry_albedo_texture: texture_2d<f32>;
@group(4) @binding(8)
var terrain_dry_normal_texture: texture_2d<f32>;
@group(4) @binding(9)
var terrain_dry_roughness_texture: texture_2d<f32>;

// G3B: HDR scene target + postprocess state. This group belongs to the
// dedicated fullscreen postprocess pipeline (its own layout); the scene
// passes never bind it, so the HDR texture/state stay out of the lighting
// paths. The sampler is nearest (1:1 texel mapping, deterministic) and the
// uniform buffer is the only postprocess state written per frame.
@group(5) @binding(0)
var hdr_scene_texture: texture_2d<f32>;
@group(5) @binding(1)
var hdr_scene_sampler: sampler;
@group(5) @binding(2)
var<uniform> postprocess: PostProcessUniform;

// ---------------------------------------------------------------------------
// Vertex IO
// ---------------------------------------------------------------------------

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
};

// RV2-3 rigid GLB scene input. Locations 4-7 are the loader-provided node
// world transform; 8-10 are its inverse-transpose normal transform. Both are
// static instance-rate data. The dynamic aircraft root remains `object.model`.
struct GpuSceneVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) instance_model_0: vec4<f32>,
    @location(5) instance_model_1: vec4<f32>,
    @location(6) instance_model_2: vec4<f32>,
    @location(7) instance_model_3: vec4<f32>,
    @location(8) instance_normal_0: vec4<f32>,
    @location(9) instance_normal_1: vec4<f32>,
    @location(10) instance_normal_2: vec4<f32>,
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
    // exponentially away from it. G3-VR1: gentler falloff widens the haze
    // band so terrain and sky meet through a gradation instead of a seam.
    let haze_strength = environment.sky_horizon.w;
    let haze_falloff = exp(-abs(elevation) * 4.0);
    sky = mix(sky, horizon, haze_falloff * haze_strength);

    // Sun disk: procedural, coherent with the directional light.
    let sun_dir = normalize(environment.light_direction.xyz);
    let sun_alignment = dot(view_dir, sun_dir);
    let sun_cos_radius = environment.sun_color.w;
    // Smooth disk edge over a tiny angular band.
    let disk = smoothstep(sun_cos_radius - 0.0005, sun_cos_radius + 0.0005, sun_alignment);
    // Subtle halo: fades from disk edge outward.
    let halo = smoothstep(sun_cos_radius - 0.06, sun_cos_radius - 0.005, sun_alignment) * 0.25;
    // G3B: no LDR clamp — the sun disk and haze stay scene-referred so the
    // postprocess exposure + tone mapper own the final display range.
    sky = sky + environment.sun_color.xyz * (disk + halo);

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

// G3E depth-only shadow caster vertex path. Each pass binds one persistent
// cascade matrix while preserving the caster's normal object transform.
@vertex
fn vs_shadow(input: VertexInput) -> @builtin(position) vec4<f32> {
    let world_position = object.model * vec4<f32>(input.position, 1.0);
    return shadow_cascade.light_view_projection * world_position;
}

// RV2-3 forward-HDR vertex entry. Mesh-local geometry is shared across scene
// nodes; the final transform is exactly aircraft_root * GLB_world_transform.
@vertex
fn vs_gpu_scene(input: GpuSceneVertexInput) -> VertexOutput {
    let instance_model = mat4x4<f32>(
        input.instance_model_0,
        input.instance_model_1,
        input.instance_model_2,
        input.instance_model_3,
    );
    let instance_normal = mat3x3<f32>(
        input.instance_normal_0.xyz,
        input.instance_normal_1.xyz,
        input.instance_normal_2.xyz,
    );
    let world_position = object.model * instance_model * vec4<f32>(input.position, 1.0);
    var output: VertexOutput;
    output.clip_position = camera.view_projection * world_position;
    output.world_normal = normalize(normal_matrix(object.model) * instance_normal * input.normal);
    output.color = input.color;
    output.uv = input.uv;
    output.world_position = world_position.xyz;
    return output;
}

// RV2-3 uses the identical instance/root composition in every shadow cascade.
@vertex
fn vs_gpu_scene_shadow(input: GpuSceneVertexInput) -> @builtin(position) vec4<f32> {
    let instance_model = mat4x4<f32>(
        input.instance_model_0,
        input.instance_model_1,
        input.instance_model_2,
        input.instance_model_3,
    );
    let world_position = object.model * instance_model * vec4<f32>(input.position, 1.0);
    return shadow_cascade.light_view_projection * world_position;
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

// G3E: deterministic cascade selection by camera-to-receiver distance.
// Beyond the documented far split the scene stays lit rather than smearing the
// last depth texel over unrepresented terrain.
fn shadow_cascade_index(view_distance_m: f32) -> i32 {
    if (view_distance_m <= shadow.split_distances_m.x) {
        return 0;
    }
    if (view_distance_m <= shadow.split_distances_m.y) {
        return 1;
    }
    if (view_distance_m <= shadow.split_distances_m.z) {
        return 2;
    }
    return -1;
}

// G3-VR1: 5x5 percentage-closer filter — twenty-five comparison taps for
// every shadowed receiver, independent of cascade and scene content. The
// wider kernel softens the penumbra without PCSS-style variability, and the
// cost stays bounded on the reference GPU.
const SHADOW_PCF_TAPS: i32 = 2;
const SHADOW_PCF_TAP_COUNT: f32 = 25.0;
fn pcf_shadow_visibility(world_position: vec3<f32>, cascade_index: u32) -> f32 {
    let light_clip = shadow.light_view_projection[cascade_index]
        * vec4<f32>(world_position, 1.0);
    if (light_clip.w <= 1e-6) {
        return 1.0;
    }
    let projected = light_clip.xyz / light_clip.w;
    let uv = projected.xy * 0.5 + vec2<f32>(0.5);
    let inside_shadow_frustum =
        uv.x >= 0.0 && uv.x <= 1.0 &&
        uv.y >= 0.0 && uv.y <= 1.0 &&
        projected.z >= 0.0 && projected.z <= 1.0;
    if (!inside_shadow_frustum) {
        return 1.0;
    }
    let receiver_depth = clamp(
        projected.z - shadow.receiver_depth_bias[cascade_index],
        0.0,
        1.0,
    );
    let texel = shadow.texel_size_uv.xy;
    var visibility = 0.0;
    for (var y = -SHADOW_PCF_TAPS; y <= SHADOW_PCF_TAPS; y = y + 1) {
        for (var x = -SHADOW_PCF_TAPS; x <= SHADOW_PCF_TAPS; x = x + 1) {
            visibility = visibility + textureSampleCompare(
                directional_shadow_depth,
                directional_shadow_sampler,
                uv + vec2<f32>(f32(x), f32(y)) * texel,
                i32(cascade_index),
                receiver_depth,
            );
        }
    }
    return visibility / SHADOW_PCF_TAP_COUNT;
}

fn directional_shadow_visibility(world_position: vec3<f32>) -> f32 {
    let view_distance_m = distance(world_position, camera.camera_position.xyz);
    let cascade_index = shadow_cascade_index(view_distance_m);
    if (cascade_index < 0) {
        return 1.0;
    }
    // G3-VR1.1: no penumbra floor. G3B's sky-diffuse / env-specular ambient
    // already lifts shadowed surfaces; multiplying direct light by a 0.30
    // floor washed contact shadows and tree shadows out. PCF 5x5 stays as the
    // softness mechanism.
    return pcf_shadow_visibility(world_position, u32(cascade_index));
}

// ---------------------------------------------------------------------------
// G3B: analytic sky response
// ---------------------------------------------------------------------------

// Hemispherical sky-diffuse irradiance.
//
// Builds a world-up hemisphere from the same zenith / horizon / ground
// gradient as the visible procedural sky, scaled by the uniform irradiance
// factor, with a subtle lift on sun-facing normals. It is directionally
// coherent with the outdoor sky, never shadowed, and never clamps: shadow
// sides keep a plausible blue-grey fill while unshadowed surfaces stay
// sun-driven. Slots for future prefiltered IBL: replace this function's body
// with a probe sample while keeping the EnvironmentUniform contract.
fn sky_diffuse_irradiance(n: vec3<f32>) -> vec3<f32> {
    let ndot_up = clamp(dot(n, WORLD_UP), 0.0, 1.0);
    // Sky gradient between horizon and zenith above ground (same power curve
    // as the visible sky), and horizon-to-ground below.
    var hemisphere: vec3<f32>;
    if (ndot_up >= 0.0) {
        hemisphere = mix(environment.sky_horizon.xyz, environment.sky_zenith.xyz, pow(ndot_up, 0.5));
    } else {
        hemisphere = mix(environment.sky_horizon.xyz, environment.sky_ground.xyz, pow(-ndot_up, 0.7));
    }
    let sun_dir = safe_normalize(environment.light_direction.xyz);
    let sun_lift = environment.sky_diffuse.w * clamp(dot(n, sun_dir), 0.0, 1.0);
    return hemisphere * environment.sky_diffuse.xyz * (1.0 + sun_lift);
}

// Roughness/metallic-aware analytic environment specular response.
//
// Uses the Schlick Fresnel through the same PBR path as the direct term, so
// metals (F0 ~ albedo) reflect the sky color strongly while dielectrics stay
// subtle; a GGX-like lobe weight (smoothness^2) makes sharp surfaces reflect
// more and rough surfaces fade toward the diffuse response. Deterministic and
// documentable — the analytic counterpart of the physical split-sum IBL.
fn environment_specular_response(
    f0: vec3<f32>,
    roughness: f32,
    ndot_v: f32,
) -> vec3<f32> {
    let smoothness = clamp(1.0 - roughness, 0.0, 1.0);
    let lobe_weight = smoothness * smoothness;
    let fresnel = schlick_fresnel(f0, clamp(ndot_v, 0.0, 1.0));
    let env_color = mix(environment.sky_horizon.xyz, environment.sky_zenith.xyz, 0.5);
    return fresnel * env_color * environment.env_specular.xyz * lobe_weight * environment.env_specular.w;
}

// ---------------------------------------------------------------------------
// RV2-5 physical image-based lighting
// ---------------------------------------------------------------------------
//
// These helpers are only reachable from the V2 entry points (`fs_lit_v2`,
// `fs_terrain_v2`, `fs_vegetation_v2`, `fs_sky_v2`), which wgpu validates
// against the V2-only environment layout. The analytic/V1 entry points never
// reference their bindings, so V1 keeps its previous layout and output.

/// Diffuse IBL: the cosine-weighted irradiance cubemap.
fn physical_irradiance(n: vec3<f32>) -> vec3<f32> {
    return textureSample(irradiance_cube, environment_sampler, safe_normalize(n)).rgb;
}

/// Specular IBL: prefiltered radiance at the roughness mip times the split-sum
/// BRDF integration (`F0 * A + B`). Never multiplied by the directional shadow.
fn physical_specular_response(
    f0: vec3<f32>,
    roughness: f32,
    ndot_v: f32,
    n: vec3<f32>,
    world_position: vec3<f32>,
) -> vec3<f32> {
    let v = safe_normalize(camera.camera_position.xyz - world_position);
    let reflected = safe_normalize(2.0 * dot(n, v) * n - v);
    // roughness 0 -> mip 0, roughness 1 -> mip 7, continuous and clamped.
    let mip = clamp(roughness, 0.0, 1.0) * 7.0;
    let prefiltered = textureSampleLevel(
        prefiltered_environment_cube,
        environment_sampler,
        reflected,
        mip,
    ).rgb;
    let brdf = textureSample(
        brdf_lut,
        atmosphere_sampler,
        vec2<f32>(clamp(ndot_v, 0.0, 1.0), clamp(roughness, 0.0, 1.0)),
    ).rg;
    let fresnel = schlick_fresnel(f0, clamp(ndot_v, 0.0, 1.0));
    return prefiltered * (fresnel * brdf.x + brdf.y);
}

// ---------------------------------------------------------------------------
// RV2-6 physical aerial perspective
// ---------------------------------------------------------------------------
//
// Four deterministic midpoint samples integrate the bounded RC-scale segment
// directly in the Scene pass. The result is physical compositing:
//
//   Lout = Lsurface * T(camera -> fragment) + Lscattered(camera -> fragment)
//
// The RV2-5 multiple-scattering texture stores a sun-independent transfer.
// The real SunState irradiance is therefore multiplied exactly once below.

const AERIAL_PERSPECTIVE_SAMPLE_COUNT: i32 = 4;

struct PhysicalAerialPerspective {
    transmittance_rgb: vec3<f32>,
    inscattered_radiance_rgb: vec3<f32>,
};

fn aerial_altitude(world_position: vec3<f32>) -> f32 {
    return clamp(
        world_position.y - aerial_perspective.planet_ground.z,
        0.0,
        aerial_perspective.planet_ground.y,
    );
}

fn aerial_rayleigh_density(height_m: f32) -> f32 {
    return exp(-height_m / aerial_perspective.density_profile.x);
}

fn aerial_mie_density(height_m: f32) -> f32 {
    return exp(-height_m / aerial_perspective.density_profile.y);
}

fn aerial_ozone_density(height_m: f32) -> f32 {
    return max(
        0.0,
        1.0
            - abs(height_m - aerial_perspective.density_profile.z)
                / aerial_perspective.density_profile.w,
    );
}

fn aerial_extinction(height_m: f32) -> vec3<f32> {
    return aerial_perspective.rayleigh_scattering.rgb * aerial_rayleigh_density(height_m)
        + aerial_perspective.mie_extinction_anisotropy.rgb * aerial_mie_density(height_m)
        + aerial_perspective.ozone_absorption.rgb * aerial_ozone_density(height_m);
}

fn aerial_scattering(height_m: f32) -> vec3<f32> {
    return aerial_perspective.rayleigh_scattering.rgb * aerial_rayleigh_density(height_m)
        + aerial_perspective.mie_scattering.rgb * aerial_mie_density(height_m);
}

fn aerial_rayleigh_phase(cos_theta: f32) -> f32 {
    return 3.0 / (16.0 * PI) * (1.0 + cos_theta * cos_theta);
}

fn aerial_mie_phase(cos_theta: f32) -> f32 {
    let g = aerial_perspective.mie_extinction_anisotropy.w;
    let denominator = max(1.0 + g * g - 2.0 * g * cos_theta, 1e-4);
    return (1.0 - g * g) / (4.0 * PI * pow(denominator, 1.5));
}

fn aerial_sample_up(sample_position: vec3<f32>, height_m: f32) -> vec3<f32> {
    // Local tangent-frame construction: no Earth-scale world translation or
    // subtraction. The large radius only establishes the slowly varying up.
    return safe_normalize(vec3<f32>(
        sample_position.x,
        aerial_perspective.planet_ground.x + height_m,
        sample_position.z,
    ));
}

fn aerial_sun_is_visible(height_m: f32, sun_mu: f32) -> bool {
    if (sun_mu >= 0.0) {
        return true;
    }
    // Stable equivalent of the ray/sphere horizon test:
    // sqrt(1 - R^2/(R+h)^2) = sqrt(h(2R+h))/(R+h).
    // It preserves the exact surface decision without subtracting R-sized
    // f32 values and naturally permits the depressed horizon at altitude.
    let planet_radius = aerial_perspective.planet_ground.x;
    let sample_radius = planet_radius + height_m;
    let horizon_sine = sqrt(max(height_m * (2.0 * planet_radius + height_m), 0.0))
        / max(sample_radius, 1.0);
    return sun_mu >= -horizon_sine;
}

fn aerial_lut_uv(height_m: f32, sun_mu: f32) -> vec2<f32> {
    return vec2<f32>(
        clamp(sun_mu * 0.5 + 0.5, 0.0, 1.0),
        sqrt(clamp(height_m / aerial_perspective.planet_ground.y, 0.0, 1.0)),
    );
}

fn integrate_physical_aerial_perspective(
    camera_position: vec3<f32>,
    world_position: vec3<f32>,
) -> PhysicalAerialPerspective {
    let segment = world_position - camera_position;
    let distance = length(segment);
    if (distance <= 1e-4) {
        return PhysicalAerialPerspective(vec3<f32>(1.0), vec3<f32>(0.0));
    }

    let view_direction = segment / distance;
    let sun_direction = safe_normalize(environment.light_direction.xyz);
    let cos_theta = clamp(dot(view_direction, sun_direction), -1.0, 1.0);
    let rayleigh_phase = aerial_rayleigh_phase(cos_theta);
    let mie_phase = aerial_mie_phase(cos_theta);
    let sun_irradiance = environment.sun_color.xyz * environment.atmosphere_state.y;
    let step_length = distance / f32(AERIAL_PERSPECTIVE_SAMPLE_COUNT);
    var optical_depth = vec3<f32>(0.0);
    var inscattered = vec3<f32>(0.0);

    for (
        var sample_index = 0;
        sample_index < AERIAL_PERSPECTIVE_SAMPLE_COUNT;
        sample_index = sample_index + 1
    ) {
        let sample_distance = (f32(sample_index) + 0.5) * step_length;
        let sample_position = camera_position + view_direction * sample_distance;
        let height_m = aerial_altitude(sample_position);
        let rho_rayleigh = aerial_rayleigh_density(height_m);
        let rho_mie = aerial_mie_density(height_m);
        let extinction = aerial_extinction(height_m);
        let transmittance_to_sample = exp(
            -(optical_depth + extinction * (0.5 * step_length)),
        );

        let up = aerial_sample_up(sample_position, height_m);
        let sun_mu = clamp(dot(up, sun_direction), -1.0, 1.0);
        var sun_transmittance = vec3<f32>(0.0);
        if (aerial_sun_is_visible(height_m, sun_mu)) {
            sun_transmittance = textureSampleLevel(
                transmittance_lut,
                atmosphere_sampler,
                aerial_lut_uv(height_m, sun_mu),
                0.0,
            ).rgb;
        }

        let single_source = sun_irradiance * sun_transmittance
            * (aerial_perspective.rayleigh_scattering.rgb * rho_rayleigh * rayleigh_phase
                + aerial_perspective.mie_scattering.rgb * rho_mie * mie_phase);
        let multiple_transfer = textureSampleLevel(
            multi_scattering_lut,
            atmosphere_sampler,
            aerial_lut_uv(height_m, sun_mu),
            0.0,
        ).rgb;
        let multiple_source = sun_irradiance * multiple_transfer * aerial_scattering(height_m);

        inscattered = inscattered
            + transmittance_to_sample * (single_source + multiple_source) * step_length;
        optical_depth = optical_depth + extinction * step_length;
    }

    return PhysicalAerialPerspective(
        clamp(exp(-optical_depth), vec3<f32>(0.0), vec3<f32>(1.0)),
        max(inscattered, vec3<f32>(0.0)),
    );
}

fn apply_physical_aerial_perspective(
    surface_radiance: vec3<f32>,
    world_position: vec3<f32>,
) -> vec3<f32> {
    if (aerial_perspective.validation_control.x < 0.5) {
        return surface_radiance;
    }
    let physical = integrate_physical_aerial_perspective(
        camera.camera_position.xyz,
        world_position,
    );
    return surface_radiance * physical.transmittance_rgb
        + physical.inscattered_radiance_rgb;
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
//   sky diffuse / env specular as configured in `EnvironmentUniform`
//   direction     = normalize(vec3(0.4, 0.8, -0.3))  (above, right, slightly forward)
//   intensity     = 0.80
//   fog_density   = 0.0015
//   fog_color     = sky_horizon color
// G3A: shared PBR response used by both fragment paths (`fs_lit` and
// `fs_terrain`). Returns the lit color BEFORE fog. The operands and order
// exactly mirror the pre-G3A `fs_lit` body, so the established look is
// bit-compatible. This is the analytic/V1 path; `lit_pbr_response_physical`
// below is the RV2-5 split-sum IBL counterpart.
fn lit_pbr_response(
    base_rgba: vec4<f32>,
    n: vec3<f32>,
    world_position: vec3<f32>,
    metallic: f32,
    roughness: f32,
) -> vec3<f32> {
    // BRDF basis vectors (all guarded against zero-length inputs).
    let l = safe_normalize(environment.light_direction.xyz);
    let v = safe_normalize(camera.camera_position.xyz - world_position);
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
    let direct_unshadowed = (diffuse_albedo / PI + specular) * irradiance * ndot_l;
    let shadow_visibility = directional_shadow_visibility(world_position);
    let direct = direct_unshadowed * shadow_visibility;

    // G3B analytic sky response (replaces the legacy flat ambient).
    // Sky diffuse: hemispherical irradiance built from the same zenith /
    // horizon / ground gradient as the visible sky, lifted slightly on
    // sun-facing normals. It is directional in world-up, never shadowed, and
    // keeps the shadow side readable without washing the aircraft out.
    // Sky specular: roughness-aware analytic environment response through the
    // PBR path — metals (high F0) reflect strongly, dielectric clearcoat stays
    // subtle, and sharper surfaces (low roughness) are boosted.
    let ambient_diffuse = diffuse_albedo * sky_diffuse_irradiance(n);
    let ambient_specular = environment_specular_response(f0, roughness, ndot_v);
    let ambient = mix(ambient_diffuse, ambient_specular, metallic);

    let lit_rgb = direct + ambient;
    return lit_rgb;
}

// RV2-5 physical PBR response.
//
// Identical direct term, driven by the single SunState:
//   direct = sun BRDF * NdotL * SunState.irradiance * atmospheric
//            transmittance * shadow
//   ambient = diffuse IBL (irradiance cubemap) + specular IBL
//             (prefiltered cube at the roughness mip * split-sum BRDF)
// The IBL terms are additive and are never multiplied by the directional
// shadow. No analytic environment term is added in physical mode.
fn lit_pbr_response_physical(
    base_rgba: vec4<f32>,
    n: vec3<f32>,
    world_position: vec3<f32>,
    metallic: f32,
    roughness: f32,
) -> vec3<f32> {
    let l = safe_normalize(environment.light_direction.xyz);
    let v = safe_normalize(camera.camera_position.xyz - world_position);
    let h = safe_normalize(v + l);

    let ndot_l = max(dot(n, l), 0.0);
    let ndot_v = max(dot(n, v), 1e-4);
    let ndot_h = max(dot(n, h), 0.0);
    let vdot_h = max(dot(v, h), 0.0);

    let f0 = mix(vec3<f32>(DIELECTRIC_F0), base_rgba.rgb, metallic);
    let diffuse_albedo = base_rgba.rgb * (1.0 - metallic);

    let alpha = roughness * roughness;
    let distribution = ggx_distribution(ndot_h, alpha);
    let geometry = smith_geometry(ndot_v, ndot_l, roughness);
    let fresnel = schlick_fresnel(f0, vdot_h);
    let specular_denominator = max(4.0 * ndot_v * ndot_l, 1e-4);
    let specular = min(
        distribution * geometry * fresnel / specular_denominator,
        vec3<f32>(SPECULAR_CLAMP),
    );

    // Sun irradiance from the single SunState: disk radiance * solid angle.
    let irradiance = environment.sun_color.xyz * environment.atmosphere_state.y;
    let direct_unshadowed = (diffuse_albedo / PI + specular) * irradiance * ndot_l
        * environment.sun_transmittance.rgb;
    let shadow_visibility = directional_shadow_visibility(world_position);
    let direct = direct_unshadowed * shadow_visibility;

    let ambient_diffuse = diffuse_albedo * physical_irradiance(n);
    let ambient_specular = physical_specular_response(f0, roughness, ndot_v, n, world_position);
    let ambient = ambient_diffuse + ambient_specular;

    let lit_rgb = direct + ambient;
    return lit_rgb;
}

// Distance fog for a world position, shared by both lit fragment paths.
fn apply_distance_fog(lit_rgb: vec3<f32>, world_position: vec3<f32>) -> vec3<f32> {
    let camera_pos = camera.camera_position.xyz;
    let distance = length(world_position - camera_pos);
    let density = environment.sky_ground.w;
    let fog = fog_factor(distance, density);
    let fog_color = environment.sky_horizon.xyz;
    let final_rgb = mix(lit_rgb, fog_color, fog);
    return final_rgb;
}

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

    let n = safe_normalize(input.world_normal);
    let lit_rgb = lit_pbr_response(base_rgba, n, input.world_position, metallic, roughness);
    let final_rgb = apply_distance_fog(lit_rgb, input.world_position);

    return vec4<f32>(final_rgb, base_rgba.a);
}

// RV2 physical counterpart of `fs_lit`. Material resolution is unchanged;
// physical lighting is followed by RV2-6 transmittance + in-scattering.
@fragment
fn fs_lit_v2(input: VertexOutput) -> @location(0) vec4<f32> {
    let texture_rgba = textureSample(base_color_texture, base_color_sampler, input.uv);
    let base_rgba = input.color * texture_rgba;

    let metallic = clamp(material.metallic, 0.0, 1.0);
    let roughness = clamp(material.roughness, MIN_ROUGHNESS, 1.0);

    let n = safe_normalize(input.world_normal);
    let lit_rgb = lit_pbr_response_physical(base_rgba, n, input.world_position, metallic, roughness);
    let final_rgb = apply_physical_aerial_perspective(lit_rgb, input.world_position);

    return vec4<f32>(final_rgb, base_rgba.a);
}
// Same PBR + G2B shadow + fog pipeline as `fs_lit`, preceded by the
// production grass detail stack:
//   1. three-frequency albedo stack (macro tone / base + anti-repetition
//      rotated second sample / distance-faded detail grain) x vertex color
//      (G2D macro variation carrier);
//   2. tangent-space normal stack (base + distance-faded detail) perturbing
//      the geometric normal through a fragment TBN reconstructed from
//      screen-space derivatives, with high-frequency detail fading with
//      distance so oblique views stay stable (no shimmer, no popping);
//   3. multi-scale roughness stack (base + macro + detail) scaling the
//      material base roughness.
// The TBN is fragment-local (derivative-based), so the terrain needs no
// per-vertex tangent attribute and no buffer-layout change; it is exact on
// the flat plane and falls back to the world-aligned frame on degenerate
// fragments, keeping the math finite. All UVs are world-anchored, so the
// whole stack is invariant to camera and chunking.
//
// RV2-5: the material stack is shared through `terrain_surface`, while the two
// entry points select the analytic (`fs_terrain`, V1) or the physical
// split-sum IBL response (`fs_terrain_v2`). Keeping the physical sampling out
// of the shared helpers means a V1 pipeline never references the RV2-5
// bindings at all.
struct TerrainSurface {
    base_rgba: vec4<f32>,
    albedo: vec4<f32>,
    n: vec3<f32>,
    n_ts: vec3<f32>,
    metallic: f32,
    roughness: f32,
    albedo_macro_s: vec4<f32>,
    albedo_detail_s: vec4<f32>,
    region_weights: vec4<f32>,
};

fn normalize_tangent_normal(normal: vec3<f32>) -> vec3<f32> {
    let length_squared = dot(normal, normal);
    if (length_squared <= 1e-12) {
        return vec3<f32>(0.0, 0.0, 1.0);
    }
    return normal * inverseSqrt(length_squared);
}

// A secondary sample uses UV_B = R(theta) * UV_A * scale + offset. Its
// tangent-space normal is expressed in B's rotated texture basis, so XY must
// be transformed by R(-theta) before blending in the primary terrain frame.
fn reorient_secondary_tangent_normal(
    normal: vec3<f32>,
    angle_cos: f32,
    angle_sin: f32,
) -> vec3<f32> {
    return normalize_tangent_normal(vec3<f32>(
        angle_cos * normal.x + angle_sin * normal.y,
        -angle_sin * normal.x + angle_cos * normal.y,
        normal.z,
    ));
}

// ---------------------------------------------------------------------------
// FFV1: deterministic field region composition
// ---------------------------------------------------------------------------
//
// The field is composed from four spatial regions evaluated analytically from
// the world-anchored fragment position: the mown grass runway, a worn /
// compacted collar plus meso wear paths, the maintained mown field, and a
// drier rougher band under the vegetation boundary. Every boundary is a
// signed-distance band perturbed by two octaves of value noise, so the
// transitions are irregular and never geometric bands, and the whole mask is a
// pure function of world XZ: chunk-independent, frame-independent and
// reproducible bit-for-bit (mirrored in Rust by
// `terrain::field_region_weights`).
//
// Layout constants are world metres and mirror `terrain.rs`.

const FIELD_RUNWAY_HALF: vec2<f32> = vec2<f32>(6.0, 60.0);
const FIELD_RUNWAY_RADIUS: f32 = 6.0;
const FIELD_WORN_COLLAR_M: f32 = 5.0;
const FIELD_WORN_COLLAR_BAND_M: f32 = 2.0;
const FIELD_MOWN_HALF: vec2<f32> = vec2<f32>(34.0, 118.0);
const FIELD_MOWN_RADIUS: f32 = 30.0;
const FIELD_EDGE_BOX_HALF: vec2<f32> = vec2<f32>(150.0, 150.0);
const FIELD_EDGE_BOX_RADIUS: f32 = 60.0;
const FIELD_EDGE_INNER_M: f32 = 30.0;
const FIELD_EDGE_OUTER_M: f32 = 26.0;
const FIELD_MOW_LAP_M: f32 = 2.4;

fn field_sd_round_box(p: vec2<f32>, half: vec2<f32>, radius: f32) -> f32 {
    let q = abs(p) - half + vec2<f32>(radius);
    return length(max(q, vec2<f32>(0.0))) + min(max(q.x, q.y), 0.0) - radius;
}

fn field_hash21(p: vec2<f32>) -> f32 {
    let q = fract(p * vec2<f32>(0.1031, 0.1030));
    let h = q * (q.yx + vec2<f32>(33.71, 19.63));
    return fract((h.x + h.y) * 37.91);
}

fn field_value_noise(p: vec2<f32>) -> f32 {
    let cell = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = field_hash21(cell);
    let b = field_hash21(cell + vec2<f32>(1.0, 0.0));
    let c = field_hash21(cell + vec2<f32>(0.0, 1.0));
    let d = field_hash21(cell + vec2<f32>(1.0, 1.0));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

fn field_fbm(p: vec2<f32>) -> f32 {
    return 0.65 * field_value_noise(p)
        + 0.35 * field_value_noise(p * 2.7 + vec2<f32>(17.3, 9.1));
}

/// Region weights (mown runway, worn, maintained field, dry edge). They form a
/// partition of unity up to the clamps; the material blend normalizes anyway.
fn field_region_weights(p: vec2<f32>) -> vec4<f32> {
    let n = field_fbm(p * 0.11);
    let n_fine = field_value_noise(p * 0.32 + vec2<f32>(4.7, 11.3));
    let jitter = (n - 0.5) * 2.0;
    let d_runway = field_sd_round_box(p, FIELD_RUNWAY_HALF, FIELD_RUNWAY_RADIUS)
        + jitter * 2.5 + (n_fine - 0.5) * 1.6;
    let mown_strip = 1.0 - smoothstep(-1.2, 1.8, d_runway);
    let worn_collar = 1.0 - smoothstep(
        FIELD_WORN_COLLAR_M - FIELD_WORN_COLLAR_BAND_M,
        FIELD_WORN_COLLAR_M + FIELD_WORN_COLLAR_BAND_M,
        d_runway + jitter * 3.0,
    );
    let d_mown = field_sd_round_box(p, FIELD_MOWN_HALF, FIELD_MOWN_RADIUS) + jitter * 7.0;
    let mown_field = 1.0 - smoothstep(-6.0, 8.0, d_mown);
    let d_edge = field_sd_round_box(p, FIELD_EDGE_BOX_HALF, FIELD_EDGE_BOX_RADIUS)
        + jitter * 26.0;
    let edge = smoothstep(-FIELD_EDGE_INNER_M, FIELD_EDGE_OUTER_M, d_edge);
    // Meso wear paths inside the mown field: a rare gated noise ridge, never
    // on the mown strip, so foot/maintenance traffic reads at 1-10 m scale.
    let path_gate = smoothstep(
        0.75,
        0.82,
        field_value_noise(p * 0.16 + vec2<f32>(61.7, 23.9)),
    );
    let runway = clamp(mown_strip, 0.0, 1.0);
    // Wear paths may only fill the headroom the runway leaves, so the four
    // weights stay a partition of unity (the blend normalizes regardless).
    let worn = min(
        clamp(worn_collar - mown_strip, 0.0, 1.0)
            + path_gate * mown_field * (1.0 - mown_strip) * 0.45,
        1.0 - runway,
    );
    let dry = clamp(edge, 0.0, 1.0);
    let field = clamp(1.0 - runway - worn - dry, 0.0, 1.0);
    return vec4<f32>(runway, worn, field, dry);
}

/// Mown-lap modulation across the runway axis: real mown grass shows parallel
/// laps a couple of metres apart, brightest on the maintained strip.
fn field_mow_stripe(p: vec2<f32>) -> f32 {
    return 0.5 + 0.5 * sin(p.x * (6.2831853 / FIELD_MOW_LAP_M));
}

@fragment
fn fs_terrain(input: VertexOutput) -> @location(0) vec4<f32> {
    let surface = terrain_surface(input);
    let lit_rgb = lit_pbr_response(
        surface.base_rgba,
        surface.n,
        input.world_position,
        surface.metallic,
        surface.roughness,
    );
    let final_rgb = apply_distance_fog(lit_rgb, input.world_position);
    return terrain_fragment_output(surface, final_rgb);
}

@fragment
fn fs_terrain_v2(input: VertexOutput) -> @location(0) vec4<f32> {
    let surface = terrain_surface(input);
    let lit_rgb = lit_pbr_response_physical(
        surface.base_rgba,
        surface.n,
        input.world_position,
        surface.metallic,
        surface.roughness,
    );
    let final_rgb = apply_physical_aerial_perspective(lit_rgb, input.world_position);
    return terrain_fragment_output(surface, final_rgb);
}

fn terrain_surface(input: VertexOutput) -> TerrainSurface {
    // ENV1-B0: `uv` is world position X/Z divided by the physical 2 m scan
    // span. `base_uv` is the single registered coordinate for the base albedo,
    // normal, and roughness triplet. Macro/detail remain inherited
    // presentation carriers with their own absolute-world scales.
    let uv = input.uv;
    let base_uv = uv + terrain_material.albedo_uv_offset;
    let base_scale = terrain_material.base_scale_m;
    let macro_uv = uv * (base_scale / terrain_material.macro_scale_m)
        + terrain_material.macro_uv_offset;
    let detail_uv = uv * (base_scale / terrain_material.detail_scale_m)
        + terrain_material.detail_uv_offset;

    // G3A-R: anti-repetition rotated second sample. Rotating the base UV by
    // the fixed 2D angle, scaling it, and shifting it makes the second sample
    // tile at a different frequency and angle than the base grid, so no
    // single physical base-tile border repeats recognizably across the field.
    let ar_scale = terrain_material.ar_scale_offset.x;
    let ar_offset = terrain_material.ar_scale_offset.yz;
    let ar_cos = terrain_material.ar_angle_cos_sin.x;
    let ar_sin = terrain_material.ar_angle_cos_sin.y;
    let ar_uv = vec2<f32>(
            ar_cos * base_uv.x - ar_sin * base_uv.y,
            ar_sin * base_uv.x + ar_cos * base_uv.y,
        ) * ar_scale + ar_offset;
    // PV2: decorrelate the macro carrier as well as the physical base tile. The
    // second macro sample has an unrelated angle/period, so 30-80 m tonal
    // patches cannot reveal a single repeated square in oblique views.
    let macro_ar_uv = vec2<f32>(
            TERRAIN_MACRO_AR_COS * macro_uv.x - TERRAIN_MACRO_AR_SIN * macro_uv.y,
            TERRAIN_MACRO_AR_SIN * macro_uv.x + TERRAIN_MACRO_AR_COS * macro_uv.y,
        ) * TERRAIN_MACRO_AR_SCALE + TERRAIN_MACRO_AR_OFFSET;

    // G3A-R: detail distance fade — 1.0 close, 0.0 far, smoothstep
    // (zero-derivative ends, no popping).
    let camera_position = camera.camera_position.xyz;
    let distance = length(input.world_position - camera_position);
    let fade_near = terrain_material.detail_fade_near_far.x;
    let fade_far = terrain_material.detail_fade_near_far.y;
    let detail_fade = 1.0 - smoothstep(fade_near, fade_far, distance);

    // FFV1: region weights (pure function of world XZ). A dynamic branch that
    // skips the companion samples where the mask is zero was tried and
    // REVERTED: divergent texturing cost far more than the samples it saved
    // (scene pass 775 -> 11102 us at 1080p), so the blend stays uniform.
    let region = field_region_weights(input.world_position.xz);

    // --- Albedo stack -------------------------------------------------------
    let albedo_base = textureSample(terrain_albedo_texture, terrain_sampler, base_uv);
    let albedo_ar = textureSample(terrain_albedo_texture, terrain_sampler, ar_uv);
    let albedo_macro_s = textureSample(terrain_albedo_texture, terrain_sampler, macro_uv);
    let albedo_macro_ar_s = textureSample(terrain_albedo_texture, terrain_sampler, macro_ar_uv);
    let albedo_detail_s = textureSample(terrain_albedo_texture, terrain_sampler, detail_uv);

    // Base + rotated second sample 50/50: tile borders of the two samples run
    // at different angles/frequencies, breaking the base grid's repetition.
    var albedo = mix(albedo_base, albedo_ar, TERRAIN_BASE_AR_BLEND);
    // Soft macro tone: multiplicative luminance modulation around the sample
    // mean, so large patches breathe without a hue shift or contrast boost.
    let macro_luminance = mix(
        dot(albedo_macro_s.rgb, vec3<f32>(0.299, 0.587, 0.114)),
        dot(albedo_macro_ar_s.rgb, vec3<f32>(0.299, 0.587, 0.114)),
        TERRAIN_MACRO_AR_BLEND,
    );
    let macro_gain = 1.0 + (macro_luminance - 0.5) * TERRAIN_MACRO_ALBEDO_GAIN * 2.0;
    albedo = vec4<f32>(albedo.rgb * macro_gain, albedo.a);
    // Fine near-field grain, mean-preserving, distance-faded.
    albedo = mix(albedo, albedo_detail_s, TERRAIN_DETAIL_ALBEDO_BLEND * detail_fade);

    // FFV1: region composition. The worn and dry companion materials sample at
    // their own physical tile spans (1.0 m and 3.15 m) and interlock with the
    // maintained 2.0 m base through a luminance-weighted blend of the region
    // weights, so a boundary follows the photographed material instead of
    // drawing a geometric seam. Where the mask gives the companions zero
    // weight (the mown field interior, most of the screen) the companion
    // samples are skipped entirely: that guard is what keeps the 3-material
    // field inside its performance budget.
    let worn_uv = input.world_position.xz / FFV1_WORN_TILE_SCALE_M
        + vec2<f32>(0.41, 0.13);
    let worn_ar_uv = vec2<f32>(
            ar_cos * worn_uv.x - ar_sin * worn_uv.y,
            ar_sin * worn_uv.x + ar_cos * worn_uv.y,
        ) * ar_scale + ar_offset;
    let dry_uv = input.world_position.xz / FFV1_DRY_TILE_SCALE_M
        + vec2<f32>(0.27, 0.61);
    let dry_ar_uv = vec2<f32>(
            ar_cos * dry_uv.x - ar_sin * dry_uv.y,
            ar_sin * dry_uv.x + ar_cos * dry_uv.y,
        ) * ar_scale + ar_offset;
    let albedo_worn = mix(
        textureSample(terrain_worn_albedo_texture, terrain_sampler, worn_uv),
        textureSample(terrain_worn_albedo_texture, terrain_sampler, worn_ar_uv),
        TERRAIN_BASE_AR_BLEND,
    );
    let albedo_dry = mix(
        textureSample(terrain_dry_albedo_texture, terrain_sampler, dry_uv),
        textureSample(terrain_dry_albedo_texture, terrain_sampler, dry_ar_uv),
        TERRAIN_BASE_AR_BLEND,
    );
    let luminance_weights = vec3<f32>(0.299, 0.587, 0.114);
    let w_maint = (region.x + region.z)
        * (dot(albedo.rgb, luminance_weights) + FIELD_BLEND_BIAS);
    let w_worn = region.y * (dot(albedo_worn.rgb, luminance_weights) + FIELD_BLEND_BIAS);
    let w_dry = region.w * (dot(albedo_dry.rgb, luminance_weights) + FIELD_BLEND_BIAS);
    let w_sum = max(w_maint + w_worn + w_dry, 1e-4);
    var composed = (albedo.rgb * w_maint
        + albedo_worn.rgb * w_worn
        + albedo_dry.rgb * w_dry) / w_sum;
    // Keep the scanned turf as the common base, so region masks vary the
    // surface without producing isolated pale yellow-green islands.
    composed = mix(albedo.rgb, composed, 0.30);
    // Regional tone. The maintained scan (sparse_grass) photographs as dry
    // warm turf (linear R/G ~ 1.4); the Wikimedia RC-field reference and the
    // Poly Haven meadow backplates both show a maintained mown field at
    // linear R/G ~ 0.5-1.1, i.e. clearly green. The maintained regions
    // therefore carry a documented mown-grass white-balance, worn traffic
    // stays grassy-but-paler, and the vegetation edge dries out toward soil.
    let tint_maint = vec3<f32>(0.66, 1.10, 0.74);
    let tint_worn = vec3<f32>(0.72, 0.96, 0.72);
    let tint_dry = vec3<f32>(0.55, 0.52, 0.40);
    let region_tint = (tint_maint * (region.x + region.z)
        + tint_worn * region.y
        + tint_dry * region.w)
        / max(region.x + region.y + region.z + region.w, 1e-4);
    // MACRO scale (tens of metres): healthy vs dry patches across the field,
    // so the ground never reads as one repeated texture.
    let health = field_fbm(input.world_position.xz * 0.022);
    let health_tint = vec3<f32>(
        1.00 - 0.05 * health,
        0.96 + 0.06 * health,
        1.00 - 0.03 * health,
    );
    // Mown laps modulate the maintained regions only.
    let mow = field_mow_stripe(input.world_position.xz);
    let mow_gain = 1.0 + (mow - 0.5) * (region.x * 0.08 + region.z * 0.04);
    composed = composed * region_tint * health_tint * mow_gain;
    albedo = vec4<f32>(composed, albedo.a);

    let base_rgba = input.color * vec4<f32>(albedo.rgb, 1.0);

    let metallic = clamp(terrain_material.metallic, 0.0, 1.0);

    // G3A-R: multi-scale roughness stack around the material base factor.
    // The base/macro/detail weights sum to 1 at full detail, so the mean is
    // preserved; the detail term is distance-faded and the weights are
    // renormalized so the surface never turns wet/specular at distance.
    let r_base_a = textureSample(terrain_roughness_texture, terrain_sampler, base_uv).r;
    let r_base_b = textureSample(terrain_roughness_texture, terrain_sampler, ar_uv).r;
    let r_base = mix(r_base_a, r_base_b, TERRAIN_BASE_AR_BLEND);
    let r_macro_a = textureSample(terrain_roughness_texture, terrain_sampler, macro_uv).r;
    let r_macro_b = textureSample(terrain_roughness_texture, terrain_sampler, macro_ar_uv).r;
    let r_macro_s = mix(r_macro_a, r_macro_b, TERRAIN_MACRO_AR_BLEND);
    let r_detail_s = textureSample(terrain_roughness_texture, terrain_sampler, detail_uv).r;
    let detail_weight = TERRAIN_ROUGHNESS_DETAIL_WEIGHT * detail_fade;
    let r_stack = (r_base * TERRAIN_ROUGHNESS_BASE_WEIGHT
        + r_macro_s * TERRAIN_ROUGHNESS_MACRO_WEIGHT
        + r_detail_s * detail_weight)
        / (TERRAIN_ROUGHNESS_BASE_WEIGHT + TERRAIN_ROUGHNESS_MACRO_WEIGHT + detail_weight);
    // FFV1: the companion layers carry their own roughness response, blended
    // with the same normalized region weights as the albedo. The worn layer
    // has no roughness map of its own: compacted traffic reads as a slightly
    // tighter (lower) roughness on the maintained stack. The dry sample is
    // skipped with the albedo guard so the mown interior pays only the
    // maintained stack.
    let r_worn = r_stack * FIELD_WORN_ROUGHNESS_SCALE;
    let r_dry = mix(
        textureSample(terrain_dry_roughness_texture, terrain_sampler, dry_uv).r,
        textureSample(terrain_dry_roughness_texture, terrain_sampler, dry_ar_uv).r,
        TERRAIN_BASE_AR_BLEND,
    );
    let r_maintained = terrain_material.roughness * r_stack;
    let roughness = clamp(
        (r_maintained * w_maint + r_worn * w_worn + r_dry * w_dry) / w_sum,
        MIN_ROUGHNESS,
        1.0,
    );

    // G3A-R: tangent-space normal stack; Z is rebuilt so the vector stays
    // unit length (linear map data, decoded to [-1, 1]). The detail normal
    // contributes only near the camera (detail_fade), stabilizing distant
    // oblique views without popping.
    let normal_base_a_raw = normalize_tangent_normal(
        textureSample(terrain_normal_texture, terrain_sampler, base_uv).rgb
            * 2.0
            - vec3<f32>(1.0),
    );
    let normal_base_b_rotated_raw = textureSample(terrain_normal_texture, terrain_sampler, ar_uv)
        .rgb
        * 2.0
        - vec3<f32>(1.0);
    let normal_base_b_raw = reorient_secondary_tangent_normal(
        normal_base_b_rotated_raw,
        ar_cos,
        ar_sin,
    );
    let normal_base_raw = normalize_tangent_normal(mix(
        normal_base_a_raw,
        normal_base_b_raw,
        TERRAIN_BASE_AR_BLEND,
    ));
    let normal_detail_raw = textureSample(
        terrain_normal_texture,
        terrain_sampler,
        detail_uv,
    )
    .rgb
        * 2.0
        - vec3<f32>(1.0);
    let strength = clamp(terrain_material.normal_strength, 0.0, 1.0);
    let n_ts_base_xy = normal_base_raw.xy * strength
        + normal_detail_raw.xy * (strength * TERRAIN_DETAIL_NORMAL_BLEND * detail_fade);
    let n_ts_base = normalize_tangent_normal(vec3<f32>(
        n_ts_base_xy,
        sqrt(max(1.0 - dot(n_ts_base_xy, n_ts_base_xy), 0.0)),
    ));
    // FFV1: companion-layer tangent normals, blended by the same normalized
    // region weights. The companion relief is scaled slightly down so the
    // maintained base keeps authority over the mown surfaces. Skipped with
    // the albedo guard so the mown interior pays only the maintained stack.
    let n_worn_raw = normalize_tangent_normal(mix(
        textureSample(terrain_worn_normal_texture, terrain_sampler, worn_uv).rgb * 2.0
            - vec3<f32>(1.0),
        reorient_secondary_tangent_normal(
            textureSample(terrain_worn_normal_texture, terrain_sampler, worn_ar_uv).rgb * 2.0
                - vec3<f32>(1.0),
            ar_cos,
            ar_sin,
        ),
        TERRAIN_BASE_AR_BLEND,
    ));
    let n_dry_raw = normalize_tangent_normal(mix(
        textureSample(terrain_dry_normal_texture, terrain_sampler, dry_uv).rgb * 2.0
            - vec3<f32>(1.0),
        reorient_secondary_tangent_normal(
            textureSample(terrain_dry_normal_texture, terrain_sampler, dry_ar_uv).rgb * 2.0
                - vec3<f32>(1.0),
            ar_cos,
            ar_sin,
        ),
        TERRAIN_BASE_AR_BLEND,
    ));
    let n_region_xy = n_ts_base.xy * (w_maint / w_sum)
        + n_worn_raw.xy * (w_worn / w_sum) * FIELD_COMPANION_NORMAL_SCALE
        + n_dry_raw.xy * (w_dry / w_sum) * FIELD_COMPANION_NORMAL_SCALE;
    let n_ts = normalize_tangent_normal(vec3<f32>(
        n_region_xy,
        sqrt(max(1.0 - dot(n_region_xy, n_region_xy), 0.0)),
    ));

    // G3A: fragment TBN from screen-space derivatives of the interpolated
    // world position and UV. `uv` is world-anchored, so the frame is
    // chunk-independent and seamless across chunk boundaries. Degenerate
    // fragments fall back to the world-aligned basis instead of NaN.
    let dp1 = dpdx(input.world_position);
    let dp2 = dpdy(input.world_position);
    let duv1 = dpdx(input.uv);
    let duv2 = dpdy(input.uv);
    let det = duv1.x * duv2.y - duv1.y * duv2.x;
    let has_basis = abs(det) > 1e-8;
    let inv_det = select(0.0, 1.0 / det, has_basis);
    let tangent = select(
        vec3<f32>(1.0, 0.0, 0.0),
        normalize((duv2.y * dp1 - duv1.y * dp2) * inv_det),
        has_basis,
    );
    let bitangent = select(
        vec3<f32>(0.0, 0.0, 1.0),
        normalize((-duv2.x * dp1 + duv1.x * dp2) * inv_det),
        has_basis,
    );
    let geom_normal = safe_normalize(input.world_normal);
    let n = safe_normalize(tangent * n_ts.x + bitangent * n_ts.y + geom_normal * n_ts.z);

    return TerrainSurface(
        base_rgba,
        albedo,
        n,
        n_ts,
        metallic,
        roughness,
        albedo_macro_s,
        albedo_detail_s,
        region,
    );
}

// Shared terrain tail for presentation-only debug channels. Each entry point
// supplies its final atmosphere-composited color, while raw debug channels
// remain diagnostic and deliberately bypass both fog implementations.
fn terrain_fragment_output(
    surface: TerrainSurface,
    final_rgb: vec3<f32>,
) -> vec4<f32> {
    // G3A-R: presentation-only debug channels. The uniform selector is
    // defaulted to 0 (FINAL) by the renderer; the lit path above is computed
    // identically regardless of the selector, so the production output is
    // untouched when debugging is disabled.
    var output_rgb = final_rgb;
    let mode = terrain_material.debug_mode;
    if (mode == 1u) {
        output_rgb = surface.albedo.rgb;
    } else if (mode == 2u) {
        output_rgb = surface.n_ts * 0.5 + vec3<f32>(0.5);
    } else if (mode == 3u) {
        output_rgb = vec3<f32>(surface.roughness);
    } else if (mode == 4u) {
        output_rgb = surface.albedo_macro_s.rgb;
    } else if (mode == 5u) {
        output_rgb = surface.albedo_detail_s.rgb;
    } else if (mode == 6u) {
        // FFV1 region mask: runway = R, worn = G, dry edge = B, maintained
        // field as a neutral floor so empty regions read dark grey.
        let floor_tone = 0.35 * surface.region_weights.z;
        output_rgb = vec3<f32>(
            floor_tone + surface.region_weights.x,
            floor_tone + surface.region_weights.y,
            floor_tone + surface.region_weights.w,
        );
    }
    return vec4<f32>(output_rgb, surface.base_rgba.a);
}

// G3A-R: terrain stack tuning constants (WGSL side of the central values in
// `terrain.rs`). Low-contrast by design: the field must read as a maintained
// flying field, not a wild biome. G3-VR1: slightly stronger macro gain and
// detail blend break perceived base-tile repetition without changing UVs.
const TERRAIN_MACRO_ALBEDO_GAIN: f32 = 0.32;
const TERRAIN_BASE_AR_BLEND: f32 = 0.5;
const TERRAIN_DETAIL_ALBEDO_BLEND: f32 = 0.35;
const TERRAIN_DETAIL_NORMAL_BLEND: f32 = 0.55;
const TERRAIN_ROUGHNESS_BASE_WEIGHT: f32 = 0.70;
const TERRAIN_ROUGHNESS_MACRO_WEIGHT: f32 = 0.10;
const TERRAIN_ROUGHNESS_DETAIL_WEIGHT: f32 = 0.20;
const TERRAIN_MACRO_AR_SCALE: f32 = 0.71;
const TERRAIN_MACRO_AR_COS: f32 = 0.6156615;
const TERRAIN_MACRO_AR_SIN: f32 = -0.7880108;
const TERRAIN_MACRO_AR_OFFSET: vec2<f32> = vec2<f32>(0.419, 0.173);
const TERRAIN_MACRO_AR_BLEND: f32 = 0.43;

// FFV1: physical tile spans of the companion ground materials, in metres.
// They mirror `terrain.rs` and are the measured Poly Haven scan spans
// (grass_path_3 = 1.0 m, forest_ground_04 = 3.15 m), so each photograph is
// reproduced at true size exactly like the 2.0 m maintained base.
const FFV1_WORN_TILE_SCALE_M: f32 = 1.0;
const FFV1_DRY_TILE_SCALE_M: f32 = 3.15;
// FFV1: luminance bias of the height-blend that interlocks the region
// weights; small enough to keep region authority, large enough that the
// photographed material grain breaks the boundary line.
const FIELD_BLEND_BIAS: f32 = 0.12;
// FFV1: companion-layer tangent-normal contribution scale.
const FIELD_COMPANION_NORMAL_SCALE: f32 = 0.9;
// FFV1: worn/compacted traffic roughness scale on the maintained stack (no
// dedicated worn roughness map; see the binding comment above).
const FIELD_WORN_ROUGHNESS_SCALE: f32 = 0.92;

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
    return vec4<f32>(sky_color_for_direction(view_dir), 1.0);
}

// RV2-5 physical sky: samples the production Sky-View LUT (single scattering +
// the multi-scattering approximation of the generated atmosphere) and adds the
// analytic sun disk from the same SunState radiance and angular radius. Both
// stay scene-referred HDR — no clamp, no tone mapping here.
@fragment
fn fs_sky_v2(input: SkyVertexOutput) -> @location(0) vec4<f32> {
    let view_dir = view_direction_from_clip(input.clip_xy);
    let azimuth = atan2(view_dir.z, view_dir.x);
    let elevation = asin(clamp(view_dir.y, -1.0, 1.0));
    let sky_uv = vec2<f32>(
        fract(azimuth / (2.0 * PI) + 0.5),
        clamp(elevation / PI + 0.5, 0.0, 1.0),
    );
    var color = textureSample(sky_view_lut, atmosphere_sampler, sky_uv).rgb;
    // A mild clear-air blue towards the zenith restores the visible tonal
    // gradient while preserving the physical LUT's hazy horizon and sun.
    let zenith_weight = smoothstep(0.0, 0.6, max(view_dir.y, 0.0));
    color = color * mix(vec3<f32>(1.0), vec3<f32>(0.90, 0.97, 1.07), zenith_weight);

    // Solar disk: the same direction, angular radius and radiance as the PBR
    // direct term, attenuated by the physical transmittance at the observer.
    let sun_dir = safe_normalize(environment.light_direction.xyz);
    let cos_radius = cos(environment.atmosphere_state.x);
    if (dot(view_dir, sun_dir) > cos_radius) {
        let sun_transmittance = textureSample(
            transmittance_lut,
            atmosphere_sampler,
            vec2<f32>(clamp(sun_dir.y * 0.5 + 0.5, 0.0, 1.0), environment.atmosphere_state.z),
        ).rgb;
        color = color + environment.sun_color.xyz * sun_transmittance;
    }
    return vec4<f32>(max(color, vec3<f32>(0.0)), 1.0);
}

// ---------------------------------------------------------------------------
// G3B: fullscreen postprocess pass (HDR scene -> exposure -> tone map -> sRGB)
// ---------------------------------------------------------------------------

// Color-space contract (single chain, documented):
//   1. sRGB textures decode to linear on sampling (hardware conversion).
//   2. All lighting runs in linear HDR and is stored scene-referred in the
//      Rgba16Float target — no clamp before the tone mapper anywhere.
//   3. The postprocess pass applies the manual exposure, then the Khronos
//      PBR Neutral tone mapper (highlight compression, controlled saturation,
//      monotone finite response), and writes linear display values.
//   4. The sRGB surface format performs the final linear->sRGB encode.
// There is deliberately NO pow() gamma handling in shader code: the surface
// already encodes, and a second manual gamma would double-apply the transfer.
fn khronos_pbr_neutral(color: vec3<f32>) -> vec3<f32> {
    // Khronos PBR Neutral tone mapper (exact reference math, see the Khronos
    // glTF-Sample-Renderer tonemapping.glsl).
    let start_compression = 0.8 - 0.04;
    let desaturation = 0.15;

    let x = min(color.r, min(color.g, color.b));
    let offset = select(0.04, x - 6.25 * x * x, x < 0.08);
    var c = color - vec3<f32>(offset);

    let peak = max(c.r, max(c.g, c.b));
    if (peak < start_compression) {
        return c;
    }
    let d = 1.0 - start_compression;
    let new_peak = 1.0 - d * d / (peak + d - start_compression);
    c = c * vec3<f32>(new_peak / peak);
    let g = 1.0 - 1.0 / (desaturation * (peak - new_peak) + 1.0);
    return mix(c, vec3<f32>(new_peak), g);
}

@fragment
fn fs_postprocess(input: SkyVertexOutput) -> @location(0) vec4<f32> {
    // 1:1 texel mapping from the fullscreen triangle; +0.5 lands on texel
    // centers under the nearest sampler (deterministic, no cross-texel blur).
    // WebGPU framebuffer Y grows downward while clip-space Y grows upward.
    // Flip V so the offscreen HDR target keeps its orientation on the surface.
    let uv = vec2<f32>(
        input.clip_xy.x * 0.5 + 0.5,
        0.5 - input.clip_xy.y * 0.5,
    );
    let hdr_rgb = textureSample(hdr_scene_texture, hdr_scene_sampler, uv).rgb;
    let exposed_rgb = hdr_rgb * exp2(postprocess.exposure_ev);
    let display_rgb = khronos_pbr_neutral(exposed_rgb);
    return vec4<f32>(display_rgb, 1.0);
}

// ---------------------------------------------------------------------------
// PF1: Photo Field presentation (panorama composite, depth proxies, photo shadow)
// ---------------------------------------------------------------------------

// Presentation-only Photo Field state. shadow_strength is consumed by the
// photographic ground-shadow receiver; the yaw/pitch calibration by the final
// composite. Exposure is deliberately NOT duplicated here: the composite reads
// the same PostProcessUniform as the 3D path, so the photograph and the
// tone-mapped aircraft share exactly one exposure relationship.
struct PhotoFieldUniform {
    panorama_yaw_rad: f32,
    panorama_pitch_rad: f32,
    shadow_strength: f32,
    padding_0: f32,
};

// Group 6 is the first free bind group (0 camera, 1 object, 2 environment +
// shadows, 3 material, 4 terrain/vegetation, 5 postprocess). The panorama is
// an sRGB texture, so sampling yields linear display values and the sRGB
// surface re-encodes them: the photograph round-trips untouched by any tone
// mapper. The two depth textures are read 1:1 with textureLoad (no sampler, no
// cross-texel blur); the mask likewise.
@group(6) @binding(0)
var photo_panorama_texture: texture_2d<f32>;
@group(6) @binding(1)
var photo_panorama_sampler: sampler;
@group(6) @binding(2)
var photo_scene_depth: texture_depth_2d;
@group(6) @binding(3)
var photo_proxy_depth: texture_depth_2d;
@group(6) @binding(4)
var photo_shadow_mask: texture_2d<f32>;
@group(6) @binding(5)
var<uniform> photo_field: PhotoFieldUniform;

// Equirectangular UV of a world direction under the PF1 convention: the zenith
// lives on the panorama's FIRST row (v = 0), u wraps seamlessly with azimuth,
// and the calibration is a rigid rotation (yaw about +Y, then pitch about the
// rotated +X) of the world direction into panorama space. Mirrors
// crates/renderer/src/photo_field.rs::equirect_uv_from_direction exactly.
fn photo_field_equirect_uv(direction: vec3<f32>) -> vec2<f32> {
    let dir = normalize(direction);
    // The yaw is SUBTRACTED, mirroring photo_field.rs: a calibration of +yaw
    // puts panorama longitude 0 on world azimuth +yaw.
    let sy = sin(photo_field.panorama_yaw_rad);
    let cy = cos(photo_field.panorama_yaw_rad);
    let x1 = dir.x * cy + dir.z * sy;
    let z1 = -dir.x * sy + dir.z * cy;
    let sp = sin(photo_field.panorama_pitch_rad);
    let cp = cos(photo_field.panorama_pitch_rad);
    let y2 = dir.y * cp + z1 * sp;
    let z2 = -dir.y * sp + z1 * cp;
    let azimuth = atan2(z2, x1);
    var u = fract(azimuth / (2.0 * PI));
    if (u < 0.0) {
        u = u + 1.0;
    }
    let elevation = asin(clamp(y2, -1.0, 1.0));
    let v = clamp(0.5 - elevation / PI, 0.0, 1.0);
    return vec2<f32>(u, v);
}

struct PhotoProxyVertexInput {
    @location(0) position: vec3<f32>,
};

// Invisible depth-proxy rasterization. The pipeline attaches ONLY a depth
// target and has no fragment stage at all, so the proxies can never contribute
// colour to any target; they exist purely as camera-space depth.
@vertex
fn vs_photo_proxy(input: PhotoProxyVertexInput) -> @builtin(position) vec4<f32> {
    return camera.view_projection * object.model * vec4<f32>(input.position, 1.0);
}

struct PhotoMaskVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_position: vec3<f32>,
};

@vertex
fn vs_photo_shadow_mask(input: PhotoProxyVertexInput) -> PhotoMaskVertexOutput {
    let world = object.model * vec4<f32>(input.position, 1.0);
    var out: PhotoMaskVertexOutput;
    out.clip_position = camera.view_projection * world;
    out.world_position = world.xyz;
    return out;
}

// Photographic ground-shadow receiver: writes ONLY an attenuation factor into
// the presentation-only mask target. The photograph already carries its own
// static environmental shadows; this adds the dynamic aircraft shadow on top of
// the photographed grass using the existing cascaded aircraft shadow data,
// without exposing any visible ground geometry and without replacing the grass
// with a rendered PBR plane. The pass depth-tests against the proxy depth with
// writes disabled, so a nearer proxy (a photographed tree) suppresses the
// ground shadow behind it.
@fragment
fn fs_photo_shadow_mask(input: PhotoMaskVertexOutput) -> @location(0) vec4<f32> {
    let visibility = directional_shadow_visibility(input.world_position);
    let attenuation = 1.0 - photo_field.shadow_strength * (1.0 - visibility);
    return vec4<f32>(attenuation, attenuation, attenuation, 1.0);
}

// PF1 final composite: photographic panorama + tone-mapped 3D aircraft +
// photographic shadow mask -> linear display values (the sRGB surface encodes).
//
// Coverage comes from a depth comparison, never from alpha: the scene depth
// holds only real 3D geometry (the aircraft), the proxy depth holds only the
// invisible photographic stand-ins. The aircraft is visible exactly where it is
// nearer than whatever the photograph depicts at that pixel, which is what
// makes a photographed obstacle occlude it and lets it fly in front of one.
@fragment
fn fs_postprocess_photo(input: SkyVertexOutput) -> @location(0) vec4<f32> {
    let uv = vec2<f32>(
        input.clip_xy.x * 0.5 + 0.5,
        0.5 - input.clip_xy.y * 0.5,
    );
    let dims = vec2<f32>(textureDimensions(photo_scene_depth));
    let texel = vec2<i32>(floor(uv * dims));
    let scene_depth = textureLoad(photo_scene_depth, texel, 0);
    let proxy_depth = textureLoad(photo_proxy_depth, texel, 0);
    let shadow_mask = textureLoad(photo_shadow_mask, texel, 0).r;

    let view_dir = view_direction_from_clip(input.clip_xy);
    let photo = textureSample(
        photo_panorama_texture,
        photo_panorama_sampler,
        photo_field_equirect_uv(view_dir),
    ).rgb * shadow_mask;

    let hdr_rgb = textureSample(hdr_scene_texture, hdr_scene_sampler, uv).rgb;
    let aircraft = khronos_pbr_neutral(hdr_rgb * exp2(postprocess.exposure_ev));

    let coverage = select(0.0, 1.0, scene_depth < 1.0 && scene_depth < proxy_depth);
    return vec4<f32>(mix(photo, aircraft, coverage), 1.0);
}

// ---------------------------------------------------------------------------
// G3D: production vegetation (instanced trees)
// ---------------------------------------------------------------------------

// Presentation-only vegetation state (group 4 of the vegetation scene
// pipeline). debug_mode: 0 = FINAL (production path), 1 = LOD colors.
// The uniform is written once on mode change, never per frame in production.
struct VegetationUniform {
    debug_mode: u32,
    pad0: u32,
    pad1: u32,
    pad2: u32,
};

@group(4) @binding(0)
var<uniform> vegetation_state: VegetationUniform;

// Per-instance attributes ride on vertex buffer slot 1, step mode Instance:
//   offset 0  position_yaw : world position xyz + yaw (radians, around +Y)
//   offset 16 scale_tint   : uniform scale (x) + rgb color tint
//   offset 32 lod_class    : LOD class (x) + asset index (y), reserved zw
// The shader reconstructs T = translate(position) * rotY(yaw) * scale at the
// vertex — no 4x4 instance matrix is stored or built on the GPU.
struct VegetationVertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) instance_position_yaw: vec4<f32>,
    @location(5) instance_scale_tint: vec4<f32>,
    @location(6) instance_lod_class: vec4<f32>,
};

struct VegetationVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_normal: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) world_position: vec3<f32>,
    @location(4) lod_class: f32,
};

struct VegetationTransformed {
    world_position: vec3<f32>,
    world_normal: vec3<f32>,
};

// T = translate(position) * rotY(yaw) * scale on the local position; the
// normal rotates with the same Y-rotation block (uniform scale + rotation,
// so the inverse-transpose equals R). Both results are unit-safe.
fn transform_vegetation_vertex(
    local_position: vec3<f32>,
    local_normal: vec3<f32>,
    position_yaw: vec4<f32>,
    scale: f32,
) -> VegetationTransformed {
    let yaw = position_yaw.w;
    let cos_y = cos(yaw);
    let sin_y = sin(yaw);
    let p = local_position * scale;
    let world_position = vec3<f32>(
        position_yaw.x + cos_y * p.x + sin_y * p.z,
        position_yaw.y + p.y,
        position_yaw.z - sin_y * p.x + cos_y * p.z,
    );
    let world_normal = normalize(vec3<f32>(
        cos_y * local_normal.x + sin_y * local_normal.z,
        local_normal.y,
        -sin_y * local_normal.x + cos_y * local_normal.z,
    ));
    return VegetationTransformed(world_position, world_normal);
}

// Placement tint multiplies the baked per-vertex base color (±~8%).
fn tinted_vertex_color(base: vec4<f32>, tint: vec3<f32>) -> vec4<f32> {
    return vec4<f32>(base.rgb * tint, base.a);
}

@vertex
fn vs_vegetation(input: VegetationVertexInput) -> VegetationVertexOutput {
    var output: VegetationVertexOutput;
    let transformed = transform_vegetation_vertex(
        input.position,
        input.normal,
        input.instance_position_yaw,
        input.instance_scale_tint.x,
    );
    output.clip_position = camera.view_projection * vec4<f32>(transformed.world_position, 1.0);
    output.world_normal = transformed.world_normal;
    output.color = tinted_vertex_color(input.color, input.instance_scale_tint.yzw);
    output.uv = input.uv;
    output.world_position = transformed.world_position;
    output.lod_class = input.instance_lod_class.x;
    return output;
}

// G3E depth-only instanced caster for the vegetation shadow passes. Shares the
// exact instance transform. PV1-R2: adds a fragment stage for alpha-masked
// foliage shadows so leaf cards cast shaped shadows instead of solid quads.
struct VegetationShadowOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) color: vec4<f32>,
};

@vertex
fn vs_vegetation_shadow(input: VegetationVertexInput) -> VegetationShadowOutput {
    var output: VegetationShadowOutput;
    let transformed = transform_vegetation_vertex(
        input.position,
        input.normal,
        input.instance_position_yaw,
        input.instance_scale_tint.x,
    );
    output.clip_position = shadow_cascade.light_view_projection
        * vec4<f32>(transformed.world_position, 1.0);
    output.uv = input.uv;
    output.color = tinted_vertex_color(input.color, input.instance_scale_tint.yzw);
    return output;
}

// PV1-R2: shadow fragment with alpha cutoff for foliage leaf cards.
// No colour output (depth-only target); the discard carves the leaf
// silhouette into the shadow map.
@fragment
fn fs_vegetation_shadow(input: VegetationShadowOutput) {
    let texture_rgba = textureSample(base_color_texture, base_color_sampler, input.uv);
    let base_alpha = input.color.a * texture_rgba.a;
    if (base_alpha < 0.45) {
        discard;
    }
}

// FFV1: foliage leaf-card sampling bias. The leaf clusters are small, so the
// hardware-selected mip at tree-line distance averages alpha below the cutoff
// and the crown evaporates; a negative bias keeps coverage alive (mild
// shimmer is preferable to a bare skeleton).
const FOLIAGE_MIP_BIAS: f32 = -1.5;

// FFV1: foliage alpha cutoff with distance compensation. Mipmapped leaf
// atlases average coverage down at the edges, so a constant cutoff would
// erode distant crowns into specks; easing the threshold down with distance
// keeps the silhouette mass stable (no shimmer, no popping) while near-field
// leaves keep the crisp 0.45 cut.
fn vegetation_alpha_cutoff(world_position: vec3<f32>) -> f32 {
    let distance = length(world_position - camera.camera_position.xyz);
    return mix(0.45, 0.35, smoothstep(40.0, 120.0, distance));
}

// FFV1: leaf cards are two-sided; shade the face the viewer actually sees so
// back faces of a card are not lit with a normal pointing away.
fn vegetation_shading_normal(world_normal: vec3<f32>, front_facing: bool) -> vec3<f32> {
    let n = safe_normalize(world_normal);
    return select(-n, n, front_facing);
}

// Lit vegetation fragment: the exact fs_lit chain (texture * vertex color,
// metallic/roughness PBR, G2B shadow visibility, distance fog) so trees join
// the same linear HDR Rgba16Float scene — no independent tone mapping, no
// LDR clamp, no gamma. The presentation-only debug selector (0 FINAL, 1 LOD
// colors) is resolved first; production output is untouched by it.
@fragment
fn fs_vegetation(
    input: VegetationVertexOutput,
    @builtin(front_facing) front_facing: bool,
) -> @location(0) vec4<f32> {
    let mode = vegetation_state.debug_mode;
    if (mode == 1u) {
        // Deterministic LOD debug colors: LOD0 green, LOD1 yellow, LOD2 orange.
        let lod = u32(input.lod_class + 0.5);
        var debug_color: vec3<f32>;
        if (lod == 0u) {
            debug_color = vec3<f32>(0.05, 0.65, 0.15);
        } else if (lod == 1u) {
            debug_color = vec3<f32>(0.85, 0.70, 0.10);
        } else {
            debug_color = vec3<f32>(0.90, 0.42, 0.08);
        }
        return vec4<f32>(debug_color, 1.0);
    }

    let texture_rgba = textureSampleBias(
        base_color_texture,
        base_color_sampler,
        input.uv,
        FOLIAGE_MIP_BIAS,
    );
    let base_rgba = input.color * texture_rgba;

    // PV1-R: alpha-mask cutoff for foliage leaf cards. Fragments below the
    // threshold are discarded so the card silhouette reads as individual
    // leaves rather than a textured quad. The bark primitive uses an opaque
    // texture (alpha = 1) so this never clips bark geometry.
    if (base_rgba.a < vegetation_alpha_cutoff(input.world_position)) {
        discard;
    }

    let metallic = clamp(material.metallic, 0.0, 1.0);
    let roughness = clamp(material.roughness, MIN_ROUGHNESS, 1.0);

    let n = vegetation_shading_normal(input.world_normal, front_facing);
    let lit_rgb = lit_pbr_response(base_rgba, n, input.world_position, metallic, roughness);
    let final_rgb = apply_distance_fog(lit_rgb, input.world_position);

    return vec4<f32>(final_rgb, 1.0);
}

// RV2-5 physical vegetation counterpart: identical material, alpha-mask and
// debug LOD path, with the physical split-sum IBL lighting.
@fragment
fn fs_vegetation_v2(
    input: VegetationVertexOutput,
    @builtin(front_facing) front_facing: bool,
) -> @location(0) vec4<f32> {
    let mode = vegetation_state.debug_mode;
    if (mode == 1u) {
        let lod = u32(input.lod_class + 0.5);
        var debug_color: vec3<f32>;
        if (lod == 0u) {
            debug_color = vec3<f32>(0.05, 0.65, 0.15);
        } else if (lod == 1u) {
            debug_color = vec3<f32>(0.85, 0.70, 0.10);
        } else {
            debug_color = vec3<f32>(0.90, 0.42, 0.08);
        }
        return vec4<f32>(debug_color, 1.0);
    }

    let texture_rgba = textureSampleBias(
        base_color_texture,
        base_color_sampler,
        input.uv,
        FOLIAGE_MIP_BIAS,
    );
    let base_rgba = input.color * texture_rgba;

    if (base_rgba.a < vegetation_alpha_cutoff(input.world_position)) {
        discard;
    }

    let metallic = clamp(material.metallic, 0.0, 1.0);
    let roughness = clamp(material.roughness, MIN_ROUGHNESS, 1.0);

    let n = vegetation_shading_normal(input.world_normal, front_facing);
    let lit_rgb = lit_pbr_response_physical(base_rgba, n, input.world_position, metallic, roughness);
    let final_rgb = apply_physical_aerial_perspective(lit_rgb, input.world_position);

    return vec4<f32>(final_rgb, 1.0);
}
