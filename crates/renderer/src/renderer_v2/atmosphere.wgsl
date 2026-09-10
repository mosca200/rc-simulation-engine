// RV2-5: GPU physical atmosphere generation.
//
// Everything here runs ONCE, on the GPU, inside the dedicated V2 initialization
// command encoder. No production frame ever executes these entry points and no
// resource is allocated from the frame loop.
//
// Model (Hillaire, "A Scalable and Production Ready Sky and Atmosphere
// Rendering Technique", EGSR 2020, and the Bruneton precomputed-scattering
// conventions):
//   * ray/sphere intersections against the planet radius and the atmosphere
//     shell;
//   * exponential Rayleigh and Mie density profiles plus an ozone tent profile;
//   * optical depth accumulated by ray marching, with analytic transmittance
//     exp(-tau);
//   * Rayleigh phase 3/(16*pi)*(1 + mu^2) and Henyey-Greenstein Mie phase;
//   * single scattering integrated along the view ray against the transmittance
//     LUT, with explicit planet-sphere occlusion of the sun beam: when the
//     ray from a sample towards the sun intersects the ground sphere, the
//     direct sun visibility is exactly zero (never the ground-truncated
//     transmittance);
//   * multiple scattering following the Hillaire (2020) energy-compensation
//     closure (section 5.5.3, Eq. 7-8) with TWO INDEPENDENT spherical
//     integrals: the second-order scattering transfer `L_2ndOrder` (isotropic
//     phase, unit source E_I = 1, ground-bounce component included through
//     the ground albedo) and the *dimensionless* medium-transfer ratio
//     `f_ms = ∫ L_f p_u dω` with `L_f(x,v) = ∫ σ_s(x) T(x, x - t v) dt`
//     (no sun irradiance, no phase function, no planet-shadow visibility),
//     combined as `Psi_ms = L_2ndOrder / (1 - f_ms)`. The LUT therefore
//     stores a TRANSFER FUNCTION per unit sun irradiance; the real SunState
//     irradiance is applied exactly once, where the LUT is consumed in
//     `sky_radiance`. The ground albedo participates in the ground-bounce
//     radiance, never as the term of the geometric-series denominator.
//
// Coordinates: the renderer world is +Y up and the RC field stays local to the
// render origin. The atmosphere math therefore treats the camera as sitting
// `observer_altitude_m` above the ground and uses the scalar ray/sphere
// quadratic on the *radius* instead of subtracting million-metre Earth
// coordinates. The sun is a collimated source of irradiance
// `sun_irradiance.rgb` with angular radius `sun_angular_radius_radians`.

const PI: f32 = 3.141592653589793;
const ATMOSPHERE_STEPS: i32 = 32;
const SPHERE_SAMPLES: i32 = 32;
const OZONE_CENTER_M: f32 = 25000.0;
const OZONE_WIDTH_M: f32 = 15000.0;
const GOLDEN_ANGLE: f32 = 2.399963229728653;
// Unit directional source E_I = 1 (Hillaire 2020, Eq. 7): the normalized
// irradiance the multiple-scattering transfer LUT is built against.
const UNIT_IRRADIANCE: vec3<f32> = vec3<f32>(1.0, 1.0, 1.0);
// Relative band around the planet radius inside which the t = 0 tangent root
// degenerates the ground-intersection quadratic (see `ray_intersects_ground`).
// f32 evaluation of `sample_radius` at Earth scale carries ~0.2 m of noise,
// so the band is ~0.64 m: wide enough to swallow that noise, narrow enough to
// leave a real observer altitude (>= 2 m) on the exact quadratic branch.
const SURFACE_BAND_EPSILON: f32 = 1e-7;

struct AtmosphereUniform {
    planet_radius_m: f32,
    atmosphere_height_m: f32,
    rayleigh_scale_height_m: f32,
    mie_scale_height_m: f32,
    mie_anisotropy: f32,
    sun_angular_radius_radians: f32,
    observer_altitude_m: f32,
    _padding: f32,
    sun_direction: vec4<f32>,
    // Sun irradiance (radiance * disk solid angle) driving the scattering.
    sun_irradiance: vec4<f32>,
    rayleigh_scattering: vec4<f32>,
    mie_scattering: vec4<f32>,
    mie_extinction: vec4<f32>,
    ozone_absorption: vec4<f32>,
    ground_albedo: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> atmosphere: AtmosphereUniform;
@group(0) @binding(1)
var transmittance_lut: texture_2d<f32>;
@group(0) @binding(2)
var multi_scattering_lut: texture_2d<f32>;
@group(0) @binding(4)
var lut_sampler: sampler;

struct FullscreenVertexOutput {
    @builtin(position) position: vec4<f32>,
    // (u, v) of the texel this fragment writes, in the sampling convention of
    // the target texture, so generation and lookup agree without any flip.
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

// Nearest positive ray/sphere intersection, or -1 when the ray misses.
// `r0` is the ray-origin radius from the planet centre and `mu` the cosine
// against the local up, so the quadratic is solved on scalars only.
fn ray_sphere_near(r0: f32, mu: f32, radius: f32) -> f32 {
    let b = r0 * mu;
    let c = r0 * r0 - radius * radius;
    let discriminant = b * b - c;
    if (discriminant < 0.0) {
        return -1.0;
    }
    let root = sqrt(discriminant);
    let near_t = -b - root;
    if (near_t >= 0.0) {
        return near_t;
    }
    let far_t = -b + root;
    if (far_t >= 0.0) {
        return far_t;
    }
    return -1.0;
}

// Distance travelled before leaving the atmosphere shell or hitting the ground.
fn atmosphere_distance(r0: f32, mu: f32) -> f32 {
    let top = ray_sphere_near(
        r0,
        mu,
        atmosphere.planet_radius_m + atmosphere.atmosphere_height_m,
    );
    let ground = ray_sphere_near(r0, mu, atmosphere.planet_radius_m);
    if (ground > 0.0 && (top < 0.0 || ground < top)) {
        return ground;
    }
    return max(top, 0.0);
}

fn sample_radius(r0: f32, mu: f32, distance: f32) -> f32 {
    return sqrt(max(r0 * r0 + 2.0 * r0 * mu * distance + distance * distance, 0.0));
}

// True when the ray from a sample point at `sample_radius` towards the sun
// (cosine `sun_mu` against the local up) intersects the ground sphere, i.e.
// the planet itself occludes the direct sun beam at that point.
//
// Exact-surface edge case: when `sample_radius == planet_radius` the quadratic
// always has the root t = 0 (c = 0), so the nearest-root test alone can never
// see the *second* root at t = -2 r mu, which is strictly positive exactly
// when the sun is below the horizon. Inside the f32 surface band the decision
// is therefore taken directly on the horizon:
//   * surface + sun above horizon  -> visible
//   * surface + tangent (mu = 0)   -> visible
//   * surface + sun below horizon  -> OCCLUDED
// Strictly above the band (c > 0) both roots share the sign of -mu, so the
// nearest root is positive exactly when the sun sits below the local tangent:
//   * altitude + ray above tangent -> visible
//   * altitude + ray below tangent -> occluded
// Near the tangent the predicate degrades gracefully to the geometric
// condition (discriminant < 0 -> no intersection), never to a NaN.
// `ray_sphere_near` itself is intentionally left untouched: its other callers
// (atmosphere_distance, the ground-bounce distance) rely on its current
// nearest-positive-root contract.
fn ray_intersects_ground(sample_radius: f32, sun_mu: f32) -> bool {
    let planet_radius = atmosphere.planet_radius_m;
    if (sample_radius <= planet_radius * (1.0 + SURFACE_BAND_EPSILON)) {
        return sun_mu < 0.0;
    }
    return ray_sphere_near(sample_radius, sun_mu, planet_radius) > 0.0;
}

// Local up at a point of the view ray, expressed without large-coordinate
// subtraction: the planet centre sits at render (0, -r0, 0).
fn sample_up(r0: f32, distance: f32, direction: vec3<f32>, radius: f32) -> vec3<f32> {
    return vec3<f32>(
        distance * direction.x,
        distance * direction.y + r0,
        distance * direction.z,
    ) / max(radius, 1.0);
}

fn rayleigh_density(height_m: f32) -> f32 {
    return exp(-max(height_m, 0.0) / atmosphere.rayleigh_scale_height_m);
}

fn mie_density(height_m: f32) -> f32 {
    return exp(-max(height_m, 0.0) / atmosphere.mie_scale_height_m);
}

fn ozone_density(height_m: f32) -> f32 {
    return max(0.0, 1.0 - abs(height_m - OZONE_CENTER_M) / OZONE_WIDTH_M);
}

fn hg_phase(cos_theta: f32, g: f32) -> f32 {
    let denominator = max(1.0 + g * g - 2.0 * g * cos_theta, 1e-4);
    return (1.0 - g * g) / (4.0 * PI * pow(denominator, 1.5));
}

fn optical_depth(r0: f32, mu: f32, distance: f32) -> vec3<f32> {
    let dt = distance / f32(ATMOSPHERE_STEPS);
    var rayleigh = 0.0;
    var mie = 0.0;
    var ozone = 0.0;
    for (var step = 0; step < ATMOSPHERE_STEPS; step = step + 1) {
        let t = (f32(step) + 0.5) * dt;
        let radius = sample_radius(r0, mu, t);
        let height = max(radius - atmosphere.planet_radius_m, 0.0);
        rayleigh = rayleigh + rayleigh_density(height) * dt;
        mie = mie + mie_density(height) * dt;
        ozone = ozone + ozone_density(height) * dt;
    }
    return atmosphere.rayleigh_scattering.rgb * rayleigh
        + atmosphere.mie_extinction.rgb * mie
        + atmosphere.ozone_absorption.rgb * ozone;
}

fn transmittance_to_top(r0: f32, mu: f32) -> vec3<f32> {
    let distance = atmosphere_distance(r0, mu);
    return exp(-optical_depth(r0, mu, distance));
}

// Transmittance LUT parameterisation: u = mu in [0, 1], v = sqrt(h / H) so the
// steep near-ground variation keeps resolution inside the 64-row table.
fn transmittance_uv(height_m: f32, mu: f32) -> vec2<f32> {
    let v = sqrt(clamp(height_m / atmosphere.atmosphere_height_m, 0.0, 1.0));
    return vec2<f32>(clamp(mu * 0.5 + 0.5, 0.0, 1.0), v);
}

fn transmittance_lookup(height_m: f32, mu: f32) -> vec3<f32> {
    return textureSampleLevel(
        transmittance_lut,
        lut_sampler,
        transmittance_uv(height_m, mu),
        0.0,
    ).rgb;
}

// Single scattering along one view ray. The source strength is the passed
// `sun_irradiance` (the real SunState irradiance for the sky path, the unit
// source E_I = 1 for the multiple-scattering transfer LUT); `isotropic`
// selects the uniform phase used by the multiple scattering approximation.
fn single_scattering(
    r0: f32,
    view_mu: f32,
    view_direction: vec3<f32>,
    sun_direction: vec3<f32>,
    isotropic: bool,
    sun_irradiance: vec3<f32>,
) -> vec3<f32> {
    let cos_theta = clamp(dot(view_direction, sun_direction), -1.0, 1.0);
    var rayleigh_phase = 3.0 / (16.0 * PI) * (1.0 + cos_theta * cos_theta);
    var mie_phase = hg_phase(cos_theta, atmosphere.mie_anisotropy);
    if (isotropic) {
        rayleigh_phase = 1.0 / (4.0 * PI);
        mie_phase = 1.0 / (4.0 * PI);
    }

    let distance = atmosphere_distance(r0, view_mu);
    let dt = distance / f32(ATMOSPHERE_STEPS);
    var view_optical_depth = vec3<f32>(0.0);
    var rayleigh_sum = vec3<f32>(0.0);
    var mie_sum = vec3<f32>(0.0);
    for (var step = 0; step < ATMOSPHERE_STEPS; step = step + 1) {
        let t = (f32(step) + 0.5) * dt;
        let radius = sample_radius(r0, view_mu, t);
        let height = max(radius - atmosphere.planet_radius_m, 0.0);
        let rho_rayleigh = rayleigh_density(height);
        let rho_mie = mie_density(height);
        let rho_ozone = ozone_density(height);
        view_optical_depth = view_optical_depth
            + (atmosphere.rayleigh_scattering.rgb * rho_rayleigh
                + atmosphere.mie_extinction.rgb * rho_mie
                + atmosphere.ozone_absorption.rgb * rho_ozone) * dt;

        let up = sample_up(r0, t, view_direction, radius);
        let sun_mu = clamp(dot(up, sun_direction), -1.0, 1.0);
        // Planet shadow: when the sun beam from this sample hits the ground
        // sphere the direct sun visibility is exactly zero. The transmittance
        // LUT alone is not usable here, because it is integrated only up to
        // the ground and would leak a bright "sun" through the planet.
        var sun_transmittance = vec3<f32>(0.0);
        if (!ray_intersects_ground(radius, sun_mu)) {
            sun_transmittance = transmittance_lookup(height, sun_mu);
        }
        let transmittance = exp(-view_optical_depth) * sun_transmittance;
        rayleigh_sum = rayleigh_sum + transmittance * rho_rayleigh * dt;
        mie_sum = mie_sum + transmittance * rho_mie * dt;
    }

    return sun_irradiance
        * (atmosphere.rayleigh_scattering.rgb * rayleigh_sum * rayleigh_phase
            + atmosphere.mie_scattering.rgb * mie_sum * mie_phase);
}

// Lambertian response of the ground for view rays that reach the surface.
// Driven by the same `sun_irradiance` selector as `single_scattering`.
fn ground_reflection(
    r0: f32,
    view_mu: f32,
    view_direction: vec3<f32>,
    sun_direction: vec3<f32>,
    distance: f32,
    sun_irradiance: vec3<f32>,
) -> vec3<f32> {
    let radius = sample_radius(r0, view_mu, distance);
    let up = sample_up(r0, distance, view_direction, radius);
    let sun_cos = max(dot(up, sun_direction), 0.0);
    let height = max(radius - atmosphere.planet_radius_m, 0.0);
    let sun_transmittance = transmittance_lookup(height, sun_cos);
    return atmosphere.ground_albedo.rgb * sun_irradiance
        * sun_transmittance * sun_cos / PI;
}

fn multi_scattering_lookup(height_m: f32, sun_mu: f32) -> vec3<f32> {
    let uv = vec2<f32>(
        clamp(sun_mu * 0.5 + 0.5, 0.0, 1.0),
        sqrt(clamp(height_m / atmosphere.atmosphere_height_m, 0.0, 1.0)),
    );
    return textureSampleLevel(multi_scattering_lut, lut_sampler, uv, 0.0).rgb;
}

fn fibonacci_direction(index: i32, count: i32) -> vec3<f32> {
    let i = f32(index) + 0.5;
    let cos_theta = 1.0 - 2.0 * i / f32(count);
    let sin_theta = sqrt(max(1.0 - cos_theta * cos_theta, 0.0));
    let phi = i * GOLDEN_ANGLE;
    return vec3<f32>(sin_theta * cos(phi), cos_theta, sin_theta * sin(phi));
}

// Hillaire (2020) Eq. 8 inner transfer integral, evaluated by the same
// deterministic ray march used by the rest of the LUT generation:
//
//     L_f(x, v) = ∫ σ_s(x) * T(x, x - t v) dt
//
// σ_s is the volume scattering coefficient (Rayleigh + Mie) at the sample
// point and T the analytic transmittance exp(-τ) accumulated from the ray
// origin to it. This is the PURE MEDIUM TRANSFER: no sun irradiance, no phase
// function, no planet-shadow visibility and no directional-light angular
// dependence enter it. σ_s [m^-1] times dt [m] makes the result a
// dimensionless RGB ratio, the per-channel fraction of the light scattered at
// x that the medium carries back along v.
fn ms_transfer_integral(r0: f32, mu: f32) -> vec3<f32> {
    let distance = atmosphere_distance(r0, mu);
    let dt = distance / f32(ATMOSPHERE_STEPS);
    var optical_depth = vec3<f32>(0.0);
    var transfer = vec3<f32>(0.0);
    for (var step = 0; step < ATMOSPHERE_STEPS; step = step + 1) {
        let t = (f32(step) + 0.5) * dt;
        let radius = sample_radius(r0, mu, t);
        let height = max(radius - atmosphere.planet_radius_m, 0.0);
        let rho_rayleigh = rayleigh_density(height);
        let rho_mie = mie_density(height);
        let rho_ozone = ozone_density(height);
        let sigma_s = atmosphere.rayleigh_scattering.rgb * rho_rayleigh
            + atmosphere.mie_scattering.rgb * rho_mie;
        optical_depth = optical_depth
            + (atmosphere.rayleigh_scattering.rgb * rho_rayleigh
                + atmosphere.mie_extinction.rgb * rho_mie
                + atmosphere.ozone_absorption.rgb * rho_ozone) * dt;
        transfer = transfer + sigma_s * exp(-optical_depth) * dt;
    }
    return transfer;
}

// Hillaire (2020) energy-compensation multiple scattering, section 5.5.3.
//
// TWO INDEPENDENT integrals are accumulated over the same deterministic
// Fibonacci sphere (uniform phase p_u = 1/(4π), so the sphere average of the
// per-direction integrands IS the integral):
//
// 1. `l_2nd_order` — Eq. 7, the second-order scattering transfer
//
//        L_2ndOrder = ∫Ω L'(xs, -ω) p_u dω
//
//    where L' is the single scattering from the directional light (WITH its
//    planet-shadow visibility) plus the ground contribution, both driven by
//    the NORMALIZED unit source E_I = 1. The result is a TRANSFER FUNCTION
//    [radiance per unit sun irradiance], not a radiance: the real SunState
//    irradiance is deliberately absent from this LUT and is applied exactly
//    once, where the LUT is consumed in `sky_radiance`. The ground albedo
//    participates here, in the ground-bounce radiance.
//
// 2. `f_ms` — Eq. 8, the dimensionless medium-transfer ratio
//
//        f_ms = ∫Ω L_f(xs, -ω) p_u dω,   L_f(x, v) = ∫ σ_s(x) T(x, x - t v) dt
//
//    accumulated by `ms_transfer_integral`: no sun irradiance, no phase
//    function, no shadow visibility, no directional dependence. It is NOT
//    derived from `l_2nd_order` by any luminance ratio; the two accumulators
//    are distinct integrals of distinct integrands.
//
// The bounce series then closes, per channel, as
//
//     F_ms   = 1 / (1 - f_ms)          (f_ms clamped into [0, 1) first)
//     Psi_ms = L_2ndOrder * F_ms = L_2ndOrder / (1 - f_ms)
//
// and Psi_ms — still a transfer function — is what the LUT stores.
fn multi_scattering_texel(height_m: f32, sun_mu: f32) -> vec3<f32> {
    let r0 = atmosphere.planet_radius_m + height_m;
    let sun_direction = vec3<f32>(sqrt(max(1.0 - sun_mu * sun_mu, 0.0)), sun_mu, 0.0);
    var l_2nd_order = vec3<f32>(0.0);
    var f_ms_sum = vec3<f32>(0.0);
    for (var sample_index = 0; sample_index < SPHERE_SAMPLES; sample_index = sample_index + 1) {
        let direction = fibonacci_direction(sample_index, SPHERE_SAMPLES);
        let mu = clamp(direction.y, -1.0, 1.0);
        // Eq. 7 integrand against the unit source E_I = 1.
        l_2nd_order = l_2nd_order
            + single_scattering(r0, mu, direction, sun_direction, true, UNIT_IRRADIANCE);
        // Ground-bounce component: unit-source sunlight reflected by the
        // planet surface towards this direction re-enters the second-order
        // scattering field.
        let ground_distance = ray_sphere_near(r0, mu, atmosphere.planet_radius_m);
        if (ground_distance > 0.0) {
            l_2nd_order = l_2nd_order
                + ground_reflection(r0, mu, direction, sun_direction, ground_distance, UNIT_IRRADIANCE);
        }
        // Eq. 8 integrand: independent medium-transfer accumulation.
        f_ms_sum = f_ms_sum + ms_transfer_integral(r0, mu);
    }
    l_2nd_order = l_2nd_order / f32(SPHERE_SAMPLES);
    // Dimensionless RGB transfer ratio, clamped into the physically valid
    // convergence interval [0, 1) *before* the geometric series is formed.
    let f_ms = clamp(f_ms_sum / f32(SPHERE_SAMPLES), vec3<f32>(0.0), vec3<f32>(0.999));
    return l_2nd_order / (1.0 - f_ms);
}

fn sky_radiance(view_direction: vec3<f32>) -> vec3<f32> {
    let r0 = atmosphere.planet_radius_m + atmosphere.observer_altitude_m;
    let view_mu = clamp(view_direction.y, -1.0, 1.0);
    let sun_direction = atmosphere.sun_direction.xyz;
    // The single access to the real SunState irradiance in this module: every
    // radiance term below multiplies by it exactly once.
    let sun_irradiance = atmosphere.sun_irradiance.rgb;
    var radiance = single_scattering(r0, view_mu, view_direction, sun_direction, false, sun_irradiance);
    let ground_distance = ray_sphere_near(r0, view_mu, atmosphere.planet_radius_m);
    if (ground_distance > 0.0) {
        radiance = radiance
            + ground_reflection(r0, view_mu, view_direction, sun_direction, ground_distance, sun_irradiance);
    }
    // Isotropic multiple-scattering term; it is a real part of the model
    // computed from the atmosphere itself, never an aesthetic tint. The LUT
    // holds the unit-source transfer function Psi_ms, so the real SunState
    // irradiance is applied HERE, exactly once, at consumption.
    radiance = radiance
        + multi_scattering_lookup(
            atmosphere.observer_altitude_m,
            clamp(sun_direction.y, -1.0, 1.0),
        ) * sun_irradiance;
    return max(radiance, vec3<f32>(0.0));
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

@fragment
fn fs_transmittance(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let mu = input.uv.x * 2.0 - 1.0;
    let height = input.uv.y * input.uv.y * atmosphere.atmosphere_height_m;
    let r0 = atmosphere.planet_radius_m + height;
    return vec4<f32>(transmittance_to_top(r0, mu), 1.0);
}

@fragment
fn fs_multi_scattering(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let sun_mu = input.uv.x * 2.0 - 1.0;
    let height = input.uv.y * input.uv.y * atmosphere.atmosphere_height_m;
    return vec4<f32>(multi_scattering_texel(height, sun_mu), 1.0);
}

@fragment
fn fs_sky_view(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    let azimuth = input.uv.x * 2.0 * PI - PI;
    let elevation = input.uv.y * PI - 0.5 * PI;
    let cos_elevation = cos(elevation);
    let direction = vec3<f32>(
        cos_elevation * cos(azimuth),
        sin(elevation),
        cos_elevation * sin(azimuth),
    );
    return vec4<f32>(sky_radiance(direction), 1.0);
}

@fragment
fn fs_environment_cube(input: FullscreenVertexOutput) -> @location(0) vec4<f32> {
    return vec4<f32>(sky_radiance(cube_direction(input.face, input.uv)), 1.0);
}



