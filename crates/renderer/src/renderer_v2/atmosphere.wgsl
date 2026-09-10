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
//     closure: a second-order radiance `L_2ndOrder` (isotropic phase, full
//     spherical integration, ground-bounce component included through the
//     ground albedo) and a *dimensionless* energy-transfer ratio `f_ms`
//     derived from it, combined as `Psi_ms = L_2ndOrder / (1 - f_ms)`. The
//     ground albedo participates in the ground-bounce radiance, never as the
//     term of the geometric-series denominator.
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
// the planet itself occludes the direct sun beam at that point. A strictly
// positive intersection distance is required, so a ray leaving a point exactly
// on the surface towards the sky is *not* an occlusion. Near the horizon the
// predicate degrades gracefully to the geometric tangent condition
// (discriminant < 0 -> no intersection), never to a NaN.
fn ray_intersects_ground(sample_radius: f32, sun_mu: f32) -> bool {
    return ray_sphere_near(sample_radius, sun_mu, atmosphere.planet_radius_m) > 0.0;
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

// Single scattering along one view ray. The source strength is the sun's
// irradiance; `isotropic` selects the uniform phase used by the multiple
// scattering approximation.
fn single_scattering(
    r0: f32,
    view_mu: f32,
    view_direction: vec3<f32>,
    sun_direction: vec3<f32>,
    isotropic: bool,
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

    return atmosphere.sun_irradiance.rgb
        * (atmosphere.rayleigh_scattering.rgb * rayleigh_sum * rayleigh_phase
            + atmosphere.mie_scattering.rgb * mie_sum * mie_phase);
}

// Lambertian response of the ground for view rays that reach the surface.
fn ground_reflection(
    r0: f32,
    view_mu: f32,
    view_direction: vec3<f32>,
    sun_direction: vec3<f32>,
    distance: f32,
) -> vec3<f32> {
    let radius = sample_radius(r0, view_mu, distance);
    let up = sample_up(r0, distance, view_direction, radius);
    let sun_cos = max(dot(up, sun_direction), 0.0);
    let height = max(radius - atmosphere.planet_radius_m, 0.0);
    let sun_transmittance = transmittance_lookup(height, sun_cos);
    return atmosphere.ground_albedo.rgb * atmosphere.sun_irradiance.rgb
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

// Rec.709 luminance of a linear radiance triple; used only to collapse the
// energy-transfer ratio to a scalar, exactly as in Hillaire (2020).
fn luminance3(color: vec3<f32>) -> f32 {
    return dot(color, vec3<f32>(0.2126, 0.7152, 0.0722));
}

// Hillaire (2020) energy-compensation multiple scattering.
//
// Units: `l_2nd_order` is the second-order in-scattered RADIANCE
// [W m^-2 sr^-1] driven by the sun irradiance `E = sun_irradiance.rgb`
// [W m^-2]; it is the full-sphere average of the isotropic-phase single
// scattering plus the ground-bounce component, which is where the ground
// albedo participates in the model.
//
// `f_ms` is the DIMENSIONLESS energy-transfer ratio of the medium: the
// fraction of the incoming solar irradiance that one isotropic bounce returns
// to the medium,
//
//     f_ms = 4*pi * luminance(L_2ndOrder) / luminance(E)   (unitless)
//
// Because both terms are driven by the same sun irradiance, `f_ms` is
// independent of the sun intensity even though the LUT keeps the irradiance
// folded into `l_2nd_order`. The bounce series then closes as
//
//     F_ms   = 1 / (1 - f_ms)
//     Psi_ms = L_2ndOrder * F_ms
//
// `l_2nd_order` (a radiance) and `f_ms` (a ratio) are distinct quantities and
// no radiance ever appears in the geometric-series denominator.
fn multi_scattering_texel(height_m: f32, sun_mu: f32) -> vec3<f32> {
    let r0 = atmosphere.planet_radius_m + height_m;
    let sun_direction = vec3<f32>(sqrt(max(1.0 - sun_mu * sun_mu, 0.0)), sun_mu, 0.0);
    var l_2nd_order = vec3<f32>(0.0);
    for (var sample_index = 0; sample_index < SPHERE_SAMPLES; sample_index = sample_index + 1) {
        let direction = fibonacci_direction(sample_index, SPHERE_SAMPLES);
        let mu = clamp(direction.y, -1.0, 1.0);
        l_2nd_order = l_2nd_order
            + single_scattering(r0, mu, direction, sun_direction, true);
        // Ground-bounce component: sunlight reflected by the planet surface
        // towards this direction re-enters the second-order scattering field.
        let ground_distance = ray_sphere_near(r0, mu, atmosphere.planet_radius_m);
        if (ground_distance > 0.0) {
            l_2nd_order = l_2nd_order
                + ground_reflection(r0, mu, direction, sun_direction, ground_distance);
        }
    }
    l_2nd_order = l_2nd_order / f32(SPHERE_SAMPLES);
    // Dimensionless transfer ratio, clamped into the physically valid
    // convergence interval [0, 1) *before* the geometric series is formed.
    let sun_luminance = max(luminance3(atmosphere.sun_irradiance.rgb), 1e-6);
    let f_ms = clamp(4.0 * PI * luminance3(l_2nd_order) / sun_luminance, 0.0, 0.999);
    let f_ms_factor = 1.0 / (1.0 - f_ms);
    return l_2nd_order * f_ms_factor;
}

fn sky_radiance(view_direction: vec3<f32>) -> vec3<f32> {
    let r0 = atmosphere.planet_radius_m + atmosphere.observer_altitude_m;
    let view_mu = clamp(view_direction.y, -1.0, 1.0);
    let sun_direction = atmosphere.sun_direction.xyz;
    var radiance = single_scattering(r0, view_mu, view_direction, sun_direction, false);
    let ground_distance = ray_sphere_near(r0, view_mu, atmosphere.planet_radius_m);
    if (ground_distance > 0.0) {
        radiance = radiance
            + ground_reflection(r0, view_mu, view_direction, sun_direction, ground_distance);
    }
    // Isotropic multiple-scattering term; it is a real part of the model
    // computed from the atmosphere itself, never an aesthetic tint.
    radiance = radiance
        + multi_scattering_lookup(
            atmosphere.observer_altitude_m,
            clamp(sun_direction.y, -1.0, 1.0),
        );
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



