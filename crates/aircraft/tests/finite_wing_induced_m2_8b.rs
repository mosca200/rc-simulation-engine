//! M2.8B — Deterministic Finite-Wing Induced Physics tests.
//!
//! All fixtures are synthetic. No real LT-40 or Clark Y data is used.

use aircraft::{AircraftSimulation, AircraftSimulationConfig, evaluate_aerodynamic_wrench};
use model::{AircraftModel, AircraftModelLoader};
use sim_core::{AeroEnvironment, BodyWrench, RigidBodyState};
use sim_math::{Orientation, Vec3};
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn state_at_velocity(vx: f64) -> RigidBodyState {
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(vx, 0.0, 0.0),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn state_at_alpha(airspeed: f64, alpha_deg: f64) -> RigidBodyState {
    let alpha = alpha_deg.to_radians();
    RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(airspeed * alpha.cos(), 0.0, airspeed * alpha.sin()),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    }
}

fn standard_env() -> AeroEnvironment {
    AeroEnvironment::new(1.225, Vec3::zeros()).unwrap()
}

fn config() -> AircraftSimulationConfig {
    AircraftSimulationConfig::new(0.002, Vec3::new(0.0, 0.0, -9.81), standard_env()).unwrap()
}

/// Builds a v5 model JSON with the given polars, elements, and surfaces.
fn build_v5_model(
    polars_json: &str,
    polar_families_json: &str,
    elements_json: &str,
    surfaces_json: &str,
) -> AircraftModel {
    let json = format!(
        r#"{{
  "schema_version": 5,
  "model_id": "synthetic-m2-8b-test",
  "display_name": "Synthetic M2.8B Test",
  "classification": "synthetic_test",
  "reference_aircraft": null,
  "rigid_body": {{
    "mass_kg": 2.0,
    "inertia_body_kg_m2": [[0.2,0,0],[0,0.3,0],[0,0,0.4]]
  }},
  "aerodynamics": {{
    "kinematic_viscosity_m2_s": 1.5e-5,
    "polars": {polars_json},
    "polar_families": {polar_families_json},
    "elements": {elements_json},
    "surfaces": {surfaces_json}
  }},
  "controls": {{
    "response": {{"roll":{{"rate":1,"expo":0}},"pitch":{{"rate":1,"expo":0}},"yaw":{{"rate":1,"expo":0}}}},
    "servos": {{
      "aileron":{{"min_angle_rad":-0.5,"neutral_angle_rad":0.0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false}},
      "elevator":{{"min_angle_rad":-0.5,"neutral_angle_rad":0.0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false}},
      "rudder":{{"min_angle_rad":-0.5,"neutral_angle_rad":0.0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false}}
    }}
  }},
  "control_surface_bindings": [],
  "propulsion": null,
  "presentation": null
}}"#
    );
    AircraftModelLoader::from_json_str(&json).unwrap()
}

/// Linear polar: CL = alpha (rad), CD = cd0, CM = 0.
fn linear_polar_json(id: &str, cd0: f64) -> String {
    let samples: Vec<String> = (-10..=10)
        .map(|i| {
            let alpha = i as f64 * 0.05;
            format!(r#"{{"alpha_rad":{alpha},"cl":{alpha},"cd":{cd0},"cm":0.0}}"#)
        })
        .collect();
    format!(r#"{{"id":"{id}","samples":[{}]}}"#, samples.join(","))
}

fn element_json(id: &str, pos: [f64; 3], area: f64, chord: f64, polar_id: &str) -> String {
    format!(
        r#"{{"id":"{id}","position_body_m":[{},{},{}],"orientation_body_from_element_wxyz":[1,0,0,0],"area_m2":{area},"chord_m":{chord},"polar_binding":{{"kind":"polar","polar_id":"{polar_id}"}}}}"#,
        pos[0], pos[1], pos[2]
    )
}

fn element_json_reynolds(
    id: &str,
    pos: [f64; 3],
    area: f64,
    chord: f64,
    family_id: &str,
) -> String {
    format!(
        r#"{{"id":"{id}","position_body_m":[{},{},{}],"orientation_body_from_element_wxyz":[1,0,0,0],"area_m2":{area},"chord_m":{chord},"polar_binding":{{"kind":"reynolds_family","family_id":"{family_id}"}}}}"#,
        pos[0], pos[1], pos[2]
    )
}

fn reynolds_family_json(id: &str, nodes: &str) -> String {
    format!(r#"{{"id":"{id}","nodes":[{nodes}]}}"#)
}

fn reynolds_node_json(re: f64, samples_json: &str) -> String {
    format!(r#"{{"reynolds_number":{re},"samples":{samples_json}}}"#)
}

fn linear_samples_json(cd0: f64) -> String {
    let samples: Vec<String> = (-10..=10)
        .map(|i| {
            let alpha = i as f64 * 0.05;
            format!(r#"{{"alpha_rad":{alpha},"cl":{alpha},"cd":{cd0},"cm":0.0}}"#)
        })
        .collect();
    format!("[{}]", samples.join(","))
}

fn linear_polar_with_slope_json(id: &str, slope: f64, cd0: f64) -> String {
    let samples: Vec<String> = (-10..=10)
        .map(|i| {
            let alpha = i as f64 * 0.05;
            let cl = slope * alpha;
            format!(r#"{{"alpha_rad":{alpha},"cl":{cl},"cd":{cd0},"cm":0.0}}"#)
        })
        .collect();
    format!(r#"{{"id":"{id}","samples":[{}]}}"#, samples.join(","))
}

fn linear_samples_with_slope_json(slope: f64, cd0: f64) -> String {
    let samples: Vec<String> = (-10..=10)
        .map(|i| {
            let alpha = i as f64 * 0.05;
            let cl = slope * alpha;
            format!(r#"{{"alpha_rad":{alpha},"cl":{cl},"cd":{cd0},"cm":0.0}}"#)
        })
        .collect();
    format!("[{}]", samples.join(","))
}

fn surface_json(id: &str, element_ids: &[&str], span_m: f64, e: f64) -> String {
    let ids: Vec<String> = element_ids.iter().map(|s| format!("\"{s}\"")).collect();
    format!(
        r#"{{"id":"{id}","element_ids":[{}],"span_axis_body":[0,1,0],"span_m":{span_m},"span_efficiency_factor":{e}}}"#,
        ids.join(",")
    )
}

/// First-stage wrench from one simulation step.
fn first_stage_wrench(
    model: &AircraftModel,
    state: &RigidBodyState,
    cfg: &AircraftSimulationConfig,
) -> BodyWrench {
    let effective = model
        .aero_elements()
        .iter()
        .map(|re| *re.element())
        .collect::<Vec<_>>();
    evaluate_aerodynamic_wrench(state, &effective, model, cfg.aero_environment())
}

// ---------------------------------------------------------------------------
// Legacy regression: no surfaces
// ---------------------------------------------------------------------------

#[test]
fn legacy_v1_model_unchanged() {
    let json = r#"{
  "schema_version": 1,
  "model_id": "synthetic-legacy",
  "display_name": "Legacy",
  "rigid_body": {"mass_kg":2.0,"inertia_body_kg_m2":[[0.2,0,0],[0,0.3,0],[0,0,0.4]]},
  "aerodynamics": {
    "polars": [{"id":"p1","samples":[
      {"alpha_rad":-0.3,"cl":-0.5,"cd":0.04,"cm":0.0},
      {"alpha_rad": 0.0,"cl": 0.0,"cd":0.02,"cm":0.0},
      {"alpha_rad": 0.3,"cl": 0.5,"cd":0.04,"cm":0.0}
    ]}],
    "elements": [{"id":"e1","position_body_m":[0,0,0],"orientation_body_from_element_wxyz":[1,0,0,0],"area_m2":0.5,"chord_m":0.25,"polar_id":"p1"}]
  },
  "controls": {"response":{"roll":{"rate":1,"expo":0},"pitch":{"rate":1,"expo":0},"yaw":{"rate":1,"expo":0}},"servos":{"aileron":{"min_angle_rad":-0.5,"neutral_angle_rad":0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false},"elevator":{"min_angle_rad":-0.5,"neutral_angle_rad":0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false},"rudder":{"min_angle_rad":-0.5,"neutral_angle_rad":0,"max_angle_rad":0.5,"max_speed_rad_s":10,"reversed":false}}},
  "control_surface_bindings": [],
  "propulsion": null,
  "presentation": null
}"#;
    let model = AircraftModelLoader::from_json_str(json).unwrap();
    assert!(model.aero_surfaces().is_empty());
    // Create a state with nonzero alpha (velocity has a positive Z component in body frame)
    let alpha_geom = 5.0_f64.to_radians();
    let state = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(20.0 * alpha_geom.cos(), 0.0, 20.0 * alpha_geom.sin()),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let wrench = first_stage_wrench(&model, &state, &config());
    assert!(
        wrench.force_body_n.z < 0.0,
        "positive alpha should produce lift (negative z in FRD)"
    );
}

#[test]
fn v5_empty_surfaces_remains_legacy() {
    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("e1", [0.0, -0.2, 0.0], 0.3, 0.2, "lp"),
            element_json("e2", [0.0, 0.2, 0.0], 0.4, 0.25, "lp"),
        ),
        "[]",
    );
    assert!(model.aero_surfaces().is_empty());
    let state = state_at_alpha(20.0, 5.0);

    // Compare evaluate_aerodynamic_wrench against explicit element-by-element sum
    let wrench = first_stage_wrench(&model, &state, &config());

    let effective: Vec<sim_core::AeroElement> = model
        .aero_elements()
        .iter()
        .map(|re| *re.element())
        .collect();
    let mut manual_wrench = sim_core::BodyWrench::zero();
    for (eff, runtime) in effective.iter().zip(model.aero_elements()) {
        let output = aircraft::evaluate_aircraft_aero_element(
            &state,
            eff,
            runtime,
            &model,
            &sim_core::AeroEnvironment::new(1.225, sim_math::Vec3::zeros()).unwrap(),
        );
        let aero = output.aero();
        manual_wrench.force_body_n += aero.wrench_body.force_body_n;
        manual_wrench.moment_body_nm += aero.wrench_body.moment_body_nm;
    }

    // Bitwise-identical force and moment
    assert_eq!(wrench.force_body_n, manual_wrench.force_body_n);
    assert_eq!(wrench.moment_body_nm, manual_wrench.moment_body_nm);
}

// ---------------------------------------------------------------------------
// Analytic linear-polar test
// ---------------------------------------------------------------------------

#[test]
fn analytic_linear_polar_finite_wing_solution() {
    let a = 1.0; // dCL/dalpha for linear polar CL = alpha
    let chord = 0.2;
    let span = 1.0;
    let e_factor = 0.9;
    let area_per_element = chord * (span / 2.0);
    let ar = span * span / (2.0 * area_per_element);
    let alpha_geom_deg: f64 = 5.0;
    let alpha_geom = alpha_geom_deg.to_radians();

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("linear", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per_element, chord, "linear"),
            element_json("right", [0.0, 0.25, 0.0], area_per_element, chord, "linear"),
        ),
        &format!(
            "[{}]",
            surface_json("wing", &["left", "right"], span, e_factor)
        ),
    );
    assert_eq!(model.aero_surfaces().len(), 1);

    let vx = 25.0;
    let state = state_at_alpha(vx, alpha_geom_deg);
    let wrench = first_stage_wrench(&model, &state, &config());

    // Analytic solution:
    let denom = 1.0 + a / (PI * ar * e_factor);
    let cl_surface_expected = a * alpha_geom / denom;
    let alpha_i_expected = cl_surface_expected / (PI * ar * e_factor);
    let cdi_expected = cl_surface_expected * cl_surface_expected / (PI * ar * e_factor);

    let q = 0.5 * 1.225 * vx * vx;
    let total_area = 2.0 * area_per_element;
    let expected_lift = q * total_area * cl_surface_expected;
    let expected_induced_drag = q * total_area * cdi_expected;

    // Project force onto actual section lift and drag directions.
    // At alpha_geom with identity orientation, section velocity is (V*cos(a), 0, V*sin(a)).
    let alpha = alpha_geom;
    let v_hat = Vec3::new(alpha.cos(), 0.0, alpha.sin());
    let lift_dir = Vec3::y().cross(&v_hat);
    let drag_dir = -v_hat;

    let actual_lift = wrench.force_body_n.dot(&lift_dir);
    let actual_drag = wrench.force_body_n.dot(&drag_dir);

    let lift_tol = 1.0e-3 * expected_lift.abs().max(1.0);
    let drag_tol = 5.0e-3 * expected_induced_drag.abs().max(0.01);

    assert!(
        (actual_lift - expected_lift).abs() < lift_tol,
        "lift: actual={actual_lift:.6}, expected={expected_lift:.6}, tol={lift_tol:.6e}"
    );
    assert!(
        (actual_drag - expected_induced_drag).abs() < drag_tol,
        "induced drag: actual={actual_drag:.6}, expected={expected_induced_drag:.6}, tol={drag_tol:.6e}"
    );

    // Verify alpha_i sign for positive CL: alpha_i > 0
    assert!(alpha_i_expected > 0.0);
    // Verify CDi > 0
    assert!(cdi_expected > 0.0);
    // Verify that the finite-wing wrench differs from the quasi-2D wrench
    let model_2d = build_v5_model(
        &format!("[{}]", linear_polar_json("linear", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per_element, chord, "linear"),
            element_json("right", [0.0, 0.25, 0.0], area_per_element, chord, "linear"),
        ),
        "[]",
    );
    let wrench_2d = first_stage_wrench(&model_2d, &state, &config());
    assert!(
        (wrench.force_body_n - wrench_2d.force_body_n).norm() > 1e-6,
        "finite-wing wrench should differ from quasi-2D"
    );
}

// ---------------------------------------------------------------------------
// Finite-wing reduces lift compared to quasi-2D
// ---------------------------------------------------------------------------

#[test]
fn finite_wing_reduces_lift_vs_quasi2d() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    // Model WITHOUT surface (quasi-2D)
    let model_2d = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        "[]",
    );

    // Model WITH surface (finite wing)
    let model_3d = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let state = state_at_alpha(25.0, 5.0);
    let wrench_2d = first_stage_wrench(&model_2d, &state, &config());
    let wrench_3d = first_stage_wrench(&model_3d, &state, &config());

    let lift_2d = -wrench_2d.force_body_n.z;
    let lift_3d = -wrench_3d.force_body_n.z;

    assert!(
        lift_3d < lift_2d,
        "finite wing should reduce lift: 2D={lift_2d:.4}, 3D={lift_3d:.4}"
    );
    assert!(
        lift_3d > 0.0,
        "finite wing should still produce positive lift"
    );
}

// ---------------------------------------------------------------------------
// Force direction preservation
// ---------------------------------------------------------------------------

#[test]
fn force_direction_unchanged_by_finite_wing() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let state = state_at_alpha(20.0, 5.0);
    let wrench = first_stage_wrench(&model, &state, &config());

    // At positive alpha with identity orientation:
    // - Lift should be in -Z body (up in FRD)
    // - Drag should be in -X body (backward)
    // - No lateral force (symmetric aircraft, zero sideslip)
    assert!(wrench.force_body_n.z < 0.0, "lift should be negative Z");
    assert!(wrench.force_body_n.x < 0.0, "drag should be negative X");
    assert!(
        wrench.force_body_n.y.abs() < 1e-10,
        "symmetric aircraft should have zero lateral force"
    );
}

// ---------------------------------------------------------------------------
// Induced drag is nonnegative
// ---------------------------------------------------------------------------

#[test]
fn induced_drag_is_nonnegative() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    for alpha_deg in [-5.0_f64, -2.0, 0.0, 3.0, 8.0] {
        let state = state_at_alpha(20.0, alpha_deg);
        let wrench = first_stage_wrench(&model, &state, &config());
        // Drag should oppose the section-plane velocity.
        // At nonzero alpha, the velocity has both X and Z components.
        // The drag force projected onto the velocity direction should be negative (opposing).
        let vx = 20.0_f64 * alpha_deg.to_radians().cos();
        let vz = 20.0_f64 * alpha_deg.to_radians().sin();
        let drag_projection = wrench.force_body_n.x * vx + wrench.force_body_n.z * vz;
        assert!(
            drag_projection <= 1e-6,
            "drag should oppose velocity at alpha={alpha_deg} deg, projection={drag_projection:.4}"
        );
    }
}

// ---------------------------------------------------------------------------
// Zero CL -> alpha_i = 0, CDi = 0
// ---------------------------------------------------------------------------

#[test]
fn zero_alpha_zero_induced() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    // With zero alpha and symmetric polar, CL=0 -> alpha_i=0, CDi=0
    let model_with_surface = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let model_without = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        "[]",
    );

    let state = state_at_velocity(20.0);
    let wrench_surface = first_stage_wrench(&model_with_surface, &state, &config());
    let wrench_no_surface = first_stage_wrench(&model_without, &state, &config());

    // At zero geometric alpha with symmetric polar, both should produce the same result
    // (alpha_i = 0, CDi = 0, so finite-wing correction is zero)
    let tol = 1e-10;
    assert!(
        (wrench_surface.force_body_n - wrench_no_surface.force_body_n).norm() < tol,
        "zero alpha should produce identical wrenches with/without surface"
    );
}

// ---------------------------------------------------------------------------
// Higher AR reduces finite-wing correction
// ---------------------------------------------------------------------------

#[test]
fn higher_ar_reduces_correction() {
    let chord: f64 = 0.2;
    let e_factor: f64 = 0.9;

    // For each span, compute the ratio of finite-wing lift to quasi-2D lift.
    // Higher AR should produce a ratio closer to 1.0 (less correction).
    let build_ratio = |span: f64| -> f64 {
        let area_per = chord * span / 2.0;
        // 3D (finite-wing) model
        let model_3d = build_v5_model(
            &format!("[{}]", linear_polar_json("lp", 0.0)),
            "[]",
            &format!(
                "[{},{}]",
                element_json("left", [0.0, -span / 4.0, 0.0], area_per, chord, "lp"),
                element_json("right", [0.0, span / 4.0, 0.0], area_per, chord, "lp"),
            ),
            &format!(
                "[{}]",
                surface_json("wing", &["left", "right"], span, e_factor)
            ),
        );
        // 2D (no surface) model with same elements
        let model_2d = build_v5_model(
            &format!("[{}]", linear_polar_json("lp", 0.0)),
            "[]",
            &format!(
                "[{},{}]",
                element_json("left", [0.0, -span / 4.0, 0.0], area_per, chord, "lp"),
                element_json("right", [0.0, span / 4.0, 0.0], area_per, chord, "lp"),
            ),
            "[]",
        );
        let state = state_at_alpha(25.0, 5.0);
        let lift_3d = -first_stage_wrench(&model_3d, &state, &config())
            .force_body_n
            .z;
        let lift_2d = -first_stage_wrench(&model_2d, &state, &config())
            .force_body_n
            .z;
        lift_3d / lift_2d
    };

    let ratio_low_ar = build_ratio(0.6); // AR = 3
    let ratio_high_ar = build_ratio(2.0); // AR = 10

    assert!(ratio_low_ar > 0.0);
    assert!(ratio_high_ar > 0.0);
    assert!(ratio_low_ar < 1.0, "finite wing should reduce lift");
    assert!(ratio_high_ar < 1.0, "finite wing should reduce lift");

    // Higher AR ratio should be closer to 1.0
    assert!(
        ratio_high_ar > ratio_low_ar,
        "higher AR should be closer to 2D: low_ar_ratio={ratio_low_ar:.4}, high_ar_ratio={ratio_high_ar:.4}"
    );
}

// ---------------------------------------------------------------------------
// Higher e reduces finite-wing correction
// ---------------------------------------------------------------------------

#[test]
fn higher_e_reduces_correction() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    fn build_with_e(e: f64, area_per: f64, chord: f64, span: f64) -> f64 {
        let model = build_v5_model(
            &format!("[{}]", linear_polar_json("lp", 0.0)),
            "[]",
            &format!(
                "[{},{}]",
                element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
                element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
            ),
            &format!("[{}]", surface_json("wing", &["left", "right"], span, e)),
        );
        let wrench = first_stage_wrench(&model, &state_at_alpha(25.0, 5.0), &config());
        -wrench.force_body_n.z
    }

    let lift_low_e = build_with_e(0.7, area_per, chord, span);
    let lift_high_e = build_with_e(0.95, area_per, chord, span);

    assert!(
        lift_high_e > lift_low_e,
        "higher e should produce more lift (closer to 2D): low_e={lift_low_e:.4}, high_e={lift_high_e:.4}"
    );
}

// ---------------------------------------------------------------------------
// Unassigned element remains quasi-2D
// ---------------------------------------------------------------------------

#[test]
fn unassigned_element_remains_quasi2d() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;
    let fuselage_area = 0.3;

    // Model with wing surface + unassigned fuselage element
    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
            element_json("fuselage", [0.3, 0.0, -0.05], fuselage_area, 0.15, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );
    assert_eq!(model.aero_surfaces().len(), 1);
    assert_eq!(model.aero_surfaces()[0].element_indices().len(), 2);

    let state = state_at_alpha(20.0, 5.0);
    let wrench = first_stage_wrench(&model, &state, &config());

    // The fuselage element should contribute forces (it's evaluated quasi-2D)
    assert!(wrench.force_body_n.norm() > 0.0);
    assert!(
        wrench.force_body_n.z < 0.0,
        "total lift should be negative Z (up)"
    );
}

// ---------------------------------------------------------------------------
// Deterministic repeated evaluation
// ---------------------------------------------------------------------------

#[test]
fn deterministic_repeated_evaluation() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let state = state_at_velocity(22.0);
    let w1 = first_stage_wrench(&model, &state, &config());
    let w2 = first_stage_wrench(&model, &state, &config());

    assert_eq!(w1.force_body_n, w2.force_body_n);
    assert_eq!(w1.moment_body_nm, w2.moment_body_nm);
}

// ---------------------------------------------------------------------------
// Full simulation determinism
// ---------------------------------------------------------------------------

#[test]
fn simulation_step_deterministic_with_surfaces() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let cfg = config();
    let initial = state_at_velocity(20.0);

    let mut sim_a = AircraftSimulation::new(model.clone(), cfg, initial).unwrap();
    let mut sim_b = AircraftSimulation::new(model, cfg, initial).unwrap();

    for _ in 0..50 {
        let snap_a = sim_a.step(&sim_core::PilotInput::neutral());
        let snap_b = sim_b.step(&sim_core::PilotInput::neutral());
        assert_eq!(snap_a, snap_b, "simulation steps must be bitwise identical");
    }
}

// ---------------------------------------------------------------------------
// Post-stall non-monotonic polar does not crash
// ---------------------------------------------------------------------------

#[test]
fn post_stall_polar_deterministic() {
    // Non-monotonic CL: rises then drops (stall).
    // Fixed full-bracket bisection provides deterministic root selection,
    // not physical root uniqueness.
    let samples = r#"[
        {"alpha_rad":-0.3,"cl":-0.6,"cd":0.10,"cm":0.0},
        {"alpha_rad":-0.1,"cl":-0.3,"cd":0.04,"cm":0.0},
        {"alpha_rad": 0.0,"cl": 0.0,"cd":0.02,"cm":0.0},
        {"alpha_rad": 0.1,"cl": 0.5,"cd":0.03,"cm":0.0},
        {"alpha_rad": 0.2,"cl": 0.9,"cd":0.06,"cm":0.0},
        {"alpha_rad": 0.3,"cl": 0.7,"cd":0.12,"cm":0.0},
        {"alpha_rad": 0.5,"cl": 0.5,"cd":0.20,"cm":0.0}
    ]"#;
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{{\"id\":\"stall\",\"samples\":{samples}}}]",),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "stall"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "stall"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    // Use nonzero alpha that samples the non-monotonic post-stall region
    let state = state_at_alpha(15.0, 12.0);

    // Run twice — must produce bitwise-identical wrench
    let wrench_a = first_stage_wrench(&model, &state, &config());
    let wrench_b = first_stage_wrench(&model, &state, &config());

    assert!(wrench_a.force_body_n.iter().all(|v| v.is_finite()));
    assert!(wrench_a.moment_body_nm.iter().all(|v| v.is_finite()));
    assert_eq!(
        wrench_a.force_body_n, wrench_b.force_body_n,
        "post-stall bisection must be deterministic (force)"
    );
    assert_eq!(
        wrench_a.moment_body_nm, wrench_b.moment_body_nm,
        "post-stall bisection must be deterministic (moment)"
    );
}

// ---------------------------------------------------------------------------
// No NaN/Inf for valid configurations
// ---------------------------------------------------------------------------

#[test]
fn no_nan_inf_for_valid_configs() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.02)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    for vx in [0.0, 1.0, 10.0, 30.0, 50.0] {
        let state = state_at_velocity(vx);
        let wrench = first_stage_wrench(&model, &state, &config());
        assert!(
            wrench.force_body_n.iter().all(|v| v.is_finite()),
            "non-finite force at vx={vx}"
        );
        assert!(
            wrench.moment_body_nm.iter().all(|v| v.is_finite()),
            "non-finite moment at vx={vx}"
        );
    }
}

// ---------------------------------------------------------------------------
// Assigned element not double-counted
// ---------------------------------------------------------------------------

#[test]
fn assigned_element_not_double_counted() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let model = build_v5_model(
        &format!("[{}]", linear_polar_json("lp", 0.0)),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_per, chord, "lp"),
            element_json("right", [0.0, 0.25, 0.0], area_per, chord, "lp"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );

    let state = state_at_velocity(25.0);
    let wrench = first_stage_wrench(&model, &state, &config());

    // The total lift should be the surface CL * q * S_total, NOT doubled
    let q = 0.5 * 1.225 * 25.0 * 25.0;
    let s_total = 2.0 * area_per;
    let ar = span * span / s_total;
    let e = 0.9;
    let _alpha = 0.0_f64; // zero alpha for this state (identity orientation, velocity along x)
    // Actually at zero alpha, CL=0 so let's check that lift is zero
    let lift = -wrench.force_body_n.z;
    assert!(
        lift.abs() < 1e-6,
        "zero alpha should produce near-zero lift, got {lift:.6e}"
    );

    // Now check at nonzero alpha
    let alpha_geom = 5.0_f64.to_radians();
    let state2 = RigidBodyState {
        position_world_m: Vec3::new(0.0, 0.0, -100.0),
        linear_velocity_world_mps: Vec3::new(25.0 * alpha_geom.cos(), 0.0, 25.0 * alpha_geom.sin()),
        orientation_world_from_body: Orientation::identity(),
        angular_velocity_body_radps: Vec3::zeros(),
    };
    let wrench2 = first_stage_wrench(&model, &state2, &config());
    let lift2 = -wrench2.force_body_n.z;

    let cl_expected = alpha_geom / (1.0 + 1.0 / (PI * ar * e));
    let expected_lift = q * s_total * cl_expected;

    assert!(
        (lift2 - expected_lift).abs() < 0.01 * expected_lift.abs(),
        "lift should match analytic: actual={lift2:.4}, expected={expected_lift:.4}"
    );
}

// ---------------------------------------------------------------------------
// q*S weighting proof: different CL slopes AND different areas
//
// Previous version used the same polar for both members, so CL_left == CL_right
// at every bisection iteration.  When CL is uniform, sum(w_j*CL_j)/sum(w_j) = CL
// regardless of weights, so the test could not distinguish weighted from
// unweighted averaging.
//
// This version assigns each member a polar with a different lift-curve slope
// (a_L = 0.8, a_R = 1.2) AND different areas (S_L = 0.06, S_R = 0.14).
// Both asymmetries make CL_left != CL_right, so the q*S weighting matters:
//
//   Weighted:   a_eff = (S_L*a_L + S_R*a_R) / (S_L + S_R) = 1.08
//   Unweighted: a_eff = (a_L + a_R) / 2 = 1.00
//
// The two predictions differ by ~10 %, far above numerical tolerance.
// ---------------------------------------------------------------------------

#[test]
fn asymmetric_q_s_weighting() {
    let chord: f64 = 0.2;
    let span: f64 = 1.0;
    let e_factor: f64 = 0.9;
    let area_left: f64 = 0.06;
    let area_right: f64 = 0.14;
    let alpha_geom_deg: f64 = 5.0;
    let alpha_geom = alpha_geom_deg.to_radians();
    let vx: f64 = 25.0;

    // Two polars with DIFFERENT lift-curve slopes.
    let slope_left: f64 = 0.8;
    let slope_right: f64 = 1.2;

    let model = build_v5_model(
        &format!(
            "[{},{}]",
            linear_polar_with_slope_json("lp_lo", slope_left, 0.0),
            linear_polar_with_slope_json("lp_hi", slope_right, 0.0),
        ),
        "[]",
        &format!(
            "[{},{}]",
            element_json("left", [0.0, -0.25, 0.0], area_left, chord, "lp_lo"),
            element_json("right", [0.0, 0.25, 0.0], area_right, chord, "lp_hi"),
        ),
        &format!(
            "[{}]",
            surface_json("wing", &["left", "right"], span, e_factor)
        ),
    );

    let state = state_at_alpha(vx, alpha_geom_deg);
    let wrench = first_stage_wrench(&model, &state, &config());

    let q = 0.5 * 1.225 * vx * vx;
    let total_area = area_left + area_right;
    let ar = span * span / total_area;
    let pi_ar_e = PI * ar * e_factor;

    // --- q*S-weighted prediction (correct) ---
    // Both members see the same section speed and geometric alpha (identity
    // orientation, zero angular rate, same body-x position).  Weights are
    // w_j = q * S_j, so the weighted-average slope is:
    let weighted_slope = (area_left * slope_left + area_right * slope_right) / total_area;
    let alpha_eff_weighted = alpha_geom / (1.0 + weighted_slope / pi_ar_e);
    let cl_weighted = weighted_slope * alpha_eff_weighted;
    let expected_weighted_lift = q * total_area * cl_weighted;

    // --- Unweighted prediction (intentionally wrong) ---
    // If the solver averaged CL without q*S weights:
    //   CL_avg = (CL_L + CL_R) / 2 = (a_L + a_R)/2 * alpha_eff
    let unweighted_slope = (slope_left + slope_right) / 2.0;
    let alpha_eff_unweighted = alpha_geom / (1.0 + unweighted_slope / pi_ar_e);
    let cl_unweighted = unweighted_slope * alpha_eff_unweighted;
    let expected_unweighted_lift = q * total_area * cl_unweighted;

    // --- Verify the two predictions are materially different ---
    let lift_gap = (expected_weighted_lift - expected_unweighted_lift).abs();
    assert!(
        lift_gap > 0.1,
        "test must discriminate: gap={lift_gap:.4} N"
    );

    // --- Project actual wrench onto the section lift direction ---
    let alpha = alpha_geom;
    let v_hat = Vec3::new(alpha.cos(), 0.0, alpha.sin());
    let lift_dir = Vec3::y().cross(&v_hat);
    let actual_lift = wrench.force_body_n.dot(&lift_dir);

    // Actual must match the q*S-weighted prediction within 0.5 %
    let tol = 5.0e-3 * expected_weighted_lift.abs();
    assert!(
        (actual_lift - expected_weighted_lift).abs() < tol,
        "q*S-weighted lift: actual={actual_lift:.6}, \
         expected={expected_weighted_lift:.6}, tol={tol:.2e}"
    );

    // Actual must NOT match the unweighted prediction
    let unweighted_gap = (actual_lift - expected_unweighted_lift).abs();
    assert!(
        unweighted_gap > 0.05,
        "actual must diverge from unweighted: gap={unweighted_gap:.4}"
    );
}

// ---------------------------------------------------------------------------
// Reynolds-family member support
// ---------------------------------------------------------------------------

#[test]
fn reynolds_family_member_support() {
    let chord = 0.2;
    let span = 1.0;
    let area_per = chord * span / 2.0;

    let low_samples = linear_samples_json(0.02);
    let high_samples = linear_samples_json(0.015);

    let model = build_v5_model(
        "[]",
        &format!(
            "[{}]",
            reynolds_family_json(
                "re-family",
                &format!(
                    "{},{}",
                    reynolds_node_json(200000.0, &low_samples),
                    reynolds_node_json(400000.0, &high_samples),
                )
            )
        ),
        &format!(
            "[{},{}]",
            element_json_reynolds("left", [0.0, -0.25, 0.0], area_per, chord, "re-family"),
            element_json_reynolds("right", [0.0, 0.25, 0.0], area_per, chord, "re-family"),
        ),
        &format!("[{}]", surface_json("wing", &["left", "right"], span, 0.9)),
    );
    assert_eq!(model.aero_surfaces().len(), 1);

    let state = state_at_velocity(20.0);
    let wrench = first_stage_wrench(&model, &state, &config());
    assert!(wrench.force_body_n.iter().all(|v| v.is_finite()));
    assert!(wrench.moment_body_nm.iter().all(|v| v.is_finite()));
}

// ---------------------------------------------------------------------------
// Reynolds proof: physical section speed determines Re, proven by tight match
//
// Previous version only checked that lift fell between broad low/high bounds,
// which did not prove WHICH Reynolds value the solver actually sampled.
//
// This version:
// 1. Builds a Reynolds family with two nodes whose CL slopes differ by 3×
//    (slope 0.5 at Re=50 000 vs slope 1.5 at Re=500 000).
// 2. Computes the physical section speed from the exact section kinematics.
// 3. Calculates Re_physical = V_section * chord / nu.
// 4. Interpolates the expected CL slope in ln(Re) space at Re_physical.
// 5. Solves the finite-wing equations analytically with that slope.
// 6. Verifies the actual wrench matches with < 0.5 % tolerance.
// 7. Proves that using the wrong Reynolds node would fail badly (~4.8 N gap).
// ---------------------------------------------------------------------------

#[test]
fn reynolds_uses_physical_speed_not_effective_alpha() {
    let chord: f64 = 0.2;
    let span: f64 = 1.0;
    let area_per = chord * span / 2.0;
    let viscosity: f64 = 1.5e-5;
    let vx: f64 = 20.0;
    let alpha_geom_deg: f64 = 5.0;
    let alpha_geom = alpha_geom_deg.to_radians();
    let e_factor: f64 = 0.9;

    // Reynolds family with widely separated nodes and very different slopes.
    let re_lo: f64 = 50_000.0;
    let re_hi: f64 = 500_000.0;
    let slope_lo: f64 = 0.5;
    let slope_hi: f64 = 1.5;

    let model = build_v5_model(
        "[]",
        &format!(
            "[{}]",
            reynolds_family_json(
                "re-fam",
                &format!(
                    "{},{}",
                    reynolds_node_json(re_lo, &linear_samples_with_slope_json(slope_lo, 0.02)),
                    reynolds_node_json(re_hi, &linear_samples_with_slope_json(slope_hi, 0.015)),
                )
            )
        ),
        &format!(
            "[{},{}]",
            element_json_reynolds("left", [0.0, -0.25, 0.0], area_per, chord, "re-fam"),
            element_json_reynolds("right", [0.0, 0.25, 0.0], area_per, chord, "re-fam"),
        ),
        &format!(
            "[{}]",
            surface_json("wing", &["left", "right"], span, e_factor)
        ),
    );

    let state = state_at_alpha(vx, alpha_geom_deg);
    let wrench = first_stage_wrench(&model, &state, &config());

    // --- Compute physical section speed from section kinematics ---
    // With identity orientation and zero angular velocity, both elements see
    // the same section velocity: body-frame velocity = (V*cos(a), 0, V*sin(a)).
    let section_airspeed = vx; // identity orientation, zero angular rate
    let physical_re = section_airspeed * chord / viscosity;

    // --- Canonical ln(Re) interpolation at Re_physical ---
    let frac = (physical_re.ln() - re_lo.ln()) / (re_hi.ln() - re_lo.ln());
    let interpolated_slope = slope_lo + frac * (slope_hi - slope_lo);

    // --- Analytic finite-wing solution at the physical-Re slope ---
    let total_area = 2.0 * area_per;
    let ar = span * span / total_area;
    let pi_ar_e = PI * ar * e_factor;
    let alpha_eff = alpha_geom / (1.0 + interpolated_slope / pi_ar_e);
    let cl_surface = interpolated_slope * alpha_eff;
    let expected_lift = 0.5 * 1.225 * vx * vx * total_area * cl_surface;

    // --- Same calculation using ONLY the low-Re node (deliberately wrong) ---
    let alpha_eff_lo = alpha_geom / (1.0 + slope_lo / pi_ar_e);
    let cl_lo = slope_lo * alpha_eff_lo;
    let lift_at_lo_re = 0.5 * 1.225 * vx * vx * total_area * cl_lo;

    // --- Same calculation using ONLY the high-Re node (deliberately wrong) ---
    let alpha_eff_hi = alpha_geom / (1.0 + slope_hi / pi_ar_e);
    let cl_hi = slope_hi * alpha_eff_hi;
    let lift_at_hi_re = 0.5 * 1.225 * vx * vx * total_area * cl_hi;

    // --- Verify the three predictions are mutually distinct ---
    let gap_lo_vs_phys = (expected_lift - lift_at_lo_re).abs();
    let gap_hi_vs_phys = (expected_lift - lift_at_hi_re).abs();
    assert!(
        gap_lo_vs_phys > 1.0,
        "low-Re prediction must differ from physical-Re: gap={gap_lo_vs_phys:.4} N"
    );
    assert!(
        gap_hi_vs_phys > 0.5,
        "high-Re prediction must differ from physical-Re: gap={gap_hi_vs_phys:.4} N"
    );

    // --- Project actual wrench onto the section lift direction ---
    let v_hat = Vec3::new(alpha_geom.cos(), 0.0, alpha_geom.sin());
    let lift_dir = Vec3::y().cross(&v_hat);
    let actual_lift = wrench.force_body_n.dot(&lift_dir);

    // Actual must match the physical-Re prediction within 0.5 %
    let tol = 5.0e-3 * expected_lift.abs();
    assert!(
        (actual_lift - expected_lift).abs() < tol,
        "physical-Re lift: actual={actual_lift:.6}, \
         expected={expected_lift:.6} (Re_phys={physical_re:.0}, \
         slope={interpolated_slope:.4}), tol={tol:.2e}"
    );

    // Actual must NOT match the low-Re-only prediction
    let wrong_gap = (actual_lift - lift_at_lo_re).abs();
    assert!(
        wrong_gap > 1.0,
        "actual must diverge from low-Re-only: gap={wrong_gap:.4} N"
    );
}
