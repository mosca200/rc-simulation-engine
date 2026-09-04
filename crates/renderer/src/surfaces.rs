#![forbid(unsafe_code)]
//! G1E: presentation-level articulated control surfaces.
//!
//! The renderer never depends on simulation crates. This module operates on
//! render-body-frame hinge geometry plus deflection angles the `app` layer
//! copies from actual simulated `ControlSurfacePositions` (servo state).

use crate::Mat4;

/// Named visual subparts. Indices are stable and deterministic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SurfaceId {
    LeftAileron,
    RightAileron,
    Elevator,
    Rudder,
    /// Reserved slot for a future propeller visual. Unused by G1E draws.
    Propeller,
}

impl SurfaceId {
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::LeftAileron => 0,
            Self::RightAileron => 1,
            Self::Elevator => 2,
            Self::Rudder => 3,
            Self::Propeller => 4,
        }
    }

    #[must_use]
    pub const fn control_surfaces() -> [Self; 4] {
        [
            Self::LeftAileron,
            Self::RightAileron,
            Self::Elevator,
            Self::Rudder,
        ]
    }
}

/// Control-surface slots in the per-frame presentation state.
pub const CONTROL_SURFACE_COUNT: usize = 4;
/// Total visual slots including the reserved propeller node.
pub const VISUAL_SLOT_COUNT: usize = 5;

/// Presentation-only hinge for one named visual subpart.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceHinge {
    surface: SurfaceId,
    pivot_render_body_m: [f32; 3],
    axis_render_body: [f32; 3],
    visual_gain: f32,
}

impl SurfaceHinge {
    /// Finite-checked constructor with deterministic axis normalization.
    #[must_use]
    pub fn new(
        surface: SurfaceId,
        pivot_render_body_m: [f32; 3],
        axis_render_body: [f32; 3],
        visual_gain: f32,
    ) -> Option<Self> {
        if !pivot_render_body_m.into_iter().all(f32::is_finite) {
            return None;
        }
        if !axis_render_body.into_iter().all(f32::is_finite) {
            return None;
        }
        if !visual_gain.is_finite() {
            return None;
        }
        let norm = (axis_render_body[0] * axis_render_body[0]
            + axis_render_body[1] * axis_render_body[1]
            + axis_render_body[2] * axis_render_body[2])
            .sqrt();
        if !(norm.is_finite() && norm > 1.0e-9) {
            return None;
        }
        Some(Self {
            surface,
            pivot_render_body_m,
            axis_render_body: [
                axis_render_body[0] / norm,
                axis_render_body[1] / norm,
                axis_render_body[2] / norm,
            ],
            visual_gain,
        })
    }

    #[must_use]
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    #[must_use]
    pub const fn pivot_render_body_m(&self) -> [f32; 3] {
        self.pivot_render_body_m
    }

    #[must_use]
    pub const fn axis_render_body(&self) -> [f32; 3] {
        self.axis_render_body
    }

    #[must_use]
    pub const fn visual_gain(&self) -> f32 {
        self.visual_gain
    }

    /// Local hinge matrix: `T(pivot) * R(axis, gain*deflection) * T(-pivot)`.
    /// Zero deflection is exactly identity (neutral transform).
    #[must_use]
    pub fn local_matrix(&self, surface_deflection_rad: f32) -> Mat4 {
        if !surface_deflection_rad.is_finite() {
            return Mat4::identity();
        }
        let angle = self.visual_gain * surface_deflection_rad;
        if angle == 0.0 {
            return Mat4::identity();
        }
        let rotation = rotation_about_axis(self.axis_render_body, angle);
        translate(self.pivot_render_body_m)
            * rotation
            * translate([
                -self.pivot_render_body_m[0],
                -self.pivot_render_body_m[1],
                -self.pivot_render_body_m[2],
            ])
    }
}

/// Per-frame state copied from actual simulated servo output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlSurfacePresentation {
    deflections_rad: [f32; CONTROL_SURFACE_COUNT],
    propeller_angle_rad: f32,
}

impl ControlSurfacePresentation {
    #[must_use]
    pub const fn neutral() -> Self {
        Self {
            deflections_rad: [0.0; CONTROL_SURFACE_COUNT],
            propeller_angle_rad: 0.0,
        }
    }

    #[must_use]
    pub fn new(
        left_aileron_rad: f32,
        right_aileron_rad: f32,
        elevator_rad: f32,
        rudder_rad: f32,
        propeller_angle_rad: f32,
    ) -> Option<Self> {
        let deflections_rad = [
            left_aileron_rad,
            right_aileron_rad,
            elevator_rad,
            rudder_rad,
        ];
        if !deflections_rad.into_iter().all(f32::is_finite) {
            return None;
        }
        if !propeller_angle_rad.is_finite() {
            return None;
        }
        Some(Self {
            deflections_rad,
            propeller_angle_rad,
        })
    }

    #[must_use]
    pub fn deflection(&self, surface: SurfaceId) -> f32 {
        match surface {
            SurfaceId::LeftAileron => self.deflections_rad[0],
            SurfaceId::RightAileron => self.deflections_rad[1],
            SurfaceId::Elevator => self.deflections_rad[2],
            SurfaceId::Rudder => self.deflections_rad[3],
            SurfaceId::Propeller => self.propeller_angle_rad,
        }
    }
}

impl Default for ControlSurfacePresentation {
    fn default() -> Self {
        Self::neutral()
    }
}

/// Fixed binding table: one hinge per visual slot, `None` stays rigid.
#[derive(Debug, Clone, PartialEq)]
pub struct SurfaceBindingTable {
    hinges: [Option<SurfaceHinge>; VISUAL_SLOT_COUNT],
}

impl SurfaceBindingTable {
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            hinges: [None, None, None, None, None],
        }
    }

    #[must_use]
    pub fn with_hinge(mut self, hinge: SurfaceHinge) -> Self {
        self.hinges[hinge.surface().index()] = Some(hinge);
        self
    }

    #[must_use]
    pub const fn hinge(&self, surface: SurfaceId) -> Option<&SurfaceHinge> {
        self.hinges[surface.index()].as_ref()
    }

    #[must_use]
    pub fn local_matrix(
        &self,
        surface: SurfaceId,
        presentation: &ControlSurfacePresentation,
    ) -> Mat4 {
        match self.hinge(surface) {
            Some(hinge) => hinge.local_matrix(presentation.deflection(surface)),
            None => Mat4::identity(),
        }
    }

    /// Composed matrix: `root * local`; root stays exactly the pose matrix.
    #[must_use]
    pub fn composed_matrix(
        &self,
        root: &Mat4,
        surface: SurfaceId,
        presentation: &ControlSurfacePresentation,
    ) -> Mat4 {
        *root * self.local_matrix(surface, presentation)
    }
}

impl Default for SurfaceBindingTable {
    fn default() -> Self {
        Self::empty()
    }
}

fn translate(offset: [f32; 3]) -> Mat4 {
    Mat4::from_rows([
        [1.0, 0.0, 0.0, offset[0]],
        [0.0, 1.0, 0.0, offset[1]],
        [0.0, 0.0, 1.0, offset[2]],
        [0.0, 0.0, 0.0, 1.0],
    ])
}

fn rotation_about_axis(axis: [f32; 3], angle_rad: f32) -> Mat4 {
    let (sin, cos) = angle_rad.sin_cos();
    let one_minus_cos = 1.0 - cos;
    let [x, y, z] = axis;
    Mat4::from_rows([
        [
            cos + x * x * one_minus_cos,
            x * y * one_minus_cos - z * sin,
            x * z * one_minus_cos + y * sin,
            0.0,
        ],
        [
            y * x * one_minus_cos + z * sin,
            cos + y * y * one_minus_cos,
            y * z * one_minus_cos - x * sin,
            0.0,
        ],
        [
            z * x * one_minus_cos - y * sin,
            z * y * one_minus_cos + x * sin,
            cos + z * z * one_minus_cos,
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ])
}
