#![forbid(unsafe_code)]
//! G1E part 1: surface ids and presentation state.

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
    pivot: [f32; 3],
    axis: [f32; 3],
    gain: f32,
}

impl SurfaceHinge {
    #[must_use]
    pub fn new(surface: SurfaceId, pivot: [f32; 3], axis: [f32; 3], gain: f32) -> Option<Self> {
        if !pivot.into_iter().all(f32::is_finite) {
            return None;
        }
        if !axis.into_iter().all(f32::is_finite) {
            return None;
        }
        if !gain.is_finite() {
            return None;
        }
        let norm = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if !(norm.is_finite() && norm > 1.0e-9) {
            return None;
        }
        Some(Self {
            surface,
            pivot,
            axis: [axis[0] / norm, axis[1] / norm, axis[2] / norm],
            gain,
        })
    }

    #[must_use]
    pub const fn surface(&self) -> SurfaceId {
        self.surface
    }

    #[must_use]
    pub const fn pivot_render_body_m(&self) -> [f32; 3] {
        self.pivot
    }

    #[must_use]
    pub const fn axis_render_body(&self) -> [f32; 3] {
        self.axis
    }

    #[must_use]
    pub const fn visual_gain(&self) -> f32 {
        self.gain
    }

    /// Local hinge matrix: `T(pivot) * R(axis, gain*d) * T(-pivot)`.
    #[must_use]
    pub fn local_matrix(&self, d: f32) -> Mat4 {
        if !d.is_finite() {
            return Mat4::identity();
        }
        let angle = self.gain * d;
        if angle == 0.0 {
            return Mat4::identity();
        }
        let rotation = rotation_about_axis(self.axis, angle);
        translate(self.pivot)
            * rotation
            * translate([-self.pivot[0], -self.pivot[1], -self.pivot[2]])
    }
}

/// Per-frame state copied from simulated servo output.
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
    pub fn new(la: f32, ra: f32, el: f32, ru: f32, prop: f32) -> Option<Self> {
        let deflections_rad = [la, ra, el, ru];
        if !deflections_rad.into_iter().all(f32::is_finite) {
            return None;
        }
        if !prop.is_finite() {
            return None;
        }
        Some(Self {
            deflections_rad,
            propeller_angle_rad: prop,
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
    let omc = 1.0 - cos;
    let [x, y, z] = axis;
    Mat4::from_rows([
        [
            cos + x * x * omc,
            x * y * omc - z * sin,
            x * z * omc + y * sin,
            0.0,
        ],
        [
            y * x * omc + z * sin,
            cos + y * y * omc,
            y * z * omc - x * sin,
            0.0,
        ],
        [
            z * x * omc - y * sin,
            z * y * omc + x * sin,
            cos + z * z * omc,
            0.0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ])
}
