use sim_math::Mat3;
use thiserror::Error;

const INERTIA_SYMMETRY_RELATIVE_TOLERANCE: f64 = 1.0e-12;

/// Validated rigid-body mass properties. Inverse inertia is cached at construction.
#[derive(Debug, Clone, PartialEq)]
pub struct RigidBodyParams {
    mass_kg: f64,
    inertia_body_kg_m2: Mat3,
    inverse_inertia_body_per_kg_m2: Mat3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum ParameterError {
    #[error("mass must be finite and greater than zero")]
    InvalidMass,
    #[error("inertia must contain only finite values")]
    NonFiniteInertia,
    #[error("inertia must be symmetric within relative tolerance")]
    NonSymmetricInertia,
    #[error("inertia must be positive definite")]
    NonPositiveDefiniteInertia,
}

impl RigidBodyParams {
    pub fn new(mass_kg: f64, inertia_body_kg_m2: Mat3) -> Result<Self, ParameterError> {
        if !mass_kg.is_finite() || mass_kg <= 0.0 {
            return Err(ParameterError::InvalidMass);
        }
        if !inertia_body_kg_m2.iter().all(|value| value.is_finite()) {
            return Err(ParameterError::NonFiniteInertia);
        }

        let scale = inertia_body_kg_m2.abs().max().max(1.0);
        if (inertia_body_kg_m2 - inertia_body_kg_m2.transpose())
            .abs()
            .max()
            > INERTIA_SYMMETRY_RELATIVE_TOLERANCE * scale
        {
            return Err(ParameterError::NonSymmetricInertia);
        }

        let Some(cholesky) = inertia_body_kg_m2.cholesky() else {
            return Err(ParameterError::NonPositiveDefiniteInertia);
        };
        let inverse_inertia_body_per_kg_m2 = cholesky.inverse();

        Ok(Self {
            mass_kg,
            inertia_body_kg_m2,
            inverse_inertia_body_per_kg_m2,
        })
    }

    #[must_use]
    pub const fn mass_kg(&self) -> f64 {
        self.mass_kg
    }

    #[must_use]
    pub const fn inertia_body_kg_m2(&self) -> &Mat3 {
        &self.inertia_body_kg_m2
    }

    #[must_use]
    pub(crate) const fn inverse_inertia_body_per_kg_m2(&self) -> &Mat3 {
        &self.inverse_inertia_body_per_kg_m2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rejects_invalid_mass_and_inertia() {
        assert_eq!(
            RigidBodyParams::new(0.0, Mat3::identity()),
            Err(ParameterError::InvalidMass)
        );
        let non_symmetric = Mat3::new(1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(
            RigidBodyParams::new(1.0, non_symmetric),
            Err(ParameterError::NonSymmetricInertia)
        );
        let indefinite = Mat3::from_diagonal_element(-1.0);
        assert_eq!(
            RigidBodyParams::new(1.0, indefinite),
            Err(ParameterError::NonPositiveDefiniteInertia)
        );
    }
}
