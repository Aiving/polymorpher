use std::fmt;

use crate::{cubic::Cubic, geometry::PointTransformer};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum FeatureType {
    Edge,
    Corner { convex: bool },
}

/// While a polygon's shape can be drawn solely using a list of [`Cubic`]
/// objects representing its raw curves and lines, features add an extra layer
/// of context to groups of cubics.
///
/// Features group cubics into (straight) edges, convex corners, or concave
/// corners. For example, rounding a rectangle adds many cubics around its
/// edges, but the rectangle's overall number of corners remains the same.
#[derive(Debug, Clone, PartialEq)]
pub struct Feature {
    pub ty: FeatureType,
    pub cubics: Vec<Cubic>,
}

impl fmt::Display for Feature {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.ty {
            FeatureType::Edge => write!(f, "Edge"),
            FeatureType::Corner { convex } => write!(
                f,
                "Corner: cubics={} convex={convex}",
                self.cubics.iter().map(|c| format!("[{c}]")).collect::<Vec<_>>().join(", ")
            ),
        }
    }
}

impl Feature {
    pub const fn edge(cubics: Vec<Cubic>) -> Self {
        Self { cubics, ty: FeatureType::Edge }
    }

    pub const fn corner(cubics: Vec<Cubic>, convex: bool) -> Self {
        Self {
            cubics,
            ty: FeatureType::Corner { convex },
        }
    }

    /// Returns a [`Feature`] with cubics transformed using the provided
    /// reference to type that implements [`PointTransformer`] trait.
    #[must_use]
    pub fn transformed<T: PointTransformer>(self, f: &T) -> Self {
        match self.ty {
            FeatureType::Edge => Self {
                cubics: self.cubics.into_iter().map(|cubic| cubic.transformed(f)).collect(),
                ty: FeatureType::Edge,
            },
            FeatureType::Corner { convex } => Self {
                cubics: self.cubics.into_iter().map(|cubic| cubic.transformed(f)).collect(),
                ty: FeatureType::Corner { convex },
            },
        }
    }

    /// Returns `true` if the feature type is corner.
    pub const fn is_corner(&self) -> bool {
        matches!(self.ty, FeatureType::Corner { .. })
    }

    /// Returns the result of calling `func` if the type of feature is an angle
    /// and passed to `func`, otherwise returning `false`.
    pub fn is_corner_and<F: FnOnce(bool) -> bool>(&self, func: F) -> bool {
        if let FeatureType::Corner { convex } = &self.ty {
            func(*convex)
        } else {
            false
        }
    }
}
