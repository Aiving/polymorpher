use std::{
    fmt,
    ops::{Add, Div, Mul},
};

use crate::geometry::{Aabb, DISTANCE_EPSILON, GeometryExt, Point, PointTransformer};

/// Contains 4 points forming a cubic Bézier curve: 2 anchor points at the start
/// and end, and 2 control points between them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cubic {
    pub(crate) points: [Point; 4],
}

impl fmt::Display for Cubic {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let [anchor0, control0, control1, anchor1] = self.points;

        write!(
            f,
            "anchor0: ({:?}, {:?}) control0: ({:?}, {:?}), control1: ({:?}, {:?}), anchor1: ({:?}, {:?})",
            anchor0.x, anchor0.y, control0.x, control0.y, control1.x, control1.y, anchor1.x, anchor1.y
        )
    }
}

impl Cubic {
    pub const fn new(anchor0: Point, control0: Point, control1: Point, anchor1: Point) -> Self {
        Self {
            points: [anchor0, control0, control1, anchor1],
        }
    }

    pub fn from_fn<F: FnMut(usize) -> Point>(f: F) -> Self {
        Self {
            points: std::array::from_fn(f),
        }
    }

    /// Returns a cubic Bézier curve with points swapped (anchor with anchor and
    /// control with control).
    #[must_use]
    pub fn reversed(mut self) -> Self {
        self.points.reverse();

        self
    }

    pub const fn control0(&self) -> Point {
        self.points[1]
    }

    pub const fn control1(&self) -> Point {
        self.points[2]
    }

    pub const fn anchor0(&self) -> Point {
        self.points[0]
    }

    pub const fn anchor1(&self) -> Point {
        self.points[3]
    }

    /// Returns a cubic Bézier curve that forms a straight line.
    pub fn straight_line(start: Point, end: Point) -> Self {
        Self {
            points: [start, start.lerp(end, 1.0 / 3.0), start.lerp(end, 2.0 / 3.0), end],
        }
    }

    /// Returns a cubic Bézier curve that forms a circular arc.
    pub fn circular_arc(center: Point, p0: Point, p1: Point) -> Self {
        let p0d = (p0 - center).normalize();
        let p1d = (p1 - center).normalize();
        let rotated_p0 = p0d.rotate90();
        let rotated_p1 = p1d.rotate90();
        let clockwise = rotated_p0.dot(p1 - center) >= 0.0;
        let cosa = p0d.dot(p1d);

        if cosa > 0.999 {
            return Self::straight_line(p0, p1);
        }

        let k = (p0.x - center.x).hypot(p0.y - center.y) * 4.0 / 3.0 * ((2.0 * (1.0 - cosa)).sqrt() - cosa.mul_add(-cosa, 1.0).sqrt()) / (1.0 - cosa)
            * if clockwise { 1.0 } else { -1.0 };

        Self {
            points: [p0, p0 + (rotated_p0 * k), p1 + rotated_p1 * -k, p1],
        }
    }

    /// Returns a cubic Bézier curve with points transformed using the provided
    /// reference to type that implements [`PointTransformer`] trait.
    #[must_use]
    pub fn transformed<T: PointTransformer>(mut self, f: &T) -> Self {
        self.points = self.points.map(|p| f.transform(p));

        self
    }

    /// Returns an axis-aligned bounding box describing bounds of the curve.
    ///
    /// If `approximate` is `true`, a fast but sometimes inaccurate algorithm is
    /// used. Otherwise, it finds the derivative, which is a quadratic Bézier
    /// curve, and then solves the equation for `t` using the quadratic formula.
    #[allow(clippy::cognitive_complexity)]
    pub fn aabb(&self, approximate: bool) -> Aabb {
        let anchor0 = self.anchor0();

        // A curve might be of zero-length, with both anchors co-lated.
        // Just return the point itself.
        if self.zero_length() {
            return Aabb::new(anchor0, anchor0);
        }

        let anchor1 = self.anchor1();
        let control0 = self.control0();
        let control1 = self.control1();

        let [mut min, mut max] = [anchor0.min(anchor1), anchor0.max(anchor1)];

        if approximate {
            // Approximate bounds use the bounding box of all anchors and controls
            return Aabb::new(min.min(control0.min(control1)), max.max(control0.max(control1)));
        }

        // Find the derivative, which is a quadratic Bezier. Then we can solve for t
        // using the quadratic formula
        let xa = 3f32.mul_add(-control1.x, 3f32.mul_add(control0.x, -anchor0.x)) + anchor1.x;
        let xb = 2f32.mul_add(control1.x, 2f32.mul_add(anchor0.x, -(4.0 * control0.x)));
        let xc = -anchor0.x + control0.x;

        if xa.abs() < DISTANCE_EPSILON {
            // Try Muller's method instead; it can find a single root when a is 0
            if xb != 0.0 {
                let t = 2.0 * xc / (-2.0 * xb);

                if (0.0..=1.0).contains(&t) {
                    let value = self.point_on_curve(t).x;

                    if value < min.x {
                        min.x = value;
                    }

                    if value > max.x {
                        max.x = value;
                    }
                }
            }
        } else {
            let xs = xb.mul_add(xb, -(4.0 * xa * xc));

            if xs >= 0.0 {
                let t1 = (-xb + xs.sqrt()) / (2.0 * xa);

                if (0.0..=1.0).contains(&t1) {
                    let value = self.point_on_curve(t1).x;

                    if value < min.x {
                        min.x = value;
                    }

                    if value > max.x {
                        max.x = value;
                    }
                }

                let t2 = (-xb - xs.sqrt()) / (2.0 * xa);

                if (0.0..=1.0).contains(&t2) {
                    let value = self.point_on_curve(t2).x;

                    if value < min.x {
                        min.x = value;
                    }

                    if value > max.x {
                        max.x = value;
                    }
                }
            }
        }

        // Repeat the above for y coordinate
        let ya = 3f32.mul_add(-control1.y, 3f32.mul_add(control0.y, -anchor0.y)) + anchor1.y;
        let yb = 2f32.mul_add(control1.y, 2f32.mul_add(anchor0.y, -(4.0 * control0.y)));
        let yc = -anchor0.y + control0.y;

        if ya.abs() < DISTANCE_EPSILON {
            if yb != 0.0 {
                let t = 2.0 * yc / (-2.0 * yb);

                if (0.0..=1.0).contains(&t) {
                    let value = self.point_on_curve(t).y;

                    if value < min.y {
                        min.y = value;
                    }

                    if value > max.y {
                        max.y = value;
                    }
                }
            }
        } else {
            let ys = yb.mul_add(yb, -(4.0 * ya * yc));

            if ys >= 0.0 {
                let t1 = (-yb + ys.sqrt()) / (2.0 * ya);

                if (0.0..=1.0).contains(&t1) {
                    let value = self.point_on_curve(t1).y;

                    if value < min.y {
                        min.y = value;
                    }

                    if value > max.y {
                        max.y = value;
                    }
                }

                let t2 = (-yb - ys.sqrt()) / (2.0 * ya);

                if (0.0..=1.0).contains(&t2) {
                    let value = self.point_on_curve(t2).y;

                    if value < min.y {
                        min.y = value;
                    }

                    if value > max.y {
                        max.y = value;
                    }
                }
            }
        }

        Aabb::new(min, max)
    }

    /// Returns `true` if the length between anchor points is zero.
    pub fn zero_length(&self) -> bool {
        let anchor0 = self.anchor0();
        let anchor1 = self.anchor1();

        (anchor0.x - anchor1.x).abs() < DISTANCE_EPSILON && (anchor0.y - anchor1.y).abs() < DISTANCE_EPSILON
    }

    /// Returns a point on the curve for parameter `t`, representing the
    /// proportional distance along the curve between its starting anchor and
    /// ending anchor point.
    pub fn point_on_curve(&self, t: f32) -> Point {
        let u = 1.0 - t;

        self.anchor0() * (u * u * u)
            + (self.control0() * (3.0 * t * u * u)).to_vector()
            + (self.control1() * (3.0 * t * t * u)).to_vector()
            + (self.anchor1() * (t * t * t)).to_vector()
    }

    /// Returns two [`Cubic`]s, created by splitting this curve at the given
    /// distance of `t` between the original starting and ending anchor points.
    pub fn split(self, t: f32) -> (Self, Self) {
        let u = 1.0 - t;
        let point_on_curve = self.point_on_curve(t);

        let [anchor0, control0, control1, anchor1] = self.points;

        (
            Self::new(
                anchor0,
                anchor0 * u + (control0 * t).to_vector(),
                anchor0 * (u * u) + (control0 * (2.0 * u * t)).to_vector() + (control1 * (t * t)).to_vector(),
                point_on_curve,
            ),
            Self::new(
                // TODO: should calculate once and share the result
                point_on_curve,
                control0 * (u * u) + (control1 * (2.0 * u * t)).to_vector() + (anchor1 * (t * t)).to_vector(),
                control1 * u + (anchor1 * t).to_vector(),
                anchor1,
            ),
        )
    }
}

impl Add for Cubic {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            points: [
                self.points[0] + rhs.points[0].to_vector(),
                self.points[1] + rhs.points[1].to_vector(),
                self.points[2] + rhs.points[2].to_vector(),
                self.points[3] + rhs.points[3].to_vector(),
            ],
        }
    }
}

impl Mul<f32> for Cubic {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            points: self.points.map(|point| point * rhs),
        }
    }
}

impl Div<f32> for Cubic {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            points: self.points.map(|point| point / rhs),
        }
    }
}
