use core::f32;
use std::fmt;

use crate::{
    Cubic, Feature, RoundedPolygonBuilder,
    geometry::{Aabb, GeometryExt, Point, PointTransformer, Size, Vector},
    path::{PathBuilder, add_cubics},
    polygon_builder::{Circle, Pill, PillStar, Rectangle, Star},
    util::radial_to_cartesian,
};

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct CornerRounding {
    pub radius: f32,
    pub smoothing: f32,
}

impl CornerRounding {
    pub const UNROUNDED: Self = Self { radius: 0.0, smoothing: 0.0 };

    pub const fn new(radius: f32) -> Self {
        Self { radius, smoothing: 0.0 }
    }

    pub const fn smoothed(radius: f32, smoothing: f32) -> Self {
        Self { radius, smoothing }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RoundedPoint {
    pub offset: Point,
    pub rounding: CornerRounding,
}

impl RoundedPoint {
    pub const fn unrounded(offset: Point) -> Self {
        Self {
            offset,
            rounding: CornerRounding::UNROUNDED,
        }
    }

    pub const fn new(offset: Point, rounding: CornerRounding) -> Self {
        Self { offset, rounding }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct RoundedCorner {
    p0: Point,
    p1: Point,
    p2: Point,

    d1: Vector,
    d2: Vector,

    corner_radius: f32,
    smoothing: f32,
    expected_round_cut: f32,

    center: Point,
}

const DISTANCE_EPSILON: f32 = 1e-4f32;

impl RoundedCorner {
    pub fn new(p0: Point, p1: Point, p2: Point, rounding: Option<CornerRounding>) -> Self {
        let mut d1 = Vector::zero();
        let mut d2 = Vector::zero();
        let mut corner_radius = 0.0;
        let mut smoothing = 0.0;
        let mut expected_round_cut = 0.0;

        let v01 = p0 - p1;
        let v21 = p2 - p1;
        let d01 = v01.length();
        let d21 = v21.length();

        if d01 > 0.0 && d21 > 0.0 {
            d1 = v01 / d01;
            d2 = v21 / d21;

            corner_radius = rounding.map_or(0.0, |r| r.radius);
            smoothing = rounding.map_or(0.0, |r| r.smoothing);

            // cosine of angle at p1 is dot product of unit vectors to the other two
            // vertices
            let cos_angle = d1.dot(d2);

            // identity: sin^2 + cos^2 = 1
            // sinAngle gives us the intersection
            let sin_angle = cos_angle.mul_add(-cos_angle, 1.0).sqrt();

            // How much we need to cut, as measured on a side, to get the required radius
            // calculating where the rounding circle hits the edge
            // This uses the identity of tan(A/2) = sinA/(1 + cosA), where tan(A/2) =
            // radius/cut

            if sin_angle > 1e-3 {
                expected_round_cut = corner_radius * (cos_angle + 1.0) / sin_angle;
            }
        }

        Self {
            p0,
            p1,
            p2,
            d1,
            d2,
            corner_radius,
            smoothing,
            expected_round_cut,
            center: Point::zero(),
        }
    }

    pub const fn expected_cut(&self) -> f32 {
        (1.0 + self.smoothing) * self.expected_round_cut
    }

    fn calculate_actual_smoothing_value(&self, allowed_cut: f32) -> f32 {
        if allowed_cut > self.expected_cut() {
            self.smoothing
        } else if allowed_cut > self.expected_round_cut {
            self.smoothing * (allowed_cut - self.expected_round_cut) / (self.expected_cut() - self.expected_round_cut)
        } else {
            0.0
        }
    }

    fn line_intersection(p0: Point, d0: Point, p1: Point, d1: Point) -> Option<Point> {
        let rotated_d1 = d1.rotate90();
        let den = d0.to_vector().dot(rotated_d1.to_vector());

        if den.abs() < DISTANCE_EPSILON {
            return None;
        }

        let num = (p1 - p0).dot(rotated_d1.to_vector());

        // Also check the relative value. This is equivalent to abs(den/num) <
        // DISTANCE_EPSILON, but avoid doing a division
        if den.abs() < DISTANCE_EPSILON * num.abs() {
            None
        } else {
            let k = num / den;

            Some(p0 + (d0 * k).to_vector())
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn compute_flanking_curve(
        actual_round_cut: f32,
        actual_smoothing_values: f32,
        corner: Point,
        side_start: Point,
        circle_segment_intersection: Point,
        other_circle_segment_intersection: Point,
        circle_center: Point,
        actual_r: f32,
    ) -> Cubic {
        // side_start is the anchor, 'anchor' is actual control point
        let side_direction = (side_start - corner).get_direction();
        let curve_start = corner + (side_direction * actual_round_cut * (1.0 + actual_smoothing_values));
        // We use an approximation to cut a part of the circle section proportional to 1
        // - smooth, When smooth = 0, we take the full section, when smooth = 1,
        // we take nothing. TODO: revisit this, it can be problematic as it
        // approaches 180 degrees
        let p = circle_segment_intersection.lerp(
            (circle_segment_intersection + other_circle_segment_intersection.to_vector()) / 2.0,
            actual_smoothing_values,
        );
        // The flanking curve ends on the circle
        let curve_end = circle_center + (p - circle_center).get_direction() * actual_r;
        // The anchor on the circle segment side is in the intersection between the
        // tangent to the circle in the circle/flanking curve boundary and the
        // linear segment.
        let circle_tangent = (curve_end - circle_center).to_point().rotate90();
        let anchor_end = Self::line_intersection(side_start, side_direction.to_point(), curve_end, circle_tangent).unwrap_or(circle_segment_intersection);
        // From what remains, we pick a point for the start anchor.
        // 2/3 seems to come from design tools?
        let anchor_start = (curve_start + (anchor_end * 2.0).to_vector()) / 3.0;

        Cubic::new(curve_start, anchor_start, anchor_end, curve_end)
    }

    pub fn get_cubics(&mut self, allowed_cut0: f32, allowed_cut1: f32) -> Vec<Cubic> {
        // We use the minimum of both cuts to determine the radius, but if there is more
        // space in one side we can use it for smoothing.
        let allowed_cut = allowed_cut0.min(allowed_cut1);

        // Nothing to do, just use lines, or a point
        if self.expected_round_cut < DISTANCE_EPSILON || allowed_cut < DISTANCE_EPSILON || self.corner_radius < DISTANCE_EPSILON {
            self.center = self.p1;

            return vec![Cubic::straight_line(self.p1, self.p1)];
        }

        // How much of the cut is required for the rounding part.
        let actual_round_cut = allowed_cut.min(self.expected_round_cut);
        // We have two smoothing values, one for each side of the vertex
        // Space is used for rounding values first. If there is space left over, then we
        // apply smoothing, if it was requested
        let actual_smoothing0 = self.calculate_actual_smoothing_value(allowed_cut0);
        let actual_smoothing1 = self.calculate_actual_smoothing_value(allowed_cut1);
        // Scale the radius if needed
        let actual_r = self.corner_radius * actual_round_cut / self.expected_round_cut;
        // Distance from the corner (p1) to the center
        #[allow(clippy::imprecise_flops)]
        let center_distance = (actual_r * actual_r + actual_round_cut * actual_round_cut).sqrt();

        // Center of the arc we will use for rounding
        self.center = self.p1 + ((self.d1 + self.d2) / 2.0).get_direction() * center_distance;

        let circle_intersection0 = self.p1 + (self.d1 * actual_round_cut);
        let circle_intersection2 = self.p1 + (self.d2 * actual_round_cut);

        let flanking0 = Self::compute_flanking_curve(
            actual_round_cut,
            actual_smoothing0,
            self.p1,
            self.p0,
            circle_intersection0,
            circle_intersection2,
            self.center,
            actual_r,
        );

        let flanking2 = Self::compute_flanking_curve(
            actual_round_cut,
            actual_smoothing1,
            self.p1,
            self.p2,
            circle_intersection2,
            circle_intersection0,
            self.center,
            actual_r,
        )
        .reversed();

        let flanking1 = Cubic::circular_arc(self.center, flanking0.anchor1(), flanking2.anchor0());

        vec![flanking0, flanking1, flanking2]
    }
}

/// [`RoundedPolygon`] allows simple construction of polygonal shapes with
/// optional rounding at the vertices.
///
/// Polygons can be constructed with either the number of vertices desired or an
/// ordered list of vertices.
#[derive(Debug, Clone, PartialEq)]
pub struct RoundedPolygon {
    pub features: Vec<Feature>,
    pub center: Point,
    /// A flattened version of the [`Feature`]s.
    pub cubics: Vec<Cubic>,
}

impl fmt::Display for RoundedPolygon {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "[RoundedPolygon. Cubics = {} || Features = {} || Center = ({}, {})]",
            self.cubics.iter().map(ToString::to_string).collect::<Vec<_>>().join(", "),
            self.features.iter().map(ToString::to_string).collect::<Vec<_>>().join(", "),
            self.center.x,
            self.center.y
        )
    }
}

impl RoundedPolygon {
    #[track_caller]
    pub fn new(features: Vec<Feature>, center: Point) -> Self {
        let mut cubics = Vec::new();

        // The first/last mechanism here ensures that the final anchor point in the
        // shape exactly matches the first anchor point. There can be rendering
        // artifacts introduced by those points being slightly off, even by much
        // less than a pixel
        let mut first_cubic: Option<Cubic> = None;
        let mut last_cubic: Option<Cubic> = None;
        let mut first_feature_split_start: Option<Vec<Cubic>> = None;
        let mut first_feature_split_end: Option<Vec<Cubic>> = None;

        if !features.is_empty() && features[0].cubics.len() == 3 {
            let center_cubic = features[0].cubics[1];
            let (start, end) = center_cubic.split(0.5);

            first_feature_split_start.replace(vec![features[0].cubics[0], start]);
            first_feature_split_end.replace(vec![end, features[0].cubics[2]]);
        }

        // iterating one past the features list size allows us to insert the initial
        // split cubic if it exists
        for i in 0..=features.len() {
            let feature_cubics = if i == 0 && first_feature_split_end.is_some() {
                let Some(cubics) = &first_feature_split_end else { unreachable!() };

                cubics
            } else if i == features.len() {
                if let Some(cubics) = &first_feature_split_start {
                    cubics
                } else {
                    break;
                }
            } else {
                &features[i].cubics
            };

            for &cubic in feature_cubics {
                // Skip zero-length curves; they add nothing and can trigger rendering artifacts
                if !cubic.zero_length() {
                    if let Some(last_cubic) = last_cubic {
                        cubics.push(last_cubic);
                    }

                    last_cubic.replace(cubic);

                    if first_cubic.is_none() {
                        first_cubic.replace(cubic);
                    }
                } else if let Some(last_cubic) = last_cubic.as_mut() {
                    // Dropping several zero-ish length curves in a row can lead to
                    // enough discontinuity to throw an exception later, even though the
                    // distances are quite small. Account for that by making the last
                    // cubic use the latest anchor point, always.
                    last_cubic.points[3] = cubic.anchor1();
                }
            }
        }

        if let (Some(last_cubic), Some(first_cubic)) = (last_cubic, first_cubic) {
            cubics.push(Cubic::new(
                last_cubic.anchor0(),
                last_cubic.control0(),
                last_cubic.control1(),
                first_cubic.anchor0(),
            ));
        } else {
            // Empty / 0-sized polygon.
            cubics.push(Cubic::new(center, center, center, center));
        }

        Self { features, center, cubics }
    }

    pub fn from_features(features: Vec<Feature>, center: Option<Point>) -> Self {
        let center = center.unwrap_or_else(|| Point::splat(f32::NAN));
        let vertices = features
            .iter()
            .flat_map(|feature| feature.cubics.iter().map(Cubic::anchor0))
            .collect::<Vec<_>>();

        let center = if center.x.is_nan() || center.y.is_nan() {
            let center_from_verts = center_from_vertices(&vertices);

            Point::new(
                if center.x.is_nan() { center_from_verts.x } else { center.x },
                if center.y.is_nan() { center_from_verts.y } else { center.y },
            )
        } else {
            center
        };

        Self::new(features, center)
    }

    fn builder<D>(data: D) -> RoundedPolygonBuilder<D> {
        RoundedPolygonBuilder {
            data,
            center: Point::zero(),
            rounding: CornerRounding::UNROUNDED,
            per_vertex_rounding: Vec::new(),
        }
    }

    /// Returns a [`RoundedPolygonBuilder`] for creating a circle.
    pub fn circle() -> RoundedPolygonBuilder<Circle> {
        Self::builder(Circle { vertices: 8, radius: 1.0 })
    }

    /// Returns a [`RoundedPolygonBuilder`] for creating a rectangle.
    pub fn rectangle() -> RoundedPolygonBuilder<Rectangle> {
        Self::builder(Rectangle { size: Size::splat(2.0) })
    }

    /// Returns a [`RoundedPolygonBuilder`] for creating a star.
    pub fn star(vertices_per_radius: usize) -> RoundedPolygonBuilder<Star> {
        Self::builder(Star {
            vertices_per_radius,
            radius: 1.0,
            inner_radius: 0.5,
            inner_rounding: None,
        })
    }

    /// Returns a [`RoundedPolygonBuilder`] for creating a pill.
    pub fn pill() -> RoundedPolygonBuilder<Pill> {
        Self::builder(Pill {
            size: Size::new(2.0, 1.0),
            smoothing: 0.0,
        })
    }

    /// Returns a [`RoundedPolygonBuilder`] for creating a pill star.
    pub fn pill_star() -> RoundedPolygonBuilder<PillStar> {
        Self::builder(PillStar {
            size: Size::new(2.0, 1.0),
            vertices_per_radius: 8,
            inner_radius_ratio: 0.5,
            inner_rounding: None,
            vertex_spacing: 0.5,
            start_location: 0.0,
        })
    }

    /// Creates a rounded polygon from a set of points that are connected to
    /// each other and optionally rounded. The `repeats` argument specifies the
    /// number of vertices generated by each point. If `mirroring` is `true`, a
    /// mirrored version is also added for each point.
    pub fn from_points(points: &[RoundedPoint], repeats: usize, mirroring: bool) -> Self {
        custom_polygon(points, repeats, None, mirroring)
    }

    /// Creates a rounded polygon from a set of points that are connected to
    /// each other at center and optionally rounded. The `repeats` argument
    /// specifies the number of vertices generated by each point. If
    /// `mirroring` is `true`, a mirrored version is also added for each
    /// point.
    pub fn from_points_at(points: &[RoundedPoint], repeats: usize, center: Point, mirroring: bool) -> Self {
        custom_polygon(points, repeats, Some(center), mirroring)
    }

    pub fn from_vertices_count(vertices: usize, radius: f32, rounding: Option<CornerRounding>, per_vertex_rounding: &[CornerRounding]) -> Self {
        Self::from_vertices_count_at(vertices, radius, Point::zero(), rounding, per_vertex_rounding)
    }

    pub fn from_vertices_count_at(
        vertices: usize,
        radius: f32,
        center: Point,
        rounding: Option<CornerRounding>,
        per_vertex_rounding: &[CornerRounding],
    ) -> Self {
        Self::from_vertices(
            &vertices_from_count(vertices, radius, center),
            rounding.unwrap_or(CornerRounding::UNROUNDED),
            per_vertex_rounding,
            center,
        )
    }

    /// # Panics
    ///
    /// May panic if:
    /// - The polygon contains fewer than 3 vertices
    /// - The `per_vertex_rounding` is not empty, but its size does not
    ///   correspond to the number of vertices in the polygon
    pub fn from_vertices(vertices: &[Point], rounding: CornerRounding, per_vertex_rounding: &[CornerRounding], center: Point) -> Self {
        assert!(vertices.len() >= 3, "Polygons must have at least 3 vertices");
        assert!(
            per_vertex_rounding.is_empty() || per_vertex_rounding.len() == vertices.len(),
            "per_vertex_rounding array should be either empty or the same size as the number of vertices"
        );

        let mut corners = <Vec<Vec<Cubic>>>::new();
        let n = vertices.len();
        let mut rounded_corners = <Vec<RoundedCorner>>::new();

        for i in 0..n {
            rounded_corners.push(RoundedCorner::new(
                vertices[(i + n - 1) % n],
                vertices[i],
                vertices[(i + 1) % n],
                Some(per_vertex_rounding.get(i).copied().unwrap_or(rounding)),
            ));
        }

        let cut_adjusts = (0..n)
            .map(|ix| {
                let expected_round_cut = rounded_corners[ix].expected_round_cut + rounded_corners[(ix + 1) % n].expected_round_cut;
                let expected_cut = rounded_corners[ix].expected_cut() + rounded_corners[(ix + 1) % n].expected_cut();
                let vtx = vertices[ix];
                let next_vtx = vertices[(ix + 1) % n];
                let side_size = (vtx.x - next_vtx.x).hypot(vtx.y - next_vtx.y);

                // Check expected_round_cut first, and ensure we fulfill rounding needs first
                // for both corners before using space for smoothing
                if expected_round_cut > side_size {
                    // Not enough room for fully rounding, see how much we can actually do.
                    (side_size / expected_round_cut, 0.0)
                } else if expected_cut > side_size {
                    // We can do full rounding, but not full smoothing.
                    (1.0, (side_size - expected_round_cut) / (expected_cut - expected_round_cut))
                } else {
                    // There is enough room for rounding & smoothing.
                    (1.0, 1.0)
                }
            })
            .collect::<Vec<_>>();

        // Create and store list of beziers for each potentially rounded corner
        for i in 0..n {
            // allowed_cuts[0] is for the side from the previous corner to this one,
            // allowed_cuts[1] is for the side from this corner to the next one.
            let mut allowed_cuts = [0.0; 2];

            for delta in 0..=1 {
                let (round_cut_ratio, cut_ratio) = cut_adjusts[(i + n - 1 + delta) % n];

                allowed_cuts[delta] = rounded_corners[i].expected_round_cut.mul_add(
                    round_cut_ratio,
                    (rounded_corners[i].expected_cut() - rounded_corners[i].expected_round_cut) * cut_ratio,
                );
            }

            corners.push(rounded_corners[i].get_cubics(allowed_cuts[0], allowed_cuts[1]));
        }

        let mut temp_features = <Vec<Feature>>::new();

        for i in 0..n {
            // Note that these indices are for pairs of values (points), they need to be
            // doubled to access the xy values in the vertices float array
            let prev_vtx_index = (i + n - 1) % n;
            let next_vtx_index = (i + 1) % n;
            let convex = vertices[prev_vtx_index].is_convex(vertices[i], vertices[next_vtx_index]);

            temp_features.push(Feature::corner(corners[i].clone(), convex));
            temp_features.push(Feature::edge(vec![Cubic::straight_line(
                corners[i].last().unwrap().anchor1(),
                corners[(i + 1) % n].first().unwrap().anchor0(),
            )]));
        }

        let center = if center.x == f32::MIN || center.y == f32::MIN {
            center_from_vertices(vertices)
        } else {
            center
        };

        Self::new(temp_features, center)
    }

    /// Returns a [`RoundedPolygon`] with features transformed using the
    /// provided reference to type that implements [`PointTransformer`] trait.
    #[must_use]
    #[allow(clippy::needless_pass_by_value)]
    pub fn transformed<T: PointTransformer>(self, f: T) -> Self {
        let center = f.transform(self.center);

        Self::new(self.features.into_iter().map(|feature| feature.transformed(&f)).collect(), center)
    }

    /// Returns an axis-aligned bounding box describing bounds of the polygon.
    ///
    /// If `approximate` is `true`, a fast but sometimes inaccurate algorithm is
    /// used to calculate AABB of cubics. Otherwise, it finds the derivative,
    /// which is a quadratic Bézier curve, and then solves the equation for `t`
    /// using the quadratic formula.
    pub fn aabb(&self, approximate: bool) -> Aabb {
        let mut aabb = Aabb::new(Point::splat(f32::MAX), Point::splat(f32::MIN));

        for cubic in &self.cubics {
            let cubic_aabb = cubic.aabb(approximate);

            aabb = Aabb {
                min: aabb.min.min(cubic_aabb.min),
                max: aabb.max.max(cubic_aabb.max),
            };
        }

        aabb
    }

    /// Moves and resizes [`RoundedPolygon`], so it's completely inside the 0x0
    /// -> 1x1 square, centered if there extra space in one direction.
    #[must_use]
    pub fn normalized(self) -> Self {
        let bounds = self.aabb(true);
        let size = bounds.size();
        let max_side = size.width.max(size.height);
        let offset = ((Point::splat(max_side) - size) / 2.0 - bounds.min).to_point();

        self.transformed(|point| (point + offset.to_vector()) / max_side)
    }

    /// Returns a path with a drawn rounded polygon. Path is created using the
    /// provided `T`, which should implement `PathBuilder` and `Default` traits.
    pub fn as_path<T: PathBuilder + Default>(&self, repeat_path: bool, close_path: bool) -> T::Path {
        let mut path = T::default();

        self.add_to(&mut path, repeat_path, close_path);

        path.build()
    }

    /// Adds a rounded polygon to the `builder`.
    pub fn add_to<T: PathBuilder>(&self, builder: &mut T, repeat_path: bool, close_path: bool) {
        add_cubics(builder, repeat_path, close_path, &self.cubics);
    }
}

fn center_from_vertices(vertices: &[Point]) -> Point {
    let mut cumulative = Point::zero();

    for vertice in vertices {
        cumulative += vertice.to_vector();
    }

    cumulative / vertices.len() as f32
}

#[allow(clippy::manual_is_multiple_of)] // For MSRV compability
fn custom_polygon(points: &[RoundedPoint], repeats: usize, center: Option<Point>, mirroring: bool) -> RoundedPolygon {
    let center = center.unwrap_or(Point::new(0.5, 0.5));
    let mut actual_points = Vec::new();

    if mirroring {
        let angles = points
            .iter()
            .map(|it| {
                let offset = it.offset - center;

                offset.y.atan2(offset.x).to_degrees()
            })
            .collect::<Vec<_>>();

        let distances = points.iter().map(|it| (it.offset - center).length()).collect::<Vec<_>>();
        let actual_repeats = repeats * 2;
        let section_angle = 360.0 / actual_repeats as f32;

        for iteration in 0..actual_repeats {
            for index in 0..points.len() {
                let i = if iteration % 2 == 0 { index } else { points.len() - index - 1 };

                if i > 0 || iteration % 2 == 0 {
                    let angle = (section_angle * iteration as f32
                        + if iteration % 2 == 0 {
                            angles[i]
                        } else {
                            2.0f32.mul_add(angles[0], section_angle - angles[i])
                        })
                        / 360.0
                        * 2.0
                        * f32::consts::PI;

                    let final_point = Point::new(angle.cos(), angle.sin()) * distances[i] + center.to_vector();

                    actual_points.push(RoundedPoint {
                        offset: final_point,
                        rounding: points[i].rounding,
                    });
                }
            }
        }
    } else {
        let size = points.len();

        for it in 0..size * repeats {
            let point = points[it % size].offset.rotated((it / size) as f32 * 360.0 / repeats as f32, center);

            actual_points.push(RoundedPoint {
                offset: point,
                rounding: points[it % size].rounding,
            });
        }
    }

    RoundedPolygon::from_vertices(
        &actual_points.iter().map(|p| p.offset).collect::<Vec<_>>(),
        CornerRounding::UNROUNDED,
        &actual_points.iter().map(|p| p.rounding).collect::<Vec<_>>(),
        center,
    )
}

fn vertices_from_count(count: usize, radius: f32, center: Point) -> Vec<Point> {
    (0..count)
        .map(|i| center + radial_to_cartesian(radius, f32::consts::PI / count as f32 * 2.0 * i as f32))
        .collect()
}
