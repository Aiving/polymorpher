use crate::{Cubic, geometry::Point};

/// A necessary trait for creating paths from polygons or adding polygons to
/// existing paths.
pub trait PathBuilder {
    type Path;

    fn move_to(&mut self, point: Point);
    fn line_to(&mut self, point: Point);
    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point);
    fn close(&mut self);

    fn build(self) -> Self::Path;
}

pub fn add_cubics<T: PathBuilder>(builder: &mut T, repeat_path: bool, close_path: bool, cubics: &[Cubic]) {
    let mut first = true;

    for it in cubics {
        if first {
            builder.move_to(it.anchor0());

            first = false;
        }

        builder.cubic_to(it.control0(), it.control1(), it.anchor1());
    }

    if repeat_path {
        let mut first = true;

        for it in cubics {
            if first {
                builder.line_to(it.anchor0());

                first = false;
            }

            builder.cubic_to(it.control0(), it.control1(), it.anchor1());
        }
    }

    if close_path {
        builder.close();
    }
}

#[cfg(feature = "kurbo")]
impl PathBuilder for kurbo::BezPath {
    type Path = Self;

    fn move_to(&mut self, point: Point) {
        self.move_to((point.x, point.y));
    }

    fn line_to(&mut self, point: Point) {
        self.line_to((point.x, point.y));
    }

    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.curve_to((ctrl1.x, ctrl1.y), (ctrl2.x, ctrl2.y), (to.x, to.y));
    }

    fn close(&mut self) {
        self.close_path();
    }

    fn build(self) -> Self::Path {
        self
    }
}

#[cfg(feature = "lyon")]
impl<T: lyon_path::traits::PathBuilder + lyon_path::traits::Build> PathBuilder for T {
    type Path = T::PathType;

    fn move_to(&mut self, point: Point) {
        self.begin(point, &[]);
    }

    fn line_to(&mut self, point: Point) {
        self.line_to(point, &[]);
    }

    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.cubic_bezier_to(ctrl1, ctrl2, to, &[]);
    }

    fn close(&mut self) {
        self.end(true);
    }

    fn build(self) -> Self::Path {
        Self::build(self)
    }
}

#[cfg(feature = "tiny-skia")]
impl PathBuilder for tiny_skia_path::PathBuilder {
    type Path = Option<tiny_skia_path::Path>;

    fn move_to(&mut self, point: Point) {
        self.move_to(point.x, point.y);
    }

    fn line_to(&mut self, point: Point) {
        self.line_to(point.x, point.y);
    }

    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.cubic_to(ctrl1.x, ctrl1.y, ctrl2.x, ctrl2.y, to.x, to.y);
    }

    fn close(&mut self) {
        self.close();
    }

    fn build(self) -> Self::Path {
        self.finish()
    }
}

#[cfg(feature = "skia")]
impl PathBuilder for skia_safe::PathBuilder {
    type Path = skia_safe::Path;

    fn move_to(&mut self, point: Point) {
        self.move_to((point.x, point.y));
    }

    fn line_to(&mut self, point: Point) {
        self.line_to((point.x, point.y));
    }

    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.cubic_to((ctrl1.x, ctrl1.y), (ctrl2.x, ctrl2.y), (to.x, to.y));
    }

    fn close(&mut self) {
        self.close();
    }

    fn build(mut self) -> Self::Path {
        self.detach()
    }
}

#[cfg(feature = "skia")]
impl PathBuilder for skia_safe::Path {
    type Path = Self;

    fn move_to(&mut self, point: Point) {
        self.move_to((point.x, point.y));
    }

    fn line_to(&mut self, point: Point) {
        self.line_to((point.x, point.y));
    }

    fn cubic_to(&mut self, ctrl1: Point, ctrl2: Point, to: Point) {
        self.cubic_to((ctrl1.x, ctrl1.y), (ctrl2.x, ctrl2.y), (to.x, to.y));
    }

    fn close(&mut self) {
        self.close();
    }

    fn build(self) -> Self {
        self
    }
}
