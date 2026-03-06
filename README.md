# Polymorpher

[![crates.io: polymorpher](https://img.shields.io/crates/v/polymorpher.svg?style=for-the-badge)](https://crates.io/crates/polymorpher)
[![Documentation](https://img.shields.io/docsrs/polymorpher.svg?style=for-the-badge)](https://docs.rs/polymorpher)
[![Build Status](https://img.shields.io/github/actions/workflow/status/Aiving/polymorpher/CI.yml.svg?style=for-the-badge)](https://github.com/Aiving/polymorpher/actions)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-634f7d.svg?style=for-the-badge)](LICENSE)

A library for creating and morphing potentially rounded polygons. Based on the [Kotlin implementation](https://android.googlesource.com/platform/frameworks/support/+/refs/heads/androidx-main/graphics/graphics-shapes) of [Shape Morph from M3 Expressive](https://m3.material.io/styles/shape/shape-morph).

## Features

- `kurbo`: implements `PathBuilder` for `kurbo::BezPath`.
- `skia`: implements `PathBuilder` for `skia_safe::PathBuilder` and `skia_safe::Path`.
- `tiny-skia`: implements `PathBuilder` for `tiny_skia::PathBuilder`.
- `lyon`: implements `PathBuilder` for everything that implements `lyon`'s `PathBuilder`.

## Example with `lyon`

```rust
use lyon_path::Builder as PathBuilder;
use polymorpher::{
    CornerRounding, RoundedPoint,
    RoundedPolygon, geometry::Point
};

let path = RoundedPolygon::from_points(
    &[
        RoundedPoint::new(
            Point::new(0.499, 1.023),
            CornerRounding::smoothed(0.241, 0.778)
        ),
        RoundedPoint::new(
            Point::new(-0.005, 0.792),
            CornerRounding::new(0.208)
        ),
        RoundedPoint::new(
            Point::new(0.073, 0.258),
            CornerRounding::new(0.228)
        ),
        RoundedPoint::new(
            Point::new(0.433, -0.000),
            CornerRounding::new(0.491)
        ),
    ],
    1,
    true,
)
.normalized()
.transformed(|point| point * 128.0)
.as_path::<PathBuilder>(false, true);

// Render it however you want!
```

## MSRV

The Minimum Supported Rust Version is currently 1.85.1.

## License

Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) to be compatible with the AOSP. This project may not be copied, modified, or distributed except according to those terms.
